package api

import (
	"archive/zip"
	"bytes"
	"context"
	"database/sql"
	"encoding/binary"
	"encoding/json"
	"image"
	"image/color"
	"image/png"
	"math"
	"mime/multipart"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strconv"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// ---------------------------------------------------------------------------
// Minimal GeoTIFF writer (uncompressed RGBA, one strip) for tests.
// ---------------------------------------------------------------------------

type testGeo struct {
	epsg       int
	geographic bool
	tie        [6]float64
	scale      [3]float64
	pixelPoint bool
	noData     string
}

func buildGeoTIFF(t *testing.T, img *image.NRGBA, g testGeo) []byte {
	t.Helper()
	w, h := img.Bounds().Dx(), img.Bounds().Dy()
	bo := binary.LittleEndian
	type ent struct {
		tag, typ uint16
		count    uint32
		data     []byte
	}
	u16 := func(v ...uint16) []byte {
		b := make([]byte, 2*len(v))
		for i, x := range v {
			bo.PutUint16(b[2*i:], x)
		}
		return b
	}
	u32 := func(v ...uint32) []byte {
		b := make([]byte, 4*len(v))
		for i, x := range v {
			bo.PutUint32(b[4*i:], x)
		}
		return b
	}
	f64 := func(v ...float64) []byte {
		b := make([]byte, 8*len(v))
		for i, x := range v {
			bo.PutUint64(b[8*i:], math.Float64bits(x))
		}
		return b
	}
	pix := img.Pix
	keys := []uint16{1, 1, 0, 0}
	add := func(k, v uint16) { keys = append(keys, k, 0, 1, v); keys[3]++ }
	if g.geographic {
		add(geoKeyModelType, 2)
		add(geoKeyRasterType, map[bool]uint16{false: 1, true: 2}[g.pixelPoint])
		add(geoKeyGeographicType, uint16(g.epsg))
	} else {
		add(geoKeyModelType, 1)
		add(geoKeyRasterType, map[bool]uint16{false: 1, true: 2}[g.pixelPoint])
		add(geoKeyProjectedCSType, uint16(g.epsg))
	}
	ents := []ent{
		{256, 4, 1, u32(uint32(w))},
		{257, 4, 1, u32(uint32(h))},
		{258, 3, 4, u16(8, 8, 8, 8)},
		{259, 3, 1, u16(1, 0)},
		{262, 3, 1, u16(2, 0)},
		{273, 4, 1, nil}, // strip offset, patched below
		{277, 3, 1, u16(4, 0)},
		{278, 4, 1, u32(uint32(h))},
		{279, 4, 1, u32(uint32(len(pix)))},
		{284, 3, 1, u16(1, 0)},
		{338, 3, 1, u16(2, 0)}, // unassociated alpha
		{geoTagPixelScale, 12, 3, f64(g.scale[:]...)},
		{geoTagTiepoint, 12, 6, f64(g.tie[:]...)},
		{geoTagKeyDirectory, 3, uint32(len(keys)), u16(keys...)},
	}
	if g.noData != "" {
		s := g.noData + "\x00"
		ents = append(ents, ent{gdalTagNoData, 2, uint32(len(s)), []byte(s)})
	}
	// Layout: header(8) | IFD | out-of-line data | pixels
	ifdSize := 2 + 12*len(ents) + 4
	dataOff := 8 + ifdSize
	var extra bytes.Buffer
	offsets := make([]uint32, len(ents))
	for i, e := range ents {
		if len(e.data) > 4 {
			offsets[i] = uint32(dataOff + extra.Len())
			extra.Write(e.data)
			if extra.Len()%2 == 1 {
				extra.WriteByte(0)
			}
		}
	}
	pixOff := uint32(dataOff + extra.Len())
	var out bytes.Buffer
	out.WriteString("II")
	out.Write(u16(42))
	out.Write(u32(8))
	out.Write(u16(uint16(len(ents))))
	for i, e := range ents {
		out.Write(u16(e.tag, e.typ))
		out.Write(u32(e.count))
		switch {
		case e.tag == 273:
			out.Write(u32(pixOff))
		case len(e.data) > 4:
			out.Write(u32(offsets[i]))
		default:
			v := make([]byte, 4)
			copy(v, e.data)
			out.Write(v)
		}
	}
	out.Write(u32(0))
	out.Write(extra.Bytes())
	out.Write(pix)
	return out.Bytes()
}

// halfImage is left red / right blue, fully opaque.
func halfImage(w, h int) *image.NRGBA {
	img := image.NewNRGBA(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := color.NRGBA{255, 0, 0, 255}
			if x >= w/2 {
				c = color.NRGBA{0, 0, 255, 255}
			}
			img.SetNRGBA(x, y, c)
		}
	}
	return img
}

// Somewhere in Germany, UTM 32N.
const testLat, testLon = 50.1, 8.6

func utmTestGeo(t *testing.T) testGeo {
	n, e, zone, north := llToUTM(testLat, testLon)
	require.Equal(t, 32, zone)
	require.True(t, north)
	return testGeo{epsg: 32632, tie: [6]float64{0, 0, 0, e, n, 0}, scale: [3]float64{0.05, 0.05, 0}}
}

// ---------------------------------------------------------------------------
// Maths
// ---------------------------------------------------------------------------

func TestTileMathRoundTrip(t *testing.T) {
	x, y := lonLatToTileFrac(0, 0, 0)
	assert.InDelta(t, 0.5, x, 1e-12)
	assert.InDelta(t, 0.5, y, 1e-12)
	for _, z := range []int{16, 20, 23} {
		fx, fy := lonLatToTileFrac(testLon, testLat, z)
		lon, lat := tileFracToLonLat(fx, fy, z)
		assert.InDelta(t, testLon, lon, 1e-10)
		assert.InDelta(t, testLat, lat, 1e-10)
	}
	// Known OSM tile: Frankfurt-ish at z=18.
	fx, fy := lonLatToTileFrac(8.6, 50.1, 18)
	assert.Equal(t, 137334, int(fx))
	assert.Equal(t, 88791, int(fy))
	x0, y0, x1, y1 := tileRange([4]float64{8.6, 50.1, 8.6001, 50.1001}, 23)
	assert.LessOrEqual(t, x0, x1)
	assert.LessOrEqual(t, y0, y1) // north edge maps to the smaller row
	assert.Equal(t, 0, tmsToXYZ(1, 1))
	assert.Equal(t, 7, tmsToXYZ(3, 0))
}

func TestZoomRange(t *testing.T) {
	// 2 cm/px at 50° lat resolves ~z22.
	minZ, maxZ := imageryZoomRange(0.02, 50)
	assert.Equal(t, 16, minZ)
	assert.Equal(t, 22, maxZ)
	_, maxZ = imageryZoomRange(0.005, 50)
	assert.Equal(t, 23, maxZ)
	_, maxZ = imageryZoomRange(1.0, 50)
	assert.Equal(t, 18, maxZ)
}

func TestCRSRoundTrip(t *testing.T) {
	for _, code := range []int{4326, 3857, 32632, 25832} {
		c, err := crsFromEPSG(code)
		require.NoError(t, err)
		x, y := c.fromLonLat(testLon, testLat)
		lon, lat := c.toLonLat(x, y)
		assert.InDelta(t, testLon, lon, 1e-8, "EPSG:%d", code)
		assert.InDelta(t, testLat, lat, 1e-8, "EPSG:%d", code)
	}
	c, _ := crsFromEPSG(3857)
	x, _ := c.fromLonLat(180, 0)
	assert.InDelta(t, 20037508.342789244, x, 1e-6)
	_, err := crsFromEPSG(2154)
	assert.Error(t, err)
	// Forced zone works outside the natural zone (8.6°E is zone 32, use 31).
	n31, e31 := llToUTMInZone(testLat, testLon, 31, true)
	lat, lon := utmToLL(n31, e31, 31, true)
	assert.InDelta(t, testLat, lat, 1e-6) // series error grows off the CM
	assert.InDelta(t, testLon, lon, 1e-6)
}

func TestGeoAffineInverse(t *testing.T) {
	a := geoAffine{A: 100, B: 0.5, C: 0.1, D: 200, E: -0.05, F: -0.5}
	x, y := a.forward(12.5, 7.25)
	col, row, ok := a.inverse(x, y)
	require.True(t, ok)
	assert.InDelta(t, 12.5, col, 1e-9)
	assert.InDelta(t, 7.25, row, 1e-9)
	_, _, ok = geoAffine{}.inverse(1, 1)
	assert.False(t, ok)
}

func TestParseGeoTIFF(t *testing.T) {
	g := utmTestGeo(t)
	b := buildGeoTIFF(t, halfImage(40, 20), g)
	info, err := parseGeoTIFF(bytes.NewReader(b))
	require.NoError(t, err)
	assert.Equal(t, 40, info.Width)
	assert.Equal(t, 20, info.Height)
	assert.Equal(t, crsUTM, info.CRS.Kind)
	assert.Equal(t, 32, info.CRS.Zone)
	x, y := info.Affine.forward(0, 0)
	assert.InDelta(t, g.tie[3], x, 1e-9)
	assert.InDelta(t, g.tie[4], y, 1e-9)
	x, y = info.Affine.forward(40, 20)
	assert.InDelta(t, g.tie[3]+2, x, 1e-9)
	assert.InDelta(t, g.tie[4]-1, y, 1e-9)
	assert.InDelta(t, 0.05, info.groundResolution(), 1e-3)
	bb := info.lonLatBounds()
	assert.InDelta(t, testLon, bb[0], 1e-6)
	assert.InDelta(t, testLat, bb[3], 1e-6)
	assert.Greater(t, bb[2], bb[0])
	assert.Less(t, bb[1], bb[3])

	// PixelIsPoint shifts the corner by half a pixel.
	g.pixelPoint = true
	g.noData = "0"
	info, err = parseGeoTIFF(bytes.NewReader(buildGeoTIFF(t, halfImage(4, 4), g)))
	require.NoError(t, err)
	x, y = info.Affine.forward(0, 0)
	assert.InDelta(t, g.tie[3]-0.025, x, 1e-9)
	assert.InDelta(t, g.tie[4]+0.025, y, 1e-9)
	require.NotNil(t, info.NoData)
	assert.Equal(t, 0.0, *info.NoData)

	// Geographic.
	gg := testGeo{epsg: 4326, geographic: true, tie: [6]float64{0, 0, 0, testLon, testLat, 0}, scale: [3]float64{1e-6, 1e-6, 0}}
	info, err = parseGeoTIFF(bytes.NewReader(buildGeoTIFF(t, halfImage(4, 4), gg)))
	require.NoError(t, err)
	assert.Equal(t, crsLonLat, info.CRS.Kind)

	_, err = parseGeoTIFF(bytes.NewReader([]byte("not a tiff at all")))
	assert.Error(t, err)
}

func TestProcessGeoTIFFRendersTiles(t *testing.T) {
	dir := t.TempDir()
	src := filepath.Join(dir, "source.tif")
	g := utmTestGeo(t)
	require.NoError(t, os.WriteFile(src, buildGeoTIFF(t, halfImage(200, 100), g), 0o644))
	res, err := processGeoTIFF(context.Background(), src, filepath.Join(dir, "tiles"), func(float64) {})
	require.NoError(t, err)
	assert.Equal(t, 16, res.minZoom)
	assert.Equal(t, 21, res.maxZoom) // 5 cm/px at 50°N
	require.NotNil(t, res.bounds)

	// Sample a point in the left (red) and right (blue) half at max zoom.
	check := func(col, row float64, want color.NRGBA) {
		x, y := g.tie[3]+col*0.05, g.tie[4]-row*0.05
		c, _ := crsFromEPSG(32632)
		lon, lat := c.toLonLat(x, y)
		fx, fy := lonLatToTileFrac(lon, lat, res.maxZoom)
		p := tilePath(filepath.Join(dir, "tiles"), res.maxZoom, int(fx), int(fy), ".png")
		tile := readTile(p)
		require.NotNil(t, tile, p)
		px := int((fx - math.Floor(fx)) * imageryTileSize)
		py := int((fy - math.Floor(fy)) * imageryTileSize)
		got := color.NRGBAModel.Convert(tile.At(px, py)).(color.NRGBA)
		assert.Equal(t, want, got, "col=%v row=%v", col, row)
	}
	check(50, 50, color.NRGBA{255, 0, 0, 255})
	check(150, 50, color.NRGBA{0, 0, 255, 255})

	// Lower zooms exist via the pyramid.
	fx, fy := lonLatToTileFrac(res.bounds[0], res.bounds[3], 16)
	assert.FileExists(t, tilePath(filepath.Join(dir, "tiles"), 16, int(fx), int(fy), ".png"))
}

func TestProcessGeoTIFFRejectsUngeoreferenced(t *testing.T) {
	dir := t.TempDir()
	src := filepath.Join(dir, "source.tif")
	var buf bytes.Buffer
	require.NoError(t, png.Encode(&buf, halfImage(2, 2)))
	require.NoError(t, os.WriteFile(src, buf.Bytes(), 0o644))
	_, err := processGeoTIFF(context.Background(), src, filepath.Join(dir, "tiles"), func(float64) {})
	assert.Error(t, err)
}

func TestFitWithin(t *testing.T) {
	w, h := fitWithin(8000, 4000, 4096)
	assert.Equal(t, 4096, w)
	assert.Equal(t, 2048, h)
	w, h = fitWithin(100, 50, 4096)
	assert.Equal(t, 100, w)
	assert.Equal(t, 50, h)
}

func TestReorderOverlays(t *testing.T) {
	ov := []*imageryOverlay{{Name: "a"}, {Name: "b"}, {Name: "c"}}
	got := reorderOverlays(ov, []string{"c", "x", "a"})
	names := []string{}
	for _, o := range got {
		names = append(names, o.Name)
	}
	assert.Equal(t, []string{"c", "a", "b"}, names)
}

func TestImageryKindForFile(t *testing.T) {
	for f, k := range map[string]string{"o.TIF": "geotiff", "a.jpg": "image", "t.zip": "xyz", "m.mbtiles": "mbtiles"} {
		got, _, err := imageryKindForFile(f)
		require.NoError(t, err)
		assert.Equal(t, k, got)
	}
	_, _, err := imageryKindForFile("x.exe")
	assert.Error(t, err)
}

// ---------------------------------------------------------------------------
// Routes
// ---------------------------------------------------------------------------

func newImageryTestServer(t *testing.T) (*gin.Engine, *imageryStore) {
	gin.SetMode(gin.TestMode)
	s := newImageryStore(t.TempDir())
	r := gin.New()
	imageryRoutesWithStore(r.Group("/api"), s)
	return r, s
}

func uploadFile(t *testing.T, r *gin.Engine, filename string, data []byte, fields map[string]string) *httptest.ResponseRecorder {
	var body bytes.Buffer
	mw := multipart.NewWriter(&body)
	for k, v := range fields {
		require.NoError(t, mw.WriteField(k, v))
	}
	fw, err := mw.CreateFormFile("file", filename)
	require.NoError(t, err)
	_, _ = fw.Write(data)
	require.NoError(t, mw.Close())
	req := httptest.NewRequest(http.MethodPost, "/api/mowglinext/imagery", &body)
	req.Header.Set("Content-Type", mw.FormDataContentType())
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}

func waitReady(t *testing.T, s *imageryStore, name string) imageryOverlay {
	deadline := time.Now().Add(20 * time.Second)
	for time.Now().Before(deadline) {
		o, ok := s.get(name)
		require.True(t, ok)
		if o.Status != "processing" {
			return o
		}
		time.Sleep(20 * time.Millisecond)
	}
	t.Fatalf("overlay %s still processing", name)
	return imageryOverlay{}
}

func imgReq(r *gin.Engine, method, path string, body any) *httptest.ResponseRecorder {
	var rd *bytes.Reader
	if body != nil {
		b, _ := json.Marshal(body)
		rd = bytes.NewReader(b)
	} else {
		rd = bytes.NewReader(nil)
	}
	req := httptest.NewRequest(method, path, rd)
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}

func TestImageryGeoTIFFUploadAndTileRoute(t *testing.T) {
	r, s := newImageryTestServer(t)
	g := utmTestGeo(t)
	w := uploadFile(t, r, "orthophoto.tif", buildGeoTIFF(t, halfImage(100, 100), g), map[string]string{"label": "Drone June"})
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	var o imageryOverlay
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &o))
	assert.Equal(t, "drone-june", o.Name)
	assert.Equal(t, "geotiff", o.Kind)
	o = waitReady(t, s, o.Name)
	require.Equal(t, "ready", o.Status, o.Error)
	assert.Contains(t, o.CRS, "32632")

	// Persisted.
	s2 := newImageryStore(s.root)
	assert.Len(t, s2.list(), 1)

	fx, fy := lonLatToTileFrac(testLon+0.00001, testLat-0.00001, o.MaxZoom)
	w = imgReq(r, http.MethodGet, "/api/mowglinext/tiles/drone-june/"+itoa(o.MaxZoom)+"/"+itoa(int(fx))+"/"+itoa(int(fy))+".png", nil)
	require.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, "image/png", w.Header().Get("Content-Type"))
	assert.Contains(t, w.Header().Get("Cache-Control"), "max-age")
	_, err := png.Decode(bytes.NewReader(w.Body.Bytes()))
	require.NoError(t, err)

	w = imgReq(r, http.MethodGet, "/api/mowglinext/tiles/drone-june/23/1/1.png", nil)
	assert.Equal(t, http.StatusNoContent, w.Code)
	w = imgReq(r, http.MethodGet, "/api/mowglinext/tiles/..%2Fetc/1/1/1.png", nil)
	assert.NotEqual(t, http.StatusOK, w.Code)
	w = imgReq(r, http.MethodGet, "/api/mowglinext/tiles/nope/1/1/1.png", nil)
	assert.Equal(t, http.StatusNotFound, w.Code)

	// Opacity / visibility.
	w = imgReq(r, http.MethodPatch, "/api/mowglinext/imagery/drone-june", map[string]any{"opacity": 1.7, "visible": false})
	require.Equal(t, http.StatusOK, w.Code)
	o, _ = s.get("drone-june")
	assert.Equal(t, 1.0, o.Opacity)
	assert.False(t, o.Visible)

	// Delete removes files.
	w = imgReq(r, http.MethodDelete, "/api/mowglinext/imagery/drone-june", nil)
	require.Equal(t, http.StatusOK, w.Code)
	assert.NoDirExists(t, filepath.Join(s.root, "drone-june"))
	assert.Empty(t, s.list())
}

func itoa(i int) string { return strconv.Itoa(i) }

func TestImageryPlainImageUploadPlacementAndOrder(t *testing.T) {
	r, s := newImageryTestServer(t)
	var buf bytes.Buffer
	require.NoError(t, png.Encode(&buf, halfImage(64, 32)))
	w := uploadFile(t, r, "garden.png", buf.Bytes(), nil)
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	o := waitReady(t, s, "garden")
	require.Equal(t, "ready", o.Status, o.Error)
	assert.Equal(t, 64, o.Width)
	assert.Equal(t, ".png", o.ImageExt)

	w = imgReq(r, http.MethodGet, "/api/mowglinext/imagery/garden/image", nil)
	require.Equal(t, http.StatusOK, w.Code)

	pl := imageryPlacement{CenterX: 1.5, CenterY: -2, MetersPerPixel: 0.03, RotationDeg: 12}
	w = imgReq(r, http.MethodPatch, "/api/mowglinext/imagery/garden", map[string]any{
		"placement":     pl,
		"controlPoints": []imageryControlPoint{{U: 0.1, V: 0.2, X: 1, Y: 2}},
	})
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
	o, _ = s.get("garden")
	require.NotNil(t, o.Placement)
	assert.Equal(t, pl, *o.Placement)
	assert.Len(t, o.ControlPoints, 1)

	w = imgReq(r, http.MethodPatch, "/api/mowglinext/imagery/garden", map[string]any{"placement": imageryPlacement{MetersPerPixel: 0}})
	assert.Equal(t, http.StatusBadRequest, w.Code)

	// Second upload with same label gets a unique name; then reorder.
	w = uploadFile(t, r, "garden.png", buf.Bytes(), nil)
	require.Equal(t, http.StatusOK, w.Code)
	waitReady(t, s, "garden-2")
	w = imgReq(r, http.MethodPost, "/api/mowglinext/imagery-order", map[string]any{"names": []string{"garden-2", "garden"}})
	require.Equal(t, http.StatusOK, w.Code)
	l := s.list()
	assert.Equal(t, "garden-2", l[0].Name)
}

func TestImageryUploadRejectsUnknownType(t *testing.T) {
	r, _ := newImageryTestServer(t)
	w := uploadFile(t, r, "virus.exe", []byte("MZ"), nil)
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func pngBytes(t *testing.T) []byte {
	var buf bytes.Buffer
	require.NoError(t, png.Encode(&buf, halfImage(256, 256)))
	return buf.Bytes()
}

func TestImageryXYZZipTMSFlip(t *testing.T) {
	dir := t.TempDir()
	zp := filepath.Join(dir, "t.zip")
	var buf bytes.Buffer
	zw := zip.NewWriter(&buf)
	for _, p := range []string{"site/tilemapresource.xml", "site/20/549321/670000.png", "site/19/274660/335000.png"} {
		f, err := zw.Create(p)
		require.NoError(t, err)
		_, _ = f.Write(pngBytes(t))
	}
	require.NoError(t, zw.Close())
	require.NoError(t, os.WriteFile(zp, buf.Bytes(), 0o644))
	res, err := processXYZZip(context.Background(), zp, filepath.Join(dir, "tiles"), "auto", func(float64) {})
	require.NoError(t, err)
	assert.Equal(t, 19, res.minZoom)
	assert.Equal(t, 20, res.maxZoom)
	assert.FileExists(t, tilePath(filepath.Join(dir, "tiles"), 20, 549321, tmsToXYZ(20, 670000), ".png"))

	// Explicit xyz: no flip.
	res, err = processXYZZip(context.Background(), zp, filepath.Join(dir, "tiles2"), "xyz", func(float64) {})
	require.NoError(t, err)
	assert.FileExists(t, tilePath(filepath.Join(dir, "tiles2"), 20, 549321, 670000, ".png"))
	_ = res
}

func TestImageryMBTiles(t *testing.T) {
	dir := t.TempDir()
	p := filepath.Join(dir, "s.mbtiles")
	db, err := sql.Open("sqlite", p)
	require.NoError(t, err)
	_, err = db.Exec(`CREATE TABLE metadata (name text, value text);
		CREATE TABLE tiles (zoom_level integer, tile_column integer, tile_row integer, tile_data blob);
		INSERT INTO metadata VALUES ('format','png');`)
	require.NoError(t, err)
	_, err = db.Exec(`INSERT INTO tiles VALUES (21, 1098642, 1340000, ?)`, pngBytes(t))
	require.NoError(t, err)
	require.NoError(t, db.Close())
	res, err := processMBTiles(context.Background(), p, filepath.Join(dir, "tiles"), func(float64) {})
	require.NoError(t, err)
	assert.Equal(t, 21, res.maxZoom)
	assert.Equal(t, ".png", res.tileExt)
	assert.FileExists(t, tilePath(filepath.Join(dir, "tiles"), 21, 1098642, tmsToXYZ(21, 1340000), ".png"))
}
