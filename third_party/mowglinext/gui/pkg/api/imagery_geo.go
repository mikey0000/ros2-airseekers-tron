package api

// Custom imagery: GeoTIFF georeferencing, CRS reprojection and web-mercator
// XYZ tile maths. Pure Go (no GDAL in the GUI image): the GeoTIFF tags are read
// with a small IFD parser, the pixels with golang.org/x/image/tiff.

import (
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"strconv"
	"strings"
)

// ---------------------------------------------------------------------------
// CRS
// ---------------------------------------------------------------------------

type crsKind int

const (
	crsLonLat      crsKind = iota // EPSG:4326 (and other geographic lon/lat CRSs)
	crsWebMercator                // EPSG:3857
	crsUTM                        // WGS84 / ETRS89 / NAD83 UTM zones
)

type imageryCRS struct {
	Kind  crsKind
	EPSG  int
	Zone  int
	North bool
}

func (c imageryCRS) String() string {
	switch c.Kind {
	case crsWebMercator:
		return "EPSG:3857"
	case crsUTM:
		h := "N"
		if !c.North {
			h = "S"
		}
		return fmt.Sprintf("EPSG:%d (UTM %d%s)", c.EPSG, c.Zone, h)
	default:
		return fmt.Sprintf("EPSG:%d", c.EPSG)
	}
}

const webMercatorR = 6378137.0

// crsFromEPSG maps a projected/geographic EPSG code to a supported CRS.
func crsFromEPSG(code int) (imageryCRS, error) {
	switch {
	case code == 4326 || code == 4258 || code == 4269 || code == 4979:
		return imageryCRS{Kind: crsLonLat, EPSG: code}, nil
	case code == 3857 || code == 3785 || code == 900913 || code == 102100:
		return imageryCRS{Kind: crsWebMercator, EPSG: code}, nil
	case code >= 32601 && code <= 32660:
		return imageryCRS{Kind: crsUTM, EPSG: code, Zone: code - 32600, North: true}, nil
	case code >= 32701 && code <= 32760:
		return imageryCRS{Kind: crsUTM, EPSG: code, Zone: code - 32700, North: false}, nil
	case code >= 25828 && code <= 25838: // ETRS89 / UTM (treated as WGS84, sub-metre)
		return imageryCRS{Kind: crsUTM, EPSG: code, Zone: code - 25800, North: true}, nil
	case code >= 26901 && code <= 26923: // NAD83 / UTM (treated as WGS84, ~1-2 m)
		return imageryCRS{Kind: crsUTM, EPSG: code, Zone: code - 26900, North: true}, nil
	}
	return imageryCRS{}, fmt.Errorf("unsupported CRS EPSG:%d (supported: 4326, 3857, UTM 326xx/327xx/258xx/269xx); reproject with `gdalwarp -t_srs EPSG:3857`", code)
}

// toLonLat converts CRS coordinates (X, Y) to WGS84 lon/lat degrees.
func (c imageryCRS) toLonLat(x, y float64) (lon, lat float64) {
	switch c.Kind {
	case crsWebMercator:
		lon = x / webMercatorR * 180 / math.Pi
		lat = (2*math.Atan(math.Exp(y/webMercatorR)) - math.Pi/2) * 180 / math.Pi
		return
	case crsUTM:
		lat, lon = utmToLL(y, x, c.Zone, c.North)
		return
	default:
		return x, y
	}
}

// fromLonLat converts WGS84 lon/lat degrees to CRS coordinates (X, Y).
func (c imageryCRS) fromLonLat(lon, lat float64) (x, y float64) {
	switch c.Kind {
	case crsWebMercator:
		x = webMercatorR * lon * math.Pi / 180
		y = webMercatorR * math.Log(math.Tan(math.Pi/4+lat*math.Pi/360))
		return
	case crsUTM:
		n, e := llToUTMInZone(lat, lon, c.Zone, c.North)
		return e, n
	default:
		return lon, lat
	}
}

// ---------------------------------------------------------------------------
// Affine pixel <-> CRS transform. Pixel coordinates use the PixelIsArea
// convention: (0,0) is the top-left corner of the top-left pixel, the centre of
// pixel (i,j) is (i+0.5, j+0.5).
//   X = A + B*col + C*row
//   Y = D + E*col + F*row
// ---------------------------------------------------------------------------

type geoAffine struct{ A, B, C, D, E, F float64 }

func (g geoAffine) forward(col, row float64) (x, y float64) {
	return g.A + g.B*col + g.C*row, g.D + g.E*col + g.F*row
}

func (g geoAffine) inverse(x, y float64) (col, row float64, ok bool) {
	det := g.B*g.F - g.C*g.E
	if det == 0 {
		return 0, 0, false
	}
	dx, dy := x-g.A, y-g.D
	return (g.F*dx - g.C*dy) / det, (-g.E*dx + g.B*dy) / det, true
}

// ---------------------------------------------------------------------------
// GeoTIFF tag parsing
// ---------------------------------------------------------------------------

const (
	tiffTagWidth          = 256
	tiffTagHeight         = 257
	tiffTagCompression    = 259
	geoTagPixelScale      = 33550
	geoTagTiepoint        = 33922
	geoTagTransformation  = 34264
	geoTagKeyDirectory    = 34735
	gdalTagNoData         = 42113
	geoKeyModelType       = 1024
	geoKeyRasterType      = 1025
	geoKeyGeographicType  = 2048
	geoKeyProjectedCSType = 3072
)

type geoTIFFInfo struct {
	Width, Height int
	Compression   int
	Affine        geoAffine
	CRS           imageryCRS
	NoData        *float64
}

type tiffEntry struct {
	tag, typ uint16
	count    uint32
	raw      []byte // value bytes (count*size)
}

var tiffTypeSize = map[uint16]int{1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8, 11: 4, 12: 8}

// readTIFFEntries parses the first IFD of a classic (non-Big) TIFF.
func readTIFFEntries(r io.ReaderAt) (map[uint16]tiffEntry, binary.ByteOrder, error) {
	hdr := make([]byte, 8)
	if _, err := r.ReadAt(hdr, 0); err != nil {
		return nil, nil, fmt.Errorf("read TIFF header: %w", err)
	}
	var bo binary.ByteOrder
	switch string(hdr[:2]) {
	case "II":
		bo = binary.LittleEndian
	case "MM":
		bo = binary.BigEndian
	default:
		return nil, nil, errors.New("not a TIFF file")
	}
	switch bo.Uint16(hdr[2:4]) {
	case 42:
	case 43:
		return nil, nil, errors.New("BigTIFF is not supported; re-export with `gdal_translate -co BIGTIFF=NO -co COMPRESS=DEFLATE` or pre-tile with gdal2tiles")
	default:
		return nil, nil, errors.New("not a TIFF file")
	}
	off := int64(bo.Uint32(hdr[4:8]))
	cnt := make([]byte, 2)
	if _, err := r.ReadAt(cnt, off); err != nil {
		return nil, nil, fmt.Errorf("read IFD: %w", err)
	}
	n := int(bo.Uint16(cnt))
	buf := make([]byte, 12*n)
	if _, err := r.ReadAt(buf, off+2); err != nil {
		return nil, nil, fmt.Errorf("read IFD entries: %w", err)
	}
	out := make(map[uint16]tiffEntry, n)
	for i := 0; i < n; i++ {
		e := buf[i*12 : i*12+12]
		ent := tiffEntry{tag: bo.Uint16(e[0:2]), typ: bo.Uint16(e[2:4]), count: bo.Uint32(e[4:8])}
		sz, ok := tiffTypeSize[ent.typ]
		if !ok {
			continue
		}
		total := int64(sz) * int64(ent.count)
		if total > 1<<20 {
			continue // huge arrays (strip offsets etc.) are not needed here
		}
		if total <= 4 {
			ent.raw = append([]byte(nil), e[8:8+total]...)
		} else {
			ent.raw = make([]byte, total)
			if _, err := r.ReadAt(ent.raw, int64(bo.Uint32(e[8:12]))); err != nil {
				return nil, nil, fmt.Errorf("read tag %d: %w", ent.tag, err)
			}
		}
		out[ent.tag] = ent
	}
	return out, bo, nil
}

func (e tiffEntry) uints(bo binary.ByteOrder) []uint64 {
	sz := tiffTypeSize[e.typ]
	out := make([]uint64, 0, e.count)
	for i := 0; i+sz <= len(e.raw); i += sz {
		switch sz {
		case 1:
			out = append(out, uint64(e.raw[i]))
		case 2:
			out = append(out, uint64(bo.Uint16(e.raw[i:])))
		case 4:
			out = append(out, uint64(bo.Uint32(e.raw[i:])))
		}
	}
	return out
}

func (e tiffEntry) doubles(bo binary.ByteOrder) []float64 {
	out := make([]float64, 0, e.count)
	switch e.typ {
	case 12:
		for i := 0; i+8 <= len(e.raw); i += 8 {
			out = append(out, math.Float64frombits(bo.Uint64(e.raw[i:])))
		}
	case 11:
		for i := 0; i+4 <= len(e.raw); i += 4 {
			out = append(out, float64(math.Float32frombits(bo.Uint32(e.raw[i:]))))
		}
	}
	return out
}

// parseGeoTIFF reads the georeferencing of a GeoTIFF (first IFD).
func parseGeoTIFF(r io.ReaderAt) (*geoTIFFInfo, error) {
	ents, bo, err := readTIFFEntries(r)
	if err != nil {
		return nil, err
	}
	info := &geoTIFFInfo{Compression: 1}
	if e, ok := ents[tiffTagWidth]; ok && len(e.uints(bo)) > 0 {
		info.Width = int(e.uints(bo)[0])
	}
	if e, ok := ents[tiffTagHeight]; ok && len(e.uints(bo)) > 0 {
		info.Height = int(e.uints(bo)[0])
	}
	if e, ok := ents[tiffTagCompression]; ok && len(e.uints(bo)) > 0 {
		info.Compression = int(e.uints(bo)[0])
	}
	if info.Width <= 0 || info.Height <= 0 {
		return nil, errors.New("TIFF has no image size")
	}

	// GeoKeys
	keys := map[int]int{}
	if e, ok := ents[geoTagKeyDirectory]; ok {
		v := e.uints(bo)
		if len(v) >= 4 {
			nk := int(v[3])
			for i := 0; i < nk && 4+i*4+3 < len(v); i++ {
				k := v[4+i*4:]
				if k[1] == 0 { // value stored inline
					keys[int(k[0])] = int(k[3])
				}
			}
		}
	} else {
		return nil, errors.New("TIFF is not georeferenced (no GeoKeyDirectory tag); export a GeoTIFF or use the plain-image upload with manual alignment")
	}

	switch {
	case keys[geoKeyProjectedCSType] != 0 && keys[geoKeyProjectedCSType] != 32767:
		info.CRS, err = crsFromEPSG(keys[geoKeyProjectedCSType])
	case keys[geoKeyModelType] == 2 || (keys[geoKeyModelType] == 0 && keys[geoKeyGeographicType] != 0):
		code := keys[geoKeyGeographicType]
		if code == 0 || code == 32767 {
			code = 4326
		}
		info.CRS, err = crsFromEPSG(code)
	default:
		err = errors.New("GeoTIFF CRS is user-defined or missing an EPSG code; reproject with `gdalwarp -t_srs EPSG:3857 in.tif out.tif`")
	}
	if err != nil {
		return nil, err
	}

	// Affine
	if e, ok := ents[geoTagTransformation]; ok && len(e.doubles(bo)) >= 16 {
		m := e.doubles(bo)
		info.Affine = geoAffine{A: m[3], B: m[0], C: m[1], D: m[7], E: m[4], F: m[5]}
	} else {
		tp, ok1 := ents[geoTagTiepoint]
		sc, ok2 := ents[geoTagPixelScale]
		if !ok1 || !ok2 {
			return nil, errors.New("GeoTIFF has no tiepoint/pixel-scale or transformation tag")
		}
		t := tp.doubles(bo)
		s := sc.doubles(bo)
		if len(t) < 6 || len(s) < 2 || s[0] == 0 || s[1] == 0 {
			return nil, errors.New("GeoTIFF tiepoint/pixel-scale tags are malformed")
		}
		info.Affine = geoAffine{
			A: t[3] - t[0]*s[0], B: s[0], C: 0,
			D: t[4] + t[1]*s[1], E: 0, F: -s[1],
		}
	}
	if keys[geoKeyRasterType] == 2 { // PixelIsPoint: tiepoint refers to pixel centres
		a := info.Affine
		a.A -= 0.5*a.B + 0.5*a.C
		a.D -= 0.5*a.E + 0.5*a.F
		info.Affine = a
	}
	if e, ok := ents[gdalTagNoData]; ok && e.typ == 2 {
		s := strings.TrimRight(string(e.raw), "\x00 ")
		if v, err := strconv.ParseFloat(strings.TrimSpace(s), 64); err == nil {
			info.NoData = &v
		}
	}
	return info, nil
}

// lonLatBounds returns the WGS84 bounding box [west, south, east, north] of
// the raster, sampling the edges so curved UTM/mercator edges are covered.
func (g *geoTIFFInfo) lonLatBounds() [4]float64 {
	b := [4]float64{math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)}
	add := func(col, row float64) {
		x, y := g.Affine.forward(col, row)
		lon, lat := g.CRS.toLonLat(x, y)
		b[0] = math.Min(b[0], lon)
		b[1] = math.Min(b[1], lat)
		b[2] = math.Max(b[2], lon)
		b[3] = math.Max(b[3], lat)
	}
	const steps = 16
	w, h := float64(g.Width), float64(g.Height)
	for i := 0; i <= steps; i++ {
		f := float64(i) / steps
		add(f*w, 0)
		add(f*w, h)
		add(0, f*h)
		add(w, f*h)
	}
	return b
}

// groundResolution returns the approximate ground sample distance (m/pixel)
// at the raster centre.
func (g *geoTIFFInfo) groundResolution() float64 {
	cx, cy := float64(g.Width)/2, float64(g.Height)/2
	x0, y0 := g.Affine.forward(cx, cy)
	x1, y1 := g.Affine.forward(cx+1, cy)
	lon0, lat0 := g.CRS.toLonLat(x0, y0)
	lon1, lat1 := g.CRS.toLonLat(x1, y1)
	return haversineM(lat0, lon0, lat1, lon1)
}

func haversineM(lat1, lon1, lat2, lon2 float64) float64 {
	const r = 6371008.8
	p1, p2 := lat1*math.Pi/180, lat2*math.Pi/180
	dp := p2 - p1
	dl := (lon2 - lon1) * math.Pi / 180
	a := math.Sin(dp/2)*math.Sin(dp/2) + math.Cos(p1)*math.Cos(p2)*math.Sin(dl/2)*math.Sin(dl/2)
	return 2 * r * math.Asin(math.Min(1, math.Sqrt(a)))
}

// ---------------------------------------------------------------------------
// Web-mercator XYZ tile maths (Google/OSM scheme, y=0 at the north edge).
// ---------------------------------------------------------------------------

const imageryTileSize = 256

// lonLatToTileFrac returns fractional tile coordinates of lon/lat at zoom z.
func lonLatToTileFrac(lon, lat float64, z int) (x, y float64) {
	n := math.Exp2(float64(z))
	lat = math.Max(-85.05112878, math.Min(85.05112878, lat))
	x = (lon + 180) / 360 * n
	latR := lat * math.Pi / 180
	y = (1 - math.Log(math.Tan(latR)+1/math.Cos(latR))/math.Pi) / 2 * n
	return
}

// tileFracToLonLat inverts lonLatToTileFrac.
func tileFracToLonLat(x, y float64, z int) (lon, lat float64) {
	n := math.Exp2(float64(z))
	lon = x/n*360 - 180
	lat = math.Atan(math.Sinh(math.Pi*(1-2*y/n))) * 180 / math.Pi
	return
}

// tileRange returns the inclusive tile index range covering bounds at zoom z.
func tileRange(b [4]float64, z int) (x0, y0, x1, y1 int) {
	fx0, fy0 := lonLatToTileFrac(b[0], b[3], z) // north-west
	fx1, fy1 := lonLatToTileFrac(b[2], b[1], z) // south-east
	max := int(math.Exp2(float64(z))) - 1
	clamp := func(v int) int {
		if v < 0 {
			return 0
		}
		if v > max {
			return max
		}
		return v
	}
	return clamp(int(math.Floor(fx0))), clamp(int(math.Floor(fy0))), clamp(int(math.Floor(fx1))), clamp(int(math.Floor(fy1)))
}

// nativeZoom returns the zoom whose 256-px tile pixel size matches the given
// ground resolution (m/px) at latitude lat.
func nativeZoom(gsd, lat float64) float64 {
	if gsd <= 0 {
		return 23
	}
	return math.Log2(156543.03392804097 * math.Cos(lat*math.Pi/180) / gsd)
}

// imageryZoomRange picks the tile pyramid to generate: up to the zoom that
// resolves the source GSD (clamped 18..23), down to 16 for overview.
func imageryZoomRange(gsd, lat float64) (minZ, maxZ int) {
	maxZ = int(math.Round(nativeZoom(gsd, lat)))
	if maxZ > 23 {
		maxZ = 23
	}
	if maxZ < 18 {
		maxZ = 18
	}
	return 16, maxZ
}

// tmsToXYZ flips a TMS tile row into the XYZ scheme.
func tmsToXYZ(z, y int) int { return (1 << uint(z)) - 1 - y }
