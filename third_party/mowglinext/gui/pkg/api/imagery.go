package api

// Custom map imagery (drone orthomosaics etc.) layered above the Mapbox
// satellite base on the Map page.
//
// Storage: $MAP_TILES_DIR (default /map_tiles, mounted from
// /userdata/ros2/map_tiles) holds overlays.json plus one folder per overlay:
//
//   <name>/source.<ext>          the upload as received
//   <name>/tiles/<z>/<x>/<y>.png XYZ tiles (geotiff / xyz / mbtiles kinds)
//   <name>/display.(png|jpg)     downscaled image (plain-image kind)
//
// Kinds:
//   geotiff  georeferenced GeoTIFF (EPSG:4326/3857/UTM) tiled in pure Go
//   xyz      a .zip of a z/x/y tile folder (gdal2tiles, QGIS "XYZ tiles")
//   mbtiles  an .mbtiles SQLite tile set, unpacked to tiles/
//   image    plain PNG/JPEG placed in the ROBOT MAP FRAME (metres) by the web
//            client (control points / drag-rotate-scale); rendered client-side
//            as a Mapbox image source so it follows the map offset.

import (
	"archive/zip"
	"context"
	"database/sql"
	"encoding/json"
	"errors"
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"image/png"
	"io"
	"log"
	"math"
	"mime"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gin-gonic/gin"
	xdraw "golang.org/x/image/draw"
	"golang.org/x/image/tiff"
	_ "golang.org/x/image/webp"
	_ "modernc.org/sqlite"
)

const (
	imageryMaxUploadBytes = 300 << 20
	imageryMaxPixels      = 100_000_000 // ~400 MB decoded RGBA; the mower has ~1 GB free
	imageryDisplayMaxSide = 4096        // WebGL-safe texture size for the image kind
)

type imageryControlPoint struct {
	// Normalised image coordinates (0..1, origin top-left).
	U float64 `json:"u"`
	V float64 `json:"v"`
	// Robot map-frame target (metres, x east / y north).
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

type imageryPlacement struct {
	CenterX        float64 `json:"centerX"`
	CenterY        float64 `json:"centerY"`
	MetersPerPixel float64 `json:"metersPerPixel"`
	RotationDeg    float64 `json:"rotationDeg"`
}

type imageryOverlay struct {
	Name          string                `json:"name"`
	Label         string                `json:"label"`
	Kind          string                `json:"kind"`
	Status        string                `json:"status"` // processing | ready | error
	Error         string                `json:"error,omitempty"`
	Progress      float64               `json:"progress"`
	Opacity       float64               `json:"opacity"`
	Visible       bool                  `json:"visible"`
	MinZoom       int                   `json:"minZoom,omitempty"`
	MaxZoom       int                   `json:"maxZoom,omitempty"`
	Bounds        *[4]float64           `json:"bounds,omitempty"` // west,south,east,north
	CRS           string                `json:"crs,omitempty"`
	GSD           float64               `json:"gsd,omitempty"` // m/px of the source
	TileExt       string                `json:"tileExt,omitempty"`
	Width         int                   `json:"width,omitempty"`
	Height        int                   `json:"height,omitempty"`
	ImageExt      string                `json:"imageExt,omitempty"`
	Placement     *imageryPlacement     `json:"placement,omitempty"`
	ControlPoints []imageryControlPoint `json:"controlPoints,omitempty"`
	SizeBytes     int64                 `json:"sizeBytes"`
	CreatedAt     int64                 `json:"createdAt"`
}

type imageryStore struct {
	root     string
	mu       sync.Mutex
	overlays []*imageryOverlay
	cancels  map[string]context.CancelFunc
}

var imageryNameRe = regexp.MustCompile(`^[a-z0-9][a-z0-9_-]{0,63}$`)

func newImageryStore(root string) *imageryStore {
	s := &imageryStore{root: root, cancels: map[string]context.CancelFunc{}}
	if b, err := os.ReadFile(filepath.Join(root, "overlays.json")); err == nil {
		var list []*imageryOverlay
		if err := json.Unmarshal(b, &list); err == nil {
			for _, o := range list {
				if o.Status == "processing" {
					o.Status = "error"
					o.Error = "processing was interrupted (GUI restarted); delete and upload again"
				}
			}
			s.overlays = list
		} else {
			log.Printf("imagery: bad overlays.json: %v", err)
		}
	}
	return s
}

// saveLocked persists the overlay list atomically. Caller holds s.mu.
func (s *imageryStore) saveLocked() error {
	if err := os.MkdirAll(s.root, 0o755); err != nil {
		return err
	}
	if s.overlays == nil {
		s.overlays = []*imageryOverlay{}
	}
	b, err := json.MarshalIndent(s.overlays, "", "  ")
	if err != nil {
		return err
	}
	tmp := filepath.Join(s.root, ".overlays.json.tmp")
	if err := os.WriteFile(tmp, b, 0o644); err != nil {
		return err
	}
	return os.Rename(tmp, filepath.Join(s.root, "overlays.json"))
}

func (s *imageryStore) list() []imageryOverlay {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]imageryOverlay, 0, len(s.overlays))
	for _, o := range s.overlays {
		out = append(out, *o)
	}
	return out
}

func (s *imageryStore) findLocked(name string) (int, *imageryOverlay) {
	for i, o := range s.overlays {
		if o.Name == name {
			return i, o
		}
	}
	return -1, nil
}

func (s *imageryStore) get(name string) (imageryOverlay, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	_, o := s.findLocked(name)
	if o == nil {
		return imageryOverlay{}, false
	}
	return *o, true
}

func (s *imageryStore) update(name string, fn func(o *imageryOverlay)) (imageryOverlay, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	_, o := s.findLocked(name)
	if o == nil {
		return imageryOverlay{}, os.ErrNotExist
	}
	fn(o)
	return *o, s.saveLocked()
}

// setProgress updates progress in memory only (persisted on completion).
func (s *imageryStore) setProgress(name string, p float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, o := s.findLocked(name); o != nil {
		o.Progress = p
	}
}

// slugify turns a label/file name into a unique overlay name.
func (s *imageryStore) uniqueNameLocked(label string) string {
	base := strings.ToLower(label)
	base = regexp.MustCompile(`[^a-z0-9_-]+`).ReplaceAllString(base, "-")
	base = strings.Trim(base, "-_")
	if len(base) > 48 {
		base = base[:48]
	}
	if base == "" {
		base = "imagery"
	}
	name := base
	for i := 2; ; i++ {
		_, o := s.findLocked(name)
		if _, err := os.Stat(filepath.Join(s.root, name)); o == nil && os.IsNotExist(err) {
			return name
		}
		name = fmt.Sprintf("%s-%d", base, i)
	}
}

func imageryKindForFile(filename string) (kind, ext string, err error) {
	ext = strings.ToLower(filepath.Ext(filename))
	switch ext {
	case ".tif", ".tiff":
		return "geotiff", ext, nil
	case ".png", ".jpg", ".jpeg", ".webp":
		return "image", ext, nil
	case ".zip":
		return "xyz", ext, nil
	case ".mbtiles":
		return "mbtiles", ext, nil
	}
	return "", ext, fmt.Errorf("unsupported file type %q (use .tif/.tiff GeoTIFF, .png/.jpg image, .zip of XYZ tiles or .mbtiles)", ext)
}

// ---------------------------------------------------------------------------
// Routes
// ---------------------------------------------------------------------------

func imageryRoot() string {
	if d := os.Getenv("MAP_TILES_DIR"); d != "" {
		return d
	}
	return "/map_tiles"
}

// ImageryRoutes registers the custom-imagery API under /mowglinext.
func ImageryRoutes(r *gin.RouterGroup) {
	imageryRoutesWithStore(r, newImageryStore(imageryRoot()))
}

func imageryRoutesWithStore(r *gin.RouterGroup, s *imageryStore) {
	g := r.Group("/mowglinext")

	// @Summary list custom imagery overlays
	// @Tags imagery
	// @Router /mowglinext/imagery [get]
	g.GET("/imagery", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{"overlays": s.list()})
	})

	// @Summary upload a custom imagery overlay (multipart: file, label, scheme)
	// @Tags imagery
	// @Router /mowglinext/imagery [post]
	g.POST("/imagery", func(c *gin.Context) { s.handleUpload(c) })

	// @Summary update overlay label/opacity/visibility/placement
	// @Tags imagery
	// @Router /mowglinext/imagery/{name} [patch]
	g.PATCH("/imagery/:name", func(c *gin.Context) {
		name := c.Param("name")
		var req struct {
			Label         *string                `json:"label"`
			Opacity       *float64               `json:"opacity"`
			Visible       *bool                  `json:"visible"`
			Placement     *imageryPlacement      `json:"placement"`
			ControlPoints *[]imageryControlPoint `json:"controlPoints"`
		}
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		if req.Placement != nil && (!(req.Placement.MetersPerPixel > 0) || math.IsInf(req.Placement.MetersPerPixel, 0) ||
			math.IsNaN(req.Placement.CenterX) || math.IsNaN(req.Placement.CenterY) || math.IsNaN(req.Placement.RotationDeg)) {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid placement"})
			return
		}
		o, err := s.update(name, func(o *imageryOverlay) {
			if req.Label != nil && strings.TrimSpace(*req.Label) != "" {
				o.Label = strings.TrimSpace(*req.Label)
			}
			if req.Opacity != nil {
				o.Opacity = math.Max(0, math.Min(1, *req.Opacity))
			}
			if req.Visible != nil {
				o.Visible = *req.Visible
			}
			if req.Placement != nil && o.Kind == "image" {
				p := *req.Placement
				o.Placement = &p
			}
			if req.ControlPoints != nil && o.Kind == "image" {
				o.ControlPoints = *req.ControlPoints
			}
		})
		if errors.Is(err, os.ErrNotExist) {
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "no such overlay"})
			return
		} else if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, o)
	})

	// @Summary reorder overlays (first = bottom-most)
	// @Tags imagery
	// @Router /mowglinext/imagery-order [post]
	g.POST("/imagery-order", func(c *gin.Context) {
		var req struct {
			Names []string `json:"names"`
		}
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		s.mu.Lock()
		s.overlays = reorderOverlays(s.overlays, req.Names)
		err := s.saveLocked()
		s.mu.Unlock()
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"overlays": s.list()})
	})

	// @Summary delete an overlay and its files
	// @Tags imagery
	// @Router /mowglinext/imagery/{name} [delete]
	g.DELETE("/imagery/:name", func(c *gin.Context) {
		name := c.Param("name")
		if !imageryNameRe.MatchString(name) {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid name"})
			return
		}
		s.mu.Lock()
		i, o := s.findLocked(name)
		if o == nil {
			s.mu.Unlock()
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "no such overlay"})
			return
		}
		if cancel, ok := s.cancels[name]; ok {
			cancel()
			delete(s.cancels, name)
		}
		s.overlays = append(s.overlays[:i], s.overlays[i+1:]...)
		err := s.saveLocked()
		s.mu.Unlock()
		_ = os.RemoveAll(filepath.Join(s.root, name))
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, OkResponse{})
	})

	// @Summary the (downscaled) image of a plain-image overlay
	// @Tags imagery
	// @Router /mowglinext/imagery/{name}/image [get]
	g.GET("/imagery/:name/image", func(c *gin.Context) {
		o, ok := s.get(c.Param("name"))
		if !ok || o.Kind != "image" || o.Status != "ready" {
			c.Status(http.StatusNotFound)
			return
		}
		c.Header("Cache-Control", "public, max-age=86400")
		c.File(filepath.Join(s.root, o.Name, "display"+o.ImageExt))
	})

	// @Summary an XYZ tile of an overlay ({y} may carry an image extension)
	// @Tags imagery
	// @Router /mowglinext/tiles/{name}/{z}/{x}/{y} [get]
	g.GET("/tiles/:name/:z/:x/:y", func(c *gin.Context) { s.serveTile(c) })
}

// reorderOverlays returns overlays ordered by names; unknown names are
// ignored and overlays missing from names keep their relative order at the end.
func reorderOverlays(overlays []*imageryOverlay, names []string) []*imageryOverlay {
	out := make([]*imageryOverlay, 0, len(overlays))
	used := map[string]bool{}
	for _, n := range names {
		for _, o := range overlays {
			if o.Name == n && !used[n] {
				out = append(out, o)
				used[n] = true
			}
		}
	}
	for _, o := range overlays {
		if !used[o.Name] {
			out = append(out, o)
		}
	}
	return out
}

func (s *imageryStore) serveTile(c *gin.Context) {
	name := c.Param("name")
	if !imageryNameRe.MatchString(name) {
		c.Status(http.StatusBadRequest)
		return
	}
	z, err1 := strconv.Atoi(c.Param("z"))
	x, err2 := strconv.Atoi(c.Param("x"))
	yStr := c.Param("y")
	if i := strings.IndexByte(yStr, '.'); i >= 0 {
		yStr = yStr[:i]
	}
	y, err3 := strconv.Atoi(yStr)
	if err1 != nil || err2 != nil || err3 != nil || z < 0 || z > 30 || x < 0 || y < 0 {
		c.Status(http.StatusBadRequest)
		return
	}
	o, ok := s.get(name)
	if !ok || o.Status != "ready" {
		c.Status(http.StatusNotFound)
		return
	}
	ext := o.TileExt
	if ext == "" {
		ext = ".png"
	}
	p := filepath.Join(s.root, name, "tiles", strconv.Itoa(z), strconv.Itoa(x), strconv.Itoa(y)+ext)
	if _, err := os.Stat(p); err != nil {
		// Outside the imagery: an empty response renders as transparent and
		// keeps the browser console free of 404 noise.
		c.Header("Cache-Control", "public, max-age=86400")
		c.Status(http.StatusNoContent)
		return
	}
	c.Header("Cache-Control", "public, max-age=604800")
	if ct := mime.TypeByExtension(ext); ct != "" {
		c.Header("Content-Type", ct)
	}
	c.File(p)
}

// handleUpload streams the multipart upload to disk (no in-memory buffering)
// and starts the background conversion.
func (s *imageryStore) handleUpload(c *gin.Context) {
	c.Request.Body = http.MaxBytesReader(c.Writer, c.Request.Body, imageryMaxUploadBytes+1<<20)
	mr, err := c.Request.MultipartReader()
	if err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "expected multipart/form-data: " + err.Error()})
		return
	}
	fields := map[string]string{}
	var name, kind, ext, srcPath string
	var size int64
	for {
		part, err := mr.NextPart()
		if err == io.EOF {
			break
		}
		if err != nil {
			s.abortUpload(name)
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "upload failed: " + err.Error()})
			return
		}
		if part.FormName() != "file" {
			b, _ := io.ReadAll(io.LimitReader(part, 4096))
			fields[part.FormName()] = string(b)
			continue
		}
		if srcPath != "" {
			continue // one file per overlay
		}
		kind, ext, err = imageryKindForFile(part.FileName())
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}
		label := strings.TrimSpace(fields["label"])
		if label == "" {
			label = strings.TrimSuffix(filepath.Base(part.FileName()), filepath.Ext(part.FileName()))
		}
		s.mu.Lock()
		name = s.uniqueNameLocked(label)
		dir := filepath.Join(s.root, name)
		err = os.MkdirAll(dir, 0o755)
		s.mu.Unlock()
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		fields["label"] = label
		srcPath = filepath.Join(dir, "source"+ext)
		f, err := os.Create(srcPath)
		if err != nil {
			s.abortUpload(name)
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		size, err = io.Copy(f, io.LimitReader(part, imageryMaxUploadBytes+1))
		cerr := f.Close()
		if err == nil {
			err = cerr
		}
		if err != nil {
			s.abortUpload(name)
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "upload failed: " + err.Error()})
			return
		}
		if size > imageryMaxUploadBytes {
			s.abortUpload(name)
			c.JSON(http.StatusRequestEntityTooLarge, ErrorResponse{Error: fmt.Sprintf("file exceeds %d MB", imageryMaxUploadBytes>>20)})
			return
		}
	}
	if srcPath == "" {
		c.JSON(http.StatusBadRequest, ErrorResponse{Error: "no file in upload"})
		return
	}

	o := &imageryOverlay{
		Name: name, Label: fields["label"], Kind: kind, Status: "processing",
		Opacity: 1, Visible: true, SizeBytes: size, CreatedAt: time.Now().Unix(),
	}
	ctx, cancel := context.WithCancel(context.Background())
	s.mu.Lock()
	s.overlays = append(s.overlays, o)
	s.cancels[name] = cancel
	err = s.saveLocked()
	snapshot := *o
	s.mu.Unlock()
	if err != nil {
		c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
		return
	}
	go s.process(ctx, name, kind, srcPath, fields["scheme"])
	c.JSON(http.StatusOK, snapshot)
}

func (s *imageryStore) abortUpload(name string) {
	if name != "" {
		_ = os.RemoveAll(filepath.Join(s.root, name))
	}
}

// process converts an upload in the background and records the outcome.
func (s *imageryStore) process(ctx context.Context, name, kind, srcPath, scheme string) {
	dir := filepath.Dir(srcPath)
	progress := func(p float64) { s.setProgress(name, p) }
	var res imageryResult
	var err error
	func() {
		defer func() {
			if r := recover(); r != nil {
				err = fmt.Errorf("conversion crashed: %v", r)
			}
		}()
		switch kind {
		case "geotiff":
			res, err = processGeoTIFF(ctx, srcPath, filepath.Join(dir, "tiles"), progress)
		case "image":
			res, err = processPlainImage(srcPath, dir)
		case "xyz":
			res, err = processXYZZip(ctx, srcPath, filepath.Join(dir, "tiles"), scheme, progress)
		case "mbtiles":
			res, err = processMBTiles(ctx, srcPath, filepath.Join(dir, "tiles"), progress)
		default:
			err = fmt.Errorf("unknown kind %q", kind)
		}
	}()
	runtime.GC()
	if ctx.Err() != nil {
		return // deleted while processing
	}
	if err == nil && kind != "image" {
		_ = os.Remove(srcPath) // tiles are the product; keep disk usage down
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.cancels, name)
	_, o := s.findLocked(name)
	if o == nil {
		return
	}
	if err != nil {
		log.Printf("imagery %s: %v", name, err)
		o.Status = "error"
		o.Error = err.Error()
	} else {
		o.Status = "ready"
		o.Progress = 1
		o.MinZoom, o.MaxZoom = res.minZoom, res.maxZoom
		if res.bounds != nil {
			b := *res.bounds
			o.Bounds = &b
		}
		o.CRS, o.GSD, o.TileExt = res.crs, res.gsd, res.tileExt
		o.Width, o.Height, o.ImageExt = res.width, res.height, res.imageExt
	}
	if err := s.saveLocked(); err != nil {
		log.Printf("imagery: save overlays.json: %v", err)
	}
}

type imageryResult struct {
	minZoom, maxZoom int
	bounds           *[4]float64
	crs              string
	gsd              float64
	tileExt          string
	width, height    int
	imageExt         string
}

// ---------------------------------------------------------------------------
// GeoTIFF -> XYZ
// ---------------------------------------------------------------------------

// rasterSampler reads non-premultiplied RGBA from a decoded image.
type rasterSampler struct {
	img    image.Image
	w, h   int
	noData *float64
}

func (r *rasterSampler) px(x, y int) (cr, cg, cb, ca float64) {
	if x < 0 || y < 0 || x >= r.w || y >= r.h {
		return 0, 0, 0, 0
	}
	b := r.img.Bounds()
	x += b.Min.X
	y += b.Min.Y
	switch im := r.img.(type) {
	case *image.NRGBA:
		i := im.PixOffset(x, y)
		p := im.Pix[i : i+4 : i+4]
		cr, cg, cb, ca = float64(p[0]), float64(p[1]), float64(p[2]), float64(p[3])
	case *image.RGBA:
		i := im.PixOffset(x, y)
		p := im.Pix[i : i+4 : i+4]
		ca = float64(p[3])
		if ca == 0 {
			return 0, 0, 0, 0
		}
		cr, cg, cb = float64(p[0])*255/ca, float64(p[1])*255/ca, float64(p[2])*255/ca
	case *image.Gray:
		v := float64(im.Pix[im.PixOffset(x, y)])
		cr, cg, cb, ca = v, v, v, 255
	default:
		c := color.NRGBAModel.Convert(r.img.At(x, y)).(color.NRGBA)
		cr, cg, cb, ca = float64(c.R), float64(c.G), float64(c.B), float64(c.A)
	}
	if r.noData != nil && ca > 0 {
		nd := *r.noData
		if cr == nd && cg == nd && cb == nd {
			return 0, 0, 0, 0
		}
	}
	return
}

// bilinear samples at continuous pixel coordinates (pixel centres at i+0.5),
// blending in premultiplied space so transparent / nodata edges don't bleed.
func (r *rasterSampler) bilinear(col, row float64) color.NRGBA {
	fx, fy := col-0.5, row-0.5
	x0, y0 := int(math.Floor(fx)), int(math.Floor(fy))
	tx, ty := fx-float64(x0), fy-float64(y0)
	var sr, sg, sb, sa float64
	for j := 0; j < 2; j++ {
		for i := 0; i < 2; i++ {
			w := (1 - tx)
			if i == 1 {
				w = tx
			}
			if j == 0 {
				w *= 1 - ty
			} else {
				w *= ty
			}
			if w == 0 {
				continue
			}
			cr, cg, cb, ca := r.px(x0+i, y0+j)
			a := ca * w
			sr += cr * a
			sg += cg * a
			sb += cb * a
			sa += a
		}
	}
	if sa < 1 {
		return color.NRGBA{}
	}
	return color.NRGBA{R: clampU8(sr / sa), G: clampU8(sg / sa), B: clampU8(sb / sa), A: clampU8(sa)}
}

func clampU8(v float64) uint8 {
	if v <= 0 {
		return 0
	}
	if v >= 255 {
		return 255
	}
	return uint8(v + 0.5)
}

// renderGeoTile renders one 256x256 XYZ tile from a georeferenced raster. The
// exact tile-pixel -> source-pixel mapping is evaluated on a 17x17 grid and
// interpolated in between (error << 1 source pixel at z>=16).
func renderGeoTile(src *rasterSampler, info *geoTIFFInfo, z, tx, ty int) (*image.NRGBA, bool) {
	const step = 16
	const n = imageryTileSize/step + 1
	var gc, gr [n][n]float64
	var gok [n][n]bool
	for j := 0; j < n; j++ {
		for i := 0; i < n; i++ {
			lon, lat := tileFracToLonLat(float64(tx)+float64(i*step)/imageryTileSize, float64(ty)+float64(j*step)/imageryTileSize, z)
			x, y := info.CRS.fromLonLat(lon, lat)
			gc[j][i], gr[j][i], gok[j][i] = info.Affine.inverse(x, y)
		}
	}
	out := image.NewNRGBA(image.Rect(0, 0, imageryTileSize, imageryTileSize))
	any := false
	for py := 0; py < imageryTileSize; py++ {
		fj := (float64(py) + 0.5) / step
		j := int(fj)
		if j >= n-1 {
			j = n - 2
		}
		v := fj - float64(j)
		for px := 0; px < imageryTileSize; px++ {
			fi := (float64(px) + 0.5) / step
			i := int(fi)
			if i >= n-1 {
				i = n - 2
			}
			u := fi - float64(i)
			if !gok[j][i] {
				continue
			}
			col := (1-u)*(1-v)*gc[j][i] + u*(1-v)*gc[j][i+1] + (1-u)*v*gc[j+1][i] + u*v*gc[j+1][i+1]
			row := (1-u)*(1-v)*gr[j][i] + u*(1-v)*gr[j][i+1] + (1-u)*v*gr[j+1][i] + u*v*gr[j+1][i+1]
			if col < -1 || row < -1 || col > float64(src.w+1) || row > float64(src.h+1) {
				continue
			}
			c := src.bilinear(col, row)
			if c.A != 0 {
				out.SetNRGBA(px, py, c)
				any = true
			}
		}
	}
	return out, any
}

func writePNG(path string, img image.Image) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	enc := png.Encoder{CompressionLevel: png.BestSpeed}
	if err := enc.Encode(f, img); err != nil {
		f.Close()
		return err
	}
	return f.Close()
}

func tilePath(dir string, z, x, y int, ext string) string {
	return filepath.Join(dir, strconv.Itoa(z), strconv.Itoa(x), strconv.Itoa(y)+ext)
}

func imageryWorkers() int {
	n := runtime.NumCPU() / 2 // leave headroom for the ROS stack
	if n < 1 {
		n = 1
	}
	return n
}

func processGeoTIFF(ctx context.Context, srcPath, tileDir string, progress func(float64)) (imageryResult, error) {
	f, err := os.Open(srcPath)
	if err != nil {
		return imageryResult{}, err
	}
	defer f.Close()
	info, err := parseGeoTIFF(f)
	if err != nil {
		return imageryResult{}, err
	}
	if info.Width*info.Height > imageryMaxPixels {
		return imageryResult{}, fmt.Errorf("GeoTIFF is %dx%d (%.0f MP); the mower can tile at most %d MP. Downsample (`gdal_translate -outsize 50%% 50%% -co COMPRESS=DEFLATE in.tif out.tif`) or pre-tile with gdal2tiles and upload the .zip",
			info.Width, info.Height, float64(info.Width*info.Height)/1e6, imageryMaxPixels/1_000_000)
	}
	if _, err := f.Seek(0, io.SeekStart); err != nil {
		return imageryResult{}, err
	}
	img, err := tiff.Decode(f)
	if err != nil {
		return imageryResult{}, fmt.Errorf("decode GeoTIFF (compression %d): %v. Supported: uncompressed, LZW, Deflate, PackBits; re-export with `gdal_translate -co COMPRESS=DEFLATE in.tif out.tif`", info.Compression, err)
	}
	src := &rasterSampler{img: img, w: img.Bounds().Dx(), h: img.Bounds().Dy(), noData: info.NoData}
	bounds := info.lonLatBounds()
	gsd := info.groundResolution()
	minZ, maxZ := imageryZoomRange(gsd, (bounds[1]+bounds[3])/2)

	total := 0
	for z := minZ; z <= maxZ; z++ {
		x0, y0, x1, y1 := tileRange(bounds, z)
		total += (x1 - x0 + 1) * (y1 - y0 + 1)
	}
	var done int64
	tick := func() {
		d := atomic.AddInt64(&done, 1)
		if d%32 == 0 {
			progress(float64(d) / float64(total))
		}
	}

	// Max zoom straight from the source.
	x0, y0, x1, y1 := tileRange(bounds, maxZ)
	type job struct{ x, y int }
	jobs := make(chan job)
	var wg sync.WaitGroup
	var firstErr atomic.Value
	for w := 0; w < imageryWorkers(); w++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := range jobs {
				tile, any := renderGeoTile(src, info, maxZ, j.x, j.y)
				if any {
					if err := writePNG(tilePath(tileDir, maxZ, j.x, j.y, ".png"), tile); err != nil {
						firstErr.CompareAndSwap(nil, err)
					}
				}
				tick()
			}
		}()
	}
loop:
	for x := x0; x <= x1; x++ {
		for y := y0; y <= y1; y++ {
			select {
			case <-ctx.Done():
				break loop
			case jobs <- job{x, y}:
			}
		}
	}
	close(jobs)
	wg.Wait()
	if e, ok := firstErr.Load().(error); ok && e != nil {
		return imageryResult{}, e
	}
	if ctx.Err() != nil {
		return imageryResult{}, ctx.Err()
	}
	img, src = nil, nil
	runtime.GC()

	if err := buildPyramid(ctx, tileDir, bounds, minZ, maxZ, tick); err != nil {
		return imageryResult{}, err
	}
	return imageryResult{minZoom: minZ, maxZoom: maxZ, bounds: &bounds, crs: info.CRS.String(), gsd: gsd, tileExt: ".png"}, nil
}

// buildPyramid derives zoom levels below maxZ by 2x2 box-downsampling the
// children, which anti-aliases far better than resampling the source.
func buildPyramid(ctx context.Context, tileDir string, bounds [4]float64, minZ, maxZ int, tick func()) error {
	for z := maxZ - 1; z >= minZ; z-- {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		x0, y0, x1, y1 := tileRange(bounds, z)
		for x := x0; x <= x1; x++ {
			for y := y0; y <= y1; y++ {
				tile, any := downsampleChildren(tileDir, z, x, y)
				if any {
					if err := writePNG(tilePath(tileDir, z, x, y, ".png"), tile); err != nil {
						return err
					}
				}
				tick()
			}
		}
	}
	return nil
}

func readTile(path string) image.Image {
	f, err := os.Open(path)
	if err != nil {
		return nil
	}
	defer f.Close()
	img, _, err := image.Decode(f)
	if err != nil {
		return nil
	}
	return img
}

func downsampleChildren(tileDir string, z, x, y int) (*image.NRGBA, bool) {
	out := image.NewNRGBA(image.Rect(0, 0, imageryTileSize, imageryTileSize))
	any := false
	for dy := 0; dy < 2; dy++ {
		for dx := 0; dx < 2; dx++ {
			child := readTile(tilePath(tileDir, z+1, 2*x+dx, 2*y+dy, ".png"))
			if child == nil {
				continue
			}
			s := &rasterSampler{img: child, w: child.Bounds().Dx(), h: child.Bounds().Dy()}
			ox, oy := dx*imageryTileSize/2, dy*imageryTileSize/2
			for py := 0; py < imageryTileSize/2; py++ {
				for px := 0; px < imageryTileSize/2; px++ {
					// Centre of the 2x2 block = exact bilinear box average.
					c := s.bilinear(float64(2*px+1), float64(2*py+1))
					if c.A != 0 {
						out.SetNRGBA(ox+px, oy+py, c)
						any = true
					}
				}
			}
		}
	}
	return out, any
}

// ---------------------------------------------------------------------------
// Plain image (manual georeference in the web client)
// ---------------------------------------------------------------------------

func decodeImageFile(path string) (image.Image, string, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, "", err
	}
	defer f.Close()
	cfg, format, err := image.DecodeConfig(f)
	if err != nil {
		return nil, "", fmt.Errorf("not a readable PNG/JPEG/WebP image: %v", err)
	}
	if cfg.Width*cfg.Height > imageryMaxPixels {
		return nil, "", fmt.Errorf("image is %dx%d; at most %d MP is supported", cfg.Width, cfg.Height, imageryMaxPixels/1_000_000)
	}
	if _, err := f.Seek(0, io.SeekStart); err != nil {
		return nil, "", err
	}
	img, _, err := image.Decode(f)
	return img, format, err
}

// fitWithin scales (w,h) down so the longer side is at most max.
func fitWithin(w, h, max int) (int, int) {
	if w <= max && h <= max {
		return w, h
	}
	if w >= h {
		return max, int(math.Round(float64(h) * float64(max) / float64(w)))
	}
	return int(math.Round(float64(w) * float64(max) / float64(h))), max
}

func processPlainImage(srcPath, dir string) (imageryResult, error) {
	img, format, err := decodeImageFile(srcPath)
	if err != nil {
		return imageryResult{}, err
	}
	w, h := fitWithin(img.Bounds().Dx(), img.Bounds().Dy(), imageryDisplayMaxSide)
	var out image.Image = img
	if w != img.Bounds().Dx() || h != img.Bounds().Dy() {
		dst := image.NewNRGBA(image.Rect(0, 0, w, h))
		xdraw.BiLinear.Scale(dst, dst.Bounds(), img, img.Bounds(), xdraw.Src, nil)
		out = dst
	}
	ext := ".png"
	if format == "jpeg" {
		ext = ".jpg"
		fo, err := os.Create(filepath.Join(dir, "display.jpg"))
		if err != nil {
			return imageryResult{}, err
		}
		if err := jpeg.Encode(fo, out, &jpeg.Options{Quality: 90}); err != nil {
			fo.Close()
			return imageryResult{}, err
		}
		if err := fo.Close(); err != nil {
			return imageryResult{}, err
		}
	} else if err := writePNG(filepath.Join(dir, "display.png"), out); err != nil {
		return imageryResult{}, err
	}
	return imageryResult{width: w, height: h, imageExt: ext}, nil
}

// ---------------------------------------------------------------------------
// Pre-tiled sets: XYZ zip and MBTiles
// ---------------------------------------------------------------------------

var xyzEntryRe = regexp.MustCompile(`(?:^|/)(\d{1,2})/(\d+)/(\d+)\.(png|jpg|jpeg|webp)$`)

type tileSetStats struct {
	minZ, maxZ int
	ext        string
	xr, yr     map[int][4]int // z -> x0,y0,x1,y1
	count      int
}

func newTileSetStats() *tileSetStats {
	return &tileSetStats{minZ: 99, maxZ: -1, xr: map[int][4]int{}}
}

func (t *tileSetStats) add(z, x, y int) {
	t.count++
	if z < t.minZ {
		t.minZ = z
	}
	if z > t.maxZ {
		t.maxZ = z
	}
	r, ok := t.xr[z]
	if !ok {
		r = [4]int{x, y, x, y}
	}
	r[0], r[1] = min(r[0], x), min(r[1], y)
	r[2], r[3] = max(r[2], x), max(r[3], y)
	t.xr[z] = r
}

func (t *tileSetStats) result() (imageryResult, error) {
	if t.count == 0 {
		return imageryResult{}, errors.New("no z/x/y tiles found")
	}
	r := t.xr[t.maxZ]
	w, n := tileFracToLonLat(float64(r[0]), float64(r[1]), t.maxZ)
	e, s := tileFracToLonLat(float64(r[2]+1), float64(r[3]+1), t.maxZ)
	b := [4]float64{w, s, e, n}
	return imageryResult{minZoom: t.minZ, maxZoom: t.maxZ, bounds: &b, tileExt: t.ext}, nil
}

func processXYZZip(ctx context.Context, srcPath, tileDir, scheme string, progress func(float64)) (imageryResult, error) {
	zr, err := zip.OpenReader(srcPath)
	if err != nil {
		return imageryResult{}, fmt.Errorf("open zip: %v", err)
	}
	defer zr.Close()
	tms := scheme == "tms"
	if scheme == "" || scheme == "auto" {
		for _, f := range zr.File {
			if strings.HasSuffix(strings.ToLower(f.Name), "tilemapresource.xml") {
				tms = true // gdal2tiles default (non --xyz) output
			}
		}
	}
	st := newTileSetStats()
	for i, f := range zr.File {
		if ctx.Err() != nil {
			return imageryResult{}, ctx.Err()
		}
		m := xyzEntryRe.FindStringSubmatch(strings.ReplaceAll(f.Name, "\\", "/"))
		if m == nil || f.FileInfo().IsDir() {
			continue
		}
		z, _ := strconv.Atoi(m[1])
		x, _ := strconv.Atoi(m[2])
		y, _ := strconv.Atoi(m[3])
		if z > 30 {
			continue
		}
		ext := "." + m[4]
		if ext == ".jpeg" {
			ext = ".jpg"
		}
		if st.ext == "" {
			st.ext = ext
		} else if st.ext != ext {
			continue // mixed formats: keep the first one
		}
		if tms {
			y = tmsToXYZ(z, y)
		}
		if err := extractZipEntry(f, tilePath(tileDir, z, x, y, ext)); err != nil {
			return imageryResult{}, err
		}
		st.add(z, x, y)
		if i%64 == 0 {
			progress(float64(i) / float64(len(zr.File)))
		}
	}
	res, err := st.result()
	if err != nil {
		return res, fmt.Errorf("%v in the zip (expected <z>/<x>/<y>.png, e.g. from `gdal2tiles.py --xyz`)", err)
	}
	return res, nil
}

func extractZipEntry(f *zip.File, dst string) error {
	if f.UncompressedSize64 > 16<<20 {
		return fmt.Errorf("tile %s is implausibly large", f.Name)
	}
	rc, err := f.Open()
	if err != nil {
		return err
	}
	defer rc.Close()
	if err := os.MkdirAll(filepath.Dir(dst), 0o755); err != nil {
		return err
	}
	out, err := os.Create(dst)
	if err != nil {
		return err
	}
	if _, err := io.Copy(out, io.LimitReader(rc, 16<<20)); err != nil {
		out.Close()
		return err
	}
	return out.Close()
}

func processMBTiles(ctx context.Context, srcPath, tileDir string, progress func(float64)) (imageryResult, error) {
	db, err := sql.Open("sqlite", "file:"+srcPath+"?mode=ro&immutable=1")
	if err != nil {
		return imageryResult{}, err
	}
	defer db.Close()
	format := "png"
	_ = db.QueryRowContext(ctx, "SELECT value FROM metadata WHERE name='format'").Scan(&format)
	ext := "." + strings.ToLower(format)
	if ext == ".jpeg" {
		ext = ".jpg"
	}
	if ext != ".png" && ext != ".jpg" && ext != ".webp" {
		return imageryResult{}, fmt.Errorf("MBTiles format %q is not a raster image format", format)
	}
	var total int
	_ = db.QueryRowContext(ctx, "SELECT COUNT(*) FROM tiles").Scan(&total)
	rows, err := db.QueryContext(ctx, "SELECT zoom_level, tile_column, tile_row, tile_data FROM tiles")
	if err != nil {
		return imageryResult{}, fmt.Errorf("not a raster MBTiles file: %v", err)
	}
	defer rows.Close()
	st := newTileSetStats()
	st.ext = ext
	for rows.Next() {
		var z, x, y int
		var data []byte
		if err := rows.Scan(&z, &x, &y, &data); err != nil {
			return imageryResult{}, err
		}
		if z < 0 || z > 30 {
			continue
		}
		y = tmsToXYZ(z, y) // MBTiles rows are TMS
		p := tilePath(tileDir, z, x, y, ext)
		if err := os.MkdirAll(filepath.Dir(p), 0o755); err != nil {
			return imageryResult{}, err
		}
		if err := os.WriteFile(p, data, 0o644); err != nil {
			return imageryResult{}, err
		}
		st.add(z, x, y)
		if total > 0 && st.count%64 == 0 {
			progress(float64(st.count) / float64(total))
		}
	}
	if err := rows.Err(); err != nil {
		return imageryResult{}, err
	}
	return st.result()
}
