# Custom map imagery (drone orthomosaics)

The Map page can draw your own imagery above the Mapbox satellite base: a
high-resolution drone orthomosaic, a pre-tiled XYZ/MBTiles set, or a plain
photo you align by hand against the robot's RTK position.

Where to find it:

- **Desktop:** Map page, right-hand card, **Imagery** section (under Map Offset).
- **Mobile:** Map page, `…` menu, **Imagery** (opens a bottom sheet).

Each overlay has a visibility switch, an opacity slider, up/down buttons
(the top of the list draws on top), zoom-to, delete, and for plain images
**Align**. Overlays always draw above the satellite base and below the areas,
paths and robot. The list is saved on the robot and survives reloads and
restarts.

## What you can upload (max 300 MB per file)

| File | What happens on the robot |
|------|---------------------------|
| `.tif` / `.tiff` GeoTIFF (EPSG:4326, EPSG:3857, or UTM: WGS84 `326xx`/`327xx`, ETRS89 `258xx`, NAD83 `269xx`) | Tiled in pure Go (the GUI image has no GDAL) into XYZ PNG tiles from z16 up to the zoom that matches the image resolution (z18–z23). |
| `.zip` of an XYZ/TMS tile folder (`<z>/<x>/<y>.png\|jpg\|webp`) | Unpacked as-is. TMS (gdal2tiles default) is detected from `tilemapresource.xml`. You can also force XYZ or TMS with the "Tile .zip rows" selector. |
| `.mbtiles` (raster png/jpg/webp) | Unpacked to tiles. MBTiles rows are TMS and get flipped automatically. |
| `.png` / `.jpg` / `.webp` (no georeference) | Downscaled to at most 4096 px on its long side and placed in the robot map frame with the **Align** tool (see below). |

GeoTIFF limits on the robot: classic TIFF (not BigTIFF), compression none,
LZW, Deflate or PackBits (JPEG-compressed TIFFs are **not** supported), and at
most 100 megapixels, because the mower has about 1 GB of free RAM. Tiling
uses half the CPU cores so the ROS stack keeps its headroom. Converting
while the robot is mowing works, but it's better done while the robot is docked.

If your GeoTIFF does not fit those limits, convert or pre-tile it on a PC
with GDAL:

```bash
# Re-compress (JPEG-compressed or BigTIFF exports), keep the georeference
gdal_translate -co COMPRESS=DEFLATE -co BIGTIFF=NO -co TILED=YES in.tif out.tif

# Too many pixels: halve the resolution
gdal_translate -outsize 50% 50% -co COMPRESS=DEFLATE in.tif out.tif

# Unsupported CRS: reproject to web mercator
gdalwarp -t_srs EPSG:3857 -r bilinear -co COMPRESS=DEFLATE in.tif out.tif

# Or skip on-robot tiling entirely: XYZ tiles (GDAL >= 3.1), zip, upload the zip
gdal2tiles.py --xyz -z 16-23 -w none --processes=8 in.tif tiles/
(cd tiles && zip -r ../garden_tiles.zip .)

# MBTiles from GDAL
gdal_translate -of MBTILES in.tif garden.mbtiles && gdaladdo -r average garden.mbtiles 2 4 8 16
```

## Drone workflow

1. **Fly.** Fly a mapping grid (70–80 % front and side overlap) at 20–40 m.
   That gives roughly 0.5–1.5 cm/px with a typical 1" sensor drone. Fly
   around midday on a cloudy or bright day so shadows are short. Put 2–4
   easy-to-spot ground marks (white plates, chalk crosses) somewhere the robot
   can drive. They make the alignment step much easier.
2. **Process into an orthomosaic:**
   - **WebODM / ODM:** "Default" or "High Resolution" preset. Download
     *Orthophoto → GeoTIFF* (`odm_orthophoto.tif`, UTM, Deflate, with alpha:
     upload it directly).
   - **Pix4D (Mapper/Matic):** Export → Orthomosaic → GeoTIFF
     (`*_transparent_mosaic_group1.tif`). Pick "Merged" and a UTM or WGS84
     output CRS. If it was written as BigTIFF or with JPEG compression, run the
     `gdal_translate` line above.
   - **DroneDeploy:** Export → Orthomosaic → GeoTIFF, projection "Web
     Mercator (EPSG:3857)" or "UTM", resolution "Native" (or a coarser value if
     it comes out over 100 MP). DroneDeploy can also export *Tiles (XYZ/MBTiles)*:
     upload the `.mbtiles` or the zipped tile folder.
   - **QGIS:** Processing → "Generate XYZ tiles (Directory)" (zip the folder)
     or "Generate XYZ tiles (MBTiles)".
3. **Upload** on the Map page (drag-and-drop or click). The overlay shows
   "Converting" with progress, then appears on the map.
4. **Check the fit.** Drone GNSS without RTK/GCPs is often off by 1–3 m. The
   Mapbox base, the drone image and the robot's RTK map can all disagree. The
   robot's RTK frame is the ground truth that matters for mowing. If the
   orthomosaic is visibly offset from the robot track or areas, either
   reprocess with ground control points, or export it as a plain PNG/JPEG
   (`gdal_translate -of PNG in.tif out.png`) and use the manual alignment
   below. That ties it to the robot's own RTK map.

## Manual alignment (plain images): pin the image to RTK ground truth

A plain image is placed in the **robot map frame** (the same metres frame
the areas, dock and `/pose` use) with a similarity transform: centre,
rotation and pixel size. It is then drawn through the same projection and
**Map Offset** as the areas, so it stays registered to the robot even if you
change the offset or the Mapbox base is off.

1. Upload the image, then press **Align** (target icon) on it. The image
   appears at the centre of the view, 3 cm/px, north-up and semi-transparent.
2. Rough placement (optional): drag the **orange round handle** (centre) to
   move the image, and the **green square handle** (top-right corner) to rotate
   and scale it. The sliders and number fields give fine control over
   rotation, pixel size (cm/px), X/Y (m) and the preview opacity.
3. Exact placement with two control points, **A** and **B**. Pick two
   features that are sharp in the image and as far apart as possible (opposite
   corners of the lawn, the dock, a ground mark, a paving-stone corner):
   - Press **On image** for A, then click that feature on the image. An orange
     square "A" marks it.
   - Drive the robot (manual mode, with the usual care) so its centre is over
     that feature and wait for **RTK Fixed**. Then press **Robot**. The live RTK
     pose becomes A's true position (green round "A"). The panel warns when
     pose accuracy is worse than 5 cm.
     If you can't drive there, press the pin button and click the true
     location on the map instead (less accurate: it relies on the base map).
   - Repeat for B.
   - As soon as A and B each have an image point and a true position, the
     placement is solved exactly (translation, rotation and uniform scale; no
     shear or mirroring). All markers stay draggable for touch-ups.
4. Press **Save**. The placement and control points are stored with the
   overlay. **Cancel** discards the session.

Tips: control points more than 20 m apart give sub-0.1° rotation error from
~2 cm RTK. The robot's reference point is `base_link` (between the drive
wheels), not the front of the deck. Lens distortion and perspective in a
single non-orthorectified photo can't be removed by a similarity transform,
so use an orthomosaic when you can.

## Storage, API and limits

- Files live on the mower in `/userdata/ros2/map_tiles/` (mounted into
  `mower_gui` as `/map_tiles`, env `MAP_TILES_DIR`): `overlays.json` (the
  list with order, opacity, visibility and placement), and one folder per
  overlay holding `tiles/<z>/<x>/<y>.png` (and `display.png|jpg` plus
  `source.*` for plain images). The uploaded GeoTIFF/zip/mbtiles is deleted
  once it has been tiled.
- API (GUI backend, port 4006):
  - `GET /api/mowglinext/imagery`: list overlays, bottom first.
  - `POST /api/mowglinext/imagery`: multipart upload with fields `label`,
    `scheme` (`auto|xyz|tms`, for zips) and `file` (last).
  - `PATCH /api/mowglinext/imagery/<name>`: update `label`, `opacity`,
    `visible`, `placement`, `controlPoints`.
  - `POST /api/mowglinext/imagery-order`: reorder with `{"names":[...]}`,
    bottom first.
  - `DELETE /api/mowglinext/imagery/<name>`
  - `GET /api/mowglinext/tiles/<name>/{z}/{x}/{y}.png`: cached for 7 days.
    Tiles outside the imagery return `204`.
  - `GET /api/mowglinext/imagery/<name>/image`: the plain image.
- Restarting the GUI while a conversion runs marks that overlay as an error.
  Delete it and upload again.
- Disk use is small: a 2 cm/px orthophoto of a 60 × 60 m garden tiles to
  z22, which is a few hundred PNG tiles (tens of MB).
