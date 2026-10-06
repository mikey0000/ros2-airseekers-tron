# Robot model images

The Hardware settings model picker shows one image per preset, loaded from
`/robots/<file stem>.<ext>`. For each card the GUI tries, in order:

1. `<file stem>.jpg` (your photo, optional)
2. `<file stem>.svg` (bundled illustration)
3. no image (the card still works)

## Using a photo of your robot

Drop a photo into this directory named after the file stem, for example
`airseekers_tron.jpg` for the Airseekers Tron (the other presets use their model
id, e.g. `YardForce500.jpg`). A landscape or square image around 400x300 px works
best; it is scaled to fit a 72 px high slot. Rebuild (or copy it into the served
`web/dist/robots/`) and reload; nothing else needs changing. Delete the file to
fall back to the bundled SVG.

The bundled SVGs are placeholders: `airseekers_tron.svg` is a schematic of the
Tron, the others are generic mower silhouettes.
