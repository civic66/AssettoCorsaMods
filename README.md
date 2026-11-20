# AssettoCorsaMods

## trackgen: real-world road loops for Assetto Corsa

`trackgen` turns a loop of real roads into an Assetto Corsa track. You give it
a list of lat/lon waypoints; it

1. downloads the road network from **OpenStreetMap** and routes a loop along real roads through your waypoints,
2. downloads real **terrain elevation** (AWS Terrain Tiles: SRTM / USGS 3DEP / EU-DEM…),
3. smooths the centreline and builds a road ribbon, grass shoulders, terrain that blends into the road, and barriers,
4. places the AC spawn/timing objects (grid, pits, hotlap start, start/finish line),
5. writes the AC track folder (`ui_track.json`, preview, outline, minimap + `map.ini`, `surfaces.ini`).

Then a Blender script turns it into an FBX, and Kunos' **ksEditor** turns that into the `.kn5` the game loads.

> **Why not Google Maps data?** Google's terms forbid extracting its map, satellite
> and 3D data for use in other products, so a mod built from it can't be shared.
> OpenStreetMap + public elevation data cover the same places and can be redistributed
> (with attribution, see below). Using Google Maps to *look* at a place and copy
> waypoint coordinates is fine.

### Requirements

* Python 3.9+ and `pip install -r requirements.txt`
* [Blender](https://www.blender.org/) 3.6 or newer (tested with 4.2 LTS)
* Assetto Corsa with the **AC SDK** (Steam → Library → Tools → *Assetto Corsa SDK*), which contains ksEditor

### 1. Describe the track

Copy `tracks/example/` to `tracks/<your_track>/` and edit `track.yaml`:

```yaml
name: my_loop                 # AC folder name: lowercase, digits, underscores
display_name: My Loop
waypoints:                    # [lat, lon] in driving order; the loop closes itself
  - [37.74691, -122.45072]
  - [37.75125, -122.45213]
  - ...
```

To get coordinates, right-click a spot in Google Maps or on openstreetmap.org and copy
the lat/lon. Put a waypoint on every road you want to use. If the route takes a
shortcut you don't want, add another waypoint on the road you meant.

All other settings are optional: road width, shoulders, barrier distance, smoothing,
terrain size and `reverse: true` to drive the loop the other way. The example file lists them all.

### 2. Generate

```bash
python -m trackgen tracks/<your_track>/track.yaml
```

Output goes to `output/<name>/`:

```
output/<name>/
  source/
    route_check.png      <- LOOK AT THIS FIRST: OSM roads (grey), your loop (red), waypoints (blue)
    track.obj/.mtl       road, shoulders, terrain, barriers, start line
    textures/            placeholder asphalt/grass/barrier textures
    markers.json         AC_START_n, AC_PIT_n, AC_HOTLAP_START_0, AC_TIME_0_L/R
    centerline.csv
  content/tracks/<name>/ the folder that goes into Assetto Corsa
    ui/ui_track.json, preview.png, outline.png
    data/surfaces.ini, map.ini
    map.png
```

The console prints the lap length, elevation range, max grade and the tightest corner
radius, and warns about corners too tight for the road width.
Fix those with more `smoothing.xy_m`, a narrower `road.width`, or by moving waypoints.

Downloads are cached in `~/.cache/trackgen`, so later runs are fast. If the Overpass servers
are busy, the tool falls back to the main OSM API automatically
(`TRACKGEN_OSM_SOURCE=api` skips Overpass entirely).

### 3. Blender → FBX

```bash
blender --background --python blender/import_track.py -- output/<name>/source
```

This imports the OBJ, creates the AC empties from `markers.json`, saves `track.blend`
and exports `source/fbx/track.fbx` with AC-friendly settings (forward −Z, up Y, textures
copied next to it). You can also open `track.blend` and add buildings, trees, kerbs and so on
before exporting.

### 4. ksEditor → KN5

1. Open ksEditor, then *File → Import FBX* → `source/fbx/track.fbx`.
2. In the *Materials* panel, give each material a shader. `ksPerPixel` is fine to start with
   (set `txDiffuse` to the texture). Use `ksPerPixelMultiMap` later for nicer asphalt.
3. *File → Save KN5* as `content/tracks/<name>/<name>.kn5`.
4. Copy `output/<name>/content/tracks/<name>` into `<Assetto Corsa>/content/tracks/`.

Load it in Practice / Hotlap. You can drive without an AI line, but you need one
for AI opponents and the racing-line assist. To record it, enable *Developer apps* in
Content Manager (Settings → Assetto Corsa → System), drive one clean lap with the
**AI** app recording, and save it. That creates `ai/fast_lane.ai` in the track folder.

### Naming conventions used (what makes it drivable)

| Object             | Meaning in AC                                              |
|--------------------|------------------------------------------------------------|
| `1ROAD` …          | physical asphalt (`ROAD` surface, built into AC)           |
| `1GRASS` …         | physical grass (shoulders + terrain)                       |
| `1BARRIER` …       | physical wall, defined in the track's `data/surfaces.ini`  |
| `start_line`       | visual only                                                |
| `AC_START_n`       | grid slots, `AC_PIT_n` pit boxes, `AC_HOTLAP_START_0`      |
| `AC_TIME_0_L/R`    | start/finish line gates                                    |

Meshes are split so that none has more than 60k vertices, because KN5 uses 16-bit indices.
Coordinates are metres, with X = east, Z = −north and Y = up in game. The lowest point of the road is at Y ≈ 0.

### Known limitations / next steps

* Textures are procedural placeholders, and there are no buildings, trees or road markings beyond the edge lines.
* Pit boxes are on the track behind the grid; there's no separate pit lane.
* One-way tags are ignored when routing. If a road is split into two carriageways in OSM, the loop uses one of them.
* If the loop crosses itself (a bridge or junction), the geometry there needs fixing by hand in Blender.
* Elevation comes from ~5–30 m terrain models: good for the shape of hills, not for road crowns or kerbs.
* If cars spawn facing backwards, set `EXTRA_YAW_DEG = 180` in `blender/import_track.py`.

### Data licences / attribution

Put this in your mod's description and readme when you share it:

* Road data © OpenStreetMap contributors, available under the
  [Open Database Licence](https://www.openstreetmap.org/copyright).
* Elevation: [AWS Terrain Tiles](https://registry.opendata.aws/terrain-tiles/). The sources
  include SRTM, USGS 3DEP/NED, GMTED, ETOPO1, EU-DEM (© European Union, Copernicus), and others,
  [attribution list](https://github.com/tilezen/joerd/blob/master/docs/attribution.md).
