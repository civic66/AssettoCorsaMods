# AssettoCorsaMods: working notes for Claude

This repo holds **trackgen**, a Python tool that turns real roads (OpenStreetMap +
public elevation data) into ready-to-play Assetto Corsa tracks, including the `.kn5`.
See `README.md` for what it does and every config option.

## How we work

- **Change the tool, not the output.** Features and fixes go into `trackgen/` (and the
  track configs / docs). Never hand-edit a generated track to fix something.
- **Test on the AR-12 track only:** `tracks/ar12_gentry/track.yaml`. Don't build or
  test the Twin Peaks example (`tracks/example/`) unless asked.
- **Rebuild / install AR-12 only when asked.** Building it to check a change is fine;
  copying it into the game folder or `ready_tracks/` happens only on request.
- **Never commit `.kn5` files** (they are 40-50 MB each). Build them locally instead.
- Keep `README.md` and `tracks/example/track.yaml` (which documents every option and
  its default) up to date when adding or changing settings.

## Commands (Windows, run from the repo folder)

```powershell
py -m pip install -r requirements.txt
py -m trackgen tracks/ar12_gentry/track.yaml                # build into output\ar12_gentry\
py -m trackgen tracks/ar12_gentry/track.yaml --install "C:\Program Files (x86)\Steam\steamapps\common\assettocorsa\content\tracks"
```

Build output: `output/ar12_gentry/content/tracks/ar12_gentry/` is the game folder (with
`ar12_gentry.kn5`); `output/ar12_gentry/source/` has `route_check.png`, `track.obj`,
`markers.json` and `centerline.csv` for checking results. Map/elevation downloads are
cached in `~/.cache/trackgen`.

## Code map

| Area | File |
|---|---|
| Config defaults, pipeline, materials | `trackgen/__main__.py` |
| Road, paved shoulders, grass, terrain, kerbs, brake boards, walls, AC markers | `trackgen/build.py` |
| OSM download (Overpass, falls back to the OSM API) and routing | `trackgen/osm.py` |
| Elevation (AWS Terrain Tiles) | `trackgen/elevation.py` |
| Centreline smoothing / curvature | `trackgen/centerline.py` |
| `.kn5` writer, plus `read_kn5_summary()` to parse one back for checks | `trackgen/kn5.py` |
| Textures, minimap, preview, ui_track.json | `trackgen/package.py` |

## Checking a change

The game can't run in CI, so verify by parsing the built `.kn5` with
`trackgen.kn5.read_kn5_summary()` and checking geometry numerically (face directions,
heights, distances, object names), and look at `route_check.png` / the `.obj`.
Say clearly what was verified this way and what still needs an in-game test.

## Assetto Corsa conventions this tool relies on

- Physical surfaces by object name: `<n>ROAD`, `<n>GRASS`, `<n>KERB` (built-in surfaces),
  `<n>WALL` for collision walls (faces must point at the road). Other names are visual only.
- Spawn/timing empties: `AC_START_n`, `AC_PIT_n`, `AC_HOTLAP_START_0`, circuits use
  `AC_TIME_0_L/R`, point-to-point uses `AC_AB_START_L/R` and `AC_AB_FINISH_L/R`.
- KN5 meshes are limited to 65,535 vertices (the tool splits at 60,000).
- kn5 space is Y-up: (x, y, z) = (east, up, -north).
