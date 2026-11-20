"""python -m trackgen tracks/<name>/track.yaml [-o output]"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
import yaml

from . import centerline as cl
from .build import TrackGeometry
from .elevation import ElevationSampler
from .geo import LocalProjection
from .mesh import write_mtl, write_obj
from .osm import RoadGraph, fetch_roads, route_loop
from .package import SURFACES_INI, write_map, write_route_debug, write_outline, write_preview, write_textures, write_ui

DEFAULTS = {
    "display_name": None,
    "reverse": False,
    "road": {"width": 8.0, "shoulder": 3.0, "wall_offset": 6.0, "wall_height": 1.2},
    "smoothing": {"xy_m": 4.0, "z_m": 25.0, "step_m": 2.0},
    "terrain": {"margin_m": 200.0, "grid_m": 3.0, "blend_m": 30.0},
    "elevation_zoom": 14,
    "start_offset_m": "auto",
    "grid_slots": 12,
    "pit_boxes": 12,
    "osm": {"margin_m": 400.0, "highways": None, "reuse_penalty": 4.0},
}


def merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = v
    return out


def main():
    ap = argparse.ArgumentParser(description="Generate an Assetto Corsa track from OSM + elevation data")
    ap.add_argument("config", type=Path)
    ap.add_argument("-o", "--output", type=Path, default=Path("output"))
    args = ap.parse_args()

    cfg = merge(DEFAULTS, yaml.safe_load(args.config.read_text()))
    name = cfg["name"]
    if not name.replace("_", "").isalnum() or name.lower() != name:
        raise SystemExit("'name' must be lowercase letters/digits/underscores (it becomes the AC folder name)")

    out = args.output / name
    src_dir = out / "source"
    track_dir = out / "content" / "tracks" / name
    for d in (src_dir / "textures", track_dir / "ui", track_dir / "data"):
        d.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------ route
    if "waypoints" in cfg:
        wps = np.array(cfg["waypoints"], dtype=float)
    else:
        raise SystemExit("config needs 'waypoints': a list of [lat, lon] in driving order")
    proj = LocalProjection(wps[:, 0].mean(), wps[:, 1].mean())
    wxy = np.column_stack(proj.to_xy(wps[:, 0], wps[:, 1]))

    if cfg.get("follow_roads", True):
        m = cfg["osm"]["margin_m"]
        lat_lo, lon_lo = proj.to_latlon(wxy[:, 0].min() - m, wxy[:, 1].min() - m)
        lat_hi, lon_hi = proj.to_latlon(wxy[:, 0].max() + m, wxy[:, 1].max() + m)
        print("Fetching OpenStreetMap roads...")
        osm = fetch_roads(float(lat_lo), float(lon_lo), float(lat_hi), float(lon_hi), cfg["osm"]["highways"])
        graph = RoadGraph(osm, proj)
        print("Routing loop through waypoints...")
        route_xy, _ = route_loop(graph, wxy, closed=True, reuse_penalty=cfg["osm"]["reuse_penalty"])
        write_route_debug(src_dir / "route_check.png", graph, route_xy, wxy)
        print(f"  route preview: {src_dir / 'route_check.png'}")
    else:
        route_xy = wxy  # waypoints are the centreline itself (e.g. traced from a GPX)
    if cfg["reverse"]:
        route_xy = route_xy[::-1]

    sm = cfg["smoothing"]
    xy = cl.build_centerline(route_xy, sm["step_m"], sm["xy_m"])
    clockwise = float(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1) - np.roll(xy[:, 0], -1) * xy[:, 1])) < 0
    cfg["_clockwise"] = clockwise

    # ------------------------------------------------ elevation
    tm = cfg["terrain"]["margin_m"] + 50
    lat_lo, lon_lo = proj.to_latlon(xy[:, 0].min() - tm, xy[:, 1].min() - tm)
    lat_hi, lon_hi = proj.to_latlon(xy[:, 0].max() + tm, xy[:, 1].max() + tm)
    print("Fetching elevation tiles...")
    elev = ElevationSampler(float(lat_lo), float(lon_lo), float(lat_hi), float(lon_hi), cfg["elevation_zoom"])

    def dem_fn(x, y):
        la, lo = proj.to_latlon(x, y)
        return elev.sample(la, lo)

    raw_z = dem_fn(xy[:, 0], xy[:, 1])
    z = cl.fit_heights(raw_z, sm["step_m"], sm["z_m"])
    zero = float(z.min())  # put the lowest point of the road near y = 0
    z -= zero

    def dem_rel(x, y):
        return dem_fn(x, y) - zero

    # ------------------------------------------------ geometry
    print("Building geometry...")
    geo = TrackGeometry(xy, z, cfg)
    geo.report()
    meshes = geo.road_meshes()
    meshes += geo.shoulder_meshes(dem_rel, start_num=1)
    n_sh = len(meshes) - len(geo.road_meshes())
    meshes += geo.terrain_meshes(dem_rel, start_num=1 + n_sh)
    meshes += geo.barrier_meshes(dem_rel, start_num=1)
    start = cfg["start_offset_m"]
    if start == "auto":
        start = geo.auto_start(cfg["grid_slots"], cfg["pit_boxes"])
        print(f"  start/finish placed {start:.0f} m into the lap (straightest run-up)")
    markers, start_i = geo.markers(float(start), cfg["grid_slots"], cfg["pit_boxes"])
    meshes.append(geo.start_line_mesh(start_i))

    write_textures(src_dir / "textures")
    write_mtl(src_dir / "track.mtl", {"asphalt": "textures/asphalt.png", "grass": "textures/grass.png",
                                      "barrier": "textures/barrier.png", "line": "textures/line.png"})
    write_obj(meshes, src_dir / "track.obj", "track.mtl")
    (src_dir / "markers.json").write_text(json.dumps({
        "frame": "blender (x=east, y=north, z=up, metres)",
        "markers": markers}, indent=2))
    lat0, lon0 = proj.to_latlon(*xy[start_i])
    (src_dir / "centerline.csv").write_text(
        "x_east,y_north,z_up\n" + "\n".join(f"{a:.3f},{b:.3f},{c:.3f}" for (a, b), c in zip(xy, z)))

    # ------------------------------------------------ AC folder
    (track_dir / "data" / "surfaces.ini").write_text(SURFACES_INI)
    write_map(track_dir, xy)
    write_outline(track_dir / "ui" / "outline.png", xy, start_i)
    write_preview(track_dir / "ui" / "preview.png", geo.terrain, xy)
    write_ui(track_dir, cfg, geo.length, cfg["pit_boxes"], (float(lat0), float(lon0)))

    nverts = sum(len(m.vertices) for m in meshes)
    print(f"Done: {len(meshes)} objects, {nverts:,} vertices")
    print(f"  Blender source : {src_dir}")
    print(f"  AC track folder: {track_dir}")


if __name__ == "__main__":
    main()
