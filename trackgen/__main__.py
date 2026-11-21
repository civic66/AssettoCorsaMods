"""python -m trackgen tracks/<name>/track.yaml [-o output] [--install DIR]"""
import argparse
import copy
import json
import shutil
from pathlib import Path

import numpy as np
import yaml

from . import centerline as cl
from .build import TrackGeometry
from .elevation import ElevationSampler
from .geo import LocalProjection
from .kn5 import write_kn5
from .mesh import write_mtl, write_obj
from .osm import RoadGraph, fetch_roads, route_loop, route_open
from .package import (write_map, write_outline, write_preview, write_route_debug,
                      write_marker_textures, write_textures, write_ui)

DEFAULTS = {
    "display_name": None,
    "type": "loop",               # loop | point_to_point
    "reverse": False,
    "road": {"width": 8.0, "shoulder": 3.0, "wall_offset": 6.0, "wall_height": 1.2},
    # red/white rumble kerbs on both sides of corners tighter than max_radius_m
    # red/white rumble kerbs along the whole of every corner (+ extend_m each end).
    # A corner = a stretch turning tighter than extent_radius_m whose tightest point is
    # under max_radius_m and that turns at least min_turn_deg in total.
    "kerbs": {"enabled": True, "max_radius_m": 150.0, "min_turn_deg": 15.0,
              "extent_radius_m": 1000.0, "extend_m": 10.0, "merge_gap_m": 15.0,
              "width_m": 1.0, "height_m": 0.05, "stripe_m": 1.0, "min_length_m": 10.0,
              "smooth_m": 6.0},
    # foam brake-marker boards before tighter corners, on the outside of the approach
    "brake_markers": {"enabled": True, "distances_m": [200, 150, 100, 50],
                      "max_radius_m": 80.0, "min_turn_deg": 30.0,
                      "size_m": 1.0, "thickness_m": 0.12, "offset_from_kerb_m": 1.0,
                      # what the distances count to: "turn_in" (estimated) or "corner_start"
                      "zero_point": "turn_in", "turn_in_fraction": 0.5, "zero_offset_m": 0.0},
    "smoothing": {"xy_m": 4.0, "z_m": 25.0, "step_m": 2.0},
    "terrain": {"margin_m": 200.0, "grid_m": 3.0, "blend_m": 30.0},
    "elevation_zoom": 14,
    "start_offset_m": "auto",     # loops only
    "runoff_m": 80.0,             # point-to-point: road after the finish gate
    "grid_slots": 12,
    "pit_boxes": 12,
    "osm": {"margin_m": 400.0, "highways": None, "reuse_penalty": 4.0,
            "corridor_m": 2500.0, "prefer_refs": None, "other_road_cost": 3.0},
}

MATERIALS = {
    "asphalt": {"texture": "asphalt.png", "props": {"ksSpecular": 0.08, "ksSpecularEXP": 25.0}},
    "grass": {"texture": "grass.png", "props": {}},
    "barrier": {"texture": "barrier.png", "props": {"ksSpecular": 0.1, "ksSpecularEXP": 15.0}},
    "line": {"texture": "line.png", "props": {"ksAmbient": 0.55, "ksDiffuse": 0.5}},
    "kerb": {"texture": "kerb.png", "props": {"ksSpecular": 0.1, "ksSpecularEXP": 20.0}},
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
    ap.add_argument("--install", type=Path, default=None,
                    help="also copy the finished track folder here (e.g. .../assettocorsa/content/tracks)")
    args = ap.parse_args()

    cfg = merge(DEFAULTS, yaml.safe_load(args.config.read_text()))
    name = cfg["name"]
    if not name.replace("_", "").isalnum() or name.lower() != name:
        raise SystemExit("'name' must be lowercase letters/digits/underscores (it becomes the AC folder name)")
    closed = cfg["type"] != "point_to_point"

    out = args.output / name
    src_dir = out / "source"
    track_dir = out / "content" / "tracks" / name
    for d in (src_dir / "textures", track_dir / "ui", track_dir / "data"):
        d.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------ route
    if "waypoints" not in cfg:
        raise SystemExit("config needs 'waypoints': a list of [lat, lon] in driving order")
    wps = np.array(cfg["waypoints"], dtype=float)
    proj = LocalProjection(wps[:, 0].mean(), wps[:, 1].mean())
    wxy = np.column_stack(proj.to_xy(wps[:, 0], wps[:, 1]))

    if cfg.get("follow_roads", True):
        oc = cfg["osm"]
        m = oc["margin_m"] if closed else max(oc["margin_m"], oc["corridor_m"])
        lat_lo, lon_lo = proj.to_latlon(wxy[:, 0].min() - m, wxy[:, 1].min() - m)
        lat_hi, lon_hi = proj.to_latlon(wxy[:, 0].max() + m, wxy[:, 1].max() + m)
        corridor = [tuple(p) for p in wps.tolist()] + ([tuple(wps[0])] if closed else [])
        print("Fetching OpenStreetMap roads...")
        osm = fetch_roads(float(lat_lo), float(lon_lo), float(lat_hi), float(lon_hi), oc["highways"],
                          corridor=corridor, buffer_deg=oc["corridor_m"] / 111000.0)
        graph = RoadGraph(osm, proj, oc["prefer_refs"], oc["other_road_cost"])
        print("Routing through waypoints...")
        if closed:
            route_xy, _ = route_loop(graph, wxy, closed=True, reuse_penalty=oc["reuse_penalty"])
        else:
            route_xy, _ = route_open(graph, wxy)
        write_route_debug(src_dir / "route_check.png", graph, route_xy, wxy, closed=closed)
        print(f"  route preview: {src_dir / 'route_check.png'}")
    else:
        route_xy = wxy  # waypoints are the centreline itself (e.g. traced from a GPX)
    if cfg["reverse"]:
        route_xy = route_xy[::-1]

    sm = cfg["smoothing"]
    xy = cl.build_centerline(route_xy, sm["step_m"], sm["xy_m"], closed=closed)
    clockwise = float(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1) - np.roll(xy[:, 0], -1) * xy[:, 1])) < 0
    cfg["_clockwise"] = clockwise

    # ------------------------------------------------ elevation
    tm = max(cfg["terrain"]["margin_m"], 150.0) + 50
    lat_lo, lon_lo = proj.to_latlon(xy[:, 0].min() - tm, xy[:, 1].min() - tm)
    lat_hi, lon_hi = proj.to_latlon(xy[:, 0].max() + tm, xy[:, 1].max() + tm)
    print("Fetching elevation tiles...")
    elev = ElevationSampler(float(lat_lo), float(lon_lo), float(lat_hi), float(lon_hi), cfg["elevation_zoom"])

    def dem_fn(x, y):
        la, lo = proj.to_latlon(x, y)
        return elev.sample(la, lo)

    raw_z = dem_fn(xy[:, 0], xy[:, 1])
    z = cl.fit_heights(raw_z, sm["step_m"], sm["z_m"], closed=closed)
    zero = float(z.min())  # put the lowest point of the road near y = 0
    z -= zero

    def dem_rel(x, y):
        return dem_fn(x, y) - zero

    # ------------------------------------------------ geometry
    print("Building geometry...")
    geo = TrackGeometry(xy, z, cfg, closed=closed)
    geo.report()
    meshes = geo.road_meshes()
    shoulders = geo.shoulder_meshes(start_num=1)
    meshes += shoulders
    meshes += geo.terrain_meshes(dem_rel, start_num=1 + len(shoulders))
    meshes += geo.kerb_meshes(start_num=1)
    board_meshes, _ = geo.brake_marker_meshes(dem_rel)
    meshes += board_meshes
    meshes += geo.barrier_meshes(dem_rel, start_num=1)
    if closed:
        start = cfg["start_offset_m"]
        if start == "auto":
            start = geo.auto_start(cfg["grid_slots"], cfg["pit_boxes"])
            print(f"  start/finish placed {start:.0f} m into the lap (straightest run-up)")
        markers, gates = geo.markers(float(start), cfg["grid_slots"], cfg["pit_boxes"])
        meshes.append(geo.line_mesh("start_line", gates[0]))
    else:
        markers, gates = geo.markers_ab(cfg["grid_slots"], cfg["pit_boxes"], cfg["runoff_m"])
        meshes.append(geo.line_mesh("start_line", gates[0]))
        meshes.append(geo.line_mesh("finish_line", gates[1]))
        print(f"  start gate at {gates[0]:.0f} m, finish gate at {gates[1]:.0f} m "
              f"(timed distance {gates[1] - gates[0]:.0f} m)")
    gate_idx = [geo.index_at(g) for g in gates]

    # ------------------------------------------------ source files (Blender route)
    materials = dict(MATERIALS)
    write_textures(src_dir / "textures")
    if board_meshes:
        dists = cfg["brake_markers"]["distances_m"]
        write_marker_textures(src_dir / "textures", dists)
        board_props = {"ksAmbient": 0.5, "ksDiffuse": 0.5, "ksSpecular": 0.02, "ksSpecularEXP": 5.0}
        materials["foam"] = {"texture": "foam.png", "props": board_props}
        for d in dists:
            materials[f"marker_{d}"] = {"texture": f"marker_{d}.png", "props": board_props}
    write_mtl(src_dir / "track.mtl", {k: f"textures/{v['texture']}" for k, v in materials.items()})
    write_obj(meshes, src_dir / "track.obj", "track.mtl")
    (src_dir / "markers.json").write_text(json.dumps({
        "frame": "blender (x=east, y=north, z=up, metres)",
        "markers": markers}, indent=2))
    (src_dir / "centerline.csv").write_text(
        "x_east,y_north,z_up\n" + "\n".join(f"{a:.3f},{b:.3f},{c:.3f}" for (a, b), c in zip(xy, z)))

    # ------------------------------------------------ AC folder (ready to play)
    textures = {v["texture"]: (src_dir / "textures" / v["texture"]).read_bytes() for v in materials.values()}
    write_kn5(track_dir / f"{name}.kn5", name, meshes, markers, materials, textures)
    # ROAD, GRASS and WALL are built-in AC surfaces, so no data/surfaces.ini is needed
    write_map(track_dir, xy, closed=closed)
    write_outline(track_dir / "ui" / "outline.png", xy, gate_idx, closed=closed)
    write_preview(track_dir / "ui" / "preview.png", dem_rel, xy, closed=closed)
    lat0, lon0 = proj.to_latlon(*xy[gate_idx[0]])
    write_ui(track_dir, cfg, geo.length if closed else gates[1] - gates[0], cfg["pit_boxes"],
             (float(lat0), float(lon0)), closed=closed)

    nverts = sum(len(m.vertices) for m in meshes)
    size_mb = (track_dir / f"{name}.kn5").stat().st_size / 1e6
    print(f"Done: {len(meshes)} meshes, {nverts:,} vertices, {name}.kn5 = {size_mb:.1f} MB")
    print(f"  AC track folder: {track_dir}")
    if args.install:
        dest = args.install / name
        if dest.exists():
            shutil.rmtree(dest)
        shutil.copytree(track_dir, dest)
        print(f"  installed to   : {dest}")


if __name__ == "__main__":
    main()
