"""Fetch the OpenStreetMap road network and route a loop through waypoints."""
import heapq
import json
import math
import os
import time

import numpy as np
import requests

from .cache import cache_path

OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]

DEFAULT_HIGHWAYS = [
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "unclassified", "residential", "living_street", "service",
    "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
    "track",
]

USER_AGENT = "trackgen/0.1 (Assetto Corsa track generator)"


def _fetch_overpass(query):
    last_err = None
    for url in OVERPASS_ENDPOINTS:
        try:
            r = requests.post(url, data={"data": query}, timeout=(10, 90),
                              headers={"User-Agent": USER_AGENT})
            if r.status_code == 200:
                return r.json()
            last_err = f"{url}: HTTP {r.status_code}"
        except requests.RequestException as e:  # try next mirror
            last_err = f"{url}: {type(e).__name__}"
        print(f"  overpass mirror failed ({last_err})")
    return None


def _fetch_osm_api(south, west, north, east, highways, tile_deg=0.015):
    """Fallback: the main OSM API /map call, tiled (it caps bbox size and node count)."""
    import xml.etree.ElementTree as ET

    nodes, ways = {}, {}
    lats = np.arange(south, north, tile_deg)
    lons = np.arange(west, east, tile_deg)
    total = len(lats) * len(lons)
    k = 0
    for la in lats:
        for lo in lons:
            k += 1
            bbox = f"{lo:.6f},{la:.6f},{min(lo + tile_deg, east):.6f},{min(la + tile_deg, north):.6f}"
            path = cache_path("osmapi", bbox, ".osm")
            if not path.exists():
                print(f"  OSM API tile {k}/{total}")
                for attempt in range(4):
                    try:
                        r = requests.get("https://api.openstreetmap.org/api/0.6/map",
                                         params={"bbox": bbox}, timeout=(10, 120),
                                         headers={"User-Agent": USER_AGENT})
                        if r.status_code == 200:
                            path.write_bytes(r.content)
                            break
                        if r.status_code == 400:
                            raise RuntimeError("OSM API refused the tile (too many nodes); "
                                               "lower tile_deg in osm.py")
                    except requests.RequestException:
                        pass
                    time.sleep(3 * (attempt + 1))
                else:
                    raise RuntimeError("OSM API download failed")
            root = ET.parse(path).getroot()
            for n in root.iter("node"):
                nodes[int(n.get("id"))] = (float(n.get("lat")), float(n.get("lon")))
            for w in root.iter("way"):
                tags = {t.get("k"): t.get("v") for t in w.iter("tag")}
                if tags.get("highway") in highways:
                    ways[int(w.get("id"))] = [int(nd.get("ref")) for nd in w.iter("nd")]
    elements = [{"type": "node", "id": i, "lat": a, "lon": b} for i, (a, b) in nodes.items()]
    elements += [{"type": "way", "id": i, "nodes": nds} for i, nds in ways.items()]
    return {"elements": elements}


def fetch_roads(south, west, north, east, highways=None):
    """Drivable OSM ways in a bounding box, as Overpass-style JSON."""
    highways = highways or DEFAULT_HIGHWAYS
    hw = "|".join(highways)
    query = (
        f"[out:json][timeout:120];"
        f'way["highway"~"^({hw})$"]({south:.6f},{west:.6f},{north:.6f},{east:.6f});'
        f"(._;>;);out body;"
    )
    path = cache_path("osm", query, ".json")
    if path.exists():
        return json.loads(path.read_text())
    data = None
    if os.environ.get("TRACKGEN_OSM_SOURCE", "").lower() != "api":
        data = _fetch_overpass(query)
    if data is None:
        print("  using the main OpenStreetMap API instead")
        data = _fetch_osm_api(south, west, north, east, set(highways))
    path.write_text(json.dumps(data))
    return data


class RoadGraph:
    def __init__(self, osm_json, proj):
        self.coords = {}  # node id -> (x, y)
        self.latlon = {}
        self.adj = {}  # node id -> list of (neighbour, length)
        for el in osm_json["elements"]:
            if el["type"] == "node":
                self.latlon[el["id"]] = (el["lat"], el["lon"])
                x, y = proj.to_xy(el["lat"], el["lon"])
                self.coords[el["id"]] = (float(x), float(y))
        for el in osm_json["elements"]:
            if el["type"] != "way":
                continue
            nodes = [n for n in el["nodes"] if n in self.coords]
            for a, b in zip(nodes, nodes[1:]):
                ax, ay = self.coords[a]
                bx, by = self.coords[b]
                d = math.hypot(bx - ax, by - ay)
                self.adj.setdefault(a, []).append((b, d))
                self.adj.setdefault(b, []).append((a, d))
        if not self.adj:
            raise RuntimeError("No roads found in the area; check the waypoints / bbox.")
        self._ids = np.array(list(self.adj.keys()))
        self._xy = np.array([self.coords[i] for i in self._ids])

    def nearest_node(self, x, y):
        d = np.hypot(self._xy[:, 0] - x, self._xy[:, 1] - y)
        i = int(np.argmin(d))
        return int(self._ids[i]), float(d[i])

    def shortest_path(self, src, dst, edge_penalty=None):
        """Dijkstra. edge_penalty: dict {(a,b): multiplier} to discourage reuse."""
        dist = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == dst:
                break
            if d > dist.get(u, math.inf):
                continue
            for v, w in self.adj[u]:
                if edge_penalty:
                    w *= edge_penalty.get((u, v), 1.0)
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if dst not in dist:
            raise RuntimeError(
                f"No road connection between OSM nodes {src} and {dst}. "
                "Add a waypoint or widen 'osm_margin_m'.")
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        return path[::-1]


def route_loop(graph, waypoints_xy, closed=True, reuse_penalty=4.0):
    """Route through waypoints (in order) and back to the first one.

    Roads already used by an earlier leg are made more expensive so the
    router prefers a real loop over driving out and back on the same road.
    """
    snapped = []
    for i, (x, y) in enumerate(waypoints_xy):
        nid, d = graph.nearest_node(x, y)
        if d > 60:
            print(f"  warning: waypoint {i} is {d:.0f} m from the nearest road")
        snapped.append(nid)
    legs = list(zip(snapped, snapped[1:]))
    if closed:
        legs.append((snapped[-1], snapped[0]))

    penalty = {}
    nodes = []
    for a, b in legs:
        if a == b:
            continue
        p = graph.shortest_path(a, b, penalty)
        for u, v in zip(p, p[1:]):
            penalty[(u, v)] = reuse_penalty
            penalty[(v, u)] = reuse_penalty
        nodes.extend(p if not nodes else p[1:])
    if closed and len(nodes) > 1 and nodes[0] == nodes[-1]:
        nodes = nodes[:-1]

    seen = {}
    for n in nodes:
        seen[n] = seen.get(n, 0) + 1
    repeats = sum(1 for c in seen.values() if c > 1)
    if repeats:
        print(f"  warning: route passes {repeats} road nodes more than once "
              "(crossing or shared road section); check the map preview.")
    xy = np.array([graph.coords[n] for n in nodes])
    return xy, nodes
