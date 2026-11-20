"""Write the Assetto Corsa track folder skeleton, textures and preview images."""
import json

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

SURFACES_INI = """\
; Track-specific surfaces. ROAD and GRASS come from AC's built-in surfaces.ini.
[SURFACE_0]
KEY=BARRIER
FRICTION=0.6
DAMPING=0
WAV=
WAV_PITCH=0
FF_EFFECT=NULL
DIRT_ADDITIVE=0
IS_VALID_TRACK=0
BLACK_FLAG_TIME=0
SIN_HEIGHT=0
SIN_LENGTH=0
IS_PITLANE=0
VIBRATION_GAIN=0
VIBRATION_LENGTH=0
"""


# ---------------------------------------------------------------- textures
def _noise(size, seed, scale):
    rng = np.random.default_rng(seed)
    img = Image.fromarray((rng.random((size // scale, size // scale)) * 255).astype(np.uint8))
    return np.asarray(img.resize((size, size), Image.BICUBIC), dtype=float) / 255.0


def write_textures(tex_dir, size=512):
    tex_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(1)

    fine = rng.random((size, size))
    a = 0.30 + 0.06 * fine + 0.05 * _noise(size, 2, 32)
    # faint white edge lines at u ~ 0.02 / 0.98 (road UV spans the full width)
    u = np.linspace(0, 1, size)[None, :]
    edge = ((u > 0.015) & (u < 0.035)) | ((u > 0.965) & (u < 0.985))
    rgb = np.dstack([a, a, a * 1.02])
    rgb[np.broadcast_to(edge, (size, size))] = 0.85
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(tex_dir / "asphalt.png")

    g = 0.5 + 0.25 * _noise(size, 3, 16) + 0.15 * rng.random((size, size))
    grass = np.dstack([0.22 * g, 0.42 * g, 0.14 * g])
    Image.fromarray((np.clip(grass, 0, 1) * 255).astype(np.uint8)).save(tex_dir / "grass.png")

    v = np.linspace(0, 1, size)[:, None]
    stripe = (np.floor(np.linspace(0, 8, size))[None, :] % 2).astype(float)
    stripe = np.broadcast_to(stripe, (size, size))
    bar = np.dstack([0.75 + 0.2 * stripe, 0.75 - 0.5 * stripe, 0.75 - 0.5 * stripe])
    bar *= (0.9 + 0.1 * rng.random((size, size)))[..., None]
    bar[((v > 0.28) & (v < 0.30)).ravel(), :] = 0.2
    Image.fromarray((np.clip(bar, 0, 1) * 255).astype(np.uint8)).save(tex_dir / "barrier.png")

    Image.new("RGB", (64, 64), (235, 235, 235)).save(tex_dir / "line.png")


# ---------------------------------------------------------------- map / outline
def _fit(xy, w, h, margin):
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    scale = min((w - 2 * margin) / (hi[0] - lo[0]), (h - 2 * margin) / (hi[1] - lo[1]))
    px = (xy[:, 0] - lo[0]) * scale + margin
    py = h - ((xy[:, 1] - lo[1]) * scale + margin)
    return np.column_stack([px, py]), scale


def write_map(track_dir, xy, max_size=1024, margin=20, line=10):
    """map.png + data/map.ini. AC world: X = east, Z = -north (we use x, -y)."""
    wx, wz = xy[:, 0], -xy[:, 1]
    ext = max(wx.max() - wx.min(), wz.max() - wz.min())
    scale_factor = ext / (max_size - 2 * margin)  # metres per pixel
    W = int((wx.max() - wx.min()) / scale_factor + 2 * margin)
    H = int((wz.max() - wz.min()) / scale_factor + 2 * margin)
    x_off = -wx.min() + margin * scale_factor
    z_off = -wz.min() + margin * scale_factor
    pts = np.column_stack([(wx + x_off) / scale_factor, (wz + z_off) / scale_factor])
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    p = [tuple(q) for q in pts] + [tuple(pts[0])]
    d.line(p, fill=(255, 255, 255, 255), width=line, joint="curve")
    img.save(track_dir / "map.png")
    (track_dir / "data" / "map.ini").write_text(
        "[PARAMETERS]\n"
        f"WIDTH={W}\nHEIGHT={H}\nMARGIN={margin}\n"
        f"SCALE_FACTOR={scale_factor:.6f}\nX_OFFSET={x_off:.4f}\nZ_OFFSET={z_off:.4f}\n"
        f"MAX_SIZE={max_size}\nDRAWING_SIZE={line}\n")


def write_outline(path, xy, start_i, size=(365, 365)):
    img = Image.new("RGBA", size, (0, 0, 0, 0))
    pts, _ = _fit(xy, size[0], size[1], 20)
    d = ImageDraw.Draw(img)
    p = [tuple(q) for q in pts] + [tuple(pts[0])]
    d.line(p, fill=(255, 255, 255, 255), width=6, joint="curve")
    sx, sy = pts[start_i]
    d.ellipse([sx - 7, sy - 7, sx + 7, sy + 7], fill=(230, 40, 40, 255))
    img.save(path)


def write_preview(path, terrain, xy, size=(1022, 575)):
    """Hill-shaded top-down render of the generated terrain with the road on top."""
    X, Y, H = terrain
    gy, gx = np.gradient(H)
    light = np.array([-1.0, 1.0, 1.4])
    light /= np.linalg.norm(light)
    n = np.dstack([-gx, -gy, np.ones_like(H) * 1.0])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    shade = np.clip(n @ light, 0, 1)
    hn = (H - H.min()) / max(np.ptp(H), 1e-6)
    base = np.dstack([0.25 + 0.25 * hn, 0.42 + 0.2 * hn, 0.2 + 0.1 * hn])
    rgb = base * (0.35 + 0.75 * shade[..., None])
    img = Image.fromarray((np.clip(rgb[::-1], 0, 1) * 255).astype(np.uint8))
    # road
    px = (xy[:, 0] - X[0, 0]) / (X[0, 1] - X[0, 0])
    py = (H.shape[0] - 1) - (xy[:, 1] - Y[0, 0]) / (Y[1, 0] - Y[0, 0])
    d = ImageDraw.Draw(img)
    p = list(zip(px, py)) + [(px[0], py[0])]
    d.line(p, fill=(55, 55, 58), width=max(3, int(8 / (X[0, 1] - X[0, 0]))), joint="curve")
    # crop/letterbox to the AC preview aspect
    img.thumbnail(size, Image.LANCZOS)
    canvas = Image.new("RGB", size, (20, 20, 20))
    canvas.paste(img, ((size[0] - img.width) // 2, (size[1] - img.height) // 2))
    canvas.filter(ImageFilter.SMOOTH).save(path)


def write_ui(track_dir, cfg, length_m, n_pits, latlon):
    ui = {
        "name": cfg.get("display_name", cfg["name"]),
        "description": cfg.get("description", "Generated from OpenStreetMap and public elevation data."),
        "tags": ["circuit", "street", "original"],
        "geotags": [f"{latlon[0]:.5f}", f"{latlon[1]:.5f}"],
        "country": cfg.get("country", ""),
        "city": cfg.get("city", ""),
        "length": f"{length_m:.0f}",
        "width": f"{cfg['road']['width']:.0f}",
        "pitboxes": str(n_pits),
        "run": "clockwise" if cfg.get("_clockwise") else "counter-clockwise",
        "author": cfg.get("author", ""),
        "version": "0.1",
        "url": "",
        "year": None,
    }
    (track_dir / "ui" / "ui_track.json").write_text(json.dumps(ui, indent=2))


def write_route_debug(path, graph, route_xy, waypoints_xy, size=1400):
    """All OSM roads in grey, the routed loop in red, numbered waypoints."""
    allxy = np.vstack([route_xy, waypoints_xy])
    lo, hi = allxy.min(axis=0) - 150, allxy.max(axis=0) + 150
    scale = size / max(hi - lo)
    W, H = int((hi[0] - lo[0]) * scale), int((hi[1] - lo[1]) * scale)

    def px(p):
        return ((p[0] - lo[0]) * scale, H - (p[1] - lo[1]) * scale)

    img = Image.new("RGB", (W, H), (250, 250, 248))
    d = ImageDraw.Draw(img)
    for a, nbrs in graph.adj.items():
        for b, _ in nbrs:
            if a < b:
                d.line([px(graph.coords[a]), px(graph.coords[b])], fill=(170, 170, 170), width=2)
    pts = [px(p) for p in route_xy] + [px(route_xy[0])]
    d.line(pts, fill=(220, 30, 30), width=4)
    # direction arrows every ~1/20 of the lap
    for i in range(0, len(route_xy), max(1, len(route_xy) // 20)):
        j = (i + 1) % len(route_xy)
        x0, y0 = px(route_xy[i])
        x1, y1 = px(route_xy[j])
        d.ellipse([x0 - 5, y0 - 5, x0 + 5, y0 + 5], outline=(120, 0, 0), width=2)
        d.line([(x0, y0), (x0 + (x1 - x0) * 3, y0 + (y1 - y0) * 3)], fill=(120, 0, 0), width=3)
    for k, w in enumerate(waypoints_xy):
        x, y = px(w)
        d.ellipse([x - 12, y - 12, x + 12, y + 12], fill=(30, 90, 220))
        d.text((x - 4, y - 6), str(k), fill=(255, 255, 255))
    img.save(path)
