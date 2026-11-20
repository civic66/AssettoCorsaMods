"""Elevation from the free AWS Terrain Tiles (Terrarium encoding).

Source: https://registry.opendata.aws/terrain-tiles/ (SRTM, USGS 3DEP/NED,
EU-DEM, GMTED and others merged; ~1-10 m in the US/EU, ~30 m elsewhere).
Attribution is required: see the README.
"""
import io
import math

import numpy as np
import requests
from PIL import Image

from .cache import cache_path

TILE_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
TILE = 256


def _lonlat_to_global_px(lat, lon, z):
    n = TILE * (2 ** z)
    x = (np.asarray(lon) + 180.0) / 360.0 * n
    lat_r = np.radians(np.asarray(lat))
    y = (1.0 - np.log(np.tan(lat_r) + 1.0 / np.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def _get_tile(z, x, y):
    url = TILE_URL.format(z=z, x=x, y=y)
    path = cache_path("terrarium", url, ".png")
    if not path.exists():
        r = requests.get(url, timeout=60)
        r.raise_for_status()
        path.write_bytes(r.content)
    img = np.asarray(Image.open(io.BytesIO(path.read_bytes())).convert("RGB"), dtype=np.float64)
    return img[..., 0] * 256.0 + img[..., 1] + img[..., 2] / 256.0 - 32768.0


class ElevationSampler:
    """Loads a mosaic of tiles covering a lat/lon box and samples it bilinearly."""

    def __init__(self, south, west, north, east, zoom=14):
        self.z = zoom
        x0, y0 = _lonlat_to_global_px(north, west, zoom)
        x1, y1 = _lonlat_to_global_px(south, east, zoom)
        tx0, ty0 = int(x0 // TILE), int(y0 // TILE)
        tx1, ty1 = int(x1 // TILE), int(y1 // TILE)
        ntiles = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
        if ntiles > 400:
            raise RuntimeError(f"Area needs {ntiles} elevation tiles; lower elevation_zoom.")
        rows = []
        for ty in range(ty0, ty1 + 1):
            rows.append(np.hstack([_get_tile(zoom, tx, ty) for tx in range(tx0, tx1 + 1)]))
        self.grid = np.vstack(rows)
        self.ox = tx0 * TILE
        self.oy = ty0 * TILE

    def sample(self, lat, lon):
        px, py = _lonlat_to_global_px(lat, lon, self.z)
        # Terrarium pixel centres are at +0.5
        fx = np.asarray(px) - self.ox - 0.5
        fy = np.asarray(py) - self.oy - 0.5
        h, w = self.grid.shape
        fx = np.clip(fx, 0, w - 1.001)
        fy = np.clip(fy, 0, h - 1.001)
        ix = np.floor(fx).astype(int)
        iy = np.floor(fy).astype(int)
        ax = fx - ix
        ay = fy - iy
        g = self.grid
        top = g[iy, ix] * (1 - ax) + g[iy, ix + 1] * ax
        bot = g[iy + 1, ix] * (1 - ax) + g[iy + 1, ix + 1] * ax
        return top * (1 - ay) + bot * ay
