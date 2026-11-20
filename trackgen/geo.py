"""Lat/lon <-> local metric coordinates.

A local tangent-plane (equirectangular) projection centred on the track is
accurate to well under a metre over the few kilometres a track covers.
Local frame: x = east (m), y = north (m).
"""
import math

import numpy as np

EARTH_R = 6378137.0


class LocalProjection:
    def __init__(self, lat0, lon0):
        self.lat0 = lat0
        self.lon0 = lon0
        self.kx = math.radians(1.0) * EARTH_R * math.cos(math.radians(lat0))
        self.ky = math.radians(1.0) * EARTH_R

    def to_xy(self, lat, lon):
        lat = np.asarray(lat, dtype=float)
        lon = np.asarray(lon, dtype=float)
        return (lon - self.lon0) * self.kx, (lat - self.lat0) * self.ky

    def to_latlon(self, x, y):
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        return self.lat0 + y / self.ky, self.lon0 + x / self.kx
