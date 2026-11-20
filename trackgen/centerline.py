"""Turn a jagged OSM polyline into a smooth, evenly spaced 3D centreline."""
import numpy as np


def dedupe(xy, min_dist=0.5):
    keep = [0]
    for i in range(1, len(xy)):
        if np.hypot(*(xy[i] - xy[keep[-1]])) >= min_dist:
            keep.append(i)
    return xy[keep]


def resample(xy, step, closed=True):
    pts = np.vstack([xy, xy[:1]]) if closed else xy
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(int(round(total / step)), 8)
    t = np.linspace(0, total, n, endpoint=not closed)
    return np.column_stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])])


def gaussian_smooth(values, sigma_samples, closed=True):
    """Gaussian filter along the track (wraps around for a loop)."""
    if sigma_samples <= 0:
        return values.copy()
    r = int(np.ceil(3 * sigma_samples))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_samples) ** 2)
    k /= k.sum()
    v = np.asarray(values, dtype=float)
    squeeze = v.ndim == 1
    if squeeze:
        v = v[:, None]
    out = np.empty_like(v)
    for c in range(v.shape[1]):
        col = v[:, c]
        if closed:
            padded = np.concatenate([col[-r:], col, col[:r]])
        else:
            padded = np.concatenate([np.full(r, col[0]), col, np.full(r, col[-1])])
        out[:, c] = np.convolve(padded, k, mode="valid")
    return out[:, 0] if squeeze else out


def tangents(xy, closed=True):
    if closed:
        t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    else:
        t = np.gradient(xy, axis=0)
    return t / np.linalg.norm(t, axis=1, keepdims=True)


def curvature_radius(xy, closed=True):
    """Signed turning radius at each point (positive = turning left)."""
    t = tangents(xy, closed)
    ang = np.arctan2(t[:, 1], t[:, 0])
    dang = np.diff(np.unwrap(np.concatenate([ang, ang[:1]] if closed else ang)))
    if not closed:
        dang = np.concatenate([dang, dang[-1:]])
    ds = np.hypot(*np.diff(np.vstack([xy, xy[:1]]) if closed else np.vstack([xy, xy[-1:]]), axis=0).T)
    with np.errstate(divide="ignore"):
        return ds / dang


def build_centerline(route_xy, step, smooth_xy_m, closed=True):
    xy = dedupe(route_xy)
    xy = resample(xy, 1.0, closed)  # fine resample before smoothing
    xy = gaussian_smooth(xy, smooth_xy_m / 1.0, closed)
    return resample(xy, step, closed)


def fit_heights(raw_z, step, smooth_z_m, max_grade=None, closed=True):
    z = gaussian_smooth(raw_z, smooth_z_m / step, closed)
    if max_grade:
        # Iteratively clamp grade between consecutive samples
        limit = max_grade * step
        for _ in range(50):
            dz = np.diff(np.concatenate([z, z[:1]]) if closed else z)
            if np.all(np.abs(dz) <= limit + 1e-6):
                break
            z = gaussian_smooth(z, 2.0, closed)
    return z
