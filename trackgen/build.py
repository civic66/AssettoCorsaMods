"""Geometry generation: road ribbon, shoulders, terrain, barriers, AC markers."""
import numpy as np

from .centerline import curvature_radius, tangents
from .mesh import Mesh, strip_chunks

ROAD_SINK = 0.0      # road surface sits exactly on the centreline height
SHOULDER_DROP = 0.03  # shoulder sits slightly below the road edge
TERRAIN_DROP = 0.15   # terrain under road/shoulder hidden just below them


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def conform_height(d, zc, dem, inner, blend):
    """Height of the ground at distance d from the centreline."""
    flat = zc - TERRAIN_DROP
    w = _smoothstep((d - inner) / blend)
    return flat * (1 - w) + dem * w


class TrackGeometry:
    def __init__(self, xy, z, cfg):
        self.xy = xy
        self.z = z
        self.cfg = cfg
        self.n = len(xy)
        self.t = tangents(xy, closed=True)
        self.nrm = np.column_stack([-self.t[:, 1], self.t[:, 0]])  # left normal
        seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])  # n+1 entries, s[-1] = length
        self.length = float(self.s[-1])

    # ------------------------------------------------------------------ checks
    def report(self):
        hw = self.cfg["road"]["width"] / 2
        r = np.abs(curvature_radius(self.xy))
        tight = np.sum(r < hw * 1.5)
        grade = np.diff(np.concatenate([self.z, self.z[:1]])) / np.diff(self.s)
        print(f"  length {self.length:.0f} m, elevation range {self.z.min():.1f}..{self.z.max():.1f} m, "
              f"max grade {np.abs(grade).max() * 100:.1f}%, tightest radius {r.min():.1f} m")
        if tight:
            print(f"  warning: {tight} samples have a radius below 1.5x the half-width; "
                  "the road may fold there. Increase smoothing.xy_m or reduce road.width.")

    # ------------------------------------------------------------------ road
    def _ring_rows(self):
        """Close the loop by repeating the first row at the end."""
        idx = np.concatenate([np.arange(self.n), [0]])
        return idx

    def road_meshes(self):
        rc = self.cfg["road"]
        hw = rc["width"] / 2
        cols = 5
        lat = np.linspace(-hw, hw, cols)  # right ... left
        idx = self._ring_rows()
        P = self.xy[idx][:, None, :] + self.nrm[idx][:, None, :] * lat[None, :, None]
        Z = np.repeat(self.z[idx][:, None] - ROAD_SINK, cols, axis=1)
        V = np.dstack([P, Z])
        u = (lat + hw) / (2 * hw)
        v = self.s[:, None] / rc["width"]
        UV = np.dstack([np.broadcast_to(u, (len(idx), cols)), np.broadcast_to(v, (len(idx), cols))])
        meshes = []
        for k, (a, b) in enumerate(strip_chunks(len(idx), cols)):
            m = Mesh(f"{k + 1}ROAD", "asphalt")
            m.add_grid(V[a:b + 1], UV[a:b + 1])
            meshes.append(m)
        return meshes

    def shoulder_meshes(self, dem_fn, start_num):
        rc = self.cfg["road"]
        hw = rc["width"] / 2
        sw = rc["shoulder"]
        if sw <= 0:
            return []
        idx = self._ring_rows()
        meshes = []
        num = start_num
        for side in (1, -1):
            off = np.array([hw, hw + sw]) * side
            if side < 0:
                off = off[::-1]  # keep column order right->left so winding is consistent
            P = self.xy[idx][:, None, :] + self.nrm[idx][:, None, :] * off[None, :, None]
            Z = np.repeat(self.z[idx][:, None] - SHOULDER_DROP, 2, axis=1)
            V = np.dstack([P, Z])
            UV = np.dstack([np.broadcast_to(off / 4.0, (len(idx), 2)),
                            np.broadcast_to(self.s[:, None] / 4.0, (len(idx), 2))])
            for a, b in strip_chunks(len(idx), 2):
                m = Mesh(f"{num}GRASS", "grass")
                m.add_grid(V[a:b + 1], UV[a:b + 1])
                meshes.append(m)
                num += 1
        return meshes

    def start_line_mesh(self, start_idx):
        hw = self.cfg["road"]["width"] / 2
        i = start_idx
        c, t, nl = self.xy[i], self.t[i], self.nrm[i]
        corners = []
        for along in (-0.3, 0.3):
            row = [c + t * along + nl * lat for lat in (-hw, hw)]
            corners.append(row)
        P = np.array(corners)
        Z = np.full(P.shape[:2] + (1,), self.z[i] + 0.01)
        UV = np.array([[[0, 0], [1, 0]], [[0, 1], [1, 1]]], dtype=float)
        m = Mesh("start_line", "line")  # visual only (no surface prefix)
        m.add_grid(np.dstack([P, Z]), UV)
        return m

    # ------------------------------------------------------------------ distance field
    def distance_field(self, X, Y):
        """Distance to the centreline and centreline height at the closest point."""
        rc = self.cfg["road"]
        tc = self.cfg["terrain"]
        reach = rc["width"] / 2 + rc["shoulder"] + tc["blend_m"] + 5
        D = np.full(X.shape, np.inf)
        ZC = np.zeros(X.shape)
        x0, y0 = X[0, 0], Y[0, 0]
        g = tc["grid_m"]
        for i in range(self.n):
            j = (i + 1) % self.n
            a, b = self.xy[i], self.xy[j]
            lo = np.minimum(a, b) - reach
            hi = np.maximum(a, b) + reach
            c0 = max(int((lo[0] - x0) / g), 0)
            c1 = min(int((hi[0] - x0) / g) + 2, X.shape[1])
            r0 = max(int((lo[1] - y0) / g), 0)
            r1 = min(int((hi[1] - y0) / g) + 2, X.shape[0])
            if c0 >= c1 or r0 >= r1:
                continue
            px = X[r0:r1, c0:c1] - a[0]
            py = Y[r0:r1, c0:c1] - a[1]
            ab = b - a
            L2 = ab @ ab
            t = np.clip((px * ab[0] + py * ab[1]) / L2, 0, 1)
            d = np.hypot(px - t * ab[0], py - t * ab[1])
            zc = self.z[i] * (1 - t) + self.z[j] * t
            sub = D[r0:r1, c0:c1]
            better = d < sub
            sub[better] = d[better]
            ZC[r0:r1, c0:c1][better] = zc[better]
        return D, ZC

    def terrain_meshes(self, dem_fn, start_num):
        rc = self.cfg["road"]
        tc = self.cfg["terrain"]
        g = tc["grid_m"]
        m = tc["margin_m"]
        lo = self.xy.min(axis=0) - m
        hi = self.xy.max(axis=0) + m
        xs = np.arange(lo[0], hi[0] + g, g)
        ys = np.arange(lo[1], hi[1] + g, g)
        X, Y = np.meshgrid(xs, ys)
        print(f"  terrain grid {X.shape[1]} x {X.shape[0]} ({X.size:,} vertices)")
        dem = dem_fn(X, Y)
        D, ZC = self.distance_field(X, Y)
        inner = rc["width"] / 2 + rc["shoulder"]
        H = np.where(np.isfinite(D), conform_height(D, ZC, dem, inner, tc["blend_m"]), dem)
        V = np.dstack([X, Y, H])
        UV = np.dstack([X / 8.0, Y / 8.0])
        meshes = []
        num = start_num
        # chunk into tiles of <= MAX_VERTS
        tile = 200
        for r in range(0, X.shape[0] - 1, tile):
            for c in range(0, X.shape[1] - 1, tile):
                sl = (slice(r, min(r + tile + 1, X.shape[0])), slice(c, min(c + tile + 1, X.shape[1])))
                mm = Mesh(f"{num}GRASS", "grass")
                mm.add_grid(V[sl], UV[sl], flip=True)
                meshes.append(mm)
                num += 1
        self.terrain = (X, Y, H)
        return meshes

    # ------------------------------------------------------------------ barriers
    def barrier_meshes(self, dem_fn, start_num):
        rc = self.cfg["road"]
        if rc.get("wall_offset") is None:
            return []
        hw = rc["width"] / 2
        dist = hw + rc["wall_offset"]
        height = rc["wall_height"]
        inner = hw + rc["shoulder"]
        meshes = []
        num = start_num
        for side in (1, -1):
            P = self.xy + self.nrm * dist * side
            # drop points that ended up close to another part of the road
            # (inside of tight corners, crossings, parallel sections)
            keep = np.ones(len(P), dtype=bool)
            for k in range(0, len(P), 500):
                blk = P[k:k + 500]
                d = np.hypot(blk[:, None, 0] - self.xy[None, :, 0], blk[:, None, 1] - self.xy[None, :, 1])
                keep[k:k + 500] = d.min(axis=1) > dist - 0.75
            base = conform_height(dist, self.z, dem_fn(P[:, 0], P[:, 1]), inner, self.cfg["terrain"]["blend_m"])
            # split into continuous runs (wrapping around the loop)
            if keep.all():
                runs = [list(range(len(P))) + [0]]  # closed ring
            else:
                first_gap = int(np.argmin(keep))
                order = (np.arange(len(P)) + first_gap) % len(P)
                runs, cur = [], []
                for i in order:
                    if keep[i]:
                        cur.append(int(i))
                    elif cur:
                        runs.append(cur)
                        cur = []
                if cur:
                    runs.append(cur)
            for run in runs:
                if len(run) < 3:
                    continue
                run = np.array(run)
                bot = np.column_stack([P[run], base[run] - 0.5])
                top = np.column_stack([P[run], base[run] + height])
                s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P[run], axis=0).T))])
                V = np.stack([bot, top], axis=1)
                UV = np.dstack([np.stack([s, s], 1) / 4.0, np.broadcast_to([0.0, 1.0], (len(run), 2))])
                for a, b in strip_chunks(len(run), 4):
                    mm = Mesh(f"{num}BARRIER", "barrier")
                    # double sided: both windings
                    mm.add_grid(V[a:b + 1], UV[a:b + 1], flip=(side > 0))
                    mm.add_grid(V[a:b + 1], UV[a:b + 1], flip=(side < 0))
                    meshes.append(mm)
                    num += 1
        return meshes

    # ------------------------------------------------------------------ markers
    def index_at(self, s):
        s = s % self.length
        return int(np.searchsorted(self.s, s, side="right") - 1) % self.n

    def auto_start(self, n_grid, n_pits):
        """Distance along the lap whose preceding stretch (grid + pits) is straightest."""
        back = 8 + (n_grid + n_pits) * 7 + 20
        step = self.length / self.n
        w = max(int(back / step), 2)
        ang = np.unwrap(np.arctan2(self.t[:, 1], self.t[:, 0]))
        dang = np.abs(np.angle(np.exp(1j * np.diff(np.concatenate([ang, ang[:1]])))))
        csum = np.concatenate([[0], np.cumsum(np.concatenate([dang, dang]))])
        # turning over the window ending at i (wrapping)
        idx = np.arange(self.n) + self.n
        turn = csum[idx] - csum[idx - w]
        i = int(np.argmin(turn))
        return float(self.s[i])

    def markers(self, start_offset, n_grid, n_pits):
        """AC dummy objects. Position in local frame, forward = driving direction."""
        hw = self.cfg["road"]["width"] / 2
        out = []

        def put(name, s, lateral=0.0, lift=0.2):
            i = self.index_at(s)
            p = self.xy[i] + self.nrm[i] * lateral
            out.append({"name": name,
                        "position": [float(p[0]), float(p[1]), float(self.z[i] + lift)],
                        "forward": [float(self.t[i][0]), float(self.t[i][1]), 0.0]})

        s0 = start_offset
        i0 = self.index_at(s0)
        put("AC_TIME_0_L", s0, hw + 0.5, 0.0)
        put("AC_TIME_0_R", s0, -(hw + 0.5), 0.0)
        for k in range(n_grid):
            put(f"AC_START_{k}", s0 - 8 - k * 7, (hw / 3) * (1 if k % 2 == 0 else -1))
        pit_s = s0 - 8 - n_grid * 7 - 20
        for k in range(n_pits):
            put(f"AC_PIT_{k}", pit_s - k * 7, (hw / 3) * (1 if k % 2 == 0 else -1))
        put("AC_HOTLAP_START_0", s0 - min(200, self.length * 0.2))
        return out, i0
