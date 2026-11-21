"""Geometry generation: road ribbon, shoulders, terrain, barriers, AC markers.

Works for closed loops (circuits) and open roads (point-to-point / A-to-B).
"""
import numpy as np

from .centerline import curvature_radius, tangents
from .mesh import Mesh, strip_chunks

SHOULDER_DROP = 0.03  # shoulder sits slightly below the road edge
TERRAIN_DROP = 0.15   # terrain under road/shoulder hidden just below them
TERRAIN_TILE = 180    # cells per terrain tile side (keeps meshes < 65k vertices)


def _smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def conform_height(d, zc, dem, inner, blend):
    """Height of the ground at distance d from the centreline."""
    flat = zc - TERRAIN_DROP
    w = _smoothstep((d - inner) / blend)
    return flat * (1 - w) + dem * w


class TrackGeometry:
    def __init__(self, xy, z, cfg, closed=True):
        self.xy = xy
        self.z = z
        self.cfg = cfg
        self.closed = closed
        self.n = len(xy)
        self.t = tangents(xy, closed=closed)
        self.nrm = np.column_stack([-self.t[:, 1], self.t[:, 0]])  # left normal
        pts = np.vstack([xy, xy[:1]]) if closed else xy
        seg = np.hypot(*np.diff(pts, axis=0).T)
        # closed: n+1 entries (last = lap length); open: n entries
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.s[-1])
        # rows used for strips (closed loops repeat the first row at the end)
        self.rows = np.concatenate([np.arange(self.n), [0]]) if closed else np.arange(self.n)
        # segments (i, j)
        i = np.arange(self.n if closed else self.n - 1)
        self.seg_i, self.seg_j = i, (i + 1) % self.n

    # ------------------------------------------------------------------ checks
    def report(self):
        hw = self.cfg["road"]["width"] / 2
        r = np.abs(curvature_radius(self.xy, self.closed))
        if not self.closed:
            r = r[2:-2]
        tight = np.sum(r < hw * 1.5)
        dz = self.z[self.seg_j] - self.z[self.seg_i]
        ds = np.maximum(np.diff(self.s), 1e-6)
        grade = dz / ds
        print(f"  length {self.length:.0f} m, elevation range {self.z.min():.1f}..{self.z.max():.1f} m, "
              f"max grade {np.abs(grade).max() * 100:.1f}%, tightest radius {r.min():.1f} m")
        if tight:
            print(f"  warning: {tight} samples have a radius below 1.5x the half-width; "
                  "the road may fold there. Increase smoothing.xy_m or reduce road.width.")

    # ------------------------------------------------------------------ road
    def road_meshes(self):
        rc = self.cfg["road"]
        hw = rc["width"] / 2
        cols = 5
        lat = np.linspace(-hw, hw, cols)  # right ... left
        idx = self.rows
        P = self.xy[idx][:, None, :] + self.nrm[idx][:, None, :] * lat[None, :, None]
        Z = np.repeat(self.z[idx][:, None], cols, axis=1)
        V = np.dstack([P, Z])
        u = (lat + hw) / (2 * hw)
        v = self.s[:len(idx), None] / rc["width"]
        UV = np.dstack([np.broadcast_to(u, (len(idx), cols)), np.broadcast_to(v, (len(idx), cols))])
        meshes = []
        for k, (a, b) in enumerate(strip_chunks(len(idx), cols)):
            m = Mesh(f"{k + 1}ROAD", "asphalt")
            m.add_grid(V[a:b + 1], UV[a:b + 1])
            meshes.append(m)
        return meshes

    def shoulder_meshes(self, start_num):
        rc = self.cfg["road"]
        hw = rc["width"] / 2
        sw = rc["shoulder"]
        if sw <= 0:
            return []
        idx = self.rows
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
                            np.broadcast_to(self.s[:len(idx), None] / 4.0, (len(idx), 2))])
            for a, b in strip_chunks(len(idx), 2):
                m = Mesh(f"{num}GRASS", "grass")
                m.add_grid(V[a:b + 1], UV[a:b + 1])
                meshes.append(m)
                num += 1
        return meshes

    def line_mesh(self, name, s_at):
        """Thin white line across the road (visual only)."""
        hw = self.cfg["road"]["width"] / 2
        i = self.index_at(s_at)
        c, t, nl = self.xy[i], self.t[i], self.nrm[i]
        P = np.array([[c + t * along + nl * lat for lat in (-hw, hw)] for along in (-0.3, 0.3)])
        Z = np.full(P.shape[:2] + (1,), self.z[i] + 0.01)
        UV = np.array([[[0, 0], [1, 0]], [[0, 1], [1, 1]]], dtype=float)
        m = Mesh(name, "line")
        m.add_grid(np.dstack([P, Z]), UV)
        return m

    # ------------------------------------------------------------------ distance field
    def distance_field(self, X, Y, reach):
        """Distance to the centreline and centreline height at the closest point."""
        D = np.full(X.shape, np.inf)
        ZC = np.zeros(X.shape)
        x0, y0 = X[0, 0], Y[0, 0]
        g = X[0, 1] - X[0, 0]
        A = self.xy[self.seg_i]
        B = self.xy[self.seg_j]
        lo = np.minimum(A, B) - reach
        hi = np.maximum(A, B) + reach
        near = (hi[:, 0] >= X[0, 0]) & (lo[:, 0] <= X[0, -1]) & (hi[:, 1] >= Y[0, 0]) & (lo[:, 1] <= Y[-1, 0])
        for k in np.nonzero(near)[0]:
            i, j = self.seg_i[k], self.seg_j[k]
            a, b = A[k], B[k]
            c0 = max(int((lo[k, 0] - x0) / g), 0)
            c1 = min(int((hi[k, 0] - x0) / g) + 2, X.shape[1])
            r0 = max(int((lo[k, 1] - y0) / g), 0)
            r1 = min(int((hi[k, 1] - y0) / g) + 2, X.shape[0])
            if c0 >= c1 or r0 >= r1:
                continue
            px = X[r0:r1, c0:c1] - a[0]
            py = Y[r0:r1, c0:c1] - a[1]
            ab = b - a
            L2 = max(ab @ ab, 1e-9)
            t = np.clip((px * ab[0] + py * ab[1]) / L2, 0, 1)
            d = np.hypot(px - t * ab[0], py - t * ab[1])
            zc = self.z[i] * (1 - t) + self.z[j] * t
            sub = D[r0:r1, c0:c1]
            better = d < sub
            sub[better] = d[better]
            ZC[r0:r1, c0:c1][better] = zc[better]
        return D, ZC

    def terrain_meshes(self, dem_fn, start_num):
        """Terrain in tiles, only within terrain.margin_m of the road."""
        rc = self.cfg["road"]
        tc = self.cfg["terrain"]
        g = tc["grid_m"]
        margin = tc["margin_m"]
        inner = rc["width"] / 2 + rc["shoulder"]
        lo = np.floor((self.xy.min(axis=0) - margin) / g) * g
        hi = self.xy.max(axis=0) + margin
        nx = int(np.ceil((hi[0] - lo[0]) / g)) + 1
        ny = int(np.ceil((hi[1] - lo[1]) / g)) + 1
        T = TERRAIN_TILE
        meshes = []
        num = start_num
        total = 0
        for r in range(0, ny - 1, T):
            for c in range(0, nx - 1, T):
                xs = lo[0] + g * np.arange(c, min(c + T + 1, nx))
                ys = lo[1] + g * np.arange(r, min(r + T + 1, ny))
                # cheap reject: any centreline point within margin of this tile?
                inside = ((self.xy[:, 0] > xs[0] - margin) & (self.xy[:, 0] < xs[-1] + margin) &
                          (self.xy[:, 1] > ys[0] - margin) & (self.xy[:, 1] < ys[-1] + margin))
                if not inside.any():
                    continue
                X, Y = np.meshgrid(xs, ys)
                D, ZC = self.distance_field(X, Y, margin + 2 * g)
                quad_d = np.minimum(np.minimum(D[:-1, :-1], D[1:, :-1]), np.minimum(D[:-1, 1:], D[1:, 1:]))
                mask = quad_d <= margin
                if not mask.any():
                    continue
                dem = dem_fn(X, Y)
                H = np.where(np.isfinite(D), conform_height(D, ZC, dem, inner, tc["blend_m"]), dem)
                mm = Mesh(f"{num}GRASS", "grass")
                mm.add_grid(np.dstack([X, Y, H]), np.dstack([X / 8.0, Y / 8.0]), flip=True, mask=mask)
                mm.compact()
                meshes.append(mm)
                total += len(mm.vertices)
                num += 1
        print(f"  terrain: {len(meshes)} tiles, {total:,} vertices")
        return meshes

    # ------------------------------------------------------------------ barriers
    def _wall(self, P, base, height, num, uv_s, flip):
        """A barrier strip along polyline P.

        Returns the physical wall (<num>WALL, AC's collision surface type), whose
        faces point towards the road, and a visual-only back face so the barrier
        also looks solid from outside. With flip=False the faces point to the
        right of P's direction of travel.
        """
        bot = np.column_stack([P, base - 0.5])
        top = np.column_stack([P, base + height])
        V = np.stack([bot, top], axis=1)
        UV = np.dstack([np.stack([uv_s, uv_s], 1) / 4.0, np.broadcast_to([0.0, 1.0], (len(P), 2))])
        wall = Mesh(f"{num}WALL", "barrier")
        wall.add_grid(V, UV, flip=flip)
        back = Mesh(f"wall_back_{num}", "barrier")  # no surface prefix -> no physics
        back.add_grid(V, UV, flip=not flip)
        return [wall, back]

    def barrier_meshes(self, dem_fn, start_num):
        rc = self.cfg["road"]
        if rc.get("wall_offset") is None:
            return []
        hw = rc["width"] / 2
        dist = hw + rc["wall_offset"]
        height = rc["wall_height"]
        inner = hw + rc["shoulder"]
        blend = self.cfg["terrain"]["blend_m"]
        meshes = []
        num = start_num
        n = len(self.xy)
        for side in (1, -1):
            P = self.xy + self.nrm * dist * side
            # drop points that ended up close to another part of the road
            # (inside of tight corners, crossings, parallel sections)
            keep = np.ones(n, dtype=bool)
            for k in range(0, n, 500):
                blk = P[k:k + 500]
                d = np.hypot(blk[:, None, 0] - self.xy[None, :, 0], blk[:, None, 1] - self.xy[None, :, 1])
                keep[k:k + 500] = d.min(axis=1) > dist - 0.75
            # drop kinks where the offset line runs backwards (inside of a tight curve);
            # repeat because removing a point can expose the next one
            for _ in range(10):
                idx = np.nonzero(keep)[0]
                nxt = np.roll(idx, -1)
                step = P[nxt] - P[idx]
                fwd = np.einsum("ij,ij->i", step, self.t[idx]) > 0
                if not self.closed:
                    fwd[-1] = True
                if fwd.all():
                    break
                keep[idx[~fwd]] = False
            base =conform_height(dist, self.z, dem_fn(P[:, 0], P[:, 1]), inner, blend)
            if keep.all():
                runs = [list(range(n)) + ([0] if self.closed else [])]
            else:
                first_gap = int(np.argmin(keep)) if self.closed else 0
                order = (np.arange(n) + first_gap) % n
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
                s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P[run], axis=0).T))])
                for a, b in strip_chunks(len(run), 4):
                    sl = slice(a, b + 1)
                    # left wall (side=+1) must face right, towards the road
                    meshes += self._wall(P[run][sl], base[run][sl], height, num, s[sl], flip=side < 0)
                    num += 1
        if not self.closed:
            # end caps across the road at both ends
            for i in (0, n - 1):
                lat = np.linspace(-dist, dist, 12)
                P = self.xy[i] + self.nrm[i] * lat[:, None]
                base = np.full(len(lat), self.z[i] - TERRAIN_DROP)
                # cap rows run right->left, so unflipped faces point forward:
                # the start cap faces forward, the end cap faces back
                meshes += self._wall(P, base, height, num, lat - lat[0], flip=(i != 0))
                num += 1
        return meshes

    # ------------------------------------------------------------------ markers
    def index_at(self, s):
        if self.closed:
            s = s % self.length
        else:
            s = min(max(s, 0.0), self.length)
        return min(int(np.searchsorted(self.s, s, side="right") - 1), self.n - 1) % self.n

    def auto_start(self, n_grid, n_pits):
        """Distance along the lap whose preceding stretch (grid + pits) is straightest."""
        back = 8 + (n_grid + n_pits) * 7 + 20
        step = self.length / self.n
        w = max(int(back / step), 2)
        ang = np.unwrap(np.arctan2(self.t[:, 1], self.t[:, 0]))
        dang = np.abs(np.angle(np.exp(1j * np.diff(np.concatenate([ang, ang[:1]])))))
        csum = np.concatenate([[0], np.cumsum(np.concatenate([dang, dang]))])
        idx = np.arange(self.n) + self.n
        turn = csum[idx] - csum[idx - w]
        return float(self.s[int(np.argmin(turn))])

    def _put(self, out, name, s, lateral=0.0, lift=0.2):
        i = self.index_at(s)
        p = self.xy[i] + self.nrm[i] * lateral
        out.append({"name": name,
                    "position": [float(p[0]), float(p[1]), float(self.z[i] + lift)],
                    "forward": [float(self.t[i][0]), float(self.t[i][1]), 0.0]})

    def markers(self, start_offset, n_grid, n_pits):
        """Circuit AC dummy objects. Returns (markers, start/finish distance)."""
        hw = self.cfg["road"]["width"] / 2
        out = []
        s0 = start_offset
        self._put(out, "AC_TIME_0_L", s0, hw + 0.5, 0.0)
        self._put(out, "AC_TIME_0_R", s0, -(hw + 0.5), 0.0)
        for k in range(n_grid):
            self._put(out, f"AC_START_{k}", s0 - 8 - k * 7, (hw / 3) * (1 if k % 2 == 0 else -1))
        pit_s = s0 - 8 - n_grid * 7 - 20
        for k in range(n_pits):
            self._put(out, f"AC_PIT_{k}", pit_s - k * 7, (hw / 3) * (1 if k % 2 == 0 else -1))
        self._put(out, "AC_HOTLAP_START_0", s0 - min(200, self.length * 0.2))
        return out, [s0]

    def markers_ab(self, n_grid, n_pits, runoff):
        """Point-to-point dummies: pits, then grid, then the A gate; B gate before the end."""
        hw = self.cfg["road"]["width"] / 2
        out = []
        stagger = hw / 3
        pit0 = 12.0
        for k in range(n_pits):
            self._put(out, f"AC_PIT_{k}", pit0 + k * 7, stagger * (1 if k % 2 == 0 else -1))
        s_a = pit0 + n_pits * 7 + 15 + 8 + max(n_grid - 1, 0) * 7 + 8
        for k in range(n_grid):
            self._put(out, f"AC_START_{k}", s_a - 8 - k * 7, stagger * (1 if k % 2 == 0 else -1))
        self._put(out, "AC_HOTLAP_START_0", 6.0)
        s_b = self.length - runoff
        if s_b <= s_a + 100:
            raise SystemExit("Road too short for the grid/pits; lower grid_slots/pit_boxes or runoff_m")
        for nm, s in (("AC_AB_START", s_a), ("AC_AB_FINISH", s_b)):
            self._put(out, f"{nm}_L", s, hw + 0.5, 0.0)
            self._put(out, f"{nm}_R", s, -(hw + 0.5), 0.0)
        return out, [s_a, s_b]
