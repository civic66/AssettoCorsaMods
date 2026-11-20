"""Minimal Assetto Corsa .kn5 writer (format version 5).

Layout (little-endian):
  "sc6969" + uint32 version(5)
  textures : int32 count, then {int32 active, string name, uint32 size, bytes}
  materials: int32 count, then {string name, string shader, uint8 blend, bool alphaTested,
             int32 depthMode, uint32 nProps, props{string, f, 2f, 3f, 4f},
             uint32 nSamplers, samplers{string name, uint32 slot, string texture}}
  nodes    : recursive. Base node = uint32 1, string name, uint32 nChildren, bool active, 16f matrix
             Mesh node = uint32 2, string name, uint32 0, bool active, bool castShadows,
             bool visible, bool transparent, uint32 nVerts, verts{3f pos, 3f normal, 2f uv,
             3f tangent}, uint32 nIdx, uint16 idx[], uint32 materialId, uint32 layer,
             f lodIn, f lodOut, 3f sphereCentre, f sphereRadius, bool renderable
  strings  : uint32 byte length + UTF-8 bytes

kn5 space is Y-up: (x, y, z)_kn5 = (east, up, -north). This is the same proper
rotation the FBX/Blender pipeline applies, so triangle winding is unchanged.
UVs are stored with v negated (DirectX convention).
"""
import struct

import numpy as np

# ksPerPixel material parameters (ambient, diffuse, specular, specular exponent)
DEFAULT_PROPS = {"ksAmbient": 0.45, "ksDiffuse": 0.45, "ksSpecular": 0.0,
                 "ksSpecularEXP": 1.0, "ksAlphaRef": 0.0}


class _W:
    def __init__(self, f):
        self.f = f

    def u32(self, v):
        self.f.write(struct.pack("<I", v))

    def i32(self, v):
        self.f.write(struct.pack("<i", v))

    def u8(self, v):
        self.f.write(struct.pack("<B", v))

    def bool(self, v):
        self.f.write(struct.pack("<?", bool(v)))

    def floats(self, *v):
        self.f.write(struct.pack(f"<{len(v)}f", *v))

    def string(self, s):
        b = s.encode("utf-8")
        self.u32(len(b))
        self.f.write(b)

    def blob(self, b):
        self.u32(len(b))
        self.f.write(b)


def to_kn5(v):
    """local (east, north, up) -> kn5 (x, y, z) = (east, up, -north)."""
    v = np.asarray(v, dtype=np.float64)
    return np.column_stack([v[:, 0], v[:, 2], -v[:, 1]])


def marker_matrix(position, forward):
    """Row-major 4x4 (DirectX row-vector convention, translation in the last row).
    Local +Z = driving direction, +Y = up, +X = up x forward."""
    p = to_kn5(np.array([position]))[0]
    f = to_kn5(np.array([forward]))[0]
    f[1] = 0.0
    f /= np.linalg.norm(f)
    up = np.array([0.0, 1.0, 0.0])
    x = np.cross(up, f)
    return [x[0], x[1], x[2], 0.0,
            up[0], up[1], up[2], 0.0,
            f[0], f[1], f[2], 0.0,
            p[0], p[1], p[2], 1.0]


IDENTITY = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def write_kn5(path, root_name, meshes, markers, materials, textures):
    """
    meshes   : list of trackgen.mesh.Mesh (local frame)
    markers  : list of {"name", "position", "forward"} (local frame)
    materials: {name: {"texture": texname, "props": {...}}}  (shader ksPerPixel)
    textures : {texname: bytes}  (PNG or DDS data)
    """
    mat_ids = {name: i for i, name in enumerate(materials)}
    meshes = [m for m in meshes if len(m.faces)]
    with open(path, "wb") as fh:
        w = _W(fh)
        fh.write(b"sc6969")
        w.u32(5)

        w.i32(len(textures))
        for name, data in textures.items():
            w.i32(1)
            w.string(name)
            w.blob(data)

        w.i32(len(materials))
        for name, mat in materials.items():
            w.string(name)
            w.string(mat.get("shader", "ksPerPixel"))
            w.u8(0)        # opaque
            w.bool(False)  # alpha tested
            w.i32(0)       # depth mode normal
            props = dict(DEFAULT_PROPS, **mat.get("props", {}))
            w.u32(len(props) + 1)
            for pn, val in props.items():
                w.string(pn)
                w.floats(val, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            w.string("ksEmissive")
            w.floats(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            w.u32(1)
            w.string("txDiffuse")
            w.u32(0)
            w.string(mat["texture"])

        # root
        w.u32(1)
        w.string(root_name)
        w.u32(len(meshes) + len(markers))
        w.bool(True)
        w.floats(*IDENTITY)

        for m in meshes:
            v = to_kn5(m.vertices)
            n = to_kn5(m.normals())
            uv = m.uvs.copy()
            uv[:, 1] = -uv[:, 1]
            # tangent: any unit vector perpendicular to the normal (ksPerPixel ignores it)
            tg = np.cross(n, np.array([0.0, 0.0, 1.0]))
            bad = np.linalg.norm(tg, axis=1) < 1e-3
            tg[bad] = np.cross(n[bad], np.array([1.0, 0.0, 0.0]))
            tg /= np.linalg.norm(tg, axis=1, keepdims=True)
            f = m.faces
            if len(v) > 65535:
                raise ValueError(f"{m.name}: {len(v)} vertices exceeds the kn5 limit of 65535")

            w.u32(2)
            w.string(m.name)
            w.u32(0)
            w.bool(True)   # active
            w.bool(True)   # cast shadows
            w.bool(True)   # visible
            w.bool(False)  # transparent
            w.u32(len(v))
            vb = np.hstack([v, n, uv, tg]).astype("<f4")
            fh.write(vb.tobytes())
            w.u32(f.size)
            fh.write(f.astype("<u2").tobytes())
            w.u32(mat_ids[m.material])
            w.u32(0)            # layer
            w.floats(0.0, 0.0)  # lodIn, lodOut (0 = no LOD)
            c = (v.min(axis=0) + v.max(axis=0)) / 2
            r = float(np.linalg.norm(v - c, axis=1).max())
            w.floats(*c, r)
            w.bool(True)   # renderable

        for mk in markers:
            w.u32(1)
            w.string(mk["name"])
            w.u32(0)
            w.bool(True)
            w.floats(*marker_matrix(mk["position"], mk["forward"]))


def read_kn5_summary(path):
    """Parse a kn5 back (used by tests): returns textures, materials, nodes."""
    with open(path, "rb") as fh:
        data = fh.read()
    o = 0

    def take(fmt):
        nonlocal o
        val = struct.unpack_from("<" + fmt, data, o)
        o += struct.calcsize("<" + fmt)
        return val if len(val) > 1 else val[0]

    def string():
        nonlocal o
        n = take("I")
        s = data[o:o + n].decode("utf-8")
        o += n
        return s

    assert data[:6] == b"sc6969"
    o = 6
    version = take("I")
    tex = []
    for _ in range(take("i")):
        take("i")
        name = string()
        size = take("I")
        tex.append((name, size))
        o += size
    mats = []
    for _ in range(take("i")):
        name, shader = string(), string()
        take("B"), take("?"), take("i")
        props = {}
        for _ in range(take("I")):
            pn = string()
            props[pn] = take("10f")[0]
        samplers = [(string(), take("I"), string()) for _ in range(take("I"))]
        mats.append((name, shader, props, samplers))
    nodes = []

    def node():
        nonlocal o
        cls = take("I")
        name = string()
        nch = take("I")
        take("?")
        info = {"class": cls, "name": name}
        if cls == 1:
            info["matrix"] = take("16f")
        else:
            take("???")
            nv = take("I")
            vb = np.frombuffer(data, "<f4", nv * 11, o).reshape(nv, 11)
            o += nv * 44
            ni = take("I")
            ib = np.frombuffer(data, "<u2", ni, o)
            o += ni * 2
            info.update(verts=vb, idx=ib, material=take("I"), layer=take("I"),
                        lod=take("2f"), sphere=take("4f"), renderable=take("?"))
        nodes.append(info)
        for _ in range(nch):
            node()

    node()
    assert o == len(data), f"trailing bytes: {len(data) - o}"
    return version, tex, mats, nodes
