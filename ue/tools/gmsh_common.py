"""The shared base of the gmsh-backed scripts (rib_gmsh.py, potential_flow.py)
and of the structural solver that reads their output (structural_fea.py).

Importing this module first is what makes the others work from a bare
python.exe: ``_bootstrap_dlls`` puts the environment's ``Library\\bin`` on
PATH (numpy's BLAS and gmsh's own DLLs live there, and it is on PATH only
inside an activated environment; the application runs the interpreter
directly) before numpy and gmsh are imported.

What lives here is what every gmsh job needs of a triangulated surface:
the ``.tri`` reader, the closed-surface check, the divergence-theorem
volume and the area, the split into connected shells (with the original
vertex ids, for matching a rib's rim to the body's nodes), the discrete
surface entity, and the tetrahedron volumes the fluid-domain mesher uses
to orient its elements.

``.tri`` is the application's own format: 8 bytes ``AEROTRI1``, uint32
vertex count, uint32 triangle count, float64 xyz per vertex, int32 abc per
triangle.

History: this was the head of tools/tet_gmsh.py, the skin / core
tetrahedral mesher, until 2026-11-08, when the application stopped
supporting tetrahedral structural models and lattice infill. The rib disc
mesher that was its second half is now tools/rib_gmsh.py; the potential
flow's fluid domain still meshes tetrahedra (a CFD domain, not a
structure) and takes its helpers from here.
"""

import os
import struct
import sys


def _bootstrap_dlls():
    """numpy's BLAS and gmsh's own DLLs live in <env>\\Library\\bin, which is
    only on PATH inside an activated environment; the application runs
    python.exe directly."""
    lib_bin = os.path.join(sys.prefix, "Library", "bin")
    if os.path.isdir(lib_bin):
        os.environ["PATH"] = lib_bin + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(lib_bin)


_bootstrap_dlls()

import numpy as np  # noqa: E402
import gmsh  # noqa: E402

TRI_MAGIC = b"AEROTRI1"

# gmsh element types.
GMSH_TRIANGLE = 2
GMSH_TET = 4


class JobError(Exception):
    pass


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def read_tri(path):
    """(vertices, triangles) as float64 / int32 arrays. An empty file (no
    triangles) is a surface that is not there."""
    with open(path, "rb") as f:
        if f.read(8) != TRI_MAGIC:
            raise JobError("%s is not a .tri file" % os.path.basename(path))
        nv, nt = struct.unpack("<II", f.read(8))
        verts = np.frombuffer(f.read(nv * 24), dtype="<f8").reshape(nv, 3)
        faces = np.frombuffer(f.read(nt * 12), dtype="<i4").reshape(nt, 3)
    return np.ascontiguousarray(verts), np.ascontiguousarray(faces)


def write_tri(path, verts, faces):
    with open(path, "wb") as f:
        f.write(TRI_MAGIC)
        f.write(struct.pack("<II", len(verts), len(faces)))
        f.write(np.ascontiguousarray(verts, dtype="<f8").tobytes())
        f.write(np.ascontiguousarray(faces, dtype="<i4").tobytes())


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def surface_volume(verts, faces):
    """Divergence-theorem volume; positive when the triangles face outward."""
    if len(faces) == 0:
        return 0.0
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def surface_area(verts, faces):
    if len(faces) == 0:
        return 0.0
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    return float(np.linalg.norm(np.cross(b - a, c - a), axis=1).sum() / 2.0)


def tet_volumes(nodes, tets):
    a, b, c, d = nodes[tets[:, 0]], nodes[tets[:, 1]], nodes[tets[:, 2]], nodes[tets[:, 3]]
    return np.einsum("ij,ij->i", b - a, np.cross(c - a, d - a)) / 6.0


def check_closed(verts, faces, label):
    """Every edge used once in each direction: a closed, consistently
    oriented surface. gmsh will happily mesh garbage into garbage, so this
    is refused here with a reason rather than diagnosed from the result."""
    if len(faces) == 0:
        raise JobError("'%s' has no triangles." % label)
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    forward = {}
    for a, b in e:
        forward[(int(a), int(b))] = forward.get((int(a), int(b)), 0) + 1
    bad = 0
    for (a, b), n in forward.items():
        if n != 1 or forward.get((b, a), 0) != 1:
            bad += 1
    if bad:
        raise JobError("'%s' is not a closed, consistently oriented surface (%d edges are used more than once or have no opposite)." % (label, bad))
    if not surface_volume(verts, faces) > 0.0:
        raise JobError("'%s' is turned inside out (its enclosed volume is not positive)." % label)


# ---------------------------------------------------------------------------
# Shells and discrete entities
# ---------------------------------------------------------------------------

def shells(verts, faces):
    """Splits a surface into its connected components (shared vertices),
    each as its own (vertices, faces) pair. A set of bodies handed to the
    fluid-domain mesher is naturally several closed shells -- one per
    fuselage, boom and nacelle -- and gmsh needs each as its own entity:
    handed all five in one discrete surface it refuses the volume with
    "invalid boundary mesh (overlapping facets)"."""
    return [(sv, sf) for sv, sf, _ in shell_components(verts, faces)]


def shell_components(verts, faces):
    """As `shells`, each component also with `used`: the original vertex
    ids its vertices came from, in order (the ribs' rims are matched to
    the body's nodes through it)."""
    parent = np.arange(len(verts))

    def find(x):
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    for tri in faces:
        a = find(tri[0])
        for k in (1, 2):
            b = find(tri[k])
            if a != b:
                parent[b] = a

    roots = np.array([find(i) for i in range(len(verts))])
    out = []
    for root in np.unique(roots[faces[:, 0]]):
        keep = roots[faces[:, 0]] == root
        sub_faces = faces[keep]
        used = np.unique(sub_faces)
        remap = np.full(len(verts), -1, dtype=np.int64)
        remap[used] = np.arange(len(used))
        out.append((np.ascontiguousarray(verts[used]), np.ascontiguousarray(remap[sub_faces].astype(np.int32)), used))
    return out


def add_surface(verts, faces, tag, first_node):
    """A discrete surface entity holding exactly these triangles. Returns
    the next free node tag."""
    gmsh.model.addDiscreteEntity(2, tag)
    node_tags = np.arange(first_node, first_node + len(verts), dtype=np.uint64)
    gmsh.model.mesh.addNodes(2, tag, node_tags, verts.reshape(-1))
    gmsh.model.mesh.addElementsByType(tag, GMSH_TRIANGLE, [], (faces + first_node).astype(np.uint64).reshape(-1))
    return first_node + len(verts)


def write_failure(job_dir, message):
    """The failure convention every job shares: result.json becomes
    {"ok": false, "error": message} (the application reads "ok" and then
    "error"; the process also exits 1 and the last line of its output is
    "error: ...", which the application falls back on when no result was
    written)."""
    try:
        import json
        with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
            outputs = json.load(f).get("outputs", {})
        path = os.path.join(job_dir, outputs.get("result", "result.json"))
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"ok": False, "error": message}, f, indent=1)
    except Exception:
        pass
