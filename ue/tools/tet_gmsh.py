"""The skin / core tetrahedral mesh of a body, on gmsh.

Run by the application (Entity/AerotelicTetGmsh.cpp) as a separate process
in the WingDesigner micromamba environment, where gmsh is installed
(``pip install gmsh``)::

    python tet_gmsh.py <job directory>

The job directory holds ``job.json`` (see ``run_job`` for the keys) and the
two surfaces as ``.tri`` files: the body's own triangulation and, inside
it, MeshLib's inward offset at the skin depth (the lattice job writes it,
tools/lattice_meshlib.py). The script writes ``mesh.tet`` and
``result.json``.

Why gmsh and not the surfaces' own constrained Delaunay mesher: both
surfaces go in as DISCRETE entities, and the two volumes are bounded by
them, so gmsh adds interior nodes only. It never re-triangulates a
boundary, which is what makes

  * the outer faces of the skin tetrahedra the body's own triangles,
    exactly, with no Steiner points on them at all;
  * the faces between skin and core the offset's own triangles; and
  * skin volume + core volume = the body's enclosed volume, and core
    volume = the offset's, to rounding.

The mesher this replaced (core/src/tet.cpp) recovered the surfaces into a
Delaunay tetrahedralization instead, which cascaded wherever the two
surfaces were closer to each other than their own triangles are wide -- a
hollowed airframe, where the skin is one triangle thick over a large area.

``.tri`` is the application's own format: 8 bytes ``AEROTRI1``, uint32
vertex count, uint32 triangle count, float64 xyz per vertex, int32 abc per
triangle. ``.tet`` is its companion: 8 bytes ``AEROTET1``, uint32 node
count, uint32 tetrahedron count, float64 xyz per node, int32 abcd per
tetrahedron, int32 material per tetrahedron (1 skin, 2 core).
"""

import json
import os
import struct
import sys
import time


def _bootstrap_dlls():
    """numpy's BLAS and gmsh's own DLLs live in <env>\\Library\\bin, which is
    only on PATH inside an activated environment; the application runs
    python.exe directly (as tools/lattice_meshlib.py does)."""
    lib_bin = os.path.join(sys.prefix, "Library", "bin")
    if os.path.isdir(lib_bin):
        os.environ["PATH"] = lib_bin + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(lib_bin)


_bootstrap_dlls()

import numpy as np  # noqa: E402
import gmsh  # noqa: E402

TRI_MAGIC = b"AEROTRI1"
TET_MAGIC = b"AEROTET1"

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
    triangles) is a surface that is not there -- a body that is all skin."""
    with open(path, "rb") as f:
        if f.read(8) != TRI_MAGIC:
            raise JobError("%s is not a .tri file" % os.path.basename(path))
        nv, nt = struct.unpack("<II", f.read(8))
        verts = np.frombuffer(f.read(nv * 24), dtype="<f8").reshape(nv, 3)
        faces = np.frombuffer(f.read(nt * 12), dtype="<i4").reshape(nt, 3)
    return np.ascontiguousarray(verts), np.ascontiguousarray(faces)


def write_tet(path, nodes, tets, material):
    with open(path, "wb") as f:
        f.write(TET_MAGIC)
        f.write(struct.pack("<II", nodes.shape[0], tets.shape[0]))
        f.write(np.ascontiguousarray(nodes, dtype="<f8").tobytes())
        f.write(np.ascontiguousarray(tets, dtype="<i4").tobytes())
        f.write(np.ascontiguousarray(material, dtype="<i4").tobytes())


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
# The mesh
# ---------------------------------------------------------------------------

def shells(verts, faces):
    """Splits a surface into its connected components (shared vertices),
    each as its own (vertices, faces) pair. A hollowed airframe's offset
    is naturally several closed pockets -- one per fuselage, boom and
    nacelle -- and gmsh needs each as its own entity: handed all five in
    one discrete surface it refuses the volume with "invalid boundary mesh
    (overlapping facets)"."""
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


# ---------------------------------------------------------------------------
# Ribs: planar shells inside the volume (2026-11-08)
# ---------------------------------------------------------------------------
#
# A rib is a plane network's cross-section of the body: the application
# cuts the body's surface by the plane beforehand (core mesh_cut.hpp), so
# the surface has vertices exactly along the plane and the rib's rim can
# be those vertices -- the rib shells and the skin shells then share their
# nodes. The rib's disc is meshed in a scratch gmsh model from the rim's
# vertices (transfinite rim lines with their two endpoints only, so no new
# node appears on the rim), carried into the body's model as discrete
# surfaces whose rim nodes ARE the body's nodes, and made WALLS of the
# volume: every disc partitions the body into compartments, each meshed as
# a volume bounded by its piece of the skin and the walls round it, so the
# tetrahedra conform to the ribs by construction (the mechanism of the
# skin / core interface; embedding the discs in one volume crashed gmsh).
# Two crossing networks (a spar across the ribs) share the chord where
# their planes meet: the chord is embedded in both discs with the same
# transfinite node count, so both carry the same nodes along it, and each
# disc is split along it into the walls of the compartments on either side.

def rib_loops(verts, faces, normal, offset, tol):
    """The loops of vertex ids on the plane normal . x = offset that
    separate the two sides of the surface, each counter-clockwise about
    the normal where it bounds material (the rule of the application's
    SectionLoopsOnPlane: a vertex on the plane counts as positive)."""
    normal = np.asarray(normal, dtype=np.float64)
    d = verts @ normal - float(offset)
    on = np.abs(d) <= tol
    if on.sum() < 3:
        return []
    on_count = on[faces].sum(axis=1)
    candidates = faces[on_count >= 2]
    tri_normals = np.cross(verts[candidates[:, 1]] - verts[candidates[:, 0]], verts[candidates[:, 2]] - verts[candidates[:, 0]])
    edges = {}
    for tri, tn in zip(candidates, tri_normals):
        for e in range(3):
            a, b, c = int(tri[e]), int(tri[(e + 1) % 3]), int(tri[(e + 2) % 3])
            if not (on[a] and on[b]):
                continue
            key = (a, b) if a < b else (b, a)
            info = edges.setdefault(key, [False, None])
            if d[c] < -tol:
                info[1] = (a, b, tn)
            else:
                info[0] = True
    nxt = {}
    for key, (positive, negative) in edges.items():
        if not positive or negative is None:
            continue
        a, b, tn = negative
        left = np.cross(normal, verts[b] - verts[a])
        if np.dot(left, tn) < 0.0:
            nxt[a] = b
        else:
            nxt[b] = a
    loops = []
    used = set()
    for start in list(nxt.keys()):
        if start in used:
            continue
        loop = []
        v = start
        while v not in used and v in nxt:
            used.add(v)
            loop.append(v)
            v = nxt[v]
        if len(loop) >= 3:
            loops.append(loop)
    return loops


def plane_basis(n):
    n = np.asarray(n, dtype=np.float64)
    n = n / np.linalg.norm(n)
    a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(a, n)
    u /= np.linalg.norm(u)
    return n, u, np.cross(n, u)


def loop_area(verts, loop, normal):
    _, u, v = plane_basis(normal)
    p = verts[np.array(loop)]
    pu, pv = p @ u, p @ v
    return 0.5 * float(np.sum(pu * np.roll(pv, -1) - np.roll(pu, -1) * pv))


def distance_to_polylines(points, polylines):
    """The distance from each point to the nearest segment of the closed
    polylines (arrays of xyz)."""
    best = np.full(len(points), np.inf)
    for poly in polylines:
        a = poly
        b = np.roll(poly, -1, axis=0)
        ab = b - a
        ab2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-300)
        for i in range(len(a)):
            t = np.clip(((points - a[i]) @ ab[i]) / ab2[i], 0.0, 1.0)
            proj = a[i] + t[:, None] * ab[i]
            best = np.minimum(best, np.linalg.norm(points - proj, axis=1))
    return best


def mesh_rib_disc(name, verts, loops, chords, size):
    """Meshes the disc bounded by `loops` (vertex ids into `verts`; the
    first the outer loop, the rest holes) in a scratch model. `chords` are
    (vertex a, vertex b, node count) lines embedded in the disc. Returns
    (coords: dict scratch node tag -> xyz, rim: dict scratch node tag ->
    vertex id, chord_nodes: set of scratch node tags on chords, triangles:
    (k, 3) scratch node tags). The scratch model is removed; the caller
    restores the current model."""
    gmsh.model.add(name)
    gmsh.model.setCurrent(name)
    pt = {}

    def point(v):
        if v not in pt:
            p = verts[v]
            pt[v] = gmsh.model.geo.addPoint(float(p[0]), float(p[1]), float(p[2]), size)
        return pt[v]

    curve_loops = []
    rim_lines = []
    for loop in loops:
        lines = [gmsh.model.geo.addLine(point(loop[i]), point(loop[(i + 1) % len(loop)])) for i in range(len(loop))]
        rim_lines.extend(lines)
        curve_loops.append(gmsh.model.geo.addCurveLoop(lines))
    surf = gmsh.model.geo.addPlaneSurface(curve_loops)
    chord_lines = []
    for va, vb, n in chords:
        chord_lines.append((gmsh.model.geo.addLine(point(va), point(vb)), int(n)))
    gmsh.model.geo.synchronize()
    for line in rim_lines:
        gmsh.model.mesh.setTransfiniteCurve(line, 2)
    for line, n in chord_lines:
        gmsh.model.mesh.setTransfiniteCurve(line, max(n, 2))
        gmsh.model.mesh.embed(1, [line], 2, surf)
    gmsh.model.mesh.generate(2)
    # Every node of the scratch model: the disc's interior, its rim lines
    # and their points (a query on the surface alone leaves the points out).
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    coords = {int(t): xyz[3 * i:3 * i + 3].copy() for i, t in enumerate(tags)}
    rim = {}
    for v, ptag in pt.items():
        ntags, _, _ = gmsh.model.mesh.getNodes(0, ptag)
        for t in ntags:
            rim[int(t)] = v
    chord_nodes = set()
    for line, _ in chord_lines:
        ntags, _, _ = gmsh.model.mesh.getNodes(1, line, includeBoundary=True)
        chord_nodes.update(int(t) for t in ntags)
    types, _, conn = gmsh.model.mesh.getElements(2, surf)
    tris = np.zeros((0, 3), dtype=np.int64)
    for etype, nodes_flat in zip(types, conn):
        if int(etype) == GMSH_TRIANGLE:
            tris = np.array(nodes_flat, dtype=np.int64).reshape(-1, 3)
    # The scratch model is left in place (removing the only model has
    # crashed gmsh); the caller makes the body's model current again.
    return coords, rim, chord_nodes, tris


def rib_chords(verts, loops_by_rib, ribs, tol, size):
    """For each pair of ribs whose planes cross, the chord between the two
    rim vertices lying on both planes: {rib index: [(va, vb, node count)]}.
    The count is the same for both ribs (from the chord's length and the
    rib size), so the two discs carry the same nodes along it."""
    out = {i: [] for i in range(len(ribs))}
    normals = [np.asarray(r["normal"], dtype=np.float64) for r in ribs]
    for i in range(len(ribs)):
        for j in range(i + 1, len(ribs)):
            if abs(abs(np.dot(normals[i], normals[j])) - 1.0) < 1e-9:
                continue   # parallel: no crossing
            vi = set(v for loop in loops_by_rib[i] for v in loop)
            vj = set(v for loop in loops_by_rib[j] for v in loop)
            both = sorted(vi & vj)
            if len(both) < 2:
                continue
            # Several pairs when the section is multiply connected: pair
            # them along the line of intersection in order.
            line_dir = np.cross(normals[i], normals[j])
            line_dir /= np.linalg.norm(line_dir)
            both.sort(key=lambda v: float(verts[v] @ line_dir))
            for k in range(0, len(both) - 1, 2):
                va, vb = both[k], both[k + 1]
                length = float(np.linalg.norm(verts[vb] - verts[va]))
                n = max(2, int(round(length / size)) + 1) if size > 0.0 else 2
                out[i].append((va, vb, n))
                out[j].append((va, vb, n))
    return out


def run_job(job_dir):
    """Runs the job in `job_dir`. job.json:

        outer         the body's surface (.tri), closed and outward
        inner         the offset surface inside it (.tri; an empty or
                      missing one makes the whole body skin)
        core_size     target edge length deep inside the body (0: let gmsh
                      grow from the surfaces alone, which on a fine offset
                      surface means millions of tetrahedra)
        min_pocket_volume
                      pockets of the offset smaller than this are filled
                      solid rather than meshed as core (0: keep them all).
                      A remesh leaves slivers, and gmsh refuses a whole
                      body over one bad shell, so this keeps a speck of
                      debris from costing the mesh.
        min_size      floor on the edge length (0: none)
        algorithm     gmsh's Mesh.Algorithm3D: 1 Delaunay, 10 HXT (default)
        optimize      run gmsh's quality optimisation (default true)
        optimize_netgen
                      also run Netgen's (default true). On p38a's airframe
                      it takes the worst element from 0.0001 to 0.0030 and
                      the mean from 0.655 to 0.687, for 52 s against 15 s:
                      worth it for a mesh that goes into a solver, where
                      one near-flat element spoils the conditioning.
        size_from_boundary
                      let the surfaces' own triangle sizes seed the interior
                      (gmsh's Mesh.MeshSizeExtendFromBoundary; default
                      true). False makes the size core_size throughout,
                      graded to a finer surface only by the Delaunay
                      insertion -- which on a fuselage with 5 cm surface
                      edges and a 60 cm cell put a fan of slivers along the
                      whole skin (2026-11-05: 12% of the body's tetrahedra
                      with a dihedral angle under 10 degrees, against 2%
                      seeded). Use `grading` instead.
        grading       > 0: a GRADED size field (gmsh Distance + Threshold
                      fields) in place of either rule above: the size is
                      the surface's own median edge length at the surface
                      and grows linearly with the distance from it at this
                      rate (0.5: half a metre of edge per metre of depth)
                      until it reaches core_size. The transition layer then
                      has a bounded edge ratio whatever the cell, and the
                      deep interior is still the cell. Default 0 (off).
        ribs          [{id, normal: [x, y, z], offset, height}] planar
                      shells inside the volume (see the ribs section
                      above): the body's surface must already carry
                      vertices along each plane (cut by the application);
                      each disc partitions the body into compartments the
                      tetrahedra conform to; height > 0 reports only the
                      part of the disc within that distance of its rim as
                      rib faces (a stringer, a cap)
        rib_size      target edge length inside the ribs (0: the surface's
                      median edge)
        plane_tolerance
                      a vertex within this of a rib's plane is on it (0: a
                      ten-millionth of the body's diagonal)
        volume        true (default): mesh the body's volume (skin / core
                      tetrahedra, the ribs embedded in them); false: the
                      rib discs alone, for a thin-walled body of shells
                      with ribs and no tetrahedra
        outputs       {mesh, result, ribs}: file names to write; `ribs`
                      (default "ribs.bin") holds the rib shells: uint32
                      count, int32 node triples, int32 rib id per face

    With ribs the result also carries "rib_faces", "rib_area",
    "rib_faces_missing" (rib faces that are not a face of exactly two
    tetrahedra -- zero when the embedding conformed; the mesh is retried
    without Netgen's optimiser once when it is not) and "rib_notes".

    The result also carries the dihedral angles (`min_dihedral`,
    `max_dihedral`, degrees) and the counts `tets_dihedral_under_10` /
    `tets_dihedral_over_160`, the usual acceptance bands for a linear
    element (Shewchuk 2002: small dihedrals wreck the conditioning, large
    ones the gradient's accuracy); the application shows them and gates
    the solve on them.

    Returns the result dictionary (also written to outputs.result).
    """
    t_start = time.time()
    # utf-8-sig: the application writes the job without a byte-order mark,
    # but a hand-written one from PowerShell has one.
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job["outputs"]
    core_size = float(job.get("core_size", 0.0))
    min_size = float(job.get("min_size", 0.0))
    min_pocket = float(job.get("min_pocket_volume", 0.0))
    algorithm = int(job.get("algorithm", 10))
    optimize = bool(job.get("optimize", True))
    optimize_netgen = bool(job.get("optimize_netgen", True))
    size_from_boundary = bool(job.get("size_from_boundary", True))
    grading = float(job.get("grading", 0.0))
    ribs = list(job.get("ribs", []) or [])
    rib_size = float(job.get("rib_size", 0.0))
    want_volume = bool(job.get("volume", True))
    plane_tolerance = float(job.get("plane_tolerance", 0.0))

    ov, of = read_tri(os.path.join(job_dir, job["outer"]))
    check_closed(ov, of, "the body")
    outer_volume = surface_volume(ov, of)
    diagonal = float(np.linalg.norm(ov.max(axis=0) - ov.min(axis=0)))
    if plane_tolerance <= 0.0:
        plane_tolerance = 1e-7 * diagonal
    if ribs and rib_size <= 0.0:
        e = np.concatenate([ov[of[:, 1]] - ov[of[:, 0]], ov[of[:, 2]] - ov[of[:, 1]], ov[of[:, 0]] - ov[of[:, 2]]])
        rib_size = float(np.median(np.linalg.norm(e, axis=1)))

    inner_file = job.get("inner", "")
    iv = if_ = None
    inner_volume = 0.0
    if inner_file:
        path = os.path.join(job_dir, inner_file)
        if os.path.exists(path):
            iv, if_ = read_tri(path)
            if len(if_) == 0:
                iv = if_ = None
            else:
                check_closed(iv, if_, "the offset surface")
                inner_volume = surface_volume(iv, if_)
                if not inner_volume < outer_volume:
                    raise JobError("the offset surface is not smaller than the body.")
    log("  body %d triangles (volume %.6g, area %.6g); offset %d triangles (volume %.6g)"
        % (len(of), outer_volume, surface_area(ov, of), 0 if if_ is None else len(if_), inner_volume))

    # One discrete surface, and one surface loop, per closed shell.
    outer_components = shell_components(ov, of)
    if len(outer_components) != 1:
        raise JobError("the body is %d separate closed shells; mesh them one at a time." % len(outer_components))
    outer_shell = outer_components[0]
    inner_shells = shells(iv, if_) if if_ is not None else []
    if min_pocket > 0.0 and inner_shells:
        kept = [(sv, sf) for sv, sf in inner_shells if surface_volume(sv, sf) >= min_pocket]
        dropped_volume = sum(surface_volume(sv, sf) for sv, sf in inner_shells) - sum(surface_volume(sv, sf) for sv, sf in kept)
        if len(kept) != len(inner_shells):
            log("  %d pocket(s) smaller than %.3g (%.3g in all) filled solid" % (len(inner_shells) - len(kept), min_pocket, dropped_volume))
        inner_shells = kept
        inner_volume = sum(surface_volume(sv, sf) for sv, sf in inner_shells)

    # The ribs' rims on the body's surface, before gmsh is touched.
    rib_notes = []
    loops_by_rib = []
    for r in ribs:
        loops = rib_loops(ov, of, r["normal"], float(r["offset"]), plane_tolerance)
        # The outer loop first (the largest area), the holes after.
        loops.sort(key=lambda loop: -abs(loop_area(ov, loop, r["normal"])))
        if not loops:
            rib_notes.append("rib %d: its plane meets no edge of the surface (was the surface cut by it?)" % int(r["id"]))
        loops_by_rib.append(loops)
    chords = rib_chords(ov, loops_by_rib, ribs, plane_tolerance, rib_size) if ribs else {}

    def build(netgen):
        """The whole gmsh session: returns (nodes, tets, material, quality,
        rib_faces, rib_ids, rib_missing, seconds_mesh)."""
        gmsh.initialize()
        try:
            gmsh.option.setNumber("General.Terminal", 0)
            gmsh.option.setNumber("Mesh.Algorithm3D", algorithm)
            gmsh.option.setNumber("Mesh.Optimize", 1 if optimize else 0)
            gmsh.option.setNumber("Mesh.OptimizeNetgen", 1 if netgen else 0)
            # The sizes come from the surfaces and from core_size alone: no
            # point or curvature sizing, or gmsh would try to re-derive a size
            # field from geometry these discrete surfaces do not have.
            gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
            gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
            graded = grading > 0.0 and core_size > 0.0
            gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 1 if (size_from_boundary and not graded) else 0)
            if core_size > 0.0:
                gmsh.option.setNumber("Mesh.MeshSizeMax", core_size)
                if not size_from_boundary and not graded:
                    # No size source but the cap: say so, or gmsh takes its
                    # own default (1) where the boundary would have seeded it.
                    gmsh.option.setNumber("Mesh.MeshSizeMin", min_size if min_size > 0.0 else core_size)
            if min_size > 0.0:
                gmsh.option.setNumber("Mesh.MeshSizeMin", min_size)

            gmsh.model.add("body")
            # The rib discs, in scratch models of their own (their own size),
            # the body's model made current again after.
            disc_meshes = []
            if ribs:
                saved_max = gmsh.option.getNumber("Mesh.MeshSizeMax")
                gmsh.option.setNumber("Mesh.MeshSizeMax", rib_size)
                gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 1)
                for i, r in enumerate(ribs):
                    if not loops_by_rib[i]:
                        disc_meshes.append(None)
                        continue
                    disc_meshes.append(mesh_rib_disc("rib_%d" % i, ov, loops_by_rib[i], chords.get(i, []), rib_size))
                gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
                gmsh.option.setNumber("Mesh.MeshSizeMax", saved_max)
            gmsh.model.setCurrent("body")
            sv, sf, used = outer_shell
            if ribs and inner_shells:
                raise JobError("ribs and an offset surface together are not supported: mesh the ribbed body without the offset.")

            # Every rib's disc PARTITIONS the body: the disc is a wall between
            # two compartments, meshed as a boundary of both, so the
            # tetrahedra conform to it by construction like the skin / core
            # interface. (Embedding a discrete surface in the volume instead
            # crashed gmsh.) A limited height keeps only the part of the disc
            # within it as RIB faces; the rest of the wall stays in the mesh
            # as an interface between tetrahedra of one material, which the
            # solver does not see.
            partition = [i for i, r in enumerate(ribs) if disc_meshes[i] is not None]
            part_normals = [np.asarray(ribs[i]["normal"], dtype=np.float64) for i in partition]
            part_offsets = [float(ribs[i]["offset"]) for i in partition]

            def compartment_keys(points, skip=None):
                """The side of every partition plane each point is on, as a
                (k, n) array of +-1 (0 for the plane `skip`, the disc's own)."""
                if not partition:
                    return np.zeros((len(points), 0), dtype=np.int64)
                cols = []
                for i, n, d in zip(partition, part_normals, part_offsets):
                    if i == skip:
                        cols.append(np.zeros(len(points), dtype=np.int64))
                    else:
                        cols.append(np.where(points @ n - d >= 0.0, 1, -1).astype(np.int64))
                return np.stack(cols, axis=1)

            # The body's nodes, all on the first surface entity (an element of
            # another entity may refer to them: gmsh keys nodes by tag).
            tag = 1
            node = 1
            node_tags_outer = np.arange(1, 1 + len(sv), dtype=np.uint64)
            tag_of_vertex = np.full(len(ov), -1, dtype=np.int64)
            tag_of_vertex[used] = np.arange(1, 1 + len(used))
            face_tags = (sf.astype(np.int64) + 1)   # the shell's own indices -> tags
            centroids = (sv[sf[:, 0]] + sv[sf[:, 1]] + sv[sf[:, 2]]) / 3.0
            outer_keys = compartment_keys(centroids)
            compartments = {}   # key tuple -> {"outer": [tags], "discs": [tags]}
            first = True
            for key in sorted(set(map(tuple, outer_keys.tolist()))):
                sel = np.all(outer_keys == np.array(key, dtype=np.int64), axis=1) if len(key) else np.ones(len(sf), dtype=bool)
                gmsh.model.addDiscreteEntity(2, tag)
                if first:
                    gmsh.model.mesh.addNodes(2, tag, node_tags_outer, sv.reshape(-1))
                    first = False
                gmsh.model.mesh.addElementsByType(tag, GMSH_TRIANGLE, [], face_tags[sel].astype(np.uint64).reshape(-1))
                compartments.setdefault(key, {"outer": [], "discs": []})["outer"].append(tag)
                tag += 1
            node = 1 + len(sv)
            inner_loops = []
            for isv, isf in inner_shells:
                node = add_surface(isv, isf, tag, node)
                inner_loops.append(gmsh.model.geo.addSurfaceLoop([tag]))
                tag += 1

            # The ribs: rim nodes the body's own, chord nodes shared between
            # the discs that cross, the rest new. A partitioning disc is split
            # by the other partition planes into the walls between
            # compartments; an embedded frame into the parts inside each.
            rib_tris = [np.zeros((0, 3), dtype=np.int64) for _ in ribs]   # per rib: (k, 3) node tags
            shared = {}     # rounded xyz -> node tag, for the chords
            key_scale = 1.0 / max(1e-9 * diagonal, 1e-300)
            for i, r in enumerate(ribs):
                disc = disc_meshes[i]
                if disc is None:
                    continue
                coords, rim, chord_nodes, tris = disc
                if not len(tris):
                    continue
                keep = np.ones(len(tris), dtype=bool)
                if float(r.get("height", 0.0)) > 0.0:
                    # A limited height: only the triangles within it of the
                    # rim are rib faces (their centroids); the whole disc
                    # still walls the compartments.
                    cent = np.array([(coords[int(a)] + coords[int(b)] + coords[int(c)]) / 3.0 for a, b, c in tris])
                    polylines = [ov[np.array(loop)] for loop in loops_by_rib[i]]
                    keep = distance_to_polylines(cent, polylines) <= float(r["height"])
                tag_of_scratch = {}
                new_tags, new_xyz = [], []
                for st in sorted(coords):
                    if st in rim:
                        t = int(tag_of_vertex[rim[st]])
                        if t < 0:
                            raise JobError("rib %d: a rim vertex is not on the body's surface." % int(r["id"]))
                        tag_of_scratch[st] = t
                        continue
                    key = tuple(np.round(coords[st] * key_scale).astype(np.int64)) if st in chord_nodes else None
                    if key is not None and key in shared:
                        tag_of_scratch[st] = shared[key]
                        continue
                    tag_of_scratch[st] = node
                    new_tags.append(node)
                    new_xyz.extend(float(v) for v in coords[st])
                    if key is not None:
                        shared[key] = node
                    node += 1
                mapped = np.array([[tag_of_scratch[int(v)] for v in row] for row in tris], dtype=np.int64).reshape(-1, 3)
                rib_tris[i] = mapped[keep]
                cent = np.array([(coords[int(a)] + coords[int(b)] + coords[int(c)]) / 3.0 for a, b, c in tris])
                keys = compartment_keys(cent, skip=i)
                first_part = True
                col = partition.index(i)
                for key in sorted(set(map(tuple, keys.tolist()))):
                    sel = np.all(keys == np.array(key, dtype=np.int64), axis=1)
                    gmsh.model.addDiscreteEntity(2, tag)
                    if first_part and new_tags:
                        gmsh.model.mesh.addNodes(2, tag, np.array(new_tags, dtype=np.uint64), np.array(new_xyz, dtype=np.float64))
                    first_part = False
                    gmsh.model.mesh.addElementsByType(tag, GMSH_TRIANGLE, [], mapped[sel].astype(np.uint64).reshape(-1))
                    # The wall between the compartments on either side of
                    # plane i whose other sides match.
                    for side in (1, -1):
                        k2 = list(key)
                        k2[col] = side
                        compartments.setdefault(tuple(k2), {"outer": [], "discs": []})["discs"].append(tag)
                    tag += 1

            volumes = []
            seconds_mesh = 0.0
            if want_volume:
                # One volume per compartment, bounded by its piece of the skin
                # and the walls that close it (every pocket of an offset a hole
                # of the one compartment there is then, and a core volume of its
                # own). A wall belongs to both its compartments, so its
                # triangles are shared and the interface conforms by construction.
                vtag = 1
                for key in sorted(compartments):
                    comp = compartments[key]
                    if not comp["outer"]:
                        raise JobError("the rib planes cut off a compartment with no skin of its own (planes nearly coincident, or a rim loop missed).")
                    loop = gmsh.model.geo.addSurfaceLoop(comp["outer"] + comp["discs"])
                    gmsh.model.geo.addVolume([loop] + inner_loops, vtag)
                    volumes.append((vtag, 1))
                    comp["volume"] = vtag
                    vtag += 1
                for i, loop in enumerate(inner_loops):
                    gmsh.model.geo.addVolume([loop], vtag)
                    volumes.append((vtag, 2))
                    vtag += 1
                gmsh.model.geo.synchronize()
                if partition:
                    log("  %d rib plane(s) partition the body into %d compartment(s)" % (len(partition), len(compartments)))
                if inner_shells:
                    log("  %d pocket(s) in the body: %s triangles" % (len(inner_shells), ", ".join(str(len(isf)) for _, isf in inner_shells)))

                if graded:
                    # The graded field: the surfaces' median edge at the surfaces,
                    # growing at `grading` per unit distance until it is the core
                    # size. gmsh's Threshold is linear between DistMin and DistMax.
                    e = np.concatenate([ov[of[:, 1]] - ov[of[:, 0]], ov[of[:, 2]] - ov[of[:, 1]], ov[of[:, 0]] - ov[of[:, 2]]])
                    h_surface = float(np.median(np.linalg.norm(e, axis=1)))
                    if min_size > 0.0:
                        h_surface = max(h_surface, min_size)
                    h_surface = min(h_surface, core_size)
                    dist = gmsh.model.mesh.field.add("Distance")
                    gmsh.model.mesh.field.setNumbers(dist, "SurfacesList", list(range(1, 2 + len(inner_shells))))
                    gmsh.model.mesh.field.setNumber(dist, "Sampling", 20)
                    thr = gmsh.model.mesh.field.add("Threshold")
                    gmsh.model.mesh.field.setNumber(thr, "InField", dist)
                    gmsh.model.mesh.field.setNumber(thr, "SizeMin", h_surface)
                    gmsh.model.mesh.field.setNumber(thr, "SizeMax", core_size)
                    gmsh.model.mesh.field.setNumber(thr, "DistMin", 0.0)
                    gmsh.model.mesh.field.setNumber(thr, "DistMax", max((core_size - h_surface) / grading, 1e-9))
                    gmsh.model.mesh.field.setAsBackgroundMesh(thr)
                    log("  graded size field: %.4g at the surface, %.4g at %.4g in, the cell %.4g beyond" % (h_surface, core_size, (core_size - h_surface) / grading, core_size))

                t_mesh = time.time()
                gmsh.model.mesh.generate(3)
                seconds_mesh = time.time() - t_mesh
            else:
                gmsh.model.geo.synchronize()

            # Out: one node array, the tetrahedra of each volume with its material.
            node_tags, coords, _ = gmsh.model.mesh.getNodes()
            node_tags = np.array(node_tags, dtype=np.int64)
            nodes = np.array(coords, dtype=np.float64).reshape(-1, 3)
            order = np.argsort(node_tags)
            node_tags, nodes = node_tags[order], nodes[order]
            # Tag -> row. gmsh's tags are 1-based and may have gaps.
            lookup = np.full(int(node_tags.max()) + 1, -1, dtype=np.int64)
            lookup[node_tags] = np.arange(len(node_tags))

            tets_all, material_all, quality_all = [], [], []
            for vtag, mat in volumes:
                types, tags, conn = gmsh.model.mesh.getElements(3, vtag)
                found = False
                for etype, etags, nodes_flat in zip(types, tags, conn):
                    if int(etype) != GMSH_TET:
                        continue
                    t = lookup[np.array(nodes_flat, dtype=np.int64).reshape(-1, 4)]
                    tets_all.append(t)
                    material_all.append(np.full(len(t), mat, dtype=np.int32))
                    # Signed inverse condition number: 1 is the regular
                    # tetrahedron, 0 is flat, negative is inverted.
                    quality_all.append(np.array(gmsh.model.mesh.getElementQualities(etags, "minSICN"), dtype=np.float64))
                    found = True
                if not found:
                    raise JobError("gmsh meshed no tetrahedra into the %s." % ("skin" if mat == 1 else "core"))
            tets = np.concatenate(tets_all).astype(np.int32) if tets_all else np.zeros((0, 4), dtype=np.int32)
            material = np.concatenate(material_all) if material_all else np.zeros(0, dtype=np.int32)
            quality = np.concatenate(quality_all) if quality_all else np.zeros(0)
            # The rib faces as node rows, and whether each is a face of two
            # tetrahedra (the embedding conformed).
            rib_rows, rib_ids = [], []
            for i, r in enumerate(ribs):
                rows = lookup[rib_tris[i]] if len(rib_tris[i]) else np.zeros((0, 3), dtype=np.int64)
                rib_rows.append(rows)
                rib_ids.append(np.full(len(rows), int(r["id"]), dtype=np.int32))
            rib_faces = np.concatenate(rib_rows).astype(np.int32) if rib_rows else np.zeros((0, 3), dtype=np.int32)
            rib_id = np.concatenate(rib_ids) if rib_ids else np.zeros(0, dtype=np.int32)
            rib_missing = 0
            if len(rib_faces) and len(tets):
                tf = np.sort(np.concatenate([tets[:, [0, 1, 2]], tets[:, [0, 1, 3]], tets[:, [1, 2, 3]], tets[:, [0, 2, 3]]]), axis=1)
                keys, counts = np.unique(tf, axis=0, return_counts=True)
                two = set(map(tuple, keys[counts == 2]))
                rib_missing = int(sum(1 for row in np.sort(rib_faces, axis=1) if tuple(row) not in two))
        finally:
            gmsh.finalize()
        return nodes, tets, material, quality, rib_faces, rib_id, rib_missing, seconds_mesh

    nodes, tets, material, quality, rib_faces, rib_id, rib_missing, seconds_mesh = build(optimize_netgen)
    if rib_missing > 0 and optimize_netgen and want_volume:
        log("  %d rib face(s) are not faces of the tetrahedra after Netgen's optimisation: meshing again without it" % rib_missing)
        nodes, tets, material, quality, rib_faces, rib_id, rib_missing, seconds_mesh = build(False)
        rib_notes.append("meshed without Netgen's optimiser so the ribs conform")
    if rib_missing > 0:
        rib_notes.append("%d rib face(s) are not faces of exactly two tetrahedra: the embedding did not conform there" % rib_missing)

    # The application's convention is a positive volume (tet.hpp); gmsh can
    # hand back a few the other way round, so they are swapped, not dropped.
    vol = tet_volumes(nodes, tets) if len(tets) else np.zeros(0)
    flipped = int((vol < 0.0).sum())
    if flipped:
        tets[vol < 0.0] = tets[vol < 0.0][:, [0, 2, 1, 3]]
        vol = tet_volumes(nodes, tets)
    degenerate = int((vol <= 0.0).sum())

    skin = material == 1
    core = material == 2
    skin_volume = float(vol[skin].sum()) if len(tets) else 0.0
    core_volume = float(vol[core].sum()) if len(tets) else 0.0
    write_tet(os.path.join(job_dir, outputs["mesh"]), nodes, tets, material)
    rib_area = surface_area(nodes, rib_faces) if len(rib_faces) else 0.0
    with open(os.path.join(job_dir, outputs.get("ribs", "ribs.bin")), "wb") as f:
        f.write(struct.pack("<I", len(rib_faces)))
        f.write(np.ascontiguousarray(rib_faces, dtype="<i4").tobytes())
        f.write(np.ascontiguousarray(rib_id, dtype="<i4").tobytes())
    dih = dihedral_angles(nodes, tets)
    d_min, d_max = (dih.min(axis=1), dih.max(axis=1)) if len(tets) else (np.zeros(0), np.zeros(0))

    result = {
        "ok": True,
        "nodes": int(len(nodes)),
        "tets": int(len(tets)),
        "skin_tets": int(skin.sum()),
        "core_tets": int(core.sum()),
        "outer_volume": outer_volume,
        "inner_volume": inner_volume,
        "skin_volume": skin_volume,
        "core_volume": core_volume,
        # What the whole exercise is for: the tetrahedra must fill the body.
        "volume_error": (abs(skin_volume + core_volume - outer_volume) / abs(outer_volume)) if (outer_volume and want_volume) else 0.0,
        "core_volume_error": (abs(core_volume - inner_volume) / abs(outer_volume)) if (outer_volume and if_ is not None and want_volume) else 0.0,
        "flipped": flipped,
        "degenerate": degenerate,
        "min_quality": float(quality.min()) if len(quality) else 0.0,
        "mean_quality": float(quality.mean()) if len(quality) else 0.0,
        "poor_quality": int((quality < 0.1).sum()),
        "min_dihedral": float(dih.min()) if len(tets) else 0.0,
        "max_dihedral": float(dih.max()) if len(tets) else 0.0,
        "tets_dihedral_under_10": int((d_min < 10.0).sum()),
        "tets_dihedral_over_160": int((d_max > 160.0).sum()),
        "surface_nodes": int(len(ov) + (0 if iv is None else len(iv))),
        "rib_faces": int(len(rib_faces)),
        "rib_area": rib_area,
        "rib_faces_missing": int(rib_missing),
        "rib_notes": rib_notes,
        "seconds_mesh": seconds_mesh,
        "seconds": time.time() - t_start,
        "gmsh": gmsh.GMSH_API_VERSION if hasattr(gmsh, "GMSH_API_VERSION") else "",
    }
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  %d nodes, %d skin + %d core tetrahedra; skin %.6g + core %.6g = %.6g vs body %.6g (relative %.2e); quality min %.3f mean %.3f, %d under 0.1; dihedral %.2f..%.2f deg, %d under 10, %d over 160; %d flipped, %d degenerate; %d rib faces (%d not conforming); %.2f s"
        % (result["nodes"], result["skin_tets"], result["core_tets"], skin_volume, core_volume,
           skin_volume + core_volume, outer_volume, result["volume_error"], result["min_quality"],
           result["mean_quality"], result["poor_quality"], result["min_dihedral"], result["max_dihedral"],
           result["tets_dihedral_under_10"], result["tets_dihedral_over_160"], flipped, degenerate, result["rib_faces"], result["rib_faces_missing"], result["seconds"]))
    return result


def dihedral_angles(nodes, tets):
    """The six interior dihedral angles of every tetrahedron, degrees,
    (n, 6): the angle between the two faces at each edge."""
    if len(tets) == 0:
        return np.zeros((0, 6))
    p = nodes[tets]
    faces = [(1, 2, 3), (0, 3, 2), (0, 1, 3), (0, 2, 1)]   # face k opposite vertex k
    n = np.empty((len(tets), 4, 3))
    for k, (a, b, c) in enumerate(faces):
        nk = np.cross(p[:, b] - p[:, a], p[:, c] - p[:, a])
        s = np.einsum("ij,ij->i", nk, p[:, k] - p[:, a])
        nk[s > 0] *= -1.0                                   # outward
        n[:, k] = nk / np.maximum(np.linalg.norm(nk, axis=1), 1e-300)[:, None]
    out = np.empty((len(tets), 6))
    for j, (a, b) in enumerate([(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]):
        c = -np.einsum("ij,ij->i", n[:, a], n[:, b])
        out[:, j] = np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))
    return out


def _write_failure(job_dir, message):
    try:
        with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
            outputs = json.load(f).get("outputs", {})
        path = os.path.join(job_dir, outputs.get("result", "result.json"))
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"ok": False, "error": message}, f, indent=1)
    except Exception:
        pass


def main(argv):
    if len(argv) != 2:
        log("usage: tet_gmsh.py <job directory>")
        return 2
    job_dir = argv[1]
    try:
        run_job(job_dir)
        return 0
    except JobError as exc:
        _write_failure(job_dir, str(exc))
        log("error: %s" % exc)
        return 1
    except Exception as exc:
        import traceback
        log(traceback.format_exc())
        _write_failure(job_dir, "%s: %s" % (type(exc).__name__, exc))
        log("error: %s: %s" % (type(exc).__name__, exc))
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
