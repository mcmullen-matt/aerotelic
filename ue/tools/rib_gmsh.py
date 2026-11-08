"""The RIB DISCS of a body -- planar shells inside it -- meshed on gmsh.

Run by the application as a separate process in the WingDesigner
micromamba environment, where gmsh is installed (``pip install gmsh``)::

    python rib_gmsh.py <job directory>

The job directory holds ``job.json`` (see ``run_job`` for the keys) and
the body's own triangulation as ``outer.tri``. The script writes the
nodes, the rib faces and ``result.json``.

THE MODEL (2026-11-08). Every structural element of a body is a SHELL:
the skin is the body's own surface triangles, and a rib or spar is the
disc a plane network's plane cuts out of the body. There are no
tetrahedra. The application cuts the body's surface by each rib plane
beforehand (core mesh_cut.hpp), so the surface has vertices exactly along
the plane and the rib's rim IS those vertices: the rib shells and the skin
shells share their nodes, and the structural solver (structural_fea.py)
sees one connected shell structure.

How a disc is meshed. The rim loops are read off the surface (the vertices
on the plane, chained along the surface's edges that separate the two
sides; the largest loop is the outer boundary, the others holes). The disc
is meshed in a scratch gmsh model from the rim's vertices, every rim
segment a TRANSFINITE line with its two endpoints only, so gmsh adds no
node on the rim -- the rim nodes stay the surface's own. Two crossing
networks (a spar across the ribs) share the CHORD where their planes meet
inside the body: the chord between the two rim vertices lying on both
planes is embedded in both discs with the same transfinite node count, so
the two discs carry the same nodes along it, and those nodes are merged
by position when the discs are assembled. A rib of limited ``height``
(a frame, a cap) keeps only the triangles within that distance of the rim.

FILES. ``outer.tri`` is the application's format (gmsh_common.py:
``AEROTRI1``, counts, float64 xyz, int32 abc). ``nodes.bin`` is 8 bytes
``AERONOD1``, uint32 node count, float64 xyz per node -- the SURFACE'S
VERTICES FIRST, in their own order, then the ribs' interior nodes (the
application relies on that order to splice the rib faces onto the body's
grid). ``ribs.bin`` is uint32 face count, int32 node triples (rows into
nodes.bin), then an int32 rib id per face.

History: this was the ribs half of tools/tet_gmsh.py, the skin / core
tetrahedral mesher, whose volume path (tetrahedra, offset surfaces,
compartments, Netgen, quality statistics) was dropped on 2026-11-08 when
the application stopped supporting tetrahedral structural models. The
shared surface helpers are in gmsh_common.py.
"""

import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gmsh_common as tg  # noqa: E402  (bootstraps Library\\bin before numpy and gmsh)

import numpy as np  # noqa: E402
import gmsh  # noqa: E402

JobError = tg.JobError
log = tg.log
GMSH_TRIANGLE = tg.GMSH_TRIANGLE

NODES_MAGIC = b"AERONOD1"


# ---------------------------------------------------------------------------
# The rims
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# The discs
# ---------------------------------------------------------------------------

def mesh_rib_disc(name, verts, loops, chords, size):
    """Meshes the disc bounded by `loops` (vertex ids into `verts`; the
    first the outer loop, the rest holes) in a scratch model. `chords` are
    (vertex a, vertex b, node count) lines embedded in the disc. Returns
    (coords: dict scratch node tag -> xyz, rim: dict scratch node tag ->
    vertex id, chord_nodes: set of scratch node tags on chords, triangles:
    (k, 3) scratch node tags). The scratch model is left in place
    (removing the only model has crashed gmsh)."""
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


# ---------------------------------------------------------------------------
# The job
# ---------------------------------------------------------------------------

def write_nodes(path, nodes):
    with open(path, "wb") as f:
        f.write(NODES_MAGIC)
        f.write(struct.pack("<I", len(nodes)))
        f.write(np.ascontiguousarray(nodes, dtype="<f8").tobytes())


def write_ribs(path, faces, ids):
    with open(path, "wb") as f:
        f.write(struct.pack("<I", len(faces)))
        f.write(np.ascontiguousarray(faces, dtype="<i4").tobytes())
        f.write(np.ascontiguousarray(ids, dtype="<i4").tobytes())


def run_job(job_dir):
    """Runs the job in `job_dir`. job.json:

        outer         the body's surface (.tri), closed and outward, one
                      connected shell, already cut by every rib plane
        ribs          [{id, normal: [x, y, z], offset, height}]: the rib
                      planes normal . x = offset; `id` is the plane
                      network's (several ribs share it, and the faces carry
                      it); height > 0 keeps only the part of the disc
                      within that distance of its rim (a frame, a cap)
        rib_size      target edge length inside the ribs (0: the surface's
                      median edge)
        plane_tolerance
                      a vertex within this of a rib's plane is on it (0: a
                      ten-millionth of the body's diagonal)
        outputs       {nodes, result, ribs}: file names to write (defaults
                      nodes.bin, result.json, ribs.bin)

    result.json: ok, nodes (the count written), surface_nodes (the first
    that many are the surface's vertices), outer_volume, rib_faces,
    rib_area, rib_notes (a list of strings: a plane that met no edge of
    the surface, and the like), seconds, gmsh (the API version). A failure
    writes {ok: false, error} instead and exits 1.

    Returns the result dictionary.
    """
    t_start = time.time()
    # utf-8-sig: the application writes the job without a byte-order mark,
    # but a hand-written one from PowerShell has one.
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job.get("outputs", {}) or {}
    ribs = list(job.get("ribs", []) or [])
    rib_size = float(job.get("rib_size", 0.0))
    plane_tolerance = float(job.get("plane_tolerance", 0.0))

    ov, of = tg.read_tri(os.path.join(job_dir, job["outer"]))
    tg.check_closed(ov, of, "the body")
    outer_volume = tg.surface_volume(ov, of)
    diagonal = float(np.linalg.norm(ov.max(axis=0) - ov.min(axis=0)))
    if plane_tolerance <= 0.0:
        plane_tolerance = 1e-7 * diagonal
    if rib_size <= 0.0:
        e = np.concatenate([ov[of[:, 1]] - ov[of[:, 0]], ov[of[:, 2]] - ov[of[:, 1]], ov[of[:, 0]] - ov[of[:, 2]]])
        rib_size = float(np.median(np.linalg.norm(e, axis=1)))
    log("  body %d vertices, %d triangles (volume %.6g, area %.6g); %d rib plane(s), rib size %.4g"
        % (len(ov), len(of), outer_volume, tg.surface_area(ov, of), len(ribs), rib_size))

    # One body: a section through two separate shells would read as a disc
    # with a hole, which is not what two sections are.
    n_shells = len(tg.shell_components(ov, of))
    if n_shells != 1:
        raise JobError("the body is %d separate closed shells; mesh them one at a time." % n_shells)

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

    # The nodes: the surface's vertices first, in order; the ribs' interior
    # nodes after, a chord node once however many discs carry it.
    new_xyz = []                 # the interior nodes, in order of creation
    shared = {}                  # rounded xyz -> node row, for the chords
    key_scale = 1.0 / max(1e-9 * diagonal, 1e-300)
    rib_rows, rib_ids = [], []
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMax", rib_size)
        for i, r in enumerate(ribs):
            if not loops_by_rib[i]:
                continue
            coords, rim, chord_nodes, tris = mesh_rib_disc("rib_%d" % i, ov, loops_by_rib[i], chords.get(i, []), rib_size)
            if not len(tris):
                rib_notes.append("rib %d: gmsh meshed no triangle into its disc" % int(r["id"]))
                continue
            keep = np.ones(len(tris), dtype=bool)
            if float(r.get("height", 0.0)) > 0.0:
                # A limited height: only the triangles within it of the rim
                # (their centroids) are rib faces.
                cent = np.array([(coords[int(a)] + coords[int(b)] + coords[int(c)]) / 3.0 for a, b, c in tris])
                polylines = [ov[np.array(loop)] for loop in loops_by_rib[i]]
                keep = distance_to_polylines(cent, polylines) <= float(r["height"])
            row_of_scratch = {}
            for st in sorted(coords):
                if st in rim:
                    row_of_scratch[st] = int(rim[st])       # the surface's own vertex
                    continue
                key = tuple(np.round(coords[st] * key_scale).astype(np.int64)) if st in chord_nodes else None
                if key is not None and key in shared:
                    row_of_scratch[st] = shared[key]        # a chord node another disc made
                    continue
                row = len(ov) + len(new_xyz)
                row_of_scratch[st] = row
                new_xyz.append(coords[st])
                if key is not None:
                    shared[key] = row
            mapped = np.array([[row_of_scratch[int(v)] for v in row] for row in tris], dtype=np.int64).reshape(-1, 3)
            rib_rows.append(mapped[keep])
            rib_ids.append(np.full(int(keep.sum()), int(r["id"]), dtype=np.int32))
    finally:
        gmsh.finalize()

    rib_faces = np.concatenate(rib_rows).astype(np.int64) if rib_rows else np.zeros((0, 3), dtype=np.int64)
    rib_id = np.concatenate(rib_ids) if rib_ids else np.zeros(0, dtype=np.int32)
    # Only the interior nodes a kept face uses are written (a frame's
    # middle is dropped whole): a node no shell touches would be a free
    # body in the solver.
    interior = np.array(new_xyz, dtype=np.float64).reshape(-1, 3)
    used = np.zeros(len(interior), dtype=bool)
    if len(rib_faces):
        inner = rib_faces[rib_faces >= len(ov)] - len(ov)
        used[inner] = True
    remap = np.full(len(ov) + len(interior), -1, dtype=np.int64)
    remap[:len(ov)] = np.arange(len(ov))
    remap[len(ov) + np.nonzero(used)[0]] = len(ov) + np.arange(int(used.sum()))
    nodes = np.concatenate([ov, interior[used]]) if used.any() else np.array(ov, dtype=np.float64)
    rib_faces = remap[rib_faces] if len(rib_faces) else rib_faces
    rib_area = tg.surface_area(nodes, rib_faces.astype(np.int64)) if len(rib_faces) else 0.0

    write_nodes(os.path.join(job_dir, outputs.get("nodes", "nodes.bin")), nodes)
    write_ribs(os.path.join(job_dir, outputs.get("ribs", "ribs.bin")), rib_faces, rib_id)
    result = {
        "ok": True,
        "nodes": int(len(nodes)),
        "surface_nodes": int(len(ov)),
        "outer_volume": outer_volume,
        "rib_faces": int(len(rib_faces)),
        "rib_area": rib_area,
        "rib_notes": rib_notes,
        "seconds": time.time() - t_start,
        "gmsh": gmsh.GMSH_API_VERSION if hasattr(gmsh, "GMSH_API_VERSION") else "",
    }
    with open(os.path.join(job_dir, outputs.get("result", "result.json")), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  %d nodes (%d the surface's, %d in the ribs); %d rib faces, area %.6g; %d note(s); %.2f s"
        % (result["nodes"], result["surface_nodes"], result["nodes"] - result["surface_nodes"], result["rib_faces"], rib_area, len(rib_notes), result["seconds"]))
    return result


def main(argv):
    if len(argv) != 2:
        log("usage: rib_gmsh.py <job directory>")
        return 2
    job_dir = argv[1]
    try:
        run_job(job_dir)
        return 0
    except JobError as exc:
        tg.write_failure(job_dir, str(exc))
        log("error: %s" % exc)
        return 1
    except Exception as exc:
        import traceback
        log(traceback.format_exc())
        tg.write_failure(job_dir, "%s: %s" % (type(exc).__name__, exc))
        log("error: %s: %s" % (type(exc).__name__, exc))
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
