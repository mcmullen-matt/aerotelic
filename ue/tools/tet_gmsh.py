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
        out.append((np.ascontiguousarray(verts[used]), np.ascontiguousarray(remap[sub_faces].astype(np.int32))))
    return out


def add_surface(verts, faces, tag, first_node):
    """A discrete surface entity holding exactly these triangles. Returns
    the next free node tag."""
    gmsh.model.addDiscreteEntity(2, tag)
    node_tags = np.arange(first_node, first_node + len(verts), dtype=np.uint64)
    gmsh.model.mesh.addNodes(2, tag, node_tags, verts.reshape(-1))
    gmsh.model.mesh.addElementsByType(tag, GMSH_TRIANGLE, [], (faces + first_node).astype(np.uint64).reshape(-1))
    return first_node + len(verts)


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
        outputs       {mesh, result}: file names to write

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

    ov, of = read_tri(os.path.join(job_dir, job["outer"]))
    check_closed(ov, of, "the body")
    outer_volume = surface_volume(ov, of)

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

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("Mesh.Algorithm3D", algorithm)
        gmsh.option.setNumber("Mesh.Optimize", 1 if optimize else 0)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1 if optimize_netgen else 0)
        # The sizes come from the surfaces and from core_size alone: no
        # point or curvature sizing, or gmsh would try to re-derive a size
        # field from geometry these discrete surfaces do not have.
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 1)
        if core_size > 0.0:
            gmsh.option.setNumber("Mesh.MeshSizeMax", core_size)
        if min_size > 0.0:
            gmsh.option.setNumber("Mesh.MeshSizeMin", min_size)
        gmsh.model.add("body")

        # One discrete surface, and one surface loop, per closed shell.
        outer_shells = shells(ov, of)
        if len(outer_shells) != 1:
            raise JobError("the body is %d separate closed shells; mesh them one at a time." % len(outer_shells))
        inner_shells = shells(iv, if_) if if_ is not None else []
        if min_pocket > 0.0 and inner_shells:
            kept = [(sv, sf) for sv, sf in inner_shells if surface_volume(sv, sf) >= min_pocket]
            dropped_volume = sum(surface_volume(sv, sf) for sv, sf in inner_shells) - sum(surface_volume(sv, sf) for sv, sf in kept)
            if len(kept) != len(inner_shells):
                log("  %d pocket(s) smaller than %.3g (%.3g in all) filled solid" % (len(inner_shells) - len(kept), min_pocket, dropped_volume))
            inner_shells = kept
            inner_volume = sum(surface_volume(sv, sf) for sv, sf in inner_shells)

        tag = 1
        node = 1
        node = add_surface(ov, of, tag, node)
        outer_loop = gmsh.model.geo.addSurfaceLoop([tag])
        inner_loops = []
        for sv, sf in inner_shells:
            tag += 1
            node = add_surface(sv, sf, tag, node)
            inner_loops.append(gmsh.model.geo.addSurfaceLoop([tag]))

        # The skin is the body's loop with every pocket as a hole; each
        # pocket is a core volume of its own. The pocket's surface belongs
        # to both, so its triangles are shared and the interface between
        # skin and core conforms by construction.
        gmsh.model.geo.addVolume([outer_loop] + inner_loops, 1)
        volumes = [(1, 1)]  # (entity tag, material)
        for i, loop in enumerate(inner_loops):
            gmsh.model.geo.addVolume([loop], 2 + i)
            volumes.append((2 + i, 2))
        gmsh.model.geo.synchronize()
        if inner_shells:
            log("  %d pocket(s) in the body: %s triangles" % (len(inner_shells), ", ".join(str(len(sf)) for _, sf in inner_shells)))

        t_mesh = time.time()
        gmsh.model.mesh.generate(3)
        seconds_mesh = time.time() - t_mesh

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
        for tag, material in volumes:
            types, tags, conn = gmsh.model.mesh.getElements(3, tag)
            found = False
            for etype, etags, nodes_flat in zip(types, tags, conn):
                if int(etype) != GMSH_TET:
                    continue
                t = lookup[np.array(nodes_flat, dtype=np.int64).reshape(-1, 4)]
                tets_all.append(t)
                material_all.append(np.full(len(t), material, dtype=np.int32))
                # Signed inverse condition number: 1 is the regular
                # tetrahedron, 0 is flat, negative is inverted.
                quality_all.append(np.array(gmsh.model.mesh.getElementQualities(etags, "minSICN"), dtype=np.float64))
                found = True
            if not found:
                raise JobError("gmsh meshed no tetrahedra into the %s." % ("skin" if material == 1 else "core"))
        tets = np.concatenate(tets_all).astype(np.int32)
        material = np.concatenate(material_all)
        quality = np.concatenate(quality_all)
    finally:
        gmsh.finalize()

    # The application's convention is a positive volume (tet.hpp); gmsh can
    # hand back a few the other way round, so they are swapped, not dropped.
    vol = tet_volumes(nodes, tets)
    flipped = int((vol < 0.0).sum())
    if flipped:
        tets[vol < 0.0] = tets[vol < 0.0][:, [0, 2, 1, 3]]
        vol = tet_volumes(nodes, tets)
    degenerate = int((vol <= 0.0).sum())

    skin = material == 1
    core = material == 2
    skin_volume = float(vol[skin].sum())
    core_volume = float(vol[core].sum())
    write_tet(os.path.join(job_dir, outputs["mesh"]), nodes, tets, material)

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
        "volume_error": abs(skin_volume + core_volume - outer_volume) / abs(outer_volume) if outer_volume else 0.0,
        "core_volume_error": (abs(core_volume - inner_volume) / abs(outer_volume)) if outer_volume and if_ is not None else 0.0,
        "flipped": flipped,
        "degenerate": degenerate,
        "min_quality": float(quality.min()),
        "mean_quality": float(quality.mean()),
        "poor_quality": int((quality < 0.1).sum()),
        "surface_nodes": int(len(ov) + (0 if iv is None else len(iv))),
        "seconds_mesh": seconds_mesh,
        "seconds": time.time() - t_start,
        "gmsh": gmsh.GMSH_API_VERSION if hasattr(gmsh, "GMSH_API_VERSION") else "",
    }
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  %d nodes, %d skin + %d core tetrahedra; skin %.6g + core %.6g = %.6g vs body %.6g (relative %.2e); quality min %.3f mean %.3f, %d under 0.1; %d flipped, %d degenerate; %.2f s"
        % (result["nodes"], result["skin_tets"], result["core_tets"], skin_volume, core_volume,
           skin_volume + core_volume, outer_volume, result["volume_error"], result["min_quality"],
           result["mean_quality"], result["poor_quality"], flipped, degenerate, result["seconds"]))
    return result


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
        _write_failure(job_dir, "%s: %s" % (type(exc).__name__, exc))
        log("error: %s: %s" % (type(exc).__name__, exc))
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
