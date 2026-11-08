"""Hollowing and ribs for Aerotelic's body structures, on MeshLib.

Run by the application (Entity/AerotelicMeshLib.cpp) as a separate process
in the WingDesigner micromamba environment, where MeshLib is installed
(``pip install meshlib``)::

    python structure_meshlib.py <job directory>

The job directory holds ``job.json`` (see ``run_job`` for the keys) and the
bodies as ``.tri`` files. The script writes the skin, each body's inner
offset surface and each plane network's ribs back as ``.tri`` files, and
``result.json``.

A body's structure (core body_structure.hpp) is a material, a skin
thickness and plane networks -- ribs, spars, frames as shells -- over the
body's own geometry; the generated layers are the skin and the ribs.
(Until 2026-11-08 the same job also extracted a lattice infill from a
sampled field and returned it as a third body; the lattice infill was
removed from the application that day and this script with it.)

What it does, per body:

1. **Hollowing.** Each body is offset INWARD by the skin thickness with
   MeshLib's voxel offset (``offsetMesh``: a signed distance field on
   OpenVDB, dual marching cubes back), which cannot self-intersect and
   rounds a concave crease from the inside. The interior is then opened
   (double offset -r, +r, ``inner_opening``: parts thinner than 2r -- the
   wedge tapering to nothing along a trailing edge -- are made solid, so
   the core is nowhere thinner than 2r), the surface is decimated within
   a fraction of its voxel, and pockets smaller than a few voxels are
   dropped. The **Skin** body is the body's own surface plus the inverted
   offset surface: a closed shell whose outside is exact.
2. **The hollow.** The union of the offset interiors (a mesh boolean when
   they overlap) minus the reserved volumes (mesh boolean difference), so
   the ribs never cross a payload bay.
3. **Inner surfaces.** Each body's offset surface (after the pocket drop
   and the decimation, optionally remeshed to isotropic triangles of
   ``inner_remesh_edge``) is written back on its own as ``inner_<i>.tri``
   and checked: closed, outward, not touching the body's own surface
   (MeshLib's triangle collision test) and not crossing itself. The
   application keeps it as the hollow's inner boundary for the structural
   analysis, whose rib shells lie inside it. A target that carries an
   ``inner_file`` of its own (the offset from an earlier run of the same
   body and skin, cached by the application) is not offset again.
4. **Plane networks (2026-11-08: ribs, spars, frames).** Each network is a
   family of parallel planes with a thickness: its slabs (boxes about the
   planes, wide enough to cover the body) are intersected with the hollow
   by an exact mesh boolean, so the ribs are the hollow's exact
   cross-sections, wall to wall; a network with a ``height`` reaches only
   that far in from the surface (the slabs minus the body's deeper offset
   at skin + height: a stringer, a cap). Earlier networks have priority
   (a spar stays continuous, the ribs after it are notched by it), so the
   layers are disjoint and their volumes add. Each network's body is
   written as ``ribs_<j>.tri``.

The ``.tri`` format is the application's own: 8 bytes ``AEROTRI1``,
uint32 vertex count, uint32 triangle count, float64 xyz per vertex, int32
abc per triangle (counter-clockwise seen from outside).
"""

import json
import os
import struct
import sys
import time


def _bootstrap_dlls():
    """numpy's BLAS and MeshLib's own DLLs live in <env>\\Library\\bin,
    which is only on PATH inside an activated environment; the
    application runs python.exe directly."""
    lib_bin = os.path.join(sys.prefix, "Library", "bin")
    if os.path.isdir(lib_bin):
        os.environ["PATH"] = lib_bin + os.pathsep + os.environ.get("PATH", "")
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(lib_bin)


_bootstrap_dlls()

import numpy as np  # noqa: E402
import meshlib.mrmeshpy as mm  # noqa: E402
from meshlib import mrmeshnumpy as mn  # noqa: E402

TRI_MAGIC = b"AEROTRI1"

# The isotropic remeshes of a job's offset surfaces are together held to
# about this many triangles: every body's edge is raised to the one at
# which the bodies' total area fits. The remesh is the step that runs
# away when the edge asked for is far finer than the body (p38: a 2 mm
# edge over a 148 m^2 airframe is 85 M triangles; MeshLib's split cap
# stopped it at 20 M per body after 15 minutes and 18 GB, and the
# application hung in swap). The budget is the job's, not each body's:
# the same airframe as six shells reached 2 M on four of them in turn
# and was still 12 minutes in. A million triangles remesh in about a
# minute and leave the booleans after them tractable.
MAX_INNER_FACES = 1000000


# ---------------------------------------------------------------------------
# .tri I/O
# ---------------------------------------------------------------------------

def read_tri(path):
    """Reads a .tri file into a MeshLib mesh (vertices welded by index, as
    written: the loft's own shared vertices survive)."""
    with open(path, "rb") as f:
        magic = f.read(8)
        if magic != TRI_MAGIC:
            raise ValueError("%s is not a .tri file" % path)
        nv, nt = struct.unpack("<II", f.read(8))
        verts = np.frombuffer(f.read(nv * 24), dtype="<f8").reshape(nv, 3)
        faces = np.frombuffer(f.read(nt * 12), dtype="<i4").reshape(nt, 3)
    return mn.meshFromFacesVerts(np.ascontiguousarray(faces, dtype=np.int32), np.ascontiguousarray(verts, dtype=np.float64))


def write_tri(path, mesh):
    """Writes a MeshLib mesh as .tri (packed first: no deleted faces or
    unused vertices leave the process)."""
    if mesh is None or mesh.topology.numValidFaces() == 0:
        verts = np.zeros((0, 3), dtype=np.float64)
        faces = np.zeros((0, 3), dtype=np.int32)
    else:
        mesh.pack()
        verts = np.ascontiguousarray(mn.getNumpyVerts(mesh), dtype=np.float64)
        faces = np.ascontiguousarray(mn.getNumpyFaces(mesh.topology), dtype=np.int32)
    with open(path, "wb") as f:
        f.write(TRI_MAGIC)
        f.write(struct.pack("<II", verts.shape[0], faces.shape[0]))
        f.write(verts.astype("<f8").tobytes())
        f.write(faces.astype("<i4").tobytes())
    return verts.shape[0], faces.shape[0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def log(msg):
    print(msg, flush=True)


class JobError(Exception):
    pass


def num_faces(mesh):
    return 0 if mesh is None else mesh.topology.numValidFaces()


def is_closed(mesh):
    return num_faces(mesh) > 0 and mesh.topology.findNumHoles() == 0


def require_closed_body(mesh, label, role):
    if num_faces(mesh) == 0:
        raise JobError("'%s' (%s) has no triangles." % (label, role))
    if not is_closed(mesh):
        raise JobError("'%s' (%s) is not a closed surface, so inside and outside cannot be told apart. The structure was not generated." % (label, role))
    if not mesh.volume() > 0.0:
        raise JobError("'%s' (%s) is turned inside out (its enclosed volume is not positive). The structure was not generated." % (label, role))


def mesh_boolean(a, b, operation, what):
    """A mesh boolean, or a JobError naming what failed."""
    res = mm.boolean(a, b, operation)
    if not res.valid():
        raise JobError("%s: MeshLib's boolean failed (%s)." % (what, res.errorString))
    return res.mesh


def drop_small_components(mesh, min_volume):
    """Removes closed components of less than min_volume (slivers of an
    inward offset along a trailing edge). Returns the number dropped."""
    if num_faces(mesh) == 0:
        return 0
    components = mm.MeshComponents.getAllComponents(mm.MeshPart(mesh))
    if len(components) <= 1:
        return 0
    dropped = 0
    to_delete = mm.FaceBitSet()
    for comp in components:
        if abs(mesh.volume(comp)) < min_volume:
            to_delete |= comp
            dropped += 1
    if dropped == len(components):
        # Everything is a sliver: keep nothing (the body is all skin).
        mesh.deleteFaces(to_delete)
        mesh.pack()
        return dropped
    if dropped:
        mesh.deleteFaces(to_delete)
        mesh.pack()
    return dropped


def decimate_within(mesh, max_error):
    """Decimates in place within max_error of the surface, keeping it
    closed; returns the face count after (or before, if decimation is
    unavailable or would open the surface)."""
    if max_error <= 0.0 or num_faces(mesh) == 0:
        return num_faces(mesh)
    before = num_faces(mesh)
    backup = mm.Mesh(mesh)
    try:
        settings = mm.DecimateSettings()
        settings.maxError = float(max_error)
        settings.packMesh = True
        settings.touchBdVerts = False
        mm.decimateMesh(mesh, settings)
    except Exception as exc:  # pragma: no cover - depends on the MeshLib build
        log("  decimation unavailable (%s): keeping %d faces" % (exc, before))
        return before
    if not is_closed(mesh) or not mesh.volume() > 0.0:
        log("  decimation opened the surface: keeping the %d original faces" % before)
        mesh.topology = backup.topology
        mesh.points = backup.points
        mesh.invalidateCaches()
        return before
    return num_faces(mesh)


def inward_offset(body, thickness, voxel, mode):
    """The body's inward offset by `thickness` as a closed mesh (possibly
    several components, possibly empty when nothing is thicker than two
    skins)."""
    params = mm.GeneralOffsetParameters()
    params.voxelSize = float(voxel)
    params.signDetectionMode = mm.SignDetectionMode.OpenVDB
    params.memoryEfficient = True
    params.mode = {
        "standard": mm.OffsetMode.Standard,
        "smooth": mm.OffsetMode.Smooth,
        "sharpening": mm.OffsetMode.Sharpening,
    }.get(mode, mm.OffsetMode.Standard)
    try:
        return mm.generalOffsetMesh(mm.MeshPart(body), -float(thickness), params)
    except Exception as exc:
        raise JobError("the inward offset failed: %s" % exc)


def remesh_edge_floor(area, max_faces=None):
    """The shortest isotropic edge at which surfaces of `area` in all
    remesh to no more than `max_faces` (MAX_INNER_FACES as it stands when
    called) equilateral triangles: area / (sqrt(3)/4 edge^2) = max_faces.
    Zero for no area."""
    if max_faces is None:
        max_faces = MAX_INNER_FACES
    if max_faces <= 0 or not area > 0.0:
        return 0.0
    return float(np.sqrt(area / (0.25 * np.sqrt(3.0) * max_faces)))


def remesh_isotropic(mesh, edge_len):
    """Remeshes in place toward isotropic triangles of `edge_len` (a
    better-conditioned bounding surface for the analysis), keeping it
    closed; reverts and returns False if MeshLib's remesh fails or opens
    the surface. The caller keeps `edge_len` at or above
    remesh_edge_floor; the split cap here is the backstop for a surface
    whose area the floor misjudged."""
    if edge_len <= 0.0 or num_faces(mesh) == 0:
        return False
    backup = mm.Mesh(mesh)
    try:
        settings = mm.RemeshSettings()
        settings.targetEdgeLen = float(edge_len)
        settings.maxEdgeSplits = MAX_INNER_FACES
        settings.maxAngleChangeAfterFlip = 30.0 * np.pi / 180.0
        settings.useCurvature = True
        settings.finalRelaxIters = 2
        settings.finalRelaxNoShrinkage = True
        settings.packMesh = True
        ok = mm.remesh(mesh, settings)
    except Exception as exc:  # pragma: no cover - depends on the MeshLib build
        log("  remesh unavailable (%s)" % exc)
        ok = False
    if not ok or not is_closed(mesh) or not mesh.volume() > 0.0:
        mesh.topology = backup.topology
        mesh.points = backup.points
        mesh.invalidateCaches()
        return False
    return True


def open_interior(mesh, radius, voxel):
    """Morphological opening of a closed interior: offset inward by `radius`
    then outward by it again (MeshLib's double offset on one voxel grid),
    which removes every part thinner than twice the radius -- the wedge of
    an offset interior tapering to nothing along a trailing edge, whose
    two sheets would otherwise lie closer than anything built between
    them. Returns the opened mesh (possibly empty)."""
    if radius <= 0.0 or num_faces(mesh) == 0:
        return mesh
    params = mm.OffsetParameters()
    params.voxelSize = float(voxel)
    params.signDetectionMode = mm.SignDetectionMode.OpenVDB
    try:
        return mm.doubleOffsetMesh(mm.MeshPart(mesh), -float(radius), float(radius), params)
    except Exception as exc:
        raise JobError("the opening of the offset interior failed: %s" % exc)


def inner_surface_report(body, inner, label):
    """Checks an offset interior as the hollow's inner boundary: closed,
    outward, clear of the body's own surface and not crossing itself.
    Returns (ok, note)."""
    if num_faces(inner) == 0:
        return True, "all skin: no inner surface"
    if not is_closed(inner):
        return False, "%s: the inner surface is not closed (%d holes)" % (label, inner.topology.findNumHoles())
    if not inner.volume() > 0.0:
        return False, "%s: the inner surface is turned inside out" % label
    if not inner.volume() < body.volume():
        return False, "%s: the inner surface is not smaller than the body" % label
    hits = mm.findCollidingTriangles(mm.MeshPart(body), mm.MeshPart(inner), None, True)
    if len(hits) > 0:
        return False, "%s: the inner surface touches the body's surface (the skin is thinner than the offset's voxel there)" % label
    # Self-intersection: a surface can clear the body and still cross
    # itself where the offset folds through a thin region, and anything
    # meshed against it fails there. Found on the P-38 airframe with the
    # offset remeshed at three skins; two skins is clean.
    self_hits = mm.findSelfCollidingTriangles(mm.MeshPart(inner))
    if len(self_hits) > 0:
        return False, "%s: the inner surface crosses itself in %d place(s) (the offset folds through a thin region; a finer remesh clears it)" % (label, len(self_hits))
    return True, ""


# ---------------------------------------------------------------------------
# Plane networks
# ---------------------------------------------------------------------------

def plane_basis(n):
    """An orthonormal (u, v) in the plane of unit normal n, u x v = n."""
    n = np.asarray(n, dtype=np.float64)
    n = n / np.linalg.norm(n)
    a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(a, n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    return n, u, v


def slab_mesh(normal, offset, thickness, centre, half_extent):
    """The slab of `thickness` about the plane normal . x = offset, a box
    2 * half_extent across in the plane centred on `centre` projected onto
    it, closed and outward (the core's SlabMesh, body_structure.hpp)."""
    n, u, v = plane_basis(normal)
    c = np.asarray(centre, dtype=np.float64)
    c = c - (np.dot(n, c) - float(offset)) * n
    h = float(half_extent)
    t = 0.5 * float(thickness)
    verts = np.zeros((8, 3), dtype=np.float64)
    for i in range(8):
        su = -h if (i & 1) == 0 else h
        sv = -h if (i & 2) == 0 else h
        sn = -t if (i & 4) == 0 else t
        verts[i] = c + su * u + sv * v + sn * n
    # The twelve triangles of a box over the bit pattern (bit0 u, bit1 v,
    # bit2 n), wound outward.
    faces = np.array([
        [0, 2, 3], [0, 3, 1],   # -n
        [4, 5, 7], [4, 7, 6],   # +n
        [0, 1, 5], [0, 5, 4],   # -v
        [2, 6, 7], [2, 7, 3],   # +v
        [0, 4, 6], [0, 6, 2],   # -u
        [1, 3, 7], [1, 7, 5],   # +u
    ], dtype=np.int32)
    mesh = mn.meshFromFacesVerts(faces, verts)
    if mesh.volume() < 0.0:
        mesh.topology.flipOrientation()
    return mesh


def network_slabs(net, nudge=0.0):
    """One mesh holding every plane's slab of the network (the planes are
    parallel and thinner than their spacing, so the slabs are disjoint
    components); None when the network has no planes."""
    slabs = None
    for p in net.get("planes", []):
        slab = slab_mesh(p["normal"], float(p["offset"]) + nudge, net["thickness"], net["centre"], net["half_extent"])
        if slabs is None:
            slabs = slab
        else:
            slabs.addMesh(slab)
    return slabs


def cut_networks(job, job_dir, domain, bodies, targets, voxel, mode, outputs, warnings):
    """The ribs of every network (job["networks"]) cut from `domain`, the
    domain with them removed, and the per-network result entries. The
    operations are exact mesh booleans; a boolean MeshLib refuses is
    retried once with the planes nudged by a millionth of the domain's
    diagonal, then that network is left empty with a warning."""
    networks = job.get("networks", []) or []
    ribs_prefix = outputs.get("ribs_prefix", "ribs")
    results = []
    rib_meshes = []
    if not networks:
        return domain, results, rib_meshes
    if domain is None or num_faces(domain) == 0:
        for j, net in enumerate(networks):
            label = net.get("label", "network %d" % j)
            rib_file = "%s_%d.tri" % (ribs_prefix, j)
            write_tri(os.path.join(job_dir, rib_file), None)
            results.append({"label": label, "file": rib_file, "volume": 0.0, "triangles": 0, "planes": len(net.get("planes", [])),
                            "note": "the body has no interior for the planes to cut"})
        return domain, results, rib_meshes
    bb = domain.computeBoundingBox()
    diag = (bb.max - bb.min).length()
    # The deeper offsets a limited-height network needs, by height (shared
    # between networks of the same height).
    deeper = {}

    def deeper_interior(height):
        if height in deeper:
            return deeper[height]
        inner_union = None
        for item, body in zip(targets, bodies):
            own_thickness = float(item.get("skin_thickness", job["skin_thickness"]))
            inner = inward_offset(body, own_thickness + height, voxel, mode)
            if num_faces(inner) == 0:
                continue
            if not is_closed(inner):
                raise JobError("the inward offset of '%s' at %.4g did not close." % (item.get("label", item["file"]), own_thickness + height))
            inner_union = mm.Mesh(inner) if inner_union is None else mesh_boolean(inner_union, inner, mm.BooleanOperation.Union, "uniting the deeper interiors")
        deeper[height] = inner_union
        return inner_union

    # Booleans between surfaces that COINCIDE fail in MeshLib (the ribs'
    # curved faces are the domain's own faces, so "domain minus ribs" and
    # "ribs minus the earlier ribs" both hand it coplanar faces: "cannot
    # separate mesh B"). So the CUTTER -- the slabs themselves, less the
    # deeper offset when the height is limited -- is what is intersected
    # with the domain for the ribs AND subtracted from the domain after:
    # the slab crosses the domain's surface transversally, the deeper
    # offset lies strictly inside it, and domain minus cutter is domain
    # minus ribs exactly. A later network is cut from the reduced domain,
    # which notches it by the earlier ones with no further boolean.
    for j, net in enumerate(networks):
        label = net.get("label", "network %d" % j)
        rib_file = "%s_%d.tri" % (ribs_prefix, j)
        planes = net.get("planes", [])
        height = float(net.get("height", 0.0))
        entry = {"label": label, "file": rib_file, "volume": 0.0, "triangles": 0, "planes": len(planes), "note": ""}
        ribs = None
        reduced = None
        for attempt in range(2):
            nudge = 0.0 if attempt == 0 else 1e-6 * diag
            try:
                cutter = network_slabs(net, nudge)
                if cutter is None or num_faces(cutter) == 0:
                    entry["note"] = "no planes"
                    break
                if height > 0.0:
                    core = deeper_interior(height)
                    if core is not None and num_faces(core) > 0:
                        cutter = mesh_boolean(cutter, core, mm.BooleanOperation.DifferenceAB, "limiting the height of '%s'" % label)
                cut = mesh_boolean(cutter, domain, mm.BooleanOperation.Intersection, "cutting the planes of '%s' to the interior" % label)
                if num_faces(cut) > 0:
                    reduced = mesh_boolean(domain, cutter, mm.BooleanOperation.DifferenceAB, "removing the ribs of '%s' from the hollow" % label)
                ribs = cut
                if attempt == 1:
                    warnings.append("%s: the planes were nudged by %.3g for MeshLib's boolean" % (label, nudge))
                break
            except JobError as exc:
                if attempt == 1:
                    warnings.append("%s: not built: %s" % (label, exc))
                    entry["note"] = str(exc)
                    ribs = None
                    reduced = None
        if ribs is not None and num_faces(ribs) > 0:
            if not is_closed(ribs):
                warnings.append("%s: the ribs are not a closed surface (%d holes)" % (label, ribs.topology.findNumHoles()))
            entry["volume"] = float(ribs.volume())
            if reduced is not None:
                domain = reduced
        elif ribs is not None and not entry["note"]:
            entry["note"] = "the planes miss the interior"
        rib_meshes.append(ribs if ribs is not None and num_faces(ribs) > 0 else None)
        counts = write_tri(os.path.join(job_dir, rib_file), ribs if ribs is not None and num_faces(ribs) > 0 else None)
        entry["triangles"] = counts[1]
        log("  %s: %d plane(s) -> %d faces, volume %.6g%s" % (label, len(planes), counts[1], entry["volume"], (" (%s)" % entry["note"]) if entry["note"] else ""))
        results.append(entry)
    return domain, results, rib_meshes


# ---------------------------------------------------------------------------
# The job
# ---------------------------------------------------------------------------

def run_job(job_dir):
    """Runs the job in `job_dir`. job.json:

        skin_thickness      the skin depth t (model units), > 0, of every
                            target that gives none of its own
        offset_voxel        voxel size of the offset's distance field
        offset_mode         "standard" (default) | "smooth" | "sharpening"
        decimate_tolerance  decimate the offset surface within this (0: no)
        min_pocket_volume   drop offset pockets smaller than this (0: no)
        inner_opening       open the offset interior by this radius (double
                            offset -r, +r: parts thinner than 2r are dropped,
                            so the core is nowhere thinner than that) before
                            it is used (0: no opening)
        inner_remesh_edge   remesh the offset surface to isotropic triangles
                            of this edge length before it is written as an
                            inner surface (0: keep the decimated surface);
                            raised, with a warning, where the job's inner
                            surfaces together would exceed MAX_INNER_FACES
                            triangles
        targets             [{label, file, skin_thickness?,
                            inner_remesh_edge?, inner_file?}] bodies to
                            hollow (.tri, closed), each optionally with its
                            own skin depth and remesh edge in place of the
                            job's; with inner_file (a .tri in the job
                            directory) the offset is taken from that file
                            as it is rather than computed
        reserved            [{label, file}] volumes the ribs must avoid
        networks            [{label, planes: [{normal, offset}], thickness,
                            height, centre, half_extent}] plane networks
                            cut from the hollow in order (see the module
                            docstring); may be absent
        outputs             {skin, result, inner_prefix, ribs_prefix}: file
                            names to write; the inner surface of target i
                            goes to <inner_prefix>_<i>.tri ("inner" by
                            default), network j to <ribs_prefix>_<j>.tri
                            ("ribs")

    Returns the result dictionary (also written to outputs.result):
    ok, skin_vertices, skin_triangles, skin_volume, domain_volume (the
    hollow before the ribs are taken out), ribs_volume, interiors (how
    many bodies have one), targets, networks, warnings, timings, seconds,
    meshlib. Its "targets" list has, per body in order: label,
    skin_thickness, inner_file, inner_vertices, inner_triangles,
    inner_components, inner_volume, inner_ok (closed, outward, clear of
    the body and of itself) and inner_note (why not, or "all skin"); its
    "networks" list, per network in order: label, file, volume,
    triangles, planes, note.
    """
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8") as f:
        job = json.load(f)
    thickness = float(job["skin_thickness"])
    if not thickness > 0.0:
        raise JobError("the skin thickness must be greater than zero")
    voxel = float(job.get("offset_voxel", thickness / 4.0))
    if not voxel > 0.0:
        voxel = thickness / 4.0
    mode = str(job.get("offset_mode", "standard")).lower()
    decimate_tol = float(job.get("decimate_tolerance", 0.0))
    min_pocket = float(job.get("min_pocket_volume", 0.0))
    remesh_edge = float(job.get("inner_remesh_edge", 0.0))
    opening = float(job.get("inner_opening", 0.0))
    outputs = job["outputs"]
    inner_prefix = outputs.get("inner_prefix", "inner")
    warnings = []
    timings = {}
    target_results = []

    # 1. Hollowing: the skin of every body, and the offset interiors.
    skin = mm.Mesh()
    interiors = []
    skin_volume = 0.0
    t0 = time.time()
    targets = job.get("targets", [])
    bodies = []
    for item in targets:
        label = item.get("label", item["file"])
        body = read_tri(os.path.join(job_dir, item["file"]))
        require_closed_body(body, label, "a body to hollow")
        bodies.append(body)
    # Never a remesh the machine cannot hold: the edge every body is
    # remeshed at is at least the one at which their offset surfaces
    # (about the bodies' own area, less a thin skin's worth) together fit
    # the triangle budget.
    remesh_floor = remesh_edge_floor(sum(float(b.area()) for b in bodies))
    remesh_raised = []
    for index, (item, body) in enumerate(zip(targets, bodies)):
        label = item.get("label", item["file"])
        # This body's own skin and remesh edge, else the job's.
        own_thickness = float(item.get("skin_thickness", thickness))
        if not own_thickness > 0.0:
            raise JobError("the skin thickness of '%s' must be greater than zero" % label)
        own_remesh = float(item.get("inner_remesh_edge", remesh_edge))
        if own_remesh > 0.0 and remesh_floor > own_remesh:
            remesh_raised.append((label, own_remesh))
            own_remesh = remesh_floor
        body_volume = body.volume()
        precomputed = item.get("inner_file")
        if precomputed:
            # The application's cached offset of this body at this skin:
            # already opened, decimated, remeshed and checked when it was
            # made, so none of that is done again.
            inner = read_tri(os.path.join(job_dir, precomputed))
            n_inner = num_faces(inner)
            if n_inner > 0 and not is_closed(inner):
                raise JobError("the cached offset of '%s' is not closed (%d holes)." % (label, inner.topology.findNumHoles()))
            own_remesh = 0.0
            opening_here = 0.0
            min_pocket_here = 0.0
            decimate_here = 0.0
        else:
            inner = inward_offset(body, own_thickness, voxel, mode)
            n_inner = num_faces(inner)
            if n_inner > 0 and not is_closed(inner):
                raise JobError("the inward offset of '%s' did not close (%d holes)." % (label, inner.topology.findNumHoles()))
            opening_here = opening
            min_pocket_here = min_pocket
            decimate_here = decimate_tol
        if opening_here > 0.0 and n_inner > 0:
            before = inner.volume()
            inner = open_interior(inner, opening, voxel)
            if num_faces(inner) > 0 and not is_closed(inner):
                raise JobError("the opened offset interior of '%s' did not close (%d holes)." % (label, inner.topology.findNumHoles()))
            after = inner.volume() if num_faces(inner) > 0 else 0.0
            if before > 0.0 and before - after > 0.02 * before:
                warnings.append("%s: parts of the interior thinner than %.3g (%.1f%% of it) were made solid so the core is nowhere thinner than that" % (label, 2.0 * opening, 100.0 * (before - after) / before))
        dropped = drop_small_components(inner, min_pocket_here) if min_pocket_here > 0.0 else 0
        if dropped:
            warnings.append("%s: %d pocket(s) of the offset interior smaller than %.3g were filled solid" % (label, dropped, min_pocket))
        n_dec = decimate_within(inner, decimate_here)
        remeshed = remesh_isotropic(inner, own_remesh) if own_remesh > 0.0 else False
        if remeshed and min_pocket_here > 0.0:
            # The remesh can leave its own debris: on the P-38 airframe it
            # made four pockets of about 1e-5 (three self-intersecting)
            # beside the real one of 17.1, and anything meshed against the
            # surface fails over them. Drop them the same way.
            debris = drop_small_components(inner, min_pocket)
            if debris:
                warnings.append("%s: %d sliver(s) left by the remesh of the offset surface were dropped" % (label, debris))
        inner_volume = inner.volume() if num_faces(inner) > 0 else 0.0
        n_components = mm.MeshComponents.getNumComponents(mm.MeshPart(inner)) if num_faces(inner) else 0
        log("  %s: body %d faces, volume %.6g; inward offset %.4g -> %d faces (%d after decimation%s), %d component(s), volume %.6g"
            % (label, num_faces(body), body_volume, own_thickness, n_inner, n_dec,
               (", %d after remeshing" % num_faces(inner)) if remeshed else "", n_components, inner_volume))
        if num_faces(inner) == 0:
            warnings.append("%s is nowhere thicker than two skins: it is all skin, with no interior for ribs" % label)
        skin_volume += body_volume - inner_volume
        # The inner surface on its own: the hollow's inner boundary.
        inner_ok, inner_note = inner_surface_report(body, inner, label)
        if not inner_ok:
            warnings.append(inner_note + "; the hollow's inner surface cannot be used by the structural analysis")
        inner_file = "%s_%d.tri" % (inner_prefix, index)
        inner_counts = write_tri(os.path.join(job_dir, inner_file), mm.Mesh(inner) if num_faces(inner) > 0 else None)
        target_results.append({
            "label": label,
            "skin_thickness": own_thickness,
            "inner_file": inner_file,
            "inner_vertices": inner_counts[0],
            "inner_triangles": inner_counts[1],
            "inner_components": n_components,
            "inner_volume": inner_volume,
            "inner_ok": bool(inner_ok),
            "inner_note": inner_note,
        })
        # The skin: the body's own surface outside, the offset inverted inside.
        skin.addMesh(body)
        if num_faces(inner) > 0:
            inverted = mm.Mesh(inner)
            inverted.topology.flipOrientation()
            skin.addMesh(inverted)
            interiors.append((label, inner))
    if remesh_raised:
        warnings.append("the offset surface of %s was remeshed at %.3g rather than %.3g, so the job's inner surfaces together stay under %d triangles"
                        % (", ".join("'%s'" % r[0] for r in remesh_raised), remesh_floor, min(r[1] for r in remesh_raised), MAX_INNER_FACES))
    timings["hollowing"] = time.time() - t0

    # 2. The hollow: the union of the interiors minus the reserved volumes.
    t0 = time.time()
    domain = None
    for label, inner in interiors:
        if domain is None:
            domain = mm.Mesh(inner)
        else:
            domain = mesh_boolean(domain, inner, mm.BooleanOperation.Union, "uniting the interior of '%s' with the others" % label)
    for item in job.get("reserved", []):
        label = item.get("label", item["file"])
        reserved = read_tri(os.path.join(job_dir, item["file"]))
        require_closed_body(reserved, label, "a reserved volume")
        if domain is not None and num_faces(domain) > 0:
            domain = mesh_boolean(domain, reserved, mm.BooleanOperation.DifferenceAB, "cutting the reserved volume '%s' from the hollow" % label)
    domain_volume = domain.volume() if domain is not None and num_faces(domain) > 0 else 0.0
    timings["domain"] = time.time() - t0
    log("  hollow: %d faces, volume %.6g" % (num_faces(domain), domain_volume))

    # 3. The plane networks: ribs cut from the hollow.
    t0 = time.time()
    domain, network_results, rib_meshes = cut_networks(job, job_dir, domain, bodies, targets, voxel, mode, outputs, warnings)
    ribs_volume = sum(float(r["volume"]) for r in network_results)
    timings["networks"] = time.time() - t0

    # 4. Out.
    t0 = time.time()
    skin_counts = write_tri(os.path.join(job_dir, outputs["skin"]), skin)
    timings["write"] = time.time() - t0
    result = {
        "ok": True,
        "skin_vertices": skin_counts[0],
        "skin_triangles": skin_counts[1],
        "skin_volume": skin_volume,
        "domain_volume": domain_volume,
        "ribs_volume": ribs_volume,
        "interiors": len(interiors),
        "targets": target_results,
        "networks": network_results,
        "warnings": warnings,
        "timings": timings,
        "seconds": time.time() - t_start,
        "meshlib": getattr(mm, "__version__", ""),
    }
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  skin %d triangles (volume %.6g), hollow %.6g, ribs %.6g over %d network(s), %.2f s"
        % (result["skin_triangles"], skin_volume, domain_volume, ribs_volume, len(network_results), result["seconds"]))
    return result


def main(argv):
    if len(argv) != 2:
        log("usage: structure_meshlib.py <job directory>")
        return 2
    job_dir = argv[1]
    try:
        run_job(job_dir)
        return 0
    except JobError as exc:
        _write_failure(job_dir, str(exc))
        log("error: %s" % exc)
        return 1
    except Exception as exc:  # anything MeshLib or numpy raised
        _write_failure(job_dir, "%s: %s" % (type(exc).__name__, exc))
        log("error: %s: %s" % (type(exc).__name__, exc))
        return 1


def _write_failure(job_dir, message):
    try:
        with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8") as f:
            outputs = json.load(f).get("outputs", {})
        path = os.path.join(job_dir, outputs.get("result", "result.json"))
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"ok": False, "error": message}, f, indent=1)
    except Exception:
        pass


if __name__ == "__main__":
    sys.exit(main(sys.argv))
