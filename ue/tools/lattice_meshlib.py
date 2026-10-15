"""Hollowing and infill for Aerotelic's lattices, on MeshLib.

Run by the application (Entity/AerotelicMeshLib.cpp) as a separate process
in the WingDesigner micromamba environment, where MeshLib is installed
(``pip install meshlib``)::

    python lattice_meshlib.py <job directory>

The job directory holds ``job.json`` (see ``run_job`` for the keys), the
bodies as ``.tri`` files and, when there is an infill, the lattice field
sampled on a grid as raw float32 (``lattice.f32``). The script writes the
two bodies of the lattice back as ``.tri`` files and ``result.json``.

What it does, per lattice:

1. **Hollowing.** Each body to fill is offset INWARD by the skin thickness
   with MeshLib's voxel offset (``offsetMesh``: a signed distance field on
   OpenVDB, dual marching cubes back), which cannot self-intersect and
   rounds a concave crease from the inside. The interior is then opened
   (double offset -r, +r, ``inner_opening``: parts thinner than 2r -- the
   wedge tapering to nothing along a trailing edge -- are made solid, so
   the core is nowhere thinner than 2r), the surface is decimated within
   a fraction of its voxel, and pockets smaller than a few voxels are
   dropped. The **Skin** body is the body's own surface plus the inverted
   offset surface: a closed shell whose outside is exact.
2. **Infill domain.** The union of the offset interiors (a mesh boolean
   when they overlap) minus the reserved volumes (mesh boolean
   difference), so the infill never crosses a payload bay.
3. **Infill.** The lattice field the application sampled (F <= 0 inside
   the lattice material: the core's OpenLattice TPMS / strut fields, all
   36 types) goes through MeshLib's marching cubes, padded so the surface
   closes, and is intersected with the infill domain by a mesh boolean --
   an exact cut along the offset surface rather than a resampled one.
   The **Lattice** body is that intersection.
4. **Inner surfaces for the tetrahedral mesher.** Each body's offset
   surface (after the pocket drop and the decimation, optionally
   remeshed to isotropic triangles of ``inner_remesh_edge``) is written
   back on its own as ``inner_<i>.tri`` and checked for the constrained
   Delaunay mesher (core tet.hpp, TetrahedralizeNested): closed, outward,
   and not touching the body's own surface (MeshLib's triangle collision
   test). The application meshes body and offset together into skin and
   core tetrahedra that conform to both exactly.

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
        raise JobError("'%s' (%s) is not a closed surface, so inside and outside cannot be told apart. The lattice was not generated." % (label, role))
    if not mesh.volume() > 0.0:
        raise JobError("'%s' (%s) is turned inside out (its enclosed volume is not positive). The lattice was not generated." % (label, role))


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


def remesh_isotropic(mesh, edge_len):
    """Remeshes in place toward isotropic triangles of `edge_len` (for a
    better-conditioned constraint surface), keeping it closed; reverts
    and returns False if MeshLib's remesh fails or opens the surface."""
    if edge_len <= 0.0 or num_faces(mesh) == 0:
        return False
    backup = mm.Mesh(mesh)
    try:
        settings = mm.RemeshSettings()
        settings.targetEdgeLen = float(edge_len)
        settings.maxEdgeSplits = 10000000
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
    two sheets, closer than the mesher's triangles, would otherwise split
    each other without end. Returns the opened mesh (possibly empty)."""
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
    """Checks an offset interior as the inner constraint surface of the
    tetrahedral mesher: closed, outward, clear of the body's own surface
    and not crossing itself. Returns (ok, note)."""
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
    # itself where the offset folds through a thin region, and a
    # tetrahedral mesher refuses the body over it ("a segment and a facet
    # intersect"). Found on the P-38 airframe with the offset remeshed at
    # three skins; two skins is clean.
    self_hits = mm.findSelfCollidingTriangles(mm.MeshPart(inner))
    if len(self_hits) > 0:
        return False, "%s: the inner surface crosses itself in %d place(s) (the offset folds through a thin region; a finer remesh clears it)" % (label, len(self_hits))
    return True, ""


def field_volume(spec, job_dir):
    """The lattice field as a MeshLib SimpleVolume, padded by one positive
    voxel on every side so marching cubes closes the surface, with the
    origin that puts voxel (i, j, k) on the application's grid node."""
    nx, ny, nz = int(spec["nx"]), int(spec["ny"]), int(spec["nz"])
    h = float(spec["h"])
    path = os.path.join(job_dir, spec["file"])
    raw = np.fromfile(path, dtype="<f4")
    if raw.size != nx * ny * nz:
        raise JobError("the lattice field holds %d samples, not %d x %d x %d" % (raw.size, nx, ny, nz))
    # The application writes x fastest: index = (k * ny + j) * nx + i.
    arr = raw.reshape(nz, ny, nx).transpose(2, 1, 0)
    pad = np.float32(max(h, float(np.nanmax(np.abs(raw))) if raw.size else h))
    arr = np.pad(arr, 1, mode="constant", constant_values=pad)
    vol = mn.simpleVolumeFrom3Darray(np.ascontiguousarray(arr, dtype=np.float32))
    vol.voxelSize = mm.Vector3f(h, h, h)
    o = spec["origin"]
    # Marching cubes puts voxel index i at origin + (i + 0.5) h; the
    # padding shifts the application's node i to index i + 1.
    origin = mm.Vector3f(float(o[0]) - 1.5 * h, float(o[1]) - 1.5 * h, float(o[2]) - 1.5 * h)
    return vol, origin


def lattice_surface(vol, origin):
    params = mm.MarchingCubesParams()
    params.iso = 0.0
    params.lessInside = True
    params.origin = origin
    return mm.marchingCubes(vol, params)


# ---------------------------------------------------------------------------
# The job
# ---------------------------------------------------------------------------

def run_job(job_dir):
    """Runs the job in `job_dir`. job.json:

        skin_thickness      the skin depth t (model units), > 0
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
                            inner surface (0: keep the decimated surface)
        targets             [{label, file}] bodies to fill (.tri, closed)
        reserved            [{label, file}] volumes the infill must avoid
        field               null, or {file, nx, ny, nz, origin, h}: the
                            lattice field on the grid (F <= 0 is material)
        outputs             {skin, lattice, result, inner_prefix}: file
                            names to write; the inner surface of target i
                            goes to <inner_prefix>_<i>.tri ("inner" by
                            default)

    Returns the result dictionary (also written to outputs.result). Its
    "targets" list has, per body in order: label, inner_file,
    inner_vertices, inner_triangles, inner_components, inner_volume,
    inner_ok (usable as the mesher's inner constraint surface) and
    inner_note (why not, or "all skin").
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
    for index, item in enumerate(job.get("targets", [])):
        label = item.get("label", item["file"])
        body = read_tri(os.path.join(job_dir, item["file"]))
        require_closed_body(body, label, "a body to fill")
        body_volume = body.volume()
        inner = inward_offset(body, thickness, voxel, mode)
        n_inner = num_faces(inner)
        if n_inner > 0 and not is_closed(inner):
            raise JobError("the inward offset of '%s' did not close (%d holes)." % (label, inner.topology.findNumHoles()))
        if opening > 0.0 and n_inner > 0:
            before = inner.volume()
            inner = open_interior(inner, opening, voxel)
            if num_faces(inner) > 0 and not is_closed(inner):
                raise JobError("the opened offset interior of '%s' did not close (%d holes)." % (label, inner.topology.findNumHoles()))
            after = inner.volume() if num_faces(inner) > 0 else 0.0
            if before > 0.0 and before - after > 0.02 * before:
                warnings.append("%s: parts of the interior thinner than %.3g (%.1f%% of it) were made solid so the core is nowhere thinner than that" % (label, 2.0 * opening, 100.0 * (before - after) / before))
        dropped = drop_small_components(inner, min_pocket) if min_pocket > 0.0 else 0
        if dropped:
            warnings.append("%s: %d pocket(s) of the offset interior smaller than %.3g were filled solid" % (label, dropped, min_pocket))
        n_dec = decimate_within(inner, decimate_tol)
        remeshed = remesh_isotropic(inner, remesh_edge) if remesh_edge > 0.0 else False
        if remeshed and min_pocket > 0.0:
            # The remesh can leave its own debris: on the P-38 airframe it
            # made four pockets of about 1e-5 (three self-intersecting)
            # beside the real one of 17.1, and a tetrahedral mesher refuses
            # the lot over them. Drop them the same way.
            debris = drop_small_components(inner, min_pocket)
            if debris:
                warnings.append("%s: %d sliver(s) left by the remesh of the offset surface were dropped" % (label, debris))
        inner_volume = inner.volume() if num_faces(inner) > 0 else 0.0
        n_components = mm.MeshComponents.getNumComponents(mm.MeshPart(inner)) if num_faces(inner) else 0
        log("  %s: body %d faces, volume %.6g; inward offset %.4g -> %d faces (%d after decimation%s), %d component(s), volume %.6g"
            % (label, num_faces(body), body_volume, thickness, n_inner, n_dec,
               (", %d after remeshing" % num_faces(inner)) if remeshed else "", n_components, inner_volume))
        if num_faces(inner) == 0:
            warnings.append("%s is nowhere thicker than two skins: it is all skin, with no infill" % label)
        skin_volume += body_volume - inner_volume
        # The inner surface on its own, for the tetrahedral mesher.
        inner_ok, inner_note = inner_surface_report(body, inner, label)
        if not inner_ok:
            warnings.append(inner_note + "; its tetrahedral mesh cannot be built")
        inner_file = "%s_%d.tri" % (inner_prefix, index)
        inner_counts = write_tri(os.path.join(job_dir, inner_file), mm.Mesh(inner) if num_faces(inner) > 0 else None)
        target_results.append({
            "label": label,
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
    timings["hollowing"] = time.time() - t0

    # 2. The infill domain: the union of the interiors minus the reserved volumes.
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
            domain = mesh_boolean(domain, reserved, mm.BooleanOperation.DifferenceAB, "cutting the reserved volume '%s' from the infill domain" % label)
    domain_volume = domain.volume() if domain is not None and num_faces(domain) > 0 else 0.0
    timings["domain"] = time.time() - t0
    log("  infill domain: %d faces, volume %.6g" % (num_faces(domain), domain_volume))

    # 3. The infill: the lattice surface cut to the domain.
    t0 = time.time()
    lattice = None
    field = job.get("field")
    if field is not None and domain is not None and num_faces(domain) > 0:
        vol, origin = field_volume(field, job_dir)
        surface = lattice_surface(vol, origin)
        log("  lattice field %d x %d x %d: %d faces" % (int(field["nx"]), int(field["ny"]), int(field["nz"]), num_faces(surface)))
        if num_faces(surface) > 0:
            if not is_closed(surface):
                raise JobError("the lattice surface did not close (%d holes)." % surface.topology.findNumHoles())
            lattice = mesh_boolean(surface, domain, mm.BooleanOperation.Intersection, "cutting the lattice to the infill domain")
            if num_faces(lattice) > 0 and not is_closed(lattice):
                warnings.append("the infill is not a closed surface (%d holes)" % lattice.topology.findNumHoles())
    lattice_volume = lattice.volume() if lattice is not None and num_faces(lattice) > 0 else 0.0
    timings["infill"] = time.time() - t0

    # 4. Out.
    t0 = time.time()
    skin_counts = write_tri(os.path.join(job_dir, outputs["skin"]), skin)
    lattice_counts = write_tri(os.path.join(job_dir, outputs["lattice"]), lattice)
    timings["write"] = time.time() - t0
    result = {
        "ok": True,
        "skin_vertices": skin_counts[0],
        "skin_triangles": skin_counts[1],
        "lattice_vertices": lattice_counts[0],
        "lattice_triangles": lattice_counts[1],
        "skin_volume": skin_volume,
        "domain_volume": domain_volume,
        "lattice_volume": lattice_volume,
        "achieved_fraction": (lattice_volume / domain_volume) if domain_volume > 0.0 else 0.0,
        "interiors": len(interiors),
        "targets": target_results,
        "warnings": warnings,
        "timings": timings,
        "seconds": time.time() - t_start,
        "meshlib": getattr(mm, "__version__", ""),
    }
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  skin %d triangles (volume %.6g), lattice %d triangles (volume %.6g, %.1f%% of the domain), %.2f s"
        % (result["skin_triangles"], skin_volume, result["lattice_triangles"], lattice_volume, 100.0 * result["achieved_fraction"], result["seconds"]))
    return result


def main(argv):
    if len(argv) != 2:
        log("usage: lattice_meshlib.py <job directory>")
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
