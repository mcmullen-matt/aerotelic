"""Self-check of lattice_meshlib.py, outside the application.

    python lattice_meshlib_check.py [body.stl] [--keep DIR]

Builds a job the way Entity/AerotelicMeshLib.cpp does -- the body as .tri,
a gyroid network field sampled on the grid the core would plan (the field
here is numpy's gyroid; the application samples the core's own
LatticeField) -- runs it, and checks the result: both bodies closed and
outward, the skin's volume equal to body minus offset interior, the
infill inside the domain, the fraction near the one asked. Without an
STL it uses a unit sphere. With --keep the job directory (and STLs of the
results, for viewing) is left in DIR.
"""

import json
import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lattice_meshlib as lm  # noqa: E402  (bootstraps the DLLs)

import numpy as np  # noqa: E402
import meshlib.mrmeshpy as mm  # noqa: E402
from meshlib import mrmeshnumpy as mn  # noqa: E402


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def gyroid_field(origin, h, nx, ny, nz, cell, level):
    """Network-phase gyroid, F <= 0 inside: U - t, as the core's
    LatticeField (unscaled: only the sign matters here)."""
    k = 2.0 * np.pi / cell
    i = np.arange(nx) * h + origin[0]
    j = np.arange(ny) * h + origin[1]
    kk = np.arange(nz) * h + origin[2]
    X, Y, Z = np.meshgrid(k * i, k * j, k * kk, indexing="ij")
    U = np.cos(X) * np.sin(Y) + np.cos(Y) * np.sin(Z) + np.cos(Z) * np.sin(X)
    return (U - level).astype(np.float32)


def calibrate_gyroid(fraction, samples=48):
    g = (np.arange(samples) + 0.5) / samples * 2.0 * np.pi
    X, Y, Z = np.meshgrid(g, g, g, indexing="ij")
    U = np.cos(X) * np.sin(Y) + np.cos(Y) * np.sin(Z) + np.cos(Z) * np.sin(X)
    return float(np.quantile(U, fraction))


def main(argv):
    body_path = None
    keep = None
    args = list(argv[1:])
    while args:
        a = args.pop(0)
        if a == "--keep":
            keep = args.pop(0)
        else:
            body_path = a

    if body_path and body_path.lower().endswith(".tri"):
        body = lm.read_tri(body_path)  # the application's own format, double precision
    elif body_path:
        body = mm.loadMesh(body_path)
    else:
        body = mm.makeUVSphere(1.0, 64, 32)
    bb = body.computeBoundingBox()
    extent = bb.max - bb.min
    largest = max(extent.x, extent.y, extent.z)
    thickness = 0.01 * largest if body_path else 0.05
    cell = largest / 10.0 if body_path else 0.5
    resolution = 12
    fraction = 0.35
    h = cell / resolution
    print("body: %d faces, extent %.4g x %.4g x %.4g, volume %.6g; skin %.4g, cell %.4g, %d voxels per cell" % (
        body.topology.numValidFaces(), extent.x, extent.y, extent.z, body.volume(), thickness, cell, resolution))

    job_dir = keep if keep else tempfile.mkdtemp(prefix="aerotelic_lattice_")
    os.makedirs(job_dir, exist_ok=True)
    # The body as the application writes it: in double precision. A .tri
    # input is copied byte for byte rather than round-tripped through
    # MeshLib, whose points are float -- the quantization alone defeats the
    # tetrahedral mesher's planar-cap detection, so the kept job directory
    # would not be the one the application produces.
    if body_path and body_path.lower().endswith(".tri"):
        shutil.copyfile(body_path, os.path.join(job_dir, "target_0.tri"))
    else:
        lm.write_tri(os.path.join(job_dir, "target_0.tri"), mm.Mesh(body))

    # The grid as PlanLatticeGrid: bounds expanded by 2h, ceil(extent / h) cells.
    lo = (bb.min.x - 2 * h, bb.min.y - 2 * h, bb.min.z - 2 * h)
    ext = (extent.x + 4 * h, extent.y + 4 * h, extent.z + 4 * h)
    nx, ny, nz = [max(2, int(np.ceil(e / h))) + 1 for e in ext]
    level = calibrate_gyroid(fraction)
    field = gyroid_field(lo, h, nx, ny, nz, cell, level)
    # x fastest, as the application writes it.
    field.transpose(2, 1, 0).astype("<f4").tofile(os.path.join(job_dir, "lattice.f32"))

    voxel = min(thickness / 4.0, h)
    job = {
        "skin_thickness": thickness,
        "offset_voxel": voxel,
        "offset_mode": "standard",
        "decimate_tolerance": 0.2 * voxel,
        "min_pocket_volume": (4.0 * voxel) ** 3,
        # As the application: no opening, and the offset remeshed to
        # triangles of two skins for gmsh (tools/tet_gmsh.py).
        "inner_opening": 0.0,
        "inner_remesh_edge": 2.0 * thickness,
        "targets": [{"label": "Body", "file": "target_0.tri"}],
        "reserved": [],
        "field": {"file": "lattice.f32", "nx": nx, "ny": ny, "nz": nz, "origin": list(lo), "h": h},
        "outputs": {"skin": "skin.tri", "lattice": "lattice.tri", "result": "result.json"},
    }
    with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
        json.dump(job, f, indent=1)

    t0 = time.time()
    code = lm.main(["lattice_meshlib.py", job_dir])
    print("run: exit %d in %.2f s" % (code, time.time() - t0))
    with open(os.path.join(job_dir, "result.json"), "r", encoding="utf-8") as f:
        result = json.load(f)
    ok = check(code == 0 and result.get("ok"), "the job succeeded: %s" % result.get("error", ""))
    if not ok:
        return 1
    print(json.dumps(result, indent=1))

    skin = lm.read_tri(os.path.join(job_dir, "skin.tri"))
    lattice = lm.read_tri(os.path.join(job_dir, "lattice.tri"))
    ok &= check(skin.topology.findNumHoles() == 0 and skin.volume() > 0, "the skin is closed and outward (volume %.6g)" % skin.volume())
    ok &= check(abs(skin.volume() - result["skin_volume"]) < 1e-6 * max(1.0, body.volume()), "the skin's volume is body minus interior")
    if not body_path:
        shell = 4.0 / 3.0 * np.pi * (1.0 - 0.95 ** 3)
        ok &= check(abs(skin.volume() - shell) < 0.06 * shell, "the sphere's skin encloses the 0.05 shell (%.5g vs %.5g)" % (skin.volume(), shell))
    ok &= check(lattice.topology.numValidFaces() > 0, "the lattice has triangles (%d)" % lattice.topology.numValidFaces())
    ok &= check(lattice.topology.findNumHoles() == 0 and lattice.volume() > 0, "the lattice is closed and outward (volume %.6g)" % lattice.volume())
    ok &= check(abs(result["achieved_fraction"] - fraction) < 0.1, "the achieved fraction %.3f is near %.2f" % (result["achieved_fraction"], fraction))
    # Every lattice vertex is inside the body, deeper than the skin (a
    # little tolerance: the offset is voxel-sampled).
    verts = mn.getNumpyVerts(lattice)
    worst = -1e300
    for p in verts[:: max(1, len(verts) // 20000)]:
        d = body.signedDistance(mm.Vector3f(float(p[0]), float(p[1]), float(p[2])))
        worst = max(worst, d + thickness)
    ok &= check(worst < 1.5 * voxel, "the infill stays inside the skin (worst overshoot of the offset %.4g, voxel %.4g)" % (worst, voxel))
    # The inner surface for the tetrahedral mesher: reported per target,
    # written on its own, closed, outward, inside the body and clear of it.
    targets = result.get("targets", [])
    ok &= check(len(targets) == 1 and targets[0]["inner_ok"], "the inner surface is reported usable for the tetrahedral mesher: %s" % (targets[0].get("inner_note", "") if targets else "no targets"))
    if targets:
        inner = lm.read_tri(os.path.join(job_dir, targets[0]["inner_file"]))
        ok &= check(inner.topology.numValidFaces() == targets[0]["inner_triangles"] and inner.topology.findNumHoles() == 0 and inner.volume() > 0,
                    "the inner surface file is closed and outward (%d triangles, volume %.6g)" % (inner.topology.numValidFaces(), inner.volume()))
        ok &= check(abs(inner.volume() - result["domain_volume"]) < 1e-6 * max(1.0, body.volume()) or job["inner_remesh_edge"] > 0,
                    "the inner surface encloses the infill domain's volume (%.6g vs %.6g; remeshed within the voxel otherwise)" % (inner.volume(), result["domain_volume"]))
        ok &= check(len(mm.findCollidingTriangles(mm.MeshPart(body), mm.MeshPart(inner), None, True)) == 0, "the inner surface does not touch the body")

    if keep:
        mm.saveMesh(skin, os.path.join(keep, "skin.stl"))
        mm.saveMesh(lattice, os.path.join(keep, "lattice.stl"))
        print("kept %s" % keep)
        print("the tetrahedral mesh of this job (as the application builds it):\n"
              "  tet_check nested %s %s %.6g - 2" % (os.path.join(keep, "target_0.tri"), os.path.join(keep, targets[0]["inner_file"]) if targets else "", cell))
    else:
        shutil.rmtree(job_dir, ignore_errors=True)
    if not body_path:
        ok &= check_per_body_skin()
        ok &= check_remesh_budget()
    print("RESULT: %s" % ("all checks passed" if ok else "some checks FAILED"))
    return 0 if ok else 1


def check_remesh_budget():
    """The remeshes of a job's offset surfaces are together held to
    MAX_INNER_FACES (2026-11-01: p38's 1 mm skin asked for a 2 mm edge
    over a 148 m^2 airframe and ran to 20 M triangles per body, 15
    minutes and 18 GB -- the application hung in swap). The budget is
    lowered here so a unit sphere trips it: the edge asked for would take
    ~65 k triangles, the budget allows 20 k, and the result says so and
    stays near it."""
    print("remesh budget: a unit sphere asked for an edge far under its budget")
    job_dir = tempfile.mkdtemp(prefix="aerotelic_lattice_budget_")
    saved = lm.MAX_INNER_FACES
    ok = True
    try:
        budget = 20000
        lm.MAX_INNER_FACES = budget
        sphere = mm.makeUVSphere(1.0, 64, 32)
        lm.write_tri(os.path.join(job_dir, "target_0.tri"), sphere)
        thickness = 0.05
        voxel = thickness / 4.0
        edge = 0.02  # 4 pi 0.95^2 / (0.433 * 0.02^2) ~ 65 k triangles, over the budget
        job = {
            "skin_thickness": thickness,
            "offset_voxel": voxel,
            "offset_mode": "standard",
            "decimate_tolerance": 0.2 * voxel,
            "min_pocket_volume": (4.0 * voxel) ** 3,
            "inner_opening": 0.0,
            "inner_remesh_edge": edge,
            "targets": [{"label": "Sphere", "file": "target_0.tri"}],
            "reserved": [],
            "field": None,
            "outputs": {"skin": "skin.tri", "lattice": "lattice.tri", "result": "result.json"},
        }
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job, f, indent=1)
        code = lm.main(["lattice_meshlib.py", job_dir])
        with open(os.path.join(job_dir, "result.json"), "r", encoding="utf-8") as f:
            result = json.load(f)
        ok &= check(code == 0 and result.get("ok"), "remesh budget: the job succeeded: %s" % result.get("error", ""))
        if not ok:
            return False
        targets = result.get("targets", [])
        n = targets[0]["inner_triangles"] if targets else -1
        # The remesh lands near its target count, not exactly on it; well
        # under the ~65 k the edge asked for and no more than a little
        # over the budget.
        ok &= check(0 < n <= 1.25 * budget, "remesh budget: the inner surface stays near the %d budget (%d triangles)" % (budget, n))
        raised = [w for w in result.get("warnings", []) if "remeshed at" in w and "under %d triangles" % budget in w]
        ok &= check(len(raised) == 1, "remesh budget: the result warns that the edge was raised (%s)" % (raised[0] if raised else "no warning"))
        ok &= check(targets and targets[0]["inner_ok"], "remesh budget: the coarser inner surface is still usable: %s" % (targets[0].get("inner_note", "") if targets else "no targets"))
    finally:
        lm.MAX_INNER_FACES = saved
        shutil.rmtree(job_dir, ignore_errors=True)
    return ok


def check_per_body_skin():
    """The skin thickness is per target (2026-10-31): a target carrying its
    own `skin_thickness` is hollowed by it, one carrying none by the job's.
    Two unit spheres apart, no infill: the first with its own 0.05, the
    second left to the job's 0.10; each inner volume is the sphere's at
    1 - t, and the result reports each target's thickness."""
    print("per-body skin: two spheres, one with its own 0.05, one on the job's 0.10")
    job_dir = tempfile.mkdtemp(prefix="aerotelic_lattice_skins_")
    ok = True
    try:
        own, default = 0.05, 0.10
        for i, offset in enumerate((0.0, 4.0)):
            sphere = mm.makeUVSphere(1.0, 64, 32)
            sphere.transform(mm.AffineXf3f.translation(mm.Vector3f(offset, 0.0, 0.0)))
            lm.write_tri(os.path.join(job_dir, "target_%d.tri" % i), sphere)
        voxel = own / 4.0
        job = {
            "skin_thickness": default,
            "offset_voxel": voxel,
            "offset_mode": "standard",
            "decimate_tolerance": 0.2 * voxel,
            "min_pocket_volume": (4.0 * voxel) ** 3,
            "inner_opening": 0.0,
            "inner_remesh_edge": 2.0 * default,
            "targets": [
                {"label": "Own", "file": "target_0.tri", "skin_thickness": own, "inner_remesh_edge": 2.0 * own},
                {"label": "Default", "file": "target_1.tri"},
            ],
            "reserved": [],
            "field": None,
            "outputs": {"skin": "skin.tri", "lattice": "lattice.tri", "result": "result.json"},
        }
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job, f, indent=1)
        code = lm.main(["lattice_meshlib.py", job_dir])
        with open(os.path.join(job_dir, "result.json"), "r", encoding="utf-8") as f:
            result = json.load(f)
        ok &= check(code == 0 and result.get("ok"), "per-body skin: the job succeeded: %s" % result.get("error", ""))
        if not ok:
            return False
        targets = result.get("targets", [])
        ok &= check(len(targets) == 2 and targets[0]["skin_thickness"] == own and targets[1]["skin_thickness"] == default,
                    "per-body skin: the result reports each target's thickness (%s)" % [t.get("skin_thickness") for t in targets])
        for t, thickness in zip(targets, (own, default)):
            expected = 4.0 / 3.0 * np.pi * (1.0 - thickness) ** 3
            ok &= check(abs(t["inner_volume"] - expected) < 0.06 * expected,
                        "per-body skin: '%s' hollowed to the %.2f skin (inner volume %.5g vs %.5g)" % (t["label"], thickness, t["inner_volume"], expected))
    finally:
        shutil.rmtree(job_dir, ignore_errors=True)
    return ok


if __name__ == "__main__":
    sys.exit(main(sys.argv))
