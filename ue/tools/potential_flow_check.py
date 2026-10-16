"""Self-check of potential_flow.py, outside the application.

    python potential_flow_check.py [body.tri ...] [--keep DIR] [--refine N]

With no body it runs the one case a potential flow solver can be marked
against by hand: a SPHERE in uniform flow, where the exact solution is
known in closed form. On the surface of a sphere of radius a the potential
is phi = U x (1 + a^3 / (2 r^3)), the speed is (3/2) U sin(theta) with
theta measured from the upstream stagnation point, and so

    Cp = 1 - (9/4) sin^2(theta),

which runs from +1 at the two stagnation points to -1.25 around the
equator. The check builds an icosphere, runs the real job (the same
``run_job`` the application calls, through the same job.json), reads the
VTK that comes out and compares every wetted node against that formula.
It also checks what the solver cannot get wrong without being wrong
everywhere: the far field is undisturbed, the net force is zero
(d'Alembert's paradox), and the wetted boundary of the fluid mesh encloses
the body's own volume -- i.e. the mesh really is bounded by the body's own
triangulation.

What it converges to, measured (icosphere at --refine 3, 4, 5; element
size at the wall = the mean edge of the sphere's own triangles):

    triangles   h        Cp rms error   net force / q*area
      1280      0.150      0.179            3.5e-03
      5120      0.075      0.106            7.0e-04
     20480      0.038      0.065            6.6e-04

The error is fore-aft symmetric and is a uniform 3-4% deficit in the
surface speed, worst around the equator where the speed is highest. That
is the first-order bias of the postprocessing, not of the solve: the
velocity is constant over a linear tetrahedron, and the value a wall node
is given is the average over the tetrahedra around it, all of which lie
inside the fluid where the flow has already slowed towards the free
stream. It falls off with the element size at the wall (order ~0.7-1.0
here), so the resolution of the body's own triangulation is what sets the
accuracy of the pressures on it.

With bodies given (the application's .tri files, e.g. written by a debug
build) it runs them instead and reports the same statistics without the
exact comparison. With --keep the job directory and the VTK are left in
DIR.
"""

import json
import os
import shutil
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import potential_flow as pf  # noqa: E402

import numpy as np  # noqa: E402

TRI_MAGIC = b"AEROTRI1"


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def write_tri(path, verts, faces):
    with open(path, "wb") as f:
        f.write(TRI_MAGIC)
        f.write(struct.pack("<II", len(verts), len(faces)))
        f.write(np.ascontiguousarray(verts, dtype="<f8").tobytes())
        f.write(np.ascontiguousarray(faces, dtype="<i4").tobytes())


def icosphere(radius, subdivisions):
    """A closed, outward icosphere: the icosahedron with every triangle cut
    into four and the new vertices pushed out onto the sphere."""
    t = (1.0 + 5.0 ** 0.5) / 2.0
    verts = np.array([
        [-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0],
        [0, -1, t], [0, 1, t], [0, -1, -t], [0, 1, -t],
        [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]], dtype=np.float64)
    faces = np.array([
        [0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11],
        [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8],
        [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9],
        [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1]], dtype=np.int32)
    for _ in range(subdivisions):
        midpoint = {}
        new_faces = []
        verts = list(verts)
        for a, b, c in faces:
            m = []
            for u, v in ((a, b), (b, c), (c, a)):
                key = (min(u, v), max(u, v))
                if key not in midpoint:
                    midpoint[key] = len(verts)
                    verts.append((np.asarray(verts[u]) + np.asarray(verts[v])) * 0.5)
                m.append(midpoint[key])
            new_faces += [[a, m[0], m[2]], [b, m[1], m[0]], [c, m[2], m[1]], [m[0], m[1], m[2]]]
        verts = np.array(verts, dtype=np.float64)
        faces = np.array(new_faces, dtype=np.int32)
    verts = verts / np.linalg.norm(verts, axis=1)[:, None] * radius
    return np.ascontiguousarray(verts), np.ascontiguousarray(faces)


def read_vtk(path):
    """Points, the triangle cells (the wetted surface: region 1) and the
    point data of the legacy VTK potential_flow.py writes."""
    with open(path, "r", encoding="ascii") as f:
        lines = f.read().split("\n")
    i, points, cells, cell_types, point_data = 0, None, [], None, {}
    n_cells = 0
    while i < len(lines):
        parts = lines[i].split()
        if not parts:
            i += 1
            continue
        if parts[0] == "POINTS":
            n = int(parts[1])
            points = np.array([l.split() for l in lines[i + 1:i + 1 + n]], dtype=np.float64)
            i += n + 1
        elif parts[0] == "CELLS":
            n_cells = int(parts[1])
            cells = [l.split() for l in lines[i + 1:i + 1 + n_cells]]
            i += n_cells + 1
        elif parts[0] == "CELL_TYPES":
            cell_types = np.array(lines[i + 1:i + 1 + n_cells], dtype=np.int64)
            i += n_cells + 1
        elif parts[0] in ("SCALARS", "VECTORS"):
            name = parts[1]
            width = 1 if parts[0] == "SCALARS" else 3
            # SCALARS is followed by its LOOKUP_TABLE line, VECTORS is not.
            start = i + 2 if parts[0] == "SCALARS" else i + 1
            n = len(points)
            block = np.array([l.split() for l in lines[start:start + n]], dtype=np.float64)
            point_data[name] = block.reshape(n, width) if width > 1 else block.reshape(-1)
            i = start + n
        else:
            i += 1
    tris = np.array([c[1:4] for c, t in zip(cells, cell_types) if t == 5], dtype=np.int64)
    tets = np.array([c[1:5] for c, t in zip(cells, cell_types) if t == 10], dtype=np.int64)
    return points, tets, tris, point_data


def main(argv):
    # 5 subdivisions: the coarsest sphere whose surface pressures are
    # within 0.1 rms of the exact solution (see the table above). About
    # 240,000 tetrahedra, a quarter of a minute all told.
    bodies, keep, refine = [], None, 5
    args = list(argv[1:])
    while args:
        a = args.pop(0)
        if a == "--keep":
            keep = args.pop(0)
        elif a == "--refine":
            refine = int(args.pop(0))
        else:
            bodies.append(a)

    job_dir = tempfile.mkdtemp(prefix="aerotelic_pf_check_")
    failures = 0
    try:
        sphere = not bodies
        if sphere:
            verts, faces = icosphere(1.0, refine)
            write_tri(os.path.join(job_dir, "body0.tri"), verts, faces)
            entries = [{"label": "Sphere", "file": "body0.tri"}]
            print("sphere: radius 1, %d triangles (icosphere, %d subdivisions)" % (len(faces), refine))
        else:
            entries = []
            for i, path in enumerate(bodies):
                name = "body%d.tri" % i
                shutil.copyfile(path, os.path.join(job_dir, name))
                entries.append({"label": os.path.basename(path), "file": name})

        vtk = os.path.join(job_dir, "solution.vtk")
        job = {
            "bodies": entries,
            "far_field": {"size_factor": 5.0},
            "volume": {"algorithm": 1, "optimize": True, "optimize_netgen": False},
            "flow": {"speed": 50.0, "angle_of_attack_deg": 0.0, "speed_of_sound": 340.0,
                     "density": 1.225, "element_type": "incompressible"},
            "solver": {"echo_level": 0},
            "outputs": {"vtk": vtk, "result": "result.json"},
        }
        with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump(job, f, indent=1)

        result = pf.run_job(job_dir)
        failures += not check(result["ok"], "the job ran")

        points, tets, tris, data = read_vtk(vtk)
        failures += not check(len(points) == result["mesh_nodes"] and len(tets) == result["mesh_tets"],
                              "the VTK holds the whole grid (%d nodes, %d tetrahedra)" % (len(points), len(tets)))
        failures += not check(len(tris) == result["wetted_triangles"],
                              "the VTK holds the wetted surface as triangles (%d)" % len(tris))
        for name in ("VELOCITY_POTENTIAL", "PRESSURE_COEFFICIENT", "VELOCITY"):
            failures += not check(name in data, "the VTK holds %s" % name)
        cp, vel = data["PRESSURE_COEFFICIENT"], data["VELOCITY"]
        failures += not check(np.isfinite(cp).all() and np.isfinite(vel).all(), "the solution is finite everywhere")

        # The mesh is bounded by the body's own triangulation.
        rel = abs(result["wetted_volume"] - result["body_volume"]) / abs(result["body_volume"])
        failures += not check(rel < 1e-9, "the wetted boundary encloses the body's volume (%.3g vs %.3g, relative %.2e)"
                              % (result["wetted_volume"], result["body_volume"], rel))
        # ... and the tetrahedra fill the box minus the bodies.
        fill = abs(result["fluid_volume"] - (result["box_volume"] - result["body_volume"])) / result["box_volume"]
        failures += not check(fill < 1e-9, "the tetrahedra fill the box minus the bodies (relative %.2e)" % fill)

        # d'Alembert: no wake, no vorticity, no force.
        failures += not check(result["force_coefficient"] < 0.05,
                              "the net force is zero to the discretisation (%.3e of q*frontal area)" % result["force_coefficient"])

        # The far field is undisturbed: the nodes a long way out move at
        # the free-stream speed.
        centre = points.mean(axis=0)
        far = np.linalg.norm(points - centre, axis=1) > 0.8 * result["far_field_size"] * 0.5
        speed = np.linalg.norm(vel, axis=1)
        if far.any():
            worst = float(np.abs(speed[far] / result["free_stream_speed"] - 1.0).max())
            failures += not check(worst < 0.05, "the far field is undisturbed (worst %.2f%% off the free-stream speed)" % (100.0 * worst))

        if sphere:
            on_body = np.unique(tris)
            p = points[on_body]
            r = np.linalg.norm(p, axis=1)
            failures += not check(abs(r.mean() - 1.0) < 1e-9, "every wetted node is on the sphere")
            # theta from the upstream stagnation point, which is at -x.
            sin2 = 1.0 - (p[:, 0] / r) ** 2
            exact = 1.0 - 2.25 * sin2
            err = cp[on_body] - exact
            rms = float(np.sqrt((err ** 2).mean()))
            print("sphere: Cp %.4f .. %.4f against the exact -1.25 .. 1.00; error rms %.4f, worst %.4f"
                  % (cp[on_body].min(), cp[on_body].max(), rms, float(np.abs(err).max())))
            failures += not check(rms < 0.1, "the surface Cp matches 1 - (9/4) sin^2(theta) (rms %.4f)" % rms)
            failures += not check(result["max_cp_body"] > 0.8, "the stagnation point reaches Cp = 1 (%.4f)" % result["max_cp_body"])
            failures += not check(-1.45 < result["min_cp_body"] < -1.05, "the suction peak reaches Cp = -1.25 (%.4f)" % result["min_cp_body"])
            # The speed on the surface, 1.5 U at the equator.
            equator = np.abs(p[:, 0]) < 0.05
            if equator.any():
                worst = float(np.abs(speed[on_body][equator] / result["free_stream_speed"] - 1.5).max())
                failures += not check(worst < 0.15, "the surface speed at the equator is 1.5 U (worst %.3f off)" % worst)

        if keep:
            os.makedirs(keep, exist_ok=True)
            for name in os.listdir(job_dir):
                shutil.copy2(os.path.join(job_dir, name), os.path.join(keep, name))
            print("kept: %s" % keep)
    finally:
        shutil.rmtree(job_dir, ignore_errors=True)

    print("RESULT: %d failure(s)" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
