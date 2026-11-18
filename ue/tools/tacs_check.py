"""Analysis > TACS (FEA) > Check Installation: is TACS reachable from the
application, and does it solve?

Run by the application through the same runner as the solver
(Entity/AerotelicMeshLib.cpp, inside WSL), on a job directory holding only
a job.json {"outputs": {"result": "result.json"}}; also by hand:
    python tacs_check.py <job directory>

Reports the interpreter, the platform, the versions of tacs, mpi4py,
pyNastran and numpy, whether gmsh is there (the check script's rib case
wants it), and a two-triangle cantilever plate built and solved through
structural_fea_tacs.py's own path -- the one solve that proves the whole
chain (pyNastran -> pyTACS -> solve -> stresses) works -- with its time.
Writes result.json {"ok", "report": [lines], "error"}; the report is what
the dialog shows.
"""

import json
import os
import platform
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _version(name):
    try:
        mod = __import__(name)
    except Exception as exc:  # noqa: BLE001
        return None, "%s: %s" % (type(exc).__name__, exc)
    version = getattr(mod, "__version__", None)
    if version is None:
        # the tacs package has no __version__; its distribution metadata has 3.12.3
        try:
            import importlib.metadata as metadata
            version = metadata.version(name)
        except Exception:  # noqa: BLE001
            version = "present"
    return version, None


def run(job_dir):
    lines = []
    ok = True
    lines.append("interpreter: %s" % sys.executable)
    lines.append("platform: %s %s (%s)" % (platform.system(), platform.release(), platform.machine()))
    lines.append("python: %s" % platform.python_version())
    for name in ("numpy", "mpi4py", "pyNastran", "tacs"):
        v, err = _version(name)
        if v is None:
            ok = False
            lines.append("%s: NOT IMPORTABLE (%s)" % (name, err))
        else:
            lines.append("%s: %s" % (name, v))
    v, err = _version("gmsh")
    lines.append("gmsh: %s" % (v if v is not None else "absent (fine; only structural_fea_check.py's rib case needs it)"))
    if ok:
        try:
            import numpy as np
            import structural_fea_tacs as st
            import structural_fea_check as sc  # the plate() mesher and the AL card
            plate_dir = os.path.join(job_dir, "plate")
            os.makedirs(plate_dir, exist_ok=True)
            verts, tris = sc.plate(1.0, 0.5, 4, 2)
            sc.write_mesh(os.path.join(plate_dir, "nodes.bin"), verts)
            sc.write_faces(os.path.join(plate_dir, "faces.bin"), tris, np.ones(len(tris), dtype=np.int32))
            tip = verts[np.abs(verts[:, 0] - 1.0) < 1e-9]
            job = {
                "nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
                "regions": [{"id": 1, "label": "plate", "skin": {"material": sc.AL, "thickness": 0.01}, "rib": None}],
                "supports": [{"axis": "x", "value": "min", "tolerance": 0.0, "bounds": None}],
                "loads": {"load_factor": 1.0, "pressure": 0.0, "pressure_file": None,
                          "point_forces": [{"at": [float(p[0]), float(p[1]), float(p[2])], "force": [0.0, 0.0, -100.0 / len(tip)]} for p in tip]},
                "solver": {"echo_level": 0, "modes": 2},
                "outputs": {"vtk": os.path.join(plate_dir, "plate.vtk"), "result": "plate_result.json"},
            }
            with open(os.path.join(plate_dir, "job.json"), "w", encoding="utf-8") as f:
                json.dump(job, f, indent=1)
            t0 = time.time()
            r = st.run_job(plate_dir)
            E, I_ = sc.AL["youngs_modulus"], 0.5 * 0.01 ** 3 / 12.0
            delta = 100.0 * 1.0 ** 3 / (3.0 * E * I_)
            lines.append("solve: a 1 x 0.5 m cantilever plate of %d shells under 100 N at the tip and 1 g, %d dof, %.2f s on TACS %s"
                         % (r["shells"], r["dofs"], time.time() - t0, r["tacs_version"]))
            lines.append("       tip deflection %.4g m (the beam's %.4g), reactions (%.4g, %.4g, %.4g) N against %.4g N applied: %s"
                         % (r["max_displacement"], delta, r["reaction"][0], r["reaction"][1], r["reaction"][2],
                            100.0 + r["mass"] * st.G0, "balanced" if r["balanced"] else "NOT BALANCED"))
            if r["natural_frequencies"]:
                lines.append("       first modes %s Hz" % ", ".join("%.4g" % f for f in r["natural_frequencies"]))
            sens = r.get("sensitivities", {})
            if sens.get("functions"):
                f0 = sens["functions"][0]
                lines.append("       adjoint: d %s / d thickness = %s" % (f0["name"], ", ".join("%.4g" % g for g in f0["gradient"])))
            ok = ok and bool(r["balanced"])
        except Exception as exc:  # noqa: BLE001
            ok = False
            lines.append("solve: FAILED -- %s: %s" % (type(exc).__name__, exc))
    report = "\n".join(lines)
    print(report, flush=True)
    with open(os.path.join(job_dir, "result.json"), "w", encoding="utf-8") as f:
        json.dump({"ok": ok, "report": lines, "error": None if ok else "TACS is not usable from this environment; see the report."}, f, indent=1)
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: tacs_check.py <job directory>")
        sys.exit(2)
    try:
        sys.exit(run(sys.argv[1]))
    except Exception as exc:  # noqa: BLE001
        with open(os.path.join(sys.argv[1], "result.json"), "w", encoding="utf-8") as f:
            json.dump({"ok": False, "error": "%s: %s" % (type(exc).__name__, exc), "report": []}, f, indent=1)
        print("error: %s: %s" % (type(exc).__name__, exc))
        sys.exit(1)
