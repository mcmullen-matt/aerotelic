"""Self-check of structural_fea.py (shells only, 2026-11-08), outside the application.

    python structural_fea_check.py [--keep DIR] [--solver auto|amgcl|sparse_lu]

Cases with closed forms, each run through the real job (the same
``run_job`` the application calls, through the same job.json):

  * a PRESSURISED SPHERE: an aluminium shell of thickness t on a unit
    icosphere under 1 bar outside. The membrane stress is p a / (2 t) in
    every direction (von Mises the same), the radial displacement
    -p a^2 (1 - nu) / (2 E t), and the pressure's resultant over the closed
    surface is zero.
  * the THIN-WALLED BOX BEAM against beam theory: the 1 x 10 x 1 m box of
    ue/Projects/simple_beam.yaml, 10 mm aluminium shells, clamped at
    y = min, two 500 N loads at the tip on the webs' mid-height. Timoshenko
    with the shells' mid-surface on the outer surface, I = 2 (b t (h/2)^2 +
    t h^3 / 12), the two webs the shear area: the linear thin shell is
    within 2% on a 0.25 m grid, and the flange stress is M c / I.
  * its NATURAL FREQUENCIES (2026-11-04): a square section's first bending
    pair is degenerate, and its frequency is the clamped-free beam's
    beta^2 / (2 pi L^2) sqrt(EI / rho A) lowered for shear flexibility and
    rotary inertia (Timoshenko) and for the tip cap's mass (cantilever_f1
    below); the modes after it are the 1 m skin panels' own.
  * a SUPPORT PATCH (2026-11-05): the same clamp plane limited to the
    upper half of the root fixes exactly the root nodes with z >= 0 and
    still carries the tip loads.
  * the box beam WITH RIBS (2026-11-08): three rib discs from rib_gmsh.py
    on the box's own nodes, as rib shells of a second region. The ribs
    stiffen the section a little (the tip moves less, not more), the mass
    is the skin's plus the ribs' rho t A, the reactions still balance, and
    per_material reports the ribs by kind.

``box_shell`` here is also what rib_gmsh_check.py builds its body from.
"""

import json
import os
import shutil
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gmsh_common as tg  # noqa: E402  (bootstraps the DLL directory)
import structural_fea as sf  # noqa: E402
import potential_flow_check as pfc  # noqa: E402  (icosphere)

import numpy as np  # noqa: E402

# The "Aluminium 2024-T3 sheet" preset: MIL-HDBK-5J Table 3.2.3.0(b1), A-basis
# (the ASM-typical figures this used to carry are no longer shipped).
AL = {"name": "Aluminium 2024-T3 sheet", "density": 2768.0, "youngs_modulus": 72.4e9, "poisson_ratio": 0.33, "allowable_stress": 324.0e6}


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def cantilever_f1(E, rho, I, A, L, nu, shear_area=None, tip_mass=0.0):
    """The first bending frequency (Hz) of a clamped-free beam: Euler-
    Bernoulli's beta^2 / (2 pi L^2) sqrt(EI / rho A) with beta L = 1.8751,
    lowered for shear flexibility and rotary inertia -- Timoshenko to first
    order in I / (A L^2), the shear over `shear_area` (the webs of a box,
    kappa A of a solid) -- and for a mass at the free end (Rayleigh: the
    mode's effective mass, a quarter of the beam's for the shape
    normalised to its tip, against the tip mass)."""
    beta2 = 1.8751 ** 2
    f = beta2 / (2 * np.pi * L ** 2) * np.sqrt(E * I / (rho * A))
    G = E / (2 * (1 + nu))
    As = shear_area if shear_area else A
    f /= np.sqrt(1.0 + beta2 * I / (A * L ** 2) * (1.0 + E * A / (G * As)))
    m_eff = 0.25 * rho * A * L
    return f * np.sqrt(m_eff / (m_eff + tip_mass))


def write_mesh(path, nodes):
    """nodes.bin as the application writes it: a uint32 count, float64 xyz."""
    with open(path, "wb") as f:
        f.write(struct.pack("<I", len(nodes)))
        f.write(np.ascontiguousarray(nodes, dtype="<f8").tobytes())


def write_faces(path, faces, region):
    """faces.bin / ribs.bin: a uint32 count, int32 triples, int32 region per face."""
    with open(path, "wb") as fh:
        fh.write(struct.pack("<I", len(faces)))
        fh.write(np.ascontiguousarray(faces, dtype="<i4").tobytes())
        fh.write(np.ascontiguousarray(region, dtype="<i4").tobytes())


def box_shell(lx, ly, lz, nx, ny, nz):
    """The closed surface of the box [-lx/2, lx/2] x [-ly/2, ly/2] x
    [-lz/2, lz/2] as nx x ny x nz cells: every face a grid of quads split in
    two, the nodes shared along the edges, wound outward (what the
    application's BoxMesh builds for a box primitive's cells)."""
    idx, verts, tris = {}, [], []

    def node(p):
        k = tuple(np.round(p, 9))
        if k not in idx:
            idx[k] = len(verts)
            verts.append(np.array(p, dtype=np.float64))
        return idx[k]

    def face(origin, du, dv, nu_, nv_, normal):
        for i in range(nu_):
            for j in range(nv_):
                p00 = origin + du * (i / nu_) + dv * (j / nv_)
                p10 = origin + du * ((i + 1) / nu_) + dv * (j / nv_)
                p11 = origin + du * ((i + 1) / nu_) + dv * ((j + 1) / nv_)
                p01 = origin + du * (i / nu_) + dv * ((j + 1) / nv_)
                a, b, c, d = node(p00), node(p10), node(p11), node(p01)
                if np.dot(np.cross(p10 - p00, p11 - p00), normal) > 0:
                    tris.extend([[a, b, c], [a, c, d]])
                else:
                    tris.extend([[a, c, b], [a, d, c]])

    X, Y, Z = np.eye(3)
    hx, hy, hz = lx / 2, ly / 2, lz / 2
    face(np.array([-hx, -hy, -hz]), 2 * hy * Y, 2 * hz * Z, ny, nz, -X)
    face(np.array([hx, -hy, -hz]), 2 * hy * Y, 2 * hz * Z, ny, nz, X)
    face(np.array([-hx, -hy, -hz]), 2 * hx * X, 2 * hy * Y, nx, ny, -Z)
    face(np.array([-hx, -hy, hz]), 2 * hx * X, 2 * hy * Y, nx, ny, Z)
    face(np.array([-hx, -hy, -hz]), 2 * hx * X, 2 * hz * Z, nx, nz, -Y)
    face(np.array([-hx, hy, -hz]), 2 * hx * X, 2 * hz * Z, nx, nz, Y)
    return np.array(verts), np.array(tris, dtype=np.int64)


def read_vtk(path):
    """Points, the cell blocks, point data and cell data of the legacy VTK
    structural_fea.py writes."""
    with open(path, "r", encoding="ascii") as f:
        lines = f.read().split("\n")
    i, n_cells = 0, 0
    points, cells, cell_types, point_data, cell_data = None, [], None, {}, {}
    section = None
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
            cells = [[int(v) for v in l.split()[1:]] for l in lines[i + 1:i + 1 + n_cells]]
            i += n_cells + 1
        elif parts[0] == "CELL_TYPES":
            cell_types = np.array(lines[i + 1:i + 1 + n_cells], dtype=np.int64)
            i += n_cells + 1
        elif parts[0] == "POINT_DATA":
            section, count = point_data, int(parts[1])
            i += 1
        elif parts[0] == "CELL_DATA":
            section, count = cell_data, int(parts[1])
            i += 1
        elif parts[0] in ("SCALARS", "VECTORS"):
            name = parts[1]
            width = 1 if parts[0] == "SCALARS" else 3
            start = i + 2 if parts[0] == "SCALARS" else i + 1
            block = np.array([l.split() for l in lines[start:start + count]], dtype=np.float64)
            section[name] = block.reshape(count, width) if width > 1 else block.reshape(-1)
            i = start + count
        else:
            i += 1
    return points, cells, cell_types, point_data, cell_data


def run(job_dir, job):
    with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
        json.dump(job, f, indent=1)
    return sf.run_job(job_dir)


def main(argv):
    keep, solver = None, "auto"
    args = list(argv[1:])
    while args:
        a = args.pop(0)
        if a == "--keep":
            keep = args.pop(0)
        elif a == "--solver":
            solver = args.pop(0)
    failures = 0
    job_dir = tempfile.mkdtemp(prefix="aerotelic_fea_check_")
    try:
        # ------------------------------------------------------------------
        # The pressurised sphere: a shell-only region.
        # ------------------------------------------------------------------
        sphere_dir = os.path.join(job_dir, "sphere")
        os.makedirs(sphere_dir)
        a, ts, p = 1.0, 0.02, 1.0e5
        verts_s, tris_s = pfc.icosphere(a, 3)
        write_mesh(os.path.join(sphere_dir, "nodes.bin"), verts_s)
        write_faces(os.path.join(sphere_dir, "faces.bin"), tris_s, np.ones(len(tris_s), dtype=np.int32))
        vtk_s = os.path.join(sphere_dir, "sphere.vtk")
        job_s = {
            "nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
            "regions": [{"id": 1, "label": "sphere", "skin": {"material": AL, "thickness": ts}, "rib": None}],
            "supports": [{"axis": "z", "value": "min", "tolerance": 0.1, "bounds": None}],
            "loads": {"load_factor": 0.0, "pressure": p, "pressure_file": None, "point_forces": []},
            "solver": {"echo_level": 0, "linear_solver": solver, "shell_element": "ShellThinElement3D3N", "modes": 0},
            "outputs": {"vtk": vtk_s, "result": "sphere_result.json"},
        }
        rs = run(sphere_dir, job_s)
        failures += not check(rs["ok"] and rs["shells"] == len(tris_s) and rs["rib_shells"] == 0 and rs["elements"] == len(tris_s), "sphere: a shell on each of the sphere's %d triangles" % len(tris_s))
        failures += not check(rs["nodes"] == len(verts_s) and rs["dofs"] == 6 * len(verts_s), "sphere: six degrees of freedom on every one of the %d nodes (%d dof)" % (len(verts_s), rs["dofs"]))
        expected_solver = "sparse_lu" if solver in ("auto", "sparse_lu") else solver
        failures += not check(rs["linear_solver"] == expected_solver, "sphere: the solver used is %s (asked for %s)" % (rs["linear_solver"], solver))
        failures += not check(abs(rs["applied_pressure"][2]) < 1e-6 * p * np.pi * a ** 2 and np.linalg.norm(rs["reaction"]) < 1e-3 * p * np.pi * a ** 2,
                              "sphere: a uniform pressure over a closed surface has no resultant, so the support carries nothing (|R| = %.3g N)" % np.linalg.norm(rs["reaction"]))
        failures += not check(rs["mean_normal_displacement"] < 0.0, "sphere: an external pressure moves the surface inward (mean u.n = %.3g m)" % rs["mean_normal_displacement"])
        area_s = tg.surface_area(verts_s, tris_s.astype(np.int32))
        failures += not check(abs(rs["mass"] - AL["density"] * ts * area_s) < 1e-9 * rs["mass"], "sphere: the mass is rho t A (%.5g kg)" % rs["mass"])
        failures += not check(len(rs["per_material"]) == 1 and rs["per_material"][0]["kind"] == "skin" and rs["per_material"][0]["margin"] > 0.0,
                              "sphere: per_material has the one skin entry with a positive margin (%.1f)" % rs["per_material"][0]["margin"])
        # Membrane stress and radial displacement, away from the support.
        points, cells, cell_types, pdata, cdata = read_vtk(vtk_s)
        failures += not check(len(points) == len(verts_s) and (cell_types == 5).sum() == len(tris_s) and len(cell_types) == len(tris_s), "sphere: the VTK holds the %d triangles and nothing else" % len(tris_s))
        failures += not check(all(k in pdata for k in ("displacement", "rotation", "von_mises")) and all(k in cdata for k in ("material", "kind", "von_mises_element")),
                              "sphere: the VTK holds displacement, rotation and von Mises per node, region, kind and von Mises per cell")
        tri = np.nonzero(cell_types == 5)[0]
        centroid_z = np.array([points[cells[i]].mean(axis=0)[2] for i in tri])
        away = centroid_z > -0.5
        vm = cdata["von_mises_element"][tri][away]
        sigma = p * a / (2 * ts)
        median = float(np.median(vm))
        failures += not check(abs(median / sigma - 1.0) < 0.10, "sphere: the shell's von Mises is the membrane stress p a / 2t = %.4g Pa (median away from the support %.4g)" % (sigma, median))
        u_r = -p * a ** 2 * (1 - AL["poisson_ratio"]) / (2 * AL["youngs_modulus"] * ts)
        wet = np.unique(np.concatenate([cells[i] for i in tri]))
        far = wet[points[wet][:, 2] > -0.5]
        radial = np.einsum("ij,ij->i", pdata["displacement"][far], points[far] / np.linalg.norm(points[far], axis=1)[:, None])
        failures += not check(abs(np.median(radial) / u_r - 1.0) < 0.10, "sphere: the radial displacement is -p a^2 (1 - nu) / 2 E t = %.4g m (median %.4g)" % (u_r, float(np.median(radial))))
        print("  sphere: %d dof in %.1f s; membrane %.4g Pa (median %.4g, peak %.4g); u_r %.4g m (median %.4g)"
              % (rs["dofs"], rs["seconds_solve"], sigma, median, vm.max(), u_r, float(np.median(radial))))

        # ------------------------------------------------------------------
        # The thin-walled box beam against beam theory (2026-11-02): the
        # 1 x 10 x 1 m box of ue/Projects/simple_beam.yaml, 10 mm aluminium
        # shells, clamped at y = min, two 500 N loads at the tip on the webs'
        # mid-height. Timoshenko with the shells' mid-surface on the outer
        # surface, I = 2 (b t (h/2)^2 + t h^3 / 12), the two webs the shear
        # area: the linear thin shell is within 2% on a 0.25 m grid.
        # ------------------------------------------------------------------
        beam_dir = os.path.join(job_dir, "beam")
        os.makedirs(beam_dir)
        bL, bb, bh, bt = 10.0, 1.0, 1.0, 0.01
        bverts, btris = box_shell(bb, bL, bh, 4, 40, 4)
        write_mesh(os.path.join(beam_dir, "nodes.bin"), bverts)
        write_faces(os.path.join(beam_dir, "faces.bin"), btris, np.ones(len(btris), dtype=np.int32))
        P_tip = 1000.0
        job_b = {
            "nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
            "regions": [{"id": 1, "label": "beam", "skin": {"material": AL, "thickness": bt}, "rib": None}],
            "supports": [{"axis": "y", "value": "min", "tolerance": 0.0, "bounds": None}],
            "loads": {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None,
                      "point_forces": [{"at": [0.5, 5.0, 0.0], "force": [0.0, 0.0, -P_tip / 2]}, {"at": [-0.5, 5.0, 0.0], "force": [0.0, 0.0, -P_tip / 2]}]},
            "solver": {"echo_level": 0, "linear_solver": solver, "shell_element": "ShellThinElement3D3N", "modes": 4},
            "outputs": {"vtk": os.path.join(beam_dir, "beam.vtk"), "result": "beam_result.json"},
        }
        rb = run(beam_dir, job_b)
        E_b, nu_b = AL["youngs_modulus"], AL["poisson_ratio"]
        G_b = E_b / (2 * (1 + nu_b))
        I_b = 2 * (bb * bt * (bh / 2) ** 2 + bt * bh ** 3 / 12)
        delta_b = P_tip * bL ** 3 / (3 * E_b * I_b) + P_tip * bL / (G_b * 2 * bh * bt)
        pts_b, cells_b, types_b, pd_b, cd_b = read_vtk(os.path.join(beam_dir, "beam.vtk"))
        tip_b = pts_b[:, 1] > pts_b[:, 1].max() - 1e-6
        tip_uz = float(pd_b["displacement"][tip_b, 2].mean())
        failures += not check(rb["ok"] and rb["balanced"] and rb["balance_error"] <= sf.BALANCE_TOLERANCE and rb["warnings"] == [] and abs(rb["reaction"][2] - P_tip) < 0.01 * P_tip,
                              "beam: the clamp carries the two tip loads with no warning (%.4g vs %.4g N)" % (rb["reaction"][2], P_tip))
        failures += not check(abs(-tip_uz / delta_b - 1.0) < 0.02, "beam: the thin shell's tip deflection is Timoshenko's (%.4g vs %.4g m, %.1f%%)" % (-tip_uz, delta_b, 100 * (-tip_uz / delta_b - 1)))
        failures += not check(abs(rb["max_displacement"] + tip_uz) < 0.1 * abs(tip_uz) and rb["max_displacement_at"][1] > 0.9 * bL / 2, "beam: the largest displacement is at the tip (y = %.3f)" % rb["max_displacement_at"][1])
        failures += not check(rb["support_nodes"] == rb["support_corner_nodes"] == int((np.abs(bverts[:, 1] - bverts[:, 1].min()) < 1e-9).sum()), "beam: the clamp is the root ring and cap alone (%d nodes)" % rb["support_nodes"])
        # The flange stress at 2.5 m from the root, M c / I.
        cen_b = np.array([pts_b[c].mean(axis=0) for c in cells_b])
        band = (np.abs(cen_b[:, 1] + 2.5) < 0.3) & (np.abs(cen_b[:, 2]) > 0.45)
        sigma_b = P_tip * (bL - 2.5) * (bh / 2) / I_b
        med = float(np.median(cd_b["von_mises_element"][band]))
        failures += not check(abs(med / sigma_b - 1.0) < 0.05, "beam: the flange von Mises is M c / I (median %.4g vs %.4g Pa)" % (med, sigma_b))
        failures += not check(abs(rb["mass"] - AL["density"] * bt * tg.surface_area(bverts, btris.astype(np.int32))) < 1e-9 * rb["mass"], "beam: the mass is rho t A (%.5g kg)" % rb["mass"])
        print("  beam: %d dof in %.1f s; tip %.4g m vs %.4g; flange %.4g Pa vs %.4g" % (rb["dofs"], rb["seconds_solve"], -tip_uz, delta_b, med, sigma_b))
        # The natural frequencies: a square box's first bending pair is
        # degenerate, at the clamped-free beam's frequency lowered by the
        # webs' shear flexibility, rotary inertia and the tip cap's mass
        # (the root cap is clamped). The modes after it are the 1 m skin
        # panels' own, well below the second beam mode (73 Hz).
        f_b = rb["natural_frequencies"]
        cap_mass = AL["density"] * bt * bb * bh
        f1_b = cantilever_f1(E_b, AL["density"], I_b, 2 * (bb + bh) * bt, bL, nu_b, shear_area=2 * bh * bt, tip_mass=cap_mass)
        failures += not check(len(f_b) == 4 and abs(f_b[1] / f_b[0] - 1.0) < 0.005, "beam: the first bending pair is degenerate (%.4g and %.4g Hz)" % tuple(f_b[:2]) if len(f_b) >= 2 else "beam: four natural frequencies came back")
        failures += not check(len(f_b) >= 1 and abs(f_b[0] / f1_b - 1.0) < 0.04,
                              "beam: the first bending frequency is the clamped-free beam's with shear, rotary inertia and the tip cap, %.4g Hz (got %.4g, %.1f%%)" % (f1_b, f_b[0] if f_b else 0.0, 100 * (f_b[0] / f1_b - 1) if f_b else 0.0))
        failures += not check(len(f_b) == 4 and f_b[2] > 1.5 * f_b[0] and f_b[3] >= f_b[2], "beam: the modes after the pair are higher and ascending (%s Hz)" % ", ".join("%.4g" % f for f in f_b))
        root_m = np.abs(pts_b[:, 1] - pts_b[:, 1].min()) < 1e-6
        failures += not check("mode_1" in pd_b and "mode_4" in pd_b and pd_b["mode_1"].shape == (len(bverts), 3) and np.abs(pd_b["mode_1"][root_m]).max() < 1e-9
                              and abs(np.linalg.norm(pd_b["mode_1"], axis=1).max() - 1.0) < 1e-6 and np.linalg.norm(pd_b["mode_1"][tip_b], axis=1).max() > 0.9,
                              "beam: the first mode shape is in the VTK, still at the clamp, scaled to a unit peak and largest at the tip")
        print("  beam modes (%s, %.2f s): %s Hz; theory f1 %.4g" % (rb["eigensolver"], rb["seconds_modes"], ", ".join("%.4g" % f for f in f_b), f1_b))

        # A support PATCH (2026-11-05): the same clamp plane y = min limited
        # to the upper half of the root (the in-plane axes of a y plane are
        # z then x): the fixed nodes are exactly the root nodes with z >= 0,
        # and the half clamp still carries the tip loads.
        job_p = dict(job_b)
        job_p["supports"] = [{"axis": "y", "value": "min", "tolerance": 0.0, "bounds": [0.0, 1.0, -1.0, 1.0]}]
        job_p["solver"] = dict(job_b["solver"], modes=0)
        job_p["outputs"] = {"vtk": os.path.join(beam_dir, "beam_patch.vtk"), "result": "beam_patch_result.json"}
        rp = run(beam_dir, job_p)
        root = np.abs(bverts[:, 1] - bverts[:, 1].min()) < 1e-9
        expected_patch = int((root & (bverts[:, 2] >= 0.0)).sum())
        failures += not check(rp["ok"] and rp["support_nodes"] == expected_patch and rp["support_nodes"] < rb["support_nodes"] and rp["support_planes"][0]["bounds"] == [0.0, 1.0, -1.0, 1.0],
                              "patch: the clamp limited to z >= 0 fixes the upper half of the root (%d of %d nodes)" % (rp["support_nodes"], rb["support_nodes"]))
        failures += not check(rp["balanced"] and abs(rp["reaction"][2] - P_tip) < 0.01 * P_tip, "patch: the half clamp carries the tip loads (%.4g vs %.4g N)" % (rp["reaction"][2], P_tip))

        # ------------------------------------------------------------------
        # The box beam with ribs (2026-11-08): three discs from rib_gmsh.py
        # at y = -2.5, 0, 2.5 on the 2 x 20 x 2 grid (its vertices lie on
        # those planes, so no cut is needed), as a second region of 5 mm rib
        # shells. The ribs hold the section square, so the tip moves no more
        # than without them, and everything else still adds up.
        # ------------------------------------------------------------------
        import rib_gmsh as rg
        rib_dir = os.path.join(job_dir, "ribs")
        os.makedirs(rib_dir)
        rverts, rtris = box_shell(bb, bL, bh, 2, 20, 2)
        tg.write_tri(os.path.join(rib_dir, "outer.tri"), rverts, rtris)
        with open(os.path.join(rib_dir, "job.json"), "w", encoding="utf-8") as f:
            json.dump({"outer": "outer.tri", "rib_size": 0.25, "plane_tolerance": 0.0,
                       "ribs": [{"id": 2, "normal": [0.0, 1.0, 0.0], "offset": y, "height": 0.0} for y in (-2.5, 0.0, 2.5)],
                       "outputs": {"nodes": "nodes_ribs.bin", "result": "ribs_result.json", "ribs": "ribs.bin"}}, f)
        rr = rg.run_job(rib_dir)
        with open(os.path.join(rib_dir, "nodes_ribs.bin"), "rb") as f:
            assert f.read(8) == rg.NODES_MAGIC
            (nn,) = struct.unpack("<I", f.read(4))
            rnodes = np.frombuffer(f.read(nn * 24), dtype="<f8").reshape(nn, 3)
        failures += not check(rr["ok"] and rr["surface_nodes"] == len(rverts) and np.allclose(rnodes[:len(rverts)], rverts) and rr["rib_faces"] > 0,
                              "ribs: rib_gmsh.py put %d rib faces on the box's own %d nodes plus %d" % (rr["rib_faces"], len(rverts), nn - len(rverts)))
        # Without ribs, on the same coarse grid, for the comparison.
        write_mesh(os.path.join(rib_dir, "nodes.bin"), rverts)
        write_faces(os.path.join(rib_dir, "faces.bin"), rtris, np.ones(len(rtris), dtype=np.int32))
        job_0 = dict(job_b)
        job_0["solver"] = dict(job_b["solver"], modes=0)
        job_0["outputs"] = {"vtk": os.path.join(rib_dir, "plain.vtk"), "result": "plain_result.json"}
        r0 = run(rib_dir, job_0)
        # With ribs: the nodes rib_gmsh.py wrote (the file keeps its header,
        # which the reader skips), the skin faces on the first of them.
        rt = 0.005
        job_r = dict(job_0)
        job_r["nodes"] = "nodes_ribs.bin"
        job_r["rib_faces"] = "ribs.bin"
        job_r["regions"] = [{"id": 1, "label": "beam", "skin": {"material": AL, "thickness": bt}, "rib": None},
                            {"id": 2, "label": "frames", "skin": None, "rib": {"material": AL, "thickness": rt}}]
        job_r["outputs"] = {"vtk": os.path.join(rib_dir, "ribbed.vtk"), "result": "ribbed_result.json"}
        rrb = run(rib_dir, job_r)
        failures += not check(rrb["ok"] and rrb["nodes"] == nn and rrb["dofs"] == 6 * nn and rrb["shells"] == len(rtris) + rr["rib_faces"] and rrb["rib_shells"] == rr["rib_faces"] and rrb["wetted_faces"] == len(rtris),
                              "ribs: %d skin shells + %d rib shells on %d nodes, the skin faces alone wetted" % (len(rtris), rr["rib_faces"], nn))
        failures += not check(rrb["balanced"] and rrb["warnings"] == [] and abs(rrb["reaction"][2] - P_tip) < 0.01 * P_tip, "ribs: the clamp still carries the tip loads (%.4g N)" % rrb["reaction"][2])
        expected_mass = r0["mass"] + AL["density"] * rt * rr["rib_area"]
        failures += not check(abs(rrb["mass"] - expected_mass) < 1e-6 * expected_mass, "ribs: the mass is the skin's plus the ribs' rho t A (%.5g vs %.5g kg)" % (rrb["mass"], expected_mass))
        failures += not check(rrb["max_displacement"] <= r0["max_displacement"] * (1.0 + 1e-6) and rrb["max_displacement"] > 0.5 * r0["max_displacement"],
                              "ribs: the ribbed box moves no more than the plain one, and not wildly less (%.4g vs %.4g m)" % (rrb["max_displacement"], r0["max_displacement"]))
        kinds = [(e["id"], e["kind"]) for e in rrb["per_material"]]
        failures += not check(kinds == [(1, "skin"), (2, "rib")] and rrb["per_material"][1]["max_von_mises"] > 0.0 and rrb["per_material"][1]["max_von_mises"] < rrb["per_material"][0]["max_von_mises"],
                              "ribs: per_material reports the skin and the ribs by kind, the ribs less stressed than the flanges (%.3g vs %.3g Pa)" % (rrb["per_material"][1]["max_von_mises"], rrb["per_material"][0]["max_von_mises"]))
        pts_r, cells_r, types_r, pd_r, cd_r = read_vtk(os.path.join(rib_dir, "ribbed.vtk"))
        failures += not check(len(pts_r) == nn and (cd_r["kind"] == 2).sum() == rr["rib_faces"] and (cd_r["material"][cd_r["kind"] == 2] == 2).all(),
                              "ribs: the VTK marks the rib shells as kind 2 of region 2")
        print("  ribs: %d dof in %.1f s; tip %.4g m vs %.4g plain; rib peak %.3g Pa" % (rrb["dofs"], rrb["seconds_solve"], rrb["max_displacement"], r0["max_displacement"], rrb["per_material"][1]["max_von_mises"]))

        if keep:
            os.makedirs(keep, exist_ok=True)
            for root_dir, _, files in os.walk(job_dir):
                for name in files:
                    rel = os.path.relpath(os.path.join(root_dir, name), job_dir)
                    os.makedirs(os.path.dirname(os.path.join(keep, rel)) or keep, exist_ok=True)
                    shutil.copy2(os.path.join(root_dir, name), os.path.join(keep, rel))
            print("kept: %s" % keep)
    finally:
        shutil.rmtree(job_dir, ignore_errors=True)

    print("RESULT: %d failure(s)" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
