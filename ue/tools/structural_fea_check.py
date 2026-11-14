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
  * COMPOSITE LAMINATES (2026-11-12) on a cantilever plate of eight plies
    of an AS4-class tape: with the fibres along the plate (the material
    axes from the ``axes`` file) the tip deflection is the E1 beam's, with
    the axes turned 90 degrees the E2 beam's (fourteen times more), with
    no axes file the region's longest extent stands in (E1 again); a drape
    file turning every ply 90 degrees gives the E2 answer too, and a woven
    ply sheared 60 degrees doubles its mass (1 / cos 60); Tsai-Wu reserve
    factors are reported with the margin, the VTK carries them, and an
    unsymmetric [0_4/90_4] strip under tension bends towards its stiff
    outer plies (the first row of the stack is on the +normal side).
  * a MASS ITEM (2026-11-13): 50 kg hung by the RBE3-type constraint on
    six nodes of the clamped box beam's tip under 1 g -- the clamp
    carries the skin's and the item's weight exactly, the tip moves as
    under a point force of that weight, an attachment on a line is
    refused.
  * FREE FLIGHT, inertia relief (2026-11-13): a plate with an offset pod
    under lift equal to its weight -- the reference node reacts nothing
    (force and moment to 1e-9 of the loads), the body pitches at
    I_cg^-1 M, the stresses are the same from any reference node,
    gravity alone loads nothing, lift of twice the weight is the trimmed
    2 g case, the free-free modes come with the rigid ones dropped, and
    planes with free flight (or nothing at all) are refused.

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


def plate(lx, ly, nx, ny):
    """An open flat plate in z = 0, [0, lx] x [0, ly], nx x ny cells split
    in two, wound with the normal along +z."""
    verts = np.array([[lx * i / nx, ly * j / ny, 0.0] for j in range(ny + 1) for i in range(nx + 1)])
    tris = []
    for j in range(ny):
        for i in range(nx):
            a = j * (nx + 1) + i
            tris.append([a, a + 1, a + nx + 2])
            tris.append([a, a + nx + 2, a + nx + 1])
    return verts, np.array(tris, dtype=np.int64)


# An AS4-class carbon/epoxy tape as a lamina card (SI), with strengths.
TAPE = {"form": "UD", "lamina": "AS4-class tape", "density": 1580.0, "thickness": 0.000125,
        "e1": 142e9, "e2": 10.3e9, "nu12": 0.27, "g12": 7.2e9, "g13": 7.2e9, "g23": 3.5e9,
        "t1": 2280e6, "c1": 1440e6, "t2": 57e6, "c2": 228e6, "s12": 71e6, "s13": 71e6, "s23": 71e6, "locking_angle": 0.0}
CLOTH = dict(TAPE, form="Woven", lamina="cloth", e1=60e9, e2=60e9, nu12=0.05, g12=4.5e9, g13=4.5e9, g23=2.0e9, thickness=0.0002, locking_angle=35.0)


def laminate_card(name, plies):
    """A laminate material card as the application writes it: the expanded
    stack (outer face first), each ply a lamina card with its angle."""
    return {"name": name, "density": 0.0, "youngs_modulus": 0.0, "poisson_ratio": 0.0, "allowable_stress": 0.0,
            "plies": [dict(lamina, angle=angle) for lamina, angle in plies]}


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

        # ------------------------------------------------------------------
        # Composite laminates (2026-11-12): a cantilever plate of eight
        # tape plies, clamped at x = 0, a tip load along -z shared by the
        # tip nodes. Beam theory with E1 (fibres along x) or E2 (across).
        # ------------------------------------------------------------------
        lam_dir = os.path.join(job_dir, "laminate")
        os.makedirs(lam_dir)
        pL, pB, pnx, pny = 1.0, 0.2, 20, 4
        pverts, ptris = plate(pL, pB, pnx, pny)
        write_mesh(os.path.join(lam_dir, "nodes.bin"), pverts)
        write_faces(os.path.join(lam_dir, "faces.bin"), ptris, np.ones(len(ptris), dtype=np.int32))
        n_ply = 8
        t_lam = n_ply * TAPE["thickness"]
        I_p = pB * t_lam ** 3 / 12.0
        F_tip = 10.0
        tip_nodes = pverts[np.abs(pverts[:, 0] - pL) < 1e-9]
        tip_forces = [{"at": [float(p[0]), float(p[1]), float(p[2])], "force": [0.0, 0.0, -F_tip / len(tip_nodes)]} for p in tip_nodes]
        delta_e1 = F_tip * pL ** 3 / (3 * TAPE["e1"] * I_p)
        delta_e2 = F_tip * pL ** 3 / (3 * TAPE["e2"] * I_p)
        zero_card = laminate_card("tape [0]_8", [(TAPE, 0.0)] * n_ply)

        def lam_job(tag, card, axes=None, drape=None, forces=None):
            skin = {"material": card, "thickness": t_lam}
            job = {"nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
                   "regions": [{"id": 1, "label": "plate", "skin": skin, "rib": None}],
                   "supports": [{"axis": "x", "value": "min", "tolerance": 0.0, "bounds": None}],
                   "loads": {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None, "point_forces": forces or tip_forces},
                   "solver": {"echo_level": 0, "linear_solver": solver, "shell_element": "ShellThinElement3D3N", "modes": 0},
                   "outputs": {"vtk": os.path.join(lam_dir, tag + ".vtk"), "result": tag + "_result.json"}}
            if axes is not None:
                np.tile(np.asarray(axes, dtype="<f8"), (len(ptris), 1)).tofile(os.path.join(lam_dir, tag + "_axes.bin"))
                job["axes"] = tag + "_axes.bin"
            if drape is not None:
                skin["drape_file"] = tag + "_drape.bin"
                np.asarray(drape, dtype="<f4").tofile(os.path.join(lam_dir, tag + "_drape.bin"))
            return run(lam_dir, job)

        def tip_uz(tag):
            pts, cells, types, pd, cd = read_vtk(os.path.join(lam_dir, tag + ".vtk"))
            tip = np.abs(pts[:, 0] - pL) < 1e-9
            return float(pd["displacement"][tip, 2].mean()), pd, cd

        r1 = lam_job("lam_x", zero_card, axes=[1.0, 0.0, 0.0])
        uz1, pd1, cd1 = tip_uz("lam_x")
        failures += not check(r1["ok"] and r1["balanced"] and r1["laminates"] == 1 and abs(r1["reaction"][2] - F_tip) < 0.01 * F_tip, "laminate: the laminate plate solves and the clamp carries the tip load (%.4g N)" % r1["reaction"][2])
        failures += not check(abs(-uz1 / delta_e1 - 1.0) < 0.03, "laminate: fibres along the plate -- the tip deflection is the E1 beam's (%.4g vs %.4g m, %.1f%%)" % (-uz1, delta_e1, 100 * (-uz1 / delta_e1 - 1)))
        failures += not check(abs(r1["mass"] - TAPE["density"] * t_lam * pL * pB) < 1e-9 * r1["mass"], "laminate: the mass is the plies' rho t A (%.5g kg)" % r1["mass"])
        e1 = r1["per_material"][0]
        failures += not check(e1["criterion"] == "Tsai-Wu" and e1["reserve_factor"] > 1.0 and abs(e1["margin"] - (e1["reserve_factor"] - 1.0)) < 1e-12 and e1["plies"] == n_ply and abs(e1["thickness"] - t_lam) < 1e-12 and e1["stacks"] == 1 and e1["drape"] is None,
                              "laminate: per_material reports Tsai-Wu's reserve factor (%.3g), margin RF - 1, %d plies in one stack, no drape" % (e1["reserve_factor"], e1["plies"]))
        failures += not check("reserve_factor" in cd1 and cd1["reserve_factor"].min() > 0.0 and abs(cd1["reserve_factor"].min() / e1["reserve_factor"] - 1.0) < 1e-6 and "ply_shear_max" in cd1 and "fibre_deviation_max" in cd1 and cd1["ply_shear_max"].max() == 0.0,
                              "laminate: the VTK carries the reserve factor per cell (min %.3g) and the drape fields (zero, undraped)" % cd1["reserve_factor"].min())
        print("  laminate [0]_8 along x: tip %.4g m vs E1 beam %.4g; RF min %.3g; %d dof in %.1f s" % (-uz1, delta_e1, e1["reserve_factor"], r1["dofs"], r1["seconds_solve"]))

        r2 = lam_job("lam_y", zero_card, axes=[0.0, 1.0, 0.0])
        uz2, _, _ = tip_uz("lam_y")
        failures += not check(abs(-uz2 / delta_e2 - 1.0) < 0.03, "laminate: the material axis turned 90 degrees -- the tip deflection is the E2 beam's (%.4g vs %.4g m, %.1f%%)" % (-uz2, delta_e2, 100 * (-uz2 / delta_e2 - 1)))
        failures += not check(r2["per_material"][0]["reserve_factor"] < e1["reserve_factor"], "laminate: ...and the reserve factor is lower across the fibres (%.3g vs %.3g)" % (r2["per_material"][0]["reserve_factor"], e1["reserve_factor"]))

        r3 = lam_job("lam_auto", zero_card)
        uz3, _, _ = tip_uz("lam_auto")
        failures += not check(abs(uz3 / uz1 - 1.0) < 1e-6, "laminate: without an axes file the region's longest extent (x) is the reference direction (tip %.4g vs %.4g m)" % (-uz3, -uz1))

        # A drape turning every ply 90 degrees is the E2 plate again.
        drape90 = np.zeros((len(ptris), n_ply, 2))
        drape90[:, :, 0] = 90.0
        r4 = lam_job("lam_drape", zero_card, axes=[1.0, 0.0, 0.0], drape=drape90)
        uz4, _, cd4 = tip_uz("lam_drape")
        failures += not check(abs(uz4 / uz2 - 1.0) < 1e-6 and r4["per_material"][0]["drape"]["max_deviation"] == 90.0 and r4["per_material"][0]["stacks"] == 1 and cd4["fibre_deviation_max"].max() == 90.0,
                              "laminate: a drape turning the fibres 90 degrees gives the E2 plate (tip %.4g vs %.4g m), one stack, deviation 90 in the result and the VTK" % (-uz4, -uz2))
        # A woven ply sheared 60 degrees: the thickness and the mass double.
        cloth_card = laminate_card("cloth [0]_2", [(CLOTH, 0.0)] * 2)
        t_cloth = 2 * CLOTH["thickness"]
        drape60 = np.zeros((len(ptris), 2, 2))
        drape60[:, :, 1] = 60.0
        r5 = lam_job("cloth_plain", cloth_card, axes=[1.0, 0.0, 0.0])
        r6 = lam_job("cloth_sheared", cloth_card, axes=[1.0, 0.0, 0.0], drape=drape60)
        failures += not check(abs(r5["mass"] - CLOTH["density"] * t_cloth * pL * pB) < 1e-9 * r5["mass"] and abs(r6["mass"] / r5["mass"] - 2.0) < 1e-6,
                              "laminate: a cloth sheared 60 degrees is 1 / cos 60 = twice as thick and heavy (%.5g vs %.5g kg)" % (r6["mass"], r5["mass"]))
        failures += not check(r6["per_material"][0]["drape"]["faces_past_locking"] == 2 * len(ptris) and r6["per_material"][0]["drape"]["max_shear"] == 60.0,
                              "laminate: every face of both plies is past the cloth's 35 degree locking angle (%d)" % r6["per_material"][0]["drape"]["faces_past_locking"])
        uz6, _, _ = tip_uz("cloth_sheared")
        uz5, _, _ = tip_uz("cloth_plain")
        failures += not check(-uz6 < -uz5, "laminate: the thickened sheared cloth deflects less (%.4g vs %.4g m)" % (-uz6, -uz5))
        # Without strengths: no margin, and the criterion says so.
        bare = laminate_card("bare [0]_8", [(dict(TAPE, t1=0.0, c1=0.0, t2=0.0, c2=0.0, s12=0.0), 0.0)] * n_ply)
        r7 = lam_job("lam_bare", bare, axes=[1.0, 0.0, 0.0])
        e7 = r7["per_material"][0]
        failures += not check(r7["ok"] and e7["reserve_factor"] == 0.0 and e7["margin"] == 0.0 and e7["criterion"].startswith("Tsai-Wu (no strengths"), "laminate: a lamina without strengths reports no margin and says so")
        # An unsymmetric [0_4/90_4] strip under axial tension bends towards
        # its stiff outer plies, which lie on the +normal (first row) side.
        unsym = laminate_card("tape [0_4/90_4]", [(TAPE, 0.0)] * 4 + [(TAPE, 90.0)] * 4)
        axial = [{"at": [float(p[0]), float(p[1]), float(p[2])], "force": [100.0 / len(tip_nodes), 0.0, 0.0]} for p in tip_nodes]
        r8 = lam_job("lam_unsym", unsym, axes=[1.0, 0.0, 0.0], forces=axial)
        uz8, _, _ = tip_uz("lam_unsym")
        failures += not check(r8["ok"] and uz8 > 1e-4, "laminate: an unsymmetric strip in tension bends towards its stiff outer plies on the +normal side (tip z %.4g m)" % uz8)
        print("  laminate: E2 plate %.4g m (theory %.4g); drape-90 %.4g; cloth sheared 60: mass x%.3f; unsymmetric tip z %.4g m" % (-uz2, delta_e2, -uz4, r6["mass"] / r5["mass"], uz8))

        # ------------------------------------------------------------------
        # MASS ITEMS (2026-11-13): a 50 kg item hung by an RBE3-type
        # constraint on the six nearest nodes of the box beam's tip cap,
        # under 1 g with the beam clamped. The clamp carries the skin's
        # weight and the item's exactly; the item puts its weight on the
        # structure; the tip moves as under a point force of that weight
        # (to the few percent the six-node spread makes).
        # ------------------------------------------------------------------
        mass_dir = os.path.join(job_dir, "mass")
        os.makedirs(mass_dir)
        write_mesh(os.path.join(mass_dir, "nodes.bin"), bverts)
        write_faces(os.path.join(mass_dir, "faces.bin"), btris, np.ones(len(btris), dtype=np.int32))
        item_at = np.array([0.0, bL / 2 + 0.3, 0.2])
        attach = np.argsort(np.linalg.norm(bverts - np.array([0.0, bL / 2, 0.0]), axis=1))[:6]
        item_mass = 50.0
        job_m = {
            "nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
            "regions": [{"id": 1, "label": "beam", "skin": {"material": AL, "thickness": bt}, "rib": None}],
            "supports": [{"axis": "y", "value": "min", "tolerance": 0.0, "bounds": None}],
            "masses": [{"name": "payload", "mass": item_mass, "at": [float(v) for v in item_at], "inertia": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0], "attach": [int(v) for v in attach]}],
            "loads": {"load_factor": 1.0, "pressure": 0.0, "pressure_file": None, "point_forces": []},
            "solver": {"echo_level": 0, "linear_solver": solver, "shell_element": "ShellThinElement3D3N", "modes": 0},
            "outputs": {"vtk": os.path.join(mass_dir, "mass.vtk"), "result": "mass_result.json"},
        }
        rm = run(mass_dir, job_m)
        W_skin = AL["density"] * bt * tg.surface_area(bverts, btris.astype(np.int32)) * sf.G0
        W_item = item_mass * sf.G0
        failures += not check(rm["ok"] and rm["balanced"] and rm["support_mode"] == "clamped" and abs(rm["reaction"][2] - (W_skin + W_item)) < 1e-6 * (W_skin + W_item)
                              and abs(rm["reaction"][0]) < 1e-6 * W_item and abs(rm["reaction"][1]) < 1e-6 * W_item,
                              "mass item: the clamp carries the skin's weight and the item's exactly (%.6g vs %.6g N)" % (rm["reaction"][2], W_skin + W_item))
        failures += not check(abs(rm["mass"] - rm["mass_structure"] - item_mass) < 1e-9 * rm["mass"] and rm["mass_items"] == item_mass and rm["elements"] == len(btris) + 1 and rm["dofs"] == 6 * (len(bverts) + 1),
                              "mass item: the mass is the structure's plus the item's, one more element and node (%.5g kg, %d elements, %d dof)" % (rm["mass"], rm["elements"], rm["dofs"]))
        it = rm["items"][0]
        failures += not check(len(rm["items"]) == 1 and it["name"] == "payload" and it["attach_nodes"] == 6 and it["node"] == len(bverts)
                              and abs(it["force"][2] + W_item) < 1e-9 * W_item and abs(it["force"][0]) < 1e-9 * W_item,
                              "mass item: the item reports its six attachment nodes and puts its weight on them (%.5g N)" % it["force"][2])
        cg_expected = (rm["mass_structure"] * np.array([0.0, 0.0, 0.0]) + item_mass * item_at) / rm["mass"]
        failures += not check(np.linalg.norm(np.array(rm["mass_centre"]) - cg_expected) < 1e-6 * bL, "mass item: the mass centre moves towards the item (%s)" % np.round(rm["mass_centre"], 4))
        # The same weight as a point force at the item's attachment centre,
        # with the skin's own weight too.
        job_f = dict(job_m, masses=[], loads={"load_factor": 1.0, "pressure": 0.0, "pressure_file": None,
                                              "point_forces": [{"at": [float(v) for v in it["attach_centre"]], "force": [0.0, 0.0, -W_item]}]},
                     outputs={"vtk": os.path.join(mass_dir, "mass_force.vtk"), "result": "mass_force_result.json"})
        rf = run(mass_dir, job_f)
        um, _, _, pdm, _ = read_vtk(os.path.join(mass_dir, "mass.vtk"))
        uf, _, _, pdf, _ = read_vtk(os.path.join(mass_dir, "mass_force.vtk"))
        tip_m = float(pdm["displacement"][tip_b, 2].mean())
        tip_f = float(pdf["displacement"][tip_b, 2].mean())
        failures += not check(abs(tip_m / tip_f - 1.0) < 0.05 and tip_m < 0.0, "mass item: the tip moves as under a point force of the item's weight (%.4g vs %.4g m, %.1f%%)" % (tip_m, tip_f, 100 * (tip_m / tip_f - 1)))
        failures += not check(abs(it["displacement"][2] - tip_m) < 0.2 * abs(tip_m), "mass item: the item moves with the tip (%.4g vs %.4g m)" % (it["displacement"][2], tip_m))
        print("  mass item: reaction %.6g N (weight %.6g), tip %.4g m vs point force %.4g" % (rm["reaction"][2], W_skin + W_item, tip_m, tip_f))
        # An attachment on a line cannot carry a moment about it: refused.
        line = [int(v) for v in np.nonzero((np.abs(bverts[:, 0] - 0.5) < 1e-9) & (np.abs(bverts[:, 2] - 0.5) < 1e-9))[0][:4]]
        job_l = dict(job_m, masses=[dict(job_m["masses"][0], attach=line)], outputs={"vtk": os.path.join(mass_dir, "line.vtk"), "result": "line_result.json"})
        refused = ""
        try:
            run(mass_dir, job_l)
        except sf.JobError as exc:
            refused = str(exc)
        failures += not check("line" in refused, "mass item: an attachment on a line is refused (%s)" % refused[:70])

        # ------------------------------------------------------------------
        # FREE FLIGHT, inertia relief (2026-11-13): a 2 x 0.5 m aluminium
        # plate, 2 mm, with a 2 kg item hung 0.1 m above it off its centre,
        # under 1 g and a uniform lift equal to the total weight. The net
        # force is zero and the net moment is not, so the free body pitches
        # (alpha = I_cg^-1 M), the inertial field balances it, and the
        # reference node's reaction force and moment vanish to round-off;
        # the stresses do not depend on which node is the reference.
        # Gravity alone loads nothing; lift of twice the weight is the same
        # case as a 2 g load factor under that lift.
        # ------------------------------------------------------------------
        ff_dir = os.path.join(job_dir, "freeflight")
        os.makedirs(ff_dir)
        plx, ply, pnx, pny = 2.0, 0.5, 20, 5
        pverts, ptris = plate(plx, ply, pnx, pny)
        write_mesh(os.path.join(ff_dir, "nodes.bin"), pverts)
        write_faces(os.path.join(ff_dir, "faces.bin"), ptris, np.ones(len(ptris), dtype=np.int32))
        pt = 0.002
        pm_at = np.array([0.5, 0.25, 0.1])
        pm_mass = 2.0
        pm_attach = [int(v) for v in np.argsort(np.linalg.norm(pverts - pm_at, axis=1))[:4]]
        m_plate = AL["density"] * pt * plx * ply
        m_total = m_plate + pm_mass
        lift = -(m_total * sf.G0) / (plx * ply)   # a negative pressure pulls the upward-facing plate up

        def ff_job(name, pressure, load_factor, reference=None, relief=True, modes=0, supports=None):
            return {
                "nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
                "regions": [{"id": 1, "label": "plate", "skin": {"material": AL, "thickness": pt}, "rib": None}],
                "supports": supports or [],
                "inertia_relief": ({"reference": reference} if relief else None),
                "masses": [{"name": "pod", "mass": pm_mass, "at": [float(v) for v in pm_at], "inertia": None, "attach": pm_attach}],
                "loads": {"load_factor": load_factor, "pressure": pressure, "pressure_file": None, "point_forces": []},
                "solver": {"echo_level": 0, "linear_solver": solver, "shell_element": "ShellThinElement3D3N", "modes": modes},
                "outputs": {"vtk": os.path.join(ff_dir, name + ".vtk"), "result": name + "_result.json"},
            }

        r1 = run(ff_dir, ff_job("trim", lift, 1.0, modes=3))
        scale1 = r1["load_scale"]
        failures += not check(r1["ok"] and r1["support_mode"] == "free flight" and r1["support_nodes"] == 1 and 0 <= r1["support_node"] < len(pverts),
                              "free flight: one reference node holds the model (node %d at %s)" % (r1["support_node"], np.round(r1["support_at"], 3)))
        failures += not check(r1["balanced"] and np.linalg.norm(r1["reaction"]) < 1e-9 * scale1 and np.linalg.norm(r1["reaction_moment"]) < 1e-9 * scale1 * plx
                              and r1["balance_error"] < 1e-9 and r1["moment_error"] < 1e-9,
                              "free flight: the reference node reacts nothing -- force %.2e N, moment %.2e Nm against loads of %.4g N" % (np.linalg.norm(r1["reaction"]), np.linalg.norm(r1["reaction_moment"]), scale1))
        failures += not check(np.linalg.norm(r1["rigid_acceleration"]) < 1e-9 * sf.G0 and abs(r1["load_factor_at_cg"] - 1.0) < 1e-9,
                              "free flight: lift equal to the weight -- no rigid acceleration, load factor 1 at the mass centre (%.3g m/s^2, n = %.6f)" % (np.linalg.norm(r1["rigid_acceleration"]), r1["load_factor_at_cg"]))
        # The pitch acceleration: the lift acts at the plate's centre, the
        # weight at the mass centre; alpha = I_cg^-1 (r_plate - cg) x L.
        cg1 = np.array(r1["mass_centre"])
        M1 = np.cross(np.array([plx / 2, ply / 2, 0.0]) - cg1, np.array([0.0, 0.0, m_total * sf.G0]))
        alpha1 = np.linalg.solve(np.array(r1["inertia_cg"]), M1)
        failures += not check(np.linalg.norm(np.array(r1["rigid_angular_acceleration"]) - alpha1) < 1e-9 * np.linalg.norm(alpha1) and abs(alpha1[1]) > 1.0,
                              "free flight: the offset mass makes the body pitch, alpha = I_cg^-1 M (%s rad/s^2)" % np.round(r1["rigid_angular_acceleration"], 4))
        failures += not check(abs(r1["mass"] - m_total) < 1e-9 * m_total and r1["max_von_mises"] > 1e5 and r1["max_displacement"] > 1e-4,
                              "free flight: the plate bends under the lift against its mass (%.4g Pa, %.4g m)" % (r1["max_von_mises"], r1["max_displacement"]))
        # The same case from a corner of the plate: the same stresses.
        r2 = run(ff_dir, ff_job("trim_corner", lift, 1.0, reference=[0.0, 0.0, 0.0]))
        _, _, _, _, cd1 = read_vtk(os.path.join(ff_dir, "trim.vtk"))
        _, _, _, _, cd2 = read_vtk(os.path.join(ff_dir, "trim_corner.vtk"))
        dvm = float(np.abs(cd1["von_mises_element"] - cd2["von_mises_element"]).max() / cd1["von_mises_element"].max())
        failures += not check(r2["ok"] and r2["support_node"] == 0 and r2["support_node"] != r1["support_node"] and dvm < 1e-8 and np.linalg.norm(r2["reaction"]) < 1e-9 * scale1,
                              "free flight: the stresses do not depend on the reference node (node %d vs %d, max relative difference %.1e)" % (r2["support_node"], r1["support_node"], dvm))
        # Gravity alone: the body falls freely, nothing is loaded.
        r3 = run(ff_dir, ff_job("fall", 0.0, 1.0))
        failures += not check(r3["ok"] and abs(r3["rigid_acceleration"][2] + sf.G0) < 1e-9 * sf.G0 and abs(r3["load_factor_at_cg"]) < 1e-9
                              and r3["max_von_mises"] < 1e-6 * r1["max_von_mises"] and r3["max_displacement"] < 1e-9 * r1["max_displacement"],
                              "free flight: gravity alone loads nothing -- the body falls at g (a0 %.4g m/s^2, n = %.2g, von Mises %.2e Pa)" % (r3["rigid_acceleration"][2], r3["load_factor_at_cg"], r3["max_von_mises"]))
        # Lift of twice the weight under 1 g: the body accelerates up at g
        # and feels 2 g; the same stresses as the 2 g trimmed case.
        r4 = run(ff_dir, ff_job("pull_up", 2.0 * lift, 1.0))
        r5 = run(ff_dir, ff_job("trim_2g", 2.0 * lift, 2.0))
        _, _, _, _, cd4 = read_vtk(os.path.join(ff_dir, "pull_up.vtk"))
        _, _, _, _, cd5 = read_vtk(os.path.join(ff_dir, "trim_2g.vtk"))
        dvm45 = float(np.abs(cd4["von_mises_element"] - cd5["von_mises_element"]).max() / cd5["von_mises_element"].max())
        failures += not check(r4["ok"] and r5["ok"] and abs(r4["rigid_acceleration"][2] - sf.G0) < 1e-9 * sf.G0 and abs(r4["load_factor_at_cg"] - 2.0) < 1e-9
                              and abs(r5["load_factor_at_cg"] - 2.0) < 1e-9 and np.linalg.norm(r5["rigid_acceleration"]) < 1e-9 * sf.G0 and dvm45 < 1e-8,
                              "free flight: lift of twice the weight = a pull-up at 2 g, the same stresses as the trimmed 2 g case (a0 %.4g m/s^2, n = %.4g, difference %.1e)" % (r4["rigid_acceleration"][2], r4["load_factor_at_cg"], dvm45))
        # The free-free modes: the six rigid modes dropped, the first
        # flexible mode of the plate (a free-free beam's first bending is
        # 22.37 / (2 pi L^2) sqrt(EI / rho A), lowered a little by the pod).
        f_ff = r1["natural_frequencies"]
        I_p = ply * pt ** 3 / 12
        f1_free = 22.373 / (2 * np.pi * plx ** 2) * np.sqrt(AL["youngs_modulus"] * I_p / (AL["density"] * ply * pt))
        failures += not check(len(f_ff) == 3 and f_ff[0] > 0.5 * f1_free and f_ff[0] < 1.2 * f1_free and f_ff[1] >= f_ff[0] and f_ff[2] >= f_ff[1],
                              "free flight: three free-free modes with the rigid ones dropped, the first near the free-free beam's %.4g Hz (%s)" % (f1_free, ", ".join("%.4g" % f for f in f_ff) if f_ff else "none: %s" % "; ".join(r1["warnings"])))
        print("  free flight: reaction %.2e N / %.2e Nm, alpha %s, von Mises %.4g Pa, modes %s Hz (%s)" % (np.linalg.norm(r1["reaction"]), np.linalg.norm(r1["reaction_moment"]), np.round(r1["rigid_angular_acceleration"], 3), r1["max_von_mises"], ", ".join("%.4g" % f for f in f_ff), r1["eigensolver"]))
        # Planes and free flight together are refused; neither is refused too.
        refused = ""
        try:
            run(ff_dir, ff_job("both", lift, 1.0, supports=[{"axis": "x", "value": "min", "tolerance": 0.0, "bounds": None}]))
        except sf.JobError as exc:
            refused = str(exc)
        failures += not check("free flight" in refused, "free flight: support planes and free flight together are refused")
        refused = ""
        try:
            run(ff_dir, ff_job("neither", lift, 1.0, relief=False))
        except sf.JobError as exc:
            refused = str(exc)
        failures += not check("no support" in refused, "free flight: no support at all is refused")

        # ------------------------------------------------------------------
        # HINGES AND PUSHRODS (2026-11-14): a stiff 1 x 0.5 m plate (20 mm)
        # clamped at x = 0 and, 20 mm aft of its trailing edge, a 2 mm flap
        # plate 0.48 m long, hinged to it by two bracket ties at the flap's
        # leading-edge corners (translations follow the main plate's
        # trailing-edge corners; the flap turns about the line through them)
        # and held by a 4 mm steel pushrod from a horn 50 mm below the hinge
        # line at mid-span to a servo arm 50 mm below the main plate 0.22 m
        # ahead. Under 1 g the ties take no moment about the hinge line, so
        # the rod alone holds the flap's weight moment: F x 0.05 = W_flap x
        # 0.24 (the flap's centroid is 0.24 m aft of the hinge), in
        # compression (the trailing edge drops, the horn swings forward).
        # Without the rod the flap is a mechanism and the solve is not
        # balanced.
        # ------------------------------------------------------------------
        hg_dir = os.path.join(job_dir, "hinge")
        os.makedirs(hg_dir)
        mverts, mtris = plate(1.0, 0.5, 10, 4)
        fverts, ftris = plate(0.48, 0.5, 5, 4)
        fverts = fverts + np.array([1.02, 0.0, 0.0])
        n_main = len(mverts)
        hverts = np.vstack([mverts, fverts])
        htris = np.vstack([mtris, ftris + n_main])
        hregion = np.concatenate([np.ones(len(mtris), dtype=np.int32), 2 * np.ones(len(ftris), dtype=np.int32)])
        write_mesh(os.path.join(hg_dir, "nodes.bin"), hverts)
        write_faces(os.path.join(hg_dir, "faces.bin"), htris, hregion)
        main_te = [10, 4 * 11 + 10]            # (1.0, 0) and (1.0, 0.5)
        flap_le = [n_main + 0, n_main + 24]    # (1.02, 0) and (1.02, 0.5)
        flap_le_mid = n_main + 12              # (1.02, 0.25)
        main_servo = 2 * 11 + 8                # (0.8, 0.25)
        assert np.allclose(hverts[main_te], [[1.0, 0.0, 0.0], [1.0, 0.5, 0.0]])
        assert np.allclose(hverts[flap_le], [[1.02, 0.0, 0.0], [1.02, 0.5, 0.0]])
        assert np.allclose(hverts[flap_le_mid], [1.02, 0.25, 0.0]) and np.allclose(hverts[main_servo], [0.8, 0.25, 0.0])
        t_main, t_flap = 0.02, 0.002
        STEEL = {"name": "4130 steel", "density": 7850.0, "youngs_modulus": 205e9, "poisson_ratio": 0.29}
        rod_area = 0.25 * np.pi * 0.004 ** 2
        ties = [{"master": main_te[0], "slave": flap_le[0]}, {"master": main_te[1], "slave": flap_le[1]}]
        links = [{"name": "horn", "on": flap_le_mid, "at": [1.02, 0.25, -0.05]}, {"name": "servo arm", "on": main_servo, "at": [0.8, 0.25, -0.05]}]
        bars = [{"name": "pushrod", "a": {"link": 0}, "b": {"link": 1}, "area": rod_area, "youngs_modulus": STEEL["youngs_modulus"], "density": STEEL["density"]}]

        def hinge_job(name, with_rod=True, with_ties=True):
            return {
                "nodes": "nodes.bin", "faces": "faces.bin", "rib_faces": None,
                "regions": [{"id": 1, "label": "main plate", "skin": {"material": AL, "thickness": t_main}, "rib": None},
                            {"id": 2, "label": "flap", "skin": {"material": AL, "thickness": t_flap}, "rib": None}],
                "supports": [{"axis": "x", "value": "min", "tolerance": 0.0, "bounds": None}],
                "inertia_relief": None, "masses": [],
                "ties": ties if with_ties else [], "links": links if with_rod else [], "bars": bars if with_rod else [],
                "loads": {"load_factor": 1.0, "pressure": 0.0, "pressure_file": None, "point_forces": []},
                "solver": {"echo_level": 0, "linear_solver": solver, "shell_element": "ShellThinElement3D3N", "modes": 0},
                "outputs": {"vtk": os.path.join(hg_dir, name + ".vtk"), "result": name + "_result.json"},
            }

        rh = run(hg_dir, hinge_job("hinged"))
        m_main = AL["density"] * t_main * 1.0 * 0.5
        m_flap = AL["density"] * t_flap * 0.48 * 0.5
        m_rod = STEEL["density"] * rod_area * 0.22
        w_all = (m_main + m_flap + m_rod) * sf.G0
        failures += not check(rh["ok"] and rh["ties"] == 2 and rh["links"] == 2 and len(rh["bars"]) == 1 and rh["dofs"] == 6 * (len(hverts) + 2),
                              "hinge: two ties, two offset links and one bar built; the links are nodes of freedom (%d dof)" % rh["dofs"])
        failures += not check(abs(rh["mass"] - (m_main + m_flap + m_rod)) < 1e-9 * rh["mass"] and abs(rh["mass_bars"] - m_rod) < 1e-12,
                              "hinge: the mass counts the rod, rho A L = %.5g kg (total %.5g)" % (m_rod, rh["mass"]))
        failures += not check(rh["balanced"] and abs(rh["reaction"][2] - w_all) < 1e-6 * w_all and abs(rh["reaction"][0]) < 1e-6 * w_all,
                              "hinge: the clamp carries the main plate, the flap and the rod (%.6g vs %.6g N)" % (rh["reaction"][2], w_all))
        rod = rh["bars"][0]
        f_expect = -(m_flap * sf.G0 * 0.24) / 0.05
        failures += not check(rod["force"] < 0.0 and abs(rod["force"] / f_expect - 1.0) < 0.02,
                              "hinge: the pushrod alone holds the flap's weight moment, in compression (%.4g N vs W d / h = %.4g N)" % (rod["force"], f_expect))
        failures += not check(abs(rod["stress"] - rod["force"] / rod_area) < 1e-6 * abs(rod["stress"]) and abs(rod["length"] - 0.22) < 1e-12 and abs(rod["mass"] - m_rod) < 1e-12,
                              "hinge: the rod's stress, length and mass are reported (%.4g Pa, %.4g m, %.4g kg)" % (rod["stress"], rod["length"], rod["mass"]))
        # The ties hold: the flap's leading-edge corners move exactly with
        # the main plate's trailing-edge corners. The flap's trailing edge
        # drops far more than the stiff main plate's edge -- mostly the 2 mm
        # flap plate's own bending from a hinge line held at two corners and
        # a horn at one node (the rod's 5 micron shortening at the lever
        # ratio 9.6 is a small part of it) -- and stays finite.
        _, _, _, pdh, _ = read_vtk(os.path.join(hg_dir, "hinged.vtk"))
        u = pdh["displacement"]
        tie_slip = float(np.abs(u[flap_le] - u[main_te]).max())
        flap_te = [n_main + j * 6 + 5 for j in range(5)]
        drop_te = float(-u[flap_te, 2].mean())
        droop_main = float(-u[main_te, 2].mean())
        shorten = abs(rod["force"]) * 0.22 / (STEEL["youngs_modulus"] * rod_area)
        failures += not check(tie_slip < 1e-12 * max(drop_te, 1e-12), "hinge: the tied corners of the flap move exactly with the main plate's (slip %.2e m)" % tie_slip)
        failures += not check(drop_te > 3.0 * droop_main and drop_te < 0.1 and drop_te > shorten * 0.48 / 0.05,
                              "hinge: the flap's trailing edge drops %.4g m (the rod's %.3g m shortening at the lever ratio 9.6 plus the flap plate's own bending) against the main plate's %.4g" % (drop_te, shorten, droop_main))
        print("  hinge: rod %.4g N (expected %.4g), reaction %.6g N, flap TE drop %.4g m" % (rod["force"], f_expect, rh["reaction"][2], drop_te))
        # Without the rod the flap is free to turn: a mechanism, which the
        # solver either refuses or returns unbalanced.
        mech = ""
        try:
            rn = run(hg_dir, hinge_job("no_rod", with_rod=False))
            mech = "unbalanced" if not rn["balanced"] or rn["max_displacement"] > 1.0 else "balanced?!"
        except Exception as exc:  # noqa: BLE001 -- a singular system can surface as any error
            mech = "refused: %s" % str(exc)[:60]
        failures += not check(mech != "balanced?!", "hinge: without the pushrod the flap is a mechanism and the solve is %s" % mech)
        # A tie naming a node the mesh lacks is refused.
        refused = ""
        try:
            bad = hinge_job("bad_tie")
            bad["ties"] = [{"master": 0, "slave": 10 ** 6}]
            run(hg_dir, bad)
        except sf.JobError as exc:
            refused = str(exc)
        failures += not check("hinge tie" in refused, "hinge: a tie on a node the mesh does not have is refused")

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
