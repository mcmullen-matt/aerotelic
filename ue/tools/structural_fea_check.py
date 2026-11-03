"""Self-check of structural_fea.py, outside the application.

    python structural_fea_check.py [--keep DIR] [--no-solid] [--solver auto|amgcl|sparse_lu]

Cases with closed forms, each run through the real job (the same
``run_job`` the application calls, through the same job.json):

  * a SANDWICH CANTILEVER: a rectangular box L x b x H clamped at x = 0
    under its own weight, aluminium shells of thickness t on every face
    over a soft isotropic core (a fully solid "cell" of a soft material,
    which the homogeniser returns exactly). Timoshenko gives the tip
    deflection q L^4 / (8 EI) + q L^2 / (2 GA) with EI = E_s (b t H^2 / 2
    + t H^3 / 6) + E_c b H^3 / 12 (the flanges at +-H/2, the two webs, the
    core filling the box as the model has it) and GA = G_s 2 t H + G_c b H,
    and the root bending stress M c / I with M = q L^2 / 2, c = H / 2.
    Shells and solids share nodes here, so this is also the test that
    Kratos takes six degrees of freedom on the shell nodes and three on the
    rest, that the shells carry VOLUME_ACCELERATION (the reactions balance
    the whole weight), and that the clamp holds the rotations.
  * the SAME BOX as skin / core tetrahedra (the ``solid`` model, 10-node)
    deflects the same, which ties the shell model to the full-resolution
    one it replaces (--no-solid skips it: it is the slow one).
  * the box over a GRID LATTICE core (three orthogonal bars, homogenised
    here): the core's micro stress is at least its macro stress and within
    the amplification bound, and the margins come out against the base
    material's allowable.
  * a PRESSURISED SPHERE: an aluminium shell of thickness t on a unit
    sphere over a core too soft to matter, under 1 bar outside. The
    membrane stress is p a / (2 t) in every direction (von Mises the same),
    the radial displacement -p a^2 (1 - nu) / (2 E t), and the pressure's
    resultant over the closed surface is zero.
"""

import json
import os
import shutil
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tet_gmsh as tg  # noqa: E402  (bootstraps the DLL directory)
import structural_fea as sf  # noqa: E402
import homogenize as hz  # noqa: E402
import potential_flow_check as pfc  # noqa: E402  (icosphere, write_tri)

import numpy as np  # noqa: E402
import gmsh  # noqa: E402

AL = {"name": "Aluminium 2024-T3", "density": 2780.0, "youngs_modulus": 73.1e9, "poisson_ratio": 0.33, "allowable_stress": 345.0e6}


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def write_mesh(path, nodes, tets, region):
    with open(path, "wb") as f:
        f.write(struct.pack("<II", len(nodes), len(tets)))
        f.write(np.ascontiguousarray(nodes, dtype="<f8").tobytes())
        f.write(np.ascontiguousarray(tets, dtype="<i4").tobytes())
        f.write(np.ascontiguousarray(region, dtype="<i4").tobytes())


def write_cell(job_dir, name, solid, period):
    """A voxel cell for a region's core: True = solid, in (i, j, k)."""
    nx, ny, nz = solid.shape
    values = np.where(np.transpose(solid, (2, 1, 0)), -1.0, 1.0).astype("<f4")
    values.tofile(os.path.join(job_dir, name))
    return {"file": name, "nx": nx, "ny": ny, "nz": nz, "period": list(period)}


def box_tets(L, b, H, size, skin=0.0):
    """Tetrahedra of the box [0, L] x [-b/2, b/2] x [-H/2, H/2] from gmsh's
    OpenCASCADE kernel; with `skin` > 0 the box is two regions, the shell
    of that thickness (1) and the core inside it (2)."""
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("box")
        outer = gmsh.model.occ.addBox(0.0, -b / 2, -H / 2, L, b, H)
        inner = None
        if skin > 0.0:
            inner_tag = gmsh.model.occ.addBox(skin, -b / 2 + skin, -H / 2 + skin, L - 2 * skin, b - 2 * skin, H - 2 * skin)
            out, _ = gmsh.model.occ.fragment([(3, outer)], [(3, inner_tag)])
            gmsh.model.occ.synchronize()
            volumes = [t for d, t in out if d == 3]
            # The inner box is the volume whose mass is the inner volume.
            inner_vol = (L - 2 * skin) * (b - 2 * skin) * (H - 2 * skin)
            inner = min(volumes, key=lambda t: abs(gmsh.model.occ.getMass(3, t) - inner_vol))
        else:
            gmsh.model.occ.synchronize()
            volumes = [outer]
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.5 * size)
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        gmsh.option.setNumber("Mesh.Optimize", 1)
        gmsh.model.mesh.generate(3)
        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        node_tags = np.array(node_tags, dtype=np.int64)
        nodes = np.array(coords, dtype=np.float64).reshape(-1, 3)
        lookup = np.full(int(node_tags.max()) + 1, -1, dtype=np.int64)
        lookup[node_tags] = np.arange(len(node_tags))
        tets, region = [], []
        for vol in volumes:
            types, _, conn = gmsh.model.mesh.getElements(3, vol)
            for etype, nodes_flat in zip(types, conn):
                if int(etype) != tg.GMSH_TET:
                    continue
                t = lookup[np.array(nodes_flat, dtype=np.int64).reshape(-1, 4)]
                tets.append(t)
                region.append(np.full(len(t), 2 if vol == inner else 1, dtype=np.int32))
        tets = np.concatenate(tets).astype(np.int32)
        region = np.concatenate(region)
    finally:
        gmsh.finalize()
    return nodes, tets, region


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


def sphere_tets(job_dir, radius, subdivisions, core_size):
    """An icosphere's interior meshed by tet_gmsh.py (the body's own
    triangles stay the boundary)."""
    verts, faces = pfc.icosphere(radius, subdivisions)
    pfc.write_tri(os.path.join(job_dir, "outer.tri"), verts, faces)
    job = {"outer": "outer.tri", "inner": "", "core_size": core_size, "algorithm": 1, "optimize": True, "optimize_netgen": False,
           "outputs": {"mesh": "sphere.tet", "result": "tet_result.json"}}
    with open(os.path.join(job_dir, "job.json"), "w", encoding="utf-8") as f:
        json.dump(job, f)
    tg.run_job(job_dir)
    with open(os.path.join(job_dir, "sphere.tet"), "rb") as f:
        assert f.read(8) == tg.TET_MAGIC
        nn, nt = struct.unpack("<II", f.read(8))
        nodes = np.frombuffer(f.read(nn * 24), dtype="<f8").reshape(nn, 3)
        tets = np.frombuffer(f.read(nt * 16), dtype="<i4").reshape(nt, 4)
        region = np.frombuffer(f.read(nt * 4), dtype="<i4")
    return np.array(nodes), np.array(tets), np.ones(len(region), dtype=np.int32), len(faces)


def read_vtk(path):
    """Points, the cell blocks, point data and cell data of the legacy VTK
    structural_fea.py writes (mixed triangles and tetrahedra)."""
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
    keep, do_solid, solver = None, True, "auto"
    args = list(argv[1:])
    while args:
        a = args.pop(0)
        if a == "--keep":
            keep = args.pop(0)
        elif a == "--no-solid":
            do_solid = False
        elif a == "--solver":
            solver = args.pop(0)
    failures = 0
    job_dir = tempfile.mkdtemp(prefix="aerotelic_fea_check_")
    try:
        # ------------------------------------------------------------------
        # The sandwich cantilever.
        # ------------------------------------------------------------------
        L, b, H, t = 1.0, 0.1, 0.1, 0.004
        soft = {"name": "soft", "density": 100.0, "youngs_modulus": 0.01 * AL["youngs_modulus"], "poisson_ratio": 0.3, "allowable_stress": 1.0e6}
        nodes, tets, region = box_tets(L, b, H, size=0.025)
        write_mesh(os.path.join(job_dir, "box.bin"), nodes, tets, region)
        solid_cell = write_cell(job_dir, "solid.f32", np.ones((4, 4, 4), dtype=bool), (1.0, 1.0, 1.0))
        vtk = os.path.join(job_dir, "box.vtk")
        job = {
            "mesh": "box.bin", "model": "shell_core",
            "regions": [{"id": 1, "label": "box", "skin": {"material": AL, "thickness": t},
                         "core": {"material": soft, "cell": solid_cell}}],
            "support": {"axis": "x", "value": "min", "tolerance": 1e-6},
            "loads": {"load_factor": 1.0, "pressure": 0.0, "pressure_file": None, "point_force": None},
            "solver": {"echo_level": 0, "linear_solver": solver, "homogenize_cache": False},
            "outputs": {"vtk": vtk, "result": "box_result.json"},
        }
        r = run(job_dir, job)
        failures += not check(r["ok"] and r["model"] == "shell_core", "box: the shell / core job ran")
        # "auto" is the direct solver at this size; the result says which
        # ran and that the reactions balance the loads (2026-11-01: amgcl
        # stopping short on a shell-only wing looked like a stiff, lightly
        # stressed structure; the balance is the only tell).
        expected_solver = "sparse_lu" if solver in ("auto", "sparse_lu") else solver
        failures += not check(r["linear_solver"] == expected_solver, "box: the solver used is %s (asked for %s)" % (r["linear_solver"], solver))
        failures += not check(r["balanced"] and r["balance_error"] <= sf.BALANCE_TOLERANCE and r["warnings"] == [],
                              "box: the reactions balance the loads (error %.2e of %.4g N) with no warning" % (r["balance_error"], r["load_scale"]))
        n_shell_nodes = len(np.unique(sf.boundary_faces(sf.orient(nodes, tets.astype(np.int64))[0])[0]))
        failures += not check(r["dofs"] == 3 * len(nodes) + 3 * n_shell_nodes,
                              "box: six degrees of freedom on the %d shell nodes, three on the other %d (%d dof)" % (n_shell_nodes, len(nodes) - n_shell_nodes, r["dofs"]))
        failures += not check(r["shells"] > 0 and r["tets"] == len(tets), "box: %d shells on the wetted faces over %d tetrahedra" % (r["shells"], r["tets"]))
        reg = r["regions"][0]
        failures += not check(abs(np.array(reg["E"]) / soft["youngs_modulus"] - 1.0).max() < 1e-9, "box: a solid cell homogenises to the base material exactly")
        # Mass: the shells' rho t A plus the core's corrected density over the volume.
        area = 2 * (L * b + L * H + b * H)
        volume = L * b * H
        expected_mass = AL["density"] * t * area + soft["density"] * (1.0 - area * t / volume) * volume
        failures += not check(abs(r["mass"] - expected_mass) < 1e-3 * expected_mass, "box: mass is the skin's rho t A plus the overlap-corrected core (%.5g vs %.5g kg)" % (r["mass"], expected_mass))
        weight = r["mass"] * sf.G0
        failures += not check(abs(r["reaction"][2] - weight) < 0.01 * weight and abs(r["reaction"][0]) < 0.01 * weight and abs(r["reaction"][1]) < 0.01 * weight,
                              "box: the clamp carries the whole weight, shells included (%.4g vs %.4g N)" % (r["reaction"][2], weight))
        # Timoshenko.
        E_s, nu_s = AL["youngs_modulus"], AL["poisson_ratio"]
        E_c, nu_c = soft["youngs_modulus"], soft["poisson_ratio"]
        G_s, G_c = E_s / (2 * (1 + nu_s)), E_c / (2 * (1 + nu_c))
        EI = E_s * (b * t * H ** 2 / 2 + t * H ** 3 / 6) + E_c * b * H ** 3 / 12
        GA = G_s * 2 * t * H + G_c * b * H
        q = weight / L
        delta = q * L ** 4 / (8 * EI) + q * L ** 2 / (2 * GA)
        failures += not check(abs(r["max_displacement"] / delta - 1.0) < 0.10,
                              "box: the tip deflects as Timoshenko's sandwich beam (%.4g vs %.4g m, %.1f%%)" % (r["max_displacement"], delta, 100 * (r["max_displacement"] / delta - 1)))
        failures += not check(r["max_displacement_at"][0] > 0.9 * L, "box: the largest displacement is at the tip (x = %.3f)" % r["max_displacement_at"][0])
        sigma_root = (q * L ** 2 / 2) * (H / 2) / EI * E_s
        peak_skin = reg["skin_max_von_mises"]
        failures += not check(0.8 * sigma_root < peak_skin < 2.5 * sigma_root,
                              "box: the skin's peak von Mises is the root bending stress M c / I, %.4g Pa, up to the clamp's concentration (got %.4g)" % (sigma_root, peak_skin))
        failures += not check(reg["core_max_micro_von_mises"] >= reg["core_max_von_mises"] * 0.999, "box: the core's micro stress is at least its macro stress")
        failures += not check(reg["skin_margin"] > 10.0, "box: aluminium under its own weight has a large margin (%.1f)" % reg["skin_margin"])
        print("  box: %d dof solved in %.1f s (homogenise %.1f s); tip %.4g m vs %.4g; root skin %.4g Pa vs %.4g"
              % (r["dofs"], r["seconds_solve"], r["seconds_homogenize"], r["max_displacement"], delta, peak_skin, sigma_root))
        shell_delta = r["max_displacement"]

        # The VTK: mixed cells, the fields the viewer needs.
        points, cells, cell_types, pdata, cdata = read_vtk(vtk)
        failures += not check(len(points) == len(nodes) and (cell_types == 10).sum() == len(tets) and (cell_types == 5).sum() == r["shells"],
                              "box: the VTK holds %d tetrahedra and %d triangles" % (len(tets), r["shells"]))
        failures += not check(all(k in pdata for k in ("displacement", "rotation", "von_mises", "micro_von_mises")), "box: the VTK holds displacement, rotation and both stresses per node")
        failures += not check(all(k in cdata for k in ("material", "kind", "von_mises_element", "micro_von_mises_element")), "box: the VTK holds the region, kind and stresses per cell")
        failures += not check(np.isfinite(pdata["displacement"]).all() and np.isfinite(cdata["von_mises_element"]).all(), "box: the solution is finite")

        # ------------------------------------------------------------------
        # The same box as skin / core tetrahedra: the full-resolution model.
        # ------------------------------------------------------------------
        if do_solid:
            nodes2, tets2, region2 = box_tets(L, b, H, size=0.012, skin=t * 2.5)
            # The skin is 2.5 t thick in tetrahedra to keep the count sane
            # at this check's size; Timoshenko with that thickness is the
            # comparison, since it changes EI.
            ts = t * 2.5
            write_mesh(os.path.join(job_dir, "box_solid.bin"), nodes2, tets2, region2)
            job2 = {
                "mesh": "box_solid.bin", "model": "solid", "second_order": True,
                "materials": [dict(AL, id=1, label="skin"), dict(soft, id=2, label="core")],
                "support": {"axis": "x", "value": "min", "tolerance": 1e-6},
                "loads": {"load_factor": 1.0, "pressure": 0.0, "pressure_file": None, "point_force": None},
                "solver": {"echo_level": 0, "linear_solver": solver},
                "outputs": {"vtk": os.path.join(job_dir, "box_solid.vtk"), "result": "box_solid_result.json"},
            }
            r2 = run(job_dir, job2)
            failures += not check(r2["ok"] and r2["model"] == "solid" and r2["second_order"], "box (solid): the skin / core tetrahedra job still runs, second order")
            inner_b, inner_H, inner_L = b - 2 * ts, H - 2 * ts, L - 2 * ts
            I_skin = (b * H ** 3 - inner_b * inner_H ** 3) / 12
            I_core = inner_b * inner_H ** 3 / 12
            EI2 = E_s * I_skin + E_c * I_core
            GA2 = G_s * (b * H - inner_b * inner_H) + G_c * inner_b * inner_H
            q2 = r2["mass"] * sf.G0 / L
            delta2 = q2 * L ** 4 / (8 * EI2) + q2 * L ** 2 / (2 * GA2)
            failures += not check(abs(r2["max_displacement"] / delta2 - 1.0) < 0.10,
                                  "box (solid): the tetrahedra deflect as the beam too (%.4g vs %.4g m, %.1f%%)" % (r2["max_displacement"], delta2, 100 * (r2["max_displacement"] / delta2 - 1)))
            print("  box (solid): %d dof solved in %.1f s; tip %.4g m vs %.4g; the shell model took %d dof" % (r2["dofs"], r2["seconds_solve"], r2["max_displacement"], delta2, r["dofs"]))
            failures += not check(r["dofs"] < 0.5 * r2["dofs"], "box: the shell model is under half the degrees of freedom of the tetrahedra (%d vs %d)" % (r["dofs"], r2["dofs"]))

        # ------------------------------------------------------------------
        # A real lattice core: the grid, homogenised.
        # ------------------------------------------------------------------
        n = 16
        a_vox = 4
        lo, hi = n // 2 - a_vox // 2, n // 2 + a_vox // 2
        grid = np.zeros((n, n, n), dtype=bool)
        grid[:, lo:hi, lo:hi] = True
        grid[lo:hi, :, lo:hi] = True
        grid[lo:hi, lo:hi, :] = True
        grid_cell = write_cell(job_dir, "grid.f32", grid, (0.02, 0.02, 0.02))
        job3 = dict(job)
        job3["regions"] = [{"id": 1, "label": "box", "skin": {"material": AL, "thickness": t},
                            "core": {"material": AL, "cell": grid_cell}}]
        job3["loads"] = {"load_factor": 4.0, "pressure": 0.0, "pressure_file": None, "point_force": None}
        job3["outputs"] = {"vtk": os.path.join(job_dir, "box_grid.vtk"), "result": "box_grid_result.json"}
        r3 = run(job_dir, job3)
        reg3 = r3["regions"][0]
        failures += not check(r3["ok"], "box (grid core): the job ran")
        failures += not check(abs(reg3["relative_density"] - grid.mean()) < 1e-9, "box (grid core): the core's relative density is the cell's (%.4f)" % reg3["relative_density"])
        failures += not check(reg3["E"][0] < 0.2 * AL["youngs_modulus"] and reg3["E"][0] > 0.02 * AL["youngs_modulus"], "box (grid core): E_core/E_s = %.4f" % (reg3["E"][0] / AL["youngs_modulus"]))
        failures += not check(reg3["core_max_micro_von_mises"] > reg3["core_max_von_mises"] and reg3["core_max_micro_von_mises"] <= reg3["core_max_micro_bound"] * (1 + 1e-6),
                              "box (grid core): micro stress above the macro (%.3g vs %.3g Pa) and within the bound %.3g" % (reg3["core_max_micro_von_mises"], reg3["core_max_von_mises"], reg3["core_max_micro_bound"]))
        failures += not check(reg3["core_margin"] > 0.0 and reg3["core_margin"] == AL["allowable_stress"] / reg3["core_max_micro_von_mises"] - 1.0, "box (grid core): the core margin is against the base allowable with the micro stress (%.1f)" % reg3["core_margin"])
        failures += not check(len(r3["per_material"]) == 2 and r3["per_material"][1]["kind"] == "core" and r3["per_material"][1]["max_von_mises"] == reg3["core_max_micro_von_mises"],
                              "box (grid core): per_material reports the core by its micro stress")
        failures += not check(reg3["homogenize_cached"] is False, "box (grid core): homogenised afresh with the cache off")
        print("  box (grid core): rho* %.3f, E/E_s %.4f, amplification %.1f; micro %.3g vs macro %.3g Pa; %d dof in %.1f s"
              % (reg3["relative_density"], reg3["E"][0] / AL["youngs_modulus"], reg3["amplification"], reg3["core_max_micro_von_mises"], reg3["core_max_von_mises"], r3["dofs"], r3["seconds_solve"]))

        # ------------------------------------------------------------------
        # The pressurised sphere.
        # ------------------------------------------------------------------
        sphere_dir = os.path.join(job_dir, "sphere")
        os.makedirs(sphere_dir)
        a, ts, p = 1.0, 0.02, 1.0e5
        nodes_s, tets_s, region_s, n_tris = sphere_tets(sphere_dir, a, 3, 0.25)
        write_mesh(os.path.join(sphere_dir, "sphere.bin"), nodes_s, tets_s, region_s)
        very_soft = {"name": "void", "density": 1.0, "youngs_modulus": 1.0e3, "poisson_ratio": 0.0, "allowable_stress": 0.0}
        cell_s = write_cell(sphere_dir, "solid.f32", np.ones((4, 4, 4), dtype=bool), (1.0, 1.0, 1.0))
        vtk_s = os.path.join(sphere_dir, "sphere.vtk")
        job_s = {
            "mesh": "sphere.bin", "model": "shell_core",
            "regions": [{"id": 1, "label": "sphere", "skin": {"material": AL, "thickness": ts}, "core": {"material": very_soft, "cell": cell_s}}],
            "support": {"axis": "z", "value": "min", "tolerance": 0.1},
            "loads": {"load_factor": 0.0, "pressure": p, "pressure_file": None, "point_force": None},
            "solver": {"echo_level": 0, "linear_solver": solver, "homogenize_cache": False},
            "outputs": {"vtk": vtk_s, "result": "sphere_result.json"},
        }
        rs = run(sphere_dir, job_s)
        failures += not check(rs["ok"] and rs["shells"] == n_tris, "sphere: a shell on each of the sphere's %d triangles" % n_tris)
        failures += not check(abs(rs["applied_pressure"][2]) < 1e-6 * p * np.pi * a ** 2 and np.linalg.norm(rs["reaction"]) < 1e-3 * p * np.pi * a ** 2,
                              "sphere: a uniform pressure over a closed surface has no resultant, so the support carries nothing (|R| = %.3g N)" % np.linalg.norm(rs["reaction"]))
        failures += not check(rs["mean_normal_displacement"] < 0.0, "sphere: an external pressure moves the surface inward (mean u.n = %.3g m)" % rs["mean_normal_displacement"])
        # Membrane stress and radial displacement, away from the support.
        points, cells, cell_types, pdata, cdata = read_vtk(vtk_s)
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
        write_mesh(os.path.join(beam_dir, "beam.bin"), bverts, np.zeros((0, 4), dtype=np.int32), np.zeros(0, dtype=np.int32))
        with open(os.path.join(beam_dir, "faces.bin"), "wb") as fh:
            fh.write(struct.pack("<I", len(btris)))
            fh.write(np.ascontiguousarray(btris, dtype="<i4").tobytes())
            fh.write(np.ones(len(btris), dtype="<i4").tobytes())
        P_tip = 1000.0
        job_b = {
            "mesh": "beam.bin", "faces": "faces.bin", "model": "regions",
            "regions": [{"id": 1, "label": "beam", "skin": {"material": AL, "thickness": bt}, "core": None, "solid": None}],
            "supports": [{"axis": "y", "value": "min", "tolerance": 0.0}],
            "loads": {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None,
                      "point_forces": [{"at": [0.5, 5.0, 0.0], "force": [0.0, 0.0, -P_tip / 2]}, {"at": [-0.5, 5.0, 0.0], "force": [0.0, 0.0, -P_tip / 2]}]},
            "solver": {"echo_level": 0, "linear_solver": solver, "homogenize_cache": False, "shell_element": "ShellThinElement3D3N"},
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
        failures += not check(rb["ok"] and rb["balanced"] and abs(rb["reaction"][2] - P_tip) < 0.01 * P_tip, "beam: the clamp carries the two tip loads (%.4g vs %.4g N)" % (rb["reaction"][2], P_tip))
        failures += not check(abs(-tip_uz / delta_b - 1.0) < 0.02, "beam: the thin shell's tip deflection is Timoshenko's (%.4g vs %.4g m, %.1f%%)" % (-tip_uz, delta_b, 100 * (-tip_uz / delta_b - 1)))
        failures += not check(rb["support_nodes"] == rb["support_corner_nodes"] == int((np.abs(bverts[:, 1] - bverts[:, 1].min()) < 1e-9).sum()), "beam: the clamp is the root ring and cap alone (%d nodes)" % rb["support_nodes"])
        # The flange stress at 2.5 m from the root, M c / I.
        cen_b = np.array([pts_b[c].mean(axis=0) for c in cells_b])
        band = (np.abs(cen_b[:, 1] + 2.5) < 0.3) & (np.abs(cen_b[:, 2]) > 0.45)
        sigma_b = P_tip * (bL - 2.5) * (bh / 2) / I_b
        med = float(np.median(cd_b["von_mises_element"][band]))
        failures += not check(abs(med / sigma_b - 1.0) < 0.05, "beam: the flange von Mises is M c / I (median %.4g vs %.4g Pa)" % (med, sigma_b))
        print("  beam: %d dof in %.1f s; tip %.4g m vs %.4g; flange %.4g Pa vs %.4g" % (rb["dofs"], rb["seconds_solve"], -tip_uz, delta_b, med, sigma_b))

        # The same bar as a solid of 10-node tetrahedra: the clamp must hold
        # the mid-edge nodes on the plane too (left free until 2026-11-02 the
        # bar came out 4-8% too flexible, converging at first order).
        nodes_t, tets_t, region_t = box_tets(bL, bb, bh, size=0.25)
        write_mesh(os.path.join(beam_dir, "bar.bin"), nodes_t, tets_t, region_t)
        job_t = {
            "mesh": "bar.bin", "faces": None, "model": "regions", "second_order": True,
            "regions": [{"id": 1, "label": "bar", "skin": None, "core": None, "solid": {"material": AL}}],
            "supports": [{"axis": "x", "value": "min", "tolerance": 0.0}],
            "loads": {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None,
                      "point_forces": [{"at": [bL, 0.5, 0.0], "force": [0.0, 0.0, -P_tip / 2]}, {"at": [bL, -0.5, 0.0], "force": [0.0, 0.0, -P_tip / 2]}]},
            "solver": {"echo_level": 0, "linear_solver": solver, "homogenize_cache": False},
            "outputs": {"vtk": os.path.join(beam_dir, "bar.vtk"), "result": "bar_result.json"},
        }
        rt = run(beam_dir, job_t)
        pts_t, cells_t, types_t, pd_t, cd_t = read_vtk(os.path.join(beam_dir, "bar.vtk"))
        root_t = np.abs(pts_t[:, 0] - pts_t[:, 0].min()) < 1e-9
        moving = int((root_t & (np.linalg.norm(pd_t["displacement"], axis=1) > 0.0)).sum())
        failures += not check(rt["ok"] and rt["second_order"] and rt["support_nodes"] > rt["support_corner_nodes"] and rt["support_nodes"] == int(root_t.sum()) and moving == 0,
                              "bar (10-node): every node on the clamped face is fixed, mid-edge nodes included (%d of %d on the face, %d corners, %d moving)" % (rt["support_nodes"], int(root_t.sum()), rt["support_corner_nodes"], moving))
        kappa = 10.0 * (1 + nu_b) / (12.0 + 11.0 * nu_b)
        I_t = bb * bh ** 3 / 12
        delta_t = P_tip * bL ** 3 / (3 * E_b * I_t) + P_tip * bL / (kappa * G_b * bb * bh)
        tip_t = pts_t[:, 0] > pts_t[:, 0].max() - 1e-6
        tip_uz_t = float(pd_t["displacement"][tip_t, 2].mean())
        failures += not check(abs(-tip_uz_t / delta_t - 1.0) < 0.03, "bar (10-node): the tip deflection is Timoshenko's (%.4g vs %.4g m, %.1f%%)" % (-tip_uz_t, delta_t, 100 * (-tip_uz_t / delta_t - 1)))
        print("  bar (10-node): %d dof in %.1f s; tip %.4g m vs %.4g" % (rt["dofs"], rt["seconds_solve"], -tip_uz_t, delta_t))

        if keep:
            os.makedirs(keep, exist_ok=True)
            for root, _, files in os.walk(job_dir):
                for name in files:
                    rel = os.path.relpath(os.path.join(root, name), job_dir)
                    os.makedirs(os.path.dirname(os.path.join(keep, rel)) or keep, exist_ok=True)
                    shutil.copy2(os.path.join(root, name), os.path.join(keep, rel))
            print("kept: %s" % keep)
    finally:
        shutil.rmtree(job_dir, ignore_errors=True)

    print("RESULT: %d failure(s)" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
