"""Linear static finite-element analysis of a lattice-filled body on
KratosMultiphysics' StructuralMechanicsApplication.

Run as a separate process in the WingDesigner micromamba environment
(``pip install KratosMultiphysics KratosStructuralMechanicsApplication
KratosLinearSolversApplication scipy``, Kratos all at the same version),
the same bridge pattern as the potential flow (potential_flow.py): the
application writes a job directory, this script reads it and writes
``result.json`` and the VTK solution the user named.

Everything in the job is SI: metres, kilograms, seconds, pascals, newtons.
The application converts from the project's units before writing the job
and converts the results back for display.

TWO MODELS of the same part, chosen by ``model``:

``shell_core`` (the default, and the one meant for a desktop). The core
of the body is tetrahedra bounded by the body's OWN surface triangles,
meshed at the lattice cell size, carrying the lattice's homogenised
elasticity tensor (homogenize.py, from one voxelised unit cell per
region, with UserProvidedLinearElastic3DLaw) and effective density; the
skin is a shell element on every one of those surface triangles, with the
skin thickness as a property (ShellThinElementCorotational3D3N: Kirchhoff
bending with the ANDES membrane, six degrees of freedom a node). Measured
on a sandwich box beam against the 10-node tetrahedra of the same part
(structural_fea_check.py): the thin triangle tracks them within 5% with or
without a core, where Kratos's thick (Reissner-Mindlin DSG) triangle came
out 23% too stiff once a core shared its nodes, and did not improve with
refinement -- so the thin one is the default and the thick one is a
``shell_element`` option only. The shells sit on the outer
surface and the core fills the whole body, so the two overlap by the skin
volume: the core is far softer than the skin, so the stiffness of that
overlap is nothing, and the core density is reduced by the overlap's
share so the mass and the inertial load are right. First-order tetrahedra
are fine here, the skin -- the bending problem -- being shells. A few
hundred thousand degrees of freedom for an airframe, against millions
for the model below.

``solid``. The skin / core tetrahedra tools/tet_gmsh.py builds (the skin
between the body's surface and MeshLib's offset of it, the core inside),
each tetrahedron a material by id. The full-resolution reference the
shell model is checked against on small bodies. First-order tetrahedra
lock in bending, and a skin one tetrahedron thick IS a bending problem,
so ``second_order`` adds a node at the middle of every edge and solves on
10-node tetrahedra (SmallDisplacementElement3D10N) with 6-node surface
conditions.

THE MESH. ``mesh.bin`` is counts, nodes as float64 xyz, tetrahedra as
int32 quads and an int32 region (material) id per tetrahedron -- the
layout tools/tet_gmsh.py's write_tet uses, without its magic. The wetted
surface is recovered as the faces that belong to one tetrahedron only;
with conforming regions those are exactly the outer surface, and each
face takes the region of the tetrahedron it came from.

LOADS. An inertial load factor (the body's own mass under ``load_factor``
x g along -z), a pressure on the wetted faces -- one number, or one per
face from ``pressure_file`` (the pressure field from the flow solvers
goes there) -- and a point force at the node nearest a point.
POSITIVE_FACE_PRESSURE on a condition acts AGAINST the face's normal, so
with the wetted faces outward a positive pressure presses inwards, as an
external pressure does; the mean of u.n over the wetted nodes is reported
so the sign can be seen to be right.

SUPPORT. Every degree of freedom (rotations too, where the node has them)
of the nodes within ``tolerance`` of the plane ``axis = value`` is fixed;
``value`` may be "min" or "max" for the mesh's own extreme. The reactions
there are summed and reported against the applied loads: equal and
opposite in a converged solution.

RESULTS. Per node the displacement; per element the von Mises stress
(the largest over its integration points -- for a shell over its top,
middle and bottom surfaces), averaged to the nodes for the VTK; per
region the peak von Mises of the skin and of the core and the margin
allowable / peak - 1. The core's von Mises is a MACRO stress, the average
over a cell: the struts carry more. So the core also reports the
estimated MICRO stress -- the peak voxel von Mises of each element's own
strain state over the cell's critical voxels (homogenize.py) -- and its
margin is against the base material's allowable using that.
"""

import hashlib
import json
import os
import struct
import sys
import time

import tet_gmsh as tg

np = tg.np
log = tg.log
JobError = tg.JobError

G0 = 9.80665
SHELL_PROPERTIES_OFFSET = 1000   # skin properties id = region id + this


# ---------------------------------------------------------------------------
# The mesh
# ---------------------------------------------------------------------------

def read_mesh(path):
    """(nodes float64 (n,3), tets int64 (m,4), region int64 (m,))."""
    with open(path, "rb") as f:
        nn, nt = struct.unpack("<II", f.read(8))
        nodes = np.frombuffer(f.read(nn * 24), dtype="<f8").reshape(nn, 3)
        tets = np.frombuffer(f.read(nt * 16), dtype="<i4").reshape(nt, 4)
        material = np.frombuffer(f.read(nt * 4), dtype="<i4")
    if nt == 0:
        raise JobError("the mesh has no tetrahedra.")
    if tets.min() < 0 or tets.max() >= nn:
        raise JobError("the mesh refers to nodes it does not have.")
    return (np.ascontiguousarray(nodes, dtype=np.float64), np.ascontiguousarray(tets, dtype=np.int64),
            np.ascontiguousarray(material, dtype=np.int64))


def signed_volumes(nodes, tets):
    a, b, c, d = (nodes[tets[:, i]] for i in range(4))
    return np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a) / 6.0


def orient(nodes, tets):
    """Positive volume throughout: Kratos wants the standard handedness."""
    vol = signed_volumes(nodes, tets)
    flipped = vol < 0.0
    tets = tets.copy()
    tets[flipped, 1], tets[flipped, 2] = tets[flipped, 2].copy(), tets[flipped, 1].copy()
    return tets, int(flipped.sum())


def boundary_faces(tets):
    """The faces that belong to one tetrahedron only, outward, with the
    tetrahedron each came from. For a mesh whose regions conform (skin and
    core share their interface) that is the outer surface."""
    # The four faces of a positively oriented tet, each wound outward.
    f = np.concatenate([tets[:, [0, 2, 1]], tets[:, [0, 1, 3]], tets[:, [1, 2, 3]], tets[:, [0, 3, 2]]])
    owner = np.concatenate([np.arange(len(tets))] * 4)
    key = np.sort(f, axis=1)
    _, first, counts = np.unique(key, axis=0, return_index=True, return_counts=True)
    single = first[counts == 1]
    return f[single], owner[single]


def face_normals_area(nodes, faces):
    """Outward normal times area of each face."""
    a, b, c = (nodes[faces[:, i]] for i in range(3))
    return 0.5 * np.cross(b - a, c - a)


def second_order(nodes, tets, faces):
    """Mid-edge nodes on every edge: 10-node tetrahedra in Kratos's order
    (corners, then the edges 01 12 20 03 13 23) and 6-node faces (corners,
    then 01 12 20). The corner nodes keep their ids."""
    n0 = len(nodes)
    edges = {}
    mids = []

    def mid(a, b):
        k = (a, b) if a < b else (b, a)
        i = edges.get(k)
        if i is None:
            i = n0 + len(mids)
            edges[k] = i
            mids.append(k)
        return i

    tet10 = np.empty((len(tets), 10), dtype=np.int64)
    tet10[:, :4] = tets
    for r, t in enumerate(tets):
        t = [int(v) for v in t]
        tet10[r, 4:] = [mid(t[0], t[1]), mid(t[1], t[2]), mid(t[2], t[0]), mid(t[0], t[3]), mid(t[1], t[3]), mid(t[2], t[3])]
    tri6 = np.empty((len(faces), 6), dtype=np.int64)
    tri6[:, :3] = faces
    for r, f in enumerate(faces):
        f = [int(v) for v in f]
        tri6[r, 3:] = [mid(f[0], f[1]), mid(f[1], f[2]), mid(f[2], f[0])]
    mids = np.array(mids, dtype=np.int64).reshape(-1, 2)
    new_nodes = np.concatenate([nodes, 0.5 * (nodes[mids[:, 0]] + nodes[mids[:, 1]])]) if len(mids) else nodes
    return new_nodes, tet10, tri6


# ---------------------------------------------------------------------------
# Homogenization of the regions' cores
# ---------------------------------------------------------------------------

def cache_dir():
    d = os.path.join(os.environ.get("TEMP") or os.environ.get("TMPDIR") or ".", "aerotelic_homogenize")
    os.makedirs(d, exist_ok=True)
    return d


def homogenize_region(job_dir, region, samples, use_cache=True):
    """The core's effective tensor, density factor and micro-stress data
    for one region, from the cell the job gives it (``cell``: file, nx,
    ny, nz, period) and its base material. Cached by the cell's bytes and
    the material, since a lattice homogenised once serves every body."""
    import homogenize as hz
    core = region["core"]
    base = core["material"]
    cell = core["cell"]
    raw_path = os.path.join(job_dir, cell["file"])
    nx, ny, nz = int(cell["nx"]), int(cell["ny"]), int(cell["nz"])
    values = np.fromfile(raw_path, dtype="<f4")
    if len(values) != nx * ny * nz:
        raise JobError("%s holds %d values for %d x %d x %d voxels." % (cell["file"], len(values), nx, ny, nz))
    period = np.asarray(cell["period"], dtype=np.float64)
    h = period / np.array([nx, ny, nz], dtype=np.float64)
    E, nu = float(base["youngs_modulus"]), float(base["poisson_ratio"])

    key = hashlib.sha1()
    key.update(values.tobytes())
    key.update(("%d %d %d %.12g %.12g %.12g %.12g %.12g %d v2" % (nx, ny, nz, h[0] / h[0], h[1] / h[0], h[2] / h[0], E, nu, samples)).encode())
    cache = os.path.join(cache_dir(), key.hexdigest() + ".json")
    if use_cache and os.path.exists(cache):
        try:
            with open(cache, "r", encoding="utf-8") as f:
                r = json.load(f)
            r["C"] = np.array(r["C"], dtype=np.float64)
            r["critical"] = np.array(r["critical"], dtype=np.float64).reshape(-1, 6, 6)
            r["cached"] = True
            return r
        except Exception:
            pass
    solid = hz.cell_from_values(values, nx, ny, nz)
    try:
        r = hz.homogenize(solid, h, E, nu, samples=samples, log=log)
    except hz.HomogenizationError as exc:
        raise JobError("%s: the core could not be homogenised: %s" % (region.get("label", region["id"]), exc))
    log("  %s: homogenised %s voxels (%d solid, relative density %.4f) in %.1f s: E/E_s = (%.4f, %.4f, %.4f), G/E_s = (%.4f, %.4f, %.4f), stress amplification %.1f"
        % (region.get("label", region["id"]), "%d x %d x %d" % (nx, ny, nz), r["elements"], r["relative_density"], r["seconds"],
           r["engineering"]["E"][0] / E, r["engineering"]["E"][1] / E, r["engineering"]["E"][2] / E,
           r["engineering"]["G"][0] / E, r["engineering"]["G"][1] / E, r["engineering"]["G"][2] / E, r["amplification"]))
    if use_cache:
        try:
            with open(cache, "w", encoding="utf-8") as f:
                json.dump(hz.to_json(r), f)
        except Exception:
            pass
    r["cached"] = False
    return r


# ---------------------------------------------------------------------------
# The Kratos input
# ---------------------------------------------------------------------------

def _write_sub_model_part(w, name, nodes, elements=(), conditions=()):
    w("Begin SubModelPart %s\n" % name)
    w("Begin SubModelPartNodes\n")
    for v in nodes:
        w("%d\n" % (int(v) + 1))
    w("End SubModelPartNodes\nBegin SubModelPartElements\n")
    for i in elements:
        w("%d\n" % (int(i) + 1))
    w("End SubModelPartElements\nBegin SubModelPartConditions\n")
    for i in conditions:
        w("%d\n" % (int(i) + 1))
    w("End SubModelPartConditions\nEnd SubModelPart\n\n")


def _write_block(w, kind, name, rows, first_id, prop_of_row):
    """Begin Elements/Conditions <name> ... End, ids first_id + row."""
    if len(rows) == 0:
        return
    w("Begin %s %s\n" % (kind, name))
    fmt = "%d %d " + " ".join(["%d"] * rows.shape[1]) + "\n"
    for i, row in enumerate(rows):
        w(fmt % ((first_id + i + 1, int(prop_of_row[i])) + tuple(int(v) + 1 for v in row)))
    w("End %s\n\n" % kind)


def write_mdpa_solid(path, nodes, elements, faces, material, support_nodes, point_node, material_ids):
    """The solid model: the tetrahedra under their material's Properties,
    the wetted faces as surface-load conditions, the point load's node as a
    point-load condition, and the sub model parts the parameters refer to
    (Material_<id>, Support, Wetted, PointLoad)."""
    element_name = "SmallDisplacementElement3D%dN" % elements.shape[1]
    face_name = "SurfaceLoadCondition3D%dN" % faces.shape[1]
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("Begin ModelPartData\nEnd ModelPartData\n\n")
        for mid in material_ids:
            w("Begin Properties %d\nEnd Properties\n" % mid)
        w("\nBegin Nodes\n")
        for i, p in enumerate(nodes):
            w("%d %.17g %.17g %.17g\n" % (i + 1, p[0], p[1], p[2]))
        w("End Nodes\n\n")
        _write_block(w, "Elements", element_name, elements, 0, material)
        _write_block(w, "Conditions", face_name, faces, 0, np.full(len(faces), material_ids[0]))
        if point_node is not None:
            w("Begin Conditions PointLoadCondition3D1N\n%d %d %d\nEnd Conditions\n\n" % (len(faces) + 1, int(material_ids[0]), point_node + 1))
        for mid in material_ids:
            sel = np.nonzero(material == mid)[0]
            _write_sub_model_part(w, "Material_%d" % mid, np.unique(elements[sel]), sel)
        _write_sub_model_part(w, "Support", support_nodes)
        _write_sub_model_part(w, "Wetted", np.unique(faces), (), range(len(faces)))
        if point_node is not None:
            _write_sub_model_part(w, "PointLoad", [point_node], (), [len(faces)])


def write_mdpa_shell_core(path, nodes, tets, faces, region_of_tet, region_of_face, support_nodes, point_node, region_ids, shell_element):
    """The shell / core model: the tetrahedra under their region's core
    Properties (id = region), a shell on every wetted face under the
    region's skin Properties (id = region + SHELL_PROPERTIES_OFFSET), the
    wetted faces as surface-load conditions, and the sub model parts
    Core_<id>, Skin_<id>, Support, Wetted, PointLoad."""
    nt = len(tets)
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("Begin ModelPartData\nEnd ModelPartData\n\n")
        for rid in region_ids:
            w("Begin Properties %d\nEnd Properties\n" % rid)
            w("Begin Properties %d\nEnd Properties\n" % (rid + SHELL_PROPERTIES_OFFSET))
        w("\nBegin Nodes\n")
        for i, p in enumerate(nodes):
            w("%d %.17g %.17g %.17g\n" % (i + 1, p[0], p[1], p[2]))
        w("End Nodes\n\n")
        _write_block(w, "Elements", "SmallDisplacementElement3D4N", tets, 0, region_of_tet)
        _write_block(w, "Elements", shell_element, faces, nt, region_of_face + SHELL_PROPERTIES_OFFSET)
        _write_block(w, "Conditions", "SurfaceLoadCondition3D3N", faces, 0, region_of_face + SHELL_PROPERTIES_OFFSET)
        if point_node is not None:
            w("Begin Conditions PointLoadCondition3D1N\n%d %d %d\nEnd Conditions\n\n" % (len(faces) + 1, int(region_ids[0]) + SHELL_PROPERTIES_OFFSET, point_node + 1))
        for rid in region_ids:
            sel = np.nonzero(region_of_tet == rid)[0]
            _write_sub_model_part(w, "Core_%d" % rid, np.unique(tets[sel]), sel)
            sel = np.nonzero(region_of_face == rid)[0]
            _write_sub_model_part(w, "Skin_%d" % rid, np.unique(faces[sel]), sel + nt)
        _write_sub_model_part(w, "Support", support_nodes)
        _write_sub_model_part(w, "Wetted", np.unique(faces), (), range(len(faces)))
        if point_node is not None:
            _write_sub_model_part(w, "PointLoad", [point_node], (), [len(faces)])


def write_materials(path, entries):
    """entries: [(sub model part name, properties id, law name, {variable: value})]."""
    props = []
    for name, pid, law, variables in entries:
        props.append({
            "model_part_name": "Structure." + name,
            "properties_id": int(pid),
            "Material": {"constitutive_law": {"name": law}, "Variables": variables, "Tables": {}},
        })
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"properties": props}, f, indent=1)


def project_parameters(mdpa_base, materials_path, loads, has_point, echo_level, rotation_dofs, linear_solver):
    constraints = [{
        "python_module": "assign_vector_variable_process",
        "kratos_module": "KratosMultiphysics",
        "Parameters": {
            "model_part_name": "Structure.Support",
            "variable_name": "DISPLACEMENT",
            "constrained": [True, True, True],
            "value": [0.0, 0.0, 0.0],
            "interval": [0.0, "End"],
        },
    }]
    if rotation_dofs:
        # A clamp: the shells' rotations at the support are fixed too. A
        # node with no shell has the variable but no equation, and fixing
        # it is harmless.
        constraints.append({
            "python_module": "assign_vector_variable_process",
            "kratos_module": "KratosMultiphysics",
            "Parameters": {
                "model_part_name": "Structure.Support",
                "variable_name": "ROTATION",
                "constrained": [True, True, True],
                "value": [0.0, 0.0, 0.0],
                "interval": [0.0, "End"],
            },
        })
    load_list = []
    g = float(loads["load_factor"]) * G0
    if g != 0.0:
        load_list.append({
            "python_module": "assign_vector_by_direction_process",
            "kratos_module": "KratosMultiphysics",
            "Parameters": {
                "model_part_name": "Structure",
                "variable_name": "VOLUME_ACCELERATION",
                # A load, not a degree of freedom: the process must not try
                # to fix it (its default would).
                "constrained": False,
                "modulus": abs(g),
                "direction": [0.0, 0.0, -1.0 if g > 0.0 else 1.0],
                "interval": [0.0, "End"],
            },
        })
    if has_point:
        pf = loads["point_force"]["force"]
        load_list.append({
            "python_module": "assign_vector_variable_process",
            "kratos_module": "KratosMultiphysics",
            "Parameters": {
                "model_part_name": "Structure.PointLoad",
                "variable_name": "POINT_LOAD",
                "constrained": [False, False, False],
                "value": [float(pf[0]), float(pf[1]), float(pf[2])],
                "interval": [0.0, "End"],
            },
        })
    if linear_solver == "sparse_lu":
        solver = {"solver_type": "sparse_lu"}
    else:
        solver = {
            "solver_type": "amgcl",
            "tolerance": 1e-9,
            "max_iteration": 3000,
            "krylov_type": "cg",
            "smoother_type": "ilu0",
            "coarsening_type": "aggregation",
            "scaling": False,
            # With shells, their nodes carry six degrees of freedom and the
            # solid nodes three, so the matrix has no uniform block; a solid
            # model keeps Kratos's 3 x 3 blocks, which converge better.
            "use_block_matrices_if_possible": not rotation_dofs,
            "verbosity": 0,
        }
    return {
        "problem_data": {
            "problem_name": "aerotelic_structure",
            "parallel_type": "OpenMP",
            "echo_level": echo_level,
            "start_time": 0.0,
            "end_time": 1.0,
        },
        "solver_settings": {
            "solver_type": "Static",
            "model_part_name": "Structure",
            "domain_size": 3,
            "echo_level": echo_level,
            "analysis_type": "linear",
            "model_import_settings": {"input_type": "mdpa", "input_filename": mdpa_base},
            "material_import_settings": {"materials_filename": materials_path},
            "time_stepping": {"time_step": 1.0},
            "line_search": False,
            "convergence_criterion": "residual_criterion",
            "displacement_relative_tolerance": 1e-6,
            "displacement_absolute_tolerance": 1e-9,
            "residual_relative_tolerance": 1e-6,
            "residual_absolute_tolerance": 1e-9,
            "max_iteration": 10,
            "rotation_dofs": bool(rotation_dofs),
            "compute_reactions": True,
            "block_builder": True,
            "linear_solver_settings": solver,
        },
        "processes": {
            "constraints_process_list": constraints,
            "loads_process_list": load_list,
            "list_other_processes": [],
        },
        "output_processes": {},
    }


# ---------------------------------------------------------------------------
# The solve
# ---------------------------------------------------------------------------

def _integration_values(elem, variable, process_info):
    try:
        values = elem.CalculateOnIntegrationPoints(variable, process_info)
    except Exception:
        return []
    return list(values) if values is not None else []


def run_kratos(job_dir, parameters, n_nodes, face_pressure, want_rotations, echo_level):
    """Builds and solves the Kratos model from the files already in
    job_dir. Returns the model part and the analysis for post-processing,
    with the solve time."""
    import KratosMultiphysics as KM
    from KratosMultiphysics.StructuralMechanicsApplication.structural_mechanics_analysis import StructuralMechanicsAnalysis
    try:
        # Registers sparse_lu (Eigen's SparseLU); amgcl is in the core.
        import KratosMultiphysics.LinearSolversApplication  # noqa: F401
    except ImportError:
        if parameters["solver_settings"]["linear_solver_settings"]["solver_type"] == "sparse_lu":
            raise JobError("the sparse_lu solver needs KratosLinearSolversApplication (pip install it at Kratos's version).")

    if echo_level == 0:
        KM.Logger.GetDefaultOutput().SetSeverity(KM.Logger.Severity.WARNING)
    model = KM.Model()
    t_solve = time.time()
    analysis = StructuralMechanicsAnalysis(model, KM.Parameters(json.dumps(parameters)))
    analysis.Initialize()
    structure = model["Structure"]
    if structure.NumberOfNodes() != n_nodes:
        raise JobError("Kratos read %d of %d nodes." % (structure.NumberOfNodes(), n_nodes))
    # The pressure, per wetted face: one mechanism whether it is a constant
    # or a field, set on the conditions after the model is in.
    wetted = structure.GetSubModelPart("Wetted")
    for cond in wetted.Conditions:
        cond.SetValue(KM.POSITIVE_FACE_PRESSURE, float(face_pressure[cond.Id - 1]))
    analysis.RunSolutionLoop()
    analysis.Finalize()
    return model, structure, analysis, time.time() - t_solve


def read_nodal(structure, n_nodes, want_rotations):
    import KratosMultiphysics as KM
    displacement = np.zeros((n_nodes, 3))
    rotation = np.zeros((n_nodes, 3)) if want_rotations else None
    for node in structure.Nodes:
        i = node.Id - 1
        u = node.GetSolutionStepValue(KM.DISPLACEMENT)
        displacement[i] = (u[0], u[1], u[2])
        if want_rotations:
            r = node.GetSolutionStepValue(KM.ROTATION)
            rotation[i] = (r[0], r[1], r[2])
    # The reactions at the support. Read from the Support sub model part
    # rather than by IsFixed(): the constraint process frees the degrees
    # of freedom again once the step is over, so nothing is fixed by now.
    reaction = np.zeros((n_nodes, 3))
    for node in structure.GetSubModelPart("Support").Nodes:
        r = node.GetSolutionStepValue(KM.REACTION)
        reaction[node.Id - 1] = (r[0], r[1], r[2])
    return displacement, rotation, reaction


def element_von_mises(structure, n_elements):
    """Peak von Mises over the integration points of every element, shells
    over their top, middle and bottom surfaces."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    process_info = structure.ProcessInfo
    von_mises = np.zeros(n_elements)
    shell_vars = [getattr(SMA, n, None) or getattr(KM, n, None)
                  for n in ("VON_MISES_STRESS_TOP_SURFACE", "VON_MISES_STRESS_MIDDLE_SURFACE", "VON_MISES_STRESS_BOTTOM_SURFACE")]
    shell_vars = [v for v in shell_vars if v is not None]
    vm_var = getattr(SMA, "VON_MISES_STRESS", None) or KM.VON_MISES_STRESS
    for elem in structure.Elements:
        n = elem.GetGeometry().PointsNumber()
        best = 0.0
        if n == 3:
            for var in shell_vars:
                values = _integration_values(elem, var, process_info)
                if values:
                    best = max(best, max(float(v) for v in values))
            if not shell_vars or best == 0.0:
                values = _integration_values(elem, vm_var, process_info)
                if values:
                    best = max(best, max(float(v) for v in values))
        else:
            values = _integration_values(elem, vm_var, process_info)
            if values:
                best = max(float(v) for v in values)
        von_mises[elem.Id - 1] = best
    return von_mises


def core_strains_stresses(structure, n_elements):
    """The strain and stress Voigt vectors (Kratos order, engineering
    shear) at the first integration point of every tetrahedron, zero for
    the shells."""
    import KratosMultiphysics as KM
    process_info = structure.ProcessInfo
    strain = np.zeros((n_elements, 6))
    stress = np.zeros((n_elements, 6))
    strain_vars = [getattr(KM, n, None) for n in ("STRAIN", "GREEN_LAGRANGE_STRAIN_VECTOR")]
    stress_vars = [getattr(KM, n, None) for n in ("CAUCHY_STRESS_VECTOR", "PK2_STRESS_VECTOR")]
    for elem in structure.Elements:
        if elem.GetGeometry().PointsNumber() != 4:
            continue
        i = elem.Id - 1
        for var in strain_vars:
            if var is None:
                continue
            values = _integration_values(elem, var, process_info)
            if values and len(values[0]) == 6:
                strain[i] = np.mean([[float(c) for c in v] for v in values], axis=0)
                break
        for var in stress_vars:
            if var is None:
                continue
            values = _integration_values(elem, var, process_info)
            if values and len(values[0]) == 6:
                stress[i] = np.mean([[float(c) for c in v] for v in values], axis=0)
                break
    return strain, stress


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

VTK_TETRA, VTK_QUADRATIC_TETRA, VTK_TRIANGLE = 10, 24, 5


def write_vtk(path, nodes, cells, cell_types, point_data, cell_data):
    """Legacy VTK unstructured grid; `cells` a list of index arrays (each
    one block of equal-size cells), point_data / cell_data dicts of arrays
    (n,) or (n, 3)."""
    n_cells = sum(len(c) for c in cells)
    size = sum(len(c) * (c.shape[1] + 1) for c in cells)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# vtk DataFile Version 3.0\nAerotelic structural analysis (SI: m, Pa)\nASCII\nDATASET UNSTRUCTURED_GRID\n")
        f.write("POINTS %d double\n" % len(nodes))
        np.savetxt(f, nodes, fmt="%.9g %.9g %.9g")
        f.write("CELLS %d %d\n" % (n_cells, size))
        for c in cells:
            np.savetxt(f, np.concatenate([np.full((len(c), 1), c.shape[1]), c], axis=1), fmt="%d")
        f.write("CELL_TYPES %d\n" % n_cells)
        np.savetxt(f, np.concatenate([np.full(len(c), t) for c, t in zip(cells, cell_types)]), fmt="%d")
        f.write("POINT_DATA %d\n" % len(nodes))
        for name, data in point_data.items():
            data = np.asarray(data)
            if data.ndim == 2:
                f.write("VECTORS %s double\n" % name)
                np.savetxt(f, data, fmt="%.9g %.9g %.9g")
            else:
                f.write("SCALARS %s double 1\nLOOKUP_TABLE default\n" % name)
                np.savetxt(f, data, fmt="%.9g")
        f.write("CELL_DATA %d\n" % n_cells)
        for name, data in cell_data.items():
            data = np.asarray(data)
            if data.dtype.kind in "iu":
                f.write("SCALARS %s int 1\nLOOKUP_TABLE default\n" % name)
                np.savetxt(f, data, fmt="%d")
            else:
                f.write("SCALARS %s double 1\nLOOKUP_TABLE default\n" % name)
                np.savetxt(f, data, fmt="%.9g")


def nodal_average(n_nodes, element_blocks, values):
    """The mean over the elements at each node of a per-element value;
    element_blocks a list of connectivity arrays laid end to end in the
    order of `values`."""
    out = np.zeros(n_nodes)
    count = np.zeros(n_nodes)
    start = 0
    for block in element_blocks:
        v = values[start:start + len(block)]
        for k in range(block.shape[1]):
            np.add.at(out, block[:, k], v)
            np.add.at(count, block[:, k], 1.0)
        start += len(block)
    return out / np.maximum(count, 1.0)


# ---------------------------------------------------------------------------
# The job
# ---------------------------------------------------------------------------

def _support_nodes(nodes, support):
    axis = {"x": 0, "y": 1, "z": 2}.get(str(support["axis"]).lower())
    if axis is None:
        raise JobError("the support axis must be x, y or z.")
    lo, hi = nodes.min(axis=0), nodes.max(axis=0)
    diagonal = float(np.linalg.norm(hi - lo))
    value = support["value"]
    if value == "min":
        value = float(lo[axis])
    elif value == "max":
        value = float(hi[axis])
    else:
        value = float(value)
    tolerance = float(support["tolerance"]) or 1e-6 * diagonal
    support_nodes = np.nonzero(np.abs(nodes[:, axis] - value) <= tolerance)[0]
    if len(support_nodes) < 3:
        raise JobError("only %d node(s) lie within %.3g of the plane %s = %.6g; a support needs a face of them. "
                       "Move the plane onto the geometry (or use min / max) or widen the tolerance."
                       % (len(support_nodes), tolerance, "xyz"[axis], value))
    return support_nodes, axis, value, tolerance


def _check_material(m, label):
    if not (float(m["youngs_modulus"]) > 0.0 and float(m["density"]) > 0.0):
        raise JobError("'%s' has no stiffness or no density." % label)
    nu = float(m["poisson_ratio"])
    if not (0.0 <= nu < 0.5):
        raise JobError("'%s' has a Poisson's ratio of %g; it must be in [0, 0.5)." % (label, nu))


def run_job(job_dir):
    """job.json:

        mesh          the mesh (read_mesh), SI, a region id per tetrahedron
        model         "shell_core" (default) or "solid"
        regions       shell_core: [{"id", "label",
                        "skin": {"material": {...}, "thickness": metres},
                        "core": {"material": {... the lattice's BASE
                                 material ...},
                                 "cell": {"file", "nx", "ny", "nz", "period"},
                                 "density_correction": true}}],
                      one per region id in the mesh. A material is
                      {"name", "density", "youngs_modulus", "poisson_ratio",
                      "allowable_stress"}, SI.
        materials     solid: [{"id", "label", "name", "density",
                      "youngs_modulus", "poisson_ratio", "allowable_stress"}]
                      one per material id in the mesh
        second_order  solid: 10-node tetrahedra (default true)
        support       {"axis": "x"|"y"|"z", "value": number | "min" |
                      "max", "tolerance": distance (0: 1e-6 of the diagonal)}
        loads         {"load_factor": g's along -z, "pressure": pascals on
                      every wetted face, "pressure_file": float64 per face
                      overriding it, "point_force": {"at": [x,y,z],
                      "force": [fx,fy,fz]} or null}
        solver        {"echo_level", "linear_solver": "amgcl" | "sparse_lu",
                      "homogenize_samples": 2000, "homogenize_cache": true,
                      "shell_element": "ShellThinElementCorotational3D3N"
                      (or ShellThickElementCorotational3D3N, see above)}
        outputs       {"vtk": the solution file, "result": the statistics}
    """
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job["outputs"]
    solver_opts = {"echo_level": 0, "linear_solver": "amgcl", "homogenize_samples": 2000, "homogenize_cache": True,
                   "shell_element": "ShellThinElementCorotational3D3N"}
    solver_opts.update(job.get("solver", {}))
    echo_level = int(solver_opts["echo_level"])
    loads = {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None, "point_force": None}
    loads.update(job.get("loads", {}))
    support = {"axis": "y", "value": 0.0, "tolerance": 0.0}
    support.update(job.get("support", {}))
    model_kind = str(job.get("model", "shell_core"))
    if model_kind not in ("shell_core", "solid"):
        raise JobError("the model must be shell_core or solid, not '%s'." % model_kind)

    nodes, tets, region = read_mesh(os.path.join(job_dir, job["mesh"]))
    tets, flipped = orient(nodes, tets)
    faces, owners = boundary_faces(tets)
    if len(faces) == 0:
        raise JobError("the mesh has no boundary faces.")
    region_of_face = region[owners]
    tet_vol = signed_volumes(nodes, tets)
    n_area = face_normals_area(nodes, faces)
    face_area = np.linalg.norm(n_area, axis=1)
    wetted_vol = tg.surface_volume(nodes, faces)
    present = sorted(set(int(v) for v in np.unique(region)))
    log("  %d nodes, %d tetrahedra (%d flipped), %d wetted faces enclosing %.6g (tetrahedra %.6g); regions %s"
        % (len(nodes), len(tets), flipped, len(faces), wetted_vol, tet_vol.sum(), present))

    support_nodes, axis, value, tolerance = _support_nodes(nodes, support)

    # The pressure, per wetted face.
    if loads.get("pressure_file"):
        face_pressure = np.fromfile(os.path.join(job_dir, loads["pressure_file"]), dtype="<f8")
        if len(face_pressure) != len(faces):
            raise JobError("the pressure file has %d values for %d wetted faces." % (len(face_pressure), len(faces)))
    else:
        face_pressure = np.full(len(faces), float(loads["pressure"]))

    # The point force: the nearest corner node.
    point_node = None
    applied_point = np.zeros(3)
    if loads.get("point_force"):
        at = np.array(loads["point_force"]["at"], dtype=np.float64)
        point_node = int(np.argmin(np.linalg.norm(nodes - at, axis=1)))
        applied_point = np.array(loads["point_force"]["force"], dtype=np.float64)

    mdpa_base = os.path.join(job_dir, "structure")
    materials_path = os.path.join(job_dir, "materials.json")
    result = {
        "ok": True, "model": model_kind, "flipped": flipped, "wetted_faces": int(len(faces)),
        "support_nodes": int(len(support_nodes)), "support_plane": {"axis": "xyz"[axis], "value": value, "tolerance": tolerance},
        "applied_point": [float(v) for v in applied_point],
    }

    if model_kind == "solid":
        materials = job["materials"]
        ids = sorted({int(m["id"]) for m in materials})
        missing = [m for m in present if m not in ids]
        if missing:
            raise JobError("the mesh has tetrahedra of material %s, which the job gives no material for." % missing)
        for m in materials:
            _check_material(m, m.get("label", m["id"]))
        density = np.zeros(len(tets))
        for m in materials:
            density[region == int(m["id"])] = float(m["density"])
        mass = float((density * tet_vol).sum())

        use_second = bool(job.get("second_order", True))
        if use_second:
            nodes_all, elements, faces_out = second_order(nodes, tets, faces)
        else:
            nodes_all, elements, faces_out = nodes, tets, faces
        t0 = time.time()
        write_mdpa_solid(mdpa_base + ".mdpa", nodes_all, elements, faces_out, region, support_nodes, point_node, ids)
        write_materials(materials_path, [
            ("Material_%d" % int(m["id"]), int(m["id"]), "LinearElastic3DLaw",
             {"DENSITY": float(m["density"]), "YOUNG_MODULUS": float(m["youngs_modulus"]), "POISSON_RATIO": float(m["poisson_ratio"])})
            for m in materials])
        seconds_mdpa = time.time() - t0
        parameters = project_parameters(mdpa_base, materials_path, loads, point_node is not None, echo_level, False, solver_opts["linear_solver"])
        model, structure, analysis, seconds_solve = run_kratos(job_dir, parameters, len(nodes_all), face_pressure, False, echo_level)
        displacement, _, reaction = read_nodal(structure, len(nodes_all), False)
        von_mises_elem = element_von_mises(structure, len(elements))
        dofs = 3 * len(nodes_all)
        element_blocks = [elements]
        cell_types = [VTK_QUADRATIC_TETRA if use_second else VTK_TETRA]
        cell_data = {"material": region.astype(np.int64), "von_mises_element": von_mises_elem}
        per_material = []
        for m in materials:
            mid = int(m["id"])
            sel = region == mid
            peak = float(von_mises_elem[sel].max()) if sel.any() else 0.0
            allowable = float(m.get("allowable_stress", 0.0))
            per_material.append({
                "id": mid, "label": m.get("label", ""), "kind": "solid", "max_von_mises": peak,
                "margin": (allowable / peak - 1.0) if (allowable > 0.0 and peak > 0.0) else 0.0,
                "mass": float((tet_vol[sel] * float(m["density"])).sum()),
            })
        result.update({"second_order": use_second, "elements": int(len(elements)), "tets": int(len(elements)), "shells": 0,
                       "seconds_mdpa": seconds_mdpa, "regions": []})
        n_vtk_nodes = len(nodes_all)
        vtk_nodes = nodes_all
        i_vm = int(np.argmax(von_mises_elem))
        max_vm_region = int(region[i_vm])
    else:
        regions = job.get("regions", [])
        by_id = {int(r["id"]): r for r in regions}
        missing = [m for m in present if m not in by_id]
        if missing:
            raise JobError("the mesh has tetrahedra of region %s, which the job gives no skin and core for." % missing)
        ids = sorted(by_id)
        for r in regions:
            _check_material(r["skin"]["material"], "%s skin" % r.get("label", r["id"]))
            _check_material(r["core"]["material"], "%s core" % r.get("label", r["id"]))
            if not float(r["skin"]["thickness"]) > 0.0:
                raise JobError("'%s' has no skin thickness." % r.get("label", r["id"]))

        # Homogenise each region's core and size its densities.
        t0 = time.time()
        homog = {}
        material_entries = []
        core_density = np.zeros(len(tets))
        skin_density_t = np.zeros(len(faces))     # rho t per face
        region_info = []
        for rid in ids:
            r = by_id[rid]
            h = homogenize_region(job_dir, r, int(solver_opts["homogenize_samples"]), bool(solver_opts["homogenize_cache"]))
            homog[rid] = h
            base, skin = r["core"]["material"], r["skin"]["material"]
            t = float(r["skin"]["thickness"])
            sel_t, sel_f = region == rid, region_of_face == rid
            volume = float(tet_vol[sel_t].sum())
            area = float(face_area[sel_f].sum())
            rho_eff = float(base["density"]) * h["relative_density"]
            # The shells and the core overlap by the skin's volume: take
            # it off the core so the mass, and so the inertial load, is
            # the part's.
            overlap = min(area * t / volume, 1.0) if volume > 0.0 else 0.0
            correct = bool(r["core"].get("density_correction", True))
            rho_core = rho_eff * (1.0 - overlap) if correct else rho_eff
            core_density[sel_t] = rho_core
            skin_density_t[sel_f] = float(skin["density"]) * t
            material_entries.append(("Core_%d" % rid, rid, "UserProvidedLinearElastic3DLaw",
                                     {"DENSITY": rho_core, "ELASTICITY_TENSOR": [[float(v) for v in row] for row in h["C"]]}))
            material_entries.append(("Skin_%d" % rid, rid + SHELL_PROPERTIES_OFFSET, "LinearElasticPlaneStress2DLaw",
                                     {"DENSITY": float(skin["density"]), "YOUNG_MODULUS": float(skin["youngs_modulus"]),
                                      "POISSON_RATIO": float(skin["poisson_ratio"]), "THICKNESS": t}))
            region_info.append({
                "id": rid, "label": r.get("label", ""), "volume": volume, "area": area, "skin_thickness": t,
                "skin_mass": float(skin["density"]) * t * area, "core_mass": rho_core * volume,
                "core_density_effective": rho_eff, "core_density_used": rho_core, "core_overlap_fraction": overlap,
                "relative_density": h["relative_density"], "dropped_fraction": h["dropped_fraction"],
                "E": h["engineering"]["E"], "G": h["engineering"]["G"],
                "nu_xy": h["engineering"]["nu_xy"], "nu_yz": h["engineering"]["nu_yz"], "nu_xz": h["engineering"]["nu_xz"],
                "amplification": h["amplification"], "homogenize_seconds": h["seconds"], "homogenize_cached": h.get("cached", False),
                "homogenize_dofs": h["dofs"], "C": [[float(v) for v in row] for row in h["C"]],
            })
        seconds_homogenize = time.time() - t0
        mass = float((core_density * tet_vol).sum() + (skin_density_t * face_area).sum())

        t0 = time.time()
        write_mdpa_shell_core(mdpa_base + ".mdpa", nodes, tets, faces, region, region_of_face, support_nodes, point_node, ids, solver_opts["shell_element"])
        write_materials(materials_path, material_entries)
        seconds_mdpa = time.time() - t0
        parameters = project_parameters(mdpa_base, materials_path, loads, point_node is not None, echo_level, True, solver_opts["linear_solver"])
        model, structure, analysis, seconds_solve = run_kratos(job_dir, parameters, len(nodes), face_pressure, True, echo_level)
        displacement, rotation, reaction = read_nodal(structure, len(nodes), True)
        n_elements = len(tets) + len(faces)
        von_mises_elem = element_von_mises(structure, n_elements)
        strain, stress = core_strains_stresses(structure, n_elements)
        # The micro stress of every core element from its own strain state.
        import homogenize as hz
        micro = np.zeros(n_elements)
        bound = np.zeros(n_elements)
        for rid in ids:
            sel = np.nonzero(region == rid)[0]
            h = homog[rid]
            micro[sel] = hz.micro_von_mises(h["critical"], strain[sel])
            bound[sel] = h["amplification"] * hz.frobenius(stress[sel])
        # Degrees of freedom: three per node, three more per shell node.
        shell_nodes = np.unique(faces)
        dofs = 3 * len(nodes) + 3 * len(shell_nodes)
        element_blocks = [tets, faces]
        cell_types = [VTK_TETRA, VTK_TRIANGLE]
        kind = np.concatenate([np.zeros(len(tets), dtype=np.int64), np.ones(len(faces), dtype=np.int64)])
        cell_data = {"material": np.concatenate([region, region_of_face]).astype(np.int64), "kind": kind,
                     "von_mises_element": von_mises_elem, "micro_von_mises_element": micro, "micro_von_mises_bound": bound}
        per_material = []
        for info in region_info:
            rid = info["id"]
            r = by_id[rid]
            skin, base = r["skin"]["material"], r["core"]["material"]
            sel_f = np.nonzero(region_of_face == rid)[0] + len(tets)
            sel_t = np.nonzero(region == rid)[0]
            peak_skin = float(von_mises_elem[sel_f].max()) if len(sel_f) else 0.0
            peak_core = float(von_mises_elem[sel_t].max()) if len(sel_t) else 0.0
            peak_micro = float(micro[sel_t].max()) if len(sel_t) else 0.0
            peak_bound = float(bound[sel_t].max()) if len(sel_t) else 0.0
            a_skin = float(skin.get("allowable_stress", 0.0))
            a_base = float(base.get("allowable_stress", 0.0))
            info.update({"skin_max_von_mises": peak_skin, "skin_margin": (a_skin / peak_skin - 1.0) if (a_skin > 0.0 and peak_skin > 0.0) else 0.0,
                         "core_max_von_mises": peak_core, "core_max_micro_von_mises": peak_micro, "core_max_micro_bound": peak_bound,
                         "core_margin": (a_base / peak_micro - 1.0) if (a_base > 0.0 and peak_micro > 0.0) else 0.0})
            per_material.append({"id": rid, "label": "%s skin" % info["label"], "kind": "skin", "max_von_mises": peak_skin,
                                 "margin": info["skin_margin"], "mass": info["skin_mass"]})
            per_material.append({"id": rid, "label": "%s core" % info["label"], "kind": "core", "max_von_mises": peak_micro,
                                 "macro_von_mises": peak_core, "margin": info["core_margin"], "mass": info["core_mass"]})
        result.update({"second_order": False, "elements": int(n_elements), "tets": int(len(tets)), "shells": int(len(faces)),
                       "seconds_mdpa": seconds_mdpa, "seconds_homogenize": seconds_homogenize, "regions": region_info})
        n_vtk_nodes = len(nodes)
        vtk_nodes = nodes
        # The peak over everything is the shells' or the cores' MICRO stress.
        combined = von_mises_elem.copy()
        combined[:len(tets)] = micro[:len(tets)]
        i_vm = int(np.argmax(combined))
        von_mises_report = combined
        max_vm_region = int(region[i_vm]) if i_vm < len(tets) else int(region_of_face[i_vm - len(tets)])

    if model_kind == "solid":
        von_mises_report = von_mises_elem

    # The force balance. Gravity on the mass; the pressure's resultant as
    # -p n A over the outward faces; the point force as given.
    g = float(loads["load_factor"]) * G0
    applied_gravity = np.array([0.0, 0.0, -mass * g])
    applied_pressure = -(face_pressure[:, None] * n_area).sum(axis=0)
    reaction_sum = reaction.sum(axis=0)

    # Inward or outward: u.n averaged over the wetted corner nodes, with
    # the node normal the area-weighted mean of its faces'.
    node_normal = np.zeros((len(nodes), 3))
    for k in range(3):
        np.add.at(node_normal, faces[:, k], n_area)
    wet_nodes = np.unique(faces)
    nn = node_normal[wet_nodes]
    nn /= np.maximum(np.linalg.norm(nn, axis=1), 1e-300)[:, None]
    mean_normal_disp = float(np.einsum("ij,ij->i", displacement[wet_nodes], nn).mean())

    u_mag = np.linalg.norm(displacement, axis=1)
    i_max = int(np.argmax(u_mag))
    result.update({
        "nodes": int(n_vtk_nodes),
        "dofs": int(dofs),
        "mass": mass,
        "max_displacement": float(u_mag[i_max]),
        "max_displacement_at": [float(v) for v in vtk_nodes[i_max]],
        "max_von_mises": float(von_mises_report[i_vm]),
        "max_von_mises_material": max_vm_region,
        "per_material": per_material,
        "reaction": [float(v) for v in reaction_sum],
        "applied_gravity": [float(v) for v in applied_gravity],
        "applied_pressure": [float(v) for v in applied_pressure],
        "mean_normal_displacement": mean_normal_disp,
        "seconds_solve": seconds_solve,
        "vtk": outputs["vtk"],
    })

    t_vtk = time.time()
    point_data = {"displacement": displacement, "von_mises": nodal_average(n_vtk_nodes, element_blocks, von_mises_elem)}
    if model_kind == "shell_core":
        point_data["rotation"] = rotation
        point_data["micro_von_mises"] = nodal_average(n_vtk_nodes, element_blocks, micro)
    write_vtk(outputs["vtk"], vtk_nodes, element_blocks, cell_types, point_data, cell_data)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved %d dofs (%s: %d tetrahedra, %d shells) in %.2f s: mass %.6g kg, max displacement %.4g m, max von Mises %.4g Pa (region %d); "
        "reaction (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g); wrote %s"
        % (dofs, model_kind, result["tets"], result["shells"], result["seconds_solve"], mass, result["max_displacement"], result["max_von_mises"], result["max_von_mises_material"],
           reaction_sum[0], reaction_sum[1], reaction_sum[2], applied_gravity[0], applied_gravity[1], applied_gravity[2],
           applied_pressure[0], applied_pressure[1], applied_pressure[2], applied_point[0], applied_point[1], applied_point[2],
           outputs["vtk"]))
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
        log("usage: structural_fea.py <job directory>")
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
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
