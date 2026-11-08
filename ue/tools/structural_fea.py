"""Linear static finite-element analysis of scene bodies as SHELLS, on
KratosMultiphysics' StructuralMechanicsApplication.

THE MODEL (2026-11-08) is shells only. Every structural element of a body
is a shell: its SKIN, a shell of the skin thickness on every triangle of
the body's own grid (the wetted faces, ``faces.bin``), and its RIBS and
SPARS, shells on the planar discs a body's plane networks cut out of it
(``ribs.bin``, meshed by tools/rib_gmsh.py). A rib disc's rim is the
skin's own nodes -- the application cuts the body's surface by each plane
first, so the rib shells and the skin shells share nodes and the solver
sees one connected structure; two crossing networks share the chord where
they meet. The model is a set of REGIONS, one per body (its skin) and one
per plane network (its ribs); a region may have a skin, ribs, or both.
Tetrahedral regions (solids, the skin / core tetrahedra of tet_gmsh.py)
and homogenised lattice cores were removed on 2026-11-08 when the
application stopped supporting tetrahedral FEA and lattice infill.

THE SHELL ELEMENT is the LINEAR thin triangle, ShellThinElement3D3N (since
2026-10-31). The corotational variants are geometrically nonlinear: solved
in the one step of a linear analysis, their internal forces are evaluated
on the displaced configuration while the step was linearised on the
undeformed one, so once the displacements are large the reactions no
longer balance the loads and the stresses are nonsense (a 5 mm skin-only
P-38 airframe under a 5 kN tip load moves 2.7 m: the corotational thin
shell reported reactions of (-10, -11, 7) kN against a 5 kN load and
8.6e10 Pa; the linear one (0.008, 0.000, 5000) N and beam theory's stress
along the wing). For the millimetre displacements of the checks the two
agree. Measured on a sandwich box beam against 10-node tetrahedra of the
same part (while that comparison existed): the thin triangle tracked them
within 5%, where Kratos's thick (Reissner-Mindlin DSG) triangle came out
23% too stiff and did not improve with refinement -- so the thin one is
the default and the corotational thin and thick triangles remain
``shell_element`` options. Kirchhoff bending with the ANDES membrane, six
degrees of freedom a node: every node of this model is a shell node.

Run as a separate process in the WingDesigner micromamba environment
(``pip install KratosMultiphysics KratosStructuralMechanicsApplication
KratosLinearSolversApplication scipy``, Kratos all at the same version),
the same bridge pattern as the potential flow (potential_flow.py): the
application writes a job directory, this script reads it and writes
``result.json`` and the VTK solution the user named.

Everything in the job is SI: metres, kilograms, seconds, pascals, newtons.
The application converts from the project's units before writing the job
and converts the results back for display.

THE MESH. ``nodes.bin`` is a uint32 count then float64 xyz per node (the
surface's vertices first, then the ribs' interior nodes, as rib_gmsh.py
writes them; its ``AERONOD1`` magic is accepted and skipped). ``faces.bin``
is a uint32 count, int32 node triples wound outward, then an int32 region
per face: the skin shells, and the WETTED faces, in this order -- a
pressure per face (``pressure_file``) is in this order. ``ribs.bin`` has
the same layout for the rib faces with their region; a rib face is a
shell of its region's rib material and thickness and NOTHING ELSE: it is
not wetted, no pressure lands on it and it takes no part in the mean
normal displacement; its mass counts.

LOADS. An inertial load factor (the model's own mass under ``load_factor``
x g along -z), a pressure on the wetted faces -- one number, or one per
face from ``pressure_file`` (the pressure field from the flow solvers
goes there) -- and a point force at the node nearest a point.
POSITIVE_FACE_PRESSURE on a condition acts AGAINST the face's normal, so
with the wetted faces outward a positive pressure presses inwards, as an
external pressure does; the mean of u.n over the wetted nodes is reported
so the sign can be seen to be right.

SUPPORT. Every degree of freedom (displacements and rotations) of the
nodes within ``tolerance`` of the plane ``axis = value`` is fixed; ``value``
may be "min" or "max" for the mesh's own extreme, and ``bounds`` limits the
plane to a rectangular patch. The result reports ``support_nodes`` (and
``support_corner_nodes``, the same number: there are no mid-edge nodes any
more, the key stays for the application). The reactions there are summed
and reported against the applied loads: equal and opposite in a converged
solution.

RESULTS. Per node the displacement and rotation; per element the von
Mises stress (the largest over its top, middle and bottom surfaces),
averaged to the nodes for the VTK; per region the peak von Mises of the
skin and of the ribs and the margin allowable / peak - 1 (``per_material``,
kind "skin" or "rib").

THE LINEAR SOLVER. "auto" takes the direct solver (sparse_lu) up to
AUTO_DIRECT_DOFS degrees of freedom and amgcl beyond. The direct solver is
exact; amgcl (cg + ilu0) can stop short on the stiffness contrasts of thin
shells and give a solution that LOOKS fine -- small displacements, low
stresses -- with reactions nowhere near the loads (the hershey_bar wing,
2026-11-01: 11,880 dofs, 75 s to 0.1 mm and reactions of 3 kN against a
1 kN load; sparse_lu took 0.55 s and matched beam theory to 0.2%). The
result's ``balanced`` / ``balance_error`` is the tell, and an unbalanced
solve is reported as a warning.

NATURAL FREQUENCIES (2026-11-04). With ``solver.modes`` > 0 the same model
-- the same mesh, materials and clamps, no loads -- is solved a second
time as the generalised eigenvalue problem K phi = omega^2 M phi on
StructuralMechanicsApplication's ``eigen_value`` solver (the dynamic
scheme: K the stiffness the static solve used, M the elements' mass
matrices from the same densities and shell thicknesses, so the
frequencies are of the part as the static analysis has it). The
eigensolver is LinearSolversApplication's Spectra shift-invert
(``spectra_sym_g_eigs_shift``, the lowest modes first) with the subspace
solver ``eigen_eigensystem`` as the fallback. The result lists
``natural_frequencies`` in hertz, lowest first, and the VTK carries each
mode shape as the point vector ``mode_k`` scaled to a unit largest
displacement. Measured on the 1 x 10 x 1 m thin-walled box beam of
structural_fea_check.py: the degenerate first bending pair at 10.81 Hz
twice (Euler-Bernoulli 11.71 Hz for the bare beam; with the tip cap's
mass, the webs' shear flexibility and rotary inertia 10.97 Hz, so the
shells are within 1.5%), then the 1 m skin panels' own plate modes from
26 Hz -- a thin-walled part's spectrum is local long before the second
beam mode. A failure of the eigensolve is a warning on an otherwise good
static result, not an error. Gotcha: the per-node EIGENVECTOR_MATRIX is
in the node's dof order, which is by variable KEY (a name hash), and
ROTATION sorts before DISPLACEMENT -- run_modes picks the displacement
columns by that same sort.
"""

import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gmsh_common as tg  # noqa: E402  (bootstraps Library\\bin before numpy)

np = tg.np
log = tg.log
JobError = tg.JobError

G0 = 9.80665
SHELL_PROPERTIES_OFFSET = 1000   # skin properties id = region id + this
RIB_PROPERTIES_OFFSET = 2000     # rib shell properties id = region id + this
NODES_MAGIC = b"AERONOD1"        # rib_gmsh.py's nodes.bin header, optional here

# "auto" takes the direct solver (sparse_lu) up to this many degrees of
# freedom and amgcl beyond (see the docstring). The threshold is a
# desktop's: Eigen's SparseLU runs on one thread and its factors grow
# faster than the system (a 64,056-dof sphere took two minutes; a
# 540,000-dof fuselage ran a 32 GB machine out of memory, 2026-11-05).
# The application mirrors it (AerotelicFeaPreprocess::kAutoDirectDofs)
# to warn before a solve is started.
AUTO_DIRECT_DOFS = 150000
# The reactions must balance the applied loads to this fraction of the
# loads' magnitude, or the solution is reported as not converged.
BALANCE_TOLERANCE = 0.01
# A zero support tolerance is this fraction of the mesh's diagonal. A
# lofted section is not exactly planar (hershey_bar's root ring sits
# within +-0.3 mm of y = 0 on a 10 m diagonal): a millionth took two of
# its thirty nodes; a ten-thousandth takes the ring.
SUPPORT_TOLERANCE_OVER_DIAGONAL = 1e-4


def resolve_linear_solver(name, dofs):
    """The solver "auto" stands for at this size; any other name as given."""
    name = str(name or "auto").lower()
    if name == "auto":
        return "sparse_lu" if dofs <= AUTO_DIRECT_DOFS else "amgcl"
    return name


# ---------------------------------------------------------------------------
# The mesh
# ---------------------------------------------------------------------------

def read_nodes(path):
    """The nodes, float64 (n, 3): a uint32 count then xyz per node, with
    rib_gmsh.py's ``AERONOD1`` header accepted in front of the count."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] == NODES_MAGIC:
        data = data[8:]
    if len(data) < 4:
        raise JobError("%s holds no node count." % os.path.basename(path))
    (nn,) = struct.unpack("<I", data[:4])
    if len(data) < 4 + nn * 24:
        raise JobError("%s is truncated (%d bytes for %d nodes)." % (os.path.basename(path), len(data), nn))
    nodes = np.frombuffer(data[4:4 + nn * 24], dtype="<f8").reshape(nn, 3)
    return np.ascontiguousarray(nodes, dtype=np.float64)


def read_faces(path):
    """(faces int64 (k, 3), region int64 (k,)): a uint32 count, int32
    triples, then an int32 region per face."""
    with open(path, "rb") as f:
        (nf,) = struct.unpack("<I", f.read(4))
        faces = np.frombuffer(f.read(nf * 12), dtype="<i4").reshape(nf, 3)
        region = np.frombuffer(f.read(nf * 4), dtype="<i4")
    return np.ascontiguousarray(faces, dtype=np.int64), np.ascontiguousarray(region, dtype=np.int64)


def face_normals_area(nodes, faces):
    """Outward normal times area of each face."""
    a, b, c = (nodes[faces[:, i]] for i in range(3))
    return 0.5 * np.cross(b - a, c - a)


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


def _write_block(w, kind, name, rows, ids, prop_of_row):
    """Begin Elements/Conditions <name> ... End, one row per element or
    condition with its id (0-based here, 1-based in the file)."""
    if len(rows) == 0:
        return
    w("Begin %s %s\n" % (kind, name))
    fmt = "%d %d " + " ".join(["%d"] * rows.shape[1]) + "\n"
    for i, row in enumerate(rows):
        w(fmt % ((int(ids[i]) + 1, int(prop_of_row[i])) + tuple(int(v) + 1 for v in row)))
    w("End %s\n\n" % kind)


def write_mdpa(path, nodes, shells, shell_prop, shell_part, wetted, wetted_prop, support_nodes, point_loads, property_ids, shell_element, point_prop):
    """The model: a shell element on every skin face and every rib face
    (`shells`, with `shell_prop` the Properties id per shell -- region +
    SHELL_PROPERTIES_OFFSET for a skin, + RIB_PROPERTIES_OFFSET for a rib
    -- and `shell_part` the sub model part each belongs to, Skin_<id> or
    Rib_<id>), the wetted faces as surface-load conditions whose ids are
    the faces' order (so a pressure per face maps by id), the point loads'
    nodes as point-load conditions, and the sub model parts the materials
    and the parameters refer to (Skin_<id>, Rib_<id>, Support, Wetted,
    PointLoad_<i>)."""
    n_cond = len(wetted)
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("Begin ModelPartData\nEnd ModelPartData\n\n")
        for pid in property_ids:
            w("Begin Properties %d\nEnd Properties\n" % int(pid))
        w("\nBegin Nodes\n")
        for i, p in enumerate(nodes):
            w("%d %.17g %.17g %.17g\n" % (i + 1, p[0], p[1], p[2]))
        w("End Nodes\n\n")
        _write_block(w, "Elements", shell_element, shells, np.arange(len(shells)), shell_prop)
        _write_block(w, "Conditions", "SurfaceLoadCondition3D3N", wetted, np.arange(n_cond), wetted_prop)
        if point_loads:
            w("Begin Conditions PointLoadCondition3D1N\n")
            for i, (node, _) in enumerate(point_loads):
                w("%d %d %d\n" % (n_cond + 1 + i, int(point_prop), int(node) + 1))
            w("End Conditions\n\n")
        parts = np.array(shell_part)
        for name in sorted(set(str(p) for p in parts)):
            sel = np.nonzero(parts == name)[0]
            _write_sub_model_part(w, name, np.unique(shells[sel]), sel)
        _write_sub_model_part(w, "Support", support_nodes)
        _write_sub_model_part(w, "Wetted", np.unique(wetted) if n_cond else [], (), range(n_cond))
        # One sub model part per point force: each carries its own POINT_LOAD.
        for i, (node, _) in enumerate(point_loads):
            _write_sub_model_part(w, "PointLoad_%d" % i, [node], (), [n_cond + i])


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


def _constraint_processes():
    """The clamp at the support, shared by the static and the modal solve:
    displacements and rotations alike (every node is a shell node)."""
    return [{
        "python_module": "assign_vector_variable_process",
        "kratos_module": "KratosMultiphysics",
        "Parameters": {
            "model_part_name": "Structure.Support",
            "variable_name": variable,
            "constrained": [True, True, True],
            "value": [0.0, 0.0, 0.0],
            "interval": [0.0, "End"],
        },
    } for variable in ("DISPLACEMENT", "ROTATION")]


def modal_parameters(mdpa_base, materials_path, echo_level, count, eigensolver):
    """The eigenvalue analysis of the same model: the clamp, no loads, the
    lowest `count` modes of K phi = omega^2 M phi."""
    settings = {"solver_type": eigensolver, "number_of_eigenvalues": int(count), "max_iteration": 1000, "echo_level": echo_level}
    if eigensolver == "eigen_eigensystem":
        settings["tolerance"] = 1e-6
    return {
        "problem_data": {
            "problem_name": "aerotelic_modes",
            "parallel_type": "OpenMP",
            "echo_level": echo_level,
            "start_time": 0.0,
            "end_time": 1.0,
        },
        "solver_settings": {
            "solver_type": "eigen_value",
            "model_part_name": "Structure",
            "domain_size": 3,
            "echo_level": echo_level,
            "model_import_settings": {"input_type": "mdpa", "input_filename": mdpa_base},
            "material_import_settings": {"materials_filename": materials_path},
            "time_stepping": {"time_step": 1.0},
            "rotation_dofs": True,
            "block_builder": True,
            "eigensolver_settings": settings,
        },
        "processes": {
            "constraints_process_list": _constraint_processes(),
            "loads_process_list": [],
            "list_other_processes": [],
        },
        "output_processes": {},
    }


def project_parameters(mdpa_base, materials_path, loads, point_loads, echo_level, linear_solver):
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
    for i, (_, pf) in enumerate(point_loads):
        load_list.append({
            "python_module": "assign_vector_variable_process",
            "kratos_module": "KratosMultiphysics",
            "Parameters": {
                "model_part_name": "Structure.PointLoad_%d" % i,
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
            "use_block_matrices_if_possible": False,
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
            "rotation_dofs": True,
            "compute_reactions": True,
            "block_builder": True,
            "linear_solver_settings": solver,
        },
        "processes": {
            "constraints_process_list": _constraint_processes(),
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


def run_kratos(job_dir, parameters, n_nodes, face_pressure, echo_level):
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
    log("  Kratos: reading the model")
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
    log("  Kratos: model read in %.1f s; assembling and solving with %s" % (time.time() - t_solve, parameters["solver_settings"]["linear_solver_settings"]["solver_type"]))
    analysis.RunSolutionLoop()
    analysis.Finalize()
    log("  Kratos: solved in %.1f s; reading the solution" % (time.time() - t_solve))
    return model, structure, analysis, time.time() - t_solve


def run_modes(job_dir, mdpa_base, materials_path, n_nodes, echo_level, count, eigensolver):
    """The lowest `count` natural frequencies (Hz, ascending) and mode shapes
    ((count, n_nodes, 3), each scaled to a unit largest displacement) of the
    model already written to mdpa_base, clamped as the static solve was.
    Spectra first; the subspace solver if it fails."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    from KratosMultiphysics.StructuralMechanicsApplication.structural_mechanics_analysis import StructuralMechanicsAnalysis
    import KratosMultiphysics.LinearSolversApplication  # noqa: F401  (the eigensolvers)
    if echo_level == 0:
        KM.Logger.GetDefaultOutput().SetSeverity(KM.Logger.Severity.WARNING)
    solvers = [eigensolver] + [s for s in ("spectra_sym_g_eigs_shift", "eigen_eigensystem") if s != eigensolver]
    # EIGENVECTOR_MATRIX has a column per degree of freedom of the node IN
    # THE NODE'S OWN ORDER, which Kratos keeps sorted by variable key -- a
    # hash of the name, so ROTATION_X/Y/Z (key 9130...) come BEFORE
    # DISPLACEMENT_X/Y/Z (9385...) on a shell node (found 2026-11-04: the
    # first three columns read as the mode shape were the rotations, and the
    # "shape" peaked at mid-span). Sort the same way and pick the
    # displacements' columns.
    disp = [KM.DISPLACEMENT_X, KM.DISPLACEMENT_Y, KM.DISPLACEMENT_Z]
    rot = [KM.ROTATION_X, KM.ROTATION_Y, KM.ROTATION_Z]
    order6 = sorted(disp + rot, key=lambda v: v.Key())
    cols6 = [order6.index(v) for v in disp]
    order3 = sorted(disp, key=lambda v: v.Key())
    cols3 = [order3.index(v) for v in disp]
    last = None
    for solver in solvers:
        model = KM.Model()
        try:
            analysis = StructuralMechanicsAnalysis(model, KM.Parameters(json.dumps(
                modal_parameters(mdpa_base, materials_path, echo_level, count, solver))))
            analysis.Run()
        except Exception as exc:  # a solver that is not built in, or did not converge
            last = "%s: %s" % (solver, exc)
            log("  modes: %s failed (%s)" % (solver, exc))
            continue
        structure = model["Structure"]
        values = structure.ProcessInfo[SMA.EIGENVALUE_VECTOR]
        lam = np.array([float(values[i]) for i in range(len(values))])
        shapes = np.zeros((len(lam), n_nodes, 3))
        for node in structure.Nodes:
            m = node.GetValue(SMA.EIGENVECTOR_MATRIX)
            rows = min(m.Size1(), len(lam))
            cols = cols6 if m.Size2() == 6 else (cols3 if m.Size2() == 3 else [0, 1, 2])
            for k in range(rows):
                shapes[k, node.Id - 1] = (m[k, cols[0]], m[k, cols[1]], m[k, cols[2]])
        order = np.argsort(lam)
        lam, shapes = lam[order], shapes[order]
        # omega^2 to hertz; a tiny negative from round-off on a rigid mode is 0.
        frequencies = np.sqrt(np.maximum(lam, 0.0)) / (2.0 * np.pi)
        for k in range(len(lam)):
            peak = float(np.linalg.norm(shapes[k], axis=1).max())
            if peak > 0.0:
                shapes[k] /= peak
        return frequencies, shapes, solver
    raise JobError("the natural frequencies could not be computed: %s" % last)


def read_nodal(structure, n_nodes):
    import KratosMultiphysics as KM
    displacement = np.zeros((n_nodes, 3))
    rotation = np.zeros((n_nodes, 3))
    for node in structure.Nodes:
        i = node.Id - 1
        u = node.GetSolutionStepValue(KM.DISPLACEMENT)
        displacement[i] = (u[0], u[1], u[2])
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
    """Peak von Mises over the integration points of every shell, over its
    top, middle and bottom surfaces."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    process_info = structure.ProcessInfo
    von_mises = np.zeros(n_elements)
    shell_vars = [getattr(SMA, n, None) or getattr(KM, n, None)
                  for n in ("VON_MISES_STRESS_TOP_SURFACE", "VON_MISES_STRESS_MIDDLE_SURFACE", "VON_MISES_STRESS_BOTTOM_SURFACE")]
    shell_vars = [v for v in shell_vars if v is not None]
    vm_var = getattr(SMA, "VON_MISES_STRESS", None) or KM.VON_MISES_STRESS
    for elem in structure.Elements:
        best = 0.0
        for var in shell_vars:
            values = _integration_values(elem, var, process_info)
            if values:
                best = max(best, max(float(v) for v in values))
        if not shell_vars or best == 0.0:
            values = _integration_values(elem, vm_var, process_info)
            if values:
                best = max(best, max(float(v) for v in values))
        von_mises[elem.Id - 1] = best
    return von_mises


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

VTK_TRIANGLE = 5


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

def _support_nodes(nodes, supports):
    """The nodes any of the support planes fixes (each once), and the
    planes resolved: [{"axis", "value", "tolerance", "bounds", "nodes"}].
    A support with ``bounds`` [u0, u1, w0, w1] is a PATCH of its plane:
    only the nodes inside that rectangle along the plane's two in-plane
    axes -- the other two axes in cyclic order, (axis + 1) % 3 then
    (axis + 2) % 3 -- are fixed (2026-11-05: an infinite plane through a
    fuselage also clamped the wing root)."""
    lo, hi = nodes.min(axis=0), nodes.max(axis=0)
    diagonal = float(np.linalg.norm(hi - lo))
    fixed = np.zeros(len(nodes), dtype=bool)
    planes = []
    for support in supports:
        axis = {"x": 0, "y": 1, "z": 2}.get(str(support["axis"]).lower())
        if axis is None:
            raise JobError("a support axis must be x, y or z.")
        value = support["value"]
        if value == "min":
            value = float(lo[axis])
        elif value == "max":
            value = float(hi[axis])
        else:
            value = float(value)
        tolerance = float(support.get("tolerance", 0.0)) or SUPPORT_TOLERANCE_OVER_DIAGONAL * diagonal
        on = np.abs(nodes[:, axis] - value) <= tolerance
        bounds = support.get("bounds")
        if bounds is not None:
            u0, u1, w0, w1 = (float(v) for v in bounds)
            u, w = (axis + 1) % 3, (axis + 2) % 3
            on &= (nodes[:, u] >= min(u0, u1)) & (nodes[:, u] <= max(u0, u1)) & (nodes[:, w] >= min(w0, w1)) & (nodes[:, w] <= max(w0, w1))
            bounds = [u0, u1, w0, w1]
        fixed |= on
        planes.append({"axis": "xyz"[axis], "value": value, "tolerance": tolerance, "bounds": bounds, "nodes": int(on.sum())})
    support_nodes = np.nonzero(fixed)[0]
    if len(support_nodes) < 3:
        raise JobError("only %d node(s) lie on the support plane(s); a support needs a face of them. "
                       "Move a plane onto the geometry (or use min / max) or widen the tolerance." % len(support_nodes))
    return support_nodes, planes


def _check_material(m, label):
    if not (float(m["youngs_modulus"]) > 0.0 and float(m["density"]) > 0.0):
        raise JobError("'%s' has no stiffness or no density." % label)
    nu = float(m["poisson_ratio"])
    if not (0.0 <= nu < 0.5):
        raise JobError("'%s' has a Poisson's ratio of %g; it must be in [0, 0.5)." % (label, nu))


def _regions_from_job(job):
    """The regions: [{"id", "label", "skin": null | {...}, "rib": null | {...}}]."""
    return [{"id": int(r["id"]), "label": r.get("label", ""), "skin": r.get("skin"), "rib": r.get("rib")}
            for r in job.get("regions", [])]


def run_job(job_dir):
    """job.json:

        nodes         the nodes (read_nodes), SI: the bodies' grid vertices
                      and the ribs' interior nodes
        faces         null, or the SKIN faces (read_faces): every triangle
                      of each body's own grid that carries a skin shell,
                      wound outward, with its region id. These are the
                      wetted faces, in this order.
        rib_faces     null, or a file in the same layout of RIB faces: the
                      planar shells inside the bodies (the cross-sections
                      of the plane networks, meshed by tools/rib_gmsh.py
                      with their rims on the skin's nodes), with their
                      region id. A rib face is a shell of its region's
                      "rib" material and thickness and NOTHING ELSE: it is
                      not wetted, so no pressure lands on it and it takes
                      no part in the mean normal displacement; its mass
                      counts.
        regions       [{"id", "label",
                        "skin": null | {"material": {...}, "thickness": m},
                        "rib": null | {"material": {...}, "thickness": m}}],
                      one per region id in the faces: shells of the skin's
                      thickness on the region's faces when it has a skin,
                      shells of the rib's thickness on its rib faces when
                      it has ribs; a region may have either or both. A
                      material is {"name", "density", "youngs_modulus",
                      "poisson_ratio", "allowable_stress"}, SI.
        supports      [{"axis": "x"|"y"|"z", "value": number | "min" |
                      "max", "tolerance": distance (0: 1e-4 of the
                      diagonal), "bounds": null | [u0, u1, w0, w1]}], every
                      plane clamping its nodes -- within the rectangle
                      along the plane's in-plane axes ((axis + 1) % 3,
                      (axis + 2) % 3) when bounds are given
        loads         {"load_factor": g's along -z, "pressure": pascals on
                      every wetted face, "pressure_file": float64 per
                      wetted face overriding it (in faces.bin order),
                      "point_forces": [{"at": [x,y,z], "force": [fx,fy,fz]}]}
        solver        {"echo_level",
                      "linear_solver": "auto" (the default: sparse_lu up to
                      AUTO_DIRECT_DOFS degrees of freedom, amgcl beyond) |
                      "amgcl" | "sparse_lu",
                      "shell_element": "ShellThinElement3D3N" (the linear
                      thin shell; or ShellThinElementCorotational3D3N /
                      ShellThickElementCorotational3D3N, see above),
                      "modes": 0 (how many natural frequencies to compute
                      after the static solve; 0: none),
                      "eigensolver": "spectra_sym_g_eigs_shift" (or
                      "eigen_eigensystem"; either falls back to the other)}
        outputs       {"vtk": the solution file, "result": the statistics}

    The result reports the model's size ("nodes", "elements", "dofs" --
    six per node --, "shells" all the shells, "rib_shells" those of them
    that are ribs, "wetted_faces", "support_nodes" and
    "support_corner_nodes", the same number), "mass", the peak
    displacement and where ("max_displacement", "max_displacement_at"),
    the peak von Mises and whose ("max_von_mises",
    "max_von_mises_material"), "per_material" entries of kind "skin" or
    "rib" with their peak, margin, allowable and mass, the force balance
    -- "reaction", "applied_gravity", "applied_pressure", "applied_point",
    "balance_error" = |sum of reactions + applied loads| over the loads'
    magnitude, "balanced" when it is within BALANCE_TOLERANCE -- a
    "warnings" list, the solver used ("linear_solver"),
    "mean_normal_displacement", and the times ("seconds",
    "seconds_solve", "seconds_modes"). An unbalanced solve is a solver
    that did not converge, and its fields are not to be trusted. With
    modes > 0 it also reports "natural_frequencies" (Hz, lowest first;
    empty with a warning if the eigensolve failed) and "eigensolver", and
    the VTK carries the mode shapes as "mode_1" ... point vectors. A
    failure writes {"ok": false, "error": ...} and exits 1.
    """
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job["outputs"]
    solver_opts = {"echo_level": 0, "linear_solver": "auto", "shell_element": "ShellThinElement3D3N", "modes": 0, "eigensolver": "spectra_sym_g_eigs_shift"}
    solver_opts.update(job.get("solver", {}))
    echo_level = int(solver_opts["echo_level"])
    loads = {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None, "point_forces": None}
    loads.update(job.get("loads", {}))
    point_forces = list(loads.get("point_forces") or [])
    supports = list(job.get("supports") or [])
    if not supports:
        raise JobError("no support: give at least one plane whose nodes are fixed.")
    regions = _regions_from_job(job)

    # The mesh: the nodes, the skin faces (wetted) and the rib faces.
    nodes = read_nodes(os.path.join(job_dir, job["nodes"]))
    if len(nodes) == 0:
        raise JobError("the mesh has no nodes.")

    def faces_of(key, what):
        if not job.get(key):
            return np.zeros((0, 3), dtype=np.int64), np.zeros(0, dtype=np.int64)
        f, r = read_faces(os.path.join(job_dir, job[key]))
        if len(f) and (f.min() < 0 or f.max() >= len(nodes)):
            raise JobError("the %s refer to nodes the mesh does not have." % what)
        return f, r

    faces, region_of_face = faces_of("faces", "skin faces")
    rib_faces, rib_region = faces_of("rib_faces", "rib faces")
    if len(faces) == 0 and len(rib_faces) == 0:
        raise JobError("nothing to analyse: no skin faces and no rib faces.")
    n_area = face_normals_area(nodes, faces) if len(faces) else np.zeros((0, 3))
    face_area = np.linalg.norm(n_area, axis=1)
    rib_area = np.linalg.norm(face_normals_area(nodes, rib_faces), axis=1) if len(rib_faces) else np.zeros(0)
    present_faces = sorted(set(int(v) for v in np.unique(region_of_face))) if len(region_of_face) else []
    present_ribs = sorted(set(int(v) for v in np.unique(rib_region))) if len(rib_region) else []
    log("  mesh read: %d nodes, %d skin faces, %d rib faces; regions %s"
        % (len(nodes), len(faces), len(rib_faces), sorted(set(present_faces) | set(present_ribs))))

    support_nodes, support_planes = _support_nodes(nodes, supports)

    # The pressure, per wetted face.
    if loads.get("pressure_file"):
        face_pressure = np.fromfile(os.path.join(job_dir, loads["pressure_file"]), dtype="<f8")
        if len(face_pressure) != len(faces):
            raise JobError("the pressure file has %d values for %d wetted faces." % (len(face_pressure), len(faces)))
    else:
        face_pressure = np.full(len(faces), float(loads["pressure"]))

    # The point forces: each at the nearest node.
    point_loads = []   # (node, force)
    applied_point = np.zeros(3)
    for pf in point_forces:
        at = np.array(pf["at"], dtype=np.float64)
        node = int(np.argmin(np.linalg.norm(nodes - at, axis=1)))
        force = np.array(pf["force"], dtype=np.float64)
        point_loads.append((node, force))
        applied_point += force

    # The regions against the mesh: every id present has one, and each
    # region is what its faces need.
    by_id = {int(r["id"]): r for r in regions}
    missing = [m for m in sorted(set(present_faces) | set(present_ribs)) if m not in by_id]
    if missing:
        raise JobError("the mesh has faces of region %s, which the job gives nothing for." % missing)
    ids = sorted(by_id)
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", rid)
        has_faces = rid in present_faces
        has_ribs = rid in present_ribs
        if r.get("skin"):
            _check_material(r["skin"]["material"], "%s skin" % label)
            if not float(r["skin"]["thickness"]) > 0.0:
                raise JobError("'%s' has no skin thickness." % label)
            if not has_faces:
                raise JobError("'%s' has a skin but no face to put its shells on." % label)
        elif has_faces:
            raise JobError("'%s' has skin faces but no skin (thickness and material) to put on them." % label)
        if r.get("rib"):
            _check_material(r["rib"]["material"], "%s ribs" % label)
            if not float(r["rib"]["thickness"]) > 0.0:
                raise JobError("'%s' has no rib thickness." % label)
            if not has_ribs:
                raise JobError("'%s' has ribs but no rib face to put their shells on." % label)
        elif has_ribs:
            raise JobError("'%s' has rib faces but says nothing of what the ribs are." % label)
        if not (r.get("skin") or r.get("rib")):
            raise JobError("'%s' is nothing: no skin, no ribs." % label)

    mdpa_base = os.path.join(job_dir, "structure")
    materials_path = os.path.join(job_dir, "materials.json")
    result = {
        "ok": True, "wetted_faces": int(len(faces)),
        "support_nodes": int(len(support_nodes)), "support_corner_nodes": int(len(support_nodes)), "support_planes": support_planes,
        "point_nodes": [int(n) for n, _ in point_loads],
        "applied_point": [float(v) for v in applied_point],
    }

    # The materials: a plane-stress law per skin and per rib, and rho t
    # per face for the mass.
    material_entries = []
    property_ids = []
    skin_density_t = np.zeros(len(faces))     # rho t per skin face
    rib_density_t = np.zeros(len(rib_faces))  # rho t per rib face
    for rid in ids:
        r = by_id[rid]
        skin = r.get("skin")
        if skin:
            t = float(skin["thickness"])
            skin_density_t[region_of_face == rid] = float(skin["material"]["density"]) * t
            material_entries.append(("Skin_%d" % rid, rid + SHELL_PROPERTIES_OFFSET, "LinearElasticPlaneStress2DLaw",
                                     {"DENSITY": float(skin["material"]["density"]), "YOUNG_MODULUS": float(skin["material"]["youngs_modulus"]),
                                      "POISSON_RATIO": float(skin["material"]["poisson_ratio"]), "THICKNESS": t}))
            property_ids.append(rid + SHELL_PROPERTIES_OFFSET)
        rib = r.get("rib")
        if rib:
            t_rib = float(rib["thickness"])
            rib_density_t[rib_region == rid] = float(rib["material"]["density"]) * t_rib
            material_entries.append(("Rib_%d" % rid, rid + RIB_PROPERTIES_OFFSET, "LinearElasticPlaneStress2DLaw",
                                     {"DENSITY": float(rib["material"]["density"]), "YOUNG_MODULUS": float(rib["material"]["youngs_modulus"]),
                                      "POISSON_RATIO": float(rib["material"]["poisson_ratio"]), "THICKNESS": t_rib}))
            property_ids.append(rid + RIB_PROPERTIES_OFFSET)
    mass = float((skin_density_t * face_area).sum()) + float((rib_density_t * rib_area).sum())

    # The elements: the skin's shells on every wetted face, then the ribs'
    # (unloaded) on the rib faces; a surface-load condition on every wetted
    # face, its id the face's order.
    n_skin_shells = len(faces)
    shells = np.concatenate([faces, rib_faces]) if len(rib_faces) else faces
    shell_region = np.concatenate([region_of_face, rib_region]) if len(rib_faces) else region_of_face
    shell_is_rib = np.concatenate([np.zeros(n_skin_shells, dtype=bool), np.ones(len(rib_faces), dtype=bool)])
    shell_prop = np.where(shell_is_rib, shell_region + RIB_PROPERTIES_OFFSET, shell_region + SHELL_PROPERTIES_OFFSET)
    shell_part = ["%s_%d" % ("Rib" if is_rib else "Skin", int(rid)) for rid, is_rib in zip(shell_region, shell_is_rib)]
    # Six degrees of freedom on every node. Known before the solve, so
    # "auto" can choose its solver by them.
    dofs = 6 * len(nodes)
    linear_solver = resolve_linear_solver(solver_opts["linear_solver"], dofs)
    result["linear_solver"] = linear_solver
    loose = len(nodes) - len(np.unique(shells))
    if loose:
        log("  note: %d node(s) belong to no shell (they stay put)" % loose)
    wetted_prop = region_of_face + SHELL_PROPERTIES_OFFSET
    point_prop = property_ids[0]
    t0 = time.time()
    log("  writing the Kratos model: %d nodes, %d shells (%d of them ribs), %d dofs" % (len(nodes), len(shells), len(rib_faces), dofs))
    write_mdpa(mdpa_base + ".mdpa", nodes, shells, shell_prop, shell_part, faces, wetted_prop, support_nodes, point_loads,
               property_ids, solver_opts["shell_element"], point_prop)
    write_materials(materials_path, material_entries)
    seconds_mdpa = time.time() - t0
    parameters = project_parameters(mdpa_base, materials_path, loads, point_loads, echo_level, linear_solver)
    log("  solving %d dofs with %s (%d shells)" % (dofs, linear_solver, len(shells)))
    model, structure, analysis, seconds_solve = run_kratos(job_dir, parameters, len(nodes), face_pressure, echo_level)
    displacement, rotation, reaction = read_nodal(structure, len(nodes))
    n_elements = len(shells)
    log("  post-processing: stresses, margins, the force balance")
    von_mises_elem = element_von_mises(structure, n_elements)

    # The peaks per region: the skin's shells and the ribs', each with the
    # margin against its own allowable.
    def margin(allowable, peak):
        return (allowable / peak - 1.0) if (allowable > 0.0 and peak > 0.0) else 0.0
    per_material = []
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", "")
        sel_s = np.nonzero((shell_region == rid) & ~shell_is_rib)[0]
        sel_r = np.nonzero((shell_region == rid) & shell_is_rib)[0]
        if r.get("skin"):
            a = float(r["skin"]["material"].get("allowable_stress", 0.0))
            peak = float(von_mises_elem[sel_s].max()) if len(sel_s) else 0.0
            per_material.append({"id": rid, "label": "%s skin" % label, "kind": "skin", "max_von_mises": peak, "margin": margin(a, peak), "allowable": a,
                                 "mass": float((skin_density_t[region_of_face == rid] * face_area[region_of_face == rid]).sum())})
        if r.get("rib"):
            a = float(r["rib"]["material"].get("allowable_stress", 0.0))
            peak = float(von_mises_elem[sel_r].max()) if len(sel_r) else 0.0
            per_material.append({"id": rid, "label": "%s ribs" % label, "kind": "rib", "max_von_mises": peak, "margin": margin(a, peak), "allowable": a,
                                 "mass": float((rib_density_t[rib_region == rid] * rib_area[rib_region == rid]).sum())})
    i_vm = int(np.argmax(von_mises_elem)) if n_elements else 0
    result.update({"elements": int(n_elements), "shells": int(len(shells)), "rib_shells": int(len(rib_faces)), "seconds_mdpa": seconds_mdpa})

    # The force balance. Gravity on the mass; the pressure's resultant as
    # -p n A over the outward faces; the point force as given.
    g = float(loads["load_factor"]) * G0
    applied_gravity = np.array([0.0, 0.0, -mass * g])
    applied_pressure = -(face_pressure[:, None] * n_area).sum(axis=0) if len(faces) else np.zeros(3)
    reaction_sum = reaction.sum(axis=0)
    # Equilibrium: the reactions and the applied loads sum to zero. The
    # error is measured against the loads' magnitude -- the gross figure,
    # so a pressure whose resultant cancels over a closed surface still
    # sets a scale, and nothing divides by zero.
    applied_total = applied_gravity + applied_pressure + applied_point
    load_scale = max(float(np.linalg.norm(applied_total)),
                     float(np.linalg.norm(applied_gravity)) + float((np.abs(face_pressure) * face_area).sum()) + sum(float(np.linalg.norm(f)) for _, f in point_loads))
    imbalance = float(np.linalg.norm(reaction_sum + applied_total))
    balance_error = imbalance / load_scale if load_scale > 0.0 else 0.0
    balanced = balance_error <= BALANCE_TOLERANCE
    warnings = []
    if not balanced:
        warnings.append("the reactions miss the applied loads by %.1f%% (|R + F| = %.4g N against loads of %.4g N): the linear solver (%s) did not converge, so the displacements and stresses are not to be trusted. Use the direct solver (sparse_lu)."
                        % (100.0 * balance_error, imbalance, load_scale, linear_solver))
        log("  WARNING: " + warnings[-1])

    # Inward or outward: u.n averaged over the wetted nodes, with the node
    # normal the area-weighted mean of its faces'.
    mean_normal_disp = 0.0
    if len(faces):
        node_normal = np.zeros((len(nodes), 3))
        for k in range(3):
            np.add.at(node_normal, faces[:, k], n_area)
        wet_nodes = np.unique(faces)
        nn = node_normal[wet_nodes]
        nn /= np.maximum(np.linalg.norm(nn, axis=1), 1e-300)[:, None]
        mean_normal_disp = float(np.einsum("ij,ij->i", displacement[wet_nodes], nn).mean())

    # The natural frequencies of the same model, when asked. A failure here
    # is a warning on a static result that stands.
    n_modes = int(solver_opts.get("modes") or 0)
    frequencies, mode_shapes, eigensolver_used, seconds_modes = np.zeros(0), None, "", 0.0
    if n_modes > 0:
        t0 = time.time()
        log("  natural frequencies: the lowest %d modes of the same model" % n_modes)
        try:
            frequencies, mode_shapes, eigensolver_used = run_modes(job_dir, mdpa_base, materials_path, len(nodes), echo_level,
                                                                  n_modes, str(solver_opts.get("eigensolver") or "spectra_sym_g_eigs_shift"))
        except JobError as exc:
            warnings.append(str(exc))
            log("  WARNING: " + warnings[-1])
        seconds_modes = time.time() - t0
    result.update({
        "modes": n_modes,
        "natural_frequencies": [float(f) for f in frequencies],
        "eigensolver": eigensolver_used,
        "seconds_modes": seconds_modes,
    })

    u_mag = np.linalg.norm(displacement, axis=1)
    i_max = int(np.argmax(u_mag))
    result.update({
        "nodes": int(len(nodes)),
        "dofs": int(dofs),
        "mass": mass,
        "max_displacement": float(u_mag[i_max]),
        "max_displacement_at": [float(v) for v in nodes[i_max]],
        "max_von_mises": float(von_mises_elem[i_vm]) if n_elements else 0.0,
        "max_von_mises_material": int(shell_region[i_vm]) if n_elements else 0,
        "per_material": per_material,
        "reaction": [float(v) for v in reaction_sum],
        "applied_gravity": [float(v) for v in applied_gravity],
        "applied_pressure": [float(v) for v in applied_pressure],
        "mean_normal_displacement": mean_normal_disp,
        "balance_error": balance_error,
        "balanced": bool(balanced),
        "load_scale": load_scale,
        "warnings": warnings,
        "seconds_solve": seconds_solve,
        "vtk": outputs["vtk"],
    })

    t_vtk = time.time()
    log("  writing the solution to %s" % outputs["vtk"])
    # kind: 1 a skin shell, 2 a rib shell.
    cell_data = {"material": shell_region.astype(np.int64), "kind": np.where(shell_is_rib, 2, 1).astype(np.int64), "von_mises_element": von_mises_elem}
    point_data = {"displacement": displacement, "rotation": rotation, "von_mises": nodal_average(len(nodes), [shells], von_mises_elem)}
    if mode_shapes is not None:
        for k in range(len(frequencies)):
            point_data["mode_%d" % (k + 1)] = mode_shapes[k]
    write_vtk(outputs["vtk"], nodes, [shells], [VTK_TRIANGLE], point_data, cell_data)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved %d dofs (%d shells, %d of them ribs) with %s in %.2f s: mass %.6g kg, max displacement %.4g m, max von Mises %.4g Pa (region %d); "
        "reaction (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g): %s; wrote %s"
        % (dofs, len(shells), len(rib_faces), linear_solver, result["seconds_solve"], mass, result["max_displacement"], result["max_von_mises"], result["max_von_mises_material"],
           reaction_sum[0], reaction_sum[1], reaction_sum[2], applied_gravity[0], applied_gravity[1], applied_gravity[2],
           applied_pressure[0], applied_pressure[1], applied_pressure[2], applied_point[0], applied_point[1], applied_point[2],
           ("balanced to %.2e" % balance_error) if balanced else ("NOT BALANCED (%.1f%%)" % (100.0 * balance_error)), outputs["vtk"]))
    if n_modes > 0 and len(frequencies):
        log("  natural frequencies (%s, %.2f s): %s Hz" % (eigensolver_used, seconds_modes, ", ".join("%.4g" % f for f in frequencies)))
    return result


def main(argv):
    if len(argv) != 2:
        log("usage: structural_fea.py <job directory>")
        return 2
    job_dir = argv[1]
    try:
        run_job(job_dir)
        return 0
    except JobError as exc:
        tg.write_failure(job_dir, str(exc))
        log("error: %s" % exc)
        return 1
    except Exception as exc:
        tg.write_failure(job_dir, "%s: %s" % (type(exc).__name__, exc))
        log("error: %s: %s" % (type(exc).__name__, exc))
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
