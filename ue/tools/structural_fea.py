"""Linear static finite-element analysis of a skin / core tetrahedral mesh
on KratosMultiphysics' StructuralMechanicsApplication.

Run as a separate process in the WingDesigner micromamba environment
(``pip install KratosMultiphysics KratosStructuralMechanicsApplication
KratosLinearSolversApplication``, all at the same version), the same bridge
pattern as the potential flow (potential_flow.py): the application writes
a job directory, this script reads it and writes ``result.json`` and the
VTK solution the user named.

Everything in the job is SI: metres, kilograms, seconds, pascals, newtons.
The application converts from the project's units before writing the job
and converts the results back for display.

THE MESH. ``mesh.bin`` is the skin / core mesh tools/tet_gmsh.py built
(without its magic): the outer faces of the skin tetrahedra are the
body's own surface triangles, the skin and the core conform, and each
tetrahedron carries a material id. The wetted surface is recovered as the
faces that belong to one tetrahedron only -- with the skin and core
conforming, those are exactly the outer surface.

SECOND ORDER. First-order tetrahedra lock in bending, and a skin one
tetrahedron thick IS a bending problem. ``second_order`` adds a node at
the middle of every edge and solves on 10-node tetrahedra
(SmallDisplacementElement3D10N) with 6-node surface conditions. The
corner nodes, and so the outer surface, are untouched.

LOADS (phase 1). An inertial load factor (the body's own mass under
``load_factor`` x g along -z), a pressure on the wetted faces -- one
number, or one per face from ``pressure_file`` (phase 2's pressure field
from the flow solvers goes there) -- and a point force at the node
nearest a point. POSITIVE_FACE_PRESSURE on a condition acts AGAINST the
face's normal, so with the wetted faces outward a positive pressure
presses inwards, as an external pressure does; the mean of u.n over the
wetted nodes is reported so the sign can be seen to be right.

SUPPORT. Every degree of freedom of the nodes within ``tolerance`` of the
plane ``axis = value`` is fixed; ``value`` may be "min" or "max" for the
mesh's own extreme. The reactions there are summed and reported against
the applied loads: equal and opposite in a converged solution.

RESULTS. Per node the displacement; per element the von Mises stress
(the largest over its integration points), averaged to the nodes for the
VTK; per material the peak von Mises and the margin allowable / peak - 1.
"""

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


# ---------------------------------------------------------------------------
# The mesh
# ---------------------------------------------------------------------------

def read_mesh(path):
    """(nodes float64 (n,3), tets int32 (m,4), material int32 (m,))."""
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
# The Kratos input
# ---------------------------------------------------------------------------

def write_mdpa(path, nodes, elements, faces, material, support_nodes, point_node, material_ids):
    """One ModelPart: the elements under their material's Properties, the
    wetted faces as surface-load conditions, the point load's node as a
    point-load condition, and the sub model parts the parameters refer to
    (Material_<id> per material, Support, Wetted, PointLoad)."""
    n_elem_nodes = elements.shape[1]
    n_face_nodes = faces.shape[1]
    element_name = "SmallDisplacementElement3D%dN" % n_elem_nodes
    face_name = "SurfaceLoadCondition3D%dN" % n_face_nodes
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("Begin ModelPartData\nEnd ModelPartData\n\n")
        for mid in material_ids:
            w("Begin Properties %d\nEnd Properties\n" % mid)
        w("\nBegin Nodes\n")
        for i, p in enumerate(nodes):
            w("%d %.17g %.17g %.17g\n" % (i + 1, p[0], p[1], p[2]))
        w("End Nodes\n\n")
        w("Begin Elements %s\n" % element_name)
        fmt = "%d %d " + " ".join(["%d"] * n_elem_nodes) + "\n"
        for i, (e, m) in enumerate(zip(elements, material)):
            w(fmt % ((i + 1, int(m)) + tuple(int(v) + 1 for v in e)))
        w("End Elements\n\n")
        cond_base = 0
        w("Begin Conditions %s\n" % face_name)
        fmt = "%d %d " + " ".join(["%d"] * n_face_nodes) + "\n"
        for i, fc in enumerate(faces):
            w(fmt % ((i + 1, int(material_ids[0])) + tuple(int(v) + 1 for v in fc)))
        w("End Conditions\n\n")
        cond_base = len(faces)
        if point_node is not None:
            w("Begin Conditions PointLoadCondition3D1N\n%d %d %d\nEnd Conditions\n\n" % (cond_base + 1, int(material_ids[0]), point_node + 1))

        for mid in material_ids:
            sel = np.nonzero(material == mid)[0]
            used = np.unique(elements[sel])
            w("Begin SubModelPart Material_%d\n" % mid)
            w("Begin SubModelPartNodes\n")
            for v in used:
                w("%d\n" % (int(v) + 1))
            w("End SubModelPartNodes\nBegin SubModelPartElements\n")
            for i in sel:
                w("%d\n" % (int(i) + 1))
            w("End SubModelPartElements\nBegin SubModelPartConditions\nEnd SubModelPartConditions\nEnd SubModelPart\n\n")

        w("Begin SubModelPart Support\nBegin SubModelPartNodes\n")
        for v in support_nodes:
            w("%d\n" % (int(v) + 1))
        w("End SubModelPartNodes\nBegin SubModelPartElements\nEnd SubModelPartElements\n"
          "Begin SubModelPartConditions\nEnd SubModelPartConditions\nEnd SubModelPart\n\n")

        w("Begin SubModelPart Wetted\nBegin SubModelPartNodes\n")
        for v in np.unique(faces):
            w("%d\n" % (int(v) + 1))
        w("End SubModelPartNodes\nBegin SubModelPartElements\nEnd SubModelPartElements\nBegin SubModelPartConditions\n")
        for i in range(len(faces)):
            w("%d\n" % (i + 1))
        w("End SubModelPartConditions\nEnd SubModelPart\n\n")

        if point_node is not None:
            w("Begin SubModelPart PointLoad\nBegin SubModelPartNodes\n%d\nEnd SubModelPartNodes\n"
              "Begin SubModelPartElements\nEnd SubModelPartElements\n"
              "Begin SubModelPartConditions\n%d\nEnd SubModelPartConditions\nEnd SubModelPart\n\n" % (point_node + 1, cond_base + 1))


def write_materials(path, materials):
    props = []
    for m in materials:
        props.append({
            "model_part_name": "Structure.Material_%d" % int(m["id"]),
            "properties_id": int(m["id"]),
            "Material": {
                "constitutive_law": {"name": "LinearElastic3DLaw"},
                "Variables": {
                    "DENSITY": float(m["density"]),
                    "YOUNG_MODULUS": float(m["youngs_modulus"]),
                    "POISSON_RATIO": float(m["poisson_ratio"]),
                },
                "Tables": {},
            },
        })
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"properties": props}, f, indent=1)


def project_parameters(mdpa_base, materials_path, loads, has_point, echo_level):
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
            "rotation_dofs": False,
            "compute_reactions": True,
            "block_builder": True,
            "linear_solver_settings": {
                "solver_type": "amgcl",
                "tolerance": 1e-9,
                "max_iteration": 3000,
                "krylov_type": "cg",
                "smoother_type": "ilu0",
                "coarsening_type": "aggregation",
                "scaling": False,
                "verbosity": 0,
            },
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

def solve(job_dir, nodes, elements, faces, material, support_nodes, point_node, materials, loads, face_pressure, echo_level):
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    from KratosMultiphysics.StructuralMechanicsApplication.structural_mechanics_analysis import StructuralMechanicsAnalysis

    material_ids = [int(m["id"]) for m in materials]
    mdpa_base = os.path.join(job_dir, "structure")
    materials_path = os.path.join(job_dir, "materials.json")
    t0 = time.time()
    write_mdpa(mdpa_base + ".mdpa", nodes, elements, faces, material, support_nodes, point_node, material_ids)
    write_materials(materials_path, materials)
    seconds_write = time.time() - t0

    if echo_level == 0:
        KM.Logger.GetDefaultOutput().SetSeverity(KM.Logger.Severity.WARNING)
    parameters = KM.Parameters(json.dumps(project_parameters(mdpa_base, materials_path, loads, point_node is not None, echo_level)))
    model = KM.Model()
    t_solve = time.time()
    analysis = StructuralMechanicsAnalysis(model, parameters)
    analysis.Initialize()
    structure = model["Structure"]
    if structure.NumberOfNodes() != len(nodes):
        raise JobError("Kratos read %d of %d nodes." % (structure.NumberOfNodes(), len(nodes)))
    # The pressure, per wetted face: one mechanism whether it is a constant
    # or a field, set on the conditions after the model is in.
    wetted = structure.GetSubModelPart("Wetted")
    for cond in wetted.Conditions:
        cond.SetValue(KM.POSITIVE_FACE_PRESSURE, float(face_pressure[cond.Id - 1]))
    analysis.RunSolutionLoop()
    analysis.Finalize()
    seconds_solve = time.time() - t_solve

    n = len(nodes)
    displacement = np.zeros((n, 3))
    reaction = np.zeros((n, 3))
    for node in structure.Nodes:
        i = node.Id - 1
        u = node.GetSolutionStepValue(KM.DISPLACEMENT)
        displacement[i] = (u[0], u[1], u[2])
    # The reactions at the support. Read from the Support sub model part
    # rather than by IsFixed(): the constraint process frees the degrees
    # of freedom again once the step is over, so nothing is fixed by now.
    for node in structure.GetSubModelPart("Support").Nodes:
        r = node.GetSolutionStepValue(KM.REACTION)
        reaction[node.Id - 1] = (r[0], r[1], r[2])
    process_info = structure.ProcessInfo
    von_mises = np.zeros(len(elements))
    for elem in structure.Elements:
        values = elem.CalculateOnIntegrationPoints(SMA.VON_MISES_STRESS, process_info)
        von_mises[elem.Id - 1] = max(float(v) for v in values) if len(values) else 0.0
    dofs = 3 * n
    return displacement, reaction, von_mises, dofs, {"seconds_mdpa": seconds_write, "seconds_solve": seconds_solve}


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_vtk(path, nodes, elements, material, displacement, von_mises_elem, von_mises_node):
    n_elem_nodes = elements.shape[1]
    cell_type = 24 if n_elem_nodes == 10 else 10   # VTK_QUADRATIC_TETRA / VTK_TETRA; same node order as Kratos
    with open(path, "w", encoding="utf-8") as f:
        f.write("# vtk DataFile Version 3.0\nAerotelic structural analysis (SI: m, Pa)\nASCII\nDATASET UNSTRUCTURED_GRID\n")
        f.write("POINTS %d double\n" % len(nodes))
        np.savetxt(f, nodes, fmt="%.9g %.9g %.9g")
        f.write("CELLS %d %d\n" % (len(elements), len(elements) * (n_elem_nodes + 1)))
        np.savetxt(f, np.concatenate([np.full((len(elements), 1), n_elem_nodes), elements], axis=1), fmt="%d")
        f.write("CELL_TYPES %d\n" % len(elements))
        np.savetxt(f, np.full(len(elements), cell_type), fmt="%d")
        f.write("POINT_DATA %d\n" % len(nodes))
        f.write("VECTORS displacement double\n")
        np.savetxt(f, displacement, fmt="%.9g %.9g %.9g")
        f.write("SCALARS von_mises double 1\nLOOKUP_TABLE default\n")
        np.savetxt(f, von_mises_node, fmt="%.9g")
        f.write("CELL_DATA %d\n" % len(elements))
        f.write("SCALARS material int 1\nLOOKUP_TABLE default\n")
        np.savetxt(f, material, fmt="%d")
        f.write("SCALARS von_mises_element double 1\nLOOKUP_TABLE default\n")
        np.savetxt(f, von_mises_elem, fmt="%.9g")


# ---------------------------------------------------------------------------
# The job
# ---------------------------------------------------------------------------

def run_job(job_dir):
    """job.json:

        mesh          the skin / core mesh (read_mesh), SI
        materials     [{"id", "label", "name", "density", "youngs_modulus",
                      "poisson_ratio", "allowable_stress"}], one per
                      material id in the mesh, SI
        second_order  10-node tetrahedra (default true)
        support       {"axis": "x"|"y"|"z", "value": number | "min" |
                      "max", "tolerance": distance (0: 1e-6 of the diagonal)}
        loads         {"load_factor": g's along -z, "pressure": pascals on
                      every wetted face, "pressure_file": float64 per face
                      overriding it, "point_force": {"at": [x,y,z],
                      "force": [fx,fy,fz]} or null}
        solver        {"echo_level"}
        outputs       {"vtk": the solution file, "result": the statistics}
    """
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job["outputs"]
    echo_level = int(job.get("solver", {}).get("echo_level", 0))
    loads = {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None, "point_force": None}
    loads.update(job.get("loads", {}))
    support = {"axis": "y", "value": 0.0, "tolerance": 0.0}
    support.update(job.get("support", {}))

    nodes, tets, material = read_mesh(os.path.join(job_dir, job["mesh"]))
    tets, flipped = orient(nodes, tets)
    materials = job["materials"]
    ids = sorted({int(m["id"]) for m in materials})
    present = sorted(set(int(v) for v in np.unique(material)))
    missing = [m for m in present if m not in ids]
    if missing:
        raise JobError("the mesh has tetrahedra of material %s, which the job gives no material for." % missing)
    for m in materials:
        if not (float(m["youngs_modulus"]) > 0.0 and float(m["density"]) > 0.0):
            raise JobError("'%s' has no stiffness or no density." % m.get("label", m["id"]))
        nu = float(m["poisson_ratio"])
        if not (0.0 <= nu < 0.5):
            raise JobError("'%s' has a Poisson's ratio of %g; it must be in [0, 0.5)." % (m.get("label", m["id"]), nu))

    faces, owners = boundary_faces(tets)
    if len(faces) == 0:
        raise JobError("the mesh has no boundary faces.")
    tet_vol = signed_volumes(nodes, tets)
    wetted_vol = tg.surface_volume(nodes, faces)
    log("  %d nodes, %d tetrahedra (%d flipped), %d wetted faces enclosing %.6g (tetrahedra %.6g)"
        % (len(nodes), len(tets), flipped, len(faces), wetted_vol, tet_vol.sum()))

    # Mass, from the first-order volumes (exact for the material model).
    density = np.zeros(len(tets))
    for m in materials:
        density[material == int(m["id"])] = float(m["density"])
    mass = float((density * tet_vol).sum())

    n_corner = len(nodes)
    use_second = bool(job.get("second_order", True))
    if use_second:
        nodes, elements, faces_out = second_order(nodes, tets, faces)
    else:
        elements, faces_out = tets, faces

    # The support.
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

    # The pressure, per wetted face.
    if loads.get("pressure_file"):
        face_pressure = np.fromfile(os.path.join(job_dir, loads["pressure_file"]), dtype="<f8")
        if len(face_pressure) != len(faces):
            raise JobError("the pressure file has %d values for %d wetted faces." % (len(face_pressure), len(faces)))
    else:
        face_pressure = np.full(len(faces), float(loads["pressure"]))

    # The point force: the nearest corner node (the mid-edge ones, if any,
    # come after the corners and are not candidates).
    point_node = None
    applied_point = np.zeros(3)
    if loads.get("point_force"):
        at = np.array(loads["point_force"]["at"], dtype=np.float64)
        point_node = int(np.argmin(np.linalg.norm(nodes[:n_corner] - at, axis=1)))
        applied_point = np.array(loads["point_force"]["force"], dtype=np.float64)

    displacement, reaction, von_mises_elem, dofs, solve_stats = solve(
        job_dir, nodes, elements, faces_out, material, support_nodes, point_node, materials, loads, face_pressure, echo_level)

    # The force balance. Gravity on the mass; the pressure's resultant as
    # -p n A over the outward faces; the point force as given.
    g = float(loads["load_factor"]) * G0
    applied_gravity = np.array([0.0, 0.0, -mass * g])
    a, b, c = (nodes[faces[:, i]] for i in range(3))
    n_area = 0.5 * np.cross(b - a, c - a)             # outward normal times area
    applied_pressure = -(face_pressure[:, None] * n_area).sum(axis=0)
    reaction_sum = reaction.sum(axis=0)

    # Nodal von Mises for the picture: the mean over the elements at a node.
    von_mises_node = np.zeros(len(nodes))
    count = np.zeros(len(nodes))
    for k in range(elements.shape[1]):
        np.add.at(von_mises_node, elements[:, k], von_mises_elem)
        np.add.at(count, elements[:, k], 1.0)
    von_mises_node /= np.maximum(count, 1.0)

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
    per_material = []
    for m in materials:
        mid = int(m["id"])
        sel = material == mid
        peak = float(von_mises_elem[sel].max()) if sel.any() else 0.0
        allowable = float(m.get("allowable_stress", 0.0))
        per_material.append({
            "id": mid, "label": m.get("label", ""), "max_von_mises": peak,
            "margin": (allowable / peak - 1.0) if (allowable > 0.0 and peak > 0.0) else 0.0,
            "mass": float((tet_vol[sel] * float(m["density"])).sum()),
        })
    i_vm = int(np.argmax(von_mises_elem))

    result = {
        "ok": True,
        "nodes": int(len(nodes)),
        "elements": int(len(elements)),
        "dofs": int(dofs),
        "second_order": use_second,
        "support_nodes": int(len(support_nodes)),
        "support_plane": {"axis": "xyz"[axis], "value": value, "tolerance": tolerance},
        "wetted_faces": int(len(faces)),
        "flipped": flipped,
        "mass": mass,
        "max_displacement": float(u_mag[i_max]),
        "max_displacement_at": [float(v) for v in nodes[i_max]],
        "max_von_mises": float(von_mises_elem[i_vm]),
        "max_von_mises_material": int(material[i_vm]),
        "per_material": per_material,
        "reaction": [float(v) for v in reaction_sum],
        "applied_gravity": [float(v) for v in applied_gravity],
        "applied_pressure": [float(v) for v in applied_pressure],
        "applied_point": [float(v) for v in applied_point],
        "mean_normal_displacement": mean_normal_disp,
        "vtk": outputs["vtk"],
    }
    result.update(solve_stats)

    t_vtk = time.time()
    write_vtk(outputs["vtk"], nodes, elements, material, displacement, von_mises_elem, von_mises_node)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved %d dofs in %.2f s: mass %.6g kg, max displacement %.4g m, max von Mises %.4g Pa (material %d); "
        "reaction (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g); wrote %s"
        % (dofs, result["seconds_solve"], mass, result["max_displacement"], result["max_von_mises"], result["max_von_mises_material"],
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
