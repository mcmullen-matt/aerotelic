"""Linear static finite-element analysis of scene bodies -- thin-walled,
solid, or lattice-filled -- on KratosMultiphysics'
StructuralMechanicsApplication.

THE MODEL (2026-10-31) is a set of REGIONS, each what a body's material
properties say it is: a SKIN (shells of a thickness on the region's wetted
faces), a homogenised lattice CORE on its tetrahedra, a SOLID of a
material on its tetrahedra, or a skin with either. A region with a skin
alone is a thin-walled body of shells and no tetrahedra at all (its faces
come in ``faces.bin``). The two earlier job forms below are still read
(``_regions_from_job``).

THE SHELL ELEMENT is the LINEAR thin triangle, ShellThinElement3D3N (since
2026-10-31, later that day). The corotational variants are geometrically
nonlinear: solved in the one step of a linear analysis, their internal
forces are evaluated on the displaced configuration while the step was
linearised on the undeformed one, so once the displacements are large the
reactions no longer balance the loads and the stresses are nonsense (a
5 mm skin-only P-38 airframe under a 5 kN tip load moves 2.7 m: the
corotational thin shell reported reactions of (-10, -11, 7) kN against a
5 kN load and 8.6e10 Pa; the linear one (0.008, 0.000, 5000) N and beam
theory's stress along the wing). For the millimetre displacements of the
checks below the two agree. The corotational thin and thick triangles
remain ``shell_element`` options.

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
``value`` may be "min" or "max" for the mesh's own extreme. With 10-node
tetrahedra the mid-edge nodes on the plane are fixed as well (the result
reports ``support_nodes``, all of them, and ``support_corner_nodes``). The
reactions there are summed and reported against the applied loads: equal
and opposite in a converged solution.

RESULTS. Per node the displacement; per element the von Mises stress
(the largest over its integration points -- for a shell over its top,
middle and bottom surfaces), averaged to the nodes for the VTK; per
region the peak von Mises of the skin and of the core and the margin
allowable / peak - 1. The core's von Mises is a MACRO stress, the average
over a cell: the struts carry more. So the core also reports the
estimated MICRO stress -- the peak voxel von Mises of each element's own
strain state over the cell's critical voxels (homogenize.py) -- and its
margin is against the base material's allowable using that.

ELEMENT ORDER PER REGION (2026-11-05). ``second_order`` on a region says
whether ITS tetrahedra are promoted to 10 nodes (null: the job's flag).
A homogenised core carries a material card, not a bending problem -- the
shells on its faces do the bending -- so the application leaves it at
4 nodes, while a solid body or the full-resolution skin tetrahedra (one
tetrahedron thick, locking at first order) take 10. The regions are
independent bodies, so an order per region costs nothing; within one
body every tetrahedron has the same order. Found the hard way: a
24,571-node fuselage core promoted to 10 nodes by default was 540,000
degrees of freedom on the direct solver, which exhausted a 32 GB desktop
and its page file after eight minutes.

NATURAL FREQUENCIES (2026-11-04). With ``solver.modes`` > 0 the same model
-- the same mesh, materials and clamps, no loads -- is solved a second
time as the generalised eigenvalue problem K phi = omega^2 M phi on
StructuralMechanicsApplication's ``eigen_value`` solver (the dynamic
scheme: K the stiffness the static solve used, M the elements' mass
matrices from the same densities and shell thicknesses, so the
frequencies are of the part as the static analysis has it, overlap
correction included). The eigensolver is LinearSolversApplication's
Spectra shift-invert (``spectra_sym_g_eigs_shift``, the lowest modes
first) with the subspace solver ``eigen_eigensystem`` as the fallback. The
result lists ``natural_frequencies`` in hertz, lowest first, and the VTK
carries each mode shape as the point vector ``mode_k`` scaled to a unit
largest displacement. Measured on the 1 x 10 x 1 m thin-walled box beam
of structural_fea_check.py: the degenerate first bending pair at 10.81 Hz
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

# "auto" takes the direct solver (sparse_lu) up to this many degrees of
# freedom and amgcl beyond. The direct solver is exact; amgcl (cg + ilu0)
# can stop short on the stiffness contrasts of thin shells and give a
# solution that LOOKS fine -- small displacements, low stresses -- with
# reactions nowhere near the loads (the hershey_bar wing, 2026-11-01:
# 11,880 dofs, 75 s to 0.1 mm and reactions of 3 kN against a 1 kN load;
# sparse_lu took 0.55 s and matched beam theory to 0.2%). The threshold
# is a desktop's: Eigen's SparseLU runs on one thread and its factors
# grow faster than the system (a 64,056-dof sphere took two minutes; a
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

def read_mesh(path):
    """(nodes float64 (n,3), tets int64 (m,4), region int64 (m,))."""
    with open(path, "rb") as f:
        nn, nt = struct.unpack("<II", f.read(8))
        nodes = np.frombuffer(f.read(nn * 24), dtype="<f8").reshape(nn, 3)
        tets = np.frombuffer(f.read(nt * 16), dtype="<i4").reshape(nt, 4)
        material = np.frombuffer(f.read(nt * 4), dtype="<i4")
    if nt and (tets.min() < 0 or tets.max() >= nn):
        raise JobError("the mesh refers to nodes it does not have.")
    return (np.ascontiguousarray(nodes, dtype=np.float64), np.ascontiguousarray(tets, dtype=np.int64).reshape(nt, 4),
            np.ascontiguousarray(material, dtype=np.int64))


def read_faces(path):
    """The shell-only regions' faces: (faces int64 (k,3) outward, region
    int64 (k,)). A count, int32 triples, then an int32 region per face."""
    with open(path, "rb") as f:
        (nf,) = struct.unpack("<I", f.read(4))
        faces = np.frombuffer(f.read(nf * 12), dtype="<i4").reshape(nf, 3)
        region = np.frombuffer(f.read(nf * 4), dtype="<i4")
    return np.ascontiguousarray(faces, dtype=np.int64), np.ascontiguousarray(region, dtype=np.int64)


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


def write_mdpa(path, nodes, tet_blocks, cond_blocks, shells, shell_region, support_nodes, point_loads, property_ids, shell_element, point_prop):
    """The model: the tetrahedra under their region's Properties (id =
    region) -- `tet_blocks` a list of (connectivity rows, region per row),
    4-node and 10-node blocks laid end to end, element ids counting on
    through the blocks -- a shell element on every wetted face of a region
    with a skin under the region's skin Properties (id = region +
    SHELL_PROPERTIES_OFFSET), the wetted faces as surface-load conditions
    (`cond_blocks`: a list of (faces, property per face, 0-based condition
    id per face), 3-node and 6-node blocks whose ids together are the
    wetted faces' order, so a pressure per face maps by id), the point
    load's node as a point-load condition, and the sub model parts the
    materials and the parameters refer to (Solid_<id>, Skin_<id>, Support,
    Wetted, PointLoad)."""
    nt = sum(len(b[0]) for b in tet_blocks)
    n_cond = sum(len(b[0]) for b in cond_blocks)
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("Begin ModelPartData\nEnd ModelPartData\n\n")
        for pid in property_ids:
            w("Begin Properties %d\nEnd Properties\n" % int(pid))
        w("\nBegin Nodes\n")
        for i, p in enumerate(nodes):
            w("%d %.17g %.17g %.17g\n" % (i + 1, p[0], p[1], p[2]))
        w("End Nodes\n\n")
        first = 0
        for tets_b, region_b in tet_blocks:
            if len(tets_b):
                _write_block(w, "Elements", "SmallDisplacementElement3D%dN" % tets_b.shape[1], tets_b, np.arange(first, first + len(tets_b)), region_b)
            first += len(tets_b)
        if len(shells):
            _write_block(w, "Elements", shell_element, shells, np.arange(nt, nt + len(shells)), shell_region + SHELL_PROPERTIES_OFFSET)
        wetted_nodes = []
        for faces_b, prop_b, ids_b in cond_blocks:
            if len(faces_b):
                _write_block(w, "Conditions", "SurfaceLoadCondition3D%dN" % faces_b.shape[1], faces_b, ids_b, prop_b)
                wetted_nodes.append(np.unique(faces_b))
        if point_loads:
            w("Begin Conditions PointLoadCondition3D1N\n")
            for i, (node, _) in enumerate(point_loads):
                w("%d %d %d\n" % (n_cond + 1 + i, int(point_prop), int(node) + 1))
            w("End Conditions\n\n")
        # The solids' sub model parts: every node and element of the region
        # over the blocks, the element ids counting on through them.
        region_all = np.concatenate([b[1] for b in tet_blocks]) if tet_blocks else np.zeros(0, dtype=np.int64)
        for rid in sorted(set(int(v) for v in region_all)):
            sel = np.nonzero(region_all == rid)[0]
            used = []
            start = 0
            for tets_b, region_b in tet_blocks:
                sel_b = np.nonzero(region_b == rid)[0]
                if len(sel_b):
                    used.append(np.unique(tets_b[sel_b]))
                start += len(tets_b)
            _write_sub_model_part(w, "Solid_%d" % rid, np.unique(np.concatenate(used)) if used else [], sel)
        for rid in sorted(set(int(v) for v in shell_region)):
            sel = np.nonzero(shell_region == rid)[0]
            _write_sub_model_part(w, "Skin_%d" % rid, np.unique(shells[sel]), sel + nt)
        _write_sub_model_part(w, "Support", support_nodes)
        _write_sub_model_part(w, "Wetted", np.unique(np.concatenate(wetted_nodes)) if wetted_nodes else [], (), range(n_cond))
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


def _constraint_processes(rotation_dofs):
    """The clamp at the support, shared by the static and the modal solve."""
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
    return constraints


def modal_parameters(mdpa_base, materials_path, echo_level, rotation_dofs, count, eigensolver):
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
            "rotation_dofs": bool(rotation_dofs),
            "block_builder": True,
            "eigensolver_settings": settings,
        },
        "processes": {
            "constraints_process_list": _constraint_processes(rotation_dofs),
            "loads_process_list": [],
            "list_other_processes": [],
        },
        "output_processes": {},
    }


def project_parameters(mdpa_base, materials_path, loads, point_loads, echo_level, rotation_dofs, linear_solver):
    constraints = _constraint_processes(rotation_dofs)
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


def run_modes(job_dir, mdpa_base, materials_path, n_nodes, echo_level, rotation_dofs, count, eigensolver):
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
    # displacements' columns; a solid node has the three displacements only.
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
                modal_parameters(mdpa_base, materials_path, echo_level, rotation_dofs, count, solver))))
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
        if elem.GetGeometry().PointsNumber() not in (4, 10):
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
    """The regions in the job's own form, from the current ``regions`` form
    or either of the two older ones (``model`` "solid" with ``materials``,
    "shell_core" with skin + core regions), with the second-order flag."""
    model = str(job.get("model", "regions"))
    if model == "solid":
        regions = [{"id": int(m["id"]), "label": m.get("label", ""), "skin": None, "core": None, "solid": {"material": m}, "second_order": None}
                   for m in job["materials"]]
        return regions, bool(job.get("second_order", True))
    if model in ("shell_core", "regions"):
        regions = [{"id": int(r["id"]), "label": r.get("label", ""), "skin": r.get("skin"), "core": r.get("core"), "solid": r.get("solid"),
                    # Per region (2026-11-05): null takes the job's flag.
                    "second_order": r.get("second_order")}
                   for r in job.get("regions", [])]
        return regions, bool(job.get("second_order", False))
    raise JobError("the model must be regions, shell_core or solid, not '%s'." % model)


def run_job(job_dir):
    """job.json:

        mesh          the mesh (read_mesh), SI, a region id per tetrahedron;
                      no tetrahedra at all is allowed when every region is
                      shells
        faces         null, or a file of the shell-only regions' faces
                      (read_faces): wetted faces that belong to no
                      tetrahedron, with their region id
        regions       [{"id", "label",
                        "skin": null | {"material": {...}, "thickness": m},
                        "core": null | {"material": {... the lattice's BASE
                                 material ...},
                                 "cell": {"file", "nx", "ny", "nz", "period"},
                                 "density_correction": true},
                        "solid": null | {"material": {...}},
                        "second_order": null | true | false}],
                      one per region id in the mesh and the faces: shells of
                      the skin's thickness on the region's wetted faces when
                      it has a skin; its tetrahedra a homogenised lattice
                      (core) or a solid of the material (solid); a region
                      with a skin alone is a thin-walled body of shells. A
                      material is {"name", "density", "youngs_modulus",
                      "poisson_ratio", "allowable_stress"}, SI. A region's
                      own "second_order" says whether ITS tetrahedra are
                      10-node (null: the job's flag below).
        second_order  10-node tetrahedra for the regions that do not say
                      (default false; the shells stay 3-node on the corner
                      nodes)
        model         "regions" (the form above; also read: "solid" with a
                      "materials" list, "shell_core" with skin + core
                      regions -- the two earlier forms)
        supports      [{"axis": "x"|"y"|"z", "value": number | "min" |
                      "max", "tolerance": distance (0: 1e-4 of the
                      diagonal), "bounds": null | [u0, u1, w0, w1]}], every
                      plane clamping its nodes -- within the rectangle
                      along the plane's in-plane axes ((axis + 1) % 3,
                      (axis + 2) % 3) when bounds are given (the older
                      single "support" is read too)
        loads         {"load_factor": g's along -z, "pressure": pascals on
                      every wetted face, "pressure_file": float64 per face
                      overriding it (the tetrahedra's boundary faces in
                      np.unique order, then the shell-only faces as given),
                      "point_forces": [{"at": [x,y,z], "force": [fx,fy,fz]}]
                      (the older single "point_force" is read too)}
        solver        {"echo_level",
                      "linear_solver": "auto" (the default: sparse_lu up to
                      AUTO_DIRECT_DOFS degrees of freedom, amgcl beyond) |
                      "amgcl" | "sparse_lu",
                      "homogenize_samples": 2000, "homogenize_cache": true,
                      "shell_element": "ShellThinElement3D3N" (the linear
                      thin shell; or ShellThinElementCorotational3D3N /
                      ShellThickElementCorotational3D3N, see above),
                      "modes": 0 (how many natural frequencies to compute
                      after the static solve; 0: none),
                      "eigensolver": "spectra_sym_g_eigs_shift" (or
                      "eigen_eigensystem"; either falls back to the other)}
        outputs       {"vtk": the solution file, "result": the statistics}

    The result reports the solver used ("linear_solver"), the force
    balance -- "balance_error" = |sum of reactions + applied loads| over
    the loads' magnitude, "balanced" when it is within BALANCE_TOLERANCE
    -- and a "warnings" list; an unbalanced solve is a solver that did not
    converge, and its fields are not to be trusted. With modes > 0 it
    also reports "natural_frequencies" (Hz, lowest first; empty with a
    warning if the eigensolve failed), "eigensolver" and "seconds_modes",
    and the VTK carries the mode shapes as "mode_1" ... point vectors.
    """
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job["outputs"]
    solver_opts = {"echo_level": 0, "linear_solver": "auto", "homogenize_samples": 2000, "homogenize_cache": True,
                   "shell_element": "ShellThinElement3D3N", "modes": 0, "eigensolver": "spectra_sym_g_eigs_shift"}
    solver_opts.update(job.get("solver", {}))
    echo_level = int(solver_opts["echo_level"])
    loads = {"load_factor": 0.0, "pressure": 0.0, "pressure_file": None, "point_force": None, "point_forces": None}
    loads.update(job.get("loads", {}))
    # Any number of point forces (the older single "point_force" is read too).
    point_forces = list(loads.get("point_forces") or [])
    if loads.get("point_force"):
        point_forces.append(loads["point_force"])
    # Any number of support planes (the older single "support" is read too).
    supports = list(job.get("supports") or [])
    if job.get("support"):
        supports.append(job["support"])
    if not supports:
        supports = [{"axis": "y", "value": 0.0, "tolerance": 0.0}]
    regions, use_second = _regions_from_job(job)

    # The mesh: the tetrahedra's boundary faces (np.unique order), then the
    # shell-only regions' faces as given.
    nodes, tets, region = read_mesh(os.path.join(job_dir, job["mesh"]))
    # The order of each region's tetrahedra: its own say, else the job's.
    # The tetrahedra are then sorted so the 4-node ones come first and the
    # 10-node ones after, which keeps every per-element array in element
    # id order with no permutation to carry about (the boundary faces'
    # order, np.unique's, does not depend on the tetrahedra's).
    second_of_region = {int(r["id"]): (bool(r["second_order"]) if r.get("second_order") is not None else use_second) for r in regions}
    if len(tets):
        tets, flipped = orient(nodes, tets)
        is_second = np.array([second_of_region.get(int(rid), use_second) for rid in region], dtype=bool)
        perm = np.argsort(is_second, kind="stable")
        tets, region, is_second = tets[perm], region[perm], is_second[perm]
        n_tet4 = int((~is_second).sum())
        tet_faces, owners = boundary_faces(tets)
        region_of_tet_face = region[owners]
        face_second = owners >= n_tet4
    else:
        flipped = 0
        n_tet4 = 0
        tet_faces = np.zeros((0, 3), dtype=np.int64)
        region_of_tet_face = np.zeros(0, dtype=np.int64)
        face_second = np.zeros(0, dtype=bool)
    log("  mesh read: %d nodes, %d tetrahedra" % (len(nodes), len(tets)))
    if job.get("faces"):
        extra_faces, extra_region = read_faces(os.path.join(job_dir, job["faces"]))
        if len(extra_faces) and (extra_faces.min() < 0 or extra_faces.max() >= len(nodes)):
            raise JobError("the shell faces refer to nodes the mesh does not have.")
    else:
        extra_faces = np.zeros((0, 3), dtype=np.int64)
        extra_region = np.zeros(0, dtype=np.int64)
    faces = np.concatenate([tet_faces, extra_faces])
    region_of_face = np.concatenate([region_of_tet_face, extra_region])
    n_tet_faces = len(tet_faces)
    if len(faces) == 0:
        raise JobError("the mesh has no wetted faces.")
    tet_vol = signed_volumes(nodes, tets) if len(tets) else np.zeros(0)
    n_area = face_normals_area(nodes, faces)
    face_area = np.linalg.norm(n_area, axis=1)
    present_tets = sorted(set(int(v) for v in np.unique(region))) if len(region) else []
    present_faces = sorted(set(int(v) for v in np.unique(region_of_face)))
    log("  %d nodes, %d tetrahedra (%d flipped), %d wetted faces (%d of shell-only bodies); regions %s"
        % (len(nodes), len(tets), flipped, len(faces), len(extra_faces), sorted(set(present_tets) | set(present_faces))))

    support_nodes, support_planes = _support_nodes(nodes, supports)

    # The pressure, per wetted face.
    if loads.get("pressure_file"):
        face_pressure = np.fromfile(os.path.join(job_dir, loads["pressure_file"]), dtype="<f8")
        if len(face_pressure) != len(faces):
            raise JobError("the pressure file has %d values for %d wetted faces." % (len(face_pressure), len(faces)))
    else:
        face_pressure = np.full(len(faces), float(loads["pressure"]))

    # The point forces: each at the nearest corner node.
    point_loads = []   # (node, force)
    applied_point = np.zeros(3)
    for pf in point_forces:
        at = np.array(pf["at"], dtype=np.float64)
        node = int(np.argmin(np.linalg.norm(nodes - at, axis=1)))
        force = np.array(pf["force"], dtype=np.float64)
        point_loads.append((node, force))
        applied_point += force

    # The regions against the mesh: every id present has one, and each
    # region is what its tetrahedra and faces need.
    by_id = {int(r["id"]): r for r in regions}
    missing = [m for m in sorted(set(present_tets) | set(present_faces)) if m not in by_id]
    if missing:
        raise JobError("the mesh has tetrahedra or faces of region %s, which the job gives nothing for." % missing)
    ids = sorted(by_id)
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", rid)
        has_tets = rid in present_tets
        has_faces = rid in present_faces
        if r.get("skin"):
            _check_material(r["skin"]["material"], "%s skin" % label)
            if not float(r["skin"]["thickness"]) > 0.0:
                raise JobError("'%s' has no skin thickness." % label)
            if not has_faces:
                raise JobError("'%s' has a skin but no wetted face to put its shells on." % label)
        if r.get("core") and r.get("solid"):
            raise JobError("'%s' is both a homogenised core and a solid." % label)
        if r.get("core"):
            _check_material(r["core"]["material"], "%s core" % label)
        if r.get("solid"):
            _check_material(r["solid"]["material"], "%s" % label)
        if has_tets and not (r.get("core") or r.get("solid")):
            raise JobError("'%s' has tetrahedra but says neither what solid nor what lattice they are." % label)
        if (r.get("core") or r.get("solid")) and not has_tets:
            raise JobError("'%s' is a solid or a core but has no tetrahedra." % label)
        if not (r.get("skin") or r.get("core") or r.get("solid")):
            raise JobError("'%s' is nothing: no skin, no core, no solid." % label)
    n_tet10 = len(tets) - n_tet4
    second_ids = sorted(rid for rid in ids if rid in present_tets and second_of_region[rid])
    use_second = n_tet10 > 0

    mdpa_base = os.path.join(job_dir, "structure")
    materials_path = os.path.join(job_dir, "materials.json")
    result = {
        # The model name as the job gave it (the older forms' callers read it back).
        "ok": True, "model": str(job.get("model", "regions")), "flipped": flipped, "wetted_faces": int(len(faces)),
        "support_nodes": int(len(support_nodes)), "support_planes": support_planes,
        "point_nodes": [int(n) for n, _ in point_loads],
        "applied_point": [float(v) for v in applied_point],
    }

    # The homogenised cores, and every region's densities.
    t0 = time.time()
    if any(r.get("core") for r in regions):
        log("  homogenising the lattice cores")
    homog = {}
    material_entries = []
    property_ids = []
    tet_density = np.zeros(len(tets))
    skin_density_t = np.zeros(len(faces))     # rho t per face
    region_info = []
    for rid in ids:
        r = by_id[rid]
        sel_t, sel_f = region == rid, region_of_face == rid
        volume = float(tet_vol[sel_t].sum()) if len(tets) else 0.0
        area = float(face_area[sel_f].sum())
        skin = r.get("skin")
        t = float(skin["thickness"]) if skin else 0.0
        if skin:
            skin_density_t[sel_f] = float(skin["material"]["density"]) * t
            material_entries.append(("Skin_%d" % rid, rid + SHELL_PROPERTIES_OFFSET, "LinearElasticPlaneStress2DLaw",
                                     {"DENSITY": float(skin["material"]["density"]), "YOUNG_MODULUS": float(skin["material"]["youngs_modulus"]),
                                      "POISSON_RATIO": float(skin["material"]["poisson_ratio"]), "THICKNESS": t}))
            property_ids.append(rid + SHELL_PROPERTIES_OFFSET)
        if r.get("core"):
            h = homogenize_region(job_dir, r, int(solver_opts["homogenize_samples"]), bool(solver_opts["homogenize_cache"]))
            homog[rid] = h
            base = r["core"]["material"]
            rho_eff = float(base["density"]) * h["relative_density"]
            # The shells and the core overlap by the skin's volume: take
            # it off the core so the mass, and so the inertial load, is
            # the part's.
            overlap = min(area * t / volume, 1.0) if (volume > 0.0 and t > 0.0) else 0.0
            correct = bool(r["core"].get("density_correction", True))
            rho_core = rho_eff * (1.0 - overlap) if correct else rho_eff
            tet_density[sel_t] = rho_core
            material_entries.append(("Solid_%d" % rid, rid, "UserProvidedLinearElastic3DLaw",
                                     {"DENSITY": rho_core, "ELASTICITY_TENSOR": [[float(v) for v in row] for row in h["C"]]}))
            property_ids.append(rid)
            region_info.append({
                "id": rid, "label": r.get("label", ""), "volume": volume, "area": area, "skin_thickness": t,
                "skin_mass": (float(skin["material"]["density"]) * t * area) if skin else 0.0, "core_mass": rho_core * volume,
                "core_density_effective": rho_eff, "core_density_used": rho_core, "core_overlap_fraction": overlap,
                "relative_density": h["relative_density"], "dropped_fraction": h["dropped_fraction"],
                "E": h["engineering"]["E"], "G": h["engineering"]["G"],
                "nu_xy": h["engineering"]["nu_xy"], "nu_yz": h["engineering"]["nu_yz"], "nu_xz": h["engineering"]["nu_xz"],
                "amplification": h["amplification"], "homogenize_seconds": h["seconds"], "homogenize_cached": h.get("cached", False),
                "homogenize_dofs": h["dofs"], "C": [[float(v) for v in row] for row in h["C"]],
            })
        elif r.get("solid"):
            m = r["solid"]["material"]
            tet_density[sel_t] = float(m["density"])
            material_entries.append(("Solid_%d" % rid, rid, "LinearElastic3DLaw",
                                     {"DENSITY": float(m["density"]), "YOUNG_MODULUS": float(m["youngs_modulus"]), "POISSON_RATIO": float(m["poisson_ratio"])}))
            property_ids.append(rid)
    seconds_homogenize = time.time() - t0
    mass = float((tet_density * tet_vol).sum() + (skin_density_t * face_area).sum()) if len(tets) else float((skin_density_t * face_area).sum())

    # The elements: the tetrahedra -- the 4-node ones first, then those of
    # the second-order regions with the mid-edge nodes added, and 6-node
    # conditions on THEIR boundary faces -- shells on the corner nodes of
    # every wetted face of a region with a skin, and a surface-load
    # condition on every wetted face. The shell-only faces and the 4-node
    # regions' faces stay 3-node: a mid-edge node there would have a load
    # and no stiffness.
    support_corner_nodes = len(support_nodes)
    tets4, region4 = tets[:n_tet4], region[:n_tet4]
    if use_second:
        nodes_all, tets10, tri6 = second_order(nodes, tets[n_tet4:], tet_faces[face_second])
        # The clamp again on EVERY node, the mid-edge ones included: a
        # mid-edge node on the support plane left free (2026-11-02, found on
        # a 1 x 10 x 1 bar at 1.08 then 1.04 of beam theory, converging only
        # at first order) makes the clamp compliant.
        support_nodes, support_planes = _support_nodes(nodes_all, supports)
    else:
        nodes_all = nodes
        tets10 = np.zeros((0, 10), dtype=np.int64)
        tri6 = np.zeros((0, 6), dtype=np.int64)
    tet_blocks = [(tets4, region4), (tets10, region[n_tet4:])]
    result["support_nodes"] = int(len(support_nodes))
    result["support_corner_nodes"] = int(support_corner_nodes)
    result["support_planes"] = support_planes
    has_skin = np.array([bool(by_id[int(rid)].get("skin")) for rid in region_of_face]) if len(faces) else np.zeros(0, dtype=bool)
    shell_sel = np.nonzero(has_skin)[0]
    shells = faces[shell_sel]
    shell_region = region_of_face[shell_sel]
    rotation_dofs = len(shells) > 0
    # Degrees of freedom: three per node, three more per shell node. Known
    # before the solve, so "auto" can choose its solver by them.
    shell_nodes = np.unique(shells) if len(shells) else np.zeros(0, dtype=np.int64)
    dofs = 3 * len(nodes_all) + 3 * len(shell_nodes)
    linear_solver = resolve_linear_solver(solver_opts["linear_solver"], dofs)
    result["linear_solver"] = linear_solver
    # A face's condition carries its region's skin Properties when it has
    # shells, its tetrahedra's otherwise. The condition ids are the faces'
    # order whatever block a face is written in.
    cond_prop = np.where(has_skin, region_of_face + SHELL_PROPERTIES_OFFSET, region_of_face)
    tet_face_ids = np.arange(n_tet_faces)
    cond_blocks = [(tet_faces[~face_second], cond_prop[:n_tet_faces][~face_second], tet_face_ids[~face_second]),
                   (tri6, cond_prop[:n_tet_faces][face_second], tet_face_ids[face_second]),
                   (extra_faces, cond_prop[n_tet_faces:], n_tet_faces + np.arange(len(extra_faces)))]
    point_prop = property_ids[0]
    t0 = time.time()
    log("  writing the Kratos model: %d nodes (%d mid-edge), %d 4-node + %d 10-node tetrahedra, %d shells, %d dofs"
        % (len(nodes_all), len(nodes_all) - len(nodes), n_tet4, n_tet10, len(shells), dofs))
    write_mdpa(mdpa_base + ".mdpa", nodes_all, tet_blocks, cond_blocks, shells, shell_region, support_nodes, point_loads,
               property_ids, solver_opts["shell_element"], point_prop)
    write_materials(materials_path, material_entries)
    seconds_mdpa = time.time() - t0
    parameters = project_parameters(mdpa_base, materials_path, loads, point_loads, echo_level, rotation_dofs, linear_solver)
    log("  solving %d dofs with %s (%d tetrahedra, %d shells)" % (dofs, linear_solver, len(tets), len(shells)))
    model, structure, analysis, seconds_solve = run_kratos(job_dir, parameters, len(nodes_all), face_pressure, rotation_dofs, echo_level)
    displacement, rotation, reaction = read_nodal(structure, len(nodes_all), rotation_dofs)
    nt = len(tets)
    n_elements = nt + len(shells)
    log("  post-processing: stresses, margins, the force balance")
    von_mises_elem = element_von_mises(structure, n_elements)
    # The micro stress of every homogenised core element from its own
    # strain state; the smeared figure is what the element reports.
    micro = np.zeros(n_elements)
    bound = np.zeros(n_elements)
    if homog:
        strain, stress = core_strains_stresses(structure, n_elements)
        import homogenize as hz
        for rid, h in homog.items():
            sel = np.nonzero(region == rid)[0]
            if len(sel):
                micro[sel] = hz.micro_von_mises(h["critical"], strain[sel])
                bound[sel] = h["amplification"] * hz.frobenius(stress[sel])
    # The peaks per region: the skin's shells, the core by its MICRO
    # stress (with the smeared figure beside it), the solid's tetrahedra;
    # each with the margin against its own allowable.
    def margin(allowable, peak):
        return (allowable / peak - 1.0) if (allowable > 0.0 and peak > 0.0) else 0.0
    per_material = []
    combined = von_mises_elem.copy()
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", "")
        sel_t = np.nonzero(region == rid)[0] if len(tets) else np.zeros(0, dtype=np.int64)
        sel_s = np.nonzero(shell_region == rid)[0] + nt
        if r.get("skin"):
            a = float(r["skin"]["material"].get("allowable_stress", 0.0))
            peak = float(von_mises_elem[sel_s].max()) if len(sel_s) else 0.0
            per_material.append({"id": rid, "label": "%s skin" % label, "kind": "skin", "max_von_mises": peak, "margin": margin(a, peak), "allowable": a,
                                 "mass": float((skin_density_t[region_of_face == rid] * face_area[region_of_face == rid]).sum())})
        if r.get("core"):
            a = float(r["core"]["material"].get("allowable_stress", 0.0))
            peak_core = float(von_mises_elem[sel_t].max()) if len(sel_t) else 0.0
            peak_micro = float(micro[sel_t].max()) if len(sel_t) else 0.0
            peak_bound = float(bound[sel_t].max()) if len(sel_t) else 0.0
            combined[sel_t] = micro[sel_t]
            info = next(i for i in region_info if i["id"] == rid)
            skin_m = r.get("skin")
            a_skin = float(skin_m["material"].get("allowable_stress", 0.0)) if skin_m else 0.0
            peak_skin = float(von_mises_elem[sel_s].max()) if len(sel_s) else 0.0
            info.update({"skin_max_von_mises": peak_skin, "skin_margin": margin(a_skin, peak_skin),
                         "core_max_von_mises": peak_core, "core_max_micro_von_mises": peak_micro, "core_max_micro_bound": peak_bound,
                         "core_margin": margin(a, peak_micro)})
            per_material.append({"id": rid, "label": "%s core" % label, "kind": "core", "max_von_mises": peak_micro,
                                 "macro_von_mises": peak_core, "margin": margin(a, peak_micro), "allowable": a, "mass": info["core_mass"]})
        if r.get("solid"):
            a = float(r["solid"]["material"].get("allowable_stress", 0.0))
            peak = float(von_mises_elem[sel_t].max()) if len(sel_t) else 0.0
            per_material.append({"id": rid, "label": label, "kind": "solid", "max_von_mises": peak, "margin": margin(a, peak), "allowable": a,
                                 "mass": float((tet_vol[region == rid] * tet_density[region == rid]).sum()) if len(tets) else 0.0})
    i_vm = int(np.argmax(combined)) if n_elements else 0
    max_vm_region = int(region[i_vm]) if i_vm < nt else int(shell_region[i_vm - nt])
    result.update({"second_order": use_second, "second_order_regions": second_ids, "tets_second_order": int(n_tet10),
                   "elements": int(n_elements), "tets": int(nt), "shells": int(len(shells)),
                   "seconds_mdpa": seconds_mdpa, "seconds_homogenize": seconds_homogenize, "regions": region_info})

    # The force balance. Gravity on the mass; the pressure's resultant as
    # -p n A over the outward faces; the point force as given.
    g = float(loads["load_factor"]) * G0
    applied_gravity = np.array([0.0, 0.0, -mass * g])
    applied_pressure = -(face_pressure[:, None] * n_area).sum(axis=0)
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

    # Inward or outward: u.n averaged over the wetted corner nodes, with
    # the node normal the area-weighted mean of its faces'.
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
            frequencies, mode_shapes, eigensolver_used = run_modes(job_dir, mdpa_base, materials_path, len(nodes_all), echo_level, rotation_dofs,
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
        "nodes": int(len(nodes_all)),
        "dofs": int(dofs),
        "mass": mass,
        "max_displacement": float(u_mag[i_max]),
        "max_displacement_at": [float(v) for v in nodes_all[i_max]],
        "max_von_mises": float(combined[i_vm]) if n_elements else 0.0,
        "max_von_mises_material": max_vm_region,
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
    element_blocks = ([tets4] if n_tet4 else []) + ([tets10] if n_tet10 else []) + ([shells] if len(shells) else [])
    cell_types = ([VTK_TETRA] if n_tet4 else []) + ([VTK_QUADRATIC_TETRA] if n_tet10 else []) + ([VTK_TRIANGLE] if len(shells) else [])
    kind = np.concatenate([np.zeros(nt, dtype=np.int64), np.ones(len(shells), dtype=np.int64)])
    cell_data = {"material": np.concatenate([region, shell_region]).astype(np.int64), "kind": kind,
                 "von_mises_element": von_mises_elem, "micro_von_mises_element": micro, "micro_von_mises_bound": bound}
    point_data = {"displacement": displacement, "von_mises": nodal_average(len(nodes_all), element_blocks, von_mises_elem)}
    if rotation_dofs:
        point_data["rotation"] = rotation
    if homog:
        point_data["micro_von_mises"] = nodal_average(len(nodes_all), element_blocks, micro)
    if mode_shapes is not None:
        for k in range(len(frequencies)):
            point_data["mode_%d" % (k + 1)] = mode_shapes[k]
    write_vtk(outputs["vtk"], nodes_all, element_blocks, cell_types, point_data, cell_data)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved %d dofs (%d tetrahedra, %d shells) with %s in %.2f s: mass %.6g kg, max displacement %.4g m, max von Mises %.4g Pa (region %d); "
        "reaction (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g): %s; wrote %s"
        % (dofs, nt, len(shells), linear_solver, result["seconds_solve"], mass, result["max_displacement"], result["max_von_mises"], result["max_von_mises_material"],
           reaction_sum[0], reaction_sum[1], reaction_sum[2], applied_gravity[0], applied_gravity[1], applied_gravity[2],
           applied_pressure[0], applied_pressure[1], applied_pressure[2], applied_point[0], applied_point[1], applied_point[2],
           ("balanced to %.2e" % balance_error) if balanced else ("NOT BALANCED (%.1f%%)" % (100.0 * balance_error)), outputs["vtk"]))
    if n_modes > 0 and len(frequencies):
        log("  natural frequencies (%s, %.2f s): %s Hz" % (eigensolver_used, seconds_modes, ", ".join("%.4g" % f for f in frequencies)))
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
