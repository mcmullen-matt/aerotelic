"""The potential flow around a selected set of bodies, on KratosMultiphysics.

Run by the application (Entity/AerotelicPotentialFlow.cpp) as a separate
process in the WingDesigner micromamba environment, where Kratos is
installed (``pip install KratosMultiphysics
KratosFluidDynamicsApplication KratosCompressiblePotentialFlowApplication``,
all at the same version)::

    python potential_flow.py <job directory>

The job directory holds ``job.json`` (see ``run_job`` for the keys) and the
selected bodies as ``.tri`` files -- the application's own triangulation of
them, placed as they sit in the scene. The script writes the solution as
one legacy VTK unstructured grid (the path the user chose) and
``result.json``.

What it does, in three stages:

  1. The FLUID DOMAIN. Potential flow is a field in the air, not in the
     aircraft, so the mesh wanted here is the other one from the structural
     route (AerotelicTetGmsh.h): a far-field box with the bodies as holes.
     The box is triangulated here at the far-field element size; every body
     goes in as a DISCRETE surface, exactly as the skin/core mesher does
     it, so the wetted boundary of the fluid mesh is the application's own
     triangulation of the body -- no re-triangulation, no envelope, and the
     pressures come out on the surface the designer sees.

  2. The SOLVE. Kratos's CompressiblePotentialFlowApplication over
     tetrahedra: one scalar velocity potential per node, grad(phi) = u.
     The far-field process fixes the potential on the inflow part of the
     box to u_inf . x and leaves the flux u_inf . n on the rest; the body
     needs no condition at all, since d(phi)/dn = 0 is this formulation's
     natural boundary condition -- that IS the non-penetration wall.

  3. The CHECKS, in result.json and in the status line. A potential flow
     solution has no wake, so it carries no lift and, by d'Alembert's
     paradox, no force at all: the integral of -Cp n dA over the body is a
     pure discretisation error and is reported as a force coefficient on
     the frontal area. The stagnation pressure coefficient should be 1
     wherever the flow comes to rest.

LIMITATION, stated plainly: with no wake surface the circulation around a
lifting section is zero, so this solves the NON-LIFTING potential flow.
The velocity field, the suction peaks and the pressure distribution are
real; the lift is not (it is zero by construction). Kratos has a 3-D wake
operation (``Define3DWakeOperation``) which needs the trailing edge
identified and a wake sheet shed from it; hooking that up is the next step,
not this prototype.

``.tri`` is the application's own format (tools/tet_gmsh.py).
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# The .tri reader, the closed-surface checks, the connected-shell split and
# the discrete-surface helper are the skin/core mesher's: one fluid body is
# the same kind of object as one pocket of a hollowed body. Importing it
# also bootstraps the environment's Library\bin onto PATH (numpy's BLAS and
# gmsh's DLLs) before numpy and gmsh come in.
import tet_gmsh as tg  # noqa: E402

import numpy as np  # noqa: E402
import gmsh  # noqa: E402

JobError = tg.JobError

GMSH_TRIANGLE = tg.GMSH_TRIANGLE
GMSH_TET = tg.GMSH_TET


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------------------
# The far-field box
# ---------------------------------------------------------------------------

def _box_face(uvals, vvals, w, k, outward_positive):
    """A grid of triangles on the plane x_k = w.

    (u, v) are the next two axes cyclically, so u x v = +e_k and the
    natural winding faces the positive side: that is what the high face
    wants and the mirror of what the low face wants."""
    nu, nv = len(uvals), len(vvals)
    iu, iv = (k + 1) % 3, (k + 2) % 3
    U, V = np.meshgrid(uvals, vvals, indexing="ij")
    pts = np.empty((nu * nv, 3), dtype=np.float64)
    pts[:, iu] = U.reshape(-1)
    pts[:, iv] = V.reshape(-1)
    pts[:, k] = w
    idx = np.arange(nu * nv, dtype=np.int64).reshape(nu, nv)
    a = idx[:-1, :-1].reshape(-1)
    b = idx[1:, :-1].reshape(-1)
    c = idx[1:, 1:].reshape(-1)
    d = idx[:-1, 1:].reshape(-1)
    tris = np.concatenate([np.column_stack([a, b, c]), np.column_stack([a, c, d])])
    if not outward_positive:
        tris = tris[:, [0, 2, 1]]
    return pts, tris


def box_surface(lo, hi, size):
    """The far-field box as a closed, outward triangulation of about this
    element size.

    Each face is a grid on the same per-axis ``linspace``, so the vertices
    the faces share come out bit for bit identical and welding them with
    ``np.unique`` closes the box exactly -- gmsh refuses a volume whose
    boundary has unpaired edges, which is what duplicated corner vertices
    would leave it."""
    counts = [max(1, int(np.ceil((hi[k] - lo[k]) / size))) for k in range(3)]
    axes = [np.linspace(lo[k], hi[k], counts[k] + 1) for k in range(3)]
    verts, faces, base = [], [], 0
    for k in range(3):
        iu, iv = (k + 1) % 3, (k + 2) % 3
        for high in (False, True):
            pts, tris = _box_face(axes[iu], axes[iv], hi[k] if high else lo[k], k, high)
            verts.append(pts)
            faces.append(tris + base)
            base += len(pts)
    verts = np.concatenate(verts)
    faces = np.concatenate(faces)
    unique, inverse = np.unique(verts, axis=0, return_inverse=True)
    inverse = np.asarray(inverse).reshape(-1)
    return np.ascontiguousarray(unique), np.ascontiguousarray(inverse[faces].astype(np.int32))


# ---------------------------------------------------------------------------
# The mesh
# ---------------------------------------------------------------------------

def mesh_fluid_domain(body_shells, lo, hi, far_size, near_size, near_distance, growth_rate, min_size,
                      algorithm, optimize, optimize_netgen):
    """Tetrahedra filling the box minus the bodies.

    Returns (nodes, tets, surfaces) where surfaces is a list of
    (region, faces) with region 0 for the far field and 1 for a body, the
    faces indexing `nodes`. Every surface comes back as the triangles that
    went in: gmsh never re-triangulates a discrete entity.

    The element size is a distance field off the bodies: `near_size` out to
    `near_distance`, then growing at `growth_rate` (size per unit
    distance) until it reaches `far_size`. Without it gmsh grows straight
    off the wetted surface, and the near-wall elements then reach out into
    air that is barely disturbed; the elemental velocities averaged onto
    the surface come out too close to the free stream and the suction peaks
    are smeared (on a unit sphere at 320 triangles, a Cp minimum of -0.90
    against the exact -1.25)."""
    far_verts, far_faces = box_surface(lo, hi, far_size)
    tg.check_closed(far_verts, far_faces, "the far-field box")
    log("  far field %.6g x %.6g x %.6g at element size %.6g (%d triangles); %.6g at the bodies, held for %.6g"
        % (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], far_size, len(far_faces), near_size, near_distance))

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("Mesh.Algorithm3D", algorithm)
        gmsh.option.setNumber("Mesh.Optimize", 1 if optimize else 0)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1 if optimize_netgen else 0)
        # As in tet_gmsh.py: the sizes come from the surfaces that went in
        # and from the field below alone. These surfaces have no curves or
        # curvature for gmsh to size from.
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
        gmsh.option.setNumber("Mesh.MeshSizeMax", far_size)
        if min_size > 0.0:
            gmsh.option.setNumber("Mesh.MeshSizeMin", min_size)
        gmsh.model.add("fluid")

        node = 1
        node = tg.add_surface(far_verts, far_faces, 1, node)
        loops = [gmsh.model.geo.addSurfaceLoop([1])]
        regions = [(1, 0)]  # (surface tag, region)
        tag = 1
        for label, verts, faces in body_shells:
            tag += 1
            node = tg.add_surface(verts, faces, tag, node)
            loops.append(gmsh.model.geo.addSurfaceLoop([tag]))
            regions.append((tag, 1))
        # One volume: the box with every body as a hole. The bodies
        # themselves are not meshed -- there is no air in them.
        gmsh.model.geo.addVolume(loops, 1)
        gmsh.model.geo.synchronize()

        # The size field. Threshold interpolates linearly in distance, so
        # the end of the growth is where the rate takes the size from
        # near to far: a rate of 0.3 is about 30% bigger per layer, the
        # usual rule of thumb. MeshSizeExtendFromBoundary is off above, or
        # gmsh would also carry the far-field box's own coarse triangles
        # inward and win the argument near the body.
        distance = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumbers(distance, "SurfacesList", [t for t, region in regions if region == 1])
        threshold = gmsh.model.mesh.field.add("Threshold")
        gmsh.model.mesh.field.setNumber(threshold, "InField", distance)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMin", near_size)
        gmsh.model.mesh.field.setNumber(threshold, "SizeMax", far_size)
        gmsh.model.mesh.field.setNumber(threshold, "DistMin", near_distance)
        gmsh.model.mesh.field.setNumber(threshold, "DistMax",
                                        near_distance + max(far_size - near_size, 0.0) / growth_rate)
        gmsh.model.mesh.field.setAsBackgroundMesh(threshold)

        t_mesh = time.time()
        gmsh.model.mesh.generate(3)
        seconds_mesh = time.time() - t_mesh

        node_tags, coords, _ = gmsh.model.mesh.getNodes()
        node_tags = np.array(node_tags, dtype=np.int64)
        nodes = np.array(coords, dtype=np.float64).reshape(-1, 3)
        order = np.argsort(node_tags)
        node_tags, nodes = node_tags[order], nodes[order]
        lookup = np.full(int(node_tags.max()) + 1, -1, dtype=np.int64)
        lookup[node_tags] = np.arange(len(node_tags))

        types, tags, conn = gmsh.model.mesh.getElements(3, 1)
        tets, quality = None, None
        for etype, etags, flat in zip(types, tags, conn):
            if int(etype) == GMSH_TET:
                tets = lookup[np.array(flat, dtype=np.int64).reshape(-1, 4)].astype(np.int32)
                # Signed inverse condition number: 1 is the regular
                # tetrahedron, 0 is flat, negative is inverted.
                quality = np.array(gmsh.model.mesh.getElementQualities(etags, "minSICN"), dtype=np.float64)
        if tets is None or len(tets) == 0:
            raise JobError("gmsh meshed no tetrahedra into the fluid domain.")

        # The triangles of a discrete surface come back exactly as they
        # went in, node order included, so the wetted boundary of the
        # fluid mesh IS the body's own triangulation, still outward.
        surfaces = []
        for tag, region in regions:
            types, _, conn = gmsh.model.mesh.getElements(2, tag)
            for etype, flat in zip(types, conn):
                if int(etype) == GMSH_TRIANGLE:
                    f = lookup[np.array(flat, dtype=np.int64).reshape(-1, 3)].astype(np.int32)
                    surfaces.append((region, f))
    finally:
        gmsh.finalize()

    # The application's convention is a positive volume; gmsh hands back a
    # few the other way round, so they are swapped, not dropped (tet_gmsh.py).
    vol = tg.tet_volumes(nodes, tets)
    flipped = int((vol < 0.0).sum())
    if flipped:
        tets[vol < 0.0] = tets[vol < 0.0][:, [0, 2, 1, 3]]
        vol = tg.tet_volumes(nodes, tets)
    stats = {
        "mesh_nodes": int(len(nodes)),
        "mesh_tets": int(len(tets)),
        "fluid_volume": float(vol.sum()),
        "flipped": flipped,
        "degenerate": int((vol <= 0.0).sum()),
        "min_quality": float(quality.min()),
        "mean_quality": float(quality.mean()),
        "poor_quality": int((quality < 0.1).sum()),
        "seconds_mesh": seconds_mesh,
    }
    return nodes, tets, surfaces, stats


# ---------------------------------------------------------------------------
# Kratos input
# ---------------------------------------------------------------------------

def write_mdpa(path, nodes, tets, far_faces, body_faces):
    """The mesh as a Kratos .mdpa.

    Generic ``Element3D4N`` / ``SurfaceCondition3D3N`` go in: the potential
    flow solver replaces every element and condition with its own
    (``IncompressiblePotentialFlowElement3D4N``,
    ``PotentialWallCondition3D3N``) when it prepares the model part, so
    naming them here would only be a second place to keep in step.

    Three sub model parts, the names ProjectParameters refers to:
    ``FluidParts_Fluid`` (the volume), ``FarField`` and ``Body`` (the two
    skins). The body conditions carry no flux -- they are there to give the
    pressures a surface to live on and to be found by name."""
    n_nodes, n_tets = len(nodes), len(tets)
    n_far, n_body = len(far_faces), len(body_faces)
    with open(path, "w", encoding="ascii") as f:
        f.write("Begin ModelPartData\nEnd ModelPartData\n\n")
        f.write("Begin Properties 0\nEnd Properties\n\n")
        f.write("Begin Nodes\n")
        np.savetxt(f, np.column_stack([np.arange(1, n_nodes + 1), nodes]),
                   fmt="%d %.17g %.17g %.17g")
        f.write("End Nodes\n\n")
        f.write("Begin Elements Element3D4N\n")
        np.savetxt(f, np.column_stack([np.arange(1, n_tets + 1), np.zeros(n_tets, dtype=np.int64), tets + 1]),
                   fmt="%d %d %d %d %d %d")
        f.write("End Elements\n\n")
        f.write("Begin Conditions SurfaceCondition3D3N\n")
        faces = np.concatenate([far_faces, body_faces]) if n_body else far_faces
        np.savetxt(f, np.column_stack([np.arange(1, len(faces) + 1), np.zeros(len(faces), dtype=np.int64), faces + 1]),
                   fmt="%d %d %d %d %d")
        f.write("End Conditions\n\n")

        def sub_part(name, node_ids, element_ids=None, condition_ids=None):
            f.write("Begin SubModelPart %s\n" % name)
            f.write("  Begin SubModelPartNodes\n")
            np.savetxt(f, node_ids, fmt="  %d")
            f.write("  End SubModelPartNodes\n")
            if element_ids is not None:
                f.write("  Begin SubModelPartElements\n")
                np.savetxt(f, element_ids, fmt="  %d")
                f.write("  End SubModelPartElements\n")
            if condition_ids is not None:
                f.write("  Begin SubModelPartConditions\n")
                np.savetxt(f, condition_ids, fmt="  %d")
                f.write("  End SubModelPartConditions\n")
            f.write("End SubModelPart\n\n")

        sub_part("FluidParts_Fluid", np.arange(1, n_nodes + 1), element_ids=np.arange(1, n_tets + 1))
        sub_part("FarField", np.unique(far_faces) + 1, condition_ids=np.arange(1, n_far + 1))
        if n_body:
            sub_part("Body", np.unique(body_faces) + 1, condition_ids=np.arange(n_far + 1, n_far + n_body + 1))


def project_parameters(mdpa_base, flow, solver, reference_chord):
    """ProjectParameters for one steady potential flow solve.

    The body is deliberately absent from the process list: in this
    formulation d(phi)/dn = 0 is the natural condition, so a wall is what a
    boundary with nothing applied to it already is. Only the far field gets
    a process, and it is the one that decides, per condition and from the
    sign of u_inf . n, which part of the box is inflow (the potential is
    fixed there) and which is outflow (the flux is applied)."""
    mach = flow["speed"] / flow["speed_of_sound"]
    return {
        "problem_data": {
            "problem_name": "aerotelic_potential_flow",
            "parallel_type": "OpenMP",
            "echo_level": solver["echo_level"],
            "start_time": 0.0,
            "end_time": 1.0,
        },
        "solver_settings": {
            "solver_type": "potential_flow",
            "model_part_name": "FluidModelPart",
            "domain_size": 3,
            "model_import_settings": {"input_type": "mdpa", "input_filename": mdpa_base},
            "formulation": {"element_type": flow["element_type"]},
            "maximum_iterations": solver["max_iterations"],
            "echo_level": solver["echo_level"],
            "relative_tolerance": solver["relative_tolerance"],
            "absolute_tolerance": solver["relative_tolerance"],
            "linear_solver_settings": {
                "solver_type": "amgcl",
                "krylov_type": "cg",
                "smoother_type": "ilu0",
                "coarsening_type": "aggregation",
                "max_iteration": 1000,
                "tolerance": solver["linear_tolerance"],
                "verbosity": 0,
            },
            "volume_model_part_name": "FluidModelPart.FluidParts_Fluid",
            "skin_parts": ["FluidModelPart.FarField", "FluidModelPart.Body"],
            "assign_neighbour_elements_to_conditions": True,
            # Both projections are asked for below; adding them here as
            # nodal variables as well costs one array each and makes the
            # result readable whichever way the process stores it.
            "auxiliary_variables_list": ["VELOCITY", "PRESSURE_COEFFICIENT"],
            "reference_chord": reference_chord,
            # One step: the solve is steady. The whole block has to be
            # here, not just the step -- Kratos validates these settings
            # one level deep, so a partial sub-dictionary replaces the
            # defaults instead of being filled in from them.
            "time_stepping": {
                "automatic_time_step": False,
                "CFL_number": 1,
                "minimum_delta_time": 1e-4,
                "maximum_delta_time": 1.0,
                "time_step": 1.0,
            },
        },
        "processes": {
            "boundary_conditions_process_list": [{
                "python_module": "apply_far_field_and_wake_process",
                "kratos_module": "KratosMultiphysics.CompressiblePotentialFlowApplication",
                "process_name": "ApplyFarFieldAndWakeProcess",
                "Parameters": {
                    "model_part_name": "FluidModelPart.FarField",
                    "angle_of_attack_units": "degrees",
                    "angle_of_attack": flow["angle_of_attack_deg"],
                    "free_stream_density": flow["density"],
                    "mach_infinity": mach,
                    "speed_of_sound": flow["speed_of_sound"],
                    "inlet_potential": 1.0,
                    "initialize_flow_field": True,
                    "perturbation_field": False,
                    "define_wake": False,
                },
            }],
            "auxiliary_process_list": [{
                "python_module": "compute_nodal_value_process",
                "kratos_module": "KratosMultiphysics.CompressiblePotentialFlowApplication",
                "process_name": "ComputeNodalValueProcess",
                "Parameters": {
                    "model_part_name": "FluidModelPart",
                    "elemental_variables_list_to_project": ["VELOCITY", "PRESSURE_COEFFICIENT"],
                },
            }],
        },
    }


# ---------------------------------------------------------------------------
# The solve
# ---------------------------------------------------------------------------

def solve(job_dir, nodes, tets, far_faces, body_faces, flow, solver, reference_chord):
    """Runs Kratos over the mesh. Returns (potential, velocity, cp, stats).

    Kratos is imported here and not at the top of the file: gmsh is done
    with and finalised by now, and the two libraries bring their own
    threading runtimes."""
    mdpa_base = os.path.join(job_dir, "model")
    t_write = time.time()
    write_mdpa(mdpa_base + ".mdpa", nodes, tets, far_faces, body_faces)
    seconds_write = time.time() - t_write

    import KratosMultiphysics as KM
    import KratosMultiphysics.CompressiblePotentialFlowApplication as CPFApp
    from KratosMultiphysics.CompressiblePotentialFlowApplication.potential_flow_analysis import PotentialFlowAnalysis

    if solver["echo_level"] == 0:
        KM.Logger.GetDefaultOutput().SetSeverity(KM.Logger.Severity.WARNING)
    parameters = KM.Parameters(json.dumps(project_parameters(mdpa_base, flow, solver, reference_chord)))
    model = KM.Model()
    t_solve = time.time()
    analysis = PotentialFlowAnalysis(model, parameters)
    analysis.Run()
    seconds_solve = time.time() - t_solve

    model_part = model["FluidModelPart"]
    n = len(nodes)
    if model_part.NumberOfNodes() != n:
        raise JobError("Kratos read %d of %d nodes." % (model_part.NumberOfNodes(), n))
    potential = np.zeros(n)
    velocity = np.zeros((n, 3))
    velocity_h = np.zeros((n, 3))
    cp = np.zeros(n)
    cp_h = np.zeros(n)
    for node in model_part.Nodes:
        i = node.Id - 1
        potential[i] = node.GetSolutionStepValue(CPFApp.VELOCITY_POTENTIAL)
        v = node.GetValue(KM.VELOCITY)
        velocity[i] = (v[0], v[1], v[2])
        cp[i] = node.GetValue(KM.PRESSURE_COEFFICIENT)
        v = node.GetSolutionStepValue(KM.VELOCITY)
        velocity_h[i] = (v[0], v[1], v[2])
        cp_h[i] = node.GetSolutionStepValue(KM.PRESSURE_COEFFICIENT)
    # ComputeNodalValueProcess may project onto either the historical or
    # the non-historical database depending on the Kratos version; take
    # whichever came back with something in it.
    if np.abs(velocity).max() < np.abs(velocity_h).max():
        velocity, cp = velocity_h, cp_h
    stats = {"seconds_mdpa": seconds_write, "seconds_solve": seconds_solve}
    return potential, velocity, cp, stats


# ---------------------------------------------------------------------------
# What the solution is worth
# ---------------------------------------------------------------------------

def stream_direction(flow):
    """The free-stream unit vector in Kratos's 3-D convention, which is the
    application's too: x downstream, y along the span, z up."""
    aoa = np.radians(flow["angle_of_attack_deg"])
    return np.array([np.cos(aoa), 0.0, np.sin(aoa)])


def surface_loads(verts, faces, cp, flow):
    """The force the pressures put on the bodies, and the frontal area.

    The bodies' own triangles are used, outward as they came in, so the
    sign is physical: F = -q_inf * integral(Cp n dA). In a potential flow
    this force is zero exactly (d'Alembert's paradox), so what comes out is
    a measure of the discretisation; it is reported against q_inf times the
    area the bodies present to the stream. That area is half the summed
    absolute projection of the surface, because a ray crosses a closed body
    once on the way in and once on the way out."""
    if len(faces) == 0:
        return np.zeros(3), 0.0, 0.0
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    cross = np.cross(b - a, c - a)
    areas = cross * 0.5
    cp_face = (cp[faces[:, 0]] + cp[faces[:, 1]] + cp[faces[:, 2]]) / 3.0
    q_inf = 0.5 * flow["density"] * flow["speed"] ** 2
    force = -q_inf * (cp_face[:, None] * areas).sum(axis=0)
    frontal = 0.5 * float(np.abs(areas @ stream_direction(flow)).sum())
    return force, frontal, q_inf


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_vtk(path, nodes, tets, body_faces, potential, velocity, cp):
    """The grid and the solution as one legacy VTK unstructured grid, in the
    same dialect as the structural export (core/src/tet_skin.cpp).

    The body's surface triangles go in beside the tetrahedra with cell data
    ``region`` 1 against the fluid's 0, so the pressure distribution on the
    aircraft can be shown on its own (threshold region = 1) without a
    second file."""
    n_nodes, n_tets, n_faces = len(nodes), len(tets), len(body_faces)
    n_cells = n_tets + n_faces
    with open(path, "w", encoding="ascii") as f:
        f.write("# vtk DataFile Version 3.0\nAerotelic potential flow solution\nASCII\nDATASET UNSTRUCTURED_GRID\n")
        f.write("POINTS %d double\n" % n_nodes)
        np.savetxt(f, nodes, fmt="%.17g %.17g %.17g")
        f.write("CELLS %d %d\n" % (n_cells, 5 * n_tets + 4 * n_faces))
        np.savetxt(f, np.column_stack([np.full(n_tets, 4), tets]), fmt="%d %d %d %d %d")
        if n_faces:
            np.savetxt(f, np.column_stack([np.full(n_faces, 3), body_faces]), fmt="%d %d %d %d")
        f.write("CELL_TYPES %d\n" % n_cells)
        np.savetxt(f, np.concatenate([np.full(n_tets, 10), np.full(n_faces, 5)]), fmt="%d")
        f.write("CELL_DATA %d\nSCALARS region int 1\nLOOKUP_TABLE default\n" % n_cells)
        np.savetxt(f, np.concatenate([np.zeros(n_tets, dtype=np.int64), np.ones(n_faces, dtype=np.int64)]), fmt="%d")
        f.write("POINT_DATA %d\n" % n_nodes)
        f.write("SCALARS VELOCITY_POTENTIAL double 1\nLOOKUP_TABLE default\n")
        np.savetxt(f, potential, fmt="%.9g")
        f.write("SCALARS PRESSURE_COEFFICIENT double 1\nLOOKUP_TABLE default\n")
        np.savetxt(f, cp, fmt="%.9g")
        f.write("VECTORS VELOCITY double\n")
        np.savetxt(f, velocity, fmt="%.9g %.9g %.9g")


# ---------------------------------------------------------------------------
# The job
# ---------------------------------------------------------------------------

def run_job(job_dir):
    """Runs the job in `job_dir`. job.json:

        bodies        [{"label", "file"}]: the selected bodies, each a
                      closed outward .tri in the scene's placement. Several
                      closed shells in one file (a propeller) are allowed.
        far_field     {"size_factor": the box's half-extent in diagonals of
                      the bodies' bounding box; "element_size": 0 for a
                      tenth of that half-extent}
        volume        {"near_size": element size at the bodies (0 for the
                      mean edge of their own triangles), "near_distance":
                      how far that size is held (0 for twice the size),
                      "growth_rate": size added per unit distance beyond
                      that, up to the far-field size (0 for 0.3),
                      "min_size", "algorithm" (gmsh Mesh.Algorithm3D),
                      "optimize", "optimize_netgen"}
        flow          {"speed", "angle_of_attack_deg", "speed_of_sound",
                      "density", "element_type": "incompressible" |
                      "compressible"}
        solver        {"max_iterations", "relative_tolerance",
                      "linear_tolerance", "echo_level"}
        outputs       {"vtk": where to write the solution (the user's own
                      path), "result": the statistics file}

    Returns the result dictionary (also written to outputs.result).
    """
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    outputs = job["outputs"]
    far = job.get("far_field", {})
    volume = job.get("volume", {})
    flow = {
        "speed": 50.0, "angle_of_attack_deg": 0.0, "speed_of_sound": 340.0,
        "density": 1.225, "element_type": "incompressible",
    }
    flow.update(job.get("flow", {}))
    solver = {"max_iterations": 10, "relative_tolerance": 1e-9, "linear_tolerance": 1e-9, "echo_level": 0}
    solver.update(job.get("solver", {}))
    if not flow["speed"] > 0.0:
        raise JobError("the free-stream speed must be positive.")

    # The bodies, each split into its closed shells.
    body_shells = []
    all_verts, all_faces, base = [], [], 0
    for entry in job.get("bodies", []):
        verts, faces = tg.read_tri(os.path.join(job_dir, entry["file"]))
        label = entry.get("label", entry["file"])
        if len(faces) == 0:
            raise JobError("'%s' has no triangles." % label)
        tg.check_closed(verts, faces, label)
        parts = tg.shells(verts, faces)
        for i, (sv, sf) in enumerate(parts):
            if len(parts) > 1:
                tg.check_closed(sv, sf, "%s, shell %d" % (label, i + 1))
            body_shells.append((label, sv, sf))
        all_verts.append(verts)
        all_faces.append(faces + base)
        base += len(verts)
    if not body_shells:
        raise JobError("no body was given to compute the flow around.")
    body_verts = np.concatenate(all_verts)
    body_faces_in = np.concatenate(all_faces)
    body_volume = float(sum(tg.surface_volume(v, f) for _, v, f in body_shells))
    log("  %d body shell(s), %d triangles, volume %.6g"
        % (len(body_shells), len(body_faces_in), body_volume))

    lo_b, hi_b = body_verts.min(axis=0), body_verts.max(axis=0)
    diagonal = float(np.linalg.norm(hi_b - lo_b))
    factor = float(far.get("size_factor", 5.0))
    if not factor > 0.5:
        raise JobError("the far-field size factor must be more than 0.5 (the box has to contain the bodies).")
    centre = (lo_b + hi_b) * 0.5
    half = factor * diagonal
    lo, hi = centre - half, centre + half
    far_size = float(far.get("element_size", 0.0)) or 0.1 * diagonal * factor
    # The near-field size defaults to the bodies' own triangles: they
    # already say how finely the designer wants the surface resolved, and
    # matching them is what gets the pressures right on it.
    edges = np.concatenate([
        np.linalg.norm(body_verts[body_faces_in[:, 1]] - body_verts[body_faces_in[:, 0]], axis=1),
        np.linalg.norm(body_verts[body_faces_in[:, 2]] - body_verts[body_faces_in[:, 1]], axis=1),
        np.linalg.norm(body_verts[body_faces_in[:, 0]] - body_verts[body_faces_in[:, 2]], axis=1)])
    mean_edge = float(edges.mean())
    near_size = float(volume.get("near_size", 0.0)) or mean_edge
    near_distance = float(volume.get("near_distance", 0.0)) or 2.0 * near_size
    growth_rate = float(volume.get("growth_rate", 0.0)) or 0.3
    min_size = float(volume.get("min_size", 0.0))

    nodes, tets, surfaces, mesh_stats = mesh_fluid_domain(
        body_shells, lo, hi, far_size, near_size, near_distance, growth_rate, min_size,
        int(volume.get("algorithm", 1)), bool(volume.get("optimize", True)),
        bool(volume.get("optimize_netgen", False)))
    far_faces = np.concatenate([f for region, f in surfaces if region == 0])
    body_faces = np.concatenate([f for region, f in surfaces if region == 1])
    box_volume = float(np.prod(hi - lo))
    log("  %d nodes, %d tetrahedra (fluid %.6g vs box - bodies %.6g); %d far-field + %d wetted triangles; quality min %.3f mean %.3f; %.2f s"
        % (mesh_stats["mesh_nodes"], mesh_stats["mesh_tets"], mesh_stats["fluid_volume"],
           box_volume - body_volume, len(far_faces), len(body_faces),
           mesh_stats["min_quality"], mesh_stats["mean_quality"], mesh_stats["seconds_mesh"]))

    # Kratos wants a reference length for the coefficients its own lift and
    # force processes report; those are not used here (there is no wake to
    # give a lift), so the bodies' streamwise extent is honest enough.
    reference_chord = float(hi_b[0] - lo_b[0]) or 1.0
    potential, velocity, cp, solve_stats = solve(
        job_dir, nodes, tets, far_faces, body_faces, flow, solver, reference_chord)

    # The pressure integral needs the wetted triangles outward from the
    # bodies, which is how they went in and how gmsh hands them back; the
    # enclosed volume says so, and is the check that the fluid mesh really
    # is bounded by the bodies' own triangulation.
    wetted_volume = tg.surface_volume(nodes, body_faces)
    if wetted_volume < 0.0:
        body_faces = np.ascontiguousarray(body_faces[:, [0, 2, 1]])
        wetted_volume = -wetted_volume
    force, frontal, q_inf = surface_loads(nodes, body_faces, cp, flow)
    speed = np.linalg.norm(velocity, axis=1)

    result = {
        "ok": True,
        "bodies": len(body_shells),
        "body_triangles": int(len(body_faces_in)),
        "body_volume": body_volume,
        # The volume the wetted boundary of the fluid mesh encloses: equal
        # to the bodies' to rounding, since it is their own triangulation.
        "wetted_volume": float(wetted_volume),
        "box_volume": box_volume,
        "far_field_size": float(2.0 * half),
        "far_field_element_size": far_size,
        "near_element_size": near_size,
        "near_distance": near_distance,
        "growth_rate": growth_rate,
        "body_mean_edge": mean_edge,
        "far_field_triangles": int(len(far_faces)),
        "wetted_triangles": int(len(body_faces)),
        "reference_chord": reference_chord,
        "frontal_area": frontal,
        "free_stream_speed": flow["speed"],
        "angle_of_attack_deg": flow["angle_of_attack_deg"],
        "mach": flow["speed"] / flow["speed_of_sound"],
        "max_speed": float(speed.max()),
        "min_cp": float(cp.min()),
        "max_cp": float(cp.max()),
        # Where the flow comes to rest Cp is 1; the largest value on the
        # body says how well the mesh resolved the stagnation point.
        "max_cp_body": float(cp[np.unique(body_faces)].max()) if len(body_faces) else 0.0,
        "min_cp_body": float(cp[np.unique(body_faces)].min()) if len(body_faces) else 0.0,
        "force": [float(force[0]), float(force[1]), float(force[2])],
        # Zero in an exact potential flow (d'Alembert): the residual is the
        # discretisation error.
        "force_coefficient": float(np.linalg.norm(force) / (q_inf * frontal)) if frontal > 0.0 else 0.0,
        "vtk": outputs["vtk"],
    }
    result.update(mesh_stats)
    result.update(solve_stats)

    t_vtk = time.time()
    write_vtk(outputs["vtk"], nodes, tets, body_faces, potential, velocity, cp)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved in %.2f s: speed up to %.4g (free stream %.4g), Cp on the bodies %.4g to %.4g; "
        "net force (%.4g, %.4g, %.4g) = %.2e of q*frontal area (zero in an exact potential flow); wrote %s in %.2f s"
        % (result["seconds_solve"], result["max_speed"], flow["speed"], result["min_cp_body"], result["max_cp_body"],
           force[0], force[1], force[2], result["force_coefficient"], outputs["vtk"], result["seconds_vtk"]))
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
        log("usage: potential_flow.py <job directory>")
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
