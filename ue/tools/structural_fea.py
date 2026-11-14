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
goes there) -- and a point force at the node nearest a point. A positive
pressure presses inwards (against the outward face normal), as an
external pressure does; the mean of u.n over the wetted nodes is reported
so the sign can be seen to be right. The pressure is applied as the
consistent NODAL loads of each face (-p n A, a third to each corner --
exactly the linear shell's consistent load for a uniform pressure on a
flat triangle), through point-load conditions, NOT through Kratos's
surface-load condition (2026-11-13): that condition is a follower load
evaluated on the displaced nodes, so the reactions Kratos reports after
the solve were those of the pressure on the deformed shape, off by first
order in the displacement (a 2 m plate deflecting 47 mm read 1e-4 of the
load as a spurious reaction, and the stresses depended on where the
support sat by 7e-5); as nodal loads the reactions balance to round-off.
Gravity and the inertial loads are a VOLUME_ACCELERATION set on every
node (the shells integrate it with their mass per unit area, exactly for
the linear fields used here).

MASS ITEMS (2026-11-13). ``masses`` lists concentrated masses that are not
part of the shell structure -- a battery, a motor, the payload -- each
with its mass, position and (optionally) its inertia tensor about its own
centre, and the structural nodes it hangs on. Each becomes a node of its
own carrying a NodalConcentratedElement (NODAL_MASS), tied to its
attachment nodes by an RBE3-type constraint: the mass node's displacement
is the weighted mean of the attachment nodes' displacements plus the
least-squares rotation of the attachment set times the offset, and its
rotation that rotation (Nastran's RBE3, translations of the attachment
nodes only, unit weights; a LinearMasterSlaveConstraint over the six
slave degrees of freedom). Such a constraint distributes the mass's
force to the attachment nodes by virtual work -- resultant force AND
moment preserved -- and adds NO stiffness to the structure, unlike a
rigid (RBE2) tie. The attachment set must not be collinear (the rotation
about the line would be undetermined); the application picks the nearest
few nodes of a body. In the result, ``mass`` is the structure's shells
plus the items (``mass_structure`` and ``mass_items`` separately).

HINGES AND PUSHRODS (2026-11-14). A control surface is its own body; how
it hands its load to the wing is three things in the job, all built on
the same constraint machinery as the mass items:
  ``ties``   [{master, slave}]: the slave node's three TRANSLATIONS equal
             the master's (an RBE2 on translations only -- the Nastran
             hinge bracket: the rotation about the hinge line stays free,
             so two brackets along a hinge make a hinge). The pair is a
             flap node and the main element's node nearest it.
  ``links``  [{name, on, at}]: an OFFSET node at `at`, rigidly tied to the
             shell node `on` -- u_off = u_on + theta_on x r (the shell
             node's rotations carry it): a control horn standing off the
             flap, a servo arm standing off the wing. Massless; its own
             rotations are pinned.
  ``bars``   [{name, a, b, area, youngs_modulus, density}]: a pin-ended rod
             (TrussLinearElement3D2N, TrussConstitutiveLaw -- the LINEAR
             truss: the geometrically nonlinear TrussElement3D2N, solved in
             one step, re-evaluates its residual on the displaced ends and
             left the reactions short of the loads by the order of the rod's
             weight on a wing with a 2 mm flap) between two ends,
             each {"node": i} (a structural node) or {"link": k} (an offset
             node): the pushrod from horn to servo arm. Its mass (rho A L)
             counts, lumped by the element's own linear shape functions,
             which the rigid-body bookkeeping matches (a uniform rod's
             second moments). The result reports each bar's axial force
             (positive tension, from the ends' displacements: E A dL / L),
             its stress and its length.
Without the pushrod a hinged flap is a mechanism and the solve is
singular (or returns displacements of 1e10): the rod is what holds the
surface, as it does on the aircraft. Checked on two plates (2026-11-14):
hinged by two ties the flap rotates freely; with a rod from a horn 50 mm
below the flap to a servo arm on a thick main plate it carries the hinge
moment over the horn arm.

SUPPORT. Two modes.

  CLAMPED (``supports``): every degree of freedom (displacements and
  rotations) of the nodes within ``tolerance`` of the plane ``axis =
  value`` is fixed; ``value`` may be "min" or "max" for the mesh's own
  extreme, and ``bounds`` limits the plane to a rectangular patch. The
  result reports ``support_nodes`` (and ``support_corner_nodes``, the same
  number: there are no mid-edge nodes any more, the key stays for the
  application). The reactions there are summed and reported against the
  applied loads: equal and opposite in a converged solution.

  FREE FLIGHT, inertia relief (``inertia_relief``, 2026-11-13): a flying
  airframe is held by nothing. The model is solved free-free the way
  Nastran's SUPORT / PARAM,INREL do it: the net force and moment of every
  applied load (pressures, point forces, gravity on the structure and the
  mass items) are turned into the six rigid-body accelerations the free
  body would have, a0 = F / m and alpha = I_cg^-1 M_cg, and the
  d'Alembert load of that motion, -m (a0 + alpha x (r - cg)) on every
  node and -I alpha on every mass item, is added so the load set is
  self-equilibrated. The singular stiffness is then removed by a
  statically determinate support: all six degrees of freedom of ONE node
  -- the structural node nearest ``reference`` (the mass centre when
  null) -- are fixed. Being statically determinate, that support cannot
  load the structure; its reaction force and moment are minus the
  residual of the whole load set, zero to round-off when the solve is
  right, and they are reported (``reaction``, ``reaction_moment``,
  ``balance_error``, ``moment_error``) as THE check of the case. The
  displacements are measured from that node (its translation and
  rotation are the datum); the stresses do not depend on which node it
  is. The accelerations themselves are reported
  (``rigid_acceleration``, ``rigid_angular_acceleration``, and
  ``load_factor_at_cg`` = the vertical inertial load the structure feels
  at its mass centre, in g): for a trimmed 1 g case they are zero and the
  load factor is 1; a lift that is not the weight shows up as a vertical
  acceleration, an aerodynamic moment about the mass centre as a pitch
  acceleration, and the stresses are those of that accelerating state --
  physical, but not the trimmed one. Gravity alone in free flight loads
  nothing (the body falls freely): stress comes from where the air loads
  act against where the mass sits.

RESULTS. Per node the displacement and rotation; per element the von
Mises stress (the largest over its top, middle and bottom surfaces),
averaged to the nodes for the VTK; per region the peak von Mises of the
skin and of the ribs and the margin allowable / peak - 1 (``per_material``,
kind "skin" or "rib").

COMPOSITE LAMINATES (2026-11-12). A region's material may be a LAMINATE:
its card carries ``plies``, the stack as laid from the OUTER face (the one
the face normal leaves) inward, every ply with its angle (degrees,
counter-clockwise about the outward normal from the region's REFERENCE
DIRECTION), thickness, density and the lamina's orthotropic figures
(e1, e2, nu12, g12, g13, g23; the strengths t1, c1, t2, c2, s12, s13, s23;
form "UD" or "Woven"; locking_angle). Such a region's shells get
StructuralMechanicsApplication's layered section -- the Properties carry
SHELL_ORTHOTROPIC_LAYERS, one row a ply [t, angle, rho, E1, E2, nu12, G12,
G13, G23 (, T1, C1, T2, C2, S12, S13, S23)], under the
LinearElasticOrthotropic2DLaw of KratosConstitutiveLawsApplication (pip
install it at Kratos's version) -- and no THICKNESS, YOUNG_MODULUS or
POISSON_RATIO, which Kratos refuses beside the layers; the plies' own
densities give the mass. Checked on Kratos 10.4.4 (2026-11-12): the first
row lies on the +normal side, and a positive ply angle turns
counter-clockwise about the normal, both as the application defines them.

The REFERENCE DIRECTION of every shell -- the fibre direction of a 0 degree
ply -- comes per face from ``axes`` (skin faces) and ``rib_axes`` (rib
faces): float64 xyz per face, in the faces' order, zero where the
application has none. The application takes it from the body's loft (the
gradient of the along-the-loft parameter in each triangle: a wing's span,
a fuselage's length, a laminator's rosette) and, on a planar member, from
its longest in-plane extent. A face without one takes the longest extent
of its region's nodes. Each is projected into the face's plane and set as
the element's LOCAL_MATERIAL_AXIS_1 before the elements initialise
(AnalysisStage.ModifyInitialGeometry), from which the shell computes its
MATERIAL_ORIENTATION_ANGLE. Without a laminate the axes change nothing (an
isotropic shell has no direction) and are not set.

DRAPE. A laminate card may name a ``drape_file``: float32 (deviation,
shear) in degrees per face of the region (in the faces' order) per ply, the
application's kinematic drape of each ply over the body (core drape.hpp: a
pin-jointed net; the deviation is the angle from the reference direction
to the draped warp, the shear how far warp and weft have closed from 90
degrees). Each face's stack then has its plies turned by their deviation
and, for a WOVEN ply, thickened by 1 / cos(shear) (areal weight is
conserved as a cloth shears; a tape cannot shear and keeps its thickness).
The stacks are binned to DRAPE_BIN_DEGREES (coarsened until a region has
at most MAX_STACK_BINS distinct stacks) and one Properties is written per
bin, its elements in a sub model part of their own (Skin_<id>_b<k>).

MARGINS of a laminate region are Tsai-Wu's (Tsai & Wu 1971, plane
stress, Kratos's TSAI_WU_RESERVE_FACTOR, the smallest over the plies and
the integration points of every element): ``per_material`` reports
"criterion" "Tsai-Wu", the smallest "reserve_factor" and "margin" =
reserve factor - 1, when every lamina has its five strengths; without
them no margin (0) and the criterion says so. An isotropic region's entry
reports "criterion" "von Mises" and its reserve factor allowable / peak,
as before. The VTK gains per cell "reserve_factor" (-1 where none),
"ply_shear_max" and "fibre_deviation_max" (degrees) when any region is a
laminate. Von Mises is still reported for every shell as a stress
magnitude; for a laminate it is not a failure measure.

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
# A laminate region's stacks: properties id = base + 1000 * region id + bin.
LAMINATE_SKIN_PROPERTIES_BASE = 100000
LAMINATE_RIB_PROPERTIES_BASE = 200000
MASS_PROPERTIES_ID = 900000      # the (empty) Properties of the mass items' concentrated elements
BAR_PROPERTIES_BASE = 910000     # a bar's Properties: base + its index (TrussConstitutiveLaw, CROSS_AREA, YOUNG_MODULUS, DENSITY)
DRAPE_BIN_DEGREES = 2.0          # the deviation and shear of a draped ply are rounded to this
MAX_STACK_BINS = 400             # distinct stacks per region before the bins are coarsened
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


def write_mdpa(path, nodes, shells, shell_prop, shell_part, support_nodes, loaded_nodes, mass_items, property_ids, shell_element, point_prop, links=(), bars=()):
    """The model: a shell element on every skin face and every rib face
    (`shells`, with `shell_prop` the Properties id per shell -- region +
    SHELL_PROPERTIES_OFFSET for a skin, + RIB_PROPERTIES_OFFSET for a rib
    -- and `shell_part` the sub model part each belongs to, Skin_<id> or
    Rib_<id>); a node and a NodalConcentratedElement per mass item after
    them (ids len(nodes) + 1 + k and len(shells) + 1 + k); a node per
    offset link after those; a TrussLinearElement3D2N per bar after the mass
    elements; a point-load condition on every node that carries a nodal
    load (the pressures' consistent loads and the point forces,
    `loaded_nodes`) and a point-moment condition on every mass node; and
    the sub model parts the materials, the parameters and the solve refer
    to (Skin_<id>, Rib_<id>, Support, Loaded, Masses, Offsets, Bar_<k>)."""
    n_shells = len(shells)
    n_masses = len(mass_items)
    n_links = len(links)
    link_base = len(nodes) + n_masses   # 0-based index of the first link node
    with open(path, "w", encoding="utf-8") as f:
        w = f.write
        w("Begin ModelPartData\nEnd ModelPartData\n\n")
        for pid in property_ids:
            w("Begin Properties %d\nEnd Properties\n" % int(pid))
        if n_masses:
            w("Begin Properties %d\nEnd Properties\n" % MASS_PROPERTIES_ID)
        for k in range(len(bars)):
            w("Begin Properties %d\nEnd Properties\n" % (BAR_PROPERTIES_BASE + k))
        w("\nBegin Nodes\n")
        for i, p in enumerate(nodes):
            w("%d %.17g %.17g %.17g\n" % (i + 1, p[0], p[1], p[2]))
        for k, item in enumerate(mass_items):
            p = item["at"]
            w("%d %.17g %.17g %.17g\n" % (len(nodes) + 1 + k, p[0], p[1], p[2]))
        for k, link in enumerate(links):
            p = link["at"]
            w("%d %.17g %.17g %.17g\n" % (link_base + 1 + k, p[0], p[1], p[2]))
        w("End Nodes\n\n")
        _write_block(w, "Elements", shell_element, shells, np.arange(n_shells), shell_prop)
        if n_masses:
            w("Begin Elements NodalConcentratedElement3D1N\n")
            for k in range(n_masses):
                w("%d %d %d\n" % (n_shells + 1 + k, MASS_PROPERTIES_ID, len(nodes) + 1 + k))
            w("End Elements\n\n")
        if bars:
            w("Begin Elements TrussLinearElement3D2N\n")
            for k, bar in enumerate(bars):
                w("%d %d %d %d\n" % (n_shells + n_masses + 1 + k, BAR_PROPERTIES_BASE + k, bar["a"] + 1, bar["b"] + 1))
            w("End Elements\n\n")
        if len(loaded_nodes):
            w("Begin Conditions PointLoadCondition3D1N\n")
            for i, node in enumerate(loaded_nodes):
                w("%d %d %d\n" % (1 + i, int(point_prop), int(node) + 1))
            w("End Conditions\n\n")
        if n_masses:
            w("Begin Conditions PointMomentCondition3D1N\n")
            for k in range(n_masses):
                w("%d %d %d\n" % (len(loaded_nodes) + 1 + k, int(point_prop), len(nodes) + 1 + k))
            w("End Conditions\n\n")
        parts = np.array(shell_part)
        for name in sorted(set(str(p) for p in parts)):
            sel = np.nonzero(parts == name)[0]
            _write_sub_model_part(w, name, np.unique(shells[sel]), sel)
        _write_sub_model_part(w, "Support", support_nodes)
        _write_sub_model_part(w, "Loaded", loaded_nodes, (), range(len(loaded_nodes)))
        if n_masses:
            _write_sub_model_part(w, "Masses", [len(nodes) + k for k in range(n_masses)], [n_shells + k for k in range(n_masses)],
                                  [len(loaded_nodes) + k for k in range(n_masses)])
        if n_links:
            _write_sub_model_part(w, "Offsets", [link_base + k for k in range(n_links)])
        for k, bar in enumerate(bars):
            _write_sub_model_part(w, "Bar_%d" % k, [bar["a"], bar["b"]], [n_shells + n_masses + k])


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


def _constraint_processes(support=True, offsets=False):
    """The clamp at the support, shared by the static and the modal solve:
    displacements and rotations alike (every node is a shell node). In
    free flight the Support part holds the one reference node; the modal
    solve of a free-flying model passes support=False (free-free). The
    offset link nodes' rotations have no stiffness and are pinned."""
    def pin(part, variable):
        return {
            "python_module": "assign_vector_variable_process",
            "kratos_module": "KratosMultiphysics",
            "Parameters": {
                "model_part_name": part,
                "variable_name": variable,
                "constrained": [True, True, True],
                "value": [0.0, 0.0, 0.0],
                "interval": [0.0, "End"],
            },
        }
    out = []
    if support:
        out += [pin("Structure.Support", v) for v in ("DISPLACEMENT", "ROTATION")]
    if offsets:
        out.append(pin("Structure.Offsets", "ROTATION"))
    return out


def modal_parameters(mdpa_base, materials_path, echo_level, count, eigensolver, support=True, offsets=False):
    """The eigenvalue analysis of the same model: the clamp, no loads, the
    lowest `count` modes of K phi = omega^2 M phi (free-free, rigid modes
    included, with support=False)."""
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
            "constraints_process_list": _constraint_processes(support, offsets),
            "loads_process_list": [],
            "list_other_processes": [],
        },
        "output_processes": {},
    }


def project_parameters(mdpa_base, materials_path, echo_level, linear_solver, offsets=False):
    """The static solve. The loads are not processes: every node's
    VOLUME_ACCELERATION, POINT_LOAD and POINT_MOMENT are set on the model
    directly (run_kratos), so the gravity, the inertial field, the
    pressures' nodal loads and the point forces are all one mechanism."""
    load_list = []
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
            # A linear analysis: the nodes stay where they are, and the
            # reactions are evaluated on the undeformed geometry.
            "move_mesh_flag": False,
            "linear_solver_settings": solver,
        },
        "processes": {
            "constraints_process_list": _constraint_processes(True, offsets),
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


def _analysis_class(element_axes):
    """StructuralMechanicsAnalysis with the elements' material axes set
    before they initialise: `element_axes` (n_elements, 3), a zero row
    leaving the element to Kratos's default (its first edge). The shell
    reads LOCAL_MATERIAL_AXIS_1 in its Initialize and turns it into its
    MATERIAL_ORIENTATION_ANGLE, so the value must be there before the
    solver initialises -- ModifyInitialGeometry is the hook between the
    model's import and that."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    from KratosMultiphysics.StructuralMechanicsApplication.structural_mechanics_analysis import StructuralMechanicsAnalysis

    class AerotelicAnalysis(StructuralMechanicsAnalysis):
        def ModifyInitialGeometry(self):
            super().ModifyInitialGeometry()
            if element_axes is None:
                return
            structure = self.model["Structure"]
            n_set = 0
            for elem in structure.Elements:
                a = element_axes[elem.Id - 1]
                if a[0] != 0.0 or a[1] != 0.0 or a[2] != 0.0:
                    elem.SetValue(SMA.LOCAL_MATERIAL_AXIS_1, KM.Vector([float(a[0]), float(a[1]), float(a[2])]))
                    n_set += 1
            log("  Kratos: material axes set on %d of %d elements" % (n_set, structure.NumberOfElements()))

    return AerotelicAnalysis


def _import_laminate_law():
    """The orthotropic ply law lives in KratosConstitutiveLawsApplication,
    a separate wheel: a laminate region needs it imported before the
    materials are read."""
    import KratosMultiphysics as KM
    try:
        import KratosMultiphysics.ConstitutiveLawsApplication  # noqa: F401
    except ImportError:
        version = ""
        try:
            version = " %s" % KM.KratosGlobals.Kernel.Version()
        except Exception:
            pass
        raise JobError("a composite laminate needs KratosConstitutiveLawsApplication, which is not installed: pip install KratosConstitutiveLawsApplication at Kratos's version%s in the same environment." % version)
    if not KM.KratosGlobals.HasConstitutiveLaw("LinearElasticOrthotropic2DLaw"):
        raise JobError("KratosConstitutiveLawsApplication is installed but does not register LinearElasticOrthotropic2DLaw (a version mismatch with Kratos?).")


def _apply_mass_items(structure, n_nodes, n_shells, mass_items, point_moments):
    """The mass items on the model Kratos has read: NODAL_MASS on each
    concentrated element, the inertial moment on each mass node, and the
    RBE3 constraint tying the mass node's six degrees of freedom to its
    attachment nodes' translations (see the docstring)."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    disp = [KM.DISPLACEMENT_X, KM.DISPLACEMENT_Y, KM.DISPLACEMENT_Z]
    rot = [KM.ROTATION_X, KM.ROTATION_Y, KM.ROTATION_Z]
    for k, item in enumerate(mass_items):
        node = structure.GetNode(n_nodes + 1 + k)
        elem = structure.GetElement(n_shells + 1 + k)
        elem.SetValue(KM.NODAL_MASS, float(item["mass"]))
        node.SetValue(KM.NODAL_MASS, float(item["mass"]))
        node.SetSolutionStepValue(SMA.POINT_MOMENT, KM.Vector([float(v) for v in point_moments[k]]))
        T = item["rbe3"]
        master_dofs = []
        for m in item["attach"]:
            mn = structure.GetNode(int(m) + 1)
            master_dofs += [mn.GetDof(v) for v in disp]
        slave_dofs = [node.GetDof(v) for v in disp + rot]
        Tm = KM.Matrix(6, T.shape[1])
        for i in range(6):
            for j in range(T.shape[1]):
                Tm[i, j] = float(T[i, j])
        structure.CreateNewMasterSlaveConstraint("LinearMasterSlaveConstraint", k + 1, master_dofs, slave_dofs, Tm, KM.Vector(6, 0.0))


def _apply_ties_and_links(structure, n_nodes, n_masses, ties, links, first_constraint_id):
    """The hinge ties (u_slave = u_master, translations) and the offset
    links (u_off = u_on + theta_on x r) on the model Kratos has read."""
    import KratosMultiphysics as KM
    disp = [KM.DISPLACEMENT_X, KM.DISPLACEMENT_Y, KM.DISPLACEMENT_Z]
    rot = [KM.ROTATION_X, KM.ROTATION_Y, KM.ROTATION_Z]
    cid = first_constraint_id
    for tie in ties:
        master, slave = structure.GetNode(int(tie["master"]) + 1), structure.GetNode(int(tie["slave"]) + 1)
        for v in disp:
            structure.CreateNewMasterSlaveConstraint("LinearMasterSlaveConstraint", cid, master, v, slave, v, 1.0, 0.0)
            cid += 1
    for k, link in enumerate(links):
        on = structure.GetNode(int(link["on"]) + 1)
        off = structure.GetNode(n_nodes + n_masses + 1 + k)
        r = np.array(link["at"], dtype=np.float64) - np.array([on.X0, on.Y0, on.Z0])
        T = np.hstack([np.eye(3), -_cross_matrix(r)])   # theta x r = -[r]x theta
        Tm = KM.Matrix(3, 6)
        for i in range(3):
            for j in range(6):
                Tm[i, j] = float(T[i, j])
        structure.CreateNewMasterSlaveConstraint("LinearMasterSlaveConstraint", cid, [on.GetDof(v) for v in disp + rot], [off.GetDof(v) for v in disp], Tm, KM.Vector(3, 0.0))
        cid += 1
    return cid


def run_kratos(job_dir, parameters, n_nodes, n_shells, nodal_load, nodal_acceleration, mass_items, mass_moments, echo_level, element_axes=None, needs_laminates=False, ties=(), links=()):
    """Builds and solves the Kratos model from the files already in
    job_dir. Returns the model part and the analysis for post-processing,
    with the solve time. `nodal_load` (n_nodes, 3) is the POINT_LOAD of
    every structural node, `nodal_acceleration` (n_nodes + n_masses +
    n_links, 3) the VOLUME_ACCELERATION of every node, mass nodes then
    link nodes last; `element_axes` the material axis per element (see
    _analysis_class), or None."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    try:
        # Registers sparse_lu (Eigen's SparseLU); amgcl is in the core.
        import KratosMultiphysics.LinearSolversApplication  # noqa: F401
    except ImportError:
        if parameters["solver_settings"]["linear_solver_settings"]["solver_type"] == "sparse_lu":
            raise JobError("the sparse_lu solver needs KratosLinearSolversApplication (pip install it at Kratos's version).")
    if needs_laminates:
        _import_laminate_law()

    if echo_level == 0:
        KM.Logger.GetDefaultOutput().SetSeverity(KM.Logger.Severity.WARNING)
    model = KM.Model()
    t_solve = time.time()
    log("  Kratos: reading the model")
    analysis = _analysis_class(element_axes)(model, KM.Parameters(json.dumps(parameters)))
    analysis.Initialize()
    structure = model["Structure"]
    n_all = n_nodes + len(mass_items) + len(links)
    if structure.NumberOfNodes() != n_all:
        raise JobError("Kratos read %d of %d nodes." % (structure.NumberOfNodes(), n_all))
    # The loads, on the nodes: the acceleration field (gravity and the
    # inertial field), the nodal loads (the pressures' consistent loads and
    # the point forces), and the mass items.
    for node in structure.Nodes:
        i = node.Id - 1
        a = nodal_acceleration[i]
        node.SetSolutionStepValue(KM.VOLUME_ACCELERATION, KM.Vector([float(a[0]), float(a[1]), float(a[2])]))
        if i < n_nodes:
            f = nodal_load[i]
            if f[0] != 0.0 or f[1] != 0.0 or f[2] != 0.0:
                node.SetSolutionStepValue(SMA.POINT_LOAD, KM.Vector([float(f[0]), float(f[1]), float(f[2])]))
    if mass_items:
        _apply_mass_items(structure, n_nodes, n_shells, mass_items, mass_moments)
    if ties or links:
        _apply_ties_and_links(structure, n_nodes, len(mass_items), ties, links, len(mass_items) + 1)
    log("  Kratos: model read in %.1f s; assembling and solving with %s" % (time.time() - t_solve, parameters["solver_settings"]["linear_solver_settings"]["solver_type"]))
    analysis.RunSolutionLoop()
    analysis.Finalize()
    log("  Kratos: solved in %.1f s; reading the solution" % (time.time() - t_solve))
    return model, structure, analysis, time.time() - t_solve


def run_modes(job_dir, mdpa_base, materials_path, n_nodes, n_shells, mass_items, echo_level, count, eigensolver, element_axes=None, support=True, ties=(), links=()):
    """The lowest `count` natural frequencies (Hz, ascending) and mode shapes
    ((count, n_nodes, 3), each scaled to a unit largest displacement) of the
    model already written to mdpa_base, clamped as the static solve was
    (or free-free with support=False: the six rigid modes come out at
    about 0 Hz and are dropped, `count` flexible modes are returned), its
    elements' material axes set as the static solve's were, its mass
    items hung on as in the static solve. Spectra first; the subspace
    solver if it fails."""
    import KratosMultiphysics as KM
    import KratosMultiphysics.StructuralMechanicsApplication as SMA
    StructuralMechanicsAnalysis = _analysis_class(element_axes)
    import KratosMultiphysics.LinearSolversApplication  # noqa: F401  (the eigensolvers)
    if echo_level == 0:
        KM.Logger.GetDefaultOutput().SetSeverity(KM.Logger.Severity.WARNING)
    solvers = [eigensolver] + [s for s in ("spectra_sym_g_eigs_shift", "eigen_eigensystem") if s != eigensolver]
    n_rigid = 0 if support else 6
    n_all = n_nodes + len(mass_items) + len(links)
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
                modal_parameters(mdpa_base, materials_path, echo_level, count + n_rigid, solver, support, bool(links)))))
            analysis.Initialize()
            structure = model["Structure"]
            if mass_items:
                _apply_mass_items(structure, n_nodes, n_shells, mass_items, np.zeros((len(mass_items), 3)))
            if ties or links:
                _apply_ties_and_links(structure, n_nodes, len(mass_items), ties, links, len(mass_items) + 1)
            analysis.RunSolutionLoop()
            analysis.Finalize()
        except Exception as exc:  # a solver that is not built in, or did not converge
            last = "%s: %s" % (solver, exc)
            log("  modes: %s failed (%s)" % (solver, exc))
            continue
        structure = model["Structure"]
        values = structure.ProcessInfo[SMA.EIGENVALUE_VECTOR]
        lam = np.array([float(values[i]) for i in range(len(values))])
        shapes = np.zeros((len(lam), n_all, 3))
        for node in structure.Nodes:
            m = node.GetValue(SMA.EIGENVECTOR_MATRIX)
            rows = min(m.Size1(), len(lam))
            cols = cols6 if m.Size2() == 6 else (cols3 if m.Size2() == 3 else [0, 1, 2])
            for k in range(rows):
                shapes[k, node.Id - 1] = (m[k, cols[0]], m[k, cols[1]], m[k, cols[2]])
        order = np.argsort(lam)
        lam, shapes = lam[order], shapes[order][:, :n_nodes]
        # omega^2 to hertz; a tiny negative from round-off on a rigid mode is 0.
        frequencies = np.sqrt(np.maximum(lam, 0.0)) / (2.0 * np.pi)
        if n_rigid:
            # Free-free: the rigid modes are the ones at (numerically) zero
            # frequency, well below the first flexible one. Drop them, and
            # complain if they are not there to drop.
            flexible = frequencies > 1e-3 * max(float(frequencies.max()), 1e-300)
            if int((~flexible).sum()) != n_rigid:
                log("  modes: free-free solve found %d rigid mode(s) at ~0 Hz, not %d; frequencies %s" % (int((~flexible).sum()), n_rigid, np.round(frequencies, 4)))
            frequencies, shapes = frequencies[flexible][:count], shapes[flexible][:count]
        for k in range(len(frequencies)):
            peak = float(np.linalg.norm(shapes[k], axis=1).max())
            if peak > 0.0:
                shapes[k] /= peak
        return frequencies, shapes, solver
    raise JobError("the natural frequencies could not be computed: %s" % last)


def read_nodal(structure, n_nodes, n_masses=0, n_links=0):
    """Displacements and rotations of the structural nodes, the reaction
    force and moment of every node of the Support part, and the
    displacements of the extra nodes (mass nodes then link nodes)."""
    import KratosMultiphysics as KM
    displacement = np.zeros((n_nodes, 3))
    rotation = np.zeros((n_nodes, 3))
    mass_displacement = np.zeros((n_masses + n_links, 3))
    for node in structure.Nodes:
        i = node.Id - 1
        u = node.GetSolutionStepValue(KM.DISPLACEMENT)
        if i < n_nodes:
            displacement[i] = (u[0], u[1], u[2])
            r = node.GetSolutionStepValue(KM.ROTATION)
            rotation[i] = (r[0], r[1], r[2])
        else:
            mass_displacement[i - n_nodes] = (u[0], u[1], u[2])
    # The reactions at the support. Read from the Support sub model part
    # rather than by IsFixed(): the constraint process frees the degrees
    # of freedom again once the step is over, so nothing is fixed by now.
    reaction = np.zeros((n_nodes, 3))
    reaction_moment = np.zeros((n_nodes, 3))
    for node in structure.GetSubModelPart("Support").Nodes:
        r = node.GetSolutionStepValue(KM.REACTION)
        reaction[node.Id - 1] = (r[0], r[1], r[2])
        m = node.GetSolutionStepValue(KM.REACTION_MOMENT)
        reaction_moment[node.Id - 1] = (m[0], m[1], m[2])
    return displacement, rotation, reaction, reaction_moment, mass_displacement


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
        if elem.Id > n_elements:
            continue   # a mass item's concentrated element: no stress
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
    if m.get("plies"):
        for i, p in enumerate(m["plies"]):
            who = "'%s' ply %d" % (label, i + 1)
            if not (float(p.get("thickness", 0.0)) > 0.0):
                raise JobError("%s has no thickness." % who)
            if not (float(p.get("density", 0.0)) > 0.0):
                raise JobError("%s has no density." % who)
            if not (float(p.get("e1", 0.0)) > 0.0 and float(p.get("e2", 0.0)) > 0.0 and float(p.get("g12", 0.0)) > 0.0):
                raise JobError("%s has no stiffness (E1, E2 and G12 must be positive)." % who)
            nu = float(p.get("nu12", 0.0))
            if not (0.0 <= nu and 1.0 - nu * nu * float(p["e2"]) / float(p["e1"]) > 0.0):
                raise JobError("%s has an inadmissible nu12 of %g." % (who, nu))
        return
    if not (float(m["youngs_modulus"]) > 0.0 and float(m["density"]) > 0.0):
        raise JobError("'%s' has no stiffness or no density." % label)
    nu = float(m["poisson_ratio"])
    if not (0.0 <= nu < 0.5):
        raise JobError("'%s' has a Poisson's ratio of %g; it must be in [0, 0.5)." % (label, nu))


# ---------------------------------------------------------------------------
# Laminates
# ---------------------------------------------------------------------------

def _stack_from_card(material):
    """The plies of a laminate card as the layer rows want them: a list of
    dicts with t, angle, rho, e1, e2, nu12, g12, g13, g23, the strengths
    (zero when unknown), woven and locking_angle. g13 defaults to g12, g23
    to g12 / 2, s13 and s23 to s12 -- the lamina's own conventions
    (materials.hpp)."""
    stack = []
    for p in material["plies"]:
        g12 = float(p["g12"])
        s12 = float(p.get("s12", 0.0))
        stack.append({
            "t": float(p["thickness"]), "angle": float(p.get("angle", 0.0)), "rho": float(p["density"]),
            "e1": float(p["e1"]), "e2": float(p["e2"]), "nu12": float(p.get("nu12", 0.0)), "g12": g12,
            "g13": float(p.get("g13", 0.0)) or g12, "g23": float(p.get("g23", 0.0)) or 0.5 * g12,
            "t1": float(p.get("t1", 0.0)), "c1": float(p.get("c1", 0.0)), "t2": float(p.get("t2", 0.0)), "c2": float(p.get("c2", 0.0)),
            "s12": s12, "s13": float(p.get("s13", 0.0)) or s12, "s23": float(p.get("s23", 0.0)) or s12,
            "woven": str(p.get("form", "UD")) == "Woven", "locking_angle": float(p.get("locking_angle", 0.0)),
        })
    return stack


def _stack_has_strengths(stack):
    return all(p["t1"] > 0.0 and p["c1"] > 0.0 and p["t2"] > 0.0 and p["c2"] > 0.0 and p["s12"] > 0.0 for p in stack)


def _read_drape(job_dir, name, n_faces, n_plies):
    """The drape file of a laminate region: float32 (deviation, shear) in
    degrees per face per ply, or None when the card names none."""
    if not name:
        return None
    data = np.fromfile(os.path.join(job_dir, name), dtype="<f4")
    if len(data) != n_faces * n_plies * 2:
        raise JobError("the drape file %s holds %d values; %d faces x %d plies x 2 were expected." % (name, len(data), n_faces, n_plies))
    return data.reshape(n_faces, n_plies, 2).astype(np.float64)


def _bin_stacks(stack, drape, n_faces):
    """Every face's stack as laid: the plies turned by their drape deviation
    and, woven, thickened by 1 / cos(shear); binned to DRAPE_BIN_DEGREES
    (coarsened until at most MAX_STACK_BINS distinct stacks). Returns
    (bin per face, the layer rows per bin, rho t per face, stats)."""
    n_plies = len(stack)
    if drape is None:
        dev = np.zeros((n_faces, n_plies))
        shear = np.zeros((n_faces, n_plies))
    else:
        dev = drape[:, :, 0]
        shear = np.abs(drape[:, :, 1])
    woven = np.array([p["woven"] for p in stack], dtype=bool)
    shear_eff = np.where(woven[None, :], np.minimum(shear, 80.0), 0.0)   # a tape does not shear
    has_strengths = _stack_has_strengths(stack)
    bin_deg = DRAPE_BIN_DEGREES
    while True:
        qd = np.round(dev / bin_deg) * bin_deg
        qs = np.round(shear_eff / bin_deg) * bin_deg
        keys = np.concatenate([qd, qs], axis=1) if n_faces else np.zeros((0, 2 * n_plies))
        uniq, inverse = np.unique(keys, axis=0, return_inverse=True) if n_faces else (np.zeros((1, 2 * n_plies)), np.zeros(0, dtype=np.int64))
        if len(uniq) <= MAX_STACK_BINS or bin_deg >= 90.0:
            break
        bin_deg = {2.0: 5.0, 5.0: 10.0, 10.0: 15.0, 15.0: 30.0}.get(bin_deg, 90.0)
    inverse = np.asarray(inverse).reshape(-1)
    rows_per_bin = []
    rho_t_per_bin = np.zeros(len(uniq))
    for k, key in enumerate(uniq):
        rows = []
        for i, p in enumerate(stack):
            t = p["t"] / np.cos(np.radians(key[n_plies + i]))
            row = [t, p["angle"] + key[i], p["rho"], p["e1"], p["e2"], p["nu12"], p["g12"], p["g13"], p["g23"]]
            if has_strengths:
                row += [p["t1"], p["c1"], p["t2"], p["c2"], p["s12"], p["s13"], p["s23"]]
            rows.append([float(v) for v in row])
            rho_t_per_bin[k] += p["rho"] * t
        rows_per_bin.append(rows)
    rho_t = rho_t_per_bin[inverse] if n_faces else np.zeros(0)
    past_locking = 0
    if n_faces and drape is not None:
        for i, p in enumerate(stack):
            if p["woven"] and p["locking_angle"] > 0.0:
                past_locking += int((shear[:, i] > p["locking_angle"]).sum())
    stats = {
        "plies": n_plies, "thickness": float(sum(p["t"] for p in stack)), "has_strengths": has_strengths,
        "bins": int(len(uniq)), "bin_degrees": float(bin_deg), "draped": drape is not None,
        "max_shear": float(shear.max()) if (n_faces and drape is not None) else 0.0,
        "max_deviation": float(np.abs(dev).max()) if (n_faces and drape is not None) else 0.0,
        "faces_past_locking": past_locking,
        "shear_per_face": shear.max(axis=1) if n_faces else np.zeros(0),
        "deviation_per_face": np.abs(dev).max(axis=1) if n_faces else np.zeros(0),
    }
    return inverse, rows_per_bin, rho_t, stats


def _longest_axis(points):
    """The unit axis of the longest bounding-box extent of a point set."""
    if len(points) == 0:
        return np.array([1.0, 0.0, 0.0])
    extent = points.max(axis=0) - points.min(axis=0)
    axis = np.zeros(3)
    axis[int(np.argmax(extent))] = 1.0
    return axis


def _project_axes(nodes, tris, axes, fallback):
    """Each face's axis projected into its plane and normalised; a zero or
    degenerate one takes the fallback vector (then the next coordinate
    axis if that too is normal to the face). Returns (n, 3)."""
    out = np.zeros((len(tris), 3))
    if len(tris) == 0:
        return out
    n = face_normals_area(nodes, tris)
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-300)[:, None]
    candidates = [axes if axes is not None else np.zeros((len(tris), 3)), np.tile(fallback, (len(tris), 1)),
                  np.tile(np.roll(fallback, 1), (len(tris), 1)), np.tile(np.roll(fallback, 2), (len(tris), 1))]
    done = np.zeros(len(tris), dtype=bool)
    for cand in candidates:
        proj = cand - n * np.einsum("ij,ij->i", cand, n)[:, None]
        length = np.linalg.norm(proj, axis=1)
        ok = (~done) & (length > 1e-6 * np.maximum(np.linalg.norm(cand, axis=1), 1e-300)) & (np.linalg.norm(cand, axis=1) > 0.0)
        out[ok] = proj[ok] / length[ok][:, None]
        done |= ok
    return out


def _regions_from_job(job):
    """The regions: [{"id", "label", "skin": null | {...}, "rib": null | {...}}]."""
    return [{"id": int(r["id"]), "label": r.get("label", ""), "skin": r.get("skin"), "rib": r.get("rib")}
            for r in job.get("regions", [])]


# ---------------------------------------------------------------------------
# Mass items, rigid-body mass properties, inertia relief
# ---------------------------------------------------------------------------

def _cross_matrix(d):
    """[d]x, the matrix of d x (.)"""
    return np.array([[0.0, -d[2], d[1]], [d[2], 0.0, -d[0]], [-d[1], d[0], 0.0]])


def rbe3_rows(masters, reference, weights=None):
    """The RBE3 relation [u_ref; theta_ref] = T [u_1; ...; u_n] over the
    translations u_i of the attachment nodes at `masters` (n, 3), for a
    reference node at `reference`: u_ref = u_bar + theta_bar x (ref -
    r_bar), theta_ref = theta_bar, with u_bar the weighted mean of the
    u_i, r_bar the weighted centroid and theta_bar = J^-1 sum w_i d_i x
    u_i the least-squares rotation of the set (d_i = r_i - r_bar, J =
    sum w_i (|d_i|^2 I - d_i d_i^T), unit weights unless given). Returns
    (T (6, 3n), r_bar), or (None, None) when the set is collinear (J
    singular: the rotation about the line is undetermined)."""
    masters = np.asarray(masters, dtype=np.float64)
    n = len(masters)
    w = np.ones(n) if weights is None else np.asarray(weights, dtype=np.float64)
    W = float(w.sum())
    rbar = (w[:, None] * masters).sum(axis=0) / W
    d = masters - rbar
    J = np.zeros((3, 3))
    for i in range(n):
        J += w[i] * (np.dot(d[i], d[i]) * np.eye(3) - np.outer(d[i], d[i]))
    sv = np.linalg.svd(J, compute_uv=False)
    if sv[0] <= 0.0 or sv[-1] <= 1e-9 * sv[0]:
        return None, None
    Jinv = np.linalg.inv(J)
    T_theta = np.zeros((3, 3 * n))
    T_u = np.zeros((3, 3 * n))
    for i in range(n):
        T_theta[:, 3 * i:3 * i + 3] = Jinv @ (w[i] * _cross_matrix(d[i]))
        T_u[:, 3 * i:3 * i + 3] = (w[i] / W) * np.eye(3)
    T_u -= _cross_matrix(np.asarray(reference, dtype=np.float64) - rbar) @ T_theta
    return np.vstack([T_u, T_theta]), rbar


def shell_mass_moments(nodes, tris, rho_t):
    """(mass, first moment, second-moment matrix S = integral r r^T dm
    about the origin) of shells of areal density `rho_t` per face, exact
    for a uniform lamina on each flat triangle (what Kratos's shells
    integrate their body load with, to the same order)."""
    if len(tris) == 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    A, B, C = nodes[tris[:, 0]], nodes[tris[:, 1]], nodes[tris[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
    m = np.asarray(rho_t, dtype=np.float64) * area
    cen = (A + B + C) / 3.0
    first = (m[:, None] * cen).sum(axis=0)
    S = np.zeros((3, 3))
    for P in (A, B, C):
        S += np.einsum("i,ij,ik->jk", m / 12.0, P, P)
    S += np.einsum("i,ij,ik->jk", 9.0 * m / 12.0, cen, cen)
    return float(m.sum()), first, S


def _inertia_about(S, mass, first, point):
    """The inertia tensor about `point` from the second-moment matrix
    about the origin: shift S, then I = tr(S') 1 - S'."""
    a = np.asarray(point, dtype=np.float64)
    Sp = S - np.outer(first, a) - np.outer(a, first) + mass * np.outer(a, a)
    return np.trace(Sp) * np.eye(3) - Sp


def _mass_items_from_job(job, nodes):
    """The mass items: [{"name", "mass" kg, "at" (3,), "inertia" (3, 3)
    about the item's own centre, "attach" the attachment node indices,
    "rbe3" the constraint rows}]; validated."""
    items = []
    for k, m in enumerate(job.get("masses") or []):
        name = str(m.get("name") or "mass %d" % (k + 1))
        mass = float(m.get("mass", 0.0))
        if not mass > 0.0:
            raise JobError("mass item '%s' has no mass." % name)
        at = np.array(m["at"], dtype=np.float64).reshape(3)
        inertia = np.zeros((3, 3))
        if m.get("inertia") is not None:
            ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in m["inertia"])
            inertia = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
            if np.linalg.eigvalsh(inertia).min() < -1e-12 * max(1.0, abs(inertia).max()):
                raise JobError("mass item '%s' has an inertia tensor that is not positive semi-definite." % name)
        attach = np.array(sorted(set(int(v) for v in (m.get("attach") or []))), dtype=np.int64)
        if len(attach) < 3:
            raise JobError("mass item '%s' hangs on %d node(s); an attachment needs at least three, not in a line." % (name, len(attach)))
        if attach.min() < 0 or attach.max() >= len(nodes):
            raise JobError("mass item '%s' hangs on nodes the mesh does not have." % name)
        T, rbar = rbe3_rows(nodes[attach], at)
        if T is None:
            raise JobError("mass item '%s' hangs on %d nodes that lie on a line: the attachment cannot carry a moment about it. Attach it to more nodes." % (name, len(attach)))
        items.append({"name": name, "mass": mass, "at": at, "inertia": inertia, "attach": attach, "rbe3": T, "attach_centre": rbar})
    return items


def rigid_body_properties(nodes, faces, skin_density_t, rib_faces, rib_density_t, mass_items, bars=(), bar_ends=None):
    """Total mass, mass centre and inertia tensor about it of the shells
    (exact laminae), the mass items (points with their own tensors) and
    the bars (uniform rods between their ends, `bar_ends` (n, 2, 3))."""
    m_s, f_s, S_s = shell_mass_moments(nodes, faces, skin_density_t)
    m_r, f_r, S_r = shell_mass_moments(nodes, rib_faces, rib_density_t)
    mass = m_s + m_r
    first = f_s + f_r
    S = S_s + S_r
    own = np.zeros((3, 3))
    for item in mass_items:
        mass += item["mass"]
        first += item["mass"] * item["at"]
        S += item["mass"] * np.outer(item["at"], item["at"])
        own += item["inertia"]
    for k, bar in enumerate(bars):
        a, b = bar_ends[k]
        m = bar["density"] * bar["area"] * float(np.linalg.norm(b - a))
        mass += m
        first += m * 0.5 * (a + b)
        # A uniform rod: integral r r^T dm = (m/3)(a a^T + b b^T) + (m/6)(a b^T + b a^T).
        S += (m / 3.0) * (np.outer(a, a) + np.outer(b, b)) + (m / 6.0) * (np.outer(a, b) + np.outer(b, a))
    if not mass > 0.0:
        raise JobError("the model has no mass.")
    cg = first / mass
    inertia = _inertia_about(S, mass, first, cg) + own
    return mass, cg, inertia


def inertia_relief(nodes, nodal_load, mass_items, mass, cg, inertia, gravity, links=()):
    """The rigid-body accelerations of the free body under the nodal loads
    and gravity: a0 = F / m at the mass centre, alpha = I_cg^-1 M_cg.
    Returns (a0, alpha, the acceleration field g - a0 - alpha x (r - cg) at
    every structural node, then every mass node, then every link node, the
    moment -I_item alpha on each item)."""
    force = nodal_load.sum(axis=0) + mass * gravity
    moment = np.cross(nodes - cg, nodal_load).sum(axis=0)
    a0 = force / mass
    alpha = np.linalg.solve(inertia, moment) if np.linalg.cond(inertia) < 1e12 else np.linalg.lstsq(inertia, moment, rcond=None)[0]
    extra = [it["at"] for it in mass_items] + [np.array(l["at"], dtype=np.float64) for l in links]
    points = np.concatenate([nodes, np.array(extra).reshape(-1, 3)]) if extra else nodes
    field = gravity[None, :] - a0[None, :] - np.cross(alpha[None, :], points - cg)
    moments = np.array([-(it["inertia"] @ alpha) for it in mass_items]).reshape(-1, 3)
    return a0, alpha, field, moments


def _ties_links_bars_from_job(job, nodes, n_masses):
    """The hinge ties, offset links and bars of the job, validated; a bar's
    ends resolved to node indices (a link k is node len(nodes) + n_masses
    + k) with its end positions."""
    ties = []
    for t in job.get("ties") or []:
        master, slave = int(t["master"]), int(t["slave"])
        if not (0 <= master < len(nodes) and 0 <= slave < len(nodes)):
            raise JobError("a hinge tie names nodes the mesh does not have.")
        if master == slave:
            raise JobError("a hinge tie ties a node to itself.")
        ties.append({"master": master, "slave": slave})
    links = []
    for k, l in enumerate(job.get("links") or []):
        on = int(l["on"])
        if not (0 <= on < len(nodes)):
            raise JobError("offset link '%s' stands off a node the mesh does not have." % l.get("name", k))
        links.append({"name": str(l.get("name") or "link %d" % (k + 1)), "on": on, "at": np.array(l["at"], dtype=np.float64).reshape(3)})
    bars = []
    ends = []
    for k, b in enumerate(job.get("bars") or []):
        name = str(b.get("name") or "bar %d" % (k + 1))
        def end(e):
            if "link" in e:
                li = int(e["link"])
                if not (0 <= li < len(links)):
                    raise JobError("bar '%s' ends on offset link %d, which the job does not have." % (name, li))
                return len(nodes) + n_masses + li, links[li]["at"]
            ni = int(e["node"])
            if not (0 <= ni < len(nodes)):
                raise JobError("bar '%s' ends on a node the mesh does not have." % name)
            return ni, nodes[ni]
        ia, pa = end(b["a"])
        ib, pb = end(b["b"])
        if ia == ib or np.linalg.norm(pb - pa) <= 0.0:
            raise JobError("bar '%s' has no length." % name)
        for key in ("area", "youngs_modulus", "density"):
            if not float(b.get(key, 0.0)) > 0.0:
                raise JobError("bar '%s' has no %s." % (name, key.replace("_", " ")))
        bars.append({"name": name, "a": ia, "b": ib, "area": float(b["area"]), "youngs_modulus": float(b["youngs_modulus"]), "density": float(b["density"])})
        ends.append((pa, pb))
    return ties, links, bars, np.array(ends).reshape(-1, 2, 3)


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
        axes          null, or float64 xyz per skin face (faces order): the
        rib_axes      material reference direction of each face (and the
                      same for the rib faces); see COMPOSITE LAMINATES above
        regions[].skin / rib may carry "drape_file": float32 (deviation,
                      shear) degrees per face of the region per ply of a
                      laminate material, and a material card may carry
                      "plies" (the laminate's stack, see above)
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
                      (axis + 2) % 3) when bounds are given. Empty or
                      absent in free flight.
        ties          [{"master": node, "slave": node}] -- hinge brackets
                      (see HINGES AND PUSHRODS above)
        links         [{"name", "on": node, "at": [x,y,z]}] -- offset nodes
        bars          [{"name", "a": {"node": i} | {"link": k}, "b": ...,
                      "area" m^2, "youngs_modulus" Pa, "density" kg/m^3}]
        inertia_relief  null (clamped), or {"reference": null | [x,y,z]}:
                      FREE FLIGHT (see SUPPORT above) -- the loads are
                      balanced by the rigid-body inertial field and the
                      structural node nearest the reference (the mass
                      centre when null) is the statically determinate
                      support whose reactions are the check.
        masses        [{"name", "mass": kg, "at": [x,y,z], "inertia": null |
                      [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] (kg m^2 about the
                      item's own centre, the products with the sign of
                      integral x y dm), "attach": [node indices, >= 3, not
                      collinear]}] -- the MASS ITEMS (see above)
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

    The result reports the model's size ("nodes" the structural nodes,
    "elements" shells plus mass elements, "dofs" -- six per node --,
    "shells" all the shells, "rib_shells" those of them that are ribs,
    "wetted_faces", "support_nodes" and "support_corner_nodes", the same
    number), the mass ("mass" = "mass_structure" + "mass_items",
    "mass_centre", "inertia_cg" the 3 x 3 tensor about it), the peak
    displacement and where ("max_displacement", "max_displacement_at"),
    the peak von Mises and whose ("max_von_mises",
    "max_von_mises_material"), "per_material" entries of kind "skin" or
    "rib" with their peak, margin, allowable and mass, "mass_items" as a
    list ("items": [{"name", "mass", "at", "node", "attach_nodes",
    "displacement", "force" the force it puts on the structure}]), the
    force balance -- "reaction", "reaction_moment" (about the support
    node, free flight), "applied_gravity", "applied_pressure",
    "applied_point", "applied_inertial" (the d'Alembert resultant, zero
    when clamped), "balance_error" = |sum of reactions + applied loads|
    over the loads' magnitude, "moment_error" likewise for the moment
    over the loads times the mesh diagonal (free flight), "balanced" when
    within BALANCE_TOLERANCE --, the support ("support_mode" "clamped" |
    "free flight", "support_node" and "support_at" in free flight), the
    rigid-body motion ("rigid_acceleration" m/s^2, "rigid_angular_
    acceleration" rad/s^2, "load_factor_at_cg" g; zero, zero and the
    load factor when clamped), a "warnings" list, the solver used
    ("linear_solver"), "mean_normal_displacement", and the times
    ("seconds", "seconds_solve", "seconds_modes"). An unbalanced solve is
    a solver that did not converge, and its fields are not to be trusted.
    With modes > 0 it also reports "natural_frequencies" (Hz, lowest
    first; empty with a warning if the eigensolve failed; free-free with
    the rigid modes dropped in free flight) and "eigensolver", and the
    VTK carries the mode shapes as "mode_1" ... point vectors. A failure
    writes {"ok": false, "error": ...} and exits 1.
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
    relief = job.get("inertia_relief")
    free_flight = relief is not None
    if not supports and not free_flight:
        raise JobError("no support: give at least one plane whose nodes are fixed, or ask for free flight (inertia relief).")
    if supports and free_flight:
        raise JobError("free flight takes no support planes: the model is held by its own inertia and one reference node.")
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

    lo, hi = nodes.min(axis=0), nodes.max(axis=0)
    diagonal = float(np.linalg.norm(hi - lo))
    if free_flight:
        support_nodes, support_planes = np.zeros(0, dtype=np.int64), []
    else:
        support_nodes, support_planes = _support_nodes(nodes, supports)
    mass_items = _mass_items_from_job(job, nodes)
    ties, links, bars, bar_ends = _ties_links_bars_from_job(job, nodes, len(mass_items))

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

    # The nodal loads: the pressures' consistent loads (-p n A, a third to
    # each corner of the face) and the point forces.
    nodal_load = np.zeros((len(nodes), 3))
    if len(faces):
        face_force = -(face_pressure[:, None] * n_area)
        for k in range(3):
            np.add.at(nodal_load, faces[:, k], face_force / 3.0)
    for node, force in point_loads:
        nodal_load[node] += force
    loaded_nodes = np.nonzero(np.any(nodal_load != 0.0, axis=1))[0]

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
        "support_planes": support_planes,
        "point_nodes": [int(n) for n, _ in point_loads],
        "applied_point": [float(v) for v in applied_point],
        "support_mode": "free flight" if free_flight else "clamped",
    }

    # The materials: a plane-stress law per isotropic skin and rib, a
    # layered section per laminate (one Properties per distinct draped
    # stack, see the docstring), and rho t per face for the mass.
    material_entries = []
    property_ids = []
    skin_density_t = np.zeros(len(faces))     # rho t per skin face
    rib_density_t = np.zeros(len(rib_faces))  # rho t per rib face
    skin_prop = region_of_face + SHELL_PROPERTIES_OFFSET
    rib_prop = rib_region + RIB_PROPERTIES_OFFSET
    skin_part = ["Skin_%d" % int(rid) for rid in region_of_face]
    rib_part = ["Rib_%d" % int(rid) for rid in rib_region]
    laminates = {}   # (region id, "skin" | "rib") -> the stack's stats
    needs_laminates = False
    for rid in ids:
        r = by_id[rid]
        for kind in ("skin", "rib"):
            card = r.get(kind)
            if not card:
                continue
            is_rib = kind == "rib"
            region_array = rib_region if is_rib else region_of_face
            sel = np.nonzero(region_array == rid)[0]
            density_t = rib_density_t if is_rib else skin_density_t
            props = rib_prop if is_rib else skin_prop
            parts = rib_part if is_rib else skin_part
            prefix = "Rib" if is_rib else "Skin"
            m = card["material"]
            t = float(card["thickness"])
            if m.get("plies"):
                needs_laminates = True
                stack = _stack_from_card(m)
                drape = _read_drape(job_dir, card.get("drape_file"), len(sel), len(stack))
                bins, rows_per_bin, rho_t, stats = _bin_stacks(stack, drape, len(sel))
                base = (LAMINATE_RIB_PROPERTIES_BASE if is_rib else LAMINATE_SKIN_PROPERTIES_BASE) + 1000 * rid
                for k, rows in enumerate(rows_per_bin):
                    material_entries.append(("%s_%d_b%d" % (prefix, rid, k), base + k, "LinearElasticOrthotropic2DLaw", {"SHELL_ORTHOTROPIC_LAYERS": rows}))
                    property_ids.append(base + k)
                props[sel] = base + bins
                for f, k in zip(sel, bins):
                    parts[f] = "%s_%d_b%d" % (prefix, rid, int(k))
                density_t[sel] = rho_t
                stats["faces"] = sel
                laminates[(rid, kind)] = stats
                log("  %s %s: a laminate of %d plies (%.4g m) in %d draped stack(s)%s%s" % (
                    r.get("label", rid), kind, stats["plies"], stats["thickness"], stats["bins"],
                    "" if not stats["draped"] else ", shear up to %.1f deg, fibres turned up to %.1f deg" % (stats["max_shear"], stats["max_deviation"]),
                    "" if stats["has_strengths"] else " (no strengths: no margin)"))
            else:
                density_t[sel] = float(m["density"]) * t
                material_entries.append(("%s_%d" % (prefix, rid), rid + (RIB_PROPERTIES_OFFSET if is_rib else SHELL_PROPERTIES_OFFSET), "LinearElasticPlaneStress2DLaw",
                                         {"DENSITY": float(m["density"]), "YOUNG_MODULUS": float(m["youngs_modulus"]),
                                          "POISSON_RATIO": float(m["poisson_ratio"]), "THICKNESS": t}))
                property_ids.append(rid + (RIB_PROPERTIES_OFFSET if is_rib else SHELL_PROPERTIES_OFFSET))
    mass_structure = float((skin_density_t * face_area).sum()) + float((rib_density_t * rib_area).sum())
    mass_of_items = float(sum(it["mass"] for it in mass_items))
    mass_of_bars = float(sum(b["density"] * b["area"] * float(np.linalg.norm(bar_ends[k][1] - bar_ends[k][0])) for k, b in enumerate(bars)))
    mass, cg, inertia_cg = rigid_body_properties(nodes, faces, skin_density_t, rib_faces, rib_density_t, mass_items, bars, bar_ends)
    if abs(mass - mass_structure - mass_of_items - mass_of_bars) > 1e-9 * mass:
        raise JobError("the mass did not add up (%.9g against %.9g + %.9g + %.9g)." % (mass, mass_structure, mass_of_items, mass_of_bars))

    # Gravity, and in free flight the inertial field that balances the
    # whole load set; the support in free flight is the structural node
    # nearest the reference point.
    g = float(loads["load_factor"]) * G0
    gravity = np.array([0.0, 0.0, -g])
    if free_flight:
        reference = cg if (relief.get("reference") is None) else np.array(relief["reference"], dtype=np.float64).reshape(3)
        support_node = int(np.argmin(np.linalg.norm(nodes - reference, axis=1)))
        support_nodes = np.array([support_node], dtype=np.int64)
        a0, alpha, nodal_acceleration, mass_moments = inertia_relief(nodes, nodal_load, mass_items, mass, cg, inertia_cg, gravity, links)
        log("  free flight: mass %.6g kg (%.6g of it in %d item(s)), mass centre (%.4g, %.4g, %.4g); the loads give a0 (%.4g, %.4g, %.4g) m/s^2, "
            "alpha (%.4g, %.4g, %.4g) rad/s^2; support node %d at (%.4g, %.4g, %.4g)"
            % (mass, mass_of_items, len(mass_items), cg[0], cg[1], cg[2], a0[0], a0[1], a0[2], alpha[0], alpha[1], alpha[2],
               support_node, nodes[support_node][0], nodes[support_node][1], nodes[support_node][2]))
    else:
        support_node = -1
        a0, alpha = np.zeros(3), np.zeros(3)
        nodal_acceleration = np.tile(gravity, (len(nodes) + len(mass_items) + len(links), 1))
        mass_moments = np.zeros((len(mass_items), 3))
    result.update({"support_nodes": int(len(support_nodes)), "support_corner_nodes": int(len(support_nodes)),
                   "support_node": support_node, "support_at": [float(v) for v in nodes[support_node]] if free_flight else None})

    # The elements: the skin's shells on every wetted face, then the ribs'
    # (unloaded) on the rib faces; a point-load condition on every node
    # that carries a nodal load.
    n_skin_shells = len(faces)
    shells = np.concatenate([faces, rib_faces]) if len(rib_faces) else faces
    shell_region = np.concatenate([region_of_face, rib_region]) if len(rib_faces) else region_of_face
    shell_is_rib = np.concatenate([np.zeros(n_skin_shells, dtype=bool), np.ones(len(rib_faces), dtype=bool)])
    shell_prop = np.concatenate([skin_prop, rib_prop]) if len(rib_faces) else skin_prop
    shell_part = skin_part + rib_part

    # The material axes, one per shell: the application's per face
    # (projected into the face), else the longest extent of the region's
    # nodes; only set on the elements when a laminate needs them.
    element_axes = None
    if needs_laminates:
        def axes_file(key, count):
            if not job.get(key):
                return None
            a = np.fromfile(os.path.join(job_dir, job[key]), dtype="<f8")
            if len(a) != 3 * count:
                raise JobError("%s holds %d values for %d faces." % (job[key], len(a), count))
            return a.reshape(count, 3)
        given = [axes_file("axes", len(faces)), axes_file("rib_axes", len(rib_faces))]
        pieces = []
        for tris, region_array, g in ((faces, region_of_face, given[0]), (rib_faces, rib_region, given[1])):
            out = np.zeros((len(tris), 3))
            for rid in ids:
                sel = np.nonzero(region_array == rid)[0]
                if len(sel) == 0:
                    continue
                fallback = _longest_axis(nodes[np.unique(tris[sel])])
                out[sel] = _project_axes(nodes, tris[sel], None if g is None else g[sel], fallback)
            pieces.append(out)
        element_axes = np.concatenate(pieces) if len(rib_faces) else pieces[0]
    # Six degrees of freedom on every node (the mass items' and the offset
    # links' too). Known before the solve, so "auto" can choose its solver
    # by them.
    dofs = 6 * (len(nodes) + len(mass_items) + len(links))
    linear_solver = resolve_linear_solver(solver_opts["linear_solver"], dofs)
    result["linear_solver"] = linear_solver
    loose = len(nodes) - len(np.unique(shells))
    if loose:
        log("  note: %d node(s) belong to no shell (they stay put)" % loose)
    # A condition needs some Properties that exist: the first material's.
    point_prop = property_ids[0]
    if mass_items:
        material_entries.append(("Masses", MASS_PROPERTIES_ID, "LinearElasticPlaneStress2DLaw", {}))
    for k, bar in enumerate(bars):
        material_entries.append(("Bar_%d" % k, BAR_PROPERTIES_BASE + k, "TrussConstitutiveLaw",
                                 {"DENSITY": bar["density"], "YOUNG_MODULUS": bar["youngs_modulus"], "CROSS_AREA": bar["area"]}))
    t0 = time.time()
    log("  writing the Kratos model: %d nodes, %d shells (%d of them ribs), %d mass item(s), %d tie(s), %d link(s), %d bar(s), %d dofs"
        % (len(nodes), len(shells), len(rib_faces), len(mass_items), len(ties), len(links), len(bars), dofs))
    write_mdpa(mdpa_base + ".mdpa", nodes, shells, shell_prop, shell_part, support_nodes, loaded_nodes, mass_items,
               property_ids, solver_opts["shell_element"], point_prop, links, bars)
    write_materials(materials_path, material_entries)
    seconds_mdpa = time.time() - t0
    parameters = project_parameters(mdpa_base, materials_path, echo_level, linear_solver, bool(links))
    log("  solving %d dofs with %s (%d shells%s)" % (dofs, linear_solver, len(shells), ", %d laminate region(s)" % len(laminates) if laminates else ""))
    model, structure, analysis, seconds_solve = run_kratos(job_dir, parameters, len(nodes), len(shells), nodal_load, nodal_acceleration, mass_items, mass_moments,
                                                           echo_level, element_axes, needs_laminates, ties, links)
    displacement, rotation, reaction, reaction_moment, mass_displacement = read_nodal(structure, len(nodes), len(mass_items), len(links))
    n_elements = len(shells)
    # Each bar's axial force from its ends' displacements: E A (du . e) / L,
    # positive in tension (independent of the element's own sign convention).
    bar_results = []
    for k, bar in enumerate(bars):
        pa, pb = bar_ends[k]
        e = pb - pa
        length = float(np.linalg.norm(e))
        e /= length
        def u_of(i):
            return displacement[i] if i < len(nodes) else mass_displacement[i - len(nodes)]
        du = u_of(bar["b"]) - u_of(bar["a"])
        force = bar["youngs_modulus"] * bar["area"] * float(np.dot(du, e)) / length
        bar_results.append({"name": bar["name"], "force": force, "stress": force / bar["area"], "length": length,
                            "a": [float(v) for v in pa], "b": [float(v) for v in pb], "mass": bar["density"] * bar["area"] * length})
    log("  post-processing: stresses, margins, the force balance")
    von_mises_elem = element_von_mises(structure, n_elements)
    # Tsai-Wu reserve factors of the laminate shells whose laminae have
    # strengths (the smallest over the plies and integration points), -1
    # elsewhere; and the drape per shell for the VTK.
    reserve_elem = np.full(n_elements, -1.0)
    shear_elem = np.zeros(n_elements)
    deviation_elem = np.zeros(n_elements)
    if laminates:
        import KratosMultiphysics.StructuralMechanicsApplication as SMA
        for (rid, kind), stats in laminates.items():
            elem_ids = stats["faces"] + (n_skin_shells if kind == "rib" else 0)
            if stats["draped"]:
                shear_elem[elem_ids] = stats["shear_per_face"]
                deviation_elem[elem_ids] = stats["deviation_per_face"]
            if not stats["has_strengths"]:
                continue
            for e in elem_ids:
                values = _integration_values(structure.GetElement(int(e) + 1), SMA.TSAI_WU_RESERVE_FACTOR, structure.ProcessInfo)
                if values:
                    reserve_elem[e] = min(float(v) for v in values)

    # The peaks per region: the skin's shells and the ribs', each with the
    # margin against its own criterion -- von Mises against the allowable,
    # or Tsai-Wu's reserve factor for a laminate.
    def margin(allowable, peak):
        return (allowable / peak - 1.0) if (allowable > 0.0 and peak > 0.0) else 0.0
    per_material = []
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", "")
        for kind in ("skin", "rib"):
            card = r.get(kind)
            if not card:
                continue
            is_rib = kind == "rib"
            sel = np.nonzero((shell_region == rid) & (shell_is_rib == is_rib))[0]
            peak = float(von_mises_elem[sel].max()) if len(sel) else 0.0
            if is_rib:
                mass_here = float((rib_density_t[rib_region == rid] * rib_area[rib_region == rid]).sum())
            else:
                mass_here = float((skin_density_t[region_of_face == rid] * face_area[region_of_face == rid]).sum())
            entry = {"id": rid, "label": "%s %s" % (label, "ribs" if is_rib else "skin"), "kind": kind, "max_von_mises": peak, "mass": mass_here}
            stats = laminates.get((rid, kind))
            if stats is None:
                a = float(card["material"].get("allowable_stress", 0.0))
                entry.update({"criterion": "von Mises", "allowable": a, "margin": margin(a, peak),
                              "reserve_factor": (a / peak) if (a > 0.0 and peak > 0.0) else 0.0})
            else:
                rf = reserve_elem[sel]
                rf = rf[rf > 0.0]
                rf_min = float(rf.min()) if len(rf) else 0.0
                entry.update({"criterion": "Tsai-Wu" if stats["has_strengths"] else "Tsai-Wu (no strengths: no margin)", "allowable": 0.0,
                              "reserve_factor": rf_min, "margin": (rf_min - 1.0) if rf_min > 0.0 else 0.0,
                              "plies": stats["plies"], "thickness": stats["thickness"], "stacks": stats["bins"],
                              "drape": None if not stats["draped"] else {"max_shear": stats["max_shear"], "max_deviation": stats["max_deviation"],
                                                                          "faces_past_locking": stats["faces_past_locking"], "bin_degrees": stats["bin_degrees"]}})
            per_material.append(entry)
    i_vm = int(np.argmax(von_mises_elem)) if n_elements else 0
    result.update({"elements": int(n_elements + len(mass_items) + len(bars)), "shells": int(len(shells)), "rib_shells": int(len(rib_faces)), "seconds_mdpa": seconds_mdpa,
                   "ties": len(ties), "links": len(links), "bars": bar_results, "mass_bars": mass_of_bars})

    # The force balance. Gravity on the mass (the items' too); the
    # pressure's resultant as -p n A over the outward faces; the point
    # forces as given; in free flight the inertial field's resultant,
    # -m a0, which cancels the rest by construction.
    applied_gravity = mass * gravity
    applied_pressure = -(face_pressure[:, None] * n_area).sum(axis=0) if len(faces) else np.zeros(3)
    applied_inertial = -mass * a0
    reaction_sum = reaction.sum(axis=0)
    reaction_moment_sum = reaction_moment.sum(axis=0)
    # Equilibrium: the reactions and the applied loads sum to zero. The
    # error is measured against the loads' magnitude -- the gross figure,
    # so a pressure whose resultant cancels over a closed surface still
    # sets a scale, and nothing divides by zero.
    applied_total = applied_gravity + applied_pressure + applied_point + applied_inertial
    load_scale = max(float(np.linalg.norm(applied_total)),
                     float(np.linalg.norm(applied_gravity)) + float((np.abs(face_pressure) * face_area).sum()) + sum(float(np.linalg.norm(f)) for _, f in point_loads))
    imbalance = float(np.linalg.norm(reaction_sum + applied_total))
    balance_error = imbalance / load_scale if load_scale > 0.0 else 0.0
    moment_error = 0.0
    if free_flight:
        # The whole load set (the inertial field included) has no resultant
        # and no moment, so the support's moment is the residual outright.
        moment_error = float(np.linalg.norm(reaction_moment_sum)) / (load_scale * diagonal) if load_scale > 0.0 and diagonal > 0.0 else 0.0
    balanced = balance_error <= BALANCE_TOLERANCE and moment_error <= BALANCE_TOLERANCE
    warnings = []
    if not balanced:
        warnings.append("the reactions miss the applied loads by %.1f%% (|R + F| = %.4g N against loads of %.4g N%s): the linear solver (%s) did not converge, so the displacements and stresses are not to be trusted. Use the direct solver (sparse_lu)."
                        % (100.0 * balance_error, imbalance, load_scale, (", moment residual %.1f%%" % (100.0 * moment_error)) if free_flight else "", linear_solver))
        log("  WARNING: " + warnings[-1])
    # What each mass item puts on the structure: its own inertial and
    # gravity load, m times the field at its node, as a check and for the
    # application's picture.
    item_results = []
    for k, it in enumerate(mass_items):
        field = nodal_acceleration[len(nodes) + k]
        item_results.append({"name": it["name"], "mass": it["mass"], "at": [float(v) for v in it["at"]], "node": int(len(nodes) + k),
                             "attach_nodes": int(len(it["attach"])), "attach_centre": [float(v) for v in it["attach_centre"]],
                             "displacement": [float(v) for v in mass_displacement[k]], "force": [float(v) for v in it["mass"] * field]})

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
            frequencies, mode_shapes, eigensolver_used = run_modes(job_dir, mdpa_base, materials_path, len(nodes), len(shells), mass_items, echo_level,
                                                                  n_modes, str(solver_opts.get("eigensolver") or "spectra_sym_g_eigs_shift"), element_axes,
                                                                  support=not free_flight, ties=ties, links=links)
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
        "mass_structure": mass_structure,
        "mass_items": mass_of_items,
        "mass_centre": [float(v) for v in cg],
        "inertia_cg": [[float(v) for v in row] for row in inertia_cg],
        "items": item_results,
        "max_displacement": float(u_mag[i_max]),
        "max_displacement_at": [float(v) for v in nodes[i_max]],
        "max_von_mises": float(von_mises_elem[i_vm]) if n_elements else 0.0,
        "max_von_mises_material": int(shell_region[i_vm]) if n_elements else 0,
        "per_material": per_material,
        "reaction": [float(v) for v in reaction_sum],
        "reaction_moment": [float(v) for v in reaction_moment_sum],
        "applied_gravity": [float(v) for v in applied_gravity],
        "applied_pressure": [float(v) for v in applied_pressure],
        "applied_inertial": [float(v) for v in applied_inertial],
        "rigid_acceleration": [float(v) for v in a0],
        "rigid_angular_acceleration": [float(v) for v in alpha],
        # The vertical inertial load the structure feels at its mass centre,
        # in g: the field there is g_vec - a0, downward positive.
        "load_factor_at_cg": float(-(gravity[2] - a0[2]) / G0),
        "mean_normal_displacement": mean_normal_disp,
        "balance_error": balance_error,
        "moment_error": moment_error,
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
    if laminates:
        cell_data.update({"reserve_factor": reserve_elem, "ply_shear_max": shear_elem, "fibre_deviation_max": deviation_elem})
        result["laminates"] = len(laminates)
    point_data = {"displacement": displacement, "rotation": rotation, "von_mises": nodal_average(len(nodes), [shells], von_mises_elem)}
    if mode_shapes is not None:
        for k in range(len(frequencies)):
            point_data["mode_%d" % (k + 1)] = mode_shapes[k]
    write_vtk(outputs["vtk"], nodes, [shells], [VTK_TRIANGLE], point_data, cell_data)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved %d dofs (%d shells, %d of them ribs, %d mass item(s)) with %s in %.2f s: mass %.6g kg, max displacement %.4g m, max von Mises %.4g Pa (region %d); "
        "reaction (%.4g, %.4g, %.4g) N%s against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g)%s: %s; wrote %s"
        % (dofs, len(shells), len(rib_faces), len(mass_items), linear_solver, result["seconds_solve"], mass, result["max_displacement"], result["max_von_mises"], result["max_von_mises_material"],
           reaction_sum[0], reaction_sum[1], reaction_sum[2],
           (" and (%.4g, %.4g, %.4g) Nm" % tuple(reaction_moment_sum)) if free_flight else "",
           applied_gravity[0], applied_gravity[1], applied_gravity[2],
           applied_pressure[0], applied_pressure[1], applied_pressure[2], applied_point[0], applied_point[1], applied_point[2],
           (" + inertial (%.4g, %.4g, %.4g)" % tuple(applied_inertial)) if free_flight else "",
           ("balanced to %.2e" % max(balance_error, moment_error)) if balanced else ("NOT BALANCED (%.1f%%)" % (100.0 * max(balance_error, moment_error))), outputs["vtk"]))
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
