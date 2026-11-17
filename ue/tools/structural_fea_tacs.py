"""Linear static (and modal) finite-element analysis of scene bodies as
SHELLS on TACS -- the Toolkit for the Analysis of Composite Structures
(smdogroup.github.io/tacs) -- through pyTACS, reading the SAME job
directory as structural_fea.py (the Kratos solver) and writing the same
result.json and VTK. Added 2026-11-17 as the second solver of Analysis >
TACS (FEA); the application picks the script by the solver chosen, the
job is identical, so any case can be run on both and compared.

WHERE IT RUNS. TACS has no Windows build (its conda packages are Linux
and macOS), so the application runs this script inside WSL
(Entity/AerotelicMeshLib.cpp: wsl.exe -d <distro> -e <python> -u
structural_fea_tacs.py /mnt/c/.../job). Paths the job names in Windows
form (the VTK to write) are mapped to /mnt/<drive>/... here (host_path).
Install: inside the WSL distribution,
    micromamba create -n tacs -c conda-forge -c smdogroup python=3.11 tacs
    micromamba run -n tacs pip install gmsh      (only the check script's rib case)
and set AEROTELIC_TACS_PYTHON=wsl:<distro>:/home/<user>/micromamba/envs/tacs/bin/python
if the environment is not at ~/micromamba/envs/tacs. (PyPI's "tacs" is an
unrelated TOML library -- never pip install it.)

THE MODEL, as in structural_fea.py: every skin face a shell of its
region's skin thickness and material, every rib face an unloaded shell of
the rib card; the mass items hung by RBE3, the hinge ties as RBE2 on the
translations, the offset links as rigid RBE2 nodes, the pushrods as
AXIAL SPRINGS (TACS has no truss element: a 6-dof spring with k_axial =
E A / L along the rod and nothing else, its mass rho A L lumped half on
each end -- the same stiffness and resultant as the linear truss). The
shell is TACS's Tri3Shell: a first-order shear deformation (Mindlin)
triangle with MITC tying against shear locking and a drilling penalty,
six degrees of freedom a node -- where Kratos's default is the Kirchhoff
thin triangle; on thin skins the two agree to a percent or two, which the
cross-check (structural_fea_check.py --backend tacs) measures.

COMPONENTS AND DESIGN VARIABLES. pyTACS groups elements by property id,
one TACS "component" each, and a component is where a thickness design
variable lives. One component is made per region's skin and per region's
ribs for an isotropic card (one thickness variable each), and for a
laminate one per distinct (draped stack, reference axis) bin -- the stack
binned exactly as the Kratos script bins it, the reference axis to about
six degrees -- with no design variable yet (TACS's laminate section has
none; its smeared section would). After the solve the ADJOINT is run for
the structural mass and the KS-aggregated failure index of every
component that has an allowable, with respect to those thickness
variables, and the gradients are reported ("sensitivities"): the first
piece of the gradient-based design the solver was brought in for.

LOADS. The pressures as the consistent nodal loads of each face (-p n A,
a third to each corner -- the same loads Kratos gets, so the two solvers
see the same case), the point forces on their nearest nodes, and the
inertial field -- gravity, and in free flight the rigid-body field g - a0
- alpha x (r - cg), with -alpha on the rotations -- applied as M a with
TACS's own consistent mass matrix (assembleMatType(MASS_MATRIX)), so the
load is exactly what the elements' mass would feel. The rigid-body
accelerations are structural_fea.py's (rigid_body_properties,
inertia_relief, imported), and the free-flight support is the same single
node fixed in six degrees of freedom; its reaction is the check.

REACTIONS. TACS applies the Dirichlet conditions by zeroing rows, so the
reactions are recovered as K u - F at the fixed degrees of freedom, K u
assembled without the conditions (assembleRes(applyBCs=False)).

STRESSES AND MARGINS are recovered here from the displacements and
rotations, not read from TACS's f5 (whose shell "stresses" are the
resultants N, M, Q): the membrane strain of each flat triangle (constant),
the curvature from the linearly interpolated rotations (kappa_x =
d theta_y / dx, kappa_y = -d theta_x / dy, kappa_xy = d theta_y / dy -
d theta_x / dx: the Mindlin field the element itself uses, with the
director d = theta x n), and from them the plane stress through the
thickness -- von Mises at the top, middle and bottom of an isotropic shell;
for a laminate the ply stresses in each ply's own axes and the Tsai-Wu
reserve factor solved from a RF^2 + b RF = 1 with F12* = -1/2 (the
application's laminate.hpp convention), the smallest over the plies and
their faces. The stack's outer ply (the job's first) lies on the +normal
side; TACS lists plies bottom-first, so the stack is reversed for it.

RESULT. The same keys as structural_fea.py's result (see its run_job
docstring) plus "backend": "tacs", "tacs_version", "tacs_dofs" (the
multiplier nodes of the rigid elements included) and "sensitivities":
{"design_variables": [{"component", "region", "kind", "thickness"}],
"functions": [{"name", "component", "value", "gradient": [d/d t_i]}]}.
"linear_solver" is "gmres" (TACS's one Krylov method, with its
near-complete ILU: the default fill makes it a direct solve in all but
name); "eigensolver" "tacs_lanczos".
"""

import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gmsh_common as tg  # noqa: E402  (the readers and the failure convention; gmsh itself optional here)
import structural_fea as sf  # noqa: E402  (the job contract, the rigid-body bookkeeping, the VTK writer)

np = tg.np
log = tg.log
JobError = tg.JobError
G0 = sf.G0

LINEAR_SOLVER = "gmres"
EIGENSOLVER = "tacs_lanczos"
KS_WEIGHT = 100.0                 # the KS aggregate of the failure index per component
AXIS_BIN = 0.1                    # a laminate's reference axis is binned to this (unit-vector components; ~6 degrees)
MAX_COMPONENTS = 400              # distinct laminate components per region before the axis bins are coarsened
RIGID_MODE_FRACTION = 1e-6        # an eigenvalue below this fraction of the largest asked for is a rigid mode (free flight)
PROP_SHELL_BASE = 1000            # property ids: shells from here, bars from BAR_PROP_BASE
BAR_PROP_BASE = 900000
DUMMY_MAT = 1

_DRIVE = re.compile(r"^([A-Za-z]):[\\/](.*)$")


def host_path(path):
    """A path the job names, as this interpreter sees it: under WSL a
    Windows path C:\\x\\y is /mnt/c/x/y; on Windows itself, unchanged."""
    if os.name == "nt":
        return path
    m = _DRIVE.match(str(path))
    if m:
        return "/mnt/%s/%s" % (m.group(1).lower(), m.group(2).replace("\\", "/"))
    return path


def _import_tacs():
    import importlib
    try:
        tacs = importlib.import_module("tacs")
        for sub in ("pyTACS", "constitutive", "elements", "functions", "TACS"):
            if not hasattr(tacs, sub):
                importlib.import_module("tacs." + sub)
    except ImportError as exc:
        raise JobError("TACS is not installed in this Python environment (%s): inside WSL, "
                       "micromamba create -n tacs -c conda-forge -c smdogroup python=3.11 tacs; "
                       "PyPI's 'tacs' is an unrelated package. (%s)" % (sys.executable, exc))
    try:
        importlib.import_module("pyNastran.bdf.bdf")
    except ImportError as exc:
        raise JobError("pyNastran is not installed beside TACS (pip install pyNastran): %s" % exc)
    return tacs


# ---------------------------------------------------------------------------
# Plane-stress laws and the laminate recovery
# ---------------------------------------------------------------------------

def _q_iso(E, nu):
    f = E / (1.0 - nu * nu)
    return np.array([[f, nu * f, 0.0], [nu * f, f, 0.0], [0.0, 0.0, 0.5 * (1.0 - nu) * f]])


def _q_ply(p):
    """The reduced stiffness of a lamina in its own axes (engineering shear)."""
    nu21 = p["nu12"] * p["e2"] / p["e1"]
    d = 1.0 - p["nu12"] * nu21
    return np.array([[p["e1"] / d, p["nu12"] * p["e2"] / d, 0.0], [p["nu12"] * p["e2"] / d, p["e2"] / d, 0.0], [0.0, 0.0, p["g12"]]])


def _t_stress(theta):
    """Stress transformation from the laminate axes to a ply turned by
    theta (radians, counter-clockwise about the normal)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c * c, s * s, 2 * c * s], [s * s, c * c, -2 * c * s], [-c * s, c * s, c * c - s * s]])


_R = np.diag([1.0, 1.0, 2.0])
_RINV = np.diag([1.0, 1.0, 0.5])


def _strain_to_ply(theta):
    """Engineering strain in the laminate axes -> engineering strain in the ply's."""
    return _R @ _t_stress(theta) @ _RINV


def _von_mises(s):
    """Plane-stress von Mises of rows [sx, sy, txy]."""
    s = np.atleast_2d(s)
    return np.sqrt(np.maximum(s[:, 0] ** 2 - s[:, 0] * s[:, 1] + s[:, 1] ** 2 + 3.0 * s[:, 2] ** 2, 0.0))


def _tsai_wu_reserve(s1, s2, t12, p):
    """The Tsai-Wu reserve factor of a ply stress state: the factor on the
    stresses that makes F1 s1 + F2 s2 + F11 s1^2 + F22 s2^2 + F66 t^2 +
    2 F12 s1 s2 = 1, with F12 = -1/2 sqrt(F11 F22) (laminate.hpp). +inf
    when unstressed."""
    t1, c1, t2, c2, s12 = p["t1"], p["c1"], p["t2"], p["c2"], p["s12"]
    F1, F2 = 1.0 / t1 - 1.0 / c1, 1.0 / t2 - 1.0 / c2
    F11, F22, F66 = 1.0 / (t1 * c1), 1.0 / (t2 * c2), 1.0 / (s12 * s12)
    F12 = -0.5 * np.sqrt(F11 * F22)
    a = F11 * s1 * s1 + F22 * s2 * s2 + F66 * t12 * t12 + 2.0 * F12 * s1 * s2
    b = F1 * s1 + F2 * s2
    with np.errstate(divide="ignore", invalid="ignore"):
        rf = np.where(a > 0.0, (-b + np.sqrt(b * b + 4.0 * a)) / (2.0 * a), np.where(b > 0.0, 1.0 / b, np.inf))
    return rf


def _element_frames(nodes, tris, axes=None):
    """Per triangle: the unit normal, the in-plane axes e1 (the given axis
    projected into the face, else the first edge) and e2 = n x e1, and the
    corners' in-plane coordinates (3, 2)."""
    A, B, C = nodes[tris[:, 0]], nodes[tris[:, 1]], nodes[tris[:, 2]]
    n = np.cross(B - A, C - A)
    area2 = np.linalg.norm(n, axis=1)
    n = n / np.maximum(area2, 1e-300)[:, None]
    e1 = B - A
    if axes is not None:
        proj = axes - n * np.einsum("ij,ij->i", axes, n)[:, None]
        ok = np.linalg.norm(proj, axis=1) > 1e-9
        e1 = np.where(ok[:, None], proj, e1)
    e1 = e1 / np.maximum(np.linalg.norm(e1, axis=1), 1e-300)[:, None]
    e2 = np.cross(n, e1)
    xy = np.zeros((len(tris), 3, 2))
    for k, P in enumerate((A, B, C)):
        d = P - A
        xy[:, k, 0] = np.einsum("ij,ij->i", d, e1)
        xy[:, k, 1] = np.einsum("ij,ij->i", d, e2)
    return n, e1, e2, xy, 0.5 * area2


def _shape_gradients(xy):
    """d N_i / dx, d N_i / dy of the linear triangle: (n, 3, 2)."""
    x, y = xy[:, :, 0], xy[:, :, 1]
    det = (x[:, 1] - x[:, 0]) * (y[:, 2] - y[:, 0]) - (x[:, 2] - x[:, 0]) * (y[:, 1] - y[:, 0])
    det = np.where(np.abs(det) > 1e-300, det, 1e-300)
    g = np.zeros_like(xy)
    g[:, 0, 0], g[:, 1, 0], g[:, 2, 0] = y[:, 1] - y[:, 2], y[:, 2] - y[:, 0], y[:, 0] - y[:, 1]
    g[:, 0, 1], g[:, 1, 1], g[:, 2, 1] = x[:, 2] - x[:, 1], x[:, 0] - x[:, 2], x[:, 1] - x[:, 0]
    return g / det[:, None, None]


def shell_strains(nodes, tris, displacement, rotation, axes=None):
    """The membrane strain (ex, ey, gxy) and the curvature (kx, ky, kxy) of
    every flat triangle in its own frame, from the nodal displacements and
    rotation vectors (global): the constant-strain triangle for the
    membrane, the Mindlin curvatures of the linearly interpolated
    rotations. Returns (eps0 (n, 3), kappa (n, 3), frames)."""
    n, e1, e2, xy, area = _element_frames(nodes, tris, axes)
    g = _shape_gradients(xy)
    eps0 = np.zeros((len(tris), 3))
    kappa = np.zeros((len(tris), 3))
    for k in range(3):
        u = displacement[tris[:, k]]
        th = rotation[tris[:, k]]
        ul = np.stack([np.einsum("ij,ij->i", u, e1), np.einsum("ij,ij->i", u, e2)], axis=1)
        tl = np.stack([np.einsum("ij,ij->i", th, e1), np.einsum("ij,ij->i", th, e2)], axis=1)
        gx, gy = g[:, k, 0], g[:, k, 1]
        eps0[:, 0] += gx * ul[:, 0]
        eps0[:, 1] += gy * ul[:, 1]
        eps0[:, 2] += gy * ul[:, 0] + gx * ul[:, 1]
        # d = theta x n = (theta_y, -theta_x) in the face: u(z) = u0 + z d.
        kappa[:, 0] += gx * tl[:, 1]
        kappa[:, 1] += -gy * tl[:, 0]
        kappa[:, 2] += gy * tl[:, 1] - gx * tl[:, 0]
    return eps0, kappa, (n, e1, e2, xy, area)


def recover_isotropic(eps0, kappa, E, nu, t):
    """Peak von Mises over the top, middle and bottom of isotropic shells."""
    Q = _q_iso(E, nu)
    best = np.zeros(len(eps0))
    for z in (-0.5 * t, 0.0, 0.5 * t):
        best = np.maximum(best, _von_mises((eps0 + z * kappa) @ Q.T))
    return best


def recover_laminate(eps0, kappa, rows, has_strengths):
    """Per element of one draped stack (rows outer-first: [t, angle_deg,
    rho, e1, e2, nu12, g12, g13, g23 (, t1, c1, t2, c2, s12, s13, s23)]):
    the largest ply von Mises (a magnitude, not a criterion) and the
    smallest Tsai-Wu reserve factor over the plies and their faces (-1
    without strengths)."""
    plies = []
    for r in rows:
        p = {"t": r[0], "angle": r[1], "e1": r[3], "e2": r[4], "nu12": r[5], "g12": r[6]}
        if has_strengths and len(r) >= 14:
            p.update({"t1": r[9], "c1": r[10], "t2": r[11], "c2": r[12], "s12": r[13]})
        plies.append(p)
    total = sum(p["t"] for p in plies)
    z_top = 0.5 * total
    vm = np.zeros(len(eps0))
    rf = np.full(len(eps0), np.inf)
    for p in plies:   # outer (the +normal side) first
        z_bot = z_top - p["t"]
        th = np.radians(p["angle"])
        Q = _q_ply(p)
        Te = _strain_to_ply(th)
        for z in (z_top, z_bot):
            e_ply = (eps0 + z * kappa) @ Te.T
            s_ply = e_ply @ Q.T
            vm = np.maximum(vm, _von_mises(s_ply))
            if has_strengths:
                rf = np.minimum(rf, _tsai_wu_reserve(s_ply[:, 0], s_ply[:, 1], s_ply[:, 2], p))
        z_top = z_bot
    if not has_strengths:
        rf = np.full(len(eps0), -1.0)
    return vm, np.where(np.isfinite(rf), rf, -1.0)


# ---------------------------------------------------------------------------
# The model: the job's shells into components, the BDF, the loads
# ---------------------------------------------------------------------------

def _bin_axes(axes, max_bins):
    """Group faces by their (projected, unit) reference axis to AXIS_BIN,
    coarsened until at most max_bins groups: (group per face, axis per group)."""
    step = AXIS_BIN
    while True:
        keys = np.round(axes / step).astype(np.int64)
        uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
        if len(uniq) <= max_bins or step >= 2.0:
            break
        step *= 2.5
    inverse = np.asarray(inverse).reshape(-1)
    centres = np.zeros((len(uniq), 3))
    for k in range(len(uniq)):
        a = axes[inverse == k].mean(axis=0)
        centres[k] = a / max(np.linalg.norm(a), 1e-300)
    return inverse, centres


def build_model(job_dir, job):
    """Everything the solve needs, from the job (validated the way the
    Kratos script validates it, with its helpers)."""
    outputs = job["outputs"]
    solver_opts = {"echo_level": 0, "modes": 0, "ks_weight": KS_WEIGHT, "pc_fill_level": None, "subspace_size": None, "f5": False}
    solver_opts.update(job.get("solver", {}))
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
    regions = sf._regions_from_job(job)

    nodes = sf.read_nodes(os.path.join(job_dir, job["nodes"]))
    if len(nodes) == 0:
        raise JobError("the mesh has no nodes.")

    def faces_of(key, what):
        if not job.get(key):
            return np.zeros((0, 3), dtype=np.int64), np.zeros(0, dtype=np.int64)
        f, r = sf.read_faces(os.path.join(job_dir, job[key]))
        if len(f) and (f.min() < 0 or f.max() >= len(nodes)):
            raise JobError("the %s refer to nodes the mesh does not have." % what)
        return f, r

    faces, region_of_face = faces_of("faces", "skin faces")
    rib_faces, rib_region = faces_of("rib_faces", "rib faces")
    if len(faces) == 0 and len(rib_faces) == 0:
        raise JobError("nothing to analyse: no skin faces and no rib faces.")
    n_area = sf.face_normals_area(nodes, faces) if len(faces) else np.zeros((0, 3))
    face_area = np.linalg.norm(n_area, axis=1)
    rib_area = np.linalg.norm(sf.face_normals_area(nodes, rib_faces), axis=1) if len(rib_faces) else np.zeros(0)

    def _degenerate(f, area):
        if not len(f):
            return 0
        same = (f[:, 0] == f[:, 1]) | (f[:, 1] == f[:, 2]) | (f[:, 0] == f[:, 2])
        return int((same | (area <= 0.0)).sum())
    bad_skin, bad_rib = _degenerate(faces, face_area), _degenerate(rib_faces, rib_area)
    if bad_skin or bad_rib:
        raise JobError("%d skin face(s) and %d rib face(s) have no area (two corners on one node, or three in a line): the solver refuses them." % (bad_skin, bad_rib))
    present_faces = sorted(set(int(v) for v in np.unique(region_of_face))) if len(region_of_face) else []
    present_ribs = sorted(set(int(v) for v in np.unique(rib_region))) if len(rib_region) else []
    log("  mesh read: %d nodes, %d skin faces, %d rib faces; regions %s" % (len(nodes), len(faces), len(rib_faces), sorted(set(present_faces) | set(present_ribs))))

    lo, hi = nodes.min(axis=0), nodes.max(axis=0)
    diagonal = float(np.linalg.norm(hi - lo))
    if free_flight:
        support_nodes, support_planes = np.zeros(0, dtype=np.int64), []
    else:
        support_nodes, support_planes = sf._support_nodes(nodes, supports)
    mass_items = sf._mass_items_from_job(job, nodes)
    ties, links, bars, bar_ends = sf._ties_links_bars_from_job(job, nodes, len(mass_items))

    if loads.get("pressure_file"):
        face_pressure = np.fromfile(os.path.join(job_dir, loads["pressure_file"]), dtype="<f8")
        if len(face_pressure) != len(faces):
            raise JobError("the pressure file has %d values for %d wetted faces." % (len(face_pressure), len(faces)))
    else:
        face_pressure = np.full(len(faces), float(loads["pressure"]))

    point_loads = []
    applied_point = np.zeros(3)
    for pf in point_forces:
        at = np.array(pf["at"], dtype=np.float64)
        node = int(np.argmin(np.linalg.norm(nodes - at, axis=1)))
        force = np.array(pf["force"], dtype=np.float64)
        point_loads.append((node, force))
        applied_point += force
    nodal_load = np.zeros((len(nodes), 3))
    if len(faces):
        face_force = -(face_pressure[:, None] * n_area)
        for k in range(3):
            np.add.at(nodal_load, faces[:, k], face_force / 3.0)
    for node, force in point_loads:
        nodal_load[node] += force

    by_id = {int(r["id"]): r for r in regions}
    missing = [m for m in sorted(set(present_faces) | set(present_ribs)) if m not in by_id]
    if missing:
        raise JobError("the mesh has faces of region %s, which the job gives nothing for." % missing)
    ids = sorted(by_id)
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", rid)
        has_faces, has_ribs = rid in present_faces, rid in present_ribs
        if r.get("skin"):
            sf._check_material(r["skin"]["material"], "%s skin" % label)
            if not float(r["skin"]["thickness"]) > 0.0:
                raise JobError("'%s' has no skin thickness." % label)
            if not has_faces:
                raise JobError("'%s' has a skin but no face to put its shells on." % label)
        elif has_faces:
            raise JobError("'%s' has skin faces but no skin (thickness and material) to put on them." % label)
        if r.get("rib"):
            sf._check_material(r["rib"]["material"], "%s ribs" % label)
            if not float(r["rib"]["thickness"]) > 0.0:
                raise JobError("'%s' has no rib thickness." % label)
            if not has_ribs:
                raise JobError("'%s' has ribs but no rib face to put their shells on." % label)
        elif has_ribs:
            raise JobError("'%s' has rib faces but says nothing of what the ribs are." % label)
        if not (r.get("skin") or r.get("rib")):
            raise JobError("'%s' is nothing: no skin, no ribs." % label)

    # The shells: skin faces then rib faces, in the job's order.
    n_skin = len(faces)
    shells = np.concatenate([faces, rib_faces]) if len(rib_faces) else faces
    shell_region = np.concatenate([region_of_face, rib_region]) if len(rib_faces) else region_of_face
    shell_is_rib = np.concatenate([np.zeros(n_skin, dtype=bool), np.ones(len(rib_faces), dtype=bool)])
    shell_area = np.concatenate([face_area, rib_area]) if len(rib_faces) else face_area

    # The material axes per shell (projected into the face), for laminates.
    def axes_file(key, count):
        if not job.get(key):
            return None
        a = np.fromfile(os.path.join(job_dir, job[key]), dtype="<f8")
        if len(a) != 3 * count:
            raise JobError("%s holds %d values for %d faces." % (job[key], len(a), count))
        return a.reshape(count, 3)
    given = [axes_file("axes", len(faces)), axes_file("rib_axes", len(rib_faces))]

    # The components: one per isotropic card, one per (stack bin, axis
    # bin) of a laminate card. Each is a dict the element callback reads by
    # its property id.
    components = []          # dicts: prop, label, region, kind, iso | rows, thickness, axis, shells (indices into shells), rho_t
    shell_prop = np.zeros(len(shells), dtype=np.int64)
    shell_rho_t = np.zeros(len(shells))
    shell_axis = np.zeros((len(shells), 3))
    laminates = {}
    next_prop = PROP_SHELL_BASE
    for rid in ids:
        r = by_id[rid]
        label = r.get("label", str(rid))
        for kind in ("skin", "rib"):
            card = r.get(kind)
            if not card:
                continue
            is_rib = kind == "rib"
            sel = np.nonzero((shell_region == rid) & (shell_is_rib == is_rib))[0]
            if len(sel) == 0:
                continue
            m = card["material"]
            t = float(card["thickness"])
            if m.get("plies"):
                stack = sf._stack_from_card(m)
                drape = sf._read_drape(job_dir, card.get("drape_file"), len(sel), len(stack))
                bins, rows_per_bin, rho_t, stats = sf._bin_stacks(stack, drape, len(sel))
                tris_sel = shells[sel]
                g = given[1] if is_rib else given[0]
                g_sel = None if g is None else g[sel - (n_skin if is_rib else 0)]
                fallback = sf._longest_axis(nodes[np.unique(tris_sel)])
                axes_sel = sf._project_axes(nodes, tris_sel, g_sel, fallback)
                shell_axis[sel] = axes_sel
                shell_rho_t[sel] = rho_t
                axis_bin, axis_centres = _bin_axes(axes_sel, max(1, MAX_COMPONENTS // max(1, len(rows_per_bin))))
                keys = {}
                for i, (sb, ab) in enumerate(zip(bins, axis_bin)):
                    keys.setdefault((int(sb), int(ab)), []).append(sel[i])
                for (sb, ab), members in sorted(keys.items()):
                    comp = {"prop": next_prop, "label": "%s %s" % (label, "ribs" if is_rib else "skin"), "region": rid, "kind": kind,
                            "iso": None, "rows": rows_per_bin[sb], "has_strengths": stats["has_strengths"], "thickness": float(sum(row[0] for row in rows_per_bin[sb])),
                            "axis": axis_centres[ab], "shells": np.array(members, dtype=np.int64), "stack_bin": sb, "axis_bin": ab}
                    components.append(comp)
                    shell_prop[members] = next_prop
                    next_prop += 1
                stats["shells"] = sel
                laminates[(rid, kind)] = stats
                log("  %s %s: a laminate of %d plies (%.4g m) in %d draped stack(s) x %d axis bin(s) = %d component(s)%s%s" % (
                    label, kind, stats["plies"], stats["thickness"], stats["bins"], len(axis_centres), len(keys),
                    "" if not stats["draped"] else ", shear up to %.1f deg, fibres turned up to %.1f deg" % (stats["max_shear"], stats["max_deviation"]),
                    "" if stats["has_strengths"] else " (no strengths: no margin)"))
            else:
                comp = {"prop": next_prop, "label": "%s %s" % (label, "ribs" if is_rib else "skin"), "region": rid, "kind": kind,
                        "iso": {"E": float(m["youngs_modulus"]), "nu": float(m["poisson_ratio"]), "rho": float(m["density"]), "allowable": float(m.get("allowable_stress", 0.0) or 0.0)},
                        "rows": None, "has_strengths": float(m.get("allowable_stress", 0.0) or 0.0) > 0.0, "thickness": t, "axis": None, "shells": sel}
                components.append(comp)
                shell_prop[sel] = next_prop
                shell_rho_t[sel] = float(m["density"]) * t
                next_prop += 1

    skin_density_t = shell_rho_t[:n_skin]
    rib_density_t = shell_rho_t[n_skin:]
    mass_structure = float((shell_rho_t * shell_area).sum())
    mass_of_items = float(sum(it["mass"] for it in mass_items))
    mass_of_bars = float(sum(b["density"] * b["area"] * float(np.linalg.norm(bar_ends[k][1] - bar_ends[k][0])) for k, b in enumerate(bars)))
    mass, cg, inertia_cg = sf.rigid_body_properties(nodes, faces, skin_density_t, rib_faces, rib_density_t, mass_items, bars, bar_ends)
    if abs(mass - mass_structure - mass_of_items - mass_of_bars) > 1e-9 * mass:
        raise JobError("the mass did not add up (%.9g against %.9g + %.9g + %.9g)." % (mass, mass_structure, mass_of_items, mass_of_bars))

    g = float(loads["load_factor"]) * G0
    gravity = np.array([0.0, 0.0, -g])
    n_extra = len(mass_items) + len(links)
    if free_flight:
        reference = cg if (relief.get("reference") is None) else np.array(relief["reference"], dtype=np.float64).reshape(3)
        support_node = int(np.argmin(np.linalg.norm(nodes - reference, axis=1)))
        support_nodes = np.array([support_node], dtype=np.int64)
        a0, alpha, nodal_acceleration, mass_moments = sf.inertia_relief(nodes, nodal_load, mass_items, mass, cg, inertia_cg, gravity, links)
        log("  free flight: mass %.6g kg (%.6g of it in %d item(s)), mass centre (%.4g, %.4g, %.4g); the loads give a0 (%.4g, %.4g, %.4g) m/s^2, "
            "alpha (%.4g, %.4g, %.4g) rad/s^2; support node %d at (%.4g, %.4g, %.4g)"
            % (mass, mass_of_items, len(mass_items), cg[0], cg[1], cg[2], a0[0], a0[1], a0[2], alpha[0], alpha[1], alpha[2],
               support_node, nodes[support_node][0], nodes[support_node][1], nodes[support_node][2]))
    else:
        support_node = -1
        a0, alpha = np.zeros(3), np.zeros(3)
        nodal_acceleration = np.tile(gravity, (len(nodes) + n_extra, 1))

    # Nodes no shell, tie, link, bar or mass touches would be singular rows
    # in TACS (every GRID carries six degrees of freedom): fixed, and said.
    used = np.zeros(len(nodes), dtype=bool)
    used[np.unique(shells)] = True
    for t_ in ties:
        used[t_["master"]] = used[t_["slave"]] = True
    for l_ in links:
        used[l_["on"]] = True
    for it in mass_items:
        used[it["attach"]] = True
    for b_ in bars:
        for end in (b_["a"], b_["b"]):
            if end < len(nodes):
                used[end] = True
    loose = np.nonzero(~used)[0]
    if len(loose):
        log("  note: %d node(s) belong to no shell (they are held still)" % len(loose))

    return {
        "outputs": outputs, "solver": solver_opts, "nodes": nodes, "faces": faces, "rib_faces": rib_faces, "shells": shells,
        "shell_region": shell_region, "shell_is_rib": shell_is_rib, "shell_area": shell_area, "shell_prop": shell_prop, "shell_rho_t": shell_rho_t,
        "shell_axis": shell_axis, "n_skin": n_skin, "n_area": n_area, "face_area": face_area, "face_pressure": face_pressure,
        "region_of_face": region_of_face, "rib_region": rib_region, "rib_area": rib_area, "by_id": by_id, "ids": ids,
        "components": components, "laminates": laminates,
        "supports": supports, "support_planes": support_planes, "support_nodes": support_nodes, "free_flight": free_flight, "support_node": support_node,
        "loose": loose, "mass_items": mass_items, "ties": ties, "links": links, "bars": bars, "bar_ends": bar_ends,
        "nodal_load": nodal_load, "point_loads": point_loads, "applied_point": applied_point,
        "mass": mass, "cg": cg, "inertia_cg": inertia_cg, "mass_structure": mass_structure, "mass_of_items": mass_of_items, "mass_of_bars": mass_of_bars,
        "gravity": gravity, "a0": a0, "alpha": alpha, "nodal_acceleration": nodal_acceleration, "diagonal": diagonal,
    }


def write_bdf(model, with_spc=True):
    """The pyNastran BDF object of the model (and its text, for the kept
    job directory): GRIDs (the structural nodes, then a node per mass item,
    then a node per offset link), CTRIA3 per shell on a dummy PSHELL per
    component (the real section is built in the element callback), CONM2 +
    RBE3 per mass item, RBE2 (123) per hinge tie, RBE2 (123456) per offset
    link, CBUSH + PBUSH per bar with half its mass as CONM2 on each end,
    SPC1 123456 on the fixed nodes (and on the loose ones)."""
    from pyNastran.bdf.bdf import BDF
    nodes, shells = model["nodes"], model["shells"]
    n_nodes = len(nodes)
    mass_items, links, bars, ties = model["mass_items"], model["links"], model["bars"], model["ties"]
    bdf = BDF(debug=False)
    for i, p in enumerate(nodes):
        bdf.add_grid(i + 1, [float(p[0]), float(p[1]), float(p[2])])
    for k, it in enumerate(mass_items):
        bdf.add_grid(n_nodes + k + 1, [float(v) for v in it["at"]])
    for k, l_ in enumerate(links):
        bdf.add_grid(n_nodes + len(mass_items) + k + 1, [float(v) for v in l_["at"]])
    bdf.add_mat1(DUMMY_MAT, 70.0e9, None, 0.3, rho=2700.0)
    for comp in model["components"]:
        bdf.add_pshell(int(comp["prop"]), mid1=DUMMY_MAT, t=float(comp["thickness"]))
    eid = 0
    for i in range(len(shells)):
        eid += 1
        bdf.add_ctria3(eid, int(model["shell_prop"][i]), [int(shells[i, 0]) + 1, int(shells[i, 1]) + 1, int(shells[i, 2]) + 1])
    n_shell_eids = eid
    for k, it in enumerate(mass_items):
        eid += 1
        I = it["inertia"]
        # CONM2 I21, I31, I32 carry integral x y dm etc. with Nastran's sign,
        # which is the job's convention (TACS negates them into its tensor).
        bdf.add_conm2(eid, n_nodes + k + 1, float(it["mass"]), cid=0,
                      I=[float(I[0, 0]), float(I[0, 1]), float(I[1, 1]), float(I[0, 2]), float(I[1, 2]), float(I[2, 2])])
    for k, it in enumerate(mass_items):
        eid += 1
        bdf.add_rbe3(eid, n_nodes + k + 1, "123456", [1.0], ["123"], [[int(v) + 1 for v in it["attach"]]])
    for t_ in ties:
        eid += 1
        bdf.add_rbe2(eid, int(t_["master"]) + 1, "123", [int(t_["slave"]) + 1])
    for k, l_ in enumerate(links):
        eid += 1
        bdf.add_rbe2(eid, int(l_["on"]) + 1, "123456", [n_nodes + len(mass_items) + k + 1])
    bar_props = []
    for k, b_ in enumerate(bars):
        pa, pb = model["bar_ends"][k]
        length = float(np.linalg.norm(pb - pa))
        k_axial = b_["youngs_modulus"] * b_["area"] / length
        pid = BAR_PROP_BASE + k
        bdf.add_pbush(pid, [k_axial, 0.0, 0.0, 0.0, 0.0, 0.0], None, None)
        eid += 1
        bdf.add_cbush(eid, pid, [int(b_["a"]) + 1, int(b_["b"]) + 1], [float(v) for v in (pb - pa) / length], None)
        bar_props.append({"prop": pid, "axis": (pb - pa) / length, "k": k_axial, "name": b_["name"], "index": k})
        half = 0.5 * b_["density"] * b_["area"] * length
        for end in (b_["a"], b_["b"]):
            eid += 1
            bdf.add_conm2(eid, int(end) + 1, half)
    fixed = sorted(set(int(v) + 1 for v in model["support_nodes"]) | set(int(v) + 1 for v in model["loose"])) if with_spc else sorted(int(v) + 1 for v in model["loose"])
    if fixed:
        bdf.add_spc1(1, "123456", fixed)
    return bdf, bar_props, n_shell_eids


def _element_callback(model, bar_props, dv_table):
    """pyTACS's element callback: the real section of each component by
    its property id -- an isotropic shell with a thickness design
    variable, a laminate (TACS wants the plies bottom-first; the job lists
    them outer-first, the outer face on the +normal side), or a bar's axial
    spring along its own axis."""
    from tacs import constitutive, elements
    by_prop = {int(c["prop"]): c for c in model["components"]}
    bars_by_prop = {int(b["prop"]): b for b in bar_props}

    def material_props(**kw):
        try:
            return constitutive.MaterialProperties(**kw)
        except TypeError:
            # Older spellings of the strengths.
            alt = dict(kw)
            for a, b in (("T1", "Xt"), ("C1", "Xc"), ("T2", "Yt"), ("C2", "Yc")):
                if a in alt:
                    alt[b] = alt.pop(a)
            return constitutive.MaterialProperties(**alt)

    def callback(dvNum, compID, compDescript, elemDescripts, globalDVs, **kwargs):
        prop = int(kwargs.get("propID", -1))
        out = []
        if prop in bars_by_prop:
            b = bars_by_prop[prop]
            con = constitutive.DOFSpringConstitutive(k=np.array([b["k"], 0.0, 0.0, 0.0, 0.0, 0.0]))
            transform = elements.SpringRefAxisTransform(np.asarray(b["axis"], dtype=np.float64))
            for d in elemDescripts:
                out.append(elements.SpringElement(transform, con))
            return out
        comp = by_prop.get(prop)
        if comp is None:
            raise JobError("pyTACS asked for property %d, which the model has no component for." % prop)
        comp["comp_id"] = int(compID)
        if comp["iso"] is not None:
            iso = comp["iso"]
            ys = iso["allowable"] if iso["allowable"] > 0.0 else 1.0e30
            props = material_props(rho=iso["rho"], E=iso["E"], nu=iso["nu"], ys=ys)
            t = comp["thickness"]
            con = constitutive.IsoShellConstitutive(props, t=t, tNum=dvNum, tlb=0.01 * t, tub=100.0 * t)
            dv_table.append({"dv": int(dvNum), "component": comp["label"], "region": int(comp["region"]), "kind": comp["kind"], "thickness": t, "prop": prop})
            comp["dv"] = int(dvNum)
            transform = None
        else:
            plies, thicknesses, angles = [], [], []
            for row in reversed(comp["rows"]):   # bottom-first for TACS
                kw = {"rho": row[2], "E1": row[3], "E2": row[4], "nu12": row[5], "G12": row[6], "G13": row[7], "G23": row[8]}
                if comp["has_strengths"] and len(row) >= 14:
                    kw.update({"T1": row[9], "C1": row[10], "T2": row[11], "C2": row[12], "S12": row[13]})
                else:
                    kw.update({"T1": 1e30, "C1": 1e30, "T2": 1e30, "C2": 1e30, "S12": 1e30})
                props = material_props(**kw)
                try:
                    ply = constitutive.OrthotropicPly(row[0], props, failureCriterion=constitutive.CompositeFailureCriterion.TSAI_WU)
                except (AttributeError, TypeError):
                    ply = constitutive.OrthotropicPly(row[0], props)
                plies.append(ply)
                thicknesses.append(row[0])
                angles.append(np.radians(row[1]))
            con = constitutive.CompositeShellConstitutive(plies, np.array(thicknesses), np.array(angles))
            transform = elements.ShellRefAxisTransform(np.asarray(comp["axis"], dtype=np.float64))
        for d in elemDescripts:
            if d in ("CTRIA3", "CTRIAR"):
                out.append(elements.Tri3Shell(transform, con))
            elif d in ("CQUAD4", "CQUADR"):
                out.append(elements.Quad4Shell(transform, con))
            else:
                raise JobError("component %d has elements of type %s, which this script does not build." % (prop, d))
        return out
    return callback


def _local_index(fea, bdf_ids):
    """The TACS local node index of each Nastran grid id (this is one
    process: every node is local)."""
    local = np.asarray(fea.getLocalNodeIDsFromGlobal([int(v) for v in bdf_ids], nastranOrdering=True), dtype=np.int64)
    if (local < 0).any():
        raise JobError("TACS does not own %d of the model's nodes." % int((local < 0).sum()))
    return local


def _assemble_mass_load(fea, problem, model, local_of):
    """The inertial load M a as a TACS vector: a the acceleration field at
    every node (structural, mass item, link), -alpha on the rotations, M
    TACS's consistent mass matrix assembled without the conditions."""
    import tacs
    assembler = problem.assembler
    acc = assembler.createVec()
    arr = acc.getArray()
    vpn = assembler.getVarsPerNode()
    field = model["nodal_acceleration"]
    alpha = model["alpha"]
    n_all = len(model["nodes"]) + len(model["mass_items"]) + len(model["links"])
    for i in range(n_all):
        l_ = local_of[i]
        arr[vpn * l_:vpn * l_ + 3] = field[i]
        arr[vpn * l_ + 3:vpn * l_ + 6] = -alpha
    M = assembler.createSchurMat()
    try:
        assembler.assembleMatType(tacs.TACS.MASS_MATRIX, M, applyBCs=False)
    except TypeError:
        assembler.assembleMatType(tacs.TACS.MASS_MATRIX, M)
    load = assembler.createVec()
    M.mult(acc, load)
    return load


def run_job(job_dir):
    """The job of structural_fea.run_job, on TACS. See the module docstring."""
    t_start = time.time()
    with open(os.path.join(job_dir, "job.json"), "r", encoding="utf-8-sig") as f:
        job = json.load(f)
    tacs = _import_tacs()
    from tacs import pyTACS, functions
    from mpi4py import MPI

    model = build_model(job_dir, job)
    outputs = model["outputs"]
    solver_opts = model["solver"]
    nodes, shells = model["nodes"], model["shells"]
    n_nodes = len(nodes)
    mass_items, links, bars, ties = model["mass_items"], model["links"], model["bars"], model["ties"]
    n_extra = len(mass_items) + len(links)
    free_flight = model["free_flight"]
    result = {
        "ok": True, "backend": "tacs", "tacs_version": str(getattr(tacs, "__version__", "?")),
        "wetted_faces": int(len(model["faces"])), "support_planes": model["support_planes"],
        "point_nodes": [int(n) for n, _ in model["point_loads"]], "applied_point": [float(v) for v in model["applied_point"]],
        "support_mode": "free flight" if free_flight else "clamped",
        "support_nodes": int(len(model["support_nodes"])), "support_corner_nodes": int(len(model["support_nodes"])),
        "support_node": model["support_node"], "support_at": [float(v) for v in nodes[model["support_node"]]] if free_flight else None,
        "linear_solver": LINEAR_SOLVER,
    }

    t0 = time.time()
    bdf, bar_props, n_shell_eids = write_bdf(model, with_spc=True)
    keep = bool(os.environ.get("AEROTELIC_FEA_KEEP"))
    if keep:
        bdf.write_bdf(os.path.join(job_dir, "structure_tacs.bdf"), size=16, is_double=True)
    seconds_bdf = time.time() - t0
    dofs = 6 * (n_nodes + n_extra)
    log("  writing the TACS model: %d nodes, %d shells (%d of them ribs) in %d component(s), %d mass item(s), %d tie(s), %d link(s), %d bar(s), %d dofs"
        % (n_nodes, len(shells), len(model["rib_faces"]), len(model["components"]), len(mass_items), len(ties), len(links), len(bars), dofs))

    comm = MPI.COMM_WORLD
    dv_table = []
    fea = pyTACS(bdf, comm=comm, options={"printDebug": False})
    fea.initialize(_element_callback(model, bar_props, dv_table))
    problem_options = {"writeSolution": bool(solver_opts.get("f5")), "printLevel": 0, "outputDir": job_dir}
    if solver_opts.get("pc_fill_level"):
        problem_options["PCFillLevel"] = int(solver_opts["pc_fill_level"])
    if solver_opts.get("subspace_size"):
        problem_options["subSpaceSize"] = int(solver_opts["subspace_size"])
    problem = fea.createStaticProblem("aerotelic", options=problem_options)
    assembler = problem.assembler
    vpn = assembler.getVarsPerNode()
    all_ids = np.arange(1, n_nodes + n_extra + 1)
    local_of = _local_index(fea, all_ids)
    tacs_dofs = int(assembler.getNumOwnedNodes()) * vpn
    result["tacs_dofs"] = tacs_dofs

    # The loads: the nodal loads (pressures and point forces) on the
    # structural nodes, the inertial field through the mass matrix.
    nodal_load = model["nodal_load"]
    loaded = np.nonzero(np.any(nodal_load != 0.0, axis=1))[0]
    if len(loaded):
        F6 = np.zeros((len(loaded), vpn))
        F6[:, :3] = nodal_load[loaded]
        problem.addLoadToNodes([int(v) + 1 for v in loaded], F6, nastranOrdering=True)
    inertial = _assemble_mass_load(fea, problem, model, local_of)
    inertial_arr = inertial.getArray().copy()
    problem.addLoadToRHS(inertial)
    # What the mass matrix put on the model, against m (g - a0): a check of
    # the mass the elements carry.
    f_inertial = np.zeros(3)
    for i in range(n_nodes + n_extra):
        l_ = local_of[i]
        f_inertial += inertial_arr[vpn * l_:vpn * l_ + 3]
    expected_inertial = model["mass"] * (model["gravity"] - model["a0"])
    log("  inertial load from TACS's mass matrix (%.6g, %.6g, %.6g) N against m (g - a0) = (%.6g, %.6g, %.6g) N"
        % (f_inertial[0], f_inertial[1], f_inertial[2], expected_inertial[0], expected_inertial[1], expected_inertial[2]))
    mass_mismatch = float(np.linalg.norm(f_inertial - expected_inertial)) / max(float(np.linalg.norm(expected_inertial)), 1e-300) if np.linalg.norm(expected_inertial) > 0 else 0.0

    # The functions the adjoint is run for: the structural mass, and the KS
    # failure index of every component with an allowable.
    problem.addFunction("mass", functions.StructuralMass)
    for comp in model["components"]:
        # The callback recorded pyTACS's component index of each property.
        if not comp["has_strengths"] or "comp_id" not in comp:
            continue
        problem.addFunction("ks_%d" % int(comp["prop"]), functions.KSFailure, compIDs=[int(comp["comp_id"])], ksWeight=float(solver_opts.get("ks_weight") or KS_WEIGHT))

    log("  solving %d dofs (%d in TACS with the rigid elements' multipliers) with %s on %d shells%s"
        % (dofs, tacs_dofs, LINEAR_SOLVER, len(shells), ", %d laminate region(s)" % len(model["laminates"]) if model["laminates"] else ""))
    t0 = time.time()
    converged = problem.solve()
    seconds_solve = time.time() - t0
    warnings = []
    if converged is False:
        warnings.append("TACS's GMRES did not reach its tolerance; the force balance below says how far off the solution is.")
        log("  WARNING: " + warnings[-1])

    # The state: displacements and rotations per node, the extra nodes too
    # (the assembler holds the solved variables; read them from it).
    u_vec = assembler.createVec()
    assembler.getVariables(u_vec)
    state = np.asarray(u_vec.getArray(), dtype=np.float64).reshape(-1).copy()
    disp_all = np.zeros((n_nodes + n_extra, 3))
    rot_all = np.zeros((n_nodes + n_extra, 3))
    for i in range(n_nodes + n_extra):
        l_ = local_of[i]
        disp_all[i] = state[vpn * l_:vpn * l_ + 3]
        rot_all[i] = state[vpn * l_ + 3:vpn * l_ + 6]
    displacement, rotation = disp_all[:n_nodes], rot_all[:n_nodes]
    mass_displacement = disp_all[n_nodes:]

    # The reactions: K u - F at the fixed degrees of freedom, K u assembled
    # without the conditions (no auxiliary elements were added, so the
    # residual is the internal force alone).
    res = assembler.createVec()
    try:
        assembler.assembleRes(res, applyBCs=False)
    except TypeError:
        assembler.assembleRes(res)
        warnings_early = ["this TACS cannot assemble without its boundary conditions: the reactions at the fixed nodes are not recovered and the balance below is meaningless"]
    else:
        warnings_early = []
    internal = res.getArray().copy()
    external = np.asarray(problem.F.getArray(), dtype=np.float64).copy()
    reaction = np.zeros((n_nodes, 3))
    reaction_moment = np.zeros((n_nodes, 3))
    for i in model["support_nodes"]:
        l_ = local_of[i]
        r6 = internal[vpn * l_:vpn * l_ + 6] - external[vpn * l_:vpn * l_ + 6]
        reaction[i] = r6[:3]
        reaction_moment[i] = r6[3:6]

    # The bars' axial forces from their ends' displacements.
    bar_results = []
    for k, bar in enumerate(bars):
        pa, pb = model["bar_ends"][k]
        e = pb - pa
        length = float(np.linalg.norm(e))
        e /= length
        du = disp_all[bar["b"]] - disp_all[bar["a"]]
        force = bar["youngs_modulus"] * bar["area"] * float(np.dot(du, e)) / length
        bar_results.append({"name": bar["name"], "force": force, "stress": force / bar["area"], "length": length,
                            "a": [float(v) for v in pa], "b": [float(v) for v in pb], "mass": bar["density"] * bar["area"] * length})

    # The stresses, from the displacement field (see the module docstring).
    log("  post-processing: stresses, margins, the force balance")
    warnings.extend(warnings_early)
    n_elements = len(shells)
    von_mises_elem = np.zeros(n_elements)
    reserve_elem = np.full(n_elements, -1.0)
    shear_elem = np.zeros(n_elements)
    deviation_elem = np.zeros(n_elements)
    eps0, kappa, _frames = shell_strains(nodes, shells, displacement, rotation, None)
    for comp in model["components"]:
        sel = comp["shells"]
        if comp["iso"] is not None:
            von_mises_elem[sel] = recover_isotropic(eps0[sel], kappa[sel], comp["iso"]["E"], comp["iso"]["nu"], comp["thickness"])
        else:
            # The laminate's strains in its own reference frame: re-evaluate
            # with the component's axis as e1.
            e0, ka, _ = shell_strains(nodes, shells[sel], displacement, rotation, np.tile(np.asarray(comp["axis"]), (len(sel), 1)))
            vm, rf = recover_laminate(e0, ka, comp["rows"], comp["has_strengths"])
            von_mises_elem[sel] = vm
            reserve_elem[sel] = rf
    for (rid, kind), stats in model["laminates"].items():
        if stats["draped"]:
            shear_elem[stats["shells"]] = stats["shear_per_face"]
            deviation_elem[stats["shells"]] = stats["deviation_per_face"]

    def margin(allowable, peak):
        return (allowable / peak - 1.0) if (allowable > 0.0 and peak > 0.0) else 0.0
    per_material = []
    shell_region, shell_is_rib, shell_area = model["shell_region"], model["shell_is_rib"], model["shell_area"]
    for rid in model["ids"]:
        r = model["by_id"][rid]
        label = r.get("label", "")
        for kind in ("skin", "rib"):
            card = r.get(kind)
            if not card:
                continue
            is_rib = kind == "rib"
            sel = np.nonzero((shell_region == rid) & (shell_is_rib == is_rib))[0]
            peak = float(von_mises_elem[sel].max()) if len(sel) else 0.0
            mass_here = float((model["shell_rho_t"][sel] * shell_area[sel]).sum())
            entry = {"id": rid, "label": "%s %s" % (label, "ribs" if is_rib else "skin"), "kind": kind, "max_von_mises": peak, "mass": mass_here}
            stats = model["laminates"].get((rid, kind))
            if stats is None:
                a = float(card["material"].get("allowable_stress", 0.0) or 0.0)
                entry.update({"criterion": "von Mises", "allowable": a, "margin": margin(a, peak), "reserve_factor": (a / peak) if (a > 0.0 and peak > 0.0) else 0.0})
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
    result.update({"elements": int(n_elements + len(mass_items) + len(bars)), "shells": int(len(shells)), "rib_shells": int(len(model["rib_faces"])),
                   "seconds_mdpa": seconds_bdf, "ties": len(ties), "links": len(links), "bars": bar_results, "mass_bars": model["mass_of_bars"],
                   "components": [{"prop": int(c["prop"]), "label": c["label"], "region": int(c["region"]), "kind": c["kind"], "shells": int(len(c["shells"])), "thickness": float(c["thickness"]),
                                   "laminate": c["iso"] is None} for c in model["components"]]})

    # The force balance, as the Kratos script measures it.
    mass, gravity, a0, alpha = model["mass"], model["gravity"], model["a0"], model["alpha"]
    face_pressure, n_area, face_area = model["face_pressure"], model["n_area"], model["face_area"]
    applied_gravity = mass * gravity
    applied_pressure = -(face_pressure[:, None] * n_area).sum(axis=0) if len(model["faces"]) else np.zeros(3)
    applied_inertial = -mass * a0
    applied_point = model["applied_point"]
    reaction_sum = reaction.sum(axis=0)
    reaction_moment_sum = reaction_moment.sum(axis=0)
    applied_total = applied_gravity + applied_pressure + applied_point + applied_inertial
    load_scale = max(float(np.linalg.norm(applied_total)),
                     float(np.linalg.norm(applied_gravity)) + float((np.abs(face_pressure) * face_area).sum()) + sum(float(np.linalg.norm(f)) for _, f in model["point_loads"]))
    imbalance = float(np.linalg.norm(reaction_sum + applied_total))
    balance_error = imbalance / load_scale if load_scale > 0.0 else 0.0
    moment_error = 0.0
    if free_flight:
        moment_error = float(np.linalg.norm(reaction_moment_sum)) / (load_scale * model["diagonal"]) if load_scale > 0.0 and model["diagonal"] > 0.0 else 0.0
    balanced = balance_error <= sf.BALANCE_TOLERANCE and moment_error <= sf.BALANCE_TOLERANCE
    if not balanced:
        warnings.append("the reactions miss the applied loads by %.1f%% (|R + F| = %.4g N against loads of %.4g N%s): the linear solver (%s) did not converge, so the displacements and stresses are not to be trusted."
                        % (100.0 * balance_error, imbalance, load_scale, (", moment residual %.1f%%" % (100.0 * moment_error)) if free_flight else "", LINEAR_SOLVER))
        log("  WARNING: " + warnings[-1])
    if mass_mismatch > 1e-6:
        warnings.append("the mass TACS's elements carry differs from the shells' rho t A by %.2g (relative, from the inertial resultant)." % mass_mismatch)
        log("  WARNING: " + warnings[-1])
    item_results = []
    field = model["nodal_acceleration"]
    for k, it in enumerate(mass_items):
        item_results.append({"name": it["name"], "mass": it["mass"], "at": [float(v) for v in it["at"]], "node": int(n_nodes + k),
                             "attach_nodes": int(len(it["attach"])), "attach_centre": [float(v) for v in it["attach_centre"]],
                             "displacement": [float(v) for v in mass_displacement[k]], "force": [float(v) for v in it["mass"] * field[n_nodes + k]]})
    mean_normal_disp = 0.0
    faces = model["faces"]
    if len(faces):
        node_normal = np.zeros((n_nodes, 3))
        for k in range(3):
            np.add.at(node_normal, faces[:, k], n_area)
        wet_nodes = np.unique(faces)
        nn = node_normal[wet_nodes]
        nn /= np.maximum(np.linalg.norm(nn, axis=1), 1e-300)[:, None]
        mean_normal_disp = float(np.einsum("ij,ij->i", displacement[wet_nodes], nn).mean())

    # The adjoint: the functions' values and their gradients with respect
    # to the thickness design variables.
    sensitivities = {"design_variables": dv_table, "functions": []}
    try:
        funcs, sens = {}, {}
        problem.evalFunctions(funcs)
        problem.evalFunctionsSens(sens, includeDVSens=True, includeXptSens=False)
        var_name = problem.getVarName() if hasattr(problem, "getVarName") else "struct"
        for key, value in funcs.items():
            short = key.split("_", 1)[1] if key.startswith("aerotelic_") else key
            grad = sens.get(key, {}).get(var_name)
            entry = {"name": short, "value": float(np.real(value)), "gradient": [float(np.real(v)) for v in np.asarray(grad).reshape(-1)] if grad is not None else []}
            if short.startswith("ks_"):
                prop = int(short[3:])
                comp = next(c for c in model["components"] if int(c["prop"]) == prop)
                entry.update({"component": comp["label"], "region": int(comp["region"]), "kind": comp["kind"], "name": "ks_failure"})
            sensitivities["functions"].append(entry)
        log("  adjoint: %d function(s) over %d thickness variable(s)" % (len(sensitivities["functions"]), len(dv_table)))
    except Exception as exc:  # noqa: BLE001 -- the gradients are a bonus on a solved case
        warnings.append("the adjoint sensitivities could not be evaluated: %s: %s" % (type(exc).__name__, exc))
        log("  WARNING: " + warnings[-1])

    # The natural frequencies: the same model with no loads; free-free in
    # free flight (a second model without the reference node's conditions),
    # the rigid modes dropped.
    n_modes = int(solver_opts.get("modes") or 0)
    frequencies, mode_shapes, seconds_modes = np.zeros(0), None, 0.0
    if n_modes > 0:
        t0 = time.time()
        log("  natural frequencies: the lowest %d modes of the same model" % n_modes)
        try:
            if free_flight:
                bdf_free, bar_props_free, _ = write_bdf(model, with_spc=False)
                fea_m = pyTACS(bdf_free, comm=comm, options={"printDebug": False})
                fea_m.initialize(_element_callback(model, bar_props_free, []))
                ask = n_modes + 6
            else:
                fea_m = fea
                ask = n_modes
            sigma = (2.0 * np.pi * 0.1) ** 2
            modal = fea_m.createModalProblem("modes", sigma, ask, options={"writeSolution": False, "printLevel": 0})
            modal.solve()
            local_m = _local_index(fea_m, np.arange(1, n_nodes + 1))
            vpn_m = modal.assembler.getVarsPerNode()
            eigs, shapes = [], []
            for i in range(ask):
                lam, vec = modal.getVariables(i)
                eigs.append(float(np.real(lam)))
                shapes.append(np.asarray(vec.getArray() if hasattr(vec, "getArray") else vec, dtype=np.float64).copy())
            eigs = np.array(eigs)
            largest = float(np.abs(eigs).max()) if len(eigs) else 0.0
            keep_mode = eigs > RIGID_MODE_FRACTION * largest if free_flight else eigs > 0.0
            if free_flight and int((~keep_mode).sum()) != 6:
                log("  note: %d rigid mode(s) found and dropped (6 expected)" % int((~keep_mode).sum()))
            order = np.argsort(eigs)
            picked = [i for i in order if keep_mode[i]][:n_modes]
            frequencies = np.sqrt(np.maximum(eigs[picked], 0.0)) / (2.0 * np.pi)
            mode_shapes = []
            for i in picked:
                shape = np.zeros((n_nodes, 3))
                for j in range(n_nodes):
                    l_ = local_m[j]
                    shape[j] = shapes[i][vpn_m * l_:vpn_m * l_ + 3]
                peak = float(np.linalg.norm(shape, axis=1).max())
                mode_shapes.append(shape / peak if peak > 0.0 else shape)
            log("  natural frequencies (%s, %.2f s): %s Hz" % (EIGENSOLVER, time.time() - t0, ", ".join("%.4g" % f for f in frequencies)))
        except Exception as exc:  # noqa: BLE001 -- a failure of the eigensolve is a warning on a good static result
            warnings.append("the eigensolve failed: %s: %s" % (type(exc).__name__, exc))
            log("  WARNING: " + warnings[-1])
            frequencies, mode_shapes = np.zeros(0), None
        seconds_modes = time.time() - t0
    result.update({"modes": n_modes, "natural_frequencies": [float(f) for f in frequencies], "eigensolver": EIGENSOLVER if len(frequencies) else "", "seconds_modes": seconds_modes})

    u_mag = np.linalg.norm(displacement, axis=1)
    i_max = int(np.argmax(u_mag))
    result.update({
        "nodes": int(n_nodes), "dofs": int(dofs),
        "mass": mass, "mass_structure": model["mass_structure"], "mass_items": model["mass_of_items"],
        "mass_centre": [float(v) for v in model["cg"]], "inertia_cg": [[float(v) for v in row] for row in model["inertia_cg"]],
        "items": item_results,
        "max_displacement": float(u_mag[i_max]), "max_displacement_at": [float(v) for v in nodes[i_max]],
        "max_von_mises": float(von_mises_elem[i_vm]) if n_elements else 0.0, "max_von_mises_material": int(shell_region[i_vm]) if n_elements else 0,
        "per_material": per_material,
        "reaction": [float(v) for v in reaction_sum], "reaction_moment": [float(v) for v in reaction_moment_sum],
        "applied_gravity": [float(v) for v in applied_gravity], "applied_pressure": [float(v) for v in applied_pressure], "applied_inertial": [float(v) for v in applied_inertial],
        "rigid_acceleration": [float(v) for v in a0], "rigid_angular_acceleration": [float(v) for v in alpha],
        "load_factor_at_cg": float(-(gravity[2] - a0[2]) / G0),
        "mean_normal_displacement": mean_normal_disp,
        "balance_error": balance_error, "moment_error": moment_error, "balanced": bool(balanced), "load_scale": load_scale,
        "warnings": warnings, "seconds_solve": seconds_solve, "vtk": outputs["vtk"], "sensitivities": sensitivities,
    })
    if model["laminates"]:
        result["laminates"] = len(model["laminates"])

    t_vtk = time.time()
    vtk_path = host_path(outputs["vtk"])
    log("  writing the solution to %s" % outputs["vtk"])
    cell_data = {"material": shell_region.astype(np.int64), "kind": np.where(shell_is_rib, 2, 1).astype(np.int64), "von_mises_element": von_mises_elem,
                 "component": model["shell_prop"].astype(np.int64)}
    if model["laminates"]:
        cell_data.update({"reserve_factor": reserve_elem, "ply_shear_max": shear_elem, "fibre_deviation_max": deviation_elem})
    point_data = {"displacement": displacement, "rotation": rotation, "von_mises": sf.nodal_average(n_nodes, [shells], von_mises_elem)}
    if mode_shapes is not None:
        for k in range(len(frequencies)):
            point_data["mode_%d" % (k + 1)] = mode_shapes[k]
    sf.write_vtk(vtk_path, nodes, [shells], [sf.VTK_TRIANGLE], point_data, cell_data)
    result["seconds_vtk"] = time.time() - t_vtk
    result["seconds"] = time.time() - t_start
    with open(os.path.join(job_dir, outputs["result"]), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    log("  solved %d dofs (%d shells, %d of them ribs, %d mass item(s)) on TACS %s in %.2f s: mass %.6g kg, max displacement %.4g m, max von Mises %.4g Pa (region %d); "
        "reaction (%.4g, %.4g, %.4g) N%s against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g)%s: %s; wrote %s"
        % (dofs, len(shells), len(model["rib_faces"]), len(mass_items), result["tacs_version"], seconds_solve, mass, result["max_displacement"], result["max_von_mises"], result["max_von_mises_material"],
           reaction_sum[0], reaction_sum[1], reaction_sum[2], (" and (%.4g, %.4g, %.4g) Nm" % tuple(reaction_moment_sum)) if free_flight else "",
           applied_gravity[0], applied_gravity[1], applied_gravity[2], applied_pressure[0], applied_pressure[1], applied_pressure[2],
           applied_point[0], applied_point[1], applied_point[2], (" + inertial (%.4g, %.4g, %.4g)" % tuple(applied_inertial)) if free_flight else "",
           ("balanced to %.2e" % max(balance_error, moment_error)) if balanced else ("NOT BALANCED (%.1f%%)" % (100.0 * max(balance_error, moment_error))), outputs["vtk"]))
    return result


def main(argv):
    if len(argv) != 2:
        log("usage: structural_fea_tacs.py <job directory>")
        return 2
    job_dir = argv[1]
    try:
        run_job(job_dir)
        return 0
    except JobError as exc:
        tg.write_failure(job_dir, str(exc))
        log("error: %s" % exc)
        return 1
    except Exception as exc:  # noqa: BLE001
        tg.write_failure(job_dir, "%s: %s" % (type(exc).__name__, exc))
        log("error: %s: %s" % (type(exc).__name__, exc))
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
