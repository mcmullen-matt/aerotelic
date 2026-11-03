"""Numerical homogenization of a periodic lattice cell: the effective
elasticity tensor of the lattice as a continuum, from a finite-element
solve of ONE unit cell.

Why. A lattice-filled body has far too many struts or walls to mesh for a
desktop finite-element analysis (the p38a airframe's skin/core tetrahedra
alone run to 600,000). The classical answer is to replace the core by an
equivalent homogeneous material: a solid element whose constitutive matrix
is the cell's AVERAGE response, computed once per lattice (type, phase,
volume fraction, base material) rather than per body. That is what this
module computes; structural_fea.py hands the result to Kratos as the
ELASTICITY_TENSOR of its UserProvidedLinearElastic3DLaw on the core
tetrahedra.

Method (Andreassen & Andreasen, "How to determine composite material
properties using numerical homogenization", Comput. Mater. Sci. 83, 2014,
extended from their 2-D code to 3-D voxels of any aspect). The cell is a
regular grid of trilinear hexahedra, one per voxel, solid where the lattice
field is <= 0 at the voxel centre (lattice.hpp's SampleUnitCell). Nodes
are numbered periodically, so the mesh has no boundary: the displacement
on one face IS the displacement on the opposite face. Six load cases
apply the unit macro strains e_k (Voigt, engineering shear) through the
equivalent nodal forces F_k = sum_e k_e chi0_k, where chi0_k is the
element displacement that would produce e_k uniformly; the periodic
fluctuation chi_k solves K chi_k = F_k (one factorisation, six right-hand
sides, three translations pinned); and the effective tensor is the
strain energy of the corrected fields,

    C_ij = (1 / |Y|) sum_e (chi0_i - chi_i)^T k_e (chi0_j - chi_j),

|Y| being the whole cell, void included. A fully solid cell returns the
base tensor exactly, which is the first thing homogenize_check.py tests.

Voigt order is Kratos's: xx, yy, zz, xy, yz, xz, with ENGINEERING shear
strains (gamma = 2 eps), so C maps [eps_xx, eps_yy, eps_zz, gamma_xy,
gamma_yz, gamma_xz] to [s_xx, s_yy, s_zz, s_xy, s_yz, s_xz].

Scale. The result does not depend on the cell SIZE -- only on the voxel
pattern (topology and volume fraction), the voxel aspect ratios and the
base material -- so a lattice homogenised once serves every body it fills.
Honeycombs come in as one period of the cross-section by two voxels along
the prism axis (the field does not vary along it), which is exact.

Micro stress. The macro stress a homogenised element reports is an
average: the struts carry more. For the core's margin the same six
solves give the stress in every voxel per unit macro strain, T_e (6 x 6).
Two things come out of that: `amplification`, the largest ratio of peak
voxel von Mises to the Frobenius norm of the macro stress over a sample of
macro strain directions (a bound a solver can apply per element with one
multiply), and `critical`, the influence tensors of the voxels that set
that peak in any sampled direction, so the solver can evaluate the actual
micro von Mises of each core element's own strain state against those
few voxels rather than all of them. Voxels not connected to the main
solid component (debris of a coarse voxelisation) are dropped and
reported, since a floating piece carries nothing and makes the system
singular.
"""

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.sparse.csgraph import connected_components

VOIGT = ("xx", "yy", "zz", "xy", "yz", "xz")

# Corner offsets of the hexahedron, in the node order the stiffness and
# strain matrices below are built in: the bottom face anticlockwise seen
# from +z, then the top face above it.
CORNERS = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                    [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], dtype=np.int64)


class HomogenizationError(Exception):
    pass


# ---------------------------------------------------------------------------
# The base material and the element
# ---------------------------------------------------------------------------

def isotropic_tensor(youngs_modulus, poisson_ratio):
    """The 6 x 6 elasticity tensor of an isotropic solid, Kratos Voigt
    order, engineering shear."""
    E, nu = float(youngs_modulus), float(poisson_ratio)
    if not E > 0.0:
        raise HomogenizationError("the base material has no stiffness (E = %g)" % E)
    if not 0.0 <= nu < 0.5:
        raise HomogenizationError("the base material's Poisson's ratio %g is not in [0, 0.5)" % nu)
    lam = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    mu = E / (2.0 * (1.0 + nu))
    D = np.zeros((6, 6))
    D[:3, :3] = lam
    D[np.arange(3), np.arange(3)] += 2.0 * mu
    D[3:, 3:] = mu * np.eye(3)
    return D


def _shape_gradients(xi, eta, zeta):
    """dN/d(xi, eta, zeta) of the eight trilinear shape functions on the
    [-1, 1]^3 parent, (8, 3), in CORNERS' order."""
    s = 2.0 * CORNERS - 1.0   # the corners' parent coordinates, +-1
    g = np.empty((8, 3))
    g[:, 0] = 0.125 * s[:, 0] * (1 + s[:, 1] * eta) * (1 + s[:, 2] * zeta)
    g[:, 1] = 0.125 * s[:, 1] * (1 + s[:, 0] * xi) * (1 + s[:, 2] * zeta)
    g[:, 2] = 0.125 * s[:, 2] * (1 + s[:, 0] * xi) * (1 + s[:, 1] * eta)
    return g


def _strain_matrix(grad_xyz):
    """B (6 x 24) from the shape gradients dN/dx (8, 3): engineering shear
    rows in Kratos order xy, yz, xz."""
    B = np.zeros((6, 24))
    for a in range(8):
        dx, dy, dz = grad_xyz[a]
        c = 3 * a
        B[0, c] = dx
        B[1, c + 1] = dy
        B[2, c + 2] = dz
        B[3, c], B[3, c + 1] = dy, dx
        B[4, c + 1], B[4, c + 2] = dz, dy
        B[5, c], B[5, c + 2] = dz, dx
    return B


def hex_element(h, D):
    """Stiffness k_e (24 x 24) of a hx x hy x hz trilinear hexahedron of
    material D, by 2 x 2 x 2 Gauss integration, and the strain matrix at
    its centre (6 x 24)."""
    hx, hy, hz = (float(v) for v in h)
    inv_j = np.array([2.0 / hx, 2.0 / hy, 2.0 / hz])
    det_j = hx * hy * hz / 8.0
    ke = np.zeros((24, 24))
    g = 1.0 / np.sqrt(3.0)
    for xi in (-g, g):
        for eta in (-g, g):
            for zeta in (-g, g):
                B = _strain_matrix(_shape_gradients(xi, eta, zeta) * inv_j)
                ke += B.T @ D @ B * det_j
    Bc = _strain_matrix(_shape_gradients(0.0, 0.0, 0.0) * inv_j)
    return 0.5 * (ke + ke.T), Bc


def unit_strain_displacements(h):
    """chi0 (24 x 6): the corner displacements of one element under each
    unit macro strain, from the corner positions (engineering shear
    gamma_xy = 1 is u_x = y / 2, u_y = x / 2)."""
    x = CORNERS * np.asarray(h, dtype=np.float64)
    chi0 = np.zeros((24, 6))
    for a in range(8):
        px, py, pz = x[a]
        c = 3 * a
        chi0[c, 0] = px
        chi0[c + 1, 1] = py
        chi0[c + 2, 2] = pz
        chi0[c, 3], chi0[c + 1, 3] = 0.5 * py, 0.5 * px
        chi0[c + 1, 4], chi0[c + 2, 4] = 0.5 * pz, 0.5 * py
        chi0[c, 5], chi0[c + 2, 5] = 0.5 * pz, 0.5 * px
    return chi0


# ---------------------------------------------------------------------------
# Stress measures
# ---------------------------------------------------------------------------

def von_mises(s):
    """Von Mises of stress Voigt vectors, over the last axis of length 6."""
    s = np.asarray(s)
    sxx, syy, szz, sxy, syz, sxz = (s[..., i] for i in range(6))
    return np.sqrt(np.maximum(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3.0 * (sxy ** 2 + syz ** 2 + sxz ** 2), 0.0))


def frobenius(s):
    """The tensor norm sqrt(s_ij s_ij) of stress Voigt vectors (the shear
    components count twice, being two tensor entries each)."""
    s = np.asarray(s)
    return np.sqrt((s[..., :3] ** 2).sum(axis=-1) + 2.0 * (s[..., 3:] ** 2).sum(axis=-1))


def engineering_constants(C):
    """Young's moduli, shear moduli and Poisson's ratios from the
    compliance, as a solver's user would read them."""
    S = np.linalg.inv(C)
    E = 1.0 / np.diag(S)[:3]
    G = 1.0 / np.diag(S)[3:]
    return {
        "E": [float(v) for v in E],
        "G": [float(v) for v in G],
        # nu_ij = -eps_j / eps_i under s_i alone: -S_ji / S_ii.
        "nu_xy": float(-S[1, 0] / S[0, 0]), "nu_yx": float(-S[0, 1] / S[1, 1]),
        "nu_yz": float(-S[2, 1] / S[1, 1]), "nu_zy": float(-S[1, 2] / S[2, 2]),
        "nu_xz": float(-S[2, 0] / S[0, 0]), "nu_zx": float(-S[0, 2] / S[2, 2]),
    }


# ---------------------------------------------------------------------------
# The homogenization
# ---------------------------------------------------------------------------

def homogenize(solid, h, youngs_modulus, poisson_ratio, samples=2000, critical=64, seed=1, log=None):
    """`solid` is a bool array (nx, ny, nz), True where the voxel is
    material, periodic in all three directions; `h` the voxel size along
    x, y, z (only the ratios matter). Returns a dict:

        C                 6 x 6 effective elasticity tensor (Voigt, Kratos order)
        relative_density  solid voxels / all voxels, after dropping debris
        dropped_fraction  solid voxels not connected to the main component,
                          as a fraction of the solid, dropped from the solve
        engineering       E, G and nu from the compliance
        amplification     max over sampled macro strains of
                          peak voxel von Mises / ||macro stress||_F
        amplification_unit  the same ratio for the six unit strains alone
        critical          (n, 6, 6) influence tensors of the voxels that
                          set the peak for some sampled direction: voxel
                          stress = T @ macro strain
        dofs, elements, nodes, seconds
    """
    import time
    t0 = time.time()
    solid = np.asarray(solid, dtype=bool)
    if solid.ndim != 3:
        raise HomogenizationError("the cell must be a 3-D voxel array")
    nx, ny, nz = solid.shape
    if min(nx, ny, nz) < 1:
        raise HomogenizationError("the cell has no voxels")
    h = np.asarray(h, dtype=np.float64).reshape(3)
    if not (h > 0.0).all():
        raise HomogenizationError("the voxel size must be positive")
    D = isotropic_tensor(youngs_modulus, poisson_ratio)

    cells = np.argwhere(solid)          # (ne, 3) of (i, j, k)
    if len(cells) == 0:
        raise HomogenizationError("the cell is empty: no voxel is solid")
    if len(cells) == solid.size:
        # Nothing to solve: the cell is the base material.
        C = D.copy()
        T = np.broadcast_to(D, (1, 6, 6)).copy()
        amp, amp_unit = _amplification(C, T, samples, seed)
        return {
            "C": C, "relative_density": 1.0, "dropped_fraction": 0.0,
            "engineering": engineering_constants(C),
            "amplification": amp, "amplification_unit": amp_unit, "critical": T,
            "dofs": 0, "elements": int(solid.size), "nodes": 0, "seconds": time.time() - t0,
        }

    # Periodic node ids: node (i, j, k) with i mod nx etc.
    def node_id(ijk):
        return ((ijk[:, 0] % nx) * ny + (ijk[:, 1] % ny)) * nz + (ijk[:, 2] % nz)

    conn = np.empty((len(cells), 8), dtype=np.int64)
    for a in range(8):
        conn[:, a] = node_id(cells + CORNERS[a])

    # Drop the solid that is not connected to the main body of it.
    n_nodes_all = nx * ny * nz
    rows = np.repeat(conn[:, 0], 7)
    cols = conn[:, 1:].ravel()
    graph = sp.coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n_nodes_all, n_nodes_all))
    _, label = connected_components(graph, directed=False)
    elem_label = label[conn[:, 0]]
    main = np.bincount(elem_label).argmax()
    keep = elem_label == main
    dropped_fraction = 1.0 - keep.sum() / float(len(cells))
    cells, conn = cells[keep], conn[keep]
    ne = len(cells)
    if log and dropped_fraction > 0.0:
        log("  homogenize: dropped %.2f%% of the solid as voxels not connected to the main component" % (100.0 * dropped_fraction))

    used, conn = np.unique(conn, return_inverse=True)
    conn = conn.reshape(ne, 8)
    n_nodes = len(used)
    edof = (3 * conn[:, :, None] + np.arange(3)[None, None, :]).reshape(ne, 24)
    ndof = 3 * n_nodes

    ke, Bc = hex_element(h, D)
    chi0 = unit_strain_displacements(h)                 # (24, 6)
    fe = ke @ chi0                                       # (24, 6)

    # K and F.
    iK = np.repeat(edof, 24, axis=1).ravel()
    jK = np.tile(edof, (1, 24)).ravel()
    K = sp.coo_matrix((np.tile(ke.ravel(), ne), (iK, jK)), shape=(ndof, ndof)).tocsr()
    F = np.zeros((ndof, 6))
    np.add.at(F, edof.ravel(), np.tile(fe, (ne, 1)))

    # Pin the three translations at one node (periodic: no rotations to pin).
    free = np.ones(ndof, dtype=bool)
    free[edof[0, :3]] = False
    free_idx = np.nonzero(free)[0]
    Kff = K[free_idx][:, free_idx].tocsc()
    chi = np.zeros((ndof, 6))
    try:
        lu = spla.splu(Kff)
        chi[free_idx] = lu.solve(F[free_idx])
    except MemoryError:
        # Fall back to a Jacobi-preconditioned conjugate gradient, one
        # right-hand side at a time.
        diag = Kff.diagonal()
        M = spla.LinearOperator(Kff.shape, matvec=lambda v: v / diag)
        for k in range(6):
            x, info = spla.cg(Kff, F[free_idx, k], M=M, rtol=1e-10, maxiter=20000)
            if info != 0:
                raise HomogenizationError("the unit-cell solve did not converge (cg info %d)" % info)
            chi[free_idx, k] = x

    # The corrected element fields and the energy average.
    ue = chi0[None, :, :] - chi[edof]                   # (ne, 24, 6)
    volume = float(nx * ny * nz) * float(np.prod(h))
    C = np.einsum("eai,ab,ebj->ij", ue, ke, ue) / volume
    C = 0.5 * (C + C.T)

    # Voxel stress per unit macro strain, at the voxel centre.
    T = np.einsum("ab,bc,ecj->eaj", D, Bc, ue)           # (ne, 6, 6)
    amp, amp_unit, critical_idx = _amplification(C, T, samples, seed, return_critical=critical)

    return {
        "C": C,
        "relative_density": ne / float(solid.size),
        "dropped_fraction": float(dropped_fraction),
        "engineering": engineering_constants(C),
        "amplification": amp,
        "amplification_unit": amp_unit,
        "critical": T[critical_idx],
        "dofs": int(ndof), "elements": int(ne), "nodes": int(n_nodes),
        "seconds": time.time() - t0,
    }


def _amplification(C, T, samples, seed, return_critical=0):
    """Peak voxel von Mises over the Frobenius norm of the macro stress,
    for the six unit strains and a sample of random macro strain
    directions; the voxels that set the peak, if asked."""
    rng = np.random.default_rng(seed)
    A = np.concatenate([np.eye(6), -np.eye(6), rng.standard_normal((max(int(samples), 0), 6))])
    A /= np.linalg.norm(A, axis=1)[:, None]
    macro = frobenius(A @ C.T)                           # (ns,)
    ratios = np.empty(len(A))
    arg = np.empty(len(A), dtype=np.int64)
    chunk = max(1, int(2e7 // max(T.shape[0] * 6, 1)))
    for s in range(0, len(A), chunk):
        a = A[s:s + chunk]                               # (c, 6)
        micro = np.einsum("eaj,cj->eca", T, a)           # (ne, c, 6)
        vm = von_mises(micro)                            # (ne, c)
        arg[s:s + chunk] = vm.argmax(axis=0)
        ratios[s:s + chunk] = vm.max(axis=0) / np.maximum(macro[s:s + chunk], 1e-300)
    amp = float(ratios.max())
    amp_unit = [float(v) for v in ratios[:6]]
    if not return_critical:
        return amp, amp_unit
    order = np.argsort(-ratios)
    critical_idx = []
    seen = set()
    for i in order:
        e = int(arg[i])
        if e not in seen:
            seen.add(e)
            critical_idx.append(e)
            if len(critical_idx) >= return_critical:
                break
    return amp, amp_unit, np.array(critical_idx, dtype=np.int64)


def micro_von_mises(critical, macro_strain):
    """The estimated peak voxel von Mises for macro strains (n, 6) from the
    critical influence tensors (m, 6, 6): the max over those voxels of the
    von Mises of T @ strain."""
    critical = np.asarray(critical)
    macro_strain = np.atleast_2d(macro_strain)
    out = np.zeros(len(macro_strain))
    chunk = max(1, int(2e7 // max(critical.shape[0] * 6, 1)))
    for s in range(0, len(macro_strain), chunk):
        eps = macro_strain[s:s + chunk]                  # (c, 6)
        micro = np.einsum("maj,cj->cma", critical, eps)  # (c, m, 6)
        out[s:s + chunk] = von_mises(micro).max(axis=1)
    return out


# ---------------------------------------------------------------------------
# Cells from the application
# ---------------------------------------------------------------------------

def read_cell(json_path):
    """A cell as aerotelic_lattice_cell / the application write it: the
    JSON header beside a float32 field file. Returns (solid, h)."""
    import json
    import os
    with open(json_path, "r", encoding="utf-8-sig") as f:
        head = json.load(f)
    nx, ny, nz = int(head["nx"]), int(head["ny"]), int(head["nz"])
    period = np.asarray(head["period"], dtype=np.float64)
    raw = os.path.join(os.path.dirname(os.path.abspath(json_path)), head["file"])
    values = np.fromfile(raw, dtype="<f4")
    if len(values) != nx * ny * nz:
        raise HomogenizationError("%s holds %d values for %d x %d x %d voxels" % (raw, len(values), nx, ny, nz))
    return cell_from_values(values, nx, ny, nz), period / np.array([nx, ny, nz], dtype=np.float64)


def cell_from_values(values, nx, ny, nz):
    """The solid mask (nx, ny, nz) from field values in (k * ny + j) * nx + i
    order, <= 0 inside."""
    v = np.asarray(values).reshape(nz, ny, nx)
    return np.ascontiguousarray(np.transpose(v <= 0.0, (2, 1, 0)))


def to_json(result):
    """The result with its arrays as lists, for result.json."""
    out = dict(result)
    out["C"] = [[float(v) for v in row] for row in result["C"]]
    out["critical"] = [[[float(v) for v in row] for row in t] for t in result["critical"]]
    return out
