"""Self-check of homogenize.py against closed forms and the lattice types.

    python homogenize_check.py [--voxels N] [--tool PATH]

Cells built here by hand, with exact or classical answers:

  * a SOLID cell returns the base material's tensor to rounding (the
    corrected fields are the uniform strains, which a trilinear hexahedron
    reproduces exactly);
  * a SQUARE HONEYCOMB (walls on the x = 0 and y = 0 planes, prisms along
    z): along the prisms the Voigt bound is attained exactly, E_z = rho* E,
    nu_zx = nu_zy = nu, since uniform axial strain with free lateral
    contraction is compatible in every wall; across them the walls along
    x carry E_x ~ (t / l) E, half the material;
  * a GRID (three orthogonal square bars through the cell centre): the bar
    along x carries E_x ~ (a / l)^2 E, and the cell has cubic symmetry, so
    E_x = E_y = E_z and the tensor has the three cubic constants only;
  * every cell: C is symmetric and positive definite, below the Voigt
    bound rho* D in its diagonal, the amplification is >= 1 and the
    critical set is not empty.

With the application's own cells (core/build/Release/
aerotelic_lattice_cell.exe, built by core/CMakeLists.txt; --tool to point
elsewhere), the real lattice types too:

  * a regular HEXAGONAL honeycomb along z: E_z = rho* E exactly (prismatic),
    and in plane Gibson & Ashby's E_1 / E = 2.3 (t / l)^3 for thin walls,
    within the factor a few-voxel wall allows;
  * the OCTET truss is stretch-dominated, E / E ~ rho* / 9 (Deshpande,
    Fleck & Ashby 2001) within a factor of two at rho* = 0.2;
  * the BCC lattice is bending-dominated, so much softer than its
    relative density: E / E_s well under rho* / 2;
  * the GYROID network sits between the bending and stretching bounds,
    and is cubic.
"""

import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import homogenize as hz  # noqa: E402

import numpy as np  # noqa: E402

E0, NU0 = 70.0e9, 0.33


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    return bool(cond)


def default_tool():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "core", "build", "Release", "aerotelic_lattice_cell.exe"))


def common_checks(name, r, D):
    f = 0
    C = r["C"]
    f += not check(np.allclose(C, C.T, rtol=0, atol=1e-6 * abs(C).max()), "%s: C is symmetric" % name)
    f += not check(np.linalg.eigvalsh(C).min() > 0.0, "%s: C is positive definite" % name)
    rho = r["relative_density"]
    f += not check((np.diag(C) <= rho * np.diag(D) * (1.0 + 1e-9)).all(), "%s: the diagonal is under the Voigt bound rho* D" % name)
    f += not check(r["amplification"] >= 1.0 - 1e-9 and np.isfinite(r["amplification"]), "%s: the stress amplification is at least 1 (%.3f)" % (name, r["amplification"]))
    f += not check(len(r["critical"]) > 0, "%s: a critical voxel set came out (%d voxels)" % (name, len(r["critical"])))
    return f


def cubic(C, tol=1e-3):
    """True when C has cubic symmetry: equal normal stiffnesses, equal
    couplings, equal shears, no normal-shear coupling."""
    s = abs(C).max()
    d = np.diag(C)
    ok = abs(d[0] - d[1]) < tol * s and abs(d[1] - d[2]) < tol * s
    ok = ok and abs(C[0, 1] - C[0, 2]) < tol * s and abs(C[0, 2] - C[1, 2]) < tol * s
    ok = ok and abs(d[3] - d[4]) < tol * s and abs(d[4] - d[5]) < tol * s
    ok = ok and abs(C[:3, 3:]).max() < tol * s and abs(C[3:, 3:] - np.diag(d[3:])).max() < tol * s
    return ok


def main(argv):
    voxels, tool = 24, default_tool()
    args = list(argv[1:])
    while args:
        a = args.pop(0)
        if a == "--voxels":
            voxels = int(args.pop(0))
        elif a == "--tool":
            tool = args.pop(0)
    failures = 0
    D = hz.isotropic_tensor(E0, NU0)
    h = (1.0 / voxels,) * 3

    # ---- solid ----
    solid = np.ones((voxels, voxels, voxels), dtype=bool)
    r = hz.homogenize(solid, h, E0, NU0, samples=200)
    failures += not check(np.allclose(r["C"], D, rtol=1e-9, atol=1e-9 * E0), "solid: C is the base tensor to rounding")
    failures += not check(abs(r["amplification"] - 1.0) < 1e-6 or r["amplification"] >= 1.0, "solid: amplification %.4f" % r["amplification"])

    # ---- square honeycomb along z ----
    n = voxels
    t_vox = max(2, n // 8)                    # wall thickness in voxels
    mask = np.zeros((n, n, n), dtype=bool)
    mask[:t_vox, :, :] = True                 # the wall on x = 0
    mask[:, :t_vox, :] = True                 # the wall on y = 0
    rho = mask.mean()
    r = hz.homogenize(mask, h, E0, NU0, samples=500)
    failures += common_checks("square honeycomb", r, D)
    eng = r["engineering"]
    failures += not check(abs(eng["E"][2] / (rho * E0) - 1.0) < 1e-6, "square honeycomb: E_z = rho* E exactly (%.6f of it)" % (eng["E"][2] / (rho * E0)))
    failures += not check(abs(eng["nu_zx"] - NU0) < 1e-6 and abs(eng["nu_zy"] - NU0) < 1e-6, "square honeycomb: nu_zx = nu_zy = nu (%.4f, %.4f)" % (eng["nu_zx"], eng["nu_zy"]))
    t_over_l = t_vox / float(n)
    failures += not check(0.8 < eng["E"][0] / (t_over_l * E0) < 1.3, "square honeycomb: E_x ~ (t / l) E (%.3f of it)" % (eng["E"][0] / (t_over_l * E0)))
    failures += not check(abs(eng["E"][0] - eng["E"][1]) < 1e-6 * eng["E"][0], "square honeycomb: E_x = E_y by symmetry")
    failures += not check(eng["E"][0] < eng["E"][2], "square honeycomb: softer across the prisms than along them")
    print("  square honeycomb rho* %.3f: E/E = (%.4f, %.4f, %.4f), G/E = (%.4f, %.4f, %.4f), amplification %.2f, %d dof in %.1f s"
          % (rho, eng["E"][0] / E0, eng["E"][1] / E0, eng["E"][2] / E0, eng["G"][0] / E0, eng["G"][1] / E0, eng["G"][2] / E0, r["amplification"], r["dofs"], r["seconds"]))

    # ---- grid: three orthogonal square bars ----
    a_vox = max(2, n // 4)
    lo, hi = n // 2 - a_vox // 2, n // 2 - a_vox // 2 + a_vox
    mask = np.zeros((n, n, n), dtype=bool)
    mask[:, lo:hi, lo:hi] = True
    mask[lo:hi, :, lo:hi] = True
    mask[lo:hi, lo:hi, :] = True
    rho = mask.mean()
    r = hz.homogenize(mask, h, E0, NU0, samples=500)
    failures += common_checks("grid", r, D)
    eng = r["engineering"]
    a_over_l = a_vox / float(n)
    failures += not check(0.85 < eng["E"][0] / (a_over_l ** 2 * E0) < 1.4, "grid: E_x ~ (a / l)^2 E (%.3f of it)" % (eng["E"][0] / (a_over_l ** 2 * E0)))
    failures += not check(cubic(r["C"]), "grid: the tensor has cubic symmetry")
    print("  grid rho* %.3f: E/E = %.4f, G/E = %.4f, nu_xy = %.3f, amplification %.2f, %d dof in %.1f s"
          % (rho, eng["E"][0] / E0, eng["G"][0] / E0, eng["nu_xy"], r["amplification"], r["dofs"], r["seconds"]))
    # The critical set estimates the micro stress of a macro strain state
    # no worse than the amplification bound would.
    eps = np.random.default_rng(3).standard_normal((50, 6)) * 1e-3
    macro = eps @ r["C"].T
    micro = hz.micro_von_mises(r["critical"], eps)
    failures += not check((micro <= r["amplification"] * hz.frobenius(macro) * (1.0 + 1e-9)).all(), "grid: the critical-voxel micro stress is within the amplification bound")
    failures += not check((micro >= hz.von_mises(macro) * 0.99).all(), "grid: the micro stress is at least the macro von Mises")

    # ---- the application's lattice types ----
    if not os.path.exists(tool):
        print("skipped: %s is not built, so the real lattice types are not checked" % tool)
    else:
        env = dict(os.environ)
        occ = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(sys.executable))), "aerotelic", "Library", "bin")
        if os.path.isdir(occ):
            env["PATH"] = occ + os.pathsep + env.get("PATH", "")
        tmp = tempfile.mkdtemp(prefix="aerotelic_hz_")

        def sample(name, *opts):
            out = os.path.join(tmp, name)
            p = subprocess.run([tool, out] + list(opts), capture_output=True, text=True, env=env)
            if p.returncode != 0:
                raise RuntimeError("%s failed: %s %s" % (tool, p.stdout, p.stderr))
            return hz.read_cell(out + ".json")

        # Regular hexagonal honeycomb along z at rho* 0.1: thin walls.
        solid, hc = sample("hex", "--family", "honeycomb", "--type", "Hexagon", "--axis", "z", "--fraction", "0.1", "--voxels", "48")
        rho = solid.mean()
        r = hz.homogenize(solid, hc, E0, NU0, samples=500)
        failures += common_checks("hexagon", r, D)
        eng = r["engineering"]
        failures += not check(abs(eng["E"][2] / (rho * E0) - 1.0) < 1e-6, "hexagon: E_z = rho* E exactly (%.6f of it)" % (eng["E"][2] / (rho * E0)))
        # Gibson & Ashby: rho* = (2 / sqrt(3)) (t / l), E_1 / E_s = 2.3 (t / l)^3.
        t_over_l = rho * np.sqrt(3.0) / 2.0
        ga = 2.3 * t_over_l ** 3 * E0
        failures += not check(0.6 < eng["E"][0] / ga < 3.0 and 0.6 < eng["E"][1] / ga < 3.0,
                              "hexagon: in-plane E is Gibson & Ashby's 2.3 (t/l)^3 E within the voxel wall's factor (%.2f, %.2f of it)" % (eng["E"][0] / ga, eng["E"][1] / ga))
        failures += not check(abs(eng["E"][0] - eng["E"][1]) < 0.05 * eng["E"][0], "hexagon: transversely isotropic in plane (E_x %.3g, E_y %.3g)" % (eng["E"][0], eng["E"][1]))
        print("  hexagon rho* %.3f: E/E = (%.5f, %.5f, %.5f), amplification %.1f, %d dof in %.1f s"
              % (rho, eng["E"][0] / E0, eng["E"][1] / E0, eng["E"][2] / E0, r["amplification"], r["dofs"], r["seconds"]))

        # Octet at 0.2: stretch-dominated, ~rho*/9 (pin-jointed).
        solid, hc = sample("octet", "--family", "strut", "--type", "Octet", "--fraction", "0.2", "--voxels", str(max(voxels, 32)))
        rho = solid.mean()
        r = hz.homogenize(solid, hc, E0, NU0, samples=500)
        failures += common_checks("octet", r, D)
        eng = r["engineering"]
        failures += not check(0.5 < eng["E"][0] / (rho / 9.0 * E0) < 2.5, "octet: E ~ rho* E / 9 within a factor of two (%.2f of it)" % (eng["E"][0] / (rho / 9.0 * E0)))
        failures += not check(cubic(r["C"], 0.02), "octet: cubic")
        print("  octet rho* %.3f: E/E = %.4f, G/E = %.4f, amplification %.1f, %d dof in %.1f s"
              % (rho, eng["E"][0] / E0, eng["G"][0] / E0, r["amplification"], r["dofs"], r["seconds"]))
        octet_E = eng["E"][0]

        # BCC at 0.2: bending-dominated, much softer than its density.
        solid, hc = sample("bcc", "--family", "strut", "--type", "BCC", "--fraction", "0.2", "--voxels", str(max(voxels, 32)))
        rho = solid.mean()
        r = hz.homogenize(solid, hc, E0, NU0, samples=500)
        failures += common_checks("bcc", r, D)
        eng = r["engineering"]
        failures += not check(eng["E"][0] < 0.5 * rho * E0, "bcc: bending-dominated, E well under rho* E / 2 (%.3f of rho* E)" % (eng["E"][0] / (rho * E0)))
        failures += not check(eng["E"][0] < octet_E, "bcc: softer than the octet at the same density")
        print("  bcc rho* %.3f: E/E = %.4f, G/E = %.4f, amplification %.1f" % (rho, eng["E"][0] / E0, eng["G"][0] / E0, r["amplification"]))

        # Gyroid network at 0.3.
        solid, hc = sample("gyroid", "--family", "tpms", "--type", "Gyroid", "--phase", "network", "--fraction", "0.3", "--voxels", str(max(voxels, 32)))
        rho = solid.mean()
        r = hz.homogenize(solid, hc, E0, NU0, samples=500)
        failures += common_checks("gyroid", r, D)
        eng = r["engineering"]
        failures += not check(0.05 * rho < eng["E"][0] / E0 < rho, "gyroid: E between the bending and stretching regimes (%.3f of rho* E)" % (eng["E"][0] / (rho * E0)))
        failures += not check(cubic(r["C"], 0.03), "gyroid: cubic")
        failures += not check(r["dropped_fraction"] < 0.01, "gyroid: the network is connected (%.4f dropped)" % r["dropped_fraction"])
        print("  gyroid rho* %.3f: E/E = %.4f, G/E = %.4f, nu = %.3f, amplification %.1f, %d dof in %.1f s"
              % (rho, eng["E"][0] / E0, eng["G"][0] / E0, eng["nu_xy"], r["amplification"], r["dofs"], r["seconds"]))

        # JSON round trip of the result.
        js = json.dumps(hz.to_json(r))
        failures += not check(len(js) > 0 and np.allclose(np.array(json.loads(js)["C"]), r["C"]), "the result serialises to JSON")

    print("RESULT: %d failure(s)" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
