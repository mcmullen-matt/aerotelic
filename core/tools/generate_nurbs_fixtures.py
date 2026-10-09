"""
Generates reference values from the real core/nurbs.py for the C++ port
(nurbs_fixture_check.cpp) to check itself against. Run with the aerotelic
conda env's python, from this repo:

    C:\\Users\\Matt\\micromamba\\envs\\aerotelic\\python.exe tools\\generate_nurbs_fixtures.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, r"C:\Users\Matt\devel_1\WingDesigner")
from core.nurbs import ClampedNURBSCurve, PeriodicNURBSCurve, FitCurve, DihedralCurve

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "nurbs_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_case(f, name, curve, s_values):
    evals = curve.evaluate(np.asarray(s_values))
    derivs = curve.derivative(np.asarray(s_values))
    f.write(f"CASE {name}\n")
    f.write(f"DEGREE {curve.degree}\n")
    f.write(f"PERIODIC {1 if curve.periodic else 0}\n")
    f.write(f"CLAMPED_SEAM {1 if getattr(curve, 'clamped_seam', False) else 0}\n")
    f.write(f"NPOLES {curve.n_poles}\n")
    f.write(f"DIM {curve.poles.shape[1]}\n")
    f.write("POLES\n")
    for row in curve.poles:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("WEIGHTS " + " ".join(fmt(v) for v in curve.weights) + "\n")
    f.write("KNOTS " + " ".join(fmt(v) for v in curve.knots) + "\n")
    f.write(f"NSAMPLES {len(s_values)}\n")
    f.write("SAMPLES " + " ".join(fmt(s) for s in s_values) + "\n")
    f.write("EVAL\n")
    for row in evals:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("DERIV\n")
    for row in derivs:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("END\n\n")


def main():
    cases = []

    # Clamped, rational, degrees 1-3.
    P = np.array([[0.0, 0.0], [1.0, 2.0], [2.5, -1.0], [4.0, 1.5], [5.5, 0.0], [7.0, 2.0], [8.0, -0.5]])
    w = np.array([1.0, 1.3, 0.8, 1.1, 1.4, 0.9, 1.2])
    for p in (1, 2, 3):
        n_int = P.shape[0] - p - 1
        spec = {"degree": p, "poles": P, "weights": w}
        if n_int > 0:
            spec["interior_knots"] = np.linspace(0, 1, n_int + 2)[1:-1]
        c = ClampedNURBSCurve(spec)
        s = np.linspace(0.001, 0.999, 51)
        cases.append((f"clamped_p{p}", c, s))

    # Periodic, rational, degrees 1-3.
    n = 8
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    Pp = np.column_stack([np.cos(ang), np.sin(ang)]) * (1 + 0.15 * np.sin(3 * ang))[:, None]
    wp = 1.0 + 0.3 * np.cos(2 * ang)
    for p in (1, 2, 3):
        c = PeriodicNURBSCurve({"degree": p, "poles": Pp, "weights": wp, "periodic": True})
        s = np.linspace(0.0, 0.999, 61)
        cases.append((f"periodic_p{p}", c, s))

    # Periodic with a clamped (p-fold) seam.
    c = PeriodicNURBSCurve({"degree": 3, "poles": Pp, "weights": wp,
                             "periodic": True, "clamped_seam": True})
    cases.append(("periodic_clamped_seam_p3", c, np.linspace(0.0, 0.999, 61)))

    with open(OUT, "w") as f:
        for name, curve, s in cases:
            write_case(f, name, curve, s)

        # FitCurve: interpolation property + a cross-check sample grid.
        t = np.linspace(0, np.pi, 25)
        pts = np.column_stack([np.cos(t), np.sin(t)])
        for p in (1, 2, 3):
            fit = FitCurve(pts, p)
            f.write(f"CASE fitcurve_p{p}\n")
            f.write(f"DEGREE {p}\n")
            f.write(f"NPTS {pts.shape[0]}\n")
            f.write(f"DIM {pts.shape[1]}\n")
            f.write("PTS\n")
            for row in pts:
                f.write(" ".join(fmt(v) for v in row) + "\n")
            f.write("PARAMS " + " ".join(fmt(v) for v in fit.collocation_parameters) + "\n")
            f.write("POLES\n")
            for row in fit.poles:
                f.write(" ".join(fmt(v) for v in row) + "\n")
            f.write("INTERIOR_KNOTS " + " ".join(fmt(v) for v in fit.interior_knots) + "\n")
            f.write("END\n\n")

        # DihedralCurve: chord-length interior knot placement.
        Pd = np.array([[0.0, 0.0], [2.0, 0.1], [4.5, 0.5], [6.0, 1.3], [8.0, 1.4]])
        d = DihedralCurve({"degree": 3, "poles": Pd})
        f.write("CASE dihedral_p3\n")
        f.write(f"NPOLES {d.n_poles}\n")
        f.write(f"DIM {d.poles.shape[1]}\n")
        f.write("POLES\n")
        for row in d.poles:
            f.write(" ".join(fmt(v) for v in row) + "\n")
        f.write("INTERIOR_KNOTS " + " ".join(fmt(v) for v in d.interior_knots) + "\n")
        f.write("END\n\n")

    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
