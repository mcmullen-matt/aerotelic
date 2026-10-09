"""
Generates reference values from the real core/airfoil.py for the C++ port
to check itself against. Run with the aerotelic conda env's python:

    python tools\\generate_airfoil_fixtures.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, r"C:\Users\Matt\devel_1\aerotelic_python")
from core.airfoil import Airfoil, naca4, is_naca4_code

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "airfoil_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_curve_block(f, curve):
    f.write(f"DEGREE {curve.degree}\n")
    f.write(f"PERIODIC {1 if curve.periodic else 0}\n")
    f.write(f"NPOLES {curve.n_poles}\n")
    f.write(f"DIM {curve.poles.shape[1]}\n")
    f.write("POLES\n")
    for row in curve.poles:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("WEIGHTS " + " ".join(fmt(v) for v in curve.weights) + "\n")
    f.write("KNOTS " + " ".join(fmt(v) for v in curve.knots) + "\n")


def write_case(f, name, af, t_values, x_values):
    f.write(f"CASE {name}\n")
    f.write(f"TLE {fmt(af._t_le)}\n")
    write_curve_block(f, af.nurbs_curve)

    upper = af.upper_xy(np.asarray(t_values))
    lower = af.lower_xy(np.asarray(t_values))
    f.write(f"NT {len(t_values)}\n")
    f.write("TVALUES " + " ".join(fmt(v) for v in t_values) + "\n")
    f.write("UPPER\n")
    for row in upper:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("LOWER\n")
    for row in lower:
        f.write(" ".join(fmt(v) for v in row) + "\n")

    camber = af.camber(np.asarray(x_values))
    thickness = af.thickness(np.asarray(x_values))
    f.write(f"NX {len(x_values)}\n")
    f.write("XVALUES " + " ".join(fmt(v) for v in x_values) + "\n")
    f.write("CAMBER " + " ".join(fmt(v) for v in camber) + "\n")
    f.write("THICKNESS " + " ".join(fmt(v) for v in thickness) + "\n")

    mt_x, mt_v = af.max_thickness()
    mc_x, mc_v = af.max_camber()
    f.write(f"MAXT {fmt(mt_x)} {fmt(mt_v)}\n")
    f.write(f"MAXC {fmt(mc_x)} {fmt(mc_v)}\n")

    contour = af.contour_points(200)
    f.write(f"NCONTOUR {contour.shape[0]}\n")
    f.write("CONTOUR\n")
    for row in contour:
        f.write(" ".join(fmt(v) for v in row) + "\n")

    f.write("END\n\n")


def main():
    assert is_naca4_code("2412") and not is_naca4_code("241") and not is_naca4_code("24a2")

    t_values = np.linspace(0.0, 1.0, 41)
    x_values = np.linspace(0.02, 0.98, 37)

    with open(OUT, "w") as f:
        for code in ("2412", "0012", "4415"):
            af = naca4(code)
            write_case(f, f"naca_{code}", af, t_values, x_values)

        # A hand-built contour (ellipse-like, asymmetric camber) through
        # Airfoil.__init__'s 'pts' path, not naca4() -- exercises FitCurve
        # fitting and winding-detection on a non-NACA shape.
        n = 41
        th = np.linspace(0, np.pi, n)
        xc = 0.5 * (1 - np.cos(th))
        thick = 0.08 * np.sin(th)
        camb = 0.03 * np.sin(th) * (1 - xc)
        lower = np.column_stack([xc, camb - thick])[::-1]
        upper = np.column_stack([xc, camb + thick])[1:]
        pts = np.vstack([lower, upper])
        af = Airfoil({"name": "handmade", "pts": pts})
        write_case(f, "handmade", af, t_values, x_values)

        # Round trip through serialize()/from 'curve', to check FromCurve.
        d = af.serialize()
        af2 = Airfoil({"name": "handmade_roundtrip", "curve": d["curve"]})
        write_case(f, "handmade_roundtrip", af2, t_values, x_values)

    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
