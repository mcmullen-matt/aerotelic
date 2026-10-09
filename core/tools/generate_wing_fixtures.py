"""
Generates reference values from the real core/wing_model.py for the C++
port to check itself against. Run with the aerotelic conda env's python.
"""

import os
import sys

import numpy as np

sys.path.insert(0, r"C:\Users\Matt\devel_1\aerotelic_python")
from core.wing_model import make_planform, WingModel
from core.airfoil import naca4

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "wing_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_planform_case(f, name, spec):
    pf = make_planform(spec)
    f.write(f"PLANFORM_CASE {name}\n")
    f.write(f"SEMI_SPAN {fmt(pf.semi_span)}\n")
    f.write(f"ROOT_CHORD {fmt(pf.root_chord)}\n")
    f.write(f"TIP_CHORD {fmt(pf.tip_chord)}\n")
    f.write(f"PLANFORM_AREA {fmt(pf.planform_area)}\n")
    f.write(f"ASPECT_RATIO {fmt(pf.aspect_ratio)}\n")

    s_values = np.linspace(0.001, 0.999, 23)
    coords = np.array([[0.0, 0.0], [1.0, 0.0], [0.25, 0.05], [0.75, -0.03]])
    f.write(f"NS {len(s_values)}\n")
    f.write("SVALUES " + " ".join(fmt(s) for s in s_values) + "\n")
    f.write("XYZ\n")
    for s in s_values:
        xyz = pf.airfoil_coords_to_xyz(float(s), coords)
        for row in xyz:
            f.write(" ".join(fmt(v) for v in row) + "\n")

    outline = pf.planform_outline(50)
    f.write(f"NOUTLINE {outline.shape[0]}\n")
    f.write("OUTLINE\n")
    for row in outline:
        f.write(" ".join(fmt(v) for v in row) + "\n")
    f.write("END\n\n")


def write_loft_stations_case(f, name, spec, sections, n_span=40):
    """sections: list of (s, naca_code)."""
    pf = make_planform(spec)
    wm = WingModel(pf, name="t")
    for s, code in sections:
        wm.add_section(naca4(code), s)
    stations = wm._loft_stations(n_span)
    f.write(f"LOFT_CASE {name}\n")
    f.write(f"NSTATIONS {len(stations)}\n")
    f.write("STATIONS " + " ".join(fmt(s) for s in stations) + "\n")

    # Volume of the resulting solid -- a real OCCT boolean/mass-property
    # cross-check, not just the station list.
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.GProp import GProp_GProps
    shape = wm.build_solid(n_span=n_span)
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    f.write(f"VOLUME {fmt(props.Mass())}\n")
    f.write("END\n\n")


def main():
    with open(OUT, "w") as f:
        write_planform_case(f, "standard", {
            "type": "standard", "semi_span": 8.0, "root_chord": 2.5, "tip_chord": 1.0,
            "sweep_angle": 15.0, "sweep_fraction": 0.25, "rotation_fraction": 0.25,
        })
        write_planform_case(f, "taper", {
            "type": "taper", "semi_span": 6.0, "root_chord": 2.0, "tip_chord": 0.8,
            "sweep_angle": 10.0, "sweep_fraction": 0.0, "rotation_fraction": 0.3,
        })
        write_planform_case(f, "double_trap", {
            "type": "double_trap", "semi_span": 9.0, "root_chord": 3.0, "tip_chord": 1.2,
            "kink_fraction": 0.4, "leading_edge_sweep_angle": 25.0,
            "trailing_edge_sweep_angle": 5.0, "rotation_fraction": 0.25,
        })
        write_planform_case(f, "nurbs", {
            "type": "nurbs", "semi_span": 7.0, "rotation_fraction": 0.25,
            "le_curve": {"degree": 3, "poles": [[0.0, 0.0], [0.5, 0.1], [0.8, 0.3], [1.0, 0.6]]},
            "te_curve": {"degree": 1, "poles": [[0.0, 2.0], [1.0, 1.0]]},
        })

        write_loft_stations_case(f, "trap_linear", {
            "type": "standard", "semi_span": 8.0, "root_chord": 2.5, "tip_chord": 1.0,
            "sweep_angle": 15.0, "sweep_fraction": 0.25, "rotation_fraction": 0.25,
        }, [(0.0, "2412"), (1.0, "0012")])

        write_loft_stations_case(f, "double_trap_kink", {
            "type": "double_trap", "semi_span": 9.0, "root_chord": 3.0, "tip_chord": 1.2,
            "kink_fraction": 0.4, "leading_edge_sweep_angle": 25.0,
            "trailing_edge_sweep_angle": 5.0, "rotation_fraction": 0.25,
        }, [(0.0, "4412"), (0.4, "2412"), (1.0, "0012")])

        write_loft_stations_case(f, "nurbs_curved_tip", {
            "type": "nurbs", "semi_span": 7.0, "rotation_fraction": 0.25,
            "le_curve": {"degree": 3, "poles": [[0.0, 0.0], [0.5, 0.1], [0.8, 0.3], [1.0, 0.6]]},
            "te_curve": {"degree": 1, "poles": [[0.0, 2.0], [1.0, 1.0]]},
        }, [(0.0, "4412"), (1.0, "0012")], n_span=60)

    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
