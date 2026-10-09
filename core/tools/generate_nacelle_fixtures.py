"""
Generates reference values from the real core/nacelle_model.py for the
C++ port to check itself against.
"""

import os
import sys

sys.path.insert(0, r"C:\Users\Matt\devel_1\WingDesigner")
from core.nacelle_model import NacelleModel

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "nacelle_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_case(f, name, d):
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.GProp import GProp_GProps

    nm = NacelleModel(d)
    f.write(f"CASE {name}\n")
    f.write(f"LIP_RADIUS {fmt(nm.lip_radius)}\n")
    f.write(f"TE_RADIUS {fmt(nm.te_radius)}\n")
    f.write(f"RING_PERIMETER {fmt(nm.ring_perimeter())}\n")
    f.write(f"TE_PERIMETER {fmt(nm.te_perimeter())}\n")
    f.write(f"TOOTH_WIDTH {fmt(nm.tooth_width())}\n")
    f.write(f"CHEVRON_TIP_ANGLE {fmt(nm.chevron_tip_angle())}\n")
    m = nm.nacelle_metrics()
    f.write(f"METRICS {fmt(m['length'])} {fmt(m['lip_diameter'])} {fmt(m['max_diameter'])} "
            f"{fmt(m['te_diameter'])} {fmt(m['chevron_depth'])}\n")

    shape = nm.build_solid()
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    f.write(f"VOLUME {fmt(props.Mass())}\n")
    f.write("END\n\n")


def main():
    with open(OUT, "w") as f:
        write_case(f, "default", {})
        write_case(f, "no_chevrons_custom", {
            "length": 3.2,
            "upper_curve": {"degree": 3, "poles": [(0.0, 1.0), (0.01, 1.2), (0.3, 1.35), (0.75, 1.2), (1.0, 0.9)]},
            "lower_curve": {"degree": 3, "poles": [(0.0, 1.0), (0.01, 0.85), (0.25, 0.82), (0.75, 0.88), (1.0, 0.9)]},
            "chevron_count": 0,
        })
        write_case(f, "chevrons", {
            "length": 2.5,
            "upper_curve": {"degree": 3, "poles": [(0.0, 1.0), (0.01, 1.15), (0.3, 1.25), (0.75, 1.15), (1.0, 0.95)]},
            "lower_curve": {"degree": 3, "poles": [(0.0, 1.0), (0.01, 0.9), (0.25, 0.88), (0.75, 0.9), (1.0, 0.95)]},
            "chevron_count": 6,
            "chevron_depth": 0.15,
        })
        write_case(f, "chevron_angle_legacy", {
            "length": 2.0,
            "chevron_count": 8,
            "chevron_angle": 40.0,
        })
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
