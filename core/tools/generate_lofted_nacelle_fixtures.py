"""
Generates reference values from the real core/lofted_nacelle_model.py for
the C++ port to check itself against.
"""

import os
import sys

import numpy as np

sys.path.insert(0, r"C:\Users\Matt\devel_1\WingDesigner")
from core.lofted_nacelle_model import LoftedNacelleModel, default_section_curve

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "lofted_nacelle_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_case(f, name, d):
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.GProp import GProp_GProps

    nm = LoftedNacelleModel(d)
    f.write(f"CASE {name}\n")
    f.write(f"TE_PERIMETER {fmt(nm.te_perimeter())}\n")
    f.write(f"CHEVRON_TIP_ANGLE {fmt(nm.chevron_tip_angle())}\n")
    m = nm.body_metrics()
    f.write(f"METRICS {fmt(m['length'])} {fmt(m['lip_diameter'])} {fmt(m['max_diameter'])} "
            f"{fmt(m['te_diameter'])} {fmt(m['max_thickness'])} {m['n_sections']} {fmt(m['chevron_depth'])}\n")

    xs = [0.0, 0.25, 0.5, 0.75, 1.0]
    f.write(f"NX {len(xs)}\n")
    f.write("XVALUES " + " ".join(fmt(x) for x in xs) + "\n")
    f.write("MEAN_RADIUS " + " ".join(fmt(nm.mean_radius(x)) for x in xs) + "\n")

    stations = nm.station_fracs(30)
    f.write(f"NSTATIONS {len(stations)}\n")
    f.write("STATIONS " + " ".join(fmt(s) for s in stations) + "\n")

    shape = nm.build_solid()
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    f.write(f"VOLUME {fmt(props.Mass())}\n")
    f.write("END\n\n")


def main():
    with open(OUT, "w") as f:
        write_case(f, "default", {})

        write_case(f, "multi_guide", {
            "length": 3.5,
            "centreline_curve": {"degree": 2, "poles": [(0.0, 0.0), (0.5, 0.05), (1.0, 0.0)]},
            "sections": [
                {"station": 0.0, "curve": default_section_curve(1.0)},
                {"station": 0.5, "curve": default_section_curve(1.3)},
                {"station": 1.0, "curve": default_section_curve(0.8)},
            ],
            "outer_curve": {"degree": 3, "poles": [(0.0, 0.0), (0.05, 0.2), (0.6, 0.18), (1.0, 0.0)]},
            "inner_curve": {"degree": 3, "poles": [(0.0, 0.0), (0.05, -0.12), (0.6, -0.1), (1.0, 0.0)]},
        })

        write_case(f, "chevrons", {
            "length": 2.8,
            "sections": [{"station": 0.0, "curve": default_section_curve(1.0)}],
            "chevron_count": 6,
            "chevron_depth": 0.2,
        })

    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
