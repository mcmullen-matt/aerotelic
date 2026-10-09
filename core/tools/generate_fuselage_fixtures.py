"""
Generates reference values from the real core/fuselage_model.py for the
C++ port to check itself against. Passes an explicit (empty) 'pole_links'
in the custom case so Python takes the modern (non-legacy) nose-station
path -- see fuselage_model.hpp's note on FuselageSpec::legacy.
"""

import os
import sys

sys.path.insert(0, r"C:\Users\Matt\devel_1\aerotelic_python")
from core.fuselage_model import FuselageModel

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "fuselage_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_case(f, name, d):
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.GProp import GProp_GProps

    fm = FuselageModel(d)
    f.write(f"CASE {name}\n")

    xs = [0.0, 0.1, 0.3, 0.5, 0.75, 1.0]
    f.write(f"NX {len(xs)}\n")
    f.write("XVALUES " + " ".join(fmt(x) for x in xs) + "\n")
    f.write("PROFILE_Z\n")
    for x in xs:
        z_top, z_bot = fm.profile_z(x)
        f.write(f"{fmt(z_top)} {fmt(z_bot)}\n")
    f.write("SECTION_PARAMS\n")
    for x in xs:
        f.write(" ".join(fmt(v) for v in fm.section_params(x)) + "\n")

    sec = fm.query_section(0.4, 80)
    f.write(f"QS_SCALARS {fmt(sec['x_pos'])} {fmt(sec['z_top'])} {fmt(sec['z_bot'])} "
            f"{fmt(sec['y_half'])} {fmt(sec['rho_upper'])} {fmt(sec['rho_lower'])} "
            f"{fmt(sec['height'])} {fmt(sec['width'])}\n")
    c3 = sec["contour_3d"]
    f.write(f"QS_NCONTOUR {c3.shape[0]}\n")
    f.write("QS_CONTOUR_3D\n")
    for row in c3:
        f.write(" ".join(fmt(v) for v in row) + "\n")

    m = fm.fuselage_metrics()
    f.write(f"METRICS {fmt(m['length'])} {fmt(m['max_width'])} {fmt(m['max_height'])} {fmt(m['fineness_ratio'])}\n")

    stations = fm.station_fracs(30)
    f.write(f"NSTATIONS {len(stations)}\n")
    f.write("STATIONS " + " ".join(fmt(s) for s in stations) + "\n")

    shape = fm.build_solid()
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    f.write(f"VOLUME {fmt(props.Mass())}\n")
    f.write("END\n\n")


def main():
    with open(OUT, "w") as f:
        write_case(f, "default", {})
        write_case(f, "custom", {
            "length": 12.0,
            "pole_links": {},
            "z_upper_curve": {"degree": 2, "poles": [(0.0, 0.3), (0.1, 1.2), (0.7, 1.1), (1.0, 0.2)]},
            "z_lower_curve": {"degree": 2, "poles": [(0.0, -0.1), (0.1, -0.9), (0.7, -0.8), (1.0, -0.05)]},
            "y_half_curve": {"degree": 2, "poles": [(0.0, 0.15), (0.15, 1.0), (0.7, 0.95), (1.0, 0.1)]},
            "rho_upper_curve": {"degree": 1, "poles": [(0.0, 0.3), (1.0, 0.5)]},
            "rho_lower_curve": {"degree": 1, "poles": [(0.0, 0.45), (1.0, 0.3)]},
        })
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
