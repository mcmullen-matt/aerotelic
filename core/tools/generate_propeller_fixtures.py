"""
Generates reference values from the real core/propeller_model.py for the
C++ port to check itself against. Uses the default blade shape throughout
(radius/blade_count/blade_x/hub_radius/spinner_length/spinner_curve only),
matching this port's scope cut (see propeller_model.hpp's file header).
"""

import os
import sys

sys.path.insert(0, r"C:\Users\Matt\devel_1\WingDesigner")
from core.propeller_model import PropellerModel

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "propeller_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_case(f, name, d, set_radius=None):
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.GProp import GProp_GProps

    pm = PropellerModel(d)
    if set_radius is not None:
        pm.set_radius(set_radius)

    f.write(f"CASE {name}\n")
    f.write(f"SPINNER_RADIUS {fmt(pm.spinner_radius())}\n")

    xs = [0.1, 0.3, 0.6, 0.9]
    f.write(f"NX {len(xs)}\n")
    f.write("XVALUES " + " ".join(fmt(x) for x in xs) + "\n")
    f.write("SPINNER_RADIUS_AT " + " ".join(fmt(pm.spinner_radius_at(x)) for x in xs) + "\n")

    etas = [0.0, 0.25, 0.5, 0.75, 1.0]
    f.write(f"NETA {len(etas)}\n")
    f.write("ETAVALUES " + " ".join(fmt(e) for e in etas) + "\n")
    f.write("CHORD_OVER_R " + " ".join(fmt(c) for c in pm.chord_over_r(etas)) + "\n")
    f.write("BLADE_ANGLE " + " ".join(fmt(pm.blade_angle(e)) for e in etas) + "\n")

    m = pm.propeller_metrics()
    f.write(f"METRICS {fmt(m['diameter'])} {m['blade_count']} {fmt(m['solidity'])} "
            f"{fmt(m['activity_factor'])} {fmt(m['beta_75'])} {fmt(m['pitch_ratio'])} "
            f"{fmt(m['spinner_diameter'])} {fmt(m['spinner_length'])}\n")

    shape = pm.build_solid()
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    f.write(f"VOLUME {fmt(props.Mass())}\n")
    f.write("END\n\n")


def main():
    with open(OUT, "w") as f:
        write_case(f, "default", {})
        write_case(f, "three_blade", {"blade_count": 3, "radius": 1.1, "blade_x": 0.3, "hub_radius": 0.15})
        write_case(f, "rescaled", {}, set_radius=1.25)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
