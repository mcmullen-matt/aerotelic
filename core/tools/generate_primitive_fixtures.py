"""
Generates reference values from the real core/primitive_model.py for the
C++ port to check itself against (volume/surface_area/extents only --
build_solid() is OCCT-only and is instead smoke-tested directly in C++
against Extents(), since both sides already share the same OCCT kernel).
"""

import os
import sys

sys.path.insert(0, r"C:\Users\Matt\devel_1\aerotelic_python")
from core.primitive_model import PrimitiveModel

OUT = os.path.join(os.path.dirname(__file__), "..", "tests", "primitive_fixtures.txt")


def fmt(x):
    return repr(float(x))


def write_case(f, name, d):
    p = PrimitiveModel(d)
    f.write(f"CASE {name}\n")
    f.write(f"SHAPE {p.shape}\n")
    f.write(f"RADIUS {fmt(p.radius)}\n")
    f.write("STRETCH " + " ".join(fmt(v) for v in p.stretch) + "\n")
    f.write("SIZE " + " ".join(fmt(v) for v in p.size) + "\n")
    f.write(f"CYL_RADIUS {fmt(p.cyl_radius)}\n")
    f.write(f"CYL_LENGTH {fmt(p.cyl_length)}\n")
    f.write(f"VOLUME {fmt(p.volume())}\n")
    f.write(f"SURFACE_AREA {fmt(p.surface_area())}\n")
    f.write("EXTENTS " + " ".join(fmt(v) for v in p.extents()) + "\n")
    f.write("END\n\n")


def main():
    with open(OUT, "w") as f:
        write_case(f, "true_sphere", {"shape": "sphere", "radius": 2.5})
        write_case(f, "ellipsoid", {"shape": "sphere", "radius": 1.5, "stretch": [1.2, 0.8, 1.6]})
        write_case(f, "cube", {"shape": "box", "size": [2.0, 2.0, 2.0]})
        write_case(f, "box", {"shape": "box", "size": [3.0, 1.5, 0.7]})
        write_case(f, "cylinder", {"shape": "cylinder", "cyl_radius": 0.8, "cyl_length": 3.2})
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
