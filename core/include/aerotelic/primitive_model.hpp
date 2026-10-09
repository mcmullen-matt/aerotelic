#pragma once

// Port of aerotelic/core/primitive_model.py. Canonical solids -- a sphere
// (or ellipsoid), a box, a cylinder -- sized by a few numbers. See the
// Python module's docstring for the full rationale (payload-bay /
// fill-exclusion volumes, centred on the origin, axis conventions).

#include <array>
#include <optional>
#include <string>

#include <TopoDS_Shape.hxx>

#include "aerotelic/solid_model.hpp"

namespace aerotelic {

enum class PrimitiveShape { Sphere, Box, Cylinder };

class PrimitiveModel : public ISolidModel {
public:
	PrimitiveModel() = default;

	PrimitiveShape shape = PrimitiveShape::Sphere;
	std::string name = "Primitive";
	double radius = 1.0;
	std::array<double, 3> stretch = {1.0, 1.0, 1.0};
	std::array<double, 3> size = {2.0, 2.0, 2.0};
	double cyl_radius = 1.0;
	double cyl_length = 2.0;

	void SetShape(PrimitiveShape s) { shape = s; }

	std::array<double, 3> SemiAxes() const; // radius * stretch
	bool IsTrueSphere() const;
	std::array<double, 3> Extents() const; // axis-aligned box, centred on origin
	double Volume() const;
	double SurfaceArea() const;

	// Throws std::invalid_argument, in words a user can act on, for a
	// shape that cannot be built.
	void Validate() const;

	// Builds and returns the primitive as an OCC solid, centred on the origin.
	TopoDS_Shape BuildSolid() const override;
};

PrimitiveShape ParseShape(const std::string& s);
const char* ShapeName(PrimitiveShape s);

} // namespace aerotelic
