#include "aerotelic/primitive_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_GTrsf.hxx>
#include <gp_Pnt.hxx>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Numerical Recipes' "gauleg": Newton's method on the Legendre polynomial
// roots, which is what np.polynomial.legendre.leggauss also computes (by a
// different route, an eigenvalue problem) -- the two need not agree bit for
// bit, only converge to the same integral, which a 64-point rule does to
// far more precision than this ever needs (see the Python module's comment).
void GaussLegendre(int n, std::vector<double>& x, std::vector<double>& w) {
	x.assign(n, 0.0);
	w.assign(n, 0.0);
	int m = (n + 1) / 2;
	for (int i = 0; i < m; ++i) {
		double z = std::cos(kPi * (i + 0.75) / (n + 0.5));
		double z1, pp;
		do {
			double p1 = 1.0, p2 = 0.0;
			for (int j = 0; j < n; ++j) {
				double p3 = p2;
				p2 = p1;
				p1 = ((2.0 * j + 1.0) * z * p2 - j * p3) / (j + 1.0);
			}
			pp = n * (z * p1 - p2) / (z * z - 1.0);
			z1 = z;
			z = z1 - p1 / pp;
		} while (std::abs(z - z1) > 1e-14);
		x[i] = -z;
		x[n - 1 - i] = z;
		w[i] = 2.0 / ((1.0 - z * z) * pp * pp);
		w[n - 1 - i] = w[i];
	}
}

double EllipsoidArea(double a, double b, double c, int n = 64) {
	std::vector<double> z, wz;
	GaussLegendre(n, z, wz);
	double total = 0.0;
	for (int i = 0; i < n; ++i) {
		double s = std::sqrt(std::max(0.0, 1.0 - z[i] * z[i]));
		double az = a * b * z[i];
		for (int j = 0; j < 2 * n; ++j) {
			double phi = (j + 0.5) * (kPi / n);
			double x = s * std::cos(phi), y = s * std::sin(phi);
			double bcx = b * c * x, acy = a * c * y;
			total += wz[i] * std::sqrt(bcx * bcx + acy * acy + az * az);
		}
	}
	return total * (kPi / n);
}

} // namespace

PrimitiveShape ParseShape(const std::string& s) {
	if (s == "sphere") return PrimitiveShape::Sphere;
	if (s == "box") return PrimitiveShape::Box;
	if (s == "cylinder") return PrimitiveShape::Cylinder;
	throw std::invalid_argument("unknown primitive shape '" + s + "' (expected one of sphere, box, cylinder)");
}

const char* ShapeName(PrimitiveShape s) {
	switch (s) {
		case PrimitiveShape::Sphere: return "sphere";
		case PrimitiveShape::Box: return "box";
		case PrimitiveShape::Cylinder: return "cylinder";
	}
	return "";
}

std::array<double, 3> PrimitiveModel::SemiAxes() const {
	return {radius * stretch[0], radius * stretch[1], radius * stretch[2]};
}

bool PrimitiveModel::IsTrueSphere() const {
	return stretch[0] == 1.0 && stretch[1] == 1.0 && stretch[2] == 1.0;
}

std::array<double, 3> PrimitiveModel::Extents() const {
	if (shape == PrimitiveShape::Sphere) {
		auto a = SemiAxes();
		return {2.0 * a[0], 2.0 * a[1], 2.0 * a[2]};
	}
	if (shape == PrimitiveShape::Cylinder) {
		double d = 2.0 * cyl_radius;
		return {cyl_length, d, d};
	}
	return size;
}

double PrimitiveModel::Volume() const {
	if (shape == PrimitiveShape::Sphere) {
		auto a = SemiAxes();
		return 4.0 / 3.0 * kPi * a[0] * a[1] * a[2];
	}
	if (shape == PrimitiveShape::Cylinder) {
		return kPi * cyl_radius * cyl_radius * cyl_length;
	}
	return size[0] * size[1] * size[2];
}

double PrimitiveModel::SurfaceArea() const {
	if (shape == PrimitiveShape::Sphere) {
		if (IsTrueSphere()) return 4.0 * kPi * radius * radius;
		auto a = SemiAxes();
		return EllipsoidArea(a[0], a[1], a[2]);
	}
	if (shape == PrimitiveShape::Cylinder) {
		double r = cyl_radius, l = cyl_length;
		return 2.0 * kPi * r * (r + l);
	}
	double lx = size[0], ly = size[1], lz = size[2];
	return 2.0 * (lx * ly + ly * lz + lz * lx);
}

void PrimitiveModel::Validate() const {
	if (shape == PrimitiveShape::Sphere) {
		if (!(radius > 0.0)) throw std::invalid_argument("the sphere's radius must be greater than zero");
		const char* axes = "XYZ";
		for (int i = 0; i < 3; ++i) {
			if (!(stretch[i] > 0.0)) {
				throw std::invalid_argument(std::string("the sphere's stretch along ") + axes[i] + " must be greater than zero");
			}
		}
		return;
	}
	if (shape == PrimitiveShape::Cylinder) {
		if (!(cyl_radius > 0.0)) throw std::invalid_argument("the cylinder's radius must be greater than zero");
		if (!(cyl_length > 0.0)) throw std::invalid_argument("the cylinder's length must be greater than zero");
		return;
	}
	const char* axes = "XYZ";
	for (int i = 0; i < 3; ++i) {
		if (!(size[i] > 0.0)) {
			throw std::invalid_argument(std::string("the box's size along ") + axes[i] + " must be greater than zero");
		}
	}
}

std::vector<NamedMesh> PrimitiveModel::BuildMeshes(double tolerance) const {
	Validate();
	if (shape == PrimitiveShape::Sphere) {
		auto a = SemiAxes();
		return {{"Main", EllipsoidMesh(a[0], a[1], a[2], tolerance)}};
	}
	if (shape == PrimitiveShape::Cylinder) return {{"Main", CylinderMesh(cyl_radius, cyl_length, tolerance)}};
	return {{"Main", BoxMesh(size[0], size[1], size[2])}};
}

TopoDS_Shape PrimitiveModel::BuildSolid() const {
	Validate();
	if (shape == PrimitiveShape::Sphere) {
		BRepPrimAPI_MakeSphere maker(gp_Pnt(0.0, 0.0, 0.0), radius);
		if (!IsTrueSphere()) {
			// An ellipsoid is the sphere stretched along each axis. That is not
			// a rigid motion, so it goes through GTransform, which re-expresses
			// the surface (as a NURBS) rather than moving it.
			maker.Build();
			if (!maker.IsDone()) throw std::runtime_error("could not build the sphere");
			gp_GTrsf g;
			for (int i = 1; i <= 3; ++i) g.SetValue(i, i, stretch[i - 1]);
			BRepBuilderAPI_GTransform stretched(maker.Solid(), g, true);
			if (!stretched.IsDone()) throw std::runtime_error("could not stretch the sphere into an ellipsoid");
			return stretched.Shape();
		}
		maker.Build();
		if (!maker.IsDone()) throw std::runtime_error("could not build the sphere");
		return maker.Solid();
	}
	if (shape == PrimitiveShape::Cylinder) {
		double r = cyl_radius, l = cyl_length;
		BRepPrimAPI_MakeCylinder maker(gp_Ax2(gp_Pnt(-0.5 * l, 0.0, 0.0), gp_Dir(1.0, 0.0, 0.0)), r, l);
		maker.Build();
		if (!maker.IsDone()) throw std::runtime_error("could not build the cylinder");
		return maker.Solid();
	}
	double lx = size[0], ly = size[1], lz = size[2];
	BRepPrimAPI_MakeBox maker(gp_Pnt(-0.5 * lx, -0.5 * ly, -0.5 * lz), lx, ly, lz);
	maker.Build();
	if (!maker.IsDone()) throw std::runtime_error("could not build the box");
	return maker.Solid();
}

} // namespace aerotelic
