#pragma once

// A minimal 3-vector for the OCCT-free mesh, distance-field and lattice
// code (mesh.hpp, sdf.hpp, lattice.hpp). Deliberately a plain aggregate
// with free functions rather than a class with operators on std::vector:
// these modules run tight loops over millions of points, where the
// per-element heap traffic of std::vector<double> would dominate.

#include <array>
#include <cmath>

namespace aerotelic {

struct Vec3 {
	double x = 0.0, y = 0.0, z = 0.0;

	constexpr Vec3() = default;
	constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
	explicit constexpr Vec3(const std::array<double, 3>& a) : x(a[0]), y(a[1]), z(a[2]) {}

	double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
	double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

	Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
	Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
	Vec3 operator-() const { return {-x, -y, -z}; }
	Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
	Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

	std::array<double, 3> ToArray() const { return {x, y, z}; }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b) {
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double LengthSq(const Vec3& v) { return Dot(v, v); }
inline double Length(const Vec3& v) { return std::sqrt(LengthSq(v)); }
inline double DistanceSq(const Vec3& a, const Vec3& b) { return LengthSq(a - b); }
inline double Distance(const Vec3& a, const Vec3& b) { return Length(a - b); }
inline Vec3 Normalized(const Vec3& v) {
	double n = Length(v);
	return n > 0.0 ? v / n : Vec3{0.0, 0.0, 0.0};
}
inline Vec3 Min(const Vec3& a, const Vec3& b) { return {std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z)}; }
inline Vec3 Max(const Vec3& a, const Vec3& b) { return {std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z)}; }
inline Vec3 Lerp(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

// Axis-aligned bounding box. Empty (min > max) until the first Add().
struct Box3 {
	Vec3 min{1e300, 1e300, 1e300};
	Vec3 max{-1e300, -1e300, -1e300};

	bool IsEmpty() const { return min.x > max.x; }
	void Add(const Vec3& p) { min = Min(min, p); max = Max(max, p); }
	void Add(const Box3& b) { if (!b.IsEmpty()) { Add(b.min); Add(b.max); } }
	Vec3 Center() const { return (min + max) * 0.5; }
	Vec3 Extent() const { return max - min; }
	double Diagonal() const { return IsEmpty() ? 0.0 : Length(max - min); }
	Box3 Expanded(double margin) const {
		Box3 b = *this;
		if (!IsEmpty()) {
			b.min -= Vec3{margin, margin, margin};
			b.max += Vec3{margin, margin, margin};
		}
		return b;
	}
	// Squared distance from p to the box (0 inside).
	double DistanceSq(const Vec3& p) const {
		double d = 0.0;
		for (int i = 0; i < 3; ++i) {
			double v = p[i];
			if (v < min[i]) d += (min[i] - v) * (min[i] - v);
			else if (v > max[i]) d += (v - max[i]) * (v - max[i]);
		}
		return d;
	}
};

} // namespace aerotelic
