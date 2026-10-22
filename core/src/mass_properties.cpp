#include "aerotelic/mass_properties.hpp"

#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aerotelic {

namespace {

// Running integrals of one mass distribution about the origin: the
// measure (volume or area), its first moment, and the second-moment
// matrix S = integral of x x^T. Everything a mesh contributes lands here,
// and the tensor falls out of S at the end: I = tr(S) 1 - S.
struct Moments {
	double measure = 0.0;
	Vec3 first;
	std::array<double, 9> second{};

	void AddSecond(double scale, const Vec3& a, const Vec3& b, const Vec3& c) {
		// For a simplex with vertices p_k (plus the origin, which adds
		// nothing), the integral of x_i x_j is
		//   scale * ( sum_k p_k,i p_k,j + (sum_k p_k,i)(sum_k p_k,j) )
		// with scale = volume / 20 for a tetrahedron and area / 12 for a
		// triangle -- the one formula, two measures.
		const Vec3 s = a + b + c;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				second[3 * i + j] += scale * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]);
			}
		}
	}
};

// The tensor about `centre` from moments taken about the origin, scaled
// from measure to mass by `density` (mass per unit measure).
MassProperties Resolve(const Moments& m, double density, double volume, double area) {
	MassProperties p;
	p.volume = volume;
	p.area = area;
	if (!(m.measure > 0.0) || !(std::isfinite(density))) {
		return p;  // nothing to spread the mass over: zero mass at the origin
	}
	p.mass = density * m.measure;
	p.centre = m.first / m.measure;

	// About the origin: I = tr(S) 1 - S, with S in mass units.
	InertiaTensor about_origin{};
	const double trace = density * (m.second[0] + m.second[4] + m.second[8]);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			about_origin[3 * i + j] = (i == j ? trace : 0.0) - density * m.second[3 * i + j];
		}
	}
	// Then to the centre of mass: the parallel-axis theorem run backwards.
	const Vec3& c = p.centre;
	const double c2 = LengthSq(c);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			p.inertia[3 * i + j] = about_origin[3 * i + j] - p.mass * ((i == j ? c2 : 0.0) - c[i] * c[j]);
		}
	}
	return p;
}

} // namespace

const char* MassModeName(MassMode m) {
	switch (m) {
		case MassMode::Unset: return "unset";
		case MassMode::Asserted: return "asserted";
		case MassMode::Density: return "density";
	}
	return "unset";
}

MassMode ParseMassMode(const std::string& s) {
	std::string l;
	for (char c : s) l += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	if (l == "unset" || l.empty()) return MassMode::Unset;
	if (l == "asserted") return MassMode::Asserted;
	if (l == "density") return MassMode::Density;
	throw std::invalid_argument("unknown mass mode '" + s + "' (expected unset, asserted or density)");
}

MassProperties SolidMassProperties(const TriMesh& mesh, double density) {
	// Every triangle with the origin makes a tetrahedron whose signed
	// volume adds up, over a closed surface, to the enclosed volume
	// whatever the origin -- the divergence theorem, one simplex at a
	// time. The moments add up the same way.
	Moments m;
	double area = 0.0;
	for (const auto& t : mesh.triangles) {
		const Vec3& a = mesh.vertices[t[0]];
		const Vec3& b = mesh.vertices[t[1]];
		const Vec3& c = mesh.vertices[t[2]];
		const double v = Dot(a, Cross(b, c)) / 6.0;
		m.measure += v;
		m.first += (a + b + c) * (v / 4.0);
		m.AddSecond(v / 20.0, a, b, c);
		area += 0.5 * Length(Cross(b - a, c - a));
	}
	if (m.measure < 0.0) {
		// Inside out: the same body, wound the other way.
		m.measure = -m.measure;
		m.first = -m.first;
		for (double& s : m.second) s = -s;
	}
	return Resolve(m, density, m.measure, area);
}

MassProperties ShellMassProperties(const TriMesh& mesh, double mass) {
	Moments m;
	for (const auto& t : mesh.triangles) {
		const Vec3& a = mesh.vertices[t[0]];
		const Vec3& b = mesh.vertices[t[1]];
		const Vec3& c = mesh.vertices[t[2]];
		const double area = 0.5 * Length(Cross(b - a, c - a));
		m.measure += area;
		m.first += (a + b + c) * (area / 3.0);
		m.AddSecond(area / 12.0, a, b, c);
	}
	const double areal_density = m.measure > 0.0 ? mass / m.measure : 0.0;
	return Resolve(m, areal_density, mesh.SignedVolume(), m.measure);
}

InertiaTensor InertiaAbout(const MassProperties& p, const Vec3& point) {
	const Vec3 d = p.centre - point;
	const double d2 = LengthSq(d);
	InertiaTensor out;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			out[3 * i + j] = p.inertia[3 * i + j] + p.mass * ((i == j ? d2 : 0.0) - d[i] * d[j]);
		}
	}
	return out;
}

MassProperties CombineMassProperties(const std::vector<MassProperties>& parts) {
	MassProperties all;
	Vec3 first;
	for (const MassProperties& p : parts) {
		all.mass += p.mass;
		all.volume += p.volume;
		all.area += p.area;
		first += p.centre * p.mass;
	}
	if (!(all.mass > 0.0)) {
		return all;
	}
	all.centre = first / all.mass;
	for (const MassProperties& p : parts) {
		const InertiaTensor carried = InertiaAbout(p, all.centre);
		for (int i = 0; i < 9; ++i) all.inertia[i] += carried[i];
	}
	return all;
}

void PrincipalInertia(const InertiaTensor& inertia, std::array<double, 3>& moments, std::array<Vec3, 3>& axes) {
	// Cyclic Jacobi on a copy: rotate away the largest off-diagonal until
	// they are all negligible against the diagonal. A dozen sweeps is far
	// more than a 3x3 ever needs.
	double a[3][3];
	double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) a[i][j] = 0.5 * (inertia[3 * i + j] + inertia[3 * j + i]);
	}
	for (int sweep = 0; sweep < 50; ++sweep) {
		double off = 0.0;
		for (int p = 0; p < 3; ++p) {
			for (int q = p + 1; q < 3; ++q) off += a[p][q] * a[p][q];
		}
		const double diag = a[0][0] * a[0][0] + a[1][1] * a[1][1] + a[2][2] * a[2][2];
		if (off <= 1e-30 * (diag + 1e-300)) break;
		for (int p = 0; p < 3; ++p) {
			for (int q = p + 1; q < 3; ++q) {
				if (a[p][q] == 0.0) continue;
				const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
				const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
				const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
				for (int k = 0; k < 3; ++k) {
					const double akp = a[k][p], akq = a[k][q];
					a[k][p] = c * akp - s * akq;
					a[k][q] = s * akp + c * akq;
				}
				for (int k = 0; k < 3; ++k) {
					const double apk = a[p][k], aqk = a[q][k];
					a[p][k] = c * apk - s * aqk;
					a[q][k] = s * apk + c * aqk;
				}
				for (int k = 0; k < 3; ++k) {
					const double vkp = v[k][p], vkq = v[k][q];
					v[k][p] = c * vkp - s * vkq;
					v[k][q] = s * vkp + c * vkq;
				}
			}
		}
	}
	// Ascending, the axes (columns of v) following their moments.
	int order[3] = {0, 1, 2};
	for (int i = 0; i < 3; ++i) {
		for (int j = i + 1; j < 3; ++j) {
			if (a[order[j]][order[j]] < a[order[i]][order[i]]) std::swap(order[i], order[j]);
		}
	}
	for (int n = 0; n < 3; ++n) {
		const int k = order[n];
		moments[n] = a[k][k];
		axes[n] = Normalized(Vec3{v[0][k], v[1][k], v[2][k]});
	}
	// A right-handed frame, so the third axis is the cross of the first two.
	if (Dot(Cross(axes[0], axes[1]), axes[2]) < 0.0) axes[2] = -axes[2];
}

} // namespace aerotelic
