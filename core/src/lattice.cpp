#include "aerotelic/lattice.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

#include "aerotelic/parallel.hpp"

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

const std::vector<std::string> kTpmsNames = {
	"Gyroid", "Diamond", "Primitive", "IWP", "Neovius", "Lidinoid", "Fischer-Koch S", "Fischer-Koch Y", "FRD", "F", "G'", "G'2",
	"D'", "K", "Octo", "PN", "Q*", "Slotted-P", "Split-P", "W", "+-Y", "Complementary D", "Complementary G",
	"Complementary I2-Y**", "Complementary +-Y", "Complementary S", "Complementary Y",
};
const std::vector<std::string> kStrutNames = {"BCC", "FCC", "G7", "Grid", "Octet", "Tesseract", "Vintiles", "X-lattice"};
const std::vector<std::string> kAuxeticNames = {"Re-entrant"};

std::string Lower(std::string s) {
	for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return s;
}

// ---------------------------------------------------------------------------
// Strut cell tables (nodes in [0, 1)^3 -- a coordinate of 1 is the periodic
// image of 0 and is written as 1 where a strut needs that end)
// ---------------------------------------------------------------------------

StrutCell MakeCell(std::vector<Vec3> nodes, std::vector<std::array<int, 2>> struts) {
	StrutCell c;
	c.nodes = std::move(nodes);
	c.struts = std::move(struts);
	return c;
}

const std::map<std::string, StrutCell>& StrutCells() {
	static const std::map<std::string, StrutCell> cells = [] {
		std::map<std::string, StrutCell> m;
		const Vec3 O{0, 0, 0}, C{0.5, 0.5, 0.5};
		auto corners = [] {
			std::vector<Vec3> v;
			for (int i = 0; i < 8; ++i) v.push_back({double(i & 1), double((i >> 1) & 1), double((i >> 2) & 1)});
			return v;
		}();

		// Grid (simple cubic): the three cell edges from the origin; every
		// other edge is a periodic image.
		m["Grid"] = MakeCell({O, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {{0, 1}, {0, 2}, {0, 3}});

		// BCC: centre node to the eight corners.
		{
			std::vector<Vec3> n = {C};
			std::vector<std::array<int, 2>> s;
			for (int i = 0; i < 8; ++i) { n.push_back(corners[i]); s.push_back({0, i + 1}); }
			m["BCC"] = MakeCell(n, s);
		}

		// FCC: each face centre to its four corners (the faces x=0, y=0, z=0;
		// the opposite faces are their periodic images).
		auto fcc_struts = [&](std::vector<Vec3>& n, std::vector<std::array<int, 2>>& s) {
			const Vec3 faces[3] = {{0, 0.5, 0.5}, {0.5, 0, 0.5}, {0.5, 0.5, 0}};
			for (int f = 0; f < 3; ++f) {
				int fi = static_cast<int>(n.size());
				n.push_back(faces[f]);
				for (int i = 0; i < 8; ++i) {
					if (corners[i][f] != 0.0) continue; // the four corners on that face
					n.push_back(corners[i]);
					s.push_back({fi, static_cast<int>(n.size()) - 1});
				}
			}
		};
		{
			std::vector<Vec3> n;
			std::vector<std::array<int, 2>> s;
			fcc_struts(n, s);
			m["FCC"] = MakeCell(n, s);
		}

		// Octet: FCC plus the octahedron joining the six face centres.
		{
			std::vector<Vec3> n;
			std::vector<std::array<int, 2>> s;
			fcc_struts(n, s);
			const Vec3 fc[6] = {{0, .5, .5}, {1, .5, .5}, {.5, 0, .5}, {.5, 1, .5}, {.5, .5, 0}, {.5, .5, 1}};
			int base = static_cast<int>(n.size());
			for (const auto& f : fc) n.push_back(f);
			for (int a = 0; a < 6; ++a)
				for (int b = a + 1; b < 6; ++b) {
					if (a / 2 == b / 2) continue; // opposite faces are not joined
					s.push_back({base + a, base + b});
				}
			m["Octet"] = MakeCell(n, s);
		}

		// Tesseract: the cell's edges, a half-size inner cube, and a strut
		// from each corner to the inner corner nearest it.
		{
			std::vector<Vec3> n = {O, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
			std::vector<std::array<int, 2>> s = {{0, 1}, {0, 2}, {0, 3}};
			int base = static_cast<int>(n.size());
			for (int i = 0; i < 8; ++i) n.push_back({0.25 + 0.5 * corners[i].x, 0.25 + 0.5 * corners[i].y, 0.25 + 0.5 * corners[i].z});
			for (int i = 0; i < 8; ++i)
				for (int j = i + 1; j < 8; ++j) {
					int diff = i ^ j;
					if (diff == 1 || diff == 2 || diff == 4) s.push_back({base + i, base + j}); // inner cube edges
				}
			for (int i = 0; i < 8; ++i) {
				n.push_back(corners[i]);
				s.push_back({static_cast<int>(n.size()) - 1, base + i});
			}
			m["Tesseract"] = MakeCell(n, s);
		}

		// X-lattice: both diagonals of each face (x=0, y=0, z=0 faces).
		{
			std::vector<Vec3> n;
			std::vector<std::array<int, 2>> s;
			for (int f = 0; f < 3; ++f) {
				int a = (f + 1) % 3, b = (f + 2) % 3;
				Vec3 p00, p11, p01, p10;
				p00[a] = 0; p00[b] = 0; p11[a] = 1; p11[b] = 1; p01[a] = 0; p01[b] = 1; p10[a] = 1; p10[b] = 0;
				int i0 = static_cast<int>(n.size());
				n.push_back(p00); n.push_back(p11); n.push_back(p01); n.push_back(p10);
				s.push_back({i0, i0 + 1});
				s.push_back({i0 + 2, i0 + 3});
			}
			m["X-lattice"] = MakeCell(n, s);
		}

		// G7: the BCC diagonals with a vertical pillar through the centre
		// (BCC-Z); see lattice.hpp on the strut tables being data.
		{
			std::vector<Vec3> n = {C};
			std::vector<std::array<int, 2>> s;
			for (int i = 0; i < 8; ++i) { n.push_back(corners[i]); s.push_back({0, i + 1}); }
			n.push_back({0.5, 0.5, 0.0});
			n.push_back({0.5, 0.5, 1.0});
			s.push_back({9, 10});
			m["G7"] = MakeCell(n, s);
		}

		// Vintiles: the cuboctahedral frame on the twelve edge midpoints,
		// each joined to its four nearest edge midpoints.
		{
			std::vector<Vec3> n;
			std::vector<std::array<int, 2>> s;
			auto add = [&](const Vec3& a, const Vec3& b) {
				int i = static_cast<int>(n.size());
				n.push_back(a);
				n.push_back(b);
				s.push_back({i, i + 1});
			};
			for (int x = 0; x <= 1; ++x)
				for (int y = 0; y <= 1; ++y)
					for (int z = 0; z <= 1; ++z) {
						Vec3 mx{0.5, double(y), double(z)}, my{double(x), 0.5, double(z)}, mz{double(x), double(y), 0.5};
						add(mx, my);
						add(mx, mz);
						add(my, mz);
					}
			m["Vintiles"] = MakeCell(n, s);
		}

		// Re-entrant (auxetic): the cell's horizontal edges, with each
		// vertical edge replaced by two struts bowed inward to a node a
		// quarter cell toward the centre at mid height.
		{
			std::vector<Vec3> n = {O, {1, 0, 0}, {0, 1, 0}};
			std::vector<std::array<int, 2>> s = {{0, 1}, {0, 2}};
			const double a = 0.25;
			for (int x = 0; x <= 1; ++x)
				for (int y = 0; y <= 1; ++y) {
					Vec3 bottom{double(x), double(y), 0.0}, top{double(x), double(y), 1.0};
					Vec3 mid{x == 0 ? a : 1.0 - a, y == 0 ? a : 1.0 - a, 0.5};
					int i = static_cast<int>(n.size());
					n.push_back(bottom);
					n.push_back(mid);
					n.push_back(top);
					s.push_back({i, i + 1});
					s.push_back({i + 1, i + 2});
				}
			m["Re-entrant"] = MakeCell(n, s);
		}
		return m;
	}();
	return cells;
}

double Fract(double x) { return x - std::floor(x); }

// Distance (cell units) from r to the nearest periodic copy of the
// piece, by the paper's rounding of the midpoint offset (eq. 2-8).
double PieceDistanceSq(const Vec3& r, const Vec3& p0, const Vec3& p1, const Vec3& mid) {
	Vec3 k{std::round(r.x - mid.x), std::round(r.y - mid.y), std::round(r.z - mid.z)};
	Vec3 a = p0 + k, b = p1 + k;
	Vec3 u = b - a, w = r - a;
	double uu = Dot(u, u);
	double t = uu > 0.0 ? std::clamp(Dot(w, u) / uu, 0.0, 1.0) : 0.0;
	Vec3 q = a + u * t;
	return DistanceSq(r, q);
}

} // namespace

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

const char* LatticeFamilyName(LatticeFamily f) {
	switch (f) {
		case LatticeFamily::Tpms: return "tpms";
		case LatticeFamily::Strut: return "strut";
		case LatticeFamily::Auxetic: return "auxetic";
	}
	return "tpms";
}

LatticeFamily ParseLatticeFamily(const std::string& s) {
	std::string l = Lower(s);
	if (l == "tpms") return LatticeFamily::Tpms;
	if (l == "strut" || l == "strut-based") return LatticeFamily::Strut;
	if (l == "auxetic") return LatticeFamily::Auxetic;
	throw std::invalid_argument("unknown lattice family '" + s + "' (expected tpms, strut or auxetic)");
}

const char* TpmsPhaseName(TpmsPhase p) {
	switch (p) {
		case TpmsPhase::Network: return "network";
		case TpmsPhase::Sheet: return "sheet";
		case TpmsPhase::IsoSurface: return "iso-surface";
	}
	return "network";
}

TpmsPhase ParseTpmsPhase(const std::string& s) {
	std::string l = Lower(s);
	if (l == "network") return TpmsPhase::Network;
	if (l == "sheet" || l == "matrix") return TpmsPhase::Sheet;
	if (l == "iso-surface" || l == "isosurface" || l == "iso") return TpmsPhase::IsoSurface;
	throw std::invalid_argument("unknown TPMS phase '" + s + "' (expected network, sheet or iso-surface)");
}

const std::vector<std::string>& TpmsTypeNames() { return kTpmsNames; }
const std::vector<std::string>& StrutTypeNames() { return kStrutNames; }
const std::vector<std::string>& AuxeticTypeNames() { return kAuxeticNames; }
const std::vector<std::string>& LatticeTypeNames(LatticeFamily f) {
	switch (f) {
		case LatticeFamily::Tpms: return kTpmsNames;
		case LatticeFamily::Strut: return kStrutNames;
		case LatticeFamily::Auxetic: return kAuxeticNames;
	}
	return kTpmsNames;
}

const StrutCell& StrutCellFor(const std::string& type) {
	const auto& cells = StrutCells();
	for (const auto& [name, cell] : cells) {
		if (Lower(name) == Lower(type)) return cell;
	}
	throw std::invalid_argument("unknown strut lattice '" + type + "'");
}

// ---------------------------------------------------------------------------
// TPMS level-set functions (OpenLattice Table 3)
// ---------------------------------------------------------------------------

double TpmsFunction(const std::string& type, double X, double Y, double Z) {
	using std::cos;
	using std::sin;
	const double Cx = cos(X), Cy = cos(Y), Cz = cos(Z), Sx = sin(X), Sy = sin(Y), Sz = sin(Z);
	const double C2x = cos(2 * X), C2y = cos(2 * Y), C2z = cos(2 * Z), S2x = sin(2 * X), S2y = sin(2 * Y), S2z = sin(2 * Z);
	const double C3x = cos(3 * X), C3y = cos(3 * Y), C3z = cos(3 * Z), S3x = sin(3 * X), S3y = sin(3 * Y), S3z = sin(3 * Z);
	const std::string t = Lower(type);
	if (t == "gyroid") return Cx * Sy + Cy * Sz + Cz * Sx;
	if (t == "diamond") return Sx * Sy * Sz + Sx * Cy * Cz + Cx * Sy * Cz + Cx * Cy * Sz;
	if (t == "primitive") return Cx + Cy + Cz;
	if (t == "iwp") return 2 * (Cx * Cy + Cy * Cz + Cz * Cx) - (C2x + C2y + C2z);
	if (t == "neovius" || t == "neovious") return 3 * (Cx + Cy + Cz) + 4 * Cx * Cy * Cz;
	if (t == "lidinoid") return (S2x * Cy * Sz + S2y * Cz * Sx + S2z * Cx * Sy) - (C2x * C2y + C2y * C2z + C2z * C2x) + 0.3;
	if (t == "fischer-koch s") return C2x * Sy * Cz + Cx * C2y * Sz + Sx * Cy * C2z;
	if (t == "fischer-koch y") return Cx * Cy * Cz + Sx * Sy * Sz + (S2x * Sy + S2y * Sz + Sx * S2z) + (Cx * S2y + Cy * S2z + S2x * Cz);
	if (t == "frd") return 8 * Cx * Cy * Cz + C2x * C2y * C2z - (C2x * C2y + C2y * C2z + C2z * C2x);
	if (t == "f") return Cx * Cy * Cz;
	if (t == "g'") return S2x * Cy * Sz + Sx * S2y * Cz + Cx * Sy * S2z + 0.32;
	if (t == "g'2") return 5 * (S2x * Cy * Sz + Sx * S2y * Cz + Cx * Sy * S2z) + (C2x * C2y + C2y * C2z + C2z * C2x);
	if (t == "d'") return 0.5 * (Cx * Cy * Cz + Cx * Sy * Sz + Sx * Cy * Sz + Sx * Sy * Cz) - 0.5 * (S2x * S2y + S2y * S2z + S2z * S2x) - 0.2;
	if (t == "k") return 0.3 * (Cx + Cy + Cz) + 0.3 * (Cx * Cy + Cy * Cz + Cz * Cx) - 0.4 * (C2x + C2y + C2z) + 0.2;
	if (t == "octo") return 0.6 * (Cx * Cy + Cy * Cz + Cz * Cx) - 0.4 * (Cx + Cy + Cz) + 0.25;
	if (t == "pn") {
		return 0.3 * (Cx * Cy * Cz) + 0.2 * (Cx + Cy + Cz) + 0.1 * (C2x * C2y * C2z) + 0.1 * (C2x + C2y + C2z) +
			0.05 * (C3x + C3y + C3z) + 0.1 * (Cx * Cy + Cy * Cz + Cz * Cx);
	}
	if (t == "q*") {
		const double Cxmy = cos(X - Y);
		return (Cx - 2 * Cy) * Cz - std::sqrt(3.0) * Sz * (Cxmy - Cx) + Cxmy * Cz;
	}
	if (t == "slotted-p") {
		return -2 * (Cx * Cy + Cy * Cz + Cx * Cz) - 2 * (C2x + C2y + C2z) + (C2x * Cy + C2y * Cz + Cx * C2z) - (Cx * C2y + Cy * C2z + C2x * Cz);
	}
	if (t == "split-p") {
		return 1.1 * (S2x * Cy * Sz + S2y * Cz * Sx + S2z * Cx * Sy) - 0.2 * (C2x * C2y + C2y * C2z + C2z * C2x) - 0.4 * (C2x + C2y + C2z);
	}
	if (t == "w") return (C2x * Cy + C2y * Cz + Cx * C2z) - (Cx * C2y + Cy * C2z + C2x * Cz);
	if (t == "+-y") return 2 * (Cx * Cy * Cz) + (S2x * Sy + S2y * Sz + Sx * S2z);
	if (t == "complementary d") {
		return cos(3 * X + Y) * Cz - sin(3 * X - Y) * Sz + cos(X + 3 * Y) * Cz + sin(X - 3 * Y) * Sz + cos(X - Y) * C3z - sin(X + Y) * S3z;
	}
	if (t == "complementary g") {
		return 3 * (Sx * Cy + Sy * Cz + Cx * Sz) + 2 * (S3x * Cy + S3y * Cz + Cx * S3z) - 2 * (Sx * C3y + Sy * C3z + C3x * Sz);
	}
	if (t == "complementary i2-y**") return 2 * (S2x * Cy * Sz + Sx * S2y * Cz + Cx * Sy * S2z) + (C2x * C2y + C2y * C2z + C2x * C2z);
	if (t == "complementary +-y") return -2 * (Cx * Cy * Cz) + (S2x * Sy + S2y * Sz + Sx * S2z);
	if (t == "complementary s") {
		return (C2x + C2y + C2z) + 2 * (S3x * S2y * Cz + Cx * S3y * S2z + S2x * Cy * S3z) + 2 * (S2x * C3y * Sz + Sx * S2y * C3z + C3x * Sy * S2z);
	}
	if (t == "complementary y") {
		return -Sx * Sy * Sz + S2x * Sy + S2y * Sz + Sx * S2z - Cx * Cy * Cz + S2x * Cz + Cx * S2y + Cy * S2z;
	}
	throw std::invalid_argument("unknown TPMS lattice '" + type + "'");
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

namespace {

// Every sample's raw value over the unit cell: U (TPMS) or the distance
// to the nearest strut in cell units.
// Stratified samples: one per sub-cell, jittered by a fixed-seed
// generator so the cells' symmetries do not pile thousands of samples
// onto identical values (which would make the solid fraction jump in
// steps as the level moves and defeat the bisection).
double Jitter(uint32_t& state) {
	state = state * 1664525u + 1013904223u;
	return (state >> 8) * (1.0 / 16777216.0);
}

std::vector<double> CellSamples(const LatticeSpec& spec, int samples, double* mean_gradient) {
	const int n = std::max(samples, 8);
	std::vector<double> values;
	values.reserve(static_cast<size_t>(n) * n * n);
	uint32_t rng = 12345u;
	if (spec.family == LatticeFamily::Tpms) {
		double grad_sum = 0.0;
		const double h = 1e-4;
		for (int i = 0; i < n; ++i)
			for (int j = 0; j < n; ++j)
				for (int k = 0; k < n; ++k) {
					double X = 2 * kPi * (i + Jitter(rng)) / n, Y = 2 * kPi * (j + Jitter(rng)) / n, Z = 2 * kPi * (k + Jitter(rng)) / n;
					values.push_back(TpmsFunction(spec.type, X, Y, Z));
					if (mean_gradient) {
						double gx = (TpmsFunction(spec.type, X + h, Y, Z) - TpmsFunction(spec.type, X - h, Y, Z)) / (2 * h);
						double gy = (TpmsFunction(spec.type, X, Y + h, Z) - TpmsFunction(spec.type, X, Y - h, Z)) / (2 * h);
						double gz = (TpmsFunction(spec.type, X, Y, Z + h) - TpmsFunction(spec.type, X, Y, Z - h)) / (2 * h);
						grad_sum += std::sqrt(gx * gx + gy * gy + gz * gz);
					}
				}
		if (mean_gradient) *mean_gradient = grad_sum / values.size();
		return values;
	}
	const StrutCell& cell = StrutCellFor(spec.type);
	std::vector<std::array<Vec3, 2>> pieces;
	for (const auto& s : cell.struts) {
		Vec3 a = cell.nodes[s[0]], b = cell.nodes[s[1]];
		int n_pieces = std::max(1, static_cast<int>(std::ceil(Distance(a, b) / 0.5)));
		for (int q = 0; q < n_pieces; ++q) pieces.push_back({Lerp(a, b, double(q) / n_pieces), Lerp(a, b, double(q + 1) / n_pieces)});
	}
	for (int i = 0; i < n; ++i)
		for (int j = 0; j < n; ++j)
			for (int k = 0; k < n; ++k) {
				Vec3 r{(i + Jitter(rng)) / n, (j + Jitter(rng)) / n, (k + Jitter(rng)) / n};
				double best = std::numeric_limits<double>::infinity();
				for (const auto& p : pieces) best = std::min(best, PieceDistanceSq(r, p[0], p[1], (p[0] + p[1]) * 0.5));
				values.push_back(std::sqrt(best));
			}
	if (mean_gradient) *mean_gradient = 1.0;
	return values;
}

double FractionAt(const LatticeSpec& spec, const std::vector<double>& values, double level) {
	size_t inside = 0;
	if (spec.family == LatticeFamily::Tpms) {
		if (spec.phase == TpmsPhase::Network) {
			for (double v : values) inside += (v <= level) ? 1 : 0;
		} else {
			for (double v : values) inside += (std::fabs(v) <= level) ? 1 : 0;
		}
	} else {
		for (double v : values) inside += (v <= level) ? 1 : 0;
	}
	return static_cast<double>(inside) / values.size();
}

} // namespace

double CellSolidFraction(const LatticeSpec& spec, double level, int samples) {
	return FractionAt(spec, CellSamples(spec, samples, nullptr), level);
}

double CalibrateLevel(const LatticeSpec& spec, double* mean_gradient, int samples) {
	std::vector<double> values = CellSamples(spec, samples, mean_gradient);
	if (spec.family == LatticeFamily::Tpms && spec.phase == TpmsPhase::IsoSurface) return 0.0;
	const double target = std::clamp(spec.volume_fraction, 0.001, 0.999);
	double lo, hi;
	if (spec.family == LatticeFamily::Tpms && spec.phase == TpmsPhase::Network) {
		lo = *std::min_element(values.begin(), values.end());
		hi = *std::max_element(values.begin(), values.end());
	} else if (spec.family == LatticeFamily::Tpms) {
		lo = 0.0;
		hi = 0.0;
		for (double v : values) hi = std::max(hi, std::fabs(v));
	} else {
		lo = 0.0;
		hi = 0.9; // a radius this large fills any cell
	}
	// FractionAt is non-decreasing in the level for every case.
	for (int it = 0; it < 60; ++it) {
		double mid = 0.5 * (lo + hi);
		if (FractionAt(spec, values, mid) < target) lo = mid;
		else hi = mid;
	}
	return 0.5 * (lo + hi);
}

// ---------------------------------------------------------------------------
// LatticeField
// ---------------------------------------------------------------------------

LatticeField::LatticeField(const LatticeSpec& spec) : spec_(spec) {
	if (!(spec.cell_size > 0.0)) throw std::invalid_argument("the lattice cell size must be greater than zero");
	k_ = 2.0 * kPi / spec.cell_size;
	if (spec.family == LatticeFamily::Tpms) {
		const auto& names = TpmsTypeNames();
		bool known = std::any_of(names.begin(), names.end(), [&](const std::string& n) { return Lower(n) == Lower(spec.type); });
		if (!known) throw std::invalid_argument("unknown TPMS lattice '" + spec.type + "'");
		double mean_grad = 1.0;
		level_ = CalibrateLevel(spec, &mean_grad);
		gradient_scale_ = std::max(mean_grad, 1e-6) * k_;
	} else {
		cell_ = &StrutCellFor(spec.type);
		level_ = CalibrateLevel(spec);
		for (const auto& s : cell_->struts) {
			Vec3 a = cell_->nodes[s[0]], b = cell_->nodes[s[1]];
			int n_pieces = std::max(1, static_cast<int>(std::ceil(Distance(a, b) / 0.5)));
			for (int q = 0; q < n_pieces; ++q) {
				std::array<Vec3, 2> piece = {Lerp(a, b, double(q) / n_pieces), Lerp(a, b, double(q + 1) / n_pieces)};
				pieces_.push_back(piece);
				piece_mid_.push_back((piece[0] + piece[1]) * 0.5);
			}
		}
	}
}

double LatticeField::Raw(const Vec3& p) const {
	Vec3 q = p - spec_.origin;
	return TpmsFunction(spec_.type, k_ * q.x, k_ * q.y, k_ * q.z);
}

double LatticeField::operator()(const Vec3& p) const {
	if (spec_.family == LatticeFamily::Tpms) {
		double u = Raw(p);
		switch (spec_.phase) {
			case TpmsPhase::Network: return (u - level_) / gradient_scale_;
			case TpmsPhase::Sheet: return (std::fabs(u) - level_) / gradient_scale_;
			case TpmsPhase::IsoSurface: {
				// Distance to the t = 0 surface from the local gradient.
				const double h = 1e-3 * spec_.cell_size;
				double gx = (Raw({p.x + h, p.y, p.z}) - Raw({p.x - h, p.y, p.z})) / (2 * h);
				double gy = (Raw({p.x, p.y + h, p.z}) - Raw({p.x, p.y - h, p.z})) / (2 * h);
				double gz = (Raw({p.x, p.y, p.z + h}) - Raw({p.x, p.y, p.z - h})) / (2 * h);
				double g = std::max(std::sqrt(gx * gx + gy * gy + gz * gz), 1e-9);
				return std::fabs(u) / g - 0.5 * spec_.wall_thickness;
			}
		}
		return u;
	}
	Vec3 r = (p - spec_.origin) / spec_.cell_size;
	double best = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < pieces_.size(); ++i) best = std::min(best, PieceDistanceSq(r, pieces_[i][0], pieces_[i][1], piece_mid_[i]));
	return (std::sqrt(best) - level_) * spec_.cell_size;
}

// ---------------------------------------------------------------------------
// Grid planning and sampling
// ---------------------------------------------------------------------------

LatticeGrid PlanLatticeGrid(const LatticeSpec& spec, const Box3& bounds) {
	if (bounds.IsEmpty()) throw std::invalid_argument("the lattice domain is empty");
	if (!(spec.cell_size > 0.0)) throw std::invalid_argument("the lattice cell size must be greater than zero");
	LatticeGrid g;
	g.h = spec.cell_size / std::max(spec.resolution, 2);
	Box3 b = bounds.Expanded(2.0 * g.h);
	g.origin = b.min;
	Vec3 ext = b.Extent();
	g.nx = std::max(2, static_cast<int>(std::ceil(ext.x / g.h)));
	g.ny = std::max(2, static_cast<int>(std::ceil(ext.y / g.h)));
	g.nz = std::max(2, static_cast<int>(std::ceil(ext.z / g.h)));
	return g;
}

LatticeResult GenerateLattice(const LatticeSpec& spec, const LatticeDomain& domain, const std::function<bool(double)>& progress,
	int threads, size_t max_nodes) {
	if (!domain.body) throw std::invalid_argument("the lattice domain has no body");
	LatticeResult result;
	result.grid = PlanLatticeGrid(spec, domain.bounds);
	const LatticeGrid& g = result.grid;
	if (g.Nodes() > max_nodes) {
		throw std::runtime_error("the lattice grid would need " + std::to_string(g.Nodes()) + " samples (limit " + std::to_string(max_nodes) +
			"): use a larger cell size or a lower resolution");
	}
	LatticeField lattice(spec);
	if (progress && !progress(0.0)) { result.cancelled = true; return result; }

	const double skin = std::max(spec.skin_thickness, 0.0);
	auto body = [&](const Vec3& p) {
		double b = domain.body(p);
		if (domain.exclusion) b = std::max(b, -domain.exclusion(p));
		return b;
	};
	auto combined = [&](const Vec3& p, double b) {
		double f = std::max(lattice(p), b);
		if (skin > 0.0) f = std::min(f, std::max(b, -skin - b));
		return f;
	};

	const int NX = g.nx + 1, NY = g.ny + 1, NZ = g.nz + 1;
	std::vector<float> values(g.Nodes(), 0.0f);
	auto node_index = [&](int i, int j, int k) { return (static_cast<size_t>(k) * NY + j) * NX + i; };

	// Blocks of nodes, classified from one body evaluation at the centre:
	// a block whose every node is more than a voxel outside the body gets
	// a positive value without further evaluations; one more than a voxel
	// (and the skin) inside needs only the lattice. The margin of one
	// voxel keeps every estimated value away from any zero crossing, so
	// the surface is never interpolated from an estimate.
	const int B = 8;
	const int bx = (NX + B - 1) / B, by = (NY + B - 1) / B, bz = (NZ + B - 1) / B;
	const size_t n_blocks = static_cast<size_t>(bx) * by * bz;
	std::atomic<size_t> body_inside{0}, lattice_inside{0}, evaluated{0};
	const float tiny = static_cast<float>(1e-6 * g.h);

	ParallelFor(n_blocks, [&](size_t block) {
		int kb = static_cast<int>(block / (static_cast<size_t>(bx) * by));
		int rem = static_cast<int>(block % (static_cast<size_t>(bx) * by));
		int jb = rem / bx, ib = rem % bx;
		int i0 = ib * B, j0 = jb * B, k0 = kb * B;
		int i1 = std::min(NX, i0 + B), j1 = std::min(NY, j0 + B), k1 = std::min(NZ, k0 + B);
		Vec3 lo = g.origin + Vec3{i0 * g.h, j0 * g.h, k0 * g.h};
		Vec3 hi = g.origin + Vec3{(i1 - 1) * g.h, (j1 - 1) * g.h, (k1 - 1) * g.h};
		Vec3 centre = (lo + hi) * 0.5;
		double radius = 0.5 * Distance(lo, hi);
		double bc = body(centre);
		size_t n_body = 0, n_lat = 0, n_eval = 1;
		if (bc - radius > g.h) {
			// Entirely outside: a positive lower bound on the distance.
			for (int k = k0; k < k1; ++k)
				for (int j = j0; j < j1; ++j)
					for (int i = i0; i < i1; ++i) {
						Vec3 p = g.origin + Vec3{i * g.h, j * g.h, k * g.h};
						values[node_index(i, j, k)] = static_cast<float>(std::max(bc - Distance(p, centre), 0.5 * g.h));
					}
		} else if (bc + radius < -(skin + g.h)) {
			// Entirely inside, past the skin: the lattice alone.
			for (int k = k0; k < k1; ++k)
				for (int j = j0; j < j1; ++j)
					for (int i = i0; i < i1; ++i) {
						Vec3 p = g.origin + Vec3{i * g.h, j * g.h, k * g.h};
						float f = static_cast<float>(lattice(p));
						if (f == 0.0f) f = -tiny;
						values[node_index(i, j, k)] = f;
						++n_body;
						if (f <= 0.0f) ++n_lat;
					}
		} else {
			for (int k = k0; k < k1; ++k)
				for (int j = j0; j < j1; ++j)
					for (int i = i0; i < i1; ++i) {
						Vec3 p = g.origin + Vec3{i * g.h, j * g.h, k * g.h};
						double b = body(p);
						++n_eval;
						float f = static_cast<float>(combined(p, b));
						if (f == 0.0f) f = -tiny;
						values[node_index(i, j, k)] = f;
						if (b <= 0.0) {
							++n_body;
							if (f <= 0.0f) ++n_lat;
						}
					}
		}
		body_inside += n_body;
		lattice_inside += n_lat;
		evaluated += n_eval;
	}, threads);
	result.nodes_evaluated = evaluated.load();
	result.achieved_fraction = body_inside.load() > 0 ? static_cast<double>(lattice_inside.load()) / body_inside.load() : 0.0;
	if (progress && !progress(0.6)) { result.cancelled = true; return result; }

	result.mesh = MarchingTetrahedra(g, values, threads);
	if (progress) progress(1.0);
	return result;
}

// ---------------------------------------------------------------------------
// Marching tetrahedra
// ---------------------------------------------------------------------------

namespace {

struct SlabOutput {
	std::vector<uint64_t> keys;     // one per local vertex: global edge key
	std::vector<Vec3> vertices;
	std::vector<std::array<int, 3>> triangles; // local indices
};

} // namespace

TriMesh MarchingTetrahedra(const LatticeGrid& g, const std::vector<float>& values, int threads) {
	const int NX = g.nx + 1, NY = g.ny + 1;
	if (values.size() != g.Nodes()) throw std::invalid_argument("MarchingTetrahedra: value count does not match the grid");
	auto node_index = [&](int i, int j, int k) { return (static_cast<size_t>(k) * NY + j) * NX + i; };

	// The six tetrahedra around the cube's main diagonal (corner 0 to 7),
	// one per ordering of the axes; corner bits are (x, y, z).
	const int axis_orders[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
	int tets[6][4];
	for (int t = 0; t < 6; ++t) {
		int a = 1 << axis_orders[t][0], b = 1 << axis_orders[t][1];
		tets[t][0] = 0;
		tets[t][1] = a;
		tets[t][2] = a | b;
		tets[t][3] = 7;
	}

	const int slab_cells = std::max(1, g.nz / std::max(1, 4 * (threads > 0 ? threads : DefaultThreadCount())));
	const int n_slabs = (g.nz + slab_cells - 1) / slab_cells;
	std::vector<SlabOutput> slabs(n_slabs);

	ParallelFor(static_cast<size_t>(n_slabs), [&](size_t s) {
		SlabOutput& out = slabs[s];
		std::unordered_map<uint64_t, int> local;
		const int kz0 = static_cast<int>(s) * slab_cells, kz1 = std::min(g.nz, kz0 + slab_cells);
		auto value_at = [&](size_t n) {
			float v = values[n];
			return v == 0.0f ? -1e-30f : v;
		};
		// Vertex on the edge between grid nodes a and b (a's bits < b's bits).
		auto edge_vertex = [&](int ci, int cj, int ck, int ca, int cb, float va, float vb) {
			int ai = ci + (ca & 1), aj = cj + ((ca >> 1) & 1), ak = ck + ((ca >> 2) & 1);
			int dir = ca ^ cb; // positive direction bits from a to b
			uint64_t key = (static_cast<uint64_t>(node_index(ai, aj, ak)) << 3) | static_cast<uint64_t>(dir);
			auto it = local.find(key);
			if (it != local.end()) return it->second;
			double t = static_cast<double>(va) / (static_cast<double>(va) - static_cast<double>(vb));
			t = std::clamp(t, 0.0, 1.0);
			Vec3 pa = g.origin + Vec3{ai * g.h, aj * g.h, ak * g.h};
			Vec3 pb = pa + Vec3{(dir & 1) * g.h, ((dir >> 1) & 1) * g.h, ((dir >> 2) & 1) * g.h};
			int idx = static_cast<int>(out.vertices.size());
			out.vertices.push_back(Lerp(pa, pb, t));
			out.keys.push_back(key);
			local.emplace(key, idx);
			return idx;
		};
		// Orientation is decided on the triangle through the three edges'
		// MIDPOINTS (never degenerate), not the interpolated vertices, which
		// can coincide when a sample sits on the surface: sliding each vertex
		// along its edge never flips the triangle's side of the tetrahedron.
		auto emit = [&](int v0, int v1, int v2, const Vec3& m0, const Vec3& m1, const Vec3& m2, const Vec3& outward) {
			Vec3 n = Cross(m1 - m0, m2 - m0);
			if (Dot(n, outward) < 0.0) std::swap(v1, v2);
			out.triangles.push_back({v0, v1, v2});
		};

		for (int ck = kz0; ck < kz1; ++ck)
			for (int cj = 0; cj < g.ny; ++cj)
				for (int ci = 0; ci < g.nx; ++ci) {
					float v[8];
					bool any_in = false, any_out = false;
					for (int c = 0; c < 8; ++c) {
						v[c] = value_at(node_index(ci + (c & 1), cj + ((c >> 1) & 1), ck + ((c >> 2) & 1)));
						if (v[c] <= 0.0f) any_in = true;
						else any_out = true;
					}
					if (!any_in || !any_out) continue;
					for (int t = 0; t < 6; ++t) {
						const int* c = tets[t];
						int inside[4], outside[4], ni = 0, no = 0;
						for (int q = 0; q < 4; ++q) {
							if (v[c[q]] <= 0.0f) inside[ni++] = c[q];
							else outside[no++] = c[q];
						}
						if (ni == 0 || ni == 4) continue;
						auto corner_pos = [&](int cc) {
							return g.origin + Vec3{(ci + (cc & 1)) * g.h, (cj + ((cc >> 1) & 1)) * g.h, (ck + ((cc >> 2) & 1)) * g.h};
						};
						Vec3 cin{0, 0, 0}, cout{0, 0, 0};
						for (int q = 0; q < ni; ++q) cin += corner_pos(inside[q]);
						for (int q = 0; q < no; ++q) cout += corner_pos(outside[q]);
						Vec3 outward = cout / no - cin / ni;
						auto ev = [&](int a, int b) {
							// Order so the lower corner bits come first.
							if (a < b) return edge_vertex(ci, cj, ck, a, b, v[a], v[b]);
							return edge_vertex(ci, cj, ck, b, a, v[b], v[a]);
						};
						auto mid = [&](int a, int b) { return (corner_pos(a) + corner_pos(b)) * 0.5; };
						if (ni == 1 || ni == 3) {
							int p = (ni == 1) ? inside[0] : outside[0];
							const int* others = (ni == 1) ? outside : inside;
							int e0 = ev(p, others[0]), e1 = ev(p, others[1]), e2 = ev(p, others[2]);
							emit(e0, e1, e2, mid(p, others[0]), mid(p, others[1]), mid(p, others[2]), outward);
						} else {
							int p = inside[0], q = inside[1], r = outside[0], s2 = outside[1];
							int pr = ev(p, r), ps = ev(p, s2), qs = ev(q, s2), qr = ev(q, r);
							emit(pr, ps, qs, mid(p, r), mid(p, s2), mid(q, s2), outward);
							emit(pr, qs, qr, mid(p, r), mid(q, s2), mid(q, r), outward);
						}
					}
				}
	}, threads);

	// Merge the slabs, sharing vertices on their common planes by key.
	TriMesh mesh;
	size_t total_v = 0, total_t = 0;
	for (const auto& s : slabs) { total_v += s.vertices.size(); total_t += s.triangles.size(); }
	mesh.vertices.reserve(total_v);
	mesh.triangles.reserve(total_t);
	std::unordered_map<uint64_t, int> global;
	global.reserve(total_v);
	std::vector<int> remap;
	for (const auto& s : slabs) {
		remap.assign(s.vertices.size(), -1);
		for (size_t i = 0; i < s.vertices.size(); ++i) {
			auto it = global.find(s.keys[i]);
			if (it == global.end()) {
				int idx = static_cast<int>(mesh.vertices.size());
				mesh.vertices.push_back(s.vertices[i]);
				it = global.emplace(s.keys[i], idx).first;
			}
			remap[i] = it->second;
		}
		for (const auto& t : s.triangles) mesh.triangles.push_back({remap[t[0]], remap[t[1]], remap[t[2]]});
	}
	mesh.RemoveDegenerateTriangles();
	return mesh;
}

} // namespace aerotelic
