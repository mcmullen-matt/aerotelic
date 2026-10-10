#include "aerotelic/airfoil_dat.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace aerotelic {

namespace {

std::string Trim(const std::string& s) {
	const size_t a = s.find_first_not_of(" \t\r\n");
	if (a == std::string::npos) return "";
	const size_t b = s.find_last_not_of(" \t\r\n");
	return s.substr(a, b - a + 1);
}

// Python's str.split(): whitespace-separated tokens.
std::vector<std::string> Split(const std::string& s) {
	std::vector<std::string> out;
	std::istringstream in(s);
	std::string tok;
	while (in >> tok) out.push_back(tok);
	return out;
}

// Python float(): accepts the usual decimal/exponent forms, rejects
// anything with trailing junk.
bool ParseDouble(const std::string& s, double& out) {
	if (s.empty()) return false;
	char* end = nullptr;
	out = std::strtod(s.c_str(), &end);
	return end != nullptr && *end == '\0' && end != s.c_str();
}

// The part of a line before any '#' comment, trimmed.
std::string Payload(const std::string& raw) {
	const size_t hash = raw.find('#');
	return Trim(hash == std::string::npos ? raw : raw.substr(0, hash));
}

std::vector<std::string> Lines(const std::string& text) {
	std::vector<std::string> out;
	std::string cur;
	for (char c : text) {
		if (c == '\n') { out.push_back(cur); cur.clear(); }
		else if (c != '\r') cur.push_back(c);
	}
	if (!cur.empty()) out.push_back(cur);
	return out;
}

} // namespace

std::string FirstDatNameLine(const std::string& text) {
	for (const std::string& raw : Lines(text)) {
		const std::string line = Payload(raw);
		if (line.empty()) continue;
		const std::vector<std::string> parts = Split(line);
		if (parts.size() == 2) {
			double a, b;
			if (ParseDouble(parts[0], a) && ParseDouble(parts[1], b)) continue; // coordinates
		}
		return line;
	}
	return "";
}

DatContents ParseSeligDat(const std::string& text) {
	DatContents out;
	std::vector<std::pair<double, double>> pts;
	const std::vector<std::string> lines = Lines(text);
	for (size_t i = 0; i < lines.size(); ++i) {
		const std::string line = Payload(lines[i]);
		if (line.empty()) continue;
		const std::vector<std::string> parts = Split(line);
		if (parts.size() == 2) {
			double x, y;
			if (ParseDouble(parts[0], x) && ParseDouble(parts[1], y)) {
				// Lednicer-format header: a count pair like "56.  57." in
				// the first lines, not a coordinate.
				if (i <= 3 && x > 1.5) continue;
				pts.emplace_back(x, y);
				continue;
			}
		}
		if (out.name.empty()) out.name = line;
	}
	if (pts.size() < 4) {
		throw std::invalid_argument("Too few coordinate pairs (" + std::to_string(pts.size()) + ")");
	}

	const int n = static_cast<int>(pts.size());
	Mat points(n, 2);
	for (int i = 0; i < n; ++i) { points(i, 0) = pts[i].first; points(i, 1) = pts[i].second; }

	auto argmin_x = [&points, n]() {
		int best = 0;
		for (int i = 1; i < n; ++i) if (points(i, 0) < points(best, 0)) best = i;
		return best;
	};

	const int le_idx = argmin_x();
	bool consistent = false;
	for (int pass = 0; pass < 5; ++pass) {
		// Leading edge to the origin.
		const double x_min = points(le_idx, 0);
		const double y_min = points(le_idx, 1);
		for (int i = 0; i < n; ++i) { points(i, 0) -= x_min; points(i, 1) -= y_min; }
		// Rotate so the trailing edge's midpoint lies on +x.
		const double te_x = 0.5 * (points(0, 0) + points(n - 1, 0));
		const double te_y = 0.5 * (points(0, 1) + points(n - 1, 1));
		const double theta = std::atan2(te_y, te_x);
		const double st = std::sin(theta), ct = std::cos(theta);
		for (int i = 0; i < n; ++i) {
			const double x = points(i, 0), y = points(i, 1);
			points(i, 0) = ct * x + st * y;
			points(i, 1) = -st * x + ct * y;
		}
		if (argmin_x() == le_idx) { consistent = true; break; }
	}
	if (!consistent) {
		throw std::invalid_argument("Input points cannot be appropriately rotated");
	}

	const double scale = 0.5 * (points(0, 0) + points(n - 1, 0));
	if (std::fabs(scale) < 1e-6) {
		throw std::invalid_argument("The set of points provided is degenerate - the rotated chord approaches zero");
	}
	// Scale to unit chord and reverse the order (TE -> upper -> LE -> lower
	// -> TE becomes lower TE -> LE -> upper TE).
	out.points = Mat(n, 2);
	for (int i = 0; i < n; ++i) {
		out.points(i, 0) = points(n - 1 - i, 0) / scale;
		out.points(i, 1) = points(n - 1 - i, 1) / scale;
	}
	return out;
}

std::unique_ptr<Airfoil> AirfoilFromDatText(const std::string& text, const std::string& fallback_name) {
	DatContents dat = ParseSeligDat(text);
	const std::string name = dat.name.empty() ? fallback_name : dat.name;
	return Airfoil::FromPoints(dat.points, 3, name);
}

} // namespace aerotelic
