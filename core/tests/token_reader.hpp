#pragma once

// Tiny whitespace-token reader shared by the fixture-check test programs.
// Fixture files are plain whitespace-separated tokens (see the generate_*
// scripts in tools/) -- no quoting needed since they're all keywords and
// numbers, so this avoids pulling in a JSON dependency just for tests.

#include <istream>
#include <stdexcept>
#include <string>
#include <vector>

#include "aerotelic/mat.hpp"

namespace aerotelic {

class TokenReader {
public:
	explicit TokenReader(std::istream& in) : in_(in) {}

	bool HasNext() { return bool(in_ >> std::ws) && in_.peek() != EOF; }

	std::string NextTok() {
		std::string t;
		if (!(in_ >> t)) throw std::runtime_error("unexpected end of fixture file");
		return t;
	}

	void Expect(const std::string& kw) {
		std::string t = NextTok();
		if (t != kw) throw std::runtime_error("expected '" + kw + "' got '" + t + "'");
	}

	int NextInt() { return std::stoi(NextTok()); }
	double NextDouble() { return std::stod(NextTok()); }

	std::vector<double> NextDoubles(int n) {
		std::vector<double> v(n);
		for (int i = 0; i < n; ++i) v[i] = NextDouble();
		return v;
	}

	Mat NextMat(int rows, int cols) {
		Mat m(rows, cols);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) m(i, j) = NextDouble();
		}
		return m;
	}

private:
	std::istream& in_;
};

} // namespace aerotelic
