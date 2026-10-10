#pragma once

// Port of core/selig_db.py's parsing helpers: reading a Selig-format
// airfoil coordinate file (.dat) into the point list Airfoil::FromPoints
// takes. The download / cache / index layer lives in the application.

#include <memory>
#include <string>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/mat.hpp"

namespace aerotelic {

struct DatContents {
	// The first non-coordinate line (the file's display name); empty when
	// the file has none.
	std::string name;
	// (N, 2) points ordered lower TE -> LE -> upper TE, normalised: the
	// leading edge at the origin, the trailing edge's midpoint on +x, the
	// chord scaled to 1.
	Mat points;
};

// _parse_selig_dat: skips '#' comments; a two-number line among the first
// four whose first value exceeds 1.5 is a Lednicer count header and is
// skipped; non-numeric lines supply the name. Then up to five passes of
// translate-the-LE-to-the-origin / rotate-the-TE-onto-+x until the LE
// index is stable, a chord scale, and a reversal of the order (Selig
// files run TE -> upper -> LE -> lower -> TE).
// Throws std::invalid_argument for fewer than four pairs, an unstable
// rotation, or a degenerate chord.
DatContents ParseSeligDat(const std::string& text);

// _first_name_line: the first non-blank, non-coordinate line, or "".
std::string FirstDatNameLine(const std::string& text);

// load_airfoil's tail: parse, then fit; the name falls back to
// `fallback_name` (the file stem) when the file has none.
std::unique_ptr<Airfoil> AirfoilFromDatText(const std::string& text, const std::string& fallback_name);

} // namespace aerotelic
