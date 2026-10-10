#pragma once

// The C++ twin of core/reference_image.py's ReferenceImage *state*: a
// drawing placed behind a curve editor so the user can trace it. The
// geometry backend never reads it; it lives on the model only so that it
// is saved with the project (`reference_images:` keyed by view) and loaded
// back. Loading the pixels, drawing and dragging are the UI's business.
//
// Units are the editor's own non-dimensional ones -- x/L and y/L (x/R, r/R
// on a propeller): `scale` is display units per source pixel, (x, y) the
// top-left corner of the (flipped) source. Changing the entity's reference
// length moves neither the image nor the control points (REQ-FUTURE-027).

#include <array>
#include <map>
#include <string>

namespace aerotelic {

struct ReferenceImageSpec {
	std::string path;                       // as saved: relative to the project file when beside it, else absolute
	double scale = 0.0025;                  // display units per source pixel
	double x = 0.0;                         // top-left of the (flipped) source, display units
	double y = 0.0;
	std::array<double, 4> crop = {0.0, 0.0, 1.0, 1.0}; // left, top, right, bottom as source fractions
	double opacity = 0.5;
	bool flip_h = false;
	bool flip_v = false;
	double angle = 0.0;                     // degrees counter-clockwise about the whole source's centre
};

// A crop may not shrink either side below this fraction of the source.
constexpr double kReferenceImageMinCropFraction = 0.01;

// One entry per curve editor / tab of a panel: "profile", "width",
// "planform", "dihedral", "side", "front", "centreline", "section",
// "skins", "spinner".
using ReferenceImageMap = std::map<std::string, ReferenceImageSpec>;

} // namespace aerotelic
