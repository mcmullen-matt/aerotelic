#pragma once

// A catalogue of colour maps for scalar fields (a pressure coefficient over
// a surface, a stress, a Mach number): a name and evenly spaced RGB stops
// on [0, 1], sampled by linear interpolation. Toolkit-free so the UE
// colour bar, a VTK writer and a test can all read the same tables.
//
// Sources, all free to reuse: viridis, plasma, inferno, magma, cividis and
// twilight are matplotlib's (CC0 / matplotlib licence), turbo is Google's
// (Apache 2.0), cool-warm is Moreland's diverging map (public domain),
// Spectral, Red-Blue, Red-Yellow-Blue, Blues and Yellow-Orange-Red are
// ColorBrewer's (Apache 2.0), cubehelix is D. A. Green's formula (2011),
// jet, rainbow, hot, cool and grey are the classic formulae.

#include <string>
#include <vector>

namespace aerotelic {

struct Rgb {
	float r = 0.0f, g = 0.0f, b = 0.0f;
};

struct Colormap {
	std::string name;
	// Evenly spaced on [0, 1], at least two.
	std::vector<Rgb> stops;
	// Is the map a diverging one (a meaningful middle, as cool-warm has)?
	// A hint for a default range centred on zero; no effect on sampling.
	bool diverging = false;

	// The colour at t in [0, 1] (clamped), linearly interpolated.
	Rgb Sample(double t) const;
};

// The catalogue, in the order a menu should offer it. Built once.
const std::vector<Colormap>& Colormaps();

// By name (case-sensitive), or null.
const Colormap* FindColormap(const std::string& name);

// The default for a pressure field.
const Colormap& DefaultColormap();

} // namespace aerotelic
