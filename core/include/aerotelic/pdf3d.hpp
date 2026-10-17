#pragma once

// 3D PDF export: the chosen bodies as one interactive, rotatable model
// inside a PDF page.
//
// A "3D PDF" is an ordinary PDF carrying a 3D annotation (ISO 32000-1
// 13.6) whose stream holds the geometry in a format the viewer understands.
// Acrobat accepts two: PRC and U3D (ECMA-363). This module writes U3D,
// which is the simpler of the two and needs no third-party library:
//
//   * WriteU3d() turns a list of named, coloured triangle meshes into a
//     U3D byte stream -- one CLOD mesh generator holding every part, with
//     one shader and material per part so the bodies keep their scene
//     colours, plus the model node that puts it in the scene.
//
//   * WritePdf3d() wraps that stream in a one-page PDF: the heading, the
//     3D annotation sized to the page, a default view aimed at the model's
//     bounding sphere, and the caption listing what is in it.
//
// The file is written in U3D's "no compression" profile (ECMA-363
// 9.4.1.2, profile bit 0x4): where the block definitions call for a
// compressed value the uncompressed one is written instead, which every
// reader is required to support. That keeps the whole encoder to
// little-endian field writes -- no arithmetic coder, no histogram state
// shared with the reader -- at the cost of a larger stream, which the
// PDF's own Flate filter then takes back (see Pdf3dOptions::deflate).
//
// Units are the model's own, as for STL: a 3D PDF carries no unit. The
// default view assumes the application's axes (+Z up, +X aft, +Y to
// starboard -- see ue/README.md's navigation cube), which is only the
// camera the file opens with; the reader can orbit freely from there.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

// One named, coloured body of the model. The mesh is in model
// coordinates, placed as it sits in the scene (as the STL writer takes
// them), and must be triangles -- there is no B-rep in a U3D stream.
struct Pdf3dPart {
	std::string name;
	TriMesh mesh;
	// Linear RGB in 0..1, the scene colour of the body.
	double colour[3] = {0.72, 0.74, 0.78};
	// 1 is opaque. Below 1 the part is drawn blended, which is how a
	// skin over a lattice is made readable.
	double opacity = 1.0;
};

struct Pdf3dOptions {
	// Drawn at the top of the page and used as the PDF's /Title.
	std::string title;
	// Drawn under the model. Empty for none.
	std::string caption;

	// Page box in PostScript points (72 to the inch). A4 portrait.
	double page_width = 595.276;
	double page_height = 841.890;
	double margin = 36.0;

	// Background of the 3D view, and the field of view of its perspective
	// camera in degrees.
	double background[3] = {1.0, 1.0, 1.0};
	double field_of_view_deg = 30.0;

	// Where the default view looks FROM, as a direction in model
	// coordinates from the model's centre, and which way is up. Both are
	// normalised here; `up` only has to be non-parallel to `view_from`.
	Vec3 view_from{1.0, -1.0, 0.55};
	Vec3 up{0.0, 0.0, 1.0};
	// Slack around the bounding sphere when placing the camera: 1 puts
	// the sphere exactly in the field of view.
	double fit = 1.2;

	// /3DLightingScheme and /3DRenderMode subtypes (ISO 32000-1 tables
	// 320 and 319). "CAD" is the three-light rig a CAD viewer uses;
	// "Solid" is shaded without edges.
	std::string lighting = "CAD";
	std::string render_mode = "Solid";

	// Normals within this angle of each other at a shared position are
	// averaged, so a curved surface shades smoothly while a genuine
	// crease (a trailing edge, the rim of a cap) stays sharp.
	double crease_angle_deg = 30.0;

	// Optional zlib deflate, used as the /FlateDecode filter for the
	// streams. The core deliberately links no zlib of its own: the
	// application's engine already loads one, and a second copy of that
	// name in the process is the collision ue/Source/AerotelicUE/
	// AerotelicUE.Build.cs documents at length. The caller therefore
	// passes its own (the engine's FCompression for the application, none
	// for the tests) -- returning false, or being unset, writes the
	// streams uncompressed, which is equally valid and about three times
	// the size.
	std::function<bool(const std::vector<std::uint8_t>& in, std::vector<std::uint8_t>& out)> deflate;
};

// What came out, for the status line and the tests.
struct Pdf3dResult {
	std::size_t parts = 0;
	std::size_t positions = 0;   // after welding the parts' shared vertices
	std::size_t normals = 0;
	std::size_t faces = 0;
	std::size_t u3d_bytes = 0;   // the U3D stream, before any Flate
	std::size_t pdf_bytes = 0;
	bool u3d_deflated = false;
};

// The U3D (ECMA-363) stream for the parts. Empty if no part has a
// triangle. `crease_angle_deg` is Pdf3dOptions::crease_angle_deg.
std::vector<std::uint8_t> WriteU3d(const std::vector<Pdf3dPart>& parts, double crease_angle_deg, Pdf3dResult* out_stats = nullptr);

// Writes the one-page 3D PDF. False with the reason in `error` if there
// is nothing to write or the file cannot be opened.
bool WritePdf3d(const std::vector<Pdf3dPart>& parts, const std::string& path, const Pdf3dOptions& options,
	std::string& error, Pdf3dResult* out_stats = nullptr);

}  // namespace aerotelic
