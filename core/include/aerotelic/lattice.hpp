#pragma once

// Lattice infill after OpenLattice (Gayir, Ozdilek, Ozdemir, Simsek,
// Sendur, Kiziltas, "OpenLattice: A Research-Oriented Platform for
// Accelerating Design-to-Manufacturing Workflow of Diverse Lattice
// Architectures for Additive Manufacturing", SSRN 6210543), section 2:
//
//   * TPMS lattices (2.1): a level-set function U(x, y, z) built from
//     sines and cosines of k_i * x_i, with k_i = 2 pi / l for a cubic unit
//     cell of size l (the paper's n_i / l, with the cell count folded into
//     the domain here). The 27 implicit formulas of the paper's Table 3
//     (Appendix) are reproduced in TpmsFunction(). The solid is one of
//     the paper's three phases, selected by the level-set parameter t:
//       Network phase  -- U <= t        (one side of the iso-surface)
//       Matrix (sheet) -- |U| <= t      (the iso-surface thickened, t^2 != 0)
//       Iso-surface    -- the t = 0 surface itself, given a physical wall
//                         thickness (a zero-thickness surface cannot be
//                         printed).
//     As the paper notes, t has a unique relation to the volume fraction
//     rho* of each lattice; CalibrateLevel() finds it numerically by
//     bisection on a sampled unit cell rather than from a fitted formula,
//     so every type and phase calibrates the same way.
//
//   * Strut-based and auxetic lattices (2.2): the paper's node-strut
//     tables turned into an implicit, periodic distance field. Every
//     strut is stored by its two end nodes in the normalised cell
//     [0, 1)^3; for a query point the closest periodic copy of each strut
//     is found by rounding (eq. 2-3), the distance to the segment taken
//     (eq. 4-8), normalised by the strut radius and the minimum over all
//     struts taken (eq. 9). The surface is the iso-surface f = 1, i.e.
//     the union of all strut cylinders; the radius follows from the
//     volume fraction by the same calibration. The nine built-in cells
//     are the paper's Fig. 4: BCC, FCC, G7, Grid, Octet, Tesseract,
//     Vintiles, X-lattice and the Re-entrant (auxetic) cell. (The paper
//     gives the pictures, not the node tables; the tables in lattice.cpp
//     follow the standard definitions of these cells and are plain data,
//     so a different definition is a table edit.)
//
//   * Honeycomb (prismatic) cells, which the paper does not cover: a 2D
//     tiling's cell walls extruded along one axis -- the sandwich core of
//     aerospace practice, and the usual print infill. Square, Rectangle,
//     Hexagon and Triangle; the wall thickness is either calibrated to
//     the volume fraction like everything else or given outright (the
//     manufacturing knob: a wall is printed in whole bead widths). Unlike
//     the TPMS and strut families these cells are anisotropic by
//     construction -- stiff along the prism axis, soft across it -- so
//     the axis is part of the spec.
//
// The lattice fills a body: the generator (GenerateLattice) samples the
// lattice field intersected with the body's signed distance field (and
// minus any reserved volumes) on a regular grid at a chosen number of
// voxels per cell -- the paper's "mesh density" -- and extracts the
// zero-crossing surface with marching tetrahedra, giving a closed
// triangle mesh ready for STL. Like OpenLattice, the output is a surface
// mesh; the body's own surface is reproduced wherever the lattice meets
// it, and an optional solid skin keeps the outer shell intact.

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

enum class LatticeFamily { Tpms, Strut, Auxetic, Honeycomb };
enum class TpmsPhase { Network, Sheet, IsoSurface };

// Which axis a prismatic (honeycomb) cell is extruded along: the cell
// walls run parallel to it and the tiling lies in the other two axes.
enum class PrismAxis { X, Y, Z };

// How a honeycomb's wall thickness is decided.
enum class WallSizing {
	VolumeFraction, // calibrated to LatticeSpec::volume_fraction
	Thickness,      // LatticeSpec::wall_thickness as it stands
};

const char* LatticeFamilyName(LatticeFamily f);
LatticeFamily ParseLatticeFamily(const std::string& s);
const char* TpmsPhaseName(TpmsPhase p);
TpmsPhase ParseTpmsPhase(const std::string& s);
const char* PrismAxisName(PrismAxis a);
PrismAxis ParsePrismAxis(const std::string& s);
const char* WallSizingName(WallSizing w);
WallSizing ParseWallSizing(const std::string& s);

// The lattice types of each family, in the paper's order.
const std::vector<std::string>& TpmsTypeNames();
const std::vector<std::string>& StrutTypeNames();
const std::vector<std::string>& AuxeticTypeNames();
const std::vector<std::string>& HoneycombTypeNames();
const std::vector<std::string>& LatticeTypeNames(LatticeFamily f);

struct LatticeSpec {
	LatticeFamily family = LatticeFamily::Tpms;
	// A name from LatticeTypeNames(family).
	std::string type = "Gyroid";
	TpmsPhase phase = TpmsPhase::Network;
	// Cubic unit cell size l, model units. For a honeycomb it is the
	// cross-section's first dimension: the side of a Square or Triangle,
	// the width of a Rectangle, a Hexagon's size across the flats.
	double cell_size = 0.5;
	// Target relative density rho* (0, 1); Network/Sheet phases, struts,
	// and honeycombs sized by volume fraction.
	double volume_fraction = 0.3;
	// Wall thickness for the IsoSurface phase and for a honeycomb sized by
	// thickness, model units.
	double wall_thickness = 0.02;
	// Voxels per unit cell along each axis (the paper's mesh density).
	int resolution = 12;
	// A solid shell of this thickness kept just inside the body surface
	// (0 = none).
	double skin_thickness = 0.0;
	// The lattice is periodic about this point.
	Vec3 origin{0.0, 0.0, 0.0};

	// ---- honeycomb family only ----
	// The cross-section's second dimension as a multiple of cell_size: a
	// Rectangle's height, a Hexagon's or Triangle's stretch across the
	// flats. 1 is the regular cell; Square ignores it.
	double cell_aspect = 1.0;
	// The axis the prisms run along.
	PrismAxis axis = PrismAxis::Z;
	// Whether the wall thickness comes from volume_fraction or
	// wall_thickness.
	WallSizing wall_sizing = WallSizing::VolumeFraction;
};

// TPMS level-set value U at phase coordinates (X, Y, Z) = k * (x, y, z).
double TpmsFunction(const std::string& type, double X, double Y, double Z);

// A strut-based cell: nodes in [0, 1)^3 and struts as node index pairs.
struct StrutCell {
	std::vector<Vec3> nodes;
	std::vector<std::array<int, 2>> struts;
};
const StrutCell& StrutCellFor(const std::string& type);

// A honeycomb's cross-section: the 2D tiling whose cell walls the lattice
// is made of, in the plane of the two axes other than LatticeSpec::axis.
// The tiling is periodic on the lattice vectors a and b (model units) and
// the wall network is its cell edges, so the solid is the set of points
// within half the wall thickness of that network.
struct HoneycombCell {
	double a[2] = {1.0, 0.0};
	double b[2] = {0.0, 1.0};
	// Square, Rectangle and Triangle: the walls are `families` sets of
	// parallel lines, set f being the lines normal[f] . p = k * spacing[f]
	// for integer k. (Two orthogonal sets tile rectangles; three at sixty
	// degrees tile triangles.)
	int families = 0;
	double normal[3][2] = {};
	double spacing[3] = {};
	// Hexagon: the walls are the edges of the lattice points' Voronoi
	// cells, which are not whole lines, so the distance goes to the
	// bisectors of the nearest lattice point's neighbours instead.
	bool voronoi = false;

	// Distance from the cross-section point (u, v) to the nearest wall
	// centre-line, model units. Exact (the fields above are planes and
	// bisectors), so the lattice field is a true signed distance.
	double Distance(double u, double v) const;
	// Area of one period of the tiling, |a x b|.
	double TileArea() const;
	// The largest distance from a wall inside a cell -- the cell's
	// inradius, i.e. the half thickness at which the tiling goes solid.
	double Inradius() const;
};

// The cross-section of a honeycomb spec. Throws std::invalid_argument on
// an unknown type or a cell size or aspect that is not positive.
HoneycombCell HoneycombCellFor(const LatticeSpec& spec);

// The wall thickness a honeycomb spec asks for: wall_thickness as it
// stands, or the thickness whose walls fill `volume_fraction` of the cell.
double HoneycombWallThickness(const LatticeSpec& spec, int samples = 64);

// (u, v) of a point in the cross-section plane of `axis`, and the axial
// coordinate the cell ignores. The plane's axes are taken cyclically
// (Z -> (x, y), X -> (y, z), Y -> (z, x)) so the frame stays right-handed.
void PrismCoords(PrismAxis axis, const Vec3& p, double* u, double* v, double* w = nullptr);

// The lattice as an implicit field in model units: F(p) <= 0 inside the
// lattice material. Calibrates the level-set parameter (or strut radius)
// to the spec's volume fraction on construction. Thread-safe to evaluate.
class LatticeField {
public:
	explicit LatticeField(const LatticeSpec& spec);
	// Approximately a signed distance (model units): the level-set value
	// scaled by the field's mean gradient, or the exact distance to the
	// nearest strut surface.
	double operator()(const Vec3& p) const;
	// The calibrated level-set parameter t (TPMS), relative strut radius R
	// (struts, in cell units) or half wall thickness (honeycomb, model
	// units).
	double level() const { return level_; }
	const LatticeSpec& spec() const { return spec_; }

private:
	LatticeSpec spec_;
	double level_ = 0.0;
	double k_ = 1.0;            // 2 pi / cell
	double gradient_scale_ = 1.0;
	const StrutCell* cell_ = nullptr;
	HoneycombCell comb_;
	// Struts split into pieces no longer than half a cell, so the
	// nearest-copy rounding always finds the true closest copy near the
	// strut surface (see lattice.cpp).
	std::vector<std::array<Vec3, 2>> pieces_;
	std::vector<Vec3> piece_mid_;

	double Raw(const Vec3& p) const; // U - t style value, unscaled
};

// Finds t (TPMS, by phase) or R (struts) so that the unit cell's solid
// fraction equals `volume_fraction`, sampling the cell at
// samples^3 points. Also returns the mean gradient magnitude of U over the
// cell (TPMS), used to express the level-set field as a distance.
double CalibrateLevel(const LatticeSpec& spec, double* mean_gradient = nullptr, int samples = 40);

// Solid fraction of the unit cell at a given level (the inverse of the
// above; for tests and the dialog's read-out).
double CellSolidFraction(const LatticeSpec& spec, double level, int samples = 40);

struct LatticeDomain {
	// Signed distance to the body (or bodies) to fill, negative inside.
	// Must be 1-Lipschitz (a true distance, or the min of several), which
	// is what lets the generator skip whole blocks of voxels far from the
	// surface.
	std::function<double(const Vec3&)> body;
	// Bounds of the body; the grid covers these plus one cell.
	Box3 bounds;
	// Signed distance to the reserved volumes the lattice must leave
	// alone (negative inside them); null if none.
	std::function<double(const Vec3&)> exclusion;
};

struct LatticeGrid {
	int nx = 0, ny = 0, nz = 0; // cells
	double h = 0.0;              // voxel size
	Vec3 origin;                 // of node (0, 0, 0)
	size_t Nodes() const { return static_cast<size_t>(nx + 1) * (ny + 1) * (nz + 1); }
	size_t Cells() const { return static_cast<size_t>(nx) * ny * nz; }
};

// The voxel size the grid below is built on: a cell divided by the
// resolution. A honeycomb's two cross-section dimensions differ, so the
// smaller one sets the voxel and the wider one just gets more of them.
double LatticeVoxelSize(const LatticeSpec& spec);

// The grid GenerateLattice would use (so a dialog can show the cost).
LatticeGrid PlanLatticeGrid(const LatticeSpec& spec, const Box3& bounds);

// Empty if `grid` can carry the spec's thinnest feature; otherwise why it
// cannot and the resolution that would. A wall thinner than one voxel can
// fall between two samples with neither inside it, and is then missing
// from the extracted surface altogether -- which no care downstream
// recovers. Only the honeycomb family and the TPMS iso-surface phase have
// a thickness set independently of the cell, so only those can say no.
// GenerateLattice throws this text rather than returning such a mesh; a
// dialog can show it while the numbers are still being chosen.
std::string LatticeResolutionWarning(const LatticeSpec& spec, const LatticeGrid& grid);

struct LatticeResult {
	// The lattice: the lattice field inside the body (inside the body's
	// inner offset by the skin thickness when there is a skin).
	TriMesh mesh;
	// The solid skin: the shell between the body surface and its inward
	// offset by spec.skin_thickness, as a closed body of its own. Empty
	// when the spec has no skin. Kept apart from `mesh` (the two share the
	// offset surface exactly, being extracted from the same samples) so a
	// downstream analysis can give the shell and the infill different
	// materials; the union of the two is the printed part.
	TriMesh skin;
	// Solid fraction actually achieved inside the body (skin included),
	// from the samples.
	double achieved_fraction = 0.0;
	LatticeGrid grid;
	size_t nodes_evaluated = 0; // body SDF evaluations actually performed
	bool cancelled = false;
};

// Generates the lattice mesh. `progress(fraction)` is called from the
// calling thread between phases and may return false to cancel. Throws
// std::invalid_argument on an unusable spec (empty bounds, bad type) and
// std::runtime_error if the grid would exceed `max_nodes` samples.
LatticeResult GenerateLattice(const LatticeSpec& spec, const LatticeDomain& domain,
	const std::function<bool(double)>& progress = nullptr, int threads = 0, size_t max_nodes = 150000000);

// Marching tetrahedra over a regular grid of samples (node (i, j, k) at
// origin + h * (i, j, k); value(i, j, k) <= 0 inside), producing a closed,
// outward-oriented, vertex-shared mesh. Exposed for tests.
TriMesh MarchingTetrahedra(const LatticeGrid& grid, const std::vector<float>& values, int threads = 0);

} // namespace aerotelic
