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

enum class LatticeFamily { Tpms, Strut, Auxetic };
enum class TpmsPhase { Network, Sheet, IsoSurface };

const char* LatticeFamilyName(LatticeFamily f);
LatticeFamily ParseLatticeFamily(const std::string& s);
const char* TpmsPhaseName(TpmsPhase p);
TpmsPhase ParseTpmsPhase(const std::string& s);

// The lattice types of each family, in the paper's order.
const std::vector<std::string>& TpmsTypeNames();
const std::vector<std::string>& StrutTypeNames();
const std::vector<std::string>& AuxeticTypeNames();
const std::vector<std::string>& LatticeTypeNames(LatticeFamily f);

struct LatticeSpec {
	LatticeFamily family = LatticeFamily::Tpms;
	// A name from LatticeTypeNames(family).
	std::string type = "Gyroid";
	TpmsPhase phase = TpmsPhase::Network;
	// Cubic unit cell size l, model units.
	double cell_size = 0.5;
	// Target relative density rho* (0, 1); Network/Sheet phases and struts.
	double volume_fraction = 0.3;
	// Wall thickness for the IsoSurface phase, model units.
	double wall_thickness = 0.02;
	// Voxels per unit cell along each axis (the paper's mesh density).
	int resolution = 12;
	// A solid shell of this thickness kept just inside the body surface
	// (0 = none).
	double skin_thickness = 0.0;
	// The lattice is periodic about this point.
	Vec3 origin{0.0, 0.0, 0.0};
};

// TPMS level-set value U at phase coordinates (X, Y, Z) = k * (x, y, z).
double TpmsFunction(const std::string& type, double X, double Y, double Z);

// A strut-based cell: nodes in [0, 1)^3 and struts as node index pairs.
struct StrutCell {
	std::vector<Vec3> nodes;
	std::vector<std::array<int, 2>> struts;
};
const StrutCell& StrutCellFor(const std::string& type);

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
	// The calibrated level-set parameter t (TPMS) or relative strut radius
	// R (struts, in cell units).
	double level() const { return level_; }
	const LatticeSpec& spec() const { return spec_; }

private:
	LatticeSpec spec_;
	double level_ = 0.0;
	double k_ = 1.0;            // 2 pi / cell
	double gradient_scale_ = 1.0;
	const StrutCell* cell_ = nullptr;
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

// The grid GenerateLattice would use (so a dialog can show the cost).
LatticeGrid PlanLatticeGrid(const LatticeSpec& spec, const Box3& bounds);

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
