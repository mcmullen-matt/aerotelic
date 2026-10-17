#pragma once

// Writes a complete DUST case from the scene's bodies.
//
// DUST (Politecnico di Milano, https://public.gitlab.polimi.it/DAER/dust)
// is a mid-fidelity unsteady aerodynamic solver: the bodies carry
// singularity elements and the wake they shed is convected as free
// vortex PARTICLES, so the wake rolls up and interacts with the
// geometry instead of being prescribed. That is the capability this
// module feeds -- a lifting, unsteady alternative to Analysis ->
// Potential Flow, which has no wake and therefore no lift at all.
//
// Nothing here runs DUST. It writes the files a DUST run consumes, and
// the dialog's Help tab gives the three commands. That keeps the
// feature useful on a machine where DUST is not installed (write the
// case here, run it on a cluster) and keeps the solver's considerable
// dependency chain -- HDF5, CGNS, LAPACK, all needing the same Fortran
// compiler -- out of this application entirely.
//
// The bodies go in as DUST's "basic" mesh format: two ASCII files per
// component, `<prefix>rr.dat` with a node per line and `<prefix>ee.dat`
// with an element per line. DUST's element array is four-wide and reads
// a zero fourth index as a triangle (mod_build_geo.f90: "if (ee(4,i)
// .eq. 0) nsides1 = 3"), which is exactly what the loft triangulation
// produces. The alternative, CGNS, would mean linking the CGNS library
// into this application for no gain.
//
// Element type is DUST's `p`, surface panels: a constant source and
// doublet on each panel, with the wake shed from the trailing edge that
// DUST detects from the surface itself. The other element types are not
// offered because they cannot be: `v` (vortex lattice) and `l`
// (lifting line) need a structured chordwise/spanwise mesh, which DUST
// only builds from its own `parametric` and `pointwise` inputs, not
// from a triangle mesh.

#include <cstddef>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

// One body of the case: a triangle mesh and the name it will carry
// through DUST's input files, its results and its load tables.
struct DustComponent {
	std::string name;
	// In the coordinates of `ref_tag`'s frame. For a body on the global
	// frame that means world coordinates (what the scene shows); for a
	// body on a rotor frame it means HUB-LOCAL, because the frame itself
	// carries the placement and applying it twice would fly the
	// propeller off into space.
	TriMesh mesh;
	// The reference frame the body rides on. Empty is DUST's frame 0, the
	// global one, which needs no definition -- a body standing still.
	std::string ref_tag;
};

// A DUST reference frame. Written only when something rides on it.
//
// A rotor (n_blades > 0) uses DUST's own rotor multiplicity: ONE blade
// is handed over and DUST replicates it n_blades times around the axis
// and spins the lot at rot_rate. That is what its post-processing is
// built around -- per-blade azimuth, and the hinge/collective/cyclic
// machinery if it is ever wanted -- rather than handing DUST the blades
// already laid out and merely turning the whole assembly.
struct DustFrame {
	std::string tag;
	std::string parent = "0";
	// Origin in the parent's coordinates, and the frame's x, y and z axes
	// in the parent's components. DUST reads the nine numbers with a
	// Fortran reshape((/.../),(/3,3/)), which fills COLUMN BY COLUMN, so
	// they go out as x-axis, then y-axis, then z-axis -- the transpose of
	// the row-major spelling the same matrix would have in C.
	Vec3 origin{};
	Vec3 x_axis{1.0, 0.0, 0.0};
	Vec3 y_axis{0.0, 1.0, 0.0};
	Vec3 z_axis{0.0, 0.0, 1.0};

	// ---- rotor multiplicity; zero blades means a plain static frame ----
	int n_blades = 0;
	// In the frame's own components. Aerotelic's propellers spin about
	// their local +x.
	Vec3 rot_axis{1.0, 0.0, 0.0};
	// Radians per second, SIGNED: a mirrored propeller turns the other
	// way and gets a negative rate (its geometry is reflected, and a
	// reflection cannot be an orientation matrix).
	double rot_rate = 0.0;
	double psi_0_deg = 0.0;
	double hub_offset = 0.0;
};

struct DustCaseOptions {
	// ---- free stream ----
	// The flow runs along +x, tilted towards +z by the angle of attack
	// and towards +y by the sideslip -- the same convention Analysis ->
	// Potential Flow uses, and the application's own axes (+Z up,
	// +X aft, +Y to starboard).
	double speed = 30.0;
	double alpha_deg = 0.0;
	double beta_deg = 0.0;
	double rho_inf = 1.225;
	double p_inf = 101325.0;
	double a_inf = 340.0;
	double mu_inf = 1.81e-5;   // only used when diffusion is on

	// ---- time ----
	// Zero means "derive it": the reference length is the model's extent
	// ALONG THE FLOW (the chord of a wing, the length of a fuselage),
	// dt is the time to cross it divided by steps_per_length, and tend
	// is the time to cross it `crossings` times -- long enough for the
	// wake to establish.
	double tstart = 0.0;
	double tend = 0.0;
	double dt = 0.0;
	double dt_out = 0.0;
	double crossings = 8.0;
	double steps_per_length = 40.0;
	// With a rotor in the case the step has to resolve the ROTATION as
	// well as the flow, and the two have nothing to do with each other: a
	// propeller on a slow-moving aircraft turns many times while the flow
	// crosses it once. The derived step is therefore the smaller of the
	// flow-derived one and the one that sweeps a blade this far per step.
	// Zero ignores rotors entirely.
	double degrees_per_step = 5.0;

	// ---- wake and particles ----
	// n_wake_panels is how many rows of panel wake are kept attached at
	// the trailing edge before the wake is turned into particles; one is
	// DUST's own default and is what makes the run a particle method.
	int n_wake_panels = 1;
	int n_wake_particles = 500000;
	// Particle core radius. Zero means "derive it": the mean panel size,
	// so a particle is about as big as the panel that shed it.
	double vortex_rad = 0.0;
	// The box particles are allowed to live in, as multiples of the
	// model's bounding-box diagonal: `box_margin` to every side, plus
	// `box_wake` more downstream so the wake has somewhere to go.
	double box_margin = 1.0;
	double box_wake = 4.0;

	// ---- models ----
	bool fmm = true;
	bool vortstretch = true;
	bool diffusion = true;
	bool penetration_avoidance = true;

	// ---- octree, used by the fast multipole method ----
	int n_octree_levels = 5;
	int min_octree_part = 5;
	int multipole_degree = 2;

	// ---- geometry ----
	// Gap below which DUST WELDS NODES TOGETHER when closing the trailing
	// edge, and the threshold on the inner product of the normals of
	// elements facing each other across it.
	//
	// The sewing tolerance has to stay below the SHORTEST EDGE in the
	// mesh. DUST merges any two nodes closer than it before it builds
	// the element connectivity, so a tolerance larger than a real edge
	// silently collapses real triangles and the connectivity comes out
	// inconsistent -- which DUST reports, confusingly, as "Neighbouring
	// elements with opposed normal orientation". Zero means "derive it",
	// as a small fraction of the shortest edge. Deriving it from the
	// MEAN panel size instead looks reasonable and is wrong: a wing's
	// panels vary by more than two orders of magnitude between mid-span
	// and the tip, so a fraction of the mean sits well above the
	// smallest genuine edges.
	//
	// Aerotelic's loft shares its trailing-edge vertices exactly when the
	// section is closed, so in practice there is nothing to sew at all
	// and the tolerance only has to absorb float noise.
	double tol_se_wing = 0.0;
	double inner_product_te = -0.5;

	// ---- output ----
	// Stem of the result files, under Output/ in the case directory.
	std::string basename = "dust";
	bool write_postprocessing = true;
};

// What was written, and every number the writer derived -- the dialog
// shows these so the user sees what the blank fields turned into.
struct DustCaseResult {
	std::vector<std::string> files;   // relative to the case directory
	std::size_t components = 0;
	// Panels DUST will actually solve: a blade on a rotor frame counts
	// once per blade, because DUST replicates it. The dense-system
	// estimate below depends on getting that right.
	std::size_t panels = 0;
	std::size_t nodes = 0;
	std::size_t rotors = 0;

	double reference_length = 0.0;    // model extent along the flow
	Vec3 u_inf{};
	double dt = 0.0, dt_out = 0.0, tend = 0.0;
	int timesteps = 0;
	// How far the fastest blade sweeps in one step, and how many steps a
	// revolution takes. Zero when there is no rotor.
	double degrees_per_step = 0.0;
	double steps_per_revolution = 0.0;
	// True when the rotor, not the flow, is what set the step.
	bool dt_from_rotor = false;
	double vortex_rad = 0.0;
	double tol_se_wing = 0.0;
	double mean_panel_size = 0.0;
	// The shortest edge in the exported mesh. The sewing tolerance above
	// must stay below it; the dialog shows both so the margin is visible.
	double min_edge = 0.0;

	Vec3 box_min{}, box_max{};
	double octree_box_length = 0.0;
	int n_box[3] = {1, 1, 1};
	Vec3 octree_origin{};

	// DUST's panel method builds a DENSE influence matrix over the
	// panels, so memory goes as the square of the panel count. This is
	// the number that decides whether a case is runnable, which is why
	// the dialog puts it next to the surface deflection.
	double linear_system_mb = 0.0;

	// Panels of DIFFERENT bodies lying on top of one another, and the
	// first two bodies found doing it. Two surfaces in the same place
	// give the panel method two unknowns with identical influence, so
	// the matrix is singular and the solution diverges on the very first
	// step -- it is not a wake instability that settles down. The
	// everyday way to produce it here is to pick a wing AND its mirror:
	// each is a closed body, and their root caps are the same wall seen
	// from both sides. Fusing them with a boolean first removes the
	// internal wall and the degeneracy with it. Reported rather than
	// refused: a user may be deliberately stacking surfaces.
	std::size_t coincident_panels = 0;
	std::string coincident_a, coincident_b;
};

// Works out everything the case needs -- the step, the run length, the
// particle box, the octree, the panel count, the overlap warning -- and
// writes nothing. The dialog calls this on every change so it can show
// what the blank fields will turn into before anything is committed to
// disk. False with the reason in `error`.
bool DeriveDustCase(const std::vector<DustComponent>& components, const std::vector<DustFrame>& frames,
	const DustCaseOptions& options, DustCaseResult& result, std::string& error);

// Derives as above, then writes the case into `directory` (created if
// missing, along with its components/, geometry/, Output/ and
// Postprocessing/ subdirectories). False with the reason in `error`.
bool WriteDustCase(const std::vector<DustComponent>& components, const std::vector<DustFrame>& frames,
	const std::string& directory, const DustCaseOptions& options, DustCaseResult& result, std::string& error);

// The three commands that run the case, one per line, as the dialog's
// Help tab and the case's own README show them.
std::string DustRunCommands();

}  // namespace aerotelic
