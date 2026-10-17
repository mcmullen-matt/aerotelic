// Checks the DUST case writer (dust_case.hpp): that every file a run
// needs is produced, that the mesh files say what DUST's "basic" reader
// expects (nodes three to a line, elements four to a line with a zero
// fourth index for a triangle), that the derived time step, particle box
// and octree are self-consistent, and that the numbers the dialog shows
// are the numbers written into the files.
//
// It cannot check that DUST LIKES the case -- only DUST can say that.
// It therefore writes build/dust_case/ for exactly that purpose: point a
// real dust_pre at it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/dust_case.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;
void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

std::string Slurp(const std::filesystem::path& p) {
	std::ifstream f(p, std::ios::binary);
	std::ostringstream ss;
	ss << f.rdbuf();
	return ss.str();
}

bool Has(const std::string& haystack, const std::string& needle) {
	return haystack.find(needle) != std::string::npos;
}

// The value of "key = ..." in a DUST input file, trimmed.
std::string Value(const std::string& text, const std::string& key) {
	std::istringstream in(text);
	std::string line;
	while (std::getline(in, line)) {
		const std::size_t bang = line.find('!');
		if (bang != std::string::npos) line = line.substr(0, bang);
		const std::size_t eq = line.find('=');
		if (eq == std::string::npos) continue;
		std::string k = line.substr(0, eq);
		k.erase(0, k.find_first_not_of(" \t"));
		k.erase(k.find_last_not_of(" \t\r") + 1);
		if (k != key) continue;
		std::string v = line.substr(eq + 1);
		v.erase(0, v.find_first_not_of(" \t"));
		v.erase(v.find_last_not_of(" \t\r") + 1);
		return v;
	}
	return std::string();
}

std::size_t CountLines(const std::string& s) {
	if (s.empty()) return 0;
	std::size_t n = 0;
	for (char c : s) {
		if (c == '\n') ++n;
	}
	return n;
}

TriMesh WingMesh(double tol) {
	PlanformSpec ps;
	WingModel wing(ps, "Wing");
	wing.AddSection(Airfoil::Naca4("2412"), 0.0);
	wing.AddSection(Airfoil::Naca4("0012"), 1.0);
	TriMesh out;
	for (const NamedMesh& m : wing.BuildMeshes(tol)) out.Append(m.mesh);
	return out;
}

}  // namespace

int main() {
	namespace fs = std::filesystem;
	const fs::path dir = "build/dust_case";
	std::error_code ec;
	fs::remove_all(dir, ec);

	std::vector<DustComponent> comps(2);
	comps[0].name = "Wing/Main";       // the label the body picker hands over
	comps[0].mesh = WingMesh(0.01);
	comps[1].name = "Wing-mirror/Main";
	comps[1].mesh = comps[0].mesh;
	comps[1].mesh.Mirror({0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
	Check(!comps[0].mesh.IsEmpty(), "a wing meshes");

	DustCaseOptions opt;
	opt.speed = 40.0;
	opt.alpha_deg = 5.0;
	opt.basename = "wing";

	DustCaseResult res;
	std::string error;
	const bool ok = WriteDustCase(comps, {}, dir.string(), opt, res, error);
	Check(ok, "case written" + (ok ? std::string() : " -- " + error));
	if (!ok) {
		std::printf("\nFAILURES (1 failure)\n");
		return 1;
	}

	// ---- the files a run needs ----------------------------------------

	for (const char* f : {"dust_pre.in", "dust.in", "dust_post.in", "References.in", "README.txt"}) {
		Check(fs::exists(dir / f), std::string("exists: ") + f);
	}
	for (const char* d : {"components", "geometry", "Output", "Postprocessing"}) {
		Check(fs::is_directory(dir / d), std::string("directory exists (DUST will not make it): ") + d);
	}
	Check(res.components == 2, "two components reported");

	// ---- the "basic" mesh files ---------------------------------------

	const std::string rr = Slurp(dir / "geometry" / "Wing_Main_rr.dat");
	const std::string ee = Slurp(dir / "geometry" / "Wing_Main_ee.dat");
	Check(!rr.empty() && !ee.empty(), "the first component's rr/ee files are there and non-empty");
	Check(CountLines(rr) == comps[0].mesh.vertices.size(), "one line per node in rr.dat");
	Check(CountLines(ee) == comps[0].mesh.triangles.size(), "one line per element in ee.dat");
	{
		// Every element line must be four integers ending in 0 (DUST's
		// spelling of a triangle), with 1-based indices inside the node
		// array -- a 0-based index would silently shift the whole mesh.
		std::istringstream in(ee);
		std::string line;
		int bad_count = 0, bad_quad = 0, bad_range = 0;
		const int n_nodes = static_cast<int>(comps[0].mesh.vertices.size());
		while (std::getline(in, line)) {
			std::istringstream ls(line);
			int a, b, c, d, extra;
			if (!(ls >> a >> b >> c >> d)) { ++bad_count; continue; }
			if (ls >> extra) ++bad_count;
			if (d != 0) ++bad_quad;
			if (a < 1 || b < 1 || c < 1 || a > n_nodes || b > n_nodes || c > n_nodes) ++bad_range;
		}
		Check(bad_count == 0, "every ee.dat line is exactly four integers");
		Check(bad_quad == 0, "every element's fourth index is 0, which is how DUST spells a triangle");
		Check(bad_range == 0, "every node index is 1-based and inside the node array");
	}
	{
		std::istringstream in(rr);
		std::string line;
		int bad = 0;
		while (std::getline(in, line)) {
			std::istringstream ls(line);
			double x, y, z, extra;
			if (!(ls >> x >> y >> z)) ++bad;
			else if (ls >> extra) ++bad;
		}
		Check(bad == 0, "every rr.dat line is exactly three reals");
	}

	// Two bodies sharing a label must not land on the same files.
	Check(fs::exists(dir / "geometry" / "Wing-mirror_Main_ee.dat"), "the second component gets its own mesh files");

	// ---- dust_pre.in ----------------------------------------------------

	{
		const std::string pre = Slurp(dir / "dust_pre.in");
		Check(Has(pre, "comp_name = Wing_Main"), "dust_pre.in names the first component");
		Check(Has(pre, "comp_name = Wing-mirror_Main"), "dust_pre.in names the second");
		Check(Value(pre, "file_name") == "./geo_input.h5", "dust_pre.in writes the geometry the solver reads");
		const std::string geo = Slurp(dir / "components" / "Wing_Main.in");
		Check(Value(geo, "mesh_file_type") == "basic", "the component is a basic mesh");
		Check(Value(geo, "el_type") == "p", "the component is surface panels");
		Check(Value(geo, "mesh_file") == "./geometry/Wing_Main_",
			"mesh_file is the PREFIX DUST appends rr.dat/ee.dat to");
	}

	// ---- dust.in, and the numbers the dialog reports --------------------

	{
		const std::string s = Slurp(dir / "dust.in");
		Check(Value(s, "geometry_file") == "./geo_input.h5", "dust.in reads the preprocessed geometry");
		Check(Value(s, "reference_file") == "./References.in", "dust.in points at the reference frames");
		Check(Value(s, "n_wake_panels") == "1", "one row of panel wake, so the rest is particles");
		Check(Value(s, "fmm") == "T" && Value(s, "vortstretch") == "T" && Value(s, "diffusion") == "T",
			"the particle models are on by default");

		// The reported dt/tend/radius are the ones in the file. The files
		// carry ten significant digits -- plenty for an input file a user
		// may want to read and edit -- so agreement is relative, not exact.
		const double rel = 1e-9;
		Check(std::fabs(std::stod(Value(s, "dt")) - res.dt) <= rel * std::fabs(res.dt),
			"the dt reported is the dt written");
		Check(std::fabs(std::stod(Value(s, "tend")) - res.tend) <= rel * std::fabs(res.tend),
			"the tend reported is the tend written");
		Check(std::fabs(std::stod(Value(s, "vortex_rad")) - res.vortex_rad) <= rel * std::fabs(res.vortex_rad),
			"the particle core radius reported is the one written");

		// u_inf must be the free stream the user asked for, tilted by alpha.
		Check(std::fabs(Length(res.u_inf) - opt.speed) < 1e-9, "u_inf has the requested speed");
		Check(res.u_inf.z > 0.0 && res.u_inf.x > 0.0, "a positive incidence tilts the flow towards +z");
		Check(std::fabs(res.u_inf.y) < 1e-12, "no sideslip asked for, none applied");
	}

	// ---- the derived geometry is self-consistent ------------------------

	Box3 model;
	for (const DustComponent& c : comps) {
		for (const Vec3& v : c.mesh.vertices) model.Add(v);
	}
	Check(res.box_min.x <= model.min.x && res.box_min.y <= model.min.y && res.box_min.z <= model.min.z &&
	      res.box_max.x >= model.max.x && res.box_max.y >= model.max.y && res.box_max.z >= model.max.z,
		"the particle box contains the model");
	// The wake goes downstream, so the box must be longer that way.
	Check(res.box_max.x - model.max.x > model.min.x - res.box_min.x,
		"the particle box extends further downstream than upstream");
	{
		// The octree must cover the particle box, or particles fall out of
		// the multipole structure.
		const Vec3 reach{
			res.octree_origin.x + res.n_box[0] * res.octree_box_length,
			res.octree_origin.y + res.n_box[1] * res.octree_box_length,
			res.octree_origin.z + res.n_box[2] * res.octree_box_length};
		const double eps = 1e-9 * res.octree_box_length;
		Check(res.octree_origin.x <= res.box_min.x + eps && res.octree_origin.y <= res.box_min.y + eps &&
		      res.octree_origin.z <= res.box_min.z + eps,
			"the octree starts at or before the particle box");
		Check(reach.x >= res.box_max.x - eps && reach.y >= res.box_max.y - eps && reach.z >= res.box_max.z - eps,
			"the octree reaches past the particle box");
	}

	// The time step should resolve the model: the flow must not cross it
	// in a handful of steps.
	Check(res.reference_length > 0.0, "a reference length along the flow was found");
	{
		const double per_length = res.reference_length / (opt.speed * res.dt);
		Check(per_length > 20.0 && per_length < 200.0,
			"the step resolves the model (" + std::to_string(static_cast<int>(per_length)) + " steps to cross it)");
		Check(res.timesteps > 10, "the run is long enough for a wake (" + std::to_string(res.timesteps) + " steps)");
	}
	Check(res.linear_system_mb > 0.0, "the dense panel system is sized, so the dialog can warn about it");

	// ---- the sewing tolerance must never reach a real edge ---------------
	// DUST welds every node pair closer than tol_se_wing together BEFORE it
	// builds the element connectivity, so a tolerance at or above a genuine
	// edge collapses real triangles and the connectivity comes out
	// inconsistent -- reported as "Neighbouring elements with opposed normal
	// orientation". This happened on the P-38 wing: the tolerance was
	// derived from the MEAN panel size (0.00194) while the mesh's shortest
	// edge was 0.00061, so dust_pre refused the case outright.
	{
		Check(res.min_edge > 0.0, "the shortest edge is measured");
		Check(res.tol_se_wing < res.min_edge,
			"the sewing tolerance is below the shortest edge (" + std::to_string(res.tol_se_wing) +
			" < " + std::to_string(res.min_edge) + ")");
		// And with a wide margin, not just barely.
		Check(res.tol_se_wing <= 0.2 * res.min_edge, "the sewing tolerance keeps a wide margin under the shortest edge");

		// A mesh whose panels vary by orders of magnitude is the case that
		// broke: the mean says nothing about the smallest edge. Shrink one
		// triangle until it is far finer than the rest and check the
		// tolerance follows the edge, not the average.
		std::vector<DustComponent> uneven(1);
		uneven[0].name = "Uneven";
		uneven[0].mesh = comps[0].mesh;
		const Vec3 keep = uneven[0].mesh.vertices[uneven[0].mesh.triangles[0][0]];
		uneven[0].mesh.vertices[uneven[0].mesh.triangles[0][1]] =
			keep + (uneven[0].mesh.vertices[uneven[0].mesh.triangles[0][1]] - keep) * 1e-4;
		DustCaseResult r6;
		std::string err6;
		Check(DeriveDustCase(uneven, {}, opt, r6, err6), "a mesh with one very fine panel still derives");
		Check(r6.min_edge < 0.01 * res.min_edge, "the fine edge is what min_edge reports");
		Check(r6.tol_se_wing < r6.min_edge,
			"the sewing tolerance follows the shortest edge, not the mean panel size");
		Check(r6.tol_se_wing < 0.01 * r6.mean_panel_size,
			"... and is far below what the mean panel size alone would have given");
	}

	// ---- explicit settings must override the derived ones ----------------

	{
		DustCaseOptions e;
		e.speed = 10.0;
		e.dt = 0.25;
		e.tend = 5.0;
		e.vortex_rad = 0.123;
		e.tol_se_wing = 4e-4;
		e.fmm = false;
		e.diffusion = false;
		e.n_wake_particles = 1234;
		e.basename = "explicit";
		DustCaseResult r2;
		std::string err2;
		const bool ok2 = WriteDustCase(comps, {}, (dir / "explicit").string(), e, r2, err2);
		Check(ok2, "a case with everything specified is written" + (ok2 ? std::string() : " -- " + err2));
		if (ok2) {
			const std::string s = Slurp(dir / "explicit" / "dust.in");
			Check(std::fabs(r2.dt - 0.25) < 1e-12 && std::fabs(r2.tend - 5.0) < 1e-12, "explicit dt and tend are kept");
			Check(std::fabs(r2.vortex_rad - 0.123) < 1e-12, "explicit particle radius is kept");
			Check(Value(s, "fmm") == "F" && Value(s, "diffusion") == "F", "the model switches reach the file");
			Check(Value(s, "n_wake_particles") == "1234", "the particle budget reaches the file");
			Check(r2.timesteps == 20, "the step count follows from the explicit dt and tend");
			const std::string g = Slurp(dir / "explicit" / "components" / "Wing_Main.in");
			Check(std::fabs(std::stod(Value(g, "tol_se_wing")) - 4e-4) < 1e-15, "explicit sewing tolerance is kept");
		}
	}

	// ---- surfaces lying on top of one another --------------------------------

	{
		// This is the configuration that made a real DUST run diverge on
		// step 0: a wing and its mirror share their root cap exactly, so
		// the panel method gets two unknowns with identical influence and
		// a singular matrix. The writer has to notice.
		Check(res.coincident_panels > 0,
			"the wing and its mirror are caught sharing their root cap (" +
			std::to_string(res.coincident_panels) + " panels)");
		Check(!res.coincident_a.empty() && !res.coincident_b.empty() && res.coincident_a != res.coincident_b,
			"the two bodies doing it are named");

		// Pulled apart, the same two bodies are fine -- and that case does
		// converge in DUST.
		std::vector<DustComponent> apart = comps;
		apart[1].mesh.Translate({0.0, -0.5, 0.0});
		DustCaseResult r4;
		std::string err4;
		Check(WriteDustCase(apart, {}, (dir / "apart").string(), opt, r4, err4), "separated bodies still write");
		Check(r4.coincident_panels == 0, "separated bodies raise no overlap warning");

		// One body on its own can never overlap another.
		std::vector<DustComponent> single(comps.begin(), comps.begin() + 1);
		DustCaseResult r5;
		std::string err5;
		Check(WriteDustCase(single, {}, (dir / "single").string(), opt, r5, err5), "a single body writes");
		Check(r5.coincident_panels == 0, "a single body raises no overlap warning");
	}

	// ---- rotor frames --------------------------------------------------------
	// DUST's own rotor multiplicity: one blade is handed over and DUST
	// replicates and spins it. The blade's mesh must therefore be
	// hub-local, with the placement carried by the frame.
	{
		std::vector<DustComponent> rotor(2);
		rotor[0].name = "Prop/Spinner";          // axisymmetric: stays on frame 0
		rotor[0].mesh = comps[0].mesh;
		rotor[1].name = "Prop/Blade 1";          // the one blade DUST replicates
		rotor[1].mesh = comps[0].mesh;
		rotor[1].ref_tag = "Prop_hub";

		DustFrame hub;
		hub.tag = "Prop_hub";
		hub.origin = Vec3{1.5, -2.0, 0.25};
		hub.n_blades = 4;
		hub.rot_axis = Vec3{1.0, 0.0, 0.0};
		hub.rot_rate = 2.0 * 3.14159265358979323846 * 2400.0 / 60.0;   // 2400 rev/min
		std::vector<DustFrame> frames{hub};

		DustCaseResult r;
		std::string err;
		const bool rok = WriteDustCase(rotor, frames, (dir / "rotor").string(), opt, r, err);
		Check(rok, "rotor case written" + (rok ? std::string() : " -- " + err));
		if (rok) {
			const std::string refs = Slurp(dir / "rotor" / "References.in");
			const std::string pre = Slurp(dir / "rotor" / "dust_pre.in");

			Check(Has(refs, "reference_tag = Prop_hub"), "the hub frame is defined");
			Check(Value(refs, "multiple") == "T", "a rotor frame is a multiple frame");
			Check(Value(refs, "moving") == "F",
				"moving stays F -- the motion comes from the multiplicity, as in DUST's own rotor example");
			Check(Value(refs, "mult_type") == "rotor", "the multiplicity is a rotor");
			Check(Value(refs, "n_blades") == "4", "the blade count reaches the frame");
			Check(std::fabs(std::stod(Value(refs, "rot_rate")) - hub.rot_rate) <= 1e-9 * hub.rot_rate,
				"the rotation rate reaches the frame, in rad/s");
			Check(Has(refs, "origin        = (/ 1.5, -2, 0.25 /)"), "the hub position reaches the frame");

			// Only the blade rides the hub; the spinner is axisymmetric about
			// the same axis, so spinning it would change nothing and it stays
			// on the global frame.
			Check(Has(pre, "comp_name = Prop_Blade_1"), "the blade is a component");
			const std::size_t blade_at = pre.find("comp_name = Prop_Blade_1");
			const std::size_t spinner_at = pre.find("comp_name = Prop_Spinner");
			Check(blade_at != std::string::npos && spinner_at != std::string::npos, "both propeller bodies are components");
			Check(pre.find("ref_tag   = Prop_hub", blade_at) != std::string::npos &&
			      pre.find("ref_tag   = Prop_hub", blade_at) < pre.find("comp_name", blade_at + 1),
				"the blade rides the hub frame");
			Check(pre.find("ref_tag   = 0", spinner_at) != std::string::npos,
				"the spinner stays on the global frame");

			// DUST replicates the blade, so the solver sees it n_blades
			// times -- the dense-system estimate has to know that.
			Check(r.rotors == 1, "one rotor reported");
			const std::size_t tri = comps[0].mesh.triangles.size();
			Check(r.panels == tri * 5, "panels count the spinner once and the blade four times (" +
				std::to_string(r.panels) + " vs " + std::to_string(tri * 5) + ")");

			// A case with no frames must define none -- Value() ignores the
			// commented-out template the file always carries.
			Check(Value(Slurp(dir / "References.in"), "reference_tag").empty(),
				"a case without rotors defines no frames at all");
			Check(Value(refs, "reference_tag") == "Prop_hub",
				"... while the rotor case defines exactly the hub");

			// n_dofs has no default in DUST, so leaving it out makes the
			// SOLVER refuse the case even though dust_pre is happy -- the
			// preprocessor never reads References.in. Zero is right for a
			// rigid propeller.
			Check(Value(refs, "n_dofs") == "0", "n_dofs is written (required, and zero for a rigid propeller)");

			// DUST builds its OWN frame for each blade, with z along the
			// rotation axis (mod_reference.f90:1237), and reads the blade's
			// coordinates in it. The blade must therefore be written in
			// that frame, not in hub coordinates, or it is mounted skewed.
			// With rot_axis = +x the frame is x=(0,-1,0) y=(0,0,-1)
			// z=(1,0,0), so a hub point (bx,by,bz) is written (-by,-bz,bx).
			{
				const std::string bladeRr = Slurp(dir / "rotor" / "geometry" / "Prop_Blade_1_rr.dat");
				std::istringstream in(bladeRr);
				double wx, wy, wz;
				in >> wx >> wy >> wz;
				const Vec3& b = rotor[1].mesh.vertices[0];
				Check(std::fabs(wx - (-b.y)) < 1e-9 && std::fabs(wy - (-b.z)) < 1e-9 && std::fabs(wz - b.x) < 1e-9,
					"the blade is written in DUST's blade frame, not in hub coordinates");
				// The spinner is on frame 0 and must NOT be transformed.
				const std::string spinRr = Slurp(dir / "rotor" / "geometry" / "Prop_Spinner_rr.dat");
				std::istringstream si(spinRr);
				double sx, sy, sz;
				si >> sx >> sy >> sz;
				const Vec3& s0 = rotor[0].mesh.vertices[0];
				Check(std::fabs(sx - s0.x) < 1e-9 && std::fabs(sy - s0.y) < 1e-9 && std::fabs(sz - s0.z) < 1e-9,
					"a body on the global frame is written as given");
			}

			// A rotor turns on its own clock: the step must resolve the
			// blade passing, not just the flow crossing the model.
			Check(r.degrees_per_step > 0.0 && r.degrees_per_step <= 5.0 + 1e-9,
				"the step resolves the rotation (" + std::to_string(r.degrees_per_step) + " deg/step)");
			Check(r.dt_from_rotor, "the rotor is what set the step here, not the flow");
			Check(r.steps_per_revolution >= 60.0,
				"a revolution takes enough steps to mean something (" +
				std::to_string(r.steps_per_revolution) + ")");

			// Without the rotor the same geometry would have taken a much
			// coarser step: that is the whole point of the rule.
			DustCaseResult plain;
			std::string perr;
			Check(DeriveDustCase(rotor, {}, opt, plain, perr) && plain.dt > r.dt * 2.0,
				"the flow alone would have given a far coarser step");
		}
	}

	// A rotor the user has asked to leave coarse: degrees_per_step = 0
	// means "do not let the rotor touch the step".
	{
		std::vector<DustComponent> rotor(1);
		rotor[0].name = "Blade";
		rotor[0].mesh = comps[0].mesh;
		rotor[0].ref_tag = "hub";
		DustFrame hub;
		hub.tag = "hub";
		hub.n_blades = 2;
		hub.rot_rate = 300.0;
		DustCaseOptions o = opt;
		o.degrees_per_step = 0.0;
		DustCaseResult r;
		std::string err;
		Check(DeriveDustCase(rotor, {hub}, o, r, err) && !r.dt_from_rotor,
			"degrees_per_step = 0 leaves the step to the flow");
	}

	// ---- refusals ----------------------------------------------------------

	{
		DustCaseResult r3;
		std::string err3;
		Check(!WriteDustCase({}, {}, (dir / "empty").string(), DustCaseOptions(), r3, err3) && !err3.empty(),
			"no bodies is refused with a reason");
		std::vector<DustComponent> blank(1);
		blank[0].name = "Nothing";
		Check(!WriteDustCase(blank, {}, (dir / "blank").string(), DustCaseOptions(), r3, err3) && !err3.empty(),
			"a body with no triangles is refused with a reason");
	}

	Check(Has(DustRunCommands(), "dust_pre") && Has(DustRunCommands(), "dust_post"),
		"the run commands name the three executables");

	std::printf("\ncase: %zu components, %zu panels, %zu nodes; dt %.4g, %d steps to t=%.4g;\n"
		"      particle radius %.4g, dense system %.1f MB; wrote %zu files to %s\n",
		res.components, res.panels, res.nodes, res.dt, res.timesteps, res.tend,
		res.vortex_rad, res.linear_system_mb, res.files.size(), dir.string().c_str());
	std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL OK" : "FAILURES", g_failures, g_failures == 1 ? "" : "s");
	return g_failures == 0 ? 0 : 1;
}

