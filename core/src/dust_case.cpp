#include "aerotelic/dust_case.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace aerotelic {
namespace {

constexpr double kPi = 3.14159265358979323846;

// DUST's parser reads reals with a list-directed READ, which takes
// Fortran and C real spellings alike; what it must never see is a
// number dressed up with thousands separators or a bare "inf".
std::string Num(double v, int digits = 10) {
	if (!std::isfinite(v)) v = 0.0;
	char buf[64];
	std::snprintf(buf, sizeof buf, "%.*g", digits, v);
	return buf;
}

std::string Vec(const Vec3& v) {
	return "(/ " + Num(v.x) + ", " + Num(v.y) + ", " + Num(v.z) + " /)";
}

std::string Flag(bool b) { return b ? "T" : "F"; }

// DUST keys off whitespace-separated tokens, and a component's name
// becomes part of file names and of the component's identity in the
// results, so it is reduced to something both can carry.
std::string SafeName(const std::string& in, std::size_t index) {
	std::string s;
	for (char c : in) {
		const unsigned char u = static_cast<unsigned char>(c);
		if ((u >= 'a' && u <= 'z') || (u >= 'A' && u <= 'Z') || (u >= '0' && u <= '9') || c == '_' || c == '-') {
			s.push_back(c);
		} else if (c == ' ' || c == '/' || c == '\\' || c == '.') {
			if (!s.empty() && s.back() != '_') s.push_back('_');
		}
	}
	while (!s.empty() && s.back() == '_') s.pop_back();
	if (s.empty()) s = "Body_" + std::to_string(index + 1);
	// A name that starts with a digit would still parse, but reads badly
	// in DUST's own output tables.
	if (s[0] >= '0' && s[0] <= '9') s = "B" + s;
	return s;
}

// Mean of sqrt(2 * area) -- the side of the square with a triangle's
// area, i.e. "how big is a panel here" -- and, separately, the SHORTEST
// edge anywhere in the mesh, which is what bounds the sewing tolerance
// (see DustCaseOptions::tol_se_wing). The two differ by orders of
// magnitude on a real wing, which is the whole point of measuring both.
double MeanPanelSize(const std::vector<DustComponent>& components, const std::vector<std::size_t>& copies,
	std::size_t& out_panels, std::size_t& out_nodes, double& out_min_edge) {
	double total = 0.0;
	std::size_t n = 0;
	out_panels = 0;
	out_nodes = 0;
	out_min_edge = std::numeric_limits<double>::infinity();
	for (std::size_t ci = 0; ci < components.size(); ++ci) {
		const DustComponent& c = components[ci];
		// A blade on a rotor frame is replicated by DUST, so it costs the
		// solver once per blade even though it is written once.
		const std::size_t mult = ci < copies.size() ? std::max<std::size_t>(1, copies[ci]) : 1;
		out_nodes += c.mesh.vertices.size() * mult;
		for (const auto& t : c.mesh.triangles) {
			const Vec3 a = c.mesh.vertices[t[0]], b = c.mesh.vertices[t[1]], d = c.mesh.vertices[t[2]];
			const double area = 0.5 * Length(Cross(b - a, d - a));
			if (!(area > 0.0)) continue;
			total += std::sqrt(2.0 * area);
			++n;
			out_panels += mult;
			for (const double e : {Length(b - a), Length(d - b), Length(a - d)}) {
				if (e > 0.0 && e < out_min_edge) out_min_edge = e;
			}
		}
	}
	if (!std::isfinite(out_min_edge)) out_min_edge = 0.0;
	return n > 0 ? total / static_cast<double>(n) : 0.0;
}

// A rigid placement: world = origin + R * v, with R held as its columns.
struct Placement {
	Vec3 origin{};
	Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

	Vec3 Rotate(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
	Vec3 Apply(const Vec3& v) const { return origin + Rotate(v); }
};

// outer applied to whatever inner produced.
Placement Compose(const Placement& outer, const Placement& inner) {
	Placement p;
	p.origin = outer.Apply(inner.origin);
	p.x = outer.Rotate(inner.x);
	p.y = outer.Rotate(inner.y);
	p.z = outer.Rotate(inner.z);
	return p;
}

// Rotation of psi radians about `axis` (which need not be unit).
Placement AxisRotation(const Vec3& axis, double psi) {
	const Vec3 a = Normalized(axis);
	if (!(LengthSq(a) > 0.0)) return Placement();
	const double c = std::cos(psi), s = std::sin(psi), t = 1.0 - c;
	Placement p;
	p.x = Vec3{c + t * a.x * a.x, t * a.x * a.y + s * a.z, t * a.x * a.z - s * a.y};
	p.y = Vec3{t * a.x * a.y - s * a.z, c + t * a.y * a.y, t * a.y * a.z + s * a.x};
	p.z = Vec3{t * a.x * a.z + s * a.y, t * a.y * a.z - s * a.x, c + t * a.z * a.z};
	return p;
}

const DustFrame* FindFrame(const std::vector<DustFrame>& frames, const std::string& tag) {
	if (tag.empty() || tag == "0") return nullptr;
	for (const DustFrame& f : frames) {
		if (f.tag == tag) return &f;
	}
	return nullptr;
}

// The frame's placement in world coordinates, parent chain included.
// Walks up to frame 0, giving up after as many steps as there are frames
// so a malformed parent -- missing, or a cycle -- cannot hang the dialog
// this runs on.
Placement FrameToWorld(const std::vector<DustFrame>& frames, const std::string& tag) {
	Placement out;
	const DustFrame* f = FindFrame(frames, tag);
	for (std::size_t guard = 0; f != nullptr && guard <= frames.size(); ++guard) {
		Placement local;
		local.origin = f->origin;
		local.x = f->x_axis;
		local.y = f->y_axis;
		local.z = f->z_axis;
		out = Compose(local, out);
		f = FindFrame(frames, f->parent);
	}
	return out;
}

// Where DUST will actually mount a component: one placement for a body on
// the global frame, and one PER BLADE for a body on a rotor frame, since
// DUST replicates it around the axis.
//
// This is the only thing that puts the two kinds of body in the same
// space. A component's mesh is in its OWN frame's coordinates (see
// DustComponent::mesh), so a rotor's blade is hub-local while the
// airframe is already in world coordinates -- and anything that compares
// or measures them together has to place them first. Leaving it out made
// two identical propellers at opposite ends of an aircraft look like one
// body lying on top of another, and sized the particle box from blades
// sitting at the origin.
//
// DUST mounts blade k at psi_0 + k * 360/n_blades about rot_axis, in the
// frame's own components. The blade frame BladeFrame builds below does
// not enter into it: the writer expresses the mesh in that frame and DUST
// reads it back in the same frame, so the two cancel and the mesh arrives
// as the hub-local coordinates it is stored in.
std::vector<Placement> ComponentPlacements(const DustComponent& c, const std::vector<DustFrame>& frames) {
	const DustFrame* f = FindFrame(frames, c.ref_tag);
	if (f == nullptr) return {Placement()};
	const Placement to_world = FrameToWorld(frames, c.ref_tag);
	if (f->n_blades <= 0) return {to_world};

	std::vector<Placement> out;
	out.reserve(static_cast<std::size_t>(f->n_blades));
	for (int k = 0; k < f->n_blades; ++k) {
		const double psi = (f->psi_0_deg + 360.0 * k / f->n_blades) * kPi / 180.0;
		out.push_back(Compose(to_world, AxisRotation(f->rot_axis, psi)));
	}
	return out;
}

// Counts panels of different bodies sitting on top of one another, which
// makes DUST's influence matrix singular (see DustCaseResult). Centroids
// are hashed into cells a panel wide, so this is linear in the panel
// count rather than quadratic -- it has to run on every dialog refresh.
//
// Everything is compared WHERE DUST WILL PUT IT (see ComponentPlacements),
// including every blade of a rotor, so a blade is checked against the
// airframe it turns in front of rather than against another hub's blade
// in hub coordinates. Two blades of the SAME rotor are not compared with
// each other: they are the one component DUST replicated, and the skip on
// `e.component == ci` is what keeps a body from overlapping itself.
//
// Blades are compared at their mounted azimuth only. A blade that sweeps
// THROUGH a body at some other azimuth is not found here -- that is a
// collision over the run, not a singular matrix on step 1, which is what
// this warning is about.
std::size_t CountCoincidentPanels(const std::vector<DustComponent>& components,
	const std::vector<DustFrame>& frames, double panel_size, std::string& out_a, std::string& out_b) {
	if (!(panel_size > 0.0)) return 0;
	const double cell = panel_size;
	const double tol = 0.05 * panel_size;
	const double tol2 = tol * tol;

	struct Entry { Vec3 c; int component; };
	std::unordered_map<long long, std::vector<Entry>> grid;
	auto key = [cell](const Vec3& p) {
		const long long x = static_cast<long long>(std::floor(p.x / cell));
		const long long y = static_cast<long long>(std::floor(p.y / cell));
		const long long z = static_cast<long long>(std::floor(p.z / cell));
		return (x * 73856093LL) ^ (y * 19349663LL) ^ (z * 83492791LL);
	};

	std::size_t hits = 0;
	for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
		const TriMesh& m = components[ci].mesh;
		// The centroid of a placed triangle is the placed centroid: the
		// placement is affine, so there is no need to move three vertices.
		for (const Placement& place : ComponentPlacements(components[ci], frames)) {
			for (const auto& t : m.triangles) {
				const Vec3 c = place.Apply((m.vertices[t[0]] + m.vertices[t[1]] + m.vertices[t[2]]) / 3.0);
				bool hit = false;
				for (int dx = -1; dx <= 1 && !hit; ++dx) {
					for (int dy = -1; dy <= 1 && !hit; ++dy) {
						for (int dz = -1; dz <= 1 && !hit; ++dz) {
							const Vec3 probe{c.x + dx * cell, c.y + dy * cell, c.z + dz * cell};
							auto it = grid.find(key(probe));
							if (it == grid.end()) continue;
							for (const Entry& e : it->second) {
								if (e.component == ci) continue;
								if (LengthSq(e.c - c) > tol2) continue;
								++hits;
								if (out_a.empty()) {
									out_a = components[e.component].name;
									out_b = components[ci].name;
								}
								hit = true;
								break;
							}
						}
					}
				}
				grid[key(c)].push_back({c, ci});
			}
		}
	}
	return hits;
}

// The frame DUST builds for each blade of a rotor, reproduced exactly
// from mod_reference.f90:1237-1244:
//
//     norm      = -cross(axis, (0,1,0))   [or cross(axis,(1,0,0))]
//     F[:,3] = axis ; F[:,2] = norm ; F[:,1] = cross(F[:,2], F[:,3])
//
// The blade's mesh coordinates are read IN THAT FRAME, whose z is the
// rotation axis -- it is never the identity. So a blade expressed in hub
// coordinates b has to be written as F^T * b, or DUST silently mounts it
// skewed and the twist it solves is not the twist that was designed.
struct BladeFrame {
	Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

	explicit BladeFrame(const Vec3& axis) {
		Vec3 a = axis;
		const double na = Length(a);
		a = na > 0.0 ? a / na : Vec3{0.0, 0.0, 1.0};
		Vec3 n = Vec3{0.0, 0.0, 0.0} - Cross(a, Vec3{0.0, 1.0, 0.0});
		if (Length(n) <= 1e-12) n = Cross(a, Vec3{1.0, 0.0, 0.0});
		n = n / Length(n);
		z = a;
		y = n;
		x = Cross(y, z);
	}

	// F^T * v: the components of v along the frame's own axes.
	Vec3 ToFrame(const Vec3& v) const { return {Dot(x, v), Dot(y, v), Dot(z, v)}; }
};

bool WriteText(const std::filesystem::path& p, const std::string& text, std::string& error) {
	std::ofstream f(p, std::ios::binary);
	if (!f) {
		error = "could not write " + p.string();
		return false;
	}
	f << text;
	if (!f) {
		error = "could not write " + p.string();
		return false;
	}
	return true;
}

}  // namespace

std::string DustRunCommands() {
	return "dust_pre  dust_pre.in\n"
	       "dust      dust.in\n"
	       "dust_post dust_post.in\n";
}

bool DeriveDustCase(const std::vector<DustComponent>& components, const std::vector<DustFrame>& frames,
	const DustCaseOptions& options, DustCaseResult& result, std::string& error) {
	error.clear();
	result = DustCaseResult();

	if (components.empty()) {
		error = "no bodies were given to the analysis";
		return false;
	}

	// How many times DUST will replicate each component: once, unless it
	// rides a rotor frame, in which case once per blade.
	std::vector<std::size_t> copies(components.size(), 1);
	for (std::size_t i = 0; i < components.size(); ++i) {
		if (components[i].ref_tag.empty()) continue;
		for (const DustFrame& f : frames) {
			if (f.tag != components[i].ref_tag) continue;
			if (f.n_blades > 0) copies[i] = static_cast<std::size_t>(f.n_blades);
			break;
		}
	}
	for (const DustFrame& f : frames) {
		if (f.n_blades > 0) ++result.rotors;
	}

	// ---- the numbers the case is derived from -------------------------

	// Every body placed where DUST will mount it, blade by blade. A rotor's
	// blade mesh is hub-local and reaches its hub only through its frame,
	// so a box measured from the meshes as stored would be a box around an
	// aircraft with all its propellers piled up at the origin -- too small
	// where the rotors actually are, which is exactly where the wake is.
	Box3 bounds;
	for (const DustComponent& c : components) {
		for (const Placement& place : ComponentPlacements(c, frames)) {
			for (const Vec3& v : c.mesh.vertices) bounds.Add(place.Apply(v));
		}
	}
	if (bounds.IsEmpty()) {
		error = "the chosen bodies carry no geometry";
		return false;
	}

	result.mean_panel_size = MeanPanelSize(components, copies, result.panels, result.nodes, result.min_edge);
	if (result.panels == 0) {
		error = "the chosen bodies carry no triangles";
		return false;
	}
	result.components = components.size();

	const double speed = options.speed > 0.0 ? options.speed : 1.0;
	const double a = options.alpha_deg * kPi / 180.0;
	const double b = options.beta_deg * kPi / 180.0;
	result.u_inf = Vec3{speed * std::cos(a) * std::cos(b), speed * std::sin(b), speed * std::sin(a) * std::cos(b)};

	// The reference length is the model's extent along the flow: the
	// chord of a wing, the length of a fuselage. That is what sets both
	// how long the flow takes to cross the model and a sensible step.
	const Vec3 ext = bounds.Extent();
	const Vec3 dir = Normalized(result.u_inf);
	double along = std::fabs(dir.x) * ext.x + std::fabs(dir.y) * ext.y + std::fabs(dir.z) * ext.z;
	if (!(along > 0.0)) along = bounds.Diagonal();
	if (!(along > 0.0)) along = 1.0;
	result.reference_length = along;

	const double steps_per_length = options.steps_per_length > 0.0 ? options.steps_per_length : 40.0;
	double flow_dt = along / (speed * steps_per_length);
	if (!(flow_dt > 0.0)) flow_dt = 1.0;

	// The fastest rotor in the case, if any. A propeller turns on its own
	// clock: the flow may cross the model once while the blades go round
	// a dozen times, so the step that resolves the flow can be far too
	// coarse to resolve the blade passing.
	double max_omega = 0.0;
	for (const DustFrame& f : frames) {
		if (f.n_blades > 0) max_omega = std::max(max_omega, std::fabs(f.rot_rate));
	}
	result.dt = options.dt > 0.0 ? options.dt : flow_dt;
	if (options.dt <= 0.0 && max_omega > 0.0 && options.degrees_per_step > 0.0) {
		const double rotor_dt = options.degrees_per_step * kPi / 180.0 / max_omega;
		if (rotor_dt < result.dt) {
			result.dt = rotor_dt;
			result.dt_from_rotor = true;
		}
	}
	if (!(result.dt > 0.0)) result.dt = 1.0;
	if (max_omega > 0.0) {
		result.degrees_per_step = max_omega * result.dt * 180.0 / kPi;
		result.steps_per_revolution = 2.0 * kPi / (max_omega * result.dt);
	}
	const double crossings = options.crossings > 0.0 ? options.crossings : 8.0;
	result.tend = options.tend > 0.0 ? options.tend : options.tstart + crossings * along / speed;
	if (result.tend <= options.tstart) result.tend = options.tstart + result.dt;
	result.dt_out = options.dt_out > 0.0 ? options.dt_out : result.dt;
	result.timesteps = static_cast<int>(std::lround((result.tend - options.tstart) / result.dt));

	result.vortex_rad = options.vortex_rad > 0.0 ? options.vortex_rad : result.mean_panel_size;
	if (!(result.vortex_rad > 0.0)) result.vortex_rad = 0.1;
	// A tenth of the SHORTEST edge, never a fraction of the mean panel:
	// DUST welds every node pair closer than this before building the
	// connectivity, so anything at or above a real edge destroys the mesh
	// (see DustCaseOptions::tol_se_wing). The loft shares its
	// trailing-edge vertices exactly anyway, so there is normally nothing
	// to sew and this only has to absorb float noise.
	result.tol_se_wing = options.tol_se_wing > 0.0 ? options.tol_se_wing
		: std::max(1e-12, 0.1 * result.min_edge);

	// The box the particles live in: clear of the model on every side,
	// and much longer downstream, which is where the wake goes.
	const double diag = std::max(bounds.Diagonal(), 1e-12);
	const double margin = std::max(0.0, options.box_margin) * diag;
	const double wake = std::max(0.0, options.box_wake) * diag;
	Vec3 lo = bounds.min - Vec3{margin, margin, margin};
	Vec3 hi = bounds.max + Vec3{margin, margin, margin};
	// Extend along the flow, in whichever direction it actually points.
	if (dir.x >= 0.0) hi.x += wake * std::fabs(dir.x); else lo.x -= wake * std::fabs(dir.x);
	if (dir.y >= 0.0) hi.y += wake * std::fabs(dir.y); else lo.y -= wake * std::fabs(dir.y);
	if (dir.z >= 0.0) hi.z += wake * std::fabs(dir.z); else lo.z -= wake * std::fabs(dir.z);
	result.box_min = lo;
	result.box_max = hi;

	// The octree has to cover that box: square boxes of the shortest
	// side, enough of them in each direction to reach across.
	const Vec3 bext = hi - lo;
	double cell = std::min(bext.x, std::min(bext.y, bext.z));
	if (!(cell > 0.0)) cell = diag;
	for (int i = 0; i < 3; ++i) {
		const double e = bext[i];
		result.n_box[i] = std::max(1, static_cast<int>(std::ceil(e / cell - 1e-9)));
	}
	result.octree_box_length = cell;
	result.octree_origin = lo;

	// DUST's panel method solves a dense system over the panels.
	result.linear_system_mb = static_cast<double>(result.panels) * static_cast<double>(result.panels) * 8.0 / (1024.0 * 1024.0);

	result.coincident_panels = CountCoincidentPanels(components, frames, result.mean_panel_size,
		result.coincident_a, result.coincident_b);

	return true;
}

bool WriteDustCase(const std::vector<DustComponent>& components, const std::vector<DustFrame>& frames,
	const std::string& directory, const DustCaseOptions& options, DustCaseResult& result, std::string& error) {
	if (!DeriveDustCase(components, frames, options, result, error)) {
		return false;
	}
	const double speed = Length(result.u_inf) > 0.0 ? Length(result.u_inf) : 1.0;

	// ---- the directories ----------------------------------------------

	namespace fs = std::filesystem;
	const fs::path root(directory);
	std::error_code ec;
	for (const char* sub : {"", "components", "geometry", "Output", "Postprocessing"}) {
		fs::create_directories(root / sub, ec);
		if (ec) {
			error = "could not create " + (root / sub).string() + ": " + ec.message();
			return false;
		}
	}

	// ---- names ---------------------------------------------------------

	std::vector<std::string> names;
	names.reserve(components.size());
	for (std::size_t i = 0; i < components.size(); ++i) {
		std::string n = SafeName(components[i].name, i);
		while (std::find(names.begin(), names.end(), n) != names.end()) n += "_" + std::to_string(i + 1);
		names.push_back(n);
	}

	// ---- the meshes, and one geometry file each -------------------------

	std::ostringstream pre;
	pre << "! DUST preprocessor input, written by Aerotelic.\n"
	       "! Run:  dust_pre dust_pre.in\n"
	       "! It collects the components below into the single geometry file\n"
	       "! that the solver reads.\n\n";

	for (std::size_t i = 0; i < components.size(); ++i) {
		const DustComponent& c = components[i];
		const std::string& name = names[i];

		// A blade on a rotor frame is written in the frame DUST builds for
		// it, not in hub coordinates (see BladeFrame).
		const DustFrame* rotor = nullptr;
		if (!c.ref_tag.empty()) {
			for (const DustFrame& f : frames) {
				if (f.tag == c.ref_tag && f.n_blades > 0) { rotor = &f; break; }
			}
		}

		// The "basic" mesh: nodes, then elements as 1-based indices with
		// a zero fourth entry, which is how DUST spells a triangle.
		{
			std::ostringstream rr;
			rr.precision(17);
			for (const Vec3& v : c.mesh.vertices) {
				const Vec3 p = rotor != nullptr ? BladeFrame(rotor->rot_axis).ToFrame(v) : v;
				rr << std::scientific << p.x << ' ' << p.y << ' ' << p.z << '\n';
			}
			if (!WriteText(root / "geometry" / (name + "_rr.dat"), rr.str(), error)) return false;
			result.files.push_back("geometry/" + name + "_rr.dat");
		}
		{
			std::ostringstream ee;
			for (const auto& t : c.mesh.triangles) {
				ee << (t[0] + 1) << ' ' << (t[1] + 1) << ' ' << (t[2] + 1) << " 0\n";
			}
			if (!WriteText(root / "geometry" / (name + "_ee.dat"), ee.str(), error)) return false;
			result.files.push_back("geometry/" + name + "_ee.dat");
		}

		std::ostringstream geo;
		geo << "! Component \"" << name << "\", written by Aerotelic.\n"
		    << "! The mesh is DUST's \"basic\" format: mesh_file is a PREFIX, and\n"
		    << "! DUST appends rr.dat (nodes) and ee.dat (elements) to it.\n\n"
		    << "mesh_file_type   = basic\n"
		    << "mesh_file        = ./geometry/" << name << "_\n"
		    << "el_type          = p\n"
		    << "scaling_factor   = 1.0\n\n"
		    << "! Trailing-edge detection. An edge is a trailing edge where the\n"
		    << "! two panels meeting at it face away from each other by more than\n"
		    << "! inner_product_te, after gaps narrower than tol_se_wing are sewn\n"
		    << "! shut. Raise inner_product_te towards 0 to catch a blunter edge;\n"
		    << "! a body with no sharp edge at all simply sheds no wake.\n"
		    << "tol_se_wing      = " << Num(result.tol_se_wing) << "\n"
		    << "inner_product_te = " << Num(options.inner_product_te) << "\n";
		if (!WriteText(root / "components" / (name + ".in"), geo.str(), error)) return false;
		result.files.push_back("components/" + name + ".in");

		pre << "comp_name = " << name << "\n"
		    << "geo_file  = ./components/" << name << ".in\n"
		    << "ref_tag   = " << (c.ref_tag.empty() ? std::string("0") : c.ref_tag) << "\n\n";
	}

	pre << "! The preprocessed geometry the solver reads.\n"
	       "file_name = ./geo_input.h5\n";
	if (!WriteText(root / "dust_pre.in", pre.str(), error)) return false;
	result.files.push_back("dust_pre.in");

	// ---- reference frames ------------------------------------------------

	{
		std::ostringstream ref;
		ref << "! Reference frames, written by Aerotelic.\n!\n";
		if (frames.empty()) {
			ref << "! Every component is on reference frame 0, the global one, which\n"
			       "! needs no definition -- so this file defines none.\n!\n";
		} else {
			ref << "! A rotor frame hands DUST ONE blade and lets it replicate and\n"
			       "! spin them: n_blades copies about rot_axis at rot_rate. The\n"
			       "! blade's mesh is therefore in HUB-LOCAL coordinates, and this\n"
			       "! frame's origin and orientation carry the placement -- which is\n"
			       "! why the blade must not also be placed in its own mesh file.\n"
			       "!\n"
			       "! The nine orientation numbers are the frame's x, y and z axes in\n"
			       "! the parent's components, written column by column, which is how\n"
			       "! DUST's Fortran reshape() reads them.\n"
			       "!\n";
			for (const DustFrame& f : frames) {
				ref << "!" << std::string(76, '-') << "\n"
				    << "reference_tag = " << f.tag << "\n"
				    << "parent_tag    = " << (f.parent.empty() ? std::string("0") : f.parent) << "\n"
				    << "origin        = " << Vec(f.origin) << "\n"
				    << "orientation   = (/ "
				    << Num(f.x_axis.x) << ", " << Num(f.x_axis.y) << ", " << Num(f.x_axis.z) << ", "
				    << Num(f.y_axis.x) << ", " << Num(f.y_axis.y) << ", " << Num(f.y_axis.z) << ", "
				    << Num(f.z_axis.x) << ", " << Num(f.z_axis.y) << ", " << Num(f.z_axis.z) << " /)\n"
				    << "multiple      = " << Flag(f.n_blades > 0) << "\n"
				    << "moving        = F\n";
				if (f.n_blades > 0) {
					// moving = F and multiple = T is not a contradiction: the
					// motion comes from the rotor multiplicity, not from a
					// `motion` block. DUST's own rotor example reads the same.
					ref << "\nmultiplicity = {\n"
					    << "  mult_type  = rotor\n"
					    << "  n_blades   = " << f.n_blades << "\n"
					    << "  rot_axis   = " << Vec(f.rot_axis) << "\n"
					    << "  rot_rate   = " << Num(f.rot_rate) << "   ! rad/s"
					    << " (" << Num(f.rot_rate * 60.0 / (2.0 * kPi), 6) << " rev/min"
					    << (f.rot_rate < 0.0 ? ", negative = the opposite hand" : "") << ")\n"
					    << "  psi_0      = " << Num(f.psi_0_deg) << "\n"
					    << "  hub_offset = " << Num(f.hub_offset) << "\n"
					    // Required, with no default: the number of hinge
					    // degrees of freedom per blade. A propeller is rigid,
					    // so zero -- DUST's dof loop then runs no iterations.
					    // Add Flap/Lag/Pitch dof blocks here for a rotor that
					    // needs collective or cyclic.
					    << "  n_dofs     = 0\n"
					    << "}\n";
				}
				ref << "\n";
			}
		}
		ref << "! To add motion of your own: uncomment and edit the block below,\n"
		       "! then point a component's ref_tag in dust_pre.in at its tag.\n"
		       "!\n"
		       "! reference_tag = Rotor\n"
		       "! parent_tag    = 0\n"
		       "! origin        = (/ 0.0, 0.0, 0.0 /)\n"
		       "! orientation   = (/ 1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0 /)\n"
		       "! multiple      = F\n"
		       "! moving        = T\n"
		       "! motion = {\n"
		       "!   pole = {\n"
		       "!     input = position\n"
		       "!     input_type = simple_function\n"
		       "!     function = (/ 0, 0, 0 /)\n"
		       "!     amplitude = 0.0\n"
		       "!     vector = (/ 0.0, 0.0, 0.0 /)\n"
		       "!     omega  = (/ 0.0, 0.0, 0.0 /)\n"
		       "!     phase  = (/ 0.0, 0.0, 0.0 /)\n"
		       "!     offset = (/ 0.0, 0.0, 0.0 /)\n"
		       "!   }\n"
		       "!   rotation = {\n"
		       "!     input = velocity\n"
		       "!     input_type = simple_function\n"
		       "!     function = 0\n"
		       "!     axis = (/ 1.0, 0.0, 0.0 /)\n"
		       "!     amplitude = 0.0\n"
		       "!     omega = 0.0\n"
		       "!     phase = 0.0\n"
		       "!     offset = 0.0\n"
		       "!     psi_0 = 0.0\n"
		       "!   }\n"
		       "! }\n";
		if (!WriteText(root / "References.in", ref.str(), error)) return false;
		result.files.push_back("References.in");
	}

	// ---- the solver ------------------------------------------------------

	{
		std::ostringstream s;
		s << "! DUST solver input, written by Aerotelic.\n"
		     "! Run:  dust dust.in      (after dust_pre)\n\n"
		     "basename        = ./Output/" << options.basename << "\n"
		     "geometry_file   = ./geo_input.h5\n"
		     "reference_file  = ./References.in\n\n"
		     "! ---- free stream -------------------------------------------\n"
		     "! +x downstream, tilted towards +z by the angle of attack ("
		  << Num(options.alpha_deg, 6) << " deg)\n"
		     "! and towards +y by the sideslip (" << Num(options.beta_deg, 6) << " deg).\n"
		     "u_inf   = " << Vec(result.u_inf) << "\n"
		     "rho_inf = " << Num(options.rho_inf) << "\n"
		     "p_inf   = " << Num(options.p_inf) << "\n"
		     "a_inf   = " << Num(options.a_inf) << "\n"
		     "mu_inf  = " << Num(options.mu_inf) << "\n\n"
		     "! ---- time ---------------------------------------------------\n"
		     "! The model is " << Num(result.reference_length, 6) << " long along the flow, so the flow\n"
		     "! crosses it in " << Num(result.reference_length / speed, 6) << " s; this runs for "
		  << Num((result.tend - options.tstart) * speed / result.reference_length, 4) << " crossings in "
		  << result.timesteps << " steps.\n";
		if (result.degrees_per_step > 0.0) {
			s << "! The fastest blade sweeps " << Num(result.degrees_per_step, 4) << " deg per step ("
			  << Num(result.steps_per_revolution, 4) << " steps a revolution)"
			  << (result.dt_from_rotor ? ", which is what set the step" : "") << ".\n";
		}
		s <<
		     "tstart  = " << Num(options.tstart) << "\n"
		     "tend    = " << Num(result.tend) << "\n"
		     "dt      = " << Num(result.dt) << "\n"
		     "dt_out  = " << Num(result.dt_out) << "\n"
		     "output_start = T\n\n"
		     "! ---- wake ----------------------------------------------------\n"
		     "! n_wake_panels rows of panel wake stay attached at the trailing\n"
		     "! edge; everything past them is carried as free vortex particles,\n"
		     "! which is what makes this a vortex particle method.\n"
		     "n_wake_panels     = " << options.n_wake_panels << "\n"
		     "n_wake_particles  = " << options.n_wake_particles << "\n"
		     "particles_box_min = " << Vec(result.box_min) << "\n"
		     "particles_box_max = " << Vec(result.box_max) << "\n"
		     "! A particle leaving the box is dropped, so the box is what bounds\n"
		     "! the cost of a long run.\n\n"
		     "! ---- models ---------------------------------------------------\n"
		     "vortex_rad            = " << Num(result.vortex_rad) << "\n"
		     "fmm                   = " << Flag(options.fmm) << "\n"
		     "vortstretch           = " << Flag(options.vortstretch) << "\n"
		     "diffusion             = " << Flag(options.diffusion) << "\n"
		     "penetration_avoidance = " << Flag(options.penetration_avoidance) << "\n\n"
		     "! ---- octree, used by the fast multipole method ----------------\n"
		     "! It has to cover the particle box above.\n"
		     "box_length        = " << Num(result.octree_box_length) << "\n"
		     "n_box             = (/ " << result.n_box[0] << ", " << result.n_box[1] << ", " << result.n_box[2] << " /)\n"
		     "octree_origin     = " << Vec(result.octree_origin) << "\n"
		     "n_octree_levels   = " << options.n_octree_levels << "\n"
		     "min_octree_part   = " << options.min_octree_part << "\n"
		     "multipole_degree  = " << options.multipole_degree << "\n";
		if (!WriteText(root / "dust.in", s.str(), error)) return false;
		result.files.push_back("dust.in");
	}

	// ---- postprocessing ----------------------------------------------------

	if (options.write_postprocessing) {
		std::ostringstream p;
		p << "! DUST postprocessor input, written by Aerotelic.\n"
		     "! Run:  dust_post dust_post.in      (after dust)\n\n"
		     "basename      = ./Postprocessing/post\n"
		     "data_basename = ./Output/" << options.basename << "\n\n"
		     "! The flow field and the wake, as VTK files one per step: open the\n"
		     "! post_viz-....vtu series in ParaView.\n"
		     "analysis = {\n"
		     "  type      = viz\n"
		     "  name      = viz\n"
		     "  format    = vtk\n"
		     "  start_res = 1\n"
		     "  end_res   = " << (result.timesteps + 1) << "\n"
		     "  step_res  = 1\n"
		     "  wake      = T\n"
		     "  variable  = Vorticity\n"
		     "  variable  = Pressure\n"
		     "}\n\n"
		     "! Forces and moments on everything, per time step.\n"
		     "analysis = {\n"
		     "  type      = integral_loads\n"
		     "  name      = loads_total\n"
		     "  start_res = 1\n"
		     "  end_res   = " << (result.timesteps + 1) << "\n"
		     "  step_res  = 1\n"
		     "  format    = dat\n"
		     "  reference_tag = 0\n";
		for (const std::string& n : names) p << "  component = " << n << "\n";
		p << "}\n";

		// ... and on each body on its own, which is what makes a load
		// breakdown by component possible at all.
		for (const std::string& n : names) {
			p << "\nanalysis = {\n"
			     "  type      = integral_loads\n"
			     "  name      = loads_" << n << "\n"
			     "  start_res = 1\n"
			     "  end_res   = " << (result.timesteps + 1) << "\n"
			     "  step_res  = 1\n"
			     "  format    = dat\n"
			     "  reference_tag = 0\n"
			     "  component = " << n << "\n"
			     "}\n";
		}
		if (!WriteText(root / "dust_post.in", p.str(), error)) return false;
		result.files.push_back("dust_post.in");
	}

	// ---- a note in the case itself -------------------------------------------

	{
		std::ostringstream r;
		r << "DUST case written by Aerotelic\n"
		     "==============================\n\n"
		     "Run these three, in this order, from this directory:\n\n"
		  << DustRunCommands()
		  << "\n"
		     "  dust_pre   reads dust_pre.in and the components/ files, and\n"
		     "             collects the meshes in geometry/ into geo_input.h5.\n"
		     "  dust       reads dust.in and solves, writing one result file per\n"
		     "             time step into Output/.\n"
		     "  dust_post  reads dust_post.in and turns those into the VTK series\n"
		     "             and the load tables in Postprocessing/.\n\n"
		     "On Windows the executables are in the DUST install directory; put\n"
		     "it on PATH first. The three names are dust_pre.exe, dust.exe and\n"
		     "dust_post.exe.\n\n"
		     "This case\n"
		     "---------\n"
		  << "  " << result.components << " component(s), " << result.panels << " panels, " << result.nodes << " nodes\n";
		if (result.rotors > 0) {
			r << "  " << result.rotors << " rotor frame(s): one blade each, replicated and spun by DUST\n"
			     "  (the panel count above already counts every blade DUST will make)\n";
		}
		r << "  free stream " << Num(speed, 6) << " at " << Num(options.alpha_deg, 4) << " deg incidence\n"
		  << "  " << result.timesteps << " steps of " << Num(result.dt, 6) << " s to t = " << Num(result.tend, 6) << "\n"
		  << "  dense panel system about " << Num(result.linear_system_mb, 4) << " MB\n\n"
		     "Results land in Output/ and Postprocessing/. Both are emptied by\n"
		     "nothing: delete them yourself between runs if you want a clean set.\n";
		if (!WriteText(root / "README.txt", r.str(), error)) return false;
		result.files.push_back("README.txt");
	}

	return true;
}

}  // namespace aerotelic
