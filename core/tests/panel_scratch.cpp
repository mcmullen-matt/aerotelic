// Scratch experiments on the lifting-loft path (2026-11-13): which part of
// a capped seam loft makes the influence matrix singular, and which part
// of the split-and-ring-wake nacelle makes its solve blow up.
#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/wing_panels.hpp"

using namespace aerotelic;
using namespace aerotelic::panel;

namespace {

const double kPi = 3.14159265358979323846;

void Report(const char* label, const std::vector<Network>& nets, const Vec3& vinf, const Options& o) {
	try {
		Solution s = Solve(nets, vinf, o);
		double cpmin = 1.0, cpmax = -1.0;
		bool finite = true;
		for (const NetworkResult& nr : s.networks) {
			if (nr.type == NetworkType::DoubletWake || nr.type == NetworkType::DoubletWakeConstant) continue;
			for (const PanelResult& p : nr.panels) { finite = finite && std::isfinite(p.cp_upper); cpmin = std::min(cpmin, p.cp_upper); cpmax = std::max(cpmax, p.cp_upper); }
		}
		std::printf("%-44s %d nets, %d unknowns, pivot %.2e, Cp %.3f .. %.3f, C_F (%.4f, %.4f, %.4f), C_L wake %s%s\n", label, (int)nets.size(), s.unknowns, s.pivot_ratio, cpmin, cpmax,
			s.force_coefficient.x, s.force_coefficient.y, s.force_coefficient.z, s.lift_from_wake ? std::to_string(*s.lift_from_wake).c_str() : "-", finite ? "" : " NOT FINITE");
	} catch (const std::exception& e) {
		std::printf("%-44s EXCEPTION %s\n", label, e.what());
	}
}

NamedLoft Slab(const std::shared_ptr<Airfoil>& af, double span, bool caps) {
	NamedLoft loft;
	loft.name = "slab";
	loft.source.u_min = -0.5 * span;
	loft.source.u_max = 0.5 * span;
	loft.source.closed_curve = false;
	loft.source.cap_start = caps;
	loft.source.cap_end = caps;
	loft.source.v_seeds = {af->LeadingEdgeParameter()};
	loft.source.section = [af](double u, const std::vector<double>& v) {
		Mat xy = af->nurbs_curve().Evaluate(v);
		Mat out(xy.rows(), 3);
		for (int j = 0; j < xy.rows(); ++j) { out(j, 0) = xy(j, 0); out(j, 1) = u; out(j, 2) = xy(j, 1); }
		return out;
	};
	loft.trailing_edge = LoftTrailingEdge::Seam();
	return loft;
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	const double alpha = 4.0 * kPi / 180.0;
	const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
	std::shared_ptr<Airfoil> af = Airfoil::Naca4("0012");
	Options o;
	o.reference_area = 6.0;
	BodyPaneling bp;
	bp.along = 12; bp.around = 24;
	bp.wake_direction = vinf;

	// ---- the slab ----
	{
		BodyPaneling src = bp; src.wake = false; src.sources = true;
		Report("slab: caps, sources, no wake", LoftNetworks(Slab(af, 6.0, true), "slab", src, vinf), vinf, o);
		BodyPaneling dbl = bp; dbl.wake = false; dbl.sources = false;
		Report("slab: caps, doublet, no wake", LoftNetworks(Slab(af, 6.0, true), "slab", dbl, vinf), vinf, o);
		Report("slab: no caps, doublet, no wake", LoftNetworks(Slab(af, 6.0, false), "slab", dbl, vinf), vinf, o);
		Report("slab: no caps, doublet, wake", LoftNetworks(Slab(af, 6.0, false), "slab", bp, vinf), vinf, o);
		Report("slab: caps, doublet, wake", LoftNetworks(Slab(af, 6.0, true), "slab", bp, vinf), vinf, o);
		// Caps and wake, the wake polyline extended to the apexes as the
		// wing's collapsed tip does.
		{
			std::vector<Network> nets = LoftNetworks(Slab(af, 6.0, true), "slab", bp, vinf);
			const Network& body = nets[0];
			std::vector<Vec3> te;
			for (int i = 1; i <= body.rows; ++i) te.push_back(body.Corner(i, 1));
			nets[1] = WakeNetwork("slab wake", te, vinf, 20.0 * 6.0);
			Report("slab: caps, wake to the apexes", nets, vinf, o);
		}
		// Fewer columns, more rows.
		BodyPaneling coarse = bp; coarse.along = 8; coarse.around = 12;
		Report("slab: caps, wake, 8 x 12", LoftNetworks(Slab(af, 6.0, true), "slab", coarse, vinf), vinf, o);
		Report("slab: no caps, wake, 8 x 12", LoftNetworks(Slab(af, 6.0, false), "slab", coarse, vinf), vinf, o);
	}

	// ---- the nacelle ----
	{
		LoftedNacelleSpec ns;
		LoftedNacelleModel nac(ns);
		Options on;
		on.reference_area = 1.0;
		const Vec3 v0{1.0, 0.0, 0.0};
		BodyPaneling np;
		np.along = 10; np.around = 16;
		np.wake_direction = v0;
		BodyPaneling src = np; src.wake = false; src.sources = true;
		Report("nacelle: sources, no wake (old)", BodyNetworks(nac, "nacelle", src, v0), v0, on);
		BodyPaneling dbl = np; dbl.wake = false; dbl.sources = false;
		Report("nacelle: doublet, no wake", BodyNetworks(nac, "nacelle", dbl, v0), v0, on);
		Report("nacelle: mean sheet + ring wake", BodyNetworks(nac, "nacelle", np, v0), v0, on);
		{
			std::vector<Network> nets = BodyNetworks(nac, "nacelle", np, v0);
			nets.pop_back();
			Report("nacelle: mean sheet, no wake", nets, v0, on);
		}
		const double a5 = 5.0 * kPi / 180.0;
		const Vec3 v5{std::cos(a5), 0.0, std::sin(a5)};
		BodyPaneling np5 = np; np5.wake_direction = v5;
		Report("nacelle: mean sheet + ring wake, 5 deg", BodyNetworks(nac, "nacelle", np5, v5), v5, on);
		NacelleSpec s2;
		NacelleModel nac2(s2);
		Report("simple nacelle: sources, no wake (old)", BodyNetworks(nac2, "nacelle", src, v0), v0, on);
		Report("simple nacelle: doublet, no wake", BodyNetworks(nac2, "nacelle", dbl, v0), v0, on);
		Report("simple nacelle: mean sheet + ring wake", BodyNetworks(nac2, "nacelle", np, v0), v0, on);
		Report("simple nacelle: mean sheet + wake, 5 deg", BodyNetworks(nac2, "nacelle", np5, v5), v5, on);
	}
	return 0;
}
