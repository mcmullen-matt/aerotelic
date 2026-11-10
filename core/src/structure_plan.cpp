#include "aerotelic/structure_plan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerotelic {

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

std::vector<int> RimOfRing(const std::vector<int>& ring) {
	std::vector<int> rim;
	for (int idx : ring) if (rim.empty() || rim.back() != idx) rim.push_back(idx);
	while (rim.size() > 1 && rim.back() == rim.front()) rim.pop_back();
	// A ring that retraces itself (a knife edge) repeats vertices out of
	// order too: such a rim bounds no area.
	std::vector<int> sorted = rim;
	std::sort(sorted.begin(), sorted.end());
	if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) return {};
	if (rim.size() < 3) return {};
	return rim;
}

int RingIndexAt(const std::vector<double>& ring_v, double c) {
	for (size_t i = 0; i < ring_v.size(); ++i) if (std::fabs(ring_v[i] - c) <= 1e-9) return static_cast<int>(i);
	return -1;
}

double MedianEdge(const TriMesh& m) {
	std::vector<double> e;
	e.reserve(m.triangles.size() * 3);
	for (const auto& t : m.triangles) {
		e.push_back(Length(m.vertices[t[1]] - m.vertices[t[0]]));
		e.push_back(Length(m.vertices[t[2]] - m.vertices[t[1]]));
		e.push_back(Length(m.vertices[t[0]] - m.vertices[t[2]]));
	}
	if (e.empty()) return 0.0;
	std::nth_element(e.begin(), e.begin() + e.size() / 2, e.end());
	return e[e.size() / 2];
}

TriMesh StripMesh(const std::vector<Vec3>& lower, const std::vector<Vec3>& upper, int across) {
	TriMesh m;
	const int ns = static_cast<int>(lower.size());
	across = std::max(across, 2);
	const double step = 1.0 / (across - 1);
	for (int i = 0; i < ns; ++i) {
		for (int j = 0; j < across; ++j) {
			const double t = j == across - 1 ? 1.0 : j * step;
			m.vertices.push_back(lower[i] + (upper[i] - lower[i]) * t);
		}
	}
	auto at = [&](int i, int j) { return i * across + j; };
	for (int i = 0; i + 1 < ns; ++i) {
		for (int j = 0; j + 1 < across; ++j) {
			m.triangles.push_back({at(i, j), at(i + 1, j), at(i + 1, j + 1)});
			m.triangles.push_back({at(i, j), at(i + 1, j + 1), at(i, j + 1)});
		}
	}
	m.RemoveDegenerateTriangles();
	return m;
}

// ---------------------------------------------------------------------
// FrameStructureSpec
// ---------------------------------------------------------------------

std::vector<FrameStringerSpec> FrameStructureSpec::DefaultStringers() {
	std::vector<FrameStringerSpec> out;
	for (int k = 0; k < 4; ++k) {
		FrameStringerSpec s;
		s.name = "Longeron " + std::to_string(k + 1);
		s.around = 0.125 + 0.25 * k;
		out.push_back(s);
	}
	return out;
}

FrameStructureSpec FrameStructureSpec::Default() {
	return FrameStructureSpec();
}

bool FrameStructureSpec::IsEmpty() const {
	return stringers.empty() && frames.between <= 0 && frames.extra.empty();
}

std::vector<double> FrameStructureSpec::FrameStations() const {
	std::vector<double> out = frames.extra;
	for (int i = 1; i <= frames.between; ++i) out.push_back(static_cast<double>(i) / (frames.between + 1));
	std::vector<double> kept;
	for (double s : out) if (s > 1e-6 && s < 1.0 - 1e-6) kept.push_back(s);
	std::sort(kept.begin(), kept.end());
	std::vector<double> merged;
	for (double s : kept) if (merged.empty() || s - merged.back() > 1e-6) merged.push_back(s);
	return merged;
}

void FrameStructureSpec::Validate() const {
	if (frames.between < 0 || frames.between > 400) throw std::invalid_argument("the number of frames between the ends must be 0 to 400");
	if (!(frames.height >= 0.0)) throw std::invalid_argument("the frames' height must not be negative");
	if (!(frames.thickness >= 0.0)) throw std::invalid_argument("the frames' thickness must not be negative");
	for (double s : frames.extra) {
		if (!(s >= 0.0 && s <= 1.0)) throw std::invalid_argument("frame station " + std::to_string(s) + " lies outside the length (0 to 1)");
	}
	for (const FrameStringerSpec& st : stringers) {
		if (!(st.around >= 0.0 && st.around < 1.0)) throw std::invalid_argument("stringer '" + st.name + "': the position round the section must lie in [0, 1)");
		if (!(st.x_start >= 0.0 && st.x_end <= 1.0 && st.x_end > st.x_start)) throw std::invalid_argument("stringer '" + st.name + "': the extent must run 0 <= start < end <= 1");
		if (!(st.thickness >= 0.0) || !(st.height >= 0.0)) throw std::invalid_argument("stringer '" + st.name + "': the thickness and the height must not be negative");
	}
}

// ---------------------------------------------------------------------
// PlanFrameStructure
// ---------------------------------------------------------------------

BodyStructurePlan PlanFrameStructure(const std::string& body_name, const TriMesh& body, const LoftRings& rings,
	const FrameStructureSpec& spec, const FrameRingSet& set, const Vec3& plane_normal, const std::vector<std::string>& notes) {
	BodyStructurePlan plan;
	plan.body = body_name;
	plan.rib_layer = "Frames";
	plan.rib_thickness = spec.frames.thickness;
	plan.rib_material = spec.frames.material;
	plan.rib_size = MedianEdge(body);
	plan.notes = notes;

	// The frames.
	int frame_no = 0;
	for (size_t k = 0; k < set.stations.size(); ++k) {
		const int ro = k < set.outer.size() ? set.outer[k] : -1;
		if (ro < 0 || ro >= static_cast<int>(rings.rings.size())) {
			plan.notes.push_back("frame at x/L " + std::to_string(set.stations[k]) + ": the loft has no ring there");
			continue;
		}
		RibPlan rib;
		rib.eta = set.stations[k];
		rib.name = "Frame " + std::to_string(++frame_no);
		rib.height = spec.frames.height;
		rib.rim = RimOfRing(rings.rings[ro]);
		if (rib.rim.empty()) {
			--frame_no;
			plan.notes.push_back("frame at x/L " + std::to_string(set.stations[k]) + ": the section there has no area (an apex)");
			continue;
		}
		const int ri = k < set.inner.size() ? set.inner[k] : -1;
		if (ri >= 0 && ri < static_cast<int>(rings.rings.size())) {
			std::vector<int> hole = RimOfRing(rings.rings[ri]);
			if (!hole.empty() && hole != rib.rim) rib.holes.push_back(std::move(hole));
		}
		plan.ribs.push_back(std::move(rib));
	}

	// The stringers: a strip from the skin rings' vertices at the
	// stringer's position to points `height` in from them.
	for (const FrameStringerSpec& st : spec.stringers) {
		std::vector<Vec3> feet, inner;
		int missing = 0;
		for (const auto& [x, ri] : set.skin_rings) {
			if (x < st.x_start - 1e-9 || x > st.x_end + 1e-9) continue;
			if (ri < 0 || ri >= static_cast<int>(rings.rings.size())) continue;
			const std::vector<int>& ring = rings.rings[ri];
			const std::vector<double>& rv = rings.ring_v[ri];
			const int j = RingIndexAt(rv, st.around);
			if (j < 0) { ++missing; continue; }
			const int nr = static_cast<int>(ring.size());
			if (nr < 3) continue;
			const Vec3& pt = body.vertices[ring[j]];
			// An apex ring repeats one vertex: no tangent, no stringer there.
			const Vec3& prev = body.vertices[ring[(j - 1 + nr) % nr]];
			const Vec3& next = body.vertices[ring[(j + 1) % nr]];
			Vec3 tangent = next - prev;
			if (Length(tangent) <= 1e-12) continue;
			Vec3 inward = Cross(plane_normal, tangent);
			const double len = Length(inward);
			if (len <= 1e-300) continue;
			inward = inward * (1.0 / len);
			Vec3 centroid{0, 0, 0};
			int n_distinct = 0;
			for (int idx : ring) { centroid = centroid + body.vertices[idx]; ++n_distinct; }
			centroid = centroid * (1.0 / std::max(n_distinct, 1));
			if (Dot(inward, centroid - pt) < 0.0) inward = inward * -1.0;
			feet.push_back(pt);
			inner.push_back(pt + inward * st.height);
		}
		if (missing > 0) plan.notes.push_back("stringer '" + st.name + "': " + std::to_string(missing) + " ring(s) along its run have no vertex at its position");
		if (feet.size() < 2) {
			plan.notes.push_back("stringer '" + st.name + "': fewer than two stations in its run");
			continue;
		}
		ShellPlan sh;
		sh.name = st.name;
		sh.thickness = st.thickness;
		sh.material = st.material;
		sh.mesh = StripMesh(feet, inner, 2);
		plan.stringers.push_back(std::move(sh));
	}
	return plan;
}

} // namespace aerotelic
