#include "aerotelic/aero_loads.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace aerotelic::aero {

namespace {

// A uniform grid over the sample centres, so each face looks at the panels
// of its own cell and the shells round it rather than at all of them.
struct SampleGrid {
	Vec3 origin;
	double cell = 1.0;
	int nx = 1, ny = 1, nz = 1;
	std::unordered_map<int64_t, std::vector<int>> cells;

	int64_t Key(int i, int j, int k) const { return (static_cast<int64_t>(i) * 73856093) ^ (static_cast<int64_t>(j) * 19349663) ^ (static_cast<int64_t>(k) * 83492791); }
	void Index(const Vec3& p, int& i, int& j, int& k) const {
		i = std::clamp(static_cast<int>(std::floor((p.x - origin.x) / cell)), 0, nx - 1);
		j = std::clamp(static_cast<int>(std::floor((p.y - origin.y) / cell)), 0, ny - 1);
		k = std::clamp(static_cast<int>(std::floor((p.z - origin.z) / cell)), 0, nz - 1);
	}
};

int64_t GridKey(int network, int row, int col) {
	return (static_cast<int64_t>(network) << 42) ^ (static_cast<int64_t>(row + (1 << 20)) << 21) ^ static_cast<int64_t>(col + (1 << 20));
}

} // namespace

std::vector<double> MapPressures(const std::vector<PressureSample>& samples, const std::vector<Vec3>& face_centres, const std::vector<Vec3>& face_normals,
	const MapOptions& options, MapStats* stats, std::vector<unsigned char>* mapped) {
	const size_t nf = face_centres.size();
	std::vector<double> out(nf, 0.0);
	if (mapped) mapped->assign(nf, 0);
	MapStats st;
	st.faces = static_cast<int>(nf);
	if (samples.empty() || nf == 0) {
		st.unmapped = st.faces;
		if (stats) *stats = st;
		return out;
	}

	// The samples' extent and a cell size: the largest panel's own size,
	// so a face's nearest panel is in its cell or the shell round it.
	Vec3 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
	Vec3 hi{-lo.x, -lo.y, -lo.z};
	double max_size = 0.0;
	for (const PressureSample& s : samples) {
		lo = {std::min(lo.x, s.centre.x), std::min(lo.y, s.centre.y), std::min(lo.z, s.centre.z)};
		hi = {std::max(hi.x, s.centre.x), std::max(hi.y, s.centre.y), std::max(hi.z, s.centre.z)};
		max_size = std::max(max_size, std::sqrt(std::max(s.area, 0.0)));
	}
	const Vec3 extent = hi - lo;
	const double diag = Length(extent);
	const double max_distance = options.max_distance > 0.0 ? options.max_distance : 0.1 * diag;
	SampleGrid grid;
	grid.cell = std::max(max_size * 2.0, diag * 1e-3);
	if (!(grid.cell > 0.0)) grid.cell = 1.0;
	grid.origin = lo;
	grid.nx = std::max(1, std::min(256, static_cast<int>(std::ceil(extent.x / grid.cell)) + 1));
	grid.ny = std::max(1, std::min(256, static_cast<int>(std::ceil(extent.y / grid.cell)) + 1));
	grid.nz = std::max(1, std::min(256, static_cast<int>(std::ceil(extent.z / grid.cell)) + 1));
	// The cell may have grown where the extent is capped at 256 cells.
	grid.cell = std::max({grid.cell, extent.x / grid.nx, extent.y / grid.ny, extent.z / grid.nz});
	for (size_t s = 0; s < samples.size(); ++s) {
		int i, j, k;
		grid.Index(samples[s].centre, i, j, k);
		grid.cells[grid.Key(i, j, k)].push_back(static_cast<int>(s));
	}
	std::unordered_map<int64_t, int> by_grid;
	by_grid.reserve(samples.size());
	for (size_t s = 0; s < samples.size(); ++s) by_grid[GridKey(samples[s].network, samples[s].row, samples[s].col)] = static_cast<int>(s);

	// Whether a sample describes the face's side, and the pressure it gives it.
	auto side_cp = [&](const PressureSample& s, const Vec3& n, double& cp) {
		const double d = Dot(s.normal, n);
		if (s.two_sided) {
			cp = d >= 0.0 ? s.cp_upper : s.cp_lower;
			return true;
		}
		if (d < options.normal_min) return false;
		cp = s.cp_upper;
		return true;
	};

	const int max_shell = std::max(1, static_cast<int>(std::ceil(max_distance / grid.cell)));
	double sum_distance = 0.0;
	for (size_t f = 0; f < nf; ++f) {
		const Vec3& c = face_centres[f];
		const Vec3 n = Normalized(face_normals[f]);
		int ci, cj, ck;
		grid.Index(c, ci, cj, ck);
		// Expanding shells of cells until an acceptable panel is found and
		// one more shell has been looked at (a nearer one may sit across a
		// cell boundary), or the distance limit is passed.
		int best = -1;
		double best_d2 = std::numeric_limits<double>::max();
		double best_cp = 0.0;
		for (int shell = 0; shell <= max_shell + 1; ++shell) {
			if (best >= 0 && shell > 0) {
				// Everything in later shells is at least (shell - 1) cells away.
				const double reach = (shell - 1) * grid.cell;
				if (reach * reach > best_d2) break;
			}
			for (int i = ci - shell; i <= ci + shell; ++i) {
				for (int j = cj - shell; j <= cj + shell; ++j) {
					for (int k = ck - shell; k <= ck + shell; ++k) {
						if (std::max({std::abs(i - ci), std::abs(j - cj), std::abs(k - ck)}) != shell) continue;
						if (i < 0 || j < 0 || k < 0 || i >= grid.nx || j >= grid.ny || k >= grid.nz) continue;
						auto it = grid.cells.find(grid.Key(i, j, k));
						if (it == grid.cells.end()) continue;
						for (int s : it->second) {
							const double d2 = DistanceSq(samples[s].centre, c);
							if (d2 >= best_d2) continue;
							double cp;
							if (!side_cp(samples[s], n, cp)) continue;
							best = s;
							best_d2 = d2;
							best_cp = cp;
						}
					}
				}
			}
		}
		if (best < 0 || best_d2 > max_distance * max_distance) {
			++st.unmapped;
			continue;
		}
		// The inverse-distance mean over the nearest panel's grid block.
		const PressureSample& b = samples[best];
		const double eps = 1e-6 * std::max(b.area, 1e-300);
		double wsum = 0.0, vsum = 0.0;
		for (int di = -options.neighbourhood; di <= options.neighbourhood; ++di) {
			for (int dj = -options.neighbourhood; dj <= options.neighbourhood; ++dj) {
				int s = best;
				if (di != 0 || dj != 0) {
					auto it = by_grid.find(GridKey(b.network, b.row + di, b.col + dj));
					if (it == by_grid.end()) continue;
					s = it->second;
				}
				double cp;
				if (!side_cp(samples[s], n, cp)) continue;
				const double w = 1.0 / (DistanceSq(samples[s].centre, c) + eps);
				wsum += w;
				vsum += w * cp;
			}
		}
		out[f] = wsum > 0.0 ? vsum / wsum : best_cp;
		if (mapped) (*mapped)[f] = 1;
		++st.mapped;
		const double d = std::sqrt(best_d2);
		st.max_distance = std::max(st.max_distance, d);
		sum_distance += d;
	}
	st.mean_distance = st.mapped > 0 ? sum_distance / st.mapped : 0.0;
	if (stats) *stats = st;
	return out;
}

} // namespace aerotelic::aero
