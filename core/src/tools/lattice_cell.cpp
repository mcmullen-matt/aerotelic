// aerotelic_lattice_cell: voxelises one period of a lattice (lattice.hpp's
// SampleUnitCell) and writes it out for ue/tools/homogenize.py, so the
// homogenizer can be checked against the real lattice types outside the
// application (ue/tools/homogenize_check.py). The application itself
// samples in-process (Entity/AerotelicStructuralFea.cpp) and does not run
// this program.
//
//   aerotelic_lattice_cell <out> [--family tpms|strut|auxetic|honeycomb]
//       [--type NAME] [--phase network|sheet|iso-surface]
//       [--fraction RHO] [--wall T] [--cell L] [--aspect A] [--axis x|y|z]
//       [--voxels N]
//
// Writes <out>.json ({"nx", "ny", "nz", "period": [x, y, z],
// "solid_fraction", "file": "<out>.f32"}) and <out>.f32, the field at the
// voxel centres as little-endian float32 in (k * ny + j) * nx + i order,
// <= 0 inside the material. Exit code 0 on success; otherwise 1 with the
// reason on stderr.

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "aerotelic/lattice.hpp"

namespace {

int Fail(const std::string& why) {
	std::fprintf(stderr, "%s\n", why.c_str());
	return 1;
}

std::string BaseName(const std::string& path) {
	const size_t cut = path.find_last_of("/\\");
	return cut == std::string::npos ? path : path.substr(cut + 1);
}

} // namespace

int main(int argc, char** argv) {
	if (argc < 2) {
		return Fail("usage: aerotelic_lattice_cell <out> [--family F] [--type T] [--phase P] [--fraction RHO] [--wall T] [--cell L] [--aspect A] [--axis x|y|z] [--voxels N]");
	}
	const std::string out = argv[1];
	aerotelic::LatticeSpec spec;
	spec.cell_size = 1.0;
	int voxels = 32;
	try {
		for (int i = 2; i + 1 < argc; i += 2) {
			const std::string key = argv[i], value = argv[i + 1];
			if (key == "--family") spec.family = aerotelic::ParseLatticeFamily(value);
			else if (key == "--type") spec.type = value;
			else if (key == "--phase") spec.phase = aerotelic::ParseTpmsPhase(value);
			else if (key == "--fraction") spec.volume_fraction = std::stod(value);
			else if (key == "--wall") { spec.wall_thickness = std::stod(value); spec.wall_sizing = aerotelic::WallSizing::Thickness; }
			else if (key == "--cell") spec.cell_size = std::stod(value);
			else if (key == "--aspect") spec.cell_aspect = std::stod(value);
			else if (key == "--axis") spec.axis = aerotelic::ParsePrismAxis(value);
			else if (key == "--voxels") voxels = std::stoi(value);
			else return Fail("unknown option " + key);
		}
		const aerotelic::UnitCellSamples cell = aerotelic::SampleUnitCell(spec, voxels);

		const std::string raw_path = out + ".f32", json_path = out + ".json";
		FILE* f = std::fopen(raw_path.c_str(), "wb");
		if (f == nullptr) return Fail("cannot write " + raw_path);
		const size_t written = std::fwrite(cell.values.data(), sizeof(float), cell.values.size(), f);
		std::fclose(f);
		if (written != cell.values.size()) return Fail("short write to " + raw_path);

		f = std::fopen(json_path.c_str(), "w");
		if (f == nullptr) return Fail("cannot write " + json_path);
		std::fprintf(f, "{\"nx\": %d, \"ny\": %d, \"nz\": %d, \"period\": [%.17g, %.17g, %.17g], \"solid_fraction\": %.17g, \"file\": \"%s\"}\n",
			cell.nx, cell.ny, cell.nz, cell.period.x, cell.period.y, cell.period.z, cell.solid_fraction, BaseName(raw_path).c_str());
		std::fclose(f);
		std::printf("%s %s: %d x %d x %d voxels, period %.6g x %.6g x %.6g, solid fraction %.4f\n",
			aerotelic::LatticeFamilyName(spec.family), spec.type.c_str(), cell.nx, cell.ny, cell.nz,
			cell.period.x, cell.period.y, cell.period.z, cell.solid_fraction);
		return 0;
	} catch (const std::exception& e) {
		return Fail(e.what());
	}
}
