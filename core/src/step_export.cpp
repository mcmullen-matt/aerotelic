// WingModel::ExportStep() and FuselageModel::ExportStep() live here,
// deliberately separated from wing_model.cpp/fuselage_model.cpp into their
// own translation unit.
//
// STEPControl_Writer pulls in OCCT's TKDESTEP/TKXSBase/TKDE toolkits,
// which transitively need TKXCAF/TKVCAF/TKCAF/TKLCAF/TKCDF/TKV3d/TKService
// and, through those, third-party imaging libraries (FreeImage, libraw)
// that themselves depend on zlib/freetype/libpng16/tiff -- names a host
// application (Unreal) is very likely to already load its own,
// ABI-different copies of for texture/font work. Loading OCCT's
// conda-forge-built copies over Unreal's own caused a real
// STATUS_HEAP_CORRUPTION crash once Unreal's code used the shadowed copy
// (see AerotelicUE.Build.cs's notes on this).
//
// A static library only pulls in the .obj files its linked executable
// actually references. By keeping ExportStep() out of wing_model.obj/
// fuselage_model.obj entirely, a consumer that never calls ExportStep()
// (the UE module, for now) never needs step_export.obj either, so it
// never needs TKDESTEP resolved and never touches that collision-prone
// dependency chain at all. A consumer that DOES call ExportStep() (the
// standalone fixture-check test programs) pulls this file in exactly as
// before, with no behavior change -- those are plain console executables
// with no bundled zlib/freetype/libpng16/tiff of their own to collide with.
//
// Revisit this split when a real "Export STEP" UI feature needs calling
// from inside the UE module itself, and solve the DLL-collision problem
// properly then (e.g. an isolated subprocess, or SxS/API-Sets redirection
// scoped to just those four names) rather than carrying the risk for a
// feature nothing uses yet.

#include <stdexcept>
#include <string>

#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/wing_model.hpp"

namespace aerotelic {

void WingModel::ExportStep(const std::string& filepath, int n_span, double station_tol_override) const {
	TopoDS_Shape shape = BuildSolid(n_span, true, station_tol_override);
	STEPControl_Writer writer;
	writer.Transfer(shape, STEPControl_AsIs);
	IFSelect_ReturnStatus status = writer.Write(filepath.c_str());
	if (status != IFSelect_RetDone) throw std::runtime_error("STEP export failed (status=" + std::to_string(static_cast<int>(status)) + ")");
}

void FuselageModel::ExportStep(const std::string& path, int n_stations, int n_pts, std::optional<double> station_tol) const {
	TopoDS_Shape solid = BuildSolid(n_stations, n_pts, station_tol);
	STEPControl_Writer writer;
	writer.Transfer(solid, STEPControl_AsIs);
	if (writer.Write(path.c_str()) != IFSelect_RetDone) throw std::runtime_error("STEP export failed: " + path);
}

} // namespace aerotelic
