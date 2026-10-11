#pragma once

#include "CoreMinimal.h"
#include <TopoDS_Shape.hxx>

#include "Entity/AerotelicBooleanModel.h"

// File -> Export STL / Export STEP (main_window.py _export_stl_dialog /
// _export_step_dialog): the bodies behind the dialogs.
namespace AerotelicExport
{
	// A body chosen for export, with the shape to write.
	struct FBody
	{
		FString Label;
		TopoDS_Shape Shape;
	};

	// ---- STL ----

	// Copies of the operands' placed shapes meshed at the given density
	// (linear deflection in model units, angular in degrees), as one
	// compound: the union of the per-body meshes, which is what the file
	// holds. Bodies that cannot be built are listed in OutFailures.
	// OutNodes / OutTriangles are the mesh statistics. Null if nothing
	// could be built.
	TopoDS_Shape MeshForStl(const TArray<FAerotelicBooleanOperand>& Operands, const TArray<FString>& Labels,
		double LinearDeflection, double AngularDeflectionDeg, int32& OutBodies, int64& OutNodes, int64& OutTriangles, TArray<FString>& OutFailures);

	// Writes the triangulation the compound carries, binary or ASCII.
	bool WriteStl(const TopoDS_Shape& Meshed, const FString& Path, bool bAscii, FString& OutError);

	// ---- STEP ----

	// WingDesigner's EXPORT_STATION_TOL (0.2 mm): finer than the
	// interactive loft, which is why an export is not a screenshot of the
	// viewport.
	double ExportStationTolerance();

	// The shape to write for one body (_step_body_shape): an entity rebuilt
	// at StationTol (unset: as it stands) and then placed; a mirror
	// reflected from its source built the same way; a boolean result as it
	// stands.
	TopoDS_Shape StepBodyShape(const FAerotelicBooleanOperand& Operand, TOptional<double> StationTol, FString& OutError);

	// Writes the bodies to one STEP file through aerotelic_step_writer, a
	// separate process (its STEP toolkits must not load into the engine --
	// see core/src/step_export.cpp). bNamed writes an XCAF product
	// structure so each body arrives as a named part; otherwise one plain
	// compound. False with the reason on failure.
	bool WriteStep(const TArray<FBody>& Bodies, const FString& Path, bool bNamed, FString& OutError);

	// Where the writer is: Binaries/Win64/StepWriter beside the application
	// (staged by tools/stage_step_writer.ps1 and the packaged build), else
	// the core build tree in a developer checkout. Empty if neither exists.
	FString FindStepWriter();
}
