#pragma once

#include "CoreMinimal.h"
#include <TopoDS_Shape.hxx>

#include "Entity/AerotelicBooleanModel.h"
#include "aerotelic/pdf3d.hpp"

// File -> Export STL / Export STEP / Export 3D PDF (main_window.py
// _export_stl_dialog / _export_step_dialog): the bodies behind the dialogs.
namespace AerotelicExport
{
	// A body chosen for export, with the shape to write.
	struct FBody
	{
		FString Label;
		TopoDS_Shape Shape;
	};

	// ---- STL ----

	struct FStlTriangle { FVector3f N, A, B, C; };

	// One body's run of triangles within FStlMesh::Triangles, named
	// "<entity>/<body>" ("Lattice 1/Skin", "Lattice 1/Lattice"): an ASCII
	// STL is written as one `solid` per part, so a mesher or FE
	// pre-processor can pick the parts up separately (and give a lattice's
	// shell and infill different materials). Binary STL has no parts.
	struct FStlPart
	{
		FString Name;
		int32 First = 0;
		int32 Count = 0;
	};

	// The union of the chosen bodies' triangle meshes: what the file holds.
	struct FStlMesh
	{
		TArray<FStlTriangle> Triangles;
		TArray<FStlPart> Parts;
		int32 Bodies = 0;
		int64 Nodes = 0;
		bool IsEmpty() const { return Triangles.Num() == 0; }
	};

	// The operands' placed bodies meshed at the given density: each
	// model's own mesh within the linear deflection (model units) of its
	// surface, or -- for a body that needs OCCT, such as a boolean result
	// -- OCCT's tessellation at the linear and angular (degrees)
	// deflections. Bodies that cannot be built are listed in OutFailures.
	FStlMesh MeshForStl(const TArray<FAerotelicBooleanOperand>& Operands, const TArray<FString>& Labels,
		double LinearDeflection, double AngularDeflectionDeg, TArray<FString>& OutFailures);

	// Writes the triangles, binary or ASCII.
	bool WriteStl(const FStlMesh& Mesh, const FString& Path, bool bAscii, FString& OutError);

	// ---- STEP ----

	// WingDesigner's EXPORT_STATION_TOL (0.2 mm): finer than the
	// interactive loft, which is why an export is not a screenshot of the
	// viewport.
	double ExportStationTolerance();

	// The shape to write for one body (_step_body_shape): an entity rebuilt
	// at StationTol (unset: as it stands) and then placed; a mirror
	// reflected from its source built the same way; a boolean result as it
	// stands. A lattice has no B-rep (it is a mesh: export it as STL).
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

	// ---- 3D PDF ----

	// The operands' placed bodies as the parts of a 3D PDF: the same
	// meshes STL export writes, at the same linear (and, for a body that
	// needs OCCT, angular) deflection -- but kept per body and INDEXED,
	// not as a triangle soup, because the U3D writer welds positions and
	// smooths normals across shared vertices and a soup would have none.
	// Each part is labelled "<entity>/<body>" and carries the colour the
	// body is drawn in, so the PDF opens in the scene's own colours.
	// Bodies that cannot be built are listed in OutFailures.
	std::vector<aerotelic::Pdf3dPart> MeshForPdf3d(const TArray<FAerotelicBooleanOperand>& Operands, const TArray<FString>& Labels,
		double LinearDeflection, double AngularDeflectionDeg, TArray<FString>& OutFailures);

	// Writes the parts as a one-page 3D PDF (core/include/aerotelic/
	// pdf3d.hpp: a U3D stream in a PDF 3D annotation). bCompress filters
	// the U3D stream with the ENGINE's zlib -- never a second copy of that
	// name in the process, which is the collision AerotelicUE.Build.cs
	// documents. False with the reason on failure.
	bool WritePdf3d(const std::vector<aerotelic::Pdf3dPart>& Parts, const FString& Path, aerotelic::Pdf3dOptions Options,
		bool bCompress, FString& OutError, aerotelic::Pdf3dResult* OutStats = nullptr);
}
