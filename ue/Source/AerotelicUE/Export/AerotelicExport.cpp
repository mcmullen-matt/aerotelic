#include "AerotelicExport.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Trsf.hxx>

#include "aerotelic/feature_tree.hpp"
#include "aerotelic/wing_model.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/propeller_model.hpp"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicEntityActor.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicExport, Log, All);

namespace AerotelicExport
{
	namespace
	{
		// main_window.py _EXPORT_N_SPAN.
		constexpr int kExportNSpan = 200;

		struct FTriangle { FVector3f N, A, B, C; };

		// The triangles a meshed shape carries, with the face orientation
		// folded into the winding so the normals point outward.
		void CollectTriangles(const TopoDS_Shape& Shape, TArray<FTriangle>& Out, int64& OutNodes)
		{
			for (TopExp_Explorer FaceExp(Shape, TopAbs_FACE); FaceExp.More(); FaceExp.Next())
			{
				const TopoDS_Face& Face = TopoDS::Face(FaceExp.Current());
				TopLoc_Location Loc;
				Handle(Poly_Triangulation) Tri = BRep_Tool::Triangulation(Face, Loc);
				if (Tri.IsNull())
				{
					continue;
				}
				OutNodes += Tri->NbNodes();
				const gp_Trsf& Trsf = Loc.Transformation();
				const bool bReversed = Face.Orientation() == TopAbs_REVERSED;
				auto Node = [&](int32 i)
				{
					gp_Pnt P = Tri->Node(i);
					P.Transform(Trsf);
					return FVector3f((float)P.X(), (float)P.Y(), (float)P.Z());
				};
				for (int32 i = 1; i <= Tri->NbTriangles(); ++i)
				{
					int32 n1, n2, n3;
					Tri->Triangle(i).Get(n1, n2, n3);
					if (bReversed)
					{
						Swap(n2, n3);
					}
					FTriangle T;
					T.A = Node(n1); T.B = Node(n2); T.C = Node(n3);
					T.N = FVector3f::CrossProduct(T.B - T.A, T.C - T.A).GetSafeNormal();
					Out.Add(T);
				}
			}
		}
	}

	TopoDS_Shape MeshForStl(const TArray<FAerotelicBooleanOperand>& Operands, const TArray<FString>& Labels,
		double LinearDeflection, double AngularDeflectionDeg, int32& OutBodies, int64& OutNodes, int64& OutTriangles, TArray<FString>& OutFailures)
	{
		OutBodies = 0;
		OutNodes = 0;
		OutTriangles = 0;
		OutFailures.Reset();
		BRep_Builder Builder;
		TopoDS_Compound Compound;
		Builder.MakeCompound(Compound);
		for (int32 i = 0; i < Operands.Num(); ++i)
		{
			const FString Label = Labels.IsValidIndex(i) ? Labels[i] : FString::Printf(TEXT("Body %d"), i + 1);
			if (!Operands[i].Entity.IsValid())
			{
				OutFailures.Add(Label + TEXT(": no longer in the scene"));
				continue;
			}
			// A deep copy: meshing rewrites a shape's triangulation in place
			// and the entity's own result is what the viewport is showing.
			TopoDS_Shape Placed = Operands[i].Entity->GetPlacedShape(Operands[i].bMirror);
			if (Placed.IsNull())
			{
				OutFailures.Add(Label + TEXT(": produced no shape"));
				continue;
			}
			Builder.Add(Compound, BRepBuilderAPI_Copy(Placed).Shape());
			++OutBodies;
		}
		if (OutBodies == 0)
		{
			return TopoDS_Shape();
		}
		try
		{
			BRepMesh_IncrementalMesh Mesher(Compound, LinearDeflection, false, FMath::DegreesToRadians(AngularDeflectionDeg), true);
			if (!Mesher.IsDone())
			{
				OutFailures.Add(TEXT("meshing did not complete"));
				return TopoDS_Shape();
			}
		}
		catch (const std::exception& e)
		{
			OutFailures.Add(FString::Printf(TEXT("meshing failed: %s"), UTF8_TO_TCHAR(e.what())));
			return TopoDS_Shape();
		}
		TArray<FTriangle> Tris;
		CollectTriangles(Compound, Tris, OutNodes);
		OutTriangles = Tris.Num();
		return Compound;
	}

	bool WriteStl(const TopoDS_Shape& Meshed, const FString& Path, bool bAscii, FString& OutError)
	{
		TArray<FTriangle> Tris;
		int64 Nodes = 0;
		CollectTriangles(Meshed, Tris, Nodes);
		if (Tris.Num() == 0)
		{
			OutError = TEXT("the bodies carry no triangulation");
			return false;
		}
		TArray<uint8> Bytes;
		if (bAscii)
		{
			FString Text;
			Text.Reserve(Tris.Num() * 200);
			Text += TEXT("solid aerotelic\n");
			for (const FTriangle& T : Tris)
			{
				Text += FString::Printf(TEXT("  facet normal %e %e %e\n    outer loop\n      vertex %e %e %e\n      vertex %e %e %e\n      vertex %e %e %e\n    endloop\n  endfacet\n"),
					T.N.X, T.N.Y, T.N.Z, T.A.X, T.A.Y, T.A.Z, T.B.X, T.B.Y, T.B.Z, T.C.X, T.C.Y, T.C.Z);
			}
			Text += TEXT("endsolid aerotelic\n");
			if (!FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceAnsi))
			{
				OutError = FString::Printf(TEXT("could not write %s"), *Path);
				return false;
			}
			return true;
		}
		// Binary: an 80-byte header, the count, then 50 bytes per facet.
		Bytes.SetNumZeroed(80);
		const FString Header = TEXT("Aerotelic binary STL");
		for (int32 i = 0; i < Header.Len() && i < 80; ++i) Bytes[i] = (uint8)Header[i];
		const uint32 Count = (uint32)Tris.Num();
		Bytes.Append((const uint8*)&Count, 4);
		Bytes.Reserve(84 + Tris.Num() * 50);
		const uint16 Attr = 0;
		for (const FTriangle& T : Tris)
		{
			const float V[12] = { T.N.X, T.N.Y, T.N.Z, T.A.X, T.A.Y, T.A.Z, T.B.X, T.B.Y, T.B.Z, T.C.X, T.C.Y, T.C.Z };
			Bytes.Append((const uint8*)V, 48);
			Bytes.Append((const uint8*)&Attr, 2);
		}
		if (!FFileHelper::SaveArrayToFile(Bytes, *Path))
		{
			OutError = FString::Printf(TEXT("could not write %s"), *Path);
			return false;
		}
		return true;
	}

	double ExportStationTolerance()
	{
		return aerotelic::kExportStationTol;
	}

	TopoDS_Shape StepBodyShape(const FAerotelicBooleanOperand& Operand, TOptional<double> StationTol, FString& OutError)
	{
		using namespace aerotelic;
		using aerotelic::project_yaml::EntityType;
		if (!Operand.Entity.IsValid())
		{
			OutError = TEXT("no longer in the scene");
			return TopoDS_Shape();
		}
		AAerotelicEntityActor* Entity = Operand.Entity.Get();
		try
		{
			// As it stands, or a boolean result (always as it stands).
			if (!StationTol.IsSet() || Entity->GetEntityType() == EntityType::Boolean)
			{
				TopoDS_Shape S = Entity->GetPlacedShape(Operand.bMirror);
				if (S.IsNull()) OutError = TEXT("produced no shape");
				return S;
			}
			// Rebuilt at the export tolerance, then placed (and mirrored).
			const std::optional<double> Tol = StationTol.GetValue();
			TopoDS_Shape Solid;
			switch (Entity->GetEntityType())
			{
			case EntityType::Wing:          Solid = Entity->GetModelAs<WingModel>()->BuildSolid(kExportNSpan, true, Tol); break;
			case EntityType::Fuselage:      Solid = Entity->GetModelAs<FuselageModel>()->BuildSolid(30, 40, Tol); break;
			case EntityType::LoftedNacelle: Solid = Entity->GetModelAs<LoftedNacelleModel>()->BuildSolid(30, 80, Tol); break;
			case EntityType::Propeller:     Solid = Entity->GetModelAs<PropellerModel>()->BuildSolid(kExportNSpan, Tol, false); break;
			default:                        Solid = Entity->GetModel()->BuildSolid(); break;
			}
			const EntityTransform T = Entity->GetEntityTransform();
			TopoDS_Shape Placed = Place(Solid, { T.tx, T.ty, T.tz }, { T.ax, T.ay, T.az }, T.angle_deg);
			if (Operand.bMirror)
			{
				Placed = ApplyMirror(Placed, { 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });
			}
			return Placed;
		}
		catch (const std::exception& e)
		{
			OutError = UTF8_TO_TCHAR(e.what());
			return TopoDS_Shape();
		}
	}

	FString FindStepWriter()
	{
		const FString Staged = FPaths::Combine(FPaths::ProjectDir(), TEXT("Binaries"), TEXT("Win64"), TEXT("StepWriter"), TEXT("aerotelic_step_writer.exe"));
		if (FPaths::FileExists(Staged))
		{
			return FPaths::ConvertRelativePathToFull(Staged);
		}
		const FString Dev = FPaths::Combine(FPaths::ProjectDir(), TEXT(".."), TEXT("core"), TEXT("build"), TEXT("Release"), TEXT("aerotelic_step_writer.exe"));
		if (FPaths::FileExists(Dev))
		{
			return FPaths::ConvertRelativePathToFull(Dev);
		}
		return FString();
	}

	bool WriteStep(const TArray<FBody>& Bodies, const FString& Path, bool bNamed, FString& OutError)
	{
		const FString Writer = FindStepWriter();
		if (Writer.IsEmpty())
		{
			OutError = TEXT("aerotelic_step_writer.exe was not found (run ue\\tools\\stage_step_writer.ps1 after building the core).");
			return false;
		}
		// The bodies go through .brep files in a scratch folder.
		const FString Scratch = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_step_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Scratch, true);
		FString Params = FString::Printf(TEXT("\"%s\" %s"), *FPaths::ConvertRelativePathToFull(Path), bNamed ? TEXT("--named") : TEXT("--plain"));
		for (int32 i = 0; i < Bodies.Num(); ++i)
		{
			const FString Brep = FPaths::Combine(Scratch, FString::Printf(TEXT("body_%d.brep"), i));
			if (!BRepTools::Write(Bodies[i].Shape, TCHAR_TO_UTF8(*Brep)))
			{
				OutError = FString::Printf(TEXT("could not write %s"), *Brep);
				IFileManager::Get().DeleteDirectory(*Scratch, false, true);
				return false;
			}
			FString Label = Bodies[i].Label;
			Label.ReplaceInline(TEXT("\""), TEXT("'"));
			Params += FString::Printf(TEXT(" \"%s\" \"%s\""), *Label, *Brep);
		}

		// The writer's own DLLs sit beside it (the staged folder); in a
		// developer checkout running from the core build tree they come
		// from the OCCT install, which the writer's directory does not hold.
		void* ReadPipe = nullptr; void* WritePipe = nullptr;
		FPlatformProcess::CreatePipe(ReadPipe, WritePipe);
		const FString WorkingDir = FPaths::GetPath(Writer);
		FProcHandle Proc = FPlatformProcess::CreateProc(*Writer, *Params, false, true, true, nullptr, 0, *WorkingDir, WritePipe, nullptr, WritePipe);
		FString Output;
		bool bOk = false;
		if (Proc.IsValid())
		{
			while (FPlatformProcess::IsProcRunning(Proc))
			{
				Output += FPlatformProcess::ReadPipe(ReadPipe);
				FPlatformProcess::Sleep(0.05f);
			}
			Output += FPlatformProcess::ReadPipe(ReadPipe);
			int32 Code = -1;
			FPlatformProcess::GetProcReturnCode(Proc, &Code);
			FPlatformProcess::CloseProc(Proc);
			bOk = Code == 0;
			if (!bOk)
			{
				// The reason is the writer's last non-statistics line.
				TArray<FString> Lines;
				Output.ParseIntoArrayLines(Lines);
				FString Reason;
				for (int32 i = Lines.Num() - 1; i >= 0; --i)
				{
					const FString L = Lines[i].TrimStartAndEnd();
					if (!L.IsEmpty() && !L.StartsWith(TEXT("*")) && !L.Contains(TEXT("Statistics")) && !L.Contains(TEXT("Transfer")) && !L.Contains(TEXT("Step File Name")))
					{
						Reason = L;
						break;
					}
				}
				OutError = Reason.IsEmpty() ? FString::Printf(TEXT("the STEP writer exited with code %d"), Code) : Reason;
				UE_LOG(LogAerotelicExport, Warning, TEXT("STEP writer failed (%d):\n%s"), Code, *Output);
			}
		}
		else
		{
			OutError = FString::Printf(TEXT("could not start %s"), *Writer);
		}
		FPlatformProcess::ClosePipe(ReadPipe, WritePipe);
		IFileManager::Get().DeleteDirectory(*Scratch, false, true);
		return bOk;
	}
}
