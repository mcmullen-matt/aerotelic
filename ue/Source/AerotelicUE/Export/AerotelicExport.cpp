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
#include "Entity/AerotelicMeshing.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicExport, Log, All);

namespace AerotelicExport
{
	namespace
	{
		// main_window.py _EXPORT_N_SPAN.
		constexpr int kExportNSpan = 200;

		using FTriangle = FStlTriangle;

		void AppendMesh(const aerotelic::TriMesh& Mesh, FStlMesh& Out)
		{
			Out.Nodes += static_cast<int64>(Mesh.vertices.size());
			Out.Triangles.Reserve(Out.Triangles.Num() + static_cast<int32>(Mesh.triangles.size()));
			for (const auto& Tri : Mesh.triangles)
			{
				FTriangle T;
				const aerotelic::Vec3& A = Mesh.vertices[Tri[0]];
				const aerotelic::Vec3& B = Mesh.vertices[Tri[1]];
				const aerotelic::Vec3& C = Mesh.vertices[Tri[2]];
				T.A = FVector3f((float)A.x, (float)A.y, (float)A.z);
				T.B = FVector3f((float)B.x, (float)B.y, (float)B.z);
				T.C = FVector3f((float)C.x, (float)C.y, (float)C.z);
				T.N = FVector3f::CrossProduct(T.B - T.A, T.C - T.A).GetSafeNormal();
				Out.Triangles.Add(T);
			}
		}
	}

	FStlMesh MeshForStl(const TArray<FAerotelicBooleanOperand>& Operands, const TArray<FString>& Labels,
		double LinearDeflection, double AngularDeflectionDeg, TArray<FString>& OutFailures)
	{
		FStlMesh Out;
		OutFailures.Reset();
		for (int32 i = 0; i < Operands.Num(); ++i)
		{
			const FString Label = Labels.IsValidIndex(i) ? Labels[i] : FString::Printf(TEXT("Body %d"), i + 1);
			if (!Operands[i].Entity.IsValid())
			{
				OutFailures.Add(Label + TEXT(": no longer in the scene"));
				continue;
			}
			std::vector<aerotelic::NamedMesh> Meshes;
			try
			{
				// The model's own meshes at the linear deflection (the OCCT
				// path inside uses the angular deflection too).
				Meshes = Operands[i].Entity->GetPlacedMeshes(Operands[i].bMirror, LinearDeflection);
				if (Meshes.empty())
				{
					Meshes = AerotelicMeshing::MeshesFromShape(Operands[i].Entity->GetPlacedShape(Operands[i].bMirror), LinearDeflection, AngularDeflectionDeg);
				}
			}
			catch (const std::exception& e)
			{
				OutFailures.Add(FString::Printf(TEXT("%s: meshing failed: %s"), *Label, UTF8_TO_TCHAR(e.what())));
				continue;
			}
			bool bAny = false;
			for (const aerotelic::NamedMesh& M : Meshes)
			{
				if (!M.mesh.IsEmpty())
				{
					FStlPart Part;
					Part.Name = FString::Printf(TEXT("%s/%s"), *Label, UTF8_TO_TCHAR(M.name.c_str()));
					Part.First = Out.Triangles.Num();
					AppendMesh(M.mesh, Out);
					Part.Count = Out.Triangles.Num() - Part.First;
					Out.Parts.Add(Part);
					bAny = true;
				}
			}
			if (!bAny)
			{
				OutFailures.Add(Label + TEXT(": produced no triangles"));
				continue;
			}
			++Out.Bodies;
		}
		return Out;
	}

	bool WriteStl(const FStlMesh& Mesh, const FString& Path, bool bAscii, FString& OutError)
	{
		const TArray<FTriangle>& Tris = Mesh.Triangles;
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
			// One `solid` per part (see FStlPart), or one for the lot when
			// the mesh carries no parts. Names keep to one token: readers
			// that take the solid name commonly stop at the first space.
			TArray<FStlPart> Parts = Mesh.Parts;
			if (Parts.Num() == 0)
			{
				FStlPart All;
				All.Name = TEXT("aerotelic");
				All.Count = Tris.Num();
				Parts.Add(All);
			}
			for (const FStlPart& Part : Parts)
			{
				FString SolidName = Part.Name.Replace(TEXT(" "), TEXT("_"));
				Text += FString::Printf(TEXT("solid %s\n"), *SolidName);
				for (int32 i = Part.First; i < Part.First + Part.Count && i < Tris.Num(); ++i)
				{
					const FTriangle& T = Tris[i];
					Text += FString::Printf(TEXT("  facet normal %e %e %e\n    outer loop\n      vertex %e %e %e\n      vertex %e %e %e\n      vertex %e %e %e\n    endloop\n  endfacet\n"),
						T.N.X, T.N.Y, T.N.Z, T.A.X, T.A.Y, T.A.Z, T.B.X, T.B.Y, T.B.Z, T.C.X, T.C.Y, T.C.Z);
				}
				Text += FString::Printf(TEXT("endsolid %s\n"), *SolidName);
			}
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
		if (Entity->GetEntityType() == EntityType::Lattice)
		{
			OutError = TEXT("a lattice is a triangle mesh with no B-rep: export it as STL");
			return TopoDS_Shape();
		}
		if (Entity->GetEntityType() == EntityType::Boolean)
		{
			OutError = TEXT("a boolean result is a triangle mesh with no B-rep: export its operands and combine them downstream, or export it as STL");
			return TopoDS_Shape();
		}
		try
		{
			// As it stands, or rebuilt at the export tolerance.
			if (!StationTol.IsSet())
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
