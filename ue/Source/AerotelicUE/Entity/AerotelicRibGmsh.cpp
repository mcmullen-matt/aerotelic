#include "AerotelicRibGmsh.h"
#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicRibGmsh, Log, All);

namespace AerotelicRibGmsh
{
	namespace
	{
		constexpr const char* kNodesMagic = "AERONOD1";

		FString Num(double V)
		{
			return FString::Printf(TEXT("%.17g"), V);
		}

		// nodes.bin as tools/rib_gmsh.py writes it: "AERONOD1" (8 bytes),
		// uint32 node count, float64 xyz per node. The surface's vertices
		// come first, in order, then the ribs' interior nodes.
		bool ReadNodes(const FString& Path, std::vector<aerotelic::Vec3>& Out, FString& OutError)
		{
			TArray<uint8> Bytes;
			if (!FFileHelper::LoadFileToArray(Bytes, *Path))
			{
				OutError = FString::Printf(TEXT("could not read %s"), *Path);
				return false;
			}
			if (Bytes.Num() < 12 || FMemory::Memcmp(Bytes.GetData(), kNodesMagic, 8) != 0)
			{
				OutError = FString::Printf(TEXT("%s is not a rib nodes file"), *Path);
				return false;
			}
			uint32 Nn = 0;
			FMemory::Memcpy(&Nn, Bytes.GetData() + 8, 4);
			const uint64 Expected = 12ull + 24ull * Nn;
			if (static_cast<uint64>(Bytes.Num()) != Expected)
			{
				OutError = FString::Printf(TEXT("%s is truncated (%d bytes, %llu expected)"), *Path, Bytes.Num(), Expected);
				return false;
			}
			Out.clear();
			Out.resize(Nn);
			const uint8* P = Bytes.GetData() + 12;
			for (uint32 i = 0; i < Nn; ++i, P += 24)
			{
				double Xyz[3];
				FMemory::Memcpy(Xyz, P, 24);
				Out[i] = {Xyz[0], Xyz[1], Xyz[2]};
			}
			return true;
		}

		// ribs.bin: uint32 count, int32 node triples, int32 rib id per face.
		bool ReadRibs(const FString& Path, int32 Nodes, TArray<FRibFace>& Out, FString& OutError)
		{
			Out.Reset();
			TArray<uint8> Bytes;
			if (!FFileHelper::LoadFileToArray(Bytes, *Path))
			{
				OutError = FString::Printf(TEXT("could not read %s"), *Path);
				return false;
			}
			if (Bytes.Num() < 4)
			{
				OutError = FString::Printf(TEXT("%s is truncated"), *Path);
				return false;
			}
			uint32 Nf = 0;
			FMemory::Memcpy(&Nf, Bytes.GetData(), 4);
			if (static_cast<uint64>(Bytes.Num()) != 4ull + 16ull * Nf)
			{
				OutError = FString::Printf(TEXT("%s is truncated (%d bytes for %u faces)"), *Path, Bytes.Num(), Nf);
				return false;
			}
			const uint8* P = Bytes.GetData() + 4;
			Out.SetNum(Nf);
			for (uint32 i = 0; i < Nf; ++i, P += 12)
			{
				FMemory::Memcpy(Out[i].N, P, 12);
				for (int32 k = 0; k < 3; ++k)
				{
					if (Out[i].N[k] < 0 || Out[i].N[k] >= Nodes)
					{
						OutError = FString::Printf(TEXT("%s: rib face %u refers to node %d of %d"), *Path, i, Out[i].N[k], Nodes);
						return false;
					}
				}
			}
			for (uint32 i = 0; i < Nf; ++i, P += 4)
			{
				FMemory::Memcpy(&Out[i].Id, P, 4);
			}
			return true;
		}
	}

	FString FindScript()
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("rib_gmsh.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError)
	{
		if (Request.Outer == nullptr || Request.Outer->IsEmpty())
		{
			OutError = TEXT("no body surface to mesh the ribs on");
			return false;
		}
		if (Request.Ribs.Num() == 0)
		{
			OutError = TEXT("no rib plane to mesh");
			return false;
		}
		// The interpreter is the one every backend shares (AerotelicMeshLib.h
		// FindPython): the WingDesigner environment holds gmsh and Kratos.
		const FString Python = AerotelicMeshLib::FindPython();
		if (Python.IsEmpty())
		{
			OutError = TEXT("The Python interpreter was not found: install gmsh (pip install gmsh) in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has it.");
			return false;
		}
		const FString Script = FindScript();
		if (Script.IsEmpty())
		{
			OutError = TEXT("tools\\rib_gmsh.py was not found beside the application.");
			return false;
		}

		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_rib_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		auto Cleanup = [&Dir]() { IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		if (!AerotelicMeshLib::WriteTri(*Request.Outer, FPaths::Combine(Dir, TEXT("outer.tri"))))
		{
			OutError = TEXT("could not write the body's surface for gmsh");
			Cleanup();
			return false;
		}

		FString RibsJson = TEXT("[");
		for (int32 i = 0; i < Request.Ribs.Num(); ++i)
		{
			const FRequest::FRib& R = Request.Ribs[i];
			if (R.Rim.Num() > 0)
			{
				TArray<FString> Rim, Chords;
				for (int32 V : R.Rim) Rim.Add(FString::FromInt(V));
				for (const FIntVector& C : R.Chords) Chords.Add(FString::Printf(TEXT("[%d, %d, %d]"), C.X, C.Y, C.Z));
				RibsJson += FString::Printf(TEXT("%s{\"id\": %d, \"rim\": [%s], \"chords\": [%s], \"height\": %s}"), i ? TEXT(", ") : TEXT(""),
					R.Id, *FString::Join(Rim, TEXT(", ")), *FString::Join(Chords, TEXT(", ")), *Num(FMath::Max(R.Height, 0.0)));
				continue;
			}
			RibsJson += FString::Printf(TEXT("%s{\"id\": %d, \"normal\": [%s, %s, %s], \"offset\": %s, \"height\": %s}"), i ? TEXT(", ") : TEXT(""),
				R.Id, *Num(R.Normal.x), *Num(R.Normal.y), *Num(R.Normal.z), *Num(R.Offset), *Num(FMath::Max(R.Height, 0.0)));
		}
		RibsJson += TEXT("]");
		// The reserved volumes as hole_<i>.tri, listed under "holes"; the key
		// is left out altogether when there are none (the script's default).
		FString HolesJson;
		{
			int32 Written = 0;
			for (const aerotelic::TriMesh* Hole : Request.Holes)
			{
				if (Hole == nullptr || Hole->IsEmpty()) continue;
				const FString File = FString::Printf(TEXT("hole_%d.tri"), Written);
				if (!AerotelicMeshLib::WriteTri(*Hole, FPaths::Combine(Dir, File)))
				{
					OutError = FString::Printf(TEXT("could not write reserved volume %d for gmsh"), Written);
					Cleanup();
					return false;
				}
				HolesJson += FString::Printf(TEXT("%s\"%s\""), Written ? TEXT(", ") : TEXT(""), *File);
				++Written;
			}
			if (Written > 0) HolesJson = FString::Printf(TEXT(" \"holes\": [%s],\n"), *HolesJson);
		}
		const FString Job = FString::Printf(
			TEXT("{\n \"outer\": \"outer.tri\",\n%s \"ribs\": %s,\n \"rib_size\": %s,\n \"plane_tolerance\": %s,\n")
			TEXT(" \"outputs\": {\"nodes\": \"nodes.bin\", \"result\": \"result.json\", \"ribs\": \"ribs.bin\"}\n}\n"),
			*HolesJson, *RibsJson, *Num(FMath::Max(Request.RibSize, 0.0)), *Num(FMath::Max(Request.PlaneTolerance, 0.0)));
		if (!FFileHelper::SaveStringToFile(Job, *FPaths::Combine(Dir, TEXT("job.json")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			OutError = TEXT("could not write the gmsh job file");
			Cleanup();
			return false;
		}

		TSharedPtr<FJsonObject> Result;
		if (!AerotelicMeshLib::RunJob(Script, Dir, Result, OutError))
		{
			Cleanup();
			return false;
		}

		FString ReadError;
		if (!ReadNodes(FPaths::Combine(Dir, TEXT("nodes.bin")), Out.Nodes, ReadError))
		{
			OutError = ReadError;
			Cleanup();
			return false;
		}
		Out.RibFaces.Reset();
		Out.Notes.Reset();
		if (!ReadRibs(FPaths::Combine(Dir, TEXT("ribs.bin")), static_cast<int32>(Out.Nodes.size()), Out.RibFaces, ReadError))
		{
			OutError = ReadError;
			Cleanup();
			return false;
		}
		auto Int = [&Result](const TCHAR* Field) { double V = 0.0; Result->TryGetNumberField(Field, V); return static_cast<int32>(V); };
		auto Real = [&Result](const TCHAR* Field) { double V = 0.0; Result->TryGetNumberField(Field, V); return V; };
		// surface_nodes: absent from a script that does not report it, in
		// which case the surface's own vertex count is the only answer.
		Out.SurfaceNodes = Result->HasField(TEXT("surface_nodes")) ? Int(TEXT("surface_nodes")) : static_cast<int32>(Request.Outer->vertices.size());
		Out.OuterVolume = Real(TEXT("outer_volume"));
		Out.RibArea = Real(TEXT("rib_area"));
		// holes: how many reserved-volume loops became holes of the discs;
		// absent from a script that was sent none.
		Out.Holes = Int(TEXT("holes"));
		{
			const TArray<TSharedPtr<FJsonValue>>* Notes = nullptr;
			if (Result->TryGetArrayField(TEXT("rib_notes"), Notes))
			{
				for (const TSharedPtr<FJsonValue>& N : *Notes)
				{
					FString S;
					if (N.IsValid() && N->TryGetString(S)) Out.Notes.Add(S);
				}
			}
		}
		Out.Seconds = Real(TEXT("seconds"));
		const int32 ReportedNodes = Int(TEXT("nodes"));
		if (ReportedNodes > 0 && ReportedNodes != static_cast<int32>(Out.Nodes.size()))
		{
			UE_LOG(LogAerotelicRibGmsh, Warning, TEXT("rib_gmsh reported %d nodes, nodes.bin holds %d"), ReportedNodes, static_cast<int32>(Out.Nodes.size()));
		}
		Cleanup();
		return true;
	}
}
