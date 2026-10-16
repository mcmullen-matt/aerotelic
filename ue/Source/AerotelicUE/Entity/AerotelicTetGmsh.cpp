#include "AerotelicTetGmsh.h"
#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicTetGmsh, Log, All);

namespace AerotelicTetGmsh
{
	namespace
	{
		constexpr const char* kTetMagic = "AEROTET1";

		FString Num(double V)
		{
			return FString::Printf(TEXT("%.17g"), V);
		}

		// The .tet file tools/tet_gmsh.py writes: "AEROTET1", uint32 node
		// count, uint32 tetrahedron count, float64 xyz per node, int32 abcd
		// per tetrahedron, int32 material per tetrahedron.
		bool ReadTet(const FString& Path, aerotelic::TetMesh& Out, FString& OutError)
		{
			TArray<uint8> Bytes;
			if (!FFileHelper::LoadFileToArray(Bytes, *Path))
			{
				OutError = FString::Printf(TEXT("could not read %s"), *Path);
				return false;
			}
			if (Bytes.Num() < 16 || FMemory::Memcmp(Bytes.GetData(), kTetMagic, 8) != 0)
			{
				OutError = FString::Printf(TEXT("%s is not a .tet file"), *Path);
				return false;
			}
			uint32 Nn = 0, Nt = 0;
			FMemory::Memcpy(&Nn, Bytes.GetData() + 8, 4);
			FMemory::Memcpy(&Nt, Bytes.GetData() + 12, 4);
			const uint64 Expected = 16ull + 24ull * Nn + 16ull * Nt + 4ull * Nt;
			if (static_cast<uint64>(Bytes.Num()) != Expected)
			{
				OutError = FString::Printf(TEXT("%s is truncated (%d bytes, %llu expected)"), *Path, Bytes.Num(), Expected);
				return false;
			}
			Out = aerotelic::TetMesh();
			Out.nodes.resize(Nn);
			Out.tets.resize(Nt);
			Out.material.resize(Nt);
			const uint8* P = Bytes.GetData() + 16;
			for (uint32 i = 0; i < Nn; ++i, P += 24)
			{
				double Xyz[3];
				FMemory::Memcpy(Xyz, P, 24);
				Out.nodes[i] = {Xyz[0], Xyz[1], Xyz[2]};
			}
			for (uint32 i = 0; i < Nt; ++i, P += 16)
			{
				int32 Abcd[4];
				FMemory::Memcpy(Abcd, P, 16);
				for (int32 k = 0; k < 4; ++k)
				{
					if (Abcd[k] < 0 || static_cast<uint32>(Abcd[k]) >= Nn)
					{
						OutError = FString::Printf(TEXT("%s: tetrahedron %u refers to node %d of %u"), *Path, i, Abcd[k], Nn);
						return false;
					}
				}
				Out.tets[i] = {Abcd[0], Abcd[1], Abcd[2], Abcd[3]};
			}
			for (uint32 i = 0; i < Nt; ++i, P += 4)
			{
				int32 M = 0;
				FMemory::Memcpy(&M, P, 4);
				Out.material[i] = M;
			}
			// No per-face constraint flags: with gmsh the boundary IS the
			// input surfaces, so nothing has to be marked to find it.
			Out.constrained.assign(Nt, 0);
			return true;
		}
	}

	FString FindScript()
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("tet_gmsh.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError)
	{
		if (Request.Outer == nullptr || Request.Outer->IsEmpty())
		{
			OutError = TEXT("no body surface to mesh");
			return false;
		}
		// The interpreter is the lattice backend's: the same environment
		// holds MeshLib and gmsh.
		const FString Python = AerotelicMeshLib::FindPython();
		if (Python.IsEmpty())
		{
			OutError = TEXT("The Python interpreter was not found: install gmsh (pip install gmsh) in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has it.");
			return false;
		}
		const FString Script = FindScript();
		if (Script.IsEmpty())
		{
			OutError = TEXT("tools\\tet_gmsh.py was not found beside the application.");
			return false;
		}

		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_tet_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		auto Cleanup = [&Dir]() { IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		if (!AerotelicMeshLib::WriteTri(*Request.Outer, FPaths::Combine(Dir, TEXT("outer.tri"))))
		{
			OutError = TEXT("could not write the body's surface for gmsh");
			Cleanup();
			return false;
		}
		const bool bHasInner = Request.Inner != nullptr && !Request.Inner->IsEmpty();
		if (bHasInner && !AerotelicMeshLib::WriteTri(*Request.Inner, FPaths::Combine(Dir, TEXT("inner.tri"))))
		{
			OutError = TEXT("could not write the offset surface for gmsh");
			Cleanup();
			return false;
		}

		const FString Job = FString::Printf(
			TEXT("{\n \"outer\": \"outer.tri\",\n \"inner\": \"%s\",\n \"core_size\": %s,\n \"min_size\": %s,\n \"min_pocket_volume\": %s,\n \"algorithm\": %d,\n \"optimize\": %s,\n \"optimize_netgen\": %s,\n")
			TEXT(" \"outputs\": {\"mesh\": \"mesh.tet\", \"result\": \"result.json\"}\n}\n"),
			bHasInner ? TEXT("inner.tri") : TEXT(""), *Num(Request.CoreSize), *Num(Request.MinSize), *Num(Request.MinPocketVolume),
			Request.Algorithm, Request.bOptimize ? TEXT("true") : TEXT("false"), Request.bOptimizeNetgen ? TEXT("true") : TEXT("false"));
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
		if (!ReadTet(FPaths::Combine(Dir, TEXT("mesh.tet")), Out.Mesh, ReadError))
		{
			OutError = ReadError;
			Cleanup();
			return false;
		}
		auto Int = [&Result](const TCHAR* Field) { return static_cast<int32>(Result->GetNumberField(Field)); };
		Out.Nodes = Int(TEXT("nodes"));
		Out.SkinTets = Int(TEXT("skin_tets"));
		Out.CoreTets = Int(TEXT("core_tets"));
		Out.OuterVolume = Result->GetNumberField(TEXT("outer_volume"));
		Out.InnerVolume = Result->GetNumberField(TEXT("inner_volume"));
		Out.SkinVolume = Result->GetNumberField(TEXT("skin_volume"));
		Out.CoreVolume = Result->GetNumberField(TEXT("core_volume"));
		Out.VolumeError = Result->GetNumberField(TEXT("volume_error"));
		Out.CoreVolumeError = Result->GetNumberField(TEXT("core_volume_error"));
		Out.MinQuality = Result->GetNumberField(TEXT("min_quality"));
		Out.MeanQuality = Result->GetNumberField(TEXT("mean_quality"));
		Out.PoorQuality = Int(TEXT("poor_quality"));
		Out.Flipped = Int(TEXT("flipped"));
		Out.Degenerate = Int(TEXT("degenerate"));
		Out.Seconds = Result->GetNumberField(TEXT("seconds"));
		Cleanup();
		return true;
	}
}
