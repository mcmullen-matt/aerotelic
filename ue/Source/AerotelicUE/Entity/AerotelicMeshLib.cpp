#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicMeshLib, Log, All);

namespace AerotelicMeshLib
{
	namespace
	{
		constexpr const char* kTriMagic = "AEROTRI1";

		// A JSON string literal (quotes, backslashes and control characters escaped).
		FString JsonString(const FString& S)
		{
			FString Out = TEXT("\"");
			for (TCHAR C : S)
			{
				switch (C)
				{
				case TEXT('"'): Out += TEXT("\\\""); break;
				case TEXT('\\'): Out += TEXT("\\\\"); break;
				case TEXT('\n'): Out += TEXT("\\n"); break;
				case TEXT('\r'): Out += TEXT("\\r"); break;
				case TEXT('\t'): Out += TEXT("\\t"); break;
				default:
					if (C < 0x20) Out += FString::Printf(TEXT("\\u%04x"), (int32)C);
					else Out.AppendChar(C);
				}
			}
			return Out + TEXT("\"");
		}

		FString Num(double V)
		{
			return FString::Printf(TEXT("%.17g"), V);
		}

		// A list of {label, file} entries, writing each body's .tri on the way.
		bool WriteBodies(const TArray<FBody>& Bodies, const FString& Dir, const TCHAR* Prefix, FString& OutJson, FString& OutError)
		{
			OutJson = TEXT("[");
			for (int32 i = 0; i < Bodies.Num(); ++i)
			{
				const FString File = FString::Printf(TEXT("%s_%d.tri"), Prefix, i);
				if (Bodies[i].Mesh == nullptr || !WriteTri(*Bodies[i].Mesh, FPaths::Combine(Dir, File)))
				{
					OutError = FString::Printf(TEXT("could not write the mesh of '%s' for MeshLib"), *Bodies[i].Label);
					return false;
				}
				OutJson += FString::Printf(TEXT("%s{\"label\": %s, \"file\": %s}"), i ? TEXT(", ") : TEXT(""), *JsonString(Bodies[i].Label), *JsonString(File));
			}
			OutJson += TEXT("]");
			return true;
		}
	}

	bool WriteTri(const aerotelic::TriMesh& Mesh, const FString& Path)
	{
		TArray<uint8> Bytes;
		const uint32 Nv = static_cast<uint32>(Mesh.vertices.size());
		const uint32 Nt = static_cast<uint32>(Mesh.triangles.size());
		Bytes.Reserve(16 + Nv * 24 + Nt * 12);
		Bytes.Append(reinterpret_cast<const uint8*>(kTriMagic), 8);
		Bytes.Append(reinterpret_cast<const uint8*>(&Nv), 4);
		Bytes.Append(reinterpret_cast<const uint8*>(&Nt), 4);
		for (const aerotelic::Vec3& V : Mesh.vertices)
		{
			const double Xyz[3] = {V.x, V.y, V.z};
			Bytes.Append(reinterpret_cast<const uint8*>(Xyz), 24);
		}
		for (const auto& T : Mesh.triangles)
		{
			const int32 Abc[3] = {T[0], T[1], T[2]};
			Bytes.Append(reinterpret_cast<const uint8*>(Abc), 12);
		}
		return FFileHelper::SaveArrayToFile(Bytes, *Path);
	}

	bool ReadTri(const FString& Path, aerotelic::TriMesh& OutMesh, FString& OutError)
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *Path))
		{
			OutError = FString::Printf(TEXT("could not read %s"), *Path);
			return false;
		}
		if (Bytes.Num() < 16 || FMemory::Memcmp(Bytes.GetData(), kTriMagic, 8) != 0)
		{
			OutError = FString::Printf(TEXT("%s is not a .tri file"), *Path);
			return false;
		}
		uint32 Nv = 0, Nt = 0;
		FMemory::Memcpy(&Nv, Bytes.GetData() + 8, 4);
		FMemory::Memcpy(&Nt, Bytes.GetData() + 12, 4);
		const uint64 Expected = 16ull + 24ull * Nv + 12ull * Nt;
		if (static_cast<uint64>(Bytes.Num()) != Expected)
		{
			OutError = FString::Printf(TEXT("%s is truncated (%d bytes, %llu expected)"), *Path, Bytes.Num(), Expected);
			return false;
		}
		OutMesh.Clear();
		OutMesh.vertices.resize(Nv);
		OutMesh.triangles.resize(Nt);
		const uint8* P = Bytes.GetData() + 16;
		for (uint32 i = 0; i < Nv; ++i, P += 24)
		{
			double Xyz[3];
			FMemory::Memcpy(Xyz, P, 24);
			OutMesh.vertices[i] = {Xyz[0], Xyz[1], Xyz[2]};
		}
		for (uint32 i = 0; i < Nt; ++i, P += 12)
		{
			int32 Abc[3];
			FMemory::Memcpy(Abc, P, 12);
			for (int32 k = 0; k < 3; ++k)
			{
				if (Abc[k] < 0 || static_cast<uint32>(Abc[k]) >= Nv)
				{
					OutError = FString::Printf(TEXT("%s: triangle %u refers to vertex %d of %u"), *Path, i, Abc[k], Nv);
					return false;
				}
			}
			OutMesh.triangles[i] = {Abc[0], Abc[1], Abc[2]};
		}
		return true;
	}

	FString FindPython()
	{
		TArray<FString> Candidates;
		const FString FromEnv = FPlatformMisc::GetEnvironmentVariable(TEXT("AEROTELIC_MESHLIB_PYTHON"));
		if (!FromEnv.IsEmpty())
		{
			Candidates.Add(FromEnv);
		}
		const FString MambaRoot = FPlatformMisc::GetEnvironmentVariable(TEXT("MAMBA_ROOT_PREFIX"));
		if (!MambaRoot.IsEmpty())
		{
			Candidates.Add(FPaths::Combine(MambaRoot, TEXT("envs"), TEXT("WingDesigner"), TEXT("python.exe")));
		}
		const FString Home = FPlatformMisc::GetEnvironmentVariable(TEXT("USERPROFILE"));
		if (!Home.IsEmpty())
		{
			Candidates.Add(FPaths::Combine(Home, TEXT("micromamba"), TEXT("envs"), TEXT("WingDesigner"), TEXT("python.exe")));
		}
		Candidates.Add(TEXT("C:\\Users\\Matt\\micromamba\\envs\\WingDesigner\\python.exe"));
		for (const FString& C : Candidates)
		{
			if (FPaths::FileExists(C))
			{
				return FPaths::ConvertRelativePathToFull(C);
			}
		}
		return FString();
	}

	FString FindScript()
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("lattice_meshlib.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError)
	{
		const FString Python = FindPython();
		if (Python.IsEmpty())
		{
			OutError = TEXT("MeshLib's Python interpreter was not found: install MeshLib (pip install meshlib) in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has it.");
			return false;
		}
		const FString Script = FindScript();
		if (Script.IsEmpty())
		{
			OutError = TEXT("tools\\lattice_meshlib.py was not found beside the application.");
			return false;
		}
		if (Request.Targets.Num() == 0)
		{
			OutError = TEXT("no body to hollow");
			return false;
		}

		// The job folder: the bodies, the field and job.json.
		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_lattice_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		auto Cleanup = [&Dir]() { IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		FString TargetsJson, ReservedJson;
		if (!WriteBodies(Request.Targets, Dir, TEXT("target"), TargetsJson, OutError) || !WriteBodies(Request.Reserved, Dir, TEXT("reserved"), ReservedJson, OutError))
		{
			Cleanup();
			return false;
		}
		FString FieldJson = TEXT("null");
		if (Request.Field != nullptr && Request.Field->Nx > 0)
		{
			const FField& F = *Request.Field;
			const size_t Count = static_cast<size_t>(F.Nx) * F.Ny * F.Nz;
			if (F.Values.size() != Count)
			{
				OutError = TEXT("the lattice field's sample count does not match its grid");
				Cleanup();
				return false;
			}
			const TArrayView64<const uint8> Raw(reinterpret_cast<const uint8*>(F.Values.data()), static_cast<int64>(Count * sizeof(float)));
			if (!FFileHelper::SaveArrayToFile(Raw, *FPaths::Combine(Dir, TEXT("lattice.f32"))))
			{
				OutError = TEXT("could not write the lattice field for MeshLib");
				Cleanup();
				return false;
			}
			FieldJson = FString::Printf(TEXT("{\"file\": \"lattice.f32\", \"nx\": %d, \"ny\": %d, \"nz\": %d, \"origin\": [%s, %s, %s], \"h\": %s}"),
				F.Nx, F.Ny, F.Nz, *Num(F.Origin.x), *Num(F.Origin.y), *Num(F.Origin.z), *Num(F.H));
		}
		const FString Job = FString::Printf(
			TEXT("{\n \"skin_thickness\": %s,\n \"offset_voxel\": %s,\n \"offset_mode\": %s,\n \"decimate_tolerance\": %s,\n \"min_pocket_volume\": %s,\n \"inner_opening\": %s,\n \"inner_remesh_edge\": %s,\n")
			TEXT(" \"targets\": %s,\n \"reserved\": %s,\n \"field\": %s,\n")
			TEXT(" \"outputs\": {\"skin\": \"skin.tri\", \"lattice\": \"lattice.tri\", \"result\": \"result.json\", \"inner_prefix\": \"inner\"}\n}\n"),
			*Num(Request.SkinThickness), *Num(Request.OffsetVoxel), *JsonString(Request.OffsetMode), *Num(Request.DecimateTolerance), *Num(Request.MinPocketVolume), *Num(Request.InnerOpening), *Num(Request.InnerRemeshEdge),
			*TargetsJson, *ReservedJson, *FieldJson);
		if (!FFileHelper::SaveStringToFile(Job, *FPaths::Combine(Dir, TEXT("job.json")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			OutError = TEXT("could not write the MeshLib job file");
			Cleanup();
			return false;
		}

		// Run it, collecting its output for the log.
		const FString Params = FString::Printf(TEXT("\"%s\" \"%s\""), *Script, *Dir);
		void* ReadPipe = nullptr; void* WritePipe = nullptr;
		FPlatformProcess::CreatePipe(ReadPipe, WritePipe);
		const FString WorkingDir = FPaths::GetPath(Script);
		const double T0 = FPlatformTime::Seconds();
		FProcHandle Proc = FPlatformProcess::CreateProc(*Python, *Params, false, true, true, nullptr, 0, *WorkingDir, WritePipe, nullptr, WritePipe);
		// CloseProc resets the handle, so remember whether it started.
		const bool bStarted = Proc.IsValid();
		FString Output;
		int32 Code = -1;
		if (bStarted)
		{
			while (FPlatformProcess::IsProcRunning(Proc))
			{
				Output += FPlatformProcess::ReadPipe(ReadPipe);
				FPlatformProcess::Sleep(0.02f);
			}
			Output += FPlatformProcess::ReadPipe(ReadPipe);
			FPlatformProcess::GetProcReturnCode(Proc, &Code);
			FPlatformProcess::CloseProc(Proc);
		}
		FPlatformProcess::ClosePipe(ReadPipe, WritePipe);
		if (!bStarted)
		{
			OutError = FString::Printf(TEXT("could not start %s"), *Python);
			Cleanup();
			return false;
		}
		UE_LOG(LogAerotelicMeshLib, Log, TEXT("lattice_meshlib.py exited with %d after %.2f s:\n%s"), Code, FPlatformTime::Seconds() - T0, *Output);

		// The result: statistics, or the reason it failed.
		FString ResultText;
		TSharedPtr<FJsonObject> Result;
		if (FFileHelper::LoadFileToString(ResultText, *FPaths::Combine(Dir, TEXT("result.json"))))
		{
			TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(ResultText);
			FJsonSerializer::Deserialize(Reader, Result);
		}
		bool bOk = false;
		if (!Result.IsValid() || !Result->TryGetBoolField(TEXT("ok"), bOk) || !bOk)
		{
			FString Reason;
			if (Result.IsValid())
			{
				Result->TryGetStringField(TEXT("error"), Reason);
			}
			if (Reason.IsEmpty())
			{
				// The script's last "error:" line, else its exit code.
				TArray<FString> Lines;
				Output.ParseIntoArrayLines(Lines);
				for (int32 i = Lines.Num() - 1; i >= 0; --i)
				{
					if (Lines[i].StartsWith(TEXT("error:")) || Lines[i].Contains(TEXT("Error")))
					{
						Reason = Lines[i].TrimStartAndEnd();
						break;
					}
				}
			}
			OutError = Reason.IsEmpty() ? FString::Printf(TEXT("MeshLib's lattice backend exited with code %d (see the log)."), Code) : Reason;
			Cleanup();
			return false;
		}

		FString ReadError;
		if (!ReadTri(FPaths::Combine(Dir, TEXT("skin.tri")), Out.Skin, ReadError) || !ReadTri(FPaths::Combine(Dir, TEXT("lattice.tri")), Out.Lattice, ReadError))
		{
			OutError = ReadError;
			Cleanup();
			return false;
		}
		Out.SkinVolume = Result->GetNumberField(TEXT("skin_volume"));
		Out.DomainVolume = Result->GetNumberField(TEXT("domain_volume"));
		Out.LatticeVolume = Result->GetNumberField(TEXT("lattice_volume"));
		Out.AchievedFraction = Result->GetNumberField(TEXT("achieved_fraction"));
		Out.Interiors = static_cast<int32>(Result->GetNumberField(TEXT("interiors")));
		Out.Seconds = Result->GetNumberField(TEXT("seconds"));
		Out.Warnings.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Warnings = nullptr;
		if (Result->TryGetArrayField(TEXT("warnings"), Warnings))
		{
			for (const TSharedPtr<FJsonValue>& W : *Warnings)
			{
				FString S;
				if (W.IsValid() && W->TryGetString(S)) Out.Warnings.Add(S);
			}
		}
		// The inner surfaces, one per target in order.
		Out.Inners.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Targets = nullptr;
		if (Result->TryGetArrayField(TEXT("targets"), Targets))
		{
			for (const TSharedPtr<FJsonValue>& V : *Targets)
			{
				const TSharedPtr<FJsonObject>* Obj = nullptr;
				if (!V.IsValid() || !V->TryGetObject(Obj) || !Obj->IsValid())
				{
					continue;
				}
				FInner Inner;
				(*Obj)->TryGetStringField(TEXT("label"), Inner.Label);
				(*Obj)->TryGetBoolField(TEXT("inner_ok"), Inner.bUsable);
				(*Obj)->TryGetStringField(TEXT("inner_note"), Inner.Note);
				(*Obj)->TryGetNumberField(TEXT("inner_volume"), Inner.Volume);
				double Components = 0.0;
				(*Obj)->TryGetNumberField(TEXT("inner_components"), Components);
				Inner.Components = static_cast<int32>(Components);
				FString File;
				if ((*Obj)->TryGetStringField(TEXT("inner_file"), File) && !ReadTri(FPaths::Combine(Dir, File), Inner.Mesh, ReadError))
				{
					Inner.bUsable = false;
					Inner.Note = ReadError;
				}
				Out.Inners.Add(MoveTemp(Inner));
			}
		}
		if (Out.Inners.Num() != Request.Targets.Num())
		{
			OutError = FString::Printf(TEXT("MeshLib returned %d inner surfaces for %d bodies"), Out.Inners.Num(), Request.Targets.Num());
			Cleanup();
			return false;
		}
		Cleanup();
		return true;
	}
}
