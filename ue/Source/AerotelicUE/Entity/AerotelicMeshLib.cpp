#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicMeshLib, Log, All);

namespace AerotelicMeshLib
{
	namespace
	{
		constexpr const char* kTriMagic = "AEROTRI1";
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

	bool RunJob(const FString& Script, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress)
	{
		const FString Python = FindPython();
		if (Python.IsEmpty())
		{
			OutError = TEXT("The backend's Python interpreter was not found: point AEROTELIC_MESHLIB_PYTHON at a python.exe that has the libraries, or install them in the WingDesigner micromamba environment.");
			return false;
		}
		const FString Name = FPaths::GetCleanFilename(Script);
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
		bool bCancelled = false;
		if (bStarted)
		{
			// The script's output as it comes, a complete line at a time to
			// whoever is watching (the structural dialog's status bar).
			FString Pending;
			auto Take = [&](bool bFlush)
			{
				const FString Chunk = FPlatformProcess::ReadPipe(ReadPipe);
				if (Chunk.IsEmpty() && !bFlush) return;
				Output += Chunk;
				if (Progress == nullptr || !Progress->OnLine) return;
				Pending += Chunk;
				int32 Nl = INDEX_NONE;
				while (Pending.FindChar(TEXT('\n'), Nl))
				{
					const FString Line = Pending.Left(Nl).TrimStartAndEnd();
					Pending.RightChopInline(Nl + 1);
					if (!Line.IsEmpty()) Progress->OnLine(Line);
				}
				if (bFlush && !Pending.TrimStartAndEnd().IsEmpty())
				{
					Progress->OnLine(Pending.TrimStartAndEnd());
					Pending.Empty();
				}
			};
			while (FPlatformProcess::IsProcRunning(Proc))
			{
				if (Progress != nullptr && Progress->bCancel.load())
				{
					FPlatformProcess::TerminateProc(Proc, true);
					bCancelled = true;
					break;
				}
				Take(false);
				FPlatformProcess::Sleep(0.02f);
			}
			Take(true);
			if (!bCancelled) FPlatformProcess::GetProcReturnCode(Proc, &Code);
			FPlatformProcess::CloseProc(Proc);
		}
		FPlatformProcess::ClosePipe(ReadPipe, WritePipe);
		if (!bStarted)
		{
			OutError = FString::Printf(TEXT("could not start %s"), *Python);
			return false;
		}
		if (bCancelled)
		{
			UE_LOG(LogAerotelicMeshLib, Log, TEXT("%s cancelled after %.2f s:\n%s"), *Name, FPlatformTime::Seconds() - T0, *Output);
			OutError = TEXT("cancelled");
			return false;
		}
		UE_LOG(LogAerotelicMeshLib, Log, TEXT("%s exited with %d after %.2f s:\n%s"), *Name, Code, FPlatformTime::Seconds() - T0, *Output);

		// The result: statistics, or the reason it failed.
		FString ResultText;
		OutResult.Reset();
		if (FFileHelper::LoadFileToString(ResultText, *FPaths::Combine(Dir, TEXT("result.json"))))
		{
			TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(ResultText);
			FJsonSerializer::Deserialize(Reader, OutResult);
		}
		bool bOk = false;
		if (!OutResult.IsValid() || !OutResult->TryGetBoolField(TEXT("ok"), bOk) || !bOk)
		{
			FString Reason;
			if (OutResult.IsValid())
			{
				OutResult->TryGetStringField(TEXT("error"), Reason);
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
			OutError = Reason.IsEmpty() ? FString::Printf(TEXT("%s exited with code %d (see the log)."), *Name, Code) : Reason;
			return false;
		}
		return true;
	}
}
