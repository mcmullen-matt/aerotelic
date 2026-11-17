#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformMisc.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
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

	FString FInterpreter::Describe() const
	{
		return bWsl ? FString::Printf(TEXT("wsl:%s:%s"), *Distro, *Python) : Python;
	}

	FString ToWslPath(const FString& WindowsPath)
	{
		FString Full = FPaths::ConvertRelativePathToFull(WindowsPath);
		Full.ReplaceInline(TEXT("\\"), TEXT("/"));
		if (Full.Len() >= 2 && FChar::IsAlpha(Full[0]) && Full[1] == TEXT(':'))
		{
			FString Rest = Full.Mid(2);
			if (!Rest.StartsWith(TEXT("/"))) Rest = TEXT("/") + Rest;
			return FString::Printf(TEXT("/mnt/%c%s"), FChar::ToLower(Full[0]), *Rest);
		}
		return Full;
	}

	namespace
	{
		FString WslExe()
		{
			const FString Root = FPlatformMisc::GetEnvironmentVariable(TEXT("SystemRoot"));
			const FString Exe = FPaths::Combine(Root.IsEmpty() ? FString(TEXT("C:\\Windows")) : Root, TEXT("System32"), TEXT("wsl.exe"));
			return FPaths::FileExists(Exe) ? Exe : FString();
		}

		// Asks the default WSL distribution for a tacs environment's python.
		// Empty when WSL is not installed, has no distribution, or none of
		// the usual environment roots holds one.
		FString ProbeWslTacsPython()
		{
			const FString Wsl = WslExe();
			if (Wsl.IsEmpty()) return FString();
			// sh expands $HOME inside the distribution; no double quotes
			// inside, the command line is one quoted argument.
			const FString Params = TEXT("-e sh -c \"for p in $HOME/micromamba/envs/tacs/bin/python $HOME/miniforge3/envs/tacs/bin/python $HOME/miniconda3/envs/tacs/bin/python $HOME/mambaforge/envs/tacs/bin/python /opt/tacs/bin/python; do if [ -x $p ]; then echo $p; exit 0; fi; done; exit 3\"");
			int32 Code = -1;
			FString Out, Err;
			const double T0 = FPlatformTime::Seconds();
			const bool bRan = FPlatformProcess::ExecProcess(*Wsl, *Params, &Code, &Out, &Err);
			// wsl.exe's own messages (no distribution, not installed) come out
			// UTF-16 and do not start with a slash; a path from sh is plain.
			Out.TrimStartAndEndInline();
			UE_LOG(LogAerotelicMeshLib, Log, TEXT("TACS interpreter probe: wsl exit %d in %.2f s: '%s' %s"), Code, FPlatformTime::Seconds() - T0, *Out, *Err.Left(200));
			if (!bRan || Code != 0 || !Out.StartsWith(TEXT("/"))) return FString();
			FString Line;
			Out.Split(TEXT("\n"), &Line, nullptr);
			return Line.IsEmpty() ? Out : Line.TrimStartAndEnd();
		}

		FCriticalSection GTacsLock;
		bool GTacsProbed = false;
		FInterpreter GTacs;
	}

	FInterpreter FindTacsInterpreter(bool bRefresh)
	{
		FScopeLock Lock(&GTacsLock);
		if (GTacsProbed && !bRefresh) return GTacs;
		GTacsProbed = true;
		GTacs = FInterpreter();
		const FString FromEnv = FPlatformMisc::GetEnvironmentVariable(TEXT("AEROTELIC_TACS_PYTHON")).TrimStartAndEnd();
		if (!FromEnv.IsEmpty())
		{
			if (FromEnv.StartsWith(TEXT("wsl:")))
			{
				// wsl:<distro>:<python>; the distro may be empty.
				const FString Rest = FromEnv.Mid(4);
				FString Distro, Python;
				if (!Rest.Split(TEXT(":"), &Distro, &Python)) Python = Rest;
				GTacs.bWsl = true;
				GTacs.Distro = Distro.TrimStartAndEnd();
				GTacs.Python = Python.TrimStartAndEnd();
				if (GTacs.Python.IsEmpty()) GTacs.Python = ProbeWslTacsPython();
			}
			else if (FPaths::FileExists(FromEnv))
			{
				GTacs.Python = FPaths::ConvertRelativePathToFull(FromEnv);
			}
			return GTacs;
		}
		const FString Probed = ProbeWslTacsPython();
		if (!Probed.IsEmpty())
		{
			GTacs.bWsl = true;
			GTacs.Python = Probed;
		}
		return GTacs;
	}

	namespace
	{
		bool RunProcess(const FString& Exe, const FString& Params, const FString& WorkingDir, const FString& Name, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress);
	}

	bool RunJob(const FString& Script, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress)
	{
		const FString Python = FindPython();
		if (Python.IsEmpty())
		{
			OutError = TEXT("The backend's Python interpreter was not found: point AEROTELIC_MESHLIB_PYTHON at a python.exe that has the libraries, or install them in the WingDesigner micromamba environment.");
			return false;
		}
		FInterpreter I;
		I.Python = Python;
		return RunJob(I, Script, Dir, OutResult, OutError, Progress);
	}

	bool RunJob(const FInterpreter& Interpreter, const FString& Script, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress)
	{
		if (!Interpreter.IsValid())
		{
			OutError = TEXT("The backend's Python interpreter was not found.");
			return false;
		}
		const FString Name = FPaths::GetCleanFilename(Script);
		const FString WorkingDir = FPaths::GetPath(Script);
		if (Interpreter.bWsl)
		{
			const FString Wsl = WslExe();
			if (Wsl.IsEmpty())
			{
				OutError = TEXT("wsl.exe was not found: the Windows Subsystem for Linux is not installed.");
				return false;
			}
			// -u: unbuffered, so the stage lines reach the dialog as they are
			// printed. The paths cross into the distribution as /mnt/<drive>/...
			const FString Params = FString::Printf(TEXT("%s-e \"%s\" -u \"%s\" \"%s\""),
				Interpreter.Distro.IsEmpty() ? TEXT("") : *FString::Printf(TEXT("-d \"%s\" "), *Interpreter.Distro),
				*Interpreter.Python, *ToWslPath(Script), *ToWslPath(Dir));
			return RunProcess(Wsl, Params, WorkingDir, Name, Dir, OutResult, OutError, Progress);
		}
		const FString Params = FString::Printf(TEXT("\"%s\" \"%s\""), *Script, *Dir);
		return RunProcess(Interpreter.Python, Params, WorkingDir, Name, Dir, OutResult, OutError, Progress);
	}

	namespace
	{
	bool RunProcess(const FString& Python, const FString& Params, const FString& WorkingDir, const FString& Name, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress)
	{
		void* ReadPipe = nullptr; void* WritePipe = nullptr;
		FPlatformProcess::CreatePipe(ReadPipe, WritePipe);
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
}
