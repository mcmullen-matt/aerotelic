#include "AerotelicStructuralFea.h"
#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include "aerotelic/laminate.hpp"

namespace AerotelicStructuralFea
{
	namespace
	{
		FString Num(double V)
		{
			return FString::Printf(TEXT("%.17g"), V);
		}

		FString JsonString(const FString& S)
		{
			return TEXT("\"") + S.Replace(TEXT("\\"), TEXT("\\\\")).Replace(TEXT("\""), TEXT("\\\"")) + TEXT("\"");
		}

		// A material's card: the four figures, and for a LAMINATE its plies
		// as laid -- the EXPANDED stack (a symmetric laminate's mirror half
		// written out), outer face first, each ply with its angle, thickness
		// and the lamina's figures, the transverse shear moduli and
		// strengths resolved to the lamina's conventions (materials.hpp).
		FString MaterialJson(const aerotelic::Material& M)
		{
			FString Plies;
			if (M.IsLaminate())
			{
				for (const aerotelic::Ply& P : aerotelic::ExpandedStack(M))
				{
					const aerotelic::Lamina& L = P.props;
					Plies += FString::Printf(TEXT("%s{\"angle\": %s, \"thickness\": %s, \"density\": %s, \"lamina\": %s, \"form\": %s, ")
						TEXT("\"e1\": %s, \"e2\": %s, \"nu12\": %s, \"g12\": %s, \"g13\": %s, \"g23\": %s, ")
						TEXT("\"t1\": %s, \"c1\": %s, \"t2\": %s, \"c2\": %s, \"s12\": %s, \"s13\": %s, \"s23\": %s, \"locking_angle\": %s}"),
						Plies.IsEmpty() ? TEXT("") : TEXT(", "), *Num(P.angle), *Num(P.Thickness()), *Num(P.density), *JsonString(UTF8_TO_TCHAR(P.lamina.c_str())), *JsonString(UTF8_TO_TCHAR(L.form.c_str())),
						*Num(L.e1), *Num(L.e2), *Num(L.nu12), *Num(L.g12), *Num(L.G13()), *Num(L.G23()),
						*Num(L.t1), *Num(L.c1), *Num(L.t2), *Num(L.c2), *Num(L.s12), *Num(L.S13()), *Num(L.S23()), *Num(L.locking_angle));
				}
				Plies = TEXT(", \"plies\": [") + Plies + TEXT("]");
			}
			return FString::Printf(TEXT("{\"name\": %s, \"density\": %s, \"youngs_modulus\": %s, \"poisson_ratio\": %s, \"allowable_stress\": %s%s}"),
				*JsonString(UTF8_TO_TCHAR(M.name.c_str())), *Num(M.density), *Num(M.youngs_modulus), *Num(M.poisson_ratio), *Num(M.allowable_stress), *Plies);
		}

		// axes.bin / rib_axes.bin: float64 xyz per face, in the faces' order.
		bool WriteAxes(const TArray<aerotelic::Vec3>& Axes, const FString& Path)
		{
			TArray<uint8> Bytes;
			Bytes.Reserve(Axes.Num() * 24);
			for (const aerotelic::Vec3& A : Axes)
			{
				const double Xyz[3] = { A.x, A.y, A.z };
				Bytes.Append(reinterpret_cast<const uint8*>(Xyz), sizeof(Xyz));
			}
			return FFileHelper::SaveArrayToFile(Bytes, *Path);
		}

		// A region's drape: float32 (deviation, shear) per face per ply.
		bool WriteDrape(const TArray<float>& Drape, const FString& Path)
		{
			TArray<uint8> Bytes;
			Bytes.Append(reinterpret_cast<const uint8*>(Drape.GetData()), Drape.Num() * sizeof(float));
			return FFileHelper::SaveArrayToFile(Bytes, *Path);
		}

		// nodes.bin for the script, already in metres: uint32 count, then
		// float64 xyz per node. No magic (unlike tools/rib_gmsh.py's
		// nodes.bin, which the model builder reads, not the solver).
		bool WriteNodes(const std::vector<aerotelic::Vec3>& Nodes, double Scale, const FString& Path)
		{
			TArray<uint8> Bytes;
			const uint32 Nn = static_cast<uint32>(Nodes.size());
			Bytes.Reserve(4 + Nn * 24);
			Bytes.Append(reinterpret_cast<const uint8*>(&Nn), sizeof(Nn));
			for (const aerotelic::Vec3& P : Nodes)
			{
				const double Xyz[3] = { P.x * Scale, P.y * Scale, P.z * Scale };
				Bytes.Append(reinterpret_cast<const uint8*>(Xyz), sizeof(Xyz));
			}
			return FFileHelper::SaveArrayToFile(Bytes, *Path);
		}

		// faces.bin / ribs.bin: a uint32 count, int32 node triples, then an
		// int32 region per face.
		bool WriteFaces(const TArray<FShellFace>& Faces, const FString& Path)
		{
			TArray<uint8> Bytes;
			const uint32 Nf = static_cast<uint32>(Faces.Num());
			Bytes.Reserve(4 + Nf * 16);
			Bytes.Append(reinterpret_cast<const uint8*>(&Nf), sizeof(Nf));
			for (const FShellFace& F : Faces)
			{
				Bytes.Append(reinterpret_cast<const uint8*>(F.N), sizeof(F.N));
			}
			for (const FShellFace& F : Faces)
			{
				Bytes.Append(reinterpret_cast<const uint8*>(&F.Region), sizeof(F.Region));
			}
			return FFileHelper::SaveArrayToFile(Bytes, *Path);
		}

		void ReadVec3(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, double Out[3])
		{
			const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
			if (Obj->TryGetArrayField(Field, Arr) && Arr->Num() == 3)
			{
				for (int32 k = 0; k < 3; ++k)
				{
					double V = 0.0;
					if ((*Arr)[k].IsValid() && (*Arr)[k]->TryGetNumber(V)) Out[k] = V;
				}
			}
		}

		double NumberOr(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, double Default = 0.0)
		{
			double V = Default;
			Obj->TryGetNumberField(Field, V);
			return V;
		}
	}

	const TCHAR* BackendName(EBackend Backend)
	{
		return Backend == EBackend::Tacs ? TEXT("TACS") : TEXT("Kratos FEA");
	}

	const TCHAR* BackendKey(EBackend Backend)
	{
		return Backend == EBackend::Tacs ? TEXT("tacs") : TEXT("kratos");
	}

	FString FindScript(EBackend Backend)
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), Backend == EBackend::Tacs ? TEXT("structural_fea_tacs.py") : TEXT("structural_fea.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool IsAvailable(EBackend Backend, FString& OutWhy)
	{
		if (FindScript(Backend).IsEmpty())
		{
			OutWhy = FString::Printf(TEXT("tools\\%s is not installed beside the application."), Backend == EBackend::Tacs ? TEXT("structural_fea_tacs.py") : TEXT("structural_fea.py"));
			return false;
		}
		if (Backend == EBackend::Tacs)
		{
			if (!AerotelicMeshLib::FindTacsInterpreter().IsValid())
			{
				OutWhy = TEXT("The TACS environment was not found. TACS has no Windows build: install the Windows Subsystem for Linux (wsl --install, as administrator, then reboot), a distribution (wsl --install -d Ubuntu-24.04), and in it\n"
					"    micromamba create -n tacs -c conda-forge -c smdogroup python=3.11 tacs\n"
					"so that ~/micromamba/envs/tacs/bin/python exists; or set AEROTELIC_TACS_PYTHON=wsl:<distro>:<path to that python>. Analysis > TACS (FEA) > Check Installation looks again.");
				return false;
			}
			return true;
		}
		if (AerotelicMeshLib::FindPython().IsEmpty())
		{
			OutWhy = TEXT("The Python interpreter was not found: install Kratos (pip install KratosMultiphysics KratosStructuralMechanicsApplication KratosLinearSolversApplication scipy, Kratos all at the same version) in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has them.");
			return false;
		}
		return true;
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError, AerotelicMeshLib::FJobProgress* Progress)
	{
		if (Request.Nodes == nullptr || Request.Nodes->empty() || (Request.ShellFaces.Num() == 0 && Request.RibFaces.Num() == 0))
		{
			OutError = TEXT("nothing to analyse: no shells");
			return false;
		}
		if (Request.Regions.Num() == 0)
		{
			OutError = TEXT("no region for the shells");
			return false;
		}
		for (const FRegion& R : Request.Regions)
		{
			if (!R.HasShells() && !R.HasRibs())
			{
				OutError = FString::Printf(TEXT("'%s' is neither skin shells nor ribs: give the body a skin thickness, or the network a thickness, under Structure... in the Scene tree."), *R.Label);
				return false;
			}
			if (R.HasShells() && !R.Skin.IsStructural())
			{
				OutError = FString::Printf(TEXT("'%s' has no structural material for its skin: give it a density, a Young's modulus and a Poisson's ratio under Structure in the Scene tree."), *R.Label);
				return false;
			}
			if (R.HasRibs() && !R.Rib.IsStructural())
			{
				OutError = FString::Printf(TEXT("'%s' has no structural material for its ribs: give the network a density, a Young's modulus and a Poisson's ratio under Structure in the Scene tree."), *R.Label);
				return false;
			}
		}
		if (Request.VtkPath.IsEmpty())
		{
			OutError = TEXT("no solution file to write");
			return false;
		}
		if (!IsAvailable(Request.Backend, OutError))
		{
			return false;
		}
		const FString Script = FindScript(Request.Backend);
		AerotelicMeshLib::FInterpreter Interpreter;
		if (Request.Backend == EBackend::Tacs)
		{
			Interpreter = AerotelicMeshLib::FindTacsInterpreter();
		}
		else
		{
			Interpreter.Python = AerotelicMeshLib::FindPython();
		}

		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_fea_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		// AEROTELIC_FEA_KEEP=1 keeps the job directory (nodes.bin, faces.bin,
		// ribs.bin, job.json, the Kratos files) for running the script by hand.
		const bool bKeep = !FPlatformMisc::GetEnvironmentVariable(TEXT("AEROTELIC_FEA_KEEP")).IsEmpty();
		if (bKeep) UE_LOG(LogTemp, Display, TEXT("AerotelicStructuralFea: keeping the job directory %s"), *Dir);
		auto Cleanup = [&Dir, bKeep]() { if (!bKeep) IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		const double L = Request.LengthToSI;
		if (!WriteNodes(*Request.Nodes, L, FPaths::Combine(Dir, TEXT("nodes.bin"))))
		{
			OutError = TEXT("could not write the nodes for the solver");
			Cleanup();
			return false;
		}
		FString FacesFile;
		if (Request.ShellFaces.Num() > 0)
		{
			FacesFile = TEXT("faces.bin");
			if (!WriteFaces(Request.ShellFaces, FPaths::Combine(Dir, FacesFile)))
			{
				OutError = TEXT("could not write the shell faces for the solver");
				Cleanup();
				return false;
			}
		}
		FString RibsFile;
		if (Request.RibFaces.Num() > 0)
		{
			RibsFile = TEXT("ribs.bin");
			if (!WriteFaces(Request.RibFaces, FPaths::Combine(Dir, RibsFile)))
			{
				OutError = TEXT("could not write the rib faces for the solver");
				Cleanup();
				return false;
			}
		}
		FString PressureFile;
		if (Request.PressurePerFace.Num() > 0)
		{
			TArray<uint8> Bytes;
			Bytes.Append(reinterpret_cast<const uint8*>(Request.PressurePerFace.GetData()), Request.PressurePerFace.Num() * sizeof(double));
			PressureFile = TEXT("pressure.bin");
			if (!FFileHelper::SaveArrayToFile(Bytes, *FPaths::Combine(Dir, PressureFile)))
			{
				OutError = TEXT("could not write the pressures for the solver");
				Cleanup();
				return false;
			}
		}

		// The material axes per face, when the builder gave them.
		FString AxesFile, RibAxesFile;
		if (Request.ShellAxes.Num() == Request.ShellFaces.Num() && Request.ShellAxes.Num() > 0)
		{
			AxesFile = TEXT("axes.bin");
			if (!WriteAxes(Request.ShellAxes, FPaths::Combine(Dir, AxesFile)))
			{
				OutError = TEXT("could not write the material axes for the solver");
				Cleanup();
				return false;
			}
		}
		if (Request.RibAxes.Num() == Request.RibFaces.Num() && Request.RibAxes.Num() > 0)
		{
			RibAxesFile = TEXT("rib_axes.bin");
			if (!WriteAxes(Request.RibAxes, FPaths::Combine(Dir, RibAxesFile)))
			{
				OutError = TEXT("could not write the rib material axes for the solver");
				Cleanup();
				return false;
			}
		}

		// The regions: a skin card, a rib card, or null for each; a laminate
		// card names its drape file when the builder draped it.
		FString Regions;
		for (int32 i = 0; i < Request.Regions.Num(); ++i)
		{
			const FRegion& R = Request.Regions[i];
			FString Skin = TEXT("null"), Rib = TEXT("null");
			auto Card = [&](const aerotelic::Material& M, double Thickness, const TArray<float>& Drape, const TCHAR* Prefix) -> FString
			{
				FString DrapeJson = TEXT("null");
				if (M.IsLaminate() && Drape.Num() > 0)
				{
					const FString File = FString::Printf(TEXT("%s_%d.bin"), Prefix, R.Id);
					if (WriteDrape(Drape, FPaths::Combine(Dir, File))) DrapeJson = JsonString(File);
					else UE_LOG(LogTemp, Warning, TEXT("AerotelicStructuralFea: could not write %s; the region is analysed undraped"), *File);
				}
				return FString::Printf(TEXT("{\"material\": %s, \"thickness\": %s, \"drape_file\": %s}"), *MaterialJson(M), *Num(Thickness * L), *DrapeJson);
			};
			if (R.HasShells()) Skin = Card(R.Skin, R.SkinThickness, R.SkinDrape, TEXT("drape"));
			if (R.HasRibs()) Rib = Card(R.Rib, R.RibThickness, R.RibDrape, TEXT("rib_drape"));
			Regions += FString::Printf(TEXT("%s{\"id\": %d, \"label\": %s, \"skin\": %s, \"rib\": %s}"),
				i ? TEXT(", ") : TEXT(""), R.Id, *JsonString(R.Label), *Skin, *Rib);
		}

		if (Request.Supports.Num() == 0 && !Request.bFreeFlight)
		{
			OutError = TEXT("no support: give at least one plane whose nodes are fixed, or choose free flight (inertia relief)");
			Cleanup();
			return false;
		}
		if (Request.Supports.Num() > 0 && Request.bFreeFlight)
		{
			OutError = TEXT("free flight takes no support planes: the model is held by its own inertia and one reference node");
			Cleanup();
			return false;
		}
		for (const FMassItem& M : Request.Masses)
		{
			if (!(M.Mass > 0.0))
			{
				OutError = FString::Printf(TEXT("mass item '%s' has no mass"), *M.Name);
				Cleanup();
				return false;
			}
			if (M.AttachNodes.Num() < 3)
			{
				OutError = FString::Printf(TEXT("mass item '%s' hangs on %d node(s); it needs at least three, not in a line"), *M.Name, M.AttachNodes.Num());
				Cleanup();
				return false;
			}
			for (int32 N : M.AttachNodes)
			{
				if (N < 0 || N >= static_cast<int32>(Request.Nodes->size()))
				{
					OutError = FString::Printf(TEXT("mass item '%s' hangs on a node the mesh does not have"), *M.Name);
					Cleanup();
					return false;
				}
			}
		}
		FString MassesJson;
		for (int32 i = 0; i < Request.Masses.Num(); ++i)
		{
			const FMassItem& M = Request.Masses[i];
			FString Attach;
			for (int32 k = 0; k < M.AttachNodes.Num(); ++k) Attach += FString::Printf(TEXT("%s%d"), k ? TEXT(", ") : TEXT(""), M.AttachNodes[k]);
			bool bInertia = false;
			for (double V : M.Inertia) bInertia = bInertia || V != 0.0;
			const FString Inertia = bInertia
				? FString::Printf(TEXT("[%s, %s, %s, %s, %s, %s]"), *Num(M.Inertia[0]), *Num(M.Inertia[1]), *Num(M.Inertia[2]), *Num(M.Inertia[3]), *Num(M.Inertia[4]), *Num(M.Inertia[5]))
				: FString(TEXT("null"));
			MassesJson += FString::Printf(TEXT("%s{\"name\": %s, \"mass\": %s, \"at\": [%s, %s, %s], \"inertia\": %s, \"attach\": [%s]}"), i ? TEXT(", ") : TEXT(""),
				*JsonString(M.Name), *Num(M.Mass), *Num(M.At.x * L), *Num(M.At.y * L), *Num(M.At.z * L), *Inertia, *Attach);
		}
		// The hinge ties, offset links and bars (2026-11-14): checked here so
		// the message names the culprit before the script is started.
		const int32 NNodes = static_cast<int32>(Request.Nodes->size());
		FString TiesJson;
		for (int32 i = 0; i < Request.Ties.Num(); ++i)
		{
			const FTie& T = Request.Ties[i];
			if (T.Master < 0 || T.Master >= NNodes || T.Slave < 0 || T.Slave >= NNodes || T.Master == T.Slave)
			{
				OutError = FString::Printf(TEXT("hinge tie %d names nodes the mesh does not have (%d, %d)"), i + 1, T.Master, T.Slave);
				Cleanup();
				return false;
			}
			TiesJson += FString::Printf(TEXT("%s{\"master\": %d, \"slave\": %d}"), i ? TEXT(", ") : TEXT(""), T.Master, T.Slave);
		}
		FString LinksJson;
		for (int32 i = 0; i < Request.Links.Num(); ++i)
		{
			const FLink& K = Request.Links[i];
			if (K.On < 0 || K.On >= NNodes)
			{
				OutError = FString::Printf(TEXT("offset link '%s' stands off a node the mesh does not have (%d)"), *K.Name, K.On);
				Cleanup();
				return false;
			}
			LinksJson += FString::Printf(TEXT("%s{\"name\": %s, \"on\": %d, \"at\": [%s, %s, %s]}"), i ? TEXT(", ") : TEXT(""), *JsonString(K.Name), K.On, *Num(K.At.x * L), *Num(K.At.y * L), *Num(K.At.z * L));
		}
		FString BarsJson;
		for (int32 i = 0; i < Request.Bars.Num(); ++i)
		{
			const FBar& Bar = Request.Bars[i];
			auto End = [&](const FBarEnd& E, FString& Json) -> bool
			{
				if (E.Link >= 0)
				{
					if (E.Link >= Request.Links.Num()) return false;
					Json = FString::Printf(TEXT("{\"link\": %d}"), E.Link);
					return true;
				}
				if (E.Node < 0 || E.Node >= NNodes) return false;
				Json = FString::Printf(TEXT("{\"node\": %d}"), E.Node);
				return true;
			};
			FString AJson, BJson;
			if (!End(Bar.A, AJson) || !End(Bar.B, BJson))
			{
				OutError = FString::Printf(TEXT("bar '%s' ends on a node or link the model does not have"), *Bar.Name);
				Cleanup();
				return false;
			}
			if (!(Bar.Area > 0.0) || !(Bar.YoungsModulus > 0.0) || !(Bar.Density > 0.0))
			{
				OutError = FString::Printf(TEXT("bar '%s' needs a positive area, modulus and density"), *Bar.Name);
				Cleanup();
				return false;
			}
			BarsJson += FString::Printf(TEXT("%s{\"name\": %s, \"a\": %s, \"b\": %s, \"area\": %s, \"youngs_modulus\": %s, \"density\": %s}"), i ? TEXT(", ") : TEXT(""),
				*JsonString(Bar.Name), *AJson, *BJson, *Num(Bar.Area), *Num(Bar.YoungsModulus), *Num(Bar.Density));
		}
		const FString ReliefJson = Request.bFreeFlight
			? (Request.bReferenceGiven
				? FString::Printf(TEXT("{\"reference\": [%s, %s, %s]}"), *Num(Request.Reference.x * L), *Num(Request.Reference.y * L), *Num(Request.Reference.z * L))
				: FString(TEXT("{\"reference\": null}")))
			: FString(TEXT("null"));
		FString SupportsJson;
		for (int32 i = 0; i < Request.Supports.Num(); ++i)
		{
			const FSupport& S = Request.Supports[i];
			const FString Value = S.bAtMin ? TEXT("\"min\"") : (S.bAtMax ? TEXT("\"max\"") : Num(S.Value * L));
			const FString Bounds = S.bBounded
				? FString::Printf(TEXT("[%s, %s, %s, %s]"), *Num(S.Lo[0] * L), *Num(S.Hi[0] * L), *Num(S.Lo[1] * L), *Num(S.Hi[1] * L))
				: FString(TEXT("null"));
			SupportsJson += FString::Printf(TEXT("%s{\"axis\": \"%c\", \"value\": %s, \"tolerance\": %s, \"bounds\": %s}"), i ? TEXT(", ") : TEXT(""), FChar::ToLower(S.Axis), *Value, *Num(S.Tolerance * L), *Bounds);
		}
		FString PointsJson;
		for (int32 i = 0; i < Request.PointForces.Num(); ++i)
		{
			const FPointForce& P = Request.PointForces[i];
			PointsJson += FString::Printf(TEXT("%s{\"at\": [%s, %s, %s], \"force\": [%s, %s, %s]}"), i ? TEXT(", ") : TEXT(""),
				*Num(P.At.x * L), *Num(P.At.y * L), *Num(P.At.z * L), *Num(P.Force.x), *Num(P.Force.y), *Num(P.Force.z));
		}
		const FString Job = FString::Printf(
			TEXT("{\n \"nodes\": \"nodes.bin\",\n \"faces\": %s,\n \"rib_faces\": %s,\n \"axes\": %s,\n \"rib_axes\": %s,\n \"regions\": [%s],\n")
			TEXT(" \"supports\": [%s],\n \"inertia_relief\": %s,\n \"masses\": [%s],\n \"ties\": [%s],\n \"links\": [%s],\n \"bars\": [%s],\n")
			TEXT(" \"loads\": {\"load_factor\": %s, \"pressure\": %s, \"pressure_file\": %s, \"point_forces\": [%s]},\n")
			TEXT(" \"solver\": {\"backend\": %s, \"echo_level\": 0, \"linear_solver\": %s, \"shell_element\": %s, \"modes\": %d},\n")
			TEXT(" \"outputs\": {\"vtk\": %s, \"result\": \"result.json\"}\n}\n"),
			FacesFile.IsEmpty() ? TEXT("null") : *JsonString(FacesFile), RibsFile.IsEmpty() ? TEXT("null") : *JsonString(RibsFile),
			AxesFile.IsEmpty() ? TEXT("null") : *JsonString(AxesFile), RibAxesFile.IsEmpty() ? TEXT("null") : *JsonString(RibAxesFile), *Regions,
			*SupportsJson, *ReliefJson, *MassesJson, *TiesJson, *LinksJson, *BarsJson,
			*Num(Request.LoadFactor), *Num(Request.Pressure), PressureFile.IsEmpty() ? TEXT("null") : *JsonString(PressureFile), *PointsJson,
			*JsonString(BackendKey(Request.Backend)),
			*JsonString(Request.LinearSolver.IsEmpty() ? FString(TEXT("auto")) : Request.LinearSolver),
			*JsonString(Request.ShellElement.IsEmpty() ? FString(TEXT("ShellThinElement3D3N")) : Request.ShellElement),
			FMath::Max(Request.Modes, 0),
			*JsonString(FPaths::ConvertRelativePathToFull(Request.VtkPath)));
		if (!FFileHelper::SaveStringToFile(Job, *FPaths::Combine(Dir, TEXT("job.json")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			OutError = TEXT("could not write the solver's job file");
			Cleanup();
			return false;
		}

		TSharedPtr<FJsonObject> Result;
		if (!AerotelicMeshLib::RunJob(Interpreter, Script, Dir, Result, OutError, Progress))
		{
			Cleanup();
			return false;
		}

		auto Int = [&Result](const TCHAR* Field) { return static_cast<int32>(NumberOr(Result, Field)); };
		auto Real = [&Result](const TCHAR* Field) { return NumberOr(Result, Field); };
		Out = FResult();
		Out.Nodes = Int(TEXT("nodes"));
		Out.Elements = Int(TEXT("elements"));
		Out.Shells = Int(TEXT("shells"));
		Out.RibShells = Int(TEXT("rib_shells"));
		Out.Dofs = Int(TEXT("dofs"));
		Out.SupportNodes = Int(TEXT("support_nodes"));
		Out.SupportCornerNodes = Result->HasField(TEXT("support_corner_nodes")) ? Int(TEXT("support_corner_nodes")) : Out.SupportNodes;
		Out.WettedFaces = Int(TEXT("wetted_faces"));
		Out.Mass = Real(TEXT("mass"));
		Out.MassStructure = Result->HasField(TEXT("mass_structure")) ? Real(TEXT("mass_structure")) : Out.Mass;
		Out.MassItems = Real(TEXT("mass_items"));
		ReadVec3(Result, TEXT("mass_centre"), Out.MassCentre);
		{
			const TArray<TSharedPtr<FJsonValue>>* Rows = nullptr;
			if (Result->TryGetArrayField(TEXT("inertia_cg"), Rows) && Rows->Num() == 3)
			{
				for (int32 r = 0; r < 3; ++r)
				{
					const TArray<TSharedPtr<FJsonValue>>* Row = nullptr;
					if (!(*Rows)[r].IsValid() || !(*Rows)[r]->TryGetArray(Row) || Row->Num() != 3) continue;
					for (int32 c = 0; c < 3; ++c)
					{
						double V = 0.0;
						if ((*Row)[c].IsValid() && (*Row)[c]->TryGetNumber(V)) Out.InertiaCg[3 * r + c] = V;
					}
				}
			}
		}
		FString Mode;
		Result->TryGetStringField(TEXT("support_mode"), Mode);
		Out.bFreeFlight = Mode == TEXT("free flight");
		Out.SupportNode = Result->HasField(TEXT("support_node")) ? Int(TEXT("support_node")) : -1;
		ReadVec3(Result, TEXT("support_at"), Out.SupportAt);
		ReadVec3(Result, TEXT("reaction_moment"), Out.ReactionMoment);
		ReadVec3(Result, TEXT("applied_inertial"), Out.AppliedInertial);
		ReadVec3(Result, TEXT("rigid_acceleration"), Out.RigidAcceleration);
		ReadVec3(Result, TEXT("rigid_angular_acceleration"), Out.RigidAngularAcceleration);
		Out.LoadFactorAtCg = Result->HasField(TEXT("load_factor_at_cg")) ? Real(TEXT("load_factor_at_cg")) : Request.LoadFactor;
		Out.MomentError = Real(TEXT("moment_error"));
		{
			const TArray<TSharedPtr<FJsonValue>>* Items = nullptr;
			if (Result->TryGetArrayField(TEXT("items"), Items))
			{
				for (const TSharedPtr<FJsonValue>& V : *Items)
				{
					const TSharedPtr<FJsonObject>* Obj = nullptr;
					if (!V.IsValid() || !V->TryGetObject(Obj)) continue;
					FResult::FItem I;
					(*Obj)->TryGetStringField(TEXT("name"), I.Name);
					I.Mass = NumberOr(*Obj, TEXT("mass"));
					I.AttachNodes = static_cast<int32>(NumberOr(*Obj, TEXT("attach_nodes")));
					ReadVec3(*Obj, TEXT("displacement"), I.Displacement);
					ReadVec3(*Obj, TEXT("force"), I.Force);
					Out.Items.Add(I);
				}
			}
		}
		// The hinges and pushrods (2026-11-14).
		Out.Ties = Int(TEXT("ties"));
		Out.Links = Int(TEXT("links"));
		Out.MassBars = Real(TEXT("mass_bars"));
		{
			const TArray<TSharedPtr<FJsonValue>>* Bars = nullptr;
			if (Result->TryGetArrayField(TEXT("bars"), Bars))
			{
				for (const TSharedPtr<FJsonValue>& V : *Bars)
				{
					const TSharedPtr<FJsonObject>* Obj = nullptr;
					if (!V.IsValid() || !V->TryGetObject(Obj)) continue;
					FResult::FBarResult Bar;
					(*Obj)->TryGetStringField(TEXT("name"), Bar.Name);
					Bar.Force = NumberOr(*Obj, TEXT("force"));
					Bar.Stress = NumberOr(*Obj, TEXT("stress"));
					Bar.Length = NumberOr(*Obj, TEXT("length"));
					Bar.Mass = NumberOr(*Obj, TEXT("mass"));
					ReadVec3(*Obj, TEXT("a"), Bar.A);
					ReadVec3(*Obj, TEXT("b"), Bar.B);
					Out.Bars.Add(Bar);
				}
			}
		}
		Out.MaxDisplacement = Real(TEXT("max_displacement"));
		ReadVec3(Result, TEXT("max_displacement_at"), Out.MaxDisplacementAt);
		Out.MaxVonMises = Real(TEXT("max_von_mises"));
		Out.MaxVonMisesRegion = Int(TEXT("max_von_mises_material"));
		const TArray<TSharedPtr<FJsonValue>>* PerMaterial = nullptr;
		if (Result->TryGetArrayField(TEXT("per_material"), PerMaterial))
		{
			for (const TSharedPtr<FJsonValue>& V : *PerMaterial)
			{
				const TSharedPtr<FJsonObject>* Obj = nullptr;
				if (V.IsValid() && V->TryGetObject(Obj))
				{
					FEntryResult E;
					E.Region = static_cast<int32>(NumberOr(*Obj, TEXT("id")));
					(*Obj)->TryGetStringField(TEXT("label"), E.Label);
					(*Obj)->TryGetStringField(TEXT("kind"), E.Kind);
					E.MaxVonMises = NumberOr(*Obj, TEXT("max_von_mises"));
					E.Margin = NumberOr(*Obj, TEXT("margin"));
					E.Allowable = NumberOr(*Obj, TEXT("allowable"));
					E.Mass = NumberOr(*Obj, TEXT("mass"));
					if (!(*Obj)->TryGetStringField(TEXT("criterion"), E.Criterion)) E.Criterion = TEXT("von Mises");
					E.ReserveFactor = NumberOr(*Obj, TEXT("reserve_factor"));
					E.Plies = static_cast<int32>(NumberOr(*Obj, TEXT("plies")));
					E.Thickness = NumberOr(*Obj, TEXT("thickness"));
					E.Stacks = static_cast<int32>(NumberOr(*Obj, TEXT("stacks")));
					const TSharedPtr<FJsonObject>* DrapeObj = nullptr;
					if ((*Obj)->TryGetObjectField(TEXT("drape"), DrapeObj) && DrapeObj != nullptr && DrapeObj->IsValid())
					{
						E.bDraped = true;
						E.DrapeMaxShear = NumberOr(*DrapeObj, TEXT("max_shear"));
						E.DrapeMaxDeviation = NumberOr(*DrapeObj, TEXT("max_deviation"));
						E.DrapeFacesPastLocking = static_cast<int32>(NumberOr(*DrapeObj, TEXT("faces_past_locking")));
					}
					Out.Entries.Add(E);
				}
			}
		}
		Out.Laminates = Int(TEXT("laminates"));
		ReadVec3(Result, TEXT("reaction"), Out.Reaction);
		ReadVec3(Result, TEXT("applied_gravity"), Out.AppliedGravity);
		ReadVec3(Result, TEXT("applied_pressure"), Out.AppliedPressure);
		ReadVec3(Result, TEXT("applied_point"), Out.AppliedPoint);
		Out.BalanceError = Real(TEXT("balance_error"));
		if (!Result->TryGetBoolField(TEXT("balanced"), Out.bBalanced))
		{
			Out.bBalanced = true;   // an older script: no verdict, so no alarm
		}
		Result->TryGetStringField(TEXT("linear_solver"), Out.LinearSolver);
		const TArray<TSharedPtr<FJsonValue>>* WarningsJson = nullptr;
		if (Result->TryGetArrayField(TEXT("warnings"), WarningsJson))
		{
			for (const TSharedPtr<FJsonValue>& W : *WarningsJson)
			{
				FString S;
				if (W.IsValid() && W->TryGetString(S)) Out.Warnings.Add(S);
			}
		}
		Out.MeanNormalDisplacement = Real(TEXT("mean_normal_displacement"));
		const TArray<TSharedPtr<FJsonValue>>* Frequencies = nullptr;
		if (Result->TryGetArrayField(TEXT("natural_frequencies"), Frequencies))
		{
			for (const TSharedPtr<FJsonValue>& F : *Frequencies)
			{
				double V = 0.0;
				if (F.IsValid() && F->TryGetNumber(V)) Out.NaturalFrequencies.Add(V);
			}
		}
		Result->TryGetStringField(TEXT("eigensolver"), Out.Eigensolver);
		Out.SecondsModes = Real(TEXT("seconds_modes"));
		Out.SecondsSolve = Real(TEXT("seconds_solve"));
		Out.Seconds = Real(TEXT("seconds"));
		Result->TryGetStringField(TEXT("vtk"), Out.VtkPath);
		if (Out.VtkPath.IsEmpty())
		{
			Out.VtkPath = Request.VtkPath;
		}
		// Which solver ran, and TACS's adjoint (2026-11-17).
		if (!Result->TryGetStringField(TEXT("backend"), Out.Backend)) Out.Backend = BackendKey(Request.Backend);
		Result->TryGetStringField(TEXT("tacs_version"), Out.SolverVersion);
		{
			const TSharedPtr<FJsonObject>* Sens = nullptr;
			if (Result->TryGetObjectField(TEXT("sensitivities"), Sens) && Sens != nullptr && Sens->IsValid())
			{
				const TArray<TSharedPtr<FJsonValue>>* Vars = nullptr;
				if ((*Sens)->TryGetArrayField(TEXT("design_variables"), Vars))
				{
					for (const TSharedPtr<FJsonValue>& V : *Vars)
					{
						const TSharedPtr<FJsonObject>* Obj = nullptr;
						if (!V.IsValid() || !V->TryGetObject(Obj)) continue;
						FResult::FDesignVariable D;
						(*Obj)->TryGetStringField(TEXT("component"), D.Component);
						(*Obj)->TryGetStringField(TEXT("kind"), D.Kind);
						D.Region = static_cast<int32>(NumberOr(*Obj, TEXT("region")));
						D.Thickness = NumberOr(*Obj, TEXT("thickness"));
						Out.DesignVariables.Add(D);
					}
				}
				const TArray<TSharedPtr<FJsonValue>>* Funcs = nullptr;
				if ((*Sens)->TryGetArrayField(TEXT("functions"), Funcs))
				{
					for (const TSharedPtr<FJsonValue>& V : *Funcs)
					{
						const TSharedPtr<FJsonObject>* Obj = nullptr;
						if (!V.IsValid() || !V->TryGetObject(Obj)) continue;
						FResult::FSensitivity S;
						(*Obj)->TryGetStringField(TEXT("name"), S.Function);
						(*Obj)->TryGetStringField(TEXT("component"), S.Component);
						S.Region = static_cast<int32>(NumberOr(*Obj, TEXT("region")));
						S.Value = NumberOr(*Obj, TEXT("value"));
						const TArray<TSharedPtr<FJsonValue>>* Grad = nullptr;
						if ((*Obj)->TryGetArrayField(TEXT("gradient"), Grad))
						{
							for (const TSharedPtr<FJsonValue>& G : *Grad)
							{
								double X = 0.0;
								if (G.IsValid() && G->TryGetNumber(X)) S.Gradient.Add(X);
							}
						}
						Out.Sensitivities.Add(S);
					}
				}
			}
		}
		Cleanup();
		return true;
	}
}
