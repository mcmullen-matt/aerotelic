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

		FString MaterialJson(const aerotelic::Material& M)
		{
			return FString::Printf(TEXT("{\"name\": %s, \"density\": %s, \"youngs_modulus\": %s, \"poisson_ratio\": %s, \"allowable_stress\": %s}"),
				*JsonString(UTF8_TO_TCHAR(M.name.c_str())), *Num(M.density), *Num(M.youngs_modulus), *Num(M.poisson_ratio), *Num(M.allowable_stress));
		}

		// The mesh for the script, already in metres: counts, then nodes as
		// float64 xyz, tetrahedra as int32 quads, region as int32 -- the
		// layout tools/tet_gmsh.py's write_tet uses, without its magic. No
		// tetrahedra at all is allowed (a shell-only model).
		bool WriteMesh(const aerotelic::TetMesh& Mesh, double Scale, const FString& Path)
		{
			TArray<uint8> Bytes;
			const uint32 Nn = static_cast<uint32>(Mesh.nodes.size());
			const uint32 Nt = static_cast<uint32>(Mesh.tets.size());
			Bytes.Reserve(8 + Nn * 24 + Nt * 20);
			Bytes.Append(reinterpret_cast<const uint8*>(&Nn), sizeof(Nn));
			Bytes.Append(reinterpret_cast<const uint8*>(&Nt), sizeof(Nt));
			for (const aerotelic::Vec3& P : Mesh.nodes)
			{
				const double Xyz[3] = { P.x * Scale, P.y * Scale, P.z * Scale };
				Bytes.Append(reinterpret_cast<const uint8*>(Xyz), sizeof(Xyz));
			}
			for (const auto& T : Mesh.tets)
			{
				const int32 Ids[4] = { T[0], T[1], T[2], T[3] };
				Bytes.Append(reinterpret_cast<const uint8*>(Ids), sizeof(Ids));
			}
			for (uint32 i = 0; i < Nt; ++i)
			{
				const int32 M = i < Mesh.material.size() ? Mesh.material[i] : 1;
				Bytes.Append(reinterpret_cast<const uint8*>(&M), sizeof(M));
			}
			return FFileHelper::SaveArrayToFile(Bytes, *Path);
		}

		// The faces of the shell-only regions: a count, int32 triples, then
		// an int32 region per face.
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

		// The voxelised unit cell for the homogeniser: the field at the voxel
		// centres as float32, and the JSON that describes it (the period in
		// metres; only its ratios matter to the result).
		bool WriteCell(const aerotelic::UnitCellSamples& Cell, double Scale, const FString& Dir, const FString& File, FString& OutJson)
		{
			const TArrayView64<const uint8> Raw(reinterpret_cast<const uint8*>(Cell.values.data()), static_cast<int64>(Cell.values.size() * sizeof(float)));
			if (!FFileHelper::SaveArrayToFile(Raw, *FPaths::Combine(Dir, File)))
			{
				return false;
			}
			OutJson = FString::Printf(TEXT("{\"file\": %s, \"nx\": %d, \"ny\": %d, \"nz\": %d, \"period\": [%s, %s, %s], \"solid_fraction\": %s}"),
				*JsonString(File), Cell.nx, Cell.ny, Cell.nz, *Num(Cell.period.x * Scale), *Num(Cell.period.y * Scale), *Num(Cell.period.z * Scale), *Num(Cell.solid_fraction));
			return true;
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

	FString FindScript()
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("structural_fea.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError, AerotelicMeshLib::FJobProgress* Progress)
	{
		if (Request.Mesh == nullptr || (Request.Mesh->tets.empty() && Request.ShellFaces.Num() == 0))
		{
			OutError = TEXT("nothing to analyse: no tetrahedra and no shells");
			return false;
		}
		if (Request.Regions.Num() == 0)
		{
			OutError = TEXT("no region for the mesh");
			return false;
		}
		for (const FRegion& R : Request.Regions)
		{
			if (!R.HasShells() && !R.HasTetrahedra())
			{
				OutError = FString::Printf(TEXT("'%s' is neither shells nor a solid nor a lattice core: give it a skin thickness or a solid material."), *R.Label);
				return false;
			}
			if (R.HasShells() && !R.Skin.IsStructural())
			{
				OutError = FString::Printf(TEXT("'%s' has no structural material for its skin: give it a density, a Young's modulus and a Poisson's ratio under Material Properties in the Scene tree."), *R.Label);
				return false;
			}
			if (R.HasTetrahedra() && !R.Core.IsStructural())
			{
				OutError = FString::Printf(TEXT("'%s' has no structural material: give it a density, a Young's modulus and a Poisson's ratio under Material Properties in the Scene tree."), *R.Label);
				return false;
			}
			if (R.Cell != nullptr && R.Cell->values.empty())
			{
				OutError = FString::Printf(TEXT("'%s' has no voxelised lattice cell to homogenise."), *R.Label);
				return false;
			}
		}
		if (Request.VtkPath.IsEmpty())
		{
			OutError = TEXT("no solution file to write");
			return false;
		}
		const FString Script = FindScript();
		if (Script.IsEmpty())
		{
			OutError = TEXT("tools\\structural_fea.py was not found beside the application.");
			return false;
		}
		if (AerotelicMeshLib::FindPython().IsEmpty())
		{
			OutError = TEXT("The Python interpreter was not found: install Kratos (pip install KratosMultiphysics KratosStructuralMechanicsApplication KratosLinearSolversApplication scipy, Kratos all at the same version) in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has them.");
			return false;
		}

		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_fea_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		// AEROTELIC_FEA_KEEP=1 keeps the job directory (mesh.bin, faces.bin,
		// job.json, the Kratos files) for running the script by hand.
		const bool bKeep = !FPlatformMisc::GetEnvironmentVariable(TEXT("AEROTELIC_FEA_KEEP")).IsEmpty();
		if (bKeep) UE_LOG(LogTemp, Display, TEXT("AerotelicStructuralFea: keeping the job directory %s"), *Dir);
		auto Cleanup = [&Dir, bKeep]() { if (!bKeep) IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		const double L = Request.LengthToSI;
		if (!WriteMesh(*Request.Mesh, L, FPaths::Combine(Dir, TEXT("mesh.bin"))))
		{
			OutError = TEXT("could not write the mesh for the solver");
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

		// The regions. Those sharing one cell (one lattice over several
		// bodies) share one file.
		TMap<const aerotelic::UnitCellSamples*, FString> CellJson;
		FString Regions;
		for (int32 i = 0; i < Request.Regions.Num(); ++i)
		{
			const FRegion& R = Request.Regions[i];
			FString Skin = TEXT("null"), Core = TEXT("null"), Solid = TEXT("null");
			if (R.HasShells())
			{
				Skin = FString::Printf(TEXT("{\"material\": %s, \"thickness\": %s}"), *MaterialJson(R.Skin), *Num(R.SkinThickness * L));
			}
			if (R.Cell != nullptr)
			{
				FString* Cell = CellJson.Find(R.Cell);
				if (Cell == nullptr)
				{
					FString Json;
					if (!WriteCell(*R.Cell, L, Dir, FString::Printf(TEXT("cell_%d.f32"), CellJson.Num() + 1), Json))
					{
						OutError = TEXT("could not write the lattice cell for the solver");
						Cleanup();
						return false;
					}
					Cell = &CellJson.Add(R.Cell, Json);
				}
				Core = FString::Printf(TEXT("{\"material\": %s, \"cell\": %s, \"density_correction\": true}"), *MaterialJson(R.Core), **Cell);
			}
			else if (R.bSolid)
			{
				Solid = FString::Printf(TEXT("{\"material\": %s}"), *MaterialJson(R.Core));
			}
			// The region's own element order, or null for the request's.
			const TCHAR* Order = R.SecondOrder < 0 ? TEXT("null") : (R.SecondOrder > 0 ? TEXT("true") : TEXT("false"));
			Regions += FString::Printf(TEXT("%s{\"id\": %d, \"label\": %s, \"skin\": %s, \"core\": %s, \"solid\": %s, \"second_order\": %s}"),
				i ? TEXT(", ") : TEXT(""), R.Id, *JsonString(R.Label), *Skin, *Core, *Solid, Order);
		}

		if (Request.Supports.Num() == 0)
		{
			OutError = TEXT("no support: give at least one plane whose nodes are fixed");
			Cleanup();
			return false;
		}
		FString SupportsJson;
		for (int32 i = 0; i < Request.Supports.Num(); ++i)
		{
			const FSupport& S = Request.Supports[i];
			const FString Value = S.bAtMin ? TEXT("\"min\"") : (S.bAtMax ? TEXT("\"max\"") : Num(S.Value * L));
			SupportsJson += FString::Printf(TEXT("%s{\"axis\": \"%c\", \"value\": %s, \"tolerance\": %s}"), i ? TEXT(", ") : TEXT(""), FChar::ToLower(S.Axis), *Value, *Num(S.Tolerance * L));
		}
		FString PointsJson;
		for (int32 i = 0; i < Request.PointForces.Num(); ++i)
		{
			const FPointForce& P = Request.PointForces[i];
			PointsJson += FString::Printf(TEXT("%s{\"at\": [%s, %s, %s], \"force\": [%s, %s, %s]}"), i ? TEXT(", ") : TEXT(""),
				*Num(P.At.x * L), *Num(P.At.y * L), *Num(P.At.z * L), *Num(P.Force.x), *Num(P.Force.y), *Num(P.Force.z));
		}
		const FString Job = FString::Printf(
			TEXT("{\n \"mesh\": \"mesh.bin\",\n \"faces\": %s,\n \"model\": \"regions\",\n \"regions\": [%s],\n \"second_order\": %s,\n")
			TEXT(" \"supports\": [%s],\n")
			TEXT(" \"loads\": {\"load_factor\": %s, \"pressure\": %s, \"pressure_file\": %s, \"point_forces\": [%s]},\n")
			TEXT(" \"solver\": {\"echo_level\": 0, \"linear_solver\": %s, \"homogenize_cache\": %s, \"shell_element\": %s, \"modes\": %d},\n")
			TEXT(" \"outputs\": {\"vtk\": %s, \"result\": \"result.json\"}\n}\n"),
			FacesFile.IsEmpty() ? TEXT("null") : *JsonString(FacesFile), *Regions, Request.bSecondOrder ? TEXT("true") : TEXT("false"),
			*SupportsJson,
			*Num(Request.LoadFactor), *Num(Request.Pressure), PressureFile.IsEmpty() ? TEXT("null") : *JsonString(PressureFile), *PointsJson,
			*JsonString(Request.LinearSolver.IsEmpty() ? FString(TEXT("auto")) : Request.LinearSolver), Request.bHomogenizeCache ? TEXT("true") : TEXT("false"),
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
		if (!AerotelicMeshLib::RunJob(Script, Dir, Result, OutError, Progress))
		{
			Cleanup();
			return false;
		}

		auto Int = [&Result](const TCHAR* Field) { return static_cast<int32>(NumberOr(Result, Field)); };
		auto Real = [&Result](const TCHAR* Field) { return NumberOr(Result, Field); };
		Out = FResult();
		Out.Nodes = Int(TEXT("nodes"));
		Out.Elements = Int(TEXT("elements"));
		Out.Tets = Int(TEXT("tets"));
		Out.Shells = Int(TEXT("shells"));
		Out.Dofs = Int(TEXT("dofs"));
		Out.SupportNodes = Int(TEXT("support_nodes"));
		Out.SupportCornerNodes = Result->HasField(TEXT("support_corner_nodes")) ? Int(TEXT("support_corner_nodes")) : Out.SupportNodes;
		Out.WettedFaces = Int(TEXT("wetted_faces"));
		Result->TryGetBoolField(TEXT("second_order"), Out.bSecondOrder);
		Out.Mass = Real(TEXT("mass"));
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
					E.MacroVonMises = NumberOr(*Obj, TEXT("macro_von_mises"));
					E.Margin = NumberOr(*Obj, TEXT("margin"));
					E.Allowable = NumberOr(*Obj, TEXT("allowable"));
					E.Mass = NumberOr(*Obj, TEXT("mass"));
					Out.Entries.Add(E);
				}
			}
		}
		const TArray<TSharedPtr<FJsonValue>>* Regions2 = nullptr;
		if (Result->TryGetArrayField(TEXT("regions"), Regions2))
		{
			for (const TSharedPtr<FJsonValue>& V : *Regions2)
			{
				const TSharedPtr<FJsonObject>* Obj = nullptr;
				if (!V.IsValid() || !V->TryGetObject(Obj)) continue;
				FRegionResult R;
				R.Id = static_cast<int32>(NumberOr(*Obj, TEXT("id")));
				(*Obj)->TryGetStringField(TEXT("label"), R.Label);
				R.RelativeDensity = NumberOr(*Obj, TEXT("relative_density"));
				ReadVec3(*Obj, TEXT("E"), R.E);
				ReadVec3(*Obj, TEXT("G"), R.G);
				R.NuXY = NumberOr(*Obj, TEXT("nu_xy"));
				R.NuYZ = NumberOr(*Obj, TEXT("nu_yz"));
				R.NuXZ = NumberOr(*Obj, TEXT("nu_xz"));
				R.Amplification = NumberOr(*Obj, TEXT("amplification"));
				R.HomogenizeSeconds = NumberOr(*Obj, TEXT("homogenize_seconds"));
				(*Obj)->TryGetBoolField(TEXT("homogenize_cached"), R.bHomogenizeCached);
				R.SkinMass = NumberOr(*Obj, TEXT("skin_mass"));
				R.CoreMass = NumberOr(*Obj, TEXT("core_mass"));
				R.CoreDensityUsed = NumberOr(*Obj, TEXT("core_density_used"));
				R.SkinMaxVonMises = NumberOr(*Obj, TEXT("skin_max_von_mises"));
				R.SkinMargin = NumberOr(*Obj, TEXT("skin_margin"));
				R.CoreMaxVonMises = NumberOr(*Obj, TEXT("core_max_von_mises"));
				R.CoreMaxMicroVonMises = NumberOr(*Obj, TEXT("core_max_micro_von_mises"));
				R.CoreMargin = NumberOr(*Obj, TEXT("core_margin"));
				Out.Regions.Add(R);
			}
		}
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
		Out.SecondsHomogenize = Real(TEXT("seconds_homogenize"));
		Out.Seconds = Real(TEXT("seconds"));
		Result->TryGetStringField(TEXT("vtk"), Out.VtkPath);
		if (Out.VtkPath.IsEmpty())
		{
			Out.VtkPath = Request.VtkPath;
		}
		Cleanup();
		return true;
	}
}
