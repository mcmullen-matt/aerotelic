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

		// The mesh for the script, already in metres: counts, then nodes as
		// float64 xyz, tetrahedra as int32 quads, material as int32 -- the
		// layout tools/tet_gmsh.py's write_tet uses, without its magic.
		bool WriteMesh(const aerotelic::TetMesh& Mesh, double Scale, const FString& Path)
		{
			TArray<uint8> Bytes;
			const uint32 Nn = static_cast<uint32>(Mesh.nodes.size());
			const uint32 Nt = static_cast<uint32>(Mesh.tets.size());
			Bytes.Append(reinterpret_cast<const uint8*>(&Nn), sizeof(Nn));
			Bytes.Append(reinterpret_cast<const uint8*>(&Nt), sizeof(Nt));
			Bytes.Reserve(8 + Nn * 24 + Nt * 20);
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
	}

	FString FindScript()
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("structural_fea.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError)
	{
		if (Request.Mesh == nullptr || Request.Mesh->tets.empty())
		{
			OutError = TEXT("no tetrahedra to analyse");
			return false;
		}
		if (Request.Materials.Num() == 0)
		{
			OutError = TEXT("no material for the tetrahedra");
			return false;
		}
		for (const FMaterialRegion& M : Request.Materials)
		{
			if (!M.Material.IsStructural())
			{
				OutError = FString::Printf(TEXT("'%s' has no structural material: give it a density, a Young's modulus and a Poisson's ratio under Material Properties in the Scene tree."), *M.Label);
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
			OutError = TEXT("The Python interpreter was not found: install Kratos (pip install KratosMultiphysics KratosStructuralMechanicsApplication KratosLinearSolversApplication, all at the same version) in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has them.");
			return false;
		}

		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_fea_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		auto Cleanup = [&Dir]() { IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		if (!WriteMesh(*Request.Mesh, Request.LengthToSI, FPaths::Combine(Dir, TEXT("mesh.bin"))))
		{
			OutError = TEXT("could not write the mesh for the solver");
			Cleanup();
			return false;
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

		FString MaterialsJson;
		for (int32 i = 0; i < Request.Materials.Num(); ++i)
		{
			const FMaterialRegion& M = Request.Materials[i];
			MaterialsJson += FString::Printf(TEXT("%s{\"id\": %d, \"label\": %s, \"name\": %s, \"density\": %s, \"youngs_modulus\": %s, \"poisson_ratio\": %s, \"allowable_stress\": %s}"),
				i ? TEXT(", ") : TEXT(""), M.Id, *JsonString(M.Label), *JsonString(UTF8_TO_TCHAR(M.Material.name.c_str())),
				*Num(M.Material.density), *Num(M.Material.youngs_modulus), *Num(M.Material.poisson_ratio), *Num(M.Material.allowable_stress));
		}
		const double L = Request.LengthToSI;
		const FString SupportValue = Request.bSupportAtMin ? TEXT("\"min\"") : (Request.bSupportAtMax ? TEXT("\"max\"") : Num(Request.SupportValue * L));
		const FString PointJson = Request.bPointForce
			? FString::Printf(TEXT("{\"at\": [%s, %s, %s], \"force\": [%s, %s, %s]}"),
				*Num(Request.PointAt.x * L), *Num(Request.PointAt.y * L), *Num(Request.PointAt.z * L),
				*Num(Request.PointForce.x), *Num(Request.PointForce.y), *Num(Request.PointForce.z))
			: TEXT("null");
		const FString Job = FString::Printf(
			TEXT("{\n \"mesh\": \"mesh.bin\",\n")
			TEXT(" \"materials\": [%s],\n")
			TEXT(" \"second_order\": %s,\n")
			TEXT(" \"support\": {\"axis\": \"%c\", \"value\": %s, \"tolerance\": %s},\n")
			TEXT(" \"loads\": {\"load_factor\": %s, \"pressure\": %s, \"pressure_file\": %s, \"point_force\": %s},\n")
			TEXT(" \"solver\": {\"echo_level\": 0},\n")
			TEXT(" \"outputs\": {\"vtk\": %s, \"result\": \"result.json\"}\n}\n"),
			*MaterialsJson,
			Request.bSecondOrder ? TEXT("true") : TEXT("false"),
			FChar::ToLower(Request.SupportAxis), *SupportValue, *Num(Request.SupportTolerance * L),
			*Num(Request.LoadFactor), *Num(Request.Pressure), PressureFile.IsEmpty() ? TEXT("null") : *JsonString(PressureFile), *PointJson,
			*JsonString(FPaths::ConvertRelativePathToFull(Request.VtkPath)));
		if (!FFileHelper::SaveStringToFile(Job, *FPaths::Combine(Dir, TEXT("job.json")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			OutError = TEXT("could not write the solver's job file");
			Cleanup();
			return false;
		}

		TSharedPtr<FJsonObject> Result;
		if (!AerotelicMeshLib::RunJob(Script, Dir, Result, OutError))
		{
			Cleanup();
			return false;
		}

		auto Int = [&Result](const TCHAR* Field) { return static_cast<int32>(Result->GetNumberField(Field)); };
		auto Real = [&Result](const TCHAR* Field) { return Result->GetNumberField(Field); };
		Out.Nodes = Int(TEXT("nodes"));
		Out.Elements = Int(TEXT("elements"));
		Out.Dofs = Int(TEXT("dofs"));
		Out.SupportNodes = Int(TEXT("support_nodes"));
		Out.WettedFaces = Int(TEXT("wetted_faces"));
		Out.bSecondOrder = Result->GetBoolField(TEXT("second_order"));
		Out.Mass = Real(TEXT("mass"));
		Out.MaxDisplacement = Real(TEXT("max_displacement"));
		ReadVec3(Result, TEXT("max_displacement_at"), Out.MaxDisplacementAt);
		Out.MaxVonMises = Real(TEXT("max_von_mises"));
		Out.MaxVonMisesMaterial = Int(TEXT("max_von_mises_material"));
		const TArray<TSharedPtr<FJsonValue>>* PerMaterial = nullptr;
		if (Result->TryGetArrayField(TEXT("per_material"), PerMaterial))
		{
			for (const TSharedPtr<FJsonValue>& V : *PerMaterial)
			{
				const TSharedPtr<FJsonObject>* Obj = nullptr;
				if (V.IsValid() && V->TryGetObject(Obj))
				{
					Out.MaxVonMisesByMaterial.Add((*Obj)->GetNumberField(TEXT("max_von_mises")));
					Out.MarginByMaterial.Add((*Obj)->GetNumberField(TEXT("margin")));
				}
			}
		}
		ReadVec3(Result, TEXT("reaction"), Out.Reaction);
		ReadVec3(Result, TEXT("applied_gravity"), Out.AppliedGravity);
		ReadVec3(Result, TEXT("applied_pressure"), Out.AppliedPressure);
		ReadVec3(Result, TEXT("applied_point"), Out.AppliedPoint);
		Out.MeanNormalDisplacement = Real(TEXT("mean_normal_displacement"));
		Out.SecondsSolve = Real(TEXT("seconds_solve"));
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
