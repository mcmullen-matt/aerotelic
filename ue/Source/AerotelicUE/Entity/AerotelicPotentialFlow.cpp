#include "AerotelicPotentialFlow.h"
#include "AerotelicMeshLib.h"

#include "HAL/PlatformProcess.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace AerotelicPotentialFlow
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
	}

	FString FindScript()
	{
		const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("potential_flow.py"));
		return FPaths::FileExists(Script) ? FPaths::ConvertRelativePathToFull(Script) : FString();
	}

	bool Run(const FRequest& Request, FResult& Out, FString& OutError)
	{
		if (Request.Bodies.Num() == 0)
		{
			OutError = TEXT("no body to compute the flow around");
			return false;
		}
		if (!(Request.Speed > 0.0))
		{
			OutError = TEXT("the free-stream speed must be positive");
			return false;
		}
		if (Request.VtkPath.IsEmpty())
		{
			OutError = TEXT("no solution file to write");
			return false;
		}
		const FString Script = FindScript();
		if (Script.IsEmpty())
		{
			OutError = TEXT("tools\\potential_flow.py was not found beside the application.");
			return false;
		}
		if (AerotelicMeshLib::FindPython().IsEmpty())
		{
			OutError = TEXT("The Python interpreter was not found: install Kratos (pip install KratosMultiphysics KratosFluidDynamicsApplication KratosCompressiblePotentialFlowApplication, all at the same version) and gmsh in the WingDesigner micromamba environment, or point AEROTELIC_MESHLIB_PYTHON at a python.exe that has them.");
			return false;
		}

		const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_flow_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
		IFileManager::Get().MakeDirectory(*Dir, true);
		auto Cleanup = [&Dir]() { IFileManager::Get().DeleteDirectory(*Dir, false, true); };

		FString BodiesJson;
		for (int32 i = 0; i < Request.Bodies.Num(); ++i)
		{
			const FBody& Body = Request.Bodies[i];
			if (Body.Mesh == nullptr || Body.Mesh->IsEmpty())
			{
				OutError = FString::Printf(TEXT("'%s' has no triangles."), *Body.Label);
				Cleanup();
				return false;
			}
			const FString File = FString::Printf(TEXT("body%d.tri"), i);
			if (!AerotelicMeshLib::WriteTri(*Body.Mesh, FPaths::Combine(Dir, File)))
			{
				OutError = FString::Printf(TEXT("could not write '%s' for the solver"), *Body.Label);
				Cleanup();
				return false;
			}
			BodiesJson += FString::Printf(TEXT("%s{\"label\": %s, \"file\": \"%s\"}"),
				i ? TEXT(", ") : TEXT(""), *JsonString(Body.Label), *File);
		}

		const FString Job = FString::Printf(
			TEXT("{\n \"bodies\": [%s],\n")
			TEXT(" \"far_field\": {\"size_factor\": %s, \"element_size\": %s},\n")
			TEXT(" \"volume\": {\"near_size\": %s, \"near_distance\": %s, \"growth_rate\": %s, \"algorithm\": 1, \"optimize\": true, \"optimize_netgen\": false},\n")
			TEXT(" \"flow\": {\"speed\": %s, \"angle_of_attack_deg\": %s, \"speed_of_sound\": %s, \"density\": %s, \"element_type\": %s},\n")
			TEXT(" \"solver\": {\"echo_level\": 0},\n")
			TEXT(" \"outputs\": {\"vtk\": %s, \"result\": \"result.json\"}\n}\n"),
			*BodiesJson,
			*Num(Request.FarFieldFactor), *Num(Request.FarElementSize),
			*Num(Request.NearElementSize), *Num(Request.NearDistance), *Num(Request.GrowthRate),
			*Num(Request.Speed), *Num(Request.AngleOfAttackDeg), *Num(Request.SpeedOfSound), *Num(Request.Density),
			*JsonString(Request.ElementType),
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
		Out.Nodes = Int(TEXT("mesh_nodes"));
		Out.Tets = Int(TEXT("mesh_tets"));
		Out.WettedTriangles = Int(TEXT("wetted_triangles"));
		Out.FarFieldTriangles = Int(TEXT("far_field_triangles"));
		Out.BodyVolume = Real(TEXT("body_volume"));
		Out.WettedVolume = Real(TEXT("wetted_volume"));
		Out.FluidVolume = Real(TEXT("fluid_volume"));
		Out.BoxVolume = Real(TEXT("box_volume"));
		Out.FarFieldSize = Real(TEXT("far_field_size"));
		Out.NearElementSize = Real(TEXT("near_element_size"));
		Out.FarElementSize = Real(TEXT("far_field_element_size"));
		Out.MinQuality = Real(TEXT("min_quality"));
		Out.MeanQuality = Real(TEXT("mean_quality"));
		Out.PoorQuality = Int(TEXT("poor_quality"));
		Out.FreeStreamSpeed = Real(TEXT("free_stream_speed"));
		Out.MaxSpeed = Real(TEXT("max_speed"));
		Out.MinCp = Real(TEXT("min_cp"));
		Out.MaxCp = Real(TEXT("max_cp"));
		Out.MinCpBody = Real(TEXT("min_cp_body"));
		Out.MaxCpBody = Real(TEXT("max_cp_body"));
		Out.ForceCoefficient = Real(TEXT("force_coefficient"));
		Out.FrontalArea = Real(TEXT("frontal_area"));
		const TArray<TSharedPtr<FJsonValue>>* Force = nullptr;
		if (Result->TryGetArrayField(TEXT("force"), Force) && Force->Num() == 3)
		{
			for (int32 k = 0; k < 3; ++k)
			{
				double V = 0.0;
				if ((*Force)[k].IsValid() && (*Force)[k]->TryGetNumber(V)) Out.Force[k] = V;
			}
		}
		Out.SecondsMesh = Real(TEXT("seconds_mesh"));
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
