#include "AerotelicBodyStructure.h"

#include "AerotelicRibGmsh.h"

#include <cmath>
#include <stdexcept>

namespace
{
	const FLinearColor kRibsColor(0.35f, 0.55f, 0.85f);     // steel blue
	const FLinearColor kSparsColor(0.85f, 0.55f, 0.25f);    // oak
	const FLinearColor kStringersColor(0.45f, 0.75f, 0.45f); // green

	void RequireClosedBody(const aerotelic::TriMesh& Mesh, const FString& Label, const TCHAR* Role)
	{
		if (!Mesh.IsClosedManifold())
		{
			throw std::runtime_error(TCHAR_TO_UTF8(*FString::Printf(
				TEXT("'%s' (%s) is not a closed surface: its mesh has edges with a triangle on one side only, so inside and outside cannot be told apart. Rebuild or repair that body first."),
				*Label, Role)));
		}
		if (!(Mesh.SignedVolume() > 0.0))
		{
			throw std::runtime_error(TCHAR_TO_UTF8(*FString::Printf(
				TEXT("'%s' (%s) is turned inside out (its enclosed volume is not positive), so inside and outside cannot be told apart."),
				*Label, Role)));
		}
	}

	FString UniqueLayerName(const FString& Wanted, const std::vector<FAerotelicStructureLayer>& Existing)
	{
		FString Name = Wanted.IsEmpty() ? FString(TEXT("Ribs")) : Wanted;
		int32 n = 2;
		auto Taken = [&Existing](const FString& N) { for (const auto& L : Existing) if (L.Name == N) return true; return false; };
		while (Taken(Name))
		{
			Name = FString::Printf(TEXT("%s (%d)"), *Wanted, n++);
		}
		return Name;
	}

}

FLinearColor FAerotelicStructureLayer::DefaultColor(EKind Kind, const FLinearColor& BodyColor)
{
	switch (Kind)
	{
	case EKind::Ribs: return kRibsColor;
	case EKind::Spars: return kSparsColor;
	case EKind::Stringers: return kStringersColor;
	case EKind::Skin:
	default: return BodyColor;
	}
}

const TCHAR* FAerotelicStructureLayer::KindName(EKind Kind)
{
	switch (Kind)
	{
	case EKind::Ribs: return TEXT("ribs");
	case EKind::Spars: return TEXT("spars");
	case EKind::Stringers: return TEXT("stringers");
	case EKind::Skin:
	default: return TEXT("skin");
	}
}

bool FAerotelicBodyStructure::GenerateFromPlan(const FPlanInputs& In, FString& OutError)
{
	const aerotelic::BodyStructure& S = In.Structure;
	if (!(S.skin_thickness > 0.0))
	{
		OutError = FString::Printf(TEXT("%s is a solid (no skin thickness): there is no interior structure to generate."), *In.Label);
		return false;
	}
	const double T0 = FPlatformTime::Seconds();
	try
	{
		RequireClosedBody(In.Body, In.Label, TEXT("the body"));
		TArray<FString> Warnings;
		for (const std::string& N : In.Plan.notes) Warnings.Add(UTF8_TO_TCHAR(N.c_str()));
		const double Area = In.Scale * In.Scale;
		auto Resolve = [&S](const aerotelic::Material& M) { return M.HasDensity() ? M : S.material; };

		// The ribs: discs on the plan's rims, the spar chords across them.
		AerotelicRibGmsh::FResult Result;
		if (!In.Plan.ribs.empty())
		{
			AerotelicRibGmsh::FRequest Request;
			Request.Outer = &In.Body;
			Request.RibSize = In.Plan.rib_size;
			for (size_t i = 0; i < In.Plan.ribs.size(); ++i)
			{
				const aerotelic::WingRibPlan& Rib = In.Plan.ribs[i];
				AerotelicRibGmsh::FRequest::FRib R;
				R.Id = static_cast<int32>(i) + 1;
				for (int V : Rib.rim) R.Rim.Add(V);
				for (const auto& C : Rib.chords) R.Chords.Add(FIntVector(C[0], C[1], C[2]));
				Request.Ribs.Add(R);
			}
			FString Error;
			if (!AerotelicRibGmsh::Run(Request, Result, Error))
			{
				OutError = FString::Printf(TEXT("%s: the ribs could not be meshed: %s"), *In.Label, *Error);
				return false;
			}
			for (const FString& N : Result.Notes)
			{
				UE_LOG(LogTemp, Warning, TEXT("AerotelicBodyStructure: %s: %s"), *In.Label, *N);
				Warnings.Add(N);
			}
		}

		std::vector<FAerotelicStructureLayer> NewLayers;
		{
			FAerotelicStructureLayer Skin;
			Skin.Name = TEXT("Skin");
			Skin.Kind = FAerotelicStructureLayer::EKind::Skin;
			Skin.Material = S.material;
			Skin.Thickness = S.skin_thickness;
			Skin.Area = In.Body.SurfaceArea() * Area;
			Skin.Volume = Skin.Area * Skin.Thickness;
			NewLayers.push_back(std::move(Skin));
		}
		double RibsArea = 0.0;
		if (!In.Plan.ribs.empty())
		{
			FAerotelicStructureLayer Layer;
			Layer.Name = UniqueLayerName(TEXT("Ribs"), NewLayers);
			Layer.Kind = FAerotelicStructureLayer::EKind::Ribs;
			Layer.Material = Resolve(In.Plan.rib_material);
			Layer.Thickness = In.Plan.rib_thickness;
			// Every rib's faces as one mesh over the nodes they use.
			double LocalArea = 0.0;
			std::vector<int> Local(Result.Nodes.size(), -1);
			for (const AerotelicRibGmsh::FRibFace& F : Result.RibFaces)
			{
				std::array<int, 3> Tri;
				for (int k = 0; k < 3; ++k)
				{
					const int32 N = F.N[k];
					if (Local[N] < 0)
					{
						Local[N] = static_cast<int>(Layer.Mesh.vertices.size());
						Layer.Mesh.vertices.push_back(Result.Nodes[N]);
					}
					Tri[k] = Local[N];
				}
				Layer.Mesh.triangles.push_back(Tri);
				const aerotelic::Vec3& A = Result.Nodes[F.N[0]];
				const aerotelic::Vec3& B = Result.Nodes[F.N[1]];
				const aerotelic::Vec3& C = Result.Nodes[F.N[2]];
				LocalArea += 0.5 * aerotelic::Length(aerotelic::Cross(B - A, C - A));
			}
			Layer.Area = LocalArea * Area;
			Layer.Volume = Layer.Area * Layer.Thickness;
			RibsArea += Layer.Area;
			TArray<FString> Names;
			for (const aerotelic::WingRibPlan& Rib : In.Plan.ribs) Names.Add(FString::Printf(TEXT("%s at %.3f"), UTF8_TO_TCHAR(Rib.name.c_str()), Rib.eta));
			Layer.Note = FString::Printf(TEXT("%d rib(s) in the section planes: %s"), static_cast<int32>(In.Plan.ribs.size()), *FString::Join(Names, TEXT(", ")));
			if (Layer.Mesh.IsEmpty()) Layer.Note = TEXT("the rib mesher made no disc");
			NewLayers.push_back(std::move(Layer));
		}
		for (const aerotelic::WingShellPlan& Sp : In.Plan.spars)
		{
			FAerotelicStructureLayer Layer;
			Layer.Name = UniqueLayerName(UTF8_TO_TCHAR(Sp.name.c_str()), NewLayers);
			Layer.Kind = FAerotelicStructureLayer::EKind::Spars;
			Layer.Material = Resolve(Sp.material);
			Layer.Thickness = Sp.thickness;
			Layer.Mesh = Sp.mesh;
			Layer.Area = Sp.mesh.SurfaceArea() * Area;
			Layer.Volume = Layer.Area * Layer.Thickness;
			RibsArea += Layer.Area;
			Layer.Note = TEXT("spar web between the upper and lower skin");
			NewLayers.push_back(std::move(Layer));
		}
		for (const aerotelic::WingShellPlan& St : In.Plan.stringers)
		{
			FAerotelicStructureLayer Layer;
			Layer.Name = UniqueLayerName(UTF8_TO_TCHAR(St.name.c_str()), NewLayers);
			Layer.Kind = FAerotelicStructureLayer::EKind::Stringers;
			Layer.Material = Resolve(St.material);
			Layer.Thickness = St.thickness;
			Layer.Mesh = St.mesh;
			Layer.Area = St.mesh.SurfaceArea() * Area;
			Layer.Volume = Layer.Area * Layer.Thickness;
			RibsArea += Layer.Area;
			Layer.Note = TEXT("stringer standing in from the skin");
			NewLayers.push_back(std::move(Layer));
		}

		Layers = std::move(NewLayers);
		Generated = S;
		Stats = FStats();
		for (const FAerotelicStructureLayer& L : Layers) Stats.Triangles += static_cast<int64>(L.Mesh.triangles.size());
		Stats.SkinArea = Layers[0].Area;
		Stats.RibsArea = RibsArea;
		Stats.Warnings = Warnings;
		Stats.Seconds = FPlatformTime::Seconds() - T0;
		bGenerated = true;
		bStale = false;
		UE_LOG(LogTemp, Log, TEXT("AerotelicBodyStructure: %s: %d layer(s) from the wing's plan: %d rib(s), %d spar(s), %d stringer(s), %lld structure triangles, skin area %.5g, structure area %.5g, %.2f s (gmsh %.2f s)"),
			*In.Label, static_cast<int32>(Layers.size()), static_cast<int32>(In.Plan.ribs.size()), static_cast<int32>(In.Plan.spars.size()), static_cast<int32>(In.Plan.stringers.size()),
			Stats.Triangles, Stats.SkinArea, Stats.RibsArea, Stats.Seconds, Result.Seconds);
		return true;
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return false;
	}
}

const FAerotelicStructureLayer* FAerotelicBodyStructure::FindLayer(const FString& Name) const
{
	for (const FAerotelicStructureLayer& L : Layers)
	{
		if (L.Name == Name) return &L;
	}
	return nullptr;
}

