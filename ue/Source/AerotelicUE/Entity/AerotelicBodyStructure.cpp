#include "AerotelicBodyStructure.h"

#include "AerotelicRibGmsh.h"

#include "aerotelic/mesh_cut.hpp"

#include <cmath>
#include <stdexcept>

namespace
{
	const FLinearColor kRibsColor(0.35f, 0.55f, 0.85f);     // steel blue
	const FLinearColor kSparsColor(0.85f, 0.55f, 0.25f);    // oak
	const FLinearColor kStringersColor(0.45f, 0.75f, 0.45f); // green

	// The cut's tolerance as a fraction of the body's diagonal: a vertex
	// within it of a plane is snapped onto the plane rather than split a hair
	// away from it (the same figure the structural model builder uses, so
	// the two cut the same surface the same way).
	constexpr double kCutToleranceOverDiag = 1e-7;

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

	// World -> the body's local frame: x = R^T (x' - T) / scale.
	void UnplaceMesh(aerotelic::TriMesh& Mesh, const std::array<double, 9>& R, const aerotelic::Vec3& T, double Scale)
	{
		const double S = Scale != 0.0 ? 1.0 / Scale : 1.0;
		const std::array<double, 9> Rt{{R[0] * S, R[3] * S, R[6] * S, R[1] * S, R[4] * S, R[7] * S, R[2] * S, R[5] * S, R[8] * S}};
		const aerotelic::Vec3 Shift{-(Rt[0] * T.x + Rt[1] * T.y + Rt[2] * T.z), -(Rt[3] * T.x + Rt[4] * T.y + Rt[5] * T.z), -(Rt[6] * T.x + Rt[7] * T.y + Rt[8] * T.z)};
		Mesh.Transform(Rt, Shift);
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

	// The faces of one rib id as a mesh of their own, over the nodes they
	// use alone (the mesher's node table is the surface's vertices plus every
	// rib's interior nodes). Open: a disc. Returns the summed triangle area.
	double CompactRib(const AerotelicRibGmsh::FResult& Result, int32 Id, aerotelic::TriMesh& Out)
	{
		Out.Clear();
		std::vector<int> Local(Result.Nodes.size(), -1);
		double Area = 0.0;
		for (const AerotelicRibGmsh::FRibFace& F : Result.RibFaces)
		{
			if (F.Id != Id) continue;
			std::array<int, 3> Tri;
			for (int k = 0; k < 3; ++k)
			{
				const int32 N = F.N[k];
				if (Local[N] < 0)
				{
					Local[N] = static_cast<int>(Out.vertices.size());
					Out.vertices.push_back(Result.Nodes[N]);
				}
				Tri[k] = Local[N];
			}
			Out.triangles.push_back(Tri);
			const aerotelic::Vec3& A = Result.Nodes[F.N[0]];
			const aerotelic::Vec3& B = Result.Nodes[F.N[1]];
			const aerotelic::Vec3& C = Result.Nodes[F.N[2]];
			Area += 0.5 * aerotelic::Length(aerotelic::Cross(B - A, C - A));
		}
		return Area;
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

bool FAerotelicBodyStructure::Generate(const FInputs& In, FString& OutError)
{
	const aerotelic::BodyStructure& S = In.Structure;
	const std::string Problem = aerotelic::BodyStructureProblem(S);
	if (!Problem.empty())
	{
		OutError = UTF8_TO_TCHAR(Problem.c_str());
		return false;
	}
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
		const double Diag = FMath::Max(In.Body.Bounds().Diagonal(), 1e-9);
		const double CutTol = kCutToleranceOverDiag * Diag;

		// The planes, placed with the body: every network's, in network order,
		// and the mesher's ribs over them (rib id = network index + 1, as the
		// structural model builder numbers them).
		std::vector<aerotelic::Plane> AllPlanes;
		TArray<AerotelicRibGmsh::FRequest::FRib> Ribs;
		for (int32 n = 0; n < static_cast<int32>(S.networks.size()); ++n)
		{
			const aerotelic::PlaneNetwork& Net = S.networks[n];
			for (int k = 0; k < Net.count; ++k)
			{
				const aerotelic::Plane P = aerotelic::PlacedPlane(aerotelic::NetworkPlane(Net, k), In.R, In.T, In.Scale);
				AllPlanes.push_back(P);
				AerotelicRibGmsh::FRequest::FRib R;
				R.Id = n + 1;
				R.Normal = P.normal;
				R.Offset = P.offset;
				R.Height = Net.height;
				Ribs.Add(R);
			}
		}

		// The body cut by every plane, so each plane's section is a loop of
		// the surface's own vertices -- the rim gmsh meshes the disc inside.
		aerotelic::TriMesh Cut;
		std::vector<aerotelic::TriMesh> CutReserved;
		AerotelicRibGmsh::FResult Result;
		if (!AllPlanes.empty())
		{
			aerotelic::MeshCutStats CutStats;
			Cut = aerotelic::CutMeshByPlanes(In.Body, AllPlanes, CutTol, nullptr, &CutStats);
			if (!Cut.IsClosedManifold())
			{
				OutError = FString::Printf(TEXT("%s: the surface cut by the %d rib plane(s) is not a closed manifold."), *In.Label, static_cast<int32>(AllPlanes.size()));
				return false;
			}
			UE_LOG(LogTemp, Log, TEXT("AerotelicBodyStructure: %s cut by %d plane(s): %d triangles split, %d vertices added, %d snapped"),
				*In.Label, static_cast<int32>(AllPlanes.size()), CutStats.split_triangles, CutStats.new_vertices, CutStats.snapped_vertices);
			// The reserved volumes by the same planes (the same tolerance, so
			// their rims lie on the planes as the body's do); one that is not
			// closed cannot bound a hole and is left out with a warning.
			for (size_t i = 0; i < In.Reserved.size(); ++i)
			{
				const aerotelic::TriMesh& Rv = In.Reserved[i];
				if (Rv.IsEmpty()) continue;
				if (!Rv.IsClosedManifold() || !(Rv.SignedVolume() > 0.0))
				{
					Warnings.Add(FString::Printf(TEXT("reserved volume %d is not a closed outward surface; the ribs do not avoid it"), static_cast<int32>(i) + 1));
					continue;
				}
				aerotelic::TriMesh RvCut = aerotelic::CutMeshByPlanes(Rv, AllPlanes, CutTol);
				if (!RvCut.IsClosedManifold())
				{
					Warnings.Add(FString::Printf(TEXT("reserved volume %d cut by the planes is not a closed manifold; the ribs do not avoid it"), static_cast<int32>(i) + 1));
					continue;
				}
				CutReserved.push_back(std::move(RvCut));
			}

			AerotelicRibGmsh::FRequest Request;
			Request.Outer = &Cut;
			Request.Ribs = Ribs;
			Request.RibSize = 0.0;    // the surface's median edge
			for (const aerotelic::TriMesh& H : CutReserved) Request.Holes.Add(&H);
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

		// The layers: the skin (the body's surface, no mesh of its own), then
		// one Ribs layer per network -- its faces compacted to a disc mesh --
		// every mesh brought back into the body's frame.
		std::vector<FAerotelicStructureLayer> NewLayers;
		{
			FAerotelicStructureLayer Skin;
			Skin.Name = TEXT("Skin");
			Skin.Kind = FAerotelicStructureLayer::EKind::Skin;
			Skin.Material = S.material;
			Skin.Thickness = S.skin_thickness;
			Skin.Area = In.Body.SurfaceArea();
			Skin.Volume = Skin.Area * Skin.Thickness;
			NewLayers.push_back(std::move(Skin));
		}
		double RibsArea = 0.0;
		for (int32 n = 0; n < static_cast<int32>(S.networks.size()); ++n)
		{
			FAerotelicStructureLayer Layer;
			Layer.Name = UniqueLayerName(UTF8_TO_TCHAR(S.networks[n].name.c_str()), NewLayers);
			Layer.Kind = FAerotelicStructureLayer::EKind::Ribs;
			Layer.NetworkIndex = n;
			Layer.Material = S.networks[n].material;
			Layer.Thickness = S.networks[n].thickness;
			Layer.Area = CompactRib(Result, n + 1, Layer.Mesh);
			Layer.Volume = Layer.Area * Layer.Thickness;
			RibsArea += Layer.Area;
			// The mesher's notes on this network's ribs ("rib <id>: ...").
			const FString Prefix = FString::Printf(TEXT("rib %d:"), n + 1);
			TArray<FString> Notes;
			for (const FString& N : Result.Notes)
			{
				if (N.StartsWith(Prefix)) Notes.Add(N);
			}
			Layer.Note = FString::Join(Notes, TEXT("\n"));
			if (Layer.Mesh.IsEmpty() && Layer.Note.IsEmpty()) Layer.Note = TEXT("the planes miss the interior");
			UnplaceMesh(Layer.Mesh, In.R, In.T, In.Scale);
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
		UE_LOG(LogTemp, Log, TEXT("AerotelicBodyStructure: %s: %d layer(s), %d rib plane(s), %lld rib triangles, %d hole(s), skin area %.5g, ribs area %.5g, %.2f s (gmsh %.2f s)"),
			*In.Label, static_cast<int32>(Layers.size()), Ribs.Num(), Stats.Triangles, Result.Holes, Stats.SkinArea, Stats.RibsArea, Stats.Seconds, Result.Seconds);
		return true;
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return false;
	}
}
