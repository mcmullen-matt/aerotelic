#include "AerotelicBodyStructure.h"

#include "AerotelicRibGmsh.h"

#include "aerotelic/mesh_cut.hpp"

#include <cmath>
#include <stdexcept>

namespace
{
	const FLinearColor kRibsColor(0.35f, 0.55f, 0.85f);     // steel blue

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
	case EKind::Skin:
	default: return BodyColor;
	}
}

const TCHAR* FAerotelicStructureLayer::KindName(EKind Kind)
{
	switch (Kind)
	{
	case EKind::Ribs: return TEXT("ribs");
	case EKind::Skin:
	default: return TEXT("skin");
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
