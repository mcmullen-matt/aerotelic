#include "AerotelicBodyStructure.h"

#include "AerotelicMeshLib.h"

#include <cmath>
#include <stdexcept>

namespace
{
	// The offset's distance field is kept under about this many voxels
	// across the body's bounds (sparse, so the real count is far lower).
	constexpr double kMaxOffsetVoxels = 60.0e6;
	// The offset surface is remeshed to isotropic triangles of this many
	// skin thicknesses (see the history in the lattice model this replaced:
	// two skins is clean and meshes in seconds, one skin is not) ...
	constexpr double kInnerRemeshEdgeOverSkin = 2.0;
	// ... and never finer than this many offset voxels.
	constexpr double kInnerRemeshEdgeOverVoxel = 2.0;
	// The slabs reach this far from the body's centre in the plane: the
	// bounding sphere's radius with margin.
	constexpr double kSlabHalfExtentOverDiag = 0.75;

	const FLinearColor kRibsColor(0.35f, 0.55f, 0.85f);     // steel blue

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

	// A key for the offset cache: the body's bytes, the skin and the voxel.
	uint64 InnerCacheKey(const aerotelic::TriMesh& Body, double Skin, double Voxel)
	{
		uint32 A = FCrc::MemCrc32(Body.vertices.data(), static_cast<int32>(Body.vertices.size() * sizeof(aerotelic::Vec3)));
		A = FCrc::MemCrc32(Body.triangles.data(), static_cast<int32>(Body.triangles.size() * sizeof(std::array<int, 3>)), A);
		uint32 B = FCrc::MemCrc32(&Skin, sizeof(Skin));
		B = FCrc::MemCrc32(&Voxel, sizeof(Voxel), B);
		return (static_cast<uint64>(A) << 32) | B;
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

double FAerotelicBodyStructure::BodyTolerance(const aerotelic::BodyStructure& S)
{
	// The body mesh is the skin's outer surface as it stands, and the
	// offset's distance field sees it to a fraction of its voxel, so it is
	// built a few voxels finer than the skin.
	const double Tolerance = S.skin_thickness > 0.0 ? 0.25 * S.skin_thickness : 1e-3;
	return FMath::Max(Tolerance, 1e-5);
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
		for (const FReserved& Rv : In.Reserved)
		{
			RequireClosedBody(Rv.Mesh, Rv.Label, TEXT("a reserved volume"));
		}
		const aerotelic::Box3 Bounds = In.Body.Bounds();
		const double Diag = FMath::Max(Bounds.Diagonal(), 1e-9);

		// The job. The offset's voxel is a quarter of the skin (so the
		// shell's thickness is resolved), capped by a voxel budget across
		// the bounds.
		AerotelicMeshLib::FRequest Request;
		{
			double Voxel = 0.25 * S.skin_thickness;
			const aerotelic::Vec3 Ext = Bounds.Extent() + aerotelic::Vec3{2.0, 2.0, 2.0} * S.skin_thickness;
			const double Volume = Ext.x * Ext.y * Ext.z;
			if (Volume / (Voxel * Voxel * Voxel) > kMaxOffsetVoxels)
			{
				Voxel = std::cbrt(Volume / kMaxOffsetVoxels);
			}
			Request.OffsetVoxel = FMath::Max(Voxel, 1e-6);
		}
		const double RemeshEdge = FMath::Max(kInnerRemeshEdgeOverSkin * S.skin_thickness, kInnerRemeshEdgeOverVoxel * Request.OffsetVoxel);
		const uint64 Key = InnerCacheKey(In.Body, S.skin_thickness, Request.OffsetVoxel);
		AerotelicMeshLib::FBody Target;
		Target.Label = In.Label;
		Target.Mesh = &In.Body;
		Target.SkinThickness = S.skin_thickness;
		Target.InnerRemeshEdge = RemeshEdge;
		// The offset from the last run of the same body at the same skin
		// is handed back: the distance field is the slow step, and editing
		// the planes does not change it.
		if (Key == InnerKey && !Inner.IsEmpty())
		{
			Target.PrecomputedInner = &Inner;
		}
		Request.Targets.Add(Target);
		for (const FReserved& Rv : In.Reserved)
		{
			AerotelicMeshLib::FBody Volume;
			Volume.Label = Rv.Label;
			Volume.Mesh = &Rv.Mesh;
			Request.Reserved.Add(Volume);
		}
		Request.SkinThickness = S.skin_thickness;
		Request.OffsetMode = TEXT("standard");
		Request.DecimateTolerance = 0.2 * Request.OffsetVoxel;
		Request.MinPocketVolume = FMath::Pow(4.0 * Request.OffsetVoxel, 3.0);
		Request.InnerOpening = 0.0;
		Request.InnerRemeshEdge = RemeshEdge;
		// The networks: each plane placed with the body (the normal rotated,
		// the offset scaled and carried), the thickness and height as typed.
		for (int32 n = 0; n < static_cast<int32>(S.networks.size()); ++n)
		{
			const aerotelic::PlaneNetwork& Net = S.networks[n];
			AerotelicMeshLib::FNetwork N;
			N.Label = UTF8_TO_TCHAR(Net.name.c_str());
			for (int k = 0; k < Net.count; ++k)
			{
				N.Planes.Add(aerotelic::PlacedPlane(aerotelic::NetworkPlane(Net, k), In.R, In.T, In.Scale));
			}
			N.Thickness = Net.thickness;
			N.Height = Net.height;
			N.Centre = Bounds.Center();
			N.HalfExtent = kSlabHalfExtentOverDiag * Diag;
			Request.Networks.Add(N);
		}

		AerotelicMeshLib::FResult Result;
		FString Error;
		if (!AerotelicMeshLib::Run(Request, Result, Error))
		{
			OutError = Error;
			return false;
		}
		if (Result.Skin.IsEmpty())
		{
			OutError = FString::Printf(TEXT("%s: the skin came out empty."), *In.Label);
			return false;
		}
		for (const FString& W : Result.Warnings)
		{
			UE_LOG(LogTemp, Warning, TEXT("AerotelicBodyStructure: %s: %s"), *In.Label, *W);
		}

		// Take the results: the layers in the body's frame, the hollowing's
		// surfaces as they are for the structural analysis.
		std::vector<FAerotelicStructureLayer> NewLayers;
		{
			FAerotelicStructureLayer Skin;
			Skin.Name = TEXT("Skin");
			Skin.Kind = FAerotelicStructureLayer::EKind::Skin;
			Skin.Mesh = std::move(Result.Skin);
			Skin.Material = S.material;
			Skin.Thickness = S.skin_thickness;
			Skin.Volume = Result.SkinVolume;
			NewLayers.push_back(std::move(Skin));
		}
		for (int32 n = 0; n < Result.Ribs.Num() && n < static_cast<int32>(S.networks.size()); ++n)
		{
			FAerotelicStructureLayer Ribs;
			Ribs.Name = UniqueLayerName(UTF8_TO_TCHAR(S.networks[n].name.c_str()), NewLayers);
			Ribs.Kind = FAerotelicStructureLayer::EKind::Ribs;
			Ribs.NetworkIndex = n;
			Ribs.Mesh = std::move(Result.Ribs[n].Mesh);
			Ribs.Material = S.networks[n].material;
			Ribs.Thickness = S.networks[n].thickness;
			Ribs.Volume = Result.Ribs[n].Volume;
			Ribs.Note = Result.Ribs[n].Note;
			if (Ribs.Mesh.IsEmpty() && Ribs.Note.IsEmpty()) Ribs.Note = TEXT("the planes miss the interior");
			NewLayers.push_back(std::move(Ribs));
		}
		for (FAerotelicStructureLayer& L : NewLayers)
		{
			if (!L.Mesh.IsEmpty() && !L.Mesh.IsClosedManifold())
			{
				UE_LOG(LogTemp, Warning, TEXT("AerotelicBodyStructure: %s: the %s layer is not a closed manifold"), *In.Label, *L.Name);
			}
			UnplaceMesh(L.Mesh, In.R, In.T, In.Scale);
		}

		Layers = std::move(NewLayers);
		Generated = S;
		Outer = In.Body;
		Inner = std::move(Result.Inners[0].Mesh);
		bInnerUsable = Result.Inners[0].bUsable;
		InnerNote = Result.Inners[0].Note;
		InnerKey = Key;
		SkinThickness = S.skin_thickness;

		Stats = FStats();
		for (const FAerotelicStructureLayer& L : Layers) Stats.Triangles += static_cast<int64>(L.Mesh.triangles.size());
		Stats.SkinVolume = Result.SkinVolume;
		Stats.DomainVolume = Result.DomainVolume;
		Stats.RibsVolume = Result.RibsVolume;
		Stats.Warnings = Result.Warnings;
		Stats.Seconds = FPlatformTime::Seconds() - T0;
		bGenerated = true;
		bStale = false;
		UE_LOG(LogTemp, Log, TEXT("AerotelicBodyStructure: %s: %d layer(s), %lld triangles, offset voxel %.4g, skin %.5g, interior %.5g, ribs %.5g, %.2f s (MeshLib %.2f s)"),
			*In.Label, static_cast<int32>(Layers.size()), Stats.Triangles, Request.OffsetVoxel, Stats.SkinVolume, Stats.DomainVolume, Stats.RibsVolume,
			Stats.Seconds, Result.Seconds);
		return true;
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return false;
	}
}
