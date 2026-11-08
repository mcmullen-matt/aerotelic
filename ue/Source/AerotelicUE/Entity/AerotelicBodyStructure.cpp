#include "AerotelicBodyStructure.h"

#include "AerotelicMeshLib.h"

#include <cmath>
#include <stdexcept>

#include "aerotelic/parallel.hpp"

namespace
{
	// The lattice field grid is refused beyond this many samples (as the
	// core's GenerateLattice).
	constexpr size_t kMaxGridNodes = 150000000;
	// The offset's distance field is kept under about this many voxels
	// across the body's bounds (sparse, so the real count is far lower).
	constexpr double kMaxOffsetVoxels = 60.0e6;
	// The offset surface is remeshed to isotropic triangles of this many
	// skin thicknesses (see the history in the lattice model this replaces:
	// two skins is clean and meshes in seconds, one skin is not) ...
	constexpr double kInnerRemeshEdgeOverSkin = 2.0;
	// ... and never finer than this many offset voxels.
	constexpr double kInnerRemeshEdgeOverVoxel = 2.0;
	// The core's target edge, as a multiple of the infill cell: the core
	// carries the cell's homogenised card, so it needs no finer a mesh.
	constexpr double kCoreSizeOverCell = 1.0;
	// gmsh's Mesh.Algorithm3D: Delaunay beat HXT on every body tried.
	constexpr int32 kGmshAlgorithm = 1;
	// The slabs reach this far from the body's centre in the plane: the
	// bounding sphere's radius with margin.
	constexpr double kSlabHalfExtentOverDiag = 0.75;

	const FLinearColor kInfillColor(0.95f, 0.65f, 0.15f);   // amber
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

	// The lattice field of the spec sampled on the grid the core would plan
	// over `Bounds`, F <= 0 inside the material.
	AerotelicMeshLib::FField SampleLatticeField(const aerotelic::LatticeSpec& Spec, const aerotelic::Box3& Bounds)
	{
		aerotelic::LatticeSpec Sampled = Spec;
		Sampled.skin_thickness = 0.0; // the skin is MeshLib's, not the sampled field's
		const aerotelic::LatticeGrid Grid = aerotelic::PlanLatticeGrid(Sampled, Bounds);
		if (Grid.Nodes() > kMaxGridNodes)
		{
			throw std::runtime_error("the lattice grid would need " + std::to_string(Grid.Nodes()) + " samples (limit " + std::to_string(kMaxGridNodes) +
				"): use a larger cell size or a lower resolution");
		}
		const std::string Unresolvable = aerotelic::LatticeResolutionWarning(Sampled, Grid);
		if (!Unresolvable.empty())
		{
			throw std::runtime_error(Unresolvable);
		}
		const aerotelic::LatticeField Lattice(Sampled);
		AerotelicMeshLib::FField Field;
		Field.Nx = Grid.nx + 1;
		Field.Ny = Grid.ny + 1;
		Field.Nz = Grid.nz + 1;
		Field.Origin = Grid.origin;
		Field.H = Grid.h;
		Field.Values.resize(Grid.Nodes());
		const int NX = Field.Nx, NY = Field.Ny;
		aerotelic::ParallelFor(static_cast<size_t>(Field.Nz), [&](size_t K)
		{
			float* Row = Field.Values.data() + K * static_cast<size_t>(NY) * NX;
			for (int J = 0; J < NY; ++J)
			{
				for (int I = 0; I < NX; ++I)
				{
					const aerotelic::Vec3 P = Grid.origin + aerotelic::Vec3{I * Grid.h, J * Grid.h, static_cast<double>(K) * Grid.h};
					Row[static_cast<size_t>(J) * NX + I] = static_cast<float>(Lattice(P));
				}
			}
		});
		return Field;
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
	case EKind::Infill: return kInfillColor;
	case EKind::Ribs: return kRibsColor;
	case EKind::Skin:
	default: return BodyColor;
	}
}

const TCHAR* FAerotelicStructureLayer::KindName(EKind Kind)
{
	switch (Kind)
	{
	case EKind::Infill: return TEXT("infill");
	case EKind::Ribs: return TEXT("ribs");
	case EKind::Skin:
	default: return TEXT("skin");
	}
}

double FAerotelicBodyStructure::BodyTolerance(const aerotelic::BodyStructure& S)
{
	// The body mesh is the skin's outer surface as it stands, and the
	// offset's distance field sees it to a fraction of its voxel, so it is
	// built a few voxels finer than the skin and than the infill's grid.
	double Tolerance = S.skin_thickness > 0.0 ? 0.25 * S.skin_thickness : 1e-3;
	if (S.infill)
	{
		Tolerance = FMath::Min(Tolerance, 0.25 * aerotelic::LatticeVoxelSize(S.infill->lattice));
	}
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

		// The infill's field over the body's bounds, when there is one.
		AerotelicMeshLib::FField Field;
		if (S.infill)
		{
			Field = SampleLatticeField(S.infill->lattice, Bounds);
		}

		// The job. The offset's voxel is a quarter of the skin (so the
		// shell's thickness is resolved) and no coarser than the infill's
		// voxel, capped by a voxel budget across the bounds.
		AerotelicMeshLib::FRequest Request;
		{
			double Voxel = 0.25 * S.skin_thickness;
			if (S.infill) Voxel = FMath::Min(Voxel, aerotelic::LatticeVoxelSize(S.infill->lattice));
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
		// the planes or the infill does not change it.
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
		Request.Field = S.infill ? &Field : nullptr;
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
		// surfaces as they are for the tetrahedral mesher.
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
		if (S.infill)
		{
			FAerotelicStructureLayer Infill;
			Infill.Name = UniqueLayerName(TEXT("Infill"), NewLayers);
			Infill.Kind = FAerotelicStructureLayer::EKind::Infill;
			Infill.Mesh = std::move(Result.Lattice);
			Infill.Material = S.infill->material;
			Infill.Volume = Result.LatticeVolume;
			if (Infill.Mesh.IsEmpty()) Infill.Note = TEXT("the infill came out empty: the cell is larger than the interior, or the volume fraction too small to resolve");
			NewLayers.push_back(std::move(Infill));
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
		MinPocketVolume = Request.MinPocketVolume;
		InfillCell = S.infill ? S.infill->lattice.cell_size : 0.0;
		SkinThickness = S.skin_thickness;
		bTetBuilt = false;
		bTetOk = false;
		TetNote.Empty();
		Tet = FTetMesh();

		Stats = FStats();
		for (const FAerotelicStructureLayer& L : Layers) Stats.Triangles += static_cast<int64>(L.Mesh.triangles.size());
		Stats.GridNodes = static_cast<int64>(Field.Values.size());
		Stats.SkinVolume = Result.SkinVolume;
		Stats.DomainVolume = Result.DomainVolume;
		Stats.InfillVolume = Result.LatticeVolume;
		Stats.RibsVolume = Result.RibsVolume;
		Stats.AchievedFraction = Result.AchievedFraction;
		Stats.Warnings = Result.Warnings;
		Stats.Seconds = FPlatformTime::Seconds() - T0;
		bGenerated = true;
		bStale = false;
		UE_LOG(LogTemp, Log, TEXT("AerotelicBodyStructure: %s: %d layer(s), %lld triangles, offset voxel %.4g, skin %.5g, interior %.5g, ribs %.5g, infill %.5g (fraction %.3f), %.2f s (MeshLib %.2f s)"),
			*In.Label, static_cast<int32>(Layers.size()), Stats.Triangles, Request.OffsetVoxel, Stats.SkinVolume, Stats.DomainVolume, Stats.RibsVolume, Stats.InfillVolume,
			Stats.AchievedFraction, Stats.Seconds, Result.Seconds);
		return true;
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return false;
	}
}

const FAerotelicBodyStructure::FTetMesh* FAerotelicBodyStructure::GetTetMesh(FString& OutNote, double CoreSize) const
{
	if (!bGenerated)
	{
		OutNote = TEXT("the structure has not been generated, so the body's surface and its offset are not available");
		return nullptr;
	}
	if (bTetBuilt)
	{
		OutNote = TetNote;
		return bTetOk ? &Tet : nullptr;
	}
	bTetBuilt = true;
	bTetOk = false;
	if (!Inner.IsEmpty() && !bInnerUsable)
	{
		TetNote = FString::Printf(TEXT("not meshed: %s"), *InnerNote);
		OutNote = TetNote;
		return nullptr;
	}
	AerotelicTetGmsh::FRequest Request;
	Request.Outer = &Outer;
	Request.Inner = Inner.IsEmpty() ? nullptr : &Inner;
	if (CoreSize > 0.0) Request.CoreSize = CoreSize;
	else if (InfillCell > 0.0) Request.CoreSize = kCoreSizeOverCell * InfillCell;
	else Request.CoreSize = FMath::Max(4.0 * SkinThickness, Outer.Bounds().Diagonal() / 40.0);
	Request.Algorithm = kGmshAlgorithm;
	Request.bOptimize = true;
	Request.bOptimizeNetgen = true;
	Request.MinPocketVolume = MinPocketVolume;
	FString Error;
	if (!AerotelicTetGmsh::Run(Request, Tet.Result, Error))
	{
		TetNote = FString::Printf(TEXT("not meshed: %s"), *Error);
		UE_LOG(LogTemp, Warning, TEXT("AerotelicBodyStructure: the body could not be meshed with tetrahedra: %s"), *Error);
		OutNote = TetNote;
		return nullptr;
	}
	Tet.SkinThickness = SkinThickness;
	const AerotelicTetGmsh::FResult& R = Tet.Result;
	UE_LOG(LogTemp, Log, TEXT("AerotelicBodyStructure: tetrahedra: %d nodes, %d skin + %d core; skin %.6g + core %.6g = %.6g vs body %.6g (relative %.2e); quality min %.3f mean %.3f; %.2f s"),
		R.Nodes, R.SkinTets, R.CoreTets, R.SkinVolume, R.CoreVolume, R.SkinVolume + R.CoreVolume, R.OuterVolume, R.VolumeError, R.MinQuality, R.MeanQuality, R.Seconds);
	TArray<FString> Notes;
	if (R.VolumeError > 1e-9) Notes.Add(FString::Printf(TEXT("the tetrahedra do not fill the body exactly (relative %.2e)"), R.VolumeError));
	if (R.Degenerate > 0) Notes.Add(FString::Printf(TEXT("%d tetrahedra came out with no volume"), R.Degenerate));
	if (Inner.IsEmpty()) Notes.Add(TEXT("nowhere thicker than two skins: every tetrahedron is skin"));
	TetNote = FString::Join(Notes, TEXT("; "));
	OutNote = TetNote;
	bTetOk = true;
	return &Tet;
}
