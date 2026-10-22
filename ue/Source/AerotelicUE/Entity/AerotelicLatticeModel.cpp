#include "AerotelicLatticeModel.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"
#include "AerotelicMeshLib.h"

#include <cmath>
#include <memory>
#include <stdexcept>

#include "aerotelic/parallel.hpp"
#include "aerotelic/sdf.hpp"

namespace
{
	using aerotelic::project_yaml::EntityType;

	// The lattice field grid is refused beyond this many samples (as the
	// core's GenerateLattice).
	constexpr size_t kMaxGridNodes = 150000000;
	// The offset's distance field is kept under about this many voxels
	// across the bodies' bounds (it is sparse, so the actual count is far
	// lower; this bounds the worst case).
	constexpr double kMaxOffsetVoxels = 60.0e6;
	// The offset surface is remeshed to isotropic triangles of this many
	// skin thicknesses before it is used. This is not for the tetrahedral
	// mesher's sake but for the surface's own: the raw marching-cubes
	// offset, decimated, touches the body and crosses itself where it
	// folds through a thin region, and no mesher can take that. Measured
	// on p38a's airframe: raw and three skins are both refused ("a segment
	// and a facet intersect"), two skins is clean and meshes in 15 s
	// against 350 s at one skin, with the same exact volumes.
	constexpr double kInnerRemeshEdgeOverSkin = 2.0;
	// Target edge length deep in the core, as a multiple of the lattice's
	// unit cell: the core carries the lattice's material card rather than
	// its geometry, so it needs no finer a mesh than the cell it
	// homogenises.
	constexpr double kCoreSizeOverCell = 1.0;
	// gmsh's Mesh.Algorithm3D. 1 (Delaunay) beat 10 (HXT) on every body
	// tried here, on time and on quality alike.
	constexpr int32 kGmshAlgorithm = 1;

	bool OperandDependsOn(const FAerotelicBooleanOperand& Op, const AAerotelicEntityActor* Entity)
	{
		if (!Op.Entity.IsValid())
		{
			return false;
		}
		if (Op.Entity.Get() == Entity)
		{
			return true;
		}
		if (Op.Entity->GetEntityType() == EntityType::Boolean)
		{
			return static_cast<const FAerotelicBooleanModel*>(Op.Entity->GetModel())->DependsOn(Entity);
		}
		if (Op.Entity->GetEntityType() == EntityType::Lattice)
		{
			return static_cast<const FAerotelicLatticeModel*>(Op.Entity->GetModel())->DependsOn(Entity);
		}
		return false;
	}

	// A placed body: its mesh and the name it is reported under.
	struct FPlacedBody
	{
		aerotelic::TriMesh Mesh;
		FString Label;
	};

	// The placed meshes of every operand, with the names of those that
	// produced nothing.
	std::vector<FPlacedBody> PlacedMeshes(const TArray<FAerotelicBooleanOperand>& Operands, double Tolerance, TArray<FString>& OutMissing)
	{
		std::vector<FPlacedBody> Out;
		for (const FAerotelicBooleanOperand& Op : Operands)
		{
			if (!Op.Entity.IsValid())
			{
				OutMissing.Add(TEXT("(removed entity)"));
				continue;
			}
			const FString EntityLabel = Op.Entity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT(""));
			std::vector<aerotelic::NamedMesh> Meshes = Op.Entity->GetPlacedMeshes(Op.bMirror, Tolerance);
			bool bAny = false;
			for (aerotelic::NamedMesh& M : Meshes)
			{
				if (!M.mesh.IsEmpty())
				{
					FPlacedBody Body;
					Body.Mesh = std::move(M.mesh);
					Body.Label = Meshes.size() > 1 ? FString::Printf(TEXT("%s / %s"), *EntityLabel, UTF8_TO_TCHAR(M.name.c_str())) : EntityLabel;
					Out.push_back(std::move(Body));
					bAny = true;
				}
			}
			if (!bAny)
			{
				OutMissing.Add(EntityLabel);
			}
		}
		return Out;
	}

	// Inside and outside are only defined on a closed, outward surface: an
	// open mesh has edges with one side only, and neither the offset's
	// distance field nor a mesh boolean can tell its inside from its
	// outside. Refuse such a body by name rather than generate from it.
	void RequireClosedBody(const FPlacedBody& Body, const TCHAR* Role)
	{
		if (!Body.Mesh.IsClosedManifold())
		{
			throw std::runtime_error(TCHAR_TO_UTF8(*FString::Printf(
				TEXT("'%s' (%s) is not a closed surface: its mesh has edges with a triangle on one side only, so inside and outside cannot be told apart. The lattice was not generated. Rebuild or repair that body first."),
				*Body.Label, Role)));
		}
		if (!(Body.Mesh.SignedVolume() > 0.0))
		{
			throw std::runtime_error(TCHAR_TO_UTF8(*FString::Printf(
				TEXT("'%s' (%s) is turned inside out (its enclosed volume is not positive), so inside and outside cannot be told apart. The lattice was not generated."),
				*Body.Label, Role)));
		}
	}

	// The lattice field of the spec sampled on the grid the core would
	// plan over `Bounds` (the generator's own grid: a cell of `resolution`
	// voxels, the bounds plus two voxels), F <= 0 inside the material.
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
		// A wall thinner than a voxel can fall between the samples and go
		// missing (a honeycomb's, or a TPMS iso-surface's, thickness is set
		// apart from its cell, so it can be asked for at any size).
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
}

TopoDS_Shape FAerotelicLatticeModel::BuildSolid() const
{
	throw std::runtime_error("A lattice is a triangle mesh, not a solid: export it as STL.");
}

bool FAerotelicLatticeModel::DependsOn(const AAerotelicEntityActor* Entity) const
{
	for (const FAerotelicBooleanOperand& Op : Targets)
	{
		if (OperandDependsOn(Op, Entity)) return true;
	}
	for (const FAerotelicBooleanOperand& Op : Exclusions)
	{
		if (OperandDependsOn(Op, Entity)) return true;
	}
	return false;
}

FBox FAerotelicLatticeModel::TargetBounds() const
{
	FBox Box(ForceInit);
	for (const FAerotelicBooleanOperand& Op : Targets)
	{
		if (Op.Entity.IsValid())
		{
			Box += Op.Entity->GetPlacedBounds(Op.bMirror);
		}
	}
	return Box;
}

double FAerotelicLatticeModel::BodyTolerance(const aerotelic::LatticeSpec& Spec)
{
	// The body mesh is the skin's outer surface as it stands, and the
	// offset's distance field sees it to a fraction of its voxel, so it is
	// built a few voxels finer than the lattice grid and than the skin.
	const double Voxel = aerotelic::LatticeVoxelSize(Spec);
	double Tolerance = 0.25 * Voxel;
	if (Spec.skin_thickness > 0.0)
	{
		Tolerance = FMath::Min(Tolerance, 0.25 * Spec.skin_thickness);
	}
	return FMath::Max(Tolerance, 1e-5);
}

std::vector<aerotelic::NamedMesh> FAerotelicLatticeModel::BuildMeshes(double) const
{
	if (bValid)
	{
		return Cached;
	}
	if (Targets.Num() == 0)
	{
		throw std::runtime_error("The lattice fills no bodies.");
	}
	if (!(Spec.skin_thickness > 0.0))
	{
		throw std::runtime_error("The solid skin thickness must be greater than zero: the skin is the part's outer shell (its own body, 'Skin'), one percent of the largest root chord by default.");
	}
	const double T0 = FPlatformTime::Seconds();

	const double MeshTolerance = BodyTolerance(Spec);
	TArray<FString> Missing;
	std::vector<FPlacedBody> Bodies = PlacedMeshes(Targets, MeshTolerance, Missing);
	if (Bodies.empty())
	{
		throw std::runtime_error(std::string("No body to fill could be built") + (Missing.Num() ? std::string(": ") + TCHAR_TO_UTF8(*FString::Join(Missing, TEXT(", "))) : std::string("")) + ".");
	}
	TArray<FString> MissingExclusions;
	std::vector<FPlacedBody> Reserved = PlacedMeshes(Exclusions, MeshTolerance, MissingExclusions);
	aerotelic::Box3 DomainBounds;
	for (const FPlacedBody& B : Bodies)
	{
		RequireClosedBody(B, TEXT("a body to fill"));
		DomainBounds.Add(B.Mesh.Bounds());
	}
	for (const FPlacedBody& B : Reserved)
	{
		RequireClosedBody(B, TEXT("a reserved volume"));
	}

	// The lattice field over the bodies' bounds: what MeshLib extracts the
	// infill surface from before cutting it to the hollowed interior.
	const AerotelicMeshLib::FField Field = SampleLatticeField(Spec, DomainBounds);

	// The job: the bodies, the field, and how the inward offset is taken.
	// Its distance field's voxel is a quarter of the skin (so the shell's
	// thickness is resolved) and no coarser than the lattice voxel, capped
	// by a voxel budget across the bounds; the offset surface is decimated
	// within a fifth of that voxel, and pockets of the interior smaller
	// than a few voxels (slivers along a trailing edge) are filled solid.
	AerotelicMeshLib::FRequest Request;
	for (const FPlacedBody& B : Bodies)
	{
		Request.Targets.Add({B.Label, &B.Mesh});
	}
	for (const FPlacedBody& B : Reserved)
	{
		Request.Reserved.Add({B.Label, &B.Mesh});
	}
	Request.SkinThickness = Spec.skin_thickness;
	{
		const double LatticeVoxel = aerotelic::LatticeVoxelSize(Spec);
		double Voxel = FMath::Min(0.25 * Spec.skin_thickness, LatticeVoxel);
		const aerotelic::Vec3 Ext = DomainBounds.Extent() + aerotelic::Vec3{2.0, 2.0, 2.0} * Spec.skin_thickness;
		const double Volume = Ext.x * Ext.y * Ext.z;
		if (Volume / (Voxel * Voxel * Voxel) > kMaxOffsetVoxels)
		{
			Voxel = std::cbrt(Volume / kMaxOffsetVoxels);
		}
		Request.OffsetVoxel = FMath::Max(Voxel, 1e-6);
	}
	Request.OffsetMode = TEXT("standard");
	Request.DecimateTolerance = 0.2 * Request.OffsetVoxel;
	Request.MinPocketVolume = FMath::Pow(4.0 * Request.OffsetVoxel, 3.0);
	// No opening: the hollow is the offset's own interior. (It was opened
	// by a skin thickness while the tetrahedral mesh came from the core's
	// constrained Delaunay mesher, which could not take a hollow whose
	// two sides were closer than its triangles; that cost up to 94% of the
	// hollow on a thick-skinned wing. gmsh needs no such help.)
	Request.InnerOpening = 0.0;
	Request.InnerRemeshEdge = kInnerRemeshEdgeOverSkin * Spec.skin_thickness;
	Request.Field = &Field;

	TetBodies.clear();
	TetMeshes.clear();
	bTetMeshesBuilt = false;
	TetNotes.Reset();
	AerotelicMeshLib::FResult Result;
	FString Error;
	if (!AerotelicMeshLib::Run(Request, Result, Error))
	{
		throw std::runtime_error(TCHAR_TO_UTF8(*Error));
	}
	// What the tetrahedral mesher will conform to: each body's surface as
	// MeshLib hollowed it, and its offset.
	for (size_t I = 0; I < Bodies.size() && static_cast<int32>(I) < Result.Inners.Num(); ++I)
	{
		FTetBody T;
		T.Label = Bodies[I].Label;
		T.Outer = std::move(Bodies[I].Mesh);
		T.Inner = std::move(Result.Inners[I].Mesh);
		T.bInnerUsable = Result.Inners[I].bUsable;
		T.InnerNote = Result.Inners[I].Note;
		T.MinPocketVolume = Request.MinPocketVolume;
		TetBodies.push_back(std::move(T));
	}
	if (Result.Skin.IsEmpty() && Result.Lattice.IsEmpty())
	{
		throw std::runtime_error("The lattice came out empty: the cell is larger than the body, or the volume fraction is too small to resolve at this resolution.");
	}
	if (!Result.Skin.IsEmpty() && !Result.Skin.IsClosedManifold())
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicLattice: the skin is not a closed manifold"));
	}
	if (!Result.Lattice.IsEmpty() && !Result.Lattice.IsClosedManifold())
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicLattice: the lattice is not a closed manifold"));
	}
	for (const FString& W : Result.Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicLattice: %s"), *W);
	}

	Stats = FStats();
	Stats.AchievedFraction = Result.AchievedFraction;
	Stats.SkinTriangles = static_cast<int64>(Result.Skin.triangles.size());
	Stats.LatticeTriangles = static_cast<int64>(Result.Lattice.triangles.size());
	Stats.Triangles = Stats.SkinTriangles + Stats.LatticeTriangles;
	Stats.GridNodes = static_cast<int64>(Field.Values.size());
	Stats.SkinVolume = Result.SkinVolume;
	Stats.DomainVolume = Result.DomainVolume;
	Stats.LatticeVolume = Result.LatticeVolume;
	Stats.GridX = Field.Nx - 1;
	Stats.GridY = Field.Ny - 1;
	Stats.GridZ = Field.Nz - 1;
	Stats.Warnings = Result.Warnings;
	Stats.Seconds = FPlatformTime::Seconds() - T0;
	UE_LOG(LogTemp, Log, TEXT("AerotelicLattice: %s %s, grid %d x %d x %d (%lld samples), offset voxel %.4g, %lld lattice + %lld skin triangles, skin %.5g, domain %.5g, lattice %.5g (fraction %.3f), %.2f s (MeshLib %.2f s)"),
		UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(Spec.family)), UTF8_TO_TCHAR(Spec.type.c_str()), Stats.GridX, Stats.GridY, Stats.GridZ,
		Stats.GridNodes, Request.OffsetVoxel, Stats.LatticeTriangles, Stats.SkinTriangles, Stats.SkinVolume, Stats.DomainVolume, Stats.LatticeVolume,
		Stats.AchievedFraction, Stats.Seconds, Result.Seconds);

	// Two bodies, so the shell and the infill can be coloured, hidden,
	// exported and given materials separately: the skin first, then the
	// lattice inside its inner surface.
	Cached.clear();
	if (!Result.Skin.IsEmpty())
	{
		Cached.push_back({"Skin", std::move(Result.Skin)});
	}
	if (!Result.Lattice.IsEmpty())
	{
		Cached.push_back({"Lattice", std::move(Result.Lattice)});
	}
	bValid = true;
	return Cached;
}

const std::vector<FAerotelicLatticeModel::FTetMesh>& FAerotelicLatticeModel::GetTetMeshes() const
{
	if (bTetMeshesBuilt)
	{
		return TetMeshes;
	}
	bTetMeshesBuilt = true;
	TetMeshes.clear();
	TetNotes.Reset();
	if (TetBodies.empty())
	{
		TetNotes.Add(TEXT("the lattice has not been generated, so its bodies and their offsets are not available"));
		return TetMeshes;
	}
	for (const FTetBody& B : TetBodies)
	{
		if (!B.Inner.IsEmpty() && !B.bInnerUsable)
		{
			TetNotes.Add(FString::Printf(TEXT("%s: not meshed: %s"), *B.Label, *B.InnerNote));
			continue;
		}
		AerotelicTetGmsh::FRequest Request;
		Request.Outer = &B.Outer;
		Request.Inner = B.Inner.IsEmpty() ? nullptr : &B.Inner;
		Request.CoreSize = kCoreSizeOverCell * Spec.cell_size;
		Request.Algorithm = kGmshAlgorithm;
		Request.bOptimize = true;
		Request.bOptimizeNetgen = true;
		// The same threshold the lattice job drops offset pockets at, so a
		// sliver the remesh left cannot cost the whole body its mesh.
		Request.MinPocketVolume = B.MinPocketVolume;
		FTetMesh M;
		M.Label = B.Label;
		FString Error;
		if (!AerotelicTetGmsh::Run(Request, M.Result, Error))
		{
			TetNotes.Add(FString::Printf(TEXT("%s: not meshed: %s"), *B.Label, *Error));
			UE_LOG(LogTemp, Warning, TEXT("AerotelicTet: '%s' could not be meshed with tetrahedra: %s"), *B.Label, *Error);
			continue;
		}
		const AerotelicTetGmsh::FResult& R = M.Result;
		UE_LOG(LogTemp, Log, TEXT("AerotelicTet: %s: %d nodes, %d skin + %d core tets; skin %.6g + core %.6g = %.6g vs body %.6g (relative %.2e); quality min %.3f mean %.3f, %d under 0.1; %d flipped, %d degenerate; %.2f s"),
			*B.Label, R.Nodes, R.SkinTets, R.CoreTets, R.SkinVolume, R.CoreVolume, R.SkinVolume + R.CoreVolume, R.OuterVolume,
			R.VolumeError, R.MinQuality, R.MeanQuality, R.PoorQuality, R.Flipped, R.Degenerate, R.Seconds);
		// The whole point of the exercise: the tetrahedra fill the body.
		// A mesher that silently did not is worth saying so about.
		if (R.VolumeError > 1e-9)
		{
			TetNotes.Add(FString::Printf(TEXT("%s: the tetrahedra do not fill the body exactly (skin + core %.9g against %.9g, relative %.2e)"), *B.Label, R.SkinVolume + R.CoreVolume, R.OuterVolume, R.VolumeError));
		}
		if (R.Degenerate > 0)
		{
			TetNotes.Add(FString::Printf(TEXT("%s: %d tetrahedra came out with no volume"), *B.Label, R.Degenerate));
		}
		if (B.Inner.IsEmpty())
		{
			TetNotes.Add(FString::Printf(TEXT("%s: nowhere thicker than two skins: every tetrahedron is skin"), *B.Label));
		}
		TetMeshes.push_back(std::move(M));
	}
	return TetMeshes;
}

aerotelic::TetMesh FAerotelicLatticeModel::MergedTets(int Material) const
{
	aerotelic::TetMesh Out;
	for (const FTetMesh& S : GetTetMeshes())
	{
		const aerotelic::TetMesh& Mesh = S.Result.Mesh;
		// Only the nodes the chosen tetrahedra use.
		std::vector<int> Remap(Mesh.nodes.size(), -1);
		for (size_t K = 0; K < Mesh.tets.size(); ++K)
		{
			if (Material >= 0 && Mesh.material[K] != Material) continue;
			for (int I = 0; I < 4; ++I)
			{
				int& R = Remap[Mesh.tets[K][I]];
				if (R < 0)
				{
					R = static_cast<int>(Out.nodes.size());
					Out.nodes.push_back(Mesh.nodes[Mesh.tets[K][I]]);
				}
			}
			const auto& T = Mesh.tets[K];
			Out.tets.push_back({Remap[T[0]], Remap[T[1]], Remap[T[2]], Remap[T[3]]});
			Out.constrained.push_back(Mesh.constrained.empty() ? 0 : Mesh.constrained[K]);
			Out.material.push_back(Mesh.material[K]);
		}
	}
	Out.neighbors.assign(Out.tets.size(), {{-1, -1, -1, -1}});
	return Out;
}
