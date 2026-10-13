#include "AerotelicLatticeModel.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"

#include <cmath>
#include <memory>
#include <stdexcept>

#include "aerotelic/sdf.hpp"

namespace
{
	using aerotelic::project_yaml::EntityType;

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

	// The distance field's sign (inside / outside) is only trustworthy on a
	// closed, outward surface: an open mesh has edges with one side only,
	// whose normals point whichever way that side faces, and the field then
	// reports "inside" in whole regions outside the body (lattice floating
	// in space). Refuse such a body by name rather than generate from it.
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

	// A body's signed distance, never negative outside the body's own
	// bounding box: a point outside the box is outside the body, whatever
	// the field says. Defence in depth behind RequireClosedBody -- a sign
	// error that slips through (a fold the manifold check cannot see) can
	// then only act within the box, never fill space beside the body.
	struct FBodyField
	{
		std::unique_ptr<aerotelic::MeshSdf> Sdf;
		aerotelic::Box3 Box;

		double Distance(const aerotelic::Vec3& P) const
		{
			const double D = Sdf->Distance(P);
			const double ToBox2 = Box.DistanceSq(P);
			return ToBox2 > 0.0 ? FMath::Max(D, std::sqrt(ToBox2)) : D;
		}
	};
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

	// The bodies as distance fields. Their meshes are built a few voxels
	// finer than the lattice grid, which is as fine as the grid can see.
	const double Voxel = Spec.cell_size / FMath::Max(Spec.resolution, 2);
	const double MeshTolerance = FMath::Max(Voxel * 0.25, 1e-5);
	TArray<FString> Missing;
	std::vector<FPlacedBody> Bodies = PlacedMeshes(Targets, MeshTolerance, Missing);
	if (Bodies.empty())
	{
		throw std::runtime_error(std::string("No body to fill could be built") + (Missing.Num() ? std::string(": ") + TCHAR_TO_UTF8(*FString::Join(Missing, TEXT(", "))) : std::string("")) + ".");
	}
	TArray<FString> MissingExclusions;
	std::vector<FPlacedBody> Reserved = PlacedMeshes(Exclusions, MeshTolerance, MissingExclusions);
	for (const FPlacedBody& B : Bodies)
	{
		RequireClosedBody(B, TEXT("a body to fill"));
	}
	for (const FPlacedBody& B : Reserved)
	{
		RequireClosedBody(B, TEXT("a reserved volume"));
	}

	// The skin of each body, from the tetrahedral mesher (tet.hpp): the
	// "Skin" body is the closed boundary of the skin tetrahedra -- the
	// body's own surface triangles outside, the exact iso-surface of its
	// distance field at the skin depth inside. (An offset COPY of the
	// surface, the earlier shell, cannot do this: at a concave crease such
	// as a tailplane meeting a boom its straight inner edges bridge the
	// corner outside the body, and where the surface is sharper than the
	// skin its triangles fold.) The lattice is clipped by the BODY's own
	// distance field shifted inward by the skin thickness: the body mesh is
	// closed and outward everywhere (RequireClosedBody), so its sign is
	// trustworthy, and b + t < 0 is exactly "deeper than the skin". A body
	// thinner than two skins then has no interior and is all skin.
	std::vector<FBodyField> DomainFields, ReservedFields;
	aerotelic::TriMesh SkinMesh;
	aerotelic::Box3 DomainBounds;
	int64 SkinTets = 0;
	SkinMeshes.clear();
	for (const FPlacedBody& B : Bodies)
	{
		FBodyField F;
		F.Sdf = std::make_unique<aerotelic::MeshSdf>(B.Mesh);
		F.Box = B.Mesh.Bounds();
		aerotelic::SkinCoreOptions Opt;
		Opt.thickness = Spec.skin_thickness;
		Opt.core_spacing = 0.0;       // the core is only meshed to bound the skin; the lattice is sampled
		Opt.improve = true;
		aerotelic::SkinCoreStats St;
		FSkinMesh S;
		S.Label = B.Label;
		try
		{
			S.Mesh = aerotelic::BuildSkinCoreMesh(B.Mesh, *F.Sdf, Opt, &St);
		}
		catch (const std::exception& E)
		{
			throw std::runtime_error(TCHAR_TO_UTF8(*FString::Printf(TEXT("The skin of '%s' could not be meshed: %s"), *B.Label, UTF8_TO_TCHAR(E.what()))));
		}
		if (St.conforming.n_faces_abandoned > 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("AerotelicLattice: the mesh of '%s' does not conform exactly (%d surface faces abandoned); its skin follows the tetrahedra there"), *B.Label, St.conforming.n_faces_abandoned);
		}
		aerotelic::TriMesh Skin = S.Mesh.BoundaryFaces(1);
		if (!Skin.IsClosedManifold())
		{
			UE_LOG(LogTemp, Warning, TEXT("AerotelicLattice: the skin of '%s' is not a closed manifold (a pinched thin region?)"), *B.Label);
		}
		SkinMesh.Append(Skin);
		SkinTets += static_cast<int64>(St.n_skin_tets);
		SkinMeshes.push_back(std::move(S));
		DomainBounds.Add(F.Box);
		DomainFields.push_back(std::move(F));
	}
	for (const FPlacedBody& B : Reserved)
	{
		FBodyField F;
		F.Sdf = std::make_unique<aerotelic::MeshSdf>(B.Mesh);
		F.Box = B.Mesh.Bounds();
		ReservedFields.push_back(std::move(F));
	}

	aerotelic::LatticeResult Result;
	if (!DomainFields.empty())
	{
		aerotelic::LatticeDomain Domain;
		Domain.bounds = DomainBounds;
		// The union of the bodies' interiors beyond the skin: the nearest
		// body wins, and each body's field is shifted by the skin thickness
		// (still a distance, so the generator's block skipping holds). A
		// point is lattice only where some body's field is below -t, so a
		// point outside every body (b >= 0) can never be.
		const double Thickness = Spec.skin_thickness;
		Domain.body = [&DomainFields, Thickness](const aerotelic::Vec3& P)
		{
			double Best = 1e300;
			for (const FBodyField& F : DomainFields)
			{
				Best = FMath::Min(Best, F.Distance(P) + Thickness);
			}
			return Best;
		};
		if (!ReservedFields.empty())
		{
			Domain.exclusion = [&ReservedFields](const aerotelic::Vec3& P)
			{
				double Best = 1e300;
				for (const FBodyField& F : ReservedFields)
				{
					Best = FMath::Min(Best, F.Distance(P));
				}
				return Best;
			};
		}
		// The explicit shell replaces the generator's own sampled skin.
		aerotelic::LatticeSpec Sampled = Spec;
		Sampled.skin_thickness = 0.0;
		Result = aerotelic::GenerateLattice(Sampled, Domain);
	}
	if (Result.mesh.IsEmpty() && SkinMesh.IsEmpty())
	{
		throw std::runtime_error("The lattice came out empty: the cell is larger than the body, or the volume fraction is too small to resolve at this resolution.");
	}

	Stats.AchievedFraction = Result.achieved_fraction;
	Stats.Triangles = static_cast<int64>(Result.mesh.triangles.size() + SkinMesh.triangles.size());
	Stats.SkinTets = SkinTets;
	Stats.GridNodes = static_cast<int64>(Result.grid.Nodes());
	Stats.BodyEvaluations = static_cast<int64>(Result.nodes_evaluated);
	Stats.GridX = Result.grid.nx;
	Stats.GridY = Result.grid.ny;
	Stats.GridZ = Result.grid.nz;
	Stats.Seconds = FPlatformTime::Seconds() - T0;
	UE_LOG(LogTemp, Log, TEXT("AerotelicLattice: %s %s, grid %d x %d x %d (%lld samples, %lld body evaluations), %lld lattice + %lld skin triangles, %lld skin tets, fraction %.3f, %.2f s"),
		UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(Spec.family)), UTF8_TO_TCHAR(Spec.type.c_str()), Stats.GridX, Stats.GridY, Stats.GridZ,
		Stats.GridNodes, Stats.BodyEvaluations, (int64)Result.mesh.triangles.size(), (int64)SkinMesh.triangles.size(), SkinTets, Stats.AchievedFraction, Stats.Seconds);

	// Two bodies, so the shell and the infill can be coloured, hidden,
	// exported and given materials separately: the skin first, then the
	// lattice inside its inner surface.
	Cached.clear();
	if (!SkinMesh.IsEmpty())
	{
		Cached.push_back({"Skin", std::move(SkinMesh)});
	}
	if (!Result.mesh.IsEmpty())
	{
		Cached.push_back({"Lattice", std::move(Result.mesh)});
	}
	bValid = true;
	return Cached;
}

std::vector<aerotelic::NamedMesh> FAerotelicLatticeModel::TargetBodies(double Tolerance, TArray<FString>& OutMissing) const
{
	std::vector<aerotelic::NamedMesh> Out;
	for (FPlacedBody& B : PlacedMeshes(Targets, Tolerance, OutMissing))
	{
		Out.push_back({TCHAR_TO_UTF8(*B.Label), std::move(B.Mesh)});
	}
	return Out;
}

aerotelic::TetMesh FAerotelicLatticeModel::MergedSkinTets() const
{
	aerotelic::TetMesh Out;
	for (const FSkinMesh& S : SkinMeshes)
	{
		// Only the nodes the skin tetrahedra use.
		std::vector<int> Remap(S.Mesh.nodes.size(), -1);
		for (size_t K = 0; K < S.Mesh.tets.size(); ++K)
		{
			if (S.Mesh.material[K] != 1) continue;
			for (int I = 0; I < 4; ++I)
			{
				int& R = Remap[S.Mesh.tets[K][I]];
				if (R < 0)
				{
					R = static_cast<int>(Out.nodes.size());
					Out.nodes.push_back(S.Mesh.nodes[S.Mesh.tets[K][I]]);
				}
			}
			const auto& T = S.Mesh.tets[K];
			Out.tets.push_back({Remap[T[0]], Remap[T[1]], Remap[T[2]], Remap[T[3]]});
			Out.constrained.push_back(S.Mesh.constrained[K]);
			Out.material.push_back(1);
		}
	}
	Out.neighbors.assign(Out.tets.size(), {{-1, -1, -1, -1}});
	return Out;
}
