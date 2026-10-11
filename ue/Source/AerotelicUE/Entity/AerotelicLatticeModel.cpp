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

	std::vector<FBodyField> BodyFields, ReservedFields;
	aerotelic::Box3 Bounds;
	for (const FPlacedBody& B : Bodies)
	{
		FBodyField F;
		F.Sdf = std::make_unique<aerotelic::MeshSdf>(B.Mesh);
		F.Box = B.Mesh.Bounds();
		Bounds.Add(F.Box);
		BodyFields.push_back(std::move(F));
	}
	for (const FPlacedBody& B : Reserved)
	{
		FBodyField F;
		F.Sdf = std::make_unique<aerotelic::MeshSdf>(B.Mesh);
		F.Box = B.Mesh.Bounds();
		ReservedFields.push_back(std::move(F));
	}

	aerotelic::LatticeDomain Domain;
	Domain.bounds = Bounds;
	// The union of the bodies: the nearest surface wins.
	Domain.body = [&BodyFields](const aerotelic::Vec3& P)
	{
		double Best = 1e300;
		for (const FBodyField& F : BodyFields)
		{
			Best = FMath::Min(Best, F.Distance(P));
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

	aerotelic::LatticeResult Result = aerotelic::GenerateLattice(Spec, Domain);
	if (Result.mesh.IsEmpty() && Result.skin.IsEmpty())
	{
		throw std::runtime_error("The lattice came out empty: the cell is larger than the body, or the volume fraction is too small to resolve at this resolution.");
	}

	Stats.AchievedFraction = Result.achieved_fraction;
	Stats.Triangles = static_cast<int64>(Result.mesh.triangles.size() + Result.skin.triangles.size());
	Stats.GridNodes = static_cast<int64>(Result.grid.Nodes());
	Stats.BodyEvaluations = static_cast<int64>(Result.nodes_evaluated);
	Stats.GridX = Result.grid.nx;
	Stats.GridY = Result.grid.ny;
	Stats.GridZ = Result.grid.nz;
	Stats.Seconds = FPlatformTime::Seconds() - T0;
	UE_LOG(LogTemp, Log, TEXT("AerotelicLattice: %s %s, grid %d x %d x %d (%lld samples, %lld body evaluations), %lld lattice + %lld skin triangles, fraction %.3f, %.2f s"),
		UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(Spec.family)), UTF8_TO_TCHAR(Spec.type.c_str()), Stats.GridX, Stats.GridY, Stats.GridZ,
		Stats.GridNodes, Stats.BodyEvaluations, (int64)Result.mesh.triangles.size(), (int64)Result.skin.triangles.size(), Stats.AchievedFraction, Stats.Seconds);

	// Two bodies, so the shell and the infill can be coloured, hidden,
	// exported and given materials separately: the skin first (the outer
	// shell between the surface and its inward offset), then the lattice
	// inside that offset. They meet on the offset surface vertex for vertex.
	Cached.clear();
	if (!Result.skin.IsEmpty())
	{
		Cached.push_back({"Skin", std::move(Result.skin)});
	}
	if (!Result.mesh.IsEmpty())
	{
		Cached.push_back({"Lattice", std::move(Result.mesh)});
	}
	bValid = true;
	return Cached;
}
