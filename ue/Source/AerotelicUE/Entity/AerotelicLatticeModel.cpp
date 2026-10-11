#include "AerotelicLatticeModel.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"

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

	// The placed meshes of every operand, with the names of those that
	// produced nothing.
	std::vector<aerotelic::TriMesh> PlacedMeshes(const TArray<FAerotelicBooleanOperand>& Operands, double Tolerance, TArray<FString>& OutMissing)
	{
		std::vector<aerotelic::TriMesh> Out;
		for (const FAerotelicBooleanOperand& Op : Operands)
		{
			if (!Op.Entity.IsValid())
			{
				OutMissing.Add(TEXT("(removed entity)"));
				continue;
			}
			std::vector<aerotelic::NamedMesh> Meshes = Op.Entity->GetPlacedMeshes(Op.bMirror, Tolerance);
			bool bAny = false;
			for (aerotelic::NamedMesh& M : Meshes)
			{
				if (!M.mesh.IsEmpty())
				{
					Out.push_back(std::move(M.mesh));
					bAny = true;
				}
			}
			if (!bAny)
			{
				OutMissing.Add(Op.Entity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT("")));
			}
		}
		return Out;
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
	const double T0 = FPlatformTime::Seconds();

	// The bodies as distance fields. Their meshes are built a few voxels
	// finer than the lattice grid, which is as fine as the grid can see.
	const double Voxel = Spec.cell_size / FMath::Max(Spec.resolution, 2);
	const double MeshTolerance = FMath::Max(Voxel * 0.25, 1e-5);
	TArray<FString> Missing;
	std::vector<aerotelic::TriMesh> Bodies = PlacedMeshes(Targets, MeshTolerance, Missing);
	if (Bodies.empty())
	{
		throw std::runtime_error(std::string("No body to fill could be built") + (Missing.Num() ? std::string(": ") + TCHAR_TO_UTF8(*FString::Join(Missing, TEXT(", "))) : std::string("")) + ".");
	}
	TArray<FString> MissingExclusions;
	std::vector<aerotelic::TriMesh> Reserved = PlacedMeshes(Exclusions, MeshTolerance, MissingExclusions);

	std::vector<std::unique_ptr<aerotelic::MeshSdf>> BodyFields, ReservedFields;
	aerotelic::Box3 Bounds;
	for (const aerotelic::TriMesh& M : Bodies)
	{
		BodyFields.push_back(std::make_unique<aerotelic::MeshSdf>(M));
		Bounds.Add(M.Bounds());
	}
	for (const aerotelic::TriMesh& M : Reserved)
	{
		ReservedFields.push_back(std::make_unique<aerotelic::MeshSdf>(M));
	}

	aerotelic::LatticeDomain Domain;
	Domain.bounds = Bounds;
	// The union of the bodies: the nearest surface wins.
	Domain.body = [&BodyFields](const aerotelic::Vec3& P)
	{
		double Best = 1e300;
		for (const auto& F : BodyFields)
		{
			Best = FMath::Min(Best, F->Distance(P));
		}
		return Best;
	};
	if (!ReservedFields.empty())
	{
		Domain.exclusion = [&ReservedFields](const aerotelic::Vec3& P)
		{
			double Best = 1e300;
			for (const auto& F : ReservedFields)
			{
				Best = FMath::Min(Best, F->Distance(P));
			}
			return Best;
		};
	}

	aerotelic::LatticeResult Result = aerotelic::GenerateLattice(Spec, Domain);
	if (Result.mesh.IsEmpty())
	{
		throw std::runtime_error("The lattice came out empty: the cell is larger than the body, or the volume fraction is too small to resolve at this resolution.");
	}

	Stats.AchievedFraction = Result.achieved_fraction;
	Stats.Triangles = static_cast<int64>(Result.mesh.triangles.size());
	Stats.GridNodes = static_cast<int64>(Result.grid.Nodes());
	Stats.BodyEvaluations = static_cast<int64>(Result.nodes_evaluated);
	Stats.GridX = Result.grid.nx;
	Stats.GridY = Result.grid.ny;
	Stats.GridZ = Result.grid.nz;
	Stats.Seconds = FPlatformTime::Seconds() - T0;
	UE_LOG(LogTemp, Log, TEXT("AerotelicLattice: %s %s, grid %d x %d x %d (%lld samples, %lld body evaluations), %lld triangles, fraction %.3f, %.2f s"),
		UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(Spec.family)), UTF8_TO_TCHAR(Spec.type.c_str()), Stats.GridX, Stats.GridY, Stats.GridZ,
		Stats.GridNodes, Stats.BodyEvaluations, Stats.Triangles, Stats.AchievedFraction, Stats.Seconds);

	Cached.clear();
	Cached.push_back({"Lattice", std::move(Result.mesh)});
	bValid = true;
	return Cached;
}
