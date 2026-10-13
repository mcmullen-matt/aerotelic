#include "AerotelicBooleanModel.h"
#include "AerotelicEntityActor.h"

#include <stdexcept>
#include <string>

#include "aerotelic/mesh_boolean.hpp"

namespace
{
	// The operands folded left to right through the mesh boolean: (A op B)
	// op C, as feature_tree.py's fold_boolean built its chain. An operand
	// with several bodies (a propeller: spinner and blades) is first fused
	// into one. Errors carry the operand names.
	aerotelic::TriMesh FoldMeshes(const TArray<FAerotelicBooleanOperand>& Operands, const std::string& Operation, double Tolerance)
	{
		const aerotelic::MeshBooleanOp Op = aerotelic::ParseMeshBooleanOp(Operation);
		aerotelic::TriMesh Result;
		FString ResultLabel;
		for (int32 i = 0; i < Operands.Num(); ++i)
		{
			const FAerotelicBooleanOperand& Operand = Operands[i];
			if (!Operand.Entity.IsValid())
			{
				throw std::runtime_error("An operand of this boolean operation no longer exists.");
			}
			const FString Label = Operand.Entity->EntityName + (Operand.bMirror ? TEXT(" (mirror)") : TEXT(""));
			std::vector<aerotelic::NamedMesh> Bodies = Operand.Entity->GetPlacedMeshes(Operand.bMirror, Tolerance);
			aerotelic::TriMesh Body;
			for (const aerotelic::NamedMesh& M : Bodies)
			{
				if (M.mesh.IsEmpty()) continue;
				if (!M.mesh.IsClosedManifold() || !(M.mesh.SignedVolume() > 0.0))
				{
					throw std::runtime_error(std::string("Operand '") + TCHAR_TO_UTF8(*Label) + "' / " + M.name + " is not a closed, outward surface.");
				}
				if (Body.IsEmpty())
				{
					Body = M.mesh;
					continue;
				}
				try
				{
					Body = aerotelic::MeshBoolean(Body, M.mesh, aerotelic::MeshBooleanOp::Fuse);
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error(std::string("Fusing the bodies of '") + TCHAR_TO_UTF8(*Label) + "': " + e.what());
				}
			}
			if (Body.IsEmpty())
			{
				throw std::runtime_error(std::string("Operand '") + TCHAR_TO_UTF8(*Label) + "' has no geometry.");
			}
			if (i == 0)
			{
				Result = std::move(Body);
				ResultLabel = Label;
				continue;
			}
			try
			{
				Result = aerotelic::MeshBoolean(Result, Body, Op);
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error(std::string(aerotelic::MeshBooleanOpName(Op)) + " of '" + TCHAR_TO_UTF8(*ResultLabel) + "' with '" + TCHAR_TO_UTF8(*Label) + "': " + e.what());
			}
			ResultLabel = Label;
			if (Result.IsEmpty())
			{
				throw std::runtime_error(std::string("The ") + aerotelic::MeshBooleanOpName(Op) + " of '" + TCHAR_TO_UTF8(*ResultLabel) + "' left nothing.");
			}
		}
		return Result;
	}
}

TopoDS_Shape FAerotelicBooleanModel::BuildSolid() const
{
	throw std::runtime_error("A boolean result is a triangle mesh, not a B-rep solid: export it as STL, or export its operands to STEP and combine them downstream.");
}

std::vector<aerotelic::NamedMesh> FAerotelicBooleanModel::BuildMeshes(double Tolerance) const
{
	if (Operands.Num() < 2)
	{
		throw std::runtime_error("A boolean operation needs at least two operands.");
	}
	return {{"Main", FoldMeshes(Operands, Operation, Tolerance)}};
}

bool FAerotelicBooleanModel::DependsOn(const AAerotelicEntityActor* Entity) const
{
	for (const FAerotelicBooleanOperand& Op : Operands)
	{
		if (!Op.Entity.IsValid())
		{
			continue;
		}
		if (Op.Entity.Get() == Entity)
		{
			return true;
		}
		if (Op.Entity->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean)
		{
			if (const FAerotelicBooleanModel* Inner = static_cast<const FAerotelicBooleanModel*>(Op.Entity->GetModel()))
			{
				if (Inner->DependsOn(Entity))
				{
					return true;
				}
			}
		}
	}
	return false;
}
