#include "AerotelicBooleanModel.h"
#include "AerotelicEntityActor.h"

#include <stdexcept>

#include "aerotelic/feature_tree.hpp"

TopoDS_Shape FAerotelicBooleanModel::BuildSolid() const
{
	if (Operands.Num() < 2)
	{
		throw std::runtime_error("A boolean operation needs at least two operands.");
	}
	if (!Offered.IsNull())
	{
		TopoDS_Shape Ready = Offered;
		Offered.Nullify();
		return Ready;
	}
	TopoDS_Shape Result;
	for (int32 i = 0; i < Operands.Num(); ++i)
	{
		const FAerotelicBooleanOperand& Op = Operands[i];
		if (!Op.Entity.IsValid())
		{
			throw std::runtime_error("An operand of this boolean operation no longer exists.");
		}
		const TopoDS_Shape Shape = Op.Entity->GetPlacedShape(Op.bMirror);
		if (Shape.IsNull())
		{
			throw std::runtime_error(std::string("Operand '") + TCHAR_TO_UTF8(*Op.Entity->EntityName) + "' has no geometry.");
		}
		if (i == 0)
		{
			Result = Shape;
			continue;
		}
		std::unique_ptr<BRepAlgoAPI_BooleanOperation> BoolOp = aerotelic::BooleanOperation(Operation, Result, Shape);
		Result = BoolOp->Shape();
	}
	Offered = Result;
	return Result;
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
