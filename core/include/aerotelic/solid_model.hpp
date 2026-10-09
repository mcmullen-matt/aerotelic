#pragma once

#include <TopoDS_Shape.hxx>

namespace aerotelic {

// Common interface every entity model (PrimitiveModel, WingModel, ...)
// implements so FeatureTree's ComponentSourceFeature/EntityShapeFeature
// can hold one generically -- the C++ equivalent of core/feature_tree.py's
// duck-typed `model.build_solid()`.
class ISolidModel {
public:
	virtual ~ISolidModel() = default;
	virtual TopoDS_Shape BuildSolid() const = 0;
};

} // namespace aerotelic
