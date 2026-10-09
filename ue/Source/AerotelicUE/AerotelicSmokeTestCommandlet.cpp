#include "AerotelicSmokeTestCommandlet.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/project_yaml.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicSmokeTest, Log, All);

int32 UAerotelicSmokeTestCommandlet::Main(const FString& Params)
{
	try {
		ProjectFile project = LoadProjectYaml("C:\\Users\\Matt\\devel_1\\aerotelic_python\\projects\\p38.yaml");
		UE_LOG(LogAerotelicSmokeTest, Display, TEXT("Loaded p38.yaml: %d entities"), project.entities.size());

		int32 built = 0;
		for (const ProjectEntity& e : project.entities) {
			if (const auto* ws = std::get_if<WingModelSpec>(&e.model)) {
				WingModel wm(*ws);
				TopoDS_Shape shape = wm.BuildSolid(20, true, std::nullopt);
				GProp_GProps props;
				BRepGProp::VolumeProperties(shape, props);
				UE_LOG(LogAerotelicSmokeTest, Display, TEXT("  wing '%s' volume=%f"), *FString(e.name.c_str()), props.Mass());
				built++;
			}
		}
		UE_LOG(LogAerotelicSmokeTest, Display, TEXT("SmokeTest: PASS (%d wings built)"), built);
		return built > 0 ? 0 : 1;
	} catch (const std::exception& ex) {
		UE_LOG(LogAerotelicSmokeTest, Error, TEXT("SmokeTest: FAIL: %s"), *FString(ex.what()));
		return 1;
	}
}
