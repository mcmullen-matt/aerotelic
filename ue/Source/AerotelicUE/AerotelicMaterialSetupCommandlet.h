#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "AerotelicMaterialSetupCommandlet.generated.h"

// Creates the project's surface material asset,
// /Game/Materials/M_AerotelicSurface: BasicShapeMaterial's "Color" vector
// parameter plus "Metallic", "Roughness" and "Specular" scalars wired to
// the matching material outputs, and a Fresnel sheen on the emissive
// output ("FresnelIntensity", "FresnelExponent", "FresnelBaseReflect",
// "FresnelColor"), so the View menu's Surface dialog can
// change every body's finish at runtime. Shaders cannot be compiled in a
// packaged game, so the material has to exist as a saved asset; this
// commandlet (Editor binaries only) is how it is authored without opening
// the Editor. Run via setup_materials.bat (`-run=AerotelicMaterialSetup`);
// idempotent.
UCLASS()
class UAerotelicMaterialSetupCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
