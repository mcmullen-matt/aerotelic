#include "AerotelicMaterialSetupCommandlet.h"

#if WITH_EDITOR
#include "Factories/MaterialFactoryNew.h"
#include "MaterialEditingLibrary.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"
#include "Misc/FeedbackContext.h"
#endif

// Last: it pulls in OCCT, whose Handle() macro breaks any engine header
// (IoDispatcher.h has a member called Handle) included after it.
#include "Entity/AerotelicEntityActor.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicMaterialSetup, Log, All);

int32 UAerotelicMaterialSetupCommandlet::Main(const FString& Params)
{
#if WITH_EDITOR
	const FString PackageName = AAerotelicEntityActor::SurfaceMaterialPackage();
	const FString AssetName = FPackageName::GetLongPackageAssetName(PackageName);

	UPackage* Package = CreatePackage(*PackageName);
	Package->FullyLoad();

	UMaterialFactoryNew* Factory = NewObject<UMaterialFactoryNew>();
	UMaterial* Material = Cast<UMaterial>(Factory->FactoryCreateNew(UMaterial::StaticClass(), Package, *AssetName, RF_Public | RF_Standalone, nullptr, GWarn));
	if (Material == nullptr)
	{
		UE_LOG(LogAerotelicMaterialSetup, Error, TEXT("RESULT: could not create %s"), *PackageName);
		return 1;
	}
	// The mirrored copies are improper transforms of their sources; two
	// sided saves worrying about which winding faces the camera.
	Material->TwoSided = true;

	auto* Color = Cast<UMaterialExpressionVectorParameter>(UMaterialEditingLibrary::CreateMaterialExpression(Material, UMaterialExpressionVectorParameter::StaticClass(), -400, -200));
	Color->ParameterName = TEXT("Color");
	Color->DefaultValue = FLinearColor(0.72f, 0.74f, 0.78f, 1.0f);
	UMaterialEditingLibrary::ConnectMaterialProperty(Color, TEXT(""), MP_BaseColor);

	struct FScalar { const TCHAR* Name; float Default; EMaterialProperty Property; int32 Y; };
	const FScalar Scalars[] = {
		{ TEXT("Metallic"),  0.0f, MP_Metallic,  0 },
		{ TEXT("Specular"),  0.5f, MP_Specular,  150 },
		{ TEXT("Roughness"), 0.5f, MP_Roughness, 300 },
	};
	for (const FScalar& S : Scalars)
	{
		auto* Param = Cast<UMaterialExpressionScalarParameter>(UMaterialEditingLibrary::CreateMaterialExpression(Material, UMaterialExpressionScalarParameter::StaticClass(), -400, S.Y));
		Param->ParameterName = S.Name;
		Param->DefaultValue = S.Default;
		UMaterialEditingLibrary::ConnectMaterialProperty(Param, TEXT(""), S.Property);
	}

	UMaterialEditingLibrary::RecompileMaterial(Material);
	FAssetRegistryModule::AssetCreated(Material);
	Package->MarkPackageDirty();

	const FString FileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;
	const bool bSaved = UPackage::SavePackage(Package, Material, *FileName, SaveArgs);
	UE_LOG(LogAerotelicMaterialSetup, Display, TEXT("RESULT: %s %s"), bSaved ? TEXT("saved") : TEXT("FAILED to save"), *FileName);
	return bSaved ? 0 : 1;
#else
	UE_LOG(LogAerotelicMaterialSetup, Error, TEXT("RESULT: FAILED -- this commandlet needs the Editor binaries"));
	return 1;
#endif
}
