#include "AerotelicMaterialSetupCommandlet.h"

#if WITH_EDITOR
#include "Factories/MaterialFactoryNew.h"
#include "MaterialEditingLibrary.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionWorldPosition.h"
#include "Materials/MaterialExpressionCameraPositionWS.h"
#include "Materials/MaterialExpressionSubtract.h"
#include "Materials/MaterialExpressionDotProduct.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionNormalize.h"
#include "Materials/MaterialExpressionConstant.h"
#include "Materials/MaterialExpressionIf.h"
#include "Materials/MaterialExpressionTwoSidedSign.h"
#include "Materials/MaterialExpressionFresnel.h"
#include "Materials/MaterialExpressionVertexColor.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"
#include "Misc/FeedbackContext.h"
#endif

#include "Standalone/AerotelicSectionGizmo.h"
#include "Standalone/AerotelicColoredMeshActor.h"
// Last: it pulls in OCCT, whose Handle() macro breaks any engine header
// (IoDispatcher.h has a member called Handle) included after it.
#include "Entity/AerotelicEntityActor.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicMaterialSetup, Log, All);

#if WITH_EDITOR
#include "UObject/UObjectBaseUtility.h"
namespace
{
	template <typename T>
	T* Node(UMaterial* Material, int32 X, int32 Y)
	{
		return Cast<T>(UMaterialEditingLibrary::CreateMaterialExpression(Material, T::StaticClass(), X, Y));
	}

	// Input names are the *pin* names the material graph shows, which for
	// the If node are the compact "A > B" / "A == B" / "A < B".
	void Link(UMaterialExpression* From, UMaterialExpression* To, const TCHAR* Input)
	{
		if (!UMaterialEditingLibrary::ConnectMaterialExpressions(From, TEXT(""), To, Input))
		{
			UE_LOG(LogAerotelicMaterialSetup, Error, TEXT("could not connect %s -> %s.%s"), *GetNameSafe(From), *GetNameSafe(To), Input);
		}
	}

	UMaterialExpressionVectorParameter* VectorParam(UMaterial* Material, const TCHAR* Name, const FLinearColor& Default, int32 X, int32 Y)
	{
		auto* P = Node<UMaterialExpressionVectorParameter>(Material, X, Y);
		P->ParameterName = Name;
		P->DefaultValue = Default;
		return P;
	}

	UMaterialExpressionScalarParameter* ScalarParam(UMaterial* Material, const TCHAR* Name, float Default, int32 X, int32 Y)
	{
		auto* P = Node<UMaterialExpressionScalarParameter>(Material, X, Y);
		P->ParameterName = Name;
		P->DefaultValue = Default;
		return P;
	}

	UMaterialExpressionConstant* Constant(UMaterial* Material, float Value, int32 X, int32 Y)
	{
		auto* C = Node<UMaterialExpressionConstant>(Material, X, Y);
		C->R = Value;
		return C;
	}

	// The section plane (view_gizmo.py SectionPlane, one Graphic3d_ClipPlane
	// per body): the pixel is kept where dot(WorldPos - ClipPoint,
	// ClipNormal) >= 0. A zero ClipNormal (the default, and what "None"
	// sets) keeps everything. Masked blend mode, so this is a hard cut.
	void AddClipPlane(UMaterial* Material, int32 X, int32 Y)
	{
		auto* WorldPos = Node<UMaterialExpressionWorldPosition>(Material, X, Y);
		auto* ClipPoint = VectorParam(Material, TEXT("ClipPoint"), FLinearColor(0, 0, 0, 0), X, Y + 150);
		auto* ClipNormal = VectorParam(Material, TEXT("ClipNormal"), FLinearColor(0, 0, 0, 0), X, Y + 300);
		auto* Diff = Node<UMaterialExpressionSubtract>(Material, X + 250, Y + 50);
		Link(WorldPos, Diff, TEXT("A"));
		Link(ClipPoint, Diff, TEXT("B"));
		auto* Dot = Node<UMaterialExpressionDotProduct>(Material, X + 450, Y + 100);
		Link(Diff, Dot, TEXT("A"));
		Link(ClipNormal, Dot, TEXT("B"));
		auto* Keep = Node<UMaterialExpressionIf>(Material, X + 650, Y + 100);
		Keep->ConstB = 0.0f;
		Link(Dot, Keep, TEXT("A"));
		Link(Constant(Material, 1.0f, X + 450, Y + 250), Keep, TEXT("A > B"));
		Link(Constant(Material, 1.0f, X + 450, Y + 320), Keep, TEXT("A == B"));
		Link(Constant(Material, 0.0f, X + 450, Y + 390), Keep, TEXT("A < B"));
		UMaterialEditingLibrary::ConnectMaterialProperty(Keep, TEXT(""), MP_OpacityMask);
		Material->BlendMode = BLEND_Masked;
		Material->OpacityMaskClipValue = 0.5f;
	}

	// A Fresnel sheen added to the emissive output: the engine's Fresnel
	// node gives the grazing-angle factor -- 0 where the surface faces the
	// eye, rising to 1 at the silhouette -- and it is scaled, tinted and
	// added on top of the lit shading. On an untextured monochrome body
	// this is what makes form read: the silhouette and any tight curvature
	// brighten, so a fuselage no longer fades into the backdrop and a
	// leading edge separates from the skin behind it.
	//
	// Four parameters, FresnelIntensity and FresnelExponent driven live by
	// View -> Surface... (FAerotelicSurfaceStyle):
	//   FresnelIntensity   how bright the rim is; 0 switches the sheen off
	//                      and leaves the plain lit shading.
	//   FresnelExponent    the falloff; 1 washes the whole body, 8 is a
	//                      thin edge line. The node's own Exponent default
	//                      matches, for a material instance that does not
	//                      set it.
	//   FresnelBaseReflect lifts the sheen everywhere, not just at the rim
	//                      (the node's base reflect fraction); 0 keeps the
	//                      front faces exactly as they were.
	//   FresnelColor       the rim's tint, a little cooler than white so it
	//                      reads as sky light rather than a white halo.
	// Emissive rather than a light: it needs no scene light to work, it
	// cannot blow out the lit response, and the clip plane still discards
	// the pixel, so a sectioned body does not glow along the cut.
	void AddFresnelSheen(UMaterial* Material, int32 X, int32 Y)
	{
		auto* Fresnel = Node<UMaterialExpressionFresnel>(Material, X + 500, Y + 60);
		Fresnel->Exponent = 3.0f;
		Fresnel->BaseReflectFraction = 0.0f;
		Link(ScalarParam(Material, TEXT("FresnelExponent"), 3.0f, X, Y), Fresnel, TEXT("ExponentIn"));
		Link(ScalarParam(Material, TEXT("FresnelBaseReflect"), 0.0f, X, Y + 150), Fresnel, TEXT("BaseReflectFractionIn"));

		auto* Scaled = Node<UMaterialExpressionMultiply>(Material, X + 750, Y + 80);
		Link(Fresnel, Scaled, TEXT("A"));
		Link(ScalarParam(Material, TEXT("FresnelIntensity"), 0.35f, X + 500, Y + 230), Scaled, TEXT("B"));

		auto* Tinted = Node<UMaterialExpressionMultiply>(Material, X + 950, Y + 130);
		Link(Scaled, Tinted, TEXT("A"));
		Link(VectorParam(Material, TEXT("FresnelColor"), FLinearColor(0.90f, 0.95f, 1.0f, 1.0f), X + 500, Y + 350), Tinted, TEXT("B"));
		UMaterialEditingLibrary::ConnectMaterialProperty(Tinted, TEXT(""), MP_EmissiveColor);
	}

	bool SaveMaterial(UMaterial* Material, UPackage* Package, const FString& PackageName)
	{
		UMaterialEditingLibrary::RecompileMaterial(Material);
		FAssetRegistryModule::AssetCreated(Material);
		Package->MarkPackageDirty();

		const FString FileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		const bool bSaved = UPackage::SavePackage(Package, Material, *FileName, SaveArgs);
		UE_LOG(LogAerotelicMaterialSetup, Display, TEXT("RESULT: %s %s"), bSaved ? TEXT("saved") : TEXT("FAILED to save"), *FileName);
		return bSaved;
	}

	UMaterial* NewMaterial(const FString& PackageName, UPackage*& OutPackage)
	{
		const FString AssetName = FPackageName::GetLongPackageAssetName(PackageName);
		OutPackage = CreatePackage(*PackageName);
		OutPackage->FullyLoad();
		UMaterialFactoryNew* Factory = NewObject<UMaterialFactoryNew>();
		UMaterial* Material = Cast<UMaterial>(Factory->FactoryCreateNew(UMaterial::StaticClass(), OutPackage, *AssetName, RF_Public | RF_Standalone, nullptr, GWarn));
		if (Material == nullptr)
		{
			UE_LOG(LogAerotelicMaterialSetup, Error, TEXT("RESULT: could not create %s"), *PackageName);
		}
		return Material;
	}

	// M_AerotelicSurface: the lit body surface. Color, Metallic, Specular
	// and Roughness parameters (View -> Surface...), a Fresnel sheen on the
	// emissive output (AddFresnelSheen), two sided, the section clip, and a
	// darker tint on back faces so a cut body reads as a shell seen from
	// inside (WingDesigner caps the cut instead; the engine has no capping,
	// so the inside of the far wall is what shows).
	bool AuthorSurface()
	{
		const FString PackageName = AAerotelicEntityActor::SurfaceMaterialPackage();
		UPackage* Package = nullptr;
		UMaterial* Material = NewMaterial(PackageName, Package);
		if (Material == nullptr)
		{
			return false;
		}
		Material->TwoSided = true;

		auto* Color = VectorParam(Material, TEXT("Color"), FLinearColor(0.72f, 0.74f, 0.78f, 1.0f), -700, -300);
		auto* Inside = Node<UMaterialExpressionMultiply>(Material, -450, -200);
		Link(Color, Inside, TEXT("A"));
		Inside->ConstB = 0.55f;
		auto* Facing = Node<UMaterialExpressionTwoSidedSign>(Material, -700, -100);
		auto* Pick = Node<UMaterialExpressionIf>(Material, -250, -250);
		Pick->ConstB = 0.0f;
		Link(Facing, Pick, TEXT("A"));
		Link(Color, Pick, TEXT("A > B"));
		Link(Color, Pick, TEXT("A == B"));
		Link(Inside, Pick, TEXT("A < B"));
		UMaterialEditingLibrary::ConnectMaterialProperty(Pick, TEXT(""), MP_BaseColor);

		struct FScalar { const TCHAR* Name; float Default; EMaterialProperty Property; int32 Y; };
		const FScalar Scalars[] = {
			{ TEXT("Metallic"),  0.0f, MP_Metallic,  0 },
			{ TEXT("Specular"),  0.5f, MP_Specular,  150 },
			{ TEXT("Roughness"), 0.5f, MP_Roughness, 300 },
		};
		for (const FScalar& S : Scalars)
		{
			auto* Param = ScalarParam(Material, S.Name, S.Default, -400, S.Y);
			UMaterialEditingLibrary::ConnectMaterialProperty(Param, TEXT(""), S.Property);
		}

		AddFresnelSheen(Material, -1700, -950);
		AddClipPlane(Material, -1000, 500);
		return SaveMaterial(Material, Package, PackageName);
	}

	// M_AerotelicWire: the triangulation's edges (occ_viewer.py draws them
	// with SetDrawEdges on the shading aspect; here the engine's wireframe
	// rasterisation of the same triangles). Unlit, in the Color parameter,
	// clipped like the surface, and nudged a little towards the camera so
	// the lines sit on top of the coplanar surface instead of fighting it
	// for depth.
	bool AuthorWire()
	{
		const FString PackageName = AAerotelicEntityActor::WireMaterialPackage();
		UPackage* Package = nullptr;
		UMaterial* Material = NewMaterial(PackageName, Package);
		if (Material == nullptr)
		{
			return false;
		}
		Material->TwoSided = true;
		Material->Wireframe = true;
		Material->SetShadingModel(MSM_Unlit);

		auto* Color = VectorParam(Material, TEXT("Color"), FLinearColor(0.02f, 0.02f, 0.03f, 1.0f), -400, -200);
		UMaterialEditingLibrary::ConnectMaterialProperty(Color, TEXT(""), MP_EmissiveColor);

		auto* Camera = Node<UMaterialExpressionCameraPositionWS>(Material, -900, 100);
		auto* WorldPos = Node<UMaterialExpressionWorldPosition>(Material, -900, 200);
		auto* ToCamera = Node<UMaterialExpressionSubtract>(Material, -700, 150);
		Link(Camera, ToCamera, TEXT("A"));
		Link(WorldPos, ToCamera, TEXT("B"));
		auto* Dir = Node<UMaterialExpressionNormalize>(Material, -550, 150);
		Link(ToCamera, Dir, TEXT("VectorInput"));
		auto* Offset = ScalarParam(Material, TEXT("EdgeOffset"), 0.02f, -550, 250);
		auto* Nudge = Node<UMaterialExpressionMultiply>(Material, -350, 180);
		Link(Dir, Nudge, TEXT("A"));
		Link(Offset, Nudge, TEXT("B"));
		UMaterialEditingLibrary::ConnectMaterialProperty(Nudge, TEXT(""), MP_WorldPositionOffset);

		AddClipPlane(Material, -1000, 500);
		return SaveMaterial(Material, Package, PackageName);
	}

	// M_AerotelicGizmo / M_AerotelicGizmoTop: the section gizmo's amber,
	// unlit. The frame is depth-tested like anything else; the arrow handle
	// draws on top of the bodies (view_gizmo.py puts it in the Topmost
	// z-layer so it stays reachable inside a body), which in the engine
	// means a translucent material with the depth test off.
	bool AuthorGizmo(const FString& PackageName, bool bOnTop)
	{
		UPackage* Package = nullptr;
		UMaterial* Material = NewMaterial(PackageName, Package);
		if (Material == nullptr)
		{
			return false;
		}
		Material->SetShadingModel(MSM_Unlit);
		auto* Color = VectorParam(Material, TEXT("Color"), FLinearColor(1.0f, 0.7f, 0.1f, 1.0f), -400, -200);
		UMaterialEditingLibrary::ConnectMaterialProperty(Color, TEXT(""), MP_EmissiveColor);
		if (bOnTop)
		{
			Material->BlendMode = BLEND_Translucent;
			Material->bDisableDepthTest = true;
			UMaterialEditingLibrary::ConnectMaterialProperty(Constant(Material, 1.0f, -400, 0), TEXT(""), MP_Opacity);
		}
		return SaveMaterial(Material, Package, PackageName);
	}

	// M_AerotelicVertexColor: a field painted on a surface (the panel
	// method's Cp). Lit like the bodies, two sided, the base colour straight
	// from the vertex colour (which AAerotelicColoredMeshActor stores
	// linearised), a matte finish so the colours read as colours and not as
	// highlights, and the same darker back faces as the surface material.
	bool AuthorVertexColor()
	{
		const FString PackageName = AAerotelicColoredMeshActor::VertexColorMaterialPackage();
		UPackage* Package = nullptr;
		UMaterial* Material = NewMaterial(PackageName, Package);
		if (Material == nullptr)
		{
			return false;
		}
		Material->TwoSided = true;

		auto* Color = Node<UMaterialExpressionVertexColor>(Material, -700, -300);
		auto* Inside = Node<UMaterialExpressionMultiply>(Material, -450, -200);
		Link(Color, Inside, TEXT("A"));
		Inside->ConstB = 0.55f;
		auto* Facing = Node<UMaterialExpressionTwoSidedSign>(Material, -700, -100);
		auto* Pick = Node<UMaterialExpressionIf>(Material, -250, -250);
		Pick->ConstB = 0.0f;
		Link(Facing, Pick, TEXT("A"));
		Link(Color, Pick, TEXT("A > B"));
		Link(Color, Pick, TEXT("A == B"));
		Link(Inside, Pick, TEXT("A < B"));
		UMaterialEditingLibrary::ConnectMaterialProperty(Pick, TEXT(""), MP_BaseColor);

		UMaterialEditingLibrary::ConnectMaterialProperty(Constant(Material, 0.0f, -400, 0), TEXT(""), MP_Metallic);
		UMaterialEditingLibrary::ConnectMaterialProperty(Constant(Material, 0.25f, -400, 150), TEXT(""), MP_Specular);
		UMaterialEditingLibrary::ConnectMaterialProperty(Constant(Material, 0.7f, -400, 300), TEXT(""), MP_Roughness);
		return SaveMaterial(Material, Package, PackageName);
	}
}
#endif

int32 UAerotelicMaterialSetupCommandlet::Main(const FString& Params)
{
#if WITH_EDITOR
	bool bOk = AuthorSurface();
	bOk = AuthorWire() && bOk;
	bOk = AuthorVertexColor() && bOk;
	bOk = AuthorGizmo(AAerotelicSectionGizmo::FrameMaterialPackage(), false) && bOk;
	bOk = AuthorGizmo(AAerotelicSectionGizmo::HandleMaterialPackage(), true) && bOk;
	return bOk ? 0 : 1;
#else
	UE_LOG(LogAerotelicMaterialSetup, Error, TEXT("RESULT: FAILED -- this commandlet needs the Editor binaries"));
	return 1;
#endif
}
