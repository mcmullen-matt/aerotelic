#pragma once

#include "CoreMinimal.h"

// How the scene is *viewed* (not what it is): none of this is saved with
// the project, exactly as WingDesigner's renderer state is not.

// The finish every body is drawn with -- View -> Surface... sets it for the
// whole scene. Metallic 0..1; Specular 0..1 (how strong the highlights
// are, "glossy"); Roughness 0..1 (how blurred the reflections are, 0 is a
// mirror polish). The defaults are the surface material's own.
struct FAerotelicSurfaceStyle
{
	float Metallic = 0.0f;
	float Specular = 0.5f;
	float Roughness = 0.5f;
	// The Fresnel sheen the surface material adds on top of the lit
	// shading (M_AerotelicSurface's FresnelIntensity / FresnelExponent):
	// grazing-angle pixels are brightened, so a silhouette and a tight
	// curvature read as a bright rim rather than fading into the
	// background. Intensity 0..1 scales it and 0 switches it off;
	// Exponent 1..8 is the falloff, 1 a broad wash over the whole body
	// and 8 a thin edge line. Unlike the three physical parameters above
	// this is not a material property of the part: it is a reading aid,
	// the shading trick a CAD viewport uses to show form on an untextured
	// monochrome surface.
	float FresnelIntensity = 0.35f;
	float FresnelExponent = 3.0f;
};

// The key light's shadow settings -- View -> Shadows... sets them for the
// scene. The studio rig (AerotelicGameMode::SpawnLighting) has one
// shadow-casting directional light, movable, so its shadows are cascaded
// shadow maps: a depth map rendered from the light, compared against each
// pixel's depth. The comparison needs a bias, or a surface shadows itself
// in stripes ("shadow acne") where it is nearly parallel to the light;
// too much bias and the shadow slides away from its caster
// ("peter-panning"), which on a thin wing looks like a detached shadow.
//
//   DepthBias  a constant offset (ULightComponent::ShadowBias), the same
//              everywhere.
//   SlopeBias  scales the offset with how steeply the surface is inclined
//              to the light (ULightComponent::ShadowSlopeBias). This is
//              the normal-facing bias: a face edge-on to the light needs
//              far more offset than one facing it, and this is the term
//              that gives it, so it is the one to reach for against acne
//              on a fuselage's flank.
//
// The project renders conventional shadow maps: virtual shadow maps are
// off (`r.Shadow.Virtual.Enable` defaults to 0 and Config/DefaultEngine.ini
// does not set it), so the engine's `r.Shadow.Virtual.NormalBias` is NOT
// the knob here and would do nothing until VSMs are turned on.
struct FAerotelicShadowStyle
{
	bool bCastShadows = true;
	float DepthBias = 0.5f;
	float SlopeBias = 0.5f;
};

// The Style dropdown (occ_viewer.py RENDER_STYLES). WingDesigner keeps the
// shaded display mode in all three and only changes the shading aspect:
// "Shaded with Edges" draws the *triangulation's* edges in near-black over
// the surface, "Wireframe" draws those edges alone in the body's colour
// (hollow interior). Rendered here with a second, wireframe-material copy
// of each body's mesh (M_AerotelicWire) over or instead of the surface.
enum class EAerotelicRenderStyle : uint8
{
	Shaded,
	ShadedEdges,
	Wireframe,
};

// The Section dropdown (core/view_gizmo.py SectionPlane): one clip plane
// normal to a world axis, applied to every body (never to the ground or
// the gizmo), positioned by dragging the arrow handle in the view. The
// positive side is kept unless flipped.
enum class EAerotelicSectionAxis : uint8
{
	None,
	X,
	Y,
	Z,
};

struct FAerotelicSectionPlane
{
	EAerotelicSectionAxis Axis = EAerotelicSectionAxis::None;
	bool bFlipped = false;
	// Signed distance of the plane from the origin along the axis.
	double Position = 0.0;

	bool IsActive() const { return Axis != EAerotelicSectionAxis::None; }

	static FVector AxisUnit(EAerotelicSectionAxis InAxis)
	{
		switch (InAxis)
		{
		case EAerotelicSectionAxis::X: return FVector(1, 0, 0);
		case EAerotelicSectionAxis::Y: return FVector(0, 1, 0);
		case EAerotelicSectionAxis::Z: return FVector(0, 0, 1);
		default:                       return FVector::ZeroVector;
		}
	}
	static int32 AxisIndex(EAerotelicSectionAxis InAxis)
	{
		switch (InAxis)
		{
		case EAerotelicSectionAxis::X: return 0;
		case EAerotelicSectionAxis::Y: return 1;
		case EAerotelicSectionAxis::Z: return 2;
		default:                       return -1;
		}
	}

	FVector Unit() const { return AxisUnit(Axis); }
	// A point on the plane.
	FVector Point() const { return Unit() * Position; }
	// Normal pointing into the half-space that stays visible
	// (view_gizmo.py section_kept_normal). Zero when inactive, which the
	// materials read as "keep everything".
	FVector KeptNormal() const { return IsActive() ? Unit() * (bFlipped ? -1.0 : 1.0) : FVector::ZeroVector; }
};
