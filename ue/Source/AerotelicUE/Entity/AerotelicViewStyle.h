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
