#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/units.hpp"

class AAerotelicEntityActor;

// THE SKIN A BOOLEAN RESULT INHERITS (2026-11-20, the owner's rule).
//
// A boolean result owns no structure. Every face of it is a piece of one
// operand body's surface (NamedMesh::face_source, exact: the mesh boolean
// cuts triangles, it never makes one out of two bodies), and that face
// keeps THAT body's skin -- its material and thickness -- fused to a
// different skin or not. A cut's pocket wall is the cutter's surface and
// keeps the cutter's skin. There is no override on the result: to change
// what a result is made of, change the operand. The operands' generated
// layers (ribs, spars, frames, stringers) are taken over the same way
// (AerotelicDocumentSubsystem::GenerateBodyStructure, since 2026-11-10).
//
// This resolves a result body's face sources against the scene: which
// operand body each is, what structure it carries as it stands, which
// faces and how much area. The mass report weighs each source's faces by
// its skin; the structural model builder makes a region of each; the Scene
// tree describes them. A source whose operand has no usable skin is a
// problem named after the OPERAND body, which is where it is fixed.
struct FAerotelicSkinSource
{
	aerotelic::MeshFaceSource Source;
	// "Wing / Main", "Wing (mirror) / Main" (the body omitted when the
	// operand has one body).
	FString Label;
	// The operand, when it is in the scene.
	TWeakObjectPtr<AAerotelicEntityActor> Entity;
	// True when the operand and its body were found; Structure is then the
	// operand body's as it stands (possibly unset).
	bool bKnown = false;
	aerotelic::BodyStructure Structure;
	// The triangles of the result body's mesh that are this source's, in
	// the mesh's order, and their area (the mesh's units).
	std::vector<int> Faces;
	double Area = 0.0;
	// The operand body's whole surface area in the same units as the mesh
	// (its own grid, placed): an asserted mass on the operand is spread
	// over its surface, and these faces carry their share of it.
	double OperandArea = 0.0;

	// This source's faces as a mesh of their own, in Faces order, over the
	// vertices they use.
	aerotelic::TriMesh SubMesh(const aerotelic::TriMesh& Body) const;
	// The share of the operand's surface these faces are (1 when unknown).
	double Share() const { return OperandArea > 0.0 ? FMath::Min(1.0, Area / OperandArea) : 1.0; }
	// Weighable: a material with a density and a skin thickness, or an
	// asserted mass.
	bool HasSkin() const { return bKnown && (Structure.IsSkin() || Structure.asserted_mass.has_value()); }
};

// The sources of a result body's mesh (placed or not: only the triangle
// order matters), resolved against the document's entities by name, in
// the order of the mesh's `sources`. Empty for a mesh without face
// sources (a parameterised body).
TArray<FAerotelicSkinSource> AerotelicSkinSourcesOf(const aerotelic::NamedMesh& Mesh);

// The same for a boolean entity's body by role, on the body's own grid.
// Empty when the entity is not a boolean result or has no such body.
TArray<FAerotelicSkinSource> AerotelicSkinSourcesOf(const AAerotelicEntityActor* Boolean, const FString& Role);

// Why the result body cannot be weighed or analysed, one sentence per
// source at fault, each naming the operand body to fix; empty when every
// face has a skin. With bStructural the skin's material must also be one
// the structural analysis can use (Material::IsStructural), and an
// asserted mass or a solid operand does not do.
TArray<FString> AerotelicSkinSourceProblems(const TArray<FAerotelicSkinSource>& Sources, bool bStructural);

// "skin inherited from the operands: Wing / Main -- Aluminium 2024-T3
// sheet 1 mm (62% of the surface); Fuselage / Main -- no material" -- one
// clause per source, in the project's units.
FString DescribeAerotelicSkinSources(const TArray<FAerotelicSkinSource>& Sources, const aerotelic::UnitSystem& Units);

// For the Scene tree: the result body's inherited skin in one line, or
// why there is none yet.
FString DescribeAerotelicInheritedSkin(const AAerotelicEntityActor* Boolean, const FString& Role, const aerotelic::UnitSystem& Units);
