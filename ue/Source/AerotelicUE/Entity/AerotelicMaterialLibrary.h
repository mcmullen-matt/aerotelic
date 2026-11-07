#pragma once

#include "CoreMinimal.h"

#include "aerotelic/materials.hpp"

// The materials a body can be made of: the presets the application ships
// with (materials.hpp) followed by the user's own, kept in one YAML file
// per user (%LOCALAPPDATA%\MS2 Aerodynamics\WingDesigner\materials.yaml,
// beside the licence) so a material defined once is offered in every
// project. Model > Material Properties shows the whole list as a table
// and edits the user's part; the Scene tree's Material Properties on an
// entity picks one. An entity keeps a COPY of the material it was given
// (name and figures, written to the project file), so editing a library
// material afterwards changes nothing already assigned until it is picked
// again -- and a project file from another machine opens with the
// materials it carries, library or no library.
namespace AerotelicMaterialLibrary
{
	// The presets, then the user's materials, in the order they are offered.
	TArray<aerotelic::Material> All();
	// The user's materials alone (loaded from the file on first use).
	const TArray<aerotelic::Material>& Custom();
	// The material of that name, preset or user's; null if there is none.
	const aerotelic::Material* Find(const std::string& Name);
	// The library material `M` matches by name AND by the four figures
	// (aerotelic::SameMaterial), else null: an entity whose copy was edited,
	// or whose material was later changed in the library, matches nothing.
	const aerotelic::Material* FindMatching(const aerotelic::Material& M);

	// Replaces the user's materials and writes the file. False, with the
	// reason, when a material is incomplete (aerotelic::MaterialProblem), a
	// name is blank, repeated or a preset's, or the file cannot be written
	// -- and then nothing changes.
	bool SetCustom(const TArray<aerotelic::Material>& Materials, FString& OutError);

	// Where the user's materials are kept.
	FString FilePath();
	// Tests: keep them in this file instead (empty: the user's own again).
	// Reloads from the new path at once.
	void SetFilePathOverride(const FString& Path);

	// Fired after SetCustom changes the list: open dialogs refresh.
	FSimpleMulticastDelegate& OnChanged();
}
