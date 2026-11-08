#pragma once

#include "CoreMinimal.h"

#include "aerotelic/lattice.hpp"

class SWidget;
template <typename OptionType> class SComboBox;

// The controls that edit an aerotelic::LatticeSpec (family, type, phase,
// prism axis, cell size and aspect, how a honeycomb's wall is sized,
// volume fraction, wall thickness, voxels per cell, the three offsets),
// shared by whatever dialog holds a lattice. The state lives in a draft
// the widgets' lambdas share, so the controls work in a modeless window.
struct FAerotelicLatticeSpecDraft
{
	aerotelic::LatticeSpec Spec;

	// The combos' option lists (SComboBox keeps a raw pointer to its list,
	// so these live as long as the controls do) and current picks.
	TArray<TSharedPtr<FString>> Families, Phases, Axes, Sizings, Types;
	TSharedPtr<FString> Family, Phase, Axis, Sizing, Type;
	TSharedPtr<SComboBox<TSharedPtr<FString>>> TypeCombo, PhaseCombo;
	TSharedPtr<SWidget> FractionRow, WallRow, AspectRow, AxisRow, SizingRow;
	// Voxels per cell as the spin box holds it.
	double Resolution = 12.0;

	FAerotelicLatticeSpecDraft();
	// The combos and fields from Spec (after Spec was replaced wholesale).
	void SyncFromSpec();
	// Spec's family, type, phase, axis, sizing and resolution from the combos
	// and the spin box; the numeric fields write Spec directly.
	void SyncToSpec();
	// Enables the rows the family and phase use.
	void UpdateRows();
};

// Builds the controls over Draft; OnChanged fires after every edit (the
// draft already updated).
TSharedRef<SWidget> MakeAerotelicLatticeSpecControls(TSharedRef<FAerotelicLatticeSpecDraft> Draft, FSimpleDelegate OnChanged);
