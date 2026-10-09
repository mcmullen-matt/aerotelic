#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include <memory>

namespace aerotelic { class WingModel; class Airfoil; }

// Slate port of ui/section_panel.py's SectionPanel, embedded as the
// Sections tab shared by the Wing dialog and the Propeller dialog's blade
// (which borrows the whole Wing tab set -- see the approved plan). One row
// per existing section (s, airfoil-name button, remove), plus an "Add
// Section" row. Root (s=0) and tip (s=1) rows cannot be removed, matching
// SectionPanel._remove_section's rule.
class SSectionListWidget : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SSectionListWidget) {}
		// Every section airfoil already in the project (this wing's own plus
		// every other wing/blade's), offered first in the airfoil picker --
		// see section_panel.py's project_airfoils().
		SLATE_ATTRIBUTE(TArray<std::shared_ptr<aerotelic::Airfoil>>, ProjectAirfoils)
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, aerotelic::WingModel* InModel);

private:
	aerotelic::WingModel* Model = nullptr;
	TAttribute<TArray<std::shared_ptr<aerotelic::Airfoil>>> ProjectAirfoilsAttr;
	FSimpleDelegate OnChangedDelegate;

	TSharedPtr<class SVerticalBox> RowsBox;
	TSharedPtr<class SAerotelicNumericEntry> AddEtaEntry;
	double PendingAddEta = 0.5;

	void RebuildRows();
	void PickAirfoil(int32 Index);
	void RemoveSection(int32 Index);
	void AddSection();
};
