#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

#include "aerotelic/materials.hpp"

template <typename OptionType> class SComboBox;

DECLARE_DELEGATE_OneParam(FOnAerotelicMaterialPicked, const aerotelic::Material&);

// One list to pick a material from: nothing (when allowed), every material
// of the library (the presets and the user's own, Model > Material
// Properties), and the figures the caller already holds when they match
// no library material (an edited copy, a material since changed or
// removed, a file from another machine). The caller keeps a COPY of what
// is picked -- the name and the figures, as the project file does -- so a
// later edit in the library changes nothing already assigned. Follows the
// library as it changes.
class SAerotelicMaterialPicker : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicMaterialPicker) : _AllowNone(true) {}
		// The material in force (re-read as the list is shown).
		SLATE_ATTRIBUTE(aerotelic::Material, Current)
		// Offer "Not set" (a material without a density) as the first entry.
		SLATE_ARGUMENT(bool, AllowNone)
		SLATE_EVENT(FOnAerotelicMaterialPicked, OnPicked)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual ~SAerotelicMaterialPicker();

private:
	struct FChoice
	{
		bool bNone = false;
		bool bOwn = false;
		aerotelic::Material M;
		FString Label;
	};
	TArray<TSharedPtr<FChoice>> Choices;
	TSharedPtr<SComboBox<TSharedPtr<FChoice>>> Combo;
	TAttribute<aerotelic::Material> Current;
	FOnAerotelicMaterialPicked OnPicked;
	bool bAllowNone = true;
	FDelegateHandle LibraryHandle;

	void Rebuild();
	FText CurrentLabel() const;
};
