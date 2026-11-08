#include "MaterialPicker.h"

#include "Widgets/Input/SComboBox.h"
#include "Widgets/Text/STextBlock.h"

#include "Entity/AerotelicMaterialLibrary.h"

#define LOCTEXT_NAMESPACE "AerotelicMaterialPicker"

using aerotelic::Material;

void SAerotelicMaterialPicker::Construct(const FArguments& InArgs)
{
	Current = InArgs._Current;
	OnPicked = InArgs._OnPicked;
	bAllowNone = InArgs._AllowNone;
	Rebuild();

	ChildSlot
	[
		SAssignNew(Combo, SComboBox<TSharedPtr<FChoice>>)
		.OptionsSource(&Choices)
		.OnGenerateWidget_Lambda([](TSharedPtr<FChoice> C) { return SNew(STextBlock).Text(FText::FromString(C->Label)); })
		.OnComboBoxOpening_Lambda([this]() { Rebuild(); if (Combo.IsValid()) Combo->RefreshOptions(); })
		.OnSelectionChanged_Lambda([this](TSharedPtr<FChoice> C, ESelectInfo::Type Info)
		{
			if (!C.IsValid() || Info == ESelectInfo::Direct) return;
			OnPicked.ExecuteIfBound(C->M);
		})
		[
			SNew(STextBlock).Text_Lambda([this]() { return CurrentLabel(); })
		]
	];

	LibraryHandle = AerotelicMaterialLibrary::OnChanged().AddLambda([this]() { Rebuild(); if (Combo.IsValid()) Combo->RefreshOptions(); });
}

SAerotelicMaterialPicker::~SAerotelicMaterialPicker()
{
	AerotelicMaterialLibrary::OnChanged().Remove(LibraryHandle);
}

void SAerotelicMaterialPicker::Rebuild()
{
	Choices.Reset();
	if (bAllowNone)
	{
		TSharedPtr<FChoice> None = MakeShared<FChoice>();
		None->bNone = true;
		None->M = Material{};
		None->M.name.clear();
		None->Label = LOCTEXT("None", "Not set").ToString();
		Choices.Add(None);
	}
	for (const Material& M : AerotelicMaterialLibrary::All())
	{
		TSharedPtr<FChoice> C = MakeShared<FChoice>();
		C->M = M;
		C->Label = UTF8_TO_TCHAR(M.name.c_str());
		Choices.Add(C);
	}
	const Material Now = Current.Get();
	if (Now.HasDensity() && AerotelicMaterialLibrary::FindMatching(Now) == nullptr)
	{
		TSharedPtr<FChoice> Own = MakeShared<FChoice>();
		Own->bOwn = true;
		Own->M = Now;
		Own->Label = FText::Format(LOCTEXT("Own", "{0} -- these figures (not in the library)"), FText::FromString(UTF8_TO_TCHAR(Now.name.c_str()))).ToString();
		Choices.Add(Own);
	}
}

FText SAerotelicMaterialPicker::CurrentLabel() const
{
	const Material Now = Current.Get();
	if (!Now.HasDensity()) return LOCTEXT("None", "Not set");
	const bool bInLibrary = AerotelicMaterialLibrary::FindMatching(Now) != nullptr;
	return bInLibrary
		? FText::FromString(UTF8_TO_TCHAR(Now.name.c_str()))
		: FText::Format(LOCTEXT("OwnShort", "{0} (own figures)"), FText::FromString(UTF8_TO_TCHAR(Now.name.c_str())));
}

#undef LOCTEXT_NAMESPACE
