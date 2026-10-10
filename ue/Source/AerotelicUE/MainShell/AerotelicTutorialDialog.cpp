#include "AerotelicTutorialDialog.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Entity/AerotelicDocumentSubsystem.h"

#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "Widgets/Views/SHeaderRow.h"
#include "Styling/CoreStyle.h"

#include "aerotelic/project_yaml.hpp"

#define LOCTEXT_NAMESPACE "AerotelicTutorialDialog"

namespace
{
	struct FTutorialRow
	{
		FString Path;        // relative to the project (resource) root
		FString Level;       // Introductory | Intermediate | Advanced
		FString Title, Subject, Teaches;
		FString AbsPath, Contents, Error;
		double SizeKb = 0.0;
		bool Ok() const { return Error.IsEmpty(); }
	};
	using FRowPtr = TSharedPtr<FTutorialRow>;

	// The catalogue, simple to complex (ui/tutorials.py CATALOGUE).
	TArray<FRowPtr> Catalogue()
	{
		TArray<FRowPtr> Rows;
		{
			FRowPtr R = MakeShared<FTutorialRow>();
			R->Path = TEXT("Projects/cessna_172.yaml");
			R->Level = TEXT("Introductory");
			R->Title = TEXT("Cessna 172");
			R->Subject = TEXT("Fuselage + NURBS wing");
			R->Teaches = TEXT("The core workflow end to end: a lofted body from profile and width curves, a wing whose leading and trailing edges are independent NURBS curves, and a boolean feature tree joining the two. Bring your own three-view drawing and the curve editors will underlay it.");
			Rows.Add(R);
		}
		{
			FRowPtr R = MakeShared<FTutorialRow>();
			R->Path = TEXT("Projects/p38.yaml");
			R->Level = TEXT("Advanced");
			R->Title = TEXT("Lockheed P-38 Lightning");
			R->Subject = TEXT("Twin-boom layout");
			R->Teaches = TEXT("The same technique on an unconventional arrangement: twin booms as separate bodies, a centre nacelle, NURBS planforms throughout, and the adaptive spanwise stations a curved tip needs.");
			Rows.Add(R);
		}
		return Rows;
	}

	// The Contents column is computed, not authored: counts per entity type,
	// the distinct planform types, the feature count.
	void Describe(FTutorialRow& Row)
	{
		Row.AbsPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / Row.Path);
		if (!FPaths::FileExists(Row.AbsPath))
		{
			Row.Error = TEXT("not found");
			return;
		}
		Row.SizeKb = IFileManager::Get().FileSize(*Row.AbsPath) / 1024.0;
		try
		{
			using namespace aerotelic::project_yaml;
			const ProjectFile P = LoadProjectYaml(TCHAR_TO_UTF8(*Row.AbsPath));
			TMap<FString, int32> Counts;
			TSet<FString> Planforms;
			for (const ProjectEntity& E : P.entities)
			{
				Counts.FindOrAdd(UTF8_TO_TCHAR(EntityTypeName(E.type)))++;
				if (const aerotelic::WingModelSpec* W = std::get_if<aerotelic::WingModelSpec>(&E.model))
				{
					Planforms.Add(FString(UTF8_TO_TCHAR(W->planform.type.c_str())));
				}
			}
			TArray<FString> Parts;
			static const TMap<FString, TPair<FString, FString>> Words = {
				{ TEXT("wing"), { TEXT("wing"), TEXT("wings") } }, { TEXT("fuselage"), { TEXT("body"), TEXT("bodies") } },
				{ TEXT("nacelle"), { TEXT("nacelle"), TEXT("nacelles") } }, { TEXT("lofted_nacelle"), { TEXT("lofted nacelle"), TEXT("lofted nacelles") } },
				{ TEXT("propeller"), { TEXT("propeller"), TEXT("propellers") } }, { TEXT("primitive"), { TEXT("primitive"), TEXT("primitives") } } };
			for (const TCHAR* Kind : { TEXT("fuselage"), TEXT("wing"), TEXT("nacelle"), TEXT("lofted_nacelle"), TEXT("propeller"), TEXT("primitive") })
			{
				if (const int32* N = Counts.Find(Kind))
				{
					const TPair<FString, FString>& W = Words[Kind];
					Parts.Add(FString::Printf(TEXT("%d %s"), *N, *N == 1 ? *W.Key : *W.Value));
				}
			}
			if (Planforms.Num() > 0)
			{
				TArray<FString> Sorted = Planforms.Array();
				Sorted.Sort();
				Parts.Add(FString::Join(Sorted, TEXT("/")) + (Sorted.Num() == 1 ? TEXT(" planform") : TEXT(" planforms")));
			}
			const int32 Features = (int32)P.booleans.size();
			Parts.Add(FString::Printf(TEXT("%d feature%s"), Features, Features == 1 ? TEXT("") : TEXT("s")));
			Row.Contents = Parts.Num() > 0 ? FString::Join(Parts, TEXT(", ")) : TEXT("empty");
		}
		catch (const std::exception& e)
		{
			Row.Error = UTF8_TO_TCHAR(e.what());
		}
	}

	const FName ColProject("Project"), ColLevel("Level"), ColSubject("Subject"), ColContents("Contents"), ColCovers("Covers"), ColFile("File");

	class STutorialRow : public SMultiColumnTableRow<FRowPtr>
	{
	public:
		SLATE_BEGIN_ARGS(STutorialRow) {}
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& Owner, FRowPtr InRow)
		{
			Row = InRow;
			SMultiColumnTableRow<FRowPtr>::Construct(FSuperRowType::FArguments().Padding(FMargin(4, 3)), Owner);
		}

		virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& Column) override
		{
			FString Text;
			bool bBold = false;
			if (Column == ColProject) { Text = Row->Title; bBold = true; }
			else if (Column == ColLevel) Text = Row->Level;
			else if (Column == ColSubject) Text = Row->Subject;
			else if (Column == ColContents) Text = Row->Ok() ? Row->Contents : Row->Error;
			else if (Column == ColCovers) Text = Row->Teaches;
			else if (Column == ColFile) Text = Row->Path;
			const FLinearColor Color = Row->Ok() ? FLinearColor::White : FLinearColor(0.55f, 0.55f, 0.55f);
			return SNew(SBox).Padding(FMargin(4, 2))
			[
				SNew(STextBlock).Text(FText::FromString(Text)).AutoWrapText(true)
				.Font(FCoreStyle::GetDefaultFontStyle(bBold ? "Bold" : "Regular", 10))
				.ColorAndOpacity(FSlateColor(Color))
			];
		}

	private:
		FRowPtr Row;
	};
}

void ShowAerotelicTutorialDialog()
{
	TArray<FRowPtr> Rows = Catalogue();
	TArray<FString> Missing;
	for (const FRowPtr& R : Rows)
	{
		Describe(*R);
		if (!R->Ok()) Missing.Add(FString::Printf(TEXT("%s (%s)"), *R->Path, *R->Error));
	}

	TSharedPtr<SListView<FRowPtr>> List;
	TOptional<FString> Chosen;
	const FVector2D Size(1200, 500);

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Tutorial Projects"))
		.ClientSize(Size)
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto OpenSelected = [&]()
	{
		TArray<FRowPtr> Selected = List->GetSelectedItems();
		if (Selected.Num() == 1 && Selected[0]->Ok())
		{
			Chosen = Selected[0]->AbsPath;
			// Close first, so the loader's own dialogs never appear behind
			// this modal.
			Window->RequestDestroyWindow();
		}
	};

	TSharedRef<SVerticalBox> Body = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 15)).Text(LOCTEXT("Heading", "Projects delivered with the application")) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
		[
			SNew(STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(0.55f, 0.55f, 0.55f)))
			.Text(LOCTEXT("SubHeading", "Pick a row and choose Open, or double-click it. Opening a project replaces the current scene."))
		]
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 10)
		[
			SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.05f, 0.05f, 0.06f)).Padding(2)
			[
				SAssignNew(List, SListView<FRowPtr>)
				.ListItemsSource(&Rows)
				.SelectionMode(ESelectionMode::Single)
				.OnGenerateRow_Lambda([](FRowPtr Row, const TSharedRef<STableViewBase>& Owner) { return SNew(STutorialRow, Owner, Row); })
				.OnMouseButtonDoubleClick_Lambda([&](FRowPtr) { OpenSelected(); })
				.HeaderRow
				(
					SNew(SHeaderRow)
					+ SHeaderRow::Column(ColProject).DefaultLabel(LOCTEXT("ColProject", "Project")).ManualWidth(150)
					+ SHeaderRow::Column(ColLevel).DefaultLabel(LOCTEXT("ColLevel", "Level")).ManualWidth(100)
					+ SHeaderRow::Column(ColSubject).DefaultLabel(LOCTEXT("ColSubject", "Subject")).ManualWidth(165)
					+ SHeaderRow::Column(ColContents).DefaultLabel(LOCTEXT("ColContents", "Contents")).ManualWidth(205)
					+ SHeaderRow::Column(ColCovers).DefaultLabel(LOCTEXT("ColCovers", "What it covers")).FillWidth(1.0f)
					+ SHeaderRow::Column(ColFile).DefaultLabel(LOCTEXT("ColFile", "File")).ManualWidth(170)
				)
			]
		];
	if (Missing.Num() > 0)
	{
		Body->AddSlot().AutoHeight().Padding(0, 0, 0, 8)
		[
			SNew(STextBlock).AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular", 10)).ColorAndOpacity(FSlateColor(FLinearColor(0.55f, 0.55f, 0.55f)))
			.Text(FText::FromString(FString::Printf(TEXT("%d project file(s) could not be read and are shown greyed: %s"), Missing.Num(), *FString::Join(Missing, TEXT(", ")))))
		];
	}
	Body->AddSlot().AutoHeight().HAlign(HAlign_Right)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
		[ SNew(SButton).Text(LOCTEXT("OpenProject", "Open Project")).OnClicked_Lambda([&]() { OpenSelected(); return FReply::Handled(); }) ]
		+ SHorizontalBox::Slot().AutoWidth()
		[ SNew(SButton).Text(LOCTEXT("Close", "Close")).OnClicked_Lambda([&]() { Window->RequestDestroyWindow(); return FReply::Handled(); }) ]
	];

	Window->SetContent(SNew(SBorder).Padding(FMargin(16, 14, 16, 12)) [ Body ]);
	// The first readable row is pre-selected.
	for (const FRowPtr& R : Rows)
	{
		if (R->Ok()) { List->SetSelection(R); break; }
	}
	FSlateApplication::Get().AddModalWindow(Window, FindAerotelicMainWindow());

	if (Chosen.IsSet())
	{
		if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
		{
			Doc->OpenProjectPath(Chosen.GetValue(), /*bAdopt*/ false);
		}
	}
}

#undef LOCTEXT_NAMESPACE
