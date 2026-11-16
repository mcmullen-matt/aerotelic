#include "LoadCasesDialog.h"

#include "FeaCaseState.h"
#include "Widgets/VnPlot.h"

#define LOCTEXT_NAMESPACE "LoadCasesDialog"

namespace
{
	using namespace AerotelicFeaCase;

	struct FDialog : public TSharedFromThis<FDialog>
	{
		TFunction<void(const FText&)> Status;
		TWeakPtr<SWindow> Window;
		aerotelic::UnitSystem Units;
		// The working copy of the document's cases; every edit is committed.
		aerotelic::LoadCasesSpec Spec;
		TArray<FAerotelicBodyChoicePtr> Choices;

		// ---- aero tab ----
		TSharedPtr<SAerotelicVnPlot> Plot;
		TSharedPtr<SVerticalBox> AeroList;
		TSharedPtr<STextBlock> LblEnvelope;
		int32 AeroSelected = -1;
		TArray<AerotelicAeroTrim::FControl> ControlList;    // every flap of every panelable wing
		TArray<TSharedPtr<FString>> ControlNames;          // "none" then each
		TMap<FString, TSharedPtr<AerotelicAeroTrim::FRunner>> Runners;   // by case name
		TMap<FString, FString> AeroStatus;                 // by case name: the last trim's words
		FDelegateHandle FieldsHandle;

		// ---- structural tab ----
		TSharedPtr<FState> Fea;
		int32 StaticSelected = -1;
		TSharedPtr<SVerticalBox> StaticList;
		TSharedPtr<SBox> BodiesBox;
		TSharedPtr<SAerotelicTabStrip> StaticTabs;
		TSharedPtr<STextBlock> LblStatic;
		TSharedPtr<SAerotelicTabStrip> Tabs;

		void Commit()
		{
			if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->SetLoadCases(Spec);
		}

		// The entities a body list stands for: as named (for the mass), or
		// expanded to what the panel method can panel (a boolean result's
		// operands).
		TArray<AAerotelicEntityActor*> EntitiesOf(const std::vector<aerotelic::LoadCaseBody>& Bodies, bool bPanelable) const
		{
			TArray<AAerotelicEntityActor*> Out;
			for (const FAerotelicBodyChoicePtr& C : Fea->ChoicesFor(Bodies))
			{
				AAerotelicEntityActor* E = C->Operand.Entity.Get();
				if (E == nullptr) continue;
				if (!bPanelable) { Out.AddUnique(E); continue; }
				if (AerotelicAeroTrim::IsPanelable(E->GetEntityType())) Out.AddUnique(E);
				else if (E->GetEntityType() == EntityType::Boolean)
				{
					if (const FAerotelicBooleanModel* M = E->GetModelAs<FAerotelicBooleanModel>())
						for (const FAerotelicBooleanOperand& Op : M->Operands)
							if (Op.Entity.IsValid() && AerotelicAeroTrim::IsPanelable(Op.Entity->GetEntityType())) Out.AddUnique(Op.Entity.Get());
				}
			}
			return Out;
		}

		// The bodies the envelope's automatic weight and area are taken from:
		// the selected aero case's, else every body no boolean already holds.
		std::vector<aerotelic::LoadCaseBody> EnvelopeBodies() const
		{
			if (Spec.aero.size() > 0 && AeroSelected >= 0 && AeroSelected < static_cast<int32>(Spec.aero.size()) && !Spec.aero[AeroSelected].bodies.empty()) return Spec.aero[AeroSelected].bodies;
			std::vector<aerotelic::LoadCaseBody> Out;
			for (const FAerotelicBodyChoicePtr& C : DefaultChosen(Choices)) Out.push_back(FState::BodyOf(*C));
			return Out;
		}

		// Weight (N) and area (m^2) as the envelope uses them, with where they came from.
		void WeightAndArea(double& OutWeight, double& OutArea, FString& OutText) const
		{
			const std::vector<aerotelic::LoadCaseBody> Bodies = EnvelopeBodies();
			if (Spec.vn.weight_auto)
			{
				double MassKg = 0.0;
				aerotelic::Vec3 Centre;
				FString Text;
				AerotelicAeroTrim::MassAndCentre(EntitiesOf(Bodies, false), Units, MassKg, Centre, Text);
				OutWeight = MassKg * 9.80665;
				OutText = FString::Printf(TEXT("W = %.4g N from the mass report (%.4g kg)"), OutWeight, MassKg);
			}
			else
			{
				OutWeight = Spec.vn.weight;
				OutText = FString::Printf(TEXT("W = %.4g N as typed"), OutWeight);
			}
			if (Spec.vn.area_auto)
			{
				double Area = 0.0, Length = 0.0;
				const TArray<AAerotelicEntityActor*> Es = EntitiesOf(Bodies, true);
				if (Es.Num() > 0) AerotelicAeroTrim::DefaultReferences(Es, Area, Length);
				OutArea = Area * Units.AreaToSI();
				OutText += FString::Printf(TEXT("; S = %.4g m² (the chosen wings' planform, both halves)"), OutArea);
			}
			else
			{
				OutArea = Spec.vn.area;
				OutText += FString::Printf(TEXT("; S = %.4g m² as typed"), OutArea);
			}
		}

		void RefreshEnvelope()
		{
			if (!Plot.IsValid()) return;
			double W = 0.0, A = 0.0;
			FString Text;
			WeightAndArea(W, A, Text);
			Plot->SetEnvelope(Spec.vn, W, A);
			const double VS = Plot->StallSpeed(), VA = Plot->ManoeuvreSpeed(), VSn = Plot->NegativeStallSpeed(), VAn = Plot->NegativeCornerSpeed();
			Text += FString::Printf(TEXT(". Stall boundary n = ½ ρ V² CL S / W: V_S = %.2f m/s (n = 1), V_A = %.2f m/s (n_max); below, %.2f m/s at n = -1 and the corner at %.2f m/s (n_min). q at V_ne: %.4g Pa."),
				VS, VA, VSn, VAn, aerotelic::aero::DynamicPressure(Spec.vn.density, Spec.vn.v_ne));
			if (LblEnvelope.IsValid()) LblEnvelope->SetText(FText::FromString(Text));
			RefreshAeroPoints();
		}

		void RefreshAeroPoints()
		{
			if (!Plot.IsValid()) return;
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			TArray<FAerotelicVnPoint> Points;
			for (const aerotelic::AeroLoadCaseSpec& A : Spec.aero)
			{
				FAerotelicVnPoint P;
				P.Speed = A.speed;
				P.LoadFactor = A.load_factor;
				P.Name = UTF8_TO_TCHAR(A.name.c_str());
				P.bSolved = Doc != nullptr && Doc->FindAeroLoadCase(P.Name).IsValid();
				Points.Add(P);
			}
			Plot->SetPoints(Points);
			Plot->SetSelected(AeroSelected);
		}

		FString UniqueAeroName() const
		{
			for (int32 k = 1; ; ++k)
			{
				const std::string Name = "Case " + std::to_string(k);
				bool bTaken = false;
				for (const aerotelic::AeroLoadCaseSpec& A : Spec.aero) bTaken = bTaken || A.name == Name;
				if (!bTaken) return UTF8_TO_TCHAR(Name.c_str());
			}
		}

		void AddAeroCase(double Speed, double LoadFactor)
		{
			aerotelic::AeroLoadCaseSpec A;
			A.name = TCHAR_TO_UTF8(*UniqueAeroName());
			A.speed = FMath::Max(Speed, 0.1);
			A.density = Spec.vn.density;
			A.load_factor = LoadFactor;
			// The bodies: the previous case's, else every body no boolean holds.
			if (!Spec.aero.empty()) A.bodies = Spec.aero.back().bodies;
			else for (const FAerotelicBodyChoicePtr& C : DefaultChosen(Choices)) A.bodies.push_back(FState::BodyOf(*C));
			// The control: the aftmost flap of the paneled wings.
			int32 Def = -1;
			const TArray<AerotelicAeroTrim::FControl> Controls = AerotelicAeroTrim::Controls(EntitiesOf(A.bodies, true), Def);
			if (!Spec.aero.empty()) { A.control_entity = Spec.aero.back().control_entity; A.control_flap = Spec.aero.back().control_flap; }
			else if (Controls.IsValidIndex(Def) && Controls[Def].Wing.IsValid())
			{
				A.control_entity = TCHAR_TO_UTF8(*Controls[Def].Wing->EntityName);
				const aerotelic::WingModel* W = Controls[Def].Wing->GetModelAs<aerotelic::WingModel>();
				if (W != nullptr && Controls[Def].Flap < static_cast<int32>(W->flaps.size())) A.control_flap = W->flaps[Controls[Def].Flap].name;
			}
			Spec.aero.push_back(A);
			AeroSelected = static_cast<int32>(Spec.aero.size()) - 1;
			Commit();
			RebuildAeroList();
			RefreshEnvelope();
		}

		void DeleteAeroCase(int32 Index)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.aero.size())) return;
			const FString Name = UTF8_TO_TCHAR(Spec.aero[Index].name.c_str());
			Spec.aero.erase(Spec.aero.begin() + Index);
			if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->RemoveAeroLoadCase(Name);
			AeroStatus.Remove(Name);
			if (AeroSelected >= static_cast<int32>(Spec.aero.size())) AeroSelected = static_cast<int32>(Spec.aero.size()) - 1;
			Commit();
			RebuildAeroList();
			RefreshEnvelope();
		}

		// A renamed case's solved field and status follow the new name.
		void RenameAeroCase(int32 Index, const FString& NewName)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.aero.size()) || NewName.IsEmpty()) return;
			const FString Old = UTF8_TO_TCHAR(Spec.aero[Index].name.c_str());
			if (Old == NewName) return;
			for (const aerotelic::AeroLoadCaseSpec& A : Spec.aero) if (UTF8_TO_TCHAR(A.name.c_str()) == NewName) { Status(LOCTEXT("NameTaken", "Another aero case has that name.")); RebuildAeroList(); return; }
			Spec.aero[Index].name = TCHAR_TO_UTF8(*NewName);
			if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
			{
				if (TSharedPtr<FAerotelicAeroLoadCase> Field = Doc->FindAeroLoadCase(Old))
				{
					Doc->RemoveAeroLoadCase(Old);
					Field->Name = NewName;
					Doc->SetAeroLoadCase(Field);
				}
			}
			if (const FString* St = AeroStatus.Find(Old)) { AeroStatus.Add(NewName, *St); AeroStatus.Remove(Old); }
			Commit();
			RebuildAeroList();
			RefreshEnvelope();
		}

		// Any geometry edit of a case invalidates its solved pressures.
		void InvalidateAero(int32 Index)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.aero.size())) return;
			if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->RemoveAeroLoadCase(UTF8_TO_TCHAR(Spec.aero[Index].name.c_str()));
			AeroStatus.Remove(UTF8_TO_TCHAR(Spec.aero[Index].name.c_str()));
		}

		void RefreshControls()
		{
			ControlList.Reset();
			ControlNames.Reset();
			ControlNames.Add(MakeShared<FString>(TEXT("none: the angle of attack alone")));
			int32 Def = -1;
			for (const AerotelicAeroTrim::FControl& C : AerotelicAeroTrim::Controls(AerotelicAeroTrim::PanelableEntities(), Def))
			{
				ControlList.Add(C);
				ControlNames.Add(MakeShared<FString>(C.Label));
			}
		}

		int32 ControlIndexOf(const aerotelic::AeroLoadCaseSpec& A) const
		{
			if (A.control_entity.empty()) return 0;
			for (int32 i = 0; i < ControlList.Num(); ++i)
			{
				const AerotelicAeroTrim::FControl& C = ControlList[i];
				if (C.Wing.IsValid() && C.Wing->EntityName == UTF8_TO_TCHAR(A.control_entity.c_str()) && C.Label.EndsWith(UTF8_TO_TCHAR(A.control_flap.c_str()))) return i + 1;
			}
			return 0;
		}

		// The set-up the panel method runs an aero case with: its panelable
		// entities (a boolean result's operands), the plane, the references.
		bool SetupFor(const aerotelic::AeroLoadCaseSpec& A, AerotelicAeroTrim::FSetup& Setup, TArray<AAerotelicEntityActor*>& Panelable) const
		{
			Panelable = EntitiesOf(A.bodies, true);
			if (Panelable.Num() == 0) return false;
			for (AAerotelicEntityActor* E : Panelable) Setup.Entities.Add(E);
			Setup.bSymmetry = AerotelicAeroTrim::DefaultSymmetry(Panelable);
			AerotelicAeroTrim::DefaultReferences(Panelable, Setup.ReferenceArea, Setup.ReferenceLength);
			return true;
		}

		// The mesh the panel method would solve for one aero case, shown in
		// the main 3D view with its edges, its report in the case's status
		// line: the user's check of what is analysed (2026-11-15).
		void ShowAeroMesh(int32 Index)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.aero.size())) return;
			const aerotelic::AeroLoadCaseSpec A = Spec.aero[Index];
			const FString Name = UTF8_TO_TCHAR(A.name.c_str());
			AerotelicAeroTrim::FSetup Setup;
			TArray<AAerotelicEntityActor*> Panelable;
			if (!SetupFor(A, Setup, Panelable))
			{
				AeroStatus.Add(Name, TEXT("no body of the case can be paneled: choose a wing, fuselage, nacelle, primitive or a boolean result over them"));
				RebuildAeroList();
				return;
			}
			try
			{
				TSharedPtr<AerotelicAeroTrim::FCase> C = AerotelicAeroTrim::BuildCase(Setup, 0.0, 0.0);
				AerotelicAeroTrim::ShowAeroMeshInViewport(*C);
				FString Text = TEXT("aero mesh (shown in the 3D view): ") + C->MeshReport.Replace(TEXT("\n"), TEXT(" | "));
				for (const FString& N : C->Notes) Text += TEXT(" | ") + N;
				AeroStatus.Add(Name, Text);
				Status(FText::FromString(FString::Printf(TEXT("Aero mesh of '%s' in the 3D view: %d triangles%s. Hide mesh takes it away."), *Name, C->Triangles, Setup.bSymmetry ? TEXT(", the y >= 0 half solved with its image") : TEXT(""))));
			}
			catch (const std::exception& E)
			{
				AeroStatus.Add(Name, FString(TEXT("the aero mesh could not be built: ")) + UTF8_TO_TCHAR(E.what()));
			}
			RebuildAeroList();
		}

		// The trim of one aero case: panels the case's bodies, trims to its
		// condition, stores the pressure field on the document.
		void SolveAero(int32 Index)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.aero.size())) return;
			const aerotelic::AeroLoadCaseSpec A = Spec.aero[Index];
			const FString Name = UTF8_TO_TCHAR(A.name.c_str());
			if (TSharedPtr<AerotelicAeroTrim::FRunner>* R = Runners.Find(Name)) if (R->IsValid() && (*R)->IsRunning()) { Status(LOCTEXT("TrimBusy", "That case's trim is still running.")); return; }
			AerotelicAeroTrim::FSetup Setup;
			TArray<AAerotelicEntityActor*> Panelable;
			if (!SetupFor(A, Setup, Panelable))
			{
				AeroStatus.Add(Name, TEXT("no body of the case can be paneled: choose a wing, fuselage, nacelle, primitive or a boolean result over them"));
				RebuildAeroList();
				return;
			}
			AerotelicAeroTrim::FCondition Cond;
			Cond.Name = Name;
			Cond.LoadFactor = A.load_factor;
			Cond.Speed = A.speed;
			Cond.Density = A.density;
			FString MassText;
			if (!AerotelicAeroTrim::MassAndCentre(EntitiesOf(A.bodies, false), Units, Cond.MassKg, Cond.MassCentre, MassText))
			{
				AeroStatus.Add(Name, TEXT("the case's bodies weigh nothing (no structure), so there is no weight to trim for"));
				RebuildAeroList();
				return;
			}
			const int32 Ctl = ControlIndexOf(A);
			if (Ctl > 0)
			{
				Cond.Control = ControlList[Ctl - 1];
				const aerotelic::WingModel* W = Cond.Control.Wing.IsValid() ? Cond.Control.Wing->GetModelAs<aerotelic::WingModel>() : nullptr;
				if (W != nullptr && Cond.Control.Flap < static_cast<int32>(W->flaps.size())) Cond.Delta0 = W->flaps[Cond.Control.Flap].deflection_deg;
			}
			TSharedPtr<AerotelicAeroTrim::FRunner> Runner = MakeShared<AerotelicAeroTrim::FRunner>();
			Runners.Add(Name, Runner);
			AeroStatus.Add(Name, TEXT("solving..."));
			RebuildAeroList();
			TWeakPtr<FDialog> Weak = AsShared();
			Runner->Start(Setup, Cond, Units,
				[Weak](const FText& T) { if (TSharedPtr<FDialog> D = Weak.Pin()) D->Status(T); },
				[Weak, Name](const AerotelicAeroTrim::FOutcome& O)
				{
					TSharedPtr<FDialog> D = Weak.Pin();
					if (!D.IsValid()) return;
					D->Runners.Remove(Name);
					if (O.bOk)
					{
						if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->SetAeroLoadCase(O.Case);
						const AerotelicAeroTrim::FCase& C = *O.Trimmed;
						FString Solved = FString::Printf(TEXT("solved: α = %.2f°%s, C_L %.4f (%.4f required; wake %.4f), C_m %.4f, q = %.4g Pa; %d panels on %d triangles%s, %s in %.1f s. Mesh: %s"), C.AlphaDeg,
							C.Control.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(", δ = %.2f°"), C.DeltaDeg), C.CL, O.CLRequired, C.Sol.lift_from_wake.value_or(0.0), C.Cm, O.DynamicPressure, O.Case->BodyPanels(), C.Triangles,
							C.bImagedSolve ? TEXT(" (the y >= 0 half with its image)") : TEXT(""), C.Sol.direct ? TEXT("direct") : *FString::Printf(TEXT("GMRES %d it"), C.Sol.iterations), C.Sol.seconds_influence + C.Sol.seconds_solve,
							*C.MeshReport.Replace(TEXT("\n"), TEXT(" | ")));
						for (const FString& W : C.Warnings) Solved += TEXT(" | WARNING: ") + W;
						D->AeroStatus.Add(Name, Solved);
						D->Status(FText::FromString(FString::Printf(TEXT("Aero case '%s' trimmed and stored."), *Name)));
					}
					else
					{
						D->AeroStatus.Add(Name, TEXT("FAILED: ") + O.Report.Replace(TEXT("\n"), TEXT(" ")));
						D->Status(FText::FromString(FString::Printf(TEXT("Aero case '%s': the trim failed."), *Name)));
					}
					UE_LOG(LogTemp, Log, TEXT("LoadCases: %s: %s"), *Name, *O.Report.Replace(TEXT("\n"), TEXT(" | ")));
					D->RebuildAeroList();
					D->RefreshAeroPoints();
				});
		}

		// A popup with the body picker for one aero case.
		void EditAeroBodies(int32 Index, TSharedRef<SWidget> Anchor)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.aero.size())) return;
			TSharedPtr<FDialog> Self = AsShared();
			TSharedPtr<SAerotelicBodyPicker> Picker;
			TSharedPtr<SWidget> Content =
				SNew(SBorder).Padding(10)
				[
					SNew(SBox).WidthOverride(620)
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
						[ SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("Solid bodies of '%s'"), UTF8_TO_TCHAR(Spec.aero[Index].name.c_str())))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).InitiallyChosen(Fea->ChoicesFor(Spec.aero[Index].bodies)).ListHeight(160.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ MakeInfoLabel(LOCTEXT("AeroBodiesHint", "The bodies the case flies: the structural model, and what the panel method panels on their own triangles (a wing as its sealed aero copy, a boolean result as its operands' aero copies, overlapping bodies fused into one surface first). Close the popup to keep the choice; the case's pressures are then solved again.")) ]
					]
				];
			TSharedPtr<IMenu> Menu = FSlateApplication::Get().PushMenu(Anchor, FWidgetPath(), Content.ToSharedRef(), FSlateApplication::Get().GetCursorPos(), FPopupTransitionEffect(FPopupTransitionEffect::ContextMenu));
			if (Menu.IsValid())
			{
				Menu->GetOnMenuDismissed().AddLambda([Self, Picker, Index](TSharedRef<IMenu>)
				{
					if (!Picker.IsValid() || Index >= static_cast<int32>(Self->Spec.aero.size())) return;
					std::vector<aerotelic::LoadCaseBody> Bodies;
					for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen()) Bodies.push_back(FState::BodyOf(*C));
					if (Bodies == Self->Spec.aero[Index].bodies) return;
					Self->Spec.aero[Index].bodies = Bodies;
					Self->InvalidateAero(Index);
					Self->Commit();
					Self->RebuildAeroList();
					Self->RefreshEnvelope();
				});
			}
		}

		void RebuildAeroList()
		{
			if (!AeroList.IsValid()) return;
			AeroList->ClearChildren();
			TSharedPtr<FDialog> Self = AsShared();
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			auto Num = [Self](int32 Index, double aerotelic::AeroLoadCaseSpec::*Member, double Min, double Max, double Delta, const FText& Suffix)
			{
				return SNew(SAerotelicNumericEntry)
					.Value_Lambda([Self, Index, Member]() { return Index < static_cast<int32>(Self->Spec.aero.size()) ? Self->Spec.aero[Index].*Member : 0.0; })
					.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix).LiveUpdate(false)
					.OnValueCommitted_Lambda([Self, Index, Member](double V, ETextCommit::Type)
					{
						if (Index >= static_cast<int32>(Self->Spec.aero.size())) return;
						if (Self->Spec.aero[Index].*Member == V) return;
						Self->Spec.aero[Index].*Member = V;
						Self->InvalidateAero(Index);
						Self->Commit();
						Self->RefreshEnvelope();
						Self->RebuildAeroList();
					});
			};
			if (Spec.aero.empty())
			{
				AeroList->AddSlot().AutoHeight() [ SNew(STextBlock).AutoWrapText(true).Text(LOCTEXT("NoAero", "No aero case yet: click inside the envelope above to add one, or Add at V_A.")) ];
				return;
			}
			// Header.
			AeroList->AddSlot().AutoHeight().Padding(0, 0, 0, 2)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.6f) [ SNew(STextBlock).Text(LOCTEXT("HName", "Name")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f) [ SNew(STextBlock).Text(LOCTEXT("HSpeed", "V, m/s")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f) [ SNew(STextBlock).Text(LOCTEXT("HRho", "ρ, kg/m³")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(0.8f) [ SNew(STextBlock).Text(LOCTEXT("HN", "n")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.8f) [ SNew(STextBlock).Text(LOCTEXT("HControl", "Trim control")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f) [ SNew(STextBlock).Text(LOCTEXT("HBodies", "Bodies")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.4f) [ SNew(STextBlock).Text(LOCTEXT("HActions", "")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
			];
			for (int32 i = 0; i < static_cast<int32>(Spec.aero.size()); ++i)
			{
				const aerotelic::AeroLoadCaseSpec& A = Spec.aero[i];
				const FString Name = UTF8_TO_TCHAR(A.name.c_str());
				const bool bSolved = Doc != nullptr && Doc->FindAeroLoadCase(Name).IsValid();
				const FString* St = AeroStatus.Find(Name);
				const FString StatusText = St ? *St : (bSolved ? TEXT("solved") : TEXT("not solved"));
				const bool bSel = i == AeroSelected;
				TSharedPtr<SButton> BodiesButton;
				AeroList->AddSlot().AutoHeight().Padding(0, 0, 0, 4)
				[
					SNew(SBorder).Padding(4).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(bSel ? FLinearColor(0.16f, 0.2f, 0.3f) : FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1.6f).Padding(0, 0, 4, 0).VAlign(VAlign_Center)
							[
								SNew(SEditableTextBox).Text(FText::FromString(Name))
								.OnTextCommitted_Lambda([Self, i](const FText& T, ETextCommit::Type) { Self->RenameAeroCase(i, T.ToString().TrimStartAndEnd()); })
							]
							+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0).VAlign(VAlign_Center) [ Num(i, &aerotelic::AeroLoadCaseSpec::speed, 0.1, 1000.0, 1.0, FText::GetEmpty()) ]
							+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0).VAlign(VAlign_Center) [ Num(i, &aerotelic::AeroLoadCaseSpec::density, 0.001, 10.0, 0.025, FText::GetEmpty()) ]
							+ SHorizontalBox::Slot().FillWidth(0.8f).Padding(0, 0, 4, 0).VAlign(VAlign_Center) [ Num(i, &aerotelic::AeroLoadCaseSpec::load_factor, -20.0, 20.0, 0.5, FText::GetEmpty()) ]
							+ SHorizontalBox::Slot().FillWidth(1.8f).Padding(0, 0, 4, 0).VAlign(VAlign_Center)
							[
								SNew(SComboBox<TSharedPtr<FString>>)
								.OptionsSource(&Self->ControlNames)
								.InitiallySelectedItem(Self->ControlNames.IsValidIndex(ControlIndexOf(A)) ? Self->ControlNames[ControlIndexOf(A)] : nullptr)
								.OnGenerateWidget_Lambda([](TSharedPtr<FString> N) { return SNew(STextBlock).Text(FText::FromString(*N)); })
								.OnSelectionChanged_Lambda([Self, i](TSharedPtr<FString> N, ESelectInfo::Type)
								{
									if (!N.IsValid() || i >= static_cast<int32>(Self->Spec.aero.size())) return;
									const int32 K = Self->ControlNames.IndexOfByKey(N);
									aerotelic::AeroLoadCaseSpec& Case = Self->Spec.aero[i];
									std::string Entity, Flap;
									if (K > 0 && Self->ControlList.IsValidIndex(K - 1) && Self->ControlList[K - 1].Wing.IsValid())
									{
										const AerotelicAeroTrim::FControl& C = Self->ControlList[K - 1];
										Entity = TCHAR_TO_UTF8(*C.Wing->EntityName);
										const aerotelic::WingModel* W = C.Wing->GetModelAs<aerotelic::WingModel>();
										if (W != nullptr && C.Flap < static_cast<int32>(W->flaps.size())) Flap = W->flaps[C.Flap].name;
									}
									if (Case.control_entity == Entity && Case.control_flap == Flap) return;
									Case.control_entity = Entity;
									Case.control_flap = Flap;
									Self->InvalidateAero(i);
									Self->Commit();
									Self->RebuildAeroList();
									Self->RefreshAeroPoints();
								})
								[ SNew(STextBlock).Text_Lambda([Self, i]() { return i < static_cast<int32>(Self->Spec.aero.size()) && Self->ControlNames.IsValidIndex(Self->ControlIndexOf(Self->Spec.aero[i])) ? FText::FromString(*Self->ControlNames[Self->ControlIndexOf(Self->Spec.aero[i])]) : FText::GetEmpty(); }) ]
							]
							+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0).VAlign(VAlign_Center)
							[
								SAssignNew(BodiesButton, SButton).Text(FText::FromString(FString::Printf(TEXT("%d bod%s..."), static_cast<int32>(A.bodies.size()), A.bodies.size() == 1 ? TEXT("y") : TEXT("ies"))))
								.OnClicked_Lambda([Self, i, &BodiesButton]() { return FReply::Handled(); })
							]
							+ SHorizontalBox::Slot().FillWidth(1.4f).VAlign(VAlign_Center)
							[
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0) [ Button(LOCTEXT("Select", "Select"), [Self, i]() { Self->AeroSelected = i; Self->RebuildAeroList(); Self->RefreshEnvelope(); }) ]
								+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0) [ Button(LOCTEXT("MeshCase", "Mesh"), [Self, i]() { Self->ShowAeroMesh(i); }) ]
								+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0) [ Button(LOCTEXT("SolveCase", "Solve"), [Self, i]() { Self->SolveAero(i); }) ]
								+ SHorizontalBox::Slot().AutoWidth() [ Button(LOCTEXT("Delete", "Delete"), [Self, i]() { Self->DeleteAeroCase(i); }) ]
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(2, 2, 0, 0)
						[ SNew(STextBlock).AutoWrapText(true).Text(FText::FromString(StatusText)).ColorAndOpacity(FSlateColor(bSolved ? FLinearColor(0.6f, 0.9f, 0.6f) : FLinearColor(0.8f, 0.8f, 0.8f))) ]
					]
				];
				// The bodies button opens the picker popup anchored on itself.
				if (BodiesButton.IsValid())
				{
					TWeakPtr<SButton> WeakBtn = BodiesButton;
					BodiesButton->SetOnClicked(FOnClicked::CreateLambda([Self, i, WeakBtn]()
					{
						if (TSharedPtr<SButton> B = WeakBtn.Pin()) Self->EditAeroBodies(i, B.ToSharedRef());
						return FReply::Handled();
					}));
				}
			}
		}

		// ---- structural ----

		FString UniqueStaticName() const
		{
			for (int32 k = 1; ; ++k)
			{
				const std::string Name = "Case " + std::to_string(k);
				bool bTaken = false;
				for (const aerotelic::StaticLoadCaseSpec& C : Spec.structural) bTaken = bTaken || C.name == Name;
				if (!bTaken) return UTF8_TO_TCHAR(Name.c_str());
			}
		}

		// The editors' rows into the selected case.
		void SaveCurrentStatic()
		{
			if (StaticSelected < 0 || StaticSelected >= static_cast<int32>(Spec.structural.size())) return;
			aerotelic::StaticLoadCaseSpec Out = Fea->SaveStatic();
			Out.name = Spec.structural[StaticSelected].name;   // the table's field owns the name
			Spec.structural[StaticSelected] = Out;
			Commit();
		}

		// The selected case into the editors: a fresh picker with its bodies.
		void SelectStatic(int32 Index, bool bSaveFirst = true)
		{
			if (bSaveFirst) SaveCurrentStatic();
			StaticSelected = Index;
			TSharedPtr<FDialog> Self = AsShared();
			if (Index < 0 || Index >= static_cast<int32>(Spec.structural.size()))
			{
				Fea->ChosenFixed.Reset();
				Fea->Picker.Reset();
				if (BodiesBox.IsValid()) BodiesBox->SetContent(SNew(STextBlock).AutoWrapText(true).Text(LOCTEXT("NoStatic", "No structural case is selected: add one above, or select a row.")));
				if (LblStatic.IsValid()) LblStatic->SetText(FText::GetEmpty());
				Fea->Built.Reset();
				Fea->RebuildPointList();
				Fea->RebuildSupportList();
				RebuildStaticList();
				return;
			}
			const TArray<FString> Missing = Fea->LoadStatic(Spec.structural[Index]);
			Fea->Built.Reset();
			if (BodiesBox.IsValid())
			{
				BodiesBox->SetContent(
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SAssignNew(Fea->Picker, SAerotelicBodyPicker).Choices(Choices).InitiallyChosen(Fea->ChosenFixed).ListHeight(170.0f)
						.OnChanged(FSimpleDelegate::CreateLambda([Self]() { Self->Fea->DescribeModel(); Self->SaveCurrentStatic(); }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4) [ Stat(Fea->LblMaterials) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("BodiesHint",
							"One row per SOLID BODY: a wing's main element and its flap, a propeller's blades, a boolean result's separate shells. Each is analysed on ITS OWN GRID -- the triangles its Mesh tab defines. What each is comes from its Structure in the Scene tree: shells of its skin thickness and material, and each generated layer as unloaded shells inside. "
							"The bodies are not joined to one another: each must reach a support on its own, or through a body it was fused with; whatever has no load path is left out automatically and named. A boolean result already contains its operands, so those start unchosen."))
					]);
			}
			if (LblStatic.IsValid())
			{
				LblStatic->SetText(FText::FromString(FString::Printf(TEXT("Editing '%s'.%s Build Model to see it on the mesh; the Loads and Support tabs edit and apply."),
					UTF8_TO_TCHAR(Spec.structural[Index].name.c_str()), Missing.Num() ? *(TEXT(" Not in the scene: ") + FString::Join(Missing, TEXT(", ")) + TEXT(".")) : TEXT(""))));
			}
			Fea->RebuildPointList();
			Fea->RebuildSupportList();
			Fea->DescribeModel();
			RebuildStaticList();
		}

		void AddStatic()
		{
			SaveCurrentStatic();
			aerotelic::StaticLoadCaseSpec C;
			C.name = TCHAR_TO_UTF8(*UniqueStaticName());
			C.load_factor = 1.0;
			for (const FAerotelicBodyChoicePtr& B : DefaultChosen(Choices)) C.bodies.push_back(FState::BodyOf(*B));
			aerotelic::LoadCaseSupportSpec S;   // y = 0, the root of a mirrored half
			C.supports.push_back(S);
			Spec.structural.push_back(C);
			Commit();
			SelectStatic(static_cast<int32>(Spec.structural.size()) - 1, false);
		}

		void DeleteStatic(int32 Index)
		{
			if (Index < 0 || Index >= static_cast<int32>(Spec.structural.size())) return;
			if (Index != StaticSelected) SaveCurrentStatic();
			Spec.structural.erase(Spec.structural.begin() + Index);
			Commit();
			int32 Next = StaticSelected;
			if (Index == StaticSelected) Next = FMath::Min(Index, static_cast<int32>(Spec.structural.size()) - 1);
			else if (Index < StaticSelected) Next = StaticSelected - 1;
			SelectStatic(Next, false);
		}

		void RebuildStaticList()
		{
			if (!StaticList.IsValid()) return;
			StaticList->ClearChildren();
			TSharedPtr<FDialog> Self = AsShared();
			if (Spec.structural.empty())
			{
				StaticList->AddSlot().AutoHeight() [ SNew(STextBlock).AutoWrapText(true).Text(LOCTEXT("NoStaticRows", "No structural case yet: Add one.")) ];
				return;
			}
			StaticList->AddSlot().AutoHeight().Padding(0, 0, 0, 2)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(2.0f) [ SNew(STextBlock).Text(LOCTEXT("HName", "Name")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f) [ SNew(STextBlock).Text(LOCTEXT("HLoadFactor", "Load factor, g")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(3.0f) [ SNew(STextBlock).Text(LOCTEXT("HSummary", "Constraints and loads")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
				+ SHorizontalBox::Slot().FillWidth(1.2f) [ SNew(STextBlock).Text(FText::GetEmpty()) ]
			];
			for (int32 i = 0; i < static_cast<int32>(Spec.structural.size()); ++i)
			{
				const aerotelic::StaticLoadCaseSpec& C = Spec.structural[i];
				const bool bSel = i == StaticSelected;
				const FString Summary = FString::Printf(TEXT("%s; %d point force(s); pressure %g %s; %d bod%s"),
					C.free_flight ? TEXT("free flight") : *FString::Printf(TEXT("%d support plane(s)"), static_cast<int32>(C.supports.size())),
					static_cast<int32>(C.point_forces.size()), C.pressure, *UnitName(Units.StressName()), static_cast<int32>(C.bodies.size()), C.bodies.size() == 1 ? TEXT("y") : TEXT("ies"));
				StaticList->AddSlot().AutoHeight().Padding(0, 0, 0, 3)
				[
					SNew(SBorder).Padding(4).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(bSel ? FLinearColor(0.16f, 0.2f, 0.3f) : FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(2.0f).Padding(0, 0, 4, 0).VAlign(VAlign_Center)
						[
							SNew(SEditableTextBox).Text(FText::FromString(UTF8_TO_TCHAR(C.name.c_str())))
							.OnTextCommitted_Lambda([Self, i](const FText& T, ETextCommit::Type)
							{
								const FString N = T.ToString().TrimStartAndEnd();
								if (N.IsEmpty() || i >= static_cast<int32>(Self->Spec.structural.size())) return;
								for (int32 k = 0; k < static_cast<int32>(Self->Spec.structural.size()); ++k) if (k != i && UTF8_TO_TCHAR(Self->Spec.structural[k].name.c_str()) == N) { Self->Status(LOCTEXT("NameTaken2", "Another structural case has that name.")); Self->RebuildStaticList(); return; }
								Self->Spec.structural[i].name = TCHAR_TO_UTF8(*N);
								if (i == Self->StaticSelected) Self->Fea->CaseName = N;
								Self->Commit();
								Self->RebuildStaticList();
							})
						]
						+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 4, 0).VAlign(VAlign_Center)
						[
							SNew(SAerotelicNumericEntry)
							.Value_Lambda([Self, i]() { return i < static_cast<int32>(Self->Spec.structural.size()) ? Self->Spec.structural[i].load_factor : 0.0; })
							.MinValue(-100.0).MaxValue(100.0).Delta(0.5).Suffix(FText::FromString(TEXT(" g"))).LiveUpdate(false)
							.OnValueCommitted_Lambda([Self, i](double V, ETextCommit::Type)
							{
								if (i >= static_cast<int32>(Self->Spec.structural.size())) return;
								Self->Spec.structural[i].load_factor = V;
								if (i == Self->StaticSelected) Self->Fea->LoadFactor = V;
								Self->Commit();
							})
						]
						+ SHorizontalBox::Slot().FillWidth(3.0f).Padding(0, 0, 4, 0).VAlign(VAlign_Center) [ SNew(STextBlock).AutoWrapText(true).Text(FText::FromString(Summary)) ]
						+ SHorizontalBox::Slot().FillWidth(1.2f).VAlign(VAlign_Center)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0) [ Button(LOCTEXT("Select", "Select"), [Self, i]() { Self->SelectStatic(i); }) ]
							+ SHorizontalBox::Slot().AutoWidth() [ Button(LOCTEXT("Delete", "Delete"), [Self, i]() { Self->DeleteStatic(i); }) ]
						]
					]
				];
			}
		}

		void Close()
		{
			SaveCurrentStatic();
			for (auto& Pair : Runners) if (Pair.Value.IsValid()) Pair.Value->Cancel();
			Runners.Empty();
			AerotelicAeroTrim::HideAeroMeshInViewport();
			if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
			{
				if (FieldsHandle.IsValid()) Doc->OnAeroLoadCasesChanged.Remove(FieldsHandle);
			}
			FieldsHandle.Reset();
			Plot.Reset(); AeroList.Reset(); LblEnvelope.Reset();
			StaticList.Reset(); BodiesBox.Reset(); StaticTabs.Reset(); LblStatic.Reset(); Tabs.Reset();
			Fea->Close();
		}
	};

	const TCHAR* kHelp = TEXT(
		"LOAD CASES\n\n"
		"What the structural analysis (Analysis > Structural) is run for, defined once here and saved with the project (load_cases: in the file). Two kinds, two tabs; the Kratos dialog picks one, renders it on the built model and solves it.\n\n"
		"AERO: THE V-n DIAGRAM\n"
		"The flight envelope: the positive stall boundary n = ½ ρ V² CL_max S / W rises from the origin to the structural limit n_max at the manoeuvre speed V_A, the limit runs out to the never-exceed speed V_ne, down the V_ne line to n_min, back along n_min to where the negative stall boundary (CL_min) meets it, and down that boundary to the origin. The stall speed V_S (n = 1) and V_A are marked; the boundaries continue faintly beyond the limits. The weight and the reference area default to the mass report of the selected case's bodies (with the project's point masses) and the chosen wings' planform area, or are typed. "
		"CLICK inside the plot to add a load case at that speed and load factor; press a point to select it, drag it to move it. Each point is a row of the table: its name, speed, air density and load factor (edit them there), the flap the trim turns (the aftmost by default; none trims the lift alone), the solid bodies it flies (the Bodies button; a boolean result panels its lofted operands and is the structural model), Solve, Delete. "
		"SOLVE runs the high order panel method's trim for the case: the configuration at the case's speed and density, trimmed to lift n W through the mass centre and no moment about it by the angle of attack and the flap (three solves for the derivatives, one at the trimmed state), and its surface pressures are stored as the case's field, a filled point on the plot. In the Kratos dialog the case flies free (inertia relief, gravity at 1 g) under those pressures, so the structure feels n at its mass centre. Any edit of a case's speed, density, load factor, control or bodies clears its field; the Kratos dialog's Build Model solves it again when needed. A point outside the envelope is drawn in red and is not refused.\n\n"
		"STRUCTURAL: BODIES, LOADS AND CONSTRAINTS\n"
		"The table at the top: one row per case with its name and inertial load factor (both editable), what it carries, Select and Delete; Add makes a new one with the scene's bodies and a y = 0 support. Below, the selected case's editors on four tabs: Bodies (the picker of solid bodies), Loads (the load factor, a uniform pressure and the point forces, with the picture of them on the built model), Masses (the project's point masses, read-only) and Support (clamped at support planes and patches, or free flight with a reference point, with the picture of the fixed nodes; drag a patch's handles there). "
		"Build Model builds the selected case's bodies -- their own grids, the ribs through gmsh -- so the pictures show the real model; Apply on a tab pushes its edits onto it. Every edit is saved to the case as you make it. The Kratos dialog shows the same pictures of the same model, read-only, and solves.\n\n"
		"UNITS\n"
		"Speeds and densities are SI (m/s, kg/m³), as the air is; the weight is newtons and the area square metres. Coordinates, tolerances and patches are the project's units, FULL-SCALE (as the parameterisations are); forces are newtons; the uniform pressure is the project's stress unit.");
}

void ShowAerotelicLoadCasesDialog(TFunction<void(const FText&)> Status)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	TArray<FAerotelicBodyChoicePtr> Choices = AnalysableChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoBody", "No body in the scene to define a load case for: add a body first. Any body with a structural material and a skin thickness under Structure... in the Scene tree can be analysed."),
			LOCTEXT("Title", "Load Cases"));
		return;
	}

	const FVector2D Size(1240, 980);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Load Cases"))
		.ClientSize(Size)
		.SupportsMaximize(true)
		.SupportsMinimize(true);

	TSharedRef<FDialog> D = MakeShared<FDialog>();
	TSharedRef<FState> S = MakeShared<FState>();
	D->Fea = S;
	D->Status = Status;
	D->Window = Window;
	D->Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	D->Choices = Choices;
	if (Doc != nullptr) D->Spec = Doc->GetLoadCases();
	S->Status = Status;
	S->Window = Window;
	S->Units = D->Units;
	FillOptions(S, Choices);
	D->RefreshControls();
	D->AeroSelected = D->Spec.aero.empty() ? -1 : 0;
	D->StaticSelected = D->Spec.structural.empty() ? -1 : 0;

	auto VnSpin = [D](double aerotelic::VnDiagramSpec::*Member, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([D, Member]() { return D->Spec.vn.*Member; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix).LiveUpdate(false)
			.OnValueCommitted_Lambda([D, Member](double V, ETextCommit::Type) { D->Spec.vn.*Member = V; D->Commit(); D->RefreshEnvelope(); });
	};

	// ---- tab 1: aero ----
	TSharedRef<SWidget> AeroInputs =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("EnvelopeGroup", "Envelope"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("ClMax", "CL_max:"), VnSpin(&aerotelic::VnDiagramSpec::cl_max, -5.0, 5.0, 0.1, FText::GetEmpty()), 120.0f) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("ClMin", "CL_min:"), VnSpin(&aerotelic::VnDiagramSpec::cl_min, -5.0, 5.0, 0.1, FText::GetEmpty()), 120.0f) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Vne", "V_ne:"), VnSpin(&aerotelic::VnDiagramSpec::v_ne, 0.1, 1000.0, 1.0, FText::FromString(TEXT(" m/s"))), 120.0f) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("NMax", "n_max:"), VnSpin(&aerotelic::VnDiagramSpec::n_max, -20.0, 20.0, 0.5, FText::FromString(TEXT(" g"))), 120.0f) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("NMin", "n_min:"), VnSpin(&aerotelic::VnDiagramSpec::n_min, -20.0, 20.0, 0.5, FText::FromString(TEXT(" g"))), 120.0f) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Density", "Air density:"), VnSpin(&aerotelic::VnDiagramSpec::density, 0.001, 10.0, 0.025, FText::FromString(TEXT(" kg/m³"))), 120.0f) ])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("WeightGroup", "Weight and reference area"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					SNew(SCheckBox)
					.IsChecked_Lambda([D]() { return D->Spec.vn.weight_auto ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
					.OnCheckStateChanged_Lambda([D](ECheckBoxState St) { D->Spec.vn.weight_auto = St == ECheckBoxState::Checked; D->Commit(); D->RefreshEnvelope(); })
					[ SNew(STextBlock).Text(LOCTEXT("WeightAuto", "Weight from the mass report of the selected case's bodies")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ SNew(SBox).IsEnabled_Lambda([D]() { return !D->Spec.vn.weight_auto; }) [ FormRow(LOCTEXT("Weight", "Weight:"), VnSpin(&aerotelic::VnDiagramSpec::weight, 0.0, 1e9, 1.0, FText::FromString(TEXT(" N"))), 120.0f) ] ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 2)
				[
					SNew(SCheckBox)
					.IsChecked_Lambda([D]() { return D->Spec.vn.area_auto ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
					.OnCheckStateChanged_Lambda([D](ECheckBoxState St) { D->Spec.vn.area_auto = St == ECheckBoxState::Checked; D->Commit(); D->RefreshEnvelope(); })
					[ SNew(STextBlock).Text(LOCTEXT("AreaAuto", "Area from the chosen wings' planform")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ SNew(SBox).IsEnabled_Lambda([D]() { return !D->Spec.vn.area_auto; }) [ FormRow(LOCTEXT("Area", "Area S:"), VnSpin(&aerotelic::VnDiagramSpec::area, 0.0, 1e6, 0.1, FText::FromString(TEXT(" m²"))), 120.0f) ] ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ Stat(D->LblEnvelope) ])
		]
		+ SScrollBox::Slot()
		[
			MakeInfoLabel(LOCTEXT("AeroHint",
				"Click inside the envelope to add a load case there; press a point to select it, drag to move it. The table below edits each case; Mesh shows the triangles the panel method will solve in the 3D view with their statistics (each body's own grid, overlapping bodies fused into one surface); Solve trims the case with the panel method on that mesh and stores its pressures for the structural analysis. Speeds, densities, weight and area are SI."))
		];

	TSharedRef<SWidget> AeroTab =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
		[ SNew(SBox).WidthOverride(330) [ AeroInputs ] ]
		+ SHorizontalBox::Slot().FillWidth(1.0f)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(0.55f)
			[ SAssignNew(D->Plot, SAerotelicVnPlot) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("AddAtVA", "Add at V_A, n_max"), [D]() { const double VA = D->Plot.IsValid() ? D->Plot->ManoeuvreSpeed() : 0.0; D->AddAeroCase(VA > 0.0 ? FMath::Min(VA, D->Spec.vn.v_ne) : 0.7 * D->Spec.vn.v_ne, D->Spec.vn.n_max); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("AddCruise", "Add cruise (n = 1)"), [D]() { D->AddAeroCase(0.6 * D->Spec.vn.v_ne, 1.0); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("SolveAll", "Solve every unsolved case"), [D]()
				{
					const UAerotelicDocumentSubsystem* Doc2 = UAerotelicDocumentSubsystem::Get();
					for (int32 i = 0; i < static_cast<int32>(D->Spec.aero.size()); ++i)
						if (Doc2 == nullptr || !Doc2->FindAeroLoadCase(UTF8_TO_TCHAR(D->Spec.aero[i].name.c_str())).IsValid()) D->SolveAero(i);
				}) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("HideMesh", "Hide mesh"), [D]() { AerotelicAeroTrim::HideAeroMeshInViewport(); D->Status(LOCTEXT("MeshHidden", "The aero mesh is out of the 3D view.")); }) ]
			]
			+ SVerticalBox::Slot().FillHeight(0.45f)
			[
				SNew(SScrollBox)
				+ SScrollBox::Slot() [ SAssignNew(D->AeroList, SVerticalBox) ]
			]
		];
	D->Plot->OnAddPoint = [D](double V, double N) { D->AddAeroCase(V, N); };
	D->Plot->OnSelectPoint = [D](int32 I) { D->AeroSelected = I; D->RebuildAeroList(); D->RefreshEnvelope(); };
	D->Plot->OnMovePoint = [D](int32 I, double V, double N)
	{
		if (I < 0 || I >= static_cast<int32>(D->Spec.aero.size())) return;
		D->Spec.aero[I].speed = FMath::Max(V, 0.1);
		D->Spec.aero[I].load_factor = N;
	};
	D->Plot->OnMoveEnd = [D](int32 I) { D->InvalidateAero(I); D->Commit(); D->RebuildAeroList(); D->RefreshEnvelope(); };

	// ---- tab 2: structural ----
	TArray<FAerotelicTab> SubTabs;
	SubTabs.Add({ LOCTEXT("BodiesSub", "Bodies"), SNew(SScrollBox) + SScrollBox::Slot() [ SAssignNew(D->BodiesBox, SBox) ] });
	// The Mesh picture (2026-11-15): the shells the case is built on, with
	// their edges -- what is analysed, in plain sight before anything is solved.
	SubTabs.Add({ LOCTEXT("MeshSub", "Mesh"), MakePictureTab(S, TabMesh, LOCTEXT("MeshEmpty", "Build Model to see the shells the case is built on: the wetted faces by region with their edges and the rib, spar and stringer shells inside -- the triangles every analysis uses."), nullptr) });
	SubTabs.Add({ LOCTEXT("LoadsSub", "Loads"), MakePictureTab(S, TabLoads, LOCTEXT("LoadsEmpty", "Build Model to see the loads as vectors on the mesh: the pressure on the wetted faces, each point force at its node, the inertial load at the centre of mass."), MakeLoadsPanel(S)) });
	SubTabs.Add({ LOCTEXT("MassesSub", "Masses"), MakePictureTab(S, TabMasses, LOCTEXT("MassesEmpty", "Build Model to see the mass items hung on it, each with bars to the nodes that carry it, and the mass centre of everything."), MakeMassesPanel(S)) });
	SubTabs.Add({ LOCTEXT("SupportSub", "Support"), MakePictureTab(S, TabSupport, LOCTEXT("SupportEmpty", "Build Model to see the constraints: a marker on every node the supports fix, each plane with its patch handles (drag them), and what reaches no support -- or, in free flight, the one reference node."), MakeSupportPanel(S)) });
	WirePatchHandles(S);

	TSharedRef<SWidget> StaticTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("StaticGroup", "Structural load cases"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[ SNew(SBox).MaxDesiredHeight(190) [ SNew(SScrollBox) + SScrollBox::Slot() [ SAssignNew(D->StaticList, SVerticalBox) ] ] ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0) [ Button(LOCTEXT("AddStatic", "Add case"), [D]() { D->AddStatic(); }) ]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0) [ SAssignNew(S->BuildButton, SButton).Text(LOCTEXT("BuildBtn", "Build Model")).HAlign(HAlign_Center).OnClicked_Lambda([D]() { D->SaveCurrentStatic(); if (D->Fea->BuildModel() && D->StaticTabs.IsValid()) D->StaticTabs->SetActiveTab(1); return FReply::Handled(); }) ]
					+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center) [ SAssignNew(D->LblStatic, STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f))) ]
				])
		]
		+ SVerticalBox::Slot().FillHeight(1.0f)
		[ SAssignNew(D->StaticTabs, SAerotelicTabStrip, SubTabs) ];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("AeroTab", "Aero (V-n diagram)"), AeroTab });
	Tabs.Add({ LOCTEXT("StaticTab", "Structural"), StaticTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(FText::FromString(kHelp)) });

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SAssignNew(D->Tabs, SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([D](const TSharedRef<SWindow>&) { D->Close(); }));

	if (Doc != nullptr)
	{
		TWeakPtr<FDialog> Weak = D;
		D->FieldsHandle = Doc->OnAeroLoadCasesChanged.AddLambda([Weak]() { if (TSharedPtr<FDialog> P = Weak.Pin()) { P->RebuildAeroList(); P->RefreshAeroPoints(); } });
	}
	D->SelectStatic(D->StaticSelected, false);
	S->RebuildMassList();
	D->RebuildAeroList();
	D->RefreshEnvelope();
	Status(LOCTEXT("Ready", "Define the load cases: click the V-n diagram for an aero case, or add a structural case."));
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
