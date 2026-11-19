#include "HighOrderPanelDialog.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Containers/Ticker.h"
#include "ImageUtils.h"
#include "Async/Async.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/SceneView.h"
#include "Widgets/ColorBar.h"
#include "Widgets/LinePlot.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Standalone/AerotelicColoredMeshActor.h"
#include "Standalone/AerotelicCameraPawn.h"
#include "ProceduralMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Materials/Material.h"
#include "ShaderCompiler.h"
#include "UnrealClient.h"

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

// Last: OCCT's Handle() macro vs engine headers.
#include "aerotelic/colormaps.hpp"
#include "Entity/AerotelicAeroTrim.h"
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "HighOrderPanelDialog"

namespace
{
	using aerotelic::project_yaml::EntityType;
	namespace tripanel = aerotelic::tripanel;
	// The meshing, the cases and their derivation live in the shared module
	// (2026-11-15): Model > Load Cases trims with the same code.
	using AerotelicAeroTrim::FCase;
	using AerotelicAeroTrim::FShare;
	using AerotelicAeroTrim::FCoefs;
	using AerotelicAeroTrim::kLoadCuts;

	TArray<FAerotelicBodyChoicePtr> PanelChoices()
	{
		TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
		// One row per entity: a mirrored entity is solved with its image (or
		// its reflected copy), never as a second pick.
		Choices.RemoveAll([](const FAerotelicBodyChoicePtr& C)
		{
			return !C->Operand.Entity.IsValid() || C->Operand.bMirror || C->Operand.Entity->GetModel() == nullptr || !AerotelicAeroTrim::IsPanelable(C->Operand.Entity->GetEntityType());
		});
		return Choices;
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field, float LabelWidth = 190.0f)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Top).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(LabelWidth) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-"))).AutoWrapText(true)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
	}

	TSharedRef<SWidget> Button(const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center).OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	}

	FString DefaultSolutionPath()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_panels.vtk"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_panels.vtk"));
	}

	// "4" or "-2, 0, 2, 4": a comma separated list of angles in degrees.
	bool ParseAngleList(const FString& Text, TArray<double>& Out, FString& OutError)
	{
		Out.Reset();
		TArray<FString> Parts;
		Text.ParseIntoArray(Parts, TEXT(","), false);
		for (FString P : Parts)
		{
			P.TrimStartAndEndInline();
			if (P.IsEmpty())
			{
				OutError = TEXT("an empty entry in the list");
				return false;
			}
			double V = 0.0;
			if (!LexTryParseString(V, *P) || !FMath::IsFinite(V))
			{
				OutError = FString::Printf(TEXT("'%s' is not a number"), *P);
				return false;
			}
			if (FMath::Abs(V) > 89.0)
			{
				OutError = FString::Printf(TEXT("%g degrees is outside -89..89"), V);
				return false;
			}
			Out.Add(V);
		}
		if (Out.Num() == 0)
		{
			OutError = TEXT("give at least one angle");
			return false;
		}
		return true;
	}

	// The panels of every body as triangles with their results, legacy VTK
	// polydata in SI lengths (as the Kratos route's file is), so a viewer
	// can colour the bodies by Cp.
	bool WritePanelsVtk(const FString& Path, const FCase& C, double LengthToSI, FString& OutError)
	{
		FString S;
		S += TEXT("# vtk DataFile Version 3.0\nAerotelic panel method on triangles: panels, doublet/source strength, pressure coefficient\nASCII\nDATASET POLYDATA\n");
		int32 NPoints = 0, NPanels = 0;
		for (const tripanel::BodyResult& B : C.Sol.bodies) { NPoints += static_cast<int32>(B.mesh.vertices.size()); NPanels += static_cast<int32>(B.mesh.triangles.size()); }
		S += FString::Printf(TEXT("POINTS %d double\n"), NPoints);
		for (const tripanel::BodyResult& B : C.Sol.bodies)
			for (const aerotelic::Vec3& P : B.mesh.vertices) S += FString::Printf(TEXT("%.9g %.9g %.9g\n"), P.x * LengthToSI, P.y * LengthToSI, P.z * LengthToSI);
		S += FString::Printf(TEXT("POLYGONS %d %d\n"), NPanels, 4 * NPanels);
		int32 Base = 0;
		for (const tripanel::BodyResult& B : C.Sol.bodies)
		{
			for (const auto& T : B.mesh.triangles) S += FString::Printf(TEXT("3 %d %d %d\n"), Base + T[0], Base + T[1], Base + T[2]);
			Base += static_cast<int32>(B.mesh.vertices.size());
		}
		S += FString::Printf(TEXT("CELL_DATA %d\n"), NPanels);
		auto Field = [&](const TCHAR* Name, TFunction<double(const tripanel::PanelResult&)> Get)
		{
			S += FString::Printf(TEXT("SCALARS %s double 1\nLOOKUP_TABLE default\n"), Name);
			for (const tripanel::BodyResult& B : C.Sol.bodies)
				for (const tripanel::PanelResult& P : B.panels) S += FString::Printf(TEXT("%.9g\n"), Get(P));
		};
		Field(TEXT("Cp"), [](const tripanel::PanelResult& P) { return P.cp; });
		Field(TEXT("doublet"), [](const tripanel::PanelResult& P) { return P.mu; });
		Field(TEXT("source"), [](const tripanel::PanelResult& P) { return P.sigma; });
		S += TEXT("SCALARS body int 1\nLOOKUP_TABLE default\n");
		for (size_t k = 0; k < C.Sol.bodies.size(); ++k)
			for (size_t p = 0; p < C.Sol.bodies[k].panels.size(); ++p) S += FString::Printf(TEXT("%d\n"), static_cast<int32>(k));
		S += TEXT("VECTORS velocity double\n");
		for (const tripanel::BodyResult& B : C.Sol.bodies)
			for (const tripanel::PanelResult& P : B.panels) S += FString::Printf(TEXT("%.9g %.9g %.9g\n"), P.velocity.x, P.velocity.y, P.velocity.z);
		if (!FFileHelper::SaveStringToFile(S, *Path))
		{
			OutError = TEXT("Could not write ") + Path;
			return false;
		}
		return true;
	}

	struct FJob
	{
		TArray<TSharedPtr<FCase>> Cases;
		tripanel::Options Options;
		std::atomic<bool> bCancel{ false };
		FString Error;
	};

	// The coloured mesh of a case: every panel as its own flat facet (three
	// vertices, both windings -- the engine's two-sided convention, as the
	// bodies are drawn) carrying its own Cp, and the image when the solve
	// had the plane. Flat facets show the panels as the panels they are.
	struct FCpMesh
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<double> Cp; // per vertex
	};

	FCpMesh BuildCpMesh(const FCase& C)
	{
		FCpMesh M;
		for (const tripanel::BodyResult& B : C.Sol.bodies)
		{
			for (size_t t = 0; t < B.mesh.triangles.size(); ++t)
			{
				const tripanel::PanelResult& P = B.panels[t];
				const auto& T = B.mesh.triangles[t];
				for (int32 Side = 0; Side < (C.bImagedSolve ? 2 : 1); ++Side)
				{
					const int32 Base = M.Vertices.Num();
					for (int32 k = 0; k < 3; ++k)
					{
						aerotelic::Vec3 V = B.mesh.vertices[T[k]];
						if (Side == 1) V.y = -V.y;
						M.Vertices.Add(FVector(V.x, V.y, V.z));
						M.Normals.Add(FVector(P.normal.x, Side == 1 ? -P.normal.y : P.normal.y, P.normal.z));
						M.Cp.Add(P.cp);
					}
					M.Triangles.Append({ Base, Base + 1, Base + 2, Base, Base + 2, Base + 1 });
				}
			}
		}
		return M;
	}

	TArray<FColor> ColorsFor(const FCpMesh& M, const aerotelic::Colormap& Map, double Lo, double Hi)
	{
		TArray<FColor> Out;
		Out.Reserve(M.Cp.Num());
		const double Span = Hi - Lo;
		for (double Cp : M.Cp)
		{
			const double T = Span > 1e-12 ? (Cp - Lo) / Span : 0.5;
			const aerotelic::Rgb C = Map.Sample(T);
			Out.Add(AAerotelicColoredMeshActor::VertexColorFromSRGB(C.r, C.g, C.b));
		}
		return Out;
	}

	// ---- the dialog's state: owned by its widgets (the window is modeless) ----

	struct FState : public TSharedFromThis<FState>
	{
		TFunction<void(const FText&)> Status;
		TWeakPtr<SWindow> Window;
		aerotelic::UnitSystem Units;

		// Inputs (tab 1).
		TSharedPtr<SAerotelicBodyPicker> Picker;
		TSharedPtr<SCheckBox> SymmetryCheck, FuseCheck, ShowMeshCheck;
		bool bSymmetryTouched = false;
		double WakeChords = 20.0;
		double ReferenceArea = 1.0, ReferenceLength = 1.0;
		TSharedPtr<SEditableTextBox> AlphaBox, BetaBox, PathBox;
		TSharedPtr<STextBlock> LblWarn, LblMesh, LblSummary, LblInput;
		TSharedPtr<SButton> SolveButton;
		// The mesh as it would be solved, built on every change of the bodies
		// (the mesh report and the viewport copy read it).
		TSharedPtr<FCase> MeshCase;

		// Results.
		TArray<TSharedPtr<FCase>> Cases;
		int32 Current = 0;
		bool bSweepAlpha = true;
		TSharedPtr<FJob> Job;

		// Surface pressure (tab 2).
		TSharedPtr<SAerotelicSceneView> View;
		TSharedPtr<SAerotelicColorBar> ColorBar;
		TWeakObjectPtr<AAerotelicColoredMeshActor> MeshActor;
		const aerotelic::Colormap* Map = &aerotelic::DefaultColormap();
		double CpLo = -1.0, CpHi = 1.0;
		bool bRangeAuto = true;
		bool bFramed = false;
		bool bEdges = true;
		FCpMesh Mesh; // of the current case
		TSharedRef<TArray<TSharedPtr<FString>>> MapNames = MakeShared<TArray<TSharedPtr<FString>>>();

		// Span loading (tab 3): the view shown, an index into LoadViews().
		TSharedPtr<SAerotelicLinePlot> LoadPlot;
		int32 CurrentLoadView = 0;

		// Forces and moments (tab 4).
		TArray<TSharedPtr<SAerotelicLinePlot>> Polars;
		TSharedPtr<SWidgetSwitcher> PolarSwitch;
		TSharedPtr<SVerticalBox> PolarTable;

		TSharedPtr<SAerotelicTabStrip> Tabs;
		// -AerotelicPanelSmoke: a developer run that solves at once, steps
		// through the result tabs saving a screenshot of each to
		// Saved/Screenshots, then quits.
		bool bSmoke = false;

		TArray<AAerotelicEntityActor*> Chosen() const
		{
			TArray<AAerotelicEntityActor*> Out;
			if (!Picker.IsValid()) return Out;
			for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen())
				if (C->Operand.Entity.IsValid()) Out.Add(C->Operand.Entity.Get());
			return Out;
		}

		// The set-up as the tab has it, for the shared module.
		AerotelicAeroTrim::FSetup Setup() const
		{
			AerotelicAeroTrim::FSetup S;
			for (AAerotelicEntityActor* E : Chosen()) S.Entities.Add(E);
			S.bSymmetry = SymmetryCheck.IsValid() && SymmetryCheck->IsChecked();
			S.bFuse = !FuseCheck.IsValid() || FuseCheck->IsChecked();
			S.WakeChords = WakeChords;
			S.ReferenceArea = ReferenceArea; S.ReferenceLength = ReferenceLength;
			return S;
		}

		const FCase* CurrentCase() const { return Cases.IsValidIndex(Current) ? Cases[Current].Get() : nullptr; }

		struct FLoadView
		{
			int32 Share = -1; // -1: all bodies
			FString Name;
		};
		TArray<FLoadView> LoadViews() const
		{
			// "All bodies" first when there is more than one share, then the
			// lifting surfaces by span (the main wing before the tailplane),
			// the fins last: their "span" loading is a side force, integral
			// about zero. The stepper used to open on the first operand of a
			// boolean -- my_drone's fin -- and its side force, a hair negative,
			// read as "the lift is not elliptic" (Matt, 2026-11-16).
			TArray<FLoadView> Out;
			const FCase* C = CurrentCase();
			if (C == nullptr) return Out;
			struct FRank { int32 Share; double Span; bool bFin; };
			TArray<FRank> Ranks;
			for (int32 i = 0; i < C->Shares.Num(); ++i)
			{
				const FShare& Sh = C->Shares[i];
				if (!Sh.bWing || Sh.bMirrorCopy) continue;
				Ranks.Add({ i, FVector::Dist(Sh.RootPoint, Sh.TipPoint), FMath::Abs(Sh.SpanAxis.Z) > 0.9 });
			}
			Ranks.Sort([](const FRank& A, const FRank& B) { return A.bFin != B.bFin ? !A.bFin : A.Span > B.Span; });
			if (C->Shares.Num() > 1 || Ranks.Num() == 0) Out.Add({ -1, TEXT("All bodies") });
			for (const FRank& R : Ranks) Out.Add({ R.Share, C->Shares[R.Share].Name });
			return Out;
		}

		FText CaseLabel() const
		{
			const FCase* C = CurrentCase();
			if (C == nullptr) return LOCTEXT("NoCase", "no solution yet");
			return FText::FromString(FString::Printf(TEXT("α = %g°, β = %g°  (%d of %d)"), C->AlphaDeg, C->BetaDeg, Current + 1, Cases.Num()));
		}

		// The mesh the solver would take, and its report; the viewport copy
		// follows when the box is ticked.
		void RefreshMesh()
		{
			MeshCase.Reset();
			const AerotelicAeroTrim::FSetup S = Setup();
			FString Text;
			if (S.LiveEntities().Num() == 0)
			{
				Text = TEXT("Choose at least one body.");
			}
			else
			{
				try
				{
					MeshCase = AerotelicAeroTrim::BuildCase(S, 0.0, 0.0);
					Text = MeshCase->MeshReport;
					for (const FString& N : MeshCase->Notes) Text += TEXT("\n") + N;
					for (const FString& Sk : MeshCase->Skipped) Text += TEXT("\n") + Sk + TEXT(": skipped (no mesh)");
				}
				catch (const std::exception& E)
				{
					Text = FString(TEXT("The aero mesh could not be built: ")) + UTF8_TO_TCHAR(E.what());
				}
			}
			if (LblMesh.IsValid()) LblMesh->SetText(FText::FromString(Text));
			if (LblWarn.IsValid())
			{
				FString Warn;
				if (MeshCase.IsValid()) for (const FString& W : MeshCase->Warnings) Warn += (Warn.IsEmpty() ? TEXT("") : TEXT("\n")) + W;
				LblWarn->SetText(FText::FromString(Warn));
				LblWarn->SetVisibility(Warn.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible);
			}
			if (ShowMeshCheck.IsValid() && ShowMeshCheck->IsChecked())
			{
				if (MeshCase.IsValid()) AerotelicAeroTrim::ShowAeroMeshInViewport(*MeshCase);
				else AerotelicAeroTrim::HideAeroMeshInViewport();
			}
		}

		void Refresh()
		{
			TArray<AAerotelicEntityActor*> Es = Chosen();
			AerotelicAeroTrim::DefaultReferences(Es, ReferenceArea, ReferenceLength);
			if (!bSymmetryTouched && SymmetryCheck.IsValid()) SymmetryCheck->SetIsChecked(AerotelicAeroTrim::DefaultSymmetry(Es) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked);
			RefreshMesh();
			Status(Es.Num() ? FText::FromString(FString::Printf(TEXT("%d bodies chosen for the panel method"), Es.Num())) : LOCTEXT("PickOne", "Choose at least one body."));
		}

		void SetInputError(const FString& Err)
		{
			if (LblInput.IsValid())
			{
				LblInput->SetText(FText::FromString(Err));
				LblInput->SetVisibility(Err.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible);
			}
		}

		void Solve()
		{
			if (Job.IsValid())
			{
				AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("Busy", "A solve is still running."), LOCTEXT("Title", "Panel Method"));
				return;
			}
			TArray<AAerotelicEntityActor*> Es = Chosen();
			if (Es.Num() == 0)
			{
				AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("PickOneWarn", "Choose at least one body."), LOCTEXT("Title", "Panel Method"));
				return;
			}
			bFramed = false;
			TArray<double> Alphas, Betas;
			FString Err;
			if (!ParseAngleList(AlphaBox->GetText().ToString(), Alphas, Err))
			{
				SetInputError(TEXT("Angle of attack: ") + Err + TEXT("."));
				return;
			}
			if (!ParseAngleList(BetaBox->GetText().ToString(), Betas, Err))
			{
				SetInputError(TEXT("Angle of sideslip: ") + Err + TEXT("."));
				return;
			}
			if (Alphas.Num() > 1 && Betas.Num() > 1)
			{
				SetInputError(TEXT("Give a comma separated list for the angle of attack OR for the sideslip, not both: the solutions are a sweep of one angle at a fixed value of the other, not every combination of the two."));
				return;
			}
			const AerotelicAeroTrim::FSetup S = Setup();
			for (double B : Betas)
			{
				if (B != 0.0 && S.bSymmetry)
				{
					SetInputError(TEXT("Sideslip needs the symmetry plane off: the image assumes a free stream in the plane of symmetry."));
					return;
				}
			}
			SetInputError(FString());
			bSweepAlpha = Betas.Num() <= 1;

			Status(LOCTEXT("Meshing", "Building the aero mesh..."));
			TSharedPtr<FJob> NewJob = MakeShared<FJob>();
			try
			{
				if (bSweepAlpha)
				{
					for (double A : Alphas) NewJob->Cases.Add(AerotelicAeroTrim::BuildCase(S, A, Betas[0]));
				}
				else
				{
					for (double B : Betas) NewJob->Cases.Add(AerotelicAeroTrim::BuildCase(S, Alphas[0], B));
				}
			}
			catch (const std::exception& E)
			{
				Status(LOCTEXT("Ready", "Ready"));
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(FString(TEXT("The analysis failed:\n")) + UTF8_TO_TCHAR(E.what())), LOCTEXT("Failed", "Panel Method Failed"));
				return;
			}
			NewJob->Options = AerotelicAeroTrim::SolveOptions(S);
			Launch(NewJob);
		}

		// Runs a job's cases on a worker thread and reports back to OnSolved.
		void Launch(TSharedPtr<FJob> NewJob)
		{
			Job = NewJob;
			if (SolveButton.IsValid()) SolveButton->SetEnabled(false);
			Status(FText::FromString(FString::Printf(TEXT("Solving 1 of %d (%d panels)..."), NewJob->Cases.Num(), NewJob->Cases[0]->Triangles)));
			TWeakPtr<FState> Weak = AsShared();
			auto Work = [NewJob, Weak]()
			{
				const int32 N = NewJob->Cases.Num();
				for (int32 i = 0; i < N; ++i)
				{
					if (NewJob->bCancel.load()) break;
					TSharedPtr<FCase> C = NewJob->Cases[i];
					try
					{
						AerotelicAeroTrim::SolveCaseNow(*C, NewJob->Options);
					}
					catch (const std::exception& E)
					{
						NewJob->Error = FString::Printf(TEXT("case %d (alpha %g, beta %g): %s"), i + 1, C->AlphaDeg, C->BetaDeg, UTF8_TO_TCHAR(E.what()));
						break;
					}
					if (i + 1 < N)
					{
						AsyncTask(ENamedThreads::GameThread, [Weak, i, N]()
						{
							if (TSharedPtr<FState> S = Weak.Pin())
								if (S->Job.IsValid()) S->Status(FText::FromString(FString::Printf(TEXT("Solving %d of %d..."), i + 2, N)));
						});
					}
				}
				AsyncTask(ENamedThreads::GameThread, [NewJob, Weak]()
				{
					if (TSharedPtr<FState> S = Weak.Pin()) S->OnSolved(NewJob);
				});
			};
			if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicPanelSync")))
			{
				Work();
			}
			else
			{
				Async(EAsyncExecution::Thread, MoveTemp(Work));
			}
		}

		void OnSolved(TSharedPtr<FJob> Done)
		{
			if (Done != Job) return; // a job the dialog let go of
			Job.Reset();
			if (SolveButton.IsValid()) SolveButton->SetEnabled(true);
			if (!Done->Error.IsEmpty())
			{
				Status(LOCTEXT("Ready", "Ready"));
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("The analysis failed:\n") + Done->Error), LOCTEXT("Failed", "Panel Method Failed"));
				return;
			}
			if (Done->bCancel.load()) return;

			double Seconds = 0.0;
			double Lo = 1e9, Hi = -1e9;
			for (TSharedPtr<FCase>& C : Done->Cases)
			{
				AerotelicAeroTrim::Derive(*C, ReferenceLength);
				Seconds += C->Sol.seconds_influence + C->Sol.seconds_solve;
				Lo = FMath::Min(Lo, C->CpMin);
				Hi = FMath::Max(Hi, C->CpMax);
				UE_LOG(LogTemp, Log, TEXT("Panel method: alpha %g beta %g: C_L %.4f (wake %.4f), C_D %.4f, C_m %.4f, Cp %.3f..%.3f, %d unknowns, %s, %.2f s"),
					C->AlphaDeg, C->BetaDeg, C->CL, C->Sol.lift_from_wake.value_or(0.0), C->CD, C->Cm, C->CpMin, C->CpMax, C->Sol.unknowns,
					C->Sol.direct ? TEXT("LU") : *FString::Printf(TEXT("GMRES %d it"), C->Sol.iterations), C->Sol.seconds_influence + C->Sol.seconds_solve);
			}
			Cases = Done->Cases;
			Current = 0;
			CurrentLoadView = 0;
			if (bRangeAuto && Lo < Hi)
			{
				CpLo = Lo;
				CpHi = Hi;
			}

			// A pressure coefficient beyond anything a potential flow gives is a
			// broken fit, not a result (my_drone's boom at its cowling read Cp
			// -5e56 on four panels and C_L -2e49, 2026-11-18). The solver now
			// caps the surface speed and says so in its notes; this is the
			// backstop, named by place, so a garbage coefficient is never shown
			// as if it meant something.
			FString Rogue;
			{
				int32 NRogue = 0;
				double Worst = 0.0;
				aerotelic::Vec3 WorstAt;
				for (const TSharedPtr<FCase>& C : Done->Cases)
					for (const tripanel::BodyResult& BR : C->Sol.bodies)
						for (const tripanel::PanelResult& P : BR.panels)
							if (P.area > 0.0 && (!FMath::IsFinite(P.cp) || FMath::Abs(P.cp) > 100.0)) { ++NRogue; if (!FMath::IsFinite(P.cp) || FMath::Abs(P.cp) > FMath::Abs(Worst)) { Worst = P.cp; WorstAt = P.centre; } }
				if (NRogue > 0)
				{
					Rogue = FString::Printf(TEXT("WARNING: %d panel(s) carry a pressure coefficient beyond anything physical (worst %.3g at (%.3f, %.3f, %.3f)). The force and moment coefficients include them and are NOT to be trusted; the mesh there (a seam, a sliver) needs attention.\n"), NRogue, Worst, WorstAt.x, WorstAt.y, WorstAt.z);
					AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Rogue), LOCTEXT("Title", "Panel Method"));
				}
				for (const std::string& N : Cases[0]->Sol.notes)
					if (N.find("WARNING") != std::string::npos) Rogue += FString(UTF8_TO_TCHAR(N.c_str())) + TEXT("\n");
			}

			const FCase& First = *Cases[0];
			FString Summary = Rogue + FString::Printf(TEXT("%d solution%s: %d panels + %d wake strips, %d unknowns%s; %s; %.2f s in all.\n"),
				Cases.Num(), Cases.Num() == 1 ? TEXT("") : TEXT("s"), First.NBody, First.NWake, First.Sol.unknowns,
				Done->Options.symmetry_y ? TEXT(" (the y >= 0 half, the image standing in for the rest)") : TEXT(", no symmetry plane"),
				First.Sol.direct ? TEXT("direct solve") : *FString::Printf(TEXT("GMRES, %d iterations, residual %.1e"), First.Sol.iterations, First.Sol.residual), Seconds);
			Summary += TEXT("Aero mesh:\n") + First.MeshReport;
			for (const FString& Sk : First.Skipped) Summary += FString::Printf(TEXT("\n%s: skipped (no mesh)"), *Sk);
			for (const FString& N : First.Notes) Summary += TEXT("\n") + N;
			for (const std::string& N : First.Sol.notes) Summary += TEXT("\n") + FString(UTF8_TO_TCHAR(N.c_str()));

			// The solution files, one per case.
			const FString Path = PathBox->GetText().ToString().TrimStartAndEnd();
			FString Written;
			if (!Path.IsEmpty())
			{
				int32 NWritten = 0;
				for (const TSharedPtr<FCase>& C : Cases)
				{
					FString CasePath = Path;
					if (Cases.Num() > 1)
					{
						const FString Ext = FPaths::GetExtension(Path, true);
						const FString Base = Path.LeftChop(Ext.Len());
						CasePath = FString::Printf(TEXT("%s_%s%g%s"), *Base, bSweepAlpha ? TEXT("aoa") : TEXT("beta"), bSweepAlpha ? C->AlphaDeg : C->BetaDeg, *Ext);
					}
					FString WErr;
					if (WritePanelsVtk(CasePath, *C, Units.LengthToSI(), WErr)) ++NWritten;
					else AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(WErr), LOCTEXT("Title", "Panel Method"));
				}
				if (NWritten > 0) Written = FString::Printf(TEXT(" -> %d file%s beside %s"), NWritten, NWritten == 1 ? TEXT("") : TEXT("s"), *FPaths::GetCleanFilename(Path));
				Summary += TEXT("\nSolution files written: ") + FString::FromInt(NWritten) + TEXT(".");
			}
			if (LblSummary.IsValid()) LblSummary->SetText(FText::FromString(Summary));

			UpdateSurface(true);
			UpdateLoads();
			UpdatePolars();
			Status(FText::FromString(FString::Printf(TEXT("Panel method: %d solution%s, C_L %.4f to %.4f, Cp %.2f to %.2f, %d unknowns in %.2f s%s"),
				Cases.Num(), Cases.Num() == 1 ? TEXT("") : TEXT("s"), Cases[0]->CL, Cases.Last()->CL, Lo, Hi, First.Sol.unknowns, Seconds, *Written)));
			if (bSmoke) RunSmoke();
		}

		void Screenshot(const FString& Name)
		{
			TSharedPtr<SWindow> W = Window.Pin();
			if (!W.IsValid()) return;
			TArray<FColor> Pixels;
			FIntVector Size;
			if (!FSlateApplication::Get().TakeScreenshot(W.ToSharedRef(), Pixels, Size) || Pixels.Num() == 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("PanelMethod smoke: screenshot %s failed"), *Name);
				return;
			}
			for (FColor& C : Pixels) C.A = 255;
			TArray64<uint8> Png;
			FImageUtils::PNGCompressImageArray(Size.X, Size.Y, Pixels, Png);
			const FString Path = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Screenshots"), Name);
			FFileHelper::SaveArrayToFile(Png, *Path);
			UE_LOG(LogTemp, Log, TEXT("PanelMethod smoke: wrote %s (%d x %d)"), *Path, Size.X, Size.Y);
		}

		void RunSmoke()
		{
			TWeakPtr<FState> Weak = AsShared();
			auto Later = [](float Seconds, TFunction<void()> Fn)
			{
				FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Fn](float) { Fn(); return false; }), Seconds);
			};
			if (Tabs.IsValid()) Tabs->SetActiveTab(1);
			float FirstDelay = 3.0f;
			FParse::Value(FCommandLine::Get(), TEXT("AerotelicPanelSmokeDelay="), FirstDelay);
			if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicPanelSmokeDebug")))
			{
				Later(FirstDelay * 0.5f, [Weak]()
				{
					if (TSharedPtr<FState> S = Weak.Pin())
						if (S->View.IsValid()) S->View->SetView(-60.0, -35.0);
				});
			}
			Later(FirstDelay, [Weak, Later]()
			{
				TSharedPtr<FState> S = Weak.Pin();
				if (!S.IsValid()) return;
				if (S->MeshActor.IsValid() && S->MeshActor->GetMesh() != nullptr)
				{
					UMaterialInterface* M = S->MeshActor->GetMesh()->GetMaterial(0);
					UE_LOG(LogTemp, Log, TEXT("PanelMethod smoke: surface material %s, base %s, %d vertices; shader compiler %s, %d jobs remaining; mesh at %s"),
						*GetNameSafe(M), M ? *GetNameSafe(M->GetBaseMaterial()) : TEXT("-"), S->Mesh.Vertices.Num(),
						GShaderCompilingManager != nullptr && GShaderCompilingManager->IsCompiling() ? TEXT("BUSY") : TEXT("idle"),
						GShaderCompilingManager != nullptr ? GShaderCompilingManager->GetNumRemainingJobs() : -1,
						*S->MeshActor->GetActorLocation().ToString());
				}
				S->Screenshot(TEXT("panel_smoke_surface.png"));
				if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicPanelSmokeDebug")))
				{
					FScreenshotRequest::RequestScreenshot(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Screenshots"), TEXT("panel_smoke_viewport.png")), false, false);
				}
				if (S->Cases.Num() > 1) S->SetCurrent(1);
				if (S->Tabs.IsValid()) S->Tabs->SetActiveTab(2);
				Later(1.5f, [Weak, Later]()
				{
					TSharedPtr<FState> S = Weak.Pin();
					if (!S.IsValid()) return;
					S->Screenshot(TEXT("panel_smoke_loads.png"));
					if (S->Tabs.IsValid()) S->Tabs->SetActiveTab(3);
					Later(1.5f, [Weak]()
					{
						TSharedPtr<FState> S = Weak.Pin();
						if (!S.IsValid()) return;
						S->Screenshot(TEXT("panel_smoke_polars.png"));
						UE_LOG(LogTemp, Log, TEXT("PanelMethod smoke: done, quitting"));
						FPlatformMisc::RequestExit(false);
					});
				});
			});
		}

		// ---- tab 2: the surface pressure ----

		void UpdateSurface(bool bRebuildMesh)
		{
			const FCase* C = CurrentCase();
			if (C == nullptr || !View.IsValid()) return;
			if (!MeshActor.IsValid())
			{
				AAerotelicCameraPawn* Pawn = AAerotelicCameraPawn::FindInGameWorld();
				UWorld* World = Pawn != nullptr ? Pawn->GetWorld() : nullptr;
				if (World == nullptr) return;
				FActorSpawnParameters Params;
				Params.ObjectFlags |= RF_Transient;
				MeshActor = World->SpawnActor<AAerotelicColoredMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
				if (!MeshActor.IsValid()) return;
				TArray<AActor*> Shown;
				Shown.Add(MeshActor.Get());
				View->SetShownActors(Shown);
				if (bSmoke && FParse::Param(FCommandLine::Get(), TEXT("AerotelicPanelSmokeDebug")))
				{
					View->SetBackdropColor(FLinearColor(0.08f, 0.10f, 0.45f));
					MeshActor->SetActorLocation(FVector(0, 0, 2));
					MeshActor->SetVisibleInCapturesOnly(false);
				}
			}
			if (bRebuildMesh || Mesh.Vertices.Num() == 0)
			{
				Mesh = BuildCpMesh(*C);
				const TArray<FColor> Colors = ColorsFor(Mesh, *Map, CpLo, CpHi);
				if (bSmoke && Mesh.Cp.Num() > 0)
				{
					UE_LOG(LogTemp, Log, TEXT("PanelMethod smoke: Cp mesh %d vertices, Cp %.3f .. %.3f mapped on %.3f .. %.3f with %s; colour[0] %s, colour[mid] %s"),
						Mesh.Cp.Num(), Mesh.Cp[0], Mesh.Cp[Mesh.Cp.Num() / 2], CpLo, CpHi, UTF8_TO_TCHAR(Map->name.c_str()), *Colors[0].ToString(), *Colors[Colors.Num() / 2].ToString());
				}
				MeshActor->SetMesh(Mesh.Vertices, Mesh.Triangles, Mesh.Normals, Colors);
				MeshActor->SetEdges(bEdges);
				if (!bFramed)
				{
					View->FrameShown();
					bFramed = true;
				}
			}
			else
			{
				MeshActor->SetColors(ColorsFor(Mesh, *Map, CpLo, CpHi));
				MeshActor->SetEdges(bEdges);
			}
			View->RequestRedraw();
		}

		void SetCurrent(int32 Index)
		{
			if (!Cases.IsValidIndex(Index) || Index == Current) return;
			Current = Index;
			UpdateSurface(true);
			UpdateLoads();
		}

		void AutoRange()
		{
			double Lo = 1e9, Hi = -1e9;
			for (const TSharedPtr<FCase>& C : Cases)
			{
				Lo = FMath::Min(Lo, C->CpMin);
				Hi = FMath::Max(Hi, C->CpMax);
			}
			if (Lo < Hi)
			{
				CpLo = Lo;
				CpHi = Hi;
			}
			bRangeAuto = true;
			UpdateSurface(false);
		}

		// The colour bar's popup: the range and the map.
		void OpenColorPopup(const FPointerEvent& Event)
		{
			if (!ColorBar.IsValid()) return;
			TSharedPtr<FState> Self = AsShared();
			auto Spin = [Self](double FState::*Member, double Delta)
			{
				return SNew(SAerotelicNumericEntry)
					.Value_Lambda([Self, Member]() { return (*Self).*Member; })
					.Delta(Delta).LiveUpdate(false)
					.OnValueCommitted_Lambda([Self, Member](double V, ETextCommit::Type)
					{
						(*Self).*Member = V;
						if (Self->CpHi <= Self->CpLo) Self->CpHi = Self->CpLo + 1e-6;
						Self->bRangeAuto = false;
						Self->UpdateSurface(false);
					});
			};
			TSharedRef<SWidget> Content =
				SNew(SBorder).Padding(10)
				[
					SNew(SBox).WidthOverride(300)
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
						[ SNew(STextBlock).Text(LOCTEXT("PopupTitle", "Pressure coefficient colour scale")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Min", "Minimum:"), Spin(&FState::CpLo, 0.1), 110.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Max", "Maximum:"), Spin(&FState::CpHi, 0.1), 110.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[
							FormRow(LOCTEXT("MapLabel", "Colour map:"),
								SNew(SComboBox<TSharedPtr<FString>>)
								.OptionsSource(&*Self->MapNames)
								.InitiallySelectedItem((*Self->MapNames)[FMath::Max(0, Self->MapNames->IndexOfByPredicate([Self](const TSharedPtr<FString>& N) { return *N == UTF8_TO_TCHAR(Self->Map->name.c_str()); }))])
								.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
								.OnSelectionChanged_Lambda([Self](TSharedPtr<FString> S, ESelectInfo::Type)
								{
									if (!S.IsValid()) return;
									if (const aerotelic::Colormap* M = aerotelic::FindColormap(TCHAR_TO_UTF8(**S)))
									{
										Self->Map = M;
										Self->UpdateSurface(false);
									}
								})
								[ SNew(STextBlock).Text_Lambda([Self]() { return FText::FromString(UTF8_TO_TCHAR(Self->Map->name.c_str())); }) ], 110.0f)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4)
						[
							SNew(SCheckBox).IsChecked_Lambda([Self]() { return Self->bEdges ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
							.OnCheckStateChanged_Lambda([Self](ECheckBoxState S) { Self->bEdges = S == ECheckBoxState::Checked; Self->UpdateSurface(false); })
							[ SNew(STextBlock).Text(LOCTEXT("Edges", "Draw the panels' edges")) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0).HAlign(HAlign_Left)
						[ Button(LOCTEXT("AutoRange", "Auto range (all solutions)"), [Self]() { Self->AutoRange(); }) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ MakeInfoLabel(LOCTEXT("PopupHint", "The range defaults to the 2nd to 98th percentile of the pressures of every solution together, so the colours mean the same from one solution to the next and a spike at a junction does not swamp the scale. Click outside to close.")) ]
					]
				];
			FSlateApplication::Get().PushMenu(ColorBar.ToSharedRef(), FWidgetPath(), Content, Event.GetScreenSpacePosition(), FPopupTransitionEffect(FPopupTransitionEffect::ContextMenu));
		}

		// ---- tab 3: the span loading ----

		void UpdateLoads()
		{
			if (!LoadPlot.IsValid()) return;
			const FCase* C = CurrentCase();
			const TArray<FLoadView> Views = LoadViews();
			if (C == nullptr || Views.Num() == 0)
			{
				LoadPlot->SetSeries({});
				LoadPlot->SetMessage(C == nullptr ? LOCTEXT("NoSolution", "Solve first (Set-up tab).") : LOCTEXT("NoWing", "No body to sum."));
				LoadPlot->SetTitle(LOCTEXT("LoadTitle", "Span loading"));
				return;
			}
			CurrentLoadView = FMath::Clamp(CurrentLoadView, 0, Views.Num() - 1);
			const FLoadView& LoadView = Views[CurrentLoadView];
			const bool bWing = LoadView.Share >= 0;
			const double Area = ReferenceArea > 0.0 ? ReferenceArea : 1.0;

			struct FPoint { double S, SMin, SMax, LoadPerSpan, Chord; };
			TArray<FPoint> Points;
			aerotelic::Vec3 LoadDir;
			bool bSideLoad = false;
			try
			{
				if (bWing)
				{
					const FShare& Sh = C->Shares[LoadView.Share];
					if (Sh.Body < 0 || Sh.Body >= static_cast<int32>(C->Sol.bodies.size())) throw std::runtime_error("the wing has no solved body");
					const tripanel::BodyResult& BR = C->Sol.bodies[Sh.Body];
					const aerotelic::Vec3 Axis{ Sh.SpanAxis.X, Sh.SpanAxis.Y, Sh.SpanAxis.Z };
					LoadDir = aerotelic::Normalized(aerotelic::Cross(C->Stream, Axis));
					bSideLoad = FMath::Abs(Sh.SpanAxis.Z) > 0.9;
					const double SRoot = FVector::DotProduct(Sh.RootPoint, Sh.SpanAxis), STip = FVector::DotProduct(Sh.TipPoint, Sh.SpanAxis);
					const int32 NCuts = Sh.CutChords.Num();
					auto Cut = [&](bool bOtherHalf, bool bMirror)
					{
						// The cuts at the centres of NCuts bands from the root to the
						// tip: never on the root ring or the tip cap, where a cut
						// catches the cap's triangles or nothing at all.
						for (int32 k = 0; k < NCuts; ++k)
						{
							const double S = SRoot + (STip - SRoot) * (k + 0.5) / NCuts;
							const double SAt = bOtherHalf ? -S : S;
							const tripanel::SectionCut Sc = tripanel::CutSection(BR, Axis, SAt, bMirror, &Sh.Mask);
							if (!Sc.found) continue;
							Points.Add({ SAt, SAt, SAt, aerotelic::Dot(Sc.force_per_span, LoadDir), Sh.CutChords[k] });
						}
					};
					Cut(false, false);
					if (FMath::Abs(Sh.SpanAxis.Y) > 0.9)
					{
						// The port half. With the plane on the solved body is the
						// starboard half and the image is the other: the same panels
						// reflected, cut at the reflected station. Without it a wing
						// built tip to tip is one body, cut at the reflected station
						// as it stands.
						if (Sh.bImaged) Cut(true, true);
						else if (Sh.bTipToTip) Cut(true, false);
					}
				}
				else
				{
					LoadDir = C->LiftDir;
					double Lo = 1e300, Hi = -1e300;
					for (const tripanel::BodyResult& BR : C->Sol.bodies)
					{
						for (const aerotelic::Vec3& P : BR.mesh.vertices)
						{
							Lo = FMath::Min(Lo, P.y);
							Hi = FMath::Max(Hi, P.y);
							if (C->bImagedSolve) { Lo = FMath::Min(Lo, -P.y); Hi = FMath::Max(Hi, -P.y); }
						}
					}
					if (!(Hi > Lo)) throw std::runtime_error("no body panels to sum");
					const int32 NBins = 40;
					std::vector<double> Edges;
					for (int32 b = 0; b <= NBins; ++b) Edges.push_back(Lo + (Hi - Lo) * b / NBins);
					const std::vector<aerotelic::Vec3> Bins = tripanel::BinnedLoads(C->Sol, aerotelic::Vec3{ 0.0, 1.0, 0.0 }, Edges, C->bImagedSolve);
					for (int32 b = 0; b < NBins; ++b) Points.Add({ 0.5 * (Edges[b] + Edges[b + 1]), Edges[b], Edges[b + 1], aerotelic::Dot(Bins[b], LoadDir) / (Edges[b + 1] - Edges[b]), 0.0 });
				}
			}
			catch (const std::exception& E)
			{
				LoadPlot->SetSeries({});
				LoadPlot->SetMessage(FText::FromString(UTF8_TO_TCHAR(E.what())));
				return;
			}
			if (Points.Num() == 0)
			{
				LoadPlot->SetSeries({});
				LoadPlot->SetMessage(LOCTEXT("NoStrips", "No section of this body crossed by the cuts."));
				return;
			}
			Points.Sort([](const FPoint& A, const FPoint& B) { return A.S < B.S; });
			double SMin = 1e300, SMax = -1e300, TotalForce = 0.0;
			for (int32 i = 0; i < Points.Num(); ++i)
			{
				const FPoint& P = Points[i];
				SMin = FMath::Min(SMin, P.SMin);
				SMax = FMath::Max(SMax, P.SMax);
				if (bWing) { if (i > 0) TotalForce += 0.5 * (P.LoadPerSpan + Points[i - 1].LoadPerSpan) * (P.S - Points[i - 1].S); }
				else TotalForce += P.LoadPerSpan * (P.SMax - P.SMin);
			}
			const double TotalCoefficient = TotalForce / Area;
			const double S0 = 0.5 * (SMin + SMax), Half = FMath::Max(0.5 * (SMax - SMin), 1e-9);
			const FString LengthUnit = UTF8_TO_TCHAR(Units.LengthName());

			// A fin (its span axis vertical) carries a SIDE force: shown root to
			// tip with no elliptic reference, and named so -- its integral is
			// about zero in symmetric flow and reads nothing like a lift.
			TArray<FAerotelicPlotSeries> Series;
			FAerotelicPlotSeries Computed;
			Computed.Label = bSideLoad ? TEXT("y / q  side force per unit span (panel method, pressures)") : bWing ? TEXT("l / q  (panel method, pressures)") : TEXT("l / q  (all bodies, pressures)");
			Computed.Color = FLinearColor(FColor::FromHex(TEXT("#d56b3a")));
			for (const FPoint& P : Points) Computed.Points.Add(FVector2D((P.S - S0) / Half, P.LoadPerSpan));
			Series.Add(Computed);
			if (!bSideLoad)
			{
				FAerotelicPlotSeries Elliptic;
				Elliptic.Label = TEXT("l / q  elliptic, same integral");
				Elliptic.Color = FLinearColor(FColor::FromHex(TEXT("#3a7bd5")));
				Elliptic.bMarkers = false;
				Elliptic.bDashed = true;
				for (int32 i = 0; i <= 100; ++i)
				{
					const double S = S0 - Half + (2.0 * Half) * i / 100.0;
					const double X = (S - S0) / Half;
					Elliptic.Points.Add(FVector2D(X, 4.0 * TotalForce / (3.14159265358979323846 * 2.0 * Half) * std::sqrt(FMath::Max(0.0, 1.0 - X * X))));
				}
				Series.Add(Elliptic);
			}
			if (bWing)
			{
				FAerotelicPlotSeries Cl;
				Cl.Label = bSideLoad ? TEXT("cy  (section side-force coefficient, right axis)") : TEXT("cl  (section lift coefficient, right axis)");
				Cl.Color = FLinearColor(FColor::FromHex(TEXT("#c8d53a")));
				Cl.bRightAxis = true;
				for (const FPoint& P : Points)
				{
					if (P.Chord <= 1e-12) continue;
					Cl.Points.Add(FVector2D((P.S - S0) / Half, P.LoadPerSpan / P.Chord));
				}
				Series.Add(Cl);
			}
			LoadPlot->SetSeries(Series);
			const FText LeftLabel = FText::FromString(bSideLoad ? FString::Printf(TEXT("y / q  side force per unit span over the dynamic pressure, %s"), *LengthUnit) : FString::Printf(TEXT("l / q  lift per unit span over the dynamic pressure, %s"), *LengthUnit));
			if (bSideLoad)
			{
				LoadPlot->SetTitle(FText::FromString(FString::Printf(TEXT("%s: side loading of a fin, root to tip, α = %g°  (Y / q = %.4f %s², C_Y = %.4f; zero in symmetric flow; cuts every %d%% of the height; no elliptic reference for a fin)"), *LoadView.Name, C->AlphaDeg, TotalForce, *LengthUnit, TotalCoefficient, 100 / kLoadCuts)));
				LoadPlot->SetAxisLabels(LOCTEXT("FinAxis", "height fraction (root -1, tip +1)"), LeftLabel, LOCTEXT("CyAxis", "cy  section side-force coefficient"));
			}
			else if (bWing)
			{
				LoadPlot->SetTitle(FText::FromString(FString::Printf(TEXT("%s: span loading, α = %g°  (L / q = %.4f %s², C_L = %.4f; cuts every %d%% of the semi-span)"), *LoadView.Name, C->AlphaDeg, TotalForce, *LengthUnit, TotalCoefficient, 100 / kLoadCuts)));
				LoadPlot->SetAxisLabels(LOCTEXT("SpanAxis", "span fraction 2s / b"), LeftLabel, LOCTEXT("ClAxis", "cl  section lift coefficient"));
			}
			else
			{
				LoadPlot->SetTitle(FText::FromString(FString::Printf(TEXT("All bodies: lift per unit span along y, α = %g°  (L / q = %.4f %s², C_L = %.4f, %d bins)"), C->AlphaDeg, TotalForce, *LengthUnit, TotalCoefficient, Points.Num())));
				LoadPlot->SetAxisLabels(LOCTEXT("SpanAxisY", "lateral fraction 2y / b"), LeftLabel);
			}
		}

		// ---- tab 4: forces and moments against the swept angle ----

		void UpdatePolars()
		{
			if (Polars.Num() < 6 || !PolarSwitch.IsValid() || !PolarTable.IsValid()) return;
			struct FCoef { const TCHAR* Name; double FCase::*Member; double FCoefs::*Body; FLinearColor Color; };
			const FCoef Coefs[6] = {
				{ TEXT("C_L  lift"), &FCase::CL, &FCoefs::CL, FLinearColor(FColor::FromHex(TEXT("#d56b3a"))) },
				{ TEXT("C_D  induced drag"), &FCase::CD, &FCoefs::CD, FLinearColor(FColor::FromHex(TEXT("#c8d53a"))) },
				{ TEXT("C_Y  side force"), &FCase::CY, &FCoefs::CY, FLinearColor(FColor::FromHex(TEXT("#30f05a"))) },
				{ TEXT("C_l  roll (about the stream)"), &FCase::Cl, &FCoefs::Cl, FLinearColor(FColor::FromHex(TEXT("#3a7bd5"))) },
				{ TEXT("C_m  pitch"), &FCase::Cm, &FCoefs::Cm, FLinearColor(FColor::FromHex(TEXT("#b07bd5"))) },
				{ TEXT("C_n  yaw (about the lift)"), &FCase::Cn, &FCoefs::Cn, FLinearColor(FColor::FromHex(TEXT("#5ad5c8"))) },
			};
			static const TCHAR* BodyHex[] = { TEXT("#d56b3a"), TEXT("#3a7bd5"), TEXT("#c8d53a"), TEXT("#b07bd5"), TEXT("#5ad5c8"), TEXT("#30f05a"), TEXT("#d53a7b"), TEXT("#7bd53a") };
			auto BodyColor = [](int32 Index) { return FLinearColor(FColor::FromHex(BodyHex[Index % UE_ARRAY_COUNT(BodyHex)])); };
			if (Cases.Num() == 0)
			{
				for (int32 i = 0; i < 6; ++i)
				{
					Polars[i]->SetSeries({});
					Polars[i]->SetMessage(LOCTEXT("NoSolution", "Solve first (Set-up tab)."));
					Polars[i]->SetTitle(FText::FromString(Coefs[i].Name));
				}
				PolarSwitch->SetActiveWidgetIndex(0);
				return;
			}
			const FText XLabel = bSweepAlpha ? LOCTEXT("AlphaAxis", "angle of attack, deg") : LOCTEXT("BetaAxis", "angle of sideslip, deg");
			if (Cases.Num() == 1)
			{
				PolarTable->ClearChildren();
				const FCase& C = *Cases[0];
				const bool bMany = C.Shares.Num() > 1;
				PolarTable->AddSlot().AutoHeight().Padding(0, 0, 0, 8)
				[
					SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("One solution, α = %g°, β = %g°. Wind axes: lift normal to the stream in the plane of symmetry, drag along it, side force along the third axis; moments about the origin on the reference length %.4g %s, roll about the stream, pitch about the side axis, yaw about the lift axis.%s"), C.AlphaDeg, C.BetaDeg, ReferenceLength, UTF8_TO_TCHAR(Units.LengthName()),
						bMany ? TEXT(" One column per body (its own triangles of a fused surface; a mirrored body with both halves) and the total of them all.") : TEXT("")))).AutoWrapText(true)
				];
				const float LabelW = 220.0f, CellW = 110.0f;
				auto Cell = [&](const FString& Text, bool bBold)
				{
					return SNew(SBox).WidthOverride(CellW).HAlign(HAlign_Right).Padding(FMargin(0, 0, 10, 0))
						[ SNew(STextBlock).Text(FText::FromString(Text)).Font(FCoreStyle::GetDefaultFontStyle(bBold ? "Bold" : "Regular", 9)) ];
				};
				if (bMany)
				{
					TSharedRef<SHorizontalBox> Head = SNew(SHorizontalBox);
					Head->AddSlot().AutoWidth() [ SNew(SBox).WidthOverride(LabelW) [ SNew(STextBlock).Text(LOCTEXT("CoefHead", "Coefficient")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ] ];
					for (const FShare& Sh : C.Shares) Head->AddSlot().AutoWidth() [ Cell(Sh.Name, false) ];
					Head->AddSlot().AutoWidth() [ Cell(TEXT("Total"), true) ];
					PolarTable->AddSlot().AutoHeight().Padding(0, 2, 0, 4) [ Head ];
				}
				for (int32 i = 0; i < 6; ++i)
				{
					TSharedRef<SHorizontalBox> Row = SNew(SHorizontalBox);
					Row->AddSlot().AutoWidth().VAlign(VAlign_Center) [ SNew(SBox).WidthOverride(LabelW) [ SNew(STextBlock).Text(FText::FromString(Coefs[i].Name)) ] ];
					if (bMany)
					{
						for (const FCoefs& K : C.ByBody) Row->AddSlot().AutoWidth() [ Cell(FString::Printf(TEXT("%.5f"), K.*(Coefs[i].Body)), false) ];
					}
					Row->AddSlot().AutoWidth() [ Cell(FString::Printf(TEXT("%.5f"), C.*(Coefs[i].Member)), bMany) ];
					PolarTable->AddSlot().AutoHeight().Padding(0, 2) [ Row ];
				}
				if (C.Sol.lift_from_wake)
				{
					PolarTable->AddSlot().AutoHeight().Padding(0, 8, 0, 0)
					[ MakeInfoLabel(FText::FromString(FString::Printf(TEXT("C_L from the wake doublet strength (the circulation of every shed edge, Kutta-Joukowski): %.5f. The pressure-integrated total above depends on the surface gradient of the doublet strength fitted over neighbouring panels; the two agree to a few per cent on a sound mesh."), *C.Sol.lift_from_wake))) ];
				}
				PolarSwitch->SetActiveWidgetIndex(1);
				return;
			}
			const bool bMany = Cases[0]->Shares.Num() > 1;
			for (int32 i = 0; i < 6; ++i)
			{
				TArray<FAerotelicPlotSeries> Series;
				if (bMany)
				{
					const int32 NBodies = Cases[0]->Shares.Num();
					for (int32 b = 0; b < NBodies; ++b)
					{
						FAerotelicPlotSeries S;
						S.Label = Cases[0]->Shares[b].Name;
						S.Color = BodyColor(b);
						S.LineWidth = 1.0f;
						S.bMarkers = false;
						S.bDashed = true;
						for (const TSharedPtr<FCase>& C : Cases)
							if (C->ByBody.IsValidIndex(b)) S.Points.Add(FVector2D(bSweepAlpha ? C->AlphaDeg : C->BetaDeg, C->ByBody[b].*(Coefs[i].Body)));
						S.Points.Sort([](const FVector2D& A, const FVector2D& B) { return A.X < B.X; });
						Series.Add(S);
					}
				}
				FAerotelicPlotSeries S;
				S.Label = bMany ? TEXT("Total") : Coefs[i].Name;
				S.Color = bMany ? FLinearColor::White : Coefs[i].Color;
				S.LineWidth = bMany ? 2.0f : 1.5f;
				for (const TSharedPtr<FCase>& C : Cases)
					S.Points.Add(FVector2D(bSweepAlpha ? C->AlphaDeg : C->BetaDeg, (*C).*(Coefs[i].Member)));
				S.Points.Sort([](const FVector2D& A, const FVector2D& B) { return A.X < B.X; });
				Series.Add(S);
				Polars[i]->SetSeries(Series);
				Polars[i]->SetMessage(LOCTEXT("NoFinite", "No finite values: the solution failed."));
				Polars[i]->SetTitle(FText::FromString(Coefs[i].Name));
				Polars[i]->SetAxisLabels(XLabel, FText::GetEmpty());
			}
			PolarSwitch->SetActiveWidgetIndex(0);
		}

		void Close()
		{
			if (Job.IsValid())
			{
				Job->bCancel = true;
				Job.Reset();
			}
			if (MeshActor.IsValid()) MeshActor->Destroy();
			MeshActor.Reset();
			AerotelicAeroTrim::HideAeroMeshInViewport();
			MeshCase.Reset();
			Cases.Empty();
			Mesh = FCpMesh();
			Picker.Reset(); SymmetryCheck.Reset(); FuseCheck.Reset(); ShowMeshCheck.Reset(); AlphaBox.Reset(); BetaBox.Reset(); PathBox.Reset();
			LblWarn.Reset(); LblMesh.Reset(); LblSummary.Reset(); LblInput.Reset(); SolveButton.Reset();
			View.Reset(); ColorBar.Reset(); LoadPlot.Reset(); Polars.Empty(); PolarSwitch.Reset(); PolarTable.Reset();
		}
	};

	// "<  label  >": steps an index; the label follows a lambda.
	TSharedRef<SWidget> MakeStepper(TFunction<FText()> Label, TFunction<void(int32)> Step, TFunction<bool()> Enabled)
	{
		auto Arrow = [Step, Enabled](const TCHAR* Glyph, int32 Delta)
		{
			return SNew(SButton).Text(FText::FromString(Glyph)).ContentPadding(FMargin(8, 2))
				.IsEnabled_Lambda([Enabled]() { return Enabled(); })
				.OnClicked_Lambda([Step, Delta]() { Step(Delta); return FReply::Handled(); });
		};
		return SNew(SBorder).Padding(FMargin(6, 4)).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.0f, 0.0f, 0.0f, 0.45f))
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center) [ Arrow(TEXT("◀"), -1) ]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0)
				[ SNew(STextBlock).Text_Lambda([Label]() { return Label(); }).ColorAndOpacity(FLinearColor::White) ]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center) [ Arrow(TEXT("▶"), +1) ]
			];
	}

	TSharedRef<SWidget> MakeCaseStepper(TSharedRef<FState> S)
	{
		return MakeStepper(
			[S]() { return S->CaseLabel(); },
			[S](int32 Delta) { S->SetCurrent(FMath::Clamp(S->Current + Delta, 0, S->Cases.Num() - 1)); },
			[S]() { return S->Cases.Num() > 1; });
	}

	const TCHAR* kHelp = TEXT(
		"PANEL METHOD ON THE BODY TRIANGULATION\n\n"
		"The incompressible potential flow about the chosen bodies, solved on the very triangles the viewport draws and the structural analysis is built on: constant-strength sources and doublets on every triangle in the interior-potential (Dirichlet) formulation of Morino, "
		"the sources prescribed by the free stream and the doublet strengths -- the outer surface potential -- solved for; a wake shed along the stream from every trailing edge the bodies tag (a wing's, a cowl rim's), its strength the circulation of that section (the Kutta condition); the image of everything about y = 0 when the symmetry plane is on. "
		"Cp = 1 - (V/V∞)², the surface velocity being the free stream's tangential part plus the surface gradient of the doublet strength, and the coefficients are C_F = -∮ Cp n dA / S_ref. A potential flow is similar at every speed and density, so nothing dimensional is asked for.\n\n"
		"THE MESH IS THE ANALYSIS\n"
		"What you see is what is solved. Each body comes in on its own grid -- the Mesh tab of its design panel sets the stations and the points round a section; refinement off gives a clean structured grid of the size you choose. A wing with flaps is paneled as its AERO COPY: the sealed one-piece wing on the same stations, every deflected flap entering as a kink of the section about its hinge (the vortex-lattice view of a flap: a potential solver cannot see a 4 mm slot through panels larger than the slot). "
		"Without flaps the aero copy is the wing's own body. A boolean result is paneled as its operands' aero copies. Bodies whose bounds overlap are FUSED into one closed surface by the mesh boolean before anything is solved, so nothing of one body lies inside another -- a wing root inside a fuselage, a tailplane on a fin -- which a panel method cannot tolerate (panels inside another body are not a wetted surface). "
		"The mesh report on the Set-up tab lists every body's triangle count, edge lengths, worst aspect ratio and closedness, what was fused, and warns of anything left overlapping; tick the box to see the mesh with its edges in the main 3D view. A dense solve is comfortable to a few thousand panels and runs by GMRES beyond; coarsen the grids on the Mesh tabs for a quick look, refine them for the answer.\n\n"
		"SET-UP\n"
		"Choose the bodies, the angles and the references, then Solve. The angle of attack and the angle of sideslip each take one value or a comma separated list, but only one of them may be a list: the solutions are a sweep of that angle with the other fixed. Sideslip needs the symmetry plane off. The solves run in the background, one per angle; the dialog stays usable.\n\n"
		"SURFACE PRESSURE\n"
		"The panels coloured by the pressure coefficient, flat per panel with their edges drawn (so the mesh is in plain sight), lit from the eye. Left-drag orbits, right- or middle-drag pans, the wheel zooms, Home frames the bodies. "
		"Click the colour bar to set the minimum, the maximum, the colour map and the edges; the range defaults to the 2nd to 98th percentile of the pressures of every solution together. The arrows step between solutions.\n\n"
		"SPAN LOADING\n"
		"For a wing, on the left axis: the lift per unit span over the dynamic pressure, l / q (a length in the project's units), against the span fraction. The wing's own triangles (its share of a fused surface) are cut by a plane normal to its span every 5% of the semi-span; the pressure on the section's outline is integrated along it, resolved normal to the stream and to the span axis -- the force per unit span itself, no strip and no width. "
		"The elliptic distribution of the same integral is dashed beside it. On the right axis: the section lift coefficient cl on the planform's chord at that station. A wing built tip to tip shows both halves; with the symmetry plane on, the image half is drawn as well. "
		"The last view, All bodies, is l / q of the whole configuration along y: every panel's load spread over uniform bins.\n\n"
		"FORCES AND MOMENTS\n"
		"The wind-axis coefficients against the swept angle: lift, induced drag (a potential flow has no friction) and side force; roll about the stream, pitch about the side axis and yaw about the lift axis, about the origin on the reference length. "
		"With more than one body each chart shows a dashed curve per body -- its own triangles of a fused surface -- and the total in white. With a single solution the values are tabulated instead, with the lift from the wake circulation beside the pressure-integrated total: the two agree to a few per cent on a sound mesh, and drift apart on a poor one.\n\n"
		"TRIM AND STRUCTURAL LOADS\n"
		"Trimming the configuration to a flight condition and handing its pressures to the structural analysis is done under Model > Load Cases: an aero load case there is a point of the V-n diagram, solved with this same method on this same mesh and stored for Analysis > Structural, where every panel's pressure lands on the structural face it is. This dialog is the analysis view: sweeps, pressures, loadings and polars.\n\n"
		"THE HIGHER-ORDER CODE\n"
		"Until 2026-11-15 this dialog ran Johnson's higher-order method (NASA CR-3079) on curved quadrilateral networks built from the sections. It stays in the core as the validation reference (tri_panel_check runs the two side by side on a wing); it could not take a boolean result, paneled overlapping bodies separately with buried panels, and ignored a symmetric wing's port half.");
}

void ShowAerotelicHighOrderPanelDialog(TFunction<void(const FText&)> Status)
{
	TArray<FAerotelicBodyChoicePtr> Choices = PanelChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoBody", "No solid body in the scene. The panel method runs on wings, fuselages, nacelles, primitives and boolean results over them."),
			LOCTEXT("Title", "Panel Method"));
		return;
	}

	const FVector2D Size(820, 940);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Panel Method"))
		.ClientSize(Size)
		.SupportsMaximize(true)
		.SupportsMinimize(true);

	TSharedRef<FState> S = MakeShared<FState>();
	S->Status = Status;
	S->Window = Window;
	const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	S->Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	const FText AreaSuffix = FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(S->Units.AreaName().c_str()));
	const FText LengthSuffix = FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(S->Units.LengthName()));
	for (const aerotelic::Colormap& M : aerotelic::Colormaps()) S->MapNames->Add(MakeShared<FString>(UTF8_TO_TCHAR(M.name.c_str())));

	auto Spin = [S](double FState::*Member, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([S, Member]() { return (*S).*Member; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([S, Member](double V, ETextCommit::Type) { (*S).*Member = V; });
	};
	auto Browse = [S, Window]()
	{
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr || !S->PathBox.IsValid()) return;
		TArray<FString> Files;
		const FString Current = S->PathBox->GetText().ToString();
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Window);
		if (!Desktop->SaveFileDialog(Handle, TEXT("Panel Solution"), FPaths::GetPath(Current), FPaths::GetCleanFilename(Current), TEXT("VTK Files (*.vtk)|*.vtk"), EFileDialogFlags::None, Files) || Files.Num() == 0) return;
		FString Path = FPaths::ConvertRelativePathToFull(Files[0]);
		if (FPaths::GetExtension(Path).IsEmpty()) Path += TEXT(".vtk");
		S->PathBox->SetText(FText::FromString(Path));
	};

	// ---- tab 1: set-up ----
	TSharedRef<SWidget> SetupTab =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("BodiesGroup", "Bodies"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SAssignNew(S->Picker, SAerotelicBodyPicker).Choices(Choices).InitiallyChosen(Choices).ListHeight(130.0f)
					.OnChanged(FSimpleDelegate::CreateLambda([S]() { S->Refresh(); }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					SAssignNew(S->SymmetryCheck, SCheckBox)
					.OnCheckStateChanged_Lambda([S](ECheckBoxState) { S->bSymmetryTouched = true; S->RefreshMesh(); })
					[ SNew(STextBlock).Text(LOCTEXT("Symmetry", "Symmetry plane at y = 0  (every panel solved with its image; a body on the centreline is cut there)")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					SAssignNew(S->FuseCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
					.OnCheckStateChanged_Lambda([S](ECheckBoxState) { S->RefreshMesh(); })
					[ SNew(STextBlock).Text(LOCTEXT("Fuse", "Fuse bodies that overlap into one surface before solving  (off: panel them separately, buried panels and all)")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("BodiesHint",
						"Every solid body of the scene can be paneled: a wing as its sealed aero copy with a wake from its trailing edge, a fuselage, nacelle or primitive as its own closed triangles (a tagged trailing edge sheds a wake too), a boolean result as its operands' aero copies. "
						"A mirrored entity appears once: with the symmetry plane on, its image stands in for the other half; off, the reflected copy is paneled as well. The plane defaults on when something is mirrored, a wing is symmetric, or a wing root sits on y = 0."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("MeshGroup", "The mesh that will be solved"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ Stat(S->LblMesh) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					SAssignNew(S->LblWarn, STextBlock).AutoWrapText(true).Visibility(EVisibility::Collapsed)
					.ColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.75f, 0.3f)))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
					[
						SAssignNew(S->ShowMeshCheck, SCheckBox)
						.OnCheckStateChanged_Lambda([S](ECheckBoxState State)
						{
							if (State == ECheckBoxState::Checked && S->MeshCase.IsValid()) AerotelicAeroTrim::ShowAeroMeshInViewport(*S->MeshCase);
							else AerotelicAeroTrim::HideAeroMeshInViewport();
						})
						[ SNew(STextBlock).Text(LOCTEXT("ShowMesh", "Show this mesh in the 3D view, edges drawn (the aero copy over the scene's bodies)")) ]
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(12, 0, 0, 0).VAlign(VAlign_Center)
					[ Button(LOCTEXT("Recheck", "Rebuild the mesh report"), [S]() { S->RefreshMesh(); }) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("MeshHint",
						"The triangles are each body's own -- the Mesh tab of its design panel sets the grid (stations, points round a section, refinement). A wing with flaps is its sealed aero copy on the same stations, the trim's flap kinked about its hinge; without flaps it is the body you see. "
						"Overlapping bodies are fused into one closed surface first (the mesh boolean), so no panel lies inside another body. Aspect ratio is the longest edge over the shortest altitude: 1.15 for an equilateral triangle, hundreds for a needle. A dense solve is comfortable to a few thousand panels; GMRES takes over beyond."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("FreeStream", "Operating conditions and references"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Aoa", "Angle of attack, deg:"), SAssignNew(S->AlphaBox, SEditableTextBox).Text(FText::FromString(TEXT("4"))).ToolTipText(LOCTEXT("AoaTip", "One angle, or a comma separated list: 0, 2, 4, 6"))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Aos", "Angle of sideslip, deg:"), SAssignNew(S->BetaBox, SEditableTextBox).Text(FText::FromString(TEXT("0"))).ToolTipText(LOCTEXT("AosTip", "Needs the symmetry plane off; a list here with a single angle of attack sweeps the sideslip"))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					SAssignNew(S->LblInput, STextBlock).AutoWrapText(true).Visibility(EVisibility::Collapsed)
					.ColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.45f, 0.35f)))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Area", "Reference area S_ref:"), Spin(&FState::ReferenceArea, 1e-12, 1e12, 1.0, AreaSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Length", "Reference length (moments):"), Spin(&FState::ReferenceLength, 1e-12, 1e12, 0.1, LengthSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Wake", "Wake length, reference lengths:"), Spin(&FState::WakeChords, 1, 1000, 5, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("FlowHint",
						"The flow runs along +x, tilted towards +z by the angle of attack. A comma separated list of angles of attack (or of sideslip, not both) solves one case per angle: the other tabs step through them. "
						"A potential flow is similar at every speed and density: Cp = 1 - (V/V∞)² and the coefficients are C_F = -∮ Cp n dA / S_ref, so neither is asked for. "
						"The reference area defaults to the planform area of the chosen wings (both halves) and the reference length to the first root chord; with no wing, to the first body's frontal ellipse and length. Both are in the project's units. "
						"To trim the configuration to a flight condition and load the structure with the result, define an aero load case under Model > Load Cases."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Output", "Solution file (triangles with Cp, doublet and source strength; one per angle; leave empty to skip)"),
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
				[ SAssignNew(S->PathBox, SEditableTextBox).Text(FText::FromString(DefaultSolutionPath())) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
				[ Button(LOCTEXT("Browse", "Browse..."), Browse) ])
		]
		+ SScrollBox::Slot()
		[
			MakeGroup(LOCTEXT("Summary", "Solution"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ Stat(S->LblSummary) ])
		];

	// ---- tab 2: surface pressure ----
	TSharedRef<SWidget> SurfaceTab =
		SAssignNew(S->View, SAerotelicSceneView)
		.Overlay()
		[
			SNew(SOverlay)
			+ SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[
				SAssignNew(S->ColorBar, SAerotelicColorBar)
				.Colormap_Lambda([S]() { return S->Map; })
				.MinValue_Lambda([S]() { return S->CpLo; })
				.MaxValue_Lambda([S]() { return S->CpHi; })
				.Title(LOCTEXT("CpTitle", "Cp"))
				.OnClicked(FOnAerotelicColorBarClicked::CreateLambda([S](const FPointerEvent& E) { S->OpenColorPopup(E); }))
			]
			+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Bottom).Padding(8)
			[ MakeCaseStepper(S) ]
			+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(8)
			[
				SNew(STextBlock).ColorAndOpacity(FLinearColor(0.85f, 0.85f, 0.85f, 0.9f))
				.Text_Lambda([S]() { return S->Cases.Num() ? LOCTEXT("ViewHint", "Left-drag orbits, right-drag pans, wheel zooms, Home frames. Click the colour bar for the range, map and edges.") : LOCTEXT("ViewEmpty", "Solve first (Set-up tab); the panels appear here coloured by Cp, edges drawn."); })
			]
		];

	// ---- tab 3: span loading ----
	TSharedRef<SWidget> LoadsTab =
		SNew(SOverlay)
		+ SOverlay::Slot()
		[ SAssignNew(S->LoadPlot, SAerotelicLinePlot).Title(LOCTEXT("LoadTitle", "Span loading")) ]
		+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Bottom).Padding(8, 0, 0, 44)
		[ MakeCaseStepper(S) ]
		+ SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Bottom).Padding(0, 0, 8, 44)
		[
			MakeStepper(
				[S]()
				{
					const TArray<FState::FLoadView> V = S->LoadViews();
					if (V.Num() == 0) return LOCTEXT("NoWingShort", "no body");
					const int32 i = FMath::Clamp(S->CurrentLoadView, 0, V.Num() - 1);
					return FText::FromString(FString::Printf(TEXT("%s  (%d of %d)"), *V[i].Name, i + 1, V.Num()));
				},
				[S](int32 Delta)
				{
					const int32 N = S->LoadViews().Num();
					if (N == 0) return;
					S->CurrentLoadView = (S->CurrentLoadView + Delta + N) % N;
					S->UpdateLoads();
				},
				[S]() { return S->LoadViews().Num() > 1; })
		];

	// ---- tab 4: forces and moments ----
	TSharedRef<SUniformGridPanel> Grid = SNew(SUniformGridPanel).SlotPadding(FMargin(3));
	for (int32 i = 0; i < 6; ++i)
	{
		TSharedPtr<SAerotelicLinePlot> P;
		Grid->AddSlot(i % 3, i / 3) [ SAssignNew(P, SAerotelicLinePlot).Compact(true).DesiredSize(FVector2D(240, 200)) ];
		S->Polars.Add(P);
	}
	TSharedRef<SWidget> PolarsTab =
		SAssignNew(S->PolarSwitch, SWidgetSwitcher)
		+ SWidgetSwitcher::Slot() [ Grid ]
		+ SWidgetSwitcher::Slot()
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot() [ SAssignNew(S->PolarTable, SVerticalBox) ]
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SetupTab", "Set-up"), SetupTab });
	Tabs.Add({ LOCTEXT("SurfaceTab", "Surface Pressure"), SurfaceTab });
	Tabs.Add({ LOCTEXT("LoadsTab", "Span Loading"), LoadsTab });
	Tabs.Add({ LOCTEXT("PolarsTab", "Forces and Moments"), PolarsTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(FText::FromString(kHelp)) });

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SAssignNew(S->Tabs, SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[
					SAssignNew(S->SolveButton, SButton).Text(LOCTEXT("SolveBtn", "Solve")).HAlign(HAlign_Center)
					.OnClicked_Lambda([S]() { S->Solve(); return FReply::Handled(); })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([S](const TSharedRef<SWindow>&) { S->Close(); }));

	S->Refresh();
	S->UpdateLoads();
	S->UpdatePolars();
	Status(LOCTEXT("Pick", "Choose the bodies and the angles, check the mesh, then Solve."));
	ShowAerotelicDialogWindow(Window, Size);

	if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicPanelSmoke")))
	{
		S->bSmoke = true;
		S->AlphaBox->SetText(FText::FromString(TEXT("0, 4")));
		S->PathBox->SetText(FText::GetEmpty());
		S->Solve();
	}
}

#undef LOCTEXT_NAMESPACE
