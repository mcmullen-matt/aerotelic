#include "SectionAeroDialog.h"
#include "AirfoilPickerDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Containers/Ticker.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

#include "Airfoils/SeligDatabase.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/LinePlot.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/MessageBox.h"
#include "Widgets/AerotelicDialogWindow.h"

#include "aerotelic/airfoil.hpp"
#include "aerotelic/section_aero.hpp"

#include <cmath>

#define LOCTEXT_NAMESPACE "SectionAeroDialog"

using namespace aerotelic;

namespace
{
	TWeakPtr<SWindow> GSectionAeroWindow;

	const FLinearColor kMethodColors[] = {
		FLinearColor(FColor::FromHex(TEXT("#3a7bd5"))),
		FLinearColor(FColor::FromHex(TEXT("#d56b3a"))),
		FLinearColor(FColor::FromHex(TEXT("#30f05a"))),
		FLinearColor(FColor::FromHex(TEXT("#c45ad5"))),
	};
	const FLinearColor kTransition(FColor::FromHex(TEXT("#f0c030")));
	const FLinearColor kSeparation(FColor::FromHex(TEXT("#f03030")));
	const FLinearColor kRangeEnd(FColor::FromHex(TEXT("#8a8a8a")));
	const FLinearColor kSource(0.55f, 0.55f, 0.55f);
	const FLinearColor kArrow(FColor::FromHex(TEXT("#f0c030")));

	const int32 kPointChoices[] = { 256, 512, 1024 };
	const SectionAeroScalar kScalars[] = { SectionAeroScalar::Cl, SectionAeroScalar::CmC4, SectionAeroScalar::CdFriction, SectionAeroScalar::CpUpperAtX, SectionAeroScalar::CpLowerAtX };

	// Everything the dialog edits and shows. Modeless, so the widgets'
	// lambdas share it; the source airfoil is never changed, the working
	// one is the re-representation the editor binds to.
	struct FState
	{
		std::shared_ptr<Airfoil> Source;
		std::shared_ptr<Airfoil> Working;

		// Representation.
		int32 Degree = 3;
		int32 NPoles = 16;
		int32 KnotMode = 0; // 0 averaging, 1 uniform, 2 keep the current knots
		bool bLockTE = true;
		FString RefitNote;
		bool bHasFit = false;
		AirfoilFitReport Fit;

		// Conditions.
		double AlphaDeg = 2.0;
		double Reynolds = 1.0e6;
		int32 PointsIndex = 1;
		TArray<bool> MethodOn;

		// Sensitivity.
		bool bShowSensitivity = true;
		int32 ScalarIndex = 0;
		double ProbeX = 0.3;
		PoleSensitivity Sens;

		bool bShowStratford = false;
		TArray<SectionAeroResult> Results;
		FString ApplyName;

		// Deferred work (the fit report and the sensitivities are too slow
		// for every drag step; they run once the poles have been still).
		bool bFitDirty = false, bSensDirty = false, bPolarDirty = false;
		double LastEdit = 0.0;
		FTSTicker::FDelegateHandle Ticker;

		TSharedPtr<SCurveEditor> Editor;
		TSharedPtr<SAerotelicLinePlot> CpPlot, PolarClPlot, PolarCmPlot;
		TSharedPtr<SVerticalBox> Table;
		TSharedPtr<STextBlock> CueText, SensText;
		TFunction<void(const FText&)> Status;
	};

	int32 NPointsOf(const FState& S) { return kPointChoices[FMath::Clamp(S.PointsIndex, 0, 2)]; }

	FText Num(double V, int32 Decimals)
	{
		if (!FMath::IsFinite(V)) return DashText();
		return FormatF(V, Decimals);
	}

	// A combo over fixed labels (see the Transform dialog's note on the
	// options' lifetime: the array must outlive the modeless window).
	TSharedRef<SWidget> ChoiceCombo(const TArray<FText>& Labels, TFunction<int32()> GetIndex, TFunction<void(int32)> SetIndex)
	{
		TSharedRef<TArray<TSharedPtr<FString>>> Options = MakeShared<TArray<TSharedPtr<FString>>>();
		for (const FText& Label : Labels) Options->Add(MakeShared<FString>(Label.ToString()));
		auto At = [Options](int32 Index) { return (*Options)[FMath::Clamp(Index, 0, Options->Num() - 1)]; };
		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&Options.Get())
			.InitiallySelectedItem(At(GetIndex()))
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([Options, SetIndex](TSharedPtr<FString> S, ESelectInfo::Type)
			{
				if (!S.IsValid()) return;
				const int32 Index = Options->IndexOfByKey(S);
				if (Index != INDEX_NONE) SetIndex(Index);
			})
			[ SNew(STextBlock).Text_Lambda([At, GetIndex]() { return FText::FromString(*At(GetIndex())); }) ];
	}

	TSharedRef<SWidget> Cell(const FText& T, float Width, bool bBold = false, FLinearColor Color = FLinearColor::White)
	{
		return SNew(SBox).WidthOverride(Width).Padding(2, 1)
			[ SNew(STextBlock).Text(T).ColorAndOpacity(FSlateColor(Color)).Font(FCoreStyle::GetDefaultFontStyle(bBold ? "Bold" : "Regular", 8)) ];
	}

	// ---- the analysis ---------------------------------------------------------

	SectionAeroRequest RequestFor(const FState& S)
	{
		SectionAeroRequest Req = SectionRequestFor(*S.Working);
		Req.alpha_deg = S.AlphaDeg;
		Req.reynolds = S.Reynolds;
		Req.n_points = NPointsOf(S);
		return Req;
	}

	void RebuildTable(const TSharedRef<FState>& S);
	void UpdateCpPlot(const TSharedRef<FState>& S);
	double InterpolateQ(const SectionSurface& Sf, double s);

	void Solve(const TSharedRef<FState>& S)
	{
		S->Results.Empty();
		if (!S->Working) { UpdateCpPlot(S); RebuildTable(S); return; }
		const SectionAeroRequest Req = RequestFor(*S);
		const auto& Methods = SectionAeroMethods();
		for (int32 i = 0; i < static_cast<int32>(Methods.size()) && i < S->MethodOn.Num(); ++i)
		{
			if (!S->MethodOn[i]) continue;
			SectionAeroResult R;
			try
			{
				R = SolveSection(*Methods[i], Req);
			}
			catch (const std::exception& E)
			{
				R.method = Methods[i]->Name();
				R.ok = false;
				R.error = E.what();
			}
			S->Results.Add(MoveTemp(R));
		}
		UpdateCpPlot(S);
		RebuildTable(S);
		S->bSensDirty = S->bShowSensitivity;
		S->bPolarDirty = true;
		if (S->Status && S->Results.Num() > 0 && S->Results[0].ok)
		{
			const SectionAeroResult& R = S->Results[0];
			S->Status(FText::Format(LOCTEXT("StatusSolved", "Section Aerodynamics: {0} at {1} deg, Re {2}: Cl {3}, Cm c/4 {4}, Cd friction {5} ({6} ms)"),
				FText::FromString(StdToFString(S->Working->name)), Num(S->AlphaDeg, 1), FormatG(S->Reynolds, 3), Num(R.cl, 4), Num(R.cm_c4, 4), Num(R.cd_friction, 5), Num(R.solve_ms, 2)));
		}
	}

	void UpdateCpPlot(const TSharedRef<FState>& S)
	{
		if (!S->CpPlot.IsValid()) return;
		TArray<FAerotelicPlotSeries> Series;
		bool bAnyCue = false;
		FString Message;
		for (int32 i = 0; i < S->Results.Num(); ++i)
		{
			const SectionAeroResult& R = S->Results[i];
			if (!R.ok) { Message += StdToFString(R.method) + TEXT(": ") + StdToFString(R.error) + TEXT("\n"); continue; }
			const FLinearColor C = kMethodColors[i % UE_ARRAY_COUNT(kMethodColors)];
			auto Surface = [&](const SectionSurface& Sf, bool bUpper)
			{
				FAerotelicPlotSeries P;
				P.Label = StdToFString(R.method) + (bUpper ? TEXT(" upper") : TEXT(" lower"));
				P.Color = C;
				P.bMarkers = false;
				P.bDashed = !bUpper;
				P.LineWidth = 1.8f;
				for (size_t k = 0; k < Sf.x.size(); ++k) P.Points.Add(FVector2D(Sf.x[k], Sf.cp[k]));
				Series.Add(MoveTemp(P));
			};
			Surface(R.upper, true);
			Surface(R.lower, false);
			FAerotelicPlotSeries Tr, Sep, End;
			Tr.Color = kTransition; Tr.bLine = false; Tr.MarkerSize = 7.0f;
			Sep.Color = kSeparation; Sep.bLine = false; Sep.MarkerSize = 9.0f;
			End.Color = kRangeEnd; End.bLine = false; End.MarkerSize = 5.0f;
			for (const SectionSurface* Sf : { &R.upper, &R.lower })
			{
				if (Sf->transition.found) Tr.Points.Add(FVector2D(Sf->transition.x, 1.0 - std::pow(InterpolateQ(*Sf, Sf->transition.s), 2)));
				if (Sf->separation.found) Sep.Points.Add(FVector2D(Sf->separation.x, 1.0 - std::pow(InterpolateQ(*Sf, Sf->separation.s), 2)));
				if (Sf->stratford_range_end_x >= 0.0)
				{
					// the Cp at that x along this surface
					for (size_t k = 0; k < Sf->x.size(); ++k) if (Sf->x[k] == Sf->stratford_range_end_x) { End.Points.Add(FVector2D(Sf->x[k], Sf->cp[k])); break; }
				}
			}
			if (!bAnyCue) { Tr.Label = TEXT("transition cue"); Sep.Label = TEXT("separation (Stratford)"); End.Label = TEXT("Cpbar > 4/7: criterion ends"); bAnyCue = true; }
			if (Tr.Points.Num()) Series.Add(MoveTemp(Tr));
			if (Sep.Points.Num()) Series.Add(MoveTemp(Sep));
			if (End.Points.Num()) Series.Add(MoveTemp(End));
			if (S->bShowStratford)
			{
				for (const SectionSurface* Sf : { &R.upper, &R.lower })
				{
					FAerotelicPlotSeries St;
					St.Label = (Sf == &R.upper) ? TEXT("Stratford S (right axis)") : FString();
					St.Color = C * 0.6f + FLinearColor(0.3f, 0.3f, 0.3f, 0.0f);
					St.Color.A = 1.0f;
					St.bMarkers = false;
					St.bRightAxis = true;
					St.bDashed = (Sf == &R.lower);
					St.LineWidth = 1.0f;
					for (size_t k = 0; k < Sf->x.size(); ++k) if (Sf->stratford[k] > 0.0) St.Points.Add(FVector2D(Sf->x[k], Sf->stratford[k]));
					if (St.Points.Num()) Series.Add(MoveTemp(St));
				}
			}
		}
		S->CpPlot->SetMessage(FText::FromString(Message.IsEmpty() ? (S->Working ? TEXT("No method enabled") : TEXT("Pick an airfoil")) : Message));
		S->CpPlot->SetAxisLabels(LOCTEXT("CpX", "x/c"), LOCTEXT("CpY", "Cp (suction up)"), S->bShowStratford ? LOCTEXT("CpYR", "Stratford S") : FText::GetEmpty());
		S->CpPlot->SetSeries(MoveTemp(Series));
	}

	// q at arc length s along a surface (linear).
	double InterpolateQ(const SectionSurface& Sf, double s)
	{
		for (size_t k = 1; k < Sf.s.size(); ++k)
		{
			if (Sf.s[k] >= s)
			{
				const double t = (Sf.s[k] - Sf.s[k - 1]) > 0.0 ? (s - Sf.s[k - 1]) / (Sf.s[k] - Sf.s[k - 1]) : 0.0;
				return Sf.q[k - 1] + t * (Sf.q[k] - Sf.q[k - 1]);
			}
		}
		return Sf.q.empty() ? 0.0 : Sf.q.back();
	}

	void RebuildTable(const TSharedRef<FState>& S)
	{
		if (!S->Table.IsValid()) return;
		S->Table->ClearChildren();
		const float W = 58.0f, WM = 190.0f;
		auto Row = [&](const TArray<TSharedRef<SWidget>>& Cells)
		{
			TSharedRef<SHorizontalBox> H = SNew(SHorizontalBox);
			for (const TSharedRef<SWidget>& C : Cells) H->AddSlot().AutoWidth() [ C ];
			S->Table->AddSlot().AutoHeight() [ H ];
		};
		Row({ Cell(LOCTEXT("HMethod", "Method"), WM, true), Cell(LOCTEXT("HCl", "Cl"), W, true), Cell(LOCTEXT("HClEx", "Cl exact"), W, true),
			Cell(LOCTEXT("HCdp", "Cd press."), W, true), Cell(LOCTEXT("HCdf", "Cd frict."), W, true), Cell(LOCTEXT("HCm", "Cm c/4"), W, true),
			Cell(LOCTEXT("HA0", "alpha0"), W, true), Cell(LOCTEXT("HCla", "Cl_alpha"), W, true), Cell(LOCTEXT("HCpmin", "Cp min"), W, true),
			Cell(LOCTEXT("HTrU", "tr. up"), W, true), Cell(LOCTEXT("HTrL", "tr. low"), W, true), Cell(LOCTEXT("HSepU", "sep. up"), W, true), Cell(LOCTEXT("HSepL", "sep. low"), W, true),
			Cell(LOCTEXT("HMs", "ms"), W, true) });
		S->Table->AddSlot().AutoHeight().Padding(0, 1) [ SNew(SSeparator) ];
		FString Cues;
		for (int32 i = 0; i < S->Results.Num(); ++i)
		{
			const SectionAeroResult& R = S->Results[i];
			const FLinearColor C = kMethodColors[i % UE_ARRAY_COUNT(kMethodColors)];
			if (!R.ok)
			{
				Row({ Cell(FText::FromString(StdToFString(R.method)), WM, false, C), Cell(FText::FromString(StdToFString(R.error)), W * 13.0f, false, FLinearColor(1.0f, 0.5f, 0.5f)) });
				continue;
			}
			auto Cue = [](const SurfaceCue& Q) { return Q.found ? FormatF(Q.x, 3) : DashText(); };
			Row({ Cell(FText::FromString(StdToFString(R.method)), WM, false, C), Cell(Num(R.cl, 4), W), Cell(Num(R.cl_exact, 4), W),
				Cell(Num(R.cd_pressure, 5), W), Cell(Num(R.cd_friction, 5), W), Cell(Num(R.cm_c4, 4), W),
				Cell(Num(R.alpha0_deg, 2), W), Cell(Num(R.cl_alpha, 3), W), Cell(Num(R.cp_min, 3), W),
				Cell(Cue(R.upper.transition), W), Cell(Cue(R.lower.transition), W), Cell(Cue(R.upper.separation), W), Cell(Cue(R.lower.separation), W),
				Cell(Num(R.solve_ms, 2), W) });
			Cues += FString::Printf(TEXT("%s -- upper: transition %s; separation %s. Lower: transition %s; separation %s. TE gap closed %.4f c, TE angle %.1f deg, %d sweeps, residual %.1e, margin %.2f.\n"),
				*StdToFString(R.method),
				*(R.upper.transition.found ? FString::Printf(TEXT("x/c %.3f (%s)"), R.upper.transition.x, *StdToFString(R.upper.transition.note)) : StdToFString(R.upper.transition.note)),
				*(R.upper.separation.found ? FString::Printf(TEXT("x/c %.3f (%s)"), R.upper.separation.x, *StdToFString(R.upper.separation.note)) : StdToFString(R.upper.separation.note)),
				*(R.lower.transition.found ? FString::Printf(TEXT("x/c %.3f (%s)"), R.lower.transition.x, *StdToFString(R.lower.transition.note)) : StdToFString(R.lower.transition.note)),
				*(R.lower.separation.found ? FString::Printf(TEXT("x/c %.3f (%s)"), R.lower.separation.x, *StdToFString(R.lower.separation.note)) : StdToFString(R.lower.separation.note)),
				R.te_gap_closed, R.te_angle_deg, R.iterations, R.residual, R.convergence_margin);
			for (const std::string& Wn : R.warnings) Cues += TEXT("   warning: ") + StdToFString(Wn) + TEXT("\n");
		}
		if (S->CueText.IsValid()) S->CueText->SetText(FText::FromString(Cues.TrimEnd()));
	}

	void UpdatePolar(const TSharedRef<FState>& S)
	{
		if (!S->PolarClPlot.IsValid() || !S->PolarCmPlot.IsValid()) return;
		TArray<FAerotelicPlotSeries> ClSeries, CmSeries;
		if (S->Working)
		{
			const auto& Methods = SectionAeroMethods();
			SectionAeroRequest Req = RequestFor(*S);
			for (int32 i = 0; i < static_cast<int32>(Methods.size()) && i < S->MethodOn.Num(); ++i)
			{
				if (!S->MethodOn[i]) continue;
				FAerotelicPlotSeries Cl, Cm;
				Cl.Label = StdToFString(Methods[i]->Name());
				Cm.Label = Cl.Label;
				Cl.Color = kMethodColors[i % UE_ARRAY_COUNT(kMethodColors)];
				Cm.Color = Cl.Color;
				Cl.bMarkers = Cm.bMarkers = false;
				for (int32 a = -6; a <= 16; ++a)
				{
					Req.alpha_deg = a;
					SectionAeroResult R;
					try { R = Methods[i]->Solve(Req); } catch (const std::exception&) { continue; }
					if (!R.ok) continue;
					Cl.Points.Add(FVector2D(a, R.cl));
					Cm.Points.Add(FVector2D(a, R.cm_c4));
				}
				ClSeries.Add(MoveTemp(Cl));
				CmSeries.Add(MoveTemp(Cm));
			}
		}
		S->PolarClPlot->SetMessage(LOCTEXT("PolarEmpty", "Pick an airfoil"));
		S->PolarCmPlot->SetMessage(LOCTEXT("PolarEmpty", "Pick an airfoil"));
		S->PolarClPlot->SetSeries(MoveTemp(ClSeries));
		S->PolarCmPlot->SetSeries(MoveTemp(CmSeries));
	}

	// Which poles the sensitivity covers: all but the locked trailing-edge
	// ones (they cannot be moved, and their gradients -- the Kutta point --
	// dwarf every other pole's).
	bool PoleMovable(const FState& S, int32 Index, int32 Count)
	{
		return !(S.bLockTE && (Index == 0 || Index == Count - 1));
	}

	void UpdateSensitivityText(const TSharedRef<FState>& S)
	{
		if (!S->SensText.IsValid()) return;
		if (!S->bShowSensitivity || !S->Sens.ok || !S->Working) { S->SensText->SetText(FText::GetEmpty()); return; }
		const Mat Poles = S->Working->nurbs_curve().poles();
		const int32 N = FMath::Min(Poles.rows(), static_cast<int32>(S->Sens.d_dx.size()));
		FString T = FString::Printf(TEXT("%-5s %8s %8s %12s %12s\n"), TEXT("pole"), TEXT("x/c"), TEXT("y/c"), TEXT("d/dx"), TEXT("d/dy"));
		for (int32 i = 0; i < N; ++i)
		{
			T += FString::Printf(TEXT("%-5d %8.4f %8.4f %12s %12s%s\n"), i, Poles(i, 0), Poles(i, 1),
				FMath::IsFinite(S->Sens.d_dx[i]) ? *FString::Printf(TEXT("%.4g"), S->Sens.d_dx[i]) : TEXT("-"),
				FMath::IsFinite(S->Sens.d_dy[i]) ? *FString::Printf(TEXT("%.4g"), S->Sens.d_dy[i]) : TEXT("-"),
				PoleMovable(*S, i, N) ? TEXT("") : TEXT("  (locked)"));
		}
		S->SensText->SetText(FText::FromString(T.TrimEnd()));
	}

	void UpdateFit(const TSharedRef<FState>& S)
	{
		S->bHasFit = false;
		if (!S->Source || !S->Working) return;
		try
		{
			S->Fit = CompareAirfoilFit(*S->Source, *S->Working);
			S->bHasFit = true;
		}
		catch (const std::exception& E)
		{
			S->RefitNote = E.what();
		}
	}

	void UpdateSensitivity(const TSharedRef<FState>& S)
	{
		S->Sens = PoleSensitivity();
		if (!S->bShowSensitivity || !S->Working) { if (S->Editor.IsValid()) S->Editor->RequestRedraw(); return; }
		const auto& Methods = SectionAeroMethods();
		const SectionAeroMethod* M = nullptr;
		for (int32 i = 0; i < static_cast<int32>(Methods.size()) && i < S->MethodOn.Num(); ++i) if (S->MethodOn[i]) { M = Methods[i]; break; }
		if (M)
		{
			const SectionAeroScalar Scalar = kScalars[FMath::Clamp(S->ScalarIndex, 0, static_cast<int32>(UE_ARRAY_COUNT(kScalars)) - 1)];
			S->Sens = ComputePoleSensitivity(S->Working->Serialize(), *M, RequestFor(*S), Scalar, S->ProbeX);
		}
		UpdateSensitivityText(S);
		if (S->Editor.IsValid()) S->Editor->RequestRedraw();
	}

	void BindEditor(const TSharedRef<FState>& S)
	{
		if (!S->Editor.IsValid()) return;
		TArray<FCurveEditorSlot> Slots;
		if (S->Working)
		{
			FCurveEditorSlot Slot;
			Slot.Curve = &S->Working->mutable_curve();
			Slot.Color = kMethodColors[0];
			Slot.Label = LOCTEXT("SlotLabel", "section");
			Slot.Basis = ECurveBasisPlacement::Below;
			Slot.bFreeX = true;
			if (S->bLockTE)
			{
				const Mat P = Slot.Curve->poles();
				Slot.FixedPoles.Add(0, FVector2D(P(0, 0), P(0, 1)));
				Slot.FixedPoles.Add(-1, FVector2D(P(P.rows() - 1, 0), P(P.rows() - 1, 1)));
			}
			Slots.Add(Slot);
		}
		S->Editor->SetSlots(Slots);
		S->Editor->FitView();
	}

	void Refit(const TSharedRef<FState>& S)
	{
		if (!S->Source) return;
		S->RefitNote.Empty();
		try
		{
			std::vector<double> Given;
			ApproximationKnots Mode = ApproximationKnots::Averaging;
			if (S->KnotMode == 1) Mode = ApproximationKnots::Uniform;
			else if (S->KnotMode == 2 && S->Working)
			{
				const NurbsCurve& C = S->Working->nurbs_curve();
				const std::vector<double> K = C.knots();
				if (static_cast<int32>(K.size()) == S->NPoles - S->Degree - 1 && !C.periodic()) { Mode = ApproximationKnots::Given; Given = K; }
				else S->RefitNote = TEXT("The current knots do not fit this pole count and degree; averaging knots used instead.");
			}
			const int32 MaxPoles = S->Source->fit_points().rows() - 1;
			if (S->NPoles > MaxPoles) { S->NPoles = MaxPoles; S->RefitNote = FString::Printf(TEXT("At most %d poles for this airfoil's %d points."), MaxPoles, MaxPoles + 1); }
			if (S->Degree > S->NPoles - 1) S->Degree = S->NPoles - 1;
			S->Working = Airfoil::Refit(*S->Source, S->Degree, S->NPoles, Mode, Mode == ApproximationKnots::Given ? &Given : nullptr);
			S->ApplyName = StdToFString(S->Working->name);
		}
		catch (const std::exception& E)
		{
			S->RefitNote = E.what();
			return;
		}
		BindEditor(S);
		UpdateFit(S);
		Solve(S);
	}

	void SetSource(const TSharedRef<FState>& S, std::shared_ptr<Airfoil> A)
	{
		S->Source = std::move(A);
		S->Working.reset();
		S->Results.Empty();
		S->Sens = PoleSensitivity();
		if (S->Source)
		{
			const int32 MaxPoles = S->Source->fit_points().rows() - 1;
			S->NPoles = FMath::Clamp(S->NPoles, S->Degree + 1, FMath::Max(S->Degree + 1, MaxPoles));
			if (S->KnotMode == 2) S->KnotMode = 0;
			Refit(S);
		}
		else
		{
			BindEditor(S);
			Solve(S);
		}
	}

	// The editor's overlay: the library contour, and the sensitivity arrows.
	TArray<FCurveEditorOutlinePath> Overlay(const TSharedRef<FState>& S)
	{
		TArray<FCurveEditorOutlinePath> Paths;
		if (S->Source)
		{
			FCurveEditorOutlinePath P;
			P.Color = kSource;
			P.Label = LOCTEXT("SourceOutline", "library section");
			P.bClosed = true;
			P.LineWidth = 1.0f;
			const Mat& Pts = S->Source->fit_points();
			for (int32 i = 0; i < Pts.rows(); ++i) P.Points.Add(FVector2D(Pts(i, 0), Pts(i, 1)));
			Paths.Add(MoveTemp(P));
		}
		if (S->bShowSensitivity && S->Sens.ok && S->Working)
		{
			const Mat Poles = S->Working->nurbs_curve().poles();
			const int32 N = FMath::Min(Poles.rows(), static_cast<int32>(S->Sens.d_dx.size()));
			double Longest = 0.0;
			for (int32 i = 0; i < N; ++i)
			{
				if (!PoleMovable(*S, i, N)) continue;
				const double L = std::hypot(S->Sens.d_dx[i], S->Sens.d_dy[i]);
				if (FMath::IsFinite(L)) Longest = FMath::Max(Longest, L);
			}
			if (Longest > 0.0)
			{
				// Length grows with the square root of the magnitude (the
				// largest 0.08 c) so the mid-chord poles, an order of magnitude
				// below the ones by the trailing edge, stay visible; the table
				// has the numbers.
				for (int32 i = 0; i < N; ++i)
				{
					if (!PoleMovable(*S, i, N)) continue;
					const double Gx = S->Sens.d_dx[i], Gy = S->Sens.d_dy[i];
					if (!FMath::IsFinite(Gx) || !FMath::IsFinite(Gy)) continue;
					const double L = std::hypot(Gx, Gy);
					if (L < 1e-12 * Longest) continue;
					const FVector2D A(Poles(i, 0), Poles(i, 1));
					const FVector2D D = FVector2D(Gx, Gy) / L * (0.08 * std::sqrt(L / Longest));
					if (D.Size() < 1e-6) continue;
					const FVector2D B = A + D;
					const FVector2D U = D / D.Size();
					const FVector2D V(-U.Y, U.X);
					FCurveEditorOutlinePath Arrow;
					Arrow.Color = kArrow;
					Arrow.LineWidth = 1.5f;
					Arrow.Points = { A, B, B - U * 0.012 + V * 0.006, B, B - U * 0.012 - V * 0.006 };
					if (Paths.Num() == 1) Arrow.Label = FText::Format(LOCTEXT("ArrowLabel", "d({0})/d(pole), sqrt scale, longest = {1} per c"), FText::FromString(SectionAeroScalarName(kScalars[FMath::Clamp(S->ScalarIndex, 0, 4)])), FormatG(Longest, 3));
					Paths.Add(MoveTemp(Arrow));
				}
			}
		}
		return Paths;
	}

	const FText kHelp = LOCTEXT("Help",
		"WHAT THIS IS\n"
		"The two-dimensional, incompressible (Mach 0) aerodynamics of one airfoil section per unit span, and a way to re-represent the section with fewer design variables while watching what that costs. Pick an airfoil from the library (the Selig database, NACA codes, .dat files, or one already in the project); it is re-fitted and analysed at once, and every edit re-analyses it.\n\n"
		"REPRESENTATION\n"
		"A library airfoil is stored as a B-spline interpolating every one of its points -- one pole per point, typically 60 to 160 poles, far too many design variables to optimise. The Representation group fits a least-squares B-spline of the chosen degree and pole count through the same points (Piegl & Tiller 9.4.1, with Hoschek's parameter correction), the two trailing-edge points interpolated exactly. Knots: Averaging places them by the data's chord-length parameters (every span holds data), Uniform spaces them equally, Keep current reuses the knots now in the editor (after you have dragged or inserted some). The error group reports the largest and RMS normal distance from the library points to the new curve, where the largest is, and the change of maximum thickness and camber. The worst point is almost always the leading edge, where curvature is highest: add poles or insert a knot there.\n\n"
		"THE EDITOR\n"
		"The curve editor is the one used by every design panel: drag poles, double-click to add one, right-click a pole or the knot track below for weights, removal, knot insertion and deletion, Reset Knots. The library section is the grey outline behind. The trailing-edge poles are held in place while 'Lock trailing edge' is on. Every edit re-analyses; the error and the sensitivities update once the poles are still.\n\n"
		"SENSITIVITY\n"
		"At each movable pole an arrow shows the gradient of the chosen scalar -- Cl, Cm about c/4, the friction drag, or Cp at a probe station on the upper or lower surface -- with respect to that pole's position, by central differences of the first enabled method (two solves per pole per axis). The largest gradient's arrow is 0.08 c long and its magnitude is stated; the others' lengths follow the SQUARE ROOT of their magnitudes, so the mid-chord poles (an order of magnitude below those by the trailing edge) stay visible. The table under the arrows gives every pole's gradient per unit chord: moving a pole 0.01 c along its arrow changes the scalar by about a hundredth of the number shown. Locked trailing-edge poles are listed but not drawn.\n\n"
		"THE METHODS\n"
		"Each enabled method is run on the same closed contour and its pressure distribution is drawn on the chart (upper surface solid, lower dashed, Cp negative upward) with its coefficients in the table, so methods can be compared directly. The first method is Theodorsen's exact conformal map (NACA TR 411 and, with Garrick, TR 452), computed as David Ives recommends in 'A Modern Look at Conformal Mapping, Including Multiply Connected Regions', AIAA Journal 14(8):1006-1011, 1976: a Karman-Trefftz transformation with exponent 2 - tau/pi (tau the trailing-edge angle) opens the trailing edge into a near-circle, then Theodorsen's map to the exact circle is found by iterating Garrick's conjugate-function relation between the near-circle's log-radius and its angle correction, each sweep an FFT. The circle flow at incidence with the Kutta condition at the trailing edge's image gives the surface speed; Cp = 1 - (V/U)^2. Cl is both integrated from Cp and taken exactly from the circulation (Kutta-Joukowski; 'Cl exact'); the integrated pressure drag is a discretisation residual (d'Alembert) and should be near zero. The map's diagnostics -- sweeps, residual, the convergence margin max |d eps/d phi| (must stay below 1) -- are listed under the table.\n\n"
		"TRAILING EDGE\n"
		"Every section is closed before analysis in XFOIL's manner: an open gap is blended out over the aft 30% of chord, the upper surface moving down and the lower up by half the gap times ((x - 0.7)/0.3)^2. The gap removed and the resulting trailing-edge angle are reported. A finite trailing-edge angle is an inviscid stagnation point (Cp = 1 there); a cusp has a finite speed.\n\n"
		"VISCOUS CUES (shared by every method)\n"
		"Reynolds number: yours, on the chord. Transition cue (yellow): on each surface from the stagnation point, the earlier of the station where Re_x = Re (V/U)(s/c) reaches 3.5e6 and the first adverse pressure gradient (the suction peak). Skin-friction drag: the turbulent flat-plate law Cf = 0.0592 Re_s^-0.2 at the local edge speed from the stagnation point on both surfaces, projected on the wind -- 'Cd frict.' (about 0.0094 for a plate at Re 1e6; the pressure Cd beside it is the inviscid residual, not a drag). Separation cue (red): Stratford's criterion for turbulent separation (J. Fluid Mech. 5:1-16, 1959), fully turbulent from stagnation given the transition cue's low fidelity: from the suction peak, Cpbar = 1 - (V/Vmax)^2, x measured from the equivalent flat-plate origin x_EL = integral of (V/Vmax)^3 ds over the favourable run, and separation where Cpbar (x dCpbar/dx)^1/2 (1e-6 Re_x)^-0.1 reaches 0.39 (0.35 where d2p/dx2 < 0). The criterion holds while Cpbar <= 4/7; the grey marker shows where that is exceeded and the cue stops. Because an inviscid distribution recovers to stagnation at a finite-angle trailing edge, the criterion often trips in the last few percent of chord at low incidence: read that as 'no separation ahead of the trailing-edge recovery'. 'Show Stratford S' draws the parameter on the right axis.\n\n"
		"POLAR\n"
		"Cl and Cm c/4 of each method from -6 to 16 degrees, two charts: the inviscid lift slope and zero-lift angle. No stall, of course.\n\n"
		"APPLY\n"
		"Adds the section as it is in the editor (its reduced representation and any edits) to the session's custom airfoil library under the name given; every wing and blade section picker lists it under Custom, and a project that uses it stores the curve itself. The library airfoil is never changed.");
}

void ShowAerotelicSectionAeroDialog(TFunction<void(const FText&)> Status, std::shared_ptr<Airfoil> Initial)
{
	if (TSharedPtr<SWindow> Existing = GSectionAeroWindow.Pin())
	{
		Existing->BringToFront();
		FSlateApplication::Get().SetKeyboardFocus(Existing);
		if (!Initial) return;
		Existing->RequestDestroyWindow();
	}

	TSharedRef<FState> S = MakeShared<FState>();
	S->Status = Status;
	const auto& Methods = SectionAeroMethods();
	S->MethodOn.Init(true, static_cast<int32>(Methods.size()));

	// ---- left: the editor and the representation ------------------------------
	FSimpleDelegate OnEdited = FSimpleDelegate::CreateLambda([S]()
	{
		if (!S->Working) return;
		S->Working->CurveEdited();
		S->NPoles = S->Working->nurbs_curve().n_poles();
		S->Degree = S->Working->nurbs_curve().degree();
		Solve(S);
		S->bFitDirty = true;
		S->LastEdit = FPlatformTime::Seconds();
	});
	S->Editor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("EdX", "x/c"))
		.YLabel(LOCTEXT("EdY", "y/c"))
		.Title(LOCTEXT("EdTitle", "Section  ·  poles of the re-representation  ·  library contour in grey"))
		.EqualAspect(true)
		.DefaultYMax(0.2)
		.OnChanged(OnEdited);
	S->Editor->SetOutlineProvider(FOnGetOutlinePaths::CreateLambda([S]() { return Overlay(S); }));

	const TArray<FText> KnotLabels = { LOCTEXT("KnotsAvg", "Averaging (by the data)"), LOCTEXT("KnotsUni", "Uniform"), LOCTEXT("KnotsKeep", "Keep current knots") };
	TSharedRef<SWidget> Representation =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[
				SNew(SButton).Text(LOCTEXT("Pick", "Pick airfoil..."))
				.OnClicked_Lambda([S]()
				{
					TOptional<std::shared_ptr<Airfoil>> Picked = PromptForAirfoil({}, TEXT("2412"));
					if (Picked.IsSet() && Picked.GetValue()) SetSource(S, Picked.GetValue());
					return FReply::Handled();
				})
			]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[
				SNew(STextBlock).AutoWrapText(true)
				.Text_Lambda([S]() { return S->Source ? FText::Format(LOCTEXT("SourceIs", "{0}: {1} points, {2} poles as stored"), FText::FromString(StdToFString(S->Source->name)), FText::AsNumber(S->Source->fit_points().rows()), FText::AsNumber(S->Source->nurbs_curve().n_poles())) : LOCTEXT("NoSource", "No airfoil picked."); })
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Degree", "Degree:")) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
			[
				SNew(SBox).WidthOverride(56)
				[
					SNew(SSpinBox<int32>).MinValue(1).MaxValue(9).Delta(1)
					.Value_Lambda([S]() { return S->Degree; })
					.OnValueCommitted_Lambda([S](int32 V, ETextCommit::Type) { if (V == S->Degree) return; S->Degree = V; if (S->NPoles < V + 1) S->NPoles = V + 1; Refit(S); })
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Poles", "Poles:")) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
			[
				SNew(SBox).WidthOverride(64)
				[
					SNew(SSpinBox<int32>).MinValue(2).MaxValue(400).Delta(1)
					.Value_Lambda([S]() { return S->NPoles; })
					.OnValueCommitted_Lambda([S](int32 V, ETextCommit::Type) { if (V == S->NPoles) return; S->NPoles = FMath::Max(V, S->Degree + 1); Refit(S); })
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Knots", "Knots:")) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
			[
				SNew(SBox).WidthOverride(170) [ ChoiceCombo(KnotLabels, [S]() { return S->KnotMode; }, [S](int32 I) { S->KnotMode = I; Refit(S); }) ]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[
				SNew(SButton).Text(LOCTEXT("Refit", "Refit")).ToolTipText(LOCTEXT("RefitTip", "Fit the library points again with these settings, discarding pole edits"))
				.IsEnabled_Lambda([S]() { return S->Source != nullptr; })
				.OnClicked_Lambda([S]() { Refit(S); return FReply::Handled(); })
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SCheckBox)
				.IsChecked_Lambda([S]() { return S->bLockTE ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
				.OnCheckStateChanged_Lambda([S](ECheckBoxState St) { S->bLockTE = St == ECheckBoxState::Checked; BindEditor(S); })
				[ SNew(STextBlock).Text(LOCTEXT("LockTE", "Lock trailing edge")) ]
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
		[
			SNew(STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.7f, 0.4f)))
			.Visibility_Lambda([S]() { return S->RefitNote.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
			.Text_Lambda([S]() { return FText::FromString(S->RefitNote); })
		];

	auto FitText = [S](TFunction<FText(const AirfoilFitReport&)> F) { return TAttribute<FText>::CreateLambda([S, F]() { return S->bHasFit ? F(S->Fit) : DashText(); }); };
	TSharedRef<SWidget> ErrorGroup =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ErrMax", "Largest deviation:"), FitText([](const AirfoilFitReport& R) { return FText::Format(LOCTEXT("ErrMaxV", "{0} c at x/c {1} ({2})"), FormatG(R.max_distance, 3), FormatF(R.max_at_x, 3), R.max_on_upper ? LOCTEXT("Upper", "upper") : LOCTEXT("Lower", "lower")); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ErrRms", "RMS deviation:"), FitText([](const AirfoilFitReport& R) { return FText::Format(LOCTEXT("ErrRmsV", "{0} c over {1} points"), FormatG(R.rms_distance, 3), FText::AsNumber(R.n_points)); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ErrT", "Max thickness change:"), FitText([](const AirfoilFitReport& R) { return FormatG(R.delta_max_thickness, 3); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ErrC", "Max camber change:"), FitText([](const AirfoilFitReport& R) { return FormatG(R.delta_max_camber, 3); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ErrTE", "Trailing-edge gap:"), FitText([](const AirfoilFitReport& R) { return FText::Format(LOCTEXT("ErrTEV", "{0} -> {1}"), FormatG(R.te_gap_source, 3), FormatG(R.te_gap_fitted, 3)); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ErrRep", "Representation:"), FitText([](const AirfoilFitReport& R) { return FText::Format(LOCTEXT("ErrRepV", "{0} poles, degree {1}: {2} design variables"), FText::AsNumber(R.n_poles), FText::AsNumber(R.degree), FText::AsNumber(2 * R.n_poles)); })) ];

	const TArray<FText> ScalarLabels = { LOCTEXT("ScCl", "Cl"), LOCTEXT("ScCm", "Cm c/4"), LOCTEXT("ScCdf", "Cd friction"), LOCTEXT("ScCpU", "Cp upper at probe x/c"), LOCTEXT("ScCpL", "Cp lower at probe x/c") };
	TSharedRef<SWidget> SensGroup =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 10, 0)
		[
			SNew(SCheckBox)
			.IsChecked_Lambda([S]() { return S->bShowSensitivity ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([S](ECheckBoxState St) { S->bShowSensitivity = St == ECheckBoxState::Checked; S->bSensDirty = true; S->LastEdit = 0.0; })
			[ SNew(STextBlock).Text(LOCTEXT("ShowSens", "Show sensitivity arrows of")) ]
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 10, 0)
		[ SNew(SBox).WidthOverride(180) [ ChoiceCombo(ScalarLabels, [S]() { return S->ScalarIndex; }, [S](int32 I) { S->ScalarIndex = I; S->bSensDirty = true; S->LastEdit = 0.0; }) ] ]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Probe", "probe x/c:")) ]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SBox).WidthOverride(70)
			[
				SNew(SAerotelicNumericEntry).Value_Lambda([S]() { return S->ProbeX; }).MinValue(0.0).MaxValue(1.0).Delta(0.05).LiveUpdate(false)
				.OnValueCommitted_Lambda([S](double V, ETextCommit::Type) { S->ProbeX = V; S->bSensDirty = true; S->LastEdit = 0.0; })
			]
		]
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(10, 0, 0, 0)
		[
			SNew(STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
			.Text_Lambda([S]()
			{
				if (!S->bShowSensitivity) return FText::GetEmpty();
				if (!S->Sens.ok) return S->Sens.error.empty() ? LOCTEXT("SensPending", "computing...") : FText::FromString(StdToFString(S->Sens.error));
				if (S->Sens.error.empty()) return FText::Format(LOCTEXT("SensBase", "base value {0}"), FormatG(S->Sens.base, 4));
				return FText::Format(LOCTEXT("SensBaseErr", "base value {0}; {1}"), FormatG(S->Sens.base, 4), FText::FromString(StdToFString(S->Sens.error)));
			})
		];
	S->SensText = SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Mono", 8)).ColorAndOpacity(FSlateColor(FLinearColor(0.75f, 0.75f, 0.75f)));
	TSharedRef<SWidget> SensTable = SNew(SBox).MaxDesiredHeight(96)
		.Visibility_Lambda([S]() { return S->bShowSensitivity && S->Sens.ok ? EVisibility::Visible : EVisibility::Collapsed; })
		[ SNew(SScrollBox) + SScrollBox::Slot() [ S->SensText.ToSharedRef() ] ];

	TSharedRef<SWidget> LeftColumn =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().FillHeight(1.0f) [ S->Editor.ToSharedRef() ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ SensGroup ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0) [ SensTable ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ MakeGroup(LOCTEXT("RepGroup", "Representation"), Representation) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ MakeGroup(LOCTEXT("ErrGroup", "Error against the library points"), ErrorGroup) ];

	// ---- right: conditions, the Cp chart, the table ----------------------------
	TSharedRef<SHorizontalBox> MethodChecks = SNew(SHorizontalBox);
	for (int32 i = 0; i < static_cast<int32>(Methods.size()); ++i)
	{
		const FString Name = StdToFString(Methods[i]->Name());
		std::string Why;
		const bool bAvail = Methods[i]->Available(&Why);
		MethodChecks->AddSlot().AutoWidth().Padding(0, 0, 10, 0)
		[
			SNew(SCheckBox)
			.IsEnabled(bAvail)
			.IsChecked_Lambda([S, i]() { return S->MethodOn.IsValidIndex(i) && S->MethodOn[i] ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([S, i](ECheckBoxState St) { if (S->MethodOn.IsValidIndex(i)) { S->MethodOn[i] = St == ECheckBoxState::Checked; Solve(S); } })
			[ SNew(STextBlock).Text(FText::FromString(bAvail ? Name : Name + TEXT(" (") + StdToFString(Why) + TEXT(")"))).ColorAndOpacity(FSlateColor(kMethodColors[i % UE_ARRAY_COUNT(kMethodColors)])) ]
		];
	}
	const TArray<FText> PointLabels = { LOCTEXT("P256", "256"), LOCTEXT("P512", "512"), LOCTEXT("P1024", "1024") };
	TSharedRef<SWidget> Conditions =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Alpha", "Angle of attack (deg):")) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
			[
				SNew(SBox).WidthOverride(80)
				[
					SNew(SAerotelicNumericEntry).Value_Lambda([S]() { return S->AlphaDeg; }).MinValue(-30.0).MaxValue(30.0).Delta(0.5).LiveUpdate(false)
					.OnValueCommitted_Lambda([S](double V, ETextCommit::Type) { if (V == S->AlphaDeg) return; S->AlphaDeg = V; Solve(S); })
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Re", "Reynolds (chord):")) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
			[
				SNew(SBox).WidthOverride(110)
				[
					SNew(SAerotelicNumericEntry).Value_Lambda([S]() { return S->Reynolds; }).MinValue(1.0e3).MaxValue(1.0e10).Delta(1.0e5).LiveUpdate(false)
					.OnValueCommitted_Lambda([S](double V, ETextCommit::Type) { if (V == S->Reynolds) return; S->Reynolds = V; Solve(S); })
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("Points", "Points:")) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
			[ SNew(SBox).WidthOverride(80) [ ChoiceCombo(PointLabels, [S]() { return S->PointsIndex; }, [S](int32 I) { S->PointsIndex = I; Solve(S); }) ] ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0) [ SNew(STextBlock).Text(LOCTEXT("Methods", "Methods:")) ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center) [ MethodChecks ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SCheckBox)
				.IsChecked_Lambda([S]() { return S->bShowStratford ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
				.OnCheckStateChanged_Lambda([S](ECheckBoxState St) { S->bShowStratford = St == ECheckBoxState::Checked; UpdateCpPlot(S); })
				[ SNew(STextBlock).Text(LOCTEXT("ShowS", "Show Stratford S")) ]
			]
		];

	S->CpPlot = SNew(SAerotelicLinePlot)
		.Title(LOCTEXT("CpTitle", "Pressure distribution"))
		.XLabel(LOCTEXT("CpX", "x/c"))
		.YLabel(LOCTEXT("CpY", "Cp (suction up)"))
		.InvertY(true)
		.DesiredSize(FVector2D(560, 360));
	S->Table = SNew(SVerticalBox);
	S->CueText = SNew(STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f))).Font(FCoreStyle::GetDefaultFontStyle("Regular", 8));

	TSharedRef<SWidget> RightColumn =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ Conditions ]
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 6, 0, 0) [ S->CpPlot.ToSharedRef() ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(SScrollBox).Orientation(Orient_Horizontal)
			+ SScrollBox::Slot() [ S->Table.ToSharedRef() ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0) [ SNew(SBox).MaxDesiredHeight(90) [ SNew(SScrollBox) + SScrollBox::Slot() [ S->CueText.ToSharedRef() ] ] ];

	TSharedRef<SWidget> SectionTab =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(0.48f).Padding(0, 0, 8, 0) [ LeftColumn ]
		+ SHorizontalBox::Slot().FillWidth(0.52f) [ RightColumn ];

	S->PolarClPlot = SNew(SAerotelicLinePlot)
		.Title(LOCTEXT("PolarClTitle", "Lift coefficient against angle of attack"))
		.XLabel(LOCTEXT("PolarX", "angle of attack (deg)"))
		.YLabel(LOCTEXT("PolarY", "Cl"))
		.DesiredSize(FVector2D(800, 300));
	S->PolarCmPlot = SNew(SAerotelicLinePlot)
		.Title(LOCTEXT("PolarCmTitle", "Pitching moment about c/4 against angle of attack"))
		.XLabel(LOCTEXT("PolarX", "angle of attack (deg)"))
		.YLabel(LOCTEXT("PolarYCm", "Cm c/4"))
		.DesiredSize(FVector2D(800, 300));
	TSharedRef<SWidget> PolarTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().FillHeight(1.0f) [ S->PolarClPlot.ToSharedRef() ]
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 6, 0, 0) [ S->PolarCmPlot.ToSharedRef() ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ MakeInfoLabel(LOCTEXT("PolarInfo", "Each enabled method from -6 to 16 degrees at the Reynolds number of the Section tab: inviscid, so straight lines (no stall).")) ];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SectionTab", "Section"), SectionTab });
	Tabs.Add({ LOCTEXT("PolarTab", "Polar"), PolarTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	// ---- footer -----------------------------------------------------------------
	TSharedRef<SWidget> Footer =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 8, 0)
		[
			SNew(STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
			.Text_Lambda([S]()
			{
				if (!S->Source) return LOCTEXT("FootPick", "Pick an airfoil from the library to begin.");
				if (!S->Working) return LOCTEXT("FootNoFit", "The re-fit failed; see the note under Representation.");
				return LOCTEXT("FootReady", "The library airfoil is unchanged; Apply adds the section as edited to the custom library under this name.");
			})
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
		[
			SNew(SBox).WidthOverride(260)
			[
				SNew(SEditableTextBox)
				.Text_Lambda([S]() { return FText::FromString(S->ApplyName); })
				.OnTextCommitted_Lambda([S](const FText& T, ETextCommit::Type) { S->ApplyName = T.ToString(); })
				.HintText(LOCTEXT("NameHint", "name in the custom library"))
			]
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SButton).Text(LOCTEXT("Apply", "Apply"))
			.ToolTipText(LOCTEXT("ApplyTip", "Add the section as it is in the editor to the session's custom airfoil library; every section picker lists it under Custom"))
			.IsEnabled_Lambda([S]() { return S->Working != nullptr && !S->ApplyName.TrimStartAndEnd().IsEmpty(); })
			.OnClicked_Lambda([S]()
			{
				if (!S->Working) return FReply::Handled();
				std::shared_ptr<Airfoil> Copy = S->Working->Clone();
				const FString Name = S->ApplyName.TrimStartAndEnd();
				Copy->name = std::string(TCHAR_TO_UTF8(*Name));
				FString Key = TEXT("custom_") + Name;
				for (TCHAR& Ch : Key) if (!FChar::IsAlnum(Ch) && Ch != TEXT('_') && Ch != TEXT('-') && Ch != TEXT('.')) Ch = TEXT('_');
				AerotelicSelig::AddCustomAirfoil(Key, Copy);
				const FText Msg = FText::Format(LOCTEXT("Applied", "'{0}' added to the custom airfoil library ({1} poles, degree {2}); section pickers list it under Custom."), FText::FromString(Name), FText::AsNumber(Copy->nurbs_curve().n_poles()), FText::AsNumber(Copy->nurbs_curve().degree()));
				if (S->Status) S->Status(Msg);
				AerotelicMessageBox(EAppMsgType::Ok, Msg, LOCTEXT("AppliedTitle", "Section Aerodynamics"));
				return FReply::Handled();
			})
		];

	const FVector2D Size(1240, 760);
	TSharedPtr<SAerotelicTabStrip> TabStrip;
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Section Aerodynamics"))
		.ClientSize(Size)
		.SupportsMaximize(true)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().FillHeight(1.0f) [ SAssignNew(TabStrip, SAerotelicTabStrip, Tabs) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0) [ Footer ]
			]
		];

	// Deferred work once the poles have been still for a moment: the fit
	// report (closest points of every library point), the sensitivities
	// (two solves per pole per axis) and the polar sweep.
	S->Ticker = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([S](float)
	{
		const double Now = FPlatformTime::Seconds();
		if (Now - S->LastEdit < 0.2) return true;
		if (S->bFitDirty) { S->bFitDirty = false; UpdateFit(S); }
		else if (S->bSensDirty) { S->bSensDirty = false; UpdateSensitivity(S); }
		else if (S->bPolarDirty) { S->bPolarDirty = false; UpdatePolar(S); }
		return true;
	}), 0.1f);
	// On close: stop the ticker and drop the state's widget pointers -- the
	// widgets' lambdas hold the state, so without this the two would keep
	// each other alive after the window is gone.
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([S](const TSharedRef<SWindow>&)
	{
		FTSTicker::GetCoreTicker().RemoveTicker(S->Ticker);
		if (S->Editor.IsValid()) S->Editor->SetSlots({});
		S->Editor.Reset();
		S->CpPlot.Reset();
		S->PolarClPlot.Reset();
		S->PolarCmPlot.Reset();
		S->Table.Reset();
		S->CueText.Reset();
		S->SensText.Reset();
		S->Status = nullptr;
	}));

	GSectionAeroWindow = Window;
	ShowAerotelicDialogWindow(Window, Size);

	if (Initial) SetSource(S, Initial);
	else { BindEditor(S); Solve(S); }

	// -AerotelicSectionAeroSmoke: a developer run (the High Order Panel
	// dialog's pattern) that opens on a NACA 2412 when no airfoil was given,
	// waits for the deferred fit report, sensitivities and polar, saves a
	// screenshot of the Section tab and of the Polar tab to Saved/Screenshots,
	// and quits -- the way the widget tree is exercised without a hand at
	// the mouse.
	if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicSectionAeroSmoke")))
	{
		if (!Initial) SetSource(S, std::shared_ptr<Airfoil>(Airfoil::Naca4("2412", 81)));
		TWeakPtr<SWindow> WeakWindow = Window;
		TWeakPtr<SAerotelicTabStrip> WeakTabs = TabStrip;
		auto Shot = [WeakWindow](const FString& Name)
		{
			TSharedPtr<SWindow> W = WeakWindow.Pin();
			if (!W.IsValid()) return;
			TArray<FColor> Pixels;
			FIntVector PixelSize;
			if (!FSlateApplication::Get().TakeScreenshot(W.ToSharedRef(), Pixels, PixelSize) || Pixels.Num() == 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("SectionAero smoke: screenshot %s failed"), *Name);
				return;
			}
			for (FColor& C : Pixels) C.A = 255;
			TArray64<uint8> Png;
			FImageUtils::PNGCompressImageArray(PixelSize.X, PixelSize.Y, Pixels, Png);
			const FString Path = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Screenshots"), Name);
			FFileHelper::SaveArrayToFile(Png, *Path);
			UE_LOG(LogTemp, Log, TEXT("SectionAero smoke: wrote %s (%d x %d)"), *Path, PixelSize.X, PixelSize.Y);
		};
		auto Later = [](float Seconds, TFunction<void()> Fn)
		{
			FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Fn](float) { Fn(); return false; }), Seconds);
		};
		float FirstDelay = 3.0f;
		FParse::Value(FCommandLine::Get(), TEXT("AerotelicSectionAeroSmokeDelay="), FirstDelay);
		Later(FirstDelay, [S, Shot, Later, WeakTabs]()
		{
			if (S->Results.Num() > 0 && S->Results[0].ok)
			{
				const SectionAeroResult& R = S->Results[0];
				UE_LOG(LogTemp, Log, TEXT("SectionAero smoke: %s cl %.4f cm_c4 %.4f cd_f %.5f alpha0 %.3f; fit max %.2e; sensitivity ok %d (%d poles)"),
					*StdToFString(R.method), R.cl, R.cm_c4, R.cd_friction, R.alpha0_deg, S->Fit.max_distance, S->Sens.ok ? 1 : 0, static_cast<int32>(S->Sens.d_dx.size()));
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("SectionAero smoke: no result (%s)"), S->Results.Num() > 0 ? *StdToFString(S->Results[0].error) : TEXT("no method ran"));
			}
			Shot(TEXT("section_aero_smoke_section.png"));
			if (TSharedPtr<SAerotelicTabStrip> T = WeakTabs.Pin()) T->SetActiveTab(1);
			Later(1.5f, [Shot]()
			{
				Shot(TEXT("section_aero_smoke_polar.png"));
				UE_LOG(LogTemp, Log, TEXT("SectionAero smoke: done, quitting"));
				FPlatformMisc::RequestExit(false);
			});
		});
	}
}

#undef LOCTEXT_NAMESPACE
