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
#include "Entity/AerotelicMeshing.h"

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

// Last: OCCT's Handle() macro vs engine headers.
#include "aerotelic/wing_panels.hpp"
#include "aerotelic/panel_loads.hpp"
#include "aerotelic/colormaps.hpp"
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "HighOrderPanelDialog"

namespace
{
	constexpr double kPi = 3.14159265358979323846;

	using aerotelic::project_yaml::EntityType;
	using aerotelic::panel::Network;
	using aerotelic::panel::NetworkResult;
	using aerotelic::panel::NetworkType;
	using aerotelic::panel::PanelResult;
	using aerotelic::panel::Solution;

	// The bodies the method can panel: every lofted solid. A wing lifts (a
	// doublet network and its wake); the rest are closed source networks
	// built from their section functions. Boolean results and lattices are
	// triangle soups with no sections to spline, and a propeller is left
	// out (rotating blades are not a steady potential flow).
	bool IsPanelable(EntityType T)
	{
		return T == EntityType::Wing || T == EntityType::Fuselage || T == EntityType::Nacelle || T == EntityType::LoftedNacelle || T == EntityType::Primitive;
	}

	TArray<FAerotelicBodyChoicePtr> PanelChoices()
	{
		TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
		// One row per entity: a mirrored entity is solved with its image (or
		// its reflected copy), never as a second pick.
		Choices.RemoveAll([](const FAerotelicBodyChoicePtr& C)
		{
			return !C->Operand.Entity.IsValid() || C->Operand.bMirror || C->Operand.Entity->GetModel() == nullptr || !IsPanelable(C->Operand.Entity->GetEntityType());
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

	// The corners of a network placed as the entity sits (project units).
	void PlaceNetwork(Network& N, const FTransform& Transform)
	{
		aerotelic::TriMesh M;
		M.vertices = N.corners;
		AerotelicMeshing::PlaceMesh(M, Transform);
		N.corners = M.vertices;
	}

	// The reflection of a network in y = 0, facing the same way (outward or
	// up): a reflection reverses the panel normals, so the columns are
	// reversed again. Edge 0 (a wake's head) stays edge 0.
	Network MirroredNetwork(const Network& N)
	{
		Network M = N;
		M.name = N.name + " (mirror)";
		for (aerotelic::Vec3& P : M.corners) P.y = -P.y;
		aerotelic::panel::FlipNormals(M);
		return M;
	}

	// A model's loft with its section function placed into the world, so the
	// network -- and its cut at the symmetry plane -- is built there.
	aerotelic::NamedLoft PlacedLoft(const aerotelic::NamedLoft& Loft, const FTransform& Transform)
	{
		aerotelic::NamedLoft Out = Loft;
		std::function<aerotelic::Mat(double, const std::vector<double>&)> Section = Loft.source.section;
		Out.source.section = [Section, Transform](double u, const std::vector<double>& v)
		{
			aerotelic::Mat Pts = Section(u, v);
			aerotelic::TriMesh M;
			for (int i = 0; i < Pts.rows(); ++i) M.vertices.push_back({ Pts(i, 0), Pts(i, 1), Pts(i, 2) });
			AerotelicMeshing::PlaceMesh(M, Transform);
			for (int i = 0; i < Pts.rows(); ++i) { Pts(i, 0) = M.vertices[i].x; Pts(i, 1) = M.vertices[i].y; Pts(i, 2) = M.vertices[i].z; }
			return Pts;
		};
		return Out;
	}

	// The panels of every network as quads with their results, legacy VTK
	// polydata in SI lengths (as the Kratos route's file is), so a viewer
	// can colour the bodies by Cp.
	bool WritePanelsVtk(const FString& Path, const std::vector<Network>& Nets, const Solution& Sol, double LengthToSI, FString& OutError)
	{
		FString S;
		S += TEXT("# vtk DataFile Version 3.0\nAerotelic high order panel method: panels, doublet/source strength, pressure coefficient\nASCII\nDATASET POLYDATA\n");
		int32 NPanels = 0;
		for (const NetworkResult& NR : Sol.networks) NPanels += static_cast<int32>(NR.panels.size());
		S += FString::Printf(TEXT("POINTS %d double\n"), 4 * NPanels);
		for (size_t k = 0; k < Sol.networks.size(); ++k)
		{
			const Network& Net = Nets[k];
			const NetworkResult& NR = Sol.networks[k];
			for (int32 i = 0; i < NR.panel_rows; ++i)
			{
				for (int32 j = 0; j < NR.panel_cols; ++j)
				{
					const aerotelic::Vec3 C[4] = { Net.Corner(i + 1, j + 1), Net.Corner(i + 1, j + 2), Net.Corner(i + 2, j + 2), Net.Corner(i + 2, j + 1) };
					for (const aerotelic::Vec3& P : C) S += FString::Printf(TEXT("%.9g %.9g %.9g\n"), P.x * LengthToSI, P.y * LengthToSI, P.z * LengthToSI);
				}
			}
		}
		S += FString::Printf(TEXT("POLYGONS %d %d\n"), NPanels, 5 * NPanels);
		for (int32 p = 0; p < NPanels; ++p) S += FString::Printf(TEXT("4 %d %d %d %d\n"), 4 * p, 4 * p + 1, 4 * p + 2, 4 * p + 3);
		S += FString::Printf(TEXT("CELL_DATA %d\n"), NPanels);
		auto Field = [&](const TCHAR* Name, TFunction<double(const PanelResult&)> Get)
		{
			S += FString::Printf(TEXT("SCALARS %s double 1\nLOOKUP_TABLE default\n"), Name);
			for (const NetworkResult& NR : Sol.networks)
				for (const PanelResult& P : NR.panels) S += FString::Printf(TEXT("%.9g\n"), Get(P));
		};
		Field(TEXT("Cp_upper"), [](const PanelResult& P) { return P.cp_upper; });
		Field(TEXT("Cp_lower"), [](const PanelResult& P) { return P.cp_lower; });
		Field(TEXT("doublet"), [](const PanelResult& P) { return P.mu; });
		Field(TEXT("source"), [](const PanelResult& P) { return P.sigma; });
		Field(TEXT("curvature_delta"), [](const PanelResult& P) { return P.delta; });
		S += TEXT("SCALARS network int 1\nLOOKUP_TABLE default\n");
		for (size_t k = 0; k < Sol.networks.size(); ++k)
			for (size_t p = 0; p < Sol.networks[k].panels.size(); ++p) S += FString::Printf(TEXT("%d\n"), static_cast<int32>(k));
		S += TEXT("VECTORS velocity_upper double\n");
		for (const NetworkResult& NR : Sol.networks)
			for (const PanelResult& P : NR.panels) S += FString::Printf(TEXT("%.9g %.9g %.9g\n"), P.v_upper.x, P.v_upper.y, P.v_upper.z);
		if (!FFileHelper::SaveStringToFile(S, *Path))
		{
			OutError = TEXT("Could not write ") + Path;
			return false;
		}
		return true;
	}

	// ---- the cases ----

	// One entity's share of the network list, so results can be attributed
	// to it: its own networks first, then (without the symmetry plane) the
	// reflected copies of a mirrored entity.
	struct FShare
	{
		FString Name;
		int32 First = 0, Count = 0, OwnCount = 0;
		bool bWing = false;
		// The wing's span axis in the world: its local +y, as placed.
		FVector SpanAxis = FVector(0, 1, 0);
		// Solved with its image about y = 0.
		bool bImaged = false;
	};

	// One operating condition: the networks (the wake sheds along this
	// free stream, so they are built per case), the solution and what is
	// read from it. Solved on a worker thread; everything else is set on
	// the game thread.
	struct FCase
	{
		double AlphaDeg = 0.0, BetaDeg = 0.0;
		aerotelic::Vec3 Stream, LiftDir, SideDir;
		bool bThick = true;
		std::vector<Network> Nets;
		TArray<FShare> Shares;
		TArray<FString> Skipped;
		Solution Sol;
		bool bSolved = false;
		// Wind-axis coefficients: lift, drag, side force; roll about the
		// stream, pitch about the side axis, yaw about the lift axis, on the
		// reference length.
		double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
		double CpMin = 1.0, CpMax = -1e9;
		int32 NBody = 0, NWake = 0;
	};

	struct FJob
	{
		TArray<TSharedPtr<FCase>> Cases;
		aerotelic::panel::Options Options;
		std::atomic<bool> bCancel{ false };
		FString Error;
	};

	// The pressure coefficient a network's panels show on the surface
	// picture: the outer side of a closed body or a thick wing, both sides
	// of a mean camber sheet.
	bool IsSheet(const FCase& C, int32 NetIndex)
	{
		if (C.bThick || C.Sol.networks[NetIndex].type != NetworkType::DoubletAnalysis) return false;
		for (const FShare& Sh : C.Shares)
			if (Sh.bWing && NetIndex >= Sh.First && NetIndex < Sh.First + Sh.Count) return true;
		return false;
	}

	// The coloured mesh of a case: every non-wake network as its corner grid
	// with the pressure averaged onto the corners (so the colour is smooth
	// across the panels), both windings of every quad (the engine's two-
	// sided convention, as the bodies are drawn), and a sheet as two
	// copies a hair apart, the upper side coloured by Cp_upper and the
	// lower by Cp_lower.
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
		FBox Bounds(ForceInit);
		for (const Network& N : C.Nets)
			for (const aerotelic::Vec3& P : N.corners) Bounds += FVector(P.x, P.y, P.z);
		const double Gap = Bounds.IsValid ? Bounds.GetSize().GetMax() * 5e-4 : 1e-3;

		for (size_t k = 0; k < C.Sol.networks.size(); ++k)
		{
			const NetworkResult& NR = C.Sol.networks[k];
			const Network& Net = C.Nets[k];
			if (NR.type == NetworkType::DoubletWake || NR.type == NetworkType::DoubletWakeConstant) continue;
			const int32 Rows = NR.panel_rows, Cols = NR.panel_cols;
			if (Rows <= 0 || Cols <= 0 || Net.rows != Rows + 1 || Net.cols != Cols + 1) continue;
			const bool bSheet = IsSheet(C, static_cast<int32>(k));
			const int32 Sides = bSheet ? 2 : 1;
			for (int32 Side = 0; Side < Sides; ++Side)
			{
				const int32 Base = M.Vertices.Num();
				for (int32 I = 0; I <= Rows; ++I)
				{
					for (int32 J = 0; J <= Cols; ++J)
					{
						double CpSum = 0.0;
						FVector NSum = FVector::ZeroVector;
						int32 Count = 0;
						for (int32 i = I - 1; i <= I; ++i)
						{
							for (int32 j = J - 1; j <= J; ++j)
							{
								if (i < 0 || j < 0 || i >= Rows || j >= Cols) continue;
								const PanelResult& P = NR.panels[static_cast<size_t>(i) * Cols + j];
								CpSum += (bSheet && Side == 1) ? P.cp_lower : P.cp_upper;
								NSum += FVector(P.normal.x, P.normal.y, P.normal.z);
								++Count;
							}
						}
						const aerotelic::Vec3 Q = Net.Corner(I + 1, J + 1);
						FVector N = NSum.GetSafeNormal();
						FVector V(Q.x, Q.y, Q.z);
						if (bSheet)
						{
							V += N * (Side == 0 ? Gap : -Gap);
							if (Side == 1) N = -N;
						}
						M.Vertices.Add(V);
						M.Normals.Add(N);
						M.Cp.Add(Count > 0 ? CpSum / Count : 0.0);
					}
				}
				auto Idx = [&](int32 I, int32 J) { return Base + I * (Cols + 1) + J; };
				for (int32 i = 0; i < Rows; ++i)
				{
					for (int32 j = 0; j < Cols; ++j)
					{
						const int32 A = Idx(i, j), B = Idx(i, j + 1), Cc = Idx(i + 1, j + 1), D = Idx(i + 1, j);
						M.Triangles.Append({ A, B, Cc, A, Cc, D, A, Cc, B, A, D, Cc });
					}
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

	// The reflection of a network result in y = 0, for the strips of the
	// other half of an imaged wing: the same panels in the same order
	// (NOT column-reversed, so they still pair with the reflected corners),
	// centres and normals reflected.
	NetworkResult ReflectedResult(const NetworkResult& R)
	{
		NetworkResult M = R;
		for (PanelResult& P : M.panels)
		{
			P.centre.y = -P.centre.y;
			P.normal.y = -P.normal.y;
		}
		return M;
	}

	Network ReflectedNetwork(const Network& N)
	{
		Network M = N;
		for (aerotelic::Vec3& P : M.corners) P.y = -P.y;
		return M;
	}

	// ---- the dialog's state: owned by its widgets (the window is modeless) ----

	struct FState : public TSharedFromThis<FState>
	{
		TFunction<void(const FText&)> Status;
		TWeakPtr<SWindow> Window;
		aerotelic::UnitSystem Units;

		// Inputs (tab 1).
		TSharedPtr<SAerotelicBodyPicker> Picker;
		TSharedPtr<SCheckBox> SymmetryCheck;
		bool bSymmetryTouched = false;
		TSharedRef<TArray<TSharedPtr<FString>>> Formulations = MakeShared<TArray<TSharedPtr<FString>>>();
		TSharedPtr<FString> Formulation;
		double Chordwise = 12.0, Spanwise = 10.0, WakeChords = 20.0;
		double Along = 16.0, Around = 12.0;
		double ReferenceArea = 1.0, ReferenceLength = 1.0;
		TSharedPtr<SEditableTextBox> AlphaBox, BetaBox, PathBox;
		TSharedPtr<STextBlock> LblWarn, LblSummary, LblInput;
		TSharedPtr<SButton> SolveButton;

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
		FCpMesh Mesh; // of the current case
		TSharedRef<TArray<TSharedPtr<FString>>> MapNames = MakeShared<TArray<TSharedPtr<FString>>>();

		// Span loading (tab 3).
		TSharedPtr<SAerotelicLinePlot> LoadPlot;
		int32 CurrentWing = 0;

		// Forces and moments (tab 4).
		TArray<TSharedPtr<SAerotelicLinePlot>> Polars;
		TSharedPtr<SWidgetSwitcher> PolarSwitch;
		TSharedPtr<SVerticalBox> PolarTable;

		TSharedPtr<SAerotelicTabStrip> Tabs;
		// -AerotelicPanelSmoke: a developer run that solves at once, steps
		// through the result tabs saving a screenshot of each to
		// Saved/Screenshots, then quits -- the only way the widget tree is
		// exercised outside a hand-driven session (see README, "Known limits").
		bool bSmoke = false;

		TArray<AAerotelicEntityActor*> Chosen() const
		{
			TArray<AAerotelicEntityActor*> Out;
			if (!Picker.IsValid()) return Out;
			for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen())
				if (C->Operand.Entity.IsValid()) Out.Add(C->Operand.Entity.Get());
			return Out;
		}

		bool IsThick() const { return Formulation.IsValid() && Formulations->Num() > 0 && *Formulation == *(*Formulations)[0]; }

		const FCase* CurrentCase() const { return Cases.IsValidIndex(Current) ? Cases[Current].Get() : nullptr; }

		// The wings among the current case's shares, for the span loading.
		TArray<int32> WingShares() const
		{
			TArray<int32> Out;
			if (const FCase* C = CurrentCase())
				for (int32 i = 0; i < C->Shares.Num(); ++i)
					if (C->Shares[i].bWing) Out.Add(i);
			return Out;
		}

		FText CaseLabel() const
		{
			const FCase* C = CurrentCase();
			if (C == nullptr) return LOCTEXT("NoCase", "no solution yet");
			return FText::FromString(FString::Printf(TEXT("α = %g°, β = %g°  (%d of %d)"), C->AlphaDeg, C->BetaDeg, Current + 1, Cases.Num()));
		}

		void DefaultReferences()
		{
			// The planform area of the chosen wings (both halves each) and the
			// first wing's root chord; with no wing, the first body's frontal
			// ellipse and its length.
			double Area = 0.0, Length = 0.0;
			FBox FirstBody(ForceInit);
			for (AAerotelicEntityActor* E : Chosen())
			{
				if (E->GetEntityType() == EntityType::Wing)
				{
					try
					{
						const aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
						Area += W->planform().PlanformArea();
						if (Length <= 0.0) Length = W->planform().RootChord();
					}
					catch (const std::exception&) {}
				}
				else if (!FirstBody.IsValid)
				{
					FirstBody = E->GetPlacedBounds(false);
				}
			}
			if (Area <= 0.0 && FirstBody.IsValid)
			{
				const FVector Size = FirstBody.GetSize();
				Area = kPi / 4.0 * Size.Y * Size.Z;
				Length = Size.X;
			}
			ReferenceArea = Area > 0.0 ? Area : 1.0;
			ReferenceLength = Length > 0.0 ? Length : 1.0;
		}

		bool DefaultSymmetry() const
		{
			// On when anything chosen is mirrored, or a wing has its root on y = 0.
			for (AAerotelicEntityActor* E : Chosen())
			{
				if (E->IsMirrored()) return true;
				if (E->GetEntityType() == EntityType::Wing)
				{
					const FBox B = E->GetPlacedBounds(false);
					if (B.IsValid && FMath::Abs(B.Min.Y) < 1e-6 * FMath::Max(B.GetSize().GetMax(), 1.0)) return true;
				}
			}
			return false;
		}

		void Refresh()
		{
			TArray<AAerotelicEntityActor*> Es = Chosen();
			DefaultReferences();
			if (!bSymmetryTouched && SymmetryCheck.IsValid()) SymmetryCheck->SetIsChecked(DefaultSymmetry() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked);
			// A panel method wants bodies that do not penetrate each other; a
			// wing root buried in a fuselage is the usual case and it still
			// solves, but the panels inside the other body are not a surface the
			// flow sees. Say so when the bounds overlap.
			FString Warn;
			for (int32 a = 0; a < Es.Num(); ++a)
			{
				for (int32 b = a + 1; b < Es.Num(); ++b)
				{
					const FBox A = Es[a]->GetPlacedBounds(false), B = Es[b]->GetPlacedBounds(false);
					if (A.IsValid && B.IsValid && A.Intersect(B))
						Warn += FString::Printf(TEXT("%s%s and %s"), Warn.IsEmpty() ? TEXT("") : TEXT("; "), *Es[a]->EntityName, *Es[b]->EntityName);
				}
			}
			if (LblWarn.IsValid())
			{
				LblWarn->SetText(Warn.IsEmpty() ? FText::GetEmpty()
					: FText::FromString(TEXT("Bounds overlap: ") + Warn + TEXT(". Bodies that penetrate each other solve, but the panels of one inside the other are not a wetted surface: pressures near the junction are not to be trusted.")));
				LblWarn->SetVisibility(Warn.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible);
			}
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

		// Panels the chosen bodies for one free stream: the networks and the
		// entities' shares of them. Throws on a model that will not build.
		TSharedPtr<FCase> BuildCase(double AlphaDeg, double BetaDeg, bool bSymmetry)
		{
			TSharedPtr<FCase> C = MakeShared<FCase>();
			C->AlphaDeg = AlphaDeg;
			C->BetaDeg = BetaDeg;
			C->bThick = IsThick();
			const double Alpha = AlphaDeg * kPi / 180.0, Beta = BetaDeg * kPi / 180.0;
			// The flow along +x, tilted towards +z by alpha and towards +y by
			// beta (unit, world frame). Wind axes: lift normal to the stream in
			// the plane of symmetry, side force along the third axis.
			C->Stream = { std::cos(Alpha) * std::cos(Beta), std::sin(Beta), std::sin(Alpha) * std::cos(Beta) };
			C->LiftDir = { -std::sin(Alpha), 0.0, std::cos(Alpha) };
			C->SideDir = aerotelic::Cross(C->LiftDir, C->Stream);
			const aerotelic::Vec3 Stream = C->Stream;

			for (AAerotelicEntityActor* E : Chosen())
			{
				const FTransform Transform = E->GetActorTransform();
				const FVector DirModel = Transform.InverseTransformVectorNoScale(FVector(Stream.x, Stream.y, Stream.z));
				const aerotelic::Vec3 StreamModel{ DirModel.X, DirModel.Y, DirModel.Z };
				FShare Share;
				Share.Name = E->EntityName;
				Share.First = static_cast<int32>(C->Nets.size());
				Share.bImaged = bSymmetry;
				std::vector<Network> Own;
				if (E->GetEntityType() == EntityType::Wing)
				{
					Share.bWing = true;
					Share.SpanAxis = Transform.TransformVectorNoScale(FVector(0, 1, 0)).GetSafeNormal();
					aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
					aerotelic::panel::WingPaneling Pan;
					Pan.chordwise = FMath::Clamp(FMath::RoundToInt(Chordwise), 2, 200);
					Pan.spanwise = FMath::Clamp(FMath::RoundToInt(Spanwise), 1, 200);
					Pan.thick = C->bThick;
					Pan.wake_length = WakeChords * W->planform().RootChord();
					Pan.wake_direction = StreamModel; // the wake sheds along the world stream, in the wing's frame
					Own = aerotelic::panel::WingNetworks(*W, Pan, StreamModel);
					for (Network& N : Own)
					{
						N.name = TCHAR_TO_UTF8(*E->EntityName) + std::string(N.name == W->name ? "" : " wake");
						PlaceNetwork(N, Transform);
						if (N.prescribed_source) N.prescribed_source = [Stream](const aerotelic::Vec3&, const aerotelic::Vec3& Nrm) { return -aerotelic::Dot(Stream, Nrm); };
					}
				}
				else
				{
					// The lofts placed into the world first, so a body on the
					// centreline is cut at y = 0 when the plane is on.
					std::vector<aerotelic::NamedLoft> Lofts = E->GetModel()->BuildLofts();
					if (Lofts.empty())
					{
						C->Skipped.Add(E->EntityName);
						continue;
					}
					aerotelic::panel::BodyPaneling BP;
					BP.along = FMath::Clamp(FMath::RoundToInt(Along), 1, 400);
					BP.around = FMath::Clamp(FMath::RoundToInt(Around), 3, 400);
					if (bSymmetry && !E->IsMirrored()) BP.half_plane_y = 0.0;
					for (const aerotelic::NamedLoft& L : Lofts)
					{
						const FString Name = Lofts.size() == 1 ? E->EntityName : E->EntityName + TEXT("/") + UTF8_TO_TCHAR(L.name.c_str());
						Own.push_back(aerotelic::panel::LoftNetwork(PlacedLoft(L, Transform), TCHAR_TO_UTF8(*Name), BP, Stream));
					}
				}
				Share.OwnCount = static_cast<int32>(Own.size());
				// A mirrored entity: the plane's image stands in for the other
				// half; without the plane, the reflected copy is paneled too.
				if (E->IsMirrored() && !bSymmetry)
				{
					const size_t N0 = Own.size();
					for (size_t k = 0; k < N0; ++k) Own.push_back(MirroredNetwork(Own[k]));
				}
				for (Network& N : Own) C->Nets.push_back(std::move(N));
				Share.Count = static_cast<int32>(C->Nets.size()) - Share.First;
				C->Shares.Add(Share);
			}
			if (C->Nets.empty()) throw std::runtime_error("none of the chosen bodies has a loft to panel");
			return C;
		}

		// What is read off a solved case.
		void Derive(FCase& C, double RefLength) const
		{
			const aerotelic::Vec3& F = C.Sol.force_coefficient;
			const aerotelic::Vec3& M = C.Sol.moment_coefficient;
			const double Chord = RefLength > 0.0 ? RefLength : 1.0;
			C.CL = aerotelic::Dot(F, C.LiftDir);
			C.CD = aerotelic::Dot(F, C.Stream);
			C.CY = aerotelic::Dot(F, C.SideDir);
			C.Cl = aerotelic::Dot(M, C.Stream) / Chord;
			C.Cm = aerotelic::Dot(M, C.SideDir) / Chord;
			C.Cn = aerotelic::Dot(M, C.LiftDir) / Chord;
			C.CpMin = 1.0;
			C.CpMax = -1e9;
			C.NBody = 0;
			C.NWake = 0;
			for (size_t k = 0; k < C.Sol.networks.size(); ++k)
			{
				const NetworkResult& NR = C.Sol.networks[k];
				const bool bWake = NR.type == NetworkType::DoubletWake || NR.type == NetworkType::DoubletWakeConstant;
				if (bWake) { C.NWake += static_cast<int32>(NR.panels.size()); continue; }
				const bool bSheet = IsSheet(C, static_cast<int32>(k));
				for (const PanelResult& P : NR.panels)
				{
					C.CpMin = FMath::Min(C.CpMin, bSheet ? FMath::Min(P.cp_upper, P.cp_lower) : P.cp_upper);
					C.CpMax = FMath::Max(C.CpMax, bSheet ? FMath::Max(P.cp_upper, P.cp_lower) : P.cp_upper);
					++C.NBody;
				}
			}
		}

		void Solve()
		{
			if (Job.IsValid())
			{
				AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("Busy", "A solve is still running."), LOCTEXT("Title", "High Order Panel"));
				return;
			}
			TArray<AAerotelicEntityActor*> Es = Chosen();
			if (Es.Num() == 0)
			{
				AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("PickOneWarn", "Choose at least one body."), LOCTEXT("Title", "High Order Panel"));
				return;
			}
			// The operating conditions: a list for the angle of attack OR for
			// the sideslip, never both (a list of each would be a grid of
			// cases, not a sweep), and sideslip held at zero for now (the
			// symmetry plane and the mirrored loads assume a stream in the
			// plane of symmetry).
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
			for (double B : Betas)
			{
				if (B != 0.0)
				{
					SetInputError(TEXT("Sideslip is not supported yet: the symmetry plane and the mirrored loads assume a free stream in the plane of symmetry. Enter 0."));
					return;
				}
			}
			SetInputError(FString());
			bSweepAlpha = Betas.Num() <= 1;
			const bool bSymmetry = SymmetryCheck->IsChecked();

			Status(LOCTEXT("Paneling", "Paneling the bodies..."));
			TSharedPtr<FJob> NewJob = MakeShared<FJob>();
			try
			{
				if (bSweepAlpha)
				{
					for (double A : Alphas) NewJob->Cases.Add(BuildCase(A, Betas[0], bSymmetry));
				}
				else
				{
					for (double B : Betas) NewJob->Cases.Add(BuildCase(Alphas[0], B, bSymmetry));
				}
			}
			catch (const std::exception& E)
			{
				Status(LOCTEXT("Ready", "Ready"));
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(FString(TEXT("The analysis failed:\n")) + UTF8_TO_TCHAR(E.what())), LOCTEXT("Failed", "High Order Panel Failed"));
				return;
			}
			if (bSymmetry) NewJob->Options.symmetry_y = 0.0;
			NewJob->Options.reference_area = ReferenceArea > 0.0 ? ReferenceArea : 1.0;

			Job = NewJob;
			if (SolveButton.IsValid()) SolveButton->SetEnabled(false);
			Status(FText::FromString(FString::Printf(TEXT("Solving 1 of %d..."), NewJob->Cases.Num())));

			// The solves on a worker thread, one case after another, each
			// reported as it lands; the dialog stays live meanwhile. The job
			// owns the cases until it is done; a closed dialog cancels it
			// between cases and the results are dropped.
			// -AerotelicPanelSync solves on the game thread instead (a
			// developer aid: the Editor host, or a comparison).
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
						C->Sol = aerotelic::panel::Solve(C->Nets, C->Stream, NewJob->Options);
						C->bSolved = true;
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
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("The analysis failed:\n") + Done->Error), LOCTEXT("Failed", "High Order Panel Failed"));
				return;
			}
			if (Done->bCancel.load()) return;

			double Seconds = 0.0, MaxDelta = 0.0;
			double Lo = 1e9, Hi = -1e9;
			for (TSharedPtr<FCase>& C : Done->Cases)
			{
				Derive(*C, ReferenceLength);
				Seconds += C->Sol.seconds_influence + C->Sol.seconds_solve;
				MaxDelta = FMath::Max(MaxDelta, C->Sol.max_delta);
				Lo = FMath::Min(Lo, C->CpMin);
				Hi = FMath::Max(Hi, C->CpMax);
				UE_LOG(LogTemp, Log, TEXT("High order panel: alpha %g beta %g: C_L %.4f (wake %.4f), C_D %.4f, C_m %.4f, Cp %.3f..%.3f, %d unknowns, pivot ratio %.2e, max delta %.3f, %.2f s"),
					C->AlphaDeg, C->BetaDeg, C->CL, C->Sol.lift_from_wake.value_or(0.0), C->CD, C->Cm, C->CpMin, C->CpMax, C->Sol.unknowns, C->Sol.pivot_ratio, C->Sol.max_delta, C->Sol.seconds_influence + C->Sol.seconds_solve);
			}
			Cases = Done->Cases;
			Current = 0;
			CurrentWing = 0;
			if (bRangeAuto && Lo < Hi)
			{
				CpLo = Lo;
				CpHi = Hi;
			}

			const FCase& First = *Cases[0];
			FString Summary = FString::Printf(TEXT("%d solution%s: %d body panels + %d wake, %d unknowns%s; panel curvature %.3f%s; %.2f s in all."),
				Cases.Num(), Cases.Num() == 1 ? TEXT("") : TEXT("s"), First.NBody, First.NWake, First.Sol.unknowns,
				Done->Options.symmetry_y ? TEXT(", with the image about y = 0") : TEXT(", no symmetry plane"),
				MaxDelta, MaxDelta > 0.066 ? TEXT(" (the method wants under 0.066: refine the paneling)") : TEXT(""), Seconds);
			for (const FString& Sk : First.Skipped) Summary += FString::Printf(TEXT("\n%s: skipped (no loft to panel)"), *Sk);

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
					if (WritePanelsVtk(CasePath, C->Nets, C->Sol, Units.LengthToSI(), WErr)) ++NWritten;
					else AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(WErr), LOCTEXT("Title", "High Order Panel"));
				}
				if (NWritten > 0) Written = FString::Printf(TEXT(" -> %d file%s beside %s"), NWritten, NWritten == 1 ? TEXT("") : TEXT("s"), *FPaths::GetCleanFilename(Path));
				Summary += TEXT("\nSolution files written: ") + FString::FromInt(NWritten) + TEXT(".");
			}
			if (LblSummary.IsValid()) LblSummary->SetText(FText::FromString(Summary));

			UpdateSurface(true);
			UpdateLoads();
			UpdatePolars();
			Status(FText::FromString(FString::Printf(TEXT("High order panel: %d solution%s, C_L %.4f to %.4f, Cp %.2f to %.2f, %d unknowns in %.2f s%s"),
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
				UE_LOG(LogTemp, Warning, TEXT("HighOrderPanel smoke: screenshot %s failed"), *Name);
				return;
			}
			for (FColor& C : Pixels) C.A = 255;
			TArray64<uint8> Png;
			FImageUtils::PNGCompressImageArray(Size.X, Size.Y, Pixels, Png);
			const FString Path = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Screenshots"), Name);
			FFileHelper::SaveArrayToFile(Png, *Path);
			UE_LOG(LogTemp, Log, TEXT("HighOrderPanel smoke: wrote %s (%d x %d)"), *Path, Size.X, Size.Y);
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
				// Half-way through the wait, swing the view: a picture that
				// still shows the first viewpoint means the capture never
				// repeated.
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
					UE_LOG(LogTemp, Log, TEXT("HighOrderPanel smoke: surface material %s, base %s, %d vertices; shader compiler %s, %d jobs remaining; mesh at %s"),
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
						UE_LOG(LogTemp, Log, TEXT("HighOrderPanel smoke: done, quitting"));
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
					// Tell the result apart from the scene's own bodies: the
					// view's backdrop blue, the result mesh lifted 2 units and
					// shown in the main view as well (a viewport screenshot
					// follows, to compare the two renders).
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
					UE_LOG(LogTemp, Log, TEXT("HighOrderPanel smoke: Cp mesh %d vertices, Cp %.3f .. %.3f mapped on %.3f .. %.3f with %s; colour[0] %s, colour[mid] %s"),
						Mesh.Cp.Num(), Mesh.Cp[0], Mesh.Cp[Mesh.Cp.Num() / 2], CpLo, CpHi, UTF8_TO_TCHAR(Map->name.c_str()), *Colors[0].ToString(), *Colors[Colors.Num() / 2].ToString());
				}
				MeshActor->SetMesh(Mesh.Vertices, Mesh.Triangles, Mesh.Normals, Colors);
				if (!bFramed)
				{
					View->FrameShown();
					bFramed = true;
				}
			}
			else
			{
				MeshActor->SetColors(ColorsFor(Mesh, *Map, CpLo, CpHi));
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
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0).HAlign(HAlign_Left)
						[ Button(LOCTEXT("AutoRange", "Auto range (all solutions)"), [Self]() { Self->AutoRange(); }) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ MakeInfoLabel(LOCTEXT("PopupHint", "The range defaults to the pressures of every solution together, so the colours mean the same from one to the next. Click outside to close.")) ]
					]
				];
			FSlateApplication::Get().PushMenu(ColorBar.ToSharedRef(), FWidgetPath(), Content, Event.GetScreenSpacePosition(), FPopupTransitionEffect(FPopupTransitionEffect::ContextMenu));
		}

		// ---- tab 3: the span loading ----

		void UpdateLoads()
		{
			if (!LoadPlot.IsValid()) return;
			const FCase* C = CurrentCase();
			TArray<int32> Wings = WingShares();
			if (C == nullptr || Wings.Num() == 0)
			{
				LoadPlot->SetSeries({});
				LoadPlot->SetMessage(C == nullptr ? LOCTEXT("NoSolution", "Solve first (Set-up tab).") : LOCTEXT("NoWing", "No wing among the bodies: the span loading is a wing's."));
				LoadPlot->SetTitle(LOCTEXT("LoadTitle", "Span loading"));
				return;
			}
			CurrentWing = FMath::Clamp(CurrentWing, 0, Wings.Num() - 1);
			const FShare& Sh = C->Shares[Wings[CurrentWing]];
			const aerotelic::Vec3 Axis{ Sh.SpanAxis.X, Sh.SpanAxis.Y, Sh.SpanAxis.Z };
			// The load per unit span normal to both the stream and the span:
			// lift for a wing, side force for a fin.
			const aerotelic::Vec3 LoadDir = aerotelic::Normalized(aerotelic::Cross(C->Stream, Axis));
			const double Chord = ReferenceLength > 0.0 ? ReferenceLength : 1.0;

			aerotelic::panel::StripOptions SO;
			SO.span_axis = Axis;
			SO.strips_are_columns = C->bThick;
			SO.two_sided = !C->bThick;
			std::vector<aerotelic::panel::StripLoad> Strips;
			try
			{
				for (int32 k = Sh.First; k < Sh.First + Sh.Count; ++k)
				{
					const NetworkResult& NR = C->Sol.networks[k];
					if (NR.type != NetworkType::DoubletAnalysis) continue; // the wakes
					// The entity's own network, and (without the plane) its
					// reflected copy: each pairs with its own corners, so the
					// copy's reversed columns are no concern.
					std::vector<aerotelic::panel::StripLoad> S = aerotelic::panel::StripLoads(C->Nets[k], NR, SO);
					Strips.insert(Strips.end(), S.begin(), S.end());
					if (Sh.bImaged && k < Sh.First + Sh.OwnCount)
					{
						// The image about y = 0: the other half of the loading.
						std::vector<aerotelic::panel::StripLoad> Im = aerotelic::panel::StripLoads(ReflectedNetwork(C->Nets[k]), ReflectedResult(NR), SO);
						Strips.insert(Strips.end(), Im.begin(), Im.end());
					}
				}
			}
			catch (const std::exception& E)
			{
				LoadPlot->SetSeries({});
				LoadPlot->SetMessage(FText::FromString(UTF8_TO_TCHAR(E.what())));
				return;
			}
			if (Strips.empty())
			{
				LoadPlot->SetSeries({});
				LoadPlot->SetMessage(LOCTEXT("NoStrips", "No lifting network for this wing."));
				return;
			}
			std::sort(Strips.begin(), Strips.end(), [](const aerotelic::panel::StripLoad& a, const aerotelic::panel::StripLoad& b) { return a.s < b.s; });
			// The total: the strip forces summed (an area, force / q), as a
			// coefficient on the reference area, and as the integral of the
			// plotted load (on the reference length) over the span.
			double SMin = 1e300, SMax = -1e300, TotalForce = 0.0;
			for (const aerotelic::panel::StripLoad& St : Strips)
			{
				SMin = FMath::Min(SMin, St.s_min);
				SMax = FMath::Max(SMax, St.s_max);
				TotalForce += aerotelic::Dot(St.force, LoadDir);
			}
			const double Total = TotalForce / Chord;
			const double TotalCoefficient = TotalForce / (ReferenceArea > 0.0 ? ReferenceArea : 1.0);
			const double S0 = 0.5 * (SMin + SMax), Half = FMath::Max(0.5 * (SMax - SMin), 1e-9);

			FAerotelicPlotSeries Computed;
			Computed.Label = TEXT("Panel method (pressures)");
			Computed.Color = FLinearColor(FColor::FromHex(TEXT("#d56b3a")));
			for (const aerotelic::panel::StripLoad& St : Strips)
			{
				if (St.width <= 0.0) continue;
				Computed.Points.Add(FVector2D((St.s - S0) / Half, aerotelic::Dot(St.force, LoadDir) / St.width / Chord));
			}
			FAerotelicPlotSeries Elliptic;
			Elliptic.Label = TEXT("Elliptic, same total");
			Elliptic.Color = FLinearColor(FColor::FromHex(TEXT("#3a7bd5")));
			Elliptic.bMarkers = false;
			Elliptic.bDashed = true;
			for (int32 i = 0; i <= 100; ++i)
			{
				const double S = S0 - Half + (2.0 * Half) * i / 100.0;
				Elliptic.Points.Add(FVector2D((S - S0) / Half, aerotelic::panel::EllipticLoad(Total, S0, Half, S)));
			}
			LoadPlot->SetSeries({ Computed, Elliptic });
			LoadPlot->SetTitle(FText::FromString(FString::Printf(TEXT("%s: span loading, α = %g°  (integrated %s = %.4f over %d strips)"), *Sh.Name, C->AlphaDeg, FMath::Abs(Sh.SpanAxis.Z) > 0.9 ? TEXT("C_Y") : TEXT("C_L"), TotalCoefficient, static_cast<int32>(Strips.size()))));
			LoadPlot->SetAxisLabels(LOCTEXT("SpanAxis", "span fraction 2s / b"), LOCTEXT("LoadAxis", "cl · c / c_ref  (load per unit span on the reference length)"));
		}

		// ---- tab 4: forces and moments against the swept angle ----

		void UpdatePolars()
		{
			if (Polars.Num() < 6 || !PolarSwitch.IsValid() || !PolarTable.IsValid()) return;
			struct FCoef { const TCHAR* Name; double FCase::*Member; FLinearColor Color; };
			const FCoef Coefs[6] = {
				{ TEXT("C_L  lift"), &FCase::CL, FLinearColor(FColor::FromHex(TEXT("#d56b3a"))) },
				{ TEXT("C_D  induced drag"), &FCase::CD, FLinearColor(FColor::FromHex(TEXT("#c8d53a"))) },
				{ TEXT("C_Y  side force"), &FCase::CY, FLinearColor(FColor::FromHex(TEXT("#30f05a"))) },
				{ TEXT("C_l  roll (about the stream)"), &FCase::Cl, FLinearColor(FColor::FromHex(TEXT("#3a7bd5"))) },
				{ TEXT("C_m  pitch"), &FCase::Cm, FLinearColor(FColor::FromHex(TEXT("#b07bd5"))) },
				{ TEXT("C_n  yaw (about the lift)"), &FCase::Cn, FLinearColor(FColor::FromHex(TEXT("#5ad5c8"))) },
			};
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
				// One operating point: a table says it better than six dots.
				PolarTable->ClearChildren();
				const FCase& C = *Cases[0];
				PolarTable->AddSlot().AutoHeight().Padding(0, 0, 0, 8)
				[
					SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("One solution, α = %g°, β = %g°. Wind axes: lift normal to the stream in the plane of symmetry, drag along it, side force along the third axis; moments about the origin on the reference length %.4g %s, roll about the stream, pitch about the side axis, yaw about the lift axis."), C.AlphaDeg, C.BetaDeg, ReferenceLength, UTF8_TO_TCHAR(Units.LengthName())))).AutoWrapText(true)
				];
				for (int32 i = 0; i < 6; ++i)
				{
					PolarTable->AddSlot().AutoHeight().Padding(0, 2)
					[ FormRow(FText::FromString(Coefs[i].Name), SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("%.5f"), C.*(Coefs[i].Member)))), 220.0f) ];
				}
				if (C.Sol.lift_from_wake)
				{
					PolarTable->AddSlot().AutoHeight().Padding(0, 8, 0, 0)
					[ MakeInfoLabel(FText::FromString(FString::Printf(TEXT("C_L from the wake doublet strength (the exact circulation of the solution): %.5f. The pressure-integrated value above leans on the panel splines and reads a little high when the paneling is sparse."), *C.Sol.lift_from_wake))) ];
				}
				// Per body.
				FString Lines;
				for (const FShare& Sh : C.Shares)
				{
					aerotelic::Vec3 F{ 0, 0, 0 };
					for (int32 k = Sh.First; k < Sh.First + Sh.Count; ++k) F += C.Sol.networks[k].force_coefficient;
					Lines += FString::Printf(TEXT("%s%s: C_L %.4f, C_D %.4f, C_Y %.4f"), Lines.IsEmpty() ? TEXT("") : TEXT("\n"), *Sh.Name, aerotelic::Dot(F, C.LiftDir), aerotelic::Dot(F, C.Stream), aerotelic::Dot(F, C.SideDir));
				}
				PolarTable->AddSlot().AutoHeight().Padding(0, 8, 0, 0) [ FormRow(LOCTEXT("ByBody", "By body:"), SNew(STextBlock).Text(FText::FromString(Lines)).AutoWrapText(true), 220.0f) ];
				PolarSwitch->SetActiveWidgetIndex(1);
				return;
			}
			for (int32 i = 0; i < 6; ++i)
			{
				FAerotelicPlotSeries S;
				S.Label = Coefs[i].Name;
				S.Color = Coefs[i].Color;
				for (const TSharedPtr<FCase>& C : Cases)
					S.Points.Add(FVector2D(bSweepAlpha ? C->AlphaDeg : C->BetaDeg, (*C).*(Coefs[i].Member)));
				S.Points.Sort([](const FVector2D& A, const FVector2D& B) { return A.X < B.X; });
				Polars[i]->SetSeries({ S });
				Polars[i]->SetTitle(FText::FromString(Coefs[i].Name));
				Polars[i]->SetAxisLabels(XLabel, FText::GetEmpty());
			}
			PolarSwitch->SetActiveWidgetIndex(0);
		}

		// The window is going: drop the results and the widgets so the
		// widget <-> state references no longer hold each other, cancel a
		// running job and remove the result actor from the world.
		void Close()
		{
			if (Job.IsValid())
			{
				Job->bCancel = true;
				Job.Reset();
			}
			if (MeshActor.IsValid()) MeshActor->Destroy();
			MeshActor.Reset();
			Cases.Empty();
			Mesh = FCpMesh();
			Picker.Reset(); SymmetryCheck.Reset(); AlphaBox.Reset(); BetaBox.Reset(); PathBox.Reset();
			LblWarn.Reset(); LblSummary.Reset(); LblInput.Reset(); SolveButton.Reset();
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
		"HIGH ORDER PANEL METHOD\n\n"
		"The incompressible potential flow about the lofted bodies of the scene, by the higher-order panel method of Johnson (NASA CR-3079): curved panels carrying linear sources and quadratic doublets, the Kutta condition at the trailing edge and a wake shed along the free stream. "
		"A potential flow is similar at every speed and density, so nothing dimensional is asked for: Cp = 1 - (V/V∞)² and the coefficients are C_F = -∮ Cp n dA / S_ref.\n\n"
		"SET-UP\n"
		"Choose the bodies, the paneling, the angles and the references, then Solve. The angle of attack and the angle of sideslip each take one value or a comma separated list, but only one of them may be a list: the solutions are a sweep of that angle with the other fixed. "
		"Sideslip is held at zero for now. The solves run in the background, one per angle; the dialog stays usable.\n\n"
		"SURFACE PRESSURE\n"
		"The bodies coloured by the pressure coefficient on their outer side (a mean camber surface by its upper and lower sides). Left-drag orbits, right- or middle-drag pans, the wheel zooms, Home frames the bodies. "
		"Click the colour bar to set the minimum, the maximum and the colour map; the range defaults to the pressures of every solution together. The arrows step between solutions.\n\n"
		"SPAN LOADING\n"
		"The lift per unit span of a wing along its span -- the pressure integrated over each strip of panels, resolved normal to the stream and to the span axis (the wing's own, so a fin shows its side load), on the reference length: cl · c / c_ref against the span fraction. "
		"The dashed curve is the elliptic distribution with the same integral, the loading of least induced drag for that lift. With the symmetry plane on, the image half is drawn as well. The arrows step between solutions and between wings.\n\n"
		"FORCES AND MOMENTS\n"
		"The wind-axis coefficients against the swept angle: lift, induced drag (a potential flow has no friction) and side force; roll about the stream, pitch about the side axis and yaw about the lift axis, about the origin on the reference length. "
		"With a single solution the values are tabulated instead, with the lift from the wake doublet strength and the force by body.");
}

void ShowAerotelicHighOrderPanelDialog(TFunction<void(const FText&)> Status)
{
	TArray<FAerotelicBodyChoicePtr> Choices = PanelChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoBody", "No lofted body in the scene. The high order panel method runs on wings, fuselages, nacelles and primitives (boolean results and lattices are meshes with no sections to panel)."),
			LOCTEXT("Title", "High Order Panel"));
		return;
	}

	const FVector2D Size(820, 940);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "High Order Panel"))
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
	S->Formulations->Add(MakeShared<FString>(TEXT("Thick surface (interior potential)")));
	S->Formulations->Add(MakeShared<FString>(TEXT("Mean camber surface")));
	S->Formulation = (*S->Formulations)[0];
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
					.OnCheckStateChanged_Lambda([S](ECheckBoxState) { S->bSymmetryTouched = true; })
					[ SNew(STextBlock).Text(LOCTEXT("Symmetry", "Symmetry plane at y = 0  (every body solved with its image; a body on the centreline is cut there)")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					SAssignNew(S->LblWarn, STextBlock).AutoWrapText(true).Visibility(EVisibility::Collapsed)
					.ColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.75f, 0.3f)))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("BodiesHint",
						"Every lofted solid in the scene can be paneled: a wing as a lifting doublet network with a wake shed from its trailing edge, a fuselage, nacelle or primitive as a closed source network built from its sections. "
						"A mirrored entity appears once: with the symmetry plane on, its image stands in for the other half; off, the reflected copy is paneled as well. The plane defaults on when something is mirrored or a wing root sits on y = 0."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("PanelingGroup", "Paneling"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					FormRow(LOCTEXT("FormRow", "Wing formulation:"),
						SNew(SComboBox<TSharedPtr<FString>>)
						.OptionsSource(&*S->Formulations)
						.InitiallySelectedItem(S->Formulation)
						.OnGenerateWidget_Lambda([](TSharedPtr<FString> F) { return SNew(STextBlock).Text(FText::FromString(*F)); })
						.OnSelectionChanged_Lambda([S](TSharedPtr<FString> F, ESelectInfo::Type) { if (F.IsValid()) S->Formulation = F; })
						[ SNew(STextBlock).Text_Lambda([S]() { return FText::FromString(S->Formulation.IsValid() ? *S->Formulation : FString()); }) ])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Chordwise", "Wing: chordwise per surface:"), Spin(&FState::Chordwise, 2, 200, 1, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Spanwise", "Wing: spanwise panels:"), Spin(&FState::Spanwise, 1, 200, 1, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Wake", "Wing: wake length, root chords:"), Spin(&FState::WakeChords, 1, 1000, 5, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Along", "Bodies: panels along:"), Spin(&FState::Along, 1, 400, 1, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Around", "Bodies: panels round a section:"), Spin(&FState::Around, 3, 400, 1, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("PanelingHint",
						"Curved panels carrying linear sources and quadratic doublets, after Johnson (NASA CR-3079). Wing, thick surface: the sections paneled round, the interior potential held at zero and the sources prescribed; mean camber surface: a doublet sheet with no flow through it. "
						"Both shed a wake along the free stream, which is what carries the lift. Cosine spacing both ways; a pointed tip is paneled a fraction short of the point. "
						"Bodies: the panel counts are added to each loft's own stations and section corners, so the total runs higher than the product."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("FreeStream", "Operating conditions and references"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Aoa", "Angle of attack, deg:"), SAssignNew(S->AlphaBox, SEditableTextBox).Text(FText::FromString(TEXT("4"))).ToolTipText(LOCTEXT("AoaTip", "One angle, or a comma separated list: 0, 2, 4, 6"))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Aos", "Angle of sideslip, deg:"), SAssignNew(S->BetaBox, SEditableTextBox).Text(FText::FromString(TEXT("0"))).ToolTipText(LOCTEXT("AosTip", "Zero for now; a list here with a single angle of attack would sweep the sideslip"))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					SAssignNew(S->LblInput, STextBlock).AutoWrapText(true).Visibility(EVisibility::Collapsed)
					.ColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.45f, 0.35f)))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Area", "Reference area S_ref:"), Spin(&FState::ReferenceArea, 1e-12, 1e12, 1.0, AreaSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Length", "Reference length (moments):"), Spin(&FState::ReferenceLength, 1e-12, 1e12, 0.1, LengthSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("FlowHint",
						"The flow runs along +x, tilted towards +z by the angle of attack. A comma separated list of angles of attack (or of sideslip, not both) solves one case per angle: the other tabs step through them. "
						"A potential flow is similar at every speed and density: Cp = 1 - (V/V∞)² and the coefficients are C_F = -∮ Cp n dA / S_ref, so neither is asked for. "
						"The reference area defaults to the planform area of the chosen wings (both halves) and the reference length to the first root chord; with no wing, to the first body's frontal ellipse and length. Both are in the project's units."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Output", "Solution file (panels with Cp, doublet and source strength; one per angle; leave empty to skip)"),
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
				.Text_Lambda([S]() { return S->Cases.Num() ? LOCTEXT("ViewHint", "Left-drag orbits, right-drag pans, wheel zooms, Home frames. Click the colour bar for the range and map.") : LOCTEXT("ViewEmpty", "Solve first (Set-up tab); the bodies appear here coloured by Cp."); })
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
					TArray<int32> W = S->WingShares();
					const FCase* C = S->CurrentCase();
					if (C == nullptr || W.Num() == 0) return LOCTEXT("NoWingShort", "no wing");
					const int32 i = FMath::Clamp(S->CurrentWing, 0, W.Num() - 1);
					return FText::FromString(FString::Printf(TEXT("%s  (%d of %d)"), *C->Shares[W[i]].Name, i + 1, W.Num()));
				},
				[S](int32 Delta)
				{
					const int32 N = S->WingShares().Num();
					if (N == 0) return;
					S->CurrentWing = FMath::Clamp(S->CurrentWing + Delta, 0, N - 1);
					S->UpdateLoads();
				},
				[S]() { return S->WingShares().Num() > 1; })
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
	Status(LOCTEXT("Pick", "Choose the bodies, the paneling and the angles, then Solve."));
	ShowAerotelicDialogWindow(Window, Size);

	if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicPanelSmoke")))
	{
		S->bSmoke = true;
		S->AlphaBox->SetText(FText::FromString(TEXT("0, 4")));
		S->PathBox->SetText(FText::GetEmpty());
		FParse::Value(FCommandLine::Get(), TEXT("AerotelicPanelChordwise="), S->Chordwise);
		FParse::Value(FCommandLine::Get(), TEXT("AerotelicPanelSpanwise="), S->Spanwise);
		S->Solve();
	}
}

#undef LOCTEXT_NAMESPACE
