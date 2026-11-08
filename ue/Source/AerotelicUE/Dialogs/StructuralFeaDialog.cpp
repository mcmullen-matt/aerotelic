#include "StructuralFeaDialog.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Crc.h"
#include "Misc/ScopeLock.h"
#include "Async/Async.h"
#include "Containers/Ticker.h"
#include "ImageUtils.h"
#include "Styling/CoreStyle.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/SceneView.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Standalone/AerotelicColoredMeshActor.h"
#include "Standalone/AerotelicCameraPawn.h"
#include "Entity/AerotelicStructuralFea.h"
#include "Entity/AerotelicFeaPreprocess.h"
#include "Entity/AerotelicTetGmsh.h"

#include "aerotelic/lattice.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/units.hpp"

#include <atomic>
#include <memory>

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBodyStructure.h"
#include "Entity/AerotelicFeaModelBuilder.h"

#define LOCTEXT_NAMESPACE "StructuralFeaDialog"

namespace
{
	using AerotelicFeaPreprocess::FFace;
	using AerotelicFeaPreprocess::FModel;
	using aerotelic::project_yaml::EntityType;

	FString Thousands(int64 N)
	{
		return FText::AsNumber(N).ToString();
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field, float LabelWidth = 200.0f)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(LabelWidth) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-")))
			.AutoWrapText(true)
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
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_fea.vtk"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_fea.vtk"));
	}

	FString UnitName(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }

	bool HasStructuralMaterial(const aerotelic::MassSpec& S)
	{
		return S.mode == aerotelic::MassMode::Material && S.material.IsStructural();
	}

	bool SameOperand(const FAerotelicBooleanOperand& A, const FAerotelicBooleanOperand& B)
	{
		return A.Entity.Get() == B.Entity.Get() && A.bMirror == B.bMirror;
	}

	FString OperandLabel(const FAerotelicBooleanOperand& Op)
	{
		return Op.Entity.IsValid() ? Op.Entity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT("")) : FString(TEXT("(removed entity)"));
	}

	// The SOLID BODIES the analysis picks among: every body of every scene
	// operand (a body's infill is analysed as its homogenised core). A
	// wing's flap, a propeller's blade, a boolean result's separate shell
	// is each a row of its own; an operand whose bodies are not known yet
	// (never tessellated) is one row for all of them.
	TArray<FAerotelicBodyChoicePtr> AnalysableChoices()
	{
		TArray<FAerotelicBodyChoicePtr> Out;
		for (const FAerotelicBodyChoicePtr& C : SceneBodyChoices())
		{
			if (!C->Operand.Entity.IsValid()) continue;
			const TArray<FAerotelicBody>& Bodies = C->Operand.Entity->GetBodies();
			if (Bodies.Num() <= 1)
			{
				FAerotelicBodyChoicePtr One = MakeShared<FAerotelicBodyChoice>(*C);
				One->Body = Bodies.Num() == 1 ? Bodies[0].Name : FString();
				Out.Add(One);
				continue;
			}
			for (const FAerotelicBody& B : Bodies)
			{
				FAerotelicBodyChoicePtr One = MakeShared<FAerotelicBodyChoice>(*C);
				One->Body = B.Name;
				One->Label = C->Label + TEXT(" / ") + B.Name;
				Out.Add(One);
			}
		}
		return Out;
	}


	// ---- the pictures' geometry: flat-shaded triangles, every one in both
	// windings so the two-sided vertex-colour material shows it from either
	// side (what the main view does for the bodies) ----

	struct FGeom
	{
		TArray<FVector> V;
		TArray<int32> T;
		TArray<FVector> N;
		TArray<FColor> C;

		void Tri(const FVector& A, const FVector& B, const FVector& Cc, const FColor& Col)
		{
			const FVector Normal = FVector::CrossProduct(B - A, Cc - A).GetSafeNormal();
			const int32 Base = V.Num();
			V.Append({ A, B, Cc, A, Cc, B });
			N.Append({ Normal, Normal, Normal, -Normal, -Normal, -Normal });
			C.Append({ Col, Col, Col, Col, Col, Col });
			T.Append({ Base, Base + 1, Base + 2, Base + 3, Base + 4, Base + 5 });
		}
		void Quad(const FVector& A, const FVector& B, const FVector& Cc, const FVector& D, const FColor& Col)
		{
			Tri(A, B, Cc, Col);
			Tri(A, Cc, D, Col);
		}
	};

	FColor Srgb(float R, float G, float B) { return AAerotelicColoredMeshActor::VertexColorFromSRGB(R, G, B); }

	void Basis(const FVector& Dir, FVector& U, FVector& W)
	{
		const FVector Helper = FMath::Abs(Dir.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector;
		U = FVector::CrossProduct(Dir, Helper).GetSafeNormal();
		W = FVector::CrossProduct(Dir, U).GetSafeNormal();
	}

	// An arrow whose HEAD sits at Tip (the point of application) pointing
	// along Dir: a square shaft and a six-sided cone.
	void AddArrow(FGeom& G, const FVector& Tip, const FVector& Dir, double Length, double Radius, const FColor& Col)
	{
		if (Length <= 0.0 || Dir.IsNearlyZero()) return;
		const FVector D = Dir.GetSafeNormal();
		FVector U, W;
		Basis(D, U, W);
		const double Head = FMath::Min(Length * 0.35, Radius * 6.0);
		const FVector Tail = Tip - D * Length;
		const FVector Neck = Tip - D * Head;
		const FVector Corners[4] = { U * Radius + W * Radius, U * Radius - W * Radius, -U * Radius - W * Radius, -U * Radius + W * Radius };
		for (int32 i = 0; i < 4; ++i)
		{
			G.Quad(Tail + Corners[i], Tail + Corners[(i + 1) % 4], Neck + Corners[(i + 1) % 4], Neck + Corners[i], Col);
		}
		G.Quad(Tail + Corners[0], Tail + Corners[3], Tail + Corners[2], Tail + Corners[1], Col);
		const double R = Radius * 2.4;
		constexpr int32 Sides = 6;
		FVector Ring[Sides];
		for (int32 i = 0; i < Sides; ++i)
		{
			const double A = 2.0 * PI * i / Sides;
			Ring[i] = Neck + (U * FMath::Cos(A) + W * FMath::Sin(A)) * R;
		}
		for (int32 i = 0; i < Sides; ++i)
		{
			G.Tri(Ring[i], Ring[(i + 1) % Sides], Tip, Col);
			G.Tri(Neck, Ring[(i + 1) % Sides], Ring[i], Col);
		}
	}

	void AddOctahedron(FGeom& G, const FVector& P, double R, const FColor& Col)
	{
		const FVector X(R, 0, 0), Y(0, R, 0), Z(0, 0, R);
		const FVector Px = P + X, Mx = P - X, Py = P + Y, My = P - Y, Pz = P + Z, Mz = P - Z;
		G.Tri(Px, Py, Pz, Col); G.Tri(Py, Mx, Pz, Col); G.Tri(Mx, My, Pz, Col); G.Tri(My, Px, Pz, Col);
		G.Tri(Py, Px, Mz, Col); G.Tri(Mx, Py, Mz, Col); G.Tri(My, Mx, Mz, Col); G.Tri(Px, My, Mz, Col);
	}

	void AddBar(FGeom& G, const FVector& A, const FVector& B, double R, const FColor& Col)
	{
		const FVector D = (B - A).GetSafeNormal();
		if (D.IsNearlyZero()) return;
		FVector U, W;
		Basis(D, U, W);
		const FVector Corners[4] = { U * R + W * R, U * R - W * R, -U * R - W * R, -U * R + W * R };
		for (int32 i = 0; i < 4; ++i)
		{
			G.Quad(A + Corners[i], A + Corners[(i + 1) % 4], B + Corners[(i + 1) % 4], B + Corners[i], Col);
		}
	}

	FColor RegionColor(int32 Index)
	{
		static const FColor Palette[] = {
			Srgb(0.36f, 0.62f, 0.90f), Srgb(0.95f, 0.65f, 0.25f), Srgb(0.45f, 0.78f, 0.45f), Srgb(0.85f, 0.45f, 0.75f),
			Srgb(0.80f, 0.80f, 0.35f), Srgb(0.40f, 0.80f, 0.80f), Srgb(0.90f, 0.50f, 0.45f), Srgb(0.65f, 0.60f, 0.90f) };
		return Palette[Index % UE_ARRAY_COUNT(Palette)];
	}

	const FColor kGrey = Srgb(0.74f, 0.75f, 0.77f);
	const FColor kPressure = Srgb(1.0f, 0.62f, 0.15f);
	const FColor kPointForce = Srgb(0.95f, 0.18f, 0.18f);
	const FColor kGravity = Srgb(0.30f, 0.85f, 0.35f);
	const FColor kFixed = Srgb(0.95f, 0.15f, 0.15f);
	const FColor kPlane = Srgb(0.35f, 0.55f, 1.0f);
	const FColor kHandle = Srgb(1.0f, 0.75f, 0.2f);
	const FColor kHandleHot = Srgb(1.0f, 0.95f, 0.5f);

	// A point force as the user types it (project units, newtons).
	struct FPointRow
	{
		double X = 0.0, Y = 0.0, Z = 0.0;
		double Fx = 0.0, Fy = 0.0, Fz = -1000.0;
	};
	// A support plane as the user types it.
	struct FSupportRow
	{
		int32 Axis = 1;     // x, y, z
		int32 Where = 0;    // at the value, at the mesh's minimum, at its maximum
		// Which body's minimum / maximum: 0 the whole mesh, i > 0 the picker's
		// choice i - 1 (BodyOptions), so the body that sticks out furthest
		// does not capture a support meant for another.
		int32 Body = 0;
		// Typed FULL-SCALE, as the parameterisations are; divided by the model
		// scale (Model > Scale) on the way into the request, whose mesh is
		// the replica's.
		double Value = 0.0;
		double Tolerance = 0.0;
		// The patch (2026-11-05): bounded, only the nodes inside Lo..Hi along
		// the plane's two in-plane axes (FSupport::InPlaneAxis) are fixed.
		// Full-scale, like Value. Set by dragging the handles in the Support
		// picture, by the fields on the card, or by Fit to body.
		bool bBounded = false;
		double Lo[2] = { 0.0, 0.0 };
		double Hi[2] = { 0.0, 0.0 };
	};

	// A handle of a support patch in the Support picture: which plane,
	// which part of its rectangle (0..3 the corners lo/lo, hi/lo, hi/hi,
	// lo/hi; 4..7 the edges u lo, u hi, w lo, w hi) and where it is drawn
	// (replica coordinates).
	struct FPatchHandle
	{
		int32 Plane = -1;
		int32 Kind = -1;
		FVector World = FVector::ZeroVector;
	};
	constexpr double kHandlePickPixels = 14.0;

	// The model as built from the chosen bodies: the full mesh as meshed,
	// the mesh the solver gets (the full one less whatever floats free of
	// the supports), the request, and what the pre-processor derives. One
	// heap object, so the request's pointers into it stay good.
	struct FBuilt
	{
		aerotelic::TetMesh FullMesh;
		TArray<AerotelicStructuralFea::FShellFace> FullShellFaces;
		TArray<AerotelicStructuralFea::FShellFace> FullRibFaces;
		aerotelic::TetMesh Mesh;
		std::vector<std::unique_ptr<aerotelic::UnitCellSamples>> Cells;   // one per lattice homogenised
		AerotelicStructuralFea::FRequest Request;
		FModel Model;
		TArray<FString> Problems;   // left out while meshing
		TArray<FString> Dropped;    // left out for floating, this derivation
	};

	// A solve in flight (2026-11-05): the script runs on a worker thread
	// through AerotelicStructuralFea::Run, which blocks for minutes on a
	// large model; the dialog's ticker reads the last line it printed and
	// shows it with the elapsed time, and Cancel ends the process. Holds
	// the built model so the request's pointers stay good until the worker
	// is done, whatever the dialog does meanwhile.
	struct FSolveJob
	{
		TSharedPtr<FBuilt> Built;
		AerotelicStructuralFea::FResult Result;
		FString Error;
		AerotelicMeshLib::FJobProgress Progress;
		std::atomic<bool> bDone{ false };
		bool bOk = false;
		double Started = 0.0;
		FCriticalSection Lock;
		FString LastLine;   // under Lock: written by the worker, read by the ticker
		FTSTicker::FDelegateHandle Ticker;

		bool IsRunning() const { return !bDone.load(); }
		FString TakeLastLine()
		{
			FScopeLock L(&Lock);
			return LastLine;
		}
	};

	enum ETab { TabSetup = 0, TabMesh = 1, TabLoads = 2, TabSupport = 3, TabHelp = 4 };
	constexpr int32 kPictures = 4;   // indexed by ETab; TabSetup's slot unused

	// ---- the dialog's state: owned by its widgets (the window is modeless) ----

	struct FState : public TSharedFromThis<FState>
	{
		TFunction<void(const FText&)> Status;
		TWeakPtr<SWindow> Window;
		aerotelic::UnitSystem Units;

		// Inputs. The option arrays live here, with the combos that read
		// them (see slate-modeless-dialog-lifetimes).
		TArray<FAerotelicBodyChoicePtr> Choices;
		TSharedPtr<SAerotelicBodyPicker> Picker;
		TArray<TSharedPtr<FString>> ModelOptions, SolverOptions, ShellOptions, AxisOptions, PlaneOptions, BodyOptions;
		int32 ModelIndex = 0, SolverIndex = 0, ShellIndex = 0;
		// The model scale (Model > Scale) as it stands: the meshes are the
		// replica's, the user types full-scale coordinates. Read live, so a
		// scale changed while the dialog is open is honoured on the next
		// Build or Apply.
		static double ModelScale() { return AAerotelicEntityActor::GetModelScale(); }
		static bool IsReplica() { return !FMath::IsNearlyEqual(ModelScale(), 1.0, 1e-9); }
		// A replica-space length or coordinate shown as the full-scale one.
		static double Full(double Replica) { return Replica / ModelScale(); }
		static FVector Full(const FVector& Replica) { return Replica / ModelScale(); }
		// "1:4" for a quarter-scale replica, "x 2.5" for an enlargement (the
		// status bar's wording).
		static FString ScaleText()
		{
			const double S = ModelScale();
			if (S < 1.0)
			{
				const double Inverse = 1.0 / S;
				return FMath::IsNearlyEqual(Inverse, FMath::RoundToDouble(Inverse), 1e-6 * Inverse)
					? FString::Printf(TEXT("1:%d"), FMath::RoundToInt(Inverse)) : FString::Printf(TEXT("1:%.4g"), Inverse);
			}
			return FString::Printf(TEXT("x %.4g"), S);
		}
		double Voxels = 32.0;
		double ElementSize = 0.0;   // solid bodies; 0: a twentieth of the largest extent
		double Modes = 6.0;         // natural frequencies computed after the static solve; 0: none
		// The loads and the supports: any number of point forces and of
		// planes, edited on their tabs and pushed by Apply.
		double LoadFactor = 0.0, Pressure = 0.0;
		TArray<TSharedPtr<FPointRow>> PointRows;
		TArray<TSharedPtr<FSupportRow>> SupportRows;
		TSharedPtr<SVerticalBox> PointList, SupportList;
		TSharedPtr<SEditableTextBox> PathBox;
		TSharedPtr<SCheckBox> SecondOrderCheck, CacheCheck;
		TSharedPtr<STextBlock> LblMaterials, LblMesh, LblCore, LblDisp, LblStress, LblMargins, LblBalance, LblModes, LblTime;
		// The solve's own status line and marquee in the dialog, and the
		// buttons that go quiet while it runs.
		TSharedPtr<STextBlock> LblProgress;
		TSharedPtr<SProgressBar> ProgressBar;
		TSharedPtr<SButton> BuildButton, SolveButton, CancelButton;

		// The model, once built (Build Model, or Solve).
		TSharedPtr<FBuilt> Built;
		// The solve in flight, if one is.
		TSharedPtr<FSolveJob> Job;
		// The tetrahedral meshes of the bodies' own grids, keyed by the
		// surface's bytes, the cell size and the ribs, so Build Model after a
		// load or support edit does not run gmsh again
		// (AerotelicFeaModelBuilder::FCache).
		AerotelicFeaModelBuilder::FCache TetCache;

		// The pre-processor pictures: one view per tab, its actors, its caption.
		TSharedPtr<SAerotelicSceneView> Views[kPictures];
		TSharedPtr<STextBlock> Captions[kPictures];
		TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>> Actors[kPictures];
		bool bFramed[kPictures] = { false, false, false, false };

		TSharedPtr<SAerotelicTabStrip> Tabs;
		bool bSmoke = false;

		// The support patches' handles as last drawn, and the one being
		// dragged (its plane and kind; -1 none). Hover is the handle under
		// the cursor, drawn brighter.
		TArray<FPatchHandle> Handles;
		int32 DragPlane = -1, DragKind = -1;
		int32 HoverHandle = -1;

		// The three ways a body with an interior may be analysed
		// (ModelOptions, AerotelicFeaModelBuilder::EModel): shells on its own
		// grid over a homogenised core with the ribs embedded; the generated
		// structure's skin / core tetrahedra at full resolution; or with the
		// interior IGNORED, the body as its skin and material alone say it is.
		bool IsSolving() const { return Job.IsValid() && Job->IsRunning(); }

		void Message(const FText& Text)
		{
			AerotelicMessageBox(EAppMsgType::Ok, Text, LOCTEXT("Title", "Structural (Kratos FEA)"));
		}

		// The builder's options as the Set-up tab stands (the element size
		// resolved against the chosen bodies' extent by the caller).
		AerotelicFeaModelBuilder::FOptions BuilderOptions(double Element) const
		{
			AerotelicFeaModelBuilder::FOptions O;
			O.Model = static_cast<AerotelicFeaModelBuilder::EModel>(FMath::Clamp(ModelIndex, 0, 2));
			O.Voxels = Voxels;
			O.ElementSize = Element;
			O.bSecondOrder = SecondOrderCheck.IsValid() && SecondOrderCheck->IsChecked();
			TFunction<void(const FText&)> St = Status;
			O.Status = [St](const FString& Line) { if (St) St(FText::FromString(Line)); };
			return O;
		}

		TArray<FAerotelicBodyChoicePtr> ChosenBodies() const
		{
			TArray<FAerotelicBodyChoicePtr> Out;
			if (!Picker.IsValid()) return Out;
			for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen()) if (C->Operand.Entity.IsValid()) Out.Add(C);
			return Out;
		}

		// What a body is, from its structure (body_structure.hpp), and how it
		// will be modelled -- AerotelicFeaModelBuilder::Describe, the same
		// rules AppendBody builds by.
		void DescribeModel()
		{
			if (!LblMaterials.IsValid()) return;
			TArray<FString> Lines;
			const AerotelicFeaModelBuilder::FOptions O = BuilderOptions(ElementSize);
			for (const FAerotelicBodyChoicePtr& C : ChosenBodies()) Lines.Add(AerotelicFeaModelBuilder::Describe(C->Operand, C->Body, C->Label, O, Units));
			LblMaterials->SetText(Lines.Num() ? FText::FromString(FString::Join(Lines, TEXT("\n"))) : LOCTEXT("NoBodies", "Choose at least one body."));
		}

		// The supports and loads as they stand on their tabs, into a request.
		void FillInputs(AerotelicStructuralFea::FRequest& Request) const
		{
			// The rows are typed full-scale; the mesh is the replica's
			// (Model > Scale), so every length goes in times the scale.
			const double Scale = ModelScale();
			Request.Supports.Reset();
			for (const TSharedPtr<FSupportRow>& R : SupportRows)
			{
				AerotelicStructuralFea::FSupport S;
				S.Axis = (*AxisOptions[FMath::Clamp(R->Axis, 0, 2)])[0];
				S.Value = R->Value * Scale;
				S.bAtMin = R->Where == 1;
				S.bAtMax = R->Where == 2;
				S.Tolerance = R->Tolerance * Scale;
				S.Body = R->Body > 0 ? R->Body - 1 : -1;
				S.bBounded = R->bBounded;
				for (int32 k = 0; k < 2; ++k) { S.Lo[k] = R->Lo[k] * Scale; S.Hi[k] = R->Hi[k] * Scale; }
				Request.Supports.Add(S);
			}
			Request.PointForces.Reset();
			for (const TSharedPtr<FPointRow>& R : PointRows)
			{
				AerotelicStructuralFea::FPointForce P;
				P.At = aerotelic::Vec3{ R->X * Scale, R->Y * Scale, R->Z * Scale };
				P.Force = aerotelic::Vec3{ R->Fx, R->Fy, R->Fz };
				Request.PointForces.Add(P);
			}
			Request.LoadFactor = LoadFactor;
			Request.Pressure = Pressure * Units.StressToSI();
			Request.bSecondOrder = SecondOrderCheck.IsValid() && SecondOrderCheck->IsChecked();
			static const TCHAR* kSolvers[] = { TEXT("auto"), TEXT("sparse_lu"), TEXT("amgcl") };
			Request.LinearSolver = kSolvers[FMath::Clamp(SolverIndex, 0, 2)];
			// The thick corotational triangle is no longer offered (2026-11-02:
			// too stiff and not converging on a thin-walled box, its force
			// balance drifting with refinement); the script still takes it.
			static const TCHAR* kShellElements[] = { TEXT("ShellThinElement3D3N"), TEXT("ShellThinElementCorotational3D3N") };
			Request.ShellElement = kShellElements[FMath::Clamp(ShellIndex, 0, 1)];
			Request.bHomogenizeCache = CacheCheck.IsValid() && CacheCheck->IsChecked();
			Request.Modes = FMath::Max(FMath::RoundToInt32(Modes), 0);
			Request.VtkPath = PathBox.IsValid() ? PathBox->GetText().ToString().TrimStartAndEnd() : FString();
		}

		// ---- the model: mesh the chosen bodies once; derive the rest ----

		bool BuildModel()
		{
			if (IsSolving())
			{
				Message(LOCTEXT("Busy", "A solve is running: wait for it, or Cancel it."));
				return false;
			}
			const TArray<FAerotelicBodyChoicePtr> Bodies = ChosenBodies();
			if (Bodies.Num() == 0)
			{
				Message(LOCTEXT("NoneChosen", "Choose at least one body to analyse."));
				return false;
			}
			Status(LOCTEXT("Meshing", "Building the model: meshing the bodies..."));
			TSharedRef<FBuilt> B = MakeShared<FBuilt>();
			AerotelicStructuralFea::FRequest& Request = B->Request;
			aerotelic::TetMesh& Mesh = B->FullMesh;

			FBox Bounds(ForceInit);
			for (const FAerotelicBodyChoicePtr& C : Bodies) Bounds += C->Operand.Entity->GetPlacedBounds(C->Operand.bMirror);
			const double Element = ElementSize > 0.0 ? ElementSize : (Bounds.IsValid ? FMath::Max(Bounds.GetSize().GetMax() / 20.0, 1e-6) : 1.0);

			// Each chosen body through the shared builder (the commandlets
			// build the same way): its own grid, its structure's skin, infill
			// and rib shells, labelled "<entity>[ (mirror)][ / <body>]". The
			// picker's index of the body is kept for a support resolved
			// against it alone (FSupport::Body).
			const AerotelicFeaModelBuilder::FOptions Options = BuilderOptions(Element);
			AerotelicFeaModelBuilder::FBuilt Built2;
			for (const FAerotelicBodyChoicePtr& C : Bodies)
			{
				AerotelicFeaModelBuilder::AppendBody(C->Operand, C->Body, C->Label, Choices.IndexOfByKey(C), Options, TetCache, Built2);
			}
			Mesh = std::move(Built2.Mesh);
			B->FullShellFaces = Built2.ShellFaces;
			B->FullRibFaces = Built2.RibFaces;
			B->Cells = std::move(Built2.Cells);
			Request.Regions = Built2.Regions;
			B->Problems = Built2.Problems;
			if (Mesh.tets.empty() && B->FullShellFaces.Num() == 0)
			{
				Status(LOCTEXT("Ready", "Ready"));
				Message(FText::FromString(TEXT("No model could be built:\n") + FString::Join(B->Problems, TEXT("\n"))));
				return false;
			}
			Request.LengthToSI = Units.LengthToSI();
			Built = B;
			if (!Derive()) return false;
			const FModel& M = B->Model;
			Status(FText::FromString(FString::Printf(TEXT("Model built: %s nodes, %s tetrahedra, %s wetted faces, %s nodes on the supports; %s%s -- see the Mesh, Loads and Support tabs"),
				*Thousands(M.Nodes), *Thousands(M.Tets), *Thousands(M.WettedFaces.Num()), *Thousands(M.SupportNodes.Num()), *CostText(M),
				M.MeshIsPoor() ? *FString::Printf(TEXT("; POOR MESH: %s"), *QualityText(M)) : TEXT(""))));
			if (B->Problems.Num() > 0 || B->Dropped.Num() > 0)
			{
				TArray<FString> All = B->Problems;
				All.Append(B->Dropped);
				Message(FText::FromString(TEXT("Left out of the model:\n") + FString::Join(All, TEXT("\n"))));
			}
			return true;
		}

		// What the solve will cost, from the pre-processor's count of the
		// degrees of freedom (the script's own rule, so the figure is the
		// one the result will report): which solver "auto" takes, and a
		// warning past the direct solver's budget.
		bool OverBudget(const FModel& M) const
		{
			return M.Dofs > AerotelicStructuralFea::kAutoDirectDofs;
		}
		// The tetrahedra's shape: the dihedral angles and how many fall
		// outside the acceptance band, with the verdict.
		FString QualityText(const FModel& M) const
		{
			if (M.Tets == 0) return TEXT("no tetrahedra (shells only)");
			FString Text = FString::Printf(TEXT("dihedral angles %.2f to %.2f deg; %s tetrahedra (%.1f%%) with an angle under 10 deg and %s (%.1f%%) over 160"),
				M.MinDihedral, M.MaxDihedral, *Thousands(M.TetsDihedralUnder10), 100.0 * M.TetsDihedralUnder10 / M.Tets, *Thousands(M.TetsDihedralOver160), 100.0 * M.TetsDihedralOver160 / M.Tets);
			if (M.MeshIsPoor())
			{
				Text += FString::Printf(TEXT(" -- %.1f%% outside the band, more than the %.0f%% a solver should be given: slivers spoil the conditioning and the stresses in them. Look at the bodies' grids on their Mesh tabs (a fuselage's nose and tail pole fans are the usual cause; a surface much finer than the lattice cell the other)"),
					100.0 * M.PoorTetFraction(), 100.0 * FModel::kPoorTetFraction);
			}
			else
			{
				Text += FString::Printf(TEXT(" -- within the %.0f%% band"), 100.0 * FModel::kPoorTetFraction);
			}
			return Text;
		}
		FString CostText(const FModel& M) const
		{
			const bool bDirect = SolverIndex == 1 || (SolverIndex == 0 && M.DirectSolverAuto());
			FString Text = FString::Printf(TEXT("about %s DOF (%s mid-edge nodes on %s 10-node tetrahedra, %s shell nodes), %s"),
				*Thousands(M.Dofs), *Thousands(M.MidEdgeNodes), *Thousands(M.SecondOrderTets), *Thousands(M.ShellNodes),
				bDirect ? TEXT("the direct solver") : TEXT("the iterative solver (amgcl, every core)"));
			if (OverBudget(M))
			{
				Text += FString::Printf(TEXT(" -- OVER the direct solver's budget of %s DOF on a desktop%s: coarsen the bodies' Mesh tab grids or switch off second order"),
					*Thousands(AerotelicStructuralFea::kAutoDirectDofs),
					bDirect ? TEXT(" (the direct solver was asked for by name: it may take hours or run out of memory)") : TEXT(" (the force balance says afterwards whether the iterative solver converged)"));
			}
			return Text;
		}

		// The supports and loads applied to the built mesh: the parts with
		// no load path to any support are left out (a control surface in
		// its gap, a payload bay inside the fuselage -- they would make the
		// system singular), the rest derived for the pictures and the solve.
		bool Derive(bool bReframe = true)
		{
			if (!Built.IsValid()) return false;
			if (IsSolving())
			{
				Message(LOCTEXT("Busy", "A solve is running: wait for it, or Cancel it."));
				return false;
			}
			FBuilt& B = *Built;
			FillInputs(B.Request);
			B.Mesh = B.FullMesh;
			B.Request.Mesh = &B.Mesh;
			B.Request.ShellFaces = B.FullShellFaces;
			B.Request.RibFaces = B.FullRibFaces;
			B.Dropped.Reset();
			FString Error;
			if (!AerotelicFeaPreprocess::Build(B.Request, B.Model, Error))
			{
				Status(LOCTEXT("Ready", "Ready"));
				Message(FText::FromString(TEXT("The model could not be prepared: ") + Error));
				return false;
			}
			if (B.Model.FloatingNodes > 0 && B.Model.SupportNodes.Num() > 0)
			{
				// Keep the elements on supported pieces; drop the rest and the
				// nodes that go with them, renumbering.
				const FModel& M = B.Model;
				auto Kept = [&M](int32 Node) { return M.ComponentSupported.IsValidIndex(M.NodeComponent[Node]) && M.ComponentSupported[M.NodeComponent[Node]]; };
				TMap<int32, int32> DroppedPerRegion;
				aerotelic::TetMesh Trimmed;
				TArray<AerotelicStructuralFea::FShellFace> TrimmedFaces, TrimmedRibs;
				TArray<int32> NewIndex;
				NewIndex.Init(-1, static_cast<int32>(B.Mesh.nodes.size()));
				auto Index = [&](int32 Old)
				{
					if (NewIndex[Old] < 0) { NewIndex[Old] = static_cast<int32>(Trimmed.nodes.size()); Trimmed.nodes.push_back(B.Mesh.nodes[Old]); }
					return NewIndex[Old];
				};
				for (size_t t = 0; t < B.Mesh.tets.size(); ++t)
				{
					const auto& T = B.Mesh.tets[t];
					const int32 Region = t < B.Mesh.material.size() ? B.Mesh.material[t] : 1;
					if (!Kept(T[0])) { ++DroppedPerRegion.FindOrAdd(Region, 0); continue; }
					Trimmed.tets.push_back({ Index(T[0]), Index(T[1]), Index(T[2]), Index(T[3]) });
					Trimmed.material.push_back(Region);
				}
				for (const AerotelicStructuralFea::FShellFace& F : B.Request.ShellFaces)
				{
					if (!Kept(F.N[0])) { ++DroppedPerRegion.FindOrAdd(F.Region, 0); continue; }
					AerotelicStructuralFea::FShellFace G = F;
					G.N[0] = Index(F.N[0]); G.N[1] = Index(F.N[1]); G.N[2] = Index(F.N[2]);
					TrimmedFaces.Add(G);
				}
				for (const AerotelicStructuralFea::FShellFace& F : B.Request.RibFaces)
				{
					if (!Kept(F.N[0])) { ++DroppedPerRegion.FindOrAdd(F.Region, 0); continue; }
					AerotelicStructuralFea::FShellFace G = F;
					G.N[0] = Index(F.N[0]); G.N[1] = Index(F.N[1]); G.N[2] = Index(F.N[2]);
					TrimmedRibs.Add(G);
				}
				for (const auto& Entry : DroppedPerRegion)
				{
					const AerotelicStructuralFea::FRegion* R = B.Request.Regions.FindByPredicate([&Entry](const AerotelicStructuralFea::FRegion& X) { return X.Id == Entry.Key; });
					const bool bWhole = M.FloatingRegions.Contains(Entry.Key);
					B.Dropped.Add(FString::Printf(TEXT("%s: %s element%s with no load path to any support left out%s"), R ? *R->Label : *FString::FromInt(Entry.Key),
						*Thousands(Entry.Value), Entry.Value == 1 ? TEXT("") : TEXT("s"), bWhole ? TEXT(" (the whole body: it touches neither a support plane nor a supported body)") : TEXT("")));
				}
				B.Mesh = std::move(Trimmed);
				B.Request.ShellFaces = TrimmedFaces;
				B.Request.RibFaces = TrimmedRibs;
				// The regions that lost everything go too, or the solver would
				// want tetrahedra for them.
				B.Request.Regions.RemoveAll([&M](const AerotelicStructuralFea::FRegion& X) { return M.FloatingRegions.Contains(X.Id); });
				if (B.Mesh.tets.empty() && B.Request.ShellFaces.Num() == 0 && B.Request.RibFaces.Num() == 0)
				{
					Status(LOCTEXT("Ready", "Ready"));
					Message(FText::FromString(TEXT("Nothing reaches a support: every chosen body floats free of the support planes.\n") + FString::Join(B.Dropped, TEXT("\n"))));
					return false;
				}
				if (!AerotelicFeaPreprocess::Build(B.Request, B.Model, Error))
				{
					Status(LOCTEXT("Ready", "Ready"));
					Message(FText::FromString(TEXT("The model could not be prepared: ") + Error));
					return false;
				}
			}
			// "min" / "max" resolved to the value the pre-processor found (per
			// body where asked): the script knows only the whole mesh, and the
			// two must fix the same nodes (the commandlet compares the counts).
			for (int32 k = 0; k < B.Request.Supports.Num() && k < B.Model.SupportPlanes.Num(); ++k)
			{
				AerotelicStructuralFea::FSupport& S = B.Request.Supports[k];
				if (S.bAtMin || S.bAtMax)
				{
					S.Value = B.Model.SupportPlanes[k].Value;
					S.bAtMin = S.bAtMax = false;
				}
			}
			// Every Build and Apply reframes the pictures: the model may be of
			// other bodies, or the vehicle at another model scale. A patch
			// drag's release does not: the view stays where the user had it.
			if (bReframe) for (bool& F : bFramed) F = false;
			UpdatePictures();
			return true;
		}

		// ---- the support patches: rectangles, handles, live nodes ----

		// The rectangle a plane is drawn with, replica coordinates: the
		// patch when bounded, else the mesh's extent plus a margin (what an
		// infinite plane has always been drawn as, and what a patch starts
		// from when a handle of it is first dragged).
		void PlaneRect(int32 k, const FModel& M, double Lo[2], double Hi[2]) const
		{
			const FModel::FPlane& Plane = M.SupportPlanes[k];
			const bool bBounded = SupportRows.IsValidIndex(k) && SupportRows[k]->bBounded;
			const double Margin = 0.05 * FMath::Max(M.Diagonal, 1e-9);
			const double MinV[3] = { M.Bounds.min.x, M.Bounds.min.y, M.Bounds.min.z };
			const double MaxV[3] = { M.Bounds.max.x, M.Bounds.max.y, M.Bounds.max.z };
			for (int32 i = 0; i < 2; ++i)
			{
				const int32 A = AerotelicStructuralFea::FSupport::InPlaneAxis(Plane.Axis, i);
				if (bBounded)
				{
					Lo[i] = FMath::Min(SupportRows[k]->Lo[i], SupportRows[k]->Hi[i]) * ModelScale();
					Hi[i] = FMath::Max(SupportRows[k]->Lo[i], SupportRows[k]->Hi[i]) * ModelScale();
				}
				else
				{
					Lo[i] = MinV[A] - Margin;
					Hi[i] = MaxV[A] + Margin;
				}
			}
		}

		// The planes as the rows have them now (the resolved value and
		// tolerance from the last derivation, the patch from the rows), for
		// a redraw between a drag's moves without the full derivation.
		TArray<FModel::FPlane> LivePlanes() const
		{
			TArray<FModel::FPlane> Planes = Built.IsValid() ? Built->Model.SupportPlanes : TArray<FModel::FPlane>();
			for (int32 k = 0; k < Planes.Num() && k < SupportRows.Num(); ++k)
			{
				Planes[k].bBounded = SupportRows[k]->bBounded;
				for (int32 i = 0; i < 2; ++i) { Planes[k].Lo[i] = SupportRows[k]->Lo[i] * ModelScale(); Planes[k].Hi[i] = SupportRows[k]->Hi[i] * ModelScale(); }
				Planes[k].Nodes = 0;
				Planes[k].Furthest = 0.0;
			}
			return Planes;
		}

		// The nodes those planes fix, by the pre-processor's own rule.
		TArray<int32> LiveSupportNodes(TArray<FModel::FPlane>& Planes) const
		{
			TArray<int32> Nodes;
			if (!Built.IsValid()) return Nodes;
			const aerotelic::TetMesh& Mesh = Built->Mesh;
			for (int32 n = 0; n < static_cast<int32>(Mesh.nodes.size()); ++n)
			{
				bool bFixed = false;
				for (FModel::FPlane& P : Planes)
				{
					if (!AerotelicFeaPreprocess::NodeOnPlane(Mesh.nodes[n], P)) continue;
					++P.Nodes;
					P.Furthest = FMath::Max(P.Furthest, std::abs(Mesh.nodes[n][P.Axis] - P.Value));
					bFixed = true;
				}
				if (bFixed) Nodes.Add(n);
			}
			return Nodes;
		}

		// The handle under a pixel, or -1.
		int32 HandleAt(const FVector2D& Pixel) const
		{
			if (!Views[TabSupport].IsValid()) return -1;
			int32 Best = -1;
			double BestD = kHandlePickPixels;
			for (int32 i = 0; i < Handles.Num(); ++i)
			{
				FVector2D P;
				if (!Views[TabSupport]->WorldToPixel(Views[TabSupport]->LastGeometry(), Handles[i].World, P)) continue;
				const double D = FVector2D::Distance(P, Pixel);
				if (D < BestD) { BestD = D; Best = i; }
			}
			return Best;
		}

		bool OnPatchHover(const FVector2D& Pixel)
		{
			const int32 H = HandleAt(Pixel);
			if (H != HoverHandle)
			{
				HoverHandle = H;
				if (Built.IsValid()) UpdateSupportPicture(Built->Model.SupportPlanes, Built->Model.SupportNodes, false);
			}
			return H >= 0;
		}

		// A press on a handle starts a drag of its patch; a plane that was
		// infinite becomes the rectangle it was drawn as.
		bool OnPatchDown(const FVector2D& Pixel)
		{
			const int32 H = HandleAt(Pixel);
			if (H < 0 || !Built.IsValid() || IsSolving()) return false;
			const FPatchHandle& Handle = Handles[H];
			if (!SupportRows.IsValidIndex(Handle.Plane)) return false;
			TSharedPtr<FSupportRow> Row = SupportRows[Handle.Plane];
			if (!Row->bBounded)
			{
				double Lo[2], Hi[2];
				PlaneRect(Handle.Plane, Built->Model, Lo, Hi);
				Row->bBounded = true;
				for (int32 i = 0; i < 2; ++i) { Row->Lo[i] = Full(Lo[i]); Row->Hi[i] = Full(Hi[i]); }
			}
			DragPlane = Handle.Plane;
			DragKind = Handle.Kind;
			return true;
		}

		// The cursor's point on the plane moves the grabbed corner or edge.
		void OnPatchMove(const FVector& RayOrigin, const FVector& RayDir)
		{
			if (DragPlane < 0 || !Built.IsValid() || !SupportRows.IsValidIndex(DragPlane) || !Built->Model.SupportPlanes.IsValidIndex(DragPlane)) return;
			const FModel::FPlane& Plane = Built->Model.SupportPlanes[DragPlane];
			const double DirA = RayDir[Plane.Axis];
			if (FMath::Abs(DirA) < 1e-9) return;   // looking along the plane: no point to drag to
			const double T = (Plane.Value - RayOrigin[Plane.Axis]) / DirA;
			if (T <= 0.0) return;
			const FVector Hit = RayOrigin + RayDir * T;
			TSharedPtr<FSupportRow> Row = SupportRows[DragPlane];
			const int32 U = AerotelicStructuralFea::FSupport::InPlaneAxis(Plane.Axis, 0);
			const int32 W = AerotelicStructuralFea::FSupport::InPlaneAxis(Plane.Axis, 1);
			const double u = Full(Hit[U]), w = Full(Hit[W]);
			// Corners 0..3: lo/lo, hi/lo, hi/hi, lo/hi; edges 4..7: u lo, u hi, w lo, w hi.
			const bool bU = DragKind <= 3 || DragKind == 4 || DragKind == 5;
			const bool bW = DragKind <= 3 || DragKind == 6 || DragKind == 7;
			const bool bULo = DragKind == 0 || DragKind == 3 || DragKind == 4;
			const bool bWLo = DragKind == 0 || DragKind == 1 || DragKind == 6;
			if (bU) { if (bULo) Row->Lo[0] = FMath::Min(u, Row->Hi[0]); else Row->Hi[0] = FMath::Max(u, Row->Lo[0]); }
			if (bW) { if (bWLo) Row->Lo[1] = FMath::Min(w, Row->Hi[1]); else Row->Hi[1] = FMath::Max(w, Row->Lo[1]); }
			// Live: the fixed nodes by the pre-processor's rule, redrawn.
			TArray<FModel::FPlane> Planes = LivePlanes();
			const TArray<int32> Nodes = LiveSupportNodes(Planes);
			UpdateSupportPicture(Planes, Nodes, true);
		}

		// Release: the full derivation (the request, every count, the
		// captions), the view left where it is.
		void OnPatchUp()
		{
			if (DragPlane < 0) return;
			DragPlane = DragKind = -1;
			Derive(false);
		}

		// Fit to body: the patch becomes the chosen body's extent on the
		// plane (the whole mesh's if none), with a small margin.
		void FitPatch(TSharedPtr<FSupportRow> Row)
		{
			if (!Built.IsValid()) { Status(LOCTEXT("NotBuilt", "Build the model first (Set-up tab); the loads and supports are then applied to it.")); return; }
			const FBuilt& B = *Built;
			const int32 A = FMath::Clamp(Row->Axis, 0, 2);
			aerotelic::Box3 Box;
			const int32 Body = Row->Body > 0 ? Row->Body - 1 : -1;
			if (Body >= 0)
			{
				TSet<int32> Regions;
				for (const AerotelicStructuralFea::FRegion& R : B.Request.Regions) if (R.Body == Body) Regions.Add(R.Id);
				for (size_t t = 0; t < B.Mesh.tets.size(); ++t)
				{
					const int32 Region = t < B.Mesh.material.size() ? B.Mesh.material[t] : 1;
					if (!Regions.Contains(Region)) continue;
					for (int32 k = 0; k < 4; ++k) Box.Add(B.Mesh.nodes[B.Mesh.tets[t][k]]);
				}
				for (const AerotelicStructuralFea::FShellFace& F : B.Request.ShellFaces)
				{
					if (!Regions.Contains(F.Region)) continue;
					for (int32 k = 0; k < 3; ++k) Box.Add(B.Mesh.nodes[F.N[k]]);
				}
				for (const AerotelicStructuralFea::FShellFace& F : B.Request.RibFaces)
				{
					if (!Regions.Contains(F.Region)) continue;
					for (int32 k = 0; k < 3; ++k) Box.Add(B.Mesh.nodes[F.N[k]]);
				}
			}
			if (Box.IsEmpty()) Box = B.Model.Bounds;
			const double Margin = 0.01 * FMath::Max(B.Model.Diagonal, 1e-9);
			const double MinV[3] = { Box.min.x, Box.min.y, Box.min.z };
			const double MaxV[3] = { Box.max.x, Box.max.y, Box.max.z };
			Row->bBounded = true;
			for (int32 i = 0; i < 2; ++i)
			{
				const int32 Ax = AerotelicStructuralFea::FSupport::InPlaneAxis(A, i);
				Row->Lo[i] = Full(MinV[Ax] - Margin);
				Row->Hi[i] = Full(MaxV[Ax] + Margin);
			}
			// The card's fields read the row live; only the model moves.
			Derive(false);
		}

		// Apply on the Loads or Support tab: the inputs onto the built model.
		void ApplyInputs()
		{
			if (!Built.IsValid())
			{
				Status(LOCTEXT("NotBuilt", "Build the model first (Set-up tab); the loads and supports are then applied to it."));
				return;
			}
			if (Derive())
			{
				const FModel& M = Built->Model;
				int32 Coarse = 0, Far = 0;
				for (const FModel::FPlane& P : M.SupportPlanes) Coarse += M.ToleranceIsCoarse(P) ? 1 : 0;
				for (const FModel::FPointLoad& P : M.PointLoads) Far += M.SnapIsFar(P) ? 1 : 0;
				Status(FText::FromString(FString::Printf(TEXT("Applied: %d support plane(s) fixing %s nodes, %d point force(s)%s%s%s"),
					M.SupportPlanes.Num(), *Thousands(M.SupportNodes.Num()), M.PointLoads.Num(),
					Built->Dropped.Num() ? *FString::Printf(TEXT("; %d part(s) left out for floating, see the Support tab"), Built->Dropped.Num()) : TEXT(""),
					Coarse ? *FString::Printf(TEXT("; WARNING: %d support tolerance(s) wider than the mesh's edges, see the Support tab"), Coarse) : TEXT(""),
					Far ? *FString::Printf(TEXT("; WARNING: %d point force(s) landed more than a mesh edge from the point typed, see the Loads tab"), Far) : TEXT(""))));
			}
		}

		// ---- the pictures ----

		AAerotelicColoredMeshActor* EnsureActor(int32 Tab, int32 Index)
		{
			TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>>& List = Actors[Tab];
			while (List.Num() <= Index) List.Add(nullptr);
			if (List[Index].IsValid()) return List[Index].Get();
			AAerotelicCameraPawn* Pawn = AAerotelicCameraPawn::FindInGameWorld();
			UWorld* World = Pawn != nullptr ? Pawn->GetWorld() : nullptr;
			if (World == nullptr) return nullptr;
			FActorSpawnParameters Params;
			Params.ObjectFlags |= RF_Transient;
			AAerotelicColoredMeshActor* A = World->SpawnActor<AAerotelicColoredMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
			List[Index] = A;
			return A;
		}

		void ShowPicture(int32 Tab, const TArray<FGeom*>& Geoms, const TArray<bool>& Edges)
		{
			if (!Views[Tab].IsValid()) return;
			TArray<AActor*> Shown;
			for (int32 i = 0; i < Geoms.Num(); ++i)
			{
				AAerotelicColoredMeshActor* A = EnsureActor(Tab, i);
				if (A == nullptr) return;
				A->SetMesh(Geoms[i]->V, Geoms[i]->T, Geoms[i]->N, Geoms[i]->C);
				A->SetEdges(Edges.IsValidIndex(i) && Edges[i]);
				Shown.Add(A);
			}
			for (int32 i = Geoms.Num(); i < Actors[Tab].Num(); ++i)
			{
				if (Actors[Tab][i].IsValid()) Actors[Tab][i]->Clear();
			}
			Views[Tab]->SetShownActors(Shown);
			if (!bFramed[Tab])
			{
				Views[Tab]->FrameShown();
				bFramed[Tab] = true;
			}
			Views[Tab]->RequestRedraw();
		}

		void UpdatePictures()
		{
			if (!Built.IsValid()) return;
			const FBuilt& B = *Built;
			const FModel& M = B.Model;
			const aerotelic::TetMesh& Mesh = B.Mesh;
			const double Diag = FMath::Max(M.Diagonal, 1e-9);
			auto At = [&Mesh](int32 Node) { const aerotelic::Vec3& P = Mesh.nodes[Node]; return FVector(P.x, P.y, P.z); };
			auto Surface = [&](TFunction<FColor(const FFace&)> ColorOf)
			{
				FGeom G;
				G.V.Reserve(M.WettedFaces.Num() * 6);
				for (const FFace& F : M.WettedFaces) G.Tri(At(F.N[0]), At(F.N[1]), At(F.N[2]), ColorOf(F));
				return G;
			};
			TMap<int32, int32> RegionIndex;
			for (int32 i = 0; i < M.Regions.Num(); ++i) RegionIndex.Add(M.Regions[i].Id, i);
			const FString LengthUnit = UnitName(Units.LengthName());
			const FString StressUnit = UnitName(Units.StressName());
			const double S2 = Units.StressToSI();

			// Mesh: the wetted faces by region, with their edges; the rib
			// shells inside (their region's colour too, drawn through the
			// surface by the two-sided material).
			{
				FGeom G = Surface([&](const FFace& F) { const int32* I = RegionIndex.Find(F.Region); return RegionColor(I ? *I : 0); });
				for (const FFace& F : M.RibFaces)
				{
					const int32* I = RegionIndex.Find(F.Region);
					G.Tri(At(F.N[0]), At(F.N[1]), At(F.N[2]), RegionColor(I ? *I : 0));
				}
				ShowPicture(TabMesh, { &G }, { true });
				FString Text = FString::Printf(TEXT("%s nodes, %s tetrahedra (%s of them 10-node), %s wetted faces (their edges drawn)%s%s"),
					*Thousands(M.Nodes), *Thousands(M.Tets), *Thousands(M.SecondOrderTets), *Thousands(M.WettedFaces.Num()),
					M.RibFaces.Num() > 0 ? *FString::Printf(TEXT(", %s rib shells inside"), *Thousands(M.RibFaces.Num())) : TEXT(""),
					M.SecondOrderTets > 0 ? TEXT("; the solver adds a mid-edge node on every edge of the 10-node tetrahedra, not drawn") : TEXT(""));
				Text += TEXT("\nCost: ") + CostText(M);
				Text += FString::Printf(TEXT("\n%s: %s"), M.MeshIsPoor() ? TEXT("QUALITY, POOR") : TEXT("Quality"), *QualityText(M));
				for (int32 i = 0; i < M.Regions.Num(); ++i)
				{
					const AerotelicFeaPreprocess::FRegionSummary& R = M.Regions[i];
					Text += FString::Printf(TEXT("\nregion %d, %s (%s): %s %s tetrahedra, %s faces%s, volume %.4g %s^3, area %.4g %s^2"),
						R.Id, *R.Label, *R.Kind, *Thousands(R.Tets), R.Tets > 0 ? (R.bSecondOrder ? TEXT("10-node") : TEXT("4-node")) : TEXT(""), *Thousands(R.Faces),
						R.RibFaces > 0 ? *FString::Printf(TEXT(", %s rib shells of %.4g %s^2"), *Thousands(R.RibFaces), R.RibArea, *LengthUnit) : TEXT(""), R.Volume, *LengthUnit, R.Area, *LengthUnit);
				}
				Text += TEXT("\nThe faces are each body's OWN grid: the triangles its Mesh tab defines and the viewport shows, cut along the rib planes where a body has plane networks so the ribs' rims are the skin's nodes. A body's infill lends its material card only.");
				for (const FString& D : B.Dropped) Text += TEXT("\nLEFT OUT: ") + D;
				if (Captions[TabMesh].IsValid()) Captions[TabMesh]->SetText(FText::FromString(Text));
			}

			// Loads: the surface in grey, the forces as arrows whose heads
			// sit where they act.
			{
				FGeom G = Surface([&](const FFace&) { return kGrey; });
				FGeom Arrows;
				FString Text;
				const int32 NFaces = M.WettedFaces.Num();
				int32 Drawn = 0;
				if (M.MaxFacePressure > 0.0)
				{
					const int32 Stride = FMath::Max(1, FMath::DivideAndRoundUp(NFaces, 1500));
					const double Radius = 0.002 * Diag;
					for (int32 i = 0; i < NFaces; i += Stride)
					{
						const double P = M.FacePressure[i];
						if (P == 0.0) continue;
						const FVector Dir = P > 0.0 ? -M.FaceNormal[i] : M.FaceNormal[i];
						AddArrow(Arrows, M.FaceCentre[i], Dir, 0.025 * Diag * FMath::Abs(P) / M.MaxFacePressure, Radius, kPressure);
						++Drawn;
					}
					Text += FString::Printf(TEXT("Pressure (orange): %.4g %s on every one of the %s wetted faces, %s; resultant (%.4g, %.4g, %.4g) N -- arrows on %s of them, the traction's direction and relative size, not the face force"),
						B.Request.Pressure / S2, *StressUnit, *Thousands(NFaces), B.Request.Pressure > 0.0 ? TEXT("pressing inward") : TEXT("pulling outward"),
						M.PressureResultant.X, M.PressureResultant.Y, M.PressureResultant.Z, *Thousands(Drawn));
				}
				else
				{
					Text += TEXT("Pressure: none");
				}
				if (M.PointLoads.Num() > 0)
				{
					for (int32 i = 0; i < M.PointLoads.Num(); ++i)
					{
						const FModel::FPointLoad& P = M.PointLoads[i];
						AddArrow(Arrows, P.At, P.Force, 0.12 * Diag, 0.005 * Diag, kPointForce);
						const FVector NodeAt = Full(P.At);
						Text += FString::Printf(TEXT("\nPoint force %d (red): (%.4g, %.4g, %.4g) N at node %d, (%.4g, %.4g, %.4g) %s -- the corner node nearest the point asked for"),
							i + 1, P.Force.X, P.Force.Y, P.Force.Z, P.Node, NodeAt.X, NodeAt.Y, NodeAt.Z, *LengthUnit);
						if (M.SnapIsFar(P))
						{
							const FVector Asked = Full(P.Asked);
							Text += FString::Printf(TEXT("\n  WARNING: the point asked for, (%.4g, %.4g, %.4g) %s, is %.4g %s from that node -- more than a mesh edge (median %.4g %s): the load is NOT where it was typed. Check the coordinates%s or refine the mesh."),
								Asked.X, Asked.Y, Asked.Z, *LengthUnit, Full(P.SnapDistance), *LengthUnit, Full(M.MedianEdge), *LengthUnit,
								IsReplica() ? TEXT(" (full-scale, as the parameterisations are)") : TEXT(""));
						}
					}
					if (M.PointLoads.Num() > 1) Text += FString::Printf(TEXT("\nPoint forces together: (%.4g, %.4g, %.4g) N"), M.PointResultant.X, M.PointResultant.Y, M.PointResultant.Z);
				}
				else
				{
					Text += TEXT("\nPoint forces: none");
				}
				if (B.Request.LoadFactor != 0.0 && M.Mass > 0.0)
				{
					AddArrow(Arrows, M.MassCentre, M.GravityForce, 0.12 * Diag, 0.005 * Diag, kGravity);
					const FVector Centre = Full(M.MassCentre);
					Text += FString::Printf(TEXT("\nInertial load (green): %.4g g on the model's mass of %.5g kg%s: (%.4g, %.4g, %.4g) N, drawn at the centre of mass (%.4g, %.4g, %.4g) %s; applied as a body force on every element"),
						B.Request.LoadFactor, M.Mass, M.bMassEstimated ? TEXT(" (a homogenised core's density estimated from the cell's solid fraction, before homogenisation)") : TEXT(""),
						M.GravityForce.X, M.GravityForce.Y, M.GravityForce.Z, Centre.X, Centre.Y, Centre.Z, *LengthUnit);
				}
				else
				{
					Text += TEXT("\nInertial load: none (load factor 0)");
				}
				Text += TEXT("\nArrow heads sit at the point of application; forces are newtons whatever the project's units. Edit the loads on the left and Apply.");
				if (IsReplica())
				{
					Text += FString::Printf(TEXT("\nCoordinates here and on the left are the FULL-SCALE aircraft's; the model is built at %s of that size (Model > Scale) and the picture shows the replica."), *ScaleText());
				}
				ShowPicture(TabLoads, { &G, &Arrows }, { false, false });
				if (Captions[TabLoads].IsValid()) Captions[TabLoads]->SetText(FText::FromString(Text));
			}

			// Support: the surface with its edges, a marker on every fixed
			// node, and each plane as a frame with its patch handles.
			UpdateSupportPicture(M.SupportPlanes, M.SupportNodes, false);
		}

		// The Support picture from a set of planes and the nodes they fix:
		// the derived model's (bLive false) or the rows' as a handle is
		// dragged (bLive true: the counts are provisional until release).
		void UpdateSupportPicture(const TArray<FModel::FPlane>& Planes, const TArray<int32>& SupportNodes, bool bLive)
		{
			if (!Built.IsValid()) return;
			const FBuilt& B = *Built;
			const FModel& M = B.Model;
			const aerotelic::TetMesh& Mesh = B.Mesh;
			const double Diag = FMath::Max(M.Diagonal, 1e-9);
			auto At = [&Mesh](int32 Node) { const aerotelic::Vec3& P = Mesh.nodes[Node]; return FVector(P.x, P.y, P.z); };
			const FString LengthUnit = UnitName(Units.LengthName());
			// The surface is rebuilt only for a full redraw: during a drag the
			// markers' actor alone is refreshed, so a large mesh follows the
			// mouse.
			const bool bMarksOnly = bLive && Actors[TabSupport].IsValidIndex(1) && Actors[TabSupport][1].IsValid() && Views[TabSupport].IsValid();
			FGeom G;
			if (!bMarksOnly)
			{
				G.V.Reserve(M.WettedFaces.Num() * 6);
				for (const FFace& F : M.WettedFaces) G.Tri(At(F.N[0]), At(F.N[1]), At(F.N[2]), kGrey);
			}
			FGeom Marks;
			const int32 NSupport = SupportNodes.Num();
			const int32 Stride = FMath::Max(1, FMath::DivideAndRoundUp(NSupport, 20000));
			const double R = 0.005 * Diag;
			for (int32 i = 0; i < NSupport; i += Stride) AddOctahedron(Marks, At(SupportNodes[i]), R, kFixed);
			const TCHAR AxisName[3] = { TEXT('x'), TEXT('y'), TEXT('z') };
			FString Text;
			Handles.Reset();
			for (int32 k = 0; k < Planes.Num(); ++k)
			{
				const FModel::FPlane& Plane = Planes[k];
				const int32 A = Plane.Axis, U = (A + 1) % 3, W = (A + 2) % 3;
				const bool bBounded = SupportRows.IsValidIndex(k) && SupportRows[k]->bBounded;
				double Lo[2], Hi[2];
				PlaneRect(k, M, Lo, Hi);
				FVector P[4];
				for (int32 c = 0; c < 4; ++c)
				{
					P[c][A] = Plane.Value;
					P[c][U] = (c == 1 || c == 2) ? Hi[0] : Lo[0];
					P[c][W] = (c >= 2) ? Hi[1] : Lo[1];
				}
				for (int32 c = 0; c < 4; ++c) AddBar(Marks, P[c], P[(c + 1) % 4], 0.002 * Diag, bBounded ? kHandle : kPlane);
				// The handles: the corners, then the edges' midpoints (u lo
				// between corners 3 and 0, u hi between 1 and 2, w lo between
				// 0 and 1, w hi between 2 and 3).
				const FVector Centres[8] = { P[0], P[1], P[2], P[3], (P[3] + P[0]) * 0.5, (P[1] + P[2]) * 0.5, (P[0] + P[1]) * 0.5, (P[2] + P[3]) * 0.5 };
				for (int32 h = 0; h < 8; ++h)
				{
					FPatchHandle Handle;
					Handle.Plane = k; Handle.Kind = h; Handle.World = Centres[h];
					const bool bHot = (DragPlane == k && DragKind == h) || (Handles.Num() == HoverHandle);
					AddOctahedron(Marks, Centres[h], (h < 4 ? 0.012 : 0.009) * Diag, bHot ? kHandleHot : kHandle);
					Handles.Add(Handle);
				}
				FString Of;
				if (Plane.Body >= 0)
				{
					const FString BodyName = BodyOptions.IsValidIndex(Plane.Body + 1) ? *BodyOptions[Plane.Body + 1] : FString::FromInt(Plane.Body);
					Of = Plane.bBodyFallback
						? FString::Printf(TEXT(" (asked of %s, which has no nodes in the model: the whole mesh's extreme was used)"), *BodyName)
						: FString::Printf(TEXT(" (the extreme of %s alone)"), *BodyName);
				}
				const FString Patch = bBounded
					? FString::Printf(TEXT(" limited to the patch %c in [%.4g, %.4g], %c in [%.4g, %.4g] %s (amber frame; drag its handles)"),
						AxisName[U], Full(Lo[0]), Full(Hi[0]), AxisName[W], Full(Lo[1]), Full(Hi[1]), *LengthUnit)
					: TEXT(" over the whole plane (blue frame; drag a handle to limit it to a patch)");
				Text += FString::Printf(TEXT("%sSupport %d: the plane %c = %.6g %s%s,%s: %s nodes within %.3g %s of it%s; the furthest of them %.3g %s from the plane%s"),
					k ? TEXT("\n") : TEXT(""), k + 1, AxisName[A], Full(Plane.Value), *LengthUnit, *Of, *Patch, *Thousands(Plane.Nodes), Full(Plane.Tolerance), *LengthUnit,
					B.Request.Supports.IsValidIndex(k) && B.Request.Supports[k].Tolerance > 0.0 ? TEXT("") : TEXT(" (a ten-thousandth of the mesh's diagonal)"),
					Full(Plane.Furthest), *LengthUnit, bLive ? TEXT(" (dragging: provisional until release)") : TEXT(""));
				if (M.ToleranceIsCoarse(Plane))
				{
					Text += FString::Printf(TEXT("\n  WARNING: that tolerance is more than a quarter of the mesh's edge length (median %.3g %s): it can clamp a second ring of nodes and shorten the part. Narrow it to the plane's own flatness."),
						Full(M.MedianEdge), *LengthUnit);
				}
			}
			if (Planes.Num() == 0) Text += TEXT("No support: add a plane on the left and Apply.");
			if (IsReplica())
			{
				Text += FString::Printf(TEXT("\nCoordinates here and on the left are the FULL-SCALE aircraft's; the model is built at %s of that size (Model > Scale) and the picture shows the replica."), *ScaleText());
			}
			Text += FString::Printf(TEXT("\nFixed nodes (red): %s of %s, every degree of freedom -- the three displacements and, on the shells, the three rotations: a clamp"),
				*Thousands(NSupport), *Thousands(M.Nodes));
			if (Stride > 1) Text += FString::Printf(TEXT(" (one marker in %d drawn)"), Stride);
			if (NSupport < 3) Text += TEXT("\nFEWER THAN THREE NODES: the solver refuses this support. Move a plane onto the geometry (or use the mesh's minimum / maximum), widen the tolerance or enlarge the patch.");
			if (M.SecondOrderTets > 0) Text += TEXT("\nThe mid-edge nodes the solver adds between two fixed nodes are fixed with them.");
			if (!bLive)
			{
				if (B.Dropped.Num() > 0)
				{
					Text += TEXT("\nLEFT OUT, no load path to any support (they would make the system singular):");
					for (const FString& D : B.Dropped) Text += TEXT("\n  ") + D;
				}
				else
				{
					Text += FString::Printf(TEXT("\nLoad path: every element reaches a support (%d connected piece%s)."), M.Components, M.Components == 1 ? TEXT("") : TEXT("s"));
				}
			}
			Text += TEXT("\nEdit the supports on the left and Apply, or drag a patch's corner or edge handles here (the fixed nodes follow as you drag; the model is re-derived when you let go).");
			if (bMarksOnly)
			{
				AAerotelicColoredMeshActor* A = Actors[TabSupport][1].Get();
				A->SetMesh(Marks.V, Marks.T, Marks.N, Marks.C);
				A->SetEdges(false);
				Views[TabSupport]->RequestRedraw();
			}
			else
			{
				ShowPicture(TabSupport, { &G, &Marks }, { true, false });
			}
			if (Captions[TabSupport].IsValid()) Captions[TabSupport]->SetText(FText::FromString(Text));
		}

		// ---- the solve ----

		void Solve()
		{
			if (IsSolving())
			{
				Message(LOCTEXT("Busy", "A solve is running: wait for it, or Cancel it."));
				return;
			}
			if (!PathBox.IsValid() || PathBox->GetText().ToString().TrimStartAndEnd().IsEmpty())
			{
				Message(LOCTEXT("NoPath", "Name the file the solution is to be written to."));
				return;
			}
			if (!BuildModel()) return;
			const FBuilt& B = *Built;
			const FModel& M = B.Model;
			if (M.SupportNodes.Num() < 3)
			{
				Message(FText::FromString(FString::Printf(TEXT("Only %d node(s) lie on the support plane(s); a support needs a face of them. See the Support tab: move a plane onto the geometry (or use the mesh's minimum / maximum) or widen the tolerance.%s"), M.SupportNodes.Num(),
					IsReplica() ? TEXT(" Coordinates are typed full-scale; the dialog scales them to the replica.") : TEXT(""))));
				if (Tabs.IsValid()) Tabs->SetActiveTab(TabSupport);
				return;
			}
			// A point force that landed more than a mesh edge from its point
			// is not where it was typed: say so before the solver runs on it.
			int32 Far = 0;
			for (const FModel::FPointLoad& P : M.PointLoads) Far += M.SnapIsFar(P) ? 1 : 0;
			if (Far > 0 && !bSmoke)
			{
				const EAppReturnType::Type Answer = AerotelicMessageBox(EAppMsgType::YesNo,
					FText::FromString(FString::Printf(TEXT("%d point force(s) landed more than a mesh edge from the point typed -- see the Loads tab for where each went. Solve anyway?"), Far)),
					LOCTEXT("Title", "Structural (Kratos FEA)"));
				if (Answer != EAppReturnType::Yes)
				{
					if (Tabs.IsValid()) Tabs->SetActiveTab(TabLoads);
					return;
				}
			}
			// The cost, before anything runs: past the direct solver's budget
			// the user decides, having been told what will happen.
			if (OverBudget(M) && !bSmoke)
			{
				const bool bDirectByName = SolverIndex == 1;
				const EAppReturnType::Type Answer = AerotelicMessageBox(EAppMsgType::YesNo,
					FText::FromString(FString::Printf(TEXT("This model is %s: %s.\n\n%s\n\nSolve anyway?"),
						*CostText(M),
						bDirectByName
							? TEXT("the direct solver was chosen by name, and at this size it can take hours on one thread or run the machine out of memory (a 540,000-DOF fuselage exhausted 32 GB and the page file)")
							: TEXT("the iterative solver (amgcl) will run instead, on every core; it can stop short on thin shells, which the force balance will report afterwards"),
						TEXT("A faster, exact solve: coarsen the bodies' Mesh tab grids (the shells and the tetrahedra are built on them), or untick the second-order tetrahedra."))),
					LOCTEXT("Title", "Structural (Kratos FEA)"));
				if (Answer != EAppReturnType::Yes) return;
			}
			// The mesh's shape: past the sliver band the user decides too.
			if (M.MeshIsPoor() && !bSmoke)
			{
				const EAppReturnType::Type Answer = AerotelicMessageBox(EAppMsgType::YesNo,
					FText::FromString(FString::Printf(TEXT("The mesh is poor: %s.\n\nThe solve will run, but the stresses in those elements are not to be trusted and the iterative solver may stop short on them.\n\nSolve anyway?"), *QualityText(M))),
					LOCTEXT("Title", "Structural (Kratos FEA)"));
				if (Answer != EAppReturnType::Yes) return;
			}

			// Off the game thread: the script runs for seconds to minutes
			// and the window must keep painting (and offering Cancel).
			TSharedPtr<FSolveJob> J = MakeShared<FSolveJob>();
			J->Built = Built;
			J->Started = FPlatformTime::Seconds();
			{
				TWeakPtr<FSolveJob> WeakJob = J;
				J->Progress.OnLine = [WeakJob](const FString& Line)
				{
					TSharedPtr<FSolveJob> P = WeakJob.Pin();
					if (!P.IsValid()) return;
					FScopeLock L(&P->Lock);
					P->LastLine = Line;
				};
			}
			Job = J;
			SetSolving(true);
			ShowProgress(FString::Printf(TEXT("starting the solver on %s tetrahedra and %s wetted faces"), *Thousands((int64)B.Mesh.tets.size()), *Thousands(M.WettedFaces.Num())));
			Async(EAsyncExecution::Thread, [J]()
			{
				J->bOk = AerotelicStructuralFea::Run(J->Built->Request, J->Result, J->Error, &J->Progress);
				J->bDone = true;
			});
			TWeakPtr<FState> Weak = AsShared();
			J->Ticker = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Weak, J](float)
			{
				TSharedPtr<FState> S = Weak.Pin();
				if (!S.IsValid()) return false;
				if (J->IsRunning())
				{
					S->ShowProgress(J->TakeLastLine());
					return true;
				}
				S->FinishSolve(J);
				return false;
			}), 0.25f);
		}

		// The dialog while a solve runs: Build and Solve quiet, Cancel live,
		// the marquee on.
		void SetSolving(bool bOn)
		{
			if (BuildButton.IsValid()) BuildButton->SetEnabled(!bOn);
			if (SolveButton.IsValid()) SolveButton->SetEnabled(!bOn);
			if (CancelButton.IsValid()) CancelButton->SetEnabled(bOn);
			if (ProgressBar.IsValid()) ProgressBar->SetVisibility(bOn ? EVisibility::Visible : EVisibility::Collapsed);
			if (LblProgress.IsValid()) LblProgress->SetVisibility(bOn ? EVisibility::Visible : EVisibility::Collapsed);
		}

		void ShowProgress(const FString& Line)
		{
			const double Elapsed = Job.IsValid() ? FPlatformTime::Seconds() - Job->Started : 0.0;
			const FString Text = FString::Printf(TEXT("Solving, %.0f s: %s"), Elapsed, Line.IsEmpty() ? TEXT("...") : *Line);
			Status(FText::FromString(Text));
			if (LblProgress.IsValid()) LblProgress->SetText(FText::FromString(Text));
		}

		void CancelSolve()
		{
			if (!IsSolving()) return;
			Job->Progress.bCancel = true;
			ShowProgress(TEXT("cancelling..."));
		}

		void FinishSolve(TSharedPtr<FSolveJob> J)
		{
			// Called from the ticker itself, which ends by returning false:
			// the handle is only forgotten here.
			J->Ticker.Reset();
			if (Job == J) Job.Reset();
			SetSolving(false);
			if (!J->bOk)
			{
				if (J->Error == TEXT("cancelled"))
				{
					Status(LOCTEXT("Cancelled", "Solve cancelled."));
				}
				else
				{
					Status(LOCTEXT("Ready", "Ready"));
					AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(J->Error), LOCTEXT("Failed", "Structural Analysis Failed"));
				}
				return;
			}
			if (J->Built.IsValid()) ShowResult(*J->Built, J->Result);
		}

		// The result into the Set-up tab's Result group and the status bar.
		void ShowResult(const FBuilt& B, const AerotelicStructuralFea::FResult& R)
		{
			const FModel& M = B.Model;
			if (!LblMesh.IsValid()) return;   // the window closed while the solver ran
			const double L2 = Units.LengthToSI(), S2 = Units.StressToSI(), M2 = Units.MassToSI();
			LblMesh->SetText(FText::FromString(FString::Printf(TEXT("%s nodes; %s tetrahedra (%s) + %s shells%s; %s DOF; %s wetted faces; %s nodes on the supports"),
				*Thousands(R.Nodes), *Thousands(R.Tets), R.bSecondOrder ? (M.SecondOrderTets == M.Tets ? TEXT("10-node") : TEXT("10-node solids, 4-node cores")) : TEXT("4-node"),
				*Thousands(R.Shells), R.RibShells > 0 ? *FString::Printf(TEXT(" (%s of them rib shells, unloaded)"), *Thousands(R.RibShells)) : TEXT(""), *Thousands(R.Dofs), *Thousands(R.WettedFaces), *Thousands(R.SupportNodes))));
			FString Core;
			for (const AerotelicStructuralFea::FRegionResult& G : R.Regions)
			{
				const AerotelicStructuralFea::FRegion* Spec = B.Request.Regions.FindByPredicate([&G](const AerotelicStructuralFea::FRegion& X) { return X.Id == G.Id; });
				const double E = Spec != nullptr ? Spec->Core.youngs_modulus : 1.0;
				Core += FString::Printf(TEXT("%s%s: relative density %.3f; E/E_s = (%.3f, %.3f, %.3f), G/E_s = (%.3f, %.3f, %.3f), nu_xy %.2f; strut stress amplification bound %.1f; core density used %.4g %s/%s^3; homogenised in %.1f s%s"),
					Core.IsEmpty() ? TEXT("") : TEXT("\n"), *G.Label, G.RelativeDensity, G.E[0] / E, G.E[1] / E, G.E[2] / E, G.G[0] / E, G.G[1] / E, G.G[2] / E, G.NuXY,
					G.Amplification, G.CoreDensityUsed / M2 * L2 * L2 * L2, *UnitName(Units.MassName()), *UnitName(Units.LengthName()),
					G.HomogenizeSeconds, G.bHomogenizeCached ? TEXT(" (from the cache)") : TEXT(""));
			}
			LblCore->SetText(Core.IsEmpty() ? LOCTEXT("NoHomog", "(no lattice-filled body: nothing homogenised)") : FText::FromString(Core));
			LblDisp->SetText(FText::FromString(FString::Printf(TEXT("%.4g %s at (%.4g, %.4g, %.4g); mean surface displacement %s (u . n = %.3g %s)"),
				R.MaxDisplacement / L2, *UnitName(Units.LengthName()), R.MaxDisplacementAt[0] / L2, R.MaxDisplacementAt[1] / L2, R.MaxDisplacementAt[2] / L2,
				R.MeanNormalDisplacement < 0.0 ? TEXT("inward") : TEXT("outward"), R.MeanNormalDisplacement / L2, *UnitName(Units.LengthName()))));
			FString Peaks;
			for (const AerotelicStructuralFea::FEntryResult& E : R.Entries)
			{
				const FString Macro = E.Kind == TEXT("core") ? FString::Printf(TEXT(" (estimated in the struts; the smeared core stress is %.4g %s)"), E.MacroVonMises / S2, *UnitName(Units.StressName())) : FString();
				Peaks += FString::Printf(TEXT("%s%s [%s]: %.4g %s%s%s"), Peaks.IsEmpty() ? TEXT("") : TEXT("\n"), *E.Label, *E.Kind, E.MaxVonMises / S2, *UnitName(Units.StressName()), *Macro,
					E.Allowable > 0.0 ? *FString::Printf(TEXT(", margin %+.2f against %.4g"), E.Margin, E.Allowable / S2) : TEXT(" (no allowable set)"));
			}
			LblStress->SetText(FText::FromString(FString::Printf(TEXT("%.4g %s, in region %d"), R.MaxVonMises / S2, *UnitName(Units.StressName()), R.MaxVonMisesRegion)));
			LblMargins->SetText(FText::FromString(Peaks));
			const FString Verdict = R.bBalanced
				? FString::Printf(TEXT("Balanced to %.2g%%."), 100.0 * R.BalanceError)
				: FString::Printf(TEXT("NOT CONVERGED: the reactions miss the applied loads by %.1f%%, so the linear solver (%s) did not converge and the displacements and stresses above mean nothing. Solve again with the direct solver."), 100.0 * R.BalanceError, *R.LinearSolver);
			LblBalance->SetText(FText::FromString(FString::Printf(
				TEXT("mass %.6g %s. Reactions (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point forces (%.4g, %.4g, %.4g) N. %s"),
				R.Mass / M2, *UnitName(Units.MassName()), R.Reaction[0], R.Reaction[1], R.Reaction[2],
				R.AppliedGravity[0], R.AppliedGravity[1], R.AppliedGravity[2], R.AppliedPressure[0], R.AppliedPressure[1], R.AppliedPressure[2],
				R.AppliedPoint[0], R.AppliedPoint[1], R.AppliedPoint[2], *Verdict)));
			// The natural frequencies: hertz whatever the project's units
			// (of the replica under a model scale, like the rest).
			if (B.Request.Modes <= 0)
			{
				LblModes->SetText(LOCTEXT("NoModes", "(not computed: set the number of natural frequencies under Model)"));
			}
			else if (R.NaturalFrequencies.Num() == 0)
			{
				LblModes->SetText(LOCTEXT("ModesFailed", "NOT COMPUTED: the eigensolve failed (the log has the solver's words); the static result above stands."));
			}
			else
			{
				FString Listed;   // not "Modes": that is the member, and a hiding local is an error here
				for (int32 k = 0; k < R.NaturalFrequencies.Num(); ++k)
				{
					Listed += FString::Printf(TEXT("%sf%d = %.4g Hz"), k ? TEXT(", ") : TEXT(""), k + 1, R.NaturalFrequencies[k]);
				}
				LblModes->SetText(FText::FromString(FString::Printf(TEXT("%s%s (%s, %.1f s; mode shapes mode_1 .. mode_%d in the VTK)"), *Listed,
					IsReplica() ? *FString::Printf(TEXT(" -- of the %s replica"), *ScaleText()) : TEXT(""), *R.Eigensolver, R.SecondsModes, R.NaturalFrequencies.Num())));
			}
			LblTime->SetText(FText::FromString(FString::Printf(TEXT("%.1f s (%.1f homogenising, %.1f solving with %s%s)"), R.Seconds, R.SecondsHomogenize, R.SecondsSolve, *R.LinearSolver,
				R.SecondsModes > 0.0 ? *FString::Printf(TEXT(", %.1f on the natural frequencies"), R.SecondsModes) : TEXT(""))));
			for (const FString& W : R.Warnings)
			{
				UE_LOG(LogTemp, Warning, TEXT("StructuralFea: %s"), *W);
			}
			if (R.SupportCornerNodes != M.SupportNodes.Num() || R.WettedFaces != M.WettedFaces.Num() || R.Dofs != M.Dofs)
			{
				UE_LOG(LogTemp, Warning, TEXT("StructuralFea: the pre-processor counted %d support nodes, %d wetted faces and %lld DOF, the solver %d, %d and %d"),
					M.SupportNodes.Num(), M.WettedFaces.Num(), M.Dofs, R.SupportCornerNodes, R.WettedFaces, R.Dofs);
			}
			Status(FText::FromString(FString::Printf(TEXT("Structural analysis solved on %s DOF (%s): max displacement %.4g %s, max von Mises %.4g %s%s -> %s%s"),
				*Thousands(R.Dofs), *R.LinearSolver, R.MaxDisplacement / L2, *UnitName(Units.LengthName()), R.MaxVonMises / S2, *UnitName(Units.StressName()),
				R.NaturalFrequencies.Num() > 0 ? *FString::Printf(TEXT(", f1 = %.4g Hz"), R.NaturalFrequencies[0]) : TEXT(""), *R.VtkPath,
				R.bBalanced ? TEXT("") : TEXT(" -- NOT CONVERGED, see the force balance"))));
			if (Tabs.IsValid()) Tabs->SetActiveTab(TabSetup);
			if (!R.bBalanced && !bSmoke)
			{
				// The one tell of a solver that stopped short: say it where it
				// cannot be missed, not only in the Result group.
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(FString::Printf(
					TEXT("The solution did not converge: the reactions at the supports miss the applied loads by %.1f%% (the linear solver was %s). The displacements and stresses it reports, and the file it wrote, are not to be trusted. Choose the direct solver (sparse_lu) under Set-up and solve again."),
					100.0 * R.BalanceError, *R.LinearSolver)), LOCTEXT("NotConverged", "Structural Analysis Did Not Converge"));
			}
		}

		// ---- the loads and supports panels ----

		TSharedRef<SWidget> NumberField(TFunction<double()> Get, TFunction<void(double)> Set, double Delta, const FText& Suffix, double Min = -1e12, double Max = 1e12)
		{
			return SNew(SAerotelicNumericEntry)
				.Value_Lambda([Get]() { return Get(); })
				.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([Set](double V, ETextCommit::Type) { Set(V); });
		}

		void RebuildPointList()
		{
			if (!PointList.IsValid()) return;
			PointList->ClearChildren();
			const FText LengthSuffix = FText::FromString(TEXT(" ") + UnitName(Units.LengthName()));
			const FText Newton = FText::FromString(TEXT(" N"));
			TSharedPtr<FState> Self = AsShared();
			for (int32 i = 0; i < PointRows.Num(); ++i)
			{
				TSharedPtr<FPointRow> Row = PointRows[i];
				auto Triple = [this, Row, &LengthSuffix, &Newton](bool bForce)
				{
					auto Field = [this, Row, bForce, &LengthSuffix, &Newton](int32 k)
					{
						return NumberField(
							[Row, bForce, k]() { return bForce ? (k == 0 ? Row->Fx : (k == 1 ? Row->Fy : Row->Fz)) : (k == 0 ? Row->X : (k == 1 ? Row->Y : Row->Z)); },
							[Row, bForce, k](double V) { (bForce ? (k == 0 ? Row->Fx : (k == 1 ? Row->Fy : Row->Fz)) : (k == 0 ? Row->X : (k == 1 ? Row->Y : Row->Z))) = V; },
							bForce ? 100.0 : 0.1, bForce ? Newton : LengthSuffix);
					};
					return SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0) [ Field(0) ]
						+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0) [ Field(1) ]
						+ SHorizontalBox::Slot().FillWidth(1) [ Field(2) ];
				};
				PointList->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
				[
					SNew(SBorder).Padding(6).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center) [ SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("Point force %d"), i + 1))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
							+ SHorizontalBox::Slot().AutoWidth()
							[ Button(LOCTEXT("Delete", "Delete"), [Self, Row]() { Self->PointRows.Remove(Row); Self->RebuildPointList(); }) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(IsReplica() ? LOCTEXT("AtFull", "At (full scale):") : LOCTEXT("At", "At:"), Triple(false), 70.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Force", "Force:"), Triple(true), 70.0f) ]
					]
				];
			}
		}

		void RebuildSupportList()
		{
			if (!SupportList.IsValid()) return;
			SupportList->ClearChildren();
			const FText LengthSuffix = FText::FromString(TEXT(" ") + UnitName(Units.LengthName()));
			TSharedPtr<FState> Self = AsShared();
			for (int32 i = 0; i < SupportRows.Num(); ++i)
			{
				TSharedPtr<FSupportRow> Row = SupportRows[i];
				auto Combo = [Self, Row](TArray<TSharedPtr<FString>> FState::*Options, int32 FSupportRow::*Index)
				{
					return SNew(SComboBox<TSharedPtr<FString>>)
						.OptionsSource(&((*Self).*Options))
						.InitiallySelectedItem(((*Self).*Options)[FMath::Clamp((*Row).*Index, 0, ((*Self).*Options).Num() - 1)])
						.OnGenerateWidget_Lambda([](TSharedPtr<FString> Item) { return SNew(STextBlock).Text(FText::FromString(*Item)); })
						.OnSelectionChanged_Lambda([Self, Row, Options, Index](TSharedPtr<FString> Item, ESelectInfo::Type)
						{
							const int32 I = ((*Self).*Options).IndexOfByKey(Item);
							if (I != INDEX_NONE) (*Row).*Index = I;
						})
						[ SNew(STextBlock).Text_Lambda([Self, Row, Options, Index]() { return FText::FromString(*((*Self).*Options)[FMath::Clamp((*Row).*Index, 0, ((*Self).*Options).Num() - 1)]); }) ];
				};
				SupportList->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
				[
					SNew(SBorder).Padding(6).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center) [ SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("Support %d: every node on the plane, clamped"), i + 1))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
							+ SHorizontalBox::Slot().AutoWidth()
							[ Button(LOCTEXT("Delete", "Delete"), [Self, Row]() { Self->SupportRows.Remove(Row); Self->RebuildSupportList(); }) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							FormRow(LOCTEXT("Plane", "Plane:"),
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0) [ Combo(&FState::AxisOptions, &FSupportRow::Axis) ]
								+ SHorizontalBox::Slot().FillWidth(1) [ Combo(&FState::PlaneOptions, &FSupportRow::Where) ], 70.0f)
						]
						// Whose minimum / maximum: the whole mesh's, or one body's.
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Of", "Of:"), Combo(&FState::BodyOptions, &FSupportRow::Body), 70.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(IsReplica() ? LOCTEXT("ValueFull", "Value (full scale):") : LOCTEXT("Value", "Value:"), NumberField([Row]() { return Row->Value; }, [Row](double V) { Row->Value = V; }, 0.1, LengthSuffix), 70.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(IsReplica() ? LOCTEXT("WithinFull", "Within (full scale):") : LOCTEXT("Within", "Within:"), NumberField([Row]() { return Row->Tolerance; }, [Row](double V) { Row->Tolerance = FMath::Max(V, 0.0); }, 0.001, LengthSuffix, 0.0), 70.0f) ]
						// The patch: a rectangle of the plane along its two in-plane
						// axes (named live from the plane's axis), or the whole plane.
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 5, 0, 2)
						[
							SNew(SCheckBox)
							.IsChecked_Lambda([Row]() { return Row->bBounded ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
							.OnCheckStateChanged_Lambda([Self, Row](ECheckBoxState State)
							{
								Row->bBounded = State == ECheckBoxState::Checked;
								if (Row->bBounded && Row->Lo[0] == Row->Hi[0] && Row->Lo[1] == Row->Hi[1]) { Self->FitPatch(Row); return; }
								if (Self->Built.IsValid()) Self->Derive(false);
							})
							[ SNew(STextBlock).Text(LOCTEXT("Patch", "Limit to a patch (drag its handles in the picture)")) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							FormRow(LOCTEXT("PatchU", "Patch, first axis:"),
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
								[ SNew(STextBlock).Text_Lambda([Row]() { return FText::FromString(FString::Printf(TEXT("%c from"), TEXT("xyz")[AerotelicStructuralFea::FSupport::InPlaneAxis(FMath::Clamp(Row->Axis, 0, 2), 0)])); }) ]
								+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 4, 0) [ NumberField([Row]() { return Row->Lo[0]; }, [Row](double V) { Row->Lo[0] = V; }, 0.1, LengthSuffix) ]
								+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("To", "to")) ]
								+ SHorizontalBox::Slot().FillWidth(1) [ NumberField([Row]() { return Row->Hi[0]; }, [Row](double V) { Row->Hi[0] = V; }, 0.1, LengthSuffix) ], 110.0f)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							FormRow(LOCTEXT("PatchW", "Patch, second axis:"),
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
								[ SNew(STextBlock).Text_Lambda([Row]() { return FText::FromString(FString::Printf(TEXT("%c from"), TEXT("xyz")[AerotelicStructuralFea::FSupport::InPlaneAxis(FMath::Clamp(Row->Axis, 0, 2), 1)])); }) ]
								+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 4, 0) [ NumberField([Row]() { return Row->Lo[1]; }, [Row](double V) { Row->Lo[1] = V; }, 0.1, LengthSuffix) ]
								+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0) [ SNew(STextBlock).Text(LOCTEXT("To", "to")) ]
								+ SHorizontalBox::Slot().FillWidth(1) [ NumberField([Row]() { return Row->Hi[1]; }, [Row](double V) { Row->Hi[1] = V; }, 0.1, LengthSuffix) ], 110.0f)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
							[ Button(LOCTEXT("FitPatch", "Fit to body"), [Self, Row]() { Self->FitPatch(Row); }) ]
							+ SHorizontalBox::Slot().AutoWidth()
							[ Button(LOCTEXT("WholePlane", "Whole plane"), [Self, Row]() { Row->bBounded = false; if (Self->Built.IsValid()) Self->Derive(false); }) ]
						]
					]
				];
			}
		}

		// ---- the smoke run ----

		void Screenshot(const FString& Name)
		{
			TSharedPtr<SWindow> W = Window.Pin();
			if (!W.IsValid()) return;
			TArray<FColor> Pixels;
			FIntVector Size;
			if (!FSlateApplication::Get().TakeScreenshot(W.ToSharedRef(), Pixels, Size) || Pixels.Num() == 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("StructuralFea smoke: screenshot %s failed"), *Name);
				return;
			}
			for (FColor& C : Pixels) C.A = 255;
			TArray64<uint8> Png;
			FImageUtils::PNGCompressImageArray(Size.X, Size.Y, Pixels, Png);
			const FString Path = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Screenshots"), Name);
			FFileHelper::SaveArrayToFile(Png, *Path);
			UE_LOG(LogTemp, Log, TEXT("StructuralFea smoke: wrote %s (%d x %d)"), *Path, Size.X, Size.Y);
		}

		void RunSmoke()
		{
			TWeakPtr<FState> Weak = AsShared();
			auto Later = [](float Seconds, TFunction<void()> Fn)
			{
				FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Fn](float) { Fn(); return false; }), Seconds);
			};
			float Delay = 3.0f;
			FParse::Value(FCommandLine::Get(), TEXT("AerotelicFeaSmokeDelay="), Delay);
			if (Tabs.IsValid()) Tabs->SetActiveTab(TabMesh);
			Later(Delay, [Weak, Later]()
			{
				TSharedPtr<FState> S = Weak.Pin();
				if (!S.IsValid()) return;
				S->Screenshot(TEXT("fea_smoke_mesh.png"));
				if (S->Tabs.IsValid()) S->Tabs->SetActiveTab(TabLoads);
				Later(2.0f, [Weak, Later]()
				{
					TSharedPtr<FState> S = Weak.Pin();
					if (!S.IsValid()) return;
					S->Screenshot(TEXT("fea_smoke_loads.png"));
					if (S->Tabs.IsValid()) S->Tabs->SetActiveTab(TabSupport);
					Later(2.0f, [Weak]()
					{
						TSharedPtr<FState> S = Weak.Pin();
						if (!S.IsValid()) return;
						S->Screenshot(TEXT("fea_smoke_support.png"));
						UE_LOG(LogTemp, Log, TEXT("StructuralFea smoke: done, quitting"));
						FPlatformMisc::RequestExit(false);
					});
				});
			});
		}

		void Close()
		{
			// A solve still running is told to stop; the worker finishes on
			// its own (it holds the built model, not the dialog).
			if (Job.IsValid())
			{
				Job->Progress.bCancel = true;
				if (Job->Ticker.IsValid()) { FTSTicker::GetCoreTicker().RemoveTicker(Job->Ticker); Job->Ticker.Reset(); }
				Job.Reset();
			}
			TetCache.Tets.Empty();
			LblProgress.Reset(); ProgressBar.Reset(); BuildButton.Reset(); SolveButton.Reset(); CancelButton.Reset();
			for (int32 Tab = 0; Tab < kPictures; ++Tab)
			{
				for (TWeakObjectPtr<AAerotelicColoredMeshActor>& A : Actors[Tab]) if (A.IsValid()) A->Destroy();
				Actors[Tab].Empty();
				Views[Tab].Reset();
				Captions[Tab].Reset();
			}
			Built.Reset();
			Picker.Reset();
			PointList.Reset(); SupportList.Reset();
			PathBox.Reset(); SecondOrderCheck.Reset(); CacheCheck.Reset();
			LblMaterials.Reset(); LblMesh.Reset(); LblCore.Reset(); LblDisp.Reset(); LblStress.Reset(); LblMargins.Reset(); LblBalance.Reset(); LblModes.Reset(); LblTime.Reset();
			Tabs.Reset();
		}
	};

	// A picture tab: an optional panel on the left (the loads, the
	// supports), the view with its caption on the right.
	TSharedRef<SWidget> MakePictureTab(TSharedRef<FState> S, int32 Tab, const FText& Empty, TSharedPtr<SWidget> SidePanel)
	{
		TSharedRef<SWidget> View =
			SAssignNew(S->Views[Tab], SAerotelicSceneView)
			.Overlay()
			[
				SNew(SOverlay)
				+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(8)
				[
					SNew(SBox).MaxDesiredWidth(640)
					[
						SNew(SBorder).Padding(FMargin(8, 6)).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.0f, 0.0f, 0.0f, 0.45f))
						[
							SAssignNew(S->Captions[Tab], STextBlock).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.92f, 0.92f, 0.92f)).Text(Empty)
						]
					]
				]
				+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Bottom).Padding(8)
				[
					SNew(STextBlock).ColorAndOpacity(FLinearColor(0.85f, 0.85f, 0.85f, 0.9f))
					.Text(LOCTEXT("ViewHint", "Left-drag orbits, right-drag pans, wheel zooms, Home frames."))
				]
			];
		if (!SidePanel.IsValid()) return View;
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(360) [ SidePanel.ToSharedRef() ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f)
			[ View ];
	}

	const TCHAR* kHelp = TEXT(
		"STRUCTURAL (KRATOS FEA)\n\n"
		"A linear static finite-element analysis of the chosen solid bodies on KratosMultiphysics' StructuralMechanicsApplication, run as a separate process in the WingDesigner environment. Everything the solver sees is SI (the mesh scaled by the length unit, the materials as they are, loads in newtons and pascals); the dialog converts the results back.\n\n"
		"BODIES\n"
		"The picker lists SOLID BODIES, not parameterisations: a wing's main element and its flap, a propeller's spinner and blades, a boolean result's separate shells are each a row. What each body is comes from its Material Properties (a boolean result inherits its first operand's). A body with a skin thickness and no lattice is a thin-walled part: shells of that thickness on its own triangles, nothing inside. A body with no skin thickness and no lattice is a solid: tetrahedra of its material at the element size. "
		"A body FILLED BY A LATTICE is said so on the Set-up tab: the lattice was not chosen and lends its material card only -- the cell to homogenise and the skin thickness. The body is then shells of that thickness on its own grid over 4-node tetrahedra at the lattice cell size carrying the lattice's homogenised stiffness, the struts of the body's material; or, at full resolution, the lattice's own skin tetrahedra as a solid over core tetrahedra with the homogenised lattice (the one model meshed on the lattice's copy of the surface, whose inner boundary is the lattice's offset); or, with 'Ignore the lattices' under Model, what its Material Properties alone say. "
		"The bodies are not joined to one another: each must reach a support on its own or through a body it was fused with. A body or part of one with no load path to any support -- a control surface in its gap, a payload bay inside the fuselage -- would make the system singular, so it is LEFT OUT automatically and named on the Mesh and Support tabs.\n\n"
		"THE GRID\n"
		"Every body is analysed on ITS OWN GRID: the triangles its Mesh tab defines, at the viewport's tolerance rule, the same ones the viewport shows and mass properties weigh. The shells are those triangles; the solids and the homogenised cores are tetrahedra filled inside them (gmsh adds interior nodes only, never a point on the surface). So the Mesh tab is where the cost of a solve is set, and the Mesh tab's count is the shell count. Until 2026-11-05 a lattice-filled body was solved on the lattice's private copy of its surface, rebuilt at a quarter of the skin thickness for the hollowing -- eighteen times finer than the fuselage drawn in the viewport -- which is what made a homogenised solve run out of memory.\n\n"
		"COST\n"
		"Build Model counts the degrees of freedom the solver will see (the script's own rule: three per node, three more per node carrying shells, a mid-edge node on every edge of the 10-node tetrahedra) and says which solver the automatic choice takes: the direct solver (exact, one thread) up to 150,000 DOF, the iterative solver (amgcl, every core; the force balance checks it afterwards) beyond. Past that budget Solve asks first. Second order (10-node tetrahedra) applies to solids and to the full-resolution skin tetrahedra, where a tetrahedron in bending needs it; a homogenised core stays 4-node whatever the box says, since it carries a material card and the shells carry the bending. For a solve in about a minute on a desktop keep the DOF in the tens of thousands: a few thousand triangles per body.\n\n"
		"QUALITY\n"
		"Build Model also measures every tetrahedron's six dihedral angles and reports the extremes and how many tetrahedra have an angle under 10 degrees or over 160 -- the acceptance band of a linear element (Shewchuk, What Is a Good Linear Finite Element?, 2002: a small dihedral angle wrecks the stiffness matrix's conditioning, a large one the accuracy of the gradient, so of the stress). More than 2 percent outside the band is reported as POOR on the Mesh tab and in the status line, and Solve asks first. The tetrahedra conform to the body's own triangles, so a poor surface makes a poor volume: a fuselage's nose and tail, where the rings shrink to a point while keeping their full count of points, are the usual cause (needle triangles with angles of a degree or two); refine or coarsen the body's grid on its Mesh tab. The interior is seeded from the surface's own triangle sizes and grows to the lattice cell, which keeps the transition layer well shaped; the gmsh and Netgen optimisation passes are always on.\n\n"
		"SET-UP\n"
		"Build Model meshes the bodies (gmsh; a body's tetrahedra are kept while the dialog is open and its grid and the cell size are unchanged) and derives what the solver will be handed for the three tabs that follow, without solving. Solve does the same and then runs Kratos on a worker thread -- the window keeps painting, the status line shows the solver's stage and the elapsed time, and Cancel ends it -- writing the displacement and stress fields to the VTK file named and the peaks, margins and force balance into the Result group; with a number of natural frequencies set under Model it then solves the same model's eigenvalue problem and lists them too, with the mode shapes written to the VTK.\n\n"
		"MESH\n"
		"The wetted faces of the model -- the boundary faces of the tetrahedra and the shell-only bodies' triangles -- coloured by region with their edges drawn. Where a region has a skin these faces ARE the shell elements. The caption counts nodes, tetrahedra (and which are 10-node) and faces, the cost, each region's kind, order, volume and area, and what was left out.\n\n"
		"LOADS\n"
		"On the left: the inertial load factor (the model's own mass under that many g along -z; 0 to begin with, so a test load acts alone), a uniform pressure on every wetted face (positive pressing inward), and any number of point forces, each at a point (the corner node nearest it takes the load) with its force in newtons -- add, edit, delete, then Apply. On the right the forces as arrows, each head at its point of application: orange the pressure as the traction on up to ~1500 faces (direction and relative size, not the face force), red each point force at its node, green the inertial load at the centre of mass.\n\n"
		"SUPPORT\n"
		"On the left: any number of support planes, each axis = value (or the mesh's minimum or maximum along the axis) with a tolerance (0 is a ten-thousandth of the mesh's diagonal, about how planar a lofted section is); every node within the tolerance of any plane is clamped in every degree of freedom (the shells' rotations too) -- add, edit, delete, then Apply. "
		"A plane may be LIMITED TO A PATCH (2026-11-05): a rectangle of it along its two in-plane axes (the other two axes in order: a plane x = c has its patch in y and z), so a clamp at x = 3 through the fuselage does not also take the wing root that crosses x = 3. Tick the patch on the card and type the rectangle, press Fit to body (the chosen body's extent on the plane, or the whole mesh's), or drag the handles in the picture: a press on a corner moves two sides, on an edge's midpoint one side; the red markers follow as you drag, and the model is re-derived when you let go (the view stays put). Dragging a handle of a whole-plane support turns it into a patch starting from the frame as drawn. On the right a red marker on every fixed node, each plane as a frame (blue for the whole plane, amber for a patch) with its eight handles, and the load path: whatever reaches no support is left out and listed.\n\n"
		"RESULT\n"
		"Linear static, small displacement, linear elastic, the linear thin shell triangle (its reactions balance the loads exactly; the corotational triangles are options for small-displacement comparisons). The reactions at the supports must balance what was applied: the Force balance line says by how much they miss, and a miss of more than one percent means the linear solver did not converge -- the solution is then reported as NOT converged and nothing in it is to be trusted (the iterative solver can do this on thin shells; the direct solver cannot, which is why the automatic choice takes it up to 150,000 degrees of freedom). Margin is allowable / peak von Mises - 1 for each region's skin, solid or core; a homogenised core's margin is against the material's allowable using the stress estimated in the struts. "
		"The natural frequencies (2026-11-04) are the lowest modes of K phi = omega^2 M phi for the same mesh, materials and clamps with no loads: the stiffness the static solve used and the elements' own mass from the densities and skin thicknesses (the core's overlap correction included). They are in hertz whatever the units; under a model scale they are the replica's. Checked on a 1 x 10 x 1 m thin-walled aluminium box: the degenerate first bending pair within 1.5% of the clamped-free beam's frequency corrected for shear, rotary inertia and the tip cap's mass, then the skin panels' own plate modes -- a thin-walled part goes local long before its second beam mode. The VTK holds the displacement, the stresses and the mode shapes (mode_1, mode_2, ... scaled to a unit largest displacement) for ParaView, in SI. No post-processing view yet.");
}

void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	TArray<FAerotelicBodyChoicePtr> Choices = AnalysableChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoBody", "Nothing to analyse: add a body first. Any body with a structural material under Material Properties can be analysed -- as shells of its skin thickness, as a solid, or over its lattice's homogenised core."),
			LOCTEXT("Title", "Structural (Kratos FEA)"));
		return;
	}
	if (AerotelicStructuralFea::FindScript().IsEmpty())
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoScript", "tools\\structural_fea.py is not installed beside the application."),
			LOCTEXT("Title", "Structural (Kratos FEA)"));
		return;
	}

	const FVector2D Size(1180, 980);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Structural (Kratos FEA)"))
		.ClientSize(Size)
		.SupportsMaximize(true)
		.SupportsMinimize(true);

	TSharedRef<FState> S = MakeShared<FState>();
	S->Status = Status;
	S->Window = Window;
	S->Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	S->Choices = Choices;
	S->ModelOptions = {
		MakeShared<FString>(TEXT("As designed: skin shells on the body's own grid, a homogenised infill core, rib shells embedded (for a desktop)")),
		MakeShared<FString>(TEXT("Full resolution: the generated structure's skin and core tetrahedra (Generate Structure first; no ribs)")),
		MakeShared<FString>(TEXT("Ignore the interior: each body as its skin and material alone say (shells or solid)")) };
	S->SolverOptions = {
		MakeShared<FString>(FString::Printf(TEXT("Automatic (direct up to %s DOF, iterative beyond)"), *Thousands(AerotelicStructuralFea::kAutoDirectDofs))),
		MakeShared<FString>(TEXT("sparse_lu (direct; exact; one thread)")),
		MakeShared<FString>(TEXT("amgcl (iterative; every core; check the force balance)")) };
	S->ShellOptions = {
		MakeShared<FString>(TEXT("Linear thin triangle (Kirchhoff; reactions balance exactly)")),
		MakeShared<FString>(TEXT("Corotational thin triangle (small displacements only)")) };
	S->AxisOptions = { MakeShared<FString>(TEXT("x")), MakeShared<FString>(TEXT("y")), MakeShared<FString>(TEXT("z")) };
	S->PlaneOptions = { MakeShared<FString>(TEXT("at the value")), MakeShared<FString>(TEXT("at the mesh's minimum")), MakeShared<FString>(TEXT("at the mesh's maximum")) };
	// Whose minimum / maximum a plane takes: the whole mesh (0), or one of
	// the picker's bodies (i + 1 for Choices[i], as FSupportRow::Body).
	S->BodyOptions.Add(MakeShared<FString>(TEXT("the whole mesh")));
	for (const FAerotelicBodyChoicePtr& C : Choices) S->BodyOptions.Add(MakeShared<FString>(C->Label));
	// One support to start with: y = 0, the root of a mirrored half.
	S->SupportRows.Add(MakeShared<FSupportRow>());

	const FText LengthSuffix = FText::FromString(TEXT(" ") + UnitName(S->Units.LengthName()));
	const FText StressSuffix = FText::FromString(TEXT(" ") + UnitName(S->Units.StressName()));

	// Chosen to start with: every body of every operand that no boolean
	// result in the scene is built on (a boolean result already contains
	// its operands; choosing both would add a second copy of each).
	TArray<FAerotelicBodyChoicePtr> InitiallyChosen;
	{
		TArray<FAerotelicBooleanOperand> Operands;
		if (Doc != nullptr)
		{
			for (const auto& Weak : Doc->GetEntities())
			{
				if (!Weak.IsValid() || Weak->GetEntityType() != EntityType::Boolean) continue;
				if (const FAerotelicBooleanModel* M = Weak->GetModelAs<FAerotelicBooleanModel>()) Operands.Append(M->Operands);
			}
		}
		for (const FAerotelicBodyChoicePtr& C : Choices)
		{
			const bool bOperand = Operands.ContainsByPredicate([&C](const FAerotelicBooleanOperand& Op) { return SameOperand(Op, C->Operand); });
			if (!bOperand) InitiallyChosen.Add(C);
		}
		if (InitiallyChosen.Num() == 0) InitiallyChosen = Choices;
	}

	auto Combo = [S](TArray<TSharedPtr<FString>> FState::*Options, int32 FState::*Index, bool bDescribe)
	{
		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&((*S).*Options))
			.InitiallySelectedItem(((*S).*Options)[(*S).*Index])
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> Item) { return SNew(STextBlock).Text(FText::FromString(*Item)); })
			.OnSelectionChanged_Lambda([S, Options, Index, bDescribe](TSharedPtr<FString> Item, ESelectInfo::Type)
			{
				const int32 I = ((*S).*Options).IndexOfByKey(Item);
				if (I != INDEX_NONE) { (*S).*Index = I; if (bDescribe) S->DescribeModel(); }
			})
			[ SNew(STextBlock).Text_Lambda([S, Options, Index]() { return FText::FromString(*((*S).*Options)[(*S).*Index]); }) ];
	};
	auto Spin = [S](double FState::*Member, double Min, double Max, double Delta, const FText& Suffix, bool bDescribe = false)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([S, Member]() { return (*S).*Member; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([S, Member, bDescribe](double V, ETextCommit::Type) { (*S).*Member = V; if (bDescribe) S->DescribeModel(); });
	};
	auto Browse = [S, Window]()
	{
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr || !S->PathBox.IsValid()) return;
		TArray<FString> Files;
		const FString Current = S->PathBox->GetText().ToString();
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Window);
		if (Desktop->SaveFileDialog(Handle, TEXT("Structural Solution"), FPaths::GetPath(Current), FPaths::GetCleanFilename(Current),
			TEXT("VTK Files (*.vtk)|*.vtk"), EFileDialogFlags::None, Files) && Files.Num() > 0)
		{
			S->PathBox->SetText(FText::FromString(FPaths::ConvertRelativePathToFull(Files[0])));
		}
	};

	// ---- tab 1: set-up ----
	TSharedRef<SWidget> SetupTab =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("BodiesGroup", "Solid bodies to analyse"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SAssignNew(S->Picker, SAerotelicBodyPicker).Choices(Choices).InitiallyChosen(InitiallyChosen).ListHeight(150.0f)
					.OnChanged(FSimpleDelegate::CreateLambda([S]() { S->DescribeModel(); }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4) [ Stat(S->LblMaterials) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("BodiesHint",
						"One row per SOLID BODY: a wing's main element and its flap, a propeller's blades, a boolean result's separate shells. Each is analysed on ITS OWN GRID -- the triangles its Mesh tab defines and the viewport shows. What each is comes from its Material Properties (a boolean result inherits its first operand's): with a skin thickness and no lattice a thin-walled part of shells; with no skin thickness a solid of its material; FILLED BY A LATTICE (said so above: the lattice was not chosen and lends its material card only), shells over a homogenised core, or as the Model group says. "
						"The bodies are not joined to one another: each must reach a support on its own, or through a body it was fused with. Whatever has no load path to any support -- a control surface in its gap, a bay inside the fuselage -- is left out automatically and named. A boolean result already contains its operands, so those start unchosen."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("ModelGroup", "Model"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Model", "Bodies with an interior as:"), Combo(&FState::ModelOptions, &FState::ModelIndex, true)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Voxels", "Voxels across the cell:"), Spin(&FState::Voxels, 8.0, 96.0, 4.0, FText::GetEmpty(), true)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("ElementSize", "Solid bodies, element size (0 = auto):"), Spin(&FState::ElementSize, 0.0, 1e9, 0.01, LengthSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					SAssignNew(S->SecondOrderCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
					.OnCheckStateChanged_Lambda([S](ECheckBoxState) { S->DescribeModel(); })
					[ SNew(STextBlock).Text(LOCTEXT("SecondOrder", "Second-order (10-node) tetrahedra for solids and skin tetrahedra (homogenised cores stay 4-node; the shells 3-node)")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					SAssignNew(S->CacheCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
					[ SNew(STextBlock).Text(LOCTEXT("Cache", "Reuse an earlier homogenisation of the same cell and material")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Solver", "Linear solver:"), Combo(&FState::SolverOptions, &FState::SolverIndex, false)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("ShellElement", "Shell element:"), Combo(&FState::ShellOptions, &FState::ShellIndex, false)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Modes", "Natural frequencies to compute (0 = none):"), Spin(&FState::Modes, 0.0, 100.0, 1.0, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ModelHint",
						"As designed: a body's OWN surface triangles (its Mesh tab's, what the viewport shows) become shell elements of its skin thickness and material; an infill becomes 4-node tetrahedra at the lattice cell size carrying the lattice's homogenised stiffness -- one unit cell, voxelised at the resolution above, solved under the six unit strains -- in the INFILL's material; each plane network becomes unloaded rib shells of its own thickness and material inside the body, the skin cut along the planes so the ribs' rims are the skin's nodes and the tetrahedra conforming to the ribs (the body is walled into compartments along them). The full-resolution model is the generated structure's skin and core tetrahedra (Generate Structure first), the skin a solid of the material and the core homogenised, no ribs; keep second order on for it. Ignore the interior analyses every body as its skin and material alone say. Solid bodies (no skin thickness) are meshed by gmsh at the element size inside their own grid. "
						"Second order promotes the solids' and skin tetrahedra to 10 nodes (a tetrahedron in bending needs it); a homogenised core stays 4-node regardless, the shells carrying the bending. Build Model reports the degrees of freedom and which solver will run: the direct solver is exact but single-threaded and its cost grows faster than the system, so the automatic choice takes it up to 150,000 degrees of freedom and the iterative solver (amgcl, every core) beyond; the force balance is checked after every solve, since the iterative solver can stop short on thin shells and hand back a solution whose reactions do not balance the loads. "
						"The linear thin triangle is the shell for a linear static analysis: its reactions balance the loads exactly whatever the displacement. The corotational triangles are geometrically nonlinear elements solved in one step: they agree with it for small displacements and report inconsistent reactions and stresses once the model moves far. "
						"The natural frequencies are the same model's -- the same mesh, materials and clamps, no loads -- solved after the static case as the eigenvalue problem K phi = omega^2 M phi for the lowest modes asked (Spectra's shift-invert eigensolver, a second or two on a few hundred thousand degrees of freedom). The mass is the elements' own, from the densities and skin thicknesses, so a thin-walled part's spectrum shows its skin panels' plate modes soon after the first beam modes."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Result", "Result"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MeshStat", "Mesh:"), Stat(S->LblMesh)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("CoreStat", "Homogenised cores:"), Stat(S->LblCore)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("DispStat", "Max displacement:"), Stat(S->LblDisp)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("StressStat", "Max von Mises:"), Stat(S->LblStress)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MarginStat", "Per region:"), Stat(S->LblMargins)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("BalanceStat", "Force balance:"), Stat(S->LblBalance)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("ModesStat", "Natural frequencies:"), Stat(S->LblModes)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("TimeStat", "Took:"), Stat(S->LblTime)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ResultHint",
						"Linear static, small displacement, linear elastic. The reactions at the supports must balance what was applied -- gravity on the mass, the pressure's resultant (zero over a closed surface), the point forces. Margin is allowable / peak von Mises - 1 for each region's skin, solid or core. A homogenised core's von Mises is a smeared average over a cell, so its margin is taken against the material's allowable using the stress ESTIMATED IN THE STRUTS. "
						"The natural frequencies are in hertz whatever the project's units, lowest first, of the structure as clamped (a replica's are the replica's). The VTK holds the displacement, the stresses and each mode shape (mode_1, mode_2, ... as point vectors scaled to a unit largest displacement) for ParaView, in SI."))
				])
		]
		+ SScrollBox::Slot()
		[
			MakeGroup(LOCTEXT("Output", "Solution file"),
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
				[ SAssignNew(S->PathBox, SEditableTextBox).Text(FText::FromString(DefaultSolutionPath())) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
				[ Button(LOCTEXT("Browse", "Browse..."), Browse) ])
		];

	// ---- the loads panel ----
	TSharedRef<SWidget> LoadsPanel =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("InertialGroup", "Inertial load"),
				FormRow(LOCTEXT("LoadFactor", "Load factor:"), Spin(&FState::LoadFactor, -100.0, 100.0, 0.5, FText::FromString(TEXT(" g"))), 110.0f))
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("PressureGroup", "Pressure on every wetted face"),
				FormRow(LOCTEXT("Pressure", "Pressure:"), Spin(&FState::Pressure, -1e12, 1e12, 100.0, StressSuffix), 110.0f))
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("PointGroup", "Point forces"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(S->PointList, SVerticalBox) ]
				+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)
				[ Button(LOCTEXT("AddPoint", "Add point force"), [S]() { S->PointRows.Add(MakeShared<FPointRow>()); S->RebuildPointList(); }) ])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8).HAlign(HAlign_Left)
		[ Button(LOCTEXT("ApplyLoads", "Apply loads"), [S]() { S->ApplyInputs(); }) ]
		+ SScrollBox::Slot()
		[
			MakeInfoLabel(LOCTEXT("LoadsHint",
				"The load factor puts the model's own mass under that many g along -z (0 to begin with, so a test load acts alone; 1 is its weight, a 4 g pull-up a real case). The pressure acts on every wetted face, positive pressing inwards -- the place a pressure field from the potential flow or DUST will plug in. Each point force goes to the corner node nearest its point; the picture warns when that node is more than a mesh edge away. Points are typed in FULL-SCALE coordinates, as the parameterisations are; with a model scale set (Model > Scale) the dialog scales them to the replica. Forces are newtons whatever the project's units or scale. Apply pushes the loads onto the built model and redraws them."))
		];

	// ---- the supports panel ----
	TSharedRef<SWidget> SupportPanel =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("SupportsGroup", "Support planes"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(S->SupportList, SVerticalBox) ]
				+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)
				[ Button(LOCTEXT("AddSupport", "Add support"), [S]() { S->SupportRows.Add(MakeShared<FSupportRow>()); S->RebuildSupportList(); }) ])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8).HAlign(HAlign_Left)
		[ Button(LOCTEXT("ApplySupports", "Apply supports"), [S]() { S->ApplyInputs(); }) ]
		+ SScrollBox::Slot()
		[
			MakeInfoLabel(LOCTEXT("SupportHint",
				"Every node within the tolerance of any plane is fixed in every degree of freedom (the shells' rotations too: a clamp). y = 0 is the root of a mirrored half -- the symmetry plane -- which is why it is the one to start with; the mesh's minimum or maximum along an axis clamps a body by its end -- 'Of' says whose end: the whole mesh's, or one body's, so a body that sticks out further does not take a support meant for another. A plane limited to a PATCH fixes only the nodes inside its rectangle: type it, Fit to body, or drag the handles in the picture (the fixed nodes follow live). Values, tolerances and patches are typed in FULL-SCALE coordinates, as the parameterisations are; with a model scale set (Model > Scale) the dialog scales them to the replica. A tolerance of 0 is a ten-thousandth of the mesh's diagonal -- about how planar a lofted section comes out, so a root ring is taken whole. Whatever reaches no plane, on its own or through a body it was fused with, is left out of the solve and listed on the right. Apply pushes the supports onto the built model and redraws them."))
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SetupTab", "Set-up"), SetupTab });
	Tabs.Add({ LOCTEXT("MeshTab", "Mesh"), MakePictureTab(S, TabMesh, LOCTEXT("MeshEmpty", "Build the model (Set-up tab) to see the mesh the solver is handed: its wetted faces by region, with their edges."), nullptr) });
	Tabs.Add({ LOCTEXT("LoadsTab", "Loads"), MakePictureTab(S, TabLoads, LOCTEXT("LoadsEmpty", "Build the model (Set-up tab) to see the loads as vectors: the pressure on the wetted faces, each point force at its node, the inertial load at the centre of mass."), LoadsPanel) });
	Tabs.Add({ LOCTEXT("SupportTab", "Support"), MakePictureTab(S, TabSupport, LOCTEXT("SupportEmpty", "Build the model (Set-up tab) to see the boundary conditions: a marker on every node the supports fix, each plane with its patch handles, and what reaches no support."), SupportPanel) });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(FText::FromString(kHelp)) });
	// The Support picture's handles: a press on one drags its patch, the
	// fixed nodes following, the model re-derived on release.
	if (S->Views[TabSupport].IsValid())
	{
		TWeakPtr<FState> Weak = S;
		S->Views[TabSupport]->OnPickHover = [Weak](const FVector2D& Pixel, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->OnPatchHover(Pixel); };
		S->Views[TabSupport]->OnPickDown = [Weak](const FVector2D& Pixel, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->OnPatchDown(Pixel); };
		S->Views[TabSupport]->OnPickMove = [Weak](const FVector2D&, const FVector& O, const FVector& D) { TSharedPtr<FState> P = Weak.Pin(); if (P.IsValid()) P->OnPatchMove(O, D); return true; };
		S->Views[TabSupport]->OnPickUp = [Weak](const FVector2D&, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); if (P.IsValid()) P->OnPatchUp(); return true; };
	}

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SAssignNew(S->Tabs, SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				// The solve's own status: its stage and elapsed time, with a
				// marquee, while the worker runs; hidden otherwise.
				+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 12, 0)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 3)
					[ SAssignNew(S->LblProgress, STextBlock).AutoWrapText(true).Visibility(EVisibility::Collapsed).ColorAndOpacity(FSlateColor(FLinearColor(0.9f, 0.9f, 0.9f))) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ SNew(SBox).HeightOverride(8) [ SAssignNew(S->ProgressBar, SProgressBar).Percent(TOptional<float>()).Visibility(EVisibility::Collapsed) ] ]
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(S->BuildButton, SButton).Text(LOCTEXT("BuildBtn", "Build Model")).HAlign(HAlign_Center).OnClicked_Lambda([S]() { if (S->BuildModel() && S->Tabs.IsValid()) S->Tabs->SetActiveTab(TabMesh); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(S->SolveButton, SButton).Text(LOCTEXT("SolveBtn", "Solve")).HAlign(HAlign_Center).OnClicked_Lambda([S]() { S->Solve(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(S->CancelButton, SButton).Text(LOCTEXT("CancelBtn", "Cancel")).HAlign(HAlign_Center).IsEnabled(false).OnClicked_Lambda([S]() { S->CancelSolve(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([S](const TSharedRef<SWindow>&) { S->Close(); }));

	S->RebuildPointList();
	S->RebuildSupportList();
	S->DescribeModel();
	Status(LOCTEXT("Ready", "Choose the bodies; Build Model to inspect them; set the loads and supports on their tabs; Solve to run."));
	ShowAerotelicDialogWindow(Window, Size);

	if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicFeaSmoke")))
	{
		// A developer run on whatever project was given: a parameterisation
		// without a structural material gets a preset with a skin of one
		// percent of its extent (the sample projects carry none). Every
		// load kind on, so every arrow kind is in the pictures.
		S->bSmoke = true;
		for (const FAerotelicBodyChoicePtr& C : Choices)
		{
			AAerotelicEntityActor* E = C->Operand.Entity.Get();
			if (E == nullptr || E->IsDerived() || HasStructuralMaterial(E->GetMassSpec())) continue;
			if (const aerotelic::Material* Preset = aerotelic::FindMaterialPreset("Aluminium 2024-T3 sheet"))
			{
				aerotelic::MassSpec Spec;
				Spec.mode = aerotelic::MassMode::Material;
				Spec.material = *Preset;
				// One percent of the FULL-SCALE extent: the placed bounds are the
				// replica's, and a skin thickness does not scale with the model.
				const FBox Bounds = E->GetPlacedBounds(false);
				Spec.skin_thickness = Bounds.IsValid ? FMath::Max(0.01 * Bounds.GetSize().GetMax() / AAerotelicEntityActor::GetModelScale(), 1e-4) : 0.01;
				E->SetMassSpec(Spec);
			}
		}
		S->DescribeModel();
		S->Pressure = 1000.0 / S->Units.StressToSI();
		S->LoadFactor = 1.0;
		TSharedPtr<FPointRow> Tip = MakeShared<FPointRow>();
		FBox All(ForceInit);
		for (const FAerotelicBodyChoicePtr& C : InitiallyChosen) All += C->Operand.Entity->GetPlacedBounds(C->Operand.bMirror);
		// The rows are typed full-scale; the bounds are the replica's.
		const double Scale = AAerotelicEntityActor::GetModelScale();
		if (All.IsValid) { Tip->X = All.GetCenter().X / Scale; Tip->Y = All.Max.Y / Scale; Tip->Z = All.GetCenter().Z / Scale; }
		S->PointRows.Add(Tip);
		S->RebuildPointList();
		if (S->BuildModel()) S->RunSmoke();
		else FPlatformMisc::RequestExit(false);
	}
}

#undef LOCTEXT_NAMESPACE
