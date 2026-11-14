#pragma once

// The structural (Kratos FEA) case ENGINE (2026-11-15): the state that
// was the Kratos dialog's -- the chosen bodies, the support planes and
// patches, the loads, the built model, its derivation, the four pictures
// with the patch dragging, the solve -- as a header both dialogs share:
// Model > Load Cases edits structural cases on it (with the full built
// model in its pictures) and Analysis > Structural renders a chosen case
// and solves it. Inline, as it was inside the dialog's translation unit;
// no behaviour was changed in the move except what the two dialogs need:
// a fixed body list where there is no picker, the aero pressure field set
// directly, and the case specs read and written (load_cases.hpp).

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
#include "Entity/AerotelicAeroLoads.h"

#include "aerotelic/materials.hpp"
#include "aerotelic/units.hpp"
#include "aerotelic/vec3.hpp"

#include <atomic>
#include <memory>
#include <vector>

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBodyStructure.h"
#include "Entity/AerotelicFeaModelBuilder.h"
#include "Entity/AerotelicAeroTrim.h"
#include "aerotelic/load_cases.hpp"

#define LOCTEXT_NAMESPACE "AerotelicFeaCase"

namespace AerotelicFeaCase
{
	using AerotelicFeaPreprocess::FFace;
	using AerotelicFeaPreprocess::FModel;
	using aerotelic::project_yaml::EntityType;

	inline FString Thousands(int64 N)
	{
		return FText::AsNumber(N).ToString();
	}

	inline TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field, float LabelWidth = 200.0f)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(LabelWidth) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	inline TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-")))
			.AutoWrapText(true)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
	}

	inline TSharedRef<SWidget> Button(const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center).OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	}

	inline FString DefaultSolutionPath()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_fea.vtk"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_fea.vtk"));
	}

	inline FString UnitName(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }

	inline bool HasStructuralMaterial(const aerotelic::MassSpec& S)
	{
		return S.mode == aerotelic::MassMode::Material && S.material.IsStructural();
	}

	inline bool SameOperand(const FAerotelicBooleanOperand& A, const FAerotelicBooleanOperand& B)
	{
		return A.Entity.Get() == B.Entity.Get() && A.bMirror == B.bMirror;
	}

	inline FString OperandLabel(const FAerotelicBooleanOperand& Op)
	{
		return Op.Entity.IsValid() ? Op.Entity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT("")) : FString(TEXT("(removed entity)"));
	}

	// The SOLID BODIES the analysis picks among: every body of every scene
	// operand, each modelled as the shells of its skin and its ribs. A
	// wing's flap, a propeller's blade, a boolean result's separate shell
	// is each a row of its own; an operand whose bodies are not known yet
	// (never tessellated) is one row for all of them.
	inline TArray<FAerotelicBodyChoicePtr> AnalysableChoices()
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

	inline FColor Srgb(float R, float G, float B) { return AAerotelicColoredMeshActor::VertexColorFromSRGB(R, G, B); }

	inline void Basis(const FVector& Dir, FVector& U, FVector& W)
	{
		const FVector Helper = FMath::Abs(Dir.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector;
		U = FVector::CrossProduct(Dir, Helper).GetSafeNormal();
		W = FVector::CrossProduct(Dir, U).GetSafeNormal();
	}

	// An arrow whose HEAD sits at Tip (the point of application) pointing
	// along Dir: a square shaft and a six-sided cone.
	inline void AddArrow(FGeom& G, const FVector& Tip, const FVector& Dir, double Length, double Radius, const FColor& Col)
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

	inline void AddOctahedron(FGeom& G, const FVector& P, double R, const FColor& Col)
	{
		const FVector X(R, 0, 0), Y(0, R, 0), Z(0, 0, R);
		const FVector Px = P + X, Mx = P - X, Py = P + Y, My = P - Y, Pz = P + Z, Mz = P - Z;
		G.Tri(Px, Py, Pz, Col); G.Tri(Py, Mx, Pz, Col); G.Tri(Mx, My, Pz, Col); G.Tri(My, Px, Pz, Col);
		G.Tri(Py, Px, Mz, Col); G.Tri(Mx, Py, Mz, Col); G.Tri(My, Mx, Mz, Col); G.Tri(Px, My, Mz, Col);
	}

	inline void AddBar(FGeom& G, const FVector& A, const FVector& B, double R, const FColor& Col)
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

	inline FColor RegionColor(int32 Index)
	{
		static const FColor Palette[] = {
			Srgb(0.36f, 0.62f, 0.90f), Srgb(0.95f, 0.65f, 0.25f), Srgb(0.45f, 0.78f, 0.45f), Srgb(0.85f, 0.45f, 0.75f),
			Srgb(0.80f, 0.80f, 0.35f), Srgb(0.40f, 0.80f, 0.80f), Srgb(0.90f, 0.50f, 0.45f), Srgb(0.65f, 0.60f, 0.90f) };
		return Palette[Index % UE_ARRAY_COUNT(Palette)];
	}

	inline const FColor kGrey = Srgb(0.74f, 0.75f, 0.77f);
	inline const FColor kPressure = Srgb(1.0f, 0.62f, 0.15f);
	inline const FColor kPointForce = Srgb(0.95f, 0.18f, 0.18f);
	inline const FColor kGravity = Srgb(0.30f, 0.85f, 0.35f);
	inline const FColor kFixed = Srgb(0.95f, 0.15f, 0.15f);
	inline const FColor kPlane = Srgb(0.35f, 0.55f, 1.0f);
	inline const FColor kHandle = Srgb(1.0f, 0.75f, 0.2f);
	inline const FColor kHandleHot = Srgb(1.0f, 0.95f, 0.5f);

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

	// A mass item as the document has it (2026-11-13; aerotelic::MassItem,
	// edited in Analysis > Mass Properties, read-only here): mass and
	// inertia in the project's units as built, the centre and the picked
	// attachment point full-scale, the body it hangs on by scene name, the
	// picker's row for that body (0 none, i > 0 the picker's choice i - 1)
	// and how many of the body's nearest nodes carry it.
	struct FMassRow
	{
		FString Name = TEXT("mass");
		double Mass = 1.0;
		double X = 0.0, Y = 0.0, Z = 0.0;
		double I[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };   // Ixx, Iyy, Izz, Ixy, Ixz, Iyz
		FString AttachEntity, AttachBody;
		bool bAttachMirror = false;
		bool bHasPoint = false;
		double PX = 0.0, PY = 0.0, PZ = 0.0;
		int32 Attach = 0;
		int32 Nodes = 8;
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

	// The model as built from the chosen bodies: the full node list and
	// faces as built, the ones the solver gets (the full set less whatever
	// floats free of the supports), the request, and what the pre-processor
	// derives. One heap object, so the request's pointers into it stay good.
	struct FBuilt
	{
		std::vector<aerotelic::Vec3> FullNodes;
		TArray<AerotelicStructuralFea::FShellFace> FullShellFaces;
		TArray<AerotelicStructuralFea::FShellFace> FullRibFaces;
		// The faces' material axes (one per face, 2026-11-12) and the
		// regions as built, with their laminates' drapes: trimmed with the
		// faces when some float.
		TArray<aerotelic::Vec3> FullShellAxes, FullRibAxes;
		TArray<AerotelicStructuralFea::FRegion> FullRegions;
		std::vector<aerotelic::Vec3> Nodes;
		AerotelicStructuralFea::FRequest Request;
		FModel Model;
		TArray<FString> Problems;   // left out while meshing
		TArray<FString> Dropped;    // left out for floating, this derivation
		TArray<FString> Hinges;     // the flaps hinged to their wings, or why not (2026-11-14)
		// The aerodynamic load case applied to the wetted faces, if one is
		// (stage 4, 2026-11-14), and how its panels landed on them.
		TSharedPtr<FAerotelicAeroLoadCase> AeroCase;
		AerotelicAeroLoads::FMapResult AeroMap;
		TArray<FString> AeroNotes;
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

	enum ETab { TabSetup = 0, TabMesh = 1, TabLoads = 2, TabMasses = 3, TabSupport = 4, TabHelp = 5 };
	constexpr int32 kPictures = 5;   // indexed by ETab; TabSetup's slot unused

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
		TArray<TSharedPtr<FString>> SolverOptions, ShellOptions, AxisOptions, PlaneOptions, BodyOptions;
		int32 SolverIndex = 0, ShellIndex = 0;
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
		double Modes = 6.0;         // natural frequencies computed after the static solve; 0: none
		// The loads and the supports: any number of point forces and of
		// planes, edited on their tabs and pushed by Apply.
		double LoadFactor = 0.0, Pressure = 0.0;
		TArray<TSharedPtr<FPointRow>> PointRows;
		TArray<TSharedPtr<FSupportRow>> SupportRows;
		TSharedPtr<SVerticalBox> PointList, SupportList;
		// The mass items (2026-11-13): the document's, edited here and
		// written back by Apply masses; the attachment options are "none"
		// then the picker's rows.
		TArray<TSharedPtr<FMassRow>> MassRows;
		TSharedPtr<SVerticalBox> MassList;
		TArray<TSharedPtr<FString>> AttachOptions, SupportModeOptions;
		// The aerodynamic pressure field applied on Apply (stage 4): set by
		// the dialog from the case it shows (an aero load case's solved
		// field); none for a structural case.
		TSharedPtr<FAerotelicAeroLoadCase> AeroField;
		// The bodies where the state has no picker (the Kratos dialog, which
		// renders a case and does not edit it): ChosenBodies() returns these.
		TArray<FAerotelicBodyChoicePtr> ChosenFixed;
		// The name of the case the state carries, for captions and files.
		FString CaseName;
		// The support mode: 0 clamped at the planes, 1 FREE FLIGHT (inertia
		// relief: no planes, one reference node, the loads balanced by the
		// rigid-body inertial field). The reference is the mass centre, or
		// a point typed full-scale.
		int32 SupportMode = 0;
		bool bReferenceAtCg = true;
		double RefX = 0.0, RefY = 0.0, RefZ = 0.0;
		bool IsFreeFlight() const { return SupportMode == 1; }
		TSharedPtr<SEditableTextBox> PathBox;
		TSharedPtr<STextBlock> LblMaterials, LblMesh, LblDisp, LblStress, LblMargins, LblBalance, LblModes, LblTime;
		// The solve's own status line and marquee in the dialog, and the
		// buttons that go quiet while it runs.
		TSharedPtr<STextBlock> LblProgress;
		TSharedPtr<SProgressBar> ProgressBar;
		TSharedPtr<SButton> BuildButton, SolveButton, CancelButton;

		// The model, once built (Build Model, or Solve).
		TSharedPtr<FBuilt> Built;
		// The solve in flight, if one is.
		TSharedPtr<FSolveJob> Job;
		// The rib meshes of the bodies' own grids, keyed by the surface's
		// bytes, the rib size and the planes, so Build Model after a load or
		// support edit does not run gmsh again
		// (AerotelicFeaModelBuilder::FCache).
		AerotelicFeaModelBuilder::FCache RibCache;

		// The pre-processor pictures: one view per tab, its actors, its caption.
		TSharedPtr<SAerotelicSceneView> Views[kPictures];
		TSharedPtr<STextBlock> Captions[kPictures];
		TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>> Actors[kPictures];
		bool bFramed[kPictures] = {};

		TSharedPtr<SAerotelicTabStrip> Tabs;
		bool bSmoke = false;

		// The support patches' handles as last drawn, and the one being
		// dragged (its plane and kind; -1 none). Hover is the handle under
		// the cursor, drawn brighter.
		TArray<FPatchHandle> Handles;
		int32 DragPlane = -1, DragKind = -1;
		int32 HoverHandle = -1;

		bool IsSolving() const { return Job.IsValid() && Job->IsRunning(); }

		void Message(const FText& Text)
		{
			AerotelicMessageBox(EAppMsgType::Ok, Text, LOCTEXT("Title", "Structural (Kratos FEA)"));
		}

		// The builder's options: the status line to write progress to (the
		// model has no other choices since 2026-11-08 -- shells only).
		AerotelicFeaModelBuilder::FOptions BuilderOptions() const
		{
			AerotelicFeaModelBuilder::FOptions O;
			TFunction<void(const FText&)> St = Status;
			O.Status = [St](const FString& Line) { if (St) St(FText::FromString(Line)); };
			return O;
		}

		TArray<FAerotelicBodyChoicePtr> ChosenBodies() const
		{
			TArray<FAerotelicBodyChoicePtr> Out;
			const TArray<FAerotelicBodyChoicePtr>& From = Picker.IsValid() ? Picker->GetChosen() : ChosenFixed;
			for (const FAerotelicBodyChoicePtr& C : From) if (C->Operand.Entity.IsValid()) Out.Add(C);
			return Out;
		}

		// ---- the case specs (load_cases.hpp): the rows from a spec and back ----

		// The picker's row for a body named in a spec, or null: the scene
		// name, the mirror and the role (a spec with no role takes the
		// entity's row whatever its role, the single-body case).
		FAerotelicBodyChoicePtr FindChoice(const aerotelic::LoadCaseBody& B) const
		{
			const FString Entity = UTF8_TO_TCHAR(B.entity.c_str());
			const FString Body = UTF8_TO_TCHAR(B.body.c_str());
			for (const FAerotelicBodyChoicePtr& C : Choices)
			{
				if (!C->Operand.Entity.IsValid() || C->Operand.Entity->EntityName != Entity || C->Operand.bMirror != B.mirror) continue;
				if (!Body.IsEmpty() && C->Body != Body) continue;
				return C;
			}
			return nullptr;
		}
		static aerotelic::LoadCaseBody BodyOf(const FAerotelicBodyChoice& C)
		{
			aerotelic::LoadCaseBody B;
			B.entity = C.Operand.Entity.IsValid() ? TCHAR_TO_UTF8(*C.Operand.Entity->EntityName) : "";
			B.mirror = C.Operand.bMirror;
			B.body = TCHAR_TO_UTF8(*C.Body);
			return B;
		}
		// The rows of a body list, in the spec's order; the names it could
		// not place are returned.
		TArray<FAerotelicBodyChoicePtr> ChoicesFor(const std::vector<aerotelic::LoadCaseBody>& Bodies, TArray<FString>* OutMissing = nullptr) const
		{
			TArray<FAerotelicBodyChoicePtr> Out;
			for (const aerotelic::LoadCaseBody& B : Bodies)
			{
				FAerotelicBodyChoicePtr C = FindChoice(B);
				if (C.IsValid()) { if (!Out.Contains(C)) Out.Add(C); }
				else if (OutMissing) OutMissing->Add(FString(UTF8_TO_TCHAR(B.entity.c_str())) + (B.mirror ? TEXT(" (mirror)") : TEXT("")) + (B.body.empty() ? FString() : FString(TEXT(" / ")) + UTF8_TO_TCHAR(B.body.c_str())));
			}
			return Out;
		}

		// A structural case's rows from its spec: the loads, the constraints,
		// the bodies (into ChosenFixed; a dialog with a picker builds it from
		// them). Returns the bodies it could not find.
		TArray<FString> LoadStatic(const aerotelic::StaticLoadCaseSpec& Spec)
		{
			CaseName = UTF8_TO_TCHAR(Spec.name.c_str());
			LoadFactor = Spec.load_factor;
			Pressure = Spec.pressure;
			SupportMode = Spec.free_flight ? 1 : 0;
			bReferenceAtCg = !Spec.reference_given;
			RefX = Spec.reference[0]; RefY = Spec.reference[1]; RefZ = Spec.reference[2];
			SupportRows.Reset();
			for (const aerotelic::LoadCaseSupportSpec& S : Spec.supports)
			{
				TSharedPtr<FSupportRow> R = MakeShared<FSupportRow>();
				R->Axis = S.axis == 'x' ? 0 : (S.axis == 'z' ? 2 : 1);
				R->Where = S.at == "min" ? 1 : (S.at == "max" ? 2 : 0);
				R->Value = S.value;
				R->Tolerance = S.tolerance;
				R->bBounded = S.bounded;
				for (int32 k = 0; k < 2; ++k) { R->Lo[k] = S.lo[k]; R->Hi[k] = S.hi[k]; }
				R->Body = 0;
				if (S.has_of)
				{
					const FAerotelicBodyChoicePtr C = FindChoice(S.of);
					const int32 I = C.IsValid() ? Choices.IndexOfByKey(C) : INDEX_NONE;
					R->Body = I != INDEX_NONE ? I + 1 : 0;
				}
				SupportRows.Add(R);
			}
			PointRows.Reset();
			for (const aerotelic::LoadCasePointForceSpec& F : Spec.point_forces)
			{
				TSharedPtr<FPointRow> R = MakeShared<FPointRow>();
				R->X = F.at[0]; R->Y = F.at[1]; R->Z = F.at[2];
				R->Fx = F.force[0]; R->Fy = F.force[1]; R->Fz = F.force[2];
				PointRows.Add(R);
			}
			TArray<FString> Missing;
			ChosenFixed = ChoicesFor(Spec.bodies, &Missing);
			AeroField.Reset();
			return Missing;
		}

		// The rows as a structural case spec (the chosen bodies as they
		// stand, from the picker where there is one).
		aerotelic::StaticLoadCaseSpec SaveStatic() const
		{
			aerotelic::StaticLoadCaseSpec Spec;
			Spec.name = TCHAR_TO_UTF8(*CaseName);
			Spec.load_factor = LoadFactor;
			Spec.pressure = Pressure;
			Spec.free_flight = IsFreeFlight();
			Spec.reference_given = !bReferenceAtCg;
			Spec.reference = { RefX, RefY, RefZ };
			for (const TSharedPtr<FSupportRow>& R : SupportRows)
			{
				aerotelic::LoadCaseSupportSpec S;
				S.axis = "xyz"[FMath::Clamp(R->Axis, 0, 2)];
				S.at = R->Where == 1 ? "min" : (R->Where == 2 ? "max" : "value");
				S.value = R->Value;
				S.tolerance = R->Tolerance;
				S.bounded = R->bBounded;
				S.lo = { R->Lo[0], R->Lo[1] };
				S.hi = { R->Hi[0], R->Hi[1] };
				if (R->Body > 0 && Choices.IsValidIndex(R->Body - 1))
				{
					S.has_of = true;
					S.of = BodyOf(*Choices[R->Body - 1]);
				}
				Spec.supports.push_back(S);
			}
			for (const TSharedPtr<FPointRow>& R : PointRows)
			{
				aerotelic::LoadCasePointForceSpec F;
				F.at = { R->X, R->Y, R->Z };
				F.force = { R->Fx, R->Fy, R->Fz };
				Spec.point_forces.push_back(F);
			}
			for (const FAerotelicBodyChoicePtr& C : ChosenBodies()) Spec.bodies.push_back(BodyOf(*C));
			return Spec;
		}

		// An aero case as the state renders it: its bodies, free flight at
		// the mass centre, gravity at 1 g, no uniform pressure, no point
		// forces, and its solved pressure field (null: not solved yet).
		TArray<FString> LoadAero(const aerotelic::AeroLoadCaseSpec& Spec, TSharedPtr<FAerotelicAeroLoadCase> Field)
		{
			CaseName = UTF8_TO_TCHAR(Spec.name.c_str());
			LoadFactor = 1.0;
			Pressure = 0.0;
			SupportMode = 1;
			bReferenceAtCg = true;
			SupportRows.Reset();
			PointRows.Reset();
			TArray<FString> Missing;
			ChosenFixed = ChoicesFor(Spec.bodies, &Missing);
			AeroField = Field;
			return Missing;
		}

		// What a body is, from its structure (body_structure.hpp), and how it
		// will be modelled -- AerotelicFeaModelBuilder::Describe, the same
		// rules AppendBody builds by.
		void DescribeModel()
		{
			if (!LblMaterials.IsValid()) return;
			TArray<FString> Lines;
			const AerotelicFeaModelBuilder::FOptions O = BuilderOptions();
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
			Request.bFreeFlight = IsFreeFlight();
			Request.bReferenceGiven = IsFreeFlight() && !bReferenceAtCg;
			Request.Reference = aerotelic::Vec3{ RefX * Scale, RefY * Scale, RefZ * Scale };
			for (const TSharedPtr<FSupportRow>& R : IsFreeFlight() ? TArray<TSharedPtr<FSupportRow>>() : SupportRows)
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
			static const TCHAR* kSolvers[] = { TEXT("auto"), TEXT("sparse_lu"), TEXT("amgcl") };
			Request.LinearSolver = kSolvers[FMath::Clamp(SolverIndex, 0, 2)];
			// The thick corotational triangle is no longer offered (2026-11-02:
			// too stiff and not converging on a thin-walled box, its force
			// balance drifting with refinement); the script still takes it.
			static const TCHAR* kShellElements[] = { TEXT("ShellThinElement3D3N"), TEXT("ShellThinElementCorotational3D3N") };
			Request.ShellElement = kShellElements[FMath::Clamp(ShellIndex, 0, 1)];
			Request.Modes = FMath::Max(FMath::RoundToInt32(Modes), 0);
			Request.VtkPath = PathBox.IsValid() ? PathBox->GetText().ToString().TrimStartAndEnd() : FString();
		}

		// The picker's rows that are boolean results built over the entity an
		// item hangs on (by scene name and mirror), in picker order: where an
		// item's attachment is INHERITED when its own body is not in the
		// model but a result fused from it is.
		TArray<int32> BooleanHeirs(const FMassRow& R) const
		{
			TArray<int32> Out;
			for (int32 i = 0; i < Choices.Num(); ++i)
			{
				const FAerotelicBodyChoice& C = *Choices[i];
				AAerotelicEntityActor* E = C.Operand.Entity.Get();
				if (E == nullptr || E->GetEntityType() != EntityType::Boolean) continue;
				const FAerotelicBooleanModel* M = E->GetModelAs<FAerotelicBooleanModel>();
				if (M == nullptr) continue;
				for (const FAerotelicBooleanOperand& Op : M->Operands)
				{
					if (Op.Entity.IsValid() && Op.Entity->EntityName == R.AttachEntity && Op.bMirror == R.bAttachMirror) { Out.Add(i); break; }
				}
			}
			return Out;
		}

		// The mass items onto the request's mesh AS IT STANDS (its nodes are
		// renumbered when floating parts are dropped, so this runs after
		// that too): each item hangs on the nodes of its body's regions
		// nearest its picked point; when that body is not in the model but a
		// boolean result built over it is, the item hangs on the result's
		// surface at the same point (the nearest of the result's bodies);
		// one with no body in the model is left out and named. Mass and
		// inertia to SI, the points to the replica.
		void ResolveMasses(FBuilt& B)
		{
			AerotelicStructuralFea::FRequest& Request = B.Request;
			Request.Masses.Reset();
			const double Scale = ModelScale();
			const double M2 = Units.MassToSI(), L2 = Units.LengthToSI();
			for (const TSharedPtr<FMassRow>& R : MassRows)
			{
				AerotelicStructuralFea::FMassItem Item;
				Item.Name = R->Name;
				Item.Mass = R->Mass * M2;
				Item.At = aerotelic::Vec3{ R->X * Scale, R->Y * Scale, R->Z * Scale };
				for (int32 k = 0; k < 6; ++k) Item.Inertia[k] = R->I[k] * M2 * L2 * L2;
				const aerotelic::Vec3 Point = R->bHasPoint ? aerotelic::Vec3{ R->PX * Scale, R->PY * Scale, R->PZ * Scale } : Item.At;
				if (R->AttachEntity.IsEmpty())
				{
					B.Dropped.Add(FString::Printf(TEXT("mass item '%s' (%.4g %s) hangs on nothing and is left out of the analysis: pick its body in Analysis > Mass Properties"), *R->Name, R->Mass, *UnitName(Units.MassName())));
					continue;
				}
				const int32 Count = FMath::Max(R->Nodes, 3);
				FString Carrier;
				if (R->Attach > 0)
				{
					Item.AttachNodes = AerotelicFeaModelBuilder::AttachmentNodes(B.Nodes, Request.ShellFaces, Request.RibFaces, Request.Regions, R->Attach - 1, Point, Count);
					Carrier = AttachOptions.IsValidIndex(R->Attach) ? *AttachOptions[R->Attach] : FString::FromInt(R->Attach);
				}
				if (Item.AttachNodes.Num() < 3)
				{
					// Inherited by a boolean result over the body: the heir whose
					// nodes come nearest the point.
					double Best = 1e300;
					for (int32 Heir : BooleanHeirs(*R))
					{
						TArray<int32> Nodes = AerotelicFeaModelBuilder::AttachmentNodes(B.Nodes, Request.ShellFaces, Request.RibFaces, Request.Regions, Heir, Point, Count);
						if (Nodes.Num() < 3) continue;
						double Near = 1e300;
						for (int32 N : Nodes) Near = FMath::Min(Near, aerotelic::DistanceSq(B.Nodes[N], Point));
						if (Near < Best) { Best = Near; Item.AttachNodes = Nodes; Carrier = Choices[Heir]->Label + TEXT(" (inherited from ") + R->AttachEntity + TEXT(")"); }
					}
				}
				if (Item.AttachNodes.Num() < 3)
				{
					B.Dropped.Add(FString::Printf(TEXT("mass item '%s' (%.4g %s) hangs on %s%s, which is not in the model (not chosen, refused, or floating), and no boolean result over it is: left out"),
						*R->Name, R->Mass, *UnitName(Units.MassName()), *R->AttachEntity, R->AttachBody.IsEmpty() ? TEXT("") : *(TEXT(" / ") + R->AttachBody)));
					continue;
				}
				Request.Masses.Add(Item);
				UE_LOG(LogTemp, Log, TEXT("StructuralFea: mass item '%s' hangs on %s through %d nodes"), *R->Name, *Carrier, Item.AttachNodes.Num());
			}
		}

		// The flaps' hinges and pushrods onto the mesh AS IT STANDS (2026-11-14;
		// after a trim too, like the masses): every chosen flap whose wing's
		// main element is chosen as well is tied to it at its hinge brackets
		// and held by its pushrod (AerotelicFeaModelBuilder::ResolveFlapHinges),
		// so it is on the load path and not dropped as floating.
		void ResolveHinges(FBuilt& B)
		{
			AerotelicStructuralFea::FRequest& Request = B.Request;
			Request.Ties.Reset();
			Request.Links.Reset();
			Request.Bars.Reset();
			B.Hinges.Reset();
			TArray<AerotelicFeaModelBuilder::FChosenBody> Chosen;
			for (const FAerotelicBodyChoicePtr& C : ChosenBodies())
			{
				AerotelicFeaModelBuilder::FChosenBody K;
				K.Operand = C->Operand;
				K.Role = C->Body;
				K.Index = Choices.IndexOfByKey(C);
				Chosen.Add(K);
			}
			AerotelicFeaModelBuilder::ResolveFlapHinges(Chosen, B.Nodes, Request.ShellFaces, Request.RibFaces, Request.Regions, Request, B.Hinges);
			for (const FString& H : B.Hinges) UE_LOG(LogTemp, Log, TEXT("StructuralFea: %s"), *H);
		}

		// The aero pressure field onto the request's wetted faces AS
		// THEY STAND (after the floating trim too, like the masses): q Cp on
		// the faces of the regions whose body the case paneled -- the
		// region's entity, or an operand of its boolean result -- added to
		// the uniform pressure; the rest keep the uniform pressure alone.
		void ApplyAeroPressure(FBuilt& B)
		{
			AerotelicStructuralFea::FRequest& Request = B.Request;
			Request.PressurePerFace.Reset();
			B.AeroCase.Reset();
			B.AeroMap = AerotelicAeroLoads::FMapResult();
			B.AeroNotes.Reset();
			TSharedPtr<FAerotelicAeroLoadCase> Case = AeroField;
			if (!Case.IsValid()) return;
			// Which regions the case covers.
			TSet<int32> Covered;
			TArray<FString> Uncovered;
			for (const AerotelicStructuralFea::FRegion& R : Request.Regions)
			{
				AAerotelicEntityActor* E = Choices.IsValidIndex(R.Body) ? Choices[R.Body]->Operand.Entity.Get() : nullptr;
				bool bCovered = E != nullptr && Case->Entities.Contains(E->EntityName);
				if (!bCovered && E != nullptr && E->GetEntityType() == EntityType::Boolean)
				{
					if (const FAerotelicBooleanModel* M = E->GetModelAs<FAerotelicBooleanModel>())
						for (const FAerotelicBooleanOperand& Op : M->Operands) bCovered = bCovered || (Op.Entity.IsValid() && Case->Entities.Contains(Op.Entity->EntityName));
				}
				if (bCovered) Covered.Add(R.Id);
				else if (R.HasShells()) Uncovered.Add(R.Label);
			}
			AerotelicAeroLoads::MapToFaces(*Case, B.Nodes, Request.ShellFaces, [&Covered](int32 Id) { return Covered.Contains(Id); }, Request.Pressure, Request.LengthToSI, B.AeroMap);
			Request.PressurePerFace = B.AeroMap.Pressure;
			B.AeroCase = Case;
			if (Uncovered.Num() > 0) B.AeroNotes.Add(TEXT("not paneled by the load case, so no air load on: ") + FString::Join(Uncovered, TEXT(", ")));
			if (B.AeroMap.Stats.unmapped > 0) B.AeroNotes.Add(FString::Printf(TEXT("%s face(s) of the covered bodies lie farther than a tenth of the case's extent from every panel on their side and got no air load"), *Thousands(B.AeroMap.Stats.unmapped)));
			UE_LOG(LogTemp, Log, TEXT("StructuralFea: load case '%s' mapped onto %d of %d covered faces (%d unmapped; farthest %.4g, mean %.4g); resultant (%.4g, %.4g, %.4g) N"),
				*Case->Name, B.AeroMap.Stats.mapped, B.AeroMap.FacesCovered, B.AeroMap.Stats.unmapped, B.AeroMap.Stats.max_distance, B.AeroMap.Stats.mean_distance, B.AeroMap.Resultant.X, B.AeroMap.Resultant.Y, B.AeroMap.Resultant.Z);
		}

		// The rows from the document's mass items (Analysis > Mass Properties
		// edits them): a row per item, the attachment matched to the picker's
		// row by scene name, mirror and body role (0 when that body is not in
		// the picker: a boolean result over it may still inherit it).
		void LoadMassRows()
		{
			MassRows.Reset();
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			if (Doc == nullptr) return;
			for (const aerotelic::MassItem& M : Doc->GetMassItems())
			{
				TSharedPtr<FMassRow> Row = MakeShared<FMassRow>();
				Row->Name = UTF8_TO_TCHAR(M.name.c_str());
				Row->Mass = M.mass;
				Row->X = M.position[0]; Row->Y = M.position[1]; Row->Z = M.position[2];
				for (int32 k = 0; k < 6; ++k) Row->I[k] = M.inertia[k];
				Row->Nodes = M.attach_nodes;
				Row->AttachEntity = UTF8_TO_TCHAR(M.attach_entity.c_str());
				Row->AttachBody = UTF8_TO_TCHAR(M.attach_body.c_str());
				Row->bAttachMirror = M.attach_mirror;
				Row->bHasPoint = M.attach_point.has_value();
				if (M.attach_point) { Row->PX = (*M.attach_point)[0]; Row->PY = (*M.attach_point)[1]; Row->PZ = (*M.attach_point)[2]; }
				Row->Attach = 0;
				if (!M.attach_entity.empty())
				{
					for (int32 i = 0; i < Choices.Num(); ++i)
					{
						const FAerotelicBodyChoice& C = *Choices[i];
						if (!C.Operand.Entity.IsValid() || C.Operand.Entity->EntityName != Row->AttachEntity || C.Operand.bMirror != M.attach_mirror) continue;
						if (!Row->AttachBody.IsEmpty() && C.Body != Row->AttachBody) continue;
						Row->Attach = i + 1;
						break;
					}
				}
				MassRows.Add(Row);
			}
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

			// Each chosen body through the shared builder (the commandlets
			// build the same way): its own grid as skin shells and its generated
			// layers as rib shells, labelled "<entity>[ (mirror)][ / <body>]".
			// The picker's index of the body is kept for a support resolved
			// against it alone (FSupport::Body).
			const AerotelicFeaModelBuilder::FOptions Options = BuilderOptions();
			AerotelicFeaModelBuilder::FBuilt Built2;
			for (const FAerotelicBodyChoicePtr& C : Bodies)
			{
				AerotelicFeaModelBuilder::AppendBody(C->Operand, C->Body, C->Label, Choices.IndexOfByKey(C), Options, RibCache, Built2);
			}
			B->FullNodes = std::move(Built2.Nodes);
			B->FullShellFaces = Built2.ShellFaces;
			B->FullRibFaces = Built2.RibFaces;
			B->FullShellAxes = Built2.ShellAxes;
			B->FullRibAxes = Built2.RibAxes;
			B->FullRegions = Built2.Regions;
			Request.Regions = Built2.Regions;
			B->Problems = Built2.Problems;
			if (B->FullShellFaces.Num() == 0)
			{
				Status(LOCTEXT("Ready", "Ready"));
				Message(FText::FromString(TEXT("No model could be built:\n") + FString::Join(B->Problems, TEXT("\n"))));
				return false;
			}
			Request.LengthToSI = Units.LengthToSI();
			Built = B;
			if (!Derive()) return false;
			const FModel& M = B->Model;
			Status(FText::FromString(FString::Printf(TEXT("Model built: %s nodes, %s wetted faces%s, %s nodes on the supports; %s -- see the Mesh, Loads and Support tabs"),
				*Thousands(M.Nodes), *Thousands(M.WettedFaces.Num()), M.RibFaces.Num() > 0 ? *FString::Printf(TEXT(", %s rib shells"), *Thousands(M.RibFaces.Num())) : TEXT(""),
				*Thousands(M.SupportNodes.Num()), *CostText(M))));
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
		FString CostText(const FModel& M) const
		{
			const bool bDirect = SolverIndex == 1 || (SolverIndex == 0 && M.DirectSolverAuto());
			FString Text = FString::Printf(TEXT("about %s DOF (six on each of %s shell nodes), %s"),
				*Thousands(M.Dofs), *Thousands(M.ShellNodes),
				bDirect ? TEXT("the direct solver") : TEXT("the iterative solver (amgcl, every core)"));
			if (OverBudget(M))
			{
				Text += FString::Printf(TEXT(" -- OVER the direct solver's budget of %s DOF on a desktop%s: coarsen the bodies' Mesh tab grids"),
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
			B.Nodes = B.FullNodes;
			B.Request.Nodes = &B.Nodes;
			B.Request.ShellFaces = B.FullShellFaces;
			B.Request.RibFaces = B.FullRibFaces;
			B.Request.ShellAxes = B.FullShellAxes;
			B.Request.RibAxes = B.FullRibAxes;
			B.Request.Regions = B.FullRegions;
			B.Dropped.Reset();
			ResolveMasses(B);
			ResolveHinges(B);
			ApplyAeroPressure(B);
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
				std::vector<aerotelic::Vec3> Trimmed;
				TArray<AerotelicStructuralFea::FShellFace> TrimmedFaces, TrimmedRibs;
				TArray<aerotelic::Vec3> TrimmedAxes, TrimmedRibAxes;
				// Which of each region's faces survive, in the region's order,
				// so a laminate's drape (per face per ply) is trimmed with them.
				TMap<int32, TArray<bool>> KeptSkinFaces, KeptRibFaces;
				TArray<int32> NewIndex;
				NewIndex.Init(-1, static_cast<int32>(B.Nodes.size()));
				auto Index = [&](int32 Old)
				{
					if (NewIndex[Old] < 0) { NewIndex[Old] = static_cast<int32>(Trimmed.size()); Trimmed.push_back(B.Nodes[Old]); }
					return NewIndex[Old];
				};
				const bool bAxes = B.Request.ShellAxes.Num() == B.Request.ShellFaces.Num() && B.Request.RibAxes.Num() == B.Request.RibFaces.Num();
				for (int32 i = 0; i < B.Request.ShellFaces.Num(); ++i)
				{
					const AerotelicStructuralFea::FShellFace& F = B.Request.ShellFaces[i];
					const bool bKeep = Kept(F.N[0]);
					KeptSkinFaces.FindOrAdd(F.Region).Add(bKeep);
					if (!bKeep) { ++DroppedPerRegion.FindOrAdd(F.Region, 0); continue; }
					AerotelicStructuralFea::FShellFace G = F;
					G.N[0] = Index(F.N[0]); G.N[1] = Index(F.N[1]); G.N[2] = Index(F.N[2]);
					TrimmedFaces.Add(G);
					if (bAxes) TrimmedAxes.Add(B.Request.ShellAxes[i]);
				}
				for (int32 i = 0; i < B.Request.RibFaces.Num(); ++i)
				{
					const AerotelicStructuralFea::FShellFace& F = B.Request.RibFaces[i];
					const bool bKeep = Kept(F.N[0]);
					KeptRibFaces.FindOrAdd(F.Region).Add(bKeep);
					if (!bKeep) { ++DroppedPerRegion.FindOrAdd(F.Region, 0); continue; }
					AerotelicStructuralFea::FShellFace G = F;
					G.N[0] = Index(F.N[0]); G.N[1] = Index(F.N[1]); G.N[2] = Index(F.N[2]);
					TrimmedRibs.Add(G);
					if (bAxes) TrimmedRibAxes.Add(B.Request.RibAxes[i]);
				}
				// The drapes of the regions that lost faces.
				auto TrimDrape = [](TArray<float>& Drape, const TArray<bool>* KeptFaces)
				{
					if (Drape.Num() == 0 || KeptFaces == nullptr || KeptFaces->Num() == 0 || Drape.Num() % (2 * KeptFaces->Num()) != 0) return;
					const int32 PerFace = Drape.Num() / KeptFaces->Num();
					TArray<float> Out;
					Out.Reserve(Drape.Num());
					for (int32 f = 0; f < KeptFaces->Num(); ++f)
					{
						if (!(*KeptFaces)[f]) continue;
						for (int32 k = 0; k < PerFace; ++k) Out.Add(Drape[f * PerFace + k]);
					}
					Drape = MoveTemp(Out);
				};
				for (AerotelicStructuralFea::FRegion& R : B.Request.Regions)
				{
					TrimDrape(R.SkinDrape, KeptSkinFaces.Find(R.Id));
					TrimDrape(R.RibDrape, KeptRibFaces.Find(R.Id));
				}
				for (const auto& Entry : DroppedPerRegion)
				{
					const AerotelicStructuralFea::FRegion* R = B.Request.Regions.FindByPredicate([&Entry](const AerotelicStructuralFea::FRegion& X) { return X.Id == Entry.Key; });
					const bool bWhole = M.FloatingRegions.Contains(Entry.Key);
					B.Dropped.Add(FString::Printf(TEXT("%s: %s element%s with no load path to any support left out%s"), R ? *R->Label : *FString::FromInt(Entry.Key),
						*Thousands(Entry.Value), Entry.Value == 1 ? TEXT("") : TEXT("s"), bWhole ? TEXT(" (the whole body: it touches neither a support plane nor a supported body)") : TEXT("")));
				}
				B.Nodes = std::move(Trimmed);
				B.Request.ShellFaces = TrimmedFaces;
				B.Request.RibFaces = TrimmedRibs;
				B.Request.ShellAxes = TrimmedAxes;
				B.Request.RibAxes = TrimmedRibAxes;
				// The regions that lost everything go too, or the solver would
				// want faces for them.
				B.Request.Regions.RemoveAll([&M](const AerotelicStructuralFea::FRegion& X) { return M.FloatingRegions.Contains(X.Id); });
				if (B.Request.ShellFaces.Num() == 0 && B.Request.RibFaces.Num() == 0)
				{
					Status(LOCTEXT("Ready", "Ready"));
					Message(FText::FromString(TEXT("Nothing reaches a support: every chosen body floats free of the support planes.\n") + FString::Join(B.Dropped, TEXT("\n"))));
					return false;
				}
				// The nodes were renumbered: the mass items hang on the new ones,
				// and the hinges tie the new ones.
				{
					TArray<FString> KeptNotes;
					for (const FString& D : B.Dropped) if (!D.StartsWith(TEXT("mass item"))) KeptNotes.Add(D);
					B.Dropped = KeptNotes;
				}
				ResolveMasses(B);
				ResolveHinges(B);
				ApplyAeroPressure(B);
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
			const std::vector<aerotelic::Vec3>& Mesh = Built->Nodes;
			for (int32 n = 0; n < static_cast<int32>(Mesh.size()); ++n)
			{
				bool bFixed = false;
				for (FModel::FPlane& P : Planes)
				{
					if (!AerotelicFeaPreprocess::NodeOnPlane(Mesh[n], P)) continue;
					++P.Nodes;
					P.Furthest = FMath::Max(P.Furthest, std::abs(Mesh[n][P.Axis] - P.Value));
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
				for (const AerotelicStructuralFea::FShellFace& F : B.Request.ShellFaces)
				{
					if (!Regions.Contains(F.Region)) continue;
					for (int32 k = 0; k < 3; ++k) Box.Add(B.Nodes[F.N[k]]);
				}
				for (const AerotelicStructuralFea::FShellFace& F : B.Request.RibFaces)
				{
					if (!Regions.Contains(F.Region)) continue;
					for (int32 k = 0; k < 3; ++k) Box.Add(B.Nodes[F.N[k]]);
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
				const FString SupportText = M.bFreeFlight
					? FString::Printf(TEXT("free flight, the reference node %d"), M.SupportNode)
					: FString::Printf(TEXT("%d support plane(s) fixing %s nodes"), M.SupportPlanes.Num(), *Thousands(M.SupportNodes.Num()));
				Status(FText::FromString(FString::Printf(TEXT("Applied: %s, %d mass item(s), %d point force(s)%s%s%s"),
					*SupportText, M.MassItems.Num(), M.PointLoads.Num(),
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
			const std::vector<aerotelic::Vec3>& Mesh = B.Nodes;
			const double Diag = FMath::Max(M.Diagonal, 1e-9);
			auto At = [&Mesh](int32 Node) { const aerotelic::Vec3& P = Mesh[Node]; return FVector(P.x, P.y, P.z); };
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
				FString Text = FString::Printf(TEXT("%s nodes, %s wetted faces (their edges drawn; where a region has a skin these faces ARE its shell elements)%s"),
					*Thousands(M.Nodes), *Thousands(M.WettedFaces.Num()),
					M.RibFaces.Num() > 0 ? *FString::Printf(TEXT(", %s rib shells inside"), *Thousands(M.RibFaces.Num())) : TEXT(""));
				Text += TEXT("\nCost: ") + CostText(M);
				for (int32 i = 0; i < M.Regions.Num(); ++i)
				{
					const AerotelicFeaPreprocess::FRegionSummary& R = M.Regions[i];
					Text += FString::Printf(TEXT("\nregion %d, %s (%s): %s faces of %.4g %s^2%s, mass %.4g kg"),
						R.Id, *R.Label, *R.Kind, *Thousands(R.Faces), R.Area, *LengthUnit,
						R.RibFaces > 0 ? *FString::Printf(TEXT(", %s rib shells of %.4g %s^2"), *Thousands(R.RibFaces), R.RibArea, *LengthUnit) : TEXT(""), R.Mass);
				}
				Text += TEXT("\nThe faces are each body's OWN grid: the triangles its Mesh tab defines and the viewport shows; its generated structure (a wing's ribs, spars, stringers) joins them as shells on the same nodes.");
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
					if (B.AeroCase.IsValid())
					{
						const FAerotelicAeroLoadCase& A = *B.AeroCase;
						// The panel method's own resultant in newtons: C_F q S_ref.
						const aerotelic::Vec3 Fc = A.ForceCoefficient();
						const double QS = A.DynamicPressure * A.ReferenceArea * Units.LengthToSI() * Units.LengthToSI();
						const FVector PanelForce(Fc.x * QS, Fc.y * QS, Fc.z * QS);
						Text += FString::Printf(TEXT("Pressure (orange): the load case '%s' (α = %g°%s, q = %.4g Pa, %d body panels%s) on %s of the %s wetted faces%s%s -- arrows on %s of them, the traction's direction and relative size, not the face force. Resultant (%.4g, %.4g, %.4g) N; the panel method's own, -∮ Cp n dA over its panels, is (%.4g, %.4g, %.4g) N, so the mapping keeps %.1f%% of the lift."),
							*A.Name, A.AlphaDeg, A.Control.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(", %s at δ = %g°"), *A.Control, A.DeltaDeg), A.DynamicPressure, A.BodyPanels(), A.bTrimmed ? *FString::Printf(TEXT(", trimmed to n = %g at W = %.4g N"), A.LoadFactor, A.Weight) : TEXT(", not trimmed"),
							*Thousands(B.AeroMap.Stats.mapped), *Thousands(NFaces),
							B.Request.Pressure != 0.0 ? *FString::Printf(TEXT(", plus %.4g %s uniform on every face"), B.Request.Pressure / S2, *StressUnit) : TEXT(""),
							B.AeroNotes.Num() ? *(TEXT("; ") + FString::Join(B.AeroNotes, TEXT("; "))) : TEXT(""), *Thousands(Drawn),
							M.PressureResultant.X, M.PressureResultant.Y, M.PressureResultant.Z,
							PanelForce.X, PanelForce.Y, PanelForce.Z,
							FMath::Abs(PanelForce.Z) > 1e-12 ? 100.0 * B.AeroMap.Resultant.Z / PanelForce.Z : 0.0);
						if (A.bTrimmed && A.Weight > 0.0)
						{
							Text += FString::Printf(TEXT("\n  Trim check: the case was trimmed to n W = %.4g N; the mapped faces carry %.4g N vertically (%.3f of it)%s"),
								A.LoadFactor * A.Weight, B.AeroMap.Resultant.Z, B.AeroMap.Resultant.Z / (A.LoadFactor * A.Weight),
								M.bFreeFlight ? TEXT(". In free flight the load factor at the mass centre below should read the case's n.") : TEXT("."));
						}
					}
					else
					{
						Text += FString::Printf(TEXT("Pressure (orange): %.4g %s on every one of the %s wetted faces, %s; resultant (%.4g, %.4g, %.4g) N -- arrows on %s of them, the traction's direction and relative size, not the face force"),
							B.Request.Pressure / S2, *StressUnit, *Thousands(NFaces), B.Request.Pressure > 0.0 ? TEXT("pressing inward") : TEXT("pulling outward"),
							M.PressureResultant.X, M.PressureResultant.Y, M.PressureResultant.Z, *Thousands(Drawn));
					}
				}
				else
				{
					Text += TEXT("Pressure: none");
					if (B.AeroNotes.Num() > 0) Text += TEXT(" (") + FString::Join(B.AeroNotes, TEXT("; ")) + TEXT(")");
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
					Text += FString::Printf(TEXT("\nGravity (green): %.4g g on the model's mass of %.5g kg (the shells' rho t A, ribs included%s): (%.4g, %.4g, %.4g) N, drawn at the centre of mass (%.4g, %.4g, %.4g) %s; applied as a body force on every element and mass item"),
						B.Request.LoadFactor, M.Mass, M.MassItems.Num() ? *FString::Printf(TEXT(", and %.4g kg in %d mass item(s)"), M.MassOfItems, M.MassItems.Num()) : TEXT(""),
						M.GravityForce.X, M.GravityForce.Y, M.GravityForce.Z, Centre.X, Centre.Y, Centre.Z, *LengthUnit);
				}
				else
				{
					Text += TEXT("\nGravity: none (load factor 0)");
				}
				if (M.bFreeFlight)
				{
					const FVector F = M.PressureResultant + M.PointResultant + M.GravityForce;
					Text += FString::Printf(TEXT("\nFREE FLIGHT: the loads above sum to (%.4g, %.4g, %.4g) N, so the free body accelerates at (%.4g, %.4g, %.4g) m/s^2 and (%.4g, %.4g, %.4g) rad/s^2 about its mass centre; the inertial field of that motion (-m a on every element and mass item) is added so the load set balances, and the structure feels %.3g g at its mass centre%s"),
						F.X, F.Y, F.Z, M.RigidAcceleration.X, M.RigidAcceleration.Y, M.RigidAcceleration.Z,
						M.RigidAngularAcceleration.X, M.RigidAngularAcceleration.Y, M.RigidAngularAcceleration.Z, M.LoadFactorAtCg,
						M.bInertiaSingular ? TEXT(" (WARNING: the inertia tensor is singular -- the mass lies on a line -- so the angular part could not be found)") : TEXT(""));
					if (F.Size() > 1e-6 * FMath::Max(M.GravityForce.Size(), 1.0))
					{
						Text += TEXT("\n  A trimmed case has no net force and no net moment: lift equal to the weight at the mass centre. What the loads miss shows up as the acceleration above.");
					}
				}
				Text += TEXT("\nArrow heads sit at the point of application; forces are newtons whatever the project's units. Edit the loads on the left and Apply.");
				if (IsReplica())
				{
					Text += FString::Printf(TEXT("\nCoordinates here and on the left are the FULL-SCALE aircraft's; the model is built at %s of that size (Model > Scale) and the picture shows the replica."), *ScaleText());
				}
				ShowPicture(TabLoads, { &G, &Arrows }, { false, false });
				if (Captions[TabLoads].IsValid()) Captions[TabLoads]->SetText(FText::FromString(Text));
			}

			// Masses: the surface in grey, each mass item as a marker sized by
			// its mass with bars to the nodes it hangs on, the mass centre in
			// green.
			{
				FGeom G = Surface([&](const FFace&) { return kGrey; });
				FGeom Marks;
				FString Text;
				const FString MassUnit = UnitName(Units.MassName());
				const double M2 = Units.MassToSI();
				double Biggest = 0.0;
				for (const FModel::FMassItemView& I : M.MassItems) Biggest = FMath::Max(Biggest, I.Mass);
				for (int32 i = 0; i < M.MassItems.Num(); ++i)
				{
					const FModel::FMassItemView& I = M.MassItems[i];
					const double R = (0.008 + 0.025 * FMath::Pow(I.Mass / FMath::Max(Biggest, 1e-300), 1.0 / 3.0)) * Diag;
					AddOctahedron(Marks, I.At, R, kPointForce);
					for (int32 N : I.AttachNodes) AddBar(Marks, I.At, At(N), 0.0015 * Diag, kHandle);
					const FVector P = Full(I.At), C = Full(I.AttachCentre);
					Text += FString::Printf(TEXT("%s%s (red): %.4g %s at (%.4g, %.4g, %.4g) %s, hung on %d nodes about (%.4g, %.4g, %.4g) %s (amber bars) -- an RBE3-type attachment: its load is shared out to those nodes, which it does not stiffen"),
						i ? TEXT("\n") : TEXT(""), *I.Name, I.Mass / M2, *MassUnit, P.X, P.Y, P.Z, *LengthUnit, I.AttachNodes.Num(), C.X, C.Y, C.Z, *LengthUnit);
				}
				if (M.MassItems.Num() == 0) Text += TEXT("No mass item in the model: add one on the left (a battery, a motor, the payload), choose the body it hangs on, Apply.");
				AddOctahedron(Marks, M.MassCentre, 0.01 * Diag, kGravity);
				const FVector Centre = Full(M.MassCentre);
				Text += FString::Printf(TEXT("\nMass: %.5g %s in all -- the shells %.5g %s%s; mass centre (green) at (%.4g, %.4g, %.4g) %s; inertia about it (kg m^2, world axes) Ixx %.4g, Iyy %.4g, Izz %.4g, Ixy %.3g, Ixz %.3g, Iyz %.3g"),
					M.Mass / M2, *MassUnit, M.MassStructure / M2, *MassUnit, M.MassItems.Num() ? *FString::Printf(TEXT(", the items %.5g %s"), M.MassOfItems / M2, *MassUnit) : TEXT(""),
					Centre.X, Centre.Y, Centre.Z, *LengthUnit, M.InertiaCg[0], M.InertiaCg[4], M.InertiaCg[8], -M.InertiaCg[1], -M.InertiaCg[2], -M.InertiaCg[5]);
				for (const FString& D : B.Dropped) if (D.StartsWith(TEXT("mass item"))) Text += TEXT("\nLEFT OUT: ") + D;
				if (IsReplica())
				{
					Text += FString::Printf(TEXT("\nPositions are the FULL-SCALE aircraft's and are scaled to the replica (%s); masses and inertias are as built, not scaled."), *ScaleText());
				}
				ShowPicture(TabMasses, { &G, &Marks }, { false, false });
				if (Captions[TabMasses].IsValid()) Captions[TabMasses]->SetText(FText::FromString(Text));
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
			const std::vector<aerotelic::Vec3>& Mesh = B.Nodes;
			const double Diag = FMath::Max(M.Diagonal, 1e-9);
			auto At = [&Mesh](int32 Node) { const aerotelic::Vec3& P = Mesh[Node]; return FVector(P.x, P.y, P.z); };
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
			// The hinges and pushrods (2026-11-14): each tie an amber bar from
			// the flap's node to the main element's, each offset link a thin
			// amber bar from its node to where it stands, each pushrod a red
			// bar between its ends.
			if (M.Ties.Num() > 0 || M.Links.Num() > 0 || M.Bars.Num() > 0 || B.Hinges.Num() > 0)
			{
				for (const FModel::FTieView& T : M.Ties)
				{
					AddBar(Marks, At(T.Master), At(T.Slave), 0.003 * Diag, kHandle);
					AddOctahedron(Marks, At(T.Master), 0.006 * Diag, kHandle);
					AddOctahedron(Marks, At(T.Slave), 0.006 * Diag, kHandle);
				}
				for (const FModel::FLinkView& K : M.Links)
				{
					if (K.On >= 0 && K.On < M.Nodes) AddBar(Marks, At(K.On), K.At, 0.0015 * Diag, kHandle);
					AddOctahedron(Marks, K.At, 0.005 * Diag, kHandle);
				}
				for (const FModel::FBarView& Bar : M.Bars) AddBar(Marks, Bar.A, Bar.B, 0.004 * Diag, kPointForce);
				if (!Text.IsEmpty()) Text += TEXT("\n");
				Text += FString::Printf(TEXT("HINGES: %d bracket tie(s) (amber bars: the flap's node follows the main element's in translation, free to turn), %d offset link(s) (horns and servo arms, amber), %d pushrod(s) (red, pin-ended bars; their axial forces are reported with the result)."),
					M.Ties.Num(), M.Links.Num(), M.Bars.Num());
				for (const FString& H : B.Hinges) Text += TEXT("\n  ") + H;
			}
			if (M.bFreeFlight)
			{
				const FVector Ref = Full(M.Reference), SupportAt = Full(M.SupportAt), Centre = Full(M.MassCentre);
				AddOctahedron(Marks, M.MassCentre, 0.01 * Diag, kGravity);
				if (!bReferenceAtCg) AddOctahedron(Marks, M.Reference, 0.008 * Diag, kHandle);
				Text += FString::Printf(TEXT("FREE FLIGHT (inertia relief): no plane holds the model. The loads are balanced by the inertial field of the rigid-body motion they would give the free body (see the Loads tab), and ONE node -- node %d at (%.4g, %.4g, %.4g) %s, the nearest to %s -- is fixed in its six degrees of freedom as a statically determinate support: it cannot load the structure, its reaction force and moment are the residual of the whole load set (zero to round-off when the solve is right) and are reported as the check, and the displacements are measured from it. The stresses do not depend on which node it is."),
					M.SupportNode, SupportAt.X, SupportAt.Y, SupportAt.Z, *LengthUnit,
					bReferenceAtCg ? *FString::Printf(TEXT("the mass centre (green, (%.4g, %.4g, %.4g) %s)"), Centre.X, Centre.Y, Centre.Z, *LengthUnit)
					               : *FString::Printf(TEXT("the reference point (amber, (%.4g, %.4g, %.4g) %s)"), Ref.X, Ref.Y, Ref.Z, *LengthUnit));
				if (M.SupportNode < 0) Text += TEXT("\nNO NODE: the model is empty.");
			}
			else if (Planes.Num() == 0) Text += TEXT("No support: add a plane on the left and Apply.");
			if (IsReplica())
			{
				Text += FString::Printf(TEXT("\nCoordinates here and on the left are the FULL-SCALE aircraft's; the model is built at %s of that size (Model > Scale) and the picture shows the replica."), *ScaleText());
			}
			if (M.bFreeFlight)
			{
				Text += TEXT("\nFixed node (red): the one reference node, every degree of freedom.");
			}
			else
			{
				Text += FString::Printf(TEXT("\nFixed nodes (red): %s of %s, every degree of freedom -- the three displacements and the three rotations: a clamp"),
					*Thousands(NSupport), *Thousands(M.Nodes));
				if (Stride > 1) Text += FString::Printf(TEXT(" (one marker in %d drawn)"), Stride);
				if (NSupport < 3) Text += TEXT("\nFEWER THAN THREE NODES: the solver refuses this support. Move a plane onto the geometry (or use the mesh's minimum / maximum), widen the tolerance or enlarge the patch.");
			}
			if (!bLive)
			{
				if (B.Dropped.Num() > 0)
				{
					Text += M.bFreeFlight ? TEXT("\nLEFT OUT, not joined to the piece that holds the reference node (a loose piece in free flight has rigid motions of its own and would make the system singular):")
					                      : TEXT("\nLEFT OUT, no load path to any support (they would make the system singular):");
					for (const FString& D : B.Dropped) Text += TEXT("\n  ") + D;
				}
				else
				{
					Text += M.bFreeFlight ? FString::Printf(TEXT("\nEvery element is joined to the piece holding the reference node (%d connected piece%s)."), M.Components, M.Components == 1 ? TEXT("") : TEXT("s"))
					                      : FString::Printf(TEXT("\nLoad path: every element reaches a support (%d connected piece%s)."), M.Components, M.Components == 1 ? TEXT("") : TEXT("s"));
				}
			}
			Text += M.bFreeFlight ? TEXT("\nChoose the mode and the reference on the left and Apply.")
			                      : TEXT("\nEdit the supports on the left and Apply, or drag a patch's corner or edge handles here (the fixed nodes follow as you drag; the model is re-derived when you let go).");
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
			if (M.bFreeFlight && M.SupportNode < 0)
			{
				Message(LOCTEXT("NoReferenceNode", "Free flight needs a reference node and the model has none."));
				return;
			}
			if (!M.bFreeFlight && M.SupportNodes.Num() < 3)
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
						TEXT("A faster, exact solve: coarsen the bodies' Mesh tab grids (the shells are built on them, a node per vertex)."))),
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
			ShowProgress(FString::Printf(TEXT("starting the solver on %s shells (%s wetted faces, %s rib shells)"), *Thousands((int64)M.WettedFaces.Num() + M.RibFaces.Num()), *Thousands(M.WettedFaces.Num()), *Thousands(M.RibFaces.Num())));
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
			LblMesh->SetText(FText::FromString(FString::Printf(TEXT("%s nodes; %s shells%s%s; %s DOF; %s wetted faces; %s"),
				*Thousands(R.Nodes), *Thousands(R.Shells), R.RibShells > 0 ? *FString::Printf(TEXT(" (%s of them rib shells, unloaded)"), *Thousands(R.RibShells)) : TEXT(""),
				*(FString(R.Items.Num() > 0 ? *FString::Printf(TEXT("; %d mass item(s), %.5g %s"), R.Items.Num(), R.MassItems / M2, *UnitName(Units.MassName())) : TEXT(""))
					+ (R.Ties > 0 || R.Bars.Num() > 0 ? FString::Printf(TEXT("; %d hinge tie(s), %d pushrod(s)"), R.Ties, R.Bars.Num()) : FString())),
				*Thousands(R.Dofs), *Thousands(R.WettedFaces),
				R.bFreeFlight ? *FString::Printf(TEXT("free flight, the reference node %d"), R.SupportNode) : *FString::Printf(TEXT("%s nodes on the supports"), *Thousands(R.SupportNodes)))));
			LblDisp->SetText(FText::FromString(FString::Printf(TEXT("%.4g %s at (%.4g, %.4g, %.4g); mean surface displacement %s (u . n = %.3g %s)"),
				R.MaxDisplacement / L2, *UnitName(Units.LengthName()), R.MaxDisplacementAt[0] / L2, R.MaxDisplacementAt[1] / L2, R.MaxDisplacementAt[2] / L2,
				R.MeanNormalDisplacement < 0.0 ? TEXT("inward") : TEXT("outward"), R.MeanNormalDisplacement / L2, *UnitName(Units.LengthName()))));
			FString Peaks;
			for (const AerotelicStructuralFea::FEntryResult& E : R.Entries)
			{
				if (E.IsLaminate())
				{
					// A laminate: Tsai-Wu's reserve factor is the margin's measure;
					// the von Mises figure is a stress magnitude, not a criterion.
					FString Drape;
					if (E.bDraped)
					{
						Drape = FString::Printf(TEXT("; draped: cloth sheared up to %.1f deg, fibres turned up to %.1f deg%s"), E.DrapeMaxShear, E.DrapeMaxDeviation,
							E.DrapeFacesPastLocking > 0 ? *FString::Printf(TEXT(", %d face-plies PAST THE LOCKING ANGLE (wrinkles)"), E.DrapeFacesPastLocking) : TEXT(""));
					}
					Peaks += FString::Printf(TEXT("%s%s [%s]: laminate of %d plies, %.4g %s, in %d stack(s); %s; von Mises %.4g %s%s"), Peaks.IsEmpty() ? TEXT("") : TEXT("\n"), *E.Label, *E.Kind,
						E.Plies, E.Thickness / L2, *UnitName(Units.LengthName()), E.Stacks,
						E.ReserveFactor > 0.0 ? *FString::Printf(TEXT("Tsai-Wu reserve factor %.3g (margin %+.2f)"), E.ReserveFactor, E.Margin) : TEXT("no strengths on a lamina, so no margin"),
						E.MaxVonMises / S2, *UnitName(Units.StressName()), *Drape);
					continue;
				}
				Peaks += FString::Printf(TEXT("%s%s [%s]: %.4g %s%s"), Peaks.IsEmpty() ? TEXT("") : TEXT("\n"), *E.Label, *E.Kind, E.MaxVonMises / S2, *UnitName(Units.StressName()),
					E.Allowable > 0.0 ? *FString::Printf(TEXT(", margin %+.2f against %.4g"), E.Margin, E.Allowable / S2) : TEXT(" (no allowable set)"));
			}
			// The pushrods (2026-11-14): the axial force each carries, tension
			// positive, and the stress in it (no allowable is set on a rod).
			for (const AerotelicStructuralFea::FResult::FBarResult& Bar : R.Bars)
			{
				Peaks += FString::Printf(TEXT("%s%s [pushrod]: %.4g N %s, %.4g %s over %.4g %s, %.4g %s"), Peaks.IsEmpty() ? TEXT("") : TEXT("\n"), *Bar.Name,
					FMath::Abs(Bar.Force), Bar.Force >= 0.0 ? TEXT("tension") : TEXT("compression"), FMath::Abs(Bar.Stress) / S2, *UnitName(Units.StressName()),
					Bar.Length / L2, *UnitName(Units.LengthName()), Bar.Mass / M2, *UnitName(Units.MassName()));
			}
			if (R.Ties > 0 && R.Bars.Num() == 0)
			{
				Peaks += FString::Printf(TEXT("%s%d hinge tie(s) and NO pushrod: a hinged flap with nothing to hold its rotation; if the balance below is off, that is why"), Peaks.IsEmpty() ? TEXT("") : TEXT("\n"), R.Ties);
			}
			LblStress->SetText(FText::FromString(FString::Printf(TEXT("%.4g %s, in region %d"), R.MaxVonMises / S2, *UnitName(Units.StressName()), R.MaxVonMisesRegion)));
			LblMargins->SetText(FText::FromString(Peaks));
			const double WorstError = FMath::Max(R.BalanceError, R.MomentError);
			FString Verdict = R.bBalanced
				? FString::Printf(TEXT("Balanced to %.2g%%."), 100.0 * WorstError)
				: FString::Printf(TEXT("NOT CONVERGED: the reactions miss the applied loads by %.1f%%, so the linear solver (%s) did not converge and the displacements and stresses above mean nothing. Solve again with the direct solver."), 100.0 * WorstError, *R.LinearSolver);
			if (B.AeroCase.IsValid())
			{
				const FAerotelicAeroLoadCase& A = *B.AeroCase;
				Verdict += FString::Printf(TEXT(" The pressure is the load case '%s' (q %.4g Pa, α %g°%s)%s."), *A.Name, A.DynamicPressure, A.AlphaDeg,
					A.Control.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(", δ %g°"), A.DeltaDeg),
					A.bTrimmed && R.bFreeFlight ? *FString::Printf(TEXT(", trimmed to n = %g: the structure feels %.3f g at its mass centre, %s"), A.LoadFactor, R.LoadFactorAtCg,
						FMath::Abs(R.LoadFactorAtCg - A.LoadFactor) < 0.1 * FMath::Max(FMath::Abs(A.LoadFactor), 0.1) ? TEXT("the two models agree") : TEXT("NOT the trim's figure: the mass flown at differs from this model's, or the mapping lost lift (see the Loads tab)")) : TEXT(""));
			}
			if (R.bFreeFlight)
			{
				FString Items;
				for (const AerotelicStructuralFea::FResult::FItem& I : R.Items)
				{
					Items += FString::Printf(TEXT("\n%s: %.4g %s on %d nodes, moved %.4g %s, puts (%.4g, %.4g, %.4g) N on the structure"), *I.Name, I.Mass / M2, *UnitName(Units.MassName()), I.AttachNodes,
						FVector(I.Displacement[0], I.Displacement[1], I.Displacement[2]).Size() / L2, *UnitName(Units.LengthName()), I.Force[0], I.Force[1], I.Force[2]);
				}
				LblBalance->SetText(FText::FromString(FString::Printf(
					TEXT("FREE FLIGHT. Mass %.6g %s (%.5g in %d mass item(s)), mass centre (%.4g, %.4g, %.4g) %s. The loads -- gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point forces (%.4g, %.4g, %.4g) N -- give the free body a0 = (%.4g, %.4g, %.4g) m/s^2 and alpha = (%.4g, %.4g, %.4g) rad/s^2, so the structure feels %.4g g at its mass centre; the inertial field (%.4g, %.4g, %.4g) N balances them. The reference node %d reacts (%.3g, %.3g, %.3g) N and (%.3g, %.3g, %.3g) N m: %s%s"),
					R.Mass / M2, *UnitName(Units.MassName()), R.MassItems / M2, R.Items.Num(), R.MassCentre[0] / L2, R.MassCentre[1] / L2, R.MassCentre[2] / L2, *UnitName(Units.LengthName()),
					R.AppliedGravity[0], R.AppliedGravity[1], R.AppliedGravity[2], R.AppliedPressure[0], R.AppliedPressure[1], R.AppliedPressure[2],
					R.AppliedPoint[0], R.AppliedPoint[1], R.AppliedPoint[2],
					R.RigidAcceleration[0], R.RigidAcceleration[1], R.RigidAcceleration[2], R.RigidAngularAcceleration[0], R.RigidAngularAcceleration[1], R.RigidAngularAcceleration[2],
					R.LoadFactorAtCg, R.AppliedInertial[0], R.AppliedInertial[1], R.AppliedInertial[2],
					R.SupportNode, R.Reaction[0], R.Reaction[1], R.Reaction[2], R.ReactionMoment[0], R.ReactionMoment[1], R.ReactionMoment[2], *Verdict, *Items)));
			}
			else
			{
				LblBalance->SetText(FText::FromString(FString::Printf(
					TEXT("mass %.6g %s%s. Reactions (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point forces (%.4g, %.4g, %.4g) N. %s"),
					R.Mass / M2, *UnitName(Units.MassName()), R.Items.Num() ? *FString::Printf(TEXT(" (%.5g in %d mass item(s))"), R.MassItems / M2, R.Items.Num()) : TEXT(""),
					R.Reaction[0], R.Reaction[1], R.Reaction[2],
					R.AppliedGravity[0], R.AppliedGravity[1], R.AppliedGravity[2], R.AppliedPressure[0], R.AppliedPressure[1], R.AppliedPressure[2],
					R.AppliedPoint[0], R.AppliedPoint[1], R.AppliedPoint[2], *Verdict)));
			}
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
			LblTime->SetText(FText::FromString(FString::Printf(TEXT("%.1f s (%.1f solving with %s%s)"), R.Seconds, R.SecondsSolve, *R.LinearSolver,
				R.SecondsModes > 0.0 ? *FString::Printf(TEXT(", %.1f on the natural frequencies"), R.SecondsModes) : TEXT(""))));
			for (const FString& W : R.Warnings)
			{
				UE_LOG(LogTemp, Warning, TEXT("StructuralFea: %s"), *W);
			}
			if (R.SupportCornerNodes != M.SupportNodes.Num() || R.WettedFaces != M.WettedFaces.Num() || R.Dofs != M.Dofs || (R.bFreeFlight && R.SupportNode != M.SupportNode))
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

		// The Masses tab's list: the document's items as they stand, read-only
		// (Analysis > Mass Properties edits them); Reload takes the document's
		// current items and re-derives.
		void RebuildMassList()
		{
			if (!MassList.IsValid()) return;
			MassList->ClearChildren();
			for (int32 i = 0; i < MassRows.Num(); ++i)
			{
				const TSharedPtr<FMassRow>& R = MassRows[i];
				const FString Attach = R->AttachEntity.IsEmpty() ? TEXT("nothing") : R->AttachEntity + (R->bAttachMirror ? TEXT(" (mirror)") : TEXT("")) + (R->AttachBody.IsEmpty() ? TEXT("") : TEXT(" / ") + R->AttachBody)
					+ (R->bHasPoint ? FString::Printf(TEXT(" at (%.4g, %.4g, %.4g)"), R->PX, R->PY, R->PZ) : FString());
				MassList->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
				[
					SNew(SBorder).Padding(6).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(STextBlock).AutoWrapText(true).Text(FText::FromString(FString::Printf(TEXT("%s: %.4g %s at (%.4g, %.4g, %.4g); Ixx %.3g, Iyy %.3g, Izz %.3g, Ixy %.3g, Ixz %.3g, Iyz %.3g; hangs on %s, %d nodes"),
							*R->Name, R->Mass, *UnitName(Units.MassName()), R->X, R->Y, R->Z, R->I[0], R->I[1], R->I[2], R->I[3], R->I[4], R->I[5], *Attach, R->Nodes)))
					]
				];
			}
			if (MassRows.Num() == 0)
			{
				MassList->AddSlot().AutoHeight() [ SNew(STextBlock).AutoWrapText(true).Text(LOCTEXT("NoMasses", "No point masses on the project. Add them under Analysis > Mass Properties, Point masses tab.")) ];
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
			LblProgress.Reset(); ProgressBar.Reset(); BuildButton.Reset(); SolveButton.Reset(); CancelButton.Reset();
			for (int32 Tab = 0; Tab < kPictures; ++Tab)
			{
				for (TWeakObjectPtr<AAerotelicColoredMeshActor>& A : Actors[Tab]) if (A.IsValid()) A->Destroy();
				Actors[Tab].Empty();
				Views[Tab].Reset();
				Captions[Tab].Reset();
			}
			AeroField.Reset();
			Built.Reset();
			Picker.Reset();
			PointList.Reset(); SupportList.Reset(); MassList.Reset();
			PathBox.Reset();
			LblMaterials.Reset(); LblMesh.Reset(); LblDisp.Reset(); LblStress.Reset(); LblMargins.Reset(); LblBalance.Reset(); LblModes.Reset(); LblTime.Reset();
			Tabs.Reset();
		}
	};

	// A picture tab: an optional panel on the left (the loads, the
	// supports), the view with its caption on the right.
	inline TSharedRef<SWidget> MakePictureTab(TSharedRef<FState> S, int32 Tab, const FText& Empty, TSharedPtr<SWidget> SidePanel)
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

	// ---- the editing panels (2026-11-15: what the Kratos dialog's Loads
	// and Support tabs carried on their left; Model > Load Cases shows them
	// now, the Kratos dialog only the pictures) ----

	inline TSharedRef<SWidget> MakeStateSpin(TSharedRef<FState> S, double FState::*Member, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([S, Member]() { return (*S).*Member; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([S, Member](double V, ETextCommit::Type) { (*S).*Member = V; });
	}

	// The inertial load factor, the uniform pressure, the point forces, Apply.
	inline TSharedRef<SWidget> MakeLoadsPanel(TSharedRef<FState> S)
	{
		const FText StressSuffix = FText::FromString(TEXT(" ") + UnitName(S->Units.StressName()));
		return SNew(SScrollBox)
			+ SScrollBox::Slot().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("InertialGroup", "Inertial load"),
					FormRow(LOCTEXT("LoadFactor", "Load factor:"), MakeStateSpin(S, &FState::LoadFactor, -100.0, 100.0, 0.5, FText::FromString(TEXT(" g"))), 110.0f))
			]
			+ SScrollBox::Slot().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("PressureGroup", "Pressure on every wetted face"),
					FormRow(LOCTEXT("Pressure", "Pressure:"), MakeStateSpin(S, &FState::Pressure, -1e12, 1e12, 100.0, StressSuffix), 110.0f))
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
					"The load factor puts the model's own mass under that many g along -z (1 is its weight, a 4 g pull-up a real case; 0 lets a test load act alone). The pressure acts on every wetted face, positive pressing inwards. Each point force goes to the corner node nearest its point; the picture warns when that node is more than a mesh edge away. Points are typed in FULL-SCALE coordinates, as the parameterisations are; with a model scale set (Model > Scale) they are scaled to the replica. Forces are newtons whatever the project's units or scale. Apply pushes the loads onto the built model and redraws them."))
			];
	}

	// The document's point masses, read-only, with Reload.
	inline TSharedRef<SWidget> MakeMassesPanel(TSharedRef<FState> S)
	{
		return SNew(SScrollBox)
			+ SScrollBox::Slot().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("MassesGroup", "Point masses (Analysis > Mass Properties)"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(S->MassList, SVerticalBox) ]
					+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)
					[ Button(LOCTEXT("ReloadMasses", "Reload from the project"), [S]() { S->LoadMassRows(); S->RebuildMassList(); if (S->Built.IsValid()) S->ApplyInputs(); }) ])
			]
			+ SScrollBox::Slot()
			[
				MakeInfoLabel(LOCTEXT("MassesHint",
					"The project's point masses -- a battery, a motor, the payload -- as defined under Analysis > Mass Properties (Point masses tab), where each is given its mass, its centre, its inertia about that centre and the body and grid node it hangs on. Here each is hung on the built model by an RBE3-type constraint on the nodes of its body nearest the picked node (the item's weight and inertial load shared out with the right resultant and moment, the nodes not stiffened). A body that has become an operand of a boolean result hands the mass on. One hung on nothing, or on a body with no path into the model, is left out and named on the right. Reload takes the project's items as they now stand."))
			];
	}

	// The support mode, the planes and patches, the free-flight reference, Apply.
	inline TSharedRef<SWidget> MakeSupportPanel(TSharedRef<FState> S)
	{
		const FText LengthSuffix = FText::FromString(TEXT(" ") + UnitName(S->Units.LengthName()));
		return SNew(SScrollBox)
			+ SScrollBox::Slot().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("SupportModeGroup", "Support mode"),
					SNew(SComboBox<TSharedPtr<FString>>)
					.OptionsSource(&S->SupportModeOptions)
					.InitiallySelectedItem(S->SupportModeOptions[FMath::Clamp(S->SupportMode, 0, S->SupportModeOptions.Num() - 1)])
					.OnGenerateWidget_Lambda([](TSharedPtr<FString> Item) { return SNew(STextBlock).Text(FText::FromString(*Item)); })
					.OnSelectionChanged_Lambda([S](TSharedPtr<FString> Item, ESelectInfo::Type)
					{
						const int32 I = S->SupportModeOptions.IndexOfByKey(Item);
						if (I != INDEX_NONE) S->SupportMode = I;
					})
					[ SNew(STextBlock).Text_Lambda([S]() { return FText::FromString(*S->SupportModeOptions[FMath::Clamp(S->SupportMode, 0, 1)]); }) ])
			]
			+ SScrollBox::Slot().Padding(0, 0, 0, 8)
			[
				SNew(SBox).Visibility_Lambda([S]() { return S->IsFreeFlight() ? EVisibility::Collapsed : EVisibility::Visible; })
				[
					MakeGroup(LOCTEXT("SupportsGroup", "Support planes"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(S->SupportList, SVerticalBox) ]
						+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)
						[ Button(LOCTEXT("AddSupport", "Add support"), [S]() { S->SupportRows.Add(MakeShared<FSupportRow>()); S->RebuildSupportList(); }) ])
				]
			]
			+ SScrollBox::Slot().Padding(0, 0, 0, 8)
			[
				SNew(SBox).Visibility_Lambda([S]() { return S->IsFreeFlight() ? EVisibility::Visible : EVisibility::Collapsed; })
				[
					MakeGroup(LOCTEXT("ReferenceGroup", "Reference node (free flight)"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[
							SNew(SCheckBox)
							.IsChecked_Lambda([S]() { return S->bReferenceAtCg ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
							.OnCheckStateChanged_Lambda([S](ECheckBoxState State) { S->bReferenceAtCg = State == ECheckBoxState::Checked; })
							[ SNew(STextBlock).Text(LOCTEXT("RefAtCg", "At the mass centre (the node nearest it)")) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							SNew(SBox).IsEnabled_Lambda([S]() { return !S->bReferenceAtCg; })
							[
								FormRow(S->IsReplica() ? LOCTEXT("RefAtFull", "Or near (full scale):") : LOCTEXT("RefAt", "Or near:"),
									SNew(SHorizontalBox)
									+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0) [ MakeStateSpin(S, &FState::RefX, -1e12, 1e12, 0.1, LengthSuffix) ]
									+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0) [ MakeStateSpin(S, &FState::RefY, -1e12, 1e12, 0.1, LengthSuffix) ]
									+ SHorizontalBox::Slot().FillWidth(1) [ MakeStateSpin(S, &FState::RefZ, -1e12, 1e12, 0.1, LengthSuffix) ], 130.0f)
							]
						])
				]
			]
			+ SScrollBox::Slot().Padding(0, 0, 0, 8).HAlign(HAlign_Left)
			[ Button(LOCTEXT("ApplySupports", "Apply supports"), [S]() { S->ApplyInputs(); }) ]
			+ SScrollBox::Slot()
			[
				MakeInfoLabel(LOCTEXT("SupportHint",
					"CLAMPED: every node within the tolerance of any plane is fixed in every degree of freedom (the shells' rotations too: a clamp). y = 0 is the root of a mirrored half -- the symmetry plane -- which is why it is the one to start with; the mesh's minimum or maximum along an axis clamps a body by its end -- 'Of' says whose end. A plane limited to a PATCH fixes only the nodes inside its rectangle: type it, Fit to body, or drag the handles in the picture (the fixed nodes follow live). Values, tolerances and patches are typed in FULL-SCALE coordinates; with a model scale set they are scaled to the replica. A tolerance of 0 is a ten-thousandth of the mesh's diagonal. Whatever reaches no plane, on its own or through a body it was fused with, is left out of the solve and listed on the right. "
					"FREE FLIGHT (inertia relief): no plane; the loads are balanced by the inertial field of the rigid-body motion they would give the free body, and one node -- the nearest to the mass centre, or to a point you type -- is fixed in its six degrees of freedom as a support that cannot load the structure. Its reaction is the residual of the load set and is reported as the check. Apply pushes the supports onto the built model and redraws them."))
			];
	}

	// The Support picture's handles: a press on one drags its patch, the
	// fixed nodes following, the model re-derived on release.
	inline void WirePatchHandles(TSharedRef<FState> S)
	{
		if (!S->Views[TabSupport].IsValid()) return;
		TWeakPtr<FState> Weak = S;
		S->Views[TabSupport]->OnPickHover = [Weak](const FVector2D& Pixel, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->OnPatchHover(Pixel); };
		S->Views[TabSupport]->OnPickDown = [Weak](const FVector2D& Pixel, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->OnPatchDown(Pixel); };
		S->Views[TabSupport]->OnPickMove = [Weak](const FVector2D&, const FVector& O, const FVector& D) { TSharedPtr<FState> P = Weak.Pin(); if (P.IsValid()) P->OnPatchMove(O, D); return true; };
		S->Views[TabSupport]->OnPickUp = [Weak](const FVector2D&, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); if (P.IsValid()) P->OnPatchUp(); return true; };
	}

	// The option lists every state starts with.
	inline void FillOptions(TSharedRef<FState> S, const TArray<FAerotelicBodyChoicePtr>& Choices)
	{
		S->Choices = Choices;
		S->SolverOptions = {
			MakeShared<FString>(FString::Printf(TEXT("Automatic (direct up to %s DOF, iterative beyond)"), *Thousands(AerotelicStructuralFea::kAutoDirectDofs))),
			MakeShared<FString>(TEXT("sparse_lu (direct; exact; one thread)")),
			MakeShared<FString>(TEXT("amgcl (iterative; every core; check the force balance)")) };
		S->ShellOptions = {
			MakeShared<FString>(TEXT("Linear thin triangle (Kirchhoff; reactions balance exactly)")),
			MakeShared<FString>(TEXT("Corotational thin triangle (small displacements only)")) };
		S->AxisOptions = { MakeShared<FString>(TEXT("x")), MakeShared<FString>(TEXT("y")), MakeShared<FString>(TEXT("z")) };
		S->PlaneOptions = { MakeShared<FString>(TEXT("at the value")), MakeShared<FString>(TEXT("at the mesh's minimum")), MakeShared<FString>(TEXT("at the mesh's maximum")) };
		S->BodyOptions.Reset();
		S->BodyOptions.Add(MakeShared<FString>(TEXT("the whole mesh")));
		for (const FAerotelicBodyChoicePtr& C : Choices) S->BodyOptions.Add(MakeShared<FString>(C->Label));
		S->AttachOptions.Reset();
		S->AttachOptions.Add(MakeShared<FString>(TEXT("nothing (left out of the analysis)")));
		for (const FAerotelicBodyChoicePtr& C : Choices) S->AttachOptions.Add(MakeShared<FString>(C->Label));
		S->SupportModeOptions = {
			MakeShared<FString>(TEXT("Clamped at the support planes")),
			MakeShared<FString>(TEXT("Free flight (inertia relief): one reference node, no planes")) };
		S->LoadMassRows();
	}

	// The bodies a boolean result already contains start unchosen (choosing
	// both would add a second copy of each).
	inline TArray<FAerotelicBodyChoicePtr> DefaultChosen(const TArray<FAerotelicBodyChoicePtr>& Choices)
	{
		TArray<FAerotelicBooleanOperand> Operands;
		if (const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
		{
			for (const auto& Weak : Doc->GetEntities())
			{
				if (!Weak.IsValid() || Weak->GetEntityType() != EntityType::Boolean) continue;
				if (const FAerotelicBooleanModel* M = Weak->GetModelAs<FAerotelicBooleanModel>()) Operands.Append(M->Operands);
			}
		}
		TArray<FAerotelicBodyChoicePtr> Out;
		for (const FAerotelicBodyChoicePtr& C : Choices)
		{
			const bool bOperand = Operands.ContainsByPredicate([&C](const FAerotelicBooleanOperand& Op) { return SameOperand(Op, C->Operand); });
			if (!bOperand) Out.Add(C);
		}
		return Out.Num() ? Out : Choices;
	}
}

#undef LOCTEXT_NAMESPACE

