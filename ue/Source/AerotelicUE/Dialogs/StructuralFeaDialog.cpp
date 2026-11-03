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
#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
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

#include <memory>

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicLatticeModel.h"

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
	// operand but the lattices themselves (a lattice is the infill of a
	// body, analysed as that body's homogenised core). A wing's flap, a
	// propeller's blade, a boolean result's separate shell is each a row of
	// its own; an operand whose bodies are not known yet (never
	// tessellated) is one row for all of them.
	TArray<FAerotelicBodyChoicePtr> AnalysableChoices()
	{
		TArray<FAerotelicBodyChoicePtr> Out;
		for (const FAerotelicBodyChoicePtr& C : SceneBodyChoices())
		{
			if (!C->Operand.Entity.IsValid() || C->Operand.Entity->GetEntityType() == EntityType::Lattice) continue;
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

	// The lattice that fills a body, if one does.
	AAerotelicEntityActor* LatticeFilling(const FAerotelicBooleanOperand& Op)
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc == nullptr) return nullptr;
		for (const auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid() || Weak->GetEntityType() != EntityType::Lattice) continue;
			const FAerotelicLatticeModel* M = Weak->GetModelAs<FAerotelicLatticeModel>();
			if (M == nullptr) continue;
			for (const FAerotelicBooleanOperand& T : M->Targets)
			{
				if (SameOperand(T, Op)) return Weak.Get();
			}
		}
		return nullptr;
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
		double Value = 0.0;
		double Tolerance = 0.0;
	};

	// The model as built from the chosen bodies: the full mesh as meshed,
	// the mesh the solver gets (the full one less whatever floats free of
	// the supports), the request, and what the pre-processor derives. One
	// heap object, so the request's pointers into it stay good.
	struct FBuilt
	{
		aerotelic::TetMesh FullMesh;
		TArray<AerotelicStructuralFea::FShellFace> FullShellFaces;
		aerotelic::TetMesh Mesh;
		std::vector<std::unique_ptr<aerotelic::UnitCellSamples>> Cells;   // one per lattice homogenised
		AerotelicStructuralFea::FRequest Request;
		FModel Model;
		TArray<FString> Problems;   // left out while meshing
		TArray<FString> Dropped;    // left out for floating, this derivation
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
		TArray<TSharedPtr<FString>> ModelOptions, SolverOptions, ShellOptions, AxisOptions, PlaneOptions;
		int32 ModelIndex = 0, SolverIndex = 0, ShellIndex = 0;
		double Voxels = 32.0;
		double ElementSize = 0.0;   // solid bodies; 0: a twentieth of the largest extent
		// The loads and the supports: any number of point forces and of
		// planes, edited on their tabs and pushed by Apply.
		double LoadFactor = 0.0, Pressure = 0.0;
		TArray<TSharedPtr<FPointRow>> PointRows;
		TArray<TSharedPtr<FSupportRow>> SupportRows;
		TSharedPtr<SVerticalBox> PointList, SupportList;
		TSharedPtr<SEditableTextBox> PathBox;
		TSharedPtr<SCheckBox> SecondOrderCheck, CacheCheck;
		TSharedPtr<STextBlock> LblMaterials, LblMesh, LblCore, LblDisp, LblStress, LblMargins, LblBalance, LblTime;

		// The model, once built (Build Model, or Solve).
		TSharedPtr<FBuilt> Built;

		// The pre-processor pictures: one view per tab, its actors, its caption.
		TSharedPtr<SAerotelicSceneView> Views[kPictures];
		TSharedPtr<STextBlock> Captions[kPictures];
		TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>> Actors[kPictures];
		bool bFramed[kPictures] = { false, false, false, false };

		TSharedPtr<SAerotelicTabStrip> Tabs;
		bool bSmoke = false;

		bool bFullResolution() const { return ModelIndex == 1; }

		void Message(const FText& Text)
		{
			AerotelicMessageBox(EAppMsgType::Ok, Text, LOCTEXT("Title", "Structural (Kratos FEA)"));
		}

		TArray<FAerotelicBodyChoicePtr> ChosenBodies() const
		{
			TArray<FAerotelicBodyChoicePtr> Out;
			if (!Picker.IsValid()) return Out;
			for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen()) if (C->Operand.Entity.IsValid()) Out.Add(C);
			return Out;
		}

		// What a body is, from its (inherited) material properties and
		// whether a lattice fills it; the reason when it cannot be analysed.
		struct FKind
		{
			bool bOk = false;
			AAerotelicEntityActor* Lattice = nullptr;
			aerotelic::MassSpec Spec;
			FString Text;
		};
		FKind Classify(const FAerotelicBodyChoice& C) const
		{
			FKind K;
			const FAerotelicBooleanOperand& Op = C.Operand;
			if (!Op.Entity.IsValid()) { K.Text = TEXT("(removed entity)"); return K; }
			K.Spec = Op.Entity->GetEffectiveMassSpec();
			const AAerotelicEntityActor* Source = Op.Entity->GetMassSpecSource();
			const FString From = Source != nullptr && Source != Op.Entity.Get() ? FString::Printf(TEXT(" (inherited from %s)"), *Source->EntityName) : FString();
			if (!HasStructuralMaterial(K.Spec))
			{
				K.Text = FString::Printf(TEXT("%s: NO STRUCTURAL MATERIAL%s -- set one under Material Properties in the Scene tree"), *C.Label, *From);
				return K;
			}
			K.Lattice = LatticeFilling(Op);
			const FString Material = UTF8_TO_TCHAR(K.Spec.material.name.c_str());
			const FString Length = UnitName(Units.LengthName());
			if (K.Lattice != nullptr)
			{
				const FAerotelicLatticeModel* M = K.Lattice->GetModelAs<FAerotelicLatticeModel>();
				const double T = FAerotelicLatticeModel::SkinThicknessFor(Op);
				K.Text = bFullResolution()
					? FString::Printf(TEXT("%s: %s%s, filled by %s -- skin tetrahedra of %s over core tetrahedra with the homogenised %s %s lattice at %.0f%% (cell %.4g %s, %d voxels across)"),
						*C.Label, *Material, *From, *K.Lattice->EntityName, *Material,
						UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(M->Spec.family)), UTF8_TO_TCHAR(M->Spec.type.c_str()), 100.0 * M->Spec.volume_fraction, M->Spec.cell_size, *Length, static_cast<int32>(Voxels))
					: FString::Printf(TEXT("%s: %s%s, filled by %s -- shells %.4g %s thick over tetrahedra at the cell size with the homogenised %s %s lattice at %.0f%% (cell %.4g %s, %d voxels across), struts of %s"),
						*C.Label, *Material, *From, *K.Lattice->EntityName, T, *Length,
						UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(M->Spec.family)), UTF8_TO_TCHAR(M->Spec.type.c_str()), 100.0 * M->Spec.volume_fraction, M->Spec.cell_size, *Length, static_cast<int32>(Voxels), *Material);
			}
			else if (K.Spec.skin_thickness > 0.0)
			{
				K.Text = FString::Printf(TEXT("%s: %s%s -- a thin-walled body: shells %.4g %s thick on its surface, nothing inside"), *C.Label, *Material, *From, K.Spec.skin_thickness, *Length);
			}
			else
			{
				K.Text = FString::Printf(TEXT("%s: %s%s -- a solid: tetrahedra of %s at the element size (no skin thickness set)"), *C.Label, *Material, *From, *Material);
			}
			K.bOk = true;
			return K;
		}

		void DescribeModel()
		{
			if (!LblMaterials.IsValid()) return;
			TArray<FString> Lines;
			for (const FAerotelicBodyChoicePtr& C : ChosenBodies()) Lines.Add(Classify(*C).Text);
			LblMaterials->SetText(Lines.Num() ? FText::FromString(FString::Join(Lines, TEXT("\n"))) : LOCTEXT("NoBodies", "Choose at least one body."));
		}

		// The supports and loads as they stand on their tabs, into a request.
		void FillInputs(AerotelicStructuralFea::FRequest& Request) const
		{
			Request.Supports.Reset();
			for (const TSharedPtr<FSupportRow>& R : SupportRows)
			{
				AerotelicStructuralFea::FSupport S;
				S.Axis = (*AxisOptions[FMath::Clamp(R->Axis, 0, 2)])[0];
				S.Value = R->Value;
				S.bAtMin = R->Where == 1;
				S.bAtMax = R->Where == 2;
				S.Tolerance = R->Tolerance;
				Request.Supports.Add(S);
			}
			Request.PointForces.Reset();
			for (const TSharedPtr<FPointRow>& R : PointRows)
			{
				AerotelicStructuralFea::FPointForce P;
				P.At = aerotelic::Vec3{ R->X, R->Y, R->Z };
				P.Force = aerotelic::Vec3{ R->Fx, R->Fy, R->Fz };
				Request.PointForces.Add(P);
			}
			Request.LoadFactor = LoadFactor;
			Request.Pressure = Pressure * Units.StressToSI();
			Request.bSecondOrder = SecondOrderCheck.IsValid() && SecondOrderCheck->IsChecked();
			static const TCHAR* kSolvers[] = { TEXT("auto"), TEXT("sparse_lu"), TEXT("amgcl") };
			Request.LinearSolver = kSolvers[FMath::Clamp(SolverIndex, 0, 2)];
			static const TCHAR* kShellElements[] = { TEXT("ShellThinElement3D3N"), TEXT("ShellThinElementCorotational3D3N"), TEXT("ShellThickElementCorotational3D3N") };
			Request.ShellElement = kShellElements[FMath::Clamp(ShellIndex, 0, 2)];
			Request.bHomogenizeCache = CacheCheck.IsValid() && CacheCheck->IsChecked();
			Request.VtkPath = PathBox.IsValid() ? PathBox->GetText().ToString().TrimStartAndEnd() : FString();
		}

		// ---- the model: mesh the chosen bodies once; derive the rest ----

		bool BuildModel()
		{
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
			TMap<AAerotelicEntityActor*, int32> CellOf;
			int32 NextId = 1;

			FBox Bounds(ForceInit);
			for (const FAerotelicBodyChoicePtr& C : Bodies) Bounds += C->Operand.Entity->GetPlacedBounds(C->Operand.bMirror);
			const double Element = ElementSize > 0.0 ? ElementSize : (Bounds.IsValid ? FMath::Max(Bounds.GetSize().GetMax() / 20.0, 1e-6) : 1.0);

			auto AppendTets = [&Mesh](const aerotelic::TetMesh& From, TFunction<int32(int32)> RegionOf)
			{
				const int Offset = static_cast<int>(Mesh.nodes.size());
				Mesh.nodes.insert(Mesh.nodes.end(), From.nodes.begin(), From.nodes.end());
				for (size_t t = 0; t < From.tets.size(); ++t)
				{
					const auto& T = From.tets[t];
					Mesh.tets.push_back({T[0] + Offset, T[1] + Offset, T[2] + Offset, T[3] + Offset});
					Mesh.material.push_back(RegionOf(t < From.material.size() ? From.material[t] : 1));
				}
			};

			for (const FAerotelicBodyChoicePtr& C : Bodies)
			{
				const FAerotelicBooleanOperand& Op = C->Operand;
				const FKind K = Classify(*C);
				const FString EntityLabel = OperandLabel(Op);
				if (!K.bOk) { B->Problems.Add(K.Text); continue; }
				const aerotelic::Material& Material = K.Spec.material;
				// The body's label as the lattice names its meshes and the
				// regions are reported: "<entity>[ (mirror)][ / <body>]".
				const std::vector<aerotelic::NamedMesh> Placed = Op.Entity->GetPlacedMeshes(Op.bMirror, FAerotelicBooleanModel::kValidationTolerance);
				if (Placed.empty()) { B->Problems.Add(C->Label + TEXT(": no geometry could be built")); continue; }
				for (const aerotelic::NamedMesh& NM : Placed)
				{
					if (NM.mesh.IsEmpty()) continue;
					if (!C->Body.IsEmpty() && Placed.size() > 1 && FString(UTF8_TO_TCHAR(NM.name.c_str())) != C->Body) continue;
					const FString BodyLabel = Placed.size() > 1 ? FString::Printf(TEXT("%s / %s"), *EntityLabel, UTF8_TO_TCHAR(NM.name.c_str())) : EntityLabel;
					if (K.Lattice != nullptr)
					{
						const FAerotelicLatticeModel* M = K.Lattice->GetModelAs<FAerotelicLatticeModel>();
						int32* CellIndex = CellOf.Find(K.Lattice);
						if (CellIndex == nullptr)
						{
							try
							{
								B->Cells.push_back(std::make_unique<aerotelic::UnitCellSamples>(aerotelic::SampleUnitCell(M->Spec, static_cast<int32>(Voxels))));
							}
							catch (const std::exception& E)
							{
								B->Problems.Add(FString::Printf(TEXT("%s: the lattice cell of %s could not be voxelised: %s"), *BodyLabel, *K.Lattice->EntityName, UTF8_TO_TCHAR(E.what())));
								continue;
							}
							CellIndex = &CellOf.Add(K.Lattice, static_cast<int32>(B->Cells.size()) - 1);
						}
						const aerotelic::UnitCellSamples* Cell = B->Cells[*CellIndex].get();
						const std::vector<FAerotelicLatticeModel::FTetMesh>& Meshes = bFullResolution() ? M->GetTetMeshes() : M->GetStructuralTetMeshes();
						const FAerotelicLatticeModel::FTetMesh* TM = nullptr;
						for (const FAerotelicLatticeModel::FTetMesh& X : Meshes) if (X.Label == BodyLabel) { TM = &X; break; }
						if (TM == nullptr)
						{
							B->Problems.Add(FString::Printf(TEXT("%s: %s has no tetrahedral mesh of it (%s)"), *BodyLabel, *K.Lattice->EntityName,
								*FString::Join(bFullResolution() ? M->GetTetNotes() : M->GetStructuralTetNotes(), TEXT("; "))));
							continue;
						}
						if (bFullResolution())
						{
							const int32 SkinId = NextId++, CoreId = NextId++;
							AppendTets(TM->Result.Mesh, [SkinId, CoreId](int32 Region) { return Region == 2 ? CoreId : SkinId; });
							AerotelicStructuralFea::FRegion Skin;
							Skin.Id = SkinId; Skin.Label = BodyLabel + TEXT(" skin"); Skin.Core = Material; Skin.bSolid = true;
							Request.Regions.Add(Skin);
							AerotelicStructuralFea::FRegion Core;
							Core.Id = CoreId; Core.Label = BodyLabel + TEXT(" core (") + K.Lattice->EntityName + TEXT(")"); Core.Core = Material; Core.Cell = Cell;
							Request.Regions.Add(Core);
						}
						else
						{
							const int32 Id = NextId++;
							AppendTets(TM->Result.Mesh, [Id](int32) { return Id; });
							AerotelicStructuralFea::FRegion R;
							R.Id = Id; R.Label = BodyLabel;
							R.Skin = Material; R.SkinThickness = TM->SkinThickness > 0.0 ? TM->SkinThickness : FAerotelicLatticeModel::SkinThicknessFor(Op);
							R.Core = Material; R.Cell = Cell;
							Request.Regions.Add(R);
						}
						continue;
					}
					if (!NM.mesh.IsClosedManifold() || !(NM.mesh.SignedVolume() > 0.0))
					{
						B->Problems.Add(BodyLabel + TEXT(": not a closed, outward surface"));
						continue;
					}
					const int32 Id = NextId++;
					AerotelicStructuralFea::FRegion R;
					R.Id = Id; R.Label = BodyLabel;
					if (K.Spec.skin_thickness > 0.0)
					{
						const int32 Offset = static_cast<int32>(Mesh.nodes.size());
						Mesh.nodes.insert(Mesh.nodes.end(), NM.mesh.vertices.begin(), NM.mesh.vertices.end());
						for (const auto& Tri : NM.mesh.triangles)
						{
							AerotelicStructuralFea::FShellFace F;
							F.N[0] = Tri[0] + Offset; F.N[1] = Tri[1] + Offset; F.N[2] = Tri[2] + Offset;
							F.Region = Id;
							B->FullShellFaces.Add(F);
						}
						R.Skin = Material; R.SkinThickness = K.Spec.skin_thickness;
					}
					else
					{
						AerotelicTetGmsh::FRequest G;
						G.Outer = &NM.mesh;
						G.Inner = nullptr;
						G.CoreSize = Element;
						G.Algorithm = 1;
						G.bOptimize = true;
						G.bOptimizeNetgen = true;
						AerotelicTetGmsh::FResult TR;
						FString Error;
						if (!AerotelicTetGmsh::Run(G, TR, Error))
						{
							B->Problems.Add(BodyLabel + TEXT(": could not be meshed with tetrahedra: ") + Error);
							continue;
						}
						AppendTets(TR.Mesh, [Id](int32) { return Id; });
						R.Core = Material; R.bSolid = true;
					}
					Request.Regions.Add(R);
				}
			}
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
			Status(FText::FromString(FString::Printf(TEXT("Model built: %s nodes, %s tetrahedra, %s wetted faces, %s nodes on the supports -- see the Mesh, Loads and Support tabs"),
				*Thousands(M.Nodes), *Thousands(M.Tets), *Thousands(M.WettedFaces.Num()), *Thousands(M.SupportNodes.Num()))));
			if (B->Problems.Num() > 0 || B->Dropped.Num() > 0)
			{
				TArray<FString> All = B->Problems;
				All.Append(B->Dropped);
				Message(FText::FromString(TEXT("Left out of the model:\n") + FString::Join(All, TEXT("\n"))));
			}
			return true;
		}

		// The supports and loads applied to the built mesh: the parts with
		// no load path to any support are left out (a control surface in
		// its gap, a payload bay inside the fuselage -- they would make the
		// system singular), the rest derived for the pictures and the solve.
		bool Derive()
		{
			if (!Built.IsValid()) return false;
			FBuilt& B = *Built;
			FillInputs(B.Request);
			B.Mesh = B.FullMesh;
			B.Request.Mesh = &B.Mesh;
			B.Request.ShellFaces = B.FullShellFaces;
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
				TArray<AerotelicStructuralFea::FShellFace> TrimmedFaces;
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
				for (const auto& Entry : DroppedPerRegion)
				{
					const AerotelicStructuralFea::FRegion* R = B.Request.Regions.FindByPredicate([&Entry](const AerotelicStructuralFea::FRegion& X) { return X.Id == Entry.Key; });
					const bool bWhole = M.FloatingRegions.Contains(Entry.Key);
					B.Dropped.Add(FString::Printf(TEXT("%s: %s element%s with no load path to any support left out%s"), R ? *R->Label : *FString::FromInt(Entry.Key),
						*Thousands(Entry.Value), Entry.Value == 1 ? TEXT("") : TEXT("s"), bWhole ? TEXT(" (the whole body: it touches neither a support plane nor a supported body)") : TEXT("")));
				}
				B.Mesh = std::move(Trimmed);
				B.Request.ShellFaces = TrimmedFaces;
				// The regions that lost everything go too, or the solver would
				// want tetrahedra for them.
				B.Request.Regions.RemoveAll([&M](const AerotelicStructuralFea::FRegion& X) { return M.FloatingRegions.Contains(X.Id); });
				if (B.Mesh.tets.empty() && B.Request.ShellFaces.Num() == 0)
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
			UpdatePictures();
			return true;
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
				Status(FText::FromString(FString::Printf(TEXT("Applied: %d support plane(s) fixing %s nodes, %d point force(s)%s"),
					M.SupportPlanes.Num(), *Thousands(M.SupportNodes.Num()), M.PointLoads.Num(),
					Built->Dropped.Num() ? *FString::Printf(TEXT("; %d part(s) left out for floating, see the Support tab"), Built->Dropped.Num()) : TEXT(""))));
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

			// Mesh: the wetted faces by region, with their edges.
			{
				FGeom G = Surface([&](const FFace& F) { const int32* I = RegionIndex.Find(F.Region); return RegionColor(I ? *I : 0); });
				ShowPicture(TabMesh, { &G }, { true });
				FString Text = FString::Printf(TEXT("%s nodes, %s %s tetrahedra, %s wetted faces (their edges drawn)%s"),
					*Thousands(M.Nodes), *Thousands(M.Tets), B.Request.bSecondOrder && M.Tets > 0 ? TEXT("10-node") : TEXT("4-node"), *Thousands(M.WettedFaces.Num()),
					B.Request.bSecondOrder && M.Tets > 0 ? TEXT("; the solver adds a mid-edge node on every tetrahedron edge, not drawn") : TEXT(""));
				for (int32 i = 0; i < M.Regions.Num(); ++i)
				{
					const AerotelicFeaPreprocess::FRegionSummary& R = M.Regions[i];
					Text += FString::Printf(TEXT("\nregion %d, %s (%s): %s tetrahedra, %s faces, volume %.4g %s^3, area %.4g %s^2"),
						R.Id, *R.Label, *R.Kind, *Thousands(R.Tets), *Thousands(R.Faces), R.Volume, *LengthUnit, R.Area, *LengthUnit);
				}
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
						Text += FString::Printf(TEXT("\nPoint force %d (red): (%.4g, %.4g, %.4g) N at node %d, (%.4g, %.4g, %.4g) %s -- the corner node nearest the point asked for"),
							i + 1, P.Force.X, P.Force.Y, P.Force.Z, P.Node, P.At.X, P.At.Y, P.At.Z, *LengthUnit);
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
					Text += FString::Printf(TEXT("\nInertial load (green): %.4g g on the model's mass of %.5g kg%s: (%.4g, %.4g, %.4g) N, drawn at the centre of mass (%.4g, %.4g, %.4g) %s; applied as a body force on every element"),
						B.Request.LoadFactor, M.Mass, M.bMassEstimated ? TEXT(" (a homogenised core's density estimated from the cell's solid fraction, before homogenisation)") : TEXT(""),
						M.GravityForce.X, M.GravityForce.Y, M.GravityForce.Z, M.MassCentre.X, M.MassCentre.Y, M.MassCentre.Z, *LengthUnit);
				}
				else
				{
					Text += TEXT("\nInertial load: none (load factor 0)");
				}
				Text += TEXT("\nArrow heads sit at the point of application; forces are newtons whatever the project's units. Edit the loads on the left and Apply.");
				ShowPicture(TabLoads, { &G, &Arrows }, { false, false });
				if (Captions[TabLoads].IsValid()) Captions[TabLoads]->SetText(FText::FromString(Text));
			}

			// Support: the surface with its edges, a marker on every fixed
			// node, and each plane as a frame round the mesh.
			{
				FGeom G = Surface([&](const FFace&) { return kGrey; });
				FGeom Marks;
				const int32 NSupport = M.SupportNodes.Num();
				const int32 Stride = FMath::Max(1, FMath::DivideAndRoundUp(NSupport, 20000));
				const double R = 0.005 * Diag;
				for (int32 i = 0; i < NSupport; i += Stride) AddOctahedron(Marks, At(M.SupportNodes[i]), R, kFixed);
				const TCHAR AxisName[3] = { TEXT('x'), TEXT('y'), TEXT('z') };
				FString Text;
				for (int32 k = 0; k < M.SupportPlanes.Num(); ++k)
				{
					const FModel::FPlane& Plane = M.SupportPlanes[k];
					const int32 A = Plane.Axis, U = (A + 1) % 3, W = (A + 2) % 3;
					const double Margin = 0.05 * Diag;
					FVector Lo(M.Bounds.min.x, M.Bounds.min.y, M.Bounds.min.z), Hi(M.Bounds.max.x, M.Bounds.max.y, M.Bounds.max.z);
					Lo[U] -= Margin; Lo[W] -= Margin; Hi[U] += Margin; Hi[W] += Margin;
					FVector P[4];
					for (int32 c = 0; c < 4; ++c)
					{
						P[c][A] = Plane.Value;
						P[c][U] = (c == 1 || c == 2) ? Hi[U] : Lo[U];
						P[c][W] = (c >= 2) ? Hi[W] : Lo[W];
					}
					for (int32 c = 0; c < 4; ++c) AddBar(Marks, P[c], P[(c + 1) % 4], 0.002 * Diag, kPlane);
					Text += FString::Printf(TEXT("%sSupport %d (blue frame): the plane %c = %.6g %s, %s nodes within %.3g %s of it%s"),
						k ? TEXT("\n") : TEXT(""), k + 1, AxisName[A], Plane.Value, *LengthUnit, *Thousands(Plane.Nodes), Plane.Tolerance, *LengthUnit,
						B.Request.Supports.IsValidIndex(k) && B.Request.Supports[k].Tolerance > 0.0 ? TEXT("") : TEXT(" (a ten-thousandth of the mesh's diagonal)"));
				}
				if (M.SupportPlanes.Num() == 0) Text += TEXT("No support: add a plane on the left and Apply.");
				Text += FString::Printf(TEXT("\nFixed nodes (red): %s of %s, every degree of freedom -- the three displacements and, on the shells, the three rotations: a clamp"),
					*Thousands(NSupport), *Thousands(M.Nodes));
				if (Stride > 1) Text += FString::Printf(TEXT(" (one marker in %d drawn)"), Stride);
				if (NSupport < 3) Text += TEXT("\nFEWER THAN THREE NODES: the solver refuses this support. Move a plane onto the geometry (or use the mesh's minimum / maximum) or widen the tolerance.");
				if (B.Request.bSecondOrder && M.Tets > 0) Text += TEXT("\nThe mid-edge nodes the solver adds between two fixed nodes are fixed with them.");
				if (B.Dropped.Num() > 0)
				{
					Text += TEXT("\nLEFT OUT, no load path to any support (they would make the system singular):");
					for (const FString& D : B.Dropped) Text += TEXT("\n  ") + D;
				}
				else
				{
					Text += FString::Printf(TEXT("\nLoad path: every element reaches a support (%d connected piece%s)."), M.Components, M.Components == 1 ? TEXT("") : TEXT("s"));
				}
				Text += TEXT("\nEdit the supports on the left and Apply.");
				ShowPicture(TabSupport, { &G, &Marks }, { true, false });
				if (Captions[TabSupport].IsValid()) Captions[TabSupport]->SetText(FText::FromString(Text));
			}
		}

		// ---- the solve ----

		void Solve()
		{
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
				Message(FText::FromString(FString::Printf(TEXT("Only %d node(s) lie on the support plane(s); a support needs a face of them. See the Support tab: move a plane onto the geometry (or use the mesh's minimum / maximum) or widen the tolerance."), M.SupportNodes.Num())));
				if (Tabs.IsValid()) Tabs->SetActiveTab(TabSupport);
				return;
			}
			Status(FText::FromString(FString::Printf(TEXT("Solving %s tetrahedra and %s wetted faces on Kratos..."),
				*Thousands((int64)B.Mesh.tets.size()), *Thousands(M.WettedFaces.Num()))));
			AerotelicStructuralFea::FResult R;
			FString Error;
			if (!AerotelicStructuralFea::Run(B.Request, R, Error))
			{
				Status(LOCTEXT("Ready", "Ready"));
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("Failed", "Structural Analysis Failed"));
				return;
			}

			const double L2 = Units.LengthToSI(), S2 = Units.StressToSI(), M2 = Units.MassToSI();
			LblMesh->SetText(FText::FromString(FString::Printf(TEXT("%s nodes; %s %s tetrahedra + %s shells; %s DOF; %s wetted faces; %s nodes on the supports"),
				*Thousands(R.Nodes), *Thousands(R.Tets), R.bSecondOrder ? TEXT("10-node") : TEXT("4-node"), *Thousands(R.Shells), *Thousands(R.Dofs), *Thousands(R.WettedFaces), *Thousands(R.SupportNodes))));
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
			LblTime->SetText(FText::FromString(FString::Printf(TEXT("%.1f s (%.1f homogenising, %.1f solving with %s)"), R.Seconds, R.SecondsHomogenize, R.SecondsSolve, *R.LinearSolver)));
			for (const FString& W : R.Warnings)
			{
				UE_LOG(LogTemp, Warning, TEXT("StructuralFea: %s"), *W);
			}
			if (R.SupportNodes != M.SupportNodes.Num() || R.WettedFaces != M.WettedFaces.Num())
			{
				UE_LOG(LogTemp, Warning, TEXT("StructuralFea: the pre-processor counted %d support nodes and %d wetted faces, the solver %d and %d"),
					M.SupportNodes.Num(), M.WettedFaces.Num(), R.SupportNodes, R.WettedFaces);
			}
			Status(FText::FromString(FString::Printf(TEXT("Structural analysis solved on %s DOF (%s): max displacement %.4g %s, max von Mises %.4g %s -> %s%s"),
				*Thousands(R.Dofs), *R.LinearSolver, R.MaxDisplacement / L2, *UnitName(Units.LengthName()), R.MaxVonMises / S2, *UnitName(Units.StressName()), *R.VtkPath,
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
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("At", "At:"), Triple(false), 70.0f) ]
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
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Value", "Value:"), NumberField([Row]() { return Row->Value; }, [Row](double V) { Row->Value = V; }, 0.1, LengthSuffix), 70.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Within", "Within:"), NumberField([Row]() { return Row->Tolerance; }, [Row](double V) { Row->Tolerance = FMath::Max(V, 0.0); }, 0.001, LengthSuffix, 0.0), 70.0f) ]
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
			LblMaterials.Reset(); LblMesh.Reset(); LblCore.Reset(); LblDisp.Reset(); LblStress.Reset(); LblMargins.Reset(); LblBalance.Reset(); LblTime.Reset();
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
		"A lattice-filled body is shells of its skin thickness over tetrahedra at the lattice cell size carrying the lattice's homogenised stiffness, the struts of the body's material; or, at full resolution, the skin tetrahedra as a solid over core tetrahedra with the homogenised lattice. The bodies are not joined to one another: each must reach a support on its own or through a body it was fused with. A body or part of one with no load path to any support -- a control surface in its gap, a payload bay inside the fuselage -- would make the system singular, so it is LEFT OUT automatically and named on the Mesh and Support tabs.\n\n"
		"SET-UP\n"
		"Build Model meshes the bodies (gmsh; a lattice's meshes are kept until it changes) and derives what the solver will be handed for the three tabs that follow, without solving. Solve does the same and then runs Kratos, writing the displacement and stress fields to the VTK file named and the peaks, margins and force balance into the Result group.\n\n"
		"MESH\n"
		"The wetted faces of the model -- the boundary faces of the tetrahedra and the shell-only bodies' triangles -- coloured by region with their edges drawn. Where a region has a skin these faces ARE the shell elements. The caption counts nodes, tetrahedra and faces, each region's kind, volume and area, and what was left out.\n\n"
		"LOADS\n"
		"On the left: the inertial load factor (the model's own mass under that many g along -z; 0 to begin with, so a test load acts alone), a uniform pressure on every wetted face (positive pressing inward), and any number of point forces, each at a point (the corner node nearest it takes the load) with its force in newtons -- add, edit, delete, then Apply. On the right the forces as arrows, each head at its point of application: orange the pressure as the traction on up to ~1500 faces (direction and relative size, not the face force), red each point force at its node, green the inertial load at the centre of mass.\n\n"
		"SUPPORT\n"
		"On the left: any number of support planes, each axis = value (or the mesh's minimum or maximum along the axis) with a tolerance (0 is a ten-thousandth of the mesh's diagonal, about how planar a lofted section is); every node within the tolerance of any plane is clamped in every degree of freedom (the shells' rotations too) -- add, edit, delete, then Apply. On the right a red marker on every fixed node, each plane as a blue frame, and the load path: whatever reaches no support is left out and listed.\n\n"
		"RESULT\n"
		"Linear static, small displacement, linear elastic, the linear thin shell triangle (its reactions balance the loads exactly; the corotational triangles are options for small-displacement comparisons). The reactions at the supports must balance what was applied: the Force balance line says by how much they miss, and a miss of more than one percent means the linear solver did not converge -- the solution is then reported as NOT converged and nothing in it is to be trusted (the iterative solver can do this on thin shells; the direct solver cannot, which is why the automatic choice takes it up to a million degrees of freedom). Margin is allowable / peak von Mises - 1 for each region's skin, solid or core; a homogenised core's margin is against the material's allowable using the stress estimated in the struts. The VTK holds the displacement and the stresses for ParaView, in SI. No post-processing view yet.");
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
		MakeShared<FString>(TEXT("Shells over a homogenised lattice core (for a desktop)")),
		MakeShared<FString>(TEXT("Skin and core tetrahedra (full resolution)")) };
	S->SolverOptions = {
		MakeShared<FString>(TEXT("Automatic (direct up to a million DOF, iterative beyond)")),
		MakeShared<FString>(TEXT("sparse_lu (direct; exact)")),
		MakeShared<FString>(TEXT("amgcl (iterative; check the force balance)")) };
	S->ShellOptions = {
		MakeShared<FString>(TEXT("Linear thin triangle (Kirchhoff; reactions balance exactly)")),
		MakeShared<FString>(TEXT("Corotational thin triangle (small displacements only)")),
		MakeShared<FString>(TEXT("Corotational thick triangle (small displacements only)")) };
	S->AxisOptions = { MakeShared<FString>(TEXT("x")), MakeShared<FString>(TEXT("y")), MakeShared<FString>(TEXT("z")) };
	S->PlaneOptions = { MakeShared<FString>(TEXT("at the value")), MakeShared<FString>(TEXT("at the mesh's minimum")), MakeShared<FString>(TEXT("at the mesh's maximum")) };
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
						"One row per SOLID BODY: a wing's main element and its flap, a propeller's blades, a boolean result's separate shells. What each is comes from its Material Properties (a boolean result inherits its first operand's): with a skin thickness and no lattice a thin-walled part of shells; with no skin thickness a solid of its material; filled by a lattice, shells over the lattice's homogenised core. "
						"The bodies are not joined to one another: each must reach a support on its own, or through a body it was fused with. Whatever has no load path to any support -- a control surface in its gap, a bay inside the fuselage -- is left out automatically and named. A boolean result already contains its operands, so those start unchosen."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("ModelGroup", "Model"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Model", "Lattice-filled bodies as:"), Combo(&FState::ModelOptions, &FState::ModelIndex, true)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Voxels", "Voxels across the cell:"), Spin(&FState::Voxels, 8.0, 96.0, 4.0, FText::GetEmpty(), true)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("ElementSize", "Solid bodies, element size (0 = auto):"), Spin(&FState::ElementSize, 0.0, 1e9, 0.01, LengthSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					SAssignNew(S->SecondOrderCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
					[ SNew(STextBlock).Text(LOCTEXT("SecondOrder", "Second-order (10-node) tetrahedra (the shells stay 3-node)")) ]
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
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ModelHint",
						"Shells over a homogenised core: a lattice-filled body's own surface triangles become shell elements of its skin thickness, and the inside is tetrahedra at the lattice cell size carrying the lattice's homogenised stiffness -- one unit cell, voxelised at the resolution above, solved under the six unit strains -- the struts of the body's material. The full-resolution model is the skin and core tetrahedra Lattice export writes, the skin a solid of the material and the core homogenised; keep second order on for it. Solid bodies without a lattice are meshed by gmsh at the element size. The direct solver is exact; the iterative one can stop short on thin shells and hand back a solution whose reactions do not balance the loads, so the automatic choice takes the direct solver up to a million degrees of freedom and the force balance is checked after every solve. "
						"The linear thin triangle is the shell for a linear static analysis: its reactions balance the loads exactly whatever the displacement. The corotational triangles are geometrically nonlinear elements solved in one step: they agree with it for small displacements and report inconsistent reactions and stresses once the model moves far."))
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
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("TimeStat", "Took:"), Stat(S->LblTime)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ResultHint",
						"Linear static, small displacement, linear elastic. The reactions at the supports must balance what was applied -- gravity on the mass, the pressure's resultant (zero over a closed surface), the point forces. Margin is allowable / peak von Mises - 1 for each region's skin, solid or core. A homogenised core's von Mises is a smeared average over a cell, so its margin is taken against the material's allowable using the stress ESTIMATED IN THE STRUTS. The VTK holds the displacement and the stresses for ParaView, in SI."))
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
				"The load factor puts the model's own mass under that many g along -z (0 to begin with, so a test load acts alone; 1 is its weight, a 4 g pull-up a real case). The pressure acts on every wetted face, positive pressing inwards -- the place a pressure field from the potential flow or DUST will plug in. Each point force goes to the corner node nearest its point. Forces are newtons whatever the project's units. Apply pushes the loads onto the built model and redraws them."))
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
				"Every node within the tolerance of any plane is fixed in every degree of freedom (the shells' rotations too: a clamp). y = 0 is the root of a mirrored half -- the symmetry plane -- which is why it is the one to start with; the mesh's minimum or maximum along an axis clamps a body by its end. A tolerance of 0 is a ten-thousandth of the mesh's diagonal -- about how planar a lofted section comes out, so a root ring is taken whole. Whatever reaches no plane, on its own or through a body it was fused with, is left out of the solve and listed on the right. Apply pushes the supports onto the built model and redraws them."))
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SetupTab", "Set-up"), SetupTab });
	Tabs.Add({ LOCTEXT("MeshTab", "Mesh"), MakePictureTab(S, TabMesh, LOCTEXT("MeshEmpty", "Build the model (Set-up tab) to see the mesh the solver is handed: its wetted faces by region, with their edges."), nullptr) });
	Tabs.Add({ LOCTEXT("LoadsTab", "Loads"), MakePictureTab(S, TabLoads, LOCTEXT("LoadsEmpty", "Build the model (Set-up tab) to see the loads as vectors: the pressure on the wetted faces, each point force at its node, the inertial load at the centre of mass."), LoadsPanel) });
	Tabs.Add({ LOCTEXT("SupportTab", "Support"), MakePictureTab(S, TabSupport, LOCTEXT("SupportEmpty", "Build the model (Set-up tab) to see the boundary conditions: a marker on every node the supports fix, each plane, and what reaches no support."), SupportPanel) });
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
				[ Button(LOCTEXT("BuildBtn", "Build Model"), [S]() { if (S->BuildModel() && S->Tabs.IsValid()) S->Tabs->SetActiveTab(TabMesh); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("SolveBtn", "Solve"), [S]() { S->Solve(); }) ]
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
			if (const aerotelic::Material* Preset = aerotelic::FindMaterialPreset("Aluminium 2024-T3"))
			{
				aerotelic::MassSpec Spec;
				Spec.mode = aerotelic::MassMode::Material;
				Spec.material = *Preset;
				const FBox Bounds = E->GetPlacedBounds(false);
				Spec.skin_thickness = Bounds.IsValid ? FMath::Max(0.01 * Bounds.GetSize().GetMax(), 1e-4) : 0.01;
				E->SetMassSpec(Spec);
			}
		}
		S->DescribeModel();
		S->Pressure = 1000.0 / S->Units.StressToSI();
		S->LoadFactor = 1.0;
		TSharedPtr<FPointRow> Tip = MakeShared<FPointRow>();
		FBox All(ForceInit);
		for (const FAerotelicBodyChoicePtr& C : InitiallyChosen) All += C->Operand.Entity->GetPlacedBounds(C->Operand.bMirror);
		if (All.IsValid) { Tip->X = All.GetCenter().X; Tip->Y = All.Max.Y; Tip->Z = All.GetCenter().Z; }
		S->PointRows.Add(Tip);
		S->RebuildPointList();
		if (S->BuildModel()) S->RunSmoke();
		else FPlatformMisc::RequestExit(false);
	}
}

#undef LOCTEXT_NAMESPACE
