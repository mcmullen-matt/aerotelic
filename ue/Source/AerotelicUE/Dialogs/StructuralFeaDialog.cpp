#include "StructuralFeaDialog.h"

#include "FeaCaseState.h"

#define LOCTEXT_NAMESPACE "StructuralFeaDialog"

// Analysis > Structural (Kratos FEA), since 2026-11-15 the SOLVER's dialog:
// pick a load case defined under Model > Load Cases, Build Model to see the
// mesh, the loads, the masses and the constraints on the real model, name
// the output file, Solve. Nothing is edited here; the engine
// (FeaCaseState.h) is the one Model > Load Cases edits with.

namespace
{
	using namespace AerotelicFeaCase;

	// One entry of the case combo: which list and which index.
	struct FCaseRef
	{
		bool bAero = false;
		int32 Index = -1;
		FString Label;
	};

	struct FDialog : public TSharedFromThis<FDialog>
	{
		TSharedPtr<FState> S;
		TWeakPtr<SWindow> Window;
		TArray<FCaseRef> Refs;
		TArray<TSharedPtr<FString>> CaseNames;
		int32 CaseIndex = -1;
		TSharedPtr<SComboBox<TSharedPtr<FString>>> CaseCombo;
		TSharedPtr<STextBlock> LblCase;
		FDelegateHandle CasesHandle, FieldsHandle;
		TSharedPtr<AerotelicAeroTrim::FRunner> Trim;
		bool bSolveAfterTrim = false;

		const FCaseRef* Current() const { return Refs.IsValidIndex(CaseIndex) ? &Refs[CaseIndex] : nullptr; }

		// The document's cases into the combo; the chosen one kept by label.
		void RefreshCases()
		{
			const FString Before = Refs.IsValidIndex(CaseIndex) ? Refs[CaseIndex].Label : FString();
			Refs.Reset();
			CaseNames.Reset();
			if (const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
			{
				const aerotelic::LoadCasesSpec& Spec = Doc->GetLoadCases();
				for (int32 i = 0; i < static_cast<int32>(Spec.aero.size()); ++i)
				{
					FCaseRef R; R.bAero = true; R.Index = i; R.Label = FString(TEXT("Aero: ")) + UTF8_TO_TCHAR(Spec.aero[i].name.c_str());
					Refs.Add(R);
				}
				for (int32 i = 0; i < static_cast<int32>(Spec.structural.size()); ++i)
				{
					FCaseRef R; R.bAero = false; R.Index = i; R.Label = FString(TEXT("Structural: ")) + UTF8_TO_TCHAR(Spec.structural[i].name.c_str());
					Refs.Add(R);
				}
			}
			for (const FCaseRef& R : Refs) CaseNames.Add(MakeShared<FString>(R.Label));
			int32 Index = Refs.IndexOfByPredicate([&Before](const FCaseRef& R) { return R.Label == Before; });
			if (Index < 0) Index = Refs.Num() ? 0 : -1;
			if (CaseCombo.IsValid())
			{
				CaseCombo->RefreshOptions();
				if (CaseNames.IsValidIndex(Index)) CaseCombo->SetSelectedItem(CaseNames[Index]);
			}
			SelectCase(Index, true);
		}

		// The case into the engine: its bodies, loads and constraints, and
		// for an aero case its solved field (null until solved).
		void SelectCase(int32 Index, bool bForce = false)
		{
			if (Index == CaseIndex && !bForce) return;
			CaseIndex = Index;
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			const FCaseRef* R = Current();
			if (R == nullptr || Doc == nullptr)
			{
				S->ChosenFixed.Reset();
				S->CaseName.Reset();
				S->AeroField.Reset();
				if (LblCase.IsValid()) LblCase->SetText(LOCTEXT("NoCases", "No load case is defined. Define one under Model > Load Cases: an aero case on the V-n diagram, or a structural case with its bodies, loads and constraints."));
				return;
			}
			const aerotelic::LoadCasesSpec& Spec = Doc->GetLoadCases();
			TArray<FString> Missing;
			FString Text;
			if (R->bAero && R->Index < static_cast<int32>(Spec.aero.size()))
			{
				const aerotelic::AeroLoadCaseSpec& A = Spec.aero[R->Index];
				TSharedPtr<FAerotelicAeroLoadCase> Field = Doc->FindAeroLoadCase(UTF8_TO_TCHAR(A.name.c_str()));
				Missing = S->LoadAero(A, Field);
				Text = FString::Printf(TEXT("AERO CASE '%s': V = %g m/s, air density %g kg/m³, load factor n = %g%s; %d bod%s. Flown free (inertia relief, gravity at 1 g) under the panel method's pressures trimmed to n W through the mass centre, so the structure feels n at its mass centre. %s"),
					UTF8_TO_TCHAR(A.name.c_str()), A.speed, A.density, A.load_factor,
					A.control_entity.empty() ? TEXT(" (the angle of attack alone trims the lift)") : *FString::Printf(TEXT(", trimmed with %s / %s"), UTF8_TO_TCHAR(A.control_entity.c_str()), UTF8_TO_TCHAR(A.control_flap.c_str())),
					static_cast<int32>(A.bodies.size()), A.bodies.size() == 1 ? TEXT("y") : TEXT("ies"),
					Field.IsValid() ? *FString::Printf(TEXT("Pressures SOLVED: α = %.3f°%s, q = %.4g Pa, %d body panels."), Field->AlphaDeg, Field->Control.IsEmpty() ? TEXT("") : *FString::Printf(TEXT(", δ = %.3f°"), Field->DeltaDeg), Field->DynamicPressure, Field->BodyPanels())
					                : TEXT("Pressures NOT SOLVED yet: Build Model runs the trim first (the panel method, a few seconds to a minute), or solve it under Model > Load Cases."));
			}
			else if (!R->bAero && R->Index < static_cast<int32>(Spec.structural.size()))
			{
				const aerotelic::StaticLoadCaseSpec& C = Spec.structural[R->Index];
				Missing = S->LoadStatic(C);
				Text = FString::Printf(TEXT("STRUCTURAL CASE '%s': load factor %g g, uniform pressure %g %s, %d point force(s); %s; %d bod%s."),
					UTF8_TO_TCHAR(C.name.c_str()), C.load_factor, C.pressure, *UnitName(S->Units.StressName()), static_cast<int32>(C.point_forces.size()),
					C.free_flight ? TEXT("free flight (inertia relief)") : *FString::Printf(TEXT("clamped at %d support plane(s)"), static_cast<int32>(C.supports.size())),
					static_cast<int32>(C.bodies.size()), C.bodies.size() == 1 ? TEXT("y") : TEXT("ies"));
			}
			if (Missing.Num() > 0) Text += TEXT("\nNOT IN THE SCENE (left out): ") + FString::Join(Missing, TEXT(", "));
			if (S->ChosenFixed.Num() == 0) Text += TEXT("\nThe case names no body that is in the scene: nothing to build.");
			if (LblCase.IsValid()) LblCase->SetText(FText::FromString(Text));
			S->Built.Reset();
			S->RebuildPointList();
			S->RebuildSupportList();
			S->DescribeModel();
		}

		// The panel method's trim for an aero case without a solved field:
		// the bodies' panelable entities (a boolean result's operands), the
		// mass report, the control by name. Build (and Solve) continue when
		// it lands.
		bool StartTrimIfNeeded(bool bThenSolve)
		{
			const FCaseRef* R = Current();
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			if (R == nullptr || !R->bAero || Doc == nullptr) return false;
			if (S->AeroField.IsValid()) return false;
			if (Trim.IsValid() && Trim->IsRunning()) { S->Message(LOCTEXT("TrimBusy", "The trim is still running.")); return true; }
			const aerotelic::LoadCasesSpec& Spec = Doc->GetLoadCases();
			if (R->Index >= static_cast<int32>(Spec.aero.size())) return false;
			const aerotelic::AeroLoadCaseSpec& A = Spec.aero[R->Index];
			TArray<AAerotelicEntityActor*> Panelable, ForMass;
			for (const FAerotelicBodyChoicePtr& C : S->ChosenBodies())
			{
				AAerotelicEntityActor* E = C->Operand.Entity.Get();
				if (E == nullptr) continue;
				ForMass.AddUnique(E);
				if (AerotelicAeroTrim::IsPanelable(E->GetEntityType())) Panelable.AddUnique(E);
				else if (E->GetEntityType() == EntityType::Boolean)
				{
					if (const FAerotelicBooleanModel* M = E->GetModelAs<FAerotelicBooleanModel>())
						for (const FAerotelicBooleanOperand& Op : M->Operands)
							if (Op.Entity.IsValid() && AerotelicAeroTrim::IsPanelable(Op.Entity->GetEntityType())) Panelable.AddUnique(Op.Entity.Get());
				}
			}
			if (Panelable.Num() == 0)
			{
				S->Message(LOCTEXT("NothingToPanel", "None of the case's bodies can be paneled (a wing, fuselage, nacelle or primitive, or a boolean result over them is needed)."));
				return true;
			}
			AerotelicAeroTrim::FSetup Setup;
			for (AAerotelicEntityActor* E : Panelable) Setup.Entities.Add(E);
			Setup.bSymmetry = AerotelicAeroTrim::DefaultSymmetry(Panelable);
			AerotelicAeroTrim::DefaultReferences(Panelable, Setup.ReferenceArea, Setup.ReferenceLength);
			AerotelicAeroTrim::FCondition Cond;
			Cond.Name = UTF8_TO_TCHAR(A.name.c_str());
			Cond.LoadFactor = A.load_factor;
			Cond.Speed = A.speed;
			Cond.Density = A.density;
			FString MassText;
			if (!AerotelicAeroTrim::MassAndCentre(ForMass, S->Units, Cond.MassKg, Cond.MassCentre, MassText))
			{
				S->Message(FText::FromString(TEXT("The case's bodies weigh nothing, so there is no weight to trim for: give them a structure (Scene tree > Structure).\n") + MassText));
				return true;
			}
			if (!A.control_entity.empty())
			{
				int32 Def = -1;
				for (const AerotelicAeroTrim::FControl& C : AerotelicAeroTrim::Controls(Panelable, Def))
				{
					if (C.Wing.IsValid() && C.Wing->EntityName == UTF8_TO_TCHAR(A.control_entity.c_str()) && C.Label.EndsWith(UTF8_TO_TCHAR(A.control_flap.c_str()))) { Cond.Control = C; break; }
				}
				if (!Cond.Control.IsSet()) UE_LOG(LogTemp, Warning, TEXT("StructuralFea: the control %s / %s of the aero case is not among the paneled bodies; trimming the lift alone"), UTF8_TO_TCHAR(A.control_entity.c_str()), UTF8_TO_TCHAR(A.control_flap.c_str()));
				if (Cond.Control.IsSet())
				{
					const aerotelic::WingModel* W = Cond.Control.Wing->GetModelAs<aerotelic::WingModel>();
					if (W != nullptr && Cond.Control.Flap < static_cast<int32>(W->flaps.size())) Cond.Delta0 = W->flaps[Cond.Control.Flap].deflection_deg;
				}
			}
			bSolveAfterTrim = bThenSolve;
			Trim = MakeShared<AerotelicAeroTrim::FRunner>();
			TWeakPtr<FDialog> Weak = AsShared();
			S->SetSolving(true);
			S->Status(FText::FromString(FString::Printf(TEXT("Trimming the aero case '%s' with the panel method..."), *Cond.Name)));
			Trim->Start(Setup, Cond, S->Units,
				[Weak](const FText& T) { if (TSharedPtr<FDialog> D = Weak.Pin()) D->S->ShowProgress(T.ToString()); },
				[Weak](const AerotelicAeroTrim::FOutcome& O)
				{
					TSharedPtr<FDialog> D = Weak.Pin();
					if (!D.IsValid()) return;
					D->S->SetSolving(false);
					if (!O.bOk)
					{
						D->S->Status(LOCTEXT("Ready", "Ready"));
						D->S->Message(FText::FromString(TEXT("The aero case could not be trimmed:\n") + O.Report));
						return;
					}
					if (UAerotelicDocumentSubsystem* Doc2 = UAerotelicDocumentSubsystem::Get()) Doc2->SetAeroLoadCase(O.Case);
						// Stored, but the trim did not reach its tolerance: the structure
						// is about to be analysed for a condition the aircraft is not
						// quite in, and the user has to be told before the solve.
						if (!O.Warning.IsEmpty())
						{
							D->S->Message(FText::FromString(FString::Printf(TEXT("The aero case was trimmed OFF TARGET and stored anyway: %s.\n\nThe loads below are that state's, not an equilibrium's. Trim the case again from this attitude in Model > Load Cases before believing the stresses.\n\n%s"), *O.Warning, *O.Report)));
						}
					D->S->AeroField = O.Case;
					UE_LOG(LogTemp, Log, TEXT("StructuralFea: %s"), *O.Report.Replace(TEXT("\n"), TEXT(" | ")));
					D->SelectCase(D->CaseIndex, true);
					D->S->AeroField = O.Case;
					if (D->bSolveAfterTrim) D->S->Solve();
					else if (D->S->BuildModel() && D->S->Tabs.IsValid()) D->S->Tabs->SetActiveTab(TabMesh);
				});
			return true;
		}

		void Build()
		{
			if (StartTrimIfNeeded(false)) return;
			if (S->BuildModel() && S->Tabs.IsValid()) S->Tabs->SetActiveTab(TabMesh);
		}

		void Solve()
		{
			if (StartTrimIfNeeded(true)) return;
			S->Solve();
		}

		void Close()
		{
			if (Trim.IsValid()) Trim->Cancel();
			Trim.Reset();
			if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
			{
				if (CasesHandle.IsValid()) Doc->OnLoadCasesChanged.Remove(CasesHandle);
				if (FieldsHandle.IsValid()) Doc->OnAeroLoadCasesChanged.Remove(FieldsHandle);
			}
			CasesHandle.Reset(); FieldsHandle.Reset();
			CaseCombo.Reset(); LblCase.Reset();
			S->Close();
		}
	};

	// Prefixed to kHelp when the dialog runs on TACS (2026-11-17).
	const TCHAR* kHelpTacs = TEXT(
		"TACS (FEA)\n\n"
		"This is the same dialog, the same load cases and the same model as Analysis > Structural (Kratos FEA), solved on TACS -- the Toolkit for the Analysis of Composite Structures (smdogroup.github.io/tacs) -- through pyTACS. TACS has no Windows build, so the application runs tools/structural_fea_tacs.py inside the Windows Subsystem for Linux: the environment is looked for at AEROTELIC_TACS_PYTHON (wsl:<distro>:<python>) or the default distribution's ~/micromamba/envs/tacs; Analysis > TACS (FEA) > Check Installation reports what was found and solves a small plate. The job directory handed over is identical to the Kratos solver's, so any case can be run on both and compared.\n\n"
		"WHAT DIFFERS\n"
		"The shell: TACS's Tri3Shell is a first-order shear deformation (Mindlin) triangle with MITC tying against shear locking and a drilling penalty, where Kratos's default is the Kirchhoff thin triangle; on thin skins they agree to a percent or two. The linear solver: GMRES with TACS's ILU preconditioner, near-complete at its default fill (a direct solve in all but name); the force balance is still the check. The pushrods: TACS has no truss element, so a rod is an axial 6-dof spring of stiffness E A / L along its own axis with its mass lumped on its ends -- the same stiffness and resultant. The stresses: recovered from the displacement and rotation fields (the membrane strain of each flat triangle, the Mindlin curvatures of its rotations), von Mises at the top, middle and bottom of an isotropic shell, the ply stresses and the Tsai-Wu reserve factor of a laminate, solved as the application's laminate.hpp does (F12* = -1/2). The inertial loads: gravity and the free-flight field go in through TACS's own consistent mass matrix. The reactions: K u - F at the fixed nodes.\n\n"
		"THE ADJOINT\n"
		"TACS is here for its gradients. Every isotropic skin and rib layer is one TACS component with one thickness design variable; after the solve the adjoint is run for the structural mass and for the KS-aggregated failure index (von Mises over the allowable, ~ its peak, 1 is failure) of every component that has an allowable, and the Result lists d mass / d t and d index / d t for every variable, per metre of thickness. Laminates have no design variable yet (TACS's laminate section has none; its smeared section would). Derivatives with respect to the geometry -- TACS gives d f / d X of every node; the application must supply d X / d parameter -- are the next step.\n\n"
		"-----\n\n");

	const TCHAR* kHelp = TEXT(
		"STRUCTURAL (KRATOS FEA)\n\n"
		"A linear static finite-element analysis of a LOAD CASE on KratosMultiphysics' StructuralMechanicsApplication, run as a separate process in the WingDesigner environment. Everything the solver sees is SI (the mesh scaled by the length unit, the materials as they are, loads in newtons and pascals); the dialog converts the results back.\n\n"
		"LOAD CASES (2026-11-15)\n"
		"The cases are defined under Model > Load Cases and saved with the project; this dialog only picks one, renders it and solves it. An AERO case is a point of the V-n diagram -- a speed, an air density and a load factor n -- flown by a set of solid bodies: the panel method, on the bodies' own triangles, trims the configuration to it (lift n W through the mass centre, no moment about it, by the angle of attack and one flap) and its pressures load the shells in free flight (inertia relief, gravity at 1 g), so the structure feels n at its mass centre; Build Model runs that trim when the case has not been solved yet. A STRUCTURAL case is the bodies, an inertial load factor, a uniform pressure, point forces and the constraints -- support planes (or patches of them), or free flight with a reference point.\n\n"
		"BODIES\n"
		"The case names SOLID BODIES, not parameterisations: a wing's main element and its flap, a propeller's blades, a boolean result's separate shells. What each body is comes from its Structure in the Scene tree: a thin-walled part of SHELLS of its skin thickness and material on its own triangles; and, for each layer its parameterisation generated inside it -- a wing's ribs, spars and stringers -- unloaded shells of that layer's thickness and material. The model is shells only. The bodies are not joined to one another: each must reach a support on its own or through a body it was fused with; whatever has no load path -- a control surface in its gap, a payload bay inside the fuselage -- is LEFT OUT automatically and named on the Mesh and Support tabs.\n\n"
		"THE GRID\n"
		"Every body is analysed on ITS OWN GRID: the triangles its Mesh tab defines, at the viewport's tolerance rule, the same ones the viewport shows and mass properties weigh. The shells ARE those triangles, a node per vertex, so the Mesh tab is where the cost of a solve is set. A body's generated structure is built on that same grid: the ribs are meshed by gmsh on the skin's rims, a spar's web takes the rib's chord nodes, so skin, ribs and spars share their degrees of freedom.\n\n"
		"COST\n"
		"Build Model counts the degrees of freedom (six per node) and says which solver the automatic choice takes: the direct solver (exact, one thread) up to 150,000 DOF, the iterative solver (amgcl, every core; the force balance checks it afterwards) beyond. Past that budget Solve asks first.\n\n"
		"SET-UP\n"
		"Pick the case. Build Model builds the model (the ribs through gmsh; a body's rib mesh is kept while the dialog is open) and derives what the solver will be handed for the picture tabs, without solving; for an unsolved aero case it runs the panel method's trim first. Solve does the same and then runs Kratos on a worker thread -- the window keeps painting, the status line shows the solver's stage and the elapsed time, Cancel ends it -- writing the displacement and stress fields to the VTK file named and the peaks, margins and force balance into the Result group; with a number of natural frequencies set it then solves the eigenvalue problem too.\n\n"
		"MESH, LOADS, MASSES, SUPPORT\n"
		"The pictures of the case on the built model: the wetted faces by region with their edges and the rib shells inside; the loads as arrows (orange the pressure as the traction on up to ~1500 faces, red each point force at its node, green the inertial load at the centre of mass) with, for an aero case, the mapped resultant against the panel method's own and against n W; the point masses hung on the model; a red marker on every node the supports fix, each plane as a frame, or in free flight the one reference node, and what reaches no support. To change any of it, change the case under Model > Load Cases.\n\n"
		"RESULT\n"
		"Linear static, small displacement, linear elastic, the linear thin shell triangle (its reactions balance the loads exactly). The pressures are applied as the consistent nodal loads of each face. The reactions at the supports must balance what was applied: the Force balance line says by how much they miss, and a miss of more than one percent means the linear solver did not converge -- the solution is then reported as NOT converged and nothing in it is to be trusted. In free flight the same line gives the rigid-body accelerations, the load factor at the mass centre (the trim's n for an aero case: the check that the two models agree) and the reference node's reaction. Margin is allowable / peak von Mises - 1 for each region. The natural frequencies are the lowest modes of the same model with no loads, in hertz; under a model scale they are the replica's. The VTK holds the displacement, the stresses and the mode shapes for ParaView, in SI.");
}

void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status, AerotelicStructuralFea::EBackend Backend)
{
	const bool bTacs = Backend == AerotelicStructuralFea::EBackend::Tacs;
	const FText Title = bTacs ? LOCTEXT("TitleTacs", "TACS (FEA)") : LOCTEXT("Title", "Structural (Kratos FEA)");
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	TArray<FAerotelicBodyChoicePtr> Choices = AnalysableChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoBody", "Nothing to analyse: add a body first. Any body with a structural material and a skin thickness under Structure... in the Scene tree can be analysed, as shells of that thickness with its generated structure as shells inside."),
			Title);
		return;
	}
	{
		// The script, and for TACS its WSL environment (the Kratos
		// interpreter is only checked when the solve starts, as before).
		FString Why;
		if (AerotelicStructuralFea::FindScript(Backend).IsEmpty() || (bTacs && !AerotelicStructuralFea::IsAvailable(Backend, Why)))
		{
			if (Why.IsEmpty()) Why = FString::Printf(TEXT("tools\\%s is not installed beside the application."), bTacs ? TEXT("structural_fea_tacs.py") : TEXT("structural_fea.py"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Why), Title);
			return;
		}
	}

	const FVector2D Size(1180, 980);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(Title)
		.ClientSize(Size)
		.SupportsMaximize(true)
		.SupportsMinimize(true);

	TSharedRef<FDialog> D = MakeShared<FDialog>();
	TSharedRef<FState> S = MakeShared<FState>();
	D->S = S;
	D->Window = Window;
	S->Status = Status;
	S->Window = Window;
	S->Backend = Backend;
	S->Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	FillOptions(S, Choices);

	auto Combo = [S](TArray<TSharedPtr<FString>> FState::*Options, int32 FState::*Index)
	{
		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&((*S).*Options))
			.InitiallySelectedItem(((*S).*Options)[(*S).*Index])
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> Item) { return SNew(STextBlock).Text(FText::FromString(*Item)); })
			.OnSelectionChanged_Lambda([S, Options, Index](TSharedPtr<FString> Item, ESelectInfo::Type)
			{
				const int32 I = ((*S).*Options).IndexOfByKey(Item);
				if (I != INDEX_NONE) (*S).*Index = I;
			})
			[ SNew(STextBlock).Text_Lambda([S, Options, Index]() { return FText::FromString(*((*S).*Options)[(*S).*Index]); }) ];
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

	// The Solver group's hint, by backend (two LOCTEXTs cannot share a ?:).
	FText ModelHint;
	if (bTacs)
	{
		ModelHint = LOCTEXT("ModelHintTacs",
			"The model is SHELLS ONLY, the same one the Kratos solver gets: a body's own surface triangles as shells of its skin thickness and material, each generated layer (ribs, spars, stringers) as unloaded shells inside it on the same grid. "
			"TACS's shell is the Tri3Shell: a first-order shear deformation (Mindlin) triangle with MITC tying and a drilling penalty, six degrees of freedom a node; the mass items, hinges and pushrods go in as RBE3, RBE2 and axial springs. The solve runs inside WSL (its first start takes a few seconds); the force balance is checked after every solve. "
			"The natural frequencies are the same model's lowest modes with no loads (TACS's Lanczos eigensolver). TACS also runs the ADJOINT: the Result lists the gradients of the mass and of each region's failure index with respect to its thickness.");
	}
	else
	{
		ModelHint = LOCTEXT("ModelHint",
			"The model is SHELLS ONLY: a body's own surface triangles become shell elements of its skin thickness and material -- the linear thin triangle, three nodes, six degrees of freedom each; each generated layer (a wing's ribs, spars, stringers) becomes unloaded shells inside the body on the same grid. "
			"Build Model reports the degrees of freedom and which solver will run: the direct solver is exact but single-threaded, so the automatic choice takes it up to 150,000 degrees of freedom and the iterative solver (amgcl) beyond; the force balance is checked after every solve. "
			"The natural frequencies are the same model's lowest modes with no loads (Spectra's shift-invert eigensolver).");
	}

	// ---- tab 1: set-up ----
	D->RefreshCases();
	TSharedRef<SWidget> SetupTab =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("CaseGroup", "Load case (Model > Load Cases)"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					FormRow(LOCTEXT("Case", "Case:"),
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1.0f)
						[
							SAssignNew(D->CaseCombo, SComboBox<TSharedPtr<FString>>)
							.OptionsSource(&D->CaseNames)
							.InitiallySelectedItem(D->CaseNames.IsValidIndex(D->CaseIndex) ? D->CaseNames[D->CaseIndex] : nullptr)
							.OnGenerateWidget_Lambda([](TSharedPtr<FString> N) { return SNew(STextBlock).Text(FText::FromString(*N)); })
							.OnSelectionChanged_Lambda([D](TSharedPtr<FString> N, ESelectInfo::Type)
							{
								if (!N.IsValid()) return;
								const int32 I = D->CaseNames.IndexOfByKey(N);
								if (I >= 0) D->SelectCase(I);
							})
							[ SNew(STextBlock).Text_Lambda([D]() { return D->CaseNames.IsValidIndex(D->CaseIndex) ? FText::FromString(*D->CaseNames[D->CaseIndex]) : LOCTEXT("NoCaseShort", "(no load case defined)"); }) ]
						]
						+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
						[ Button(LOCTEXT("ReloadCases", "Reload"), [D]() { D->RefreshCases(); }) ], 110.0f)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4) [ SAssignNew(D->LblCase, STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4) [ Stat(S->LblMaterials) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("CaseHint",
						"The cases are defined and saved under Model > Load Cases: an aero case is a point of the V-n diagram, solved by the panel method's trim and flown free; a structural case carries its own bodies, loads and constraints. "
						"Pick one, Build Model to see the mesh, the loads, the masses and the constraints on the real model (the tabs), then Solve. Reload picks up cases defined while this window is open."))
				])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("ModelGroup", "Solver"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Solver", "Linear solver:"), Combo(&FState::SolverOptions, &FState::SolverIndex)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("ShellElement", "Shell element:"), Combo(&FState::ShellOptions, &FState::ShellIndex)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[ FormRow(LOCTEXT("Modes", "Natural frequencies to compute (0 = none):"), MakeStateSpin(S, &FState::Modes, 0.0, 100.0, 1.0, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[ MakeInfoLabel(ModelHint) ])
		]
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Result", "Result"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MeshStat", "Mesh:"), Stat(S->LblMesh)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("DispStat", "Max displacement:"), Stat(S->LblDisp)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("StressStat", "Max von Mises:"), Stat(S->LblStress)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MarginStat", "Per region:"), Stat(S->LblMargins)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("BalanceStat", "Force balance:"), Stat(S->LblBalance)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("ModesStat", "Natural frequencies:"), Stat(S->LblModes)) ]
				+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("TimeStat", "Took:"), Stat(S->LblTime)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ResultHint",
						"Linear static, small displacement, linear elastic. The reactions at the supports must balance what was applied -- gravity on the mass, the pressure's resultant, the point forces. Margin is allowable / peak von Mises - 1 for each region: a body's skin, or one of its generated layers. "
						"The natural frequencies are in hertz whatever the project's units, lowest first (a replica's are the replica's). The VTK holds the displacement, the stresses and each mode shape for ParaView, in SI."))
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

	// The pictures, read-only: no side panels, no patch dragging.
	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SetupTab", "Set-up"), SetupTab });
	Tabs.Add({ LOCTEXT("MeshTab", "Mesh"), MakePictureTab(S, TabMesh, LOCTEXT("MeshEmpty", "Build the model (Set-up tab) to see the mesh the solver is handed: its wetted faces by region, with their edges."), nullptr) });
	Tabs.Add({ LOCTEXT("LoadsTab", "Loads"), MakePictureTab(S, TabLoads, LOCTEXT("LoadsEmpty", "Build the model (Set-up tab) to see the case's loads as vectors: the pressure on the wetted faces, each point force at its node, the inertial load at the centre of mass."), nullptr) });
	Tabs.Add({ LOCTEXT("MassesTab", "Masses"), MakePictureTab(S, TabMasses, LOCTEXT("MassesEmpty", "Build the model (Set-up tab) to see the mass items hung on it, each with bars to the nodes that carry it, and the mass centre of everything."), nullptr) });
	Tabs.Add({ LOCTEXT("SupportTab", "Support"), MakePictureTab(S, TabSupport, LOCTEXT("SupportEmpty", "Build the model (Set-up tab) to see the case's constraints: a marker on every node the supports fix, each plane as a frame, and what reaches no support -- or, in free flight, the one reference node."), nullptr) });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(FText::FromString(bTacs ? FString(kHelpTacs) + kHelp : FString(kHelp))) });

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SAssignNew(S->Tabs, SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 12, 0)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 3)
					[ SAssignNew(S->LblProgress, STextBlock).AutoWrapText(true).Visibility(EVisibility::Collapsed).ColorAndOpacity(FSlateColor(FLinearColor(0.9f, 0.9f, 0.9f))) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ SNew(SBox).HeightOverride(8) [ SAssignNew(S->ProgressBar, SProgressBar).Percent(TOptional<float>()).Visibility(EVisibility::Collapsed) ] ]
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(S->BuildButton, SButton).Text(LOCTEXT("BuildBtn", "Build Model")).HAlign(HAlign_Center).OnClicked_Lambda([D]() { D->Build(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(S->SolveButton, SButton).Text(LOCTEXT("SolveBtn", "Solve")).HAlign(HAlign_Center).OnClicked_Lambda([D]() { D->Solve(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(S->CancelButton, SButton).Text(LOCTEXT("CancelBtn", "Cancel")).HAlign(HAlign_Center).IsEnabled(false).OnClicked_Lambda([D]() { if (D->Trim.IsValid()) D->Trim->Cancel(); D->S->CancelSolve(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([D](const TSharedRef<SWindow>&) { D->Close(); }));

	if (Doc != nullptr)
	{
		TWeakPtr<FDialog> Weak = D;
		D->CasesHandle = Doc->OnLoadCasesChanged.AddLambda([Weak]() { if (TSharedPtr<FDialog> P = Weak.Pin()) P->RefreshCases(); });
		D->FieldsHandle = Doc->OnAeroLoadCasesChanged.AddLambda([Weak]() { if (TSharedPtr<FDialog> P = Weak.Pin()) P->SelectCase(P->CaseIndex, true); });
	}
	D->SelectCase(D->CaseIndex, true);
	S->RebuildMassList();
	Status(LOCTEXT("Ready", "Pick a load case; Build Model to inspect it; Solve to run."));
	ShowAerotelicDialogWindow(Window, Size);

	if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicFeaSmoke")))
	{
		// A developer run: the first structural case of the project, or the
		// scene's bodies as one case with the sample's defaults.
		S->bSmoke = true;
		for (const FAerotelicBodyChoicePtr& C : Choices)
		{
			AAerotelicEntityActor* E = C->Operand.Entity.Get();
			if (E == nullptr || E->IsDerived() || HasStructuralMaterial(E->GetMassSpec())) continue;
			if (const aerotelic::Material* Preset = aerotelic::FindMaterialPreset("Aluminium 2024-T3 sheet"))
			{
				aerotelic::MassSpec M = E->GetMassSpec();
				M.mode = aerotelic::MassMode::Material;
				M.material = *Preset;
				const FBox B = E->GetPlacedBounds(false);
				M.skin_thickness = B.IsValid ? 0.01 * B.GetSize().GetMax() : 0.01;
				E->SetMassSpec(M);
			}
		}
		if (D->Current() == nullptr)
		{
			S->CaseName = TEXT("smoke");
			S->ChosenFixed = DefaultChosen(Choices);
			S->LoadFactor = 1.0;
			S->Pressure = 1000.0 / S->Units.StressToSI();
			TSharedPtr<FPointRow> P = MakeShared<FPointRow>();
			S->PointRows.Add(P);
			S->SupportRows.Add(MakeShared<FSupportRow>());
		}
		if (S->BuildModel()) S->RunSmoke();
	}
}

#undef LOCTEXT_NAMESPACE
