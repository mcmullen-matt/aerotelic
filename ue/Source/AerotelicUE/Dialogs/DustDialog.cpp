#include "DustDialog.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "aerotelic/dust_case.hpp"
#include "aerotelic/propeller_model.hpp"
#include "aerotelic/units.hpp"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "DustDialog"

namespace
{
	FString Thousands(int64 N)
	{
		return FText::AsNumber(N).ToString();
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(210) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-")))
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)))
			.AutoWrapText(true);
	}

	// ---- placing a rotor frame --------------------------------------------
	//
	// DUST's rotor multiplicity wants the blade in HUB-LOCAL coordinates
	// with the placement carried by the reference frame, so the dialog has
	// to take the entity's placement apart rather than bake it into the
	// mesh the way every other body here is exported.
	//
	// Nothing in this block touches the geometry engine: it reads the
	// model's own local meshes and the entity's transform, both already
	// public, and does its own arithmetic.
	struct FRot3
	{
		// Row-major: m[r][c]. Columns are the images of the basis vectors,
		// i.e. the frame's axes in parent components -- which is the order
		// DUST wants them written in.
		double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

		aerotelic::Vec3 Column(int c) const { return {m[0][c], m[1][c], m[2][c]}; }
		aerotelic::Vec3 Apply(const aerotelic::Vec3& v) const
		{
			return { m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
			         m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
			         m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z };
		}
	};

	// Rodrigues' rotation formula; a zero or degenerate axis is the identity.
	FRot3 AxisAngleToMatrix(double ax, double ay, double az, double angle_deg)
	{
		FRot3 R;
		const double n = FMath::Sqrt(ax * ax + ay * ay + az * az);
		if (!(n > 0.0) || FMath::IsNearlyZero(angle_deg))
		{
			return R;
		}
		const double x = ax / n, y = ay / n, z = az / n;
		const double a = FMath::DegreesToRadians(angle_deg);
		const double c = FMath::Cos(a), s = FMath::Sin(a), t = 1.0 - c;
		R.m[0][0] = t * x * x + c;     R.m[0][1] = t * x * y - s * z; R.m[0][2] = t * x * z + s * y;
		R.m[1][0] = t * x * y + s * z; R.m[1][1] = t * y * y + c;     R.m[1][2] = t * y * z - s * x;
		R.m[2][0] = t * x * z - s * y; R.m[2][1] = t * y * z + s * x; R.m[2][2] = t * z * z + c;
		return R;
	}

	// A mirrored entity is reflected about world y = 0 AFTER placement, and
	// a reflection cannot be a DUST orientation matrix (its determinant is
	// -1). Writing M for that reflection, M*R = (M*R*M)*M: the proper
	// rotation M*R*M becomes the frame, and the leftover M is folded into
	// the blade's own local coordinates. The rotor then turns the other
	// way, which the caller handles by negating the rate.
	FRot3 MirrorConjugate(const FRot3& R)
	{
		// M = diag(1, -1, 1), so conjugation flips the sign of every entry
		// with exactly one index equal to 1.
		FRot3 out = R;
		for (int r = 0; r < 3; ++r)
		{
			for (int c = 0; c < 3; ++c)
			{
				const bool one = (r == 1) != (c == 1);
				if (one) out.m[r][c] = -out.m[r][c];
			}
		}
		return out;
	}

	// The case folder the dialog starts with: beside the project, named
	// after it, so a user who only presses Write Case still gets the
	// files somewhere sensible.
	FString DefaultCaseDir()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_dust"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_dust"));
	}

	const FText kHelp = LOCTEXT("DustHelp",
"DUST (VORTEX PARTICLE METHOD)\n"
"\n"
"DUST is a mid-fidelity unsteady aerodynamic solver from Politecnico di\n"
"Milano. The bodies carry surface panels and the vorticity they shed is\n"
"carried away as free vortex PARTICLES, so the wake rolls up, drifts and\n"
"blows back over the geometry instead of being prescribed. That is the\n"
"difference from Analysis -> Potential Flow, which has no wake at all and\n"
"therefore no lift: this one lifts.\n"
"\n"
"This dialog WRITES A CASE. It does not run it -- DUST is a separate\n"
"program with its own Fortran, HDF5, CGNS and LAPACK build, and keeping it\n"
"at arm's length means a case can be written here and run anywhere.\n"
"\n"
"\n"
"RUNNING THE CASE\n"
"\n"
"Open a command prompt in the case folder and run these three, in order:\n"
"\n"
"    dust_pre  dust_pre.in\n"
"    dust      dust.in\n"
"    dust_post dust_post.in\n"
"\n"
"On Windows they are dust_pre.exe, dust.exe and dust_post.exe, and the\n"
"folder DUST was installed into has to be on PATH first; in a PowerShell\n"
"prompt that is\n"
"\n"
"    $env:PATH = \"C:\\path\\to\\dust\\bin;\" + $env:PATH\n"
"\n"
"What each one does:\n"
"\n"
"  dust_pre   reads dust_pre.in and the files under components/, takes in\n"
"             the meshes under geometry/, finds each body's trailing edge\n"
"             and writes the single geo_input.h5 the solver reads. It is\n"
"             quick. Run it again whenever the geometry changes.\n"
"\n"
"  dust       reads dust.in and solves, writing one result file per time\n"
"             step into Output/. This is the long one. It prints the step\n"
"             number and the elapsed time as it goes, so progress is\n"
"             visible; stopping it just loses the steps not yet written.\n"
"\n"
"  dust_post  reads dust_post.in and turns the results into things to look\n"
"             at: a VTK series in Postprocessing/ (open the post_viz-....vtu\n"
"             files in ParaView to see the surface and the wake particles)\n"
"             and load tables, one for the whole model and one per body.\n"
"\n"
"The case folder also holds a README.txt repeating all of this, with the\n"
"numbers this particular case was written with.\n"
"\n"
"\n"
"WHAT THE DIALOG WRITES\n"
"\n"
"    dust_pre.in            the components and where their meshes are\n"
"    dust.in                the solver settings on the Flow and Wake tabs\n"
"    dust_post.in           the visualisation and the load tables\n"
"    References.in          reference frames; see MOVING PARTS below\n"
"    components/<body>.in   one per body: mesh type, element type, trailing\n"
"                           edge detection\n"
"    geometry/<body>_rr.dat the body's nodes, three to a line\n"
"    geometry/<body>_ee.dat its triangles, four indices to a line\n"
"    Output/                empty, for the solver\n"
"    Postprocessing/        empty, for the postprocessor\n"
"\n"
"The meshes go out in DUST's \"basic\" format. DUST's element array is four\n"
"wide and reads a zero fourth index as a triangle, which is exactly what\n"
"this application's loft triangulation produces, so no conversion and no\n"
"CGNS library is needed.\n"
"\n"
"\n"
"GEOMETRY\n"
"\n"
"The bodies are written as surface panels (DUST's el_type = p): a source\n"
"and a doublet on each triangle. The other element types are not offered\n"
"because they cannot be -- vortex lattice and lifting line need a\n"
"structured chordwise/spanwise mesh, which DUST only builds from its own\n"
"parametric input, not from a triangle mesh.\n"
"\n"
"Surface deflection is the only control over how fine the panelling is,\n"
"and it matters more here than in an export: DUST's panel method builds a\n"
"DENSE matrix over the panels, so memory and solve time go as the SQUARE\n"
"of the panel count. The Geometry tab shows the size of that matrix next\n"
"to the panel count. A few thousand panels is comfortable; thirty thousand\n"
"is about seven gigabytes and will not run.\n"
"\n"
"BODIES MUST NOT LIE ON TOP OF ONE ANOTHER. Two surfaces in the same place\n"
"give the panel method two unknowns with identical influence, the matrix\n"
"is singular, and the run diverges on the very first step -- it does not\n"
"settle down later. The everyday way to cause this is to pick a wing AND\n"
"its mirror: each is a closed body and their root caps are the same wall\n"
"seen from both sides. Fuse them first with Geometry -> Boolean Operation\n"
"and pick the result, which has no internal wall. The Geometry tab warns\n"
"when it finds bodies overlapping, but it only checks for surfaces lying\n"
"on each other, not for one body poking through another.\n"
"\n"
"A wake is shed only from an edge DUST recognises as a trailing edge: one\n"
"where the two panels meeting at it face away from each other by more than\n"
"the inner-product threshold. A wing's sharp trailing edge is found\n"
"easily; a blunt body such as a fuselage has no such edge, sheds nothing,\n"
"and simply contributes thickness. dust_pre prints how many trailing-edge\n"
"elements it found for each body -- if a lifting surface reports zero,\n"
"raise the threshold towards 0 and run dust_pre again.\n"
"\n"
"\n"
"FLOW AND TIME\n"
"\n"
"The flow runs along +x, tilted towards +z by the angle of attack and\n"
"towards +y by the sideslip, in the application's own axes. Density, the\n"
"speed of sound and the viscosity scale the loads and feed the\n"
"compressibility and diffusion models.\n"
"\n"
"Time step and end time may be left at zero, which means \"work it out\".\n"
"The reference length is the model's extent ALONG THE FLOW -- the chord of\n"
"a wing, the length of a fuselage. The step is then the time to cross that\n"
"length divided by the steps-per-length figure, and the run lasts for the\n"
"requested number of crossings. Both derived values are shown on the Flow\n"
"tab as soon as the geometry is chosen. A wake needs several crossings to\n"
"establish before the loads mean anything: the first steps always show a\n"
"large transient as the wake starts from nothing.\n"
"\n"
"\n"
"THE WAKE\n"
"\n"
"Rows of panel wake is how many rows stay attached as panels at the\n"
"trailing edge before the wake becomes particles. One -- DUST's own\n"
"default -- is what makes this a particle method.\n"
"\n"
"The particle budget caps how many particles may exist at once, and the\n"
"particle box bounds where they may be: a particle that leaves the box is\n"
"dropped. Together they bound the cost of a long run. The box is set from\n"
"the model's size, with extra room downstream because that is where the\n"
"wake goes.\n"
"\n"
"Particle core radius regularises the velocity a particle induces close\n"
"to itself. Left at zero it is taken as the mean panel size, so a particle\n"
"is about as big as the panel that shed it. Much smaller and the wake\n"
"becomes noisy; much larger and it smears.\n"
"\n"
"Vortex stretching and diffusion are the two physical terms that make this\n"
"a vortex particle method rather than a cloud of fixed blobs. Penetration\n"
"avoidance stops particles drifting through a solid surface. The fast\n"
"multipole method makes the particle-particle interaction tractable --\n"
"with it off, the cost is quadratic in the particle count, which is only\n"
"sensible for a short run or a small wake. The octree settings below it\n"
"exist to serve it and are sized from the particle box.\n"
"\n"
"\n"
"MOVING PARTS\n"
"\n"
"Every body is written on reference frame 0, the global one, so the case\n"
"as written is a steady body in a moving stream. References.in is written\n"
"with a commented-out template for a moving frame: uncomment it, give it a\n"
"tag, and point a component's ref_tag in dust_pre.in at that tag. A rotor\n"
"is the usual reason -- a constant rotation about its own axis. Rotor\n"
"geometry still has to come from this application as a body like any other.\n"
"\n"
"\n"
"READING THE RESULT\n"
"\n"
"Postprocessing/post_loads_total.dat has one line per time step: the time,\n"
"then the force and the moment, then the reference frame. There is one\n"
"such file per body as well. The loads are dimensional and in SI: the\n"
"case is written in metres, kilograms and seconds, converted from the\n"
"project's units (Geometry > Units), so forces come back in newtons and\n"
"moments in newton-metres whatever the project is drawn in.\n"
"\n"
"Expect the first steps to be dominated by the starting transient, and\n"
"expect a surface panel method with coarse panelling to under-resolve the\n"
"leading-edge suction, which shows up as a drag that is too low and can\n"
"even come out negative. Refine the panelling before believing a drag\n"
"number; lift is far less sensitive.\n");
}

void ShowAerotelicDustDialog(TFunction<void(const FText&)> Status)
{
	// Every scene body, as the boolean, export and potential-flow dialogs
	// offer them: a lattice or a boolean result panels as readily as a
	// loft, since all DUST is handed is a triangulation.
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("Nothing", "Nothing in the scene to analyse."), LOCTEXT("Title", "DUST (Vortex Particle Method)"));
		return;
	}

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "DUST (Vortex Particle Method)"))
		.ClientSize(FVector2D(680, 860))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	TSharedPtr<SAerotelicBodyPicker> Picker;
	TSharedPtr<SEditableTextBox> DirBox, NameBox;
	TSharedPtr<STextBlock> LblPanels, LblSystem, LblOverlap, LblStep, LblRun, LblRadius, LblBox, LblOctree, LblWritten;
	TSharedPtr<SCheckBox> FmmCheck, StretchCheck, DiffusionCheck, PenetrationCheck, PostCheck;

	double Deflection = 0.05;

	// The project's units: every field is typed in them and DUST is handed
	// SI at Write. The option defaults are SI constants (sea-level air), so
	// they are converted the other way first, or 1.225 would sit in a
	// lbm/ft^3 field and reach DUST as 19.6 kg/m^3.
	const UAerotelicDocumentSubsystem* UnitsDoc = UAerotelicDocumentSubsystem::Get();
	const aerotelic::UnitSystem Units = UnitsDoc != nullptr ? UnitsDoc->GetUnits() : aerotelic::UnitSystem{};
	const FText SpeedSuffix = FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(Units.SpeedName().c_str()));
	const FText DensitySuffix = FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(Units.DensityName().c_str()));
	const FText ViscositySuffix = FText::FromString(FString::Printf(TEXT(" %s/(%s s)"), UTF8_TO_TCHAR(Units.MassName()), UTF8_TO_TCHAR(Units.LengthName())));
	const FText LengthSuffix = FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(Units.LengthName()));

	aerotelic::DustCaseOptions Opt;
	Opt.basename = "dust";
	Opt.speed = 30.0 / Units.SpeedToSI();
	Opt.rho_inf = 1.225 / Units.DensityToSI();
	Opt.p_inf = 101325.0 / Units.PressureToSI();
	Opt.a_inf = 340.0 / Units.SpeedToSI();
	Opt.mu_inf = 1.81e-5 / Units.ViscosityToSI();

	// The chosen bodies, triangulated at the current deflection. Rebuilt
	// only when the selection or the deflection changes, since that is
	// the expensive part; every other control just re-derives.
	std::vector<aerotelic::DustComponent> Components;
	std::vector<aerotelic::DustFrame> Frames;
	TArray<FString> Failures;
	aerotelic::DustCaseResult Derived;
	bool bDerived = false;

	// One propeller among the chosen bodies. Rotational speed is an
	// ANALYSIS setting, deliberately not a property of the propeller
	// model: DUST is downstream of the geometry and must not reach back
	// into it. That also means it is not saved with the project -- it
	// lives for the life of this dialog.
	struct FRotor
	{
		FString Label;
		TWeakObjectPtr<AAerotelicEntityActor> Entity;
		bool bMirror = false;
		int32 BladeCount = 0;
		double Radius = 0.0;
		double Rpm = 0.0;     // 0 leaves the propeller standing still
		FString Tag;
	};
	TArray<FRotor> Rotors;
	TSharedPtr<SVerticalBox> RotorRows;
	TSharedPtr<STextBlock> LblRotorNote;
	// Assigned once the Rotors tab exists; Rebuild() calls it to redraw the
	// per-propeller rows, which it cannot do directly because the rows'
	// own callbacks have to be able to call Rebuild().
	TFunction<void()> RefreshRotors;

	auto Rederive = [&]()
	{
		std::string Error;
		bDerived = !Components.empty() && aerotelic::DeriveDustCase(Components, Frames, Opt, Derived, Error);
		auto Dash = [](TSharedPtr<STextBlock>& L) { L->SetText(FText::FromString(TEXT("-"))); };
		if (!bDerived)
		{
			Dash(LblPanels); Dash(LblSystem); Dash(LblOverlap);
			Dash(LblStep); Dash(LblRun); Dash(LblRadius); Dash(LblBox); Dash(LblOctree);
			return;
		}
		LblPanels->SetText(FText::FromString(FString::Printf(TEXT("%s panels, %s nodes across %s %s; mean panel %.4g"),
			*Thousands((int64)Derived.panels), *Thousands((int64)Derived.nodes), *Thousands((int64)Derived.components),
			Derived.components == 1 ? TEXT("body") : TEXT("bodies"), Derived.mean_panel_size)));
		LblSystem->SetText(FText::FromString(FString::Printf(TEXT("%.1f MB dense (it grows as the SQUARE of the panel count)"),
			Derived.linear_system_mb)));
		if (Derived.coincident_panels > 0)
		{
			LblOverlap->SetText(FText::FromString(FString::Printf(
				TEXT("WARNING: %s panels of \"%s\" and \"%s\" lie on top of each other. The panel matrix will be singular and the run will diverge on step 1. Fuse them with a boolean operation and select the result."),
				*Thousands((int64)Derived.coincident_panels), UTF8_TO_TCHAR(Derived.coincident_a.c_str()), UTF8_TO_TCHAR(Derived.coincident_b.c_str()))));
			LblOverlap->SetColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.55f, 0.25f)));
		}
		else
		{
			LblOverlap->SetText(LOCTEXT("NoOverlap", "No bodies found lying on top of one another."));
			LblOverlap->SetColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
		}
		LblStep->SetText(FText::FromString(FString::Printf(TEXT("%.6g s  (%.4g long along the flow, crossed in %.4g s)"),
			Derived.dt, Derived.reference_length, Derived.reference_length / FMath::Max(1e-12, Opt.speed))));
		LblRun->SetText(FText::FromString(FString::Printf(TEXT("%s steps to t = %.6g s  (%.3g crossings)"),
			*Thousands((int64)Derived.timesteps), Derived.tend,
			(Derived.tend - Opt.tstart) * Opt.speed / FMath::Max(1e-12, Derived.reference_length))));
		LblRadius->SetText(FText::FromString(FString::Printf(TEXT("%.6g"), Derived.vortex_rad)));
		LblBox->SetText(FText::FromString(FString::Printf(TEXT("(%.4g, %.4g, %.4g) to (%.4g, %.4g, %.4g)"),
			Derived.box_min.x, Derived.box_min.y, Derived.box_min.z, Derived.box_max.x, Derived.box_max.y, Derived.box_max.z)));
		LblOctree->SetText(FText::FromString(FString::Printf(TEXT("%d x %d x %d boxes of %.4g, %d levels"),
			Derived.n_box[0], Derived.n_box[1], Derived.n_box[2], Derived.octree_box_length, Opt.n_octree_levels)));
	};

	auto Rebuild = [&]()
	{
		// Rotational speeds already typed in survive a re-panel: they are
		// keyed by the picker's label, not by position.
		TMap<FString, double> KeptRpm;
		for (const FRotor& R : Rotors) KeptRpm.Add(R.Label, R.Rpm);

		Components.clear();
		Frames.clear();
		Rotors.Reset();
		Failures.Reset();
		const TArray<FAerotelicBodyChoicePtr>& Chosen = Picker->GetChosen();
		if (Chosen.Num() == 0)
		{
			Rederive();
			Status(LOCTEXT("SelectBody", "Select the geometry to analyse."));
			return;
		}
		Status(LOCTEXT("Meshing", "Panelling the bodies..."));
		for (const FAerotelicBodyChoicePtr& C : Chosen)
		{
			if (!C->Operand.Entity.IsValid())
			{
				Failures.Add(C->Label + TEXT(": the entity is gone"));
				continue;
			}
			AAerotelicEntityActor* Entity = C->Operand.Entity.Get();
			const bool bPropeller = Entity->GetEntityType() == aerotelic::project_yaml::EntityType::Propeller;
			const double Rpm = bPropeller ? KeptRpm.FindRef(C->Label) : 0.0;

			// A propeller that is actually turning is split: one blade on a
			// rotor frame in hub-local coordinates, which DUST replicates
			// and spins, plus the spinner left standing on the global frame
			// (it is a body of revolution about the same axis, so spinning
			// it would change nothing). A propeller at rest is just bodies,
			// exactly like everything else.
			if (bPropeller && Rpm != 0.0)
			{
				std::vector<aerotelic::NamedMesh> Local;
				try
				{
					if (Entity->GetModel() != nullptr) Local = Entity->GetModel()->BuildMeshes(Deflection);
				}
				catch (const std::exception& E)
				{
					Failures.Add(C->Label + TEXT(": ") + UTF8_TO_TCHAR(E.what()));
					continue;
				}

				const aerotelic::EntityTransform T = Entity->GetEntityTransform();
				FRot3 R = AxisAngleToMatrix(T.ax, T.ay, T.az, T.angle_deg);
				aerotelic::Vec3 Origin{T.tx, T.ty, T.tz};
				if (C->Operand.bMirror)
				{
					// Reflection about world y = 0, folded as M*R = (M*R*M)*M.
					R = MirrorConjugate(R);
					Origin.y = -Origin.y;
				}

				FRotor Rot;
				Rot.Label = C->Label;
				Rot.Entity = C->Operand.Entity;
				Rot.bMirror = C->Operand.bMirror;
				Rot.Rpm = Rpm;
				Rot.Tag = FString::Printf(TEXT("%s_hub"), *C->Label.Replace(TEXT(" "), TEXT("_")).Replace(TEXT("/"), TEXT("_")));
				bool bNegativeSpin = false;
				if (const aerotelic::PropellerModel* P = Entity->GetModelAs<aerotelic::PropellerModel>())
				{
					Rot.Radius = P->radius;
					bNegativeSpin = P->spin_direction == aerotelic::SpinDirection::Negative;
				}

				int32 Blades = 0;
				bool bBladeWritten = false;
				for (aerotelic::NamedMesh& M : Local)
				{
					if (M.mesh.IsEmpty()) continue;
					const FString Role = UTF8_TO_TCHAR(M.name.c_str());
					if (Role.StartsWith(TEXT("Blade"))) ++Blades;
				}
				for (aerotelic::NamedMesh& M : Local)
				{
					if (M.mesh.IsEmpty()) continue;
					const FString Role = UTF8_TO_TCHAR(M.name.c_str());
					aerotelic::DustComponent Comp;
					Comp.name = TCHAR_TO_UTF8(*(C->Label + TEXT("/") + Role));
					if (Role == TEXT("Blade 1"))
					{
						// Hub-local, as the frame carries the placement. The
						// leftover reflection of a mirrored propeller is
						// folded in here.
						Comp.mesh = std::move(M.mesh);
						if (C->Operand.bMirror) Comp.mesh.Mirror({0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
						Comp.ref_tag = TCHAR_TO_UTF8(*Rot.Tag);
						Components.push_back(std::move(Comp));
						bBladeWritten = true;
					}
					else if (Role.StartsWith(TEXT("Blade")))
					{
						continue;   // DUST makes the rest from Blade 1
					}
					else
					{
						// The spinner (or anything else): placed into world
						// coordinates here, since GetPlacedMeshes is not on
						// this path, and left on the global frame.
						//
						// The mirrored placement is M*(R*x + t) = R''*(M*x)
						// + M*t with R'' = M*R*M, and R/Origin already hold
						// R'' and M*t -- so the leftover reflection goes on
						// FIRST, in the body's own coordinates. Mirror()
						// also reverses the winding, which is what keeps the
						// body facing outwards.
						aerotelic::TriMesh Placed = std::move(M.mesh);
						if (C->Operand.bMirror) Placed.Mirror({0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
						for (aerotelic::Vec3& v : Placed.vertices) v = R.Apply(v) + Origin;
						Comp.mesh = std::move(Placed);
						Components.push_back(std::move(Comp));
					}
				}

				if (!bBladeWritten)
				{
					Failures.Add(C->Label + TEXT(": no blade to spin"));
					continue;
				}

				aerotelic::DustFrame Frame;
				Frame.tag = TCHAR_TO_UTF8(*Rot.Tag);
				Frame.parent = "0";
				Frame.origin = Origin;
				Frame.x_axis = R.Column(0);
				Frame.y_axis = R.Column(1);
				Frame.z_axis = R.Column(2);
				Frame.n_blades = FMath::Max(1, Blades);
				// Aerotelic spins its propellers about the model's local +x.
				Frame.rot_axis = aerotelic::Vec3{1.0, 0.0, 0.0};
				// NEGATIVE, and that is not a typo. A positive RPM has to
				// drive the blade LEADING EDGE first, and the blade's own
				// geometry decides which way that is: PropellerModel turns
				// the blade -90 degrees about y, which carries the wing's
				// chord (leading edge at low x, +x being aft) onto local +z.
				// So the leading edge sits at low z, the trailing edge at
				// high z, and the blade must travel towards -z. A blade at
				// radius r along +y moving under rate w about +x has
				// v = (w,0,0) x (0,r,0) = (0,0,+w*r), which is towards -z
				// only for w < 0. Written positive, every propeller in the
				// case turned trailing-edge first.
				Frame.rot_rate = -Rpm * 2.0 * PI / 60.0;
				// ... for a POSITIVE-spin propeller. A negative-spin one is
				// that blade reflected in z (PropellerSpec::spin_direction),
				// so its leading edge sits at high z instead of low and it
				// has to travel towards +z: the sign flips back.
				if (bNegativeSpin) Frame.rot_rate = -Frame.rot_rate;
				// A mirrored blade is reflected about y, so it spans -y with
				// its edges still where they were in z: v = (0,0,-w*r), and
				// the sign flips back. This is also what the reflection does
				// to the motion on its own -- M*R_x(t)*M = R_x(-t).
				if (C->Operand.bMirror) Frame.rot_rate = -Frame.rot_rate;
				Frames.push_back(Frame);

				Rot.BladeCount = Frame.n_blades;
				Rotors.Add(Rot);
				continue;
			}

			std::vector<aerotelic::NamedMesh> Meshes;
			try
			{
				Meshes = Entity->GetPlacedMeshes(C->Operand.bMirror, Deflection);
			}
			catch (const std::exception& E)
			{
				Failures.Add(C->Label + TEXT(": ") + UTF8_TO_TCHAR(E.what()));
				continue;
			}
			bool bAny = false;
			for (aerotelic::NamedMesh& M : Meshes)
			{
				if (M.mesh.IsEmpty()) continue;
				aerotelic::DustComponent Comp;
				const FString Label = Meshes.size() > 1 ? C->Label + TEXT("/") + UTF8_TO_TCHAR(M.name.c_str()) : C->Label;
				Comp.name = TCHAR_TO_UTF8(*Label);
				Comp.mesh = std::move(M.mesh);
				Components.push_back(std::move(Comp));
				bAny = true;
			}
			if (!bAny)
			{
				Failures.Add(C->Label + TEXT(": produced no geometry"));
			}

			// A propeller still at rest is listed so a speed can be given.
			if (bPropeller)
			{
				FRotor Rot;
				Rot.Label = C->Label;
				Rot.Entity = C->Operand.Entity;
				Rot.bMirror = C->Operand.bMirror;
				Rot.Rpm = 0.0;
				Rotors.Add(Rot);
			}
		}
		Rederive();
		if (RefreshRotors) RefreshRotors();
		if (bDerived)
		{
			Status(FText::FromString(FString::Printf(TEXT("%s panels across %s bodies; %.1f MB dense panel system"),
				*Thousands((int64)Derived.panels), *Thousands((int64)Derived.components), Derived.linear_system_mb)));
		}
		else
		{
			Status(LOCTEXT("SelectBody", "Select the geometry to analyse."));
		}
	};

	auto Browse = [&]()
	{
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr)
		{
			return;
		}
		FString Picked;
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Window);
		if (Desktop->OpenDirectoryDialog(Handle, TEXT("DUST Case Folder"), DirBox->GetText().ToString(), Picked))
		{
			DirBox->SetText(FText::FromString(FPaths::ConvertRelativePathToFull(Picked)));
		}
	};

	auto Write = [&]()
	{
		if (Components.empty())
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("SelectBodyWarn", "Select the geometry to analyse."), LOCTEXT("Title", "DUST (Vortex Particle Method)"));
			return;
		}
		const FString Dir = DirBox->GetText().ToString().TrimStartAndEnd();
		if (Dir.IsEmpty())
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NoDir", "Name the folder the case is to be written to."), LOCTEXT("Title", "DUST (Vortex Particle Method)"));
			return;
		}
		FString Name = NameBox->GetText().ToString().TrimStartAndEnd();
		if (Name.IsEmpty()) Name = TEXT("dust");
		Opt.basename = TCHAR_TO_UTF8(*Name);
		Opt.write_postprocessing = PostCheck->IsChecked();

		Status(LOCTEXT("Writing", "Writing the DUST case..."));
		// Everything DUST sees is SI: the panels scaled to metres, the rotor
		// frames' origins with them, and every dimensional option converted.
		// Times are seconds already; the dimensionless ratios pass through.
		// The derived step and run length come out the same either way (a
		// length over a speed), so what the Time tab showed still holds.
		const double L = Units.LengthToSI();
		std::vector<aerotelic::DustComponent> SiComponents = Components;
		for (aerotelic::DustComponent& C : SiComponents)
		{
			for (aerotelic::Vec3& V : C.mesh.vertices) V *= L;
		}
		std::vector<aerotelic::DustFrame> SiFrames = Frames;
		for (aerotelic::DustFrame& F : SiFrames)
		{
			F.origin *= L;
			F.hub_offset *= L;
		}
		aerotelic::DustCaseOptions SiOpt = Opt;
		SiOpt.speed *= Units.SpeedToSI();
		SiOpt.rho_inf *= Units.DensityToSI();
		SiOpt.p_inf *= Units.PressureToSI();
		SiOpt.a_inf *= Units.SpeedToSI();
		SiOpt.mu_inf *= Units.ViscosityToSI();
		SiOpt.vortex_rad *= L;
		SiOpt.tol_se_wing *= L;

		aerotelic::DustCaseResult R;
		std::string Error;
		if (!aerotelic::WriteDustCase(SiComponents, SiFrames, TCHAR_TO_UTF8(*Dir), SiOpt, R, Error))
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(UTF8_TO_TCHAR(Error.c_str())), LOCTEXT("Failed", "DUST Case Not Written"));
			return;
		}
		Derived = R;
		LblWritten->SetText(FText::FromString(FString::Printf(TEXT("%s files to %s"), *Thousands((int64)R.files.size()), *Dir)));
		Status(FText::FromString(FString::Printf(
			TEXT("DUST case written: %s panels, %s steps to t = %.4g -> %s   (run: dust_pre dust_pre.in, dust dust.in, dust_post dust_post.in)"),
			*Thousands((int64)R.panels), *Thousands((int64)R.timesteps), R.tend, *Dir)));

		FString Note;
		if (R.coincident_panels > 0)
		{
			Note += FString::Printf(TEXT("%s panels of \"%s\" and \"%s\" lie on top of each other. DUST's panel matrix will be singular and the run will diverge immediately -- fuse those bodies with a boolean operation and select the result instead.\n\n"),
				*Thousands((int64)R.coincident_panels), UTF8_TO_TCHAR(R.coincident_a.c_str()), UTF8_TO_TCHAR(R.coincident_b.c_str()));
		}
		if (Failures.Num() > 0)
		{
			Note += TEXT("Left out of the case:\n") + FString::Join(Failures, TEXT("\n")) + TEXT("\n\n");
		}
		if (!Note.IsEmpty())
		{
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Note + TEXT("The case was written anyway; see the Help tab.")), LOCTEXT("Title", "DUST (Vortex Particle Method)"));
		}
	};

	// A spin box that re-derives (cheap), and one that re-panels (not).
	auto Spin = [&](double& Var, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Var]() { return Var; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([&Var, &Rederive](double V, ETextCommit::Type) { Var = V; Rederive(); });
	};
	auto SpinInt = [&](int& Var, double Min, double Max, double Delta)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Var]() { return static_cast<double>(Var); })
			.MinValue(Min).MaxValue(Max).Delta(Delta)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([&Var, &Rederive](double V, ETextCommit::Type) { Var = static_cast<int>(FMath::RoundToDouble(V)); Rederive(); });
	};
	auto SpinMesh = [&](double& Var, double Min, double Max, double Delta)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Var]() { return Var; })
			.MinValue(Min).MaxValue(Max).Delta(Delta)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([&Var, &Rebuild](double V, ETextCommit::Type) { if (V != Var) { Var = V; Rebuild(); } });
	};
	auto Check = [&](TSharedPtr<SCheckBox>& Out, bool& Var, const FText& Label)
	{
		return SAssignNew(Out, SCheckBox)
			.IsChecked(Var ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
			.OnCheckStateChanged_Lambda([&Var, &Rederive](ECheckBoxState S) { Var = S == ECheckBoxState::Checked; Rederive(); })
			[ SNew(STextBlock).Text(Label) ];
	};
	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center)
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};

	// ---------------------------------------------------------- geometry

	TSharedRef<SWidget> GeometryTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Bodies", "Geometry for the analysis"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).ListHeight(190.0f)
					.OnChanged(FSimpleDelegate::CreateLambda([&Rebuild]() { Rebuild(); }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("BodiesHint",
						"Each body becomes a DUST component of surface panels. Bodies must not lie on top of one another: two surfaces in the same place make the panel matrix singular and the run diverges on the first step. A wing AND its mirror do exactly that at the root -- fuse them with a boolean operation and select the result."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Panelling", "Panelling"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Deflection", "Surface deflection:"), SpinMesh(Deflection, 1e-9, 1e9, 0.01)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("PanelStat", "Panels:"), Stat(LblPanels)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("SystemStat", "Panel system:"), Stat(LblSystem)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("OverlapStat", "Overlap check:"), Stat(LblOverlap)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("PanelHint",
						"DUST solves a DENSE system over the panels, so cost grows as the square of the panel count: a few thousand panels is comfortable, tens of thousands is not. Coarsen here rather than regretting it during the run."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeGroup(LOCTEXT("TeGroup", "Trailing edge"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("InnerTe", "Inner-product threshold:"), Spin(Opt.inner_product_te, -1.0, 0.0, 0.05, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("TolSew", "Sewing tolerance (0 = auto):"), Spin(Opt.tol_se_wing, 0.0, 1e9, 1e-5, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("TeHint",
						"An edge sheds a wake where the panels meeting at it face away from each other by more than this. A sharp trailing edge is found easily; a blunt body sheds nothing and only adds thickness. dust_pre prints the count per body -- if a lifting surface reports zero, raise the threshold towards 0."))
				])
		];

	// -------------------------------------------------------------- flow

	TSharedRef<SWidget> FlowTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("FreeStream", "Free stream"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Speed", "Speed:"), Spin(Opt.speed, 1e-9, 1e9, 5.0, SpeedSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Alpha", "Angle of attack:"), Spin(Opt.alpha_deg, -90.0, 90.0, 0.5, FText::FromString(TEXT(" deg")))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Beta", "Sideslip:"), Spin(Opt.beta_deg, -90.0, 90.0, 0.5, FText::FromString(TEXT(" deg")))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Rho", "Air density:"), Spin(Opt.rho_inf, 1e-12, 1e9, 0.1, DensitySuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Sound", "Speed of sound:"), Spin(Opt.a_inf, 1e-9, 1e9, 10.0, SpeedSuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Mu", "Dynamic viscosity:"), Spin(Opt.mu_inf, 0.0, 1e9, 1e-6, ViscositySuffix)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("FlowHint",
						"The flow runs along +x, tilted towards +z by the angle of attack and towards +y by the sideslip. Viscosity only matters with diffusion on. Everything here is in the project's units (Geometry > Units); the case is written in SI."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeGroup(LOCTEXT("TimeGroup", "Time"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Dt", "Time step (0 = auto):"), Spin(Opt.dt, 0.0, 1e9, 1e-3, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Tend", "End time (0 = auto):"), Spin(Opt.tend, 0.0, 1e9, 0.1, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("StepsPer", "Steps per model length:"), Spin(Opt.steps_per_length, 1.0, 1e5, 5.0, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Crossings", "Flow crossings to run:"), Spin(Opt.crossings, 0.1, 1e4, 1.0, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("DtOut", "Output interval (0 = every step):"), Spin(Opt.dt_out, 0.0, 1e9, 1e-3, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("StepStat", "Step:"), Stat(LblStep)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("RunStat", "Run:"), Stat(LblRun)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("TimeHint",
						"Left at zero, the step and the end time come from the model's extent along the flow: the step is the time to cross it divided by the steps-per-length figure, and the run lasts that many crossings. The wake needs several crossings before the loads settle."))
				])
		];

	// -------------------------------------------------------------- wake

	TSharedRef<SWidget> WakeTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Particles", "Wake particles"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("WakePanels", "Rows of panel wake:"), SpinInt(Opt.n_wake_panels, 1.0, 1000.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Budget", "Particle budget:"), SpinInt(Opt.n_wake_particles, 1000.0, 1e8, 50000.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Radius", "Particle core radius (0 = auto):"), Spin(Opt.vortex_rad, 0.0, 1e9, 0.01, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Margin", "Box margin, in model sizes:"), Spin(Opt.box_margin, 0.0, 1000.0, 0.5, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Wake", "Box length downstream:"), Spin(Opt.box_wake, 0.0, 1000.0, 1.0, FText::GetEmpty())) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("RadiusStat", "Core radius:"), Stat(LblRadius)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("BoxStat", "Particle box:"), Stat(LblBox)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ParticleHint",
						"One row of panel wake stays attached at the trailing edge; everything past it is carried as free vortex particles. A particle that leaves the box is dropped, so the box and the budget together bound the cost of a long run."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Models", "Models"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Check(StretchCheck, Opt.vortstretch, LOCTEXT("Stretch", "Vortex stretching")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Check(DiffusionCheck, Opt.diffusion, LOCTEXT("Diffusion", "Vorticity diffusion")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Check(PenetrationCheck, Opt.penetration_avoidance, LOCTEXT("Penetration", "Penetration avoidance")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Check(FmmCheck, Opt.fmm, LOCTEXT("Fmm", "Fast multipole method")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ModelHint",
						"Stretching and diffusion are what make this a vortex particle method rather than a cloud of fixed blobs. Without the fast multipole method the particle-particle cost is quadratic in the particle count."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeGroup(LOCTEXT("Octree", "Octree (serves the fast multipole method)"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Levels", "Levels:"), SpinInt(Opt.n_octree_levels, 1.0, 16.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("MinPart", "Minimum particles per box:"), SpinInt(Opt.min_octree_part, 1.0, 1000.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Degree", "Multipole degree:"), SpinInt(Opt.multipole_degree, 1.0, 10.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("OctreeStat", "Octree:"), Stat(LblOctree)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("OctreeHint",
						"The octree is sized to cover the particle box above, so it follows from it rather than being set here. More levels means smaller leaves: faster for many particles, wasteful for few."))
				])
		];

	// ------------------------------------------------------------ output

	TSharedRef<SWidget> OutputTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("CaseGroup", "Case folder"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
					[ SAssignNew(DirBox, SEditableTextBox).Text(FText::FromString(DefaultCaseDir())) ]
					+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
					[ Button(LOCTEXT("Browse", "Browse..."), [&Browse]() { Browse(); }) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[ FormRow(LOCTEXT("Basename", "Result basename:"), SAssignNew(NameBox, SEditableTextBox).Text(FText::FromString(TEXT("dust")))) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					SAssignNew(PostCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
					[ SNew(STextBlock).Text(LOCTEXT("WritePost", "Write dust_post.in (visualisation and load tables)")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ FormRow(LOCTEXT("WrittenStat", "Written:"), Stat(LblWritten)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("OutputHint",
						"The folder is created if it does not exist, along with components/, geometry/, Output/ and Postprocessing/. Existing files of the same names are overwritten; results already in Output/ are not touched, so clear it yourself between runs."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeGroup(LOCTEXT("RunGroup", "Running it"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					MakeInfoLabel(LOCTEXT("RunHint",
						"From a command prompt in the case folder, with DUST's bin folder on PATH:\n"
						"    dust_pre  dust_pre.in\n"
						"    dust      dust.in\n"
						"    dust_post dust_post.in\n"
						"The case folder's README.txt repeats this. The Help tab explains what each step does and how to read the result."))
				])
		];

	// ------------------------------------------------------------ rotors

	// Rebuilt whenever the selection changes: one row per propeller among
	// the chosen bodies.
	auto RefreshRotorRows = [&]()
	{
		RotorRows->ClearChildren();
		if (Rotors.Num() == 0)
		{
			LblRotorNote->SetText(LOCTEXT("NoRotors",
				"No propellers among the chosen bodies. Select one on the Geometry tab and it appears here."));
			return;
		}
		LblRotorNote->SetText(LOCTEXT("SomeRotors",
			"A speed of zero leaves the propeller standing still -- its blades are written where they sit, like any other body."));
		// Rows address Rotors BY INDEX, never by pointer: Rebuild() clears
		// and refills the array, which would leave captured pointers
		// dangling.
		for (int32 i = 0; i < Rotors.Num(); ++i)
		{
			RotorRows->AddSlot().AutoHeight().Padding(0, 3)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
				[ SNew(SBox).WidthOverride(210) [ SNew(STextBlock).Text(FText::FromString(Rotors[i].Label)) ] ]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
				[
					SNew(SBox).WidthOverride(130)
					[
						SNew(SAerotelicNumericEntry)
						.Value_Lambda([&Rotors, i]() { return Rotors.IsValidIndex(i) ? Rotors[i].Rpm : 0.0; })
						.MinValue(-1e7).MaxValue(1e7).Delta(100.0).Suffix(LOCTEXT("Rpm", " rev/min"))
						.LiveUpdate(false)
						.OnValueCommitted_Lambda([&Rotors, i, &Rebuild](double V, ETextCommit::Type)
						{
							if (!Rotors.IsValidIndex(i) || V == Rotors[i].Rpm) return;
							Rotors[i].Rpm = V;
							// A speed change moves a propeller between the
							// static and the rotor path, so the geometry has
							// to be gathered again, not merely re-derived.
							Rebuild();
						})
					]
				]
				+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
				[
					SNew(STextBlock).AutoWrapText(true)
					.ColorAndOpacity(FSlateColor(FLinearColor(0.75f, 0.75f, 0.75f)))
					.Text_Lambda([&Rotors, i, &Opt]()
					{
						if (!Rotors.IsValidIndex(i)) return FText::GetEmpty();
						const FRotor& Rot = Rotors[i];
						if (Rot.Rpm == 0.0) return LOCTEXT("Static", "at rest");
						const double Omega = Rot.Rpm * 2.0 * PI / 60.0;
						const double Tip = FMath::Abs(Omega) * Rot.Radius;
						// Advance ratio J = V / (n D), n in rev/s.
						const double J = Rot.Radius > 0.0
							? Opt.speed / (FMath::Abs(Rot.Rpm / 60.0) * 2.0 * Rot.Radius) : 0.0;
						const double Mach = Opt.a_inf > 0.0 ? Tip / Opt.a_inf : 0.0;
						// DUST's panel method is incompressible. A transonic
						// tip does not merely lose accuracy -- it was found
						// to fill the whole solution with NaN, silently,
						// with the solver reporting no error at all.
						const TCHAR* Verdict = Mach >= 0.8 ? TEXT("  -- TOO FAST: the solution will come out NaN")
							: Mach >= 0.6 ? TEXT("  -- high, accuracy is suspect") : TEXT("");
						return FText::FromString(FString::Printf(
							TEXT("%d blades, %.4g rad/s, tip speed %.4g, tip Mach %.2f%s, J = %.3f%s"),
							Rot.BladeCount, Omega, Tip, Mach, Verdict, J,
							Rot.bMirror ? TEXT("  (mirrored: opposite hand)") : TEXT("")));
					})
				]
			];
		}
	};
	RefreshRotors = RefreshRotorRows;

	TSharedRef<SWidget> RotorTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("RotorGroup", "Propeller rotational speed"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(RotorRows, SVerticalBox) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ Stat(LblRotorNote) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("RotorHint",
						"A turning propeller is written as DUST's own rotor: ONE blade is handed over in hub-local coordinates and DUST replicates it around the axis and spins it, which is what its post-processing is built around. The spinner stays still -- it is a body of revolution about the same axis, so turning it would change nothing. Positive is the sense Aerotelic builds the propeller in, about its local +x; a mirrored propeller turns the opposite way and is written with a negative rate."))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("RotorNotSaved",
						"Rotational speed is an analysis setting, not part of the propeller's design: it is deliberately NOT stored on the model or in the project file, so it is forgotten when this dialog closes. The time step may want checking against it -- a blade should not sweep more than a few degrees per step."))
				])
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("GeometryTab", "Geometry"), GeometryTab });
	Tabs.Add({ LOCTEXT("FlowTab", "Flow"), FlowTab });
	Tabs.Add({ LOCTEXT("RotorsTab", "Rotors"), RotorTab });
	Tabs.Add({ LOCTEXT("WakeTab", "Wake"), WakeTab });
	Tabs.Add({ LOCTEXT("OutputTab", "Output"), OutputTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SNew(SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("WriteBtn", "Write Case"), [&Write]() { Write(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);

	Status(LOCTEXT("SelectBody", "Select the geometry to analyse."));
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
}

#undef LOCTEXT_NAMESPACE
