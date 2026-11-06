#include "AerotelicDocumentTestCommandlet.h"

#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicLatticeModel.h"
#include "Entity/AerotelicMeshLib.h"
#include "Entity/AerotelicPotentialFlow.h"
#include "Entity/AerotelicMassProperties.h"
#include "Entity/AerotelicStructuralFea.h"
#include "Entity/AerotelicFeaPreprocess.h"

#include "aerotelic/materials.hpp"
#include "aerotelic/primitive_model.hpp"
#include "aerotelic/wing_model.hpp"
#include "aerotelic/wing_panels.hpp"
#include "aerotelic/panel_loads.hpp"
#include "aerotelic/colormaps.hpp"
#include "Entity/AerotelicMeshing.h"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicDocumentTest, Log, All);

namespace
{
	int32 g_pass = 0, g_fail = 0;

	void Check(bool bCond, const FString& Msg)
	{
		if (bCond)
		{
			++g_pass;
			return;
		}
		++g_fail;
		UE_LOG(LogAerotelicDocumentTest, Error, TEXT("FAIL: %s"), *Msg);
	}

	// Moves an entity the way the Transform dialog does. SetActorLocation()
	// would move the actor without touching its placement, which is what
	// gets saved and copied (AerotelicEntityActor::GetPlacement).
	void MoveTo(AAerotelicEntityActor* Entity, const FVector& Where)
	{
		if (Entity == nullptr) return;
		aerotelic::Placement P = Entity->GetPlacement();
		P.translation = {Where.X, Where.Y, Where.Z};
		Entity->ApplyPlacement(P);
	}

	// RefreshProxy() -- the only thing that normally calls BuildSolid() --
	// short-circuits under IsRunningCommandlet() (see AerotelicEntityActor.cpp;
	// that guard exists to dodge a real StaticMeshComponent hang under -run=).
	// That means this headless test would otherwise never exercise
	// BuildSolid() at all and could miss exactly the kind of bug a freshly
	// "Add Entity"-created model has in the interactive Editor (a wing with
	// no sections yet, confirmed the hard way). Call it directly here.
	void CheckBuildsSolid(AAerotelicEntityActor* Entity, const FString& Label)
	{
		if (Entity == nullptr)
		{
			Check(false, Label + ": entity is null");
			return;
		}
		try
		{
			Entity->GetModel()->BuildSolid();
			Check(true, Label + ": BuildSolid");
		}
		catch (const std::exception& e)
		{
			Check(false, FString::Printf(TEXT("%s: BuildSolid threw: %s"), *Label, UTF8_TO_TCHAR(e.what())));
		}
	}
}

int32 UAerotelicDocumentTestCommandlet::Main(const FString& Params)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	Check(Doc != nullptr, "document subsystem is available");
	if (Doc == nullptr)
	{
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
		return 1;
	}
	Check(Doc->GetEntities().Num() == 0, "scene starts empty");

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: AddWing"));
	AAerotelicEntityActor* Wing = Doc->AddWing(TEXT("standard"));
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: AddFuselage"));
	AAerotelicEntityActor* Fuselage = Doc->AddFuselage();
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: AddPrimitive"));
	AAerotelicEntityActor* Primitive = Doc->AddPrimitive(TEXT("Sphere"));
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: adds done"));
	Check(Wing != nullptr && Fuselage != nullptr && Primitive != nullptr, "all three entities were added");
	Check(Doc->GetEntities().Num() == 3, "scene now has 3 entities");

	AAerotelicEntityActor* Nacelle = Doc->AddNacelle();
	AAerotelicEntityActor* LoftedNacelle = Doc->AddLoftedNacelle();
	AAerotelicEntityActor* Propeller = Doc->AddPropeller();
	Check(Nacelle != nullptr && LoftedNacelle != nullptr && Propeller != nullptr, "nacelle/lofted-nacelle/propeller were added");

	CheckBuildsSolid(Wing, TEXT("Wing"));
	CheckBuildsSolid(Fuselage, TEXT("Fuselage"));
	CheckBuildsSolid(Primitive, TEXT("Primitive"));

	// Analysis -> High Order Panel, minus the dialog: the scene's wing
	// paneled, placed as it sits, and solved at 4 degrees -- it must lift.
	if (Wing != nullptr)
	{
		try
		{
			aerotelic::WingModel* W = Wing->GetModelAs<aerotelic::WingModel>();
			const double Alpha = 4.0 * 3.14159265358979323846 / 180.0;
			const aerotelic::Vec3 Stream{std::cos(Alpha), 0.0, std::sin(Alpha)};
			aerotelic::panel::WingPaneling Pan;
			Pan.chordwise = 8;
			Pan.spanwise = 6;
			std::vector<aerotelic::panel::Network> Nets = aerotelic::panel::WingNetworks(*W, Pan, Stream);
			const FTransform Transform = Wing->GetActorTransform();
			for (aerotelic::panel::Network& N : Nets)
			{
				aerotelic::TriMesh M;
				M.vertices = N.corners;
				AerotelicMeshing::PlaceMesh(M, Transform);
				N.corners = M.vertices;
			}
			aerotelic::panel::Options Opt;
			Opt.symmetry_y = 0.0;
			Opt.reference_area = W->planform().PlanformArea();
			aerotelic::panel::Solution Sol = aerotelic::panel::Solve(Nets, Stream, Opt);
			const double CL = Sol.force_coefficient.z * std::cos(Alpha) - Sol.force_coefficient.x * std::sin(Alpha);
			Check(Sol.unknowns > 0 && Sol.lift_from_wake.has_value(), "panel method: the scene's wing panels into a body and a wake");
			Check(*Sol.lift_from_wake > 0.1 && *Sol.lift_from_wake < 1.0, FString::Printf(TEXT("panel method: the wing lifts at 4 degrees (C_L from the wake %.4f)"), *Sol.lift_from_wake));
			Check(FMath::Abs(CL / *Sol.lift_from_wake - 1.0) < 0.25, FString::Printf(TEXT("panel method: pressures and wake agree (%.4f vs %.4f)"), CL, *Sol.lift_from_wake));

			// The Span Loading tab, minus the dialog: the strips of the placed
			// wing along its placed span axis sum to the network's own force,
			// and the elliptic distribution with that total integrates back
			// to it.
			{
				aerotelic::panel::StripOptions SO;
				const FVector Axis = Transform.TransformVectorNoScale(FVector(0, 1, 0)).GetSafeNormal();
				SO.span_axis = { Axis.X, Axis.Y, Axis.Z };
				SO.strips_are_columns = true;
				std::vector<aerotelic::panel::StripLoad> Strips = aerotelic::panel::StripLoads(Nets[0], Sol.networks[0], SO);
				aerotelic::Vec3 Sum;
				double SMin = 1e300, SMax = -1e300;
				for (const aerotelic::panel::StripLoad& St : Strips) { Sum += St.force; SMin = FMath::Min(SMin, St.s_min); SMax = FMath::Max(SMax, St.s_max); }
				const aerotelic::Vec3 Want = Sol.networks[0].force_coefficient * Opt.reference_area; // the image doubled x and z
				Check(Strips.size() == static_cast<size_t>(Sol.networks[0].panel_cols) && FMath::Abs(Sum.z - 0.5 * Want.z) < 1e-9 * (1.0 + FMath::Abs(Want.z)),
					FString::Printf(TEXT("span loading: %d strips sum to the wing's force (%.5f vs %.5f)"), static_cast<int32>(Strips.size()), Sum.z, 0.5 * Want.z));
				const aerotelic::Vec3 LiftDir{-std::sin(Alpha), 0.0, std::cos(Alpha)};
				const double Total = 2.0 * aerotelic::Dot(Sum, LiftDir);
				const double Half = FMath::Max(SMax, -SMin);
				double Integral = 0.0;
				for (int32 i = 0; i < 2000; ++i) { const double s = -Half + (i + 0.5) * 2.0 * Half / 2000; Integral += aerotelic::panel::EllipticLoad(Total, 0.0, Half, s) * 2.0 * Half / 2000; }
				Check(FMath::Abs(Integral / Total - 1.0) < 1e-3, FString::Printf(TEXT("span loading: the elliptic distribution integrates to the lift (%.5f vs %.5f)"), Integral, Total));
				Check(aerotelic::Colormaps().size() >= 15 && aerotelic::FindColormap("Turbo") != nullptr, "surface pressure: the colour map catalogue is there");
			}

			// With the fuselage as a source network (its loft cut at the
			// symmetry plane, as the dialog does for a body on the centreline):
			// one system, the body impermeable, the wing still lifting.
			if (Fuselage != nullptr && Fuselage->GetModel() != nullptr)
			{
				aerotelic::panel::BodyPaneling BP;
				BP.along = 10;
				BP.around = 8;
				BP.half_plane_y = 0.0;
				std::vector<aerotelic::panel::Network> Body = aerotelic::panel::BodyNetworks(*Fuselage->GetModel(), "Fuselage", BP, Stream);
				const FTransform FT = Fuselage->GetActorTransform();
				for (aerotelic::panel::Network& N : Body)
				{
					aerotelic::TriMesh M;
					M.vertices = N.corners;
					AerotelicMeshing::PlaceMesh(M, FT);
					N.corners = M.vertices;
					Nets.push_back(N);
				}
				aerotelic::panel::Solution Both = aerotelic::panel::Solve(Nets, Stream, Opt);
				double Vn = 0.0;
				for (const aerotelic::panel::PanelResult& P : Both.networks.back().panels) Vn = FMath::Max(Vn, FMath::Abs(aerotelic::Dot(P.v_upper, P.normal)));
				Check(Body.size() == 1 && Both.networks.size() == 3, "panel method: the fuselage panels into one source network beside the wing");
				Check(Vn < 1e-6, FString::Printf(TEXT("panel method: the fuselage is impermeable (normal velocity %.2e)"), Vn));
				Check(Both.lift_from_wake.has_value() && *Both.lift_from_wake > 0.1, FString::Printf(TEXT("panel method: the wing still lifts beside the fuselage (C_L %.4f)"), Both.lift_from_wake.value_or(0.0)));
			}
		}
		catch (const std::exception& e)
		{
			Check(false, FString::Printf(TEXT("panel method threw: %s"), UTF8_TO_TCHAR(e.what())));
		}
	}
	CheckBuildsSolid(Nacelle, TEXT("Nacelle"));
	CheckBuildsSolid(LoftedNacelle, TEXT("LoftedNacelle"));
	CheckBuildsSolid(Propeller, TEXT("Propeller"));

	for (AAerotelicEntityActor* E : { Nacelle, LoftedNacelle, Propeller })
	{
		if (E != nullptr) Doc->RemoveEntity(E);
	}

	MoveTo(Wing, FVector(10.0, 20.0, 30.0));

	// What the Transform dialog needs of a placement, which reading one
	// back off the actor's quaternion could not give it.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: placement"));
	if (Wing != nullptr)
	{
		const aerotelic::Placement Saved = Wing->GetPlacement();
		aerotelic::Placement P = Saved;

		// An axis typed while the angle is still zero has to survive: the
		// transform it derives is the identity, which has no axis at all.
		P.rotation_axis = {0.0, 1.0, 0.0};
		P.rotation_deg = 0.0;
		Wing->ApplyPlacement(P);
		const aerotelic::Placement AtZero = Wing->GetPlacement();
		Check(AtZero.rotation_axis[0] == 0.0 && AtZero.rotation_axis[1] == 1.0 && AtZero.rotation_axis[2] == 0.0,
			"placement: an axis set while the angle is zero is kept");

		// ...and an axis is kept as typed, not rescaled to unit length.
		P.rotation_axis = {0.0, 2.0, 1.0};
		P.rotation_deg = 45.0;
		Wing->ApplyPlacement(P);
		const aerotelic::Placement Unnormalised = Wing->GetPlacement();
		Check(Unnormalised.rotation_axis[1] == 2.0 && Unnormalised.rotation_axis[2] == 1.0,
			"placement: the axis is not normalised behind the user's back");
		Check(Unnormalised.rotation_deg == 45.0, "placement: the angle is kept exactly");

		// The Euler tab: the triple is what was authored, and the axis and
		// angle the geometry uses are derived from it. ZYX with only the
		// last angle set is a 90-degree roll about X.
		P.euler.used = true;
		P.euler.intrinsic = true;
		P.euler.order = aerotelic::EulerOrder::ZYX;
		P.euler.angles_deg = {0.0, 0.0, 90.0};
		Wing->ApplyPlacement(P);
		const aerotelic::EntityTransform T = Wing->GetEntityTransform();
		Check(FMath::IsNearlyEqual(T.angle_deg, 90.0, 1e-9) && FMath::IsNearlyEqual(T.ax, 1.0, 1e-9)
			&& FMath::IsNearlyZero(T.ay) && FMath::IsNearlyZero(T.az),
			FString::Printf(TEXT("placement: an Euler triple drives the axis+angle the geometry uses (got %f about %f,%f,%f)"),
				T.angle_deg, T.ax, T.ay, T.az));
		Check(Wing->GetPlacement().euler.used, "placement: the triple itself is kept, so the Euler tab reopens on it");

		Wing->ApplyPlacement(Saved);
		Check(Wing->GetPlacement().rotation_deg == Saved.rotation_deg && !Wing->GetPlacement().euler.used,
			"placement: restored for the rest of the run");
	}

	// Analysis > Mass Properties over real entities: a density weighs the
	// enclosed volume, an asserted mass is spread over the surface and the
	// mirrored half carries it again, an entity with no setting is left out
	// and named, and the assembly is the parallel-axis sum. The settings
	// stay on the entities for the save/reopen round trip checked below.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: mass properties"));
	if (Wing != nullptr && Fuselage != nullptr && Primitive != nullptr)
	{
		aerotelic::MassSpec ByDensity;
		ByDensity.mode = aerotelic::MassMode::Material;
		ByDensity.material.name = "Custom";
		ByDensity.material.density = 2.0;   // SI; the project is metres and kilograms here
		Primitive->SetMassSpec(ByDensity);
		aerotelic::MassSpec Asserted;
		Asserted.mode = aerotelic::MassMode::Asserted;
		Asserted.mass = 150.0;
		Fuselage->SetMassSpec(Asserted);
		// The wing deliberately keeps no setting.

		FAerotelicBooleanOperand P, F, W, FM;
		P.Entity = Primitive;
		F.Entity = Fuselage;
		W.Entity = Wing;
		FM.Entity = Fuselage;
		FM.bMirror = true;
		const FAerotelicMassReport Report = ComputeAerotelicMassReport({ P, F, W, FM }, 2e-3, Doc->GetUnits());

		auto Find = [&Report](const FString& Label) -> const FAerotelicMassLine*
		{
			for (const FAerotelicMassLine& L : Report.Bodies)
			{
				if (L.Label == Label) return &L;
			}
			return nullptr;
		};
		const FAerotelicMassLine* Prim = Find(Primitive->EntityName);
		const FAerotelicMassLine* Fuse = Find(Fuselage->EntityName);
		const FAerotelicMassLine* FuseMirror = Find(Fuselage->EntityName + TEXT(" (mirror)"));
		const FAerotelicMassLine* WingLine = Find(Wing->EntityName);
		Check(Prim && Fuse && FuseMirror && WingLine, "mass: one report line per chosen body, labelled by entity (mirror marked)");
		if (Prim != nullptr)
		{
			const double ExactVolume = Primitive->GetModelAs<aerotelic::PrimitiveModel>()->Volume();
			Check(Prim->bIncluded, "mass: a density weighs the body in");
			Check(FMath::IsNearlyEqual(Prim->Props.mass, 2.0 * ExactVolume, 2e-2 * 2.0 * ExactVolume),
				FString::Printf(TEXT("mass: density * exact volume (got %f, expected %f)"), Prim->Props.mass, 2.0 * ExactVolume));
			Check(Prim->Note.IsEmpty(), FString::Printf(TEXT("mass: a closed primitive carries no caveat (%s)"), *Prim->Note));
			Check(Prim->Props.inertia[0] > 0.0 && FMath::IsNearlyEqual(Prim->Props.inertia[0], Prim->Props.inertia[4], 1e-3 * Prim->Props.inertia[0]),
				"mass: the primitive's inertia is positive and (for a sphere) isotropic to the tessellation");
		}
		if (Fuse != nullptr && FuseMirror != nullptr)
		{
			Check(Fuse->bIncluded && FMath::IsNearlyEqual(Fuse->Props.mass, 150.0, 1e-9), "mass: an asserted mass comes back exactly");
			Check(FMath::IsNearlyEqual(FuseMirror->Props.mass, 150.0, 1e-9), "mass: the mirrored half carries the whole asserted mass again");
			Check(FMath::IsNearlyEqual(FuseMirror->Props.centre.y, -Fuse->Props.centre.y, 1e-9)
				&& FMath::IsNearlyEqual(FuseMirror->Props.centre.x, Fuse->Props.centre.x, 1e-9),
				"mass: the mirror's centre is the reflection of the original's");
			Check(FMath::IsNearlyEqual(FuseMirror->Props.inertia[0], Fuse->Props.inertia[0], 1e-9 * FMath::Max(1.0, Fuse->Props.inertia[0]))
				&& FMath::IsNearlyEqual(FuseMirror->Props.inertia[1], -Fuse->Props.inertia[1], 1e-9 * FMath::Max(1.0, FMath::Abs(Fuse->Props.inertia[1]))),
				"mass: reflection keeps Ixx and flips the sign of the xy product");
			Check(Fuse->Props.area > 0.0 && FMath::IsNearlyEqual(Fuse->Props.area, FuseMirror->Props.area, 1e-9 * Fuse->Props.area),
				"mass: the surface the asserted mass was spread over is reported");
		}
		if (WingLine != nullptr)
		{
			Check(!WingLine->bIncluded && WingLine->Note.Contains(TEXT("no material")),
				"mass: an entity with no setting is left out and says why");
		}
		Check(Report.IncludedCount == 3, FString::Printf(TEXT("mass: three bodies weighed (got %d)"), Report.IncludedCount));
		if (Prim != nullptr)
		{
			Check(FMath::IsNearlyEqual(Report.Assembly.mass, 300.0 + Prim->Props.mass, 1e-9),
				"mass: the assembly's mass is the sum of what was weighed");
			// Carrying each body to the joint centre can only add inertia.
			double OwnIxx = 0.0;
			for (const FAerotelicMassLine& L : Report.Bodies) if (L.bIncluded) OwnIxx += L.Props.inertia[0];
			Check(Report.Assembly.inertia[0] >= OwnIxx - 1e-9, "mass: the assembly's Ixx is at least the sum of the bodies' own (parallel axis adds)");

			// The same material as a skin: every body a shell of the
			// thickness, rho t A, its material volume t A rather than the
			// enclosed volume. The setting stays on for the round trip below
			// -- and the lattice generated on this sphere below hollows it by
			// THIS thickness, not the lattice's default.
			ByDensity.skin_thickness = 0.05;
			Primitive->SetMassSpec(ByDensity);
			const FAerotelicMassReport SkinReport = ComputeAerotelicMassReport({ P }, 2e-3, Doc->GetUnits());
			const FAerotelicMassLine* Skin = SkinReport.Bodies.Num() == 1 ? &SkinReport.Bodies[0] : nullptr;
			Check(Skin != nullptr && Skin->bIncluded && Skin->Note.Contains(TEXT("skin")), "mass: a skin thickness weighs the body in and says so");
			if (Skin != nullptr)
			{
				Check(FMath::IsNearlyEqual(Skin->Props.mass, 2.0 * 0.05 * Prim->Props.area, 1e-9 * Skin->Props.mass),
					FString::Printf(TEXT("mass: a skin weighs density * area * thickness (got %f, expected %f)"), Skin->Props.mass, 2.0 * 0.05 * Prim->Props.area));
				Check(FMath::IsNearlyEqual(Skin->Props.volume, 0.05 * Prim->Props.area, 1e-9 * Skin->Props.volume), "mass: a skin's volume is area * thickness");
				Check(Skin->Props.mass < Prim->Props.mass, "mass: a thin skin of the sphere weighs less than the solid");
			}
		}
		// The table: a header, one row per chosen component (the unset wing's
		// with dashes and the reason), and the assembly row.
		Check(Doc->GetUnits() == aerotelic::UnitSystem{}, "units: a new project is in metres and kilograms");
		const FString Text = FormatAerotelicMassReport(Report, Doc->GetUnits());
		Check(Text.Contains(TEXT("Component")) && Text.Contains(TEXT("Iyz")) && Text.Contains(TEXT("ASSEMBLY (3 bodies)")),
			"mass: the report is a table with a header and the assembly row");
		Check(Text.Contains(TEXT("(kg)")) && Text.Contains(TEXT("(kg m^2)")) && Text.Contains(TEXT("Project units: m, kg, s")),
			"mass: the report names the project's units in its header and columns");
		aerotelic::UnitSystem Imperial;
		Imperial.length = aerotelic::LengthUnit::Foot;
		Imperial.mass = aerotelic::MassUnit::PoundMass;
		const FString ImperialText = FormatAerotelicMassReport(Report, Imperial);
		Check(ImperialText.Contains(TEXT("(lbm)")) && ImperialText.Contains(TEXT("(lbm ft^2)")),
			"mass: the same report labels itself in feet and pounds when asked");
		TArray<FString> Lines;
		Text.ParseIntoArrayLines(Lines);
		int32 WingRows = 0;
		for (const FString& L : Lines)
		{
			if (L.StartsWith(Wing->EntityName) && L.Contains(TEXT("left out: no material"))) ++WingRows;
		}
		Check(WingRows == 1, "mass: the unset wing keeps its row, with the reason in place of numbers");
	}

	// A boolean result over the fuselage and the sphere, saved under
	// `booleans:` by name and rebuilt on load.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: AddBoolean"));
	{
		TArray<FAerotelicBooleanOperand> Operands;
		FAerotelicBooleanOperand A, B;
		A.Entity = Fuselage;
		B.Entity = Primitive;
		Operands.Add(A);
		Operands.Add(B);
		FString Error;
		AAerotelicEntityActor* Fused = Doc->AddBoolean(TEXT("BoolOp 1"), Operands, TEXT("fuse"), Error);
		Check(Fused != nullptr, FString::Printf(TEXT("fuse of fuselage and sphere built: %s"), *Error));
		Check(Doc->GetEntities().Num() == 4, "the boolean result is a fourth scene item");
		Check(Doc->NextBooleanName() == TEXT("BoolOp 2"), "next default boolean name is BoolOp 2");
		AAerotelicEntityActor* Dup = Doc->AddBoolean(TEXT("BoolOp 1"), Operands, TEXT("cut"), Error);
		Check(Dup == nullptr && !Error.IsEmpty(), "a duplicate name is refused");
	}

	// A lattice generated inside the sphere (a small one: a 2-unit sphere at
	// a 0.5 cell and 6 voxels per cell), saved under `lattices:` and
	// regenerated on load.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: AddLattice"));
	{
		TArray<FAerotelicBooleanOperand> Targets;
		FAerotelicBooleanOperand T;
		T.Entity = Primitive;
		Targets.Add(T);
		aerotelic::LatticeSpec Spec;
		Spec.family = aerotelic::LatticeFamily::Tpms;
		Spec.type = "Gyroid";
		Spec.phase = aerotelic::TpmsPhase::Network;
		Spec.cell_size = 0.5;
		Spec.volume_fraction = 0.35;
		Spec.resolution = 6;
		// No skin thickness of the lattice's own: the sphere carries 0.05
		// under Material Properties (set in the mass step) and is hollowed
		// by that; a body without one is refused by name.
		Spec.skin_thickness = 0.0;
		FString Error;
		{
			FAerotelicBooleanOperand W;
			W.Entity = Wing;   // no material set
			Check(FMath::IsNearlyEqual(FAerotelicLatticeModel::SkinThicknessFor(T), 0.05, 1e-12), "skin per body: the sphere's thickness is its Material Properties figure");
			Check(FAerotelicLatticeModel::SkinThicknessFor(W) == 0.0, "skin per body: a body without a material has no skin thickness");
			TArray<FAerotelicBooleanOperand> WithWing = Targets;
			WithWing.Add(W);
			AAerotelicEntityActor* Refused = Doc->AddLattice(TEXT("Lattice 1"), Spec, WithWing, TArray<FAerotelicBooleanOperand>(), Error);
			Check(Refused == nullptr && Error.Contains(Wing->EntityName) && Error.Contains(TEXT("skin thickness")), FString::Printf(TEXT("a body without a skin thickness is refused by name: %s"), *Error));
		}
		AAerotelicEntityActor* Lattice = Doc->AddLattice(TEXT("Lattice 1"), Spec, Targets, TArray<FAerotelicBooleanOperand>(), Error);
		Check(Lattice != nullptr, FString::Printf(TEXT("gyroid lattice in the sphere generated: %s"), *Error));
		Check(Doc->GetEntities().Num() == 5, "the lattice is a fifth scene item");
		Check(Doc->NextLatticeName() == TEXT("Lattice 2"), "next default lattice name is Lattice 2");
		if (Lattice != nullptr)
		{
			const FAerotelicLatticeModel* M = Lattice->GetModelAs<FAerotelicLatticeModel>();
			Check(M->GetStats().Triangles > 0, "the lattice has triangles");
			Check(FMath::IsNearlyEqual(M->MinSkinThickness(), 0.05, 1e-12) && M->GetTetBodies().size() == 1 && FMath::IsNearlyEqual(M->GetTetBodies()[0].SkinThickness, 0.05, 1e-12),
				"skin per body: the sphere was hollowed by its own 0.05 skin");
			// The lattice inherits the sphere's material; it has none of its own
			// to set, and the mass report weighs its two bodies by that density.
			Check(Lattice->IsDerived() && Lattice->GetMassSpecSource() == Primitive && Lattice->GetEffectiveMassSpec().material.name == "Custom"
				&& FMath::IsNearlyEqual(Lattice->GetEffectiveMassSpec().material.density, 2.0, 1e-12), "inheritance: the lattice's material is the sphere's");
			{
				FAerotelicBooleanOperand LOp;
				LOp.Entity = Lattice;
				const FAerotelicMassReport LatticeReport = ComputeAerotelicMassReport({ LOp }, 2e-3, Doc->GetUnits());
				double LatticeMass = 0.0, LatticeVolume = 0.0;
				bool bAll = LatticeReport.Bodies.Num() == 2;
				for (const FAerotelicMassLine& Line : LatticeReport.Bodies) { bAll = bAll && Line.bIncluded && Line.Note.Contains(TEXT("inherited")); LatticeMass += Line.Props.mass; LatticeVolume += Line.Props.volume; }
				Check(bAll && FMath::IsNearlyEqual(LatticeMass, 2.0 * LatticeVolume, 1e-9 * LatticeMass), FString::Printf(TEXT("inheritance: the lattice's skin and infill weigh by the sphere's density as solids (%.4g over %.4g)"), LatticeMass, LatticeVolume));
			}
			// The fraction is of the interior domain (inside the explicit skin).
			Check(FMath::Abs(M->GetStats().AchievedFraction - 0.35) < 0.1, FString::Printf(TEXT("the lattice's achieved volume fraction (%.3f) is near the 0.35 asked"), M->GetStats().AchievedFraction));
			Check(M->GetStats().SkinTriangles > 0 && M->GetStats().LatticeTriangles > 0, "the skin and the lattice both have triangles");
			// The tetrahedra (Export Skin Tetrahedra / Export Tetrahedral Mesh)
			// are built on request over the body and its MeshLib offset: skin
			// + core fill the sphere's mesh exactly, the core fills the offset.
			Check(M->GetTetBodies().size() == 1 && M->GetTetBodies()[0].bInnerUsable && !M->GetTetBodies()[0].Inner.IsEmpty(), "the generation kept the body and a usable offset surface for the tetrahedral mesher");
			const std::vector<FAerotelicLatticeModel::FTetMesh>& Tets = M->GetTetMeshes();
			Check(Tets.size() == 1 && M->MergedTets(1).tets.size() > 0 && M->MergedTets(-1).tets.size() > M->MergedTets(1).tets.size(), FString::Printf(TEXT("the skin and core tetrahedra can be built for export (%s)"), *FString::Join(M->GetTetNotes(), TEXT("; "))));
			if (Tets.size() == 1)
			{
				const AerotelicTetGmsh::FResult& S = Tets[0].Result;
				const double Body = M->GetTetBodies()[0].Outer.SignedVolume();
				const double Inner = M->GetTetBodies()[0].Inner.SignedVolume();
				Check(S.VolumeError <= 1e-9, FString::Printf(TEXT("skin + core tetrahedra (%.9g) fill the body's volume (%.9g): relative %.2e"), S.SkinVolume + S.CoreVolume, Body, S.VolumeError));
				Check(FMath::Abs(S.CoreVolume - Inner) <= 1e-9 * Body, FString::Printf(TEXT("the core tetrahedra (%.9g) fill the offset's volume (%.9g) exactly"), S.CoreVolume, Inner));
				Check(S.Degenerate == 0 && S.MinQuality > 0.0, FString::Printf(TEXT("no tetrahedron is flat or inverted (%d degenerate, worst quality %.4f)"), S.Degenerate, S.MinQuality));
				// gmsh adds interior nodes only, so the surfaces survive
				// node for node and the boundary is the sphere's own mesh.
				Check(S.Nodes > M->GetTetBodies()[0].Outer.vertices.size() + M->GetTetBodies()[0].Inner.vertices.size(),
					FString::Printf(TEXT("the mesh keeps both surfaces' nodes and adds interior ones (%d nodes over %d on the surfaces)"), S.Nodes,
						(int32)(M->GetTetBodies()[0].Outer.vertices.size() + M->GetTetBodies()[0].Inner.vertices.size())));
			}
			std::vector<aerotelic::NamedMesh> Meshes = Lattice->GetPlacedMeshes(false, 1e-3);
			Check(Meshes.size() == 2 && Meshes[0].name == "Skin" && Meshes[1].name == "Lattice", "the lattice is two bodies, Skin then Lattice");
			// The skin solid's volume is the 0.05 shell of the unit sphere (a
			// few percent under: the sphere is meshed coarsely at this voxel,
			// and an inscribed polyhedron is smaller than its sphere).
			if (Meshes.size() == 2)
			{
				const double Shell = 4.0 / 3.0 * PI * (1.0 - 0.95 * 0.95 * 0.95);
				Check(FMath::Abs(Meshes[0].mesh.SignedVolume() - Shell) < 0.06 * Shell, FString::Printf(TEXT("the skin body encloses the shell's volume (%.4f vs %.4f)"), Meshes[0].mesh.SignedVolume(), Shell));
			}
			bool bClosed = Meshes.size() == 2;
			for (const aerotelic::NamedMesh& NM : Meshes) bClosed = bClosed && NM.mesh.IsClosedManifold() && NM.mesh.SignedVolume() > 0.0;
			Check(bClosed, "both the skin and the lattice are closed, outward bodies");
			// Every mesh vertex lies inside the sphere (radius 1 at the origin);
			// the lattice's inside the skin's inner offset.
			double Worst = 0.0, WorstLattice = 0.0;
			for (const aerotelic::NamedMesh& NM : Meshes)
				for (const aerotelic::Vec3& V : NM.mesh.vertices)
				{
					Worst = FMath::Max(Worst, aerotelic::Length(V) - 1.0);
					if (NM.name == "Lattice") WorstLattice = FMath::Max(WorstLattice, aerotelic::Length(V) - 0.95);
				}
			Check(Worst < 0.1, FString::Printf(TEXT("the lattice stays inside the sphere (worst overshoot %.4f)"), Worst));
			Check(WorstLattice < 0.1, FString::Printf(TEXT("the infill stays inside the skin (worst overshoot %.4f)"), WorstLattice));
			bool bThrew = false;
			try { Lattice->GetModel()->BuildSolid(); } catch (const std::exception&) { bThrew = true; }
			Check(bThrew, "a lattice has no B-rep solid (BuildSolid throws)");

			// A changed skin thickness under Material Properties regenerates
			// the lattice (2026-11-04): the sphere's 0.05 becomes 0.08, the
			// document queues the lattice as it does after an Apply, and once
			// flushed the lattice is hollowed by 0.08. The same spec set again,
			// or a density edited under the same skin, queues nothing (a
			// lattice is built from the skin alone).
			{
				aerotelic::MassSpec Changed = Primitive->GetMassSpec();
				Primitive->SetMassSpec(Changed);
				Check(!Doc->HasPendingRebuilds(), "skin change: the same material properties set again queue nothing");
				Changed.material.density = 3.0;
				Primitive->SetMassSpec(Changed);
				Check(!Doc->HasPendingRebuilds(), "skin change: a density edited under the same skin queues nothing");
				Changed.material.density = 2.0;
				Changed.skin_thickness = 0.08;
				Primitive->SetMassSpec(Changed);
				Check(Doc->HasPendingRebuilds(), "skin change: a new skin thickness queues the lattice over the sphere");
				Doc->FlushPendingRebuilds();
				Check(!Doc->HasPendingRebuilds() && FMath::IsNearlyEqual(M->MinSkinThickness(), 0.08, 1e-12) && M->GetTetBodies().size() == 1 && FMath::IsNearlyEqual(M->GetTetBodies()[0].SkinThickness, 0.08, 1e-12),
					FString::Printf(TEXT("skin change: the lattice regenerated over the 0.08 skin (hollowed by %.3g)"), M->GetTetBodies().size() == 1 ? M->GetTetBodies()[0].SkinThickness : -1.0));
				// Back to 0.05 for the steps that follow (the round trip, Apply).
				Changed.skin_thickness = 0.05;
				Primitive->SetMassSpec(Changed);
				Check(Doc->HasPendingRebuilds(), "skin change: back to 0.05 queues the lattice again");
				Doc->FlushPendingRebuilds();
				Check(FMath::IsNearlyEqual(M->MinSkinThickness(), 0.05, 1e-12), "skin change: regenerated over the 0.05 skin again");
			}
		}
		AAerotelicEntityActor* Bad = Doc->AddLattice(TEXT("Lattice 1"), Spec, Targets, TArray<FAerotelicBooleanOperand>(), Error);
		Check(Bad == nullptr && !Error.IsEmpty(), "a duplicate lattice name is refused");
	}

	// A design panel's Apply: the panel edits a DRAFT, an independent copy
	// of the entity's model, and nothing in the scene changes until the
	// draft is applied -- at which point the entity gets a fresh model of
	// its own and the lattice over it follows.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: apply"));
	if (Primitive != nullptr && Wing != nullptr)
	{
		AAerotelicEntityActor* Lattice = nullptr;
		AAerotelicEntityActor* Boolean = nullptr;
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid()) continue;
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Lattice) Lattice = Weak.Get();
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean) Boolean = Weak.Get();
		}
		Check(Doc->CloneEntityModel(Boolean) == nullptr, "apply: a boolean result has no draft to edit (its operands have)");
		Check(Doc->CloneEntityModel(Lattice) == nullptr, "apply: a lattice has no draft to edit (its own dialog applies)");

		std::unique_ptr<aerotelic::ISolidModel> Draft = Doc->CloneEntityModel(Primitive);
		Check(Draft != nullptr, "apply: the sphere clones for editing");
		if (Draft != nullptr)
		{
			const aerotelic::PrimitiveModel* Live = Primitive->GetModelAs<aerotelic::PrimitiveModel>();
			aerotelic::PrimitiveModel* D = static_cast<aerotelic::PrimitiveModel*>(Draft.get());
			const double R0 = Live->radius;
			Check(D != Live && FMath::IsNearlyEqual(D->radius, R0, 1e-12), "apply: the draft is a model of its own carrying the entity's numbers");
			D->radius = R0 * 1.5;
			Check(FMath::IsNearlyEqual(Live->radius, R0, 1e-12), "apply: editing the draft leaves the entity alone");
			Check(!Doc->HasPendingRebuilds() && Doc->GetRebuildProgress() == 1.0f, "apply: nothing is waiting to rebuild before anything is applied, and the progress bar is full");

			FString Error;
			Check(Doc->ApplyEntityModel(Primitive, *Draft, Error), FString::Printf(TEXT("apply: the draft applies (%s)"), *Error));
			const aerotelic::PrimitiveModel* Applied = Primitive->GetModelAs<aerotelic::PrimitiveModel>();
			Check(Applied != Live && Applied != D, "apply: the entity has a fresh model, neither the old one nor the draft");
			Check(FMath::IsNearlyEqual(Applied->radius, R0 * 1.5, 1e-12), "apply: the entity now carries the draft's numbers");
			D->radius = R0 * 1.25;
			Check(FMath::IsNearlyEqual(Primitive->GetModelAs<aerotelic::PrimitiveModel>()->radius, R0 * 1.5, 1e-12),
				"apply: editing the draft after Apply still leaves the entity alone until the next Apply");

			// The boolean and the lattice over the sphere follow: queued by
			// the Apply, rebuilt by the flush (the ticker's job in the
			// application). Three parameterisations in all -- the sphere,
			// done, then those two -- so the bar stands at a third.
			Check(Doc->HasPendingRebuilds(), "apply: the boolean and the lattice over the sphere are queued to follow");
			Check(FMath::IsNearlyEqual(Doc->GetRebuildProgress(), 1.0f / 3.0f, 1e-6f), FString::Printf(TEXT("apply: the progress bar shows one of three rebuilt (%.3f)"), Doc->GetRebuildProgress()));
			Doc->FlushPendingRebuilds();
			Check(!Doc->HasPendingRebuilds() && Doc->GetRebuildProgress() == 1.0f, "apply: the flush rebuilt what was queued and the bar is full again");
			if (Lattice != nullptr)
			{
				const FAerotelicLatticeModel* M = Lattice->GetModelAs<FAerotelicLatticeModel>();
				Check(M->IsValid() && M->GetStats().Triangles > 0, "apply: the lattice is generated again");
				std::vector<aerotelic::NamedMesh> Meshes = Lattice->GetPlacedMeshes(false, 1e-3);
				double Reach = 0.0;
				for (const aerotelic::NamedMesh& NM : Meshes)
					for (const aerotelic::Vec3& V : NM.mesh.vertices) Reach = FMath::Max(Reach, aerotelic::Length(V));
				Check(Reach > 1.3 * R0 && Reach < 1.6 * R0, FString::Printf(TEXT("apply: the lattice now fills the larger sphere (reaches %.3f of a %.3f radius)"), Reach, 1.5 * R0));
			}

			// Back to the unit sphere for the rest of the run.
			D->radius = R0;
			Check(Doc->ApplyEntityModel(Primitive, *Draft, Error), FString::Printf(TEXT("apply: restoring applies (%s)"), *Error));
			Doc->FlushPendingRebuilds();
		}

		// A lofted model takes the spec round trip the clone is made by.
		std::unique_ptr<aerotelic::ISolidModel> WingDraft = Doc->CloneEntityModel(Wing);
		Check(WingDraft != nullptr, "apply: the wing clones for editing");
		if (WingDraft != nullptr)
		{
			aerotelic::WingModel* WD = static_cast<aerotelic::WingModel*>(WingDraft.get());
			const double Span = Wing->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan();
			Check(FMath::IsNearlyEqual(WD->planform().SemiSpan(), Span, 1e-12), "apply: the wing's draft starts on the wing's parameters");
			WD->planform().SetSemiSpan(Span * 2.0);
			Check(FMath::IsNearlyEqual(Wing->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan(), Span, 1e-12), "apply: the wing is untouched until Apply");
			FString Error;
			Check(Doc->ApplyEntityModel(Wing, *WingDraft, Error), FString::Printf(TEXT("apply: the wing's draft applies (%s)"), *Error));
			Check(FMath::IsNearlyEqual(Wing->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan(), Span * 2.0, 1e-12), "apply: the wing took the draft's span");
			CheckBuildsSolid(Wing, TEXT("Wing after Apply"));
			WD->planform().SetSemiSpan(Span);
			Check(Doc->ApplyEntityModel(Wing, *WingDraft, Error), "apply: restoring the wing applies");
		}
	}

	// The viewport's OCCT-free meshes: every entity's model builds its
	// bodies straight from its curves, closed and outward.
	// Structural analysis (Analysis -> Structural (Kratos FEA)) on the
	// sphere four ways, each held to closed forms. The sphere's material is
	// the aluminium set in the mass step (skin 0.05); the lattice that
	// fills it inherits it. The aluminium skin of a unit sphere over a very
	// soft core, stood on its bottom under its own weight and 1 bar: the
	// reactions carry the weight exactly (the pressure's resultant over a
	// closed surface is zero), the surface moves inward, and the skin's
	// hoop stress is Lame's thick-shell value, ~1.05 p for a/b = 0.95.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea"));
	if (AerotelicStructuralFea::FindScript().IsEmpty() || AerotelicMeshLib::FindPython().IsEmpty())
	{
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("skipped: tools\\structural_fea.py or the WingDesigner environment is not here"));
	}
	else
	{
		AAerotelicEntityActor* FeaLattice = nullptr;
		for (auto& Weak : Doc->GetEntities())
		{
			if (Weak.IsValid() && Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Lattice) FeaLattice = Weak.Get();
		}
		const FAerotelicLatticeModel* FeaModel = FeaLattice ? FeaLattice->GetModelAs<FAerotelicLatticeModel>() : nullptr;
		const aerotelic::Material Al = *aerotelic::FindMaterialPreset("Aluminium 2024-T3");
		aerotelic::Material Soft;
		Soft.name = "soft core";
		Soft.density = 50.0;
		Soft.youngs_modulus = 1.0e5;
		Soft.poisson_ratio = 0.3;
		auto Support = [](AerotelicStructuralFea::FRequest& Request)
		{
			Request.LengthToSI = 1.0;
			AerotelicStructuralFea::FSupport Bottom;
			Bottom.Axis = TEXT('z');
			Bottom.bAtMin = true;
			Bottom.Tolerance = 0.15;
			Request.Supports.Add(Bottom);
			Request.LoadFactor = 1.0;
			Request.Pressure = 1.0e5;
		};

		// (1) Full resolution: the skin and core tetrahedra, each region a
		// solid of its material, second order. A direct solve: the skin is
		// 7e5 times stiffer than this core, which the iterative solver
		// converges on poorly (its force balance closed to 1.7% and its
		// peak stress wandered).
		aerotelic::TetMesh FeaMesh = FeaModel ? FeaModel->MergedTets(-1) : aerotelic::TetMesh();
		Check(!FeaMesh.tets.empty(), "fea: the lattice's skin and core tetrahedra are the model");
		if (!FeaMesh.tets.empty())
		{
			AerotelicStructuralFea::FRequest Request;
			Request.Mesh = &FeaMesh;
			Request.LinearSolver = TEXT("sparse_lu");
			AerotelicStructuralFea::FRegion Skin, Core;
			Skin.Id = 1; Skin.Label = TEXT("skin"); Skin.Core = Al; Skin.bSolid = true;
			Core.Id = 2; Core.Label = TEXT("core"); Core.Core = Soft; Core.bSolid = true;
			Request.Regions.Add(Skin);
			Request.Regions.Add(Core);
			Request.bSecondOrder = true;
			Support(Request);
			Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_fea.vtk"));
			IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
			AerotelicStructuralFea::FResult R;
			FString Error;
			const bool bSolved = AerotelicStructuralFea::Run(Request, R, Error);
			Check(bSolved, FString::Printf(TEXT("fea: the sphere solved (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d elements, %d dof, %.1f s"), R.Nodes, R.Elements, R.Dofs, R.Seconds) : *Error));
			if (bSolved)
			{
				const AerotelicTetGmsh::FResult& S = FeaModel->GetTetMeshes()[0].Result;
				const double ExpectedMass = 2780.0 * S.SkinVolume + 50.0 * S.CoreVolume;
				Check(R.bSecondOrder && R.Nodes > static_cast<int32>(FeaMesh.nodes.size()), "fea: solved on 10-node tetrahedra (mid-edge nodes added)");
				Check(FMath::IsNearlyEqual(R.Mass, ExpectedMass, 1e-6 * ExpectedMass),
					FString::Printf(TEXT("fea: the mass is the material densities over the regions' volumes (%.6g vs %.6g)"), R.Mass, ExpectedMass));
				const double Weight = ExpectedMass * 9.80665;
				Check(FMath::Abs(R.Reaction[2] - Weight) <= 0.02 * Weight && FMath::Abs(R.Reaction[0]) <= 0.02 * Weight && FMath::Abs(R.Reaction[1]) <= 0.02 * Weight,
					FString::Printf(TEXT("fea: the support carries the weight, %.4g N, and nothing else (reactions %.4g, %.4g, %.4g)"), Weight, R.Reaction[0], R.Reaction[1], R.Reaction[2]));
				// The script's own verdict on the same balance, and which
				// solver ran (asked for by name here; "auto" is the dialog's).
				Check(R.bBalanced && R.BalanceError <= 0.01 && R.Warnings.Num() == 0 && R.LinearSolver == TEXT("sparse_lu"),
					FString::Printf(TEXT("fea: the result reports a balanced solve (error %.2e, %d warning(s)) by %s"), R.BalanceError, R.Warnings.Num(), *R.LinearSolver));
				Check(FMath::Abs(R.AppliedPressure[2]) <= 1e-3 * 1.0e5 * 4.0 * PI,
					FString::Printf(TEXT("fea: a uniform pressure over a closed surface has no resultant (%.3g N)"), R.AppliedPressure[2]));
				Check(R.MeanNormalDisplacement < 0.0, FString::Printf(TEXT("fea: an external pressure moves the surface inward (mean u.n = %.3g m)"), R.MeanNormalDisplacement));
				// Lame's hoop stress is 1.05 MPa; the peak sits where the weight
				// enters the fixed cap, 1.9 MPa with the direct solver.
				Check(R.Entries.Num() == 2 && R.Entries[0].Kind == TEXT("solid") && R.Entries[0].MaxVonMises > 0.8e6 && R.Entries[0].MaxVonMises < 2.5e6,
					FString::Printf(TEXT("fea: the skin's peak von Mises is Lame's hoop stress for the shell, ~1.05e6 Pa, up to the support's concentration (got %.4g)"), R.Entries.Num() ? R.Entries[0].MaxVonMises : 0.0));
				Check(R.Entries.Num() == 2 && R.Entries[1].MaxVonMises < 0.1 * R.Entries[0].MaxVonMises, "fea: the soft core carries almost nothing");
				Check(R.Entries.Num() == 2 && R.Entries[0].Margin > 100.0, "fea: 2024-T3 at 1 bar has a margin in the hundreds");
				Check(R.MaxDisplacement > 0.0 && R.MaxDisplacement < 2e-2, FString::Printf(TEXT("fea: the displacement is small and finite (%.3g m)"), R.MaxDisplacement));
				Check(IFileManager::Get().FileSize(*Request.VtkPath) > 0, FString::Printf(TEXT("fea: the solution was written to %s"), *Request.VtkPath));
				IFileManager::Get().Delete(*Request.VtkPath, false, true, true);

				// The pre-processor (the dialog's Mesh / Loads / Support tabs)
				// derives the same model the script built.
				AerotelicFeaPreprocess::FModel Pre;
				FString PreError;
				const bool bPre = AerotelicFeaPreprocess::Build(Request, Pre, PreError);
				Check(bPre, FString::Printf(TEXT("fea (pre-processor): the model derives (%s)"), *PreError));
				if (bPre)
				{
					Check(Pre.WettedFaces.Num() == R.WettedFaces, FString::Printf(TEXT("fea (pre-processor): the wetted faces are the script's (%d vs %d)"), Pre.WettedFaces.Num(), R.WettedFaces));
					Check(Pre.SupportNodes.Num() == R.SupportCornerNodes, FString::Printf(TEXT("fea (pre-processor): the fixed corner nodes are the script's (%d vs %d)"), Pre.SupportNodes.Num(), R.SupportCornerNodes));
					// Second order: the mid-edge nodes on the plane are clamped too
					// (left free until 2026-11-02, a compliant clamp).
					Check(R.SupportNodes > R.SupportCornerNodes, FString::Printf(TEXT("fea (pre-processor): the solver clamped the mid-edge nodes on the plane as well (%d nodes, %d corners)"), R.SupportNodes, R.SupportCornerNodes));
					// The cost estimate (2026-11-05): the mid-edge nodes and the
					// degrees of freedom counted before the solve are the script's.
					Check(Pre.SecondOrderTets == Pre.Tets && Pre.Nodes + Pre.MidEdgeNodes == R.Nodes && Pre.Dofs == R.Dofs,
						FString::Printf(TEXT("fea (pre-processor): the DOF estimate is the solver's (%lld vs %d; %d mid-edge nodes)"), Pre.Dofs, R.Dofs, Pre.MidEdgeNodes));
					bool bOnPlane = Pre.SupportNodes.Num() > 0 && Pre.SupportPlanes.Num() == 1 && Pre.SupportPlanes[0].Axis == 2;
					for (int32 N : Pre.SupportNodes) bOnPlane = bOnPlane && FMath::Abs(FeaMesh.nodes[N].z - Pre.SupportPlanes[0].Value) <= Pre.SupportPlanes[0].Tolerance;
					Check(bOnPlane && FMath::IsNearlyEqual(Pre.SupportPlanes[0].Value, Pre.Bounds.min.z, 1e-12), "fea (pre-processor): every fixed node lies within the tolerance of z = the mesh's minimum");
					Check(Pre.Regions.Num() == 2 && Pre.Regions[0].Label == TEXT("skin") && Pre.Regions[1].Label == TEXT("core") && Pre.Regions[0].Kind == TEXT("solid"), "fea (pre-processor): a region per material, labelled, solids");
					Check(!Pre.bMassEstimated && FMath::IsNearlyEqual(Pre.Mass, R.Mass, 1e-6 * R.Mass), FString::Printf(TEXT("fea (pre-processor): the mass is the script's (%.6g vs %.6g)"), Pre.Mass, R.Mass));
					Check(FMath::IsNearlyEqual(Pre.GravityForce.Z, R.AppliedGravity[2], 1e-6 * FMath::Abs(R.AppliedGravity[2])) && Pre.GravityForce.X == 0.0 && Pre.GravityForce.Y == 0.0,
						FString::Printf(TEXT("fea (pre-processor): the inertial load is the script's applied gravity (%.6g vs %.6g N)"), Pre.GravityForce.Z, R.AppliedGravity[2]));
					Check(Pre.PressureResultant.Size() <= 1e-3 * 1.0e5 * 4.0 * PI && Pre.MaxFacePressure == 1.0e5,
						FString::Printf(TEXT("fea (pre-processor): the pressure's resultant over the closed surface is zero (%.3g N)"), Pre.PressureResultant.Size()));
					Check(Pre.PointLoads.Num() == 0 && Pre.MassCentre.Size() < 0.05, FString::Printf(TEXT("fea (pre-processor): no point force; the centre of mass is near the sphere's centre (%.3g)"), Pre.MassCentre.Size()));
					bool bOutward = true;
					for (int32 i = 0; i < Pre.WettedFaces.Num(); ++i) bOutward = bOutward && FVector::DotProduct(Pre.FaceNormal[i], Pre.FaceCentre[i]) > 0.0;
					Check(bOutward, "fea (pre-processor): the wetted faces are wound outward");
				}
			}
		}

		// (2) Shells over a homogenised core, on the body's OWN grid
		// (2026-11-05): the sphere's placed mesh at tolerance 0 -- its Mesh
		// tab's grid at the viewport's rule, exactly what the viewport
		// shows -- is filled by gmsh at the lattice cell size with the
		// surface left as it is, as the structural dialog does; the lattice
		// lends its cell and skin thickness only. Shells of the 0.05 skin on
		// those triangles over tetrahedra carrying the gyroid's homogenised
		// stiffness, the gyroid made of PMI foam here (the API takes any
		// base material; in the app the struts are the body's). Closed
		// forms: the mass is rho t A of the skin plus the core's effective
		// density, less the overlap, over the volume; the support carries
		// the weight; the membrane stress of a thin shell under an external
		// pressure is p a / 2t = 1.0 MPa.
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea (shells over a homogenised core)"));
		if (FeaModel != nullptr && Primitive != nullptr)
		{
			const aerotelic::Material Foam = *aerotelic::FindMaterialPreset("Foam, PMI 71 kg/m^3");
			const std::vector<aerotelic::NamedMesh> OwnGrid = Primitive->GetPlacedMeshes(false);
			const std::vector<aerotelic::NamedMesh> Finer = Primitive->GetPlacedMeshes(false, 1e-3);
			Check(OwnGrid.size() == 1 && OwnGrid[0].mesh.IsClosedManifold() && OwnGrid[0].mesh.SignedVolume() > 0.0, "fea (shells): the body's own grid (tolerance 0) is one closed, outward body");
			Check(OwnGrid.size() == 1 && Finer.size() == 1 && OwnGrid[0].mesh.triangles.size() < Finer[0].mesh.triangles.size(),
				FString::Printf(TEXT("fea (shells): the own grid is the viewport's, coarser than a finer tolerance asks for (%d vs %d triangles)"), OwnGrid.size() ? (int32)OwnGrid[0].mesh.triangles.size() : -1, Finer.size() ? (int32)Finer[0].mesh.triangles.size() : -1));
			// The lattice's private copy, hollowed at a quarter of the skin, is
			// NOT what the solver gets: its triangle count differs from the grid's.
			Check(FeaModel->GetTetBodies().size() == 1, "fea (shells): the lattice keeps its one hollowed body to itself");
			AerotelicTetGmsh::FResult S;
			bool bMeshed = false;
			if (OwnGrid.size() == 1)
			{
				AerotelicTetGmsh::FRequest G;
				G.Outer = &OwnGrid[0].mesh;
				G.Inner = nullptr;
				G.CoreSize = FeaModel->Spec.cell_size;
				G.Algorithm = 1;
				G.bOptimize = true;
				G.bOptimizeNetgen = true;
				G.bSizeFromBoundary = false;
				FString GmshError;
				bMeshed = AerotelicTetGmsh::Run(G, S, GmshError);
				Check(bMeshed && !S.Mesh.tets.empty(), FString::Printf(TEXT("fea (shells): the body's own grid fills with tetrahedra at the cell size (%s)"), bMeshed ? *FString::Printf(TEXT("%d tetrahedra on %d nodes"), S.SkinTets + S.CoreTets, S.Nodes) : *GmshError));
			}
			if (bMeshed && !S.Mesh.tets.empty())
			{
				const aerotelic::TriMesh& Outer = OwnGrid[0].mesh;
				Check(S.VolumeError <= 1e-9, FString::Printf(TEXT("fea (shells): the core tetrahedra fill the body exactly (relative %.2e)"), S.VolumeError));
				Check(S.SkinTets + S.CoreTets < static_cast<int32>(FeaMesh.tets.size()), FString::Printf(TEXT("fea (shells): fewer tetrahedra than the skin / core mesh (%d vs %d)"), S.SkinTets + S.CoreTets, (int32)FeaMesh.tets.size()));
				aerotelic::TetMesh CoreMesh = S.Mesh;
				CoreMesh.material.assign(CoreMesh.tets.size(), 1);
				aerotelic::UnitCellSamples Cell;
				bool bSampled = true;
				try { Cell = aerotelic::SampleUnitCell(FeaModel->Spec, 24); } catch (const std::exception&) { bSampled = false; }
				Check(bSampled && Cell.nx == 24 && FMath::Abs(Cell.solid_fraction - 0.35) < 0.05, FString::Printf(TEXT("fea (shells): the gyroid cell voxelises near its 0.35 fraction (%.3f)"), Cell.solid_fraction));
				AerotelicStructuralFea::FRequest Request;
				Request.Mesh = &CoreMesh;
				AerotelicStructuralFea::FRegion Region;
				Region.Id = 1;
				Region.Label = TEXT("sphere");
				Region.Skin = Al;
				Region.SkinThickness = Primitive->GetEffectiveMassSpec().skin_thickness;
				Region.Core = Foam;
				Region.Cell = &Cell;
				// A homogenised core is first order whatever the request says:
				// the dialog's rule, exercised here against a request asking
				// for 10 nodes.
				Region.SecondOrder = 0;
				Request.Regions.Add(Region);
				Request.bSecondOrder = true;
				Request.bHomogenizeCache = false;
				Support(Request);
				Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_fea_shells.vtk"));
				IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
				Check(FMath::IsNearlyEqual(Region.SkinThickness, 0.05, 1e-12), "fea (shells): the shells take the body's own skin thickness");
				AerotelicStructuralFea::FResult R;
				FString Error;
				const bool bSolved = bSampled && AerotelicStructuralFea::Run(Request, R, Error);
				Check(bSolved, FString::Printf(TEXT("fea (shells): the sphere solved (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d tetrahedra + %d shells, %d dof, %.1f s"), R.Nodes, R.Tets, R.Shells, R.Dofs, R.Seconds) : *Error));
				if (bSolved)
				{
					Check(R.Shells == static_cast<int32>(Outer.triangles.size()) && R.Tets == S.SkinTets + S.CoreTets, "fea (shells): a shell on each of the body's own triangles, a tetrahedron for each of gmsh's");
					Check(!R.bSecondOrder && R.Nodes == static_cast<int32>(CoreMesh.nodes.size()) && R.Dofs > 3 * R.Nodes, "fea (shells): the core stayed 4-node under a request for 10 (no mid-edge nodes), with rotations on the shell nodes");
					Check(R.Regions.Num() == 1 && R.Regions[0].RelativeDensity > 0.25 && R.Regions[0].RelativeDensity < 0.45,
						FString::Printf(TEXT("fea (shells): the homogenised core has the gyroid's relative density (%.3f)"), R.Regions.Num() ? R.Regions[0].RelativeDensity : 0.0));
					if (R.Regions.Num() == 1)
					{
						const AerotelicStructuralFea::FRegionResult& G = R.Regions[0];
						const double Ratio = G.E[0] / Foam.youngs_modulus;
						Check(Ratio > 0.05 && Ratio < G.RelativeDensity && FMath::Abs(G.E[0] - G.E[1]) < 0.05 * G.E[0] && FMath::Abs(G.E[0] - G.E[2]) < 0.05 * G.E[0],
							FString::Printf(TEXT("fea (shells): the gyroid's effective modulus is cubic and between the bending and stretching bounds (E/E_s = %.3f, %.3f, %.3f)"), G.E[0] / Foam.youngs_modulus, G.E[1] / Foam.youngs_modulus, G.E[2] / Foam.youngs_modulus));
						Check(G.Amplification >= 1.0 && G.CoreMaxMicroVonMises >= G.CoreMaxVonMises, FString::Printf(TEXT("fea (shells): the strut stress is estimated above the smeared core stress (%.3g vs %.3g Pa, bound x%.1f)"), G.CoreMaxMicroVonMises, G.CoreMaxVonMises, G.Amplification));
						double Area = 0.0;
						for (const auto& T : Outer.triangles)
						{
							Area += 0.5 * aerotelic::Length(aerotelic::Cross(Outer.vertices[T[1]] - Outer.vertices[T[0]], Outer.vertices[T[2]] - Outer.vertices[T[0]]));
						}
						const double Volume = Outer.SignedVolume();
						const double ExpectedMass = 2780.0 * 0.05 * Area + Foam.density * G.RelativeDensity * (1.0 - Area * 0.05 / Volume) * Volume;
						Check(FMath::IsNearlyEqual(R.Mass, ExpectedMass, 1e-4 * ExpectedMass),
							FString::Printf(TEXT("fea (shells): the mass is the skin's rho t A plus the overlap-corrected core (%.6g vs %.6g)"), R.Mass, ExpectedMass));
						const double Weight = R.Mass * 9.80665;
						Check(FMath::Abs(R.Reaction[2] - Weight) <= 0.02 * Weight && FMath::Abs(R.Reaction[0]) <= 0.02 * Weight && FMath::Abs(R.Reaction[1]) <= 0.02 * Weight,
							FString::Printf(TEXT("fea (shells): the support carries the weight, %.4g N, shells included (reactions %.4g, %.4g, %.4g)"), Weight, R.Reaction[0], R.Reaction[1], R.Reaction[2]));
						Check(FMath::Abs(R.AppliedPressure[2]) <= 1e-3 * 1.0e5 * 4.0 * PI, FString::Printf(TEXT("fea (shells): a uniform pressure over a closed surface has no resultant (%.3g N)"), R.AppliedPressure[2]));
						Check(R.MeanNormalDisplacement < 0.0, FString::Printf(TEXT("fea (shells): an external pressure moves the surface inward (mean u.n = %.3g m)"), R.MeanNormalDisplacement));
						Check(G.SkinMaxVonMises > 0.8e6 && G.SkinMaxVonMises < 2.0e6, FString::Printf(TEXT("fea (shells): the skin's peak von Mises is the membrane stress p a / 2t, 1.0e6 Pa, up to the support (got %.4g)"), G.SkinMaxVonMises));
						Check(G.SkinMargin > 100.0 && G.CoreMargin != 0.0, FString::Printf(TEXT("fea (shells): margins against the skin's and the foam's allowables (%.1f, %.2f)"), G.SkinMargin, G.CoreMargin));
						Check(R.Entries.Num() == 2 && R.Entries[0].Kind == TEXT("skin") && R.Entries[1].Kind == TEXT("core"), "fea (shells): a skin entry and a core entry");
					}
					Check(R.MaxDisplacement > 0.0 && R.MaxDisplacement < 1e-3, FString::Printf(TEXT("fea (shells): the displacement is small and finite (%.3g m)"), R.MaxDisplacement));
					Check(IFileManager::Get().FileSize(*Request.VtkPath) > 0, FString::Printf(TEXT("fea (shells): the solution was written to %s"), *Request.VtkPath));
					IFileManager::Get().Delete(*Request.VtkPath, false, true, true);

					AerotelicFeaPreprocess::FModel Pre;
					FString PreError;
					const bool bPre = AerotelicFeaPreprocess::Build(Request, Pre, PreError);
					Check(bPre, FString::Printf(TEXT("fea (shells, pre-processor): the model derives (%s)"), *PreError));
					if (bPre)
					{
						Check(Pre.WettedFaces.Num() == R.Shells && Pre.WettedFaces.Num() == R.WettedFaces, FString::Printf(TEXT("fea (shells, pre-processor): a wetted face per shell (%d vs %d)"), Pre.WettedFaces.Num(), R.Shells));
						Check(Pre.SupportNodes.Num() == R.SupportCornerNodes, FString::Printf(TEXT("fea (shells, pre-processor): the fixed nodes are the script's (%d vs %d)"), Pre.SupportNodes.Num(), R.SupportCornerNodes));
						Check(Pre.SecondOrderTets == 0 && Pre.MidEdgeNodes == 0 && Pre.ShellNodes == static_cast<int32>(Outer.vertices.size()) && Pre.Dofs == R.Dofs && Pre.DirectSolverAuto(),
							FString::Printf(TEXT("fea (shells, pre-processor): the DOF estimate is the solver's, a 4-node core under 6-dof shell nodes, within the direct solver's budget (%lld vs %d)"), Pre.Dofs, R.Dofs));
						Check(Pre.Regions.Num() == 1 && !Pre.Regions[0].bSecondOrder, "fea (shells, pre-processor): the region is reported 4-node");
						Check(Pre.bMassEstimated && FMath::IsNearlyEqual(Pre.Mass, R.Mass, 0.05 * R.Mass), FString::Printf(TEXT("fea (shells, pre-processor): the estimated mass is near the script's (%.6g vs %.6g)"), Pre.Mass, R.Mass));
						Check(Pre.Regions.Num() == 1 && Pre.Regions[0].Label == TEXT("sphere") && Pre.Regions[0].Kind.Contains(TEXT("homogenised")), "fea (shells, pre-processor): one region, the body, shells over a homogenised core");
					}
				}
			}
		}

		// (3) A thin-walled body with no lattice: the sphere's own
		// triangles as shells of its 0.05 aluminium skin and nothing inside
		// -- what Analysis > Structural makes of any body with a skin
		// thickness. Closed forms: mass rho t A, the weight carried, the
		// membrane stress p a / 2t = 1.0 MPa, the surface moving inward.
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea (a thin-walled body)"));
		if (Primitive != nullptr)
		{
			const std::vector<aerotelic::NamedMesh> Placed = Primitive->GetPlacedMeshes(false);
			Check(Placed.size() == 1 && Placed[0].mesh.IsClosedManifold(), "fea (shells only): the sphere is one closed body");
			if (Placed.size() == 1)
			{
				const aerotelic::TriMesh& Surface = Placed[0].mesh;
				aerotelic::TetMesh ShellMesh;
				ShellMesh.nodes = Surface.vertices;
				AerotelicStructuralFea::FRequest Request;
				Request.Mesh = &ShellMesh;
				for (const auto& T : Surface.triangles)
				{
					AerotelicStructuralFea::FShellFace F;
					F.N[0] = T[0]; F.N[1] = T[1]; F.N[2] = T[2];
					F.Region = 1;
					Request.ShellFaces.Add(F);
				}
				AerotelicStructuralFea::FRegion Region;
				Region.Id = 1;
				Region.Label = TEXT("sphere");
				Region.Skin = Al;
				Region.SkinThickness = Primitive->GetEffectiveMassSpec().skin_thickness;
				Request.Regions.Add(Region);
				Request.bSecondOrder = true;   // no tetrahedra: ignored
				Support(Request);
				Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_fea_thin.vtk"));
				IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
				Check(FMath::IsNearlyEqual(Region.SkinThickness, 0.05, 1e-12), "fea (shells only): the skin thickness is the sphere's Material Properties figure");
				AerotelicStructuralFea::FResult R;
				FString Error;
				const bool bSolved = AerotelicStructuralFea::Run(Request, R, Error);
				Check(bSolved, FString::Printf(TEXT("fea (shells only): the thin-walled sphere solved (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d shells, %d dof, %.1f s"), R.Nodes, R.Shells, R.Dofs, R.Seconds) : *Error));
				if (bSolved)
				{
					Check(R.Tets == 0 && R.Shells == static_cast<int32>(Surface.triangles.size()) && !R.bSecondOrder && R.Dofs == 6 * R.Nodes, "fea (shells only): shells on every triangle and no tetrahedra, six degrees of freedom a node");
					const double ExpectedMass = 2780.0 * 0.05 * Surface.SurfaceArea();
					Check(FMath::IsNearlyEqual(R.Mass, ExpectedMass, 1e-6 * ExpectedMass), FString::Printf(TEXT("fea (shells only): the mass is rho t A (%.6g vs %.6g)"), R.Mass, ExpectedMass));
					const double Weight = ExpectedMass * 9.80665;
					Check(FMath::Abs(R.Reaction[2] - Weight) <= 0.02 * Weight && FMath::Abs(R.Reaction[0]) <= 0.02 * Weight && FMath::Abs(R.Reaction[1]) <= 0.02 * Weight,
						FString::Printf(TEXT("fea (shells only): the support carries the weight, %.4g N (reactions %.4g, %.4g, %.4g)"), Weight, R.Reaction[0], R.Reaction[1], R.Reaction[2]));
					Check(R.MeanNormalDisplacement < 0.0, FString::Printf(TEXT("fea (shells only): an external pressure moves the surface inward (mean u.n = %.3g m)"), R.MeanNormalDisplacement));
					Check(R.Entries.Num() == 1 && R.Entries[0].Kind == TEXT("skin") && R.Entries[0].MaxVonMises > 0.8e6 && R.Entries[0].MaxVonMises < 2.5e6,
						FString::Printf(TEXT("fea (shells only): the peak von Mises is the membrane stress p a / 2t, 1.0e6 Pa, up to the support (got %.4g)"), R.Entries.Num() ? R.Entries[0].MaxVonMises : 0.0));
					Check(R.MaxDisplacement > 0.0 && R.MaxDisplacement < 1e-3, FString::Printf(TEXT("fea (shells only): the displacement is small and finite (%.3g m)"), R.MaxDisplacement));
					IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
					AerotelicFeaPreprocess::FModel Pre;
					FString PreError;
					const bool bPre = AerotelicFeaPreprocess::Build(Request, Pre, PreError);
					Check(bPre && Pre.Tets == 0 && Pre.WettedFaces.Num() == R.WettedFaces && Pre.SupportNodes.Num() == R.SupportCornerNodes && Pre.Regions.Num() == 1 && Pre.Regions[0].Kind == TEXT("shells"),
						FString::Printf(TEXT("fea (shells only, pre-processor): the faces and the fixed nodes are the script's (%d / %d faces, %d / %d nodes; %s)"), bPre ? Pre.WettedFaces.Num() : -1, R.WettedFaces, bPre ? Pre.SupportNodes.Num() : -1, R.SupportCornerNodes, *PreError));
					Check(bPre && FMath::IsNearlyEqual(Pre.Mass, R.Mass, 1e-6 * R.Mass), "fea (shells only, pre-processor): the mass is rho t A");
				}
			}
		}

		// (4) A solid body with no lattice: the sphere meshed by gmsh at an
		// element size, tetrahedra of its aluminium, under its own weight.
		// Closed forms: mass rho V, the weight carried, a displacement of
		// the order rho g a^2 / E (a few hundred nanometres).
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea (a solid body)"));
		if (Primitive != nullptr)
		{
			const std::vector<aerotelic::NamedMesh> Placed = Primitive->GetPlacedMeshes(false, 2e-3);
			if (Placed.size() == 1)
			{
				AerotelicTetGmsh::FRequest G;
				G.Outer = &Placed[0].mesh;
				G.Inner = nullptr;
				G.CoreSize = 0.15;
				G.Algorithm = 1;
				G.bOptimize = true;
				G.bOptimizeNetgen = true;
				AerotelicTetGmsh::FResult TR;
				FString GmshError;
				const bool bMeshed = AerotelicTetGmsh::Run(G, TR, GmshError);
				Check(bMeshed && !TR.Mesh.tets.empty() && TR.VolumeError <= 1e-9, FString::Printf(TEXT("fea (solid): gmsh fills the sphere at an element size (%s)"), bMeshed ? *FString::Printf(TEXT("%d tetrahedra, relative volume error %.2e"), (int32)TR.Mesh.tets.size(), TR.VolumeError) : *GmshError));
				if (bMeshed && !TR.Mesh.tets.empty())
				{
					aerotelic::TetMesh SolidMesh = TR.Mesh;
					SolidMesh.material.assign(SolidMesh.tets.size(), 1);
					AerotelicStructuralFea::FRequest Request;
					Request.Mesh = &SolidMesh;
					AerotelicStructuralFea::FRegion Region;
					Region.Id = 1;
					Region.Label = TEXT("sphere");
					Region.Core = Al;
					Region.bSolid = true;
					Request.Regions.Add(Region);
					Request.bSecondOrder = true;
					Support(Request);
					Request.Pressure = 0.0;
					Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_fea_solid.vtk"));
					IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
					AerotelicStructuralFea::FResult R;
					FString Error;
					const bool bSolved = AerotelicStructuralFea::Run(Request, R, Error);
					Check(bSolved, FString::Printf(TEXT("fea (solid): the solid sphere solved (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d tetrahedra, %d dof, %.1f s"), R.Nodes, R.Tets, R.Dofs, R.Seconds) : *Error));
					if (bSolved)
					{
						const double ExpectedMass = 2780.0 * (TR.SkinVolume + TR.CoreVolume);
						Check(R.Shells == 0 && R.bSecondOrder && R.Dofs == 3 * R.Nodes, "fea (solid): 10-node tetrahedra and no shells, three degrees of freedom a node");
						{
							AerotelicFeaPreprocess::FModel PreCost;
							FString PreCostError;
							Check(AerotelicFeaPreprocess::Build(Request, PreCost, PreCostError) && PreCost.ShellNodes == 0 && PreCost.SecondOrderTets == PreCost.Tets && PreCost.Dofs == R.Dofs,
								FString::Printf(TEXT("fea (solid, pre-processor): the DOF estimate counts the mid-edge nodes of every tetrahedron (%lld vs %d)"), PreCost.Dofs, R.Dofs));
						}
						Check(FMath::IsNearlyEqual(R.Mass, ExpectedMass, 1e-6 * ExpectedMass), FString::Printf(TEXT("fea (solid): the mass is rho V (%.6g vs %.6g)"), R.Mass, ExpectedMass));
						const double Weight = ExpectedMass * 9.80665;
						Check(FMath::Abs(R.Reaction[2] - Weight) <= 0.02 * Weight && FMath::Abs(R.Reaction[0]) <= 0.02 * Weight && FMath::Abs(R.Reaction[1]) <= 0.02 * Weight,
							FString::Printf(TEXT("fea (solid): the support carries the weight, %.4g N (reactions %.4g, %.4g, %.4g)"), Weight, R.Reaction[0], R.Reaction[1], R.Reaction[2]));
						Check(R.MaxDisplacement > 0.0 && R.MaxDisplacement < 1e-5, FString::Printf(TEXT("fea (solid): the displacement under its own weight is of the order rho g a^2 / E (%.3g m)"), R.MaxDisplacement));
						Check(R.Entries.Num() == 1 && R.Entries[0].Kind == TEXT("solid") && R.Entries[0].MaxVonMises > 0.0 && R.Entries[0].MaxVonMises < 1.0e6,
							FString::Printf(TEXT("fea (solid): the stress under its own weight is of the order rho g a (%.4g Pa)"), R.Entries.Num() ? R.Entries[0].MaxVonMises : 0.0));
						IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
						AerotelicFeaPreprocess::FModel Pre;
						FString PreError;
						const bool bPre = AerotelicFeaPreprocess::Build(Request, Pre, PreError);
						Check(bPre && Pre.WettedFaces.Num() == R.WettedFaces && Pre.SupportNodes.Num() == R.SupportCornerNodes && R.SupportNodes > R.SupportCornerNodes && Pre.Regions.Num() == 1 && Pre.Regions[0].Kind == TEXT("solid") && FMath::IsNearlyEqual(Pre.Mass, R.Mass, 1e-6 * R.Mass),
							FString::Printf(TEXT("fea (solid, pre-processor): the faces, the fixed nodes and the mass are the script's (%s)"), *PreError));
					}
				}
			}
		}
	}

	// (5) A box primitive as a thin-walled cantilever against beam theory
	// (2026-11-02): the 1 x 10 x 1 m box of ue/Projects/simple_beam.yaml,
	// 10 mm aluminium shells, its grid 2 x 20 x 2 cells from the Mesh tab,
	// clamped at y = min, two 500 N loads at the tip on the webs' mid-height.
	// Timoshenko with the shells' mid-surface on the outer surface, I =
	// 2 (b t (h/2)^2 + t h^3 / 12), the two webs the shear area: the linear
	// thin shell lands within 3% on this grid (the beam study that found the
	// box was 12 triangles and the solid route ignored the element size).
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea (a box beam)"));
	if (!AerotelicStructuralFea::FindScript().IsEmpty() && !AerotelicMeshLib::FindPython().IsEmpty())
	{
		AAerotelicEntityActor* Beam = Doc->AddPrimitive(TEXT("Box"));
		Check(Beam != nullptr, "fea (beam): a box primitive is added");
		if (Beam != nullptr)
		{
			std::unique_ptr<aerotelic::ISolidModel> Draft = Doc->CloneEntityModel(Beam);
			aerotelic::PrimitiveModel* D = static_cast<aerotelic::PrimitiveModel*>(Draft.get());
			D->size = {1.0, 10.0, 1.0};
			D->mesh.cells = {2, 20, 2};
			FString Error;
			Check(Doc->ApplyEntityModel(Beam, *Draft, Error), FString::Printf(TEXT("fea (beam): the box's size and grid apply (%s)"), *Error));
			Doc->FlushPendingRebuilds();
			Check(Beam->GetModel() != nullptr && Beam->GetModel()->mesh.cells == std::array<int, 3>{2, 20, 2}, "fea (beam): the entity took the draft's grid (the Mesh tab's cells travel with Apply)");
			std::unique_ptr<aerotelic::ISolidModel> Again = Doc->CloneEntityModel(Beam);
			Check(Again != nullptr && Again->mesh.cells == std::array<int, 3>{2, 20, 2}, "fea (beam): a fresh draft starts on the entity's grid");
			const aerotelic::Material Al = *aerotelic::FindMaterialPreset("Aluminium 2024-T3");
			aerotelic::MassSpec Skin;
			Skin.mode = aerotelic::MassMode::Material;
			Skin.material = Al;
			Skin.skin_thickness = 0.01;
			Beam->SetMassSpec(Skin);
			const std::vector<aerotelic::NamedMesh> Placed = Beam->GetPlacedMeshes(false, 2e-3);
			const int32 ExpectedTriangles = 2 * 2 * (2 * 20 + 20 * 2 + 2 * 2);
			Check(Placed.size() == 1 && static_cast<int32>(Placed[0].mesh.triangles.size()) == ExpectedTriangles,
				FString::Printf(TEXT("fea (beam): the box is its 2 x 20 x 2 cells, %d triangles (got %d)"), ExpectedTriangles, Placed.size() == 1 ? (int32)Placed[0].mesh.triangles.size() : -1));
			if (Placed.size() == 1)
			{
				const aerotelic::TriMesh& Surface = Placed[0].mesh;
				aerotelic::TetMesh ShellMesh;
				ShellMesh.nodes = Surface.vertices;
				AerotelicStructuralFea::FRequest Request;
				Request.Mesh = &ShellMesh;
				for (const auto& T : Surface.triangles)
				{
					AerotelicStructuralFea::FShellFace F;
					F.N[0] = T[0]; F.N[1] = T[1]; F.N[2] = T[2];
					F.Region = 1;
					Request.ShellFaces.Add(F);
				}
				AerotelicStructuralFea::FRegion Region;
				Region.Id = 1;
				Region.Label = TEXT("beam");
				Region.Skin = Al;
				Region.SkinThickness = 0.01;
				Request.Regions.Add(Region);
				Request.LengthToSI = 1.0;
				Request.LinearSolver = TEXT("sparse_lu");
				AerotelicStructuralFea::FSupport Root;
				Root.Axis = TEXT('y');
				Root.bAtMin = true;
				Request.Supports.Add(Root);
				const double P = 1000.0;
				for (double Side : { -0.5, 0.5 })
				{
					AerotelicStructuralFea::FPointForce F;
					F.At = aerotelic::Vec3{ Side, 5.0, 0.0 };
					F.Force = aerotelic::Vec3{ 0.0, 0.0, -0.5 * P };
					Request.PointForces.Add(F);
				}
				Request.Modes = 4;
				Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_fea_beam.vtk"));
				IFileManager::Get().Delete(*Request.VtkPath, false, true, true);

				// The pre-processor: the default tolerance takes the root face's
				// 9 nodes and nothing more; a coarse one takes the next ring and
				// is flagged.
				AerotelicFeaPreprocess::FModel Pre;
				FString PreError;
				const bool bPre = AerotelicFeaPreprocess::Build(Request, Pre, PreError);
				Check(bPre && Pre.SupportPlanes.Num() == 1 && Pre.SupportPlanes[0].Nodes == 9 && Pre.SupportPlanes[0].Furthest == 0.0 && !Pre.ToleranceIsCoarse(Pre.SupportPlanes[0]),
					FString::Printf(TEXT("fea (beam, pre-processor): the default tolerance clamps the root face's 9 nodes and is not coarse (%d nodes, furthest %.3g, median edge %.3g; %s)"), bPre ? Pre.SupportPlanes[0].Nodes : -1, bPre ? Pre.SupportPlanes[0].Furthest : -1.0, bPre ? Pre.MedianEdge : -1.0, *PreError));
				Check(bPre && Pre.PointLoads.Num() == 2 && Pre.PointLoads[0].At.Y == 5.0 && Pre.PointLoads[1].At.Y == 5.0 && Pre.PointLoads[0].At.Z == 0.0 && Pre.PointResultant.Z == -P,
					"fea (beam, pre-processor): both loads land on the webs' mid-height nodes at the tip");
				{
					AerotelicStructuralFea::FRequest Coarse = Request;
					Coarse.Supports[0].Tolerance = 0.6;
					AerotelicFeaPreprocess::FModel PreCoarse;
					const bool bCoarse = AerotelicFeaPreprocess::Build(Coarse, PreCoarse, PreError);
					// The root face's 9 nodes plus the next ring's 8 (the cap centre has no counterpart there).
					Check(bCoarse && PreCoarse.SupportPlanes[0].Nodes == 17 && PreCoarse.ToleranceIsCoarse(PreCoarse.SupportPlanes[0]) && FMath::IsNearlyEqual(PreCoarse.SupportPlanes[0].Furthest, 0.5, 1e-9),
						FString::Printf(TEXT("fea (beam, pre-processor): a 0.6 tolerance takes the next ring too (%d nodes, furthest %.3g) and is flagged as coarse"), bCoarse ? PreCoarse.SupportPlanes[0].Nodes : -1, bCoarse ? PreCoarse.SupportPlanes[0].Furthest : -1.0));
				}
				// A point force typed off the geometry snaps to the nearest
				// node and is flagged: 3 units above the top face lands on it,
				// 2.5 away, more than the 0.5 edges; the two tip loads sit on
				// their nodes exactly and are not.
				Check(bPre && Pre.PointLoads.Num() == 2 && Pre.PointLoads[0].SnapDistance == 0.0 && !Pre.SnapIsFar(Pre.PointLoads[0]) && !Pre.SnapIsFar(Pre.PointLoads[1]),
					"fea (beam, pre-processor): the tip loads land on their nodes and are not flagged");
				{
					AerotelicStructuralFea::FRequest Off = Request;
					Off.PointForces[0].At = aerotelic::Vec3{ 0.5, 5.0, 3.0 };
					AerotelicFeaPreprocess::FModel PreOff;
					const bool bOff = AerotelicFeaPreprocess::Build(Off, PreOff, PreError);
					Check(bOff && PreOff.PointLoads.Num() == 2 && FMath::IsNearlyEqual(PreOff.PointLoads[0].SnapDistance, 2.5, 1e-9) && PreOff.SnapIsFar(PreOff.PointLoads[0])
						&& PreOff.PointLoads[0].Asked.Z == 3.0 && PreOff.PointLoads[0].At.Z == 0.5,
						FString::Printf(TEXT("fea (beam, pre-processor): a point 3 above the top face snaps 2.5 to it and is flagged as far (snap %.3g, median edge %.3g)"), bOff ? PreOff.PointLoads[0].SnapDistance : -1.0, bOff ? PreOff.MedianEdge : -1.0));
				}
				// "At the mesh's minimum" of one body: the beam is body 0, so
				// asking for body 0 gives the same 9 nodes; asking for a body
				// the model has no nodes of falls back to the whole mesh and
				// says so.
				{
					AerotelicStructuralFea::FRequest PerBody = Request;
					PerBody.Regions[0].Body = 0;
					PerBody.Supports[0].Body = 0;
					AerotelicFeaPreprocess::FModel PreBody;
					const bool bBody = AerotelicFeaPreprocess::Build(PerBody, PreBody, PreError);
					Check(bBody && PreBody.SupportPlanes.Num() == 1 && PreBody.SupportPlanes[0].Nodes == 9 && PreBody.SupportPlanes[0].Body == 0 && !PreBody.SupportPlanes[0].bBodyFallback
						&& PreBody.SupportPlanes[0].Value == -5.0,
						FString::Printf(TEXT("fea (beam, pre-processor): the minimum of body 0 alone is the beam's root, 9 nodes at y = %.3g"), bBody ? PreBody.SupportPlanes[0].Value : 0.0));
					PerBody.Supports[0].Body = 7;
					const bool bNone = AerotelicFeaPreprocess::Build(PerBody, PreBody, PreError);
					Check(bNone && PreBody.SupportPlanes[0].Nodes == 9 && PreBody.SupportPlanes[0].bBodyFallback,
						"fea (beam, pre-processor): a body with no nodes in the model falls back to the whole mesh's minimum and is flagged");
				}

				AerotelicStructuralFea::FResult R;
				const bool bSolved = AerotelicStructuralFea::Run(Request, R, Error);
				Check(bSolved, FString::Printf(TEXT("fea (beam): the box beam solved (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d shells, %d dof, %.1f s"), R.Nodes, R.Shells, R.Dofs, R.Seconds) : *Error));
				if (bSolved)
				{
					const double E = Al.youngs_modulus, G = E / (2.0 * (1.0 + Al.poisson_ratio));
					const double I = 2.0 * (1.0 * 0.01 * 0.25 + 0.01 * 1.0 / 12.0);
					const double Delta = P * 1000.0 / (3.0 * E * I) + P * 10.0 / (G * 2.0 * 1.0 * 0.01);
					Check(R.bBalanced && FMath::Abs(R.Reaction[2] - P) <= 0.01 * P && FMath::Abs(R.Reaction[0]) <= 0.01 * P && FMath::Abs(R.Reaction[1]) <= 0.01 * P,
						FString::Printf(TEXT("fea (beam): the clamp carries the two tip loads, %.4g N (reactions %.4g, %.4g, %.4g)"), P, R.Reaction[0], R.Reaction[1], R.Reaction[2]));
					Check(R.Shells == ExpectedTriangles && R.SupportNodes == 9 && R.SupportCornerNodes == 9, FString::Printf(TEXT("fea (beam): a shell per triangle, the 9 root nodes clamped (%d shells, %d fixed)"), R.Shells, R.SupportNodes));
					Check(FMath::Abs(R.MaxDisplacement / Delta - 1.0) < 0.03 && R.MaxDisplacementAt[1] > 4.9,
						FString::Printf(TEXT("fea (beam): the tip deflects as Timoshenko's thin-walled box, %.4g m (got %.4g at y = %.2f, %.1f%%)"), Delta, R.MaxDisplacement, R.MaxDisplacementAt[1], 100.0 * (R.MaxDisplacement / Delta - 1.0)));
					const double SigmaRoot = P * 10.0 * 0.5 / I;
					Check(R.Entries.Num() == 1 && R.Entries[0].MaxVonMises > 0.9 * SigmaRoot && R.Entries[0].MaxVonMises < 2.0 * SigmaRoot,
						FString::Printf(TEXT("fea (beam): the peak von Mises is the root bending stress M c / I, %.4g Pa, up to the clamp's concentration (got %.4g)"), SigmaRoot, R.Entries.Num() ? R.Entries[0].MaxVonMises : 0.0));
					// The natural frequencies (2026-11-04): a square box's first
					// bending pair is degenerate, at the clamped-free beam's
					// beta^2 / (2 pi L^2) sqrt(EI / rho A) lowered for the webs'
					// shear flexibility and rotary inertia (Timoshenko, first
					// order) and for the tip cap's mass against the mode's
					// effective mass, a quarter of the beam's -- the figure
					// structural_fea_check.py's cantilever_f1 computes.
					{
						const double Rho = Al.density, A = 4.0 * 1.0 * 0.01, As = 2.0 * 1.0 * 0.01, Lb = 10.0;
						const double Beta2 = 1.8751 * 1.8751;
						double F1 = Beta2 / (2.0 * PI * Lb * Lb) * FMath::Sqrt(E * I / (Rho * A));
						F1 /= FMath::Sqrt(1.0 + Beta2 * I / (A * Lb * Lb) * (1.0 + E * A / (G * As)));
						const double CapMass = Rho * 0.01 * 1.0 * 1.0, EffectiveMass = 0.25 * Rho * A * Lb;
						F1 *= FMath::Sqrt(EffectiveMass / (EffectiveMass + CapMass));
						const TArray<double>& F = R.NaturalFrequencies;
						Check(F.Num() == 4 && !R.Eigensolver.IsEmpty() && F[0] > 0.0 && F[1] >= F[0] && F[2] >= F[1] && F[3] >= F[2],
							FString::Printf(TEXT("fea (beam): four natural frequencies came back ascending from %s in %.2f s (%d)"), *R.Eigensolver, R.SecondsModes, F.Num()));
						Check(F.Num() >= 2 && FMath::Abs(F[1] / F[0] - 1.0) < 0.01,
							FString::Printf(TEXT("fea (beam): the first bending pair is degenerate (%.4g and %.4g Hz)"), F.Num() >= 2 ? F[0] : 0.0, F.Num() >= 2 ? F[1] : 0.0));
						Check(F.Num() >= 1 && FMath::Abs(F[0] / F1 - 1.0) < 0.05,
							FString::Printf(TEXT("fea (beam): the first bending frequency is the clamped-free beam's with shear, rotary inertia and the tip cap, %.4g Hz (got %.4g, %.1f%%)"), F1, F.Num() ? F[0] : 0.0, F.Num() ? 100.0 * (F[0] / F1 - 1.0) : 0.0));
						FString Vtk;
						Check(FFileHelper::LoadFileToString(Vtk, *Request.VtkPath) && Vtk.Contains(TEXT("VECTORS mode_1 double")) && Vtk.Contains(TEXT("VECTORS mode_4 double")),
							"fea (beam): the VTK carries the four mode shapes as point vectors");
					}
					IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
				}
			}
			// The scene as the later steps expect it.
			Doc->RemoveEntity(Beam);
			Doc->FlushPendingRebuilds();
		}
	}

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: meshes"));
	for (auto& WeakEntity : Doc->GetEntities())
	{
		if (!WeakEntity.IsValid() || WeakEntity->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean) continue;
		std::vector<aerotelic::NamedMesh> Meshes = WeakEntity->GetPlacedMeshes(false, 2e-3);
		bool bOk = !Meshes.empty();
		for (const aerotelic::NamedMesh& M : Meshes) bOk = bOk && M.mesh.IsClosedManifold() && M.mesh.SignedVolume() > 0.0;
		Check(bOk, FString::Printf(TEXT("%s: placed meshes are closed, outward bodies (%d)"), *WeakEntity->EntityName, (int32)Meshes.size()));
	}

	// Potential flow (Analysis -> Potential Flow (Kratos)) over the
	// sphere primitive, where the answer is known: the air around it is
	// meshed by the bodies' own triangles and the flow solved on Kratos.
	// The exact pressures are checked against the closed-form sphere
	// solution by tools/potential_flow_check.py, outside the application
	// and at resolutions worth the time; what is asserted here is that the
	// application's own path produces that solution at all, and the
	// invariants that hold at any resolution.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: potential flow"));
	if (AerotelicPotentialFlow::FindScript().IsEmpty() || AerotelicMeshLib::FindPython().IsEmpty())
	{
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("skipped: tools\\potential_flow.py or the WingDesigner environment is not here"));
	}
	else if (Primitive != nullptr)
	{
		std::vector<aerotelic::NamedMesh> Sphere = Primitive->GetPlacedMeshes(false, 1e-2);
		AerotelicPotentialFlow::FRequest Request;
		if (!Sphere.empty())
		{
			Request.Bodies.Add({ Primitive->EntityName, &Sphere[0].mesh });
		}
		Request.Speed = 50.0;
		// A smaller box than the dialog's default: the sphere is a tenth
		// of the detail of a real model and this runs on every test pass.
		Request.FarFieldFactor = 3.0;
		Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_flow.vtk"));
		IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
		AerotelicPotentialFlow::FResult R;
		FString Error;
		const bool bSolved = AerotelicPotentialFlow::Run(Request, R, Error);
		Check(bSolved, FString::Printf(TEXT("the potential flow around the sphere solved (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d tetrahedra, %.1f s"), R.Nodes, R.Tets, R.Seconds) : *Error));
		if (bSolved)
		{
			// The wetted boundary of the fluid mesh is the body's own
			// surface: it encloses the body's own volume, and the
			// tetrahedra fill the box minus the body.
			const double Body = Sphere[0].mesh.SignedVolume();
			Check(FMath::Abs(R.WettedVolume - Body) <= 1e-9 * Body,
				FString::Printf(TEXT("the wetted boundary encloses the body's volume (%.9g vs %.9g)"), R.WettedVolume, Body));
			Check(FMath::Abs(R.FluidVolume - (R.BoxVolume - R.BodyVolume)) <= 1e-9 * R.BoxVolume,
				FString::Printf(TEXT("the tetrahedra fill the box minus the body (%.9g vs %.9g)"), R.FluidVolume, R.BoxVolume - R.BodyVolume));
			Check(R.WettedTriangles == static_cast<int32>(Sphere[0].mesh.triangles.size()),
				FString::Printf(TEXT("every one of the body's triangles is a face of the mesh (%d of %d)"), R.WettedTriangles, (int32)Sphere[0].mesh.triangles.size()));
			// The flow speeds up round the sphere, comes to rest fore and
			// aft (Cp = 1) and accelerates to 1.5 U at the equator
			// (Cp = -1.25); a coarse surface does not quite reach either.
			Check(R.MaxSpeed > 1.2 * R.FreeStreamSpeed, FString::Printf(TEXT("the flow speeds up round the sphere (%.4g against the free stream %.4g)"), R.MaxSpeed, R.FreeStreamSpeed));
			Check(R.MaxCpBody > 0.5 && R.MaxCpBody < 1.05, FString::Printf(TEXT("the flow comes to rest on the nose (Cp %.4f, exactly 1)"), R.MaxCpBody));
			Check(R.MinCpBody < -0.5 && R.MinCpBody > -1.5, FString::Printf(TEXT("the suction peak is round the equator (Cp %.4f, exactly -1.25)"), R.MinCpBody));
			// d'Alembert's paradox: a potential flow carries no force.
			Check(R.ForceCoefficient < 0.1, FString::Printf(TEXT("the net force is zero to the discretisation (%.2e of q x frontal area)"), R.ForceCoefficient));
			Check(IFileManager::Get().FileSize(*Request.VtkPath) > 0, FString::Printf(TEXT("the solution was written to %s"), *Request.VtkPath));
			IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
		}
	}

	// Geometry > Units: set before the save, read back after the reopen.
	{
		aerotelic::UnitSystem U;
		U.length = aerotelic::LengthUnit::Foot;
		U.mass = aerotelic::MassUnit::PoundMass;
		Doc->SetUnits(U);
		Check(Doc->GetUnits().length == aerotelic::LengthUnit::Foot, "units: set on the document");
	}

	// Geometry > Scale: a 1:4 replica. The parameterisations and the
	// placements stay the full-scale aircraft; the actors, the placed
	// meshes, bounds and solids are a quarter the size and a quarter as far
	// out; a skin's mass goes as the area (1/16) while its thickness stays;
	// the boolean and the lattice follow. Set before the save, read back
	// after the reopen.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: scale"));
	double FullSpan = 0.0, FullMass = 0.0;
	FBox FullBounds(ForceInit);
	aerotelic::Placement FullPlacement{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0};
	if (Wing != nullptr && Primitive != nullptr)
	{
		Check(Doc->GetModelScale() == 1.0, "scale: a project starts at full size");
		Check(!Doc->SetModelScale(0.0) && !Doc->SetModelScale(-1.0) && Doc->GetModelScale() == 1.0, "scale: zero and negative are refused and change nothing");
		FullSpan = Wing->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan();
		// The wing sits where the placement step left it (10, 20, 30).
		FullPlacement = Wing->GetPlacement();
		FullBounds = Wing->GetPlacedBounds(false);
		FAerotelicBooleanOperand P;
		P.Entity = Primitive;
		FullMass = ComputeAerotelicMassReport({ P }, 2e-3, Doc->GetUnits()).Assembly.mass;
		Doc->FlushPendingRebuilds();

		Check(Doc->SetModelScale(0.25) && Doc->GetModelScale() == 0.25, "scale: set to 1:4");
		Check(FMath::IsNearlyEqual(Wing->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan(), FullSpan, 1e-12), "scale: the wing's parameterisation is untouched");
		const aerotelic::Placement Kept = Wing->GetPlacement();
		Check(Kept.translation[0] == FullPlacement.translation[0] && Kept.translation[2] == FullPlacement.translation[2], "scale: the authored placement is untouched (full-scale)");
		Check(Wing->GetActorScale3D().Equals(FVector(0.25), 1e-9), "scale: the actor carries the scale");
		const FVector FullAt(FullPlacement.translation[0], FullPlacement.translation[1], FullPlacement.translation[2]);
		Check(Wing->GetActorLocation().Equals(FullAt * 0.25, 1e-9), FString::Printf(TEXT("scale: the actor sits a quarter as far out (%s)"), *Wing->GetActorLocation().ToString()));
		const aerotelic::EntityTransform T = Wing->GetEntityTransform();
		Check(T.scale == 0.25 && T.tx == FullPlacement.translation[0], "scale: the feature-tree transform carries the scale with the full-scale translation");
		// The bounds come off meshes at the display tolerance (the replica
		// is meshed that much finer in model space), so to the chordal error.
		const FBox Quarter = Wing->GetPlacedBounds(false);
		const double Slack = 3e-3 * FullBounds.GetSize().GetMax();
		Check(Quarter.IsValid && FullBounds.IsValid && Quarter.GetSize().Equals(FullBounds.GetSize() * 0.25, Slack)
			&& Quarter.GetCenter().Equals(FullBounds.GetCenter() * 0.25, Slack),
			FString::Printf(TEXT("scale: the placed bounds are the replica's (%s at %s vs full %s at %s)"), *Quarter.GetSize().ToString(), *Quarter.GetCenter().ToString(), *FullBounds.GetSize().ToString(), *FullBounds.GetCenter().ToString()));
		{
			FBox MeshBox(ForceInit);
			for (const aerotelic::NamedMesh& M : Wing->GetPlacedMeshes(false, 2e-3)) MeshBox += AerotelicMeshing::BoundsOf(M.mesh);
			Check(MeshBox.IsValid && MeshBox.GetSize().Equals(FullBounds.GetSize() * 0.25, 2e-3 * FullBounds.GetSize().GetMax()),
				FString::Printf(TEXT("scale: the placed meshes are the replica's (%s)"), *MeshBox.GetSize().ToString()));
		}
		{
			// The OCCT path: the placed solid, a quarter the size.
			Bnd_Box B;
			BRepBndLib::AddOptimal(Wing->GetPlacedShape(false), B);
			double X0, Y0, Z0, X1, Y1, Z1;
			B.Get(X0, Y0, Z0, X1, Y1, Z1);
			Check(!B.IsVoid() && FMath::IsNearlyEqual(Y1 - Y0, FullBounds.GetSize().Y * 0.25, 2e-3 * FullBounds.GetSize().Y),
				FString::Printf(TEXT("scale: the placed solid is the replica's (span %.4f vs full %.4f)"), Y1 - Y0, FullBounds.GetSize().Y));
		}
		{
			const FAerotelicMassReport R = ComputeAerotelicMassReport({ P }, 2e-3, Doc->GetUnits());
			const aerotelic::MassSpec Spec = Primitive->GetMassSpec();
			Check(Spec.IsSkin(), "scale: the sphere weighs as a skin (set in the mass step)");
			Check(FullMass > 0.0 && FMath::IsNearlyEqual(R.Assembly.mass, FullMass / 16.0, 1e-2 * FullMass / 16.0),
				FString::Printf(TEXT("scale: a skin's mass goes as the area, 1/16 (%.5g vs %.5g full), its thickness as typed"), R.Assembly.mass, FullMass));
		}
		Check(Doc->HasPendingRebuilds(), "scale: the boolean and the lattice over the wing and sphere are queued to follow");
		Doc->FlushPendingRebuilds();
		Check(!Doc->HasPendingRebuilds(), "scale: ... and rebuilt");
	}

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: save"));
	const FString TmpPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test.yaml"));
	Check(Doc->SaveProjectToPath(TmpPath), "save succeeded");
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: save done"));

	// Clear via targeted removal (no confirmation dialog) rather than
	// NewProject(), which would pop a blocking "replace the scene?" modal
	// with no user present to answer it in a headless commandlet run.
	TArray<TWeakObjectPtr<AAerotelicEntityActor>> ToRemove = Doc->GetEntities();
	for (auto& E : ToRemove)
	{
		if (E.IsValid())
		{
			Doc->RemoveEntity(E.Get());
		}
	}
	Check(Doc->GetEntities().Num() == 0, "scene cleared via RemoveEntity before reopening");

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: reopen"));
	// Entities.Num() == 0 here, so OpenProjectPath's confirmation check
	// short-circuits without showing a dialog.
	Check(Doc->OpenProjectPath(TmpPath), "reopen succeeded");
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: reopen done"));
	Check(Doc->GetEntities().Num() == 5, "reopened scene has 3 entities, the boolean result and the lattice");
	Check(Doc->GetUnits().length == aerotelic::LengthUnit::Foot && Doc->GetUnits().mass == aerotelic::MassUnit::PoundMass,
		"units: feet and pounds came back with the project");
	{
		FString Saved;
		FFileHelper::LoadFileToString(Saved, *TmpPath);
		Check(Saved.Contains(TEXT("units:")) && Saved.Contains(TEXT("length: ft")) && Saved.Contains(TEXT("mass: lbm")) && Saved.Contains(TEXT("time: s")),
			"units: the project file carries them, time included");
		Check(Saved.Contains(TEXT("scale: 0.25")), "scale: the project file carries the model scale");
	}
	// The replica came back as one: the full-scale parameterisation and
	// placement in the file, the actors a quarter the size.
	Check(Doc->GetModelScale() == 0.25, "scale: 1:4 came back with the project");
	for (auto& Weak : Doc->GetEntities())
	{
		if (!Weak.IsValid() || Weak->GetEntityType() != aerotelic::project_yaml::EntityType::Wing) continue;
		Check(FMath::IsNearlyEqual(Weak->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan(), FullSpan, 1e-9), "scale: the reopened wing is the full-scale parameterisation");
		Check(FMath::IsNearlyEqual(Weak->GetPlacement().translation[0], FullPlacement.translation[0], 1e-9), "scale: the reopened placement is the full-scale one");
		Check(Weak->GetActorScale3D().Equals(FVector(0.25), 1e-9)
			&& Weak->GetActorLocation().Equals(FVector(FullPlacement.translation[0], FullPlacement.translation[1], FullPlacement.translation[2]) * 0.25, 1e-6),
			"scale: the reopened actor is the replica");
	}
	Check(Doc->SetModelScale(1.0) && Doc->GetModelScale() == 1.0, "scale: back to full size for the rest of the run");
	Doc->FlushPendingRebuilds();

	// Copy refuses the derived kinds, while they are still in the scene: a
	// boolean result and a lattice are solids built FROM other entities,
	// with no parameterisation of their own to hand a constructor.
	{
		AAerotelicEntityActor* AnyBoolean = nullptr;
		AAerotelicEntityActor* AnyLattice = nullptr;
		AAerotelicEntityActor* AnyWing = nullptr;
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid()) continue;
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean) AnyBoolean = Weak.Get();
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Lattice) AnyLattice = Weak.Get();
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Wing) AnyWing = Weak.Get();
		}
		Check(!UAerotelicDocumentSubsystem::CanCopyEntity(AnyBoolean), "copy: a boolean result cannot be copied");
		Check(!UAerotelicDocumentSubsystem::CanCopyEntity(AnyLattice), "copy: a lattice cannot be copied");
		Check(!UAerotelicDocumentSubsystem::CanCopyEntity(nullptr), "copy: nothing selected cannot be copied");
		Check(UAerotelicDocumentSubsystem::CanCopyEntity(AnyWing), "copy: a wing can be copied");
		// A refused copy must leave whatever was on the clipboard alone
		// rather than emptying it.
		Check(Doc->CopyEntityToClipboard(AnyWing) && Doc->HasClipboardEntity(), "copy: the wing went onto the clipboard");
		Check(!Doc->CopyEntityToClipboard(AnyBoolean), "copy: copying the boolean result returns false");
		Check(Doc->HasClipboardEntity() && AnyWing != nullptr && Doc->GetClipboardEntityName() == AnyWing->EntityName,
			"copy: the refused copy left the wing on the clipboard untouched");
	}

	bool bFoundWing = false, bFoundFuselage = false, bFoundPrimitive = false, bFoundBoolean = false, bFoundLattice = false;
	AAerotelicEntityActor* ReopenedFuselage = nullptr;
	AAerotelicEntityActor* ReopenedPrimitive = nullptr;
	for (auto& WeakEntity : Doc->GetEntities())
	{
		if (!WeakEntity.IsValid())
		{
			continue;
		}
		AAerotelicEntityActor* E = WeakEntity.Get();
		switch (E->GetEntityType())
		{
		case aerotelic::project_yaml::EntityType::Wing:
			bFoundWing = true;
			Check(E->GetActorLocation().Equals(FVector(10.0, 20.0, 30.0), 1e-6), "wing placement round-tripped");
			break;
		case aerotelic::project_yaml::EntityType::Fuselage:
			bFoundFuselage = true;
			ReopenedFuselage = E;
			break;
		case aerotelic::project_yaml::EntityType::Primitive:
			bFoundPrimitive = true;
			ReopenedPrimitive = E;
			break;
		case aerotelic::project_yaml::EntityType::Boolean:
		{
			bFoundBoolean = true;
			Check(E->EntityName == TEXT("BoolOp 1"), "boolean result kept its name");
			const FAerotelicBooleanModel* M = static_cast<const FAerotelicBooleanModel*>(E->GetModel());
			Check(M->Operation == "fuse" && M->Operands.Num() == 2, "boolean operation and operands round-tripped");
			// A boolean result is a mesh (the mesh boolean), not a solid.
			try
			{
				std::vector<aerotelic::NamedMesh> Meshes = M->BuildMeshes(FAerotelicBooleanModel::kValidationTolerance);
				Check(Meshes.size() == 1 && Meshes[0].mesh.IsClosedManifold() && Meshes[0].mesh.SignedVolume() > 0.0, "BoolOp 1: the fused mesh is one closed, outward body");
			}
			catch (const std::exception& Ex)
			{
				Check(false, FString::Printf(TEXT("BoolOp 1: BuildMeshes threw: %s"), UTF8_TO_TCHAR(Ex.what())));
			}
			break;
		}
		case aerotelic::project_yaml::EntityType::Lattice:
		{
			bFoundLattice = true;
			Check(E->EntityName == TEXT("Lattice 1"), "lattice kept its name");
			const FAerotelicLatticeModel* M = static_cast<const FAerotelicLatticeModel*>(E->GetModel());
			Check(M->Spec.type == "Gyroid" && M->Spec.resolution == 6 && FMath::IsNearlyEqual(M->Spec.cell_size, 0.5) && FMath::IsNearlyEqual(M->Spec.volume_fraction, 0.35),
				"lattice parameters round-tripped");
			Check(M->Targets.Num() == 1 && M->Targets[0].Entity.IsValid() && M->Targets[0].Entity->GetEntityType() == aerotelic::project_yaml::EntityType::Primitive,
				"lattice target resolved to the reopened sphere");
			Check(M->IsValid() && M->GetStats().Triangles > 0, "lattice regenerated on load");
			break;
		}
		default:
			break;
		}
	}
	Check(bFoundWing && bFoundFuselage && bFoundPrimitive && bFoundBoolean && bFoundLattice, "all three entity types, the boolean and the lattice survived the round trip");

	// Deleting an operand cascades to the result built on it.
	// The mass settings set before the save came back with their entities.
	if (ReopenedFuselage != nullptr)
	{
		const aerotelic::MassSpec& M = ReopenedFuselage->GetMassSpec();
		Check(M.mode == aerotelic::MassMode::Asserted && FMath::IsNearlyEqual(M.mass, 150.0, 1e-12),
			"mass: the fuselage's asserted mass round-tripped through the project file");
	}
	if (ReopenedPrimitive != nullptr)
	{
		const aerotelic::MassSpec& M = ReopenedPrimitive->GetMassSpec();
		Check(M.mode == aerotelic::MassMode::Material && FMath::IsNearlyEqual(M.material.density, 2.0, 1e-12) && M.material.name == "Custom",
			"mass: the primitive's material round-tripped through the project file");
		Check(M.IsSkin() && FMath::IsNearlyEqual(M.skin_thickness, 0.05, 1e-12),
			"mass: the primitive's skin thickness round-tripped through the project file");
	}

	if (ReopenedFuselage != nullptr)
	{
		Doc->RemoveEntity(ReopenedFuselage);
		Check(Doc->GetEntities().Num() == 3, "deleting an operand removed the boolean result with it (the lattice, on the sphere, stays)");
	}
	// ... and deleting a filled body cascades to the lattice.
	if (ReopenedPrimitive != nullptr)
	{
		Doc->RemoveEntity(ReopenedPrimitive);
		Check(Doc->GetEntities().Num() == 1, "deleting the filled body removed the lattice with it");
	}

	IFileManager::Get().Delete(*TmpPath);

	// ---- Edit > Copy / Paste (Ctrl+C / Ctrl+V) ----
	// The one thing that matters: the copy is INDEPENDENT. Only the wing is
	// left in the scene by now, which is all this needs.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: copy/paste"));
	{
		AAerotelicEntityActor* Source = nullptr;
		for (auto& Weak : Doc->GetEntities())
		{
			if (Weak.IsValid() && Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Wing) Source = Weak.Get();
		}
		Check(Source != nullptr, "copy/paste: a wing is left to copy");
		if (Source != nullptr)
		{
			aerotelic::WingModel* SourceModel = Source->GetModelAs<aerotelic::WingModel>();
			const double OriginalSpan = SourceModel->planform().SemiSpan();
			Source->SetColorOverride(FLinearColor(0.25f, 0.5f, 0.75f));
			MoveTo(Source, FVector(1.0, 2.0, 3.0));
			const int32 Before = Doc->GetEntities().Num();

			Check(Doc->CopyEntityToClipboard(Source), "copy/paste: the wing copies");
			FString Error;
			AAerotelicEntityActor* Copy = Doc->PasteEntity(Error);
			Check(Copy != nullptr, FString::Printf(TEXT("copy/paste: paste made an entity: %s"), *Error));
			if (Copy != nullptr)
			{
				Check(Doc->GetEntities().Num() == Before + 1, "copy/paste: the scene gained exactly one entity");
				Check(Copy != Source, "copy/paste: the copy is a different actor");
				Check(Copy->EntityName != Source->EntityName, "copy/paste: the copy got its own unique name");
				Check(!Doc->IsNameTaken(Copy->EntityName, Copy), "copy/paste: that name is unique in the scene");
				Check(Copy->GetEntityType() == Source->GetEntityType(), "copy/paste: same kind");
				Check(Doc->GetSelectedEntity() == Copy, "copy/paste: the copy is selected, ready to transform");

				// The model object itself must be a different one, or the
				// two would be one entity wearing two names.
				aerotelic::WingModel* CopyModel = Copy->GetModelAs<aerotelic::WingModel>();
				Check(CopyModel != SourceModel, "copy/paste: the copy has a model of its own, not a shared pointer");
				Check(FMath::IsNearlyEqual(CopyModel->planform().SemiSpan(), OriginalSpan, 1e-12),
					"copy/paste: the copy starts with the source's parameters");
				Check(Copy->GetActorLocation().Equals(Source->GetActorLocation(), 1e-6), "copy/paste: pasted where the original sits");
				Check(Copy->GetColorOverride().IsSet(), "copy/paste: the colour came with it");

				// THE REQUIREMENT: editing either one must not touch the
				// other, in both directions.
				SourceModel->planform().SetSemiSpan(OriginalSpan * 2.0);
				Check(FMath::IsNearlyEqual(CopyModel->planform().SemiSpan(), OriginalSpan, 1e-12),
					"copy/paste: editing the SOURCE leaves the copy alone");
				CopyModel->planform().SetSemiSpan(OriginalSpan * 5.0);
				Check(FMath::IsNearlyEqual(SourceModel->planform().SemiSpan(), OriginalSpan * 2.0, 1e-12),
					"copy/paste: editing the COPY leaves the source alone");
				MoveTo(Source, FVector(40.0, 50.0, 60.0));
				Check(Copy->GetActorLocation().Equals(FVector(1.0, 2.0, 3.0), 1e-6),
					"copy/paste: moving the source leaves the copy where it was");

				// A second paste gives a third entity, not a rename of the
				// second: the clipboard is reusable.
				AAerotelicEntityActor* Again = Doc->PasteEntity(Error);
				Check(Again != nullptr && Again != Copy && Again->EntityName != Copy->EntityName,
					"copy/paste: pasting twice gives two separate copies");
				// ...and it is a SNAPSHOT: it still carries the parameters
				// as they were when copied, not as the source is now.
				if (Again != nullptr)
				{
					Check(FMath::IsNearlyEqual(Again->GetModelAs<aerotelic::WingModel>()->planform().SemiSpan(), OriginalSpan, 1e-12),
						"copy/paste: the clipboard holds what was copied, not the source's later edits");
					Doc->RemoveEntity(Again);
				}

				// Deleting the source must not disturb the copy -- nothing
				// links them, so no cascade.
				const FString CopyName = Copy->EntityName;
				Doc->RemoveEntity(Source);
				bool bCopySurvived = false;
				for (auto& Weak : Doc->GetEntities())
				{
					if (Weak.IsValid() && Weak->EntityName == CopyName) bCopySurvived = true;
				}
				Check(bCopySurvived, "copy/paste: deleting the source does not cascade to the copy");
			}
		}
	}

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
