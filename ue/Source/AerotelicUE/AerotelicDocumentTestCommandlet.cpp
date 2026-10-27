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

#include "aerotelic/primitive_model.hpp"
#include "aerotelic/wing_model.hpp"
#include "aerotelic/wing_panels.hpp"
#include "aerotelic/panel_loads.hpp"
#include "aerotelic/colormaps.hpp"
#include "Entity/AerotelicMeshing.h"

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
		ByDensity.mode = aerotelic::MassMode::Density;
		ByDensity.density = 2.0;
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
		const FAerotelicMassReport Report = ComputeAerotelicMassReport({ P, F, W, FM }, 2e-3);

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
			Check(!WingLine->bIncluded && WingLine->Note.Contains(TEXT("no mass properties")),
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
			if (L.StartsWith(Wing->EntityName) && L.Contains(TEXT("left out: no mass properties"))) ++WingRows;
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
		Spec.skin_thickness = 0.05;
		FString Error;
		{
			// A lattice must have a skin.
			aerotelic::LatticeSpec NoSkin = Spec;
			NoSkin.skin_thickness = 0.0;
			AAerotelicEntityActor* Refused = Doc->AddLattice(TEXT("Lattice 1"), NoSkin, Targets, TArray<FAerotelicBooleanOperand>(), Error);
			Check(Refused == nullptr && Error.Contains(TEXT("skin")), FString::Printf(TEXT("a zero skin thickness is refused: %s"), *Error));
			Check(UAerotelicDocumentSubsystem::DefaultSkinThickness(Targets) > 0.0, "the default skin thickness is positive");
		}
		AAerotelicEntityActor* Lattice = Doc->AddLattice(TEXT("Lattice 1"), Spec, Targets, TArray<FAerotelicBooleanOperand>(), Error);
		Check(Lattice != nullptr, FString::Printf(TEXT("gyroid lattice in the sphere generated: %s"), *Error));
		Check(Doc->GetEntities().Num() == 5, "the lattice is a fifth scene item");
		Check(Doc->NextLatticeName() == TEXT("Lattice 2"), "next default lattice name is Lattice 2");
		if (Lattice != nullptr)
		{
			const FAerotelicLatticeModel* M = Lattice->GetModelAs<FAerotelicLatticeModel>();
			Check(M->GetStats().Triangles > 0, "the lattice has triangles");
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
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: meshes"));
	for (auto& WeakEntity : Doc->GetEntities())
	{
		if (!WeakEntity.IsValid() || WeakEntity->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean) continue;
		std::vector<aerotelic::NamedMesh> Meshes = WeakEntity->GetPlacedMeshes(false, 2e-3);
		bool bOk = !Meshes.empty();
		for (const aerotelic::NamedMesh& M : Meshes) bOk = bOk && M.mesh.IsClosedManifold() && M.mesh.SignedVolume() > 0.0;
		Check(bOk, FString::Printf(TEXT("%s: placed meshes are closed, outward bodies (%d)"), *WeakEntity->EntityName, (int32)Meshes.size()));
	}

	// Potential flow (Analysis -> Potential Flow (Kratos)...) over the
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
	}

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
		Check(M.mode == aerotelic::MassMode::Density && FMath::IsNearlyEqual(M.density, 2.0, 1e-12),
			"mass: the primitive's density round-tripped through the project file");
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
