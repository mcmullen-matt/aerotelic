#include "AerotelicDocumentTestCommandlet.h"

#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicBodyStructure.h"
#include "Entity/AerotelicMeshLib.h"
#include "Entity/AerotelicMeshing.h"
#include "Entity/AerotelicPotentialFlow.h"
#include "Entity/AerotelicMassProperties.h"
#include "Entity/AerotelicStructuralFea.h"
#include "Entity/AerotelicFeaPreprocess.h"
#include "Entity/AerotelicFeaModelBuilder.h"
#include "Entity/AerotelicMaterialLibrary.h"

#include "aerotelic/materials.hpp"
#include "aerotelic/mesh_cut.hpp"
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

	// Model > Material Properties, minus the dialog: the user's material
	// library is the presets plus a per-user YAML file (here a temp file),
	// found by name and by figures, refusing what is not a material.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: material library"));
	{
		const FString LibPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("materials_test.yaml"));
		IFileManager::Get().Delete(*LibPath);
		AerotelicMaterialLibrary::SetFilePathOverride(LibPath);
		Check(AerotelicMaterialLibrary::Custom().Num() == 0, "library: starts empty when there is no file");
		Check(AerotelicMaterialLibrary::All().Num() == (int32)aerotelic::MaterialPresets().size(), "library: with nothing custom the list is the presets");
		Check(AerotelicMaterialLibrary::Find("Aluminium 2024-T3 sheet") != nullptr, "library: a preset is found by name");

		aerotelic::Material Mine;
		Mine.name = "Balsa, test";
		Mine.category = "Wood";
		Mine.density = 160.0;
		Mine.youngs_modulus = 3.0e9;
		Mine.poisson_ratio = 0.30;
		Mine.allowable_stress = 10.0e6;
		Mine.source = "a test";
		FString Error;
		Check(AerotelicMaterialLibrary::SetCustom({ Mine }, Error), FString::Printf(TEXT("library: a complete material is accepted (%s)"), *Error));
		Check(FPaths::FileExists(LibPath), "library: the file was written");
		Check(AerotelicMaterialLibrary::All().Num() == (int32)aerotelic::MaterialPresets().size() + 1, "library: the list is the presets then the user's");
		const aerotelic::Material* Found = AerotelicMaterialLibrary::Find("Balsa, test");
		Check(Found != nullptr && Found->density == 160.0 && Found->category == "Wood", "library: the user's material is found by name");
		Check(AerotelicMaterialLibrary::FindMatching(Mine) != nullptr, "library: ...and matched by name and figures");
		aerotelic::Material Edited = Mine;
		Edited.density = 170.0;
		Check(AerotelicMaterialLibrary::FindMatching(Edited) == nullptr, "library: an edited copy matches nothing");

		aerotelic::Material Clash = Mine;
		Clash.name = "Aluminium 2024-T3 sheet";
		Check(!AerotelicMaterialLibrary::SetCustom({ Clash }, Error) && !Error.IsEmpty(), "library: a preset's name is refused");
		Check(!AerotelicMaterialLibrary::SetCustom({ Mine, Mine }, Error), "library: a repeated name is refused");
		aerotelic::Material Bad = Mine;
		Bad.youngs_modulus = 0.0;
		Check(!AerotelicMaterialLibrary::SetCustom({ Bad }, Error), "library: a material without a modulus is refused");
		Check(AerotelicMaterialLibrary::Custom().Num() == 1, "library: a refused change leaves the library as it was");

		// Reload from the file: every field comes back.
		AerotelicMaterialLibrary::SetFilePathOverride(LibPath);
		const TArray<aerotelic::Material>& Back = AerotelicMaterialLibrary::Custom();
		Check(Back.Num() == 1 && Back[0].name == "Balsa, test" && Back[0].category == "Wood" && Back[0].source == "a test"
			&& Back[0].density == 160.0 && Back[0].youngs_modulus == 3.0e9 && Back[0].allowable_stress == 10.0e6,
			"library: the user's material round-tripped through the file");

		IFileManager::Get().Delete(*LibPath);
		AerotelicMaterialLibrary::SetFilePathOverride(FString());
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
			// -- and the structure generated on this sphere below takes THIS
			// thickness as its skin's.
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

	// A body's STRUCTURE (2026-11-08) is what it is made of: material, skin
	// thickness, exclusions, asserted mass. The interior geometry comes from
	// the parameterisation (the wing's since 2026-11-09; the per-body plane
	// networks that stood in were dropped on 2026-11-10), so a primitive has
	// none to generate: the request is honoured with nothing, no error.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: BodyStructure"));
	{
		aerotelic::BodyStructure S = Primitive->GetBodyStructure(0);
		Check(S.IsSkin() && FMath::IsNearlyEqual(S.skin_thickness, 0.05, 1e-12) && FMath::IsNearlyEqual(S.material.density, 2.0, 1e-12), "structure: the sphere carries the mass step's 0.05 skin of the density-2 material");
		FString Error;
		Check(!Doc->GenerateBodyStructure(Wing, TEXT("Main"), Error) && Error.Contains(Wing->EntityName) && Error.Contains(TEXT("material")),
			FString::Printf(TEXT("structure: a body without a material is refused by name: %s"), *Error));
		Check(Wing->GetGeneratedStructure(TEXT("Main")) == nullptr, "structure: nothing is generated for the refused body");
		Check(Doc->GenerateBodyStructure(Primitive, TEXT("Main"), Error) && Primitive->GetGeneratedStructure(TEXT("Main")) == nullptr, "structure: a primitive has no interior structure of its own -- nothing generated, no error");
		Check(!Doc->HasPendingRebuilds(), "structure: nothing is queued for it either");
		Check(Doc->GetEntities().Num() == 4, "structure: generating adds no scene item");
		// The dialog's Apply sets the structure in one call.
		aerotelic::BodyStructure Thicker = S;
		Thicker.skin_thickness = 0.08;
		Check(Doc->ApplyBodyStructure(Primitive, TEXT("Main"), Thicker, Error) && FMath::IsNearlyEqual(Primitive->GetBodyStructure(0).skin_thickness, 0.08, 1e-12), FString::Printf(TEXT("structure: Apply sets the body's skin: %s"), *Error));
		Check(!Doc->HasPendingRebuilds(), "structure: Apply leaves nothing queued");
		Check(Doc->ApplyBodyStructure(Primitive, TEXT("Main"), S, Error) && FMath::IsNearlyEqual(Primitive->GetBodyStructure(0).skin_thickness, 0.05, 1e-12), "structure: ...and back to 0.05");
		// The mass report weighs the skin as rho t A over the body's mesh at
		// the report's tolerance.
		{
			FAerotelicBooleanOperand POp;
			POp.Entity = Primitive;
			const FAerotelicMassReport Report = ComputeAerotelicMassReport({ POp }, 2e-3, Doc->GetUnits());
			double ReportArea = 0.0;
			for (const aerotelic::NamedMesh& NM : Primitive->GetPlacedMeshes(false, 2e-3)) ReportArea += NM.mesh.SurfaceArea();
			Check(Report.Bodies.Num() == 1 && Report.Bodies[0].bIncluded && FMath::IsNearlyEqual(Report.Bodies[0].Props.mass, 2.0 * 0.05 * ReportArea, 1e-9 * FMath::Max(Report.Bodies[0].Props.mass, 1e-12)),
				FString::Printf(TEXT("structure: the mass report weighs the sphere's skin as density x 0.05 x area (%d line(s), %.5f vs %.5f)"), Report.Bodies.Num(), Report.Bodies.Num() ? Report.Bodies[0].Props.mass : 0.0, 2.0 * 0.05 * ReportArea));
		}
	}

	// A design panel's Apply: the panel edits a DRAFT, an independent copy
	// of the entity's model, and nothing in the scene changes until the
	// draft is applied -- at which point the entity gets a fresh model of
	// its own and the generated structure of its body follows.
	// A WING's structure comes from its parameterisation (2026-11-09): the
	// default two spars and eight ribs, planned on the wing's own grid and
	// generated as soon as the body has a skin -- no Generate step.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: wing structure"));
	{
		const aerotelic::WingModel* WM = Wing->GetModelAs<aerotelic::WingModel>();
		Check(WM->structure.spars.size() == 2 && WM->structure.ribs.between == 8, "wing structure: a new wing carries the default two spars and eight ribs");
		Check(Wing->GetLastStructurePlans().size() == 1 && Wing->GetLastStructurePlans()[0].ribs.size() == 8 && Wing->GetLastStructurePlans()[0].spars.size() == 2,
			FString::Printf(TEXT("wing structure: the build planned 8 ribs and 2 spar webs on the Main body (%d plan(s))"), (int32)Wing->GetLastStructurePlans().size()));
		aerotelic::BodyStructure WS;
		WS.material = *aerotelic::FindMaterialPreset("Aluminium 2024-T3 sheet");
		WS.skin_thickness = 0.002;
		Check(Wing->SetBodyStructure(TEXT("Main"), WS), "wing structure: the wing takes a 2 mm aluminium skin");
		Check(Doc->HasPendingRebuilds(), "wing structure: a wing body with a plan is queued as soon as it has a skin");
		Doc->FlushPendingRebuilds();
		const FAerotelicBodyStructure* GW = Wing->GetGeneratedStructure(TEXT("Main"));
		Check(GW != nullptr && GW->IsGenerated(), "wing structure: the Main body's structure generated from the plan");
		if (GW != nullptr && GW->IsGenerated())
		{
			const std::vector<FAerotelicStructureLayer>& L = GW->GetLayers();
			Check(L.size() == 4 && L[0].Kind == FAerotelicStructureLayer::EKind::Skin && L[1].Kind == FAerotelicStructureLayer::EKind::Ribs
				&& L[2].Kind == FAerotelicStructureLayer::EKind::Spars && L[3].Kind == FAerotelicStructureLayer::EKind::Spars,
				FString::Printf(TEXT("wing structure: layers Skin, Ribs, Front spar, Rear spar (%d)"), (int32)L.size()));
			if (L.size() == 4)
			{
				Check(!L[1].Mesh.IsEmpty() && L[1].Area > 0.0 && L[1].Name == TEXT("Ribs"), FString::Printf(TEXT("wing structure: the Ribs layer holds the eight discs (%d triangles, area %.4g)"), (int32)L[1].Mesh.triangles.size(), L[1].Area));
				Check(!L[2].Mesh.IsEmpty() && L[2].Name == TEXT("Front spar") && !L[3].Mesh.IsEmpty() && L[3].Name == TEXT("Rear spar"), "wing structure: both spar webs have triangles");
				Check(L[1].Material.name == WS.material.name && L[2].Material.name == WS.material.name, "wing structure: elements without a material of their own take the body's");
				Check(GW->GetStats().Warnings.Num() == 0, FString::Printf(TEXT("wing structure: no warnings (%s)"), *FString::Join(GW->GetStats().Warnings, TEXT("; "))));
				// The rib discs' rim vertices are the body's own vertices: meshed
				// on the wing's rings, not cut into it.
				const std::vector<aerotelic::NamedMesh>& Own = Wing->GetLastMeshes();
				int32 OnBody = 0;
				if (!Own.empty())
				{
					for (const aerotelic::Vec3& V : L[1].Mesh.vertices)
					{
						for (const aerotelic::Vec3& B : Own[0].mesh.vertices) { if (aerotelic::DistanceSq(V, B) < 1e-24) { ++OnBody; break; } }
					}
				}
				Check(OnBody >= 8 * 20, FString::Printf(TEXT("wing structure: the rib rims are the body's own vertices (%d rib vertices on the body)"), OnBody));
			// The analysis model takes the layers as shells on the skin's nodes
			// (AerotelicFeaModelBuilder, 2026-11-10): a region per layer, the
			// rims' and feet's nodes shared with the skin, the discs' and webs'
			// interior nodes added after it.
			{
				AerotelicFeaModelBuilder::FOptions Options;
				AerotelicFeaModelBuilder::FCache Cache;
				AerotelicFeaModelBuilder::FBuilt Built;
				FAerotelicBooleanOperand WOp;
				WOp.Entity = Wing;
				const bool bAppended = AerotelicFeaModelBuilder::AppendBody(WOp, TEXT("Main"), Wing->EntityName, 0, Options, Cache, Built);
				const int32 SkinNodes = Own.empty() ? 0 : static_cast<int32>(Own[0].mesh.vertices.size());
				int32 LayerVertices = 0;
				for (const FAerotelicStructureLayer& Lay : L) LayerVertices += static_cast<int32>(Lay.Mesh.vertices.size());
				Check(bAppended && Built.Regions.Num() == 4 && Built.RibFaces.Num() > 0 && Built.Regions[1].HasRibs() && !Built.Regions[1].HasShells(),
					FString::Printf(TEXT("wing structure (fea): the model is the skin and three layer regions of unloaded shells (%d regions, %d rib faces; %s)"), Built.Regions.Num(), Built.RibFaces.Num(), *FString::Join(Built.Problems, TEXT("; "))));
				Check(bAppended && static_cast<int32>(Built.Nodes.size()) > SkinNodes && static_cast<int32>(Built.Nodes.size()) < SkinNodes + LayerVertices,
					FString::Printf(TEXT("wing structure (fea): the layers share nodes with the skin (%d nodes of %d skin + %d layer vertices)"), (int32)Built.Nodes.size(), SkinNodes, LayerVertices));
				TSet<int32> SkinSet, RibNodes;
				for (const auto& F : Built.ShellFaces) for (int32 k = 0; k < 3; ++k) SkinSet.Add(F.N[k]);
				for (const auto& F : Built.RibFaces) for (int32 k = 0; k < 3; ++k) RibNodes.Add(F.N[k]);
				int32 OnSkin = 0;
				for (int32 N : RibNodes) if (SkinSet.Contains(N)) ++OnSkin;
				Check(OnSkin >= 8 * 20 + 2 * 10, FString::Printf(TEXT("wing structure (fea): the rib rims and spar feet are skin nodes (%d shared)"), OnSkin));
				const FString Line = AerotelicFeaModelBuilder::Describe(WOp, TEXT("Main"), Wing->EntityName, Options, Doc->GetUnits());
				Check(Line.Contains(TEXT("3 generated layer")) && Line.Contains(TEXT("Front spar")), FString::Printf(TEXT("wing structure (fea): the Set-up line names the layers (%s)"), *Line));
			}
			}
		}
	}

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: apply"));
	if (Primitive != nullptr && Wing != nullptr)
	{
		AAerotelicEntityActor* Boolean = nullptr;
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid()) continue;
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean) Boolean = Weak.Get();
		}
		Check(Doc->CloneEntityModel(Boolean) == nullptr, "apply: a boolean result has no draft to edit (its operands have)");

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

			// The boolean over the sphere follows: queued by the Apply, rebuilt
			// by the flush (the ticker's job in the application). Two items in
			// all -- the sphere, done, then the boolean -- so the bar stands at
			// a half. (The sphere has no generated structure of its own since
			// the plane networks went; the wing's follows its Apply below.)
			Check(Doc->HasPendingRebuilds(), "apply: the boolean over the sphere is queued to follow");
			Check(FMath::IsNearlyEqual(Doc->GetRebuildProgress(), 0.5f, 1e-6f), FString::Printf(TEXT("apply: the progress bar shows one of two rebuilt (%.3f)"), Doc->GetRebuildProgress()));
			Doc->FlushPendingRebuilds();
			Check(!Doc->HasPendingRebuilds() && Doc->GetRebuildProgress() == 1.0f, "apply: the flush rebuilt what was queued and the bar is full again");

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
			// The wing's generated structure follows: queued by the Apply, its
			// layers kept and marked stale until the flush regenerates them on
			// the new span -- the spars then run the doubled semi-span.
			Check(Doc->HasPendingRebuilds(), "apply: the wing's structure is queued to follow its new span");
			{
				const FAerotelicBodyStructure* Stale = Wing->GetGeneratedStructure(TEXT("Main"));
				Check(Stale != nullptr && Stale->IsStale() && Stale->IsGenerated(), "apply: the wing's layers are kept, marked stale, until the regeneration");
			}
			Doc->FlushPendingRebuilds();
			{
				const FAerotelicBodyStructure* G = Wing->GetGeneratedStructure(TEXT("Main"));
				Check(G != nullptr && G->IsGenerated() && !G->IsStale() && G->GetLayers().size() == 4, "apply: the wing's structure is generated again");
				double YMax = 0.0;
				if (G != nullptr)
				{
					for (const FAerotelicStructureLayer& L : G->GetLayers())
					{
						if (L.Kind != FAerotelicStructureLayer::EKind::Spars) continue;
						for (const aerotelic::Vec3& V : L.Mesh.vertices) YMax = FMath::Max(YMax, V.y);
					}
				}
				Check(YMax > 1.9 * Span, FString::Printf(TEXT("apply: the spars run the doubled span (to y = %.3f of %.3f)"), YMax, 2.0 * Span));
			}
			WD->planform().SetSemiSpan(Span);
			Check(Doc->ApplyEntityModel(Wing, *WingDraft, Error), "apply: restoring the wing applies");
			Doc->FlushPendingRebuilds();
		}
	}

	// The viewport's OCCT-free meshes: every entity's model builds its
	// bodies straight from its curves, closed and outward.
	// Structural analysis (Analysis -> Structural (Kratos FEA)) on the
	// sphere, shells only (2026-11-08: the tetrahedral cases -- the
	// generated structure's skin / core, a homogenised infill core, a solid
	// body -- went with the tetrahedra), each held to closed forms. The
	// sphere's skin is the 0.05 set in the mass step. The aluminium skin of
	// a unit sphere, stood on its bottom under its own weight and 1 bar:
	// the reactions carry the weight exactly (the pressure's resultant over
	// a closed surface is zero), the surface moves inward, and the skin's
	// stress is the thin shell's membrane value p a / 2t.
	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea"));
	if (AerotelicStructuralFea::FindScript().IsEmpty() || AerotelicMeshLib::FindPython().IsEmpty())
	{
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("skipped: tools\\structural_fea.py or the WingDesigner environment is not here"));
	}
	else
	{
		const aerotelic::Material Al = *aerotelic::FindMaterialPreset("Aluminium 2024-T3 sheet");
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

		// (1) A thin-walled body: the sphere's own triangles (its placed
		// mesh at tolerance 0 -- its Mesh tab's grid at the viewport's rule,
		// exactly what the viewport shows) as shells of its 0.05 aluminium
		// skin and nothing inside -- what Analysis > Structural makes of any
		// body with a skin thickness. Closed forms: mass rho t A, the weight
		// carried, the membrane stress p a / 2t = 1.0 MPa, the surface
		// moving inward.
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea (a thin-walled body)"));
		if (Primitive != nullptr)
		{
			const std::vector<aerotelic::NamedMesh> Placed = Primitive->GetPlacedMeshes(false);
			const std::vector<aerotelic::NamedMesh> Finer = Primitive->GetPlacedMeshes(false, 1e-3);
			Check(Placed.size() == 1 && Placed[0].mesh.IsClosedManifold(), "fea (shells only): the sphere is one closed body");
			Check(Placed.size() == 1 && Finer.size() == 1 && Placed[0].mesh.triangles.size() < Finer[0].mesh.triangles.size(),
				FString::Printf(TEXT("fea (shells only): the own grid is the viewport's, coarser than a finer tolerance asks for (%d vs %d triangles)"), Placed.size() ? (int32)Placed[0].mesh.triangles.size() : -1, Finer.size() ? (int32)Finer[0].mesh.triangles.size() : -1));
			if (Placed.size() == 1)
			{
				const aerotelic::TriMesh& Surface = Placed[0].mesh;
				std::vector<aerotelic::Vec3> ShellNodes = Surface.vertices;
				AerotelicStructuralFea::FRequest Request;
				Request.Nodes = &ShellNodes;
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
					Check(R.Shells == static_cast<int32>(Surface.triangles.size()) && R.Nodes == static_cast<int32>(Surface.vertices.size()) && R.Dofs == 6 * R.Nodes, "fea (shells only): a shell on every triangle, a node per vertex, six degrees of freedom a node");
					// The script's own verdict on the balance, and which solver
					// ran ("auto" here, as the dialog's default: the direct solver
					// within its budget).
					Check(R.bBalanced && R.BalanceError <= 0.01 && R.Warnings.Num() == 0 && R.LinearSolver == TEXT("sparse_lu"),
						FString::Printf(TEXT("fea (shells only): the result reports a balanced solve (error %.2e, %d warning(s)) by %s"), R.BalanceError, R.Warnings.Num(), *R.LinearSolver));
					Check(FMath::Abs(R.AppliedPressure[2]) <= 1e-3 * 1.0e5 * 4.0 * PI,
						FString::Printf(TEXT("fea (shells only): a uniform pressure over a closed surface has no resultant (%.3g N)"), R.AppliedPressure[2]));
					const double ExpectedMass = Al.density *0.05 * Surface.SurfaceArea();
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
					Check(bPre && Pre.WettedFaces.Num() == R.WettedFaces && Pre.SupportNodes.Num() == R.SupportCornerNodes && Pre.Regions.Num() == 1 && Pre.Regions[0].Kind == TEXT("shells"),
						FString::Printf(TEXT("fea (shells only, pre-processor): the faces and the fixed nodes are the script's (%d / %d faces, %d / %d nodes; %s)"), bPre ? Pre.WettedFaces.Num() : -1, R.WettedFaces, bPre ? Pre.SupportNodes.Num() : -1, R.SupportCornerNodes, *PreError));
					// The cost estimate (2026-11-05): six degrees of freedom on
					// every node, the script's figure, within the direct solver's
					// budget.
					Check(bPre && Pre.ShellNodes == Pre.Nodes && Pre.Dofs == R.Dofs && Pre.DirectSolverAuto(),
						FString::Printf(TEXT("fea (shells only, pre-processor): the DOF estimate is the solver's, every node a shell node (%lld vs %d)"), bPre ? Pre.Dofs : -1, R.Dofs));
					Check(bPre && FMath::IsNearlyEqual(Pre.Mass, R.Mass, 1e-6 * R.Mass), "fea (shells only, pre-processor): the mass is rho t A");
					Check(bPre && FMath::IsNearlyEqual(Pre.GravityForce.Z, R.AppliedGravity[2], 1e-6 * FMath::Abs(R.AppliedGravity[2])) && Pre.GravityForce.X == 0.0 && Pre.GravityForce.Y == 0.0,
						FString::Printf(TEXT("fea (shells only, pre-processor): the inertial load is the script's applied gravity (%.6g vs %.6g N)"), bPre ? Pre.GravityForce.Z : 0.0, R.AppliedGravity[2]));
					Check(bPre && Pre.PressureResultant.Size() <= 1e-3 * 1.0e5 * 4.0 * PI && Pre.MaxFacePressure == 1.0e5,
						FString::Printf(TEXT("fea (shells only, pre-processor): the pressure's resultant over the closed surface is zero (%.3g N)"), bPre ? Pre.PressureResultant.Size() : -1.0));
					Check(bPre && Pre.PointLoads.Num() == 0 && Pre.MassCentre.Size() < 0.05, FString::Printf(TEXT("fea (shells only, pre-processor): no point force; the centre of mass is near the sphere's centre (%.3g)"), bPre ? Pre.MassCentre.Size() : -1.0));
					{
						bool bOnPlane = bPre && Pre.SupportNodes.Num() > 0 && Pre.SupportPlanes.Num() == 1 && Pre.SupportPlanes[0].Axis == 2;
						for (int32 N : (bPre ? Pre.SupportNodes : TArray<int32>())) bOnPlane = bOnPlane && FMath::Abs(ShellNodes[N].z - Pre.SupportPlanes[0].Value) <= Pre.SupportPlanes[0].Tolerance;
						Check(bOnPlane && FMath::IsNearlyEqual(Pre.SupportPlanes[0].Value, Pre.Bounds.min.z, 1e-12), "fea (shells only, pre-processor): every fixed node lies within the tolerance of z = the mesh's minimum");
						bool bOutward = bPre;
						for (int32 i = 0; bPre && i < Pre.WettedFaces.Num(); ++i) bOutward = bOutward && FVector::DotProduct(Pre.FaceNormal[i], Pre.FaceCentre[i]) > 0.0;
						Check(bOutward, "fea (shells only, pre-processor): the wetted faces are wound outward");
					}

					// A support PATCH (2026-11-05): the same plane z = min limited
					// to a quadrant, x and y from -1 to 0.15 (the in-plane axes of a
					// z plane are x then y). The script and the pre-processor fix
					// the same, smaller set of nodes, and the quadrant still
					// carries the sphere's weight.
					const int32 WholePlaneNodes = R.SupportCornerNodes;
					Request.Supports[0].bBounded = true;
					Request.Supports[0].Lo[0] = -1.0; Request.Supports[0].Hi[0] = 0.15;
					Request.Supports[0].Lo[1] = -1.0; Request.Supports[0].Hi[1] = 0.15;
					Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_document_test_fea_patch.vtk"));
					AerotelicStructuralFea::FResult RP;
					FString PatchError;
					const bool bPatch = AerotelicStructuralFea::Run(Request, RP, PatchError);
					Check(bPatch, FString::Printf(TEXT("fea (patch): the thin-walled sphere solved on a quadrant patch of the support plane (%s)"), bPatch ? *FString::Printf(TEXT("%d fixed nodes of %d on the whole plane"), RP.SupportCornerNodes, WholePlaneNodes) : *PatchError));
					if (bPatch)
					{
						AerotelicFeaPreprocess::FModel PreP;
						FString PrePError;
						const bool bPreP = AerotelicFeaPreprocess::Build(Request, PreP, PrePError);
						Check(RP.SupportCornerNodes > 2 && RP.SupportCornerNodes < WholePlaneNodes, FString::Printf(TEXT("fea (patch): the patch fixes fewer nodes than the whole plane (%d vs %d)"), RP.SupportCornerNodes, WholePlaneNodes));
						Check(bPreP && PreP.SupportNodes.Num() == RP.SupportCornerNodes && PreP.SupportPlanes.Num() == 1 && PreP.SupportPlanes[0].bBounded,
							FString::Printf(TEXT("fea (patch, pre-processor): the patch's fixed nodes are the script's (%d vs %d)"), bPreP ? PreP.SupportNodes.Num() : -1, RP.SupportCornerNodes));
						bool bInside = bPreP;
						for (int32 N : (bPreP ? PreP.SupportNodes : TArray<int32>()))
						{
							const aerotelic::Vec3& P = ShellNodes[N];
							bInside = bInside && P.x >= -1.0 && P.x <= 0.15 && P.y >= -1.0 && P.y <= 0.15;
						}
						Check(bInside, "fea (patch, pre-processor): every fixed node lies inside the patch's rectangle");
						const double WeightP = RP.Mass * 9.80665;
						Check(RP.bBalanced && FMath::Abs(RP.Reaction[2] - WeightP) <= 0.02 * WeightP, FString::Printf(TEXT("fea (patch): the quadrant carries the weight (%.4g vs %.4g N)"), RP.Reaction[2], WeightP));
						IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
					}
				}
			}
		}

		// (2) A body with NO skin thickness is refused (2026-11-08: the
		// analysis models shells only; until then such a body was filled
		// with solid tetrahedra). The wing, which deliberately carries no
		// setting, is given a structural material and no skin for the
		// moment: the builder's Describe and AppendBody both name the skin
		// thickness as what is missing, and nothing is added to the model.
		UE_LOG(LogAerotelicDocumentTest, Display, TEXT("step: structural fea (a body with no skin thickness)"));
		if (Wing != nullptr)
		{
			const aerotelic::MassSpec Saved = Wing->GetMassSpec();
			aerotelic::MassSpec NoSkin;
			NoSkin.mode = aerotelic::MassMode::Material;
			NoSkin.material = Al;
			NoSkin.skin_thickness = 0.0;
			Wing->SetMassSpec(NoSkin);
			AerotelicFeaModelBuilder::FOptions Options;
			AerotelicFeaModelBuilder::FCache Cache;
			AerotelicFeaModelBuilder::FBuilt Built;
			FAerotelicBooleanOperand WingOp;
			WingOp.Entity = Wing;
			bool bOk = true;
			const FString Line = AerotelicFeaModelBuilder::Describe(WingOp, FString(), Wing->EntityName, Options, Doc->GetUnits(), &bOk);
			Check(!bOk && Line.Contains(TEXT("skin thickness")), FString::Printf(TEXT("fea (no skin): the Set-up line says a body without a skin thickness cannot be modelled (%s)"), *Line));
			const bool bAppended = AerotelicFeaModelBuilder::AppendBody(WingOp, FString(), Wing->EntityName, 0, Options, Cache, Built);
			Check(!bAppended && Built.Problems.Num() == 1 && Built.Problems[0].Contains(TEXT("skin thickness")) && Built.Problems[0].Contains(Wing->EntityName),
				FString::Printf(TEXT("fea (no skin): AppendBody refuses the body by name for its skin thickness (%s)"), *FString::Join(Built.Problems, TEXT("; "))));
			Check(Built.Nodes.empty() && Built.ShellFaces.Num() == 0 && Built.RibFaces.Num() == 0 && Built.Regions.Num() == 0, "fea (no skin): nothing was added to the model");
			Wing->SetMassSpec(Saved);
		}
	}

	// (3) A box primitive as a thin-walled cantilever against beam theory
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
			const aerotelic::Material Al = *aerotelic::FindMaterialPreset("Aluminium 2024-T3 sheet");
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
				std::vector<aerotelic::Vec3> ShellNodes = Surface.vertices;
				AerotelicStructuralFea::FRequest Request;
				Request.Nodes = &ShellNodes;
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

	// Model > Units: set before the save, read back after the reopen.
	{
		aerotelic::UnitSystem U;
		U.length = aerotelic::LengthUnit::Foot;
		U.mass = aerotelic::MassUnit::PoundMass;
		Doc->SetUnits(U);
		Check(Doc->GetUnits().length == aerotelic::LengthUnit::Foot, "units: set on the document");
	}

	// Model > Scale: a 1:4 replica. The parameterisations and the
	// placements stay the full-scale aircraft; the actors, the placed
	// meshes, bounds and solids are a quarter the size and a quarter as far
	// out; a skin's mass goes as the area (1/16) while its thickness stays;
	// the boolean and the generated structure follow. Set before the save,
	// read back after the reopen.
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
		double FullSkinMass = 0.0;
		{
			const FAerotelicMassReport Full = ComputeAerotelicMassReport({ P }, 2e-3, Doc->GetUnits());
			FullMass = Full.Assembly.mass;
			for (const FAerotelicMassLine& L : Full.Bodies) if (L.bIncluded) { FullSkinMass = L.Props.mass; break; }
		}
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
		Check(Doc->HasPendingRebuilds(), "scale: the boolean and the wing's structure are queued to follow");
		Doc->FlushPendingRebuilds();
		Check(!Doc->HasPendingRebuilds(), "scale: ... and rebuilt");
		{
			// The sphere's skin over the replica: still 0.05 thick (as typed,
			// not scaled) and, as a SHELL (density x area x thickness on the
			// body's own surface, no offset), its mass scales with the area
			// alone -- a sixteenth of the full-size skin on a quarter-size
			// sphere. As a ratio of the full-size skin's mass, so the project's
			// units (changed earlier in the run) drop out.
			const FAerotelicMassReport R = ComputeAerotelicMassReport({ P }, 2e-3, Doc->GetUnits());
			const aerotelic::MassSpec Spec = Primitive->GetMassSpec();
			Check(Spec.IsSkin(), "scale: the sphere weighs as a skin (set in the mass step)");
			const FAerotelicMassLine* SkinLine = nullptr;
			for (const FAerotelicMassLine& L : R.Bodies) if (L.bIncluded) { SkinLine = &L; break; }
			const double Ratio = 0.25 * 0.25;
			Check(SkinLine != nullptr && SkinLine->bIncluded && FullSkinMass > 0.0 && FMath::IsNearlyEqual(SkinLine->Props.mass, Ratio * FullSkinMass, 0.03 * Ratio * FullSkinMass),
				FString::Printf(TEXT("scale: the skin shell of the quarter-size sphere weighs a sixteenth of the full-size one, its thickness as typed (%.5g vs %.5g = %.4f of the full-size skin %.5g)"), SkinLine ? SkinLine->Props.mass : -1.0, Ratio * FullSkinMass, Ratio, FullSkinMass));
			Check(FullMass > 0.0 && R.Assembly.mass < FullMass / 8.0, "scale: the replica weighs far less than the full-size sphere");
		}
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
	Check(Doc->GetEntities().Num() == 4, "reopened scene has 3 entities and the boolean result");
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

	// Copy refuses the derived kind, while it is still in the scene: a
	// boolean result is a solid built FROM other entities, with no
	// parameterisation of its own to hand a constructor.
	{
		AAerotelicEntityActor* AnyBoolean = nullptr;
		AAerotelicEntityActor* AnyWing = nullptr;
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid()) continue;
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean) AnyBoolean = Weak.Get();
			if (Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Wing) AnyWing = Weak.Get();
		}
		Check(!UAerotelicDocumentSubsystem::CanCopyEntity(AnyBoolean), "copy: a boolean result cannot be copied");
		Check(!UAerotelicDocumentSubsystem::CanCopyEntity(nullptr), "copy: nothing selected cannot be copied");
		Check(UAerotelicDocumentSubsystem::CanCopyEntity(AnyWing), "copy: a wing can be copied");
		// A refused copy must leave whatever was on the clipboard alone
		// rather than emptying it.
		Check(Doc->CopyEntityToClipboard(AnyWing) && Doc->HasClipboardEntity(), "copy: the wing went onto the clipboard");
		Check(!Doc->CopyEntityToClipboard(AnyBoolean), "copy: copying the boolean result returns false");
		Check(Doc->HasClipboardEntity() && AnyWing != nullptr && Doc->GetClipboardEntityName() == AnyWing->EntityName,
			"copy: the refused copy left the wing on the clipboard untouched");
	}

	bool bFoundWing = false, bFoundFuselage = false, bFoundPrimitive = false, bFoundBoolean = false;
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
		default:
			break;
		}
	}
	Check(bFoundWing && bFoundFuselage && bFoundPrimitive && bFoundBoolean, "all three entity types and the boolean survived the round trip");
	// The wing's structure came back with it: its spec (two spars, eight
	// ribs) in the file, and its layers regenerated on load, since the body
	// has a skin.
	for (auto& W : Doc->GetEntities())
	{
		if (!W.IsValid() || W->GetEntityType() != aerotelic::project_yaml::EntityType::Wing) continue;
		const aerotelic::WingModel* RW = W->GetModelAs<aerotelic::WingModel>();
		Check(RW->structure.spars.size() == 2 && RW->structure.ribs.between == 8, "structure: the wing's spars and rib rules round-tripped");
		const FAerotelicBodyStructure* G = W->GetGeneratedStructure(TEXT("Main"));
		Check(G != nullptr && G->IsGenerated() && G->GetLayers().size() == 4, FString::Printf(TEXT("structure: the wing's layers were regenerated on load (%d)"), G != nullptr ? (int32)G->GetLayers().size() : -1));
	}

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
		Check(Doc->GetEntities().Num() == 2, "deleting an operand removed the boolean result with it (the sphere, structure and all, stays)");
	}
	// ... and the sphere's structure goes with the sphere, being its own.
	if (ReopenedPrimitive != nullptr)
	{
		Doc->RemoveEntity(ReopenedPrimitive);
		Check(Doc->GetEntities().Num() == 1, "deleting the structured body leaves the wing alone");
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
