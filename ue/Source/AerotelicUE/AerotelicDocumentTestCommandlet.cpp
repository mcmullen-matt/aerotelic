#include "AerotelicDocumentTestCommandlet.h"

#include "Misc/Paths.h"
#include "HAL/FileManager.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicLatticeModel.h"

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
	CheckBuildsSolid(Nacelle, TEXT("Nacelle"));
	CheckBuildsSolid(LoftedNacelle, TEXT("LoftedNacelle"));
	CheckBuildsSolid(Propeller, TEXT("Propeller"));

	for (AAerotelicEntityActor* E : { Nacelle, LoftedNacelle, Propeller })
	{
		if (E != nullptr) Doc->RemoveEntity(E);
	}

	if (Wing != nullptr)
	{
		Wing->SetActorLocation(FVector(10.0, 20.0, 30.0));
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
			Check(M->GetStats().SkinTets > 0 && M->GetShells().size() == 1 && M->MergedShell().tets.size() == (size_t)M->GetStats().SkinTets, "the skin's tetrahedra are kept for export");
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
			CheckBuildsSolid(E, TEXT("BoolOp 1"));
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

	UE_LOG(LogAerotelicDocumentTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
