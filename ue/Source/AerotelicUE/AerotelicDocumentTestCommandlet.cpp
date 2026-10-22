#include "AerotelicDocumentTestCommandlet.h"

#include "Misc/Paths.h"
#include "HAL/FileManager.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicLatticeModel.h"
#include "Entity/AerotelicMeshLib.h"
#include "Entity/AerotelicPotentialFlow.h"

#include "aerotelic/wing_model.hpp"

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
			Source->SetActorLocation(FVector(1.0, 2.0, 3.0));
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
				Source->SetActorLocation(FVector(40.0, 50.0, 60.0));
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
