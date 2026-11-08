#include "AerotelicRealProjectTestCommandlet.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicMeshLib.h"
#include "Entity/AerotelicStructuralFea.h"
#include "Entity/AerotelicFeaPreprocess.h"
#include "Entity/ReferenceImagePaths.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/materials.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicRealProjectTest, Log, All);

namespace
{
	int32 g_pass = 0, g_fail = 0;

	void Check(bool bCond, const FString& Msg)
	{
		if (bCond) { ++g_pass; return; }
		++g_fail;
		UE_LOG(LogAerotelicRealProjectTest, Error, TEXT("FAIL: %s"), *Msg);
	}

	// The per-cell von Mises the solver wrote to its legacy VTK
	// ("SCALARS von_mises_element double 1" under CELL_DATA), one per
	// element in the model's order: the tetrahedra, then the shells.
	bool ReadVtkCellVonMises(const FString& Path, TArray<double>& Out)
	{
		TArray<FString> Lines;
		if (!FFileHelper::LoadFileToStringArray(Lines, *Path)) return false;
		int32 Count = -1;
		for (int32 i = 0; i < Lines.Num(); ++i)
		{
			if (Lines[i].StartsWith(TEXT("CELL_DATA ")))
			{
				Count = FCString::Atoi(*Lines[i].Mid(10));
			}
			else if (Count > 0 && Lines[i].StartsWith(TEXT("SCALARS von_mises_element")))
			{
				Out.Reset();
				for (int32 j = i + 2; j < Lines.Num() && Out.Num() < Count; ++j) Out.Add(FCString::Atod(*Lines[j]));
				return Out.Num() == Count;
			}
		}
		return false;
	}

	// The thin-walled section of a closed shell surface cut by the plane
	// y = Station: every triangle crossing it gives a segment, and the
	// skin's second moment about the section's own centroid in z (with
	// its extreme fibre) follows from the segments as t ds. Beam theory's
	// bending stress at the station is then M z_max / I.
	struct FSection
	{
		double I = 0.0;        // t-weighted second moment about zc, project units^4
		double ZCentroid = 0.0;
		double ZMax = 0.0;     // the largest |z - zc|
		int32 Segments = 0;
	};
	FSection CutSection(const aerotelic::TriMesh& Mesh, double Station, double Thickness)
	{
		struct FSeg { double Z0, Z1, L; };
		TArray<FSeg> Segs;
		for (const auto& T : Mesh.triangles)
		{
			const aerotelic::Vec3* P[3] = { &Mesh.vertices[T[0]], &Mesh.vertices[T[1]], &Mesh.vertices[T[2]] };
			TArray<aerotelic::Vec3> Hits;
			for (int32 e = 0; e < 3; ++e)
			{
				const aerotelic::Vec3& A = *P[e];
				const aerotelic::Vec3& B = *P[(e + 1) % 3];
				const double Da = A.y - Station, Db = B.y - Station;
				if ((Da < 0.0) != (Db < 0.0))
				{
					const double S = Da / (Da - Db);
					Hits.Add(A + (B - A) * S);
				}
			}
			if (Hits.Num() == 2)
			{
				const double L = aerotelic::Length(Hits[1] - Hits[0]);
				if (L > 0.0) Segs.Add({ Hits[0].z, Hits[1].z, L });
			}
		}
		FSection S;
		double Sum = 0.0, First = 0.0;
		for (const FSeg& G : Segs) { Sum += Thickness * G.L; First += Thickness * G.L * 0.5 * (G.Z0 + G.Z1); }
		if (!(Sum > 0.0)) return S;
		S.ZCentroid = First / Sum;
		for (const FSeg& G : Segs)
		{
			const double Zm = 0.5 * (G.Z0 + G.Z1) - S.ZCentroid, Dz = G.Z1 - G.Z0;
			S.I += Thickness * G.L * (Zm * Zm + Dz * Dz / 12.0);
			S.ZMax = FMath::Max(S.ZMax, FMath::Max(FMath::Abs(G.Z0 - S.ZCentroid), FMath::Abs(G.Z1 - S.ZCentroid)));
		}
		S.Segments = Segs.Num();
		return S;
	}
}

int32 UAerotelicRealProjectTestCommandlet::Main(const FString& Params)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	Check(Doc != nullptr, "document subsystem is available");
	if (Doc == nullptr)
	{
		UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
		return 1;
	}

	// The Python app's repository (sibling of this one) ships the sample project.
	const FString Path = TEXT("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
	Check(Doc->OpenProjectPath(Path), "opened the real Python-saved p38.yaml");
	Check(Doc->GetEntities().Num() == 7, FString::Printf(TEXT("entity count is 7 (got %d)"), Doc->GetEntities().Num()));

	int32 nFuselage = 0, nWing = 0, nNacelle = 0, nPropeller = 0;
	for (auto& WeakEntity : Doc->GetEntities())
	{
		if (!WeakEntity.IsValid()) continue;
		switch (WeakEntity->GetEntityType())
		{
		case aerotelic::project_yaml::EntityType::Fuselage: ++nFuselage; break;
		case aerotelic::project_yaml::EntityType::Wing: ++nWing; break;
		case aerotelic::project_yaml::EntityType::Nacelle: ++nNacelle; break;
		case aerotelic::project_yaml::EntityType::Propeller: ++nPropeller; break;
		default: break;
		}
	}
	Check(nFuselage == 2 && nWing == 3 && nNacelle == 1 && nPropeller == 1,
		FString::Printf(TEXT("entity type mix matches the file (fuselage=%d wing=%d nacelle=%d propeller=%d)"), nFuselage, nWing, nNacelle, nPropeller));

	// Display state the file carries: every entity but Center is mirrored,
	// and four carry a colour override (the Wing's is navy). Both must
	// survive a save and a reload -- the first saver dropped them.
	auto Find = [Doc](const TCHAR* Name) -> AAerotelicEntityActor*
	{
		for (auto& WeakEntity : Doc->GetEntities())
		{
			if (WeakEntity.IsValid() && WeakEntity->EntityName == Name) return WeakEntity.Get();
		}
		return nullptr;
	};
	auto CheckState = [&](const TCHAR* Stage)
	{
		AAerotelicEntityActor* Center = Find(TEXT("Center"));
		AAerotelicEntityActor* Wing = Find(TEXT("Wing"));
		AAerotelicEntityActor* Boom = Find(TEXT("Boom"));
		AAerotelicEntityActor* Prop = Find(TEXT("Propeller 7"));
		Check(Center && Wing && Boom && Prop, FString::Printf(TEXT("%s: Center, Wing, Boom and Propeller 7 are present"), Stage));
		if (!(Center && Wing && Boom && Prop)) return;
		Check(!Center->IsMirrored() && Wing->IsMirrored() && Boom->IsMirrored() && Prop->IsMirrored(),
			FString::Printf(TEXT("%s: mirror flags (Center off; Wing, Boom, Propeller 7 on)"), Stage));
		const TOptional<FLinearColor> WingColor = Wing->GetColorOverride();
		Check(WingColor.IsSet() && FMath::IsNearlyEqual(WingColor->B, 0.498f, 0.01f) && WingColor->R < 0.01f && WingColor->G < 0.01f,
			FString::Printf(TEXT("%s: Wing colour override is navy (0, 0, 0.498)"), Stage));
		Check(!Center->GetColorOverride().IsSet(), FString::Printf(TEXT("%s: Center has no colour override (per-kind default applies)"), Stage));
		// (Bodies are not checked: a commandlet never tessellates, so the
		// body list is only filled in the interactive app.)
	};
	CheckState(TEXT("after open"));

	// Reference-image paths: beside the project file they are written
	// relative to it, elsewhere absolute; relative paths resolve on load.
	{
		const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir());
		aerotelic::FuselageSpec Fs;
		Fs.reference_images["profile"].path = TCHAR_TO_UTF8(*(Dir / TEXT("three_view.png")));
		Fs.reference_images["width"].path = TCHAR_TO_UTF8(*FString(TEXT("D:/elsewhere/top.png")));
		aerotelic::project_yaml::EntityModelSpec Spec = Fs;
		RelativiseReferenceImagePaths(Spec, Dir);
		const aerotelic::FuselageSpec& Out = std::get<aerotelic::FuselageSpec>(Spec);
		Check(Out.reference_images.at("profile").path == "three_view.png",
			FString::Printf(TEXT("an image beside the project file is saved relative to it (got %s)"), UTF8_TO_TCHAR(Out.reference_images.at("profile").path.c_str())));
		Check(Out.reference_images.at("width").path == "D:/elsewhere/top.png", "an image elsewhere stays absolute");
		aerotelic::FuselageModel Model(Out);
		ResolveReferenceImagePaths(&Model, aerotelic::project_yaml::EntityType::Fuselage, Dir);
		Check(FString(UTF8_TO_TCHAR(Model.reference_images.at("profile").path.c_str())).Equals(Dir / TEXT("three_view.png"), ESearchCase::IgnoreCase),
			FString::Printf(TEXT("a relative path resolves against the project directory on load (got %s)"), UTF8_TO_TCHAR(Model.reference_images.at("profile").path.c_str())));
	}

	// The other shipped tutorial is in WingDesigner's legacy format (entities
	// keyed by id + label, a raw features graph): it must load too.
	const FString Cessna = FPaths::Combine(FPaths::ProjectDir(), TEXT("Projects"), TEXT("cessna_172.yaml"));
	Check(Doc->OpenProjectPath(Cessna, /*bAdopt*/ false), "opened the legacy-format cessna_172.yaml tutorial");
	Check(Doc->GetEntities().Num() == 2, FString::Printf(TEXT("the Cessna has 2 entities (got %d)"), Doc->GetEntities().Num()));
	Check(!Doc->HasCurrentPath(), "a tutorial is not adopted as the current file");
	Check(Find(TEXT("Wing 2")) != nullptr && Find(TEXT("Wing 2"))->IsMirrored(), "the legacy label is the entity name and its mirror flag loaded");
	Check(Doc->OpenProjectPath(Path), "reopened p38");

	const FString TempPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("p38_roundtrip.yaml"));
	Check(Doc->SaveProjectToPath(TempPath), "saved the project to Saved/p38_roundtrip.yaml");
	Check(Doc->OpenProjectPath(TempPath), "reopened the saved copy");
	Check(Doc->GetEntities().Num() == 7, FString::Printf(TEXT("reopened entity count is 7 (got %d)"), Doc->GetEntities().Num()));
	CheckState(TEXT("after save+reload"));

	// A wing-tip test load on the P-38 (Matt, 2026-10-31: "the stress
	// combinations look reasonable on one side but should be near zero for
	// the unloaded side"). ue/Projects/p38a.yaml fuses the whole airframe
	// into BoolOp 1; that boolean alone -- not its operands beside it,
	// which would float free of the support -- is analysed as a thin-walled
	// shell of aluminium, clamped on the centre nacelle's ring at y = 0, a
	// point force down on the starboard wing tip and no other load. Held
	// to: the reactions equal the load; the port wing's outer panel reads
	// nothing; along the starboard outer panel (clear of the boom) the peak
	// skin stress is beam theory's M z / I of the actual section.
	UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("step: p38a wing-tip load"));
	const FString P38a = FPaths::Combine(FPaths::ProjectDir(), TEXT("Projects"), TEXT("p38a.yaml"));
	if (AerotelicStructuralFea::FindScript().IsEmpty() || AerotelicMeshLib::FindPython().IsEmpty())
	{
		UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("skipped: tools\\structural_fea.py or the WingDesigner environment is not here"));
	}
	else if (Doc->OpenProjectPath(P38a, /*bAdopt*/ false))
	{
		AAerotelicEntityActor* Bool = Find(TEXT("BoolOp 1"));
		AAerotelicEntityActor* Wing = Find(TEXT("Wing"));
		AAerotelicEntityActor* Boom = Find(TEXT("Boom"));
		Check(Bool != nullptr && Bool->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean && Wing != nullptr && Boom != nullptr, "p38a: BoolOp 1, Wing and Boom are present");
		// The airframe's material, on every parameterisation (the file
		// carries none): 2024-T3, 5 mm skins. The boolean inherits Center's.
		const double Thickness = 0.005;
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid() || Weak->IsDerived()) continue;
			aerotelic::MassSpec Spec;
			Spec.mode = aerotelic::MassMode::Material;
			Spec.material = *aerotelic::FindMaterialPreset("Aluminium 2024-T3 sheet");
			Spec.skin_thickness = Thickness;
			Weak->SetMassSpec(Spec);
		}
		if (Bool != nullptr && Wing != nullptr && Boom != nullptr)
		{
			Check(Bool->GetMassSpecSource() == Find(TEXT("Center")) && FMath::IsNearlyEqual(Bool->GetEffectiveMassSpec().skin_thickness, Thickness, 1e-12),
				"p38a: the boolean inherits Center's material and skin thickness");
			const std::vector<aerotelic::NamedMesh> Placed = Bool->GetPlacedMeshes(false, 2e-3);
			bool bClosed = Placed.size() >= 1;
			for (const aerotelic::NamedMesh& NM : Placed) bClosed = bClosed && NM.mesh.IsClosedManifold() && NM.mesh.SignedVolume() > 0.0;
			Check(bClosed, FString::Printf(TEXT("p38a: the fused airframe is %d closed, outward body(ies)"), (int32)Placed.size()));
			// The biggest solid is the airframe; a floating remnant (a
			// payload cylinder the fuse did not swallow) is left out, as the
			// dialog's floating check would have the user do.
			int32 Main = -1;
			double MainVolume = 0.0;
			for (int32 i = 0; i < (int32)Placed.size(); ++i)
			{
				const double V = Placed[i].mesh.SignedVolume();
				if (V > MainVolume) { MainVolume = V; Main = i; }
			}
			if (Main >= 0)
			{
				const aerotelic::TriMesh& Airframe = Placed[Main].mesh;
				aerotelic::TetMesh ShellMesh;
				ShellMesh.nodes = Airframe.vertices;
				AerotelicStructuralFea::FRequest Request;
				Request.Mesh = &ShellMesh;
				for (const auto& T : Airframe.triangles)
				{
					AerotelicStructuralFea::FShellFace F;
					F.N[0] = T[0]; F.N[1] = T[1]; F.N[2] = T[2];
					F.Region = 1;
					Request.ShellFaces.Add(F);
				}
				AerotelicStructuralFea::FRegion Region;
				Region.Id = 1;
				Region.Label = TEXT("airframe");
				Region.Skin = Bool->GetEffectiveMassSpec().material;
				Region.SkinThickness = Bool->GetEffectiveMassSpec().skin_thickness;
				Request.Regions.Add(Region);
				Request.LengthToSI = 1.0;
				// The starboard tip: the node furthest along +y. The force
				// down, 5 kN; nothing else.
				int32 Tip = 0;
				for (int32 n = 1; n < (int32)Airframe.vertices.size(); ++n) if (Airframe.vertices[n].y > Airframe.vertices[Tip].y) Tip = n;
				const aerotelic::Vec3 TipAt = Airframe.vertices[Tip];
				const double Force = 5000.0;
				AerotelicStructuralFea::FPointForce TipLoad;
				TipLoad.At = TipAt;
				TipLoad.Force = aerotelic::Vec3{ 0.0, 0.0, -Force };
				Request.PointForces.Add(TipLoad);
				Request.LoadFactor = 0.0;
				Request.Pressure = 0.0;
				AerotelicStructuralFea::FSupport Centre;
				Centre.Axis = TEXT('y');
				Centre.Value = 0.0;
				Centre.Tolerance = 0.05;
				Request.Supports.Add(Centre);
				Request.LinearSolver = TEXT("sparse_lu");
				Request.VtkPath = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_real_project_test_p38a.vtk"));
				IFileManager::Get().Delete(*Request.VtkPath, false, true, true);

				AerotelicFeaPreprocess::FModel Pre;
				FString PreError;
				const bool bPre = AerotelicFeaPreprocess::Build(Request, Pre, PreError);
				Check(bPre && Pre.SupportNodes.Num() >= 3 && Pre.FloatingRegions.Num() == 0,
					FString::Printf(TEXT("p38a: the airframe is one piece on the support (%d fixed nodes at y = 0, %d pieces, %d floating; %s)"), bPre ? Pre.SupportNodes.Num() : -1, bPre ? Pre.Components : -1, bPre ? Pre.FloatingRegions.Num() : -1, *PreError));
				Check(bPre && Pre.PointLoads.Num() == 1 && Pre.PointLoads[0].Node == Tip, "p38a: the point force goes to the tip node");
				const double YTip = TipAt.y;
				const FBox BoomBounds = Boom->GetPlacedBounds(false);
				const double YBoom = BoomBounds.IsValid ? BoomBounds.Max.Y : 0.3 * YTip;   // the outer panel starts outboard of the boom
				UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("p38a: %d nodes, %d shells; tip at (%.3f, %.3f, %.3f); boom to y = %.3f; %d fixed nodes"),
					(int32)Airframe.vertices.size(), (int32)Airframe.triangles.size(), TipAt.x, TipAt.y, TipAt.z, YBoom, bPre ? Pre.SupportNodes.Num() : -1);

				AerotelicStructuralFea::FResult R;
				FString Error;
				const bool bSolved = AerotelicStructuralFea::Run(Request, R, Error);
				Check(bSolved, FString::Printf(TEXT("p38a: the airframe solved under the tip load (%s)"), bSolved ? *FString::Printf(TEXT("%d nodes, %d shells, %d dof, %.1f s"), R.Nodes, R.Shells, R.Dofs, R.Seconds) : *Error));
				if (bSolved)
				{
					Check(FMath::Abs(R.Reaction[2] - Force) <= 0.02 * Force && FMath::Abs(R.Reaction[0]) <= 0.02 * Force && FMath::Abs(R.Reaction[1]) <= 0.02 * Force,
						FString::Printf(TEXT("p38a: the clamp reacts the tip load and nothing else (reactions %.4g, %.4g, %.4g N against %.4g)"), R.Reaction[0], R.Reaction[1], R.Reaction[2], Force));
					// A skin-only airframe is very compliant where the outer wing's
					// moment enters the nacelle cowl (no spar carries through), so
					// the tip moves metres under 5 kN: a linear analysis reports it
					// as such. What is checked is that it is at the tip and finite.
					Check(R.MaxDisplacement > 0.0 && FMath::IsFinite(R.MaxDisplacement) && FMath::Abs(R.MaxDisplacementAt[1] - YTip) < 0.1 * YTip,
						FString::Printf(TEXT("p38a: the largest displacement is at the loaded tip and finite (%.4g m at y = %.3f)"), R.MaxDisplacement, R.MaxDisplacementAt[1]));
					TArray<double> VonMises;
					const bool bVtk = ReadVtkCellVonMises(Request.VtkPath, VonMises) && VonMises.Num() == static_cast<int32>(Airframe.triangles.size());
					Check(bVtk, FString::Printf(TEXT("p38a: the VTK holds a von Mises per shell (%d of %d)"), VonMises.Num(), (int32)Airframe.triangles.size()));
					if (bVtk)
					{
						// Per shell: its centroid's y (the shells are the
						// airframe's triangles, in order).
						TArray<double> Yc;
						Yc.Reserve(VonMises.Num());
						for (const auto& T : Airframe.triangles) Yc.Add((Airframe.vertices[T[0]].y + Airframe.vertices[T[1]].y + Airframe.vertices[T[2]].y) / 3.0);
						// The loaded outer panel against beam theory at three
						// stations between the boom and the tip.
						double Worst = 0.0, LoadedPeak = 0.0;
						const double Panel = YTip - YBoom;
						for (double Eta : { 0.3, 0.5, 0.7 })
						{
							const double Station = YBoom + Eta * Panel;
							const FSection Sec = CutSection(Airframe, Station, Thickness);
							const double Moment = Force * (YTip - Station);
							const double Predicted = Sec.I > 0.0 ? Moment * Sec.ZMax / Sec.I : 0.0;
							double Measured = 0.0;
							for (int32 i = 0; i < VonMises.Num(); ++i) if (FMath::Abs(Yc[i] - Station) < 0.03 * Panel) Measured = FMath::Max(Measured, VonMises[i]);
							const double Ratio = Predicted > 0.0 ? Measured / Predicted : 0.0;
							Worst = FMath::Max(Worst, FMath::Abs(FMath::Loge(FMath::Max(Ratio, 1e-9))));
							UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("p38a: station y = %.3f (eta %.1f): section %d segments, I = %.4g m^4, z_max = %.4g m, M = %.4g N m -> beam theory %.4g Pa; FEA peak within the band %.4g Pa (ratio %.2f)"),
								Station, Eta, Sec.Segments, Sec.I, Sec.ZMax, Moment, Predicted, Measured, Ratio);
							Check(Ratio > 0.6 && Ratio < 1.7, FString::Printf(TEXT("p38a: at eta %.1f of the outer panel the skin's peak von Mises is beam theory's M z / I within reason (FEA %.4g vs %.4g Pa, ratio %.2f)"), Eta, Measured, Predicted, Ratio));
						}
						for (int32 i = 0; i < VonMises.Num(); ++i) if (Yc[i] > YBoom + 0.3 * Panel) LoadedPeak = FMath::Max(LoadedPeak, VonMises[i]);
						// The unloaded (port) outer panel: nothing to speak of.
						double UnloadedPeak = 0.0;
						for (int32 i = 0; i < VonMises.Num(); ++i) if (Yc[i] < -(YBoom + 0.3 * Panel)) UnloadedPeak = FMath::Max(UnloadedPeak, VonMises[i]);
						Check(LoadedPeak > 0.0 && UnloadedPeak < 0.02 * LoadedPeak,
							FString::Printf(TEXT("p38a: the unloaded wing's outer panel reads nothing against the loaded one (%.4g vs %.4g Pa)"), UnloadedPeak, LoadedPeak));
					}
					IFileManager::Get().Delete(*Request.VtkPath, false, true, true);
				}
			}
		}
	}
	else
	{
		Check(false, "p38a: ue/Projects/p38a.yaml opens");
	}

	UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
