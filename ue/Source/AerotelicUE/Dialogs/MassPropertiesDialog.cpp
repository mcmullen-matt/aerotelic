#include "MassPropertiesDialog.h"
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
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/PlatformApplicationMisc.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
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
#include "Entity/AerotelicMassProperties.h"

#include "aerotelic/mass_properties.hpp"
#include "aerotelic/mesh.hpp"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "MassPropertiesDialog"

// Analysis > Mass Properties: the report of the chosen bodies (as before)
// and, since 2026-11-13, the POINT MASSES -- the concentrated masses that
// are not part of any body's shells (a battery, a motor, the payload),
// each with its mass, its centre and its inertia tensor about that centre,
// hanging on a solid body of the model. They are part of the MODEL, not of
// any one report: the document's (UAerotelicDocumentSubsystem::
// GetMassItems), saved with the project under `masses:`, counted in the
// report and hung on the structure by Analysis > Structural (an RBE3-type
// attachment on the body's nodes nearest the mass's centre; a body that is
// an operand of a boolean result hands the mass on to the result). The
// Point masses tab's picture therefore shows EVERY solid body in the
// scene, whatever the Report tab has chosen to weigh: a click on a surface
// puts the selected mass's centre there, and the body it hangs on is
// picked from a list of the bodies (2026-11-14, Matt: one click, no
// second pick).

namespace
{
	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field, float LabelWidth = 150.0f)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(LabelWidth) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<SWidget> Button(const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center).OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	}

	// Beside the project, named after it, like the flow solution.
	FString DefaultReportPath()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_mass.txt"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_mass.txt"));
	}

	FString UnitName(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }

	// Every SOLID BODY of the scene, one row each (the Structural dialog's
	// rule): a wing's main element and its flap, a propeller's blades, a
	// boolean result's shells, the mirrored halves as rows of their own.
	TArray<FAerotelicBodyChoicePtr> AllBodyChoices()
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

	// ---- the picture's geometry (the Structural dialog's pattern): flat-
	// shaded triangles in both windings for the two-sided material ----
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
	};

	FColor Srgb(float R, float G, float B) { return AAerotelicColoredMeshActor::VertexColorFromSRGB(R, G, B); }

	void AddOctahedron(FGeom& G, const FVector& P, double R, const FColor& Col)
	{
		const FVector X(R, 0, 0), Y(0, R, 0), Z(0, 0, R);
		const FVector Px = P + X, Mx = P - X, Py = P + Y, My = P - Y, Pz = P + Z, Mz = P - Z;
		G.Tri(Px, Py, Pz, Col); G.Tri(Py, Mx, Pz, Col); G.Tri(Mx, My, Pz, Col); G.Tri(My, Px, Pz, Col);
		G.Tri(Py, Px, Mz, Col); G.Tri(Mx, Py, Mz, Col); G.Tri(My, Mx, Mz, Col); G.Tri(Px, My, Mz, Col);
	}

	const FColor kGrey = Srgb(0.74f, 0.75f, 0.77f);
	const FColor kBodyHot = Srgb(0.55f, 0.72f, 0.95f);
	const FColor kMass = Srgb(0.95f, 0.18f, 0.18f);
	const FColor kMassArmed = Srgb(1.0f, 0.75f, 0.2f);
	const FColor kCentre = Srgb(0.30f, 0.85f, 0.35f);

	// Moeller-Trumbore: the ray parameter of the hit, or a negative number.
	double RayTriangle(const FVector& O, const FVector& D, const FVector& A, const FVector& B, const FVector& C)
	{
		const FVector E1 = B - A, E2 = C - A;
		const FVector P = FVector::CrossProduct(D, E2);
		const double Det = FVector::DotProduct(E1, P);
		if (FMath::Abs(Det) < 1e-14) return -1.0;
		const double Inv = 1.0 / Det;
		const FVector S = O - A;
		const double U = FVector::DotProduct(S, P) * Inv;
		if (U < -1e-9 || U > 1.0 + 1e-9) return -1.0;
		const FVector Q = FVector::CrossProduct(S, E1);
		const double V = FVector::DotProduct(D, Q) * Inv;
		if (V < -1e-9 || U + V > 1.0 + 1e-9) return -1.0;
		return FVector::DotProduct(E2, Q) * Inv;
	}

	// A point mass as the user types it: the document's aerotelic::MassItem
	// in the dialog's terms. Everything full-scale and in the project's
	// units, as the file has it. Attach is a row of the body list (0: none,
	// i > 0: the body choice i - 1).
	struct FPointMassRow
	{
		FString Name = TEXT("mass");
		double Mass = 1.0;
		double X = 0.0, Y = 0.0, Z = 0.0;
		double I[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };   // Ixx, Iyy, Izz, Ixy, Ixz, Iyz about the centre
		int32 Attach = 0;
		int32 Nodes = 8;
		bool bPlaced = false;   // the centre has been set (by a click or by hand)
	};

	// A body in the picture: whose, which, its placed (replica) mesh.
	struct FShownBody
	{
		int32 Choice = -1;   // the row of the body list
		aerotelic::TriMesh Mesh;
	};

	struct FState : public TSharedFromThis<FState>
	{
		TFunction<void(const FText&)> Status;
		TWeakPtr<SWindow> Window;
		aerotelic::UnitSystem Units;
		TSharedPtr<SAerotelicTabStrip> Tabs;

		// The report: its own choice of bodies to weigh.
		TSharedPtr<SAerotelicBodyPicker> Picker;
		double Tolerance = 0.0;   // 0: each body's own grid; a figure: that chordal tolerance
		TSharedPtr<STextBlock> ReportText;
		FString ReportString;

		// The point masses: every solid body of the scene is in the picture
		// and in the list a mass hangs from.
		TArray<FAerotelicBodyChoicePtr> Bodies;
		TArray<TSharedPtr<FString>> AttachOptions;   // "nothing", then Bodies' labels
		TArray<TSharedPtr<FPointMassRow>> Rows;
		TSharedPtr<SVerticalBox> RowList;
		TWeakPtr<FPointMassRow> Selected;   // the row the next click in the picture places
		TSharedPtr<SAerotelicSceneView> View;
		TSharedPtr<STextBlock> Caption;
		TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>> Actors;
		bool bFramed = false;
		TArray<FShownBody> Shown;

		static double ModelScale() { return AAerotelicEntityActor::GetModelScale(); }
		static bool IsReplica() { return !FMath::IsNearlyEqual(ModelScale(), 1.0, 1e-9); }

		void Message(const FText& Text) { AerotelicMessageBox(EAppMsgType::Ok, Text, LOCTEXT("Title", "Mass Properties")); }

		// ---- the document's items in and out ----
		aerotelic::MassItem ToItem(const FPointMassRow& R) const
		{
			aerotelic::MassItem M;
			M.name = TCHAR_TO_UTF8(*R.Name);
			M.mass = R.Mass;
			M.position = { R.X, R.Y, R.Z };
			for (int32 k = 0; k < 6; ++k) M.inertia[k] = R.I[k];
			M.attach_nodes = FMath::Max(R.Nodes, 3);
			if (R.Attach > 0 && Bodies.IsValidIndex(R.Attach - 1) && Bodies[R.Attach - 1]->Operand.Entity.IsValid())
			{
				const FAerotelicBodyChoice& C = *Bodies[R.Attach - 1];
				M.attach_entity = TCHAR_TO_UTF8(*C.Operand.Entity->EntityName);
				M.attach_mirror = C.Operand.bMirror;
				M.attach_body = TCHAR_TO_UTF8(*C.Body);
			}
			return M;
		}

		TSharedPtr<FPointMassRow> FromItem(const aerotelic::MassItem& M) const
		{
			TSharedPtr<FPointMassRow> R = MakeShared<FPointMassRow>();
			R->Name = UTF8_TO_TCHAR(M.name.c_str());
			R->Mass = M.mass;
			R->X = M.position[0]; R->Y = M.position[1]; R->Z = M.position[2];
			for (int32 k = 0; k < 6; ++k) R->I[k] = M.inertia[k];
			R->Nodes = M.attach_nodes;
			R->bPlaced = true;
			if (!M.attach_entity.empty())
			{
				const FString EntityName = UTF8_TO_TCHAR(M.attach_entity.c_str());
				const FString Body = UTF8_TO_TCHAR(M.attach_body.c_str());
				for (int32 i = 0; i < Bodies.Num(); ++i)
				{
					const FAerotelicBodyChoice& C = *Bodies[i];
					if (!C.Operand.Entity.IsValid() || C.Operand.Entity->EntityName != EntityName || C.Operand.bMirror != M.attach_mirror) continue;
					if (!Body.IsEmpty() && C.Body != Body) continue;
					R->Attach = i + 1;
					break;
				}
			}
			return R;
		}

		FString AttachLabel(const FPointMassRow& R) const
		{
			return AttachOptions.IsValidIndex(R.Attach) ? *AttachOptions[R.Attach] : FString(TEXT("nothing"));
		}

		void LoadRows()
		{
			Rows.Reset();
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			if (Doc == nullptr) return;
			for (const aerotelic::MassItem& M : Doc->GetMassItems()) Rows.Add(FromItem(M));
		}

		FString RowsProblem() const
		{
			TSet<FString> Names;
			for (const TSharedPtr<FPointMassRow>& R : Rows)
			{
				if (R->Name.TrimStartAndEnd().IsEmpty()) return TEXT("a point mass has no name");
				if (Names.Contains(R->Name)) return FString::Printf(TEXT("two point masses are named '%s'"), *R->Name);
				Names.Add(R->Name);
				const std::string Problem = aerotelic::MassItemProblem(ToItem(*R));
				if (!Problem.empty()) return UTF8_TO_TCHAR(Problem.c_str());
			}
			return FString();
		}

		// Apply: the rows into the document, which saves them with the project.
		void Apply()
		{
			const FString Problem = RowsProblem();
			if (!Problem.IsEmpty()) { Message(FText::FromString(Problem)); return; }
			UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			if (Doc == nullptr) return;
			TArray<aerotelic::MassItem> Items;
			for (const TSharedPtr<FPointMassRow>& R : Rows) Items.Add(ToItem(*R));
			Doc->SetMassItems(Items);
			int32 Loose = 0;
			for (const TSharedPtr<FPointMassRow>& R : Rows) Loose += R->Attach <= 0 ? 1 : 0;
			Status(FText::FromString(FString::Printf(TEXT("%d point mass%s kept with the project%s"), Rows.Num(), Rows.Num() == 1 ? TEXT("") : TEXT("es"),
				Loose ? *FString::Printf(TEXT(" (%d hang%s on nothing: left out of the report and of the structural analysis until given a body)"), Loose, Loose == 1 ? TEXT("s") : TEXT("")) : TEXT(""))));
			UpdatePicture();
		}

		// ---- the report ----
		void Compute()
		{
			TArray<FAerotelicBooleanOperand> Chosen;
			if (Picker.IsValid()) for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen()) Chosen.Add(C->Operand);
			if (Chosen.Num() == 0 && Rows.Num() == 0)
			{
				Message(LOCTEXT("NoneChosen", "Choose at least one body to weigh, or add a point mass."));
				return;
			}
			const double Started = FPlatformTime::Seconds();
			TArray<aerotelic::MassItem> Items;
			for (const TSharedPtr<FPointMassRow>& R : Rows) Items.Add(ToItem(*R));
			const FAerotelicMassReport Report = ComputeAerotelicMassReport(Chosen, Tolerance, Units, &Items);
			ReportString = FormatAerotelicMassReport(Report, Units);
			if (ReportText.IsValid()) ReportText->SetText(FText::FromString(ReportString));
			const int32 LeftOut = Report.Bodies.Num() - Report.IncludedCount;
			Status(FText::FromString(FString::Printf(TEXT("Weighed %d component%s (%d point mass%s) in %.1f s%s"),
				Report.IncludedCount, Report.IncludedCount == 1 ? TEXT("") : TEXT("s"), Report.PointMasses, Report.PointMasses == 1 ? TEXT("") : TEXT("es"),
				FPlatformTime::Seconds() - Started, LeftOut > 0 ? *FString::Printf(TEXT(" -- %d left out, see the report"), LeftOut) : TEXT(""))));
		}

		void Copy()
		{
			if (ReportString.IsEmpty()) return;
			FPlatformApplicationMisc::ClipboardCopy(*ReportString);
			Status(LOCTEXT("Copied", "Report copied to the clipboard."));
		}

		void Save()
		{
			if (ReportString.IsEmpty()) return;
			IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
			TSharedPtr<SWindow> W = Window.Pin();
			if (Desktop == nullptr || !W.IsValid()) return;
			TArray<FString> Files;
			const FString Default = DefaultReportPath();
			const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(W.ToSharedRef());
			if (!Desktop->SaveFileDialog(Handle, TEXT("Mass Properties Report"), FPaths::GetPath(Default),
				FPaths::GetCleanFilename(Default), TEXT("Text Files (*.txt)|*.txt"), EFileDialogFlags::None, Files) || Files.Num() == 0)
			{
				return;
			}
			if (FFileHelper::SaveStringToFile(ReportString, *Files[0])) Status(FText::FromString(FString::Printf(TEXT("Report saved to %s"), *Files[0])));
			else Message(FText::FromString(TEXT("Could not write ") + Files[0]));
		}

		// ---- the picture ----
		AAerotelicColoredMeshActor* EnsureActor(int32 Index)
		{
			while (Actors.Num() <= Index) Actors.Add(nullptr);
			if (Actors[Index].IsValid()) return Actors[Index].Get();
			AAerotelicCameraPawn* Pawn = AAerotelicCameraPawn::FindInGameWorld();
			UWorld* World = Pawn != nullptr ? Pawn->GetWorld() : nullptr;
			if (World == nullptr) return nullptr;
			FActorSpawnParameters Params;
			Params.ObjectFlags |= RF_Transient;
			AAerotelicColoredMeshActor* A = World->SpawnActor<AAerotelicColoredMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
			Actors[Index] = A;
			return A;
		}

		// Every solid body's placed mesh, for the picture and the picking.
		void CollectShown()
		{
			Shown.Reset();
			for (int32 i = 0; i < Bodies.Num(); ++i)
			{
				const FAerotelicBodyChoice& C = *Bodies[i];
				AAerotelicEntityActor* E = C.Operand.Entity.Get();
				if (E == nullptr) continue;
				for (aerotelic::NamedMesh& M : E->GetPlacedMeshes(C.Operand.bMirror))
				{
					if (!C.Body.IsEmpty() && FString(UTF8_TO_TCHAR(M.name.c_str())) != C.Body) continue;
					FShownBody B;
					B.Choice = i;
					B.Mesh = std::move(M.mesh);
					Shown.Add(MoveTemp(B));
				}
			}
		}

		double Diagonal() const
		{
			aerotelic::Box3 Box;
			for (const FShownBody& B : Shown) for (const aerotelic::Vec3& V : B.Mesh.vertices) Box.Add(V);
			for (const TSharedPtr<FPointMassRow>& R : Rows) if (R->bPlaced) Box.Add(aerotelic::Vec3{ R->X, R->Y, R->Z } * ModelScale());
			return Box.IsEmpty() ? 1.0 : FMath::Max(Box.Diagonal(), 1e-9);
		}

		void UpdatePicture()
		{
			if (!View.IsValid()) return;
			const double Scale = ModelScale();
			const double Diag = Diagonal();
			TSharedPtr<FPointMassRow> Sel = Selected.Pin();
			// The body the selected mass hangs on, brighter.
			const int32 HotChoice = Sel.IsValid() && Sel->Attach > 0 ? Sel->Attach - 1 : -1;
			FGeom BodiesGeom;
			for (const FShownBody& B : Shown)
			{
				const FColor Col = B.Choice == HotChoice ? kBodyHot : kGrey;
				for (const auto& T : B.Mesh.triangles)
				{
					const aerotelic::Vec3& A = B.Mesh.vertices[T[0]];
					const aerotelic::Vec3& Bv = B.Mesh.vertices[T[1]];
					const aerotelic::Vec3& Cv = B.Mesh.vertices[T[2]];
					BodiesGeom.Tri(FVector(A.x, A.y, A.z), FVector(Bv.x, Bv.y, Bv.z), FVector(Cv.x, Cv.y, Cv.z), Col);
				}
			}
			FGeom Marks;
			double Biggest = 0.0, Total = 0.0;
			FVector First = FVector::ZeroVector;
			for (const TSharedPtr<FPointMassRow>& R : Rows) if (R->bPlaced) { Biggest = FMath::Max(Biggest, R->Mass); Total += R->Mass; First += FVector(R->X, R->Y, R->Z) * Scale * R->Mass; }
			FString Text;
			for (int32 i = 0; i < Rows.Num(); ++i)
			{
				const TSharedPtr<FPointMassRow>& R = Rows[i];
				if (R->bPlaced)
				{
					const FVector Centre = FVector(R->X, R->Y, R->Z) * Scale;
					const double Radius = (0.008 + 0.025 * FMath::Pow(R->Mass / FMath::Max(Biggest, 1e-300), 1.0 / 3.0)) * Diag;
					AddOctahedron(Marks, Centre, Radius, R == Sel ? kMassArmed : kMass);
				}
				Text += FString::Printf(TEXT("%s%s: %.4g %s%s, hangs on %s%s"), i ? TEXT("\n") : TEXT(""), *R->Name, R->Mass, *UnitName(Units.MassName()),
					R->bPlaced ? *FString::Printf(TEXT(" at (%.4g, %.4g, %.4g)"), R->X, R->Y, R->Z) : TEXT(" (not placed yet)"), *AttachLabel(*R),
					R == Sel ? TEXT("  <-- SELECTED: click a surface to put its centre there") : TEXT(""));
			}
			if (Rows.Num() == 0) Text = TEXT("No point mass yet: Add one on the left, click the surface where its centre sits, and choose the body it hangs on.");
			if (Total > 0.0)
			{
				const FVector Cg = First / Total;
				AddOctahedron(Marks, Cg, 0.01 * Diag, kCentre);
				Text += FString::Printf(TEXT("\nThe point masses together: %.5g %s, centre (green) at (%.4g, %.4g, %.4g) %s"), Total, *UnitName(Units.MassName()),
					Cg.X / Scale, Cg.Y / Scale, Cg.Z / Scale, *UnitName(Units.LengthName()));
			}
			Text += FString::Printf(TEXT("\nEvery solid body of the scene (%d); the one the selected mass hangs on in blue. Red: each mass at its centre, sized by its mass (amber: the selected one). Left-drag orbits, right-drag pans, wheel zooms, Home frames."), Shown.Num());
			if (IsReplica()) Text += TEXT("\nCoordinates here and on the left are the FULL-SCALE aircraft's; the picture shows the replica.");
			AAerotelicColoredMeshActor* A0 = EnsureActor(0);
			AAerotelicColoredMeshActor* A1 = EnsureActor(1);
			if (A0 == nullptr || A1 == nullptr) return;
			A0->SetMesh(BodiesGeom.V, BodiesGeom.T, BodiesGeom.N, BodiesGeom.C);
			A0->SetEdges(false);
			A1->SetMesh(Marks.V, Marks.T, Marks.N, Marks.C);
			A1->SetEdges(false);
			View->SetShownActors({ A0, A1 });
			if (!bFramed) { View->FrameShown(); bFramed = true; }
			View->RequestRedraw();
			if (Caption.IsValid()) Caption->SetText(FText::FromString(Text));
		}

		// A left press in the picture with a row selected: the ray's first
		// hit on any body's surface becomes the row's centre.
		bool OnPickDown(const FVector& Origin, const FVector& Dir)
		{
			TSharedPtr<FPointMassRow> Row = Selected.Pin();
			if (!Row.IsValid()) return false;
			double Best = 1e300;
			int32 BestBody = -1;
			for (int32 b = 0; b < Shown.Num(); ++b)
			{
				const aerotelic::TriMesh& M = Shown[b].Mesh;
				for (const auto& T : M.triangles)
				{
					const FVector A(M.vertices[T[0]].x, M.vertices[T[0]].y, M.vertices[T[0]].z);
					const FVector B(M.vertices[T[1]].x, M.vertices[T[1]].y, M.vertices[T[1]].z);
					const FVector C(M.vertices[T[2]].x, M.vertices[T[2]].y, M.vertices[T[2]].z);
					const double Tt = RayTriangle(Origin, Dir, A, B, C);
					if (Tt > 0.0 && Tt < Best) { Best = Tt; BestBody = b; }
				}
			}
			if (BestBody < 0)
			{
				Status(LOCTEXT("Missed", "The click missed every body: click on a surface to put the mass's centre there."));
				return true;
			}
			const FVector Hit = Origin + Dir * Best;
			const double Scale = ModelScale();
			Row->X = Hit.X / Scale; Row->Y = Hit.Y / Scale; Row->Z = Hit.Z / Scale;
			Row->bPlaced = true;
			// A mass not yet hung on anything hangs on the body clicked.
			if (Row->Attach <= 0) Row->Attach = Shown[BestBody].Choice + 1;
			Status(FText::FromString(FString::Printf(TEXT("'%s' centred at (%.4g, %.4g, %.4g) %s on %s; hangs on %s -- edit the figures on the left, then Apply"),
				*Row->Name, Row->X, Row->Y, Row->Z, *UnitName(Units.LengthName()), *Bodies[Shown[BestBody].Choice]->Label, *AttachLabel(*Row))));
			RebuildRowList();
			UpdatePicture();
			return true;
		}

		// ---- the cards ----
		TSharedRef<SWidget> NumberField(TFunction<double()> Get, TFunction<void(double)> Set, double Delta, const FText& Suffix, double Min = -1e12, double Max = 1e12)
		{
			return SNew(SAerotelicNumericEntry)
				.Value_Lambda([Get]() { return Get(); })
				.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([Set](double V, ETextCommit::Type) { Set(V); });
		}

		void RebuildRowList()
		{
			if (!RowList.IsValid()) return;
			RowList->ClearChildren();
			const FText LengthSuffix = FText::FromString(TEXT(" ") + UnitName(Units.LengthName()));
			const FText MassSuffix = FText::FromString(TEXT(" ") + UnitName(Units.MassName()));
			const FText InertiaSuffix = FText::FromString(TEXT(" ") + UnitName(Units.MassName()) + TEXT(" ") + UnitName(Units.LengthName()) + TEXT("^2"));
			TSharedPtr<FState> Self = AsShared();
			TSharedPtr<FPointMassRow> Sel = Selected.Pin();
			for (int32 i = 0; i < Rows.Num(); ++i)
			{
				TSharedPtr<FPointMassRow> Row = Rows[i];
				auto Triple = [this, Row, &LengthSuffix, &InertiaSuffix](int32 First)
				{
					// First 0: the centre (X, Y, Z); 3: Ixx, Iyy, Izz; 6: Ixy, Ixz, Iyz.
					auto Field = [this, Row, First, &LengthSuffix, &InertiaSuffix](int32 k)
					{
						auto Get = [Row, First, k]() -> double { return First == 0 ? (k == 0 ? Row->X : (k == 1 ? Row->Y : Row->Z)) : Row->I[First - 3 + k]; };
						auto Set = [Row, First, k, this](double V)
						{
							if (First == 0) { (k == 0 ? Row->X : (k == 1 ? Row->Y : Row->Z)) = V; Row->bPlaced = true; }
							else Row->I[First - 3 + k] = V;
							UpdatePicture();
						};
						return NumberField(Get, Set, First == 0 ? 0.1 : 0.01, First == 0 ? LengthSuffix : InertiaSuffix);
					};
					return SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0) [ Field(0) ]
						+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0) [ Field(1) ]
						+ SHorizontalBox::Slot().FillWidth(1) [ Field(2) ];
				};
				const bool bSelected = Sel == Row;
				RowList->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
				[
					SNew(SBorder).Padding(6).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(bSelected ? FLinearColor(0.2f, 0.16f, 0.05f) : FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center) [ SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("Point mass %d%s"), i + 1, bSelected ? TEXT(" -- selected: click a surface to place it") : TEXT("")))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
							+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
							[ Button(bSelected ? LOCTEXT("Deselect", "Done") : LOCTEXT("Select", "Select"), [Self, Row]() { Self->Selected = Self->Selected.Pin() == Row ? nullptr : Row; Self->RebuildRowList(); Self->UpdatePicture(); }) ]
							+ SHorizontalBox::Slot().AutoWidth()
							[ Button(LOCTEXT("Delete", "Delete"), [Self, Row]() { Self->Rows.Remove(Row); if (Self->Selected.Pin() == Row) Self->Selected = nullptr; Self->RebuildRowList(); Self->UpdatePicture(); }) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							FormRow(LOCTEXT("Name", "Name:"),
								SNew(SEditableTextBox).Text_Lambda([Row]() { return FText::FromString(Row->Name); })
								.OnTextCommitted_Lambda([Row, Self](const FText& T, ETextCommit::Type) { Row->Name = T.ToString().TrimStartAndEnd(); Self->UpdatePicture(); }), 100.0f)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Mass", "Mass:"), NumberField([Row]() { return Row->Mass; }, [Row, Self](double V) { Row->Mass = FMath::Max(V, 0.0); Self->UpdatePicture(); }, 0.1, MassSuffix, 0.0), 100.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(IsReplica() ? LOCTEXT("CgFull", "Centre (full scale):") : LOCTEXT("Cg", "Centre:"), Triple(0), 100.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("InertiaDiag", "Ixx, Iyy, Izz:"), Triple(3), 100.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("InertiaOff", "Ixy, Ixz, Iyz:"), Triple(6), 100.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3)
						[
							FormRow(LOCTEXT("HangsOn", "Hangs on:"),
								SNew(SComboBox<TSharedPtr<FString>>)
								.OptionsSource(&Self->AttachOptions)
								.InitiallySelectedItem(Self->AttachOptions[FMath::Clamp(Row->Attach, 0, Self->AttachOptions.Num() - 1)])
								.OnGenerateWidget_Lambda([](TSharedPtr<FString> Item) { return SNew(STextBlock).Text(FText::FromString(*Item)); })
								.OnSelectionChanged_Lambda([Self, Row](TSharedPtr<FString> Item, ESelectInfo::Type)
								{
									const int32 I = Self->AttachOptions.IndexOfByKey(Item);
									if (I != INDEX_NONE) { Row->Attach = I; Self->UpdatePicture(); }
								})
								[ SNew(STextBlock).Text_Lambda([Self, Row]() { return FText::FromString(Self->AttachLabel(*Row)); }) ], 100.0f)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Nodes", "Nearest nodes:"), NumberField([Row]() { return (double)Row->Nodes; }, [Row](double V) { Row->Nodes = FMath::Max(FMath::RoundToInt32(V), 3); }, 1.0, FText::GetEmpty(), 3.0, 200.0), 100.0f) ]
					]
				];
			}
		}

		void AddRow()
		{
			TSharedPtr<FPointMassRow> Row = MakeShared<FPointMassRow>();
			Row->Name = FString::Printf(TEXT("mass %d"), Rows.Num() + 1);
			Rows.Add(Row);
			Selected = Row;
			RebuildRowList();
			UpdatePicture();
			Status(LOCTEXT("Placing", "Click the surface where the new mass's centre sits; it hangs on the body you click unless you choose another in its list."));
		}

		void Close()
		{
			for (TWeakObjectPtr<AAerotelicColoredMeshActor>& A : Actors) if (A.IsValid()) A->Destroy();
			Actors.Empty();
			View.Reset();
			Caption.Reset();
			RowList.Reset();
			ReportText.Reset();
			Picker.Reset();
			Tabs.Reset();
		}
	};
}

void ShowAerotelicMassPropertiesDialog(TFunction<void(const FText&)> Status)
{
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("Nothing", "Nothing in the scene to weigh."),
			LOCTEXT("Title", "Mass Properties"));
		return;
	}

	const FVector2D Size(1180, 900);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Mass Properties"))
		.ClientSize(Size)
		.SupportsMaximize(true)
		.SupportsMinimize(true);

	TSharedRef<FState> S = MakeShared<FState>();
	S->Status = Status;
	S->Window = Window;
	const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	S->Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	S->Bodies = AllBodyChoices();
	S->AttachOptions.Add(MakeShared<FString>(TEXT("nothing (left out of the report and the structural analysis)")));
	for (const FAerotelicBodyChoicePtr& C : S->Bodies) S->AttachOptions.Add(MakeShared<FString>(C->Label));
	S->LoadRows();

	// ---- the Report tab (the dialog as it was) ----
	TSharedRef<SWidget> ReportTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("Bodies", "Bodies to weigh"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SAssignNew(S->Picker, SAerotelicBodyPicker).Choices(Choices).ListHeight(150.0f)
					.OnChanged(FSimpleDelegate::CreateLambda([S]()
					{
						const int32 N = S->Picker.IsValid() ? S->Picker->GetChosen().Num() : 0;
						S->Status(N ? FText::FromString(FString::Printf(TEXT("%d selected to weigh"), N)) : LOCTEXT("SelectBody", "Select the bodies to weigh."));
					}))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("BodiesHint",
						"Each body weighs the way its Structure says (right-click it in the Scene tree): its material's density over the volume it encloses, a skin as density x area x thickness with its generated layers, or an asserted mass spread over its surface. One without a setting is reported as left out rather than counted as zero. Of the POINT MASSES of the next tab, those hung on the chosen bodies (or on an operand of a chosen boolean result) are counted too, each as a line of its own; the rest are not."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("MeshGroup", "Mesh"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
				[
					FormRow(LOCTEXT("Tolerance", "Surface tolerance:"),
						SNew(SAerotelicNumericEntry)
						.Value_Lambda([S]() { return S->Tolerance; })
						.MinValue(0.0).MaxValue(1e9).Delta(0.001)
						.Suffix(FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(S->Units.LengthName())))
						.LiveUpdate(false)
						.OnValueCommitted_Lambda([S](double V, ETextCommit::Type) { S->Tolerance = FMath::Max(V, 0.0); }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("ToleranceHint",
						"0 weighs each body on ITS OWN GRID -- the triangles its Mesh tab defines and the viewport shows, the same ones the structural analysis solves on. A figure is the chordal sag the triangles may have from the true surface instead (the same tolerance STL export uses). The integrals are exact for the triangles, so this is the accuracy of the volume and the inertia of a curved body. A box is exact at any setting."))
				])
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8).HAlign(HAlign_Left)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0) [ Button(LOCTEXT("Compute", "Compute"), [S]() { S->Compute(); }) ]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0) [ Button(LOCTEXT("Copy", "Copy"), [S]() { S->Copy(); }) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Button(LOCTEXT("Save", "Save..."), [S]() { S->Save(); }) ]
		]
		+ SVerticalBox::Slot().FillHeight(1)
		[
			SNew(SBorder).Padding(8)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
			[
				// A table row is wider than the window can be made on a small
				// screen: scroll both ways rather than wrap, which would break
				// the columns.
				SNew(SScrollBox)
				+ SScrollBox::Slot()
				[
					SNew(SScrollBox).Orientation(Orient_Horizontal)
					+ SScrollBox::Slot()
					[
						SAssignNew(S->ReportText, STextBlock)
						.Font(FCoreStyle::GetDefaultFontStyle("Mono", 9))
						.Text(LOCTEXT("Empty", "Choose the bodies and press Compute."))
					]
				]
			]
		];

	// ---- the Point masses tab: the cards on the left, the picture on the right ----
	TSharedRef<SWidget> MassesPanel =
		SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(0, 0, 0, 8)
		[
			MakeGroup(LOCTEXT("MassesGroup", "Point masses"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(S->RowList, SVerticalBox) ]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0) [ Button(LOCTEXT("AddMass", "Add point mass"), [S]() { S->AddRow(); }) ]
					+ SHorizontalBox::Slot().AutoWidth() [ Button(LOCTEXT("ApplyMasses", "Apply"), [S]() { S->Apply(); }) ]
				])
		]
		+ SScrollBox::Slot()
		[
			MakeInfoLabel(LOCTEXT("MassesHint",
				"A point mass is a concentrated mass that is not part of any body's shells: a battery, a motor, the payload, a camera. It is part of the model, saved with the project (masses: in the file), counted in the report whenever the body it hangs on is weighed, and hung on the structure by the structural analysis. Add one, then click the surface in the picture where its centre sits (the picture shows every solid body of the scene); the body it hangs on is the one you clicked unless you choose another in its list. Edit the centre by hand to move it off the surface (a motor's centre sits ahead of its mount). Mass and inertia (about the mass's own centre; Ixy, Ixz, Iyz the products as integral x y dm; all zero for a point) are as BUILT, in the project's units, not scaled with the model; coordinates are full-scale, like every coordinate in the application. 'Nearest nodes' is how many of the body's grid nodes nearest the centre carry it in the structural analysis (an RBE3-type attachment: the load shared out with the right resultant and moment, the nodes not stiffened; at least three, not in a line). When the body has become an operand of a boolean result that is analysed, the mass hangs on the result's surface at the same place."))
		];

	TSharedRef<SWidget> MassesTab =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
		[ SNew(SBox).WidthOverride(380) [ MassesPanel ] ]
		+ SHorizontalBox::Slot().FillWidth(1.0f)
		[
			SAssignNew(S->View, SAerotelicSceneView)
			.Overlay()
			[
				SNew(SOverlay)
				+ SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(8)
				[
					SNew(SBox).MaxDesiredWidth(640)
					[
						SNew(SBorder).Padding(FMargin(8, 6)).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.0f, 0.0f, 0.0f, 0.45f))
						[ SAssignNew(S->Caption, STextBlock).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.92f, 0.92f, 0.92f)).Text(LOCTEXT("PictureEmpty", "Every solid body of the scene, with the point masses on them.")) ]
					]
				]
			]
		];
	{
		TWeakPtr<FState> Weak = S;
		S->View->OnPickDown = [Weak](const FVector2D&, const FVector& O, const FVector& D) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->OnPickDown(O, D); };
		S->View->OnPickMove = [](const FVector2D&, const FVector&, const FVector&) { return true; };
		S->View->OnPickUp = [](const FVector2D&, const FVector&, const FVector&) { return true; };
		S->View->OnPickHover = [Weak](const FVector2D&, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->Selected.IsValid(); };
	}

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("ReportTab", "Report"), ReportTab });
	Tabs.Add({ LOCTEXT("MassesTab", "Point masses"), MassesTab });

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SAssignNew(S->Tabs, SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([S](const TSharedRef<SWindow>&) { S->Close(); }));

	S->RebuildRowList();
	Status(LOCTEXT("SelectBody", "Select the bodies to weigh; the Point masses tab hangs concentrated masses on the model."));
	ShowAerotelicDialogWindow(Window, Size);
	S->CollectShown();
	S->UpdatePicture();
}

#undef LOCTEXT_NAMESPACE
