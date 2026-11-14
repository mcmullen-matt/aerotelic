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
// hung on a body at a grid node picked with the mouse in the dialog's own
// picture. They are the document's (UAerotelicDocumentSubsystem::
// GetMassItems), saved with the project under `masses:`, counted in the
// report and hung on the structure by Analysis > Structural (an RBE3-type
// attachment on the body's nearest nodes to the picked point; a body that
// is an operand of a boolean result hands the point on to the result).

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
		void Quad(const FVector& A, const FVector& B, const FVector& Cc, const FVector& D, const FColor& Col) { Tri(A, B, Cc, Col); Tri(A, Cc, D, Col); }
	};

	FColor Srgb(float R, float G, float B) { return AAerotelicColoredMeshActor::VertexColorFromSRGB(R, G, B); }

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
		const FVector Helper = FMath::Abs(D.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector;
		const FVector U = FVector::CrossProduct(D, Helper).GetSafeNormal(), W = FVector::CrossProduct(D, U).GetSafeNormal();
		const FVector Corners[4] = { U * R + W * R, U * R - W * R, -U * R - W * R, -U * R + W * R };
		for (int32 i = 0; i < 4; ++i) G.Quad(A + Corners[i], A + Corners[(i + 1) % 4], B + Corners[(i + 1) % 4], B + Corners[i], Col);
	}

	const FColor kGrey = Srgb(0.74f, 0.75f, 0.77f);
	const FColor kBodyHot = Srgb(0.55f, 0.72f, 0.95f);
	const FColor kMass = Srgb(0.95f, 0.18f, 0.18f);
	const FColor kMassArmed = Srgb(1.0f, 0.75f, 0.2f);
	const FColor kNode = Srgb(1.0f, 0.62f, 0.15f);
	const FColor kBar = Srgb(0.95f, 0.85f, 0.4f);
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
	// units, as the file has it.
	struct FPointMassRow
	{
		FString Name = TEXT("mass");
		double Mass = 1.0;
		double X = 0.0, Y = 0.0, Z = 0.0;
		double I[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };   // Ixx, Iyy, Izz, Ixy, Ixz, Iyz about the centre
		FString AttachEntity, AttachBody;
		bool bAttachMirror = false;
		bool bHasPoint = false;
		double PX = 0.0, PY = 0.0, PZ = 0.0;
		int32 Nodes = 8;
		bool bPlaced = false;   // the centre has been set (by a pick or by hand)

		aerotelic::MassItem ToItem() const
		{
			aerotelic::MassItem M;
			M.name = TCHAR_TO_UTF8(*Name);
			M.mass = Mass;
			M.position = { X, Y, Z };
			for (int32 k = 0; k < 6; ++k) M.inertia[k] = I[k];
			M.attach_entity = TCHAR_TO_UTF8(*AttachEntity);
			M.attach_mirror = bAttachMirror;
			M.attach_body = TCHAR_TO_UTF8(*AttachBody);
			M.attach_nodes = FMath::Max(Nodes, 3);
			if (bHasPoint) M.attach_point = std::array<double, 3>{ PX, PY, PZ };
			return M;
		}
		static TSharedPtr<FPointMassRow> FromItem(const aerotelic::MassItem& M)
		{
			TSharedPtr<FPointMassRow> R = MakeShared<FPointMassRow>();
			R->Name = UTF8_TO_TCHAR(M.name.c_str());
			R->Mass = M.mass;
			R->X = M.position[0]; R->Y = M.position[1]; R->Z = M.position[2];
			for (int32 k = 0; k < 6; ++k) R->I[k] = M.inertia[k];
			R->AttachEntity = UTF8_TO_TCHAR(M.attach_entity.c_str());
			R->AttachBody = UTF8_TO_TCHAR(M.attach_body.c_str());
			R->bAttachMirror = M.attach_mirror;
			R->Nodes = M.attach_nodes;
			R->bHasPoint = M.attach_point.has_value();
			if (M.attach_point) { R->PX = (*M.attach_point)[0]; R->PY = (*M.attach_point)[1]; R->PZ = (*M.attach_point)[2]; }
			R->bPlaced = true;
			return R;
		}
		FString AttachLabel() const
		{
			if (AttachEntity.IsEmpty()) return TEXT("nothing -- pick a body in the picture");
			FString S = AttachEntity + (bAttachMirror ? TEXT(" (mirror)") : TEXT("")) + (AttachBody.IsEmpty() ? TEXT("") : TEXT(" / ") + AttachBody);
			if (bHasPoint) S += FString::Printf(TEXT(" at (%.4g, %.4g, %.4g)"), PX, PY, PZ);
			return S;
		}
	};

	// A body in the picture: whose, which, its placed (replica) mesh.
	struct FShownBody
	{
		TWeakObjectPtr<AAerotelicEntityActor> Entity;
		bool bMirror = false;
		FString Role;
		FString Label;
		aerotelic::TriMesh Mesh;
	};

	enum ETab { TabReport = 0, TabMasses = 1 };

	struct FState : public TSharedFromThis<FState>
	{
		TFunction<void(const FText&)> Status;
		TWeakPtr<SWindow> Window;
		aerotelic::UnitSystem Units;
		TArray<FAerotelicBodyChoicePtr> Choices;
		TSharedPtr<SAerotelicBodyPicker> Picker;
		TSharedPtr<SAerotelicTabStrip> Tabs;

		// The report.
		double Tolerance = 0.0;   // 0: each body's own grid; a figure: that chordal tolerance
		TSharedPtr<STextBlock> ReportText;
		FString ReportString;

		// The point masses.
		TArray<TSharedPtr<FPointMassRow>> Rows;
		TSharedPtr<SVerticalBox> RowList;
		TWeakPtr<FPointMassRow> Armed;   // the row the next pick in the picture attaches
		TSharedPtr<SAerotelicSceneView> View;
		TSharedPtr<STextBlock> Caption;
		TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>> Actors;
		bool bFramed = false;
		TArray<FShownBody> Shown;
		int32 HotBody = -1;   // the body under the last pick, drawn brighter

		static double ModelScale() { return AAerotelicEntityActor::GetModelScale(); }
		static bool IsReplica() { return !FMath::IsNearlyEqual(ModelScale(), 1.0, 1e-9); }

		void Message(const FText& Text) { AerotelicMessageBox(EAppMsgType::Ok, Text, LOCTEXT("Title", "Mass Properties")); }

		TArray<FAerotelicBooleanOperand> ChosenOperands() const
		{
			TArray<FAerotelicBooleanOperand> Out;
			if (Picker.IsValid()) for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen()) Out.Add(C->Operand);
			return Out;
		}

		// ---- the report ----
		void Compute()
		{
			const TArray<FAerotelicBooleanOperand> Bodies = ChosenOperands();
			if (Bodies.Num() == 0 && Rows.Num() == 0)
			{
				Message(LOCTEXT("NoneChosen", "Choose at least one body to weigh, or add a point mass."));
				return;
			}
			const double Started = FPlatformTime::Seconds();
			TArray<aerotelic::MassItem> Items;
			for (const TSharedPtr<FPointMassRow>& R : Rows) Items.Add(R->ToItem());
			const FAerotelicMassReport Report = ComputeAerotelicMassReport(Bodies, Tolerance, Units, &Items);
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

		// ---- the point masses: the document's, in and out ----
		void LoadRows()
		{
			Rows.Reset();
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			if (Doc == nullptr) return;
			for (const aerotelic::MassItem& M : Doc->GetMassItems()) Rows.Add(FPointMassRow::FromItem(M));
		}

		FString RowsProblem() const
		{
			TSet<FString> Names;
			for (const TSharedPtr<FPointMassRow>& R : Rows)
			{
				if (R->Name.TrimStartAndEnd().IsEmpty()) return TEXT("a point mass has no name");
				if (Names.Contains(R->Name)) return FString::Printf(TEXT("two point masses are named '%s'"), *R->Name);
				Names.Add(R->Name);
				const std::string Problem = aerotelic::MassItemProblem(R->ToItem());
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
			for (const TSharedPtr<FPointMassRow>& R : Rows) Items.Add(R->ToItem());
			Doc->SetMassItems(Items);
			int32 Loose = 0;
			for (const TSharedPtr<FPointMassRow>& R : Rows) Loose += R->AttachEntity.IsEmpty() ? 1 : 0;
			Status(FText::FromString(FString::Printf(TEXT("%d point mass%s kept with the project%s"), Rows.Num(), Rows.Num() == 1 ? TEXT("") : TEXT("es"),
				Loose ? *FString::Printf(TEXT(" (%d hang%s on nothing: they weigh in the report and are left out of the structural analysis)"), Loose, Loose == 1 ? TEXT("s") : TEXT("")) : TEXT(""))));
			UpdatePicture();
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

		// The chosen bodies' placed meshes, for the picture and the picking.
		void CollectShown()
		{
			Shown.Reset();
			HotBody = -1;
			if (!Picker.IsValid()) return;
			for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen())
			{
				AAerotelicEntityActor* E = C->Operand.Entity.Get();
				if (E == nullptr) continue;
				for (aerotelic::NamedMesh& M : E->GetPlacedMeshes(C->Operand.bMirror))
				{
					const FString Role = UTF8_TO_TCHAR(M.name.c_str());
					if (!C->Body.IsEmpty() && Role != C->Body) continue;
					FShownBody B;
					B.Entity = E;
					B.bMirror = C->Operand.bMirror;
					B.Role = Role;
					B.Label = C->Label;
					B.Mesh = std::move(M.mesh);
					Shown.Add(MoveTemp(B));
				}
			}
		}

		double Diagonal() const
		{
			aerotelic::Box3 Box;
			for (const FShownBody& B : Shown) for (const aerotelic::Vec3& V : B.Mesh.vertices) Box.Add(V);
			for (const TSharedPtr<FPointMassRow>& R : Rows) Box.Add(aerotelic::Vec3{ R->X, R->Y, R->Z } * ModelScale());
			return Box.IsEmpty() ? 1.0 : FMath::Max(Box.Diagonal(), 1e-9);
		}

		void UpdatePicture()
		{
			if (!View.IsValid()) return;
			const double Scale = ModelScale();
			const double Diag = Diagonal();
			FGeom Bodies;
			for (int32 b = 0; b < Shown.Num(); ++b)
			{
				const FShownBody& B = Shown[b];
				const FColor Col = b == HotBody ? kBodyHot : kGrey;
				for (const auto& T : B.Mesh.triangles)
				{
					const aerotelic::Vec3& A = B.Mesh.vertices[T[0]];
					const aerotelic::Vec3& Bv = B.Mesh.vertices[T[1]];
					const aerotelic::Vec3& Cv = B.Mesh.vertices[T[2]];
					Bodies.Tri(FVector(A.x, A.y, A.z), FVector(Bv.x, Bv.y, Bv.z), FVector(Cv.x, Cv.y, Cv.z), Col);
				}
			}
			FGeom Marks;
			double Biggest = 0.0, Total = 0.0;
			FVector First = FVector::ZeroVector;
			for (const TSharedPtr<FPointMassRow>& R : Rows) { Biggest = FMath::Max(Biggest, R->Mass); Total += R->Mass; First += FVector(R->X, R->Y, R->Z) * Scale * R->Mass; }
			FString Text;
			TSharedPtr<FPointMassRow> ArmedRow = Armed.Pin();
			for (int32 i = 0; i < Rows.Num(); ++i)
			{
				const TSharedPtr<FPointMassRow>& R = Rows[i];
				const FVector Centre = FVector(R->X, R->Y, R->Z) * Scale;
				const double Radius = (0.008 + 0.025 * FMath::Pow(R->Mass / FMath::Max(Biggest, 1e-300), 1.0 / 3.0)) * Diag;
				AddOctahedron(Marks, Centre, Radius, R == ArmedRow ? kMassArmed : kMass);
				if (R->bHasPoint)
				{
					const FVector Node = FVector(R->PX, R->PY, R->PZ) * Scale;
					AddOctahedron(Marks, Node, 0.006 * Diag, kNode);
					AddBar(Marks, Centre, Node, 0.0015 * Diag, kBar);
				}
				Text += FString::Printf(TEXT("%s%s: %.4g %s at (%.4g, %.4g, %.4g); hangs on %s%s"), i ? TEXT("\n") : TEXT(""), *R->Name, R->Mass, *UnitName(Units.MassName()),
					R->X, R->Y, R->Z, *R->AttachLabel(), R == ArmedRow ? TEXT("  <-- ARMED: click a body in the picture to pick its node") : TEXT(""));
			}
			if (Rows.Num() == 0) Text = TEXT("No point mass yet: Add one on the left, then click a body in the picture to pick the node it hangs on.");
			if (Total > 0.0)
			{
				const FVector Cg = First / Total;
				AddOctahedron(Marks, Cg, 0.01 * Diag, kCentre);
				Text += FString::Printf(TEXT("\nThe point masses together: %.5g %s, centre (green) at (%.4g, %.4g, %.4g) %s%s"), Total, *UnitName(Units.MassName()),
					Cg.X / Scale, Cg.Y / Scale, Cg.Z / Scale, *UnitName(Units.LengthName()), Rows.Num() > 1 ? TEXT("") : TEXT(" (the one mass)"));
			}
			Text += TEXT("\nRed: each mass at its centre, sized by its mass (amber: the armed one); orange: the node it hangs on, with a bar between. Left-drag orbits, right-drag pans, wheel zooms, Home frames.");
			if (IsReplica()) Text += TEXT("\nCoordinates here and on the left are the FULL-SCALE aircraft's; the picture shows the replica.");
			TArray<AActor*> ShownActors;
			AAerotelicColoredMeshActor* A0 = EnsureActor(0);
			AAerotelicColoredMeshActor* A1 = EnsureActor(1);
			if (A0 == nullptr || A1 == nullptr) return;
			A0->SetMesh(Bodies.V, Bodies.T, Bodies.N, Bodies.C);
			A0->SetEdges(false);
			A1->SetMesh(Marks.V, Marks.T, Marks.N, Marks.C);
			A1->SetEdges(false);
			ShownActors.Add(A0);
			ShownActors.Add(A1);
			View->SetShownActors(ShownActors);
			if (!bFramed) { View->FrameShown(); bFramed = true; }
			View->RequestRedraw();
			if (Caption.IsValid()) Caption->SetText(FText::FromString(Text));
		}

		// The picture is rebuilt from the picker's current choice.
		void RebuildPicture(bool bReframe)
		{
			CollectShown();
			if (bReframe) bFramed = false;
			UpdatePicture();
		}

		// A left press in the picture with a row armed: the ray's first hit
		// on a shown body, and that triangle's nearest vertex, become the
		// row's attachment (and its centre, the first time).
		bool OnPickDown(const FVector& Origin, const FVector& Dir)
		{
			TSharedPtr<FPointMassRow> Row = Armed.Pin();
			if (!Row.IsValid()) return false;
			double Best = 1e300;
			int32 BestBody = -1;
			FVector BestVertex = FVector::ZeroVector;
			for (int32 b = 0; b < Shown.Num(); ++b)
			{
				const aerotelic::TriMesh& M = Shown[b].Mesh;
				for (const auto& T : M.triangles)
				{
					const FVector A(M.vertices[T[0]].x, M.vertices[T[0]].y, M.vertices[T[0]].z);
					const FVector B(M.vertices[T[1]].x, M.vertices[T[1]].y, M.vertices[T[1]].z);
					const FVector C(M.vertices[T[2]].x, M.vertices[T[2]].y, M.vertices[T[2]].z);
					const double Tt = RayTriangle(Origin, Dir, A, B, C);
					if (Tt <= 0.0 || Tt >= Best) continue;
					Best = Tt;
					BestBody = b;
					const FVector Hit = Origin + Dir * Tt;
					BestVertex = A;
					if (FVector::DistSquared(Hit, B) < FVector::DistSquared(Hit, BestVertex)) BestVertex = B;
					if (FVector::DistSquared(Hit, C) < FVector::DistSquared(Hit, BestVertex)) BestVertex = C;
				}
			}
			if (BestBody < 0)
			{
				Status(LOCTEXT("Missed", "The click missed every body: click on a surface to pick the node the mass hangs on."));
				return true;
			}
			const FShownBody& B = Shown[BestBody];
			const double Scale = ModelScale();
			Row->AttachEntity = B.Entity.IsValid() ? B.Entity->EntityName : FString();
			Row->bAttachMirror = B.bMirror;
			Row->AttachBody = B.Role;
			Row->bHasPoint = true;
			Row->PX = BestVertex.X / Scale; Row->PY = BestVertex.Y / Scale; Row->PZ = BestVertex.Z / Scale;
			if (!Row->bPlaced)
			{
				Row->X = Row->PX; Row->Y = Row->PY; Row->Z = Row->PZ;
				Row->bPlaced = true;
			}
			HotBody = BestBody;
			Status(FText::FromString(FString::Printf(TEXT("'%s' hangs on %s at node (%.4g, %.4g, %.4g) %s -- edit its centre and figures on the left, then Apply"),
				*Row->Name, *B.Label, Row->PX, Row->PY, Row->PZ, *UnitName(Units.LengthName()))));
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
				const bool bArmed = Armed.Pin() == Row;
				RowList->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
				[
					SNew(SBorder).Padding(6).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(bArmed ? FLinearColor(0.2f, 0.16f, 0.05f) : FLinearColor(0.1f, 0.1f, 0.12f))
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center) [ SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("Point mass %d%s"), i + 1, bArmed ? TEXT(" -- armed: click a body in the picture") : TEXT("")))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
							+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
							[ Button(bArmed ? LOCTEXT("Disarm", "Stop picking") : LOCTEXT("Pick", "Pick node..."), [Self, Row]() { Self->Armed = Self->Armed.Pin() == Row ? nullptr : Row; Self->RebuildRowList(); Self->UpdatePicture(); }) ]
							+ SHorizontalBox::Slot().AutoWidth()
							[ Button(LOCTEXT("Delete", "Delete"), [Self, Row]() { Self->Rows.Remove(Row); if (Self->Armed.Pin() == Row) Self->Armed = nullptr; Self->RebuildRowList(); Self->UpdatePicture(); }) ]
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
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("HangsOn", "Hangs on:"), SNew(STextBlock).AutoWrapText(true).Text_Lambda([Row]() { return FText::FromString(Row->AttachLabel()); }), 100.0f) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 3) [ FormRow(LOCTEXT("Nodes", "Nearest nodes:"), NumberField([Row]() { return (double)Row->Nodes; }, [Row](double V) { Row->Nodes = FMath::Max(FMath::RoundToInt32(V), 3); }, 1.0, FText::GetEmpty(), 3.0, 200.0), 100.0f) ]
					]
				];
			}
		}

		void AddRow()
		{
			TSharedPtr<FPointMassRow> Row = MakeShared<FPointMassRow>();
			Row->Name = FString::Printf(TEXT("mass %d"), Rows.Num() + 1);
			Row->bPlaced = false;
			Rows.Add(Row);
			Armed = Row;
			RebuildRowList();
			UpdatePicture();
			Status(LOCTEXT("Armed", "Click a body in the picture: the node you hit is where the new mass hangs (and its centre, to begin with)."));
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
	S->Choices = Choices;
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
						S->RebuildPicture(true);
					}))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeInfoLabel(LOCTEXT("BodiesHint",
						"Each body weighs the way its Structure says (right-click it in the Scene tree): its material's density over the volume it encloses, a skin as density x area x thickness with its generated layers, or an asserted mass spread over its surface. One without a setting is reported as left out rather than counted as zero. The POINT MASSES of the next tab are counted too, each as a line of its own, whatever they hang on. The chosen bodies are also what the Point masses tab's picture shows."))
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
				"A point mass is a concentrated mass that is not part of any body's shells: a battery, a motor, the payload, a camera. Add one, then click a body in the picture: the grid node you hit is the node it hangs on (and its centre, to begin with -- move the centre by hand if the mass sits elsewhere, as a motor does ahead of its mount). Mass and inertia (about the mass's own centre; Ixy, Ixz, Iyz the products as integral x y dm; all zero for a point) are the item's as BUILT, in the project's units, not scaled with the model; coordinates are full-scale, like every coordinate in the application. 'Nearest nodes' is how many of the body's grid nodes around the picked node carry it in the structural analysis (an RBE3-type attachment: the load shared out with the right resultant and moment, the nodes not stiffened; at least three, not in a line). Apply keeps the masses with the project (masses: in the file). The report counts every point mass; the structural analysis hangs each on its body -- or, when that body has become an operand of a boolean result that is analysed, on the result's surface at the same point -- and leaves out one that hangs on nothing."))
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
						[ SAssignNew(S->Caption, STextBlock).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.92f, 0.92f, 0.92f)).Text(LOCTEXT("PictureEmpty", "The chosen bodies, with the point masses on them.")) ]
					]
				]
			]
		];
	{
		TWeakPtr<FState> Weak = S;
		S->View->OnPickDown = [Weak](const FVector2D&, const FVector& O, const FVector& D) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->OnPickDown(O, D); };
		S->View->OnPickMove = [](const FVector2D&, const FVector&, const FVector&) { return true; };
		S->View->OnPickUp = [](const FVector2D&, const FVector&, const FVector&) { return true; };
		S->View->OnPickHover = [Weak](const FVector2D&, const FVector&, const FVector&) { TSharedPtr<FState> P = Weak.Pin(); return P.IsValid() && P->Armed.IsValid(); };
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
	Status(LOCTEXT("SelectBody", "Select the bodies to weigh; the Point masses tab hangs concentrated masses on them."));
	ShowAerotelicDialogWindow(Window, Size);
	S->RebuildPicture(true);
}

#undef LOCTEXT_NAMESPACE
