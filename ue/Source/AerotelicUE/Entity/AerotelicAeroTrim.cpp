#include "AerotelicAeroTrim.h"

#include "Async/Async.h"
#include "Engine/World.h"

#include "AerotelicMeshing.h"
#include "Standalone/AerotelicColoredMeshActor.h"
#include "Standalone/AerotelicCameraPawn.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

// Last: OCCT's Handle() macro vs engine headers.
#include "aerotelic/mesh_boolean.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/wing_panels.hpp"
#include "AerotelicDocumentSubsystem.h"
#include "AerotelicEntityActor.h"
#include "AerotelicBooleanModel.h"
#include "AerotelicAeroLoads.h"
#include "AerotelicMassProperties.h"

namespace AerotelicAeroTrim
{
	namespace
	{
		constexpr double kPi = 3.14159265358979323846;

		// The points of a polyline placed as the entity sits (project units),
		// reflected when asked.
		std::vector<aerotelic::Vec3> PlacePoints(const std::vector<aerotelic::Vec3>& Points, const FTransform& Transform, const AAerotelicEntityActor* Reflect)
		{
			aerotelic::TriMesh M;
			M.vertices = Points;
			AerotelicMeshing::PlaceMesh(M, Transform);
			if (Reflect != nullptr)
			{
				for (aerotelic::Vec3& P : M.vertices)
				{
					const FVector R = Reflect->ReflectPoint(FVector(P.x, P.y, P.z));
					P = { R.X, R.Y, R.Z };
				}
			}
			return M.vertices;
		}

		// One entity's (or copy's) aero body before any fusing.
		struct FPiece
		{
			FShare Share;
			tripanel::Body Body;
			aerotelic::Box3 Bounds;
		};

		// The panelable entities an entity stands for: itself, or a boolean's
		// operands (recursively), each with whether it is the reflected copy.
		struct FLeaf { AAerotelicEntityActor* Entity = nullptr; bool bMirror = false; };
		void Leaves(AAerotelicEntityActor* E, bool bMirror, TArray<FLeaf>& Out, int32 Depth = 0)
		{
			if (E == nullptr || E->GetModel() == nullptr || Depth > 8) return;
			if (E->GetEntityType() == EntityType::Boolean)
			{
				const FAerotelicBooleanModel* M = E->GetModelAs<FAerotelicBooleanModel>();
				if (M == nullptr) return;
				for (const FAerotelicBooleanOperand& Op : M->Operands)
				{
					if (!Op.Entity.IsValid()) continue;
					Leaves(Op.Entity.Get(), bMirror != Op.bMirror, Out, Depth + 1);
				}
				return;
			}
			if (!IsPanelable(E->GetEntityType())) return;
			for (const FLeaf& L : Out) if (L.Entity == E && L.bMirror == bMirror) return;
			Out.Add({ E, bMirror });
		}

		// The leaves of a set of entities: every chosen entity, the operands
		// of a chosen boolean, and the reflected copy of a mirrored one when
		// the plane is off.
		TArray<FLeaf> AllLeaves(const TArray<AAerotelicEntityActor*>& Entities, bool bSymmetry)
		{
			TArray<FLeaf> Out;
			for (AAerotelicEntityActor* E : Entities)
			{
				Leaves(E, false, Out);
				if (E->IsMirrored() && !bSymmetry) Leaves(E, true, Out);
			}
			return Out;
		}

		aerotelic::Box3 BoundsOf(const aerotelic::TriMesh& M)
		{
			aerotelic::Box3 B;
			for (const aerotelic::Vec3& V : M.vertices) B.Add(V);
			return B;
		}

		bool Overlap(const aerotelic::Box3& A, const aerotelic::Box3& B, double Margin)
		{
			if (A.IsEmpty() || B.IsEmpty()) return false;
			return A.min.x <= B.max.x + Margin && B.min.x <= A.max.x + Margin
				&& A.min.y <= B.max.y + Margin && B.min.y <= A.max.y + Margin
				&& A.min.z <= B.max.z + Margin && B.min.z <= A.max.z + Margin;
		}

		FString Thousands(int64 N)
		{
			FString S = FString::Printf(TEXT("%lld"), N);
			for (int32 i = S.Len() - 3; i > 0; i -= 3) S.InsertAt(i, TEXT(","));
			return S;
		}

		// The aero mesh in the main view: one actor per body, grey, edged.
		TArray<TWeakObjectPtr<AAerotelicColoredMeshActor>> GAeroMeshActors;

		FString LengthUnitName()
		{
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			return Doc != nullptr ? FString(UTF8_TO_TCHAR(Doc->GetUnits().LengthName())) : FString();
		}
	}

	bool IsPanelable(EntityType T)
	{
		return T == EntityType::Wing || T == EntityType::Fuselage || T == EntityType::Nacelle || T == EntityType::LoftedNacelle || T == EntityType::Primitive || T == EntityType::Boolean;
	}

	TArray<AAerotelicEntityActor*> FSetup::LiveEntities() const
	{
		TArray<AAerotelicEntityActor*> Out;
		for (const TWeakObjectPtr<AAerotelicEntityActor>& W : Entities) if (W.IsValid() && W->GetModel() != nullptr) Out.Add(W.Get());
		return Out;
	}

	TArray<AAerotelicEntityActor*> PanelableEntities()
	{
		TArray<AAerotelicEntityActor*> Out;
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc == nullptr) return Out;
		for (const TWeakObjectPtr<AAerotelicEntityActor>& W : Doc->GetEntities())
		{
			if (W.IsValid() && W->GetModel() != nullptr && IsPanelable(W->GetEntityType())) Out.Add(W.Get());
		}
		return Out;
	}

	void DefaultReferences(const TArray<AAerotelicEntityActor*>& Entities, double& OutArea, double& OutLength)
	{
		double Area = 0.0, Length = 0.0;
		FBox FirstBody(ForceInit);
		for (const FLeaf& L : AllLeaves(Entities, true))
		{
			AAerotelicEntityActor* E = L.Entity;
			if (E->GetEntityType() == EntityType::Wing)
			{
				try
				{
					const double S = AAerotelicEntityActor::GetModelScale();
					const aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
					Area += W->planform().PlanformArea() * S * S;
					if (Length <= 0.0) Length = W->planform().RootChord() * S;
				}
				catch (const std::exception&) {}
			}
			else if (!FirstBody.IsValid)
			{
				FirstBody = E->GetPlacedBounds(false);
			}
		}
		if (Area <= 0.0 && FirstBody.IsValid)
		{
			const FVector Size = FirstBody.GetSize();
			Area = kPi / 4.0 * Size.Y * Size.Z;
			Length = Size.X;
		}
		OutArea = Area > 0.0 ? Area : 1.0;
		OutLength = Length > 0.0 ? Length : 1.0;
	}

	bool DefaultSymmetry(const TArray<AAerotelicEntityActor*>& Entities)
	{
		for (const FLeaf& L : AllLeaves(Entities, true))
		{
			AAerotelicEntityActor* E = L.Entity;
			if (E->IsMirrored()) return true;
			if (E->GetEntityType() == EntityType::Wing)
			{
				const aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
				if (W == nullptr) continue;
				if (W->symmetric) return true;
				// The root on the plane: the placed root quarter-chord point (the
				// bounds would miss it -- a dihedral at the root tilts the root
				// section through the plane by its thickness).
				try
				{
					aerotelic::Mat Q = W->planform().AirfoilCoordsToXyz(0.0, aerotelic::Mat::FromRows({ { 0.25, 0.0 } }));
					const std::vector<aerotelic::Vec3> Root = PlacePoints({ { Q(0, 0), Q(0, 1), Q(0, 2) } }, E->GetActorTransform(), nullptr);
					const FBox B = E->GetPlacedBounds(false);
					const double Size = B.IsValid ? FMath::Max(B.GetSize().GetMax(), 1.0) : 1.0;
					if (!Root.empty() && FMath::Abs(Root[0].y) < 1e-6 * Size) return true;
				}
				catch (const std::exception&) {}
			}
		}
		return false;
	}

	TArray<FControl> Controls(const TArray<AAerotelicEntityActor*>& Entities, int32& OutDefault)
	{
		TArray<FControl> Out;
		OutDefault = -1;
		double AftX = -1e300;
		for (const FLeaf& L : AllLeaves(Entities, true))
		{
			AAerotelicEntityActor* E = L.Entity;
			if (E->GetEntityType() != EntityType::Wing) continue;
			const aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
			if (W == nullptr) continue;
			const FBox B = E->GetPlacedBounds(false);
			for (int32 k = 0; k < static_cast<int32>(W->flaps.size()); ++k)
			{
				FControl C;
				C.Wing = E;
				C.Flap = k;
				C.Label = E->EntityName + TEXT(" / ") + UTF8_TO_TCHAR(W->flaps[k].name.c_str());
				Out.Add(C);
				const double X = B.IsValid ? B.GetCenter().X : 0.0;
				if (X > AftX) { AftX = X; OutDefault = Out.Num() - 1; }
			}
		}
		return Out;
	}

	bool MassAndCentre(const TArray<AAerotelicEntityActor*>& Entities, const aerotelic::UnitSystem& Units, double& OutMassKg, aerotelic::Vec3& OutCentre, FString& OutText)
	{
		const double Scale = AAerotelicEntityActor::GetModelScale();
		TArray<FAerotelicBooleanOperand> Ops;
		for (AAerotelicEntityActor* E : Entities)
		{
			FAerotelicBooleanOperand Op;
			Op.Entity = E;
			Ops.Add(Op);
			if (E->IsMirrored())
			{
				Op.bMirror = true;
				Ops.Add(Op);
			}
		}
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		const FAerotelicMassReport Report = ComputeAerotelicMassReport(Ops, 2e-3, Units, Doc != nullptr ? &Doc->GetMassItems() : nullptr);
		OutMassKg = Report.Assembly.mass * Units.MassToSI();
		OutCentre = Report.Assembly.centre;
		const aerotelic::Vec3 Full = OutCentre * (1.0 / Scale);
		OutText = FString::Printf(TEXT("Mass report of the chosen bodies: %d of %d bodies weighed%s, mass %.5g %s = %.5g kg, mass centre (%.4g, %.4g, %.4g) %s%s."),
			Report.IncludedCount, Report.Bodies.Num(), Report.PointMasses > 0 ? *FString::Printf(TEXT(" with %d point mass(es)"), Report.PointMasses) : TEXT(""),
			Report.Assembly.mass, UTF8_TO_TCHAR(Units.MassName()), OutMassKg, Full.x, Full.y, Full.z, UTF8_TO_TCHAR(Units.LengthName()),
			FMath::IsNearlyEqual(Scale, 1.0) ? TEXT("") : TEXT(" (full-scale)"));
		for (const FAerotelicMassLine& L : Report.Bodies) if (!L.bIncluded) OutText += TEXT("\n  left out: ") + L.Label + TEXT(": ") + L.Note;
		return OutMassKg > 0.0;
	}

	// ---- the case ----

	TSharedPtr<FCase> BuildCase(const FSetup& Setup, double AlphaDeg, double BetaDeg, const FControl* Control, double DeltaDeg)
	{
		TSharedPtr<FCase> C = MakeShared<FCase>();
		C->AlphaDeg = AlphaDeg;
		C->BetaDeg = BetaDeg;
		if (Control != nullptr && Control->IsSet())
		{
			C->Control = Control->Label;
			C->DeltaDeg = DeltaDeg;
		}
		const double Alpha = AlphaDeg * kPi / 180.0, Beta = BetaDeg * kPi / 180.0;
		C->Stream = { std::cos(Alpha) * std::cos(Beta), std::sin(Beta), std::sin(Alpha) * std::cos(Beta) };
		C->LiftDir = { -std::sin(Alpha), 0.0, std::cos(Alpha) };
		C->SideDir = aerotelic::Cross(C->LiftDir, C->Stream);
		const bool bSymmetry = Setup.bSymmetry;
		const double Scale = AAerotelicEntityActor::GetModelScale();

		// ---- the pieces: one aero body per leaf (or per body of a leaf) ----
		TArray<FPiece> Pieces;
		for (const FLeaf& L : AllLeaves(Setup.LiveEntities(), bSymmetry))
		{
			AAerotelicEntityActor* E = L.Entity;
			const FTransform Transform = E->GetActorTransform();
			const AAerotelicEntityActor* Reflect = L.bMirror ? E : nullptr;
			const FString Name = E->EntityName + (L.bMirror ? TEXT(" (mirror)") : TEXT(""));
			if (E->GetEntityType() == EntityType::Wing)
			{
				aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
				if (W == nullptr) { C->Skipped.Add(Name); continue; }
				// The trim's control: this wing's flap at the asked deflection
				// for the build, the model's own value put back after.
				const bool bOverride = Control != nullptr && Control->Wing.Get() == E && Control->Flap >= 0 && Control->Flap < static_cast<int32>(W->flaps.size());
				const double Saved = bOverride ? W->flaps[Control->Flap].deflection_deg : 0.0;
				if (bOverride) W->flaps[Control->Flap].deflection_deg = DeltaDeg;
				aerotelic::NamedMesh Aero;
				std::vector<std::vector<aerotelic::Vec3>> Te;
				std::vector<std::string> Notes;
				try
				{
					Aero = W->BuildAeroMesh(E->DisplayTolerance(), &Te, &Notes);
				}
				catch (...)
				{
					if (bOverride) W->flaps[Control->Flap].deflection_deg = Saved;
					throw;
				}
				if (bOverride) W->flaps[Control->Flap].deflection_deg = Saved;
				for (const std::string& N : Notes) C->Notes.Add(Name + TEXT(": ") + UTF8_TO_TCHAR(N.c_str()));
				FPiece P;
				P.Share.Name = Name;
				P.Share.bWing = true;
				P.Share.bTipToTip = W->symmetric;
				P.Share.bMirrorCopy = L.bMirror;
				P.Body.name = TCHAR_TO_UTF8(*Name);
				P.Body.mesh = Aero.mesh;
				AerotelicMeshing::PlaceMesh(P.Body.mesh, Transform);
				if (Reflect != nullptr) Reflect->ReflectMesh(P.Body.mesh);
				for (const std::vector<aerotelic::Vec3>& Line : Te) P.Body.trailing_edges.push_back(PlacePoints(Line, Transform, Reflect));
				// The span axis and the quarter-chord root and tip, for the loading cuts.
				FVector Axis = Transform.TransformVectorNoScale(FVector(0, 1, 0)).GetSafeNormal();
				std::vector<aerotelic::Vec3> Ends;
				for (double Eta : { 0.0, 1.0 })
				{
					aerotelic::Mat Q = W->planform().AirfoilCoordsToXyz(Eta, aerotelic::Mat::FromRows({ { 0.25, 0.0 } }));
					Ends.push_back({ Q(0, 0), Q(0, 1), Q(0, 2) });
				}
				Ends = PlacePoints(Ends, Transform, Reflect);
				if (Reflect != nullptr) Axis = (Reflect->ReflectPoint(Axis) - Reflect->ReflectPoint(FVector::ZeroVector)).GetSafeNormal();
				P.Share.SpanAxis = Axis;
				P.Share.RootPoint = FVector(Ends[0].x, Ends[0].y, Ends[0].z);
				P.Share.TipPoint = FVector(Ends[1].x, Ends[1].y, Ends[1].z);
				// The chord at the centre of each of the kLoadCuts bands the
				// loading is cut at (never the root ring or the tip cap).
				for (int32 k = 0; k < kLoadCuts; ++k) P.Share.CutChords.Add(W->planform().ChordAt((k + 0.5) / kLoadCuts) * Scale);
				P.Share.Triangles = static_cast<int32>(P.Body.mesh.triangles.size());
				P.Bounds = BoundsOf(P.Body.mesh);
				Pieces.Add(MoveTemp(P));
			}
			else
			{
				std::vector<aerotelic::NamedMesh> Meshes = E->GetPlacedMeshes(L.bMirror);
				if (Meshes.empty()) { C->Skipped.Add(Name); continue; }
				std::vector<aerotelic::NamedLoft> Lofts;
				try { Lofts = E->GetModel()->BuildLofts(); } catch (const std::exception&) {}
				for (size_t b = 0; b < Meshes.size(); ++b)
				{
					if (Meshes[b].mesh.triangles.empty()) continue;
					FPiece P;
					P.Share.Name = Meshes.size() == 1 ? Name : Name + TEXT(" / ") + UTF8_TO_TCHAR(Meshes[b].name.c_str());
					P.Share.bMirrorCopy = L.bMirror;
					P.Body.name = TCHAR_TO_UTF8(*P.Share.Name);
					P.Body.mesh = Meshes[b].mesh;
					// The trailing edge the loft of the same name tags (one loft:
					// the body's).
					for (const aerotelic::NamedLoft& Lf : Lofts)
					{
						if (Lofts.size() > 1 && Lf.name != Meshes[b].name) continue;
						for (const std::vector<aerotelic::Vec3>& Line : aerotelic::panel::LoftTrailingEdgePolylines(Lf)) P.Body.trailing_edges.push_back(PlacePoints(Line, Transform, Reflect));
					}
					P.Share.Triangles = static_cast<int32>(P.Body.mesh.triangles.size());
					P.Bounds = BoundsOf(P.Body.mesh);
					Pieces.Add(MoveTemp(P));
				}
			}
		}
		if (Pieces.Num() == 0) throw std::runtime_error("none of the chosen bodies has a mesh to panel");

		// ---- the mesh check ----
		aerotelic::Box3 All;
		for (const FPiece& P : Pieces) All.Add(P.Bounds);
		const double Diag = FMath::Max(All.Diagonal(), 1e-300);
		TArray<tripanel::MeshStats> Stats;
		for (FPiece& P : Pieces)
		{
			tripanel::MeshStats S = tripanel::ComputeMeshStats(P.Body.mesh);
			if (!S.closed) C->Warnings.Add(P.Share.Name + TEXT(": the mesh is not a closed surface"));
			else if (!S.outward) C->Warnings.Add(P.Share.Name + TEXT(": the mesh faces inward"));
			P.Body.trailing_edge_tolerance = 1e-6 * Diag;
			Stats.Add(S);
		}

		// ---- fusing: bodies whose bounds overlap, by the mesh boolean ----
		TArray<int32> Group;
		for (int32 i = 0; i < Pieces.Num(); ++i) Group.Add(i);
		auto Root = [&Group](int32 i) { while (Group[i] != i) i = Group[i]; return i; };
		if (Setup.bFuse)
		{
			for (int32 a = 0; a < Pieces.Num(); ++a)
				for (int32 b = a + 1; b < Pieces.Num(); ++b)
					if (Overlap(Pieces[a].Bounds, Pieces[b].Bounds, 0.0)) Group[Root(b)] = Root(a);
		}
		TMap<int32, TArray<int32>> Groups;
		for (int32 i = 0; i < Pieces.Num(); ++i) Groups.FindOrAdd(Root(i)).Add(i);
		TArray<int32> GroupOrder;
		Groups.GetKeys(GroupOrder);
		GroupOrder.Sort();
		for (int32 g : GroupOrder)
		{
			TArray<int32>& Members = Groups[g];
			Members.Sort();
			if (Members.Num() == 1)
			{
				FPiece& P = Pieces[Members[0]];
				P.Share.Body = static_cast<int32>(C->Bodies.size());
				C->Owner.push_back(std::vector<int32>(P.Body.mesh.triangles.size(), C->Shares.Num()));
				C->Shares.Add(P.Share);
				C->Bodies.push_back(P.Body);
				continue;
			}
			// Fold the fuse. The boolean refuses an EXACT degenerate contact (a
			// vertex of one body on a face of the other -- a fin root ending
			// on the plane of a boom's end cap, 2026-11-16); the second body
			// is then tried again nudged by a millionth of the configuration
			// along an oblique direction, which no modelled surface lies along,
			// and the nudge is noted. A fuse that still fails leaves the
			// members separate.
			aerotelic::TriMesh Fused = Pieces[Members[0]].Body.mesh;
			FString Label = Pieces[Members[0]].Share.Name;
			bool bOk = true;
			FString Why;
			for (int32 k = 1; k < Members.Num() && bOk; ++k)
			{
				try
				{
					Fused = aerotelic::MeshBoolean(Fused, Pieces[Members[k]].Body.mesh, aerotelic::MeshBooleanOp::Fuse);
					Label += TEXT(" + ") + Pieces[Members[k]].Share.Name;
				}
				catch (const aerotelic::MeshBooleanError& E)
				{
					const double Eps = 1e-6 * Diag;
					// In the plane of symmetry (no y), so a centreline body stays on
					// y = 0: a fin nudged off it by a millionth left its trailing edge
					// a whisker off the plane (2026-11-16).
					const aerotelic::Vec3 Shift{ Eps * 0.7071067812, 0.0, Eps * 0.7071067812 };
					try
					{
						aerotelic::TriMesh Nudged = Pieces[Members[k]].Body.mesh;
						Nudged.Translate(Shift);
						Fused = aerotelic::MeshBoolean(Fused, Nudged, aerotelic::MeshBooleanOp::Fuse);
						Label += TEXT(" + ") + Pieces[Members[k]].Share.Name;
						C->Notes.Add(FString::Printf(TEXT("%s touched %s exactly (%s) and was fused nudged by %.2g %s"), *Pieces[Members[k]].Share.Name, *Label, UTF8_TO_TCHAR(E.what()), Eps, *LengthUnitName()));
					}
					catch (const std::exception& E2)
					{
						bOk = false;
						Why = FString::Printf(TEXT("%s with %s: %s"), *Label, *Pieces[Members[k]].Share.Name, UTF8_TO_TCHAR(E2.what()));
					}
				}
				catch (const std::exception& E)
				{
					bOk = false;
					Why = FString::Printf(TEXT("%s with %s: %s"), *Label, *Pieces[Members[k]].Share.Name, UTF8_TO_TCHAR(E.what()));
				}
			}
			if (!bOk || Fused.triangles.empty())
			{
				C->Warnings.Add(TEXT("could not fuse ") + Why + TEXT(" -- these bodies are paneled separately, so the panels of one inside another are not a wetted surface and the pressures near their junction are not to be trusted"));
				for (int32 m : Members)
				{
					FPiece& P = Pieces[m];
					P.Share.Body = static_cast<int32>(C->Bodies.size());
					C->Owner.push_back(std::vector<int32>(P.Body.mesh.triangles.size(), C->Shares.Num()));
					C->Shares.Add(P.Share);
					C->Bodies.push_back(P.Body);
				}
				continue;
			}
			// Who owns each fused triangle: the member whose surface its
			// centroid lies on (the pieces of a cut triangle lie on the
			// original surface too).
			std::vector<std::unique_ptr<aerotelic::MeshSdf>> Sdfs;
			for (int32 m : Members) Sdfs.push_back(std::make_unique<aerotelic::MeshSdf>(Pieces[m].Body.mesh));
			const int32 FirstShare = C->Shares.Num();
			std::vector<int32> Owner(Fused.triangles.size(), FirstShare);
			for (size_t t = 0; t < Fused.triangles.size(); ++t)
			{
				const auto& Tr = Fused.triangles[t];
				const aerotelic::Vec3 Cc = (Fused.vertices[Tr[0]] + Fused.vertices[Tr[1]] + Fused.vertices[Tr[2]]) * (1.0 / 3.0);
				double Best = 1e300;
				int32 Who = 0;
				for (int32 k = 0; k < Members.Num(); ++k)
				{
					const double D = FMath::Abs(Sdfs[k]->Distance(Cc));
					if (D < Best) { Best = D; Who = k; }
				}
				Owner[t] = FirstShare + Who;
			}
			tripanel::Body Body;
			Body.name = TCHAR_TO_UTF8(*Label);
			Body.mesh = MoveTemp(Fused);
			Body.trailing_edge_tolerance = 1e-6 * Diag;
			for (int32 m : Members)
			{
				FPiece& P = Pieces[m];
				for (const auto& Line : P.Body.trailing_edges) Body.trailing_edges.push_back(Line);
				P.Share.Body = static_cast<int32>(C->Bodies.size());
				C->Shares.Add(P.Share);
			}
			C->FusedGroups.Add(FString::Printf(TEXT("%s (%s triangles)"), *Label, *Thousands(static_cast<int64>(Body.mesh.triangles.size()))));
			C->Owner.push_back(MoveTemp(Owner));
			C->Bodies.push_back(MoveTemp(Body));
		}

		// ---- the report ----
		C->Triangles = 0;
		for (const tripanel::Body& B : C->Bodies) C->Triangles += static_cast<int32>(B.mesh.triangles.size());
		FString R;
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		const FString LengthUnit = Doc != nullptr ? UTF8_TO_TCHAR(Doc->GetUnits().LengthName()) : TEXT("");
		for (int32 i = 0; i < Pieces.Num(); ++i)
		{
			const tripanel::MeshStats& S = Stats[i];
			R += FString::Printf(TEXT("%s: %s triangles, edges %.4g to %.4g (median %.4g) %s, worst aspect %.0f%s%s%s\n"),
				*Pieces[i].Share.Name, *Thousands(S.triangles), S.min_edge, S.max_edge, S.median_edge, *LengthUnit, S.worst_aspect,
				S.slivers > 0 ? *FString::Printf(TEXT(" (%d slivers over 20)"), S.slivers) : TEXT(""),
				S.closed ? TEXT("") : TEXT(", NOT CLOSED"), S.closed && !S.outward ? TEXT(", INWARD") : TEXT(""));
		}
		for (const FString& G : C->FusedGroups) R += TEXT("Fused into one surface: ") + G + TEXT("\n");
		// Bodies left separate that come within a panel of each other.
		for (int32 a = 0; a < Pieces.Num(); ++a)
		{
			for (int32 b = a + 1; b < Pieces.Num(); ++b)
			{
				if (Pieces[a].Share.Body == Pieces[b].Share.Body) continue;
				const double Near = FMath::Max(Stats[a].median_edge, Stats[b].median_edge);
				if (!Overlap(Pieces[a].Bounds, Pieces[b].Bounds, Near)) continue;
				const int32 N = tripanel::TrianglesNear(Pieces[a].Body.mesh, Pieces[b].Body.mesh, Near);
				if (N > 0) C->Warnings.Add(FString::Printf(TEXT("%d triangle(s) of %s lie within a panel of %s, which is paneled separately: their pressures are not to be trusted"), N, *Pieces[a].Share.Name, *Pieces[b].Share.Name));
			}
		}
		R += FString::Printf(TEXT("%s panels in all%s"), *Thousands(C->Triangles), bSymmetry ? TEXT(" before the cut at the symmetry plane (the image stands in for y < 0)") : TEXT(""));
		for (const FString& W : C->Warnings) R += TEXT("\nWARNING: ") + W;
		C->MeshReport = R;
		return C;
	}

	tripanel::Options SolveOptions(const FSetup& Setup, const aerotelic::Vec3* MomentReference)
	{
		tripanel::Options Opt;
		if (Setup.bSymmetry) Opt.symmetry_y = 0.0;
		Opt.reference_area = Setup.ReferenceArea > 0.0 ? Setup.ReferenceArea : 1.0;
		Opt.wake_length = Setup.WakeChords * (Setup.ReferenceLength > 0.0 ? Setup.ReferenceLength : 1.0);
		if (MomentReference != nullptr) Opt.moment_reference = *MomentReference;
		return Opt;
	}

	void SolveCaseNow(FCase& C, const tripanel::Options& Options)
	{
		C.MomentReference = Options.moment_reference;
		C.bImagedSolve = Options.symmetry_y.has_value();
		C.ReferenceAreaSolved = Options.reference_area > 0.0 ? Options.reference_area : 1.0;
		C.Sol = tripanel::Solve(C.Bodies, C.Stream, Options);
		C.bSolved = true;
	}

	void Derive(FCase& C, double RefLength)
	{
		const double Chord = RefLength > 0.0 ? RefLength : 1.0;
		auto WindAxes = [&C, Chord](const aerotelic::Vec3& F, const aerotelic::Vec3& M)
		{
			FCoefs K;
			K.CL = aerotelic::Dot(F, C.LiftDir);
			K.CD = aerotelic::Dot(F, C.Stream);
			K.CY = aerotelic::Dot(F, C.SideDir);
			K.Cl = aerotelic::Dot(M, C.Stream) / Chord;
			K.Cm = aerotelic::Dot(M, C.SideDir) / Chord;
			K.Cn = aerotelic::Dot(M, C.LiftDir) / Chord;
			return K;
		};
		const FCoefs Total = WindAxes(C.Sol.force_coefficient, C.Sol.moment_coefficient);
		C.CL = Total.CL; C.CD = Total.CD; C.CY = Total.CY; C.Cl = Total.Cl; C.Cm = Total.Cm; C.Cn = Total.Cn;
		// The shares' masks over the solved bodies' panels (the cut at the
		// plane re-indexed them: BodyResult::parent leads back).
		for (int32 s = 0; s < C.Shares.Num(); ++s)
		{
			FShare& Sh = C.Shares[s];
			Sh.Mask.clear();
			if (Sh.Body < 0 || Sh.Body >= static_cast<int32>(C.Sol.bodies.size())) continue;
			const tripanel::BodyResult& BR = C.Sol.bodies[Sh.Body];
			Sh.Mask.assign(BR.panels.size(), 0);
			const std::vector<int32>& Owner = C.Owner[Sh.Body];
			for (size_t t = 0; t < BR.panels.size(); ++t)
			{
				const int Parent = t < BR.parent.size() ? BR.parent[t] : static_cast<int>(t);
				if (Parent >= 0 && Parent < static_cast<int>(Owner.size()) && Owner[Parent] == s) Sh.Mask[t] = 1;
			}
		}
		// Per share: its panels' force and moment, with the image when the
		// solve had the plane.
		C.ByBody.Reset();
		for (FShare& Sh : C.Shares)
		{
			aerotelic::Vec3 F{ 0, 0, 0 }, M{ 0, 0, 0 };
			if (Sh.Body >= 0 && Sh.Body < static_cast<int32>(C.Sol.bodies.size()))
			{
				const tripanel::BodyResult& BR = C.Sol.bodies[Sh.Body];
				for (size_t t = 0; t < BR.panels.size(); ++t)
				{
					if (!Sh.Mask.empty() && !Sh.Mask[t]) continue;
					const tripanel::PanelResult& P = BR.panels[t];
					if (!(P.area > 0.0)) continue;
					const aerotelic::Vec3 f = P.normal * (-P.cp * P.area);
					F += f;
					M += aerotelic::Cross(P.centre - C.MomentReference, f);
					if (C.bImagedSolve)
					{
						const aerotelic::Vec3 fi{ f.x, -f.y, f.z };
						const aerotelic::Vec3 ci{ P.centre.x, -P.centre.y, P.centre.z };
						F += fi;
						M += aerotelic::Cross(ci - C.MomentReference, fi);
					}
				}
			}
			Sh.bImaged = C.bImagedSolve;
			const double S = C.ReferenceAreaSolved > 0.0 ? C.ReferenceAreaSolved : 1.0;
			C.ByBody.Add(WindAxes(F * (1.0 / S), M * (1.0 / S)));
		}
		C.CpMin = 1.0;
		C.CpMax = -1e9;
		C.NBody = 0;
		C.NWake = static_cast<int32>(C.Sol.wakes.size());
		TArray<double> Cps;
		for (const tripanel::BodyResult& BR : C.Sol.bodies)
		{
			for (const tripanel::PanelResult& P : BR.panels)
			{
				if (!(P.area > 0.0)) continue;
				if (FMath::IsFinite(P.cp)) Cps.Add(P.cp);
				++C.NBody;
			}
		}
		if (Cps.Num() > 0)
		{
			Cps.Sort();
			C.CpMin = Cps[FMath::Clamp(FMath::FloorToInt32(0.02 * (Cps.Num() - 1)), 0, Cps.Num() - 1)];
			C.CpMax = Cps[FMath::Clamp(FMath::CeilToInt32(0.98 * (Cps.Num() - 1)), 0, Cps.Num() - 1)];
		}
	}

	TSharedPtr<FAerotelicAeroLoadCase> MakeLoadCase(const FCase& C, const FSetup& Setup, const FString& Name, double DynamicPressure, const FCondition* Trim, const FOutcome* Outcome)
	{
		TSharedPtr<FAerotelicAeroLoadCase> Case = MakeShared<FAerotelicAeroLoadCase>();
		Case->Name = Name;
		Case->DynamicPressure = DynamicPressure;
		Case->AlphaDeg = C.AlphaDeg;
		Case->BetaDeg = C.BetaDeg;
		Case->DeltaDeg = C.DeltaDeg;
		Case->Control = C.Control;
		Case->CL = C.CL;
		Case->Cm = C.Cm;
		Case->ReferenceArea = Setup.ReferenceArea > 0.0 ? Setup.ReferenceArea : 1.0;
		Case->Stream = C.Stream;
		if (Trim != nullptr && Outcome != nullptr)
		{
			Case->bTrimmed = true;
			Case->LoadFactor = Trim->LoadFactor;
			Case->Weight = Outcome->Weight;
			Case->MassCentre = Trim->MassCentre;
		}
		for (const FShare& Sh : C.Shares)
		{
			// The entity's name (a mirrored copy counts for its entity).
			FString N = Sh.Name;
			N.RemoveFromEnd(TEXT(" (mirror)"));
			int32 Slash = INDEX_NONE;
			if (N.FindChar(TEXT('/'), Slash) && Slash > 1) N = N.Left(Slash - 1);
			Case->Entities.AddUnique(N);
		}
		int32 Network = 0;
		for (const tripanel::BodyResult& BR : C.Sol.bodies)
		{
			for (const tripanel::PanelResult& P : BR.panels)
			{
				if (!(P.area > 0.0)) continue;
				aerotelic::aero::PressureSample S;
				S.centre = P.centre;
				S.normal = P.normal;
				S.area = P.area;
				S.cp_upper = P.cp;
				S.cp_lower = P.cp;
				S.two_sided = false;
				// Every panel its own network: a triangle has no grid
				// neighbours to average with, and a face of the structure that
				// IS this triangle takes its value exactly.
				S.network = Network++;
				S.row = 0;
				S.col = 0;
				Case->Samples.push_back(S);
				if (C.bImagedSolve)
				{
					aerotelic::aero::PressureSample M = S;
					M.centre.y = -M.centre.y;
					M.normal.y = -M.normal.y;
					M.network = Network++;
					Case->Samples.push_back(M);
				}
			}
		}
		return Case;
	}

	// ---- the aero mesh in the main view ----

	void HideAeroMeshInViewport()
	{
		for (TWeakObjectPtr<AAerotelicColoredMeshActor>& A : GAeroMeshActors) if (A.IsValid()) A->Destroy();
		GAeroMeshActors.Reset();
	}

	bool IsAeroMeshShown()
	{
		for (const TWeakObjectPtr<AAerotelicColoredMeshActor>& A : GAeroMeshActors) if (A.IsValid()) return true;
		return false;
	}

	void ShowAeroMeshInViewport(const FCase& C)
	{
		HideAeroMeshInViewport();
		AAerotelicCameraPawn* Pawn = AAerotelicCameraPawn::FindInGameWorld();
		UWorld* World = Pawn != nullptr ? Pawn->GetWorld() : nullptr;
		if (World == nullptr) return;
		const FColor Grey = AAerotelicColoredMeshActor::VertexColorFromSRGB(0.78f, 0.80f, 0.84f);
		for (const tripanel::Body& B : C.Bodies)
		{
			TArray<FVector> Vertices;
			TArray<int32> Triangles;
			TArray<FVector> Normals;
			TArray<FColor> Colors;
			// Flat faces: three vertices per triangle, both windings, so the
			// panels show as the facets they are.
			for (const auto& T : B.mesh.triangles)
			{
				const aerotelic::Vec3& A = B.mesh.vertices[T[0]];
				const aerotelic::Vec3& Bv = B.mesh.vertices[T[1]];
				const aerotelic::Vec3& Cv = B.mesh.vertices[T[2]];
				const aerotelic::Vec3 N = aerotelic::Normalized(aerotelic::Cross(Bv - A, Cv - A));
				const int32 Base = Vertices.Num();
				for (const aerotelic::Vec3* P : { &A, &Bv, &Cv })
				{
					Vertices.Add(FVector(P->x, P->y, P->z));
					Normals.Add(FVector(N.x, N.y, N.z));
					Colors.Add(Grey);
				}
				Triangles.Append({ Base, Base + 1, Base + 2, Base, Base + 2, Base + 1 });
			}
			if (Vertices.Num() == 0) continue;
			FActorSpawnParameters Params;
			Params.ObjectFlags |= RF_Transient;
			AAerotelicColoredMeshActor* Actor = World->SpawnActor<AAerotelicColoredMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
			if (Actor == nullptr) continue;
			Actor->SetMesh(Vertices, Triangles, Normals, Colors);
			Actor->SetEdges(true, FLinearColor(0.05f, 0.05f, 0.08f));
			Actor->SetVisibleInCapturesOnly(false);
			GAeroMeshActors.Add(Actor);
		}
	}

	// ---- the runner ----

	void FRunner::Start(const FSetup& InSetup, const FCondition& InCondition, const aerotelic::UnitSystem& InUnits,
		TFunction<void(const FText&)> InOnStatus, TFunction<void(const FOutcome&)> InOnDone)
	{
		Setup = InSetup;
		Condition = InCondition;
		Units = InUnits;
		OnStatus = InOnStatus;
		OnDone = InOnDone;
		Outcome = FOutcome();
		Driver.Reset();
		LastCase.Reset();
		BestCase.Reset();
		MeshText.Empty();
		StopReason.Empty();
		bCancel = false;
		bRunning = true;
		++Generation;

		if (Setup.LiveEntities().Num() == 0) { Finish(false, TEXT("no body to panel")); return; }
		if (!(Condition.MassKg > 0.0)) { Finish(false, TEXT("no mass to trim for: give the bodies a structure, or type a mass")); return; }
		Outcome.DynamicPressure = aerotelic::aero::DynamicPressure(Condition.Density, Condition.Speed);
		if (!(Outcome.DynamicPressure > 0.0)) { Finish(false, TEXT("the speed and the density must be positive")); return; }
		Outcome.Weight = Condition.MassKg * 9.80665;
		Outcome.ReferenceAreaSI = (Setup.ReferenceArea > 0.0 ? Setup.ReferenceArea : 1.0) * Units.AreaToSI();
		Outcome.CLRequired = aerotelic::aero::RequiredLiftCoefficient(Condition.LoadFactor, Outcome.Weight, Outcome.DynamicPressure, Outcome.ReferenceAreaSI);

		aerotelic::aero::TrimDriver::Settings S;
		S.CL_target = Outcome.CLRequired;
		S.control = Condition.Control.IsSet();
		S.tolerance.CL_fraction = Condition.TolCLFraction;
		S.tolerance.CL_floor = Condition.TolCLFloor;
		S.tolerance.Cm = Condition.TolCm;
		S.step = Condition.Step != 0.0 ? Condition.Step : 1.0;
		S.max_step_deg = Condition.MaxStepDeg;
		S.jacobian_move_deg = Condition.JacobianMoveDeg;
		S.stall_ratio = Condition.StallRatio;
		S.max_iterations = FMath::Max(Condition.MaxIterations, 1);
		S.max_jacobians = FMath::Max(Condition.MaxJacobians, 1);
		Driver = MakeShared<aerotelic::aero::TrimDriver>(S, Condition.Alpha0, Condition.Delta0);
		Advance();
	}

	void FRunner::Advance()
	{
		if (!Driver.IsValid()) { Finish(false, TEXT("no trim is running")); return; }
		if (bCancel.load()) { Finish(false, TEXT("cancelled")); return; }
		using Ask = aerotelic::aero::TrimDriver::Ask;
		switch (Driver->Next())
		{
		case Ask::Derivatives:
			RequestDerivatives();
			return;
		case Ask::State:
			RequestState();
			return;
		case Ask::Done:
			FinishFromDriver();
			return;
		case Ask::Failed:
		default:
			Finish(false, (MeshText.IsEmpty() ? FString() : TEXT("Aero mesh:\n") + MeshText) + HistoryText()
				+ TEXT("\nNO TRIM: ") + UTF8_TO_TCHAR(Driver->Note().c_str()));
			return;
		}
	}

	void FRunner::RequestDerivatives()
	{
		const bool bControl = Condition.Control.IsSet();
		const FControl* Ctl = bControl ? &Condition.Control : nullptr;
		const aerotelic::aero::TrimPoint B = Driver->BaseState(), A = Driver->StepAlphaState(), D = Driver->StepDeltaState();
		const bool bHaveBase = Driver->BaseAlreadySolved() && LastCase.IsValid();
		if (OnStatus)
		{
			OnStatus(FText::FromString(FString::Printf(
				TEXT("Trim '%s': %s the derivatives about α = %.3f°, δ = %.3f° (C_L required %.4f)..."),
				*Condition.Name, Driver->Jacobians() == 0 ? TEXT("measuring") : TEXT("re-measuring"), B.alpha_deg, B.delta_deg, Outcome.CLRequired)));
		}
		TSharedPtr<FCase> BaseCase, AlphaCase, DeltaCase;
		TArray<TSharedPtr<FCase>> ToSolve;
		try
		{
			if (bHaveBase)
			{
				BaseCase = LastCase;
			}
			else
			{
				BaseCase = BuildCase(Setup, B.alpha_deg, 0.0, Ctl, B.delta_deg);
				ToSolve.Add(BaseCase);
			}
			AlphaCase = BuildCase(Setup, A.alpha_deg, 0.0, Ctl, A.delta_deg);
			ToSolve.Add(AlphaCase);
			if (bControl)
			{
				DeltaCase = BuildCase(Setup, D.alpha_deg, 0.0, Ctl, D.delta_deg);
				ToSolve.Add(DeltaCase);
			}
		}
		catch (const std::exception& E)
		{
			Finish(false, FString(TEXT("the bodies could not be meshed for the panel method: ")) + UTF8_TO_TCHAR(E.what()));
			return;
		}
		if (MeshText.IsEmpty()) MeshText = BaseCase->MeshReport;
		Outcome.Solves += ToSolve.Num();
		TWeakPtr<FRunner> Weak = AsShared();
		SolveCases(ToSolve, SolveOptions(Setup, &Condition.MassCentre), [Weak, BaseCase, AlphaCase, DeltaCase, bHaveBase](bool bOk, const FString& Why)
		{
			TSharedPtr<FRunner> Self = Weak.Pin();
			if (!Self.IsValid()) return;
			if (!bOk) { Self->Finish(false, Why); return; }
			// Every state a derivative is taken between must itself be an
			// operating point, or the derivative is meaningless.
			FString Bad;
			const bool bBaseOk = bHaveBase || Self->Accept(*BaseCase, Bad);
			const bool bAlphaOk = bBaseOk && Self->Accept(*AlphaCase, Bad);
			const bool bDeltaOk = bAlphaOk && (!DeltaCase.IsValid() || Self->Accept(*DeltaCase, Bad));
			if (!bDeltaOk)
			{
				// A state of the derivative set is not an operating point, so no
				// derivative taken on it means anything. Stop -- but keep the best
				// HEALTHY iterate reached so far rather than throwing the run away.
				Self->StopReason = Bad;
				if (Self->Driver->HasBest() && Self->BestCase.IsValid()) { Self->FinishFromDriver(); return; }
				Self->Driver->Fail(TCHAR_TO_UTF8(*Bad));
				Self->Advance();
				return;
			}
			auto Point = [](const FCase& C) { return aerotelic::aero::TrimPoint{ C.AlphaDeg, C.DeltaDeg, C.CL, C.Cm }; };
			Self->LastCase = BaseCase;
			Self->Driver->FeedDerivatives(Point(*BaseCase), Point(*AlphaCase), DeltaCase.IsValid() ? Point(*DeltaCase) : Point(*BaseCase));
			Self->Outcome.Jacobian = Self->Driver->Jacobian();
			Self->Outcome.Jacobians = Self->Driver->Jacobians();
			if (!bHaveBase) Self->Note(BaseCase, true, false);
			else if (Self->Outcome.History.Num() > 0) Self->Outcome.History.Last().bJacobian = true;
			Self->Advance();
		});
	}

	void FRunner::RequestState()
	{
		const bool bControl = Condition.Control.IsSet();
		const FControl* Ctl = bControl ? &Condition.Control : nullptr;
		const aerotelic::aero::TrimPoint N = Driver->NextState();
		const bool bClamped = Driver->LastStepClamped();
		if (OnStatus)
		{
			OnStatus(FText::FromString(FString::Printf(
				TEXT("Trim '%s': correction %d of at most %d%s -- solving α = %.3f°%s..."),
				*Condition.Name, Driver->Iterations() + 1, FMath::Max(Condition.MaxIterations, 1), bClamped ? TEXT(" (step held to the trust region)") : TEXT(""),
				N.alpha_deg, bControl ? *FString::Printf(TEXT(", δ = %.3f°"), N.delta_deg) : TEXT(""))));
		}
		TSharedPtr<FCase> C;
		try
		{
			C = BuildCase(Setup, N.alpha_deg, 0.0, Ctl, N.delta_deg);
		}
		catch (const std::exception& E)
		{
			Finish(false, HistoryText() + FString(TEXT("\nThe next state failed to mesh: ")) + UTF8_TO_TCHAR(E.what()));
			return;
		}
		if (MeshText.IsEmpty()) MeshText = C->MeshReport;
		++Outcome.Solves;
		TWeakPtr<FRunner> Weak = AsShared();
		TArray<TSharedPtr<FCase>> ToSolve;
		ToSolve.Add(C);
		SolveCases(ToSolve, SolveOptions(Setup, &Condition.MassCentre), [Weak, C, bClamped](bool bOk, const FString& Why)
		{
			TSharedPtr<FRunner> Self = Weak.Pin();
			if (!Self.IsValid()) return;
			if (!bOk) { Self->Finish(false, Why); return; }
			FString Bad;
			if (!Self->Accept(*C, Bad))
			{
				Self->Note(C, false, bClamped);   // in the history, but never fed to the driver
				Self->StopReason = Bad;
				if (Self->Driver->HasBest() && Self->BestCase.IsValid()) { Self->FinishFromDriver(); return; }
				Self->Driver->Fail(TCHAR_TO_UTF8(*Bad));
				Self->Advance();
				return;
			}
			Self->LastCase = C;
			Self->Driver->FeedState({ C->AlphaDeg, C->DeltaDeg, C->CL, C->Cm });
			Self->Outcome.Iterations = Self->Driver->Iterations();
			Self->Note(C, false, bClamped);
			Self->Advance();
		});
	}

	bool FRunner::Accept(FCase& C, FString& OutWhy) const
	{
		Derive(C, Setup.ReferenceLength);
		if (!(Condition.WakeCheckFraction > 0.0) || !C.Sol.lift_from_wake.has_value()) return true;
		const double CLw = C.Sol.lift_from_wake.value();
		const double Band = FMath::Max(Condition.WakeCheckFraction * FMath::Abs(CLw), Condition.WakeCheckFloor);
		if (FMath::Abs(C.CL - CLw) <= Band) return true;
		OutWhy = FString::Printf(
			TEXT("the panel solution at α = %.3f°, δ = %.3f° is not an operating point: the lift from the PRESSURES is C_L %.4f while the WAKE CIRCULATION gives %.4f, and in a potential flow those are the same number. ")
			TEXT("Something in the flow field is wrong -- a wake sheet running into a body downstream, panels in near contact, a mesh that is not closed -- and no trim taken on it would mean anything. ")
			TEXT("Run Analysis > Panel Method at this attitude and look at the mesh and the pressures before trimming again."),
			C.AlphaDeg, C.DeltaDeg, C.CL, CLw);
		return false;
	}

	void FRunner::Note(const TSharedPtr<FCase>& C, bool bJacobianHere, bool bClamped)
	{
		FTrimIterate It;
		It.AlphaDeg = C->AlphaDeg;
		It.DeltaDeg = C->DeltaDeg;
		It.CL = C->CL;
		It.Cm = C->Cm;
		It.bWake = C->Sol.lift_from_wake.has_value();
		It.CLWake = C->Sol.lift_from_wake.value_or(0.0);
		It.ResidualNorm = aerotelic::aero::ResidualNorm({ C->AlphaDeg, C->DeltaDeg, C->CL, C->Cm },
			Outcome.CLRequired, Condition.Control.IsSet(), Driver->settings().tolerance);
		It.bJacobian = bJacobianHere;
		It.bClamped = bClamped;
		Outcome.History.Add(It);
		// The iterate to keep is the one with the smallest residual, which the
		// driver has just chosen; hold on to its case.
		const aerotelic::aero::TrimPoint& Best = Driver->Best();
		if (Driver->HasBest() && FMath::IsNearlyEqual(Best.alpha_deg, C->AlphaDeg, 1e-9) && FMath::IsNearlyEqual(Best.delta_deg, C->DeltaDeg, 1e-9))
		{
			BestCase = C;
		}
	}

	FString FRunner::HistoryText() const
	{
		if (Outcome.History.Num() == 0) return FString();
		const bool bControl = Condition.Control.IsSet();
		FString Text = FString::Printf(TEXT("\nThe iteration (C_L required %.4f; the residual is in tolerances, 1 or less is trimmed):"), Outcome.CLRequired);
		for (int32 i = 0; i < Outcome.History.Num(); ++i)
		{
			const FTrimIterate& It = Outcome.History[i];
			Text += FString::Printf(TEXT("\n  %2d  α %8.3f°%s  C_L %8.4f  C_m %9.5f  residual %8.2f%s%s%s"),
				i, It.AlphaDeg,
				bControl ? *FString::Printf(TEXT("  δ %8.3f°"), It.DeltaDeg) : TEXT(""),
				It.CL, It.Cm, It.ResidualNorm,
				It.bWake ? *FString::Printf(TEXT("  (wake C_L %.4f)"), It.CLWake) : TEXT(""),
				It.bJacobian ? TEXT("  [derivatives measured here]") : TEXT(""),
				It.bClamped ? TEXT("  [step held to the trust region]") : TEXT(""));
		}
		return Text;
	}

	void FRunner::FinishFromDriver()
	{
		if (!Driver->HasBest() || !BestCase.IsValid())
		{
			Finish(false, HistoryText() + TEXT("\nNO TRIM: no state of the iteration was solved."));
			return;
		}
		FCase& C = *BestCase;
		const FOutcome& O = Outcome;
		Outcome.Trimmed = BestCase;
		Outcome.bConverged = Driver->Trimmed();
		Outcome.LiftErrorFraction = Driver->BestLiftErrorFraction();
		Outcome.ResidualNorm = Driver->BestNorm();
		Outcome.Iterations = Driver->Iterations();
		Outcome.Jacobians = Driver->Jacobians();
		Outcome.Jacobian = Driver->Jacobian();
		const bool bControl = Condition.Control.IsSet();

		FString Text = TEXT("Aero mesh:\n") + MeshText;
		Text += FString::Printf(TEXT("\nDerivatives last measured about α = %.3f°, δ = %.3f°: dC_L/dα = %.4f, dC_m/dα = %.4f per degree%s."),
			O.Jacobian.alpha_deg, O.Jacobian.delta_deg, O.Jacobian.dCL_dalpha, O.Jacobian.dCm_dalpha,
			bControl ? *FString::Printf(TEXT("; dC_L/dδ = %.4f, dC_m/dδ = %.4f per degree (%s)"), O.Jacobian.dCL_ddelta, O.Jacobian.dCm_ddelta, *Condition.Control.Label) : TEXT(""));
		Text += HistoryText();
		const double Lift = C.CL * O.DynamicPressure * O.ReferenceAreaSI;
		Text += FString::Printf(TEXT("\n%s at α = %.3f°%s: C_L %.4f against %.4f required (lift %.4g N against n W = %.4g N, %.2f%% off), C_m about the mass centre %.5f; C_L from the wake circulation %.4f."),
			Outcome.bConverged ? TEXT("TRIMMED") : TEXT("NOT TRIMMED -- the best state reached"),
			C.AlphaDeg, bControl ? *FString::Printf(TEXT(", %s at δ = %.3f°"), *Condition.Control.Label, C.DeltaDeg) : TEXT(" (angle of attack alone; the moment is what it is)"),
			C.CL, O.CLRequired, Lift, Condition.LoadFactor * O.Weight, 100.0 * Outcome.LiftErrorFraction, C.Cm, C.Sol.lift_from_wake.value_or(0.0));
		Text += FString::Printf(TEXT("\n%d panel solves in %d correction%s and %d derivative measurement%s; the residual is %.2f tolerances (lift to %.4f, moment to %.4f)."),
			O.Solves, O.Iterations, O.Iterations == 1 ? TEXT("") : TEXT("s"), O.Jacobians, O.Jacobians == 1 ? TEXT("") : TEXT("s"),
			Outcome.ResidualNorm, aerotelic::aero::LiftTolerance(O.CLRequired, Driver->settings().tolerance), Driver->settings().tolerance.Cm);
		Text += FString::Printf(TEXT("\nSolve: %s unknowns, %s, %.2f s."), *Thousands(C.Sol.unknowns), C.Sol.direct ? TEXT("direct") : *FString::Printf(TEXT("GMRES %d iterations, residual %.1e"), C.Sol.iterations, C.Sol.residual), C.Sol.seconds_influence + C.Sol.seconds_solve);
		for (const std::string& N : C.Sol.notes) Text += TEXT("\n") + FString(UTF8_TO_TCHAR(N.c_str()));
		if (!Driver->Note().empty()) Text += TEXT("\n") + FString(UTF8_TO_TCHAR(Driver->Note().c_str())) + TEXT(".");
		if (!StopReason.IsEmpty()) Text += TEXT("\nTHE ITERATION STOPPED EARLY: ") + StopReason + TEXT(" The state above is the best one reached before that.");

		// WHAT IS STORED. A case that is not in equilibrium is not a load
		// case: the structure would be sized for a flight condition the
		// aircraft is not in. Case 1 of my_drone was stored 148% off.
		const double Err = FMath::Abs(Outcome.LiftErrorFraction);
		if (!Outcome.bConverged && Condition.StoreLimitFraction > 0.0 && Err > Condition.StoreLimitFraction)
		{
			Finish(false, Text + FString::Printf(
				TEXT("\nNOT STORED: the lift is %.1f%% off what the condition needs, beyond the %.0f%% a load case may be out by. ")
				TEXT("Trim again from this attitude (it is the best the iteration reached), loosen the case, or look at whether the configuration can hold this load factor at this speed at all."),
				100.0 * Outcome.LiftErrorFraction, 100.0 * Condition.StoreLimitFraction));
			return;
		}
		if (!Outcome.bConverged)
		{
			Outcome.Warning = FString::Printf(TEXT("stored off target: the lift is %.2f%% from what the condition needs (%.2f tolerances out)"), 100.0 * Outcome.LiftErrorFraction, Outcome.ResidualNorm);
			Text += FString::Printf(TEXT("\nWARNING: %s. The pressures stored are this state's, not an equilibrium's."), *Outcome.Warning);
		}
		if (bControl)
		{
			Text += FString::Printf(TEXT("\nThe model's own %s deflection is still %g°: set it to %.2f° in the wing's Flaps tab so the structural model's flap body sits where the air loads were computed."), *Condition.Control.Label, Condition.Delta0, C.DeltaDeg);
		}
		Outcome.Case = MakeLoadCase(C, Setup, Condition.Name, O.DynamicPressure, &Condition, &Outcome);
		const aerotelic::Vec3 Fc = Outcome.Case->ForceCoefficient();
		Text += FString::Printf(TEXT("\nLoad case '%s': %d panels, q = %.4g Pa; the panels' own resultant over S_ref is (%.4f, %.4f, %.4f)%s."),
			*Outcome.Case->Name, Outcome.Case->BodyPanels(), Outcome.Case->DynamicPressure, Fc.x, Fc.y, Fc.z,
			C.bImagedSolve ? TEXT(" with the image about y = 0") : TEXT(""));
		Outcome.Report = Text;
		Finish(true, FString());
	}

	void FRunner::Cancel()
	{
		bCancel = true;
	}

	void FRunner::SolveCases(TArray<TSharedPtr<FCase>> Cases, const tripanel::Options& Options, TFunction<void(bool, const FString&)> Then)
	{
		TWeakPtr<FRunner> Weak = AsShared();
		const int32 Gen = Generation;
		Async(EAsyncExecution::Thread, [Cases, Options, Then, Weak, Gen]()
		{
			FString Error;
			bool bOk = true;
			for (int32 i = 0; i < Cases.Num(); ++i)
			{
				{
					TSharedPtr<FRunner> Self = Weak.Pin();
					if (!Self.IsValid() || Self->bCancel.load() || Self->Generation != Gen) { bOk = false; Error = TEXT("cancelled"); break; }
				}
				TSharedPtr<FCase> C = Cases[i];
				try
				{
					SolveCaseNow(*C, Options);
				}
				catch (const std::exception& E)
				{
					bOk = false;
					Error = FString::Printf(TEXT("case %d (alpha %g, delta %g): %s"), i + 1, C->AlphaDeg, C->DeltaDeg, UTF8_TO_TCHAR(E.what()));
					break;
				}
			}
			AsyncTask(ENamedThreads::GameThread, [Then, bOk, Error, Weak, Gen]()
			{
				TSharedPtr<FRunner> Self = Weak.Pin();
				if (!Self.IsValid() || Self->Generation != Gen) return;
				Then(bOk, Error);
			});
		});
	}


	void FRunner::Finish(bool bOk, const FString& Why)
	{
		bRunning = false;
		Outcome.bOk = bOk;
		if (!bOk) Outcome.Report = Why;
		if (OnDone) OnDone(Outcome);
	}
}
