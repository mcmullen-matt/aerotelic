#include "AerotelicBooleanProbeCommandlet.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicAeroTrim.h"
#include "Misc/Paths.h"

#include <algorithm>
#include <cmath>

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicBooleanProbe, Log, All);

namespace
{
	using aerotelic::project_yaml::EntityType;

	FString OperandLabel(const FAerotelicBooleanOperand& Op)
	{
		if (!Op.Entity.IsValid()) return TEXT("(gone)");
		return Op.Entity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT(""));
	}

	FString JoinOperands(const TArray<FAerotelicBooleanOperand>& Ops)
	{
		FString S;
		for (int32 i = 0; i < Ops.Num(); ++i) S += (i ? TEXT(", ") : TEXT("")) + OperandLabel(Ops[i]);
		return S;
	}

	TArray<FString> SplitNames(const FString& List)
	{
		TArray<FString> Out;
		List.ParseIntoArray(Out, TEXT(","), true);
		for (FString& S : Out) S.TrimStartAndEndInline();
		return Out;
	}

	// One attempt: the dialog's call, timed; the result removed again when
	// asked (the probe adds nothing to the file -- it never saves anyway).
	bool TryBoolean(UAerotelicDocumentSubsystem* Doc, const FString& Name, const TArray<FAerotelicBooleanOperand>& Ops, const FString& Op, bool bKeep, FString& OutError, AAerotelicEntityActor** OutResult = nullptr)
	{
		const double T0 = FPlatformTime::Seconds();
		AAerotelicEntityActor* Result = Doc->AddBoolean(Name, Ops, Op, OutError);
		const double Seconds = FPlatformTime::Seconds() - T0;
		if (Result == nullptr)
		{
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %s of [%s]: FAILED in %.2f s: %s"), *Op, *JoinOperands(Ops), Seconds, *OutError);
			return false;
		}
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %s of [%s]: ok in %.2f s, %d body(ies)"), *Op, *JoinOperands(Ops), Seconds, Result->GetBodies().Num());
		if (OutResult != nullptr) *OutResult = Result;
		if (!bKeep) Doc->RemoveEntity(Result);
		return true;
	}

	// Distance from a point to a segment.
	double SegmentDistance(const aerotelic::Vec3& P, const aerotelic::Vec3& A, const aerotelic::Vec3& B)
	{
		const aerotelic::Vec3 D = B - A;
		const double L2 = aerotelic::Dot(D, D);
		double T = L2 > 0.0 ? aerotelic::Dot(P - A, D) / L2 : 0.0;
		T = std::clamp(T, 0.0, 1.0);
		return aerotelic::Length(P - (A + D * T));
	}

	// ---- the panel-method mode ----

	int32 PanelProbe(UAerotelicDocumentSubsystem* Doc, const FString& EntityName, double AlphaDeg, double BetaDeg, bool bNoSymmetry, const FString& ShareFilter)
	{
		namespace tripanel = aerotelic::tripanel;
		using AerotelicAeroTrim::FCase;
		using AerotelicAeroTrim::FShare;

		AAerotelicEntityActor* Entity = nullptr;
		for (const TWeakObjectPtr<AAerotelicEntityActor>& W : Doc->GetEntities())
			if (W.IsValid() && W->EntityName == EntityName) Entity = W.Get();
		if (Entity == nullptr)
		{
			UE_LOG(LogAerotelicBooleanProbe, Error, TEXT("no entity named '%s'"), *EntityName);
			return 1;
		}
		TArray<AAerotelicEntityActor*> Es;
		Es.Add(Entity);
		AerotelicAeroTrim::FSetup S;
		S.Entities.Add(Entity);
		S.bFuse = true;
		S.WakeChords = 20.0;
		S.bSymmetry = !bNoSymmetry && AerotelicAeroTrim::DefaultSymmetry(Es);
		AerotelicAeroTrim::DefaultReferences(Es, S.ReferenceArea, S.ReferenceLength);
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("panel method on '%s': alpha %g, beta %g, symmetry plane %s, S_ref %.5g, L_ref %.5g, wake %g L_ref"),
			*EntityName, AlphaDeg, BetaDeg, S.bSymmetry ? TEXT("ON") : TEXT("off"), S.ReferenceArea, S.ReferenceLength, S.WakeChords);

		TSharedPtr<FCase> C;
		try
		{
			C = AerotelicAeroTrim::BuildCase(S, AlphaDeg, BetaDeg);
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogAerotelicBooleanProbe, Error, TEXT("the aero mesh could not be built: %s"), UTF8_TO_TCHAR(E.what()));
			return 1;
		}
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("mesh report:\n%s"), *C->MeshReport);
		for (const FString& N : C->Notes) UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("note: %s"), *N);
		for (const FString& W : C->Warnings) UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("WARNING: %s"), *W);

		// The trailing edges the solver will find: per body and polyline, the
		// polyline's points and the mesh edges on it at the solver's tolerance,
		// and at a thousand times that, to show when the polyline and the mesh
		// disagree by more than round-off (a curved trailing edge sampled finely
		// against straight mesh edges between stations).
		{
			aerotelic::Box3 All;
			for (const tripanel::Body& B : C->Bodies) for (const aerotelic::Vec3& V : B.mesh.vertices) All.Add(V);
			const double Diag = All.Diagonal();
			for (const tripanel::Body& B : C->Bodies)
			{
				const double Tol = B.trailing_edge_tolerance > 0.0 ? B.trailing_edge_tolerance : 1e-6 * Diag;
				for (size_t l = 0; l < B.trailing_edges.size(); ++l)
				{
					const std::vector<std::vector<aerotelic::Vec3>> One{ B.trailing_edges[l] };
					const int32 NTight = (int32)tripanel::EdgesOnPolylines(B.mesh, One, Tol).size();
					const int32 NLoose = (int32)tripanel::EdgesOnPolylines(B.mesh, One, 1000.0 * Tol).size();
					const aerotelic::Vec3& P0 = B.trailing_edges[l].front();
					const aerotelic::Vec3& P1 = B.trailing_edges[l].back();
					UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("trailing edge %d of '%s': %d points from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f); mesh edges on it: %d at tol %.2g, %d at tol %.2g"),
						(int32)l, UTF8_TO_TCHAR(B.name.c_str()), (int32)B.trailing_edges[l].size(), P0.x, P0.y, P0.z, P1.x, P1.y, P1.z, NTight, Tol, NLoose, 1000.0 * Tol);
				}
			}
		}

		const tripanel::Options Opt = AerotelicAeroTrim::SolveOptions(S);
		try
		{
			AerotelicAeroTrim::SolveCaseNow(*C, Opt);
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogAerotelicBooleanProbe, Error, TEXT("the solve failed: %s"), UTF8_TO_TCHAR(E.what()));
			return 1;
		}
		AerotelicAeroTrim::Derive(*C, S.ReferenceLength);
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("solved: %d unknowns, %s, %.2f s; C_L %.4f (from the wake %.4f), C_D %.4f, C_m %.4f (about the origin); Cp 2nd..98th pct %.3f..%.3f; %d wake strips"),
			C->Sol.unknowns, C->Sol.direct ? TEXT("LU") : *FString::Printf(TEXT("GMRES %d it, residual %.1e"), C->Sol.iterations, C->Sol.residual),
			C->Sol.seconds_influence + C->Sol.seconds_solve, C->CL, C->Sol.lift_from_wake.value_or(0.0), C->CD, C->Cm, C->CpMin, C->CpMax, (int32)C->Sol.wakes.size());
		for (const std::string& N : C->Sol.notes) UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("solver note: %s"), UTF8_TO_TCHAR(N.c_str()));
		for (int32 s = 0; s < C->Shares.Num(); ++s)
		{
			const FShare& Sh = C->Shares[s];
			int32 Panels = 0;
			for (char M : Sh.Mask) Panels += M ? 1 : 0;
			const AerotelicAeroTrim::FCoefs& K = C->ByBody[s];
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("share %d '%s': body %d, %d panels%s%s; C_L %.4f, C_D %.4f, C_Y %.4f, C_m %.4f"),
				s, *Sh.Name, Sh.Body, Panels, Sh.bWing ? TEXT(", wing") : TEXT(""), Sh.bTipToTip ? TEXT(" (tip to tip)") : TEXT(""), K.CL, K.CD, K.CY, K.Cm);
		}

		// The share that sheds each wake strip: the one owning its upper panel.
		auto ShareOfPanel = [&C](int32 Body, int32 Panel) -> int32
		{
			for (int32 s = 0; s < C->Shares.Num(); ++s)
			{
				const FShare& Sh = C->Shares[s];
				if (Sh.Body == Body && Panel >= 0 && Panel < (int32)Sh.Mask.size() && Sh.Mask[Panel]) return s;
			}
			return -1;
		};
		TArray<int32> WakeShare;
		TMap<int32, int32> WakeCount, RoutedCount, StoppedCount;
		for (const tripanel::WakeStrip& W : C->Sol.wakes)
		{
			const int32 s = ShareOfPanel(W.body, W.upper >= 0 ? W.upper : W.lower);
			WakeShare.Add(s);
			++WakeCount.FindOrAdd(s);
			if (W.routed_around >= 0 && !W.stopped) ++RoutedCount.FindOrAdd(s);
			if (W.stopped) ++StoppedCount.FindOrAdd(s);
		}
		for (const auto& KV : WakeCount)
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("wake: %d strips shed by %s (%d routed around a body, %d stopped short of one)"), KV.Value, KV.Key >= 0 ? *C->Shares[KV.Key].Name : TEXT("(unowned)"), RoutedCount.FindRef(KV.Key), StoppedCount.FindRef(KV.Key));
		// A strip's centreline z where its path crosses x (the path runs
		// downstream, x increasing); false when it never reaches x.
		auto SheetZAt = [](const tripanel::WakeStrip& W, double X, double& OutZ) -> bool
		{
			for (size_t k = 0; k + 1 < W.a_path.size(); ++k)
			{
				const aerotelic::Vec3 P0 = (W.a_path[k] + W.b_path[k]) * 0.5, P1 = (W.a_path[k + 1] + W.b_path[k + 1]) * 0.5;
				if ((P0.x <= X && X <= P1.x) || (P1.x <= X && X <= P0.x))
				{
					const double T = std::fabs(P1.x - P0.x) > 1e-15 ? (X - P0.x) / (P1.x - P0.x) : 0.0;
					OutZ = P0.z + (P1.z - P0.z) * T;
					return true;
				}
			}
			return false;
		};

		// Every wing share: the dialog's cuts, and the wakes' proximity.
		for (int32 s = 0; s < C->Shares.Num(); ++s)
		{
			const FShare& Sh = C->Shares[s];
			if (!Sh.bWing || Sh.bMirrorCopy) continue;
			if (!ShareFilter.IsEmpty() && !Sh.Name.Contains(ShareFilter)) continue;
			if (Sh.Body < 0 || Sh.Body >= (int32)C->Sol.bodies.size()) continue;
			const tripanel::BodyResult& BR = C->Sol.bodies[Sh.Body];
			const aerotelic::Vec3 Axis{ Sh.SpanAxis.X, Sh.SpanAxis.Y, Sh.SpanAxis.Z };
			const aerotelic::Vec3 LoadDir = aerotelic::Normalized(aerotelic::Cross(C->Stream, Axis));
			const double SRoot = FVector::DotProduct(Sh.RootPoint, Sh.SpanAxis), STip = FVector::DotProduct(Sh.TipPoint, Sh.SpanAxis);
			const int32 NCuts = Sh.CutChords.Num();
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("---- span loading of '%s': axis (%.3f, %.3f, %.3f), root s = %.4f, tip s = %.4f, %d cuts%s%s ----"),
				*Sh.Name, Axis.x, Axis.y, Axis.z, SRoot, STip, NCuts, Sh.bImaged ? TEXT(", imaged") : TEXT(""), Sh.bTipToTip ? TEXT(", tip to tip") : TEXT(""));
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %9s %8s %10s %9s %9s %6s %8s %8s"), TEXT("s"), TEXT("chord"), TEXT("l/q"), TEXT("cl"), TEXT("outline"), TEXT("pts"), TEXT("Cp min"), TEXT("Cp max"));
			auto Cut = [&](bool bOtherHalf, bool bMirror)
			{
				for (int32 k = 0; k < NCuts; ++k)
				{
					const double St = SRoot + (STip - SRoot) * (k + 0.5) / NCuts;
					const double SAt = bOtherHalf ? -St : St;
					const tripanel::SectionCut Sc = tripanel::CutSection(BR, Axis, SAt, bMirror, &Sh.Mask);
					if (!Sc.found)
					{
						UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %9.4f %8.4f %10s"), SAt, Sh.CutChords[k], TEXT("(no section)"));
						continue;
					}
					double CpLo = 1e300, CpHi = -1e300;
					for (double Cp : Sc.cp) { CpLo = std::min(CpLo, Cp); CpHi = std::max(CpHi, Cp); }
					const double L = aerotelic::Dot(Sc.force_per_span, LoadDir);
					UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %9.4f %8.4f %10.5f %9.4f %9.4f %6d %8.3f %8.3f"),
						SAt, Sh.CutChords[k], L, Sh.CutChords[k] > 1e-12 ? L / Sh.CutChords[k] : 0.0, Sc.length, (int32)Sc.points.size(), CpLo, CpHi);
				}
			};
			Cut(false, false);
			if (FMath::Abs(Sh.SpanAxis.Y) > 0.9)
			{
				if (Sh.bImaged) Cut(true, true);
				else if (Sh.bTipToTip) Cut(true, false);
			}

			// The wakes' proximity to this share's panels: for every shedding
			// share, the closest its sheet comes to one of these panels (the
			// sheet sampled along its first few reference lengths), and how
			// many of these panels lie within one local panel size of it.
			double MedianEdge = 0.0;
			{
				std::vector<double> Sizes;
				for (size_t t = 0; t < BR.panels.size(); ++t) if (Sh.Mask[t]) Sizes.push_back(std::sqrt(BR.panels[t].area));
				if (!Sizes.empty()) { std::sort(Sizes.begin(), Sizes.end()); MedianEdge = Sizes[Sizes.size() / 2]; }
			}
			TMap<int32, double> Closest;
			TMap<int32, int32> Within;
			TMap<int32, aerotelic::Vec3> ClosestAt;
			TMap<int32, int32> ClosestStrip;
			for (size_t t = 0; t < BR.panels.size(); ++t)
			{
				if (!Sh.Mask[t]) continue;
				const aerotelic::Vec3 P = BR.panels[t].centre;
				TMap<int32, double> Best;
				TMap<int32, int32> BestStrip;
				for (int32 w = 0; w < (int32)C->Sol.wakes.size(); ++w)
				{
					const tripanel::WakeStrip& W = C->Sol.wakes[w];
					const int32 From = WakeShare[w];
					if (From == s) continue;   // its own wake
					// The strip's two long edges and centreline over their first 3
					// reference lengths, segment by segment (a routed strip kinks).
					double D = 1e300, Cum = 0.0;
					for (size_t k = 0; k + 1 < W.a_path.size() && Cum < 3.0 * S.ReferenceLength; ++k)
					{
						const aerotelic::Vec3 A0 = W.a_path[k], A1 = W.a_path[k + 1], B0 = W.b_path[k], B1 = W.b_path[k + 1];
						D = std::min({ D, SegmentDistance(P, A0, A1), SegmentDistance(P, B0, B1), SegmentDistance(P, (A0 + B0) * 0.5, (A1 + B1) * 0.5) });
						Cum += aerotelic::Length(A1 - A0);
					}
					double* Cur = Best.Find(From);
					if (Cur == nullptr || D < *Cur) { Best.Add(From, D); BestStrip.Add(From, w); }
				}
				for (const auto& KV : Best)
				{
					double* Cur = Closest.Find(KV.Key);
					if (Cur == nullptr || KV.Value < *Cur) { Closest.Add(KV.Key, KV.Value); ClosestAt.Add(KV.Key, P); ClosestStrip.Add(KV.Key, BestStrip[KV.Key]); }
					if (KV.Value < MedianEdge) ++Within.FindOrAdd(KV.Key);
				}
			}
			for (const auto& KV : Closest)
			{
				const aerotelic::Vec3 At = ClosestAt[KV.Key];
				const tripanel::WakeStrip& W = C->Sol.wakes[ClosestStrip[KV.Key]];
				UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  wake of %s comes within %.4f of this share's panels (median panel size %.4f; %d panels within one panel size), closest at (%.3f, %.3f, %.3f); that strip spans y %.3f..%.3f, %d segments%s"),
					KV.Key >= 0 ? *C->Shares[KV.Key].Name : TEXT("(unowned)"), KV.Value, MedianEdge, Within.FindRef(KV.Key), At.x, At.y, At.z,
					std::min(W.a_path[0].y, W.b_path[0].y), std::max(W.a_path[0].y, W.b_path[0].y), (int32)W.Segments(), W.routed_around >= 0 ? TEXT(", routed") : TEXT(", straight"));
			}

			// Where the other shares' sheets stand in z as they pass this share:
			// for each strip, the sheet's height at this share's leading and
			// trailing x, against the share's own z range in that strip's y band.
			aerotelic::Box3 Bounds;
			for (size_t t = 0; t < BR.panels.size(); ++t) if (Sh.Mask[t]) Bounds.Add(BR.panels[t].centre);
			for (int32 w = 0; w < (int32)C->Sol.wakes.size(); ++w)
			{
				const tripanel::WakeStrip& W = C->Sol.wakes[w];
				const int32 From = WakeShare[w];
				if (From == s || From < 0) continue;
				const aerotelic::Vec3 Mid = (W.a_path[0] + W.b_path[0]) * 0.5;
				if (Mid.x > Bounds.min.x) continue;   // only sheets that reach this share from upstream
				const double YLo = std::min(W.a_path[0].y, W.b_path[0].y), YHi = std::max(W.a_path[0].y, W.b_path[0].y);
				double ZLo = 1e300, ZHi = -1e300;
				int32 N = 0;
				for (size_t t = 0; t < BR.panels.size(); ++t)
				{
					if (!Sh.Mask[t]) continue;
					const aerotelic::Vec3& P = BR.panels[t].centre;
					if (P.y < YLo || P.y > YHi) continue;
					ZLo = std::min(ZLo, P.z); ZHi = std::max(ZHi, P.z); ++N;
				}
				if (N == 0) continue;
				double ZAtLead = 0.0, ZAtTrail = 0.0;
				if (!SheetZAt(W, Bounds.min.x, ZAtLead) || !SheetZAt(W, Bounds.max.x, ZAtTrail)) continue;
				const bool bThrough = (ZAtLead >= ZLo && ZAtLead <= ZHi) || (ZAtTrail >= ZLo && ZAtTrail <= ZHi) || (ZAtLead < ZLo && ZAtTrail > ZHi) || (ZAtLead > ZHi && ZAtTrail < ZLo);
				const FString Routed = W.stopped ? FString::Printf(TEXT(" [STOPPED short of body %d]"), W.routed_around) : W.routed_around >= 0 ? FString::Printf(TEXT(" [routed around body %d, %d segments]"), W.routed_around, (int32)W.Segments()) : FString();
				UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  sheet of %s, y %.3f..%.3f, mu %.4f: z %.4f at x = %.3f, z %.4f at x = %.3f; this share there spans z %.4f..%.4f (%d panels)%s%s"),
					*C->Shares[From].Name, YLo, YHi, W.mu, ZAtLead, Bounds.min.x, ZAtTrail, Bounds.max.x, ZLo, ZHi, N, bThrough ? TEXT("  <-- THROUGH THE BODY") : TEXT(""), *Routed);
			}
		}

		// Panels whose Cp is beyond anything physical: their geometry and
		// the surface gradient that produced them.
		for (const tripanel::BodyResult& BR : C->Sol.bodies)
		{
			for (size_t t = 0; t < BR.panels.size(); ++t)
			{
				const tripanel::PanelResult& P = BR.panels[t];
				if (!(std::fabs(P.cp) > 100.0)) continue;
				const auto& Tr = BR.mesh.triangles[t];
				const aerotelic::Vec3& A = BR.mesh.vertices[Tr[0]];
				const aerotelic::Vec3& Bv = BR.mesh.vertices[Tr[1]];
				const aerotelic::Vec3& Cv = BR.mesh.vertices[Tr[2]];
				const double E0 = aerotelic::Length(Bv - A), E1 = aerotelic::Length(Cv - Bv), E2 = aerotelic::Length(A - Cv);
				const double Longest = std::max({ E0, E1, E2 });
				const double Aspect = P.area > 0.0 ? Longest * Longest / (2.0 * P.area) : 0.0;
				UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("ROGUE panel %d: Cp %.3g at (%.4f, %.4f, %.4f), area %.3g, edges %.4g/%.4g/%.4g, aspect ~%.0f, |grad mu| %.3g, mu %.4f"),
					(int32)t, P.cp, P.centre.x, P.centre.y, P.centre.z, P.area, E0, E1, E2, Aspect, aerotelic::Length(P.grad_mu), P.mu);
			}
		}

		// The Cp extremes of every share and where they sit.
		for (int32 s = 0; s < C->Shares.Num(); ++s)
		{
			const FShare& Sh = C->Shares[s];
			if (Sh.Body < 0 || Sh.Body >= (int32)C->Sol.bodies.size()) continue;
			const tripanel::BodyResult& BR = C->Sol.bodies[Sh.Body];
			double Lo = 1e300, Hi = -1e300;
			aerotelic::Vec3 LoAt, HiAt;
			int32 Below2 = 0;
			for (size_t t = 0; t < BR.panels.size(); ++t)
			{
				if (!Sh.Mask[t]) continue;
				const double Cp = BR.panels[t].cp;
				if (Cp < Lo) { Lo = Cp; LoAt = BR.panels[t].centre; }
				if (Cp > Hi) { Hi = Cp; HiAt = BR.panels[t].centre; }
				if (Cp < -2.0) ++Below2;
			}
			if (Lo > Hi) continue;
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("Cp of '%s': min %.3f at (%.3f, %.3f, %.3f), max %.3f at (%.3f, %.3f, %.3f), %d panels below -2"),
				*Sh.Name, Lo, LoAt.x, LoAt.y, LoAt.z, Hi, HiAt.x, HiAt.y, HiAt.z, Below2);
		}
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("RESULT: panel probe done"));
		return 0;
	}
}

int32 UAerotelicBooleanProbeCommandlet::Main(const FString& Params)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr)
	{
		UE_LOG(LogAerotelicBooleanProbe, Error, TEXT("no document subsystem"));
		return 1;
	}

	FString Project = FPaths::Combine(FPaths::ProjectDir(), TEXT("Projects"), TEXT("my_drone.yaml"));
	FParse::Value(*Params, TEXT("Project="), Project);
	FString Op = TEXT("fuse");
	FParse::Value(*Params, TEXT("Op="), Op);
	FString IncludeList, ExcludeList;
	FParse::Value(*Params, TEXT("Include="), IncludeList);
	FParse::Value(*Params, TEXT("Exclude="), ExcludeList);
	const TArray<FString> Include = SplitNames(IncludeList), Exclude = SplitNames(ExcludeList);

	UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("opening %s"), *Project);
	if (!Doc->OpenProjectPath(Project, /*bAdopt*/ false))
	{
		UE_LOG(LogAerotelicBooleanProbe, Error, TEXT("could not open %s"), *Project);
		return 1;
	}

	FString PanelEntity;
	if (FParse::Value(*Params, TEXT("Panel="), PanelEntity) && !PanelEntity.IsEmpty())
	{
		double Alpha = 4.0, Beta = 0.0;
		FParse::Value(*Params, TEXT("Alpha="), Alpha);
		FParse::Value(*Params, TEXT("Beta="), Beta);
		FString Share;
		FParse::Value(*Params, TEXT("Share="), Share);
		return PanelProbe(Doc, PanelEntity, Alpha, Beta, FParse::Param(*Params, TEXT("NoSymmetry")), Share);
	}

	// The operands: every non-derived solid but the propellers, in the
	// file's order; a mirrored entity contributes its mirror copy too.
	TArray<FAerotelicBooleanOperand> Ops;
	TArray<FString> LeftOut;
	for (const TWeakObjectPtr<AAerotelicEntityActor>& W : Doc->GetEntities())
	{
		if (!W.IsValid() || W->GetModel() == nullptr) continue;
		AAerotelicEntityActor* E = W.Get();
		const bool bNamed = Include.Contains(E->EntityName);
		if (!bNamed && (Include.Num() > 0 || E->GetEntityType() == EntityType::Propeller || E->IsDerived() || Exclude.Contains(E->EntityName)))
		{
			LeftOut.Add(E->EntityName);
			continue;
		}
		FAerotelicBooleanOperand A;
		A.Entity = E;
		Ops.Add(A);
		if (E->IsMirrored())
		{
			A.bMirror = true;
			Ops.Add(A);
		}
	}
	UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("%d operand(s): %s"), Ops.Num(), *JoinOperands(Ops));
	if (LeftOut.Num() > 0) UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("left out: %s"), *FString::Join(LeftOut, TEXT(", ")));

	// Each operand on its own: closed and outward as placed?
	for (const FAerotelicBooleanOperand& O : Ops)
	{
		try
		{
			const std::vector<aerotelic::NamedMesh> Meshes = O.Entity->GetPlacedMeshes(O.bMirror, FAerotelicBooleanModel::kValidationTolerance);
			FString Line;
			for (const aerotelic::NamedMesh& NM : Meshes)
			{
				Line += FString::Printf(TEXT(" [%s: %d tris, %s, volume %.4g]"), UTF8_TO_TCHAR(NM.name.c_str()), (int32)NM.mesh.triangles.size(),
					NM.mesh.IsClosedManifold() ? (NM.mesh.SignedVolume() > 0.0 ? TEXT("closed, outward") : TEXT("closed, INWARD")) : TEXT("NOT CLOSED"), NM.mesh.SignedVolume());
			}
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %s:%s"), *OperandLabel(O), *Line);
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  %s: no placed mesh (%s)"), *OperandLabel(O), UTF8_TO_TCHAR(E.what()));
		}
	}

	UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("step: %s of everything"), *Op);
	FString Error;
	AAerotelicEntityActor* Result = nullptr;
	const bool bOk = TryBoolean(Doc, TEXT("Probe"), Ops, Op, /*bKeep*/ true, Error, &Result);
	if (bOk && Result != nullptr)
	{
		try
		{
			const std::vector<aerotelic::NamedMesh> Bodies = Result->GetPlacedMeshes(false, FAerotelicBooleanModel::kValidationTolerance);
			for (const aerotelic::NamedMesh& NM : Bodies)
			{
				aerotelic::Box3 B;
				for (const aerotelic::Vec3& V : NM.mesh.vertices) B.Add(V);
				UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  body %s: %d triangles, %s, volume %.4g, bounds (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)"),
					UTF8_TO_TCHAR(NM.name.c_str()), (int32)NM.mesh.triangles.size(),
					NM.mesh.IsClosedManifold() ? (NM.mesh.SignedVolume() > 0.0 ? TEXT("closed, outward") : TEXT("closed, INWARD")) : TEXT("NOT CLOSED"),
					NM.mesh.SignedVolume(), B.min.x, B.min.y, B.min.z, B.max.x, B.max.y, B.max.z);
			}
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("  the result's meshes could not be rebuilt: %s"), UTF8_TO_TCHAR(E.what()));
		}
		Doc->RemoveEntity(Result);
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("RESULT: %s succeeded"), *Op);
		return 0;
	}

	// Which operand breaks it: the chain folds left to right, so the
	// shortest failing prefix names the culprit (with what it met).
	UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("step: growing prefixes"));
	int32 FirstBad = -1;
	for (int32 k = 2; k <= Ops.Num(); ++k)
	{
		TArray<FAerotelicBooleanOperand> Prefix;
		for (int32 i = 0; i < k; ++i) Prefix.Add(Ops[i]);
		FString E2;
		if (!TryBoolean(Doc, FString::Printf(TEXT("Probe %d"), k), Prefix, Op, false, E2)) { FirstBad = k - 1; break; }
	}
	if (FirstBad >= 0)
	{
		// The culprit against each earlier operand alone.
		UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("step: %s against each earlier operand alone"), *OperandLabel(Ops[FirstBad]));
		for (int32 i = 0; i < FirstBad; ++i)
		{
			FString E3;
			TryBoolean(Doc, FString::Printf(TEXT("Probe pair %d"), i), { Ops[i], Ops[FirstBad] }, Op, false, E3);
		}
	}
	UE_LOG(LogAerotelicBooleanProbe, Display, TEXT("RESULT: %s FAILED: %s"), *Op, *Error);
	return 1;
}
