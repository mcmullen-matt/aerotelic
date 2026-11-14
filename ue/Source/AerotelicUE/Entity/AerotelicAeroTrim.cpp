#include "AerotelicAeroTrim.h"

#include "Async/Async.h"

#include "AerotelicMeshing.h"

#include <cmath>
#include <functional>
#include <string>

// Last: OCCT's Handle() macro vs engine headers.
#include "aerotelic/wing_panels.hpp"
#include "AerotelicDocumentSubsystem.h"
#include "AerotelicEntityActor.h"
#include "AerotelicAeroLoads.h"
#include "AerotelicMassProperties.h"

namespace AerotelicAeroTrim
{
	namespace
	{
		constexpr double kPi = 3.14159265358979323846;

		// The corners of a network placed as the entity sits (project units).
		void PlaceNetwork(Network& N, const FTransform& Transform)
		{
			aerotelic::TriMesh M;
			M.vertices = N.corners;
			AerotelicMeshing::PlaceMesh(M, Transform);
			N.corners = M.vertices;
		}

		// The reflection of a network in y = 0, facing the same way: a
		// reflection reverses the panel normals, so they are flipped back.
		Network MirroredNetwork(const Network& N)
		{
			Network M = N;
			M.name = N.name + " (mirror)";
			for (aerotelic::Vec3& P : M.corners) P.y = -P.y;
			aerotelic::panel::FlipNormals(M);
			return M;
		}

		// A model's loft with its section function placed into the world.
		aerotelic::NamedLoft PlacedLoft(const aerotelic::NamedLoft& Loft, const FTransform& Transform)
		{
			aerotelic::NamedLoft Out = Loft;
			std::function<aerotelic::Mat(double, const std::vector<double>&)> Section = Loft.source.section;
			Out.source.section = [Section, Transform](double u, const std::vector<double>& v)
			{
				aerotelic::Mat Pts = Section(u, v);
				aerotelic::TriMesh M;
				for (int i = 0; i < Pts.rows(); ++i) M.vertices.push_back({ Pts(i, 0), Pts(i, 1), Pts(i, 2) });
				AerotelicMeshing::PlaceMesh(M, Transform);
				for (int i = 0; i < Pts.rows(); ++i) { Pts(i, 0) = M.vertices[i].x; Pts(i, 1) = M.vertices[i].y; Pts(i, 2) = M.vertices[i].z; }
				return Pts;
			};
			return Out;
		}
	}

	bool IsPanelable(EntityType T)
	{
		return T == EntityType::Wing || T == EntityType::Fuselage || T == EntityType::Nacelle || T == EntityType::LoftedNacelle || T == EntityType::Primitive;
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
		for (AAerotelicEntityActor* E : Entities)
		{
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
		for (AAerotelicEntityActor* E : Entities)
		{
			if (E->IsMirrored()) return true;
			if (E->GetEntityType() == EntityType::Wing)
			{
				const FBox B = E->GetPlacedBounds(false);
				if (B.IsValid && FMath::Abs(B.Min.Y) < 1e-6 * FMath::Max(B.GetSize().GetMax(), 1.0)) return true;
			}
		}
		return false;
	}

	TArray<FControl> Controls(const TArray<AAerotelicEntityActor*>& Entities, int32& OutDefault)
	{
		TArray<FControl> Out;
		OutDefault = -1;
		double AftX = -1e300;
		for (AAerotelicEntityActor* E : Entities)
		{
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
		const aerotelic::Vec3 Stream = C->Stream;
		const bool bSymmetry = Setup.bSymmetry;

		const TArray<AAerotelicEntityActor*> ChosenEntities = Setup.LiveEntities();
		std::vector<std::vector<aerotelic::NamedMesh>> PlacedMeshes;
		for (AAerotelicEntityActor* E : ChosenEntities)
		{
			std::vector<aerotelic::NamedMesh> Mine = E->GetPlacedMeshes(false);
			if (E->IsMirrored()) for (aerotelic::NamedMesh& M : E->GetPlacedMeshes(true)) Mine.push_back(std::move(M));
			PlacedMeshes.push_back(std::move(Mine));
		}
		std::vector<std::string> Notes;

		for (int32 Index = 0; Index < ChosenEntities.Num(); ++Index)
		{
			AAerotelicEntityActor* E = ChosenEntities[Index];
			const FTransform Transform = E->GetActorTransform();
			const FVector DirModel = Transform.InverseTransformVectorNoScale(FVector(Stream.x, Stream.y, Stream.z));
			const aerotelic::Vec3 StreamModel{ DirModel.X, DirModel.Y, DirModel.Z };
			FShare Share;
			Share.Name = E->EntityName;
			Share.First = static_cast<int32>(C->Nets.size());
			Share.bImaged = bSymmetry;
			std::vector<Network> Own;
			if (E->GetEntityType() == EntityType::Wing)
			{
				Share.bWing = true;
				Share.SpanAxis = Transform.TransformVectorNoScale(FVector(0, 1, 0)).GetSafeNormal();
				aerotelic::WingModel* W = E->GetModelAs<aerotelic::WingModel>();
				for (int32 k = 0; k <= kLoadCuts; ++k) Share.CutChords.Add(W->planform().ChordAt(static_cast<double>(k) / kLoadCuts) * Transform.GetScale3D().X);
				aerotelic::panel::WingPaneling Pan;
				Pan.chordwise = FMath::Clamp(W->mesh.chordwise > 0 ? W->mesh.chordwise : FMath::RoundToInt(Setup.Chordwise), 2, 200);
				Pan.spanwise = FMath::Clamp(W->mesh.spanwise > 0 ? W->mesh.spanwise : FMath::RoundToInt(Setup.Spanwise), 1, 200);
				Pan.wake_length = Setup.WakeChords * W->planform().RootChord();
				Pan.wake_direction = StreamModel;
				// The trim's control: this wing's flap at the asked deflection
				// for the build, the model's own value put back after.
				const bool bOverride = Control != nullptr && Control->Wing.Get() == E && Control->Flap >= 0 && Control->Flap < static_cast<int32>(W->flaps.size());
				const double Saved = bOverride ? W->flaps[Control->Flap].deflection_deg : 0.0;
				if (bOverride) W->flaps[Control->Flap].deflection_deg = DeltaDeg;
				try
				{
					Own = aerotelic::panel::WingNetworks(*W, Pan, StreamModel);
				}
				catch (...)
				{
					if (bOverride) W->flaps[Control->Flap].deflection_deg = Saved;
					throw;
				}
				if (bOverride) W->flaps[Control->Flap].deflection_deg = Saved;
				for (Network& N : Own)
				{
					N.name = TCHAR_TO_UTF8(*E->EntityName) + std::string(N.name == W->name ? "" : " wake");
					PlaceNetwork(N, Transform);
					if (N.prescribed_source) N.prescribed_source = [Stream](const aerotelic::Vec3&, const aerotelic::Vec3& Nrm) { return -aerotelic::Dot(Stream, Nrm); };
				}
				{
					const Network& Body = Own[0];
					FVector Root = FVector::ZeroVector, Tip = FVector::ZeroVector;
					for (int32 I = 1; I <= Body.rows; ++I)
					{
						const aerotelic::Vec3 R = Body.Corner(I, 1), T = Body.Corner(I, Body.cols);
						Root += FVector(R.x, R.y, R.z);
						Tip += FVector(T.x, T.y, T.z);
					}
					Share.RootPoint = Root / Body.rows;
					Share.TipPoint = Tip / Body.rows;
				}
			}
			else
			{
				std::vector<aerotelic::NamedLoft> Lofts = E->GetModel()->BuildLofts();
				if (Lofts.empty())
				{
					C->Skipped.Add(E->EntityName);
					continue;
				}
				const aerotelic::MeshSpec& Grid = E->GetModel()->mesh;
				aerotelic::panel::BodyPaneling BP;
				BP.along = FMath::Clamp(Grid.stations > 0 ? Grid.stations : FMath::RoundToInt(Setup.Along), 1, 400);
				BP.around = FMath::Clamp(Grid.around > 0 ? Grid.around : FMath::RoundToInt(Setup.Around), 3, 400);
				if (bSymmetry && !E->IsMirrored()) BP.half_plane_y = 0.0;
				BP.wake_direction = Stream;
				BP.notes = &Notes;
				for (int32 k = 0; k < static_cast<int32>(PlacedMeshes.size()); ++k)
				{
					if (k == Index) continue;
					for (const aerotelic::NamedMesh& M : PlacedMeshes[k]) BP.other_bodies.push_back(&M.mesh);
				}
				for (const aerotelic::NamedLoft& L : Lofts)
				{
					const FString Name = Lofts.size() == 1 ? E->EntityName : E->EntityName + TEXT("/") + UTF8_TO_TCHAR(L.name.c_str());
					for (Network& N : aerotelic::panel::LoftNetworks(PlacedLoft(L, Transform), TCHAR_TO_UTF8(*Name), BP, Stream)) Own.push_back(std::move(N));
				}
			}
			Share.OwnCount = static_cast<int32>(Own.size());
			if (E->IsMirrored() && !bSymmetry)
			{
				const size_t N0 = Own.size();
				for (size_t k = 0; k < N0; ++k) Own.push_back(MirroredNetwork(Own[k]));
			}
			for (Network& N : Own) C->Nets.push_back(std::move(N));
			Share.Count = static_cast<int32>(C->Nets.size()) - Share.First;
			C->Shares.Add(Share);
		}
		for (const std::string& N : Notes) C->Notes.Add(UTF8_TO_TCHAR(N.c_str()));
		if (C->Nets.empty()) throw std::runtime_error("none of the chosen bodies has a loft to panel");
		return C;
	}

	aerotelic::panel::Options SolveOptions(const FSetup& Setup, const aerotelic::Vec3* MomentReference)
	{
		aerotelic::panel::Options Opt;
		if (Setup.bSymmetry) Opt.symmetry_y = 0.0;
		Opt.reference_area = Setup.ReferenceArea > 0.0 ? Setup.ReferenceArea : 1.0;
		if (MomentReference != nullptr) Opt.moment_reference = *MomentReference;
		return Opt;
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
		C.ByBody.Reset();
		for (const FShare& Sh : C.Shares)
		{
			aerotelic::Vec3 F{ 0, 0, 0 }, M{ 0, 0, 0 };
			for (int32 k = Sh.First; k < Sh.First + Sh.Count; ++k)
			{
				F += C.Sol.networks[k].force_coefficient;
				M += C.Sol.networks[k].moment_coefficient;
			}
			C.ByBody.Add(WindAxes(F, M));
		}
		C.CpMin = 1.0;
		C.CpMax = -1e9;
		C.NBody = 0;
		C.NWake = 0;
		TArray<double> Cps;
		for (size_t k = 0; k < C.Sol.networks.size(); ++k)
		{
			const NetworkResult& NR = C.Sol.networks[k];
			const bool bWake = NR.type == NetworkType::DoubletWake || NR.type == NetworkType::DoubletWakeConstant;
			if (bWake) { C.NWake += static_cast<int32>(NR.panels.size()); continue; }
			for (const PanelResult& P : NR.panels)
			{
				if (FMath::IsFinite(P.cp_upper)) Cps.Add(P.cp_upper);
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
		for (const FShare& Sh : C.Shares) Case->Entities.Add(Sh.Name);
		const bool bImaged = C.Shares.Num() > 0 && C.Shares[0].bImaged;
		const int32 N = static_cast<int32>(C.Sol.networks.size());
		for (int32 k = 0; k < N; ++k)
		{
			const NetworkResult& NR = C.Sol.networks[k];
			if (NR.type == NetworkType::DoubletWake || NR.type == NetworkType::DoubletWakeConstant) continue;
			const Network& Net = C.Nets[k];
			const bool bOneSided = Net.type == NetworkType::SourceAnalysis || Net.bc.kind == aerotelic::panel::BoundaryCondition::Kind::Potential;
			for (int32 i = 0; i < NR.panel_rows; ++i)
			{
				for (int32 j = 0; j < NR.panel_cols; ++j)
				{
					const PanelResult& P = NR.panels[static_cast<size_t>(i) * NR.panel_cols + j];
					if (!(P.area > 0.0)) continue;
					aerotelic::aero::PressureSample S;
					S.centre = P.centre;
					S.normal = P.normal;
					S.area = P.area;
					S.cp_upper = P.cp_upper;
					S.cp_lower = P.cp_lower;
					S.two_sided = !bOneSided;
					S.network = k;
					S.row = i;
					S.col = j;
					Case->Samples.push_back(S);
					if (bImaged)
					{
						aerotelic::aero::PressureSample M = S;
						M.centre.y = -M.centre.y;
						M.normal.y = -M.normal.y;
						M.network = k + N;
						Case->Samples.push_back(M);
					}
				}
			}
		}
		return Case;
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

		if (OnStatus) OnStatus(FText::FromString(FString::Printf(TEXT("Trim '%s': paneling the base and stepped states (α = %g°, δ = %g°; C_L required %.4f)..."), *Condition.Name, Condition.Alpha0, Condition.Delta0, Outcome.CLRequired)));
		TArray<TSharedPtr<FCase>> Cases;
		try
		{
			const FControl* Ctl = Condition.Control.IsSet() ? &Condition.Control : nullptr;
			Cases.Add(BuildCase(Setup, Condition.Alpha0, 0.0, Ctl, Condition.Delta0));
			Cases.Add(BuildCase(Setup, Condition.Alpha0 + Condition.Step, 0.0, Ctl, Condition.Delta0));
			if (Ctl != nullptr) Cases.Add(BuildCase(Setup, Condition.Alpha0, 0.0, Ctl, Condition.Delta0 + Condition.Step));
		}
		catch (const std::exception& E)
		{
			Finish(false, FString(TEXT("the bodies could not be paneled: ")) + UTF8_TO_TCHAR(E.what()));
			return;
		}
		Outcome.Derivatives = Cases;
		TWeakPtr<FRunner> Weak = AsShared();
		SolveCases(Cases, SolveOptions(Setup, &Condition.MassCentre), [Weak](bool bOk, const FString& Why)
		{
			TSharedPtr<FRunner> Self = Weak.Pin();
			if (!Self.IsValid()) return;
			if (!bOk) { Self->Finish(false, Why); return; }
			Self->OnDerivativesSolved();
		});
	}

	void FRunner::Cancel()
	{
		bCancel = true;
	}

	void FRunner::SolveCases(TArray<TSharedPtr<FCase>> Cases, const aerotelic::panel::Options& Options, TFunction<void(bool, const FString&)> Then)
	{
		TWeakPtr<FRunner> Weak = AsShared();
		const int32 Gen = Generation;
		TSharedRef<std::atomic<bool>> CancelFlag = MakeShared<std::atomic<bool>>(false);
		// The runner's flag is read on the worker through this copy, so a
		// runner released mid-solve is not touched.
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
					C->Sol = aerotelic::panel::Solve(C->Nets, C->Stream, Options);
					C->bSolved = true;
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

	void FRunner::OnDerivativesSolved()
	{
		TArray<aerotelic::aero::TrimPoint> Points;
		for (TSharedPtr<FCase>& C : Outcome.Derivatives)
		{
			Derive(*C, Setup.ReferenceLength);
			Points.Add({ C->AlphaDeg, C->DeltaDeg, C->CL, C->Cm });
		}
		const bool bControl = Condition.Control.IsSet();
		Outcome.Result = bControl ? aerotelic::aero::SolveTrim(Points[0], Points[1], Points[2], Outcome.CLRequired)
			: aerotelic::aero::SolveTrimAlpha(Points[0], Points[1], Outcome.CLRequired);
		FString Text = FString::Printf(TEXT("Base α = %g°, δ = %g°: C_L %.4f, C_m(cg) %.4f. dC_L/dα = %.4f, dC_m/dα = %.4f per degree%s."),
			Condition.Alpha0, Condition.Delta0, Points[0].CL, Points[0].Cm, Outcome.Result.dCL_dalpha, Outcome.Result.dCm_dalpha,
			bControl ? *FString::Printf(TEXT("; dC_L/dδ = %.4f, dC_m/dδ = %.4f per degree (%s)"), Outcome.Result.dCL_ddelta, Outcome.Result.dCm_ddelta, *Condition.Control.Label) : TEXT(""));
		if (!Outcome.Result.ok)
		{
			Finish(false, Text + TEXT("\nNO TRIM: ") + UTF8_TO_TCHAR(Outcome.Result.note.c_str()));
			return;
		}
		if (FMath::Abs(Outcome.Result.alpha_deg) > 25.0 || FMath::Abs(Outcome.Result.delta_deg) > 45.0)
		{
			Finish(false, Text + FString::Printf(TEXT("\nNO TRIM: the step asks for α = %.2f°, δ = %.2f°, outside anything a potential flow describes (|α| < 25°, |δ| < 45°). The condition is too slow or too heavy for these bodies, or the control too small."), Outcome.Result.alpha_deg, Outcome.Result.delta_deg));
			return;
		}
		Outcome.Report = Text;
		if (OnStatus) OnStatus(FText::FromString(FString::Printf(TEXT("Trim '%s': step to α = %.3f°, δ = %.3f°; solving the trimmed state..."), *Condition.Name, Outcome.Result.alpha_deg, Outcome.Result.delta_deg)));
		TArray<TSharedPtr<FCase>> Cases;
		try
		{
			Cases.Add(BuildCase(Setup, Outcome.Result.alpha_deg, 0.0, bControl ? &Condition.Control : nullptr, Outcome.Result.delta_deg));
		}
		catch (const std::exception& E)
		{
			Finish(false, Text + FString(TEXT("\nThe trimmed state failed to panel: ")) + UTF8_TO_TCHAR(E.what()));
			return;
		}
		Outcome.Trimmed = Cases[0];
		TWeakPtr<FRunner> Weak = AsShared();
		SolveCases(Cases, SolveOptions(Setup, &Condition.MassCentre), [Weak](bool bOk, const FString& Why)
		{
			TSharedPtr<FRunner> Self = Weak.Pin();
			if (!Self.IsValid()) return;
			if (!bOk) { Self->Finish(false, Why); return; }
			FCase& C = *Self->Outcome.Trimmed;
			Derive(C, Self->Setup.ReferenceLength);
			const FOutcome& O = Self->Outcome;
			const double Lift = C.CL * O.DynamicPressure * O.ReferenceAreaSI;
			FString Text = Self->Outcome.Report;
			Text += FString::Printf(TEXT("\nTRIMMED at α = %.3f°%s: C_L %.4f against %.4f required (lift %.4g N against n W = %.4g N, %.2f%% off), C_m about the mass centre %.5f."),
				C.AlphaDeg, Self->Condition.Control.IsSet() ? *FString::Printf(TEXT(", %s at δ = %.3f°"), *Self->Condition.Control.Label, C.DeltaDeg) : TEXT(" (angle of attack alone; the moment is what it is)"),
				C.CL, O.CLRequired, Lift, Self->Condition.LoadFactor * O.Weight, 100.0 * (C.CL / FMath::Max(O.CLRequired, 1e-12) - 1.0), C.Cm);
			Text += TEXT("\nThe residual is the method's nonlinearity over the step (the wake sheds along the stream; a flap kinks the section): trim again from here to tighten it.");
			if (Self->Condition.Control.IsSet())
			{
				Text += FString::Printf(TEXT("\nThe model's own %s deflection is still %g°: set it to %.2f° in the wing's Flaps tab so the structural model's flap body sits where the air loads were computed."), *Self->Condition.Control.Label, Self->Condition.Delta0, C.DeltaDeg);
			}
			Self->Outcome.Case = MakeLoadCase(C, Self->Setup, Self->Condition.Name, O.DynamicPressure, &Self->Condition, &Self->Outcome);
			const aerotelic::Vec3 Fc = Self->Outcome.Case->ForceCoefficient();
			Text += FString::Printf(TEXT("\nLoad case '%s': %d body panels, q = %.4g Pa; the panels' own resultant over S_ref is (%.4f, %.4f, %.4f)%s."),
				*Self->Outcome.Case->Name, Self->Outcome.Case->BodyPanels(), Self->Outcome.Case->DynamicPressure, Fc.x, Fc.y, Fc.z,
				C.Shares.Num() && C.Shares[0].bImaged ? TEXT(" with the image about y = 0") : TEXT(""));
			Self->Outcome.Report = Text;
			Self->Finish(true, FString());
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
