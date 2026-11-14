#include "AerotelicFeaModelBuilder.h"

#include "AerotelicBodyStructure.h"
#include "AerotelicDocumentSubsystem.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"

#include "aerotelic/drape.hpp"
#include "aerotelic/laminate.hpp"
#include "aerotelic/mat.hpp"
#include "aerotelic/section_cut.hpp"
#include "aerotelic/structure_plan.hpp"
#include "aerotelic/wing_model.hpp"

#include <cmath>
#include <limits>

namespace AerotelicFeaModelBuilder
{
	namespace
	{
		using aerotelic::Vec3;

		FString Thousands(int64 N) { return FText::AsNumber(N).ToString(); }

		// The unit axis of the longest bounding-box extent of a mesh: the
		// reference direction of a body that is no loft (a primitive, a
		// boolean result: a wing-like one's span, a fuselage-like one's
		// length) and of a planar layer (a rib's chord, a spar's span).
		Vec3 LongestExtentAxis(const aerotelic::TriMesh& M)
		{
			const aerotelic::Box3 B = M.Bounds();
			const Vec3 E = B.Extent();
			if (E.x >= E.y && E.x >= E.z) return Vec3{1.0, 0.0, 0.0};
			if (E.y >= E.z) return Vec3{0.0, 1.0, 0.0};
			return Vec3{0.0, 0.0, 1.0};
		}

		// A direction per triangle of a body's grid: the gradient of the
		// loft's along-the-loft parameter u over the triangle (from the
		// plan's vertex parameters: the direction in which u grows is the
		// span of a wing, the length of a fuselage -- a laminator's
		// rosette), or where u is unknown or constant (a cap, a pocket face,
		// a body with no loft) the fallback axis projected into the triangle
		// (the next coordinate axis if that is normal to it). The fibres
		// have no arrow, so the sign of the gradient does not matter.
		std::vector<Vec3> ReferenceAxes(const aerotelic::TriMesh& M, const std::vector<double>* VertexU, const Vec3& Fallback)
		{
			std::vector<Vec3> Out(M.triangles.size());
			const bool bHaveU = VertexU != nullptr && VertexU->size() == M.vertices.size();
			for (size_t t = 0; t < M.triangles.size(); ++t)
			{
				const auto& Tri = M.triangles[t];
				const Vec3& P0 = M.vertices[Tri[0]];
				const Vec3& P1 = M.vertices[Tri[1]];
				const Vec3& P2 = M.vertices[Tri[2]];
				const Vec3 N2 = aerotelic::Cross(P1 - P0, P2 - P0);   // 2 A n
				const double TwoA = aerotelic::Length(N2);
				if (!(TwoA > 0.0)) { Out[t] = Fallback; continue; }
				const Vec3 N = N2 / TwoA;
				Vec3 Axis{};
				if (bHaveU)
				{
					const double U0 = (*VertexU)[Tri[0]], U1 = (*VertexU)[Tri[1]], U2 = (*VertexU)[Tri[2]];
					if (std::isfinite(U0) && std::isfinite(U1) && std::isfinite(U2))
					{
						// grad u = sum_i u_i grad lambda_i, grad lambda_i = n x e_i / 2A
						// with e_i the edge opposite vertex i, counter-clockwise.
						const Vec3 G = (aerotelic::Cross(N, P2 - P1) * U0 + aerotelic::Cross(N, P0 - P2) * U1 + aerotelic::Cross(N, P1 - P0) * U2) / TwoA;
						const double Scale = std::fabs(U0) + std::fabs(U1) + std::fabs(U2) + 1e-300;
						// A gradient that is not just round-off of equal values.
						if (aerotelic::Length(G) * (aerotelic::Length(P1 - P0) + aerotelic::Length(P2 - P0)) > 1e-7 * Scale) Axis = aerotelic::Normalized(G);
					}
				}
				if (aerotelic::LengthSq(Axis) == 0.0)
				{
					// The fallback projected into the triangle; if normal to it,
					// the next coordinate axis.
					Vec3 Candidates[3] = { Fallback, Vec3{Fallback.z, Fallback.x, Fallback.y}, Vec3{Fallback.y, Fallback.z, Fallback.x} };
					for (const Vec3& C : Candidates)
					{
						const Vec3 Proj = C - N * aerotelic::Dot(C, N);
						if (aerotelic::Length(Proj) > 1e-6) { Axis = aerotelic::Normalized(Proj); break; }
					}
				}
				Out[t] = Axis;
			}
			return Out;
		}

		// The DRAPE of a laminate skin over its body (core drape.hpp): for
		// every distinct ply direction of the stack one pin-jointed net from
		// a seed -- the grid vertex half-way along the loft and half-way
		// round the section (a wing's leading edge, a fuselage's keel), or
		// the vertex nearest the centroid when the body is no loft -- with
		// the warp along the face's reference direction turned by the ply's
		// angle; per face per ply the deviation of the draped warp from the
		// ply's nominal direction and the net's shear, as FRegion::SkinDrape
		// wants them (faces x plies x 2 floats). Returns false, with a note,
		// when no net could be laid (the drape is then left out and the
		// region analysed undraped).
		bool DrapeSkin(const aerotelic::TriMesh& Surface, const std::vector<Vec3>& Axes, const aerotelic::BodyStructurePlan* Plan,
			const aerotelic::Material& Laminate, TArray<float>& OutDrape, FString& OutNote, FString& OutSummary)
		{
			const std::vector<aerotelic::Ply> Stack = aerotelic::ExpandedStack(Laminate);
			const int32 NumFaces = static_cast<int32>(Surface.triangles.size());
			const int32 NumPlies = static_cast<int32>(Stack.size());
			if (NumPlies == 0 || NumFaces == 0) return false;
			// The seed vertex.
			int Seed = -1;
			if (Plan != nullptr && Plan->vertex_u.size() == Surface.vertices.size())
			{
				double UMin = std::numeric_limits<double>::infinity(), UMax = -std::numeric_limits<double>::infinity();
				for (double U : Plan->vertex_u) if (std::isfinite(U)) { UMin = std::min(UMin, U); UMax = std::max(UMax, U); }
				const double UMid = 0.5 * (UMin + UMax);
				double Best = std::numeric_limits<double>::infinity();
				for (size_t i = 0; i < Surface.vertices.size(); ++i)
				{
					const double U = Plan->vertex_u[i];
					const double V = i < Plan->vertex_v.size() ? Plan->vertex_v[i] : 0.5;
					if (!std::isfinite(U)) continue;
					const double Score = std::fabs(U - UMid) / std::max(UMax - UMin, 1e-300) + (std::isfinite(V) ? std::fabs(V - 0.5) : 0.5);
					if (Score < Best) { Best = Score; Seed = static_cast<int>(i); }
				}
			}
			if (Seed < 0)
			{
				const Vec3 C = Surface.Bounds().Center();
				double Best = std::numeric_limits<double>::infinity();
				for (size_t i = 0; i < Surface.vertices.size(); ++i)
				{
					const double D = aerotelic::DistanceSq(Surface.vertices[i], C);
					if (D < Best) { Best = D; Seed = static_cast<int>(i); }
				}
			}
			// A face at the seed, for the reference direction there.
			int SeedFace = -1;
			for (size_t t = 0; t < Surface.triangles.size() && SeedFace < 0; ++t)
			{
				const auto& Tri = Surface.triangles[t];
				if (Tri[0] == Seed || Tri[1] == Seed || Tri[2] == Seed) SeedFace = static_cast<int>(t);
			}
			if (SeedFace < 0) return false;
			const Vec3 SeedPoint = Surface.vertices[Seed];
			const Vec3 SeedAxis = Axes[SeedFace];
			const Vec3 SeedNormal = Surface.TriangleNormal(SeedFace);
			if (aerotelic::LengthSq(SeedAxis) == 0.0) return false;

			OutDrape.Init(0.0f, NumFaces * NumPlies * 2);
			// One net per distinct ply angle.
			TMap<int64, int32> Done;   // angle in millidegrees -> first ply index with that angle
			double MaxShear = 0.0, MaxDev = 0.0;
			int32 Nets = 0;
			double Covered = 1.0;
			for (int32 p = 0; p < NumPlies; ++p)
			{
				const int64 Key = static_cast<int64>(std::llround(Stack[p].angle * 1000.0));
				if (const int32* First = Done.Find(Key))
				{
					for (int32 f = 0; f < NumFaces; ++f)
					{
						OutDrape[(f * NumPlies + p) * 2] = OutDrape[(f * NumPlies + *First) * 2];
						OutDrape[(f * NumPlies + p) * 2 + 1] = OutDrape[(f * NumPlies + *First) * 2 + 1];
					}
					continue;
				}
				Done.Add(Key, p);
				const double Theta = Stack[p].angle * PI / 180.0;
				aerotelic::DrapeOptions O;
				O.seed = SeedPoint;
				O.warp = SeedAxis * std::cos(Theta) + aerotelic::Cross(SeedNormal, SeedAxis) * std::sin(Theta);
				O.cell = Surface.Bounds().Diagonal() / 60.0;
				O.max_steps = 150;
				const aerotelic::DrapeResult D = aerotelic::DrapeNet(Surface, O);
				++Nets;
				if (D.cells == 0)
				{
					OutNote = FString::Printf(TEXT("the drape of the %g degree plies could not be laid%s"), Stack[p].angle, D.notes.empty() ? TEXT("") : *FString::Printf(TEXT(" (%s)"), UTF8_TO_TCHAR(D.notes[0].c_str())));
					OutDrape.Reset();
					return false;
				}
				Covered = std::min(Covered, D.covered_fraction);
				// The deviation from the PLY's nominal direction: the draped
				// warp's angle from the reference, less the ply's own angle,
				// wrapped to (-90, 90] (a fibre has no arrow).
				const std::vector<double> Dev = aerotelic::WarpDeviation(Surface, D, Axes);
				for (int32 f = 0; f < NumFaces; ++f)
				{
					if (!D.covered[f]) continue;
					double A = Dev[f] - Stack[p].angle;
					while (A > 90.0) A -= 180.0;
					while (A <= -90.0) A += 180.0;
					OutDrape[(f * NumPlies + p) * 2] = static_cast<float>(A);
					OutDrape[(f * NumPlies + p) * 2 + 1] = static_cast<float>(D.shear[f]);
					MaxDev = std::max(MaxDev, std::fabs(A));
					MaxShear = std::max(MaxShear, D.shear[f]);
				}
			}
			OutSummary = FString::Printf(TEXT("%d net(s) of pitch %.4g, %.0f%% of the skin reached: cloth sheared up to %.1f deg, fibres turned up to %.1f deg"),
				Nets, Surface.Bounds().Diagonal() / 60.0, 100.0 * Covered, MaxShear, MaxDev);
			return true;
		}

		// The project's length unit in metres: a laminate's thickness is its
		// stack's, in SI, and the region wants it in project units.
		double ProjectLengthToSI()
		{
			const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			return Doc != nullptr ? Doc->GetUnits().LengthToSI() : 1.0;
		}

		// The plan of the entity's body by role, or null.
		const aerotelic::BodyStructurePlan* PlanOf(const AAerotelicEntityActor* Entity, const FString& Role)
		{
			for (const aerotelic::BodyStructurePlan& P : Entity->GetLastStructurePlans())
			{
				if (FString(UTF8_TO_TCHAR(P.body.c_str())) == Role) return &P;
			}
			return nullptr;
		}

		bool HasStructuralMaterial(const aerotelic::BodyStructure& S)
		{
			return S.material.HasDensity() && S.material.IsStructural();
		}

		// The one refusal the shells-only model adds (2026-11-08): no skin,
		// nothing to make shells of. Describe and AppendBody say the same.
		FString NoSkinProblem(const FString& Label)
		{
			return FString::Printf(TEXT("%s: no skin thickness -- the analysis models shells only; give the body a skin thickness under Structure... in the Scene tree"), *Label);
		}

		// A body's generated layers that carry a mesh (the ribs, spars,
		// stringers; the Skin layer has none), placed like the body -- the
		// same transform and mirror GetPlacedLayerMeshes applies, so a layer
		// vertex that was a body vertex still is, to the bit.
		struct FPlacedLayer
		{
			FString Name;
			aerotelic::TriMesh Mesh;
			double Thickness = 0.0;
			aerotelic::Material Material;
		};
		TArray<FPlacedLayer> PlacedLayers(const AAerotelicEntityActor* Entity, const FString& Role, bool bMirror)
		{
			TArray<FPlacedLayer> Out;
			const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(Role);
			if (G == nullptr || !G->IsGenerated()) return Out;
			const FTransform Transform = Entity->GetActorTransform();
			for (const FAerotelicStructureLayer& L : G->GetLayers())
			{
				if (L.Mesh.IsEmpty() || !(L.Thickness > 0.0)) continue;
				FPlacedLayer P;
				P.Name = L.Name;
				P.Mesh = L.Mesh;
				P.Thickness = L.Thickness;
				P.Material = L.Material;
				AerotelicMeshing::PlaceMesh(P.Mesh, Transform);
				if (bMirror) Entity->ReflectMesh(P.Mesh);
				Out.Add(MoveTemp(P));
			}
			return Out;
		}

		// Nodes by position, quantised: a layer vertex within the step of a
		// surface vertex (or of an earlier layer's) is that node.
		struct FNodeKey
		{
			int64 X = 0, Y = 0, Z = 0;
			bool operator==(const FNodeKey& O) const { return X == O.X && Y == O.Y && Z == O.Z; }
		};
		uint32 GetTypeHash(const FNodeKey& K)
		{
			return HashCombine(HashCombine(::GetTypeHash(K.X), ::GetTypeHash(K.Y)), ::GetTypeHash(K.Z));
		}
		struct FNodeIndex
		{
			double Step = 1e-9;
			TMap<FNodeKey, int32> Index;
			FNodeKey KeyOf(const aerotelic::Vec3& P) const
			{
				return FNodeKey{ static_cast<int64>(FMath::RoundToDouble(P.x / Step)), static_cast<int64>(FMath::RoundToDouble(P.y / Step)), static_cast<int64>(FMath::RoundToDouble(P.z / Step)) };
			}
			void Add(const aerotelic::Vec3& P, int32 Node) { Index.FindOrAdd(KeyOf(P), Node); }
			int32 FindOrAdd(const aerotelic::Vec3& P, std::vector<aerotelic::Vec3>& Nodes)
			{
				const FNodeKey K = KeyOf(P);
				if (const int32* Found = Index.Find(K)) return *Found;
				const int32 N = static_cast<int32>(Nodes.size());
				Nodes.push_back(P);
				Index.Add(K, N);
				return N;
			}
		};
	}

	const aerotelic::BodyStructure* StructureOf(const AAerotelicEntityActor* Entity, const FString& Role)
	{
		if (Entity == nullptr) return nullptr;
		if (const aerotelic::BodyStructure* S = Entity->GetBodyStructure(Role)) return S;
		return Entity->GetBodies().Num() > 0 ? &Entity->GetBodyStructure(0) : nullptr;
	}

	TArray<int32> AttachmentNodes(const std::vector<aerotelic::Vec3>& Nodes, const TArray<AerotelicStructuralFea::FShellFace>& ShellFaces,
		const TArray<AerotelicStructuralFea::FShellFace>& RibFaces, const TArray<AerotelicStructuralFea::FRegion>& Regions,
		int32 BodyIndex, const aerotelic::Vec3& At, int32 Count)
	{
		TSet<int32> RegionIds;
		for (const AerotelicStructuralFea::FRegion& R : Regions) if (BodyIndex < 0 || R.Body == BodyIndex) RegionIds.Add(R.Id);
		TArray<bool> Candidate;
		Candidate.Init(false, static_cast<int32>(Nodes.size()));
		for (const AerotelicStructuralFea::FShellFace& F : ShellFaces)
		{
			if (!RegionIds.Contains(F.Region)) continue;
			for (int32 k = 0; k < 3; ++k) if (F.N[k] >= 0 && F.N[k] < Candidate.Num()) Candidate[F.N[k]] = true;
		}
		for (const AerotelicStructuralFea::FShellFace& F : RibFaces)
		{
			if (!RegionIds.Contains(F.Region)) continue;
			for (int32 k = 0; k < 3; ++k) if (F.N[k] >= 0 && F.N[k] < Candidate.Num()) Candidate[F.N[k]] = true;
		}
		TArray<TPair<double, int32>> Ranked;
		for (int32 n = 0; n < Candidate.Num(); ++n)
		{
			if (Candidate[n]) Ranked.Add(TPair<double, int32>(aerotelic::DistanceSq(Nodes[n], At), n));
		}
		Ranked.Sort([](const TPair<double, int32>& A, const TPair<double, int32>& B) { return A.Key < B.Key || (A.Key == B.Key && A.Value < B.Value); });
		TArray<int32> Out;
		for (int32 i = 0; i < Ranked.Num() && i < FMath::Max(Count, 0); ++i) Out.Add(Ranked[i].Value);
		Out.Sort();
		return Out;
	}

	FString RoleOf(const AAerotelicEntityActor* Entity, const FString& Role)
	{
		if (!Role.IsEmpty() || Entity == nullptr) return Role;
		const TArray<FAerotelicBody>& Bodies = Entity->GetBodies();
		return Bodies.Num() > 0 ? Bodies[0].Name : FString();
	}

	int32 ResolveFlapHinges(const TArray<FChosenBody>& Chosen, const std::vector<aerotelic::Vec3>& Nodes,
		const TArray<AerotelicStructuralFea::FShellFace>& ShellFaces, const TArray<AerotelicStructuralFea::FShellFace>& RibFaces,
		const TArray<AerotelicStructuralFea::FRegion>& Regions, AerotelicStructuralFea::FRequest& Request, TArray<FString>& Notes)
	{
		using namespace AerotelicStructuralFea;
		static const FString kPort = TEXT(" (port)");
		const double LengthToSI = ProjectLengthToSI();
		int32 Hinged = 0;
		for (const FChosenBody& C : Chosen)
		{
			AAerotelicEntityActor* Entity = C.Operand.Entity.Get();
			if (Entity == nullptr || C.Index < 0) continue;
			// Only a wing has flaps. GetModelAs is an UNCHECKED cast: on a
			// boolean result's model it returned garbage whose "flaps" were
			// read here, and Build Model on my_drone's BoolOp 1 crashed with an
			// access violation (2026-11-14). Ask the type first.
			if (Entity->GetEntityType() != aerotelic::project_yaml::EntityType::Wing) continue;
			const aerotelic::WingModel* W = Entity->GetModelAs<aerotelic::WingModel>();
			if (W == nullptr) continue;
			// A flap body's role is the flap's name, or that plus " (port)" for
			// the port copy of a symmetric wing; the main element is "Main"
			// either way (a symmetric wing's is one body tip to tip).
			const bool bPort = C.Role.EndsWith(kPort);
			const FString FlapName = bPort ? C.Role.LeftChop(kPort.Len()) : C.Role;
			const aerotelic::FlapSpec* Flap = nullptr;
			for (const aerotelic::FlapSpec& F : W->flaps) if (UTF8_TO_TCHAR(F.name.c_str()) == FlapName) { Flap = &F; break; }
			if (Flap == nullptr) continue;
			const FString MainRole = TEXT("Main");
			const FChosenBody* Main = nullptr;
			for (const FChosenBody& D : Chosen)
			{
				if (D.Operand.Entity.Get() == Entity && D.Operand.bMirror == C.Operand.bMirror && D.Role == MainRole && D.Index >= 0) { Main = &D; break; }
			}
			if (Main == nullptr)
			{
				Notes.Add(FString::Printf(TEXT("flap '%s' of %s: its main element (%s) is not chosen, so it has no hinge and floats"), *C.Role, *Entity->EntityName, *MainRole));
				continue;
			}
			// Has either body any node in the model (not refused, not trimmed)?
			auto HasNodes = [&](int32 Body) { return AttachmentNodes(Nodes, ShellFaces, RibFaces, Regions, Body, aerotelic::Vec3{}, 1).Num() > 0; };
			if (!HasNodes(C.Index) || !HasNodes(Main->Index))
			{
				Notes.Add(FString::Printf(TEXT("flap '%s' of %s: the flap or its main element has no nodes in the model, so no hinge was built"), *C.Role, *Entity->EntityName));
				continue;
			}

			// A point of the flap's section at station eta, in airfoil
			// coordinates, placed: the wing's planform puts it in the model,
			// the port copy is mirrored in the model's y = 0, the actor
			// transform places it (the replica), the operand's mirror reflects
			// it.
			const FTransform Transform = Entity->GetActorTransform();
			auto Place = [&](double Eta, double X, double Y) -> aerotelic::Vec3
			{
				aerotelic::Mat P = W->planform().AirfoilCoordsToXyz(Eta, aerotelic::Mat::FromRows({{X, Y}}));
				aerotelic::TriMesh M;
				M.vertices.push_back(aerotelic::Vec3{P(0, 0), bPort ? -P(0, 1) : P(0, 1), P(0, 2)});
				AerotelicMeshing::PlaceMesh(M, Transform);
				if (C.Operand.bMirror) Entity->ReflectMesh(M);
				return M.vertices[0];
			};
			auto Nearest = [&](int32 Body, const aerotelic::Vec3& At) -> int32
			{
				const TArray<int32> N = AttachmentNodes(Nodes, ShellFaces, RibFaces, Regions, Body, At, 1);
				return N.Num() > 0 ? N[0] : -1;
			};
			auto CutAt = [&](double Eta, aerotelic::FlapCut& Cut) -> bool
			{
				try
				{
					const double Chord = W->planform().ChordAt(Eta);
					if (!(Chord > 0.0)) return false;
					std::shared_ptr<aerotelic::Airfoil> Af = W->AirfoilAt(Eta);
					Cut = aerotelic::SolveFlapCut(*Af, W->HingeFraction(*Flap, Eta), Flap->gap / Chord);
					return true;
				}
				catch (const std::exception&) { return false; }
			};

			// The brackets.
			const int32 NBrackets = FMath::Max(Flap->hinge_brackets, 1);
			int32 TiesAdded = 0;
			TSet<int32> SlavesUsed;
			for (int32 k = 0; k < NBrackets; ++k)
			{
				const double Eta = Flap->eta_root + (k + 0.5) / NBrackets * (Flap->eta_tip - Flap->eta_root);
				aerotelic::FlapCut Cut;
				if (!CutAt(Eta, Cut)) continue;
				const aerotelic::Vec3 Hinge = Place(Eta, Cut.x_hinge, Cut.y_hinge);
				const int32 Slave = Nearest(C.Index, Hinge);
				if (Slave < 0 || SlavesUsed.Contains(Slave)) continue;   // a coarse flap: two brackets on one node is one bracket
				const int32 Master = Nearest(Main->Index, Nodes[Slave]);
				if (Master < 0 || Master == Slave) continue;
				FTie T;
				T.Master = Master; T.Slave = Slave;
				Request.Ties.Add(T);
				SlavesUsed.Add(Slave);
				++TiesAdded;
			}
			if (TiesAdded == 0)
			{
				Notes.Add(FString::Printf(TEXT("flap '%s' of %s: no hinge bracket could be placed (the section cut failed at every bracket station)"), *C.Role, *Entity->EntityName));
				continue;
			}

			// The horn, the servo arm and the pushrod at the flap's mid-span.
			bool bRod = false;
			{
				const double Eta = 0.5 * (Flap->eta_root + Flap->eta_tip);
				aerotelic::FlapCut Cut;
				if (CutAt(Eta, Cut) && Flap->horn_arm > 0.0 && Flap->pushrod_forward > 0.0 && Flap->pushrod_diameter > 0.0)
				{
					// The horn is on the flap: it turns with the deflection
					// (clamped like the body's).
					double D = Flap->deflection_deg;
					if (Cut.deflection_down_limit_deg > 0.0 && D > Cut.deflection_down_limit_deg) D = Cut.deflection_down_limit_deg;
					if (Cut.deflection_up_limit_deg > 0.0 && D < -Cut.deflection_up_limit_deg) D = -Cut.deflection_up_limit_deg;
					const aerotelic::Mat HornXY = aerotelic::DeflectAboutHinge(aerotelic::Mat::FromRows({{Cut.x_hinge, Cut.y_hinge - Flap->horn_arm}}), Cut.x_hinge, Cut.y_hinge, D);
					const aerotelic::Vec3 Hinge = Place(Eta, Cut.x_hinge, Cut.y_hinge);
					const aerotelic::Vec3 Horn = Place(Eta, HornXY(0, 0), HornXY(0, 1));
					const aerotelic::Vec3 Servo = Place(Eta, Cut.x_hinge - Flap->pushrod_forward, Cut.y_hinge - Flap->horn_arm);
					const int32 OnFlap = Nearest(C.Index, Hinge);
					const int32 OnMain = Nearest(Main->Index, Servo);
					if (OnFlap >= 0 && OnMain >= 0 && aerotelic::DistanceSq(Horn, Servo) > 0.0)
					{
						FLink HornLink, ServoLink;
						HornLink.Name = FString::Printf(TEXT("%s horn"), *C.Role);
						HornLink.On = OnFlap; HornLink.At = Horn;
						ServoLink.Name = FString::Printf(TEXT("%s servo arm"), *C.Role);
						ServoLink.On = OnMain; ServoLink.At = Servo;
						const int32 HornIndex = Request.Links.Add(HornLink);
						const int32 ServoIndex = Request.Links.Add(ServoLink);
						FBar Rod;
						Rod.Name = FString::Printf(TEXT("%s pushrod"), *C.Role);
						Rod.A.Link = HornIndex;
						Rod.B.Link = ServoIndex;
						const double Dia = Flap->pushrod_diameter * LengthToSI;
						Rod.Area = 0.25 * PI * Dia * Dia;
						Rod.YoungsModulus = Flap->pushrod_youngs_modulus;
						Rod.Density = Flap->pushrod_density;
						Request.Bars.Add(Rod);
						bRod = true;
					}
				}
			}
			++Hinged;
			Notes.Add(FString::Printf(TEXT("flap '%s' of %s hinged to %s: %d bracket tie(s)%s%s"), *C.Role, *Entity->EntityName, *MainRole, TiesAdded,
				TiesAdded < NBrackets ? *FString::Printf(TEXT(" (%d asked; the flap's grid is too coarse for more)"), NBrackets) : TEXT(""),
				bRod ? *FString::Printf(TEXT(", a %.4g mm pushrod from a horn %.3g c below the hinge to a servo arm %.3g c ahead of it%s"), Flap->pushrod_diameter * LengthToSI * 1000.0, Flap->horn_arm, Flap->pushrod_forward,
					Flap->deflection_deg != 0.0 ? *FString::Printf(TEXT(", the flap deflected %.3g deg"), Flap->deflection_deg) : TEXT(""))
					: TEXT(", NO PUSHROD (a zero horn arm, forward reach or diameter): the flap can turn freely about its hinge, which makes the system singular")));
		}
		return Hinged;
	}

	FString Describe(const FAerotelicBooleanOperand& Op, const FString& Role, const FString& Label, const FOptions& Options, const aerotelic::UnitSystem& Units, bool* bOutOk)
	{
		if (bOutOk) *bOutOk = false;
		if (!Op.Entity.IsValid()) return Label + TEXT(": (removed entity)");
		const aerotelic::BodyStructure* Own = StructureOf(Op.Entity.Get(), Role);
		if (Own == nullptr) return Label + TEXT(": the body is not known yet (never built)");
		const aerotelic::BodyStructure& S = *Own;
		if (!HasStructuralMaterial(S)) return FString::Printf(TEXT("%s: NO STRUCTURAL MATERIAL -- set one under Structure... in the Scene tree"), *Label);
		if (!(S.skin_thickness > 0.0)) return NoSkinProblem(Label);
		const FString Material = UTF8_TO_TCHAR(S.material.name.c_str());
		const FString Length = UTF8_TO_TCHAR(Units.LengthName());
		FString Inside = TEXT("nothing inside");
		const TArray<FPlacedLayer> Layers = PlacedLayers(Op.Entity.Get(), RoleOf(Op.Entity.Get(), Role), Op.bMirror);
		if (Layers.Num() > 0)
		{
			TArray<FString> Names;
			for (const FPlacedLayer& L : Layers) Names.Add(FString::Printf(TEXT("%s (%.4g %s %s, %s triangles)"), *L.Name, L.Thickness, *Length, UTF8_TO_TCHAR(L.Material.name.c_str()), *Thousands((int64)L.Mesh.triangles.size())));
			Inside = FString::Printf(TEXT("%d generated layer(s) as unloaded shells inside it, on the skin's nodes: %s"), Layers.Num(), *FString::Join(Names, TEXT(", ")));
		}
		const FString Text = FString::Printf(TEXT("%s: %s -- a thin-walled body: shells %.4g %s thick on its own grid, %s"), *Label, *Material, S.skin_thickness, *Length, *Inside);
		if (bOutOk) *bOutOk = true;
		return Text;
	}

	bool AppendBody(const FAerotelicBooleanOperand& Op, const FString& Role, const FString& Label, int32 BodyIndex, const FOptions& Options, FCache& Cache, FBuilt& B)
	{
		(void)Cache;
		if (!Op.Entity.IsValid())
		{
			B.Problems.Add(Label + TEXT(": no longer in the scene"));
			return false;
		}
		AAerotelicEntityActor* Entity = Op.Entity.Get();
		const aerotelic::BodyStructure* Own = StructureOf(Entity, Role);
		if (Own == nullptr)
		{
			B.Problems.Add(Label + TEXT(": the body is not known yet (never built)"));
			return false;
		}
		const aerotelic::BodyStructure S = *Own;
		if (!HasStructuralMaterial(S))
		{
			B.Problems.Add(FString::Printf(TEXT("%s: NO STRUCTURAL MATERIAL -- set one under Structure... in the Scene tree"), *Label));
			return false;
		}
		{
			const std::string Problem = aerotelic::BodyStructureProblem(S);
			if (!Problem.empty())
			{
				B.Problems.Add(FString::Printf(TEXT("%s: %s"), *Label, UTF8_TO_TCHAR(Problem.c_str())));
				return false;
			}
		}
		// Shells only: a body without a skin has nothing to make them of.
		if (!(S.skin_thickness > 0.0))
		{
			B.Problems.Add(NoSkinProblem(Label));
			return false;
		}
		const FString ResolvedRole = RoleOf(Entity, Role);
		// The body's OWN grid: the triangles its Mesh tab defines and the
		// viewport shows (GetPlacedMeshes at tolerance 0).
		const std::vector<aerotelic::NamedMesh> Placed = Entity->GetPlacedMeshes(Op.bMirror);
		const aerotelic::NamedMesh* Found = nullptr;
		for (const aerotelic::NamedMesh& NM : Placed)
		{
			if (NM.mesh.IsEmpty()) continue;
			if (Placed.size() == 1 || FString(UTF8_TO_TCHAR(NM.name.c_str())) == ResolvedRole) { Found = &NM; break; }
		}
		if (Found == nullptr)
		{
			B.Problems.Add(Label + TEXT(": no geometry could be built"));
			return false;
		}
		const aerotelic::TriMesh& Surface = Found->mesh;
		if (!Surface.IsClosedManifold() || !(Surface.SignedVolume() > 0.0))
		{
			B.Problems.Add(Label + TEXT(": not a closed, outward surface"));
			return false;
		}
		if (Options.Status)
		{
			Options.Status(FString::Printf(TEXT("Building the model: %s on its own grid (%s triangles)..."), *Label, *Thousands((int64)Surface.triangles.size())));
		}

		// The skin: shells on the surface's own triangles, each with its
		// material reference direction (the along-the-loft direction from
		// the body's plan, else the longest extent).
		const int32 Offset = static_cast<int32>(B.Nodes.size());
		B.Nodes.insert(B.Nodes.end(), Surface.vertices.begin(), Surface.vertices.end());
		const int32 Id = B.NextId++;
		for (const auto& Tri : Surface.triangles)
		{
			AerotelicStructuralFea::FShellFace F;
			F.N[0] = Tri[0] + Offset; F.N[1] = Tri[1] + Offset; F.N[2] = Tri[2] + Offset;
			F.Region = Id;
			B.ShellFaces.Add(F);
		}
		const aerotelic::BodyStructurePlan* Plan = PlanOf(Entity, ResolvedRole);
		const std::vector<Vec3> Axes = ReferenceAxes(Surface, Plan != nullptr && Plan->HasParameters() ? &Plan->vertex_u : nullptr, LongestExtentAxis(Surface));
		for (const Vec3& A : Axes) B.ShellAxes.Add(A);
		AerotelicStructuralFea::FRegion R;
		R.Id = Id; R.Label = Label; R.Body = BodyIndex;
		R.Skin = S.material; R.SkinThickness = S.skin_thickness;
		if (S.material.IsLaminate())
		{
			// A laminate skin: the thickness is the stack's, and each ply is
			// draped over the body.
			R.SkinThickness = aerotelic::StackThickness(aerotelic::ExpandedStack(S.material)) / ProjectLengthToSI();
			if (Options.Status) Options.Status(FString::Printf(TEXT("Draping the %d plies of %s over %s..."), (int32)aerotelic::ExpandedStack(S.material).size(), UTF8_TO_TCHAR(S.material.name.c_str()), *Label));
			FString Note, Summary;
			const double T0 = FPlatformTime::Seconds();
			if (DrapeSkin(Surface, Axes, Plan, S.material, R.SkinDrape, Note, Summary))
			{
				UE_LOG(LogTemp, Log, TEXT("AerotelicFeaModelBuilder: %s: drape of '%s': %s (%.2f s)"), *Label, UTF8_TO_TCHAR(S.material.name.c_str()), *Summary, FPlatformTime::Seconds() - T0);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("AerotelicFeaModelBuilder: %s: %s; the laminate is analysed as laid flat along the reference direction"), *Label, Note.IsEmpty() ? TEXT("no drape could be computed") : *Note);
			}
		}
		B.Regions.Add(R);

		// The generated layers: a region of unloaded shells each, their
		// nodes matched by position onto the skin's and one another's.
		const TArray<FPlacedLayer> Layers = PlacedLayers(Entity, ResolvedRole, Op.bMirror);
		if (Layers.Num() > 0)
		{
			FNodeIndex Index;
			Index.Step = 1e-9 * FMath::Max(Surface.Bounds().Diagonal(), 1e-12);
			for (size_t i = 0; i < Surface.vertices.size(); ++i) Index.Add(Surface.vertices[i], Offset + static_cast<int32>(i));
			int32 Shared = 0, Added = 0, Faces = 0;
			for (const FPlacedLayer& L : Layers)
			{
				AerotelicStructuralFea::FRegion LR;
				LR.Id = B.NextId++;
				LR.Label = FString::Printf(TEXT("%s / %s"), *Label, *L.Name);
				LR.Body = BodyIndex;
				LR.Rib = L.Material.HasDensity() ? L.Material : S.material;
				LR.RibThickness = L.Thickness;
				// A laminate layer's thickness is its stack's (a planar member
				// is not draped: its plies lie as laid).
				if (LR.Rib.IsLaminate()) LR.RibThickness = aerotelic::StackThickness(aerotelic::ExpandedStack(LR.Rib)) / ProjectLengthToSI();
				B.Regions.Add(LR);
				std::vector<int32> Map(L.Mesh.vertices.size(), -1);
				for (size_t i = 0; i < L.Mesh.vertices.size(); ++i)
				{
					const size_t Before = B.Nodes.size();
					Map[i] = Index.FindOrAdd(L.Mesh.vertices[i], B.Nodes);
					if (B.Nodes.size() == Before) ++Shared; else ++Added;
				}
				// The layer's reference direction: its own longest extent (a
				// rib's chord, a spar's or stringer's span).
				const std::vector<Vec3> LayerAxes = ReferenceAxes(L.Mesh, nullptr, LongestExtentAxis(L.Mesh));
				for (size_t t = 0; t < L.Mesh.triangles.size(); ++t)
				{
					const auto& Tri = L.Mesh.triangles[t];
					AerotelicStructuralFea::FShellFace F;
					F.N[0] = Map[Tri[0]]; F.N[1] = Map[Tri[1]]; F.N[2] = Map[Tri[2]];
					F.Region = LR.Id;
					B.RibFaces.Add(F);
					B.RibAxes.Add(LayerAxes[t]);
					++Faces;
				}
			}
			UE_LOG(LogTemp, Log, TEXT("AerotelicFeaModelBuilder: %s: %d layer(s) joined to the skin: %d faces, %d nodes shared with the skin or earlier layers, %d new"), *Label, Layers.Num(), Faces, Shared, Added);
		}
		return true;
	}
}
