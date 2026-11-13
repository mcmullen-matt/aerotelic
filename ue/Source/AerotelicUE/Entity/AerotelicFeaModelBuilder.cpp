#include "AerotelicFeaModelBuilder.h"

#include "AerotelicBodyStructure.h"
#include "AerotelicDocumentSubsystem.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"

#include "aerotelic/drape.hpp"
#include "aerotelic/laminate.hpp"
#include "aerotelic/structure_plan.hpp"

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

	FString RoleOf(const AAerotelicEntityActor* Entity, const FString& Role)
	{
		if (!Role.IsEmpty() || Entity == nullptr) return Role;
		const TArray<FAerotelicBody>& Bodies = Entity->GetBodies();
		return Bodies.Num() > 0 ? Bodies[0].Name : FString();
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
