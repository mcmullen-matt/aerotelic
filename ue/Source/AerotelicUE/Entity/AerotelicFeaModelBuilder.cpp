#include "AerotelicFeaModelBuilder.h"

#include "AerotelicBodyStructure.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"

namespace AerotelicFeaModelBuilder
{
	namespace
	{
		FString Thousands(int64 N) { return FText::AsNumber(N).ToString(); }

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
				if (bMirror) AerotelicMeshing::MirrorY(P.Mesh);
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

		// The skin: shells on the surface's own triangles.
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
		AerotelicStructuralFea::FRegion R;
		R.Id = Id; R.Label = Label; R.Body = BodyIndex;
		R.Skin = S.material; R.SkinThickness = S.skin_thickness;
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
				B.Regions.Add(LR);
				std::vector<int32> Map(L.Mesh.vertices.size(), -1);
				for (size_t i = 0; i < L.Mesh.vertices.size(); ++i)
				{
					const size_t Before = B.Nodes.size();
					Map[i] = Index.FindOrAdd(L.Mesh.vertices[i], B.Nodes);
					if (B.Nodes.size() == Before) ++Shared; else ++Added;
				}
				for (const auto& Tri : L.Mesh.triangles)
				{
					AerotelicStructuralFea::FShellFace F;
					F.N[0] = Map[Tri[0]]; F.N[1] = Map[Tri[1]]; F.N[2] = Map[Tri[2]];
					F.Region = LR.Id;
					B.RibFaces.Add(F);
					++Faces;
				}
			}
			UE_LOG(LogTemp, Log, TEXT("AerotelicFeaModelBuilder: %s: %d layer(s) joined to the skin: %d faces, %d nodes shared with the skin or earlier layers, %d new"), *Label, Layers.Num(), Faces, Shared, Added);
		}
		return true;
	}
}
