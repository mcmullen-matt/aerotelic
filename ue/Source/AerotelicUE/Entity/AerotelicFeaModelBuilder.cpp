#include "AerotelicFeaModelBuilder.h"

#include "AerotelicBodyStructure.h"
#include "AerotelicEntityActor.h"

#include "aerotelic/mesh_cut.hpp"

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

		// The networks of a body as placed planes (mirrored for the mirrored
		// half), one entry per network that can be built; a problem per
		// network that cannot.
		struct FPlacedNetwork
		{
			int32 Index = 0;
			TArray<aerotelic::Plane> Planes;
		};
		TArray<FPlacedNetwork> PlacedNetworks(const AAerotelicEntityActor* Entity, const aerotelic::BodyStructure& S, bool bMirror, TArray<FString>& Problems, const FString& Label)
		{
			TArray<FPlacedNetwork> Out;
			std::array<double, 9> R;
			aerotelic::Vec3 T;
			double Scale = 1.0;
			Entity->GetPlacementMatrix(R, T, Scale);
			for (int32 n = 0; n < static_cast<int32>(S.networks.size()); ++n)
			{
				const aerotelic::PlaneNetwork& Net = S.networks[n];
				const std::string Problem = aerotelic::PlaneNetworkProblem(Net);
				if (!Problem.empty())
				{
					Problems.Add(FString::Printf(TEXT("%s / %s: %s"), *Label, UTF8_TO_TCHAR(Net.name.c_str()), UTF8_TO_TCHAR(Problem.c_str())));
					continue;
				}
				FPlacedNetwork P;
				P.Index = n;
				for (int k = 0; k < Net.count; ++k)
				{
					aerotelic::Plane Plane = aerotelic::PlacedPlane(aerotelic::NetworkPlane(Net, k), R, T, Scale);
					if (bMirror) Plane = aerotelic::MirrorPlaneY(Plane);
					P.Planes.Add(Plane);
				}
				Out.Add(P);
			}
			return Out;
		}

		// The ribs of the placed networks as gmsh takes them: rib id = network
		// index + 1 within the body.
		TArray<AerotelicRibGmsh::FRequest::FRib> RibsOf(const TArray<FPlacedNetwork>& Networks, const aerotelic::BodyStructure& S)
		{
			TArray<AerotelicRibGmsh::FRequest::FRib> Out;
			for (const FPlacedNetwork& N : Networks)
			{
				for (const aerotelic::Plane& P : N.Planes)
				{
					AerotelicRibGmsh::FRequest::FRib R;
					R.Id = N.Index + 1;
					R.Normal = P.normal;
					R.Offset = P.offset;
					R.Height = S.networks[N.Index].height;
					Out.Add(R);
				}
			}
			return Out;
		}

		// gmsh's ribs over a surface, cached by the surface's bytes and the
		// request's figures.
		const AerotelicRibGmsh::FResult* RibMeshFor(FCache& Cache, const aerotelic::TriMesh& Surface, const AerotelicRibGmsh::FRequest& Request, const FString& Label,
			const FOptions& Options, FString& OutError)
		{
			uint32 Crc = FCrc::MemCrc32(Surface.vertices.data(), static_cast<int32>(Surface.vertices.size() * sizeof(Surface.vertices[0])));
			Crc = FCrc::MemCrc32(Surface.triangles.data(), static_cast<int32>(Surface.triangles.size() * sizeof(Surface.triangles[0])), Crc);
			FString Key = FString::Printf(TEXT("%s|%u|%d|%d|%.9g|%.9g"), *Label, Crc, static_cast<int32>(Surface.vertices.size()), static_cast<int32>(Surface.triangles.size()),
				Request.RibSize, Request.PlaneTolerance);
			for (const AerotelicRibGmsh::FRequest::FRib& R : Request.Ribs)
			{
				Key += FString::Printf(TEXT("|%d:%.9g,%.9g,%.9g,%.9g,%.9g"), R.Id, R.Normal.x, R.Normal.y, R.Normal.z, R.Offset, R.Height);
			}
			if (const TSharedPtr<AerotelicRibGmsh::FResult>* Found = Cache.Ribs.Find(Key))
			{
				return Found->Get();
			}
			if (Options.Status)
			{
				Options.Status(FString::Printf(TEXT("Building the model: meshing the %d rib plane(s) of %s on its own grid (%s triangles)..."), Request.Ribs.Num(), *Label, *Thousands((int64)Surface.triangles.size())));
			}
			TSharedPtr<AerotelicRibGmsh::FResult> R = MakeShared<AerotelicRibGmsh::FResult>();
			if (!AerotelicRibGmsh::Run(Request, *R, OutError))
			{
				return nullptr;
			}
			UE_LOG(LogTemp, Log, TEXT("AerotelicFeaModelBuilder: %s (%d triangles) ribs meshed: %d nodes (%d the surface's), %d rib faces of %.6g area; body volume %.6g; %.2f s%s%s"),
				*Label, static_cast<int32>(Surface.triangles.size()), static_cast<int32>(R->Nodes.size()), R->SurfaceNodes, R->RibFaces.Num(), R->RibArea, R->OuterVolume, R->Seconds,
				R->Notes.Num() ? TEXT("; ") : TEXT(""), *FString::Join(R->Notes, TEXT("; ")));
			Cache.Ribs.Add(Key, R);
			return R.Get();
		}

		// The surface's vertices are the first nodes of a gmsh result, in
		// order: checked, since the shell faces index them that way.
		bool SurfaceNodesFirst(const aerotelic::TriMesh& Surface, const std::vector<aerotelic::Vec3>& Nodes, double Tol)
		{
			if (Nodes.size() < Surface.vertices.size()) return false;
			for (size_t i = 0; i < Surface.vertices.size(); ++i)
			{
				if (aerotelic::DistanceSq(Nodes[i], Surface.vertices[i]) > Tol * Tol) return false;
			}
			return true;
		}
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
		FString Ribs;
		if (!S.networks.empty())
		{
			int32 Planes = 0;
			TArray<FString> Names;
			for (const aerotelic::PlaneNetwork& N : S.networks) { Planes += N.count; Names.Add(FString::Printf(TEXT("%s (%d x %.4g %s %s)"), UTF8_TO_TCHAR(N.name.c_str()), N.count, N.thickness, *Length, UTF8_TO_TCHAR(N.material.name.c_str()))); }
			Ribs = FString::Printf(TEXT("; %d rib plane(s) as unloaded shells inside it, their rims on the skin's nodes: %s"), Planes, *FString::Join(Names, TEXT(", ")));
		}
		const FString Text = FString::Printf(TEXT("%s: %s -- a thin-walled body: shells %.4g %s thick on its own grid, nothing inside%s"), *Label, *Material, S.skin_thickness, *Length, *Ribs);
		if (bOutOk) *bOutOk = true;
		return Text;
	}

	bool AppendBody(const FAerotelicBooleanOperand& Op, const FString& Role, const FString& Label, int32 BodyIndex, const FOptions& Options, FCache& Cache, FBuilt& B)
	{
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
		if (!Found->mesh.IsClosedManifold() || !(Found->mesh.SignedVolume() > 0.0))
		{
			B.Problems.Add(Label + TEXT(": not a closed, outward surface"));
			return false;
		}
		const aerotelic::Material& Material = S.material;

		// The networks: the surface cut by every plane, so the ribs' rims are
		// the skin's nodes.
		TArray<FPlacedNetwork> Networks;
		aerotelic::TriMesh Surface = Found->mesh;
		if (!S.networks.empty())
		{
			Networks = PlacedNetworks(Entity, S, Op.bMirror, B.Problems, Label);
			std::vector<aerotelic::Plane> All;
			for (const FPlacedNetwork& N : Networks) for (const aerotelic::Plane& P : N.Planes) All.push_back(P);
			if (!All.empty())
			{
				const double Diag = Surface.Bounds().Diagonal();
				aerotelic::MeshCutStats Stats;
				Surface = aerotelic::CutMeshByPlanes(Surface, All, 1e-7 * Diag, nullptr, &Stats);
				if (!Surface.IsClosedManifold())
				{
					B.Problems.Add(Label + TEXT(": the surface cut by the rib planes is not a closed manifold"));
					return false;
				}
				UE_LOG(LogTemp, Log, TEXT("AerotelicFeaModelBuilder: %s cut by %d rib plane(s): %d triangles split, %d vertices added, %d snapped"),
					*Label, static_cast<int32>(All.size()), Stats.split_triangles, Stats.new_vertices, Stats.snapped_vertices);
			}
		}
		const TArray<AerotelicRibGmsh::FRequest::FRib> Ribs = RibsOf(Networks, S);
		// One region per network with ribs; the rib faces take them by id.
		TMap<int32, int32> RegionOfRib;
		auto AddRibRegions = [&]()
		{
			for (const FPlacedNetwork& N : Networks)
			{
				AerotelicStructuralFea::FRegion R;
				R.Id = B.NextId++;
				R.Label = FString::Printf(TEXT("%s / %s"), *Label, UTF8_TO_TCHAR(S.networks[N.Index].name.c_str()));
				R.Body = BodyIndex;
				R.Rib = S.networks[N.Index].material;
				R.RibThickness = S.networks[N.Index].thickness;
				B.Regions.Add(R);
				RegionOfRib.Add(N.Index + 1, R.Id);
			}
		};
		auto AddRibFaces = [&](const TArray<AerotelicRibGmsh::FRibFace>& Faces, int32 Offset)
		{
			for (const AerotelicRibGmsh::FRibFace& F : Faces)
			{
				const int32* Region = RegionOfRib.Find(F.Id);
				if (Region == nullptr) continue;
				AerotelicStructuralFea::FShellFace G;
				G.N[0] = F.N[0] + Offset; G.N[1] = F.N[1] + Offset; G.N[2] = F.N[2] + Offset;
				G.Region = *Region;
				B.RibFaces.Add(G);
			}
		};

		// A thin-walled body: shells on the (cut) surface; the ribs, when
		// any, meshed as discs on the surface's nodes.
		const int32 Id = B.NextId++;
		AerotelicStructuralFea::FRegion R;
		R.Id = Id; R.Label = Label; R.Body = BodyIndex;
		int32 Offset = static_cast<int32>(B.Nodes.size());
		if (Ribs.Num() > 0)
		{
			AerotelicRibGmsh::FRequest G;
			G.Outer = &Surface;
			G.Ribs = Ribs;
			G.RibSize = Options.RibSize;
			FString Error;
			const AerotelicRibGmsh::FResult* TR = RibMeshFor(Cache, Surface, G, Label, Options, Error);
			if (TR == nullptr)
			{
				B.Problems.Add(FString::Printf(TEXT("%s: the ribs could not be meshed: %s"), *Label, *Error));
				return false;
			}
			if (!SurfaceNodesFirst(Surface, TR->Nodes, 1e-9 * FMath::Max(Surface.Bounds().Diagonal(), 1e-12)))
			{
				B.Problems.Add(Label + TEXT(": the rib mesh does not carry the surface's nodes first"));
				return false;
			}
			Offset = static_cast<int32>(B.Nodes.size());
			B.Nodes.insert(B.Nodes.end(), TR->Nodes.begin(), TR->Nodes.end());
			AddRibRegions();
			AddRibFaces(TR->RibFaces, Offset);
		}
		else
		{
			B.Nodes.insert(B.Nodes.end(), Surface.vertices.begin(), Surface.vertices.end());
		}
		for (const auto& Tri : Surface.triangles)
		{
			AerotelicStructuralFea::FShellFace F;
			F.N[0] = Tri[0] + Offset; F.N[1] = Tri[1] + Offset; F.N[2] = Tri[2] + Offset;
			F.Region = Id;
			B.ShellFaces.Add(F);
		}
		R.Skin = Material; R.SkinThickness = S.skin_thickness;
		B.Regions.Add(R);
		return true;
	}
}
