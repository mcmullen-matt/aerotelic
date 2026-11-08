#include "AerotelicFeaModelBuilder.h"

#include "AerotelicBodyStructure.h"
#include "AerotelicEntityActor.h"

#include "aerotelic/mass_properties.hpp"
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
		TArray<AerotelicTetGmsh::FRequest::FRib> RibsOf(const TArray<FPlacedNetwork>& Networks, const aerotelic::BodyStructure& S)
		{
			TArray<AerotelicTetGmsh::FRequest::FRib> Out;
			for (const FPlacedNetwork& N : Networks)
			{
				for (const aerotelic::Plane& P : N.Planes)
				{
					AerotelicTetGmsh::FRequest::FRib R;
					R.Id = N.Index + 1;
					R.Normal = P.normal;
					R.Offset = P.offset;
					R.Height = S.networks[N.Index].height;
					Out.Add(R);
				}
			}
			return Out;
		}

		// gmsh over a surface, cached by the surface's bytes and the request's
		// figures.
		const AerotelicTetGmsh::FResult* TetMeshFor(FCache& Cache, const aerotelic::TriMesh& Surface, const AerotelicTetGmsh::FRequest& Request, const FString& Label,
			const FOptions& Options, FString& OutError)
		{
			uint32 Crc = FCrc::MemCrc32(Surface.vertices.data(), static_cast<int32>(Surface.vertices.size() * sizeof(Surface.vertices[0])));
			Crc = FCrc::MemCrc32(Surface.triangles.data(), static_cast<int32>(Surface.triangles.size() * sizeof(Surface.triangles[0])), Crc);
			FString Key = FString::Printf(TEXT("%s|%u|%d|%d|%.9g|%d|%.9g"), *Label, Crc, static_cast<int32>(Surface.vertices.size()), static_cast<int32>(Surface.triangles.size()),
				Request.CoreSize, Request.bVolume ? 1 : 0, Request.RibSize);
			for (const AerotelicTetGmsh::FRequest::FRib& R : Request.Ribs)
			{
				Key += FString::Printf(TEXT("|%d:%.9g,%.9g,%.9g,%.9g,%.9g"), R.Id, R.Normal.x, R.Normal.y, R.Normal.z, R.Offset, R.Height);
			}
			if (const TSharedPtr<AerotelicTetGmsh::FResult>* Found = Cache.Tets.Find(Key))
			{
				return Found->Get();
			}
			if (Options.Status)
			{
				Options.Status(Request.bVolume
					? FString::Printf(TEXT("Building the model: meshing %s's own grid (%s triangles) with tetrahedra at %.4g%s..."), *Label, *Thousands((int64)Surface.triangles.size()), Request.CoreSize,
						Request.Ribs.Num() ? *FString::Printf(TEXT(", %d rib plane(s) embedded"), Request.Ribs.Num()) : TEXT(""))
					: FString::Printf(TEXT("Building the model: meshing the %d rib plane(s) of %s..."), Request.Ribs.Num(), *Label));
			}
			TSharedPtr<AerotelicTetGmsh::FResult> R = MakeShared<AerotelicTetGmsh::FResult>();
			if (!AerotelicTetGmsh::Run(Request, *R, OutError))
			{
				return nullptr;
			}
			UE_LOG(LogTemp, Log, TEXT("AerotelicFeaModelBuilder: %s (%d triangles) meshed: %d nodes, %d tetrahedra, %d rib faces (%d not conforming); %.6g vs body %.6g (relative %.2e); quality min %.3f mean %.3f; dihedral %.2f..%.2f deg, %d under 10, %d over 160; %.2f s"),
				*Label, static_cast<int32>(Surface.triangles.size()), R->Nodes, R->SkinTets + R->CoreTets, R->RibFaces.Num(), R->RibFacesMissing, R->SkinVolume + R->CoreVolume, R->OuterVolume, R->VolumeError,
				R->MinQuality, R->MeanQuality, R->MinDihedral, R->MaxDihedral, R->TetsDihedralUnder10, R->TetsDihedralOver160, R->Seconds);
			Cache.Tets.Add(Key, R);
			return R.Get();
		}

		// Appends the tetrahedra (region id per tetrahedron through RegionOf)
		// and returns the node offset they were appended at.
		int32 AppendTets(aerotelic::TetMesh& Mesh, const aerotelic::TetMesh& From, TFunction<int32(int32)> RegionOf)
		{
			const int32 Offset = static_cast<int32>(Mesh.nodes.size());
			Mesh.nodes.insert(Mesh.nodes.end(), From.nodes.begin(), From.nodes.end());
			for (size_t t = 0; t < From.tets.size(); ++t)
			{
				const auto& T = From.tets[t];
				Mesh.tets.push_back({T[0] + Offset, T[1] + Offset, T[2] + Offset, T[3] + Offset});
				Mesh.material.push_back(RegionOf(t < From.material.size() ? From.material[t] : 1));
			}
			return Offset;
		}

		// The surface's vertices are the first nodes of a gmsh result, in
		// order: checked, since the shell faces index them that way.
		bool SurfaceNodesFirst(const aerotelic::TriMesh& Surface, const aerotelic::TetMesh& Mesh, double Tol)
		{
			if (Mesh.nodes.size() < Surface.vertices.size()) return false;
			for (size_t i = 0; i < Surface.vertices.size(); ++i)
			{
				if (aerotelic::DistanceSq(Mesh.nodes[i], Surface.vertices[i]) > Tol * Tol) return false;
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
		const FString Material = UTF8_TO_TCHAR(S.material.name.c_str());
		const FString Length = UTF8_TO_TCHAR(Units.LengthName());
		const bool bTenNode = Options.bSecondOrder;
		const bool bHasInfill = S.IsSkin() && S.infill.has_value();
		const bool bInfill = bHasInfill && Options.Model != EModel::IgnoreInterior;
		const bool bFull = Options.Model == EModel::FullResolution;
		FString Ribs;
		if (S.IsSkin() && !S.networks.empty())
		{
			int32 Planes = 0;
			TArray<FString> Names;
			for (const aerotelic::PlaneNetwork& N : S.networks) { Planes += N.count; Names.Add(FString::Printf(TEXT("%s (%d x %.4g %s %s)"), UTF8_TO_TCHAR(N.name.c_str()), N.count, N.thickness, *Length, UTF8_TO_TCHAR(N.material.name.c_str()))); }
			if (Options.Model == EModel::IgnoreInterior) Ribs = FString::Printf(TEXT("; %d plane network(s) ignored, as asked under Model"), static_cast<int32>(S.networks.size()));
			else if (bFull && bInfill) Ribs = FString::Printf(TEXT("; the %d plane network(s) are LEFT OUT of the full-resolution model"), static_cast<int32>(S.networks.size()));
			else Ribs = FString::Printf(TEXT("; %d rib plane(s) as unloaded shells embedded in it, their rims on the skin's nodes: %s"), Planes, *FString::Join(Names, TEXT(", ")));
		}
		FString Text;
		if (bInfill)
		{
			const aerotelic::LatticeSpec& L = S.infill->lattice;
			const FString InfillMaterial = UTF8_TO_TCHAR(S.infill->material.name.c_str());
			const FString Cell = FString::Printf(TEXT("the homogenised %s %s lattice at %.0f%% (cell %.4g %s, %d voxels across) of %s"),
				UTF8_TO_TCHAR(aerotelic::LatticeFamilyName(L.family)), UTF8_TO_TCHAR(L.type.c_str()), 100.0 * L.volume_fraction, L.cell_size, *Length, static_cast<int32>(Options.Voxels), *InfillMaterial);
			if (bFull)
			{
				const bool bGenerated = Op.Entity->GetGeneratedStructure(RoleOf(Op.Entity.Get(), Role)) != nullptr;
				Text = FString::Printf(TEXT("%s: %s skin %.4g %s -- the generated structure's skin / core tetrahedra%s: %s skin tetrahedra of %s over core tetrahedra with %s, %s%s"),
					*Label, *Material, S.skin_thickness, *Length, bGenerated ? TEXT("") : TEXT(" (NOT GENERATED: Scene tree > body > Generate Structure first)"),
					bTenNode ? TEXT("10-node") : TEXT("4-node"), *Material, *Cell, bTenNode ? TEXT("10-node too") : TEXT("4-node"), *Ribs);
			}
			else
			{
				Text = FString::Printf(TEXT("%s: %s -- shells %.4g %s thick on the body's OWN grid (the Mesh tab's, what the viewport shows) over 4-node tetrahedra at the cell size carrying %s%s"),
					*Label, *Material, S.skin_thickness, *Length, *Cell, *Ribs);
			}
		}
		else if (S.skin_thickness > 0.0)
		{
			Text = FString::Printf(TEXT("%s: %s -- a thin-walled body: shells %.4g %s thick on its own grid, nothing inside%s%s"), *Label, *Material, S.skin_thickness, *Length,
				bHasInfill ? TEXT(" (its infill ignored, as asked under Model)") : TEXT(""), *Ribs);
		}
		else
		{
			Text = FString::Printf(TEXT("%s: %s -- a solid: %s tetrahedra of %s at the element size (no skin thickness set)"), *Label, *Material, bTenNode ? TEXT("10-node") : TEXT("4-node"), *Material);
		}
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
		const bool bInfill = S.IsSkin() && S.infill.has_value() && Options.Model != EModel::IgnoreInterior;
		const bool bFull = Options.Model == EModel::FullResolution;

		// The networks: the surface cut by every plane, so the ribs' rims are
		// the skin's nodes. Not when the interior is ignored, and not in the
		// full-resolution model (its tetrahedra are the generation's).
		TArray<FPlacedNetwork> Networks;
		aerotelic::TriMesh Surface = Found->mesh;
		if (S.IsSkin() && !S.networks.empty() && Options.Model != EModel::IgnoreInterior)
		{
			if (bFull && bInfill)
			{
				B.Problems.Add(FString::Printf(TEXT("%s: its %d plane network(s) are left out of the full-resolution model (the generated skin / core tetrahedra know nothing of them); choose the as-designed model for the ribs"), *Label, static_cast<int32>(S.networks.size())));
			}
			else
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
		}
		const TArray<AerotelicTetGmsh::FRequest::FRib> Ribs = RibsOf(Networks, S);
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
				R.SecondOrder = 0;
				B.Regions.Add(R);
				RegionOfRib.Add(N.Index + 1, R.Id);
			}
		};
		auto AddRibFaces = [&](const TArray<AerotelicTetGmsh::FRibFace>& Faces, int32 Offset)
		{
			for (const AerotelicTetGmsh::FRibFace& F : Faces)
			{
				const int32* Region = RegionOfRib.Find(F.Id);
				if (Region == nullptr) continue;
				AerotelicStructuralFea::FShellFace G;
				G.N[0] = F.N[0] + Offset; G.N[1] = F.N[1] + Offset; G.N[2] = F.N[2] + Offset;
				G.Region = *Region;
				B.RibFaces.Add(G);
			}
		};

		if (bInfill)
		{
			// The infill's cell, voxelised once per distinct spec.
			const aerotelic::LatticeSpec& Spec = S.infill->lattice;
			const aerotelic::Material& InfillMaterial = S.infill->material;
			const aerotelic::UnitCellSamples* Cell = nullptr;
			try
			{
				B.Cells.push_back(std::make_unique<aerotelic::UnitCellSamples>(aerotelic::SampleUnitCell(Spec, static_cast<int32>(Options.Voxels))));
				Cell = B.Cells.back().get();
			}
			catch (const std::exception& E)
			{
				B.Problems.Add(FString::Printf(TEXT("%s: the infill's lattice cell could not be voxelised: %s"), *Label, UTF8_TO_TCHAR(E.what())));
				return false;
			}
			if (bFull)
			{
				// The generated structure's skin / core tetrahedra.
				const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(ResolvedRole);
				FString Note;
				const FAerotelicBodyStructure::FTetMesh* TM = G != nullptr ? G->GetTetMesh(Note) : nullptr;
				if (TM == nullptr)
				{
					B.Problems.Add(FString::Printf(TEXT("%s: no skin / core tetrahedral mesh (%s)"), *Label, G == nullptr ? TEXT("the structure has not been generated: Scene tree > body > Generate Structure") : *Note));
					return false;
				}
				const int32 SkinId = B.NextId++, CoreId = B.NextId++;
				aerotelic::TetMesh Tets = TM->Result.Mesh;
				if (Op.bMirror) for (aerotelic::Vec3& P : Tets.nodes) P.y = -P.y;
				AppendTets(B.Mesh, Tets, [SkinId, CoreId](int32 Region) { return Region == 2 ? CoreId : SkinId; });
				AerotelicStructuralFea::FRegion Skin;
				Skin.Id = SkinId; Skin.Label = Label + TEXT(" skin"); Skin.Core = Material; Skin.bSolid = true; Skin.Body = BodyIndex;
				B.Regions.Add(Skin);
				AerotelicStructuralFea::FRegion Core;
				Core.Id = CoreId; Core.Label = Label + TEXT(" core (infill)"); Core.Core = InfillMaterial; Core.Cell = Cell; Core.Body = BodyIndex;
				B.Regions.Add(Core);
				return true;
			}
			// Shells over a homogenised core, on the body's own grid: gmsh
			// fills exactly these triangles at the cell size, the ribs embedded.
			AerotelicTetGmsh::FRequest G;
			G.Outer = &Surface;
			G.Inner = nullptr;
			G.CoreSize = Spec.cell_size;
			G.Algorithm = 1;
			G.bOptimize = true;
			G.bOptimizeNetgen = true;
			G.bSizeFromBoundary = true;
			G.Ribs = Ribs;
			G.RibSize = Options.RibSize;
			FString Error;
			const AerotelicTetGmsh::FResult* TR = TetMeshFor(Cache, Surface, G, Label, Options, Error);
			if (TR == nullptr)
			{
				B.Problems.Add(FString::Printf(TEXT("%s: its own grid could not be filled with tetrahedra at the cell size: %s"), *Label, *Error));
				return false;
			}
			const int32 Id = B.NextId++;
			const int32 Offset = AppendTets(B.Mesh, TR->Mesh, [Id](int32) { return Id; });
			AerotelicStructuralFea::FRegion R;
			R.Id = Id; R.Label = Label; R.Body = BodyIndex;
			R.Skin = Material; R.SkinThickness = S.skin_thickness;
			R.Core = InfillMaterial; R.Cell = Cell;
			// A material card, not a bending problem: first order, whatever
			// the request says.
			R.SecondOrder = 0;
			B.Regions.Add(R);
			AddRibRegions();
			AddRibFaces(TR->RibFaces, Offset);
			if (TR->RibFacesMissing > 0) B.Problems.Add(FString::Printf(TEXT("%s: %d rib face(s) did not conform to the tetrahedra (%s)"), *Label, TR->RibFacesMissing, *FString::Join(TR->Notes, TEXT("; "))));
			return true;
		}

		const int32 Id = B.NextId++;
		AerotelicStructuralFea::FRegion R;
		R.Id = Id; R.Label = Label; R.Body = BodyIndex;
		if (S.skin_thickness > 0.0)
		{
			// A thin-walled body: shells on the (cut) surface; the ribs, when
			// any, meshed as discs on the surface's nodes without a volume.
			int32 Offset = static_cast<int32>(B.Mesh.nodes.size());
			if (Ribs.Num() > 0)
			{
				AerotelicTetGmsh::FRequest G;
				G.Outer = &Surface;
				G.Inner = nullptr;
				G.bVolume = false;
				G.Ribs = Ribs;
				G.RibSize = Options.RibSize;
				FString Error;
				const AerotelicTetGmsh::FResult* TR = TetMeshFor(Cache, Surface, G, Label, Options, Error);
				if (TR == nullptr)
				{
					B.Problems.Add(FString::Printf(TEXT("%s: the ribs could not be meshed: %s"), *Label, *Error));
					return false;
				}
				if (!SurfaceNodesFirst(Surface, TR->Mesh, 1e-9 * FMath::Max(Surface.Bounds().Diagonal(), 1e-12)))
				{
					B.Problems.Add(Label + TEXT(": the rib mesh does not carry the surface's nodes first"));
					return false;
				}
				Offset = static_cast<int32>(B.Mesh.nodes.size());
				B.Mesh.nodes.insert(B.Mesh.nodes.end(), TR->Mesh.nodes.begin(), TR->Mesh.nodes.end());
				AddRibRegions();
				AddRibFaces(TR->RibFaces, Offset);
			}
			else
			{
				B.Mesh.nodes.insert(B.Mesh.nodes.end(), Surface.vertices.begin(), Surface.vertices.end());
			}
			for (const auto& Tri : Surface.triangles)
			{
				AerotelicStructuralFea::FShellFace F;
				F.N[0] = Tri[0] + Offset; F.N[1] = Tri[1] + Offset; F.N[2] = Tri[2] + Offset;
				F.Region = Id;
				B.ShellFaces.Add(F);
			}
			R.Skin = Material; R.SkinThickness = S.skin_thickness;
		}
		else
		{
			// A solid: tetrahedra of the material at the element size.
			AerotelicTetGmsh::FRequest G;
			G.Outer = &Surface;
			G.Inner = nullptr;
			G.CoreSize = Options.ElementSize;
			G.Algorithm = 1;
			G.bOptimize = true;
			G.bOptimizeNetgen = true;
			FString Error;
			const AerotelicTetGmsh::FResult* TR = TetMeshFor(Cache, Surface, G, Label, Options, Error);
			if (TR == nullptr)
			{
				B.Problems.Add(Label + TEXT(": could not be meshed with tetrahedra: ") + Error);
				return false;
			}
			AppendTets(B.Mesh, TR->Mesh, [Id](int32) { return Id; });
			R.Core = Material; R.bSolid = true;
		}
		B.Regions.Add(R);
		return true;
	}
}
