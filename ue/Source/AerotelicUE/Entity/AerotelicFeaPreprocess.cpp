#include "AerotelicFeaPreprocess.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace AerotelicFeaPreprocess
{
	namespace
	{
		constexpr double kG0 = 9.80665;   // the script's G0

		FVector ToF(const aerotelic::Vec3& V) { return FVector(V.x, V.y, V.z); }

		// A face keyed by its sorted node ids, the way np.unique(axis=0) on
		// the sorted rows keys it; sorted lexicographically that is the
		// script's wetted-face order.
		struct FKeyedFace
		{
			int32 Key[3];
			FFace Face;
			bool operator<(const FKeyedFace& O) const
			{
				if (Key[0] != O.Key[0]) return Key[0] < O.Key[0];
				if (Key[1] != O.Key[1]) return Key[1] < O.Key[1];
				return Key[2] < O.Key[2];
			}
			bool SameKey(const FKeyedFace& O) const { return Key[0] == O.Key[0] && Key[1] == O.Key[1] && Key[2] == O.Key[2]; }
		};
	}

	bool Build(const AerotelicStructuralFea::FRequest& Request, FModel& Out, FString& OutError)
	{
		Out = FModel();
		if (Request.Mesh == nullptr || (Request.Mesh->tets.empty() && Request.ShellFaces.Num() == 0 && Request.RibFaces.Num() == 0))
		{
			OutError = TEXT("nothing to analyse: no tetrahedra and no shells");
			return false;
		}
		const aerotelic::TetMesh& Mesh = *Request.Mesh;
		const double L = Request.LengthToSI;
		Out.Nodes = static_cast<int32>(Mesh.nodes.size());
		Out.Tets = static_cast<int32>(Mesh.tets.size());
		for (const aerotelic::Vec3& P : Mesh.nodes) Out.Bounds.Add(P);
		Out.Diagonal = Out.Bounds.Diagonal();
		auto RegionOf = [&Request](int32 Id) { return Request.Regions.FindByPredicate([Id](const AerotelicStructuralFea::FRegion& G) { return G.Id == Id; }); };

		// Every face of every tetrahedron, wound outward whatever the
		// tetrahedron's handedness (the script orients them positive first;
		// testing the fourth node against the face's normal is the same
		// thing without the copy).
		TArray<FKeyedFace> All;
		All.Reserve(Out.Tets * 4);
		std::vector<double> TetVolume(Mesh.tets.size(), 0.0);
		for (int32 t = 0; t < Out.Tets; ++t)
		{
			const auto& T = Mesh.tets[t];
			const aerotelic::Vec3& A = Mesh.nodes[T[0]];
			const aerotelic::Vec3& B = Mesh.nodes[T[1]];
			const aerotelic::Vec3& C = Mesh.nodes[T[2]];
			const aerotelic::Vec3& D = Mesh.nodes[T[3]];
			TetVolume[t] = std::abs(aerotelic::Dot(B - A, aerotelic::Cross(C - A, D - A))) / 6.0;
			const int32 Region = t < static_cast<int32>(Mesh.material.size()) ? Mesh.material[t] : 1;
			for (int32 f = 0; f < 4; ++f)
			{
				int32 N[3] = { T[aerotelic::TetMesh::kFace[f][0]], T[aerotelic::TetMesh::kFace[f][1]], T[aerotelic::TetMesh::kFace[f][2]] };
				const int32 Opposite = T[f];
				const aerotelic::Vec3& P0 = Mesh.nodes[N[0]];
				const aerotelic::Vec3 Normal = aerotelic::Cross(Mesh.nodes[N[1]] - P0, Mesh.nodes[N[2]] - P0);
				if (aerotelic::Dot(Normal, Mesh.nodes[Opposite] - P0) > 0.0)
				{
					std::swap(N[1], N[2]);   // the fourth node was on the normal's side: turn the face round
				}
				FKeyedFace K;
				K.Key[0] = N[0]; K.Key[1] = N[1]; K.Key[2] = N[2];
				std::sort(K.Key, K.Key + 3);
				K.Face.N[0] = N[0]; K.Face.N[1] = N[1]; K.Face.N[2] = N[2];
				K.Face.Tet = t;
				K.Face.Region = Region;
				All.Add(K);
			}
		}
		All.Sort();
		// A face that occurs once is on the boundary; one shared by two
		// tetrahedra (any regions: the skin / core interface is interior) is
		// not. Then the shell-only regions' faces, as given.
		for (int32 i = 0; i < All.Num();)
		{
			int32 j = i + 1;
			while (j < All.Num() && All[j].SameKey(All[i])) ++j;
			if (j - i == 1) Out.WettedFaces.Add(All[i].Face);
			i = j;
		}
		for (const AerotelicStructuralFea::FShellFace& S : Request.ShellFaces)
		{
			FFace F;
			F.N[0] = S.N[0]; F.N[1] = S.N[1]; F.N[2] = S.N[2];
			F.Tet = -1;
			F.Region = S.Region;
			if (F.N[0] < 0 || F.N[1] < 0 || F.N[2] < 0 || F.N[0] >= Out.Nodes || F.N[1] >= Out.Nodes || F.N[2] >= Out.Nodes)
			{
				OutError = TEXT("a shell face refers to a node the mesh does not have");
				return false;
			}
			Out.WettedFaces.Add(F);
		}
		if (Out.WettedFaces.Num() == 0)
		{
			OutError = TEXT("the mesh has no boundary faces");
			return false;
		}
		// The rib faces, as given.
		for (const AerotelicStructuralFea::FShellFace& S : Request.RibFaces)
		{
			FFace F;
			F.N[0] = S.N[0]; F.N[1] = S.N[1]; F.N[2] = S.N[2];
			F.Tet = -1;
			F.Region = S.Region;
			if (F.N[0] < 0 || F.N[1] < 0 || F.N[2] < 0 || F.N[0] >= Out.Nodes || F.N[1] >= Out.Nodes || F.N[2] >= Out.Nodes)
			{
				OutError = TEXT("a rib face refers to a node the mesh does not have");
				return false;
			}
			Out.RibFaces.Add(F);
		}
		auto FaceAreaOf = [&Mesh](const FFace& F)
		{
			const aerotelic::Vec3& A = Mesh.nodes[F.N[0]];
			return 0.5 * aerotelic::Length(aerotelic::Cross(Mesh.nodes[F.N[1]] - A, Mesh.nodes[F.N[2]] - A));
		};

		// Per face: centre, outward normal, area.
		const int32 NFaces = Out.WettedFaces.Num();
		Out.FaceCentre.SetNum(NFaces);
		Out.FaceNormal.SetNum(NFaces);
		Out.FaceArea.SetNum(NFaces);
		for (int32 i = 0; i < NFaces; ++i)
		{
			const FFace& F = Out.WettedFaces[i];
			const aerotelic::Vec3& A = Mesh.nodes[F.N[0]];
			const aerotelic::Vec3& B = Mesh.nodes[F.N[1]];
			const aerotelic::Vec3& C = Mesh.nodes[F.N[2]];
			const aerotelic::Vec3 NA = aerotelic::Cross(B - A, C - A) * 0.5;
			const double Area = aerotelic::Length(NA);
			Out.FaceCentre[i] = ToF((A + B + C) / 3.0);
			Out.FaceNormal[i] = Area > 0.0 ? ToF(NA / Area) : FVector::ZeroVector;
			Out.FaceArea[i] = Area;
		}
		{
			// The median edge of the wetted faces (every edge of every face,
			// shared ones counted twice, which does not move a median much).
			TArray<double> Edges;
			Edges.Reserve(3 * NFaces);
			for (int32 i = 0; i < NFaces; ++i)
			{
				const FFace& F = Out.WettedFaces[i];
				for (int32 e = 0; e < 3; ++e) Edges.Add(aerotelic::Length(Mesh.nodes[F.N[(e + 1) % 3]] - Mesh.nodes[F.N[e]]));
			}
			if (Edges.Num() > 0)
			{
				Edges.Sort();
				Out.MedianEdge = Edges[Edges.Num() / 2];
			}
		}

		// The tetrahedra's dihedral angles: the six interior angles between
		// the faces at each edge, from the outward face normals (face k
		// opposite vertex k; turned outward against the fourth vertex).
		{
			Out.MinDihedral = Out.Tets > 0 ? 180.0 : 0.0;
			Out.MaxDihedral = 0.0;
			constexpr int32 kFaceOf[4][3] = { {1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1} };
			constexpr int32 kPair[6][2] = { {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3} };
			for (int32 t = 0; t < Out.Tets; ++t)
			{
				const auto& T = Mesh.tets[t];
				aerotelic::Vec3 N[4];
				for (int32 k = 0; k < 4; ++k)
				{
					const aerotelic::Vec3& A = Mesh.nodes[T[kFaceOf[k][0]]];
					aerotelic::Vec3 Nk = aerotelic::Cross(Mesh.nodes[T[kFaceOf[k][1]]] - A, Mesh.nodes[T[kFaceOf[k][2]]] - A);
					if (aerotelic::Dot(Nk, Mesh.nodes[T[k]] - A) > 0.0) Nk = Nk * -1.0;
					const double NormalLength = aerotelic::Length(Nk);
					N[k] = NormalLength > 0.0 ? Nk / NormalLength : aerotelic::Vec3{};
				}
				double Lo = 180.0, Hi = 0.0;
				for (const auto& P : kPair)
				{
					const double C = FMath::Clamp(-aerotelic::Dot(N[P[0]], N[P[1]]), -1.0, 1.0);
					const double Deg = FMath::RadiansToDegrees(FMath::Acos(C));
					Lo = FMath::Min(Lo, Deg);
					Hi = FMath::Max(Hi, Deg);
				}
				Out.MinDihedral = FMath::Min(Out.MinDihedral, Lo);
				Out.MaxDihedral = FMath::Max(Out.MaxDihedral, Hi);
				if (Lo < 10.0) ++Out.TetsDihedralUnder10;
				if (Hi > 160.0) ++Out.TetsDihedralOver160;
			}
		}

		// The regions: counts, volumes, areas; the labels, kinds and the
		// masses from the request. The script's densities are SI and its
		// volumes metres, so the masses are kg here too.
		std::map<int32, FRegionSummary> Regions;
		for (int32 t = 0; t < Out.Tets; ++t)
		{
			const int32 Id = t < static_cast<int32>(Mesh.material.size()) ? Mesh.material[t] : 1;
			FRegionSummary& R = Regions[Id];
			R.Id = Id;
			++R.Tets;
			R.Volume += TetVolume[t];
		}
		// The system's size, the script's way: a mid-edge node on every
		// distinct edge of the second-order regions' tetrahedra, six degrees
		// of freedom on a corner node that carries a shell, three elsewhere.
		{
			std::map<int32, bool> SecondOrderOf;
			for (const AerotelicStructuralFea::FRegion& R : Request.Regions) SecondOrderOf[R.Id] = R.IsSecondOrder(Request.bSecondOrder);
			TArray<uint64> Edges;
			for (int32 t = 0; t < Out.Tets; ++t)
			{
				const int32 Id = t < static_cast<int32>(Mesh.material.size()) ? Mesh.material[t] : 1;
				const auto It = SecondOrderOf.find(Id);
				if (It == SecondOrderOf.end() || !It->second) continue;
				++Out.SecondOrderTets;
				const auto& T = Mesh.tets[t];
				for (int32 a = 0; a < 4; ++a)
				{
					for (int32 b = a + 1; b < 4; ++b)
					{
						const uint32 Lo = static_cast<uint32>(FMath::Min(T[a], T[b])), Hi = static_cast<uint32>(FMath::Max(T[a], T[b]));
						Edges.Add((static_cast<uint64>(Lo) << 32) | Hi);
					}
				}
			}
			Edges.Sort();
			for (int32 i = 0; i < Edges.Num(); ++i) if (i == 0 || Edges[i] != Edges[i - 1]) ++Out.MidEdgeNodes;
			for (auto& Entry : Regions)
			{
				const auto It = SecondOrderOf.find(Entry.first);
				Entry.second.bSecondOrder = It != SecondOrderOf.end() && It->second && Entry.second.Tets > 0;
			}
		}
		for (int32 i = 0; i < NFaces; ++i)
		{
			FRegionSummary& R = Regions[Out.WettedFaces[i].Region];
			R.Id = Out.WettedFaces[i].Region;
			++R.Faces;
			R.Area += Out.FaceArea[i];
		}
		for (const FFace& F : Out.RibFaces)
		{
			FRegionSummary& R = Regions[F.Region];
			R.Id = F.Region;
			++R.RibFaces;
			R.RibArea += FaceAreaOf(F);
		}
		// Densities per region id (kg/m^3) for the mass and its centre: a
		// solid region's own; a homogenised core's as the base density times
		// the cell's solid fraction, less the skin's overlap -- the script's
		// rule with the homogeniser's relative density stood in for by the
		// voxel fraction it is computed from; shells as rho t per unit area
		// on the region's faces; rib shells as rho t on the rib faces.
		std::map<int32, double> CoreDensity, SkinArealDensity, RibArealDensity;
		for (auto& Entry : Regions)
		{
			FRegionSummary& R = Entry.second;
			const AerotelicStructuralFea::FRegion* Spec = RegionOf(R.Id);
			if (Spec == nullptr)
			{
				OutError = FString::Printf(TEXT("the mesh has tetrahedra or faces of region %d, which the job gives nothing for"), R.Id);
				return false;
			}
			R.Label = Spec->Label;
			const double Volume = R.Volume * L * L * L;
			const double Area = R.Area * L * L;
			const double RibArea = R.RibArea * L * L;
			const double T = Spec->SkinThickness * L;
			double RhoCore = 0.0;
			if (Spec->Cell != nullptr)
			{
				const double Overlap = (Volume > 0.0 && T > 0.0) ? FMath::Min(Area * T / Volume, 1.0) : 0.0;
				RhoCore = Spec->Core.density * Spec->Cell->solid_fraction * (1.0 - Overlap);
				Out.bMassEstimated = true;
				R.Kind = Spec->HasShells() ? TEXT("shells over a homogenised core") : TEXT("homogenised core");
			}
			else if (Spec->bSolid)
			{
				RhoCore = Spec->Core.density;
				R.Kind = Spec->HasShells() ? TEXT("shells over a solid") : TEXT("solid");
			}
			else if (Spec->HasShells())
			{
				R.Kind = TEXT("shells");
			}
			else
			{
				R.Kind = TEXT("rib shells");
			}
			if (Spec->HasRibs() && !R.Kind.Equals(TEXT("rib shells"))) R.Kind += TEXT(" + rib shells");
			CoreDensity[R.Id] = RhoCore;
			SkinArealDensity[R.Id] = Spec->HasShells() ? Spec->Skin.density * T : 0.0;
			RibArealDensity[R.Id] = Spec->HasRibs() ? Spec->Rib.density * Spec->RibThickness * L : 0.0;
			R.Mass = RhoCore * Volume + SkinArealDensity[R.Id] * Area + RibArealDensity[R.Id] * RibArea;
		}
		for (const auto& Entry : Regions) Out.Regions.Add(Entry.second);
		{
			// The corner nodes that carry shells: those of the wetted faces of
			// the regions with a skin, and those of the rib faces.
			TArray<bool> OnShell;
			OnShell.Init(false, Out.Nodes);
			for (int32 i = 0; i < NFaces; ++i)
			{
				const AerotelicStructuralFea::FRegion* Spec = RegionOf(Out.WettedFaces[i].Region);
				if (Spec == nullptr || !Spec->HasShells()) continue;
				for (int32 k = 0; k < 3; ++k) OnShell[Out.WettedFaces[i].N[k]] = true;
			}
			for (const FFace& F : Out.RibFaces)
			{
				for (int32 k = 0; k < 3; ++k) OnShell[F.N[k]] = true;
			}
			for (bool B : OnShell) Out.ShellNodes += B ? 1 : 0;
			Out.Dofs = 3ll * (Out.Nodes + Out.MidEdgeNodes) + 3ll * Out.ShellNodes;
		}

		// The mass and its centre (project units), mass-weighted over the
		// tetrahedra, the shells and the rib shells.
		double Mass = 0.0;
		aerotelic::Vec3 First;
		for (int32 t = 0; t < Out.Tets; ++t)
		{
			const int32 Id = t < static_cast<int32>(Mesh.material.size()) ? Mesh.material[t] : 1;
			const double M = CoreDensity[Id] * TetVolume[t] * L * L * L;
			const auto& T = Mesh.tets[t];
			const aerotelic::Vec3 C = (Mesh.nodes[T[0]] + Mesh.nodes[T[1]] + Mesh.nodes[T[2]] + Mesh.nodes[T[3]]) * 0.25;
			Mass += M;
			First += C * M;
		}
		for (int32 i = 0; i < NFaces; ++i)
		{
			const double M = SkinArealDensity[Out.WettedFaces[i].Region] * Out.FaceArea[i] * L * L;
			Mass += M;
			First += aerotelic::Vec3{ Out.FaceCentre[i].X, Out.FaceCentre[i].Y, Out.FaceCentre[i].Z } * M;
		}
		for (const FFace& F : Out.RibFaces)
		{
			const double M = RibArealDensity[F.Region] * FaceAreaOf(F) * L * L;
			Mass += M;
			First += (Mesh.nodes[F.N[0]] + Mesh.nodes[F.N[1]] + Mesh.nodes[F.N[2]]) * (M / 3.0);
		}
		Out.Mass = Mass;
		Out.MassCentre = Mass > 0.0 ? ToF(First / Mass) : ToF(Out.Bounds.Center());
		Out.GravityForce = FVector(0.0, 0.0, -Mass * Request.LoadFactor * kG0);

		// The supports: the script's _support_nodes on the corner nodes, in
		// project units (the tolerance scales with the mesh, so the set is
		// the same as in metres); a node on any plane is fixed, once.
		{
			TArray<bool> Fixed;
			Fixed.Init(false, Out.Nodes);
			// The nodes of each chosen body (FRegion::Body), for a "min" /
			// "max" resolved against one body rather than the whole mesh.
			TMap<int32, aerotelic::Box3> BodyBounds;
			{
				std::map<int32, int32> RegionBody;
				for (const AerotelicStructuralFea::FRegion& R : Request.Regions) RegionBody[R.Id] = R.Body;
				auto AddNode = [&](int32 Region, int32 Node)
				{
					const auto It = RegionBody.find(Region);
					if (It == RegionBody.end() || It->second < 0) return;
					BodyBounds.FindOrAdd(It->second).Add(Mesh.nodes[Node]);
				};
				for (int32 t = 0; t < Out.Tets; ++t)
				{
					const int32 Id = t < static_cast<int32>(Mesh.material.size()) ? Mesh.material[t] : 1;
					for (int32 k = 0; k < 4; ++k) AddNode(Id, Mesh.tets[t][k]);
				}
				for (const FFace& F : Out.WettedFaces) for (int32 k = 0; k < 3; ++k) AddNode(F.Region, F.N[k]);
				for (const FFace& F : Out.RibFaces) for (int32 k = 0; k < 3; ++k) AddNode(F.Region, F.N[k]);
			}
			for (const AerotelicStructuralFea::FSupport& S : Request.Supports)
			{
				FModel::FPlane P;
				const TCHAR Axis = FChar::ToLower(S.Axis);
				P.Axis = Axis == TEXT('x') ? 0 : (Axis == TEXT('z') ? 2 : 1);
				P.Body = S.Body;
				const aerotelic::Box3* Extent = &Out.Bounds;
				if (S.Body >= 0 && (S.bAtMin || S.bAtMax))
				{
					const aerotelic::Box3* Own = BodyBounds.Find(S.Body);
					if (Own != nullptr && !Own->IsEmpty()) Extent = Own;
					else P.bBodyFallback = true;
				}
				if (S.bAtMin) P.Value = Extent->min[P.Axis];
				else if (S.bAtMax) P.Value = Extent->max[P.Axis];
				else P.Value = S.Value;
				// The script's rule (SUPPORT_TOLERANCE_OVER_DIAGONAL): a
				// ten-thousandth of the diagonal, which takes a lofted
				// section's whole ring (planar to about that); a millionth
				// took two of hershey_bar's thirty root nodes.
				P.Tolerance = S.Tolerance > 0.0 ? S.Tolerance : 1e-4 * Out.Diagonal;
				P.bBounded = S.bBounded;
				for (int32 k = 0; k < 2; ++k) { P.Lo[k] = S.Lo[k]; P.Hi[k] = S.Hi[k]; }
				for (int32 n = 0; n < Out.Nodes; ++n)
				{
					if (!NodeOnPlane(Mesh.nodes[n], P)) continue;
					const double D = std::abs(Mesh.nodes[n][P.Axis] - P.Value);
					++P.Nodes; Fixed[n] = true; P.Furthest = FMath::Max(P.Furthest, D);
				}
				Out.SupportPlanes.Add(P);
			}
			for (int32 n = 0; n < Out.Nodes; ++n) if (Fixed[n]) Out.SupportNodes.Add(n);
		}

		// The load path: union-find over the tetrahedra's and faces' nodes,
		// then which pieces hold a support node, then which regions have
		// none that does.
		{
			TArray<int32> Parent;
			Parent.SetNum(Out.Nodes);
			for (int32 n = 0; n < Out.Nodes; ++n) Parent[n] = n;
			auto Find = [&Parent](int32 n)
			{
				while (Parent[n] != n) { Parent[n] = Parent[Parent[n]]; n = Parent[n]; }
				return n;
			};
			auto Unite = [&Parent, &Find](int32 a, int32 b)
			{
				a = Find(a); b = Find(b);
				if (a != b) Parent[a] = b;
			};
			for (const auto& T : Mesh.tets) { Unite(T[0], T[1]); Unite(T[0], T[2]); Unite(T[0], T[3]); }
			for (const FFace& F : Out.WettedFaces) { Unite(F.N[0], F.N[1]); Unite(F.N[0], F.N[2]); }
			for (const FFace& F : Out.RibFaces) { Unite(F.N[0], F.N[1]); Unite(F.N[0], F.N[2]); }
			TMap<int32, int32> ComponentOfRoot;
			Out.NodeComponent.SetNum(Out.Nodes);
			for (int32 n = 0; n < Out.Nodes; ++n)
			{
				const int32 Root = Find(n);
				int32* C = ComponentOfRoot.Find(Root);
				if (C == nullptr) C = &ComponentOfRoot.Add(Root, ComponentOfRoot.Num());
				Out.NodeComponent[n] = *C;
			}
			Out.Components = ComponentOfRoot.Num();
			TArray<bool>& Supported = Out.ComponentSupported;
			Supported.Init(false, Out.Components);
			for (int32 n : Out.SupportNodes) Supported[Out.NodeComponent[n]] = true;
			TMap<int32, bool> RegionSupported;
			for (int32 t = 0; t < Out.Tets; ++t)
			{
				const int32 Id = t < static_cast<int32>(Mesh.material.size()) ? Mesh.material[t] : 1;
				bool& S = RegionSupported.FindOrAdd(Id, false);
				S = S || Supported[Out.NodeComponent[Mesh.tets[t][0]]];
			}
			for (const FFace& F : Out.WettedFaces)
			{
				bool& S = RegionSupported.FindOrAdd(F.Region, false);
				S = S || Supported[Out.NodeComponent[F.N[0]]];
			}
			for (const FFace& F : Out.RibFaces)
			{
				bool& S = RegionSupported.FindOrAdd(F.Region, false);
				S = S || Supported[Out.NodeComponent[F.N[0]]];
			}
			for (const FRegionSummary& R : Out.Regions)
			{
				const bool* S = RegionSupported.Find(R.Id);
				if (S == nullptr || !*S) Out.FloatingRegions.Add(R.Id);
			}
			for (int32 n = 0; n < Out.Nodes; ++n) if (!Supported[Out.NodeComponent[n]]) ++Out.FloatingNodes;
		}

		// The pressure per wetted face and its resultant, -p n A in newtons.
		Out.FacePressure.SetNum(NFaces);
		const bool bPerFace = Request.PressurePerFace.Num() == NFaces;
		for (int32 i = 0; i < NFaces; ++i)
		{
			const double P = bPerFace ? Request.PressurePerFace[i] : Request.Pressure;
			Out.FacePressure[i] = P;
			Out.MaxFacePressure = FMath::Max(Out.MaxFacePressure, std::abs(P));
			Out.PressureResultant -= Out.FaceNormal[i] * (P * Out.FaceArea[i] * L * L);
		}

		// The point forces: each at its nearest corner node.
		for (const AerotelicStructuralFea::FPointForce& F : Request.PointForces)
		{
			FModel::FPointLoad P;
			double Best = 1e300;
			for (int32 n = 0; n < Out.Nodes; ++n)
			{
				const double D = aerotelic::DistanceSq(Mesh.nodes[n], F.At);
				if (D < Best) { Best = D; P.Node = n; }
			}
			if (P.Node < 0) continue;
			P.At = ToF(Mesh.nodes[P.Node]);
			P.Asked = ToF(F.At);
			P.SnapDistance = std::sqrt(Best);
			P.Force = ToF(F.Force);
			Out.PointResultant += P.Force;
			Out.PointLoads.Add(P);
		}
		return true;
	}
}
