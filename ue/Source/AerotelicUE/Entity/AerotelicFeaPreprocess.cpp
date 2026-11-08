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
	}

	bool Build(const AerotelicStructuralFea::FRequest& Request, FModel& Out, FString& OutError)
	{
		Out = FModel();
		if (Request.Nodes == nullptr || Request.Nodes->empty() || (Request.ShellFaces.Num() == 0 && Request.RibFaces.Num() == 0))
		{
			OutError = TEXT("nothing to analyse: no shells");
			return false;
		}
		const std::vector<aerotelic::Vec3>& Nodes = *Request.Nodes;
		const double L = Request.LengthToSI;
		Out.Nodes = static_cast<int32>(Nodes.size());
		for (const aerotelic::Vec3& P : Nodes) Out.Bounds.Add(P);
		Out.Diagonal = Out.Bounds.Diagonal();
		auto RegionOf = [&Request](int32 Id) { return Request.Regions.FindByPredicate([Id](const AerotelicStructuralFea::FRegion& G) { return G.Id == Id; }); };

		// The wetted faces: the skin regions' faces, as given and in order
		// (the script's order too, so a pressure per face maps face for
		// face).
		for (const AerotelicStructuralFea::FShellFace& S : Request.ShellFaces)
		{
			FFace F;
			F.N[0] = S.N[0]; F.N[1] = S.N[1]; F.N[2] = S.N[2];
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
			OutError = TEXT("the model has no wetted faces (ribs alone, no skin)");
			return false;
		}
		// The rib faces, as given.
		for (const AerotelicStructuralFea::FShellFace& S : Request.RibFaces)
		{
			FFace F;
			F.N[0] = S.N[0]; F.N[1] = S.N[1]; F.N[2] = S.N[2];
			F.Region = S.Region;
			if (F.N[0] < 0 || F.N[1] < 0 || F.N[2] < 0 || F.N[0] >= Out.Nodes || F.N[1] >= Out.Nodes || F.N[2] >= Out.Nodes)
			{
				OutError = TEXT("a rib face refers to a node the mesh does not have");
				return false;
			}
			Out.RibFaces.Add(F);
		}
		auto FaceAreaOf = [&Nodes](const FFace& F)
		{
			const aerotelic::Vec3& A = Nodes[F.N[0]];
			return 0.5 * aerotelic::Length(aerotelic::Cross(Nodes[F.N[1]] - A, Nodes[F.N[2]] - A));
		};

		// Per wetted face: centre, outward normal, area.
		const int32 NFaces = Out.WettedFaces.Num();
		Out.FaceCentre.SetNum(NFaces);
		Out.FaceNormal.SetNum(NFaces);
		Out.FaceArea.SetNum(NFaces);
		for (int32 i = 0; i < NFaces; ++i)
		{
			const FFace& F = Out.WettedFaces[i];
			const aerotelic::Vec3& A = Nodes[F.N[0]];
			const aerotelic::Vec3& B = Nodes[F.N[1]];
			const aerotelic::Vec3& C = Nodes[F.N[2]];
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
				for (int32 e = 0; e < 3; ++e) Edges.Add(aerotelic::Length(Nodes[F.N[(e + 1) % 3]] - Nodes[F.N[e]]));
			}
			if (Edges.Num() > 0)
			{
				Edges.Sort();
				Out.MedianEdge = Edges[Edges.Num() / 2];
			}
		}

		// The regions: counts and areas; the labels, kinds and the masses
		// from the request. The script's densities are SI and its areas
		// metres, so the masses are kg here too.
		std::map<int32, FRegionSummary> Regions;
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
		// Areal densities per region id (kg/m^2) for the mass and its
		// centre: shells as rho t on the region's wetted faces; rib shells as
		// rho t on the rib faces -- the script's rule.
		std::map<int32, double> SkinArealDensity, RibArealDensity;
		for (auto& Entry : Regions)
		{
			FRegionSummary& R = Entry.second;
			const AerotelicStructuralFea::FRegion* Spec = RegionOf(R.Id);
			if (Spec == nullptr)
			{
				OutError = FString::Printf(TEXT("the mesh has faces of region %d, which the job gives nothing for"), R.Id);
				return false;
			}
			R.Label = Spec->Label;
			const double Area = R.Area * L * L;
			const double RibArea = R.RibArea * L * L;
			R.Kind = Spec->HasShells() ? TEXT("shells") : TEXT("rib shells");
			if (Spec->HasShells() && Spec->HasRibs()) R.Kind += TEXT(" + rib shells");
			SkinArealDensity[R.Id] = Spec->HasShells() ? Spec->Skin.density * Spec->SkinThickness * L : 0.0;
			RibArealDensity[R.Id] = Spec->HasRibs() ? Spec->Rib.density * Spec->RibThickness * L : 0.0;
			R.Mass = SkinArealDensity[R.Id] * Area + RibArealDensity[R.Id] * RibArea;
		}
		for (const auto& Entry : Regions) Out.Regions.Add(Entry.second);
		{
			// The nodes that carry shells: those of the wetted faces of the
			// regions with a skin, and those of the rib faces. Every node of
			// the mesh should be one; the solver gives each six degrees of
			// freedom.
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
			Out.Dofs = 6ll * Out.Nodes;
		}

		// The mass and its centre (project units), mass-weighted over the
		// shells and the rib shells.
		double Mass = 0.0;
		aerotelic::Vec3 First;
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
			First += (Nodes[F.N[0]] + Nodes[F.N[1]] + Nodes[F.N[2]]) * (M / 3.0);
		}
		Out.Mass = Mass;
		Out.MassCentre = Mass > 0.0 ? ToF(First / Mass) : ToF(Out.Bounds.Center());
		Out.GravityForce = FVector(0.0, 0.0, -Mass * Request.LoadFactor * kG0);

		// The supports: the script's _support_nodes, in project units (the
		// tolerance scales with the mesh, so the set is the same as in
		// metres); a node on any plane is fixed, once.
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
					BodyBounds.FindOrAdd(It->second).Add(Nodes[Node]);
				};
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
					if (!NodeOnPlane(Nodes[n], P)) continue;
					const double D = std::abs(Nodes[n][P.Axis] - P.Value);
					++P.Nodes; Fixed[n] = true; P.Furthest = FMath::Max(P.Furthest, D);
				}
				Out.SupportPlanes.Add(P);
			}
			for (int32 n = 0; n < Out.Nodes; ++n) if (Fixed[n]) Out.SupportNodes.Add(n);
		}

		// The load path: union-find over the faces' nodes, then which pieces
		// hold a support node, then which regions have none that does.
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

		// The point forces: each at its nearest node.
		for (const AerotelicStructuralFea::FPointForce& F : Request.PointForces)
		{
			FModel::FPointLoad P;
			double Best = 1e300;
			for (int32 n = 0; n < Out.Nodes; ++n)
			{
				const double D = aerotelic::DistanceSq(Nodes[n], F.At);
				if (D < Best) { Best = D; P.Node = n; }
			}
			if (P.Node < 0) continue;
			P.At = ToF(Nodes[P.Node]);
			P.Asked = ToF(F.At);
			P.SnapDistance = std::sqrt(Best);
			P.Force = ToF(F.Force);
			Out.PointResultant += P.Force;
			Out.PointLoads.Add(P);
		}
		return true;
	}
}
