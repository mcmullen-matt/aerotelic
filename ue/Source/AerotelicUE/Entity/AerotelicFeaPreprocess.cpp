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
			Out.Dofs = 6ll * (Out.Nodes + Request.Masses.Num());
		}

		// The mass, its centre (project units) and the inertia tensor about
		// it (SI), over the shells and the rib shells as uniform laminae on
		// each triangle (exact: the script's shell_mass_moments) and the
		// mass items as points with their own tensors. The second-moment
		// matrix S = integral r r^T dm is accumulated about the origin in
		// metres, the tensor is tr(S) 1 - S shifted to the centre.
		double Mass = 0.0;
		aerotelic::Vec3 First;   // kg x project units
		double Second[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };   // kg m^2, about the origin
		auto AddTriangle = [&](const aerotelic::Vec3& A, const aerotelic::Vec3& B, const aerotelic::Vec3& C, double M)
		{
			Mass += M;
			const aerotelic::Vec3 Cen = (A + B + C) / 3.0;
			First += Cen * M;
			const aerotelic::Vec3 P[3] = { A * L, B * L, C * L };
			const aerotelic::Vec3 Cm = Cen * L;
			for (int32 i = 0; i < 3; ++i)
			{
				for (int32 j = 0; j < 3; ++j)
				{
					Second[3 * i + j] += (M / 12.0) * (P[0][i] * P[0][j] + P[1][i] * P[1][j] + P[2][i] * P[2][j] + 9.0 * Cm[i] * Cm[j]);
				}
			}
		};
		for (int32 i = 0; i < NFaces; ++i)
		{
			const FFace& F = Out.WettedFaces[i];
			AddTriangle(Nodes[F.N[0]], Nodes[F.N[1]], Nodes[F.N[2]], SkinArealDensity[F.Region] * Out.FaceArea[i] * L * L);
		}
		for (const FFace& F : Out.RibFaces)
		{
			AddTriangle(Nodes[F.N[0]], Nodes[F.N[1]], Nodes[F.N[2]], RibArealDensity[F.Region] * FaceAreaOf(F) * L * L);
		}
		Out.MassStructure = Mass;
		double OwnInertia[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		for (const AerotelicStructuralFea::FMassItem& Item : Request.Masses)
		{
			FModel::FMassItemView V;
			V.Name = Item.Name;
			V.Mass = Item.Mass;
			V.At = ToF(Item.At);
			V.AttachNodes = Item.AttachNodes;
			aerotelic::Vec3 Centre;
			int32 Counted = 0;
			for (int32 N : Item.AttachNodes)
			{
				if (N < 0 || N >= Out.Nodes) continue;
				Centre += Nodes[N];
				++Counted;
			}
			V.AttachCentre = Counted > 0 ? ToF(Centre / Counted) : V.At;
			Out.MassItems.Add(V);
			Out.MassOfItems += Item.Mass;
			Mass += Item.Mass;
			First += Item.At * Item.Mass;
			const aerotelic::Vec3 P = Item.At * L;
			for (int32 i = 0; i < 3; ++i) for (int32 j = 0; j < 3; ++j) Second[3 * i + j] += Item.Mass * P[i] * P[j];
			// Ixx, Iyy, Izz, Ixy, Ixz, Iyz with the products as integral x y dm:
			// the tensor's off-diagonals are their negatives.
			OwnInertia[0] += Item.Inertia[0]; OwnInertia[4] += Item.Inertia[1]; OwnInertia[8] += Item.Inertia[2];
			OwnInertia[1] -= Item.Inertia[3]; OwnInertia[3] -= Item.Inertia[3];
			OwnInertia[2] -= Item.Inertia[4]; OwnInertia[6] -= Item.Inertia[4];
			OwnInertia[5] -= Item.Inertia[5]; OwnInertia[7] -= Item.Inertia[5];
		}
		Out.Mass = Mass;
		Out.MassCentre = Mass > 0.0 ? ToF(First / Mass) : ToF(Out.Bounds.Center());
		Out.GravityForce = FVector(0.0, 0.0, -Mass * Request.LoadFactor * kG0);
		if (Mass > 0.0)
		{
			// Shift S to the mass centre: S' = S - f c^T - c f^T + m c c^T
			// with f the first moment (kg m) and c the centre (m).
			const aerotelic::Vec3 Cm = (First / Mass) * L;
			const aerotelic::Vec3 Fm = First * L;
			double Sp[9];
			for (int32 i = 0; i < 3; ++i) for (int32 j = 0; j < 3; ++j) Sp[3 * i + j] = Second[3 * i + j] - Fm[i] * Cm[j] - Cm[i] * Fm[j] + Mass * Cm[i] * Cm[j];
			const double Trace = Sp[0] + Sp[4] + Sp[8];
			for (int32 i = 0; i < 3; ++i) for (int32 j = 0; j < 3; ++j) Out.InertiaCg[3 * i + j] = (i == j ? Trace : 0.0) - Sp[3 * i + j] + OwnInertia[3 * i + j];
		}

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
			// Free flight: one node, the nearest to the reference (the mass
			// centre unless one was given), carries the six fixed degrees of
			// freedom; the planes are ignored.
			Out.bFreeFlight = Request.bFreeFlight;
			if (Request.bFreeFlight)
			{
				Out.Reference = Request.bReferenceGiven ? ToF(Request.Reference) : Out.MassCentre;
				const aerotelic::Vec3 Ref{ Out.Reference.X, Out.Reference.Y, Out.Reference.Z };
				double Best = 1e300;
				for (int32 n = 0; n < Out.Nodes; ++n)
				{
					const double D = aerotelic::DistanceSq(Nodes[n], Ref);
					if (D < Best) { Best = D; Out.SupportNode = n; }
				}
				if (Out.SupportNode >= 0)
				{
					Fixed[Out.SupportNode] = true;
					Out.SupportAt = ToF(Nodes[Out.SupportNode]);
				}
			}
			for (const AerotelicStructuralFea::FSupport& S : Request.bFreeFlight ? TArray<AerotelicStructuralFea::FSupport>() : Request.Supports)
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

		// Free flight: the rigid-body motion the loads give the free body,
		// a0 = F / m and alpha = I_cg^-1 M_cg (the moment of the pressures
		// at their face centres and of the point forces at their nodes about
		// the mass centre; gravity has none), the script's inertia_relief().
		Out.LoadFactorAtCg = Request.LoadFactor;
		if (Request.bFreeFlight && Mass > 0.0)
		{
			const FVector F = Out.PressureResultant + Out.PointResultant + Out.GravityForce;
			Out.RigidAcceleration = F / Mass;
			Out.InertialResultant = -F;
			FVector Moment = FVector::ZeroVector;   // N m
			for (int32 i = 0; i < NFaces; ++i)
			{
				const FVector Fi = -Out.FaceNormal[i] * (Out.FacePressure[i] * Out.FaceArea[i] * L * L);
				Moment += FVector::CrossProduct((Out.FaceCentre[i] - Out.MassCentre) * L, Fi);
			}
			for (const FModel::FPointLoad& P : Out.PointLoads) Moment += FVector::CrossProduct((P.At - Out.MassCentre) * L, P.Force);
			const double* I = Out.InertiaCg;
			const double Det = I[0] * (I[4] * I[8] - I[5] * I[7]) - I[1] * (I[3] * I[8] - I[5] * I[6]) + I[2] * (I[3] * I[7] - I[4] * I[6]);
			double Scale = 0.0;
			for (double V : Out.InertiaCg) Scale = FMath::Max(Scale, FMath::Abs(V));
			if (FMath::Abs(Det) > 1e-12 * Scale * Scale * Scale && Scale > 0.0)
			{
				double Inv[9] = {
					(I[4] * I[8] - I[5] * I[7]) / Det, (I[2] * I[7] - I[1] * I[8]) / Det, (I[1] * I[5] - I[2] * I[4]) / Det,
					(I[5] * I[6] - I[3] * I[8]) / Det, (I[0] * I[8] - I[2] * I[6]) / Det, (I[2] * I[3] - I[0] * I[5]) / Det,
					(I[3] * I[7] - I[4] * I[6]) / Det, (I[1] * I[6] - I[0] * I[7]) / Det, (I[0] * I[4] - I[1] * I[3]) / Det };
				Out.RigidAngularAcceleration = FVector(
					Inv[0] * Moment.X + Inv[1] * Moment.Y + Inv[2] * Moment.Z,
					Inv[3] * Moment.X + Inv[4] * Moment.Y + Inv[5] * Moment.Z,
					Inv[6] * Moment.X + Inv[7] * Moment.Y + Inv[8] * Moment.Z);
			}
			else
			{
				Out.bInertiaSingular = true;
			}
			// The field at the mass centre is g - a0; its vertical part in g,
			// downward positive.
			Out.LoadFactorAtCg = -(-Request.LoadFactor * kG0 - Out.RigidAcceleration.Z) / kG0;
		}
		return true;
	}
}
