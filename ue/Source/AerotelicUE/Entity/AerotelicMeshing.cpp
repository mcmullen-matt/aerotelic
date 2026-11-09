#include "AerotelicMeshing.h"

#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Trsf.hxx>

namespace AerotelicMeshing
{
	std::vector<aerotelic::NamedMesh> MeshesFromShape(const TopoDS_Shape& Shape, double LinearDeflection, double AngularDeflectionDeg)
	{
		std::vector<aerotelic::NamedMesh> Out;
		if (Shape.IsNull())
		{
			return Out;
		}
		BRepMesh_IncrementalMesh Mesher(Shape, FMath::Max(LinearDeflection, 1e-6), false, FMath::DegreesToRadians(AngularDeflectionDeg), true);
		if (!Mesher.IsDone())
		{
			return Out;
		}
		TArray<TopoDS_Shape> Solids;
		if (Shape.ShapeType() == TopAbs_COMPOUND)
		{
			for (TopoDS_Iterator It(Shape); It.More(); It.Next())
			{
				if (!It.Value().IsNull())
				{
					Solids.Add(It.Value());
				}
			}
		}
		if (Solids.Num() == 0)
		{
			Solids.Add(Shape);
		}
		for (const TopoDS_Shape& Solid : Solids)
		{
			aerotelic::NamedMesh Body;
			aerotelic::TriMesh& M = Body.mesh;
			for (TopExp_Explorer FaceExp(Solid, TopAbs_FACE); FaceExp.More(); FaceExp.Next())
			{
				const TopoDS_Face& Face = TopoDS::Face(FaceExp.Current());
				TopLoc_Location Loc;
				Handle(Poly_Triangulation) Tri = BRep_Tool::Triangulation(Face, Loc);
				if (Tri.IsNull())
				{
					continue;
				}
				const gp_Trsf& Trsf = Loc.Transformation();
				const bool bReversed = Face.Orientation() == TopAbs_REVERSED;
				const int32 Base = static_cast<int32>(M.vertices.size());
				for (int32 i = 1; i <= Tri->NbNodes(); ++i)
				{
					gp_Pnt P = Tri->Node(i);
					P.Transform(Trsf);
					M.vertices.push_back({P.X(), P.Y(), P.Z()});
				}
				for (int32 i = 1; i <= Tri->NbTriangles(); ++i)
				{
					int32 n1, n2, n3;
					Tri->Triangle(i).Get(n1, n2, n3);
					if (bReversed)
					{
						Swap(n2, n3);
					}
					M.triangles.push_back({Base + n1 - 1, Base + n2 - 1, Base + n3 - 1});
				}
			}
			if (M.triangles.empty())
			{
				continue;
			}
			// Faces carry their own copies of shared vertices; weld them so the
			// mesh is closed for the distance field, and face it outward.
			M.WeldVertices(FMath::Max(LinearDeflection * 1e-3, 1e-9));
			M.FlipIfInsideOut();
			Out.push_back(std::move(Body));
		}
		return Out;
	}

	void PlaceMesh(aerotelic::TriMesh& Mesh, const FTransform& Transform)
	{
		// With the scale: an entity actor's transform carries the project's
		// model scale (Model > Scale) as a uniform scale, and a placed
		// mesh is the replica's.
		const FMatrix M = Transform.ToMatrixWithScale();
		// FMatrix is row-vector convention (v' = v * M): M.M[row][col].
		const std::array<double, 9> Linear = {
			M.M[0][0], M.M[1][0], M.M[2][0],
			M.M[0][1], M.M[1][1], M.M[2][1],
			M.M[0][2], M.M[1][2], M.M[2][2],
		};
		const FVector T = Transform.GetTranslation();
		Mesh.Transform(Linear, aerotelic::Vec3{T.X, T.Y, T.Z});
	}

	void MirrorY(aerotelic::TriMesh& Mesh)
	{
		Mesh.Mirror(aerotelic::Vec3{0.0, 0.0, 0.0}, aerotelic::Vec3{0.0, 1.0, 0.0});
	}

	void ToProceduralMesh(const aerotelic::TriMesh& Mesh, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles, TArray<FVector>& OutNormals)
	{
		OutVertices.Reset(static_cast<int32>(Mesh.vertices.size()));
		for (const aerotelic::Vec3& V : Mesh.vertices)
		{
			OutVertices.Add(FVector(V.x, V.y, V.z));
		}
		OutTriangles.Reset(static_cast<int32>(Mesh.triangles.size()) * 6);
		OutNormals.SetNumZeroed(OutVertices.Num());
		for (const auto& T : Mesh.triangles)
		{
			// Both windings, as the OCCT path does (see AerotelicEntityActor's
			// RefreshTessellatedMesh); normals from the first.
			OutTriangles.Add(T[0]); OutTriangles.Add(T[1]); OutTriangles.Add(T[2]);
			OutTriangles.Add(T[0]); OutTriangles.Add(T[2]); OutTriangles.Add(T[1]);
			const FVector& A = OutVertices[T[0]];
			const FVector& B = OutVertices[T[1]];
			const FVector& C = OutVertices[T[2]];
			const FVector N = FVector::CrossProduct(B - A, C - A);
			OutNormals[T[0]] += N;
			OutNormals[T[1]] += N;
			OutNormals[T[2]] += N;
		}
		for (FVector& N : OutNormals)
		{
			N = N.GetSafeNormal();
		}
	}

	void ToTwoSidedProceduralMesh(const aerotelic::TriMesh& Mesh, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles, TArray<FVector>& OutNormals)
	{
		const int32 Nv = static_cast<int32>(Mesh.vertices.size());
		OutVertices.Reset(2 * Nv);
		for (const aerotelic::Vec3& V : Mesh.vertices)
		{
			OutVertices.Add(FVector(V.x, V.y, V.z));
		}
		OutTriangles.Reset(static_cast<int32>(Mesh.triangles.size()) * 6);
		OutNormals.SetNumZeroed(Nv);
		// The front: the triangles as they are, smooth normals from them.
		for (const auto& T : Mesh.triangles)
		{
			OutTriangles.Add(T[0]); OutTriangles.Add(T[1]); OutTriangles.Add(T[2]);
			const FVector& A = OutVertices[T[0]];
			const FVector& B = OutVertices[T[1]];
			const FVector& C = OutVertices[T[2]];
			const FVector N = FVector::CrossProduct(B - A, C - A);
			OutNormals[T[0]] += N;
			OutNormals[T[1]] += N;
			OutNormals[T[2]] += N;
		}
		for (FVector& N : OutNormals)
		{
			N = N.GetSafeNormal();
		}
		// The back: the vertices again with the normals flipped, the
		// triangles reversed over them. Each value is copied out before it is
		// appended: TArray::Add refuses an element that lives in the array
		// being grown (it may reallocate under the reference), and that
		// assertion is what crashed Generate on the P-38 wing on 2026-11-08.
		OutVertices.Reserve(2 * Nv);
		OutNormals.Reserve(2 * Nv);
		for (int32 i = 0; i < Nv; ++i)
		{
			const FVector V = OutVertices[i];
			const FVector N = -OutNormals[i];
			OutVertices.Add(V);
			OutNormals.Add(N);
		}
		for (const auto& T : Mesh.triangles)
		{
			OutTriangles.Add(T[0] + Nv); OutTriangles.Add(T[2] + Nv); OutTriangles.Add(T[1] + Nv);
		}
	}

	FBox BoundsOf(const aerotelic::TriMesh& Mesh)
	{
		FBox Box(ForceInit);
		for (const aerotelic::Vec3& V : Mesh.vertices)
		{
			Box += FVector(V.x, V.y, V.z);
		}
		return Box;
	}
}

#include <algorithm>
#include <map>
#include <tuple>

std::vector<aerotelic::TriMesh> AerotelicMeshing::SplitShells(const aerotelic::TriMesh& Mesh, double WeldTolerance)
{
	std::vector<aerotelic::TriMesh> Out;
	const int N = static_cast<int>(Mesh.vertices.size());
	if (N == 0 || Mesh.triangles.empty())
	{
		return Out;
	}
	// A representative per position (quantised), so doubled seam vertices
	// join their shell; then union-find over the triangles.
	std::vector<int> Rep(N);
	{
		std::map<std::tuple<long long, long long, long long>, int> ByCell;
		const double Q = WeldTolerance > 0.0 ? 1.0 / WeldTolerance : 1e9;
		for (int i = 0; i < N; ++i)
		{
			const aerotelic::Vec3& P = Mesh.vertices[i];
			const auto Key = std::make_tuple(static_cast<long long>(std::floor(P.x * Q)), static_cast<long long>(std::floor(P.y * Q)), static_cast<long long>(std::floor(P.z * Q)));
			auto It = ByCell.find(Key);
			if (It == ByCell.end()) { ByCell.emplace(Key, i); Rep[i] = i; }
			else Rep[i] = It->second;
		}
	}
	std::vector<int> Parent(N);
	for (int i = 0; i < N; ++i) Parent[i] = Rep[i];
	auto Find = [&Parent](int a) { while (Parent[a] != a) { Parent[a] = Parent[Parent[a]]; a = Parent[a]; } return a; };
	auto Unite = [&Parent, &Find](int a, int b) { a = Find(a); b = Find(b); if (a != b) Parent[a] = b; };
	for (const auto& T : Mesh.triangles) { Unite(T[0], T[1]); Unite(T[0], T[2]); }
	std::map<int, int> ShellOfRoot;
	std::vector<int> ShellOfTriangle(Mesh.triangles.size());
	for (size_t t = 0; t < Mesh.triangles.size(); ++t)
	{
		const int Root = Find(Mesh.triangles[t][0]);
		auto It = ShellOfRoot.find(Root);
		if (It == ShellOfRoot.end()) It = ShellOfRoot.emplace(Root, static_cast<int>(ShellOfRoot.size())).first;
		ShellOfTriangle[t] = It->second;
	}
	Out.resize(ShellOfRoot.size());
	// One remap table, stamped per shell, so a result of many shells costs
	// no more than one table of the mesh's vertices.
	std::vector<int> Remap(N, -1), Stamp(N, -1);
	std::vector<std::vector<size_t>> TrianglesOfShell(ShellOfRoot.size());
	for (size_t t = 0; t < Mesh.triangles.size(); ++t) TrianglesOfShell[ShellOfTriangle[t]].push_back(t);
	for (size_t S = 0; S < Out.size(); ++S)
	{
		aerotelic::TriMesh& Shell = Out[S];
		for (size_t t : TrianglesOfShell[S])
		{
			std::array<int, 3> Tri;
			for (int k = 0; k < 3; ++k)
			{
				const int V = Mesh.triangles[t][k];
				if (Stamp[V] != static_cast<int>(S)) { Stamp[V] = static_cast<int>(S); Remap[V] = static_cast<int>(Shell.vertices.size()); Shell.vertices.push_back(Mesh.vertices[V]); }
				Tri[k] = Remap[V];
			}
			Shell.triangles.push_back(Tri);
		}
	}
	std::stable_sort(Out.begin(), Out.end(), [](const aerotelic::TriMesh& A, const aerotelic::TriMesh& B) { return std::abs(A.SignedVolume()) > std::abs(B.SignedVolume()); });
	return Out;
}