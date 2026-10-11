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
		const FMatrix M = Transform.ToMatrixNoScale();
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
