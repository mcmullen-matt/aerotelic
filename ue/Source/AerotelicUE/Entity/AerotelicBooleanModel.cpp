#include "AerotelicBooleanModel.h"
#include "AerotelicEntityActor.h"
#include "AerotelicMeshing.h"

#include <stdexcept>
#include <string>

#include "aerotelic/mesh_boolean.hpp"

namespace
{
	// A mesh with, per triangle, the operand body whose surface it is a
	// piece of (an index into Sources).
	struct FLabelled
	{
		aerotelic::TriMesh Mesh;
		std::vector<int> Labels;
	};

	// The index of a source in the table, added if new.
	int32 SourceIndex(std::vector<aerotelic::MeshFaceSource>& Sources, const aerotelic::MeshFaceSource& S)
	{
		for (size_t i = 0; i < Sources.size(); ++i) if (Sources[i] == S) return static_cast<int32>(i);
		Sources.push_back(S);
		return static_cast<int32>(Sources.size() - 1);
	}

	// One operand body, labelled: a parameterised body is one source (the
	// entity, the body, mirrored or not); an earlier boolean result's body
	// brings its own sources along, so a boolean over a boolean still names
	// the parameterised bodies every face came from.
	FLabelled Labelled(const aerotelic::NamedMesh& M, const FAerotelicBooleanOperand& Operand, std::vector<aerotelic::MeshFaceSource>& Sources)
	{
		FLabelled L;
		L.Mesh = M.mesh;
		if (M.HasFaceSources())
		{
			std::vector<int> Map(M.sources.size(), 0);
			for (size_t s = 0; s < M.sources.size(); ++s)
			{
				aerotelic::MeshFaceSource S = M.sources[s];
				// The mirror of a result made of mirrored pieces is the unmirrored
				// ones, and the other way about.
				if (Operand.bMirror) S.mirror = !S.mirror;
				Map[s] = SourceIndex(Sources, S);
			}
			L.Labels.reserve(M.face_source.size());
			for (int f : M.face_source) L.Labels.push_back(f >= 0 && f < static_cast<int>(Map.size()) ? Map[f] : 0);
		}
		else
		{
			aerotelic::MeshFaceSource S;
			S.entity = TCHAR_TO_UTF8(*Operand.Entity->EntityName);
			S.body = M.name;
			S.mirror = Operand.bMirror;
			L.Labels.assign(M.mesh.triangles.size(), SourceIndex(Sources, S));
		}
		return L;
	}

	FLabelled Boolean(const FLabelled& A, const FLabelled& B, aerotelic::MeshBooleanOp Op)
	{
		aerotelic::MeshBooleanLabels Labels;
		Labels.a = &A.Labels;
		Labels.b = &B.Labels;
		FLabelled R;
		R.Mesh = aerotelic::MeshBoolean(A.Mesh, B.Mesh, Op, nullptr, {}, &Labels);
		R.Labels = std::move(Labels.out);
		return R;
	}

	// The operands folded left to right through the mesh boolean: (A op B)
	// op C, as feature_tree.py's fold_boolean built its chain. An operand
	// with several bodies (a propeller: spinner and blades) is first fused
	// into one. Errors carry the operand names. Every triangle of the result
	// is labelled with the operand body it came from (Sources), exactly --
	// the mesh boolean cuts triangles but never makes one from two bodies.
	FLabelled FoldMeshes(const TArray<FAerotelicBooleanOperand>& Operands, const std::string& Operation, double Tolerance, std::vector<aerotelic::MeshFaceSource>& Sources)
	{
		const aerotelic::MeshBooleanOp Op = aerotelic::ParseMeshBooleanOp(Operation);
		FLabelled Result;
		FString ResultLabel;
		for (int32 i = 0; i < Operands.Num(); ++i)
		{
			const FAerotelicBooleanOperand& Operand = Operands[i];
			if (!Operand.Entity.IsValid())
			{
				throw std::runtime_error("An operand of this boolean operation no longer exists.");
			}
			const FString Label = Operand.Entity->EntityName + (Operand.bMirror ? TEXT(" (mirror)") : TEXT(""));
			std::vector<aerotelic::NamedMesh> Bodies = Operand.Entity->GetPlacedMeshes(Operand.bMirror, Tolerance);
			FLabelled Body;
			for (const aerotelic::NamedMesh& M : Bodies)
			{
				if (M.mesh.IsEmpty()) continue;
				if (!M.mesh.IsClosedManifold() || !(M.mesh.SignedVolume() > 0.0))
				{
					throw std::runtime_error(std::string("Operand '") + TCHAR_TO_UTF8(*Label) + "' / " + M.name + " is not a closed, outward surface.");
				}
				FLabelled Piece = Labelled(M, Operand, Sources);
				if (Body.Mesh.IsEmpty())
				{
					Body = std::move(Piece);
					continue;
				}
				try
				{
					Body = Boolean(Body, Piece, aerotelic::MeshBooleanOp::Fuse);
				}
				catch (const std::exception& e)
				{
					throw std::runtime_error(std::string("Fusing the bodies of '") + TCHAR_TO_UTF8(*Label) + "': " + e.what());
				}
			}
			if (Body.Mesh.IsEmpty())
			{
				throw std::runtime_error(std::string("Operand '") + TCHAR_TO_UTF8(*Label) + "' has no geometry.");
			}
			if (i == 0)
			{
				Result = std::move(Body);
				ResultLabel = Label;
				continue;
			}
			try
			{
				Result = Boolean(Result, Body, Op);
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error(std::string(aerotelic::MeshBooleanOpName(Op)) + " of '" + TCHAR_TO_UTF8(*ResultLabel) + "' with '" + TCHAR_TO_UTF8(*Label) + "': " + e.what());
			}
			ResultLabel = Label;
			if (Result.Mesh.IsEmpty())
			{
				throw std::runtime_error(std::string("The ") + aerotelic::MeshBooleanOpName(Op) + " of '" + TCHAR_TO_UTF8(*ResultLabel) + "' left nothing.");
			}
		}
		return Result;
	}

	// A body of the result with its face sources: only the sources it uses,
	// renumbered.
	aerotelic::NamedMesh WithSources(const std::string& Name, aerotelic::TriMesh&& Mesh, const std::vector<int>& Labels, const std::vector<aerotelic::MeshFaceSource>& Sources)
	{
		aerotelic::NamedMesh Out;
		Out.name = Name;
		Out.mesh = std::move(Mesh);
		if (Labels.size() != Out.mesh.triangles.size()) return Out;
		std::vector<int> Map(Sources.size(), -1);
		Out.face_source.reserve(Labels.size());
		for (int L : Labels)
		{
			if (L < 0 || L >= static_cast<int>(Sources.size())) { Out.face_source.clear(); Out.sources.clear(); return Out; }
			if (Map[L] < 0) { Map[L] = static_cast<int>(Out.sources.size()); Out.sources.push_back(Sources[L]); }
			Out.face_source.push_back(Map[L]);
		}
		return Out;
	}
}

TopoDS_Shape FAerotelicBooleanModel::BuildSolid() const
{
	throw std::runtime_error("A boolean result is a triangle mesh, not a B-rep solid: export it as STL, or export its operands to STEP and combine them downstream.");
}

std::vector<aerotelic::NamedMesh> FAerotelicBooleanModel::BuildMeshes(double Tolerance) const
{
	if (Operands.Num() < 2)
	{
		throw std::runtime_error("A boolean operation needs at least two operands.");
	}
	// The result as the solid bodies it really is: a shell that touches
	// nothing else -- a wing's flap, parted from the wing by its gap, fused
	// with the fuselage alongside it -- is a body of its own, not part of
	// the airframe's. One shell keeps the name "Main"; several are "Body 1"
	// (the largest) onwards. Each carries its face sources: which operand
	// body every triangle is a piece of, so the skin it inherits is known.
	std::vector<aerotelic::MeshFaceSource> Sources;
	FLabelled Folded = FoldMeshes(Operands, Operation, Tolerance, Sources);
	std::vector<std::vector<int>> ShellLabels;
	std::vector<aerotelic::TriMesh> Shells = AerotelicMeshing::SplitShells(Folded.Mesh, 1e-6, &Folded.Labels, &ShellLabels);
	std::vector<aerotelic::NamedMesh> Out;
	if (Shells.size() <= 1)
	{
		if (Shells.empty()) Out.push_back({"Main", aerotelic::TriMesh()});
		else Out.push_back(WithSources("Main", std::move(Shells[0]), ShellLabels.size() == 1 ? ShellLabels[0] : std::vector<int>(), Sources));
		return Out;
	}
	for (size_t i = 0; i < Shells.size(); ++i)
	{
		Out.push_back(WithSources("Body " + std::to_string(i + 1), std::move(Shells[i]), i < ShellLabels.size() ? ShellLabels[i] : std::vector<int>(), Sources));
	}
	return Out;
}

bool FAerotelicBooleanModel::DependsOn(const AAerotelicEntityActor* Entity) const
{
	for (const FAerotelicBooleanOperand& Op : Operands)
	{
		if (!Op.Entity.IsValid())
		{
			continue;
		}
		if (Op.Entity.Get() == Entity)
		{
			return true;
		}
		if (Op.Entity->GetEntityType() == aerotelic::project_yaml::EntityType::Boolean)
		{
			if (const FAerotelicBooleanModel* Inner = static_cast<const FAerotelicBooleanModel*>(Op.Entity->GetModel()))
			{
				if (Inner->DependsOn(Entity))
				{
					return true;
				}
			}
		}
	}
	return false;
}
