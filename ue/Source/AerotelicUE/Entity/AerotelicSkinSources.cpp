#include "AerotelicSkinSources.h"

#include "AerotelicDocumentSubsystem.h"
#include "AerotelicEntityActor.h"

namespace
{
	FString Len(double V, const aerotelic::UnitSystem& Units)
	{
		return FString::Printf(TEXT("%.4g %s"), V, UTF8_TO_TCHAR(Units.LengthName()));
	}

	AAerotelicEntityActor* FindEntityNamed(const FString& Name)
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc == nullptr) return nullptr;
		for (const TWeakObjectPtr<AAerotelicEntityActor>& Weak : Doc->GetEntities())
		{
			if (Weak.IsValid() && Weak->EntityName == Name) return Weak.Get();
		}
		return nullptr;
	}

	double TriangleArea(const aerotelic::TriMesh& M, int T)
	{
		const auto& Tri = M.triangles[T];
		return 0.5 * aerotelic::Length(aerotelic::Cross(M.vertices[Tri[1]] - M.vertices[Tri[0]], M.vertices[Tri[2]] - M.vertices[Tri[0]]));
	}
}

aerotelic::TriMesh FAerotelicSkinSource::SubMesh(const aerotelic::TriMesh& Body) const
{
	aerotelic::TriMesh Out;
	std::vector<int> Map(Body.vertices.size(), -1);
	Out.triangles.reserve(Faces.size());
	for (int T : Faces)
	{
		if (T < 0 || T >= static_cast<int>(Body.triangles.size())) continue;
		std::array<int, 3> Tri;
		for (int k = 0; k < 3; ++k)
		{
			const int V = Body.triangles[T][k];
			if (Map[V] < 0) { Map[V] = static_cast<int>(Out.vertices.size()); Out.vertices.push_back(Body.vertices[V]); }
			Tri[k] = Map[V];
		}
		Out.triangles.push_back(Tri);
	}
	return Out;
}

TArray<FAerotelicSkinSource> AerotelicSkinSourcesOf(const aerotelic::NamedMesh& Mesh)
{
	TArray<FAerotelicSkinSource> Out;
	if (!Mesh.HasFaceSources()) return Out;
	Out.SetNum(static_cast<int32>(Mesh.sources.size()));
	for (int32 i = 0; i < Out.Num(); ++i)
	{
		FAerotelicSkinSource& S = Out[i];
		S.Source = Mesh.sources[i];
		const FString EntityName = UTF8_TO_TCHAR(S.Source.entity.c_str());
		const FString Body = UTF8_TO_TCHAR(S.Source.body.c_str());
		AAerotelicEntityActor* Entity = FindEntityNamed(EntityName);
		S.Entity = Entity;
		S.Label = EntityName + (S.Source.mirror ? TEXT(" (mirror)") : TEXT(""));
		if (Entity != nullptr)
		{
			if (Entity->GetBodies().Num() > 1) S.Label += TEXT(" / ") + Body;
			if (const aerotelic::BodyStructure* Own = Entity->GetBodyStructure(Body))
			{
				S.bKnown = true;
				S.Structure = *Own;
			}
			// The operand's whole surface, on its own grid (the mirror has the
			// same area as the body).
			for (const aerotelic::NamedMesh& M : Entity->GetPlacedMeshes(false))
			{
				if (FString(UTF8_TO_TCHAR(M.name.c_str())) == Body) S.OperandArea = M.mesh.SurfaceArea();
			}
		}
		else
		{
			S.Label += TEXT(" / ") + Body;
		}
	}
	for (size_t t = 0; t < Mesh.face_source.size(); ++t)
	{
		const int Src = Mesh.face_source[t];
		if (Src < 0 || Src >= Out.Num()) continue;
		Out[Src].Faces.push_back(static_cast<int>(t));
		Out[Src].Area += TriangleArea(Mesh.mesh, static_cast<int>(t));
	}
	return Out;
}

TArray<FAerotelicSkinSource> AerotelicSkinSourcesOf(const AAerotelicEntityActor* Boolean, const FString& Role)
{
	if (Boolean == nullptr || !Boolean->IsDerived()) return {};
	const int32 Index = Boolean->FindBodyIndex(Role);
	if (Index == INDEX_NONE) return {};
	const FString Name = Boolean->GetBodies()[Index].Name;
	const std::vector<aerotelic::NamedMesh> Placed = Boolean->GetPlacedMeshes(false);
	for (const aerotelic::NamedMesh& M : Placed)
	{
		if (FString(UTF8_TO_TCHAR(M.name.c_str())) == Name || Placed.size() == 1) return AerotelicSkinSourcesOf(M);
	}
	return {};
}

TArray<FString> AerotelicSkinSourceProblems(const TArray<FAerotelicSkinSource>& Sources, bool bStructural)
{
	TArray<FString> Problems;
	if (Sources.Num() == 0)
	{
		Problems.Add(TEXT("the body carries no record of which operand each face came from (rebuild it)"));
		return Problems;
	}
	for (const FAerotelicSkinSource& S : Sources)
	{
		if (!S.bKnown)
		{
			Problems.Add(FString::Printf(TEXT("%s is no longer in the scene"), *S.Label));
			continue;
		}
		const aerotelic::BodyStructure& B = S.Structure;
		if (bStructural)
		{
			if (!B.material.HasDensity())
			{
				Problems.Add(FString::Printf(TEXT("%s has NO STRUCTURAL MATERIAL -- set one under Structure... on %s in the Scene tree (a boolean result inherits its operands' skins; it has none of its own)"), *S.Label, *S.Label));
			}
			else if (!B.material.IsStructural())
			{
				Problems.Add(FString::Printf(TEXT("%s's material '%s' is not one the analysis can use (no modulus) -- set a structural material under Structure... on %s"), *S.Label, UTF8_TO_TCHAR(B.material.name.c_str()), *S.Label));
			}
			else if (!(B.skin_thickness > 0.0))
			{
				Problems.Add(FString::Printf(TEXT("%s has no skin thickness -- the analysis models shells only; give it one under Structure... on %s"), *S.Label, *S.Label));
			}
		}
		else if (!B.IsSet())
		{
			Problems.Add(FString::Printf(TEXT("%s has no material -- set one, with a skin thickness, under Structure... on %s in the Scene tree (a boolean result inherits its operands' skins; it has none of its own)"), *S.Label, *S.Label));
		}
		else if (!B.asserted_mass && !(B.skin_thickness > 0.0))
		{
			Problems.Add(FString::Printf(TEXT("%s is a solid (no skin thickness): a boolean result has only its skin to weigh -- give %s a skin thickness under Structure..."), *S.Label, *S.Label));
		}
	}
	return Problems;
}

FString DescribeAerotelicSkinSources(const TArray<FAerotelicSkinSource>& Sources, const aerotelic::UnitSystem& Units)
{
	if (Sources.Num() == 0) return TEXT("no skin: the body carries no record of its operands' faces");
	double Total = 0.0;
	for (const FAerotelicSkinSource& S : Sources) Total += S.Area;
	TArray<FString> Parts;
	for (const FAerotelicSkinSource& S : Sources)
	{
		FString What;
		if (!S.bKnown) What = TEXT("operand gone");
		else if (S.Structure.asserted_mass) What = FString::Printf(TEXT("asserted %.4g %s, %.0f%% of it here"), *S.Structure.asserted_mass, UTF8_TO_TCHAR(Units.MassName()), 100.0 * S.Share());
		else if (!S.Structure.material.HasDensity()) What = TEXT("no material");
		else if (!(S.Structure.skin_thickness > 0.0)) What = FString::Printf(TEXT("%s, solid (no skin)"), UTF8_TO_TCHAR(S.Structure.material.name.c_str()));
		else What = FString::Printf(TEXT("%s %s"), UTF8_TO_TCHAR(S.Structure.material.name.c_str()), *Len(S.Structure.skin_thickness, Units));
		Parts.Add(FString::Printf(TEXT("%s -- %s (%.0f%% of the surface)"), *S.Label, *What, Total > 0.0 ? 100.0 * S.Area / Total : 0.0));
	}
	return TEXT("skin inherited from the operands: ") + FString::Join(Parts, TEXT("; "));
}

FString DescribeAerotelicInheritedSkin(const AAerotelicEntityActor* Boolean, const FString& Role, const aerotelic::UnitSystem& Units)
{
	if (Boolean == nullptr || !Boolean->IsDerived()) return FString();
	const TArray<FAerotelicSkinSource> Sources = AerotelicSkinSourcesOf(Boolean, Role);
	if (Sources.Num() == 0) return TEXT("a boolean result inherits its operands' skins; this body is not built yet");
	return DescribeAerotelicSkinSources(Sources, Units);
}
