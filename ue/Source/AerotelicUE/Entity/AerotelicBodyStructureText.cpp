#include "AerotelicBodyStructureText.h"

#include "AerotelicEntityActor.h"

namespace
{
	FString Len(double V, const aerotelic::UnitSystem& Units)
	{
		return FString::Printf(TEXT("%.4g %s"), V, UTF8_TO_TCHAR(Units.LengthName()));
	}

	FString MatName(const aerotelic::Material& M)
	{
		return M.HasDensity() ? FString(UTF8_TO_TCHAR(M.name.c_str())) : FString(TEXT("no material"));
	}
}

FString DescribeAerotelicBodyStructure(const aerotelic::BodyStructure& S, const aerotelic::UnitSystem& Units)
{
	if (!S.IsSet() && !S.HasInterior()) return TEXT("not set");
	TArray<FString> Parts;
	if (S.asserted_mass)
	{
		Parts.Add(FString::Printf(TEXT("asserted %.4g %s"), *S.asserted_mass, UTF8_TO_TCHAR(Units.MassName())));
	}
	if (S.material.HasDensity())
	{
		Parts.Add(S.skin_thickness > 0.0
			? FString::Printf(TEXT("%s skin %s"), *MatName(S.material), *Len(S.skin_thickness, Units))
			: FString::Printf(TEXT("%s, solid"), *MatName(S.material)));
	}
	else if (S.skin_thickness > 0.0)
	{
		Parts.Add(FString::Printf(TEXT("skin %s, no material"), *Len(S.skin_thickness, Units)));
	}
	for (const aerotelic::PlaneNetwork& N : S.networks)
	{
		FString Axis = N.axis == aerotelic::PlaneAxis::Custom
			? FString::Printf(TEXT("normal (%.2f, %.2f, %.2f)"), N.normal.x, N.normal.y, N.normal.z)
			: FString::Printf(TEXT("normal %s"), UTF8_TO_TCHAR(aerotelic::PlaneAxisName(N.axis)));
		FString Line = FString::Printf(TEXT("%s: %d plane%s %s, %s"), UTF8_TO_TCHAR(N.name.c_str()), N.count, N.count == 1 ? TEXT("") : TEXT("s"), *Axis, *Len(N.thickness, Units));
		if (N.height > 0.0) Line += FString::Printf(TEXT(", %s deep"), *Len(N.height, Units));
		Line += FString::Printf(TEXT(" (%s)"), *MatName(N.material));
		Parts.Add(Line);
	}
	if (!S.exclusions.empty())
	{
		TArray<FString> Ex;
		for (const std::string& E : S.exclusions) Ex.Add(UTF8_TO_TCHAR(E.c_str()));
		Parts.Add(TEXT("avoiding ") + FString::Join(Ex, TEXT(", ")));
	}
	return FString::Join(Parts, TEXT("; "));
}

FString DescribeAerotelicEntityStructures(const AAerotelicEntityActor* Entity, const aerotelic::UnitSystem& Units)
{
	if (Entity == nullptr) return FString();
	TArray<FString> Lines;
	for (const FAerotelicBody& B : Entity->GetBodies())
	{
		Lines.Add(FString::Printf(TEXT("%s: %s"), *B.Name, *DescribeAerotelicBodyStructure(B.Structure, Units)));
	}
	return FString::Join(Lines, TEXT("\n"));
}
