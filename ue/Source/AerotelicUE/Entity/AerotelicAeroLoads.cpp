#include "AerotelicAeroLoads.h"

aerotelic::Vec3 FAerotelicAeroLoadCase::ForceCoefficient() const
{
	aerotelic::Vec3 F{ 0, 0, 0 };
	for (const aerotelic::aero::PressureSample& S : Samples)
	{
		const double CpNet = S.two_sided ? (S.cp_lower - S.cp_upper) : -S.cp_upper;
		F += S.normal * (CpNet * S.area);
	}
	return ReferenceArea > 0.0 ? F * (1.0 / ReferenceArea) : F;
}

namespace AerotelicAeroLoads
{
	void MapToFaces(const FAerotelicAeroLoadCase& Case, const std::vector<aerotelic::Vec3>& Nodes, const TArray<AerotelicStructuralFea::FShellFace>& Faces,
		TFunctionRef<bool(int32 RegionId)> Covered, double UniformPressure, double LengthToSI, FMapResult& Out)
	{
		Out = FMapResult();
		const int32 NFaces = Faces.Num();
		Out.Pressure.Init(UniformPressure, NFaces);
		Out.Mapped.Init(0, NFaces);
		// The covered faces' centroids and outward normals.
		std::vector<aerotelic::Vec3> Centres, Normals;
		TArray<int32> Which;
		TArray<double> Areas;
		for (int32 i = 0; i < NFaces; ++i)
		{
			const AerotelicStructuralFea::FShellFace& F = Faces[i];
			if (!Covered(F.Region)) continue;
			const aerotelic::Vec3& A = Nodes[F.N[0]];
			const aerotelic::Vec3& B = Nodes[F.N[1]];
			const aerotelic::Vec3& C = Nodes[F.N[2]];
			const aerotelic::Vec3 Cr = aerotelic::Cross(B - A, C - A);
			const double Twice = aerotelic::Length(Cr);
			Centres.push_back((A + B + C) * (1.0 / 3.0));
			Normals.push_back(Twice > 0.0 ? Cr * (1.0 / Twice) : aerotelic::Vec3{ 0, 0, 1 });
			Areas.Add(0.5 * Twice);
			Which.Add(i);
		}
		Out.FacesCovered = Which.Num();
		if (Which.Num() == 0) return;
		std::vector<unsigned char> Mapped;
		const std::vector<double> Cp = aerotelic::aero::MapPressures(Case.Samples, Centres, Normals, {}, &Out.Stats, &Mapped);
		const double L2 = LengthToSI * LengthToSI;
		for (int32 k = 0; k < Which.Num(); ++k)
		{
			if (!Mapped[k]) continue;
			const int32 i = Which[k];
			const double P = Case.DynamicPressure * Cp[k];
			Out.Pressure[i] += P;
			Out.Mapped[i] = 1;
			const aerotelic::Vec3& N = Normals[k];
			Out.Resultant -= FVector(N.x, N.y, N.z) * (P * Areas[k] * L2);
		}
	}
}
