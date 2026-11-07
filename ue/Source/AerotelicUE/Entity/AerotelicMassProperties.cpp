#include "AerotelicMassProperties.h"

#include "AerotelicEntityActor.h"

using aerotelic::InertiaTensor;
using aerotelic::MassMode;
using aerotelic::MassProperties;
using aerotelic::Vec3;

namespace
{
	// The table: one row per component, the columns the user reads an
	// inertia summary by. Widths are fixed so the monospace text lines up.
	// (The widths below are the literals in the format strings: 30 and 12.)
	constexpr int32 kLabelWidth = 30;
	constexpr int32 kCellWidth = 12;

	FString Cell(double V)
	{
		return FString::Printf(TEXT("%12.5g"), V);
	}

	FString Dash()
	{
		return FString::Printf(TEXT("%12s"), TEXT("-"));
	}

	FString Label(const FString& Text)
	{
		// Clipped rather than overflowing, so a long entity name cannot push
		// its numbers out of their columns.
		const FString Clipped = Text.Len() > kLabelWidth ? Text.Left(kLabelWidth - 1) + TEXT("~") : Text;
		return FString::Printf(TEXT("%-30s"), *Clipped);
	}

	// Two header lines: the quantity, then its unit in parentheses.
	FString HeaderRows(const aerotelic::UnitSystem& Units)
	{
		FString Row = Label(TEXT("Component"));
		for (const TCHAR* Column : { TEXT("Mass"), TEXT("CG x"), TEXT("CG y"), TEXT("CG z"),
			TEXT("Ixx"), TEXT("Iyy"), TEXT("Izz"), TEXT("Ixy"), TEXT("Ixz"), TEXT("Iyz") })
		{
			Row += FString::Printf(TEXT("%12s"), Column);
		}
		Row += TEXT("   Note\n");
		const FString M = FString::Printf(TEXT("(%s)"), UTF8_TO_TCHAR(Units.MassName()));
		const FString L = FString::Printf(TEXT("(%s)"), UTF8_TO_TCHAR(Units.LengthName()));
		const FString I = FString::Printf(TEXT("(%s)"), UTF8_TO_TCHAR(Units.InertiaName().c_str()));
		Row += Label(FString());
		Row += FString::Printf(TEXT("%12s%12s%12s%12s"), *M, *L, *L, *L);
		for (int32 i = 0; i < 6; ++i) Row += FString::Printf(TEXT("%12s"), *I);
		return Row + TEXT("\n");
	}

	// A weighed component: its mass, centre and the six independent entries
	// of the tensor about that centre. The tensor's off-diagonals carry
	// -Ixy etc., so the products an engineer reads are their negatives.
	FString ValueRow(const FString& Name, const MassProperties& P, const FString& Note)
	{
		const InertiaTensor& I = P.inertia;
		return Label(Name)
			+ Cell(P.mass)
			+ Cell(P.centre.x) + Cell(P.centre.y) + Cell(P.centre.z)
			+ Cell(I[0]) + Cell(I[4]) + Cell(I[8])
			+ Cell(-I[1]) + Cell(-I[2]) + Cell(-I[5])
			+ (Note.IsEmpty() ? FString() : TEXT("   ") + Note) + TEXT("\n");
	}

	// A component that could not be weighed keeps its row, so the table
	// lists everything that was chosen, with the reason in place of numbers.
	FString EmptyRow(const FString& Name, const FString& Note)
	{
		FString Row = Label(Name);
		for (int32 i = 0; i < 10; ++i) Row += Dash();
		return Row + TEXT("   left out: ") + Note + TEXT("\n");
	}

	FString Rule(int32 Length)
	{
		return FString::ChrN(Length, TEXT('-')) + TEXT("\n");
	}
}

FAerotelicMassReport ComputeAerotelicMassReport(const TArray<FAerotelicBooleanOperand>& Bodies, double Tolerance,
	const aerotelic::UnitSystem& Units)
{
	FAerotelicMassReport Report;
	Report.Tolerance = Tolerance;
	std::vector<MassProperties> Included;

	for (const FAerotelicBooleanOperand& Op : Bodies)
	{
		AAerotelicEntityActor* Entity = Op.Entity.Get();
		if (Entity == nullptr)
		{
			FAerotelicMassLine Gone;
			Gone.Label = TEXT("(deleted entity)");
			Gone.Note = TEXT("no longer in the scene");
			Report.Bodies.Add(Gone);
			continue;
		}
		const FString Suffix = Op.bMirror ? TEXT(" (mirror)") : TEXT("");
		// A derived entity inherits (the first operand's / first filled
		// body's). A lattice's two bodies -- the hollowed shell and the
		// struts -- are real solids, so its inherited skin thickness does
		// not apply to them: they weigh by density over their volume.
		aerotelic::MassSpec Spec = Entity->GetEffectiveMassSpec();
		const AAerotelicEntityActor* Source = Entity->GetMassSpecSource();
		const FString Inherited = Source != nullptr && Source != Entity ? FString::Printf(TEXT("inherited from %s"), *Source->EntityName) : FString();
		if (Entity->GetEntityType() == aerotelic::project_yaml::EntityType::Lattice)
		{
			Spec.skin_thickness = 0.0;
		}
		const std::vector<aerotelic::NamedMesh> Meshes = Entity->GetPlacedMeshes(Op.bMirror, Tolerance);
		if (Meshes.empty())
		{
			FAerotelicMassLine None;
			None.Label = Entity->EntityName + Suffix;
			None.Mode = Spec.mode;
			None.Note = TEXT("no geometry could be built");
			Report.Bodies.Add(None);
			continue;
		}

		// An asserted mass is the whole entity's; its bodies share it by
		// area, the same uniform shell the integrals assume. The mirrored
		// half is a second part and carries the whole mass again.
		double TotalArea = 0.0;
		if (Spec.mode == MassMode::Asserted)
		{
			for (const aerotelic::NamedMesh& M : Meshes) TotalArea += M.mesh.SurfaceArea();
		}

		for (const aerotelic::NamedMesh& M : Meshes)
		{
			FAerotelicMassLine Line;
			Line.Label = Entity->EntityName
				+ (Meshes.size() > 1 ? TEXT(" / ") + FString(UTF8_TO_TCHAR(M.name.c_str())) : FString())
				+ Suffix;
			Line.Mode = Spec.mode;
			switch (Spec.mode)
			{
			case MassMode::Unset:
				Line.Note = Inherited.IsEmpty() ? TEXT("no material assigned (Scene tree > Material Properties)")
					: FString::Printf(TEXT("no material assigned on %s, which it inherits from (Scene tree > Material Properties)"), *Source->EntityName);
				break;
			case MassMode::Material:
				if (!Spec.material.HasDensity())
				{
					Line.Note = TEXT("the material has no density (Scene tree > Material Properties)");
					break;
				}
				// The material's density is SI; the mesh is in project units.
				if (Spec.IsSkin())
				{
					// A skin of the material: every body of the entity is a shell
					// of the thickness, rho t A; open or closed alike.
					Line.Props = aerotelic::SkinMassProperties(M.mesh, Spec.material.density / Units.DensityToSI(), Spec.skin_thickness);
					Line.bIncluded = true;
					Line.Note = FString::Printf(TEXT("skin %.4g %s thick"), Spec.skin_thickness, UTF8_TO_TCHAR(Units.LengthName()));
					break;
				}
				Line.Props = aerotelic::SolidMassProperties(M.mesh, Spec.material.density / Units.DensityToSI());
				Line.bIncluded = true;
				if (!M.mesh.IsClosedManifold())
				{
					Line.Note = TEXT("open surface: the enclosed volume, and so this mass, is not well defined -- give the material a skin thickness or assert the mass instead");
				}
				break;
			case MassMode::Asserted:
			{
				const double Area = M.mesh.SurfaceArea();
				const double Share = TotalArea > 0.0 ? Spec.mass * Area / TotalArea : 0.0;
				Line.Props = aerotelic::ShellMassProperties(M.mesh, Share);
				Line.bIncluded = true;
				break;
			}
			}
			if (Line.bIncluded)
			{
				Included.push_back(Line.Props);
				++Report.IncludedCount;
				if (!Inherited.IsEmpty()) Line.Note += (Line.Note.IsEmpty() ? TEXT("") : TEXT("; ")) + Inherited;
			}
			Report.Bodies.Add(Line);
		}
	}

	Report.Assembly = aerotelic::CombineMassProperties(Included);
	return Report;
}

FString FormatAerotelicMassReport(const FAerotelicMassReport& Report, const aerotelic::UnitSystem& Units)
{
	const int32 Width = kLabelWidth + 10 * kCellWidth;
	FString Out;
	Out += TEXT("MASS PROPERTIES\n");
	Out += FString::Printf(TEXT("Project units: %s (Model > Units). Mass in %s, lengths in %s, inertia in %s. %s\n"),
		UTF8_TO_TCHAR(Units.Describe().c_str()), UTF8_TO_TCHAR(Units.MassName()), UTF8_TO_TCHAR(Units.LengthName()),
		UTF8_TO_TCHAR(Units.InertiaName().c_str()),
		Report.Tolerance > 0.0
			? *FString::Printf(TEXT("Surfaces meshed at tolerance %.5g %s."), Report.Tolerance, UTF8_TO_TCHAR(Units.LengthName()))
			: TEXT("Each body weighed on its own grid (its Mesh tab's, the viewport's -- the grid the structural analysis solves on)."));
	Out += TEXT("Each component's inertia is about its own centre of mass, in the model's X, Y, Z axes; the assembly's is about the assembly's centre, every component carried there by the parallel-axis theorem.\n");
	Out += TEXT("Ixy, Ixz, Iyz are the products of inertia, the integrals of xy, xz, yz (the tensor's off-diagonals are their negatives).\n\n");

	Out += HeaderRows(Units);
	Out += Rule(Width);
	for (const FAerotelicMassLine& L : Report.Bodies)
	{
		Out += L.bIncluded ? ValueRow(L.Label, L.Props, L.Note) : EmptyRow(L.Label, L.Note);
	}
	Out += Rule(Width);
	if (Report.IncludedCount > 0)
	{
		Out += ValueRow(FString::Printf(TEXT("ASSEMBLY (%d %s)"), Report.IncludedCount, Report.IncludedCount == 1 ? TEXT("body") : TEXT("bodies")),
			Report.Assembly, FString());
	}
	else
	{
		Out += EmptyRow(TEXT("ASSEMBLY"), TEXT("no chosen component has mass properties assigned"));
	}
	return Out;
}
