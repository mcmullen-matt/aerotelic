#include "AerotelicMassProperties.h"

#include "AerotelicBodyStructure.h"
#include "AerotelicEntityActor.h"
#include "AerotelicSkinSources.h"

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
	const aerotelic::UnitSystem& Units, const TArray<aerotelic::MassItem>* PointMasses)
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
		const std::vector<aerotelic::NamedMesh> Meshes = Entity->GetPlacedMeshes(Op.bMirror, Tolerance);
		if (Meshes.empty())
		{
			FAerotelicMassLine None;
			None.Label = Entity->EntityName + Suffix;
			None.Note = TEXT("no geometry could be built");
			Report.Bodies.Add(None);
			continue;
		}
		// The placement (StructureMassParts takes it; nothing in the estimate
		// depends on it since the plane networks went).
		std::array<double, 9> R;
		Vec3 T;
		double Scale = 1.0;
		Entity->GetPlacementMatrix(R, T, Scale);

		for (const aerotelic::NamedMesh& M : Meshes)
		{
			const FString Role = UTF8_TO_TCHAR(M.name.c_str());
			const FString BodyLabel = Entity->EntityName
				+ (Meshes.size() > 1 ? TEXT(" / ") + Role : FString())
				+ Suffix;
			// One generated layer as a line: a THIN SHELL of its own material
			// and thickness (density x area x thickness, the core's
			// SkinMassProperties) on the shell mesh given.
			auto WeighLayer = [&](const FAerotelicStructureLayer& L, const aerotelic::TriMesh* Shell, const TCHAR* Stale)
			{
				FAerotelicMassLine Line;
				Line.Label = BodyLabel + TEXT(" / ") + L.Name;
				Line.Mode = aerotelic::MassMode::Material;
				if (Shell == nullptr || Shell->IsEmpty())
				{
					Line.Note = L.Note.IsEmpty() ? TEXT("the layer came out empty") : L.Note;
				}
				else if (!L.Material.HasDensity())
				{
					Line.Note = FString::Printf(TEXT("no material on the %s"), FAerotelicStructureLayer::KindName(L.Kind));
				}
				else if (!(L.Thickness > 0.0))
				{
					Line.Note = FString::Printf(TEXT("no thickness on the %s"), FAerotelicStructureLayer::KindName(L.Kind));
				}
				else
				{
					Line.Props = aerotelic::SkinMassProperties(*Shell, L.Material.density / Units.DensityToSI(), L.Thickness);
					Line.bIncluded = true;
					Line.Note = FString::Printf(TEXT("%s %.4g %s thick, %s%s"),
						L.Kind == FAerotelicStructureLayer::EKind::Skin ? TEXT("skin") : TEXT("rib shells"),
						L.Thickness, UTF8_TO_TCHAR(Units.LengthName()), UTF8_TO_TCHAR(L.Material.name.c_str()), Stale);
				}
				if (Line.bIncluded)
				{
					Included.push_back(Line.Props);
					++Report.IncludedCount;
				}
				Report.Bodies.Add(Line);
			};
			const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(Role);
			const TCHAR* Stale = G != nullptr && G->IsStale() ? TEXT("; generated before the body's last change") : TEXT("");

			// A BOOLEAN RESULT's body (2026-11-20) owns no structure: each
			// piece of its surface keeps the skin of the operand body it was
			// cut from (the mesh's face sources), and is weighed by it -- one
			// line per operand piece, rho t A over that piece's triangles, or
			// its share of the operand's asserted mass. The layers inside are
			// the operands', taken over when its structure was generated.
			if (M.HasFaceSources())
			{
				const TArray<FAerotelicSkinSource> Sources = AerotelicSkinSourcesOf(M);
				for (const FAerotelicSkinSource& Src : Sources)
				{
					if (Src.Faces.empty()) continue;
					FAerotelicMassLine Line;
					Line.Label = BodyLabel + TEXT(" / skin of ") + Src.Label;
					Line.Mode = Src.bKnown ? aerotelic::MassModeOf(Src.Structure) : aerotelic::MassMode::Unset;
					const aerotelic::TriMesh Piece = Src.SubMesh(M.mesh);
					if (!Src.bKnown)
					{
						Line.Note = FString::Printf(TEXT("%s is no longer in the scene"), *Src.Label);
					}
					else if (Src.Structure.asserted_mass)
					{
						Line.Props = aerotelic::ShellMassProperties(Piece, *Src.Structure.asserted_mass * Src.Share());
						Line.bIncluded = true;
						Line.Note = FString::Printf(TEXT("%.0f%% of the %.4g %s asserted on %s (its share of that surface)"), 100.0 * Src.Share(), *Src.Structure.asserted_mass, UTF8_TO_TCHAR(Units.MassName()), *Src.Label);
					}
					else if (!Src.Structure.material.HasDensity())
					{
						Line.Note = FString::Printf(TEXT("no material on %s (Scene tree > Structure... on the operand; a boolean result inherits its operands' skins)"), *Src.Label);
					}
					else if (!(Src.Structure.skin_thickness > 0.0))
					{
						Line.Note = FString::Printf(TEXT("%s is a solid: a boolean result has only its skin to weigh -- give the operand a skin thickness"), *Src.Label);
					}
					else
					{
						Line.Props = aerotelic::SkinMassProperties(Piece, Src.Structure.material.density / Units.DensityToSI(), Src.Structure.skin_thickness);
						Line.bIncluded = true;
						Line.Note = FString::Printf(TEXT("skin %.4g %s thick, %s, inherited from %s"), Src.Structure.skin_thickness, UTF8_TO_TCHAR(Units.LengthName()), UTF8_TO_TCHAR(Src.Structure.material.name.c_str()), *Src.Label);
					}
					if (Line.bIncluded)
					{
						Included.push_back(Line.Props);
						++Report.IncludedCount;
					}
					Report.Bodies.Add(Line);
				}
				if (G != nullptr && G->IsGenerated())
				{
					const std::vector<aerotelic::NamedMesh> LayerMeshes = Entity->GetPlacedLayerMeshes(Role, Op.bMirror);
					for (const FAerotelicStructureLayer& L : G->GetLayers())
					{
						if (L.Kind == FAerotelicStructureLayer::EKind::Skin) continue;   // the pieces above ARE the skin
						const aerotelic::TriMesh* Shell = nullptr;
						for (const aerotelic::NamedMesh& LM : LayerMeshes) if (FString(UTF8_TO_TCHAR(LM.name.c_str())) == L.Name) Shell = &LM.mesh;
						WeighLayer(L, Shell, Stale);
					}
				}
				continue;
			}

			// Each other body weighs by ITS structure.
			aerotelic::BodyStructure S;
			if (const aerotelic::BodyStructure* Own = Entity->GetBodyStructure(Role); Own != nullptr && Own->IsSet())
			{
				S = *Own;
			}
			// A body whose structure has been GENERATED weighs by its layers,
			// each a THIN SHELL of its own material and thickness: the skin on
			// the body's own placed mesh -- the Skin layer has no mesh, the
			// body's surface is the skin -- and each Ribs layer on its placed
			// discs, in place of the estimates from the figures. An asserted
			// mass still stands in for all of it.
			if (G != nullptr && G->IsGenerated() && S.IsSkin() && !S.asserted_mass)
			{
				const std::vector<aerotelic::NamedMesh> LayerMeshes = Entity->GetPlacedLayerMeshes(Role, Op.bMirror);
				for (const FAerotelicStructureLayer& L : G->GetLayers())
				{
					const aerotelic::TriMesh* Shell = nullptr;
					if (L.Kind == FAerotelicStructureLayer::EKind::Skin)
					{
						Shell = &M.mesh;
					}
					else
					{
						for (const aerotelic::NamedMesh& LM : LayerMeshes) if (FString(UTF8_TO_TCHAR(LM.name.c_str())) == L.Name) Shell = &LM.mesh;
					}
					WeighLayer(L, Shell, Stale);
				}
				continue;
			}
			const std::vector<aerotelic::StructureMassPart> Parts = aerotelic::StructureMassParts(M.mesh, S, R, T, Scale, Op.bMirror, Units.DensityToSI());
			for (const aerotelic::StructureMassPart& Part : Parts)
			{
				FAerotelicMassLine Line;
				Line.Label = Parts.size() > 1 ? BodyLabel + TEXT(" / ") + FString(UTF8_TO_TCHAR(Part.label.c_str())) : BodyLabel;
				Line.Mode = aerotelic::MassModeOf(S);
				Line.Props = Part.props;
				Line.bIncluded = Part.included;
				Line.Note = UTF8_TO_TCHAR(Part.note.c_str());
				if (!S.IsSet())
				{
					Line.Note = TEXT("no material assigned (Scene tree > Structure...)");
				}
				else if (Part.label == "skin" && Part.included)
				{
					Line.Note = FString::Printf(TEXT("skin %.4g %s thick"), S.skin_thickness, UTF8_TO_TCHAR(Units.LengthName()));
				}
				if (Line.bIncluded)
				{
					Included.push_back(Line.Props);
					++Report.IncludedCount;
				}
				Report.Bodies.Add(Line);
			}
		}
	}

	// The point masses: each its own line, at its position scaled to the
	// replica (every placed mesh above is the replica's), with its own
	// tensor about that point -- the products as integral x y dm, so the
	// tensor's off-diagonals are their negatives.
	if (PointMasses != nullptr)
	{
		const double Scale = AAerotelicEntityActor::GetModelScale();
		// Only the point masses hung on the chosen bodies (2026-11-14): an
		// item hangs on an entity (or its mirror), chosen directly or as an
		// operand of a chosen boolean result (which inherits it).
		auto Hangs = [&Bodies](const aerotelic::MassItem& M)
		{
			if (M.attach_entity.empty()) return false;
			const FString Name = UTF8_TO_TCHAR(M.attach_entity.c_str());
			for (const FAerotelicBooleanOperand& Op : Bodies)
			{
				AAerotelicEntityActor* E = Op.Entity.Get();
				if (E == nullptr) continue;
				if (E->EntityName == Name && Op.bMirror == M.attach_mirror) return true;
				// The cast is unchecked: ask the type first.
				if (E->GetEntityType() != aerotelic::project_yaml::EntityType::Boolean) continue;
				if (const FAerotelicBooleanModel* B = E->GetModelAs<FAerotelicBooleanModel>())
				{
					for (const FAerotelicBooleanOperand& Inner : B->Operands)
					{
						if (Inner.Entity.IsValid() && Inner.Entity->EntityName == Name && Inner.bMirror == M.attach_mirror) return true;
					}
				}
			}
			return false;
		};
		for (const aerotelic::MassItem& M : *PointMasses)
		{
			if (!Hangs(M)) continue;
			++Report.PointMasses;
			FAerotelicMassLine Line;
			Line.Label = FString::Printf(TEXT("[point mass] %s"), UTF8_TO_TCHAR(M.name.c_str()));
			Line.Mode = aerotelic::MassMode::Asserted;
			const std::string Problem = aerotelic::MassItemProblem(M);
			if (!Problem.empty())
			{
				Line.Note = UTF8_TO_TCHAR(Problem.c_str());
				Report.Bodies.Add(Line);
				continue;
			}
			Line.Props.mass = M.mass;
			Line.Props.centre = Vec3{ M.position[0] * Scale, M.position[1] * Scale, M.position[2] * Scale };
			Line.Props.inertia = { M.inertia[0], -M.inertia[3], -M.inertia[4],
			                       -M.inertia[3], M.inertia[1], -M.inertia[5],
			                       -M.inertia[4], -M.inertia[5], M.inertia[2] };
			Line.bIncluded = true;
			Line.Note = FString::Printf(TEXT("hangs on %s%s%s, %d nodes"), UTF8_TO_TCHAR(M.attach_entity.c_str()), M.attach_mirror ? TEXT(" (mirror)") : TEXT(""),
				M.attach_body.empty() ? TEXT("") : *(TEXT(" / ") + FString(UTF8_TO_TCHAR(M.attach_body.c_str()))), M.attach_nodes);
			Included.push_back(Line.Props);
			++Report.IncludedCount;
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
		// The label column clips at 30 characters: the point-mass count goes
		// in the note.
		Out += ValueRow(FString::Printf(TEXT("ASSEMBLY (%d %s)"), Report.IncludedCount, Report.IncludedCount == 1 ? TEXT("component") : TEXT("components")),
			Report.Assembly,
			Report.PointMasses > 0 ? FString::Printf(TEXT("%d point mass%s among them"), Report.PointMasses, Report.PointMasses == 1 ? TEXT("") : TEXT("es")) : FString());
	}
	else
	{
		Out += EmptyRow(TEXT("ASSEMBLY"), TEXT("no chosen component has mass properties assigned"));
	}
	return Out;
}
