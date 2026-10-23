#include "AerotelicCommands.h"

#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "AerotelicCommands"

FAerotelicCommands::FAerotelicCommands()
	: TCommands<FAerotelicCommands>(TEXT("Aerotelic"), LOCTEXT("Context", "Aerotelic"), NAME_None, FCoreStyle::Get().GetStyleSetName())
{
}

void FAerotelicCommands::RegisterCommands()
{
	// File (main_window.py _setup_menu)
	UI_COMMAND(NewProject, "New Project", "Clear the scene and start a new, empty project", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control, EKeys::N));
	UI_COMMAND(Save, "Save", "Save to the file this project was opened from or last saved as; asks for a file only if there is none yet.", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control, EKeys::S));
	UI_COMMAND(SaveAs, "Save As...", "Save the project to a new file", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control | EModifierKey::Shift, EKeys::S));
	UI_COMMAND(OpenProject, "Open Project...", "Open a project file", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control, EKeys::O));
	UI_COMMAND(ExportStep, "Export STEP...", "Write chosen bodies to one STEP file as named parts, rebuilt at the export tolerance", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ExportStl, "Export STL...", "Mesh chosen bodies at one density and write them to one STL file", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(Export3DPdf, "Export 3D PDF...", "Write chosen bodies to a PDF page as one interactive 3D model, each body a named part in its scene colour", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(Quit, "Quit", "Close the application", EUserInterfaceActionType::Button, FInputChord());

	// Edit
	UI_COMMAND(CopyEntity, "Copy", "Copy the selected parameterisation. Paste makes a second, independent entity from it -- edits to one never reach the other. Boolean results and lattices cannot be copied: they are solids built from other entities, not parameterisations of their own.", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control, EKeys::C));
	UI_COMMAND(PasteEntity, "Paste", "Add a copy of the parameterisation on the clipboard, placed where the original sits and ready to be transformed or edited on its own", EUserInterfaceActionType::Button, FInputChord(EModifierKey::Control, EKeys::V));
	// Edit -> Preferences
	UI_COMMAND(ColorTheme, "Color Theme...", "Pick the application's colour theme (dark, light and operational palettes)", EUserInterfaceActionType::Button, FInputChord());

	// Geometry
	UI_COMMAND(Airfoils, "Airfoils...", "The airfoil library: the Selig database, NACA codes and .dat files, with a preview (assign one from a wing's Sections tab)", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(BooleanOperation, "Boolean Operation...", "Fuse/Cut/Common two entities (or previously-created boolean results)", EUserInterfaceActionType::Button, FInputChord());

	// Lattice
	UI_COMMAND(GenerateLattice, "Generate Lattice...", "Fill the scene's bodies with a lattice (TPMS, strut-based or auxetic, after OpenLattice): pick the type and the parameters that control it", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(RegenerateLattices, "Regenerate Lattices", "Rebuild every lattice over its bodies as they now stand (a lattice already follows an Apply on its bodies; this is for one that failed to, or that was left as it was)", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ExportSkinVtk, "Export Skin Tetrahedra (VTK)...", "Write the skin of every body a lattice fills as tetrahedra conforming exactly to the body's surface outside and to its offset at the skin depth inside (legacy VTK unstructured grid, cell data 'material' = 1), for a finite-element model", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ExportTetVtk, "Export Tetrahedral Mesh (VTK)...", "Write every body a lattice fills as tetrahedra conforming exactly to its surface, split at its offset into skin (material 1) and core (material 2) that together fill the body's volume exactly, as a legacy VTK unstructured grid for a finite-element model", EUserInterfaceActionType::Button, FInputChord());

	// Analysis
	UI_COMMAND(MassProperties, "Mass Properties...", "Weigh selected bodies: the mass, centre of mass and inertia of each, by the density or asserted mass set on its entity in the Scene tree, and of all of them as one assembly about its own centre (parallel-axis theorem)", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(DustVpm, "DUST (Vortex Particle Method)...", "Write a DUST case for selected geometry: surface panels shedding a free vortex-particle wake, which -- unlike the potential flow above -- lifts. The dialog writes the input files and its Help tab gives the commands that run them", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(PotentialFlow, "Potential Flow (Kratos)...", "Compute the potential flow around selected geometry on KratosMultiphysics: the air around it is meshed with tetrahedra bounded by the bodies' own triangulation, and the velocity potential, the velocity and the pressure coefficient are written to a VTK file. No wake, so no lift: the pressure distribution is real, the lift is zero by construction", EUserInterfaceActionType::Button, FInputChord());

	// View
	UI_COMMAND(ViewIsometric, "Isometric", "Look at the scene from the isometric direction", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ViewTop, "Top", "Look straight down on the scene", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ViewFront, "Front", "Look at the scene from the front", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ViewSide, "Side", "Look at the scene from the side", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(GroundGrid, "Ground Grid", "Show or hide the ground lattice, sized to the model", EUserInterfaceActionType::ToggleButton, FInputChord(EModifierKey::Control, EKeys::G));
	UI_COMMAND(NavigationCube, "Navigation Cube", "Orientation widget in the top-right corner of the view: click a face, edge or corner to look from there", EUserInterfaceActionType::ToggleButton, FInputChord());
	UI_COMMAND(ShowIntermediateFeatures, "Show Intermediate Features", "List the individual steps that make up each authored operation. Not available yet in this version.", EUserInterfaceActionType::ToggleButton, FInputChord());
	UI_COMMAND(SurfaceStyle, "Surface...", "Metallic, glossiness, blur and Fresnel edge sheen of every rendered surface", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ShadowStyle, "Shadows...", "Whether the key light casts shadows, and the depth and slope (normal) bias its shadow map is read with", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(Rebuild, "Rebuild Wing", "Rebuild the selected entity's geometry", EUserInterfaceActionType::Button, FInputChord(EKeys::F5));

	// Left toolbar dropdowns (_setup_panel_toolbar)
	UI_COMMAND(StyleShaded, "Shaded", "Draw the bodies shaded", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(StyleShadedEdges, "Shaded with Edges", "Draw the bodies shaded, with the triangulation's edges", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(StyleWireframe, "Wireframe", "Draw the triangulation's edges alone", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionNone, "None", "No section cut", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionX, "X Plane", "Cut the bodies with a plane normal to X; drag the arrow in the view to move it", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionY, "Y Plane", "Cut the bodies with a plane normal to Y; drag the arrow in the view to move it", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionZ, "Z Plane", "Cut the bodies with a plane normal to Z; drag the arrow in the view to move it", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionFlip, "Flip Side", "Hide the other side of the section plane", EUserInterfaceActionType::ToggleButton, FInputChord());

	// Help
	UI_COMMAND(WelcomeScreen, "Welcome Screen...", "The welcome screen: licence state, the legal text, the tutorial projects", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(TutorialProjects, "Tutorial Projects...", "The example projects delivered with the application", EUserInterfaceActionType::Button, FInputChord(EKeys::F1));
	UI_COMMAND(About, "About", "About Aerotelic", EUserInterfaceActionType::Button, FInputChord());

	// Top toolbar (_setup_horizontal_toolbar)
	UI_COMMAND(AddWingTrapezoid, "Trapezoid", "Add wing -- trapezoidal planform (Ct/Cr or taper ratio)", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddWingCranked, "Cranked", "Add wing -- cranked (double-trapezoid) planform", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddWingNurbs, "NURBS", "Add wing -- NURBS LE/TE planform", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddFuselage, "Conic", "Add new Fuselage entity", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddNacelleRevolved, "Revolved", "Add new Nacelle entity -- a cowl swept around its axis, with optional trailing-edge chevrons", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddNacelleLofted, "Lofted", "Add new Nacelle entity -- the second parameterisation: a cowl whose wall is carried on a mean line, drawn as a NURBS centreline and closed guide curves, with outer and inner skins offset from it and optional trailing-edge chevrons", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddPropeller, "Propeller", "Add new Propeller entity -- blades defined like a NURBS wing (planform, sections, rake, blade angle) in units of the tip radius, and a spinner revolved from one curve", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddSphere, "Sphere", "Add new Primitive entity -- a sphere, sized by its radius and stretched along any axis into an ellipsoid, centred on the origin until moved with Transform", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddBox, "Box", "Add new Primitive entity -- a box, sized along X, Y and Z (a cube by default), centred on the origin until moved with Transform", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(AddCylinder, "Cylinder", "Add new Primitive entity -- a cylinder, sized by its radius and length, with its axis along X and centred on the origin until moved with Transform", EUserInterfaceActionType::Button, FInputChord());
}

#undef LOCTEXT_NAMESPACE
