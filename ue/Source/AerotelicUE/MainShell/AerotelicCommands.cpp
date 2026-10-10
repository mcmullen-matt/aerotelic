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
	UI_COMMAND(ExportStep, "Export STEP...", "Not available yet in this version", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ExportStl, "Export STL...", "Not available yet in this version", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(Quit, "Quit", "Close the application", EUserInterfaceActionType::Button, FInputChord());

	// Edit -> Preferences
	UI_COMMAND(ColorTheme, "Color Theme...", "Not available yet in this version", EUserInterfaceActionType::Button, FInputChord());

	// Geometry
	UI_COMMAND(Airfoils, "Airfoils...", "Not available yet in this version -- use the Sections tab of a wing's design panel", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(BooleanOperation, "Boolean Operation...", "Fuse/Cut/Common two entities. Not available yet in this version.", EUserInterfaceActionType::Button, FInputChord());

	// View
	UI_COMMAND(ViewIsometric, "Isometric", "Look at the scene from the isometric direction", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ViewTop, "Top", "Look straight down on the scene", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ViewFront, "Front", "Look at the scene from the front", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(ViewSide, "Side", "Look at the scene from the side", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(GroundGrid, "Ground Grid", "Show or hide the ground lattice, sized to the model", EUserInterfaceActionType::ToggleButton, FInputChord(EModifierKey::Control, EKeys::G));
	UI_COMMAND(NavigationCube, "Navigation Cube", "Orientation widget in the top-right corner of the view. Not available yet in this version.", EUserInterfaceActionType::ToggleButton, FInputChord());
	UI_COMMAND(ShowIntermediateFeatures, "Show Intermediate Features", "List the individual steps that make up each authored operation. Not available yet in this version.", EUserInterfaceActionType::ToggleButton, FInputChord());
	UI_COMMAND(SurfaceStyle, "Surface...", "Metallic, glossiness and blur of every rendered surface", EUserInterfaceActionType::Button, FInputChord());
	UI_COMMAND(Rebuild, "Rebuild Wing", "Rebuild the selected entity's geometry", EUserInterfaceActionType::Button, FInputChord(EKeys::F5));

	// Left toolbar dropdowns (_setup_panel_toolbar)
	UI_COMMAND(StyleShaded, "Shaded", "Draw the bodies shaded", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(StyleShadedEdges, "Shaded with Edges", "Draw the bodies shaded, with the triangulation's edges. Not available yet in this version.", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(StyleWireframe, "Wireframe", "Draw the triangulation's edges alone. Not available yet in this version.", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionNone, "None", "No section cut", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionX, "X Plane", "Cut the bodies with a plane normal to X. Not available yet in this version.", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionY, "Y Plane", "Cut the bodies with a plane normal to Y. Not available yet in this version.", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionZ, "Z Plane", "Cut the bodies with a plane normal to Z. Not available yet in this version.", EUserInterfaceActionType::RadioButton, FInputChord());
	UI_COMMAND(SectionFlip, "Flip Side", "Hide the other side of the section plane. Not available yet in this version.", EUserInterfaceActionType::ToggleButton, FInputChord());

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
