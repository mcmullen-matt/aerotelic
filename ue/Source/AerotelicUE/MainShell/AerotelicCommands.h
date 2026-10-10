#pragma once

#include "CoreMinimal.h"
#include "Framework/Commands/Commands.h"

// Every menu, toolbar and keyboard action of the application, with the same
// labels and shortcuts as main_window.py's QActions. Items whose backend is
// not in the C++ port yet still appear (disabled) so the menus match the
// Qt version item for item; see SAerotelicMainShell::BindCommands.
class FAerotelicCommands : public TCommands<FAerotelicCommands>
{
public:
	FAerotelicCommands();
	virtual void RegisterCommands() override;

	// File
	TSharedPtr<FUICommandInfo> NewProject;
	TSharedPtr<FUICommandInfo> Save;
	TSharedPtr<FUICommandInfo> SaveAs;
	TSharedPtr<FUICommandInfo> OpenProject;
	TSharedPtr<FUICommandInfo> ExportStep;
	TSharedPtr<FUICommandInfo> ExportStl;
	TSharedPtr<FUICommandInfo> Quit;

	// Edit -> Preferences
	TSharedPtr<FUICommandInfo> ColorTheme;

	// Geometry
	TSharedPtr<FUICommandInfo> Airfoils;
	TSharedPtr<FUICommandInfo> BooleanOperation;

	// View
	TSharedPtr<FUICommandInfo> ViewIsometric;
	TSharedPtr<FUICommandInfo> ViewTop;
	TSharedPtr<FUICommandInfo> ViewFront;
	TSharedPtr<FUICommandInfo> ViewSide;
	TSharedPtr<FUICommandInfo> GroundGrid;
	TSharedPtr<FUICommandInfo> NavigationCube;
	TSharedPtr<FUICommandInfo> ShowIntermediateFeatures;
	TSharedPtr<FUICommandInfo> SurfaceStyle;
	TSharedPtr<FUICommandInfo> Rebuild;

	// Left toolbar dropdowns
	TSharedPtr<FUICommandInfo> StyleShaded;
	TSharedPtr<FUICommandInfo> StyleShadedEdges;
	TSharedPtr<FUICommandInfo> StyleWireframe;
	TSharedPtr<FUICommandInfo> SectionNone;
	TSharedPtr<FUICommandInfo> SectionX;
	TSharedPtr<FUICommandInfo> SectionY;
	TSharedPtr<FUICommandInfo> SectionZ;
	TSharedPtr<FUICommandInfo> SectionFlip;

	// Help
	TSharedPtr<FUICommandInfo> WelcomeScreen;
	TSharedPtr<FUICommandInfo> TutorialProjects;
	TSharedPtr<FUICommandInfo> About;

	// Top toolbar: add entity
	TSharedPtr<FUICommandInfo> AddWingTrapezoid;
	TSharedPtr<FUICommandInfo> AddWingCranked;
	TSharedPtr<FUICommandInfo> AddWingNurbs;
	TSharedPtr<FUICommandInfo> AddFuselage;
	TSharedPtr<FUICommandInfo> AddNacelleRevolved;
	TSharedPtr<FUICommandInfo> AddNacelleLofted;
	TSharedPtr<FUICommandInfo> AddPropeller;
	TSharedPtr<FUICommandInfo> AddSphere;
	TSharedPtr<FUICommandInfo> AddBox;
	TSharedPtr<FUICommandInfo> AddCylinder;
};
