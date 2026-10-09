using UnrealBuildTool;

// The standalone application: builds Binaries/Win64/AerotelicUE.exe, a
// monolithic game-type executable that hosts the Aerotelic shell, dialogs and
// 3D view directly -- no Unreal Editor in the loop. See run_aerotelic.bat.
public class AerotelicUETarget : TargetRules
{
	public AerotelicUETarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("AerotelicUE");
	}
}
