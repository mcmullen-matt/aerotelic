using UnrealBuildTool;

// Editor target: retained for the headless test commandlets
// (-run=AerotelicDocumentTest / AerotelicRealProjectTest / AerotelicSmokeTest)
// and as a fallback host for the shell (Window -> Aerotelic). The application
// itself is the AerotelicUE game target (AerotelicUE.Target.cs).
public class AerotelicUEEditorTarget : TargetRules
{
	public AerotelicUEEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("AerotelicUE");
	}
}
