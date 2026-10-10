using System.IO;
using System.Linq;
using System.Collections.Generic;
using UnrealBuildTool;

public class AerotelicUE : ModuleRules
{
	public AerotelicUE(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// One translation unit per .cpp. Several dialogs keep same-named
		// helpers (FormRow, Row, ...) in their own anonymous namespaces; a
		// unity build concatenates those files and the names collide. The
		// module is small, so the compile-time saving of unity is negligible.
		bUseUnity = false;

		// Lets any file in this module #include another by a module-root-
		// relative path (e.g. "Entity/AerotelicDocumentSubsystem.h" from
		// MainShell/AerotelicMainShell.cpp) -- bLegacyPublicIncludePaths
		// being off by default otherwise only searches the including file's
		// own directory plus whatever's explicitly listed here.
		PublicIncludePaths.Add(ModuleDirectory);

		// OCCT requires RTTI and C++ exceptions; both are off by default for
		// UE modules. Proven in aerotelic_occt_spike before committing to
		// this real project.
		bUseRTTI = true;
		bEnableExceptions = true;

		// Runtime module: this is the standalone application's primary game
		// module (see AerotelicUE.Target.cs). Everything the app needs --
		// Slate for the shell/dialogs, ProceduralMeshComponent for the
		// tessellated OCCT meshes, DesktopPlatform for the native Open/Save
		// file pickers -- is available to a non-Shipping game target.
		PublicDependencyModuleNames.AddRange(new string[] {
			"Core", "CoreUObject", "Engine", "InputCore", "ApplicationCore",
			"Slate", "SlateCore",
			// Runtime module (Engine/Source/Runtime/AppFramework): the
			// colour picker (SColorPicker / OpenColorPicker) and SColorBlock
			// the scene tree's Color command and swatches use.
			"AppFramework",
			"ProceduralMeshComponent",
			"DesktopPlatform",
			// Licensing: the licence server's JSON over HTTPS.
			"HTTP", "Json",
		});

		// Licensing: Ed25519 signature verification and SHA-256 for the
		// machine id, from the engine's own OpenSSL (the same copy its HTTPS
		// stack uses).
		AddEngineThirdPartyPrivateStaticDependencies(Target, "OpenSSL");

		// Editor-only extras: the "Window -> Aerotelic" nomad tab that hosts
		// the same shell inside the Unreal Editor (kept as a fallback and for
		// the test commandlets' editor target). None of this is linked into
		// the standalone AerotelicUE.exe.
		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.AddRange(new string[] {
				"UnrealEd", "LevelEditor", "ToolMenus", "EditorSubsystem",
				"WorkspaceMenuStructure", "EditorStyle",
				// AerotelicMaterialSetupCommandlet authors the surface material.
				"MaterialEditor",
			});
		}

		// aerotelic_cpp_core: the already-built, fixture-validated geometry
		// backend (nurbs/airfoil/wing/fuselage/nacelle/lofted_nacelle/
		// propeller/primitive/feature_tree/project_yaml). Built Release to
		// match yaml-cpp's conda-forge binaries' CRT (see the project's own
		// notes on the Debug/Release crash this avoids) -- UE's own
		// Development config links the same, non-debug CRT.
		// The repository is <root>/core + <root>/ue; this module lives at
		// ue/Source/AerotelicUE, so the core is three levels up and over.
		string CoreRoot = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..", "..", "core"));
		PublicIncludePaths.Add(Path.Combine(CoreRoot, "include"));
		PublicAdditionalLibraries.Add(Path.Combine(CoreRoot, "build", "Release", "aerotelic_core.lib"));

		// OCCT 7.9.3, core geometry/lofting toolkits only -- same set
		// aerotelic_occt_spike proved. Deliberately EXCLUDES the STEP
		// export / XDE toolkits (TKDESTEP, TKXSBase, TKDE, and transitively
		// TKXCAF/TKVCAF/TKCAF/TKLCAF/TKCDF/TKV3d/TKService): nothing in this
		// module calls *Model::ExportStep() yet, and pulling that whole
		// chain in dragged along third-party imaging libraries (FreeImage,
		// libraw, and -- critically -- zlib/freetype/libpng16/tiff under
		// the SAME names UE's own engine already loads for texture/font
		// work). Staging OCCT's conda-forge-built copies of those four over
		// UE's own caused a real STATUS_HEAP_CORRUPTION crash once UE's code
		// used the shadowed, ABI-incompatible copy. ExportStep() now lives in
		// its own translation unit (aerotelic_cpp_core/src/step_export.cpp)
		// specifically so that a consumer that never calls it -- this module
		// -- never pulls its .obj into the link and therefore never needs
		// TKDESTEP resolved at all (ordinary static-lib "only link what's
		// referenced" semantics; MSVC's function-level stripping within an
		// already-pulled-in .obj does NOT reliably discard this on its own).
		// Revisit when a real "Export STEP" UI feature needs them, and
		// solve the DLL-collision problem properly then (e.g. an isolated
		// subprocess, or Windows API Sets / SxS redirection for just those
		// four names) rather than carrying the risk for an unused feature.
		string OcctRoot = @"C:\Users\Matt\micromamba\envs\aerotelic\Library";
		PublicIncludePaths.Add(Path.Combine(OcctRoot, "include", "opencascade"));
		string[] OcctLibs = {
			"TKernel", "TKMath", "TKG2d", "TKG3d", "TKGeomAlgo", "TKGeomBase",
			"TKBRep", "TKTopAlgo", "TKPrim", "TKBO", "TKBool", "TKShHealing", "TKMesh",
			"TKFillet", "TKOffset", "TKFeat",
		};
		foreach (string Lib in OcctLibs) {
			string LibPath = Path.Combine(OcctRoot, "lib", Lib + ".lib");
			if (File.Exists(LibPath)) PublicAdditionalLibraries.Add(LibPath);
		}

		// yaml-cpp -- vendored from source and built as a static library with
		// this same toolchain (core/third_party/yaml-cpp, a git submodule),
		// NOT linked against the prebuilt conda-forge DLL. yaml-cpp's API
		// crosses module boundaries carrying std::string/std::vector-heavy
		// YAML::Node values; a DLL built by a different toolchain/CRT config
		// than this module caused a real STATUS_HEAP_CORRUPTION crash
		// (allocated in one CRT heap, freed in another) the first time
		// LoadProjectYaml() actually ran inside Unreal. A statically-linked,
		// same-toolchain build has no separate heap to mismatch.
		string YamlCppRoot = Path.Combine(CoreRoot, "third_party", "yaml-cpp");
		PublicIncludePaths.Add(Path.Combine(YamlCppRoot, "include"));
		PublicAdditionalLibraries.Add(Path.Combine(YamlCppRoot, "build", "Release", "yaml-cpp.lib"));
		PublicDefinitions.Add("YAML_CPP_STATIC_DEFINE");

		// Stage the runtime DLLs for exactly the OCCT toolkits linked above,
		// next to the output binary (UnrealEditor-AerotelicUE.dll for the
		// editor target, AerotelicUE.exe for the standalone one), so Windows'
		// "search the loading module's directory" rule finds them before any
		// of our code runs (implicit dependencies resolve before
		// StartupModule() runs, so a PATH-env workaround can't carry over).
		// yaml-cpp needs no entry here now that it's statically linked.
		var DllSources = new List<string>();
		foreach (string Lib in OcctLibs) {
			string DllPath = Path.Combine(OcctRoot, "bin", Lib + ".dll");
			if (File.Exists(DllPath)) DllSources.Add(DllPath);
		}

		foreach (string DllPath in DllSources) {
			string FileName = Path.GetFileName(DllPath);
			RuntimeDependencies.Add(Path.Combine("$(BinaryOutputDir)", FileName), DllPath);
		}
	}
}
