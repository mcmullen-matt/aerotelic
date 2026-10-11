#include "AerotelicTheme.h"

#include "Misc/ConfigCacheIni.h"

#define LOCTEXT_NAMESPACE "AerotelicTheme"

namespace AerotelicTheme
{
	const TCHAR* DefaultTheme = TEXT("Dark Aerospace");

	namespace
	{
		FLinearColor Hex(const TCHAR* H) { return FLinearColor(FColor::FromHex(H)); }

		// ui/themes.py's palettes, token for token (sRGB hex, converted
		// to linear by FColor).
		FAerotelicPalette Make(const TCHAR* Name,
			const TCHAR* WinBg, const TCHAR* PanelBg, const TCHAR* BorderDim, const TCHAR* Border, const TCHAR* BorderHover, const TCHAR* TabHover,
			const TCHAR* TextMuted, const TCHAR* Text, const TCHAR* TextBright, const TCHAR* InputBg, const TCHAR* BtnBg, const TCHAR* BtnHover,
			const TCHAR* Accent, const TCHAR* AccentHover, const TCHAR* AccentPress, const TCHAR* AccentFg,
			const TCHAR* Scrollbar, const TCHAR* ScrollbarHover, const TCHAR* StatusBg,
			const TCHAR* ViewportBgTop, const TCHAR* ViewportBgBot, const TCHAR* TitleBarBg, const TCHAR* TitleBarText)
		{
			FAerotelicPalette P;
			P.Name = Name;
			P.WinBg = Hex(WinBg); P.PanelBg = Hex(PanelBg); P.BorderDim = Hex(BorderDim); P.Border = Hex(Border); P.BorderHover = Hex(BorderHover); P.TabHover = Hex(TabHover);
			P.TextMuted = Hex(TextMuted); P.Text = Hex(Text); P.TextBright = Hex(TextBright); P.InputBg = Hex(InputBg); P.BtnBg = Hex(BtnBg); P.BtnHover = Hex(BtnHover);
			P.Accent = Hex(Accent); P.AccentHover = Hex(AccentHover); P.AccentPress = Hex(AccentPress); P.AccentFg = Hex(AccentFg);
			P.Scrollbar = Hex(Scrollbar); P.ScrollbarHover = Hex(ScrollbarHover); P.StatusBg = Hex(StatusBg);
			P.ViewportBgTop = Hex(ViewportBgTop); P.ViewportBgBot = Hex(ViewportBgBot); P.TitleBarBg = Hex(TitleBarBg); P.TitleBarText = Hex(TitleBarText);
			return P;
		}

		const TArray<FAerotelicPalette>& All()
		{
			static const TArray<FAerotelicPalette> Palettes = {
				Make(TEXT("Dark Aerospace"), TEXT("#0d1117"), TEXT("#161b22"), TEXT("#21262d"), TEXT("#30363d"), TEXT("#484f58"), TEXT("#1c2128"),
					TEXT("#8b949e"), TEXT("#c9d1d9"), TEXT("#e6edf3"), TEXT("#161b22"), TEXT("#21262d"), TEXT("#30363d"),
					TEXT("#1f6feb"), TEXT("#388bfd"), TEXT("#1158c7"), TEXT("#58a6ff"), TEXT("#30363d"), TEXT("#484f58"), TEXT("#161b22"),
					TEXT("#141a26"), TEXT("#0a0d14"), TEXT("#080e18"), TEXT("#7cc0ff")),
				Make(TEXT("Dark Carbon"), TEXT("#0c0e12"), TEXT("#13161c"), TEXT("#1c2128"), TEXT("#282f3a"), TEXT("#3a4455"), TEXT("#181c24"),
					TEXT("#6a7585"), TEXT("#b0bec8"), TEXT("#d0dce8"), TEXT("#13161c"), TEXT("#1c2128"), TEXT("#282f3a"),
					TEXT("#00b4d8"), TEXT("#38c8e8"), TEXT("#0090b0"), TEXT("#38c8e8"), TEXT("#282f3a"), TEXT("#3a4455"), TEXT("#13161c"),
					TEXT("#0e1118"), TEXT("#08090f"), TEXT("#090b0f"), TEXT("#38c8e8")),
				Make(TEXT("Dark Warm"), TEXT("#0f0d09"), TEXT("#1a1710"), TEXT("#2a2418"), TEXT("#3d3420"), TEXT("#584a2a"), TEXT("#201c12"),
					TEXT("#9a8870"), TEXT("#d4c4a8"), TEXT("#eadcc8"), TEXT("#1a1710"), TEXT("#2a2418"), TEXT("#3d3420"),
					TEXT("#d4772a"), TEXT("#e8963e"), TEXT("#b05818"), TEXT("#e8a84a"), TEXT("#3d3420"), TEXT("#584a2a"), TEXT("#1a1710"),
					TEXT("#16120a"), TEXT("#0c0904"), TEXT("#0c0a06"), TEXT("#e8a84a")),
				Make(TEXT("Dark Nord"), TEXT("#2e3440"), TEXT("#3b4252"), TEXT("#434c5e"), TEXT("#4c566a"), TEXT("#616e88"), TEXT("#353d4d"),
					TEXT("#616e88"), TEXT("#d8dee9"), TEXT("#eceff4"), TEXT("#3b4252"), TEXT("#434c5e"), TEXT("#4c566a"),
					TEXT("#5e81ac"), TEXT("#81a1c1"), TEXT("#4c6d8c"), TEXT("#81a1c1"), TEXT("#4c566a"), TEXT("#616e88"), TEXT("#3b4252"),
					TEXT("#262e3c"), TEXT("#1c2430"), TEXT("#222a38"), TEXT("#81a1c1")),
				Make(TEXT("Dark Monokai"), TEXT("#272822"), TEXT("#1e1f1c"), TEXT("#383830"), TEXT("#49483e"), TEXT("#75715e"), TEXT("#2d2e2a"),
					TEXT("#75715e"), TEXT("#f8f8f2"), TEXT("#ffffff"), TEXT("#1e1f1c"), TEXT("#383830"), TEXT("#49483e"),
					TEXT("#a6e22e"), TEXT("#b8f040"), TEXT("#85c020"), TEXT("#a6e22e"), TEXT("#49483e"), TEXT("#75715e"), TEXT("#1e1f1c"),
					TEXT("#1c1d1a"), TEXT("#141510"), TEXT("#191a17"), TEXT("#a6e22e")),
				Make(TEXT("Dark OLED"), TEXT("#000000"), TEXT("#0a0a0a"), TEXT("#141414"), TEXT("#1f1f1f"), TEXT("#333333"), TEXT("#141414"),
					TEXT("#555555"), TEXT("#cccccc"), TEXT("#ffffff"), TEXT("#0a0a0a"), TEXT("#141414"), TEXT("#1f1f1f"),
					TEXT("#00aaff"), TEXT("#33bbff"), TEXT("#0077cc"), TEXT("#33bbff"), TEXT("#1f1f1f"), TEXT("#333333"), TEXT("#0a0a0a"),
					TEXT("#050505"), TEXT("#000000"), TEXT("#000000"), TEXT("#33bbff")),
				Make(TEXT("Dark WingDesigner"), TEXT("#303030"), TEXT("#282828"), TEXT("#1d1d1d"), TEXT("#3d3d3d"), TEXT("#4a4a4a"), TEXT("#3a3a3a"),
					TEXT("#989898"), TEXT("#e6e6e6"), TEXT("#ffffff"), TEXT("#1d1d1d"), TEXT("#545454"), TEXT("#656565"),
					TEXT("#4772b3"), TEXT("#5680c2"), TEXT("#3d6299"), TEXT("#71a8ff"), TEXT("#545454"), TEXT("#656565"), TEXT("#181818"),
					TEXT("#3f3f3f"), TEXT("#3f3f3f"), TEXT("#181818"), TEXT("#e6e6e6")),
				Make(TEXT("Light Classic"), TEXT("#f5f6f8"), TEXT("#ffffff"), TEXT("#d0d7de"), TEXT("#b0bac8"), TEXT("#8a94a0"), TEXT("#eaeef2"),
					TEXT("#57606a"), TEXT("#24292f"), TEXT("#1c2128"), TEXT("#ffffff"), TEXT("#eaeef2"), TEXT("#d0d7de"),
					TEXT("#0969da"), TEXT("#0550ae"), TEXT("#033d8b"), TEXT("#0550ae"), TEXT("#b0bac8"), TEXT("#8a94a0"), TEXT("#eaeef2"),
					TEXT("#d8dde5"), TEXT("#c8cdd8"), TEXT("#e0e4ea"), TEXT("#0550ae")),
				Make(TEXT("Light Blueprint"), TEXT("#e8edf5"), TEXT("#f4f7fc"), TEXT("#c0ccde"), TEXT("#90a0c0"), TEXT("#6878a0"), TEXT("#dce4f0"),
					TEXT("#4a5a7a"), TEXT("#1a2a48"), TEXT("#0a1830"), TEXT("#f4f7fc"), TEXT("#dce4f0"), TEXT("#c0ccde"),
					TEXT("#2354c8"), TEXT("#3468d8"), TEXT("#1440a8"), TEXT("#2354c8"), TEXT("#90a0c0"), TEXT("#6878a0"), TEXT("#dce4f0"),
					TEXT("#ccd4e8"), TEXT("#bcc8e0"), TEXT("#d0d8ec"), TEXT("#2354c8")),
				Make(TEXT("Light Solarized"), TEXT("#fdf6e3"), TEXT("#eee8d5"), TEXT("#d3cbb9"), TEXT("#c2baa5"), TEXT("#a09890"), TEXT("#e8e0ce"),
					TEXT("#93a1a1"), TEXT("#657b83"), TEXT("#586e75"), TEXT("#eee8d5"), TEXT("#e0d9c8"), TEXT("#d3cbb9"),
					TEXT("#268bd2"), TEXT("#3a9fe0"), TEXT("#1a70b0"), TEXT("#268bd2"), TEXT("#c2baa5"), TEXT("#a09890"), TEXT("#eee8d5"),
					TEXT("#e0d8c4"), TEXT("#d0c8b0"), TEXT("#ddd5c0"), TEXT("#268bd2")),
				Make(TEXT("Light High Contrast"), TEXT("#ffffff"), TEXT("#f0f0f0"), TEXT("#cccccc"), TEXT("#999999"), TEXT("#666666"), TEXT("#e0e0e0"),
					TEXT("#555555"), TEXT("#000000"), TEXT("#000000"), TEXT("#ffffff"), TEXT("#e0e0e0"), TEXT("#c8c8c8"),
					TEXT("#0000cc"), TEXT("#0000ff"), TEXT("#000099"), TEXT("#0000cc"), TEXT("#999999"), TEXT("#666666"), TEXT("#f0f0f0"),
					TEXT("#d8d8d8"), TEXT("#c0c0c0"), TEXT("#e0e0e0"), TEXT("#0000cc")),
				Make(TEXT("Night Vision"), TEXT("#0d0200"), TEXT("#180400"), TEXT("#2a0800"), TEXT("#3d1000"), TEXT("#581800"), TEXT("#200600"),
					TEXT("#8b2000"), TEXT("#cc3300"), TEXT("#ff5500"), TEXT("#180400"), TEXT("#2a0800"), TEXT("#3d1000"),
					TEXT("#cc2200"), TEXT("#e63300"), TEXT("#aa1800"), TEXT("#ff4400"), TEXT("#3d1000"), TEXT("#581800"), TEXT("#180400"),
					TEXT("#140400"), TEXT("#0a0200"), TEXT("#0a0100"), TEXT("#ff4400")),
				Make(TEXT("Radar"), TEXT("#000d00"), TEXT("#001400"), TEXT("#002200"), TEXT("#003300"), TEXT("#004400"), TEXT("#001a00"),
					TEXT("#006600"), TEXT("#00aa44"), TEXT("#00cc66"), TEXT("#001400"), TEXT("#002200"), TEXT("#003300"),
					TEXT("#00cc44"), TEXT("#00ff55"), TEXT("#009933"), TEXT("#00dd55"), TEXT("#003300"), TEXT("#004400"), TEXT("#001400"),
					TEXT("#001800"), TEXT("#000d00"), TEXT("#000a00"), TEXT("#00dd55")),
			};
			return Palettes;
		}

		// themes.py _RENAMED_THEMES: a name saved under a previous product name.
		FString Canonical(const FString& Name)
		{
			return Name == TEXT("Dark Aerotelic") ? FString(TEXT("Dark WingDesigner")) : Name;
		}

		const TCHAR* kSection = TEXT("Aerotelic");
		const TCHAR* kKey = TEXT("Theme");

		FString& StoredName()
		{
			static FString Name;
			static bool bLoaded = false;
			if (!bLoaded)
			{
				bLoaded = true;
				FString Saved;
				if (GConfig != nullptr && GConfig->GetString(kSection, kKey, Saved, GGameUserSettingsIni) && !Saved.IsEmpty())
				{
					Name = Canonical(Saved);
				}
				if (Name.IsEmpty())
				{
					Name = DefaultTheme;
				}
			}
			return Name;
		}

		FOnThemeChanged& Delegate()
		{
			static FOnThemeChanged D;
			return D;
		}
	}

	const TArray<FGroup>& Groups()
	{
		static const TArray<FGroup> G = {
			{ LOCTEXT("Dark", "── Dark ──"), { TEXT("Dark Aerospace"), TEXT("Dark Carbon"), TEXT("Dark Warm"), TEXT("Dark Nord"), TEXT("Dark Monokai"), TEXT("Dark OLED"), TEXT("Dark WingDesigner") } },
			{ LOCTEXT("Light", "── Light ──"), { TEXT("Light Classic"), TEXT("Light Blueprint"), TEXT("Light Solarized"), TEXT("Light High Contrast") } },
			{ LOCTEXT("Operational", "── Operational ──"), { TEXT("Night Vision"), TEXT("Radar") } },
		};
		return G;
	}

	const FAerotelicPalette& Palette(const FString& Name)
	{
		const FString Wanted = Canonical(Name);
		for (const FAerotelicPalette& P : All())
		{
			if (P.Name == Wanted)
			{
				return P;
			}
		}
		return All()[0];
	}

	const FAerotelicPalette& Current()
	{
		return Palette(StoredName());
	}

	FString CurrentName()
	{
		return StoredName();
	}

	void SetCurrent(const FString& Name)
	{
		const FString Wanted = Palette(Name).Name;
		StoredName() = Wanted;
		if (GConfig != nullptr)
		{
			GConfig->SetString(kSection, kKey, *Wanted, GGameUserSettingsIni);
			GConfig->Flush(false, GGameUserSettingsIni);
		}
		Delegate().Broadcast();
	}

	FOnThemeChanged& OnChanged()
	{
		return Delegate();
	}
}

#undef LOCTEXT_NAMESPACE
