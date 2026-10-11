#pragma once

#include "CoreMinimal.h"

// Edit -> Preferences -> Color Theme... (ui/themes.py): WingDesigner's
// thirteen palettes, one current, persisted like its
// QSettings("appearance/theme"). The frame's chrome -- title bar, menu and
// tool bars, scene tree, panels, status bar -- and the 3D view's backdrop
// read their colours from here through attributes, so a change applies at
// once. Dialog windows keep the engine's own window style.
struct FAerotelicPalette
{
	FString Name;
	FLinearColor WinBg, PanelBg, BorderDim, Border, BorderHover, TabHover;
	FLinearColor TextMuted, Text, TextBright, InputBg, BtnBg, BtnHover;
	FLinearColor Accent, AccentHover, AccentPress, AccentFg;
	FLinearColor Scrollbar, ScrollbarHover, StatusBg;
	FLinearColor ViewportBgTop, ViewportBgBot, TitleBarBg, TitleBarText;
};

namespace AerotelicTheme
{
	extern const TCHAR* DefaultTheme;

	// The groups as the picker lists them: Dark, Light, Operational.
	struct FGroup
	{
		FText Title;
		TArray<FString> Names;
	};
	const TArray<FGroup>& Groups();

	const FAerotelicPalette& Palette(const FString& Name); // falls back to the default
	const FAerotelicPalette& Current();
	FString CurrentName();
	// Applies and persists; a saved name from a previous product name is
	// mapped to its current one (canonical_theme_name).
	void SetCurrent(const FString& Name);

	DECLARE_MULTICAST_DELEGATE(FOnThemeChanged);
	FOnThemeChanged& OnChanged();
}
