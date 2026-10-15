#pragma once

#include "CoreMinimal.h"

// View -> Shadows...: a modeless window over the key light's shadow
// settings (the document's FAerotelicShadowStyle) -- a Cast shadows
// checkbox, a Depth bias slider and a Slope (normal) bias slider, with
// Default / No acne / No peter-panning presets. The scene has one
// shadow-casting directional light and renders cascaded shadow maps, so
// these two biases are what decides between a surface shadowing itself in
// stripes and a shadow sliding off its caster. Applied live, like the
// Surface dialog. One instance; a second call brings it to the front.
void OpenAerotelicShadowDialog();
