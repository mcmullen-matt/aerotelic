#pragma once

#include "CoreMinimal.h"

// View -> Surface...: a modeless window with Matte / Satin / Glossy /
// Metal presets, three sliders for the finish -- Metallic, Specular
// (gloss) and Roughness (blur) -- and two for the Fresnel sheen the
// surface material adds at grazing angles, its brightness and its
// falloff. Sets all of it for every rendered body at once (the document's
// FAerotelicSurfaceStyle). One instance; a second call brings it to the
// front.
void OpenAerotelicSurfaceDialog();
