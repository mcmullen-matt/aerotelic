#pragma once

#include "CoreMinimal.h"

// View -> Surface...: a modeless window with three sliders -- Metallic,
// Specular (gloss) and Roughness (blur) -- and Matte / Glossy / Metal
// presets, setting the finish of every rendered body at once (the
// document's FAerotelicSurfaceStyle). One instance; a second call brings
// it to the front.
void OpenAerotelicSurfaceDialog();
