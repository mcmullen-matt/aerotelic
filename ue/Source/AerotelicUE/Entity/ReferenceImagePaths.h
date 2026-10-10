#pragma once

#include "CoreMinimal.h"
#include "aerotelic/project_yaml.hpp"

// Reference-image path handling around Save/Open, core/reference_image.py's
// project_directory() rule: an image inside the project file's directory is
// written relative to it (forward slashes), so a project can be moved or
// installed and still find its underlays; anything else stays absolute.
// In memory every path is absolute.

// Before writing: rewrites the paths in a serialised model spec.
void RelativiseReferenceImagePaths(aerotelic::project_yaml::EntityModelSpec& Spec, const FString& ProjectDir);

// After loading: resolves relative paths on the live model against the
// project's directory.
void ResolveReferenceImagePaths(aerotelic::ISolidModel* Model, aerotelic::project_yaml::EntityType Type, const FString& ProjectDir);
