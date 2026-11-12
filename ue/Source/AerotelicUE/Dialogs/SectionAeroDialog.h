#pragma once

#include "CoreMinimal.h"
#include <memory>

namespace aerotelic { class Airfoil; }

// Analysis -> Section Aerodynamics (also "Analyse section..." in the Airfoil
// library): the two-dimensional aerodynamics of one airfoil per unit span,
// on core section_aero.hpp.
//
// Left, the section's control-point editor over a re-representation of the
// library airfoil -- a least-squares B-spline of the degree, pole count and
// knot placement chosen in the Representation group (Airfoil::Refit), with
// the error it introduces against the library points reported (max and RMS
// normal distance, where, the change of thickness and camber, the
// trailing-edge gap). The poles can be dragged, added, removed and their
// knots edited as in every curve editor; the library contour is drawn
// behind as a grey outline. At each pole an arrow shows the sensitivity of
// a chosen scalar (Cl, Cm c/4, friction drag, Cp at a probe) to moving
// that pole, by central differences on the analysis.
//
// Right, the pressure distribution of every enabled method on one chart
// (upper surface solid, lower dashed, Cp negative upward), with a
// transition cue (the earlier of Re_x reaching 3.5e6 and the first adverse
// gradient) and Stratford's turbulent-separation cue on each surface, the
// Stratford parameter optionally on the right axis; below it a table of
// the force and moment coefficients per method. A Polar tab sweeps the
// angle of attack. Apply adds the edited section to the session's custom
// airfoil library under a name of your choosing, where every section
// picker lists it.
//
// The first (and so far only) method is Theodorsen's conformal map in the
// form of Ives (1976); the registry in core adds more side by side.
// Modeless; solves in place (well under a millisecond each). Status
// receives the status-bar line.
void ShowAerotelicSectionAeroDialog(TFunction<void(const FText&)> Status, std::shared_ptr<aerotelic::Airfoil> Initial = nullptr);
