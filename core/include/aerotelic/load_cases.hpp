#pragma once

// LOAD CASES (Model > Load Cases, 2026-11-15): what the structural analysis
// is run for, defined once and saved with the project under `load_cases:`
// (project_yaml.hpp encodes them). Two kinds:
//
//   AERO: a point of the V-n diagram -- a speed and a load factor in air of
//   a density -- flown by a set of solid bodies. The high order panel
//   method trims the configuration to it (angle of attack and one flap) and
//   its pressures load the shells in free flight (inertia relief, gravity at
//   1 g, so the structure feels n at its mass centre). The solved pressure
//   field is session data, recomputed on demand; only the definition is
//   saved.
//
//   STRUCTURAL (non-aero): the bodies, the inertial load factor, a uniform
//   pressure, point forces, and the constraints -- support planes (or
//   patches of them) or free flight with a reference point -- as the Kratos
//   dialog took them until now.
//
// Lengths and coordinates are the project's units, full-scale (as the
// parameterisations are); the aero figures are SI (m/s, kg/m^3, N, m^2),
// since the air is not in the project's units; forces are newtons and the
// uniform pressure is in the project's stress unit, as the dialog shows it.

#include <array>
#include <string>
#include <vector>

namespace aerotelic {

// A solid body a case involves: the scene name of its entity (or boolean
// result), the mirrored half or not, and the body's role ("Main", "Flap 1",
// "Body 2"; empty: the entity's every body).
struct LoadCaseBody {
	std::string entity;
	bool mirror = false;
	std::string body;
	bool operator==(const LoadCaseBody& o) const { return entity == o.entity && mirror == o.mirror && body == o.body; }
};

// The V-n diagram's envelope: the stall boundaries n = q CL S / W from
// CL_max and CL_min, the structural limits n_max and n_min, the never-
// exceed speed. Weight and area default to the mass report of the chosen
// bodies and the chosen wings' planform area ("auto"); typed otherwise.
struct VnDiagramSpec {
	double cl_max = 1.4;
	double cl_min = -0.8;
	double v_ne = 60.0;        // m/s
	double n_max = 4.0;
	double n_min = -2.0;
	double density = 1.225;    // kg/m^3
	bool weight_auto = true;
	double weight = 0.0;       // N, when not auto
	bool area_auto = true;
	double area = 0.0;         // m^2, when not auto

	// The stall boundary's load factor at a speed: q CL S / W (0 without
	// a weight and area).
	static double StallLoadFactor(double cl, double speed, double density, double weight, double area) {
		return weight > 0.0 && area > 0.0 ? 0.5 * density * speed * speed * cl * area / weight : 0.0;
	}
	// The speed at which the stall boundary meets a load factor: sqrt(2 n W / (rho CL S)).
	static double StallSpeed(double cl, double n, double density, double weight, double area);
};

struct AeroLoadCaseSpec {
	std::string name = "Case";
	double speed = 20.0;       // m/s
	double density = 1.225;    // kg/m^3
	double load_factor = 1.0;
	// The flap the trim turns ("<entity>", "<flap name>"); empty: the angle
	// of attack alone, the moment untrimmed.
	std::string control_entity, control_flap;
	std::vector<LoadCaseBody> bodies;
};

// A support plane or patch (AerotelicStructuralFea.h FSupport, as typed).
struct LoadCaseSupportSpec {
	char axis = 'y';
	std::string at = "value";   // "value" | "min" | "max"
	double value = 0.0;
	double tolerance = 0.0;     // 0: the pre-processor's default
	bool bounded = false;
	std::array<double, 2> lo{0.0, 0.0}, hi{0.0, 0.0};
	// Whose minimum / maximum (at = min | max): a body, or none for the
	// whole mesh.
	bool has_of = false;
	LoadCaseBody of;
};

struct LoadCasePointForceSpec {
	std::array<double, 3> at{0.0, 0.0, 0.0};
	std::array<double, 3> force{0.0, 0.0, -1000.0};
};

struct StaticLoadCaseSpec {
	std::string name = "Case";
	double load_factor = 1.0;
	double pressure = 0.0;      // the project's stress unit, positive inward
	bool free_flight = false;
	bool reference_given = false;   // free flight: a typed reference point rather than the mass centre
	std::array<double, 3> reference{0.0, 0.0, 0.0};
	std::vector<LoadCaseSupportSpec> supports;
	std::vector<LoadCasePointForceSpec> point_forces;
	std::vector<LoadCaseBody> bodies;
};

struct LoadCasesSpec {
	VnDiagramSpec vn;
	std::vector<AeroLoadCaseSpec> aero;
	std::vector<StaticLoadCaseSpec> structural;
	bool IsEmpty() const { return aero.empty() && structural.empty(); }
};

// Why a spec cannot be used, or empty: a case without a name, two cases
// of one kind with the same name, a support axis other than x, y, z, an
// `at` other than the three words, a V-n envelope with n_min >= n_max or
// CL_min >= CL_max.
std::string LoadCasesProblem(const LoadCasesSpec& spec);

} // namespace aerotelic
