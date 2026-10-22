#include "aerotelic/project_yaml.hpp"

#include <fstream>
#include <set>
#include <stdexcept>

namespace aerotelic::project_yaml {

namespace {

std::vector<double> DecodeDoubles(const YAML::Node& node) {
	std::vector<double> out;
	if (!node) return out;
	for (const auto& v : node) out.push_back(v.as<double>());
	return out;
}

YAML::Node EncodeDoubles(const std::vector<double>& v) {
	YAML::Node n(YAML::NodeType::Sequence);
	for (double x : v) n.push_back(x);
	return n;
}

} // namespace

// ===========================================================================
// NurbsSpec
// ===========================================================================

YAML::Node Encode(const NurbsSpec& spec) {
	YAML::Node n;
	n["degree"] = spec.degree;
	YAML::Node poles(YAML::NodeType::Sequence);
	for (int i = 0; i < spec.poles.rows(); ++i) {
		YAML::Node row(YAML::NodeType::Sequence);
		for (int j = 0; j < spec.poles.cols(); ++j) row.push_back(spec.poles(i, j));
		poles.push_back(row);
	}
	n["poles"] = poles;
	if (spec.weights) n["weights"] = EncodeDoubles(*spec.weights);
	if (spec.periodic) {
		n["periodic"] = true;
		if (spec.clamped_seam) n["clamped_seam"] = true;
		n["knots"] = EncodeDoubles(spec.knots.value_or(std::vector<double>{}));
	} else {
		n["interior_knots"] = EncodeDoubles(spec.interior_knots.value_or(std::vector<double>{}));
	}
	return n;
}

NurbsSpec DecodeNurbsSpec(const YAML::Node& node) {
	NurbsSpec spec;
	spec.degree = node["degree"].as<int>();
	const YAML::Node& poles_node = node["poles"];
	int n = static_cast<int>(poles_node.size());
	int d = n > 0 ? static_cast<int>(poles_node[0].size()) : 0;
	Mat poles(n, d);
	for (int i = 0; i < n; ++i)
		for (int j = 0; j < d; ++j) poles(i, j) = poles_node[i][j].as<double>();
	spec.poles = poles;
	if (node["weights"]) spec.weights = DecodeDoubles(node["weights"]);
	if (node["periodic"] && node["periodic"].as<bool>()) {
		spec.periodic = true;
		if (node["clamped_seam"]) spec.clamped_seam = node["clamped_seam"].as<bool>();
		spec.knots = DecodeDoubles(node["knots"]);
	} else {
		spec.interior_knots = DecodeDoubles(node["interior_knots"]);
	}
	return spec;
}

// ===========================================================================
// PlanformSpec
// ===========================================================================

YAML::Node Encode(const PlanformSpec& spec) {
	YAML::Node n;
	n["type"] = spec.type;
	n["semi_span"] = spec.semi_span;
	if (spec.type == "standard" || spec.type == "taper") {
		n["root_chord"] = spec.root_chord;
		n["tip_chord"] = spec.tip_chord;
		n["sweep_angle"] = spec.sweep_angle;
		n["sweep_fraction"] = spec.sweep_fraction;
		n["rotation_fraction"] = spec.rotation_fraction;
	} else if (spec.type == "double_trap") {
		n["root_chord"] = spec.root_chord;
		n["tip_chord"] = spec.tip_chord;
		n["kink_fraction"] = spec.kink_fraction;
		n["rotation_fraction"] = spec.rotation_fraction;
		n["leading_edge_sweep_angle"] = spec.leading_edge_sweep_angle;
		n["trailing_edge_sweep_angle"] = spec.trailing_edge_sweep_angle;
	} else if (spec.type == "nurbs") {
		n["rotation_fraction"] = spec.rotation_fraction;
		if (spec.le_curve) n["le_curve"] = Encode(*spec.le_curve);
		if (spec.te_curve) n["te_curve"] = Encode(*spec.te_curve);
	}
	n["twist_curve"] = Encode(spec.twist_curve);
	n["dihedral_curve"] = Encode(spec.dihedral_curve);
	return n;
}

PlanformSpec DecodePlanformSpec(const YAML::Node& node) {
	PlanformSpec spec;
	spec.type = node["type"] ? node["type"].as<std::string>() : "standard";
	spec.semi_span = node["semi_span"].as<double>();
	if (node["root_chord"]) spec.root_chord = node["root_chord"].as<double>();
	if (node["tip_chord"]) spec.tip_chord = node["tip_chord"].as<double>();
	if (node["sweep_angle"]) spec.sweep_angle = node["sweep_angle"].as<double>();
	if (node["sweep_fraction"]) spec.sweep_fraction = node["sweep_fraction"].as<double>();
	if (node["rotation_fraction"]) spec.rotation_fraction = node["rotation_fraction"].as<double>();
	if (node["kink_fraction"]) spec.kink_fraction = node["kink_fraction"].as<double>();
	if (node["leading_edge_sweep_angle"]) spec.leading_edge_sweep_angle = node["leading_edge_sweep_angle"].as<double>();
	if (node["trailing_edge_sweep_angle"]) spec.trailing_edge_sweep_angle = node["trailing_edge_sweep_angle"].as<double>();
	if (node["le_curve"]) spec.le_curve = DecodeNurbsSpec(node["le_curve"]);
	if (node["te_curve"]) spec.te_curve = DecodeNurbsSpec(node["te_curve"]);
	if (node["twist_curve"]) spec.twist_curve = DecodeNurbsSpec(node["twist_curve"]);
	if (node["dihedral_curve"]) spec.dihedral_curve = DecodeNurbsSpec(node["dihedral_curve"]);
	return spec;
}

// ===========================================================================
// Reference images (core/reference_image.py's serialize()/_migrate)
// ===========================================================================

namespace {

// Written only when there is at least one image, under the model, keyed by
// view -- the Python key order: path, scale, x, y, crop, opacity, flip_h,
// flip_v, angle.
void EncodeReferenceImages(YAML::Node& model_node, const ReferenceImageMap& images) {
	if (images.empty()) return;
	YAML::Node all;
	for (const auto& [key, r] : images) {
		YAML::Node n;
		n["path"] = r.path;
		n["scale"] = r.scale;
		n["x"] = r.x;
		n["y"] = r.y;
		YAML::Node crop;
		for (double v : r.crop) crop.push_back(v);
		n["crop"] = crop;
		n["opacity"] = r.opacity;
		n["flip_h"] = r.flip_h;
		n["flip_v"] = r.flip_v;
		n["angle"] = r.angle;
		all[key] = n;
	}
	model_node["reference_images"] = all;
}

// Every key is optional (Python's {**_DEFAULT, **d}). `unit` is the
// entity's reference length, for the legacy metre-based keys (mpp, x_m,
// y_m), applied only when no "scale" key is present so a file can never be
// converted twice.
ReferenceImageMap DecodeReferenceImages(const YAML::Node& model_node, double unit) {
	ReferenceImageMap out;
	const YAML::Node& all = model_node["reference_images"];
	if (!all || !all.IsMap()) return out;
	const double u = unit > 0.0 ? unit : 1.0;
	for (const auto& kv : all) {
		const std::string key = kv.first.as<std::string>();
		const YAML::Node& n = kv.second;
		if (!n || !n.IsMap()) continue;
		ReferenceImageSpec r;
		if (n["path"]) r.path = n["path"].as<std::string>();
		if (n["scale"]) {
			r.scale = n["scale"].as<double>();
			if (n["x"]) r.x = n["x"].as<double>();
			if (n["y"]) r.y = n["y"].as<double>();
		} else {
			if (n["mpp"]) r.scale = n["mpp"].as<double>() / u;
			if (n["x_m"]) r.x = n["x_m"].as<double>() / u;
			else if (n["x"]) r.x = n["x"].as<double>();
			if (n["y_m"]) r.y = n["y_m"].as<double>() / u;
			else if (n["y"]) r.y = n["y"].as<double>();
		}
		if (n["crop"] && n["crop"].IsSequence() && n["crop"].size() == 4) {
			for (size_t i = 0; i < 4; ++i) r.crop[i] = n["crop"][i].as<double>();
		}
		if (n["opacity"]) r.opacity = n["opacity"].as<double>();
		if (n["flip_h"]) r.flip_h = n["flip_h"].as<bool>();
		if (n["flip_v"]) r.flip_v = n["flip_v"].as<bool>();
		if (n["angle"]) r.angle = n["angle"].as<double>();
		out[key] = r;
	}
	return out;
}

} // namespace

// ===========================================================================
// WingModelSpec
// ===========================================================================

YAML::Node Encode(const WingModelSpec& spec) {
	YAML::Node n;
	n["name"] = spec.name;
	n["planform"] = Encode(spec.planform);
	YAML::Node secs(YAML::NodeType::Sequence);
	for (const auto& s : spec.sections) {
		YAML::Node sn;
		sn["s"] = s.s;
		YAML::Node af;
		af["name"] = s.airfoil_name;
		af["curve"] = Encode(s.airfoil_curve);
		sn["airfoil"] = af;
		secs.push_back(sn);
	}
	n["sections"] = secs;
	EncodeReferenceImages(n, spec.reference_images);
	return n;
}

WingModelSpec DecodeWingModelSpec(const YAML::Node& node) {
	WingModelSpec spec;
	spec.name = node["name"] ? node["name"].as<std::string>() : "Wing";
	spec.planform = DecodePlanformSpec(node["planform"]);
	spec.reference_images = DecodeReferenceImages(node, spec.planform.semi_span);
	if (node["sections"]) {
		for (const auto& sn : node["sections"]) {
			WingSectionSpec ss;
			ss.s = sn["s"].as<double>();
			const YAML::Node& af = sn["airfoil"];
			ss.airfoil_name = af["name"] ? af["name"].as<std::string>() : "Unnamed";
			ss.airfoil_curve = DecodeNurbsSpec(af["curve"]);
			spec.sections.push_back(ss);
		}
	}
	return spec;
}

// ===========================================================================
// NacelleSpec
// ===========================================================================

YAML::Node Encode(const NacelleSpec& spec) {
	YAML::Node n;
	n["length"] = spec.length;
	n["name"] = spec.name;
	n["upper_curve"] = Encode(spec.upper_curve);
	n["lower_curve"] = Encode(spec.lower_curve);
	n["front_curve"] = Encode(spec.front_curve);
	n["chevron_count"] = spec.chevron_count;
	if (spec.chevron_depth) n["chevron_depth"] = *spec.chevron_depth;
	EncodeReferenceImages(n, spec.reference_images);
	return n;
}

NacelleSpec DecodeNacelleSpec(const YAML::Node& node) {
	NacelleSpec spec;
	spec.length = node["length"].as<double>();
	spec.reference_images = DecodeReferenceImages(node, 1.0);
	spec.name = node["name"] ? node["name"].as<std::string>() : "Nacelle";
	spec.upper_curve = DecodeNurbsSpec(node["upper_curve"]);
	spec.lower_curve = DecodeNurbsSpec(node["lower_curve"]);
	spec.front_curve = DecodeNurbsSpec(node["front_curve"]);
	spec.chevron_count = node["chevron_count"] ? node["chevron_count"].as<int>() : 0;
	if (node["chevron_depth"]) spec.chevron_depth = node["chevron_depth"].as<double>();
	else if (node["chevron_angle"]) spec.chevron_angle = node["chevron_angle"].as<double>();
	return spec;
}

// ===========================================================================
// LoftedNacelleSpec
// ===========================================================================

YAML::Node Encode(const LoftedNacelleSpec& spec) {
	YAML::Node n;
	n["length"] = spec.length;
	n["name"] = spec.name;
	n["centreline_curve"] = Encode(spec.centreline_curve);
	YAML::Node secs(YAML::NodeType::Sequence);
	for (const auto& s : spec.sections) {
		YAML::Node sn;
		sn["station"] = s.station;
		sn["curve"] = Encode(s.curve);
		secs.push_back(sn);
	}
	n["sections"] = secs;
	n["outer_curve"] = Encode(spec.outer_curve);
	n["inner_curve"] = Encode(spec.inner_curve);
	n["chevron_count"] = spec.chevron_count;
	n["chevron_depth"] = spec.chevron_depth;
	EncodeReferenceImages(n, spec.reference_images);
	return n;
}

LoftedNacelleSpec DecodeLoftedNacelleSpec(const YAML::Node& node) {
	LoftedNacelleSpec spec;
	spec.length = node["length"].as<double>();
	spec.reference_images = DecodeReferenceImages(node, 1.0);
	spec.name = node["name"] ? node["name"].as<std::string>() : "Nacelle";
	spec.centreline_curve = DecodeNurbsSpec(node["centreline_curve"]);
	spec.sections.clear();
	if (node["sections"]) {
		for (const auto& sn : node["sections"]) {
			GuideSectionSpec gs;
			gs.station = sn["station"].as<double>();
			gs.curve = DecodeNurbsSpec(sn["curve"]);
			spec.sections.push_back(gs);
		}
	}
	spec.outer_curve = DecodeNurbsSpec(node["outer_curve"]);
	spec.inner_curve = DecodeNurbsSpec(node["inner_curve"]);
	spec.chevron_count = node["chevron_count"] ? node["chevron_count"].as<int>() : 0;
	spec.chevron_depth = node["chevron_depth"] ? node["chevron_depth"].as<double>() : 0.25;
	return spec;
}

// ===========================================================================
// PropellerSpec
// ===========================================================================

YAML::Node Encode(const PropellerSpec& spec) {
	YAML::Node n;
	n["name"] = spec.name;
	n["radius"] = spec.radius;
	n["blade_count"] = spec.blade_count;
	n["blade_x"] = spec.blade_x;
	n["hub_radius"] = spec.hub_radius;
	n["spinner"] = spec.spinner;
	n["spinner_length"] = spec.spinner_length;
	n["spin_direction"] = std::string(SpinDirectionName(spec.spin_direction));
	n["spinner_curve"] = Encode(spec.spinner_curve);
	if (spec.blade) n["blade"] = Encode(*spec.blade);
	EncodeReferenceImages(n, spec.reference_images);
	return n;
}

PropellerSpec DecodePropellerSpec(const YAML::Node& node) {
	PropellerSpec spec;
	spec.name = node["name"] ? node["name"].as<std::string>() : "Propeller";
	spec.radius = node["radius"].as<double>();
	spec.reference_images = DecodeReferenceImages(node, 1.0);
	spec.blade_count = node["blade_count"] ? node["blade_count"].as<int>() : 2;
	spec.blade_x = node["blade_x"] ? node["blade_x"].as<double>() : 0.26;
	spec.hub_radius = node["hub_radius"] ? node["hub_radius"].as<double>() : 0.12;
	// Absent means on: every project written before the spinner became
	// optional had one, and reading those files must not silently remove it.
	spec.spinner = node["spinner"] ? node["spinner"].as<bool>() : true;
	spec.spinner_length = node["spinner_length"] ? node["spinner_length"].as<double>() : 0.40;
	// Absent means positive: that was the only hand the model had before
	// this field, so an older project must read back turning as it did.
	spec.spin_direction = node["spin_direction"] ? ParseSpinDirection(node["spin_direction"].as<std::string>()) : SpinDirection::Positive;
	if (node["spinner_curve"]) spec.spinner_curve = DecodeNurbsSpec(node["spinner_curve"]);
	if (node["blade"]) spec.blade = DecodeWingModelSpec(node["blade"]);
	return spec;
}

// ===========================================================================
// FuselageSpec
// ===========================================================================

YAML::Node Encode(const FuselageSpec& spec) {
	YAML::Node n;
	n["length"] = spec.length;
	n["name"] = spec.name;
	n["z_upper_curve"] = Encode(spec.z_upper_curve);
	n["z_lower_curve"] = Encode(spec.z_lower_curve);
	n["y_half_curve"] = Encode(spec.y_half_curve);
	n["rho_upper_curve"] = Encode(spec.rho_upper_curve);
	n["rho_lower_curve"] = Encode(spec.rho_lower_curve);
	// Present-but-empty: signals "not a pre-2026 legacy file" to the Python
	// app without asserting a nose link this port doesn't actually track.
	n["pole_links"] = YAML::Node(YAML::NodeType::Map);
	EncodeReferenceImages(n, spec.reference_images);
	return n;
}

FuselageSpec DecodeFuselageSpec(const YAML::Node& node, bool* out_legacy) {
	FuselageSpec spec;
	spec.length = node["length"].as<double>();
	spec.reference_images = DecodeReferenceImages(node, spec.length);
	spec.name = node["name"] ? node["name"].as<std::string>() : "Fuselage";
	spec.z_upper_curve = DecodeNurbsSpec(node["z_upper_curve"]);
	spec.z_lower_curve = DecodeNurbsSpec(node["z_lower_curve"]);
	spec.y_half_curve = DecodeNurbsSpec(node["y_half_curve"]);
	spec.rho_upper_curve = DecodeNurbsSpec(node["rho_upper_curve"]);
	spec.rho_lower_curve = DecodeNurbsSpec(node["rho_lower_curve"]);
	bool legacy = !node["pole_links"];
	spec.legacy = legacy;
	if (out_legacy) *out_legacy = legacy;
	return spec;
}

// ===========================================================================
// PrimitiveSpec
// ===========================================================================

YAML::Node Encode(const PrimitiveSpec& spec) {
	YAML::Node n;
	n["shape"] = std::string(ShapeName(spec.shape));
	n["name"] = spec.name;
	n["radius"] = spec.radius;
	YAML::Node stretch(YAML::NodeType::Sequence);
	for (double v : spec.stretch) stretch.push_back(v);
	n["stretch"] = stretch;
	YAML::Node size(YAML::NodeType::Sequence);
	for (double v : spec.size) size.push_back(v);
	n["size"] = size;
	n["cyl_radius"] = spec.cyl_radius;
	n["cyl_length"] = spec.cyl_length;
	return n;
}

PrimitiveSpec DecodePrimitiveSpec(const YAML::Node& node) {
	PrimitiveSpec spec;
	spec.shape = ParseShape(node["shape"] ? node["shape"].as<std::string>() : "sphere");
	spec.name = node["name"] ? node["name"].as<std::string>() : "Primitive";
	spec.radius = node["radius"] ? node["radius"].as<double>() : 1.0;
	if (node["stretch"]) {
		int i = 0;
		for (const auto& v : node["stretch"]) spec.stretch[i++] = v.as<double>();
	}
	if (node["size"]) {
		int i = 0;
		for (const auto& v : node["size"]) spec.size[i++] = v.as<double>();
	}
	spec.cyl_radius = node["cyl_radius"] ? node["cyl_radius"].as<double>() : 1.0;
	spec.cyl_length = node["cyl_length"] ? node["cyl_length"].as<double>() : 2.0;
	return spec;
}

// ===========================================================================
// Placement
// ===========================================================================

YAML::Node Encode(const Placement& p) {
	YAML::Node n;
	n["translation"] = EncodeDoubles({p.translation[0], p.translation[1], p.translation[2]});
	n["rotation_axis"] = EncodeDoubles({p.rotation_axis[0], p.rotation_axis[1], p.rotation_axis[2]});
	n["rotation_deg"] = p.rotation_deg;
	// Only when the rotation was authored as an Euler triple. Older files
	// have no such key and every reader older than this one ignores it:
	// rotation_axis/rotation_deg above are always the whole rotation.
	if (p.euler.used) {
		YAML::Node e;
		e["order"] = std::string(EulerOrderName(p.euler.order));
		e["frame"] = std::string(p.euler.intrinsic ? "intrinsic" : "extrinsic");
		e["angles_deg"] = EncodeDoubles({p.euler.angles_deg[0], p.euler.angles_deg[1], p.euler.angles_deg[2]});
		n["rotation_euler"] = e;
	}
	return n;
}

Placement DecodePlacement(const YAML::Node& node) {
	Placement p{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0};
	if (!node) return p;
	if (node["translation"]) {
		int i = 0;
		for (const auto& v : node["translation"]) p.translation[i++] = v.as<double>();
	}
	if (node["rotation_axis"]) {
		int i = 0;
		for (const auto& v : node["rotation_axis"]) p.rotation_axis[i++] = v.as<double>();
	}
	if (node["rotation_deg"]) p.rotation_deg = node["rotation_deg"].as<double>();
	if (const YAML::Node e = node["rotation_euler"]) {
		p.euler.used = true;
		if (e["order"]) p.euler.order = ParseEulerOrder(e["order"].as<std::string>());
		if (e["frame"]) p.euler.intrinsic = e["frame"].as<std::string>() != "extrinsic";
		if (e["angles_deg"]) {
			int i = 0;
			for (const auto& v : e["angles_deg"]) p.euler.angles_deg[i++] = v.as<double>();
		}
		// The triple is the authored rotation, so it wins over whatever
		// axis+angle the file carries -- a hand-edited rotation_deg beside
		// an Euler block would otherwise silently disagree with the tab.
		SyncPlacementRotation(p);
	}
	return p;
}

// ===========================================================================
// Entity type
// ===========================================================================

const char* EntityTypeName(EntityType t) {
	switch (t) {
		case EntityType::Wing: return "wing";
		case EntityType::Fuselage: return "fuselage";
		case EntityType::Nacelle: return "nacelle";
		case EntityType::LoftedNacelle: return "lofted_nacelle";
		case EntityType::Propeller: return "propeller";
		case EntityType::Primitive: return "primitive";
		case EntityType::Boolean: return "boolean";
		case EntityType::Lattice: return "lattice";
	}
	return "";
}

EntityType ParseEntityType(const std::string& s) {
	if (s == "wing") return EntityType::Wing;
	if (s == "fuselage") return EntityType::Fuselage;
	if (s == "nacelle") return EntityType::Nacelle;
	if (s == "lofted_nacelle") return EntityType::LoftedNacelle;
	if (s == "propeller") return EntityType::Propeller;
	if (s == "primitive") return EntityType::Primitive;
	if (s == "boolean") return EntityType::Boolean;
	if (s == "lattice") return EntityType::Lattice;
	throw std::invalid_argument("Unknown entity type '" + s + "'");
}

// ===========================================================================
// Boolean operations (`booleans:`)
// ===========================================================================

namespace {

YAML::Node EncodeColor(const std::array<double, 3>& color) {
	YAML::Node c;
	for (double v : color) c.push_back(v);
	c.SetStyle(YAML::EmitterStyle::Flow);
	return c;
}

std::optional<std::array<double, 3>> DecodeColor(const YAML::Node& node) {
	if (node && node.IsSequence() && node.size() >= 3) {
		return std::array<double, 3>{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
	}
	return std::nullopt;
}

} // namespace

// ===========================================================================
// Mass (`mass:` on an entity, a boolean result or a lattice)
// ===========================================================================

YAML::Node EncodeMassSpec(const MassSpec& m) {
	// The key says the mode, so the file reads as the dialog does:
	// `mass: {asserted: 12.5}` or `mass: {density: 2700}`. Unset writes
	// nothing (the caller leaves the key out).
	YAML::Node n;
	if (m.mode == MassMode::Asserted) n["asserted"] = m.mass;
	else if (m.mode == MassMode::Density) n["density"] = m.density;
	n.SetStyle(YAML::EmitterStyle::Flow);
	return n;
}

MassSpec DecodeMassSpec(const YAML::Node& node) {
	MassSpec m;
	if (!node || !node.IsMap()) return m;
	if (node["asserted"]) {
		m.mode = MassMode::Asserted;
		m.mass = node["asserted"].as<double>();
	} else if (node["density"]) {
		m.mode = MassMode::Density;
		m.density = node["density"].as<double>();
	}
	return m;
}

YAML::Node Encode(const ProjectBoolean& b) {
	YAML::Node n;
	n["name"] = b.name;
	n["operation"] = b.operation;
	YAML::Node operands(YAML::NodeType::Sequence);
	for (const BooleanOperand& op : b.operands) {
		if (op.mirror) {
			YAML::Node m;
			m["mirror"] = op.name;
			m.SetStyle(YAML::EmitterStyle::Flow);
			operands.push_back(m);
		} else {
			operands.push_back(op.name);
		}
	}
	n["operands"] = operands;
	if (b.fillet_radius) n["fillet_radius"] = *b.fillet_radius;
	if (b.hidden) n["hidden"] = true;
	if (b.color) n["color"] = EncodeColor(*b.color);
	if (b.mass.mode != MassMode::Unset) n["mass"] = EncodeMassSpec(b.mass);
	if (!b.bodies.empty()) {
		YAML::Node bodies(YAML::NodeType::Sequence);
		for (const ProjectBooleanBody& body : b.bodies) {
			YAML::Node bn;
			bn["name"] = body.name;
			if (body.color) bn["color"] = EncodeColor(*body.color);
			if (body.hidden) bn["hidden"] = true;
			bodies.push_back(bn);
		}
		n["bodies"] = bodies;
	}
	return n;
}

ProjectBoolean DecodeProjectBoolean(const YAML::Node& node) {
	ProjectBoolean b;
	if (!node["name"]) throw std::invalid_argument("a boolean operation has no name");
	b.name = node["name"].as<std::string>();
	b.operation = node["operation"] ? node["operation"].as<std::string>() : "fuse";
	if (b.operation != "fuse" && b.operation != "cut" && b.operation != "common") {
		throw std::invalid_argument("Boolean '" + b.name + "' has an unknown operation '" + b.operation + "'");
	}
	for (const auto& ref : node["operands"]) {
		BooleanOperand op;
		if (ref.IsMap() && ref["mirror"]) {
			op.name = ref["mirror"].as<std::string>();
			op.mirror = true;
		} else {
			op.name = ref.as<std::string>();
		}
		b.operands.push_back(op);
	}
	if (b.operands.size() < 2) throw std::invalid_argument("Boolean '" + b.name + "' needs at least two operands.");
	if (node["fillet_radius"]) b.fillet_radius = node["fillet_radius"].as<double>();
	b.hidden = node["hidden"] ? node["hidden"].as<bool>() : false;
	b.color = DecodeColor(node["color"]);
	b.mass = DecodeMassSpec(node["mass"]);
	for (const auto& bn : node["bodies"]) {
		ProjectBooleanBody body;
		body.name = bn["name"] ? bn["name"].as<std::string>() : "";
		body.color = DecodeColor(bn["color"]);
		body.hidden = bn["hidden"] ? bn["hidden"].as<bool>() : false;
		b.bodies.push_back(body);
	}
	return b;
}

// ===========================================================================
// Lattices (`lattices:`)
// ===========================================================================

namespace {

YAML::Node EncodeOperands(const std::vector<BooleanOperand>& ops) {
	YAML::Node out(YAML::NodeType::Sequence);
	for (const BooleanOperand& op : ops) {
		if (op.mirror) {
			YAML::Node m;
			m["mirror"] = op.name;
			m.SetStyle(YAML::EmitterStyle::Flow);
			out.push_back(m);
		} else {
			out.push_back(op.name);
		}
	}
	return out;
}

std::vector<BooleanOperand> DecodeOperands(const YAML::Node& node) {
	std::vector<BooleanOperand> out;
	if (!node) return out;
	for (const auto& ref : node) {
		BooleanOperand op;
		if (ref.IsMap() && ref["mirror"]) {
			op.name = ref["mirror"].as<std::string>();
			op.mirror = true;
		} else {
			op.name = ref.as<std::string>();
		}
		out.push_back(op);
	}
	return out;
}

} // namespace

YAML::Node Encode(const LatticeSpec& spec) {
	YAML::Node n;
	n["family"] = std::string(LatticeFamilyName(spec.family));
	n["type"] = spec.type;
	if (spec.family == LatticeFamily::Tpms) n["phase"] = std::string(TpmsPhaseName(spec.phase));
	if (spec.family == LatticeFamily::Honeycomb) {
		n["cell_aspect"] = spec.cell_aspect;
		n["axis"] = std::string(PrismAxisName(spec.axis));
		n["wall_sizing"] = std::string(WallSizingName(spec.wall_sizing));
	}
	n["cell_size"] = spec.cell_size;
	n["volume_fraction"] = spec.volume_fraction;
	n["wall_thickness"] = spec.wall_thickness;
	n["resolution"] = spec.resolution;
	n["skin_thickness"] = spec.skin_thickness;
	YAML::Node origin;
	origin.push_back(spec.origin.x);
	origin.push_back(spec.origin.y);
	origin.push_back(spec.origin.z);
	origin.SetStyle(YAML::EmitterStyle::Flow);
	n["origin"] = origin;
	return n;
}

LatticeSpec DecodeLatticeSpec(const YAML::Node& node) {
	LatticeSpec s;
	if (!node) return s;
	if (node["family"]) s.family = ParseLatticeFamily(node["family"].as<std::string>());
	if (node["type"]) s.type = node["type"].as<std::string>();
	if (node["phase"]) s.phase = ParseTpmsPhase(node["phase"].as<std::string>());
	if (node["cell_aspect"]) s.cell_aspect = node["cell_aspect"].as<double>();
	if (node["axis"]) s.axis = ParsePrismAxis(node["axis"].as<std::string>());
	if (node["wall_sizing"]) s.wall_sizing = ParseWallSizing(node["wall_sizing"].as<std::string>());
	if (node["cell_size"]) s.cell_size = node["cell_size"].as<double>();
	if (node["volume_fraction"]) s.volume_fraction = node["volume_fraction"].as<double>();
	if (node["wall_thickness"]) s.wall_thickness = node["wall_thickness"].as<double>();
	if (node["resolution"]) s.resolution = node["resolution"].as<int>();
	if (node["skin_thickness"]) s.skin_thickness = node["skin_thickness"].as<double>();
	if (node["origin"] && node["origin"].IsSequence() && node["origin"].size() >= 3) {
		s.origin = Vec3{node["origin"][0].as<double>(), node["origin"][1].as<double>(), node["origin"][2].as<double>()};
	}
	const auto& names = LatticeTypeNames(s.family);
	bool known = false;
	for (const std::string& nm : names) {
		if (nm == s.type) known = true;
	}
	if (!known) throw std::invalid_argument("unknown " + std::string(LatticeFamilyName(s.family)) + " lattice type '" + s.type + "'");
	if (!(s.cell_size > 0.0)) throw std::invalid_argument("the lattice cell size must be greater than zero");
	if (s.resolution < 2) throw std::invalid_argument("the lattice resolution must be at least 2 voxels per cell");
	if (s.family == LatticeFamily::Honeycomb) {
		if (!(s.cell_aspect > 0.0)) throw std::invalid_argument("the honeycomb cell aspect must be greater than zero");
		if (s.wall_sizing == WallSizing::Thickness && !(s.wall_thickness > 0.0)) {
			throw std::invalid_argument("the honeycomb wall thickness must be greater than zero");
		}
	}
	return s;
}

YAML::Node Encode(const ProjectLattice& l) {
	YAML::Node n;
	n["name"] = l.name;
	n["lattice"] = Encode(l.spec);
	n["targets"] = EncodeOperands(l.targets);
	if (!l.exclusions.empty()) n["exclusions"] = EncodeOperands(l.exclusions);
	if (l.hidden) n["hidden"] = true;
	if (l.color) n["color"] = EncodeColor(*l.color);
	if (l.mass.mode != MassMode::Unset) n["mass"] = EncodeMassSpec(l.mass);
	return n;
}

ProjectLattice DecodeProjectLattice(const YAML::Node& node) {
	ProjectLattice l;
	if (!node["name"]) throw std::invalid_argument("a lattice has no name");
	l.name = node["name"].as<std::string>();
	l.spec = DecodeLatticeSpec(node["lattice"]);
	l.targets = DecodeOperands(node["targets"]);
	l.exclusions = DecodeOperands(node["exclusions"]);
	if (l.targets.empty()) throw std::invalid_argument("Lattice '" + l.name + "' fills no bodies.");
	l.hidden = node["hidden"] ? node["hidden"].as<bool>() : false;
	l.color = DecodeColor(node["color"]);
	l.mass = DecodeMassSpec(node["mass"]);
	return l;
}

// ===========================================================================
// Project file
// ===========================================================================

namespace {

YAML::Node EncodeEntity(const ProjectEntity& e) {
	YAML::Node n;
	n["name"] = e.name;
	n["type"] = std::string(EntityTypeName(e.type));
	n["mirror"] = e.mirror;
	n["placement"] = Encode(e.placement);
	YAML::Node model_node;
	std::visit([&](const auto& spec) { model_node = Encode(spec); }, e.model);
	n["model"] = model_node;
	if (e.color) {
		YAML::Node c;
		for (double v : *e.color) c.push_back(v);
		c.SetStyle(YAML::EmitterStyle::Flow);
		n["color"] = c;
	}
	if (e.hidden) n["hidden"] = true;
	if (e.mass.mode != MassMode::Unset) n["mass"] = EncodeMassSpec(e.mass);
	return n;
}

ProjectEntity DecodeEntity(const YAML::Node& node) {
	ProjectEntity e;
	// The legacy WingDesigner format (projects/cessna_172.yaml) keys an
	// entity by `id` with a display `label` and carries a raw `features`
	// graph in place of `booleans`; the label is the name there.
	if (node["name"]) e.name = node["name"].as<std::string>();
	else if (node["label"]) e.name = node["label"].as<std::string>();
	else if (node["id"]) e.name = node["id"].as<std::string>();
	else e.name = "Entity";
	e.type = ParseEntityType(node["type"].as<std::string>());
	e.mirror = node["mirror"] ? node["mirror"].as<bool>() : false;
	e.placement = DecodePlacement(node["placement"]);
	const YAML::Node& model_node = node["model"];
	switch (e.type) {
		case EntityType::Wing: e.model = DecodeWingModelSpec(model_node); break;
		case EntityType::Fuselage: e.model = DecodeFuselageSpec(model_node); break;
		case EntityType::Nacelle: e.model = DecodeNacelleSpec(model_node); break;
		case EntityType::LoftedNacelle: e.model = DecodeLoftedNacelleSpec(model_node); break;
		case EntityType::Propeller: e.model = DecodePropellerSpec(model_node); break;
		case EntityType::Primitive: e.model = DecodePrimitiveSpec(model_node); break;
		case EntityType::Boolean: throw std::invalid_argument("Entity '" + e.name + "': boolean results belong under 'booleans', not 'entities'");
		case EntityType::Lattice: throw std::invalid_argument("Entity '" + e.name + "': lattices belong under 'lattices', not 'entities'");
	}
	if (node["color"] && node["color"].size() >= 3) {
		const YAML::Node& c = node["color"];
		e.color = std::array<double, 3>{c[0].as<double>(), c[1].as<double>(), c[2].as<double>()};
	}
	e.hidden = node["hidden"] ? node["hidden"].as<bool>() : false;
	e.mass = DecodeMassSpec(node["mass"]);
	return e;
}

} // namespace

ProjectFile LoadProjectYaml(const std::string& path) {
	YAML::Node root = YAML::LoadFile(path);
	ProjectFile project;
	// Absent means metres and kilograms -- every project before the key
	// existed was taken to be in them. A unit the application does not
	// offer, or a time unit other than seconds, is refused rather than
	// silently read as something else.
	if (const YAML::Node units = root["units"]) {
		if (units["length"]) project.units.length = ParseLengthUnit(units["length"].as<std::string>());
		if (units["mass"]) project.units.mass = ParseMassUnit(units["mass"].as<std::string>());
		if (units["time"]) CheckTimeUnit(units["time"].as<std::string>());
	}
	if (root["entities"]) {
		for (const auto& en : root["entities"]) project.entities.push_back(DecodeEntity(en));
	}
	// Validated as the Python loader does: every operand must name an
	// entity or a boolean listed before it, and a mirror operand a
	// mirrored entity.
	std::set<std::string> seen, mirrored;
	for (const auto& e : project.entities) {
		seen.insert(e.name);
		if (e.mirror) mirrored.insert(e.name);
	}
	if (root["booleans"]) {
		for (const auto& bn : root["booleans"]) {
			ProjectBoolean b = DecodeProjectBoolean(bn);
			for (const BooleanOperand& op : b.operands) {
				if (!seen.count(op.name)) {
					throw std::invalid_argument("Boolean '" + b.name + "' refers to '" + op.name + "', which does not exist (or is listed after it).");
				}
				if (op.mirror && !mirrored.count(op.name)) {
					throw std::invalid_argument("Boolean '" + b.name + "' refers to the mirror of '" + op.name + "', which is not mirrored.");
				}
			}
			seen.insert(b.name);
			project.booleans.push_back(std::move(b));
		}
	}
	if (root["lattices"]) {
		for (const auto& ln : root["lattices"]) {
			ProjectLattice l = DecodeProjectLattice(ln);
			for (const auto* list : {&l.targets, &l.exclusions}) {
				for (const BooleanOperand& op : *list) {
					if (!seen.count(op.name)) {
						throw std::invalid_argument("Lattice '" + l.name + "' refers to '" + op.name + "', which does not exist.");
					}
					if (op.mirror && !mirrored.count(op.name)) {
						throw std::invalid_argument("Lattice '" + l.name + "' refers to the mirror of '" + op.name + "', which is not mirrored.");
					}
				}
			}
			project.lattices.push_back(std::move(l));
		}
	}
	return project;
}

void SaveProjectYaml(const ProjectFile& project, const std::string& path) {
	YAML::Node root;
	// First in the file, and always written -- the defaults included --
	// so a reader sees what the numbers below mean without knowing the
	// application's assumption. Time is stated too, though it is not a
	// choice: the file documents it rather than relying on the reader.
	YAML::Node units;
	units["length"] = std::string(LengthUnitName(project.units.length));
	units["mass"] = std::string(MassUnitName(project.units.mass));
	units["time"] = std::string("s");
	units.SetStyle(YAML::EmitterStyle::Flow);
	root["units"] = units;
	YAML::Node entities(YAML::NodeType::Sequence);
	for (const auto& e : project.entities) entities.push_back(EncodeEntity(e));
	root["entities"] = entities;
	if (!project.booleans.empty()) {
		YAML::Node booleans(YAML::NodeType::Sequence);
		for (const auto& b : project.booleans) booleans.push_back(Encode(b));
		root["booleans"] = booleans;
	}
	if (!project.lattices.empty()) {
		YAML::Node lattices(YAML::NodeType::Sequence);
		for (const auto& l : project.lattices) lattices.push_back(Encode(l));
		root["lattices"] = lattices;
	}

	std::ofstream out(path);
	if (!out) throw std::runtime_error("could not open " + path + " for writing");
	out << YAML::Dump(root);
}

} // namespace aerotelic::project_yaml
