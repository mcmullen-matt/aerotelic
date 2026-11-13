#include "aerotelic/project_yaml.hpp"

#include "aerotelic/laminate.hpp"

#include <algorithm>
#include <cmath>
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

// The wing's internal structure (wing_model.hpp WingStructureSpec), under
// the wing model's `structure:` key (2026-11-09). Always written, so a
// saved wing says what it has even when that is nothing; a wing read
// without the key gets the defaults (two spars, eight ribs), which is
// what a wing made before the key existed is taken to have.
YAML::Node EncodeWingStructure(const WingStructureSpec& s) {
	YAML::Node n;
	YAML::Node ribs;
	ribs["at_sections"] = s.ribs.at_sections;
	ribs["at_flap_ends"] = s.ribs.at_flap_ends;
	ribs["between"] = s.ribs.between;
	if (!s.ribs.extra.empty()) {
		YAML::Node extra(YAML::NodeType::Sequence);
		for (double e : s.ribs.extra) extra.push_back(e);
		ribs["extra"] = extra;
	}
	ribs["thickness"] = s.ribs.thickness;
	if (s.ribs.material.HasDensity()) ribs["material"] = EncodeMaterial(s.ribs.material);
	n["ribs"] = ribs;
	YAML::Node spars(YAML::NodeType::Sequence);
	for (const WingSparSpec& sp : s.spars) {
		YAML::Node m;
		m["name"] = sp.name;
		m["x_root"] = sp.x_root;
		m["x_tip"] = sp.x_tip;
		m["eta_root"] = sp.eta_root;
		m["eta_tip"] = sp.eta_tip;
		m["thickness"] = sp.thickness;
		if (sp.material.HasDensity()) m["material"] = EncodeMaterial(sp.material);
		spars.push_back(m);
	}
	n["spars"] = spars;
	YAML::Node stringers(YAML::NodeType::Sequence);
	for (const WingStringerSpec& st : s.stringers) {
		YAML::Node m;
		m["name"] = st.name;
		m["x_root"] = st.x_root;
		m["x_tip"] = st.x_tip;
		m["surface"] = st.upper ? "upper" : "lower";
		m["eta_root"] = st.eta_root;
		m["eta_tip"] = st.eta_tip;
		m["height"] = st.height;
		m["thickness"] = st.thickness;
		if (st.material.HasDensity()) m["material"] = EncodeMaterial(st.material);
		stringers.push_back(m);
	}
	n["stringers"] = stringers;
	return n;
}

// The frames and stringers of a body lofted along its length (a fuselage,
// a nacelle: structure_plan.hpp FrameStructureSpec), under the model's
// `structure:` key (2026-11-10). Always written; a model read without the
// key gets the defaults (eight frames, four longerons).
YAML::Node EncodeFrameStructure(const FrameStructureSpec& s) {
	YAML::Node n;
	YAML::Node frames;
	frames["between"] = s.frames.between;
	if (!s.frames.extra.empty()) {
		YAML::Node extra(YAML::NodeType::Sequence);
		for (double e : s.frames.extra) extra.push_back(e);
		frames["extra"] = extra;
	}
	frames["height"] = s.frames.height;
	frames["thickness"] = s.frames.thickness;
	if (s.frames.material.HasDensity()) frames["material"] = EncodeMaterial(s.frames.material);
	n["frames"] = frames;
	YAML::Node stringers(YAML::NodeType::Sequence);
	for (const FrameStringerSpec& st : s.stringers) {
		YAML::Node m;
		m["name"] = st.name;
		m["around"] = st.around;
		m["x_start"] = st.x_start;
		m["x_end"] = st.x_end;
		m["height"] = st.height;
		m["thickness"] = st.thickness;
		if (st.material.HasDensity()) m["material"] = EncodeMaterial(st.material);
		stringers.push_back(m);
	}
	n["stringers"] = stringers;
	return n;
}

FrameStructureSpec DecodeFrameStructure(const YAML::Node& node) {
	if (!node || !node.IsMap()) return FrameStructureSpec::Default();
	FrameStructureSpec s;
	// The file says what the body has; the fresh spec's default stringers go.
	s.stringers.clear();
	if (const YAML::Node frames = node["frames"]) {
		if (frames["between"]) s.frames.between = frames["between"].as<int>();
		if (frames["extra"] && frames["extra"].IsSequence()) for (const auto& e : frames["extra"]) s.frames.extra.push_back(e.as<double>());
		if (frames["height"]) s.frames.height = frames["height"].as<double>();
		if (frames["thickness"]) s.frames.thickness = frames["thickness"].as<double>();
		if (frames["material"]) s.frames.material = DecodeMaterial(frames["material"]);
	}
	if (node["stringers"] && node["stringers"].IsSequence()) {
		for (const auto& m : node["stringers"]) {
			FrameStringerSpec st;
			if (m["name"]) st.name = m["name"].as<std::string>();
			if (m["around"]) st.around = m["around"].as<double>();
			if (m["x_start"]) st.x_start = m["x_start"].as<double>();
			if (m["x_end"]) st.x_end = m["x_end"].as<double>();
			if (m["height"]) st.height = m["height"].as<double>();
			if (m["thickness"]) st.thickness = m["thickness"].as<double>();
			if (m["material"]) st.material = DecodeMaterial(m["material"]);
			s.stringers.push_back(st);
		}
	}
	return s;
}

WingStructureSpec DecodeWingStructure(const YAML::Node& node) {
	if (!node || !node.IsMap()) return WingStructureSpec::Default();
	WingStructureSpec s;
	// The file says what the wing has; the fresh spec's default spars go.
	s.spars.clear();
	s.stringers.clear();
	if (const YAML::Node ribs = node["ribs"]) {
		if (ribs["at_sections"]) s.ribs.at_sections = ribs["at_sections"].as<bool>();
		if (ribs["at_flap_ends"]) s.ribs.at_flap_ends = ribs["at_flap_ends"].as<bool>();
		if (ribs["between"]) s.ribs.between = ribs["between"].as<int>();
		if (ribs["extra"] && ribs["extra"].IsSequence()) for (const auto& e : ribs["extra"]) s.ribs.extra.push_back(e.as<double>());
		if (ribs["thickness"]) s.ribs.thickness = ribs["thickness"].as<double>();
		if (ribs["material"]) s.ribs.material = DecodeMaterial(ribs["material"]);
	}
	if (node["spars"] && node["spars"].IsSequence()) {
		for (const auto& m : node["spars"]) {
			WingSparSpec sp;
			if (m["name"]) sp.name = m["name"].as<std::string>();
			if (m["x_root"]) sp.x_root = m["x_root"].as<double>();
			if (m["x_tip"]) sp.x_tip = m["x_tip"].as<double>();
			if (m["eta_root"]) sp.eta_root = m["eta_root"].as<double>();
			if (m["eta_tip"]) sp.eta_tip = m["eta_tip"].as<double>();
			if (m["thickness"]) sp.thickness = m["thickness"].as<double>();
			if (m["material"]) sp.material = DecodeMaterial(m["material"]);
			s.spars.push_back(sp);
		}
	}
	if (node["stringers"] && node["stringers"].IsSequence()) {
		for (const auto& m : node["stringers"]) {
			WingStringerSpec st;
			if (m["name"]) st.name = m["name"].as<std::string>();
			if (m["x_root"]) st.x_root = m["x_root"].as<double>();
			if (m["x_tip"]) st.x_tip = m["x_tip"].as<double>();
			if (m["surface"]) st.upper = m["surface"].as<std::string>() != "lower";
			if (m["eta_root"]) st.eta_root = m["eta_root"].as<double>();
			if (m["eta_tip"]) st.eta_tip = m["eta_tip"].as<double>();
			if (m["height"]) st.height = m["height"].as<double>();
			if (m["thickness"]) st.thickness = m["thickness"].as<double>();
			if (m["material"]) st.material = DecodeMaterial(m["material"]);
			s.stringers.push_back(st);
		}
	}
	return s;
}

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
	// Written only when there are any, so a wing without flaps serialises
	// exactly as before.
	if (!spec.flaps.empty()) {
		YAML::Node flaps(YAML::NodeType::Sequence);
		for (const FlapSpec& f : spec.flaps) {
			YAML::Node fn;
			fn["name"] = f.name;
			fn["eta_root"] = f.eta_root;
			fn["eta_tip"] = f.eta_tip;
			fn["hinge_root"] = f.hinge_root;
			fn["hinge_tip"] = f.hinge_tip;
			fn["gap"] = f.gap;
			fn["side_gap"] = f.side_gap;
			flaps.push_back(fn);
		}
		n["flaps"] = flaps;
	}
	n["structure"] = EncodeWingStructure(spec.structure);
	// Written only when set, so an older file reads the same.
	if (spec.symmetric) n["symmetric"] = true;
	return n;
}

WingModelSpec DecodeWingModelSpec(const YAML::Node& node) {
	WingModelSpec spec;
	spec.name = node["name"] ? node["name"].as<std::string>() : "Wing";
	spec.planform = DecodePlanformSpec(node["planform"]);
	spec.reference_images = DecodeReferenceImages(node, spec.planform.semi_span);
	if (node["flaps"] && node["flaps"].IsSequence()) {
		for (const auto& fn : node["flaps"]) {
			FlapSpec f; // every key optional, defaults from FlapSpec
			if (fn["name"]) f.name = fn["name"].as<std::string>();
			if (fn["eta_root"]) f.eta_root = fn["eta_root"].as<double>();
			if (fn["eta_tip"]) f.eta_tip = fn["eta_tip"].as<double>();
			if (fn["hinge_root"]) f.hinge_root = fn["hinge_root"].as<double>();
			if (fn["hinge_tip"]) f.hinge_tip = fn["hinge_tip"].as<double>();
			if (fn["gap"]) f.gap = fn["gap"].as<double>();
			if (fn["side_gap"]) f.side_gap = fn["side_gap"].as<double>();
			spec.flaps.push_back(f);
		}
	}
	// After the flaps: a wing without the key gets defaults that keep the
	// rear spar ahead of its flaps' hinges.
	spec.structure = (node["structure"] && node["structure"].IsMap()) ? DecodeWingStructure(node["structure"]) : WingStructureSpec::DefaultFor(spec.flaps);
	spec.symmetric = node["symmetric"] ? node["symmetric"].as<bool>() : false;
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
	n["structure"] = EncodeFrameStructure(spec.structure);
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
	spec.structure = DecodeFrameStructure(node["structure"]);
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
	n["structure"] = EncodeFrameStructure(spec.structure);
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
	spec.structure = DecodeFrameStructure(node["structure"]);
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
	n["structure"] = EncodeFrameStructure(spec.structure);
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
	spec.structure = DecodeFrameStructure(node["structure"]);
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
	if (s == "lattice") throw std::invalid_argument("Entity type 'lattice' is no longer supported: lattice entities were removed on 2026-11-08 (the structure is shells only: skins and the parameterisations' ribs, spars and frames)");
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
// Mass (`mass:` on an entity or a boolean result)
// ===========================================================================

YAML::Node EncodeMassSpec(const MassSpec& m) {
	// The key says the mode, so the file reads as the dialog does:
	// `mass: {asserted: 12.5}` (the project's mass unit) or
	// `mass: {material: {name: Aluminium 2024-T3, density: 2780, ...}}` (SI,
	// as every material is). The numbers are written beside the name so the
	// file stands on its own -- a later change to a preset's figures does not
	// reach back into saved projects. Unset writes nothing (the caller
	// leaves the key out).
	YAML::Node n;
	if (m.mode == MassMode::Asserted) {
		n["asserted"] = m.mass;
	} else if (m.mode == MassMode::Material) {
		n["material"] = EncodeMaterial(m.material);
		// A skin, in the project's length unit (as typed); absent is solid.
		if (m.skin_thickness > 0.0) n["skin_thickness"] = m.skin_thickness;
	}
	n.SetStyle(YAML::EmitterStyle::Flow);
	return n;
}

MassSpec DecodeMassSpec(const YAML::Node& node) {
	MassSpec m;
	if (!node || !node.IsMap()) return m;
	if (node["asserted"]) {
		m.mode = MassMode::Asserted;
		m.mass = node["asserted"].as<double>();
	} else if (const YAML::Node mat = node["material"]) {
		m.mode = MassMode::Material;
		m.material = DecodeMaterial(mat);
		if (node["skin_thickness"]) m.skin_thickness = node["skin_thickness"].as<double>();
	} else if (node["density"]) {
		// The form before materials existed: a bare density, SI (every
		// project then was metres and kilograms), as a custom material.
		m.mode = MassMode::Material;
		m.material.name = "Custom";
		m.material.density = node["density"].as<double>();
	}
	return m;
}

// ===========================================================================
// Bodies (`bodies:` on an entity or a boolean result): display overrides
// and the structure of each solid body (body_structure.hpp)
// ===========================================================================

namespace {

// A lamina's figures as a flow map (materials.hpp Lamina; the keys of the
// library file, materials.cpp), and back. The optional figures are written
// only when set.
YAML::Node EncodeLamina(const Lamina& l) {
	YAML::Node n;
	n["form"] = l.form;
	n["ply_thickness"] = l.ply_thickness;
	n["e1"] = l.e1;
	n["e2"] = l.e2;
	n["nu12"] = l.nu12;
	n["g12"] = l.g12;
	if (l.g13 > 0.0) n["g13"] = l.g13;
	if (l.g23 > 0.0) n["g23"] = l.g23;
	n["t1"] = l.t1;
	n["c1"] = l.c1;
	n["t2"] = l.t2;
	n["c2"] = l.c2;
	n["s12"] = l.s12;
	if (l.s13 > 0.0) n["s13"] = l.s13;
	if (l.s23 > 0.0) n["s23"] = l.s23;
	if (l.locking_angle > 0.0) n["locking_angle"] = l.locking_angle;
	n.SetStyle(YAML::EmitterStyle::Flow);
	return n;
}

Lamina DecodeLamina(const YAML::Node& n) {
	Lamina l;
	auto num = [&n](const char* key) { return n[key] ? n[key].as<double>() : 0.0; };
	if (n["form"]) l.form = n["form"].as<std::string>();
	l.ply_thickness = num("ply_thickness");
	l.e1 = num("e1"); l.e2 = num("e2"); l.nu12 = num("nu12");
	l.g12 = num("g12"); l.g13 = num("g13"); l.g23 = num("g23");
	l.t1 = num("t1"); l.c1 = num("c1"); l.t2 = num("t2"); l.c2 = num("c2");
	l.s12 = num("s12"); l.s13 = num("s13"); l.s23 = num("s23");
	l.locking_angle = num("locking_angle");
	return l;
}

} // namespace

YAML::Node EncodeMaterial(const Material& m) {
	// The name and the four figures (SI) in flow style, so the file stands
	// on its own when a preset changes. A LAMINATE (2026-11-12) adds its
	// plies, each with the lamina's name AND figures (the file must stand
	// alone here too: a library of another machine may not have the
	// lamina), and its symmetry; a lamina row adds its figures.
	YAML::Node mat;
	mat["name"] = m.name;
	mat["density"] = m.density;
	mat["youngs_modulus"] = m.youngs_modulus;
	mat["poisson_ratio"] = m.poisson_ratio;
	mat["allowable_stress"] = m.allowable_stress;
	if (m.IsLamina()) mat["lamina"] = EncodeLamina(*m.lamina);
	if (m.IsLaminate()) {
		mat["stacking"] = StackingNotation(m);
		mat["symmetric"] = m.symmetric;
		if (m.symmetric_odd) mat["symmetric_odd"] = true;
		YAML::Node plies(YAML::NodeType::Sequence);
		for (const Ply& p : m.plies) {
			YAML::Node pn;
			pn["angle"] = p.angle;
			if (p.thickness > 0.0) pn["thickness"] = p.thickness;
			pn["lamina"] = p.lamina;
			pn["density"] = p.density;
			pn["props"] = EncodeLamina(p.props);
			pn.SetStyle(YAML::EmitterStyle::Flow);
			plies.push_back(pn);
		}
		mat["plies"] = plies;
		mat.SetStyle(YAML::EmitterStyle::Block);
	} else {
		mat.SetStyle(YAML::EmitterStyle::Flow);
	}
	return mat;
}

Material DecodeMaterial(const YAML::Node& mat) {
	Material m;
	if (!mat || !mat.IsMap()) return m;
	if (mat["name"]) m.name = mat["name"].as<std::string>();
	if (mat["density"]) m.density = mat["density"].as<double>();
	if (mat["youngs_modulus"]) m.youngs_modulus = mat["youngs_modulus"].as<double>();
	if (mat["poisson_ratio"]) m.poisson_ratio = mat["poisson_ratio"].as<double>();
	if (mat["allowable_stress"]) m.allowable_stress = mat["allowable_stress"].as<double>();
	if (const YAML::Node l = mat["lamina"]; l && l.IsMap()) {
		m.lamina = DecodeLamina(l);
		if (m.category.empty()) m.category = "Lamina";
	}
	if (const YAML::Node plies = mat["plies"]; plies && plies.IsSequence()) {
		if (mat["symmetric"]) m.symmetric = mat["symmetric"].as<bool>();
		if (mat["symmetric_odd"]) m.symmetric_odd = mat["symmetric_odd"].as<bool>();
		for (const YAML::Node& pn : plies) {
			if (!pn.IsMap()) continue;
			Ply p;
			if (pn["angle"]) p.angle = pn["angle"].as<double>();
			if (pn["thickness"]) p.thickness = pn["thickness"].as<double>();
			if (pn["lamina"]) p.lamina = pn["lamina"].as<std::string>();
			if (pn["density"]) p.density = pn["density"].as<double>();
			if (const YAML::Node lp = pn["props"]; lp && lp.IsMap()) {
				p.props = DecodeLamina(lp);
			} else if (const Material* preset = FindMaterialPreset(p.lamina); preset != nullptr && preset->IsLamina()) {
				// A file written without the figures (by hand): the preset's.
				p.props = *preset->lamina;
				if (!(p.density > 0.0)) p.density = preset->density;
			}
			m.plies.push_back(p);
		}
		if (!m.plies.empty()) {
			m.lamina.reset();
			if (m.category.empty()) m.category = "Laminate";
			RefreshLaminate(m);
		}
	}
	return m;
}

namespace {

YAML::Node EncodeVec3(const Vec3& v) {
	YAML::Node n;
	n.push_back(v.x);
	n.push_back(v.y);
	n.push_back(v.z);
	n.SetStyle(YAML::EmitterStyle::Flow);
	return n;
}

bool DecodeVec3(const YAML::Node& node, Vec3* out) {
	if (!node || !node.IsSequence() || node.size() < 3) return false;
	*out = Vec3{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
	return true;
}

} // namespace

void EncodeBodyStructure(const BodyStructure& s, YAML::Node& into) {
	// A default Material is named "Custom" with no density: nothing to write.
	if (s.material.HasDensity()) into["material"] = EncodeMaterial(s.material);
	if (s.skin_thickness > 0.0) into["skin_thickness"] = s.skin_thickness;
	if (!s.exclusions.empty()) {
		YAML::Node ex(YAML::NodeType::Sequence);
		for (const std::string& name : s.exclusions) ex.push_back(name);
		ex.SetStyle(YAML::EmitterStyle::Flow);
		into["exclusions"] = ex;
	}
	if (s.asserted_mass) into["asserted_mass"] = *s.asserted_mass;
}

BodyStructure DecodeBodyStructure(const YAML::Node& node, bool* had_infill, bool* had_networks) {
	BodyStructure s;
	if (had_infill) *had_infill = false;
	if (had_networks) *had_networks = false;
	if (!node || !node.IsMap()) return s;
	if (node["material"]) s.material = DecodeMaterial(node["material"]);
	if (node["skin_thickness"]) s.skin_thickness = node["skin_thickness"].as<double>();
	// A lattice infill (removed 2026-11-08) and plane networks (removed
	// 2026-11-10): the keys are ignored, not errors, so a file of their
	// time still opens; the caller reports them.
	if (node["infill"] && had_infill) *had_infill = true;
	if (node["networks"] && had_networks) *had_networks = true;
	if (const YAML::Node ex = node["exclusions"]) {
		for (const auto& e : ex) s.exclusions.push_back(e.as<std::string>());
	}
	if (node["asserted_mass"]) s.asserted_mass = node["asserted_mass"].as<double>();
	const std::string why = BodyStructureProblem(s);
	if (!why.empty()) throw std::invalid_argument(why);
	return s;
}

YAML::Node Encode(const ProjectBody& b) {
	YAML::Node n;
	n["name"] = b.name;
	if (b.color) n["color"] = EncodeColor(*b.color);
	if (b.hidden) n["hidden"] = true;
	EncodeBodyStructure(b.structure, n);
	return n;
}

ProjectBody DecodeProjectBody(const YAML::Node& node, bool* had_infill, bool* had_networks) {
	ProjectBody b;
	if (!node || !node.IsMap()) throw std::invalid_argument("a body entry is not a mapping");
	b.name = node["name"] ? node["name"].as<std::string>() : "";
	b.color = DecodeColor(node["color"]);
	b.hidden = node["hidden"] ? node["hidden"].as<bool>() : false;
	try {
		b.structure = DecodeBodyStructure(node, had_infill, had_networks);
	} catch (const std::invalid_argument& e) {
		throw std::invalid_argument("body '" + b.name + "': " + e.what());
	}
	return b;
}

namespace {

YAML::Node EncodeBodies(const std::vector<ProjectBody>& bodies) {
	YAML::Node out(YAML::NodeType::Sequence);
	for (const ProjectBody& body : bodies) {
		if (!body.IsDefault()) out.push_back(Encode(body));
	}
	return out;
}

// `ignored_infill` / `ignored_networks`, when given, collect "<owner> /
// <body>" for every body whose entry carried a legacy `infill:` or
// `networks:` key.
std::vector<ProjectBody> DecodeBodies(const YAML::Node& node, const std::string& owner, std::vector<std::string>* ignored_infill,
	std::vector<std::string>* ignored_networks) {
	std::vector<ProjectBody> out;
	if (!node) return out;
	for (const auto& bn : node) {
		try {
			bool had_infill = false, had_networks = false;
			out.push_back(DecodeProjectBody(bn, &had_infill, &had_networks));
			if (had_infill && ignored_infill) ignored_infill->push_back(owner + " / " + out.back().name);
			if (had_networks && ignored_networks) ignored_networks->push_back(owner + " / " + out.back().name);
		} catch (const std::invalid_argument& e) {
			throw std::invalid_argument("'" + owner + "': " + e.what());
		}
	}
	return out;
}

} // namespace

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
	const YAML::Node bodies = EncodeBodies(b.bodies);
	if (bodies.size() > 0) n["bodies"] = bodies;
	return n;
}

ProjectBoolean DecodeProjectBoolean(const YAML::Node& node, std::vector<std::string>* ignored_infill, std::vector<std::string>* ignored_networks) {
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
	b.bodies = DecodeBodies(node["bodies"], b.name, ignored_infill, ignored_networks);
	return b;
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
	if (e.mirror_normal != std::array<double, 3>{0.0, 1.0, 0.0} || e.mirror_offset != 0.0) {
		YAML::Node plane;
		YAML::Node normal;
		for (double v : e.mirror_normal) normal.push_back(v);
		normal.SetStyle(YAML::EmitterStyle::Flow);
		plane["normal"] = normal;
		plane["offset"] = e.mirror_offset;
		plane.SetStyle(YAML::EmitterStyle::Flow);
		n["mirror_plane"] = plane;
	}
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
	if (!e.mesh.IsDefault()) n["mesh"] = EncodeMeshSpec(e.mesh);
	const YAML::Node bodies = EncodeBodies(e.bodies);
	if (bodies.size() > 0) n["bodies"] = bodies;
	return n;
}

ProjectEntity DecodeEntity(const YAML::Node& node, std::vector<std::string>* ignored_infill, std::vector<std::string>* ignored_networks) {
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
	if (const YAML::Node& plane = node["mirror_plane"]; plane && plane.IsMap()) {
		if (plane["normal"] && plane["normal"].IsSequence() && plane["normal"].size() == 3) {
			for (size_t i = 0; i < 3; ++i) e.mirror_normal[i] = plane["normal"][i].as<double>();
		}
		if (plane["offset"]) e.mirror_offset = plane["offset"].as<double>();
		if (e.mirror_normal[0] == 0.0 && e.mirror_normal[1] == 0.0 && e.mirror_normal[2] == 0.0) {
			throw std::invalid_argument("Entity '" + e.name + "': the mirror plane's normal is a zero vector.");
		}
	}
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
	}
	if (node["color"] && node["color"].size() >= 3) {
		const YAML::Node& c = node["color"];
		e.color = std::array<double, 3>{c[0].as<double>(), c[1].as<double>(), c[2].as<double>()};
	}
	e.hidden = node["hidden"] ? node["hidden"].as<bool>() : false;
	e.mass = DecodeMassSpec(node["mass"]);
	e.mesh = DecodeMeshSpec(node["mesh"]);
	e.bodies = DecodeBodies(node["bodies"], e.name, ignored_infill, ignored_networks);
	return e;
}

} // namespace

YAML::Node EncodeMeshSpec(const MeshSpec& m) {
	YAML::Node n;
	if (m.stations > 0) n["stations"] = m.stations;
	if (m.around > 0) n["around"] = m.around;
	if (!m.refine) n["refine"] = false;
	if (m.tolerance > 0.0) n["tolerance"] = m.tolerance;
	if (m.max_aspect != 0.0) n["max_aspect"] = m.max_aspect;
	if (m.cells != std::array<int, 3>{0, 0, 0}) {
		YAML::Node cells(YAML::NodeType::Sequence);
		for (int c : m.cells) cells.push_back(c);
		cells.SetStyle(YAML::EmitterStyle::Flow);
		n["cells"] = cells;
	}
	if (m.cap_rings > 0) n["cap_rings"] = m.cap_rings;
	if (m.chordwise > 0) n["chordwise"] = m.chordwise;
	if (m.spanwise > 0) n["spanwise"] = m.spanwise;
	n.SetStyle(YAML::EmitterStyle::Flow);
	return n;
}

MeshSpec DecodeMeshSpec(const YAML::Node& node) {
	MeshSpec m;
	if (!node || !node.IsMap()) return m;
	if (node["stations"]) m.stations = std::max(node["stations"].as<int>(), 0);
	if (node["around"]) m.around = std::max(node["around"].as<int>(), 0);
	if (node["refine"]) m.refine = node["refine"].as<bool>();
	if (node["tolerance"]) m.tolerance = std::max(node["tolerance"].as<double>(), 0.0);
	if (node["max_aspect"]) m.max_aspect = node["max_aspect"].as<double>();
	if (node["cells"] && node["cells"].IsSequence()) {
		int i = 0;
		for (const auto& c : node["cells"]) { if (i < 3) m.cells[i++] = std::max(c.as<int>(), 0); }
	}
	if (node["cap_rings"]) m.cap_rings = std::max(node["cap_rings"].as<int>(), 0);
	if (node["chordwise"]) m.chordwise = std::max(node["chordwise"].as<int>(), 0);
	if (node["spanwise"]) m.spanwise = std::max(node["spanwise"].as<int>(), 0);
	return m;
}

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
		if (units["stress"]) project.units.stress = ParseStressUnit(units["stress"].as<std::string>());
	}
	// The model scale: absent means full size. Anything but a positive
	// number is refused -- a replica at zero or negative size is not a
	// misreading worth guessing at.
	if (const YAML::Node scale = root["scale"]) {
		project.scale = scale.as<double>();
		if (!(std::isfinite(project.scale) && project.scale > 0.0)) {
			throw std::invalid_argument("scale: the model scale must be a positive number");
		}
	}
	if (root["entities"]) {
		for (const auto& en : root["entities"]) project.entities.push_back(DecodeEntity(en, &project.ignored_infill, &project.ignored_networks));
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
			ProjectBoolean b = DecodeProjectBoolean(bn, &project.ignored_infill, &project.ignored_networks);
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
	// The lattice entities of an older file: not decoded (lattices are no
	// longer supported at all), named so the application can say what it
	// skipped.
	if (root["lattices"]) {
		for (const auto& ln : root["lattices"]) {
			project.unsupported_lattices.push_back(ln.IsMap() && ln["name"] ? ln["name"].as<std::string>() : std::string("(unnamed lattice)"));
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
	units["stress"] = std::string(StressUnitName(project.units.stress));
	units.SetStyle(YAML::EmitterStyle::Flow);
	root["units"] = units;
	// The model scale, only when the vehicle is not built full size, so a
	// file saved at 1:1 reads as it always did. Refused rather than written
	// when it is not a positive number: the file must open again.
	if (project.scale != 1.0) {
		if (!(std::isfinite(project.scale) && project.scale > 0.0)) {
			throw std::invalid_argument("scale: the model scale must be a positive number");
		}
		root["scale"] = project.scale;
	}
	YAML::Node entities(YAML::NodeType::Sequence);
	for (const auto& e : project.entities) entities.push_back(EncodeEntity(e));
	root["entities"] = entities;
	if (!project.booleans.empty()) {
		YAML::Node booleans(YAML::NodeType::Sequence);
		for (const auto& b : project.booleans) booleans.push_back(Encode(b));
		root["booleans"] = booleans;
	}
	std::ofstream out(path);
	if (!out) throw std::runtime_error("could not open " + path + " for writing");
	out << YAML::Dump(root);
}

} // namespace aerotelic::project_yaml
