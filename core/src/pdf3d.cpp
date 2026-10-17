#include "aerotelic/pdf3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <unordered_map>
#include <vector>

namespace aerotelic {
namespace {

// ---------------------------------------------------------------- bytes

// A little-endian field writer. Every U3D "uncompressed" write in
// ECMA-363 10.4.8 is the byte's bits reversed, written most significant
// first, into a bit stream that fills each 32-bit word from its least
// significant bit -- which composes to the plain little-endian byte, so
// an uncompressed-profile U3D data section is exactly this.
class Bytes {
public:
	void U8(std::uint32_t v) { b_.push_back(static_cast<std::uint8_t>(v & 0xFFu)); }
	void U16(std::uint32_t v) { U8(v); U8(v >> 8); }
	void U32(std::uint32_t v) { U16(v & 0xFFFFu); U16(v >> 16); }
	void U64(std::uint64_t v) { U32(static_cast<std::uint32_t>(v & 0xFFFFFFFFull)); U32(static_cast<std::uint32_t>(v >> 32)); }
	void I16(int v) { U16(static_cast<std::uint32_t>(static_cast<std::uint16_t>(static_cast<std::int16_t>(v)))); }
	void I32(int v) { U32(static_cast<std::uint32_t>(v)); }

	void F32(double v) {
		const float f = static_cast<float>(v);
		std::uint32_t bits;
		std::memcpy(&bits, &f, 4);
		U32(bits);
	}

	// ECMA-363 10.4.2: a U16 count of bytes, then the bytes. The file
	// header declares UTF-8; names here are made ASCII by SafeName().
	void Str(const std::string& s) {
		const std::size_t n = s.size() > 0xFFFFu ? 0xFFFFu : s.size();
		U16(static_cast<std::uint32_t>(n));
		for (std::size_t i = 0; i < n; ++i) U8(static_cast<std::uint8_t>(s[i]));
	}

	void Pad4() { while ((b_.size() & 3u) != 0u) U8(0); }
	void Raw(const std::vector<std::uint8_t>& v) { b_.insert(b_.end(), v.begin(), v.end()); }

	std::size_t Size() const { return b_.size(); }
	const std::vector<std::uint8_t>& Data() const { return b_; }
	std::vector<std::uint8_t>& Data() { return b_; }

private:
	std::vector<std::uint8_t> b_;
};

// ECMA-363 9.2: block type, data size, meta data size, the data, then
// zero to three zero bytes so the next block starts 32-bit aligned. No
// meta data is written, so its size is zero and it needs no padding.
void AppendBlock(Bytes& out, std::uint32_t block_type, const Bytes& data) {
	out.U32(block_type);
	out.U32(static_cast<std::uint32_t>(data.Size()));
	out.U32(0);
	out.Raw(data.Data());
	out.Pad4();
}

// ------------------------------------------------- welded mesh for U3D

// U3D indexes positions and normals separately, with an index per corner,
// so both arrays are welded: the loft shares its vertices exactly and
// whole runs of coplanar faces share a normal.
struct Welded {
	std::vector<float> positions;    // 3 per entry
	std::vector<float> normals;      // 3 per entry
	// Per face, three (position, normal) pairs. Every part is its own mesh
	// generator with a single shading description, so there is no shading
	// index to carry here.
	struct Face { std::uint32_t pos[3]; std::uint32_t nrm[3]; };
	std::vector<Face> faces;
};

std::uint64_t KeyOf(float x, float y, float z) {
	// -0.0f and 0.0f must land on the same key.
	if (x == 0.0f) x = 0.0f;
	if (y == 0.0f) y = 0.0f;
	if (z == 0.0f) z = 0.0f;
	std::uint32_t a, b, c;
	std::memcpy(&a, &x, 4);
	std::memcpy(&b, &y, 4);
	std::memcpy(&c, &z, 4);
	// 64 bits of a 96-bit key: collisions are resolved by comparing the
	// stored triple, so this only has to spread well.
	std::uint64_t h = 1469598103934665603ull;
	for (std::uint32_t w : {a, b, c}) {
		h ^= w;
		h *= 1099511628211ull;
	}
	return h;
}

// Welds a float triple into `values`, returning its index.
class Welder {
public:
	explicit Welder(std::vector<float>& values) : v_(values) {}

	std::uint32_t Add(float x, float y, float z) {
		if (x == 0.0f) x = 0.0f;
		if (y == 0.0f) y = 0.0f;
		if (z == 0.0f) z = 0.0f;
		const std::uint64_t key = KeyOf(x, y, z);
		auto range = map_.equal_range(key);
		for (auto it = range.first; it != range.second; ++it) {
			const std::size_t i = it->second * 3;
			if (v_[i] == x && v_[i + 1] == y && v_[i + 2] == z) return static_cast<std::uint32_t>(it->second);
		}
		const std::size_t index = v_.size() / 3;
		v_.push_back(x);
		v_.push_back(y);
		v_.push_back(z);
		map_.emplace(key, index);
		return static_cast<std::uint32_t>(index);
	}

private:
	std::vector<float>& v_;
	std::unordered_multimap<std::uint64_t, std::size_t> map_;
};

// Per-corner normals with a crease angle: a corner's normal is the
// area-weighted average of the normals of the faces around that vertex
// that lie within the crease angle of the corner's own face, so a
// curved surface shades smoothly while a trailing edge or the rim of an
// end cap keeps its edge. (U3D would do its own merging from the
// declaration's Normal Crease Parameter, which is written to match, but
// doing it here makes the shading in the file explicit rather than a
// property of whichever reader opens it.)
Welded WeldPart(const TriMesh& m, double crease_angle_deg) {
	Welded w;
	Welder pos(w.positions);
	Welder nrm(w.normals);
	const double cos_crease = std::cos(crease_angle_deg * 3.14159265358979323846 / 180.0);

	// Face normals (unit) and areas; a degenerate triangle has no normal
	// and is dropped.
	std::vector<Vec3> fn(m.triangles.size());
	std::vector<double> area(m.triangles.size(), 0.0);
	for (std::size_t t = 0; t < m.triangles.size(); ++t) {
		const auto& tri = m.triangles[t];
		if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0) continue;
		const int n_v = static_cast<int>(m.vertices.size());
		if (tri[0] >= n_v || tri[1] >= n_v || tri[2] >= n_v) continue;
		const Vec3 n = Cross(m.vertices[tri[1]] - m.vertices[tri[0]], m.vertices[tri[2]] - m.vertices[tri[0]]);
		const double len = Length(n);
		if (!(len > 0.0)) continue;
		fn[t] = n / len;
		area[t] = 0.5 * len;
	}

	// The faces around each vertex.
	std::vector<std::vector<int>> around(m.vertices.size());
	for (std::size_t t = 0; t < m.triangles.size(); ++t) {
		if (area[t] <= 0.0) continue;
		for (int k = 0; k < 3; ++k) around[m.triangles[t][k]].push_back(static_cast<int>(t));
	}

	for (std::size_t t = 0; t < m.triangles.size(); ++t) {
		if (area[t] <= 0.0) continue;
		const auto& tri = m.triangles[t];
		Welded::Face f;
		for (int k = 0; k < 3; ++k) {
			const Vec3& v = m.vertices[tri[k]];
			f.pos[k] = pos.Add(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));

			Vec3 sum{0.0, 0.0, 0.0};
			for (int fi : around[tri[k]]) {
				if (Dot(fn[fi], fn[t]) >= cos_crease) sum = sum + fn[fi] * area[fi];
			}
			const Vec3 n = Length(sum) > 0.0 ? Normalized(sum) : fn[t];
			f.nrm[k] = nrm.Add(static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z));
		}
		w.faces.push_back(f);
	}
	return w;
}

// U3D object names: ASCII, no quotes, never empty, never repeated.
std::string SafeName(const std::string& in, const std::string& fallback) {
	std::string s;
	for (char c : in) {
		const unsigned char u = static_cast<unsigned char>(c);
		s.push_back(u >= 0x20 && u < 0x7F ? c : '_');
	}
	while (!s.empty() && s.back() == ' ') s.pop_back();
	return s.empty() ? fallback : s;
}

// ------------------------------------------------------------ U3D blocks

constexpr std::uint32_t kBlockFileHeader = 0x00443355u;
constexpr std::uint32_t kBlockModifierChain = 0xFFFFFF14u;
constexpr std::uint32_t kBlockModelNode = 0xFFFFFF22u;
constexpr std::uint32_t kBlockCLODMeshDeclaration = 0xFFFFFF31u;
constexpr std::uint32_t kBlockCLODBaseMesh = 0xFFFFFF3Bu;
constexpr std::uint32_t kBlockShadingModifier = 0xFFFFFF45u;
constexpr std::uint32_t kBlockLitTextureShader = 0xFFFFFF53u;
constexpr std::uint32_t kBlockMaterialResource = 0xFFFFFF54u;

// ECMA-363 9.4.1.2: base profile plus "no compression mode", under which
// every field the block definitions mark compressed is written as the
// plain value of its type. Readers are required to support it.
constexpr std::uint32_t kProfileNoCompression = 0x00000004u;
// IANA MIBenum for UTF-8 (9.4.1.5).
constexpr std::uint32_t kCharacterEncodingUtf8 = 106u;

}  // namespace

// Every part becomes its own mesh generator, model node, shader and
// material, so Acrobat's model tree has one named, separately hideable
// row per body rather than one lump -- the same reason Export STEP
// offers "Write named parts".
std::vector<std::uint8_t> WriteU3d(const std::vector<Pdf3dPart>& parts, double crease_angle_deg, Pdf3dResult* out_stats) {
	Pdf3dResult stats;

	// Names are made ASCII (the palettes are keyed by name) and made
	// unique: two bodies may well carry the same label in the scene, and
	// the second would otherwise replace the first in the palette.
	std::vector<std::string> names;
	names.reserve(parts.size());
	for (std::size_t i = 0; i < parts.size(); ++i) {
		char fallback[32];
		std::snprintf(fallback, sizeof fallback, "Body %zu", i + 1);
		std::string name = SafeName(parts[i].name, fallback);
		if (std::find(names.begin(), names.end(), name) != names.end()) {
			char suffix[24];
			std::snprintf(suffix, sizeof suffix, " (%zu)", i + 1);
			name += suffix;
		}
		names.push_back(name);
	}

	Bytes declarations, continuations;

	for (std::size_t i = 0; i < parts.size(); ++i) {
		const Welded w = WeldPart(parts[i].mesh, crease_angle_deg);
		if (w.faces.empty()) continue;  // nothing to draw: no node for it either

		const std::string& mesh_name = names[i];
		const std::string shader_name = mesh_name + " shader";
		const std::string material_name = mesh_name + " material";
		const std::uint32_t n_positions = static_cast<std::uint32_t>(w.positions.size() / 3);
		const std::uint32_t n_normals = static_cast<std::uint32_t>(w.normals.size() / 3);
		const std::uint32_t n_faces = static_cast<std::uint32_t>(w.faces.size());

		++stats.parts;
		stats.positions += n_positions;
		stats.normals += n_normals;
		stats.faces += n_faces;

		// ---- the model resource chain: the mesh generator, then its shading

		Bytes decl;  // CLOD Mesh Declaration (9.6.1.1)
		decl.Str(mesh_name);
		decl.U32(0);  // chain index: the generator is first in the chain
		decl.U32(0);  // mesh attributes: the faces carry normal indices
		decl.U32(n_faces);
		decl.U32(n_positions);
		decl.U32(n_normals);
		decl.U32(0);  // diffuse colour count
		decl.U32(0);  // specular colour count
		decl.U32(0);  // texture coord count
		decl.U32(1);  // shading count: one shader list for the whole body
		decl.U32(0);  // shading attributes: no per-vertex colours
		decl.U32(0);  // texture layer count (so no Texture Coord Dimensions follow)
		decl.U32(0);  // original shading id
		// CLOD description: the base mesh is the whole mesh, so minimum and
		// maximum resolution are equal and no progressive updates follow.
		decl.U32(n_positions);
		decl.U32(n_positions);
		// Resource description. The quality factors are informational
		// (9.6.1.1.5.1) and the inverse quantization factors only
		// reconstruct quantized progressive updates, of which there are
		// none -- the base mesh is never quantized (9.6.1.2).
		decl.U32(1000); decl.U32(1000); decl.U32(1000);
		decl.F32(1.0); decl.F32(1.0); decl.F32(1.0); decl.F32(1.0); decl.F32(1.0);
		// Normal crease / update / tolerance: the crease matches the angle
		// the corner normals above were averaged over, so a reader that
		// merges normals itself reaches the same shading.
		decl.F32(std::cos(crease_angle_deg * 3.14159265358979323846 / 180.0));
		decl.F32(0.0);
		decl.F32(0.0);
		decl.U32(0);  // bone count: no skeleton

		Bytes shade;  // Shading Modifier (9.7.5)
		shade.Str(mesh_name);  // must match the chain's name (9.4.3.8)
		shade.U32(1);  // chain index: after the mesh generator
		shade.U32(1);  // shading attributes: applies to the renderable mesh group
		shade.U32(1);  // one shader list
		shade.U32(1);  // one shader in it
		shade.Str(shader_name);

		Bytes chain;  // Modifier Chain (9.4.3)
		chain.Str(mesh_name);
		chain.U32(1);  // a model resource modifier chain
		chain.U32(0);  // no bounding sphere, no bounding box
		chain.Pad4();  // 9.4.3.6: aligns Modifier Count and the nested blocks
		chain.U32(2);
		AppendBlock(chain, kBlockCLODMeshDeclaration, decl);
		AppendBlock(chain, kBlockShadingModifier, shade);

		// ---- the shader and the material it names

		Bytes shader;  // Lit Texture Shader (9.8.3)
		shader.Str(shader_name);
		shader.U32(1);           // lighting enabled
		shader.F32(0.0);         // alpha test reference
		shader.U32(0x00000617);  // alpha test function: ALWAYS
		shader.U32(0x00000606);  // colour blend: FB_ALPHA_BLEND
		shader.U32(1);           // used in render pass 0
		shader.U32(0);           // shader channels: no texture layers
		shader.U32(0);           // alpha texture channels
		shader.Str(material_name);

		const double* c = parts[i].colour;
		const double opacity = parts[i].opacity < 0.0 ? 0.0 : (parts[i].opacity > 1.0 ? 1.0 : parts[i].opacity);
		Bytes material;  // Material Resource (9.8.4)
		material.Str(material_name);
		material.U32(0x0000003F);  // ambient | diffuse | specular | emissive | reflectivity | opacity
		// Ambient is a fraction of the body's own colour rather than grey,
		// so a body still reads as its scene colour where the viewer's
		// lighting rig does not reach it.
		material.F32(c[0] * 0.25); material.F32(c[1] * 0.25); material.F32(c[2] * 0.25);
		material.F32(c[0]); material.F32(c[1]); material.F32(c[2]);
		material.F32(0.18); material.F32(0.18); material.F32(0.18);  // a neutral highlight
		material.F32(0.0); material.F32(0.0); material.F32(0.0);     // emissive
		material.F32(0.1);                                           // reflectivity
		material.F32(opacity);

		// ---- the node that puts this body in the scene

		Bytes node;  // Model Node (9.5.2)
		node.Str(mesh_name);
		node.U32(1);   // one parent
		node.Str("");  // the default group node of the node palette
		// The parent transform is the identity: the meshes arrive placed in
		// model coordinates, exactly as the STL writer takes them.
		for (int k = 0; k < 16; ++k) node.F32((k % 5) == 0 ? 1.0 : 0.0);
		node.Str(mesh_name);  // the model resource chain above
		node.U32(3);          // front and back faces visible

		// ---- the base mesh: this body's one continuation block

		Bytes base;  // CLOD Base Mesh Continuation (9.6.1.2)
		base.Str(mesh_name);
		base.U32(0);  // chain index
		base.U32(n_faces);
		base.U32(n_positions);
		base.U32(n_normals);
		base.U32(0); base.U32(0); base.U32(0);  // no diffuse, specular or texture coords
		for (float v : w.positions) base.F32(v);
		for (float v : w.normals) base.F32(v);
		for (const Welded::Face& f : w.faces) {
			base.U32(0);  // shading id: the single shader list
			for (int k = 0; k < 3; ++k) {
				base.U32(f.pos[k]);
				base.U32(f.nrm[k]);
			}
		}

		AppendBlock(declarations, kBlockModifierChain, chain);
		AppendBlock(declarations, kBlockLitTextureShader, shader);
		AppendBlock(declarations, kBlockMaterialResource, material);
		AppendBlock(declarations, kBlockModelNode, node);
		AppendBlock(continuations, kBlockCLODBaseMesh, base);
	}

	if (stats.faces == 0) {
		if (out_stats) *out_stats = stats;
		return {};
	}

	// ---- the file: the header, the declarations, then the continuations

	// The header's own data is 24 bytes (two I16, three U32 and one U64)
	// and its block header 12 more. No units scaling factor: the
	// defined-units profile bit is unset, as a model carries no unit here
	// any more than an STL does.
	const std::uint32_t header_block_size = 12u + 24u;
	const std::uint32_t declaration_size = header_block_size + static_cast<std::uint32_t>(declarations.Size());
	const std::uint64_t file_size = declaration_size + continuations.Size();

	Bytes header;
	header.I16(0);  // major version
	header.I16(0);  // minor version
	header.U32(kProfileNoCompression);
	header.U32(declaration_size);
	header.U64(file_size);
	header.U32(kCharacterEncodingUtf8);

	Bytes file;
	AppendBlock(file, kBlockFileHeader, header);
	file.Raw(declarations.Data());
	file.Raw(continuations.Data());

	stats.u3d_bytes = file.Size();
	if (out_stats) *out_stats = stats;
	return file.Data();
}

namespace {

// ------------------------------------------------------------------ PDF

// PDF has no exponent notation for numbers (ISO 32000-1 7.3.3), so every
// real is written fixed and trimmed.
std::string Num(double v) {
	if (!std::isfinite(v)) v = 0.0;
	if (std::fabs(v) < 1e-6) return "0";
	char buf[64];
	std::snprintf(buf, sizeof buf, "%.6f", v);
	std::string s(buf);
	const std::size_t dot = s.find('.');
	if (dot != std::string::npos) {
		while (!s.empty() && s.back() == '0') s.pop_back();
		if (!s.empty() && s.back() == '.') s.pop_back();
	}
	return s.empty() ? "0" : s;
}

// A literal string in a content stream or dictionary.
std::string PdfString(const std::string& in) {
	std::string s = "(";
	for (char c : in) {
		if (c == '(' || c == ')' || c == '\\') s.push_back('\\');
		const unsigned char u = static_cast<unsigned char>(c);
		s.push_back(u >= 0x20 && u < 0x7F ? c : ' ');
	}
	s.push_back(')');
	return s;
}

std::string NowAsPdfDate() {
	const std::time_t t = std::time(nullptr);
	std::tm local{};
#if defined(_WIN32)
	localtime_s(&local, &t);
#else
	localtime_r(&t, &local);
#endif
	char buf[64];
	std::snprintf(buf, sizeof buf, "D:%04d%02d%02d%02d%02d%02d",
		local.tm_year + 1900, local.tm_mon + 1, local.tm_mday, local.tm_hour, local.tm_min, local.tm_sec);
	return buf;
}

// Collects the objects and their byte offsets, then writes the xref.
class PdfFile {
public:
	// Reserves an object number, to be filled in later with Begin().
	int Reserve() {
		offsets_.push_back(0);
		return static_cast<int>(offsets_.size());  // object numbers are 1-based
	}

	void Begin(int object) {
		offsets_[object - 1] = body_.size();
		Append(std::to_string(object) + " 0 obj\n");
	}

	void End() { Append("endobj\n"); }

	void Append(const std::string& s) { body_.insert(body_.end(), s.begin(), s.end()); }
	void Append(const std::vector<std::uint8_t>& v) { body_.insert(body_.end(), v.begin(), v.end()); }

	// A stream object: the dictionary entries (without << >>), then the data.
	void Stream(int object, const std::string& entries, const std::vector<std::uint8_t>& data) {
		Begin(object);
		Append("<< " + entries + " /Length " + std::to_string(data.size()) + " >>\nstream\n");
		Append(data);
		Append("\nendstream\n");
		End();
	}

	std::vector<std::uint8_t> Finish(int root, int info) {
		std::vector<std::uint8_t> out;
		const std::string head = "%PDF-1.7\n%\xE2\xE3\xCF\xD3\n";
		out.insert(out.end(), head.begin(), head.end());
		const std::size_t base = out.size();
		out.insert(out.end(), body_.begin(), body_.end());

		const std::size_t xref = out.size();
		std::string x = "xref\n0 " + std::to_string(offsets_.size() + 1) + "\n0000000000 65535 f \n";
		for (std::size_t off : offsets_) {
			char buf[32];
			std::snprintf(buf, sizeof buf, "%010zu 00000 n \n", base + off);
			x += buf;
		}
		x += "trailer\n<< /Size " + std::to_string(offsets_.size() + 1) +
			" /Root " + std::to_string(root) + " 0 R /Info " + std::to_string(info) + " 0 R >>\n" +
			"startxref\n" + std::to_string(xref) + "\n%%EOF\n";
		out.insert(out.end(), x.begin(), x.end());
		return out;
	}

private:
	std::vector<std::uint8_t> body_;
	std::vector<std::size_t> offsets_;
};

std::vector<std::uint8_t> AsBytes(const std::string& s) { return std::vector<std::uint8_t>(s.begin(), s.end()); }

}  // namespace

bool WritePdf3d(const std::vector<Pdf3dPart>& parts, const std::string& path, const Pdf3dOptions& options,
	std::string& error, Pdf3dResult* out_stats) {
	error.clear();
	Pdf3dResult stats;

	const std::vector<std::uint8_t> u3d = WriteU3d(parts, options.crease_angle_deg, &stats);
	if (u3d.empty()) {
		error = "the bodies carry no triangulation";
		if (out_stats) *out_stats = stats;
		return false;
	}

	// ---- the camera: the bounding sphere of everything, in the field of view

	Box3 bounds;
	for (const Pdf3dPart& p : parts) {
		for (const Vec3& v : p.mesh.vertices) bounds.Add(v);
	}
	const Vec3 centre = bounds.IsEmpty() ? Vec3{0.0, 0.0, 0.0} : bounds.Center();
	double radius = 0.5 * bounds.Diagonal();
	if (!(radius > 0.0)) radius = 1.0;

	const double kPi = 3.14159265358979323846;
	double fov = options.field_of_view_deg;
	if (!(fov > 1.0) || fov > 150.0) fov = 30.0;
	const double fit = options.fit > 0.0 ? options.fit : 1.0;
	const double distance = fit * radius / std::sin(0.5 * fov * kPi / 180.0);

	// The camera sits at the origin of camera space looking down its own
	// -Z, so +Z of the camera points back along the view direction.
	Vec3 zc = options.view_from;
	if (!(Dot(zc, zc) > 0.0)) zc = Vec3{1.0, -1.0, 0.55};
	zc = zc / std::sqrt(Dot(zc, zc));
	Vec3 up = options.up;
	if (!(Dot(up, up) > 0.0)) up = Vec3{0.0, 0.0, 1.0};
	Vec3 xc = Cross(up, zc);
	if (std::sqrt(Dot(xc, xc)) < 1e-9) {
		// Looking straight along the up axis: any perpendicular will do.
		xc = Cross(Vec3{1.0, 0.0, 0.0}, zc);
		if (std::sqrt(Dot(xc, xc)) < 1e-9) xc = Cross(Vec3{0.0, 1.0, 0.0}, zc);
	}
	xc = xc / std::sqrt(Dot(xc, xc));
	Vec3 yc = Cross(zc, xc);
	const Vec3 eye = centre + zc * distance;

	// ---- the page

	const double page_w = options.page_width > 72.0 ? options.page_width : 595.276;
	const double page_h = options.page_height > 72.0 ? options.page_height : 841.890;
	const double margin = options.margin >= 0.0 ? options.margin : 36.0;
	const double heading_h = options.title.empty() ? 0.0 : 26.0;
	const double caption_h = options.caption.empty() ? 0.0 : 18.0;
	const double rx0 = margin;
	const double ry0 = margin + caption_h;
	const double rx1 = page_w - margin;
	const double ry1 = page_h - margin - heading_h;
	if (!(rx1 > rx0 + 1.0) || !(ry1 > ry0 + 1.0)) {
		error = "the page is too small for the margins asked";
		if (out_stats) *out_stats = stats;
		return false;
	}

	PdfFile pdf;
	const int obj_catalog = pdf.Reserve();
	const int obj_pages = pdf.Reserve();
	const int obj_page = pdf.Reserve();
	const int obj_3d = pdf.Reserve();
	const int obj_view = pdf.Reserve();
	const int obj_annot = pdf.Reserve();
	const int obj_appearance = pdf.Reserve();
	const int obj_content = pdf.Reserve();
	const int obj_font = pdf.Reserve();
	const int obj_info = pdf.Reserve();

	pdf.Begin(obj_catalog);
	pdf.Append("<< /Type /Catalog /Pages " + std::to_string(obj_pages) + " 0 R >>\n");
	pdf.End();

	pdf.Begin(obj_pages);
	pdf.Append("<< /Type /Pages /Kids [" + std::to_string(obj_page) + " 0 R] /Count 1 >>\n");
	pdf.End();

	pdf.Begin(obj_page);
	pdf.Append("<< /Type /Page /Parent " + std::to_string(obj_pages) + " 0 R"
		" /MediaBox [0 0 " + Num(page_w) + " " + Num(page_h) + "]"
		" /Resources << /Font << /F1 " + std::to_string(obj_font) + " 0 R >> >>"
		" /Contents " + std::to_string(obj_content) + " 0 R"
		" /Annots [" + std::to_string(obj_annot) + " 0 R] >>\n");
	pdf.End();

	// The 3D stream. /VA carries the one view and /DV selects it, so the
	// page opens on the camera computed above rather than the reader's
	// guess.
	{
		std::vector<std::uint8_t> data = u3d;
		std::string entries = "/Type /3D /Subtype /U3D"
			" /VA [" + std::to_string(obj_view) + " 0 R] /DV 0";
		if (options.deflate) {
			std::vector<std::uint8_t> packed;
			if (options.deflate(u3d, packed) && !packed.empty() && packed.size() < u3d.size()) {
				data = std::move(packed);
				entries += " /Filter /FlateDecode";
				stats.u3d_deflated = true;
			}
		}
		pdf.Stream(obj_3d, entries, data);
	}

	pdf.Begin(obj_view);
	pdf.Append("<< /Type /3DView /XN " + PdfString("Default") + " /IN " + PdfString("Default") +
		" /MS /M /C2W [" +
		Num(xc.x) + " " + Num(xc.y) + " " + Num(xc.z) + " " +
		Num(yc.x) + " " + Num(yc.y) + " " + Num(yc.z) + " " +
		Num(zc.x) + " " + Num(zc.y) + " " + Num(zc.z) + " " +
		Num(eye.x) + " " + Num(eye.y) + " " + Num(eye.z) + "]"
		" /CO " + Num(distance) +
		" /P << /Subtype /P /FOV " + Num(fov) + " >>"
		" /BG << /Type /3DBG /Subtype /SC /CS /DeviceRGB /C [" +
		Num(options.background[0]) + " " + Num(options.background[1]) + " " + Num(options.background[2]) + "] >>"
		" /LS << /Type /3DLightingScheme /Subtype /" + (options.lighting.empty() ? "CAD" : options.lighting) + " >>"
		" /RM << /Type /3DRenderMode /Subtype /" + (options.render_mode.empty() ? "Solid" : options.render_mode) + " >>"
		" >>\n");
	pdf.End();

	// The annotation. /3DA activates the model when the page opens, so the
	// reader shows the live model rather than a poster to click; /3DI
	// keeps it interactive once the pointer leaves it. The keys and their
	// values follow what a known-good 3D PDF carries (a SOLIDWORKS export,
	// which differs only in writing PRC where this writes U3D).
	pdf.Begin(obj_annot);
	pdf.Append("<< /Type /Annot /Subtype /3D /F 4 /Border [0 0 0]"
		" /Contents " + PdfString(options.title.empty() ? std::string("3D model") : options.title) +
		" /Rect [" + Num(rx0) + " " + Num(ry0) + " " + Num(rx1) + " " + Num(ry1) + "]"
		" /3DD " + std::to_string(obj_3d) + " 0 R"
		" /3DV " + std::to_string(obj_view) + " 0 R"
		" /3DI true"
		" /3DA << /Type /3DActivation /A /PO /AIS /L /D /PC /DIS /I /TB true /NP false >>"
		" /AP << /N " + std::to_string(obj_appearance) + " 0 R >>"
		" /P " + std::to_string(obj_page) + " 0 R >>\n");
	pdf.End();

	// The appearance the reader replaces with the live model (and prints,
	// if the model was never activated): the frame alone.
	{
		const double w = rx1 - rx0, h = ry1 - ry0;
		const std::string s = "0.6 G 0.5 w 0.25 0.25 " + Num(w - 0.5) + " " + Num(h - 0.5) + " re S\n";
		pdf.Stream(obj_appearance, "/Type /XObject /Subtype /Form /FormType 1"
			" /BBox [0 0 " + Num(w) + " " + Num(h) + "] /Resources << >>", AsBytes(s));
	}

	{
		std::string s;
		if (!options.title.empty()) {
			s += "BT /F1 14 Tf 1 0 0 1 " + Num(rx0) + " " + Num(page_h - margin - 16.0) + " Tm " +
				PdfString(options.title) + " Tj ET\n";
		}
		if (!options.caption.empty()) {
			s += "BT /F1 8 Tf 1 0 0 1 " + Num(rx0) + " " + Num(margin + 5.0) + " Tm " +
				PdfString(options.caption) + " Tj ET\n";
		}
		pdf.Stream(obj_content, "", AsBytes(s));
	}

	pdf.Begin(obj_font);
	pdf.Append("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>\n");
	pdf.End();

	pdf.Begin(obj_info);
	pdf.Append("<< /Title " + PdfString(options.title.empty() ? std::string("Aerotelic model") : options.title) +
		" /Creator " + PdfString("Aerotelic") +
		" /Producer " + PdfString("Aerotelic 3D PDF export (U3D, ECMA-363)") +
		" /CreationDate " + PdfString(NowAsPdfDate()) + " >>\n");
	pdf.End();

	const std::vector<std::uint8_t> out = pdf.Finish(obj_catalog, obj_info);
	stats.pdf_bytes = out.size();

	std::FILE* f = nullptr;
#if defined(_WIN32)
	if (fopen_s(&f, path.c_str(), "wb") != 0) f = nullptr;
#else
	f = std::fopen(path.c_str(), "wb");
#endif
	if (f == nullptr) {
		error = "could not open " + path + " for writing";
		if (out_stats) *out_stats = stats;
		return false;
	}
	const bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
	std::fclose(f);
	if (!ok) error = "could not write " + path;
	if (out_stats) *out_stats = stats;
	return ok;
}

}  // namespace aerotelic
