// Checks the 3D PDF export (pdf3d.hpp): the U3D stream's block structure
// walks exactly to its declared sizes, the welding keeps the loft's shared
// vertices, the PDF's cross-reference table points at its objects, and the
// options reach the file. Writes build/aerotelic_3d_sample.pdf, which is
// the file to open in Acrobat when the renderer itself is in question --
// no amount of structural checking here can say whether Acrobat draws it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/pdf3d.hpp"
#include "aerotelic/primitive_model.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;
void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

std::uint32_t ReadU32(const std::vector<std::uint8_t>& b, std::size_t at) {
	return static_cast<std::uint32_t>(b[at]) | (static_cast<std::uint32_t>(b[at + 1]) << 8) |
		(static_cast<std::uint32_t>(b[at + 2]) << 16) | (static_cast<std::uint32_t>(b[at + 3]) << 24);
}

std::uint64_t ReadU64(const std::vector<std::uint8_t>& b, std::size_t at) {
	return static_cast<std::uint64_t>(ReadU32(b, at)) | (static_cast<std::uint64_t>(ReadU32(b, at + 4)) << 32);
}

std::uint32_t ReadU16(const std::vector<std::uint8_t>& b, std::size_t at) {
	return static_cast<std::uint32_t>(b[at]) | (static_cast<std::uint32_t>(b[at + 1]) << 8);
}

// One block of a U3D file, as ECMA-363 9.2 lays it out.
struct Block {
	std::uint32_t type = 0;
	std::size_t data_at = 0;
	std::uint32_t data_size = 0;
};

// Walks the blocks of a U3D buffer. Returns false if any block runs past
// the end or is not 32-bit aligned -- which is what a wrong Data Size or
// a forgotten padding byte looks like from the reader's side.
bool WalkBlocks(const std::vector<std::uint8_t>& b, std::vector<Block>& out) {
	std::size_t at = 0;
	while (at < b.size()) {
		if ((at & 3u) != 0u) return false;
		if (at + 12 > b.size()) return false;
		Block blk;
		blk.type = ReadU32(b, at);
		blk.data_size = ReadU32(b, at + 4);
		const std::uint32_t meta_size = ReadU32(b, at + 8);
		blk.data_at = at + 12;
		const std::size_t padded_data = (blk.data_size + 3u) & ~std::size_t(3);
		const std::size_t padded_meta = (meta_size + 3u) & ~std::size_t(3);
		if (blk.data_at + padded_data + padded_meta > b.size()) return false;
		out.push_back(blk);
		at = blk.data_at + padded_data + padded_meta;
	}
	return at == b.size();
}

std::size_t CountType(const std::vector<Block>& blocks, std::uint32_t type) {
	std::size_t n = 0;
	for (const Block& b : blocks) {
		if (b.type == type) ++n;
	}
	return n;
}

std::vector<std::uint8_t> ReadFile(const std::string& path) {
	std::vector<std::uint8_t> out;
	std::FILE* f = nullptr;
#if defined(_WIN32)
	if (fopen_s(&f, path.c_str(), "rb") != 0) f = nullptr;
#else
	f = std::fopen(path.c_str(), "rb");
#endif
	if (f == nullptr) return out;
	std::uint8_t buf[65536];
	std::size_t n = 0;
	while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out.insert(out.end(), buf, buf + n);
	std::fclose(f);
	return out;
}

TriMesh PrimitiveMesh(PrimitiveShape shape, double tolerance) {
	PrimitiveModel m;
	m.shape = shape;
	TriMesh out;
	for (const NamedMesh& nm : m.BuildMeshes(tolerance)) out.Append(nm.mesh);
	return out;
}

// Every object the cross-reference table names must actually start with
// "<n> 0 obj" at the offset it gives.
void CheckXref(const std::vector<std::uint8_t>& pdf) {
	const std::string text(reinterpret_cast<const char*>(pdf.data()), pdf.size());
	const std::size_t startxref = text.rfind("startxref");
	Check(startxref != std::string::npos, "pdf: has a startxref");
	if (startxref == std::string::npos) return;
	const std::size_t xref_at = static_cast<std::size_t>(std::strtoull(text.c_str() + startxref + 9, nullptr, 10));
	Check(xref_at < pdf.size() && text.compare(xref_at, 4, "xref") == 0, "pdf: startxref points at the xref table");
	if (xref_at >= pdf.size() || text.compare(xref_at, 4, "xref") != 0) return;

	// "xref\n<first> <count>\n", then one 20-byte entry per object in the
	// subsection (object 0 is the free-list head).
	std::size_t at = text.find('\n', xref_at) + 1;
	char* after_first = nullptr;
	std::strtoul(text.c_str() + at, &after_first, 10);
	const unsigned long count = std::strtoul(after_first, nullptr, 10);
	at = text.find('\n', at) + 1;
	Check(count >= 2, "pdf: the xref names at least one object");
	int bad = 0;
	for (unsigned long i = 1; i < count; ++i) {
		const std::size_t entry = at + static_cast<std::size_t>(i) * 20;
		if (entry + 20 > pdf.size()) { ++bad; continue; }
		const std::size_t offset = static_cast<std::size_t>(std::strtoull(text.substr(entry, 10).c_str(), nullptr, 10));
		const std::string expect = std::to_string(i) + " 0 obj";
		if (offset + expect.size() > pdf.size() || text.compare(offset, expect.size(), expect) != 0) ++bad;
	}
	Check(bad == 0, "pdf: every xref offset lands on its object header (" + std::to_string(count - 1) + " objects)");
}

}  // namespace

int main() {
	const double tol = 0.01;

	// ---------------------------------------------- the U3D block structure

	{
		std::vector<Pdf3dPart> parts(1);
		parts[0].name = "Sphere";
		parts[0].mesh = PrimitiveMesh(PrimitiveShape::Sphere, tol);
		Check(!parts[0].mesh.IsEmpty(), "a sphere primitive meshes");

		Pdf3dResult stats;
		const std::vector<std::uint8_t> u3d = WriteU3d(parts, 30.0, &stats);
		Check(!u3d.empty(), "u3d: the sphere produces a stream");
		Check(u3d.size() == stats.u3d_bytes, "u3d: the reported size is the stream's size");
		Check(u3d.size() >= 12 && ReadU32(u3d, 0) == 0x00443355u, "u3d: the first block is the File Header");

		// The header's own fields (9.4.1): the no-compression profile, and
		// the two sizes that let a reader skip to the continuations.
		Check(ReadU32(u3d, 12 + 4) == 0x00000004u, "u3d: the no-compression profile bit is set");
		const std::uint32_t declaration_size = ReadU32(u3d, 12 + 8);
		const std::uint64_t file_size = ReadU64(u3d, 12 + 12);
		Check(ReadU32(u3d, 12 + 20) == 106u, "u3d: the character encoding is UTF-8 (MIBenum 106)");
		Check(file_size == u3d.size(), "u3d: File Size is the whole file");
		Check(declaration_size < file_size, "u3d: the declarations stop before the continuations");

		std::vector<Block> blocks;
		Check(WalkBlocks(u3d, blocks), "u3d: the blocks walk exactly to the end of the file");
		Check(CountType(blocks, 0xFFFFFF14u) == 1, "u3d: one Modifier Chain");
		Check(CountType(blocks, 0xFFFFFF53u) == 1, "u3d: one Lit Texture Shader");
		Check(CountType(blocks, 0xFFFFFF54u) == 1, "u3d: one Material Resource");
		Check(CountType(blocks, 0xFFFFFF22u) == 1, "u3d: one Model Node");
		Check(CountType(blocks, 0xFFFFFF3Bu) == 1, "u3d: one CLOD Base Mesh Continuation");

		// Declaration Size must land exactly on a block boundary: the first
		// continuation block. Anything else and a reader loading only the
		// declarations stops mid-block.
		std::size_t at = 0;
		bool on_boundary = false;
		for (const Block& b : blocks) {
			at = b.data_at - 12;
			if (at == declaration_size) { on_boundary = true; break; }
		}
		Check(on_boundary, "u3d: Declaration Size falls on a block boundary");

		// The mesh declaration and its base mesh must agree, or the reader
		// allocates one mesh and is handed another. The CLOD Mesh
		// Declaration is nested inside the Modifier Chain's data, after the
		// chain's name, type, attributes, padding and modifier count.
		Block chain{}, decl{}, base{};
		for (const Block& b : blocks) {
			if (b.type == 0xFFFFFF14u) chain = b;
			if (b.type == 0xFFFFFF3Bu) base = b;
		}
		Check(chain.type != 0 && base.type != 0, "u3d: the chain and the base mesh are both there");
		if (chain.type != 0 && base.type != 0) {
			std::size_t p = chain.data_at + 2 + ReadU16(u3d, chain.data_at) + 8;
			p = (p + 3u) & ~std::size_t(3);  // Modifier Chain Padding (9.4.3.6)
			Check(ReadU32(u3d, p) == 2u, "u3d: the chain holds two modifiers");
			p += 4;
			Check(ReadU32(u3d, p) == 0xFFFFFF31u, "u3d: the chain's first modifier is the CLOD Mesh Declaration");
			decl.type = 0xFFFFFF31u;
			decl.data_size = ReadU32(u3d, p + 4);
			decl.data_at = p + 12;

			// Past the mesh name and the chain index in each block.
			const std::size_t d = decl.data_at + 2 + ReadU16(u3d, decl.data_at) + 4;
			const std::size_t m = base.data_at + 2 + ReadU16(u3d, base.data_at) + 4;
			Check(ReadU32(u3d, d) == 0u, "u3d: the mesh declares normals at every corner");
			Check(ReadU32(u3d, d + 4) == stats.faces, "u3d: the declared face count is the mesh's");
			Check(ReadU32(u3d, d + 4) == ReadU32(u3d, m), "u3d: declaration and base mesh agree on the face count");
			Check(ReadU32(u3d, d + 8) == ReadU32(u3d, m + 4), "u3d: ... and on the position count");
			Check(ReadU32(u3d, d + 12) == ReadU32(u3d, m + 8), "u3d: ... and on the normal count");

			// The base mesh's data is exactly its header, the arrays and
			// the faces: 3 floats a position, 3 a normal, and 7 U32s a face
			// (the shading id and three position/normal index pairs).
			const std::uint32_t nf = ReadU32(u3d, m);
			const std::uint32_t np = ReadU32(u3d, m + 4);
			const std::uint32_t nn = ReadU32(u3d, m + 8);
			const std::size_t expect = (m + 24 - base.data_at) + 12u * np + 12u * nn + 28u * nf;
			Check(base.data_size == expect, "u3d: the base mesh block is exactly its arrays and faces");
		}

		// The loft shares its vertices exactly, so welding must find them
		// all and not invent any: one position per mesh vertex.
		Check(stats.positions == parts[0].mesh.vertices.size(),
			"u3d: welding keeps the sphere's shared vertices (" + std::to_string(stats.positions) + " positions)");
		Check(stats.faces == parts[0].mesh.triangles.size(), "u3d: every triangle becomes a face");
		// A sphere is smooth everywhere: inside a 30 degree crease the
		// normals collapse to one per vertex.
		Check(stats.normals <= stats.positions + 2,
			"u3d: a smooth body needs about one normal per position (" + std::to_string(stats.normals) + ")");
	}

	// ------------------------------------------------- empty and degenerate

	{
		Pdf3dResult stats;
		Check(WriteU3d({}, 30.0, &stats).empty(), "u3d: no parts, no stream");

		std::vector<Pdf3dPart> degenerate(1);
		degenerate[0].name = "Flat";
		degenerate[0].mesh.vertices = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
		degenerate[0].mesh.triangles = {{0, 1, 2}};
		Check(WriteU3d(degenerate, 30.0, &stats).empty(), "u3d: a zero-area triangle is dropped, leaving nothing");

		std::string error;
		Check(!WritePdf3d({}, "should_not_appear.pdf", Pdf3dOptions(), error) && !error.empty(),
			"pdf: refused with a reason when there is nothing to write");
	}

	// ----------------------------------------------- crease angle and parts

	{
		// A box is all creases: at 30 degrees every corner keeps its three
		// face normals, and at 180 they all merge into one per vertex.
		std::vector<Pdf3dPart> parts(1);
		parts[0].mesh = PrimitiveMesh(PrimitiveShape::Box, tol);
		Pdf3dResult sharp, smooth;
		WriteU3d(parts, 30.0, &sharp);
		WriteU3d(parts, 180.0, &smooth);
		Check(sharp.normals == 6, "u3d: a box at a 30 degree crease has six normals, one per face");
		Check(smooth.normals < sharp.normals || smooth.normals <= 8,
			"u3d: a box at a 180 degree crease averages its corners (" + std::to_string(smooth.normals) + " normals)");
	}

	{
		// Three bodies: three of everything, and three model nodes, so
		// Acrobat's model tree gets a row each.
		std::vector<Pdf3dPart> parts(3);
		parts[0].name = "Sphere";
		parts[0].mesh = PrimitiveMesh(PrimitiveShape::Sphere, tol);
		parts[1].name = "Box";
		parts[1].mesh = PrimitiveMesh(PrimitiveShape::Box, tol);
		parts[2].name = "Cylinder";
		parts[2].mesh = PrimitiveMesh(PrimitiveShape::Cylinder, tol);

		Pdf3dResult stats;
		const std::vector<std::uint8_t> u3d = WriteU3d(parts, 30.0, &stats);
		std::vector<Block> blocks;
		Check(WalkBlocks(u3d, blocks), "u3d: three bodies' blocks walk to the end");
		Check(stats.parts == 3, "u3d: three parts written");
		Check(CountType(blocks, 0xFFFFFF22u) == 3, "u3d: one Model Node per body");
		Check(CountType(blocks, 0xFFFFFF3Bu) == 3, "u3d: one base mesh per body");
		Check(CountType(blocks, 0xFFFFFF54u) == 3, "u3d: one Material Resource per body");
		Check(ReadU64(u3d, 12 + 12) == u3d.size(), "u3d: File Size still matches");

		// Two bodies under the same label must not collide in the palette.
		std::vector<Pdf3dPart> same = parts;
		same[1].name = same[0].name = "Wing";
		const std::vector<std::uint8_t> u3d_same = WriteU3d(same, 30.0, &stats);
		std::vector<Block> same_blocks;
		Check(WalkBlocks(u3d_same, same_blocks) && CountType(same_blocks, 0xFFFFFF22u) == 3,
			"u3d: bodies sharing a label still get a node each");
	}

	// ------------------------------------------------------------- the PDF

	{
		std::vector<Pdf3dPart> parts(3);
		parts[0].name = "Sphere";
		parts[0].mesh = PrimitiveMesh(PrimitiveShape::Sphere, tol);
		parts[0].mesh.Translate({-2.6, 0.0, 0.0});
		parts[0].colour[0] = 0.27; parts[0].colour[1] = 0.47; parts[0].colour[2] = 0.72;
		parts[1].name = "Box";
		parts[1].mesh = PrimitiveMesh(PrimitiveShape::Box, tol);
		parts[1].colour[0] = 0.78; parts[1].colour[1] = 0.62; parts[1].colour[2] = 0.28;
		parts[2].name = "Cylinder";
		parts[2].mesh = PrimitiveMesh(PrimitiveShape::Cylinder, tol);
		parts[2].mesh.Translate({2.6, 0.0, 0.0});
		parts[2].colour[0] = 0.40; parts[2].colour[1] = 0.63; parts[2].colour[2] = 0.42;
		parts[2].opacity = 0.55;

		Pdf3dOptions options;
		options.title = "Aerotelic 3D PDF check";
		options.caption = "Sphere, box and a half-transparent cylinder. Click the model and drag to orbit.";

		const std::string path = "build/aerotelic_3d_sample.pdf";
		std::string error;
		Pdf3dResult stats;
		const bool ok = WritePdf3d(parts, path, options, error, &stats);
		Check(ok, "pdf: written" + (ok ? std::string() : " -- " + error));
		if (ok) {
			const std::vector<std::uint8_t> pdf = ReadFile(path);
			Check(pdf.size() == stats.pdf_bytes, "pdf: the reported size is the file's size");
			const std::string text(reinterpret_cast<const char*>(pdf.data()), pdf.size());
			Check(text.compare(0, 8, "%PDF-1.7") == 0, "pdf: the header says 1.7 (3D annotations need 1.6)");
			Check(text.size() > 6 && text.compare(text.size() - 6, 6, "%%EOF\n") == 0, "pdf: ends with %%EOF");
			Check(text.find("/Subtype /3D") != std::string::npos, "pdf: has a 3D annotation");
			Check(text.find("/Subtype /U3D") != std::string::npos, "pdf: its stream is U3D");
			Check(text.find("/3DA << /Type /3DActivation /A /PO") != std::string::npos,
				"pdf: the model is activated when the page opens");
			Check(text.find("/3DI true") != std::string::npos, "pdf: the model stays interactive");
			Check(text.find("/C2W [") != std::string::npos && text.find("/CO ") != std::string::npos,
				"pdf: the default view carries a camera matrix and an orbit distance");
			Check(text.find("/Filter") == std::string::npos,
				"pdf: no filter without a deflate hook, so the stream is the U3D bytes");
			Check(text.find(options.title) != std::string::npos, "pdf: the title reaches the page");
			CheckXref(pdf);
			std::printf("    %s: %zu bodies, %zu positions, %zu normals, %zu faces, u3d %zu bytes, pdf %zu bytes\n",
				path.c_str(), stats.parts, stats.positions, stats.normals, stats.faces, stats.u3d_bytes, stats.pdf_bytes);
		}

		// With a deflate hook the stream is filtered. A hook that only
		// stores (a zlib stream of stored deflate blocks) is enough to
		// check the plumbing without linking zlib into the core -- the
		// application passes the engine's real one.
		options.deflate = [](const std::vector<std::uint8_t>& in, std::vector<std::uint8_t>& out) {
			out.clear();
			out.push_back(0x78); out.push_back(0x01);          // zlib header, no preset dictionary
			std::size_t at = 0;
			while (at < in.size() || in.empty()) {
				const std::uint16_t n = static_cast<std::uint16_t>(std::min<std::size_t>(0xFFFF, in.size() - at));
				const bool last = static_cast<std::size_t>(at) + n >= in.size();
				out.push_back(last ? 1 : 0);                   // BFINAL, stored block
				out.push_back(static_cast<std::uint8_t>(n & 0xFF));
				out.push_back(static_cast<std::uint8_t>(n >> 8));
				out.push_back(static_cast<std::uint8_t>(~n & 0xFF));
				out.push_back(static_cast<std::uint8_t>((~n >> 8) & 0xFF));
				out.insert(out.end(), in.begin() + at, in.begin() + at + n);
				at += n;
				if (last) break;
			}
			// Adler-32 of the uncompressed data, big-endian.
			std::uint32_t a = 1, b = 0;
			for (std::uint8_t v : in) { a = (a + v) % 65521; b = (b + a) % 65521; }
			const std::uint32_t adler = (b << 16) | a;
			out.push_back(static_cast<std::uint8_t>(adler >> 24));
			out.push_back(static_cast<std::uint8_t>(adler >> 16));
			out.push_back(static_cast<std::uint8_t>(adler >> 8));
			out.push_back(static_cast<std::uint8_t>(adler));
			// Stored blocks are larger than the input, which WritePdf3d
			// treats as "do not bother" -- so claim success only when it
			// would actually be smaller. It never is here, which is the
			// point: the fallback must stay valid.
			return true;
		};
		Pdf3dResult filtered;
		Check(WritePdf3d(parts, "build/aerotelic_3d_sample_filtered.pdf", options, error, &filtered),
			"pdf: written again with a deflate hook");
		Check(!filtered.u3d_deflated,
			"pdf: a hook whose output is larger than the input is declined and the stream stays raw");
	}

	std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL OK" : "FAILURES", g_failures, g_failures == 1 ? "" : "s");
	return g_failures == 0 ? 0 : 1;
}
