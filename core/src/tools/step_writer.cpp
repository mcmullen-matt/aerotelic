// aerotelic_step_writer: writes one STEP file from BREP files, in its own
// process.
//
// The application cannot link OCCT's STEP toolkits (see step_export.cpp
// and ue/Source/AerotelicUE/AerotelicUE.Build.cs: the chain drags in
// zlib/freetype/libpng16/tiff under the names the engine already loads its
// own copies of, which corrupted the heap once). So File -> Export STEP
// writes each body to a temporary .brep with the toolkits it does have
// (TKBRep's BRepTools::Write), then runs this program, whose DLLs sit
// beside it and never enter the engine's process.
//
//   aerotelic_step_writer <out.step> [--named|--plain] <label1> <body1.brep> [<label2> <body2.brep> ...]
//
// --named (the default) writes an XCAF product structure so each body
// arrives in the receiving CAD system as a named part (main_window.py
// _write_step); --plain writes one anonymous compound through
// STEPControl_Writer for a tool that chokes on the assembly structure.
// Exit code 0 on success; otherwise 1 with the reason on stderr.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace {

int Fail(const std::string& why) {
	std::fprintf(stderr, "%s\n", why.c_str());
	return 1;
}

} // namespace

int main(int argc, char** argv) {
	if (argc < 4) {
		return Fail("usage: aerotelic_step_writer <out.step> [--named|--plain] <label> <body.brep> ...");
	}
	const std::string out_path = argv[1];
	bool named = true;
	int first = 2;
	if (std::strcmp(argv[2], "--named") == 0) { named = true; first = 3; }
	else if (std::strcmp(argv[2], "--plain") == 0) { named = false; first = 3; }
	if ((argc - first) < 2 || (argc - first) % 2 != 0) {
		return Fail("bodies must be given as <label> <body.brep> pairs");
	}

	try {
		std::vector<std::pair<std::string, TopoDS_Shape>> bodies;
		BRep_Builder builder;
		for (int i = first; i + 1 < argc; i += 2) {
			TopoDS_Shape shape;
			if (!BRepTools::Read(shape, argv[i + 1], builder) || shape.IsNull()) {
				return Fail(std::string("could not read ") + argv[i + 1]);
			}
			bodies.emplace_back(argv[i], shape);
		}

		IFSelect_ReturnStatus status;
		if (named) {
			Handle(TDocStd_Document) doc = new TDocStd_Document("MDTV-XCAF");
			Handle(XCAFDoc_ShapeTool) shape_tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
			for (const auto& [label, shape] : bodies) {
				// false: a part, not an assembly node.
				TDF_Label part = shape_tool->AddShape(shape, false);
				TDataStd_Name::Set(part, TCollection_ExtendedString(label.c_str(), true));
			}
			STEPCAFControl_Writer writer;
			if (!writer.Transfer(doc, STEPControl_AsIs)) {
				return Fail("STEP transfer failed");
			}
			status = writer.Write(out_path.c_str());
		} else {
			TopoDS_Compound compound;
			builder.MakeCompound(compound);
			for (const auto& [label, shape] : bodies) builder.Add(compound, shape);
			STEPControl_Writer writer;
			writer.Transfer(compound, STEPControl_AsIs);
			status = writer.Write(out_path.c_str());
		}
		if (status != IFSelect_RetDone) {
			return Fail("STEP writer reported status " + std::to_string(static_cast<int>(status)));
		}
	} catch (const Standard_Failure& e) {
		return Fail(std::string("OCCT error: ") + (e.GetMessageString() ? e.GetMessageString() : "unknown"));
	} catch (const std::exception& e) {
		return Fail(e.what());
	}
	return 0;
}
