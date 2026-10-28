cmake_minimum_required(VERSION 3.20)
project(aerotelic_core C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Where the fixture-check executables find their tests/*_fixtures.txt,
# wherever the repository is checked out.
add_compile_definitions(AEROTELIC_TESTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests")

# Reuses the OCCT 7.9.3 dev kit already installed for the Python/pythonocc-core
# build of Aerotelic (see environment.yml), instead of fetching/building OCCT
# separately -- same reasoning as the aerotelic_occt_spike UE plugin.
set(OCCT_ROOT "C:/Users/Matt/micromamba/envs/aerotelic/Library")
set(OCCT_LIBS
	TKernel TKMath TKG2d TKG3d TKGeomAlgo TKGeomBase
	TKBRep TKTopAlgo TKPrim TKBO TKBool TKShHealing TKMesh TKFillet TKOffset TKFeat
	TKDESTEP TKDE TKXSBase TKRWMesh
)
foreach(lib ${OCCT_LIBS})
	list(APPEND OCCT_LINK_LIBS "${OCCT_ROOT}/lib/${lib}.lib")
endforeach()

# yaml-cpp, for the project-file (de)serialization layer. Vendored from
# source and built as a static library with this project's own toolchain
# (third_party/yaml-cpp), rather than linked against the prebuilt
# conda-forge DLL: yaml-cpp's API crosses module boundaries carrying
# std::string/std::vector-heavy YAML::Node values, and a DLL built by a
# different toolchain/CRT config than the consumer caused a real
# STATUS_HEAP_CORRUPTION crash when embedded in Unreal Engine (allocated in
# one CRT heap, freed in another). A statically-linked, same-toolchain build
# has no separate heap to mismatch.
set(YAMLCPP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp")
set(YAMLCPP_INCLUDE_DIR "${YAMLCPP_SRC_DIR}/include")
set(YAMLCPP_LIB "${YAMLCPP_SRC_DIR}/build/Release/yaml-cpp.lib")

add_library(aerotelic_core STATIC
	src/nurbs.cpp
	src/airfoil.cpp
	src/airfoil_dat.cpp
	src/primitive_model.cpp
	src/feature_tree.cpp
	src/wing_model.cpp
	src/section_cut.cpp
	src/panel_integrals.cpp
	src/panel_method.cpp
	src/wing_panels.cpp
	src/panel_loads.cpp
	src/colormaps.cpp
	src/nacelle_model.cpp
	src/lofted_nacelle_model.cpp
	src/propeller_model.cpp
	src/fuselage_model.cpp
	src/project_yaml.cpp
	src/step_export.cpp
	src/mesh.cpp
	src/sdf.cpp
	src/lattice.cpp
	src/shell.cpp
	src/tet.cpp
	src/tet_skin.cpp
	src/tet_improve.cpp
	src/mesh_boolean.cpp
	src/pdf3d.cpp
	src/dust_case.cpp
	src/mass_properties.cpp
	src/units.cpp
	# Shewchuk's exact geometric predicates (public domain), compiled as C
	# with strict IEEE rounding: the adaptive arithmetic is defeated by
	# fast-math style reassociation.
	third_party/predicates/predicates.c
)
set_source_files_properties(third_party/predicates/predicates.c PROPERTIES COMPILE_OPTIONS "/fp:strict")
# predicates.c has a routine called compress() (it compresses an expansion),
# which is also zlib's entry point: linking the standalone application, whose
# engine modules pull in zlibstatic, failed with LNK2005 "compress already
# defined". It is internal to that translation unit (predicates.h does not
# declare it), so it is renamed at compile time rather than patching the
# vendored source.
set_source_files_properties(third_party/predicates/predicates.c PROPERTIES COMPILE_DEFINITIONS "compress=aerotelic_predicates_compress")
target_include_directories(aerotelic_core PUBLIC include "${OCCT_ROOT}/include/opencascade" "${YAMLCPP_INCLUDE_DIR}")
target_link_libraries(aerotelic_core PUBLIC ${OCCT_LINK_LIBS} "${YAMLCPP_LIB}")
target_compile_definitions(aerotelic_core PUBLIC YAML_CPP_STATIC_DEFINE)

# The STEP writer the application runs as a separate process (see
# src/tools/step_writer.cpp): the STEP + XCAF toolkits, which the
# application itself must not load.
add_executable(aerotelic_step_writer src/tools/step_writer.cpp)
target_include_directories(aerotelic_step_writer PRIVATE "${OCCT_ROOT}/include/opencascade")
foreach(lib TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo
		TKDE TKXSBase TKDESTEP TKCDF TKLCAF TKXCAF TKVCAF TKV3d TKService)
	target_link_libraries(aerotelic_step_writer PRIVATE "${OCCT_ROOT}/lib/${lib}.lib")
endforeach()

add_executable(step_writer_check tests/step_writer_check.cpp)
target_link_libraries(step_writer_check PRIVATE aerotelic_core)
target_compile_definitions(step_writer_check PRIVATE AEROTELIC_STEP_WRITER="$<TARGET_FILE:aerotelic_step_writer>")
add_dependencies(step_writer_check aerotelic_step_writer)

add_executable(nurbs_fixture_check tests/nurbs_fixture_check.cpp)
target_link_libraries(nurbs_fixture_check PRIVATE aerotelic_core)

add_executable(airfoil_fixture_check tests/airfoil_fixture_check.cpp)
target_link_libraries(airfoil_fixture_check PRIVATE aerotelic_core)

add_executable(airfoil_blend_check tests/airfoil_blend_check.cpp)
target_link_libraries(airfoil_blend_check PRIVATE aerotelic_core)

add_executable(airfoil_dat_check tests/airfoil_dat_check.cpp)
target_link_libraries(airfoil_dat_check PRIVATE aerotelic_core)

add_executable(primitive_fixture_check tests/primitive_fixture_check.cpp)
target_link_libraries(primitive_fixture_check PRIVATE aerotelic_core)

add_executable(feature_tree_check tests/feature_tree_check.cpp)
target_link_libraries(feature_tree_check PRIVATE aerotelic_core)

add_executable(wing_fixture_check tests/wing_fixture_check.cpp)
target_link_libraries(wing_fixture_check PRIVATE aerotelic_core)

add_executable(section_cut_check tests/section_cut_check.cpp)
target_link_libraries(section_cut_check PRIVATE aerotelic_core)

add_executable(flap_mesh_check tests/flap_mesh_check.cpp)
target_link_libraries(flap_mesh_check PRIVATE aerotelic_core)

add_executable(panel_integrals_check tests/panel_integrals_check.cpp)
target_link_libraries(panel_integrals_check PRIVATE aerotelic_core)

add_executable(panel_method_check tests/panel_method_check.cpp)
target_link_libraries(panel_method_check PRIVATE aerotelic_core)

add_executable(wing_panels_check tests/wing_panels_check.cpp)
target_link_libraries(wing_panels_check PRIVATE aerotelic_core)

add_executable(panel_loads_check tests/panel_loads_check.cpp)
target_link_libraries(panel_loads_check PRIVATE aerotelic_core)

add_executable(panel_p38_check tests/panel_p38_check.cpp)
target_link_libraries(panel_p38_check PRIVATE aerotelic_core)

add_executable(panel_p38_config_check tests/panel_p38_config_check.cpp)
target_link_libraries(panel_p38_config_check PRIVATE aerotelic_core)

add_executable(nacelle_fixture_check tests/nacelle_fixture_check.cpp)
target_link_libraries(nacelle_fixture_check PRIVATE aerotelic_core)

add_executable(lofted_nacelle_fixture_check tests/lofted_nacelle_fixture_check.cpp)
target_link_libraries(lofted_nacelle_fixture_check PRIVATE aerotelic_core)

add_executable(propeller_fixture_check tests/propeller_fixture_check.cpp)
target_link_libraries(propeller_fixture_check PRIVATE aerotelic_core)

add_executable(fuselage_fixture_check tests/fuselage_fixture_check.cpp)
target_link_libraries(fuselage_fixture_check PRIVATE aerotelic_core)

add_executable(wing_propeller_roundtrip_check tests/wing_propeller_roundtrip_check.cpp)
target_link_libraries(wing_propeller_roundtrip_check PRIVATE aerotelic_core)

add_executable(project_yaml_check tests/project_yaml_check.cpp)
target_link_libraries(project_yaml_check PRIVATE aerotelic_core)

add_executable(mesh_check tests/mesh_check.cpp)
target_link_libraries(mesh_check PRIVATE aerotelic_core)

add_executable(sdf_check tests/sdf_check.cpp)
target_link_libraries(sdf_check PRIVATE aerotelic_core)

add_executable(shell_check tests/shell_check.cpp)
target_link_libraries(shell_check PRIVATE aerotelic_core)

add_executable(lattice_check tests/lattice_check.cpp)
target_link_libraries(lattice_check PRIVATE aerotelic_core)

add_executable(tet_check tests/tet_check.cpp)
target_link_libraries(tet_check PRIVATE aerotelic_core)

add_executable(mesh_boolean_check tests/mesh_boolean_check.cpp)
target_link_libraries(mesh_boolean_check PRIVATE aerotelic_core)

add_executable(pdf3d_check tests/pdf3d_check.cpp)
target_link_libraries(pdf3d_check PRIVATE aerotelic_core)

add_executable(dust_case_check tests/dust_case_check.cpp)
target_link_libraries(dust_case_check PRIVATE aerotelic_core)

add_executable(mass_properties_check tests/mass_properties_check.cpp)
target_link_libraries(mass_properties_check PRIVATE aerotelic_core)

add_executable(units_check tests/units_check.cpp)
target_link_libraries(units_check PRIVATE aerotelic_core)

