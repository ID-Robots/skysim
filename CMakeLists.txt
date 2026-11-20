cmake_minimum_required(VERSION 3.24)
# C only for the x264 build below (its configure needs a C compiler); skysim itself is C++.
project(skysim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# --- Jolt Physics (pinned; bump deliberately, re-run determinism tests after) ---------------
set(CPP_RTTI_ENABLED ON CACHE BOOL "" FORCE)
set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
FetchContent_Declare(JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG        v5.2.0
    SOURCE_SUBDIR  Build
)
FetchContent_MakeAvailable(JoltPhysics)
# Jolt headers are third-party: treat as SYSTEM so our -Wshadow etc. don't fire inside them.
set_target_properties(Jolt PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES $<TARGET_PROPERTY:Jolt,INTERFACE_INCLUDE_DIRECTORIES>)

# --- cpp-httplib (control plane, M4; header-only) ---------------------------------------------
set(HTTPLIB_REQUIRE_OPENSSL OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_OPENSSL_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG        v0.18.3
)
FetchContent_MakeAvailable(httplib)

# --- stb (single-header JPEG writer for the camera feed) --------------------------------------
# Header-only and dependency-free, which is the point: the render path must not
# drag an image library into a physics server that runs on Fargate.
FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        f0569113c93ad095470c54bf34a17b36646bbbb5
)
FetchContent_MakeAvailable(stb)
add_library(stb_image_write INTERFACE)
target_include_directories(stb_image_write SYSTEM INTERFACE ${stb_SOURCE_DIR})

# --- x264 (H.264 encoder for the RTSP camera, TASK-2561) -------------------------------------
# Built from a pinned upstream commit into a static library linked into skysim, so the runtime
# image needs no codec package. Constrained Baseline at a pilot camera's sizes needs nothing
# but libx264 itself: no ffmpeg, no GStreamer. GPL-2.0-or-later, compatible with skysim's GPL-3.0.
# Its x86 assembly (several times faster) needs nasm >= 2.13; without one it is built in plain C
# with a warning — fine for tests, not for a deployment (the Dockerfile installs nasm).
include(ExternalProject)
include(ProcessorCount)
set(SKYSIM_X264_COMMIT b35605ace3ddf7c1a5d67a2eb553f034aef41d55) # stable, 2025-06-08, X264_BUILD 165
set(SKYSIM_X264_ASM AUTO CACHE STRING "x264 assembly: AUTO (when nasm is found), ON (require nasm), OFF")
set_property(CACHE SKYSIM_X264_ASM PROPERTY STRINGS AUTO ON OFF)
if(DEFINED ENV{CMAKE_BUILD_PARALLEL_LEVEL})
    set(_x264_jobs_default $ENV{CMAKE_BUILD_PARALLEL_LEVEL})
else()
    ProcessorCount(_x264_jobs_default)
    if(_x264_jobs_default EQUAL 0)
        set(_x264_jobs_default 1)
    endif()
endif()
set(SKYSIM_X264_JOBS ${_x264_jobs_default} CACHE STRING "make -j for the x264 build")
# nasm is x264's assembler on x86 only. On ARM64 its assembly goes through the C compiler, and
# handing it nasm as AS would make its configure fail, so there nothing is overridden.
set(_x264_env CC=${CMAKE_C_COMPILER})
set(_x264_asm_flags "")
if(SKYSIM_X264_ASM STREQUAL "OFF")
    set(_x264_asm_flags --disable-asm)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|i[3-6]86|x86)$")
    find_program(SKYSIM_NASM nasm)
    if(SKYSIM_NASM)
        # x264's configure refuses anything older than 2.13 outright; treat that like no nasm.
        execute_process(COMMAND ${SKYSIM_NASM} -v OUTPUT_VARIABLE _nasm_v ERROR_QUIET)
        string(REGEX MATCH "version ([0-9]+\\.[0-9]+)" _nasm_match "${_nasm_v}")
        if(NOT CMAKE_MATCH_1 OR CMAKE_MATCH_1 VERSION_LESS 2.13)
            message(STATUS "skysim: ${SKYSIM_NASM} is not nasm >= 2.13 ('${_nasm_v}'), not using it for x264")
            set(SKYSIM_NASM "")
        endif()
    endif()
    if(SKYSIM_NASM)
        list(APPEND _x264_env AS=${SKYSIM_NASM})
    elseif(SKYSIM_X264_ASM STREQUAL "ON")
        message(FATAL_ERROR "SKYSIM_X264_ASM=ON but nasm >= 2.13 was not found")
    else()
        message(WARNING "nasm not found: x264 is built without assembly, several times slower. Install "
                        "nasm for a deployment build (or pass -DSKYSIM_X264_ASM=OFF to silence this).")
        set(_x264_asm_flags --disable-asm)
    endif()
endif()
set(SKYSIM_X264_PREFIX ${CMAKE_BINARY_DIR}/x264)
file(MAKE_DIRECTORY ${SKYSIM_X264_PREFIX}/include)
ExternalProject_Add(x264_build
    GIT_REPOSITORY    https://code.videolan.org/videolan/x264.git
    GIT_TAG           ${SKYSIM_X264_COMMIT}
    GIT_SHALLOW       OFF
    UPDATE_DISCONNECTED ON
    PREFIX            ${CMAKE_BINARY_DIR}/x264-build
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env ${_x264_env}
                      <SOURCE_DIR>/configure --prefix=${SKYSIM_X264_PREFIX} --enable-static --enable-pic
                      --disable-cli --disable-opencl --disable-avs --disable-swscale --disable-lavf
                      --disable-ffms --disable-gpac --disable-lsmash --bit-depth=8 --chroma-format=420
                      ${_x264_asm_flags}
    BUILD_COMMAND     make -j${SKYSIM_X264_JOBS}
    INSTALL_COMMAND   make install-lib-static
    BUILD_IN_SOURCE   ON
    BUILD_BYPRODUCTS  ${SKYSIM_X264_PREFIX}/lib/libx264.a
    LOG_CONFIGURE     ON
    LOG_BUILD         ON
    LOG_INSTALL       ON
    LOG_OUTPUT_ON_FAILURE ON
)
add_library(skysim_x264 STATIC IMPORTED GLOBAL)
set_target_properties(skysim_x264 PROPERTIES
    IMPORTED_LOCATION ${SKYSIM_X264_PREFIX}/lib/libx264.a
    INTERFACE_INCLUDE_DIRECTORIES ${SKYSIM_X264_PREFIX}/include)
find_package(Threads REQUIRED)
target_link_libraries(skysim_x264 INTERFACE Threads::Threads m ${CMAKE_DL_LIBS})
add_dependencies(skysim_x264 x264_build)

# --- link-time optimisation for skysim's own targets ------------------------------------------
# Only for targets whose results do not depend on how floating point is contracted across
# translation units: on ARM64, GCC's default -ffp-contract=fast can fuse a multiply-add that
# inlining across files exposes, and the physics (core, vehicle) and radio model (whose test
# vector is a cross-repo contract) must give the same numbers with or without LTO. Off for
# Debug and coverage builds, where it only costs link time. GCC only (the Dockerfile's
# compiler): GCC's linker plugin links LTO objects into any executable, while with Clang every
# test linking one of these libraries would need the LLVM gold plugin as well.
option(SKYSIM_IPO "Link-time optimisation for skysim's non-numeric targets (Release/RelWithDebInfo)" ON)
set(SKYSIM_IPO_ACTIVE OFF)
if(SKYSIM_IPO AND NOT SKYSIM_COVERAGE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
   AND CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_why LANGUAGES CXX)
    if(_ipo_ok)
        set(SKYSIM_IPO_ACTIVE ON)
        # GCC's -flto=auto runs one LTRANS job per CPU at link time, whatever parallelism the
        # build was given; keep the link to CMAKE_BUILD_PARALLEL_LEVEL when that is set.
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND "$ENV{CMAKE_BUILD_PARALLEL_LEVEL}" MATCHES "^[0-9]+$")
            set(CMAKE_CXX_COMPILE_OPTIONS_IPO -flto=$ENV{CMAKE_BUILD_PARALLEL_LEVEL} -fno-fat-lto-objects)
        endif()
    else()
        message(STATUS "skysim: no IPO/LTO (${_ipo_why})")
    endif()
endif()
function(skysim_ipo)
    if(SKYSIM_IPO_ACTIVE)
        set_target_properties(${ARGN} PROPERTIES INTERPROCEDURAL_OPTIMIZATION ON)
    endif()
endfunction()

# --- coverage instrumentation (our targets only; third-party stays clean) --------------------
# Everything declared after this point picks up --coverage; Jolt/httplib are declared above.
option(SKYSIM_COVERAGE "Instrument skysim targets for gcov/gcovr" OFF)
if(SKYSIM_COVERAGE)
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
endif()

# --- protocol (no Jolt dependency; shared by server and tests) -------------------------------
add_library(skysim_protocol STATIC
    src/protocol/packets.cpp
    src/protocol/udp_endpoint.cpp
)
target_include_directories(skysim_protocol PUBLIC src)
target_link_libraries(skysim_protocol PUBLIC Threads::Threads)
target_compile_options(skysim_protocol PRIVATE -Wall -Wextra -Wshadow)

# --- vehicle model + lifecycle (no Jolt dependency) ------------------------------------------
add_library(skysim_vehicle STATIC
    src/vehicle/motor_model.cpp
    src/vehicle/quad.cpp
    src/vehicle/manager.cpp
)
target_include_directories(skysim_vehicle PUBLIC src)
target_compile_options(skysim_vehicle PRIVATE -Wall -Wextra -Wshadow)

# --- simulated 4G radio field (no Jolt, no HTTP; docs/RADIO.md) -------------------------------
add_library(skysim_radio STATIC
    src/radio/json.cpp
    src/radio/radio_spec.cpp
    src/radio/radio_model.cpp
)
target_include_directories(skysim_radio PUBLIC src)
target_compile_options(skysim_radio PRIVATE -Wall -Wextra -Wshadow)

# --- control plane (M4) -----------------------------------------------------------------------
add_library(skysim_api STATIC src/api/control_server.cpp)
target_include_directories(skysim_api PUBLIC src)
target_link_libraries(skysim_api PUBLIC skysim_vehicle skysim_radio Threads::Threads PRIVATE httplib::httplib)
target_compile_options(skysim_api PRIVATE -Wall -Wextra -Wshadow)

# --- core: Jolt world (the only Jolt consumer above tools/) ----------------------------------
add_library(skysim_core STATIC src/core/world.cpp src/core/cpus.cpp)
target_include_directories(skysim_core PUBLIC src)
target_link_libraries(skysim_core PRIVATE Jolt)
target_compile_options(skysim_core PRIVATE -Wall -Wextra -Wshadow)

# --- the dock marker's image (TASK-2562) -------------------------------------------------------
# Observer 1's e-ArUco dock marker, as skyhub_core's generator drew it (assets/markers/README.md),
# compiled into the renderer as bytes: the binary needs no asset file at run time, and a Docker
# image or a test run cannot miss it. Regenerated when the PNG changes.
set(SKYSIM_DOCK_MARKER_PNG ${CMAKE_SOURCE_DIR}/assets/markers/e_aruco_228_11_100cm.png)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${SKYSIM_DOCK_MARKER_PNG})
file(READ ${SKYSIM_DOCK_MARKER_PNG} _dock_marker_hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _dock_marker_bytes "${_dock_marker_hex}")
file(WRITE ${CMAKE_BINARY_DIR}/generated/dock_marker_png.cpp.in
     "// Generated by CMakeLists.txt from assets/markers/e_aruco_228_11_100cm.png; do not edit.\n"
     "#include <cstddef>\n\nnamespace skysim::render {\n"
     "extern const unsigned char kDockMarkerPng[] = {${_dock_marker_bytes}};\n"
     "extern const std::size_t kDockMarkerPngSize = sizeof(kDockMarkerPng);\n"
     "} // namespace skysim::render\n")
configure_file(${CMAKE_BINARY_DIR}/generated/dock_marker_png.cpp.in
               ${CMAKE_BINARY_DIR}/generated/dock_marker_png.cpp COPYONLY)

# --- terrain: OBJ -> MeshShape cooker + tile streamer ----------------------------------------
add_library(skysim_render STATIC
    src/render/renderer.cpp
    src/render/raster.cpp
    src/render/render_service.cpp
    src/render/jpeg.cpp
    src/render/scene.cpp
    src/render/still.cpp
    src/render/texture.cpp
    src/render/dock_marker.cpp
    src/render/ground_cache.cpp
    src/render/resize.cpp
    ${CMAKE_BINARY_DIR}/generated/dock_marker_png.cpp
)
target_include_directories(skysim_render PUBLIC src)
# stb_image_write's include directory is the whole stb checkout: stb_image.h (the dock marker's
# PNG decoder, src/render/dock_marker.cpp) comes from it too.
target_link_libraries(skysim_render PUBLIC skysim_core Threads::Threads PRIVATE stb_image_write)
target_compile_options(skysim_render PRIVATE -Wall -Wextra -Wshadow)

add_library(skysim_terrain STATIC
    src/terrain/cook.cpp
    src/terrain/tile_streamer.cpp
)
target_include_directories(skysim_terrain PUBLIC src)
target_link_libraries(skysim_terrain PRIVATE Jolt)
target_compile_options(skysim_terrain PRIVATE -Wall -Wextra -Wshadow)

# --- video: the camera as H.264 over RTSP (TASK-2561) ------------------------------------------
add_library(skysim_video STATIC
    src/video/h264.cpp
    src/video/h264_encoder.cpp
    src/video/rtp_h264.cpp
    src/video/rtsp_server.cpp
    src/video/video_service.cpp
)
target_include_directories(skysim_video PUBLIC src)
target_link_libraries(skysim_video PUBLIC skysim_render Threads::Threads PRIVATE skysim_x264)
target_compile_options(skysim_video PRIVATE -Wall -Wextra -Wshadow)

# --- skysim server --------------------------------------------------------------------------
add_executable(skysim src/main.cpp)
target_include_directories(skysim PRIVATE src)
target_link_libraries(skysim PRIVATE skysim_protocol skysim_core skysim_vehicle skysim_api skysim_terrain skysim_render
                                     skysim_video)
target_compile_options(skysim PRIVATE -Wall -Wextra -Wshadow)
skysim_ipo(skysim skysim_protocol skysim_api skysim_video)

# --- world-step benchmark (performance) ------------------------------------------------------
add_executable(world_bench tools/bench/world_bench.cpp)
target_link_libraries(world_bench PRIVATE skysim_core skysim_vehicle)
target_compile_options(world_bench PRIVATE -Wall -Wextra -Wshadow)

add_executable(proto_bench tools/bench/proto_bench.cpp)
target_link_libraries(proto_bench PRIVATE skysim_protocol)
target_compile_options(proto_bench PRIVATE -Wall -Wextra -Wshadow)

add_executable(reply_bench tools/bench/reply_bench.cpp)
target_link_libraries(reply_bench PRIVATE skysim_protocol Threads::Threads)
target_compile_options(reply_bench PRIVATE -Wall -Wextra -Wshadow)

# --- tile cooker ------------------------------------------------------------------------------
add_executable(render_probe tools/render_probe/main.cpp)
target_include_directories(render_probe PRIVATE src)
target_link_libraries(render_probe PRIVATE skysim_render skysim_core)

# --- survey stills without a simulator (tools/survey/survey_fixture.py drives it) -------------
add_executable(survey_still tools/survey/survey_still.cpp)
target_include_directories(survey_still PRIVATE src)
target_link_libraries(survey_still PRIVATE skysim_render skysim_core)
target_compile_options(survey_still PRIVATE -Wall -Wextra -Wshadow)

add_executable(tile_cooker tools/cooker/main.cpp)
target_include_directories(tile_cooker PRIVATE src)
target_link_libraries(tile_cooker PRIVATE skysim_terrain)
target_compile_options(tile_cooker PRIVATE -Wall -Wextra -Wshadow)

# --- tests ----------------------------------------------------------------------------------
enable_testing()
add_executable(test_packets tests/test_packets.cpp)
target_link_libraries(test_packets PRIVATE skysim_protocol)
target_compile_definitions(test_packets PRIVATE SKYSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(test_packets PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME packets COMMAND test_packets)

add_executable(test_json tests/test_json.cpp)
target_link_libraries(test_json PRIVATE skysim_protocol)
target_compile_options(test_json PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME json COMMAND test_json)

add_executable(test_frames tests/test_frames.cpp)
target_include_directories(test_frames PRIVATE src)
target_compile_options(test_frames PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME frames COMMAND test_frames)

add_executable(test_quad tests/test_quad.cpp)
target_link_libraries(test_quad PRIVATE skysim_vehicle)
target_compile_options(test_quad PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME quad COMMAND test_quad)

add_executable(test_world tests/test_world.cpp)
target_link_libraries(test_world PRIVATE skysim_core)
target_compile_options(test_world PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME world COMMAND test_world)

add_executable(test_collision tests/test_collision.cpp)
target_link_libraries(test_collision PRIVATE skysim_core skysim_terrain)
target_compile_options(test_collision PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME collision COMMAND test_collision)

add_executable(test_render tests/test_render.cpp)
target_link_libraries(test_render PRIVATE skysim_render skysim_core skysim_terrain)
target_compile_options(test_render PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME render COMMAND test_render)

add_executable(test_still tests/test_still.cpp)
target_link_libraries(test_still PRIVATE skysim_render skysim_core skysim_terrain skysim_api httplib::httplib
                                         stb_image_write)
target_compile_options(test_still PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME still COMMAND test_still)

# Observer 1's e-ArUco dock marker (TASK-2562): the image compiled in, its texture, the frames.
add_executable(test_dock_marker tests/test_dock_marker.cpp)
target_link_libraries(test_dock_marker PRIVATE skysim_render skysim_core)
target_compile_definitions(test_dock_marker PRIVATE SKYSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(test_dock_marker PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME dock_marker COMMAND test_dock_marker)

add_executable(test_dock_api tests/test_dock_api.cpp)
target_link_libraries(test_dock_api PRIVATE skysim_api skysim_render httplib::httplib)
target_compile_options(test_dock_api PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME dock_api COMMAND test_dock_api)

add_executable(test_endpoint tests/test_endpoint.cpp)
target_link_libraries(test_endpoint PRIVATE skysim_protocol)
target_compile_options(test_endpoint PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME endpoint COMMAND test_endpoint)

add_executable(test_manager tests/test_manager.cpp)
target_link_libraries(test_manager PRIVATE skysim_vehicle)
target_include_directories(test_manager PRIVATE src)
target_compile_options(test_manager PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME manager COMMAND test_manager)

add_executable(test_api tests/test_api.cpp)
target_link_libraries(test_api PRIVATE skysim_api httplib::httplib)
target_compile_options(test_api PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME api COMMAND test_api)

add_executable(test_radio tests/test_radio.cpp)
target_link_libraries(test_radio PRIVATE skysim_radio Threads::Threads)
target_compile_definitions(test_radio PRIVATE SKYSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(test_radio PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME radio COMMAND test_radio)

add_executable(test_radio_api tests/test_radio_api.cpp)
target_link_libraries(test_radio_api PRIVATE skysim_api httplib::httplib)
target_compile_definitions(test_radio_api PRIVATE SKYSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(test_radio_api PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME radio_api COMMAND test_radio_api)

add_executable(test_video tests/test_video.cpp)
target_link_libraries(test_video PRIVATE skysim_video skysim_render)
target_compile_options(test_video PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME video COMMAND test_video)

add_executable(test_video_api tests/test_video_api.cpp)
target_link_libraries(test_video_api PRIVATE skysim_api skysim_video httplib::httplib)
target_compile_options(test_video_api PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME video_api COMMAND test_video_api)

add_executable(test_thread_pool tests/test_thread_pool.cpp)
target_include_directories(test_thread_pool PRIVATE src)
target_link_libraries(test_thread_pool PRIVATE Threads::Threads)
target_compile_options(test_thread_pool PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME thread_pool COMMAND test_thread_pool)

add_executable(test_cpus tests/test_cpus.cpp)
target_link_libraries(test_cpus PRIVATE skysim_core)
target_compile_options(test_cpus PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME cpus COMMAND test_cpus)

add_executable(test_streamer tests/test_streamer.cpp)
target_link_libraries(test_streamer PRIVATE skysim_terrain skysim_core)
target_compile_options(test_streamer PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME streamer COMMAND test_streamer)

set_tests_properties(
    packets json frames quad world collision endpoint manager api radio radio_api thread_pool streamer still cpus
    video video_api dock_marker dock_api
    PROPERTIES LABELS "unit"
)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
    add_test(NAME app_smoke
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/app_smoke.py
                     $<TARGET_FILE:skysim>)
    add_test(NAME app_churn
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/harness/churn.py
                     --sim $<TARGET_FILE:skysim>
                     --cycles 100
                     --api-port 18645
                     --base-instance 96)
    # The reconstruction fixture (tools/survey): frame count, geotags and determinism.
    add_test(NAME survey_fixture
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/survey_fixture_smoke.py
                     $<TARGET_FILE:survey_still> $<TARGET_FILE:tile_cooker>)
    # Building heights match the map's (TASK-2679): three real Plovdiv buildings cooked from OSM by
    # the Mapbox Streets v8 rules and from Mapbox vector tiles, each roof probed through
    # POST /mission/check the way the gateway's GET /map/structures does for the Arbiter.
    add_test(NAME building_heights
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/building_heights_smoke.py
                     $<TARGET_FILE:skysim> $<TARGET_FILE:tile_cooker>)
    # Survey scans replace the map's prisms where we have them (TASK-2735): a synthetic ODM model
    # over a real OSM building, merged by tools/cooker/scan_tiles.py, cooked, and probed through
    # POST /mission/check — the scanned roof and walls, the dropped prism, the OSM buildings kept.
    add_test(NAME scan_buildings
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/scan_buildings_smoke.py
                     $<TARGET_FILE:skysim> $<TARGET_FILE:tile_cooker>)
    # The camera as H.264 over RTSP, pulled with ffprobe (TASK-2561): two vehicles, both
    # transports, codec, profile, frame rate and keyframe interval. Skipped (77) without ffprobe.
    add_test(NAME rtsp_smoke
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/rtsp_smoke.py $<TARGET_FILE:skysim>)
    set_tests_properties(rtsp_smoke PROPERTIES SKIP_RETURN_CODE 77)
    # Observer 1's e-ArUco dock marker (TASK-2562): the proof — a fake SITL hovers a vehicle over
    # its dock at 30 m down to 0.7 m, and every still.jpg and RTSP frame is decoded with cv2.aruco
    # and solvePnP against the published intrinsics and ground truth. Skipped (77) without
    # python3-opencv or ffmpeg. Three heights in the coverage build, whose -O0 renders are slow.
    # The frames stay in build/dock_marker_frames for --analyse with another OpenCV (ci.yml).
    set(_dock_smoke_args "")
    if(SKYSIM_COVERAGE)
        set(_dock_smoke_args --quick)
    endif()
    add_test(NAME dock_marker_smoke
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/dock_marker_smoke.py $<TARGET_FILE:skysim>
                     --frames ${CMAKE_BINARY_DIR}/dock_marker_frames ${_dock_smoke_args})
    set_tests_properties(dock_marker_smoke PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
    # A vehicle's own cameras draw its own dock alone (TASK-2609): slot 0 flown over its dock with
    # slots 1 and 2 parked on theirs 5 and 10 m east; at 30, 10 and 5 m its camera.mjpg, RTSP frames
    # and stills decode exactly one 228, slot 0's, and with --dock-markers-visible all more than one.
    # The pictures stay in build/dock_visibility_frames for --analyse with another OpenCV (ci.yml).
    add_test(NAME dock_visibility_smoke
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/dock_visibility_smoke.py $<TARGET_FILE:skysim>
                     --frames ${CMAKE_BINARY_DIR}/dock_visibility_frames ${_dock_smoke_args})
    set_tests_properties(dock_visibility_smoke PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
    # Per-vehicle camera modes and the landing camera (TASK-2661): spawn with a camera, change and
    # reset it over the API, and a vehicle flown up from its dock switches its RTSP picture size in
    # the landing zone and back above it, on one session.
    add_test(NAME camera_modes_smoke
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/camera_modes_smoke.py $<TARGET_FILE:skysim>)
    # The altitude-hold probe's own check (TASK-2661): hold() judges the telemetry and opens the
    # window that watches the rest of it under one lock, so a reply cannot fall between. No simulator.
    add_test(NAME stream_check_selftest
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/render_probe/stream_check.py --selftest)
    set_tests_properties(stream_check_selftest PROPERTIES LABELS "unit" TIMEOUT 60)
    set_tests_properties(app_smoke app_churn survey_fixture building_heights scan_buildings rtsp_smoke camera_modes_smoke
        PROPERTIES
        LABELS "happy-path"
        TIMEOUT 180
        RUN_SERIAL TRUE
    )
    set_tests_properties(dock_marker_smoke dock_visibility_smoke PROPERTIES LABELS "happy-path" RUN_SERIAL TRUE)
endif()

add_test(NAME performance_world
         COMMAND world_bench 200 2000 --max-p99-us 1250)
add_test(NAME performance_protocol
         COMMAND proto_bench 2000000 --max-combined-ns 2000)
add_test(NAME performance_reply
         COMMAND reply_bench 100 1000 3 --max-pooled-p99-us 1250)
set_tests_properties(
    performance_world performance_protocol performance_reply
    PROPERTIES
        LABELS "performance"
        TIMEOUT 120
        RUN_SERIAL TRUE
)
