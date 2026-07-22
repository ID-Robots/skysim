cmake_minimum_required(VERSION 3.24)
project(skysim LANGUAGES CXX)

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

# --- coverage instrumentation (our targets only; third-party stays clean) --------------------
# Everything declared after this point picks up --coverage; Jolt/httplib are declared above.
option(SKYSIM_COVERAGE "Instrument skysim targets for gcov/gcovr" OFF)
if(SKYSIM_COVERAGE)
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
endif()

# --- protocol (no Jolt dependency; shared by server and tests) -------------------------------
find_package(Threads REQUIRED)
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

# --- control plane (M4) -----------------------------------------------------------------------
add_library(skysim_api STATIC src/api/control_server.cpp)
target_include_directories(skysim_api PUBLIC src)
target_link_libraries(skysim_api PUBLIC skysim_vehicle Threads::Threads PRIVATE httplib::httplib)
target_compile_options(skysim_api PRIVATE -Wall -Wextra -Wshadow)

# --- core: Jolt world (the only Jolt consumer above tools/) ----------------------------------
add_library(skysim_core STATIC src/core/world.cpp)
target_include_directories(skysim_core PUBLIC src)
target_link_libraries(skysim_core PRIVATE Jolt)
target_compile_options(skysim_core PRIVATE -Wall -Wextra -Wshadow)

# --- terrain: OBJ -> MeshShape cooker + tile streamer ----------------------------------------
add_library(skysim_terrain STATIC
    src/terrain/cook.cpp
    src/terrain/tile_streamer.cpp
)
target_include_directories(skysim_terrain PUBLIC src)
target_link_libraries(skysim_terrain PRIVATE Jolt)
target_compile_options(skysim_terrain PRIVATE -Wall -Wextra -Wshadow)

# --- skysim server --------------------------------------------------------------------------
add_executable(skysim src/main.cpp)
target_include_directories(skysim PRIVATE src)
target_link_libraries(skysim PRIVATE skysim_protocol skysim_core skysim_vehicle skysim_api skysim_terrain)
target_compile_options(skysim PRIVATE -Wall -Wextra -Wshadow)

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

add_executable(test_thread_pool tests/test_thread_pool.cpp)
target_include_directories(test_thread_pool PRIVATE src)
target_link_libraries(test_thread_pool PRIVATE Threads::Threads)
target_compile_options(test_thread_pool PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME thread_pool COMMAND test_thread_pool)

add_executable(test_streamer tests/test_streamer.cpp)
target_link_libraries(test_streamer PRIVATE skysim_terrain skysim_core)
target_compile_options(test_streamer PRIVATE -Wall -Wextra -Wshadow)
add_test(NAME streamer COMMAND test_streamer)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
    add_test(NAME app_smoke
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/app_smoke.py
                     $<TARGET_FILE:skysim>)
    set_tests_properties(app_smoke PROPERTIES TIMEOUT 180)
endif()
