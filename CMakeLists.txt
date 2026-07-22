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

# --- protocol (no Jolt dependency; shared by server and tests) -------------------------------
find_package(Threads REQUIRED)
add_library(skysim_protocol STATIC
    src/protocol/packets.cpp
    src/protocol/udp_endpoint.cpp
)
target_include_directories(skysim_protocol PUBLIC src)
target_link_libraries(skysim_protocol PUBLIC Threads::Threads)
target_compile_options(skysim_protocol PRIVATE -Wall -Wextra -Wshadow)

# --- vehicle model (no Jolt dependency) ------------------------------------------------------
add_library(skysim_vehicle STATIC
    src/vehicle/motor_model.cpp
    src/vehicle/quad.cpp
)
target_include_directories(skysim_vehicle PUBLIC src)
target_compile_options(skysim_vehicle PRIVATE -Wall -Wextra -Wshadow)

# --- core: Jolt world (the only Jolt consumer above tools/) ----------------------------------
add_library(skysim_core STATIC src/core/world.cpp)
target_include_directories(skysim_core PUBLIC src)
target_link_libraries(skysim_core PRIVATE Jolt)
target_compile_options(skysim_core PRIVATE -Wall -Wextra -Wshadow)

# --- skysim server --------------------------------------------------------------------------
add_executable(skysim src/main.cpp)
target_include_directories(skysim PRIVATE src)
target_link_libraries(skysim PRIVATE skysim_protocol skysim_core skysim_vehicle)
target_compile_options(skysim PRIVATE -Wall -Wextra -Wshadow)

# --- tile cooker (M5) -----------------------------------------------------------------------
add_executable(tile_cooker tools/cooker/main.cpp)
target_include_directories(tile_cooker PRIVATE src)
target_link_libraries(tile_cooker PRIVATE Jolt)

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
