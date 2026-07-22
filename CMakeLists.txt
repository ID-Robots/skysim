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

# --- protocol (no Jolt dependency; shared by server and tests) -------------------------------
find_package(Threads REQUIRED)
add_library(skysim_protocol STATIC
    src/protocol/packets.cpp
    src/protocol/udp_endpoint.cpp
)
target_include_directories(skysim_protocol PUBLIC src)
target_link_libraries(skysim_protocol PUBLIC Threads::Threads)
target_compile_options(skysim_protocol PRIVATE -Wall -Wextra -Wshadow)

# --- skysim server --------------------------------------------------------------------------
add_executable(skysim
    src/main.cpp
    # M1+: add src/core/*.cpp, src/vehicle/*.cpp, src/terrain/*.cpp, src/api/*.cpp
)
target_include_directories(skysim PRIVATE src)
target_link_libraries(skysim PRIVATE skysim_protocol Jolt)
if(UNIX)
    target_link_libraries(skysim PRIVATE pthread)
endif()
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
