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

# --- skysim server --------------------------------------------------------------------------
add_executable(skysim
    src/main.cpp
    # M1+: add src/protocol/*.cpp, src/core/*.cpp, src/vehicle/*.cpp, src/terrain/*.cpp, src/api/*.cpp
)
target_include_directories(skysim PRIVATE src)
target_link_libraries(skysim PRIVATE Jolt)
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
target_include_directories(test_packets PRIVATE src)
add_test(NAME packets COMMAND test_packets)
