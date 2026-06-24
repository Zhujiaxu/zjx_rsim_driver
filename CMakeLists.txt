cmake_minimum_required(VERSION 3.14)
project(rsim_driver
    DESCRIPTION "RSim driver plugins (default-driver, plugin-acc, ...)"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Plugin SDK header path. Default points to the third_party symlink in the
# repo root (which should target the third_party-feature-plugin worktree).
# Override with -DRSIM_PLUGIN_SDK_INCLUDE=... if you keep RSimAPI elsewhere.
if(NOT DEFINED RSIM_PLUGIN_SDK_INCLUDE)
    set(RSIM_PLUGIN_SDK_INCLUDE ${CMAKE_SOURCE_DIR}/third_party/RSimAPI/source)
endif()

if(NOT EXISTS ${RSIM_PLUGIN_SDK_INCLUDE}/rsim/worldsim_plugin/PluginInterface.hpp)
    message(FATAL_ERROR
        "Plugin SDK header not found under ${RSIM_PLUGIN_SDK_INCLUDE}.\n"
        "Expected: <RSIM_PLUGIN_SDK_INCLUDE>/rsim/worldsim_plugin/PluginInterface.hpp\n"
        "Make sure 'third_party' symlink targets the third_party-feature-plugin worktree, "
        "or pass -DRSIM_PLUGIN_SDK_INCLUDE=/abs/path/to/RSimAPI/source.")
endif()

message(STATUS "RSim plugin SDK include: ${RSIM_PLUGIN_SDK_INCLUDE}")

# --- Eigen3 (header-only) ---
# Eigen source tree lives under third_party/eigen.
set(EIGEN3_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/eigen CACHE PATH "Eigen3 include directory")
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3_Eigen INTERFACE)
    target_include_directories(Eigen3_Eigen INTERFACE ${EIGEN3_INCLUDE_DIR})
    add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)
endif()

# --- OsqpEigen (C++ Eigen wrapper for OSQP) ---
# Locally installed at /usr/local; provides OsqpEigen::OsqpEigen.
# Transitive deps: osqp, Eigen3 (already defined above from third_party/eigen).
find_package(OsqpEigen REQUIRED)

# Pull in third_party as a sub-build to expose the OpenDriveParser target
# (and its transitive deps: eigen, proj, pugixml, CommonMini, ...).
# EXCLUDE_FROM_ALL keeps the rsim-driver default build minimal — only OpenDriveParser
# and the targets we actually link will get compiled.
set(RSIM_THIRD_PARTY_SRC ${CMAKE_SOURCE_DIR}/third_party CACHE PATH
    "Path to third_party source tree (defaults to repo's symlink).")

set(_RSIM_WARN_DEPRECATED_WAS_DEFINED FALSE)
if(DEFINED CMAKE_WARN_DEPRECATED)
    set(_RSIM_WARN_DEPRECATED_WAS_DEFINED TRUE)
    set(_RSIM_WARN_DEPRECATED_PREVIOUS ${CMAKE_WARN_DEPRECATED})
endif()

set(_RSIM_SUPPRESS_DEVELOPER_WARNINGS_WAS_DEFINED FALSE)
if(DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(_RSIM_SUPPRESS_DEVELOPER_WARNINGS_WAS_DEFINED TRUE)
    set(_RSIM_SUPPRESS_DEVELOPER_WARNINGS_PREVIOUS
        ${CMAKE_SUPPRESS_DEVELOPER_WARNINGS})
endif()

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL
    "Suppress CMake deprecation warnings while configuring third_party." FORCE)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL
    "Suppress CMake developer warnings while configuring third_party." FORCE)
add_subdirectory(${RSIM_THIRD_PARTY_SRC} ${CMAKE_BINARY_DIR}/_third_party EXCLUDE_FROM_ALL)

if(_RSIM_WARN_DEPRECATED_WAS_DEFINED)
    set(CMAKE_WARN_DEPRECATED ${_RSIM_WARN_DEPRECATED_PREVIOUS} CACHE BOOL
        "Whether to issue warnings for deprecated CMake functionality." FORCE)
else()
    unset(CMAKE_WARN_DEPRECATED CACHE)
    unset(CMAKE_WARN_DEPRECATED)
endif()

if(_RSIM_SUPPRESS_DEVELOPER_WARNINGS_WAS_DEFINED)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS
        ${_RSIM_SUPPRESS_DEVELOPER_WARNINGS_PREVIOUS} CACHE BOOL
        "Suppress CMake warnings that are meant for project developers." FORCE)
else()
    unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS CACHE)
    unset(CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
endif()

if(TARGET proj)
    target_compile_options(proj PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wno-format-overflow>)
endif()

add_subdirectory(source)
