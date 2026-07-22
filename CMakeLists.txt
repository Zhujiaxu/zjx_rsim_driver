cmake_minimum_required(VERSION 3.14)
project(rsim_driver
    DESCRIPTION "RSim driver plugins (default-driver, plugin-acc, ...)"
    LANGUAGES CXX)

include(CTest)
configure_file(${CMAKE_SOURCE_DIR}/cmake/CTestCustom.cmake.in
               ${CMAKE_BINARY_DIR}/CTestCustom.cmake @ONLY)

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
# Keep all project-side Eigen code on the repository-pinned headers.  Define
# the canonical target before loading OsqpEigen so its exported dependency also
# resolves to this target instead of a system Eigen installation.
set(EIGEN3_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/eigen/include CACHE PATH
    "Vendored Eigen3 include directory")
if(NOT EXISTS ${EIGEN3_INCLUDE_DIR}/Eigen/Core)
    message(FATAL_ERROR
        "Vendored Eigen headers not found under ${EIGEN3_INCLUDE_DIR}.")
endif()
if(NOT TARGET Eigen3::Eigen)
    add_library(rsim_eigen INTERFACE)
    target_include_directories(rsim_eigen SYSTEM INTERFACE
        ${EIGEN3_INCLUDE_DIR})
    # libOsqpEigen is a prebuilt, unsanitized shared library.  Keep Eigen's
    # allocation convention identical on both sides of that ABI boundary;
    # Eigen otherwise changes it automatically when ASan is enabled.
    target_compile_definitions(rsim_eigen INTERFACE
        EIGEN_MALLOC_ALREADY_ALIGNED=1)
    add_library(Eigen3::Eigen ALIAS rsim_eigen)
endif()

# --- OsqpEigen (C++ Eigen wrapper for OSQP) ---
# Locally installed at /usr/local; provides OsqpEigen::OsqpEigen.
# Transitive deps: osqp and the vendored Eigen3 target above.
find_package(OsqpEigen REQUIRED)

# Build only the third-party targets used by this plugin. Adding the entire
# external worktree configures RSimAPI and rpclib, both of which generate files
# in their source trees and therefore break read-only/clean worktrees.
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
if(NOT TARGET eigen)
    add_library(eigen INTERFACE)
    target_link_libraries(eigen INTERFACE Eigen3::Eigen)
endif()

if(NOT TARGET pugixml)
    add_library(pugixml STATIC
        ${RSIM_THIRD_PARTY_SRC}/pugixml/src/pugixml.cpp)
    target_include_directories(pugixml PUBLIC
        ${RSIM_THIRD_PARTY_SRC}/pugixml/src)
    add_library(pugixml::pugixml ALIAS pugixml)
endif()

set(PROJ_TESTS OFF CACHE BOOL "Disable bundled PROJ tests" FORCE)
set(BUILD_CCT OFF CACHE BOOL "" FORCE)
set(BUILD_CS2CS OFF CACHE BOOL "" FORCE)
set(BUILD_GEOD OFF CACHE BOOL "" FORCE)
set(BUILD_GIE OFF CACHE BOOL "" FORCE)
set(BUILD_NAD2BIN OFF CACHE BOOL "" FORCE)
set(BUILD_PROJ OFF CACHE BOOL "" FORCE)
add_subdirectory(${RSIM_THIRD_PARTY_SRC}/PROJ
                 ${CMAKE_BINARY_DIR}/_third_party/PROJ EXCLUDE_FROM_ALL)
add_subdirectory(${RSIM_THIRD_PARTY_SRC}/OpenDriveParser
                 ${CMAKE_BINARY_DIR}/_third_party/OpenDriveParser EXCLUDE_FROM_ALL)
target_include_directories(OpenDriveParser PUBLIC
    ${RSIM_THIRD_PARTY_SRC}/utils
    ${RSIM_THIRD_PARTY_SRC}/spdlog/include)

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
