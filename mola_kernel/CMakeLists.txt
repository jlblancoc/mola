# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_kernel LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
find_package(mola_yaml REQUIRED)

# find dependencies:
find_package(MRPT 2.1.0 REQUIRED COMPONENTS gui obs maps)

# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::obs
    mrpt::gui
    mola::mola_yaml
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt-obs
    mrpt-gui
    mola_yaml
)

if("$ENV{VERBOSE}")
  message(STATUS "mola_kernel: Using MRPT version: ${MRPT_VERSION}")
endif()

#----
# Extract version from package.xml
# Example line:" <version>0.3.2</version>"
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MOLA_VERSION ${CMAKE_MATCH_1})
message(STATUS "MOLA version: ${MOLA_VERSION} (detected in package.xml)")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${MOLA_VERSION})
set(MOLA_MAJOR_VERSION ${CMAKE_MATCH_1})
set(MOLA_MINOR_VERSION ${CMAKE_MATCH_2})
set(MOLA_PATCH_VERSION ${CMAKE_MATCH_3})
#----

message(STATUS "MOLA detected version (from package.xml): ${MOLA_VERSION}")

target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_MAJOR_VERSION=${MOLA_MAJOR_VERSION})
target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_MINOR_VERSION=${MOLA_MINOR_VERSION})
target_compile_definitions(${PROJECT_NAME} PUBLIC MOLA_PATCH_VERSION=${MOLA_PATCH_VERSION})
