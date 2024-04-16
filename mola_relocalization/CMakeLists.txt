# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2024, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_relocalization LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt-obs REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mp2p_icp REQUIRED)
find_package(mola_pose_list REQUIRED)

find_package(mola_test_datasets) # optional

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::obs
		mrpt::maps
		mola::mp2p_icp
		mola::mola_pose_list
#	PRIVATE_LINK_LIBRARIES
#		mrpt::obs
	CMAKE_DEPENDENCIES
		mola_common
		mp2p_icp
		mola_pose_list
)

# -----------------------
# define tests:
enable_testing()
add_subdirectory(tests)
