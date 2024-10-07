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
find_package(MRPT 2.1.0 REQUIRED COMPONENTS gui obs maps topography)

# define lib:
set(LIB_SRCS
  src/factors/FactorBase.cpp
  src/factors/FactorRelativePose3.cpp
  src/factors/SmartFactorIMU.cpp
  src/factors/SmartFactorStereoProjectionPose.cpp
  src/factors/FactorConstVelKinematics.cpp
  src/factors/FactorStereoProjectionPose.cpp
  src/register.cpp
  src/WorldModel.cpp
  src/interfaces/NavStateFilter.cpp
  src/interfaces/RawDataSourceBase.cpp
  src/interfaces/Dataset_UI.cpp
  src/interfaces/FrontEndBase.cpp
  src/interfaces/BackEndBase.cpp
  src/interfaces/FilterBase.cpp
  src/interfaces/ExecutableBase.cpp
  src/interfaces/Relocalization.cpp
  src/interfaces/MapServer.cpp
  src/Entity.cpp
  src/Factor.cpp
  src/entities/RelPose3.cpp
  src/entities/RefPose3.cpp
  src/entities/RelDynPose3KF.cpp
  src/entities/LandmarkPoint3.cpp
  src/entities/EntityBase.cpp
  src/entities/RelPose3KF.cpp
  src/pretty_print_exception.cpp
  src/LazyLoadResource.cpp
)

set(LIB_PUBLIC_HDRS
  include/mola_kernel/factors/FactorConstVelKinematics.h
  include/mola_kernel/factors/SmartFactorStereoProjectionPose.h
  include/mola_kernel/factors/FactorStereoProjectionPose.h
  include/mola_kernel/factors/SmartFactorIMU.h
  include/mola_kernel/factors/FactorRelativePose3.h
  include/mola_kernel/factors/factors-common.h
  include/mola_kernel/factors/FactorBase.h
  include/mola_kernel/LazyLoadResource.h
  include/mola_kernel/pretty_print_exception.h
  include/mola_kernel/WorldModel.h
  include/mola_kernel/variant_helper.h
  include/mola_kernel/FastAllocator.h
  include/mola_kernel/Entity.h
  include/mola_kernel/interfaces/BackEndBase.h
  include/mola_kernel/interfaces/FrontEndBase.h
  include/mola_kernel/interfaces/VizInterface.h
  include/mola_kernel/interfaces/ExecutableBase.h
  include/mola_kernel/interfaces/Relocalization.h
  include/mola_kernel/interfaces/MapServer.h
  include/mola_kernel/interfaces/NavStateFilter.h
  include/mola_kernel/interfaces/FilterBase.h
  include/mola_kernel/interfaces/RawDataSourceBase.h
  include/mola_kernel/interfaces/RawDataConsumer.h
  include/mola_kernel/interfaces/OfflineDatasetSource.h
  include/mola_kernel/interfaces/LocalizationSourceBase.h
  include/mola_kernel/interfaces/MapSourceBase.h
  include/mola_kernel/interfaces/Dataset_UI.h
  include/mola_kernel/Factor.h
  include/mola_kernel/entities/KeyFrameBase.h
  include/mola_kernel/entities/EntityBase.h
  include/mola_kernel/entities/RelPose3KF.h
  include/mola_kernel/entities/RelPose3.h
  include/mola_kernel/entities/EntityRelativeBase.h
  include/mola_kernel/entities/RelDynPose3KF.h
  include/mola_kernel/entities/LandmarkPoint3.h
  include/mola_kernel/entities/RefPose3.h
  include/mola_kernel/entities/entities-common.h
  include/mola_kernel/id.h
  include/mola_kernel/Yaml.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::obs
    mrpt::gui
    mrpt::topography
    mola::mola_yaml
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt-obs
    mrpt-gui
    mrpt-topography
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
