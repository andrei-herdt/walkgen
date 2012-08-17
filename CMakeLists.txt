# Copyright 2012, Olivier Stasse, JRL, CNRS/AIST
# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.8)


# Define properties of the project
SET(PROJECT_NAME mpc-walkgen)
SET(PROJECT_DESCRIPTION "MPC Walking pattern generator for Humanoids")
SET(PROJECT_URL "https://gforge.inria.fr/projects/mpc-walkgen")
SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(CXX_DISABLE_WERROR True)

find_package(jrl-cmake REQUIRED
    HINTS cmake)
INCLUDE(${jrl-cmake_DIR}/base.cmake)
INCLUDE(${jrl-cmake_DIR}/cpack.cmake)
INCLUDE(cmakeutils.cmake)
# custom cmake modules to find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

# Initialize generic properties of the project
SETUP_PROJECT()

# Handle OS specificities
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
IF(SYS_TIME_H)
  ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
ENDIF(SYS_TIME_H)
# make all symbols hidden with gcc.
if(UNIX)
  ADD_DEFINITIONS("-fvisibility=hidden")
endif()

find_package(qibuild QUIET)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Optional dependencies: use an external QP solver library.
find_optional_package(
  LSSOL "build mpc-walkgen with support for LSSOL QP solver ")
find_optional_package(
  QPOASES "build mpc-walkgen with support for qpOASES QP solver")

if(NOT WITH_LSSOL AND NOT WITH_QPOASES)
  message(FATAL_ERROR
    "No QP solver found.  Possible solvers: LSSOL and qpOASES")
endif()

if(WITH_QPOASES)
  # There are two versions of qpOASES, that change the way the inclusion
  # has to be done.
  if(NOT QPOASES_VERSION VERSION_LESS 3.0)
    ADD_DEFINITIONS("-DUSE_QPOASES_3_0")
  endif()
endif()

# Define the set of headers to be installed
SET(${PROJECT_NAME}_HEADERS
  include/mpc-walkgen/walkgen-abstract.h
  include/mpc-walkgen/sharedtypes.h
)

SET(mpc-walkgen_SRC
  src/types.cpp
  include/mpc-walkgen/types.h
  src/gettimeofday.cpp
  include/mpc-walkgen/gettimeofday.h
  src/interpolation.cpp
  include/mpc-walkgen/interpolation.h
  src/mpc-debug.cpp
  include/mpc-walkgen/mpc-debug.h
  src/qp-matrix.cpp
  include/mpc-walkgen/qp-matrix.h
  src/qp-solver.cpp
  include/mpc-walkgen/qp-solver.h
  src/lssol-parser.cpp
  include/mpc-walkgen/lssol-parser.h
  src/qpoases-parser.cpp
  include/mpc-walkgen/qpoases-parser.h
  src/qp-vector.cpp
  include/mpc-walkgen/qp-vector.h
  src/tools.cpp
  include/mpc-walkgen/tools.h
  include/mpc-walkgen/tools-inl.h
  src/sharedtypes.cpp
  include/mpc-walkgen/sharedtypes.h
  src/orientations-preview.cpp
  include/mpc-walkgen/orientations-preview.h
  src/qp-generator.cpp
  include/mpc-walkgen/qp-generator.h
  src/qp-preview.cpp
  include/mpc-walkgen/qp-preview.h
  src/com-body.cpp
  include/mpc-walkgen/com-body.h
  src/foot-body.cpp
  include/mpc-walkgen/foot-body.h
  src/rigid-body.cpp
  include/mpc-walkgen/rigid-body.h
  src/rigid-body-system.cpp
  include/mpc-walkgen/rigid-body-system.h
  src/state-fsm.cpp
  include/mpc-walkgen/state-fsm.h
  src/types.cpp
  include/mpc-walkgen/types.h
  src/walkgen.cpp
  include/mpc-walkgen/walkgen.h
  src/sharedtypes.cpp
  src/walkgen-abstract.cpp
)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/include)

ADD_LIBRARY(mpc-walkgen SHARED ${mpc-walkgen_SRC} ${${PROJECT_NAME}_HEADERS})
SET_TARGET_PROPERTIES(mpc-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS mpc-walkgen DESTINATION lib)

PKG_CONFIG_APPEND_CFLAGS("-I${EIGEN3_INCLUDE_DIR}")
if(WITH_LSSOL)
  PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen lssol)
endif()

if(WITH_QPOASES)
  PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen qpoases)
endif()

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(unittest)

# Generate dependency to jrl-walkgen in pc file
PKG_CONFIG_APPEND_LIBS("mpc-walkgen")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

