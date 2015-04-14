# ----------------------------------------------------------------------------
#  Root CMake file for the elm project
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.7)

# ----------------------------------------------------------------------------
# Define root variables
# ----------------------------------------------------------------------------
set(ROOT_PROJECT elm)
project(${ROOT_PROJECT})
set(${ROOT_PROJECT}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/modules) # enforce includes to start with project's parent dir (e.g. #include <elm/core/core.h>)
set(${ROOT_PROJECT}_LIBS    "")
set(${ROOT_PROJECT}_MODULES "")

# ----------------------------------------------------------------------------
# Set default behavior
# ----------------------------------------------------------------------------
set(BUILD_TESTS ON CACHE BOOL "Whether to build test projects.")
set(WITH_TEST_COVERAGE OFF CACHE BOOL "Whether to produce test coverage reports.")
set(FORCE_IN_SOURCE OFF CACHE BOOL "Whether to force allow in-source builds.")

include(cmake/CompilerUtils.cmake)
if(WITH_TEST_COVERAGE)
    include(cmake/CodeCoverage.cmake)
endif(WITH_TEST_COVERAGE)
include(cmake/FileSystemUtils.cmake)
include(cmake/ListUtils.cmake)
include(cmake/MessageUtils.cmake)

# ----------------------------------------------------------------------------
# Set global CMake variables
# ----------------------------------------------------------------------------
if(NOT DEFINED CMAKE_BUILD_TYPE OR NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    status("Since no build type was specified falling back on ${CMAKE_BUILD_TYPE} as the default build type.")
endif()

# Print all warnings and enable C++11 support
if(WIN32 AND NOT MSVC) # gtest on windows (MinGW) doesn't do well with C++11
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
else(WIN32 AND NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
endif(WIN32 AND NOT MSVC)

# -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
# position indepdendent code for CMake older 2.8.9
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" STREQUAL "2.8")
    if(${CMAKE_PATCH_VERSION} LESS 9)
        if(LINUX)
            set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fPIC")
        endif(LINUX)
    endif(${CMAKE_PATCH_VERSION} LESS 9)
endif()

set(CMAKE_VERBOSE ON CACHE BOOL "Verbose mode")

# Profiling - appliable to BUILD_TYPE=Profile
SETUP_PROFILING()

# Setup build and install locations
if(NOT RUNTIME_INSTALL_DIRECTORY)
    set(RUNTIME_INSTALL_DIRECTORY "bin" CACHE STRING "Install sub-directory of CMAKE_INSTALL_PREFIX for RUNTIME targets (binaries, and *.dll on windows).")
endif(NOT RUNTIME_INSTALL_DIRECTORY)

if(NOT LIBRARY_INSTALL_DIRECTORY)
    set(LIBRARY_INSTALL_DIRECTORY "lib" CACHE STRING "Install sub-directory of CMAKE_INSTALL_PREFIX for LIBRARY targets (shared libs)")
endif(NOT LIBRARY_INSTALL_DIRECTORY)

if(NOT ARCHIVE_INSTALL_DIRECTORY)
    set(ARCHIVE_INSTALL_DIRECTORY "lib" CACHE STRING "Install sub-directory of CMAKE_INSTALL_PREFIX for ARCHIVE targets (static libs and *.def on windows)")
endif(NOT ARCHIVE_INSTALL_DIRECTORY)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib CACHE PATH "Output directory for libraries")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin CACHE PATH "Output directory for applications")

# Prevent in-source builds
get_filename_component(PATH_A ${CMAKE_SOURCE_DIR} REALPATH)
get_filename_component(PATH_B ${CMAKE_BINARY_DIR} REALPATH)
if(${FORCE_IN_SOURCE} AND (${PATH_A} STREQUAL ${PATH_B}))
    message(FATAL_ERROR "In-source builds not permitted pass FORCE_IN_SOURCE=ON to override.")
endif()

# ----------------------------------------------------------------------------
# Locate Dependencies: GTest, OpenCV, Boost
# ----------------------------------------------------------------------------

status("Resolve Dependencies:")

# ----------------------------------------------------------------------------
# GTest package
# ----------------------------------------------------------------------------
# Testing framework
# ----------------------------------------------------------------------------
# NOTE: This needs to happen before OpenCV as OpenCV introduced a bug:
# Their ts module results in multiple definitions at linking time
# ----------------------------------------------------------------------------
if(BUILD_TESTS)
    include(cmake/DetectGTest.cmake)
endif(BUILD_TESTS)

# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# Matrix definitions operations
# Image processing
# Image visuzalization
# Image and Video I/O
# Exception handling
# ----------------------------------------------------------------------------
include(cmake/DetectOpenCV.cmake)

# ----------------------------------------------------------------------------
# Boost
# ----------------------------------------------------------------------------
# Whatever STL can't do yet
#
# File system operations
# serialization
# graph representation
#
# platform independence
#
# If a feature does appear in STL, check it out and migrate over.
# ----------------------------------------------------------------------------
include(cmake/DetectBoost.cmake)

# Required dependencies covered\
# ----------------------------------------------------------------------------
# Locate Optional dependencies: PCL
# ----------------------------------------------------------------------------

status("Resolve Opt. Dependencies:")

# ----------------------------------------------------------------------------
# Point Cloud Library (PCL)
# ----------------------------------------------------------------------------
# Iterative closest point, triangulation, point clouds
# ----------------------------------------------------------------------------
# the components we're currently interested in:
#     common
#     io            - loading pcd files
#     registration  - icp
#     features      - triangulation
#     kdtree        - triangulation
#     surface       - triangulation
# ----------------------------------------------------------------------------
set(WITH_PCL ON CACHE BOOL "Whether to build with PCL support.")

if(WITH_PCL)
    include(cmake/DetectPCL.cmake)
endif(WITH_PCL)

# ----------------------------------------------------------------------------
#  MAT File I/O Library "MATIO"
# ----------------------------------------------------------------------------
# Reading from MATLAB's .mat files for importing datasets
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
set(WITH_MATIO ON CACHE BOOL "Whether to build with MATIO support.")

if(WITH_MATIO)
    include(cmake/DetectMATIO.cmake)
endif(WITH_MATIO)

# Dependencies covered
# ----------------------------------------------------------------------------

include_directories(${${ROOT_PROJECT}_INCLUDE_DIRS})

add_subdirectory(modules/${ROOT_PROJECT})
add_subdirectory(samples)

# ----------------------------------------------------------------------------
# Test Projects
# ----------------------------------------------------------------------------

status("")
status("Configure test projects:")

if(BUILD_TESTS)

    include(cmake/BuildTests.cmake)
    status("    Configure unittest project")
    build_tests("run_${ROOT_PROJECT}_unittests" "*unittest*")
    
    if(WITH_TEST_COVERAGE)
        setup_target_for_coverage(
            ${ROOT_PROJECT}_unittests_coverage  # Name for custom target.
            run_${ROOT_PROJECT}_unittests       # Name of the test driver executable that runs the tests.
                                                # NOTE! This should always have a ZERO as exit code
                                                # otherwise the coverage generation will not complete.
            coverage # Name of output directory.
		)
    endif(WITH_TEST_COVERAGE)
    
    status("    Configure integrationtest project")
    build_tests("run_${ROOT_PROJECT}_integrationtests" "*integ*test*")

    add_definitions(-D__WITH_GTEST) # Useful in case test class needs to be friend of class under test

    # Copy over test data (if it exists)
    if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/testdata")

        status("Copying test data")
        file(COPY ${CMAKE_SOURCE_DIR}/testdata DESTINATION ${EXECUTABLE_OUTPUT_PATH})

        if(WITH_TEST_COVERAGE)
            file(COPY ${CMAKE_SOURCE_DIR}/testdata DESTINATION ${CMAKE_BINARY_DIR})
        endif(WITH_TEST_COVERAGE)

    endif(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/testdata")

endif(BUILD_TESTS)

#----------------------------------------------------------------------------
# Copy instructions
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
# file configurations
#----------------------------------------------------------------------------

# variable substitutions
set(ROOT_PROJECT_CONFIG elm-config.cmake)

# Set full-name variables for suffix variables, otherwise config won't substitute them:
set(ROOT_PROJECT_INCLUDE_DIRS ${${ROOT_PROJECT}_INCLUDE_DIRS})

set(ROOT_PROJECT_MODULES ${${ROOT_PROJECT}_MODULES})
set(ROOT_PROJECT_DEPS ${${ROOT_PROJECT}_LIBS})

string(TOUPPER ${ROOT_PROJECT} ROOT_PROJECT_UPPER)

configure_file(cmake/${ROOT_PROJECT_CONFIG}.in ${ROOT_PROJECT_CONFIG} @ONLY)

#----------------------------------------------------------------------------
# installation targets that aren't already performed on the module level
#----------------------------------------------------------------------------

install(FILES ${CMAKE_BINARY_DIR}/${ROOT_PROJECT_CONFIG} DESTINATION share/${ROOT_PROJECT}/)

# ----------------------------------------------------------------------------
# Done configuring the build.
# Let's print some summary info
# ----------------------------------------------------------------------------

status("")

status("Build settings:")
status("")
status("    Build type:\t${CMAKE_BUILD_TYPE}")
string(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE_UPPER)
status("    C Flags: (${CMAKE_BUILD_TYPE})\t${CMAKE_C_FLAGS_${BUILD_TYPE_UPPER}}")
status("    C Flags:\t${CMAKE_C_FLAGS}")
status("    C++ Flags:\t${CMAKE_CXX_FLAGS}")
status("    C++ Flags (${CMAKE_BUILD_TYPE}):\t${CMAKE_CXX_FLAGS_${BUILD_TYPE_UPPER}}")

status("")
status("Targets:")
set(MODULE_NAMES ${${ROOT_PROJECT}_MODULES})
strip_lib_prefix(MODULE_NAMES ${MODULE_NAMES})
status("    modules:  "      ${MODULE_NAMES})
status("    unit-tests:  "   BUILD_TESTS THEN YES ELSE NO)

status("")

status("Dependencies:")
status("")
status("    GTest:    "   GTEST_FOUND THEN ${GTEST_ROOT} ELSE NO)
status("")
status("    OpenCV:    "   OpenCV_FOUND THEN "${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH} (${OpenCV_VERSION})" ELSE NO)
status("        lib Dir.:      "   OpenCV_FOUND THEN "${OpenCV_DIR}/lib" ELSE "-")
status("        include Dir.:  "   OpenCV_FOUND THEN "${OpenCV_INCLUDE_DIRS}" ELSE "-")
set(OpenCV_MODULES ${OpenCV_LIBS})
convert_to_lib_name(OpenCV_MODULES ${OpenCV_MODULES})
status("        modules :  "      OpenCV_FOUND THEN ${OpenCV_MODULES} ELSE "-")
status("")
status("    Boost:    "   Boost_FOUND THEN "${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION} (${Boost_VERSION})"  ELSE NO)
status("        lib Dir.:  "   Boost_FOUND THEN "${BOOST_ROOT}/lib" ELSE "-")
set(Boost_LIB_NAMES ${Boost_LIBRARIES})
convert_to_lib_name(Boost_LIB_NAMES ${Boost_LIB_NAMES})
status("        components :  "      Boost_FOUND THEN ${Boost_LIB_NAMES} ELSE "-")

status("")

status("Optional Dependencies:")
status("")
status("    PCL:  "   PCL_FOUND THEN YES (${PCL_VERSION}) ELSE NO)
status("        lib Dir.:  "      PCL_FOUND THEN "${PCL_LIBRARY_DIRS}" ELSE "-")
status("        components :  "   PCL_FOUND THEN "${PCL_COMPONENTS_REQ}" ELSE "-")

status("")
status("    MATIO:  "   MATIO_FOUND THEN YES ELSE NO)
status("        include Dir.:  "  MATIO_FOUND THEN "${MATIO_INCLUDE_DIRS}" ELSE "-")
status("        lib Dir.:  "      MATIO_FOUND THEN "${MATIO_LIBRARY_DIR}" ELSE "-")
status("        components :  "   MATIO_FOUND THEN "${MATIO_LIBS}" ELSE "-")
status("        hdf5 :  "   MATIO_FOUND THEN "${HDF5_DIR}" ELSE "-")
status("            components :  "   MATIO_FOUND THEN "${HDF5_LIBS}" ELSE "-")

status("")

status("paths:")
status("    install prefix:\t${CMAKE_INSTALL_PREFIX}")
status("    test-data     :\t${EXECUTABLE_OUTPUT_PATH}/test-data")

status("")
