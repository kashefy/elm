# ----------------------------------------------------------------------------
#  Root CMake file for SEM
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.7)

# ----------------------------------------------------------------------------
# Define root variables
# ----------------------------------------------------------------------------
set(ROOT_PROJECT sem)
project(${ROOT_PROJECT})
set(${ROOT_PROJECT}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/modules) # enforce includes to start with project's parent dir (e.g. #include <sem/core/core.h>)
set(${ROOT_PROJECT}_LIBS    "")
set(${ROOT_PROJECT}_MODULES "")
set(${ROOT_PROJECT}_CONFIG cmake/SEMConfig.cmake)

# ----------------------------------------------------------------------------
# Set default behavior
# ----------------------------------------------------------------------------
set(BUILD_TESTS ON CACHE BOOL "Whether to build test projects.")
set(FORCE_IN_SOURCE OFF CACHE BOOL "Whether to force allow in-source builds.")

include(cmake/FileSystemUtils.cmake)
include(cmake/ListUtils.cmake)
include(cmake/MessageUtils.cmake)

# ----------------------------------------------------------------------------
# Set global CMake variables
# ----------------------------------------------------------------------------
# Print all warnings and enable C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
set(CMAKE_VERBOSE ON CACHE BOOL "Verbose mode")

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
#
# DEFINES: GTEST_FOUND, GTEST_INCLUDE_DIRS, GTEST_BOTH_LIBRARIES, GTEST_LIBRARIES (libgtest), GTEST_MAIN_LIBRARIES (liggtest_main)
# INPUTS:  GTEST_ROOT, GTEST_MSVC_SEARCH
# For details, look for FindGTest.cmake located in the cmake folder: /usr/share/cmake28/Modules/FindGTest.cmake
# ----------------------------------------------------------------------------

if(BUILD_TESTS)
    status("GTEST_ROOT=${GTEST_ROOT}")
    find_package(GTest REQUIRED)
    if(GTEST_FOUND)
        list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
        list(APPEND ${ROOT_PROJECT}_LIBS ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    else(GTEST_FOUND)
        if(NOT DEFINED GTEST_ROOT)
            set(GTEST_ROOT "" CACHE PATH "Root directory for GTEST in source build. Typically ../gmock-1.7.0/gtest")
        endif(NOT DEFINED GTEST_ROOT)
        message(FATAL_ERROR "Failed to find GoogleTest. Double check that \"GTEST_ROOT\" is properly set")
    endif(GTEST_FOUND)
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
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
# Force the user to tell us which OpenCV they want (otherwise find_package can find the wrong one, cache it and changes to OpenCV_DIR are ignored)

if(DEFINED OpenCV_DIR)
    find_package(OpenCV REQUIRED core highgui imgproc PATHS ${OpenCV_DIR})
    if(OpenCV_FOUND)
        list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
        list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS})
    else(OpenCV_FOUND)
        message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" to the root build directory of OpenCV.")
    endif(OpenCV_FOUND)
else(DEFINED OpenCV_DIR)
    set(OpenCV_DIR "" CACHE PATH "Root directory for OpenCV build directory." )
    message(FATAL_ERROR "\"OpenCV_DIR\" not set. Please explicitly provide the path to the root build directory of OpenCV.")
endif(DEFINED OpenCV_DIR)

# ----------------------------------------------------------------------------
# Boost
# ----------------------------------------------------------------------------
# Whatever STL can't do yet
#
# File system operations
# serialization
#
# platform independence
#
# If a feature does appear in STL, check it out and migrate over.
# ----------------------------------------------------------------------------
#
# find_package Boost
# Defines: Boost_FOUND, Boost_LIBRARIES,...
# ----------------------------------------------------------------------------
# Force the user to tell us which OpenCV they want (otherwise find_package can find the wrong one, cache it and changes to OpenCV_DIR are ignored)

set(BOOST_MIN_VERSION "1.56.0" CACHE STRING "Minimum version of boost to link against (e.g. C:/BOOST_1_56_0 is 1.56.0")

# Don't forget to include 'system'
set(BOOST_COMPONENTS system filesystem serialization)
status("")
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS ${BOOST_COMPONENTS})
if(Boost_FOUND)
    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
    list(APPEND ${ROOT_PROJECT}_LIBS ${Boost_LIBRARIES})
else(Boost_FOUND)
    if(NOT DEFINED BOOST_ROOT)
        set(BOOST_ROOT "" CACHE PATH "Boost installation prefix.")
    endif(NOT DEFINED BOOST_ROOT)
    message(FATAL_ERROR "Failed to find Boost (or missing components). Check that \"BOOST_ROOT\" is properly set")
endif(Boost_FOUND)

# Required dependencies covered\
# ----------------------------------------------------------------------------
# Locate Optional dependencies: PCL
# ----------------------------------------------------------------------------

status("Resolve Opt. Dependencies:")

# ----------------------------------------------------------------------------
# Point Cloud Library (PCL)
# ----------------------------------------------------------------------------
# Iterative closest point
# ----------------------------------------------------------------------------
# the components we're currently interested in:
#     common
#     io            - loading pcd files
#     registration  - icp
#     features      - triangulation
#     kdtree        - triangulation
#     surface       - triangulation
# find_package PCL
# Defines:
#     PCL_FOUND:        set to 1 if PCL is found, otherwise unset
#     PCL_INCLUDE_DIRS: set to the paths to PCL installed headers and the dependency headers
#     PCL_LIBRARIES:    set to the file names of the built and installed PCL libraries
#     PCL_LIBRARY_DIRS: set to the paths to where PCL libraries and 3rd party dependencies reside
#     PCL_VERSION:      the version of the found PCL
#     PCL_COMPONENTS:   lists all available components
# ----------------------------------------------------------------------------
set(WITH_PCL ON CACHE BOOL "Whether to build with PCL support.")

if(WITH_PCL)
    set(PCL_COMPONENTS_REQ common io registration features kdtree surface)
    status("Find PCL:")
    find_package(PCL 1.7 REQUIRED COMPONENTS ${PCL_COMPONENTS_REQ})
    if(PCL_FOUND)   # true
        add_definitions(-D__WITH_PCL) # Useful for conditional building of PCL dependent functionality
        list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
        list(APPEND ${ROOT_PROJECT}_LIBS ${PCL_LIBRARIES})
    else(PCL_FOUND) # false
        message(FATAL_ERROR "Failed to find PCL (or missing components). Check that the library is properly installed")
    endif(PCL_FOUND)
    status("")
endif(WITH_PCL)

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

    status("    Configure unittest project")
    include(cmake/BuildTests.cmake)
    build_tests("run_unittests" "*unittest*")
    add_definitions(-D__WITH_GTEST) # Useful in case test class needs to be friend of class under test

    # Copy over test data (if it exists)
    if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/testdata")

        status("Copying test data")
        file(COPY ${CMAKE_SOURCE_DIR}/testdata DESTINATION ${EXECUTABLE_OUTPUT_PATH})

    endif(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/testdata")

endif(BUILD_TESTS)

#----------------------------------------------------------------------------
# Copy instructions
#----------------------------------------------------------------------------

file(COPY ${${ROOT_PROJECT}_CONFIG} DESTINATION ${CMAKE_BINARY_DIR})



#----------------------------------------------------------------------------
# installation targets that aren't already performed on the module level
#----------------------------------------------------------------------------

install(FILES ${${ROOT_PROJECT}_CONFIG} DESTINATION share/${ROOT_PROJECT}/)


# ----------------------------------------------------------------------------
# Done configuring the build.
# Let's print some summary info
# ----------------------------------------------------------------------------

status("")

status("Build:")
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
status("        lib Dir.:  "   OpenCV_FOUND THEN "${OpenCV_DIR}/lib" ELSE "-")
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
status("        components.:  "   PCL_FOUND THEN "${PCL_COMPONENTS_REQ}" ELSE "-")

status("")

status("paths:")
status("    install prefix:\t${CMAKE_INSTALL_PREFIX}")
status("    test-data     :\t${EXECUTABLE_OUTPUT_PATH}/test-data")

status("")
