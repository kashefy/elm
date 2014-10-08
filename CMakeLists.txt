# ----------------------------------------------------------------------------
#  Root CMake file for SEM
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.8)

# ----------------------------------------------------------------------------
# Define root variables
# ----------------------------------------------------------------------------
set(ROOT_PROJECT sem)
project(${ROOT_PROJECT})
set(${ROOT_PROJECT}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/modules)
set(${ROOT_PROJECT}_LIBS "")
set(${ROOT_PROJECT}_MODULES "")


# ----------------------------------------------------------------------------
# Set default behavior
# ----------------------------------------------------------------------------
set(BUILD_TESTS ON CACHE BOOL "Whether to build test projects.")

include(cmake/FileSystemUtils.cmake)

# ----------------------------------------------------------------------------
# Set global CMake variables
# ----------------------------------------------------------------------------
# Print all warnings
# With C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

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

# ----------------------------------------------------------------------------
# LOCATE DEPENDENCIES: GTest, OpenCV
# ----------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# GTest package
# ----------------------------------------------------------------------------
# NOTE: This needs to happen before OpenCV as OpenCV introduced a bug:
# Their ts module results in multiple definitions at linking time
#
# DEFINES: GTEST_FOUND, GTEST_INCLUDE_DIRS, GTEST_BOTH_LIBRARIES, GTEST_LIBRARIES (libgtest), GTEST_MAIN_LIBRARIES (liggtest_main)
# INPUTS:  GTEST_ROOT, GTEST_MSVC_SEARCH
# For details, look for FindGTest.cmake located in the cmake folder: /usr/share/cmake28/Modules/FindGTest.cmake
# ----------------------------------------------------------------------------

if(BUILD_TESTS)
    message(STATUS "GTEST_ROOT=${GTEST_ROOT}")
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
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
# Force the user to tell us which OpenCV they want (otherwise find_package can find the wrong one, cache it and changes to OpenCV_DIR are ignored)

if(DEFINED OpenCV_DIR)
    find_package(OpenCV REQUIRED core PATHS ${OpenCV_DIR})
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

include_directories(${${ROOT_PROJECT}_INCLUDE_DIRS})

add_subdirectory(modules)
add_subdirectory(samples)

# ----------------------------------------------------------------------------
# Test Projects
# ----------------------------------------------------------------------------
if(BUILD_TESTS)

    message(STATUS "Configure unittest project")
    include(cmake/BuildTests.cmake)
    build_tests("run_unittests" "*unittest*")
    add_definitions(-D__WITH_GTEST) # Useful in case test class needs to be friend of class under test

    # Copy over test data (if it exists)
    if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/test-data")

        status("Copying test-data")
        file(COPY ${CMAKE_SOURCE_DIR}/testdata DESTINATION ${EXECUTABLE_OUTPUT_PATH})

    endif(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/test-data")

endif(BUILD_TESTS)
