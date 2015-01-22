# - Config file for the Elm package
# ===================================================================================
#  The Elm CMake configuration file
#  Config file for the Elm package
#
#  Usage from an external project:
#    In your project's CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(Elm REQUIRED)
#	 include_directories(${Elm_INCLUDE_DIRS})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${Elm_LIBS})
#
#    Searching for specific Elm modules not supported yet.
#
#    If the packages is found then Elm_FOUND is set to TRUE/ON.
#
#    This file will define the following variables:
#      - Elm_LIBS                     : The list of libraries to link against.
#      - Elm_LIB_DIR                  : The directory(es) where lib files are. Calling LINK_DIRECTORIES
#                                          with this path is NOT needed.
#      - Elm_INCLUDE_DIRS             : The Elm include directories.
#	 
#		TODO: define and expose version variables
#      - Elm_VERSION                  : The version of this package build. Example: "2.4.0"
#      - Elm_VERSION_MAJOR            : Major version part of Elm_VERSION. Example: "2"
#      - Elm_VERSION_MINOR            : Minor version part of Elm_VERSION. Example: "4"
#      - Elm_VERSION_PATCH            : Patch version part of Elm_VERSION. Example: "0"
#
# ===================================================================================
#

cmake_minimum_required(VERSION 2.8.7)

set(Elm_FOUND FALSE)
 
# Compute paths
get_filename_component(Elm_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH CACHE)

get_filename_component(PARENT_DIR ${Elm_CMAKE_DIR} PATH)

message(STATUS "PARENT_DIR= ${PARENT_DIR}")
# Are we in install prefix or in a build directory?
if(EXISTS ${PARENT_DIR}/modules AND IS_DIRECTORY ${PARENT_DIR}/modules)
	
	#build directory
	set(Elm_INCLUDE_DIRS ${PARENT_DIR}/modules)
	set(Elm_LIB_DIR ${PARENT_DIR}/lib)
	
elseif(EXISTS ${PARENT_DIR}/../include AND IS_DIRECTORY ${PARENT_DIR}/../include)
	
	# install prefix
	set(Elm_INCLUDE_DIRS ${PARENT_DIR}/../include)
	set(Elm_LIB_DIR ${PARENT_DIR}/../lib)

else()

	message(STATUS "???")

endif()
 
file(GLOB Elm_LIBS "${Elm_LIB_DIR}/*.a")

# verify it's not an empty lib directory
list(LENGTH Elm_LIBS Elm_NB_LIBS)

if(${Elm_NB_LIBS})
	set(Elm_FOUND TRUE)
else(${Elm_NB_LIBS})
	set(Elm_FOUND FALSE)
    message (FATAL_ERROR "Identified ${Elm_NB_LIBS} Elm libs.")
endif(${Elm_NB_LIBS})

# TODO: instead of duplicating, order them from depdendent to independent
list(APPEND ${Elm_LIBS} ${Elm_LIBS})
