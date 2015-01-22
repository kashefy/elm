# - Config file for the SEM package
# ===================================================================================
#  The SEM CMake configuration file
#  Config file for the SEM package
#
#  Usage from an external project:
#    In your project's CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(SEM REQUIRED)
#	 include_directories(${SEM_INCLUDE_DIRS})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${SEM_LIBS})
#
#    Searching for specific SEM modules not supported yet.
#
#    If the packages is found then SEM_FOUND is set to TRUE/ON.
#
#    This file will define the following variables:
#      - SEM_LIBS                     : The list of libraries to link against.
#      - SEM_LIB_DIR                  : The directory(es) where lib files are. Calling LINK_DIRECTORIES
#                                          with this path is NOT needed.
#      - SEM_INCLUDE_DIRS             : The SEM include directories.
#	 
#		TODO: define and expose version variables
#      - SEM_VERSION                  : The version of this package build. Example: "2.4.0"
#      - SEM_VERSION_MAJOR            : Major version part of SEM_VERSION. Example: "2"
#      - SEM_VERSION_MINOR            : Minor version part of SEM_VERSION. Example: "4"
#      - SEM_VERSION_PATCH            : Patch version part of SEM_VERSION. Example: "0"
#
# ===================================================================================
#

cmake_minimum_required(VERSION 2.8.7)

set(SEM_FOUND FALSE)
 
# Compute paths
get_filename_component(SEM_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH CACHE)

get_filename_component(PARENT_DIR ${SEM_CMAKE_DIR} PATH)

message(STATUS "PARENT_DIR= ${PARENT_DIR}")
# Are we in install prefix or in a build directory?
if(EXISTS ${PARENT_DIR}/modules AND IS_DIRECTORY ${PARENT_DIR}/modules)
	
	# install prefix
	set(SEM_INCLUDE_DIRS ${PARENT_DIR}/modules)
	set(SEM_LIB_DIR ${PARENT_DIR}/lib)
	
elseif(EXISTS ${PARENT_DIR}/../include AND IS_DIRECTORY ${PARENT_DIR}/../include)
	
	#build directory
	set(SEM_INCLUDE_DIRS ${PARENT_DIR}/../include)
	set(SEM_LIB_DIR ${PARENT_DIR}/../lib)

else()

	message(STATUS "???")

endif()
 
file(GLOB SEM_LIBS "${SEM_LIB_DIR}/*.a")

# verify it's not an empty lib directory
list(LENGTH SEM_LIBS SEM_NB_LIBS)

if(${SEM_NB_LIBS})
	set(SEM_FOUND TRUE)
else(${SEM_NB_LIBS})
	set(SEM_FOUND FALSE)
    message (FATAL_ERROR "Identified ${SEM_NB_LIBS} sem libs.")
endif(${SEM_NB_LIBS})

# TODO: instead of duplicating, order them from depdendent to independent
list(APPEND ${SEM_LIBS} ${SEM_LIBS})
