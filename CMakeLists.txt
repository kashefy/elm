# ----------------------------------------------------------------------------
#  Root CMake file for SEM
# ----------------------------------------------------------------------------

cmake_minimum_required (VERSION 2.8.11)

set (ROOT_PROJECT sem)
project (${ROOT_PROJECT})
set (${ROOT_PROJECT}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/modules) 
set (${ROOT_PROJECT}_LIBS "")
set (${ROOT_PROJECT}_MODULES "")

include (cmake/FileSystemUtils.cmake)

# Include all the directories
include_directories (${${ROOT_PROJECT}_INCLUDE_DIRS})

add_subdirectory (modules)
add_subdirectory (samples)
