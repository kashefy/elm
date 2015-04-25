# ----------------------------------------------------------------------------
#  Root CMake file for bindings
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.7)

if(BUILD_python)
    add_subdirectory(python)
endif(BUILD_python)

# Propagate the targets up to parent level
set(${ROOT_PROJECT}_BIND ${${ROOT_PROJECT}_BIND} PARENT_SCOPE)

