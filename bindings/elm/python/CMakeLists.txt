# --------------
# CMake file for python support
#
# Expects the following variables to be set
#
# BIND_PYTHON_INCLUDE_DIRS ///< include directories required by bindings (e.g. numpy, python includes)
# BIND_PYTHON_LIBS         ///< libraries required by bindings (e.g. boost.python, libpython)
#
# Appends project name (e.g. python) to ${ROOT_PROJECT}_BIND in parent scope
# --------------
cmake_minimum_required(VERSION 2.6)

set(BIND_NAME python)
 
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  message( ERROR "Cannot build python bindings around a win32 debug build." )
endif()
if(ANDROID OR IOS)
  message( ERROR "Cannot build python bindings around a mobile build." )
endif(ANDROID OR IOS)
if(NOT PYTHONLIBS_FOUND OR NOT PYTHON_USE_NUMPY)
  message( ERROR "Cannot build python bindings without python libs, numpy, boost.python." )
endif()

project(${BIND_NAME})

file(GLOB SRC_LIST *.c*)
file(GLOB HEADERS  *.h*)

# Filter out test related sources
set(REGEX_EXCLUDE_FROM_MODULE "(.*test.*)|(.*~.*)|(.*cv.cpp)")
list_filterout(SRC_LIST ${REGEX_EXCLUDE_FROM_MODULE})
list_filterout(HEADERS  ${REGEX_EXCLUDE_FROM_MODULE})

add_definitions(-DPYTHON_USE_NUMPY=1)

include_directories(${BIND_PYTHON_INCLUDE_DIRS})

add_library(${BIND_NAME} SHARED ${SRC_LIST} ${HEADERS})

target_link_libraries(${BIND_NAME} ${BIND_PYTHON_LIBS} ${${ROOT_PROJECT}_LIBS} ${${ROOT_PROJECT}_MODULES})

list(APPEND ${ROOT_PROJECT}_BIND ${BIND_NAME})
set(${ROOT_PROJECT}_BIND ${${ROOT_PROJECT}_BIND} PARENT_SCOPE)

execute_process( COMMAND ${PYTHON_EXECUTABLE} -c "import distutils.sysconfig; print distutils.sysconfig.get_config_var('SO')"
                 RESULT_VARIABLE PYTHON_PY_PROCESS
                 OUTPUT_VARIABLE PY_SUFFIX
                 OUTPUT_STRIP_TRAILING_WHITESPACE )

set_target_properties( ${BIND_NAME} PROPERTIES
                       PREFIX ""
                       OUTPUT_NAME ${ROOT_PROJECT}
                       SUFFIX ${PY_SUFFIX} )
                      
