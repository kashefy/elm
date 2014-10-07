# ----------------------------------------------------------------------------
#  CMake file for core module
# ----------------------------------------------------------------------------

set(MODULE_NAME core)

project(${MODULE_NAME})

file(GLOB SRC_LIST *.c*)
file(GLOB HEADERS  *.h*)

add_library(${MODULE_NAME} ${SRC_LIST} ${HEADERS})

list(APPEND ${ROOT_PROJECT}_MODULES ${MODULE_NAME})
set(${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)
