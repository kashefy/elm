# ----------------------------------------------------------------------------
#  CMake file for evaluation module
# ----------------------------------------------------------------------------

set(MODULE_NAME ${ROOT_PROJECT}_eval)

project(${MODULE_NAME})

file(GLOB SRC_LIST *.c*)
file(GLOB HEADERS  *.h*)

add_library(${MODULE_NAME} ${SRC_LIST} ${HEADERS})
target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS})

list(APPEND ${ROOT_PROJECT}_MODULES ${MODULE_NAME})
set(${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)
