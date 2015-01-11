# ----------------------------------------------------------------------------
#  CMake file for core module
# ----------------------------------------------------------------------------

set(MODULE_NAME ${ROOT_PROJECT}_core)

project(${MODULE_NAME})

file(GLOB_RECURSE SRC_LIST *.c*)
file(GLOB_RECURSE HEADERS  *.h*)

# Filter out test related sources
set(REGEX_EXCLUDE_FROM_MODULE "(.*test.*)|(.*~.*)")
list_filterout(SRC_LIST ${REGEX_EXCLUDE_FROM_MODULE})
list_filterout(HEADERS  ${REGEX_EXCLUDE_FROM_MODULE})

add_library(${MODULE_NAME} ${SRC_LIST} ${HEADERS})
target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS})

list(APPEND ${ROOT_PROJECT}_MODULES ${MODULE_NAME})
set(${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)
