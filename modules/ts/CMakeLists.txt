# ----------------------------------------------------------------------------
#  CMake file for ts module with test utilities
# ----------------------------------------------------------------------------

set (MODULE_NAME ts)

project (${MODULE_NAME})

file (GLOB SRC_LIST *.c*)
file (GLOB HEADERS  *.h*)

add_library (${MODULE_NAME} ${SRC_LIST} ${HEADERS})
target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS})

