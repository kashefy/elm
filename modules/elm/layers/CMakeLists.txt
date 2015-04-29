# ----------------------------------------------------------------------------
#  CMake file for layers module
# ----------------------------------------------------------------------------

set (MODULE_NAME ${ROOT_PROJECT}_layers)

project (${MODULE_NAME})

file(GLOB_RECURSE SRC_LIST *.c*)
file(GLOB_RECURSE HEADERS  *.h*)

# Filter out test related sources
set(REGEX_EXCLUDE_FROM_MODULE "(.*test.*)|(.*~.*)|(.*~)")
list_filterout(SRC_LIST ${REGEX_EXCLUDE_FROM_MODULE})
list_filterout(HEADERS  ${REGEX_EXCLUDE_FROM_MODULE})

add_library (${MODULE_NAME} ${SRC_LIST} ${HEADERS})

add_dependencies(${MODULE_NAME} ${ROOT_PROJECT}_core ${ROOT_PROJECT}_neuron ${ROOT_PROJECT}_encoding)
target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS} ${ROOT_PROJECT}_core ${ROOT_PROJECT}_neuron ${ROOT_PROJECT}_encoding)

list(APPEND ${ROOT_PROJECT}_MODULES ${ROOT_PROJECT}_core ${ROOT_PROJECT}_neuron ${ROOT_PROJECT}_encoding ${MODULE_NAME})
set(${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)

# add module's install targets, header installation is centralized
install(TARGETS ${MODULE_NAME} DESTINATION lib)
