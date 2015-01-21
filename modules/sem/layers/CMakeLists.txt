# ----------------------------------------------------------------------------
#  CMake file for layers module
# ----------------------------------------------------------------------------

set (MODULE_NAME ${ROOT_PROJECT}_layers)

project (${MODULE_NAME})

file (GLOB SRC_LIST *.c*)
file (GLOB HEADERS  *.h*)

add_library (${MODULE_NAME} ${SRC_LIST} ${HEADERS})

list (APPEND ${ROOT_PROJECT}_MODULES ${MODULE_NAME})
add_dependencies(${MODULE_NAME} ${ROOT_PROJECT}_core ${ROOT_PROJECT}_encoding)
target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS} ${ROOT_PROJECT}_core ${ROOT_PROJECT}_encoding)
set (${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)

# add module's install targets
install(TARGETS ${MODULE_NAME} DESTINATION lib)
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "test" EXCLUDE)
