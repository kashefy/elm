# ----------------------------------------------------------------------------
#  CMake file for encoding module
# ----------------------------------------------------------------------------

set (MODULE_NAME ${ROOT_PROJECT}_encoding)

project (${MODULE_NAME})

file (GLOB SRC_LIST *.c*)
file (GLOB HEADERS  *.h*)

add_library (${MODULE_NAME} ${SRC_LIST} ${HEADERS})

list (APPEND ${ROOT_PROJECT}_MODULES ${MODULE_NAME})
add_dependencies(${MODULE_NAME} ${ROOT_PROJECT}_core)
target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS} ${ROOT_PROJECT}_core)
set (${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)

# add module's install targets
install(TARGETS ${MODULE_NAME} DESTINATION lib)
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "test" EXCLUDE)
