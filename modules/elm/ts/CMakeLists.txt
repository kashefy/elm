# ----------------------------------------------------------------------------
#  CMake file for ts module with test utilities
# ----------------------------------------------------------------------------

if(BUILD_TESTS) # This module is built conditionally

	set (MODULE_NAME ${ROOT_PROJECT}_ts)

	project (${MODULE_NAME})

	file (GLOB SRC_LIST *.c*)
	file (GLOB HEADERS  *.h*)

	list(APPEND ${ROOT_PROJECT}_MODULES ${MODULE_NAME})
	add_library (${MODULE_NAME} ${SRC_LIST} ${HEADERS})
	add_dependencies(${MODULE_NAME} ${ROOT_PROJECT}_core ${ROOT_PROJECT}_layers)
	target_link_libraries(${MODULE_NAME} ${${ROOT_PROJECT}_LIBS} ${ROOT_PROJECT}_core ${ROOT_PROJECT}_layers)
	set (${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)

	install(TARGETS ${MODULE_NAME} DESTINATION lib)

endif(BUILD_TESTS)

