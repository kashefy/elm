# ----------------------------------------------------------------------------
#  Root CMake file for modules
# ----------------------------------------------------------------------------
SUBDIRLIST(MODULES_LIST ${CMAKE_CURRENT_SOURCE_DIR})
foreach(MODULE_NAME ${MODULES_LIST})
    add_subdirectory(${MODULE_NAME})
endforeach()

# Propagate the modules up to parent level
set( ${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE )

install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "test" EXCLUDE)
