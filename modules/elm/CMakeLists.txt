# ----------------------------------------------------------------------------
#  Root CMake file for modules
# ----------------------------------------------------------------------------
SUBDIRLIST(MODULES_LIST ${CMAKE_CURRENT_SOURCE_DIR})
foreach(MODULE_NAME ${MODULES_LIST})
    add_subdirectory(${MODULE_NAME})
endforeach()

# modules populate list of modules by prepending their dependencies
# by removing duplicates we preserve the ordering from independent modules preceding dependent ones
list(REMOVE_DUPLICATES ${ROOT_PROJECT}_MODULES)

# Propagate the modules up to parent level
set(${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE)

install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "test" EXCLUDE)
