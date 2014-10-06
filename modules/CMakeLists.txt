# ----------------------------------------------------------------------------
#  Root CMake file for modules
# ----------------------------------------------------------------------------

add_subdirectory (core)
add_subdirectory (encoding)

# Propagate the modules up to parent level
set( ${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE )
