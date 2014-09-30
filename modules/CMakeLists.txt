# ----------------------------------------------------------------------------
#  Root CMake file for modules
# ----------------------------------------------------------------------------

add_subdirectory (core)

# Propagate the modules up to parent level
set( ${ROOT_PROJECT}_MODULES ${${ROOT_PROJECT}_MODULES} PARENT_SCOPE )