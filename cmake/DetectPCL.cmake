# ----------------------------------------------------------------------------
# Point Cloud Library (PCL)
# ----------------------------------------------------------------------------
# Iterative closest point, triangulation, point clouds
# ----------------------------------------------------------------------------
# find_package PCL
# Defines:
#     PCL_FOUND:        set to 1 if PCL is found, otherwise unset
#     PCL_INCLUDE_DIRS: set to the paths to PCL installed headers and the dependency headers
#     PCL_LIBRARIES:    set to the file names of the built and installed PCL libraries
#     PCL_LIBRARY_DIRS: set to the paths to where PCL libraries and 3rd party dependencies reside
#     PCL_VERSION:      the version of the found PCL
#     PCL_COMPONENTS:   lists all available components
# ----------------------------------------------------------------------------

set(PCL_COMPONENTS_REQ common io registration features kdtree surface)
status("Find PCL:")
find_package(PCL 1.7 REQUIRED COMPONENTS ${PCL_COMPONENTS_REQ})
if(PCL_FOUND)   # true
	add_definitions(-D__WITH_PCL) # Useful for conditional building of PCL dependent functionality
	list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
	list(APPEND ${ROOT_PROJECT}_LIBS ${PCL_LIBRARIES})
else(PCL_FOUND) # false
	message(FATAL_ERROR "Failed to find PCL (or missing components). Check that the library is properly installed")
endif(PCL_FOUND)
status("")
