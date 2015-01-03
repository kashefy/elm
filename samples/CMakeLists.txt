# ----------------------------------------------------------------------------
#  CMake file for sample projects
# ----------------------------------------------------------------------------

# Scan the project sub-folder for a list of potential sample projects
SUBDIRLIST(PROJECT_LIST ${CMAKE_CURRENT_SOURCE_DIR})
list(LENGTH PROJECT_LIST NB_PROJECTS)

if(${NB_PROJECTS})
    message (STATUS "Identified ${NB_PROJECTS} sample project to build.")
else(${NB_PROJECTS})
    message (STATUS "Identified ${NB_PROJECTS} sample projects to build.")
endif(${NB_PROJECTS})

foreach( SUB_PROJECT ${PROJECT_LIST} )

    # Set up each sub folder as a new executable
    message(STATUS "Setting up sample project: ${SUB_PROJECT}")
    project(${SUB_PROJECT})
    file(GLOB_RECURSE SRCS RELATIVE    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_PROJECT}/*.c*)
    file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_PROJECT}/*.h*)
    add_executable(${SUB_PROJECT} ${SRCS} ${HDRS})
    add_dependencies(${SUB_PROJECT} ${${ROOT_PROJECT}_MODULES})
    target_link_libraries(${SUB_PROJECT} ${${ROOT_PROJECT}_LIBS} ${${ROOT_PROJECT}_MODULES})
   
endforeach()
