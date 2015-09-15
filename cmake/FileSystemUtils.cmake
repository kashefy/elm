# ----------------------------------------------------------------------------
# CMake Macro for listing sub-directories
# Source: http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
# ----------------------------------------------------------------------------
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

# ----------------------------------------------------------------------------
# Custom CMake Macro to get parent directory and support CMake version before and after 2.8.12
# Info: http://stackoverflow.com/questions/7035734/cmake-parent-directory
# ----------------------------------------------------------------------------
MACRO(GET_PARENT_DIR result curpath)
    # Get parent directory of .pc file
    if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" STREQUAL "2.8"
        AND ${CMAKE_PATCH_VERSION} LESS 12)

        get_filename_component(result ${curpath} PATH)
    else()
        get_filename_component(result ${curpath} DIRECTORY)
    endif()
ENDMACRO()

