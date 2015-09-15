# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
if(DEFINED OpenCV_DIR)
    get_filename_component(OpenCV_DIR_EXT ${OpenCV_DIR} EXT)
    message(STATUS "OpenCV_DIR_EXT${OpenCV_DIR_EXT}...")
    if(OpenCV_DIR_EXT STREQUAL ".pc")

        if(NOT EXISTS ${OpenCV_DIR})
            message(SEND_ERROR "${OpenCV_DIR}. File does not exist.")
        endif(NOT EXISTS ${OpenCV_DIR})

        # Get parent directory of .pc file
        # GET_PARENT_DIR is a custom macro (may need include(cmake/FileSystemUtils.cmake))
        GET_PARENT_DIR(OpenCV_DIR_PKG_CONFIG_PATH ${OpenCV_DIR})

        message(STATUS "Prepending PKG_CONFIG_PATH with ${OpenCV_DIR_PKG_CONFIG_PATH}")
        set(ENV{PKG_CONFIG_PATH} "${OpenCV_DIR_PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}" )
        message(STATUS "PKG_CONFIG_PATH=$ENV{PKG_CONFIG_PATH}")

        find_package(PkgConfig)
        pkg_check_modules(OpenCV opencv)

        message(STATUS "OpenCV_PREFIX=${OpenCV_PREFIX}")
        message(STATUS "OpenCV_CFLAGS=${OpenCV_CFLAGS}")
        message(STATUS "OpenCV_CFLAGS_OTHER=${OpenCV_CFLAGS_OTHER}")

    else(OpenCV_DIR_EXT STREQUAL ".pc")

        message(STATUS "OpenCV_DIR not .pc")
        find_package(OpenCV REQUIRED core highgui imgproc ml PATHS ${OpenCV_DIR})
    endif(OpenCV_DIR_EXT STREQUAL ".pc")

else(DEFINED OpenCV_DIR)
    message(STATUS "OpenCV_DIR not defined")
    find_package(OpenCV REQUIRED core highgui imgproc ml)
endif(DEFINED OpenCV_DIR)

if(OpenCV_FOUND)

    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

    list(LENGTH ${OpenCV_LIBS} OpenCV_NUM_LIBS)
    message(STATUS "OpenCV_NUM_LIBS=${OpenCV_NUM_LIBS}")
    message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
    message(STATUS "OpenCV_Libs=${OpenCV_Libs}")
    list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS})

    # Define OpenCV_LIBRARY_DIR if missing
    if(NOT DEFINED OpenCV_LIBRARY_DIR)
        # Append lib to prefix
        file(TO_CMAKE_PATH ${OpenCV_PREFIX}/lib OpenCV_LIBRARY_DIR)
    endif(NOT DEFINED OpenCV_LIBRARY_DIR)

else(OpenCV_FOUND)
    message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" points to the root of an OpenCV build directory.")
endif(OpenCV_FOUND)
