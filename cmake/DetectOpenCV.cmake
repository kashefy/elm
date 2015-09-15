# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
set(OpenCV_REQUIRED_MODULES core highgui imgproc ml)
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
        set(ENV{PKG_CONFIG_PATH} "${OpenCV_DIR_PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH}")
        message(STATUS "PKG_CONFIG_PATH=$ENV{PKG_CONFIG_PATH}")

        find_package(PkgConfig)
        pkg_check_modules(OpenCV opencv ${OpenCV_REQUIRED_MODULES})

#   <XPREFIX>_LIBRARIES      ... only the libraries (w/o the '-l')
#   <XPREFIX>_LIBRARY_DIRS   ... the paths of the libraries (w/o the '-L')
#   <XPREFIX>_LDFLAGS        ... all required linker flags
#   <XPREFIX>_LDFLAGS_OTHER  ... all other linker flags
#   <XPREFIX>_INCLUDE_DIRS   ... the '-I' preprocessor flags (w/o the '-I')
#   <XPREFIX>_CFLAGS         ... all required cflags
#   <XPREFIX>_CFLAGS_OTHER   ... the other compiler flags
#
#   <XPREFIX> = <PREFIX>        for common case
#   <XPREFIX> = <PREFIX>_STATIC for static linking
#
# There are some special variables whose prefix depends on the count
# of given modules. When there is only one module, <PREFIX> stays
# unchanged. When there are multiple modules, the prefix will be
# changed to <PREFIX>_<MODNAME>:
#   <XPREFIX>_VERSION    ... version of the module
#   <XPREFIX>_PREFIX     ... prefix-directory of the module
#   <XPREFIX>_INCLUDEDIR ... include-dir of the module
#   <XPREFIX>_LIBDIR     ... lib-dir of the module

        message(STATUS "OpenCV_PREFIX=${OpenCV_PREFIX}")
        message(STATUS "OpenCV_CFLAGS=${OpenCV_CFLAGS}")

        message(STATUS "Prepending CFLAGS with ${OpenCV_CFLAGS}")
        set(CMAKE_C_FLAGS "${OpenCV_CFLAGS} ${CMAKE_C_FLAGS}")

    else(OpenCV_DIR_EXT STREQUAL ".pc")

        message(STATUS "OpenCV_DIR not .pc")
        find_package(OpenCV REQUIRED ${OpenCV_REQUIRED_MODULES} PATHS ${OpenCV_DIR})
    endif(OpenCV_DIR_EXT STREQUAL ".pc")

else(DEFINED OpenCV_DIR)
    message(STATUS "OpenCV_DIR not defined")
    find_package(OpenCV REQUIRED ${OpenCV_REQUIRED_MODULES})
endif(DEFINED OpenCV_DIR)

if(OpenCV_FOUND)

    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

    list(LENGTH ${OpenCV_LIBS} OpenCV_NUM_LIBS)
    message(STATUS "OpenCV_NUM_LIBS=${OpenCV_NUM_LIBS}")
    message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
    message(STATUS "OpenCV_Libs=${OpenCV_Libs}")
    list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS} ${OpenCV_LIBRARIES})

    # Define OpenCV_LIBRARY_DIR if missing
    if(NOT DEFINED OpenCV_LIBRARY_DIR)

        if(DEFINED OpenCV_LIBDIR)
            set(OpenCV_LIBRARY_DIR ${OpenCV_LIBDIR})
        else(DEFINED OpenCV_LIBDIR)
            # Append lib to prefix
            file(TO_CMAKE_PATH ${OpenCV_PREFIX}/lib OpenCV_LIBRARY_DIR)
        endif(DEFINED OpenCV_LIBDIR)

    endif(NOT DEFINED OpenCV_LIBRARY_DIR)

else(OpenCV_FOUND)
    message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" points to the root of an OpenCV build directory.")
endif(OpenCV_FOUND)
