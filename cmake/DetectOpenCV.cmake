# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
set(OpenCV_REQUIRED_MODULES core highgui imgproc ml)
if(DEFINED OpenCV_DIR)

    get_filename_component(OpenCV_DIR_EXT ${OpenCV_DIR} EXT)

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
        pkg_check_modules(OpenCV opencv)

        message(STATUS "OpenCV_PREFIX=${OpenCV_PREFIX}")

        message(STATUS "Prepending CFLAGS with ${OpenCV_CFLAGS}")
        set(CMAKE_C_FLAGS "${OpenCV_CFLAGS} ${CMAKE_C_FLAGS}")

        #message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
        #message(STATUS "OpenCV_Libs=${OpenCV_Libs}")
        #message(STATUS "OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")         # <XPREFIX>_LIBRARIES...only the libraries (w/o the '-l')
        #message(STATUS "OpenCV_LIBRARY_DIRS=${OpenCV_LIBRARY_DIRS}")   # <XPREFIX>_LIBRARY_DIRS...the paths of the libraries (w/o the '-L')
        #message(STATUS "OpenCV_LDFLAGS=${OpenCV_LDFLAGS}")             # <XPREFIX>_LDFLAGS...all required linker flags
        #message(STATUS "OpenCV_LDFLAGS_OTHER=${OpenCV_LDFLAGS_OTHER}") # <XPREFIX>_LDFLAGS_OTHER...all other linker flags
        #message(STATUS "OpenCV_LIBDIR=${OpenCV_LIBDIR}")               # <XPREFIX>_LIBDIR...lib-dir of the module

        #message(STATUS "Prepending linker flags with ${OpenCV_LDFLAGS}")
        #set(CMAKE_EXE_LINKER_FLAGS "${OpenCV_LDFLAGS} ${CMAKE_EXE_LINKER_FLAGS}")
        #set(CMAKE_MODULE_LINKER_FLAGS "${OpenCV_LDFLAGS} ${CMAKE_MODULE_LINKER_FLAGS}")
        foreach(LDFLAGS_ITEM ${OpenCV_LDFLAGS})
            if(LDFLAGS_ITEM MATCHES ".so$")

                message(STATUS "is .so ${LDFLAGS_ITEM}")
            endif(LDFLAGS_ITEM MATCHES ".so$")
        endforeach()

        list(APPEND OpenCV_LIBS ${OpenCV_LIBRARIES})

    else(OpenCV_DIR_EXT STREQUAL ".pc")
        find_package(OpenCV REQUIRED ${OpenCV_REQUIRED_MODULES} PATHS ${OpenCV_DIR})
    endif(OpenCV_DIR_EXT STREQUAL ".pc")

else(DEFINED OpenCV_DIR)
    message(STATUS "OpenCV_DIR not defined")
    find_package(OpenCV REQUIRED ${OpenCV_REQUIRED_MODULES})
endif(DEFINED OpenCV_DIR)

if(OpenCV_FOUND)

    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
    list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS})

    # Define OpenCV_LIBRARY_DIR if missing
    if(NOT DEFINED OpenCV_LIBRARY_DIR)

        if(DEFINED OpenCV_LIBDIR AND NOT "${OpenCV_LIBDIR}" STREQUAL "")

            set(OpenCV_LIBRARY_DIR ${OpenCV_LIBDIR})

        else(DEFINED OpenCV_LIBDIR  AND NOT "${OpenCV_LIBDIR}" STREQUAL "")

            # Append lib to prefix
            file(TO_CMAKE_PATH ${OpenCV_PREFIX}/lib OpenCV_LIBRARY_DIR)

        endif(DEFINED OpenCV_LIBDIR AND NOT "${OpenCV_LIBDIR}" STREQUAL "")

    endif(NOT DEFINED OpenCV_LIBRARY_DIR)

else(OpenCV_FOUND)
    message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" points to the root of an OpenCV build directory.")
endif(OpenCV_FOUND)
