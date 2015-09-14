# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
if(DEFINED OpenCV_DIR)
    get_filename_component(OpenCV_DIR_TYPE ${OpenCV_DIR} EXT)

    if(OpenCV_DIR_EXT STREQUAL ".pc")

        if(NOT EXISTS ${OpenCV_DIR})
            message(SEND_ERROR "${OpenCV_DIR}. File does not exist.")
        endif(NOT EXISTS ${OpenCV_DIR})

        find_package(PkgConfig)
        pkg_check_modules(OpenCV opencv)

    else(OpenCV_DIR_EXT STREQUAL ".pc")
        find_package(OpenCV REQUIRED core highgui imgproc ml PATHS ${OpenCV_DIR})
    endif(OpenCV_DIR_EXT STREQUAL ".pc")

else(DEFINED OpenCV_DIR)
    find_package(OpenCV REQUIRED core highgui imgproc ml)
endif(DEFINED OpenCV_DIR)

if(OpenCV_FOUND)

    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
    list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS})

else(OpenCV_FOUND)
    message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" points to the root of an OpenCV build directory.")
endif(OpenCV_FOUND)
