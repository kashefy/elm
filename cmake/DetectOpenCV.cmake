# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
# Force the user to tell us which OpenCV they want (otherwise find_package can find the wrong one, cache it and changes to OpenCV_DIR are ignored)

if(DEFINED OpenCV_DIR)
    find_package(OpenCV REQUIRED core highgui imgproc ml PATHS ${OpenCV_DIR})
    if(OpenCV_FOUND)
        list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
        list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS})
    else(OpenCV_FOUND)
        message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" to the root build directory of OpenCV.")
    endif(OpenCV_FOUND)
else(DEFINED OpenCV_DIR)
    set(OpenCV_DIR "" CACHE PATH "Root directory for OpenCV build directory." )
    message(FATAL_ERROR "\"OpenCV_DIR\" not set. Please explicitly provide the path to the root build directory of OpenCV.")
endif(DEFINED OpenCV_DIR)
