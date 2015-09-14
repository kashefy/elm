# ----------------------------------------------------------------------------
# OpenCV
# ----------------------------------------------------------------------------
# find_package OpenCV
# Defines: OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS, OpenCV_LINK_LIBRARIES
# ----------------------------------------------------------------------------
if(DEFINED OpenCV_DIR)

    find_package(OpenCV REQUIRED core highgui imgproc ml PATHS ${OpenCV_DIR})

else(DEFINED OpenCV_DIR)

    find_package(OpenCV REQUIRED core highgui imgproc ml)

endif(DEFINED OpenCV_DIR)

if(OpenCV_FOUND)

    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
    list(APPEND ${ROOT_PROJECT}_LIBS ${OpenCV_LIBS})

else(OpenCV_FOUND)

    message(SEND_ERROR "Failed to find OpenCV. Double check that \"OpenCV_DIR\" points to the root of an OpenCV build directory.")

endif(OpenCV_FOUND)
