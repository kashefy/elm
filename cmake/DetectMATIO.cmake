# ----------------------------------------------------------------------------
#  MAT File I/O Library "MATIO"
# ----------------------------------------------------------------------------
# Defines: MATIO_FOUND, MATIO_INCLUDE_DIRS, MATIO_LIBS, MATIO_LIBRARY_DIR
#
# To link against non-system HDF5 installation define HDFS5_DIR
# This defines HDFS5_LIBS if HDFS5_DIR is defined
#
# Appends to ${ROOT_PROJECT}_INCLUDE_DIRS, ${ROOT_PROJECT}_LIBS
# ----------------------------------------------------------------------------
# Force the user to tell us which MATIO they want

set(MATIO_FOUND FALSE)

if(DEFINED MATIO_DIR)

    if(IS_DIRECTORY ${MATIO_DIR})

        if(IS_DIRECTORY ${MATIO_DIR}/include AND IS_DIRECTORY ${MATIO_DIR}/lib)

            file(TO_CMAKE_PATH ${MATIO_DIR}/include MATIO_INCLUDE_DIRS)
            list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${MATIO_INCLUDE_DIRS})

            set(MATIO_LIBS matio)
            list(APPEND ${ROOT_PROJECT}_LIBS ${MATIO_LIBS})

            file(TO_CMAKE_PATH ${MATIO_DIR}/lib MATIO_LIBRARY_DIR)
            LINK_DIRECTORIES(${MATIO_LIBRARY_DIR})

            # To link against non-system HDF5 installation
            if(DEFINED HDF5_DIR)
                LINK_DIRECTORIES(${HDF5_DIR}/lib)
                set(HDF5_LIBS hdf5)
                list(APPEND ${ROOT_PROJECT}_LIBS ${HDF5_LIBS})
            endif(DEFINED HDF5_DIR)

            add_definitions(-D__WITH_MATIO)

            set(MATIO_FOUND TRUE)

        else()
            message(FATAL_ERROR "\"MATIO_DIR\" missing include/ and/or lib/ subdirectories.")
        endif()
    else()
        message(FATAL_ERROR "\"MATIO_DIR\" set to an invalid path. Please provide path to installation/build directory or pkg-config (.pc) file")
    endif()

else(DEFINED MATIO_DIR)
    set(MATIO_DIR "" CACHE PATH "Root directory for MATIO build directory." )
    message(FATAL_ERROR "\"MATIO_DIR\" not set. Please provide the path to the root build or installation directory of MATIO.")
endif(DEFINED MATIO_DIR)
