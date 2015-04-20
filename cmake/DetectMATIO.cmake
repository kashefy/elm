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

set(MATIO_FOUND TRUE)

status("MATIO_DIR:${MATIO_DIR}")

find_package(PkgConfig)
pkg_check_modules(PC_MATIO QUIET matio)
set(MATIO_DEFINITIONS ${PC_MATIO_CFLAGS_OTHER})

file(TO_CMAKE_PATH ${MATIO_DIR}/lib MATIO_TMP_LIB_DIR)

find_library(MATIO_LIBS
             NAMES libmatio.a matio
             HINTS ${PC_MATIO_LIBDIR} ${PC_MATIO_LIBRARY_DIRS}
             PATHS ${MATIO_DIR} ${MATIO_TMP_LIB_DIR}
             DOC "find MATIO library")

if(${MATIO_LIBS} STREQUAL MATIO_LIBS-NOTFOUND)
    message(SEND_ERROR "Failed to find MATIO library.")
    set(MATIO_FOUND FALSE)
endif()

file(TO_CMAKE_PATH ${MATIO_DIR}/include MATIO_TMP_INCLUDE_DIR)

find_path(MATIO_INCLUDE_DIR matio.h
          HINTS ${MATIO_TMP_INCLUDE_DIR} ${PC_MATIO_INCLUDEDIR} ${PC_MATIO_INCLUDE_DIRS}
          PATH_SUFFIXES matio)

if(${MATIO_INCLUDE_DIR} STREQUAL MATIO_INCLUDE_DIR-NOTFOUND)
    message(SEND_ERROR "Failed to find MATIO headers.")
    set(MATIO_FOUND FALSE)
endif()

if(MATIO_FOUND)
    set(MATIO_INCLUDE_DIRS ${MATIO_INCLUDE_DIR})
    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${MATIO_INCLUDE_DIRS})
    list(APPEND ${ROOT_PROJECT}_LIBS ${MATIO_LIBS})

    #LINK_DIRECTORIES(${MATIO_LIBRARY_DIR})

    # To link against non-system HDF5 installation
    status("HDF5_DIR=${HDF5_DIR}")
    if(DEFINED HDF5_DIR)

        file(TO_CMAKE_PATH ${HDF5_DIR}/lib HDF5_TMP_LIB_DIR)

        find_library(HDF5_LIBS
                     NAMES libhdf5.a hdf5
                     HINTS ${HDF5_DIR} ${HDF5_TMP_LIB_DIR}
                     PATHS ${HDF5_DIR} ${HDF5_TMP_LIB_DIR}
                     DOC "find HDF5 library"
                     NO_SYSTEM_ENVIRONMENT_PATH)

        if(NOT ${HDF5_LIBS} STREQUAL HDF5_LIBS-NOTFOUND)

            list(APPEND ${ROOT_PROJECT}_LIBS ${HDF5_LIBS})

            if(UNIX)
                # HDF5 on Unix systems references to dlclose()
                list(APPEND ${ROOT_PROJECT}_LIBS dl)
            endif(UNIX)

        endif(NOT ${HDF5_LIBS} STREQUAL HDF5_LIBS-NOTFOUND)

#LINK_DIRECTORIES(${HDF5_DIR}/lib)
#        set(HDF5_LIBS hdf5)
    endif(DEFINED HDF5_DIR)

    add_definitions(-D__WITH_MATIO)
else(MATIO_FOUND)
    message(FATAL_ERROR "Failed to find MATIO. Verify that MATIO_DIR points to a valid pkf-config file, build or installation directory.")
endif(MATIO_FOUND)



