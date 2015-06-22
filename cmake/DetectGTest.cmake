# ----------------------------------------------------------------------------
# GTest package
# ----------------------------------------------------------------------------
# NOTE: This needs to happen before OpenCV as OpenCV introduced a bug:
# Their ts module results in multiple definitions at linking time
#
# DEFINES: GTEST_FOUND, GTEST_INCLUDE_DIRS, GTEST_BOTH_LIBRARIES, GTEST_LIBRARIES (libgtest), GTEST_MAIN_LIBRARIES (liggtest_main)
# INPUTS:  GTEST_ROOT, GTEST_MSVC_SEARCH
# For details, look for FindGTest.cmake located in the cmake folder: /usr/share/cmake28/Modules/FindGTest.cmake
# ----------------------------------------------------------------------------

status("GTEST_ROOT=${GTEST_ROOT}")
find_package(GTest REQUIRED)
if(GTEST_FOUND)

        if(BUILD_SHARED_LIBS)
            get_filename_component(__GTEST_LIB_EXT ${GTEST_LIBRARIES} EXT)

            if(NOT ${__GTEST_LIB_EXT} EQUAL ${CMAKE_SHARED_LIBRARY_SUFFIX})
                message(WARNING "Linking shared build to static gtest may lead to testrunner crash due to memory leaks.")
            endif()

        endif(BUILD_SHARED_LIBS)

	list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
	list(APPEND ${ROOT_PROJECT}_LIBS ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
else(GTEST_FOUND)
	if(NOT DEFINED GTEST_ROOT)
		set(GTEST_ROOT "" CACHE PATH "Root directory for GTEST in source build. Typically ../gmock-1.7.0/gtest")
	endif(NOT DEFINED GTEST_ROOT)
	message(FATAL_ERROR "Failed to find GoogleTest. Double check that \"GTEST_ROOT\" is properly set")
endif(GTEST_FOUND)
