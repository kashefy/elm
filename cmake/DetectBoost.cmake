# ----------------------------------------------------------------------------
# Boost
# ----------------------------------------------------------------------------
#
# find_package Boost
# Defines: Boost_FOUND, Boost_LIBRARIES,...
# ----------------------------------------------------------------------------

set(BOOST_MIN_VERSION "1.46.0" CACHE STRING "Minimum version of boost to link against (e.g. C:/BOOST_1_56_0 is 1.56.0")

if(MSVC)
    # mitigate interference of the auto-linking
    add_definitions(-DBOOST_ALL_NO_LIB)
endif(MSVC)

# Don't forget to include 'system'
set(BOOST_COMPONENTS system)
list(APPEND ${BOOST_COMPONENTS} filesystem graph serialization thread)

if(WITH_PYTHON)
    list(APPEND ${BOOST_COMPONENTS} python)
if(WITH_PYTHON)

status("")
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS ${BOOST_COMPONENTS})
if(Boost_FOUND)
    list(APPEND ${ROOT_PROJECT}_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
    list(APPEND ${ROOT_PROJECT}_LIBS ${Boost_LIBRARIES})
else(Boost_FOUND)
    if(NOT DEFINED BOOST_ROOT)
        set(BOOST_ROOT "" CACHE PATH "Boost installation prefix.")
    endif(NOT DEFINED BOOST_ROOT)
    message(FATAL_ERROR "Failed to find Boost (or missing components). Check that \"BOOST_ROOT\" is properly set")
endif(Boost_FOUND)
