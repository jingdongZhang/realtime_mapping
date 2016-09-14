# A CMake script to find the Poco library
# and any required components
#
# Usage:
# find_package(Poco COMPONENTS XML Net Data Util ...)
# ...
# include_directories(${Poco_INCLUDE_DIRS})
# ...
# target_link_libraries(<targetname> ... ${Poco_LIBRARIES})
#
# Notes:
# If you wish the build to fail if Poco and the requested
# components are NOT found, then use
# find_package(Poco REQUIRED XML Net Data Util ...)
#
# Cache Variables:
# Poco_LIBRARY
# Poco_INCLUDE_DIR
#
# Non-cache variables you might use in your CMakeLists.txt:
# Poco_FOUND
# Poco_LIBRARIES
# Poco_INCLUDE_DIRS
#
# Poco_ROOT_DIR is searched preferentially for these files
#
# Requires these CMake modules:
# FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# This version:
# 2011 Alastair Harrison
#
# Derived from FindVRPN.cmake
# Original Author:
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu> <abiryan@ryand.net>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC
#
# Copyright Iowa State University 2009-2010.
# Distributed under the Boost Software License, Version 1.0.
# http://www.boost.org/LICENSE_1_0.txt

set(Poco_ROOT_DIR
    "${Poco_ROOT_DIR}"
    CACHE
    PATH
    "Root directory to search for Poco"
)

if(CMAKE_SIZEOF_VOID_P MATCHES "8")
    set(_libsuffixes lib64 lib)

    # 64-bit dir: only set on win64
    file(TO_CMAKE_PATH "$ENV{ProgramW6432}" _progfiles)
else()
    set(_libsuffixes lib)
    if(NOT "$ENV{ProgramFiles(x86)}" STREQUAL "")
        # 32-bit dir: only set on win64
        file(TO_CMAKE_PATH "$ENV{ProgramFiles(x86)}" _progfiles)
    else()
        # 32-bit dir on win32, useless to us on win64
        file(TO_CMAKE_PATH "$ENV{ProgramFiles}" _progfiles)
    endif()
endif()

# Decide whether to look for debug libraries or not
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(DBG "d")
else()
    set(DBG "")
endif()


###
# Configure Poco
###

find_path(Poco_INCLUDE_DIR
    NAMES
    Poco/Poco.h
    PATH_SUFFIXES
    include
    include/Poco
    HINTS
    "${Poco_ROOT_DIR}"
    PATHS
    "${_progfiles}/Poco"
)

find_library(Poco_LIBRARY
    NAMES
    PocoFoundation${DBG}
    PATH_SUFFIXES
    ${_libsuffixes}
    HINTS
    "${Poco_ROOT_DIR}"
    "${Poco_ROOT_DIR}/Poco"
    PATHS
    "${_progfiles}/Poco"
)

# Look for each of the requested components in turn
list(REMOVE_DUPLICATES Poco_FIND_COMPONENTS)
foreach(COMPONENT ${Poco_FIND_COMPONENTS})

    if (COMPONENT STREQUAL "Foundation")
        message(WARNING
          "You shouldn't explicitly add the 'Foundation' component in your find_package(Poco ...) call.")
    endif()
    
    list(APPEND Poco_REQUESTED_COMPONENTS Poco_${COMPONENT}_LIBRARY)

    find_library(Poco_${COMPONENT}_LIBRARY
        NAMES
        Poco${COMPONENT}${DBG}
        PATH_SUFFIXES
        ${_libsuffixes}
        HINTS
        "${Poco_ROOT_DIR}"
        PATHS
        "${_progfiles}/Poco"
    )
    
    if (Poco_${COMPONENT}_LIBRARY)
        list(APPEND Poco_COMPONENT_LIBRARIES ${Poco_${COMPONENT}_LIBRARY})
        mark_as_advanced(Poco_${COMPONENT}_LIBRARY)
    endif()

endforeach()


# handle the QUIETLY and REQUIRED arguments and set xxx_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Poco
    DEFAULT_MSG
    Poco_LIBRARY
    Poco_INCLUDE_DIR
    "${Poco_REQUESTED_COMPONENTS}"
)

set(Poco_FOUND ${POCO_FOUND})
if(Poco_FOUND)
    set(Poco_INCLUDE_DIRS "${Poco_INCLUDE_DIR}")
    set(Poco_LIBRARIES "${Poco_LIBRARY}" "${Poco_COMPONENT_LIBRARIES}")

    mark_as_advanced(Poco_ROOT_DIR)
endif()

mark_as_advanced(Poco_LIBRARY
    Poco_INCLUDE_DIR
)