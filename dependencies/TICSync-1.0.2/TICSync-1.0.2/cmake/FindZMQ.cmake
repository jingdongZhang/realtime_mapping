# - try to find ZMQ library
#
# Cache Variables:
#  ZMQ_LIBRARY
#  ZMQ_INCLUDE_DIR
#
# Non-cache variables you might use in your CMakeLists.txt:
#  ZMQ_FOUND
#  ZMQ_SERVER_LIBRARIES
#  ZMQ_LIBRARIES
#  ZMQ_INCLUDE_DIRS
#
# ZMQ_ROOT_DIR is searched preferentially for these files
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#

set(ZMQ_ROOT_DIR
  "${ZMQ_ROOT_DIR}"
  CACHE
  PATH
  "Root directory to search for ZMQ")

if("${CMAKE_SIZEOF_VOID_P}" MATCHES "8")
  set(_libsuffixes lib64 lib)
else()
  set(_libsuffixes lib)
endif()

###
# Configure ZMQ
###

find_path(ZMQ_INCLUDE_DIR
  NAMES
  zmq.h
  zmq.hpp
  PATH_SUFFIXES
  include
  HINTS
  "${ZMQ_ROOT_DIR}")

find_library(ZMQ_LIBRARY
  NAMES
  zmq
  PATH_SUFFIXES
  ${_libsuffixes}
  HINTS
  "${ZMQ_ROOT_DIR}")

# handle the QUIETLY and REQUIRED arguments and set xxx_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZMQ
  DEFAULT_MSG
  ZMQ_LIBRARY
  ZMQ_INCLUDE_DIR
)

if(ZMQ_FOUND)
  set(ZMQ_INCLUDE_DIRS "${ZMQ_INCLUDE_DIR}")
  set(ZMQ_LIBRARIES "${ZMQ_LIBRARY}")

  mark_as_advanced(ZMQ_ROOT_DIR)
endif()

mark_as_advanced(ZMQ_LIBRARY
  ZMQ_SERVER_LIBRARY
  ZMQ_INCLUDE_DIR)
