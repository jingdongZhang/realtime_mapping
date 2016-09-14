CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(TICSync)

# The subdirectory below bin/ or lib/ that files should be installed to
set(TICSync_NAMESPACE TICSync)

# We want to create dynamic libraries
set(BUILD_SHARED_LIBS true)

# Make sure stuff goes in to the correct places in the build dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Point CMake to the directory containing various build scripts
set(CMakeScripts_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMakeScripts_MODULE_DIR})

# Turn on all compiler warnings
include(EnableAllWarnings)

# Enable Doxygen build with 'make doxygen'
include(UseDoxygen)

# Allow use of dependent options, that disappaear from the CMake
# GUI if they're not relevant
include(CMakeDependentOption)

# Add option to allow publishing of build results to CDash server
option(ENABLE_CDASH_TARGETS "Turn on testing targets that upload results to CDash" OFF)
if( ENABLE_CDASH_TARGETS )
  enable_testing()
  include( CTest )
endif()

# Build some required thirdparty libraries that are distributed with TICSync
add_subdirectory(thirdparty)

# Empty the export targets list
set(TICSync_TARGETS
  CACHE INTERNAL
  "Targets which should be exported"
)

option(BUILD_Samples "Build sample programs which show how to use TICSync" OFF)
option(BUILD_Tests "Build test programs used during development.  Not usually necessary." OFF)
mark_as_advanced(BUILD_Tests)

add_subdirectory(Core)
list(APPEND TICSync_INCLUDE_DIRS ${TICSync_Core_INCLUDE_DIRS})
list(APPEND TICSync_LIBRARIES    ${TICSync_Core_LIBRARIES})

option(BUILD_TimeServices
    "Build TimeServer and TimeClient library. Depends on Poco and ZMQ libraries."
    OFF
)
if (BUILD_TimeServices)
  add_subdirectory(TimeServices)
  list(APPEND TICSync_INCLUDE_DIRS ${TimeServices_INCLUDE_DIRS})
  list(APPEND TICSync_LIBRARIES    ${TimeServices_LIBRARIES})
endif ()

cmake_dependent_option(BUILD_Global_Samples "Build sample programs that may depend on all TICSync libs, as well as Poco and ZMQ." OFF
    "BUILD_Samples; BUILD_TimeServices" OFF)

if (BUILD_Global_Samples)
    add_subdirectory(samples)
endif ()


#########################
# CONFIG SCRIPT CREATION
#########################

# Provide a cmake config script for the build directory
if (TICSync_TARGETS)
  list(REMOVE_DUPLICATES TICSync_TARGETS)
  export(TARGETS ${TICSync_TARGETS} FILE UseTICSync.cmake)
endif ()

configure_file( "config/TICSyncConfig.cmake.in" "${PROJECT_BINARY_DIR}/TICSyncConfig.cmake" @ONLY )

# Stick build location in the central CMake user package registry, so that it may be easily found
if ("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.7)
  export(PACKAGE TICSync)
endif()


###############
# INSTALLATION
###############

# Note: The TICSync library target handles its own installation

# Create configuration files that external projects may use to include TICSync
# This will pick up stuff from sub-directories that export targets with the
# 'TICSyncExport' tag
if (TICSync_TARGETS)
  install(EXPORT TICSyncExport
          FILE UseTICSync.cmake
          DESTINATION lib/${TICSync_NAMESPACE}
  )
endif ()

# We're going to reuse the TICSyncConfig.cmake.in file from before, so we'll
# need to override the includes path to point at the right place in the install
# location
# TODO : This needs fixing to include paths to dependencies too.
set(TICSync_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
configure_file( "config/TICSyncConfig.cmake.in" "${PROJECT_BINARY_DIR}/TICSyncConfig.cmake.install" @ONLY )

install(FILES "${PROJECT_BINARY_DIR}/TICSyncConfig.cmake.install"
        DESTINATION lib/${TICSync_NAMESPACE}
        RENAME TICSyncConfig.cmake
)


# Provide an 'uninstall' target so that we can use 'make uninstall'
include(UninstallTarget)
configure_uninstall_target("${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
