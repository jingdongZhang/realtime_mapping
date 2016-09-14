#project(Core)

set(SOURCES
  # There are no cpp files.  This library is header-only
)

set(PUBLIC_HEADERS
  include/TICSync/TICSync.h
 
  # Utils 
  include/TICSync/dataTypes.h
  include/TICSync/geometry.h
  include/TICSync/exceptions.h
 
  # Low level classes
  include/TICSync/ConvexEnvelope.h
  include/TICSync/TimingEnvelope.h
  include/TICSync/MaxSepFilter.h
  include/TICSync/TimestampMapper.h

  # User classes 
  include/TICSync/LinearOneWayClockSync.h
  include/TICSync/LinearTwoWayClockSync.h
  include/TICSync/SwitchingOneWayClockSync.h
  include/TICSync/SwitchingTwoWayClockSync.h 
)

if (BUILD_DEVEL)

  set(DEVEL_SOURCES
  )
  list(APPEND SOURCES ${DEVEL_SOURCES})

  set(DEVEL_HEADERS
  )
  list(APPEND PUBLIC_HEADERS ${DEVEL_HEADERS})

endif()

#######################
# Create config header
#######################

# Current version number of the TICSync library goes in a dynamically created header
set(TICSync_VERSION_MAJOR 0)
set(TICSync_VERSION_MINOR 5)

configure_file (
  "TICSyncConfig.h.in"
  "${PROJECT_BINARY_DIR}/include/${TICSync_NAMESPACE}/TICSyncConfig.h"
)


######################################

# Set location of includes and target for the TICSync Core library, so that
# other projects can use them.
# The binary dir is also added, to allow access to the config header
# created earlier.
set(TICSync_Core_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  CACHE INTERNAL
  "Where to find libTICSync's include directories"
)
list(REMOVE_DUPLICATES TICSync_Core_INCLUDE_DIRS)

set(TICSync_Core_LIBRARIES
    #TICSync
    CACHE INTERNAL
    "List of libraries needed to link libTICSync"
)
list(REMOVE_DUPLICATES TICSync_Core_LIBRARIES)



#######################
# Build the library
#######################



include_directories(${TICSync_Core_INCLUDE_DIRS})

# There is no library to build - it's header-only.
#add_library(TICSync ${SOURCES} ${PUBLIC_HEADERS})


# Build the supplementary programs
if (BUILD_Tests)
    add_subdirectory(testing)
endif()    

if (BUILD_Samples)
    add_subdirectory(samples)
endif()    




###############
# INSTALLATION
###############

# No binary to install, so this is commented out
# install (TARGETS TICSync
#          EXPORT TICSyncExport
#          RUNTIME DESTINATION bin/${TICSync_NAMESPACE}
#          LIBRARY DESTINATION lib/${TICSync_NAMESPACE}
#          ARCHIVE DESTINATION lib/${TICSync_NAMESPACE}
# )


# Install the necessary public header files
install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${PROJECT_BINARY_DIR}/include
	DESTINATION ${CMAKE_INSTALL_PREFIX}
	FILES_MATCHING PATTERN "*.h"
)
