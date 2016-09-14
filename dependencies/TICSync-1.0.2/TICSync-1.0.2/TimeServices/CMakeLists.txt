#project(TimeServices)

set(SOURCES
  src/BasicTimeClient.cpp
  src/TimeServerResponseMsg.cpp
  src/TimestampSerialization.cpp
  src/SynchronizationClient.cpp
  src/TwoWayTimeSim.cpp
)

set(PUBLIC_HEADERS
  include/TICSync/TimeServerResponseMsg.h
  include/TICSync/BasicTimeClient.h
  include/TICSync/timeserviceExceptions.h
  include/TICSync/TwoWayTimeSim.h
  include/TICSync/TimestampSerialization.h
)

#################################################

find_package(ZMQ REQUIRED)
find_package(Poco REQUIRED Net Util)

set(DEPEND_INCLUDE_DIRS ${ZMQ_INCLUDE_DIRS} ${Poco_INCLUDE_DIRS})
set(DEPEND_LIBRARIES ${ZMQ_LIBRARIES} ${Poco_LIBRARIES})

# Get location of includes and target for the TimeServices library, so that
# other projects can use them.
set(TimeServices_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  #${CMAKE_CURRENT_BINARY_DIR}/include
  CACHE INTERNAL
  "Where to find TimeServices include directories"
)
list(REMOVE_DUPLICATES TimeServices_INCLUDE_DIRS)

set(TimeServices_LIBRARIES
    TICSyncTimeServices
    CACHE INTERNAL
    "List of libraries needed to link TimeServices library"
)
list(REMOVE_DUPLICATES TimeServices_LIBRARIES)

#################################################

include_directories(${TimeServices_INCLUDE_DIRS} ${DEPEND_INCLUDE_DIRS} ${TICSync_Core_INCLUDE_DIRS})
add_library(TICSyncTimeServices ${SOURCES} ${PUBLIC_HEADERS})
target_link_libraries(TICSyncTimeServices ${DEPEND_LIBRARIES} ${TICSync_Core_LIBRARIES})


if (BUILD_Tests)
    add_subdirectory(testing)
endif()    

if (BUILD_Samples)
    add_subdirectory(samples)
endif()    


#########################
# CONFIG SCRIPT CREATION
#########################
set(TICSync_TARGETS ${TICSync_TARGETS} TICSyncTimeServices
  CACHE INTERNAL
  "Targets which should be exported"
)


###############
# INSTALLATION
###############

# Provide install targets
install (TARGETS TICSyncTimeServices
         EXPORT TICSyncExport
         RUNTIME DESTINATION bin/ #${TICSync_NAMESPACE}
         LIBRARY DESTINATION lib/ #${TICSync_NAMESPACE}
         ARCHIVE DESTINATION lib/ #${TICSync_NAMESPACE}
)


# Install the necessary public header files
install(DIRECTORY include
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        FILES_MATCHING PATTERN "*.h"
)
