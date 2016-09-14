#project(tictest)

include_directories(${TICSync_Core_INCLUDE_DIRS})

add_executable(ConvexEnvelopeTest ConvexEnvelopeTest.cpp)
target_link_libraries(ConvexEnvelopeTest ${TICSync_Core_LIBRARIES})

add_executable(MaxSepFilterTest MaxSepFilterTest.cpp)
target_link_libraries(MaxSepFilterTest ${TICSync_Core_LIBRARIES})

add_executable(LinearOneWayClockSyncTest LinearOneWayClockSyncTest.cpp)
target_link_libraries(LinearOneWayClockSyncTest ${TICSync_Core_LIBRARIES})

add_executable(LinearTwoWayClockSyncTest LinearTwoWayClockSyncTest.cpp)
target_link_libraries(LinearTwoWayClockSyncTest ${TICSync_Core_LIBRARIES})

if (BUILD_DEVEL)
endif ()
