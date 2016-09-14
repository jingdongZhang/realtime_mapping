#project(Int64TwoWaySync)

find_package(Poco REQUIRED Net Util)

include_directories(${TICSync_INCLUDE_DIRS})
include_directories(${Poco_INCLUDE_DIRS})

add_executable(Int64TwoWaySync Int64TwoWaySync.cpp)
target_link_libraries(Int64TwoWaySync ${TICSync_LIBRARIES} ${Poco_LIBRARIES})

