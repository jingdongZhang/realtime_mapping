include_directories(${TimeServices_INCLUDE_DIRS} ${GetPot_INCLUDE_DIRS})

add_executable(epochClockMapperTest epochClockMapperTest.cpp)
target_link_libraries(epochClockMapperTest ${TimeServices_LIBRARIES})

