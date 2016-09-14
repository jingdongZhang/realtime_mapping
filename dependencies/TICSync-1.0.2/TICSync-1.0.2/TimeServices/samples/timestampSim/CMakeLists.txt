#project(timestampSim)

include_directories(${TimeServices_INCLUDE_DIRS} ${GetPot_INCLUDE_DIRS})

add_executable(timestampSim src/timestampSimMain.cpp)
target_link_libraries(timestampSim ${TimeServices_LIBRARIES})

