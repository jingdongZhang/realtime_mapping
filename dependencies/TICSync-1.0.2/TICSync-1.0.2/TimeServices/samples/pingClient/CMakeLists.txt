#project(pingClient)

include_directories(${TimeServices_INCLUDE_DIRS} ${GetPot_INCLUDE_DIRS})

add_executable(pingClient src/pingClientMain.cpp)
target_link_libraries(pingClient ${TimeServices_LIBRARIES})

