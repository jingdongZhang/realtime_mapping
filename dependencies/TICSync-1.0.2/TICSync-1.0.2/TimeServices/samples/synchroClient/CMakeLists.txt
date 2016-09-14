#project(SynchroClient)

include_directories(${TimeServices_INCLUDE_DIRS} ${GetPot_INCLUDE_DIRS})

add_executable(SynchroClient src/SynchroClientMain.cpp)
target_link_libraries(SynchroClient ${TimeServices_LIBRARIES})

