#project(BasicTimeClient)

include_directories(${TimeServices_INCLUDE_DIRS})

add_executable(BasicTimeClient src/BasicTimeClientMain.cpp)
target_link_libraries(BasicTimeClient ${TimeServices_LIBRARIES})

