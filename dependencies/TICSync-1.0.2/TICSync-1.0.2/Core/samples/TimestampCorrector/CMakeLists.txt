#project(TimestampCorrector)

include_directories(${TICSync_Core_INCLUDE_DIRS})

add_executable(TimestampCorrector TimestampCorrector.cpp)
target_link_libraries(TimestampCorrector ${TICSync_Core_LIBRARIES})

