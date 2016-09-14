#project(TimeServer)

include_directories(${TimeServices_INCLUDE_DIRS} ${GetPot_INCLUDE_DIRS})

add_executable(TICSyncTimeServer src/TimeServerMain.cpp)
target_link_libraries(TICSyncTimeServer ${TimeServices_LIBRARIES})

# Provide install targets
install (TARGETS TICSyncTimeServer
         EXPORT TICSyncExport
         RUNTIME DESTINATION bin/
)