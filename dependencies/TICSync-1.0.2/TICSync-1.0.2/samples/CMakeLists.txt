if (NOT BUILD_TimeServices)
    message(SEND_ERROR "Global samples require TimeServices library to be enabled.")
    return()
endif()

add_subdirectory(Int64TwoWaySync)
