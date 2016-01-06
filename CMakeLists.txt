project("ESP8266-mqtt" C)
cmake_minimum_required(VERSION 3.0)

add_subdirectory(external)

add_subdirectory(user)

set_property(TARGET USER_CODE
    PROPERTY INTERFACE_LINK_LIBRARIES
    user_lib
    gpio16
    mqtt
)
