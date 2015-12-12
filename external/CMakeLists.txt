add_subdirectory(esp8266-cmake/sdk)
add_subdirectory(esp8266-cmake/main)

set(MQTT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/esp_mqtt)

add_library(MQTT_LIB INTERFACE)
set_property(TARGET MQTT_LIB
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    ${MQTT_SOURCE_DIR}/include
    ${MQTT_SOURCE_DIR}/mqtt/include
    ${MQTT_SOURCE_DIR}/modules/include
)

file(GLOB MQTT_SRC
    ${MQTT_SOURCE_DIR}/mqtt/*.c
    ${MQTT_SOURCE_DIR}/modules/*.c
    ${MQTT_SOURCE_DIR}/driver/*.c
)

add_library(mqtt STATIC
    ${MQTT_SRC}
)

target_include_directories(mqtt PRIVATE
    $<TARGET_PROPERTY:USER_LIB,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:MQTT_LIB,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:ESP8266_SDK,INTERFACE_INCLUDE_DIRECTORIES>
)

