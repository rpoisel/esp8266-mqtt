set(ESP8266_SDK_VARIANT "NONOS")
set(ESP8266_SDK_VERSION "1.4.0")

add_subdirectory(esp8266-cmake/main)
add_subdirectory(esp8266-cmake/sdk)

### GPIO16

set(GPIO16_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/esp8266-gpio16)

file(GLOB GPIO16_SRC
    ${GPIO16_SOURCE_DIR}/driver/gpio16.c
)

add_library(gpio16 STATIC
    ${GPIO16_SRC}
)

target_include_directories(gpio16 PUBLIC
    ${GPIO16_SOURCE_DIR}/include
)

target_link_libraries(gpio16 PUBLIC
    user_code
    ESP8266_SDK
)

### esp_mqtt
set(MQTT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/esp_mqtt)

file(GLOB MQTT_SRC
    ${MQTT_SOURCE_DIR}/mqtt/*.c
    ${MQTT_SOURCE_DIR}/modules/*.c
    ${MQTT_SOURCE_DIR}/driver/*.c
)

add_library(mqtt STATIC
    ${MQTT_SRC}
)

target_include_directories(mqtt PUBLIC
    ${MQTT_SOURCE_DIR}/include
    ${MQTT_SOURCE_DIR}/mqtt/include
    ${MQTT_SOURCE_DIR}/modules/include
)

target_link_libraries(mqtt PUBLIC
    user_code
    ESP8266_SDK
)
