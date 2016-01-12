file(GLOB USER_LIB_SRC
    ${CMAKE_CURRENT_LIST_DIR}/*.c
)

add_library(user_code STATIC
    ${USER_LIB_SRC}
)

target_include_directories(user_code PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(user_code
    ESP8266_SDK
    gpio16
    mqtt
)

