zephyr_library_named(gpio_tca9544apwr)

zephyr_library_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/
)

zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/tca9544apwr.c
)