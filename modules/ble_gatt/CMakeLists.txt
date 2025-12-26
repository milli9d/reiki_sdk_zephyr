zephyr_library_named(ble_gatt)

set_target_properties(ble_gatt PROPERTIES LINKER_LANGUAGE CXX)

zephyr_compile_definitions(

    # PICO_BUILD
    __LINUX__
)

zephyr_compile_options(
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-unused-parameter
    -Wno-maybe-uninitialized
)

file(GLOB srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/*.c*
)

zephyr_library_sources(${srcs})

zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/include)