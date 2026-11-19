if(CONFIG_SH8601)
    zephyr_library_named(display_sh8601)
    zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/display_sh8601.c)
endif()
