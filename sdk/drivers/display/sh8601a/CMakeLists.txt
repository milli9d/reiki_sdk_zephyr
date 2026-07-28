zephyr_library_named(display_sh8601a)

zephyr_library_include_directories_ifdef(CONFIG_DISPLAY_SH8601A
    ${CMAKE_CURRENT_LIST_DIR}/
)

zephyr_library_sources_ifdef(CONFIG_DISPLAY_SH8601A
    ${CMAKE_CURRENT_LIST_DIR}/sh8601a.c
)