zephyr_library_named(imu_qmi8658c)

zephyr_library_include_directories_ifdef(CONFIG_IMU_QMI8658C
    ${CMAKE_CURRENT_LIST_DIR}/
)

zephyr_library_sources_ifdef(CONFIG_IMU_QMI8658C
    ${CMAKE_CURRENT_LIST_DIR}/qmi8658c.c
)