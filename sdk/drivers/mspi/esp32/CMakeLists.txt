if(CONFIG_MSPI_ESP32)
    zephyr_library_named(mspi_esp32)
    zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/mspi_esp32.c)

    # hal_espressif only builds the GPSPI HAL for the SPI driver
    if(NOT CONFIG_ESP32_SPIM)
        set(gpspi ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_gpspi)
        zephyr_library_sources(
            ${gpspi}/spi_hal.c
            ${gpspi}/spi_hal_iram.c
            ${gpspi}/${CONFIG_SOC_SERIES}/spi_periph.c
        )
    endif()
endif()
