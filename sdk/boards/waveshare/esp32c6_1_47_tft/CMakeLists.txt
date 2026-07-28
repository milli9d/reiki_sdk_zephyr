# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_ESP32C6_1_47_TFT)
  zephyr_library()
  zephyr_library_sources(esp32c6_1_47_tft_board.c)
endif()
