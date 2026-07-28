# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BOARD_ESP32C6_1_47_TFT)
  file(GLOB srcs 
    init/*.c
  )
  zephyr_library()
  zephyr_library_sources(${srcs})
  zephyr_library_include_directories(init/include)
  zephyr_library_link_libraries_ifdef(CONFIG_FAT_FILESYSTEM_ELM   
    FS 
    INTERFACE ELMFAT
  ) 
endif()
