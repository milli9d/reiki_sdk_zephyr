#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hpcore_init);

extern int32_t esp32c6_1_47_tft_display_init(void);
extern int32_t esp32c6_1_47_tft_diskfs_init();

int32_t esp32c6_1_47_tft_hp_core_init(void)
{
    int rc = esp32c6_1_47_tft_display_init();
    if (rc != 0) {
        printk("Failed to initialize display.\n");
    }

    rc = esp32c6_1_47_tft_diskfs_init();
    if (rc != 0) {
        printk("Failed to initialize SD Card.\n");
    }

    printk("ESP32C6 HP Core init complete.\n");
    return 0;
}

SYS_INIT_NAMED(hpcore_init, esp32c6_1_47_tft_hp_core_init, APPLICATION,
               CONFIG_BOARD_ESP32C6_1_47_TFT_INIT_PRIORITY);