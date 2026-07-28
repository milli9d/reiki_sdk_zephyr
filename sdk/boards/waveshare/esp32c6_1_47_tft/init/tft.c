#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_init);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define TFT_LED_GPIO     GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, tft_led_gpios)

/**
 * @brief Initialize display backlight
 */
static int32_t tft_led_gpio(void)
{
    // turn on TFT backlight
    const struct gpio_dt_spec _gpio = TFT_LED_GPIO;
    if (!gpio_is_ready_dt(&_gpio)) {
        printk("TFT LED GPIO not ready.\n");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&_gpio, GPIO_OUTPUT_ACTIVE);
    return 0;
}

/**
 * @brief Initialize display
 */
int32_t esp32c6_1_47_tft_display_init(void)
{
    int rc = tft_led_gpio();
    if (rc != 0) {
        return rc;
    }

    printk("ESP32C6 Display init complete.\n");
    return 0;
}
