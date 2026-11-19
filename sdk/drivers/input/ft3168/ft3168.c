#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ft3168, LOG_LEVEL_DBG);

#include "ft3168.h"

/* ========================================================================= */
/* Static API Declarations */
/* ========================================================================= */

static void ft3168_isr_cb(const struct device* port, struct gpio_callback* cb, unsigned int pins);
static int  ft3168_init(const struct device* dev);
static int  ft3168_reset(const struct device* dev);
static void ft3168_work_handler(struct k_work* work);

/* ========================================================================= */
/* Zephyr Driver pre-Boilerplate */
/* ========================================================================= */

#define DT_DRV_COMPAT focaltech_ft3168

#define FT3168_UNPACK(dev)                      \
    const ft3168_config_t* _cfg  = dev->config; \
    ft3168_data_t*         _data = dev->data;

/* ========================================================================= */
/* Wrapper Layer */
/* ========================================================================= */

#ifdef __ZEPHYR__

int32_t ft3168_reg_write(void* i2c_ctx, uint8_t reg_addr, uint8_t val)
{ return i2c_reg_write_byte_dt((struct i2c_dt_spec*)i2c_ctx, reg_addr, val); }

int32_t ft3168_reg_read(void* i2c_ctx, uint8_t reg_addr, uint8_t* val, size_t n_bytes)
{ return i2c_burst_read_dt((struct i2c_dt_spec*)i2c_ctx, reg_addr, val, n_bytes); }

int32_t ft3168_reg_update(void* i2c_ctx, uint8_t reg_addr, uint8_t val, uint8_t mask)
{ return i2c_reg_update_byte_dt((struct i2c_dt_spec*)i2c_ctx, reg_addr, mask, val); }

int32_t ft3168_i2c_init(void* i2c_ctx)
{
    struct i2c_dt_spec* i2c = (struct i2c_dt_spec*)i2c_ctx;

    // validate bus is present; no ID check on this device
    if (!i2c_is_ready_dt(i2c)) {
        LOG_ERR("I2C bus is not ready.");
        return -ENODEV;
    }

    return 0;
}

#ifdef CONFIG_INPUT_FT3168_INTERRUPT

int32_t ft3168_msgq_init(const void* dev_ctx)
{
    const struct device* dev = (const struct device*)dev_ctx;
    FT3168_UNPACK(dev);

    msgq_if_t* mif = &_data->q_if;

    return 0;
}

int32_t ft3168_isr_init(const void* dev_ctx)
{
    const struct device* dev = (const struct device*)dev_ctx;
    FT3168_UNPACK(dev);

    const struct gpio_dt_spec* gpio = &_cfg->int_gpio;

    if (!gpio_is_ready_dt(gpio)) {
        LOG_ERR("INT GPIO port %s is not ready", gpio->port->name);
        return -ENODEV;
    }

    int rc = gpio_pin_configure_dt(gpio, GPIO_INPUT);
    if (rc != 0) {
        LOG_ERR("Error %d: failed to configure %s pin %d", rc, gpio->port->name, gpio->pin);
        return rc;
    }

    gpio_init_callback(&_data->isr_cb, ft3168_isr_cb, BIT(gpio->pin));
    rc = gpio_add_callback(gpio->port, &_data->isr_cb);
    if (rc != 0) {
        LOG_ERR("Error %d: failed to add callback on %s pin %d", rc, gpio->port->name, gpio->pin);
        return rc;
    }

    // enable last, once the callback is in place
    rc = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_INACTIVE);
    if (rc != 0) {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", rc, gpio->port->name,
                gpio->pin);
        return rc;
    }

    LOG_DBG("Set up gpio at %s pin %d", gpio->port->name, gpio->pin);

    return 0;
}
#endif

int32_t ft3168_gpio_pulse(void* gpio_ctx, bool pulse_hi, uint64_t time_ms)
{
    struct gpio_dt_spec* gpio = (struct gpio_dt_spec*)gpio_ctx;

    // reset-gpios is optional in devicetree
    if (gpio->port == NULL) {
        return -ENOTSUP;
    }

    if (!gpio_is_ready_dt(gpio)) {
        LOG_ERR("GPIO port %s is not ready", gpio->port->name);
        return -ENODEV;
    }

    int rc = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
    if (rc) {
        LOG_ERR("Error %d: failed to configure %s pin %d", rc, gpio->port->name, gpio->pin);
        return rc;
    }

    rc = gpio_pin_set_dt(gpio, (pulse_hi ? 1 : 0));
    if (rc) {
        LOG_ERR("Failed to set GPIO value. %s", strerror(abs(rc)));
        return rc;
    }

    k_sleep(K_MSEC(time_ms));

    rc = gpio_pin_set_dt(gpio, (!pulse_hi ? 1 : 0));
    if (rc) {
        LOG_ERR("Failed to set GPIO value. %s", strerror(abs(rc)));
        return rc;
    }

    return 0;
}

#endif

/* ========================================================================= */
/* Register Access */
/* ========================================================================= */

static void ft3168_work_handler(struct k_work* work)
{
    ft3168_data_t*         _data = CONTAINER_OF(work, ft3168_data_t, touch_work);
    const struct device*   dev   = _data->dev;
    const ft3168_config_t* _cfg  = dev->config;

    ft3168_reg_map_t regs;

    // one burst covers 0x00..0x0E: status + both touch point blocks
    int rc = ft3168_reg_read((void*)&_cfg->i2c, FT3168_REG_MODE_SW, (uint8_t*)&regs, sizeof(regs));
    if (rc) {
        LOG_ERR("Failed to read touch data. %s", strerror(abs(rc)));
        return;
    }

    // only the first point is reported; the event flag is authoritative because
    // the lift-up report still carries the last coordinates
    const ft3168_evt_flag_t evt = regs.tp1.xh.evt_flag;
    const bool down = (regs.curr_pt.n_touch_pts > 0u) &&
                      (evt == FT3168_EVT_PRESS_DOWN || evt == FT3168_EVT_CONTACT);

    if (down) {
        const uint16_t x = ft3168_tp_x(&regs.tp1);
        const uint16_t y = ft3168_tp_y(&regs.tp1);

        input_touchscreen_report_pos(dev, x, y, K_FOREVER);
        input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
        _data->pressed = true;

        LOG_DBG("touch x=%u y=%u evt=%u", x, y, evt);
    } else if (_data->pressed) {
        input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
        _data->pressed = false;

        LOG_DBG("release");
    }
}

/**
 * @brief INT pin callback; defers the I2C read to the system work queue.
 */
static void ft3168_isr_cb(const struct device* port, struct gpio_callback* cb, unsigned int pins)
{
    ft3168_data_t* _data = CONTAINER_OF(cb, ft3168_data_t, isr_cb);
    k_work_submit(&_data->touch_work);
}

/**
 * @brief Get or set the device mode (register 0x00, bits [6:4]).
 *
 * @param[in]       dev     device node pointer
 * @param[in]       get_set true to read the mode into *mode, false to write *mode
 * @param[in,out]   mode    mode to write, or where the read mode is stored
 */
static int32_t ft3168_work_mode(const struct device* dev, bool get_set, ft3168_mode_t* mode)
{
    FT3168_UNPACK(dev);

    ft3168_mode_sw_reg_t mode_sw_reg = {};

    int rc = 0;
    if (get_set) {
        rc = ft3168_reg_read((void*)&_cfg->i2c, FT3168_REG_MODE_SW, (uint8_t*)&mode_sw_reg, 1u);
        if (rc) {
            LOG_ERR("Failed to fetch FT3168 mode. %s", strerror(abs(rc)));
        } else {
            *mode = mode_sw_reg.mode;
        }
    } else {
        mode_sw_reg.mode = *mode;
        uint8_t mask     = (0b111) << 4u;
        rc = ft3168_reg_update((void*)&_cfg->i2c, FT3168_REG_MODE_SW, *((uint8_t*)&mode_sw_reg),
                               mask);
        if (rc) {
            LOG_ERR("Failed to set FT3168 mode. %s", strerror(abs(rc)));
        }
    }

    return rc;
}

/**
 * @brief Reset the FT3168 with a 10 ms pulse and wait for its firmware to boot.
 *
 * Skipped when no reset-gpios is defined.
 *
 * @param[in]   dev     device node pointer
 */
static int ft3168_reset(const struct device* dev)
{
    FT3168_UNPACK(dev);

    int rc = ft3168_gpio_pulse((void*)&_cfg->reset_gpio, true, 10u);
    if (rc == -ENOTSUP) {
        LOG_DBG("No reset GPIO; skipping hardware reset");
        return 0;
    }
    if (rc) {
        LOG_ERR("Failed to reset FT3168. %s", strerror(abs(rc)));
        return rc;
    }

    // controller firmware does not respond until it has booted
    k_msleep(FT3168_RESET_BOOT_MS);

    return 0;
}

/**
 * @brief Initialize the FT3168 device
 *
 * @param[in]   dev     device node pointer
 */
static int ft3168_init(const struct device* dev)
{
    // ======================== SW INIT =======================================
    FT3168_UNPACK(dev);

    // store for use in ISR and other contexts
    _data->dev = dev;

    // this will handle touch points
    k_work_init(&_data->touch_work, ft3168_work_handler);

    // ======================== HW INIT =======================================

    int rc = ft3168_i2c_init((void*)&_cfg->i2c);
    if (rc) {
        LOG_ERR("Failed to initialize I2C. %s", strerror(abs(rc)));
        return rc;
    }

    rc = ft3168_reset(dev);
    if (rc) {
        LOG_ERR("Failed to reset FT3168. %s", strerror(abs(rc)));
        return rc;
    }

    // mode check
    ft3168_mode_t mode = FT3168_MODE_WORKING;
    rc                 = ft3168_work_mode(dev, true, &mode);
    if (rc) {
        LOG_ERR("Failed to verify FT3168. %s", strerror(abs(rc)));
        return rc;
    }
    if (mode != FT3168_MODE_WORKING) {
        LOG_ERR("FT3168 not in working mode (mode %u)", mode);
        return -EIO;
    }

#ifdef CONFIG_INPUT_FT3168_INTERRUPT
    // enable INT only once the controller is up and answering
    rc = ft3168_isr_init(dev);
    if (rc) {
        LOG_ERR("Failed to configure GPIO interrupt. %s", strerror(abs(rc)));
        return rc;
    }
#endif

    // ======================== INIT OK =======================================
    LOG_DBG("FT3168 init successful.");
    return 0;
}

/* ========================================================================= */
/* Zephyr Driver Boilerplate */
/* ========================================================================= */

#define FT3168_INIT(n)                                                                             \
    static ft3168_data_t ft3168_data_##n;                                                          \
                                                                                                   \
    static const ft3168_config_t ft3168_config_##n = {                                             \
        .common     = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(n),                                   \
        .i2c        = I2C_DT_SPEC_INST_GET(n),                                                     \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, { 0 }),                             \
        COND_CODE_1(CONFIG_INPUT_FT3168_INTERRUPT,                                                 \
                    (.int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios), ), ())                       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, ft3168_init, NULL, &ft3168_data_##n, &ft3168_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

INPUT_TOUCH_STRUCT_CHECK(ft3168_config_t);

DT_INST_FOREACH_STATUS_OKAY(FT3168_INIT)
