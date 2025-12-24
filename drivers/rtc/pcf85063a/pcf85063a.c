#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(PCF86053A, 4);

#include <pcf85063a.h>

#define DT_DRV_COMPAT nxp_pcf85063a_rtc

/* ========================================================================= */
/* Private Data */
/* ========================================================================= */

typedef struct pcf85063a_cfg {
    const struct i2c_dt_spec i2c;
} pcf85063a_cfg_t;

typedef struct pcf85063a_data {
    /* nothing for now */
} pcf85063a_data_t;

/* ========================================================================= */
/* Private Helper API Declarations */
/* ========================================================================= */

static int pcf86053a_decode_secs(uint8_t reg);
static int pcf86053a_decode_mins(uint8_t reg);

/* ========================================================================= */
/* Private API Declarations */
/* ========================================================================= */

static bool pcf86053a_ping(const struct device* dev);
static int  pcf86053a_init(const struct device* dev);
static int  pcf86053a_set_time(const struct device* dev, const struct rtc_time* timeptr);
static int  pcf86053a_get_time(const struct device* dev, struct rtc_time* timeptr);

/* ========================================================================= */
/* Private Helper API Definitions */
/* ========================================================================= */

static int pcf86053a_decode_secs(uint8_t reg)
{
    int  out        = 0;
    bool osc_stable = (reg & PCF85063A_REG_SECS_OS);
    if (!osc_stable) {
        LOG_ERR("Oscillator is not stable.");
    }
    out += PCF85063A_REG_SECS_UNITS(reg);
    out += (PCF85063A_REG_SECS_TENS(reg) * 10u);
    return out;
}

static int pcf86053a_decode_mins(uint8_t reg)
{
    int out = 0;
    out += PCF85063A_REG_MINS_UNITS(reg);
    out += (PCF85063A_REG_MINS_TENS(reg) * 10u);
    return out;
}

/* ========================================================================= */
/* Private API Definitions */
/* ========================================================================= */

static DEVICE_API(rtc, pcf85063a_api) = {
    .set_time = pcf86053a_set_time,
    .get_time = pcf86053a_get_time,
};

static int pcf86053a_set_time(const struct device* dev, const struct rtc_time* timeptr)
{
    return 0;
}

static int pcf86053a_get_time(const struct device* dev, struct rtc_time* timeptr)
{
    /* check if I2C bus is up */
    const pcf85063a_cfg_t* cfg = (pcf85063a_cfg_t*)dev->config;
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus is not available.");
        return -ENODEV;
    }

    uint8_t time_regs[PCF85063A_REG_YRS - PCF85063A_REG_SECS] = {};
    int     rc = i2c_burst_read_dt(&cfg->i2c, PCF85063A_REG_SECS, time_regs,
                                   PCF85063A_REG_YRS - PCF85063A_REG_SECS);
    if (rc != 0) {
        LOG_ERR("Error reading time registers");
        return rc;
    }

    timeptr->tm_sec = pcf86053a_decode_secs(time_regs[PCF85063A_REG_SECS - PCF85063A_REG_SECS]);
    timeptr->tm_min = pcf86053a_decode_secs(time_regs[PCF85063A_REG_MINS - PCF85063A_REG_SECS]);

    return 0;
}

/**
 * @brief Checks if the PCF85063A RTC device is present and accessible on the I2C bus.
 *
 * This function verifies that the I2C bus associated with the device is ready,
 * and then attempts to read a control register from the PCF85063A RTC to confirm
 * device presence and communication.
 *
 * @param dev Pointer to the device structure for the PCF85063A RTC.
 * @return true if the device is present and accessible; false otherwise.
 *         If the I2C bus is not available, returns -ENODEV.
 */
static bool pcf86053a_ping(const struct device* dev)
{
    /* check if I2C bus is up */
    const pcf85063a_cfg_t* cfg = (pcf85063a_cfg_t*)dev->config;
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus is not available.");
        return -ENODEV;
    }

    /* read a register to check presence */
    uint8_t reg = 0;
    int     rc  = i2c_reg_read_byte_dt(&cfg->i2c, PCF85063A_REG_CTRL1, &reg);
    if (rc != 0) {
        return false;
    }

    return true;
}

/**
 * @brief Initializes the PCF86053A RTC device.
 *
 * This function checks if the I2C bus is ready and if the PCF86053A device is present
 * on the bus. If either check fails, an error is logged and a negative error code is returned.
 *
 * @param dev Pointer to the device structure for the PCF86053A.
 * @return 0 on success, -ENODEV if the I2C bus or device is not available.
 */
static int pcf86053a_init(const struct device* dev)
{
    /* check if I2C bus is up */
    const pcf85063a_cfg_t* cfg = (pcf85063a_cfg_t*)dev->config;
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus is not available.");
        return -ENODEV;
    }

    /* check if device is present */
    if (!pcf86053a_ping(dev)) {
        LOG_ERR("PCF86053A device is not present.");
        return -ENODEV;
    }

    LOG_DBG("PCF86053A device initialized!");
    return 0;
}

#define PCF85063A_INIT(inst)                                                                      \
    static const pcf85063a_cfg_t pcf85063a_cfg_##inst = {                                         \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                        \
    };                                                                                            \
    static pcf85063a_data_t pcf85063a_data_##inst;                                                \
                                                                                                  \
    DEVICE_DT_INST_DEFINE(inst, &pcf86053a_init, NULL, &pcf85063a_data_##inst,                    \
                          &pcf85063a_cfg_##inst, POST_KERNEL, CONFIG_RTC_PCF85063A_INIT_PRIORITY, \
                          &pcf85063a_api);

DT_INST_FOREACH_STATUS_OKAY(PCF85063A_INIT)