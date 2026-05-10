/**
 * @file tca9544.c
 * @brief TCA9544APWR I2C GPIO Expander Driver
 *
 * This file contains the implementation of the TCA9544APWR GPIO expander driver
 * for the Zephyr RTOS. The driver communicates with the TCA9544APWR device over
 * the I2C bus to control GPIO pins and read their states.
 *
 * The driver provides functions to:
 * - Read and write registers of the TCA9544APWR device.
 * - Get the raw value of the GPIO port.
 * - Initialize the driver and check the availability of the I2C bus.
 *
 * The driver is designed to be used with the Zephyr device model and follows
 * the standard GPIO driver API.
 *
 * @note This driver is intended for use with the TCA9544APWR I2C GPIO expander
 *       and may not be compatible with other devices.
 *
 * @copyright 2025 Milind Singh <milind345@gmail.com>
 * @license SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_tca9544apwr, 4);

#include <tca9544apwr.h>

#if 0
__subsystem struct gpio_driver_api {
    int (*pin_configure)(const struct device* port, gpio_pin_t pin, gpio_flags_t flags);
#ifdef CONFIG_GPIO_GET_CONFIG
    int (*pin_get_config)(const struct device* port, gpio_pin_t pin, gpio_flags_t* flags);
#endif
    int (*port_get_raw)(const struct device* port, gpio_port_value_t* value);
    int (*port_set_masked_raw)(const struct device* port, gpio_port_pins_t mask,
                               gpio_port_value_t value);
    int (*port_set_bits_raw)(const struct device* port, gpio_port_pins_t pins);
    int (*port_clear_bits_raw)(const struct device* port, gpio_port_pins_t pins);
    int (*port_toggle_bits)(const struct device* port, gpio_port_pins_t pins);
    int (*pin_interrupt_configure)(const struct device* port, gpio_pin_t pin,
                                   enum gpio_int_mode mode, enum gpio_int_trig trig);
    int (*manage_callback)(const struct device* port, struct gpio_callback* cb, bool set);
    uint32_t (*get_pending_int)(const struct device* dev);
#ifdef CONFIG_GPIO_GET_DIRECTION
    int (*port_get_direction)(const struct device* port, gpio_port_pins_t map,
                              gpio_port_pins_t* inputs, gpio_port_pins_t* outputs);
#endif /* CONFIG_GPIO_GET_DIRECTION */
};

/**
 * This structure is common to all GPIO drivers and is expected to be the first
 * element in the driver's struct driver_data declaration.
 */
struct gpio_driver_data {
    /** Mask identifying pins that are configured as active low.
     *
     * Management of this mask is the responsibility of the
     * wrapper functions in this header.
     */
    gpio_port_pins_t invert;
};

/**
 * This structure is common to all GPIO drivers and is expected to be
 * the first element in the object pointed to by the config field
 * in the device structure.
 */
struct gpio_driver_config {
    /** Mask identifying pins supported by the controller.
     *
     * Initialization of this mask is the responsibility of device
     * instance generation in the driver.
     */
    gpio_port_pins_t port_pin_mask;
};

#endif

/* ========================================================================= */
/* Private Data */
/* ========================================================================= */

/* GPIO private config */
typedef struct gpio_tca9544apwr_config {
    struct gpio_driver_config drv_cfg;
    struct i2c_dt_spec        i2c_bus;
    uint8_t                   capabilities;
} gpio_tca9544apwr_config_t;

/* GPIO private data */
typedef struct gpio_tca9544apwr_data {
    struct gpio_driver_data drv_data;
} gpio_tca9544apwr_data_t;

/* ========================================================================= */
/* Private API Declarations */
/* ========================================================================= */

static int gpio_tca9544apwr_reg_read(const struct device* dev, uint8_t reg, uint8_t* data);
static int gpio_tca9544apwr_reg_update(const struct device* dev, uint8_t reg, uint8_t mask,
                                       uint8_t data);
static int gpio_tca9544apwr_reg_write(const struct device* dev, uint8_t reg, uint8_t data);

static int gpio_tca9544apwr_port_get_direction(const struct device* port, gpio_port_pins_t map,
                                               gpio_port_pins_t* inputs, gpio_port_pins_t* outputs);
static int gpio_tca9544apwr_port_clear_bits_raw(const struct device* dev, gpio_port_pins_t pins);
static int gpio_tca9544apwr_port_set_bits_raw(const struct device* dev, gpio_port_pins_t pins);
static int gpio_tca9544apwr_port_set_masked_raw(const struct device* dev, gpio_port_pins_t mask,
                                                gpio_port_value_t value);
static int gpio_tca9544apwr_port_get_raw(const struct device* dev, gpio_port_value_t* value);
static int gpio_tca9544apwr_init(const struct device* dev);

/* ========================================================================= */
/* Private API */
/* ========================================================================= */

/**
 * @brief Reads a register from the TCA9544A GPIO expander.
 *
 * This function communicates with the TCA9544A device to read the value
 * of a specified register. The data read from the register is stored in
 * the provided data buffer.
 *
 * @param dev Pointer to the device structure representing the TCA9544A.
 * @param reg The register address to read from.
 * @param data Pointer to a buffer where the read data will be stored.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int gpio_tca9544apwr_reg_read(const struct device* dev, uint8_t reg, uint8_t* data)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* check that I2C bus is available */
    const struct i2c_dt_spec* i2c_bus = &drv_cfg->i2c_bus;
    if (!device_is_ready(i2c_bus->bus)) {
        LOG_ERR("I2C bus is not available");
        return -EINVAL;
    }

    int rc = i2c_reg_read_byte_dt(i2c_bus, reg, data);
    if (rc < 0) {
        LOG_ERR("Failed to I2C read register 0x%02X", reg);
        return rc;
    }

    return 0;
}

/**
 * @brief Update a specific register of the TCA9544A GPIO multiplexer.
 *
 * This function updates the specified register of the TCA9544A device by applying
 * a mask to the current register value and writing the new data.
 *
 * @param dev Pointer to the device structure representing the TCA9544A device.
 * @param reg The register address to be updated.
 * @param mask The mask to apply to the current register value.
 * @param data The new data to write to the register after applying the mask.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int gpio_tca9544apwr_reg_update(const struct device* dev, uint8_t reg, uint8_t mask,
                                       uint8_t data)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* check that I2C bus is available */
    const struct i2c_dt_spec* i2c_bus = &drv_cfg->i2c_bus;
    if (!device_is_ready(i2c_bus->bus)) {
        LOG_ERR("I2C bus is not available");
        return -EINVAL;
    }

    int rc = i2c_reg_update_byte_dt(i2c_bus, reg, mask, data);
    if (rc < 0) {
        LOG_ERR("Failed to I2C write register 0x%02X", reg);
        return rc;
    }

    return 0;
}

/**
 * @brief Write data to a specific register of the TCA9544A GPIO expander.
 *
 * This function writes a byte of data to the specified register of the
 * TCA9544A device. It is used to configure the GPIO expander's behavior
 * by setting the appropriate registers.
 *
 * @param dev Pointer to the device structure representing the TCA9544A.
 * @param reg The register address to which the data will be written.
 * @param data The data byte to be written to the specified register.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int gpio_tca9544apwr_reg_write(const struct device* dev, uint8_t reg, uint8_t data)
{
    return gpio_tca9544apwr_reg_update(dev, reg, UINT8_MAX, data);
}

#ifdef CONFIG_GPIO_GET_DIRECTION
/**
 * @brief Get the direction of specified GPIO pins in a port.
 *
 * This function retrieves the direction of each pin specified in the map.
 * If inputs or outputs are NULL, the respective directions will not be filled.
 *
 * @param port Pointer to the device structure for the driver instance.
 * @param map Bitmap of pin directions to query.
 * @param inputs Pointer to a variable where input directions will be stored.
 * @param outputs Pointer to a variable where output directions will be stored.
 *
 * @retval 0 If successful.
 * @retval -ENODEV if the port is not ready.
 * @retval -EINVAL if the pin mask is invalid.
 */
static int gpio_tca9544apwr_port_get_direction(const struct device* port, gpio_port_pins_t map,
                                               gpio_port_pins_t* inputs, gpio_port_pins_t* outputs)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = port->config;
    const gpio_tca9544apwr_data_t* const   drv_data = port->data;
    gpio_port_pins_t                       pin_mask = drv_cfg->drv_cfg.port_pin_mask;

    /* Check if the port is ready */
    if (!device_is_ready(port)) {
        LOG_ERR("GPIO port is not ready");
        return -ENODEV;
    }

    /* Validate the pin mask */
    if ((map & pin_mask) != map) {
        LOG_ERR("Invalid pin mask: 0x%08X", map);
        return -EINVAL;
    }

    /* If inputs or outputs are NULL, we do not need to fill them */
    if (inputs == NULL && outputs == NULL) {
        LOG_DBG("No inputs or outputs requested, returning early");
        return 0;
    }

    /* If inputs are requested, set them to the pin mask */
    if (inputs != NULL) {
        *inputs = map & pin_mask;
    }

    /* If outputs are requested, set them to the pin mask */
    if (outputs != NULL) {
        *outputs = map & pin_mask;
    }

    LOG_DBG("GPIO port direction retrieved: inputs=0x%08X, outputs=0x%08X", *inputs, *outputs);

    /* Return success */
    return 0;
}
#endif /* CONFIG_GPIO_GET_DIRECTION */

/**
 * @brief Toggles specified bits in the output port register of the TCA9544A device.
 *
 * This function reads the current value of the output port register, toggles
 * the specified bits, and writes the new value back to the register.
 *
 * @param dev Pointer to the device structure representing the TCA9544A device.
 * @param pins A bitmask representing the pins to be toggled.
 *
 * @return 0 on success, or a negative error code on failure.
 *
 * @note This function will log an error message if the I2C read or write operation fails.
 */
static int gpio_tca9544apwr_port_toggle_bits(const struct device* dev, gpio_port_pins_t pins)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    gpio_port_value_t current_value = 0u;

    /* read the register */
    int rc = gpio_tca9544apwr_port_get_raw(dev, &current_value);
    if (rc < 0) {
        LOG_ERR("Failed to read current port value");
        return rc;
    }

    /* toggle the bits */
    current_value ^= pins;

    /* write the new value back to the port */
    rc = gpio_tca9544apwr_reg_write(dev, TCA9544APWR_REG_OUTPUT_PORT_WR, current_value);
    if (rc < 0) {
        LOG_ERR("Failed to write toggled port value");
        return rc;
    }

    return 0;
}

/**
 * @brief Clears specified bits in the output port register of the TCA9544A device.
 *
 * This function writes a value of 0 to the output port register, effectively clearing
 * the specified pins. It uses a mask to determine which bits to clear.
 *
 * @param dev Pointer to the device structure representing the TCA9544A device.
 * @param pins A bitmask representing the pins to be cleared.
 *
 * @return 0 on success, or a negative error code on failure.
 *
 * @note This function will log an error message if the I2C write operation fails.
 */
static int gpio_tca9544apwr_port_clear_bits_raw(const struct device* dev, gpio_port_pins_t pins)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* write 0u to output port register with mask */
    int rc = gpio_tca9544apwr_reg_update(dev, TCA9544APWR_REG_OUTPUT_PORT_WR, pins, 0u);
    if (rc < 0) {
        LOG_ERR("Failed to I2C write output port register");
        return rc;
    }

    return 0;
}

/**
 * @brief Set the specified GPIO pins to high or low.
 *
 * This function updates the output port register of the TCA9544A GPIO expander
 * by setting the specified pins to the desired state. It uses the I2C interface
 * to communicate with the device.
 *
 * @param dev Pointer to the device structure representing the TCA9544A.
 * @param pins A bitmask representing the GPIO pins to be set. Each bit in the
 *             mask corresponds to a pin, where a value of 1 sets the pin high
 *             and a value of 0 sets it low.
 *
 * @return 0 on success, or a negative error code on failure. If the I2C write
 *         operation fails, an error message is logged.
 */
static int gpio_tca9544apwr_port_set_bits_raw(const struct device* dev, gpio_port_pins_t pins)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* write to output port register with mask */
    int rc = gpio_tca9544apwr_reg_update(dev, TCA9544APWR_REG_OUTPUT_PORT_WR, pins, pins);
    if (rc < 0) {
        LOG_ERR("Failed to I2C write output port register");
        return rc;
    }

    return 0;
}

/**
 * @brief Set the output port of the TCA9544A GPIO expander with a masked value.
 *
 * This function updates the output port register of the TCA9544A device
 * by applying a mask to the specified value. Only the bits specified by
 * the mask will be modified, while the others will remain unchanged.
 *
 * @param dev Pointer to the device structure representing the TCA9544A.
 * @param mask A bitmask indicating which pins to modify.
 * @param value The new values to set for the specified pins in the mask.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int gpio_tca9544apwr_port_set_masked_raw(const struct device* dev, gpio_port_pins_t mask,
                                                gpio_port_value_t value)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* write to output port register with mask */
    int rc = gpio_tca9544apwr_reg_update(dev, TCA9544APWR_REG_OUTPUT_PORT_WR, mask, value);
    if (rc < 0) {
        LOG_ERR("Failed to I2C write output port register with mask");
        return rc;
    }

    return 0;
}

/**
 * @brief Get the raw value of the GPIO port from the TCA9544APWR device.
 *
 * This function reads the input port register of the TCA9544APWR device
 * and retrieves the current value of the GPIO port. The value is then
 * converted to a gpio_port_value_t type and returned through the provided
 * pointer.
 *
 * @param dev Pointer to the device structure representing the TCA9544APWR.
 * @param value Pointer to a gpio_port_value_t where the read value will be stored.
 *
 * @return 0 on success, or a negative error code on failure.
 */
static int gpio_tca9544apwr_port_get_raw(const struct device* dev, gpio_port_value_t* value)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* read TCA9544APWR input port */
    uint8_t val = 0u;
    int     rc  = gpio_tca9544apwr_reg_read(dev, TCA9544APWR_REG_INPUT_PORT_RD, &val);
    if (rc < 0) {
        LOG_ERR("Failed to I2C read input port register");
        return rc;
    }

    /* Convert the read value to a gpio_port_value_t */
    *value = (gpio_port_value_t)val;

    return 0;
}

/**
 * @brief Initializes the TCA9544A GPIO multiplexer.
 *
 * This function configures the TCA9544A device for operation. It sets up
 * the necessary GPIO pins and prepares the device for use in the system.
 *
 * @param dev Pointer to the device structure for the TCA9544A.
 * @return 0 on success, or a negative error code on failure.
 */
static int gpio_tca9544apwr_init(const struct device* dev)
{
    const gpio_tca9544apwr_config_t* const drv_cfg  = dev->config;
    const gpio_tca9544apwr_data_t* const   drv_data = dev->data;

    /* check that I2C bus is available */
    const struct i2c_dt_spec* i2c_bus = &drv_cfg->i2c_bus;
    if (!device_is_ready(i2c_bus->bus)) {
        LOG_ERR("I2C bus is not available");
        return -EINVAL;
    }

    /* check presence of TCA9544APWR by ACK */
    uint8_t dummy;
    int     rc = i2c_reg_read_byte_dt(i2c_bus, TCA9544APWR_REG_INPUT_PORT_RD, &dummy);
    if (rc < 0) {
        LOG_ERR("Failed to I2C read from TCA9544APWR device");
        return rc;
    }
    if (dummy == 0xFF) {
        LOG_ERR("TCA9544APWR device not found or not responding");
        return -ENODEV;
    }

    /* initialize the output port */
    rc = gpio_tca9544apwr_reg_write(dev, TCA9544APWR_REG_OUTPUT_PORT_WR, 0x00);
    if (rc < 0) {
        LOG_ERR("Failed to I2C write output port register");
        return rc;
    }

    /* set pin polarity to default (0x00); Zephyr GPIO driver manages inversion in the wrapper */
    rc = gpio_tca9544apwr_reg_write(dev, TCA9544APWR_REG_POLARITY_INVERSION,
                                    drv_data->drv_data.invert);
    if (rc < 0) {
        LOG_ERR("Failed to I2C write polarity invert register");
        return rc;
    }

    LOG_DBG("TCA9544APWR device initialized successfully");

    return 0;
}

/* ========================================================================= */
/* Zephyr Driver Declaration */
/* ========================================================================= */

#define DT_DRV_COMPAT ti_tca9544apwr

static DEVICE_API(gpio, gpio_tca9544apwr_drv_api) = {
    .pin_configure = NULL,
#ifdef CONFIG_GPIO_GET_CONFIG
    .pin_get_config = NULL,
#endif
    .port_get_raw            = gpio_tca9544apwr_port_get_raw,
    .port_set_masked_raw     = gpio_tca9544apwr_port_set_masked_raw,
    .port_set_bits_raw       = gpio_tca9544apwr_port_set_bits_raw,
    .port_clear_bits_raw     = gpio_tca9544apwr_port_clear_bits_raw,
    .port_toggle_bits        = gpio_tca9544apwr_port_toggle_bits,
    .pin_interrupt_configure = NULL,
    .manage_callback         = NULL,
    .get_pending_int         = NULL,
#ifdef CONFIG_GPIO_GET_DIRECTION
    .port_get_direction = gpio_tca9544apwr_port_get_direction,
#endif
};

/* TODO: Add DT_DEFINE */
#define GPIO_TCA9544APWR_DEVICE_INSTANCE(inst)                                                   \
    static const gpio_tca9544apwr_config_t gpio_tca9544apwr_##inst##_cfg = {                     \
        .drv_cfg = { .port_pin_mask = GPIO_DT_INST_PORT_PIN_MASK_NGPIOS_EXC(                     \
                         inst, DT_INST_PROP(inst, ngpios)) },                                    \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst)                                                    \
    };                                                                                           \
                                                                                                 \
    static gpio_tca9544apwr_data_t gpio_tca9544apwr_##inst##_drvdata;                            \
                                                                                                 \
    DEVICE_DT_INST_DEFINE(inst, gpio_tca9544apwr_init, NULL, &gpio_tca9544apwr_##inst##_drvdata, \
                          &gpio_tca9544apwr_##inst##_cfg, POST_KERNEL,                           \
                          CONFIG_GPIO_TCA9544APWR_INIT_PRIORITY, &gpio_tca9544apwr_drv_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_TCA9544APWR_DEVICE_INSTANCE)
