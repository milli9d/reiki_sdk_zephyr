#pragma once

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/drivers/gpio/gpio_utils.h>

#define TCA9544APWR_REG_INPUT_PORT_RD      0x00u
#define TCA9544APWR_REG_OUTPUT_PORT_WR     0x01u
#define TCA9544APWR_REG_POLARITY_INVERSION 0x02u
#define TCA9544APWR_REG_CONFIGURATION      0x03u
