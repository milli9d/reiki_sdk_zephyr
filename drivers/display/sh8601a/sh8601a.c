/*
 * Copyright (c) 2025 Milind Singh [milind345@gmail.com]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dbi.h>

#include <sh8601a.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sh8601a, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT shenge_sh8601a

typedef struct sh8601a_config {
    const struct device*   mipi_dev;
    struct mipi_dbi_config dbi_config;
} sh8601a_config_t;

typedef struct sh8601a_data {
    /* nothing for now */
} sh8601a_data_t;

static DEVICE_API(display, sh8601a_driver_api) = {};

static int sh8601a_init(const struct device* dev)
{
    const sh8601a_config_t* config = dev->config;

    LOG_DBG("Initializing SH8601 display");

    if (!device_is_ready(config->mipi_dev)) {
        LOG_ERR("MIPI Device not ready!");
        return -EINVAL;
    }

    /* check presence using a nop command */
    int rc = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, SH8601_CMD_NOP, NULL, 0);
    if (rc < 0) {
        LOG_ERR("Failed to send NOP command");
        return rc;
    }

    LOG_DBG("SH8601 display initialized");
    return 0;
}

#define SH8601_INIT(inst)                                                                       \
    static struct sh8601a_data         sh8601a_data_##inst;                                     \
    static const struct sh8601a_config sh8601a_cfg_##inst = {                                   \
        .mipi_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                           \
    };                                                                                          \
                                                                                                \
    DEVICE_DT_INST_DEFINE(inst, &sh8601a_init, NULL, &sh8601a_data_##inst, &sh8601a_cfg_##inst, \
                          POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &sh8601a_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SH8601_INIT)
