/*
 * Copyright (c) 2026 Milind Singh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT shenge_sh8601

#include <zephyr/device.h>
#include <zephyr/display/mipi_display.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <string.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(mspi)
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sh8601, CONFIG_DISPLAY_LOG_LEVEL);

/*
 * The QSPI interface frames every access as an opcode followed by a 24-bit
 * address that carries the DCS command: {0x00, CMD, 0x00}.
 */
#define SH8601_OPCODE_WRITE_1LINE 0x02
#define SH8601_OPCODE_WRITE_QUAD  0x32

#define SH8601_MSPI_TIMEOUT_MS 100

#define SH8601_RESET_PULSE_MS 10
#define SH8601_RESET_WAIT_MS  120
#define SH8601_SLPOUT_WAIT_MS 120

/* WRCTRLD: enable the brightness control block so that WRDISBV (0x51) applies */
#define SH8601_WRCTRLD_BCTRL BIT(5)

typedef int (*sh8601_cmd_write_fn)(const struct device* dev, uint8_t cmd, const void* buf,
                                   size_t len);

struct sh8601_config {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
    struct spi_dt_spec spi;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(mspi)
    const struct device* mspi_bus;
    struct mspi_dev_id   dev_id;
    enum mspi_io_mode    pixel_io_mode;
    bool                 sw_multi_periph;
#endif
    /* Commands and parameters */
    sh8601_cmd_write_fn cmd_write;
    /* Pixel data following a memory write command */
    sh8601_cmd_write_fn pixel_write;
    struct gpio_dt_spec reset_gpio;
    uint16_t            width;
    uint16_t            height;
    uint16_t            x_offset;
    uint16_t            y_offset;
    bool                red_blue_swap;
};

struct sh8601_data {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(mspi)
    struct mspi_dev_cfg mspi_cfg;
    /* Pixel transfers try DMA first and drop to PIO if unsupported */
    enum mspi_xfer_mode pixel_xfer_mode;
#endif
    enum display_pixel_format pixel_format;
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
/**
 * @brief Send a DCS command and its parameters over single-line SPI.
 *
 * Frames the command as opcode 0x02 plus a 24-bit address {0x00, cmd, 0x00},
 * followed by the parameter bytes, in one chip select window.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   cmd         DCS command byte
 * @param[in]   buf         parameter bytes, may be NULL
 * @param[in]   len         number of parameter bytes
 *
 * @return 0 on success, negative errno from the SPI driver otherwise
 */
static int sh8601_spi_cmd_write(const struct device* dev, uint8_t cmd, const void* buf, size_t len)
{
    const struct sh8601_config* config     = dev->config;
    uint8_t                     header[4]  = { SH8601_OPCODE_WRITE_1LINE, 0x00, cmd, 0x00 };
    struct spi_buf              tx_bufs[2] = {
        { .buf = header,     .len = sizeof(header) },
        { .buf = (void*)buf, .len = len            },
    };
    struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count   = (buf != NULL && len > 0) ? 2 : 1,
    };

    return spi_write_dt(&config->spi, &tx);
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(mspi)
/**
 * @brief Acquire the MSPI bus and switch it to the requested IO mode.
 *
 * Re-applies the full device config when the controller is shared in software
 * (sw-multi-periph), and releases the bus again if the IO mode change fails.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   io_mode     IO mode for the next transfer
 *
 * @return 0 with the bus held on success, negative errno otherwise
 */
static int sh8601_mspi_acquire(const struct device* dev, enum mspi_io_mode io_mode)
{
    const struct sh8601_config* config = dev->config;
    struct sh8601_data*         data   = dev->data;
    int                         ret;

    if (config->sw_multi_periph) {
        ret = mspi_dev_config(config->mspi_bus, &config->dev_id, MSPI_DEVICE_CONFIG_ALL,
                              &data->mspi_cfg);
    } else {
        ret = mspi_dev_config(config->mspi_bus, &config->dev_id, MSPI_DEVICE_CONFIG_NONE, NULL);
    }
    if (ret < 0) {
        return ret;
    }

    if (data->mspi_cfg.io_mode != io_mode) {
        data->mspi_cfg.io_mode = io_mode;
        ret = mspi_dev_config(config->mspi_bus, &config->dev_id, MSPI_DEVICE_CONFIG_IO_MODE,
                              &data->mspi_cfg);
        if (ret < 0) {
            (void)mspi_get_channel_status(config->mspi_bus, 0);
        }
    }

    return ret;
}

/**
 * @brief Run one framed write on the MSPI bus and release the bus.
 *
 * The opcode goes in the command phase, the DCS command in the 24-bit address
 * phase and @p buf in the data phase, so the controller drives each phase
 * with the line count of @p io_mode.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   io_mode     IO mode of the transfer
 * @param[in]   xfer_mode   MSPI_PIO or MSPI_DMA
 * @param[in]   opcode      SH8601 QSPI opcode (0x02 or 0x32)
 * @param[in]   cmd         DCS command byte
 * @param[in]   buf         data bytes, may be NULL
 * @param[in]   len         number of data bytes
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_mspi_write(const struct device* dev, enum mspi_io_mode io_mode,
                             enum mspi_xfer_mode xfer_mode, uint8_t opcode, uint8_t cmd,
                             const void* buf, size_t len)
{
    const struct sh8601_config* config = dev->config;
    struct mspi_xfer_packet     pkt    = {
        .dir       = MSPI_TX,
        .cmd       = opcode,
        .address   = (uint32_t)cmd << 8,
        .num_bytes = (buf != NULL) ? len : 0,
        .data_buf  = (uint8_t*)buf,
    };
    struct mspi_xfer xfer = {
        .xfer_mode   = xfer_mode,
        .cmd_length  = 1,
        .addr_length = 3,
        .packets     = &pkt,
        .num_packet  = 1,
        .timeout     = SH8601_MSPI_TIMEOUT_MS,
    };
    int ret;

    ret = sh8601_mspi_acquire(dev, io_mode);
    if (ret < 0) {
        return ret;
    }

    ret = mspi_transceive(config->mspi_bus, &config->dev_id, &xfer);

    while (mspi_get_channel_status(config->mspi_bus, 0) == -EBUSY) {
        k_yield();
    }

    return ret;
}

/**
 * @brief Send a DCS command and its parameters over MSPI, single line, PIO.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   cmd         DCS command byte
 * @param[in]   buf         parameter bytes, may be NULL
 * @param[in]   len         number of parameter bytes
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_mspi_cmd_write(const struct device* dev, uint8_t cmd, const void* buf, size_t len)
{
    /**
     * @brief Run one framed write on the MSPI bus and release the bus.
     *
     * The opcode goes in the command phase, the DCS command in the 24-bit address
     * phase and @p buf in the data phase, so the controller drives each phase
     * with the line count of @p io_mode.
     *
     * @param[in]   dev         device node pointer
     * @param[in]   io_mode     IO mode of the transfer
     * @param[in]   xfer_mode   MSPI_PIO or MSPI_DMA
     * @param[in]   opcode      SH8601 QSPI opcode (0x02 or 0x32)
     * @param[in]   cmd         DCS command byte
     * @param[in]   buf         data bytes, may be NULL
     * @param[in]   len         number of data bytes
     *
     * @return 0 on success, negative errno otherwise
     */
    return sh8601_mspi_write(dev, MSPI_IO_MODE_SINGLE, MSPI_PIO, SH8601_OPCODE_WRITE_1LINE, cmd,
                             buf, len);
}

/**
 * @brief Send pixel data following a memory write command over MSPI.
 *
 * Uses opcode 0x32 with a quad data phase when mspi-io-mode is QUAD_1_1_4.
 * Tries DMA first and falls back to PIO for good if the controller rejects it.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   cmd         memory write command (0x2C or 0x3C)
 * @param[in]   buf         pixel data
 * @param[in]   len         number of pixel bytes
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_mspi_pixel_write(const struct device* dev, uint8_t cmd, const void* buf,
                                   size_t len)
{
    const struct sh8601_config* config = dev->config;
    struct sh8601_data*         data   = dev->data;
    uint8_t opcode = (config->pixel_io_mode == MSPI_IO_MODE_QUAD_1_1_4) ? SH8601_OPCODE_WRITE_QUAD
                                                                        : SH8601_OPCODE_WRITE_1LINE;
    int     ret;

    ret =
        sh8601_mspi_write(dev, config->pixel_io_mode, data->pixel_xfer_mode, opcode, cmd, buf, len);
    if (ret == -ENOTSUP && data->pixel_xfer_mode == MSPI_DMA) {
        LOG_INF("MSPI controller has no DMA, using PIO");
        data->pixel_xfer_mode = MSPI_PIO;
        ret = sh8601_mspi_write(dev, config->pixel_io_mode, MSPI_PIO, opcode, cmd, buf, len);
    }

    return ret;
}

/**
 * @brief Validate the MSPI bus and push the initial device configuration.
 *
 * @param[in]   dev         device node pointer
 *
 * @retval 0          success
 * @retval -ENODEV    MSPI bus not ready
 * @retval -ENOTSUP   mspi-io-mode is neither SINGLE nor QUAD_1_1_4
 * @return other negative errno from the MSPI controller
 */
static int sh8601_mspi_init(const struct device* dev)
{
    const struct sh8601_config* config = dev->config;
    struct sh8601_data*         data   = dev->data;
    int                         ret;

    if (!device_is_ready(config->mspi_bus)) {
        LOG_ERR("MSPI bus not ready");
        return -ENODEV;
    }

    if (config->pixel_io_mode != MSPI_IO_MODE_SINGLE &&
        config->pixel_io_mode != MSPI_IO_MODE_QUAD_1_1_4) {
        LOG_ERR("mspi-io-mode must be SINGLE or QUAD_1_1_4");
        return -ENOTSUP;
    }

    /* Commands are single line; the pixel mode is applied per transfer */
    data->mspi_cfg.io_mode = MSPI_IO_MODE_SINGLE;
    ret =
        mspi_dev_config(config->mspi_bus, &config->dev_id, MSPI_DEVICE_CONFIG_ALL, &data->mspi_cfg);
    if (ret < 0) {
        LOG_ERR("MSPI device config failed (%d)", ret);
        return ret;
    }

    ret = mspi_get_channel_status(config->mspi_bus, 0);
    if (ret < 0) {
        LOG_ERR("MSPI bus release failed (%d)", ret);
    }

    return ret;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(mspi) */

/**
 * @brief Send a DCS command through the bus-specific command writer.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   cmd         DCS command byte
 * @param[in]   buf         parameter bytes, may be NULL
 * @param[in]   len         number of parameter bytes
 *
 * @return 0 on success, negative errno otherwise
 */
static inline int sh8601_cmd(const struct device* dev, uint8_t cmd, const void* buf, size_t len)
{
    const struct sh8601_config* config = dev->config;

    return config->cmd_write(dev, cmd, buf, len);
}

/**
 * @brief Set the column and page address window, applying the panel offsets.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   x           first column
 * @param[in]   y           first row
 * @param[in]   w           window width in pixels
 * @param[in]   h           window height in pixels
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_set_window(const struct device* dev, uint16_t x, uint16_t y, uint16_t w,
                             uint16_t h)
{
    const struct sh8601_config* config = dev->config;
    uint16_t                    range[2];
    int                         ret;

    x += config->x_offset;
    y += config->y_offset;

    range[0] = sys_cpu_to_be16(x);
    range[1] = sys_cpu_to_be16(x + w - 1U);
    ret      = sh8601_cmd(dev, MIPI_DCS_SET_COLUMN_ADDRESS, range, sizeof(range));
    if (ret < 0) {
        return ret;
    }

    range[0] = sys_cpu_to_be16(y);
    range[1] = sys_cpu_to_be16(y + h - 1U);
    return sh8601_cmd(dev, MIPI_DCS_SET_PAGE_ADDRESS, range, sizeof(range));
}

/**
 * @brief Display API: write a rectangle of pixels to the panel.
 *
 * Contiguous buffers go out in one transfer; strided buffers are written a row
 * at a time with WRITE_MEMORY_CONTINUE.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   x           first column
 * @param[in]   y           first row
 * @param[in]   desc        buffer descriptor
 * @param[in]   buf         pixel data in the current pixel format
 *
 * @retval 0          success
 * @retval -EINVAL    empty or out of bounds rectangle, or pitch below width
 * @return other negative errno from the bus
 */
static int sh8601_write(const struct device* dev, const uint16_t x, const uint16_t y,
                        const struct display_buffer_descriptor* desc, const void* buf)
{
    const struct sh8601_config* config = dev->config;
    struct sh8601_data*         data   = dev->data;
    const uint8_t*              row    = buf;
    size_t                      bpp    = DISPLAY_BITS_PER_PIXEL(data->pixel_format) / 8U;
    uint8_t                     cmd    = MIPI_DCS_WRITE_MEMORY_START;
    int                         ret;

    if (desc->width == 0U || desc->height == 0U || desc->pitch < desc->width) {
        return -EINVAL;
    }

    if ((uint32_t)x + desc->width > config->width || (uint32_t)y + desc->height > config->height) {
        LOG_ERR("Write area outside panel");
        return -EINVAL;
    }

    ret = sh8601_set_window(dev, x, y, desc->width, desc->height);
    if (ret < 0) {
        return ret;
    }

    if (desc->pitch == desc->width) {
        return config->pixel_write(dev, cmd, buf, (size_t)desc->width * desc->height * bpp);
    }

    /* Strided buffer: one row per transfer, continuing the memory write */
    for (uint16_t i = 0; i < desc->height; i++) {
        ret = config->pixel_write(dev, cmd, row, (size_t)desc->width * bpp);
        if (ret < 0) {
            return ret;
        }
        cmd = MIPI_DCS_WRITE_MEMORY_CONTINUE;
        row += (size_t)desc->pitch * bpp;
    }

    return 0;
}

/**
 * @brief Display API: turn the panel output off (DISPOFF).
 *
 * @param[in]   dev         device node pointer
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_blanking_on(const struct device* dev)
{ return sh8601_cmd(dev, MIPI_DCS_SET_DISPLAY_OFF, NULL, 0); }

/**
 * @brief Display API: turn the panel output on (DISPON).
 *
 * @param[in]   dev         device node pointer
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_blanking_off(const struct device* dev)
{ return sh8601_cmd(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0); }

/**
 * @brief Display API: set the panel brightness (WRDISBV).
 *
 * @param[in]   dev         device node pointer
 * @param[in]   brightness  0 (darkest) to 255 (brightest)
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_set_brightness(const struct device* dev, const uint8_t brightness)
{ return sh8601_cmd(dev, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, &brightness, 1); }

/**
 * @brief Display API: report resolution, pixel formats and orientation.
 *
 * @param[in]   dev         device node pointer
 * @param[out]  capabilities filled with the panel capabilities
 */
static void sh8601_get_capabilities(const struct device*         dev,
                                    struct display_capabilities* capabilities)
{
    const struct sh8601_config* config = dev->config;
    struct sh8601_data*         data   = dev->data;

    memset(capabilities, 0, sizeof(*capabilities));
    capabilities->x_resolution            = config->width;
    capabilities->y_resolution            = config->height;
    capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565X | PIXEL_FORMAT_RGB_888;
    capabilities->current_pixel_format    = data->pixel_format;
    capabilities->current_orientation     = DISPLAY_ORIENTATION_NORMAL;
}

/**
 * @brief Display API: select RGB565X or RGB888 (COLMOD).
 *
 * @param[in]   dev         device node pointer
 * @param[in]   pixel_format requested pixel format
 *
 * @retval 0          success
 * @retval -ENOTSUP   pixel format not supported
 * @return other negative errno from the bus
 */
static int sh8601_set_pixel_format(const struct device*            dev,
                                   const enum display_pixel_format pixel_format)
{
    struct sh8601_data* data = dev->data;
    uint8_t             colmod;
    int                 ret;

    switch (pixel_format) {
        case PIXEL_FORMAT_RGB_565X:
            colmod = MIPI_DCS_PIXEL_FORMAT_16BIT;
            break;
        case PIXEL_FORMAT_RGB_888:
            colmod = MIPI_DCS_PIXEL_FORMAT_24BIT;
            break;
        default:
            return -ENOTSUP;
    }

    ret = sh8601_cmd(dev, MIPI_DCS_SET_PIXEL_FORMAT, &colmod, 1);
    if (ret < 0) {
        return ret;
    }

    data->pixel_format = pixel_format;
    return 0;
}

/**
 * @brief Display API: set the orientation; only normal is supported.
 *
 * @param[in]   dev         device node pointer
 * @param[in]   orientation requested orientation
 *
 * @retval 0          orientation is DISPLAY_ORIENTATION_NORMAL
 * @retval -ENOTSUP   any other orientation
 */
static int sh8601_set_orientation(const struct device*           dev,
                                  const enum display_orientation orientation)
{
    ARG_UNUSED(dev);

    if (orientation == DISPLAY_ORIENTATION_NORMAL) {
        return 0;
    }

    return -ENOTSUP;
}

/**
 * @brief Reset the panel.
 *
 * Pulses reset-gpios when present, otherwise sends SWRESET, then waits for
 * the controller to come out of reset.
 *
 * @param[in]   dev         device node pointer
 *
 * @retval 0          success
 * @retval -ENODEV    reset GPIO not ready
 * @return other negative errno from the GPIO or bus
 */
static int sh8601_reset(const struct device* dev)
{
    const struct sh8601_config* config = dev->config;
    int                         ret;

    if (config->reset_gpio.port == NULL) {
        ret = sh8601_cmd(dev, MIPI_DCS_SOFT_RESET, NULL, 0);
        if (ret < 0) {
            return ret;
        }
        k_msleep(SH8601_RESET_WAIT_MS);
        return 0;
    }

    if (!gpio_is_ready_dt(&config->reset_gpio)) {
        LOG_ERR("Reset GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return ret;
    }
    k_msleep(SH8601_RESET_PULSE_MS);

    ret = gpio_pin_set_dt(&config->reset_gpio, 0);
    if (ret < 0) {
        return ret;
    }
    k_msleep(SH8601_RESET_WAIT_MS);

    return 0;
}

/**
 * @brief Bring the panel out of sleep and program its default state.
 *
 * Sleep out, pixel format, MADCTL (BGR when red-blue-swap), tearing effect on,
 * brightness control enabled and full brightness. The display stays off.
 *
 * @param[in]   dev         device node pointer
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_configure(const struct device* dev)
{
    const struct sh8601_config* config = dev->config;
    struct sh8601_data*         data   = dev->data;
    uint8_t                     param;
    int                         ret;

    ret = sh8601_cmd(dev, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0);
    if (ret < 0) {
        return ret;
    }
    k_msleep(SH8601_SLPOUT_WAIT_MS);

    ret = sh8601_set_pixel_format(dev, data->pixel_format);
    if (ret < 0) {
        return ret;
    }

    param = config->red_blue_swap ? MIPI_DCS_ADDRESS_MODE_BGR : 0U;
    ret   = sh8601_cmd(dev, MIPI_DCS_SET_ADDRESS_MODE, &param, 1);
    if (ret < 0) {
        return ret;
    }

    /* Tearing effect output, V-blank only */
    param = 0U;
    ret   = sh8601_cmd(dev, MIPI_DCS_SET_TEAR_ON, &param, 1);
    if (ret < 0) {
        return ret;
    }

    param = SH8601_WRCTRLD_BCTRL;
    ret   = sh8601_cmd(dev, MIPI_DCS_WRITE_CONTROL_DISPLAY, &param, 1);
    if (ret < 0) {
        return ret;
    }

    return sh8601_set_brightness(dev, UINT8_MAX);
}

/**
 * @brief Initialize the SH8601: bus, reset, then panel configuration.
 *
 * The panel is left blanked; call display_blanking_off() to show content.
 *
 * @param[in]   dev         device node pointer
 *
 * @return 0 on success, negative errno otherwise
 */
static int sh8601_init(const struct device* dev)
{
    const struct sh8601_config* config = dev->config;
    int                         ret;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(mspi)
    if (config->mspi_bus != NULL) {
        ret = sh8601_mspi_init(dev);
        if (ret < 0) {
            return ret;
        }
    }
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
    if (config->spi.bus != NULL && !spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }
#endif

    ret = sh8601_reset(dev);
    if (ret < 0) {
        LOG_ERR("Reset failed (%d)", ret);
        return ret;
    }

    ret = sh8601_configure(dev);
    if (ret < 0) {
        LOG_ERR("Panel configuration failed (%d)", ret);
        return ret;
    }

    /* Leave the panel blanked; users call display_blanking_off() */
    return 0;
}

static DEVICE_API(display, sh8601_api) = {
    .blanking_on      = sh8601_blanking_on,
    .blanking_off     = sh8601_blanking_off,
    .write            = sh8601_write,
    .set_brightness   = sh8601_set_brightness,
    .get_capabilities = sh8601_get_capabilities,
    .set_pixel_format = sh8601_set_pixel_format,
    .set_orientation  = sh8601_set_orientation,
};

#define SH8601_CONFIG_COMMON(inst, _cmd_write, _pixel_write)                            \
    .cmd_write = _cmd_write, .pixel_write = _pixel_write,                               \
    .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, { 0 }),                   \
    .width = DT_INST_PROP(inst, width), .height = DT_INST_PROP(inst, height),           \
    .x_offset = DT_INST_PROP(inst, x_offset), .y_offset = DT_INST_PROP(inst, y_offset), \
    .red_blue_swap = DT_INST_PROP(inst, red_blue_swap)

#define SH8601_CONFIG_SPI(inst)                                                      \
    {                                                                                \
        SH8601_CONFIG_COMMON(inst, sh8601_spi_cmd_write, sh8601_spi_cmd_write),      \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8)), \
    }

#define SH8601_CONFIG_MSPI(inst)                                                         \
    {                                                                                    \
        SH8601_CONFIG_COMMON(inst, sh8601_mspi_cmd_write, sh8601_mspi_pixel_write),      \
        .mspi_bus        = DEVICE_DT_GET(DT_INST_BUS(inst)),                             \
        .dev_id          = MSPI_DEVICE_ID_DT_INST(inst),                                 \
        .pixel_io_mode   = DT_INST_ENUM_IDX_OR(inst, mspi_io_mode, MSPI_IO_MODE_SINGLE), \
        .sw_multi_periph = DT_PROP(DT_INST_BUS(inst), software_multiperipheral),         \
    }

#define SH8601_DEFINE(inst)                                                                        \
    static const struct sh8601_config sh8601_config_##inst =                                       \
        COND_CASE_1(DT_INST_ON_BUS(inst, mspi), (SH8601_CONFIG_MSPI(inst)),                        \
                    DT_INST_ON_BUS(inst, spi), (SH8601_CONFIG_SPI(inst)), ());                     \
                                                                                                   \
    static struct sh8601_data sh8601_data_##inst = {                                               \
        COND_CODE_1(DT_INST_ON_BUS(inst, mspi),                                                    \
                    (.mspi_cfg = MSPI_DEVICE_CONFIG_DT_INST(inst), .pixel_xfer_mode = MSPI_DMA, ), \
                    ())                                                                            \
            .pixel_format = DT_INST_PROP(inst, pixel_format),                                      \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, sh8601_init, NULL, &sh8601_data_##inst, &sh8601_config_##inst,     \
                          POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &sh8601_api);

DT_INST_FOREACH_STATUS_OKAY(SH8601_DEFINE)
