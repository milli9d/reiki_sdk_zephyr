/*
 * Copyright (c) 2026 Milind Singh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_mspi

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <hal/spi_hal.h>
#include <hal/spi_ll.h>
#include <esp_clk_tree.h>
#include <esp_memory_utils.h>
#include <esp_attr.h>

#if CONFIG_ESP32_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP && SOC_SPI_SUPPORT_SLEEP_RETENTION
#define MSPI_ESP32_SLEEP_RETENTION 1
#include <soc/spi_periph.h>
#include <esp_private/sleep_retention.h>
#else
#define MSPI_ESP32_SLEEP_RETENTION 0
#endif

#if defined(CONFIG_MSPI_DMA) && defined(SOC_GDMA_SUPPORTED)
#define MSPI_ESP32_DMA 1
#include <hal/gdma_channel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#else
#define MSPI_ESP32_DMA 0
#endif

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#ifdef CONFIG_MSPI_ESP32_INTERRUPT
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#endif
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mspi_esp32, CONFIG_MSPI_LOG_LEVEL);

#ifdef SPI_LL_SRC_PRE_DIV_MAX
#error "Clock source pre-divider (ESP32-C5/P4 class SoCs) is not supported yet"
#endif

/* Without DMA, data goes through the 16-word data buffer */
#define MSPI_ESP32_PIO_MAX_BYTES (SPI_LL_CPU_MAX_BIT_LEN / 8)
/* With DMA, one transaction is bounded by the data bit length register */
#define MSPI_ESP32_DMA_MAX_BYTES (SPI_LL_DMA_MAX_BIT_LEN / 8)

#define MSPI_ESP32_MAX_CMD_BYTES  2
#define MSPI_ESP32_MAX_ADDR_BYTES 4
#define MSPI_ESP32_MAX_DUMMY      256

/* Hardware CS lines: SPI2 has CS0..CS5, SPI3 has CS0..CS2 */
#define MSPI_ESP32_HW_CS_NUM(host) ((host) == SPI2_HOST ? 6 : 3)

struct mspi_esp32_config {
    spi_dev_t*                       spi;
    const struct device*             clock_dev;
    clock_control_subsys_t           clock_subsys;
    const struct pinctrl_dev_config* pcfg;
    struct mspi_cfg                  mspicfg;
    spi_host_device_t                host;
    spi_clock_source_t               clock_source;
    bool                             use_iomux;
    uint8_t                          cs_setup;
    uint8_t                          cs_hold;
#ifdef CONFIG_MSPI_ESP32_INTERRUPT
    int irq_source;
    int irq_priority;
    int irq_flags;
#endif
#if MSPI_ESP32_DMA
    const struct device* dma_dev;
    uint8_t              dma_tx_ch;
    uint8_t              dma_rx_ch;
#endif
};

struct mspi_esp32_data {
    spi_hal_context_t         hal;
    spi_hal_dev_config_t      hal_dev;
    spi_line_mode_t           lines;
    struct mspi_dev_cfg       dev_cfg;
    const struct mspi_dev_id* dev_id;
    /* Bus ownership: taken by dev_config, given by get_channel_status */
    struct k_sem lock;
    /* A transceive is in progress */
    volatile bool busy;
#ifdef CONFIG_MSPI_ESP32_INTERRUPT
    /* Signalled by the transaction done interrupt */
    struct k_sem done;
#endif
    uint32_t clock_source_hz;
};

/**
 * @brief Map an MSPI IO mode to GPSPI per-phase line counts.
 *
 * @param[in]   io_mode     MSPI IO mode
 * @param[out]  lines       command, address and data line counts
 *
 * @retval 0          success
 * @retval -ENOTSUP   IO mode has no GPSPI equivalent
 */
static int mspi_esp32_io_mode_to_lines(enum mspi_io_mode io_mode, spi_line_mode_t* lines)
{
    switch (io_mode) {
        case MSPI_IO_MODE_SINGLE:
            *lines = (spi_line_mode_t){ 1, 1, 1 };
            break;
        case MSPI_IO_MODE_DUAL:
            *lines = (spi_line_mode_t){ 2, 2, 2 };
            break;
        case MSPI_IO_MODE_DUAL_1_1_2:
            *lines = (spi_line_mode_t){ 1, 1, 2 };
            break;
        case MSPI_IO_MODE_DUAL_1_2_2:
            *lines = (spi_line_mode_t){ 1, 2, 2 };
            break;
        case MSPI_IO_MODE_QUAD:
            *lines = (spi_line_mode_t){ 4, 4, 4 };
            break;
        case MSPI_IO_MODE_QUAD_1_1_4:
            *lines = (spi_line_mode_t){ 1, 1, 4 };
            break;
        case MSPI_IO_MODE_QUAD_1_4_4:
            *lines = (spi_line_mode_t){ 1, 4, 4 };
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

/**
 * @brief Compute the GPSPI clock dividers for a bus frequency.
 *
 * The result is stored in the HAL device timing config and applied by the
 * next spi_hal_setup_device().
 *
 * @param[in]   dev         controller device
 * @param[in]   freq        requested SCLK frequency in Hz
 *
 * @retval 0          success
 * @retval -ENOTSUP   frequency out of range or not achievable
 */
static int mspi_esp32_set_freq(const struct device* dev, uint32_t freq)
{
    const struct mspi_esp32_config* cfg    = dev->config;
    struct mspi_esp32_data*         data   = dev->data;
    spi_hal_timing_param_t          timing = {
        .clk_src_hz     = data->clock_source_hz,
        .half_duplex    = true,
        .no_compensate  = false,
        .expected_freq  = freq,
        .duty_cycle     = 128,
        .input_delay_ns = 0,
        .use_gpio       = !cfg->use_iomux,
    };

    if (freq == 0 || (cfg->mspicfg.max_freq != 0 && freq > cfg->mspicfg.max_freq)) {
        LOG_ERR("Unsupported frequency %u", freq);
        return -ENOTSUP;
    }

    if (spi_hal_cal_clock_conf(&timing, &data->hal_dev.timing_conf) != ESP_OK) {
        LOG_ERR("Frequency %u not achievable", freq);
        return -ENOTSUP;
    }

    LOG_DBG("Requested %u Hz, got %d Hz", freq, data->hal_dev.timing_conf.real_freq);
    return 0;
}

/**
 * @brief Check the fields selected by @p mask against controller limits.
 *
 * @param[in]   dev         controller device
 * @param[in]   mask        fields of @p cfg to check
 * @param[in]   cfg         requested device config
 *
 * @retval 0          all selected fields are supported
 * @retval -EINVAL    CE number beyond the host's hardware chip selects
 * @retval -ENOTSUP   IO mode, data rate, endianness, DQS, command, address or
 *                    dummy length not supported
 */
static int mspi_esp32_check_dev_cfg(const struct device* dev, enum mspi_dev_cfg_mask mask,
                                    const struct mspi_dev_cfg* cfg)
{
    const struct mspi_esp32_config* config = dev->config;
    spi_line_mode_t                 lines;

    if ((mask & MSPI_DEVICE_CONFIG_IO_MODE) &&
        mspi_esp32_io_mode_to_lines(cfg->io_mode, &lines) < 0) {
        LOG_ERR("Unsupported IO mode %d", cfg->io_mode);
        return -ENOTSUP;
    }

    if ((mask & MSPI_DEVICE_CONFIG_DATA_RATE) && cfg->data_rate != MSPI_DATA_RATE_SINGLE) {
        LOG_ERR("Only single data rate is supported");
        return -ENOTSUP;
    }

    if ((mask & MSPI_DEVICE_CONFIG_ENDIAN) && cfg->endian != MSPI_XFER_LITTLE_ENDIAN) {
        LOG_ERR("Only little endian transfers are supported");
        return -ENOTSUP;
    }

    if ((mask & MSPI_DEVICE_CONFIG_DQS) && cfg->dqs_enable) {
        LOG_ERR("DQS is not supported");
        return -ENOTSUP;
    }

    if ((mask & MSPI_DEVICE_CONFIG_CE_NUM) && cfg->ce_num >= MSPI_ESP32_HW_CS_NUM(config->host)) {
        LOG_ERR("Invalid CE number %u", cfg->ce_num);
        return -EINVAL;
    }

    if ((mask & MSPI_DEVICE_CONFIG_CMD_LEN) && cfg->cmd_length > MSPI_ESP32_MAX_CMD_BYTES) {
        LOG_ERR("Command length %u too long", cfg->cmd_length);
        return -ENOTSUP;
    }

    if ((mask & MSPI_DEVICE_CONFIG_ADDR_LEN) && cfg->addr_length > MSPI_ESP32_MAX_ADDR_BYTES) {
        LOG_ERR("Address length %u too long", cfg->addr_length);
        return -ENOTSUP;
    }

    if (((mask & MSPI_DEVICE_CONFIG_RX_DUMMY) && cfg->rx_dummy > MSPI_ESP32_MAX_DUMMY) ||
        ((mask & MSPI_DEVICE_CONFIG_TX_DUMMY) && cfg->tx_dummy > MSPI_ESP32_MAX_DUMMY)) {
        LOG_ERR("Too many dummy cycles");
        return -ENOTSUP;
    }

    return 0;
}

/**
 * @brief MSPI API: acquire the bus for a device and apply its config.
 *
 * Switching to a different device takes the bus lock, which is held until
 * mspi_get_channel_status() reports the controller idle. With
 * MSPI_DEVICE_CONFIG_NONE only the bus is acquired.
 *
 * @param[in]   dev         controller device
 * @param[in]   dev_id      peripheral device ID
 * @param[in]   param_mask  fields of @p cfg to apply
 * @param[in]   cfg         device config, may be NULL with MSPI_DEVICE_CONFIG_NONE
 *
 * @retval 0          success, bus held by @p dev_id
 * @retval -EBUSY     bus held by another device past the timeout
 * @retval -EINVAL    invalid config
 * @retval -ENOTSUP   unsupported config
 */
static int mspi_esp32_dev_config(const struct device* dev, const struct mspi_dev_id* dev_id,
                                 const enum mspi_dev_cfg_mask param_mask,
                                 const struct mspi_dev_cfg*   cfg)
{
    const struct mspi_esp32_config* config    = dev->config;
    struct mspi_esp32_data*         data      = dev->data;
    struct mspi_dev_cfg*            cur       = &data->dev_cfg;
    bool                            switching = (data->dev_id != dev_id);
    int                             ret;

    if (param_mask != MSPI_DEVICE_CONFIG_NONE) {
        if (cfg == NULL) {
            return -EINVAL;
        }
        ret = mspi_esp32_check_dev_cfg(dev, param_mask, cfg);
        if (ret < 0) {
            return ret;
        }
    }

    if (switching) {
        if (k_sem_take(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE))) {
            return -EBUSY;
        }
        data->dev_id = dev_id;

        if (dev_id->ce.port != NULL) {
            ret = gpio_pin_configure_dt(&dev_id->ce, GPIO_OUTPUT_INACTIVE);
            if (ret < 0) {
                goto release;
            }
        }
    }

    if (param_mask == MSPI_DEVICE_CONFIG_NONE) {
        /* Just acquiring the bus */
        return 0;
    }

    if (param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
        ret = mspi_esp32_set_freq(dev, cfg->freq);
        if (ret < 0) {
            goto release;
        }
        cur->freq = cfg->freq;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_IO_MODE) {
        cur->io_mode = cfg->io_mode;
        (void)mspi_esp32_io_mode_to_lines(cur->io_mode, &data->lines);
    }
    if (param_mask & MSPI_DEVICE_CONFIG_CPP) {
        cur->cpp = cfg->cpp;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_CE_NUM) {
        cur->ce_num = cfg->ce_num;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_CE_POL) {
        cur->ce_polarity = cfg->ce_polarity;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) {
        cur->data_rate = cfg->data_rate;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_ENDIAN) {
        cur->endian = cfg->endian;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_DQS) {
        cur->dqs_enable = cfg->dqs_enable;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_RX_DUMMY) {
        cur->rx_dummy = cfg->rx_dummy;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_TX_DUMMY) {
        cur->tx_dummy = cfg->tx_dummy;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_READ_CMD) {
        cur->read_cmd = cfg->read_cmd;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_WRITE_CMD) {
        cur->write_cmd = cfg->write_cmd;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_CMD_LEN) {
        cur->cmd_length = cfg->cmd_length;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) {
        cur->addr_length = cfg->addr_length;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_MEM_BOUND) {
        cur->mem_boundary = cfg->mem_boundary;
    }
    if (param_mask & MSPI_DEVICE_CONFIG_BREAK_TIME) {
        cur->time_to_break = cfg->time_to_break;
    }

    data->hal_dev.mode        = cur->cpp;
    data->hal_dev.positive_cs = (cur->ce_polarity == MSPI_CE_ACTIVE_HIGH);
    data->hal_dev.cs_pin_id   = (dev_id->ce.port != NULL) ? -1 : cur->ce_num;
    data->hal_dev.cs_setup    = config->cs_setup;
    data->hal_dev.cs_hold     = config->cs_hold;
    spi_hal_setup_device(&data->hal, &data->hal_dev);

    return 0;

release:
    if (switching) {
        data->dev_id = NULL;
        k_sem_give(&data->lock);
    }
    return ret;
}

#if MSPI_ESP32_DMA
/**
 * @brief Decide whether a packet can go through GDMA.
 *
 * Requires a DMA controller, a packet longer than the PIO FIFO and a DMA
 * capable buffer; RX buffers must also be word aligned and sized.
 *
 * @param[in]   dev         controller device
 * @param[in]   pkt         packet to transfer
 *
 * @return true if the packet can use DMA
 */
static bool mspi_esp32_dma_usable(const struct device* dev, const struct mspi_xfer_packet* pkt)
{
    const struct mspi_esp32_config* cfg = dev->config;
    const void*                     buf = pkt->data_buf;
    bool                            capable;

    if (cfg->dma_dev == NULL || pkt->num_bytes <= MSPI_ESP32_PIO_MAX_BYTES) {
        return false;
    }

    capable = esp_ptr_dma_capable(buf);
#ifdef CONFIG_ESP_SPIRAM
    capable = capable || esp_ptr_dma_ext_capable(buf);
#endif
    if (pkt->dir == MSPI_RX) {
        /* GDMA writes whole words */
        capable = capable && IS_ALIGNED(buf, 4) && (pkt->num_bytes % 4U) == 0U;
    }

    return capable;
}

/**
 * @brief Configure and start the GDMA channel for one hardware transaction.
 *
 * Syncs the cache over the buffer, resets the peripheral FIFO and enables
 * the GPSPI DMA path in the transfer direction.
 *
 * @param[in]   dev         controller device
 * @param[in]   trans       HAL transaction holding the send or receive buffer
 * @param[in]   len         number of bytes
 *
 * @return 0 on success, negative errno from the DMA driver otherwise
 */
static int mspi_esp32_dma_start(const struct device* dev, const spi_hal_trans_config_t* trans,
                                uint32_t len)
{
    const struct mspi_esp32_config* cfg  = dev->config;
    struct mspi_esp32_data*         data = dev->data;
    spi_dev_t*                      hw   = data->hal.hw;
    bool                            rx   = (trans->rcv_buffer != NULL);
    uint8_t                         ch   = rx ? cfg->dma_rx_ch : cfg->dma_tx_ch;
    struct dma_block_config         blk  = {
        .block_size = len,
    };
    struct dma_config dma_cfg = {
        .channel_direction = rx ? PERIPHERAL_TO_MEMORY : MEMORY_TO_PERIPHERAL,
        .dma_slot          = SOC_GDMA_TRIG_PERIPH_SPI2 + (cfg->host - SPI2_HOST),
        .block_count       = 1,
        .head_block        = &blk,
    };
    int ret;

    if (rx) {
        blk.dest_address = (uint32_t)trans->rcv_buffer;
        sys_cache_data_flush_and_invd_range(trans->rcv_buffer, len);
    } else {
        blk.source_address = (uint32_t)trans->send_buffer;
        sys_cache_data_flush_range(trans->send_buffer, len);
    }

    ret = dma_config(cfg->dma_dev, ch, &dma_cfg);
    if (ret < 0) {
        LOG_ERR("DMA config failed (%d)", ret);
        return ret;
    }

    if (rx) {
        spi_ll_dma_rx_fifo_reset(hw);
        spi_ll_infifo_full_clr(hw);
        spi_ll_dma_rx_enable(hw, 1);
    } else {
        spi_ll_dma_tx_fifo_reset(hw);
        spi_ll_outfifo_empty_clr(hw);
        spi_ll_dma_tx_enable(hw, 1);
    }

    return dma_start(cfg->dma_dev, ch);
}
#endif /* MSPI_ESP32_DMA */

/**
 * @brief Run one GPSPI hardware transaction and wait for it to finish.
 *
 * DMA transactions sleep on the done interrupt when
 * CONFIG_MSPI_ESP32_INTERRUPT is set; PIO transactions are polled.
 *
 * @param[in]   dev         controller device
 * @param[in,out] trans     HAL transaction (phases, buffers, line modes)
 * @param[in]   len         number of data bytes
 * @param[in]   use_dma     move data with GDMA instead of the CPU FIFO
 * @param[in]   timeout_ms  completion timeout in milliseconds
 *
 * @retval 0          success
 * @retval -ETIMEDOUT transaction did not complete in time
 * @return other negative errno from the DMA driver
 */
static int mspi_esp32_hw_xfer(const struct device* dev, spi_hal_trans_config_t* trans, uint32_t len,
                              bool use_dma, uint32_t timeout_ms)
{
    struct mspi_esp32_data* data = dev->data;
    spi_hal_context_t*      hal  = &data->hal;
    int                     ret  = 0;

    spi_hal_setup_trans(hal, &data->hal_dev, trans);

#if MSPI_ESP32_DMA
    if (use_dma && len > 0) {
        ret = mspi_esp32_dma_start(dev, trans, len);
        if (ret < 0) {
            return ret;
        }
    } else
#endif
    {
        ARG_UNUSED(use_dma);
        spi_ll_dma_tx_enable(hal->hw, 0);
        spi_ll_dma_rx_enable(hal->hw, 0);
        spi_hal_push_tx_buffer(hal, trans);
    }

    spi_hal_enable_data_line(hal->hw, trans->send_buffer != NULL, trans->rcv_buffer != NULL);

#ifdef CONFIG_MSPI_ESP32_INTERRUPT
    /*
     * Only long (DMA) transactions sleep on the interrupt; short PIO ones
     * finish faster than an interrupt round trip and are polled.
     */
    if (use_dma) {
        k_sem_reset(&data->done);
        spi_ll_clear_int_stat(hal->hw);
        spi_ll_enable_int(hal->hw);
        spi_hal_user_start(hal);

        if (k_sem_take(&data->done, K_MSEC(timeout_ms)) != 0) {
            spi_ll_disable_int(hal->hw);
            LOG_ERR("Transaction timed out");
            ret = -ETIMEDOUT;
        }
    } else
#endif
    {
        spi_hal_user_start(hal);

        if (!WAIT_FOR(spi_hal_usr_is_done(hal), (uint32_t)timeout_ms * USEC_PER_MSEC, k_yield())) {
            LOG_ERR("Transaction timed out");
            ret = -ETIMEDOUT;
        }
    }

#if MSPI_ESP32_DMA
    if (use_dma && len > 0) {
        const struct mspi_esp32_config* cfg = dev->config;

        if (trans->rcv_buffer != NULL) {
            dma_stop(cfg->dma_dev, cfg->dma_rx_ch);
            sys_cache_data_invd_range(trans->rcv_buffer, len);
        } else {
            dma_stop(cfg->dma_dev, cfg->dma_tx_ch);
        }
        return ret;
    }
#endif

    if (ret == 0) {
        spi_hal_fetch_result(hal);
    }

    return ret;
}

/**
 * @brief Transfer one MSPI packet as one or more hardware transactions.
 *
 * The first transaction carries the command, address and dummy phases; data
 * beyond the PIO or DMA limit follows in data-only transactions with chip
 * select held active. Drives the CE GPIO when the device uses one.
 *
 * @param[in]   dev         controller device
 * @param[in]   xfer        transfer the packet belongs to
 * @param[in]   pkt         packet to send or receive
 * @param[in]   hold_ce     keep chip select active after the packet
 *
 * @retval 0          success
 * @retval -EINVAL    non-empty packet without a buffer
 * @return other negative errno from the transaction
 */
static int mspi_esp32_xfer_packet(const struct device* dev, const struct mspi_xfer* xfer,
                                  const struct mspi_xfer_packet* pkt, bool hold_ce)
{
    struct mspi_esp32_data*    data      = dev->data;
    const struct gpio_dt_spec* ce        = &data->dev_id->ce;
    uint8_t*                   buf       = pkt->data_buf;
    uint32_t                   remaining = pkt->num_bytes;
    uint32_t                   max_chunk = MSPI_ESP32_PIO_MAX_BYTES;
    bool                       use_dma   = false;
    spi_hal_trans_config_t     trans     = {
        .cmd        = (uint16_t)pkt->cmd,
        .cmd_bits   = xfer->cmd_length * 8,
        .addr       = pkt->address,
        .addr_bits  = xfer->addr_length * 8,
        .dummy_bits = (pkt->dir == MSPI_RX) ? xfer->rx_dummy : xfer->tx_dummy,
        .line_mode  = data->lines,
    };
    int ret = 0;

    if (remaining > 0 && buf == NULL) {
        return -EINVAL;
    }

#if MSPI_ESP32_DMA
    if (xfer->xfer_mode == MSPI_DMA && mspi_esp32_dma_usable(dev, pkt)) {
        use_dma   = true;
        max_chunk = MSPI_ESP32_DMA_MAX_BYTES;
    }
#endif

    if (ce->port != NULL) {
        gpio_pin_set_dt(ce, 1);
    }

    /*
     * The first hardware transaction carries the command, address and dummy
     * phases; data that does not fit follows in data-only transactions with
     * CS held active in between.
     */
    do {
        uint32_t chunk = MIN(remaining, max_chunk);

        trans.send_buffer = NULL;
        trans.rcv_buffer  = NULL;
        trans.tx_bitlen   = 0;
        trans.rx_bitlen   = 0;
        if (chunk > 0) {
            if (pkt->dir == MSPI_TX) {
                trans.send_buffer = buf;
                trans.tx_bitlen   = chunk * 8;
            } else {
                trans.rcv_buffer = buf;
                trans.rx_bitlen  = chunk * 8;
            }
        }
        remaining -= chunk;
        buf += chunk;
        trans.cs_keep_active = (remaining > 0) || hold_ce;

        ret = mspi_esp32_hw_xfer(dev, &trans, chunk, use_dma, xfer->timeout);
        if (ret < 0) {
            break;
        }

        trans.cmd_bits   = 0;
        trans.addr_bits  = 0;
        trans.dummy_bits = 0;
    }
    while (remaining > 0);

    if (ce->port != NULL && (!hold_ce || ret < 0)) {
        gpio_pin_set_dt(ce, 0);
    }

    return ret;
}

/**
 * @brief MSPI API: run a synchronous transfer on the acquired bus.
 *
 * @param[in]   dev         controller device
 * @param[in]   dev_id      device that acquired the bus with dev_config
 * @param[in]   xfer        transfer description
 *
 * @retval 0          success
 * @retval -ESTALE    bus not acquired by @p dev_id
 * @retval -EINVAL    no packets
 * @retval -ENOTSUP   async transfer, DMA without a DMA controller, or phase
 *                    lengths beyond the hardware limits
 * @return other negative errno from the transaction
 */
static int mspi_esp32_transceive(const struct device* dev, const struct mspi_dev_id* dev_id,
                                 const struct mspi_xfer* xfer)
{
    struct mspi_esp32_data* data = dev->data;
    int                     ret  = 0;

    if (dev_id != data->dev_id) {
        LOG_ERR("Bus not acquired by this device");
        return -ESTALE;
    }

    if (xfer->async) {
        return -ENOTSUP;
    }

    switch (xfer->xfer_mode) {
        case MSPI_PIO:
            break;
        case MSPI_DMA:
#if MSPI_ESP32_DMA
            if (((const struct mspi_esp32_config*)dev->config)->dma_dev != NULL) {
                break;
            }
#endif
            return -ENOTSUP;
        default:
            return -ENOTSUP;
    }

    if (xfer->packets == NULL || xfer->num_packet == 0) {
        return -EINVAL;
    }

    if (xfer->cmd_length > MSPI_ESP32_MAX_CMD_BYTES ||
        xfer->addr_length > MSPI_ESP32_MAX_ADDR_BYTES || xfer->tx_dummy > MSPI_ESP32_MAX_DUMMY ||
        xfer->rx_dummy > MSPI_ESP32_MAX_DUMMY) {
        return -ENOTSUP;
    }

    data->busy = true;
    /* Keep the SoC out of light sleep while the peripheral is clocking */
    if (IS_ENABLED(CONFIG_PM)) {
        pm_policy_state_all_lock_get();
    }

    for (uint32_t i = 0; i < xfer->num_packet; i++) {
        bool last = (i == xfer->num_packet - 1U);

        ret = mspi_esp32_xfer_packet(dev, xfer, &xfer->packets[i], last && xfer->hold_ce);
        if (ret < 0) {
            break;
        }
    }

    if (IS_ENABLED(CONFIG_PM)) {
        pm_policy_state_all_lock_put();
    }
    data->busy = false;

    return ret;
}

/**
 * @brief MSPI API: report whether the controller is busy, releasing the bus if idle.
 *
 * @param[in]   dev         controller device
 * @param[in]   ch          channel, unused (single channel)
 *
 * @retval 0          idle; the bus lock is released
 * @retval -EBUSY     a transfer is in progress
 */
static int mspi_esp32_get_channel_status(const struct device* dev, uint8_t ch)
{
    struct mspi_esp32_data* data = dev->data;

    ARG_UNUSED(ch);

    if (data->busy) {
        return -EBUSY;
    }

    if (data->dev_id != NULL) {
        data->dev_id = NULL;
        k_sem_give(&data->lock);
    }

    return 0;
}

#ifdef CONFIG_MSPI_ESP32_INTERRUPT
/**
 * @brief Transaction done interrupt: mask it and wake the waiting thread.
 *
 * @param[in]   arg         controller device
 */
static void IRAM_ATTR mspi_esp32_isr(void* arg)
{
    const struct device*    dev  = arg;
    struct mspi_esp32_data* data = dev->data;

    /*
     * Mask only: on GPSPI the raw trans_done bit is also the "user
     * transaction done" status, so it must stay set until the next
     * spi_hal_setup_trans() clears it.
     */
    spi_ll_disable_int(data->hal.hw);
    k_sem_give(&data->done);
}
#endif

/**
 * @brief MSPI API: validate the controller config and bring up the peripheral.
 *
 * Enables the clock, initializes the HAL, reads the clock source frequency
 * and applies the default pinctrl state.
 *
 * @param[in]   spec        controller device and config
 *
 * @retval 0          success
 * @retval -ENOTSUP   not controller mode, not half-duplex, or DQS requested
 * @retval -ENODEV    clock or DMA device not ready
 * @retval -EIO       clock source frequency unknown
 * @return other negative errno from clock control or pinctrl
 */
static int mspi_esp32_config(const struct mspi_dt_spec* spec)
{
    const struct device*            dev  = spec->bus;
    const struct mspi_esp32_config* cfg  = dev->config;
    struct mspi_esp32_data*         data = dev->data;
    const struct mspi_cfg*          mcfg = &spec->config;
    int                             ret;

    if (mcfg->op_mode != MSPI_OP_MODE_CONTROLLER) {
        LOG_ERR("Only controller mode is supported");
        return -ENOTSUP;
    }

    if (mcfg->duplex != MSPI_HALF_DUPLEX) {
        LOG_ERR("Only half-duplex is supported");
        return -ENOTSUP;
    }

    if (mcfg->dqs_support) {
        LOG_ERR("DQS is not supported");
        return -ENOTSUP;
    }

    if (!device_is_ready(cfg->clock_dev)) {
        return -ENODEV;
    }

#if MSPI_ESP32_DMA
    if (cfg->dma_dev != NULL && !device_is_ready(cfg->dma_dev)) {
        LOG_ERR("DMA device not ready");
        return -ENODEV;
    }
#endif

    ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
    if (ret < 0) {
        LOG_ERR("Failed to enable clock (%d)", ret);
        return ret;
    }

    spi_hal_init(&data->hal, cfg->host);
    spi_ll_enable_clock(cfg->host, true);
    spi_ll_set_clk_source(cfg->spi, cfg->clock_source);
    data->hal.dma_enabled = false;

    ret = esp_clk_tree_src_get_freq_hz(cfg->clock_source, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX,
                                       &data->clock_source_hz);
    if (ret) {
        LOG_ERR("Could not get clock source frequency (%d)", ret);
        return -EIO;
    }

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Failed to apply pinctrl (%d)", ret);
        return ret;
    }

    data->hal_dev.half_duplex = 1;
    data->hal_dev.cs_pin_id   = -1;
    data->dev_cfg.io_mode     = MSPI_IO_MODE_SINGLE;
    (void)mspi_esp32_io_mode_to_lines(MSPI_IO_MODE_SINGLE, &data->lines);

    return 0;
}

#if MSPI_ESP32_SLEEP_RETENTION
/**
 * @brief Sleep retention callback: create the register backup entries for this host.
 *
 * @param[in]   arg         controller device
 *
 * @return ESP_OK on success, ESP error code otherwise
 */
static esp_err_t mspi_esp32_retention_create_cb(void* arg)
{
    const struct device*            dev = arg;
    const struct mspi_esp32_config* cfg = dev->config;
    unsigned int                    idx = cfg->host - SPI2_HOST;

    return sleep_retention_entries_create(
        spi_reg_retention_info[idx].entry_array, spi_reg_retention_info[idx].array_size,
        REGDMA_LINK_PRI_GPSPI, spi_reg_retention_info[idx].module_id);
}

/**
 * @brief Save and restore GPSPI registers when the peripheral domain powers down.
 *
 * Failures are logged and otherwise ignored.
 *
 * @param[in]   dev         controller device
 */
static void mspi_esp32_sleep_retention_init(const struct device* dev)
{
    const struct mspi_esp32_config* cfg = dev->config;
    sleep_retention_module_t module     = spi_reg_retention_info[cfg->host - SPI2_HOST].module_id;
    sleep_retention_module_init_param_t init_param = {
        .cbs       = { .create = { .handle = mspi_esp32_retention_create_cb, .arg = (void*)dev } },
        .attribute = SLEEP_RETENTION_MODULE_ATTR_ATTACH,
        .depends   = RETENTION_MODULE_BITMAP_INIT(CLOCK_SYSTEM),
    };
    esp_err_t err = sleep_retention_module_init(module, &init_param);

    if (err == ESP_OK) {
        err = sleep_retention_module_allocate(module);
    }
    if (err == ESP_OK) {
        err = sleep_retention_module_attach(module);
    }
    if (err != ESP_OK) {
        LOG_WRN("Sleep retention init failed (%d)", err);
    }
}
#endif /* MSPI_ESP32_SLEEP_RETENTION */

/**
 * @brief Initialize the controller: lock, peripheral, interrupt and sleep retention.
 *
 * @param[in]   dev         controller device
 *
 * @retval 0          success
 * @retval -EIO       interrupt allocation failed
 * @return other negative errno from mspi_esp32_config()
 */
static int mspi_esp32_init(const struct device* dev)
{
    const struct mspi_esp32_config* cfg  = dev->config;
    struct mspi_esp32_data*         data = dev->data;
    const struct mspi_dt_spec       spec = {
        .bus    = dev,
        .config = cfg->mspicfg,
    };

    int ret;

    k_sem_init(&data->lock, 1, 1);

    ret = mspi_esp32_config(&spec);
    if (ret < 0) {
        return ret;
    }

#ifdef CONFIG_MSPI_ESP32_INTERRUPT
    k_sem_init(&data->done, 0, 1);

    /*
     * spi_hal_init() leaves the done interrupt enabled and pending; mask it
     * but keep the raw bit, which doubles as the idle status.
     */
    spi_ll_disable_int(cfg->spi);

    ret = esp_intr_alloc(cfg->irq_source,
                         ESP_PRIO_TO_FLAGS(cfg->irq_priority) |
                             ESP_INT_FLAGS_CHECK(cfg->irq_flags) | ESP_INTR_FLAG_IRAM,
                         (intr_handler_t)mspi_esp32_isr, (void*)dev, NULL);
    if (ret != 0) {
        LOG_ERR("Could not allocate interrupt (%d)", ret);
        return -EIO;
    }
#endif

#if MSPI_ESP32_SLEEP_RETENTION
    mspi_esp32_sleep_retention_init(dev);
#endif

    return 0;
}

static DEVICE_API(mspi, mspi_esp32_api) = {
    .config             = mspi_esp32_config,
    .dev_config         = mspi_esp32_dev_config,
    .get_channel_status = mspi_esp32_get_channel_status,
    .transceive         = mspi_esp32_transceive,
};

#define MSPI_ESP32_DMA_CFG(n)                                                                      \
    .dma_dev = ESP32_DT_INST_DMA_CTLR(n, tx), .dma_tx_ch = ESP32_DT_INST_DMA_CELL(n, tx, channel), \
    .dma_rx_ch = ESP32_DT_INST_DMA_CELL(n, rx, channel),

#define MSPI_ESP32_IRQ_CFG(n)                           \
    .irq_source   = DT_INST_IRQ_BY_IDX(n, 0, irq),      \
    .irq_priority = DT_INST_IRQ_BY_IDX(n, 0, priority), \
    .irq_flags    = DT_INST_IRQ_BY_IDX(n, 0, flags),

#define MSPI_ESP32_CHECK_DMA(n)                                                                  \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                                                  \
                (BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),         \
                              "espressif,esp32-mspi: the DMA controller in dmas is disabled");), \
                ())

#define MSPI_ESP32_INIT(n)                                                                        \
    MSPI_ESP32_CHECK_DMA(n)                                                                       \
    PINCTRL_DT_INST_DEFINE(n);                                                                    \
                                                                                                  \
    static const struct mspi_esp32_config mspi_esp32_config_##n = {                            \
		.spi = (spi_dev_t *)DT_INST_REG_ADDR(n),                                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, offset),            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.mspicfg =                                                                         \
			{                                                                          \
				.op_mode = DT_INST_ENUM_IDX_OR(n, op_mode,                         \
							       MSPI_OP_MODE_CONTROLLER),           \
				.duplex = DT_INST_ENUM_IDX_OR(n, duplex, MSPI_HALF_DUPLEX),        \
				.dqs_support = DT_INST_PROP(n, dqs_support),                       \
				.max_freq = DT_INST_PROP_OR(n, clock_frequency, 0),                \
				.num_periph = DT_INST_CHILD_NUM(n),                                \
				.sw_multi_periph = DT_INST_PROP(n, software_multiperipheral),      \
			},                                                                         \
		.host = (spi_host_device_t)(DT_INST_PROP(n, dma_host) + 1),                        \
		.clock_source = SPI_CLK_SRC_DEFAULT,                                               \
		.use_iomux = DT_INST_PROP(n, use_iomux),                                           \
		.cs_setup = DT_INST_PROP(n, cs_setup_time),                                        \
		.cs_hold = DT_INST_PROP(n, cs_hold_time),                                          \
		IF_ENABLED(MSPI_ESP32_DMA, (MSPI_ESP32_DMA_CFG(n)))                                \
		IF_ENABLED(CONFIG_MSPI_ESP32_INTERRUPT, (MSPI_ESP32_IRQ_CFG(n)))                   \
	};  \
                                                                                                  \
    static struct mspi_esp32_data mspi_esp32_data_##n = {                                         \
        .hal = { .hw = (spi_dev_t*)DT_INST_REG_ADDR(n) },                                         \
    };                                                                                            \
                                                                                                  \
    DEVICE_DT_INST_DEFINE(n, mspi_esp32_init, NULL, &mspi_esp32_data_##n, &mspi_esp32_config_##n, \
                          POST_KERNEL, CONFIG_MSPI_INIT_PRIORITY, &mspi_esp32_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_ESP32_INIT)
