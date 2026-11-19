/*
 * Copyright (c) 2026 Milind Singh
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Full-screen write benchmark for the chosen display: reports frame time,
 * bus throughput and how much CPU a lowest-priority thread got meanwhile.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define BAND_ROWS 56
#define FRAMES    50

/* Width is checked against the panel at runtime */
#define MAX_WIDTH 368
#define BPP       2

static uint8_t band[MAX_WIDTH * BAND_ROWS * BPP] __aligned(4);

static volatile uint32_t spin_count;

/**
 * @brief Lowest-priority busy loop; its count rate measures free CPU.
 *
 * @param[in]   a           unused
 * @param[in]   b           unused
 * @param[in]   c           unused
 */
static void spinner(void* a, void* b, void* c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (true) {
        spin_count++;
    }
}

/* Started only once the display is up, so it cannot hide boot problems */
K_THREAD_DEFINE(spin_tid, 1024, spinner, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0,
                SYS_FOREVER_MS);

/**
 * @brief Fill the band buffer with one RGB565X color.
 *
 * @param[in]   color       RGB565 color value
 */
static void fill(uint16_t color)
{
    for (size_t i = 0; i < sizeof(band); i += 2) {
        /* RGB565X: most significant byte first */
        band[i]     = color >> 8;
        band[i + 1] = color & 0xFF;
    }
}

/**
 * @brief Measure the spinner rate over 200 ms.
 *
 * @return spinner iterations per millisecond
 */
static uint32_t spin_per_ms(void)
{
    uint32_t start = spin_count;

    k_msleep(200);
    return (spin_count - start) / 200U;
}

/**
 * @brief Write full frames to the chosen display and print timing, throughput and free CPU.
 *
 * @return 0
 */
int main(void)
{
    const struct device*             disp     = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    static const uint16_t            colors[] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };
    struct display_capabilities      caps;
    struct display_buffer_descriptor desc = { 0 };
    uint32_t                         idle_rate, busy_spins, start_spins;
    int64_t                          start;
    uint32_t                         elapsed_ms;
    size_t                           frame_bytes;

    if (!device_is_ready(disp)) {
        printk("Display not ready\n");
        return 0;
    }

    display_get_capabilities(disp, &caps);
    if (caps.x_resolution > MAX_WIDTH || caps.current_pixel_format != PIXEL_FORMAT_RGB_565X) {
        printk("Unexpected panel: %ux%u fmt 0x%x\n", caps.x_resolution, caps.y_resolution,
               caps.current_pixel_format);
        return 0;
    }

    display_blanking_off(disp);
    k_thread_start(spin_tid);

    desc.width    = caps.x_resolution;
    desc.pitch    = caps.x_resolution;
    desc.height   = BAND_ROWS;
    desc.buf_size = desc.width * BAND_ROWS * BPP;
    frame_bytes   = (size_t)caps.x_resolution * caps.y_resolution * BPP;

    /* Baseline: how fast the spinner runs with nothing else going on */
    idle_rate = spin_per_ms();

    start_spins = spin_count;
    start       = k_uptime_get();

    for (int f = 0; f < FRAMES; f++) {
        fill(colors[f % ARRAY_SIZE(colors)]);
        for (uint16_t y = 0; y < caps.y_resolution; y += BAND_ROWS) {
            desc.height = MIN(BAND_ROWS, caps.y_resolution - y);
            display_write(disp, 0, y, &desc, band);
        }
    }

    elapsed_ms = (uint32_t)(k_uptime_get() - start);
    busy_spins = spin_count - start_spins;

    printk("\n%d frames of %ux%u in %u ms\n", FRAMES, caps.x_resolution, caps.y_resolution,
           elapsed_ms);
    printk("  frame time : %u.%02u ms (%u.%u fps)\n", elapsed_ms / FRAMES,
           (elapsed_ms * 100 / FRAMES) % 100, FRAMES * 1000 / elapsed_ms,
           (FRAMES * 10000 / elapsed_ms) % 10);
    printk("  throughput : %u KB/s\n",
           (uint32_t)((uint64_t)frame_bytes * FRAMES / elapsed_ms * 1000 / 1024));
    printk("  free CPU   : %u%% (spinner %u/ms idle, %u/ms during writes)\n",
           idle_rate ? (uint32_t)((uint64_t)busy_spins * 100 / elapsed_ms / idle_rate) : 0,
           idle_rate, busy_spins / elapsed_ms);

    return 0;
}
