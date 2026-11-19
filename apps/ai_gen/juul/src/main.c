/*
 * Copyright (c) 2026 Milind Singh
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file main.c
 * @brief Juul: light painting wand on the SH8601 AMOLED and QMI8658 IMU.
 *
 * - SLEEP  panel off while the device lies still
 * - DIM    fades up to a dim white once it is picked up
 * - PAINT  colours while it is swung: the hue follows the swing direction and
 *          rolls along with the distance travelled. Gentle motion gives soft
 *          pastels that glide; harder shaking makes the colours brighter, more
 *          saturated and quicker to change. A sharp flick flashes white.
 * - FADE   fades out after it has been put down, then sleeps
 *
 * Nothing jumps: the drawn colour eases toward its target and brightness
 * slews on a perceptual scale, so every transition is a blend.
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(juul, LOG_LEVEL_INF);

/* ========================================================================= */
/* Tuning */
/* ========================================================================= */

/*
 * Motion is measured bias free: accel as the distance from its own low-pass
 * (so gravity and tilt drop out) and gyro with its zero-rate offset removed.
 */
#define WAKE_ACCEL_MS2  1.2f /* wake deadzone: below both is ignored */
#define WAKE_GYRO_RADS  0.6f
#define STILL_ACCEL_MS2 0.08f /* smoothed motion below both: lying still */
#define STILL_GYRO_RADS 0.05f
/*
 * Dim/paint deadzone: handling it stays dim white; painting needs a real
 * swing or shake over the ENTER thresholds, held for PAINT_ENTER_MS. Once
 * painting, motion only has to stay over the lower KEEP thresholds.
 */
#define PAINT_ENTER_GYRO_RADS 3.5f
#define PAINT_ENTER_ACCEL_MS2 7.0f
#define PAINT_KEEP_GYRO_RADS  2.0f
#define PAINT_KEEP_ACCEL_MS2  4.0f
#define FULL_GYRO_RADS        12.0f /* full energy from rotation */
#define FULL_ACCEL_MS2        20.0f /* full energy from shaking */
#define FLICK_ACCEL_MS2       25.0f /* sharp flick: white flash */

#define WAKE_HOLD_MS   80  /* motion must stay over the deadzone this long */
#define SLEEP_AFTER_MS 750 /* still this long: fade out and sleep */
#define PAINT_ENTER_MS 60  /* swing must last this long to start painting */
#define PAINT_HOLD_MS  250 /* calm this long: blend back to dim white */
#define FLICK_GAP_MS   150 /* minimum time between flashes */

/* Perceived brightness levels, 0..1, mapped to the panel through a gamma */
#define LEVEL_DIM       0.35f
#define LEVEL_PAINT_MIN 0.6f
#define LEVEL_GAMMA     2.2f

/* Brightness slew in perceived level per second */
#define RATE_WAKE  1.2f /* 0 -> dim in 0.3 s */
#define RATE_FADE  0.8f /* dim -> 0 in 0.45 s */
#define RATE_DIM   2.0f /* paint -> dim */
#define RATE_PAINT 6.0f

/* Colour blend time constants: slow motion glides, fast motion snaps */
#define TAU_SLOW_S 0.15f
#define TAU_FAST_S 0.02f
#define TAU_CALM_S 0.12f

/* Saturation from soft pastel at low energy to vivid at full energy */
#define SAT_SLOW 0.4f
#define SAT_FAST 1.0f

/* Hue travel per radian of rotation, rising with energy */
#define HUE_PER_RAD_SLOW 60.0f
#define HUE_PER_RAD_FAST 240.0f

/* Energy smoothing: rises quickly, falls back a little slower */
#define TAU_ENERGY_UP_S   0.03f
#define TAU_ENERGY_DOWN_S 0.25f

/* Filter time constants */
#define TAU_ACCEL_LP_S 0.5f  /* slow accel: gravity and tilt */
#define TAU_GYRO_LP_S  0.5f  /* slow gyro: steady rotation or offset */
#define TAU_FAST_LP_S  0.08f /* noise smoothing for the stillness check */
#define TAU_GYRO_BIAS  1.0f
#define TAU_SWING_S    0.06f

/* Log the motion figures twice a second, for tuning the thresholds */
#define JUUL_TELEMETRY      1
#define TELEMETRY_PERIOD_MS 500

/* ========================================================================= */
/* Configuration */
/* ========================================================================= */

#define LOOP_PERIOD_MS  10 /* idle loop period while lit */
#define SLEEP_PERIOD_MS 20 /* IMU poll period while asleep */

#define BAND_ROWS 32  /* rows per display write */
#define MAX_WIDTH 368 /* widest panel the band buffer fits */
#define BPP       2   /* bytes per RGB565X pixel */

/* ========================================================================= */
/* Types */
/* ========================================================================= */

/** Wand state. */
enum juul_state {
    JUUL_SLEEP, /**< panel off, waiting to be picked up */
    JUUL_DIM,   /**< held: dim white */
    JUUL_PAINT, /**< swung: colours */
    JUUL_FADE,  /**< put down: fading out before sleep */
};

/** Linear RGB colour, each channel 0..1. */
struct rgb {
    float r;
    float g;
    float b;
};

/** Filtered IMU state and the motion figures derived from it. */
struct motion {
    float accel_lp[3];   /**< slow accel, tracks gravity and tilt */
    float accel_fast[3]; /**< accel with sample noise smoothed out */
    float gyro_lp[3];    /**< slow gyro */
    float gyro_fast[3];  /**< gyro with sample noise smoothed out */
    float gyro_bias[3];  /**< zero-rate offset, learnt while at rest */
    float swing[2];      /**< smoothed X/Y rotation, picks the hue */
    bool  primed;        /**< filters seeded with a first sample */
    float accel;         /**< |a - accel_lp| in m/s^2, instantaneous */
    float gyro;          /**< |w - gyro_bias| in rad/s, instantaneous */
    float accel_still;   /**< |accel_fast - accel_lp|, for stillness */
    float gyro_still;    /**< |gyro_fast - gyro_bias|, for stillness */
    float energy;        /**< smoothed motion intensity, 0..1 */
};

/* ========================================================================= */
/* Private Data */
/* ========================================================================= */

/** State names for logging, indexed by enum juul_state. */
static const char* const state_names[] = { "sleep", "dim", "paint", "fade" };

/** Full-white colour. */
static const struct rgb WHITE = { 1.0f, 1.0f, 1.0f };

/** One band of the frame; the whole screen is one colour. */
static uint8_t band[MAX_WIDTH * BAND_ROWS * BPP] __aligned(4);

/** Chosen display (SH8601). */
static const struct device* const disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/** IMU (QMI8658). */
static const struct device* const imu = DEVICE_DT_GET_ONE(qst_qmi8658a);

/** Panel capabilities, read once at start up. */
static struct display_capabilities caps;

/* ========================================================================= */
/* Easing and Colour */
/* ========================================================================= */

/**
 * @brief Move @p cur toward @p target with first-order (exponential) easing.
 *
 * @param[in]   cur         current value
 * @param[in]   target      target value
 * @param[in]   tau         time constant in seconds
 * @param[in]   dt          time step in seconds
 *
 * @return the eased value
 */
static float ease(float cur, float target, float tau, float dt)
{ return cur + (target - cur) * (1.0f - expf(-dt / tau)); }

/**
 * @brief Move @p cur toward @p target by at most @p rate * @p dt.
 *
 * @param[in]   cur         current value
 * @param[in]   target      target value
 * @param[in]   rate        maximum change per second
 * @param[in]   dt          time step in seconds
 *
 * @return the slewed value
 */
static float slew(float cur, float target, float rate, float dt)
{
    float step = rate * dt;

    if (target > cur) {
        return MIN(cur + step, target);
    }
    return MAX(cur - step, target);
}

/**
 * @brief Linear interpolation between @p a and @p b.
 *
 * @param[in]   a           value at t = 0
 * @param[in]   b           value at t = 1
 * @param[in]   t           position, 0..1
 *
 * @return the interpolated value
 */
static float lerp(float a, float b, float t)
{ return a + (b - a) * t; }

/**
 * @brief Convert a hue and saturation to a full value colour.
 *
 * @param[in]   hue         hue in degrees, any range
 * @param[in]   sat         saturation, 0 (white) to 1 (pure hue)
 *
 * @return the colour
 */
static struct rgb hue_color(float hue, float sat)
{
    float      h = fmodf(hue, 360.0f);
    float      x;
    struct rgb c;

    if (h < 0.0f) {
        h += 360.0f;
    }
    x = 1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f);

    switch ((int)(h / 60.0f)) {
        case 0:
            c = (struct rgb){ 1.0f, x, 0.0f };
            break;
        case 1:
            c = (struct rgb){ x, 1.0f, 0.0f };
            break;
        case 2:
            c = (struct rgb){ 0.0f, 1.0f, x };
            break;
        case 3:
            c = (struct rgb){ 0.0f, x, 1.0f };
            break;
        case 4:
            c = (struct rgb){ x, 0.0f, 1.0f };
            break;
        default:
            c = (struct rgb){ 1.0f, 0.0f, x };
            break;
    }

    /* Wash the pure hue toward white */
    c.r = 1.0f - sat * (1.0f - c.r);
    c.g = 1.0f - sat * (1.0f - c.g);
    c.b = 1.0f - sat * (1.0f - c.b);

    return c;
}

/**
 * @brief Quantise a colour to RGB565.
 *
 * @param[in]   c           colour, channels 0..1
 *
 * @return RGB565 value
 */
static uint16_t to_rgb565(struct rgb c)
{
    return ((uint16_t)(CLAMP(c.r, 0.0f, 1.0f) * 31.0f + 0.5f) << 11) |
           ((uint16_t)(CLAMP(c.g, 0.0f, 1.0f) * 63.0f + 0.5f) << 5) |
           (uint16_t)(CLAMP(c.b, 0.0f, 1.0f) * 31.0f + 0.5f);
}

/**
 * @brief Map a perceived brightness level to the panel brightness register.
 *
 * @param[in]   level       perceived level 0..1
 *
 * @return panel brightness 0..255
 */
static uint8_t level_to_brightness(float level)
{ return (uint8_t)(powf(CLAMP(level, 0.0f, 1.0f), LEVEL_GAMMA) * 255.0f + 0.5f); }

/* ========================================================================= */
/* Display */
/* ========================================================================= */

/**
 * @brief Fill the whole screen with one colour.
 *
 * @param[in]   color       RGB565 colour
 *
 * @return 0 on success, negative errno from the display driver otherwise
 */
static int fill_screen(uint16_t color)
{
    struct display_buffer_descriptor desc = {
        .width = caps.x_resolution,
        .pitch = caps.x_resolution,
    };
    int ret;

    /* RGB565X: most significant byte first */
    for (size_t i = 0; i < sizeof(band); i += 2) {
        band[i]     = color >> 8;
        band[i + 1] = color & 0xFF;
    }

    for (uint16_t y = 0; y < caps.y_resolution; y += BAND_ROWS) {
        desc.height   = MIN(BAND_ROWS, caps.y_resolution - y);
        desc.buf_size = desc.width * desc.height * BPP;
        ret           = display_write(disp, 0, y, &desc, band);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

/* ========================================================================= */
/* Motion */
/* ========================================================================= */

/**
 * @brief Read the IMU and update the motion filters.
 *
 * @param[in,out] m         motion filters and derived figures
 * @param[in]   dt          time since the previous sample in seconds
 *
 * @return 0 on success, negative errno from the sensor driver otherwise
 */
static int read_motion(struct motion* m, float dt)
{
    struct sensor_value raw_a[3];
    struct sensor_value raw_w[3];
    float               a[3], w[3];
    int                 ret;

    ret = sensor_sample_fetch(imu);
    if (ret == 0) {
        ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, raw_a);
    }
    if (ret == 0) {
        ret = sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, raw_w);
    }
    if (ret < 0) {
        return ret;
    }

    for (int i = 0; i < 3; i++) {
        a[i] = sensor_value_to_float(&raw_a[i]);
        w[i] = sensor_value_to_float(&raw_w[i]);
    }

    if (!m->primed) {
        for (int i = 0; i < 3; i++) {
            m->accel_lp[i]   = a[i];
            m->accel_fast[i] = a[i];
            m->gyro_lp[i]    = w[i];
            m->gyro_fast[i]  = w[i];
            m->gyro_bias[i]  = w[i];
        }
        m->primed = true;
    }

    {
        float da2 = 0.0f, dw2 = 0.0f, sa2 = 0.0f, sw2 = 0.0f, rot2 = 0.0f;

        for (int i = 0; i < 3; i++) {
            float da, dw, sa, sw, rot;

            m->accel_fast[i] = ease(m->accel_fast[i], a[i], TAU_FAST_LP_S, dt);
            m->gyro_fast[i]  = ease(m->gyro_fast[i], w[i], TAU_FAST_LP_S, dt);

            da  = a[i] - m->accel_lp[i];
            dw  = w[i] - m->gyro_bias[i];
            sa  = m->accel_fast[i] - m->accel_lp[i];
            sw  = m->gyro_fast[i] - m->gyro_bias[i];
            rot = m->gyro_fast[i] - m->gyro_lp[i];

            da2 += da * da;
            dw2 += dw * dw;
            sa2 += sa * sa;
            sw2 += sw * sw;
            rot2 += rot * rot;

            m->accel_lp[i] = ease(m->accel_lp[i], a[i], TAU_ACCEL_LP_S, dt);
            m->gyro_lp[i]  = ease(m->gyro_lp[i], w[i], TAU_GYRO_LP_S, dt);
        }

        m->accel       = sqrtf(da2);
        m->gyro        = sqrtf(dw2);
        m->accel_still = sqrtf(sa2);
        m->gyro_still  = sqrtf(sw2);

        /*
         * At rest the slow gyro is the offset. Learn it whenever accel is
         * quiet and the gyro is steady, which needs no prior offset, so a
         * bad first sample cannot lock learning out.
         */
        if (m->accel_still < STILL_ACCEL_MS2 && sqrtf(rot2) < STILL_GYRO_RADS) {
            for (int i = 0; i < 3; i++) {
                m->gyro_bias[i] = ease(m->gyro_bias[i], m->gyro_lp[i], TAU_GYRO_BIAS, dt);
            }
        }
    }

    m->swing[0] = ease(m->swing[0], w[0] - m->gyro_bias[0], TAU_SWING_S, dt);
    m->swing[1] = ease(m->swing[1], w[1] - m->gyro_bias[1], TAU_SWING_S, dt);

    {
        /* 0 at the paint keep threshold, so painting starts out pastel */
        float rg  = (m->gyro - PAINT_KEEP_GYRO_RADS) / (FULL_GYRO_RADS - PAINT_KEEP_GYRO_RADS);
        float ra  = (m->accel - PAINT_KEEP_ACCEL_MS2) / (FULL_ACCEL_MS2 - PAINT_KEEP_ACCEL_MS2);
        float raw = CLAMP(MAX(rg, ra), 0.0f, 1.0f);

        m->energy =
            ease(m->energy, raw, (raw > m->energy) ? TAU_ENERGY_UP_S : TAU_ENERGY_DOWN_S, dt);
    }

    return 0;
}

/* ========================================================================= */
/* Application */
/* ========================================================================= */

/**
 * @brief Wait for the display and IMU, then run the light painting loop.
 *
 * @return 0 when a device is missing, otherwise never returns
 */
int main(void)
{
    enum juul_state state       = JUUL_SLEEP;
    struct motion   m           = { 0 };
    struct rgb      cur         = WHITE;
    struct rgb      target      = WHITE;
    float           level       = 0.0f;
    float           level_goal  = 0.0f;
    float           phase       = 0.0f;
    int64_t         prev        = k_uptime_get();
    int64_t         last_move   = prev;
    int64_t         last_swing  = 0;
    int64_t         swing_start = -1;
    int64_t         last_log    = 0;
    int64_t         last_flick  = 0;
    int64_t         wake_start  = -1;
    int             drawn_rgb   = -1;
    int             drawn_bri   = -1;

    if (!device_is_ready(disp) || !device_is_ready(imu)) {
        LOG_ERR("Display or IMU not ready");
        return 0;
    }

    display_get_capabilities(disp, &caps);
    if (caps.x_resolution > MAX_WIDTH || caps.current_pixel_format != PIXEL_FORMAT_RGB_565X) {
        LOG_ERR("Unexpected panel %ux%u fmt 0x%x", caps.x_resolution, caps.y_resolution,
                caps.current_pixel_format);
        return 0;
    }

    display_blanking_on(disp);
    LOG_INF("Ready: pick me up");

    while (true) {
        int64_t         now  = k_uptime_get();
        float           dt   = MIN((float)(now - prev) / 1000.0f, 0.1f);
        enum juul_state next = state;
        bool            swinging, keep_swinging, still;
        float           rate = RATE_PAINT;
        float           tau  = TAU_CALM_S;
        uint16_t        rgb565;
        uint8_t         bri;

        prev = now;

        if (read_motion(&m, dt) < 0) {
            LOG_WRN("IMU read failed");
            k_msleep(LOOP_PERIOD_MS);
            continue;
        }

        keep_swinging = m.gyro > PAINT_KEEP_GYRO_RADS || m.accel > PAINT_KEEP_ACCEL_MS2;

        /* A swing only counts once it has lasted PAINT_ENTER_MS */
        if (m.gyro > PAINT_ENTER_GYRO_RADS || m.accel > PAINT_ENTER_ACCEL_MS2) {
            if (swing_start < 0) {
                swing_start = now;
            }
        } else if (!keep_swinging) {
            swing_start = -1;
        }
        swinging = swing_start >= 0 && now - swing_start >= PAINT_ENTER_MS;
        still    = m.accel_still < STILL_ACCEL_MS2 && m.gyro_still < STILL_GYRO_RADS;
        if (!still) {
            last_move = now;
        }

#if JUUL_TELEMETRY
        if (now - last_log >= TELEMETRY_PERIOD_MS) {
            last_log = now;
            /* milli-units: m/s^2 and rad/s; still flags the sleep timer */
            LOG_INF("%-5s a=%5d g=%5d | still a=%4d g=%4d %s | e=%3d%% sleep in %d ms",
                    state_names[state], (int)(m.accel * 1000), (int)(m.gyro * 1000),
                    (int)(m.accel_still * 1000), (int)(m.gyro_still * 1000), still ? "yes" : "no ",
                    (int)(m.energy * 100), (int)MAX(0, SLEEP_AFTER_MS - (now - last_move)));
        }
#endif

        /* ---- state ---- */
        switch (state) {
            case JUUL_SLEEP:
                /* Ignore knocks: wake only on motion that persists */
                if (m.accel < WAKE_ACCEL_MS2 && m.gyro < WAKE_GYRO_RADS) {
                    wake_start = -1;
                } else if (wake_start < 0) {
                    wake_start = now;
                } else if (now - wake_start >= WAKE_HOLD_MS) {
                    wake_start = -1;
                    next       = JUUL_DIM;
                }
                break;
            case JUUL_DIM:
                if (swinging) {
                    next = JUUL_PAINT;
                } else if (now - last_move > SLEEP_AFTER_MS) {
                    next = JUUL_FADE;
                }
                break;
            case JUUL_PAINT:
                if (keep_swinging) {
                    last_swing = now;
                } else if (now - last_swing > PAINT_HOLD_MS) {
                    next = JUUL_DIM;
                }
                break;
            case JUUL_FADE:
                if (swinging) {
                    next = JUUL_PAINT;
                } else if (!still) {
                    next = JUUL_DIM;
                } else if (level <= 0.0f) {
                    next = JUUL_SLEEP;
                }
                break;
        }

        if (next != state) {
            LOG_INF("%s -> %s", state_names[state], state_names[next]);
            if (state == JUUL_SLEEP) {
                /* Start from black at white so the fade-in has no flash */
                level = 0.0f;
                cur   = WHITE;
                display_set_brightness(disp, 0);
                fill_screen(to_rgb565(cur));
                display_blanking_off(disp);
                drawn_rgb = to_rgb565(cur);
                drawn_bri = 0;
            }
            if (next == JUUL_PAINT) {
                last_swing = now;
            }
            if (next == JUUL_SLEEP) {
                display_blanking_on(disp);
            }
            state = next;
        }

        if (state == JUUL_SLEEP) {
            k_msleep(SLEEP_PERIOD_MS);
            continue;
        }

        /* ---- targets ---- */
        switch (state) {
            case JUUL_DIM:
                target     = WHITE;
                level_goal = LEVEL_DIM;
                rate       = (level < LEVEL_DIM) ? RATE_WAKE : RATE_DIM;
                tau        = TAU_CALM_S;
                break;
            case JUUL_PAINT: {
                float e   = m.energy;
                float hue = atan2f(m.swing[1], m.swing[0]) * (180.0f / 3.14159265f);

                /*
                 * Swing direction picks the hue and travel rolls it along;
                 * energy decides how vivid, bright and quick it all is.
                 */
                phase  = fmodf(phase + m.gyro * dt * lerp(HUE_PER_RAD_SLOW, HUE_PER_RAD_FAST, e),
                               360.0f);
                target = hue_color(hue + phase, lerp(SAT_SLOW, SAT_FAST, e));
                level_goal = lerp(LEVEL_PAINT_MIN, 1.0f, e);
                rate       = RATE_PAINT;
                tau        = lerp(TAU_SLOW_S, TAU_FAST_S, e);

                if (m.accel > FLICK_ACCEL_MS2 && now - last_flick > FLICK_GAP_MS) {
                    /* Jump to white and let it blend back out */
                    last_flick = now;
                    cur        = WHITE;
                    level      = 1.0f;
                }
                break;
            }
            case JUUL_FADE:
                level_goal = 0.0f;
                rate       = RATE_FADE;
                tau        = TAU_CALM_S;
                break;
            default:
                break;
        }

        /* ---- render ---- */
        cur.r = ease(cur.r, target.r, tau, dt);
        cur.g = ease(cur.g, target.g, tau, dt);
        cur.b = ease(cur.b, target.b, tau, dt);
        level = slew(level, level_goal, rate, dt);

        rgb565 = to_rgb565(cur);
        bri    = level_to_brightness(level);

        if (bri != drawn_bri) {
            display_set_brightness(disp, bri);
            drawn_bri = bri;
        }
        if (rgb565 != drawn_rgb) {
            /* A frame write takes ~17 ms and paces the loop by itself */
            fill_screen(rgb565);
            drawn_rgb = rgb565;
        } else {
            k_msleep(LOOP_PERIOD_MS);
        }
    }

    return 0;
}
