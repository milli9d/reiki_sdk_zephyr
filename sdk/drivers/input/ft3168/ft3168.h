#pragma once

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>

/* ========================================================================= */
/* Zephyr Driver pre-Boilerplate */
/* ========================================================================= */

#define TP_QUEUE_SIZE

#define FT3168_RESET_BOOT_MS (300u) // controller firmware boot time after reset

typedef struct ft3168_tp_t {
    uint8_t  weight;
    uint16_t x;
    uint16_t y;
    uint64_t ts_ms;
} ft3168_tp_t;

typedef struct msgq_if_t {
    struct k_msgq msgq;
} msgq_if_t;

// runtime data
typedef struct ft3168_data_t {
    const struct device* dev;
    struct k_work        touch_work;
    bool                 print_en;
    struct gpio_callback isr_cb;
    msgq_if_t            q_if;
    bool                 pressed; // last reported INPUT_BTN_TOUCH state
} ft3168_data_t;

// constant data
typedef struct ft3168_config_t {
    struct input_touchscreen_common_config common; // must be first
    struct i2c_dt_spec                     i2c;
    struct gpio_dt_spec                    reset_gpio;
#ifdef CONFIG_INPUT_FT3168_INTERRUPT
    struct gpio_dt_spec int_gpio;
#endif
} ft3168_config_t;

/* ========================================================================= */
/* Register Access */
/* ========================================================================= */

#pragma pack(push, 1)

/* ========================================================================= */
#define FT3168_REG_MODE_SW (0x00u)

typedef enum ft3168_mode_t : uint8_t {
    FT3168_MODE_WORKING = 0b000u,
    FT3168_MODE_TEST    = 0b100u
} ft3168_mode_t;

typedef struct {
    uint8_t       rsv0 : 4u;
    ft3168_mode_t mode : 3u;
    uint8_t       rsv1 : 1u;
} ft3168_mode_sw_reg_t;

/* ========================================================================= */
#define FT3168_REG_GESTURE (0x01u)

typedef enum ft3168_gesture_id_t : uint8_t {
    FT3168_GESTURE_NONE  = 0x00u,
    FT3168_GESTURE_UP    = 0x10u,
    FT3168_GESTURE_RIGHT = 0x14u,
    FT3168_GESTURE_DOWN  = 0x18u,
    FT3168_GESTURE_LEFT  = 0x1Cu,
    FT3168_GESTURE_IN    = 0x48u,
    FT3168_GESTURE_OUT   = 0x49u,
} ft3168_gesture_id_t;

typedef struct {
    ft3168_gesture_id_t gesture_id : 8u;
} ft3168_gesture_reg_t;

/* ========================================================================= */
#define FT3168_REG_CURR_PT (0x02u)

/* Datasheet lists [7:0], but only 2 points are supported; the upper nibble has
 * been seen set with no finger down on related FocalTech parts, so mask it. */
typedef struct {
    uint8_t n_touch_pts : 4u;
    uint8_t rsv0        : 4u;
} ft3168_curr_pt_reg_t;

/* ========================================================================= */
/* Touch point block: XH, XL, YH, YL, WEIGHT, MISC (6 bytes, MSB first)      */
/* ========================================================================= */
#define FT3168_REG_T1_XH     (0x03u)
#define FT3168_REG_T1_XL     (0x04u)
#define FT3168_REG_T1_YH     (0x05u)
#define FT3168_REG_T1_YL     (0x06u)
#define FT3168_REG_T1_WEIGHT (0x07u)
#define FT3168_REG_T1_MISC   (0x08u)
#define FT3168_REG_T2_XH     (0x09u)
#define FT3168_REG_T2_XL     (0x0Au)
#define FT3168_REG_T2_YH     (0x0Bu)
#define FT3168_REG_T2_YL     (0x0Cu)
#define FT3168_REG_T2_WEIGHT (0x0Du)
#define FT3168_REG_T2_MISC   (0x0Eu)

typedef enum ft3168_evt_flag_t : uint8_t {
    FT3168_EVT_PRESS_DOWN = 0b00u,
    FT3168_EVT_LIFT_UP    = 0b01u,
    FT3168_EVT_CONTACT    = 0b10u,
    FT3168_EVT_NONE       = 0b11u,
} ft3168_evt_flag_t;

/* TOUCHn_XH: [7:6] event flag, [5:4] reserved, [3:0] X[11:8] */
typedef struct {
    uint8_t           x_hi     : 4u;
    uint8_t           rsv0     : 2u;
    ft3168_evt_flag_t evt_flag : 2u;
} ft3168_tp_xh_reg_t;

/* TOUCHn_YH: [7:4] touch ID, [3:0] Y[11:8] */
typedef struct {
    uint8_t y_hi     : 4u;
    uint8_t touch_id : 4u;
} ft3168_tp_yh_reg_t;

/* TOUCHn_MISC: [7:4] touch area, [3:0] reserved */
typedef struct {
    uint8_t rsv0       : 4u;
    uint8_t touch_area : 4u;
} ft3168_tp_misc_reg_t;

typedef struct {
    ft3168_tp_xh_reg_t   xh;
    uint8_t              x_lo;
    ft3168_tp_yh_reg_t   yh;
    uint8_t              y_lo;
    uint8_t              weight;
    ft3168_tp_misc_reg_t misc;
} ft3168_tp_reg_t;

static inline uint16_t ft3168_tp_x(const ft3168_tp_reg_t* tp)
{ return (uint16_t)(((uint16_t)tp->xh.x_hi << 8u) | tp->x_lo); }

static inline uint16_t ft3168_tp_y(const ft3168_tp_reg_t* tp)
{ return (uint16_t)(((uint16_t)tp->yh.y_hi << 8u) | tp->y_lo); }

/* Registers 0x00..0x0E read in one burst from FT3168_REG_MODE_SW */
typedef struct {
    ft3168_mode_sw_reg_t mode_sw;    // 0x00
    ft3168_gesture_reg_t gesture_id; // 0x01
    ft3168_curr_pt_reg_t curr_pt;    // 0x02
    ft3168_tp_reg_t      tp1;        // 0x03..0x08
    ft3168_tp_reg_t      tp2;        // 0x09..0x0E
} ft3168_reg_map_t;

BUILD_ASSERT(sizeof(ft3168_tp_reg_t) == 6u, "FT3168 touch point block must be 6 bytes");
BUILD_ASSERT(sizeof(ft3168_reg_map_t) == (FT3168_REG_T2_MISC + 1u),
             "FT3168 register map must cover 0x00..0x0E");

#pragma pack(pop)