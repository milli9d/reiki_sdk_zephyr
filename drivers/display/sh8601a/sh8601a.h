#pragma once

#include <zephyr/kernel.h>

#define SH8601_CMD_NOP      0x00
#define SH8601_CMD_SWRESET  0x01
#define SH8601_CMD_RDID     0x04
#define SH8601_CMD_RDERR    0x09
#define SH8601_CMD_RDPM     0x0A
#define SH8601_CMD_RDMADCTL 0x0B
#define SH8601_CMD_RDCOLMOD 0x0C
