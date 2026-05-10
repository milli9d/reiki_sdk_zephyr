#pragma once

#include <stdio.h>
#include <stdint.h>
#include <errno.h>

/* ========================================================================= */
/* General Purpose Registers */
/* ========================================================================= */

#define QMI8658C_REG_WHOAMI     0x00u
#define QMI8658C_REG_WHOAMI_DEF 0x05u

#define QMI8658C_REG_REVID     0x01u
#define QMI8658C_REG_REVID_DEF 0x7Cu

/* ========================================================================= */
/* General Purpose Registers */
/* ========================================================================= */

#define QMI8658C_REG_CTRL2 0x0u

#define QMI8658C_REG_CTRL2_AODR(x)   (x << 0u)
#define QMI8658C_REG_CTRL2_AODR_MASK QMI8658C_REG_CTRL2_AODR(0xFu)

#define QMI8658C_REG_CTRL2_AODR_8KHZ     0x00u
#define QMI8658C_REG_CTRL2_AODR_4KHZ     0x01u
#define QMI8658C_REG_CTRL2_AODR_2KHZ     0x02u
#define QMI8658C_REG_CTRL2_AODR_1KHZ     0x03u
#define QMI8658C_REG_CTRL2_AODR_500HZ    0x04u
#define QMI8658C_REG_CTRL2_AODR_250HZ    0x05u
#define QMI8658C_REG_CTRL2_AODR_125HZ    0x06u
#define QMI8658C_REG_CTRL2_AODR_62_5HZ   0x07u
#define QMI8658C_REG_CTRL2_AODR_31_25HZ  0x08u
#define QMI8658C_REG_CTRL2_AODR_LP_128HZ 0x0Cu
#define QMI8658C_REG_CTRL2_AODR_LP_21HZ  0x0Du
#define QMI8658C_REG_CTRL2_AODR_LP_11HZ  0x0Eu
#define QMI8658C_REG_CTRL2_AODR_LP_3HZ   0x0Fu

#define QMI8658C_REG_CTRL2_AFS(x)   (x << 4u)
#define QMI8658C_REG_CTRL2_AFS_MASK QMI8658C_REG_CTRL2_AFS(0b111u)

#define QMI8658C_REG_CTRL2_AFS_2G  0x00
#define QMI8658C_REG_CTRL2_AFS_4G  0x01
#define QMI8658C_REG_CTRL2_AFS_8G  0x02
#define QMI8658C_REG_CTRL2_AFS_16G 0x03

#define QMI8658C_REG_ 0x0u
