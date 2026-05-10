#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(QMI8658C, LOG_LEVEL_DBG);

#include "qmi8658c.h"

#define DT_DRV_COMPAT qorvo_qmi8658c

/* ========================================================================= */
/* Private Data Types */
/* ========================================================================= */

typedef struct {
    const struct i2c_dt_spec  i2c;
    const struct gpio_dt_spec int1_gpio;
} qmi8658c_cfg_t;

typedef struct {
} qmi8658c_data_t;

/* ========================================================================= */
/* Private Helper API Declarations */
/* ========================================================================= */

static const char* qmi8658c_aodr_to_str(uint8_t odr)
{
    switch (odr) {
        case QMI8658C_REG_CTRL2_AODR_8KHZ:
            return "8 kHz";
        case QMI8658C_REG_CTRL2_AODR_4KHZ:
            return "4 kHz";
        case QMI8658C_REG_CTRL2_AODR_2KHZ:
            return "2 kHz";
        case QMI8658C_REG_CTRL2_AODR_1KHZ:
            return "1 kHz";
        case QMI8658C_REG_CTRL2_AODR_500HZ:
            return "500 Hz";
        case QMI8658C_REG_CTRL2_AODR_250HZ:
            return "250 Hz";
        case QMI8658C_REG_CTRL2_AODR_125HZ:
            return "125 Hz";
        case QMI8658C_REG_CTRL2_AODR_62_5HZ:
            return "62.5 Hz";
        case QMI8658C_REG_CTRL2_AODR_31_25HZ:
            return "31.25 Hz";
        case QMI8658C_REG_CTRL2_AODR_LP_128HZ:
            return "LP 128 Hz";
        case QMI8658C_REG_CTRL2_AODR_LP_21HZ:
            return "LP 21 Hz";
        case QMI8658C_REG_CTRL2_AODR_LP_11HZ:
            return "LP 11 Hz";
        case QMI8658C_REG_CTRL2_AODR_LP_3HZ:
            return "LP 3 Hz";
        default:
            return "Unknown ODR";
    }
}

static const char* qmi8658c_afs_to_str(uint8_t fs)
{
    switch (fs) {
        case QMI8658C_REG_CTRL2_AFS_2G:
            return "2G";
        case QMI8658C_REG_CTRL2_AFS_4G:
            return "4G";
        case QMI8658C_REG_CTRL2_AFS_8G:
            return "8G";
        case QMI8658C_REG_CTRL2_AFS_16G:
            return "16G";
        default:
            return "Unknown FS";
    }
}

/* ========================================================================= */
/* Private API Declarations */
/* ========================================================================= */

static int qmi8658c_set_accel_fs(const struct device* dev, uint8_t fs)
{
    const qmi8658c_cfg_t*     cfg = (qmi8658c_cfg_t*)dev->config;
    const struct i2c_dt_spec* i2c = &cfg->i2c;

    if (fs < QMI8658C_REG_CTRL2_AFS_2G && fs > QMI8658C_REG_CTRL2_AFS_16G) {
        LOG_ERR("Unsupported aFS reg value. See driver headers or datasheet.");
        return -EINVAL;
    }

    uint8_t mask = 0u;
    int     rc   = i2c_reg_read_byte_dt(i2c, QMI8658C_REG_CTRL2, &mask);
    if (rc != 0) {
        LOG_ERR("Cannot read CTRL2 register, rc = %d", rc);
        return rc;
    }

    mask &= ~(QMI8658C_REG_CTRL2_AFS_MASK);
    mask |= QMI8658C_REG_CTRL2_AFS(fs);

    rc = i2c_reg_write_byte_dt(i2c, QMI8658C_REG_CTRL2, mask);
    if (rc != 0) {
        LOG_ERR("Cannot write CTRL2 register, rc = %d", rc);
        return rc;
    }

    return 0;
}

static int qmi8658c_set_accel_odr(const struct device* dev, uint8_t odr)
{
    const qmi8658c_cfg_t*     cfg = (qmi8658c_cfg_t*)dev->config;
    const struct i2c_dt_spec* i2c = &cfg->i2c;

    if (odr > QMI8658C_REG_CTRL2_AODR_LP_3HZ ||
        (odr < QMI8658C_REG_CTRL2_AODR_LP_128HZ && odr > QMI8658C_REG_CTRL2_AODR_31_25HZ)) {
        LOG_ERR("Unsupported aODR reg value. See driver headers or datasheet.");
        return -EINVAL;
    }

    uint8_t mask = 0u;
    int     rc   = i2c_reg_read_byte_dt(i2c, QMI8658C_REG_CTRL2, &mask);
    if (rc != 0) {
        LOG_ERR("Cannot read CTRL2 register, rc = %d", rc);
        return rc;
    }

    mask &= ~(QMI8658C_REG_CTRL2_AODR_MASK);
    mask |= QMI8658C_REG_CTRL2_AODR(odr);

    rc = i2c_reg_write_byte_dt(i2c, QMI8658C_REG_CTRL2, mask);
    if (rc != 0) {
        LOG_ERR("Cannot write CTRL2 register, rc = %d", rc);
        return rc;
    }

    return 0;
}

static int qmi8658c_attr_set_accel_chan(const struct device* dev, enum sensor_channel chan,
                                        enum sensor_attribute attr, const struct sensor_value* val)
{
    if (chan != SENSOR_CHAN_ACCEL_X && chan != SENSOR_CHAN_ACCEL_Y && chan != SENSOR_CHAN_ACCEL_Z &&
        chan != SENSOR_CHAN_ACCEL_XYZ) {
        LOG_ERR("Invalid channel for setting accelerometer attribute.");
        return -EINVAL;
    }

    const qmi8658c_cfg_t*     cfg = (qmi8658c_cfg_t*)dev->config;
    const struct i2c_dt_spec* i2c = &cfg->i2c;

    switch (attr) {
        case SENSOR_ATTR_SAMPLING_FREQUENCY: {
            LOG_INF("Set sampling frequency to %s", qmi8658c_aodr_to_str(val->val1));
            int rc = qmi8658c_set_accel_odr(dev, val->val1);
            if (rc != 0) {
                LOG_ERR("Failed to set Accel ODR. rc = %d", rc);
                return rc;
            }
            break;
        }

        case SENSOR_ATTR_FULL_SCALE: {
            LOG_INF("Set full-scale range to %s", qmi8658c_afs_to_str(val->val1));
            int rc = qmi8658c_set_accel_fs(dev, val->val1);
            if (rc != 0) {
                LOG_ERR("Failed to set Accel FS. rc = %d", rc);
                return rc;
            }
            break;
        }

        default:
            LOG_ERR("Invalid attribute selected");
            return -EINVAL;
    }

    return 0;
}

static int qmi8658c_attr_set_gyro_chan(const struct device* dev, enum sensor_channel chan,
                                       enum sensor_attribute attr, const struct sensor_value* val)
{
    return -EINVAL;
}

static int qmi8658c_attr_set_magn_chan(const struct device* dev, enum sensor_channel chan,
                                       enum sensor_attribute attr, const struct sensor_value* val)
{
    return -EINVAL;
}

static int qmi8658c_attr_set(const struct device* dev, enum sensor_channel chan,
                             enum sensor_attribute attr, const struct sensor_value* val)
{
    switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
        case SENSOR_CHAN_ACCEL_Y:
        case SENSOR_CHAN_ACCEL_Z:
        case SENSOR_CHAN_ACCEL_XYZ:
            return qmi8658c_attr_set_accel_chan(dev, chan, attr, val);
        case SENSOR_CHAN_GYRO_X:
        case SENSOR_CHAN_GYRO_Y:
        case SENSOR_CHAN_GYRO_Z:
        case SENSOR_CHAN_GYRO_XYZ:
            return qmi8658c_attr_set_gyro_chan(dev, chan, attr, val);
        case SENSOR_CHAN_MAGN_X:
        case SENSOR_CHAN_MAGN_Y:
        case SENSOR_CHAN_MAGN_Z:
        case SENSOR_CHAN_MAGN_XYZ:
            return qmi8658c_attr_set_magn_chan(dev, chan, attr, val);
        default:
            LOG_ERR("Unsupported Channel 0x%X", (uint32_t)chan);
            break;
    };

    return -EINVAL;
}

/**
 * @brief Probe QMI8658 instance via chip ID, fetch chip revision
 *
 * @param[in] dev   device dt node
 * @return          true, if qmi8658c found
 */
static bool qmi8658_probe(const struct device* dev)
{
    const qmi8658c_cfg_t*     cfg = (qmi8658c_cfg_t*)dev->config;
    const struct i2c_dt_spec* i2c = &cfg->i2c;

    uint8_t whoami = 0u;
    uint8_t rev    = 0u;

    int rc = i2c_reg_read_byte_dt(i2c, QMI8658C_REG_WHOAMI, &whoami);
    if (rc != 0) {
        LOG_ERR("Failed to read register 0x%X", QMI8658C_REG_WHOAMI);
        return false;
    }

    if (whoami != QMI8658C_REG_WHOAMI_DEF) {
        LOG_DBG("QMI8658C chip ID does not match 0x%X / 0x%X", whoami, QMI8658C_REG_WHOAMI_DEF);
        return false;
    }

    rc = i2c_reg_read_byte_dt(i2c, QMI8658C_REG_REVID, &rev);
    if (rc != 0) {
        LOG_ERR("Failed to read register 0x%X", QMI8658C_REG_REVID);
        return false;
    }

    LOG_DBG("Found Qorvo QMI8658C Rev: 0x%X", rev);
    return true;
}

/* ========================================================================= */
/* Private Helper API Definitions */
/* ========================================================================= */

static int qmi8658c_init(const struct device* dev)
{
    const qmi8658c_cfg_t* cfg = (qmi8658c_cfg_t*)dev->config;
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("i2c bus is not available.");
        return -ENODEV;
    }

    if (!qmi8658_probe(dev)) {
        LOG_ERR("Device not found!");
        return -ENODEV;
    }

    return 0;
}

static DEVICE_API(sensor, qmi8658c_api) = {
    .attr_set = qmi8658c_attr_set,
};

#define QMI8658_INIT(inst)                                                                      \
    static const qmi8658c_cfg_t qmi8658c_cfg_##inst = {                                         \
        .i2c       = I2C_DT_SPEC_INST_GET(inst),                                                \
        .int1_gpio = GPIO_DT_SPEC_INST_GET(inst, int1_gpios),                                   \
    };                                                                                          \
                                                                                                \
    static qmi8658c_data_t qmi8658c_data_##inst;                                                \
                                                                                                \
    DEVICE_DT_INST_DEFINE(inst, &qmi8658c_init, NULL /* PM */, &qmi8658c_data_##inst,           \
                          &qmi8658c_cfg_##inst, POST_KERNEL, CONFIG_IMU_QMI8658C_INIT_PRIORITY, \
                          &qmi8658c_api);

DT_INST_FOREACH_STATUS_OKAY(QMI8658_INIT)