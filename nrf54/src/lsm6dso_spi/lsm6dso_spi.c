/**
 * @file lsm6dso_spi.c
 * @brief LSM6DSO/LSM6DSL 6-axis IMU SPI driver implementation for Zephyr RTOS
 */

#include "lsm6dso_spi.h"
#include "../helper/spi_helper.h"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(lsm6dso_spi, LOG_LEVEL_INF);

/* SPI read bit (MSB of first byte) */
#define LSM6DSO_SPI_READ  0x80

/* Maximum burst read length */
#define LSM6DSO_MAX_BURST_LEN 14

/**
 * @brief Parse three little-endian 16-bit axis values from a 6-byte buffer
 */
static void parse_axis_3d(const uint8_t *raw, int16_t *x, int16_t *y, int16_t *z) {
    *x = (int16_t)((raw[1] << 8) | raw[0]);
    *y = (int16_t)((raw[3] << 8) | raw[2]);
    *z = (int16_t)((raw[5] << 8) | raw[4]);
}

int lsm6dso_init(lsm6dso_config_t *config) {
    int ret;

    if (!device_is_ready(config->spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    /* Configure INT1 GPIO if specified */
    if (config->int1_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&config->int1_gpio)) {
            LOG_ERR("INT1 GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->int1_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT1 GPIO: %d", ret);
            return ret;
        }
    }

    /* Configure INT2 GPIO if specified */
    if (config->int2_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&config->int2_gpio)) {
            LOG_ERR("INT2 GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->int2_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT2 GPIO: %d", ret);
            return ret;
        }
    }

    LOG_INF("LSM6DSO initialized");
    return 0;
}

int lsm6dso_read_register(lsm6dso_config_t *config, uint8_t reg, uint8_t *value) {
    uint8_t tx[2] = { LSM6DSO_SPI_READ | reg, 0x00 };
    uint8_t rx[2] = { 0 };

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx, rx, sizeof(tx));
    if (ret) {
        LOG_ERR("LSM6DSO SPI error: %d", ret);
        return ret;
    }

    /* rx[0] is garbage (clocked in while sending address), rx[1] is the value */
    *value = rx[1];
    LOG_DBG("LSM6DSO reg 0x%02x = 0x%02x", reg, *value);
    return 0;
}

int lsm6dso_write_register(lsm6dso_config_t *config, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg, value };
    uint8_t rx[2] = { 0 };

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx, rx, sizeof(tx));
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Write reg 0x%02x = 0x%02x", reg, value);
    return 0;
}

int lsm6dso_read_registers(lsm6dso_config_t *config, uint8_t start_reg, uint8_t *buf, size_t len) {
    if (len > LSM6DSO_MAX_BURST_LEN) {
        LOG_ERR("Burst read too long: %zu (max %d)", len, LSM6DSO_MAX_BURST_LEN);
        return -EINVAL;
    }

    uint8_t tx[LSM6DSO_MAX_BURST_LEN + 1] = {0};
    uint8_t rx[LSM6DSO_MAX_BURST_LEN + 1] = {0};

    tx[0] = LSM6DSO_SPI_READ | start_reg;

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx, rx, len + 1);
    if (ret) {
        LOG_ERR("SPI burst read failed: %d", ret);
        return ret;
    }

    memcpy(buf, &rx[1], len);
    return 0;
}

int lsm6dso_write_register_masked(lsm6dso_config_t *config, uint8_t reg,
    uint8_t value, uint8_t mask) {
    int ret;
    uint8_t reg_val;

    ret = lsm6dso_read_register(config, reg, &reg_val);
    if (ret < 0) {
        return ret;
    }

    reg_val &= ~mask;

    /* Auto-shift value to mask position (same convention as ADS131M02) */
    uint8_t temp_mask = mask;
    int shift = 0;
    while (temp_mask && !(temp_mask & 0x01)) {
        temp_mask >>= 1;
        shift++;
    }
    reg_val |= ((value << shift) & mask);

    ret = lsm6dso_write_register(config, reg, reg_val);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Write masked reg 0x%02x, mask 0x%02x, val 0x%02x", reg, mask, value);
    return 0;
}

int lsm6dso_read_data(lsm6dso_config_t *config, lsm6dso_data_t *data) {
    uint8_t raw[12] = { 0 };  /* 6 axes x 2 bytes each */

    /* Burst read starting from OUTX_L_G (0x22):
     * Bytes 0-5:  Gyro  X_L, X_H, Y_L, Y_H, Z_L, Z_H
     * Bytes 6-11: Accel X_L, X_H, Y_L, Y_H, Z_L, Z_H */
    int ret = lsm6dso_read_registers(config, LSM6DSO_REG_OUTX_L_G, raw, 12);
    if (ret < 0) {
        return ret;
    }

    parse_axis_3d(&raw[0], &data->gyro_x, &data->gyro_y, &data->gyro_z);
    parse_axis_3d(&raw[6], &data->accel_x, &data->accel_y, &data->accel_z);

    LOG_DBG("IMU: AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d",
            data->accel_x, data->accel_y, data->accel_z,
            data->gyro_x, data->gyro_y, data->gyro_z);
    return 0;
}

int lsm6dso_read_accel(lsm6dso_config_t *config, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t raw[6];

    int ret = lsm6dso_read_registers(config, LSM6DSO_REG_OUTX_L_A, raw, 6);
    if (ret < 0) {
        return ret;
    }

    parse_axis_3d(raw, x, y, z);
    return 0;
}

int lsm6dso_read_gyro(lsm6dso_config_t *config, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t raw[6];

    int ret = lsm6dso_read_registers(config, LSM6DSO_REG_OUTX_L_G, raw, 6);
    if (ret < 0) {
        return ret;
    }

    parse_axis_3d(raw, x, y, z);
    return 0;
}

int lsm6dso_set_accel_odr(lsm6dso_config_t *config, uint8_t odr) {
    if (odr > 0x0A) {
        LOG_ERR("Invalid accel ODR: 0x%02x", odr);
        return -EINVAL;
    }

    int ret = lsm6dso_write_register_masked(config, LSM6DSO_REG_CTRL1_XL, odr, LSM6DSO_MASK_XL_ODR);
    if (ret < 0) {
        LOG_ERR("Failed to set accel ODR: %d", ret);
        return ret;
    }

    LOG_INF("Accel ODR set to 0x%02x", odr);
    return 0;
}

int lsm6dso_set_fifo_mode(lsm6dso_config_t *config, uint8_t mode) {
    mode &= 0b111;
    if (mode == 0b010 || mode == 0b101) {
        LOG_ERR("Invalid FIFO mode: 0x%02x", mode);
        return -EINVAL;
    }

    int ret = lsm6dso_write_register_masked(config, LSM6DSO_REG_FIFO, mode, LSM6DSO_MASK_FIFO_MODE);
    if (ret < 0) {
        LOG_ERR("Failed to set FIFO mode: %d", ret);
        return ret;
    }

    LOG_INF("FIFO mode set to 0x%02x", mode);
    return 0;
}

int lsm6dso_set_accel_fs(lsm6dso_config_t *config, uint8_t fs) {
    int ret = lsm6dso_write_register_masked(config, LSM6DSO_REG_CTRL1_XL,
        fs, LSM6DSO_MASK_XL_FS);
    if (ret < 0) {
        LOG_ERR("Failed to set accel FS: %d", ret);
        return ret;
    }

    LOG_INF("Accel FS set to 0x%02x", fs);
    return 0;
}

int lsm6dso_set_gyro_odr(lsm6dso_config_t *config, uint8_t odr) {
    if (odr > 0x0A) {
        LOG_ERR("Invalid gyro ODR: 0x%02x", odr);
        return -EINVAL;
    }

    int ret = lsm6dso_write_register_masked(config, LSM6DSO_REG_CTRL2_G,
        odr, LSM6DSO_MASK_GY_ODR);
    if (ret < 0) {
        LOG_ERR("Failed to set gyro ODR: %d", ret);
        return ret;
    }

    LOG_INF("Gyro ODR set to 0x%02x", odr);
    return 0;
}

int lsm6dso_set_gyro_fs(lsm6dso_config_t *config, uint8_t fs) {
    int ret = lsm6dso_write_register_masked(config, LSM6DSO_REG_CTRL2_G,
        fs, LSM6DSO_MASK_GY_FS);
    if (ret < 0) {
        LOG_ERR("Failed to set gyro FS: %d", ret);
        return ret;
    }

    LOG_INF("Gyro FS set to 0x%02x", fs);
    return 0;
}

int lsm6dso_reset(lsm6dso_config_t *config) {
    int ret;

    ret = lsm6dso_write_register_masked(config, LSM6DSO_REG_CTRL3_C,
        1, LSM6DSO_CTRL3_C_SW_RESET);
    if (ret < 0) {
        LOG_ERR("Failed to send reset: %d", ret);
        return ret;
    }

    /* Wait for reset to complete (SW_RESET bit auto-clears) */
    k_busy_wait(10000);  /* 10ms */

    uint8_t ctrl3;
    ret = lsm6dso_read_register(config, LSM6DSO_REG_CTRL3_C, &ctrl3);
    if (ret < 0) {
        return ret;
    }

    if (ctrl3 & LSM6DSO_CTRL3_C_SW_RESET) {
        LOG_ERR("Reset did not complete (SW_RESET still set)");
        return -EIO;
    }

    LOG_INF("LSM6DSO reset complete");
    return 0;
}

int lsm6dso_is_data_ready(lsm6dso_config_t *config) {
    uint8_t status;

    int ret = lsm6dso_read_register(config, LSM6DSO_REG_STATUS_REG, &status);
    if (ret < 0) {
        return ret;
    }

    return status & (LSM6DSO_STATUS_XLDA | LSM6DSO_STATUS_GDA);
}

int lsm6dso_read_who_am_i(lsm6dso_config_t *config, uint8_t *id) {
    int ret = lsm6dso_read_register(config, LSM6DSO_REG_WHO_AM_I, id);
    if (ret < 0) {
        LOG_ERR("Failed to read WHO_AM_I: %d", ret);
        return ret;
    }

    LOG_INF("WHO_AM_I: 0x%02x", *id);
    return 0;
}

int lsm6dso_int1_is_active(lsm6dso_config_t *config) {
    if (config->int1_gpio.port == NULL) {
        LOG_WRN("INT1 GPIO not configured");
        return -ENOTSUP;
    }

    int val = gpio_pin_get_dt(&config->int1_gpio);
    if (val < 0) {
        LOG_ERR("Failed to read INT1 GPIO: %d", val);
        return val;
    }

    return val;
}

int lsm6dso_wait_for_int1(lsm6dso_config_t *config, uint32_t timeout_us) {
    if (config->int1_gpio.port == NULL) {
        LOG_WRN("INT1 GPIO not configured");
        return -ENOTSUP;
    }

    uint64_t start_cycles = k_cycle_get_64();
    uint64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);

    LOG_DBG("Waiting for INT1, timeout=%u us", timeout_us);

    while (!lsm6dso_int1_is_active(config)) {
        if ((k_cycle_get_64() - start_cycles) > timeout_cycles) {
            LOG_ERR("Timeout waiting for INT1");
            return -ETIMEDOUT;
        }
    }

    LOG_DBG("INT1 active");
    return 0;
}

/* ========== High-Level Configuration ========== */

/* Static IMU configuration - shares SPI21 with ADS131M02 */
#define LSM6DSO_CS_IDX          0   /* cs-gpios index in spi21 node */

static lsm6dso_config_t imu_config = {
    .spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi21)),
    .spi_cfg = {
        .frequency = 1000000U,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER |
                     SPI_MODE_CPHA | SPI_MODE_CPOL,  /* SPI Mode 3 */
        .cs = {
            .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi21), cs_gpios, LSM6DSO_CS_IDX),
            .delay = 2,
        },
    },
    .int1_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), int1_gpios),
    .int2_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), int2_gpios),
};

lsm6dso_config_t *lsm6dso_get_config(void) {
    return &imu_config;
}

int lsm6dso_full_setup(lsm6dso_config_t *config) {
    int err;
    uint8_t who_am_i;

    LOG_INF("Initializing LSM6DSO...\n");

    err = lsm6dso_init(config);
    if (err < 0) {
        LOG_ERR("Failed to initialize LSM6DSO (%d)\n", err);
        return err;
    }

    LOG_INF("Resetting LSM6DSO...\n");
    err = lsm6dso_reset(config);
    if (err < 0) {
        LOG_ERR("Failed to reset LSM6DSO (%d)\n", err);
        return err;
    }

    err = lsm6dso_read_who_am_i(config, &who_am_i);
    if (err < 0) {
        LOG_ERR("Failed to read WHO_AM_I (%d)\n", err);
        return err;
    }

    if (who_am_i != LSM6DSO_WHO_AM_I_VALUE && who_am_i != LSM6DSL_WHO_AM_I_VALUE) {
        LOG_ERR("Unexpected WHO_AM_I: 0x%02x (expected 0x%02x or 0x%02x)\n",
            who_am_i, LSM6DSO_WHO_AM_I_VALUE, LSM6DSL_WHO_AM_I_VALUE);
        return -ENODEV;
    }

    LOG_INF("Configuring LSM6DSO...\n");

    err = lsm6dso_set_fifo_mode(config, LSM6DSO_CTRL4_FIFO_BYPASS);
    if (err < 0) {
        LOG_ERR("Failed to set FIFO mode (%d)\n", err);
        return err;
    }

    err = lsm6dso_set_accel_odr(config, LSM6DSO_XL_ODR_208HZ);
    if (err < 0) {
        LOG_ERR("Failed to set accel ODR (%d)\n", err);
        return err;
    }

    err = lsm6dso_set_accel_fs(config, LSM6DSO_XL_FS_4G);
    if (err < 0) {
        LOG_ERR("Failed to set accel FS (%d)\n", err);
        return err;
    }

    err = lsm6dso_set_gyro_odr(config, LSM6DSO_GY_ODR_208HZ);
    if (err < 0) {
        LOG_ERR("Failed to set gyro ODR (%d)\n", err);
        return err;
    }

    err = lsm6dso_set_gyro_fs(config, LSM6DSO_GY_FS_500DPS);
    if (err < 0) {
        LOG_ERR("Failed to set gyro FS (%d)\n", err);
        return err;
    }

    LOG_INF("LSM6DSO setup complete (accel: 208Hz +/-4g, gyro: 208Hz +/-500dps)\n");
    return 0;
}

int lsm6dso_print_info(lsm6dso_config_t *config) {
    int err;
    uint8_t who_am_i, status, ctrl1, ctrl2, ctrl3, int1_ctrl;

    err = lsm6dso_read_who_am_i(config, &who_am_i);
    if (err < 0) { return err; }

    err = lsm6dso_read_register(config, LSM6DSO_REG_STATUS_REG, &status);
    if (err < 0) { return err; }

    err = lsm6dso_read_register(config, LSM6DSO_REG_CTRL1_XL, &ctrl1);
    if (err < 0) { return err; }

    err = lsm6dso_read_register(config, LSM6DSO_REG_CTRL2_G, &ctrl2);
    if (err < 0) { return err; }

    err = lsm6dso_read_register(config, LSM6DSO_REG_CTRL3_C, &ctrl3);
    if (err < 0) { return err; }

    err = lsm6dso_read_register(config, LSM6DSO_REG_INT1_CTRL, &int1_ctrl);
    if (err < 0) { return err; }

    LOG_INF("=================== LSM6DSO Info ===================\n");
    LOG_INF("WHO_AM_I:  0x%02x (%s)\n", who_am_i,
        who_am_i == LSM6DSO_WHO_AM_I_VALUE ? "LSM6DSO" :
        who_am_i == LSM6DSL_WHO_AM_I_VALUE ? "LSM6DSL" : "Unknown");
    LOG_INF("STATUS:    0x%02x (XL_DA=%d, G_DA=%d, T_DA=%d)\n", status,
        !!(status & LSM6DSO_STATUS_XLDA),
        !!(status & LSM6DSO_STATUS_GDA),
        !!(status & LSM6DSO_STATUS_TDA));
    LOG_INF("CTRL1_XL:  0x%02x (ODR=%d, FS=0x%02x)\n", ctrl1,
        (ctrl1 >> 4), (ctrl1 & 0x0C));
    LOG_INF("CTRL2_G:   0x%02x (ODR=%d, FS=0x%02x)\n", ctrl2,
        (ctrl2 >> 4), (ctrl2 & 0x0E));
    LOG_INF("CTRL3_C:   0x%02x (BDU=%d, IF_INC=%d)\n", ctrl3,
        !!(ctrl3 & LSM6DSO_CTRL3_C_BDU),
        !!(ctrl3 & LSM6DSO_CTRL3_C_IF_INC));
    LOG_INF("INT1_CTRL: 0x%02x (DRDY_XL=%d, DRDY_G=%d)\n", int1_ctrl,
        !!(int1_ctrl & LSM6DSO_INT1_DRDY_XL),
        !!(int1_ctrl & LSM6DSO_INT1_DRDY_G));
    LOG_INF("====================================================\n");

    return 0;
}
