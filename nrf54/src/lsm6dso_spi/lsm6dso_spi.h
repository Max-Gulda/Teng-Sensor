/**
 * @file lsm6dso_spi.h
 * @brief LSM6DSO/LSM6DSL 6-axis IMU SPI driver for Zephyr RTOS
 *
 * This driver provides SPI-based communication with the ST LSM6DSO/LSM6DSL
 * accelerometer + gyroscope IMU for nRF54L15.
 * Only accelerometer and gyroscope data are supported.
 */

#ifndef LSM6DSO_SPI_H
#define LSM6DSO_SPI_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/* WHO_AM_I expected values */
#define LSM6DSO_WHO_AM_I_VALUE  0x6C
#define LSM6DSL_WHO_AM_I_VALUE  0x6A

/* IMU output data structure */
typedef struct {
    int16_t accel_x;    /* Accelerometer X-axis [raw LSB] */
    int16_t accel_y;    /* Accelerometer Y-axis */
    int16_t accel_z;    /* Accelerometer Z-axis */
    int16_t gyro_x;     /* Gyroscope X-axis [raw LSB] */
    int16_t gyro_y;     /* Gyroscope Y-axis */
    int16_t gyro_z;     /* Gyroscope Z-axis */
} lsm6dso_data_t;

/* Device configuration structure */
typedef struct {
    const struct device *spi_dev;
    struct spi_config spi_cfg;
    struct gpio_dt_spec int1_gpio;
    struct gpio_dt_spec int2_gpio;
} lsm6dso_config_t;

/* ========== Register Map ========== */

/* Identification */
#define LSM6DSO_REG_WHO_AM_I         0x0F

/* Interrupt configuration */
#define LSM6DSO_REG_INT1_CTRL        0x0D
#define LSM6DSO_REG_INT2_CTRL        0x0E

/* FIFO configuration */
#define LSM6DSO_REG_FIFO             0x0A

/* Control registers */
#define LSM6DSO_REG_CTRL1_XL         0x10  /* Accelerometer ODR + FS */
#define LSM6DSO_REG_CTRL2_G          0x11  /* Gyroscope ODR + FS */
#define LSM6DSO_REG_CTRL3_C          0x12  /* BDU, IF_INC, reset, etc */
#define LSM6DSO_REG_CTRL4_C          0x13
#define LSM6DSO_REG_CTRL5_C          0x14
#define LSM6DSO_REG_CTRL6_C          0x15
#define LSM6DSO_REG_CTRL7_G          0x16
#define LSM6DSO_REG_CTRL8_XL         0x17

/* Status */
#define LSM6DSO_REG_STATUS_REG       0x1E

/* Output registers (gyroscope then accelerometer, contiguous for burst read) */
#define LSM6DSO_REG_OUTX_L_G         0x22
#define LSM6DSO_REG_OUTX_H_G         0x23
#define LSM6DSO_REG_OUTY_L_G         0x24
#define LSM6DSO_REG_OUTY_H_G         0x25
#define LSM6DSO_REG_OUTZ_L_G         0x26
#define LSM6DSO_REG_OUTZ_H_G         0x27
#define LSM6DSO_REG_OUTX_L_A         0x28
#define LSM6DSO_REG_OUTX_H_A         0x29
#define LSM6DSO_REG_OUTY_L_A         0x2A
#define LSM6DSO_REG_OUTY_H_A         0x2B
#define LSM6DSO_REG_OUTZ_L_A         0x2C
#define LSM6DSO_REG_OUTZ_H_A         0x2D

/* ========== Register Bit Definitions ========== */

/* CTRL3_C */
#define LSM6DSO_CTRL3_C_BOOT       0x80
#define LSM6DSO_CTRL3_C_BDU        0x40 /* Block Data Update */
#define LSM6DSO_CTRL3_C_H_LACTIVE  0x20  /* Interrupt active-low */
#define LSM6DSO_CTRL3_C_PP_OD      0x10  /* Push-pull / Open-drain */
#define LSM6DSO_CTRL3_C_SIM        0x08  /* SPI 3-wire mode */
#define LSM6DSO_CTRL3_C_IF_INC     0x04  /* Auto-increment address */
#define LSM6DSO_CTRL3_C_SW_RESET   0x01  /* Software reset */

#define LSM6DSO_CTRL4_FIFO_BYPASS               0b000
#define LSM6DSO_CTRL4_FIFO_FIFO                 0b001
#define LSM6DSO_CTRL4_FIFO_CONTINUOUS_TO_FIFO   0b011
#define LSM6DSO_CTRL4_FIFO_BYPASS_TO_CONTINUOUS 0b100
#define LSM6DSO_CTRL4_FIFO_CONTINUOUS           0b110
#define LSM6DSO_CTRL4_FIFO_BYPASS_TO_FIFO       0b111



/* STATUS_REG */
#define LSM6DSO_STATUS_XLDA         0x01  /* Accelerometer new data available */
#define LSM6DSO_STATUS_GDA          0x02  /* Gyroscope new data available */
#define LSM6DSO_STATUS_TDA          0x04  /* Temperature new data available */

/* INT1_CTRL */
#define LSM6DSO_INT1_DRDY_XL        0x01  /* Accel data-ready on INT1 */
#define LSM6DSO_INT1_DRDY_G         0x02  /* Gyro data-ready on INT1 */

/* INT2_CTRL */
#define LSM6DSO_INT2_DRDY_XL        0x01  /* Accel data-ready on INT2 */
#define LSM6DSO_INT2_DRDY_G         0x02  /* Gyro data-ready on INT2 */

/* ========== Accelerometer ODR (Output Data Rate) ========== */
/* Values to pass to lsm6dso_set_accel_odr() - shifted internally */
#define LSM6DSO_XL_ODR_OFF          0x00
#define LSM6DSO_XL_ODR_12_5HZ      0x01
#define LSM6DSO_XL_ODR_26HZ        0x02
#define LSM6DSO_XL_ODR_52HZ        0x03
#define LSM6DSO_XL_ODR_104HZ       0x04
#define LSM6DSO_XL_ODR_208HZ       0x05
#define LSM6DSO_XL_ODR_416HZ       0x06
#define LSM6DSO_XL_ODR_833HZ       0x07
#define LSM6DSO_XL_ODR_1666HZ      0x08
#define LSM6DSO_XL_ODR_3332HZ      0x09
#define LSM6DSO_XL_ODR_6664HZ      0x0A

/* ========== Accelerometer Full Scale ========== */
/* Raw index values for bits [3:2] of CTRL1_XL (auto-shifted by write_register_masked) */
#define LSM6DSO_XL_FS_2G            0  /* bits[3:2] = 00 → ±2g  */
#define LSM6DSO_XL_FS_16G           1  /* bits[3:2] = 01 → ±16g */
#define LSM6DSO_XL_FS_4G            2  /* bits[3:2] = 10 → ±4g  */
#define LSM6DSO_XL_FS_8G            3  /* bits[3:2] = 11 → ±8g  */

/* ========== Gyroscope ODR ========== */
/* Values to pass to lsm6dso_set_gyro_odr() - shifted internally */
#define LSM6DSO_GY_ODR_OFF          0x00
#define LSM6DSO_GY_ODR_12_5HZ      0x01
#define LSM6DSO_GY_ODR_26HZ        0x02
#define LSM6DSO_GY_ODR_52HZ        0x03
#define LSM6DSO_GY_ODR_104HZ       0x04
#define LSM6DSO_GY_ODR_208HZ       0x05
#define LSM6DSO_GY_ODR_416HZ       0x06
#define LSM6DSO_GY_ODR_833HZ       0x07
#define LSM6DSO_GY_ODR_1666HZ      0x08
#define LSM6DSO_GY_ODR_3332HZ      0x09
#define LSM6DSO_GY_ODR_6664HZ      0x0A

/* ========== Gyroscope Full Scale ========== */
/* Raw index values for bits [3:1] of CTRL2_G (auto-shifted by write_register_masked) */
#define LSM6DSO_GY_FS_250DPS       0  /* bits[3:1] = 000 → ±250 dps  */
#define LSM6DSO_GY_FS_125DPS       1  /* bits[3:1] = 001 → ±125 dps  */
#define LSM6DSO_GY_FS_500DPS       2  /* bits[3:1] = 010 → ±500 dps  */
#define LSM6DSO_GY_FS_1000DPS      4  /* bits[3:1] = 100 → ±1000 dps */
#define LSM6DSO_GY_FS_2000DPS      6  /* bits[3:1] = 110 → ±2000 dps */

/* ========== Register Masks ========== */
#define LSM6DSO_MASK_XL_ODR         0xF0  /* CTRL1_XL bits [7:4] */
#define LSM6DSO_MASK_XL_FS          0x0C  /* CTRL1_XL bits [3:2] */
#define LSM6DSO_MASK_GY_ODR         0xF0  /* CTRL2_G bits [7:4] */
#define LSM6DSO_MASK_GY_FS          0x0E  /* CTRL2_G bits [3:1] */
#define LSM6DSO_MASK_FIFO_MODE      0x07 /* CTRL4_FIFO bits [2:0] */

/* ========== API Functions ========== */

/**
 * @brief Get the default IMU configuration
 *
 * Returns a pointer to the static IMU configuration structure
 * with pre-configured GPIO pins and SPI settings.
 * CS=P1.04, INT1=P1.05, INT2=P1.06, SPI21 @ 8 MHz Mode 3
 *
 * @return Pointer to lsm6dso_config_t structure
 */
lsm6dso_config_t *lsm6dso_get_config(void);

/**
 * @brief Initialize LSM6DSO device (configure GPIOs)
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_init(lsm6dso_config_t *config);

/**
 * @brief Perform software reset
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_reset(lsm6dso_config_t *config);

/**
 * @brief Read a single register
 * @param config Pointer to device configuration
 * @param reg Register address
 * @param value Pointer to store register value
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_read_register(lsm6dso_config_t *config, uint8_t reg, uint8_t *value);

/**
 * @brief Write a single register
 * @param config Pointer to device configuration
 * @param reg Register address
 * @param value Value to write
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_write_register(lsm6dso_config_t *config, uint8_t reg, uint8_t value);

/**
 * @brief Read multiple consecutive registers (burst read)
 *
 * Requires IF_INC bit set in CTRL3_C (set by lsm6dso_full_setup).
 *
 * @param config Pointer to device configuration
 * @param start_reg Starting register address
 * @param buf Buffer to store values
 * @param len Number of bytes to read (max 14)
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_read_registers(lsm6dso_config_t *config, uint8_t start_reg,
    uint8_t *buf, size_t len);

/**
 * @brief Write a masked value to a register (read-modify-write)
 * @param config Pointer to device configuration
 * @param reg Register address
 * @param value Value to write (already shifted to mask position)
 * @param mask Bit mask for the field
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_write_register_masked(lsm6dso_config_t *config, uint8_t reg,
    uint8_t value, uint8_t mask);

/**
 * @brief Read accelerometer and gyroscope data (burst read of all 6 axes)
 *
 * Reads gyro X/Y/Z and accel X/Y/Z in a single 12-byte burst.
 *
 * @param config Pointer to device configuration
 * @param data Pointer to structure to store readings
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_read_data(lsm6dso_config_t *config, lsm6dso_data_t *data);

/**
 * @brief Read accelerometer data only (3 axes)
 * @param config Pointer to device configuration
 * @param x Pointer for X-axis value
 * @param y Pointer for Y-axis value
 * @param z Pointer for Z-axis value
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_read_accel(lsm6dso_config_t *config, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Read gyroscope data only (3 axes)
 * @param config Pointer to device configuration
 * @param x Pointer for X-axis value
 * @param y Pointer for Y-axis value
 * @param z Pointer for Z-axis value
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_read_gyro(lsm6dso_config_t *config, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Set accelerometer output data rate
 * @param config Pointer to device configuration
 * @param odr ODR value (use LSM6DSO_XL_ODR_* defines)
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_set_accel_odr(lsm6dso_config_t *config, uint8_t odr);

/**
 * @brief Set accelerometer full scale range
 * @param config Pointer to device configuration
 * @param fs Full scale value (use LSM6DSO_XL_FS_* defines)
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_set_accel_fs(lsm6dso_config_t *config, uint8_t fs);

/**
 * @brief Set gyroscope output data rate
 * @param config Pointer to device configuration
 * @param odr ODR value (use LSM6DSO_GY_ODR_* defines)
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_set_gyro_odr(lsm6dso_config_t *config, uint8_t odr);

/**
 * @brief Set gyroscope full scale range
 * @param config Pointer to device configuration
 * @param fs Full scale value (use LSM6DSO_GY_FS_* defines)
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_set_gyro_fs(lsm6dso_config_t *config, uint8_t fs);

/**
 * @brief Check if new data is available
 * @param config Pointer to device configuration
 * @return Bitmask of LSM6DSO_STATUS_XLDA | LSM6DSO_STATUS_GDA, negative on error
 */
int lsm6dso_is_data_ready(lsm6dso_config_t *config);

/**
 * @brief Read WHO_AM_I register
 * @param config Pointer to device configuration
 * @param id Pointer to store WHO_AM_I value
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_read_who_am_i(lsm6dso_config_t *config, uint8_t *id);

/**
 * @brief Check if INT1 pin is active (data ready)
 * @param config Pointer to device configuration
 * @return 1 if active, 0 if inactive, negative on error
 */
int lsm6dso_int1_is_active(lsm6dso_config_t *config);

/**
 * @brief Wait for INT1 pin to go active (busy-wait polling)
 * @param config Pointer to device configuration
 * @param timeout_us Timeout in microseconds
 * @return 0 if INT1 became active, -ETIMEDOUT on timeout
 */
int lsm6dso_wait_for_int1(lsm6dso_config_t *config, uint32_t timeout_us);

/**
 * @brief Complete IMU setup with default configuration
 *
 * Performs the full initialization sequence:
 * - GPIO initialization
 * - Software reset
 * - Verify WHO_AM_I (accepts LSM6DSO 0x6C or LSM6DSL 0x6A)
 * - Enable BDU (Block Data Update) and IF_INC (auto-increment)
 * - Set accelerometer to 104 Hz, +/-4g
 * - Set gyroscope to 104 Hz, +/-500 dps
 * - Enable data-ready interrupt on INT1 (accel + gyro)
 *
 * @note This device shares SPI21 with ADS131M02 (different CS pins).
 *       Ensure SPI transactions are not concurrent (e.g. read from same thread).
 *
 * @param config Pointer to device configuration (use lsm6dso_get_config())
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_full_setup(lsm6dso_config_t *config);

/**
 * @brief Print device ID and configuration information
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int lsm6dso_print_info(lsm6dso_config_t *config);

#endif /* LSM6DSO_SPI_H */
