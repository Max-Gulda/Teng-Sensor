/**
 * @file ads131m02_spi.h
 * @brief ADS131M02 24-bit ADC SPI driver for Zephyr RTOS
 *
 * This driver provides SPI-based communication with the TI ADS131M02
 * dual-channel simultaneous-sampling ADC for nRF54L15.
 */

#ifndef ADS131M02_SPI_H
#define ADS131M02_SPI_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

 /* Number of channels for ADS131M02 */
#define ADS131M02_NUM_CHANNELS 2

/* SPI frame length calculation */
#define ADS131M02_MSG_LEN_ONE (3 * (ADS131M02_NUM_CHANNELS + 2))  /* 12 bytes */
#define ADS131M02_MSG_LEN_TWO (ADS131M02_MSG_LEN_ONE * 2)          /* 24 bytes */

/* ADC output structure */
typedef struct {
    int32_t ch0;
    int32_t ch1;
} ads131m02_data_t;

/* Device configuration structure */
typedef struct {
    const struct device *spi_dev;
    struct spi_config spi_cfg;
    struct gpio_dt_spec drdy_gpio;
    struct gpio_dt_spec reset_gpio;
} ads131m02_config_t;

/* Commands */
#define ADS131M02_CMD_NULL        0x0000
#define ADS131M02_CMD_RESET       0x0011
#define ADS131M02_CMD_STANDBY     0x0022
#define ADS131M02_CMD_WAKEUP      0x0033
#define ADS131M02_CMD_LOCK        0x0555
#define ADS131M02_CMD_UNLOCK      0x0655
#define ADS131M02_CMD_READ_REG    0xa000  /* 101a aaaa annn nnnn */
#define ADS131M02_CMD_WRITE_REG   0x6000  /* 011a aaaa annn nnnn */

/* Command responses */
#define ADS131M02_RESP_RESET      0xff22
#define ADS131M02_RESP_STANDBY    0x0022
#define ADS131M02_RESP_WAKEUP     0x0033
#define ADS131M02_RESP_LOCK       0x0555
#define ADS131M02_RESP_UNLOCK     0x0655

/* Registers - Read Only */
#define ADS131M02_REG_ID          0x00
#define ADS131M02_REG_STATUS      0x01

/* Global settings registers */
#define ADS131M02_REG_MODE        0x02
#define ADS131M02_REG_CLOCK       0x03
#define ADS131M02_REG_GAIN        0x04
#define ADS131M02_REG_CFG         0x06
#define ADS131M02_REG_THRSHLD_MSB 0x07
#define ADS131M02_REG_THRSHLD_LSB 0x08

/* Channel 0 registers */
#define ADS131M02_REG_CH0_CFG     0x09
#define ADS131M02_REG_CH0_OCAL_MSB 0x0A
#define ADS131M02_REG_CH0_OCAL_LSB 0x0B
#define ADS131M02_REG_CH0_GCAL_MSB 0x0C
#define ADS131M02_REG_CH0_GCAL_LSB 0x0D

/* Channel 1 registers */
#define ADS131M02_REG_CH1_CFG     0x0E
#define ADS131M02_REG_CH1_OCAL_MSB 0x0F
#define ADS131M02_REG_CH1_OCAL_LSB 0x10
#define ADS131M02_REG_CH1_GCAL_MSB 0x11
#define ADS131M02_REG_CH1_GCAL_LSB 0x12

/* Register MAP CRC */
#define ADS131M02_REG_MAP_CRC     0x3E

/* Register masks - STATUS */
#define ADS131M02_REGMASK_STATUS_LOCK    0x8000
#define ADS131M02_REGMASK_STATUS_RESYNC  0x4000
#define ADS131M02_REGMASK_STATUS_REGMAP  0x2000
#define ADS131M02_REGMASK_STATUS_CRC_ERR 0x1000
#define ADS131M02_REGMASK_STATUS_RESET   0x0400
#define ADS131M02_REGMASK_STATUS_DRDY1   0x0002
#define ADS131M02_REGMASK_STATUS_DRDY0   0x0001

/* Register masks - CLOCK */
#define ADS131M02_REGMASK_CLOCK_CH1_EN   0x0200
#define ADS131M02_REGMASK_CLOCK_CH0_EN   0x0100
#define ADS131M02_REGMASK_CLOCK_OSR      0x001C
#define ADS131M02_REGMASK_CLOCK_PWR      0x0003

/* Oversampling ratio options */
#define ADS131M02_OSR_128    0b000  /* 32 kSPS */
#define ADS131M02_OSR_256    0b001  /* 16 kSPS */
#define ADS131M02_OSR_512    0b010  /* 8 kSPS */
#define ADS131M02_OSR_1024   0b011  /* 4 kSPS (default) */
#define ADS131M02_OSR_2048   0b100  /* 2 kSPS */
#define ADS131M02_OSR_4096   0b101  /* 1 kSPS */
#define ADS131M02_OSR_8192   0b110  /* 500 SPS */
#define ADS131M02_OSR_16384  0b111  /* 250 SPS */

/* Power modes */
#define ADS131M02_PWR_VERY_LOW_POWER  0
#define ADS131M02_PWR_LOW_POWER       1
#define ADS131M02_PWR_HIGH_RESOLUTION 2  /* Default */

/* PGA Gain settings */
#define ADS131M02_GAIN_1    0b000
#define ADS131M02_GAIN_2    0b001
#define ADS131M02_GAIN_4    0b010
#define ADS131M02_GAIN_8    0b011
#define ADS131M02_GAIN_16   0b100
#define ADS131M02_GAIN_32   0b101
#define ADS131M02_GAIN_64   0b110
#define ADS131M02_GAIN_128  0b111

/* Register masks - GAIN */
#define ADS131M02_REGMASK_GAIN_PGAGAIN1  0x0070
#define ADS131M02_REGMASK_GAIN_PGAGAIN0  0x0007

/* Input channel mux options */
#define ADS131M02_MUX_AIN_DIFFERENTIAL   0  /* Default: AIN0P-AIN0N */
#define ADS131M02_MUX_INPUT_SHORTED      1
#define ADS131M02_MUX_POSITIVE_DC_TEST   2
#define ADS131M02_MUX_NEGATIVE_DC_TEST   3

// Mask Register THRSHLD_LSB
#define ADS131M02_REGMASK_THRSHLD_LSB_CD_TH_LSB 0xFF00
#define ADS131M02_REGMASK_THRSHLD_LSB_DCBLOCK 0x000F

/* Register masks - CHx_CFG */
#define ADS131M02_REGMASK_CHX_CFG_MUX    0x0003
#define ADS131M02_REGMASK_CHX_CFG_PHASE 0xFFC0
#define ADS131M02_REGMASK_CHX_CFG_DCBLKX_DIS0 0x0004

/*  Mask Register CHX_OCAL_LSB */
#define ADS131M02_REGMASK_CHX_OCAL0_LSB 0xFF00

/*  Mask Register CHX_GCAL_LSB */
#define ADS131M02_REGMASK_CHX_GCAL0_LSB 0xFF00

/**
 * @brief Initialize ADS131M02 device
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int ads131m02_init(ads131m02_config_t *config);

/**
 * @brief Perform hardware reset of ADS131M02
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int ads131m02_reset(ads131m02_config_t *config);

/**
 * @brief Send a command to the ADS131M02
 * @param config Pointer to device configuration
 * @param cmd Command word to send
 * @param response Pointer to store response word
 * @return 0 on success, negative error code on failure
 */
int ads131m02_send_command(ads131m02_config_t *config, uint16_t cmd, uint16_t *response);

/**
 * @brief Read a register from ADS131M02
 * @param config Pointer to device configuration
 * @param address Register address (0x00-0x3E)
 * @param value Pointer to store register value
 * @return 0 on success, negative error code on failure
 */
int ads131m02_read_register(ads131m02_config_t *config, uint8_t address, uint16_t *value);

/**
 * @brief Write a register to ADS131M02
 * @param config Pointer to device configuration
 * @param address Register address (0x00-0x3E)
 * @param value Value to write
 * @return 0 on success, negative error code on failure
 */
int ads131m02_write_register(ads131m02_config_t *config, uint8_t address, uint16_t value);

/**
 * @brief Write a masked value to a register
 * @param config Pointer to device configuration
 * @param address Register address
 * @param value Value to write (will be shifted according to mask)
 * @param mask Bit mask for the field
 * @return 0 on success, negative error code on failure
 */
int ads131m02_write_register_masked(ads131m02_config_t *config, uint8_t address,
    uint16_t value, uint16_t mask);

/**
 * @brief Read ADC data from all channels
 * @param config Pointer to device configuration
 * @param data Pointer to structure to store ADC readings
 * @return 0 on success, negative error code on failure
 */
int ads131m02_read_adc(ads131m02_config_t *config, ads131m02_data_t *data);

/**
 * @brief Read ADC data from all channels after discarding buffered values
 * @param config Pointer to device configuration
 * @param data Pointer to structure to store ADC readings
 * @return 0 on success, negative error code on failure
 */
int ads131m02_read_adc_after_pause(ads131m02_config_t *config, ads131m02_data_t *data);

/**
 * @brief Set the oversampling ratio
 * @param config Pointer to device configuration
 * @param osr_value OSR setting (use ADS131M02_OSR_* defines)
 * @return 0 on success, negative error code on failure
 */
int ads131m02_set_oversampling(ads131m02_config_t *config, uint8_t osr_value);

/**
 * @brief Set the power mode
 * @param config Pointer to device configuration
 * @param power_mode Power mode (use ADS131M02_PWR_* defines)
 * @return 0 on success, negative error code on failure
 */
int ads131m02_set_power_mode(ads131m02_config_t *config, uint8_t power_mode);

/**
 * @brief Set channel gain
 * @param config Pointer to device configuration
 * @param channel Channel number (0 or 1)
 * @param gain Gain setting (use ADS131M02_GAIN_* defines)
 * @return 0 on success, negative error code on failure
 */
int ads131m02_set_channel_gain(ads131m02_config_t *config, uint8_t channel, uint8_t gain);

/**
 * @brief Set channel input mux
 * @param config Pointer to device configuration
 * @param channel Channel number (0 or 1)
 * @param mux Mux setting (use ADS131M02_MUX_* defines)
 * @return 0 on success, negative error code on failure
 */
int ads131m02_set_channel_mux(ads131m02_config_t *config, uint8_t channel, uint8_t mux);

/**
 * @brief Enable/disable ADC channels
 * @param config Pointer to device configuration
 * @param ch0_enable Enable channel 0 (true/false)
 * @param ch1_enable Enable channel 1 (true/false)
 * @return 0 on success, negative error code on failure
 */
int ads131m02_enable_channels(ads131m02_config_t *config, bool ch0_enable, bool ch1_enable);

/**
 * @brief Assert hardware reset pin (pull low)
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int ads131m02_reset_assert(ads131m02_config_t *config);

/**
 * @brief Release hardware reset pin (pull high)
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int ads131m02_reset_release(ads131m02_config_t *config);

/**
 * @brief Check if DRDY pin is low
 * @param config Pointer to device configuration
 * @return 1 if DRDY is low, 0 if high, negative on error
 */
int ads131m02_drdy_is_low(ads131m02_config_t *config);

/**
 * @brief Wait for DRDY pin to go low (falling edge)
 * @param config Pointer to device configuration
 * @param timeout_us Timeout in microseconds
 * @return 0 if DRDY went low, -ETIMEDOUT on timeout, negative on other errors
 */
int ads131m02_wait_for_drdy_falling(ads131m02_config_t *config, uint32_t timeout_us);

/**
 * @brief Perform hardware reset and read ADC once when data is ready
 *
 * This function performs a hardware reset pulse, waits for the ADC to
 * become ready (DRDY falling edge), and reads one sample.
 *
 * @param config Pointer to device configuration
 * @param data Pointer to structure to store ADC readings
 * @param timeout_us Timeout in microseconds to wait for DRDY
 * @return 0 on success, negative error code on failure
 */
int ads131m02_reset_and_read_once(ads131m02_config_t *config, ads131m02_data_t *data,
    uint32_t timeout_us);

/**
 * @brief Perform hardware reset using reset pin
 *
 * This function performs a hardware reset by pulsing the reset pin low.
 * It's different from ads131m02_reset() which sends a reset command via SPI.
 *
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int ads131m02_hardware_reset(ads131m02_config_t *config);

/**
 * @brief Read the STATUS register
 * @param config Pointer to device configuration
 * @param status Pointer to store status value
 * @return 0 on success, negative error code on failure
 */
int ads131m02_read_status(ads131m02_config_t *config, uint16_t *status);

/**
 * @brief Print decoded STATUS register information
 * @param status Status register value to decode and print
 */
void ads131m02_print_status(uint16_t status);

/**
 * @brief Read ID register
 * @param config Pointer to device configuration
 * @param id Pointer to store ID value
 * @return 0 on success, negative error code on failure
 */
int ads131m02_read_id(ads131m02_config_t *config, uint16_t *id);

/**
 * @brief Calibrate ADC
 * @param config Pointer to device configuration
 * @return void
 */
void ads131m02_calibrate(ads131m02_config_t *config);

/**
 * @brief Initialize GRTC clock for ADC external clock
 *
 * Waits for the GRTC 8.192 MHz clock output on P1.12 to stabilize.
 * The clock is automatically configured by Zephyr via devicetree:
 * - Frequency: 8.192 MHz (clkout-fast-frequency-hz)
 * - Output pin: P1.12 (GRTC_CLKOUT_FAST)
 *
 * @note The clock is started automatically by the kernel, this function
 *       only provides a stabilization delay.
 *
 * @return 0 on success, negative error code on failure
 */
int ads131m02_pwm_init(void);

/**
 * @brief Get the default ADC configuration
 *
 * Returns a pointer to the static ADC configuration structure
 * with pre-configured GPIO pins and SPI settings.
 *
 * @return Pointer to ads131m02_config_t structure
 */
ads131m02_config_t *ads131m02_get_config(void);

/**
 * @brief Complete ADC setup and configuration
 *
 * Performs the full initialization sequence:
 * - Initialize driver (GPIO pins)
 * - Hardware reset
 * - Set power mode to high resolution
 * - Set oversampling ratio to OSR 128 (32 kSPS)
 * - Set both channels to 1x gain
 * - Set both channels to differential input mode
 * - Enable both channels
 * - Perform calibration
 *
 * @note PWM clock must be initialized and running before calling this
 * @param config Pointer to device configuration (use ads131m02_get_config())
 * @return 0 on success, negative error code on failure
 */
int ads131m02_full_setup(ads131m02_config_t *config);

/**
 * @brief Print ADC ID and status information
 *
 * Reads and prints the ID register and decoded status register
 * for debugging and verification.
 *
 * @param config Pointer to device configuration
 * @return 0 on success, negative error code on failure
 */
int ads131m02_print_info(ads131m02_config_t *config);

/**
 * @brief Set DC blocking filter for a channel
 * @param config Pointer to device configuration
 * @param ch Channel number (0 or 1)
 * @param onOff 1 to enable DC blocking, 0 to disable
 */
void ads131m02_set_channel_dcblock(ads131m02_config_t *config, uint8_t ch, uint32_t onOff);

/**
 * @brief Set high-pass filter cutoff frequency
 * @param config Pointer to device configuration
 * @param value DCBLOCK field value for THRSHLD_LSB register
 */
void ads131m02_set_high_pass(ads131m02_config_t *config, uint8_t value);

#endif /* ADS131M02_SPI_H */
