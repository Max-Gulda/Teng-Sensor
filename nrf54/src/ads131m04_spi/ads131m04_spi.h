/**
 * @file ads131m04_spi.h
 * @brief ADS131M04 24-bit ADC SPI driver for Zephyr RTOS
 *
 * This driver provides SPI-based communication with the TI ADS131M04
 * four-channel simultaneous-sampling ADC for nRF54L15.
 */

#ifndef ADS131M04_SPI_H
#define ADS131M04_SPI_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/* Number of channels for ADS131M04 */
#define ADS131M04_NUM_CHANNELS 4

/* SPI frame length calculation */
#define ADS131M04_WORD_LEN_BYTES 3
#define ADS131M04_MSG_LEN_ONE (ADS131M04_WORD_LEN_BYTES * (ADS131M04_NUM_CHANNELS + 2))
#define ADS131M04_MSG_LEN_TWO (ADS131M04_MSG_LEN_ONE * 2)

/* ADC output structure */
typedef struct {
    int32_t ch0;
    int32_t ch1;
    int32_t ch2;
    int32_t ch3;
} ads131m04_data_t;

/* Device configuration structure */
typedef struct {
    const struct device *spi_dev;
    struct spi_config spi_cfg;
    struct gpio_dt_spec drdy_gpio;
    struct gpio_dt_spec reset_gpio;
} ads131m04_config_t;

/* Commands */
#define ADS131M04_CMD_NULL        0x0000
#define ADS131M04_CMD_RESET       0x0011
#define ADS131M04_CMD_STANDBY     0x0022
#define ADS131M04_CMD_WAKEUP      0x0033
#define ADS131M04_CMD_LOCK        0x0555
#define ADS131M04_CMD_UNLOCK      0x0655
#define ADS131M04_CMD_READ_REG    0xA000  /* 101a aaaa annn nnnn */
#define ADS131M04_CMD_WRITE_REG   0x6000  /* 011a aaaa annn nnnn */

/* Command responses */
#define ADS131M04_RESP_RESET      0xFF22
#define ADS131M04_RESP_STANDBY    0x0022
#define ADS131M04_RESP_WAKEUP     0x0033
#define ADS131M04_RESP_LOCK       0x0555
#define ADS131M04_RESP_UNLOCK     0x0655

/* Registers - Read Only */
#define ADS131M04_REG_ID          0x00
#define ADS131M04_REG_STATUS      0x01

/* Global settings registers */
#define ADS131M04_REG_MODE        0x02
#define ADS131M04_REG_CLOCK       0x03
#define ADS131M04_REG_GAIN        0x04
#define ADS131M04_REG_CFG         0x06
#define ADS131M04_REG_THRSHLD_MSB 0x07
#define ADS131M04_REG_THRSHLD_LSB 0x08

/* Channel 0 registers */
#define ADS131M04_REG_CH0_CFG      0x09
#define ADS131M04_REG_CH0_OCAL_MSB 0x0A
#define ADS131M04_REG_CH0_OCAL_LSB 0x0B
#define ADS131M04_REG_CH0_GCAL_MSB 0x0C
#define ADS131M04_REG_CH0_GCAL_LSB 0x0D

/* Channel 1 registers */
#define ADS131M04_REG_CH1_CFG      0x0E
#define ADS131M04_REG_CH1_OCAL_MSB 0x0F
#define ADS131M04_REG_CH1_OCAL_LSB 0x10
#define ADS131M04_REG_CH1_GCAL_MSB 0x11
#define ADS131M04_REG_CH1_GCAL_LSB 0x12

/* Channel 2 registers */
#define ADS131M04_REG_CH2_CFG      0x13
#define ADS131M04_REG_CH2_OCAL_MSB 0x14
#define ADS131M04_REG_CH2_OCAL_LSB 0x15
#define ADS131M04_REG_CH2_GCAL_MSB 0x16
#define ADS131M04_REG_CH2_GCAL_LSB 0x17

/* Channel 3 registers */
#define ADS131M04_REG_CH3_CFG      0x18
#define ADS131M04_REG_CH3_OCAL_MSB 0x19
#define ADS131M04_REG_CH3_OCAL_LSB 0x1A
#define ADS131M04_REG_CH3_GCAL_MSB 0x1B
#define ADS131M04_REG_CH3_GCAL_LSB 0x1C

/* Register MAP CRC */
#define ADS131M04_REG_MAP_CRC     0x3E

/* Register masks - STATUS */
#define ADS131M04_REGMASK_STATUS_LOCK    0x8000
#define ADS131M04_REGMASK_STATUS_RESYNC  0x4000
#define ADS131M04_REGMASK_STATUS_REGMAP  0x2000
#define ADS131M04_REGMASK_STATUS_CRC_ERR 0x1000
#define ADS131M04_REGMASK_STATUS_RESET   0x0400
#define ADS131M04_REGMASK_STATUS_DRDY3   0x0008
#define ADS131M04_REGMASK_STATUS_DRDY2   0x0004
#define ADS131M04_REGMASK_STATUS_DRDY1   0x0002
#define ADS131M04_REGMASK_STATUS_DRDY0   0x0001

/* Register masks - CLOCK */
#define ADS131M04_REGMASK_CLOCK_CH3_EN   0x0800
#define ADS131M04_REGMASK_CLOCK_CH2_EN   0x0400
#define ADS131M04_REGMASK_CLOCK_CH1_EN   0x0200
#define ADS131M04_REGMASK_CLOCK_CH0_EN   0x0100
#define ADS131M04_REGMASK_CLOCK_TBM      0x0020
#define ADS131M04_REGMASK_CLOCK_OSR      0x001C
#define ADS131M04_REGMASK_CLOCK_PWR      0x0003

/* Oversampling ratio options */
#define ADS131M04_OSR_128    0b000  /* 32 kSPS */
#define ADS131M04_OSR_256    0b001  /* 16 kSPS */
#define ADS131M04_OSR_512    0b010  /* 8 kSPS */
#define ADS131M04_OSR_1024   0b011  /* 4 kSPS (default) */
#define ADS131M04_OSR_2048   0b100  /* 2 kSPS */
#define ADS131M04_OSR_4096   0b101  /* 1 kSPS */
#define ADS131M04_OSR_8192   0b110  /* 500 SPS */
#define ADS131M04_OSR_16384  0b111  /* 250 SPS */

/* Power modes */
#define ADS131M04_PWR_VERY_LOW_POWER  0
#define ADS131M04_PWR_LOW_POWER       1
#define ADS131M04_PWR_HIGH_RESOLUTION 2

/* PGA Gain settings */
#define ADS131M04_GAIN_1    0b000
#define ADS131M04_GAIN_2    0b001
#define ADS131M04_GAIN_4    0b010
#define ADS131M04_GAIN_8    0b011
#define ADS131M04_GAIN_16   0b100
#define ADS131M04_GAIN_32   0b101
#define ADS131M04_GAIN_64   0b110
#define ADS131M04_GAIN_128  0b111

/* Register masks - GAIN */
#define ADS131M04_REGMASK_GAIN_PGAGAIN3 0x7000
#define ADS131M04_REGMASK_GAIN_PGAGAIN2 0x0700
#define ADS131M04_REGMASK_GAIN_PGAGAIN1 0x0070
#define ADS131M04_REGMASK_GAIN_PGAGAIN0 0x0007

/* Input channel mux options */
#define ADS131M04_MUX_AIN_DIFFERENTIAL 0
#define ADS131M04_MUX_INPUT_SHORTED    1
#define ADS131M04_MUX_POSITIVE_DC_TEST 2
#define ADS131M04_MUX_NEGATIVE_DC_TEST 3

/* Register masks - THRSHLD_LSB */
#define ADS131M04_REGMASK_THRSHLD_LSB_CD_TH_LSB 0xFF00
#define ADS131M04_REGMASK_THRSHLD_LSB_DCBLOCK   0x000F

/* Register masks - CHx_CFG */
#define ADS131M04_REGMASK_CHX_CFG_MUX         0x0003
#define ADS131M04_REGMASK_CHX_CFG_PHASE       0xFFC0
#define ADS131M04_REGMASK_CHX_CFG_DCBLKX_DIS0 0x0004

/* Register masks - CHx_OCAL_LSB / CHx_GCAL_LSB */
#define ADS131M04_REGMASK_CHX_OCAL0_LSB 0xFF00
#define ADS131M04_REGMASK_CHX_GCAL0_LSB 0xFF00

int ads131m04_init(ads131m04_config_t *config);
int ads131m04_reset(ads131m04_config_t *config);
int ads131m04_send_command(ads131m04_config_t *config, uint16_t cmd, uint16_t *response);
int ads131m04_read_register(ads131m04_config_t *config, uint8_t address, uint16_t *value);
int ads131m04_write_register(ads131m04_config_t *config, uint8_t address, uint16_t value);
int ads131m04_write_register_masked(ads131m04_config_t *config, uint8_t address,
    uint16_t value, uint16_t mask);
int ads131m04_read_adc(ads131m04_config_t *config, ads131m04_data_t *data);
int ads131m04_read_adc_after_pause(ads131m04_config_t *config, ads131m04_data_t *data);
int ads131m04_set_oversampling(ads131m04_config_t *config, uint8_t osr_value);
int ads131m04_set_power_mode(ads131m04_config_t *config, uint8_t power_mode);
int ads131m04_set_channel_gain(ads131m04_config_t *config, uint8_t channel, uint8_t gain);
int ads131m04_set_channel_mux(ads131m04_config_t *config, uint8_t channel, uint8_t mux);
int ads131m04_enable_channels(ads131m04_config_t *config, bool ch0_enable, bool ch1_enable,
    bool ch2_enable, bool ch3_enable);
int ads131m04_reset_assert(ads131m04_config_t *config);
int ads131m04_reset_release(ads131m04_config_t *config);
int ads131m04_drdy_is_low(ads131m04_config_t *config);
int ads131m04_wait_for_drdy_falling(ads131m04_config_t *config, uint32_t timeout_us);
int ads131m04_reset_and_read_once(ads131m04_config_t *config, ads131m04_data_t *data,
    uint32_t timeout_us);
int ads131m04_hardware_reset(ads131m04_config_t *config);
int ads131m04_read_status(ads131m04_config_t *config, uint16_t *status);
void ads131m04_print_status(uint16_t status);
int ads131m04_read_id(ads131m04_config_t *config, uint16_t *id);
void ads131m04_calibrate(ads131m04_config_t *config);
int ads131m04_pwm_init(void);
ads131m04_config_t *ads131m04_get_config(void);
int ads131m04_full_setup(ads131m04_config_t *config);
int ads131m04_print_info(ads131m04_config_t *config);
void ads131m04_set_channel_dcblock(ads131m04_config_t *config, uint8_t channel, uint32_t on_off);
void ads131m04_set_high_pass(ads131m04_config_t *config, uint8_t value);

#endif /* ADS131M04_SPI_H */
