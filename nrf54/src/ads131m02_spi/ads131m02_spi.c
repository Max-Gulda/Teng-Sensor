/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

/**
 * @file ads131m02_spi.c
 * @brief ADS131M02 24-bit ADC SPI driver implementation for Zephyr RTOS
 */

#include "ads131m02_spi.h"
#include "../helper/spi_helper.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ads131m02_spi, LOG_LEVEL_NONE);

/* Timing delays (in microseconds) */
#define ADS131M02_RESET_DELAY_US   1

/**
 * @brief Convert 24-bit two's complement to signed 32-bit integer
 */
static inline int32_t twos_complement_24bit(int32_t val) {
    if ((val >> 23) & 0x01) {
        return val - (1 << 24);
    }
    return val;
}

/**
 * @brief Parse channel data from a raw ADC SPI response buffer
 */
static void parse_adc_data(const uint8_t *rx_buf, ads131m02_data_t *data) {
    int32_t raw_ch0 = ((int32_t)rx_buf[3] << 16) |
                      ((int32_t)rx_buf[4] << 8) |
                      rx_buf[5];
    raw_ch0 &= 0x00FFFFFF;
    data->ch0 = twos_complement_24bit(raw_ch0);

    int32_t raw_ch1 = ((int32_t)rx_buf[6] << 16) |
                      ((int32_t)rx_buf[7] << 8) |
                      rx_buf[8];
    raw_ch1 &= 0x00FFFFFF;
    data->ch1 = twos_complement_24bit(raw_ch1);
}

int ads131m02_init(ads131m02_config_t *config) {
    int ret;

    if (!device_is_ready(config->spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    /* Configure DRDY GPIO if specified */
    if (config->drdy_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&config->drdy_gpio)) {
            LOG_ERR("DRDY GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure DRDY GPIO: %d", ret);
            return ret;
        }
    }

    /* Configure RESET GPIO if specified */
    if (config->reset_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("RESET GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure RESET GPIO: %d", ret);
            return ret;
        }

        LOG_DBG("RESET GPIO configured as output inactive (not in reset)");
    }

    LOG_INF("ADS131M02 initialized");
    return 0;
}

int ads131m02_reset(ads131m02_config_t *config) {
    int ret;
    uint16_t response;

    ret = ads131m02_send_command(config, ADS131M02_CMD_RESET, &response);
    if (ret < 0) {
        LOG_ERR("Reset command failed: %d", ret);
        return ret;
    }

    k_busy_wait(ADS131M02_RESET_DELAY_US);

    LOG_INF("ADS131M02 reset complete");
    return 0;
}

int ads131m02_send_command(ads131m02_config_t *config, uint16_t cmd, uint16_t *response) {
    int ret;
    uint8_t tx_buf[ADS131M02_MSG_LEN_TWO] = { 0 };
    uint8_t rx_buf[ADS131M02_MSG_LEN_TWO] = { 0 };

    tx_buf[0] = (cmd >> 8) & 0xFF;
    tx_buf[1] = cmd & 0xFF;

    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_TWO);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d", ret);
        return ret;
    }

    /* Extract response - special handling for RESET command */
    if (cmd == ADS131M02_CMD_RESET) {
        *response = (rx_buf[0] << 8) | rx_buf[1];
    } else {
        *response = (rx_buf[ADS131M02_MSG_LEN_TWO / 2] << 8) |
                     rx_buf[ADS131M02_MSG_LEN_TWO / 2 + 1];
    }

    switch (cmd) {
        case ADS131M02_CMD_RESET:    if (*response != ADS131M02_RESP_RESET)   LOG_WRN("Unexpected RESET response: 0x%04x (expected 0x%04x)", *response, ADS131M02_RESP_RESET);   return 0; break;
        case ADS131M02_CMD_STANDBY:  if (*response != ADS131M02_RESP_STANDBY) LOG_WRN("Unexpected STANDBY response: 0x%04x (expected 0x%04x)", *response, ADS131M02_RESP_STANDBY); return 0; break;
        case ADS131M02_CMD_WAKEUP:   if (*response != ADS131M02_RESP_WAKEUP)  LOG_WRN("Unexpected WAKEUP response: 0x%04x (expected 0x%04x)", *response, ADS131M02_RESP_WAKEUP);  return 0; break;
        case ADS131M02_CMD_LOCK:     if (*response != ADS131M02_RESP_LOCK)    LOG_WRN("Unexpected LOCK response: 0x%04x (expected 0x%04x)", *response, ADS131M02_RESP_LOCK);     return 0; break;
        case ADS131M02_CMD_UNLOCK:   if (*response != ADS131M02_RESP_UNLOCK)  LOG_WRN("Unexpected UNLOCK response: 0x%04x (expected 0x%04x)", *response, ADS131M02_RESP_UNLOCK);  return 0; break;
    }

    LOG_DBG("Command 0x%04x -> Response 0x%04x", cmd, *response);
    return 0;
}

int ads131m02_read_register(ads131m02_config_t *config, uint8_t address, uint16_t *value) {
    int ret;
    uint16_t cmd;
    uint8_t tx_buf[ADS131M02_MSG_LEN_TWO] = { 0 };
    uint8_t rx_buf[ADS131M02_MSG_LEN_TWO] = { 0 };
    uint8_t tx_nop[ADS131M02_MSG_LEN_TWO] = { 0 };
    uint8_t rx_nop[ADS131M02_MSG_LEN_TWO] = { 0 };

    cmd = ADS131M02_CMD_READ_REG | ((address << 7) | 0);

    tx_buf[0] = (cmd >> 8) & 0xFF;
    tx_buf[1] = cmd & 0xFF;

    /* Issue read command (response arrives on next frame) */
    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_TWO);
    if (ret < 0) {
        return ret;
    }

    /* Clock out response with a NOP frame */
    tx_nop[0] = (ADS131M02_CMD_NULL >> 8) & 0xFF;
    tx_nop[1] = ADS131M02_CMD_NULL & 0xFF;
    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_nop, rx_nop, ADS131M02_MSG_LEN_TWO);
    if (ret < 0) {
        return ret;
    }

    *value = (rx_nop[ADS131M02_MSG_LEN_TWO / 2] << 8) |
              rx_nop[ADS131M02_MSG_LEN_TWO / 2 + 1];

    LOG_DBG("Read reg 0x%02x = 0x%04x", address, *value);
    return 0;
}

int ads131m02_write_register(ads131m02_config_t *config, uint8_t address, uint16_t value) {
    int ret;
    uint16_t cmd;
    uint8_t tx_buf[ADS131M02_MSG_LEN_TWO] = { 0 };
    uint8_t rx_buf[ADS131M02_MSG_LEN_TWO] = { 0 };

    cmd = ADS131M02_CMD_WRITE_REG | ((address << 7) | 0);

    tx_buf[0] = (cmd >> 8) & 0xFF;
    tx_buf[1] = cmd & 0xFF;
    tx_buf[3] = (value >> 8) & 0xFF;
    tx_buf[4] = value & 0xFF;

    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_TWO);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Write reg 0x%02x = 0x%04x", address, value);
    return 0;
}

int ads131m02_write_register_masked(ads131m02_config_t *config, uint8_t address,
    uint16_t value, uint16_t mask) {
    int ret;
    uint16_t reg_contents;
    int shift = 0;

    ret = ads131m02_read_register(config, address, &reg_contents);
    if (ret < 0) {
        return ret;
    }

    reg_contents &= ~mask;

    /* Calculate shift amount from mask LSB */
    uint16_t temp_mask = mask;
    while ((temp_mask & 0x01) != 1) {
        temp_mask >>= 1;
        shift++;
    }

    reg_contents |= (value << shift);

    ret = ads131m02_write_register(config, address, reg_contents);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Write masked reg 0x%02x, mask 0x%04x", address, mask);
    return 0;
}

int ads131m02_read_adc(ads131m02_config_t *config, ads131m02_data_t *data) {
    uint8_t tx_buf[ADS131M02_MSG_LEN_ONE] = { 0 };
    uint8_t rx_buf[ADS131M02_MSG_LEN_ONE] = { 0 };

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_ONE);
    if (ret < 0) {
        return ret;
    }

    parse_adc_data(rx_buf, data);

    LOG_DBG("ADC Read: CH0=%d, CH1=%d", data->ch0, data->ch1);
    return 0;
}

int ads131m02_read_adc_after_pause(ads131m02_config_t *config, ads131m02_data_t *data) {
    uint8_t tx_buf[ADS131M02_MSG_LEN_ONE] = { 0 };
    uint8_t rx_buf[ADS131M02_MSG_LEN_ONE] = { 0 };

    /* Discard two buffered conversions before reading the current one */
    spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_ONE);
    spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_ONE);

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M02_MSG_LEN_ONE);
    if (ret < 0) {
        return ret;
    }

    parse_adc_data(rx_buf, data);

    LOG_DBG("ADC Read (after pause): CH0=%d, CH1=%d", data->ch0, data->ch1);
    return 0;
}

int ads131m02_drdy_is_low(ads131m02_config_t *config) {
    if (config->drdy_gpio.port == NULL) {
        LOG_WRN("DRDY GPIO not configured");
        return -ENOTSUP;
    }

    int val = gpio_pin_get_dt(&config->drdy_gpio);
    if (val < 0) {
        LOG_ERR("Failed to read DRDY GPIO: %d", val);
        return val;
    }

    return val;
}

int ads131m02_set_oversampling(ads131m02_config_t *config, uint8_t osr_value) {
    if (osr_value > 0x07) {
        LOG_ERR("Invalid OSR value: %d", osr_value);
        return -EINVAL;
    }

    int ret = ads131m02_write_register_masked(config, ADS131M02_REG_CLOCK,
        osr_value, ADS131M02_REGMASK_CLOCK_OSR);
    if (ret < 0) {
        LOG_ERR("Failed to set OSR: %d", ret);
        return ret;
    }

    LOG_INF("OSR set to %d", osr_value);
    return 0;
}

int ads131m02_set_power_mode(ads131m02_config_t *config, uint8_t power_mode) {
    if (power_mode > 2) {
        LOG_ERR("Invalid power mode: %d", power_mode);
        return -EINVAL;
    }

    int ret = ads131m02_write_register_masked(config, ADS131M02_REG_CLOCK,
        power_mode, ADS131M02_REGMASK_CLOCK_PWR);
    if (ret < 0) {
        LOG_ERR("Failed to set power mode: %d", ret);
        return ret;
    }

    LOG_INF("Power mode set to %d", power_mode);
    return 0;
}

int ads131m02_set_channel_gain(ads131m02_config_t *config, uint8_t channel, uint8_t gain) {
    if (channel >= ADS131M02_NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %d", channel);
        return -EINVAL;
    }

    if (gain > 0x07) {
        LOG_ERR("Invalid gain: %d", gain);
        return -EINVAL;
    }

    uint16_t mask = (channel == 0) ? ADS131M02_REGMASK_GAIN_PGAGAIN0 : ADS131M02_REGMASK_GAIN_PGAGAIN1;

    int ret = ads131m02_write_register_masked(config, ADS131M02_REG_GAIN, gain, mask);
    if (ret < 0) {
        LOG_ERR("Failed to set channel %d gain: %d", channel, ret);
        return ret;
    }

    LOG_INF("Channel %d gain set to %d", channel, gain);
    return 0;
}

int ads131m02_set_channel_mux(ads131m02_config_t *config, uint8_t channel, uint8_t mux) {
    if (channel >= ADS131M02_NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %d", channel);
        return -EINVAL;
    }

    if (mux > 3) {
        LOG_ERR("Invalid mux setting: %d", mux);
        return -EINVAL;
    }

    uint8_t reg_addr = (channel == 0) ? ADS131M02_REG_CH0_CFG : ADS131M02_REG_CH1_CFG;

    int ret = ads131m02_write_register_masked(config, reg_addr, mux,
        ADS131M02_REGMASK_CHX_CFG_MUX);
    if (ret < 0) {
        LOG_ERR("Failed to set channel %d mux: %d", channel, ret);
        return ret;
    }

    LOG_INF("Channel %d mux set to %d", channel, mux);
    return 0;
}

int ads131m02_enable_channels(ads131m02_config_t *config, bool ch0_enable, bool ch1_enable) {
    uint16_t clock_reg;
    int ret;

    ret = ads131m02_read_register(config, ADS131M02_REG_CLOCK, &clock_reg);
    if (ret < 0) {
        return ret;
    }

    if (ch0_enable) {
        clock_reg |= ADS131M02_REGMASK_CLOCK_CH0_EN;
    } else {
        clock_reg &= ~ADS131M02_REGMASK_CLOCK_CH0_EN;
    }

    if (ch1_enable) {
        clock_reg |= ADS131M02_REGMASK_CLOCK_CH1_EN;
    } else {
        clock_reg &= ~ADS131M02_REGMASK_CLOCK_CH1_EN;
    }

    ret = ads131m02_write_register(config, ADS131M02_REG_CLOCK, clock_reg);
    if (ret < 0) {
        LOG_ERR("Failed to enable channels: %d", ret);
        return ret;
    }

    LOG_INF("Channels enabled - CH0: %d, CH1: %d", ch0_enable, ch1_enable);
    return 0;
}

int ads131m02_reset_assert(ads131m02_config_t *config) {
    if (config->reset_gpio.port == NULL) {
        LOG_WRN("RESET GPIO not configured");
        return -ENOTSUP;
    }

    gpio_pin_set_dt(&config->reset_gpio, 1);
    LOG_DBG("RESET asserted (active)");
    return 0;
}

int ads131m02_reset_release(ads131m02_config_t *config) {
    if (config->reset_gpio.port == NULL) {
        LOG_WRN("RESET GPIO not configured");
        return -ENOTSUP;
    }

    gpio_pin_set_dt(&config->reset_gpio, 0);
    LOG_DBG("RESET released (inactive)");
    return 0;
}

int ads131m02_wait_for_drdy_falling(ads131m02_config_t *config, uint32_t timeout_us) {
    if (config->drdy_gpio.port == NULL) {
        LOG_WRN("DRDY GPIO not configured");
        return -ENOTSUP;
    }

    uint64_t start_cycles = k_cycle_get_64();
    uint64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);

    LOG_DBG("Waiting for DRDY falling edge, timeout=%u us", timeout_us);

    /* Wait for DRDY to go high if it's currently low */
    while (ads131m02_drdy_is_low(config)) {
        if ((k_cycle_get_64() - start_cycles) > timeout_cycles) {
            LOG_ERR("Timeout waiting for DRDY to go high (stuck low?)");
            return -ETIMEDOUT;
        }
    }

    LOG_DBG("DRDY is high, waiting for falling edge...");

    /* Wait for DRDY to go low (falling edge) */
    start_cycles = k_cycle_get_64();
    while (!ads131m02_drdy_is_low(config)) {
        if ((k_cycle_get_64() - start_cycles) > timeout_cycles) {
            LOG_ERR("Timeout waiting for DRDY falling edge");
            return -ETIMEDOUT;
        }
    }

    LOG_DBG("DRDY falling edge detected");
    return 0;
}

int ads131m02_hardware_reset(ads131m02_config_t *config) {
    int ret;

    if (config->reset_gpio.port == NULL) {
        LOG_WRN("RESET GPIO not configured, cannot perform hardware reset");
        return -ENOTSUP;
    }

    LOG_DBG("Starting hardware reset...");

    ret = ads131m02_reset_assert(config);
    if (ret < 0) {
        return ret;
    }

    /* Hold reset for 10us - well within the 250us max for soft reset */
    k_busy_wait(10);

    ret = ads131m02_reset_release(config);
    if (ret < 0) {
        return ret;
    }

    /* Wait for device to stabilize after reset */
    k_busy_wait(750);

    LOG_DBG("Hardware reset completed");
    return 0;
}

int ads131m02_reset_and_read_once(ads131m02_config_t *config, ads131m02_data_t *data,
    uint32_t timeout_us) {
    int ret;

    if (config->reset_gpio.port == NULL) {
        LOG_ERR("RESET GPIO not configured");
        return -ENOTSUP;
    }

    if (config->drdy_gpio.port == NULL) {
        LOG_ERR("DRDY GPIO not configured");
        return -ENOTSUP;
    }

    ret = ads131m02_hardware_reset(config);
    if (ret < 0) {
        LOG_ERR("Hardware reset failed: %d", ret);
        return ret;
    }

    ret = ads131m02_wait_for_drdy_falling(config, timeout_us);
    if (ret < 0) {
        if (ret == -ETIMEDOUT) {
            LOG_ERR("Timeout waiting for DRDY after reset");
            int drdy = ads131m02_drdy_is_low(config);
            LOG_ERR("Final DRDY state: %d (1=low/ready, 0=high/not-ready)", drdy);
        }
        return ret;
    }

    ret = ads131m02_read_adc(config, data);
    if (ret < 0) {
        LOG_ERR("Failed to read ADC after reset: %d", ret);
        return ret;
    }

    LOG_DBG("Reset and read once completed: CH0=%d, CH1=%d", data->ch0, data->ch1);
    return 0;
}

int ads131m02_read_status(ads131m02_config_t *config, uint16_t *status) {
    uint8_t tx_buf[3] = { 0 };
    uint8_t rx_buf[3] = { 0 };

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, 3);
    if (ret < 0) {
        return ret;
    }

    *status = (rx_buf[0] << 8) | rx_buf[1];

    LOG_DBG("Read status: 0x%04x", *status);
    return 0;
}

void ads131m02_print_status(uint16_t status) {
    LOG_INF("*************************** Status Start ***************************\n");
    LOG_INF("Raw value: 0x%04x\n", status);

    if (status & ADS131M02_REGMASK_STATUS_LOCK) {
        LOG_ERR("Bit 15 = 1   : Locked\n");
    } else {
        LOG_ERR("Bit 15 = 0   : Unlocked (default)\n");
    }

    if (status & ADS131M02_REGMASK_STATUS_RESYNC) {
        LOG_INF("Bit 14 = 1   : Resynchronization occurred\n");
    } else {
        LOG_INF("Bit 14 = 0   : No Resynchronization (default)\n");
    }

    if (status & ADS131M02_REGMASK_STATUS_REGMAP) {
        LOG_INF("Bit 13 = 1   : Register map CRC changed\n");
    } else {
        LOG_INF("Bit 13 = 0   : No change in register map CRC (default)\n");
    }

    if (status & ADS131M02_REGMASK_STATUS_CRC_ERR) {
        LOG_INF("Bit 12 = 1   : Input CRC error occurred\n");
    } else {
        LOG_INF("Bit 12 = 0   : No CRC error (default)\n");
    }

    if (status & 0x0800) {
        LOG_INF("Bit 11 = 1   : 16 bit ANSI\n");
    } else {
        LOG_INF("Bit 11 = 0   : 16 bit CCITT (default)\n");
    }

    if (status & ADS131M02_REGMASK_STATUS_RESET) {
        LOG_INF("Bit 10 = 1   : Reset Occurred (default after reset)\n");
    } else {
        LOG_INF("Bit 10 = 0   : Not Reset\n");
    }

    int wlen = (status & 0x0300) >> 8;
    switch (wlen) {
        case 0b00: LOG_INF("Bit 9:8 = 00 : 16 bit\n"); break;
        case 0b01: LOG_INF("Bit 9:8 = 01 : 24 bits (default)\n"); break;
        case 0b10: LOG_INF("Bit 9:8 = 10 : 32 bits; zero padding\n"); break;
        case 0b11: LOG_INF("Bit 9:8 = 11 : 32 bits; sign extension\n"); break;
    }

    if (status & ADS131M02_REGMASK_STATUS_DRDY1) {
        LOG_INF("Bit 1 = 1    : New data available (DRDY1)\n");
    } else {
        LOG_INF("Bit 1 = 0    : No new data available (DRDY1)\n");
    }

    if (status & ADS131M02_REGMASK_STATUS_DRDY0) {
        LOG_INF("Bit 0 = 1    : New data available (DRDY0)\n");
    } else {
        LOG_INF("Bit 0 = 0    : No new data available (DRDY0)\n");
    }

    LOG_INF("*************************** Status End *****************************\n");
}

int ads131m02_read_id(ads131m02_config_t *config, uint16_t *id) {
    int ret = ads131m02_read_register(config, ADS131M02_REG_ID, id);
    if (ret < 0) {
        LOG_ERR("Failed to read ID register: %d", ret);
        return ret;
    }

    LOG_INF("Device ID: 0x%04x", *id);
    return 0;
}

int ads131m02_set_channel_offset_calibration(ads131m02_config_t *config, uint8_t channel, int32_t offset) {
    uint16_t MSB = offset >> 8;
    uint8_t LSB = offset;

    if (channel >= ADS131M02_NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %d", channel);
        return -EINVAL;
    }

    switch (channel) {
        case 0:
            ads131m02_write_register_masked(config, ADS131M02_REG_CH0_OCAL_MSB, MSB, 0xFFFF);
            ads131m02_write_register_masked(config, ADS131M02_REG_CH0_OCAL_LSB, LSB, ADS131M02_REGMASK_CHX_OCAL0_LSB);
            break;

        case 1:
            ads131m02_write_register_masked(config, ADS131M02_REG_CH1_OCAL_MSB, MSB, 0xFFFF);
            ads131m02_write_register_masked(config, ADS131M02_REG_CH1_OCAL_LSB, LSB, ADS131M02_REGMASK_CHX_OCAL0_LSB);
            break;
    }

    return 0;
}

void ads131m02_calibrate(ads131m02_config_t *config) {
    ads131m02_set_channel_mux(config, 0, ADS131M02_MUX_INPUT_SHORTED);
    ads131m02_set_channel_mux(config, 1, ADS131M02_MUX_INPUT_SHORTED);
    k_busy_wait(10000);

    ads131m02_data_t adc_data;

    ads131m02_read_adc(config, &adc_data);

    ads131m02_set_channel_offset_calibration(config, 0, adc_data.ch0);
    ads131m02_set_channel_offset_calibration(config, 1, adc_data.ch1);

    LOG_INF("calibration values: adc val 0: %d\r\nadc val 1: %d\r\n", adc_data.ch0, adc_data.ch1);

    ads131m02_set_channel_mux(config, 0, ADS131M02_MUX_AIN_DIFFERENTIAL);
    ads131m02_set_channel_mux(config, 1, ADS131M02_MUX_AIN_DIFFERENTIAL);
}

/* ========== High-Level Configuration Functions ========== */

/*
 * ADC External Clock Configuration
 *
 * The ADS131M02 requires an external clock input (CLK pin).
 * This is provided by the nRF54L15's GRTC (Global Real-Time Counter)
 * configured to output 8.192 MHz on P1.12.
 *
 * Configuration is done via devicetree overlay:
 * - clkout-fast-frequency-hz = <8192000>  (8.192 MHz)
 * - Pin P1.12 configured as GRTC_CLKOUT_FAST
 *
 * The GRTC clock is automatically started by the Zephyr kernel during
 * initialization, so no runtime code is needed to start the clock.
 */
#define ADS131M02_CS_IDX         1   /* cs-gpios index in spi21 node */
#define ADS131M02_GPIO_NODE      DT_NODELABEL(gpio1)
#define ADS131M02_DRDY_PIN       9
#define ADS131M02_RESET_PIN      10

static ads131m02_config_t adc_config = {
    .spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi21)),
    .spi_cfg = {
        .frequency = 1000000U,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER |
                     SPI_MODE_CPHA, /* SPI Mode 1: CPOL=0, CPHA=1 */
        .cs = {
            .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi21), cs_gpios, ADS131M02_CS_IDX),
            .delay = 2,
        },
    },
    .drdy_gpio = {
        .port = DEVICE_DT_GET(ADS131M02_GPIO_NODE),
        .pin = ADS131M02_DRDY_PIN,
        .dt_flags = GPIO_ACTIVE_LOW
    },
    .reset_gpio = {
        .port = DEVICE_DT_GET(ADS131M02_GPIO_NODE),
        .pin = ADS131M02_RESET_PIN,
        .dt_flags = GPIO_ACTIVE_LOW
    },
};

ads131m02_config_t *ads131m02_get_config(void) {
    return &adc_config;
}

int ads131m02_full_setup(ads131m02_config_t *config) {
    int err;

    LOG_INF("Initializing ADS131M02...\n");
    err = ads131m02_init(config);
    if (err < 0) {
        LOG_ERR("Failed to initialize ADS131M02 (%d)\n", err);
        return err;
    }

    LOG_INF("Resetting ADS131M02...\n");
    err = ads131m02_reset(config);
    if (err < 0) {
        LOG_ERR("Failed to reset ADS131M02 (%d)\n", err);
        return err;
    }

    LOG_INF("Configuring ADS131M02...\n");

    err = ads131m02_set_power_mode(config, ADS131M02_PWR_HIGH_RESOLUTION);
    if (err < 0) {
        LOG_ERR("Failed to set power mode (%d)\n", err);
        return err;
    }

    err = ads131m02_set_oversampling(config, ADS131M02_OSR_128);
    if (err < 0) {
        LOG_ERR("Failed to set OSR (%d)\n", err);
        return err;
    }

    err = ads131m02_set_channel_gain(config, 0, ADS131M02_GAIN_1); /* ADC channel 0 */
    if (err < 0) {
        LOG_ERR("Failed to set CH0 gain (%d)\n", err);
        return err;
    }

    err = ads131m02_set_channel_gain(config, 1, ADS131M02_GAIN_4); /* ECG */
    if (err < 0) {
        LOG_ERR("Failed to set CH1 gain (%d)\n", err);
        return err;
    }

    err = ads131m02_set_channel_mux(config, 0, ADS131M02_MUX_AIN_DIFFERENTIAL);
    if (err < 0) {
        LOG_ERR("Failed to set CH0 mux (%d)\n", err);
        return err;
    }

    err = ads131m02_set_channel_mux(config, 1, ADS131M02_MUX_AIN_DIFFERENTIAL);
    if (err < 0) {
        LOG_ERR("Failed to set CH1 mux (%d)\n", err);
        return err;
    }

    ads131m02_set_channel_dcblock(config, 0, 0);
    ads131m02_set_channel_dcblock(config, 1, 0);
    ads131m02_set_high_pass(config, 0x00);

    err = ads131m02_enable_channels(config, true, true);
    if (err < 0) {
        LOG_ERR("Failed to enable channels (%d)\n", err);
        return err;
    }

    k_busy_wait(10000);
    ads131m02_calibrate(config);
    k_busy_wait(10000);

    LOG_INF("ADS131M02 setup complete\n");
    return 0;
}

int ads131m02_print_info(ads131m02_config_t *config) {
    int err;
    uint16_t id;
    uint16_t status;

    err = ads131m02_read_id(config, &id);
    if (err < 0) {
        LOG_ERR("Failed to read ID (%d)\n", err);
        return err;
    }
    LOG_INF("ADS131M02 ID: %d", id);

    err = ads131m02_read_status(config, &status);
    if (err < 0) {
        LOG_ERR("Failed to read status (%d)\n", err);
        return err;
    }
    ads131m02_print_status(status);

    return 0;
}

void ads131m02_set_channel_dcblock(ads131m02_config_t *config, uint8_t ch, uint32_t onOff) {
    switch (ch) {
        case 0: ads131m02_write_register_masked(config, ADS131M02_REG_CH0_CFG, !onOff, ADS131M02_REGMASK_CHX_CFG_DCBLKX_DIS0); break;
        case 1: ads131m02_write_register_masked(config, ADS131M02_REG_CH1_CFG, !onOff, ADS131M02_REGMASK_CHX_CFG_DCBLKX_DIS0); break;
        default: break;
    }
}

void ads131m02_set_high_pass(ads131m02_config_t *config, uint8_t value) {
    ads131m02_write_register_masked(config, ADS131M02_REG_THRSHLD_LSB, value, ADS131M02_REGMASK_THRSHLD_LSB_DCBLOCK);
}
