/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Jesper Sjöberg (@repsejs, KTH: @jessjobe)
 * Copyright (c) 2026 Max Gulda (@Max-Gulda, KTH: @gulda)
 */

/**
 * @file ads131m04_spi.c
 * @brief ADS131M04 24-bit ADC SPI driver implementation for Zephyr RTOS
 */

#include "ads131m04_spi.h"
#include "../helper/spi_helper.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ads131m04_spi, LOG_LEVEL_NONE);

/* Timing delays (in microseconds) */
#define ADS131M04_RESET_DELAY_US 1

static const uint16_t gain_masks[ADS131M04_NUM_CHANNELS] = {
    ADS131M04_REGMASK_GAIN_PGAGAIN0,
    ADS131M04_REGMASK_GAIN_PGAGAIN1,
    ADS131M04_REGMASK_GAIN_PGAGAIN2,
    ADS131M04_REGMASK_GAIN_PGAGAIN3,
};

static const uint8_t cfg_registers[ADS131M04_NUM_CHANNELS] = {
    ADS131M04_REG_CH0_CFG,
    ADS131M04_REG_CH1_CFG,
    ADS131M04_REG_CH2_CFG,
    ADS131M04_REG_CH3_CFG,
};

static const uint8_t ocal_msb_registers[ADS131M04_NUM_CHANNELS] = {
    ADS131M04_REG_CH0_OCAL_MSB,
    ADS131M04_REG_CH1_OCAL_MSB,
    ADS131M04_REG_CH2_OCAL_MSB,
    ADS131M04_REG_CH3_OCAL_MSB,
};

static const uint8_t ocal_lsb_registers[ADS131M04_NUM_CHANNELS] = {
    ADS131M04_REG_CH0_OCAL_LSB,
    ADS131M04_REG_CH1_OCAL_LSB,
    ADS131M04_REG_CH2_OCAL_LSB,
    ADS131M04_REG_CH3_OCAL_LSB,
};

static const uint16_t channel_enable_masks[ADS131M04_NUM_CHANNELS] = {
    ADS131M04_REGMASK_CLOCK_CH0_EN,
    ADS131M04_REGMASK_CLOCK_CH1_EN,
    ADS131M04_REGMASK_CLOCK_CH2_EN,
    ADS131M04_REGMASK_CLOCK_CH3_EN,
};

/**
 * @brief Convert 24-bit two's complement to signed 32-bit integer
 */
static inline int32_t twos_complement_24bit(int32_t val) {
    if ((val >> 23) & 0x01) {
        return val - (1 << 24);
    }
    return val;
}

static int32_t parse_adc_word(const uint8_t *rx_buf, size_t word_index) {
    size_t offset = word_index * ADS131M04_WORD_LEN_BYTES;
    int32_t raw = ((int32_t)rx_buf[offset] << 16) |
                  ((int32_t)rx_buf[offset + 1] << 8) |
                  rx_buf[offset + 2];

    return twos_complement_24bit(raw & 0x00FFFFFF);
}

/**
 * @brief Parse channel data from a raw ADC SPI response buffer
 */
static void parse_adc_data(const uint8_t *rx_buf, ads131m04_data_t *data) {
    data->ch0 = parse_adc_word(rx_buf, 1);
    data->ch1 = parse_adc_word(rx_buf, 2);
    data->ch2 = parse_adc_word(rx_buf, 3);
    data->ch3 = parse_adc_word(rx_buf, 4);
}

static int32_t get_channel_sample(const ads131m04_data_t *data, uint8_t channel) {
    switch (channel) {
        case 0:
            return data->ch0;
        case 1:
            return data->ch1;
        case 2:
            return data->ch2;
        case 3:
            return data->ch3;
        default:
            return 0;
    }
}

static int ads131m04_gain_to_reg_value(uint8_t gain_x, uint8_t *reg_value) {
    if (reg_value == NULL) {
        return -EINVAL;
    }

    switch (gain_x) {
        case ADS131M04_GAIN_1:
            *reg_value = 0b000;
            return 0;
        case ADS131M04_GAIN_2:
            *reg_value = 0b001;
            return 0;
        case ADS131M04_GAIN_4:
            *reg_value = 0b010;
            return 0;
        case ADS131M04_GAIN_8:
            *reg_value = 0b011;
            return 0;
        case ADS131M04_GAIN_16:
            *reg_value = 0b100;
            return 0;
        case ADS131M04_GAIN_32:
            *reg_value = 0b101;
            return 0;
        case ADS131M04_GAIN_64:
            *reg_value = 0b110;
            return 0;
        case ADS131M04_GAIN_128:
            *reg_value = 0b111;
            return 0;
        default:
            return -EINVAL;
    }
}

static int ads131m04_set_channel_offset_calibration(ads131m04_config_t *config, uint8_t channel,
    int32_t offset) {
    uint32_t raw_offset = (uint32_t)offset & 0x00FFFFFF;
    uint16_t msb = (raw_offset >> 8) & 0xFFFF;
    uint8_t lsb = raw_offset & 0xFF;

    if (channel >= ADS131M04_NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %d", channel);
        return -EINVAL;
    }

    int ret = ads131m04_write_register_masked(config, ocal_msb_registers[channel], msb, 0xFFFF);
    if (ret < 0) {
        return ret;
    }

    return ads131m04_write_register_masked(config, ocal_lsb_registers[channel], lsb,
        ADS131M04_REGMASK_CHX_OCAL0_LSB);
}

int ads131m04_init(ads131m04_config_t *config) {
    int ret;

    if (!device_is_ready(config->spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

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

    LOG_INF("ADS131M04 initialized");
    return 0;
}

int ads131m04_reset(ads131m04_config_t *config) {
    int ret;
    uint16_t response;

    ret = ads131m04_send_command(config, ADS131M04_CMD_RESET, &response);
    if (ret < 0) {
        LOG_ERR("Reset command failed: %d", ret);
        return ret;
    }

    k_busy_wait(ADS131M04_RESET_DELAY_US);

    LOG_INF("ADS131M04 reset complete");
    return 0;
}

int ads131m04_send_command(ads131m04_config_t *config, uint16_t cmd, uint16_t *response) {
    int ret;
    uint8_t tx_buf[ADS131M04_MSG_LEN_TWO] = { 0 };
    uint8_t rx_buf[ADS131M04_MSG_LEN_TWO] = { 0 };

    tx_buf[0] = (cmd >> 8) & 0xFF;
    tx_buf[1] = cmd & 0xFF;

    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_TWO);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d", ret);
        return ret;
    }

    if (cmd == ADS131M04_CMD_RESET) {
        *response = (rx_buf[0] << 8) | rx_buf[1];
    } else {
        *response = (rx_buf[ADS131M04_MSG_LEN_TWO / 2] << 8) |
                    rx_buf[ADS131M04_MSG_LEN_TWO / 2 + 1];
    }

    switch (cmd) {
        case ADS131M04_CMD_RESET:
            if (*response != ADS131M04_RESP_RESET) {
                LOG_WRN("Unexpected RESET response: 0x%04x (expected 0x%04x)",
                    *response, ADS131M04_RESP_RESET);
            }
            return 0;
        case ADS131M04_CMD_STANDBY:
            if (*response != ADS131M04_RESP_STANDBY) {
                LOG_WRN("Unexpected STANDBY response: 0x%04x (expected 0x%04x)",
                    *response, ADS131M04_RESP_STANDBY);
            }
            return 0;
        case ADS131M04_CMD_WAKEUP:
            if (*response != ADS131M04_RESP_WAKEUP) {
                LOG_WRN("Unexpected WAKEUP response: 0x%04x (expected 0x%04x)",
                    *response, ADS131M04_RESP_WAKEUP);
            }
            return 0;
        case ADS131M04_CMD_LOCK:
            if (*response != ADS131M04_RESP_LOCK) {
                LOG_WRN("Unexpected LOCK response: 0x%04x (expected 0x%04x)",
                    *response, ADS131M04_RESP_LOCK);
            }
            return 0;
        case ADS131M04_CMD_UNLOCK:
            if (*response != ADS131M04_RESP_UNLOCK) {
                LOG_WRN("Unexpected UNLOCK response: 0x%04x (expected 0x%04x)",
                    *response, ADS131M04_RESP_UNLOCK);
            }
            return 0;
        default:
            break;
    }

    LOG_DBG("Command 0x%04x -> Response 0x%04x", cmd, *response);
    return 0;
}

int ads131m04_read_register(ads131m04_config_t *config, uint8_t address, uint16_t *value) {
    int ret;
    uint16_t cmd = ADS131M04_CMD_READ_REG | (address << 7);
    size_t resp_offset = ADS131M04_MSG_LEN_ONE;
    uint8_t tx_buf[ADS131M04_MSG_LEN_TWO] = { 0 };
    uint8_t rx_buf[ADS131M04_MSG_LEN_TWO] = { 0 };

    tx_buf[0] = (cmd >> 8) & 0xFF;
    tx_buf[1] = cmd & 0xFF;

    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_TWO);
    if (ret < 0) {
        return ret;
    }

    /*
     * For a single-register read, the requested register contents are returned
     * in the response word of the frame immediately following the RREG
     * command. The second 18-byte half of this transfer is that next frame.
     */
    *value = (rx_buf[resp_offset] << 8) | rx_buf[resp_offset + 1];

    LOG_DBG("Read reg 0x%02x = 0x%04x", address, *value);
    return 0;
}

int ads131m04_write_register(ads131m04_config_t *config, uint8_t address, uint16_t value) {
    int ret;
    uint16_t cmd = ADS131M04_CMD_WRITE_REG | (address << 7);
    uint8_t tx_buf[ADS131M04_MSG_LEN_TWO] = { 0 };
    uint8_t rx_buf[ADS131M04_MSG_LEN_TWO] = { 0 };

    tx_buf[0] = (cmd >> 8) & 0xFF;
    tx_buf[1] = cmd & 0xFF;
    tx_buf[3] = (value >> 8) & 0xFF;
    tx_buf[4] = value & 0xFF;

    ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_TWO);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Write reg 0x%02x = 0x%04x", address, value);
    return 0;
}

int ads131m04_write_register_masked(ads131m04_config_t *config, uint8_t address,
    uint16_t value, uint16_t mask) {
    int ret;
    uint16_t reg_contents;
    int shift = 0;
    uint16_t temp_mask = mask;

    ret = ads131m04_read_register(config, address, &reg_contents);
    if (ret < 0) {
        return ret;
    }

    reg_contents &= ~mask;

    while ((temp_mask & 0x01U) != 1U) {
        temp_mask >>= 1;
        shift++;
    }

    reg_contents |= ((value << shift) & mask);

    ret = ads131m04_write_register(config, address, reg_contents);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Write masked reg 0x%02x, mask 0x%04x", address, mask);
    return 0;
}

int ads131m04_read_adc(ads131m04_config_t *config, ads131m04_data_t *data) {
    uint8_t tx_buf[ADS131M04_MSG_LEN_ONE] = { 0 };
    uint8_t rx_buf[ADS131M04_MSG_LEN_ONE] = { 0 };

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_ONE);
    if (ret < 0) {
        return ret;
    }

    parse_adc_data(rx_buf, data);

    LOG_DBG("ADC Read: CH0=%d, CH1=%d, CH2=%d, CH3=%d",
        data->ch0, data->ch1, data->ch2, data->ch3);
    return 0;
}

int ads131m04_read_adc_after_pause(ads131m04_config_t *config, ads131m04_data_t *data) {
    uint8_t tx_buf[ADS131M04_MSG_LEN_ONE] = { 0 };
    uint8_t rx_buf[ADS131M04_MSG_LEN_ONE] = { 0 };

    spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_ONE);
    spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_ONE);

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_MSG_LEN_ONE);
    if (ret < 0) {
        return ret;
    }

    parse_adc_data(rx_buf, data);

    LOG_DBG("ADC Read (after pause): CH0=%d, CH1=%d, CH2=%d, CH3=%d",
        data->ch0, data->ch1, data->ch2, data->ch3);
    return 0;
}

int ads131m04_drdy_is_low(ads131m04_config_t *config) {
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

int ads131m04_set_oversampling(ads131m04_config_t *config, uint8_t osr_value) {
    if (osr_value > 0x07) {
        LOG_ERR("Invalid OSR value: %d", osr_value);
        return -EINVAL;
    }

    int ret = ads131m04_write_register_masked(config, ADS131M04_REG_CLOCK,
        osr_value, ADS131M04_REGMASK_CLOCK_OSR);
    if (ret < 0) {
        LOG_ERR("Failed to set OSR: %d", ret);
        return ret;
    }

    LOG_INF("OSR set to %d", osr_value);
    return 0;
}

int ads131m04_set_power_mode(ads131m04_config_t *config, uint8_t power_mode) {
    if (power_mode > 2) {
        LOG_ERR("Invalid power mode: %d", power_mode);
        return -EINVAL;
    }

    int ret = ads131m04_write_register_masked(config, ADS131M04_REG_CLOCK,
        power_mode, ADS131M04_REGMASK_CLOCK_PWR);
    if (ret < 0) {
        LOG_ERR("Failed to set power mode: %d", ret);
        return ret;
    }

    LOG_INF("Power mode set to %d", power_mode);
    return 0;
}

int ads131m04_set_channel_gain(ads131m04_config_t *config, uint8_t channel, uint8_t gain) {
    uint8_t gain_reg_value;

    if (channel >= ADS131M04_NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %d", channel);
        return -EINVAL;
    }

    if (ads131m04_gain_to_reg_value(gain, &gain_reg_value) < 0) {
        LOG_ERR("Invalid gain multiplier: %u", gain);
        return -EINVAL;
    }

    int ret = ads131m04_write_register_masked(config, ADS131M04_REG_GAIN,
        gain_reg_value, gain_masks[channel]);
    if (ret < 0) {
        LOG_ERR("Failed to set channel %d gain: %d", channel, ret);
        return ret;
    }

    LOG_INF("Channel %d gain set to %ux", channel, gain);
    return 0;
}

int ads131m04_set_channel_mux(ads131m04_config_t *config, uint8_t channel, uint8_t mux) {
    if (channel >= ADS131M04_NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %d", channel);
        return -EINVAL;
    }

    if (mux > 3) {
        LOG_ERR("Invalid mux setting: %d", mux);
        return -EINVAL;
    }

    int ret = ads131m04_write_register_masked(config, cfg_registers[channel], mux,
        ADS131M04_REGMASK_CHX_CFG_MUX);
    if (ret < 0) {
        LOG_ERR("Failed to set channel %d mux: %d", channel, ret);
        return ret;
    }

    LOG_INF("Channel %d mux set to %d", channel, mux);
    return 0;
}

int ads131m04_enable_channels(ads131m04_config_t *config, bool ch0_enable, bool ch1_enable,
    bool ch2_enable, bool ch3_enable) {
    uint16_t clock_reg;
    bool enabled[ADS131M04_NUM_CHANNELS] = {
        ch0_enable,
        ch1_enable,
        ch2_enable,
        ch3_enable,
    };

    int ret = ads131m04_read_register(config, ADS131M04_REG_CLOCK, &clock_reg);
    if (ret < 0) {
        return ret;
    }

    for (uint8_t channel = 0; channel < ADS131M04_NUM_CHANNELS; channel++) {
        if (enabled[channel]) {
            clock_reg |= channel_enable_masks[channel];
        } else {
            clock_reg &= ~channel_enable_masks[channel];
        }
    }

    ret = ads131m04_write_register(config, ADS131M04_REG_CLOCK, clock_reg);
    if (ret < 0) {
        LOG_ERR("Failed to enable channels: %d", ret);
        return ret;
    }

    LOG_INF("Channels enabled - CH0: %d, CH1: %d, CH2: %d, CH3: %d",
        ch0_enable, ch1_enable, ch2_enable, ch3_enable);
    return 0;
}

int ads131m04_reset_assert(ads131m04_config_t *config) {
    if (config->reset_gpio.port == NULL) {
        LOG_WRN("RESET GPIO not configured");
        return -ENOTSUP;
    }

    gpio_pin_set_dt(&config->reset_gpio, 1);
    LOG_DBG("RESET asserted (active)");
    return 0;
}

int ads131m04_reset_release(ads131m04_config_t *config) {
    if (config->reset_gpio.port == NULL) {
        LOG_WRN("RESET GPIO not configured");
        return -ENOTSUP;
    }

    gpio_pin_set_dt(&config->reset_gpio, 0);
    LOG_DBG("RESET released (inactive)");
    return 0;
}

int ads131m04_wait_for_drdy_falling(ads131m04_config_t *config, uint32_t timeout_us) {
    if (config->drdy_gpio.port == NULL) {
        LOG_WRN("DRDY GPIO not configured");
        return -ENOTSUP;
    }

    uint64_t start_cycles = k_cycle_get_64();
    uint64_t timeout_cycles = k_us_to_cyc_ceil64(timeout_us);

    LOG_DBG("Waiting for DRDY falling edge, timeout=%u us", timeout_us);

    while (ads131m04_drdy_is_low(config)) {
        if ((k_cycle_get_64() - start_cycles) > timeout_cycles) {
            LOG_ERR("Timeout waiting for DRDY to go high (stuck low?)");
            return -ETIMEDOUT;
        }
    }

    LOG_DBG("DRDY is high, waiting for falling edge...");

    start_cycles = k_cycle_get_64();
    while (!ads131m04_drdy_is_low(config)) {
        if ((k_cycle_get_64() - start_cycles) > timeout_cycles) {
            LOG_ERR("Timeout waiting for DRDY falling edge");
            return -ETIMEDOUT;
        }
    }

    LOG_DBG("DRDY falling edge detected");
    return 0;
}

int ads131m04_hardware_reset(ads131m04_config_t *config) {
    int ret;

    if (config->reset_gpio.port == NULL) {
        LOG_WRN("RESET GPIO not configured, cannot perform hardware reset");
        return -ENOTSUP;
    }

    LOG_DBG("Starting hardware reset...");

    ret = ads131m04_reset_assert(config);
    if (ret < 0) {
        return ret;
    }

    k_busy_wait(10);

    ret = ads131m04_reset_release(config);
    if (ret < 0) {
        return ret;
    }

    k_busy_wait(750);

    LOG_DBG("Hardware reset completed");
    return 0;
}

int ads131m04_reset_and_read_once(ads131m04_config_t *config, ads131m04_data_t *data,
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

    ret = ads131m04_hardware_reset(config);
    if (ret < 0) {
        LOG_ERR("Hardware reset failed: %d", ret);
        return ret;
    }

    ret = ads131m04_wait_for_drdy_falling(config, timeout_us);
    if (ret < 0) {
        if (ret == -ETIMEDOUT) {
            int drdy = ads131m04_drdy_is_low(config);

            LOG_ERR("Timeout waiting for DRDY after reset");
            LOG_ERR("Final DRDY state: %d (1=low/ready, 0=high/not-ready)", drdy);
        }
        return ret;
    }

    ret = ads131m04_read_adc(config, data);
    if (ret < 0) {
        LOG_ERR("Failed to read ADC after reset: %d", ret);
        return ret;
    }

    LOG_DBG("Reset and read once completed: CH0=%d, CH1=%d, CH2=%d, CH3=%d",
        data->ch0, data->ch1, data->ch2, data->ch3);
    return 0;
}

int ads131m04_read_status(ads131m04_config_t *config, uint16_t *status) {
    uint8_t tx_buf[ADS131M04_WORD_LEN_BYTES] = { 0 };
    uint8_t rx_buf[ADS131M04_WORD_LEN_BYTES] = { 0 };

    int ret = spi_write_read(config->spi_dev, &config->spi_cfg, tx_buf, rx_buf, ADS131M04_WORD_LEN_BYTES);
    if (ret < 0) {
        return ret;
    }

    *status = (rx_buf[0] << 8) | rx_buf[1];

    LOG_DBG("Read status: 0x%04x", *status);
    return 0;
}

void ads131m04_print_status(uint16_t status) {
    LOG_INF("*************************** Status Start ***************************\n");
    LOG_INF("Raw value: 0x%04x\n", status);

    if (status & ADS131M04_REGMASK_STATUS_LOCK) {
        LOG_ERR("Bit 15 = 1   : Locked\n");
    } else {
        LOG_ERR("Bit 15 = 0   : Unlocked (default)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_RESYNC) {
        LOG_INF("Bit 14 = 1   : Resynchronization occurred\n");
    } else {
        LOG_INF("Bit 14 = 0   : No Resynchronization (default)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_REGMAP) {
        LOG_INF("Bit 13 = 1   : Register map CRC changed\n");
    } else {
        LOG_INF("Bit 13 = 0   : No change in register map CRC (default)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_CRC_ERR) {
        LOG_INF("Bit 12 = 1   : Input CRC error occurred\n");
    } else {
        LOG_INF("Bit 12 = 0   : No CRC error (default)\n");
    }

    if (status & 0x0800) {
        LOG_INF("Bit 11 = 1   : 16 bit ANSI\n");
    } else {
        LOG_INF("Bit 11 = 0   : 16 bit CCITT (default)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_RESET) {
        LOG_INF("Bit 10 = 1   : Reset occurred (default after reset)\n");
    } else {
        LOG_INF("Bit 10 = 0   : Not reset\n");
    }

    switch ((status & 0x0300) >> 8) {
        case 0b00:
            LOG_INF("Bit 9:8 = 00 : 16 bit\n");
            break;
        case 0b01:
            LOG_INF("Bit 9:8 = 01 : 24 bits (default)\n");
            break;
        case 0b10:
            LOG_INF("Bit 9:8 = 10 : 32 bits; zero padding\n");
            break;
        case 0b11:
            LOG_INF("Bit 9:8 = 11 : 32 bits; sign extension\n");
            break;
    }

    if (status & ADS131M04_REGMASK_STATUS_DRDY3) {
        LOG_INF("Bit 3 = 1    : New data available (DRDY3)\n");
    } else {
        LOG_INF("Bit 3 = 0    : No new data available (DRDY3)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_DRDY2) {
        LOG_INF("Bit 2 = 1    : New data available (DRDY2)\n");
    } else {
        LOG_INF("Bit 2 = 0    : No new data available (DRDY2)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_DRDY1) {
        LOG_INF("Bit 1 = 1    : New data available (DRDY1)\n");
    } else {
        LOG_INF("Bit 1 = 0    : No new data available (DRDY1)\n");
    }

    if (status & ADS131M04_REGMASK_STATUS_DRDY0) {
        LOG_INF("Bit 0 = 1    : New data available (DRDY0)\n");
    } else {
        LOG_INF("Bit 0 = 0    : No new data available (DRDY0)\n");
    }

    LOG_INF("*************************** Status End *****************************\n");
}

int ads131m04_read_id(ads131m04_config_t *config, uint16_t *id) {
    int ret = ads131m04_read_register(config, ADS131M04_REG_ID, id);
    if (ret < 0) {
        LOG_ERR("Failed to read ID register: %d", ret);
        return ret;
    }

    LOG_INF("Device ID: 0x%04x", *id);
    return 0;
}

void ads131m04_calibrate(ads131m04_config_t *config) {
    ads131m04_data_t adc_data = { 0 };

    for (uint8_t channel = 0; channel < ADS131M04_NUM_CHANNELS; channel++) {
        ads131m04_set_channel_mux(config, channel, ADS131M04_MUX_INPUT_SHORTED);
    }

    k_busy_wait(10000);

    if (ads131m04_read_adc(config, &adc_data) < 0) {
        LOG_WRN("Calibration read failed");
    } else {
        for (uint8_t channel = 0; channel < ADS131M04_NUM_CHANNELS; channel++) {
            int ret = ads131m04_set_channel_offset_calibration(config, channel,
                get_channel_sample(&adc_data, channel));
            if (ret < 0) {
                LOG_WRN("Failed to calibrate channel %d: %d", channel, ret);
            }
        }

        LOG_INF("Calibration values: CH0=%d, CH1=%d, CH2=%d, CH3=%d",
            adc_data.ch0, adc_data.ch1, adc_data.ch2, adc_data.ch3);
    }

    for (uint8_t channel = 0; channel < ADS131M04_NUM_CHANNELS; channel++) {
        ads131m04_set_channel_mux(config, channel, ADS131M04_MUX_AIN_DIFFERENTIAL);
    }
}

/* ========== High-Level Configuration Functions ========== */

/*
 * ADC External Clock Configuration
 *
 * The ADS131M04 requires an external clock input (CLK pin).
 * This is provided by the nRF54L15 GRTC clock output configured in the
 * devicetree overlay.
 */
#define ADS131M04_CS_IDX    0
#define ADS131M04_GPIO_NODE DT_NODELABEL(gpio1)
#define ADS131M04_DRDY_PIN  9
#define ADS131M04_RESET_PIN 10

static ads131m04_config_t adc_config = {
    .spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi21)),
    .spi_cfg = {
        .frequency = 1000000U,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER |
                     SPI_MODE_CPHA,
        .cs = {
            .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi21), cs_gpios, ADS131M04_CS_IDX),
            .delay = 2,
        },
    },
    .drdy_gpio = {
        .port = DEVICE_DT_GET(ADS131M04_GPIO_NODE),
        .pin = ADS131M04_DRDY_PIN,
        .dt_flags = GPIO_ACTIVE_LOW
    },
    .reset_gpio = {
        .port = DEVICE_DT_GET(ADS131M04_GPIO_NODE),
        .pin = ADS131M04_RESET_PIN,
        .dt_flags = GPIO_ACTIVE_LOW
    },
};

ads131m04_config_t *ads131m04_get_config(void) {
    return &adc_config;
}

int ads131m04_full_setup(ads131m04_config_t *config) {
    int err;

    LOG_INF("Initializing ADS131M04...\n");
    err = ads131m04_init(config);
    if (err < 0) {
        LOG_ERR("Failed to initialize ADS131M04 (%d)\n", err);
        return err;
    }

    LOG_INF("Resetting ADS131M04...\n");
    err = ads131m04_reset(config);
    if (err < 0) {
        LOG_ERR("Failed to reset ADS131M04 (%d)\n", err);
        return err;
    }

    LOG_INF("Configuring ADS131M04...\n");

    err = ads131m04_set_power_mode(config, ADS131M04_PWR_HIGH_RESOLUTION);
    if (err < 0) {
        LOG_ERR("Failed to set power mode (%d)\n", err);
        return err;
    }

    err = ads131m04_set_oversampling(config, ADS131M04_OSR_8192);
    if (err < 0) {
        LOG_ERR("Failed to set OSR (%d)\n", err);
        return err;
    }

    err = ads131m04_set_channel_gain(config, 0, ADS131M04_GAIN_1);
    if (err < 0) {
        LOG_ERR("Failed to set CH0 gain (%d)\n", err);
        return err;
    }

    err = ads131m04_set_channel_gain(config, 1, ADS131M04_GAIN_1);
    if (err < 0) {
        LOG_ERR("Failed to set CH1 gain (%d)\n", err);
        return err;
    }

    err = ads131m04_set_channel_gain(config, 2, ADS131M04_GAIN_1);
    if (err < 0) {
        LOG_ERR("Failed to set CH2 gain (%d)\n", err);
        return err;
    }

    err = ads131m04_set_channel_gain(config, 3, ADS131M04_GAIN_1);
    if (err < 0) {
        LOG_ERR("Failed to set CH3 gain (%d)\n", err);
        return err;
    }

    for (uint8_t channel = 0; channel < ADS131M04_NUM_CHANNELS; channel++) {
        err = ads131m04_set_channel_mux(config, channel, ADS131M04_MUX_AIN_DIFFERENTIAL);
        if (err < 0) {
            LOG_ERR("Failed to set CH%d mux (%d)\n", channel, err);
            return err;
        }

        ads131m04_set_channel_dcblock(config, channel, 0);
    }

    ads131m04_set_high_pass(config, 0x00);

    err = ads131m04_enable_channels(config, true, true, true, true);
    if (err < 0) {
        LOG_ERR("Failed to enable channels (%d)\n", err);
        return err;
    }

    k_busy_wait(10000);
    ads131m04_calibrate(config);
    k_busy_wait(10000);

    LOG_INF("ADS131M04 setup complete\n");
    return 0;
}

int ads131m04_print_info(ads131m04_config_t *config) {
    int err;
    uint16_t id;
    uint16_t status;

    err = ads131m04_read_id(config, &id);
    if (err < 0) {
        LOG_ERR("Failed to read ID (%d)\n", err);
        return err;
    }
    LOG_INF("ADS131M04 ID: %d", id);

    err = ads131m04_read_status(config, &status);
    if (err < 0) {
        LOG_ERR("Failed to read status (%d)\n", err);
        return err;
    }
    ads131m04_print_status(status);

    return 0;
}

void ads131m04_set_channel_dcblock(ads131m04_config_t *config, uint8_t channel, uint32_t on_off) {
    if (channel >= ADS131M04_NUM_CHANNELS) {
        return;
    }

    (void)ads131m04_write_register_masked(config, cfg_registers[channel], !on_off,
        ADS131M04_REGMASK_CHX_CFG_DCBLKX_DIS0);
}

void ads131m04_set_high_pass(ads131m04_config_t *config, uint8_t value) {
    (void)ads131m04_write_register_masked(config, ADS131M04_REG_THRSHLD_LSB, value,
        ADS131M04_REGMASK_THRSHLD_LSB_DCBLOCK);
}
