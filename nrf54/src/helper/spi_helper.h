/**
 * @file spi_helper.h
 * @brief Shared SPI transceive helper for Zephyr sensor drivers.
 *
 * CS is managed by Zephyr via spi_config.cs (configured in devicetree).
 * The ADS131M04 driver uses this helper.
 */

#ifndef SPI_HELPER_H
#define SPI_HELPER_H

#include <zephyr/drivers/spi.h>

/**
 * @brief Full-duplex SPI transfer with equal-length TX and RX buffers.
 *
 * @param dev  SPI device handle
 * @param cfg  SPI configuration (CS spec from devicetree via spi_config.cs)
 * @param tx   Transmit buffer (must be @p len bytes)
 * @param rx   Receive buffer  (must be @p len bytes)
 * @param len  Transfer length in bytes
 * @return 0 on success, negative errno on failure
 */
static inline int spi_write_read(const struct device *dev,
    const struct spi_config *cfg,
    const uint8_t *tx, uint8_t *rx, size_t len)
{
    const struct spi_buf tx_buf = { .buf = (void *)tx, .len = len };
    const struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = rx, .len = len };
    const struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

    return spi_transceive(dev, cfg, &tx_bufs, &rx_bufs);
}

#endif /* SPI_HELPER_H */
