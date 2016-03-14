/*
 * Copyright (c) 2015 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_ARM_DEVICE_SPI_H
#define __ARCH_ARM_DEVICE_SPI_H

#include <errno.h>
#include <stdint.h>

#include <nuttx/device.h>
#include <nuttx/device_spi_board.h>

#define DEVICE_TYPE_SPI_HW          "spi"

/* SPI Flag */
#define SPI_FLAG_HALF_DUPLEX        0x0001      /* can’t do full duplex */
#define SPI_FLAG_NO_RX              0x0002      /* can’t do buffer read */
#define SPI_FLAG_NO_TX              0x0004      /* can’t do buffer write */

/**
 * SPI a read/write buffer pair
 */
struct device_spi_transfer {
    /** Data to be written, or NULL */
    void *txbuffer;
    /** Data to be read, or NULL */
    void *rxbuffer;
    /** Size of rx and tx buffers */
    size_t nwords;
};

/**
 * SPI hardware capabilities info
 */
struct device_spi_master_config {
    /** number of bits per word supported */
    uint32_t bpw_mask;
    /** minimum Transfer speed in Hz */
    uint32_t min_speed_hz;
    /** maximum Transfer speed in Hz */
    uint32_t max_speed_hz;
    /** bit masks of supported SPI protocol mode */
    uint16_t mode;
    /** bit masks of supported SPI protocol flags */
    uint16_t flags;
    /** number of chip select pins supported */
    uint16_t csnum;
    /** maximum divider supported */
    uint16_t max_div;
};

struct device_spi_device_config {
    /** chip name */
    uint8_t name[32];
    /** max speed be set in device */
    uint32_t max_speed_hz;
    /** mode be set in device */
    uint16_t mode;
    /** bit per word be set in device */
    uint8_t bpw;
    /** SPI device type */
    enum device_spi_type device_type;
};

/**
 * SPI device driver operations
 */
struct device_spi_type_ops {
    /** Lock SPI bus for exclusive access */
    int (*lock)(struct device *dev);
    /** Unlock SPI bus for exclusive access */
    int (*unlock)(struct device *dev);
    /** Enable the SPI chip select pin */
    int (*select)(struct device *dev, uint8_t devid);
    /** Disable the SPI chip select pin */
    int (*deselect)(struct device *dev, uint8_t devid);
    /** Configure SPI clock */
    int (*setfrequency)(struct device *dev, uint8_t cs, uint32_t *frequency);
    /** Configure SPI mode */
    int (*setmode)(struct device *dev, uint8_t cs, uint8_t mode);
    /** Set the number of bits per word in transmission */
    int (*setbits)(struct device *dev, uint8_t cs, uint8_t nbits);
    /** Exchange a block of data from SPI */
    int (*exchange)(struct device *dev, struct device_spi_transfer *transfer);
    /** Get SPI device driver hardware capabilities information */
    int (*get_master_config)(struct device *dev,
                             struct device_spi_master_config *master_cfg);
    /** Get configuration parameters from chip */
    int (*get_device_config)(struct device *dev, uint8_t cs,
                             struct device_spi_device_config *device_cfg);
};

/**
 * @brief SPI lock wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_lock(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->lock) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->lock(dev);
    }
    return -ENOSYS;
}

/**
 * @brief SPI unlock wrap function
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_unlock(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->unlock) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->unlock(dev);
    }
    return -ENOSYS;
}

/**
 * @brief SPI select wrap function
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_select(struct device *dev, uint8_t devid)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->select) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->select(dev, devid);
    }
    return -ENOSYS;
}

/**
 * @brief SPI deselect wrap function
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_deselect(struct device *dev, uint8_t devid)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->deselect) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->deselect(dev, devid);
    }
    return -ENOSYS;
}

/**
 * @brief SPI setfrequency wrap function
 *
 * @param dev pointer to structure of device data
 * @param cs required chip number
 * @param frequency SPI frequency requested (unit: Hz)
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_setfrequency(struct device *dev, uint8_t cs,
                                          uint32_t *frequency)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->setfrequency) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->setfrequency(dev, cs,
                                                             frequency);
    }
    return -ENOSYS;
}

/**
 * @brief SPI setmode wrap function
 *
 * @param dev pointer to structure of device data
 * @param cs required chip number
 * @param mode SPI protocol mode requested
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_setmode(struct device *dev, uint8_t cs,
                                     uint8_t mode)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->setmode) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->setmode(dev, cs, mode);
    }
    return -ENOSYS;
}

/**
 * @brief SPI setbits wrap function
 *
 * @param dev pointer to structure of device data
 * @param cs required chip number
 * @param nbits The number of bits requested. The nbits value range is from
 *        1 to 32. The generic nbits value is 8, 16, 32, but this value still
 *        depends on hardware supported.
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_setbits(struct device *dev, uint8_t cs,
                                     uint8_t nbits)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->setbits) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->setbits(dev, cs, nbits);
    }
    return -ENOSYS;
}

/**
 * @brief SPI exchange wrap function
 *
 * @param dev pointer to structure of device data
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_exchange(struct device *dev,
                                      struct device_spi_transfer *transfer)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->exchange) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->exchange(dev, transfer);
    }
    return -ENOSYS;
}

/**
 * @brief SPI get_master_config wrap function
 *
 * @param dev pointer to structure of device data
 * @param master_config pointer to the structure to receive the capabilities
 *             information.
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_get_master_config(struct device *dev,
                                               struct device_spi_master_config *master_cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->get_master_config) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->get_master_config(dev, master_cfg);
    }
    return -ENOSYS;
}

/**
 * @brief SPI device configuration
 *
 * @param dev pointer to structure of device data
 * @param cs required chip number
 * @param device_cfg pointer to the device_spi_device_config structure to receive
 * the specific chip of configuration.
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_get_device_config(struct device *dev, uint8_t cs,
                                               struct device_spi_device_config *device_cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->get_device_config) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->get_device_config(dev, cs,
                                                                  device_cfg);
    }
    return -ENOSYS;
}

#endif /* __ARCH_ARM_DEVICE_SPI_H */
