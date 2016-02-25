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

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#define DEVICE_TYPE_SPI_HW          "spi"

#define SPI_FLAG_ASYNC_TRANSFER     0x01        /* non-blocking transfer */
#define SPI_FLAG_DMA_TRNSFER        0x02        /* DMA transfer */

/* SPI mode definition */
#define SPI_MODE_CPHA               0x01        /* clock phase */
#define SPI_MODE_CPOL               0x02        /* clock polarity */
#define SPI_MODE_CS_HIGH            0x04        /* chipselect active high */
#define SPI_MODE_LSB_FIRST          0x08        /* per-word bits-on-wire */
#define SPI_MODE_3WIRE              0x10        /* SI/SO signals shared */
#define SPI_MODE_LOOP               0x20        /* loopback mode */
#define SPI_MODE_NO_CS              0x40        /* 1 dev/bus, no chipselect */
#define SPI_MODE_READY              0x80        /* slave pulls low to pause */

#define SPI_MODE_0                  (0 | 0)     /* (original MicroWire) */
#define SPI_MODE_1                  (0 | SPI_MODE_CPHA)
#define SPI_MODE_2                  (SPI_MODE_CPOL | 0)
#define SPI_MODE_3                  (SPI_MODE_CPOL | SPI_MODE_CPHA)

/* SPI Flag */
#define SPI_FLAG_HALF_DUPLEX        0x0001      /* can’t do full duplex */
#define SPI_FLAG_NO_RX              0x0002      /* can’t do buffer read */
#define SPI_FLAG_NO_TX              0x0004      /* can’t do buffer write */

#define SPI_XFER_READ               0x01
#define SPI_XFER_WRITE              0x02

#define SPI_DEV_TYPE                0x00
#define SPI_NOR_TYPE                0x01
#define SPI_MODALIAS_TYPE           0x02

/* error code */
#define SUCCESS                     0

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
    /** SPI transfer mode */
    uint16_t flags;
    /**
     * Timeout value (milliseconds) for SPI transfer. Timeout value must
     * larger then 0, timeout <=0 means infinite timeout.
     */
    int timeout;
    /**
     * Called to report transaction completion, only for asynchronous SPI
     * transfer.
     */
    void (*complete)(void *context);
    /** The argument to complete() function when it's called */
    void *context;
    /** Return code for asynchronous SPI transfer */
    int status;
};

/**
 * SPI hardware capabilities info
 */
struct master_spi_caps {
    /** number of bits per word supported */
    uint32_t bpw;
    /** minimum Transfer speed in Hz */
    uint32_t min_speed_hz;
    /** maximum Transfer speed in Hz */
    uint32_t max_speed_hz;
    /** bit masks of supported SPI protocol mode */
    uint16_t modes;
    /** bit masks of supported SPI protocol flags */
    uint16_t flags;
    /** number of chip select pins supported */
    uint16_t csnum;
    /** maximum divider supported */
    uint16_t max_div;
};

struct device_spi_cfg {
    /** mode be set in device */
    uint16_t mode;
    /** bit per word be set in device */
    uint8_t bpw;
    /** max speed be set in device */
    uint32_t max_speed_hz;
    /** SPI device type */
    uint8_t device_type;
    /** chip name */
    uint8_t name[32];
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
    int (*get_master_caps)(struct device *dev, struct master_spi_caps *caps);
    /** Get configuration parameters from chip */
    int (*get_device_cfg)(struct device *dev, uint8_t cs,
                          struct device_spi_cfg *dev_cfg);
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
 * @brief SPI getcaps wrap function
 *
 * @param dev pointer to structure of device data
 * @param caps pointer to the spi_caps structure to receive the capabilities
 *             information.
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_get_master_caps(struct device *dev,
                                             struct master_spi_caps *caps)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->get_master_caps) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->get_master_caps(dev, caps);
    }
    return -ENOSYS;
}

/**
 * @brief SPI device configuration
 *
 * @param dev pointer to structure of device data
 * @param cs required chip number
 * @param dev_cfg pointer to the device_spi_cfg structure to receive the
 *                specific chip of configuration.
 * @return 0 on success, negative errno on error
 */
static inline int device_spi_get_device_cfg(struct device *dev, uint8_t cs,
                                            struct device_spi_cfg *dev_cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->get_master_caps) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->get_device_cfg(dev, cs,
                                                               dev_cfg);
    }
    return -ENOSYS;
}

#endif /* __ARCH_ARM_DEVICE_SPI_H */
