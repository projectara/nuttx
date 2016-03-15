/*
 * Copyright (c) 2015-2016 Google, Inc.
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

#ifndef __INCLUDE_NUTTX_DEVICE_SPI_H
#define __INCLUDE_NUTTX_DEVICE_SPI_H

/**
 * @file nuttx/device_spi.h
 * @brief SPI API
 * @attention This file is officially included in the Firmware Documentation.
 * Please contact the Firmware Documentation team before modifying it.
 */

#include <errno.h>
#include <stdint.h>

#include <nuttx/device.h>
#include <nuttx/device_spi_board.h>

/** SPI Device type */
#define DEVICE_TYPE_SPI_HW  "spi"

/** @defgroup SPI_MASTER_FLAGS SPI Master Flags
 * @{
 */
/** Only supports half-duplex transmission */
#define SPI_FLAG_HALF_DUPLEX    0x0001
/** Cannot do buffer read */
#define SPI_FLAG_NO_RX          0x0002
/** Cannot do buffer write */
#define SPI_FLAG_NO_TX          0x0004
/** @} */

/** SPI transfer structure */
struct device_spi_transfer {
    /** Buffer of data to be sent (NULL if read-only) */
    void *txbuffer;
    /** Buffer of data to be read (NULL if write-only) */
    void *rxbuffer;
    /** Size of data buffers */
    size_t nwords;
};

/** SPI master configuration */
struct device_spi_master_config {
    /** Mask of supported numbers of bits per word */
    uint32_t bpw_mask;
    /** Minimum supported transfer speed (in Hz) */
    uint32_t min_speed_hz;
    /** Maximum supported transfer speed (in Hz) */
    uint32_t max_speed_hz;
    /** Supported SPI modes (\ref SPI_MODE_FLAGS and \ref SPI_MODE) */
    uint16_t mode;
    /** Supported SPI flags (\ref SPI_MASTER_FLAGS) */
    uint16_t flags;
    /** Number of supported SPI slaves */
    uint16_t dev_num;
    /** Maximum supported frequency divider */
    uint16_t max_div;
};

/** SPI slave configuration */
struct device_spi_device_config {
    /** Slave name */
    uint8_t name[32];
    /** Maximum supported transfer speed (in Hz) */
    uint32_t max_speed_hz;
    /** Default SPI mode */
    uint16_t mode;
    /** Default bits per word */
    uint8_t bpw;
    /** SPI device type */
    enum device_spi_type device_type;
};

/** SPI device driver operations */
struct device_spi_type_ops {
    /** Lock the SPI bus (for exclusive access)
     * @param dev Pointer to the SPI master
     * @return 0 on success, negative errno on failure
     */
    int (*lock)(struct device *dev);
    /** Unlock the SPI bus
     * @param dev Pointer to the SPI master
     * @return 0 on success, negative errno on failure
     */
    int (*unlock)(struct device *dev);
    /** Select a SPI slave for a transmission (through its CS pin)
     * @param dev Pointer to the SPI master
     * @param devid The identifier of the SPI slave to select
     * @return 0 on success, negative errno on failure
     */
    int (*select)(struct device *dev, uint8_t devid);
    /** Deselect a SPI slave after a transmission (through its CS pin)
     * @param dev Pointer to the SPI master
     * @param devid The identifier of the SPI slave to deselect
     * @return 0 on success, negative errno on failure
     */
    int (*deselect)(struct device *dev, uint8_t devid);
    /** Configure the frequency before a transmission
     * @param dev Pointer to the SPI master
     * @param devid The identifier of the SPI slave whose frequency is to be
     * configured
     * @param frequency The frequency to use
     * @return 0 on success, negative errno on failure
     */
    int (*setfrequency)(struct device *dev, uint8_t devid, uint32_t *frequency);
    /** Configure the mode before a transmission
     * @param dev Pointer to the SPI master
     * @param devid The identifier of the SPI slave whose mode is to be
     * configured
     * @param mode The mode to use
     * @return 0 on success, negative errno on failure
     */
    int (*setmode)(struct device *dev, uint8_t devid, uint8_t mode);
    /** Configure the number of bits per word before a transmission
     * @param dev Pointer to the SPI master
     * @param devid The identifier of the SPI slave whose number of bits per
     * word is to be configured
     * @param bpw The number of bits per word to use
     * @return 0 on success, negative errno on failure
     */
    int (*setbpw)(struct device *dev, uint8_t devid, uint8_t bpw);
    /** Perform a SPI transmission
     * @param dev Pointer to the SPI master
     * @param transfer Pointer to a SPI transfer structure
     * @return 0 on success, negative errno on failure
     */
    int (*exchange)(struct device *dev, struct device_spi_transfer *transfer);
    /** Get the SPI master configuration
     * @param dev Pointer to the SPI master
     * @param master_cfg Pointer to a variable whose value is to be filled out
     * with the SPI master configuration
     * @return 0 on success, negative errno on failure
     */
    int (*get_master_config)(struct device *dev,
                             struct device_spi_master_config *master_cfg);
    /** Get a SPI slave configuration
     * @param dev Pointer to the SPI master
     * @param devid The identifier of the SPI slave whose configuration is to be
     * returned
     * @param device_cfg Pointer to a variable whose value is to be filled out
     * with the SPI slave configuration
     * @return 0 on success, negative errno on failure
     */
    int (*get_device_config)(struct device *dev, uint8_t devid,
                             struct device_spi_device_config *device_cfg);
};

/** Lock the SPI bus (for exclusive access)
 * @param dev Pointer to the SPI master
 * @return 0 on success, negative errno on failure
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

/** Unlock the SPI bus
 * @param dev Pointer to the SPI master
 * @return 0 on success, negative errno on failure
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

/** Select a SPI slave for a transmission (through its CS pin)
 * @param dev Pointer to the SPI master
 * @param devid The identifier of the SPI slave to select
 * @return 0 on success, negative errno on failure
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

/** Deselect a SPI slave after a transmission (through its CS pin)
 * @param dev Pointer to the SPI master
 * @param devid The identifier of the SPI slave to deselect
 * @return 0 on success, negative errno on failure
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

/** Configure the frequency before a transmission
 * @param dev Pointer to the SPI master
 * @param devid The identifier of the SPI slave whose frequency is to be
 * configured
 * @param frequency The frequency to use
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_setfrequency(struct device *dev, uint8_t devid,
                                          uint32_t *frequency)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->setfrequency) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->setfrequency(dev, devid,
                                                             frequency);
    }
    return -ENOSYS;
}

/** Configure the mode before a transmission
 * @param dev Pointer to the SPI master
 * @param devid The identifier of the SPI slave whose mode is to be configured
 * @param mode The mode to use
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_setmode(struct device *dev, uint8_t devid,
                                     uint8_t mode)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->setmode) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->setmode(dev, devid, mode);
    }
    return -ENOSYS;
}

/** Configure the number of bits per word before a transmission
 * @param dev Pointer to the SPI master
 * @param devid The identifier of the SPI slave whose number of bits per word is to
 * be configured
 * @param bpw The number of bits per word to use
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_setbpw(struct device *dev, uint8_t devid,
                                     uint8_t bpw)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->setbpw) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->setbpw(dev, devid, bpw);
    }
    return -ENOSYS;
}

/** Perform a SPI transmission
 * @param dev Pointer to the SPI master
 * @param transfer Pointer to a SPI transfer structure
 * @return 0 on success, negative errno on failure
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

/** Get the SPI master configuration
 * @param dev Pointer to the SPI master
 * @param master_cfg Pointer to a variable whose value is to be filled out with
 * the SPI master configuration
 * @return 0 on success, negative errno on failure
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

/** Get a SPI slave configuration
 * @param dev Pointer to the SPI master
 * @param devid The identifier of the SPI slave whose configuration is to be
 * returned
 * @param device_cfg Pointer to a variable whose value is to be filled out with
 * the SPI slave configuration
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_get_device_config(struct device *dev, uint8_t devid,
                                               struct device_spi_device_config *device_cfg)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, spi)->get_device_config) {
        return DEVICE_DRIVER_GET_OPS(dev, spi)->get_device_config(dev, devid,
                                                                  device_cfg);
    }
    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_SPI_H */
