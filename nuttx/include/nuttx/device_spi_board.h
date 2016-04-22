/**
 * Copyright (c) 2016 Google, Inc.
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

#ifndef __INCLUDE_NUTTX_DEVICE_SPI_BOARD_H
#define __INCLUDE_NUTTX_DEVICE_SPI_BOARD_H

/**
 * @file nuttx/device_spi_board.h
 * @brief SPI Board API
 * @attention This file is officially included in the Firmware Documentation.
 * Please contact the Firmware Documentation team before modifying it.
 */

#include <errno.h>
#include <stdint.h>

#include <nuttx/device.h>

/** SPI Board Device type */
#define DEVICE_TYPE_SPI_BOARD_HW    "spi_board"

/** @defgroup SPI_MODE_FLAGS SPI mode flags
 * @{
 */
/** Clock phase (0: sample on first clock, 1: on second) */
#define SPI_MODE_CPHA       0x01
/** Clock polarity (0: clock low on idle, 1: high on idle) */
#define SPI_MODE_CPOL       0x02
/** Chip select active high */
#define SPI_MODE_CS_HIGH    0x04
/** Transmit least significant bits first */
#define SPI_MODE_LSB_FIRST  0x08
/** Use only three wires: SI/SO signals are shared */
#define SPI_MODE_3WIRE      0x10
/** Loopback mode */
#define SPI_MODE_LOOP       0x20
/** Only one slave per bus, no need for chip select */
#define SPI_MODE_NO_CS      0x40
/** Slave pulls low to pause */
#define SPI_MODE_READY      0x80
/** @} */

/** @defgroup SPI_MODE SPI standard modes
 * @{
 */
/** Mode 0 (original MicroWire): CPOL=0 | CPHA=0 */
#define SPI_MODE_0  (0 | 0)
/** Mode 1: CPOL=0 | CPHA=1 */
#define SPI_MODE_1  (0 | SPI_MODE_CPHA)
/** Mode 2: CPOL=1 | CPHA=0 */
#define SPI_MODE_2  (SPI_MODE_CPOL | 0)
/** Mode 3: CPOL=1 | CPHA=1 */
#define SPI_MODE_3  (SPI_MODE_CPOL | SPI_MODE_CPHA)
/** @} */

/** SPI board device type */
enum device_spi_type {
    /** Generic SPI device (translate into 'spidev' on Linux) */
    SPI_DEV_TYPE,
    /** SPI NOR flash device that supports JEDEC READ ID */
    SPI_NOR_TYPE,
    /** SPI device that can be identified by its name */
    SPI_MODALIAS_TYPE,
};

/** SPI board device driver operations */
struct device_spi_board_type_ops {
    /** Get the name of a SPI board
     * @param dev Pointer to the SPI board
     * @param name Buffer to fill out with the name (size must be 32 bytes)
     * @return 0 on success, negative errno on failure
     */
    int (*get_name)(struct device *dev, uint8_t *name);
    /** Get the max speed of a SPI board (in Hz)
     * @param dev Pointer to the SPI board
     * @param max_speed_hz Pointer to a variable whose value is to be filled out
     * with the max speed (in Hz)
     * @return 0 on success, negative errno on failure
     */
    int (*get_max_speed_hz)(struct device *dev, uint32_t *max_speed_hz);
    /** Get the SPI board device type
     * @param dev Pointer to the SPI board
     * @param type Pointer to a variable whose value is to be filled out with
     * the SPI board device type
     * @return 0 on success, negative errno on failure
     */
    int (*get_type)(struct device *dev, enum device_spi_type *type);
    /** Get the SPI board mode
     * @param dev Pointer to the SPI board
     * @param mode Pointer to a variable whose value is to be filled out with
     * the SPI board mode (\ref SPI_MODE and \ref SPI_MODE_FLAGS)
     * @return 0 on success, negative errno on failure
     */
    int (*get_mode)(struct device *dev, uint16_t *mode);
    /** Get the SPI board default word size (i.e. bits per word)
     * @param dev Pointer to the SPI board
     * @param bpw Pointer to a variable whose value is to be filled out with the
     * SPI board default word size
     * @return 0 on success, negative errno on failure
     */
    int (*get_bpw)(struct device *dev, uint8_t *bpw);
    /** Selects the SPI board for a transfer (chip-select emulation)
     * @param dev Pointer to the SPI board
     * @param val Chip-select value that should be applied in order to select
     * this SPI board
     * @return 0 on success, negative errno on failure
     */
    int (*cs_select)(struct device *dev, uint8_t val);
};

/** Get the name of a SPI board
 * @param dev Pointer to the SPI board
 * @param name Buffer to fill out with the name (size must be 32 bytes)
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_board_get_name(struct device *dev, uint8_t *name)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_name)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_name(dev, name);

    return -ENOSYS;
}

/** Get the max speed of a SPI board (in Hz)
 * @param dev Pointer to the SPI board
 * @param max_speed_hz Pointer to a variable whose value is to be filled out
 * with the max speed (in Hz)
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_board_get_max_speed_hz(struct device *dev, uint32_t *max_speed_hz)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_max_speed_hz)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_max_speed_hz(dev, max_speed_hz);

    return -ENOSYS;
}

/** Get the SPI board device type
 * @param dev Pointer to the SPI board
 * @param type Pointer to a variable whose value is to be filled out with the
 * SPI board device type
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_board_get_type(struct device *dev, enum device_spi_type *type)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_type)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_type(dev, type);

    return -ENOSYS;
}

/** Get the SPI board mode
 * @param dev Pointer to the SPI board
 * @param mode Pointer to a variable whose value is to be filled out with the
 * SPI board mode (\ref SPI_MODE and \ref SPI_MODE_FLAGS)
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_board_get_mode(struct device *dev, uint16_t *mode)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_mode)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_mode(dev, mode);

    return -ENOSYS;
}

/** Get the SPI board default word size (i.e. bits per word)
 * @param dev Pointer to the SPI board
 * @param bpw Pointer to a variable whose value is to be filled out with the SPI
 * board default word size
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_board_get_bpw(struct device *dev, uint8_t *bpw)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_bpw)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_bpw(dev, bpw);

    return -ENOSYS;
}

/** Selects the SPI board for a transfer (chip-select emulation)
 * @param dev Pointer to the SPI board
 * @param val Chip-select value that should be applied in order to select this
 * SPI board
 * @return 0 on success, negative errno on failure
 */
static inline int device_spi_board_cs_select(struct device *dev, uint8_t val)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->cs_select)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->cs_select(dev, val);

    return -ENOSYS;
}

#endif
