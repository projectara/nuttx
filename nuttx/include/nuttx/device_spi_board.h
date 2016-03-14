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

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_SPI_BOARD_HW    "spi_board"

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

enum device_spi_type {
    /* Normal SPI device */
    SPI_DEV_TYPE,
    /* MTD SPI device */
    SPI_NOR_TYPE,
    /* Fixed name device */
    SPI_MODALIAS_TYPE,
};

/** SPI board device driver operations */
struct device_spi_board_type_ops {
    int (*get_name)(struct device *dev, uint8_t *name);
    int (*get_max_speed_hz)(struct device *dev, uint32_t *max_speed_hz);
    int (*get_type)(struct device *dev, enum device_spi_type *type);
    int (*get_mode)(struct device *dev, uint16_t *mode);
    int (*get_bpw)(struct device *dev, uint8_t *bpw);
    int (*cs_select)(struct device *dev, uint8_t val);
};

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
