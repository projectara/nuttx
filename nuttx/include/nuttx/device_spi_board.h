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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_SPI_BOARD_HW "spi_board"

struct device_spi_board {
    /** chip name */
    uint8_t name[32];
    /** max speed be set in device */
    uint32_t max_speed_hz;
    /** type be set in device */
    uint8_t device_type;
    /** mode be set in device */
    uint16_t mode;
    /** bit per word be set in device */
    uint8_t bpw;
    /** external chip-select pin */
    uint8_t ext_cs;
    /** default cs pin state when using external chip-select pin */
    uint8_t init_cs_state;
};

/**
 * SPI board device driver operations
 */
struct device_spi_board_type_ops {
    /** Get SPI specific chip configured information. */
    int (*get_device_cfg)(struct device *dev, uint8_t cs,
                          struct device_spi_board *spi_board);
};

/**
 * @brief Get SPI specific chip configured information.
 *
 * @param dev Pointer to structure of device.
 * @param cs the specific chip number.
 * @param spi_board pointer to the device_spi_board structure to receive the
 *                configuration that be set in chip.
 * @return 0 on success, negative errno on error.
 */
static inline int device_spi_board_get_device_cfg(struct device *dev,
                              uint8_t cs, struct device_spi_board *spi_board)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_device_cfg)
        return DEVICE_DRIVER_GET_OPS(dev, spi_board)->get_device_cfg(dev, cs,
                                                                     spi_board);
    return -ENOSYS;
}

#endif
