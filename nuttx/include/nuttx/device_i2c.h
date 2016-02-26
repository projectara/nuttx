/*
 * Copyright (c) 2016 Google Inc.
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
 *
 */

#ifndef __INCLUDE_NUTTX_DEVICE_I2C_H
#define __INCLUDE_NUTTX_DEVICE_I2C_H

/**
 * @file nuttx/device_i2c.h
 * @brief I2C API
 * @author Joel Porquet
 */

#include <errno.h>
#include <stdint.h>

#include <nuttx/device.h>

/** I2C Device type */
#define DEVICE_TYPE_I2C_HW  "i2c"

/** @defgroup I2C_FLAGS I2C flags
 * @{
 */
/** Read data, from slave to master */
#define I2C_FLAG_READ       0x0001
/** Ten bit addressing */
#define I2C_FLAG_TEN        0x0002
/** Do not start transfer with (re-)start */
#define I2C_FLAG_NORESTART  0x0080
/** @} */

/** I2C request structure */
struct device_i2c_request {
    /** Slave address */
    uint16_t addr;
    /** \ref I2C_FLAGS */
    uint16_t flags;
    /** Buffer for data to transfer (read or write) */
    uint8_t *buffer;
    /** Size of the buffer */
    int length;
};

/** I2C device driver operations */
struct device_i2c_type_ops {
    /** Perform I2C transfers
     * @param dev Pointer to the I2C device controller
     * @param requests List of I2C requests
     * @param count Number of I2C requests
     * @return 0 on success, negative errno on failure
     */
    int (*transfer)(struct device *dev, struct device_i2c_request *requests,
            uint32_t count);
};

/**
 * @brief Perform I2C transfers
 *
 * @param dev Pointer to the I2C device controller
 * @param requests List of I2C requests
 * @param count Number of I2C requests
 * @return 0 on success, negative errno on failure
 */
static inline int device_i2c_transfer(struct device *dev, struct
        device_i2c_request *requests, uint32_t count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (!DEVICE_DRIVER_GET_OPS(dev, i2c)->transfer) {
        return -ENOSYS;
    }
    return DEVICE_DRIVER_GET_OPS(dev, i2c)->transfer(dev, requests, count);
}

#endif /* __INCLUDE_NUTTX_DEVICE_I2C_H */
