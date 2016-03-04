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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/gpio.h>
#include <nuttx/device_spi_board.h>
#include <nuttx/ara/spi_board.h>

/* SPI board flags */
#define SPI_BOARD_FLAG_OPEN BIT(0)

/**
 * @brief SPI board device private information
 */
struct spi_board_info {
    /** Device driver handler */
    struct device *dev;
    /** SPI board driver state */
    uint32_t flags;
};

/**
 * @brief Initialize SPI board
 *
 * @param dev Pointer to structure of device.
 * @return 0 on success, negative errno on error.
 */
static int spi_board_initialize(struct device *dev)
{
    struct spi_board_init_data *data;
    int ret = 0, i = 0;

    if (!dev || !dev->init_data) {
        return -EINVAL;
    }
    data = dev->init_data;

    /* initialize device */
    if (data->using_gpio) {
        for (i = 0; i < data->num; i++) {
            ret = gpio_activate(data->devices[i].ext_cs);
            if (ret) {
                /* deactive all activated gpio pin */
                while(--i >= 0) {
                    gpio_deactivate(data->devices[i].ext_cs);
                }
                return ret;
            }
            /* set gpio direction and initial output state */
            gpio_direction_out(data->devices[i].ext_cs,
                               data->devices[i].init_cs_state);
        }
    }
    return 0;
}

/**
 * @brief Deinitialize SPI board
 *
 * @param dev Pointer to structure of device.
 * @return 0 on success, negative errno on error.
 */
static int spi_board_deinitialize(struct device *dev)
{
    struct spi_board_init_data *data;
    int i;

    if (!dev || !dev->init_data) {
        return -EINVAL;
    }
    data = dev->init_data;

    /* deinitialize device */
    if (data->using_gpio) {
        for (i = 0; i < data->num; i++) {
            gpio_deactivate(data->devices[i].ext_cs);
        }
    }
    return 0;
}

/**
 * @brief Get number of SPI device
 *
 * @param dev Pointer to structure of device.
 * @param num number of device.
 * @return 0 on success, negative errno on error.
 */
static int spi_board_get_device_num(struct device *dev, uint8_t *num)
{
    struct spi_board_init_data *data;

    if (!dev || !dev->init_data) {
        return -EINVAL;
    }
    data = dev->init_data;
    *num = data->num;
    return 0;
}

/**
 * @brief Get SPI specific chip configured information.
 *
 * @param dev pointer to structure of device data
 * @param cs the specific chip number
 * @param dev_cfg pointer to the device_spi_cfg structure to receive the
 *                configuration that be set in chip.
 * @return 0 on success, negative errno on error
 */
static int spi_board_get_device_cfg(struct device *dev, uint8_t cs,
                                    struct spi_board_device_cfg *dev_cfg)
{
    struct spi_board_init_data *data;

    if (!dev || !dev->init_data || !dev_cfg) {
        return -EINVAL;
    }
    data = dev->init_data;

    if (cs >= data->num) {
        return -EINVAL;
    }
    memcpy(dev_cfg, &data->devices[cs], sizeof(struct spi_board_device_cfg));
    return 0;
}

/**
 * @brief Check whether using normal gpio instead of internal chip-select
 *
 * @param dev Pointer to structure of device.
 * @param using_gpio whether using normal gpio or not.
 * @return 0 on success, negative errno on error.
 */
static int spi_board_is_using_gpio_cs(struct device *dev, bool *using_gpio)
{
    struct spi_board_init_data *data;

    if (!dev || !dev->init_data) {
        return -EINVAL;
    }
    data = dev->init_data;
    *using_gpio = data->using_gpio;
    return 0;
}

/**
* @brief The device open function.
*
* This function is called when protocol preparing to use the driver ops
* in initial stage. It is called after probe() was invoked by the system.
* The function checks whether the driver is already open or not. If it was
* opened, it returns driver busy error number, otherwise it keeps a flag to
* identify the driver was opened and returns success.
*
* @param dev Pointer to the SPI board device structure.
* @return 0 for success, negative errno on error
*/
static int spi_board_dev_open(struct device *dev)
{
    struct spi_board_info *info = NULL;
    int ret = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (info->flags & SPI_BOARD_FLAG_OPEN) {
        ret = -EBUSY;
        return ret;
    }

    /* initialize SPI board config */
    ret = spi_board_initialize(dev);
    if (ret) {
        return ret;
    }
    info->flags |= SPI_BOARD_FLAG_OPEN;
    return ret;
}

/**
* @brief The device close function.
*
* This function is called when protocol no longer using this driver.
* The driver must be opened before calling this function.
*
* @param dev Pointer to the SPI board device structure.
* @return None.
*/
static void spi_board_dev_close(struct device *dev)
{
    struct spi_board_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (!(info->flags & SPI_BOARD_FLAG_OPEN)) {
        return;
    }
    /* deinitialize SPI board config */
    spi_board_deinitialize(dev);

    info->flags &= ~SPI_BOARD_FLAG_OPEN;
}

/**
* @brief The device probe function.
*
* This function is called by the system to allocate memory for saving driver
* internal information data when the system boots up.
*
* @param dev Pointer to the SPI board device structure.
* @return 0 for success, negative errno on error.
*/
static int spi_board_dev_probe(struct device *dev)
{
    struct spi_board_info *info = NULL;

    if (!dev || !dev->init_data) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    device_set_private(dev, info);

    return 0;
}

/**
* @brief The device remove function.
*
* This function is called by the system to unregister this driver. It must be
* called after probe() and open(). It frees the internal information memory
* space.
*
* @param dev Pointer to the SPI board device structure.
* @return None.
*/
static void spi_board_dev_remove(struct device *dev)
{
    struct spi_board_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->flags & SPI_BOARD_FLAG_OPEN) {
        spi_board_dev_close(dev);
    }
    info->flags = 0;

    free(info);
    device_set_private(dev, NULL);
}

static struct device_spi_board_type_ops spi_board_type_ops = {
    .get_device_num = spi_board_get_device_num,
    .get_device_cfg = spi_board_get_device_cfg,
    .is_using_gpio_cs = spi_board_is_using_gpio_cs,
};

static struct device_driver_ops spi_board_driver_ops = {
    .probe    = spi_board_dev_probe,
    .remove   = spi_board_dev_remove,
    .open     = spi_board_dev_open,
    .close    = spi_board_dev_close,
    .type_ops = &spi_board_type_ops,
};

struct device_driver spi_board_driver = {
    .type = DEVICE_TYPE_SPI_BOARD_HW,
    .name = "spi_board",
    .desc = "SPI Board Driver",
    .ops  = &spi_board_driver_ops,
};
