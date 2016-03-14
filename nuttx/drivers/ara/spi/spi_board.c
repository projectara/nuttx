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

struct device_spi_board {
    /** external chip-select pin */
    uint8_t cs_gpio;
};

static int spi_board_get_name(struct device *dev, uint8_t *name)
{
    struct spi_board_info *info;

    if (!dev || !device_get_init_data(dev) || !name) {
        return -EINVAL;
    }
    info = device_get_init_data(dev);

    memcpy(name, info->name, sizeof(info->name));
    return 0;
}

static int spi_board_get_max_speed_hz(struct device *dev, uint32_t *max_speed_hz)
{
    struct spi_board_info *info;

    if (!dev || !device_get_init_data(dev) || !max_speed_hz) {
        return -EINVAL;
    }
    info = device_get_init_data(dev);

    *max_speed_hz = info->max_speed_hz;
    return 0;
}

static int spi_board_get_type(struct device *dev, enum device_spi_type *type)
{
    struct spi_board_info *info;

    if (!dev || !device_get_init_data(dev) || !type) {
        return -EINVAL;
    }
    info = device_get_init_data(dev);

    *type = info->type;
    return 0;
}

static int spi_board_get_mode(struct device *dev, uint16_t *mode)
{
    struct spi_board_info *info;

    if (!dev || !device_get_init_data(dev) || !mode) {
        return -EINVAL;
    }
    info = device_get_init_data(dev);

    *mode = info->mode;
    return 0;
}

static int spi_board_get_bpw(struct device *dev, uint8_t *bpw)
{
    struct spi_board_info *info;

    if (!dev || !device_get_init_data(dev) || !bpw) {
        return -EINVAL;
    }
    info = device_get_init_data(dev);

    *bpw = info->bpw;
    return 0;
}

static int spi_board_cs_select(struct device *dev, uint8_t val)
{
    struct device_spi_board *board;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    board = device_get_private(dev);

#if defined(CONFIG_TSB_SPI_GPIO)
    gpio_set_value(board->cs_gpio, val);
#endif
    return 0;
}

static int spi_board_dev_open(struct device *dev)
{
    int ret;
    struct device_spi_board *board;
    struct spi_board_info *info;

    if (!dev || !device_get_private(dev) || !device_get_init_data(dev)) {
        return -EINVAL;
    }
    board = device_get_private(dev);
    info = device_get_init_data(dev);

#if defined(CONFIG_TSB_SPI_GPIO)
    ret = gpio_activate(board->cs_gpio);
    if (ret)
        return ret;
    /* the initial value is deduced from whether the slave is defined as having
     * an active high CS */
    gpio_direction_out(board->cs_gpio, !(info->mode & SPI_MODE_CS_HIGH));
#endif

    return 0;
}

static void spi_board_dev_close(struct device *dev)
{
    struct device_spi_board *board;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    board = device_get_private(dev);

#if defined(CONFIG_TSB_SPI_GPIO)
    gpio_deactivate(board->cs_gpio);
#endif
}

static int spi_board_dev_probe(struct device *dev)
{
    struct device_spi_board *board;
    struct device_resource *r;

    if (!dev || !device_get_init_data(dev)) {
        return -EINVAL;
    }

    board = zalloc(sizeof(*board));
    if (!board) {
        return -ENOMEM;
    }

#if defined(CONFIG_TSB_SPI_GPIO)
    /* get the CS GPIO pin */
    r = device_resource_get(dev, DEVICE_RESOURCE_TYPE_GPIO, 0);
    if (!r) {
        free(board);
        return -EINVAL;
    }
    board->cs_gpio = (uint8_t)r->start;
#endif

    device_set_private(dev, board);

    return 0;
}

static void spi_board_dev_remove(struct device *dev)
{
    struct device_spi_board *board;
    if (!dev || !device_get_private(dev)) {
        return;
    }
    board = device_get_private(dev);
    free(board);
    device_set_private(dev, NULL);
}

static struct device_spi_board_type_ops spi_board_type_ops = {
    .get_name           = spi_board_get_name,
    .get_max_speed_hz   = spi_board_get_max_speed_hz,
    .get_type           = spi_board_get_type,
    .get_mode           = spi_board_get_mode,
    .get_bpw            = spi_board_get_bpw,
    .cs_select          = spi_board_cs_select,
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
