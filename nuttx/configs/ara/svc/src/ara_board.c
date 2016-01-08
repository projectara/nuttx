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
 */

/**
 * @brief: Functions and definitions for boards management. A unified image
 *         is built for various boards. The board information is assigned
 *         using HWID detection.
 *
 * @author: Jean Pihet
 */

#define DBG_COMP ARADBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/gpio.h>

#include <errno.h>

#include "stm32.h"
#include <ara_debug.h>
#include <ara_board.h>
#include "vreg.h"

static struct ara_board_info *board_info;

static bool read_hwid_bit(uint32_t gpio)
{
    bool level;

    /*
     * Configure the pin as input pull-up. A '0' state if forced by installing
     * a pull-down resistor.
     */
    stm32_configgpio(gpio | GPIO_PULLUP);

    /* Read level */
    level = stm32_gpioread(gpio);

    /*
     * To minimize power consumption, if level reads '0' configure the pin
     * as input pull-down.
     */
    if (!level) {
        stm32_configgpio(gpio | GPIO_PULLDOWN);
    }

    return level;
}

enum hwid board_get_hwid(void) {
    int hwid_read;

    hwid_read = ((read_hwid_bit(HWID_0) << 0) |
                 (read_hwid_bit(HWID_1) << 1) |
                 (read_hwid_bit(HWID_2) << 2) |
                 (read_hwid_bit(HWID_3) << 3));

    switch (hwid_read) {
    case 0b1111:
        return HWID_DB3_0_1;
    case 0b1110:
        return HWID_DB3_2;
    case 0b0001:
        return HWID_DB3_5;
    case 0b0010:
        return HWID_EVT1;
    case 0b0011:
        return HWID_EVT1_5;
    default:
        dbg_error("Unknown HWID 0x%x, aborting\n", hwid_read);
        break;
    }

    return ERROR;
}

struct ara_board_info *ara_board_init(void) {
    int i;
    enum hwid hwid = board_get_hwid();
    board_info = NULL;

    /* Check HWID, assign board_info struct and the Switch ES revision */
    switch (hwid) {
    case HWID_DB3_0_1:
        dbg_info("HWID found as DB3.0/3.1\n");
        break;
    case HWID_DB3_2:
        dbg_info("HWID found as DB3.2\n");
        break;
    case HWID_DB3_5:
        dbg_info("HWID found as DB3.5\n");
        break;
    case HWID_EVT1_5:
        dbg_info("HWID found as EVT1.5\n");
        break;
    default:
        return NULL;
    }

    /* Disable the I/O Expanders for now */
    for (i = 0; i < board_info->nr_io_expanders; i++) {
        struct io_expander_info *io_exp = &board_info->io_expanders[i];
        stm32_configgpio(io_exp->reset);
        stm32_gpiowrite(io_exp->reset, false);
    }

    /*
     * Register the STM32 GPIOs to Gpio Chip
     *
     * This needs to happen before the I/O Expanders registration, which
     * uses some STM32 pins
     */
    stm32_gpio_init();

    /* Register the TCA64xx I/O Expanders GPIOs to Gpio Chip */
    for (i = 0; i < board_info->nr_io_expanders; i++) {
        struct io_expander_info *io_exp = &board_info->io_expanders[i];

        io_exp->i2c_dev = up_i2cinitialize(io_exp->i2c_bus);
        if (!io_exp->i2c_dev) {
            dbg_error("%s(): Failed to get I/O Expander I2C bus %u\n",
                      __func__, io_exp->i2c_bus);
            goto err_deinit_gpio;
        } else {
            if (tca64xx_init(&io_exp->io_exp_driver_data,
                             io_exp->part,
                             io_exp->i2c_dev,
                             io_exp->i2c_addr,
                             io_exp->reset,
                             io_exp->irq,
                             io_exp->gpio_base) < 0) {
                dbg_error("%s(): Failed to register I/O Expander(0x%02x)\n",
                          __func__, io_exp->i2c_addr);
                goto err_uninit_i2c;
            }
        }
    }

    /* Run the board specific code */
    if (board_info->board_init) {
        if (board_info->board_init(board_info)) {
            dbg_error("%s(): Failed to initalize board\n", __func__);
            goto err_uninit_i2c;
        }
    }

    /* Return the board specific info */
    return board_info;

 err_uninit_i2c:
    /* Done in reverse order to account for possible IRQ chaining. */
    for (i = board_info->nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &board_info->io_expanders[i];
        if (io_exp->i2c_dev) {
            up_i2cuninitialize(io_exp->i2c_dev);
        }
    }
 err_deinit_gpio:
    stm32_gpio_deinit();
    /* Leave the I/O expanders in reset here. */

    return NULL;
}

void ara_board_exit(void) {
    int i;

    /* Run the board specific code */
    if (board_info->board_exit) {
        board_info->board_exit(board_info);
    }

    /*
     * First unregister the TCA64xx I/O Expanders and associated I2C bus(ses).
     * Done in reverse order from registration to account for IRQ chaining
     * between I/O Expander chips.
     */
    for (i = board_info->nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &board_info->io_expanders[i];

        if (io_exp->io_exp_driver_data)
            tca64xx_deinit(io_exp->io_exp_driver_data);

        if (io_exp->i2c_dev)
            up_i2cuninitialize(io_exp->i2c_dev);
    }

    /* Lastly unregister the GPIO Chip driver */
    stm32_gpio_deinit();

    board_info = NULL;
}
