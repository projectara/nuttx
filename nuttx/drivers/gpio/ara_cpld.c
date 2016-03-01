/**
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
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/gpio.h>
#include <nuttx/gpio_chip.h>
#include <nuttx/gpio/ara_cpld.h>
#include <nuttx/i2c.h>

#include <errno.h>
#include <syslog.h>

/* Line counts */
#define ARA_CPLD_CLK_LINE_COUNT     9
#define ARA_CPLD_WAKE_LINE_COUNT    0 /* not implemented yet */

#define ARA_CPLD_LINE_COUNT (ARA_CPLD_CLK_LINE_COUNT + ARA_CPLD_WAKE_LINE_COUNT)

/* GPIO bases */
#define ARA_CPLD_CLK_LINE_BASE      0
#define ARA_CPLD_WAKE_LINE_BASE     ARA_CPLD_CLK_LINE_COUNT

/* Registers */
#define REFCLK_REG_0                0x2

/* Electrical */
#define RESET_HOLD_TIME_IN_USEC     100

static int ara_cpld_gpio2reg(uint8_t which)
{
    if (which >= ARA_CPLD_LINE_COUNT) {
        return -EINVAL;
    }

    if (which >= ARA_CPLD_CLK_LINE_BASE &&
        which < ARA_CPLD_CLK_LINE_BASE + ARA_CPLD_CLK_LINE_COUNT) {
        return REFCLK_REG_0 + ((which - ARA_CPLD_CLK_LINE_BASE) % 8);
    }

    return -EINVAL;
}

static int ara_cpld_read(struct ara_cpld_pdata *pdata, uint8_t reg,
                         uint8_t *value)
{
    struct i2c_msg_s msg[] = {
        {
            .addr = pdata->i2c_addr,
            .buffer = &reg,
            .length = 1,
        },
        {
            .addr = pdata->i2c_addr,
            .flags = I2C_M_READ,
            .buffer = value,
            .length = 1,
        },
    };

    I2C_SETADDRESS(pdata->i2c_dev, pdata->i2c_addr, 7);
    return I2C_TRANSFER(pdata->i2c_dev, msg, 2);
}

static int ara_cpld_write(struct ara_cpld_pdata *pdata, uint8_t reg,
                          uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    struct i2c_msg_s msg[] = {
        {
            .addr = pdata->i2c_addr,
            .buffer = buf,
            .length = 2,
        },
    };

    I2C_SETADDRESS(pdata->i2c_dev, pdata->i2c_addr, 7);
    return I2C_TRANSFER(pdata->i2c_dev, msg, 1);
}

static uint8_t ara_cpld_get_value(void *drv, uint8_t which)
{
    uint8_t val;
    int retval;
    int reg;

    reg = ara_cpld_gpio2reg(which);
    if (reg < 0) {
        lowsyslog("%s(): invalid gpio %hhu\n", __func__, which);
        return 0;
    }

    retval = ara_cpld_read(drv, reg, &val);
    if (retval) {
        lowsyslog("%s(): failed to read gpio value: %hhu\n", __func__, which);
        return 0;
    }

    return val & (1 << (which % 8));
}

static void ara_cpld_set_value(void *drv, uint8_t which, uint8_t value)
{
    uint8_t val;
    int retval;
    int reg;

    reg = ara_cpld_gpio2reg(which);
    if (reg < 0) {
        lowsyslog("%s(): invalid gpio %hhu\n", __func__, which);
        return;
    }

    retval = ara_cpld_read(drv, reg, &val);
    if (retval) {
        lowsyslog("%s(): failed to read gpio value: %hhu\n", __func__, which);
        return;
    }

    if (value) {
        val |= 1 << (which % 8);
    } else {
        val &= ~(1 << (which % 8));
    }

    retval = ara_cpld_write(drv, reg, val);
    if (retval) {
        lowsyslog("%s(): failed to write gpio value: %hhu\n", __func__, which);
        return;
    }
}

static int ara_cpld_get_direction(void *drv, uint8_t which)
{
    /*
     * TODO: as long as we only support the REFCLK IOs, we only support one
     * direction: out
     */
    return 1;
}

static void ara_cpld_set_direction_out(void *drv, uint8_t which, uint8_t value)
{
    /*
     * TODO: for the wake lines, we must implement the direction change, but
     * as of today wake is not supported and we don't have any direction
     * registers
     */
    ara_cpld_set_value(drv, which, value);
}

static int ara_cpld_activate(void *drv, uint8_t which)
{
    return 0;
}

static int ara_cpld_deactivate(void *drv, uint8_t which)
{
    return 0;
}

static uint8_t ara_cpld_line_count(void *drv)
{
    return ARA_CPLD_LINE_COUNT;
}

static struct gpio_ops_s ara_cpld_gpio_ops = {
    .get_value = ara_cpld_get_value,
    .set_value = ara_cpld_set_value,

    .get_direction = ara_cpld_get_direction,
    .direction_out = ara_cpld_set_direction_out,

    .activate = ara_cpld_activate,
    .deactivate = ara_cpld_deactivate,

    .line_count = ara_cpld_line_count,
};

int ara_cpld_register(struct ara_cpld_pdata *pdata)
{
    if (!pdata) {
        return -EINVAL;
    }

    pdata->i2c_dev = up_i2cinitialize(pdata->i2c_bus);
    if (!pdata->i2c_dev) {
        lowsyslog("%s(): Failed to get I2C bus %u\n", __func__, pdata->i2c_bus);
        return -ENODEV;
    }

    gpio_direction_out(pdata->reset, 0);
    up_udelay(RESET_HOLD_TIME_IN_USEC);
    gpio_direction_out(pdata->reset, 1);

    return register_gpio_chip(&ara_cpld_gpio_ops, pdata->base, pdata);
}

int ara_cpld_unregister(struct ara_cpld_pdata *pdata)
{
    if (!pdata) {
        return -EINVAL;
    }

    up_i2cuninitialize(pdata->i2c_dev);

    gpio_direction_out(pdata->reset, 0); /* Put CPLD back in reset */
    return unregister_gpio_chip(pdata);
}
