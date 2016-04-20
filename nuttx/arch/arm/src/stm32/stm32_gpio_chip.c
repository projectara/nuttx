/**
 * Copyright (c) 2014-2016 Google Inc.
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
 * @brief STM32 GPIO Chip Driver
 *
 * STM32 has 16 GPIO pins per port. Pin A0 is pin 0, pin B0 is pin 16 etc.
 * @author Jean Pihet
 */

#include <nuttx/config.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <nuttx/gpio/stm32_gpio_chip.h>
#include <nuttx/gpio.h>
#include <nuttx/gpio_chip.h>
#include <nuttx/util.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"

/* disable the verbose debug output */
#undef lldbg
#define lldbg(x...)

#define STM32_GPIO_FLAG_RISING      BIT(0)
#define STM32_GPIO_FLAG_FALLING     BIT(1)

/* Internal struct for IRQ edges and ISR management */
struct stm32_gpio_internal {
    xcpt_t isr;
    uint8_t flags;
};


/* Private data and functions */
static bool stm32_gpio_chip_initalized = false;
static struct stm32_gpio_internal stm32_gpio[STM32_NGPIO + 1];

/* Map pin number to cfgset used by the STM32 GPIO framework */
static int map_pin_nr_to_cfgset(uint8_t pin, uint32_t *cfgset)
{
    if (pin > STM32_NGPIO)
        return -EINVAL;

    // 16 pins per port
    *cfgset = ((pin / 16) << GPIO_PORT_SHIFT) |
              ((pin % 16) << GPIO_PIN_SHIFT);

    return 0;
}

static void stm32_gpio_set_direction_in(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return;
    }

    // Configure pin as input, preserve pull up/pull down
    cfgset |= GPIO_INPUT | stm32_get_pupd(cfgset);
    lldbg("cfgset=0x%x\n", cfgset);

    ret = stm32_configgpio(cfgset);
    if (ret)
        lldbg("%s: stm32_configgpio returns %d\n", __func__, ret);
}

static void stm32_gpio_set_direction_out(void *driver_data, uint8_t pin,
                                         uint8_t value)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return;
    }

    // Configure pin as output
    if (value)
        cfgset |= GPIO_OUTPUT | GPIO_OUTPUT_SET;
    else
        cfgset |= GPIO_OUTPUT | GPIO_OUTPUT_CLEAR;

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    lldbg("cfgset=0x%x\n", cfgset);

    ret = stm32_configgpio(cfgset);
    if (ret)
        lldbg("%s: stm32_configgpio returns %d\n", __func__, ret);
}

// STM32 GPIO API does not have a direction query function
static int stm32_gpio_get_direction(void *driver_data, uint8_t pin)
{
    return -EOPNOTSUPP;
}

static void stm32_gpio_set(void *driver_data, uint8_t pin, uint8_t val)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu, val=%d\n", __func__, pin, val);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return;
    }

    stm32_gpiowrite(cfgset, val);
}

static uint8_t stm32_gpio_get(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return -EINVAL;
    }

    return stm32_gpioread(cfgset);
}

static uint8_t stm32_gpio_line_count(void *driver_data)
{
    return STM32_NGPIO + 1;
}

static int stm32_gpio_activate(void *driver_data, uint8_t pin)
{
    return 0;
}

// Configure pin as input floating
static int stm32_gpio_deactivate(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return -EINVAL;
    }

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    stm32_unconfiggpio(cfgset);
    return 0;
}

static int stm32_gpio_set_pull(void *driver_data, uint8_t pin,
                             enum gpio_pull_type pull_type)
{
    uint32_t setting;
    uint32_t cfgset;
    int ret;

    lldbg("%s: pin=%hhu, val=%d\n", __func__, pin, pull_type);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return -EINVAL;
    }

    switch (pull_type)
    {
        case GPIO_PULL_TYPE_PULL_DOWN:
            setting = GPIO_PULLDOWN;
            break;

        case GPIO_PULL_TYPE_PULL_UP:
            setting = GPIO_PULLUP;
            break;

        case GPIO_PULL_TYPE_PULL_NONE:
            setting = GPIO_FLOAT;
            break;

        default:
            lldbg("%s: unsupported pull up/pull down type: %d\n",
                __func__, pull_type);
            return -EINVAL;
    }

    stm32_set_pupd(cfgset, setting);

    return 0;
}

static enum gpio_pull_type stm32_gpio_get_pull(void *driver_data, uint8_t pin)
{
    uint32_t cfgset, pupd;
    int ret = 0;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return ret;
    }

    pupd = stm32_get_pupd(cfgset);

    switch (pupd) {
        case GPIO_PULLDOWN:
            return GPIO_PULL_TYPE_PULL_DOWN;
        case GPIO_PULLUP:
            return GPIO_PULL_TYPE_PULL_UP;
        case GPIO_FLOAT:
        default:
            return GPIO_PULL_TYPE_PULL_NONE;
    }
}

static int stm32_gpio_set_debounce(void *driver_data, uint8_t pin,
                                   uint16_t delay)
{
    uint32_t cfgset;
    int ret = 0;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return ret;
    }

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    return stm32_gpiosetdebounce(cfgset, delay);
}

static int stm32_gpio_irqattach(void *driver_data, uint8_t pin, xcpt_t isr,
                                uint8_t base)
{
    uint32_t cfgset;
    int ret = 0;

    lldbg("%s: pin=%hhu, handler=%p\n", __func__, pin, isr);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return ret;
    }

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    /*
     * Install the handler for the pin.
     *
     * The IRQ line config is done in stm32_gpiosetevent.
     * By default cfgset is set as input, floating.
     */
    stm32_gpio[pin].isr = isr;
    stm32_gpiosetevent_priv(cfgset,
                            stm32_gpio[pin].flags & STM32_GPIO_FLAG_RISING,
                            stm32_gpio[pin].flags & STM32_GPIO_FLAG_FALLING,
                            true,
                            stm32_gpio[pin].isr,
                            NULL);

    return ret;
}

static int stm32_gpio_set_triggering(void *driver_data, uint8_t pin,
                                     int trigger)
{
    uint32_t cfgset;
    int ret = 0;

    lldbg("%s: pin=%hhu, trigger=0x%x\n", __func__, pin, trigger);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return ret;
    }

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    switch(trigger) {
    case IRQ_TYPE_NONE:
        stm32_gpio[pin].flags &= ~(STM32_GPIO_FLAG_RISING);
        stm32_gpio[pin].flags &= ~(STM32_GPIO_FLAG_FALLING);
        break;
    case IRQ_TYPE_EDGE_RISING:
        stm32_gpio[pin].flags |= STM32_GPIO_FLAG_RISING;
        stm32_gpio[pin].flags &= ~(STM32_GPIO_FLAG_FALLING);
        break;
    case IRQ_TYPE_EDGE_FALLING:
        stm32_gpio[pin].flags &= ~(STM32_GPIO_FLAG_RISING);
        stm32_gpio[pin].flags |= STM32_GPIO_FLAG_FALLING;
        break;
    case IRQ_TYPE_EDGE_BOTH:
        stm32_gpio[pin].flags |= STM32_GPIO_FLAG_RISING;
        stm32_gpio[pin].flags |= STM32_GPIO_FLAG_FALLING;
        break;
    /* Level IRQ: not supported by low level STM32 gpio support */
    case IRQ_TYPE_LEVEL_HIGH:
    case IRQ_TYPE_LEVEL_LOW:
    default:
        return -EINVAL;
    }

    /* Install handler with edge triggering settings */
    stm32_gpiosetevent_priv(cfgset,
                            stm32_gpio[pin].flags & STM32_GPIO_FLAG_RISING,
                            stm32_gpio[pin].flags & STM32_GPIO_FLAG_FALLING,
                            true,
                            stm32_gpio[pin].isr,
                            NULL);

    return ret;
}

static int stm32_gpio_mask_irq(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret = 0;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return ret;
    }

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    /* Mask interrupt */
    stm32_gpiosetevent_priv(cfgset,
                            false,
                            false,
                            true,
                            stm32_gpio[pin].isr,
                            NULL);

    return ret;
}

static int stm32_gpio_unmask_irq(void *driver_data, uint8_t pin)
{
    uint32_t cfgset;
    int ret = 0;

    lldbg("%s: pin=%hhu\n", __func__, pin);

    ret = map_pin_nr_to_cfgset(pin, &cfgset);
    if (ret) {
        lldbg("%s: Invalid pin %hhu\n", __func__, pin);
        return ret;
    }

    // preserve pull up/pull down
    cfgset |= stm32_get_pupd(cfgset);

    /* Re-install handler */
    stm32_gpiosetevent_priv(cfgset,
                            stm32_gpio[pin].flags & STM32_GPIO_FLAG_RISING,
                            stm32_gpio[pin].flags & STM32_GPIO_FLAG_FALLING,
                            true,
                            stm32_gpio[pin].isr,
                            NULL);

    return ret;
}

static int stm32_gpio_clear_interrupt(void *driver_data, uint8_t which)
{
    /* Clear IRQ: automatically done in STM32 GPIO low-level code */
    return 0;
}

static struct gpio_ops_s stm32_gpio_ops = {
    .direction_in =     stm32_gpio_set_direction_in,
    .direction_out =    stm32_gpio_set_direction_out,
    .get_direction =    stm32_gpio_get_direction,
    .activate =         stm32_gpio_activate,
    .get_value =        stm32_gpio_get,
    .set_value =        stm32_gpio_set,
    .deactivate =       stm32_gpio_deactivate,
    .line_count =       stm32_gpio_line_count,
    .irqattach =        stm32_gpio_irqattach,
    .set_triggering =   stm32_gpio_set_triggering,
    .mask_irq =         stm32_gpio_mask_irq,
    .unmask_irq =       stm32_gpio_unmask_irq,
    .clear_interrupt =  stm32_gpio_clear_interrupt,
    .set_pull =         stm32_gpio_set_pull,
    .get_pull =         stm32_gpio_get_pull,
    .set_debounce =     stm32_gpio_set_debounce,
};

/* Public functions */
void stm32_gpio_init(void)
{
    int i;

    if (stm32_gpio_chip_initalized) {
        lldbg("%s: already initialized, exiting\n", __func__);
        return;
    }

    /* Init internal data: default edge triggering and ISR */
    for (i = 0; i < (STM32_NGPIO + 1); i++) {
        stm32_gpio[i].isr = NULL;
        stm32_gpio[i].flags = 0;
    }

    /* Register to gpio_chip */
    register_gpio_chip(&stm32_gpio_ops, STM32_GPIO_CHIP_BASE, &stm32_gpio_ops);

    stm32_gpio_chip_initalized = true;
}

void stm32_gpio_deinit(void)
{
    if (!stm32_gpio_chip_initalized) {
        lldbg("%s: not initialized, exiting\n", __func__);
        return;
    }

    /* Unregister to gpio_chip */
    unregister_gpio_chip(&stm32_gpio_ops);

    stm32_gpio_chip_initalized = false;
}
