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
 */

/**
 * @brief Driver for simulating GPIOs with a fixed value.
 */

#include <nuttx/gpio_fixed.h>

#include <stdlib.h>
#include <errno.h>

#include <debug.h>
#include <nuttx/list.h>
#include <nuttx/irq.h>
#include <nuttx/gpio_chip.h>

#define LINE_COUNT 1

struct fixed_gpio_drvdata {
    int base;
    uint8_t value;
    struct list_head list;
};

static struct list_head fixed_gpio_drvdata_list =
    LIST_INIT(fixed_gpio_drvdata_list);

static int fixed_gpio_get_direction(void *driver_data, uint8_t which)
{
    DEBUGASSERT(which < LINE_COUNT);
    return 0;
}

static void fixed_gpio_direction_in(void *driver_data, uint8_t which)
{
    DEBUGASSERT(which < LINE_COUNT);
}

static void fixed_gpio_direction_out(void *driver_data, uint8_t which,
                                     uint8_t value)
{
    struct fixed_gpio_drvdata *drvdata = driver_data;
    DEBUGASSERT(which < LINE_COUNT);
    if (value != drvdata->value) {
        lldbg("can't set value to %u; %u is the fixed value\n",
              value, drvdata->value);
    }
}

static int fixed_gpio_activate(void *driver_data, uint8_t which)
{
    int rc = which < LINE_COUNT ? 0 : -EINVAL;
    if (rc) {
        lldbg("invalid gpio %u, max is %u\n", which, LINE_COUNT - 1);
    }
    return rc;
}

static uint8_t fixed_gpio_get_value(void *driver_data, uint8_t which)
{
    struct fixed_gpio_drvdata *drvdata = driver_data;
    DEBUGASSERT(which < LINE_COUNT);
    return drvdata->value;
}

static void fixed_gpio_set_value(void *driver_data, uint8_t which,
                                 uint8_t value)
{
    struct fixed_gpio_drvdata *drvdata = driver_data;
    DEBUGASSERT(which < LINE_COUNT);
    if (value != drvdata->value) {
        lldbg("can't set value to %u; %u is the fixed value\n",
              value, drvdata->value);
    }
}

static int fixed_gpio_deactivate(void *driver_data, uint8_t which)
{
    return which < LINE_COUNT ? 0 : -EINVAL;
}

static uint8_t fixed_gpio_line_count(void *driver_data)
{
    return LINE_COUNT;
}

static int fixed_gpio_irqattach(void *driver_data, uint8_t which, xcpt_t isr,
                                uint8_t base)
{
    return 0;
}

static int fixed_gpio_set_triggering(void *driver_data, uint8_t which,
                                     int trigger)
{
    /*
     * These GPIOs never change value, so any attached edge-triggered
     * interrupt handler would never trigger, and is thus easy to
     * simulate.
     *
     * We can also simulate support for level-triggered interrupts
     * which differ from our constant value, since those would never
     * trigger either.
     *
     * Level-triggered interrupts which have the constant value we're
     * set to are almost certainly not desired by the user, unless
     * they're trying to simulate an interrupt storm. Either way, we
     * don't support it.
     */
    struct fixed_gpio_drvdata *drvdata = driver_data;
    if ((trigger == IRQ_TYPE_LEVEL_HIGH && drvdata->value) ||
        (trigger == IRQ_TYPE_LEVEL_LOW && !drvdata->value)) {
        return -EOPNOTSUPP;
    }
    return 0;
}

static int fixed_gpio_mask_irq(void *driver_data, uint8_t which)
{
    return which < LINE_COUNT ? 0 : -EINVAL;
}

static int fixed_gpio_unmask_irq(void *driver_data, uint8_t which)
{
    return which < LINE_COUNT ? 0 : -EINVAL;
}

static int fixed_gpio_clear_interrupt(void *driver_data, uint8_t which)
{
    return which < LINE_COUNT ? 0 : -EINVAL;
}

static int fixed_gpio_set_pull(void *driver_data, uint8_t pin,
                               enum gpio_pull_type pull_type)

{
    if (pull_type != GPIO_PULL_TYPE_PULL_NONE) {
        return -EOPNOTSUPP;
    }
    return 0;
}

static enum gpio_pull_type fixed_gpio_get_pull(void *driver_data,
                                               uint8_t which)
{
    return GPIO_PULL_TYPE_PULL_NONE;
}

static int fixed_gpio_set_debounce(void *driver_data, uint8_t which,
                                   uint16_t delay)
{
    /* Constant values never bounce. */
    return 0;
}

static struct gpio_ops_s fixed_gpio_ops = {
    .get_direction   = fixed_gpio_get_direction,
    .direction_in    = fixed_gpio_direction_in,
    .direction_out   = fixed_gpio_direction_out,
    .activate        = fixed_gpio_activate,
    .get_value       = fixed_gpio_get_value,
    .set_value       = fixed_gpio_set_value,
    .deactivate      = fixed_gpio_deactivate,
    .line_count      = fixed_gpio_line_count,
    .irqattach       = fixed_gpio_irqattach,
    .set_triggering  = fixed_gpio_set_triggering,
    .mask_irq        = fixed_gpio_mask_irq,
    .unmask_irq      = fixed_gpio_unmask_irq,
    .clear_interrupt = fixed_gpio_clear_interrupt,
    .set_pull        = fixed_gpio_set_pull,
    .get_pull        = fixed_gpio_get_pull,
    .set_debounce    = fixed_gpio_set_debounce,
};

int fixed_gpio_init(uint8_t value, int base)
{
    irqstate_t flags;
    struct fixed_gpio_drvdata *drvdata;
    int rc;
    drvdata = malloc(sizeof(*drvdata));
    if (!drvdata) {
        return -ENOMEM;
    }
    drvdata->base = base;
    drvdata->value = value;
    list_init(&drvdata->list);
    rc = register_gpio_chip(&fixed_gpio_ops, base, drvdata);
    if (rc) {
        free(drvdata);
        return rc;
    }

    flags = irqsave();
    list_add(&fixed_gpio_drvdata_list, &drvdata->list);
    irqrestore(flags);
    return 0;
}

void fixed_gpio_deinit(int base)
{
    irqstate_t flags;
    bool found = false;
    struct fixed_gpio_drvdata *drvdata;
    struct list_head *iter, *iter_next;
    flags = irqsave();
    list_foreach_safe(&fixed_gpio_drvdata_list, iter, iter_next) {
        drvdata = list_entry(iter, struct fixed_gpio_drvdata, list);
        if (drvdata->base == base) {
            found = true;
            list_del(iter);
            break;
        }
    }
    irqrestore(flags);
    if (found) {
        unregister_gpio_chip(drvdata);
    } else {
        lldbg("no fixed GPIO chip at base %d\n", base);
    }
}
