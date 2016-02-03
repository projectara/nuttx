/*
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
 * Authors: Fabien Parent <fparent@baylibre.com>
 *          Benoit Cousson <bcousson@baylibre.com>
 */

#include <nuttx/gpio.h>
#include <nuttx/gpio_chip.h>
<<<<<<< HEAD
#include <nuttx/power/pm.h>
=======
#include <nuttx/gpio/debounce.h>
>>>>>>> GPIO: TSB: add debouncing to TSB GPIO driver
#include <arch/tsb/gpio.h>
#include <arch/tsb/pm.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <stdlib.h>

#include "tsb_scm.h"
#include "up_arch.h"
#include "nuttx/arch.h"
#include "irq/irq.h"

#define ES2_APBRIDGE_LINE_COUNT     23
#define ES2_GPBRIDGE_LINE_COUNT     27
#define ES3_APBRIDGE_LINE_COUNT     24
#define ES3_GPBRIDGE_LINE_COUNT     32

#define GPIO_BASE           0x40003000
#define GPIO_DATA           (GPIO_BASE)
#define GPIO_ODATA          (GPIO_BASE + 0x4)
#define GPIO_ODATASET       (GPIO_BASE + 0x8)
#define GPIO_ODATACLR       (GPIO_BASE + 0xc)
#define GPIO_DIR            (GPIO_BASE + 0x10)
#define GPIO_DIROUT         (GPIO_BASE + 0x14)
#define GPIO_DIRIN          (GPIO_BASE + 0x18)
#define GPIO_INTMASK        (GPIO_BASE + 0x1c)
#define GPIO_INTMASKSET     (GPIO_BASE + 0x20)
#define GPIO_INTMASKCLR     (GPIO_BASE + 0x24)
#define GPIO_RAWINTSTAT     (GPIO_BASE + 0x28)
#define GPIO_INTSTAT        (GPIO_BASE + 0x2c)
#define GPIO_INTCTRL0       (GPIO_BASE + 0x30)
#define GPIO_INTCTRL1       (GPIO_BASE + 0x34)
#define GPIO_INTCTRL2       (GPIO_BASE + 0x38)
#define GPIO_INTCTRL3       (GPIO_BASE + 0x3c)

#define TSB_IO_PULL_UPDOWN0         0x40000A10
#define TSB_IO_PULL_UPDOWN_ENABLE0  0x40000A20
#define TSB_IO_PULL_UP_ENABLE0      0x40000A30

#define TSB_GPIO_ACTIVITY       9

/* Structure for storing interrupt handlers and debounce data for each GPIO */
struct tsb_gpio_irq_vectors_s {
    xcpt_t irq_vector;
    uint8_t irq_gpio_base;
    struct debounce_data debounce;
    int irq_trigger_type;
};

/* A table of handlers and debounce data for each GPIO interrupt */
static struct tsb_gpio_irq_vectors_s *tsb_gpio_irq_vectors;
static volatile uint32_t refcount;

/* GPIO state */
#define TSB_GPIO_FLAG_OPEN BIT(0)
static uint8_t gpio_state[32];

/* Internal pinsharing info */
static uint8_t pinshare_bit_count[32];
static uint32_t currently_owned;

/* Pinsharing configuration (ES2/ES3, Apbridge, Gpbridge) */
struct gpio_pinsharing_conf {
    uint8_t count:4;
    struct {
        uint8_t num:5;
        uint8_t sts:1;
    } pin[2];
} __attribute__ ((packed));

#define PIN_CLR_ST  0
#define PIN_SET_ST  1

#define PIN_NONE                {0}
#define PIN_CLR_1(pin)          {1, {{pin,   PIN_CLR_ST},}}
#define PIN_CLR_1(pin)          {1, {{pin,   PIN_CLR_ST},}}
#define PIN_SET_1(pin)          {1, {{pin,   PIN_SET_ST},}}
#define PIN_CLR_2(pin1, pin2)   {2, {{pin1,  PIN_CLR_ST}, {pin2,   PIN_CLR_ST}}}
#define PIN_CLR_SET(pin1, pin2) {2, {{pin1,  PIN_CLR_ST}, {pin2,   PIN_SET_ST}}}

static struct gpio_pinsharing_conf tsb_gpb_gpio_pinsharing[] = {
    /* GPIO0 */
    PIN_CLR_1(TSB_PIN_UART_CTSRTS_BIT),
    /* GPIO1, GPIO2 */
    PIN_CLR_1(TSB_PIN_UART_RXTX_BIT), PIN_CLR_1(TSB_PIN_UART_RXTX_BIT),
    /* GPIO3, GPIO4, GPIO5 */
    PIN_CLR_1(TSB_PIN_SDIO_BIT), PIN_CLR_1(TSB_PIN_SDIO_BIT), PIN_CLR_1(TSB_PIN_SDIO_BIT),
    /* GPIO6 */
    PIN_CLR_2(TSB_PIN_SDIO_BIT, TSB_PIN_SPIM_CS1_BIT),
    /* GPIO7, GPIO8 */
    PIN_CLR_1(TSB_PIN_SDIO_BIT), PIN_CLR_1(TSB_PIN_SDIO_BIT),
    /* GPIO9 */
    PIN_SET_1(TSB_PIN_GPIO9_BIT),
    /* GPIO10, GPIO11, GPIO12 */
    PIN_SET_1(TSB_PIN_GPIO10_BIT), PIN_SET_1(TSB_PIN_GPIO10_BIT), PIN_SET_1(TSB_PIN_GPIO10_BIT),
    /* GPIO13, GPIO14 */
    PIN_SET_1(TSB_PIN_GPIO13_BIT), PIN_SET_1(TSB_PIN_GPIO13_BIT),
    /* GPIO15 */
    PIN_SET_1(TSB_PIN_GPIO15_BIT),
    /* GPIO16, GPIO17 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO16_BIT), PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO16_BIT),
    /* GPIO18 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO18_BIT),
    /* GPIO19 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO19_BIT),
    /* GPIO20 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO20_BIT),
    /* GPIO21 */
    PIN_CLR_SET(TSB_PIN_I2C_BIT, TSB_PIN_GPIO21_BIT),
    /* GPIO22 */
    PIN_CLR_SET(TSB_PIN_I2C_BIT, TSB_PIN_GPIO22_BIT),
    /* GPIO23, GPIO24, GPIO25, GPIO26 */
    PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
    /* GPIO27, GPIO28 */
    PIN_SET_1(TSB_PIN_GPIO27_BIT), PIN_SET_1(TSB_PIN_GPIO27_BIT),
    /* GPIO29 */
    PIN_SET_1(TSB_PIN_GPIO29_BIT),
    /* GPIO30 */
    PIN_NONE,
    /* GPIO31 */
    PIN_SET_1(TSB_PIN_GPIO31_BIT),
};

static struct gpio_pinsharing_conf tsb_apb_gpio_pinsharing[] = {
    /* GPIO0 */
    PIN_CLR_1(TSB_PIN_UART_RXTX_BIT),
    /* GPIO1, GPIO2 */
    PIN_CLR_1(TSB_PIN_UART_CTSRTS_BIT), PIN_CLR_1(TSB_PIN_UART_CTSRTS_BIT),
    /* GPIO3, GPIO4, GPIO5 */
    PIN_NONE, PIN_NONE, PIN_NONE,
    /* GPIO6 */
    PIN_CLR_1(TSB_PIN_SPIM_CS1_BIT),
    /* GPIO7, GPIO8 */
    PIN_NONE, PIN_NONE,
    /* GPIO9 */
    PIN_SET_1(TSB_PIN_GPIO9_BIT),
    /* GPIO10, GPIO11, GPIO12 */
    PIN_SET_1(TSB_PIN_GPIO10_BIT), PIN_SET_1(TSB_PIN_GPIO10_BIT), PIN_SET_1(TSB_PIN_GPIO10_BIT),
    /* GPIO13, GPIO14 */
    PIN_SET_1(TSB_PIN_GPIO13_BIT), PIN_SET_1(TSB_PIN_GPIO13_BIT),
    /* GPIO15 */
    PIN_SET_1(TSB_PIN_GPIO15_BIT),
    /* GPIO16, GPIO17 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO16_BIT), PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO16_BIT),
    /* GPIO18 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO18_BIT),
    /* GPIO19 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO19_BIT),
    /* GPIO20 */
    PIN_CLR_SET(TSB_PIN_ETM_BIT, TSB_PIN_GPIO20_BIT),
    /* GPIO21 */
    PIN_SET_1(TSB_PIN_GPIO21_BIT),
    /* GPIO22 */
    PIN_SET_1(TSB_PIN_GPIO22_BIT),
    /* GPIO23 */
    PIN_NONE,
};

static struct gpio_pinsharing_conf *tsb_gpio_pinsharing;

static uint8_t tsb_gpio_get_value(void *driver_data, uint8_t which)
{
    pm_activity(TSB_GPIO_ACTIVITY);
    return !!(getreg32(GPIO_DATA) & (1 << which));
}

static void tsb_gpio_set_value(void *driver_data, uint8_t which, uint8_t value)
{
    pm_activity(TSB_GPIO_ACTIVITY);
    putreg32(1 << which, value ? GPIO_ODATASET : GPIO_ODATACLR);
}

static int tsb_gpio_get_direction(void *driver_data, uint8_t which)
{
    uint32_t dir = getreg32(GPIO_DIR);
    pm_activity(TSB_GPIO_ACTIVITY);
    return !(dir & (1 << which));
}

static void tsb_gpio_direction_in(void *driver_data, uint8_t which)
{
    pm_activity(TSB_GPIO_ACTIVITY);
    putreg32(1 << which, GPIO_DIRIN);
}

static void tsb_gpio_direction_out(void *driver_data, uint8_t which, uint8_t value)
{
    pm_activity(TSB_GPIO_ACTIVITY);
    tsb_gpio_set_value(NULL, which, value);
    putreg32(1 << which, GPIO_DIROUT);
}

uint8_t tsb_gpio_line_count(void *driver_data)
{
    switch (tsb_get_rev_id()) {
    case tsb_rev_es2:
        return tsb_get_product_id() == tsb_pid_apbridge ?
            ES2_APBRIDGE_LINE_COUNT : ES2_GPBRIDGE_LINE_COUNT;

    case tsb_rev_es3:
        return tsb_get_product_id() == tsb_pid_apbridge ?
            ES3_APBRIDGE_LINE_COUNT : ES3_GPBRIDGE_LINE_COUNT;

    default:
        return 0;
    }
}

static int tsb_gpio_set_pull(void *driver_data, uint8_t which,
                             enum gpio_pull_type pull_type)
{
    if ( which == 21 || which == 22 ) {
        return -EINVAL; // no pull resistors availabe for GPIO21 or GPIO22
    }

    if ( which >= tsb_gpio_line_count(NULL) ) {
        return -EINVAL; // GPIO line number out of range
    }

    if ( which >= 3 && which <= 8) {
        switch (pull_type) {
            case GPIO_PULL_TYPE_PULL_DOWN:
                return -EINVAL; // no pulldown resistors for GPIO3:GPIO8
            case GPIO_PULL_TYPE_PULL_UP:
                modifyreg32(TSB_IO_PULL_UP_ENABLE0, 0, 1 << which);
                return 0;
            case GPIO_PULL_TYPE_PULL_NONE:
                modifyreg32(TSB_IO_PULL_UP_ENABLE0, 1 << which, 0);
                return 0;
            default:
                return -EINVAL;
        }
    }

    switch (pull_type) {
        case GPIO_PULL_TYPE_PULL_DOWN:
            modifyreg32(TSB_IO_PULL_UPDOWN0, 1 << which, 0);
            modifyreg32(TSB_IO_PULL_UPDOWN_ENABLE0, 1 << which, 0);
            return 0;
        case GPIO_PULL_TYPE_PULL_UP:
            modifyreg32(TSB_IO_PULL_UPDOWN0, 0, 1 << which);
            modifyreg32(TSB_IO_PULL_UPDOWN_ENABLE0, 1 << which, 0);
            return 0;
        case GPIO_PULL_TYPE_PULL_NONE:
            modifyreg32(TSB_IO_PULL_UPDOWN_ENABLE0, 0, 1 << which);
        default:
            return -EINVAL;
    }
}

static enum gpio_pull_type tsb_gpio_get_pull(void *driver_data, uint8_t which)
{
    uint32_t pull_type = 0;

    if ( which == 21 || which == 22 ) {
        return GPIO_PULL_TYPE_PULL_NONE;    // no pull resistors for GPIO21:22
    }

    if ( which >= tsb_gpio_line_count(NULL) ) {
        return GPIO_PULL_TYPE_PULL_NONE;    // gpio line number out of bounds
    }

    if ( which >= 3 && which <= 8 ) {
        pull_type = getreg32(TSB_IO_PULL_UP_ENABLE0) & (1 << which);
        return pull_type ? GPIO_PULL_TYPE_PULL_UP : GPIO_PULL_TYPE_PULL_NONE;
    }

    pull_type = getreg32(TSB_IO_PULL_UPDOWN_ENABLE0) & (1 << which);
    if (pull_type) {
        return GPIO_PULL_TYPE_PULL_NONE;
    }

    pull_type = getreg32(TSB_IO_PULL_UPDOWN0) & (1 << which);
    return pull_type ? GPIO_PULL_TYPE_PULL_UP : GPIO_PULL_TYPE_PULL_DOWN;
}

static int tsb_gpio_mask_irq(void *driver_data, uint8_t which)
{
    putreg32(1 << which, GPIO_INTMASKSET);
    return 0;
}

static int tsb_gpio_unmask_irq(void *driver_data, uint8_t which)
{
    putreg32(1 << which, GPIO_INTMASKCLR);
    return 0;
}

static int tsb_gpio_clear_interrupt(void *driver_data, uint8_t which)
{
    putreg32(1 << which, GPIO_RAWINTSTAT);
    return 0;
}

uint32_t tsb_gpio_get_raw_interrupt(void)
{
    return getreg32(GPIO_RAWINTSTAT);
}

uint32_t tsb_gpio_get_interrupt(void)
{
    return getreg32(GPIO_INTSTAT);
}

static int tsb_set_gpio_triggering(void *driver_data, uint8_t which, int trigger)
{
    int tsb_trigger;
    uint32_t reg = GPIO_INTCTRL0 + ((which >> 1) & 0xfc);
    uint32_t shift = 4 * (which & 0x7);
    uint32_t v = getreg32(reg);

    if (which >= tsb_nr_gpio()) {
        return -EINVAL;
    }

    switch(trigger) {
    case IRQ_TYPE_EDGE_RISING:
        tsb_trigger = TSB_IRQ_TYPE_EDGE_RISING;
        break;
    case IRQ_TYPE_EDGE_FALLING:
        tsb_trigger = TSB_IRQ_TYPE_EDGE_FALLING;
        break;
    case IRQ_TYPE_EDGE_BOTH:
        tsb_trigger = TSB_IRQ_TYPE_EDGE_BOTH;
        break;
    case IRQ_TYPE_LEVEL_HIGH:
        tsb_trigger = TSB_IRQ_TYPE_LEVEL_HIGH;
        break;
    case IRQ_TYPE_LEVEL_LOW:
        tsb_trigger = TSB_IRQ_TYPE_LEVEL_LOW;
        break;
    default:
        return -EINVAL;
    }

    /* update trigger type in IRQ vectors and reset debounce state */
    tsb_gpio_irq_vectors[which].irq_trigger_type = trigger;
    tsb_gpio_irq_vectors[which].debounce.db_state = DB_ST_INVALID;

    putreg32(v | (tsb_trigger << shift), reg);
    return 0;
}

static int tsb_gpio_irq_debounce_handler(int irq, void *context)
{
    /* preserve ISR context */
    tsb_gpio_irq_vectors[irq].debounce.context = context;

    bool value = !!tsb_gpio_get_value(NULL, irq);

    /* check if GPIO value is stable */
    if (debounce_gpio(&tsb_gpio_irq_vectors[irq].debounce, value)) {

        int trigger_type = tsb_gpio_irq_vectors[irq].irq_trigger_type;
        enum debounce_state state = tsb_gpio_irq_vectors[irq].debounce.db_state;

        switch (trigger_type) {
            case IRQ_TYPE_EDGE_RISING:
                if (state == DB_ST_ACTIVE_STABLE) {
                    goto call_irq;
                }
                break;
            case IRQ_TYPE_EDGE_FALLING:
                if (state == DB_ST_INACTIVE_STABLE) {
                    goto call_irq;
                }
                break;
            case IRQ_TYPE_EDGE_BOTH:
                    /*
                     * no need to check state because it must be ACTIVE_STABLE
                     * or INACTIVE_STABLE for debounce_gpio() to return true
                     */
                    goto call_irq;
                break;
            case IRQ_TYPE_LEVEL_HIGH:
                if (state == DB_ST_ACTIVE_STABLE) {
                    goto call_irq;
                }
                break;
            case IRQ_TYPE_LEVEL_LOW:
                if (state == DB_ST_INACTIVE_STABLE) {
                    goto call_irq;
                }
                break;
            case IRQ_TYPE_NONE:
            default:
                break;
        }
    }

    return 0;

call_irq:
    /* reset debounce state and call attached irq handler */
    tsb_gpio_irq_vectors[irq].debounce.db_state = DB_ST_INVALID;
    tsb_gpio_irq_vectors[irq].irq_vector(irq, context);
    return 0;
}

static int tsb_gpio_irq_handler(int irq, void *context)
{
    /*
     * Handle each pending GPIO interrupt.  "The GPIO MIS register is the masked
     * interrupt status register. Bits read High in GPIO MIS reflect the status
     * of input lines triggering an interrupt. Bits read as Low indicate that
     * either no interrupt has been generated, or the interrupt is masked."
     */
    uint32_t irqstat;
    uint8_t base;
    int pin;
    size_t nr_gpio = tsb_nr_gpio();

    pm_activity(TSB_GPIO_ACTIVITY);

    /*
     * Clear all GPIO interrupts that we are going to process. "The GPIO_RAWINTSTAT
     * register is the interrupt clear register. Writing a 1 to a bit in this
     * register clears the corresponding interrupt edge detection logic register.
     * Writing a 0 has no effect."
     */
    irqstat = tsb_gpio_get_interrupt();
    putreg32(irqstat, GPIO_RAWINTSTAT);

    /* Now process each IRQ pending in the GPIO */
    for (pin = 0; pin < nr_gpio && irqstat != 0; pin++, irqstat >>= 1) {
        if ((irqstat & 1) != 0) {

            base = tsb_gpio_irq_vectors[pin].irq_gpio_base;

            if (tsb_gpio_irq_vectors[pin].debounce.ms == 0) {
                /* no debouncing, call attached irq handler */
                tsb_gpio_irq_vectors[pin].irq_vector(base + pin, context);
            } else {
                /* else, call debounce handler */
                tsb_gpio_irq_debounce_handler(base + pin, context);
            }
        }
    }

    return 0;
}

static int tsb_gpio_irqattach(void *driver_data, uint8_t irq, xcpt_t isr, uint8_t base)
{
    irqstate_t flags;

    if (irq >= tsb_nr_gpio())
        return -EINVAL;

    flags = irqsave();

    /*
     * If the new ISR is NULL, then the ISR is being detached.
     * In this case, disable the ISR and direct any interrupts
     * to the unexpected interrupt handler.
     */
    if (isr == NULL) {
        isr = irq_unexpected_isr;
    }

    /* Save the new ISR in the table. */
    tsb_gpio_irq_vectors[irq].irq_vector = isr;
    tsb_gpio_irq_vectors[irq].irq_gpio_base = base;
    irqrestore(flags);

    return OK;
}

static void tsb_gpio_irqinitialize(void)
{
    int i;

    for (i = 0; i < tsb_nr_gpio(); i++) {
        /* Point all interrupt vectors to the unexpected interrupt */
        tsb_gpio_irq_vectors[i].irq_vector = irq_unexpected_isr;

        /* Initialize all debounce info */
        tsb_gpio_irq_vectors[i].debounce.gpio = i;
        tsb_gpio_irq_vectors[i].debounce.ms = 0; // 0 = debouncing disabled
        tsb_gpio_irq_vectors[i].debounce.isr = tsb_gpio_irq_debounce_handler;
        tsb_gpio_irq_vectors[i].debounce.db_state = DB_ST_INVALID;
        tsb_gpio_irq_vectors[i].irq_trigger_type = IRQ_TYPE_NONE;
    }
}

#ifdef CONFIG_PM
static void tsb_gpio_pm_notify(struct pm_callback_s *cb,
                               enum pm_state_e pmstate)
{
    irqstate_t flags;

    flags = irqsave();

    switch (pmstate) {
    case PM_NORMAL:
        tsb_clk_enable(TSB_CLK_GPIO);
        break;
    case PM_IDLE:
    case PM_STANDBY:
        /* Nothing to do in idle or standby. */
        break;
    case PM_SLEEP:
        tsb_clk_disable(TSB_CLK_GPIO);
        break;
    default:
        /* Can never happen. */
        PANIC();
    }

    irqrestore(flags);
}
#else
static void tsb_gpio_pm_notify(struct pm_callback_s *cb,
                               enum pm_state_e pmstate)
{

}
#endif

static int tsb_gpio_pm_prepare(struct pm_callback_s *cb,
                               enum pm_state_e pmstate)
{
    return OK;
}

static void tsb_gpio_initialize(void)
{
    irqstate_t flags;

    flags = irqsave();
    if (refcount++)
        goto out;

    tsb_clk_enable(TSB_CLK_GPIO);
    tsb_reset(TSB_RST_GPIO);

    tsb_gpio_irqinitialize();

    /* Attach Interrupt Handler */
    irq_attach(TSB_IRQ_GPIO, tsb_gpio_irq_handler);

    /* Enable Interrupt Handler */
    up_enable_irq(TSB_IRQ_GPIO);
out:
    irqrestore(flags);
}

static void tsb_gpio_uninitialize(void)
{
    irqstate_t flags;

    flags = irqsave();
    if (!refcount)
        goto out;

    if (--refcount)
        goto out;

    tsb_clk_disable(TSB_CLK_GPIO);

    /* Detach Interrupt Handler */
    irq_detach(TSB_IRQ_GPIO);
out:
    irqrestore(flags);
}

static int tsb_gpio_activate(void *driver_data, uint8_t which)
{
    struct gpio_pinsharing_conf *conf;
    uint32_t mask = 0;
    uint8_t i;
    int retval = 0;

    if (gpio_state[which] == TSB_GPIO_FLAG_OPEN)
        return -EINVAL;

    tsb_gpio_initialize();

    conf = &tsb_gpio_pinsharing[which];

    /* build the mask of requested bits */
    for (i = 0; i < conf->count; i++) {
        uint8_t bit = conf->pin[i].num;
        mask |= BIT(bit);
        pinshare_bit_count[bit]++;
    }

    /* compare it to what we already have */
    mask = mask ^ (currently_owned & mask);

    /* request the missing bits */
    if (mask) {
        retval = tsb_request_pinshare(mask);
        if (retval)
            goto err;

        /* we own these bits now */
        currently_owned |= mask;

        /* clr or set them accordingly */
        for (i = 0; i < conf->count; i++) {
            uint8_t bit = conf->pin[i].num;
            if (conf->pin[i].sts == PIN_CLR_ST)
                tsb_clr_pinshare(BIT(bit));
            else
                tsb_set_pinshare(BIT(bit));
        }
    }

    gpio_state[which] = TSB_GPIO_FLAG_OPEN;

    return 0;

err:
    for (i = 0; i < conf->count; i++)
        pinshare_bit_count[conf->pin[i].num]--;
    return retval;
}

static int tsb_gpio_deactivate(void *driver_data, uint8_t which)
{
    struct gpio_pinsharing_conf *conf;
    uint32_t mask = 0;
    uint8_t i;

    if (gpio_state[which] != TSB_GPIO_FLAG_OPEN)
        return -EINVAL;

    tsb_gpio_uninitialize();

    conf = &tsb_gpio_pinsharing[which];

    /* build the mask of bits we can release */
    for (i = 0; i < conf->count; i++) {
        uint8_t bit = conf->pin[i].num;
        if (--pinshare_bit_count[bit] == 0)
            mask |= BIT(bit);
    }

    /* release these bits and stop owning them */
    if (mask) {
        tsb_release_pinshare(mask);
        currently_owned &= ~mask;
    }

    gpio_state[which] = 0;

    return 0;
}

static struct gpio_ops_s tsb_gpio_ops = {
    .get_direction = tsb_gpio_get_direction,
    .direction_in = tsb_gpio_direction_in,
    .direction_out = tsb_gpio_direction_out,
    .activate = tsb_gpio_activate,
    .get_value = tsb_gpio_get_value,
    .set_value = tsb_gpio_set_value,
    .deactivate = tsb_gpio_deactivate,
    .line_count = tsb_gpio_line_count,
    .irqattach = tsb_gpio_irqattach,
    .set_triggering = tsb_set_gpio_triggering,
    .mask_irq = tsb_gpio_mask_irq,
    .unmask_irq = tsb_gpio_unmask_irq,
    .clear_interrupt = tsb_gpio_clear_interrupt,
    .set_pull = tsb_gpio_set_pull,
    .get_pull = tsb_gpio_get_pull,
};

int tsb_gpio_register(void *driver_data)
{
    int retval;

    if (tsb_get_rev_id() == tsb_rev_es2) {
        /* GPIO6 has a different pinshare on both es2 */
        if (tsb_get_product_id() == tsb_pid_gpbridge) {
            tsb_gpio_pinsharing[6].count = 1;
        } else if (tsb_get_product_id() == tsb_pid_apbridge) {
            tsb_gpio_pinsharing[6].count = 0;
        }
    }

    if (tsb_get_product_id() == tsb_pid_apbridge) {
        tsb_gpio_pinsharing = &tsb_apb_gpio_pinsharing[0];
    } else if (tsb_get_product_id() == tsb_pid_gpbridge) {
        tsb_gpio_pinsharing = &tsb_gpb_gpio_pinsharing[0];
    }

    tsb_gpio_irq_vectors =
        calloc(sizeof(struct tsb_gpio_irq_vectors_s), tsb_nr_gpio());
    if (!tsb_gpio_irq_vectors)
        return -ENOMEM;

    retval = register_gpio_chip(&tsb_gpio_ops, TSB_GPIO_CHIP_BASE, driver_data);
    if (retval)
        goto err_register_gpio_chip;

    retval = tsb_pm_register(tsb_gpio_pm_prepare, tsb_gpio_pm_notify, NULL);
    if (retval < 0) {
        goto err_register_gpio_chip;
    }

    return 0;

err_register_gpio_chip:
    free(tsb_gpio_irq_vectors);
    tsb_gpio_irq_vectors = NULL;

    return retval;
}
