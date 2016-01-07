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
#include <arch/tsb/gpio.h>

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

/* A table of handlers for each GPIO interrupt */
static xcpt_t *tsb_gpio_irq_vector;
static uint8_t *tsb_gpio_irq_gpio_base;
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
    return !!(getreg32(GPIO_DATA) & (1 << which));
}

static void tsb_gpio_set_value(void *driver_data, uint8_t which, uint8_t value)
{
    putreg32(1 << which, value ? GPIO_ODATASET : GPIO_ODATACLR);
}

static int tsb_gpio_get_direction(void *driver_data, uint8_t which)
{
    uint32_t dir = getreg32(GPIO_DIR);
    return !(dir & (1 << which));
}

static void tsb_gpio_direction_in(void *driver_data, uint8_t which)
{
    putreg32(1 << which, GPIO_DIRIN);
}

static void tsb_gpio_direction_out(void *driver_data, uint8_t which, uint8_t value)
{
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

    putreg32(v | (tsb_trigger << shift), reg);
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
            base = tsb_gpio_irq_gpio_base[pin];
            tsb_gpio_irq_vector[pin](base + pin, context);
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
    tsb_gpio_irq_vector[irq] = isr;
    tsb_gpio_irq_gpio_base[irq] = base;
    irqrestore(flags);

    return OK;
}

static void tsb_gpio_irqinitialize(void)
{
    int i;

    /* Point all interrupt vectors to the unexpected interrupt */
    for (i = 0; i < tsb_nr_gpio(); i++) {
        tsb_gpio_irq_vector[i] = irq_unexpected_isr;
    }
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

static void tsb_gpio_activate(void *driver_data, uint8_t which)
{
    struct gpio_pinsharing_conf *conf;
    uint32_t mask = 0;
    uint8_t i;

    if (gpio_state[which] == TSB_GPIO_FLAG_OPEN)
        return;

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
        if (tsb_request_pinshare(mask))
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

    return;

err:
    for (i = 0; i < conf->count; i++)
        pinshare_bit_count[conf->pin[i].num]--;
}

static void tsb_gpio_deactivate(void *driver_data, uint8_t which)
{
    struct gpio_pinsharing_conf *conf;
    uint32_t mask = 0;
    uint8_t i;

    if (gpio_state[which] != TSB_GPIO_FLAG_OPEN)
        return;

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

    tsb_gpio_irq_vector = calloc(sizeof(*tsb_gpio_irq_vector), tsb_nr_gpio());
    if (!tsb_gpio_irq_vector)
        return -ENOMEM;

    tsb_gpio_irq_gpio_base =
        calloc(sizeof(*tsb_gpio_irq_gpio_base), tsb_nr_gpio());
    if (!tsb_gpio_irq_gpio_base) {
        retval = -ENOMEM;
        goto err_irq_gpio_base_alloc;
    }

    retval = register_gpio_chip(&tsb_gpio_ops, TSB_GPIO_CHIP_BASE, driver_data);
    if (retval)
        goto err_register_gpio_chip;

    return 0;

err_register_gpio_chip:
    free(tsb_gpio_irq_gpio_base);
    tsb_gpio_irq_gpio_base = NULL;
err_irq_gpio_base_alloc:
    free(tsb_gpio_irq_vector);
    tsb_gpio_irq_vector = NULL;

    return retval;
}
