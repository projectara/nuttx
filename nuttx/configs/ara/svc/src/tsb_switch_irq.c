/*
 * Copyright (c) 2015, 2016 Google Inc.
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

#include "tsb_switch.h"

#define DBG_COMP ARADBG_SWITCH
#include <stddef.h>
#include <nuttx/unipro/unipro.h>
#include "stm32.h"
#include <sys/wait.h>
#include <ara_debug.h>

#include <nuttx/power/pm.h>

#include "svc_pm.h"

#define SWITCH_IRQ_MAX    16

/* TSB attributes fields values */
#define TSB_INTERRUPTENABLE_ALL     (0xFFFF)
#define TSB_L4_INTERRUPTENABLE_ALL  (0x2800)
#define TSB_INTERRUPT_SPI3ES        (1 << 11)
#define TSB_INTERRUPT_SPI4ES        (1 << 12)
#define TSB_INTERRUPT_SPI5ES        (1 << 13)
#define TSB_INTERRUPT_SPICES        (1 << 15)
#define TSB_INTERRUPT_SPIPORT4_RX   (1 << 20)
#define TSB_INTERRUPT_SPIPORT5_RX   (1 << 21)
#define TSB_INTERRUPT_SWINTERNAL    (1 << 31)

/* Switch internal attributes values */
#define SWINE_ENABLE_ALL            (0x7FFFB805)
#define SPIINTE_ENABLE_ALL          (0x3)

/*
 * Map from port IRQ status bits to the UniPro attributes needed to
 * clear the IRQ source.
 */
#define IRQ_STATUS_ENDPOINTRESETIND     (0x0)
#define IRQ_STATUS_LINKSTARTUPIND       (0x1)
#define IRQ_STATUS_LINKLOSTIND          (0x2)
#define IRQ_STATUS_HIBERNATEENTERIND    (0x3)
#define IRQ_STATUS_HIBERNATEEXITIND     (0x4)
#define IRQ_STATUS_POWERMODEIND         (0x5)
#define IRQ_STATUS_TESTMODEIND          (0x6)
#define IRQ_STATUS_ERRORPHYIND          (0x7)
#define IRQ_STATUS_ERRORPAIND           (0x8)
#define IRQ_STATUS_ERRORDIND            (0x9)
#define IRQ_STATUS_ERRORNIND            (0xa)
#define IRQ_STATUS_ERRORTIND            (0xb)
#define IRQ_STATUS_PAINITERROR          (0xc)
#define IRQ_STATUS_DEBUGCOUNTEROVERFLOW (0xd)
#define IRQ_STATUS_LINKSTARTUPCNF       (0xe)
#define IRQ_STATUS_MAILBOX              (0xf)

/* Attributes as sources of interrupts from the Unipro ports */
static uint16_t unipro_irq_attr[SWITCH_IRQ_MAX] = {
    [IRQ_STATUS_ENDPOINTRESETIND]     = TSB_DME_ENDPOINTRESETIND,
    [IRQ_STATUS_LINKSTARTUPIND]       = TSB_DME_LINKSTARTUPIND,
    [IRQ_STATUS_LINKLOSTIND]          = TSB_DME_LINKLOSTIND,
    [IRQ_STATUS_HIBERNATEENTERIND]    = TSB_DME_HIBERNATEENTERIND,
    [IRQ_STATUS_HIBERNATEEXITIND]     = TSB_DME_HIBERNATEEXITIND,

    /*
     * FIXME: hackaround until SW-1237 is implemented.
     *
     * The proper attribute is TSB_DME_POWERMODEIND, but that's
     * currently polled by switch_apply_power_mode() to determine when
     * a power mode change is done, after writing PA_PWRMODE. We would
     * race with that thread if we read it in the IRQ worker.
     */
    [IRQ_STATUS_POWERMODEIND]         = 0, /* TSB_DME_POWERMODEIND */

    [IRQ_STATUS_TESTMODEIND]          = TSB_DME_TESTMODEIND,
    [IRQ_STATUS_ERRORPHYIND]          = TSB_DME_ERRORPHYIND,
    [IRQ_STATUS_ERRORPAIND]           = TSB_DME_ERRORPAIND,
    [IRQ_STATUS_ERRORDIND]            = TSB_DME_ERRORDIND,
    [IRQ_STATUS_ERRORNIND]            = 0, /* Not recommended to read */
    [IRQ_STATUS_ERRORTIND]            = TSB_DME_ERRORTIND,
    [IRQ_STATUS_PAINITERROR]          = TSB_DME_ERRORDIND,
    [IRQ_STATUS_DEBUGCOUNTEROVERFLOW] = TSB_DEBUGCOUNTEROVERFLOW,
    [IRQ_STATUS_LINKSTARTUPCNF]       = TSB_DME_LINKSTARTUPCNF,
    [IRQ_STATUS_MAILBOX]              = TSB_MAILBOX
};

/*
 * Helper for pulling data out of a switch SPI FIFO, checking status,
 * and passing off to the UniPro core.
 */

static int irq_fifo_rx(struct tsb_switch *sw, unsigned int spi_fifo) {
    DEBUGASSERT(sw->ops->__irq_fifo_rx);
    return sw->ops->__irq_fifo_rx(sw, spi_fifo);
}

int switch_port_irq_enable(struct tsb_switch *sw,
                           uint8_t port_id,
                           bool enable) {
    if (switch_dme_set(sw, port_id, TSB_INTERRUPTENABLE, 0x0,
                       enable ? TSB_INTERRUPTENABLE_ALL : 0)) {
        dbg_error("Port %d INTERRUPTENABLE register write failed\n", port_id);
        return -EIO;
    }

    return OK;
}

/* Low level switch IRQ handler
 *
 * Posts a message in a list in order to defer the work to perform
 */
static int switch_hard_irq_handler(int irq, void *context, void *priv)
{
    struct tsb_switch *sw = priv;

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return -EINVAL;
    }

    switch_post_irq(sw);

    pm_activity(SVC_SWITCH_IRQ_ACTIVITY);

    return 0;
}

int switch_irq_enable(struct tsb_switch *sw, bool enable) {
    if (enable) {
        // Enable switch interrupt sources and install handler
        if (!sw->pdata->gpio_irq) {
            dbg_error("%s: no Switch context\n", __func__);
            return -EINVAL;
        }

        /*
         * Configure switch IRQ line: parameters from pdata, install
         * handler, and pass the tsb_switch struct to the handler.
         */
        stm32_gpiosetevent_priv(sw->pdata->gpio_irq,
                                sw->pdata->irq_rising_edge,
                                !sw->pdata->irq_rising_edge,
                                true, switch_hard_irq_handler, sw);

        // Enable the switch internal interrupt sources
        if (switch_internal_setattr(sw, SWINE, SWINE_ENABLE_ALL)) {
            dbg_error("Switch SWINE register write failed\n");
            return -EIO;
        }

        // Enable the L4 interrupts
        if (switch_dme_set(sw, SWITCH_PORT_ID, TSB_INTERRUPTENABLE, 0x0,
                           TSB_L4_INTERRUPTENABLE_ALL)) {
            dbg_error("Switch INTERRUPTENABLE register write failed\n");
            return -EIO;
        }

        // Enable the SPI interrupts
        if (switch_internal_setattr(sw, SPIINTE, SPIINTE_ENABLE_ALL)) {
            dbg_error("Switch SPIINTE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPICEE,
                                    sw->rdata->spicee_enable_all)) {
            dbg_error("Switch SPICEE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI3EE,
                                    sw->rdata->spi3ee_enable_all)) {
            dbg_error("Switch SPI3EE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI4EE,
                                    sw->rdata->spi45ee_enable_all)) {
            dbg_error("Switch SPI4EE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI5EE,
                                    sw->rdata->spi45ee_enable_all)) {
            dbg_error("Switch SPI5EE register write failed\n");
            return -EIO;
        }
    } else {
        // Disable switch interrupt
        stm32_gpiosetevent_priv(sw->pdata->gpio_irq, false, false, false,
                                NULL, NULL);
    }

    return OK;
}

int tsb_switch_event_notify(struct tsb_switch *sw,
                            struct tsb_switch_event *event) {

    struct list_head *node, *next;
    struct tsb_switch_event_listener *l;

    list_foreach_safe(&sw->listeners, node, next) {
        l = list_entry(node, struct tsb_switch_event_listener, entry);
        if (l->cb) {
            l->cb(event);
        }
    }

    return 0;
}

static void switch_port_irq_reenable(struct tsb_switch *sw, uint8_t port)
{
    uint32_t attr_value;

    /*
     * We got interrupted, but something inside the switch
     * disabled the interrupt source. This happens if e.g. a port
     * re-links up.
     *
     * Check the interrupt enable attribute, if interrupts are
     * disabled for the port, re-enable them. Since the interrupt is
     * latched in the switch it will fire again after enablement.
     */
    if (switch_dme_get(sw, port, TSB_INTERRUPTENABLE, 0x0,
                       &attr_value)) {
        dbg_error("IRQ: Port %u TSB_INTERRUPTENABLE read failed\n",
                  port);
    } else {
        if (!attr_value) {
            dbg_insane("IRQ: port %u TSB_INTERRUPTENABLE=%d\n",
                       port, attr_value);
            switch_port_irq_enable(sw, port, true);
        }
    }
}

int switch_irq_handler(struct tsb_switch *sw) {
    uint32_t swint, swins, port_irq_status, attr_value;
    int i, j, rc;

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return -EINVAL;
    }

    do {
        // Read Switch Interrupt Status register
        if (switch_internal_getattr(sw, SWINT, &swint)) {
            dbg_error("IRQ: SWINT register read failed\n");
            return -EIO;
        }
        dbg_insane("IRQ: SWINT=0x%x\n", swint);

        // Handle the Switch internal interrupts
        if (swint & TSB_INTERRUPT_SWINTERNAL) {
            if (switch_internal_getattr(sw, SWINS, &swins)) {
                dbg_error("IRQ: SWINS register read failed\n");
            }
            dbg_insane("IRQ: Switch internal irq, SWINS=0x%04x\n", swins);

            if (swins & TSB_INTERRUPT_SPICES) {
                if (switch_internal_getattr(sw, SPICES, &attr_value)) {
                    dbg_error("IRQ: SPICES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPICES=0x%04x\n",
                           attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI3ES) {
                if (switch_internal_getattr(sw, SPI3ES, &attr_value)) {
                    dbg_error("IRQ: SPI3ES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPI3ES=0x%04x\n",
                           attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI4ES) {
                if (switch_internal_getattr(sw, SPI4ES, &attr_value)) {
                    dbg_error("IRQ: SPI4ES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPI4ES=0x%04x\n",
                           attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI5ES) {
                if (switch_internal_getattr(sw, SPI5ES, &attr_value)) {
                    dbg_error("IRQ: SPI5ES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPI5ES=0x%04x\n",
                           attr_value);
            }
        }

        // Handle external interrupts: CPorts 4 & 5
        if (swint & TSB_INTERRUPT_SPIPORT4_RX) {
            dbg_insane("IRQ: Switch SPI port 4 RX irq\n");
            irq_fifo_rx(sw, 4);
        }
        if (swint & TSB_INTERRUPT_SPIPORT5_RX) {
            dbg_insane("IRQ: Switch SPI port 5 RX irq\n");
            irq_fifo_rx(sw, 5);
        }

        /*
         * Handle Unipro interrupts: read the interrupt status on
         * enabled Unipro ports.
         * SW-1527: Read the interrupt status on a disabled port returns
         * the error 0x23 (DISABLED_TARGET) and generates a new IRQ (!)
         */
        for (i = 0; i < SWITCH_PORT_MAX; i++) {
            if (sw->rdata->rflags & TSB_SWITCH_RFLAG_REENABLE_PORT_IRQ_HACK) {
                if (interface_get_power_state(interface_get_by_portid(i)) ==
                    ARA_IFACE_PWR_UP) {
                    switch_port_irq_reenable(sw, (uint8_t)i);
                }
            }

            // If Unipro interrupt pending, read the interrupt status attribute
            if (swint & (1 << i)) {
                if (switch_dme_get(sw, i, TSB_INTERRUPTSTATUS, 0x0,
                                   &port_irq_status)) {
                    dbg_error("IRQ: TSB_INTERRUPTSTATUS(%d) register read failed\n",
                              i);
                    break;
                }
                dbg_insane("IRQ: TSB_INTERRUPTSTATUS(%d)=0x%04x\n",
                           i, port_irq_status);

                // Read the attributes associated to the interrupt sources
                for (j = 0; j < SWITCH_IRQ_MAX; j++) {
                    if ((port_irq_status & (1 << j)) && unipro_irq_attr[j]) {
                        if (switch_dme_get(sw, i, unipro_irq_attr[j], 0x0,
                                           &attr_value)) {
                            dbg_error("IRQ: Port %d line %d attr(0x%04x) read failed\n",
                                      i, j, unipro_irq_attr[j]);
                        } else {
                            dbg_insane("IRQ: Port %d line %d asserted, attr(0x%04x)=0x%04x\n",
                                       i, j, unipro_irq_attr[j], attr_value);
                        }

                        uint32_t irq_type = j;
                        uint32_t port = i;
                        switch (irq_type) {
                        case IRQ_STATUS_LINKSTARTUPCNF: {
                            struct tsb_switch_event e;
                            e.type = TSB_SWITCH_EVENT_LINKUP;
                            e.linkup.port = port;
                            e.linkup.val = attr_value;
                            rc = tsb_switch_event_notify(sw, &e);
                            if (rc) {
                                dbg_error("IRQ: LinkUp event notification failed for port %d: %d\n",
                                          port, rc);
                            }
                            break;
                        }
                        case IRQ_STATUS_MAILBOX: {
                            struct tsb_switch_event e;
                            e.type = TSB_SWITCH_EVENT_MAILBOX;
                            e.mbox.port = port;
                            e.mbox.val = attr_value;
                            rc = tsb_switch_event_notify(sw, &e);
                            if (rc) {
                                dbg_error("IRQ: Mailbox event notification failed for port %d: %d\n",
                                          port, rc);
                            }
                            break;
                        }
                        default:
                            break;
                        }
                    }
                }
            }
        }

    } while (swint);

    return 0;
}

/* Post a message to the IRQ worker */
int switch_post_irq(struct tsb_switch *sw)
{
    sem_post(&sw->sw_irq_lock);

    return 0;
}

/* IRQ worker */
int _switch_irq_pending_worker(int argc, char *argv[])
{
    struct tsb_switch *sw = (struct tsb_switch *) strtol(argv[1], NULL, 16);

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return ERROR;
    }

    while (!sw->sw_irq_worker_exit) {

        sem_wait(&sw->sw_irq_lock);
        if (sw->sw_irq_worker_exit)
            break;

        /* Calls the low level handler to clear the interrupt source */
        switch_irq_handler(sw);
    }

    return 0;
}
