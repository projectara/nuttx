/**
 * Copyright (c) 2015 Google Inc.
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
 * @author Kim Mui
 * @brief TSB I2S device driver's data movement module.
 */
/*
 * Clocks:
 *  MCLK    - Master Clock: used to drive the other clocks.
 *  BCLK    - Bit Clock: used for clocking each bit in/out.
 *            Also referred to as SCLK (Serial Clock).
 *  WCLK    - Word Clock: determines which channel audio data is for.
 *            Also referred to as LRCLK (Left-right Clock).
 */

#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_pll.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>

#include <arch/byteorder.h>

#include "up_arch.h"
#include "tsb_scm.h"

#include "tsb_i2s.h"

static int tsb_i2s_drain_fifo(struct tsb_i2s_info *info,
                              enum device_i2s_event *event)
{
    unsigned int i;
    uint32_t intstat;
    uint32_t *dp;
    int ret = 0;

    dp = ring_buf_get_tail(info->rx_rb);

    for (i = 0; i < ring_buf_space(info->rx_rb); i += sizeof(*dp)) {
        intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INTSTAT);

        if (intstat & TSB_I2S_REG_INT_ERROR_MASK) {
            *event = tsb_i2s_intstat2event(intstat);
            ret = -EIO;
            break;
        }

        if (!(intstat & TSB_I2S_REG_INT_INT))
            break;

        *dp++ = tsb_i2s_read_raw(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_LMEM00);

        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI, intstat);
    }

    ring_buf_put(info->rx_rb, i);

    return ret;
}

static int tsb_i2s_fill_fifo(struct tsb_i2s_info *info,
                             enum device_i2s_event *event)
{
    unsigned int i;
    uint32_t intstat;
    uint32_t *dp;
    int ret = 0;

    dp = (uint32_t *)ring_buf_get_head(info->tx_rb);

    for (i = 0; i < ring_buf_len(info->tx_rb); i += sizeof(*dp)) {
        intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INTSTAT);

        if (intstat & TSB_I2S_REG_INT_ERROR_MASK) {
            *event = tsb_i2s_intstat2event(intstat);
            ret = -EIO;
            break;
        }

        if (!(intstat & TSB_I2S_REG_INT_INT))
            break;

        tsb_i2s_write_raw(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_LMEM00, *dp++);

        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO, intstat);
    }

    ring_buf_pull(info->tx_rb, i);

    return ret;
}

int tsb_i2s_rx_data(struct tsb_i2s_info *info)
{
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int ret = 0;

    while (ring_buf_is_producers(info->rx_rb)) {
        if (ring_buf_space(info->rx_rb) % 4) {
            event = DEVICE_I2S_EVENT_DATA_LEN;
            ret = -EINVAL;
            break;
        }

        ret = tsb_i2s_drain_fifo(info, &event);
        if (ret)
            break;

        if (!ring_buf_is_full(info->rx_rb)) {
            /*
             * The FIFO must be empty so unmask the irq and exit.  When there
             * is data in the FIFO, the irq handler will call this routine and
             * the FIFO will be drained again.
             */
            tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INT_INT);
            return 0;
        }

        ring_buf_pass(info->rx_rb);

        if (info->rx_callback)
            info->rx_callback(info->rx_rb, DEVICE_I2S_EVENT_RX_COMPLETE,
                              info->rx_arg);

        info->rx_rb = ring_buf_get_next(info->rx_rb);
    }

    if (ret) {
        tsb_i2s_stop_receiver(info, 1);

        if (info->rx_callback)
            info->rx_callback(info->rx_rb, event, info->rx_arg);
    }

    /*
     * No room in the ring buffer so mask irq to prevent irq flood.
     * This routine will be called again when there is room in the ring buffer.
     */
    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INT_INT);

    return ret;
}

int tsb_i2s_tx_data(struct tsb_i2s_info *info)
{
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int ret = 0;

    while (ring_buf_is_consumers(info->tx_rb)) {
        if (ring_buf_len(info->tx_rb) % 4) {
            event = DEVICE_I2S_EVENT_DATA_LEN;
            ret = -EINVAL;
            break;
        }

        ret = tsb_i2s_fill_fifo(info, &event);
        if (ret)
            break;

        if (!ring_buf_is_empty(info->tx_rb)) {
            /*
             * The FIFO must be full so unmask the irq and exit.  When there
             * is room in the FIFO, the irq handler will call this routine and
             * the FIFO will be filled again.
             */
            tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INT_INT);
            return 0;
        }

        ring_buf_reset(info->tx_rb);
        ring_buf_pass(info->tx_rb);

        if (info->tx_callback)
            info->tx_callback(info->tx_rb, DEVICE_I2S_EVENT_TX_COMPLETE,
                              info->tx_arg);

        info->tx_rb = ring_buf_get_next(info->tx_rb);
    }

    if (ret) {
        tsb_i2s_stop_transmitter(info, 1);

        if (info->tx_callback)
            info->tx_callback(info->tx_rb, event, info->tx_arg);
    }

    /*
     * No more data to send so mask the irq to prevent irq flood.
     * This routine will be called again when there is more data.
     */
    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INT_INT);

    return ret;
}

static int tsb_i2s_irq_so_handler(int irq, void *context)
{
    struct tsb_i2s_info *info = device_get_private(saved_dev);
    uint32_t intstat;

    intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INTSTAT);

    if (intstat & TSB_I2S_REG_INT_INT)
        tsb_i2s_tx_data(info);
    else
        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO, intstat);

    return OK;
}

static int tsb_i2s_irq_si_handler(int irq, void *context)
{
    struct tsb_i2s_info *info = device_get_private(saved_dev);
    uint32_t intstat;

    intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INTSTAT);

    if (intstat & TSB_I2S_REG_INT_INT)
        tsb_i2s_rx_data(info);
    else
        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI, intstat);

    return OK;
}

int tsb_i2s_xfer_irq_attach(struct tsb_i2s_info *info)
{
    int retval = OK;

    retval = irq_attach(info->so_irq, tsb_i2s_irq_so_handler);
    if (retval != OK) {
        lldbg("Failed to attach I2SO irq.\n");
        return retval;
    }

    retval = irq_attach(info->si_irq, tsb_i2s_irq_si_handler);
    if (retval != OK) {
        lldbg("Failed to attach I2SO irq.\n");
        return retval;
    }

    return retval;
}

int tsb_i2s_xfer_irq_detach(struct tsb_i2s_info *info)
{
    int retval = OK;

    retval = irq_detach(info->so_irq);
    if (retval != OK) {
        irq_detach(info->si_irq);
        return retval;
    }

    return irq_detach(info->si_irq);
}
