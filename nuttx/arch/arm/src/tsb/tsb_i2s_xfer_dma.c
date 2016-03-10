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
 */

#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_pll.h>
#include <nuttx/device_i2s.h>
#include <nuttx/device_dma.h>
#include <nuttx/ring_buf.h>
#include <arch/byteorder.h>

#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_i2s.h"

#ifdef CONFIG_ARCH_I2S_USE_DMA_DEBUG
#define DBG_I2S_DMA(fmt, ...) lldbg(fmt, ##__VA_ARGS__)
#else
#define DBG_I2S_DMA(fmt, ...) ((void)0)
#endif

static struct {
    struct device *dev;
    void   *tx_chan;
    void   *rx_chan;
    sem_t  dma_chan_lock;
    bool   tx_dma_running;
    bool   rx_dma_running;
} i2s_dma;

static int tsb_i2s_rx_enqueue_rb(struct tsb_i2s_info *info,
                                 enum device_i2s_event *event);
static int tsb_i2s_tx_enqueue_rb(struct tsb_i2s_info *info,
                                 enum device_i2s_event *event);

static int i2s_dma_rx_callback(struct device *dev, void *chan,
                               struct device_dma_op *op,
                               unsigned int callback_event, void *arg)
{
    struct tsb_i2s_info *info = arg;
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int retval = 0;

    if (callback_event & DEVICE_DMA_CALLBACK_EVENT_COMPLETE) {
        struct ring_buf *rx_rb = info->rx_rb;

        info->rx_rb = ring_buf_get_next(info->rx_rb);

        i2s_dma.rx_dma_running = false;

        if (ring_buf_is_producers(info->rx_rb)) {
            ring_buf_reset(info->rx_rb);
            if (ring_buf_space(info->rx_rb) % 4) {
                event = DEVICE_I2S_EVENT_DATA_LEN;
                DBG_I2S_DMA("Error %x\n", ring_buf_len(info->rx_rb));
            } else {
                retval = tsb_i2s_rx_enqueue_rb(info, &event);
                if (retval) {
                    DBG_I2S_DMA("Failed to enqueue I2S buffer.\n");
                }
            }
        }

        ring_buf_put(rx_rb, op->sg[0].len);
        ring_buf_pass(rx_rb);

        if (info->rx_callback)
            info->rx_callback(rx_rb, DEVICE_I2S_EVENT_RX_COMPLETE,
                              info->rx_arg);

        device_dma_op_free(i2s_dma.dev, op);
    }

    return 0;
}

static int tsb_i2s_rx_enqueue_rb(struct tsb_i2s_info *info,
                                 enum device_i2s_event *event)
{
    int retval = 0;
    struct device_dma_op *dma_op = NULL;
    uint32_t base;
    uint32_t *dp;

    dp = (uint32_t *)ring_buf_get_head(info->rx_rb);

    retval = device_dma_op_alloc(i2s_dma.dev, 1, 0, &dma_op);
    if (retval) {
        DBG_I2S_DMA("tsb_i2s_rx_enqueue_rb: failed allocate a DMA op, retval \
                    = %d.\n", retval);
        return retval;
    }

    dma_op->callback = i2s_dma_rx_callback;
    dma_op->callback_arg = info;
    dma_op->callback_events = DEVICE_DMA_CALLBACK_EVENT_COMPLETE;
    dma_op->sg_count = 1;

    base = tsb_i2s_get_block_base(info, TSB_I2S_BLOCK_SI);
    base += TSB_I2S_REG_LMEM00;
    dma_op->sg[0].src_addr = (off_t) base;
    dma_op->sg[0].dst_addr = (off_t) dp;
    dma_op->sg[0].len = (size_t)ring_buf_space(info->rx_rb);

    retval = device_dma_enqueue(i2s_dma.dev, i2s_dma.rx_chan, dma_op);
    if (retval) {
        DBG_I2S_DMA("failed to start DMA transfer: %d\n", retval);
    } else
        i2s_dma.rx_dma_running = true;

    return retval;
}

int tsb_i2s_rx_data(struct tsb_i2s_info *info)
{
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int retval = 0;

    if (ring_buf_is_producers(info->rx_rb)) {
        ring_buf_reset(info->rx_rb);
        if (ring_buf_space(info->rx_rb) % 4) {
            event = DEVICE_I2S_EVENT_DATA_LEN;
            retval = -EINVAL;
        } else {
            if (!i2s_dma.rx_dma_running) {
                retval = tsb_i2s_rx_enqueue_rb(info, &event);
                if (retval) {
                    DBG_I2S_DMA("Failed to enqueue I2S buffer.\n");
                }
            }
        }
    }

    if (retval) {
        tsb_i2s_stop_receiver(info, 1);

        if (info->rx_callback)
            info->rx_callback(info->rx_rb, event, info->rx_arg);
    }

    /*
     * No room in the ring buffer so mask irq to prevent irq flood.
     * This routine will be called again when there is room in the ring buffer.
     */
    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INT_INT);

    return retval;
}

static int i2s_dma_tx_callback(struct device *dev, void *chan,
                               struct device_dma_op *op,
                               unsigned int callback_event, void *arg)
{
    struct tsb_i2s_info *info = arg;
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int retval = 0;

    if (callback_event & DEVICE_DMA_CALLBACK_EVENT_COMPLETE) {
        ring_buf_reset(info->tx_rb);
        ring_buf_pass(info->tx_rb);
        if (info->tx_callback)
            info->tx_callback(info->tx_rb, DEVICE_I2S_EVENT_TX_COMPLETE,
                              info->tx_arg);

        i2s_dma.tx_dma_running = false;

        info->tx_rb = ring_buf_get_next(info->tx_rb);

        if (tsb_i2s_tx_is_active(info)) {
            if (ring_buf_is_consumers(info->tx_rb)) {
                if (ring_buf_len(info->tx_rb) % 4) {
                    event = DEVICE_I2S_EVENT_DATA_LEN;
                    DBG_I2S_DMA("Error: %x\n", ring_buf_len(info->tx_rb));
                } else {
                    retval = tsb_i2s_tx_enqueue_rb(info, &event);
                    if (retval) {
                        DBG_I2S_DMA("Failed to enqueue I2S buffer.\n");
                    }
                }
            }
        }
        device_dma_op_free(i2s_dma.dev, op);
    }

    return retval;
}

static int tsb_i2s_tx_enqueue_rb(struct tsb_i2s_info *info,
                                 enum device_i2s_event *event)
{
    int retval = 0;
    struct device_dma_op *dma_op = NULL;
    uint32_t base;
    uint32_t *dp;

    dp = (uint32_t *)ring_buf_get_head(info->tx_rb);

    retval = device_dma_op_alloc(i2s_dma.dev, 1, 0, &dma_op);
    if (retval != OK) {
        DBG_I2S_DMA("tsb_i2s_tx_enqueue_rb: failed allocate a DMA op, retval \
                    = %d.\n", retval);
        return retval;
    }

    dma_op->callback = i2s_dma_tx_callback;
    dma_op->callback_arg = info;
    dma_op->callback_events = DEVICE_DMA_CALLBACK_EVENT_COMPLETE;
    dma_op->sg_count = 1;

    base = tsb_i2s_get_block_base(info, TSB_I2S_BLOCK_SO);
    base += TSB_I2S_REG_LMEM00;

    dma_op->sg[0].src_addr = (off_t) dp;
    dma_op->sg[0].dst_addr = (off_t) base;
    dma_op->sg[0].len = ring_buf_len(info->tx_rb);

    retval = device_dma_enqueue(i2s_dma.dev, i2s_dma.tx_chan, dma_op);
    if (retval) {
        DBG_I2S_DMA("failed to start DMA transfer: %d\n", retval);
    } else
        i2s_dma.tx_dma_running = true;

    return retval;
}

int tsb_i2s_tx_data(struct tsb_i2s_info *info)
{
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int retval = 0;

    if (ring_buf_is_consumers(info->tx_rb)) {
        if (ring_buf_len(info->tx_rb) % 4) {
            event = DEVICE_I2S_EVENT_DATA_LEN;
            retval = -EINVAL;
        } else {
            if (!i2s_dma.tx_dma_running) {
                retval = tsb_i2s_tx_enqueue_rb(info, &event);
                if (retval) {
                    DBG_I2S_DMA("Failed to enqueue I2S buffer.\n");
                }
            }
        }
    }

    if (retval) {
        tsb_i2s_stop_transmitter(info, 1);

        if (info->tx_callback)
            info->tx_callback(info->tx_rb, event, info->tx_arg);
    }

    /*
     * No more data to send so mask the irq to prevent irq flood.
     * This routine will be called again when there is more data.
     */
    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INT_INT);

    return retval;
}

int tsb_i2s_start_receiver(struct tsb_i2s_info *info)
{
    irqstate_t flags;
    int retval;

    flags = irqsave();

    retval = tsb_i2s_rx_data(info);
    if (retval)
        goto err_irqrestore;

    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI,
                       TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                       TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_INT);
    tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SI,
                        TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                        TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);

    retval = tsb_i2s_start(info, TSB_I2S_BLOCK_SI);
    if (retval)
        goto err_mask_irqs;

    info->flags |= TSB_I2S_FLAG_RX_ACTIVE;

    irqrestore(flags);

    return 0;

err_mask_irqs:
    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SI,
                      TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                      TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);
err_irqrestore:
    irqrestore(flags);

    return retval;
}

void tsb_i2s_stop_receiver(struct tsb_i2s_info *info, int is_err)
{
    irqstate_t flags;

    flags = irqsave();

    tsb_i2s_stop(info, TSB_I2S_BLOCK_SI, is_err);

    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SI,
                      TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                      TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);
    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI,
                       TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                       TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);

    info->flags &= ~TSB_I2S_FLAG_RX_ACTIVE;

    irqrestore(flags);
}

int tsb_i2s_start_transmitter(struct tsb_i2s_info *info)
{
    irqstate_t flags;
    int retval;

    flags = irqsave();

    if (!tsb_i2s_tx_is_active(info)) {
        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO,
                           TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                           TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_INT);
        /* TSB_I2S_REG_INT_INT is unmasked in tsb_i2s_tx_data() */
        tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SO,
                            TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                            TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);

        retval = tsb_i2s_start(info, TSB_I2S_BLOCK_SO);
        if (retval)
            goto err_irqrestore;

        info->flags |= TSB_I2S_FLAG_TX_ACTIVE;
    }

    retval = tsb_i2s_tx_data(info);

err_irqrestore:
    irqrestore(flags);

    return retval;
}

void tsb_i2s_stop_transmitter(struct tsb_i2s_info *info, int is_err)
{
    irqstate_t flags;

    flags = irqsave();

    tsb_i2s_stop(info, TSB_I2S_BLOCK_SO, is_err);

    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SO,
                      TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                      TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);
    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO,
                       TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                       TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_DMACMSK);

    info->flags &= ~TSB_I2S_FLAG_TX_ACTIVE;

    irqrestore(flags);
}

int tsb_i2s_xfer_open(struct tsb_i2s_info *info)
{
    int retval = 0;

#if defined(CONFIG_ARCH_UNIPROTX_I2S_SHARE_DMA)
    i2s_dma.dev = tsb_dma_share_open();
#else
    i2s_dma.dev = device_open(DEVICE_TYPE_DMA_HW, 0);
#endif
    if (!i2s_dma.dev) {
        retval = -ENODEV;
        DBG_I2S_DMA("DMA open fail\n");
    } else
        DBG_I2S_DMA("I2S dma enable\n");
    return retval;
}

void tsb_i2s_xfer_close(struct tsb_i2s_info *info)
{
    if (i2s_dma.dev) {
        device_close(i2s_dma.dev);
    }
}

int tsb_i2s_xfer_prepare_receiver(struct tsb_i2s_info *info)
{
    struct device_dma_params chan_params = {
            .src_dev = DEVICE_DMA_DEV_IO,
            .src_devid = 1,
            .src_inc_options = DEVICE_DMA_INC_NOAUTO,
            .dst_dev = DEVICE_DMA_DEV_MEM,
            .dst_devid = 0,
            .dst_inc_options = DEVICE_DMA_INC_AUTO,
            .transfer_size = DEVICE_DMA_TRANSFER_SIZE_32,
            .burst_len = DEVICE_DMA_BURST_LEN_1,
            .swap = DEVICE_DMA_SWAP_SIZE_NONE,
    };

    device_dma_chan_alloc(i2s_dma.dev, &chan_params,
            &i2s_dma.rx_chan);

    if (!i2s_dma.rx_chan) {
        DBG_I2S_DMA("i2s: couldn't allocate rx channel\n");
        return -ENOMEM;
    }

    return 0;
}

int tsb_i2s_xfer_prepare_transmitter(struct tsb_i2s_info *info)
{
    struct device_dma_params chan_params = {
            .src_dev = DEVICE_DMA_DEV_MEM,
            .src_devid = 0,
            .src_inc_options = DEVICE_DMA_INC_AUTO,
            .dst_dev = DEVICE_DMA_DEV_IO,
            .dst_devid = 0,
            .dst_inc_options = DEVICE_DMA_INC_NOAUTO,
            .transfer_size = DEVICE_DMA_TRANSFER_SIZE_32,
            .burst_len = DEVICE_DMA_BURST_LEN_1,
            .swap = DEVICE_DMA_SWAP_SIZE_NONE,
    };

    device_dma_chan_alloc(i2s_dma.dev, &chan_params,
            &i2s_dma.tx_chan);

    if (!i2s_dma.tx_chan) {
        DBG_I2S_DMA("i2s: couldn't allocate tx channel\n");
        return -ENOMEM;
    }

    return 0;
}
