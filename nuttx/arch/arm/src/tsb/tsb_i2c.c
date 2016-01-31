/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/device.h>
#include <nuttx/device_i2c.h>
#include <nuttx/power/pm.h>

#include <arch/irq.h>
#include <arch/tsb/pm.h>

#include "up_arch.h"

#include "chip.h"
#include "tsb_scm.h"
#include "tsb_i2c.h"

#define TSB_I2C_FLAG_OPENED     BIT(0)

/** Set I2C speed based on kconfig value */
#if defined(CONFIG_TSB_I2C_SPEED_FAST)
#define TSB_I2C_CON_SPEED	TSB_I2C_CON_SPEED_FAST
#else
#define TSB_I2C_CON_SPEED	TSB_I2C_CON_SPEED_STD
#endif

/** Default I2C controller configuration */
#define TSB_I2C_CONFIG (TSB_I2C_CON_RESTART_EN      | \
                        TSB_I2C_CON_MASTER          | \
                        TSB_I2C_CON_SLAVE_DISABLE   | \
                        TSB_I2C_CON_SPEED)

/** Size of transmission and reception FIFOs */
#define TSB_I2C_TX_FIFO_DEPTH   8
#define TSB_I2C_RX_FIFO_DEPTH   8

/* List of IRQ events handled by this driver */
#define TSB_I2C_INTR_DEFAULT_MASK (TSB_I2C_INTR_RX_FULL     | \
                                   TSB_I2C_INTR_TX_EMPTY    | \
                                   TSB_I2C_INTR_TX_ABRT     | \
                                   TSB_I2C_INTR_STOP_DET)

/** Timeout for enabling the I2C controller */
#define TSB_I2C_ENABLE_TIMEOUT_PERIOD   25      /** periods of 25ms */
#define TSB_I2C_ENABLE_TIMEOUT_RETRIES  50      /** total of 1.250 ms */
/** Timeout for waiting that the I2C bus is ready */
#define TSB_I2C_BUS_TIMEOUT_PERIOD      1000    /** periods of 1 ms */
#define TSB_I2C_BUS_TIMEOUT_RETRIES     20      /** total of 20 ms */
/** Timeout for waiting on a transfer */
#define TSB_I2C_TRANSFER_TIMEOUT ((1000 * CLK_TCK) / 1000) /* 1000 ms (100Hz tick) */

#define TSB_I2C_ACTIVITY            9

/**
 * struct tsb_i2c_info - The driver internal information
 */
struct tsb_i2c_info {
    struct device *dev;         /**< Device driver handler */
    uint32_t flags;             /**< I2C driver flags */
    uint32_t reg_base;          /**< Base address of I2C controller registers */
    uint32_t i2c_irq;           /**< IRQ number */

    sem_t wait;                 /**< Transfer mutex */
    sem_t mutex;                /**< Exclusive resource access */
    WDOG_ID timeout;            /**< Watchdog for transfer timeout */

    struct device_i2c_request *requests;    /**< List of requests */
    uint32_t request_count;                 /**< Number of requests */

    uint32_t tx_length;         /**< Number of transmitting messages */
    uint32_t tx_index;          /**< Index for transmitting messages */
    uint8_t *tx_buffer;         /**< Buffer of transmitting messages */

    uint32_t rx_index;          /**< Index for receiving messages */
    uint32_t rx_length;         /**< Number of receiving messages */
    uint8_t *rx_buffer;         /**< Buffer of receiving messages */

    uint32_t cmd_err;           /**< Command error code */
    uint32_t msg_err;           /**< Message error code */
    uint32_t status;            /**< Status of the transfer */
    uint32_t abort_source;      /**< Source of a transmission abort */
    uint32_t rx_outstanding;    /**< Reception outstanding */
};

/** device structure for interrupt handler */
static struct device *saved_dev;

/** Read a register of the I2C controller */
static uint32_t i2c_read(uint32_t base, int offset)
{
    return getreg32(base + offset);
}

/** Write a register of the I2C controller */
static void i2c_write(uint32_t base, int offset, uint32_t b)
{
    putreg32(b, base + offset);
}

/** Clear all interruptions */
static void tsb_i2c_clear_int(struct tsb_i2c_info *info)
{
    i2c_read(info->reg_base, TSB_I2C_CLR_INTR);
}

/** Disable all interruptions */
static void tsb_i2c_disable_int(struct tsb_i2c_info *info)
{
    i2c_write(info->reg_base, TSB_I2C_INTR_MASK, 0);
}

/** Enable or disable the I2C controller */
static void i2c_set_enable(struct tsb_i2c_info *info, int enable)
{
    int i;

    for (i = 0; i < TSB_I2C_ENABLE_TIMEOUT_RETRIES; i++) {
        i2c_write(info->reg_base, TSB_I2C_ENABLE, enable);

        if ((i2c_read(info->reg_base, TSB_I2C_ENABLE_STATUS) & 0x1) == enable)
            return;

        usleep(TSB_I2C_ENABLE_TIMEOUT_PERIOD);
    }

    lldbg("I2C timeout!");
}

/** Enable the I2C controller */
static void tsb_i2c_enable(struct tsb_i2c_info *info)
{
    i2c_set_enable(info, 1);
}

/** Disable the I2C controller */
static void tsb_i2c_disable(struct tsb_i2c_info *info)
{
    i2c_set_enable(info, 0);

    tsb_i2c_disable_int(info);
    tsb_i2c_clear_int(info);
}

/**
 * Initialize the TSB I2 controller
 */
static void tsb_i2c_init(struct tsb_i2c_info *info)
{
    /* Disable the adapter */
    tsb_i2c_disable(info);

    /*
     * Set timings for Standard and Fast Speed mode:
     * Values taken from ARA_ES2_GPBridge_AppendixA and tweaked for
     * 98.4 kHz in standard mode and 396.7 kHz in full speed mode
     */
    i2c_write(info->reg_base, TSB_I2C_SS_SCL_HCNT, 205);
    i2c_write(info->reg_base, TSB_I2C_SS_SCL_LCNT, 270);
    i2c_write(info->reg_base, TSB_I2C_FS_SCL_HCNT, 45);
    i2c_write(info->reg_base, TSB_I2C_FS_SCL_LCNT, 63);

    /* Configure Tx/Rx FIFO threshold levels */
    i2c_write(info->reg_base, TSB_I2C_TX_TL, TSB_I2C_TX_FIFO_DEPTH - 1);
    i2c_write(info->reg_base, TSB_I2C_RX_TL, 0);

    /* configure the i2c master */
    i2c_write(info->reg_base, TSB_I2C_CON, TSB_I2C_CONFIG);
}

/** Wait until the bus is ready */
static int tsb_i2c_wait_bus_ready(struct tsb_i2c_info *info)
{
    int i;

    for (i = 0; i < TSB_I2C_BUS_TIMEOUT_RETRIES; i++) {
        if (!(i2c_read(info->reg_base, TSB_I2C_STATUS) &
                    TSB_I2C_STATUS_ACTIVITY))
            return 0;

        usleep(TSB_I2C_BUS_TIMEOUT_PERIOD);
    }

    lldbg("I2C timeout!");
    return -ETIMEDOUT;
}

/** Start an I2C transfer */
static void tsb_i2c_start_transfer(struct tsb_i2c_info *info)
{
    llvdbg("\n");

    /* Disable the adapter */
    tsb_i2c_disable(info);

    /* write target address */
    i2c_write(info->reg_base, TSB_I2C_TAR, info->requests[info->tx_index].addr);

    /* Disable the interrupts */
    tsb_i2c_disable_int(info);

    /* Enable the adapter */
    tsb_i2c_enable(info);

    /* Clear interrupts */
    tsb_i2c_clear_int(info);

    /* Enable interrupts */
    i2c_write(info->reg_base, TSB_I2C_INTR_MASK, TSB_I2C_INTR_DEFAULT_MASK);
}

/**
 * Internal function that handles the read or write requests
 * It is called from the IRQ handler.
 */
static void tsb_i2c_transfer_request(struct tsb_i2c_info *info)
{
    uint32_t intr_mask;
    uint32_t addr = info->requests[info->tx_index].addr;
    uint8_t *buffer = info->tx_buffer;
    uint32_t length = info->tx_length;

    bool need_restart = false;

    int tx_avail;
    int rx_avail;

    llvdbg("tx_index %d\n", info->tx_index);

    /* loop over the i2c message array */
    for (; info->tx_index < info->request_count; info->tx_index++) {

        if (info->requests[info->tx_index].addr != addr) {
            lldbg("invalid target address\n");
            info->msg_err = -EINVAL;
            break;
        }

        if (info->requests[info->tx_index].length == 0) {
            lldbg("invalid message length\n");
            info->msg_err = -EINVAL;
            break;
        }

        if (!(info->status & TSB_I2C_STATUS_WRITE_IN_PROGRESS)) {
            /* init a new request */
            buffer = info->requests[info->tx_index].buffer;
            length = info->requests[info->tx_index].length;

            /* force a restart between messages */
            if (info->tx_index > 0)
                need_restart = true;
        }

        /* Get the amount of free space in the internal buffer */
        tx_avail = TSB_I2C_TX_FIFO_DEPTH - i2c_read(info->reg_base, TSB_I2C_TXFLR);
        rx_avail = TSB_I2C_RX_FIFO_DEPTH - i2c_read(info->reg_base, TSB_I2C_RXFLR);

        /* loop until one of the fifo is full or buffer is consumed */
        while (length > 0 && tx_avail > 0 && rx_avail > 0) {
            uint32_t cmd = 0;

            if (info->tx_index == info->request_count - 1 && length == 1) {
                /* Last msg, issue a STOP */
                cmd |= (1 << 9);
                llvdbg("STOP\n");
            }

            if (need_restart) {
                cmd |= (1 << 10); /* RESTART */
                need_restart = false;
                llvdbg("RESTART\n");
            }

            if (info->requests[info->tx_index].flags & I2C_FLAG_READ) {
                if (rx_avail - info->rx_outstanding <= 0)
                    break;

                i2c_write(info->reg_base, TSB_I2C_DATA_CMD, cmd | 1 << 8); /* READ */
                llvdbg("READ\n");

                rx_avail--;
                info->rx_outstanding++;
            } else {
                i2c_write(info->reg_base, TSB_I2C_DATA_CMD, cmd | *buffer++);
                llvdbg("WRITE\n");
            }

            tx_avail--;
            length--;
        }

        info->tx_buffer = buffer;
        info->tx_length = length;

        if (length > 0) {
            info->status |= TSB_I2C_STATUS_WRITE_IN_PROGRESS;
            break;
        } else {
            info->status &= ~TSB_I2C_STATUS_WRITE_IN_PROGRESS;
        }
    }

    intr_mask = TSB_I2C_INTR_DEFAULT_MASK;

    /* No more data to write. Stop the TX IRQ */
    if (info->tx_index == info->request_count)
        intr_mask &= ~TSB_I2C_INTR_TX_EMPTY;

    /* In case of error, mask all the IRQs */
    if (info->msg_err)
        intr_mask = 0;

    i2c_write(info->reg_base, TSB_I2C_INTR_MASK, intr_mask);
}

/** Get responses for read requests */
static void tsb_i2c_read(struct tsb_i2c_info *info)
{
    int rx_valid;

    llvdbg("rx_index %d\n", info->rx_index);

    for (; info->rx_index < info->request_count; info->rx_index++) {
        uint32_t len;
        uint8_t *buffer;

        if (!(info->requests[info->rx_index].flags & I2C_FLAG_READ))
            continue;

        if (!(info->status & TSB_I2C_STATUS_READ_IN_PROGRESS)) {
            len = info->requests[info->rx_index].length;
            buffer = info->requests[info->rx_index].buffer;
        } else {
            len = info->rx_length;
            buffer = info->rx_buffer;
        }

        rx_valid = i2c_read(info->reg_base, TSB_I2C_RXFLR);

        for (; len > 0 && rx_valid > 0; len--, rx_valid--) {
            *buffer++ = i2c_read(info->reg_base, TSB_I2C_DATA_CMD);
            info->rx_outstanding--;
        }

        if (len > 0) {
            /* start the read process */
            info->status |= TSB_I2C_STATUS_READ_IN_PROGRESS;
            info->rx_length = len;
            info->rx_buffer = buffer;

            return;
        } else {
            info->status &= ~TSB_I2C_STATUS_READ_IN_PROGRESS;
        }
    }
}

/**
 * Watchdog handler for timeout of I2C operation
 */
static void tsb_i2c_timeout(int argc, uint32_t arg, ...)
{
    struct tsb_i2c_info *info = (struct tsb_i2c_info*)arg;

    lldbg("\n");

    irqstate_t flags = irqsave();

    if (info->status != TSB_I2C_STATUS_IDLE)
    {
        lldbg("finished\n");
        /* Mark the transfer as finished */
        info->status = TSB_I2C_STATUS_TIMEOUT;
        sem_post(&info->wait);
    }

    irqrestore(flags);
}

/** Handle transmission abort */
static int tsb_i2c_handle_tx_abort(struct tsb_i2c_info *info)
{
    unsigned long abort_source = info->abort_source;

    llvdbg("%s: 0x%x\n", __func__, abort_source);

    if (abort_source & TSB_I2C_TX_ABRT_NOACK) {
        lldbg("%s: TSB_I2C_TX_ABRT_NOACK 0x%x\n", __func__, abort_source);
        return -EREMOTEIO;
    }

    if (abort_source & TSB_I2C_TX_ARB_LOST)
        return -EAGAIN;
    else if (abort_source & TSB_I2C_TX_ABRT_GCALL_READ)
        return -EINVAL; /* wrong g_msgs[] data */
    else
        return -EIO;
}

/** Perform a sequence of I2C requests */
static int tsb_i2c_transfer(struct device *dev,
        struct device_i2c_request *requests, uint32_t count)
{
#ifdef CONFIG_PM
    irqstate_t flags;
#endif
    int ret;
    struct tsb_i2c_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

#ifdef CONFIG_PM
    flags = irqsave();
    /*
     * We're going to get stuck on sem_wait(&g_wait) if we're sleeping because
     * the i2c interrupts are disabled, so bail-out immediately. The user
     * trying to send data over a sleeping bridge should see it as an
     * input-output error.
     */
    if (tsb_pm_getstate() == PM_SLEEP) {
        irqrestore(flags);
        return -EIO;
    }

    /*
     * Call pm_activity() with interrupts disabled to make sure the pm
     * framework won't enter the sleep state between it and the above check.
     */
    pm_activity(TSB_I2C_ACTIVITY);
    irqrestore(flags);
#endif

    sem_wait(&info->mutex);
    llvdbg("requests: %d\n", count);

    /* initialize new requests */
    info->requests = requests;
    info->request_count = count;
    info->tx_index = 0;
    info->rx_index = 0;
    info->rx_outstanding = 0;

    info->cmd_err = 0;
    info->msg_err = 0;
    info->status = TSB_I2C_STATUS_IDLE;
    info->abort_source = 0;

    ret = tsb_i2c_wait_bus_ready(info);
    if (ret < 0)
        goto done;

    /* start a watchdog to timeout the transfer if the bus is locked up... */
    wd_start(info->timeout, TSB_I2C_TRANSFER_TIMEOUT, tsb_i2c_timeout, 1, info);

    /* start the transfers */
    tsb_i2c_start_transfer(info);

    /* wait for completion */
    sem_wait(&info->wait);

    /* cancel the watchdog */
    wd_cancel(info->timeout);

    /* check the result of the transfers */
    if (info->status == TSB_I2C_STATUS_TIMEOUT) {
        lldbg("controller timed out\n");

        /* Re-init the adapter */
        tsb_i2c_init(info);
        ret = -ETIMEDOUT;
        goto done;
    }

    tsb_i2c_disable(info);

    if (info->msg_err) {
        ret = info->msg_err;
        lldbg("error msg_err %x\n", info->msg_err);
        goto done;
    }

    if (!info->cmd_err) {
        ret = 0;
        llvdbg("no error %d\n", count);
        goto done;
    }

    /* Handle abort errors */
    if (info->cmd_err == TSB_I2C_ERR_TX_ABRT) {
        ret = tsb_i2c_handle_tx_abort(info);
        goto done;
    }

    /* default error code */
    ret = -EIO;
    lldbg("unknown error %x\n", ret);

done:
    sem_post(&info->mutex);

    return ret;
}

static uint32_t tsb_i2c_read_clear_intrbits(struct tsb_i2c_info *info)
{
    uint32_t stat = i2c_read(info->reg_base, TSB_I2C_INTR_STAT);

    if (stat & TSB_I2C_INTR_RX_UNDER)
        i2c_read(info->reg_base, TSB_I2C_CLR_RX_UNDER);
    if (stat & TSB_I2C_INTR_RX_OVER)
        i2c_read(info->reg_base, TSB_I2C_CLR_RX_OVER);
    if (stat & TSB_I2C_INTR_TX_OVER)
        i2c_read(info->reg_base, TSB_I2C_CLR_TX_OVER);
    if (stat & TSB_I2C_INTR_RD_REQ)
        i2c_read(info->reg_base, TSB_I2C_CLR_RD_REQ);
    if (stat & TSB_I2C_INTR_TX_ABRT) {
        /* IC_TX_ABRT_SOURCE reg is cleared upon read, store it */
        info->abort_source = i2c_read(info->reg_base, TSB_I2C_TX_ABRT_SOURCE);
        i2c_read(info->reg_base, TSB_I2C_CLR_TX_ABRT);
    }
    if (stat & TSB_I2C_INTR_RX_DONE)
        i2c_read(info->reg_base, TSB_I2C_CLR_RX_DONE);
    if (stat & TSB_I2C_INTR_ACTIVITY)
        i2c_read(info->reg_base, TSB_I2C_CLR_ACTIVITY);
    if (stat & TSB_I2C_INTR_STOP_DET)
        i2c_read(info->reg_base, TSB_I2C_CLR_STOP_DET);
    if (stat & TSB_I2C_INTR_START_DET)
        i2c_read(info->reg_base, TSB_I2C_CLR_START_DET);
    if (stat & TSB_I2C_INTR_GEN_CALL)
        i2c_read(info->reg_base, TSB_I2C_CLR_GEN_CALL);

    return stat;
}

/** I2C interrupt service routine */
static int tsb_i2c_irq_handler(int irq, void *context)
{
    uint32_t stat, enabled;
    struct tsb_i2c_info *info;

    if (!saved_dev || !device_get_private(saved_dev)) {
        return ERROR;
    }

    info = device_get_private(saved_dev);

    enabled = i2c_read(info->reg_base, TSB_I2C_ENABLE);
    stat = i2c_read(info->reg_base, TSB_I2C_RAW_INTR_STAT);

    lldbg("enabled=0x%x stat=0x%x\n", enabled, stat);

    if (!enabled || !(stat & ~TSB_I2C_INTR_ACTIVITY))
        return -1;

    stat = tsb_i2c_read_clear_intrbits(info);

    if (stat & TSB_I2C_INTR_TX_ABRT) {
        lldbg("abort\n");
        info->cmd_err |= TSB_I2C_ERR_TX_ABRT;
        info->status = TSB_I2C_STATUS_IDLE;

        tsb_i2c_disable_int(info);
        goto tx_aborted;
    }

    if (stat & TSB_I2C_INTR_RX_FULL)
        tsb_i2c_read(info);

    if (stat & TSB_I2C_INTR_TX_EMPTY)
        tsb_i2c_transfer_request(info);

tx_aborted:
    if (stat & TSB_I2C_INTR_TX_ABRT)
        lldbg("aborted %x %x\n", stat, info->abort_source);

    if ((stat & (TSB_I2C_INTR_TX_ABRT | TSB_I2C_INTR_STOP_DET)) || info->msg_err) {
        lldbg("release sem\n");
        sem_post(&info->wait);
    }

    return 0;
}

/** Check device driver open state */
static int tsb_i2c_device_is_open(struct tsb_i2c_info *info)
{
    return (info->flags & TSB_I2C_FLAG_OPENED) ? true : false;
}

/** Device driver open function */
static int tsb_i2c_dev_open(struct device *dev)
{
    struct tsb_i2c_info *info = NULL;
    int ret = 0;

    lldbg("opening i2c device\n");

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->mutex);

    if (tsb_i2c_device_is_open(info)) {
        lldbg("i2c device is alreay open!\n");
        ret = -EBUSY;
        goto err_open;
    }

    up_enable_irq(info->i2c_irq);

    info->flags = TSB_I2C_FLAG_OPENED;

err_open:
    sem_post(&info->mutex);

    return ret;
}

/** Device driver close function */
static void tsb_i2c_dev_close(struct device *dev)
{
    struct tsb_i2c_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    sem_wait(&info->mutex);

    if (!tsb_i2c_device_is_open(info)) {
        goto err_close;
    }

    up_disable_irq(info->i2c_irq);

    info->flags = 0;

err_close:
    sem_post(&info->mutex);
}

#ifdef CONFIG_PM
static void tsb_i2c_pm_notify(struct pm_callback_s *cb,
                              enum pm_state_e pmstate)
{
    irqstate_t flags;

    flags = irqsave();

    switch (pmstate) {
    case PM_NORMAL:
        tsb_clk_enable(TSB_CLK_I2CP);
        tsb_clk_enable(TSB_CLK_I2CS);

        tsb_reset(TSB_RST_I2CP);
        tsb_reset(TSB_RST_I2CS);
        break;
    case PM_IDLE:
    case PM_STANDBY:
        /* Nothing to do in idle or standby. */
        break;
    case PM_SLEEP:
        tsb_clk_disable(TSB_CLK_I2CP);
        tsb_clk_disable(TSB_CLK_I2CS);
        break;
    default:
        /* Can never happen. */
        PANIC();
    }

    irqrestore(flags);
}
#else
static void tsb_i2c_pm_notify(struct pm_callback_s *cb,
                              enum pm_state_e pmstate)
{

}
#endif

static int tsb_i2c_pm_prepare(struct pm_callback_s *cb,
                              enum pm_state_e pmstate)
{
    return OK;
}

/**
 * Initialise an I2C device
 */
static int tsb_i2c_dev_probe(struct device *dev)
{
    struct tsb_i2c_info *info = NULL;
    struct device_resource *r;
    irqstate_t flags;
    int ret = 0;

    lldbg("probing i2c device!\n");

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
       return -ENOMEM;
    }

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REG, "i2c_base");
    if (!r) {
        ret = -EINVAL;
        lldbg("get i2c_base error!\n");
        goto err_free_info;
    }

    info->reg_base = (uint32_t)r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_IRQ, "i2cintr");
    if (!r) {
        ret = -EINVAL;
        goto err_free_info;
    }

    info->i2c_irq = (uint32_t)r->start;

    flags = irqsave();

    ret = irq_attach(info->i2c_irq, tsb_i2c_irq_handler);
    if (ret) {
        irqrestore(flags);
        lldbg("i2c irq attach error!\n");
        ret = -EINTR;
        goto err_free_info;
    }

    irqrestore(flags);

    ret = tsb_request_pinshare(TSB_PIN_I2C | TSB_PIN_GPIO21 | TSB_PIN_GPIO22);
    if (ret) {
        lowsyslog("I2C: cannot get ownership of I2C pins\n");
        goto err_irq_detach;
    }

    /* enable I2C pins */
    tsb_clr_pinshare(TSB_PIN_I2C);
    tsb_clr_pinshare(TSB_PIN_GPIO21);
    tsb_clr_pinshare(TSB_PIN_GPIO22);

    /* enable I2C clocks */
    tsb_clk_enable(TSB_CLK_I2CP);
    tsb_clk_enable(TSB_CLK_I2CS);

    /* reset I2C module */
    tsb_reset(TSB_RST_I2CP);
    tsb_reset(TSB_RST_I2CS);

    /* Initialize the I2C controller */
    tsb_i2c_init(info);

    /* Allocate a watchdog timer */
    info->timeout = wd_create();
    DEBUGASSERT(info->timeout != 0);

    device_set_private(dev, info);
    saved_dev = dev;

    sem_init(&info->mutex, 0, 1);
    sem_init(&info->wait, 0, 0);

    ret = tsb_pm_register(tsb_i2c_pm_prepare, tsb_i2c_pm_notify, &dev);
    if (ret < 0) {
        lowsyslog("I2C: pm register\n");
        goto err_pm;
    }

    return 0;

err_pm:
    tsb_release_pinshare(TSB_PIN_I2C | TSB_PIN_GPIO21 | TSB_PIN_GPIO22);
    wd_delete(info->timeout);
    device_set_private(dev, NULL);
    sem_destroy(&info->mutex);
    sem_destroy(&info->wait);
err_irq_detach:
    irq_detach(info->i2c_irq);
err_free_info:
    free(info);
    return ret;
}

/**
 * Uninitialise an I2C device
 */
static void tsb_i2c_dev_remove(struct device *dev)
{
    struct tsb_i2c_info *info = NULL;
    irqstate_t flags;

    llvdbg("Deinit I2C port\n");

    if (!dev || !device_get_private(dev)) {
        return;
    }

    flags = irqsave();

    info = device_get_private(dev);

    tsb_release_pinshare(TSB_PIN_I2C | TSB_PIN_GPIO21 | TSB_PIN_GPIO22);

    sem_destroy(&info->mutex);
    sem_destroy(&info->wait);

    /* Detach Interrupt Handler */
    irq_detach(info->i2c_irq);

    wd_delete(info->timeout);

    free(info);
    device_set_private(dev, NULL);

    irqrestore(flags);
    return;
}

static struct device_i2c_type_ops tsb_i2c_type_ops = {
    .transfer   = tsb_i2c_transfer,
};

static struct device_driver_ops tsb_i2c_driver_ops = {
    .probe      = tsb_i2c_dev_probe,
    .remove     = tsb_i2c_dev_remove,
    .open       = tsb_i2c_dev_open,
    .close      = tsb_i2c_dev_close,
    .type_ops   = &tsb_i2c_type_ops,
};

struct device_driver tsb_i2c_driver = {
    .type   = DEVICE_TYPE_I2C_HW,
    .name   = "tsb_i2c",
    .desc   = "TSB I2C Driver",
    .ops    = &tsb_i2c_driver_ops,
};
