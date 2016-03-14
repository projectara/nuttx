/*
 * Copyright (c) 2016 Google, Inc.
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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/gpio.h>
#include <nuttx/device_spi.h>
#include <nuttx/device_spi_board.h>
#include <nuttx/ara/spi_board.h>
#include <nuttx/power/pm.h>
#include <arch/tsb/pm.h>

#include "up_arch.h"
#include "tsb_scm.h"

#define DW_SPI_CTRLR0   0x00
#define DW_SPI_CTRLR1   0x04
#define DW_SPI_SSIENR   0x08
#define DW_SPI_SER      0x10
#define DW_SPI_BAUDR    0x14
#define DW_SPI_TXFTLR   0x18
#define DW_SPI_RXFTLR   0x1C
#define DW_SPI_TXFLR    0x20
#define DW_SPI_RXFLR    0x24
#define DW_SPI_SR       0x28
#define DW_SPI_IMR      0x2C
#define DW_SPI_ISR      0x30
#define DW_SPI_RISR     0x34

#define DW_SPI_TXOICR   0x38
#define DW_SPI_RXOICR   0x3C
#define DW_SPI_RXUICR   0x40
#define DW_SPI_MSTICR   0x44
#define DW_SPI_ICR      0x48

#define DW_SPI_DMACR    0x4C
#define DW_SPI_DMATDLR  0x50
#define DW_SPI_DMARDLR  0x54
#define DW_SPI_DR       0x60

/** bit for DW_SPI_CTRLR0 */
#define SPI_CTRLR0_SCPH         BIT(6)
#define SPI_CTRLR0_SCPOL        BIT(7)
#define SPI_CTRLR0_SRL          BIT(11)
#define SPI_CTRL0_TMOD_MASK     (0x03 << 8)
#define SPI_CTRLR0_DFS32_OFFSET 16
#define SPI_CTRLR0_DFS32_MASK   (0x1f << SPI_CTRLR0_DFS32_OFFSET)

/** bit for DW_SPI_SR */
#define SPI_SR_BUSY_MASK    BIT(0)
#define SPI_SR_TFNF_MASK    BIT(1)
#define SPI_SR_TFE_MASK     BIT(2)
#define SPI_SR_RFNE_MASK    BIT(3)
#define SPI_SR_RFF_MASK     BIT(4)
#define SPI_SR_TXE_MASK     BIT(5)
#define SPI_SR_DCOL_MASK    BIT(6)

/** bit for DW_SPI_IMR */
#define SPI_IMR_TXEIM_MASK  BIT(0)
#define SPI_IMR_TXOIM_MASK  BIT(1)
#define SPI_IMR_RXUIM_MASK  BIT(2)
#define SPI_IMR_RXOIM_MASK  BIT(3)
#define SPI_IMR_RXFIM_MASK  BIT(4)
#define SPI_IMR_MSTIM_MASK  BIT(5)

/** bit for DW_SPIISR */
#define SPI_ISR_TXEIS_MASK  BIT(0)
#define SPI_ISR_TXOIS_MASK  BIT(1)
#define SPI_ISR_RXUIS_MASK  BIT(2)
#define SPI_ISR_RXOIS_MASK  BIT(3)
#define SPI_ISR_RXFIS_MASK  BIT(4)
#define SPI_ISR_MSTIS_MASK  BIT(5)

/** SPI master characteristics */
#define SPI_BUS_CLOCK       48000000    /* SPI bus clock: 48MHz */
#define SPI_MAX_FREQ        (SPI_BUS_CLOCK >> 1)    /* Maximum frequency: 24MHz */
#define SPI_MIN_FREQ        15000       /* Minimum frequency: 15kHz */
#define SPI_MAX_DIV         0xFFFE      /* Maximum Divider */
#define SPI_BPW_MASK        0xFFFFFFF8  /* Minimum of 4 bits-per-word */
#define SPI_TX_FIFO_DEPTH   16          /* TX FIFO depth */
#define SPI_RX_FIFO_DEPTH   16          /* RX FIFO depth */
#define _SPI_BASE_MODE      (SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_MODE_LOOP)
#if defined(CONFIG_TSB_SPI_GPIO)
# define SPI_MAX_SLAVES      16
# define SPI_DEFAULT_MODE   (_SPI_BASE_MODE | SPI_MODE_CS_HIGH)
#else
# define SPI_MAX_SLAVES      2
# define SPI_DEFAULT_MODE   _SPI_BASE_MODE
#endif

#define TSB_SPI_ACTIVITY           9

/**
 * SPI device state
 */
enum tsb_spi_state {
    TSB_SPI_STATE_INVALID,
    TSB_SPI_STATE_CLOSED,
    TSB_SPI_STATE_OPEN,
    TSB_SPI_STATE_LOCKED,
};

/**
 * Transfer information
 */
struct xfer_info {
    /** TX buffer pointer */
    uint8_t *tx;

    /** RX buffer pointer */
    uint8_t *rx;

    /** bits per word used for this transfer */
    uint8_t bpw;

    /** chip-select pin active high for this transfer */
    uint8_t cs_high;

    /** Tx buffer remaining bytes */
    uint32_t tx_remaining;

    /** Tx buffer remaining bytes */
    uint32_t rx_remaining;

    /** transmission status */
    int status;
};

static struct device *spi_dev = NULL;

/**
 * @brief private SPI device information
 */
struct tsb_spi_dev_info {
    /** SPI device base address */
    uint32_t reg_base;

    /** current transfer information */
    struct xfer_info curr_xfer;

    /** Exclusive access for SPI bus */
    sem_t bus;

    /** Exclusive access for operation */
    sem_t lock;

    /** RX thread notification flag */
    sem_t xfer_completed;

    /** pointers to SPI slaves */
    struct device *dev_spi_board[SPI_MAX_SLAVES];

    /** number of SPI slaves attached */
    uint8_t num_boards;

    /** SPI controller power state */
    int spi_pmstate;

    /** SPI device state */
    enum tsb_spi_state state;
};

/**
 * @brief Read value from register.
 *
 * This function returns register content by basic register read function.
 *
 * @param base Base address of this Controller.
 * @param addr Specific register of offset.
 *
 * @return Returns content for a specific register.
 */
static uint32_t tsb_spi_read(uint32_t base, uint32_t addr)
{
    return getreg32(base + addr);
}

/**
 * @brief Write value to register.
 *
 * This function write value to register by basic register write function.
 *
 * @param base Base address of this Controller.
 * @param addr Specific register of offset.
 * @param val The content will be write for Specific register.
 */
static void tsb_spi_write(uint32_t base, uint32_t addr, uint32_t val)
{
    putreg32(val, base + addr);
}

/**
 * @brief Lock SPI bus for exclusive access
 *
 * On SPI buses where there are multiple devices, it will be necessary to lock
 * SPI to have exclusive access to the buses for a sequence of transfers.
 * The bus should be locked before the chip is selected. After locking the SPI
 * bus, the caller should then also call the setfrequency(), setbpw() , and
 * setmode() methods to make sure that the SPI is properly configured for the
 * device. If the SPI buses is being shared, then it may have been left in an
 * incompatible state.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_lock(struct device *dev)
{
    struct tsb_spi_dev_info *info = NULL;
    int ret = 0, loop = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->spi_pmstate == PM_SLEEP) {
        pm_activity(TSB_SPI_ACTIVITY);

        /* SPI not powered, fail any accessing */;
        while (info->spi_pmstate == PM_SLEEP) {
            usleep(1000);
            if (++loop > 10) {
                return -EIO;
            }
        }
    }

    /* Take the semaphore (perhaps waiting) */
    ret = sem_wait(&info->bus);
    if (ret != OK) {
        /* The sem_wait() call should fail only if we are awakened by
         * a signal.
         */
        return -get_errno();
    }
    info->state = TSB_SPI_STATE_LOCKED;
    return 0;
}

/**
 * @brief unlock SPI bus for exclusive access
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_unlock(struct device *dev)
{
    struct tsb_spi_dev_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_OPEN;
    sem_post(&info->bus);
    return 0;
}

/**
 * @brief Enable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_select(struct device *dev, uint8_t devid)
{
    struct tsb_spi_dev_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_select;
    }

    if (devid >=  info->num_boards) {
        ret = -EINVAL;
        goto err_select;
    }
#if defined(CONFIG_TSB_SPI_GPIO)
    /* from the SPI master's point of view, we fake that we're always talking to
     * slave #1. The trick is that by default, CS1 doesn't belong to the SPI
     * master, so the transaction will still be initiated and meanwhile we can
     * manually setup the CS with GPIOs the way we want. */
    tsb_spi_write(info->reg_base, DW_SPI_SER, 0x2);
#else
    /* otherwise we really select the slave through the SPI master */
    tsb_spi_write(info->reg_base, DW_SPI_SER, (1 << devid));
#endif

    ret = device_spi_board_cs_select(info->dev_spi_board[devid],
            info->curr_xfer.cs_high);

err_select:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Disable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock(),
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_deselect(struct device *dev, uint8_t devid)
{
    struct tsb_spi_dev_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_deselect;
    }

    tsb_spi_write(info->reg_base, DW_SPI_SER, 0);

    ret = device_spi_board_cs_select(info->dev_spi_board[devid],
            !info->curr_xfer.cs_high);

err_deselect:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Configure SPI clock.
 *
 * If SPI hardware doesn’t support this frequency value, this function should
 * find the nearest lower frequency in which hardware supported and then
 * configure SPI clock to this value. It will return the actual frequency
 * selected value back to the caller via parameter frequency.
 * This function should be called after lock(), if the driver is not in lock
 * state, it returns an error code to notify a problem.
 *
 * @param dev pointer to structure of device data
 * @param cs the specific chip number
 * @param frequency SPI frequency requested (unit: Hz)
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setfrequency(struct device *dev, uint8_t cs,
                                uint32_t *frequency)
{
    struct tsb_spi_dev_info *info = NULL;
    uint32_t freq;
    uint32_t max_freq_slave;
    uint32_t div;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !frequency || !(*frequency)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (cs >= info->num_boards) {
        ret = -EINVAL;
        goto err_freq_set;
    }

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_freq_set;
    }

    freq = *frequency;
    ret = device_spi_board_get_max_speed_hz(info->dev_spi_board[cs], &max_freq_slave);
    if (ret)
        goto err_freq_set;

    /* check the frequency range */
    if (freq > SPI_MAX_FREQ || freq < SPI_MIN_FREQ || freq > max_freq_slave) {
        ret = -EINVAL;
        goto err_freq_set;
    }

    div = SPI_BUS_CLOCK / freq;
    /* the 'div' doesn't support odd number */
    div = (div + 1) & 0xFFFE;

    if (div > SPI_MAX_DIV) {
        ret = -EINVAL;
        goto err_freq_set;
    }

    freq = SPI_BUS_CLOCK / div;

    tsb_spi_write(info->reg_base, DW_SPI_BAUDR, div);

    *frequency = freq;
err_freq_set:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Configure SPI mode.
 *
 * To configure SPI configuration such as clock polarity and phase via the mode
 * parameter. Other possible definition of SPI mode can be found in SPI mode
 * definition. If the value of mode parameter is out of SPI mode definition or
 * this mode isn’t supported by the current hardware, this function should
 * return -EOPNOTSUPP error code.
 * This function should be called after lock(), if driver is not in lock state,
 * function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param cs the specific chip number
 * @param mode SPI protocol mode requested
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setmode(struct device *dev, uint8_t cs, uint8_t mode)
{
    struct tsb_spi_dev_info *info = NULL;
    uint32_t ctrl0 = 0;
    uint16_t mode_slave;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (cs >= info->num_boards) {
        ret = -EINVAL;
        goto err_setmode;
    }

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_setmode;
    }

    ret = device_spi_board_get_mode(info->dev_spi_board[cs], &mode_slave);
    if (ret)
        goto err_setmode;

    /* check hardware mode capabilities */
    if (((mode & SPI_DEFAULT_MODE) != mode) &&
            ((mode & mode_slave) != mode)) {
        ret = -EINVAL;
        goto err_setmode;
    }

    ctrl0 = tsb_spi_read(info->reg_base, DW_SPI_CTRLR0);

    /* set SPI clock phase */
    if (mode & SPI_MODE_CPHA)
        ctrl0 |= SPI_CTRLR0_SCPH;
    else
        ctrl0 &= ~SPI_CTRLR0_SCPH;

    /* set SPI clock polarity */
    if (mode & SPI_MODE_CPOL)
        ctrl0 |= SPI_CTRLR0_SCPOL;
    else
        ctrl0 &= ~SPI_CTRLR0_SCPOL;

    /* set SPI loopback mode */
    if (mode & SPI_MODE_LOOP)
        ctrl0 |= SPI_CTRLR0_SRL;
    else
        ctrl0 &= ~SPI_CTRLR0_SRL;

    /* SPI CS_HIGH mode */
    if (mode & SPI_MODE_CS_HIGH)
        info->curr_xfer.cs_high = 1;
    else
        info->curr_xfer.cs_high = 0;

    tsb_spi_write(info->reg_base, DW_SPI_CTRLR0, ctrl0);

err_setmode:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Set the number of bits per word in transmission.
 *
 * This function should be called after lock(), if driver is not in lock state,
 * this function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param cs the specific chip number
 * @param bpw The number of bpw requested. The bpw value range is from
 *        4 to 32.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setbpw(struct device *dev, uint8_t cs, uint8_t bpw)
{
    struct tsb_spi_dev_info *info = NULL;
    uint32_t ctrl0 = 0;
    uint8_t bpw_slave;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (cs >= info->num_boards) {
        ret = -EINVAL;
        goto exit_setbit;
    }

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto exit_setbit;
    }

    ret = device_spi_board_get_bpw(info->dev_spi_board[cs], &bpw_slave);
    if (ret)
        goto exit_setbit;

    /* check hardware bpw capabilities */
    if (!(BIT(bpw - 1) & SPI_BPW_MASK) || (bpw != bpw_slave)) {
        ret = -EINVAL;
        goto exit_setbit;
    }

    ctrl0 = tsb_spi_read(info->reg_base, DW_SPI_CTRLR0);
    ctrl0 &= ~SPI_CTRLR0_DFS32_MASK;
    ctrl0 |= ((bpw - 1) << SPI_CTRLR0_DFS32_OFFSET);

    tsb_spi_write(info->reg_base, DW_SPI_CTRLR0, ctrl0);
    info->curr_xfer.bpw = bpw;

exit_setbit:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Exchange a block of data from SPI
 *
 * Device driver uses this function to transfer and receive data from SPI bus.
 * This function should be called after lock() , if the driver is not in lock
 * state, it returns -EPERM error code.
 * The transfer structure is consists of the read/write buffer, transfer
 * length, transfer flags and callback function.
 *
 * @param dev pointer to structure of device data
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_exchange(struct device *dev,
                             struct device_spi_transfer *transfer)
{
    struct tsb_spi_dev_info *info = NULL;
    uint32_t imr_reg;
    int ret = 0;
    struct timespec abstime;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !transfer) {
        return -EINVAL;
    }

    /* check transfer buffer */
    if (!transfer->txbuffer && !transfer->rxbuffer) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_unlock;
    }

    info->curr_xfer.status = 0;
    info->curr_xfer.tx = transfer->txbuffer;
    info->curr_xfer.rx = transfer->rxbuffer;

    /* event has read request, we still need to read data because SPI data
     * exchange behavior */
    info->curr_xfer.tx_remaining = transfer->nwords;
    info->curr_xfer.rx_remaining = transfer->nwords;

    /* enable interrupt */
    imr_reg = SPI_IMR_TXEIM_MASK | SPI_IMR_RXOIM_MASK | SPI_IMR_RXUIM_MASK;
    tsb_spi_write(info->reg_base, DW_SPI_IMR,
                  (tsb_spi_read(info->reg_base, DW_SPI_IMR) | imr_reg));

    (void)clock_gettime(CLOCK_REALTIME, &abstime);
    abstime.tv_nsec += info->curr_xfer.tx_remaining * (100 * 1000);

    /* Enable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 1);
    ret = sem_timedwait(&info->xfer_completed, &abstime);

    /* get transmission status */
    if (info->curr_xfer.status) {
        ret = info->curr_xfer.status;
    }

err_unlock:
    /* Disable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 0);
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Read SPI data from FIFO
 *
 * @param info pointer to the tsb_spi_dev_info struct.
 * @return 0 on success, negative errno on error
 */
int tsb_spi_process_rx(struct tsb_spi_dev_info *info)
{
    struct xfer_info *xfer = &info->curr_xfer;
    uint32_t rx_left, rx_reads, data;

    /* calculate how much available data in Rx FIFO */
    rx_left = tsb_spi_read(info->reg_base, DW_SPI_RXFLR);
    rx_reads = (rx_left > xfer->rx_remaining)? xfer->rx_remaining : rx_left;

    while (rx_reads--) {
        data = tsb_spi_read(info->reg_base, DW_SPI_DR);

        if (xfer->bpw <= 8) {
            if (xfer->rx) {
                *(uint8_t *)(xfer->rx) = data;
                xfer->rx += 1;
            }
        } else if (xfer->bpw <= 16) {
            if (xfer->rx) {
                *(uint16_t *)(xfer->rx) = data;
                xfer->rx += 2;
            }
        } else if (xfer->bpw <= 32) {
            if (xfer->rx) {
                *(uint32_t *)(xfer->rx) = data;
                xfer->rx += 4;
            }
        } else {
            return -EINVAL;
        }
        xfer->rx_remaining--;
    }
    return 0;
}

/**
 * @brief Write SPI data to FIFO
 *
 * @param info pointer to the tsb_spi_dev_info struct.
 * @return 0 on success, negative errno on error
 */
int tsb_spi_process_tx(struct tsb_spi_dev_info *info)
{
    struct xfer_info *xfer = &info->curr_xfer;
    uint32_t tx_room, tx_writes, data = 0;

    /* calculate how much available space in Tx FIFO */
    tx_room = SPI_TX_FIFO_DEPTH - tsb_spi_read(info->reg_base, DW_SPI_TXFLR);
    tx_writes = (xfer->tx_remaining > tx_room)? tx_room : xfer->tx_remaining;

    while (tx_writes--) {
        if (xfer->bpw <= 8) {
            if (xfer->tx) {
                data = *(uint8_t *)(xfer->tx);
                xfer->tx += 1;
            }
        } else if (xfer->bpw <= 16) {
            if (xfer->tx) {
                data = *(uint16_t *)(xfer->tx);
                xfer->tx += 2;
            }
        } else if (xfer->bpw <= 32) {
            if (xfer->tx) {
                data = *(uint32_t *)(xfer->tx);
                xfer->tx += 4;
            }
        } else {
            return -EINVAL;
        }
        tsb_spi_write(info->reg_base, DW_SPI_DR, data);
        xfer->tx_remaining--;
    }
    return 0;
}

/**
 * @brief SPI interrupt handler
 *
 * @param irq interrupt number
 * @param context argument for interrupt handler
 * @return 0 if successful, negative error code otherwise.
 */
static int tsb_spi_irq_handler(int irq, void *context)
{
    struct device *dev = spi_dev;
    struct tsb_spi_dev_info *info = NULL;
    uint32_t isr, imr;

    if (!dev || !device_get_private(dev)) {
        return ERROR;
    }
    info = device_get_private(dev);

    isr = tsb_spi_read(info->reg_base, DW_SPI_ISR);
    imr = tsb_spi_read(info->reg_base, DW_SPI_IMR);
    if (!isr && !imr) {
        /* ignore unexpected interrupt */
        return OK;
    }

    if (isr & (SPI_ISR_RXUIS_MASK | SPI_ISR_RXOIS_MASK)) {
        /* disable all interrupts */
        tsb_spi_write(info->reg_base, DW_SPI_IMR, 0);
        /* clean interrupt status */
        tsb_spi_read(info->reg_base, DW_SPI_RXOICR);
        tsb_spi_read(info->reg_base, DW_SPI_RXUICR);
        tsb_spi_read(info->reg_base, DW_SPI_ICR);

        /* abort the transfer and return error status*/
        info->curr_xfer.status = -EIO;
        sem_post(&info->xfer_completed);
        return ERROR;
    }

    /* receive data */
    tsb_spi_process_rx(info);

    if (!info->curr_xfer.rx_remaining) {
        /* disable Tx empty interrupt */
        tsb_spi_write(info->reg_base, DW_SPI_IMR, (imr & ~SPI_ISR_TXEIS_MASK));
        info->curr_xfer.status = 0;
        /* receive data completed */
        sem_post(&info->xfer_completed);
        return OK;
    }

    if (isr & SPI_ISR_TXEIS_MASK) {
        /* disable Tx empty interrupt */
        imr = tsb_spi_read(info->reg_base, DW_SPI_IMR);
        tsb_spi_write(info->reg_base, DW_SPI_IMR, (imr & ~SPI_ISR_TXEIS_MASK));
        /* transfer data */
        tsb_spi_process_tx(info);
        /* enable Tx empty interrupt */
        tsb_spi_write(info->reg_base, DW_SPI_IMR, (imr | SPI_ISR_TXEIS_MASK));
    }
    return OK;
}

/**
 * @brief Get SPI device driver hardware capabilities information.
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param master_config pointer to the spi_master_config structure to receive the capabilities
 *             information.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_get_master_config(struct device *dev,
                                     struct device_spi_master_config *master_config)
{
    struct tsb_spi_dev_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !master_config) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    master_config->mode = SPI_DEFAULT_MODE;
    master_config->flags = 0; /* nothing that we can't do */
    master_config->bpw_mask = SPI_BPW_MASK;
    master_config->csnum = info->num_boards;
    master_config->min_speed_hz = SPI_MIN_FREQ;
    master_config->max_speed_hz = SPI_MAX_FREQ;
    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Get SPI specific chip configured information.
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param cs the specific chip number
 * @param device_cfg pointer to the device_spi_device_config structure to receive
 * the configuration that be set in chip.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_get_device_config(struct device *dev, uint8_t cs,
                                     struct device_spi_device_config *device_cfg)
{
    struct tsb_spi_dev_info *info = NULL;
    struct device *spi_board;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !device_cfg) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (!info->dev_spi_board || cs >= info->num_boards) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    spi_board = info->dev_spi_board[cs];

    /* get device config */
    device_spi_board_get_mode(spi_board, &device_cfg->mode);
    device_spi_board_get_bpw(spi_board, &device_cfg->bpw);
    device_spi_board_get_max_speed_hz(spi_board, &device_cfg->max_speed_hz);
    device_spi_board_get_type(spi_board, &device_cfg->device_type);
    device_spi_board_get_name(spi_board, device_cfg->name);

    sem_post(&info->lock);
    return 0;
}

/**
 * @brief Deinitialize SPI controller
 *
 * @param info pointer to the tsb_spi_dev_info struct.
 */
static void tsb_spi_hw_deinit(struct tsb_spi_dev_info *info)
{
    uint32_t pinshare = 0;

    /* Disable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 0);

    /* release SPIM_CLK, SPIM_SDI and SPIM_SDO */
    pinshare |= TSB_PIN_GPIO13;
#if !defined(CONFIG_TSB_SPI_GPIO)
    /* release also SPIM_CS0 and SPIM_CS1 */
    pinshare |= (TSB_PIN_GPIO15 | TSB_PIN_SDIO | TSB_PIN_SPIM_CS1);
#endif
    tsb_release_pinshare(pinshare);

    tsb_clk_disable(TSB_CLK_SPIP);
    tsb_clk_disable(TSB_CLK_SPIS);
    info->spi_pmstate = PM_SLEEP;
}

/**
 * @brief Initialize SPI controller
 *
 * @param info pointer to the tsb_spi_dev_info struct.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_hw_init(struct tsb_spi_dev_info *info)
{
    uint32_t ctrl0, pinshare = 0;
    int ret = 0;

    /* Enable Clock */
    tsb_clk_enable(TSB_CLK_SPIP);
    tsb_clk_enable(TSB_CLK_SPIS);

    /* Reset */
    tsb_reset(TSB_RST_SPIP);
    tsb_reset(TSB_RST_SPIS);

    info->spi_pmstate = PM_NORMAL;

    /* Disable SPI controller */
    tsb_spi_write(info->reg_base, DW_SPI_SSIENR, 0);

    /* Disable SPI all interrupts */
    tsb_spi_write(info->reg_base, DW_SPI_IMR, 0);

    /* Clear interrupt status register */
    tsb_spi_read(info->reg_base, DW_SPI_ICR);

    /* The SPI support both transmit and receiver mode */
    ctrl0 = tsb_spi_read(info->reg_base, DW_SPI_CTRLR0);
    ctrl0 &= ~SPI_CTRL0_TMOD_MASK;
    tsb_spi_write(info->reg_base, DW_SPI_CTRLR0, ctrl0);

    /* get SPIM_CLK, SPIM_SDI and SPIM_SDO */
    pinshare |= TSB_PIN_GPIO13;
#if !defined(CONFIG_TSB_SPI_GPIO)
    /* get also SPIM_CS0 */
    pinshare |= TSB_PIN_GPIO15;
    /* and SPIM_CS1 */
    pinshare |= (TSB_PIN_SDIO | TSB_PIN_SPIM_CS1);
#endif

    ret = tsb_request_pinshare(pinshare);
    if (ret) {
        lowsyslog("SPI: cannot get ownership of SPI pins\n");
        goto err_req_pinshare;
    }
    /* Configure pin functionality for SPI */
    /* CLK, SDO, SDI */
    tsb_clr_pinshare(TSB_PIN_GPIO13);
#if !defined(CONFIG_TSB_SPI_GPIO)
    /* CS0_N */
    tsb_clr_pinshare(TSB_PIN_GPIO15);
    /* CS1_N */
    tsb_clr_pinshare(TSB_PIN_SDIO);
    tsb_set_pinshare(TSB_PIN_SPIM_CS1);
#endif
err_req_pinshare:
    return ret;
}

/**
 * @brief Open SPI device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_dev_info *info = NULL;
    struct device *spi_board;
    int i;
    int ret = 0;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_CLOSED) {
        ret = -EBUSY;
        goto err_open;
    }

    /* open all the consecutive slaves we can find */
    info->num_boards = 0;
    for (i = 0; i < SPI_MAX_SLAVES; i++) {
        spi_board = device_open(DEVICE_TYPE_SPI_BOARD_HW, i);
        if (spi_board) {
            info->num_boards++;
            info->dev_spi_board[i] = spi_board;
        } else
            break;
    }

    info->curr_xfer.cs_high = 0;
    info->curr_xfer.bpw = 8;

    ret = tsb_spi_hw_init(info);
    if (ret) {
        goto err_hwinit;
    }
    /* register SPI IRQ number */
    ret = irq_attach(TSB_IRQ_SPI, tsb_spi_irq_handler);
    if (ret != OK) {
        ret = -EIO;
        goto err_irq;
    }

    up_enable_irq(TSB_IRQ_SPI);

    info->state = TSB_SPI_STATE_OPEN;
    sem_post(&info->lock);
    return ret;

err_irq:
    tsb_spi_hw_deinit(info);
err_hwinit:
    for (i = 0; i < info->num_boards; i++) {
        device_close(info->dev_spi_board[i]);
        info->dev_spi_board[i] = NULL;
    }
err_open:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close SPI device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_close(struct device *dev)
{
    int i;
    struct tsb_spi_dev_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    up_disable_irq(TSB_IRQ_SPI);
    irq_detach(TSB_IRQ_SPI);

    tsb_spi_hw_deinit(info);

    for (i = 0; i < info->num_boards; i++) {
        device_close(info->dev_spi_board[i]);
        info->dev_spi_board[i] = NULL;
    }
    info->state = TSB_SPI_STATE_CLOSED;
    sem_post(&info->lock);
}

static int tsb_spi_suspend(struct device *dev)
{
    struct tsb_spi_dev_info *info;

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->curr_xfer.rx_remaining) {
        sem_post(&info->lock);
        return -EBUSY;
    }

    tsb_clk_disable(TSB_CLK_SPIP);
    tsb_clk_disable(TSB_CLK_SPIS);
    info->spi_pmstate = PM_SLEEP;

    sem_post(&info->lock);

    return 0;
}

static int tsb_spi_poweroff(struct device *dev)
{
    return tsb_spi_suspend(dev);
}

static int tsb_spi_resume(struct device *dev)
{
    struct tsb_spi_dev_info *info;

    info = device_get_private(dev);

    tsb_clk_enable(TSB_CLK_SPIP);
    tsb_clk_enable(TSB_CLK_SPIS);

    info->spi_pmstate = PM_NORMAL;

    return 0;
}

#ifdef CONFIG_PM
static void tsb_spi_pm_notify(struct pm_callback_s *cb,
                              enum pm_state_e pmstate)
{
    struct device *dev;
    irqstate_t flags;

    dev = cb->priv;

    flags = irqsave();

    switch (pmstate) {
    case PM_NORMAL:
        tsb_spi_resume(dev);
        break;
    case PM_IDLE:
    case PM_STANDBY:
        /* Nothing to do in idle or standby. */
        break;
    case PM_SLEEP:
        tsb_spi_suspend(dev);
        break;
    default:
        /* Can never happen. */
        PANIC();
    }

    irqrestore(flags);
}

static int tsb_spi_pm_prepare(struct pm_callback_s *cb,
                              enum pm_state_e pmstate)
{
    struct tsb_spi_dev_info *info = NULL;
    struct device *dev;
    irqstate_t flags;

    dev = cb->priv;
    info = device_get_private(dev);

    flags = irqsave();

    switch (pmstate) {
    case PM_NORMAL:
    case PM_IDLE:
    case PM_STANDBY:
        /* Nothing to do in idle or standby. */
        break;
    case PM_SLEEP:
        if (info->curr_xfer.rx_remaining) {
            /* return not ready to SLEEP because exchange not complete yet */
            irqrestore(flags);
            return -EIO;
        }
        break;
    default:
        /* Can never happen. */
        PANIC();
    }

    irqrestore(flags);
    return OK;
}
#else
static void tsb_spi_pm_notify(struct pm_callback_s *cb,
                               enum pm_state_e pmstate)
{

}
static int tsb_spi_pm_prepare(struct pm_callback_s *cb,
                              enum pm_state_e pmstate)
{
    return 0;
}
#endif

/**
 * @brief Probe SPI device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private SPI device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_probe(struct device *dev)
{
    struct tsb_spi_dev_info *info;
    struct device_resource *r;
    int ret = 0;

    if (tsb_get_rev_id() == tsb_rev_es2) {
        return -ENODEV;
    }

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    /* get register data from resource block */
    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REG, "reg_base");
    if (!r) {
        ret = -EINVAL;
        goto err_freemem;
    }

    info->reg_base = (uint32_t)r->start;
    info->state = TSB_SPI_STATE_CLOSED;
    info->spi_pmstate = PM_SLEEP;
    device_set_private(dev, info);
    spi_dev = dev;

    sem_init(&info->bus, 0, 1);
    sem_init(&info->lock, 0, 1);
    sem_init(&info->xfer_completed, 0, 0);

    ret = tsb_pm_register(tsb_spi_pm_prepare, tsb_spi_pm_notify, dev);
    if (ret) {
        goto err_freemem;
    }
    return 0;

err_freemem:
    free(info);
    return ret;
}

/**
 * @brief Remove SPI device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_remove(struct device *dev)
{
    struct tsb_spi_dev_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_INVALID;
    sem_destroy(&info->lock);
    sem_destroy(&info->bus);
    sem_destroy(&info->xfer_completed);
    device_set_private(dev, NULL);
    spi_dev = NULL;
    free(info);
}

static struct device_pm_ops tsb_spi_pm_ops = {
    .suspend    = tsb_spi_suspend,
    .poweroff   = tsb_spi_poweroff,
    .resume     = tsb_spi_resume,
};

static struct device_spi_type_ops tsb_spi_type_ops = {
    .lock               = tsb_spi_lock,
    .unlock             = tsb_spi_unlock,
    .select             = tsb_spi_select,
    .deselect           = tsb_spi_deselect,
    .setfrequency       = tsb_spi_setfrequency,
    .setmode            = tsb_spi_setmode,
    .setbpw             = tsb_spi_setbpw,
    .exchange           = tsb_spi_exchange,
    .get_master_config  = tsb_spi_get_master_config,
    .get_device_config  = tsb_spi_get_device_config,
};

static struct device_driver_ops tsb_spi_driver_ops = {
    .probe          = tsb_spi_dev_probe,
    .remove         = tsb_spi_dev_remove,
    .open           = tsb_spi_dev_open,
    .close          = tsb_spi_dev_close,
    .type_ops       = &tsb_spi_type_ops,
};

struct device_driver tsb_spi_driver = {
    .type       = DEVICE_TYPE_SPI_HW,
    .name       = "tsb_spi",
    .desc       = "TSB SPI Driver",
    .ops        = &tsb_spi_driver_ops,
    .pm         = &tsb_spi_pm_ops,
};
