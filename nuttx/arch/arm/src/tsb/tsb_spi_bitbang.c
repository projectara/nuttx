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
#include <nuttx/device_spi.h>
#include <nuttx/gpio.h>

#include <arch/tsb/chip.h>
#include <tsb_scm.h>

/**
 * Since the Toshiba bridge chip doesn't support SPI master feature on ES1/ES2
 * chip, we implemented the SPI bitbang driver to simulate SPI signal. The
 * Toshiba bridge chip has 16 GPIO pins, but GPIO#1~GPIO#15 pins have been
 * assigned to other purposes on BDB1B board.
 *
 * The SPI bitbang driver needs 4 GPIO pins to simulate SPI signal and the
 * BDB1B board can't provide enough free GPIO pins for the SPI bitbang driver
 * to be used. So I shared the JTAG debug pins (GPIO#10~GPIO#12) and GPIO#0 for SPI
 * bitbang driver used.
 */
#define SPI_SDO             10          /* GPIO 10 */
#define SPI_SDI             0           /* GPIO  0 */
#define SPI_SCK             12          /* GPIO 12 */
#define SPI_CS              11          /* GPIO 11 */

#define NUMOFCS             1

/* Because we use a simple delay loop mechanism to simulate the SPI signal,
 * any extra operation will impact the SPI timing. We need to eliminate such
 * factors, and to do so I minused out the bit transfer overhead, using an
 * oscilloscope to calibrate the XFER_OVERHEAD value to meet the SPI timing
 * signal.
 */
#define XFER_OVERHEAD       3           /* 3 microseconds */

/**
 * @brief SPI device state
 */
enum tsb_spi_state {
    TSB_SPI_STATE_INVALID,              /** Invalid SPI state */
    TSB_SPI_STATE_CLOSED,               /** Device has been closed */
    TSB_SPI_STATE_OPEN,                 /** Device has been opened */
    TSB_SPI_STATE_LOCKED,               /** SPI bus has been locked */
};

/**
 * Private SPI device information
 */
struct tsb_spi_info {
    /** Driver model representation of the device */
    struct device *dev;
    /** SPI device state */
    enum tsb_spi_state state;
    /** SPI hardware capabilities */
    struct device_spi_caps caps;

    /** SPI mode */
    uint16_t modes;
    /** SPI protocol flags */
    uint16_t flags;
    /** number of bits per word */
    uint32_t bpw;
    /** SPI frequency */
    uint32_t frequency;
    /** holdtime in microseconds */
    uint32_t holdtime;

    /** selected device id */
    int selected;
    /** GPIO array for chip-select pins */
    int chipselect[NUMOFCS];
    /** simulate SPI one bit transaction function */
    uint8_t (*bitexchange)(uint32_t outdata, uint32_t holdtime, bool loopback);

    /** Exclusive access for SPI bus */
    sem_t bus;
    /** Exclusive access for operation */
    sem_t lock;
};

/**
 * @brief Hardware initialization
 *
 * The function initializes the GPIO pins used in the bitbang interface and
 * also assigns a default output value for SPI signal.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_hw_init(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int i = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    /* setup GPIO pins */
    gpio_activate(SPI_SCK);
    gpio_direction_out(SPI_SCK, 0);

    gpio_activate(SPI_SDI);
    gpio_direction_in(SPI_SDI);

    gpio_activate(SPI_SDO);
    gpio_direction_out(SPI_SDO, 0);

    /* setup all chip-select pins */
    for (i = 0; i < info->caps.csnum; i++) {
        gpio_activate(info->chipselect[i]);
        gpio_direction_out(info->chipselect[i], 1);
    }

    return 0;
}

/**
 * @brief Hardware deinitialization
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_hw_deinit(struct device *dev) {
    struct tsb_spi_info *info = NULL;
    int i = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    /* release GPIO pins */
    for (i = 0; i < info->caps.csnum; i++) {
        gpio_deactivate(info->chipselect[i]);
    }
    gpio_deactivate(SPI_SDO);
    gpio_deactivate(SPI_SDI);
    gpio_deactivate(SPI_SCK);

    return 0;
}

/**
 * @brief Exchange one bit in mode 0
 *
 * The function simulates a one bit transaction function in SPI mode 0.
 *
 * @param outdata data for SPI output
 * @param holdtime SPI hold time timing
 * @param loopback lookback mode supported or not
 * @return input data that read from SPI input pin
 */
static uint8_t tsb_spi_bitexchange0(uint32_t outdata, uint32_t holdtime, bool loopback)
{
    uint8_t indata = 0;

    gpio_set_value(SPI_SDO, (outdata)? 1 : 0);
    up_udelay(holdtime);
    gpio_set_value(SPI_SCK, 1);
    indata = gpio_get_value((loopback)? SPI_SDO : SPI_SDI);
    up_udelay(holdtime);
    gpio_set_value(SPI_SCK, 0);
    return indata;
}

/**
 * @brief Exchange one bit in mode 1
 *
 * The function simulates a one bit transaction function in SPI mode 1.
 *
 * @param outdata data for SPI output
 * @param holdtime SPI hold time timing
 * @param loopback lookback mode supported or not
 * @return input data that read from SPI input pin
 */
static uint8_t tsb_spi_bitexchange1(uint32_t outdata, uint32_t holdtime, bool loopback)
{
    uint8_t indata = 0;

    gpio_set_value(SPI_SCK, 1);
    gpio_set_value(SPI_SDO, (outdata)? 1 : 0);
    up_udelay(holdtime);
    gpio_set_value(SPI_SCK, 0);
    indata = gpio_get_value((loopback)? SPI_SDO : SPI_SDI);
    up_udelay(holdtime);

    return indata;
}

/**
 * @brief Exchange one bit in mode 2
 *
 * The function simulates a one bit transaction function in SPI mode 2.
 *
 * @param outdata data for SPI output
 * @param holdtime SPI hold time timing
 * @param loopback lookback mode supported or not
 * @return input data that read from SPI input pin
 */
static uint8_t tsb_spi_bitexchange2(uint32_t outdata, uint32_t holdtime, bool loopback)
{
    uint8_t indata = 0;

    gpio_set_value(SPI_SDO, (outdata)? 1 : 0);
    up_udelay(holdtime);
    gpio_set_value(SPI_SCK, 0);
    indata = gpio_get_value((loopback)? SPI_SDO : SPI_SDI);
    up_udelay(holdtime);
    gpio_set_value(SPI_SCK, 1);

    return indata;
}

/**
 * @brief Exchange one bit in mode 3
 *
 * The function simulates a one bit transaction function in SPI mode 3.
 *
 * @param outdata data for SPI output
 * @param holdtime SPI hold time timing
 * @param loopback lookback mode supported or not
 * @return input data that read from SPI input pin
 */
static uint8_t tsb_spi_bitexchange3(uint32_t outdata, uint32_t holdtime, bool loopback)
{
    uint8_t indata = 0;

    gpio_set_value(SPI_SCK, 0);
    gpio_set_value(SPI_SDO, (outdata)? 1 : 0);
    up_udelay(holdtime);
    gpio_set_value(SPI_SCK, 1);
    indata = gpio_get_value((loopback)? SPI_SDO : SPI_SDI);
    up_udelay(holdtime);

    return indata;
}

/**
 * @brief 8-bits SPI data transfer
 *
 * The function transfers one word (8 bits) to SPI hardware
 *
 * @param info pointer to the data of tsb_spi_info
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_transfer_8(struct tsb_spi_info *info,
                       struct device_spi_transfer *transfer)
{
    int i = 0, j = 0;
    uint8_t *txbuf = NULL, *rxbuf = NULL;
    uint8_t dataout = 0, datain = 0, tmp = 0;
    uint32_t holdtime = 0;
    bool loopback = false;

    txbuf = transfer->txbuffer;
    rxbuf = transfer->rxbuffer;
    holdtime = info->holdtime;
    loopback = (info->modes & SPI_MODE_LOOP)? true : false;

    for (i = 0; i < transfer->nwords; i++) {
        datain = 0;
        /* if user doesn't provide output data, driver will send 0 by default */
        dataout = (transfer->txbuffer)? txbuf[i] : 0;
        if (info->modes & SPI_MODE_LSB_FIRST) {
            /* LSB first out and MSB last */
            for (j = 0; j <info->bpw; j++) {
                tmp = info->bitexchange(dataout & (1 << j), holdtime, loopback);
                datain = datain | (tmp << j);
            }
        } else {
            /* MSB first out and LSB last */
            for (j = info->bpw - 1; j >= 0; j--) {
                tmp = info->bitexchange(dataout & (1 << j), holdtime, loopback);
                datain = datain | (tmp << j);
            }
        }
        if (rxbuf) {
            /* save return data to rxbuf */
            rxbuf[i] = datain;
        }
    }
    return 0;
}

/**
 * @brief 16-bits SPI data transfer
 *
 * The function transfers one word (16 bits) to SPI hardware
 *
 * @param info pointer to the data of tsb_spi_info
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_transfer_16(struct tsb_spi_info *info,
                       struct device_spi_transfer *transfer)
{
    int i = 0, j = 0;
    uint16_t *txbuf = NULL, *rxbuf = NULL;
    uint16_t dataout = 0, datain = 0, tmp = 0;
    uint32_t holdtime = 0;
    bool loopback = false;

    txbuf = transfer->txbuffer;
    rxbuf = transfer->rxbuffer;
    holdtime = info->holdtime;
    loopback = (info->modes & SPI_MODE_LOOP)? true : false;

    for (i = 0; i < transfer->nwords; i++) {
        /* if user doesn't provide output data, driver will send 0 by default */
        dataout = (transfer->txbuffer)? txbuf[i] : 0;
        if (info->modes & SPI_MODE_LSB_FIRST) {
            /* LSB first out and MSB last */
            for (j = 0; j < info->bpw; j++) {
                tmp = info->bitexchange(dataout & (1 << j), holdtime, loopback);
                datain = datain | (tmp << j);
            }
        } else {
            /* MSB first out and LSB last */
            for (j = info->bpw - 1; j >= 0; j--) {
                tmp = info->bitexchange(dataout & (1 << j), holdtime, loopback);
                datain = datain | (tmp << j);
            }
        }
        if (rxbuf) {
            /* save return data to rxbuf */
            rxbuf[i] = datain;
        }
    }
    return 0;
}

/**
 * @brief 32-bits SPI data transfer
 *
 * The function transfers one word (32 bits) to SPI hardware
 *
 * @param info pointer to the data of tsb_spi_info
 * @param transfer pointer to the spi transfer request
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_transfer_32(struct tsb_spi_info *info,
                       struct device_spi_transfer *transfer)
{
    int i = 0, j = 0;
    uint32_t *txbuf = NULL, *rxbuf = NULL;
    uint32_t dataout = 0, datain = 0, tmp = 0;
    uint32_t holdtime = 0;
    bool loopback = false;

    txbuf = transfer->txbuffer;
    rxbuf = transfer->rxbuffer;
    holdtime = info->holdtime;
    loopback = (info->modes & SPI_MODE_LOOP)? true : false;

    for (i = 0; i < transfer->nwords; i++) {
        /* if user doesn't provide output data, driver will send 0 by default */
        dataout = (transfer->txbuffer)? txbuf[i] : 0;
        if (info->modes & SPI_MODE_LSB_FIRST) {
            /* LSB first out and MSB last */
            for (j = 0; j < info->bpw; j++) {
                tmp = info->bitexchange(dataout & (1 << j), holdtime, loopback);
                datain = datain | (tmp << j);
            }
        } else {
            /* MSB first out and LSB last */
            for (j = info->bpw - 1; j >= 0; j--) {
                tmp = info->bitexchange(dataout & (1 << j), holdtime, loopback);
                datain = datain | (tmp << j);
            }
        }
        if (rxbuf) {
            /* save return data to rxbuf */
            rxbuf[i] = datain;
        }
    }
    return 0;
}

/**
 * @brief Lock SPI bus for exclusive access
 *
 * On SPI buses where there are multiple devices, it will be necessary to lock
 * SPI to have exclusive access to the buses for a sequence of transfers.
 * The bus should be locked before the chip is selected. After locking the SPI
 * bus, the caller should then also call the setfrequency(), setbits(), and
 * setmode() methods to make sure that the SPI is properly configured for the
 * device. If the SPI bus is being shared, then it may have been left in an
 * incompatible state.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_lock(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

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
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state == TSB_SPI_STATE_LOCKED) {
        info->state = TSB_SPI_STATE_OPEN;
    } else {
        return -EIO;
    }
    sem_post(&info->bus);

    return 0;
}

/**
 * @brief Enable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock();
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_select(struct device *dev, int devid)
{
    struct tsb_spi_info *info = NULL;
    int i = 0, ret = 0;
    bool selected = true;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_unlock;
    }

    if (info->modes & SPI_MODE_NO_CS) {
        ret = -EINVAL;
        goto err_unlock;
    }

    if (devid > info->caps.csnum) {
        ret = -EINVAL;
        goto err_unlock;
    }

    info->selected = devid;

    if (!(info->modes & SPI_MODE_CS_HIGH)) {
        selected = false;
    }

    for (i = 0; i < info->caps.csnum; i++) {
        /* only selected pin can be activated */
        gpio_set_value(info->chipselect[i], (i == devid)? selected : !selected);
    }
err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Disable the SPI chip select pin
 *
 * The implementation of this method must include handshaking. If a device is
 * selected, it must hold off all the other attempts to select the device
 * until the device is deselected. This function should be called after lock();
 * if the driver isn’t in lock state, it returns an error code to notify a
 * problem.
 *
 * @param dev pointer to structure of device data
 * @param devid identifier of a selected SPI slave device
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_deselect(struct device *dev, int devid)
{
    struct tsb_spi_info *info = NULL;
    int i = 0, ret = 0;
    bool selected = false;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_unlock;
    }

    if (info->modes & SPI_MODE_NO_CS) {
        ret = -EINVAL;
        goto err_unlock;
    }

    if (devid > info->caps.csnum) {
        ret = -EINVAL;
        goto err_unlock;
    }

    info->selected = -1;

    if (!(info->modes & SPI_MODE_CS_HIGH)) {
        selected = true;
    }

    for (i = 0; i < info->caps.csnum; i++) {
        /* deactivate all chip-select pins */
        gpio_set_value(info->chipselect[i], selected);
    }
err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Configure SPI clock
 *
 * If SPI hardware doesn’t support this frequency value, this function should
 * find the nearest lower frequency that is hardware supported and then
 * configure SPI clock to this value. It will return the actual frequency
 * value back to the caller via parameter frequency.
 * This function should be called after lock(); if the driver is not in lock
 * state, it returns an error code to notify a problem.
 *
 * @param dev pointer to structure of device data
 * @param frequency SPI frequency requested (unit: Hz)
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setfrequency(struct device *dev, uint32_t *frequency)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !frequency) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != TSB_SPI_STATE_LOCKED)
        return -EPERM;

    info->frequency = *frequency;

    /* We support SPI frequency range from 1kHz to 500kHz */
    if (info->frequency <= 1000) {
        info->frequency = 1000;
    }

    if (info->frequency >= 500000) {
        info->frequency = 500000;
    }

    /* calc holdtime */
    info->holdtime = 500000 / info->frequency;
    if (info->holdtime > XFER_OVERHEAD) {
        info->holdtime = info->holdtime - XFER_OVERHEAD;
    } else {
        info->holdtime = 0;
    }

    *frequency = info->frequency;
    return 0;
}

/**
 * @brief Configure SPI mode
 *
 * The mode parameter is used to configure the clock polarity and phase of the
 * SPI master. This function will accept a value for one of the standard SPI
 * modes. If the value of the mode parameter is out of bounds or the requested
 * SPI mode is not supported by this hardware, an -ENOSYS error will be
 * returned. This function should be called after lock(); if the driver is not
 * in a locked state, this function will return -EPERM.
 *
 * @param dev pointer to structure of device data
 * @param mode SPI protocol mode requested
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setmode(struct device *dev, uint16_t mode)
{
    struct tsb_spi_info *info = NULL;
    int i = 0, value = 0, ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != TSB_SPI_STATE_LOCKED)
        return -EPERM;

    /* check hardware mode capabilities */
    if (mode & ~info->caps.modes)
        return -ENOSYS;

    info->modes = mode;
    switch (mode & (SPI_MODE_CPHA | SPI_MODE_CPOL)) {
        case SPI_MODE_0:
            info->bitexchange = tsb_spi_bitexchange0;
            break;
        case SPI_MODE_1:
            info->bitexchange = tsb_spi_bitexchange1;
            break;
        case SPI_MODE_2:
            info->bitexchange = tsb_spi_bitexchange2;
            break;
        case SPI_MODE_3:
            info->bitexchange = tsb_spi_bitexchange3;
            break;
    }
    /* After changed SPI mode, we need to change the SCK default output level */
    gpio_set_value(SPI_SCK, (mode & SPI_MODE_CPOL)? 1 : 0);

    /* After changed SPI mode, we need to change the CS default output level */
    for (i = 0; i < info->caps.csnum; i++) {
        if (mode & SPI_MODE_CS_HIGH) {
            /* if not selected, set default CS output level to low */
            value = (i == info->selected)? 1 : 0;
        } else {
            /* if not selected, set default CS output level to high */
            value = (i == info->selected)? 0 : 1;
        }
        gpio_set_value(info->chipselect[i], value);
    }

    return ret;
}

/**
 * @brief Set the number of bits per word in transmission
 *
 * This function should be called after lock(); if driver is not in lock state,
 * this function returns -EPERM error code.
 *
 * @param dev pointer to structure of device data
 * @param nbits The number of bits requested. The nbits value range is from
 *        1 to 32. The generic nbits value is 8, 16, 32, but the value still
 *        depends on hardware supported.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_setbits(struct device *dev, int nbits)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (info->state != TSB_SPI_STATE_LOCKED)
        return -EPERM;

    /* check hardware bpw capabilities */
    if ((BIT(nbits - 1) & info->caps.bpw) == 0)
        return -ENOSYS;

    info->bpw = nbits;
    return ret;
}

/**
 * @brief Exchange a block of data from SPI
 *
 * Device driver uses this function to transfer and receive data from SPI bus.
 * This function should be called after lock(); if the driver is not in lock
 * state, it returns -EPERM error code.
 * The transfer structure consists of the read/write buffer, transfer length,
 * transfer flags and callback function.
 *
 * @param dev pointer to structure of device data
 * @param transfer pointer to the spi transfer request
 * @param devid the specific chip number
 * @param config pointer to the device_spi_device_config structure to set
 * the configuration for the chip. If config is NULL, the configuration
 * associated with devid will be used.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_exchange(struct device *dev,
                            struct device_spi_transfer *transfer,
                            uint8_t devid,
                            struct device_spi_device_config *config)
{
    struct tsb_spi_info *info = NULL;
    uint8_t bpw;
    uint16_t mode;
    uint32_t frequency;
    int ret = 0;

    /* check input parameters */
    if (!dev || !device_get_private(dev) || !transfer) {
        return -EINVAL;
    }

    /* check transfer buffer */
    if(!transfer->txbuffer && !transfer->rxbuffer) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    if (devid >= info->num_boards)
        return -EINVAL;

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_LOCKED) {
        ret = -EPERM;
        goto err_unlock;
    }

    if (config == NULL) {
        /* get master config values from devid */
        struct device *slave_dev = info->dev_spi_board[devid];
        device_spi_board_get_max_speed_hz(slave_dev, &frequency);
        device_spi_board_get_bpw(slave_dev, &bpw);
        device_spi_board_get_mode(slave_dev, &mode);
    } else {
        /* get master config values from provided config parameter */
        frequency = config->max_speed_hz;
        bpw = config->bpw;
        mode = config->mode;
    }

    /* set master frequency */
    ret = tsb_spi_setfrequency(dev, devid, &frequency);
    if (ret)
        goto err_unlock;

    /* set master bits per word */
    ret = tsb_spi_setbpw(dev, devid, bpw);
    if (ret)
        goto err_unlock;

    /* set master mode */
    ret = tsb_spi_setmode(dev, devid, mode);
    if (ret)
        goto err_unlock;

    if (info->bpw <= 8) {
        tsb_spi_transfer_8(info, transfer);
    } else if (info->bpw <= 16) {
        tsb_spi_transfer_16(info, transfer);
    } else if (info->bpw <= 32) {
        tsb_spi_transfer_32(info, transfer);
    } else {
        ret = -EINVAL;
    }

err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get SPI device driver hardware capabilities information
 *
 * This function can be called whether lock() has been called or not.
 *
 * @param dev pointer to structure of device data
 * @param caps pointer to the spi_caps structure to receive the capabilities
 *             information.
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_getcaps(struct device *dev, struct device_spi_caps *caps)
{
    struct tsb_spi_info *info = NULL;
    /* check input parameters */
    if (!dev || !device_get_private(dev) || !caps) {
        return -EINVAL;
    }

    info = device_get_private(dev);

    sem_wait(&info->lock);
    memcpy(caps, &info->caps, sizeof(struct device_spi_caps));
    sem_post(&info->lock);

    return 0;
}

/**
 * @brief Open SPI device
 *
 * This function is called when the caller is preparing to use this device
 * driver. It should be called after probe() function and needs to
 * check whether the driver is already open. If driver was opened, it needs
 * to return an error code to the caller.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_open(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    int ret = 0;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    sem_wait(&info->lock);

    if (info->state != TSB_SPI_STATE_CLOSED) {
        ret = -EBUSY;
        goto err_unlock;
    }
    info->state = TSB_SPI_STATE_OPEN;

err_unlock:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close SPI device
 *
 * This function is called when the caller is no longer using this driver. It
 * should release or close all resources that were allocated by the open()
 * function. This function should be called after the open() function. If the
 * device is not opened yet, this function should return without any operations.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_close(struct device *dev)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_CLOSED;
}

/**
 * @brief Probe SPI device
 *
 * This function is called by the system to register the driver when the system
 * boots up. This function allocates memory for the private SPI device
 * information, and then sets up the hardware resource and interrupt handler.
 *
 * @param dev pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int tsb_spi_dev_probe(struct device *dev)
{
    struct tsb_spi_info *info = NULL;
    struct device_spi_caps *caps = NULL;
    int ret = 0;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    info->state = TSB_SPI_STATE_CLOSED;
    device_set_private(dev, info);

    /* set up spi hardware capabilities */
    caps = &info->caps;
    caps->modes = SPI_MODE_CPHA |
                  SPI_MODE_CPOL |
                  SPI_MODE_CS_HIGH |
                  SPI_MODE_LSB_FIRST |
                  SPI_MODE_LOOP;
    caps->flags = 0; /* Full Duplex */
    caps->bpw = BIT(8-1) | BIT(16-1) | BIT(32-1);
    caps->csnum = 1; /* support 1 chipselect */

    info->chipselect[0] = SPI_CS;
    info->selected = -1; /* select none */
    info->bitexchange = tsb_spi_bitexchange0;

    /* initialize gpio pins */
    tsb_spi_hw_init(dev);

    sem_init(&info->bus, 0, 1);
    sem_init(&info->lock, 0, 1);

    return ret;
}

/**
 * @brief Remove SPI device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that was allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev pointer to structure of device data
 */
static void tsb_spi_dev_remove(struct device *dev)
{
    struct tsb_spi_info *info = NULL;

    /* check input parameter */
    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    info->state = TSB_SPI_STATE_INVALID;
    sem_destroy(&info->lock);
    sem_destroy(&info->bus);

    /* deinitialize gpio pins */
    tsb_spi_hw_deinit(dev);
    device_set_private(dev, NULL);
    free(info);
}

static struct device_spi_type_ops tsb_spi_type_ops = {
    .lock           = tsb_spi_lock,
    .unlock         = tsb_spi_unlock,
    .select         = tsb_spi_select,
    .deselect       = tsb_spi_deselect,
    .exchange       = tsb_spi_exchange,
    .getcaps        = tsb_spi_getcaps,
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
    .desc       = "TSB SPI BitBang Driver",
    .ops        = &tsb_spi_driver_ops,
};
