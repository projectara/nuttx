/*
 * Copyright (c) 2015-2016 Google, Inc.
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

#ifndef __INCLUDE_NUTTX_DEVICE_UART_H
#define __INCLUDE_NUTTX_DEVICE_UART_H

#include <errno.h>
#include <stdint.h>

#include <nuttx/device.h>
#include <nuttx/util.h>

/** UART Device type */
#define DEVICE_TYPE_UART_HW                     "UART"

/** UART Parity setting */
enum uart_parity {
    /** No parity */
    NO_PARITY,
    /** Odd parity */
    ODD_PARITY,
    /** Even parity */
    EVEN_PARITY,
    /** Mark parity */
    MARK_PARITY,
    /** Space parity */
    SPACE_PARITY,
};

/** UART Stop bit setting */
enum uart_stopbit {
    /** One stop bit */
    ONE_STOP_BIT,
    /** One and a half stop bit */
    ONE5_STOP_BITS,
    /** two stop bit */
    TWO_STOP_BITS,
};

/** @defgroup UART_MCR Modem control register
 * @{
 */
/** Data terminal ready */
#define MCR_DTR         BIT(0)
/** Request to send */
#define MCR_RTS         BIT(1)
/** Auxiliary user-defined output 1 */
#define MCR_OUT1        BIT(2)
/** Auxiliary user-defined output 2 */
#define MCR_OUT2        BIT(3)
/** Loopback mode select */
#define MCR_LPBK        BIT(4)
/* @} */

/** @defgroup UART_LSR Line status register
 * @{
 */
/** Data Ready */
#define LSR_DR          BIT(0)
/** Overrun Error */
#define LSR_OE          BIT(1)
/** Parity Error */
#define LSR_PE          BIT(2)
/** Framing Error */
#define LSR_FE          BIT(3)
/** Break Interrupt */
#define LSR_BI          BIT(4)
/** Transmitter Holding Register */
#define LSR_THRE        BIT(5)
/** Transmitter Empty */
#define LSR_TEMT        BIT(6)
/** Error in RCVR FIFO */
#define LSR_RXFE        BIT(7)
/* @} */

/** @defgroup UART_MSR Modem status register
 * @{
 */
/** Delta Clear to Send */
#define MSR_DCTS        BIT(0)
/** Delta Data Set Ready */
#define MSR_DDSR        BIT(1)
/** Trailing Edge Ring Indicator */
#define MSR_TERI        BIT(2)
/** Delta Data Carrier Detect */
#define MSR_DDCD        BIT(3)
/** Clear to Send */
#define MSR_CTS         BIT(4)
/** Data Set Ready */
#define MSR_DSR         BIT(5)
/** Ring Indicator */
#define MSR_RI          BIT(6)
/** Data Carrier Detect */
#define MSR_DCD         BIT(7)
/** @} */

/**
 * @brief UART Status callback function
 *
 * This callback function is called upon modification of a Status Register.
 *
 * @param data Private data passed to the callback
 * @param status The status register's content
 */
typedef void (*uart_status_callback)(void *data, uint8_t status);

/**
 * @brief UART transmit and receive callback
 *
 * This callback function is called upon the completion of a transmit or receive
 * operation.
 *
 * @param dev Pointer to the UART device controller
 * @param data Private data passed to the callback
 * @param buffer Buffer of data (to receive or to transmit)
 * @param length Amount of received/sent data
 * @param error Error code upon completion
 */
typedef void (*uart_xfer_callback)(struct device *dev, void *data,
                                   uint8_t *buffer, int length, int error);

/** UART device driver operations  */
struct device_uart_type_ops {
    /** Configures a UART device
     * @param dev Pointer to the UART device controller to configure
     * @param baud The baud rate setting
     * @param parity The parity setting
     * @param databits The number of data bits (between 5 and 8)
     * @param stopbit The stop setting
     * @param flow 0 for disabling flow control, 1 for enabling
     * @return 0 on success, negative errno on failure
     */
    int (*set_configuration)(struct device *dev, unsigned int baud,
                             enum uart_parity parity, int databits,
                             enum uart_stopbit stopbit, int flow);
    /** Get the Modem Control register of a UART device controller
     * @param dev Pointer to the UART device controller whose MCR to return
     * @param modem_ctrl The Modem Control's content to fill out \ref UART_MCR
     * @return 0 on success, negative errno on failure
     */
    int (*get_modem_ctrl)(struct device *dev, uint8_t *modem_ctrl);
    /** Set the Modem Control register of a UART device
     * @param dev Pointer to the UART device controller whose MCR to set
     * @param modem_ctrl The Modem Control's content to read from \ref UART_MCR
     * @return 0 on success, negative errno on failure
     */
    int (*set_modem_ctrl)(struct device *dev, uint8_t *modem_ctrl);
    /** Get the Modem Status register of a UART device
     * @param dev Pointer to the UART device controller whose MSR to return
     * @param modem_ctrl The Modem Status's content to fill out \ref UART_MSR
     * @return 0 on success, negative errno on failure
     */
    int (*get_modem_status)(struct device *dev, uint8_t *modem_status);
    /** Get the Line Status register of a UART device
     * @param dev Pointer to the UART device controller whose LSR to return
     * @param modem_ctrl The Modem Line's content to fill out \ref UART_MSR
     * @return 0 on success, negative errno on failure
     */
    int (*get_line_status)(struct device *dev, uint8_t *line_status);
    /** Setup break condition on the transmit line
     * @param dev Pointer to the UART device controller
     * @param break_on 1 for creating break conditions, 0 otherwise
     * @return 0 on success, negative errno on failure
     */
    int (*set_break)(struct device *dev, uint8_t break_on);
    /** Register a callback on Modem Status
     * @param dev Pointer to the UART device controller
     * @param callback The callback function to be called on an event
     * @param data Private data to be passed to the callback
     * @return 0 on success, negative errno on failure
     */
    int (*attach_ms_callback)(struct device *dev,
                              uart_status_callback callback,
                              void *data);
    /** Register a callback on Line Status
     * @param dev Pointer to the UART device controller
     * @param callback The callback function to be called on an event
     * @param data Private data to be passed to the callback
     * @return 0 on success, negative errno on failure
     */
    int (*attach_ls_callback)(struct device *dev,
                              uart_status_callback callback,
                              void *data);
    /** Transmit data through the UART controller
     * @param dev Pointer to the UART device controller
     * @param buffer Buffer of data to transmit
     * @param length Length of buffer in bytes
     * @param dma DMA handle
     * @param sent If not NULL, receives the amount of transmitted data
     * @param callback If not NULL, the function returns and the callback is
     * called upon termination of the transmitting operation. Otherwise the
     * function does not return until the transmitting operation is complete.
     * @return 0 on success, negative errno on failure
     */
    int (*start_transmitter)(struct device *dev, uint8_t *buffer, int length,
                             void *dma, int *sent,
                             uart_xfer_callback callback);
    /** Stop the current transmitting operation
     * @param dev Pointer to the UART device controller
     * @return 0 on success, negative errno on failure
     */
    int (*stop_transmitter)(struct device *dev);
    /** Receive data through the UART controller
     * @param dev Pointer to the UART device controller
     * @param buffer Buffer to data to receive
     * @param length Length of buffer in bytes
     * @param dma DMA handle
     * @param got If not NULL, receives the amount of received data
     * @param callback If not NULL, the function returns and the callback is
     * called upon termination of the receiving operation. Otherwise the
     * function does not return until the receiving operation is complete.
     * @return 0 on success, negative errno on failure
     */
    int (*start_receiver)(struct device *dev, uint8_t*buffer, int length,
                          void *dma, int *got,
                          uart_xfer_callback callback);
    /** Stop the current receiving operation
     * @param dev Pointer to the UART device controller
     * @return 0 on success, negative errno on failure
     */
    int (*stop_receiver)(struct device *dev);
};

/** Configures a UART device
 * @param dev Pointer to the UART device controller to configure
 * @param baud The baud rate setting
 * @param parity The parity setting
 * @param databits The number of data bits (between 5 and 8)
 * @param stopbit The stop setting
 * @param flow 0 for disabling flow control, 1 for enabling
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_set_configuration(struct device *dev,
                                                unsigned int baud,
                                                enum uart_parity parity,
                                                int databits,
                                                enum uart_stopbit stopbit,
                                                int flow)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->set_configuration)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->set_configuration(dev, baud,
                                                                   parity,
                                                                   databits,
                                                                   stopbit,
                                                                   flow);

    return -ENOSYS;
}

/** Get the Modem Control register of a UART device controller
 * @param dev Pointer to the UART device controller whose MCR to return
 * @param modem_ctrl The Modem Control's content to fill out \ref UART_MCR
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_get_modem_ctrl(struct device *dev,
                                             uint8_t *modem_ctrl)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_ctrl)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_ctrl(dev,
                                                                modem_ctrl);

    return -ENOSYS;
}

/** Set the Modem Control register of a UART device
 * @param dev Pointer to the UART device controller whose MCR to set
 * @param modem_ctrl The Modem Control's content to read from \ref UART_MCR
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_set_modem_ctrl(struct device *dev,
                                             uint8_t *modem_ctrl)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->set_modem_ctrl)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->set_modem_ctrl(dev,
                                                                modem_ctrl);

    return -ENOSYS;
}

/** Get the Modem Status register of a UART device
 * @param dev Pointer to the UART device controller whose MSR to return
 * @param modem_status The Modem Status's content to fill out \ref UART_MSR
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_get_modem_status(struct device *dev,
                                               uint8_t *modem_status)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_status)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->get_modem_status(dev,
                                                                  modem_status);

    return -ENOSYS;
}

/** Get the Line Status register of a UART device
 * @param dev Pointer to the UART device controller whose LSR to return
 * @param line_status The Modem Line's content to fill out \ref UART_MSR
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_get_line_status(struct device *dev,
                                              uint8_t *line_status)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->get_line_status)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->get_line_status(dev,
                                                                 line_status);

    return -ENOSYS;
}

/** Setup break condition on the transmit line
 * @param dev Pointer to the UART device controller
 * @param break_on 1 for creating break conditions, 0 otherwise
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_set_break(struct device *dev, uint8_t break_on)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->set_break)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->set_break(dev, break_on);

    return -ENOSYS;
}

/** Register a callback on Modem Status
 * @param dev Pointer to the UART device controller
 * @param callback The callback function to be called on an event
 * @param data Private data to be passed to the callback.
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_attach_ms_callback(struct device *dev,
                                                 uart_status_callback callback,
                                                 void *data)

{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ms_callback)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ms_callback(dev,
                                                                    callback,
                                                                    data);

    return -ENOSYS;
}

/** Register a callback on Line Status
 * @param dev Pointer to the UART device controller
 * @param callback The callback function to be called on an event
 * @param data Private data to be passed to the callback.
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_attach_ls_callback(struct device *dev,
                                                 uart_status_callback callback,
                                                 void *data)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ls_callback)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->attach_ls_callback(dev,
                                                                    callback,
                                                                    data);

    return -ENOSYS;
}


/** Transmit data through the UART controller
 * @param dev Pointer to the UART device controller
 * @param buffer Buffer of data to transmit
 * @param length Length of buffer in bytes
 * @param dma DMA handle
 * @param sent If not NULL, receives the amount of transmitted data
 * @param callback If not NULL, the function returns and the callback is called
 * upon termination of the transmitting operation. Otherwise the function does
 * not return until the transmitting operation is complete.
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_start_transmitter(struct device *dev,
                                                uint8_t *buffer, int length,
                                                void *dma, int *sent,
                                                uart_xfer_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->start_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->start_transmitter(dev, buffer,
                                                                   length, dma,
                                                                   sent,
                                                                   callback);

    return -ENOSYS;
}

/** Stop the current transmitting operation
 * @param dev Pointer to the UART device controller
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_stop_transmitter(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->stop_transmitter)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->stop_transmitter(dev);

    return -ENOSYS;
}

/** Receive data through the UART controller
 * @param dev Pointer to the UART device controller
 * @param buffer Buffer to data to receive
 * @param length Length of buffer in bytes
 * @param dma DMA handle
 * @param got If not NULL, receives the amount of received data
 * @param callback If not NULL, the function returns and the callback is called
 * upon termination of the receiving operation. Otherwise the function does not
 * return until the receiving operation is complete.
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_start_receiver(struct device *dev,
                                             uint8_t* buffer,
                                             int length, void *dma, int *got,
                                             uart_xfer_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->start_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->start_receiver(dev, buffer,
                                                                length, dma,
                                                                got, callback);

    return -ENOSYS;
}

/** Stop the current receiving operation
 * @param dev Pointer to the UART device controller
 * @return 0 on success, negative errno on failure
 */
static inline int device_uart_stop_receiver(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, uart)->stop_receiver)
        return DEVICE_DRIVER_GET_OPS(dev, uart)->stop_receiver(dev);

    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_UART_H */
