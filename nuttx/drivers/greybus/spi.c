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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_spi.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/debug.h>
#include <apps/greybus-utils/utils.h>

#include <arch/byteorder.h>

#include "spi-gb.h"

#define GB_SPI_VERSION_MAJOR 0
#define GB_SPI_VERSION_MINOR 1

/**
 * @brief Returns the major and minor Greybus SPI protocol version number
 *        supported by the SPI master
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_version(struct gb_operation *operation)
{
    struct gb_spi_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->major = GB_SPI_VERSION_MAJOR;
    response->minor = GB_SPI_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

/**
 * @brief Returns a set of configuration parameters related to SPI master.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_master_config(struct gb_operation *operation)
{
    struct gb_spi_master_config_response *response;
    struct master_spi_caps caps;
    int ret = 0;

    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    DEBUGASSERT(bundle);

    /* get hardware capabilities */
    ret = device_spi_get_master_caps(bundle->dev, &caps);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->bpw_mask = cpu_to_le32(caps.bpw);
    response->min_speed_hz = cpu_to_le32(caps.min_speed_hz);
    response->max_speed_hz = cpu_to_le32(caps.max_speed_hz);
    response->mode = cpu_to_le16(caps.modes);
    response->flags = cpu_to_le16(caps.flags);
    response->num_chipselect = cpu_to_le16(caps.csnum);

    return GB_OP_SUCCESS;
}

/**
 * @brief Get configuration parameters from chip
 *
 * Returns a set of configuration parameters taht related to SPI device is
 * selected.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_device_config(struct gb_operation *operation)
{
    struct gb_spi_device_config_request *request;
    struct gb_spi_device_config_response *response;
    size_t request_size;
    struct device_spi_cfg dev_cfg;
    uint8_t cs;
    int ret = 0;

    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    DEBUGASSERT(bundle);

    request_size = gb_operation_get_request_payload_size(operation);
    if (request_size < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    cs = request->chip_select;

    /* get selected chip of configuration */
    ret = device_spi_get_device_cfg(bundle->dev, cs, &dev_cfg);
    if (ret) {
        return gb_errno_to_op_result(ret);
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    response->device_type = dev_cfg.device_type;
    response->mode = cpu_to_le16(dev_cfg.mode);
    response->bpw = dev_cfg.bpw;
    response->max_speed_hz = cpu_to_le32(dev_cfg.max_speed_hz);
    memcpy(response->name, &dev_cfg.name, sizeof(dev_cfg.name));

    return GB_OP_SUCCESS;
}

/**
 * @brief Performs a SPI transaction as one or more SPI transfers, defined
 *        in the supplied array.
 *
 * @param operation pointer to structure of Greybus operation message
 * @return GB_OP_SUCCESS on success, error code on failure
 */
static uint8_t gb_spi_protocol_transfer(struct gb_operation *operation)
{
    struct gb_spi_transfer_desc *desc;
    struct gb_spi_transfer_request *request;
    struct gb_spi_transfer_response *response;
    struct device_spi_transfer transfer;
    uint32_t size = 0, freq = 0;
    uint8_t *write_data, *read_buf;
    bool selected = false;
    int i, op_count;
    int ret = 0, errcode = GB_OP_SUCCESS;
    size_t request_size;
    size_t expected_size;

    struct gb_bundle *bundle = gb_operation_get_bundle(operation);
    DEBUGASSERT(bundle);

    request_size = gb_operation_get_request_payload_size(operation);
    if (request_size < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    op_count = le16_to_cpu(request->count);

    expected_size = sizeof(*request) +
                    op_count * sizeof(request->transfers[0]);
    if (request_size < expected_size) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    write_data = (uint8_t *)&request->transfers[op_count];

    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        if (desc->rdwr & SPI_XFER_READ) {
            size += le32_to_cpu(desc->len);
        }
    }

    response = gb_operation_alloc_response(operation, size);
    if (!response) {
        return GB_OP_NO_MEMORY;
    }
    read_buf = response->data;

    /* lock SPI bus */
    ret = device_spi_lock(bundle->dev);
    if (ret) {
        return (ret == -EINVAL)? GB_OP_INVALID : GB_OP_UNKNOWN_ERROR;
    }

    /* set SPI mode */
    ret = device_spi_setmode(bundle->dev, request->chip_select, request->mode);
    if (ret) {
        goto spi_err;
    }

    /* parse all transfer request from AP host side */
    for (i = 0; i < op_count; i++) {
        desc = &request->transfers[i];
        freq = le32_to_cpu(desc->speed_hz);

        /* set SPI bits-per-word */
        ret = device_spi_setbits(bundle->dev, request->chip_select,
                                 desc->bits_per_word);
        if (ret) {
            goto spi_err;
        }

        /* set SPI clock */
        ret = device_spi_setfrequency(bundle->dev, request->chip_select, &freq);
        if (ret) {
            goto spi_err;
        }

        /* assert chip-select pin */
        if (!selected) {
            ret = device_spi_select(bundle->dev, request->chip_select);
            if (ret) {
                goto spi_err;
            }
            selected = true;
        }

        /* setup SPI transfer */
        memset(&transfer, 0, sizeof(struct device_spi_transfer));
        transfer.txbuffer = write_data;
        /* If rdwr without SPI_XFER_READ flag, not need to pass read buffer */
        if (desc->rdwr & SPI_XFER_READ) {
            transfer.rxbuffer = read_buf;
        } else {
            transfer.rxbuffer = NULL;
        }

        transfer.nwords = le32_to_cpu(desc->len);
        transfer.flags = SPI_FLAG_DMA_TRNSFER; // synchronous & DMA transfer

        /* start SPI transfer */
        ret = device_spi_exchange(bundle->dev, &transfer);
        if (ret) {
            goto spi_err;
        }
        /* move to next gb_spi_transfer data buffer */
        write_data += le32_to_cpu(desc->len);

        /* If rdwr without SPI_XFER_READ flag, not need to resize
         * read buffer
         */
        if (desc->rdwr & SPI_XFER_READ) {
            read_buf += le32_to_cpu(desc->len);
        }

        if (le16_to_cpu(desc->delay_usecs) > 0) {
            usleep(le16_to_cpu(desc->delay_usecs));
        }

        /* if cs_change enable, change the chip-select pin signal */
        if (desc->cs_change) {
            /* force deassert chip-select pin */
            ret = device_spi_deselect(bundle->dev, request->chip_select);
            if (ret) {
                goto spi_err;
            }
            selected = false;
        }
    }

spi_err:
    errcode = ret;

    if (selected) {
        /* deassert chip-select pin */
        ret = device_spi_deselect(bundle->dev, request->chip_select);
        if (ret) {
            errcode = ret;
        }
    }

    /* unlock SPI bus*/
    ret = device_spi_unlock(bundle->dev);
    if (ret) {
        errcode = ret;
    }

    if (errcode) {
        /* get error code */
        errcode = (errcode == -EINVAL)? GB_OP_INVALID : GB_OP_UNKNOWN_ERROR;
    }
    return errcode;
}

/**
 * @brief Greybus SPI protocol initialize function
 *
 * @param cport CPort number
 * @param bundle Greybus bundle handle
 * @return 0 on success, negative errno on error
 */
static int gb_spi_init(unsigned int cport, struct gb_bundle *bundle)
{
    bundle->dev = device_open(DEVICE_TYPE_SPI_HW, 0);
    if (!bundle->dev) {
        return -EIO;
    }
    return 0;
}

/**
 * @brief Greybus SPI protocol deinitialize function
 *
 * @param cport CPort number
 * @param bundle Greybus bundle handle
 */
static void gb_spi_exit(unsigned int cport, struct gb_bundle *bundle)
{
    if (bundle->dev) {
        device_close(bundle->dev);
        bundle->dev = NULL;
    }
}

/**
 * @brief Greybus SPI protocol operation handler
 */
static struct gb_operation_handler gb_spi_handlers[] = {
    GB_HANDLER(GB_SPI_PROTOCOL_VERSION, gb_spi_protocol_version),
    GB_HANDLER(GB_SPI_TYPE_MASTER_CONFIG, gb_spi_protocol_master_config),
    GB_HANDLER(GB_SPI_TYPE_DEVICE_CONFIG, gb_spi_protocol_device_config),
    GB_HANDLER(GB_SPI_PROTOCOL_TRANSFER, gb_spi_protocol_transfer),
};

static struct gb_driver gb_spi_driver = {
    .init = gb_spi_init,
    .exit = gb_spi_exit,
    .op_handlers = gb_spi_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_spi_handlers),
};

/**
 * @brief Register Greybus SPI protocol
 *
 * @param cport CPort number
 * @param bundle Bundle number.
 */
void gb_spi_register(int cport, int bundle)
{
    gb_register_driver(cport, bundle, &gb_spi_driver);
}

