/*
 * Copyright (c) 2014-2015 Google Inc.
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

#include <arch/byteorder.h>
#include <nuttx/device.h>
#include <nuttx/device_i2c.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/debug.h>

#include "i2c-gb.h"

/** Greybus I2C internal data structure */
struct gb_i2c_info {
    uint16_t        cport;      /**< assigned CPort number */
    struct device   *dev;       /**< opened device driver handler */
    char            *dev_type;  /**< device type for this device */
    uint16_t        dev_id;     /**< Id for device in device table */
};

/** A instance of struct gb_i2c_info for internal usage */
static struct gb_i2c_info *i2c_info = NULL;

static uint8_t gb_i2c_protocol_version(struct gb_operation *operation)
{
    struct gb_i2c_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_I2C_VERSION_MAJOR;
    response->minor = GB_I2C_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

static uint8_t gb_i2c_protocol_functionality(struct gb_operation *operation)
{
    struct gb_i2c_functionality_rsp *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->functionality = cpu_to_le32(GB_I2C_FUNC_I2C |
                                          GB_I2C_FUNC_SMBUS_READ_BYTE |
                                          GB_I2C_FUNC_SMBUS_WRITE_BYTE |
                                          GB_I2C_FUNC_SMBUS_READ_BYTE_DATA |
                                          GB_I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
                                          GB_I2C_FUNC_SMBUS_READ_WORD_DATA |
                                          GB_I2C_FUNC_SMBUS_WRITE_WORD_DATA |
                                          GB_I2C_FUNC_SMBUS_READ_I2C_BLOCK |
                                          GB_I2C_FUNC_SMBUS_WRITE_I2C_BLOCK);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2c_protocol_timeout(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

static uint8_t gb_i2c_protocol_retries(struct gb_operation *operation)
{
    return GB_OP_SUCCESS;
}

static uint8_t gb_i2c_protocol_transfer(struct gb_operation *operation)
{
    int i, op_count;
    uint32_t size = 0;
    int ret;
    uint8_t *write_data;
    bool read_op;
    int read_count = 0;

    struct device_i2c_request *requests;

    struct gb_i2c_transfer_desc *desc;
    struct gb_i2c_transfer_req *request;
    struct gb_i2c_transfer_rsp *response;
    const size_t req_size = gb_operation_get_request_payload_size(operation);

    if (req_size < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    request = gb_operation_get_request_payload(operation);
    op_count = le16_to_cpu(request->op_count);
    write_data = (uint8_t *)&request->desc[op_count];

    if (req_size < sizeof(*request) + op_count * sizeof(request->desc[0])) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    for (i = 0; i < op_count; i++) {
        desc = &request->desc[i];
        read_op = (le16_to_cpu(desc->flags) & GB_I2C_M_RD) ? true : false;

        if (read_op)
            size += le16_to_cpu(desc->size);
    }

    response = gb_operation_alloc_response(operation, size);
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    requests = malloc(sizeof(struct device_i2c_request) * op_count);
    if (!requests) {
        return GB_OP_NO_MEMORY;
    }

    for (i = 0; i < op_count; i++) {
        desc = &request->desc[i];
        read_op = (le16_to_cpu(desc->flags) & GB_I2C_M_RD) ? true : false;

        requests[i].flags = 0;
        requests[i].addr = le16_to_cpu(desc->addr);
        requests[i].length = le16_to_cpu(desc->size);

        if (read_op) {
            requests[i].flags |= GB_I2C_M_RD;
            requests[i].buffer = &response->data[read_count];
            read_count += le16_to_cpu(desc->size);
        } else {
            requests[i].buffer = write_data;
            write_data += le16_to_cpu(desc->size);
        }
    }

    ret = device_i2c_transfer(i2c_info->dev, requests, op_count);

    free(requests);

    return gb_errno_to_op_result(ret);
}

static int gb_i2c_init(unsigned int cport, struct gb_bundle *bundle)
{
    i2c_info = zalloc(sizeof(*i2c_info));
    if (!i2c_info) {
        return -ENOMEM;
    }

    i2c_info->cport = cport;
    i2c_info->dev_type = DEVICE_TYPE_I2C_HW;
    i2c_info->dev_id = 0;

    i2c_info->dev = device_open(i2c_info->dev_type, i2c_info->dev_id);
    if (!i2c_info->dev) {
        free(i2c_info);
        gb_info("%s(): failed to open device!\n", __func__);
        return -EIO;
    }

    return 0;
}

static void gb_i2c_exit(unsigned int cport, struct gb_bundle *bundle)
{
    if (i2c_info->dev) {
        device_close(i2c_info->dev);
    }

    free(i2c_info);
    i2c_info = NULL;
}

static struct gb_operation_handler gb_i2c_handlers[] = {
    GB_HANDLER(GB_I2C_PROTOCOL_VERSION, gb_i2c_protocol_version),
    GB_HANDLER(GB_I2C_PROTOCOL_FUNCTIONALITY, gb_i2c_protocol_functionality),
    GB_HANDLER(GB_I2C_PROTOCOL_TIMEOUT, gb_i2c_protocol_timeout),
    GB_HANDLER(GB_I2C_PROTOCOL_RETRIES, gb_i2c_protocol_retries),
    GB_HANDLER(GB_I2C_PROTOCOL_TRANSFER, gb_i2c_protocol_transfer),
};

static struct gb_driver gb_i2c_driver = {
    .init = gb_i2c_init,
    .exit = gb_i2c_exit,
    .op_handlers = gb_i2c_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_i2c_handlers),
};

void gb_i2c_register(int cport, int bundle)
{
    gb_register_driver(cport, bundle, &gb_i2c_driver);
}

