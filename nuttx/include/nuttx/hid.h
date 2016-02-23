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

#ifndef __INCLUDE_NUTTX_HID_H
#define __INCLUDE_NUTTX_HID_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/list.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>

#include <nuttx/greybus/types.h>

#include <semaphore.h>

/** HID device type */
#define HID_DEVICE_NAME             "HID device module"
#define HID_DRIVER_DESCRIPTION      "HID device module driver"

/** Number of report types */
#define HID_REPORT_TYPE_NUM         (HID_FEATURE_REPORT + 1)

/**
 * HID report length structure
 */
struct report_len {
    /** input report length */
    uint16_t input;
    /** output report length */
    uint16_t output;
    /** feature report length */
    uint16_t feature;
} __packed;

/**
 * HID Report Size Structure. Type define for a report item size
 * information structure, to retain the size of a device's reports by ID.
 */
struct hid_size_info {
    /** Report ID */
    uint8_t id;

    /**
     * HID Report length array
     *
     * size[0] : Input Report length
     * size[1] : Output Report length
     * size[2] : Feature Report length
     */
    union {
        uint16_t size[HID_REPORT_TYPE_NUM];
        struct report_len len;
    } reports;
};

/**
 * Private information for HID type device
 */
struct hid_info {
    /** Chain to device linking list. */
    struct list_head device_list;

    /** Driver module representation of the device */
    struct device *dev;

    struct hid_vendor_ops *hid_dev_ops;

    /** HID device descriptor */
    struct hid_descriptor *hdesc;

    /** HID report descriptor */
    uint8_t *rdesc;

    /** Number of HID Report structure */
    int num_ids;

    /** Report length of each HID Report */
    struct hid_size_info *sinfo;

    /** HID device driver operation state*/
    int state;

    /** Hid input event callback function */
    hid_event_callback event_callback;

    /** Private data passed to event_callbacks */
    void *event_data;

    /** Exclusive access for operation */
    sem_t lock;
};

extern int hid_device_init(struct device *dev, struct hid_info *dev_info);

/**
 * HID vendor device driver operations
 */
struct hid_vendor_ops {
    /** vendor hw initialize routine */
    int (*hw_initialize)(struct device *dev, struct hid_info *dev_info);
    /** vendor hw deinitialize routine */
    int (*hw_deinitialize)(struct device *dev);
    /** vendor hw power control routine */
    int (*power_control)(struct device *dev, bool on);
    /** Get HID Input / Feature report data */
    int (*get_report)(struct device *dev, enum hid_report_type report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    /** Set HID Output / Feature report data */
    int (*set_report)(struct device *dev, enum hid_report_type report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
};

#endif

