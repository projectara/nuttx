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

#ifndef __INCLUDE_NUTTX_DEVICE_HID_H
#define __INCLUDE_NUTTX_DEVICE_HID_H

/**
 * @file nuttx/device_hid.h
 * @brief HID Device driver API
 */

#include <errno.h>
#include <stdint.h>

#include <nuttx/device.h>

/** HID Device type */
#define DEVICE_TYPE_HID_HW          "hid"

/** HID report types */
enum hid_report_type {
    /** HID input report (from HID device to AP) */
    HID_INPUT_REPORT,
    /** HID output report (from AP to HID device) */
    HID_OUTPUT_REPORT,
    /** HID feature report (bidirectional) */
    HID_FEATURE_REPORT,
};

/** HID Descriptor */
struct hid_descriptor {
    /** Length of this descriptor */
    uint8_t length;
    /** Length of the report descriptor */
    uint16_t report_desc_length;
    /** Version of the HID Protocol */
    uint16_t hid_version;
    /** Product identifier of the HID device */
    uint16_t product_id;
    /** Vendor identifier of the HID device */
    uint16_t vendor_id;
    /** Country code of the HID device */
    uint8_t country_code;
};

/**
 * @brief HID event callback function
 *
 * This callback function is registered by the client using the HID device
 * driver and should be called by the HID device driver whenever an event
 * arises.
 *
 * @param dev The device that created the event
 * @param data Private user data passed to the callback
 * @param report_type The HID report's type
 * @param report The HID report's content
 * @param len The length of HID report's content
 * @return 0 on success, !=0 on failure
 */
typedef int (*hid_event_callback)(struct device *dev, void *data,
                                  enum hid_report_type report_type,
                                  uint8_t *report, uint16_t len);

/** HID device driver operations */
struct device_hid_type_ops {
    /** Power on a HID device
     * @param dev The device to power on
     * @return 0 on success, !=0 on failure
     */
    int (*power_on)(struct device *dev);
    /** Power off a HID device
     * @param dev The device to power off
     * @return 0 on success, !=0 on failure
     */
    int (*power_off)(struct device *dev);
    /** Get the HID descriptor of a device
     * @param dev The device whose HID descriptor to get
     * @param desc The HID descriptor structure to fill out
     * @return 0 on success, !=0 on failure
     */
    int (*get_descriptor)(struct device *dev, struct hid_descriptor *desc);
    /** Get the HID report descriptor
     * @param dev The device whose HID report descriptor to get
     * @param desc The HID report descriptor structure to fill out
     * @return 0 on success, !=0 on failure
     */
    int (*get_report_descriptor)(struct device *dev, uint8_t *desc);
    /** Get a HID report's length by its type and identifier
     * @param dev The device whose HID report's length to return
     * @param report_type The HID report's type
     * @param report_d The HID report's identifier
     * @return The requested HID report's length on success, <0 on failure
     */
    int (*get_report_length)(struct device *dev,
                             enum hid_report_type report_type,
                             uint8_t report_id);
    /** Get the maximum HID report length for a certain HID report type
     * @param dev The device whose maximum HID report length to return
     * @param report_type The HID report's type
     * @return The maximum HID report length for the specified HID report type,
     * <0 on failure
     */
    int (*get_maximum_report_length)(struct device *dev,
                                     enum hid_report_type report_type);
    /** Get a HID input or feature report
     * @param dev The device whose HID input/feature report to return
     * @param report_type The HID report's type (input or feature)
     * @param report_id The HID report's identifier
     * @param data The HID report's content to fill out
     * @param len The size of the HID report's content
     * @return 0 on success, !=0 on failure
     */
    int (*get_report)(struct device *dev, enum hid_report_type report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    /** Set a HID output or feature report
     * @param dev The device whose HID output/feature report to set
     * @param report_type The HID report's type (output or feature)
     * @param report_id The HID report's identifier
     * @param data The HID report's content to read from
     * @param len The size of the HID report's content
     * @return 0 on success, !=0 on failure
     */
    int (*set_report)(struct device *dev, enum hid_report_type report_type,
                      uint8_t report_id, uint8_t *data, uint16_t len);
    /** Register a HID event callback function
     * @param dev The device creating the event
     * @param data Private data passed to the callback
     * @param callback The callback function to be called on an event
     * @return 0 on success, !=0 on failure
     */
    int (*register_callback)(struct device *dev, void *data,
                             hid_event_callback callback);
    /**
     * @brief Remove the registered HID event callback function
     * @param dev The device creating the event
     * @return 0 on success, !=0 on failure
     */
    int (*unregister_callback)(struct device *dev);
};

/**
 * @brief Power on a HID device
 * @param dev The device to power on
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_power_on(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->power_on) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->power_on(dev);
}

/**
 * @brief Power off a HID device
 * @param dev The device to power off
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_power_off(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->power_off) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->power_off(dev);
}

/**
 * @brief Get the HID descriptor of a device
 * @param dev The device whose HID descriptor to get
 * @param desc The HID descriptor structure to fill out
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_get_descriptor(struct device *dev,
                                            struct hid_descriptor *desc)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_descriptor) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_descriptor(dev, desc);
}

/**
 * @brief Get a HID report's length by its type and identifier
 * @param dev The device whose HID report's length to return
 * @param report_type The HID report's type
 * @param report_id The HID report's identifier
 * @return The requested HID report's length on success, <0 on failure
 */
static inline int device_hid_get_report_length(struct device *dev,
                                               enum hid_report_type report_type,
                                               uint8_t report_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_length) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_length(dev, report_type,
                                                              report_id);
}

/**
 * @brief Get the maximum HID report length for a certain HID report type
 * @param dev The device whose maximum HID report length to return
 * @param report_type The HID report's type
 * @return The maximum HID report length for the specified HID report type, <0
 * on failure
 */
static inline int device_hid_get_max_report_length(struct device *dev,
                                                   enum hid_report_type report_type)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_maximum_report_length) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_maximum_report_length(dev,
                                                             HID_INPUT_REPORT);
}

/**
 * @brief Get the HID report descriptor
 * @param dev The device whose HID report descriptor to get
 * @param desc The HID report descriptor structure to fill out
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_get_report_descriptor(struct device *dev,
                                                   uint8_t *desc)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_descriptor) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_report_descriptor(dev, desc);
}

/**
 * @brief Get a HID input or feature report
 * @param dev The device whose HID input/feature report to return
 * @param report_type The HID report's type (input or feature)
 * @param report_id The HID report's identifier
 * @param data The HID report's content to fill out
 * @param len The size of the HID report's content
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_get_report(struct device *dev,
                                        enum hid_report_type report_type,
                                        uint8_t report_id,
                                        uint8_t *data, uint32_t len)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->get_report) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->get_report(dev, report_type,
                                                       report_id, data, len);
}

/**
 * @brief Set a HID output or feature report
 * @param dev The device whose HID output/feature report to set
 * @param report_type The HID report's type (output or feature)
 * @param report_id The HID report's identifier
 * @param data The HID report's content to read from
 * @param len The size of the HID report's content
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_set_report(struct device *dev,
                                        enum hid_report_type report_type,
                                        uint8_t report_id,
                                        uint8_t *data, uint32_t len)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->set_report) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->set_report(dev, report_type,
                                                       report_id, data, len);
}

/**
 * @brief Register a HID event callback function
 * @param dev The device creating the event
 * @param data User private data (will be passed to the callback)
 * @param callback The callback function to be called on an event
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_register_callback(struct device *dev, void *data,
                                               hid_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->register_callback(dev, data,
                                                              callback);
}

/**
 * @brief Remove the registered HID event callback function
 * @param dev The device creating the event
 * @return 0 on success, !=0 on failure
 */
static inline int device_hid_unregister_callback(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, hid)->unregister_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, hid)->unregister_callback(dev);
}

#endif
