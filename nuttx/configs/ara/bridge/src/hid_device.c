/**
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
 *
 * Author: Philip Yang <philipy@bsquare.com>
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/hid.h>

#include <tsb_scm.h>

#define HID_DEVICE_FLAG_PROBE   BIT(0)
#define HID_DEVICE_FLAG_OPEN    BIT(1)
#define HID_DEVICE_FLAG_POWERON BIT(2)

static struct device *hid_dev;

/**
 * @brief Get HID report length
 *
 * @param dev Pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @return 0 on success, negative errno on error
 */
static int hid_dev_get_report_length(struct device *dev,
                                     enum hid_report_type report_type,
                                     uint8_t report_id)
{
    struct hid_info *info = NULL;
    int ret = 0, i;

    /* check input parameters */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    /* lookup the hid_size_info table to find the report size */
    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].id == report_id) {
            ret = info->sinfo[i].reports.size[report_type];
            break;
        }
    }
    return ret;
}

/**
 * @brief Power-on the HID device.
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int hid_dev_power_on(struct device *dev)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_poweron;
    }

    if (!(info->state & HID_DEVICE_FLAG_POWERON)) {
        if (!info->hid_dev_ops->power_control) {
            ret = -ENOSYS;
            goto err_poweron;
        }

        ret = info->hid_dev_ops->power_control(dev, true);
        if (ret) {
            goto err_poweron;
        }

        info->state |= HID_DEVICE_FLAG_POWERON;
    }

    /** HID greybus kernel design HID can multiple open, so always return
     * 0.
     */

err_poweron:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Power-off the HID device.
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int hid_dev_power_off(struct device *dev)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_poweroff;
    }

    if (info->state & HID_DEVICE_FLAG_POWERON) {
        if (!info->hid_dev_ops->power_control) {
            ret = -ENOSYS;
            goto err_poweroff;
        }

        ret = info->hid_dev_ops->power_control(dev, false);
        /* changed power-on state */
        info->state &= ~HID_DEVICE_FLAG_POWERON;
        /* disable interrupt */

    } else {
        ret = -EIO;
    }

err_poweroff:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID Descriptor
 *
 * @param dev Pointer to structure of device data
 * @param desc Pointer to structure of HID device descriptor
 * @return 0 on success, negative errno on error
 */
static int hid_dev_get_desc(struct device *dev, struct hid_descriptor *desc)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_desc;
    }

    /* get HID device descriptor */
    memcpy(desc, info->hdesc, sizeof(struct hid_descriptor));

err_desc:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get HID Report Descriptor
 *
 * @param dev pointer to structure of device data
 * @param desc pointer to HID report descriptor
 * @return 0 on success, negative errno on error
 */
static int hid_dev_get_report_desc(struct device *dev, uint8_t *desc)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!desc || !dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_report_desc;
    }

    /* get HID report descriptor */
    memcpy(desc, info->rdesc, info->hdesc->report_desc_length);

err_report_desc:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Get maximum report descriptor size
 *
 * Since the descriptor size is flexible for each REPORT type, so to create
 * this function for caller if they need for buffer design.
 *
 * @param dev Pointer to structure of device data
 * @param report_type HID report type
 * @return the report size on success, negative errno on error
 */
static int hid_dev_get_maximum_report_length(struct device *dev,
                                             enum hid_report_type report_type)
{
    struct hid_info *info = NULL;
    int i = 0, maxlen = 0, id = 0;

    /* check input parameters */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    /* lookup the hid_size_info table to find the max report size
     * in specific Report type  */

    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].reports.size[report_type] > maxlen) {
            id = info->sinfo[i].id;
            maxlen = info->sinfo[i].reports.size[report_type];
        }
    }

    /* If the Report ID isn't zero, add an extra 1-byte in return length for
     * Report ID store.
     */
    if (id) {
        maxlen++;
    }

    return maxlen;
}

/**
 * @brief Get HID Input / Feature report data
 *
 * @param dev Pointer to structure of device data
 * @param report_type HID report type
 * @param report_id HID report id
 * @param data Pointer of input buffer size
 * @param len Report buffer size
 * @return 0 on success, negative errno on error
 */
static int hid_dev_get_report(struct device *dev, enum hid_report_type report_type,
                              uint8_t report_id, uint8_t *data,
                              uint16_t len)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!data || !dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_getreport;
    }

    if (!info->hid_dev_ops->get_report) {
         ret = -ENOSYS;
         goto err_getreport;
    }

    if (report_type == HID_INPUT_REPORT || report_type == HID_FEATURE_REPORT) {
        ret = info->hid_dev_ops->get_report(dev, HID_INPUT_REPORT, report_id,
                                            data, len);
    } else {
        ret = -EINVAL;
    }

err_getreport:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Set HID Output / Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @param data - pointer of output buffer size
 * @param len - max output buffer size
 * @return 0 on success, negative errno on error
 */
static int hid_dev_set_report(struct device *dev, enum hid_report_type report_type,
                              uint8_t report_id, uint8_t *data, uint16_t len)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameters */
    if (!data || !dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        ret = -EIO;
        goto err_setreport;
    }

    if (!info->hid_dev_ops->set_report) {
        ret = -ENOSYS;
        goto err_setreport;
    }

    if (report_type == HID_OUTPUT_REPORT ||
        report_type == HID_FEATURE_REPORT) {
        ret = info->hid_dev_ops->set_report(dev, HID_OUTPUT_REPORT, report_id,
                                            data, len);
    } else {
        ret = -EINVAL;
    }

err_setreport:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Register callback for HID Event Report
 *
 * @param dev Pointer to structure of device data
 * @param callback Callback function for event notify
 * @return 0 on success, negative errno on error
 */
static int hid_dev_register_callback(struct device *dev, void *data,
                                     hid_event_callback callback)
{
    struct hid_info *info = NULL;

    /* check input parameters */
    if (!callback || !dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    info->event_callback = callback;
    info->event_data = data;

    return 0;
}

/**
 * @brief Remove HID Event Report of callback register
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int hid_dev_unregister_callback(struct device *dev)
{
    struct hid_info *info = NULL;

    /* check input parameters */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return -EIO;
    }

    info->event_callback = NULL;
    info->event_data = NULL;
    return 0;
}

/**
 * @brief Open HID device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int hid_dev_open(struct device *dev)
{
    struct hid_info *info = NULL;
    int ret = 0;

    /* check input parameter */
    if (!dev || !(info = device_get_private(dev))) {
        return -EINVAL;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_PROBE)) {
        ret = -EIO;
        goto err_open;
    }

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        /* device has been opened, return error */
        ret = -EBUSY;
        goto err_open;
    }

    if (!info->hid_dev_ops->hw_initialize) {
         ret = -ENOSYS;
         goto err_open;
    }

    ret = info->hid_dev_ops->hw_initialize(dev, info);
    if (ret) {
        goto err_open;
    }

    info->event_callback = NULL;
    info->state |= HID_DEVICE_FLAG_OPEN;

err_open:
    sem_post(&info->lock);
    return ret;
}

/**
 * @brief Close HID device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev Pointer to structure of device data
 */
static void hid_dev_close(struct device *dev)
{
    struct hid_info *info = NULL;

    /* check input parameter */
    if (!dev || !(info = device_get_private(dev))) {
        return;
    }

    sem_wait(&info->lock);

    if (!(info->state & HID_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        goto err_close;
    }

    if (info->state & HID_DEVICE_FLAG_POWERON) {
        hid_dev_power_off(dev);
    }

    if (info->hid_dev_ops->hw_deinitialize) {
        info->hid_dev_ops->hw_deinitialize(dev);
    }

    info->event_callback = NULL;

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        /* clear open state */
        info->state &= ~HID_DEVICE_FLAG_OPEN;
    }

err_close:
    sem_post(&info->lock);
}

/**
 * @brief Probe HID device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private HID device
 * information, and then setup the hardware resource and interrupt handler.
 *
 * @param dev Pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int hid_dev_probe(struct device *dev)
{
    struct hid_info *info = NULL;
    int ret = 0;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    ret = hid_device_init(dev, info);
    if (ret) {
        free(info);
        return ret;
    }

    hid_dev = dev;
    info->dev = dev;
    info->state = HID_DEVICE_FLAG_PROBE;
    device_set_private(dev, info);

    info->device_list.prev = &info->device_list;
    info->device_list.next = &info->device_list;

    sem_init(&info->lock, 0, 1);

    return 0;
}

/**
 * @brief Remove HID device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev Pointer to structure of device data
 */
static void hid_dev_remove(struct device *dev)
{
    struct hid_info *info = NULL;

    /* check input parameter */
    if (!dev || !(info = device_get_private(dev))) {
        return;
    }

    if (info->state & HID_DEVICE_FLAG_OPEN) {
        hid_dev_close(dev);
    }

    info->num_ids = 0;
    info->state = 0;
    sem_destroy(&info->lock);

    device_set_private(dev, NULL);
    hid_dev = NULL;
    free(info);
}

static struct device_hid_type_ops hid_dev_type_ops = {
    .power_on = hid_dev_power_on,
    .power_off = hid_dev_power_off,
    .get_descriptor = hid_dev_get_desc,
    .get_report_descriptor = hid_dev_get_report_desc,
    .get_report_length = hid_dev_get_report_length,
    .get_maximum_report_length = hid_dev_get_maximum_report_length,
    .get_report = hid_dev_get_report,
    .set_report = hid_dev_set_report,
    .register_callback = hid_dev_register_callback,
    .unregister_callback = hid_dev_unregister_callback,
};

static struct device_driver_ops hid_dev_driver_ops = {
    .probe          = hid_dev_probe,
    .remove         = hid_dev_remove,
    .open           = hid_dev_open,
    .close          = hid_dev_close,
    .type_ops       = &hid_dev_type_ops,
};

struct device_driver hid_dev_driver = {
    .type           = DEVICE_TYPE_HID_HW,
    .name           = HID_DEVICE_NAME,
    .desc           = HID_DRIVER_DESCRIPTION,
    .ops            = &hid_dev_driver_ops,
};
