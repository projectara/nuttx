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
 */

#ifndef __INCLUDE_NUTTX_DEVICE_AUDIO_BOARD_H
#define __INCLUDE_NUTTX_DEVICE_AUDIO_BOARD_H

#include <stdint.h>
#include <assert.h>

#include <nuttx/device.h>

#define DEVICE_TYPE_AUDIO_BOARD_HW "audio_board"

struct device_audio_board_type_ops {
    int (*get_bundle_count)(struct device *dev, unsigned int *bundle_count);
    int (*get_mgmt_cport)(struct device *dev, unsigned int bundle_idx,
                          uint16_t *mgmt_cport);
    int (*get_codec_dev_id)(struct device *dev, unsigned int bundle_idx,
                            unsigned int *codec_dev_id);
    int (*get_dai_count)(struct device *dev, unsigned int bundle_idx,
                         unsigned int *dai_count);
    int (*get_data_cport)(struct device *dev, unsigned int bundle_idx,
                          unsigned int dai_idx, uint16_t *data_cport);
    int (*get_i2s_dev_id)(struct device *dev, unsigned int bundle_idx,
                          unsigned int dai_idx, unsigned int *i2s_dev_id);
};

/**
 * @brief Get number of bundles on this board
 *
 * @param dev Pointer to structure of device.
 * @param bundle_count Number of audio bundles
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_board_get_bundle_count(struct device *dev,
                                                   unsigned int  *bundle_count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_bundle_count) {
        return DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_bundle_count(dev,
                                                                 bundle_count);
    }

    return -ENOSYS;
}

/**
 * @brief Get Audio Management CPort for specified bundle
 *
 * @param dev Pointer to structure of device.
 * @param bundle_idx Index of the bundle
 * @param mgmt_cport audio management CPort for the bundle
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_board_get_mgmt_cport(struct device *dev,
                                                    unsigned int bundle_idx,
                                                    uint16_t *mgmt_cport)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_mgmt_cport) {
        return DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_mgmt_cport(dev,
                                                                   bundle_idx,
                                                                   mgmt_cport);
    }

    return -ENOSYS;
}

/**
 * @brief Get device ID of codec associated with specified bundle
 *
 * @param dev Pointer to structure of device.
 * @param bundle_idx Index of the bundle
 * @param codec_dev_id Device ID of codec associated with specified bundle
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_board_get_codec_dev_id(struct device *dev,
                                                      unsigned int bundle_idx,
                                                      uint16_t *codec_dev_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_codec_dev_id) {
        return DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_codec_dev_id(dev,
                                                                 bundle_idx,
                                                                 codec_dev_id);
    }

    return -ENOSYS;
}

/**
 * @brief Get number of DAIs associated with specified bundle
 *
 * @param dev Pointer to structure of device.
 * @param bundle_idx Index of the bundle
 * @param dai_count Number of DAIs associated with bundle
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_board_get_dai_count(struct device *dev,
                                                   unsigned int bundle_idx,
                                                   unsigned int *dai_count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_dai_count) {
        return DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_dai_count(dev,
                                                                    bundle_idx,
                                                                    dai_count);
    }

    return -ENOSYS;
}

/**
 * @brief Get Audio Data CPort associated with specified bundle and DAI
 *
 * @param dev Pointer to structure of device.
 * @param bundle_idx Index of the bundle
 * @param dai_idx Index of the DAI associated with data_cport
 * @param data_cport Data CPort associated with specified bundle and DAI
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_board_get_data_cport(struct device *dev,
                                                    unsigned int bundle_idx,
                                                    unsigned int dai_idx,
                                                    uint16_t *data_cport)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_data_cport) {
        return DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_data_cport(dev,
                                                                   bundle_idx,
                                                                   dai_idx,
                                                                   data_cport);
    }

    return -ENOSYS;
}

/**
 * @brief Get device ID of I2S controller associated with bundle & DAI
 *
 * @param dev Pointer to structure of device.
 * @param bundle_idx Index of the bundle
 * @param dai_idx Index of the DAI associated with i2s_dev_id
 * @param i2s_dev_id I2S Device ID associated with specified bundle and DAI
 * @return 0 on success, negative errno on error.
 */
static inline int device_audio_board_get_i2s_dev_id(struct device *dev,
                                                   unsigned int bundle_idx,
                                                    unsigned int dai_idx,
                                                    unsigned int *i2s_dev_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_i2s_dev_id) {
        return DEVICE_DRIVER_GET_OPS(dev, audio_board)->get_i2s_dev_id(dev,
                                                                   bundle_idx,
                                                                   dai_idx,
                                                                   i2s_dev_id);
    }

    return -ENOSYS;
}

#endif  /* __INCLUDE_NUTTX_DEVICE_AUDIO_BOARD_H */
