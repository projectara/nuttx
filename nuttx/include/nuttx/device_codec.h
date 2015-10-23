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

#ifndef __ARCH_ARM_DEVICE_CODEC_H
#define __ARCH_ARM_DEVICE_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include "audio.h"

#define DEVICE_TYPE_CODEC_HW        "codec"
#define AUDIO_CODEC_NAME_MAX        32

#define DEVICE_CODEC_PROTOCOL_PCM                         BIT(0)
#define DEVICE_CODEC_PROTOCOL_I2S                         BIT(1)
#define DEVICE_CODEC_PROTOCOL_LR_STEREO                   BIT(2)

#define DEVICE_CODEC_ROLE_MASTER                          BIT(0)
#define DEVICE_CODEC_ROLE_SLAVE                           BIT(1)

#define DEVICE_CODEC_POLARITY_NORMAL                      BIT(0)
#define DEVICE_CODEC_POLARITY_REVERSED                    BIT(1)

#define DEVICE_CODEC_EDGE_RISING                          BIT(0)
#define DEVICE_CODEC_EDGE_FALLING                         BIT(1)

struct device_codec_dai_config {
    uint32_t    format;   /* same as GB_AUDIO */
    uint32_t    rate;     /* same as GB_AUDIO */
    uint8_t     channels; /* same as GB_AUDIO */
    uint8_t     sig_bits; /* same as GB_AUDIO - may be able to remove */
    uint32_t    protocol; /* low-level protocol defining WCLK, offset, etc. */
    uint8_t     mclk_role;
    uint8_t     bclk_role;
    uint8_t     wclk_role;
    uint8_t     wclk_polarity;
    uint8_t     wclk_change_edge;
    uint8_t     data_rx_edge;
    uint8_t     data_tx_edge;
};

struct bitctl {
    uint32_t reg;
    uint32_t reg2;
    uint32_t shift;
    uint32_t shift2;
    uint32_t inv;
    uint32_t max;
    uint32_t min;
};

typedef int (*device_codec_event_callback)(struct device *dev, uint32_t event);

struct device_codec_type_ops {
    int (*get_topology_size)(struct device *dev, uint16_t *size);
    int (*get_topology)(struct device *dev, struct gb_audio_topology *topology);

    int (*get_dai_config)(struct device *dev,
                           struct device_codec_dai_config *dai_config);
    int (*set_dai_config)(struct device *dev,
                           struct device_codec_dai_config *dai_config);

    int (*get_control)(struct device *dev, uint8_t control_id, uint32_t *value);
    int (*set_control)(struct device *dev, uint8_t control_id, uint32_t value);

    int (*enable_widget)(struct device *dev, uint8_t widget_id);
    int (*disable_widget)(struct device *dev, uint8_t widget_id);

    int (*get_tx_delay)(struct device *dev, uint32_t *delay);

    int (*start_tx)(struct device *dev);
    int (*stop_tx)(struct device *dev);

    int (*register_tx_callback)(struct device *dev,
                                device_codec_event_callback *callback);

    int (*get_rx_delay)(struct device *dev, uint32_t *delay);

    int (*start_rx)(struct device *dev);
    int (*stop_rx)(struct device *dev);

    int (*register_rx_callback)(struct device *dev,
                                device_codec_event_callback *callback);
};

static inline int device_codec_get_topology_size(struct device *dev,
                                                 uint16_t *size)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->get_topology_size) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->get_topology_size(dev, size);
    }
    return -ENOSYS;
}

static inline int device_codec_get_topology(struct device *dev,
                                            struct gb_audio_topology *topology)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->get_topology) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->get_topology(dev, topology);
    }
    return -ENOSYS;
}

static inline int device_codec_get_dai_config(struct device *dev,
                                              struct device_codec_dai_config
                                                                    *dai_config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->get_dai_config) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->get_dai_config(dev, dai_config);
    }
    return -ENOSYS;
}

static inline int device_codec_set_dai_config(struct device *dev,
                                              struct device_codec_dai_config
                                                                    *dai_config)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->set_dai_config) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->set_dai_config(dev, dai_config);
    }
    return -ENOSYS;
}

static inline int device_codec_get_control(struct device *dev,
                                           uint8_t control_id, uint32_t *value)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->get_control) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->get_control(dev, control_id,
                                                              value);
    }
    return -ENOSYS;
}

static inline int device_codec_set_control(struct device *dev,
                                           uint8_t control_id, uint32_t value)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->set_control) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->set_control(dev, control_id,
                                                              value);
    }
    return -ENOSYS;
}

static inline int device_codec_enable_widget(struct device *dev,
                                             uint8_t widget_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->enable_widget) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->enable_widget(dev,
                                                                widget_id);
    }
    return -ENOSYS;
}

static inline int device_codec_disable_widget(struct device *dev,
                                              uint8_t widget_id)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->disable_widget) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->disable_widget(dev,
                                                                 widget_id);
    }
    return -ENOSYS;
}

static inline int device_codec_get_tx_delay(struct device *dev, uint32_t *delay)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->get_tx_delay) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->get_tx_delay(dev, delay);
    }
    return -ENOSYS;
}

static inline int device_codec_start_tx(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->start_tx) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->start_tx(dev);
    }
    return -ENOSYS;
}

static inline int device_codec_stop_tx(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->stop_tx) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->stop_tx(dev);
    }
    return -ENOSYS;
}

static inline int device_codec_register_tx_callback(struct device *dev,
                                                    device_codec_event_callback
                                                                      *callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->register_tx_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->register_tx_callback(dev,
                                                                      callback);
    }
    return -ENOSYS;
}

static inline int device_codec_get_rx_delay(struct device *dev, uint32_t *delay)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->get_rx_delay) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->get_rx_delay(dev, delay);
    }
    return -ENOSYS;
}

static inline int device_codec_start_rx(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->start_rx) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->start_rx(dev);
    }
    return -ENOSYS;
}

static inline int device_codec_stop_rx(struct device *dev)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->stop_rx) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->stop_rx(dev);
    }
    return -ENOSYS;
}

static inline int device_codec_register_rx_callback(struct device *dev,
                                                    device_codec_event_callback
                                                                      *callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }
    if (DEVICE_DRIVER_GET_OPS(dev, codec)->register_rx_callback) {
        return DEVICE_DRIVER_GET_OPS(dev, codec)->register_rx_callback(dev,
                                                                      callback);
    }
    return -ENOSYS;
}

#endif /* __ARCH_ARM_DEVICE_CODEC_H */
