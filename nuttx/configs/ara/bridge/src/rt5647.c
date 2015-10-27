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
#include "rt5647.h"

// ignore unused warning message for current stage
#pragma GCC diagnostic ignored "-Wunused-function"

#define RT5647_CODEC_NAME   "rt5647"

#define AUDIO_ACCESS_RW (GB_AUDIO_ACCESS_READ | GB_AUDIO_ACCESS_WRITE)

struct rt5647_reg {
    uint8_t reg;
    uint16_t val;
};

struct rt5647_info {
    struct device *dev;
    uint8_t name[AUDIO_CODEC_NAME_MAX];

    struct gb_audio_dai *dai;

    struct gb_audio_control *controls;
    int num_controls;
    struct gb_audio_widget *widgets;
    int num_widgets;
    struct gb_audio_route *routes;
    int num_routes;
};

struct rt5647_reg rt5647_init_regs[] = {
    // {xxxx, xxxx},
};

struct gb_audio_dai rt5647_dai = {
    .name = "rt5647-aif1",
    .cport = 0,
    .capture = {
        .stream_name = "AIF1 Capture",
        .formats = RT5647_FORMATS,
        .rates = RT5647_STEREO_RATES,
        .chan_min = 1,
        .chan_max = 2,
    },
    .playback = {
        .stream_name = "AIF1 Playback",
        .formats = RT5647_FORMATS,
        .rates = RT5647_STEREO_RATES,
        .chan_min = 1,
        .chan_max = 2,
    },
};

struct gb_audio_control rt5647_controls[] = {
    {
        "SPK Playback Switch", 0, GB_AUDIO_IFACE_MIXER, AUDIO_ACCESS_RW, 0,
        GB_AUDIO_STREAM_TYPE_PLAYBACK,
    },
    {
        "SPK Playback Volume", 1, GB_AUDIO_IFACE_MIXER, AUDIO_ACCESS_RW, 0,
        GB_AUDIO_STREAM_TYPE_PLAYBACK,
    },
};

struct gb_audio_widget rt5647_widgets[] = {
    {
        "I2S1", 0, GB_AUDIO_WIDGET_TYPE_SUPPLY, GB_AUDIO_WIDGET_STATE_DISABLED,
        EN_I2S1,
    },
};

struct gb_audio_route rt5647_routes[] = {
    // { src, dest, control },
};

/* TODO: copy from Linux source code temporarily,
 * we should modify this function by ourself.
 */
static int fls(int x)
{
    int r = 32;

    if (!x)
        return 0;
    if (!(x & 0xffff0000u)) {
        x <<= 16;
        r -= 16;
    }
    if (!(x & 0xff000000u)) {
        x <<= 8;
        r -= 8;
    }
    if (!(x & 0xf0000000u)) {
        x <<= 4;
        r -= 4;
    }
    if (!(x & 0xc0000000u)) {
        x <<= 2;
        r -= 2;
    }
    if (!(x & 0x80000000u)) {
        x <<= 1;
        r -= 1;
    }
    return r;
}

static uint32_t audcodec_read(uint32_t reg, uint32_t *value) {
    // read register via i2c
    return 0;
}

static uint32_t audcodec_write(uint32_t reg, uint32_t value) {
    // write register via i2c
    return 0;
}

static int audcodec_read_bit(struct bitctl *ctl, uint32_t *value)
{
    uint32_t reg = 0, inv = 0, shift = 0, data = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg = ctl->reg;
    shift = ctl->shift;
    inv = ctl->inv;

    ret = audcodec_read(reg, &data);
    if (ret) {
        return -EIO;
    }
    data = (data >> shift) & 0x01;
    if (inv) {
        data = (data)? 0: 1;
    }

    // TODO: need to fill return value to 'value'.
    return 0;
}

static int audcodec_read_bits(struct bitctl *ctl, uint32_t *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & 0x01;
    if (inv) {
        data1 = (data1)? 0: 1;
    }
    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* two controls */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & 0x01;
        if (inv) {
            data2 = (data2)? 0: 1;
        }
    }
    // TODO: need to fill return value to 'value'.
    return 0;
}

static int audcodec_read_value(struct bitctl *ctl, uint32_t *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t max = 0, mask = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;

    if (!ctl || !value) {
        return -EINVAL;
    }

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;
    max = ctl->max;
    mask = (1 << fls(max)) - 1;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & mask;
    if (inv) {
        data1 = max - data1;
    }
    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* two controls */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & mask;
        if (inv) {
            data2 = max - data2;
        }
    }
    // TODO: need to fill return value to 'value'.
    return 0;
}

static int rt5647_get_topology_size(struct device *dev, uint16_t *size)
{
    struct rt5647_info *info = NULL;
    int tpg_size = 0;

    if (!dev || !device_get_private(dev) || !size) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    tpg_size = sizeof(struct gb_audio_topology);
    tpg_size += sizeof(struct gb_audio_dai);
    tpg_size += info->num_controls * sizeof(struct gb_audio_control);
    tpg_size += info->num_widgets * sizeof(struct gb_audio_widget);
    tpg_size += info->num_routes * sizeof(struct gb_audio_route);

    *size = tpg_size;
    return 0;
}

static int rt5647_get_topology(struct device *dev,
                               struct gb_audio_topology *topology)
{
    struct rt5647_info *info = NULL;
    int len = 0;
    uint8_t *data = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    topology->num_dais = 1;
    topology->num_controls = info->num_controls;
    topology->num_widgets = info->num_widgets;
    topology->num_routes = info->num_routes;

    data = topology->data;
    /* fill dai object */
    len = sizeof(struct gb_audio_dai);
    memcpy(data, info->dai, len);
    data += len;

    /* fill audio control object */
    len = topology->num_controls * sizeof(struct gb_audio_control);
    memcpy(data, info->controls, len);
    data += len;

    /* fill audio widget object */
    len = topology->num_widgets * sizeof(struct gb_audio_widget);
    memcpy(data, info->widgets, len);
    data += len;

    /* fill audio route object */
    len = topology->num_routes * sizeof(struct gb_audio_route);
    memcpy(data, info->routes, len);
    return 0;
}

static int rt5647_get_dai_config(struct device *dev,
                                 struct device_codec_dai_config *dai_config)
{
    return 0;
}

static int rt5647_set_dai_config(struct device *dev,
                                 struct device_codec_dai_config *dai_config)
{
    return 0;
}

static int rt5647_get_control(struct device *dev, uint8_t control_id,
                              uint32_t *value)
{
    int ret = 0;
    struct bitctl ctl;

    /* TODO: control_id should define as '#define ctl_xxx' or enum ctl_xxx. */
    switch (control_id) {
        case 0: /* 0: SPO L/R Playback Switch */
            ctl.reg = RT5647_SPKOUT;
            ctl.reg2 = RT5647_SPKOUT;
            ctl.shift = 15; /* TODO: should define as RT5647_L_MUTE_SFT */
            ctl.shift2 = 7;
            ctl.inv = 1;
            audcodec_read_bits(&ctl, value);
        break;
        case 1: /* 1: SPO L/R Playback Volume */
            ctl.reg = RT5647_SPKOUT;
            ctl.reg2 = RT5647_SPKOUT;
            ctl.shift = 8;
            ctl.shift2 = 0;
            ctl.inv = 0;
            ctl.max = 0x27;
            ctl.min = 0;
            audcodec_read_value(&ctl, value);
        break;
        case 2: /* 2: I2S Power enable */
            ctl.reg = RT5647_PWR_CTRL1;
            ctl.shift = 15;
            ctl.inv = 0;
            audcodec_read_bit(&ctl, value);
        break;
    }
    return ret;
}

static int rt5647_set_control(struct device *dev, uint8_t control_id,
                              uint32_t value)
{
    int ret = 0;
    struct bitctl ctl;

    /* TODO: control_id should define as '#define ctl_xxx' or enum ctl_xxx. */
    switch (control_id) {
        case 0: /* 0: SPO L/R Playback Switch */
            ctl.reg = RT5647_SPKOUT;
            ctl.reg2 = RT5647_SPKOUT;
            ctl.shift = 15; /* TODO: should define as RT5647_L_MUTE_SFT */
            ctl.shift2 = 7;
            ctl.inv = 1;
            //audcodec_write_bits(&ctl, value);
        break;
        case 1: /* 1: SPO L/R Playback Volume */
            ctl.reg = RT5647_SPKOUT;
            ctl.reg2 = RT5647_SPKOUT;
            ctl.shift = 8;
            ctl.shift2 = 0;
            ctl.inv = 0;
            ctl.max = 0x27;
            ctl.min = 0;
            //audcodec_write_value(&ctl, value);
        break;
        case 2: /* 2: I2S Power enable */
            ctl.reg = RT5647_PWR_CTRL1;
            ctl.shift = 15;
            ctl.inv = 0;
            //audcodec_write_bit(&ctl, value);
        break;
    }
    return ret;
}

static int rt5647_enable_widget(struct device *dev, uint8_t widget_id)
{
    return 0;
}

static int rt5647_disable_widget(struct device *dev, uint8_t widget_id)
{
    return 0;
}

static int rt5647_get_tx_delay(struct device *dev, uint32_t *delay)
{
    return 0;
}

static int rt5647_start_tx(struct device *dev)
{
    return 0;
}

static int rt5647_stop_tx(struct device *dev)
{
    return 0;
}

static int rt5647_register_tx_callback(struct device *dev,
                                       device_codec_event_callback *callback)
{
    return 0;
}

static int rt5647_get_rx_delay(struct device *dev, uint32_t *delay)
{
    return 0;
}

static int rt5647_start_rx(struct device *dev)
{
    return 0;
}

static int rt5647_stop_rx(struct device *dev)
{
    return 0;
}

static int rt5647_register_rx_callback(struct device *dev,
                                       device_codec_event_callback *callback)
{
    return 0;
}

static int rt5647_audcodec_open(struct device *dev)
{
    struct rt5647_info *info = NULL;
    int ret = 0, size = 0, i = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    size = ARRAY_SIZE(rt5647_init_regs);
    for (i = 0; i < size; i++) {
        audcodec_write(rt5647_init_regs[i].reg , rt5647_init_regs[i].val);
    }
    return ret;
}

static void rt5647_audcodec_close(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);
}

static int rt5647_audcodec_probe(struct device *dev)
{
    struct rt5647_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    strcpy((char*)info->name, RT5647_CODEC_NAME);

    info->dai = &rt5647_dai;
    info->controls = rt5647_controls;
    info->num_controls = ARRAY_SIZE(rt5647_controls);
    info->widgets = rt5647_widgets;
    info->num_widgets = ARRAY_SIZE(rt5647_widgets);
    info->routes = rt5647_routes;
    info->num_routes = ARRAY_SIZE(rt5647_routes);

    device_set_private(dev, info);
    return 0;
}

static void rt5647_audcodec_remove(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    device_set_private(dev, NULL);
    free(info);
}

static struct device_codec_type_ops rt5647_audcodec_type_ops = {
    .get_topology_size = rt5647_get_topology_size,
    .get_topology = rt5647_get_topology,
    .get_dai_config = rt5647_get_dai_config,
    .set_dai_config = rt5647_set_dai_config,
    .get_control = rt5647_get_control,
    .set_control = rt5647_set_control,
    .enable_widget = rt5647_enable_widget,
    .disable_widget = rt5647_disable_widget,
    .get_tx_delay = rt5647_get_tx_delay,
    .start_tx = rt5647_start_tx,
    .stop_tx = rt5647_stop_tx,
    .register_tx_callback = rt5647_register_tx_callback,
    .get_rx_delay = rt5647_get_rx_delay,
    .start_rx = rt5647_start_rx,
    .stop_rx = rt5647_stop_rx,
    .register_rx_callback = rt5647_register_rx_callback,
};

static struct device_driver_ops rt5647_audcodec_ops = {
    .probe          = rt5647_audcodec_probe,
    .remove         = rt5647_audcodec_remove,
    .open           = rt5647_audcodec_open,
    .close          = rt5647_audcodec_close,
    .type_ops       = &rt5647_audcodec_type_ops,
};

struct device_driver rt5647_audcodec = {
    .type       = DEVICE_TYPE_CODEC_HW,
    .name       = "rt5647",
    .desc       = "ALC5647 Audio Codec driver",
    .ops        = &rt5647_audcodec_ops,
};
