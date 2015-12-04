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
#include <stdio.h>

#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/list.h>

#include <nuttx/device_i2s.h>
#include <nuttx/device_codec.h>
#include "i2s_test.h"
#include "gen_pcm.h"
#include "gb_mirror.h"

/**
 * hard coded test configuration parameters
 */
struct device_i2s_pcm i2s_test_pcm = {
    DEVICE_I2S_PCM_FMT_16,
    DEVICE_I2S_PCM_RATE_48000,
    2
};
struct device_codec_pcm codec_test_pcm = {
    GB_AUDIO_PCM_FMT_S16_LE,
    GB_AUDIO_PCM_RATE_48000,
    2,
    16
};

// rem chris i2s_test -t -i -f 1000 -v 19 -C 2 100
int negotiate_i2s_to_codec_interface(struct i2s_test_info *info,
                                     struct device *i2s_dev,
                                     struct device *codec_dev)
{
    int ret = OK;

    struct gb_audio_dai_info dai;
    struct gb_audio_info gb_info;

    /* fill in just enough to make things work */
    gb_info.initialized = true;
    gb_info.codec_dev = codec_dev;

    dai.info = &gb_info;
    dai.i2s_dev = i2s_dev;
    dai.dai_idx = 0;

    ret = gb_audio_config_connection(&dai,
                                     codec_test_pcm.format,
                                     codec_test_pcm.rate,
                                     codec_test_pcm.channels,
                                     codec_test_pcm.sig_bits);

    if (ret) {
        fprintf(stderr, "Failed to configure negotiate i2s_to_codec interface. %d\n",ret);
    }

    return ret;
}

static int stream_i2s_to_codec(struct i2s_test_info *info,
                               struct device *i2s_dev,
                               struct device *codec_dev)
{
    int ret = OK;

    ret = device_codec_start_rx(codec_dev, 0);
    if (ret)
        goto err_codec;

    printf("start to playback...\n");
    ret = i2s_test_start_transmitter(info, i2s_dev);
    if (ret)
        goto err_i2s;

#if 1
    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&i2s_test_done_sem) && (errno == EINTR));
#else
    i = 0;

    // delay 15s
    for (i = 0; i < 1000; i++) {
        usleep(1000);
    }
#endif
    printf("stop to playback.\n");
    i2s_test_stop_transmitter(i2s_dev);

err_i2s:
    device_codec_stop_rx(codec_dev, 0);

err_codec:

    return ret;
}

/*copied from codec driver */
enum {
    RT5647_CTL_SPKOUT_MUTE,
    RT5647_CTL_SPKOUT_VOL,
    RT5647_CTL_SPKVOL_MUTE,
    RT5647_CTL_DAC2_SWITCH,
    RT5647_CTL_DAC2_VOL
};

struct gb_audio_widget * find_widget(struct gb_audio_widget *widgets,
                                     int num_widgets, int id)
{
    int i = 0;
    for (i = 0; i < num_widgets; i++) {
        if (widgets[i].id == id) {
            return &widgets[i];
        }
    }
    return NULL;
}

static int enable_codec_speaker(struct i2s_test_info *info,
                                struct device *dev)
{
    int ret = 0, i = 0, offset = 0;
    uint16_t tp_size = 0;
    struct gb_audio_topology *tp = NULL;
    struct gb_audio_dai *dais = NULL;
    struct gb_audio_control *controls = NULL;
    struct gb_audio_widget *widgets = NULL, *src = NULL, *dst = NULL;
    struct gb_audio_route *routes = NULL;
    uint8_t *buf = NULL;
    struct gb_audio_ctl_elem_value values[2];

    printf("%s\n",__func__);
    if (!dev) {
        return -EINVAL;
    }

    ret = device_codec_get_topology_size(dev, &tp_size);
    if (ret) {
        printf("get topology size fail!\n");
        return -EINVAL;
    }

    printf("tp_size = %d\n",tp_size);
    tp = zalloc(tp_size);
    if (!tp) {
        printf("failed to allocate memory. size = %d\n", tp_size);
        return -EINVAL;
    }

    ret = device_codec_get_topology(dev, tp);
    if (ret) {
        printf("get topology data fail!\n");
        goto codec_err;
    }

    buf = tp->data;
    offset = 0;

    dais = (struct gb_audio_dai *)(buf + offset);
    offset += tp->num_dais * sizeof(struct gb_audio_dai);

    controls = (struct gb_audio_control *)(buf + offset);
    offset += tp->num_controls * sizeof(struct gb_audio_control);

    widgets = (struct gb_audio_widget *)(buf + offset);
    offset += tp->num_widgets * sizeof(struct gb_audio_widget);

    routes = (struct gb_audio_route *)(buf + offset);

    // list all component
    for (i = 0; i < tp->num_dais; i++) {
        printf("dai[%d] : %s\n", i, dais[i].name);
    }
    for (i = 0; i < tp->num_controls; i++) {
        printf("control[%d] : %s\n", i, controls[i].name);
    }
    for (i = 0; i < tp->num_widgets; i++) {
        printf("widget[%d] : %s\n", i, widgets[i].name);
    }
    for (i = 0; i < tp->num_routes; i++) {
        printf("route[%d] : %d -> %d ->%d-%d\n", i, routes[i].source_id,
             routes[i].control_id, routes[i].destination_id, routes[i].index );
    }

    // initialize routing table
    for (i = 0; i < tp->num_routes; i++) {
        /* enable widget of source */
        src = find_widget(widgets, tp->num_routes, routes[i].source_id);
        dst = find_widget(widgets, tp->num_routes, routes[i].destination_id);
        if (!src || !dst) {
            /* can't find these widgets, skip it */
            continue;
        }
        printf("Route: %s[%d] -> %s[%d] [%x-%u]\n", src->name,
               routes[i].source_id, dst->name, routes[i].destination_id,
               routes[i].control_id, routes[i].index);
        /* enable widgets of srouce and destination */
        device_codec_enable_widget(dev, src->id);
        device_codec_enable_widget(dev, dst->id);

        if (routes[i].control_id != 0xFF) {
            if (dst->type == GB_AUDIO_WIDGET_TYPE_MUX) {
                values[0].value.integer_value = routes[i].index;
            } else {
                values[0].value.integer_value = 1;
            }
            device_codec_set_control(dev, routes[i].control_id, 
                                     0, values);
        }
    }

    // enable audio controls
    values[0].value.integer_value = 8;
    values[1].value.integer_value = 8;
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_SPKOUT_VOL,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_SPKOUT_VOL did not work: error %d\n",ret);
        goto codec_err;
    }
    values[0].value.integer_value =0x75;
    values[1].value.integer_value = 0x75;
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_DAC2_VOL,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_DAC2_VOL did not work: error %d\n",ret);
        goto codec_err;
    }
    values[0].value.integer_value = 1;
    values[1].value.integer_value = 1;
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_SPKOUT_MUTE,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_SPKOUT_MUTE did not work: error %d\n",ret);
        goto codec_err;
    }
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_SPKVOL_MUTE,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_SPKVOL_MUTE did not work: error %d\n",ret);
        goto codec_err;
    }
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_DAC2_SWITCH,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_DAC2_SWITCH did not work: error %d\n",ret);
        goto codec_err;
    }

codec_err:
    free(tp);
    return ret;
}

static int disable_codec_speaker(struct i2s_test_info *info,
                                struct device *dev)
{
    int ret = 0;
    struct gb_audio_ctl_elem_value values[2];

    printf("%s\n",__func__);
    if (!dev) {
        return -EINVAL;
    }

    // enable audio controls
    values[0].value.integer_value = 0x27;
    values[1].value.integer_value = 0x27;
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_SPKOUT_VOL,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_SPKOUT_VOL did not work: error %d\n",ret);
        goto codec_err;
    }
    values[0].value.integer_value = 0x0;
    values[1].value.integer_value = 0x0;
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_DAC2_VOL,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_DAC2_VOL did not work: error %d\n",ret);
        goto codec_err;
    }
    values[0].value.integer_value = 0;
    values[1].value.integer_value = 0;
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_SPKOUT_MUTE,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_SPKOUT_MUTE did not work: error %d\n",ret);
        goto codec_err;
    }
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_SPKVOL_MUTE,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_SPKVOL_MUTE did not work: error %d\n",ret);
        goto codec_err;
    }
    ret = device_codec_set_control(dev,
                                   RT5647_CTL_DAC2_SWITCH,
                                   0,  //no parent widget
                                   values);
    if (ret) {
        fprintf(stderr, "RT5647_CTL_DAC2_SWITCH did not work: error %d\n",ret);
        goto codec_err;
    }

codec_err:
    return ret;
}

int play_sine_wave_via_codec(struct i2s_test_info *info)
{
    struct device *i2s_dev = NULL;
    struct device *codec_dev = NULL;
    int ret = OK;

    if ((!info->is_transmitter) ||
        (!info->use_codec)) {
        fprintf(stderr, "Settings not for Codec setup\n");
        ret = -EINVAL;
    }

    i2s_dev = device_open(DEVICE_TYPE_I2S_HW, 0);
    if (!i2s_dev) {
        fprintf(stderr, "open i2s failed\n");
        ret = -EIO;
    }

    codec_dev = device_open(DEVICE_TYPE_CODEC_HW, 0);
    if (!codec_dev) {
        fprintf(stderr, "open codec failed\n");
        ret = -EIO;
        goto dev_close;
    }

    ret = negotiate_i2s_to_codec_interface(info, i2s_dev, codec_dev);
    if(ret)
        goto dev_close;

    ret = enable_codec_speaker(info, codec_dev);
    if(ret)
        goto dev_close;

    ret = stream_i2s_to_codec(info, i2s_dev, codec_dev);
    if(ret)
        goto dev_close;

    ret = disable_codec_speaker(info, codec_dev);
    if(ret)
        goto dev_close;
dev_close:
    if (i2s_dev) {
        device_close(i2s_dev);
    }
    if (codec_dev) {
        device_close(codec_dev);
    }

    return ret;
}

