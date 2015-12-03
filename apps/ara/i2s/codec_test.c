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

struct device_dai test_dai = {
    6144000,
    0,
    DEVICE_DAI_POLARITY_NORMAL,
    0,
    DEVICE_DAI_EDGE_RISING,
    DEVICE_DAI_EDGE_FALLING
};

/* these variables are set by calling find_common_pcm_settings */
bool i2s_is_mclk_master = false;
bool codec_is_mclk_master = false;
struct device_dai compatible_dai;

static uint32_t choose_single_bit(uint32_t bit_field)
{
    int i;
    uint32_t mask = 1;

    for (i = 0; i < 32; i++) {
        if ((mask & bit_field) > 0) {
            return mask;
        } else {
            mask <<= 1;
        }
    }

    return 0;
}

static int find_common_dai_settings(struct i2s_test_info *info,
                                    struct device *i2s_dev,
                                    struct device *codec_dev)
{
    struct device_dai i2s_dai;
    struct device_dai codec_dai;
    bool i2s_supports_mclk_master;
    bool codec_supports_mclk_master;
    int ret = OK;

    /*get i2s master capabilities */
    ret = device_i2s_get_caps(i2s_dev,
                              DEVICE_DAI_ROLE_MASTER,
                              &i2s_test_pcm,
                              &i2s_dai);
    /* Check for matching configuration */
    if (ret) {
        if (ret == -EOPNOTSUPP) {
            fprintf(stderr, "I2S does support master role\n");
            i2s_supports_mclk_master = false;
        } else {
            fprintf(stderr, "I2S PCM settings not supported\n");
            goto dev_close;
        }
    } else {
        i2s_supports_mclk_master = true;
    }

    /*get codec master capabilities of first dai index */
    ret = device_codec_get_caps(codec_dev,
                                0,
                                DEVICE_DAI_ROLE_MASTER,
                                &codec_test_pcm,
                                &codec_dai);
    /* Check for matching configuration */
    if (ret) {
        if (ret == -EOPNOTSUPP) {
            fprintf(stderr, "Codec does support master role\n");
            codec_supports_mclk_master = false;
        } else {
            fprintf(stderr, "Codec PCM settings not supported\n");
            goto dev_close;
        }
    } else {
        codec_supports_mclk_master = true;
    }

    /* get compatible set of dai settings */
    compatible_dai.protocol = i2s_dai.protocol & codec_dai.protocol;
    if(info->is_i2s)
    {
        if(!(compatible_dai.protocol | DEVICE_DAI_PROTOCOL_I2S)) {
            fprintf(stderr, "I2S requested but is not a compatible protocol\n");
            goto dev_close;
        }
        compatible_dai.protocol = DEVICE_DAI_PROTOCOL_I2S;
    } else {
        if(!(compatible_dai.protocol | DEVICE_DAI_PROTOCOL_LR_STEREO)) {
            fprintf(stderr, "LRStereo requested but is not a compatible protocol\n");
            goto dev_close;
        }
        compatible_dai.protocol = DEVICE_DAI_PROTOCOL_LR_STEREO;
    }

    compatible_dai.wclk_polarity = i2s_dai.wclk_polarity & codec_dai.wclk_polarity;
    compatible_dai.wclk_change_edge = i2s_dai.wclk_change_edge & codec_dai.wclk_change_edge;
    compatible_dai.data_rx_edge = i2s_dai.data_rx_edge & codec_dai.data_rx_edge;
    compatible_dai.data_tx_edge = i2s_dai.data_tx_edge & codec_dai.data_tx_edge;

    /* ensure there is compatibility to all fields */
    if( (compatible_dai.protocol == 0) ||
        (compatible_dai.wclk_polarity == 0) ||
        (compatible_dai.wclk_change_edge == 0) ||
        (compatible_dai.data_rx_edge == 0) ||
        (compatible_dai.data_tx_edge == 0))
    {
        fprintf(stderr, "Incompatible DAI setting between I2S and Codec\n");
        goto dev_close;
    }

    /* make single bit compatibility fields */
    if(!ONE_BIT_IS_SET(compatible_dai.wclk_polarity)) {
        compatible_dai.wclk_polarity = choose_single_bit(compatible_dai.wclk_polarity);
    }

    if(!ONE_BIT_IS_SET(compatible_dai.wclk_change_edge)) {
        //rem chris compatible_dai.wclk_change_edge = choose_single_bit(compatible_dai.wclk_change_edge);
        compatible_dai.wclk_change_edge = DEVICE_DAI_EDGE_FALLING;
    }

    if(!ONE_BIT_IS_SET(compatible_dai.data_rx_edge)) {
        //rem chris compatible_dai.data_rx_edge = choose_single_bit(compatible_dai.data_rx_edge);
        compatible_dai.data_rx_edge = DEVICE_DAI_EDGE_RISING;
    }

    if(!ONE_BIT_IS_SET(compatible_dai.data_tx_edge)) {
        // rem chris compatible_dai.data_tx_edge = choose_single_bit(compatible_dai.data_tx_edge);
        compatible_dai.data_tx_edge = compatible_dai.wclk_change_edge;
    }

    /* find a master */
    if (i2s_supports_mclk_master) {
        /* check if the codec slave works with the i2s mclk */
        compatible_dai.mclk_freq = i2s_dai.mclk_freq;
        ret = device_codec_get_caps(codec_dev,
                                    0,
                                    DEVICE_DAI_ROLE_SLAVE,
                                    &codec_test_pcm,
                                    &compatible_dai);

        if(ret == OK)
        {
            fprintf(stderr, "Found compatible settings for I2S as master\n");
            i2s_is_mclk_master = true;
            goto dev_close;
        }
    }

    /* find a master */
    if (codec_supports_mclk_master) {
        /* check if the i2s slave works with the codec mclk */
        compatible_dai.mclk_freq = codec_dai.mclk_freq;
        ret = device_i2s_get_caps(i2s_dev,
                                  DEVICE_DAI_ROLE_SLAVE,
                                  &i2s_test_pcm,
                                  &compatible_dai);

        if(ret == OK)
        {
            fprintf(stderr, "Found compatible settings for Codec as master\n");
            codec_is_mclk_master = true;
            goto dev_close;
        }
    }

    /* normally here we would loop back and iterate though all codec dai index here */
    /* to keep the test simple only try index 0 */
    ret = -EOPNOTSUPP;

dev_close:

    return ret;
}

static int set_common_dai_settings(struct i2s_test_info *info,
                                   struct device *i2s_dev,
                                   struct device *codec_dev)
{
    int ret = OK;

    if (i2s_is_mclk_master) {
        ret = device_i2s_set_config(i2s_dev,
                                   DEVICE_DAI_ROLE_MASTER,
                                   &i2s_test_pcm,
                                   &compatible_dai);
        if (ret) {
            fprintf(stderr, "set I2S configuration failed: %d\n", ret);
            goto dev_close;
        }

        ret = device_codec_set_config(codec_dev,
                                      0,
                                      DEVICE_DAI_ROLE_SLAVE,
                                      &codec_test_pcm,
                                      &compatible_dai);
        if (ret) {
            fprintf(stderr, "set Codec configuration failed: %d\n", ret);
            goto dev_close;
        }
    } else if (codec_is_mclk_master) {
        ret = device_i2s_set_config(i2s_dev,
                                   DEVICE_DAI_ROLE_SLAVE,
                                   &i2s_test_pcm,
                                   &compatible_dai);
        if (ret) {
            fprintf(stderr, "set I2S configuration failed: %d\n", ret);
            goto dev_close;
        }

        ret = device_codec_set_config(codec_dev,
                                      0,
                                      DEVICE_DAI_ROLE_MASTER,
                                      &codec_test_pcm,
                                      &compatible_dai);
        if (ret) {
            fprintf(stderr, "set Codec configuration failed: %d\n", ret);
            goto dev_close;
        }
    }

dev_close:

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

    ret = i2s_test_start_transmitter(info, i2s_dev);
    if (ret)
        goto err_i2s;

    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&i2s_test_done_sem) && (errno == EINTR));

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

    ret = find_common_dai_settings(info, i2s_dev, codec_dev);
    if(ret)
        goto dev_close;

    ret = set_common_dai_settings(info, i2s_dev, codec_dev);
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

