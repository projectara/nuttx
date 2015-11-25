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

#include <nuttx/audio.h>
#include <nuttx/device_i2s.h>
#include <nuttx/device_codec.h>
#include "i2s_test.h"

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
bool i2s_is_master = false;
bool codec_is_master = false;
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

int find_common_dai_settings(struct i2s_test_info *info)
{
    struct device *i2s_dev;
    struct device *codec_dev;
    struct device_dai i2s_dai;
    struct device_dai codec_dai;
    bool i2s_supports_master;
    bool codec_supports_master;
    int ret = OK;

    if ((!info->is_transmitter) ||
        (!info->init_codec)) {
        fprintf(stderr, "Settings not for Codec setup\n");
        ret = -EINVAL;
    }

    i2s_dev = device_open(DEVICE_TYPE_I2S_HW, 0);
    if (!i2s_dev) {
        fprintf(stderr, "open i2s failed\n");
        return -EIO;
    }

    codec_dev = device_open(DEVICE_TYPE_CODEC_HW, 0);
    if (!codec_dev) {
        fprintf(stderr, "open codec failed\n");
        goto dev_close;
    }

    /*get i2s master capabilities */
    ret = device_i2s_get_caps(i2s_dev,
                              DEVICE_DAI_ROLE_MASTER,
                              &i2s_test_pcm,
                              &i2s_dai);
    /* Check for matching configuration */
    if (ret) {
        if (ret == -EOPNOTSUPP) {
            fprintf(stderr, "I2S does support master role\n");
            i2s_supports_master = false;
        } else {
            fprintf(stderr, "I2S PCM settings not supported\n");
            goto dev_close;
        }
    } else {
        i2s_supports_master = true;
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
            codec_supports_master = false;
        } else {
            fprintf(stderr, "Codec PCM settings not supported\n");
            goto dev_close;
        }
    } else {
        codec_supports_master = true;
    }

    /* get compatible set of dai settings */
    compatible_dai.protocol = i2s_dai.protocol & codec_dai.protocol;
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
    if(!ONE_BIT_IS_SET(compatible_dai.protocol)) {
        compatible_dai.protocol = choose_single_bit(compatible_dai.protocol);
    }

    if(!ONE_BIT_IS_SET(compatible_dai.wclk_polarity)) {
        compatible_dai.wclk_polarity = choose_single_bit(compatible_dai.wclk_polarity);
    }

    if(!ONE_BIT_IS_SET(compatible_dai.wclk_change_edge)) {
        compatible_dai.wclk_change_edge = choose_single_bit(compatible_dai.wclk_change_edge);
    }

    if(!ONE_BIT_IS_SET(compatible_dai.data_rx_edge)) {
        compatible_dai.data_rx_edge = choose_single_bit(compatible_dai.data_rx_edge);
    }

    if(!ONE_BIT_IS_SET(compatible_dai.data_tx_edge)) {
        compatible_dai.data_tx_edge = choose_single_bit(compatible_dai.data_tx_edge);
    }

    /* find a master */
    if (i2s_supports_master) {
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
            i2s_is_master = true;
            goto dev_close;
        }
    }

    /* find a master */
    if (codec_supports_master) {
        /* check if the i2s slave works with the codec mclk */
        compatible_dai.mclk_freq = codec_dai.mclk_freq;
        ret = device_i2s_get_caps(i2s_dev,
                                  DEVICE_DAI_ROLE_SLAVE,
                                  &i2s_test_pcm,
                                  &compatible_dai);

        if(ret == OK)
        {
            fprintf(stderr, "Found compatible settings for Codec as master\n");
            codec_is_master = true;
            goto dev_close;
        }
    }

    /* normally here we would loop back and iterate though all codec dai index here */
    /* to keep the test simple only try index 0 */
    ret = -EOPNOTSUPP;

dev_close:
    device_close(i2s_dev);
    if (codec_dev) {
        device_close(codec_dev);
    }

    return ret;
}

int set_common_dai_settings(struct i2s_test_info *info)
{

    struct device *i2s_dev;
    struct device *codec_dev;
    int ret = OK;

    if ((!info->is_transmitter) ||
        (!info->init_codec)) {
        fprintf(stderr, "Settings not for Codec setup\n");
        ret = -EINVAL;
    }

    if ((!i2s_is_master) &&
        (!codec_is_master)) {
        fprintf(stderr, "in compatable settig to I2S and Codec\n");
        ret = -EINVAL;
    }

    i2s_dev = device_open(DEVICE_TYPE_I2S_HW, 0);
    if (!i2s_dev) {
        fprintf(stderr, "open i2s failed\n");
        return -EIO;
    }

    codec_dev = device_open(DEVICE_TYPE_CODEC_HW, 0);
    if (!codec_dev) {
        fprintf(stderr, "open codec failed\n");
        goto dev_close;
    }

    if (i2s_is_master) {
        ret = device_i2s_set_config(i2s_dev,
                                   DEVICE_DAI_ROLE_MASTER,
                                   &i2s_test_pcm,
                                   &compatible_dai);
        if (ret) {
            fprintf(stderr, "set I2S configuration failed: %d\n", ret);
            goto dev_close;
        }

        ret = device_codec_set_config(i2s_dev,
                                   0,
                                   DEVICE_DAI_ROLE_SLAVE,
                                   &codec_test_pcm,
                                   &compatible_dai);
        if (ret) {
            fprintf(stderr, "set Codec configuration failed: %d\n", ret);
            goto dev_close;
        }
    } else if (codec_is_master) {
        ret = device_i2s_set_config(i2s_dev,
                                   DEVICE_DAI_ROLE_SLAVE,
                                   &i2s_test_pcm,
                                   &compatible_dai);
        if (ret) {
            fprintf(stderr, "set I2S configuration failed: %d\n", ret);
            goto dev_close;
        }

        ret = device_codec_set_config(i2s_dev,
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
    device_close(i2s_dev);
    if (codec_dev) {
        device_close(codec_dev);
    }

    return ret;
}

