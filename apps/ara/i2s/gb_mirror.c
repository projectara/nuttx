/*
* Copyright (c) 2015 Google Inc.
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


/**
Mirror greybus functions to that determine I2S and codec compatibility
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
#include "gb_mirror.h"



static int gb_audio_gb_to_i2s_format(uint32_t gb_format, uint32_t *i2s_format,
                                     unsigned int *bytes)
{
    int ret = 0;

    switch (gb_format) {
    case GB_AUDIO_PCM_FMT_S8:
    case GB_AUDIO_PCM_FMT_U8:
        *i2s_format = DEVICE_I2S_PCM_FMT_8;
        *bytes = 1;
        break;
    case GB_AUDIO_PCM_FMT_S16_LE:
    case GB_AUDIO_PCM_FMT_S16_BE:
    case GB_AUDIO_PCM_FMT_U16_LE:
    case GB_AUDIO_PCM_FMT_U16_BE:
        *i2s_format = DEVICE_I2S_PCM_FMT_16;
        *bytes = 2;
        break;
    case GB_AUDIO_PCM_FMT_S24_LE:
    case GB_AUDIO_PCM_FMT_S24_BE:
    case GB_AUDIO_PCM_FMT_U24_LE:
    case GB_AUDIO_PCM_FMT_U24_BE:
        *i2s_format = DEVICE_I2S_PCM_FMT_24;
        *bytes = 3;
        break;
    case GB_AUDIO_PCM_FMT_S32_LE:
    case GB_AUDIO_PCM_FMT_S32_BE:
    case GB_AUDIO_PCM_FMT_U32_LE:
    case GB_AUDIO_PCM_FMT_U32_BE:
        *i2s_format = DEVICE_I2S_PCM_FMT_32;
        *bytes = 4;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int gb_audio_convert_rate(uint32_t gb_rate, uint32_t *i2s_rate,
                                 unsigned int *freq)
{
    switch (gb_rate) {
    case GB_AUDIO_PCM_RATE_5512:
        *i2s_rate = DEVICE_I2S_PCM_RATE_5512;
        *freq = 5512;
        break;
    case GB_AUDIO_PCM_RATE_8000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_8000;
        *freq = 8000;
        break;
    case GB_AUDIO_PCM_RATE_11025:
        *i2s_rate = DEVICE_I2S_PCM_RATE_11025;
        *freq = 11025;
        break;
    case GB_AUDIO_PCM_RATE_16000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_16000;
        *freq = 16000;
        break;
    case GB_AUDIO_PCM_RATE_22050:
        *i2s_rate = DEVICE_I2S_PCM_RATE_22050;
        *freq = 22050;
        break;
    case GB_AUDIO_PCM_RATE_32000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_32000;
        *freq = 32000;
        break;
    case GB_AUDIO_PCM_RATE_44100:
        *i2s_rate = DEVICE_I2S_PCM_RATE_44100;
        *freq = 44100;
        break;
    case GB_AUDIO_PCM_RATE_48000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_48000;
        *freq = 48000;
        break;
    case GB_AUDIO_PCM_RATE_64000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_64000;
        *freq = 64000;
        break;
    case GB_AUDIO_PCM_RATE_88200:
        *i2s_rate = DEVICE_I2S_PCM_RATE_88200;
        *freq = 88200;
        break;
    case GB_AUDIO_PCM_RATE_96000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_96000;
        *freq = 96000;
        break;
    case GB_AUDIO_PCM_RATE_176400:
        *i2s_rate = DEVICE_I2S_PCM_RATE_176400;
        *freq = 176400;
        break;
    case GB_AUDIO_PCM_RATE_192000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_192000;
        *freq = 192000;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int gb_audio_determine_protocol(struct device_codec_dai *codec_dai,
                                       struct device_i2s_dai *i2s_dai)
{
    if ((codec_dai->protocol & DEVICE_CODEC_PROTOCOL_I2S) &&
        (i2s_dai->protocol & DEVICE_I2S_PROTOCOL_I2S)) {
        codec_dai->protocol = DEVICE_CODEC_PROTOCOL_I2S;
        i2s_dai->protocol = DEVICE_I2S_PROTOCOL_I2S;
    } else if ((codec_dai->protocol & DEVICE_CODEC_PROTOCOL_LR_STEREO) &&
               (i2s_dai->protocol & DEVICE_I2S_PROTOCOL_LR_STEREO)) {
        codec_dai->protocol = DEVICE_CODEC_PROTOCOL_LR_STEREO;
        i2s_dai->protocol = DEVICE_I2S_PROTOCOL_LR_STEREO;
    } else if ((codec_dai->protocol & DEVICE_CODEC_PROTOCOL_PCM) &&
               (i2s_dai->protocol & DEVICE_I2S_PROTOCOL_PCM)) {
        codec_dai->protocol = DEVICE_CODEC_PROTOCOL_PCM;
        i2s_dai->protocol = DEVICE_I2S_PROTOCOL_PCM;
    } else {
        return -EINVAL;
    }

    return 0;
}

static int gb_audio_determine_wclk_polarity(struct device_codec_dai *codec_dai,
                                            struct device_i2s_dai *i2s_dai)
{
    if ((codec_dai->wclk_polarity & DEVICE_CODEC_POLARITY_NORMAL) &&
        (i2s_dai->wclk_polarity & DEVICE_I2S_POLARITY_NORMAL)) {
        codec_dai->wclk_polarity = DEVICE_CODEC_POLARITY_NORMAL;
        i2s_dai->wclk_polarity = DEVICE_I2S_POLARITY_NORMAL;
    } else if ((codec_dai->wclk_polarity & DEVICE_CODEC_POLARITY_REVERSED) &&
               (i2s_dai->wclk_polarity & DEVICE_I2S_POLARITY_REVERSED)) {
        codec_dai->wclk_polarity = DEVICE_CODEC_POLARITY_REVERSED;
        i2s_dai->wclk_polarity = DEVICE_I2S_POLARITY_REVERSED;
    } else {
        return -EINVAL;
    }

    return 0;
}

static int gb_audio_determine_wclk_change_edge(uint8_t codec_clk_role,
                                            struct device_codec_dai *codec_dai,
                                            struct device_i2s_dai *i2s_dai)
{
    if ((codec_clk_role == DEVICE_CODEC_ROLE_MASTER) &&
        (codec_dai->wclk_change_edge & DEVICE_CODEC_EDGE_FALLING) &&
        (i2s_dai->wclk_change_edge & DEVICE_I2S_EDGE_RISING)) {
        codec_dai->wclk_change_edge = DEVICE_CODEC_EDGE_FALLING;
        i2s_dai->wclk_change_edge = DEVICE_I2S_EDGE_RISING;
    } if ((codec_clk_role == DEVICE_CODEC_ROLE_SLAVE) &&
          (codec_dai->wclk_change_edge & DEVICE_CODEC_EDGE_RISING) &&
          (i2s_dai->wclk_change_edge & DEVICE_I2S_EDGE_FALLING)) {
          codec_dai->wclk_change_edge = DEVICE_CODEC_EDGE_RISING;
          i2s_dai->wclk_change_edge = DEVICE_I2S_EDGE_FALLING;
    } else {
        return -EINVAL;
    }

    return 0;
}

static int gb_audio_determine_data_edges(struct device_codec_dai *codec_dai,
                                         struct device_i2s_dai *i2s_dai)
{
    if ((codec_dai->data_tx_edge & DEVICE_CODEC_EDGE_FALLING) &&
        (i2s_dai->data_rx_edge & DEVICE_I2S_EDGE_RISING)) {
        codec_dai->data_tx_edge = DEVICE_CODEC_EDGE_FALLING;
        i2s_dai->data_rx_edge = DEVICE_I2S_EDGE_RISING;
    } else if ((codec_dai->data_tx_edge & DEVICE_CODEC_EDGE_RISING) &&
               (i2s_dai->data_rx_edge & DEVICE_I2S_EDGE_FALLING)) {
        codec_dai->data_tx_edge = DEVICE_CODEC_EDGE_RISING;
        i2s_dai->data_rx_edge = DEVICE_I2S_EDGE_FALLING;
    } else {
        return -EINVAL;
    }

    if ((codec_dai->data_rx_edge & DEVICE_CODEC_EDGE_RISING) &&
        (i2s_dai->data_tx_edge & DEVICE_I2S_EDGE_FALLING)) {
        codec_dai->data_rx_edge = DEVICE_CODEC_EDGE_RISING;
        i2s_dai->data_tx_edge = DEVICE_I2S_EDGE_FALLING;
    } else if ((codec_dai->data_rx_edge & DEVICE_CODEC_EDGE_FALLING) &&
               (i2s_dai->data_tx_edge & DEVICE_I2S_EDGE_RISING)) {
        codec_dai->data_rx_edge = DEVICE_CODEC_EDGE_FALLING;
        i2s_dai->data_tx_edge = DEVICE_I2S_EDGE_RISING;
    } else {
        return -EINVAL;
    }

    return 0;
}

static int gb_audio_set_config(struct gb_audio_dai_info *dai,
                               uint8_t codec_clk_role,
                               struct device_codec_pcm *codec_pcm,
                               struct device_i2s_pcm *i2s_pcm)
{
    struct device_codec_dai codec_dai;
    struct device_i2s_dai i2s_dai;
    uint8_t i2s_clk_role;
    int ret;

    if (codec_clk_role == DEVICE_CODEC_ROLE_MASTER) {
        ret = device_codec_get_caps(dai->info->codec_dev, dai->dai_idx,
                                    codec_clk_role, codec_pcm, &codec_dai);
        if (ret) {
            return ret;
        }

        /*
         * When codec_clk_role is DEVICE_CODEC_ROLE_MASTER,
         * device_codec_get_caps() sets codec_dai.mclk_freq.
         */
        i2s_dai.mclk_freq = codec_dai.mclk_freq;

        ret = device_i2s_get_caps(dai->i2s_dev, DEVICE_I2S_ROLE_SLAVE, i2s_pcm,
                                  &i2s_dai);
        if (ret) {
            return ret;
        }

        i2s_clk_role = DEVICE_I2S_ROLE_SLAVE;
    } else {
        ret = device_i2s_get_caps(dai->i2s_dev, DEVICE_I2S_ROLE_MASTER, i2s_pcm,
                                  &i2s_dai);
        if (ret) {
            return ret;
        }

        /*
         * When i2s_clk_role is DEVICE_I2S_ROLE_MASTER,
         * device_i2s_get_caps() sets i2s_dai.mclk_freq.
         */
        codec_dai.mclk_freq = i2s_dai.mclk_freq;

        ret = device_codec_get_caps(dai->info->codec_dev, dai->dai_idx,
                                    codec_clk_role, codec_pcm, &codec_dai);
        if (ret) {
            return ret;
        }

        i2s_clk_role = DEVICE_I2S_ROLE_MASTER;
    }

    ret = gb_audio_determine_protocol(&codec_dai, &i2s_dai);
    if (ret) {
        return ret;
    }

    ret = gb_audio_determine_wclk_polarity(&codec_dai, &i2s_dai);
    if (ret) {
        return ret;
    }

    ret = gb_audio_determine_wclk_change_edge(codec_clk_role, &codec_dai,
                                              &i2s_dai);
    if (ret) {
        return ret;
    }

    ret = gb_audio_determine_data_edges(&codec_dai, &i2s_dai);
    if (ret) {
        return ret;
    }

    ret = device_codec_set_config(dai->info->codec_dev, dai->dai_idx,
                                  codec_clk_role, codec_pcm, &codec_dai);
    if (ret) {
        return ret;
    }

    ret = device_i2s_set_config(dai->i2s_dev, i2s_clk_role, i2s_pcm, &i2s_dai);
    if (ret) {
        return ret;
    }

    return 0;
}

int gb_audio_config_connection(struct gb_audio_dai_info *dai,
                                      uint32_t format, uint32_t rate,
                                      uint8_t channels, uint8_t sig_bits)
{
    struct device_codec_pcm codec_pcm;
    struct device_i2s_pcm i2s_pcm;
    unsigned int bytes, freq;
    int ret;

    codec_pcm.format = format;
    codec_pcm.rate = rate;
    codec_pcm.channels = channels;
    codec_pcm.sig_bits = sig_bits;

    ret = gb_audio_gb_to_i2s_format(format, &i2s_pcm.format, &bytes);
    if (ret) {
        return ret;
    }

    ret = gb_audio_convert_rate(rate, &i2s_pcm.rate, &freq);
    if (ret) {
        return ret;
    }

    i2s_pcm.channels = channels;

    ret = gb_audio_set_config(dai, DEVICE_CODEC_ROLE_MASTER, &codec_pcm,
                              &i2s_pcm);
    if (ret) {
        ret = gb_audio_set_config(dai, DEVICE_CODEC_ROLE_SLAVE, &codec_pcm,
                                  &i2s_pcm);
        if (ret) {
            return ret;
        }
    }

    dai->sample_size = bytes * dai->channels;
    dai->sample_freq = freq;

    return 0;
}
