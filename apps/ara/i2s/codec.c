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

struct device_i2s_dai i2s_test_dai = {
    6144000,
    0,
    DEVICE_I2S_POLARITY_NORMAL,
    0,
    DEVICE_I2S_EDGE_RISING,
    DEVICE_I2S_EDGE_FALLING
};
struct device_codec_dai codec_test_dai = {
    6144000,
    0,
    DEVICE_CODEC_POLARITY_NORMAL,
    0,
    DEVICE_CODEC_EDGE_RISING,
    DEVICE_CODEC_EDGE_FALLING
};

static int find_common_pcm_settings(struct i2s_test_info *info)
{
    struct device *i2s_dev;
    struct device *codec_dev;
    struct device_i2s_dai dai;
    bool i2s_supports_master;
    bool codec_supports_master;
    int ret;

    if ((!info->is_transmitter) ||
        (!info->init_codec)) {
        fprintf(stderr, "Start streaming call does not match device type\n");
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
        goto err_dev_close;
    }

    /*get i2s master capabilities */
    ret = devcie_i2s_get_caps(i2s_dev,
                              DEVICE_I2S_ROLE_MASTER,
                              &i2s_test_pcm,
                              &dai);
    /* Check for matching configuration */
    if (ret) {
        fprintf(stderr, "I2S master does support hard coded pcm test configuration\n");
        i2s_supports_master = false;
    } else {
        i2s_supports_master = true;
    }

    /*get codec master capabilities of first dai index */
    ret = device_codec_get_caps(codec_dev,
                                0,
                                DEVICE_CODEC_ROLE_MASTER,
                                &i2s_test_pcm,
                                &dai);
    /* Check for matching configuration */
    if (ret) {
        fprintf(stderr, "Codec master does support hard coded pcm test configuration\n");
        codec_supports_master = false;
    } else {
        codec_supports_master = true;
    }



    if (!((dai.mclk_freq == i2s_test_dai.mclk_freq) &&
         (dai.wclk_polarity | i2s_test_dai.wclk_polarity) &&
         (dai.data_rx_edge | i2s_test_dai.data_rx_edge) &&
         (dai.data_tx_edge | i2s_test_dai.data_tx_edge))) {

        fprintf(stderr, "I2S master does support hard coded dai test configuration\n");
        goto err_dev_close;
    }

    /* master is opposite the slave setting */
    if (!(dai.wclk_change_edge | DEVICE_I2S_EDGE_FALLING)) {
        fprintf(stderr, "Transmitter test mode settings require wclk falling\n");
        goto err_dev_close;
    }
    i2s_test_dai.wclk_change_edge |= DEVICE_I2S_EDGE_FALLING;

    if (info->is_i2s) {
        if(!(dai.protocol | DEVICE_I2S_PROTOCOL_I2S)) {
            fprintf(stderr, "I2S master port does not support I2S protocol\n");
            goto err_dev_close;
        }
        i2s_test_dai.protocol |= DEVICE_I2S_PROTOCOL_I2S;
    } else {
        if(!(dai.protocol | DEVICE_I2S_PROTOCOL_LR_STEREO)) {
            fprintf(stderr, "I2S master port does not support LR protocol\n");
            goto err_dev_close;
        }
        i2s_test_dai.protocol |= DEVICE_I2S_PROTOCOL_LR_STEREO;
    }

    ret = devcie_i2s_set_config(i2s_dev,
                                DEVICE_I2S_ROLE_MASTER,
                                &i2s_test_pcm,
                                &i2s_test_dai);
    if (ret) {
        fprintf(stderr, "set configuration failed: %d\n", ret);
        goto err_dev_close;
    }

    ret = i2s_test_start_transmitter(info, i2s_dev);
    if (ret)
        goto err_dev_close;

    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&i2s_test_done_sem) && (errno == EINTR));

    i2s_test_stop_transmitter(i2s_dev);

err_dev_close:
    device_close(i2s_dev);
    if (codec_dev) {
        device_close(codec_dev);
    }

    return ret;
}
