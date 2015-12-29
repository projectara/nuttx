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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/device.h>
#include <nuttx/device_audio_board.h>

#include <arch/board/audio_board.h>

static int audio_board_get_bundle_count(struct device *dev,
                                        unsigned int *bundle_count)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data) {
        return -EINVAL;
    }

    *bundle_count = init_data->bundle_count;

    return 0;
}

static int audio_board_get_mgmt_cport(struct device *dev,
                                      unsigned int bundle_idx,
                                      uint16_t *mgmt_cport)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data || (bundle_idx >= init_data->bundle_count) ||
        !init_data->bundle) {
        return -EINVAL;
    }

    *mgmt_cport = init_data->bundle[bundle_idx].mgmt_cport;

    return 0;
}

static int audio_board_get_codec_dev_id(struct device *dev,
                                        unsigned int bundle_idx,
                                        unsigned int *codec_dev_id)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data || (bundle_idx >= init_data->bundle_count) ||
        !init_data->bundle) {
        return -EINVAL;
    }

    *codec_dev_id = init_data->bundle[bundle_idx].codec_dev_id;

    return 0;
}

static int audio_board_get_dai_count(struct device *dev,
                                     unsigned int bundle_idx,
                                     unsigned int *dai_count)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data || (bundle_idx >= init_data->bundle_count) ||
        !init_data->bundle) {
        return -EINVAL;
    }

    *dai_count = init_data->bundle[bundle_idx].dai_count;

    return 0;
}

static int audio_board_get_data_cport(struct device *dev,
                                      unsigned int bundle_idx,
                                      unsigned int dai_idx,
                                      uint16_t *data_cport)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data || (bundle_idx >= init_data->bundle_count) ||
        !init_data->bundle || !init_data->bundle[bundle_idx].dai ||
        (dai_idx >= init_data->bundle[bundle_idx].dai_count)) {
        return -EINVAL;
    }

    *data_cport = init_data->bundle[bundle_idx].dai[dai_idx].data_cport;

    return 0;
}

static int audio_board_get_i2s_dev_id(struct device *dev,
                                      unsigned int bundle_idx,
                                      unsigned int dai_idx,
                                      unsigned int *i2s_dev_id)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data || (bundle_idx >= init_data->bundle_count) ||
        !init_data->bundle || !init_data->bundle[bundle_idx].dai ||
        (dai_idx >= init_data->bundle[bundle_idx].dai_count)) {
        return -EINVAL;
    }

    *i2s_dev_id = init_data->bundle[bundle_idx].dai[dai_idx].i2s_dev_id;

    return 0;
}

static int audio_board_dev_probe(struct device *dev)
{
    struct audio_board_init_data *init_data;

    if (!dev) {
        return -EINVAL;
    }

    init_data = device_get_init_data(dev);
    if (!init_data || !init_data->bundle_count) {
        return -EINVAL;
    }

    return 0;
}

static struct device_audio_board_type_ops audio_board_type_ops = {
    .get_bundle_count   = audio_board_get_bundle_count,
    .get_mgmt_cport     = audio_board_get_mgmt_cport,
    .get_codec_dev_id   = audio_board_get_codec_dev_id,
    .get_dai_count      = audio_board_get_dai_count,
    .get_data_cport     = audio_board_get_data_cport,
    .get_i2s_dev_id     = audio_board_get_i2s_dev_id,
};

static struct device_driver_ops audio_board_driver_ops = {
    .probe      = audio_board_dev_probe,
    .type_ops   = &audio_board_type_ops,
};

struct device_driver audio_board_driver = {
    .type = DEVICE_TYPE_AUDIO_BOARD_HW,
    .name = "audio_board",
    .desc = "Board-specific Audio Driver",
    .ops  = &audio_board_driver_ops,
};
