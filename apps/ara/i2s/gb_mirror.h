/*
 * Copyright (c) 2016 Google, Inc.
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

#ifndef __GB_MIRROR_H__
#define __GB_MIRROR_H__

/* structures from the original */
struct gb_audio_info { /* One per audio Bundle */
    bool                    initialized;
    uint16_t                mgmt_cport;
    struct device           *codec_dev;
    struct list_head        dai_list;   /* list of gb_audio_dai_info structs */
    struct list_head        list;       /* next gb_audio_info struct */
};

struct gb_audio_dai_info {
    uint32_t                flags;
    uint16_t                data_cport;
    unsigned int            dai_idx;
    struct device           *i2s_dev;
    uint32_t                format;
    uint32_t                rate;
    uint8_t                 channels;
    uint8_t                 sig_bits;
    unsigned int            sample_size;
    unsigned int            sample_freq;

    struct ring_buf         *tx_rb;
    unsigned int            tx_data_size;
    unsigned int            tx_samples_per_msg;
    uint8_t                 *tx_dummy_data;

    struct ring_buf         *rx_rb;
    unsigned int            rx_data_size;
    unsigned int            rx_samples_per_msg;

    struct gb_audio_info    *info;      /* parent gb_audio_info struct */
    struct list_head        list;       /* next gb_audio_dai_info struct */
};

int gb_audio_config_connection(struct gb_audio_dai_info *dai,
                               uint32_t format, uint32_t rate,
                               uint8_t channels, uint8_t sig_bits);

#endif /* __GB_MIRROR_H__ */
