/**
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

#define GB_APBA_AUDIO_PCM_RATE_5512		BIT(0)
#define GB_APBA_AUDIO_PCM_RATE_8000		BIT(1)
#define GB_APBA_AUDIO_PCM_RATE_11025		BIT(2)
#define GB_APBA_AUDIO_PCM_RATE_16000		BIT(3)
#define GB_APBA_AUDIO_PCM_RATE_22050		BIT(4)
#define GB_APBA_AUDIO_PCM_RATE_32000		BIT(5)
#define GB_APBA_AUDIO_PCM_RATE_44100		BIT(6)
#define GB_APBA_AUDIO_PCM_RATE_48000		BIT(7)
#define GB_APBA_AUDIO_PCM_RATE_64000		BIT(8)
#define GB_APBA_AUDIO_PCM_RATE_88200		BIT(9)
#define GB_APBA_AUDIO_PCM_RATE_96000		BIT(10)
#define GB_APBA_AUDIO_PCM_RATE_176400		BIT(11)
#define GB_APBA_AUDIO_PCM_RATE_192000		BIT(12)

#define GB_APBA_AUDIO_PROTOCOL_PCM		BIT(0)
#define GB_APBA_AUDIO_PROTOCOL_I2S		BIT(1)
#define GB_APBA_AUDIO_PROTOCOL_LR_STEREO	BIT(2)

#define GB_APBA_AUDIO_ROLE_MASTER		BIT(0)
#define GB_APBA_AUDIO_ROLE_SLAVE		BIT(1)

#define GB_APBA_AUDIO_POLARITY_NORMAL		BIT(0)
#define GB_APBA_AUDIO_POLARITY_REVERSED		BIT(1)

#define GB_APBA_AUDIO_EDGE_RISING		BIT(0)
#define GB_APBA_AUDIO_EDGE_FALLING		BIT(1)

struct gb_apba_audio_pcm_config {
	uint32_t	rate;
	uint8_t		bits;	/* bits per channel (must be 2 channels) */
	uint8_t		sig_bits;
	uint32_t	ll_protocol; /* defines WCLK, offset, etc. */
	uint32_t	ll_mclk_freq;
	uint8_t		ll_mclk_role;
	uint8_t		ll_bclk_role;
	uint8_t		ll_wclk_role;
	uint8_t		ll_wclk_polarity;
	uint8_t		ll_wclk_change_edge;
	uint8_t		ll_data_rx_edge;
	uint8_t		ll_data_tx_edge;
};

int get_pcm(__u8 i2s_port, struct gb_apba_audio_pcm_config *config);
int set_pcm(__u8 i2s_port, struct gb_apba_audio_pcm_config *config);

int register_cport(__u8 i2s_port, __u16 cport);
int unregister_cport(__u8 i2s_port, __u16 cport);

/* Tx means AP -> Module */
int set_tx_data_size(__u8 i2s_port, __u16 size);
int get_tx_delay(__u8 i2s_port, __u32 *delay);

int start_tx(__u8 i2s_port, __u16 cport, __u64 timestamp);
int stop_tx(__u8 i2s_port, __u16 cport);

/* Rx means AP <- Module */
int set_rx_data_size(__u8 i2s_port, __u16 size);
int get_rx_delay(__u8 i2s_port, __u32 *delay);

int start_rx(__u8 i2s_port, __u16 cport);
int stop_rx(__u8 i2s_port, __u16 cport);
