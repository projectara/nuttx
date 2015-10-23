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
typedef uint8_t __u8;
typedef uint16_t __u16;
typedef uint32_t __u32;
typedef uint64_t __u64;

#define GB_AUDIO_PCM_NAME_MAX			32
#define AUDIO_DAI_NAME_MAX			32
#define AUDIO_CONTROL_NAME_MAX			32
#define AUDIO_WIDGET_NAME_MAX			32


/* See SNDRV_PCM_FMTBIT_* in Linux source */
#define GB_AUDIO_PCM_FMT_S8			BIT(0)
#define GB_AUDIO_PCM_FMT_U8			BIT(1)
#define GB_AUDIO_PCM_FMT_S16_LE			BIT(2)
#define GB_AUDIO_PCM_FMT_S16_BE			BIT(3)
#define GB_AUDIO_PCM_FMT_U16_LE			BIT(4)
#define GB_AUDIO_PCM_FMT_U16_BE			BIT(5)
#define GB_AUDIO_PCM_FMT_S24_LE			BIT(6)
#define GB_AUDIO_PCM_FMT_S24_BE			BIT(7)
#define GB_AUDIO_PCM_FMT_U24_LE			BIT(8)
#define GB_AUDIO_PCM_FMT_U24_BE			BIT(9)
#define GB_AUDIO_PCM_FMT_S32_LE			BIT(10)
#define GB_AUDIO_PCM_FMT_S32_BE			BIT(11)
#define GB_AUDIO_PCM_FMT_U32_LE			BIT(12)
#define GB_AUDIO_PCM_FMT_U32_BE			BIT(13)

/* See SNDRV_PCM_RATE_* in Linux source */
#define GB_AUDIO_PCM_RATE_5512			BIT(0)
#define GB_AUDIO_PCM_RATE_8000			BIT(1)
#define GB_AUDIO_PCM_RATE_11025			BIT(2)
#define GB_AUDIO_PCM_RATE_16000			BIT(3)
#define GB_AUDIO_PCM_RATE_22050			BIT(4)
#define GB_AUDIO_PCM_RATE_32000			BIT(5)
#define GB_AUDIO_PCM_RATE_44100			BIT(6)
#define GB_AUDIO_PCM_RATE_48000			BIT(7)
#define GB_AUDIO_PCM_RATE_64000			BIT(8)
#define GB_AUDIO_PCM_RATE_88200			BIT(9)
#define GB_AUDIO_PCM_RATE_96000			BIT(10)
#define GB_AUDIO_PCM_RATE_176400		BIT(11)
#define GB_AUDIO_PCM_RATE_192000		BIT(12)

#define GB_AUDIO_STREAM_TYPE_CAPTURE		0x1
#define GB_AUDIO_STREAM_TYPE_PLAYBACK		0x2

/* See SNDRV_CTL_ELEM_IFACE_* in Linux source */
#define GB_AUDIO_IFACE_CARD			0x1
#define GB_AUDIO_IFACE_HWDEP			0x2
#define GB_AUDIO_IFACE_MIXER			0x3
#define GB_AUDIO_IFACE_PCM			0x4
#define GB_AUDIO_IFACE_RAWMIDI			0x5
#define GB_AUDIO_IFACE_TIMER			0x6
#define GB_AUDIO_IFACE_SEQUENCER		0x7

#define GB_AUDIO_ACCESS_READ			BIT(0)
#define GB_AUDIO_ACCESS_WRITE			BIT(1)

/* enum snd_soc_dapm_type */
#define GB_AUDIO_WIDGET_TYPE_INPUT		0x1
#define GB_AUDIO_WIDGET_TYPE_OUTPUT		0x2
#define GB_AUDIO_WIDGET_TYPE_MUX		0x3
#define GB_AUDIO_WIDGET_TYPE_VIRT_MUX		0x4
#define GB_AUDIO_WIDGET_TYPE_VALUE_MUX		0x5
#define GB_AUDIO_WIDGET_TYPE_MIXER		0x6
#define GB_AUDIO_WIDGET_TYPE_MIXER_NAMED_CTL	0x7
#define GB_AUDIO_WIDGET_TYPE_PGA		0x8
#define GB_AUDIO_WIDGET_TYPE_OUT_DRV		0x9
#define GB_AUDIO_WIDGET_TYPE_ADC		0xa
#define GB_AUDIO_WIDGET_TYPE_DAC		0xb
#define GB_AUDIO_WIDGET_TYPE_MICBIAS		0xc
#define GB_AUDIO_WIDGET_TYPE_MIC		0xd
#define GB_AUDIO_WIDGET_TYPE_HP			0xe
#define GB_AUDIO_WIDGET_TYPE_SPK		0xf
#define GB_AUDIO_WIDGET_TYPE_LINE		0x10
#define GB_AUDIO_WIDGET_TYPE_SWITCH		0x11
#define GB_AUDIO_WIDGET_TYPE_VMID		0x12
#define GB_AUDIO_WIDGET_TYPE_PRE		0x13
#define GB_AUDIO_WIDGET_TYPE_POST		0x14
#define GB_AUDIO_WIDGET_TYPE_SUPPLY		0x15
#define GB_AUDIO_WIDGET_TYPE_REGULATOR_SUPPLY	0x16
#define GB_AUDIO_WIDGET_TYPE_CLOCK_SUPPLY	0x17
#define GB_AUDIO_WIDGET_TYPE_AIF_IN		0x18
#define GB_AUDIO_WIDGET_TYPE_AIF_OUT		0x19
#define GB_AUDIO_WIDGET_TYPE_SIGGEN		0x1a
#define GB_AUDIO_WIDGET_TYPE_DAI_IN		0x1b
#define GB_AUDIO_WIDGET_TYPE_DAI_OUT		0x1c
#define GB_AUDIO_WIDGET_TYPE_DAI_LINK		0x1d

#define GB_AUDIO_WIDGET_STATE_DISABLED		0x01
#define GB_AUDIO_WIDGET_STATE_ENAABLED		0x02

struct gb_audio_pcm {
	__u8	stream_name[GB_AUDIO_PCM_NAME_MAX];
	__u32	formats;	/* GB_AUDIO_PCM_FMT_* */
	__u32	rates;		/* GB_AUDIO_PCM_RATE_* */
	__u8	chan_min;
	__u8	chan_max;
	__u8	sig_bits;	/* number of bits of content */
};

struct gb_audio_dai {
	__u8			name[AUDIO_DAI_NAME_MAX];
	__u16			cport;
	struct gb_audio_pcm	capture;
	struct gb_audio_pcm	playback;
};

struct gb_audio_control {
	__u8	name[AUDIO_CONTROL_NAME_MAX];
	__u8	id;		/* 0-63 */
	__u8	iface;		/* GB_AUDIO_IFACE_* */
	__u8	access_rights;	/* GB_AUDIO_ACCESS_* */
	__u16	dai_cport;
	__u8	stream_type;	/* GB_AUDIO_STREAM_TYPE_* */
	__u32	min;
	__u32	max;
	__u32	step;
	__u32	current_value;
};

struct gb_audio_widget {
	__u8	name[AUDIO_WIDGET_NAME_MAX];
	__u8	id;
	__u8	type;		/* GB_AUDIO_WIDGET_TYPE_* */
	__u8	state;		/* GB_AUDIO_WIDGET_STATE_* */
	__u64	control_ids;	/* bit field of control ids */
};

struct gb_audio_route {
	__u8	source_id;	/* widget id */
	__u8	destination_id;	/* widget id */
	__u8	control_id;	/* 0-63 */
};

struct gb_audio_topology {
	__u8	num_dais;
	__u8	num_controls;
	__u8	num_widgets;
	__u8	num_routes;
	/*
	 * struct gb_audio_dai		dai[num_dais];
	 * struct gb_audio_control	controls[num_controls];
	 * struct gb_audio_widget	widgets[num_widgets];
	 * struct gb_audio_route	routes[num_routes];
	 */
	__u8	data[0];
};

int get_topology_size(__u16 *size);
int get_topology(struct gb_audio_topology *topology);

int get_control(__u8 control_id, __u32 *value);
int set_control(__u8 control_id, __u32 value);

int enable_widget(__u8 widget_id);
int disable_widget(__u8 widget_id);

int get_pcm(__u16 dai_cport, __u32 *format, __u32 *rate, __u8 *channels,
	    __u8 *sig_bits);
int set_pcm(__u16 dai_cport, __u32 format, __u32 rate, __u8 channels,
	    __u8 sig_bits);

/* TX means AP -> Module */
int set_tx_data_size(__u16 dai_cport, __u16 size);
int get_tx_delay(__u16 dai_cport, __u32 *delay);
int start_tx(__u16 dai_cport);
int stop_tx(__u16 dai_cport);

/* RX means AP <- Module */
int set_rx_data_size(__u16 dai_cport, __u16 size);
int get_rx_delay(__u16 dai_cport, __u32 *delay);
int start_rx(__u16 dai_cport);
int stop_rx(__u16 dai_cport);

/**
 * @brief Report event to AP (used by module to report events to the AP)
 */
#define GB_AUDIO_EVENT_UNSPECIFIED			0x1
#define GB_AUDIO_EVENT_JACK_INSERTION			0x2
#define GB_AUDIO_EVENT_JACK_REMOVAL			0x3
#define GB_AUDIO_EVENT_JACK_BUTTON_PRESS		0x4
#define GB_AUDIO_EVENT_HALT				0x5
#define GB_AUDIO_EVENT_INTERNAL_ERROR			0x6
#define GB_AUDIO_EVENT_PROTOCOL_ERROR			0x7
#define GB_AUDIO_EVENT_FAILURE				0x8
#define GB_AUDIO_EVENT_UNDERRUN				0x9
#define GB_AUDIO_EVENT_OVERRUN				0xa
#define GB_AUDIO_EVENT_CLOCKING				0xb
#define GB_AUDIO_EVENT_DATA_LEN				0xc

#define GB_AUDIO_EVENT_BUTTON_PRESS				0x1
#define GB_AUDIO_EVENT_BUTTON_RELEASE				0x2

int report_event(__u8 widget_id, __u8 *event, __u8 *button, __u8 *button_event);





struct gb_audio_request {
	__u64	timestamp;
	__u8	data[0];
};

int send_data(__u16 dai_cport, struct gb_audio_data *audio_data, __u16 size);
