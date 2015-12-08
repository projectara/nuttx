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
#include <stdio.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c.h>
#include <nuttx/list.h>
#include <nuttx/device_audio_board.h>
#include "audcodec.h"
#include "NXP9890.h"

// ignore unused warning message for current stage
#pragma GCC diagnostic ignored "-Wunused-function"

#define NXP9890_CODEC_NAME   "NXP9890"
/* One bundle count for each codec on the audio module */
#define CODEC_BUNDLE        0
#define CODEC_DEVICE_FLAG_PROBE           BIT(0)  /* device probed */
#define CODEC_DEVICE_FLAG_OPEN            BIT(1)  /* device opened */
#define CODEC_DEVICE_FLAG_CONFIG          BIT(2)  /* device configured */
#define CODEC_DEVICE_FLAG_TX_START        BIT(3)  /* device tx started */
#define CODEC_DEVICE_FLAG_RX_START        BIT(4)  /* device rx started */
#define CODEC_DEVICE_FLAG_CLOSE           BIT(5)  /* device closed */

#undef VERBOSE_MSG

static int NXP9890_speaker_event(struct device *dev, uint8_t widget_id,
                                uint8_t event);
int NXP9890_playback_vol_set(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);
int NXP9890_playback_vol_get(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);

static struct device *codec_dev = NULL;

/**
 * audio control linklist node
 */
struct control_node {
    /* linklist head */
    struct list_head list;
    /* control index of widget */
    int index;
    /* audio control */
    struct audio_control *control;
    /* widget */
    struct audio_widget *parent_widget;
};

/**
 * NXP9890 codec register structure
 */
struct NXP9890_reg {
    /** register address */
    uint8_t reg;
    /** register value */
    uint16_t val;
};


/**
 * NXP9890 private information
 */
struct NXP9890_info {
    /** Driver model representation of the device */
    struct device *dev;
    /** i2c device handle */
    struct i2c_dev_s *i2c;
    /** codec name */
    uint8_t name[AUDIO_CODEC_NAME_MAX];
    /** device state */
    int state;

    /** enable TDM mode */
    int tdm_en;

    /** NXP9890 codec initialization array */
    struct NXP9890_reg *init_regs;
    /** number of codec initialization array */
    int num_regs;

    /** DAI device array */
    struct audio_dai *dais;
    /** number of DAI device */
    int num_dais;
    /** audio control array */
    struct audio_control *controls;
    /** number of audio control */
    int num_controls;
    /** audio widget array */
    struct audio_widget *widgets;
    /** number of audio widget */
    int num_widgets;
    /** audio routing table */
    audio_route *routes;
    /** number of audio routing */
    int num_routes;

    struct list_head control_list;

    /** rx delay count */
    uint32_t rx_delay;
    /** rx callback event */
    device_codec_event_callback rx_callback;
    /** rx callback event argument */
    void* rx_callback_arg;
    /** tx delay count */
    uint32_t tx_delay;
    /** tx callback event */
    device_codec_event_callback tx_callback;
    /** tx callback event argument */
    void* tx_callback_arg;
    /** jack callback event */
    device_codec_jack_event_callback jack_event_callback;
    /** jack callback event argument */
    void* jack_event_callback_arg;
    /** button callback event */
    device_codec_button_event_callback button_event_callback;
    /** button callback event argument */
    void* button_event_callback_arg;

    /** codec hardware access function for read */
    uint32_t (*codec_read)(uint32_t reg, uint32_t *value);
    /** codec hardware access function for write */
    uint32_t (*codec_write)(uint32_t reg, uint32_t value);
};

/**
 * codec register initialization table
 */
struct NXP9890_reg NXP9890_init_regs[] = {
    { NXP9890_DUMMY_VOL_MUTE_REG, 0x88 },
    { NXP9890_DUMMY_POWER_REG, 0x00 }
};

/**
 * DAI device table
 */
struct audio_dai NXP9890_dais[] = {
    {
        .dai = {
            .name = "NXP9890-aif1",
            .data_cport = 0,
            .capture = {
                .stream_name = "AIF1 Capture",
                .formats = NXP9890_FORMATS,
                .rates = NXP9890_STEREO_RATES,
                .chan_min = 1,
                .chan_max = 2,
            },
            .playback = {
                .stream_name = "AIF1 Playback",
                .formats = NXP9890_FORMATS,
                .rates = NXP9890_STEREO_RATES,
                .chan_min = 1,
                .chan_max = 2,
            },
        },
        /* clear if mclk master not supported */
        .m_caps = {
            .protocol = 0,
            .wclk_polarity = 0,
            .wclk_change_edge = 0,
            .data_rx_edge = 0,
            .data_tx_edge = 0,
        },
        .s_caps = {
            .protocol = DEVICE_CODEC_PROTOCOL_I2S,
            .wclk_polarity = DEVICE_CODEC_POLARITY_NORMAL,
            .wclk_change_edge = DEVICE_CODEC_EDGE_RISING,
            .data_rx_edge = DEVICE_CODEC_EDGE_RISING,
            .data_tx_edge = DEVICE_CODEC_EDGE_FALLING,
        },
    },
};

/**
 * audio control id
 */
enum {
    NXP9890_CTL_PLAYBACK_MUTE,
    NXP9890_CTL_PLAYBACK_VOL,
    NXP9890_CTL_SPKAMP_SWITCH,
    NXP9890_CTL_MAX
};

/**
 * audio widget id
 */
enum {
    NXP9890_WIDGET_SPK_AMP_SWITCH,
    NXP9890_WIDGET_MAX
};


/**
 * audio control list
 */
struct audio_control NXP9890_controls[] = {
    AUDCTL_BITS("Playback Mute", RT5647_CTL_PLAYBACK_MUTE, MIXER,
            NXP9890_DUMMY_VOL_MUTE_REG, NXP9890_L_MUTE_SFT, NXP9890_R_MUTE_SFT, 1),
    AUDCTL_BITSVE("Playback Volume", RT5647_CTL_PLAYBACK_VOL, MIXER,
            NXP9890_DUMMY_VOL_MUTE_REG, NXP9890_L_VOL_SFT, NXP9890_R_VOL_SFT,
                 0, 8, 0xAF, 0x7F, 0, NXP9890_playback_vol_get,
                 NXP9890_playback_vol_set)
};

/**
 * audio control list
 */
struct audio_control NXP9890_spk_amp_switch[] = {
           AUDCTL_DUMMY("Switch", NXP9890_CTL_SPKAMP_SWITCH, MIXER,
                        DUMMY_REG, DUMMY_BIT_00, 0),
};

/**
 * audio widget table
 */
struct audio_widget NXP9890_widgets[] = {
    WIDGET_E("SPK Amp", NXP9890_WIDGET_SPK_AMP_SWITCH, SWITCH,
             NXP9890_spk_amp_switch, ARRAY_SIZE(NXP9890_spk_amp_switch),
             NOPWRCTL, 0, 0, NXP9890_speaker_event),
};

/**
 * audio route table
 */
audio_route NXP9890_routes[] = {

};

/**
 * @brief get codec driver device handle
 *
 * @return codec driver handle on success, NULL on error
 */
struct device* get_codec_dev()
{
    return codec_dev;
}

/**
 * @brief get codec-specific register read function
 *
 * @param dev - pointer to structure of device data
 * @return read() function pointer on success, NULL on error
 */
void* get_codec_read_func(struct device *dev)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return NULL;
    }
    info = device_get_private(dev);
    return (void*)info->codec_read;
}

/**
 * @brief get codec-specific register write function
 *
 * @param dev - pointer to structure of device data
 * @return write() function pointer on success, NULL on error
 */
void* get_codec_write_func(struct device *dev)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return NULL;
    }
    info = device_get_private(dev);
    return (void*)info->codec_write;
}

/**
 * @brief read data from codec register
 *
 * @param reg - rt5645 codec register
 * @param value - data buffer of read
 * @return 0 on success, negative errno on error
 */
static uint32_t NXP9890_audcodec_hw_read(uint32_t reg, uint32_t *value)
{
    struct device *dev = get_codec_dev();
    struct NXP9890_info *info = NULL;
    uint8_t cmd = 0;
    uint16_t data = 0;

    /* NXP9890 i2c read format :
     * [SA + W] + [DA] + [SA + R] + [DATA_HIGH] + [DATA_LOW]
     */
    struct i2c_msg_s msg[] = {
        {
            .addr = NXP9890_I2C_ADDR,
            .flags = 0,
            .buffer = &cmd,
            .length = 1,
        },
        {
            .addr = NXP9890_I2C_ADDR,
            .flags = I2C_M_READ,
            .buffer = (uint8_t*)&data,
            .length = 2,
        }
    };

    if (!dev || !device_get_private(dev) || !value) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    if (!info->i2c) {
        return -EINVAL;
    }

    cmd = (uint8_t)reg;

    if (I2C_TRANSFER(info->i2c, msg, 2)) {
        return -EIO;
    }

    *value = (uint32_t) (data & 0xFF) << 8 | ((data >> 8) & 0xFF);
#ifdef VERBOSE_MSG
    printf("I2C-R %02X %04X\n", reg, *value);
#endif
    return 0;
}

/**
 * @brief write data to codec register
 *
 * @param reg - rt5645 codec register
 * @param value - register value that we want to write
 * @return 0 on success, negative errno on error
 */
static uint32_t NXP9890_audcodec_hw_write(uint32_t reg, uint32_t value)
{
    struct device *dev = get_codec_dev();
    struct NXP9890_info *info = NULL;
    uint8_t cmd[3] = {0x00, 0x00, 0x00};

    /* NXP9890 i2c write format :
     * [SA + W] + [DA] + [DATA_HIGH] + [DATA_LOW]
     */

    struct i2c_msg_s msg[] = {
        {
            .addr = NXP9890_I2C_ADDR,
            .flags = 0,
            .buffer = cmd,
            .length = 3,
        },
    };

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    if (!info->i2c) {
        return -EINVAL;
    }

    cmd[0] = (uint8_t)reg;
    cmd[1] = (uint8_t)((value >> 8) & 0xFF);
    cmd[2] = (uint8_t)(value & 0xFF);

    if (I2C_TRANSFER(info->i2c, msg, 1)) {
        return -EIO;
    }
#ifdef VERBOSE_MSG
    printf("I2C-W %02X %04X\n", reg, value);
#endif
    return 0;
}


#ifdef VERBOSE_MSG
/* NXP9890 register map */
const uint8_t NXP9890_reg_map[] = {
    0x10,0x20
};

/**
 * @brief dump all NXP9890 register
 */
static void NXP9890_dump_register(void)
{
    int i = 0, j = 0, k = 0;
    uint32_t value = 0;
    uint8_t bstr[20];

    printf("\nDump NXP9890 register:\n");
    for (i = 0; i < ARRAY_SIZE(NXP9890_reg_map); i++) {
        if (NXP9890_audcodec_hw_read(NXP9890_reg_map[i], &value)) {
            continue;
        }
        bstr[19] = 0;
        for (j = 0, k = 0; j < 16; j++) {
            bstr[k++] = (value & (1 << (15 - j)))? 0x31: 0x30;
            if (j == 3 || j == 7 || j == 11) {
                bstr[k++] = 0x20;
            }
        }
        printf("REG[%02X] = %04X, %s\n", NXP9890_reg_map[i], value, bstr);
    }
}
#endif

static int get_data_cport(unsigned int bundle_index, unsigned int dai_index, uint16_t *data_cport)
{
    struct device *dev;
    int ret;

    dev = device_open(DEVICE_TYPE_AUDIO_BOARD_HW, 0);
    if (!dev) {
        return ret = -EIO;
    }

    ret = device_audio_board_get_data_cport(dev, CODEC_BUNDLE, dai_index, data_cport);

    device_close(dev);

    return ret;
}


int NXP9890_playback_vol_get(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value)
{
    int ret = 0, volr = 0, voll = 0, regmax = 0, ctlmax = 0;
    struct bitctl *ctl = NULL;
    struct gb_audio_ctl_elem_info   *info;

    ret = audcodec_value_get(control, value);
    if (ret) {
        return ret;
    }
    ctl = control->priv;
    info = &control->control.info;

    regmax = ctl->max;
    ctlmax = info->value.integer.max;

    voll = value->value.integer_value[0];
    volr = value->value.integer_value[1];

    value->value.integer_value[0] = voll * ctlmax / regmax;
    value->value.integer_value[1] = volr * ctlmax / regmax;
    return 0;
}

int NXP9890_playback_vol_set(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value)
{
    int volr = 0, voll = 0, regmax = 0, ctlmax = 0;
    struct bitctl *ctl = NULL;
    struct gb_audio_ctl_elem_info   *info;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;
    info = &control->control.info;

    regmax = ctl->max;
    ctlmax = info->value.integer.max;

    voll = value->value.integer_value[0];
    volr = value->value.integer_value[1];

    voll = (voll > ctlmax)? ctlmax : voll;
    volr = (volr > ctlmax)? ctlmax : volr;

    value->value.integer_value[0] = voll * regmax / ctlmax;
    value->value.integer_value[1] = volr * regmax / ctlmax;
    return audcodec_value_set(control, value);
}


/**
 * @brief get audio topology data size
 *
 * @param dev - pointer to structure of device data
 * @param size - size of audio topology data
 * @return 0 on success, negative errno on error
 */
static int NXP9890_get_topology_size(struct device *dev, uint16_t *size)
{
    struct NXP9890_info *info = NULL;
    struct audio_widget *widgets = NULL;
    int tpg_size = 0, i = 0;

    if (!dev || !device_get_private(dev) || !size) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    tpg_size = sizeof(struct gb_audio_topology);
    tpg_size += info->num_dais * sizeof(struct gb_audio_dai);
    tpg_size += info->num_controls * sizeof(struct gb_audio_control);
    tpg_size += info->num_widgets * sizeof(struct gb_audio_widget);

    widgets = info->widgets;
    for (i = 0; i < info->num_widgets; i++) {
        if (widgets[i].num_controls) {
            tpg_size += widgets[i].num_controls *
                        sizeof(struct gb_audio_control);
        }
    }
    tpg_size += info->num_routes * sizeof(struct gb_audio_route);
    *size = tpg_size;
    return 0;
}

/**
 * @brief get audio topology binary data
 *
 * @param dev - pointer to structure of device data
 * @param topology - audio topology data
 * @return 0 on success, negative errno on error
 */
static int NXP9890_get_topology(struct device *dev,
                               struct gb_audio_topology *topology)
{
    struct NXP9890_info *info = NULL;
    struct audio_widget *widgets = NULL;
    struct audio_control *controls = NULL;
    int i = 0, j = 0;
    int dsize = 0, csize = 0, wsize = 0, rsize = 0;
    int ret;
    uint16_t data_cport;
    uint8_t *data = NULL;

    if (!dev || !device_get_private(dev) || !topology) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!info->dais || !info->controls || !info->widgets || !info->routes) {
        return -EINVAL;
    }

    dsize = sizeof(struct gb_audio_dai);
    csize = sizeof(struct gb_audio_control);
    wsize = sizeof(struct gb_audio_widget);
    rsize = sizeof(struct gb_audio_route);

    topology->num_dais = info->num_dais;
    topology->num_controls = info->num_controls;
    topology->num_widgets = info->num_widgets;
    topology->num_routes = info->num_routes;

    topology->size_dais = info->num_dais * dsize;
    topology->size_controls = info->num_controls * csize;
    topology->size_widgets = info->num_widgets * wsize;
    topology->size_routes = info->num_routes * rsize;

    data = topology->data;
    /* fill dai object */
    for (i = 0; i < info->num_dais; i++) {
        ret = get_data_cport(CODEC_BUNDLE, i, &data_cport);
        if (ret) {
            return ret;
        }

        info->dais[i].dai.data_cport = data_cport,

        memcpy(data, &info->dais[i].dai, dsize);

        data += dsize;
    }

    /* fill audio control object */
    for (i = 0; i < info->num_controls; i++) {
        memcpy(data, &info->controls[i].control, csize);
        data += csize;
    }

    /* fill audio widget object */
    widgets = info->widgets;
    for (i = 0; i < info->num_widgets; i++) {
        memcpy(data, &widgets[i].widget, wsize);
        data += wsize;
        /* fill widget's control objects */
        controls = widgets[i].controls;

        if (controls) {
            for (j = 0; j < widgets[i].num_controls; j++) {
                memcpy(data, &controls[j].control, csize);
                topology->size_widgets += csize;
                data += csize;;
            }
        }
    }

    /* fill audio route object */
    for (i = 0; i < info->num_routes; i++) {
        memcpy(data, &info->routes[i], rsize);
        data += rsize;
    }
    return 0;
}

/**
 * @brief convert pcm rate setting to frequency
 *
 * @param rate - pcm rate
 * @return frequency on success, negative errno on error
 */
static int NXP9890_rate_to_freq(uint32_t rate)
{
    uint32_t freq = 0;

    if (!ONE_BIT_IS_SET(rate)) {
        return -EINVAL;
    }

    switch (rate) {
    case GB_AUDIO_PCM_RATE_5512:
        freq = 5512;
        break;
    case GB_AUDIO_PCM_RATE_8000:
        freq = 8000;
        break;
    case GB_AUDIO_PCM_RATE_11025:
        freq = 11025;
        break;
    case GB_AUDIO_PCM_RATE_16000:
        freq = 16000;
        break;
    case GB_AUDIO_PCM_RATE_22050:
        freq = 22050;
        break;
    case GB_AUDIO_PCM_RATE_32000:
        freq = 32000;
        break;
    case GB_AUDIO_PCM_RATE_44100:
        freq = 44100;
        break;
    case GB_AUDIO_PCM_RATE_48000:
        freq = 48000;
        break;
    case GB_AUDIO_PCM_RATE_64000:
        freq = 64000;
        break;
    case GB_AUDIO_PCM_RATE_88200:
        freq = 88200;
        break;
    case GB_AUDIO_PCM_RATE_96000:
        freq = 96000;
        break;
    case GB_AUDIO_PCM_RATE_176400:
        freq = 176400;
        break;
    case GB_AUDIO_PCM_RATE_192000:
        freq = 192000;
        break;
    default:
        return -EINVAL;
    }
    return freq;
}

/**
 * @brief convert pcm format setting to bit number
 *
 * @param fmtbit - pcm format
 * @return bit number on success, negative errno on error
 */
static int NXP9890_fmtbit_to_bitnum(uint32_t fmtbit)
{
    uint32_t bits = 0;

    if (!ONE_BIT_IS_SET(fmtbit)) {
        return -EINVAL;
    }

    switch (fmtbit) {
    case GB_AUDIO_PCM_FMT_S8:
    case GB_AUDIO_PCM_FMT_U8:
        bits = 8;
        break;
    case GB_AUDIO_PCM_FMT_S16_LE:
    case GB_AUDIO_PCM_FMT_S16_BE:
    case GB_AUDIO_PCM_FMT_U16_LE:
    case GB_AUDIO_PCM_FMT_U16_BE:
        bits = 16;
        break;
    case GB_AUDIO_PCM_FMT_S24_LE:
    case GB_AUDIO_PCM_FMT_S24_BE:
    case GB_AUDIO_PCM_FMT_U24_LE:
    case GB_AUDIO_PCM_FMT_U24_BE:
        bits = 24;
        break;
    case GB_AUDIO_PCM_FMT_S32_LE:
    case GB_AUDIO_PCM_FMT_S32_BE:
    case GB_AUDIO_PCM_FMT_U32_LE:
    case GB_AUDIO_PCM_FMT_U32_BE:
        bits = 32;
        break;
    default:
        return -EINVAL;
    }
    return bits;
}

/**
 * @brief get audio dai setting and capability
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - dai index
 * @param clk_role - codec mode, master or slave
 * @param pcm - audio pcm setting
 * @param dai - audio dai setting
 * @return 0 on success, negative errno on error
 */
static int NXP9890_get_caps(struct device *dev, unsigned int dai_idx,
                           uint8_t clk_role, struct device_codec_pcm *pcm,
                           struct device_codec_dai *dai)
{
    struct NXP9890_info *info = NULL;
    struct gb_audio_pcm *pbpcm = NULL;

    if (!dev || !device_get_private(dev) || !pcm || !dai) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (dai_idx >= info->num_dais) {
        return -EINVAL;
    }
    pbpcm = &info->dais[dai_idx].dai.playback;

    /* check pcm capability */
    if (!ONE_BIT_IS_SET(pcm->rate) || !(pbpcm->rates & pcm->rate)) {
        return -EINVAL;
    }
    if (!ONE_BIT_IS_SET(pcm->format) || !(pbpcm->formats & pcm->format)) {
        return -EINVAL;
    }
    if (pcm->channels > pbpcm->chan_max || pcm->channels < pbpcm->chan_min) {
        return -EINVAL;
    }

    if (clk_role != DEVICE_CODEC_ROLE_SLAVE) {
        /* In current audio module, we only supported slave mode. */
        return -EOPNOTSUPP;
    } else {
        /* query for slave
          Test if mclk will work
          return full set of slave hardware capabilities
        */
        uint32_t mclk_temp = dai->mclk_freq;

        memcpy(dai, &info->dais[dai_idx].s_caps, sizeof(struct device_codec_dai));

        /* restore the value overwritten */
        dai->mclk_freq = mclk_temp;

        /* TODO check if the pcm->mclk value will work in slave mode*/
    }

    return 0;
}



/**
 * @brief set audio dai setting
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - dai index
 * @param clk_role - codec mode, master or slave
 * @param pcm - audio pcm setting
 * @param dai - audio dai setting
 * @return 0 on success, negative errno on error
 */
static int NXP9890_set_config(struct device *dev, unsigned int dai_idx,
                             uint8_t clk_role, struct device_codec_pcm *pcm,
                             struct device_codec_dai *dai)
{
    struct NXP9890_info *info = NULL;
    struct gb_audio_pcm *pbpcm = NULL;
    struct pll_code code;
    int sysclk = 0, ratefreq = 0, numbits = 0, ret = 0;
    uint32_t value = 0, mask = 0, format = 0;
    uint32_t tdm1 = 0, tdm2 = 0, tdm3 = 0;

    if (!dev || !device_get_private(dev) || !pcm || !dai) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (dai_idx >= info->num_dais) {
        return -EINVAL;
    }
    pbpcm = &info->dais[dai_idx].dai.playback;

    /* check pcm capability */
    if (!ONE_BIT_IS_SET(pcm->rate) || !(pbpcm->rates & pcm->rate)) {
        return -EINVAL;
    }
    if (!ONE_BIT_IS_SET(pcm->format) || !(pbpcm->formats & pcm->format)) {
        return -EINVAL;
    }
    if (pcm->channels > pbpcm->chan_max || pcm->channels < pbpcm->chan_min) {
        return -EINVAL;
    }

    if (clk_role != DEVICE_CODEC_ROLE_SLAVE) {
        /* In current audio module, we only supported slave mode. */
        return -EINVAL;
    }
    // check clock setting
    ratefreq = NXP9890_rate_to_freq(pcm->rate);
    numbits = NXP9890_fmtbit_to_bitnum(pcm->format);

    if (ratefreq <= 0 || numbits <= 0 || numbits < pcm->sig_bits) {
        return -EINVAL;
    }

    /* verify settings work and apply settings */


    /* save config to dai[dai_idx] structure */
    info->dais[dai_idx].clk_role = clk_role;
    memcpy(&info->dais[dai_idx].pcm_config, pcm,
           sizeof(struct device_codec_pcm));
    memcpy(&info->dais[dai_idx].dai_config, dai,
           sizeof(struct device_codec_dai));

    info->state |= CODEC_DEVICE_FLAG_CONFIG;
    return 0;
}

/**
 * @brief get audio control value
 *
 * @param dev - pointer to structure of device data
 * @param control_id - control id
 * @param index - index of control (for mux control)
        return -EINVAL;
    }
 * @param value - audio control return value
 * @return 0 on success, negative errno on error
 */
static int NXP9890_get_control(struct device *dev, uint8_t control_id,
                          uint8_t index, struct gb_audio_ctl_elem_value *value)
{
    struct NXP9890_info *info = NULL;
    struct list_head *iter, *iter_next;
    struct control_node *ctl = NULL;
    struct audio_control *aud_ctl = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    /* find an audio control by control id */
    list_foreach_safe(&info->control_list, iter, iter_next) {
        ctl = list_entry(iter, struct control_node, list);
        aud_ctl = ctl->control;
        if ((aud_ctl->control.id == control_id) && (ctl->index == index)) {
            if (aud_ctl->get) {
                /* perform get() to get control value or codec register */
                return aud_ctl->get(aud_ctl, value);
            }
        }
    }
    return -EINVAL;
}

/**
 * @brief set audio control
 *
 * @param dev - pointer to structure of device data
 * @param control_id - control id
 * @param index - index of control (for mux control)
 * @param value - audio control value
 * @return 0 on success, negative errno on error
 */
static int NXP9890_set_control(struct device *dev, uint8_t control_id,
                          uint8_t index, struct gb_audio_ctl_elem_value *value)
{
    struct NXP9890_info *info = NULL;
    struct list_head *iter, *iter_next;
    struct control_node *ctl = NULL;
    struct audio_control *aud_ctl = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    /* find an audio control by control id */
    list_foreach_safe(&info->control_list, iter, iter_next) {
        ctl = list_entry(iter, struct control_node, list);
        aud_ctl = ctl->control;
        if ((aud_ctl->control.id == control_id) && (ctl->index == index)) {
            if (aud_ctl->set) {
                /* perform set() to write control value or codec register */
                return aud_ctl->set(aud_ctl, value);
            }
        }
    }
    return -EINVAL;
}

/**
 * @brief enable widget
 *
 * @param dev - pointer to structure of device data
 * @param widget_id - widget id
 * @return 0 on success, negative errno on error
 */
static int NXP9890_enable_widget(struct device *dev, uint8_t widget_id)
{
    struct NXP9890_info *info = NULL;
    struct audio_widget *widget = NULL;
    int i = 0;
    uint32_t data = 0, mask = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    widget = info->widgets;

    for (i = 0; i < info->num_widgets; i++) {
        if (widget->widget.id == widget_id) {
            if (widget->widget.state == GB_AUDIO_WIDGET_STATE_ENAABLED) {
                /* widget has enabled */
                return 0;
            }
            if (widget->event) {
                widget->event(dev, widget_id, WIDGET_EVENT_PRE_PWRUP);
            }
            if (widget->reg != NOPWRCTL) {
                /* turn on the widget */
                if (audcodec_read(widget->reg, &data)) {
                    return -EIO;
                }
                mask = 1 << widget->shift;
                data = (data & ~mask) | ((widget->inv)? 0 : mask);
                if (audcodec_write(widget->reg, data)) {
                    return -EIO;
                }
            }
            widget->widget.state = GB_AUDIO_WIDGET_STATE_ENAABLED;
            if (widget->event) {
                widget->event(dev, widget_id, WIDGET_EVENT_POST_PWRUP);
            }
            return 0;
        }
        widget++;
    }
    return -EINVAL;
}

/**
 * @brief disable widget
 *
 * @param dev - pointer to structure of device data
 * @param widget_id - widget id
 * @return 0 on success, negative errno on error
 */
static int NXP9890_disable_widget(struct device *dev, uint8_t widget_id)
{
    struct NXP9890_info *info = NULL;
    struct audio_widget *widget = NULL;
    int i = 0;
    uint32_t data = 0, mask = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    widget = info->widgets;

    for (i = 0; i < info->num_widgets; i++) {
        if (widget->widget.id == widget_id) {
            if (widget->widget.state == GB_AUDIO_WIDGET_STATE_DISABLED) {
                /* widget has disabled */
                return 0;
            }
            if (widget->event) {
                widget->event(dev, widget_id, WIDGET_EVENT_PRE_PWRDOWN);
            }
            if (widget->reg != NOPWRCTL) {
                /* turn off the widget */
                if (audcodec_read(widget->reg, &data)) {
                    return -EIO;
                }
                mask = 1 << widget->shift;
                data = (data & ~mask) | ((widget->inv)? mask : 0);
                if (audcodec_write(widget->reg, data)) {
                    return -EIO;
                }
            }
            widget->widget.state = GB_AUDIO_WIDGET_STATE_DISABLED;
            if (widget->event) {
                widget->event(dev, widget_id, WIDGET_EVENT_POST_PWRDOWN);
            }
            return 0;
        }
        widget++;
    }
    return -EINVAL;
}

/**
 * @brief get tx delay count
 *
 * @param dev - pointer to structure of device data
 * @param delay - buffer for get delay count
 * @return 0 on success, negative errno on error
 */
static int NXP9890_get_tx_delay(struct device *dev, uint32_t *delay)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev) || !delay) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    *delay = info->tx_delay;
    return 0;
}

/**
 * @brief start to transfer audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int NXP9890_start_tx(struct device *dev, uint32_t dai_idx)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & CODEC_DEVICE_FLAG_CONFIG)) {
        /* device isn't configured. */
        return -EIO;
    }
    info->state |= CODEC_DEVICE_FLAG_TX_START;
    return 0;
}

/**
 * @brief stop to transfer audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int NXP9890_stop_tx(struct device *dev, uint32_t dai_idx)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & CODEC_DEVICE_FLAG_TX_START)) {
        /* device isn't start to transfer data. */
        return -EIO;
    }
    info->state &= ~CODEC_DEVICE_FLAG_TX_START;
    return 0;
}

/**
 * @brief register tx callback event
 *
 * @param dev - pointer to structure of device data
 * @param callback - callback function for notify tx event
 * @param arg - argument for callback event
 * @return 0 on success, negative errno on error
 */
static int NXP9890_register_tx_callback(struct device *dev,
                                       device_codec_event_callback callback,
                                       void *arg)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    info->tx_callback = callback;
    info->tx_callback_arg = arg;
    return 0;
}

/**
 * @brief get rx delay count
 *
 * @param dev - pointer to structure of device data
 * @param delay - buffer for get delay count
 * @return 0 on success, negative errno on error
 */
static int NXP9890_get_rx_delay(struct device *dev, uint32_t *delay)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev) || !delay) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    *delay = info->rx_delay;
    return 0;
}

/**
 * @brief start to receive audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int NXP9890_start_rx(struct device *dev, uint32_t dai_idx)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & CODEC_DEVICE_FLAG_CONFIG)) {
        /* device isn't configured. */
        return -EIO;
    }

    if (dai_idx >= info->num_dais) {
        return -EINVAL;
    }

    /* ToDo if needed,
     * add code here that enables codec to recive data
     * the code should not interfere with any controls
     */
    }

    info->state |= CODEC_DEVICE_FLAG_RX_START;

#ifdef VERBOSE_MSG
    NXP9890_dump_register();
#endif
    return 0;
}

/**
 * @brief stop to receive audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int NXP9890_stop_rx(struct device *dev, uint32_t dai_idx)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & CODEC_DEVICE_FLAG_RX_START)) {
        /* device isn't start to receive data. */
        return -EIO;
    }

    if (dai_idx >= info->num_dais) {
        return -EINVAL;
    }

    /* ToDo if needed,
     * add code here that disables codec to receive data
     * the code should not interfere with any controls
     */

    info->state &= ~CODEC_DEVICE_FLAG_RX_START;
    return 0;
}

/**
 * @brief register rx callback event
 *
 * @param dev - pointer to structure of device data
 * @param callback - callback function for notify rx event
 * @param arg - argument for callback event
 * @return 0 on success, negative errno on error
 */
static int NXP9890_register_rx_callback(struct device *dev,
                                       device_codec_event_callback callback,
                                       void *arg)
{
    struct NXP9890_info *info = NULL;

    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    info->rx_callback = callback;
    info->rx_callback_arg = arg;
    return 0;
}

/**
 * @brief register speaker amp callback event
 *
 * @param dev - pointer to structure of device data
 * @param callback - callback function for notify jack event
 * @param arg - argument for callback event
 * @return 0 on success, negative errno on error
 */
static int NXP9890_speaker_event(struct device *dev, uint8_t widget_id,
                                uint8_t event)
{
    uint32_t mask = 0;
    switch (event) {
    case WIDGET_EVENT_POST_PWRUP:
        /* turn on Class-D power */
        mask = 1 << NXP9890_PWR1_CLSD_R_EN | 1 << NXP9890_PWR1_CLSD_L_EN | \
               1 << NXP9890_PWR1_CLSD_EN;
        audcodec_update(NXP9890_PWR_MGT_1, mask, mask);

        break;
    case WIDGET_EVENT_PRE_PWRDOWN:
        /* turn off Class-D power */
        audcodec_update(NXP9890_PWR_MGT_1, 0, mask);
        break;
    }
    return 0;
}

/**
 * @brief Open audio codec device
 *
 * This function is called when the caller is preparing to use this device
 * driver. This function should be called after probe () function and need to
 * check whether the driver already open or not. If driver was opened, it needs
 * to return an error code to the caller to notify the driver was opened.
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int NXP9890_audcodec_open(struct device *dev)
{
    struct NXP9890_info *info = NULL;
    int ret = 0, i = 0;
    uint32_t id = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!(info->state & CODEC_DEVICE_FLAG_PROBE)) {
        return -EIO;
    }

    if (info->state & CODEC_DEVICE_FLAG_OPEN) {
        /* device has been opened, return error */
        return -EBUSY;
    }

    /* ToDo verify codec id */
    if (audcodec_read(NXP9890_VENDOR_ID, &id) || (id != NXP9890_DEFAULT_VID)) {
        /* can't read codec register or vendor id isn't correct */
        return -EIO;
    }

    /* ToDo add code initialize the codec hardware */

    /* Write out initial general codec register settings */
    for (i = 0; i < info->num_regs; i++) {
        audcodec_write(info->init_regs[i].reg , info->init_regs[i].val);
    }

    return ret;
}

/**
 * @brief Close audio codec device
 *
 * This function is called when the caller no longer using this driver. It
 * should release or close all resources that allocated by the open() function.
 * This function should be called after the open() function. If the device
 * is not opened yet, this function should return without any operations.
 *
 * @param dev - pointer to structure of device data
 */
static void NXP9890_audcodec_close(struct device *dev)
{
    struct NXP9890_info *info = NULL;
    struct audio_widget *widget = NULL;
    int i = 0;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (!(info->state & CODEC_DEVICE_FLAG_OPEN)) {
        /* device isn't opened. */
        return;
    }

    /* check device state */
    if (info->state & CODEC_DEVICE_FLAG_CONFIG) {
        if (info->state & CODEC_DEVICE_FLAG_TX_START) {
            for (i = 0; i < info->num_dais; i++) {
                NXP9890_stop_tx(dev, i);
            }
        }
        if (info->state & CODEC_DEVICE_FLAG_RX_START) {
            for (i = 0; i < info->num_dais; i++) {
                NXP9890_stop_rx(dev, i);
            }
        }
    }

    /* disable all widget */
    widget = info->widgets;

    for (i = 0; i < info->num_widgets; i++) {
        NXP9890_disable_widget(dev,widget->widget.id);
        widget++;
    }

    /* ToDo add code put the codec hardware in a power down state*/

    /* clear open state */
    info->state &= ~(CODEC_DEVICE_FLAG_OPEN | CODEC_DEVICE_FLAG_CONFIG);
}

/**
 * @brief Probe audio codec device
 *
 * This function is called by the system to register the driver when the system
 * boot up. This function allocates memory for the private codec device
 * information, and then setup the hardware resource and interrupt handler if
 * driver needed.
 *
 * @param dev - pointer to structure of device data
 * @return 0 on success, negative errno on error
 */
static int NXP9890_audcodec_probe(struct device *dev)
{
    struct NXP9890_info *info = NULL;
    struct control_node *node = NULL;
    struct audio_control *controls = NULL;
    struct audio_widget *widgets = NULL;
    int i = 0, j = 0;

    if (!dev) {
        return -EINVAL;
    }

    /* allocate codec private information structure memory space */
    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    strcpy((char*)info->name, NXP9890_CODEC_NAME);

    /* link pre-defined setting */
    info->init_regs = NXP9890_init_regs;
    info->num_regs = ARRAY_SIZE(NXP9890_init_regs);
    info->dais = NXP9890_dais;
    info->num_dais = ARRAY_SIZE(NXP9890_dais);
    info->controls = NXP9890_controls;
    info->num_controls = ARRAY_SIZE(NXP9890_controls);
    info->widgets = NXP9890_widgets;
    info->num_widgets = ARRAY_SIZE(NXP9890_widgets);
    info->routes = NXP9890_routes;
    info->num_routes = ARRAY_SIZE(NXP9890_routes);
    info->rx_delay = 0;
    info->tx_delay = 0;

    /* enable TDM mode support */
    info->tdm_en = 1;

    /* initialize i2c bus */
    info->i2c = up_i2cinitialize(0);
    if (!info->i2c) {
        free(info);
        return -EIO;
    }

    /* assign codec register access function,
     * common codec function will use two function acces codec hardware
     */
    info->codec_read = NXP9890_audcodec_hw_read;
    info->codec_write = NXP9890_audcodec_hw_write;

    device_set_private(dev, info);
    codec_dev = dev;

    /* create control object linklist to link all controls */
    controls = info->controls;
    widgets = info->widgets;

    list_init(&info->control_list);

    for (i = 0; i < info->num_controls; i++) {
        node = zalloc(sizeof(struct control_node));
        if (!node) {
            return -ENOMEM;
        }
        node->control = &controls[i];
        node->index = 0;
        node->parent_widget = NULL;
        list_add(&info->control_list, &node->list);
    }
    for (i = 0; i < info->num_widgets; i++) {
        if (!widgets[i].controls) {
            continue;
        }
        for (j = 0; j < widgets[i].num_controls; j++) {
            node = zalloc(sizeof(struct control_node));
            if (!node) {
                return -ENOMEM;
            }
            node->control = &widgets[i].controls[j];
            node->index = j;
            node->parent_widget = &widgets[i];
            list_add(&info->control_list, &node->list);
        }
    }
    info->state |= CODEC_DEVICE_FLAG_PROBE;
    return 0;
}

/**
 * @brief Remove audio codec device
 *
 * This function is called by the system to unregister the driver. It should
 * release the hardware resource and interrupt setting, and then free memory
 * that allocated by the probe() function.
 * This function should be called after probe() function. If driver was opened,
 * this function should call close() function before releasing resources.
 *
 * @param dev - pointer to structure of device data
 */
static void NXP9890_audcodec_remove(struct device *dev)
{
    struct NXP9890_info *info = NULL;
    struct list_head *iter, *iter_next;
    struct control_node *ctlnode = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->state & CODEC_DEVICE_FLAG_OPEN) {
        NXP9890_audcodec_close(dev);
    }
    if (info->i2c) {
        up_i2cuninitialize(info->i2c);
        info->i2c = NULL;
    }
    info->codec_read = NULL;
    info->codec_write = NULL;
    info->state = 0;

    /* free control linklist */
    list_foreach_safe(&info->control_list, iter, iter_next) {
        ctlnode = list_entry(iter, struct control_node, list);
        list_del(&ctlnode->list);
    }
    device_set_private(dev, NULL);
    codec_dev = NULL;
    free(info);
}

static struct device_codec_type_ops NXP9890_audcodec_type_ops = {
    .get_topology_size = NXP9890_get_topology_size,
    .get_topology = NXP9890_get_topology,
    .get_control = NXP9890_get_control,
    .set_control = NXP9890_set_control,
    .enable_widget = NXP9890_enable_widget,
    .disable_widget = NXP9890_disable_widget,
    .get_caps = NXP9890_get_caps,
    .set_config = NXP9890_set_config,
    .get_tx_delay = NXP9890_get_tx_delay,
    .start_tx = NXP9890_start_tx,
    .stop_tx = NXP9890_stop_tx,
    .register_tx_callback = NXP9890_register_tx_callback,
    .get_rx_delay = NXP9890_get_rx_delay,
    .start_rx = NXP9890_start_rx,
    .stop_rx = NXP9890_stop_rx,
    .register_rx_callback = NXP9890_register_rx_callback,
    .register_jack_event_callback = NXP9890_register_jack_event_callback,
    .register_button_event_callback = NXP9890_register_button_event_callback,
};

static struct device_driver_ops NXP9890_audcodec_ops = {
    .probe          = NXP9890_audcodec_probe,
    .remove         = NXP9890_audcodec_remove,
    .open           = NXP9890_audcodec_open,
    .close          = NXP9890_audcodec_close,
    .type_ops       = &NXP9890_audcodec_type_ops,
};

struct device_driver NXP9890_audcodec = {
    .type       = DEVICE_TYPE_CODEC_HW,
    .name       = "NXP9890",
    .desc       = "TFA9890A Audio Codec driver",
    .ops        = &NXP9890_audcodec_ops,
};
