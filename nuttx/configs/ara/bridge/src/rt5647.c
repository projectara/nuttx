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
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c.h>
#include "audcodec.h"
#include "rt5647.h"

// ignore unused warning message for current stage
#pragma GCC diagnostic ignored "-Wunused-function"

#define RT5647_CODEC_NAME   "rt5647"

static struct device *codec_dev = NULL;

struct rt5647_reg {
    uint8_t reg;
    uint16_t val;
};

struct pll_code {
    int m;
    int n;
    int k;
    int bp;
};

struct rt5647_info {
    struct device *dev;
    struct i2c_dev_s *i2c;
    uint8_t name[AUDIO_CODEC_NAME_MAX];

    struct rt5647_reg *init_regs;
    int num_regs;

    struct audio_dai *dais;
    int num_dais;
    struct audio_control *controls;
    int num_controls;
    struct audio_widget *widgets;
    int num_widgets;
    audio_route *routes;
    int num_routes;

    uint32_t rx_delay;
    device_codec_event_callback *rx_callback;
    void* rx_callback_arg;
    uint32_t tx_delay;
    device_codec_event_callback *tx_callback;
    void* tx_callback_arg;
    device_codec_jack_event_callback *jack_event_callback;
    void* jack_event_callback_arg;
    device_codec_button_event_callback *button_event_callback;
    void* button_event_callback_arg;

    int clk_id; // sysclk source : mclk:0 , pll:1

    /* hardware access function */
    uint32_t (*codec_read)(uint32_t reg, uint32_t *value);
    uint32_t (*codec_write)(uint32_t reg, uint32_t value);
};

struct rt5647_reg rt5647_init_regs[] = {
    { RT5647_GENERAL_CTRL_1, 0x2061 }, /* enable MCLK Gate Control */
    { RT5647_ADC_DAC_CLK_CTRL, 0x0000 },
    { RT5647_PR_INDEX, 0x003d },
    { RT5647_PR_DATA, 0x3600 },

    /* playback */
    { RT5647_DAC_STO_DIGI_MIXER, 0x4646 },/* DACL2 & DACR2 */
    { RT5647_OUT_L_MIXER_MUTE, 0x001F },
    { RT5647_OUT_R_MIXER_MUTE, 0x001F },
    { RT5647_LOUT_MIXER, 0xF000 },
    { RT5647_LOUT_VOL, 0xC8C8 },
    { RT5647_HP_L_MIXER_MUTE, 0x001F },
    { RT5647_HP_R_MIXER_MUTE, 0x001F },
    { RT5647_HPO_MIXER_CTRL, 0x6000 },
    { RT5647_HPOUT_VOL, 0xC8C8 },

    { RT5647_SPK_L_MIXER_CTRL, 0x003C }, /* DACL1 */
    { RT5647_SPK_R_MIXER_CTRL, 0x003C }, /* DACR1 */
    { RT5647_SPO_MIXER_CTRL, 0xD806 }, /* SPKVOLL & SPKVOLR */

    { RT5647_SPKOUT_VOL, 0x8888 }, /* SPOL&SPOR output mute */
    /* record */
    { RT5647_IN2_CTRL, 0x0000 },/* IN1 boost 20db and signal ended mode */
    { RT5647_REC_MIXER_L_CTRL, 0x007F },/* Mic1 -> RECMIXL */
    { RT5647_REC_MIXER_R_CTRL, 0x007F },/* Mic1 -> RECMIXR */

    { RT5647_STO1_ADC_DIGI_MIXER, 0x7060 },

    { RT5647_STO1_ADC_DIGI_VOL, 0xAFAF },/* Mute STO1 ADC for depop, Digital Input Gain */
};

struct audio_dai rt5647_dais[] = {
    {
        .dai = {
            .name = "rt5647-aif1",
            .cport = 0,
            .capture = {
                .stream_name = "AIF1 Capture",
                .formats = RT5647_FORMATS,
                .rates = RT5647_STEREO_RATES,
                .chan_min = 1,
                .chan_max = 2,
            },
            .playback = {
                .stream_name = "AIF1 Playback",
                .formats = RT5647_FORMATS,
                .rates = RT5647_STEREO_RATES,
                .chan_min = 1,
                .chan_max = 2,
            },
        },
        .caps = {
            .protocol = DEVICE_CODEC_PROTOCOL_PCM |
                        DEVICE_CODEC_PROTOCOL_I2S |
                        DEVICE_CODEC_PROTOCOL_LR_STEREO,
            .wclk_polarity = DEVICE_CODEC_POLARITY_NORMAL |
                             DEVICE_CODEC_POLARITY_REVERSED,
            .wclk_change_edge = DEVICE_CODEC_EDGE_RISING |
                                DEVICE_CODEC_EDGE_FALLING,
            .data_rx_edge = DEVICE_CODEC_EDGE_RISING |
                            DEVICE_CODEC_EDGE_FALLING,
            .data_tx_edge = DEVICE_CODEC_EDGE_RISING |
                            DEVICE_CODEC_EDGE_FALLING,
        },
    },
};

enum {
    RT5647_CTL_SPKOUT_MUTE,
    RT5647_CTL_SPKOUT_VOL,
    RT5647_CTL_SPKVOL_MUTE,
    RT5647_CTL_DAC2_SWITCH,
    RT5647_CTL_DAC2_VOL,
    RT5647_CTL_DAC2_LSRC,
    RT5647_CTL_DAC2_RSRC,
    RT5647_CTL_DACL1_MIXL,
    RT5647_CTL_DACL2_MIXL,
    RT5647_CTL_DACR1_MIXL,
    RT5647_CTL_DACR1_MIXR,
    RT5647_CTL_DACR2_MIXR,
    RT5647_CTL_DACL1_MIXR,
    RT5647_CTL_SPKL_BST1,
    RT5647_CTL_SPKL_BST3,
    RT5647_CTL_SPKL_INL,
    RT5647_CTL_SPKL_DACL2,
    RT5647_CTL_SPKL_DACL1,
    RT5647_CTL_SPKR_BST2,
    RT5647_CTL_SPKR_BST3,
    RT5647_CTL_SPKR_INL,
    RT5647_CTL_SPKR_DACR2,
    RT5647_CTL_SPKR_DACR1,
    RT5647_CTL_SPOL_DACL1,
    RT5647_CTL_SPOL_DACR1,
    RT5647_CTL_SPOL_SPKVOLL,
    RT5647_CTL_SPOL_SPKVOLR,
    RT5647_CTL_SPOL_BST3,
    RT5647_CTL_SPOR_DACR1,
    RT5647_CTL_SPOR_BST3,
    RT5647_CTL_SPOR_SPKVOLR,
    RT5647_CTL_MAX
};

enum {
    RT5647_WIDGET_AIF1TX,
    RT5647_WIDGET_AIF1RX,
    RT5647_WIDGET_I2S1,
    RT5647_WIDGET_IF1_ADC,
    RT5647_WIDGET_IF1_DAC1,
    RT5647_WIDGET_IF1_DAC2,
    RT5647_WIDGET_IF1_DAC1L,
    RT5647_WIDGET_IF1_DAC1R,
    RT5647_WIDGET_IF1_DAC2L,
    RT5647_WIDGET_IF1_DAC2R,
    RT5647_WIDGET_DACL2_MUX,
    RT5647_WIDGET_DACR2_MUX,
    RT5647_WIDGET_DACL2_VOL,
    RT5647_WIDGET_DACR2_VOL,
    RT5647_WIDGET_STODAC_MIXL,
    RT5647_WIDGET_STODAC_MIXR,
    RT5647_WIDGET_DAC_L1,
    RT5647_WIDGET_DAC_L2,
    RT5647_WIDGET_DAC_R1,
    RT5647_WIDGET_DAC_R2,
    RT5647_WIDGET_SPK_MIXL,
    RT5647_WIDGET_SPK_MIXR,
    RT5647_WIDGET_SPKVOLL,
    RT5647_WIDGET_SPKVOLR,
    RT5647_WIDGET_SPOL_MIX,
    RT5647_WIDGET_SPOR_MIX,
    RT5647_WIDGET_SPK_AMP,
    RT5647_WIDGET_SPOL,
    RT5647_WIDGET_SPOR,
    RT5647_WIDGET_MAX
};

struct audio_control rt5647_controls[] = {
    AUDCTL_BITS("SPKOUT Mute", RT5647_CTL_SPKOUT_MUTE, MIXER, RT5647_SPKOUT_VOL,
                RT5647_L_MUTE_SFT, RT5647_R_MUTE_SFT, 1),
    AUDCTL_BITSV("SPKOUT Volume", RT5647_CTL_SPKOUT_VOL, MIXER,
                 RT5647_SPKOUT_VOL, RT5647_L_VOL_SFT, RT5647_R_VOL_SFT, 0, 6,
                 0x27, 0),
    AUDCTL_BITS("SPKVOL Mute", RT5647_CTL_SPKVOL_MUTE, MIXER, RT5647_SPKOUT_VOL,
                RT5647_VOL_L_SFT, RT5647_VOL_R_SFT, 1),

    AUDCTL_BITS("DAC2 Switch", RT5647_CTL_DAC2_SWITCH, MIXER,
                RT5647_DACL2_R2_DIGI_MUTE, RT5647_DAC2_VOL_L_SFT,
                RT5647_DAC2_VOL_R_SFT, 1),
    AUDCTL_BITSV("DAC2 Volume", RT5647_CTL_DAC2_VOL, MIXER,
                 RT5647_DACL2_R2_DIGI_VOL, RT5647_L_VOL_SFT, RT5647_R_VOL_SFT,
                 0, 8, 0xAF, 0),
};

char *rt5647_dac12_src[] = {
    "IF1 DAC", "IF2 DAC", "IF3 DAC", "Mono ADC", "VAD_ADC"
};

struct audio_control rt5647_dac_l2_mux[] = {
    AUDCTL_ENUM("DAC2 L source", RT5647_CTL_DAC2_LSRC, MIXER,
                RT5647_DACL2_R2_DIGI_MUTE, RT5647_DAC2_SEL_L_SFT, 3,
                rt5647_dac12_src)
};

char *rt5647_dacr2_src[] = {
    "IF1 DAC", "IF2 DAC", "IF3 DAC", "Mono ADC", "Haptic"
};

struct audio_control rt5647_dac_r2_mux[] = {
    AUDCTL_ENUM("DAC2 R source", RT5647_CTL_DAC2_RSRC, MIXER,
                RT5647_DACL2_R2_DIGI_MUTE, RT5647_DAC2_SEL_R_SFT, 3,
                rt5647_dacr2_src)
};

struct audio_control rt5647_stereo_dac_l_mix[] = {
    AUDCTL_BIT("DAC L1 Switch", RT5647_CTL_DACL1_MIXL, MIXER,
               RT5647_DAC_STO_DIGI_MIXER, RT5647_DAC_L1_SFT, 1),
    AUDCTL_BIT("DAC L2 Switch", RT5647_CTL_DACL2_MIXL, MIXER,
               RT5647_DAC_STO_DIGI_MIXER, RT5647_DAC_L2_SFT, 1),
    AUDCTL_BIT("DAC R1 Switch", RT5647_CTL_DACR1_MIXL, MIXER,
               RT5647_DAC_STO_DIGI_MIXER, RT5647_DAC_R1_L_SFT, 1),
};

struct audio_control rt5647_stereo_dac_r_mix[] = {
    AUDCTL_BIT("DAC R1 Switch", RT5647_CTL_DACR1_MIXR, MIXER,
               RT5647_DAC_STO_DIGI_MIXER, RT5647_DAC_R1_SFT, 1),
    AUDCTL_BIT("DAC R2 Switch", RT5647_CTL_DACR2_MIXR, MIXER,
               RT5647_DAC_STO_DIGI_MIXER, RT5647_DAC_R2_SFT, 1),
    AUDCTL_BIT("DAC L1 Switch", RT5647_CTL_DACL1_MIXR, MIXER,
               RT5647_DAC_STO_DIGI_MIXER, RT5647_DAC_L1_R_SFT, 1),
};

struct audio_control rt5647_spk_l_mix[] = {
    AUDCTL_BIT("BST1 Switch", RT5647_CTL_SPKL_BST1, MIXER,
               RT5647_SPK_L_MIXER_CTRL, RT5647_SPKMIX_L_BST1_SFT, 1),
    AUDCTL_BIT("BST3 Switch", RT5647_CTL_SPKL_BST3, MIXER,
               RT5647_SPK_L_MIXER_CTRL, RT5647_SPKMIX_L_BST3_SFT, 1),
    AUDCTL_BIT("INL Switch", RT5647_CTL_SPKL_INL, MIXER,
               RT5647_SPK_L_MIXER_CTRL, RT5647_SPKMIX_L_INL_SFT, 1),
    AUDCTL_BIT("DAC L2 Switch", RT5647_CTL_SPKL_DACL2, MIXER,
               RT5647_SPK_L_MIXER_CTRL, RT5647_SPKMIX_L_DACL2_SFT, 1),
    AUDCTL_BIT("DAC L1 Switch", RT5647_CTL_SPKL_DACL1, MIXER,
               RT5647_SPK_L_MIXER_CTRL, RT5647_SPKMIX_L_DACL1_SFT, 1),
};

struct audio_control rt5647_spk_r_mix[] = {
    AUDCTL_BIT("BST2 Switch", RT5647_CTL_SPKR_BST2, MIXER,
               RT5647_SPK_R_MIXER_CTRL, RT5647_SPKMIX_R_BST2_SFT, 1),
    AUDCTL_BIT("BST3 Switch", RT5647_CTL_SPKR_BST3, MIXER,
               RT5647_SPK_R_MIXER_CTRL, RT5647_SPKMIX_R_BST3_SFT, 1),
    AUDCTL_BIT("INL Switch", RT5647_CTL_SPKR_INL, MIXER,
               RT5647_SPK_R_MIXER_CTRL, RT5647_SPKMIX_R_INL_SFT, 1),
    AUDCTL_BIT("DAC R2 Switch", RT5647_CTL_SPKR_DACR2, MIXER,
               RT5647_SPK_R_MIXER_CTRL, RT5647_SPKMIX_R_DACR2_SFT, 1),
    AUDCTL_BIT("DAC R1 Switch", RT5647_CTL_SPKR_DACR1, MIXER,
               RT5647_SPK_R_MIXER_CTRL, RT5647_SPKMIX_R_DACR1_SFT, 1),
};

struct audio_control rt5647_spo_l_mix[] = {
    AUDCTL_BIT("DACL1 Switch", RT5647_CTL_SPOL_DACL1, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_L_DACL1_SFT, 1),
    AUDCTL_BIT("DACR1 Switch", RT5647_CTL_SPOL_DACR1, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_L_DACR1_SFT, 1),
    AUDCTL_BIT("SPKVOLL Switch", RT5647_CTL_SPOL_SPKVOLL, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_L_SPKVOLL_SFT, 1),
    AUDCTL_BIT("SPKVOLR Switch", RT5647_CTL_SPOL_SPKVOLR, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_L_SPKVOLR_SFT, 1),
    AUDCTL_BIT("BST3 Switch", RT5647_CTL_SPOL_BST3, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_L_BST3_SFT, 1),
};

struct audio_control rt5647_spo_r_mix[] = {
    AUDCTL_BIT("DACR1 Switch", RT5647_CTL_SPOR_DACR1, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_R_DACR1_SFT, 1),
    AUDCTL_BIT("BST3 Switch", RT5647_CTL_SPOR_BST3, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_R_BST3_SFT, 1),
    AUDCTL_BIT("SPKVOLR Switch", RT5647_CTL_SPOR_SPKVOLR, MIXER,
               RT5647_SPO_MIXER_CTRL, RT5647_SPOMIX_R_SPKVOLR_SFT, 1),
};

struct audio_widget rt5647_widgets[] = {

    WIDGET("AIF1TX", RT5647_WIDGET_AIF1TX, AIF_OUT, NULL, NOPWRCTL, 0, 0),
    WIDGET("IF1 ADC", RT5647_WIDGET_IF1_ADC, PGA, NULL, NOPWRCTL, 0, 0),

    WIDGET("AIF1RX", RT5647_WIDGET_AIF1RX, AIF_IN, NULL, NOPWRCTL, 0, 0),

    WIDGET("I2S1", RT5647_WIDGET_I2S1, SUPPLY, NULL, RT5647_PWR_MGT_1,
           RT5647_PWR1_I2S1_EN, 0),

    WIDGET("IF1 DAC1", RT5647_WIDGET_IF1_DAC1, PGA, NULL, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC1 L", RT5647_WIDGET_IF1_DAC1L, PGA, NULL, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC1 R", RT5647_WIDGET_IF1_DAC1R, PGA, NULL, NOPWRCTL, 0, 0),

    WIDGET("IF1 DAC2", RT5647_WIDGET_IF1_DAC2, PGA, NULL, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC2 L", RT5647_WIDGET_IF1_DAC2L, PGA, NULL, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC2 R", RT5647_WIDGET_IF1_DAC2R, PGA, NULL, NOPWRCTL, 0, 0),

    WIDGET("DAC L2 MUX", RT5647_WIDGET_DACL2_MUX, MUX, rt5647_dac_l2_mux,
           NOPWRCTL, 0, 0),
    WIDGET("DAC R2 MUX", RT5647_WIDGET_DACR2_MUX, MUX, rt5647_dac_r2_mux,
           NOPWRCTL, 0, 0),

    WIDGET("DAC L2 Volume", RT5647_WIDGET_DACL2_VOL, PGA, NULL,
           RT5647_PWR_MGT_1, RT5647_PWR1_DACL2_EN, 0),
    WIDGET("DAC R2 Volume", RT5647_WIDGET_DACR2_VOL, PGA, NULL,
           RT5647_PWR_MGT_1, RT5647_PWR1_DACR2_EN, 0),

    WIDGET("Stereo DAC MIXL", RT5647_WIDGET_STODAC_MIXL, MIXER,
           rt5647_stereo_dac_l_mix, NOPWRCTL, 0, 0),
    WIDGET("Stereo DAC MIXR", RT5647_WIDGET_STODAC_MIXR, MIXER,
           rt5647_stereo_dac_r_mix, NOPWRCTL, 0, 0),

    WIDGET("DAC L1", RT5647_WIDGET_DAC_L1, DAC, NULL, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACL1_EN, 0),
    WIDGET("DAC L2", RT5647_WIDGET_DAC_L2, DAC, NULL, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACL2_EN, 0),
    WIDGET("DAC R1", RT5647_WIDGET_DAC_R1, DAC, NULL, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACR1_EN, 0),
    WIDGET("DAC R2", RT5647_WIDGET_DAC_R2, DAC, NULL, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACR2_EN, 0),

    WIDGET("SPK MIXL", RT5647_WIDGET_SPK_MIXL, MIXER, rt5647_spk_l_mix,
           RT5647_PWR_MGT_5, RT5647_PWR5_SPKMIXL_EN, 0),
    WIDGET("SPK MIXR", RT5647_WIDGET_SPK_MIXR, MIXER, rt5647_spk_r_mix,
           RT5647_PWR_MGT_5, RT5647_PWR5_SPKMIXR_EN, 0),

    WIDGET("SPKVOL L", RT5647_WIDGET_SPKVOLL, PGA, NULL, RT5647_PWR_MGT_6,
           RT5647_PWR6_SPOVOLL_EN, 0),
    WIDGET("SPKVOL R", RT5647_WIDGET_SPKVOLR, PGA, NULL, RT5647_PWR_MGT_6,
           RT5647_PWR6_SPOVOLR_EN, 0),

    WIDGET("SPOL MIX", RT5647_WIDGET_SPOL_MIX, MIXER, rt5647_spo_l_mix,
           NOPWRCTL, 0, 0),
    WIDGET("SPOR MIX", RT5647_WIDGET_SPOR_MIX, MIXER, rt5647_spo_r_mix,
           NOPWRCTL, 0, 0),

    WIDGET("SPK amp", RT5647_WIDGET_SPK_AMP, PGA, NULL, NOPWRCTL, 0, 0),

    WIDGET("SPOL", RT5647_WIDGET_SPOL, OUTPUT, NULL, NOPWRCTL, 0, 0),
    WIDGET("SPOR", RT5647_WIDGET_SPOR, OUTPUT, NULL, NOPWRCTL, 0, 0),
};

audio_route rt5647_routes[] = {
    // AIF1RX
    { RT5647_WIDGET_AIF1RX, RT5647_WIDGET_IF1_DAC1, NOCONTROL },
    { RT5647_WIDGET_AIF1RX, RT5647_WIDGET_IF1_DAC2, NOCONTROL },

    // I2S1
    { RT5647_WIDGET_I2S1, RT5647_WIDGET_IF1_ADC, NOCONTROL },
    { RT5647_WIDGET_I2S1, RT5647_WIDGET_IF1_DAC1, NOCONTROL },
    { RT5647_WIDGET_I2S1, RT5647_WIDGET_IF1_DAC2, NOCONTROL },

    // IF1 DAC1
    { RT5647_WIDGET_IF1_DAC1, RT5647_WIDGET_IF1_DAC1L, NOCONTROL },
    { RT5647_WIDGET_IF1_DAC1, RT5647_WIDGET_IF1_DAC1R, NOCONTROL },
    // IF1 DAC2
    { RT5647_WIDGET_IF1_DAC2, RT5647_WIDGET_IF1_DAC2L, NOCONTROL },
    { RT5647_WIDGET_IF1_DAC2, RT5647_WIDGET_IF1_DAC2R, NOCONTROL },

    // IF1 DAC2 L
    { RT5647_WIDGET_IF1_DAC2L, RT5647_WIDGET_DACL2_MUX,
      MUXID(RT5647_CTL_DAC2_LSRC, 0) },
    // IF1 DAC2 R
    { RT5647_WIDGET_IF1_DAC2R, RT5647_WIDGET_DACR2_MUX,
      MUXID(RT5647_CTL_DAC2_RSRC, 0) },

    // DAC L2 Mux
    { RT5647_WIDGET_DACL2_MUX, RT5647_WIDGET_DACL2_VOL, NOCONTROL },
    // DAC R2 Mux
    { RT5647_WIDGET_DACR2_MUX, RT5647_WIDGET_DACR2_VOL, NOCONTROL },


    // DAC L2 Volume
    { RT5647_WIDGET_DACL2_VOL, RT5647_WIDGET_STODAC_MIXL,
      RT5647_CTL_DACL2_MIXL },
    // DAC R2 Volume
    { RT5647_WIDGET_DACR2_VOL, RT5647_WIDGET_STODAC_MIXR,
      RT5647_CTL_DACR2_MIXR },

    // Stereo DAC MIXL
    { RT5647_WIDGET_STODAC_MIXL, RT5647_WIDGET_DAC_L1, NOCONTROL },
    // Stereo DAC MIXR
    { RT5647_WIDGET_STODAC_MIXR, RT5647_WIDGET_DAC_R1, NOCONTROL },

    // DAC L1
    { RT5647_WIDGET_DAC_L1, RT5647_WIDGET_SPK_MIXL, RT5647_CTL_SPKL_DACL1 },
    { RT5647_WIDGET_DAC_L1, RT5647_WIDGET_SPOL_MIX, RT5647_CTL_SPOL_DACL1 },
    // DAC R1
    { RT5647_WIDGET_DAC_R1, RT5647_WIDGET_SPK_MIXR, RT5647_CTL_SPKR_DACR1 },
    { RT5647_WIDGET_DAC_R1, RT5647_WIDGET_SPOL_MIX, RT5647_CTL_SPOL_DACR1 },
    { RT5647_WIDGET_DAC_R1, RT5647_WIDGET_SPOR_MIX, RT5647_CTL_SPOR_DACR1 },

    // SPK MIXL
    { RT5647_WIDGET_SPK_MIXL, RT5647_WIDGET_SPKVOLL, NOCONTROL },
    // SPK MIXR
    { RT5647_WIDGET_SPK_MIXR, RT5647_WIDGET_SPKVOLR, NOCONTROL },

    // SPKVOL L
    { RT5647_WIDGET_SPKVOLL, RT5647_WIDGET_SPOL_MIX, RT5647_CTL_SPOL_SPKVOLL },
    // SPKVOL R
    { RT5647_WIDGET_SPKVOLR, RT5647_WIDGET_SPOL_MIX, RT5647_CTL_SPOL_SPKVOLR },
    { RT5647_WIDGET_SPKVOLR, RT5647_WIDGET_SPOR_MIX, RT5647_CTL_SPOR_SPKVOLR },

    // SPOL MIX
    { RT5647_WIDGET_SPOL_MIX, RT5647_WIDGET_SPK_AMP, NOCONTROL },
    // SPOR MIX
    { RT5647_WIDGET_SPOR_MIX, RT5647_WIDGET_SPK_AMP, NOCONTROL },

    // SPK amp
    { RT5647_WIDGET_SPK_AMP, RT5647_WIDGET_SPOL, NOCONTROL },
    { RT5647_WIDGET_SPK_AMP, RT5647_WIDGET_SPOR, NOCONTROL },
};

struct device* get_codec_dev()
{
    return codec_dev;
}

void* get_codec_read_func(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return NULL;
    }
    info = device_get_private(dev);
    return (void*)info->codec_read;
}

void* get_codec_write_func(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return NULL;
    }
    info = device_get_private(dev);
    return (void*)info->codec_write;
}

static uint32_t rt5647_audcodec_hw_read(uint32_t reg, uint32_t *value)
{
    struct device *dev = get_codec_dev();
    struct rt5647_info *info = NULL;
    uint8_t cmd = 0;
    uint16_t data = 0;

    struct i2c_msg_s msg[] = {
        {
            .addr = RT5647_I2C_ADDR,
            .flags = 0,
            .buffer = &cmd,
            .length = 1,
        },
        {
            .addr = RT5647_I2C_ADDR,
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

    *value = (uint32_t)data;
    return 0;
}

static uint32_t rt5647_audcodec_hw_write(uint32_t reg, uint32_t value)
{
    struct device *dev = get_codec_dev();
    struct rt5647_info *info = NULL;
    uint8_t cmd[3] = {0x00, 0x00, 0x00};
    struct i2c_msg_s msg[] = {
        {
            .addr = RT5647_I2C_ADDR,
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
    cmd[1] = (uint8_t)(value & 0xFF);
    cmd[2] = (uint8_t)((value >> 8) & 0xFF);

    if (I2C_TRANSFER(info->i2c, msg, 1)) {
        return -EIO;
    }
    return 0;
}

static int rt5647_get_topology_size(struct device *dev, uint16_t *size)
{
    struct rt5647_info *info = NULL;
    int tpg_size = 0;

    if (!dev || !device_get_private(dev) || !size) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    tpg_size = sizeof(struct gb_audio_topology);
    tpg_size += info->num_dais * sizeof(struct gb_audio_dai);
    tpg_size += info->num_controls * sizeof(struct gb_audio_control);
    tpg_size += info->num_widgets * sizeof(struct gb_audio_widget);
    tpg_size += info->num_routes * sizeof(struct gb_audio_route);

    *size = tpg_size;
    return 0;
}

static int rt5647_get_topology(struct device *dev,
                               struct gb_audio_topology *topology)
{
    struct rt5647_info *info = NULL;
    int len = 0, i = 0;
    uint8_t *data = NULL;

    if (!dev || !device_get_private(dev) || !topology) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    if (!info->dais || !info->controls || !info->widgets || !info->routes) {
        return -EINVAL;
    }

    topology->num_dais = info->num_dais;
    topology->num_controls = info->num_controls;
    topology->num_widgets = info->num_widgets;
    topology->num_routes = info->num_routes;

    data = topology->data;
    /* fill dai object */
    len = sizeof(struct gb_audio_dai);
    for (i = 0; i < info->num_dais; i++) {
        memcpy(data, &info->dais[i].dai, len);
        data += len;
    }

    /* fill audio control object */
    len = sizeof(struct gb_audio_control);
    for (i = 0; i < info->num_controls; i++) {
        memcpy(data, &info->controls[i].control, len);
        data += len;
    }

    /* fill audio widget object */
    len = sizeof(struct gb_audio_widget);
    for (i = 0; i < info->num_widgets; i++) {
        memcpy(data, &info->widgets[i].widget, len);
        data += len;
    }

    /* fill audio route object */
    len = sizeof(struct gb_audio_route);
    for (i = 0; i < info->num_routes; i++) {
        memcpy(data, &info->routes[i], len);
        data += len;
    }
    return 0;
}

static int rt5647_rate_to_freq(uint32_t rate)
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

static int rt5647_fmtbit_to_bitnum(uint32_t fmtbit)
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

static int rt5647_get_caps(struct device *dev, unsigned int dai_idx,
                           uint8_t clk_role, struct device_codec_pcm *pcm,
                           struct device_codec_dai *dai)
{
    struct rt5647_info *info = NULL;
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

    if (!(clk_role & DEVICE_CODEC_ROLE_SLAVE)) {
        /* In current audio module, we only supported slave mode. */
        return -EINVAL;
    }
    memcpy(dai, &info->dais[dai_idx].caps, sizeof(struct device_codec_dai));
    return 0;
}

int rt5647_pll_calc(uint32_t infreq, uint32_t outfreq, struct pll_code *code) {
    int m = 0, n = 0, k = 0, find = 0;
    int t = 0, t1 = 0, out;

    if (!code) {
        return -EINVAL;
    }
    k = 2; /* assume K = 2 (typical)*/
    t1 = outfreq / 1000; /* assume clock tolerance is 1KHz */

    for (n = 0; n <= RT5647_PLL_N_CODE_MAX; n++) {
        for (m = 0; m <= RT5647_PLL_M_CODE_MAX; m++) {
            out = (infreq * (n + 2)) / ((m + 2) * (k + 2));
            t = abs(outfreq - out);
            if (t1 >= t) {
                find = 1;
                goto findout;
            }
        }
    }

findout:
    if (find) {
        code->m = m;
        code->n = n;
        code->k = k;
        code->bp = 0; /* assume m_bypass = 0 temporarily */

        return 0;
    }
    return -EINVAL;
}

static int rt5647_set_config(struct device *dev, unsigned int dai_idx,
                             uint8_t clk_role, struct device_codec_pcm *pcm,
                             struct device_codec_dai *dai)
{
    struct rt5647_info *info = NULL;
    struct gb_audio_pcm *pbpcm = NULL;
    struct pll_code code;
    int sysclk = 0, ratefreq = 0, numbits = 0, channels = 0, ret;
    uint32_t value = 0, mask = 0;

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

    if (!(clk_role & DEVICE_CODEC_ROLE_SLAVE)) {
        /* In current audio module, we only supported slave mode. */
        return -EINVAL;
    }
    // check clock setting
    ratefreq = rt5647_rate_to_freq(pcm->rate);
    numbits = rt5647_fmtbit_to_bitnum(pcm->format);
    channels = pcm->channels;

    if (ratefreq <= 0 || numbits <= 0) {
        return -EINVAL;
    }

    sysclk = 256 * ratefreq; /* 256*FS */
    ret = rt5647_pll_calc(dai->mclk_freq, sysclk, &code);
    if (ret) {
        return -EINVAL;
    }
    // write clock setting
    value = RT5647_SYSCLK_S_MCLK | RT5647_PLL_S_MCLK;
    mask = RT5647_SYSCLK_S_MASK | RT5647_PLL_S_MASK;
    audcodec_update(RT5647_GLOBAL_CLOCK, value, mask);

    /* set n & k code */
    value = (code.n << RT5647_PLL_N_CODE_SFT) |
            (code.k << RT5647_PLL_K_CODE_SFT);
    audcodec_write(RT5647_PLL1, value);
    /* set m & m_bypass code */
    value = (code.m << RT5647_PLL_M_CODE_SFT) |
            (code.bp << RT5647_PLL_M_BYPASS_SFT);
    audcodec_write(RT5647_PLL2, value);

    return 0;
}

static int rt5647_get_control(struct device *dev, uint8_t control_id,
                              struct gb_audio_ctl_elem_value *value)
{
    struct rt5647_info *info = NULL;
    int i = 0;
    struct audio_control *controls = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    controls = info->controls;

    for (i = 0; i < info->num_controls; i++) {
        if (controls[i].control.id == control_id) {
            if (controls[i].get) {
                return controls[i].get(&controls[i], value);
            }
        }
    }
    return -EINVAL;
}

static int rt5647_set_control(struct device *dev, uint8_t control_id,
                              struct gb_audio_ctl_elem_value *value)
{
    struct rt5647_info *info = NULL;
    int i = 0;
    struct audio_control *controls = NULL;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    controls = info->controls;

    for (i = 0; i < info->num_controls; i++) {
        if (controls[i].control.id == control_id) {
            if (controls[i].set) {
                return controls[i].set(&controls[i], value);
            }
        }
    }
    return -EINVAL;
}

static int rt5647_enable_widget(struct device *dev, uint8_t widget_id)
{
    struct rt5647_info *info = NULL;
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
            if (widget->reg == NOPWRCTL) {
                widget->widget.state = GB_AUDIO_WIDGET_STATE_ENAABLED;
                return 0;
            }
            /* turn on the widget */
            if (audcodec_read(widget->reg, &data)) {
                return -EIO;
            }
            mask = 1 << widget->shift;
            data = (data & ~mask) | ((widget->inv)? 0 : mask);
            if (audcodec_write(widget->reg, data)) {
                return -EIO;
            }
            widget->widget.state = GB_AUDIO_WIDGET_STATE_ENAABLED;
            return 0;
        }
        widget++;
    }
    return -EINVAL;
}

static int rt5647_disable_widget(struct device *dev, uint8_t widget_id)
{
    struct rt5647_info *info = NULL;
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
            if (widget->reg == NOPWRCTL) {
                widget->widget.state = GB_AUDIO_WIDGET_STATE_DISABLED;
                return 0;
            }
            /* turn off the widget */
            if (audcodec_read(widget->reg, &data)) {
                return -EIO;
            }
            mask = 1 << widget->shift;
            data = (data & ~mask) | ((widget->inv)? mask : 0);
            if (audcodec_write(widget->reg, data)) {
                return -EIO;
            }
            widget->widget.state = GB_AUDIO_WIDGET_STATE_DISABLED;
            return 0;
        }
        widget++;
    }
    return -EINVAL;
}

static int rt5647_get_tx_delay(struct device *dev, uint32_t *delay)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev) || !delay) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    *delay = info->tx_delay;
    return 0;
}

static int rt5647_start_tx(struct device *dev, uint32_t dai_idx)
{
    return 0;
}

static int rt5647_stop_tx(struct device *dev, uint32_t dai_idx)
{
    return 0;
}

static int rt5647_register_tx_callback(struct device *dev,
                                       device_codec_event_callback *callback,
                                       void *arg)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    info->tx_callback = callback;
    info->tx_callback_arg = arg;
    return 0;
}

static int rt5647_get_rx_delay(struct device *dev, uint32_t *delay)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev) || !delay) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    *delay = info->rx_delay;
    return 0;
}

static int rt5647_start_rx(struct device *dev, uint32_t dai_idx)
{
    return 0;
}

static int rt5647_stop_rx(struct device *dev, uint32_t dai_idx)
{
    return 0;
}

static int rt5647_register_rx_callback(struct device *dev,
                                       device_codec_event_callback *callback,
                                       void *arg)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    info->rx_callback = callback;
    info->rx_callback_arg = arg;
    return 0;
}

static int rt5647_register_jack_event_callback(struct device *dev,
                                    device_codec_jack_event_callback *callback,
                                    void *arg)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    info->jack_event_callback = callback;
    info->jack_event_callback_arg = arg;
    return 0;
}

static int rt5647_register_button_event_callback(struct device *dev,
                                  device_codec_button_event_callback *callback,
                                  void *arg)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev) || !callback) {
        return -EINVAL;
    }
    info = device_get_private(dev);
    info->button_event_callback = callback;
    info->button_event_callback_arg = arg;
    return 0;
}

static int rt5647_audcodec_open(struct device *dev)
{
    struct rt5647_info *info = NULL;
    int ret = 0, i = 0;

    if (!dev || !device_get_private(dev)) {
        return -EINVAL;
    }
    info = device_get_private(dev);

    /* codec power on sequence */
    audcodec_write(RT5647_RESET, 0);    /* software reset */

    /* initialize audio codec */
    for (i = 0; i < info->num_regs; i++) {
        audcodec_write(info->init_regs[i].reg , info->init_regs[i].val);
    }
    return ret;
}

static void rt5647_audcodec_close(struct device *dev)
{
    struct rt5647_info *info = NULL;
    struct audio_widget *widget = NULL;
    int i = 0;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    /* disable all widget */
    widget = info->widgets;

    for (i = 0; i < info->num_widgets; i++) {
        rt5647_disable_widget(dev,widget->widget.id);
        widget++;
    }
}

static int rt5647_audcodec_probe(struct device *dev)
{
    struct rt5647_info *info;

    if (!dev) {
        return -EINVAL;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->dev = dev;
    strcpy((char*)info->name, RT5647_CODEC_NAME);

    info->init_regs = rt5647_init_regs;
    info->num_regs = ARRAY_SIZE(rt5647_init_regs);
    info->dais = rt5647_dais;
    info->num_dais = ARRAY_SIZE(rt5647_dais);
    info->controls = rt5647_controls;
    info->num_controls = ARRAY_SIZE(rt5647_controls);
    info->widgets = rt5647_widgets;
    info->num_widgets = ARRAY_SIZE(rt5647_widgets);
    info->routes = rt5647_routes;
    info->num_routes = ARRAY_SIZE(rt5647_routes);
    info->rx_delay = 0;
    info->tx_delay = 0;

    /* initialize i2c bus */
    info->i2c = up_i2cinitialize(0);
    if (!info->i2c) {
        free(info);
        return -EIO;
    }

    info->codec_read = rt5647_audcodec_hw_read;
    info->codec_write = rt5647_audcodec_hw_write;
    device_set_private(dev, info);
    codec_dev = dev;

    return 0;
}

static void rt5647_audcodec_remove(struct device *dev)
{
    struct rt5647_info *info = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }
    info = device_get_private(dev);

    if (info->i2c) {
        up_i2cuninitialize(info->i2c);
        info->i2c = NULL;
    }
    info->codec_read = NULL;
    info->codec_write = NULL;
    device_set_private(dev, NULL);
    codec_dev = NULL;
    free(info);
}

static struct device_codec_type_ops rt5647_audcodec_type_ops = {
    .get_topology_size = rt5647_get_topology_size,
    .get_topology = rt5647_get_topology,
    .get_control = rt5647_get_control,
    .set_control = rt5647_set_control,
    .enable_widget = rt5647_enable_widget,
    .disable_widget = rt5647_disable_widget,
    .get_caps = rt5647_get_caps,
    .set_config = rt5647_set_config,
    .get_tx_delay = rt5647_get_tx_delay,
    .start_tx = rt5647_start_tx,
    .stop_tx = rt5647_stop_tx,
    .register_tx_callback = rt5647_register_tx_callback,
    .get_rx_delay = rt5647_get_rx_delay,
    .start_rx = rt5647_start_rx,
    .stop_rx = rt5647_stop_rx,
    .register_rx_callback = rt5647_register_rx_callback,
    .register_jack_event_callback = rt5647_register_jack_event_callback,
    .register_button_event_callback = rt5647_register_button_event_callback,
};

static struct device_driver_ops rt5647_audcodec_ops = {
    .probe          = rt5647_audcodec_probe,
    .remove         = rt5647_audcodec_remove,
    .open           = rt5647_audcodec_open,
    .close          = rt5647_audcodec_close,
    .type_ops       = &rt5647_audcodec_type_ops,
};

struct device_driver rt5647_audcodec = {
    .type       = DEVICE_TYPE_CODEC_HW,
    .name       = "rt5647",
    .desc       = "ALC5647 Audio Codec driver",
    .ops        = &rt5647_audcodec_ops,
};
