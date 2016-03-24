/*
 * Copyright (c) 2015-2016 Google, Inc.
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
#include <ara_debug.h>
#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_i2c.h>
#include <nuttx/list.h>
#include <nuttx/device_audio_board.h>
#include <nuttx/ara/audio_board.h>
#include <nuttx/ara/codec.h>

#undef ENABLE_HAPTIC_TEST
#define MIN_SPEAKER_SUPPORT

#define RT5647_CODEC_NAME                   "rt5647"

/* One bundle count for each codec on the audio module */
/* TODO: Pass the bundle ID in via init_data */
#define CODEC_BUNDLE                        0

#define CODEC_DEVICE_FLAG_PROBE             BIT(0)  /* device probed */
#define CODEC_DEVICE_FLAG_OPEN              BIT(1)  /* device opened */
#define CODEC_DEVICE_FLAG_CONFIG            BIT(2)  /* device configured */
#define CODEC_DEVICE_FLAG_TX_START          BIT(3)  /* device tx started */
#define CODEC_DEVICE_FLAG_RX_START          BIT(4)  /* device rx started */
#define CODEC_DEVICE_FLAG_CLOSE             BIT(5)  /* device closed */

/* Reset */
#define RT5647_RESET                        0x00

#define RT5647_SPKOUT_VOL                   0x01
#define RT5647_HPOUT_VOL                    0x02
#define RT5647_LOUT_VOL                     0x03
#define RT5647_LOUT_CTRL                    0x05
#define RT5647_MONO_OUT_CTRL                0x04
#define RT5647_IN1_CTRL1                    0x0A
#define RT5647_IN1_CTRL2                    0x0B
#define RT5647_IN1_CTRL3                    0x0C
#define RT5647_IN2_CTRL                     0x0D
#define RT5647_IN3_CTRL                     0x0E
#define RT5647_INL_INR_VOL                  0x0F

#define RT5647_SIDETONE_CTRL                0x18

/* Digital Gain/Volume */
#define RT5647_DACL1_R1_DIGI_VOL            0x19
#define RT5647_DACL2_R2_DIGI_VOL            0x1A
#define RT5647_DACL2_R2_DIGI_MUTE           0x1B
#define RT5647_STO1_ADC_DIGI_VOL            0x1C
#define RT5647_MONO_ADC_DIGI_VOL            0x1D
#define RT5647_STO1_ADC_BST_GAIN            0x1E
#define RT5647_MONO_ADC_BST_GAIN            0x20

/* Digital Mixer */
#define RT5647_STO1_ADC_DIGI_MIXER          0x27
#define RT5647_MONO_ADC_DIGI_MIXER          0x28
#define RT5647_ADC_DAC_DIGI_MIXER           0x29
#define RT5647_DAC_STO_DIGI_MIXER           0x2A
#define RT5647_DAC_MONO_DIGI_MIXER          0x2B
#define RT5647_DAC_DD_DIGI_MIXER            0x2C
#define RT5647_DATA_COPY_MODE               0x2F

/* PDM Interface */
#define RT5647_PDM_CTRL                     0x31

/* Input Mixer */
#define RT5647_REC_MIXER_L_GAIN             0x3B
#define RT5647_REC_MIXER_L_CTRL             0x3C
#define RT5647_REC_MIXER_R_GAIN             0x3D
#define RT5647_REC_MIXER_R_CTRL             0x3E

/* Output Mixer */
#define RT5647_HP_L_MIXER                   0x3F
#define RT5647_HP_L_MIXER_MUTE              0x40
#define RT5647_HP_R_MIXER                   0x41
#define RT5647_HP_R_MIXER_MUTE              0x42
#define RT5647_HPO_MIXER_CTRL               0x45
#define RT5647_SPK_L_MIXER_CTRL             0x46
#define RT5647_SPK_R_MIXER_CTRL             0x47
#define RT5647_SPO_MIXER_CTRL               0x48
#define RT5647_SPK_AMP_GAIN                 0x4A
#define RT5647_MONO_MIXER_GAIN              0x4B
#define RT5647_MONO_MIXER_CTRL              0x4C
#define RT5647_OUT_L_MIXER_GAIN1            0x4D
#define RT5647_OUT_L_MIXER_GAIN2            0x4E
#define RT5647_OUT_L_MIXER_MUTE             0x4F
#define RT5647_OUT_R_MIXER_GAIN1            0x50
#define RT5647_OUT_R_MIXER_GAIN2            0x51
#define RT5647_OUT_R_MIXER_MUTE             0x52
#define RT5647_LOUT_MIXER                   0x53

/* Hpatic Generator */
#define RT5647_HAPTIC_CTRL1                 0x56
#define RT5647_HAPTIC_CTRL2                 0x57
#define RT5647_HAPTIC_CTRL3                 0x58
#define RT5647_HAPTIC_CTRL4                 0x59
#define RT5647_HAPTIC_CTRL5                 0x5A
#define RT5647_HAPTIC_CTRL6                 0x5B
#define RT5647_HAPTIC_CTRL7                 0x5C
#define RT5647_HAPTIC_CTRL8                 0x5D
#define RT5647_HAPTIC_CTRL9                 0x5E
#define RT5647_HAPTIC_CTRL10                0x5F

/* Power Management */
#define RT5647_PWR_MGT_1                    0x61
#define RT5647_PWR_MGT_2                    0x62
#define RT5647_PWR_MGT_3                    0x63
#define RT5647_PWR_MGT_4                    0x64
#define RT5647_PWR_MGT_5                    0x65
#define RT5647_PWR_MGT_6                    0x66

/* PR Register */
#define RT5647_PR_INDEX                     0x6A
#define RT5647_PR_DATA                      0x6C

/* Digital Interface */
#define RT5647_I2S1_CTRL                    0x70
#define RT5647_I2S2_CTRL                    0x71
#define RT5647_I2S3_CTRL                    0x72
#define RT5647_ADC_DAC_CLK_CTRL             0x73
#define RT5647_ADC_DAC_HPF_CTRL             0x74

/* Digital MIC */
#define RT5647_DMIC1                        0x75
#define RT5647_DMIC2                        0x76

/* TDM */
#define RT5647_TDM1                         0x77
#define RT5647_TDM2                         0x78
#define RT5647_TDM3                         0x79

/* Global Clock */
#define RT5647_GLOBAL_CLOCK                 0x80
#define RT5647_PLL1                         0x81
#define RT5647_PLL2                         0x82
#define RT5647_ASRC1                        0x83
#define RT5647_ASRC2                        0x84
#define RT5647_ASRC3                        0x85
#define RT5647_ASRC4                        0x8A

/* HP Amp */
#define RT5647_HP_DEPOP_1                   0x8E
#define RT5647_HP_DEPOP_2                   0x8F
#define RT5647_HP_AMP_CTRL                  0xD6

/* MICBIAS */
#define RT5647_MICBIAS                      0x93

/* JD1 */
#define RT5647_JD1                          0x94

/* SPK AMP */
#define RT5647_CLS_D_AMP                    0xA0

/* EQ */
#define RT5647_ADC_EQ1                      0xAE
#define RT5647_ADC_EQ2                      0xAF
#define RT5647_DAC_EQ1                      0xB0
#define RT5647_DAC_EQ2                      0xB1

/* DRC/AGC */
#define RT5647_DRC_AGC_2                    0xB3
#define RT5647_DRC_AGC_3                    0xB4
#define RT5647_DRC_AGC_4                    0xB5
#define RT5647_DRC_AGC_5                    0xB6
#define RT5647_DRC_AGC_6                    0xB7
#define RT5647_DRC_LIMITER                  0xE7
#define RT5647_DRC_BASS_LIMITER             0xE9
#define RT5647_DRC_CTRL                     0xEA
#define RT5647_DRC_BASS_CTRL_1              0xF0
#define RT5647_DRC_BASS_CTRL_2              0xF1
#define RT5647_DRC_BASS_CTRL_3              0xF2
#define RT5647_DRC_BASS_CTRL_4              0xF3
#define RT5647_DRC_BASS_CTRL_5              0xF4

/* Jack Detection */
#define RT5647_JACK_DET_CTRL_1              0xBB
#define RT5647_JACK_DET_CTRL_2              0xBC
#define RT5647_JACK_DET_CTRL_3              0xF8
#define RT5647_JACK_DET_CTRL_4              0xF9

/* IRQ */
#define RT5647_IRQ_CTRL_1                   0xBD
#define RT5647_IRQ_CTRL_2                   0xBE
#define RT5647_IRQ_CTRL_3                   0xBF

/* GPIO */
#define RT5647_GPIO_CTRL_1                  0xC0
#define RT5647_GPIO_CTRL_2                  0xC1
#define RT5647_GPIO_CTRL_3                  0xC2
#define RT5647_GPIO_CTRL_4                  0xC3

/* SounzReal Sound effect */
#define RT5647_TDM_PCM_MODE_B               0xCF
#define RT5647_TRUTREBLE_CTRL_1             0xD0
#define RT5647_TRUTREBLE_CTRL_2             0xD1

/* Wind Filter */
#define RT5647_STO1_ADC_WIND_FILTER1        0xD3
#define RT5647_STO1_ADC_WIND_FILTER2        0xD4
#define RT5647_MONO_ADC_WIND_FILTER1        0xEC
#define RT5647_MONO_ADC_WIND_FILTER2        0xED

/* SVOL & ZCD */
#define RT5647_SOFT_VOL_ZCD_CTRL1           0xD9
#define RT5647_SOFT_VOL_ZCD_CTRL2           0xDA

/* InLine Command */
#define RT5647_INLINE_CMD_CTRL1             0xDB
#define RT5647_INLINE_CMD_CTRL2             0xDC
#define RT5647_INLINE_CMD_CTRL3             0xDD

/* General Control */
#define RT5647_GENERAL_CTRL_1               0xFA
#define RT5647_GENERAL_CTRL_2               0xFC

/* Vendor ID */
#define RT5647_VENDOR_ID                    0xFE

#define RT5647_L_MUTE                       (0x1 << 15)
#define RT5647_L_MUTE_SFT                   15
#define RT5647_VOL_L_MUTE                   (0x1 << 14)
#define RT5647_VOL_L_SFT                    14
#define RT5647_R_MUTE                       (0x1 << 7)
#define RT5647_R_MUTE_SFT                   7
#define RT5647_VOL_R_MUTE                   (0x1 << 6)
#define RT5647_VOL_R_SFT                    6
#define RT5647_L_VOL_MASK                   (0x3F << 8)
#define RT5647_L_VOL_SFT                    8
#define RT5647_R_VOL_MASK                   (0x3F)
#define RT5647_R_VOL_SFT                    0

/* DAC2 */
#define RT5647_DAC2_VOL_L_SFT               13
#define RT5647_DAC2_VOL_R_SFT               12
#define RT5647_DAC2_SEL_L_SFT               4
#define RT5647_DAC2_SEL_R_SFT               0

/* Stereo DAC Digital Mixer control (MX-2A) */
#define RT5647_DAC_L1_SFT                   14
#define RT5647_DAC_L2_SFT                   12
#define RT5647_DAC_R1_L_SFT                 9
#define RT5647_DAC_R1_SFT                   6
#define RT5647_DAC_R2_SFT                   4
#define RT5647_DAC_L1_R_SFT                 1

/* SPKMIXL control (MX-46) */
#define RT5647_SPKMIX_L_BST1_SFT            5
#define RT5647_SPKMIX_L_BST3_SFT            4
#define RT5647_SPKMIX_L_INL_SFT             3
#define RT5647_SPKMIX_L_DACL2_SFT           2
#define RT5647_SPKMIX_L_DACL1_SFT           1

/* SPKMIXR control (MX-47) */
#define RT5647_SPKMIX_R_BST2_SFT            5
#define RT5647_SPKMIX_R_BST3_SFT            4
#define RT5647_SPKMIX_R_INL_SFT             3
#define RT5647_SPKMIX_R_DACR2_SFT           2
#define RT5647_SPKMIX_R_DACR1_SFT           1

/* SPOMIX control (MX-48) */
#define RT5647_SPOMIX_L_DACL1_SFT           15
#define RT5647_SPOMIX_L_DACR1_SFT           14
#define RT5647_SPOMIX_L_SPKVOLL_SFT         13
#define RT5647_SPOMIX_L_SPKVOLR_SFT         12
#define RT5647_SPOMIX_L_BST3_SFT            11

#define RT5647_SPOMIX_R_DACR1_SFT           3
#define RT5647_SPOMIX_R_BST3_SFT            2
#define RT5647_SPOMIX_R_SPKVOLR_SFT         1

/* Haptic Generator control (MX-56) */
#define RT5645_HAPTIC_TYPE                  11
#define RT5645_HAPTIC_ACTMODE               12
#define RT5645_HAPTIC_ACTMODE_MASK          (0x3 << 12)
#define RT5645_HAPTIC_FREQ                  0

/* Power management control 1 (MX-61) */
#define RT5647_PWR1_I2S1_EN                 15
#define RT5647_PWR1_I2S2_EN                 14
#define RT5647_PWR1_I2S3_EN                 13
#define RT5647_PWR1_DACL1_EN                12
#define RT5647_PWR1_DACR1_EN                11
#define RT5647_PWR1_CLSD_R_EN               9
#define RT5647_PWR1_CLSD_L_EN               8
#define RT5647_PWR1_DACL2_EN                7
#define RT5647_PWR1_DACR2_EN                6
#define RT5647_PWR1_ADC_L_EN                2
#define RT5647_PWR1_ADC_R_EN                1
#define RT5647_PWR1_CLSD_EN                 0

/* Power management control 3 (MX-63) */
#define RT5647_PWR3_VREF1_EN                15
#define RT5647_PWR3_FASTB1_EN               14
#define RT5647_PWR3_MBIAS_EN                13
#define RT5647_PWR3_LOUT_EN                 12
#define RT5647_PWR3_BGBIAS_EN               11
#define RT5647_PWR3_MONO_EN                 10
#define RT5647_PWR3_HPL_EN                  7
#define RT5647_PWR3_HPR_EN                  6
#define RT5647_PWR3_HPAMP_EN                5
#define RT5647_PWR3_VREF2_EN                4
#define RT5647_PWR3_FASTB2_EN               3
#define RT5647_PWR3_LDO1_EN                 0
#define RT5647_PWR3_LDO1_MASK               (0x3 << 0)

/* Power management control 5 (MX-65) */
#define RT5647_PWR5_OUTMIXL_EN              15
#define RT5647_PWR5_OUTMIXR_EN              14
#define RT5647_PWR5_SPKMIXL_EN              13
#define RT5647_PWR5_SPKMIXR_EN              12
#define RT5647_PWR5_RECMIXL_EN              11
#define RT5647_PWR5_RECMIXR_EN              10
#define RT5647_PWR5_MONOMIX_EN              8
#define RT5647_PWR5_HPMIXL_EN               7
#define RT5647_PWR5_HPMIXR_EN               6
#define RT5647_PWR5_LDO2_EN                 1

/* Power management control 6 (MX-66) */
#define RT5647_PWR6_SPOVOLL_EN              15
#define RT5647_PWR6_SPOVOLR_EN              14
#define RT5647_PWR6_OUTVOLL_EN              13
#define RT5647_PWR6_OUTVOLR_EN              12
#define RT5647_PWR6_HPOVOLL_EN              11
#define RT5647_PWR6_HPOVOLR_EN              10
#define RT5647_PWR6_INLVOL_EN               9
#define RT5647_PWR6_INRVOL_EN               8
#define RT5647_PWR6_MONOVOL_EN              7
#define RT5647_PWR6_MICINDET_EN             5

/* I2S1/I2S2/I2S3 control (MX-70,71,72) */
#define RT5647_I2S_MODE                     15
#define RT5647_I2S_MODE_MASTER              (0x0 << 15)
#define RT5647_I2S_MODE_SLAVE               (0x1 << 15)
#define RT5647_I2S_OUT_COMP                 10
#define RT5647_I2S_OUT_COMP_MASK            (0x3 << 10)
#define RT5647_I2S_IN_COMP                  8
#define RT5647_I2S_IN_COMP_MASK             (0x3 << 8)
#define RT5647_I2S_INV_BCLK                 7
#define RT5647_I2S_LEN                      2
#define RT5647_I2S_LEN_MASK                 (0x3 << 2)
#define RT5647_I2S_LEN_16                   (0x0 << 2)
#define RT5647_I2S_LEN_20                   (0x1 << 2)
#define RT5647_I2S_LEN_24                   (0x2 << 2)
#define RT5647_I2S_LEN_8                    (0x3 << 2)
#define RT5647_I2S_FORMAT                   0
#define RT5647_I2S_FORMAT_MASK              (0x3 << 0)
#define RT5647_I2S_FORMAT_I2S               0
#define RT5647_I2S_FORMAT_LEFT_J            1
#define RT5647_I2S_FORMAT_PCM_A             2
#define RT5647_I2S_FORMAT_PCM_B             3

/* TDM interface control 1 (MX-77) */
#define RT5647_TDM_MODE_SEL                 14
#define RT5647_TDM_MODE_I2S                 (0 << 14)
#define RT5647_TDM_MODE_TDM                 (1 << 14)
#define RT5647_TDM_SLOTNUM_SEL              12
#define RT5647_TDM_SLOTNUM_SEL_MASK         (0x3 << 12)
#define RT5647_TDM_SLOTNUM_2CH              (0x0 << 12)
#define RT5647_TDM_SLOTNUM_4CH              (0x1 << 12)
#define RT5647_TDM_SLOTNUM_6CH              (0x2 << 12)
#define RT5647_TDM_SLOTNUM_8CH              (0x3 << 12)
#define RT5647_TDM_CH_LEN                   10
#define RT5647_TDM_CH_LEN_MASK              (0x3 << 10)
#define RT5647_TDM_CH_LEN_16                (0x0 << 10)
#define RT5647_TDM_CH_LEN_20                (0x1 << 10)
#define RT5647_TDM_CH_LEN_24                (0x2 << 10)
#define RT5647_TDM_CH_LEN_32                (0x3 << 10)
#define RT5647_TDM_RXDAC_SEL                8
#define RT5647_TDM_RXDAC_SEL_MASK           (0x3 << 8)
#define RT5647_TDM_RXCH2_SWAP               6
#define RT5647_TDM_RXCH2_SWAP_MASK          (0x3 << 6)
#define RT5647_TDM_RXCH4_SWAP               4
#define RT5647_TDM_RXCH4_SWAP_MASK          (0x3 << 4)
#define RT5647_TDM_RXCH6_SWAP               2
#define RT5647_TDM_RXCH6_SWAP_MASK          (0x3 << 2)
#define RT5647_TDM_RXCH8_SWAP               0
#define RT5647_TDM_RXCH8_SWAP_MASK          (0x3 << 0)
#define RT5647_TDM_SWAP_LR                  0
#define RT5647_TDM_SWAP_RL                  1
#define RT5647_TDM_SWAP_LL                  2
#define RT5647_TDM_SWAP_RR                  3

/* TDM interface control 2 (MX-78) */
#define RT5647_TDM_LRCK_POL                 15
#define RT5647_TDM_LRCK_PULSE               11
#define RT5647_TDM_OUTL2_MUTE               7
#define RT5647_TDM_OUTR2_MUTE               6
#define RT5647_TDM_OUTL4_MUTE               5
#define RT5647_TDM_OUTR4_MUTE               4
#define RT5647_TDM_OUTL6_MUTE               3
#define RT5647_TDM_OUTR6_MUTE               2
#define RT5647_TDM_OUTL8_MUTE               1
#define RT5647_TDM_OUTR8_MUTE               0

/* TDM interface control 3 (MX-79) */
#define RT5647_TDM_TXL_CH2                  12
#define RT5647_TDM_TXL_CH2_MASK             (0x7 << 12)
#define RT5647_TDM_TXR_CH2                  8
#define RT5647_TDM_TXR_CH2_MASK             (0x7 << 8)
#define RT5647_TDM_TXL_CH4                  4
#define RT5647_TDM_TXL_CH4_MASK             (0x7 << 4)
#define RT5647_TDM_TXR_CH4                  0
#define RT5647_TDM_TXR_CH4_MASK             (0x7 << 0)
#define RT5647_TDM_SLOT0                    0
#define RT5647_TDM_SLOT1                    1
#define RT5647_TDM_SLOT2                    2
#define RT5647_TDM_SLOT3                    3
#define RT5647_TDM_SLOT4                    4
#define RT5647_TDM_SLOT5                    5
#define RT5647_TDM_SLOT6                    6
#define RT5647_TDM_SLOT7                    7

/* Global Clock Control (MX-80) */
#define RT5647_SYSCLK_S_SFT                 14
#define RT5647_SYSCLK_S_MASK                (0x3 << 14)
#define RT5647_SYSCLK_S_MCLK                (0 << 14)
#define RT5647_SYSCLK_S_PLL                 (1 << 14)

#define RT5647_PLL_S_SFT                    11
#define RT5647_PLL_S_MASK                   (0x7 << 11)
#define RT5647_PLL_S_MCLK                   (0 << 11)
#define RT5647_PLL_S_BCLK1                  (1 << 11)
#define RT5647_PLL_S_BCLK2                  (2 << 11)
#define RT5647_PLL_S_BCLK3                  (3 << 11)
#define RT5647_PLL_S_INTCLK                 (4 << 11)

#define RT5647_PLL_PREDIV_SFT               3
#define RT5647_PLL_PREDIV_DIV1              (0 << 3)
#define RT5647_PLL_PREDIV_DIV2              (1 << 3)

#define RT5647_SYSCLK_DIV_DA_FILTER         0
#define RT5647_SYSCLK_DIV_DA_FILTER_MASK    (0x7 << 0)

/* PLL Control 1 (MX-81) */
#define RT5647_PLL_N_CODE_SFT               7
#define RT5647_PLL_N_CODE_MASK              (0x1FF << 7)
#define RT5647_PLL_N_CODE_MAX               0x1FF
#define RT5647_PLL_K_CODE_SFT               0
#define RT5647_PLL_K_CODE_MASK              (0x1F << 0)
#define RT5647_PLL_K_CODE_MAX               0x1F

/* PLL Control 2 (MX-82) */
#define RT5647_PLL_M_CODE_SFT               12
#define RT5647_PLL_M_CODE_MASK              (0xF << 12)
#define RT5647_PLL_M_CODE_MAX               0xF
#define RT5647_PLL_M_BYPASS_SFT             11

/* RT5647 vendor id (MX-FE) */
#define RT5647_DEFAULT_VID                  0x10EC

#define RT5647_FORMATS (GB_AUDIO_PCM_FMT_S8 | GB_AUDIO_PCM_FMT_S16_LE | \
                       GB_AUDIO_PCM_FMT_S24_LE)

#define RT5647_STEREO_RATES (GB_AUDIO_PCM_RATE_8000 | GB_AUDIO_PCM_RATE_11025 |\
                            GB_AUDIO_PCM_RATE_16000 | GB_AUDIO_PCM_RATE_22050 |\
                            GB_AUDIO_PCM_RATE_32000 | GB_AUDIO_PCM_RATE_44100 |\
                            GB_AUDIO_PCM_RATE_48000 | GB_AUDIO_PCM_RATE_64000 |\
                            GB_AUDIO_PCM_RATE_88200 | GB_AUDIO_PCM_RATE_96000 |\
                            GB_AUDIO_PCM_RATE_176400 | GB_AUDIO_PCM_RATE_192000)

static int rt5647_speaker_event(struct device *dev, uint8_t widget_id,
                                uint8_t event);
int rt5647_playback_vol_set(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);
int rt5647_playback_vol_get(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);

/* TODO: Turn into a list to support > 1 codec instance */
static struct device *codec_dev;

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
 * rt5647 codec register structure
 */
struct rt5647_reg {
    /** register address */
    uint8_t reg;
    /** register value */
    uint16_t val;
};

/**
 * rt5647 pll code
 */
struct pll_code {
    /** PLL_M_CODE */
    int m;
    /** PLL_N_CODE */
    int n;
    /** PLL_K_CODE */
    int k;
    /** PLL_M_BYPASS */
    int bp;
};

/**
 * rt5647 private information
 */
struct rt5647_info {
    /** Driver model representation of the device */
    struct device *dev;
    /** i2c device handle */
    struct device *i2c;
    /** i2c slave address (7-bits) */
    uint8_t i2c_addr;
    /** codec name */
    uint8_t name[AUDIO_CODEC_NAME_MAX];
    /** device state */
    int state;

    /** enable TDM mode */
    int tdm_en;

    /** rt5647 codec initialization array */
    struct rt5647_reg *init_regs;
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
struct rt5647_reg rt5647_init_regs[] = {
    { RT5647_PR_INDEX, 0x003d },
    { RT5647_PR_DATA, 0x3600 }, /* turn on ADC/DAC clock generator */

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
    { RT5647_STO1_ADC_DIGI_VOL, 0xAFAF },/* Mute STO1 ADC for depop */

    /* power */
    { RT5647_PWR_MGT_2, 0x0E00 }, /* turn on filter power */
    { RT5647_PWR_MGT_4, 0x0200 }, /* turn on PLL power */
    { RT5647_PWR_MGT_5, 0x3002 }, /* turn on LDO2 power */
    { RT5647_CLS_D_AMP, 0xA0E8 }, /* enable auto powerdown when over current */
    { RT5647_GENERAL_CTRL_2, 0x4C00 }, /* Turn off Class D AMP when No mclk */
#ifdef ENABLE_HAPTIC_TEST
    /* turn on Haptic generator control for testing */
    { RT5647_HAPTIC_CTRL1, 0x2888 }, /* AC and 888Hz */
#endif

#ifdef MIN_SPEAKER_SUPPORT
    { RT5647_DACL2_R2_DIGI_VOL, 0xA1A1 },
    { RT5647_PWR_MGT_1, 0x9BC1 },
    { RT5647_DACL2_R2_DIGI_MUTE, 0x0000 },
    { RT5647_PWR_MGT_5, 0x3002 },
    { RT5647_PWR_MGT_6, 0xC000 },
#endif
};

/**
 * DAI device table
 */
struct audio_dai rt5647_dais[] = {
    {
        .dai = {
            .name = "rt5647-aif1",
            .data_cport = 0,
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
#ifdef MIN_SPEAKER_SUPPORT
    RT5647_CTL_PLAYBACK_MUTE,
    RT5647_CTL_PLAYBACK_VOL,
    RT5647_CTL_SPKAMP_SWITCH,
#endif
    RT5647_CTL_MAX
};

/**
 * audio widget id
 */
enum {
#ifndef MIN_SPEAKER_SUPPORT
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
#else
    RT5647_WIDGET_SPK,
    RT5647_WIDGET_SPKOUT,
    RT5647_WIDGET_SPK_AMP_SWITCH,
    RT5647_WIDGET_SPK_DAC,
    RT5647_WIDGET_SPK_MIXER,
    RT5647_WIDGET_AIF1RX,
#endif
    RT5647_WIDGET_MAX
};

/**
 * audio control list
 */
struct audio_control rt5647_controls[] = {
#ifndef MIN_SPEAKER_SUPPORT
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
#else
    AUDCTL_BITS("Playback Mute", RT5647_CTL_PLAYBACK_MUTE, MIXER,
                RT5647_SPKOUT_VOL, RT5647_L_MUTE_SFT, RT5647_R_MUTE_SFT, 1),
    /*
     * Note that actual reg max is 0xAF = 0 dB but this can burn out the
     * speaker of ARA white module.  Changing the max to 0xA1 = -5.25 db
     * doesn't warm up the speaker much but it is too quiet.  So, change
     * it to 0xA7 = -3.0 db to keep the max volume fairly loud but not
     * heat up the speaker too much.  WARNING: The speaker still gets hot
     * after several minutes at the max.
     */
    AUDCTL_BITSVE("Playback Volume", RT5647_CTL_PLAYBACK_VOL, MIXER,
                  RT5647_DACL2_R2_DIGI_VOL, RT5647_L_VOL_SFT, RT5647_R_VOL_SFT,
                  0, 8, 0xA7, 0x7F, 0, rt5647_playback_vol_get,
                  rt5647_playback_vol_set),
#endif
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

#ifdef MIN_SPEAKER_SUPPORT
struct audio_control rt5647_spk_amp_switch[] = {
    AUDCTL_DUMMY("Switch", RT5647_CTL_SPKAMP_SWITCH, MIXER,
                 DUMMY_REG, DUMMY_BIT_00, 0),
};
#endif

/**
 * audio widget table
 */
struct audio_widget rt5647_widgets[] = {
#ifndef MIN_SPEAKER_SUPPORT
    WIDGET_S("AIF1TX", "AIF1 Capture", RT5647_WIDGET_AIF1TX, AIF_OUT, NULL, 0,
             NOPWRCTL, 0, 0),
    WIDGET("IF1 ADC", RT5647_WIDGET_IF1_ADC, PGA, NULL, 0, NOPWRCTL, 0, 0),

    WIDGET_S("AIF1RX", "AIF1 Playback", RT5647_WIDGET_AIF1RX, AIF_IN, NULL, 0,
             NOPWRCTL, 0, 0),

    WIDGET("I2S1", RT5647_WIDGET_I2S1, SUPPLY, NULL, 0, RT5647_PWR_MGT_1,
           RT5647_PWR1_I2S1_EN, 0),

    WIDGET("IF1 DAC1", RT5647_WIDGET_IF1_DAC1, PGA, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC1 L", RT5647_WIDGET_IF1_DAC1L, PGA, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC1 R", RT5647_WIDGET_IF1_DAC1R, PGA, NULL, 0, NOPWRCTL, 0, 0),

    WIDGET("IF1 DAC2", RT5647_WIDGET_IF1_DAC2, PGA, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC2 L", RT5647_WIDGET_IF1_DAC2L, PGA, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("IF1 DAC2 R", RT5647_WIDGET_IF1_DAC2R, PGA, NULL, 0, NOPWRCTL, 0, 0),

    WIDGET("DAC L2 MUX", RT5647_WIDGET_DACL2_MUX, MUX, rt5647_dac_l2_mux,
           ARRAY_SIZE(rt5647_dac_l2_mux),NOPWRCTL, 0, 0),
    WIDGET("DAC R2 MUX", RT5647_WIDGET_DACR2_MUX, MUX, rt5647_dac_r2_mux,
           ARRAY_SIZE(rt5647_dac_r2_mux), NOPWRCTL, 0, 0),

    WIDGET("DAC L2 Volume", RT5647_WIDGET_DACL2_VOL, PGA, NULL,
           0, RT5647_PWR_MGT_1, RT5647_PWR1_DACL2_EN, 0),
    WIDGET("DAC R2 Volume", RT5647_WIDGET_DACR2_VOL, PGA, NULL,
           0, RT5647_PWR_MGT_1, RT5647_PWR1_DACR2_EN, 0),

    WIDGET("Stereo DAC MIXL", RT5647_WIDGET_STODAC_MIXL, MIXER,
           rt5647_stereo_dac_l_mix, ARRAY_SIZE(rt5647_stereo_dac_l_mix),
           NOPWRCTL, 0, 0),
    WIDGET("Stereo DAC MIXR", RT5647_WIDGET_STODAC_MIXR, MIXER,
           rt5647_stereo_dac_r_mix, ARRAY_SIZE(rt5647_stereo_dac_r_mix),
           NOPWRCTL, 0, 0),

    WIDGET("DAC L1", RT5647_WIDGET_DAC_L1, DAC, NULL, 0, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACL1_EN, 0),
    WIDGET("DAC L2", RT5647_WIDGET_DAC_L2, DAC, NULL, 0, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACL2_EN, 0),
    WIDGET("DAC R1", RT5647_WIDGET_DAC_R1, DAC, NULL, 0, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACR1_EN, 0),
    WIDGET("DAC R2", RT5647_WIDGET_DAC_R2, DAC, NULL, 0, RT5647_PWR_MGT_1,
           RT5647_PWR1_DACR2_EN, 0),

    WIDGET("SPK MIXL", RT5647_WIDGET_SPK_MIXL, MIXER, rt5647_spk_l_mix,
           ARRAY_SIZE(rt5647_spk_l_mix), RT5647_PWR_MGT_5,
           RT5647_PWR5_SPKMIXL_EN, 0),
    WIDGET("SPK MIXR", RT5647_WIDGET_SPK_MIXR, MIXER, rt5647_spk_r_mix,
           ARRAY_SIZE(rt5647_spk_r_mix), RT5647_PWR_MGT_5,
           RT5647_PWR5_SPKMIXR_EN, 0),

    WIDGET("SPKVOL L", RT5647_WIDGET_SPKVOLL, PGA, NULL, 0, RT5647_PWR_MGT_6,
           RT5647_PWR6_SPOVOLL_EN, 0),
    WIDGET("SPKVOL R", RT5647_WIDGET_SPKVOLR, PGA, NULL, 0, RT5647_PWR_MGT_6,
           RT5647_PWR6_SPOVOLR_EN, 0),

    WIDGET("SPOL MIX", RT5647_WIDGET_SPOL_MIX, MIXER, rt5647_spo_l_mix,
           ARRAY_SIZE(rt5647_spo_l_mix), NOPWRCTL, 0, 0),
    WIDGET("SPOR MIX", RT5647_WIDGET_SPOR_MIX, MIXER, rt5647_spo_r_mix,
           ARRAY_SIZE(rt5647_spo_r_mix), NOPWRCTL, 0, 0),

    WIDGET_E("SPK amp", RT5647_WIDGET_SPK_AMP, PGA, NULL, 0, NOPWRCTL, 0, 0,
             rt5647_speaker_event),

    WIDGET("SPOL", RT5647_WIDGET_SPOL, OUTPUT, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("SPOR", RT5647_WIDGET_SPOR, OUTPUT, NULL, 0, NOPWRCTL, 0, 0),
#else
    WIDGET("Spk", RT5647_WIDGET_SPK, SPK, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("SPKOUT", RT5647_WIDGET_SPKOUT, OUTPUT, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("SPK DAC", RT5647_WIDGET_SPK_DAC, PGA, NULL, 0, NOPWRCTL, 0, 0),
    WIDGET("SPK MIXER", RT5647_WIDGET_SPK_MIXER, MIXER, NULL, 0, NOPWRCTL, 0,
           0),
    WIDGET_S("AIF1RX", "GB Audio Playback", RT5647_WIDGET_AIF1RX, AIF_IN, NULL,
             0, NOPWRCTL, 0, 0),
    WIDGET_E("SPK Amp", RT5647_WIDGET_SPK_AMP_SWITCH, SWITCH,
             rt5647_spk_amp_switch, ARRAY_SIZE(rt5647_spk_amp_switch),
             NOPWRCTL, 0, 0, rt5647_speaker_event),
#endif
};

/**
 * audio route table
 */
audio_route rt5647_routes[] = {
#ifndef MIN_SPEAKER_SUPPORT
    /* AIF1RX */
    { RT5647_WIDGET_AIF1RX, RT5647_WIDGET_IF1_DAC1, NOCONTROL, 0 },
    { RT5647_WIDGET_AIF1RX, RT5647_WIDGET_IF1_DAC2, NOCONTROL, 0 },

    /* I2S1 */
    { RT5647_WIDGET_I2S1, RT5647_WIDGET_IF1_ADC, NOCONTROL, 0 },
    { RT5647_WIDGET_I2S1, RT5647_WIDGET_IF1_DAC1, NOCONTROL, 0 },
    { RT5647_WIDGET_I2S1, RT5647_WIDGET_IF1_DAC2, NOCONTROL, 0 },

    /* IF1 DAC1 */
    { RT5647_WIDGET_IF1_DAC1, RT5647_WIDGET_IF1_DAC1L, NOCONTROL, 0 },
    { RT5647_WIDGET_IF1_DAC1, RT5647_WIDGET_IF1_DAC1R, NOCONTROL, 0 },
    /* IF1 DAC2 */
    { RT5647_WIDGET_IF1_DAC2, RT5647_WIDGET_IF1_DAC2L, NOCONTROL, 0 },
    { RT5647_WIDGET_IF1_DAC2, RT5647_WIDGET_IF1_DAC2R, NOCONTROL, 0 },

    /* IF1 DAC2 L */
    { RT5647_WIDGET_IF1_DAC2L, RT5647_WIDGET_DACL2_MUX,
      RT5647_CTL_DAC2_LSRC, 0 },
    /* IF1 DAC2 R */
    { RT5647_WIDGET_IF1_DAC2R, RT5647_WIDGET_DACR2_MUX,
#ifndef ENABLE_HAPTIC_TEST
      RT5647_CTL_DAC2_RSRC, 0 },
#else
      RT5647_CTL_DAC2_RSRC, 4 }, /* input source : haptic control */
#endif /* ENABLE_HAPTIC_TEST */

    /* DAC L2 Mux */
    { RT5647_WIDGET_DACL2_MUX, RT5647_WIDGET_DACL2_VOL, NOCONTROL, 0 },
    /* DAC R2 Mux */
    { RT5647_WIDGET_DACR2_MUX, RT5647_WIDGET_DACR2_VOL, NOCONTROL, 0 },


    /* DAC L2 Volume */
    { RT5647_WIDGET_DACL2_VOL, RT5647_WIDGET_STODAC_MIXL,
      RT5647_CTL_DACL2_MIXL, 0 },
    /* DAC R2 Volume */
    { RT5647_WIDGET_DACR2_VOL, RT5647_WIDGET_STODAC_MIXR,
      RT5647_CTL_DACR2_MIXR, 0 },

    /* Stereo DAC MIXL */
    { RT5647_WIDGET_STODAC_MIXL, RT5647_WIDGET_DAC_L1, NOCONTROL, 0 },
    /* Stereo DAC MIXR */
    { RT5647_WIDGET_STODAC_MIXR, RT5647_WIDGET_DAC_R1, NOCONTROL, 0 },

    /* DAC L1 */
    { RT5647_WIDGET_DAC_L1, RT5647_WIDGET_SPK_MIXL, RT5647_CTL_SPKL_DACL1, 0 },
    /*
    { RT5647_WIDGET_DAC_L1, RT5647_WIDGET_SPOL_MIX, RT5647_CTL_SPOL_DACL1, 0 } ,
    */
    /* DAC R1 */
    { RT5647_WIDGET_DAC_R1, RT5647_WIDGET_SPK_MIXR, RT5647_CTL_SPKR_DACR1, 0 },
    /*
    { RT5647_WIDGET_DAC_R1, RT5647_WIDGET_SPOL_MIX, RT5647_CTL_SPOL_DACR1, 0 },
    { RT5647_WIDGET_DAC_R1, RT5647_WIDGET_SPOR_MIX, RT5647_CTL_SPOR_DACR1, 0 },
    */

    /* SPK MIXL */
    { RT5647_WIDGET_SPK_MIXL, RT5647_WIDGET_SPKVOLL, NOCONTROL, 0 },
    /* SPK MIXR */
    { RT5647_WIDGET_SPK_MIXR, RT5647_WIDGET_SPKVOLR, NOCONTROL, 0 },

    /* SPKVOL L */
    { RT5647_WIDGET_SPKVOLL, RT5647_WIDGET_SPOL_MIX,
      RT5647_CTL_SPOL_SPKVOLL, 0 },
    /* SPKVOL R */
    { RT5647_WIDGET_SPKVOLR, RT5647_WIDGET_SPOL_MIX,
      RT5647_CTL_SPOL_SPKVOLR, 0 },
    { RT5647_WIDGET_SPKVOLR, RT5647_WIDGET_SPOR_MIX,
      RT5647_CTL_SPOR_SPKVOLR, 0 },

    /* SPOL MIX */
    { RT5647_WIDGET_SPOL_MIX, RT5647_WIDGET_SPK_AMP, NOCONTROL, 0 },
    /* SPOR MIX */
    { RT5647_WIDGET_SPOR_MIX, RT5647_WIDGET_SPK_AMP, NOCONTROL, 0 },

    /* SPK amp */
    { RT5647_WIDGET_SPK_AMP, RT5647_WIDGET_SPOL, NOCONTROL, 0 },
    { RT5647_WIDGET_SPK_AMP, RT5647_WIDGET_SPOR, NOCONTROL, 0 },
#else
    /* Playback path */
    { RT5647_WIDGET_SPKOUT, RT5647_WIDGET_SPK, NOCONTROL, 0 },
    { RT5647_WIDGET_SPK_AMP_SWITCH, RT5647_WIDGET_SPKOUT, NOCONTROL, 0 },
    { RT5647_WIDGET_SPK_DAC, RT5647_WIDGET_SPK_AMP_SWITCH, RT5647_CTL_SPKAMP_SWITCH, NOINDEX},
    { RT5647_WIDGET_SPK_MIXER, RT5647_WIDGET_SPK_DAC, NOCONTROL, 0 },
    { RT5647_WIDGET_AIF1RX, RT5647_WIDGET_SPK_MIXER, NOCONTROL, 0},
#endif
};

/**
 * @brief get codec driver device handle
 *
 * @return codec driver handle on success, NULL on error
 */
struct device *get_codec_dev()
{
    return codec_dev;
}

/**
 * @brief get codec-specific register read function
 *
 * @param dev - pointer to structure of device data
 * @return read() function pointer on success, NULL on error
 */
void *get_codec_read_func(struct device *dev)
{
    struct rt5647_info *info = NULL;

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
void *get_codec_write_func(struct device *dev)
{
    struct rt5647_info *info = NULL;

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
static uint32_t rt5647_codec_hw_read(uint32_t reg, uint32_t *value)
{
    struct device *dev = get_codec_dev();
    struct rt5647_info *info = NULL;
    uint8_t cmd = 0;
    uint16_t data = 0;

    /*
     * rt5647 i2c read format :
     * [SA + W] + [DA] + [SA + R] + [DATA_HIGH] + [DATA_LOW]
     */
    struct device_i2c_request msg[] = {
        {
            .flags = 0,
            .buffer = &cmd,
            .length = 1,
        },
        {
            .flags = I2C_FLAG_READ,
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

    msg[0].addr = info->i2c_addr;
    msg[1].addr = info->i2c_addr;

    cmd = (uint8_t)reg;

    if (device_i2c_transfer(info->i2c, msg, 2)) {
        return -EIO;
    }

    *value = (uint32_t) (data & 0xFF) << 8 | ((data >> 8) & 0xFF);

    return 0;
}

/**
 * @brief write data to codec register
 *
 * @param reg - rt5645 codec register
 * @param value - register value that we want to write
 * @return 0 on success, negative errno on error
 */
static uint32_t rt5647_codec_hw_write(uint32_t reg, uint32_t value)
{
    struct device *dev = get_codec_dev();
    struct rt5647_info *info = NULL;
    uint8_t cmd[3] = {0x00, 0x00, 0x00};

    /*
     * rt5647 i2c write format :
     * [SA + W] + [DA] + [DATA_HIGH] + [DATA_LOW]
     */

    struct device_i2c_request msg[] = {
        {
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

    msg[0].addr = info->i2c_addr;

    cmd[0] = (uint8_t)reg;
    cmd[1] = (uint8_t)((value >> 8) & 0xFF);
    cmd[2] = (uint8_t)(value & 0xFF);

    if (device_i2c_transfer(info->i2c, msg, 1)) {
        return -EIO;
    }

    dbg_verbose("I2C-W %02X %04X\n", reg, value);

    return 0;
}

/**
 * @brief read data from codec private register
 *
 * @param reg - rt5645 codec PR-xx register
 * @param value - data buffer of read
 * @return 0 on success, negative errno on error
 */
static uint32_t __attribute__((__unused__))
rt5647_codec_hw_pr_read(uint32_t reg, uint32_t *value)
{
    if (rt5647_codec_hw_write(RT5647_PR_INDEX, reg)) {
        return -EIO;
    }

    if (rt5647_codec_hw_read(RT5647_PR_DATA, value)) {
        return -EIO;
    }

    dbg_verbose("I2C-R PR-%02X %04X\n", reg, *value);

    return 0;
}

/**
 * @brief write data to codec private register
 *
 * @param reg - rt5645 codec PR-xx register
 * @param value - register value that we want to write
 * @return 0 on success, negative errno on error
 */
static uint32_t __attribute__((__unused__))
rt5647_codec_hw_pr_write(uint32_t reg, uint32_t value)
{
    if (rt5647_codec_hw_write(RT5647_PR_INDEX, reg)) {
        return -EIO;
    }

    if (rt5647_codec_hw_write(RT5647_PR_DATA, value)) {
        return -EIO;
    }

    dbg_verbose("I2C-W PR-%02X %04X\n", reg, value);

    return 0;
}

static int get_data_cport(unsigned int bundle_index, unsigned int dai_index,
                          uint16_t *data_cport)
{
    struct device *dev;
    int ret;

    dev = device_open(DEVICE_TYPE_AUDIO_BOARD_HW, 0);
    if (!dev) {
        return ret = -EIO;
    }

    ret = device_audio_board_get_data_cport(dev, CODEC_BUNDLE, dai_index,
                                            data_cport);

    device_close(dev);

    return ret;
}

#ifdef MIN_SPEAKER_SUPPORT
int rt5647_playback_vol_get(struct audio_control *control,
                            struct gb_audio_ctl_elem_value *value)
{
    int ret = 0, volr = 0, voll = 0, regmax = 0, ctlmax = 0;
    struct bitctl *ctl = NULL;
    struct gb_audio_ctl_elem_info   *info;

    ret = codec_value_get(control, value);
    if (ret) {
        return ret;
    }

    ctl = control->priv;
    info = &control->control.info;

    regmax = ctl->max;
    ctlmax = info->value.integer.max;

    voll = value->value.integer_value[0];
    volr = value->value.integer_value[1];

    value->value.integer_value[0] = ((voll * ctlmax) + (regmax >> 1)) / regmax;
    value->value.integer_value[1] = ((volr * ctlmax) + (regmax >> 1)) / regmax;

    return 0;
}

int rt5647_playback_vol_set(struct audio_control *control,
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

    value->value.integer_value[0] = ((voll * regmax) + (ctlmax >> 1)) / ctlmax;
    value->value.integer_value[1] = ((volr * regmax) + (ctlmax >> 1)) / ctlmax;

    return codec_value_set(control, value);
}
#endif

/**
 * @brief get audio topology data size
 *
 * @param dev - pointer to structure of device data
 * @param size - size of audio topology data
 * @return 0 on success, negative errno on error
 */
static int rt5647_get_topology_size(struct device *dev, uint16_t *size)
{
    struct rt5647_info *info = NULL;
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
static int rt5647_get_topology(struct device *dev,
                               struct gb_audio_topology *topology)
{
    struct rt5647_info *info = NULL;
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

    widgets = info->widgets;

    /* fill audio widget object */
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

/**
 * @brief convert pcm format setting to bit number
 *
 * @param fmtbit - pcm format
 * @return bit number on success, negative errno on error
 */
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

    if (clk_role != DEVICE_CODEC_ROLE_SLAVE) {
        /* In current audio module, we only supported slave mode. */
        return -EOPNOTSUPP;
    } else {
        /* query for slave
          Test if mclk will work
          return full set of slave hardware capabilities
        */
        uint32_t mclk_temp = dai->mclk_freq;

        memcpy(dai, &info->dais[dai_idx].s_caps,
               sizeof(struct device_codec_dai));

        /* restore the value overwritten */
        dai->mclk_freq = mclk_temp;

        /* TODO check if the pcm->mclk value will work in slave mode*/
    }

    return 0;
}

/**
 * @brief calculate codec pll setting
 *
 * @param dev - pointer to structure of device data
 * @param infreq - mclk frequency
 * @param outfreq - sysclk frequency
 * @param code - pll_code structure, included m,n,k,bypass setting
 * @return 0 on success, negative errno on error
 */
int rt5647_pll_calc(uint32_t infreq, uint32_t outfreq, struct pll_code *code)
{
    int n = 0, n1, m = 0, m1, k = 0, bp = 0;
    int min = 0, tmp = 0, in1 = 0, in2 = 0, out1 = 0;

    if (!code) {
        return -EINVAL;
    }

    k = (outfreq + (infreq / 2)) / infreq;

    min = abs(outfreq - infreq);

    for (n1 = 0; n1 <= RT5647_PLL_N_CODE_MAX; n1++) {
        in1 = infreq / (k + 2);
        out1 = outfreq / (n1 + 2);

        tmp = abs(out1 - in1);
        if (tmp <= min) {
            bp = 1;
            n = n1;
            if (!tmp) {
                goto findout;
            }
            min = tmp;
        }

        for (m1 = 0; m1 <= RT5647_PLL_M_CODE_MAX; m1++) {
            in2 = in1 / (m1 + 2);

            tmp = abs(out1 - in2);
            if (tmp <= min) {
                bp = 0;
                n = n1;
                m = m1;
                if (!tmp) {
                    goto findout;
                }
                min = tmp;
            }
        }
    }

findout:
    code->m = m;
    code->n = n;
    code->k = k;
    code->bp = bp;

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
static int rt5647_set_config(struct device *dev, unsigned int dai_idx,
                             uint8_t clk_role, struct device_codec_pcm *pcm,
                             struct device_codec_dai *dai)
{
    struct rt5647_info *info = NULL;
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

    /* check clock setting */
    ratefreq = rt5647_rate_to_freq(pcm->rate);
    numbits = rt5647_fmtbit_to_bitnum(pcm->format);

    if (ratefreq <= 0 || numbits <= 0 || numbits < pcm->sig_bits) {
        return -EINVAL;
    }

    sysclk = 256 * ratefreq; /* 256*FS */

    ret = rt5647_pll_calc(dai->mclk_freq, sysclk, &code);
    if (ret) {
        return -EINVAL;
    }

    /* setup codec hw */
    if (clk_role & DEVICE_CODEC_ROLE_SLAVE) {
        format |= RT5647_I2S_MODE_SLAVE;
    }

    switch (dai->protocol) {
    case DEVICE_CODEC_PROTOCOL_PCM:
        format |= RT5647_I2S_FORMAT_PCM_A;
        break;
    case DEVICE_CODEC_PROTOCOL_I2S:
        format |= RT5647_I2S_FORMAT_I2S;
        if (info->tdm_en) {
            /* enable TDM mode */
            tdm1 = RT5647_TDM_MODE_TDM |
                  (RT5647_TDM_SWAP_LR << RT5647_TDM_RXCH2_SWAP);
        }
        break;
    case DEVICE_CODEC_PROTOCOL_LR_STEREO:
        format |= RT5647_I2S_FORMAT_LEFT_J;
        break;
    default:
        return -EINVAL;
    }

    switch (numbits) {
    case 8:
        format |= RT5647_I2S_LEN_8;
        if (info->tdm_en) {
            /* use 16 bits */
            tdm1 |= RT5647_TDM_CH_LEN_16;
        }
        break;
    case 16:
        format |= RT5647_I2S_LEN_16;
        if (info->tdm_en) {
            tdm1 |= RT5647_TDM_CH_LEN_16;
        }
        break;
    case 20:
        format |= RT5647_I2S_LEN_20;
        if (info->tdm_en) {
            tdm1 |= RT5647_TDM_CH_LEN_20;
        }
        break;
    case 24:
        format |= RT5647_I2S_LEN_24;
        if (info->tdm_en) {
            tdm1 |= RT5647_TDM_CH_LEN_24;
        }
        break;
    default:
        return -EINVAL;
    }

    if (info->tdm_en) {
        if (dai->wclk_polarity == DEVICE_CODEC_POLARITY_REVERSED) {
            tdm2 |= 1 << RT5647_TDM_LRCK_POL;
        }

        tdm3 = RT5647_TDM_SLOT2 << RT5647_TDM_TXL_CH2 |
               RT5647_TDM_SLOT3 << RT5647_TDM_TXR_CH2 |
               RT5647_TDM_SLOT0 << RT5647_TDM_TXL_CH4 |
               RT5647_TDM_SLOT1 << RT5647_TDM_TXR_CH4;
    }

    /* write clock setting */
    value = RT5647_SYSCLK_S_PLL | RT5647_PLL_S_MCLK; /* MCLK->PLL->SYSCLK */
    mask = RT5647_SYSCLK_S_MASK | RT5647_PLL_S_MASK;
    codec_update(RT5647_GLOBAL_CLOCK, value, mask);

    /* set n & k code */
    value = (code.n << RT5647_PLL_N_CODE_SFT) |
            (code.k << RT5647_PLL_K_CODE_SFT);
    codec_write(RT5647_PLL1, value);
    /* set m & m_bypass code */
    value = (code.m << RT5647_PLL_M_CODE_SFT) |
            (code.bp << RT5647_PLL_M_BYPASS_SFT);
    codec_write(RT5647_PLL2, value);

    if (dai_idx == 0) { /* i2s1 */
        if (info->tdm_en) {
            codec_write(RT5647_TDM1, tdm1);
            codec_write(RT5647_TDM2, tdm2);
            codec_write(RT5647_TDM3, tdm3);
        }

        codec_write(RT5647_I2S1_CTRL, format);
    }

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
static int rt5647_get_control(struct device *dev, uint8_t control_id,
                              uint8_t index,
                              struct gb_audio_ctl_elem_value *value)
{
    struct rt5647_info *info = NULL;
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

        if ((aud_ctl->control.id == control_id) &&
            ((index == GB_AUDIO_INVALID_INDEX) ||
             (ctl->index == index))) {
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
static int rt5647_set_control(struct device *dev, uint8_t control_id,
                              uint8_t index,
                              struct gb_audio_ctl_elem_value *value)
{
    struct rt5647_info *info = NULL;
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

        if ((aud_ctl->control.id == control_id) &&
            ((index == GB_AUDIO_INVALID_INDEX) ||
             (ctl->index == index))) {
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

            if (widget->event) {
                widget->event(dev, widget_id, WIDGET_EVENT_PRE_PWRUP);
            }

            if (widget->reg != NOPWRCTL) {
                /* turn on the widget */
                if (codec_read(widget->reg, &data)) {
                    return -EIO;
                }
                mask = 1 << widget->shift;
                data = (data & ~mask) | ((widget->inv)? 0 : mask);
                if (codec_write(widget->reg, data)) {
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

            if (widget->event) {
                widget->event(dev, widget_id, WIDGET_EVENT_PRE_PWRDOWN);
            }

            if (widget->reg != NOPWRCTL) {
                /* turn off the widget */
                if (codec_read(widget->reg, &data)) {
                    return -EIO;
                }

                mask = 1 << widget->shift;
                data = (data & ~mask) | ((widget->inv)? mask : 0);

                if (codec_write(widget->reg, data)) {
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

/**
 * @brief start to transfer audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int rt5647_start_tx(struct device *dev, uint32_t dai_idx)
{
    struct rt5647_info *info = NULL;

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
static int rt5647_stop_tx(struct device *dev, uint32_t dai_idx)
{
    struct rt5647_info *info = NULL;

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
static int rt5647_register_tx_callback(struct device *dev,
                                       device_codec_event_callback callback,
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

/**
 * @brief get rx delay count
 *
 * @param dev - pointer to structure of device data
 * @param delay - buffer for get delay count
 * @return 0 on success, negative errno on error
 */
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

/**
 * @brief start to receive audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int rt5647_start_rx(struct device *dev, uint32_t dai_idx)
{
    struct rt5647_info *info = NULL;

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

    if (dai_idx == 0) { /* i2s1 */
        codec_update(RT5647_PWR_MGT_1, 1 << RT5647_PWR1_I2S1_EN,
                        1 << RT5647_PWR1_I2S1_EN);
    }

    info->state |= CODEC_DEVICE_FLAG_RX_START;

    return 0;
}

/**
 * @brief stop to receive audio streaming
 *
 * @param dev - pointer to structure of device data
 * @param dai_idx - DAI index
 * @return 0 on success, negative errno on error
 */
static int rt5647_stop_rx(struct device *dev, uint32_t dai_idx)
{
    struct rt5647_info *info = NULL;

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

    if (dai_idx == 0) { /* i2s1 */
        codec_update(RT5647_PWR_MGT_1, 0 << RT5647_PWR1_I2S1_EN,
                        1 << RT5647_PWR1_I2S1_EN);
    }

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
static int rt5647_register_rx_callback(struct device *dev,
                                       device_codec_event_callback callback,
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

/**
 * @brief register jack callback event
 *
 * @param dev - pointer to structure of device data
 * @param callback - callback function for notify jack event
 * @param arg - argument for callback event
 * @return 0 on success, negative errno on error
 */
static int rt5647_register_jack_event_callback(struct device *dev,
                                    device_codec_jack_event_callback callback,
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

/**
 * @brief register button callback event
 *
 * @param dev - pointer to structure of device data
 * @param callback - callback function for notify button event
 * @param arg - argument for callback event
 * @return 0 on success, negative errno on error
 */
static int rt5647_register_button_event_callback(struct device *dev,
                                  device_codec_button_event_callback callback,
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

static int rt5647_speaker_event(struct device *dev, uint8_t widget_id,
                                uint8_t event)
{
    uint32_t mask = 0;

    switch (event) {
    case WIDGET_EVENT_POST_PWRUP:
        /* turn on Class-D power */
        mask = 1 << RT5647_PWR1_CLSD_R_EN | 1 << RT5647_PWR1_CLSD_L_EN | \
               1 << RT5647_PWR1_CLSD_EN;
        codec_update(RT5647_PWR_MGT_1, mask, mask);
        break;
    case WIDGET_EVENT_PRE_PWRDOWN:
        /* turn off Class-D power */
        codec_update(RT5647_PWR_MGT_1, 0, mask);
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
static int rt5647_codec_open(struct device *dev)
{
    struct rt5647_info *info = NULL;
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

    /* verify codec id */
    if (codec_read(RT5647_VENDOR_ID, &id) || (id != RT5647_DEFAULT_VID)) {
        /* can't read codec register or vendor id isn't correct */
        return -EIO;
    }

    /* codec power on sequence */
    codec_write(RT5647_RESET, 0);    /* software reset */

    codec_update(RT5647_PWR_MGT_3,
                    BIT(RT5647_PWR3_VREF1_EN) | BIT(RT5647_PWR3_MBIAS_EN) |
                    BIT(RT5647_PWR3_BGBIAS_EN) | BIT(RT5647_PWR3_VREF2_EN),
                    BIT(RT5647_PWR3_VREF1_EN) | BIT(RT5647_PWR3_MBIAS_EN) |
                    BIT(RT5647_PWR3_BGBIAS_EN) | BIT(RT5647_PWR3_VREF2_EN));
    usleep(10000);
    codec_update(RT5647_PWR_MGT_3,
                    BIT(RT5647_PWR3_FASTB1_EN) | BIT(RT5647_PWR3_FASTB2_EN),
                    BIT(RT5647_PWR3_FASTB1_EN) | BIT(RT5647_PWR3_FASTB2_EN));

    /* initialize audio codec */
    for (i = 0; i < info->num_regs; i++) {
        codec_write(info->init_regs[i].reg , info->init_regs[i].val);
    }

    codec_update(RT5647_PWR_MGT_3, 0x02, RT5647_PWR3_LDO1_MASK);

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
static void rt5647_codec_close(struct device *dev)
{
    struct rt5647_info *info = NULL;
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
                rt5647_stop_tx(dev, i);
            }
        }

        if (info->state & CODEC_DEVICE_FLAG_RX_START) {
            for (i = 0; i < info->num_dais; i++) {
                rt5647_stop_rx(dev, i);
            }
        }
    }

    /* disable all widget */
    widget = info->widgets;

    for (i = 0; i < info->num_widgets; i++) {
        rt5647_disable_widget(dev,widget->widget.id);
        widget++;
    }

    codec_write(RT5647_RESET, 0);    /* software reset */

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
static int rt5647_codec_probe(struct device *dev)
{
    struct rt5647_info *info = NULL;
    struct control_node *node = NULL;
    struct audio_control *controls = NULL;
    struct audio_widget *widgets = NULL;
    struct device_resource *r;
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
    strncpy((char*)info->name, RT5647_CODEC_NAME, AUDIO_CODEC_NAME_MAX - 1);
    info->name[AUDIO_CODEC_NAME_MAX - 1] = '\0';

    /* link pre-defined setting */
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

    /* enable TDM mode support */
    info->tdm_en = 1;

    /* initialize i2c bus */
    info->i2c = device_open(DEVICE_TYPE_I2C_HW, 0);
    if (!info->i2c) {
        free(info);
        return -EIO;
    }

    /* get i2c slave address */
    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_I2C_ADDR,
                                    "rt5647_i2c_addr");
    if (!r) {
        free(info);
        return -EINVAL;
    }

    info->i2c_addr = (uint8_t)r->start;

    /* assign codec register access function,
     * common codec function will use two function acces codec hardware
     */
    info->codec_read = rt5647_codec_hw_read;
    info->codec_write = rt5647_codec_hw_write;

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
static void rt5647_codec_remove(struct device *dev)
{
    struct rt5647_info *info = NULL;
    struct list_head *iter, *iter_next;
    struct control_node *ctlnode = NULL;

    if (!dev || !device_get_private(dev)) {
        return;
    }

    info = device_get_private(dev);

    if (info->state & CODEC_DEVICE_FLAG_OPEN) {
        rt5647_codec_close(dev);
    }

    if (info->i2c) {
        device_close(info->i2c);
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

static struct device_codec_type_ops rt5647_codec_type_ops = {
    .get_topology_size              = rt5647_get_topology_size,
    .get_topology                   = rt5647_get_topology,
    .get_control                    = rt5647_get_control,
    .set_control                    = rt5647_set_control,
    .enable_widget                  = rt5647_enable_widget,
    .disable_widget                 = rt5647_disable_widget,
    .get_caps                       = rt5647_get_caps,
    .set_config                     = rt5647_set_config,
    .get_tx_delay                   = rt5647_get_tx_delay,
    .start_tx                       = rt5647_start_tx,
    .stop_tx                        = rt5647_stop_tx,
    .register_tx_callback           = rt5647_register_tx_callback,
    .get_rx_delay                   = rt5647_get_rx_delay,
    .start_rx                       = rt5647_start_rx,
    .stop_rx                        = rt5647_stop_rx,
    .register_rx_callback           = rt5647_register_rx_callback,
    .register_jack_event_callback   = rt5647_register_jack_event_callback,
    .register_button_event_callback = rt5647_register_button_event_callback,
};

static struct device_driver_ops rt5647_codec_ops = {
    .probe      = rt5647_codec_probe,
    .remove     = rt5647_codec_remove,
    .open       = rt5647_codec_open,
    .close      = rt5647_codec_close,
    .type_ops   = &rt5647_codec_type_ops,
};

struct device_driver rt5647_codec = {
    .type       = DEVICE_TYPE_CODEC_HW,
    .name       = "rt5647",
    .desc       = "ALC5647 Audio Codec driver",
    .ops        = &rt5647_codec_ops,
};
