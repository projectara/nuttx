#ifndef _RT5647_H_
#define _RT5647_H_

#include <nuttx/device.h>
#include <nuttx/device_codec.h>

/* I2C info */
#define RT5647_I2C_ADDR                     0x1B /* 7-bits address */

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

// DAC2
#define RT5647_DAC2_VOL_L_SFT               13
#define RT5647_DAC2_VOL_R_SFT               12
#define RT5647_DAC2_SEL_L_SFT               4
#define RT5647_DAC2_SEL_R_SFT               0

// Stereo DAC Digital Mixer control (MX-2A)
#define RT5647_DAC_L1_SFT                   14
#define RT5647_DAC_L2_SFT                   12
#define RT5647_DAC_R1_L_SFT                 9
#define RT5647_DAC_R1_SFT                   6
#define RT5647_DAC_R2_SFT                   4
#define RT5647_DAC_L1_R_SFT                 1

// SPKMIXL control (MX-46)
#define RT5647_SPKMIX_L_BST1_SFT            5
#define RT5647_SPKMIX_L_BST3_SFT            4
#define RT5647_SPKMIX_L_INL_SFT             3
#define RT5647_SPKMIX_L_DACL2_SFT           2
#define RT5647_SPKMIX_L_DACL1_SFT           1

// SPKMIXR control (MX-47)
#define RT5647_SPKMIX_R_BST2_SFT            5
#define RT5647_SPKMIX_R_BST3_SFT            4
#define RT5647_SPKMIX_R_INL_SFT             3
#define RT5647_SPKMIX_R_DACR2_SFT           2
#define RT5647_SPKMIX_R_DACR1_SFT           1

// SPOMIX control (MX-48)
#define RT5647_SPOMIX_L_DACL1_SFT             15
#define RT5647_SPOMIX_L_DACR1_SFT             14
#define RT5647_SPOMIX_L_SPKVOLL_SFT           13
#define RT5647_SPOMIX_L_SPKVOLR_SFT           12
#define RT5647_SPOMIX_L_BST3_SFT              11

#define RT5647_SPOMIX_R_DACR1_SFT             3
#define RT5647_SPOMIX_R_BST3_SFT              2
#define RT5647_SPOMIX_R_SPKVOLR_SFT           1

// Power management control 1 (MX-61)
#define RT5647_PWR1_I2S1_EN                   15
#define RT5647_PWR1_I2S2_EN                   14
#define RT5647_PWR1_I2S3_EN                   13
#define RT5647_PWR1_DACL1_EN                  12
#define RT5647_PWR1_DACR1_EN                  11
#define RT5647_PWR1_CLSD_R_EN                 9
#define RT5647_PWR1_CLSD_L_EN                 8
#define RT5647_PWR1_DACL2_EN                  7
#define RT5647_PWR1_DACR2_EN                  6
#define RT5647_PWR1_ADC_L_EN                  2
#define RT5647_PWR1_ADC_R_EN                  1
#define RT5647_PWR1_CLSD_EN                   0

// Power management control 5 (MX-65)
#define RT5647_PWR5_OUTMIXL_EN                15
#define RT5647_PWR5_OUTMIXR_EN                14
#define RT5647_PWR5_SPKMIXL_EN                13
#define RT5647_PWR5_SPKMIXR_EN                12
#define RT5647_PWR5_RECMIXL_EN                11
#define RT5647_PWR5_RECMIXR_EN                10
#define RT5647_PWR5_MONOMIX_EN                8
#define RT5647_PWR5_HPMIXL_EN                 7
#define RT5647_PWR5_HPMIXR_EN                 6
#define RT5647_PWR5_LDO2_EN                   1

// Power management control 6 (MX-66)
#define RT5647_PWR6_SPOVOLL_EN                15
#define RT5647_PWR6_SPOVOLR_EN                14
#define RT5647_PWR6_OUTVOLL_EN                13
#define RT5647_PWR6_OUTVOLR_EN                12
#define RT5647_PWR6_HPOVOLL_EN                11
#define RT5647_PWR6_HPOVOLR_EN                10
#define RT5647_PWR6_INLVOL_EN                 9
#define RT5647_PWR6_INRVOL_EN                 8
#define RT5647_PWR6_MONOVOL_EN                7
#define RT5647_PWR6_MICINDET_EN               5

#define RT5647_FORMATS (GB_AUDIO_PCM_FMT_S8 | GB_AUDIO_PCM_FMT_S16_LE | \
                       GB_AUDIO_PCM_FMT_S24_LE)

#define RT5647_STEREO_RATES (GB_AUDIO_PCM_RATE_8000 | GB_AUDIO_PCM_RATE_11025 |\
                            GB_AUDIO_PCM_RATE_16000 | GB_AUDIO_PCM_RATE_22050 |\
                            GB_AUDIO_PCM_RATE_32000 | GB_AUDIO_PCM_RATE_44100 |\
                            GB_AUDIO_PCM_RATE_48000 | GB_AUDIO_PCM_RATE_64000 |\
                            GB_AUDIO_PCM_RATE_88200 | GB_AUDIO_PCM_RATE_96000 |\
                            GB_AUDIO_PCM_RATE_176400 | GB_AUDIO_PCM_RATE_192000)

#define NOPWRCTL        (0xFFFFFFFF)    /* No power control for widget */

/* audcodec_xxx_get()/audcodec_xxx_set() for audio control */
static int audcodec_bit_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
static int audcodec_bit_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
static int audcodec_bits_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
static int audcodec_bits_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
static int audcodec_value_get(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);
static int audcodec_value_set(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);
static int audcodec_enum_get(struct audio_control *control,
                                   struct gb_audio_ctl_elem_value *value);
static int audcodec_enum_set(struct audio_control *control,
                                   struct gb_audio_ctl_elem_value *value);

/* Audio control Macro */

#define BITSRVCTL(xreg, xreg2, xshift, xshift2, xinv, xmask, xmax, xmin) \
    ((void*)&(struct bitctl) { \
        .reg = xreg, .reg2 = xreg2, .shift = xshift, .shift2 = xshift2, \
        .inv = xinv, .mask = (0x1 << xmask) - 1, .max = xmax, .min = xmin, \
    })

#define BITSRCTL(xreg, xreg2, xshift, xshift2, xinv) \
    BITSRVCTL(xreg, xreg2, xshift, xshift2, xinv, 1, 1, 0)

#define BITSCTL(xreg, xshift, xshift2, xinv) \
    BITSRVCTL(xreg, xreg, xshift, xshift2, xinv, 1, 1, 0)

#define BITCTL(xreg, xshift, xinv) \
    BITSRVCTL(xreg, xreg, xshift, xshift, xinv, 1, 1, 0)

#define ENUMCTL(xreg, xreg2, xshift, xshift2, xmask, xmax, xtexts, xvalues) \
    ((void*)&(struct enumctl) { \
        .reg = xreg, .reg2 = xreg2, .shift = xshift, .shift2 = xshift2, \
        .mask = (0x1 << xmask) - 1, .max = xmax, .texts = xtexts, \
        .values = xvalues, \
    })

#define AUDCTL_BIT(xname, xid, xiface, xreg, xshift, xinv) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_bit_get, .set = audcodec_bit_set, \
        .priv = BITCTL(xreg, xshift, xinv), \
    }

#define AUDCTL_BITS(xname, xid, xiface, xreg, xshift, xshift2, xinv) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_bits_get, .set = audcodec_bits_set, \
        .priv = BITSCTL(xreg, xshift, xshift2, xinv) \
    }

#define AUDCTL_BITSR(xname, xid, xiface, xreg, xshift, xreg2, xshift2, xinv) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = (control_get*)audcodec_bits_get, .set = (control_set *)audcodec_bits_set, \
        .priv = BITSRCTL(xreg, xreg2, xshift, xshift2, xinv) \
    }

#define AUDCTL_BITSRV(xname, xid, xiface, xreg, xshift, xreg2, xshift2, xinv, \
                      xmask, xmax, xmin) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_value_get, .set = audcodec_value_set, \
        .priv = BITSRVCTL(xreg, xreg2, xshift, xshift2, xinv, xmask, xmax, \
                          xmin) \
    }

#define AUDCTL_BITSV(xname, xid, xiface, xreg, xshift, xshift2, xinv, \
                      xmask, xmax, xmin) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_value_get, .set = audcodec_value_set, \
        .priv = BITSRVCTL(xreg, xreg, xshift, xshift2, xinv, xmask, xmax, \
                          xmin) \
    }

#define AUDCTL_BITV(xname, xid, xiface, xreg, xshift, xinv, xmask, xmax, xmin) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_value_get, .set = audcodec_value_set, \
        .priv = BITSRVCTL(xreg, xreg, xshift, xshift, xinv, xmask, xmax, \
                          xmin) \
    }

#define AUDCTL_ENUM(xname, xid, xiface, xreg, xshift, xmask, xtexts) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_bit_get, .set = audcodec_bit_set, \
        .priv = ENUMCTL(xreg, xreg, xshift, xshift, xmask, ARRAY_SIZE(xtexts), \
                        xtexts, NULL), \
    }

#define AUDCTL_ENUMS(xname, xid, xiface, xreg, xshift, xreg2, xshift2, xmask, \
                     xtexts) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_bit_get, .set = audcodec_bit_set, \
        .priv = ENUMCTL(xreg, xreg2, xshift, xshift2, xmask, \
                        ARRAY_SIZE(xtexts), xtexts, NULL), \
    }

#define AUDCTL_ENUMV(xname, xid, xiface, xreg, xshift, xmask, xtexts, xvalues) \
    { \
        .control = { \
            .name = xname, .id = xid, .iface = GB_AUDIO_IFACE_##xiface, \
        }, \
        .get = audcodec_bit_get, .set = audcodec_bit_set, \
        .priv = ENUMCTL(xreg, xreg, xshift, xshift, xmask, ARRAY_SIZE(xtexts), \
                        xtexts, xvalues), \
    }

/* Audio widget macro */
#define WIDGET(xname, xid, xtype, xcontrols, xreg, xshift, xinv) \
    { \
        .widget = { \
            .name = xname, .id = xid, .type = GB_AUDIO_WIDGET_TYPE_##xtype, \
            .state = GB_AUDIO_WIDGET_STATE_DISABLED \
        }, \
        .controls = xcontrols, .num_controls = ARRAY_SIZE(xcontrols), \
        .reg = xreg, .shift = xshift, .inv = xinv \
    }
#endif /* _RT5647_H_ */
