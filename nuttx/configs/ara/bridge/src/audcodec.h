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
#ifndef __AUDCODEC_H__
#define __AUDCODEC_H__

#include <nuttx/device.h>
#include <nuttx/device_codec.h>

#define MUXID(id, idx) ((id & 0xFF) | (idx << 8)) /* control_id macro for
                                                   * struct gb_audio_route */

#define NOCONTROL 0xFF /* no control object for struct gb_audio_route */
#define NOINDEX	0xFF /* no control object for struct gb_audio_route */

#define DUMMY_REG 0x00

#define DUMMY_BIT_00 00
#define DUMMY_BIT_01 01
#define DUMMY_BIT_02 02
#define DUMMY_BIT_03 03
#define DUMMY_BIT_04 04
#define DUMMY_BIT_05 05
#define DUMMY_BIT_06 06
#define DUMMY_BIT_07 07
#define DUMMY_BIT_08 08
#define DUMMY_BIT_09 09
#define DUMMY_BIT_10 10
#define DUMMY_BIT_11 11
#define DUMMY_BIT_12 12
#define DUMMY_BIT_13 13
#define DUMMY_BIT_14 14
#define DUMMY_BIT_15 15

/**
 * codec register access structure
 */
struct bitctl {
    /** register-1 address */
    uint32_t reg;
    /** register-2 address */
    uint32_t reg2;
    /** number of bit shift for reg-1 */
    uint32_t shift;
    /** number of bit shift for reg-2 */
    uint32_t shift2;
    /** mask value */
    uint32_t mask;
    /** invert */
    uint32_t inv;
    /** max value */
    uint32_t max;
    /** min value */
    uint32_t min;
};

struct enumctl {
    uint32_t reg;
    uint32_t reg2;
    uint32_t shift;
    uint32_t shift2;
    uint32_t max;
    uint32_t mask;
    char **texts;
    uint32_t *values;
};

struct audio_dai {
    struct gb_audio_dai dai;
    struct device_codec_dai m_caps;
    struct device_codec_dai s_caps;
    uint8_t clk_role;
    struct device_codec_pcm pcm_config;
    struct device_codec_dai dai_config;
};

struct audio_control {
    struct gb_audio_control control;
    uint32_t dummy_reg;
    void *priv;
    int (*get)(struct audio_control *ctl,
               struct gb_audio_ctl_elem_value *value);
    int (*set)(struct audio_control *ctl,
               struct gb_audio_ctl_elem_value *value);
};

struct audio_widget {
    struct gb_audio_widget widget;
    struct audio_control *controls;
    uint32_t num_controls;
    /* widget power control */
    uint32_t reg;
    uint32_t shift;
    uint32_t inv;
    int (*event)(struct device *dev, uint8_t widget_id, uint8_t event);
};

typedef struct gb_audio_route audio_route;

typedef uint32_t (*codec_read_func)(uint32_t reg, uint32_t *value);
typedef uint32_t (*codec_write_func)(uint32_t reg, uint32_t value);

struct device* get_codec_dev(void);
void* get_codec_read_func(struct device *dev);
void* get_codec_write_func(struct device *dev);

uint32_t audcodec_read(uint32_t reg, uint32_t *value);
uint32_t audcodec_write(uint32_t reg, uint32_t value);
uint32_t audcodec_update(uint32_t reg, uint32_t value, uint32_t mask);

/* audcodec_xxx_get()/audcodec_xxx_set() for audio control */
int audcodec_dummy_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
int audcodec_dummy_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
int audcodec_bit_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
int audcodec_bit_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
int audcodec_bits_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
int audcodec_bits_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value);
int audcodec_value_get(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);
int audcodec_value_set(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value);
int audcodec_enum_get(struct audio_control *control,
                                   struct gb_audio_ctl_elem_value *value);
int audcodec_enum_set(struct audio_control *control,
                                   struct gb_audio_ctl_elem_value *value);

#define NOPWRCTL        (0xFFFFFFFF)    /* No power control for widget */

/* Audio widget power event */
#define WIDGET_EVENT_PRE_PWRUP          0x1 /* before widget power up */
#define WIDGET_EVENT_POST_PWRUP         0x2 /* after widget power up */
#define WIDGET_EVENT_PRE_PWRDOWN        0x4 /* before widget power down */
#define WIDGET_EVENT_POST_PWRDOWN       0x8 /* after widget power down */

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

#define AUDCTL_DUMMY(xname, xid, xiface, xreg, xshift, xinv) \
{ \
    .control = { \
        .name = xname, .id = xid, .count = 1, .count_values = 1, \
        .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
        .info = { \
            .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
            .dimen = {0,0,0,0}, \
            .value = { \
                .integer = { \
                    .min = 0, .max = 1, .step = 1 \
                } \
            } \
        } \
    }, \
    .dummy_reg = 0, \
    .get = audcodec_dummy_get, .set = audcodec_dummy_set, \
    .priv = BITCTL(xreg, xshift, xinv), \
}

#define AUDCTL_BIT(xname, xid, xiface, xreg, xshift, xinv) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 1, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = 0, .max = 1, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = audcodec_bit_get, .set = audcodec_bit_set, \
        .priv = BITCTL(xreg, xshift, xinv), \
    }

#define AUDCTL_BITS(xname, xid, xiface, xreg, xshift, xshift2, xinv) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 2, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = 0, .max = 1, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = audcodec_bits_get, .set = audcodec_bits_set, \
        .priv = BITSCTL(xreg, xshift, xshift2, xinv) \
    }

#define AUDCTL_BITSR(xname, xid, xiface, xreg, xshift, xreg2, xshift2, xinv) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 2, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = 0, .max = 1, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = (control_get*)audcodec_bits_get, .set = (control_set *)audcodec_bits_set, \
        .priv = BITSRCTL(xreg, xreg2, xshift, xshift2, xinv) \
    }

#define AUDCTL_BITSRV(xname, xid, xiface, xreg, xshift, xreg2, xshift2, xinv, \
                      xmask, xmax, xmin) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 2, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = xmin, .max = xmax, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = audcodec_value_get, .set = audcodec_value_set, \
        .priv = BITSRVCTL(xreg, xreg2, xshift, xshift2, xinv, xmask, xmax, \
                          xmin) \
    }

#define AUDCTL_BITSV(xname, xid, xiface, xreg, xshift, xshift2, xinv, \
                      xmask, xmax, xmin) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 2, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = xmin, .max = xmax, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = audcodec_value_get, .set = audcodec_value_set, \
        .priv = BITSRVCTL(xreg, xreg, xshift, xshift2, xinv, xmask, xmax, \
                          xmin) \
    }

#define AUDCTL_BITV(xname, xid, xiface, xreg, xshift, xinv, xmask, xmax, xmin) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 1, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = xmin, .max = xmax, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = audcodec_value_get, .set = audcodec_value_set, \
        .priv = BITSRVCTL(xreg, xreg, xshift, xshift, xinv, xmask, xmax, \
                          xmin) \
    }

#define AUDCTL_BITSVE(xname, xid, xiface, xreg, xshift, xshift2, xinv, \
                      xmask, xregmax, xctlmax, xmin, xget, xset) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 2, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
            .info = { \
                .type = GB_AUDIO_CTL_ELEM_TYPE_INTEGER, \
                .dimen = {0,0,0,0}, \
                .value = { \
                    .integer = { \
                        .min = xmin, .max = xctlmax, .step = 1 \
                    } \
                } \
            } \
        }, \
        .get = xget, .set = xset, \
        .priv = BITSRVCTL(xreg, xreg, xshift, xshift2, xinv, xmask, xregmax, \
                          xmin) \
    }

#define AUDCTL_ENUM(xname, xid, xiface, xreg, xshift, xmask, xtexts) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 1, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
        }, \
        .get = audcodec_enum_get, .set = audcodec_enum_set, \
        .priv = ENUMCTL(xreg, xreg, xshift, xshift, xmask, ARRAY_SIZE(xtexts), \
                        xtexts, NULL), \
    }

#define AUDCTL_ENUMS(xname, xid, xiface, xreg, xshift, xreg2, xshift2, xmask, \
                     xtexts) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 2, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
        }, \
        .get = audcodec_enum_get, .set = audcodec_enum_set, \
        .priv = ENUMCTL(xreg, xreg2, xshift, xshift2, xmask, \
                        ARRAY_SIZE(xtexts), xtexts, NULL), \
    }

#define AUDCTL_ENUMV(xname, xid, xiface, xreg, xshift, xmask, xtexts, xvalues) \
    { \
        .control = { \
            .name = xname, .id = xid, .count = 1, .count_values = 1, \
            .iface = GB_AUDIO_CTL_ELEM_IFACE_##xiface, \
        }, \
        .get = audcodec_enum_get, .set = audcodec_enum_set, \
        .priv = ENUMCTL(xreg, xreg, xshift, xshift, xmask, ARRAY_SIZE(xtexts), \
                        xtexts, xvalues), \
    }

/* Audio widget macro */
#define WIDGET(xname, xid, xtype, xcontrols, xnum, xreg, xshift, xinv) \
    { \
        .widget = { \
            .name = xname, .id = xid, .type = GB_AUDIO_WIDGET_TYPE_##xtype, \
            .state = GB_AUDIO_WIDGET_STATE_DISABLED, .ncontrols = xnum, \
        }, \
        .controls = xcontrols, .num_controls = xnum, \
        .reg = xreg, .shift = xshift, .inv = xinv, .event = NULL \
    }

#define WIDGET_E(xname, xid, xtype, xcontrols, xnum, xreg, xshift, xinv, xevent) \
    { \
        .widget = { \
            .name = xname, .id = xid, .type = GB_AUDIO_WIDGET_TYPE_##xtype, \
            .state = GB_AUDIO_WIDGET_STATE_DISABLED, .ncontrols = xnum, \
        }, \
        .controls = xcontrols, .num_controls = xnum, \
        .reg = xreg, .shift = xshift, .inv = xinv, .event = xevent \
    }

#define WIDGET_S(xname, xsname, xid, xtype, xcontrols, xnum, xreg, xshift, xinv) \
    { \
        .widget = { \
            .name = xname, .id = xid, .type = GB_AUDIO_WIDGET_TYPE_##xtype, \
            .state = GB_AUDIO_WIDGET_STATE_DISABLED, .same = xsname, \
            .ncontrols = xnum, \
        }, \
        .controls = xcontrols, .num_controls = xnum, \
        .reg = xreg, .shift = xshift, .inv = xinv, .event = NULL \
    }
#endif /* __AUDCODEC_H__ */
