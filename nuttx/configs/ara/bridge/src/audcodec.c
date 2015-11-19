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

uint32_t audcodec_read(uint32_t reg, uint32_t *value)
{
    struct device *dev = get_codec_dev();
    codec_read_func read_func = NULL;
    if (!dev) {
        return -EINVAL;
    }
    read_func = (codec_read_func)get_codec_read_func(dev);
    if (!read_func) {
        return -EINVAL;
    }
    return read_func(reg, value);
}

uint32_t audcodec_write(uint32_t reg, uint32_t value)
{
    struct device *dev = get_codec_dev();
    codec_write_func write_func = NULL;
    if (!dev) {
        return -EINVAL;
    }
    write_func = (codec_write_func)get_codec_write_func(dev);
    if (!write_func) {
        return -EINVAL;
    }
    return write_func(reg, value);
}

uint32_t audcodec_update(uint32_t reg, uint32_t value, uint32_t mask)
{
    uint32_t data = 0;

    if (audcodec_read(reg, &data)) {
        return -EIO;
    }
    data = (data & ~mask) | value;
    if (audcodec_write(reg, data)) {
        return -EIO;
    }
    return 0;
}

int audcodec_bit_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg = 0, inv = 0, shift = 0, data = 0, mask = 0;
    int ret = 0;
    struct bitctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg = ctl->reg;
    shift = ctl->shift;
    inv = ctl->inv;
    mask = ctl->mask;

    ret = audcodec_read(reg, &data);
    if (ret) {
        return -EIO;
    }
    data = (data >> shift) & mask;
    if (inv) {
        data = (data)? 0: 1;
    }

    value[0].value.integer_value = data;
    return ret;
}

int audcodec_bit_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg = 0, inv = 0, shift = 0, data = 0, mask = 0;
    int ret = 0;
    struct bitctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg = ctl->reg;
    shift = ctl->shift;
    inv = ctl->inv;
    mask = ctl->mask;

    data = value[0].value.integer_value;

    if (inv) {
        data = (data)? 0: 1;
    }
    data = (data << shift);

    ret = audcodec_update(reg, data, mask << shift);
    if (ret) {
        return -EIO;
    }
    return ret;
}

int audcodec_bits_get(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0, mask = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;
    struct bitctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;


    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;
    mask = ctl->mask;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & mask;
    if (inv) {
        data1 = (data1)? 0: 1;
    }
    value[0].value.integer_value = data1;
    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* second control */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & mask;
        if (inv) {
            data2 = (data2)? 0: 1;
        }
        value[1].value.integer_value = data2;
    }
    return ret;
}

int audcodec_bits_set(struct audio_control *control,
                             struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0, mask = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;
    struct bitctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;
    mask = ctl->mask;

    data1 = value[0].value.integer_value;

    if (inv) {
        data1 = (data1)? 0: 1;
    }
    data1 = (data1 << shift1);

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* second control */
        data2 = value[1].value.integer_value;
        if (inv) {
            data2 = (data2)? 0: 1;
        }
        data2 = (data2 << shift2);
    }

    if (reg1 == reg2) {
        ret = audcodec_update(reg1, data1 | data2,
                             (mask << shift1) | (mask << shift2));
    } else {
        ret = audcodec_update(reg1, data1, mask << shift1);
        if (!ret) {
            ret = audcodec_update(reg2, data2, mask << shift2);
        }
    }
    return ret;
}

int audcodec_value_get(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg1 = 0, reg2 = 0, inv = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t max = 0, min = 0, mask = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;
    struct bitctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    inv = ctl->inv;
    max = ctl->max;
    min = ctl->min;
    mask = ctl->mask;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & mask;
    if (inv) {
        data1 = max - data1;
    }
    data1 = data1 - min;
    value[0].value.integer_value = data1;

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* two controls */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & mask;
        if (inv) {
            data2 = max - data2;
        }
        data2 = data2 - min;
        value[1].value.integer_value = data2;
    }
    return ret;
}

int audcodec_value_set(struct audio_control *control,
                              struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg1 = 0, reg2 = 0, mask = 0;
    uint32_t shift1 = 0, shift2 = 0, max = 0, min = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;
    struct bitctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    mask = ctl->mask;
    max = ctl->max;
    min = ctl->min;

    data1 = value[0].value.integer_value;
    data1 = (data1 > max)? max : data1;
    data1 = (data1 < min)? min : data1;

    data1 = (data1 << shift1);

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* second control */
        data2 = value[1].value.integer_value;
        data2 = (data2 > max)? max : data2;
        data2 = (data2 < min)? min : data2;
        data2 = (data2 << shift2);
    }

    if (reg1 == reg2) {
        ret = audcodec_update(reg1, data1 | data2,
                             (mask << shift1) | (mask << shift2));
    } else {
        ret = audcodec_update(reg1, data1, mask << shift1);
        if (!ret) {
            ret = audcodec_update(reg2, data2, mask << shift2);
        }
    }
    return ret;
}

int audcodec_enum_get(struct audio_control *control,
                                   struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg1 = 0, reg2 = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t mask = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;
    struct enumctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    mask = ctl->mask;

    ret = audcodec_read(reg1, &data1);
    if (ret) {
        return -EIO;
    }
    data1 = (data1 >> shift1) & mask;
    value[0].value.integer_value = data1;

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* second control */
        ret = audcodec_read(reg2, &data2);
        if (ret) {
            return -EIO;
        }
        data2 = (data2 >> shift2) & mask;
        value[1].value.integer_value = data2;
    }
    return ret;
}

int audcodec_enum_set(struct audio_control *control,
                                   struct gb_audio_ctl_elem_value *value)
{
    uint32_t reg1 = 0, reg2 = 0;
    uint32_t shift1 = 0, shift2 = 0;
    uint32_t mask = 0, max = 0;
    uint32_t data1 = 0, data2 = 0;
    int ret = 0;
    struct enumctl *ctl = NULL;

    if (!control || !control->priv || !value) {
        return -EINVAL;
    }
    ctl = control->priv;

    reg1 = ctl->reg;
    reg2 = ctl->reg2;
    shift1 = ctl->shift;
    shift2 = ctl->shift2;
    mask = ctl->mask;
    max = ctl->max;

    data1 = value[0].value.integer_value;
    if (data1 > max) {
        data1 = max;
    }
    data1 = data1 << shift1;

    if (((reg1 == reg2) && (shift1 != shift2)) || (reg1 != reg2)) {
        /* second control */
        data2 = value[1].value.integer_value;
        if (data2 > max) {
            data2 = max;
        }
        data2 = data2 << shift2;
    }

    if (reg1 == reg2) {
        ret = audcodec_update(reg1, data1 | data2,
                             (mask << shift1) | (mask << shift2));
    } else {
        ret = audcodec_update(reg1, data1, mask << shift1);
        if (!ret) {
            ret = audcodec_update(reg2, data2, mask << shift2);
        }
    }
    return ret;
}
