/*
 * Copyright (c) 2014-2015 Google Inc.
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
 *
 * Authors: Fabien Parent <fparent@baylibre.com>
 *          Benoit Cousson <bcousson@baylibre.com>
 */

#ifndef _TSB_GPIO_H_
#define _TSB_GPIO_H_

#include <stdint.h>
#include <stdlib.h>
#include <nuttx/util.h>

#define TSB_GPIO_CHIP_BASE          0

#define TSB_IRQ_TYPE_LEVEL_LOW      0x0
#define TSB_IRQ_TYPE_LEVEL_HIGH     0x1
#define TSB_IRQ_TYPE_EDGE_FALLING   0x2
#define TSB_IRQ_TYPE_EDGE_RISING    0x3
#define TSB_IRQ_TYPE_EDGE_BOTH      0x7

#define TSB_IRQ_TRIGGER_MASK        0x7

int tsb_gpio_register(void *driver_data);

static inline size_t tsb_nr_gpio(void)
{
    extern uint8_t tsb_gpio_line_count(void *driver_data);
    return tsb_gpio_line_count(NULL);
}

uint32_t tsb_gpio_get_raw_interrupt(void);
uint32_t tsb_gpio_get_interrupt(void);

#endif /* _TSB_GPIO_H_ */

