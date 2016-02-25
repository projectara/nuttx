/*
 * Copyright (c) 2016 Google Inc.
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

#ifndef _NUTTX_GPIO_FIXED_H_
#define _NUTTX_GPIO_FIXED_H_

#include <stdint.h>

/**
 * @file nuttx/gpio_fixed.h
 * @brief "Fixed" GPIO chip support
 *
 * This API can be used to create GPIO chips which simulate a single
 * GPIO with a fixed, or constant, value.
 *
 * These can be useful when a GPIO handle is needed by some API, but a
 * physical GPIO isn't available in hardware.
 */

/**
 * @brief Register a GPIO chip with a fixed value
 *
 * The GPIO chip will have exactly one GPIO line, which will report a
 * constant value when read. No actual GPIOs are used -- the resulting
 * GPIO chip is a software simulation only.
 *
 * Unsupported:
 *   - output (writes and direction-out requests are ignored,
 *     and errors may be logged)
 *   - pull-ups and pull-downs
 *   - level-triggered IRQs with the same value as the one the
 *     gpio is fixed to
 *
 * @param value Value the single GPIO in the GPIO chip should
 *              return when read.
 * @param base  Base address of GPIO chip
 * @return      Zero on success, negative on error
 * @see fixed_gpio_deinit()
 */
int fixed_gpio_init(uint8_t value, int base);

/**
 * @brief Deregister a fixed GPIO chip
 *
 * Remove a previously-registered fixed GPIO chip.
 *
 * @param base Base GPIO number of fixed GPIO chip to deregister
 * @see fixed_gpio_init()
 */
void fixed_gpio_deinit(int base);

#endif
