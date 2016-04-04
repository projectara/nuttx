/*
 * Copyright (c) 2014-2016 Google Inc.
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

#ifndef _GPIO_H_
#define _GPIO_H_

/**
 * @file nuttx/gpio.h
 * @brief GPIO API
 */

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/irq.h>

/**
 * @defgroup IRQ_TYPE IRQ trigger type
 *
 * Defines which event triggers an interrupt on a certain GPIO line. For
 * example, an interrupt can be edge-triggered or level-triggered.
 * @{
 */
/** Do not trigger an interrupt */
#define IRQ_TYPE_NONE           0x00000000
/** Triggers an interrupt on a rising edge */
#define IRQ_TYPE_EDGE_RISING    0x00000001
/** Triggers an interrupt on a falling edge */
#define IRQ_TYPE_EDGE_FALLING   0x00000002
/** Triggers an interrupt on a both types of edges */
#define IRQ_TYPE_EDGE_BOTH      (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
/** Triggers an interrupt when the signal is high */
#define IRQ_TYPE_LEVEL_HIGH     0x00000004
/** Triggers an interrupt when the signal is low */
#define IRQ_TYPE_LEVEL_LOW      0x00000008
/** @} */

/** GPIO internal pull resistor types */
enum gpio_pull_type {
    /** Internal pull down resistor */
    GPIO_PULL_TYPE_PULL_DOWN,
    /** Internal pull up resistor */
    GPIO_PULL_TYPE_PULL_UP,
    /** No internal pull resistor */
    GPIO_PULL_TYPE_PULL_NONE,
};

/**
 * @brief Set the interrupt trigger type corresponding to a GPIO line
 * @param which The number of the GPIO line
 * @param trigger \ref IRQ_TYPE
 * @return 0 on success, !=0 on failure
 */
int gpio_irq_settriggering(uint8_t which, int trigger);

/**
 * @brief Get the direction of a GPIO line
 * @param which The number of the GPIO line
 * @return 0 if the GPIO line is set as an input, 1 if the GPIO line is set as
 * an output
 */
int gpio_get_direction(uint8_t which);

/**
 * @brief Set a GPIO line as an input
 * @param which The number of the GPIO line
 */
void gpio_direction_in(uint8_t which);

/**
 * @brief Set a GPIO line as an output and set its initial value
 * @param which The number of the GPIO line
 * @param value 0 for setting the initial value of the GPIO line to logical 0,
 * !=0 for setting the initial value of GPIO line to logical 1
 */
void gpio_direction_out(uint8_t which, uint8_t value);

/**
 * @brief Get the value of a GPIO line
 * @param which The number of the GPIO line
 * @return 0 if the value of the GPIO line is logical 0, 1 if the value of the
 * GPIO line is logical 1
 */
uint8_t gpio_get_value(uint8_t which);

/**
 * @brief Set the value of a GPIO line
 * @param which The number of the GPIO line
 * @param value 0 for setting the value of the GPIO line to logical 0, !=0 for
 * setting the value of GPIO line to logical 1
 */
void gpio_set_value(uint8_t which, uint8_t value);

/**
 * @brief Set the debouncing delay of a GPIO line
 * @param which The number of the GPIO line
 * @param delay Debouncing delay in ms, delay of zero disables debouncing
 * @return 0 on success, !=0 on failure
 */
int gpio_set_debounce(uint8_t which, uint16_t delay);

/**
 * @brief Set the internal pull resistor for a GPIO input line
 * @param which The number of the GPIO line
 * @param pull_type The type of internal pull resistor
 * @return 0 on success, !=0 on failure
 */
int gpio_set_pull(uint8_t which, enum gpio_pull_type pull_type);

/**
 * @brief Get the current internal pull configuration for a GPIO input line
 * @param which The number of the GPIO line
 * @return The type of the internal pull resistor
 */
enum gpio_pull_type gpio_get_pull(uint8_t which);

/**
 * @brief Get the number of GPIO lines
 * @return The number of GPIO lines
 */
uint8_t gpio_line_count(void);

/**
 * @brief Attach an interrupt handler to a GPIO line
 * @param which The number of the GPIO line
 * @param isr The interrupt handler
 * (int (*xcpt_t)(int irq, void *context, void *priv))
 * @return 0 on success, !=0 on failure
 */
int gpio_irq_attach(uint8_t which, xcpt_t isr);

/**
 * @brief Mask the interrupt corresponding to a GPIO line
 * @param which The number of the GPIO line
 * @return 0 on success, !=0 on failure
 */
int gpio_irq_mask(uint8_t which);

/**
 * @brief Unmask the interrupt corresponding to a GPIO line
 * @param which The number of the GPIO line
 * @return 0 on success, !=0 on failure
 */
int gpio_irq_unmask(uint8_t which);

/**
 * @brief Clear the interrupt corresponding to a GPIO line
 * @param which The number of the GPIO line
 * @return 0 on success, !=0 on failure
 */
int gpio_irq_clear(uint8_t which);

/**
 * @brief Request a GPIO line
 * @param which The number of the GPIO line
 * @return 0 on success, !=0 on failure
 */
int gpio_activate(uint8_t which);

/**
 * @brief Free a GPIO line
 * @param which The number of the GPIO line
 * @return 0 on success, !=0 on failure
 */
int gpio_deactivate(uint8_t which);

/**
 * @brief Check whether a GPIO line is valid
 *
 * This function can be used to check if a GPIO number is valid;
 * i.e. if a chip has been registered for this line.
 *
 * It is an error to use the GPIO chip API on invalid lines.
 *
 * @param which GPIO to check
 * @return true if the GPIO is valid, false otherwise
 */
bool gpio_is_valid(uint8_t which);

#endif

