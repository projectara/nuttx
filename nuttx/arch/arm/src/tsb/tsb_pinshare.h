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
 *
 * Authors: Joel Porquet <joel@porquet.org>
 */

/*
 * Every selectable pin or group of pins
 */
enum tsb_pin {
    /* GPIO (first in the enum to map the GPIO numbers) */
    PIN_GPIO0,  PIN_GPIO1,  PIN_GPIO2,  PIN_GPIO3,  PIN_GPIO4,  PIN_GPIO5,
    PIN_GPIO6,  PIN_GPIO7,  PIN_GPIO8,  PIN_GPIO9,  PIN_GPIO10, PIN_GPIO11,
    PIN_GPIO12, PIN_GPIO13, PIN_GPIO14, PIN_GPIO15, PIN_GPIO16, PIN_GPIO17,
    PIN_GPIO18, PIN_GPIO19, PIN_GPIO20, PIN_GPIO21, PIN_GPIO22, PIN_GPIO23,
    PIN_GPIO24, PIN_GPIO25, PIN_GPIO26, PIN_GPIO27, PIN_GPIO28, PIN_GPIO29,
    PIN_GPIO30, PIN_GPIO31,
    /* UART */
    PIN_UART,  PIN_UART_XTS,
    /* PWM */
    PIN_PWM0, PIN_PWM1,
    /* I2S0 */
    PIN_I2S0,
    /* I2S1 */
    PIN_I2S1,
    /* JTAG */
    PIN_JTAG,
    /* ETM */
    PIN_ETM,
    /* SPI Master */
    PIN_SPIM, PIN_SPIM_CSX_N,
    /* I2C */
    PIN_I2C,
    /* SD */
    PIN_SD, PIN_SD_POWER, PIN_SD_CD_N, PIN_SD_WP,
    /* end */
    MAX_TSB_PIN,
};

/*
 * Public API
 */

void tsb_pinshare_init(void);

int tsb_pin_request(enum tsb_pin pin);
int tsb_pin_release(enum tsb_pin pin);

