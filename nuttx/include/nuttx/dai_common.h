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

/* common audo data types between Codec and I2S drivers */

#ifndef __DAI_COMMON_H
#define __DAI_COMMON_H

#include "../../drivers/greybus/audio-gb.h"

#define DEVICE_DAI_PROTOCOL_PCM                         BIT(0)
#define DEVICE_DAI_PROTOCOL_I2S                         BIT(1)
#define DEVICE_DAI_PROTOCOL_LR_STEREO                   BIT(2)

#define DEVICE_DAI_ROLE_MASTER                          BIT(0)
#define DEVICE_DAI_ROLE_SLAVE                           BIT(1)

#define DEVICE_DAI_POLARITY_NORMAL                      BIT(0)
#define DEVICE_DAI_POLARITY_REVERSED                    BIT(1)

#define DEVICE_DAI_EDGE_RISING                          BIT(0)
#define DEVICE_DAI_EDGE_FALLING                         BIT(1)

/* low level DAI communication capabilities of a driver
 * Most capabilities are bitfields,
 * To find matching capabilities between drivers AND the bitfields
 * returned from get_caps
 */
struct device_dai {
    uint32_t    mclk_freq;          /* mclk frequency generated/required */
    uint8_t     protocol;           /* DEVICE_DAI_PROTOCOL_* */
    uint8_t     wclk_polarity;      /* DEVICE_DAI_POLARITY_* */
    uint8_t     wclk_change_edge;   /* DEVICE_DAI_EDGE_* */
    uint8_t     data_rx_edge;       /* DEVICE_DAI_EDGE_* */
    uint8_t     data_tx_edge;       /* DEVICE_DAI_EDGE_* */
};

#endif /* __DAI_COMMON_H */
