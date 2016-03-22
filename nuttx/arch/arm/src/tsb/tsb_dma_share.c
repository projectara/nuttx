/**
 * Copyright (c) 2015-2016 Google Inc.
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
 * @author Ann Chen
 * @brief  TSB UNIPRO and I2S Share the DMA dev
 */

#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_pll.h>
#include <nuttx/device_i2s.h>
#include <nuttx/device_dma.h>
#include <nuttx/ring_buf.h>
#include "tsb_dma_share.h"

static struct device *dev = NULL;
static int dma_share_driver_cnt;
static sem_t lock;

int tsb_dma_share_init(void)
{
    return sem_init(&lock, 0, 1);
}

struct device *tsb_dma_share_open(void)
{
    sem_wait(&lock);
    dma_share_driver_cnt++;
    if (dma_share_driver_cnt == 1) {
        dev = device_open(DEVICE_TYPE_DMA_HW, 0);
        if (!dev) {
            lldbg("dma_open fail\n");
            dma_share_driver_cnt--;
        }
    }
    sem_post(&lock);
    return dev;
}

void tsb_dma_share_close(void)
{
    sem_wait(&lock);
    dma_share_driver_cnt--;

    if (!dma_share_driver_cnt) {
        device_close(dev);
    }
    sem_post(&lock);
}

void tsb_dma_share_destroy(void)
{
    sem_destroy(&lock);
}
