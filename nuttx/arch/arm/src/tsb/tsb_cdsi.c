/*
 * Copyright (c) 2015 Google Inc.
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

#include <stdlib.h>

#include <nuttx/arch.h>
#include <arch/tsb/cdsi.h>

#include "chip.h"
#include "up_arch.h"
#include "tsb_scm.h"

/* -----------------------------------------------------------------------------
 * Video Crypto Engine
 */

#define VCRYP_CONFIG                    0x0000
#define VCRYP_CONFIG_CH_BYPASS_TX       0
#define VCRYP_CONFIG_CH_BYPASS_RX       1
#define VCRYP_CONFIG_CH_DECRYPT         2
#define VCRYP_CONFIG_CH_ENCRYPT         3
#define VCRYP_CONFIG_CH_MASK            3
#define VCRYP_CONFIG_CH1_SHIFT          2
#define VCRYP_CONFIG_CH0_SHIFT          0

#define VCRYP_START                     0x0004
#define VCRYP_START_CH1                 (1 << 2)
#define VCRYP_START_CH0                 (1 << 0)

#define VCRYP_STOP                      0x0008
#define VCRYP_STOP_CH1_FORCE            (3 << 2)
#define VCRYP_STOP_CH1                  (1 << 2)
#define VCRYP_STOP_CH0_FORCE            (3 << 0)
#define VCRYP_STOP_CH0                  (1 << 0)

static unsigned int vcrypt_enable_count;

static uint32_t vcrypt_read(uint32_t offset)
{
    return getreg32(CRYP_BASE + offset);
}

static void vcrypt_write(uint32_t offset, uint32_t v)
{
    putreg32(v, CRYP_BASE + offset);
}

static void vcrypt_enable(struct cdsi_dev *dev)
{
    unsigned int cdsi = dev->base == CDSI0_BASE ? TSB_CDSI0 : TSB_CDSI1;
    unsigned int mode = dev->dir == TSB_CDSI_RX? VCRYP_CONFIG_CH_BYPASS_RX
                      : VCRYP_CONFIG_CH_BYPASS_TX;
    unsigned int shift = cdsi ? VCRYP_CONFIG_CH1_SHIFT : VCRYP_CONFIG_CH0_SHIFT;
    uint32_t val;

    if (vcrypt_enable_count++ == 0) {
        tsb_clk_enable(TSB_CLK_VCRYP);
        tsb_reset(TSB_RST_VCRYPSYS);
    }

    if (cdsi == TSB_CDSI0) {
        tsb_reset(TSB_RST_VCRYPCH0);
    } else {
        tsb_reset(TSB_RST_VCRYPCH1);
    }

    /* Configure the channel in bypass mode and start it. */
    val = vcrypt_read(VCRYP_CONFIG);
    val &= ~(VCRYP_CONFIG_CH_MASK << shift);
    val |= mode << shift;
    vcrypt_write(VCRYP_CONFIG, val);

    vcrypt_write(VCRYP_START, cdsi ? VCRYP_START_CH1 : VCRYP_START_CH0);
}

static void vcrypt_disable(struct cdsi_dev *dev)
{
    unsigned int cdsi = dev->base == CDSI0_BASE ? TSB_CDSI0 : TSB_CDSI1;

    vcrypt_write(VCRYP_START, cdsi ? VCRYP_STOP_CH1_FORCE : VCRYP_STOP_CH0_FORCE);

    if (--vcrypt_enable_count == 0) {
        tsb_clk_disable(TSB_CLK_VCRYP);
    }
}

/* -----------------------------------------------------------------------------
 * CDSI
 */

void cdsi_write(struct cdsi_dev *dev, uint32_t addr, uint32_t v)
{
    putreg32(v, dev->base + addr);
}

uint32_t cdsi_read(struct cdsi_dev *dev, uint32_t addr)
{
    return getreg32(dev->base + addr);
}

/**
 * @brief Open a CDSI device
 * @param cdsi The CDSI instance to initialize (0 or 1)
 * @param dir The CDSI instance operating direction (TSB_CDSI_RX or TSB_CDSI_TX)
 * @return a cdsi_dev pointer or NULL on any faillure.
 */
struct cdsi_dev *cdsi_open(int cdsi, enum cdsi_direction dir)
{
    struct cdsi_dev *dev;

    dev = malloc(sizeof(struct cdsi_dev));
    if (!dev)
        return NULL;

    dev->dir = dir;
    dev->base = cdsi == TSB_CDSI0 ? CDSI0_BASE : CDSI1_BASE;

    return dev;
}

/**
 * @brief Close a CDSI device
 * @param dev Pointer to CDSI device
 */
void cdsi_close(struct cdsi_dev *dev)
{
    free(dev);
}

/**
 * @brief Enable a CDSI device
 * @param dev Pointer to CDSI device
 *
 * This function isn't thread-safe, the callers need to ensure proper
 * serialization.
 */
void cdsi_enable(struct cdsi_dev *dev)
{
    int cdsi = dev->base == CDSI0_BASE ? TSB_CDSI0 : TSB_CDSI1;

    tsb_clk_enable(cdsi == TSB_CDSI0 ? TSB_CLK_CDSI0_REF : TSB_CLK_CDSI1_REF);

    if (dev->dir == TSB_CDSI_TX) {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_enable(TSB_CLK_CDSI0_TX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI0_TX_APB);
            tsb_reset(TSB_RST_CDSI0_TX);
            tsb_reset(TSB_RST_CDSI0_TX_AIO);
        } else {
            tsb_clk_enable(TSB_CLK_CDSI1_TX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI1_TX_APB);
            tsb_reset(TSB_RST_CDSI1_TX);
            tsb_reset(TSB_RST_CDSI1_TX_AIO);
        }
    } else {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_enable(TSB_CLK_CDSI0_RX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI0_RX_APB);
            tsb_reset(TSB_RST_CDSI0_RX);
            tsb_reset(TSB_RST_CDSI0_RX_AIO);
        } else {
            tsb_clk_enable(TSB_CLK_CDSI1_RX_SYS);
            tsb_clk_enable(TSB_CLK_CDSI1_RX_APB);
            tsb_reset(TSB_RST_CDSI1_RX);
            tsb_reset(TSB_RST_CDSI1_RX_AIO);
        }
    }

    if (tsb_get_rev_id() == tsb_rev_es3)
        vcrypt_enable(dev);
}

/**
 * @brief Disable a CDSI device
 * @param dev Pointer to CDSI device
 *
 * This function isn't thread-safe, the callers need to ensure proper
 * serialization.
 */
void cdsi_disable(struct cdsi_dev *dev)
{
    int cdsi = dev->base == CDSI0_BASE ? TSB_CDSI0 : TSB_CDSI1;

    if (tsb_get_rev_id() == tsb_rev_es3)
        vcrypt_disable(dev);

    tsb_clk_disable(cdsi == TSB_CDSI0 ? TSB_CLK_CDSI0_REF : TSB_CLK_CDSI1_REF);

    if (dev->dir == TSB_CDSI_TX) {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_disable(TSB_CLK_CDSI0_TX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI0_TX_APB);
        } else {
            tsb_clk_disable(TSB_CLK_CDSI1_TX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI1_TX_APB);
        }
    } else {
        if (cdsi == TSB_CDSI0) {
            tsb_clk_disable(TSB_CLK_CDSI0_RX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI0_RX_APB);
        } else {
            tsb_clk_disable(TSB_CLK_CDSI1_RX_SYS);
            tsb_clk_disable(TSB_CLK_CDSI1_RX_APB);
        }
    }
}
