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

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <semaphore.h>
#include <nuttx/util.h>

#include "chip.h"
#include "tsb_scm.h"
#include "tsb_pinshare.h"
#include "up_arch.h"

/*
 * Pinsharing configuration for each pin or group of pins
 */
struct pinsharing_conf {
    uint32_t mask;
    uint32_t value;
};

/* The default pinsharing configuration is based on GPBridge/ES3 because it's a
 * superset of every other configurations. It is dynamically patched at boot. */
static struct pinsharing_conf tsb_pinsharing_conf[MAX_TSB_PIN] = {
    /* GPIO */
    [PIN_GPIO0]         = {BIT(1), 0},
    [PIN_GPIO1]         = {BIT(0), 0},
    [PIN_GPIO2]         = {BIT(0), 0},
    [PIN_GPIO3]         = {BIT(2), 0},
    [PIN_GPIO4]         = {BIT(2), 0},
    [PIN_GPIO5]         = {BIT(2), 0},
    [PIN_GPIO6]         = {BIT(2) | BIT(20), 0},
    [PIN_GPIO7]         = {BIT(2), 0},
    [PIN_GPIO8]         = {BIT(2), 0},
    [PIN_GPIO9]         = {BIT(3), BIT(3)},
    [PIN_GPIO10]        = {BIT(5), BIT(5)},
    [PIN_GPIO11]        = {BIT(5), BIT(5)},
    [PIN_GPIO12]        = {BIT(5), BIT(5)},
    [PIN_GPIO13]        = {BIT(6), BIT(6)},
    [PIN_GPIO14]        = {BIT(6), BIT(6)},
    [PIN_GPIO15]        = {BIT(7), BIT(7)},
    [PIN_GPIO16]        = {BIT(4) | BIT(10), BIT(10)},
    [PIN_GPIO17]        = {BIT(4) | BIT(10), BIT(10)},
    [PIN_GPIO18]        = {BIT(4) | BIT(11), BIT(11)},
    [PIN_GPIO19]        = {BIT(4) | BIT(12), BIT(12)},
    [PIN_GPIO20]        = {BIT(4) | BIT(13), BIT(13)},
    [PIN_GPIO21]        = {BIT(8) | BIT(14), BIT(14)},
    [PIN_GPIO22]        = {BIT(8) | BIT(15), BIT(15)},
    [PIN_GPIO23]        = {0, 0},
    [PIN_GPIO24]        = {0, 0},
    [PIN_GPIO25]        = {0, 0},
    [PIN_GPIO26]        = {0, 0},
    [PIN_GPIO27]        = {BIT(16), BIT(16)},
    [PIN_GPIO28]        = {BIT(16), BIT(16)},
    [PIN_GPIO29]        = {BIT(17), BIT(17)},
    [PIN_GPIO30]        = {0, 0},
    [PIN_GPIO31]        = {BIT(19), BIT(19)},
    /* UART */
    [PIN_UART]          = {BIT(0), BIT(0)},
    [PIN_UART_XTS]      = {BIT(1) | BIT(3), BIT(1)},
    /* PWM */
    [PIN_PWM0]          = {BIT(2), 0},
    [PIN_PWM1]          = {BIT(1) | BIT(3), 0},
    /* I2S0 */
    [PIN_I2S0]          = {BIT(4) | BIT(10) | BIT(11) | BIT(12) | BIT(13), 0},
    /* I2S1 */
    [PIN_I2S1]          = {BIT(16) | BIT(17) | BIT(19), 0},
    /* JTAG */
    [PIN_JTAG]          = {BIT(5), 0},
    [PIN_ETM]           = {BIT(4), BIT(4)},
    /* SPI Master */
    [PIN_SPIM]          = {BIT(6), 0},
    [PIN_SPIM_CSX_N]    = {BIT(2) | BIT(7) | BIT(20), BIT(20)},
    /* I2C */
    [PIN_I2C]           = {BIT(8) | BIT(14) | BIT(15), 0},
    /* SD */
    [PIN_SD]            = {BIT(2), BIT(2)},
    [PIN_SD_POWER]      = {BIT(6), BIT(6)},
    [PIN_SD_CD_N]       = {BIT(8), BIT(8)},
    [PIN_SD_WP]         = {BIT(8), BIT(8)},
};

/*
 * Status for each pinshare bit
 */
static uint32_t pinshare_reserved;
static uint32_t pinshare_value;
static uint8_t pinshare_refcount[32];

static sem_t tsb_pinsharing_lock;

/*
 * Internal API
 */

#define TSB_SCM_PINSHARE 0x00000800

static uint32_t tsb_pinshare_set(uint32_t mask, uint32_t value)
{
    uint32_t r;

    r = (mask & value) | (~mask & pinshare_value);
    putreg32(r, SYSCTL_BASE + TSB_SCM_PINSHARE);
    return r;
}

/*
 * Public API
 */
int tsb_pin_request(enum tsb_pin pin)
{
    struct pinsharing_conf *conf;
    int i;
    uint32_t mask;
    int ret;

    conf = &tsb_pinsharing_conf[pin];

    /* the pin does not exist in the target hardware:
     * we make use of mask[31] which is OK since pinshare[31] is reserved */
    if ((signed)conf->mask < 0) {
        return -EINVAL;
    }

    /* the pin is not shared */
    if (conf->mask == 0) {
        return 0;
    }

    ret = sem_wait(&tsb_pinsharing_lock);
    if (ret) {
        return -get_errno();

    }

    /* check if the reserved bits have the value we need */
    if ((conf->mask & pinshare_reserved & conf->value) !=
            (conf->mask & pinshare_reserved & pinshare_value)) {
        sem_post(&tsb_pinsharing_lock);
        return -EBUSY;
    }

    /* reserved the pinshare bits */
    pinshare_reserved |= conf->mask;

    /* set up the correct pinshare value */
    pinshare_value = tsb_pinshare_set(conf->mask, conf->value);

    mask = conf->mask;
    while ((i = __builtin_ffs(mask)) != 0) {
        i--; /* because __builtin_ffs() returns index + 1 */

        /* adjust refcount */
        pinshare_refcount[i]++;

        /* remove the bit from the mask */
        mask &= ~BIT(i);
    }

    sem_post(&tsb_pinsharing_lock);
    return 0;
}

int tsb_pin_release(enum tsb_pin pin)
{
    struct pinsharing_conf *conf;
    int i;
    uint32_t mask;
    int ret;

    conf = &tsb_pinsharing_conf[pin];

    /* the pin does not exist in the target hardware */
    if ((signed)conf->mask < 0) {
        return -EINVAL;
    }

    /* the pin is not shared */
    if (conf->mask == 0) {
        return 0;
    }

    ret = sem_wait(&tsb_pinsharing_lock);
    if (ret) {
        return -get_errno();
    }

    mask = conf->mask;
    while ((i = __builtin_ffs(mask)) != 0) {
        i--; /* because __builtin_ffs() returns index + 1 */

        /* adjust refcount */
        if (pinshare_refcount[i] == 0) {
            /* error, the bit should be used at least by this pin! */
            sem_post(&tsb_pinsharing_lock);
            return -EINVAL;
        }
        pinshare_refcount[i]--;

        /* adjust reserved and reset pinshare */
        if (pinshare_refcount[i] == 0) {
            pinshare_reserved &= ~BIT(i);
            tsb_pinshare_set(BIT(i), 0);
        }
        /* remove the bit from the mask */
        mask &= ~BIT(i);
    }

    sem_post(&tsb_pinsharing_lock);
    return 0;
}

/*
 * Initialization
 */
void tsb_pinshare_init()
{
    /* patch the default pinsharing configuration according to the hardware */

    /* ES2 common */
    if (tsb_get_rev_id() == tsb_rev_es2) {
        /* GPIO6 is not shared with SPIM_CS1_N */
        tsb_pinsharing_conf[PIN_GPIO6].mask &= ~BIT(20);

        /* SPIM_CS1_N does not exist */
        tsb_pinsharing_conf[PIN_SPIM_CSX_N].mask &= ~(BIT(2) | BIT(20));
        tsb_pinsharing_conf[PIN_SPIM_CSX_N].value &= ~BIT(20);
    }

    /* ES2 or APBridge */
    if (tsb_get_rev_id() == tsb_rev_es2 ||
            tsb_get_product_id() == tsb_pid_apbridge) {
        /* GPIO24-31 do not exist */
        tsb_pinsharing_conf[PIN_GPIO24].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO25].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO26].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO27].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO28].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO29].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO30].mask = -1;
        tsb_pinsharing_conf[PIN_GPIO31].mask = -1;

        /* I2S1 pins do not exist */
        tsb_pinsharing_conf[PIN_I2S1].mask  = -1;
    }

    /* APBridge */
    if (tsb_get_product_id() == tsb_pid_apbridge) {
        /* GPIO3-8 are not shared with SD */
        tsb_pinsharing_conf[PIN_GPIO3].mask &= ~BIT(2);
        tsb_pinsharing_conf[PIN_GPIO4].mask &= ~BIT(2);
        tsb_pinsharing_conf[PIN_GPIO5].mask &= ~BIT(2);
        tsb_pinsharing_conf[PIN_GPIO6].mask &= ~BIT(2);
        tsb_pinsharing_conf[PIN_GPIO7].mask &= ~BIT(2);
        tsb_pinsharing_conf[PIN_GPIO8].mask &= ~BIT(2);

        /* GPIO21-22 are not shared with SD */
        tsb_pinsharing_conf[PIN_GPIO21].mask &= ~BIT(8);
        tsb_pinsharing_conf[PIN_GPIO22].mask &= ~BIT(8);

        /* PWM0 is not shared with SD */
        tsb_pinsharing_conf[PIN_PWM0].mask &= ~BIT(2);

        /* I2C is not shared with SD */
        tsb_pinsharing_conf[PIN_I2C].mask &= ~BIT(8);

        /* SD pins do not exist */
        tsb_pinsharing_conf[PIN_SD].mask = -1;
        tsb_pinsharing_conf[PIN_SD_POWER].mask = -1;
        tsb_pinsharing_conf[PIN_SD_CD_N].mask = -1;
        tsb_pinsharing_conf[PIN_SD_WP].mask = -1;

        /* Only for ES3 */
        if (tsb_get_rev_id() == tsb_rev_es3) {
            /* SPIM_CS1_N is not shared with SD */
            tsb_pinsharing_conf[PIN_SPIM_CSX_N].mask &= ~BIT(2);
        }
    }

    sem_init(&tsb_pinsharing_lock, 0, 1);
}

