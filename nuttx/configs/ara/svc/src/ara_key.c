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

/**
 * @author: Jeffrey Carlyle
 */

#define DBG_COMP ARADBG_SVC

#include <stdint.h>

#include <ara_debug.h>

#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/wqueue.h>
#include <nuttx/power/pm.h>

#include "ara_board.h"
#include "ara_key.h"
#include "gb_svc.h"
#include "svc_pm.h"

#define ARA_KEY_LONGPRESS_TIME_MS (5000)    /* 5s */
#define ARA_KEY_DEBOUNCE_TIME_MS (300)	    /* 300ms */

static struct ara_key_context {
    struct work_s longpress_work;
    bool rising_edge;
    int (*longpress_callback)(void *priv);
    uint8_t gpio;
} the_ara_key;

/* triggered after long key press timer expiration */
static void ara_key_longpress_worker(void *priv)
{
    struct ara_key_context *key = priv;

    dbg_info("ara key long press\n");
    if (key->longpress_callback)
        key->longpress_callback(NULL);
}

/* schedule/cancel longpress event as needed */
static void ara_key_longpress_update(struct ara_key_context *key, bool active)
{
    irqstate_t flags;

    flags = irqsave();
    if (active) {
	/* if not already scheduled, schedule the longpress event */
        if (work_available(&key->longpress_work))
            work_queue(HPWORK, &key->longpress_work,
		    ara_key_longpress_worker, key,
		    MSEC2TICK(ARA_KEY_LONGPRESS_TIME_MS));
    } else {
	/* if key is released, cancel any pending longpress events */
        if (!work_available(&key->longpress_work))
            work_cancel(HPWORK, &key->longpress_work);
    }

    irqrestore(flags);
}

/*
 * This serves as the GPIO IRQ handler. The longpress worker is updated
 * and a key event greybus message is sent.
 */
static int ara_key_irqhandler(int irq, void *context, void *priv)
{
    struct ara_key_context *key = &the_ara_key;
    bool value, active;
    uint8_t event;

    value = !!gpio_get_value(key->gpio);
    active = (value == key->rising_edge);

    ara_key_longpress_update(key, active);

    event = active ? GB_SVC_KEY_PRESSED : GB_SVC_KEY_RELEASED;

    gb_svc_key_event(GB_KEYCODE_ARA, event);

    pm_activity(SVC_ARA_KEY_ACTIVITY);

    return OK;
}

int ara_key_enable(const struct ara_board_info *info,
                   int (*longpress_callback)(void *priv), bool enable)
{
    if (enable) {
        if (!info->ara_key_configured) {
            dbg_error("%s: no ara key gpio defined\n", __func__);
            return -EINVAL;
        }

        the_ara_key.longpress_callback = longpress_callback;
        the_ara_key.gpio = info->ara_key_gpio;
        the_ara_key.rising_edge = info->ara_key_rising_edge;

        gpio_activate(info->ara_key_gpio);
        gpio_direction_in(info->ara_key_gpio);
        gpio_irq_mask(info->ara_key_gpio);
        gpio_irq_settriggering(info->ara_key_gpio, IRQ_TYPE_EDGE_BOTH);
        gpio_irq_attach(info->ara_key_gpio, ara_key_irqhandler, NULL);
        gpio_set_debounce(info->ara_key_gpio, ARA_KEY_DEBOUNCE_TIME_MS);
    } else {
        gpio_irq_mask(info->ara_key_gpio);
        gpio_deactivate(info->ara_key_gpio);
    }

    return OK;
}
