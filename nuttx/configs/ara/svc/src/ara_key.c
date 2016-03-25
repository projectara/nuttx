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
#include <nuttx/gpio/debounce.h>
#include <nuttx/wqueue.h>
#include <nuttx/power/pm.h>

#include "ara_board.h"
#include "ara_key.h"
#include "gb_svc.h"

#define ARA_KEY_LONGPRESS_TIME_MS (5000)    /* 5s */
#define ARA_KEY_DEBOUNCE_TIME_MS (300)	    /* 300ms */

static struct ara_key_context {
    struct work_s longpress_work;
    struct work_s irq_work;
    bool rising_edge;
    int (*longpress_callback)(void *priv);
    struct debounce_data db;
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


/* ara key irq handler bottom-half -- interrupts on */
static void ara_key_irqworker(void *priv)
{
    struct ara_key_context *key = &the_ara_key;
    bool active = (bool)(uintptr_t)priv;
    uint8_t event;

    ara_key_longpress_update(key, active);

    /* send event to ap */
    event = active ? GB_SVC_KEY_PRESSED : GB_SVC_KEY_RELEASED;

    gb_svc_key_event(GB_KEYCODE_ARA, event);
}

/*
 * This serves as both the GPIO IRQ handler and the debounce handler. When
 * called as the debounce handler, IRQs are probably not already disabled when
 * entering this function.
 */
static int ara_key_irqhandler(int irq, void *context)
{
    struct ara_key_context *key = &the_ara_key;
    bool value, active;
    irqstate_t flags;

    flags = irqsave();

    value = !!gpio_get_value(key->db.gpio);
    active = (value == key->rising_edge);

    dbg_insane("ara key press value: %u active: %u\n", value, active);

    if (!debounce_gpio(&key->db, active)) {
	goto out;
    }

    dbg_insane("ara key press value: %u active: %u (stable)\n", value, active);

    /* if something is pending, cancel it so we can pass the correct active */
    if (!work_available(&key->irq_work)) {
	work_cancel(HPWORK, &key->irq_work);
    }
    work_queue(HPWORK, &key->irq_work, ara_key_irqworker,
               (void*)(uintptr_t)active, 0);

out:
    irqrestore(flags);

    pm_activity(9);

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
        the_ara_key.db.gpio = info->ara_key_gpio;
	the_ara_key.db.ms = ARA_KEY_DEBOUNCE_TIME_MS;
        the_ara_key.db.isr = ara_key_irqhandler;
        the_ara_key.db.db_state = DB_ST_INVALID;
        the_ara_key.rising_edge = info->ara_key_rising_edge;

        gpio_activate(info->ara_key_gpio);
        gpio_direction_in(info->ara_key_gpio);
        gpio_irq_mask(info->ara_key_gpio);
        gpio_irq_settriggering(info->ara_key_gpio, IRQ_TYPE_EDGE_BOTH);
        gpio_irq_attach(info->ara_key_gpio, ara_key_irqhandler);
    } else {
        gpio_irq_mask(info->ara_key_gpio);
        gpio_deactivate(info->ara_key_gpio);
    }

    return OK;
}
