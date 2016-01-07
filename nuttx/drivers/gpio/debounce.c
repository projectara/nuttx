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

#include <nuttx/clock.h>
#include <nuttx/gpio/debounce.h>

/* work queue handler for debounce timeout */
static void debounce_delayed_worker(void *data)
{
    struct debounce_data *db = data;

    db->isr(db->gpio, NULL);
}

/* queue debounce timeout handler if not already scheduled */
static int debounce_delay_check(struct debounce_data *db)
{
    /*
     * If the work is already scheduled, do not schedule another one now.
     * A new one will be scheduled if more debounce is needed.
     */
    if (!work_available(&db->work)) {
        return 0;
    }

    /* Schedule the work to run after the debounce timeout */
    return work_queue(HPWORK, &db->work, debounce_delayed_worker, db,
                      MSEC2TICK(db->ms));
}

/*
 * returns true if gpio is stable
 */
bool debounce_gpio(struct debounce_data *db, bool active)
{
    struct timeval now, diff, timeout_tv = { 0, db->ms * 1000 };
    bool stable = false;

    /*
     * Short pulses are filtered out.
     */
    switch (db->db_state) {
    case DB_ST_INVALID:
    default:
        gettimeofday(&db->debounce_tv, NULL);
        db->db_state = active ?
                       DB_ST_ACTIVE_DEBOUNCE : DB_ST_INACTIVE_DEBOUNCE;
        debounce_delay_check(db);
        break;
    case DB_ST_ACTIVE_DEBOUNCE:
        if (active) {
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &db->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                db->db_state = DB_ST_ACTIVE_STABLE;
		stable = true;
            } else {
                /* Check for a stable signal after the debounce timeout */
                debounce_delay_check(db);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&db->debounce_tv, NULL);
            db->db_state = DB_ST_INACTIVE_DEBOUNCE;
            debounce_delay_check(db);
        }
        break;
    case DB_ST_INACTIVE_DEBOUNCE:
        if (!active) {
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &db->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                db->db_state = DB_ST_INACTIVE_STABLE;
		stable = true;
            } else {
                /* Check for a stable signal after the debounce timeout */
                debounce_delay_check(db);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&db->debounce_tv, NULL);
            db->db_state = DB_ST_ACTIVE_DEBOUNCE;
            debounce_delay_check(db);
        }
        break;
    case DB_ST_ACTIVE_STABLE:
        if (!active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&db->debounce_tv, NULL);
            db->db_state = DB_ST_INACTIVE_DEBOUNCE;
            debounce_delay_check(db);
        }
        break;
    case DB_ST_INACTIVE_STABLE:
        if (active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&db->debounce_tv, NULL);
            db->db_state = DB_ST_ACTIVE_DEBOUNCE;
            debounce_delay_check(db);
        }
        break;
    }

    return stable;
}

