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

#define DBG_COMP ARADBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/clock.h>
#include <nuttx/greybus/timesync.h>
#include <arch/board/board.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <ara_debug.h>

#include "tsb_scm.h"
#include "tsb_tmr.h"

#define TIMESYNC_TASK_PRIORITY          (40)
#define TIMESYNC_TASK_STACK_SIZE        (1024)
#define TSB_TMR_FREQUENCY               48000000
#define TIMESYNC_MAX_STROBE_DELAY_US    10000

/*
 * Define CONFIG_ARCH_TIMESYNC_DEBUG to get printouts locally and a 1 second strobe
 * after completion of the initial synchronization operation
 */

/* The current local frame-time */
static long double timesync_frame_time;
static uint64_t timesync_frame_time_offset;
static uint64_t timesync_strobe_time[GB_TIMESYNC_MAX_STROBES];
static double timesync_div;
static double timesync_increment;
static double timesync_ns_per_clock;
static bool timesync_offset_down;

/* Last ping frame time */
static uint64_t timesync_last_event_time;

/* Internal state, timers, indices etc */
static int timesync_state = TIMESYNC_STATE_INVALID;
static uint32_t timesync_strobe_delay_ns;
static uint32_t timesync_refclk;
static int timesync_strobe_count;
static int timesync_strobe_index;
static struct tsb_tmr_ctx *timesync_rollover_timer;
static uint64_t timesync_rx_strobes;

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
static uint32_t timesync_counter_time[GB_TIMESYNC_MAX_STROBES];
static sem_t dbg_thread_sem;

#define timesync_notify_dbg() sem_post(&dbg_thread_sem)
#else
#define timesync_notify_dbg()
#endif

/* TIME_SYNC strobe timer */
#define TIMESYNC_STROBE_TIMER_PERIOD        1000000

/* Cascaded rollover timer */
#define TIMESYNC_ROLLOVER_TOTAL             ULONG_MAX

/**
 * @brief Reads raw counter value from TSB counter block
 *
 * Read in the TSB raw counter value. Convert from irritating
 * down counter to up counter. Frame-time is a monotonic clock so
 * we need our local timer to count upwards.
 */
static uint32_t timesync_get_counter(void) {
    return (ULONG_MAX - tsb_tmr_get_counter(timesync_rollover_timer));
}

/**
 * @brief Return a 64 bit frame time expressed in timer clocks.
 *
 * All timers must tick at the rate the AP has mandated 19.2MHz in Ara V1
 * unfortunately this means we end up having to do some floating point maths
 * to divide down the 48MHz local (fixed) clock to the AP supplied clock.
 *
 * Latch lower 32 bits (hardware) and upper 32 bits soft-IRQ tracked
 * to derive a 64 frame-time - expressed in AP refclk clocks.
 *
 */
uint64_t timesync_get_frame_time(void) {
    uint64_t hi, tmp;
    uint32_t lod;

    while (1) {
        hi = timesync_frame_time;
        lod = timesync_get_counter();
        tmp = timesync_frame_time;
        if (tmp == hi)
            break;
    }

    lod = ((double)lod) / timesync_div;

    if (timesync_offset_down)
        hi -= timesync_frame_time_offset;
    else
        hi += timesync_frame_time_offset;

    return (hi + lod);
}

/**
 * @brief return the current state of TIME_SYNC
 *
 * Allows a caller to understand if the frame-time returned by
 * timesync_get_frame_time() is currently ACTIVE and hence trusted
 * or not ACTIVE and hence not to be relied upon.
 */
int timesync_get_state(void) {
    return timesync_state;
}

/**
 * @brief Responsible for logging the local frame-time on strobe with the
*   counter if CONFIG_ARCH_TIMESYNC_DEBUG is defined.
*/
static void timesync_log_frame_time(void) {
    if (timesync_strobe_index < GB_TIMESYNC_MAX_STROBES) {
       if (!timesync_strobe_index)
            tsb_tmr_start_ext(timesync_rollover_timer, TIMESYNC_ROLLOVER_TOTAL, true);
        timesync_strobe_time[timesync_strobe_index] = timesync_get_frame_time();
#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
        timesync_counter_time[timesync_strobe_index] = timesync_get_counter();
#endif
        timesync_strobe_index += 1;
    }
}

/**
 * @brief Log the incoming strobe time
 */
int timesync_strobe_handler(void) {
    irqstate_t flags;

    flags = irqsave();
    if (timesync_state == TIMESYNC_STATE_SYNCING) {
        timesync_log_frame_time();
        timesync_rx_strobes++;
    } else if (timesync_state == TIMESYNC_STATE_ACTIVE) {
        timesync_last_event_time = timesync_get_frame_time();
        timesync_notify_dbg();
        timesync_rx_strobes++;
    }
    irqrestore(flags);

    return 0;
}

/**
 * @brief Responsible for handling roll-over of the lower 32 bit integer into the 64 bit frame-time
 *
 */
static int timesync_frame_time_rollover_handler(int irq, void *context,
                                                void *priv) {
    tsb_tmr_ack_irq(timesync_rollover_timer);
    timesync_frame_time += timesync_increment;
    return 0;
}

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
/**
 * @brief A simple thread to print the frame-time on an on-going basis
 *
 */
static int timesync_debugd_main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    while (timesync_state != TIMESYNC_STATE_INVALID) {
        sem_wait(&dbg_thread_sem);

        if (timesync_state == TIMESYNC_STATE_INVALID)
            break;

        lldbg("frame-time=%llu last-event-time=%llu rx-strobes=%d\n",
              timesync_get_frame_time(), timesync_last_event_time,
              timesync_rx_strobes);
    }
    return 0;
}
#endif

/**
 * @brief Enable TimeSync given reference frame_time
 *
 * On the APB/GPB side we have a fixed rate clock which means we need to do some
 * floating point maths to divide down the local counter to the rate supplied by
 * the AP.
 * @param strobe_count - the number of strobes to expect
 * @param frame_time - the initial time to seed to on the first strobe
 * @param strobe_delay - the expected delay between each strobe in useconds
 * @param refclk - the clock the AP is using 19.2MHz on MSM8994
 */
int timesync_enable(uint8_t strobe_count, uint64_t frame_time,
                    uint32_t strobe_delay, uint32_t refclk) {

    if (!strobe_count || strobe_count > GB_TIMESYNC_MAX_STROBES)
        return -EINVAL;

    if (!strobe_delay || strobe_delay > TIMESYNC_MAX_STROBE_DELAY_US)
        return -EINVAL;

    if (!refclk)
        return -EINVAL;

    if (TSB_TMR_FREQUENCY % refclk)
        dbg_verbose("Time-Sync clock %dHz doesn't divide APB clock %dHz evenly\n",
                    refclk, TSB_TMR_FREQUENCY);

    timesync_disable();

    timesync_refclk = refclk;
    timesync_strobe_count = strobe_count;
    timesync_strobe_index = 0;
    timesync_rx_strobes = 0;
    timesync_strobe_delay_ns = strobe_delay * NSEC_PER_USEC;
    timesync_ns_per_clock = ((double)NSEC_PER_SEC) / ((double)refclk);
    timesync_state = TIMESYNC_STATE_SYNCING;
    timesync_frame_time = frame_time;
    timesync_div = ((double)TSB_TMR_FREQUENCY) / ((double)refclk);
    timesync_increment = ((double)TIMESYNC_ROLLOVER_TOTAL+1) / timesync_div;
    timesync_frame_time_offset = 0;

    /************************************************************
     * Config TMR3 as a free running timer at the fixed frequency
     * 48MHz TMRx run at on the Toshiba part.
     ************************************************************/
    tsb_tmr_configure(timesync_rollover_timer, TSB_TMR_MODE_FREERUN,
                      timesync_frame_time_rollover_handler);

    dbg_verbose("ref-clk-freq=%dHz timer-freq=%dHz period=0x%08x\n",
             TSB_TMR_FREQUENCY, timesync_refclk,
             TIMESYNC_ROLLOVER_TOTAL);

    return 0;
}

/**
 * @brief Disable TimeSync
 */
int timesync_disable(void) {
    irqstate_t flags;

    flags = irqsave();

    (void)tsb_tmr_cancel(timesync_rollover_timer);
    tsb_tmr_ack_irq(timesync_rollover_timer);
    timesync_state = TIMESYNC_STATE_INACTIVE;

    irqrestore(flags);

    return 0;
}

/**
 * @brief - get the difference between two doubles
 */
static double timesync_diff(double x, double y)
{
    if (x > y)
        return x - y;
    else
        return y - x;
}

/**
 * @brief Align the local frame time to the frame-time provided by SVC
 */
int timesync_authoritative(uint64_t *frame_time) {
    int i = 0;
    double delta, least = 0;
    uint32_t best_match;

    /* This means the AP has seen a pulse we have not on the Module end */
    if (timesync_strobe_index != timesync_strobe_count) {
        fprintf(stderr, "unexpected authoritative rx count %d expect %d\n",
                timesync_strobe_index, timesync_strobe_count);
        return -ETIMEDOUT;
    }

    for (i = 1; i < timesync_strobe_count; i++) {

        /* Derive the difference in clocks */
        delta = timesync_strobe_time[i] - timesync_strobe_time[i - 1];
        delta *= timesync_ns_per_clock;
        best_match = delta;
        delta = timesync_diff(delta, timesync_strobe_delay_ns);

        /* Sync to the first or best match to the target number of clocks */
        if (!least || delta < least) {
            least = delta;
            if (frame_time[i] > timesync_strobe_time[i]) {
                timesync_frame_time_offset =
                    frame_time[i] - timesync_strobe_time[i];
                timesync_offset_down = false;
            } else {
                timesync_frame_time_offset =
                    timesync_strobe_time[i] - frame_time[i];
                timesync_offset_down = true;
            }
            dbg_verbose("adjust clock %s local %llu svc %llu adjustment=%llu\n",
                        timesync_offset_down ? "downwards" : "upwards",
                        timesync_strobe_time[i], frame_time[i],
                        timesync_frame_time_offset);

            dbg_verbose("strobe %d and %d best match clock diff %lu target %lu\n",
                        i - 1, i, best_match, timesync_strobe_delay_ns);
        }
    }
#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
    for (i = 1; i < timesync_strobe_index; i++) {
        lldbg("frame-time diff %llu-frame-clocks %llu-%llu\n",
              timesync_strobe_time[i] - timesync_strobe_time[i - 1],
              timesync_strobe_time[i], timesync_strobe_time[i - 1]);
    }
    for (i = 0; i < timesync_strobe_index; i++) {
        lldbg("counter @ strobe %d counter = %lu time = %llu\n",
              i, timesync_counter_time[i], timesync_strobe_time[i]);
    }
#endif
    timesync_state = TIMESYNC_STATE_ACTIVE;
    timesync_notify_dbg();

    return 0;
}

/**
 * @brief Return the frame-time at the last event pulse
 */
int timesync_get_last_event(uint64_t *frame_time) {

    if (!frame_time)
        return -EINVAL;

    if (timesync_state != TIMESYNC_STATE_ACTIVE) {
        return -ENODEV;
    }
    *frame_time = timesync_last_event_time;
    return 0;
}

void timesync_exit(void) {
    timesync_disable();
}

/*
 * System entrypoint. CONFIG_USER_ENTRYPOINT should point to this function.
 */
int timesync_init(void) {
#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
    int rc;
#endif

    timesync_rollover_timer = tsb_tmr_get(TSB_TMR_TMR3);
    if (!timesync_rollover_timer) {
        lldbg("error initializing master time-sync timer TSB_TMR3\n");
        PANIC();
        return ERROR;
    }

    lldbg("Frame-time basic-frequency %dHz\n", TSB_TMR_FREQUENCY);
    timesync_disable();

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
    /* timesync_debug exists for debug and informational purposes only */
    sem_init(&dbg_thread_sem, 0, 0);
    rc = task_create("timesync_debugd", TIMESYNC_TASK_PRIORITY,
                     TIMESYNC_TASK_STACK_SIZE, timesync_debugd_main, NULL);
    if (rc == ERROR) {
        dbg_error("failed to start timesync_debugd\n");
        return rc;
    }
#endif

    return 0;
}
