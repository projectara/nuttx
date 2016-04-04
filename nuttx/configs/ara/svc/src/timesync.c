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

#include <nuttx/greybus/greybus.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <arch/board/board.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <ara_debug.h>

#include "ara_board.h"
#include "interface.h"
#include "timesync.h"

#define TIMESYNC_TASK_PRIORITY      (50)
#define TIMESYNC_TASK_STACK_SIZE    (1024)

/* The current local frame-time */
static uint64_t timesync_frame_time;
static uint64_t timesync_strobe_time[GB_TIMESYNC_MAX_STROBES];

static int timesync_state = TIMESYNC_STATE_INVALID;
static uint32_t timesync_strobe_delay;
static uint32_t timesync_pin_strobe_mask;
static uint32_t timesync_refclk;
static int timesync_strobe_count;
static int timesync_strobe_index;
static struct stm32_tim_dev_s *timesync_rollover_master_timer;
static struct stm32_tim_dev_s *timesync_rollover_slave_timer;
static struct stm32_tim_dev_s *timesync_strobe_timer;
static int timesync_rollover_timer_irq = -1;
static int timesync_strobe_timer_irq = -1;
static int timesync_debug_pid = -1;

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
static uint64_t timesync_strobe_time_counter[GB_TIMESYNC_MAX_STROBES];
static sem_t dbg_thread_sem;
#endif

/*
 * Define CONFIG_ARCH_TIMESYNC_DEBUG to get printouts locally and a 1 second strobe
 * after completion of the intial synchronization operation
 */

/* TIME_SYNC strobe timer */
#define TIMESYNC_STROBE_TIMER_ID            6
#define TIMESYNC_STROBE_TIMER_PERIOD        1000000

/* Cascaded rollover timer */
#define TIMESYNC_ROLLOVER_TIMER_MASTER_ID   8
#define TIMESYNC_ROLLOVER_TIMER_SLAVE_ID    4
#define TIMESYNC_ROLLOVER_TIMER_PERIOD      0xFFFF
#define TIMESYNC_ROLLOVER_CASCADE_TOTAL     0x100000000ULL

/* Interface hackery */
#define WD_PORT                             GPIO_PORTA

/**
 * @brief Return the current hardware count of clocks - accounting for lower
 *        16 bit over-flow during read
 *
 * Latch the two-step hardware read
 */
static uint32_t timesync_get_counter(void) {
    uint16_t hi, tmp, lo;

    while (1) {
        hi = STM32_TIM_GETCOUNTER(timesync_rollover_slave_timer);
        lo = STM32_TIM_GETCOUNTER(timesync_rollover_master_timer);
        tmp = STM32_TIM_GETCOUNTER(timesync_rollover_slave_timer);
        if (tmp == hi)
            return ((uint32_t) hi << 16) | lo;
    }
};

/**
 * @brief Return a 64 bit frame time expressed in nanoseconds - accounting for
 *        lower 32 bit over-flow during read
 *
 * Latch lower 32 bits (hardware) and upper 32 bits soft-IRQ tracked
 * to derive a 64 frame-time, in a similar fashion to timesync_get_counter()
 * except we add instead of 'or' in the lower value.
 */
uint64_t timesync_get_frame_time(void) {
    uint64_t hi, tmp;
    uint32_t lo;

    while (1) {
        hi = timesync_frame_time;
        lo = timesync_get_counter();
        tmp = timesync_frame_time;
        if (tmp == hi)
            return (hi + lo);
    }
}

/**
 * @brief return the current state of TIME_SYNC
 *
 * Allows a caller to understand if the frame-time returned by
 * timesync_get_frame_time() is currently ACTIVE and hence trusted or not
 * ACTIVE and hence not to be relied upon.
 */
int timesync_get_state(void) {
    return timesync_state;
}

/**
 * @brief Responsible for handling roll-over of the lower 32 bit integer into the 64 bit frame-time
 *
 * Handily the amount to append to the frame-time will always be 0x100000000
 */
static int timesync_frame_time_rollover_handler(int irq, void *context) {

    STM32_TIM_ACKINT(timesync_rollover_slave_timer, irq);
    timesync_frame_time += TIMESYNC_ROLLOVER_CASCADE_TOTAL;

    return 0;
}

/**
 * @brief Logs the frame time. If compiled with CONFIG_ARCH_TIMESYNC_DEBUG logs the raw counter value too
 *
 */
static void timesync_log_frame_time(uint64_t strobe_time) {
    if (timesync_strobe_index < GB_TIMESYNC_MAX_STROBES) {
        timesync_strobe_time[timesync_strobe_index] = strobe_time;
#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
        timesync_strobe_time_counter[timesync_strobe_index] = timesync_get_counter();
#endif
        timesync_strobe_index += 1;
    }
}

/**
 * @brief Context safe state transition routine.
 *
 */
static void timesync_set_state(int state) {
    irqstate_t flags;

    flags = irqsave();

    switch (state) {
    case TIMESYNC_STATE_INVALID:
        interfaces_timesync_fini();
        break;
    case TIMESYNC_STATE_INACTIVE:
    case TIMESYNC_STATE_ACTIVE:
    case TIMESYNC_STATE_DEBUG_INIT:
    case TIMESYNC_STATE_DEBUG_ACTIVE:
    case TIMESYNC_STATE_SYNCING:
        break;
    }
    timesync_state = state;

    irqrestore(flags);
}

/**
 * @brief Write the WD_PORT pins simultaneously for TIME_SYNC
 *        Capture the authoritative frame on the rising edge of the output.
 */
static void timesync_strobe(uint32_t strobe_mask, uint64_t *latch_frame_time) {
    stm32_gpio_set_port_pins(WD_PORT, strobe_mask, true);
    *latch_frame_time = timesync_get_frame_time();
    stm32_gpio_set_port_pins(WD_PORT, strobe_mask, false);
}

/**
 * @brief Responsible for sending out the TIME_SYNC pulse
 *
 * Logs the time of the strobe and strobes. If the requested iteration
 * count is reached we transition to the TIMESYNC_STATE_ACTIVE state or
 * the TIMESYNC_STATE_DEBUT_INIT state depending on CONFIG_ARCH_TIMESYNC_DEBUG.
 *
 */
static int timesync_strobe_handler(int irq, void *context) {
    irqstate_t flags;
    uint64_t strobe_time;

    flags = irqsave();

    STM32_TIM_ACKINT(timesync_strobe_timer, irq);

    timesync_strobe(timesync_pin_strobe_mask, &strobe_time);
    timesync_log_frame_time(strobe_time);

    if (timesync_strobe_index == timesync_strobe_count) {
        STM32_TIM_DISABLEINT(timesync_strobe_timer, 0);
        STM32_TIM_SETMODE(timesync_strobe_timer, STM32_TIM_MODE_DISABLED);
        STM32_TIM_SETCLOCK(timesync_strobe_timer, 0);
#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
        timesync_set_state(TIMESYNC_STATE_DEBUG_INIT);
        sem_post(&dbg_thread_sem);
#else
        timesync_set_state(TIMESYNC_STATE_ACTIVE);
#endif
    }

    irqrestore(flags);
    return 0;
}

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
/**
 * @brief print TIME_SYNC debug metrics
 *
 * Note this thread exists in-lieu of a hardware timer based
 * solution - considered to be a WIP and not a real-time solution
 */
static int timesync_debugd_main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    uint8_t i;

    while (timesync_state != TIMESYNC_STATE_INVALID) {
        /* Don't block when in the active debug state */
        if (timesync_state != TIMESYNC_STATE_DEBUG_ACTIVE)
            sem_wait(&dbg_thread_sem);

        if (timesync_state == TIMESYNC_STATE_INVALID)
            break;

        switch (timesync_state) {
        case TIMESYNC_STATE_DEBUG_ACTIVE:
            lldbg("frame-time %llu\n",
                  timesync_get_frame_time());
            sleep(1);
            break;
        case TIMESYNC_STATE_DEBUG_INIT:
            for (i = 1; i < timesync_strobe_index; i++) {
                lldbg("frame-time diff %llu-useconds %llu-%llu counter-diff is %llu %llu-%llu\n",
                      timesync_strobe_time[i] - timesync_strobe_time[i-1],
                      timesync_strobe_time[i], timesync_strobe_time[i-1],
                      timesync_strobe_time_counter[i] - timesync_strobe_time_counter[i - 1],
                      timesync_strobe_time_counter[i], timesync_strobe_time_counter[i - 1]);
            }
            timesync_set_state(TIMESYNC_STATE_DEBUG_ACTIVE);
            break;
        default:
            break;
        }
    }

    return 0;
}
#endif

/**
 * @brief Generate a ping to the given bit-mask of interfaces
 *        return authoritative time at the ping.
 *        AP must have already completed a timesync_wd_pins_init() routine
 */
int timesync_ping(uint64_t *frame_time) {

    irqstate_t flags;

    if (!frame_time)
        return -EINVAL;

    if (timesync_state != TIMESYNC_STATE_DEBUG_ACTIVE &&
        timesync_state != TIMESYNC_STATE_ACTIVE) {
        lldbg("timesync invalid state %d cannot ping\n", timesync_state);
        return -ENODEV;
    }

    flags = irqsave();
    timesync_strobe(timesync_pin_strobe_mask, frame_time);
    irqrestore(flags);

    dbg_verbose("Ping pinmask=0x%08lx frame-time=%llu\n", timesync_pin_strobe_mask,
                *frame_time);
    return 0;
}

/**
 * @brief Take ownership of WDx pins prior to issuing timesync strobe signals
 *        ensuring a known initial state on WDM lines before APB/GPB get
 *        transitioned to monitor incoming timesync strobe signals
 */
int timesync_wd_pins_init(uint32_t interface_strobe_mask) {
    struct interface *intf_apb1, *intf_apb2;

    if (timesync_state == TIMESYNC_STATE_SYNCING ||
        timesync_state == TIMESYNC_STATE_INVALID) {
        return -EBUSY;
    }

    intf_apb1 = interface_get_by_name(INTF_APB1);
    intf_apb2 = interface_get_by_name(INTF_APB2);

    if (intf_apb1) {
        interface_strobe_mask |= 1 << intf_apb1->dev_id;
    } else {
        lldbg("Couldn't find APB1 interface!\n");
    }
    if (intf_apb2) {
        interface_strobe_mask |= 1 << intf_apb2->dev_id;
    } else {
        lldbg("Couldn't find APB2 interface!\n");
    }

    timesync_pin_strobe_mask = interfaces_timesync_init(interface_strobe_mask);
    dbg_verbose("interface-mask 0x%08lx strobe-mask 0x%08lx\n",
                interface_strobe_mask, timesync_pin_strobe_mask);
    return 0;
}

/**
 * @brief Releases WDx pins after timesync-enable or timesync-ping operations
 */
int timesync_wd_pins_fini(void) {
    interfaces_timesync_fini();
    return 0;
}

/**
 * @brief Disable timers
 */
static void timesync_disable_timers(void) {
    STM32_TIM_DISABLEINT(timesync_strobe_timer, 0);
    STM32_TIM_SETMODE(timesync_strobe_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(timesync_strobe_timer, 0);
    STM32_TIM_DISABLEINT(timesync_rollover_master_timer, 0);
    STM32_TIM_SETMODE(timesync_rollover_master_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(timesync_rollover_master_timer, 0);
    STM32_TIM_DISABLEINT(timesync_rollover_slave_timer, 0);
    STM32_TIM_SETMODE(timesync_rollover_slave_timer, STM32_TIM_MODE_DISABLED);
    STM32_TIM_SETCLOCK(timesync_rollover_slave_timer, 0);
}

/**
 * @brief Disable TimeSync
 */
int timesync_disable(void) {
    irqstate_t flags;

    flags = irqsave();
    timesync_disable_timers();
    timesync_set_state(TIMESYNC_STATE_INACTIVE);

    irqrestore(flags);

    return 0;
}

/**
 * @brief Enable TimeSync at the given reference frame_time
 *
 * Setup TIM8 as master clock cascading into TIM4 see DocID018909 Rev 10 page 612
 * https://www.rapitasystems.com/blog/chaining-two-16-bit-timers-together-stm32f4
 * 622/1728 DocID018909 Rev 10 - details which timers can be cascaded
 * TIM8 has been selected since we can clock that @ 96MHz and divide down by 5 to
 * get to 19.2MHz with just a little work @ the PLL configuration.
 */
int timesync_enable(uint8_t strobe_count, uint64_t frame_time,
                    uint32_t strobe_delay, uint32_t refclk) {
    if (!strobe_count || strobe_count > GB_TIMESYNC_MAX_STROBES) {
        return -EINVAL;
    }

    if (!strobe_delay) {
        return -EINVAL;
    }

    if (!refclk || STM32_TIM18_FREQUENCY % refclk) {
        lldbg("Error Time-Sync clock %dHz doesn't divide APB clock %dHz evenly\n",
              refclk, STM32_TIM18_FREQUENCY);
        return -ENODEV;
    }

    timesync_refclk = refclk;
    timesync_strobe_count = strobe_count;
    timesync_strobe_index = 0;
    timesync_strobe_delay = strobe_delay;
    timesync_set_state(TIMESYNC_STATE_SYNCING);
    timesync_frame_time = frame_time;

    /* Disable timers */
    timesync_disable_timers();

    /*******************************************************************
     * Enable TIM8 against the hardware input clock output on TIM8_TRGO
     *******************************************************************/

    /* Configure MMS=010 in TIM8_CR2 output TIM8_TRGO on rollover (master mode) */
    STM32_TIM_SETMASTER_MODE(timesync_rollover_master_timer, STM32_TIM_MASTER_MODE_UPDATE);
    /* Configures TIM8_ARR - auto reload register */
    STM32_TIM_SETPERIOD(timesync_rollover_master_timer, TIMESYNC_ROLLOVER_TIMER_PERIOD);
    /* Configures TIM8_PSC - prescaler value to get our desired master clock */
    STM32_TIM_SETCLOCK(timesync_rollover_master_timer, timesync_refclk);

    /*********************************************************************
     * Enable TIM4 with clock input source ITR2 (TIM8_TRGO), no pre-scaler
     * interrupt on over-flow
     *********************************************************************/

    /* Configure ITR0 as internal trigger TIM4_SMCR:TS=011(TIM8) external clock mode TIM4_SMCR:SMS=111 (slave mode) */
    STM32_TIM_SETSLAVE_MODE(timesync_rollover_slave_timer, STM32_TIM_SLAVE_EXTERNAL_MODE, STM32_TIM_SLAVE_INTERNAL_TRIGGER3);
    /* Configures TIM4_ARR - auto reload register */
    STM32_TIM_SETPERIOD(timesync_rollover_slave_timer, TIMESYNC_ROLLOVER_TIMER_PERIOD);
    /* Configures TIM4_PSC - set to STM32_TIM18_FREQUENCY to get a prescaler value of 0 */
    STM32_TIM_SETCLOCK(timesync_rollover_slave_timer, STM32_TIM18_FREQUENCY);

    /****************************************************************************
     * Enable TIM6 as a simple up-counter at the strobe_delay specified by the AP
     * strobe_delay is expressed in microseconds.
     ****************************************************************************/

    /* Delay is expressed in microseconds so apply directly to TIM6_ARR */
    STM32_TIM_SETPERIOD(timesync_strobe_timer, strobe_delay);
    /* Clock is simply the fundamental input Hz (clocks per second) / 1000 */
    STM32_TIM_SETCLOCK(timesync_strobe_timer, 1000000UL);

    /***************
     * Enable timers
     ***************/

    /* Configures TIM8_CR1 - mode up-counter */
    STM32_TIM_SETMODE(timesync_rollover_master_timer, STM32_TIM_MODE_UP);
    /* Configures TIM4_CR1 - mode up-counter */
    STM32_TIM_SETMODE(timesync_rollover_slave_timer, STM32_TIM_MODE_UP);
    /* Configures TIM6_CR1 - mode up-counter */
    STM32_TIM_SETMODE(timesync_strobe_timer, STM32_TIM_MODE_UP);

    /* Enable roll-over timer interrupt */
    STM32_TIM_ACKINT(timesync_rollover_slave_timer, timesync_rollover_timer_irq);
    STM32_TIM_ENABLEINT(timesync_rollover_slave_timer, 0);

    /* Enable strobe timer interrupt */
    STM32_TIM_ACKINT(timesync_strobe_timer, timesync_strobe_timer_irq);
    STM32_TIM_ENABLEINT(timesync_strobe_timer, 0);

    dbg_verbose("ref-clk-freq=%dHz timer-freq=%dHz period=%d\n",
                STM32_TIM18_FREQUENCY, timesync_refclk,
                TIMESYNC_ROLLOVER_TIMER_PERIOD);
    dbg_verbose("strobe-clk=%dHz strobe-pin-mask=0x%08lx period=%d\n",
                STM32_TIM18_FREQUENCY / 1000UL, timesync_pin_strobe_mask, strobe_delay);

    return 0;
}

/**
 * @brief Return each authoritative frame_time to the passed frame_time output
 */
int timesync_authoritative(uint64_t *frame_time) {
    memcpy(frame_time, timesync_strobe_time, sizeof(uint64_t) * GB_TIMESYNC_MAX_STROBES);
    return 0;
}

/**
 * @brief exit routine
 */
void timesync_exit(void) {
    int status;

    timesync_disable();

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
    timesync_set_state(TIMESYNC_STATE_INVALID);
    sem_post(&dbg_thread_sem);
    waitpid(timesync_debug_pid, &status, 0);
#else
    (void)status;
    (void)timesync_debug_pid;
#endif
}

/**
 * @brief System entrypoint. CONFIG_USER_ENTRYPOINT should point to this function.
 */
int timesync_init(void) {
    int rc;

    timesync_rollover_master_timer = stm32_tim_init(TIMESYNC_ROLLOVER_TIMER_MASTER_ID);
    if (!timesync_rollover_master_timer) {
        lldbg("error initializing master time-sync rollover timer #%d\n",
              TIMESYNC_ROLLOVER_TIMER_MASTER_ID);
        PANIC();
        return ERROR;
    }
    timesync_rollover_slave_timer = stm32_tim_init(TIMESYNC_ROLLOVER_TIMER_SLAVE_ID);
    if (!timesync_rollover_slave_timer) {
        lldbg("error initializing slave time-sync rollover timer #%d\n",
              TIMESYNC_ROLLOVER_TIMER_SLAVE_ID);
        PANIC();
        return ERROR;
    }

    timesync_strobe_timer = stm32_tim_init(TIMESYNC_STROBE_TIMER_ID);
    if (!timesync_strobe_timer) {
        lldbg("error initializing timesync strobe timer #%d\n",
              TIMESYNC_STROBE_TIMER_ID);
        PANIC();
        return ERROR;
    }

    /* TIM8 fundamental frequency */
    lldbg("Frame-time basic-frequency %dHz\n", STM32_TIM18_FREQUENCY);

    timesync_disable();
    timesync_rollover_timer_irq = STM32_TIM_SETISR(timesync_rollover_slave_timer,
                                                   timesync_frame_time_rollover_handler, 0);
    if (timesync_rollover_timer_irq == ERROR) {
        lldbg("Unable to latch timer #%d interrupt!\n",
              TIMESYNC_ROLLOVER_TIMER_SLAVE_ID);
        PANIC();
        return ERROR;
    }
    timesync_strobe_timer_irq = STM32_TIM_SETISR(timesync_strobe_timer,
                                                 timesync_strobe_handler, 0);
    if (timesync_strobe_timer_irq == ERROR) {
        lldbg("Unable to latch timer #%d interrupt!\n",
              TIMESYNC_STROBE_TIMER_ID);
        PANIC();
        return ERROR;
    }

#ifdef CONFIG_ARCH_TIMESYNC_DEBUG
    /* timesync_debug exists for debug and informational purposes only */
    sem_init(&dbg_thread_sem, 0, 0);

    rc = task_create("timesync_debugd", TIMESYNC_TASK_PRIORITY,
                     TIMESYNC_TASK_STACK_SIZE, timesync_debugd_main, NULL);
    if (rc == ERROR) {
        dbg_error("failed to start timesync_debugd\n");
        return rc;
    }
    timesync_debug_pid = rc;
#else
    (void)rc;
#endif

    return 0;
}
