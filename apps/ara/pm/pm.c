/**
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
 * Author: Bartosz Golaszewski <bartekgola@gmail.com>
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/power/pm.h>
#include <nuttx/util.h>
#include <arch/tsb/pm.h>

static const char help_txt[] =
    "Power management utility\n\n"
    "usage: pm <options>\n\n"
    "options:\n"
    "\t-r:\t\tregister with pm framework\n"
    "\t-a <level>:\tnotify the pm framework about new activity\n"
    "\t-d:\t\tdisable power management\n"
    "\t-e:\t\tre-enable power management\n"
    "\t-s:\t\tprint current power state\n"
    "\t-c <NORMAL|IDLE|STANDBY|SLEEP>:\n"
    "\t\tforce power state change for all drivers\n";

struct pm_state_descr {
    int flag;
    const char *name;
};

static struct pm_state_descr pm_states[] = {
    {
        .flag = PM_NORMAL,
        .name = "NORMAL",
    },
    {
        .flag = PM_IDLE,
        .name = "IDLE",
    },
    {
        .flag = PM_STANDBY,
        .name = "STANDBY",
    },
    {
        .flag = PM_SLEEP,
        .name = "SLEEP",
    }
};

static const char *state_to_str(enum pm_state_e pmstate)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(pm_states); i++) {
        if (pm_states[i].flag == pmstate)
            return pm_states[i].name;
    }

    return "UNDEFINED";
}

static int str_to_state(const char *state)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(pm_states); i++) {
        if (strcmp(pm_states[i].name, state) == 0)
            return pm_states[i].flag;
    }

    return -1;
}

static void pm_notify(struct pm_callback_s *cb, enum pm_state_e pmstate)
{
    printf("[POWER] state changed to %s\n", state_to_str(pmstate));
}

static struct pm_callback_s pmctx = {
    .notify = pm_notify,
};

int pm_main(int argc, char **argv)
{
    int opt, status, state;

    if (argc < 2) {
        puts(help_txt);
        return EXIT_FAILURE;
    }

    optind = -1;
    while ((opt = getopt (argc, argv, "ra:desc:")) != -1) {
        switch (opt) {
        case 'r':
            status = pm_register(&pmctx);
            if (status != OK) {
                printf("error registering pm callbacks: %d\n", status);
                return EXIT_FAILURE;
            }
            break;
        case 'a':
            pm_activity(strtoul(optarg, NULL, 10));
            break;
        case 'd':
            tsb_pm_disable();
            break;
        case 'e':
            tsb_pm_enable();
            break;
        case 's':
            puts(state_to_str(tsb_pm_getstate()));
            break;
        case 'c':
            state = str_to_state(optarg);
            if (state < 0) {
                puts(help_txt);
                return EXIT_FAILURE;
            }

            status = tsb_pm_driver_state_change(state);
            if (status != OK) {
                printf("error forcing power state change: %d\n", status);
                return EXIT_FAILURE;
            }
            break;
        default:
            puts(help_txt);
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
