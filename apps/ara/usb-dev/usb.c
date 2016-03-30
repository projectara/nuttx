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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <nuttx/util.h>
#include <nuttx/config.h>
#include <nuttx/unipro/unipro.h>
#include <arch/board/apbridgea_debug.h>

#include "dwc_otg_pcd_debug.h"

struct command {
    const char shortc;
    const char *longc;
    const char *help;
};

#define INVALID -1
enum {
    HELP,
#ifdef CONFIG_USB_DUMP
    DUMP,
#endif
#ifdef CONFIG_USB_LOG
    LOG,
#endif
    DEBUG,
    MAX_CMD,
};

static const struct command commands[] = {
    [HELP] = {'h', "help", "print this usage and exit"},
#ifdef CONFIG_USB_DUMP
    [DUMP] = {'d', "dump", "dump usb device registers"},
#endif
#ifdef CONFIG_USB_LOG
    [LOG] = {'l', "log", "set dwc otg driver log level"},
#endif
    [DEBUG] = {'D', "debug", "debug commands"}
};

static void usage(int exit_status) {
    int i;
    printf("usb: usage:\n");
    for (i = 0; i < MAX_CMD; i++) {
        printf("    usb [%c|%s] : %s\n",
               commands[i].shortc, commands[i].longc, commands[i].help);
    }
    exit(exit_status);
}

#ifdef CONFIG_USB_DUMP
static void usb_dump_usage(int exit_status)
{
    printf("usb %s: usage:\n", commands[DUMP].longc);
    printf("    -h: print this message and exit\n");
    printf("\n");
    printf("    -g: Dump the global device registers.\n");
    printf("    -e <epnum>: Endpoint's registers to dump.\n");
    exit(exit_status);
}

static int usb_dump(int argc, char *argv[])
{
    char **args = argv + 1;
    int c;
    int i;
    int rc = 0;

    int epnum = -1;
    int global = 0;

    const char opts[] = "he:g";

    argc--;
    optind = -1; /* Force NuttX's getopt() to reinitialize. */

    if (argc < 2) {
        usb_dump_usage(EXIT_SUCCESS);
    }

    while ((c = getopt(argc, args, opts)) != -1) {
        switch (c) {
        case 'h':
            usb_dump_usage(EXIT_SUCCESS);
            break;
        case 'e':
            rc = sscanf(optarg, "%d", &epnum);
            if (rc != 1) {
                printf("A valid endpoint number is expected\n");
                usb_dump_usage(EXIT_FAILURE);
            }
            break;
        case 'g':
            global = true;
            break;
        default:
            printf("Unrecognized argument '%c'.\n", (char)c);
            usb_dump_usage(EXIT_FAILURE);
        }
    }

    if (global)
        usb_dump_global_device();
    if (epnum == -1) {
        for (i = 0; i < DWC_NENDPOINTS; i++) {
            usb_dump_ep(i);
        }
    } else {
        usb_dump_ep(epnum);
    }

    return rc;
}
#endif /* CONFIG_USB_DUMP */

#ifdef CONFIG_USB_LOG
extern uint32_t g_dbg_lvl;

static void usb_log_usage(int exit_status)
{
    printf("usb %s <level_bitmask>:\n", commands[LOG].longc);
    exit(exit_status);
}

static int usb_log(int argc, char *argv[])
{
    char **args = argv + 1;
    int level;
    int rc = 0;
    argc--;

    if (argc < 2 || !strcmp(args[1], "-h")) {
        usb_log_usage(EXIT_FAILURE);
    }

    rc = sscanf(args[1], "%x", &level);
    if (rc != 1) {
        printf("A valid log level is expected\n");
        usb_log_usage(EXIT_FAILURE);
    }

    /* TODO: check the value */
    g_dbg_lvl = level;

    return rc;
}
#endif

static void usb_debug_usage(int exit_status)
{
    printf("usb %s: usage:\n", commands[DEBUG].longc);
    printf("    -h: print this message and exit\n");
    printf("\n");
    printf("    -d <cport_id>: Drop data comming from or going to a cport\n");
    printf("    -t <cport_id>: Transfer data comming from or going to a cport\n");
    printf("    -C <epnum>: Cancel pending requests\n");
    exit(exit_status);
}

static int usb_debug(int argc, char *argv[])
{
    char **args = argv + 1;
    int c;
    int rc = 0;

    int drop = 0;
    int transfer = 0;
    unsigned int epnum;
    unsigned int cport_id = -1;

    const char opts[] = "hdtc:C:";

    argc--;
    optind = -1; /* Force NuttX's getopt() to reinitialize. */

    if (argc < 2) {
        usb_debug_usage(EXIT_SUCCESS);
    }

    while ((c = getopt(argc, args, opts)) != -1) {
        switch (c) {
        case 'h':
            usb_debug_usage(EXIT_SUCCESS);
            break;
        case 'd':
            drop = 1;
            break;
        case 't':
            transfer = 1;
            break;
        case 'c':
            rc = sscanf(optarg, "%u", &cport_id);
            if (rc != 1 || cport_id >= unipro_cport_count()) {
                printf("A valid cport id is expected\n");
                usb_debug_usage(EXIT_FAILURE);
            }
            break;
        case 'C':
            rc = sscanf(optarg, "%u", &epnum);
            if (rc != 1 || usb_debug_ep_cancel(epnum)) {
                printf("A valid endpoint number is expected\n");
                usb_debug_usage(EXIT_FAILURE);
            }
            return 0;
        default:
            printf("Unrecognized argument '%c'.\n", (char)c);
            usb_debug_usage(EXIT_FAILURE);
        }
    }

    if (cport_id == -1 && (drop || transfer)) {
        printf("A cport id is expected\n");
        usb_debug_usage(EXIT_FAILURE);
    }

    if (drop) {
        apbridgea_enable_cport_dropping(cport_id);
    }

    if (transfer) {
        apbridgea_disable_cport_dropping(cport_id);
    }

    return rc;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int usb_main(int argc, char *argv[])
#endif
{
    int i;
    int rc = 0;
    int cmd = INVALID;
    const char *cmd_str;

    /* Parse arguments. */
    if (argc < 2) {
        usage(EXIT_FAILURE);
    }
    cmd_str = argv[1];
    for (i = 0; i < MAX_CMD; i++) {
        if (!strcmp(cmd_str, commands[i].longc)) {
            cmd = i;
            break;
        } else if (strlen(cmd_str) == 1 &&
                   cmd_str[0] == commands[i].shortc) {
            cmd = i;
            break;
        }
    }
    if (cmd == INVALID) {
        usage(EXIT_FAILURE);
    }

    /* Run the command. */
    switch (cmd) {
    case HELP:
        usage(EXIT_SUCCESS);
#ifdef CONFIG_USB_DUMP
    case DUMP:
        rc = usb_dump(argc, argv);
        break;
#endif
#ifdef CONFIG_USB_LOG
    case LOG:
        rc = usb_log(argc, argv);
        break;
#endif
    case DEBUG:
        rc = usb_debug(argc, argv);
        break;
    default:
        usage(EXIT_FAILURE);
    }

    return rc;
}
