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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <nuttx/util.h>
#include <nuttx/config.h>

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
    MAX_CMD,
};

static const struct command commands[] = {
    [HELP] = {'h', "help", "print this usage and exit"},
#ifdef CONFIG_USB_DUMP
    [DUMP] = {'d', "dump", "dump usb device registers"},
#endif
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
    default:
        usage(EXIT_FAILURE);
    }

    return rc;
}
