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
 * @file i2c.c
 * @brief Ara Toshiba bridge ASIC I2C test program
 * @author Joel Porquet <joel@porquet.org>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_i2c.h>

/* specify an register address
 * (otherwise, continue from the currently active register) */
#define FLAG_SET_ADDR    0x1
/* the address is 16 bits */
#define FLAG_ADDR16      0x2

#define CMD_INVALID -1
enum {
    CMD_HELP,
    CMD_LIST,
    CMD_GET,
    CMD_SET,
    CMD_MAX,
};

struct command {
    const char shortc;
    const char *longc;
    const char *argspec;
    const char *help;
};

static const struct command commands[CMD_MAX] = {
    [CMD_HELP]  = {'h', "help", NULL, "Print this message and exit"},
    [CMD_LIST]  = {'l', "list", "<i2cbus> [first last]",
                   "Detect I2C chips (between 0x03 and 0x77 by default)"},
    [CMD_GET]   = {'g', "get", "<i2cbus> <chip-address> [data-address]",
                   "Read from I2C chip registers"},
    [CMD_SET]   = {'s', "set", "<i2cbus> <chip-address> <data-address> [value] ...",
                   "Write to I2C chip registers (max: 32 values)"},
};

static void print_usage(void)
{
    int i;

    fprintf(stderr, "i2c: usage:\n");
    fprintf(stderr,
            "    i2c [16]: enable 16-bit register addressing\n"
            "              (address must be specified in two bytes, high byte first)\n");
    for (i = 0; i < CMD_MAX; i++) {
        const char *argspec = commands[i].argspec;
        const char *space = " ";

        if (!argspec) {
            space = "";
            argspec = "";
        }
        fprintf(stderr, "    i2c [16] [%c|%s]%s%s: %s\n",
                commands[i].shortc, commands[i].longc,
                space, argspec,
                commands[i].help);
    }
}

static void do_list(uint8_t i2cbus, uint8_t *bounds)
{
    int i, j;
    struct device *i2c_dev;

    i2c_dev = device_open(DEVICE_TYPE_I2C_HW, i2cbus);
    if (!i2c_dev) {
        fprintf(stderr, "Failed to open I2C controller on bus %d\n", i2cbus);
        return;
    }

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

    for (i = 0; i < ((bounds[1] / 16) + 1); i++) {

        printf("%02x:", i);

        for (j = 0; j < 16; j++) {
            struct device_i2c_request request;
            int ret;

            uint8_t chip_addr = i*16 + j;
            uint8_t data;

            if (chip_addr < bounds[0] || chip_addr > bounds[1]) {
                printf("   ");
                continue;
            }

            request.addr = chip_addr;
            request.flags = I2C_FLAG_READ;
            request.buffer = &data;
            request.length = 1;

            ret = device_i2c_transfer(i2c_dev, &request, 1);

            if (!ret)
                printf(" %02x", chip_addr);
            else
                printf(" --");
        }
        printf("\n");
    }

    device_close(i2c_dev);
}

static void do_get(uint8_t i2cbus, uint8_t chip_addr,
                   uint8_t flags, uint8_t *payload)
{
    int ret;
    struct device *i2c_dev;
    struct device_i2c_request request;
    uint8_t data;

    i2c_dev = device_open(DEVICE_TYPE_I2C_HW, i2cbus);
    if (!i2c_dev) {
        fprintf(stderr, "Failed to open I2C controller on bus %d\n", i2cbus);
        return;
    }

    if (flags & FLAG_SET_ADDR) {
        request.addr = chip_addr;
        request.flags = 0;
        request.buffer = payload;
        request.length = 1;
        if (flags & FLAG_ADDR16) {
            request.length = 2;
        }

        ret = device_i2c_transfer(i2c_dev, &request, 1);
        if (ret) {
            fprintf(stderr, "Failed to set the register address: 0x%02x 0x%02x\n",
                    payload[0], payload[1]);
            goto out;
        }
    }

    request.addr = chip_addr;
    request.flags = I2C_FLAG_READ;
    request.buffer = &data;
    request.length = 1;

    ret = device_i2c_transfer(i2c_dev, &request, 1);
    if (ret) {
        fprintf(stderr, "Failed to access the register at address 0x%02x 0x%02x\n",
                payload[0], payload[1]);
        goto out;
    }

    printf("0x%02x\n", data);

out:
    device_close(i2c_dev);
    return;
}

static void do_set(uint8_t i2cbus, uint8_t chip_addr,
                   uint8_t flags, uint8_t *payload, uint8_t count)
{
    int ret;
    struct device *i2c_dev;
    struct device_i2c_request request;

    i2c_dev = device_open(DEVICE_TYPE_I2C_HW, i2cbus);
    if (!i2c_dev) {
        fprintf(stderr, "Failed to open I2C controller on bus %d\n", i2cbus);
        return;
    }

    request.addr = chip_addr;
    request.flags = 0;
    request.buffer = payload;
    request.length = count;

    ret = device_i2c_transfer(i2c_dev, &request, 1);
    if (ret) {
        printf("Failed to access the register(s) at address 0x%02x 0x%02x\n",
                payload[0], payload[1]);
        goto out;
    }

    printf("Success\n");

out:
    device_close(i2c_dev);
    return;
}

#define GET_ARG(argv) (uint8_t)strtol(*argv++, NULL, 0)

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int i2c_main(int argc, char *argv[])
#endif
{
    int i;
    int cmd = CMD_INVALID;
    uint8_t flags = 0, sz_reg_addr = 1;
    uint8_t i2cbus, chip_addr, payload[34];
    uint8_t bounds[2] = {0x03, 0x77};
    char **cur_argv, *cmd_str;
    int rc = EXIT_SUCCESS;

    cur_argv = &argv[1];
    argc -= 1;
get_cmd:
    if (argc < 1)
        goto err;

    if (!strcmp(*cur_argv, "16")) {
        flags |= FLAG_ADDR16;
        sz_reg_addr = 2;
        cur_argv++;
        argc -= 1;
        goto get_cmd;
    }

    cmd_str = *cur_argv++;
    argc -= 1;
    for (i = 0; i < CMD_MAX; i++) {
        if (!strcmp(cmd_str, commands[i].longc) ||
                (strlen(cmd_str) == 1 && cmd_str[0] == commands[i].shortc)) {
            cmd = i;
            break;
        }
    }

    /* parse command-specific arguments */
    switch (cmd) {
        case CMD_HELP:
            print_usage();
            break;
        case CMD_LIST:
            if (argc < 1) {
                fprintf(stderr, "Error: you must specify an I2C bus\n");
                goto err;
            }
            i2cbus = GET_ARG(cur_argv);
            argc -= 1;
            for (i = 0; i < argc && i < 2; i++)
                bounds[i] = GET_ARG(cur_argv);
            do_list(i2cbus, bounds);
            break;
        case CMD_GET:
            if (argc < 2) {
                fprintf(stderr, "Error: you must specify an I2C bus and a chip address\n");
                goto err;
            }
            i2cbus = GET_ARG(cur_argv);
            chip_addr = GET_ARG(cur_argv);
            argc -= 2;
            if (argc !=0) {
                if (argc < sz_reg_addr) {
                    fprintf(stderr, "Error: the register address must be a 16-bit value\n");
                    goto err;
                }
                for (i = 0; i < sz_reg_addr; i++) {
                    flags |= FLAG_SET_ADDR;
                    payload[i] = GET_ARG(cur_argv);
                }
            }
            do_get(i2cbus, chip_addr, flags, payload);
            break;
        case CMD_SET:
            if (argc < 3) {
                fprintf(stderr,
                        "Error: you must specify an I2C bus, a chip address and a register address\n");
                goto err;
            }
            i2cbus = GET_ARG(cur_argv);
            chip_addr = GET_ARG(cur_argv);
            argc -= 2;
            if (argc < sz_reg_addr) {
                fprintf(stderr, "Error: the register address is of the wrong size\n");
                goto err;
            }
            /* reg address and values are concatenated into the same payload,
             * starting with the address (this is to avoid an i2c restart
             * between the address and the values) */
            for (i = 0; i < sz_reg_addr; i++) {
                flags |= FLAG_SET_ADDR;
                payload[i] = GET_ARG(cur_argv);
            }
            for (; i < argc; i++) {
                payload[i] = GET_ARG(cur_argv);
            }
            do_set(i2cbus, chip_addr, flags, payload, i);
            break;
        default:
            goto err;
            break;
    }

    goto out;

err:
    print_usage();
    rc = EXIT_FAILURE;
out:
    return rc;
}
