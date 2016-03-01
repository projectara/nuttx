/*
 * Copyright (c) 2015-2016 Google Inc.
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

#ifndef  _ARA_BOARD_H_
#define  _ARA_BOARD_H_

#include "nuttx/gpio/stm32_gpio_chip.h"
#include "nuttx/gpio/tca64xx.h"

#include "vreg.h"
#include "tsb_switch.h"
#include "pwr_mon.h"

/* HWID values */
enum hwid {
    HWID_DB3_0_1,           /* DB3.0 and 3.1 (ES2) */
    HWID_DB3_2,             /* DB3.2 = ES3 DB3.1 */
    HWID_DB3_5,             /* DB3.5 (ES3) */
    HWID_EVT1,              /* EVT1 (ES2) */
    HWID_EVT1_5,            /* EVT1.5 (ES3) */
    HWID_EVT1_6,            /* EVT1.6 (ES2) */
};

/* HWID pins */
#define HWID_0              (GPIO_INPUT | GPIO_PORTB | GPIO_PIN5)
#define HWID_1              (GPIO_INPUT | GPIO_PORTB | GPIO_PIN6)
#define HWID_2              (GPIO_INPUT | GPIO_PORTD | GPIO_PIN12)
#define HWID_3              (GPIO_INPUT | GPIO_PORTD | GPIO_PIN13)

/*
 * GPIO Chip pins
 *
 * STM32 GPIO starts at STM32_GPIO_CHIP_BASE
 */
#define STM32_GPIO_CHIP_NR      140

/* DB3 and EVT1 */
#define U4550_GPIO_CHIP_START   (STM32_GPIO_CHIP_BASE + STM32_GPIO_CHIP_NR)
#define U4550_GPIO_CHIP_NR      16
#define U4570_GPIO_CHIP_START   (U4550_GPIO_CHIP_START + U4550_GPIO_CHIP_NR)
#define U4570_GPIO_CHIP_NR      16

/* GPIO Chip pin number macro */
#define STM32_GPIO_PIN(p)       (STM32_GPIO_CHIP_BASE + (p))
#define U4550_GPIO_PIN(p)       (U4550_GPIO_CHIP_START + (p))
#define U4570_GPIO_PIN(p)       (U4570_GPIO_CHIP_START + (p))

/* ES2-specific repurposing of attributes */
#define ES2_MBOX_ACK_ATTR       T_TSTSRCINTERMESSAGEGAP
#define MBOX_ACK_ATTR           ES2_MBOX_ACK_ATTR
#define ES2_DME_INIT_STATUS     T_TSTSRCINCREMENT

/*
 * Common timing parameters
 */

/* Module power on hold time (10 ms). */
#define HOLD_TIME_MODULE                (10000)
/* Switch 1.1V power on hold time (50 ms). */
#define HOLD_TIME_SW_1P1                (50000)
/* Switch 1.8V power on hold time (10 ms). */
#define HOLD_TIME_SW_1P8                (10000)
/* Stabilization time after configuring switch vregs at init (10 ms). */
#define POWER_SWITCH_OFF_STAB_TIME_US   (10000)

struct io_expander_info {
    void *io_exp_driver_data;
    uint32_t reset;
    uint32_t irq;
    int gpio_base;
    struct i2c_dev_s *i2c_dev;
    uint8_t i2c_bus;
    uint8_t i2c_addr;
    tca64xx_part part;
};

struct ara_board_info {
    /* Interfaces */
    struct interface **interfaces;
    size_t nr_interfaces;
    size_t nr_spring_interfaces;

    /* Switch data */
    struct tsb_switch_data sw_data;

    /* IO Expanders data */
    struct io_expander_info *io_expanders;
    size_t nr_io_expanders;

    /* Power monitoring */
    pwrmon_board_info *pwrmon;

    /*  Board specific init and exit code */
    int (*board_init)(struct ara_board_info *);
    int (*board_exit)(struct ara_board_info *);

    /* ara key GPIO */
    uint8_t ara_key_gpio;
    bool ara_key_rising_edge;
    bool ara_key_configured;

    /* latch vregs */
    struct vreg *vlatch_vdd;
    struct vreg *latch_ilim;
};

struct ara_board_info *ara_board_init(void);
void ara_board_exit(void);
pwrmon_board_info *board_get_pwrmon_info(void);

 /*
  * Board specific information, to be passed from the board files
  */
extern struct ara_board_info db3_board_info;
extern pwrmon_board_info db3_pwrmon;
extern pwrmon_board_info db3_5_pwrmon;
extern struct interface *db3_5_interfaces[];
extern size_t db3_5_nr_interfaces;
extern struct ara_board_info evt1_board_info;
extern struct ara_board_info evt1_5_board_info;

#endif
