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
 * @brief: Support for DB3.0/3.1/3.2 boards
 *
 * @author: Jean Pihet
 */

#define ARADBG_COMP ARADBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio.h>
#include <nuttx/clock.h>

#include "nuttx/gpio/stm32_gpio_chip.h"
#include "nuttx/gpio/tca64xx.h"

#include <ara_debug.h>
#include "ara_board.h"
#include "interface.h"
#include "stm32.h"
#include "pwr_mon.h"

/* U4550 I/O Expander reset */
#define SVC_RST_IOEXP1_PIN  STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN14)
#define SVC_RST_IOEXP1      (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_PORTC | GPIO_PIN14)
/* U4570 I/O Expander reset */
#define SVC_RST_IOEXP2_PIN  STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN15)
#define SVC_RST_IOEXP2      (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                             GPIO_PORTC | GPIO_PIN15)

/* I/O Expanders IRQ to SVC */
#define IO_EXP_IRQ          STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN10)

/* I/O Expander: I2C bus and addresses */
#define IOEXP_I2C_BUS           1
#define IOEXP_U4550_I2C_ADDR    0x21
#define IOEXP_U4570_I2C_ADDR    0x20

/* 19.2MHz system reference clocks */
#define REFCLK_REQ        STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN9)
#define REFCLK_APB1_EN    U4570_GPIO_PIN(0) // On-board APB1
#define REFCLK_APB2_EN    U4570_GPIO_PIN(1) // On-board APB2
#define REFCLK_BUFFERS_EN U4570_GPIO_PIN(2) // Shared clock enable for modules
#define REFCLK_SW_EN      U4570_GPIO_PIN(3) // Switch
#define REFCLK_1_EN       U4570_GPIO_PIN(4) // Modules
#define REFCLK_2_EN       U4570_GPIO_PIN(5)
#define REFCLK_3_EN       U4570_GPIO_PIN(6)
#define REFCLK_4A_EN      U4570_GPIO_PIN(7)
#define REFCLK_4B_EN      U4570_GPIO_PIN(8)
#define REFCLK_5_EN       U4570_GPIO_PIN(9)
#define REFCLK_6_EN       U4570_GPIO_PIN(10)

/*
 * WAKE_DETECT lines directly connected to the SVC.
 * Configured by default as input floating without pull-up.
 */
#define WD_1_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define WD_3_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define WD_4A_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define WD_4B_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define WD_5_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define WD_6_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define WD8A_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define WD8B_DET_IN_GPIO     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)

#define WD_1_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN2)
#define WD_3_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN3)
#define WD_4A_DET    STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN4)
#define WD_4B_DET    STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN5)
#define WD_5_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN6)
#define WD_6_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN7)
#define WD8A_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN11)
#define WD8B_DET     STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN12)

/*
 * MOD_ACT_SW lines connected to SVC.
 *
 * Configured by default as input pullup, as there is no external
 * pullup.
 */
#define MOD_ACT_SW_1    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN0)
#define MOD_ACT_SW_2    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN2)
#define MOD_ACT_SW_3    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN3)
#define MOD_ACT_SW_4    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN4)
#define MOD_ACT_SW_5    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN6)
#define MOD_ACT_SW_6    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN7)
#define MOD_ACT_SW_7    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN8)

/* Module release pins */
#define MOD_RELEASE_1_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN2)
#define MOD_RELEASE_4_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_5_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_6_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN7)

#define MOD_RELEASE_1         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN2)
#define MOD_RELEASE_4         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_5         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_6         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN7)

#define ARA_KEY_CONFIG        (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA \
                               | GPIO_PIN0)
#define ARA_KEY               STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN0)

/* The on-board bridges vsys GPIOs are controlled by the AP. */

static struct vreg_data apb1_vsys_vreg_data[] = {
};

static struct vreg_data apb2_vsys_vreg_data[] = {
};

/* On-board bridges clock control */

static struct vreg_data apb1_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_APB1_EN),
};

static struct vreg_data apb2_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_APB2_EN),
};

/*
 * Modules voltage regulator list
 *
 * Notes:
 *
 * 1. The modules loadswitches are controlled by the INA231 ALERT output,
 * to be driven via I2C commands. Take a shortcut for bring-up until the
 * INA231 support is in: directly control it via the VSYS_ENx GPIO pins.
 * (ToDo)
 * 2. Module port 6 (Display and TS) is controlled by the AP. Interface
 * control pins to the SVC are present but not used:
 * WD_6, REFCLK_6_EN, VSYS_EN6.
 */
static struct vreg_data module_1_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(0), HOLD_TIME_MODULE),
};

static struct vreg_data module_2_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(2), HOLD_TIME_MODULE),
};

static struct vreg_data module_3_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(4), HOLD_TIME_MODULE),
};

static struct vreg_data module_4a_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(6), HOLD_TIME_MODULE),
};

static struct vreg_data module_4b_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(6), HOLD_TIME_MODULE),
};

static struct vreg_data module_5_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(U4550_GPIO_PIN(8), HOLD_TIME_MODULE),
};

/* Modules clock control list. */

static struct vreg_data module_1_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data module_2_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data module_3_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_3_EN),
};

static struct vreg_data module_4a_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data module_4b_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data module_5_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

/*
 * Interfaces on this board
 */
DECLARE_MODULE_PORT_INTERFACE(apb1, "apb1", apb1_vsys_vreg_data,
                              apb1_refclk_vreg_data, 3, WD8A_DET,
                              ARA_IFACE_WD_ACTIVE_HIGH, false, 0);
DECLARE_MODULE_PORT_INTERFACE(apb2, "apb2", apb2_vsys_vreg_data,
                              apb2_refclk_vreg_data, 1, WD8B_DET,
                              ARA_IFACE_WD_ACTIVE_HIGH, false, 0);
DECLARE_MODULE_PORT_INTERFACE(module_1, "module_1", module_1_vsys_vreg_data,
                              module_1_refclk_vreg_data, 13, WD_1_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE(module_2, "module_2", module_2_vsys_vreg_data,
                              module_2_refclk_vreg_data, 11, WD_2_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE(module_3, "module_3", module_3_vsys_vreg_data,
                              module_3_refclk_vreg_data, 4, WD_3_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3);
DECLARE_MODULE_PORT_INTERFACE(module_4a, "module_4a", module_4a_vsys_vreg_data,
                              module_4a_refclk_vreg_data, 8, WD_4A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4);
DECLARE_MODULE_PORT_INTERFACE(module_4b, "module_4b", module_4b_vsys_vreg_data,
                              module_4b_refclk_vreg_data, 6, WD_4B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4);
DECLARE_MODULE_PORT_INTERFACE(module_5, "module_5", module_5_vsys_vreg_data,
                              module_5_refclk_vreg_data, 10, WD_5_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_5);

#define I2C_SEL1_A      BIT(0)
#define I2C_SEL1_B      BIT(1)
#define I2C_SEL1_INH    BIT(2)

enum {
    I2C_INA230_SEL1_A = 0,
    I2C_INA230_SEL1_B,
    I2C_INA230_SEL1_INH,
};

/*
 * Power rail groups definitions.
 *
 * If a bit is set in i2c_sel, then drive the GPIO low.
 */
static struct pwrmon_dev_ctx db3_pwrmon_devs[] = {
    {
        .name = "SWitch",
        .i2c_sel = I2C_SEL1_A | I2C_SEL1_B | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VSW_1P1_PLL",        0x42),
            DEFINE_PWR_RAIL("VSW_1P1_CORE",       0x41),
            DEFINE_PWR_RAIL("VSW_1P8_UNIPRO",     0x47),
            DEFINE_PWR_RAIL("VSW_1P8_IO",         0x48),
        },
        .num_rails = 4,
    },
    {
        .name = "APB1",
        .i2c_sel = I2C_SEL1_B | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VAPB1_1P1_CORE",     0x41),
            DEFINE_PWR_RAIL("VAPB1_1P1_PLL1",     0x42),
            DEFINE_PWR_RAIL("VAPB1_1P2_CDSI_PLL", 0x4A),
            DEFINE_PWR_RAIL("VAPB1_1P2_CDSI",     0x4B),
            DEFINE_PWR_RAIL("VAPB1_1P2_HSIC",     0x46),
            DEFINE_PWR_RAIL("VAPB1_1P8_UNIPRO",   0x47),
            DEFINE_PWR_RAIL("VAPB1_1P8_IO",       0x48),
            DEFINE_PWR_RAIL("VAPB1_1P1_PLL2",     0x43),
        },
        .num_rails = 8,
    },
    {
        .name = "APB2",
        .i2c_sel = I2C_SEL1_A | I2C_SEL1_INH,
        .rails = {
            /*
             * NOTE:
             *
             * The APB2 1.2V CDSI and 1.1V PLL2 rails cannot be
             * measured on DB3.1, because they were accidentally on
             * the wrong I2C bus (they're on the bus for APB1 instead,
             * where they have an address conflict with the INA231s
             * associated with APB1's power rails).
             *
             * This was worked around by removing the following
             * commented-out power rails. If your board is different,
             * you can uncomment them.
             */
            DEFINE_PWR_RAIL("VAPB2_1P1_CORE",     0x41),
            DEFINE_PWR_RAIL("VAPB2_1P1_PLL1",     0x42),
            DEFINE_PWR_RAIL("VAPB2_1P2_CDSI_PLL", 0x4A),
            /* DEFINE_PWR_RAIL("VAPB2_1P2_CDSI",     0x4B), */
            DEFINE_PWR_RAIL("VAPB2_1P2_HSIC",     0x46),
            DEFINE_PWR_RAIL("VAPB2_1P8_UNIPRO",   0x47),
            DEFINE_PWR_RAIL("VAPB2_1P8_IO",       0x48),
            /* DEFINE_PWR_RAIL("VAPB2_1P1_PLL2",     0x43), */
        },
        .num_rails = 6,
    },
    {
        .name = "SVC",
        .i2c_sel = I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("SVC_1P8_VDD",        0x42),
            DEFINE_PWR_RAIL("SVC_1P8_VBAT",       0x41),
            DEFINE_PWR_RAIL("SVC_1P8_VDDA",       0x47),
        },
        .num_rails = 3,
    },
};

static struct pwrmon_dev_ctx db3_5_pwrmon_devs[] = {
    {
        .name = "SWitch",
        .i2c_sel = I2C_SEL1_A | I2C_SEL1_B | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VSW_1P1_PLL",        0x42),
            DEFINE_PWR_RAIL("VSW_1P1_CORE",       0x41),
            DEFINE_PWR_RAIL("VSW_1P8_UNIPRO",     0x47),
            DEFINE_PWR_RAIL("VSW_1P8_IO",         0x48),
        },
        .num_rails = 4,
    },
    {
        .name = "APB1",
        .i2c_sel = I2C_SEL1_B | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VAPB1_1P1_CORE",     0x41),
            DEFINE_PWR_RAIL("VAPB1_1P1_PLL1",     0x42),
            DEFINE_PWR_RAIL("VAPB1_1P2_CDSI_PLL", 0x4A),
            DEFINE_PWR_RAIL("VAPB1_1P2_CDSI",     0x4B),
            DEFINE_PWR_RAIL("VAPB1_1P2_HSIC",     0x46),
            DEFINE_PWR_RAIL("VAPB1_1P8_UNIPRO",   0x47),
            DEFINE_PWR_RAIL("VAPB1_1P8_IO",       0x48),
            DEFINE_PWR_RAIL("VAPB1_1P1_PLL2",     0x43),
        },
        .num_rails = 8,
    },
    {
        .name = "APB2",
        .i2c_sel = I2C_SEL1_A | I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("VAPB2_1P1_CORE",     0x41),
            DEFINE_PWR_RAIL("VAPB2_1P1_PLL1",     0x42),
            DEFINE_PWR_RAIL("VAPB2_1P2_CDSI_PLL", 0x4A),
            DEFINE_PWR_RAIL("VAPB2_1P2_CDSI",     0x4B),
            DEFINE_PWR_RAIL("VAPB2_1P2_HSIC",     0x46),
            DEFINE_PWR_RAIL("VAPB2_1P8_UNIPRO",   0x47),
            DEFINE_PWR_RAIL("VAPB2_1P8_IO",       0x48),
            DEFINE_PWR_RAIL("VAPB2_1P1_PLL2",     0x43),
        },
        .num_rails = 8,
    },
    {
        .name = "SVC",
        .i2c_sel = I2C_SEL1_INH,
        .rails = {
            DEFINE_PWR_RAIL("SVC_1P8_VDD",        0x42),
            DEFINE_PWR_RAIL("SVC_1P8_VBAT",       0x41),
            DEFINE_PWR_RAIL("SVC_1P8_VDDA",       0x47),
        },
        .num_rails = 3,
    },
};

static void db3_pwrmon_reset_i2c_sel(pwrmon_board_info* board)
{
    gpio_set_value(board->i2c_sel_gpio_base + I2C_INA230_SEL1_INH, 0);
    gpio_set_value(board->i2c_sel_gpio_base + I2C_INA230_SEL1_A, 0);
    gpio_set_value(board->i2c_sel_gpio_base + I2C_INA230_SEL1_B, 0);
}

static void db3_pwrmon_init_i2c_sel(pwrmon_board_info* board)
{
    gpio_direction_out(board->i2c_sel_gpio_base + I2C_INA230_SEL1_A, 0);
    gpio_direction_out(board->i2c_sel_gpio_base + I2C_INA230_SEL1_B, 0);
    gpio_direction_out(board->i2c_sel_gpio_base + I2C_INA230_SEL1_INH, 0);
}

static int db3_pwrmon_do_i2c_sel(pwrmon_board_info* board, uint8_t dev)
{
    if (dev >= board->num_devs) {
        return -EINVAL;
    }

    /* First inhibit all lines, to make sure there is no short/collision */
    gpio_set_value(I2C_INA230_SEL1_INH, 1);

    gpio_set_value(board->i2c_sel_gpio_base + I2C_INA230_SEL1_A,
                   board->devs[dev].i2c_sel & I2C_SEL1_A ? 0 : 1);
    gpio_set_value(board->i2c_sel_gpio_base + I2C_INA230_SEL1_B,
                   board->devs[dev].i2c_sel & I2C_SEL1_B ? 0 : 1);

    if (board->devs[dev].i2c_sel & I2C_SEL1_INH) {
        gpio_set_value(board->i2c_sel_gpio_base + I2C_INA230_SEL1_INH, 0);
    }

    return 0;
}

pwrmon_board_info db3_pwrmon = {
    .i2c_bus        = 1,
    .devs           = db3_pwrmon_devs,
    .num_devs       = ARRAY_SIZE(db3_pwrmon_devs),
    .reset_i2c_sel  = db3_pwrmon_reset_i2c_sel,
    .init_i2c_sel   = db3_pwrmon_init_i2c_sel,
    .do_i2c_sel     = db3_pwrmon_do_i2c_sel,
    .i2c_sel_gpio_base = STM32_GPIO_CHIP_BASE + 71, /* PE7 */
};

pwrmon_board_info db3_5_pwrmon = {
    .i2c_bus        = 1,
    .devs           = db3_5_pwrmon_devs,
    .num_devs       = ARRAY_SIZE(db3_5_pwrmon_devs),
    .reset_i2c_sel  = db3_pwrmon_reset_i2c_sel,
    .init_i2c_sel   = db3_pwrmon_init_i2c_sel,
    .do_i2c_sel     = db3_pwrmon_do_i2c_sel,
    .i2c_sel_gpio_base = U4570_GPIO_PIN(11),
};

/*
 * Important note: Always declare the spring interfaces last.
 * Assumed by Spring Power Measurement Library (up_spring_pm.c).
 */
static struct interface *db3_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &module_1_interface,
    &module_2_interface,
    &module_3_interface,
    &module_4a_interface,
    &module_4b_interface,
    &module_5_interface,
};

/*
 * Switch clock and power supplies
 */
static struct vreg_data sw_vreg_data[] = {
    /* 19.2MHz clock */
    INIT_MODULE_CLK_DATA(REFCLK_SW_EN),
    /* Switch 1P1 */
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN0),
                               HOLD_TIME_SW_1P1),
    /* Switch 1P8 */
    INIT_ACTIVE_HIGH_VREG_DATA(STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN1),
                               HOLD_TIME_SW_1P8),
};
DECLARE_VREG(sw_vreg, sw_vreg_data);

static struct io_expander_info db3_io_expanders[] = {
        {
            .part       = TCA6416_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U4550_I2C_ADDR,
            .reset      = SVC_RST_IOEXP1_PIN,
            .irq        = IO_EXP_IRQ,
            .gpio_base  = U4550_GPIO_CHIP_START,
        },
        {
            .part       = TCA6416_PART,
            .i2c_bus    = IOEXP_I2C_BUS,
            .i2c_addr   = IOEXP_U4570_I2C_ADDR,
            .reset      = SVC_RST_IOEXP2_PIN,
            .irq        = IO_EXP_IRQ,
            .gpio_base  = U4570_GPIO_CHIP_START,
        },
};

static int db3_board_init(struct ara_board_info *board_info) {
    /*
     * Turn on the global system clock and its buffered copy (which
     * goes to the modules and the switch).
     *
     * FIXME replace with real power management of clocks later.
     */
    gpio_activate(REFCLK_REQ);
    gpio_activate(REFCLK_BUFFERS_EN);
    gpio_direction_out(REFCLK_REQ, 1);
    gpio_direction_out(REFCLK_BUFFERS_EN, 1);

    /*
     * Configure the switch power supply lines.
     * Hold all the lines low while we turn on the power rails.
     */
    vreg_config(&sw_vreg);
    stm32_configgpio(db3_board_info.sw_data.gpio_reset);
    up_udelay(POWER_SWITCH_OFF_STAB_TIME_US);

    /*
     * Configure the SVC WAKE_DETECT pins
     */
    stm32_configgpio(WD_1_DET_IN_GPIO);
    stm32_configgpio(WD_2_DET_IN_GPIO);
    stm32_configgpio(WD_3_DET_IN_GPIO);
    stm32_configgpio(WD_4A_DET_IN_GPIO);
    stm32_configgpio(WD_4B_DET_IN_GPIO);
    stm32_configgpio(WD_5_DET_IN_GPIO);
    stm32_configgpio(WD_6_DET_IN_GPIO);
    stm32_configgpio(WD8A_DET_IN_GPIO);
    stm32_configgpio(WD8B_DET_IN_GPIO);

    /*
     * Configure the MOD_ACT_SW pins.
     */
    stm32_configgpio(MOD_ACT_SW_1);
    stm32_configgpio(MOD_ACT_SW_2);
    stm32_configgpio(MOD_ACT_SW_3);
    stm32_configgpio(MOD_ACT_SW_4);
    stm32_configgpio(MOD_ACT_SW_5);
    stm32_configgpio(MOD_ACT_SW_6);
    stm32_configgpio(MOD_ACT_SW_7);

    /*
     * Configure the module release pins
     */
    stm32_configgpio(MOD_RELEASE_1_CONFIG);
    stm32_configgpio(MOD_RELEASE_2_CONFIG);
    stm32_configgpio(MOD_RELEASE_3_CONFIG);
    stm32_configgpio(MOD_RELEASE_4_CONFIG);
    stm32_configgpio(MOD_RELEASE_5_CONFIG);
    stm32_configgpio(MOD_RELEASE_6_CONFIG);

    /* Configure ARA key input pin */
    stm32_configgpio(ARA_KEY_CONFIG);

    return 0;
}

struct ara_board_info db3_board_info = {
    .interfaces = db3_interfaces,
    .nr_interfaces = ARRAY_SIZE(db3_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg             = &sw_vreg,
        .gpio_reset       = (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
                             GPIO_PORTD | GPIO_PIN11),
        .gpio_irq         = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                             GPIO_PORTC | GPIO_PIN9),
        .irq_rising_edge  = true,
        .rev              = SWITCH_REV_ES2,
        .bus              = SW_SPI_PORT_2,
        .spi_cs           = (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | \
                             GPIO_PORTB | GPIO_PIN12)
    },

    .io_expanders         = db3_io_expanders,
    .nr_io_expanders      = ARRAY_SIZE(db3_io_expanders),

    .pwrmon               = &db3_pwrmon,

    .ara_key_gpio         = ARA_KEY,
    .ara_key_rising_edge  = true,
    .ara_key_configured   = true,

    .board_init           = db3_board_init,
};
