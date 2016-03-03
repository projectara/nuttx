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

/*
 * - TODO INA231s / arapm support
 * - TODO UFS module port
 */

#define ARADBG_COMP ARADBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/i2c.h>
#include <nuttx/gpio.h>
#include <nuttx/gpio/ara_cpld.h>

#include "nuttx/gpio/stm32_gpio_chip.h"
#include "nuttx/gpio/tca64xx.h"

#include <ara_debug.h>
#include "ara_board.h"
#include "interface.h"
#include "stm32.h"

#define HOLD_TIME_SW_CLK_US     10000

/*
 * I/O expander config. U4550 is IOEXP1, U4570 is IOEXP2. (These
 * defines match schematic net names.)
 */
#define IOEXP_I2C_BUS           1
#define IOEXP_U4430_I2C_ADDR    0x0a
#define IOEXP_U4550_I2C_ADDR    0x21
#define IOEXP_U4570_I2C_ADDR    0x20
#define IOEXP_U4430_RESET       U4550_GPIO_PIN(14)
#define IOEXP1_INT_N            STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN9)
#define EVT2_IOEXP1_INT_N       STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN10)
#define IOEXP2_INT_N            STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN10)
#define SVC_RST_IOEXP1          STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN14)
#define EVT2_SVC_RST_IOEXP1     STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN3)
#define SVC_RST_IOEXP2          STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN15)
#define SVC_RST_IOEXP1_GPIO     (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                                 GPIO_PORTC | GPIO_PIN14)
#define SVC_RST_IOEXP2_GPIO     (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP | \
                                 GPIO_PORTC | GPIO_PIN15)

/* 19.2 MHz system reference clocks */
#define REFCLK_REQ        STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN9)
#define REFCLK_BUFFERS_EN U4570_GPIO_PIN(2) /* Shared clock enable for modules */
#define REFCLK_SW_EN      U4570_GPIO_PIN(3) /* Switch */
#define REFCLK_1_EN       U4570_GPIO_PIN(4) /* Modules */
#define REFCLK_2_EN       U4570_GPIO_PIN(5)
#define REFCLK_3A_EN      U4570_GPIO_PIN(6)
#define REFCLK_3B_EN      U4570_GPIO_PIN(7)
#define REFCLK_4A_EN      U4570_GPIO_PIN(8)
#define REFCLK_4B_EN      U4570_GPIO_PIN(9)
#define REFCLK_5_EN       U4570_GPIO_PIN(10)

#define EVT2_REFCLK_1_EN       U4430_GPIO_PIN(0) /* Modules */
#define EVT2_REFCLK_2_EN       U4430_GPIO_PIN(1)
#define EVT2_REFCLK_3A_EN      U4430_GPIO_PIN(2)
#define EVT2_REFCLK_3B_EN      U4430_GPIO_PIN(3)
#define EVT2_REFCLK_4A_EN      U4430_GPIO_PIN(4)
#define EVT2_REFCLK_4B_EN      U4430_GPIO_PIN(5)
#define EVT2_REFCLK_APB1_EN    U4430_GPIO_PIN(6) /* Built-in bridges */
#define EVT2_REFCLK_APB2_EN    U4430_GPIO_PIN(7)
#define EVT2_REFCLK_SVC_EN     U4430_GPIO_PIN(8) /* SVC */
#define EVT2_REFCLK_SW_EN      U4430_GPIO_PIN(9) /* Switch */

/* Wake/detect pins */
#define WD_1_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define WD_3A_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define WD_3B_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define WD_4A_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define WD_4B_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
/*  LCD */
#define WD_5_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define WD_8A_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define WD_8B_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)

#define WD_1_DET    STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET    STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN2)
#define WD_3A_DET   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN3)
#define WD_3B_DET   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN4)
#define WD_4A_DET   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN5)
#define WD_4B_DET   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN6)
#define WD_5_DET    STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN7) /* LCD */
#define WD_8A_DET   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN11)
#define WD_8B_DET   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN12)

/*
 * Latch pins
 */
#define LATCH_1_DET    STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN0)
#define LATCH_2_DET    STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN13)
#define LATCH_3A_DET   STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN14)
#define LATCH_3B_DET   STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN15)
#define LATCH_4A_DET   STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN8)
#define LATCH_4B_DET   STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN7)

#define LATCH_1_DET_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN0)
#define LATCH_2_DET_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13)
#define LATCH_3A_DET_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN14)
#define LATCH_3B_DET_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN15)
#define LATCH_4A_DET_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN8)
#define LATCH_4B_DET_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN7)

#define MOD_SENSE           STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN2)
#define VREG_3V_ALWAYS_ON   STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN0)
#define LATCH_ILIM_EN       STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN7)
#define LATCH_VDD_EN        STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN2)

/* Module release pins */
#define MOD_RELEASE_1_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3A_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN2)
#define EVT2_MOD_RELEASE_3A_CONFIG (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define MOD_RELEASE_3B_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_4A_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_4B_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN7)
#define MOD_RELEASE_5_CONFIG   (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN7)

#define MOD_RELEASE_1       STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2       STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3A      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN2)
#define EVT2_MOD_RELEASE_3A STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN7)
#define MOD_RELEASE_3B      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_4A      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_4B      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN7)
#define MOD_RELEASE_5       STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN7)

/* UART RX line from MSM. Used to wake-up the SVC on incoming traffic */
#define UART_MSM_TX_SVC_RX  STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN13)

/* Ara key */
#define ARA_KEY_CONFIG      (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA \
                             | GPIO_PIN0)
#define ARA_KEY             STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN0)

/* Switch control pins */
#define SW_1P1_EN          STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN0)
#define SW_1P8_IO_EN       STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN1)
#define SW_1P8_UNIPRO_EN   STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN8)
#define SW_STANDBY_N       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | \
                            GPIO_PORTA | GPIO_PIN8)
#define SVC_RST_SW         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN11)
#define SVC_RST_SW_GPIO    (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTD | GPIO_PIN11)
#define SW_UNIPRO_1P8_PWM  STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN2)
#define SW_IO_1P8_PWM      STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN4)
#define SW_TO_SVC_INT      STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN9)
#define SW_TO_SVC_INT_GPIO (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | \
                            GPIO_PORTC | GPIO_PIN9)
#define SVC_SW_SPI_CS_GPIO (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | \
                            GPIO_PORTB | GPIO_PIN12)

/* VSYS enable */
#define VSYS_EN_PULL       U4550_GPIO_PIN(12)
#define VSYS_EN1_N         U4550_GPIO_PIN(0)
#define VSYS_EN2_N         U4550_GPIO_PIN(2)
#define VSYS_EN3A_N        U4550_GPIO_PIN(4)
#define VSYS_EN3B_N        U4550_GPIO_PIN(6)
#define VSYS_EN4A_N        U4550_GPIO_PIN(8)
#define VSYS_EN4B_N        U4550_GPIO_PIN(10)
#define VSYS_EN5_N         U4550_GPIO_PIN(12)

/* VCHG enable */
#define VCHG_EN1_N         U4550_GPIO_PIN(1)
#define VCHG_EN2_N         U4550_GPIO_PIN(3)
#define VCHG_EN3A_N        U4550_GPIO_PIN(5)
#define VCHG_EN3B_N        U4550_GPIO_PIN(7)
#define VCHG_EN4A_N        U4550_GPIO_PIN(9)
#define VCHG_EN4B_N        U4550_GPIO_PIN(11)
#define VCHG_EN5_N         U4550_GPIO_PIN(13)

/*
 * SVC to MSM Wake from Off pin: Active low, pulled up high internally by
 * the PMIC.
 * Not used for now, let it in default state of the I/O Expander (input).
 */
#define PM_CBL_PWR_N_GPIO  U4550_GPIO_PIN(15)

/*
 * APBridges regulator list and interface declarations
 *
 * TODO: clean up APB during bringup
 *
 * Unlike on other boards and other interfaces on this board, there's
 * nothing to do regulator or clock-wise for the APBridges. The AP
 * controls their power and clock enable lines, and should be able to
 * ensure REFCLK_MAIN is supplied when necessary. So no vreg_data is
 * needed here.
 *
 * However, to be safe for pre-bringup, we:
 *
 * - turn on REFCLK_MAIN unconditionally by asserting REFCLK_REQ at
 *   init time to ensure APB clocks are enabled, and
 *
 * - declare these empty vreg_data arrays to avoid possible crashes
 *   due to having a null vreg_data in an ARA_IFACE_TYPE_MODULE_PORT
 *   interface crop up at an inconvenient time.
 *
 * This should get cleaned up (modules including the REFCLK_REQ in
 * their vreg_data, no separate refclk_main_vreg_data, and NULL APB
 * vreg_data) during bringup when we can test on real HW.
 *
 * Only APB1 is in use, and WD8B is reserved for time sync.
 *
 ** ----------------------------------------------------------------------
 *
 * HOWEVER, we still rely on APB2 during the QA and factory line
 * testing process, so DO NOT DELETE IT.
 *
 * ----------------------------------------------------------------------
 */

/*
 * EVT1 built-in bridges
 */
static struct vreg_data apb1_vsys_vreg_data[] = {
};

static struct vreg_data apb2_vsys_vreg_data[] = {
};

static struct vreg_data apb1_refclk_vreg_data[] = {
};

static struct vreg_data apb2_refclk_vreg_data[] = {
};

DECLARE_MODULE_PORT_INTERFACE(apb1, INTF_APB1, apb1_vsys_vreg_data,
                              apb1_refclk_vreg_data, 3, WD_8A_DET,
                              ARA_IFACE_WD_ACTIVE_HIGH, false, 0);
DECLARE_MODULE_PORT_INTERFACE(apb2, INTF_APB2, apb2_vsys_vreg_data,
                              apb2_refclk_vreg_data, 1, WD_8B_DET,
                              ARA_IFACE_WD_ACTIVE_HIGH, false, 0);

/*
 * EVT2 built-in bridges
 */
static struct vreg_data evt2_apb1_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_APB1_EN),
};

static struct vreg_data evt2_apb2_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_APB2_EN),
};

DECLARE_MODULE_PORT_INTERFACE(evt2_apb1, INTF_APB1, apb1_vsys_vreg_data,
                              evt2_apb1_refclk_vreg_data, 3, WD_8A_DET,
                              ARA_IFACE_WD_ACTIVE_HIGH, false, 0);
DECLARE_MODULE_PORT_INTERFACE(evt2_apb2, INTF_APB2, apb2_vsys_vreg_data,
                              evt2_apb2_refclk_vreg_data, 1, WD_8B_DET,
                              ARA_IFACE_WD_ACTIVE_HIGH, false, 0);

/*
 * Modules voltage regulator list and interface declarations.
 *
 * As on DB3, the modules' load switches are controlled by the INA231
 * ALERT output, which should eventually driven via I2C commands to
 * e.g. disable power on overcurrent.
 *
 * Take a shortcut until the relevant INA231-related support is in:
 * directly control it via the VSYS_ENx GPIO pins.
 */

/*
 * EVT1 interface ports
 */
static struct vreg_data evt1_module_1_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN1_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_2_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN2_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_3A_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN3A_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_3B_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN3B_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_4A_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN4A_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_4B_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN4B_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_5_lcd_vsys_vreg_data[] = {
    INIT_ACTIVE_LOW_VREG_DATA(VSYS_EN5_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_module_1_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data evt1_module_2_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data evt1_module_3A_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_3A_EN),
};

static struct vreg_data evt1_module_3B_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_3B_EN),
};

static struct vreg_data evt1_module_4A_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data evt1_module_4B_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data evt1_module_5_lcd_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

DECLARE_MODULE_PORT_INTERFACE(evt1_module_1, "module_1",
                              evt1_module_1_vsys_vreg_data,
                              evt1_module_1_refclk_vreg_data, 13, WD_1_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE(evt1_module_2, "module_2",
                              evt1_module_2_vsys_vreg_data,
                              evt1_module_2_refclk_vreg_data, 11, WD_2_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE(evt1_module_3A, "module_3a",
                              evt1_module_3A_vsys_vreg_data,
                              evt1_module_3A_refclk_vreg_data, 4, WD_3A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3A);
DECLARE_MODULE_PORT_INTERFACE(evt1_module_3B, "module_3b",
                              evt1_module_3B_vsys_vreg_data,
                              evt1_module_3B_refclk_vreg_data, 2, WD_3B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE(evt1_module_4A, "module_4a",
                              evt1_module_4A_vsys_vreg_data,
                              evt1_module_4A_refclk_vreg_data, 6, WD_4A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4A);
DECLARE_MODULE_PORT_INTERFACE(evt1_module_4B, "module_4b",
                              evt1_module_4B_vsys_vreg_data,
                              evt1_module_4B_refclk_vreg_data, 8, WD_4B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4B);
DECLARE_MODULE_PORT_INTERFACE(evt1_module_5_lcd, "module_5",
                              evt1_module_5_lcd_vsys_vreg_data,
                              evt1_module_5_lcd_refclk_vreg_data, 10, WD_5_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, false, MOD_RELEASE_5);

/*
 * Power rail groups definitions.
 */
static struct pwrmon_dev_ctx evt1_pwrmon_devs[] = {
    {
        .name = "Port 1",
        .rails = {
            DEFINE_PWR_RAIL("MOD_PORT1_VSYS",       0x40),
        },
        .num_rails = 1,
    },
    {
        .name = "Port 2",
        .rails = {
            DEFINE_PWR_RAIL("MOD_PORT2_VSYS",       0x41),
        },
        .num_rails = 1,
    },
    {
        .name = "Port 3A",
        .rails = {
            DEFINE_PWR_RAIL("MOD_PORT3A_VSYS",      0x42),
        },
        .num_rails = 1,
    },
    {
        .name = "Port 3B",
        .rails = {
            DEFINE_PWR_RAIL("MOD_PORT3B_VSYS",      0x43),
        },
        .num_rails = 1,
    },
    {
        .name = "Port 4A",
        .rails = {
            DEFINE_PWR_RAIL("MOD_PORT4A_VSYS",      0x44),
        },
        .num_rails = 1,
    },
    {
        .name = "Port 4B",
        .rails = {
            DEFINE_PWR_RAIL("MOD_PORT4B_VSYS",      0x45),
        },
        .num_rails = 1,
    },
    {
        .name = "Display",
        .rails = {
            DEFINE_PWR_RAIL("DISPLAY_VSYS",         0x46),
        },
        .num_rails = 1,
    },
};

static void evt1_pwrmon_reset_i2c_sel(pwrmon_board_info* board)
{
}

static void evt1_pwrmon_init_i2c_sel(pwrmon_board_info* board)
{
}

static int evt1_pwrmon_do_i2c_sel(pwrmon_board_info* board, uint8_t dev)
{
    return 0;
}

pwrmon_board_info evt1_pwrmon = {
        .i2c_bus        = 2,
        .devs           = evt1_pwrmon_devs,
        .num_devs       = ARRAY_SIZE(evt1_pwrmon_devs),
        .reset_i2c_sel  = evt1_pwrmon_reset_i2c_sel,
        .init_i2c_sel   = evt1_pwrmon_init_i2c_sel,
        .do_i2c_sel     = evt1_pwrmon_do_i2c_sel,
};

static struct interface *evt1_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &evt1_module_1_interface,
    &evt1_module_2_interface,
    &evt1_module_3A_interface,
    &evt1_module_3B_interface,
    &evt1_module_4A_interface,
    &evt1_module_4B_interface,
    &evt1_module_5_lcd_interface,
};

/*
 * EVT1.5 interface ports
 */
static struct vreg_data evt1_5_module_1_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN1_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_2_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN2_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_3A_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN3A_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_3B_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN3B_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_4A_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN4A_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_4B_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN4B_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_5_lcd_vsys_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN5_N, HOLD_TIME_MODULE),
};

static struct vreg_data evt1_5_module_1_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data evt1_5_module_2_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data evt1_5_module_3A_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_3A_EN),
};

static struct vreg_data evt1_5_module_3B_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_3B_EN),
};

static struct vreg_data evt1_5_module_4A_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data evt1_5_module_4B_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data evt1_5_module_5_lcd_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_1, "module_1",
                              evt1_5_module_1_vsys_vreg_data,
                              evt1_5_module_1_refclk_vreg_data, 13, WD_1_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_2, "module_2",
                              evt1_5_module_2_vsys_vreg_data,
                              evt1_5_module_2_refclk_vreg_data, 11, WD_2_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_3A, "module_3a",
                              evt1_5_module_3A_vsys_vreg_data,
                              evt1_5_module_3A_refclk_vreg_data, 4, WD_3A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3A);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_3B, "module_3b",
                              evt1_5_module_3B_vsys_vreg_data,
                              evt1_5_module_3B_refclk_vreg_data, 2, WD_3B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_4A, "module_4a",
                              evt1_5_module_4A_vsys_vreg_data,
                              evt1_5_module_4A_refclk_vreg_data, 6, WD_4A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4A);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_4B, "module_4b",
                              evt1_5_module_4B_vsys_vreg_data,
                              evt1_5_module_4B_refclk_vreg_data, 8, WD_4B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4B);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_5_lcd, "module_5",
                              evt1_5_module_5_lcd_vsys_vreg_data,
                              evt1_5_module_5_lcd_refclk_vreg_data, 10, WD_5_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_5);

static struct interface *evt1_5_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &evt1_5_module_1_interface,
    &evt1_5_module_2_interface,
    &evt1_5_module_3A_interface,
    &evt1_5_module_3B_interface,
    &evt1_5_module_4A_interface,
    &evt1_5_module_4B_interface,
    &evt1_5_module_5_lcd_interface,
};

/*
 * EVT2 interface ports
 */
static struct vreg_data evt2_module_1_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_1_EN),
};

static struct vreg_data evt2_module_2_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_2_EN),
};

static struct vreg_data evt2_module_3A_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_3A_EN),
};

static struct vreg_data evt2_module_3B_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_3B_EN),
};

static struct vreg_data evt2_module_4A_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_4A_EN),
};

static struct vreg_data evt2_module_4B_refclk_vreg_data[] = {
    INIT_MODULE_CLK_DATA(EVT2_REFCLK_4B_EN),
};

#if defined(CONFIG_ARA_SVC_EVT1_5_MODULE_COMPATIBILITY)
DECLARE_MODULE_PORT_INTERFACE(evt2_module_1, "module_1",
                              evt1_5_module_1_vsys_vreg_data,
                              evt2_module_1_refclk_vreg_data, 13, WD_1_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE(evt2_module_2, "module_2",
                              evt1_5_module_2_vsys_vreg_data,
                              evt2_module_2_refclk_vreg_data, 11, WD_2_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE(evt2_module_3A, "module_3a",
                              evt1_5_module_3A_vsys_vreg_data,
                              evt2_module_3A_refclk_vreg_data, 4, WD_3A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true,
                              EVT2_MOD_RELEASE_3A);
DECLARE_MODULE_PORT_INTERFACE(evt2_module_3B, "module_3b",
                              evt1_5_module_3B_vsys_vreg_data,
                              evt2_module_3B_refclk_vreg_data, 2, WD_3B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE(evt2_module_4A, "module_4a",
                              evt1_5_module_4A_vsys_vreg_data,
                              evt2_module_4A_refclk_vreg_data, 6, WD_4A_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4A);
DECLARE_MODULE_PORT_INTERFACE(evt2_module_4B, "module_4b",
                              evt1_5_module_4B_vsys_vreg_data,
                              evt2_module_4B_refclk_vreg_data, 8, WD_4B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4B);
#else
DECLARE_MODULE_PORT_INTERFACE2(evt2_module_1, "module_1",
                               evt1_5_module_1_vsys_vreg_data,
                               evt2_module_1_refclk_vreg_data, 13, WD_1_DET,
                               ARA_IFACE_WD_ACTIVE_HIGH, LATCH_1_DET,
                               ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE2(evt2_module_2, "module_2",
                               evt1_5_module_2_vsys_vreg_data,
                               evt2_module_2_refclk_vreg_data, 11, WD_2_DET,
                               ARA_IFACE_WD_ACTIVE_HIGH, LATCH_2_DET,
                               ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE2(evt2_module_3A, "module_3a",
                               evt1_5_module_3A_vsys_vreg_data,
                               evt2_module_3A_refclk_vreg_data, 4, WD_3A_DET,
                               ARA_IFACE_WD_ACTIVE_HIGH, LATCH_3A_DET,
                               ARA_IFACE_WD_ACTIVE_LOW, true,
                               EVT2_MOD_RELEASE_3A);
DECLARE_MODULE_PORT_INTERFACE2(evt2_module_3B, "module_3b",
                               evt1_5_module_3B_vsys_vreg_data,
                               evt2_module_3B_refclk_vreg_data, 2, WD_3B_DET,
                               ARA_IFACE_WD_ACTIVE_HIGH, LATCH_3B_DET,
                               ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE2(evt2_module_4A, "module_4a",
                               evt1_5_module_4A_vsys_vreg_data,
                               evt2_module_4A_refclk_vreg_data, 6, WD_4A_DET,
                               ARA_IFACE_WD_ACTIVE_HIGH, LATCH_4A_DET,
                               ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4A);
DECLARE_MODULE_PORT_INTERFACE2(evt2_module_4B, "module_4b",
                               evt1_5_module_4B_vsys_vreg_data,
                               evt2_module_4B_refclk_vreg_data, 8, WD_4B_DET,
                               ARA_IFACE_WD_ACTIVE_HIGH, LATCH_4B_DET,
                               ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_4B);
#endif

static struct interface *evt2_interfaces[] = {
    &evt2_apb1_interface,
    &evt2_apb2_interface,
    &evt2_module_1_interface,
    &evt2_module_2_interface,
    &evt2_module_3A_interface,
    &evt2_module_3B_interface,
    &evt2_module_4A_interface,
    &evt2_module_4B_interface,
};

static struct vreg_data latch_ilim_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(LATCH_ILIM_EN, 0),
};
DECLARE_VREG(latch_ilim_vreg, latch_ilim_vreg_data);

static struct vreg_data vlatch_vdd_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(LATCH_VDD_EN, 0),
};
DECLARE_VREG(vlatch_vdd_vreg, vlatch_vdd_vreg_data);

static struct vreg_data vreg_3v_always_on_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VREG_3V_ALWAYS_ON, 0),
};
DECLARE_VREG(vreg_3v_always_on, vreg_3v_always_on_data);

/*
 * DB3.5 interface ports
 * Similar to EVT1.5 interfaces, with different mapping of the physical ports
 */
DECLARE_MODULE_PORT_INTERFACE(db3_5_module_3B, "module_3b",
                              evt1_5_module_3B_vsys_vreg_data,
                              evt1_5_module_3B_refclk_vreg_data, 10, WD_3B_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE(db3_5_module_5_lcd, "module_5",
                              evt1_5_module_5_lcd_vsys_vreg_data,
                              evt1_5_module_5_lcd_refclk_vreg_data, 9, WD_5_DET,
                              ARA_IFACE_WD_ACTIVE_LOW, true, MOD_RELEASE_5);

/* Interfaces on DB3.5 */
struct interface *db3_5_interfaces[] = {
    &apb1_interface,
    &apb2_interface,
    &evt1_5_module_1_interface,
    &evt1_5_module_2_interface,
    &evt1_5_module_3A_interface,
    &db3_5_module_3B_interface,
    &evt1_5_module_4A_interface,
    &evt1_5_module_4B_interface,
    &db3_5_module_5_lcd_interface,
};
size_t db3_5_nr_interfaces = ARRAY_SIZE(db3_5_interfaces);

/*
 * UniPro Switch vreg
 */
static struct vreg_data sw_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(SW_1P1_EN, HOLD_TIME_SW_1P1),

    /*
     * HACK: put the 1.8V power supplies into PWM mode always.
     *
     * TODO [SW-1934] investigate methods for allowing supply to
     *      alternate between PWM/PFM modes automatically without
     *      causing voltage droops observed in DB3 bringup.
     */
    INIT_ACTIVE_HIGH_VREG_DATA(SW_UNIPRO_1P8_PWM, 0),
    INIT_ACTIVE_HIGH_VREG_DATA(SW_IO_1P8_PWM, 0),
    /* END HACK */

    INIT_ACTIVE_HIGH_VREG_DATA(SW_1P8_IO_EN, 0),
    INIT_ACTIVE_HIGH_VREG_DATA(SW_1P8_UNIPRO_EN, HOLD_TIME_SW_1P8),
    INIT_ACTIVE_HIGH_VREG_DATA(REFCLK_SW_EN, HOLD_TIME_SW_CLK_US),
};
DECLARE_VREG(sw_vreg, sw_vreg_data);

/*
 * I/O expanders
 */

static struct io_expander_info evt1_io_expanders[] = {
    {
        .part      = TCA6416_PART,
        .i2c_bus   = IOEXP_I2C_BUS,
        .i2c_addr  = IOEXP_U4550_I2C_ADDR,
        .reset     = SVC_RST_IOEXP1,
        /*
         * TODO [SW-1936] U4550 IRQs are needed when the VSYS
         *      etc. pins are inputs and the INA231 controls
         *      VSYS. This is so the ALERT pin can be used as an IRQ
         *      source on overcurrent.
         */
        .irq       = TCA64XX_IO_UNUSED,
        .gpio_base = U4550_GPIO_CHIP_START,
    },
    {
        .part      = TCA6416_PART,
        .i2c_bus   = IOEXP_I2C_BUS,
        .i2c_addr  = IOEXP_U4570_I2C_ADDR,
        .reset     = SVC_RST_IOEXP2,
        .irq       = TCA64XX_IO_UNUSED,
        .gpio_base = U4570_GPIO_CHIP_START,
    },
};

static struct io_expander_info evt2_io_expanders[] = {
    {
        .part      = TCA6416_PART,
        .i2c_bus   = IOEXP_I2C_BUS,
        .i2c_addr  = IOEXP_U4550_I2C_ADDR,
        .reset     = EVT2_SVC_RST_IOEXP1,
        .irq       = TCA64XX_IO_UNUSED,
        .gpio_base = U4550_GPIO_CHIP_START,
    },
};

static struct ara_cpld_pdata evt2_cpld = {
    .base = U4430_GPIO_CHIP_START,
    .i2c_bus = IOEXP_I2C_BUS,
    .i2c_addr = IOEXP_U4430_I2C_ADDR,
    .reset = IOEXP_U4430_RESET,
};

/*
 * Board info, init, and exit
 */
static struct vreg_data refclk_main_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_REQ),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
};

DECLARE_VREG(refclk_main_vreg, refclk_main_vreg_data);

static struct vreg_data evt2_refclk_main_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_REQ),
};
DECLARE_VREG(evt2_refclk_main_vreg, evt2_refclk_main_vreg_data);

static int uart_msm_rx_gpio_handler(int irq, void *context)
{
    return OK;
}

/* EVT1 */
static int evt1_board_init(struct ara_board_info *board_info) {
    int rc;

    /* For now, just always enable REFCLK_MAIN and the buffers. */
    rc = vreg_config(&refclk_main_vreg) || vreg_get(&refclk_main_vreg);
    if (rc) {
        dbg_error("%s: can't start REFCLK_MAIN: %d\n", __func__, rc);
        return ERROR;
    }

    /* Configure the switch power supply lines. */
    rc = vreg_config(&sw_vreg);
    if (rc) {
        dbg_error("%s: can't configure switch regulators: %d\n", __func__, rc);
        return ERROR;
    }
    stm32_configgpio(evt1_board_info.sw_data.gpio_reset);
    up_udelay(POWER_SWITCH_OFF_STAB_TIME_US);

    /* Configure the wake/detect lines. */
    stm32_configgpio(WD_1_DET_IN_GPIO);
    stm32_configgpio(WD_2_DET_IN_GPIO);
    stm32_configgpio(WD_3A_DET_IN_GPIO);
    stm32_configgpio(WD_3B_DET_IN_GPIO);
    stm32_configgpio(WD_4A_DET_IN_GPIO);
    stm32_configgpio(WD_4B_DET_IN_GPIO);
    stm32_configgpio(WD_5_DET_IN_GPIO);
    stm32_configgpio(WD_8A_DET_IN_GPIO);
    stm32_configgpio(WD_8B_DET_IN_GPIO);

    /* Configure the module release pins */
    stm32_configgpio(MOD_RELEASE_1_CONFIG);
    stm32_configgpio(MOD_RELEASE_2_CONFIG);
    stm32_configgpio(MOD_RELEASE_3A_CONFIG);
    stm32_configgpio(MOD_RELEASE_3B_CONFIG);
    stm32_configgpio(MOD_RELEASE_4A_CONFIG);
    stm32_configgpio(MOD_RELEASE_4B_CONFIG);
    stm32_configgpio(MOD_RELEASE_5_CONFIG);

    /* Configure ARA key input pin */
    stm32_configgpio(ARA_KEY_CONFIG);

    /*
     * Install IRQ for the MSM UART RX line. Used to wake-up the SVC
     * from low power idle
     */
    gpio_direction_in(UART_MSM_TX_SVC_RX);
    if (gpio_irq_settriggering(UART_MSM_TX_SVC_RX, IRQ_TYPE_EDGE_BOTH) ||
        gpio_irq_attach(UART_MSM_TX_SVC_RX, uart_msm_rx_gpio_handler) ||
        gpio_irq_unmask(UART_MSM_TX_SVC_RX)) {
        dbg_error("%s: cannot install wake-up handler for MSM UART RX\n",
                  __func__);
    }

    /*
     * (Module hotplug pins unconfigured. TODO, part of SW-1942.)
     */

    return 0;
}

struct ara_board_info evt1_board_info = {
    .interfaces = evt1_interfaces,
    .nr_interfaces = ARRAY_SIZE(evt1_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg            = &sw_vreg,
        .gpio_reset      = SVC_RST_SW_GPIO,
        .gpio_irq        = SW_TO_SVC_INT_GPIO,
        .irq_rising_edge = false,
        .rev             = SWITCH_REV_ES2,
        .bus             = SW_SPI_PORT_2,
        .spi_cs          = SVC_SW_SPI_CS_GPIO,
    },

    .io_expanders = evt1_io_expanders,
    .nr_io_expanders = ARRAY_SIZE(evt1_io_expanders),

    .pwrmon               = &evt1_pwrmon,

    .board_init           = evt1_board_init,

    .ara_key_gpio         = ARA_KEY,
    .ara_key_rising_edge  = true,
    .ara_key_configured   = true,
};

/* EVT1.5 */
static int evt1_5_board_init(struct ara_board_info *board_info) {
    int rc;

    /*
     * VSYS and VCHG are active high with a pull-up.
     * Initialize these lines as output low to prevent any spurious
     * activation at boot time.
     */
    gpio_direction_out(VSYS_EN1_N, 0);
    gpio_direction_out(VSYS_EN2_N, 0);
    gpio_direction_out(VSYS_EN3A_N, 0);
    gpio_direction_out(VSYS_EN3B_N, 0);
    gpio_direction_out(VSYS_EN4A_N, 0);
    gpio_direction_out(VSYS_EN4B_N, 0);
    gpio_direction_out(VSYS_EN5_N, 0);
    gpio_direction_out(VCHG_EN1_N, 0);
    gpio_direction_out(VCHG_EN2_N, 0);
    gpio_direction_out(VCHG_EN3A_N, 0);
    gpio_direction_out(VCHG_EN3B_N, 0);
    gpio_direction_out(VCHG_EN4A_N, 0);
    gpio_direction_out(VCHG_EN4B_N, 0);
    gpio_direction_out(VCHG_EN5_N, 0);

    /* For now, just always enable REFCLK_MAIN and the buffers. */
    rc = vreg_config(&refclk_main_vreg) || vreg_get(&refclk_main_vreg);
    if (rc) {
        dbg_error("%s: can't start REFCLK_MAIN: %d\n", __func__, rc);
        return ERROR;
    }

    /* Configure Switch Standby Boot line */
    stm32_configgpio(SW_STANDBY_N);

    /* Configure the switch power supply lines. */
    rc = vreg_config(&sw_vreg);
    if (rc) {
        dbg_error("%s: can't configure switch regulators: %d\n", __func__, rc);
        return ERROR;
    }
    stm32_configgpio(evt1_5_board_info.sw_data.gpio_reset);
    up_udelay(POWER_SWITCH_OFF_STAB_TIME_US);

    /* Configure the wake/detect lines. */
    stm32_configgpio(WD_1_DET_IN_GPIO);
    stm32_configgpio(WD_2_DET_IN_GPIO);
    stm32_configgpio(WD_3A_DET_IN_GPIO);
    stm32_configgpio(WD_3B_DET_IN_GPIO);
    stm32_configgpio(WD_4A_DET_IN_GPIO);
    stm32_configgpio(WD_4B_DET_IN_GPIO);
    stm32_configgpio(WD_5_DET_IN_GPIO);
    stm32_configgpio(WD_8A_DET_IN_GPIO);
    stm32_configgpio(WD_8B_DET_IN_GPIO);

    /* Configure the module release pins */
    stm32_configgpio(MOD_RELEASE_1_CONFIG);
    stm32_configgpio(MOD_RELEASE_2_CONFIG);
    stm32_configgpio(MOD_RELEASE_3A_CONFIG);
    stm32_configgpio(MOD_RELEASE_3B_CONFIG);
    stm32_configgpio(MOD_RELEASE_4A_CONFIG);
    stm32_configgpio(MOD_RELEASE_4B_CONFIG);
    stm32_configgpio(MOD_RELEASE_5_CONFIG);

    /* Configure ARA key input pin */
    stm32_configgpio(ARA_KEY_CONFIG);

    /*
     * Install IRQ for the MSM UART RX line. Used to wake-up the SVC
     * from low power idle
     */
    gpio_direction_in(UART_MSM_TX_SVC_RX);
    if (gpio_irq_settriggering(UART_MSM_TX_SVC_RX, IRQ_TYPE_EDGE_BOTH) ||
        gpio_irq_attach(UART_MSM_TX_SVC_RX, uart_msm_rx_gpio_handler) ||
        gpio_irq_unmask(UART_MSM_TX_SVC_RX)) {
        dbg_error("%s: cannot install wake-up handler for MSM UART RX\n",
                  __func__);
    }

    /*
     * (Module hotplug pins unconfigured. TODO, part of SW-1942.)
     */

    /* Configure AP Wake from OFF pin */
    gpio_set_value(PM_CBL_PWR_N_GPIO, 1);
    gpio_direction_out(PM_CBL_PWR_N_GPIO, 1);

    return 0;
}

struct ara_board_info evt1_5_board_info = {
    .interfaces = evt1_5_interfaces,
    .nr_interfaces = ARRAY_SIZE(evt1_5_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg               = &sw_vreg,
        .gpio_reset         = SVC_RST_SW_GPIO,
        .gpio_irq           = SW_TO_SVC_INT_GPIO,
        .irq_rising_edge    = false,
        .rev                = SWITCH_REV_ES3,
        .bus                = SW_SPI_PORT_2,
        .spi_cs             = SVC_SW_SPI_CS_GPIO,
    },

    .io_expanders           = evt1_io_expanders,
    .nr_io_expanders        = ARRAY_SIZE(evt1_io_expanders),

    /* TODO pwrmon support */
    .pwrmon                 = NULL,

    .board_init             = evt1_5_board_init,

    .ara_key_gpio           = ARA_KEY,
    .ara_key_rising_edge    = true,
    .ara_key_configured     = true,
};

/* EVT2 */
static int evt2_board_init(struct ara_board_info *board_info) {
    int rc;

    /* For now, always enable 3V_ALWAYS_ON */
    rc = vreg_config(&vreg_3v_always_on) || vreg_get(&vreg_3v_always_on);
    if (rc) {
        dbg_error("%s: can't enable 3V_ALWAYS_ON: %d\n", __func__, rc);
        return rc;
    }

    /*
     * VSYS and VCHG are active high with a pull-up.
     * Initialize these lines as output low to prevent any spurious
     * activation at boot time.
     */
    gpio_direction_out(VSYS_EN1_N, 0);
    gpio_direction_out(VSYS_EN2_N, 0);
    gpio_direction_out(VSYS_EN3A_N, 0);
    gpio_direction_out(VSYS_EN3B_N, 0);
    gpio_direction_out(VSYS_EN4A_N, 0);
    gpio_direction_out(VSYS_EN4B_N, 0);
    gpio_direction_out(VCHG_EN1_N, 0);
    gpio_direction_out(VCHG_EN2_N, 0);
    gpio_direction_out(VCHG_EN3A_N, 0);
    gpio_direction_out(VCHG_EN3B_N, 0);
    gpio_direction_out(VCHG_EN4A_N, 0);
    gpio_direction_out(VCHG_EN4B_N, 0);

    /* Now that VSYS are driven we can enable VSYS_EN_PULL */
    gpio_direction_out(VSYS_EN_PULL, 1);

    gpio_direction_out(IOEXP_U4430_RESET, 0);

    rc = ara_cpld_register(&evt2_cpld);
    if (rc) {
        dbg_error("%s: can't initialize CPLD: %d\n", __func__, rc);
        return rc;
    }

    /* For now, just always enable REFCLK_MAIN and the buffers. */
    rc = vreg_config(&evt2_refclk_main_vreg) ||
         vreg_get(&evt2_refclk_main_vreg);
    if (rc) {
        dbg_error("%s: can't start REFCLK_MAIN: %d\n", __func__, rc);
        return rc;
    }

    /* Configure Switch Standby Boot line */
    stm32_configgpio(SW_STANDBY_N);

    /* Configure the switch power supply lines. */
    rc = vreg_config(&sw_vreg);
    if (rc) {
        dbg_error("%s: can't configure switch regulators: %d\n", __func__, rc);
        return rc;
    }
    stm32_configgpio(evt2_board_info.sw_data.gpio_reset);
    up_udelay(POWER_SWITCH_OFF_STAB_TIME_US);

    /* Configure the wake/detect lines. */
    stm32_configgpio(WD_1_DET_IN_GPIO);
    stm32_configgpio(WD_2_DET_IN_GPIO);
    stm32_configgpio(WD_3A_DET_IN_GPIO);
    stm32_configgpio(WD_3B_DET_IN_GPIO);
    stm32_configgpio(WD_4A_DET_IN_GPIO);
    stm32_configgpio(WD_4B_DET_IN_GPIO);
    stm32_configgpio(WD_8A_DET_IN_GPIO);
    stm32_configgpio(WD_8B_DET_IN_GPIO);

    /* Configure the latch lines. */
    stm32_configgpio(LATCH_1_DET_GPIO);
    stm32_configgpio(LATCH_2_DET_GPIO);
    stm32_configgpio(LATCH_3A_DET_GPIO);
    stm32_configgpio(LATCH_3B_DET_GPIO);
    stm32_configgpio(LATCH_4A_DET_GPIO);
    stm32_configgpio(LATCH_4B_DET_GPIO);

    /* Configure the module release pins */
    stm32_configgpio(MOD_RELEASE_1_CONFIG);
    stm32_configgpio(MOD_RELEASE_2_CONFIG);
    stm32_configgpio(EVT2_MOD_RELEASE_3A_CONFIG);
    stm32_configgpio(MOD_RELEASE_3B_CONFIG);
    stm32_configgpio(MOD_RELEASE_4A_CONFIG);
    stm32_configgpio(MOD_RELEASE_4B_CONFIG);

    /*
     * (Module hotplug pins unconfigured. TODO, part of SW-1942.)
     */

    /* Configure AP Wake from OFF pin */
    gpio_set_value(PM_CBL_PWR_N_GPIO, 1);
    gpio_direction_out(PM_CBL_PWR_N_GPIO, 1);

    return 0;
}

struct ara_board_info evt2_board_info = {
    .interfaces = evt2_interfaces,
    .nr_interfaces = ARRAY_SIZE(evt2_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg               = &sw_vreg,
        .gpio_reset         = SVC_RST_SW_GPIO,
        .gpio_irq           = SW_TO_SVC_INT_GPIO,
        .irq_rising_edge    = false,
        .rev                = SWITCH_REV_ES3,
        .bus                = SW_SPI_PORT_2,
        .spi_cs             = SVC_SW_SPI_CS_GPIO,
    },

    .io_expanders           = evt2_io_expanders,
    .nr_io_expanders        = ARRAY_SIZE(evt2_io_expanders),

    /* TODO pwrmon support */
    .pwrmon                 = NULL,

    .board_init             = evt2_board_init,

    .vlatch_vdd = &vlatch_vdd_vreg,
    .latch_ilim = &latch_ilim_vreg,

    .mod_sense = MOD_SENSE,
};
