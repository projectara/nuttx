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
#define IOEXP_U4550_I2C_ADDR    0x21
#define IOEXP_U4570_I2C_ADDR    0x20
#define IOEXP1_INT_N            STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN9)
#define IOEXP2_INT_N            STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN10)
#define SVC_RST_IOEXP1          STM32_GPIO_PIN(GPIO_PORTC | GPIO_PIN14)
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

/* Wake/detect pins */
#define WD_1_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define WD_3A_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define WD_3B_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define WD_4A_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define WD_4B_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define WD_5_DET_IN_GPIO    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7) /* LCD */
#define WD_8A_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define WD_8B_DET_IN_GPIO   (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)

/*
 * MOD_ACT_SW lines connected to SVC.
 *
 * Configured by default as input pullup, as there is no external
 * pullup.
 */
#define MOD_ACT_SW_1        (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN0)
#define MOD_ACT_SW_2        (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN2)
#define MOD_ACT_SW_3A       (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN3)
#define MOD_ACT_SW_3B       (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN4)
#define MOD_ACT_SW_4A       (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN6)
#define MOD_ACT_SW_4B       (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN7)
#define MOD_ACT_SW_5        (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN8)

/* Module release pins */
#define MOD_RELEASE_1_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3A_CONFIG  (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR \
                                | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN2)
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
#define MOD_RELEASE_3B      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_4A      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_4B      STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN7)
#define MOD_RELEASE_5       STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN7)

#define ARA_KEY_CONFIG        (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA \
                               | GPIO_PIN0)
#define ARA_KEY               STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN0)

/* Switch control pins */
#define SW_1P1_EN          STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN0)
#define SW_1P8_IO_EN       STM32_GPIO_PIN(GPIO_PORTB | GPIO_PIN1)
#define SW_1P8_UNIPRO_EN   STM32_GPIO_PIN(GPIO_PORTE | GPIO_PIN8)
#define SW_STANDBY_N       STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN8)
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
#define PM_CBL_PWR_N GPIO  U4550_GPIO_PIN(15)

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
 */

static struct vreg_data apb1_vreg_data[] = {
};

/* Only APB1 is in use. WD8B is reserved for time sync */
DECLARE_MODULE_PORT_INTERFACE(apb1, apb1_vreg_data, 3,
                              WD_8A_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_HIGH,
			                  false, 0);

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
static struct vreg_data evt1_5_module_1_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN1_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data evt1_5_module_2_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN2_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data evt1_5_module_3A_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN3A_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_3A_EN),
};

static struct vreg_data evt1_5_module_3B_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN3B_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_3B_EN),
};

static struct vreg_data evt1_5_module_4A_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN4A_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data evt1_5_module_4B_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN4B_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data evt1_5_module_5_lcd_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN5_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_1, evt1_5_module_1_vreg_data, 13,
                              WD_1_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_2, evt1_5_module_2_vreg_data, 11,
                              WD_2_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_3A, evt1_5_module_3A_vreg_data, 4,
                              WD_3A_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_3A);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_3B, evt1_5_module_3B_vreg_data, 2,
                              WD_3B_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_4A, evt1_5_module_4A_vreg_data, 6,
                              WD_4A_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_4A);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_4B, evt1_5_module_4B_vreg_data, 8,
                              WD_4B_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_4B);
DECLARE_MODULE_PORT_INTERFACE(evt1_5_module_5_lcd,
                              evt1_5_module_5_lcd_vreg_data, 10,
                              WD_5_DET_IN_GPIO, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_5);

static struct interface *evt1_5_interfaces[] = {
    &apb1_interface,
    &evt1_5_module_1_interface,
    &evt1_5_module_2_interface,
    &evt1_5_module_3A_interface,
    &evt1_5_module_3B_interface,
    &evt1_5_module_4A_interface,
    &evt1_5_module_4B_interface,
    &evt1_5_module_5_lcd_interface,
};

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
DECLARE_VREG(sw, sw_vreg_data);

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

/*
 * Board info, init, and exit
 */
static struct vreg_data refclk_main_vreg_data[] = {
    INIT_MODULE_CLK_DATA(REFCLK_REQ),
    INIT_MODULE_CLK_DATA(REFCLK_BUFFERS_EN),
};

DECLARE_VREG(refclk_main, refclk_main_vreg_data);

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
     * (Module hotplug pins unconfigured. TODO, part of SW-1942.)
     */

    return 0;
}

struct ara_board_info evt1_5_board_info = {
    .interfaces = evt1_5_interfaces,
    .nr_interfaces = ARRAY_SIZE(evt1_5_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg            = &sw_vreg,
        .gpio_reset      = SVC_RST_SW_GPIO,
        .gpio_irq        = SW_TO_SVC_INT_GPIO,
        .irq_rising_edge = false,
        .rev             = SWITCH_REV_ES3,
        .bus             = SW_SPI_PORT_2,
        .spi_cs          = SVC_SW_SPI_CS_GPIO,
    },

    .io_expanders = evt1_io_expanders,
    .nr_io_expanders = ARRAY_SIZE(evt1_io_expanders),

    /* TODO pwrmon support */
    .pwrmon               = NULL,

    .board_init           = evt1_5_board_init,

    .ara_key_gpio         = ARA_KEY,
    .ara_key_rising_edge  = true,
    .ara_key_configured   = true,
};
