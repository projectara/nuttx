/*
 * Copyright (c) 2015 Google Inc.
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
#define REFCLK_BUFFERS_EN U4570_GPIO_PIN(2) // Shared clock enable for modules
#define REFCLK_SW_EN      U4570_GPIO_PIN(3) // Switch
#define REFCLK_1_EN       U4570_GPIO_PIN(4) // Modules
#define REFCLK_2_EN       U4570_GPIO_PIN(5)
#define REFCLK_3A_EN      U4570_GPIO_PIN(6)
#define REFCLK_3B_EN      U4570_GPIO_PIN(7)
#define REFCLK_4A_EN      U4570_GPIO_PIN(8)
#define REFCLK_4B_EN      U4570_GPIO_PIN(9)
#define REFCLK_5_EN       U4570_GPIO_PIN(10)

/*
 * WAKE_DETECT lines directly connected to the SVC.
 * Configured by default as input floating without pull-up.
 */
#define WD_1_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN1)
#define WD_2_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN2)
#define WD_3A_DET_IN    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN3)
#define WD_3B_DET_IN    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN4)
#define WD_4A_DET_IN    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN5)
#define WD_4B_DET_IN    (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN6)
#define WD_5_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN7)
#define WD8A_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN11)
#define WD8B_DET_IN     (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTA | GPIO_PIN12)

/*
 * MOD_ACT_SW lines connected to SVC.
 *
 * Configured by default as input pullup, as there is no external
 * pullup.
 */
#define MOD_ACT_SW_1    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN0)
#define MOD_ACT_SW_2    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN2)
#define MOD_ACT_SW_3A   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN3)
#define MOD_ACT_SW_3B   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN4)
#define MOD_ACT_SW_4A   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN6)
#define MOD_ACT_SW_4B   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN7)
#define MOD_ACT_SW_5    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN8)

/* VSYS enable */
#define VSYS_EN1_N      U4550_GPIO_PIN(0)
#define VSYS_EN2_N      U4550_GPIO_PIN(2)
#define VSYS_EN3A_N     U4550_GPIO_PIN(4)
#define VSYS_EN3B_N     U4550_GPIO_PIN(6)
#define VSYS_EN4A_N     U4550_GPIO_PIN(8)
#define VSYS_EN4B_N     U4550_GPIO_PIN(10)
#define VSYS_EN5_N      U4550_GPIO_PIN(12)

/* VCHG enable */
#define VCHG_EN1_N      U4550_GPIO_PIN(1)
#define VCHG_EN2_N      U4550_GPIO_PIN(3)
#define VCHG_EN3A_N     U4550_GPIO_PIN(5)
#define VCHG_EN3B_N     U4550_GPIO_PIN(7)
#define VCHG_EN4A_N     U4550_GPIO_PIN(9)
#define VCHG_EN4B_N     U4550_GPIO_PIN(11)
#define VCHG_EN5_N      U4550_GPIO_PIN(13)

/*
 * SVC to MSM Wake from Off pin: Active low, pulled up high internally by
 * the PMIC.
 * Not used for now, let it in default state of the I/O Expander (input).
 */
#define PM_CBL_PWR_N GPIO   U4550_GPIO_PIN(15)

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

#define MOD_RELEASE_1         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN0)
#define MOD_RELEASE_2         STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN1)
#define MOD_RELEASE_3A        STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN2)
#define MOD_RELEASE_3B        STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN4)
#define MOD_RELEASE_4A        STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN6)
#define MOD_RELEASE_4B        STM32_GPIO_PIN(GPIO_PORTD | GPIO_PIN7)

#define ARA_KEY_CONFIG        (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA \
                               | GPIO_PIN0)
#define ARA_KEY               STM32_GPIO_PIN(GPIO_PORTA | GPIO_PIN0)

/*
 * On-board bridges clock control
 */

/* FIXME: put the bridge enables back in here once the handshaking
 * with MSM is OK. */

/* Only APB1 is in use. WD8B is reserved for time sync */
static struct vreg_data apb1_vreg_data[] = {
};

/*
 * Modules voltage regulator list
 *
 * Notes:
 *
 * The modules loadswitches are controlled by the INA231 ALERT output,
 * to be driven via I2C commands. Take a shortcut for bring-up until the
 * INA231 support is in: directly control it via the VSYS_ENx GPIO pins.
 * (ToDo)
 */
static struct vreg_data module_1_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN1_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_1_EN),
};

static struct vreg_data module_2_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN2_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_2_EN),
};

static struct vreg_data module_3A_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN3A_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_3A_EN),
};

static struct vreg_data module_3B_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN3B_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_3B_EN),
};

static struct vreg_data module_4A_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN4A_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_4A_EN),
};

static struct vreg_data module_4B_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN4B_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_4B_EN),
};

static struct vreg_data module_5_lcd_vreg_data[] = {
    INIT_ACTIVE_HIGH_VREG_DATA(VSYS_EN5_N, HOLD_TIME_MODULE),
    INIT_MODULE_CLK_DATA(REFCLK_5_EN),
};

/*
 * Interfaces on this board
 */
DECLARE_MODULE_PORT_INTERFACE(apb1, apb1_vreg_data, 3,
                              WD8A_DET_IN, ARA_IFACE_WD_ACTIVE_HIGH, false, 0);
DECLARE_MODULE_PORT_INTERFACE(module_1, module_1_vreg_data, 13,
                              WD_1_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_1);
DECLARE_MODULE_PORT_INTERFACE(module_2, module_2_vreg_data, 11,
                              WD_2_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_2);
DECLARE_MODULE_PORT_INTERFACE(module_3a, module_3A_vreg_data, 4,
                              WD_3A_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_3A);
DECLARE_MODULE_PORT_INTERFACE(module_3b, module_3B_vreg_data, 10,
                              WD_3B_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_3B);
DECLARE_MODULE_PORT_INTERFACE(module_4a, module_4A_vreg_data, 6,
                              WD_4A_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_4A);
DECLARE_MODULE_PORT_INTERFACE(module_4b, module_4B_vreg_data, 8,
                              WD_4B_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              true, MOD_RELEASE_4B);
DECLARE_MODULE_PORT_INTERFACE(module_5, module_5_lcd_vreg_data, INVALID_PORT,
                              WD_5_DET_IN, ARA_IFACE_WD_ACTIVE_LOW,
                              false, 0);

#define I2C_SEL1_A      BIT(0)
#define I2C_SEL1_B      BIT(1)
#define I2C_SEL1_INH    BIT(2)

enum {
    I2C_INA230_SEL1_A = STM32_GPIO_CHIP_BASE + 71, /* PE7 */
    I2C_INA230_SEL1_B,
    I2C_INA230_SEL1_INH,
};

/*
 * Global power monitoring I2C bus.
 */
const int pwrmon_i2c_bus = 1;

/*
 * Power rail groups definitions.
 *
 * If a bit is set in i2c_sel, then drive the GPIO low.
 */
const struct pwrmon_dev_ctx pwrmon_devs[] = {
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

const size_t pwrmon_num_devs = ARRAY_SIZE(pwrmon_devs);

void pwrmon_reset_i2c_sel(void)
{
    gpio_set_value(I2C_INA230_SEL1_INH, 1);
    gpio_set_value(I2C_INA230_SEL1_A, 1);
    gpio_set_value(I2C_INA230_SEL1_B, 1);
}

void pwrmon_init_i2c_sel(void)
{
    gpio_direction_out(I2C_INA230_SEL1_A, 1);
    gpio_direction_out(I2C_INA230_SEL1_B, 1);
    gpio_direction_out(I2C_INA230_SEL1_INH, 1);
}

int pwrmon_do_i2c_sel(uint8_t dev)
{
    if (dev >= ARRAY_SIZE(pwrmon_devs)) {
        return -EINVAL;
    }

    /* First inhibit all lines, to make sure there is no short/collision */
    gpio_set_value(I2C_INA230_SEL1_INH, 1);

    gpio_set_value(I2C_INA230_SEL1_A, pwrmon_devs[dev].i2c_sel & I2C_SEL1_A ? 0 : 1);
    gpio_set_value(I2C_INA230_SEL1_B, pwrmon_devs[dev].i2c_sel & I2C_SEL1_B ? 0 : 1);

    if (pwrmon_devs[dev].i2c_sel & I2C_SEL1_INH) {
        gpio_set_value(I2C_INA230_SEL1_INH, 0);
    }

    return 0;
}

/*
 * Important note: Always declare the spring interfaces last.
 * Assumed by Spring Power Measurement Library (up_spring_pm.c).
 */
static struct interface *db3_interfaces[] = {
    &apb1_interface,
    &module_1_interface,
    &module_2_interface,
    &module_3a_interface,
    &module_3b_interface,
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
DECLARE_VREG(sw, sw_vreg_data);

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

static struct ara_board_info db3_board_info = {
    .interfaces = db3_interfaces,
    .nr_interfaces = ARRAY_SIZE(db3_interfaces),
    .nr_spring_interfaces = 0,

    .sw_data = {
        .vreg             = &sw_vreg,
        .gpio_reset       = (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
                             GPIO_PORTD | GPIO_PIN11),
        .gpio_irq         = (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                             GPIO_PORTC | GPIO_PIN9),
        .irq_rising_edge  = false,
        .rev              = SWITCH_REV_ES3,
        .bus              = SW_SPI_PORT_2,
        .spi_cs           = (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET | \
                             GPIO_PORTB | GPIO_PIN12)
    },

    .io_expanders   = db3_io_expanders,
    .nr_io_expanders = ARRAY_SIZE(db3_io_expanders),

    .ara_key_gpio         = ARA_KEY,
    .ara_key_rising_edge  = true,
    .ara_key_configured   = true,
};

struct ara_board_info *board_init(void) {
    int i;

    /* Disable the I/O Expanders for now */
    stm32_configgpio(SVC_RST_IOEXP1);
    stm32_gpiowrite(SVC_RST_IOEXP1, false);
    stm32_configgpio(SVC_RST_IOEXP2);
    stm32_gpiowrite(SVC_RST_IOEXP2, false);

    /*
     * Register the STM32 GPIOs to Gpio Chip
     *
     * This needs to happen before the I/O Expanders registration, which
     * uses some STM32 pins
     */
    stm32_gpio_init();

    /* Register the TCA64xx I/O Expanders GPIOs to Gpio Chip */
    for (i = 0; i < db3_board_info.nr_io_expanders; i++) {
        struct io_expander_info *io_exp = &db3_board_info.io_expanders[i];

        io_exp->i2c_dev = up_i2cinitialize(io_exp->i2c_bus);
        if (!io_exp->i2c_dev) {
            dbg_error("%s(): Failed to get I/O Expander I2C bus %u\n",
                      __func__, io_exp->i2c_bus);
            goto err_deinit_gpio;
        } else {
            if (tca64xx_init(&io_exp->io_exp_driver_data,
                             io_exp->part,
                             io_exp->i2c_dev,
                             io_exp->i2c_addr,
                             io_exp->reset,
                             io_exp->irq,
                             io_exp->gpio_base) < 0) {
                dbg_error("%s(): Failed to register I/O Expander(0x%02x)\n",
                          __func__, io_exp->i2c_addr);
                goto err_uninit_i2c;
            }
        }
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
    gpio_direction_out(VSYS_EN5_N, 0);
    gpio_direction_out(VCHG_EN1_N, 0);
    gpio_direction_out(VCHG_EN2_N, 0);
    gpio_direction_out(VCHG_EN3A_N, 0);
    gpio_direction_out(VCHG_EN3B_N, 0);
    gpio_direction_out(VCHG_EN4A_N, 0);
    gpio_direction_out(VCHG_EN4B_N, 0);
    gpio_direction_out(VCHG_EN5_N, 0);

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
    stm32_configgpio(WD_1_DET_IN);
    stm32_configgpio(WD_2_DET_IN);
    stm32_configgpio(WD_3A_DET_IN);
    stm32_configgpio(WD_3B_DET_IN);
    stm32_configgpio(WD_4A_DET_IN);
    stm32_configgpio(WD_4B_DET_IN);
    stm32_configgpio(WD_5_DET_IN);
    stm32_configgpio(WD8A_DET_IN);
    stm32_configgpio(WD8B_DET_IN);

    /*
     * Configure the MOD_ACT_SW pins.
     */
    stm32_configgpio(MOD_ACT_SW_1);
    stm32_configgpio(MOD_ACT_SW_2);
    stm32_configgpio(MOD_ACT_SW_3A);
    stm32_configgpio(MOD_ACT_SW_3B);
    stm32_configgpio(MOD_ACT_SW_4A);
    stm32_configgpio(MOD_ACT_SW_4B);
    stm32_configgpio(MOD_ACT_SW_5);

    /*
     * Configure the module release pins
     */
    stm32_configgpio(MOD_RELEASE_1_CONFIG);
    stm32_configgpio(MOD_RELEASE_2_CONFIG);
    stm32_configgpio(MOD_RELEASE_3A_CONFIG);
    stm32_configgpio(MOD_RELEASE_3B_CONFIG);
    stm32_configgpio(MOD_RELEASE_4A_CONFIG);
    stm32_configgpio(MOD_RELEASE_4B_CONFIG);

    /* Configure ARA key input pin */
    stm32_configgpio(ARA_KEY_CONFIG);

    return &db3_board_info;

 err_uninit_i2c:
    /* Done in reverse order to account for possible IRQ chaining. */
    for (i = db3_board_info.nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &db3_board_info.io_expanders[i];
        if (io_exp->i2c_dev) {
            up_i2cuninitialize(io_exp->i2c_dev);
        }
    }
 err_deinit_gpio:
    stm32_gpio_deinit();
    /* Leave the I/O expanders in reset here. */
    return NULL;
}

void board_exit(void) {
    int i;
    /*
     * First unregister the TCA64xx I/O Expanders and associated I2C bus(ses).
     * Done in reverse order from registration to account for IRQ chaining
     * between I/O Expander chips.
     */
    for (i = db3_board_info.nr_io_expanders - 1; i >= 0; i--) {
        struct io_expander_info *io_exp = &db3_board_info.io_expanders[i];

        if (io_exp->io_exp_driver_data)
            tca64xx_deinit(io_exp->io_exp_driver_data);

        if (io_exp->i2c_dev)
            up_i2cuninitialize(io_exp->i2c_dev);
    }

    /* Lastly unregister the GPIO Chip driver */
    stm32_gpio_deinit();
}
