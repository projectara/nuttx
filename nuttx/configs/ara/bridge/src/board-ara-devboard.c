/**
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
 *
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>
#include <nuttx/device_hid.h>
#include <nuttx/util.h>
#include <nuttx/usb.h>
#include <nuttx/device_lights.h>

#include "tsb_scm.h"
#include "up_arch.h"

#include <arch/board/csi_tx_service.h>
#include <arch/tsb/gpio.h>

#ifdef CONFIG_ARA_BRIDGE_HAVE_POWER_SUPPLY
#include <nuttx/device_power_supply.h>
#endif

#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
#include <nuttx/device_sdio_board.h>
#define SD_POWER_EN_PIN    9 /* GPIO 9 */
#define SD_CARD_DETECT_PIN 22 /* GPIO 22 */
#endif

#ifdef CONFIG_APBRIDGEA
/* must pull up or drive high on SDB APBridgeA to bring Helium out of reset */
#define HELIUM_EXT_NRST_BTN_GPIO 0
#endif

#ifdef CONFIG_DEVICE_CORE
#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
static struct device_resource sdio_board_resources[] = {
    {
        .name  = "sdio_gpio_power",
        .type  = DEVICE_RESOURCE_TYPE_GPIO,
        .start = SD_POWER_EN_PIN,
        .count = 1,
    },
    {
        .name  = "sdio_gpio_cd",
        .type  = DEVICE_RESOURCE_TYPE_GPIO,
        .start = SD_CARD_DETECT_PIN,
        .count = 1,
    },
};
#endif

static struct device devices[] = {
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB4624
    {
        .type           = DEVICE_TYPE_HSIC_DEVICE,
        .name           = "usb4624",
        .desc           = "USB4624 HSIC Hub",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB3813
    {
        .type           = DEVICE_TYPE_HSIC_DEVICE,
        .name           = "usb3813",
        .desc           = "USB3813 HSIC Hub",
        .id             = 1,
    },
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_POWER_SUPPLY
    {
        .type           = DEVICE_TYPE_POWER_SUPPLY_DEVICE,
        .name           = "power_supply",
        .desc           = "Power Supply Controller",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_HID_DEVICE
    {
        .type           = DEVICE_TYPE_HID_HW,
        .name           = HID_DEVICE_NAME,
        .desc           = HID_DRIVER_DESCRIPTION,
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_LIGHTS
    {
        .type           = DEVICE_TYPE_LIGHTS_HW,
        .name           = "lights",
        .desc           = "Lights Device Controller",
        .id             = 0,
    },
#endif
#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
    {
        .type           = DEVICE_TYPE_SDIO_BOARD_HW,
        .name           = "sdio_board",
        .desc           = "SDIO Board Device Driver",
        .id             = 0,
        .resources      = sdio_board_resources,
        .resource_count = ARRAY_SIZE(sdio_board_resources),
    },
#endif
};

static struct device_table bdb_device_table = {
    .device = devices,
    .device_count = ARRAY_SIZE(devices),
};

static void bdb_driver_register(void)
{
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB4624
    extern struct device_driver usb4624_driver;
    device_register_driver(&usb4624_driver);
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_USB3813
    extern struct device_driver usb3813_driver;
    device_register_driver(&usb3813_driver);
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_POWER_SUPPLY
    extern struct device_driver power_supply_driver;
    device_register_driver(&power_supply_driver);
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_HID_DEVICE
    extern struct device_driver hid_dev_driver;
    device_register_driver(&hid_dev_driver);
#endif
#ifdef CONFIG_ARA_BRIDGE_HAVE_LIGHTS
    extern struct device_driver lights_driver;
    device_register_driver(&lights_driver);
#endif
#ifdef CONFIG_ARCH_CHIP_DEVICE_SDIO
    extern struct device_driver sdio_board_driver;
    device_register_driver(&sdio_board_driver);
#endif

}
#endif

void ara_module_early_init(void)
{
}

void ara_module_init(void)
{
#ifdef CONFIG_DEVICE_CORE
    device_table_register(&bdb_device_table);
    bdb_driver_register();
#endif

#if defined(CONFIG_APBRIDGEA) && defined(CONFIG_ARCH_CHIP_DEVICE_CSI)
    csi_tx_srv_init();
#endif
}
