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

/**
 * @brief: Functions and definitions for interface block management.
 */

#define DBG_COMP ARADBG_SVC     /* DBG_COMP macro of the component */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/gpio.h>
#include <nuttx/power/pm.h>

#include <time.h>
#include <sys/time.h>
#include <errno.h>

#include <ara_debug.h>
#include "interface.h"
#include "vreg.h"
#include "string.h"
#include "svc.h"
#include "tsb_switch.h"
#include "tsb_switch_event.h"

#define POWER_OFF_TIME_IN_US                        (500000)
#define WAKEOUT_PULSE_DURATION_IN_US                (100000)
#define MODULE_PORT_WAKEOUT_PULSE_DURATION_IN_US    (500000)
#define LINKUP_WD_DELAY_IN_MS                       (100)
#define LINKUP_WD_DELAY        ((LINKUP_WD_DELAY_IN_MS * CLOCKS_PER_SEC) / 1000)

static struct interface **interfaces;
static unsigned int nr_interfaces;
static unsigned int nr_spring_interfaces;
static struct work_s linkup_work;

static void interface_power_cycle(void *data);
static void interface_uninstall_wd_handler(struct wd_data *wd);
static int interface_install_wd_handler(struct interface *iface, bool);

/**
 * @brief Configure all the voltage regulators associated with an interface
 * to their default states.
 * @param iface interface to configure
 */
static int interface_config(struct interface *iface)
{
    int rc_pwr, rc_clk;

    dbg_verbose("Configuring interface %s.\n",
                iface->name ? iface->name : "unknown");

    /* Configure default state for the regulator pins */
    rc_pwr = vreg_config(iface->vsys_vreg);
    rc_clk = vreg_config(iface->refclk_vreg);

    /* Configure the interfaces pin according to the interface type */
    switch (iface->if_type) {
    case ARA_IFACE_TYPE_MODULE_PORT:
        /*
         * DB3 module port:
         * The WAKEOUT pin is configured as interrupt input at handler
         * installation time
         */

        /* Configure the release line */
        if (iface->ejectable) {
            gpio_activate(iface->release_gpio);
            gpio_direction_out(iface->release_gpio, 0);
        }
        break;
    default:
        break;
    }

    /* Init power state */
    atomic_init(&iface->power_state,
                rc_pwr ? ARA_IFACE_PWR_ERROR : ARA_IFACE_PWR_DOWN);
    atomic_init(&iface->refclk_state,
                rc_clk ? ARA_IFACE_PWR_ERROR : ARA_IFACE_PWR_DOWN);

    return rc_pwr ? rc_pwr : rc_clk ? rc_clk : 0;
}

/**
 * @brief Supply the reference clock to an interface
 *
 * This function attempts to apply the reference clock to the
 * interface, and updates the interface's refclk state accordingly.
 * This affects the value returned by interface_get_refclk_state(iface).
 *
 * @param iface Interface whose reference clock to enable
 * @return 0 on success, <0 on error
 */
static int interface_refclk_enable(struct interface *iface)
{
    int rc;

    if (!iface) {
        return -EINVAL;
    }

    rc = vreg_get(iface->refclk_vreg);
    if (rc) {
        dbg_error("Failed to enable the reference clock for interface %s: %d\n",
                  iface->name, rc);
        atomic_init(&iface->refclk_state, ARA_IFACE_PWR_ERROR);
    } else {
        atomic_init(&iface->refclk_state, ARA_IFACE_PWR_UP);
    }

    return rc;
}


/**
 * @brief Disable the reference clock supply to this interface
 *
 * This function attempts to remove the reference clock from the
 * interface, and updates the interface's refclk state accordingly.
 * This affects the value returned by interface_get_refclk_state(iface).
 *
 * @param iface Interface whose reference clock to disable
 * @return 0 on success, <0 on error
 */
static int interface_refclk_disable(struct interface *iface)
{
    int rc;

    if (!iface) {
        return -EINVAL;
    }

    rc = vreg_put(iface->refclk_vreg);
    if (rc) {
        dbg_error("Failed to disable the reference clock for interface %s: %d\n",
                  iface->name, rc);
        atomic_init(&iface->refclk_state, ARA_IFACE_PWR_ERROR);
    } else {
        atomic_init(&iface->refclk_state, ARA_IFACE_PWR_DOWN);
    }

    return rc;
}

/**
 * @brief Turn on the power to this interface
 *
 * This function attempts to apply power, clock, etc. to the
 * interface, and updates the interface's power state accordingly.
 * This affects the value returned by interface_get_pwr_state(iface).
 *
 * @param iface Interface whose power to enable
 * @return 0 on success, <0 on error
 */
static int interface_power_enable(struct interface *iface)
{
    int rc;

    if (!iface) {
        dbg_verbose("%s: called with null interface\n", __func__);
        return -EINVAL;
    }

    rc = vreg_get(iface->vsys_vreg);
    if (rc) {
        dbg_error("Failed to enable interface %s: %d\n", iface->name, rc);
        atomic_init(&iface->power_state, ARA_IFACE_PWR_ERROR);
    } else {
        atomic_init(&iface->power_state, ARA_IFACE_PWR_UP);
    }

    return rc;
}


/**
 * @brief Turn off the power to this interface
 *
 * This function attempts to remove power, clock, etc. from the
 * interface, and updates the interface's power state accordingly.
 * This affects the value returned by interface_get_pwr_state(iface).
 *
 * @param iface Interface whose power to disable
 * @return 0 on success, <0 on error
 */
static int interface_power_disable(struct interface *iface)
{
    int rc;

    if (!iface) {
        dbg_verbose("%s: called with null interface\n", __func__);
        return -EINVAL;
    }

    rc = vreg_put(iface->vsys_vreg);
    if (rc) {
        dbg_error("Failed to disable interface %s: %d\n", iface->name, rc);
        atomic_init(&iface->power_state, ARA_IFACE_PWR_ERROR);
    } else {
        atomic_init(&iface->power_state, ARA_IFACE_PWR_DOWN);
    }

    return rc;
}


/*
 * @brief Generate a WAKEOUT signal to wake-up/power-up modules.
 * If assert is true, keep the WAKEOUT lines asserted.
 *
 * The corresponding power supplies must already be enabled.
 */
int interface_generate_wakeout(struct interface *iface, bool assert,
                               int length)
{
    int rc;

    if (!iface) {
        dbg_error("%s: called with null interface\n", __func__);
        return -ENODEV;
    }

    if (length <= 0) {
        dbg_info("Generating WAKEOUT on interface %s\n",
                 iface->name ? iface->name : "unknown");
    } else {
        dbg_info("Generating WAKEOUT on interface %s (%d us)\n",
                 iface->name ? iface->name : "unknown", length);
    }

    /* Generate a WAKEOUT pulse
     *
     * DB3 module port:
     * Generate a pulse on the WD line. The polarity is reversed
     * from the DETECT_IN polarity.
     */
    if (iface->detect_in.gpio) {
        bool polarity = iface->detect_in.polarity;
        int pulse_len = (length > 0) ?
                        length : MODULE_PORT_WAKEOUT_PULSE_DURATION_IN_US;

        /* First uninstall the interrupt handler on the pin */
        interface_uninstall_wd_handler(&iface->detect_in);
        /* Then configure the pin as output and assert it */
        gpio_direction_out(iface->detect_in.gpio, polarity ? 0 : 1);

        /* Keep the line asserted for the given duration */
        up_udelay(pulse_len);

        /* Finally re-install the interrupt handler on the pin */
        rc = interface_install_wd_handler(iface, true);
        if (rc) {
            return rc;
        }
    }

    return 0;
}


/**
 * @brief Get interface power supply state
 * @param iface Interface whose power state to retrieve
 * @return iface's power state, or ARA_IFACE_PWR_ERROR if iface == NULL.
 */
enum ara_iface_pwr_state interface_get_power_state(struct interface *iface)
{
    if (!iface) {
        return ARA_IFACE_PWR_ERROR;
    }

    return atomic_get(&iface->power_state);
}

/**
 * @brief Get interface refclk supply state
 * @param iface Interface whose refclk state to retrieve
 * @return iface's refclk state, or ARA_IFACE_PWR_ERROR if iface == NULL.
 */
static enum ara_iface_pwr_state
interface_get_refclk_state(struct interface *iface)
{
    if (!iface) {
        return ARA_IFACE_PWR_ERROR;
    }

    return atomic_get(&iface->refclk_state);
}


/*
 * Interface power control helper, to be used by the DETECT_IN/hotplug
 * mechanism.
 *
 * Power OFF the interface
 */
int interface_power_off(struct interface *iface)
{
    int rc;

    if (!iface) {
        return -EINVAL;
    }

    wd_cancel(&iface->linkup_wd);

    /* Disable Switch port */
    rc = switch_enable_port(svc->sw, iface->switch_portid, false);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to disable switch port for interface %s: %d.\n",
                  iface->name, rc);
    }

    /* Power off the interface */
    rc = interface_power_disable(iface);
    if (rc < 0) {
        return rc;
    }

    rc = interface_refclk_disable(iface);
    if (rc < 0) {
        return rc;
    }

    return 0;
}

static void interface_linkup_timeout(int argc, uint32_t arg1, ...)
{
    struct interface *iface = (struct interface*) arg1;

    DEBUGASSERT(sizeof(struct interface*) == sizeof(uint32_t));

    iface->linkup_retries++;

    dbg_warn("Link-up took more than %d ms, turning interface '%s' OFF and ON again\n",
             LINKUP_WD_DELAY_IN_MS, iface->name);

    work_queue(HPWORK, &linkup_work, interface_power_cycle, iface, 0);
}

/*
 * Interface power control helper, to be used by the DETECT_IN/hotplug
 * mechanism.
 *
 * Power ON the interface in order to cleanly reboot the interface
 * module(s). Then an initial handshake between the module(s) and the
 * interface can take place.
 */
int interface_power_on(struct interface *iface)
{
    int rc;

    if (!iface) {
        return -EINVAL;
    }

    /* If powered OFF, power it ON now */
    if (!interface_get_power_state(iface)) {
        rc = interface_power_enable(iface);
        if (rc < 0) {
            return rc;
        }
    }

    if (!interface_get_refclk_state(iface)) {
        rc = interface_refclk_enable(iface);
        if (rc < 0) {
            return rc;
        }
    }

    /* Generate WAKE_OUT */
    rc = interface_generate_wakeout(iface, false, -1);
    if (rc) {
        dbg_error("Failed to generate wakeout on interface %s\n", iface->name);
        return rc;
    }

    /*
     * HACK (SW-2591)
     *
     * There are issues with cold boot support for built-in
     * (non-ejectable) interfaces which are leading to a significant
     * percentage of boots failing to result in a working UniPro
     * network.
     *
     * Skip the watchdog and linkup retries while those are being
     * debugged. The race condition which this watchdog is intended to
     * avoid doesn't happen as often, so not dealing with it actually
     * leads to better behavior for now.
     */
    if (iface->ejectable) {
        wd_start(&iface->linkup_wd, LINKUP_WD_DELAY, interface_linkup_timeout, 1,
                 iface);
    } else {
        dbg_info("%s: skipping linkup watchdog for interface %s\n",
                 __func__, iface->name);
    }

    /* Enable Switch port */
    rc = switch_enable_port(svc->sw, iface->switch_portid, true);
    if (rc && (rc != -EOPNOTSUPP)) {
        dbg_error("Failed to enable switch port for interface %s: %d.\n",
                  iface->name, rc);
        return rc;
    }

    /* Request manual LinkUp of the Unipro port */
    struct tsb_switch_event e;
    e.type = TSB_SWITCH_EVENT_LINKUP;
    e.linkup.port = iface->switch_portid;
    e.linkup.val = SW_LINKUP_INITIATE;
    rc = tsb_switch_event_notify(svc->sw, &e);
    if (rc) {
        dbg_error("Failed to request LinkUp for interface %s\n", iface->name);
    }

    return 0;
}

void interface_cancel_linkup_wd(struct interface *iface)
{
    if (!iface->ejectable) {
        /*
         * HACK (SW-2591): see interface_power_on() comment with this
         * issue tag.
         */
        return;
    }
    dbg_verbose("Canceling linkup watchdog for '%s'\n", iface->name);
    wd_cancel(&iface->linkup_wd);
}

static void interface_power_cycle(void *data)
{
    struct interface *iface = data;
    uint8_t retries;

    interface_power_off(iface);

    if (iface->linkup_retries >= INTERFACE_MAX_LINKUP_TRIES) {
        dbg_error("Could not link-up with '%s' in less than %d ms, aborting after %d tries\n",
                  iface->name, LINKUP_WD_DELAY_IN_MS,
                  INTERFACE_MAX_LINKUP_TRIES);
        return;
    }

    retries = iface->linkup_retries;
    interface_power_on(iface);
    iface->linkup_retries = retries;
}

/**
 * @brief           Return the name of the interface
 * @return          Interface name (string), NULL in case of error.
 * @param[in]       iface: configured interface structure
 */
const char *interface_get_name(struct interface *iface)
{
    if (!iface) {
        return NULL;
    }

    return iface->name;
}

/**
 * @brief Get the interface struct from the index, as specified in the MDK.
 *        Index 0 is for the first interface (aka 'A').
 * @return interface* on success, NULL on error
 */
struct interface* interface_get(uint8_t index)
{
    if ((!interfaces) || (index >= nr_interfaces))
        return NULL;

    return interfaces[index];
}


/**
 * @brief           Return the interface struct from the name
 * @return interface* on success, NULL on error
 */
struct interface* interface_get_by_name(const char *name)
{
    struct interface *iface;
    int i;

    interface_foreach(iface, i) {
      if (!strcmp(iface->name, name)) {
        return iface;
      }
    }

    return NULL;
}

/**
 * @brief           Return the interface struct from the port_id
 * @return interface* on success, NULL on error
 */
struct interface* interface_get_by_portid(uint8_t port_id)
{
    int iface_idx = interface_get_id_by_portid(port_id);

    if (iface_idx <= 0) {
        return NULL;
    }

    return interface_get(iface_idx - 1);
}

/*
 * Interface numbering is defined as it's position in the interface table + 1.
 *
 * By convention, the AP module should be interface number 1.
 */

/**
 * @brief find an intf_id given a portid
 */
int interface_get_id_by_portid(uint8_t port_id) {
    unsigned int i;

    if (port_id == INVALID_PORT) {
        return -ENODEV;
    }

    for (i = 0; i < nr_interfaces; i++) {
        if (interfaces[i]->switch_portid == port_id) {
            return i + 1;
        }
    }

    return -EINVAL;
}

/**
 * @brief find a port_id given an intf_id
 */
int interface_get_portid_by_id(uint8_t intf_id) {
    int portid;

    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }

    portid = interfaces[intf_id - 1]->switch_portid;
    if (portid == INVALID_PORT) {
        return -ENODEV;
    }

    return portid;
}

/**
 * @brief find a dev_id given an intf_id
 */
int interface_get_devid_by_id(uint8_t intf_id) {
    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }

    return interfaces[intf_id - 1]->dev_id;
}

/**
 * @brief set a devid for a given an intf_id
 */
int interface_set_devid_by_id(uint8_t intf_id, uint8_t dev_id) {
    if (!intf_id || intf_id > nr_interfaces) {
        return -EINVAL;
    }
    interfaces[intf_id - 1]->dev_id = dev_id;

    return 0;
}

/**
 * @brief           Return the spring interface struct from the index.
 * @warning         Index 0 is for the first spring interface.
 * @return          Interface structure, NULL in case of error.
 * @param[in]       index: configured interface structure
 */
struct interface* interface_spring_get(uint8_t index)
{
    if ((!interfaces) || (index >= nr_spring_interfaces))
        return NULL;

    return interfaces[nr_interfaces - nr_spring_interfaces + index];
}


/**
 * @brief           Return the number of available interfaces.
 * @return          Number of available interfaces, 0 in case of error.
 */
uint8_t interface_get_count(void)
{
    return nr_interfaces;
}


/**
 * @brief           Return the number of available spring interfaces.
 * @return          Number of available spring interfaces, 0 in case of error.
 */
uint8_t interface_get_spring_count(void)
{
    return nr_spring_interfaces;
}


/**
 * @brief           Return the ADC instance used for this interface
 *                  current measurement.
 * @return          ADC instance, 0 in case of error
 * @param[in]       iface: configured interface structure
 */
uint8_t interface_pm_get_adc(struct interface *iface)
{
    if ((!iface) || (!iface->pm)) {
        return 0;
    }

    return iface->pm->adc;
}


/**
 * @brief           Return the ADC channel used for this interface
 *                  current measurement.
 * @return          ADC channel, 0 in case of error
 * @param[in]       iface: configured interface structure
 */
uint8_t interface_pm_get_chan(struct interface *iface)
{
    if ((!iface) || (!iface->pm)) {
        return 0;
    }

    return iface->pm->chan;
}


/**
 * @brief Store the hotplug/unplug state of the interface, from the
 * port ID.
 *
 * This function is used to bufferize the last state of the interface
 * while the AP is not yet ready to accept hotplug events. This prevents
 * multiple hotplug and hot-unplug events to be sent to the AP when
 * it is ready.
 */
int interface_store_hotplug_state(uint8_t port_id, enum hotplug_state hotplug)
{
    irqstate_t flags;
    int intf_id;

    intf_id = interface_get_id_by_portid(port_id);
    if (intf_id < 0) {
        dbg_error("%s: cannot get interface from portId %u\n" , __func__,
                  port_id);
        return -EINVAL;
    }

    flags = irqsave();
    interfaces[intf_id - 1]->hp_state = hotplug;
    irqrestore(flags);

    return 0;
}


/**
 * @brief Consume the hotplug/unplug state of the interface, from the
 * port ID.
 *
 * This function is used to retrieve and clear the state of the interface
 * in order to generate an event to be sent to the AP.
 */
enum hotplug_state interface_consume_hotplug_state(uint8_t port_id)
{
    enum hotplug_state hp_state;
    int intf_id;
    irqstate_t flags;

    intf_id = interface_get_id_by_portid(port_id);
    if (intf_id < 0) {
        return HOTPLUG_ST_UNKNOWN;
    }

    flags = irqsave();
    hp_state = interfaces[intf_id - 1]->hp_state;
    interfaces[intf_id - 1]->hp_state = HOTPLUG_ST_UNKNOWN;
    irqrestore(flags);

    return hp_state;
}


/**
 * @brief Get the hotplug state of an interface from the DETECT_IN signal
 */
enum hotplug_state interface_get_hotplug_state(struct interface *iface)
{
    bool polarity, active;
    enum hotplug_state hs = HOTPLUG_ST_UNKNOWN;
    irqstate_t flags;

    flags = irqsave();

    if (iface->detect_in.gpio) {
        polarity = iface->detect_in.polarity;
        active = (gpio_get_value(iface->detect_in.gpio) == polarity);
        if (active) {
            hs = HOTPLUG_ST_PLUGGED;
        } else {
            hs = HOTPLUG_ST_UNPLUGGED;
        }
    }

    irqrestore(flags);

    return hs;
}


/**
 * @brief Get the interface struct from the Wake & Detect gpio
 * @return interface* on success, NULL on error
 */
static struct interface* interface_get_by_wd(uint32_t gpio)
{
    int i;
    struct interface *ifc;

    interface_foreach(ifc, i) {
        if (ifc->detect_in.gpio == gpio) {
            return ifc;
        }
    }

    return NULL;
}

static int interface_wd_handler(int irq, void *context);
static void interface_wd_delayed_handler(void *data)
{
    struct wd_data *wd = (struct wd_data *) data;

    interface_wd_handler(wd->gpio, NULL);
}

/* Delayed debounce check */
static int interface_wd_delay_check(struct wd_data *wd, uint32_t delay)
{
    /*
     * If the work is already scheduled, do not schedule another one now.
     * A new one will be scheduled if more debounce is needed.
     */
    if (!work_available(&wd->work)) {
        return 0;
    }

    pm_activity(7);

    /* Schedule the work to run after the debounce timeout */
    return work_queue(HPWORK, &wd->work, interface_wd_delayed_handler, wd,
                      MSEC2TICK(delay));
}

/* Defer notifying the SVC about a finished debounce process until
 * we're out of IRQ context. */
static void interface_wd_delay_notify_svc(struct interface *iface)
{
    int rc;
    struct wd_data *wd = &iface->detect_in;
    /*
     * Just cancel anything left in the queue -- we've already decided
     * the line is stable.
     */
    if (!work_available(&wd->work)) {
        rc = work_cancel(HPWORK, &wd->work);
        /*
         * work_cancel() doesn't fail in the current
         * implementation. And if it did, we'd be dead in the water
         * anyway.
         */
        DEBUGASSERT(!rc);
    }
    /*
     * Run the work right away. The signal is stable; there's no point
     * in waiting.
     */
    rc = work_queue(HPWORK, &wd->work, interface_wd_delayed_handler, wd, 0);
    DEBUGASSERT(!rc);
}

/*
 * Handle an active stable signal as on DB3. The fact that there's
 * only one wake/detect pin to debounce there is assumed.
 *
 * WD as DETECT_IN transition to active
 *
 * - Power ON the interface
 *   Note: If coming back to the active stable state from
 *         the same last stable state after an unstable
 *         transition, power cycle (OFF/ON) the interface.
 *         In that case consecutive hotplug events are
 *         sent to the AP.
 * - Signal HOTPLUG state to the higher layer
 */
static void interface_wd_handle_active_stable(struct interface *iface)
{
    struct wd_data *wd = &iface->detect_in;

    if (up_interrupt_context()) {
        /* Delay handling this transition to work queue context. */
        interface_wd_delay_notify_svc(iface);
        return;
    }

    wd->db_state = WD_ST_ACTIVE_STABLE;
    dbg_verbose("W&D: got stable %s_WD Act (gpio %d)\n",
                iface->name, wd->gpio);

    if (wd->last_state == WD_ST_ACTIVE_STABLE) {
        interface_power_off(iface);
    }
    interface_power_on(iface);
    if (iface->switch_portid != INVALID_PORT) {
        svc_hot_plug(iface->switch_portid);
    }
    /* Save last stable state for power ON/OFF handling */
    wd->last_state = wd->db_state;
}

/*
 * Handle an inactive stable signal as on DB3. The fact that theres
 * only one wake/detect pin to debounce there is assumed.
 *
 * WD as DETECT_IN transition to inactive
 *
 * Power OFF the interface
 * Signal HOTPLUG state to the higher layer
 *
 */
static void interface_wd_handle_inactive_stable(struct interface *iface)
{
    struct wd_data *wd = &iface->detect_in;

    if (up_interrupt_context()) {
        /* Delay handling this transition to work queue context. */
        interface_wd_delay_notify_svc(iface);
        return;
    }

    wd->db_state = WD_ST_INACTIVE_STABLE;
    dbg_verbose("W&D: got stable %s_WD Ina (gpio %d)\n",
                iface->name, wd->gpio);
    interface_power_off(iface);
    if (iface->switch_portid != INVALID_PORT) {
        svc_hot_unplug(iface->switch_portid);
    }
    /* Save last stable state for power ON/OFF handling */
    wd->last_state = wd->db_state;
}

/*
 * Debounce the single WD signal, as on DB3.
 * This handler is also handling the low power mode transitions and
 * wake-ups.
 */
static int interface_debounce_wd(struct interface *iface,
                                 struct wd_data *wd,
                                 bool active)
{
    struct timeval now, diff, timeout_tv = { 0, 0 };
    irqstate_t flags;

    flags = irqsave();

    /* Debounce WD signal to act as detection, which will trigger
     * the power on/off of the interface and hotplug notifications to
     * the AP.
     * Short pulses are filtered out.
     */
    switch (wd->db_state) {
    case WD_ST_INVALID:
    default:
        gettimeofday(&wd->debounce_tv, NULL);
        wd->db_state = active ?
                       WD_ST_ACTIVE_DEBOUNCE : WD_ST_INACTIVE_DEBOUNCE;
        interface_wd_delay_check(wd, (active ?
                                      WD_ACTIVATION_DEBOUNCE_TIME_MS :
                                      WD_INACTIVATION_DEBOUNCE_TIME_MS));
        break;
    case WD_ST_ACTIVE_DEBOUNCE:
        if (active) {
            timeout_tv.tv_usec = WD_ACTIVATION_DEBOUNCE_TIME_MS * 1000;
            /* Signal did not change ... for how long ? */
            gettimeofday(&now, NULL);
            timersub(&now, &wd->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                interface_wd_handle_active_stable(iface);
            } else {
                /* Check for a stable signal after the debounce timeout */
                interface_wd_delay_check(wd, WD_ACTIVATION_DEBOUNCE_TIME_MS);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_INACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd, WD_INACTIVATION_DEBOUNCE_TIME_MS);
        }
        break;
    case WD_ST_INACTIVE_DEBOUNCE:
        if (!active) {
            /* Signal did not change ... for how long ? */
            timeout_tv.tv_usec = WD_INACTIVATION_DEBOUNCE_TIME_MS;
            gettimeofday(&now, NULL);
            timersub(&now, &wd->debounce_tv, &diff);
            if (timercmp(&diff, &timeout_tv, >=)) {
                /* We have a stable signal */
                interface_wd_handle_inactive_stable(iface);
            } else {
                /* Check for a stable signal after the debounce timeout */
                interface_wd_delay_check(wd, WD_INACTIVATION_DEBOUNCE_TIME_MS);
            }
        } else {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_ACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd, WD_ACTIVATION_DEBOUNCE_TIME_MS);
        }
        break;
    case WD_ST_ACTIVE_STABLE:
        if (!active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_INACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd, WD_INACTIVATION_DEBOUNCE_TIME_MS);
        }
        break;
    case WD_ST_INACTIVE_STABLE:
        if (active) {
            /* Signal did change, reset the debounce timer */
            gettimeofday(&wd->debounce_tv, NULL);
            wd->db_state = WD_ST_ACTIVE_DEBOUNCE;
            interface_wd_delay_check(wd, WD_ACTIVATION_DEBOUNCE_TIME_MS);
        }
        break;
    }

    irqrestore(flags);

    return 0;
}

/* Interface Wake & Detect handler */
static int interface_wd_handler(int irq, void *context)
{
    struct interface *iface = NULL;
    struct wd_data *wd;
    bool polarity, active;
    int ret;

    /* Retrieve interface from the GPIO number */
    iface = interface_get_by_wd(irq);
    if (!iface) {
        dbg_error("%s: cannot get interface for pin %d\n", __func__, irq);
        return -ENODEV;
    }

    /* Get signal type, polarity, active state etc. */
    wd = &iface->detect_in;
    polarity = iface->detect_in.polarity;
    active = (gpio_get_value(irq) == polarity);

    dbg_insane("W&D: got %s DETECT_IN %s (gpio %d)\n",
               iface->name,
               active ? "Act" : "Ina",
               irq);

    /* Debounce and handle state changes */
    ret = interface_debounce_wd(iface, wd, active);

    return ret;
}

/*
 * Uninstall handler for Wake & Detect pin
 */
static void interface_uninstall_wd_handler(struct wd_data *wd)
{
    if (wd->gpio) {
        gpio_irq_mask(wd->gpio);
        gpio_irq_attach(wd->gpio, NULL);
    }
}

static void interface_check_unplug_during_wake_out(struct interface *iface)
{
    enum hotplug_state hs = interface_get_hotplug_state(iface);
    switch (hs) {
    case HOTPLUG_ST_PLUGGED:
        return;
    case HOTPLUG_ST_UNKNOWN:
        /* fall through */
    default:
        dbg_warn("%s: %s: invalid or unknown hotplug state %u (gpio %u)\n",
                 __func__, iface->name, hs, iface->detect_in.gpio);
        /* fall through */
    case HOTPLUG_ST_UNPLUGGED:
        /*
         * The interface hotplug state is either invalid (in which
         * case we need to figure out what's going on) or it now reads
         * as unplugged, despite having been plugged before (or we
         * wouldn't have sent wake out).
         *
         * We'd better debounce the interface again. A full debounce
         * is needed to disambiguate the interface being unplugged
         * from something sending a wake out pulse to the SVC when we
         * checked the hotplug state.
         */
        dbg_warn("Possible unplug during wake out!\n");
        iface->detect_in.db_state = WD_ST_INVALID;
        interface_debounce_wd(iface, &iface->detect_in, false);
    }
}

/*
 * Install handler for Wake & Detect pin
 *
 * Other than being called during initialization, it's called again
 * after wake out pulses are performed. However, if the module was
 * forcibly removed during the wake out pulse itself, we'll have
 * missed the interrupt. The check_for_unplug parameter determines
 * whether we need to check for that case here.
 */
static int interface_install_wd_handler(struct interface *iface,
                                        bool check_for_unplug)
{
    struct wd_data *wd = &iface->detect_in;
    if (wd->gpio) {
        gpio_direction_in(wd->gpio);
        gpio_set_pull(wd->gpio, GPIO_PULL_TYPE_PULL_NONE);
        if (check_for_unplug) {
            interface_check_unplug_during_wake_out(iface);
        }
        if (gpio_irq_settriggering(wd->gpio, IRQ_TYPE_EDGE_BOTH) ||
            gpio_irq_attach(wd->gpio, interface_wd_handler) ||
            gpio_irq_unmask(wd->gpio)) {
            dbg_error("Failed to attach Wake & Detect handler for pin %d\n",
                      wd->gpio);
            interface_uninstall_wd_handler(wd);
            return -EINVAL;
        }
    }

    return 0;
}

/**
 * @brief           Return the measurement sign pin GPIO configuration.
 * @return          Measurement sign pin GPIO configuration, 0 in case of error.
 * @param[in]       iface: configured interface structure
 */
uint32_t interface_pm_get_spin(struct interface *iface)
{
    if ((!iface) || (!iface->pm)) {
        return 0;
    }

    return iface->pm->spin;
}

/**
 * @brief Toggle the MOD_RELEASE_ signal for all interfaces with such a GPIO
 *        defined
 * @param delay pulse width in ms (~10ms precision)
 */
void interface_forcibly_eject_all(uint32_t delay)
{
    unsigned int i;
    struct interface *ifc;

    interface_foreach(ifc, i) {
        interface_forcibly_eject(ifc, delay);
    }
}

int interface_forcibly_eject(struct interface *ifc, uint32_t delay)
{
    uint8_t gpio = ifc->release_gpio;
    struct wd_data *wd = &ifc->detect_in;
    bool enable_power = false;

    if (!ifc->ejectable) {
        return -ENOTTY;
    }

    dbg_info("Module %s ejecting: using gpio 0x%02X\n", ifc->name, gpio);

    /*
     * HACK: if there is a module in the slot, but it isn't powered on
     * for some reason (e.g. dummy module), enable power.
     */
    if (gpio_is_valid(wd->gpio)) {
        gpio_direction_in(wd->gpio);
        if ( (gpio_get_value(wd->gpio) == wd->polarity) &&
                (interface_get_power_state(ifc) != ARA_IFACE_PWR_UP) ){
            interface_power_enable(ifc);
            enable_power = true;
        }
    }

    gpio_set_value(gpio, 1);
    usleep(delay * 1000);
    gpio_set_value(gpio, 0);

    if (enable_power) {
        interface_power_disable(ifc);
    }

    return 0;
}

/**
 * @brief Given a table of interfaces, power off all associated
 *        power supplies
 * @param interfaces table of interfaces to initialize
 * @param nr_ints number of interfaces to initialize
 * @param nr_spring_ints number of spring interfaces
 * @return 0 on success, <0 on error
 */
int interface_early_init(struct interface **ints,
                         size_t nr_ints, size_t nr_spring_ints) {
    unsigned int i;
    int rc;
    int fail = 0;
    struct interface *ifc;

    dbg_info("Power off all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;
    nr_spring_interfaces = nr_spring_ints;

    interface_foreach(ifc, i) {
        rc = interface_config(ifc);
        if (rc < 0) {
            dbg_error("Failed to configure interface %s\n", ifc->name);
            fail = 1;
            /* Continue configuring remaining interfaces */
            continue;
        }
    }

    if (fail) {
        return -1;
    }

    /* Let everything settle for a good long while.*/
    up_udelay(POWER_OFF_TIME_IN_US);

    return 0;
}


/**
 * @brief Given a table of interfaces, initialize and enable all associated
 *        power supplies
 * @param interfaces table of interfaces to initialize
 * @param nr_ints number of interfaces to initialize
 * @param nr_spring_ints number of spring interfaces
 * @return 0 on success, <0 on error
 * @sideeffects: leaves interfaces powered off on error.
 */
int interface_init(struct interface **ints,
                   size_t nr_ints, size_t nr_spring_ints) {
    unsigned int i;
    int rc;
    struct interface *ifc;

    dbg_info("Initializing all interfaces\n");

    if (!ints) {
        return -ENODEV;
    }

    interfaces = ints;
    nr_interfaces = nr_ints;
    nr_spring_interfaces = nr_spring_ints;

    interface_foreach(ifc, i) {
        /* Initialize the hotplug state */
        ifc->hp_state = interface_get_hotplug_state(ifc);
        /* Power on/off the interface based on the DETECT_IN signal state */
        switch (ifc->hp_state) {
        case HOTPLUG_ST_PLUGGED:
            /* Port is plugged in, power ON the interface */
            if (interface_power_on(ifc) < 0) {
                dbg_error("Failed to power ON interface %s\n", ifc->name);
            }
            /*
             * SW-3364. Unipro port interrupts need to be enabled initially
             * for the LinkUp and Mailbox IRQs to be signalled by the Switch.
             * Later the port interrupts are enabled and disabled as part of
             * the hotplug/unplug sequence.
             */
            if (switch_port_irq_enable(svc->sw, ifc->switch_portid, true)) {
                dbg_error("Failed to enable port IRQs for interface %s\n",
                          ifc->name);
                return rc;
            }
            break;
        case HOTPLUG_ST_UNPLUGGED:
            /* Port unplugged, power OFF the interface */
            if (interface_power_off(ifc) < 0) {
                dbg_error("Failed to power OFF interface %s\n", ifc->name);
            }
            break;
        case HOTPLUG_ST_UNKNOWN:
        default:
            break;
        }

        /* Install handlers for DETECT_IN signal */
        ifc->detect_in.db_state = WD_ST_INVALID;
        ifc->detect_in.last_state = WD_ST_INVALID;
        rc = interface_install_wd_handler(ifc, false);
        if (rc) {
            return rc;
        }
    }

    return 0;
}


/**
 * @brief Disable all associated power supplies. Must have been previously
 * configured with interface_init()
 */
void interface_exit(void) {
    unsigned int i;
    struct interface *ifc;

    dbg_info("Disabling all interfaces\n");

    if (!interfaces) {
        return;
    }

    /* Uninstall handlers for DETECT_IN signal */
    interface_foreach(ifc, i) {
        interface_uninstall_wd_handler(&ifc->detect_in);
    }

    /* Power off */
    interface_foreach(ifc, i) {
        /*
         * Continue turning off the rest even if this one failed - just ignore
         * the return value of interface_power_off().
         */
        (void)interface_power_off(ifc);
    }

    interfaces = NULL;
    nr_interfaces = 0;
}
