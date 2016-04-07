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

#define DBG_COMP ARADBG_SVC

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/power/pm.h>

#include <apps/greybus-utils/utils.h>

#include <arch/board/board.h>

#include <sys/wait.h>
#include <apps/nsh.h>

#include "string.h"
#include "ara_board.h"
#include <ara_debug.h>
#include "ara_key.h"
#include "interface.h"
#include "tsb_switch.h"
#include "svc.h"
#include "vreg.h"
#include "gb_svc.h"
#include <stm32_pm.h>

#define SVCD_PRIORITY               (40)
#define SVCD_STACK_SIZE             (2048)
#define SVC_PROTOCOL_CPORT_ID       (4)
#define MODULE_CONTROL_CPORT_ID     (0)
#define MODULE_FIRMWARE_CPORT_ID    (1)

static struct svc the_svc;
struct svc *svc = &the_svc;

#define SVC_EVENT_TYPE_READY_AP     0x1
#define SVC_EVENT_TYPE_READY_OTHER  0x2
#define SVC_EVENT_TYPE_HOT_UNPLUG   0x3
#define SVC_EVENT_TYPE_EJECT        0x4

#define INA230_SHUNT_VALUE          2 /* mohm */
#define DEFAULT_CURRENT_LSB         100 /* 100uA current LSB */

struct svc_event_ready_other {
    uint8_t port;
};

struct svc_event_hot_unplug {
    uint8_t port;
};

enum svc_eject_action {
    SVC_EJECT_START,
    SVC_EJECT_COMPLETED,
};

struct svc_event_eject {
    enum svc_eject_action action;
    struct interface *iface;
    uint32_t delay;
};

struct svc_event {
    int type;
    struct list_head events;
    union {
        struct svc_event_ready_other ready_other;
        struct svc_event_hot_unplug hot_unplug;
        struct svc_event_eject eject;
    } data;
};

static struct list_head svc_events;

/* List of interfaces to eject */
struct svc_eject_entry {
    struct interface *iface;
    uint32_t delay;
    struct list_head list;
};
static struct list_head svc_eject_list;

static struct svc_event *svc_event_create(int type) {
    struct svc_event *event;

    event = malloc(sizeof(*event));
    if (!event) {
        return NULL;
    }

    event->type = type;
    list_init(&event->events);
    return event;
}

static inline void svc_event_destroy(struct svc_event *event) {
    list_del(&event->events);
    free(event);
}

static int event_cb(struct tsb_switch_event *ev);
static int event_mailbox(struct tsb_switch_event *ev);
static int event_linkup_ind(struct tsb_switch_event *ev);
static int event_linkup(struct tsb_switch_event *ev);

static struct tsb_switch_event_listener evl = {
    .cb = event_cb,
};

static int event_cb(struct tsb_switch_event *ev) {

    switch (ev->type) {
    case TSB_SWITCH_EVENT_LINKUP_IND:
        event_linkup_ind(ev);
        break;
    case TSB_SWITCH_EVENT_LINKUP:
        event_linkup(ev);
        break;
    case TSB_SWITCH_EVENT_MAILBOX:
        event_mailbox(ev);
        break;
    }
    return 0;
}

static int event_linkup_ind(struct tsb_switch_event *ev) {
    int rc;
    char *linkup_ind_result;
    struct interface *iface;
    bool release_mutex;

    rc = pthread_mutex_lock(&svc->lock);
    if (rc) {
        if (rc == EDEADLK) {
            dbg_error("%s: mutex DEADLOCK detected, fix required\n", __func__);
        } else {
            return rc;
        }
    }
    release_mutex = !rc;

    switch (ev->linkup.val) {
    case TSB_LINKUP_IND_FAIL:
        linkup_ind_result = "fail";
        break;
    case TSB_LINKUP_IND_SUCCESS:
        linkup_ind_result = "success";
        break;
    default:
        linkup_ind_result = "unknown";
        break;
    }
    iface = interface_get_by_portid(ev->linkup.port);

    if (!iface) {
        dbg_error("%s: No interface for portId %u\n", __func__,
                  ev->linkup.port);
        if (release_mutex) {
            pthread_mutex_unlock(&svc->lock);
        }
        return -EINVAL;
    }

    dbg_info("event received: type: %u (LINKUP_IND) port: %u (%s) val: %u (%s)\n",
             ev->type, ev->linkup.port, interface_get_name(iface),
             ev->linkup.val, linkup_ind_result);

    switch (ev->linkup.val) {
    case TSB_LINKUP_IND_SUCCESS:
        /*
         * LinkUpInd success returned from the module.
         * End the WAKEOUT pulse and initiate a single manual LinkUp from
         * the Switch.
         */
        if (!iface->linkup_req_sent) {
            interface_cancel_wakeout_atomic(iface);
            iface->linkup_req_sent = true;
            rc = switch_link_startup(svc->sw, ev->linkup.port);
        }
        break;
    case TSB_LINKUP_IND_FAIL:
        /*
         * LinkUpInd failure returned from the module, do nothing.
         * The module will try again until it succeeds
         */
        iface->linkup_req_sent = false;
        break;
    default:
        dbg_error("%s: Unexpected LinkUpInd value: %u port: %u\n",
                  __func__, ev->linkup.val, ev->linkup.port);
       rc = -EINVAL;
    }

    if (release_mutex) {
        pthread_mutex_unlock(&svc->lock);
    }

    return rc;
}

static int event_linkup(struct tsb_switch_event *ev) {
    int rc;
    char *linkup_result;
    struct interface *iface;
    bool release_mutex;

    rc = pthread_mutex_lock(&svc->lock);
    if (rc) {
        if (rc == EDEADLK) {
            dbg_error("%s: mutex DEADLOCK detected, fix required\n", __func__);
        } else {
            return rc;
        }
    }
    release_mutex = !rc;

    switch (ev->linkup.val) {
    case TSB_LINKUP_FAIL:
        linkup_result = "fail";
        break;
    case TSB_LINKUP_SUCCESS:
        linkup_result = "success";
        break;
    default:
        linkup_result = "unknown";
        break;
    }
    iface = interface_get_by_portid(ev->linkup.port);

    if (!iface) {
        dbg_error("%s: No interface for portId %u\n", __func__,
                  ev->linkup.port);
        if (release_mutex) {
            pthread_mutex_unlock(&svc->lock);
        }
        return -EINVAL;
    }

    dbg_info("event received: type: %u (LINKUP) port: %u (%s) val: %u (%s)\n",
             ev->type, ev->linkup.port, interface_get_name(iface),
             ev->linkup.val, linkup_result);

    switch (ev->linkup.val) {
    case TSB_LINKUP_FAIL:
        interface_cancel_linkup_wd_atomic(iface);
        /* LinkUp failed, retry until max retries count is reached */
        if (iface->linkup_retries >= INTERFACE_MAX_LINKUP_TRIES) {
            interface_power_off_atomic(iface);
            dbg_error("%s: Max LinkUp retry count reached for %s, aborting...\n",
                      __func__, iface->name);
            rc = -ENOLINK;
        } else {
            interface_set_linkup_retries_atomic(iface, iface->linkup_retries+1);
            rc = switch_link_startup(svc->sw, ev->linkup.port);
        }
       break;
    case TSB_LINKUP_SUCCESS:
        interface_cancel_linkup_wd_atomic(iface);
        /* LinkUp succeeded, do nothing. The mailbox handshake follows */
        break;
    default:
        dbg_error("%s: Unexpected LinkUp value: %u port: %u\n",
                  __func__, ev->linkup.val, ev->linkup.port);
        rc = -EINVAL;
    }

    if (release_mutex) {
        pthread_mutex_unlock(&svc->lock);
    }

    return rc;
}

static int event_mailbox(struct tsb_switch_event *ev) {
    struct svc_event *svc_ev;
    struct interface *iface;
    int rc = 0;

    iface = interface_get_by_portid(ev->mbox.port);

    dbg_info("event received: type: %u (MBOX) port: %u (%s) val: %u\n",
             ev->type, ev->mbox.port, interface_get_name(iface), ev->mbox.val);

    pthread_mutex_lock(&svc->lock);

    switch (ev->mbox.val) {
    case TSB_MAIL_READY_AP:
        rc = interface_get_id_by_portid(ev->mbox.port);
        if (rc < 0) {
            dbg_error("Unknown module on port %u: %d\n", ev->mbox.port, rc);
            rc = -ENODEV;
            goto out;
        }

        svc->ap_intf_id = rc;
        break;

    case TSB_MAIL_READY_OTHER:
        svc_ev = svc_event_create(SVC_EVENT_TYPE_READY_OTHER);
        if (!svc_ev) {
            dbg_error("Couldn't create TSB_MAIL_READY_OTHER event\n");
            rc = -ENOMEM;
            goto out;
        }
        svc_ev->data.ready_other.port = ev->mbox.port;
        list_add(&svc_events, &svc_ev->events);
        break;
    default:
        dbg_error("unexpected mailbox value: %u port: %u",
                  ev->mbox.val, ev->mbox.port);
    }

 out:
    pthread_cond_signal(&svc->cv);
    pthread_mutex_unlock(&svc->lock);

    return rc;
}

/**
 * @brief Request to eject an interface
 *
 * This is the public facing function, to be called to request the
 * ejection.
 */
int svc_interface_eject_request(struct interface *iface, uint32_t delay)
{
    bool release_mutex;
    struct svc_event *svc_ev;
    int rc;

    rc = pthread_mutex_lock(&svc->lock);
    if (rc) {
        if (rc == EDEADLK) {
            dbg_error("%s: mutex DEADLOCK detected, fix required\n", __func__);
        } else {
            return rc;
        }
    }
    release_mutex = !rc;

    svc_ev = svc_event_create(SVC_EVENT_TYPE_EJECT);
    if (!svc_ev) {
        dbg_error("Couldn't create SVC_EVENT_TYPE_EJECT event\n");
        rc = -ENOMEM;
    } else {
        svc_ev->data.eject.iface = iface;
        svc_ev->data.eject.action = SVC_EJECT_START;
        svc_ev->data.eject.delay = delay;
        list_add(&svc_events, &svc_ev->events);
        pthread_cond_signal(&svc->cv);
    }

    if (release_mutex) {
        pthread_mutex_unlock(&svc->lock);
    }

    return rc;
}

/**
 * @brief Notify the completion of the interface ejection
 *
 * This is the public facing function
 */
int svc_interface_eject_completion_notify(struct interface *iface)
{
    bool release_mutex;
    struct svc_event *svc_ev;
    int rc;

    rc = pthread_mutex_lock(&svc->lock);
    if (rc) {
        if (rc == EDEADLK) {
            dbg_error("%s: mutex DEADLOCK detected, fix required\n", __func__);
        } else {
            return rc;
        }
    }
    release_mutex = !rc;

    svc_ev = svc_event_create(SVC_EVENT_TYPE_EJECT);
    if (!svc_ev) {
        dbg_error("Couldn't create SVC_EVENT_TYPE_EJECT event\n");
        rc = -ENOMEM;
    } else {
        svc_ev->data.eject.iface = iface;
        svc_ev->data.eject.action = SVC_EJECT_COMPLETED;
        list_add(&svc_events, &svc_ev->events);
        pthread_cond_signal(&svc->cv);
    }

    if (release_mutex) {
        pthread_mutex_unlock(&svc->lock);
    }

    return rc;
}

/**
 * @brief Request to eject all interfaces
 *
 * This is the public facing function, to be called to request the
 * ejections.
 */
int svc_interface_eject_request_all(uint32_t delay)
{
    unsigned int i;
    struct interface *iface;
    int rc;

    interface_foreach(iface, i) {
        rc = svc_interface_eject_request(iface, delay);
        if (rc) {
            return rc;
        }
    }

    return 0;
}

static int svc_event_init(void) {
    list_init(&svc_events);
    switch_event_register_listener(svc->sw, &evl);
    return 0;
}

static struct unipro_driver svc_greybus_driver = {
    .name = "svcd-greybus",
    .rx_handler = greybus_rx_handler,
};

static int svc_listen(unsigned int cport) {
    return unipro_driver_register(&svc_greybus_driver, cport);
}

static struct gb_transport_backend svc_backend = {
    .init   = unipro_init,
    .exit   = unipro_deinit,
    .send   = unipro_send,
    .listen = svc_listen,
    .alloc_buf = malloc,
    .free_buf = free,
};

static int svc_gb_init(void) {
    gb_init(&svc_backend);
    gb_svc_register(SVC_PROTOCOL_CPORT_ID);
    return gb_listen(SVC_PROTOCOL_CPORT_ID);
}

/**
 * @brief "Poke" a bridge's mailbox. The mailbox is a DME register in the
 * vendor-defined space present on Toshiba bridges. This is used as a notification
 * to the bridge that a connection has been established and that it can enable
 * transmission of E2EFC credits.
 *
 * Value 'x' means that cport 'x - 1' is ready. When the bridge responds, it
 * clears its own mailbox to notify the SVC that it has received the
 * notification.
 */
static int svc_mailbox_poke(uint8_t intf_id, uint8_t cport) {
    uint32_t mid, pid, val, retries = 2048;
    uint16_t mbox_ack_attr;
    int rc;
    uint32_t portid = interface_get_portid_by_id(intf_id);

    /*
     * We got interrupted by the bridge via the mailbox.
     * First get the bridge revision and decide which mailbox Ack attribute
     * to use
     */
    rc = switch_dme_peer_get(svc->sw, portid, DME_DDBL1_MANUFACTURERID, 0,
                             &mid);
    if (rc) {
        dbg_error("Failed to read DME_DDBL1_MANUFACTURERID for interface %s: %d\n",
                  interface_get_name(interface_get(intf_id - 1)), rc);
        return rc;
    }
    rc = switch_dme_peer_get(svc->sw, portid, DME_DDBL1_PRODUCTID, 0,
                             &pid);
    if (rc) {
        dbg_error("Failed to read DME_DDBL1_PRODUCTID from port %u: %d\n",
                  intf_id, rc);
        return rc;
    }

    if ((mid == MANUFACTURER_TOSHIBA) &&
        (pid == PRODUCT_ES2_TSB_BRIDGE)) {
        mbox_ack_attr = ES2_MBOX_ACK_ATTR;
    } else if ((mid == MANUFACTURER_TOSHIBA) &&
               (pid == PRODUCT_ES3_TSB_APBRIDGE ||
                pid == PRODUCT_ES3_TSB_GPBRIDGE)) {
        mbox_ack_attr = ES3_MBOX_ACK_ATTR;
    } else {
        dbg_error("Unknown manufacturer (0x%x) and product (0x%x) IDs\n",
                  mid, pid);
        return ERROR;
    }

    /* Ack (= reset to 0) the reception to the bridge */
    rc = switch_dme_peer_set(svc->sw, portid, mbox_ack_attr, 0,
                             TSB_MAIL_RESET);
    if (rc) {
        dbg_error("Failed to re-ack mbox of intf %u: %d (attr=0x%x)\n",
                  intf_id, rc, mbox_ack_attr);
        return rc;
    }

    /*
     * Notify the bridge via the mailbox. This generates an IRQ on the bridge
     * side
     */
    rc = switch_dme_peer_set(svc->sw, portid,
                             TSB_MAILBOX, 0, cport + 1);
    if (rc) {
        dbg_error("Failed to notify intf %u: %d\n", intf_id, rc);
        return rc;
    }

    /* Wait the Ack from the bridge */
    do {
        rc = switch_dme_peer_get(svc->sw, portid, mbox_ack_attr, 0, &val);
        if (rc) {
            dbg_error("Failed to poll MBOX_ACK_ATTR (0x%x) on intf %u: %d\n",
                      mbox_ack_attr, intf_id, rc);
            return rc;
        }
    } while ((uint16_t)val != (uint16_t)(cport + 1) && --retries > 0);

    if (!retries) {
        dbg_error("MBOX_ACK_ATTR (0x%x) poll on intf %u timeout: 0x%x != 0x%x\n",
                  mbox_ack_attr, intf_id, (uint16_t)(cport + 1),
                  (uint16_t)val);
        return -ETIMEDOUT;
    }

    return 0;
}

/**
 * @brief Eject a device given an interface id
 */
int svc_intf_eject(uint8_t intf_id)
{
    struct interface *iface;

    iface = interface_get(intf_id - 1);
    if (!iface) {
        return -EINVAL;
    }

    return svc_interface_eject_request(iface, MOD_RELEASE_PULSE_WIDTH);
}

/**
 * @brief Assign a device id given an interface id
 */
int svc_intf_device_id(uint8_t intf_id, uint8_t dev_id) {
    struct tsb_switch *sw = svc->sw;
    int rc;

    rc = switch_if_dev_id_set(sw, interface_get_portid_by_id(intf_id), dev_id);
    if (rc) {
        return rc;
    }

    return interface_set_devid_by_id_atomic(intf_id, dev_id);
}

/**
 * @brief Create a UniPro connection
 */
int svc_connection_create(uint8_t intf1_id, uint16_t cport1_id,
                          uint8_t intf2_id, uint16_t cport2_id, u8 tc, u8 flags) {
    struct tsb_switch *sw = svc->sw;
    struct unipro_connection c;
    int rc;

    memset(&c, 0x0, sizeof c);

    c.port_id0      = interface_get_portid_by_id(intf1_id);
    c.device_id0    = interface_get_devid_by_id(intf1_id);
    c.cport_id0     = cport1_id;

    c.port_id1      = interface_get_portid_by_id(intf2_id);
    c.device_id1    = interface_get_devid_by_id(intf2_id);
    c.cport_id1     = cport2_id;

    c.tc            = tc;
    c.flags         = flags;

    rc = switch_connection_create(sw, &c);
    if (rc) {
        return rc;
    }

    /*
     * Let's connect the ap side first: SW-1231
     */
    if (intf1_id == svc->ap_intf_id) {
        rc = switch_cport_connect(sw, c.port_id0, c.cport_id0);
        if (rc) {
            dbg_error("Failed to enable connection to [p=%u,c=%u].\n",
                      c.port_id0, c.cport_id0);
            return rc;
        }
    }

    rc = switch_cport_connect(sw, c.port_id1, c.cport_id1);
    if (rc) {
        dbg_error("Failed to enable connection to [p=%u,c=%u].\n", c.port_id1,
                  c.cport_id1);
        return rc;
    }

    if (intf1_id != svc->ap_intf_id) {
        rc = switch_cport_connect(sw, c.port_id0, c.cport_id0);
        if (rc) {
            dbg_error("Failed to enable connection to [p=%u,c=%u].\n",
                      c.port_id0, c.cport_id0);
            return rc;
        }
    }

    /*
     * Poke bridge mailboxes.
     * @jira{ENG-376}
     *
     * Poke only CPort 0 and 1 of modules in order to not break the support
     * of bridges ES3's bootrom.
     */
    if (intf1_id != svc->ap_intf_id &&
            (cport1_id == MODULE_CONTROL_CPORT_ID ||
             cport1_id == MODULE_FIRMWARE_CPORT_ID)) {
        rc = svc_mailbox_poke(intf1_id, cport1_id);
        if (rc) {
            dbg_error("Failed to notify intf %u\n", intf1_id);
            return rc;
        }
    }

    if (intf2_id != svc->ap_intf_id &&
            (cport2_id == MODULE_CONTROL_CPORT_ID ||
             cport2_id == MODULE_FIRMWARE_CPORT_ID)) {
        rc = svc_mailbox_poke(intf2_id, cport2_id);
        if (rc) {
            dbg_error("Failed to notify intf %u\n", intf2_id);
            return rc;
        }
    }

    return 0;
}

/**
 * @brief Retrieve a peer DME attribute value
 */
int svc_dme_peer_get(uint8_t intf_id, uint16_t attr, uint16_t selector,
                     uint16_t *result_code, uint32_t *value) {
    int rc;
    int portid;

    if (!result_code || !value) {
        return -EINVAL;
    }

    portid = interface_get_portid_by_id(intf_id);
    if (portid < 0) {
        return -EINVAL;
    }

    rc = switch_dme_peer_get(svc->sw, portid, attr, selector, value);
    *result_code = rc;
    if (rc) {
        dbg_error("Failed to retrieve DME peer attribute [p=%d,a=%u,s=%u,rc=%d]\n",
                  portid, attr, selector, rc);
        return rc;
    }

    return 0;
}

/**
 * @brief Update a peer DME attribute value
 */
int svc_dme_peer_set(uint8_t intf_id, uint16_t attr, uint16_t selector,
                     uint32_t value, uint16_t* result_code) {
    int rc;
    int portid;

    if(!result_code) {
        return -EINVAL;
    }

    portid = interface_get_portid_by_id(intf_id);
    if (portid < 0) {
        return -EINVAL;
    }

    rc = switch_dme_peer_set(svc->sw, portid, attr, selector, value);
    *result_code = rc;
    if (rc) {
        dbg_error("Failed to update DME peer attribute [p=%d,a=%u,s=%u]\n",
                  portid, attr, selector);
        return rc;
    }

    return 0;
}

/*
 * @brief Destroy a UniPro connection
 */
int svc_connection_destroy(uint8_t intf1_id, uint16_t cport1_id,
                           uint8_t intf2_id, uint16_t cport2_id)
{
    struct tsb_switch *sw = svc->sw;
    struct unipro_connection c = {0};

    c.port_id0      = interface_get_portid_by_id(intf1_id);
    c.device_id0    = interface_get_devid_by_id(intf1_id);
    c.cport_id0     = cport1_id;

    c.port_id1      = interface_get_portid_by_id(intf2_id);
    c.device_id1    = interface_get_devid_by_id(intf2_id);
    c.cport_id1     = cport2_id;

    return switch_connection_destroy(sw, &c);
}

/**
 * @brief Create a bidirectional route through the switch
 */
int svc_route_create(uint8_t intf1_id, uint8_t dev1_id,
                     uint8_t intf2_id, uint8_t dev2_id) {
    struct tsb_switch *sw = svc->sw;
    int rc;
    int port1_id, port2_id;

    port1_id = interface_get_portid_by_id(intf1_id);
    port2_id = interface_get_portid_by_id(intf2_id);
    if (port1_id < 0 || port2_id < 0) {
        return -EINVAL;
    }

    rc = switch_setup_routing_table(sw,
                                    dev1_id,
                                    port1_id,
                                    dev2_id,
                                    port2_id);
    if (rc) {
        dbg_error("Failed to create route [p=%d,d=%d]<->[p=%d,d=%d]\n",
                  port1_id, dev1_id, port2_id, dev2_id);
        return rc;
    }

    return 0;
}

/**
 * @brief Destroy a bidirectional route through the switch
 */
int svc_route_destroy(uint8_t intf1_id, uint8_t intf2_id) {
    struct tsb_switch *sw = svc->sw;
    int rc;
    int port1_id, port2_id;
    int dev1_id, dev2_id;

    port1_id = interface_get_portid_by_id(intf1_id);
    port2_id = interface_get_portid_by_id(intf2_id);
    if (port1_id < 0 || port2_id < 0) {
        return -EINVAL;
    }
    dev1_id = interface_get_devid_by_id(intf1_id);
    dev2_id = interface_get_devid_by_id(intf2_id);
    if (dev1_id < 0 || dev2_id < 0) {
        return -EINVAL;
    }

    rc = switch_invalidate_routing_table(sw,
                                         dev1_id,
                                         port1_id,
                                         dev2_id,
                                         port2_id);
    if (rc) {
        dbg_error("Failed to destroy route [p=%d,d=%d]<->[p=%d,d=%d]\n",
                  port1_id, dev1_id, port2_id, dev2_id);
        return rc;
    }

    return 0;
}

/**
 * @brief Configure the power mode of an interface
 */
int svc_intf_set_power_mode(uint8_t intf_id, struct unipro_link_cfg *cfg)
{
    struct tsb_switch *sw = svc->sw;
    int port_id;

    port_id = interface_get_portid_by_id(intf_id);
    if (port_id < 0)
        return -EINVAL;

    return switch_configure_link(sw, port_id, cfg, NULL);
}

/**
 * @brief Handle AP module boot
 */
static int svc_handle_ap(void) {
    struct unipro_connection svc_conn;
    uint8_t ap_port_id = interface_get_portid_by_id(svc->ap_intf_id);
    int rc;

    dbg_info("Creating initial SVC connection\n");

    /* Assign a device ID. The AP is always GB_SVC_DEVICE_ID = 1 */
    rc = svc_intf_device_id(svc->ap_intf_id, GB_SVC_DEVICE_ID);
    if (rc) {
        dbg_error("Failed to assign device id to AP: %d\n", rc);
    }

    rc = switch_setup_routing_table(svc->sw, SWITCH_DEVICE_ID, SWITCH_PORT_ID,
                                    GB_SVC_DEVICE_ID, ap_port_id);
    if (rc) {
        dbg_error("Failed to set initial SVC route: %d\n", rc);
    }

    svc_conn.port_id0      = SWITCH_PORT_ID;
    svc_conn.device_id0    = SWITCH_DEVICE_ID;
    svc_conn.cport_id0     = SVC_PROTOCOL_CPORT_ID;
    svc_conn.port_id1      = ap_port_id;
    svc_conn.device_id1    = GB_SVC_DEVICE_ID;
    svc_conn.cport_id1     = GB_SVC_CPORT_ID;
    svc_conn.tc            = 0;
    svc_conn.flags         = CPORT_FLAGS_E2EFC | CPORT_FLAGS_CSD_N | CPORT_FLAGS_CSV_N;
    rc = switch_connection_create(svc->sw, &svc_conn);
    if (rc) {
        dbg_error("Failed to create initial SVC connection: %d\n", rc);
    }

    /*
     * Enable connection to AP
     */
    rc = switch_cport_connect(svc->sw, svc_conn.port_id0, svc_conn.cport_id0);
    if (rc) {
        dbg_error("Failed to enable connection to [p=%u,c=%u].\n",
                  svc_conn.port_id0, svc_conn.cport_id0);
        return rc;
    }

    rc = switch_cport_connect(svc->sw, svc_conn.port_id1, svc_conn.cport_id1);
    if (rc) {
        dbg_error("Failed to enable connection to [p=%u,c=%u].\n",
                  svc_conn.port_id1, svc_conn.cport_id1);
        return rc;
    }

    /*
     * Now turn on E2EFC on the switch so that it can transmit FCTs.
     */
    rc = switch_fct_enable(svc->sw);
    if (rc) {
        dbg_error("Failed to enable FCT on switch.\n");
        return rc;
    }

    /*
     * Now start the SVC protocol handshake.
     */
    rc = gb_svc_protocol_version();
    if (rc) {
        return rc;
    }

    /*
     * Tell the AP what kind of endo it's in, and the AP's intf_id.
     */
    return gb_svc_hello(svc->ap_intf_id);
}

static int svc_handle_hot_unplug(uint8_t portid) {
    int intf_id;

    dbg_info("Hot_unplug event received for port %u (%s)\n",
             portid, interface_get_name(interface_get_by_portid(portid)));
    intf_id = interface_get_id_by_portid(portid);
    if (intf_id < 0) {
        return intf_id;
    }

    return gb_svc_intf_hot_unplug(intf_id);
}

static int svc_get_module_id(uint32_t portid, uint32_t ddbl1_prod_id,
                             uint32_t *ara_vend_id, uint32_t *ara_prod_id)
{
    int rc;

    if (!ara_vend_id || !ara_prod_id) {
        return -EINVAL;
    }

    switch (ddbl1_prod_id) {
    case PRODUCT_ES3_TSB_APBRIDGE:
    case PRODUCT_ES3_TSB_GPBRIDGE:
        rc = switch_dme_peer_get(svc->sw, portid, TSB_ARA_VID, 0,
                                 ara_vend_id);
        if (rc) {
            dbg_error("Failed to read vid: %d\n", rc);
            return rc;
        }

        rc = switch_dme_peer_get(svc->sw, portid, TSB_ARA_PID, 0,
                                 ara_prod_id);
        if (rc) {
            dbg_error("Failed to read pid: %d\n", rc);
            return rc;
        }
        break;
    /*
     * Module serial number, Ara vendor id and product ID attributes are not
     * via DME on ES2 silicon.
     */
    case PRODUCT_ES2_TSB_BRIDGE:
    default:
        *ara_vend_id = 0;
        *ara_prod_id = 0;
        break;
    }

    return 0;
}

static int svc_handle_module_ready(uint8_t portid) {
    int rc, intf_id;
    uint32_t ddbl1_mfr_id, ddbl1_prod_id, ara_vend_id, ara_prod_id;
    uint64_t serial_number;

    dbg_info("Hotplug event received for port %u (%s)\n",
             portid, interface_get_name(interface_get_by_portid(portid)));
    intf_id = interface_get_id_by_portid(portid);
    if (intf_id < 0) {
        return intf_id;
    }

    rc = switch_dme_peer_get(svc->sw, portid, DME_DDBL1_MANUFACTURERID, 0,
                             &ddbl1_mfr_id);
    if (rc) {
        dbg_error("Failed to read manufacturer id: %d\n", rc);
        return rc;
    }

    rc = switch_dme_peer_get(svc->sw, portid, DME_DDBL1_PRODUCTID, 0,
                             &ddbl1_prod_id);
    if (rc) {
        dbg_error("Failed to read product id: %d\n", rc);
        return rc;
    }

    rc = svc_get_module_id(portid, ddbl1_prod_id, &ara_vend_id, &ara_prod_id);
    if (rc) {
        return rc;
    }
    serial_number = 0x0000000000000000;

    return gb_svc_intf_hotplug(intf_id, ddbl1_mfr_id, ddbl1_prod_id,
                               ara_vend_id, ara_prod_id, serial_number);
}

/**
 * @brief Send hot_unplug event to the AP
 */
int svc_hot_unplug(uint8_t portid, bool lock_interface)
{
    bool release_mutex;
    struct svc_event *svc_ev;
    int rc;

    rc = pthread_mutex_lock(&svc->lock);
    if (rc) {
        if (rc == EDEADLK) {
            dbg_error("%s: mutex DEADLOCK detected, fix required\n", __func__);
        } else {
            return rc;
        }
    }
    release_mutex = !rc;

    if (svc->ap_initialized) {
        /*
         * If AP is ready, generate an event to send
         *
         * Filter out AP hot_unplug
         */
        if (interface_get_portid_by_id(svc->ap_intf_id) == portid) {
            goto out;
        }
        svc_ev = svc_event_create(SVC_EVENT_TYPE_HOT_UNPLUG);
        if (!svc_ev) {
            dbg_error("Couldn't create SVC_EVENT_TYPE_HOT_UNPLUG event\n");
            rc = -ENOMEM;
        } else {
            svc_ev->data.hot_unplug.port = portid;
            list_add(&svc_events, &svc_ev->events);
            pthread_cond_signal(&svc->cv);
        }
    }

out:
    if (release_mutex) {
        pthread_mutex_unlock(&svc->lock);
    }

    return rc;
}

static int svc_handle_eject(struct interface *iface,
                            enum svc_eject_action action,
                            uint32_t delay)
{
    struct svc_eject_entry *entry;
    struct list_head *iter, *iter_next;
    int rc = 0;

    if (!iface) {
        dbg_error("%s called with NULL interface\n", __func__);
        return -ENODEV;
    }

    switch (action) {
    case SVC_EJECT_START:
        /*
         * New interface to eject, add the entry to the list.
         * If the list was empty start the ejection.
         */
        dbg_verbose("Eject request for interface %s\n", iface->name);
        if (list_is_empty(&svc_eject_list)) {
            rc = interface_forcibly_eject_atomic(iface, delay);
            /* If ejection fails do not add the entry to the list */
            if (rc) {
                break;
            }
        }
        entry = malloc(sizeof(struct svc_eject_entry));
        if (!entry) {
            return -ENOMEM;
        }
        entry->iface = iface;
        entry->delay = delay;
        list_init(&entry->list);
        list_add(&svc_eject_list, &entry->list);
        break;
    case SVC_EJECT_COMPLETED:
        dbg_verbose("Done ejecting interface %s\n", iface->name);
        /*
         * Last interface eject is completed, remove the entry and
         * start next one.
         */
        list_foreach_safe(&svc_eject_list, iter, iter_next) {
            entry = list_entry(iter, struct svc_eject_entry, list);
            if (entry->iface == iface) {
                list_del(iter);
                free(entry);
                break;
            }
        }
        /* If list is empty we are done */
        if (list_is_empty(&svc_eject_list)) {
            dbg_verbose("Done ejecting interfaces\n");
            break;
        }
        /* Start ejection of the next interface from the list */
        list_foreach_safe(&svc_eject_list, iter, iter_next) {
            entry = list_entry(iter, struct svc_eject_entry, list);
            dbg_verbose("Ejecting next interface %s\n", entry->iface->name);
            rc = interface_forcibly_eject_atomic(entry->iface, entry->delay);
            /* If ejection fails remove the entry from the list */
            if (rc) {
                list_del(iter);
                free(entry);
            }
            /* We are done with one interface from the list */
            break;
        }
        break;
    default:
        dbg_error("Unknonw eject action value %d\n", action);
        break;
    }

    return rc;
}

/**
 * @brief Main event loop processing routine
 */
static int svc_handle_events(void) {
    struct list_head *node, *next;
    struct svc_event *event;

    list_foreach_safe(&svc_events, node, next) {
        event = list_entry(node, struct svc_event, events);
        switch (event->type) {
        case SVC_EVENT_TYPE_READY_OTHER:
            svc_handle_module_ready(event->data.ready_other.port);
            break;
        case SVC_EVENT_TYPE_HOT_UNPLUG:
            svc_handle_hot_unplug(event->data.hot_unplug.port);
            break;
        case SVC_EVENT_TYPE_EJECT:
            svc_handle_eject(event->data.eject.iface,
                             event->data.eject.action,
                             event->data.eject.delay);
            break;
        default:
            dbg_error("Unknown event %d\n", event->type);
        }

        svc_event_destroy(event);
    }

    return 0;
}

/*
 * EMERGENCY MODULE RELEASE: The svc event queue is only processed after
 * ap_initialized is true.
 */
static int svc_ara_key_longpress_callback(void *priv)
{
    return svc_interface_eject_request_all(MOD_RELEASE_PULSE_WIDTH);
}

/* state helpers */
#define svcd_state_running() (svc->state == SVC_STATE_RUNNING)
#define svcd_state_stopped() (svc->state == SVC_STATE_STOPPED)
static inline void svcd_set_state(enum svc_state state) {
    svc->state = state;
}

static int svcd_startup(void) {
    struct ara_board_info *info;
    struct tsb_switch *sw;
    int rc;

    /*
     * Board-specific initialization, all boards must define this.
     */
    info = ara_board_init();
    if (!info) {
        dbg_error("%s: No board information provided.\n", __func__);
        goto error0;
    }
    svc->board_info = info;
    rc = interface_early_init(info->interfaces, info->nr_interfaces,
                              info->nr_spring_interfaces, info->vlatch_vdd,
                              info->latch_ilim, info->mod_sense);
    if (rc < 0) {
        dbg_error("%s: Failed to power off interfaces\n", __func__);
        goto error0;
    }

    /* Init Switch */
    sw = switch_init(&info->sw_data);
    if (!sw) {
        dbg_error("%s: Failed to initialize switch.\n", __func__);
        goto error1;
    }
    svc->sw = sw;

    /* Enable the switch IRQ */
    rc = switch_irq_enable(sw, true);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error2;
    }

    /*
     * Initialize event system
     * Done before the interfaces bring-up, so that events can be handled
     */
    rc = svc_event_init();
    if (rc) {
        goto error2;
    }

    list_init(&svc_eject_list);

    /* Power on all provided interfaces */
    if (!info->interfaces) {
        dbg_error("%s: No interface information provided\n", __func__);
        goto error2;
    }
    rc = interface_init(info->interfaces, info->nr_interfaces,
                        info->nr_spring_interfaces, info->vlatch_vdd,
                        info->latch_ilim, info->mod_sense);
    if (rc < 0) {
        dbg_error("%s: Failed to initialize interfaces\n", __func__);
        goto error2;
    }

    /* Register svc protocol greybus driver*/
    rc = svc_gb_init();
    if (rc) {
        dbg_error("%s: Failed to initialize SVC protocol\n", __func__);
        goto error3;
    }

    /*
     * enable the ARA key IRQ
     */
    rc = ara_key_enable(info, svc_ara_key_longpress_callback, true);

    return 0;

error3:
    interface_exit();
error2:
    switch_exit(sw);
    svc->sw = NULL;
error1:
    ara_board_exit();
error0:
    return -1;
}

static int svcd_cleanup(void) {
    struct list_head *node, *next;

    gb_deinit();

    interface_exit();

    switch_exit(svc->sw);
    svc->sw = NULL;

    ara_board_exit();
    svc->board_info = NULL;

    list_foreach_safe(&svc_events, node, next) {
        svc_event_destroy(list_entry(node, struct svc_event, events));
    }

    list_foreach_safe(&svc_eject_list, node, next) {
        list_del(node);
        free(node);
    }

    return 0;
}


static int svcd_main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    int rc = 0;

    pthread_mutex_lock(&svc->lock);
    rc = svcd_startup();
    if (rc < 0) {
        goto done;
    }

    /* The wake-up sources are installed. Allow idle now */
    atomic_init(&svc->allow_idle, 1);
    while (!svc->stop) {
        pthread_cond_wait(&svc->cv, &svc->lock);
        /* check to see if we were told to stop */
        if (svc->stop) {
            dbg_verbose("svc stop requested\n");
            break;
        }

        if (svc->ap_intf_id && !svc->ap_initialized) {
            if (svc_handle_ap()) {
                break;
            }

            dbg_info("AP initialized on interface %u\n", svc->ap_intf_id);
            svc->ap_initialized = 1;
        }

        /* If AP is found let's handle the queued svc_events */
        if (svc->ap_initialized) {
            svc_handle_events();
        }
    };

    rc = svcd_cleanup();

done:
    svcd_set_state(SVC_STATE_STOPPED);
    pthread_mutex_unlock(&svc->lock);
    return rc;
}

#ifdef CONFIG_PM
static int svcd_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate)
{
    /*
     * Allow low power mode in idle if svcd started correctly and
     * installed the wake-up events
     */
    int rc;

    switch (pmstate) {
    case PM_NORMAL:
        rc = OK;
        break;
    default:
        rc = atomic_get(&svc->allow_idle) ? OK : ERROR;
        break;
    }

    return rc;
}

static  struct pm_callback_s svcd_cb =
{
    .prepare = svcd_prepare,
};
#endif

/*
 * System entrypoint. CONFIG_USER_ENTRYPOINT should point to this function.
 */
int svc_init(int argc, char **argv) {
    int rc;

    svc->sw = NULL;
    svc->board_info = NULL;
    svc->svcd_pid = 0;
    svc->stop = 0;
    atomic_init(&svc->allow_idle, 0);
    pthread_mutex_init(&svc->lock, NULL);
    pthread_cond_init(&svc->cv, NULL);
    svcd_set_state(SVC_STATE_STOPPED);

    /* Register to receive power management callbacks */
#ifdef CONFIG_PM
    rc = pm_register(&svcd_cb);
    if (rc) {
        dbg_error("%s: could not register PM callback: %d, aborting\n",
                  __func__, rc);
        return rc;
    }
#endif

    rc = svcd_start();
    if (rc) {
        return rc;
    }

    /*
     * Now start the shell.
     */
    return nsh_main(argc, argv);
}

int svcd_start(void) {
    int rc;

    pthread_mutex_lock(&svc->lock);
    dbg_info("starting svcd\n");
    if (!svcd_state_stopped()) {
        dbg_info("svcd already started\n");
        pthread_mutex_unlock(&svc->lock);
        return -EBUSY;
    }

    svc->ap_initialized = 0;
    svc->ap_intf_id = 0;

    rc = task_create("svcd", SVCD_PRIORITY, SVCD_STACK_SIZE, svcd_main, NULL);
    if (rc == ERROR) {
        dbg_error("failed to start svcd\n");
        pthread_mutex_unlock(&svc->lock);
        return rc;
    }
    svc->svcd_pid = rc;

    svc->stop = 0;
    svcd_set_state(SVC_STATE_RUNNING);
    pthread_mutex_unlock(&svc->lock);

    return 0;
}

void svcd_stop(void) {
    int status;
    int rc;
    pid_t pid_tmp;

    pthread_mutex_lock(&svc->lock);
    dbg_verbose("stopping svcd\n");

    pid_tmp = svc->svcd_pid;

    if (!svcd_state_running()) {
        dbg_info("svcd not running\n");
        pthread_mutex_unlock(&svc->lock);
        return;
    }

    /* signal main thread to stop */
    svc->stop = 1;
    atomic_init(&svc->allow_idle, 0);
    pthread_cond_signal(&svc->cv);
    pthread_mutex_unlock(&svc->lock);

    /* wait for the svcd to stop */
    rc = waitpid(pid_tmp, &status, 0);
    if (rc != pid_tmp) {
        dbg_error("failed to stop svcd\n");
    } else {
        dbg_info("svcd stopped\n");
    }
}

int svc_connect_interfaces(struct interface *iface1, uint16_t cportid1,
                           struct interface *iface2, uint16_t cportid2,
                           uint8_t tc, uint8_t flags) {
    int rc;
    uint8_t devids[2];
    struct tsb_switch *sw = svc->sw;
    struct unipro_connection con;

    if (!iface1 || !iface2) {
        return -EINVAL;
    }

    pthread_mutex_lock(&svc->lock);

    /* Retrieve the interface structures and device IDs for the interfaces. */
    rc = switch_if_dev_id_get(sw, iface1->switch_portid, &devids[0]);
    if (rc) {
        goto error_exit;
    }

    rc = switch_if_dev_id_get(sw, iface2->switch_portid, &devids[1]);
    if (rc) {
        goto error_exit;
    }

    /* Create the route between the two devices. */
    rc = switch_setup_routing_table(sw,
                                    devids[0], iface1->switch_portid,
                                    devids[1], iface2->switch_portid);
    if (rc) {
        dbg_error("Failed to create route: [d=%u,p=%u]<->[d=%u,p=%u]\n",
                  devids[0], iface1->switch_portid,
                  devids[1], iface2->switch_portid);
        goto error_exit;
    }
    /* Create the connection between the two devices. */
    con.port_id0   = iface1->switch_portid;
    con.device_id0 = devids[0];
    con.cport_id0  = cportid1;
    con.port_id1   = iface2->switch_portid;
    con.device_id1 = devids[1];
    con.cport_id1  = cportid2;
    con.tc         = tc;
    con.flags      = flags;
    rc = switch_connection_create(sw, &con);
    if (rc) {
        dbg_error("Failed to create [p=%u,d=%u,c=%u]<->[p=%u,d=%u,c=%u] TC: %u Flags: 0x%x\n",
                  con.port_id0, con.device_id0, con.cport_id0,
                  con.port_id1, con.device_id1, con.cport_id1,
                  con.tc, con.flags);
        goto error_exit;
    }

    /*
     * Let's connect the ap side first: SW-1231
     */
    if (interface_get_id_by_portid(iface1->switch_portid) == svc->ap_intf_id) {
        rc = switch_cport_connect(sw, con.port_id0, con.cport_id0);
        if (rc) {
            dbg_error("Failed to enable connection to [p=%u,c=%u].\n",
                      con.port_id0, con.cport_id0);
            goto error_exit;
        }
    }

    rc = switch_cport_connect(sw, con.port_id1, con.cport_id1);
    if (rc) {
        dbg_error("Failed to enable connection to [p=%u,c=%u].\n", con.port_id1,
                  con.cport_id1);
        goto error_exit;
    }

    if (interface_get_id_by_portid(iface1->switch_portid) != svc->ap_intf_id) {
        rc = switch_cport_connect(sw, con.port_id0, con.cport_id0);
        if (rc) {
            dbg_error("Failed to enable connection to [p=%u,c=%u].\n",
                      con.port_id0, con.cport_id0);
            goto error_exit;
        }
    }

 error_exit:
    pthread_mutex_unlock(&svc->lock);
    return rc;
}

static int power_down_ina231(void)
{
    int i;
    int j;
    pwrmon_board_info *board_info;
    int pwrmon_num_devs;

    board_info = board_get_pwrmon_info();
    if (!board_info) {
        dbg_error("error at getting board info\n");
        return -ENODEV;
    }

    pwrmon_init(DEFAULT_CURRENT_LSB, ina230_ct_1_1ms, ina230_avg_count_64, &pwrmon_num_devs);

    for (i = 0; i < board_info->num_devs; i++) {
        for (j = 0; j < board_info->devs[i].num_rails; j++) {
            pwrmon_rail *rails = pwrmon_init_rail(i,j);
            /* deinit shuts down INA231 by default */
            pwrmon_deinit_rail(rails);
        }
    }

    /* Reset i2c selection gpios and relese i2c resource */
    pwrmon_deinit();

    return 0;
}

int svcd_power_down(void) {
    int rc;

    dbg_info("powering down INA231 chips...\n");
    rc = power_down_ina231();
    dbg_info("powering down all modules...\n");
    interface_exit();
    dbg_info("powering down switch...\n");
    switch_exit(svc->sw);
    dbg_info("putting svc into stm32_standby mode...\n");
    svcd_stop();
    stm32_pmstandby();
    return rc;
}
