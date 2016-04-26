/*
 * Copyright (c) 2014-2015 Google Inc.
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

#include <nuttx/config.h>

#include <errno.h>
#include <stdio.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>
#include <arch/board/apbridgea_gadget.h>
#include <apps/greybus-utils/utils.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

/*
 * TODO
 * Already defined in tsb_unipro.c
 * Move them to tsb_unipro.h
 */

#define CPORTID_CDSI0    (16)
#define CPORTID_CDSI1    (17)

#define TIMEOUT_IN_MS           300
#define ONE_SEC_IN_MSEC         1000
#define RESET_TIMEOUT_DELAY (TIMEOUT_IN_MS * CLOCKS_PER_SEC) / ONE_SEC_IN_MSEC

static sem_t linkup_sem;
static pthread_t g_apbridge_thread;

struct cport_reset_priv {
    struct usbdev_s *dev;
    struct wdog_s timeout_wd;
    atomic_t refcount;
};

static int release_buffer(int status, const void *buf, void *priv)
{
    return usb_release_buffer(priv, buf);
}

int recv_from_unipro(unsigned int cportid, void *buf, size_t len)
{
    /*
     * FIXME: Remove when UniPro driver provides the actual buffer length.
     */
    len = gb_packet_size(buf);

    gb_dump(buf, len);

    if (len < sizeof(struct gb_operation_hdr)) {
       lowsyslog("%s: Packet smaller than Greybus header\n", __func__);
        return -EPROTO;
    }

    if (len != gb_packet_size(buf)) {
        lowsyslog("%s: Invalid message size: %u != %u\n",
                  __func__, len, gb_packet_size(buf));
        return -EPROTO;
    }

    return rx_transfer(get_apbridge_dev(), cportid, buf, len);
}

int unipro_tx_transfer(unsigned int cportid,
                       void *buf, size_t len, void *priv)
{
    gb_dump(buf, len);

    if (len < sizeof(struct gb_operation_hdr)) {
        lowsyslog("%s: Packet smaller than Greybus header\n", __func__);
        return -EPROTO;
    }

    if (len != gb_packet_size(buf)) {
        lowsyslog("%s: Invalid message size: %u != %u\n",
                  __func__, len, gb_packet_size(buf));
        return -EPROTO;
    }

    return unipro_send_async(cportid, buf, len, release_buffer, priv);
}

static void cport_reset_cb(unsigned int cportid, void *data)
{
    struct cport_reset_priv *priv = data;

    if (atomic_dec(&priv->refcount)) {
        vendor_request_deferred_submit(priv->dev, OK);
    } else {
        wd_delete(&priv->timeout_wd);
        free(priv);
    }
}

static void cport_reset_timeout(int argc, uint32_t data, ...)
{
    struct cport_reset_priv *priv = (struct cport_reset_priv*) data;

    if (argc != 1)
        return;

    if (atomic_dec(&priv->refcount)) {
        vendor_request_deferred_submit(priv->dev, -ETIMEDOUT);
    } else {
        wd_delete(&priv->timeout_wd);
        free(priv);
    }
}

static int reset_cport(unsigned int cportid, struct usbdev_s *dev)
{
    struct cport_reset_priv *priv;

    priv = zalloc(sizeof(*priv));
    if (!priv) {
        return -ENOMEM;
    }

    wd_static(&priv->timeout_wd);
    priv->dev = dev;

    /* a ref for the watchdog and one for the unipro stack */
    atomic_init(&priv->refcount, 2);

    wd_start(&priv->timeout_wd, RESET_TIMEOUT_DELAY, cport_reset_timeout, 1,
             priv);
    return unipro_reset_cport(cportid, cport_reset_cb, priv);
}

static int cport_reset_vendor_request_out(struct usbdev_s *dev, uint8_t req,
                                          uint16_t index, uint16_t value,
                                          void *buf, uint16_t len)
{
    return reset_cport(value, dev);
}

static int cport_count_vendor_request_in(struct usbdev_s *dev, uint8_t req,
                                         uint16_t index, uint16_t value,
                                         void *buf, uint16_t len)
{
    *(uint16_t *) buf = cpu_to_le16(unipro_cport_count());
    return sizeof(uint16_t);
}

static int cport_enable_fct_tx_flow_vendor_request_out(struct usbdev_s *dev,
                                                       uint8_t req,
                                                       uint16_t index,
                                                       uint16_t value,
                                                       void *buf, uint16_t len)
{
    return unipro_enable_fct_tx_flow(value);
}

static int cport_disable_fct_tx_flow_vendor_request_out(struct usbdev_s *dev,
                                                        uint8_t req,
                                                        uint16_t index,
                                                        uint16_t value,
                                                        void *buf, uint16_t len)
{
    return unipro_disable_fct_tx_flow(value);
}

static struct unipro_driver unipro_driver = {
    .name = "APBridge",
    .rx_handler = recv_from_unipro,
};

static void apbridge_unipro_evt_handler(enum unipro_event evt)
{
    switch (evt) {
    case UNIPRO_EVT_LUP_DONE:
        tsb_unipro_set_init_status(INIT_STATUS_S3FW_BOOT_FINISHED);
        sem_post(&linkup_sem);
        break;

    default:
        break;
    }
}

static void _apbridgea_unipro_enable(void)
{
    sem_wait(&linkup_sem);
    tsb_unipro_mbox_send(TSB_MAIL_READY_AP);
}

void unipro_cport_mapping(unsigned int cportid, enum ep_mapping mapping)
{
    switch (mapping) {
    case MULTIPLEXED_EP:
        unipro_set_max_inflight_rxbuf_count(cportid, 1);
        break;

    case DIRECT_EP:
        unipro_set_max_inflight_rxbuf_count(cportid,
                                    CONFIG_TSB_UNIPRO_MAX_INFLIGHT_BUFCOUNT);
        break;
    }
}

static void *usb_wait_and_init(void *p_data)
{
    usb_wait(get_apbridge_dev());
    _apbridgea_unipro_enable();

    return NULL;
}

void apbridgea_unipro_init(void)
{
    int i;
    unsigned int cport_count = unipro_cport_count();

    sem_init(&linkup_sem, 0, 0);

    /* unipro_init{_*}() will initialize any non-display, non-camera CPorts */
    unipro_init_with_event_handler(apbridge_unipro_evt_handler);

    /* Now register a driver for those CPorts */
    for (i = 0; i < cport_count; i++) {
        /* These cports are already allocated for display and camera */
        if (i == CPORTID_CDSI0 || i == CPORTID_CDSI1)
            continue;
        unipro_driver_register(&unipro_driver, i);
    }

    if (register_vendor_request(APBRIDGE_ROREQUEST_CPORT_COUNT,
                                VENDOR_REQ_IN,
                                cport_count_vendor_request_in)) {
        printf("Fail to register APBRIDGE_ROREQUEST_CPORT_COUNT"
               " vendor request\n");
    }
    if (register_vendor_request(APBRIDGE_WOREQUEST_CPORT_RESET,
                            VENDOR_REQ_OUT | VENDOR_REQ_DEFER,
                            cport_reset_vendor_request_out)) {
        printf("Fail to register APBRIDGE_WOREQUEST_CPORT_RESET"
               " vendor request\n");
    }
    if (register_vendor_request(APBRIDGE_WOREQUEST_CPORT_ENA_FCT_TX_FLOW,
                                VENDOR_REQ_OUT,
                                cport_enable_fct_tx_flow_vendor_request_out)) {
        printf("Fail to register APBRIDGE_WOREQUEST_CPORT_ENA_FCT_TX_FLOW"
               " vendor request\n");
    }
    if (register_vendor_request(APBRIDGE_WOREQUEST_CPORT_DIS_FCT_TX_FLOW,
                                VENDOR_REQ_OUT,
                                cport_disable_fct_tx_flow_vendor_request_out)) {
        printf("Fail to register APBRIDGE_WOREQUEST_CPORT_DIS_FCT_TX_FLOW"
               " vendor request\n");
    }


}

int apbridgea_unipro_enable(void)
{
    int ret;

    ret = pthread_create(&g_apbridge_thread, NULL,
                         usb_wait_and_init, NULL);
    return ret;
}
