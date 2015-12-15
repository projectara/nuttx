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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/list.h>
#include <nuttx/sched.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/greybus.h>
#include <arch/byteorder.h>

#include <arch/board/audio_apbridgea.h>
#include <arch/board/apbridgea_audio.h>

#include "../../../../drivers/greybus/audio-gb.h"
#include "../../../../../apps/ara/apbridge/apbridge_backend.h"

#undef APBRIDGEA_AUDIO_I2S_MASTER

#define APBRIDGEA_AUDIO_FLAG_SET_CONFIG            BIT(0)
#define APBRIDGEA_AUDIO_FLAG_TX_DATA_SIZE_SET      BIT(1)
#define APBRIDGEA_AUDIO_FLAG_TX_STARTED            BIT(2)
#define APBRIDGEA_AUDIO_FLAG_RX_DATA_SIZE_SET      BIT(3)
#define APBRIDGEA_AUDIO_FLAG_RX_STARTED            BIT(4)

#define AUDIO_IS_CONFIGURED(_info, _type)                                   \
            (((_info)->flags & APBRIDGEA_AUDIO_FLAG_SET_CONFIG) &&          \
             ((_info)->flags & APBRIDGEA_AUDIO_FLAG_##_type##_DATA_SIZE_SET))

/* One per physical I2S port on the bridge */
struct apbridgea_audio_info {
    unsigned int        flags;
    uint16_t            i2s_port;
    struct device       *i2s_dev;
    unsigned int        sample_size;
    unsigned int        sample_frequency;
    struct ring_buf     *rx_rb;
    unsigned int        tx_data_size;
#if 0 /* FIXME */
    struct ring_buf     *tx_rb;
    unsigned int        rx_data_size;
#endif
    struct list_head    cport_list;
    struct list_head    list;
};

struct apbridgea_audio_audio_demux {
    struct list_head            list;
    void                        *buf;
    uint16_t                    len;
};

struct apbridgea_audio_cport {
    uint16_t                    data_cportid;
    struct apbridgea_audio_info *info;
    struct list_head            list;
};

static unsigned int mag_msg_count = 0;
static unsigned int mag_msg_reset = 0;

static atomic_t request_id;

static pthread_t apbridgea_audio_demux_thread;
static sem_t apbridgea_audio_demux_sem;

static LIST_DECLARE(apbridgea_audio_info_list);
static LIST_DECLARE(apbridgea_audio_demux_list);

static struct apbridgea_audio_info *apbridgea_audio_find_info(uint16_t i2s_port)
{
    struct apbridgea_audio_info *info;
    struct list_head *iter;
    irqstate_t flags;

    flags = irqsave();
    list_foreach(&apbridgea_audio_info_list, iter) {
        info = list_entry(iter, struct apbridgea_audio_info, list);

        if (info->i2s_port == i2s_port) {
            irqrestore(flags);
            return info;
        }
    }
    irqrestore(flags);

    return NULL;
}

static struct apbridgea_audio_cport *apbridgea_audio_find_cport(
                                             struct apbridgea_audio_info *info,
                                             uint16_t data_cportid)
{
    struct apbridgea_audio_cport *cport;
    struct list_head *iter;
    irqstate_t flags;

    if (!info) {
        return NULL;
    }

    flags = irqsave();
    list_foreach(&info->cport_list, iter) {
        cport = list_entry(iter, struct apbridgea_audio_cport, list);

        if (cport->data_cportid == data_cportid) {
            return cport;
        }
    }
    irqrestore(flags);

    return NULL;
}

static int apbridgea_audio_get_tx_delay(void *buf, uint16_t len)
{
#if 0 /* FIXME: Need implementation change to support request/response */
    struct apbridgea_audio_get_tx_delay_response *resp = buf;

XXX check that not AUDIO_IS_CONFIGURED(_info, _type)

    if (!resp || (len < sizeof(*resp))) {
        return -EINVAL;
    }

    resp->delay = 0; /* TODO: measure and set here */

    return sizeof(*resp);
#else
    return -ENOSYS;
#endif
}

static int apbridgea_audio_get_rx_delay(void *buf, uint16_t len)
{
#if 0 /* FIXME: Need implementation change to support request/response */
    struct apbridgea_audio_get_rx_delay_response *resp = buf;

    if (!resp || (len < sizeof(*resp))) {
        return -EINVAL;
    }

XXX check that not AUDIO_IS_CONFIGURED(_info, _type)

    resp->delay = 0; /* TODO: measure and set here */

    return sizeof(*resp);
#else
    return -ENOSYS;
#endif
}

int apbridgea_audio_in_demux(uint16_t data_cportid, uint16_t i2s_port,
                                   uint8_t *buf, uint16_t len)
{
    struct audio_apbridgea_hdr *pkt_hdr = (struct audio_apbridgea_hdr *)buf;
    int ret;

    if (!pkt_hdr || (len < sizeof(*pkt_hdr))) {
        return -EINVAL;
    }

    switch (pkt_hdr->type) {
    case AUDIO_APBRIDGEA_TYPE_GET_TX_DELAY:
        ret = apbridgea_audio_get_tx_delay(pkt_hdr, len);
        break;
    case AUDIO_APBRIDGEA_TYPE_GET_RX_DELAY:
        ret = apbridgea_audio_get_rx_delay(pkt_hdr, len);
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int apbridgea_audio_convert_format(uint32_t audio_fmt, uint32_t *i2s_fmt,
                                          unsigned int *bytes)
{
    int ret = 0;

    switch (audio_fmt) {
    case AUDIO_APBRIDGEA_PCM_FMT_8:
        *i2s_fmt = DEVICE_I2S_PCM_FMT_8;
        *bytes = 1;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_16:
        *i2s_fmt = DEVICE_I2S_PCM_FMT_16;
        *bytes = 2;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_24:
        *i2s_fmt = DEVICE_I2S_PCM_FMT_24;
        *bytes = 3;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_32:
        *i2s_fmt = DEVICE_I2S_PCM_FMT_32;
        *bytes = 4;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_64:
        *i2s_fmt = DEVICE_I2S_PCM_FMT_64;
        *bytes = 8;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int apbridgea_audio_convert_rate(uint32_t audio_rate, uint32_t *i2s_rate,
                                        unsigned int *freq)
{
    int ret = 0;

    switch (audio_rate) {
    case AUDIO_APBRIDGEA_PCM_RATE_5512:
        *i2s_rate = DEVICE_I2S_PCM_RATE_5512;
        *freq = 5512;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_8000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_8000;
        *freq = 8000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_11025:
        *i2s_rate = DEVICE_I2S_PCM_RATE_11025;
        *freq = 11025;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_16000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_16000;
        *freq = 16000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_22050:
        *i2s_rate = DEVICE_I2S_PCM_RATE_22050;
        *freq = 22050;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_32000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_32000;
        *freq = 32000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_44100:
        *i2s_rate = DEVICE_I2S_PCM_RATE_44100;
        *freq = 44100;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_48000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_48000;
        *freq = 48000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_64000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_64000;
        *freq = 64000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_88200:
        *i2s_rate = DEVICE_I2S_PCM_RATE_88200;
        *freq = 88200;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_96000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_96000;
        *freq = 96000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_176400:
        *i2s_rate = DEVICE_I2S_PCM_RATE_176400;
        *freq = 176400;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_192000:
        *i2s_rate = DEVICE_I2S_PCM_RATE_192000;
        *freq = 192000;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int apbridgea_audio_set_config(struct apbridgea_audio_info *info,
                                      void *buf, uint16_t len)
{
    struct audio_apbridgea_set_config_request *req = buf;
    struct device_i2s_pcm pcm;
    struct device_i2s_dai dai;
    uint32_t format, rate;
    unsigned int bytes, freq;
    int ret;

    if (!req || (len < sizeof(*req))) {
        return -EINVAL;
    }

    if ((info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_RX_STARTED)) {
        return -EPROTO;
    }

    ret = apbridgea_audio_convert_format(le32_to_cpu(req->format), &format,
                                         &bytes);
    if (ret) {
        return ret;
    }

    ret = apbridgea_audio_convert_rate(le32_to_cpu(req->rate), &rate, &freq);
    if (ret) {
        return ret;
    }

    pcm.format = format;
    pcm.rate = rate;
    pcm.channels = 2;

    dai.mclk_freq = le32_to_cpu(req->mclk_freq);
    dai.protocol = DEVICE_I2S_PROTOCOL_I2S;
    dai.wclk_polarity = DEVICE_I2S_POLARITY_NORMAL;

#ifdef APBRIDGEA_AUDIO_I2S_MASTER
    dai.wclk_change_edge = DEVICE_I2S_EDGE_FALLING;
    dai.data_tx_edge = DEVICE_I2S_EDGE_FALLING;
    dai.data_rx_edge = DEVICE_I2S_EDGE_RISING;

    ret = device_i2s_set_config(info->i2s_dev, DEVICE_I2S_ROLE_MASTER, &pcm,
                                &dai);
    if (ret) {
        return ret;
    }
#else
    dai.wclk_change_edge = DEVICE_I2S_EDGE_RISING;
    dai.data_tx_edge = DEVICE_I2S_EDGE_RISING;
    dai.data_rx_edge = DEVICE_I2S_EDGE_FALLING;

    ret = device_i2s_set_config(info->i2s_dev, DEVICE_I2S_ROLE_SLAVE, &pcm,
                                &dai);
    if (ret) {
        return ret;
    }
#endif

    info->sample_size = bytes * 2; /* two channels */
    info->sample_frequency = freq;
    info->flags |= APBRIDGEA_AUDIO_FLAG_SET_CONFIG;

    return 0;
}

static int apbridgea_audio_rx_handler(unsigned int cportid, void *buf,
                                      size_t len)
{
    /* TODO: Implement receive data processing but for now, ignore it */
    unipro_rxbuf_free(cportid, buf);
    return 0;
}

static int apbridgea_audio_register_cport(struct apbridgea_audio_info *info,
                                          void *buf, uint16_t len)
{
    struct audio_apbridgea_register_cport_request *req = buf;
    uint16_t data_cportid = le16_to_cpu(req->cport);
    struct apbridgea_audio_cport *cport;
    irqstate_t flags;
    int ret;

    cport = apbridgea_audio_find_cport(info, data_cportid);
    if (cport) {
        return -EBUSY;
    }

    cport = malloc(sizeof(*cport));
    if (!cport) {
        return -ENOMEM;
    }

    /*
     * FIXME: if APBRIDGEA_AUDIO_FLAG_RX_STARTED, make sure only 1 cport
     * is registered
     */

    flags = irqsave();

    /* Make all incoming UniPro messages to to apbridgea_audio_rx_handler */
    ret = apbridgea_local_rx_enable(data_cportid, apbridgea_audio_rx_handler);
    if (ret) {
        goto err_irqrestore;
    }

    cport->data_cportid = data_cportid;
    cport->info = info;
    list_init(&cport->list);

    list_add(&info->cport_list, &cport->list);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int apbridgea_audio_unregister_cport(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    struct audio_apbridgea_unregister_cport_request *req = buf;
    uint16_t data_cportid = le16_to_cpu(req->cport);
    struct apbridgea_audio_cport *cport;
    irqstate_t flags;
    int ret;

    cport = apbridgea_audio_find_cport(info, data_cportid);
    if (!cport) {
        return -EINVAL;
    }

    flags = irqsave();

    ret = apbridgea_local_rx_disable(data_cportid);
    if (ret) { /* This is really bad; should never happen */
        goto err_irqrestore;
    }

    list_del(&cport->list);

    irqrestore(flags);

    free(cport);
    return 0;

err_irqrestore:
    irqrestore(flags);

    return 0;
}

static int apbridgea_audio_set_tx_data_size(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    struct audio_apbridgea_set_tx_data_size_request *req = buf;

    if (!req || (len < sizeof(*req))) {
        return -EINVAL;
    }

    if (info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED) {
        return -EPROTO;
    }

    info->tx_data_size = le16_to_cpu(req->size);
    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_DATA_SIZE_SET;

    return 0;
}

static int apbridgea_audio_send_data_completion(int status, const void *buf,
                                                void *priv)
{
    struct ring_buf *rb = priv;

    if (rb) {
        ring_buf_reset(rb);
        ring_buf_pass(rb);
mag_msg_reset++;
    } else {
lldbg("rb is NULL\n");
    }

    return 0;
}

static int apbridgea_audio_send_data(struct apbridgea_audio_info *info,
                                     struct ring_buf *rb)
{
    struct apbridgea_audio_cport *cport;
    struct gb_operation_hdr *gb_hdr;
    struct ring_buf *arg;
    struct list_head *iter;
    int ret;

mag_msg_count++;

    gb_hdr = ring_buf_get_priv(rb);

    gb_hdr->id = cpu_to_le16(atomic_inc(&request_id));
    if (gb_hdr->id == 0) /* ID 0 is for request with no response */
        gb_hdr->id = cpu_to_le16(atomic_inc(&request_id));

    list_foreach(&info->cport_list, iter) {
        cport = list_entry(iter, struct apbridgea_audio_cport, list);

        arg = list_node_is_last(&info->cport_list, iter) ? rb : NULL;

if (!arg) {
    lldbg("list_nost_is_last() didn't return true\n");
}

        ret = unipro_send_async(cport->data_cportid, gb_hdr,
                                le16_to_cpu(gb_hdr->size),
                                apbridgea_audio_send_data_completion, arg);
        if (ret) {
lldbg("unipro_send failed: %d\n", ret);
        }
    }

    return 0;
}

static void apbridgea_audio_i2s_rx_cb(struct ring_buf *rb,
                                      enum device_i2s_event event, void *arg)
{
    struct apbridgea_audio_info *info = arg;

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {
        return;
    }

    if (event == DEVICE_I2S_EVENT_RX_COMPLETE) {
        apbridgea_audio_send_data(info, rb);
    } else {
lldbg("RX callback event: 0x%x\n", event);
lldbg("****** mag_msg_count: %u\n", mag_msg_count);
lldbg("****** mag_msg_reset: %u\n", mag_msg_reset);
    }
}

static int apbridgea_audio_rb_alloc(struct ring_buf *rb, void *arg)
{
    struct apbridgea_audio_info *info = arg;
    struct gb_audio_send_data_request *request;
    struct gb_operation_hdr *gb_hdr;
    size_t gb_hdr_size, total_size;

    gb_hdr_size = sizeof(*gb_hdr) + sizeof(*request);
    total_size = gb_hdr_size + info->tx_data_size;

    gb_hdr = malloc(total_size);
    if (!gb_hdr) {
        return -ENOMEM;
    }

    ring_buf_init(rb, gb_hdr, gb_hdr_size, info->tx_data_size);

    gb_hdr->size = cpu_to_le16(total_size);
    gb_hdr->type = GB_AUDIO_TYPE_SEND_DATA;

    request = (struct gb_audio_send_data_request *)
                  ((unsigned char *)gb_hdr + sizeof(*gb_hdr));
    request->timestamp = 0; /* TODO: Implement timestamp support */

    ring_buf_set_priv(rb, gb_hdr);

    return 0;
}

static void apbridgea_audio_rb_free(struct ring_buf *rb, void *arg)
{
    free(ring_buf_get_priv(rb));
}

/* "Transmitting" means receiving from I2S and transmitting over Greybus */
static int apbridgea_audio_start_tx(struct apbridgea_audio_info *info,
                                    void *buf, uint16_t len)
{
    struct audio_apbridgea_start_tx_request *req = buf;
    irqstate_t flags;
    int ret;

    if (!req || (len < sizeof(*req))) {
        return -EINVAL;
    }

    if (!AUDIO_IS_CONFIGURED(info, TX) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {
        return -EPROTO;
    }

    info->rx_rb = ring_buf_alloc_ring(64, 0, 0, 0, apbridgea_audio_rb_alloc,
                                      apbridgea_audio_rb_free, info);
    if (!info->rx_rb) {
        return -ENOMEM;
    }

    ret = device_i2s_prepare_receiver(info->i2s_dev, info->rx_rb,
                                      apbridgea_audio_i2s_rx_cb, info);
    if (ret) {
        goto err_free_ring;
    }

    flags = irqsave();

    ret = device_i2s_start_receiver(info->i2s_dev);
    if (ret) {
        goto err_shutdown_receiver;
    }

    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_STARTED;

    irqrestore(flags);
    return 0;

err_shutdown_receiver:
    irqrestore(flags);
    device_i2s_shutdown_receiver(info->i2s_dev);
err_free_ring:
    ring_buf_free_ring(info->rx_rb, apbridgea_audio_rb_free, info);
    info->rx_rb = NULL;

    return ret;
}

static int apbridgea_audio_stop_tx(struct apbridgea_audio_info *info)
{
    irqstate_t flags;

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {
        return -EPROTO;
    }

    flags = irqsave();

    device_i2s_stop_receiver(info->i2s_dev);
    device_i2s_shutdown_receiver(info->i2s_dev);

    ring_buf_free_ring(info->rx_rb, apbridgea_audio_rb_free, info);
    info->rx_rb = NULL;

    info->flags &= ~APBRIDGEA_AUDIO_FLAG_TX_STARTED;

    irqrestore(flags);

lldbg("****** mag_msg_count: %u\n", mag_msg_count);
lldbg("****** mag_msg_reset: %u\n", mag_msg_reset);

    return 0;
}

static int apbridgea_audio_set_rx_data_size(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
#if 0 /* FIXME */
    struct audio_apbridgea_set_rx_data_size_request *req = buf;
    int ret;

    if (!req || (len < sizeof(*req))) {
        return -EINVAL;
    }

    if (info->flags & APBRIDGEA_AUDIO_FLAG_RX_STARTED) {
        return -EPROTO;
    }

    info->rx_data_size = le16_to_cpu(req->size);
    info->flags |= APBRIDGEA_AUDIO_FLAG_RX_DATA_SIZE_SET;

    return 0;
#else
    return -ENOSYS;
#endif
}

/* "Receiving" means receiving from Greybus and transmitting over I2S */
static int apbridgea_audio_start_rx(struct apbridgea_audio_info *info)
{
    /* TODO: Impement */
    /* XXX: Make sure there is only one cport registered for rx */
    return -ENOSYS;
}

static int apbridgea_audio_stop_rx(struct apbridgea_audio_info *info)
{
    /* TODO: Impement */
    return -ENOSYS;
}

int apbridgea_audio_out_demux(void *buf, uint16_t len)
{
    struct apbridgea_audio_audio_demux *demux_entry;
    irqstate_t flags;

    if (!buf || !len) {
lldbg("fail 1 - buf: 0x%p, len: %u\n", buf, len);
        return -EINVAL;
    }

    /* In interrupt context now,
     * nuttx is non-preemptive so its safe to manipulate the list here
     * so long as all non-interrupt context code dealing with the list
     * is guarded by sched_lock
     */
    demux_entry = malloc(sizeof(*demux_entry) + len);
    if (!demux_entry) {
lldbg("fail 2\n");
        return -ENOMEM;
    }

    demux_entry->buf = (void *)demux_entry + sizeof(*demux_entry);
    demux_entry->len = len;

    memcpy(demux_entry->buf, buf, len);

    list_init(&demux_entry->list);

    flags = irqsave();
    list_add(&apbridgea_audio_demux_list, &demux_entry->list);
    irqrestore(flags);

    sem_post(&apbridgea_audio_demux_sem);

    return 0;
}

static void *apbridgea_audio_demux_thread_start(void *ignored)
{
    struct apbridgea_audio_info *info;
    struct apbridgea_audio_audio_demux *demux_entry;
    struct audio_apbridgea_hdr *pkt_hdr;
    struct list_head *head;
    uint16_t len;
    irqstate_t flags;
    int ret;

    while(1) {
        ret = sem_wait(&apbridgea_audio_demux_sem);
        if (ret != OK) {
lldbg("fail 1\n");
            continue;
        }

        flags = irqsave();
        head = apbridgea_audio_demux_list.next;
        list_del(head);
        irqrestore(flags);

        demux_entry = list_entry(head, struct apbridgea_audio_audio_demux,
                                 list);

#if 0
lldbg("demux_entry: 0x%p, demux_entry->buf: 0x%x, demux_entry->len: %d\n",
        demux_entry, demux_entry->buf, demux_entry->len);
#endif

        pkt_hdr = demux_entry->buf;
        len = demux_entry->len;

        if (!pkt_hdr || (len < sizeof(*pkt_hdr))) {
lldbg("fail 2 - pkt_hdr: 0x%p, len: %u\n", pkt_hdr, len);
            free(demux_entry);
            continue;
        }

        info = apbridgea_audio_find_info(pkt_hdr->i2s_port);
        if (!info) {

lldbg("fail 3 - demux: 0x%p, pkt_hdr: 0x%p, len: %u, i2s_port: %u\n",
        demux_entry, pkt_hdr, demux_entry->len, pkt_hdr->i2s_port);

            free(demux_entry);
            continue;
        }

        switch (pkt_hdr->type) {
        case AUDIO_APBRIDGEA_TYPE_SET_CONFIG:
lldbg("apbridgea_audio_set_config - call\n");
            ret = apbridgea_audio_set_config(info, pkt_hdr, len);
lldbg("apbridgea_audio_set_config - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_REGISTER_CPORT:
lldbg("apbridgea_audio_register_cport - call\n");
            ret = apbridgea_audio_register_cport(info, pkt_hdr, len);
lldbg("apbridgea_audio_register_cport - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_UNREGISTER_CPORT:
lldbg("apbridgea_audio_unregister_cport - call\n");
            ret = apbridgea_audio_unregister_cport(info, pkt_hdr, len);
lldbg("apbridgea_audio_unregister_cport - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_SET_TX_DATA_SIZE:
lldbg("apbridgea_audio_set_tx_data_size - call\n");
            ret = apbridgea_audio_set_tx_data_size(info, pkt_hdr, len);
lldbg("apbridgea_audio_set_tx_data_size - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_START_TX:
lldbg("apbridgea_audio_start_tx - call\n");
            ret = apbridgea_audio_start_tx(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_STOP_TX:
            ret = apbridgea_audio_stop_tx(info);
lldbg("apbridgea_audio_stop_tx - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_SET_RX_DATA_SIZE:
lldbg("apbridgea_audio_set_rx_data_size - call\n");
            ret = apbridgea_audio_set_rx_data_size(info, pkt_hdr, len);
lldbg("apbridgea_audio_set_rx_data_size - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_START_RX:
lldbg("apbridgea_audio_start_rx - call\n");
            ret = apbridgea_audio_start_rx(info);
lldbg("apbridgea_audio_start_rx - exit %d\n", ret);
            break;
        case AUDIO_APBRIDGEA_TYPE_STOP_RX:
lldbg("apbridgea_audio_stop_rx - call\n");
            ret = apbridgea_audio_stop_rx(info);
lldbg("apbridgea_audio_stop_rx - exit %d\n", ret);
            break;
        default:
            lldbg("AAAAHHHHHHHHHHH - type: %d/0x%x\n", pkt_hdr->type,
                  pkt_hdr->type);
            {
                unsigned int i;
                unsigned char *bp = (unsigned char *)pkt_hdr;

                for (i = 0; i < len; i++) {
                    lldbg("%u: 0x%x\n", i, bp[i]);
                }
            }
        }

        free(demux_entry);
    }

    /* NOTREACHED */
    return NULL;
}

/* Only support one I2S port right now (but multiple tx cports) */
int apbridgea_audio_init(void)
{
    struct apbridgea_audio_info *info;
    int ret;

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->i2s_port = 0; /* FIXME: Determine from board data */

    info->i2s_dev = device_open(DEVICE_TYPE_I2S_HW, info->i2s_port);
    if (!info->i2s_dev) {
        ret = -EIO;
        goto err_free_info;
    }

    ret = sem_init(&apbridgea_audio_demux_sem, 0, 0);
    if (ret) {
        goto err_close_i2s;
    }

    ret = pthread_create(&apbridgea_audio_demux_thread, NULL,
                         apbridgea_audio_demux_thread_start, NULL);
    if (ret) {
        goto err_destroy_sem;
    }

    list_init(&info->cport_list);
    list_init(&info->list);

    list_add(&apbridgea_audio_info_list, &info->list);

    atomic_init(&request_id, 0);

    return 0;

err_destroy_sem:
    sem_destroy(&apbridgea_audio_demux_sem);
err_close_i2s:
    device_close(info->i2s_dev);
err_free_info:
    free(info);

    return ret;
}
