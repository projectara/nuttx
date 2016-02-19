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
#include <nuttx/ring_buf.h>
#include <arch/byteorder.h>

#include <arch/board/audio_apbridgea.h>
#include <arch/board/apbridgea_audio.h>
#include <arch/board/apbridgea_gadget.h>

#include "../../../../drivers/greybus/audio-gb.h"
#include "../../../../../apps/ara/apbridge/apbridge_backend.h"

#undef APBRIDGEA_AUDIO_I2S_MASTER

#define APBRIDGEA_AUDIO_I2S_CHANNELS            2 /* Don't ever change */
#define APBRIDGEA_AUDIO_RING_ENTRIES_TX         8

#define APBRIDGEA_AUDIO_FLAG_SET_CONFIG         BIT(0)
#define APBRIDGEA_AUDIO_FLAG_TX_DATA_SIZE_SET   BIT(1)
#define APBRIDGEA_AUDIO_FLAG_TX_STARTED         BIT(2)
#define APBRIDGEA_AUDIO_FLAG_RX_DATA_SIZE_SET   BIT(3)
#define APBRIDGEA_AUDIO_FLAG_RX_STARTED         BIT(4)

#define AUDIO_IS_CONFIGURED(_info, _type)                                   \
            (((_info)->flags & APBRIDGEA_AUDIO_FLAG_SET_CONFIG) &&          \
             ((_info)->flags & APBRIDGEA_AUDIO_FLAG_##_type##_DATA_SIZE_SET))

/* One per physical I2S port on the bridge */
struct apbridgea_audio_info {
    unsigned int        flags;
    uint16_t            i2s_port;
    struct device       *i2s_dev;
    struct ring_buf     *rx_rb;
    unsigned int        tx_data_size;
    struct list_head    cport_list;
    struct list_head    list;
};

struct apbridgea_audio_cport {
    uint16_t                    data_cportid;
    struct apbridgea_audio_info *info;
    struct list_head            list;
};

struct apbridgea_audio_demux_entry {
    void                        *buf;
    uint16_t                    len;
    struct list_head            list;
};

struct apbridga_audio_rb_hdr {
    uint32_t    not_acked;
};

static atomic_t request_id;

static pthread_t apbridgea_audio_demux_thread;
static sem_t apbridgea_audio_demux_sem;

static LIST_DECLARE(apbridgea_audio_info_list);
static LIST_DECLARE(apbridgea_audio_demux_list);

static struct apbridgea_audio_info *apbridgea_audio_find_info(uint16_t i2s_port)
{
    struct apbridgea_audio_info *info;
    struct list_head *iter;

    list_foreach(&apbridgea_audio_info_list, iter) {
        info = list_entry(iter, struct apbridgea_audio_info, list);

        if (info->i2s_port == i2s_port) {
            return info;
        }
    }

    return NULL;
}

static struct apbridgea_audio_cport *apbridgea_audio_find_cport(
                                             struct apbridgea_audio_info *info,
                                             uint16_t data_cportid)
{
    struct apbridgea_audio_cport *cport;
    struct list_head *iter;

    if (!info) {
        return NULL;
    }

    list_foreach(&info->cport_list, iter) {
        cport = list_entry(iter, struct apbridgea_audio_cport, list);

        if (cport->data_cportid == data_cportid) {
            return cport;
        }
    }

    return NULL;
}

static int apbridgea_audio_convert_format(uint32_t audio_fmt, uint32_t *i2s_fmt,
                                          unsigned int *bytes_per_chan)
{
    uint32_t fmt;
    unsigned int bytes;

    switch (audio_fmt) {
    case AUDIO_APBRIDGEA_PCM_FMT_8:
        fmt = DEVICE_I2S_PCM_FMT_8;
        bytes = 1;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_16:
        fmt = DEVICE_I2S_PCM_FMT_16;
        bytes = 2;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_24:
        fmt = DEVICE_I2S_PCM_FMT_24;
        bytes = 3;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_32:
        fmt = DEVICE_I2S_PCM_FMT_32;
        bytes = 4;
        break;
    case AUDIO_APBRIDGEA_PCM_FMT_64:
        fmt = DEVICE_I2S_PCM_FMT_64;
        bytes = 8;
        break;
    default:
        return -EINVAL;
    }

    if (i2s_fmt) {
        *i2s_fmt = fmt;
    }

    if (bytes_per_chan) {
        *bytes_per_chan = bytes;
    }

    return 0;
}

static int apbridgea_audio_convert_rate(uint32_t audio_rate, uint32_t *i2s_rate,
                                        unsigned int *sample_freq)
{
    uint32_t rate;
    unsigned int freq;
    int ret = 0;

    switch (audio_rate) {
    case AUDIO_APBRIDGEA_PCM_RATE_5512:
        rate = DEVICE_I2S_PCM_RATE_5512;
        freq = 5512;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_8000:
        rate = DEVICE_I2S_PCM_RATE_8000;
        freq = 8000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_11025:
        rate = DEVICE_I2S_PCM_RATE_11025;
        freq = 11025;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_16000:
        rate = DEVICE_I2S_PCM_RATE_16000;
        freq = 16000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_22050:
        rate = DEVICE_I2S_PCM_RATE_22050;
        freq = 22050;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_32000:
        rate = DEVICE_I2S_PCM_RATE_32000;
        freq = 32000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_44100:
        rate = DEVICE_I2S_PCM_RATE_44100;
        freq = 44100;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_48000:
        rate = DEVICE_I2S_PCM_RATE_48000;
        freq = 48000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_64000:
        rate = DEVICE_I2S_PCM_RATE_64000;
        freq = 64000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_88200:
        rate = DEVICE_I2S_PCM_RATE_88200;
        freq = 88200;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_96000:
        rate = DEVICE_I2S_PCM_RATE_96000;
        freq = 96000;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_176400:
        rate = DEVICE_I2S_PCM_RATE_176400;
        freq = 176400;
        break;
    case AUDIO_APBRIDGEA_PCM_RATE_192000:
        rate = DEVICE_I2S_PCM_RATE_192000;
        freq = 192000;
        break;
    default:
        ret = -EINVAL;
    }

    if (i2s_rate) {
        *i2s_rate = rate;
    }

    if (sample_freq) {
        *sample_freq = freq;
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
    unsigned int bytes;
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

    ret = apbridgea_audio_convert_rate(le32_to_cpu(req->rate), &rate, NULL);
    if (ret) {
        return ret;
    }

    pcm.format = format;
    pcm.rate = rate;
    pcm.channels = APBRIDGEA_AUDIO_I2S_CHANNELS;

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

    info->flags |= APBRIDGEA_AUDIO_FLAG_SET_CONFIG;

    return 0;
}

static int apbridgea_audio_from_usb(unsigned int cportid, void *buf, size_t len)
{
    usb_release_buffer(NULL, buf);
    return 0;
}

static int apbridgea_audio_from_unipro(unsigned int cportid, void *buf,
                                       size_t len)
{
    /* TODO: Implement receive data processing */
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

    /* TODO: There can be multiple tx CPorts but only one rx CPort active. */

    cport = malloc(sizeof(*cport));
    if (!cport) {
        return -ENOMEM;
    }

    cport->data_cportid = data_cportid;
    cport->info = info;
    list_init(&cport->list);

    flags = irqsave();

    /* Make all incoming UniPro messages go to apbridgea_audio_rx_handler */
    ret = map_offloaded_cport(get_apbridge_dev(), data_cportid,
                              apbridgea_audio_from_unipro,
                              apbridgea_audio_from_usb);
    if (ret) {
        goto err_free_cport;
    }

    list_add(&info->cport_list, &cport->list);

    irqrestore(flags);

    return 0;

err_free_cport:
    irqrestore(flags);
    free(cport);

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

    ret = unmap_offloaded_cport(get_apbridge_dev(), data_cportid);
    if (ret) {
        goto err_irqrestore;
    }

    list_del(&cport->list);

    irqrestore(flags);

    free(cport);
    return 0;

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int apbridgea_audio_set_tx_data_size(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    struct audio_apbridgea_set_tx_data_size_request *req = buf;
    uint16_t size;

    if (!req || (len < sizeof(*req))) {
        return -EINVAL;
    }

    if (info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED) {
        return -EPROTO;
    }

    size = le16_to_cpu(req->size);

    /* Size must be a multiple of 4 (required by i2s FIFO) */
    if (!size || (size % 4)) {
        return -EINVAL;
    }

    info->tx_data_size = size;
    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_DATA_SIZE_SET;

    return 0;
}

static int apbridgea_audio_send_data_completion(int status, const void *buf,
                                                void *priv)
{
    struct ring_buf *rb = priv;
    struct apbridga_audio_rb_hdr *rb_hdr;
    irqstate_t flags;

    rb_hdr = ring_buf_get_buf(rb);

    if (!rb_hdr->not_acked) { /* Should never happen */
        lowsyslog("%s: not_acked of zero\n", __func__);
        return 0;
    }

    flags = irqsave();

    rb_hdr->not_acked--;

    if (!rb_hdr->not_acked) {
        ring_buf_reset(rb);
        ring_buf_pass(rb);
    }

    irqrestore(flags);

    return 0;
}

static void apbridgea_audio_send_data(struct apbridgea_audio_info *info,
                                      struct ring_buf *rb)
{
    struct apbridgea_audio_cport *cport;
    struct apbridga_audio_rb_hdr *rb_hdr;
    struct gb_operation_hdr *gb_hdr;
    struct list_head *iter;
    int ret;

    rb_hdr = ring_buf_get_buf(rb);
    gb_hdr = ring_buf_get_priv(rb);

    gb_hdr->id = cpu_to_le16(atomic_inc(&request_id));
    if (gb_hdr->id == 0) /* ID 0 is for request with no response */
        gb_hdr->id = cpu_to_le16(atomic_inc(&request_id));

    rb_hdr->not_acked = 0;

    list_foreach(&info->cport_list, iter) {
        cport = list_entry(iter, struct apbridgea_audio_cport, list);

        rb_hdr->not_acked++;

        ret = unipro_send_async(cport->data_cportid, gb_hdr,
                                le16_to_cpu(gb_hdr->size),
                                apbridgea_audio_send_data_completion, rb);
        if (ret) {
            rb_hdr->not_acked--;
        }
    }
}

static void apbridgea_audio_i2s_rx_cb(struct ring_buf *rb,
                                      enum device_i2s_event event, void *arg)
{
    struct apbridgea_audio_info *info = arg;

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED) ||
        (event != DEVICE_I2S_EVENT_RX_COMPLETE)) {

        ring_buf_reset(rb);
        ring_buf_pass(rb);

        return;
    }

    apbridgea_audio_send_data(info, rb);
}

/*
 * Each ring buffer entry will point to a memory area containing:
 * - A ring buffer data header containing:
 *   - a 4-byte count field used by apbridgea_audio_send_data_completion()
 *     to tell when the last UniPro send has completed and the rb entry
 *     can be passed back to the i2s driver.
 * - The Greybus Audio Data Message containing:
 *   - the Greybus header;
 *   - the Greybus Audio Data Message header;
 *   - the audio data.
 *
 * The value returned by ring_buf_get_buf() points to this area.
 * The private area (ring_buf_get_priv()/ring_buf_set_priv()) is
 * used to point to the Greybus Audio Data Message.
 */
static int apbridgea_audio_rb_alloc(struct ring_buf *rb, void *arg)
{
    struct apbridgea_audio_info *info = arg;
    struct apbridga_audio_rb_hdr *rb_hdr;
    struct gb_operation_hdr *gb_hdr;
    size_t headroom, total_size;
    void *buf;

    headroom = sizeof(*rb_hdr) + sizeof(*gb_hdr) +
               sizeof(struct gb_audio_send_data_request);
    total_size = headroom + info->tx_data_size;

    buf = malloc(total_size);
    if (!buf) {
        return -ENOMEM;
    }

    memset(buf, 0, headroom);

    gb_hdr = buf + sizeof(*rb_hdr);
    gb_hdr->size = cpu_to_le16(total_size - sizeof(*rb_hdr));
    gb_hdr->type = GB_AUDIO_TYPE_SEND_DATA;

    ring_buf_init(rb, buf, headroom, info->tx_data_size);
    ring_buf_set_priv(rb, gb_hdr);

    return 0;
}

static void apbridgea_audio_rb_free(struct ring_buf *rb, void *arg)
{
    free(ring_buf_get_buf(rb));
}

/* "Transmitting" means receiving from I2S and transmitting over UniPro */
static int apbridgea_audio_start_tx(struct apbridgea_audio_info *info,
                                    void *buf, uint16_t len)
{
    struct audio_apbridgea_start_tx_request *req = buf;
    int ret;

    if (!req || (len < sizeof(*req))) {
        return -EINVAL;
    }

    if (!AUDIO_IS_CONFIGURED(info, TX) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {
        return -EPROTO;
    }

    info->rx_rb = ring_buf_alloc_ring(APBRIDGEA_AUDIO_RING_ENTRIES_TX, 0, 0, 0,
                                      apbridgea_audio_rb_alloc,
                                      apbridgea_audio_rb_free, info);
    if (!info->rx_rb) {
        return -ENOMEM;
    }

    ret = device_i2s_prepare_receiver(info->i2s_dev, info->rx_rb,
                                      apbridgea_audio_i2s_rx_cb, info);
    if (ret) {
        goto err_free_ring;
    }

    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_STARTED;

    ret = device_i2s_start_receiver(info->i2s_dev);
    if (ret) {
        info->flags &= ~APBRIDGEA_AUDIO_FLAG_TX_STARTED;
        goto err_shutdown_receiver;
    }

    return 0;

err_shutdown_receiver:
    device_i2s_shutdown_receiver(info->i2s_dev);
err_free_ring:
    ring_buf_free_ring(info->rx_rb, apbridgea_audio_rb_free, info);
    info->rx_rb = NULL;

    return ret;
}

static int apbridgea_audio_stop_tx(struct apbridgea_audio_info *info)
{
    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {
        return -EPROTO;
    }

    device_i2s_stop_receiver(info->i2s_dev);
    device_i2s_shutdown_receiver(info->i2s_dev);

    info->flags &= ~APBRIDGEA_AUDIO_FLAG_TX_STARTED;

    ring_buf_free_ring(info->rx_rb, apbridgea_audio_rb_free, info);
    info->rx_rb = NULL;

    return 0;
}

static int apbridgea_audio_set_rx_data_size(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    return -ENOSYS;
}

/* "Receiving" means receiving from UniPro and transmitting over I2S */
static int apbridgea_audio_start_rx(struct apbridgea_audio_info *info)
{
    return -ENOSYS;
}

static int apbridgea_audio_stop_rx(struct apbridgea_audio_info *info)
{
    return -ENOSYS;
}

static void *apbridgea_audio_demux(void *ignored)
{
    struct apbridgea_audio_info *info;
    struct apbridgea_audio_demux_entry *demux_entry;
    struct audio_apbridgea_hdr *pkt_hdr;
    struct list_head *first_entry;
    uint16_t len;
    irqstate_t flags;
    int ret;

    while(1) {
        ret = sem_wait(&apbridgea_audio_demux_sem);
        if (ret != OK) {
            continue;
        }

        flags = irqsave();
        first_entry = apbridgea_audio_demux_list.next;
        list_del(first_entry);
        irqrestore(flags);

        demux_entry = list_entry(first_entry,
                                 struct apbridgea_audio_demux_entry, list);

        pkt_hdr = demux_entry->buf;
        len = demux_entry->len;

        if (!pkt_hdr || (len < sizeof(*pkt_hdr))) {
            free(demux_entry);
            continue;
        }

        info = apbridgea_audio_find_info(pkt_hdr->i2s_port);
        if (!info) {
            free(demux_entry);
            continue;
        }

        switch (pkt_hdr->type) {
        case AUDIO_APBRIDGEA_TYPE_SET_CONFIG:
            ret = apbridgea_audio_set_config(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_REGISTER_CPORT:
            ret = apbridgea_audio_register_cport(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_UNREGISTER_CPORT:
            ret = apbridgea_audio_unregister_cport(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_SET_TX_DATA_SIZE:
            ret = apbridgea_audio_set_tx_data_size(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_START_TX:
            ret = apbridgea_audio_start_tx(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_STOP_TX:
            ret = apbridgea_audio_stop_tx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_SET_RX_DATA_SIZE:
            ret = apbridgea_audio_set_rx_data_size(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_START_RX:
            ret = apbridgea_audio_start_rx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_STOP_RX:
            ret = apbridgea_audio_stop_rx(info);
            break;
        }

        free(demux_entry);
    }

    /* NOTREACHED */
    return NULL;
}

int apbridgea_audio_out_demux(void *buf, uint16_t len)
{
    struct apbridgea_audio_demux_entry *demux_entry;

    if (!buf || !len) {
        return -EINVAL;
    }

    demux_entry = malloc(sizeof(*demux_entry) + len);
    if (!demux_entry) {
        return -ENOMEM;
    }

    demux_entry->buf = (uint8_t *)demux_entry + sizeof(*demux_entry);
    demux_entry->len = len;

    memcpy(demux_entry->buf, buf, len);

    list_init(&demux_entry->list);

    /* In irq context */
    list_add(&apbridgea_audio_demux_list, &demux_entry->list);

    sem_post(&apbridgea_audio_demux_sem);

    return 0;
}

/* Only support one I2S port right now (but multiple TX CPorts) */
int apbridgea_audio_init(void)
{
    struct apbridgea_audio_info *info;
    int ret;

    info = zalloc(sizeof(*info));
    if (!info) {
        return -ENOMEM;
    }

    info->i2s_port = 0; /* TODO: Get from init_data */

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
                         apbridgea_audio_demux, NULL);
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
