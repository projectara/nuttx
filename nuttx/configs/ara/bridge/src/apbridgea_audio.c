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
#define DBG_COMP ARADBG_AUDIO
#include <ara_debug.h>
#include <nuttx/irq.h>
#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/list.h>
#include <nuttx/sched.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/ring_buf.h>
#include <nuttx/bufram.h>
#include <arch/byteorder.h>

#include <arch/board/audio_apbridgea.h>
#include <arch/board/apbridgea_audio.h>
#include <arch/board/apbridgea_gadget.h>

#include "../../../../drivers/greybus/audio-gb.h"

#undef APBRIDGEA_AUDIO_I2S_MASTER

#define APBRIDGEA_AUDIO_I2S_CHANNELS            2 /* Don't ever change */
#define APBRIDGEA_AUDIO_RING_ENTRIES_TX         8
#define APBRIDGEA_AUDIO_RING_ENTRIES_RX         8

#define APBRIDGEA_AUDIO_UNIPRO_ALIGNMENT        8 /* Should come from unipro */

#define APBRIDGEA_AUDIO_FLAG_SET_CONFIG         BIT(0)
#define APBRIDGEA_AUDIO_FLAG_TX_DATA_SIZE_SET   BIT(1)
#define APBRIDGEA_AUDIO_FLAG_TX_PREPARED        BIT(2)
#define APBRIDGEA_AUDIO_FLAG_TX_STARTED         BIT(3)
#define APBRIDGEA_AUDIO_FLAG_RX_DATA_SIZE_SET   BIT(4)
#define APBRIDGEA_AUDIO_FLAG_RX_PREPARED        BIT(5)
#define APBRIDGEA_AUDIO_FLAG_RX_STARTED         BIT(6)

#define AUDIO_IS_CONFIGURED(_info, _type)                                   \
            (((_info)->flags & APBRIDGEA_AUDIO_FLAG_SET_CONFIG) &&          \
             ((_info)->flags & APBRIDGEA_AUDIO_FLAG_##_type##_DATA_SIZE_SET))

/* One per physical I2S port on the bridge */
struct apbridgea_audio_info {
    unsigned int        flags;
    uint16_t            i2s_port;
    struct device       *i2s_dev;

    struct ring_buf     *tx_rb;
    unsigned int        tx_data_size;
    size_t              tx_rb_headroom;
    size_t              tx_rb_total_size;

    struct ring_buf     *rx_rb;
    unsigned int        rx_data_size;
    uint8_t             *rx_dummy_data;
    uint16_t            rx_data_cportid;
    unsigned int        rx_rb_count;

    struct list_head    cport_list;
    struct list_head    list;
};

struct apbridgea_audio_cport {
    uint16_t                    data_cportid;
    uint8_t                     direction;
    struct apbridgea_audio_info *info;
    struct list_head            list;
};

struct apbridgea_audio_demux_entry {
    void                        *buf;
    uint16_t                    len;
    struct list_head            list;
};

/* Must be multiple of APBRIDGEA_AUDIO_UNIPRO_ALIGNMENT bytes in size */
struct apbridga_audio_rb_hdr {
    uint32_t                    not_acked;
    sem_t                       complete;
    struct apbridgea_audio_info *info;
    uint8_t                     pad[4];
};

static void apbridgea_audio_i2s_tx(struct apbridgea_audio_info *info,
                                   uint8_t *data);

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

static struct apbridgea_audio_info *apbridgea_audio_find_rx_info(
                                                      uint16_t rx_data_cportid)
{
    struct apbridgea_audio_info *info;
    struct list_head *iter;

    if (!rx_data_cportid) {
        return NULL;
    }

    list_foreach(&apbridgea_audio_info_list, iter) {
        info = list_entry(iter, struct apbridgea_audio_info, list);

        if (info->rx_data_cportid == rx_data_cportid) {
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
        dbg_error("%s: invalid audio format 0x%x\n", __func__, audio_fmt);
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
        dbg_error("%s: invalid audio rate 0x%x\n", __func__, audio_rate);
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

static void apbridgea_audio_dump_config(struct device_i2s_pcm *pcm,
                                        struct device_i2s_dai *dai)
{
    dbg_verbose("    pcm: format 0x%x, rate 0x%x, channels %u\n",
                pcm->format, pcm->rate, pcm->channels);
    dbg_verbose("    dai: mclk_freq %u, protocol 0x%x\n",
                dai->mclk_freq, dai->protocol);
    dbg_verbose("         wclk_polarity 0x%x, wclk_change_edge 0x%x\n",
                dai->wclk_polarity, dai->wclk_change_edge);
    dbg_verbose("         data_rx_edge 0x%x, data_tx_edge 0x%x\n",
                dai->data_rx_edge, dai->data_tx_edge);
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
        dbg_error("%s: NULL req or bad length 0x%p/%u\n", __func__, req, len);
        return -EINVAL;
    }

    if ((info->flags & APBRIDGEA_AUDIO_FLAG_TX_PREPARED) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_RX_PREPARED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
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

    dbg_verbose("%s: configuring i2s %u as MASTER\n", __func__, info->i2s_port);
    apbridgea_audio_dump_config(&pcm, &dai);

    ret = device_i2s_set_config(info->i2s_dev, DEVICE_I2S_ROLE_MASTER, &pcm,
                                &dai);
    if (ret) {
        dbg_error("%s: can't set i2s MASTER config %d\n", __func__, ret);
        return ret;
    }
#else
    dai.wclk_change_edge = DEVICE_I2S_EDGE_RISING;
    dai.data_tx_edge = DEVICE_I2S_EDGE_RISING;
    dai.data_rx_edge = DEVICE_I2S_EDGE_FALLING;

    dbg_verbose("%s: configuring i2s %u as SLAVE\n", __func__, info->i2s_port);
    apbridgea_audio_dump_config(&pcm, &dai);

    ret = device_i2s_set_config(info->i2s_dev, DEVICE_I2S_ROLE_SLAVE, &pcm,
                                &dai);
    if (ret) {
        dbg_error("%s: can't set i2s SLAVE config %d\n", __func__, ret);
        return ret;
    }
#endif

    info->flags |= APBRIDGEA_AUDIO_FLAG_SET_CONFIG;

    return 0;
}

static int apbridgea_audio_from_usb(unsigned int cportid, void *buf,
                                    size_t len, void *priv)
{
    dbg_error("%s: unexpected message for cport %d\n", __func__, cportid);
    usb_release_buffer(NULL, buf);
    return 0;
}

static int apbridgea_audio_from_unipro(unsigned int cportid, void *buf,
                                       size_t len, void *priv)
{
    struct apbridgea_audio_info *info;
    size_t hdr_size;

    info = apbridgea_audio_find_rx_info(cportid);
    if (info && (info->flags & APBRIDGEA_AUDIO_FLAG_RX_STARTED)) {
        hdr_size = sizeof(struct gb_operation_hdr) +
                   sizeof(struct gb_audio_send_data_request);

        if (len == (hdr_size + info->rx_data_size)) {
            apbridgea_audio_i2s_tx(info, buf + hdr_size);
        } else {
            dbg_error("%s: bad rx message from unipro, cport %u, len %u\n",
                      __func__, cportid, len);
        }
    } else {
        dbg_error("%s: unexpected message from unipro, cport %u, len %u\n",
                  __func__, cportid, len);
    }

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

    req->direction &= (AUDIO_APBRIDGEA_DIRECTION_TX |
                       AUDIO_APBRIDGEA_DIRECTION_RX);

    if (!data_cportid || !req->direction) {
        dbg_error("%s: bad cport or direction %u/0x%x\n", __func__,
                  data_cportid, req->direction);
        return -EINVAL;
    }

    cport = apbridgea_audio_find_cport(info, data_cportid);
    if (!cport) {
        dbg_verbose("%s: allocating cport struct, cportid %u\n", __func__,
                    data_cportid);

        cport = malloc(sizeof(*cport));
        if (!cport) {
            dbg_error("%s: can't alloc cport struct\n", __func__);
            return -ENOMEM;
        }

        cport->data_cportid = data_cportid;
        cport->direction = 0;
        cport->info = info;
        list_init(&cport->list);

        flags = irqsave();

        ret = map_offloaded_cport(get_apbridge_dev(), data_cportid,
                                  apbridgea_audio_from_unipro,
                                  apbridgea_audio_from_usb);
        if (ret) {
            irqrestore(flags);
            free(cport);
            dbg_error("%s: can't offload cport %u\n", __func__, data_cportid);
            return ret;
        }

        list_add(&info->cport_list, &cport->list);

        irqrestore(flags);
    }

    if (!info->rx_data_cportid &&
        (req->direction & AUDIO_APBRIDGEA_DIRECTION_RX)) {

        info->rx_data_cportid = data_cportid;
    }

    cport->direction |= req->direction;

    dbg_verbose("%s: registered cportid %u, direction 0x%0x/0x%x\n", __func__,
                data_cportid, req->direction, cport->direction);

    return 0;
}

static int apbridgea_audio_unregister_cport(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    struct audio_apbridgea_unregister_cport_request *req = buf;
    uint16_t data_cportid = le16_to_cpu(req->cport);
    struct apbridgea_audio_cport *cport;
    irqstate_t flags;
    int ret;

    req->direction &= (AUDIO_APBRIDGEA_DIRECTION_TX |
                       AUDIO_APBRIDGEA_DIRECTION_RX);

    if (!data_cportid || !req->direction) {
        dbg_error("%s: bad cport or direction %u/0x%x\n", __func__,
                  data_cportid, req->direction);
        return -EINVAL;
    }

    cport = apbridgea_audio_find_cport(info, data_cportid);
    if (!cport) {
        dbg_verbose("%s: cport %u not registered\n", __func__, data_cportid);
        return -EINVAL;
    }

    if ((data_cportid == info->rx_data_cportid) &&
        (req->direction & AUDIO_APBRIDGEA_DIRECTION_RX)) {

        info->rx_data_cportid = 0;
    }

    cport->direction &= ~req->direction;

    if (!cport->direction) {
        dbg_verbose("%s: freeing cport struct, cportid %u\n", __func__,
                    data_cportid);

        flags = irqsave();

        list_del(&cport->list);

        ret = unmap_offloaded_cport(get_apbridge_dev(), data_cportid);
        if (ret) {
            dbg_error("%s: can't unoffload cport %u\n", __func__, data_cportid);
        }

        irqrestore(flags);

        free(cport);
    }

    dbg_verbose("%s: unregistered cportid %u, direction 0x%x\n", __func__,
                data_cportid, req->direction);

    return 0;
}

static int apbridgea_audio_set_tx_data_size(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    struct audio_apbridgea_set_tx_data_size_request *req = buf;
    uint16_t size;

    if (!req || (len < sizeof(*req))) {
        dbg_error("%s: NULL req or bad length 0x%p/%u\n", __func__, req, len);
        return -EINVAL;
    }

    if (info->flags & APBRIDGEA_AUDIO_FLAG_TX_PREPARED) {
        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    size = le16_to_cpu(req->size);

    /* Size must be a multiple of 4 (required by i2s FIFO) */
    if (!size || (size % 4)) {
        dbg_error("%s: bad message size %u\n", __func__, size);
        return -EINVAL;
    }

    info->tx_data_size = size;
    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_DATA_SIZE_SET;

    dbg_verbose("%s: tx data size set to %u\n", __func__, size);

    return 0;
}

static int apbridgea_audio_unipro_tx_cb(int status, const void *buf, void *priv)
{
    struct ring_buf *rb = priv;
    struct apbridga_audio_rb_hdr *rb_hdr;
    struct apbridgea_audio_info *info;
    irqstate_t flags;

    rb_hdr = ring_buf_get_buf(rb);

    if (!rb_hdr->not_acked) { /* Should never happen */
        dbg_error("%s: not_acked value of zero\n", __func__);
        return 0;
    }

    flags = irqsave();

    rb_hdr->not_acked--;

    if (!rb_hdr->not_acked) {
        info = rb_hdr->info;

        if (info->flags & APBRIDGEA_AUDIO_FLAG_TX_PREPARED) {
            ring_buf_reset(rb);
            ring_buf_pass(rb);
        } else {
            sem_post(&rb_hdr->complete);
        }
    }

    irqrestore(flags);

    return 0;
}

static void apbridgea_audio_unipro_tx(struct apbridgea_audio_info *info,
                                      struct ring_buf *rb)
{
    struct apbridgea_audio_cport *cport;
    struct apbridga_audio_rb_hdr *rb_hdr;
    struct gb_operation_hdr *gb_hdr;
    struct list_head *iter;
    int ret;

    rb_hdr = ring_buf_get_buf(rb);
    gb_hdr = ring_buf_get_priv(rb);

    rb_hdr->not_acked = 0;

    list_foreach(&info->cport_list, iter) {
        cport = list_entry(iter, struct apbridgea_audio_cport, list);

        if (cport->direction & AUDIO_APBRIDGEA_DIRECTION_TX) {
            rb_hdr->not_acked++;

            ret = unipro_send_async(cport->data_cportid, gb_hdr,
                                    le16_to_cpu(gb_hdr->size),
                                    apbridgea_audio_unipro_tx_cb, rb);
            if (ret) {
                rb_hdr->not_acked--;
                dbg_verbose("%s: can't send UniPro message on cport %u, ret %u\n",
                            __func__, cport->data_cportid, ret);
            }
        }
    }
}

static void apbridgea_audio_i2s_rx_cb(struct ring_buf *rb,
                                      enum device_i2s_event event, void *arg)
{
    struct apbridgea_audio_info *info = arg;

    if (event == DEVICE_I2S_EVENT_RX_COMPLETE) {
        if (info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED) {
            apbridgea_audio_unipro_tx(info, rb);
            return;
        }
    } else if (event != DEVICE_I2S_EVENT_NONE) {
        dbg_error("%s: bad event %u from i2s ctlr %u, flags 0x%x\n", __func__,
                  event, info->i2s_port, info->flags);
    }

    if (ring_buf_is_consumers(rb)) {
        ring_buf_reset(rb);
        ring_buf_pass(rb);
    }
}

/*
 * Each ring buffer entry will point to a memory area containing:
 * - A ring buffer data header containing:
 *   - a 4-byte count field used by apbridgea_audio_send_data_completion()
 *     to tell when the last UniPro send has completed and the rb entry
 *     can be passed back to the i2s driver;
 *   - four bytes of padding so the following Greybus message is 8-byte
 *     aligned when the ring buffer data header is 8-byte aligned.
 * - The Greybus Audio Data Message containing:
 *   - the Greybus header;
 *   - the Greybus Audio Data Message header;
 *   - the audio data.
 * - Whatever extra space there is from the allocation.
 *
 * The value returned by ring_buf_get_buf() points to this area.
 * The private area (ring_buf_get_priv()/ring_buf_set_priv()) is
 * used to point to the Greybus Audio Data Message.
 *
 * The data area will be allocated from BUFRAM to make it safe for the
 * UniPro subsystem to DMA the Greybus message.  Another requirement for
 * DMA is that the Greybus message be 8-byte aligned.
 */
static int apbridgea_audio_rb_alloc(struct ring_buf *rb, void *arg)
{
    struct apbridgea_audio_info *info = arg;
    struct apbridga_audio_rb_hdr *rb_hdr;
    struct gb_operation_hdr *gb_hdr;
    void *buf;
    int ret;

    buf = bufram_page_alloc(bufram_size_to_page_count(info->tx_rb_total_size));
    if (!buf) {
        dbg_error("%s: can't alloc from bufram\n", __func__);
        return -ENOMEM;
    }

    if ((int)buf % APBRIDGEA_AUDIO_UNIPRO_ALIGNMENT) {
        dbg_error("%s: bad buf alignment 0x%p\n", __func__, buf);
        ret = -EIO;
        goto err_free_bufram;
    }

    memset(buf, 0, info->tx_rb_headroom);

    rb_hdr = buf;
    rb_hdr->info = info;

    ret = sem_init(&rb_hdr->complete, 0, 0);
    if (ret) {
        dbg_error("%s: can't init rb semaphore %d\n", __func__, ret);
        ret = -EIO;
        goto err_free_bufram;
    }

    gb_hdr = buf + sizeof(*rb_hdr);
    gb_hdr->size = cpu_to_le16(info->tx_rb_total_size - sizeof(*rb_hdr));
    gb_hdr->type = GB_AUDIO_TYPE_SEND_DATA;

    ring_buf_init(rb, buf, info->tx_rb_headroom, info->tx_data_size);
    ring_buf_set_priv(rb, gb_hdr);

    return 0;

err_free_bufram:
    bufram_page_free(buf, bufram_size_to_page_count(info->tx_rb_total_size));

    return ret;
}

static void apbridgea_audio_rb_free(struct ring_buf *rb, void *arg)
{
    struct apbridgea_audio_info *info = arg;
    struct apbridga_audio_rb_hdr *rb_hdr;

    rb_hdr = ring_buf_get_buf(rb);

    /*
     * Need to wait to make sure that the UniPro subsystem is finished using
     * the ring buffer entry (the Greybus message in the entry may have been
     * sent to multiple modules).  Note that it is guaranteed that
     * APBRIDGEA_AUDIO_FLAG_TX_PREPARED in info->flags is unset so we know
     * apbridgea_audio_unipro_tx_cb() will do a sem_post() once
     * rb_hdr->not_acked is set to zero.
     */
    if (rb_hdr->not_acked) {
        sem_wait(&rb_hdr->complete);
    }

    sem_destroy(&rb_hdr->complete);

    bufram_page_free(ring_buf_get_buf(rb),
                     bufram_size_to_page_count(info->tx_rb_total_size));
}

/* "Transmitting" means receiving from I2S and transmitting over UniPro */
static int apbridgea_audio_prepare_tx(struct apbridgea_audio_info *info)
{
    int ret;

    if (!AUDIO_IS_CONFIGURED(info, TX) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_TX_PREPARED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    info->tx_rb_headroom = sizeof(struct apbridga_audio_rb_hdr) +
                           sizeof(struct gb_operation_hdr) +
                           sizeof(struct gb_audio_send_data_request);
    info->tx_rb_total_size = info->tx_rb_headroom + info->tx_data_size;

    info->tx_rb = ring_buf_alloc_ring(APBRIDGEA_AUDIO_RING_ENTRIES_TX, 0, 0, 0,
                                      apbridgea_audio_rb_alloc,
                                      apbridgea_audio_rb_free, info);
    if (!info->tx_rb) {
        dbg_error("%s: can't alloc ring\n", __func__);
        return -ENOMEM;
    }

    ret = device_i2s_prepare_receiver(info->i2s_dev, info->tx_rb,
                                      apbridgea_audio_i2s_rx_cb, info);
    if (ret) {
        dbg_error("%s: device_i2s_prepare_receiver() failed %d\n", __func__,
                  ret);
        goto err_free_ring;
    }

    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_PREPARED;

    dbg_verbose("%s: TX prepared (I2S RX), flags 0x%x\n", __func__,
                info->flags);

    return 0;

err_free_ring:
    ring_buf_free_ring(info->tx_rb, apbridgea_audio_rb_free, info);
    info->tx_rb = NULL;
    info->tx_rb_headroom = 0;
    info->tx_rb_total_size = 0;

    return ret;
}

static int apbridgea_audio_start_tx(struct apbridgea_audio_info *info,
                                    void *buf, uint16_t len)
{
    struct audio_apbridgea_start_tx_request *req = buf;
    int ret;

    if (!req || (len < sizeof(*req))) {
        dbg_error("%s: NULL req or bad length 0x%p/%u\n", __func__, req, len);
        return -EINVAL;
    }

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_TX_PREPARED) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    info->flags |= APBRIDGEA_AUDIO_FLAG_TX_STARTED;

    dbg_verbose("%s: Starting TX (I2S RX), flags 0x%x\n", __func__,
                info->flags);

    ret = device_i2s_start_receiver(info->i2s_dev);
    if (ret) {
        dbg_error("%s: device_i2s_start_receiver() failed %d\n", __func__, ret);
        info->flags &= ~APBRIDGEA_AUDIO_FLAG_TX_STARTED;
    }

    return ret;
}

static int apbridgea_audio_stop_tx(struct apbridgea_audio_info *info)
{
    int ret;

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED)) {
        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    ret = device_i2s_stop_receiver(info->i2s_dev);
    if (ret) {
        dbg_verbose("%s: device_i2s_stop_receiver() failed %d\n", __func__,
                    ret);
    }

    info->flags &= ~APBRIDGEA_AUDIO_FLAG_TX_STARTED;

    dbg_verbose("%s: TX stopped (I2S RX), flags 0x%x\n", __func__, info->flags);

    return 0;
}

static int apbridgea_audio_shutdown_tx(struct apbridgea_audio_info *info)
{
    int ret;

    if ((info->flags & APBRIDGEA_AUDIO_FLAG_TX_STARTED) ||
        !(info->flags & APBRIDGEA_AUDIO_FLAG_TX_PREPARED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    ret = device_i2s_shutdown_receiver(info->i2s_dev);
    if (ret) {
        dbg_error("%s: device_i2s_shutdown_receiver() failed %d\n", __func__,
                  ret);
    }

    info->flags &= ~APBRIDGEA_AUDIO_FLAG_TX_PREPARED;

    ring_buf_free_ring(info->tx_rb, apbridgea_audio_rb_free, info);
    info->tx_rb = NULL;
    info->tx_rb_headroom = 0;
    info->tx_rb_total_size = 0;

    dbg_verbose("%s: TX shutdown (I2S RX), flags 0x%x\n", __func__,
                info->flags);

    return 0;
}

static int apbridgea_audio_set_rx_data_size(struct apbridgea_audio_info *info,
                                            void *buf, uint16_t len)
{
    struct audio_apbridgea_set_rx_data_size_request *req = buf;
    uint16_t size;

    if (!req || (len < sizeof(*req))) {
        dbg_error("%s: NULL req or bad length 0x%p/%u\n", __func__, req, len);
        return -EINVAL;
    }

    if (info->flags & APBRIDGEA_AUDIO_FLAG_RX_PREPARED) {
        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    size = le16_to_cpu(req->size);

    /* Size must be a multiple of 4 (required by i2s FIFO) */
    if (!size || (size % 4)) {
        dbg_error("%s: bad message size %u\n", __func__, size);
        return -EINVAL;
    }

    info->rx_data_size = size;
    info->flags |= APBRIDGEA_AUDIO_FLAG_RX_DATA_SIZE_SET;

    dbg_verbose("%s: rx data size set to %u\n", __func__, size);

    return 0;
}

static void apbridgea_audio_i2s_tx(struct apbridgea_audio_info *info,
                                   uint8_t *data)
{
    ring_buf_reset(info->rx_rb);

    memcpy(ring_buf_get_tail(info->rx_rb), data, info->rx_data_size);

    ring_buf_put(info->rx_rb, info->rx_data_size);
    ring_buf_pass(info->rx_rb);

    info->rx_rb = ring_buf_get_next(info->rx_rb);

    info->rx_rb_count++;
}

static void apbridgea_audio_i2s_tx_cb(struct ring_buf *rb,
                                      enum device_i2s_event event, void *arg)
{
    struct apbridgea_audio_info *info = arg;

    if (event == DEVICE_I2S_EVENT_TX_COMPLETE) {
        info->rx_rb_count--;

        /* Prevent underrun by adding an entry with dummy data */
        if (!info->rx_rb_count) {
            apbridgea_audio_i2s_tx(info, info->rx_dummy_data);
            dbg_error("%s: RX underrun\n", __func__);
        }
    } else if (event != DEVICE_I2S_EVENT_NONE) {
        dbg_error("%s: bad event %u from i2s ctlr %u, flags 0x%x\n", __func__,
                  event, info->i2s_port, info->flags);
    }
}

/* "Receiving" means receiving from UniPro and transmitting over I2S */
static int apbridgea_audio_prepare_rx(struct apbridgea_audio_info *info)
{
    int ret;

    if (!AUDIO_IS_CONFIGURED(info, RX) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_RX_PREPARED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    info->rx_rb = ring_buf_alloc_ring(APBRIDGEA_AUDIO_RING_ENTRIES_RX, 0,
                                      info->rx_data_size, 0, NULL, NULL, NULL);
    if (!info->rx_rb) {
        dbg_error("%s: can't alloc ring\n", __func__);
        return -ENOMEM;
    }

    info->rx_dummy_data = zalloc(info->rx_data_size);
    if (!info->rx_dummy_data) {
        dbg_error("%s: can't alloc dummy_data\n", __func__);
        ret = -ENOMEM;
        goto err_free_ring;
    }

    ret = device_i2s_prepare_transmitter(info->i2s_dev, info->rx_rb,
                                         apbridgea_audio_i2s_tx_cb, info);
    if (ret) {
        dbg_error("%s: device_i2s_prepare_transmitter() failed %d\n", __func__,
                  ret);
        goto err_free_dummy;
    }

    info->flags |= APBRIDGEA_AUDIO_FLAG_RX_PREPARED;

    /* Prime the ring with one empty entry */
    apbridgea_audio_i2s_tx(info, info->rx_dummy_data);

    dbg_verbose("%s: RX prepared (I2S TX), flags 0x%x\n", __func__,
                info->flags);

    return 0;

err_free_dummy:
    free(info->rx_dummy_data);
    info->rx_dummy_data = NULL;
err_free_ring:
    ring_buf_free_ring(info->rx_rb, apbridgea_audio_rb_free, info);
    info->rx_rb = NULL;

    return ret;
}

static int apbridgea_audio_start_rx(struct apbridgea_audio_info *info)
{
    int ret;

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_RX_PREPARED) ||
        (info->flags & APBRIDGEA_AUDIO_FLAG_RX_STARTED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    info->flags |= APBRIDGEA_AUDIO_FLAG_RX_STARTED;

    dbg_verbose("%s: Starting RX (I2S TX), flags 0x%x\n", __func__,
                info->flags);

    ret = device_i2s_start_transmitter(info->i2s_dev);
    if (ret) {
        dbg_error("%s: device_i2s_start_transmitter() failed %d\n", __func__,
                  ret);
        info->flags &= ~APBRIDGEA_AUDIO_FLAG_RX_STARTED;
    }

    return ret;
}

static int apbridgea_audio_stop_rx(struct apbridgea_audio_info *info)
{
    int ret;

    if (!(info->flags & APBRIDGEA_AUDIO_FLAG_RX_STARTED)) {
        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    ret = device_i2s_stop_transmitter(info->i2s_dev);
    if (ret) {
        dbg_error("%s: device_i2s_stop_transmitter() failed %d\n", __func__,
                  ret);
    }

    info->flags &= ~APBRIDGEA_AUDIO_FLAG_RX_STARTED;

    dbg_verbose("%s: RX stopped (I2S TX), flags 0x%x\n", __func__, info->flags);

    return 0;
}

static int apbridgea_audio_shutdown_rx(struct apbridgea_audio_info *info)
{
    int ret;

    if ((info->flags & APBRIDGEA_AUDIO_FLAG_RX_STARTED) ||
        !(info->flags & APBRIDGEA_AUDIO_FLAG_RX_PREPARED)) {

        dbg_verbose("%s: protocol error, flags 0x%x\n", __func__, info->flags);
        return -EPROTO;
    }

    ret = device_i2s_shutdown_transmitter(info->i2s_dev);
    if (ret) {
        dbg_error("%s: device_i2s_shutdown_transmitter() failed %d\n",
                  __func__, ret);
    }

    info->flags &= ~APBRIDGEA_AUDIO_FLAG_RX_PREPARED;

    free(info->rx_dummy_data);
    info->rx_dummy_data = NULL;

    ring_buf_free_ring(info->rx_rb, NULL, NULL);
    info->rx_rb = NULL;

    dbg_verbose("%s: RX shutdown (I2S TX), flags 0x%x\n", __func__,
                info->flags);

    return 0;
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
            dbg_error("%s: sem_wait failed %d\n", __func__, ret);
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
            dbg_error("%s: NULL pkt or bad length 0x%p/%u\n", __func__, pkt_hdr,
                      len);
            free(demux_entry);
            continue;
        }

        info = apbridgea_audio_find_info(pkt_hdr->i2s_port);
        if (!info) {
            dbg_error("%s: no info struct() %u\n", __func__, pkt_hdr->i2s_port);
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
        case AUDIO_APBRIDGEA_TYPE_PREPARE_TX:
            ret = apbridgea_audio_prepare_tx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_START_TX:
            ret = apbridgea_audio_start_tx(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_STOP_TX:
            ret = apbridgea_audio_stop_tx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_SHUTDOWN_TX:
            ret = apbridgea_audio_shutdown_tx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_SET_RX_DATA_SIZE:
            ret = apbridgea_audio_set_rx_data_size(info, pkt_hdr, len);
            break;
        case AUDIO_APBRIDGEA_TYPE_PREPARE_RX:
            ret = apbridgea_audio_prepare_rx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_START_RX:
            ret = apbridgea_audio_start_rx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_STOP_RX:
            ret = apbridgea_audio_stop_rx(info);
            break;
        case AUDIO_APBRIDGEA_TYPE_SHUTDOWN_RX:
            ret = apbridgea_audio_shutdown_rx(info);
            break;
        default:
            dbg_error("%s: invalid pkt type %u\n", __func__, pkt_hdr->type);
        }

        free(demux_entry);
    }

    /* NOTREACHED */
    return NULL;
}

int apbridgea_audio_out_demux(void *buf, uint16_t len)
{
    struct apbridgea_audio_demux_entry *demux_entry;
    int ret;

    if (!buf || !len) {
        dbg_error("%s: NULL buf or bad length 0x%p/%u\n", __func__, buf, len);
        return -EINVAL;
    }

    demux_entry = malloc(sizeof(*demux_entry) + len);
    if (!demux_entry) {
        dbg_error("%s: can't alloc demux_entry\n", __func__);
        return -ENOMEM;
    }

    demux_entry->buf = (uint8_t *)demux_entry + sizeof(*demux_entry);
    demux_entry->len = len;

    memcpy(demux_entry->buf, buf, len);

    list_init(&demux_entry->list);

    /* In irq context */
    list_add(&apbridgea_audio_demux_list, &demux_entry->list);

    ret = sem_post(&apbridgea_audio_demux_sem);
    if (ret != OK) {
        dbg_error("%s: sem_post failed %d\n", __func__, ret);
    }

    dbg_insane("%s: request received from AP, len %u\n", __func__, len);

    return 0;
}

/* Only support one I2S port right now (but multiple TX CPorts) */
int apbridgea_audio_init(void)
{
    struct apbridgea_audio_info *info;
    int ret;

    dbg_verbose("%s: initializing audio driver\n", __func__);

    if (sizeof(struct apbridga_audio_rb_hdr) %
        APBRIDGEA_AUDIO_UNIPRO_ALIGNMENT) {

        dbg_error("%s: apbridga_audio_rb_hdr wrong size %u\n", __func__,
                  sizeof(struct apbridga_audio_rb_hdr));
        return -EIO;
    }

    info = zalloc(sizeof(*info));
    if (!info) {
        dbg_error("%s: can't alloc info struct\n", __func__);
        return -ENOMEM;
    }

    info->i2s_port = 0; /* TODO: Get from init_data */

    info->i2s_dev = device_open(DEVICE_TYPE_I2S_HW, info->i2s_port);
    if (!info->i2s_dev) {
        dbg_error("%s: can't open i2s ctlr()\n", __func__);
        ret = -EIO;
        goto err_free_info;
    }

    ret = sem_init(&apbridgea_audio_demux_sem, 0, 0);
    if (ret) {
        dbg_error("%s: sem_init() failed %d\n", __func__, ret);
        goto err_close_i2s;
    }

    ret = pthread_create(&apbridgea_audio_demux_thread, NULL,
                         apbridgea_audio_demux, NULL);
    if (ret) {
        dbg_error("%s: pthread_create() failed %d\n", __func__, ret);
        goto err_destroy_sem;
    }

    list_init(&info->cport_list);
    list_init(&info->list);

    list_add(&apbridgea_audio_info_list, &info->list);

    dbg_verbose("%s: audio driver initialized\n", __func__);

    return 0;

err_destroy_sem:
    sem_destroy(&apbridgea_audio_demux_sem);
err_close_i2s:
    device_close(info->i2s_dev);
err_free_info:
    free(info);

    return ret;
}
