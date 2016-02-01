/*
 * Copyright (c) 2015, 2016 Google Inc.
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
 * @author: Perry Hung
 */

#define DBG_COMP    ARADBG_SWITCH

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/unipro/unipro.h>

#include <pthread.h>
#include <errno.h>
#include <string.h>

#include <arch/byteorder.h>

#include <ara_debug.h>
#include "tsb_switch.h"
#include "tsb_es2_mphy_fixups.h"

#define SWITCH_SPI_INIT_DELAY   (700)   // us

/* Max NULL frames to wait for a reply from the switch */
#define ES2_SWITCH_WAIT_REPLY_LEN   (16)
/* Write status reply length */
#define ES2_SWITCH_WRITE_STATUS_LEN (11)
/* Total number of NULLs to clock out to ensure a write status is read */
#define ES2_SWITCH_WRITE_STATUS_NNULL (ES2_SWITCH_WAIT_REPLY_LEN + \
                                       ES2_SWITCH_WRITE_STATUS_LEN)

/* Interrupt enable bits */
#define ES2_SPICEE_ENABLE_ALL        (0x3)
#define ES2_SPI3EE_ENABLE_ALL        (0x6F)
#define ES2_SPI45EE_ENABLE_ALL       (0xFE00006F)
/* Device ID mask set is the longest NCP request. */
#define ES2_NCP_MAX_REQ_SIZE 19
/* Device ID masks are per-port, with 16 bytes of mask for each port. */
#define ES2_DEV_ID_MASK_SIZE 16
/* 16-byte max delay + 5-byte header + 272-byte max payload + 2-byte footer */
#define ES2_CPORT_RX_MAX_SIZE        (16 + 5 + 272 + 2)
#define ES2_CPORT_NCP_MAX_PAYLOAD    (256)
#define ES2_CPORT_DATA_MAX_PAYLOAD   (272)
/* CPorts maximum FIFO data size */
#define ES2_CPORT_NCP_FIFO_SIZE      (256)
#define ES2_CPORT_DATA_FIFO_SIZE     (576)

struct es2_cport {
    pthread_mutex_t lock;
    uint8_t rxbuf[ES2_CPORT_RX_MAX_SIZE];
};

struct sw_es2_priv {
    struct spi_dev_s    *spi_dev;
    struct es2_cport ncp_cport;
    struct es2_cport data_cport4;
    struct es2_cport data_cport5;
};

#define CPORT_NCP   (0x03)
#define CPORT_DATA4 (0x04)
#define CPORT_DATA5 (0x05)

static inline uint8_t *cport_to_rxbuf(struct sw_es2_priv *priv, unsigned int cportid) {
    switch (cportid) {
    case CPORT_NCP:
        return priv->ncp_cport.rxbuf;
    case CPORT_DATA4:
        return priv->data_cport4.rxbuf;
    case CPORT_DATA5:
        return priv->data_cport5.rxbuf;
    };
    return NULL;
}

static void es2_set_valid_entry(struct tsb_switch *sw,
                                uint8_t *table, int entry, bool valid) {
    if (valid) {
        table[15 - ((entry) / 8)] |= (1 << ((entry)) % 8);
    } else {
        table[15 - ((entry) / 8)] &= ~(1 << ((entry)) % 8);
    }
}

static bool es2_check_valid_entry(struct tsb_switch *sw,
                                  uint8_t *table, int entry) {
    return table[15 - ((entry) / 8)] & (1 << ((entry)) % 8);
}

/* 19 bytes for entire report (7+12) and max 16 bytes of delay */
#define SRPT_SIZE           (19 + 16)
#define SRPT_REPORT_SIZE    (12)

struct __attribute__ ((__packed__)) srpt_read_status_report {
    unsigned char raw[SRPT_REPORT_SIZE];
    size_t rx_fifo_size; // number of bytes available in the rx buffer
    size_t tx_fifo_size; // bytes available for enqueue in the tx buffer
};

/**
 * @brief retrieve the state of the cport spi fifo
 */
static int es2_read_status(struct tsb_switch *sw,
                           unsigned int cport,
                           struct srpt_read_status_report *status) {
    struct sw_es2_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    unsigned int offset;
    struct srpt_read_status_report *rpt = NULL;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cport);

    const char srpt_cmd[] = {HNUL, SRPT, cport, 0, 0, ENDP, HNUL};
    const char srpt_report_header[] = {HNUL, SRPT, cport, 0x00, 0xC};

    _switch_spi_select(sw, true);
    SPI_SNDBLOCK(spi_dev, srpt_cmd, sizeof srpt_cmd);
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, SRPT_SIZE);
    _switch_spi_select(sw, false);

    /* Find the header */
    for (offset = 0; offset < (SRPT_SIZE - SRPT_REPORT_SIZE); offset++) {
        if (!memcmp(srpt_report_header,
                    &rxbuf[offset],
                    sizeof srpt_report_header)) {
            /* jump past the header */
            rpt = (struct srpt_read_status_report *)
                  &rxbuf[offset + sizeof srpt_report_header];
            break;
        }
    }

    if (!rpt) {
        return -ENOMSG;
    }

    /* Fill in the report and parse useful fields */
    if (status) {
        size_t fifo_max_size;

        memcpy(status, rpt, SRPT_REPORT_SIZE);

        switch (cport) {
        case CPORT_NCP:
            fifo_max_size = ES2_CPORT_NCP_FIFO_SIZE;
            break;
        case CPORT_DATA4:
        case CPORT_DATA5:
        default:
            fifo_max_size = ES2_CPORT_DATA_FIFO_SIZE;
            break;
        }

        status->tx_fifo_size = rpt->raw[7] * 8;
        status->rx_fifo_size = fifo_max_size -
                               ((rpt->raw[10] << 8) | rpt->raw[11]);
    }

    return 0;
}

static int es2_write(struct tsb_switch *sw,
                     uint8_t cportid,
                     uint8_t *tx_buf,
                     size_t tx_size) {
    struct sw_es2_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    struct srpt_read_status_report rpt;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cportid);
    unsigned int size;
    int ret = OK;

    uint8_t write_header[] = {
        LNUL,
        STRW,
        cportid,
        (tx_size & 0xFF00) >> 8,
        (tx_size & 0xFF),
    };

    uint8_t write_trailer[] = {
        ENDP,
        LNUL,
    };

    switch (cportid) {
    case CPORT_NCP:
        if (tx_size >= ES2_CPORT_NCP_MAX_PAYLOAD) {
            return -ENOMEM;
        }
        break;
    case CPORT_DATA4:
    case CPORT_DATA5:
        /*
         * Must read the fifo status for data to ensure there is enough space.
         */
        ret = es2_read_status(sw, cportid, &rpt);
        if (ret) {
            return ret;
        }

        /* messages greater than one fifo's worth are not supported */
        if (tx_size >= ES2_CPORT_DATA_MAX_PAYLOAD) {
            return -EINVAL;
        }

        if (tx_size >= rpt.tx_fifo_size) {
            return -ENOMEM;
        }
        break;
    default:
        return -EINVAL;
    }

    _switch_spi_select(sw, true);
    /* Write */
    SPI_SNDBLOCK(spi_dev, write_header, sizeof write_header);
    SPI_SNDBLOCK(spi_dev, tx_buf, tx_size);
    SPI_SNDBLOCK(spi_dev, write_trailer, sizeof write_trailer);
    // Wait write status, send NULL frames while waiting
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, ES2_SWITCH_WRITE_STATUS_NNULL);

    dbg_insane("Write payload:\n");
    dbg_print_buf(ARADBG_INSANE, tx_buf, tx_size);
    dbg_insane("Write status:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, ES2_SWITCH_WRITE_STATUS_NNULL);

    // Make sure we use 16-bit frames
    size = sizeof write_header + tx_size + sizeof write_trailer
           + ES2_SWITCH_WRITE_STATUS_NNULL;
    if (size % 2) {
        SPI_SEND(spi_dev, LNUL);
    }

    // Parse the write status and bail on error.
    ret = _switch_transfer_check_write_status(rxbuf,
                                              ES2_SWITCH_WRITE_STATUS_NNULL);
    if (ret) {
        goto out;
    }

out:
    _switch_spi_select(sw, false);
    return ret;
}

static int es2_read(struct tsb_switch *sw,
                    uint8_t cportid,
                    uint8_t *rx_buf,
                    size_t rx_size) {
    struct sw_es2_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cportid);
    size_t size;
    int rcv_done = 0;
    bool null_rxbuf;
    int ret = 0;

    uint8_t read_header[] = {
        LNUL,
        STRR,
        cportid,
        0,      // LENM
        0,      // LENL
        ENDP,
        LNUL
    };

    _switch_spi_select(sw, true);

    // Read CNF and retry if NACK received
    do {
        // Read the CNF
        size = ES2_SWITCH_WAIT_REPLY_LEN + rx_size + sizeof read_header;
        SPI_SNDBLOCK(spi_dev, read_header, sizeof read_header);
        SPI_EXCHANGE(spi_dev, NULL, rxbuf, size - sizeof read_header);
        /* Make sure we use 16-bit frames */
        if (size & 0x1) {
            SPI_SEND(spi_dev, LNUL);
        }

        dbg_insane("RX Data:\n");
        dbg_print_buf(ARADBG_INSANE, rxbuf, size);

        if (!rx_buf) {
            break;
        }

        /*
         * Find the STRR and copy the response; handle other cases:
         * NACK, switch not responding.
         *
         * In some cases (e.g. wrong function ID in the NCP command) the
         * switch does respond on the command with all NULs.
         * In that case bail out with error.
         */
        uint8_t *resp_start = NULL;
        unsigned int i;
        null_rxbuf = true;

        for (i = 0; i < size; i++) {
            // Detect an all-[LH]NULs RX buffer
            if ((rxbuf[i] != LNUL) && (rxbuf[i] != HNUL)) {
                null_rxbuf = false;
            }
            // Check for STRR or NACK
            if (rxbuf[i] == STRR) {
                // STRR found, parse the reply length and data
                resp_start = &rxbuf[i];
                size_t resp_len = resp_start[2] << 8 | resp_start[3];
                memcpy(rx_buf, &resp_start[4], resp_len);
                rcv_done = 1;
                break;
            } else if (rxbuf[i] == NACK) {
                // NACK found, retry the CNF read
                break;
            }
        }

        // If all NULs in RX buffer, bail out with error code
        if (null_rxbuf) {
            ret = -EIO;
            dbg_error("Switch not responding, aborting command\n");
        }

    } while (!rcv_done && !null_rxbuf);

    _switch_spi_select(sw, false);

    return ret;
}

static int es2_ncp_transfer(struct tsb_switch *sw,
                            uint8_t *tx_buf,
                            size_t tx_size,
                            uint8_t *rx_buf,
                            size_t rx_size) {
    struct sw_es2_priv *priv = sw->priv;
    int rc;

    pthread_mutex_lock(&priv->ncp_cport.lock);

    /* Send the request */
    rc = es2_write(sw, CPORT_NCP, tx_buf, tx_size);
    if (rc) {
        dbg_error("%s() write failed: rc=%d\n", __func__, rc);
        goto done;
    }

    /* Read the CNF */
    rc = es2_read(sw, CPORT_NCP, rx_buf, rx_size);
    if (rc) {
        dbg_error("%s() read failed: rc=%d\n", __func__, rc);
        goto done;
    }

done:
    pthread_mutex_unlock(&priv->ncp_cport.lock);

    return rc;
}

static int es2_irq_fifo_rx(struct tsb_switch *sw, unsigned int cportid) {
    struct sw_es2_priv *priv = sw->priv;
    struct es2_cport *cport;
    struct srpt_read_status_report rpt;
    size_t len;
    int rc;

    switch (cportid) {
    case CPORT_DATA4:
        cport = &priv->data_cport4;
        break;
    case CPORT_DATA5:
        cport = &priv->data_cport5;
        break;
    default:
        return -EINVAL;
    }

    pthread_mutex_lock(&cport->lock);

    rc = es2_read_status(sw, cportid, &rpt);
    if (rc) {
        rc = -EIO;
        goto fill_done;
    }
    len = rpt.rx_fifo_size;

    /*
     * Drain the fifo data.
     */
    rc = es2_read(sw, cportid, cport->rxbuf, len);
    if (rc) {
        dbg_error("%s: read failed: %d\n", __func__, rc);
        rc = -EIO;
        goto fill_done;
    }
    dbg_print_buf(ARADBG_VERBOSE, cport->rxbuf, len);

fill_done:
    pthread_mutex_unlock(&cport->lock);
    if (rc) {
        return rc;
    }

    /* Give it to unipro. */
    unipro_if_rx(cportid, cport->rxbuf, len);

    return 0;
}

/* Switch communication init procedure */
int es2_init_seq(struct tsb_switch *sw)
{
    struct sw_es2_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    const char init_reply[] = { INIT, LNUL };
    uint8_t *rxbuf = cport_to_rxbuf(priv, CPORT_NCP);
    int i, rc = -1;

    dbg_info("Initializing ES2 switch...\n");
    _switch_spi_select(sw, true);

    // Delay needed before the switch is ready on the SPI bus
    up_udelay(SWITCH_SPI_INIT_DELAY);

    SPI_SEND(spi_dev, INIT);
    SPI_SEND(spi_dev, INIT);
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, ES2_SWITCH_WAIT_REPLY_LEN);

    dbg_insane("Init RX Data:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, ES2_SWITCH_WAIT_REPLY_LEN);

    // Check for the transition from INIT to LNUL after sending INITs
    for (i = 0; i < ES2_SWITCH_WAIT_REPLY_LEN - 1; i++) {
        if (!memcmp(rxbuf + i, init_reply, sizeof(init_reply)))
            rc = 0;
    }

    _switch_spi_select(sw, false);

    if (rc) {
        dbg_error("%s: Failed to init the SPI link with the switch\n",
                  __func__);
        return rc;
    }
    dbg_info("... Done!\n");

    rc = tsb_switch_es2_fixup_mphy(sw);
    if (rc) {
        dbg_error("%s: failed to fix up the M-PHY attributes\n",
                  __func__);
    }

    return rc;
}

/*
 * NCP command request helpers.
 */

static void es2_dme_set_req(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint16_t attrid,
                            uint16_t select_index,
                            uint32_t val,
                            uint8_t *req, size_t *req_size) {
    uint8_t set_req[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_SETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };
    DEBUGASSERT(*req_size >= sizeof(set_req));
    memcpy(req, set_req, sizeof(set_req));
    *req_size = sizeof(set_req);
}

static void es2_dme_get_req(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint16_t attrid,
                            uint16_t select_index,
                            uint8_t *req, size_t *req_size) {
    uint8_t get_req[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_GETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };
    DEBUGASSERT(*req_size >= sizeof(get_req));
    memcpy(req, get_req, sizeof(get_req));
    *req_size = sizeof(get_req);
}

static void es2_dme_peer_set_req(struct tsb_switch *sw,
                                 uint8_t port_id,
                                 uint16_t attrid,
                                 uint16_t select_index,
                                 uint32_t val,
                                 uint8_t *req, size_t *req_size) {
    uint8_t peer_set[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_PEERSETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };
    DEBUGASSERT(*req_size >= sizeof(peer_set));
    memcpy(req, peer_set, sizeof(peer_set));
    *req_size = sizeof(peer_set);
}

static void es2_dme_peer_get_req(struct tsb_switch *sw,
                                 uint8_t port_id,
                                 uint16_t attrid,
                                 uint16_t select_index,
                                 uint8_t *req, size_t *req_size) {
    uint8_t peer_get[] = {
        SWITCH_DEVICE_ID,
        port_id,
        NCP_PEERGETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };
    DEBUGASSERT(*req_size >= sizeof(peer_get));
    memcpy(req, peer_get, sizeof(peer_get));
    *req_size = sizeof(peer_get);
}

static void es2_lut_set_req(struct tsb_switch *sw,
                            uint8_t unipro_portid,
                            uint8_t lut_address,
                            uint8_t dest_portid,
                            uint8_t *req, size_t *req_size) {
    uint8_t lut_set[] = {
        SWITCH_DEVICE_ID,
        lut_address,
        NCP_LUTSETREQ,
        unipro_portid,
        dest_portid
    };
    DEBUGASSERT(*req_size >= sizeof(lut_set));
    memcpy(req, lut_set, sizeof(lut_set));
    *req_size = sizeof(lut_set);
}

static void es2_lut_get_req(struct tsb_switch *sw,
                            uint8_t unipro_portid,
                            uint8_t lut_address,
                            uint8_t *req, size_t *req_size) {
    uint8_t lut_get[] = {
        SWITCH_DEVICE_ID,
        lut_address,
        NCP_LUTGETREQ,
        unipro_portid,
        NCP_RESERVED,
    };
    DEBUGASSERT(*req_size >= sizeof(lut_get));
    memcpy(req, lut_get, sizeof(lut_get));
    *req_size = sizeof(lut_get);
}

static void es2_switch_attr_set_req(struct tsb_switch *sw,
                                    uint16_t attrid,
                                    uint32_t val,
                                    uint8_t *req, size_t *req_size) {
    uint8_t switch_attr_set[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SWITCHATTRSETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };
    DEBUGASSERT(*req_size >= sizeof(switch_attr_set));
    memcpy(req, switch_attr_set, sizeof(switch_attr_set));
    *req_size = sizeof(switch_attr_set);
}

static void es2_switch_attr_get_req(struct tsb_switch *sw,
                                    uint16_t attrid,
                                    uint8_t *req, size_t *req_size) {
    uint8_t switch_attr_get[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SWITCHATTRGETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
    };
    DEBUGASSERT(*req_size >= sizeof(switch_attr_get));
    memcpy(req, switch_attr_get, sizeof(switch_attr_get));
    *req_size = sizeof(switch_attr_get);
}

static void es2_sys_ctrl_set_req(struct tsb_switch *sw,
                                 uint16_t sc_addr,
                                 uint32_t val,
                                 uint8_t *req, size_t *req_size)
{
    uint8_t sys_ctrl_set[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SYSCTRLSETREQ,
        sc_addr >> 8,
        sc_addr & 0xff,
        val >> 24,
        (val >> 16) & 0xff,
        (val >> 8) & 0xff,
        val & 0xff,
    };
    DEBUGASSERT(*req_size >= sizeof(sys_ctrl_set));
    memcpy(req, sys_ctrl_set, sizeof(sys_ctrl_set));
    *req_size = sizeof(sys_ctrl_set);
}

static void es2_sys_ctrl_get_req(struct tsb_switch *sw,
                                 uint16_t sc_addr,
                                 uint8_t *req, size_t *req_size)
{
    uint8_t sys_ctrl_get[] = {
        SWITCH_DEVICE_ID,
        NCP_RESERVED,
        NCP_SYSCTRLGETREQ,
        sc_addr >> 8,
        sc_addr & 0xff,
    };
    DEBUGASSERT(*req_size >= sizeof(sys_ctrl_get));
    memcpy(req, sys_ctrl_get, sizeof(sys_ctrl_get));
    *req_size = sizeof(sys_ctrl_get);
}

static void es2_qos_attr_set_req(struct tsb_switch *sw,
                                 uint8_t  portid,
                                 uint8_t  attrid,
                                 uint32_t attr_val,
                                 uint8_t *req, size_t *req_size) {
    uint8_t qos_attr_set[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_QOSATTRSETREQ,
        NCP_RESERVED,
        attrid,
        (attr_val & 0xff000000) >> 24,
        (attr_val & 0xff0000) >> 16,
        (attr_val & 0xff00) >> 8,
        (attr_val & 0xff) >> 0
    };
    DEBUGASSERT(*req_size >= sizeof(qos_attr_set));
    memcpy(req, qos_attr_set, sizeof(qos_attr_set));
    *req_size = sizeof(qos_attr_set);
}

static void es2_qos_attr_get_req(struct tsb_switch *sw,
                                 uint8_t portid,
                                 uint8_t attrid,
                                 uint8_t *req, size_t *req_size) {
    uint8_t qos_attr_get[] = {
        SWITCH_DEVICE_ID,
        portid,
        NCP_QOSATTRGETREQ,
        NCP_RESERVED,
        attrid
    };
    DEBUGASSERT(*req_size >= sizeof(qos_attr_get));
    memcpy(req, qos_attr_get, sizeof(qos_attr_get));
    *req_size = sizeof(qos_attr_get);
}

static void es2_dev_id_mask_set_req(struct tsb_switch *sw,
                                    uint8_t unipro_portid,
                                    uint8_t *mask,
                                    uint8_t *req, size_t *req_size) {
    struct __attribute__ ((__packed__)) {
        uint8_t dest_deviceid;
        uint8_t portid;
        uint8_t function_id;
        uint8_t mask[ES2_DEV_ID_MASK_SIZE];
    } dev_id_mask_set = {
        SWITCH_DEVICE_ID,
        unipro_portid,
        NCP_SETDEVICEIDMASKREQ,
    };
    DEBUGASSERT(*req_size >= sizeof(dev_id_mask_set));
    memcpy(&dev_id_mask_set.mask, mask, sizeof(dev_id_mask_set.mask));
    memcpy(req, &dev_id_mask_set, sizeof(dev_id_mask_set));
    *req_size = sizeof(dev_id_mask_set);
}

static void es2_dev_id_mask_get_req(struct tsb_switch *sw,
                                    uint8_t unipro_portid,
                                    uint8_t *req, size_t *req_size) {
    uint8_t dev_id_mask_get[] = {
        SWITCH_DEVICE_ID,
        unipro_portid,
        NCP_GETDEVICEIDMASKREQ,
    };
    DEBUGASSERT(*req_size >= sizeof(dev_id_mask_get));
    memcpy(req, dev_id_mask_get, sizeof(dev_id_mask_get));
    *req_size = sizeof(dev_id_mask_get);
}

static void es2_switch_id_set_req(struct tsb_switch *sw,
                                  uint8_t cportid,
                                  uint8_t peer_cportid,
                                  uint8_t dis,
                                  uint8_t irt,
                                  uint8_t *req, size_t *req_size) {
    uint8_t switch_id_set[] = {
        SWITCH_DEVICE_ID,
        (dis << 2) | (irt << 0),
        NCP_SWITCHIDSETREQ,
        SWITCH_DEVICE_ID,
        cportid,            // L4 CPortID
        SWITCH_DEVICE_ID,
        peer_cportid,
        NCP_RESERVED,
        SWITCH_PORT_ID      // Source portID
    };
    DEBUGASSERT(*req_size >= sizeof(switch_id_set));
    memcpy(req, switch_id_set, sizeof(switch_id_set));
    *req_size = sizeof(switch_id_set);
}

/**
 * @brief Send raw data down CPort 4. We're only using this for the SVC
 * connection, so fix the CPort number.
 */
static int es2_data_send(struct tsb_switch *sw, void *data, size_t len) {
    struct sw_es2_priv *priv = sw->priv;
    int rc;

    pthread_mutex_lock(&priv->data_cport4.lock);

    rc = es2_write(sw, CPORT_DATA4, data, len);
    if (rc) {
        dbg_error("%s() write failed: rc=%d\n", __func__, rc);
        goto done;
    }

done:
    pthread_mutex_unlock(&priv->data_cport4.lock);

    return rc;
}

static struct tsb_rev_data es2_rev_data = {
    .ncp_req_max_size       = ES2_NCP_MAX_REQ_SIZE,
    .dev_id_mask_size       = ES2_DEV_ID_MASK_SIZE,
    .spicee_enable_all      = ES2_SPICEE_ENABLE_ALL,
    .spi3ee_enable_all      = ES2_SPI3EE_ENABLE_ALL,
    .spi45ee_enable_all     = ES2_SPI45EE_ENABLE_ALL,
};

static struct tsb_switch_ops es2_ops = {
    .init_comm             = es2_init_seq,

    .switch_id_set_req     = es2_switch_id_set_req,

    .set_req               = es2_dme_set_req,
    .get_req               = es2_dme_get_req,

    .peer_set_req          = es2_dme_peer_set_req,
    .peer_get_req          = es2_dme_peer_get_req,

    .lut_set_req           = es2_lut_set_req,
    .lut_get_req           = es2_lut_get_req,

    .switch_attr_get_req   = es2_switch_attr_get_req,
    .switch_attr_set_req   = es2_switch_attr_set_req,

    .sys_ctrl_set_req      = es2_sys_ctrl_set_req,
    .sys_ctrl_get_req      = es2_sys_ctrl_get_req,

    .qos_attr_set_req      = es2_qos_attr_set_req,
    .qos_attr_get_req      = es2_qos_attr_get_req,

    .dev_id_mask_get_req   = es2_dev_id_mask_get_req,
    .dev_id_mask_set_req   = es2_dev_id_mask_set_req,

    .switch_data_send      = es2_data_send,

    .__irq_fifo_rx         = es2_irq_fifo_rx,
    .__ncp_transfer        = es2_ncp_transfer,
    .__set_valid_entry     = es2_set_valid_entry,
    .__check_valid_entry   = es2_check_valid_entry,
};

int tsb_switch_es2_init(struct tsb_switch *sw, struct spi_dev_s *spi_dev)
{
    struct sw_es2_priv *priv;
    int rc = 0;

    priv = malloc(sizeof(struct sw_es2_priv));
    if (!priv) {
        dbg_error("%s: Failed to alloc the priv struct\n", __func__);
        rc = -ENOMEM;
        goto error;
    }

    priv->spi_dev = spi_dev;
    pthread_mutex_init(&priv->ncp_cport.lock, NULL);
    pthread_mutex_init(&priv->data_cport4.lock, NULL);
    pthread_mutex_init(&priv->data_cport5.lock, NULL);

    sw->priv = priv;
    sw->ops = &es2_ops;
    sw->rdata = &es2_rev_data;

    return rc;

error:
    tsb_switch_es2_exit(sw);
    return rc;
}

void tsb_switch_es2_exit(struct tsb_switch *sw) {
    struct sw_es2_priv *priv;

    if (!sw)
        return;

    priv = sw->priv;
    free(priv);
    sw->priv = NULL;
    sw->ops = NULL;
}
