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

#include "stm32.h"
#include <ara_debug.h>
#include "tsb_switch.h"
#include "tsb_switch_driver_es3.h"
#include "tsb_switch_event.h"

#define SWITCH_SPI_INIT_DELAY       (700)   // us

// Max number of device IDs in the switch
#define ES3_DEVICEID_MAX            31

#define ES3_IRQ_MAX                 16
/* 9-byte max delay + 4-byte header + 272-byte max payload + 1-byte footer */
#define ES3_CPORT_RX_MAX_SIZE       (9 + 4 + 272 + 1)
#define ES3_CPORT_NCP_MAX_PAYLOAD   (256)
#define ES3_CPORT_DATA_MAX_PAYLOAD  (272)
/* CPorts maximum FIFO data size */
#define ES3_CPORT_NCP_FIFO_SIZE     (256)
#define ES3_CPORT_DATA_FIFO_SIZE    (576)

/* Max NULL frames to wait for a reply from the switch */
#define SWITCH_WAIT_REPLY_LEN       (9)
/* NCP read data command: 4 bytes header + END */
#define SWITCH_NCP_READ_DATA_LEN    (5)
/* Write status reply length */
#define SWITCH_WRITE_STATUS_LEN     (9)
/* Returned session data header/trailer: 4 bytes header + END */
#define SWITCH_DATA_READ_LEN        (5)
/* Total number of NULLs to clock out to ensure a write status is read */
#define SWITCH_WRITE_STATUS_NNULL   (SWITCH_WAIT_REPLY_LEN + \
                                     SWITCH_WRITE_STATUS_LEN)
/* Total number of NULLs to clock out to ensure the returned NCP is read */
#define SWITCH_NCP_READ_LEN         (SWITCH_WAIT_REPLY_LEN + \
                                     SWITCH_NCP_READ_DATA_LEN)
/* Status report data size */
#define SRPT_REPORT_LEN             (12)
/* Status report total size: 7 bytes header + data + Switch reply delay */
#define SRPT_SIZE                   (7 + SRPT_REPORT_LEN + \
                                     SWITCH_WAIT_REPLY_LEN)

struct es3_cport {
    pthread_mutex_t lock;
    uint8_t rxbuf[ES3_CPORT_RX_MAX_SIZE];
};

struct sw_es3_priv {
    struct spi_dev_s *spi_dev;
    struct es3_cport ncp_cport;
    struct es3_cport data_cport4;
    struct es3_cport data_cport5;
};

#define LNUL        (0x00)
#define STRW        (0x01)
#define STRR        (0x02)
#define SRPT        (0x03)
#define NACK        (0x04)
#define STRD        (0x05)
#define ENDP        (0x06)
#define ENDD        (0x07)
#define INIT        (0x08)
#define HNUL        (0xff)

#define CPORT_NCP   (0x03)
#define CPORT_DATA4 (0x04)
#define CPORT_DATA5 (0x05)

#define WSTATUS0_TXENTFIFOREMAIN_MASK  (0x03)
#define WSTATUS1_TXENTFIFOREMAIN_MASK  (0xff)
#define WSTATUS2_TXDATAFIFOREMAIN_MASK (0x7f)
#define WSTATUS3_LENERR_MASK           (0x40)
#define WSTATUS3_TXENTFIFOFULL_MASK    (0x02)
#define WSTATUS3_TXDATAFIFOFULL_MASK   (0x01)

#define IRQ_STATUS_ENDPOINTRESETIND     (0x0)
#define IRQ_STATUS_LINKSTARTUPIND       (0x1)
#define IRQ_STATUS_LINKLOSTIND          (0x2)
#define IRQ_STATUS_HIBERNATEENTERIND    (0x3)
#define IRQ_STATUS_HIBERNATEEXITIND     (0x4)
#define IRQ_STATUS_POWERMODEIND         (0x5)
#define IRQ_STATUS_TESTMODEIND          (0x6)
#define IRQ_STATUS_ERRORPHYIND          (0x7)
#define IRQ_STATUS_ERRORPAIND           (0x8)
#define IRQ_STATUS_ERRORDIND            (0x9)
#define IRQ_STATUS_ERRORNIND            (0xa)
#define IRQ_STATUS_ERRORTIND            (0xb)
#define IRQ_STATUS_PAINITERROR          (0xc)
#define IRQ_STATUS_DEBUGCOUNTEROVERFLOW (0xd)
#define IRQ_STATUS_LINKSTARTUPCNF       (0xe)
#define IRQ_STATUS_MAILBOX              (0xf)

/* Attributes as sources of interrupts from the Unipro ports */
static uint16_t unipro_irq_attr[ES3_IRQ_MAX] = {
    [IRQ_STATUS_ENDPOINTRESETIND]     = TSB_DME_ENDPOINTRESETIND,
    [IRQ_STATUS_LINKSTARTUPIND]       = TSB_DME_LINKSTARTUPIND,
    [IRQ_STATUS_LINKLOSTIND]          = TSB_DME_LINKLOSTIND,
    [IRQ_STATUS_HIBERNATEENTERIND]    = TSB_DME_HIBERNATEENTERIND,
    [IRQ_STATUS_HIBERNATEEXITIND]     = TSB_DME_HIBERNATEEXITIND,

    /*
     * FIXME: hackaround until SW-1237 is implemented.
     *
     * The proper attribute is TSB_DME_POWERMODEIND, but that's
     * currently polled by switch_apply_power_mode() to determine when
     * a power mode change is done, after writing PA_PWRMODE. We would
     * race with that thread if we read it in the IRQ worker.
     */
    [IRQ_STATUS_POWERMODEIND]         = 0, /* TSB_DME_POWERMODEIND */

    [IRQ_STATUS_TESTMODEIND]          = TSB_DME_TESTMODEIND,
    [IRQ_STATUS_ERRORPHYIND]          = TSB_DME_ERRORPHYIND,
    [IRQ_STATUS_ERRORPAIND]           = TSB_DME_ERRORPAIND,
    [IRQ_STATUS_ERRORDIND]            = TSB_DME_ERRORDIND,
    [IRQ_STATUS_ERRORNIND]            = 0, /* Not recommended to read */
    [IRQ_STATUS_ERRORTIND]            = TSB_DME_ERRORTIND,
    [IRQ_STATUS_PAINITERROR]          = TSB_DME_ERRORDIND,
    [IRQ_STATUS_DEBUGCOUNTEROVERFLOW] = TSB_DEBUGCOUNTEROVERFLOW,
    [IRQ_STATUS_LINKSTARTUPCNF]       = TSB_DME_LINKSTARTUPCNF,
    [IRQ_STATUS_MAILBOX]              = TSB_MAILBOX
};

static inline uint8_t *cport_to_rxbuf(struct sw_es3_priv *priv, unsigned int cportid) {
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

static int es3_transfer_check_write_status(uint8_t *status_block,
                                           size_t size)
{
    size_t i;
    struct __attribute__((__packed__)) write_status {
        uint8_t strw;
        uint8_t cport;
        uint16_t len;
        uint8_t status[4];
        uint8_t endp;
    };
    struct write_status *w_status;
    uint16_t tx_ent_remain;
    uint8_t tx_data_remain;
    int len_err, tx_ent_full, tx_data_full;

    /*
     * Find the status report within the block.
     */
    for (i = 0; i < size; i++) {
        switch (status_block[i]) {
        case STRW:
            w_status = (struct write_status*)&status_block[i];
            goto block_found;
        case HNUL: /* fall through */
        case LNUL:
            continue;
        default:
            dbg_error("%s: invalid byte 0x%x in status block\n",
                      __func__, status_block[i]);
            dbg_print_buf(ARADBG_ERROR, status_block, SWITCH_WRITE_STATUS_NNULL);
            return -EPROTO;
        }
    }
    dbg_error("%s: no STRW found in write status block:\n", __func__);
    dbg_print_buf(ARADBG_ERROR, status_block, size);
    return -EPROTO;

 block_found:
    /*
     * Sanity check the header and footer.
     */
    if (be16_to_cpu(w_status->len) != sizeof(w_status->status)) {
        dbg_error("%s: unexpected write status length %u (expected %u)\n",
                  __func__, be16_to_cpu(w_status->len),
                  (unsigned int)sizeof(w_status->status));
        return -EPROTO;
    } else if (w_status->endp != ENDP) {
        dbg_error("%s: unexpected write status byte 0x%02x in ENDP position (expected 0x02%x)\n",
                  __func__, w_status->endp, ENDP);
        return -EPROTO;
    }

    /*
     * Parse the status report itself.
     */
    tx_ent_remain =
        (((w_status->status[0] & WSTATUS0_TXENTFIFOREMAIN_MASK) << 8) |
         (w_status->status[1] & WSTATUS1_TXENTFIFOREMAIN_MASK));
    tx_data_remain = (w_status->status[2] & WSTATUS2_TXDATAFIFOREMAIN_MASK) *
                     8;
    len_err = !!(w_status->status[3] & WSTATUS3_LENERR_MASK);
    tx_ent_full = !!(w_status->status[3] & WSTATUS3_TXENTFIFOFULL_MASK);
    tx_data_full = !!(w_status->status[3] & WSTATUS3_TXDATAFIFOFULL_MASK);
    if (tx_ent_remain == 0 && !tx_ent_full) {
        dbg_error("%s: TXENTFIFOREMAIN=0, but TXENTFIFOFULL is not set.\n",
                  __func__);
        return -EPROTO;
    }
    if (tx_data_remain == 0 && !tx_data_full) {
        dbg_error("%s: TXDATAFIFOREMAIN=0, but TXDATAFIFOFULL is not set.\n",
                  __func__);
        return -EPROTO;
    }
    if (len_err) {
        dbg_error("%s: payload length error.\n", __func__);
        return -EIO;
    }
    if (tx_ent_full) {
        dbg_warn("%s: TX entry FIFO is full; write data was discarded.\n",
                 __func__);
        return -EAGAIN;
    }
    if (tx_data_full) {
        dbg_warn("%s: TX data FIFO is full; write data was discarded.\n",
                 __func__);
        return -EAGAIN;
    }
    return 0;
}

struct __attribute__ ((__packed__)) srpt_read_status_report {
    unsigned char raw[SRPT_REPORT_LEN];
    size_t rx_fifo_size; // number of bytes available in the rx buffer
    size_t tx_fifo_size; // bytes available for enqueue in the tx buffer
};

/*
 * Actual SPI select routine
 */
static inline void es3_spi_select(struct tsb_switch *sw, int select) {
    /*
     * SW-472: The STM32 SPI peripheral does not delay until the last
     * falling edge of SCK, instead dropping RXNE as soon as the rising
     * edge is clocked out.
     * Manually add a hacked delay in these cases...
     */
    if ((!select) && (SWITCH_SPI_FREQUENCY < 8000000))
        up_udelay(2);

    /* Set the GPIO low to select and high to de-select */
    stm32_gpiowrite(sw->pdata->spi_cs, !select);
}

static int es3_ncp_write(struct tsb_switch *sw,
                         uint8_t cportid,
                         uint8_t *tx_buf,
                         size_t tx_size,
                         size_t *out_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;

    uint8_t write_header[] = {
        STRW,
        cportid,
        (tx_size & 0xFF00) >> 8,
        (tx_size & 0xFF),
    };

    uint8_t write_trailer[] = {
        ENDP,
    };

    if (tx_size >= ES3_CPORT_NCP_MAX_PAYLOAD) {
        return -ENOMEM;
    }

    es3_spi_select(sw, true);

    SPI_SNDBLOCK(spi_dev, write_header, sizeof write_header);
    SPI_SNDBLOCK(spi_dev, tx_buf, tx_size);
    SPI_SNDBLOCK(spi_dev, write_trailer, sizeof write_trailer);
    *out_size = sizeof write_header + tx_size + sizeof write_trailer;

    dbg_insane("TX Data (%d):\n", *out_size);
    dbg_print_buf(ARADBG_INSANE, write_header, sizeof write_header);
    dbg_print_buf(ARADBG_INSANE, tx_buf, tx_size);
    dbg_print_buf(ARADBG_INSANE, write_trailer, sizeof write_trailer);

    es3_spi_select(sw, false);

    return OK;
}

static int es3_ncp_read(struct tsb_switch *sw,
                        uint8_t cportid,
                        uint8_t *rx_buf,
                        size_t rx_size,
                        size_t out_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cportid);
    size_t size;
    int rcv_done = 0;
    bool null_rxbuf;
    int ret = 0;

    uint8_t read_header[] = {
        STRR,
        cportid,
        0,      // LENM
        0,      // LENL
        ENDP,
    };

    es3_spi_select(sw, true);

    // Read CNF and retry if NACK received
    do {
        // Read the CNF
        size = SWITCH_NCP_READ_LEN + rx_size;
        SPI_SNDBLOCK(spi_dev, read_header, sizeof read_header);
        SPI_EXCHANGE(spi_dev, NULL, rxbuf, size);
        /*
         * Make sure we use 16-bit frames for the write and read commands
         * combined.
         */
        if ((out_size + size) & 0x1) {
            SPI_SEND(spi_dev, LNUL);
        }

        dbg_insane("TX Data (%d):\n", sizeof read_header);
        dbg_print_buf(ARADBG_INSANE, read_header, sizeof read_header);
        dbg_insane("TX Data (%d NULLs):\n", size);
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

    es3_spi_select(sw, false);

    return ret;
}

static int es3_ncp_transfer(struct tsb_switch *sw,
                            uint8_t *tx_buf,
                            size_t tx_size,
                            uint8_t *rx_buf,
                            size_t rx_size) {
    struct sw_es3_priv *priv = sw->priv;
    size_t out_size;
    int rc;

    pthread_mutex_lock(&priv->ncp_cport.lock);

    /* Send the request */
    rc = es3_ncp_write(sw, CPORT_NCP, tx_buf, tx_size, &out_size);
    if (rc) {
        dbg_error("%s() write failed: rc=%d\n", __func__, rc);
        goto done;
    }

    /* Read the CNF, back-to-back after the NCP request */
    rc = es3_ncp_read(sw, CPORT_NCP, rx_buf, rx_size, out_size);
    if (rc) {
        dbg_error("%s() read failed: rc=%d\n", __func__, rc);
        goto done;
    }

done:
    pthread_mutex_unlock(&priv->ncp_cport.lock);

    return rc;
}

/**
 * @brief retrieve the state of the cport spi fifo.
 */
static int es3_read_status(struct tsb_switch *sw,
                           unsigned int cport,
                           struct srpt_read_status_report *status) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    unsigned int offset;
    struct srpt_read_status_report *rpt = NULL;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cport);

    const char srpt_cmd[] = {SRPT, cport, 0, 0, ENDP};
    const char srpt_report_header[] = {SRPT, cport, 0x00, 0xC};

    es3_spi_select(sw, true);

    SPI_SNDBLOCK(spi_dev, srpt_cmd, sizeof srpt_cmd);
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, SRPT_SIZE);

    /* Make sure we use 16-bit frames */
    if ((sizeof srpt_cmd + SRPT_SIZE) & 0x1) {
        SPI_SEND(spi_dev, LNUL);
    }

    es3_spi_select(sw, false);

    /* Find the header */
    for (offset = 0; offset < (SRPT_SIZE - SRPT_REPORT_LEN); offset++) {
        if (!memcmp(srpt_report_header,
                    &rxbuf[offset],
                    sizeof srpt_report_header)) {
            /* Jump past the header */
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

        memcpy(status, rpt, SRPT_REPORT_LEN);

        switch (cport) {
        case CPORT_NCP:
            fifo_max_size = ES3_CPORT_NCP_FIFO_SIZE;
            break;
        case CPORT_DATA4:
        case CPORT_DATA5:
        default:
            fifo_max_size = ES3_CPORT_DATA_FIFO_SIZE;
            break;
        }

        status->tx_fifo_size = rpt->raw[7] * 8;
        status->rx_fifo_size = fifo_max_size -
                               ((rpt->raw[10] << 8) | rpt->raw[11]);
    }

    return 0;
}

static int es3_session_write(struct tsb_switch *sw,
                             uint8_t cportid,
                             uint8_t *tx_buf,
                             size_t tx_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    struct srpt_read_status_report rpt;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cportid);
    unsigned int size;
    int ret = OK;

    uint8_t write_header[] = {
        STRW,
        cportid,
        (tx_size & 0xFF00) >> 8,
        (tx_size & 0xFF),
    };

    uint8_t write_trailer[] = {
        ENDP,
    };

    /*
     * Must read the fifo status for data to ensure there is enough space.
     */
    ret = es3_read_status(sw, cportid, &rpt);
    if (ret) {
        return ret;
    }

    /* messages greater than one fifo's worth are not supported */
    if (tx_size >= ES3_CPORT_DATA_MAX_PAYLOAD) {
        return -EINVAL;
    }

    if (tx_size >= rpt.tx_fifo_size) {
        return -ENOMEM;
    }

    es3_spi_select(sw, true);
    /* Write */
    SPI_SNDBLOCK(spi_dev, write_header, sizeof write_header);
    SPI_SNDBLOCK(spi_dev, tx_buf, tx_size);
    SPI_SNDBLOCK(spi_dev, write_trailer, sizeof write_trailer);

    /* Wait write status, send NULL frames while waiting */
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, SWITCH_WRITE_STATUS_NNULL);

    dbg_insane("Write payload:\n");
    dbg_print_buf(ARADBG_INSANE, tx_buf, tx_size);
    dbg_insane("Write status:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, SWITCH_WRITE_STATUS_NNULL);

    /* Make sure we use 16-bit frames */
    size = sizeof write_header + tx_size + sizeof write_trailer
           + SWITCH_WRITE_STATUS_NNULL;
    if (size & 1) {
        SPI_SEND(spi_dev, LNUL);
    }

    /* Parse the write status and bail on error. */
    ret = es3_transfer_check_write_status(rxbuf,
                                          SWITCH_WRITE_STATUS_NNULL);
    if (ret) {
        goto out;
    }

out:
    es3_spi_select(sw, false);
    return ret;
}

static int es3_session_read(struct tsb_switch *sw,
                            uint8_t cportid,
                            uint8_t *rx_buf,
                            size_t rx_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    uint8_t *rxbuf = cport_to_rxbuf(priv, cportid);
    size_t size;
    int rcv_done = 0;
    bool null_rxbuf;
    int ret = 0;

    uint8_t read_header[] = {
        STRR,
        cportid,
        0,      // LENM
        0,      // LENL
        ENDP,
    };

    es3_spi_select(sw, true);

    // Read CNF and retry if NACK received
    do {
        // Read the CNF
        size = SWITCH_WAIT_REPLY_LEN + rx_size + SWITCH_DATA_READ_LEN;
        SPI_SNDBLOCK(spi_dev, read_header, sizeof read_header);
        SPI_EXCHANGE(spi_dev, NULL, rxbuf, size);
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

    es3_spi_select(sw, false);

    return ret;
}

static int es3_irq_fifo_rx(struct tsb_switch *sw, unsigned int cportid) {
    struct sw_es3_priv *priv = sw->priv;
    struct es3_cport *cport;
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

    rc = es3_read_status(sw, cportid, &rpt);
    if (rc) {
        rc = -EIO;
        goto fill_done;
    }
    len = rpt.rx_fifo_size;

    /*
     * Drain the fifo data.
     */
    rc = es3_session_read(sw, cportid, cport->rxbuf, len);
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

/*
 * Manual link startup request sequence for a port
 */
static int es3_linkstartup_request(struct tsb_switch *sw, uint8_t port)
{
    int rc;
    uint32_t val;

    dbg_info("%s(%d)\n", __func__, port);

    /*
     * Reset the port. The reset and clock have already been handled
     * during the port enable operation.
     */
    rc = switch_dme_set(sw, port, TSB_DME_RESETREQ, 0, 2);
    if (rc) {
        dbg_error("%s: TSB_DME_RESETREQ failed for port %d\n", __func__, port);
        return rc;
    }
    rc = switch_dme_get(sw, port, TSB_DME_RESETCNF, 0, &val);
    if (rc) {
        dbg_error("%s: TSB_DME_RESETCNF failed for port %d\n", __func__, port);
        return rc;
    }
    if (val != 1) {
        dbg_error("%s: Could not complete warm reset for port %d (%d)\n",
                  __func__, port, val);
        return ERROR;
    }

    /* Enable layers */
    rc = switch_dme_set(sw, port, TSB_DME_LAYERENABLEREQ, 0, 1);
    if (rc) {
        dbg_error("%s: TSB_DME_LAYERENABLEREQ failed for port %d\n",
                  __func__, port);
        return rc;
    }
    rc = switch_dme_get(sw, port, TSB_DME_LAYERENABLECNF, 0, &val);
    if (rc) {
        dbg_error("%s: TSB_DME_LAYERENABLECNF failed for port %d\n",
                  __func__, port);
        return rc;
    }
    if (val != 1) {
        dbg_error("%s: Could not enable layers for port %d (%d)\n",
                  __func__, port, val);
        return ERROR;
    }

    /* Request LinkStartup */
    if (switch_dme_set(sw, port, TSB_DME_LINKSTARTUPREQ, 0, 1)) {
        dbg_error("%s: LinkUp request failed for port %d\n", __func__, port);
    }

    /* Clear LinkStartupInd */
    rc = switch_dme_get(sw, port, TSB_DME_LINKSTARTUPIND, 0, &val);
    if (rc) {
        dbg_error("%s: TSB_DME_LINKSTARTUPIND read failed for port %d\n",
                  __func__, port);
        return rc;
    }

    /* Enable interrupts for Unipro port */
    rc = switch_port_irq_enable(sw, port, true);
    if (rc) {
        dbg_error("%s: Failed to enable port IRQs for port %d\n",
                  __func__, port);
        return rc;
    }

    /*
     * Wait for a LinkUpRequestCnf IRQ from the Switch. In case of
     * failure, let's retry
     */

    return 0;
}

/*
 * Enable port VDD and Clock, release reset lines
 */
static int es3_enable_port(struct tsb_switch *sw, uint8_t port, bool enable)
{
    uint32_t value = 0;

    if (enable) {
        /* Enable VDDVnPower */
        if (switch_sys_ctrl_set(sw, SC_VDDVNPOWERON, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnPowerOn register write failed\n");
            return -EIO;
        }

        /* Wait for VDDVnPSWAck */
        while (!(value & SC_VDDVN_PORT(port))) {
            if (switch_sys_ctrl_get(sw, SC_VDDVNPSWACK, &value)) {
                dbg_error("VDDVnPSWAck register read failed, aborting\n");
                return -EIO;
            }
        }

        /* Enable VDDVnCPowered */
        if (switch_sys_ctrl_set(sw, SC_VDDNCPOWEREDSET, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnCPoweredSet register write failed\n");
            return -EIO;
        }

        /*
         * Turn off isolation between VDDVn domain and VDDV domain
         * (VDDVnIsoClr)
         */
        if (switch_sys_ctrl_set(sw, SC_VDDVNISOCLR, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnIsoClr register write failed\n");
            return -EIO;
        }

        /* Enable Hibern8 Clock */
        if (switch_sys_ctrl_set(sw, SC_VDDVNHB8CLKEN, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnHB8ClkEnable register write failed\n");
            return -EIO;
        }

        /* Enable Clock */
        if (switch_sys_ctrl_set(sw, SC_SYSCLOCKENABLE,
                                SC_SYSCLOCKENABLE_PORT(port))) {
            dbg_error("SysClkEnable register write failed\n");
            return -EIO;
        }

        /* Release reset for M-Port */
        if (switch_sys_ctrl_set(sw, SC_SOFTRESETRELEASE,
                                SC_RESET_CLK_MPORT(port))) {
            dbg_error("SoftResetRelease register write failed\n");
            return -EIO;
        }

        /* Release reset for Unipro port */
        if (switch_sys_ctrl_set(sw, SC_SOFTRESETRELEASE,
                                SC_RESET_CLK_UNIPROPORT(port))) {
            dbg_error("SoftResetRelease register write failed\n");
            return -EIO;
        }
    } else {
        /* Hold reset for Unipro port */
        if (switch_sys_ctrl_set(sw, SC_SOFTRESET,
                                SC_RESET_CLK_UNIPROPORT(port))) {
            dbg_error("SoftReset register write failed\n");
            return -EIO;
        }

        /* Hold reset for M-Port */
        if (switch_sys_ctrl_set(sw, SC_SOFTRESET,
                                SC_RESET_CLK_MPORT(port))) {
            dbg_error("SoftReset register write failed\n");
            return -EIO;
        }

        /* Gate Clock */
        if (switch_sys_ctrl_set(sw, SC_SYSCLOCKGATE,
                                SC_SYSCLOCKENABLE_PORT(port))) {
            dbg_error("SysClkGate register write failed\n");
            return -EIO;
        }

        /* Gate Hibern8 Clock */
        if (switch_sys_ctrl_set(sw, SC_VDDVNHB8CLKGATE, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnHB8ClkGate register write failed\n");
            return -EIO;
        }

        /*
         * Turn on isolation between VDDVn domain and VDDV domain (VDDVnIsoClr)
         */
        if (switch_sys_ctrl_set(sw, SC_VDDVNISOSET, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnIsoSet register write failed\n");
            return -EIO;
        }

        /* Disable VDDVnCPowered */
        if (switch_sys_ctrl_set(sw, SC_VDDNCPOWEREDCLR, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnCPoweredClr register write failed\n");
            return -EIO;
        }

        /* Disable VDDVnPower */
        if (switch_sys_ctrl_set(sw, SC_VDDVNPOWEROFF, SC_VDDVN_PORT(port))) {
            dbg_error("VDDVnPowerOff register write failed\n");
            return -EIO;
        }
    }

    return 0;
}

/*
 * Switch communication init procedure.
 * Followed by common init code (Shared domain, Routing config etc.).
 */
int es3_init_seq(struct tsb_switch *sw)
{
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = priv->spi_dev;
    const char init_reply[] = { INIT, LNUL };
    uint8_t *rxbuf = cport_to_rxbuf(priv, CPORT_NCP);
    int i, rc = -1;


    es3_spi_select(sw, true);

    // Delay needed before the switch is ready on the SPI bus
    up_udelay(SWITCH_SPI_INIT_DELAY);

    SPI_SEND(spi_dev, INIT);
    SPI_SEND(spi_dev, INIT);
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, SWITCH_WAIT_REPLY_LEN);

    dbg_insane("Init RX Data:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, SWITCH_WAIT_REPLY_LEN);

    // Check for the transition from INIT to LNUL after sending INITs
    for (i = 0; i < SWITCH_WAIT_REPLY_LEN - 1; i++) {
        if (!memcmp(rxbuf + i, init_reply, sizeof(init_reply)))
            rc = 0;
    }

    es3_spi_select(sw, false);

    if (rc) {
        dbg_error("%s: Failed to init the SPI link with the switch\n",
                  __func__);
        return rc;
    }

    /* Hold reset for all Unipro and M ports */
    if (switch_sys_ctrl_set(sw, SC_SOFTRESET, SC_SYSCLOCK_ALL_PORTS)) {
        dbg_error("SoftReset register write failed\n");
        return -EIO;
    }

    /* Enable Shared domain, PMU; RT10b = 0 */
    if (switch_sys_ctrl_set(sw, SC_SYSCLOCKENABLE, SC_SYSCLOCKENABLE_SH_PMU)) {
        dbg_error("SysClkEnable register write failed\n");
        return -EIO;
    }

    /* Release reset for Shared domain */
    if (switch_sys_ctrl_set(sw, SC_SOFTRESETRELEASE, SC_RESET_CLK_SHARED)) {
        dbg_error("SoftResetRelease register write failed\n");
        return -EIO;
    }

    /*
     * Configure the Routing Table as 5 bit addressable
     * (RT10b = 0, RTBitSel = 1).
     */
    if (switch_internal_setattr(sw, RTBITSEL, 0x1)) {
        dbg_error("RTBITSEL register write failed\n");
        return -EIO;
    }

    /* Setup manual LinkStartupMode for all ports */
    if (switch_sys_ctrl_set(sw, SC_LINKSTARTUPMODE, SC_LINK_MANUAL_ALL_PORTS)) {
        dbg_error("LinkStartupMode register write failed\n");
        return -EIO;
    }

    return rc;
}

/* ES3 specific interrupt handler. Clears the source of interrupt */
int es3_switch_irq_handler(struct tsb_switch *sw)
{
    uint32_t swint, swins, port_irq_status, attr_value;
    int i, j, rc;

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return -EINVAL;
    }

    do {
        /* Read Switch Interrupt Status register */
        if (switch_internal_getattr(sw, SWINT, &swint)) {
            dbg_error("IRQ: SWINT register read failed\n");
            return -EIO;
        }
        dbg_insane("IRQ: SWINT=0x%x\n", swint);

        /* Handle the Switch internal interrupts */
        if (swint & TSB_INTERRUPT_SWINTERNAL) {
            if (switch_internal_getattr(sw, SWINS, &swins)) {
                dbg_error("IRQ: SWINS register read failed\n");
            }
            dbg_insane("IRQ: Switch internal irq, SWINS=0x%04x\n", swins);

            if (swins & TSB_INTERRUPT_SPICES) {
                if (switch_internal_getattr(sw, SPICES, &attr_value)) {
                    dbg_error("IRQ: SPICES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPICES=0x%04x\n",
                           attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI3ES) {
                if (switch_internal_getattr(sw, SPI3ES, &attr_value)) {
                    dbg_error("IRQ: SPI3ES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPI3ES=0x%04x\n",
                           attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI4ES) {
                if (switch_internal_getattr(sw, SPI4ES, &attr_value)) {
                    dbg_error("IRQ: SPI4ES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPI4ES=0x%04x\n",
                           attr_value);
            }
            if (swins & TSB_INTERRUPT_SPI5ES) {
                if (switch_internal_getattr(sw, SPI5ES, &attr_value)) {
                    dbg_error("IRQ: SPI5ES register read failed\n");
                }
                dbg_insane("IRQ: Switch internal irq, SPI5ES=0x%04x\n",
                           attr_value);
            }
        }

        /* Handle external interrupts: CPorts 4 & 5 */
        if (swint & TSB_INTERRUPT_SPIPORT4_RX) {
            dbg_insane("IRQ: Switch SPI port 4 RX irq\n");
            es3_irq_fifo_rx(sw, 4);
        }
        if (swint & TSB_INTERRUPT_SPIPORT5_RX) {
            dbg_insane("IRQ: Switch SPI port 5 RX irq\n");
            es3_irq_fifo_rx(sw, 5);
        }

        /* Handle LinkStartupCnf interrupts */
        if (swint & TSB_INTERRUPT_LSCINTERNAL) {
            if (switch_internal_getattr(sw, SWLCS, &attr_value)) {
                dbg_error("IRQ: SWLCS register read failed\n");
            }
            dbg_insane("IRQ: LinkStartCnf irq, SWLCS=0x%08x\n",
                       attr_value);

            /*
             * If LinkUp failed retry it manually. Only useful in the case
             * of automatic LinkUp.
             */
            for (i = 0; i < SWITCH_PORT_MAX; i++) {
                if (attr_value & (2 << (i * 2))) {
                    struct tsb_switch_event e;
                    e.type = TSB_SWITCH_EVENT_LINKUP;
                    e.linkup.port = i;
                    e.linkup.val = TSB_LINKUP_FAIL;
                    rc = tsb_switch_event_notify(sw, &e);
                    if (rc) {
                        dbg_error("IRQ: LinkUp event notification failed for port %d: %d\n",
                                  i, rc);
                    }
                }
            }
        }

        /* Handle Unipro interrupts: read the Unipro ports interrupt status */
        for (i = 0; i < SWITCH_PORT_MAX; i++) {
            /*
             * If Unipro interrupt pending, read the interrupt status
             * attribute
             */
            if (swint & (1 << i)) {
                if (switch_dme_get(sw, i, TSB_INTERRUPTSTATUS, 0x0,
                                   &port_irq_status)) {
                    dbg_error("IRQ: TSB_INTERRUPTSTATUS(%d) register read failed\n",
                              i);
                    break;
                }
                dbg_insane("IRQ: TSB_INTERRUPTSTATUS(%d)=0x%04x\n",
                           i, port_irq_status);

                /* Read the attributes associated to the interrupt sources */
                for (j = 0; j < ES3_IRQ_MAX; j++) {
                    if ((port_irq_status & (1 << j)) && unipro_irq_attr[j]) {
                        if (switch_dme_get(sw, i, unipro_irq_attr[j], 0x0,
                                           &attr_value)) {
                            dbg_error("IRQ: Port %d line %d attr(0x%04x) read failed\n",
                                      i, j, unipro_irq_attr[j]);
                        } else {
                            dbg_insane("IRQ: Port %d line %d asserted, attr(0x%04x)=0x%04x\n",
                                       i, j, unipro_irq_attr[j], attr_value);
                        }

                        uint32_t irq_type = j;
                        uint32_t port = i;
                        switch (irq_type) {
                        case IRQ_STATUS_LINKSTARTUPCNF: {
                            struct tsb_switch_event e;
                            e.type = TSB_SWITCH_EVENT_LINKUP;
                            e.linkup.port = port;
                            e.linkup.val = attr_value;
                            rc = tsb_switch_event_notify(sw, &e);
                            if (rc) {
                                dbg_error("IRQ: LinkUp event notification failed for port %d: %d\n",
                                          port, rc);
                            }
                            break;
                        }
                        case IRQ_STATUS_MAILBOX: {
                            struct tsb_switch_event e;
                            e.type = TSB_SWITCH_EVENT_MAILBOX;
                            e.mbox.port = port;
                            e.mbox.val = attr_value;
                            rc = tsb_switch_event_notify(sw, &e);
                            if (rc) {
                                dbg_error("IRQ: Mailbox event notification failed for port %d: %d\n",
                                          port, rc);
                            }
                            break;
                        }
                        default:
                            break;
                        }
                    }
                }
            }
        }

    } while (swint);

    return 0;
}

/* Low level switch IRQ handler
 *
 * Posts a message in a list in order to defer the work to perform
 */
static int switch_irq_handler(int irq, void *context, void *priv)
{
    struct tsb_switch *sw = priv;

    if (!sw) {
        dbg_error("%s: no Switch context\n", __func__);
        return -EINVAL;
    }

    switch_post_irq(sw);

    return 0;
}

/* Switch interrupt enable/disable */
static int es3_switch_irq_enable(struct tsb_switch *sw, bool enable)
{
    if (enable) {
        /* Enable switch interrupt sources and install handler */
        if (!sw->pdata->gpio_irq) {
            dbg_error("%s: no Switch context\n", __func__);
            return -EINVAL;
        }

        /*
         * Configure switch IRQ line: parameters from pdata, install
         * handler, and pass the tsb_switch struct to the handler.
         */
        stm32_gpiosetevent_priv(sw->pdata->gpio_irq,
                                sw->pdata->irq_rising_edge,
                                !sw->pdata->irq_rising_edge,
                                true, switch_irq_handler, sw);

        /* Enable the switch internal interrupt sources */
        if (switch_internal_setattr(sw, SWINE, SWINE_ENABLE_ALL)) {
            dbg_error("Switch SWINE register write failed\n");
            return -EIO;
        }

        /* Enable the L4 interrupts */
        if (switch_dme_set(sw, SWITCH_PORT_ID, TSB_INTERRUPTENABLE, 0x0,
                       TSB_L4_INTERRUPTENABLE_ALL)) {
            dbg_error("Switch INTERRUPTENABLE register write failed\n");
            return -EIO;
        }

        /* Enable the SPI interrupts */
        if (switch_internal_setattr(sw, SPIINTE, SPIINTE_ENABLE_ALL)) {
            dbg_error("Switch SPIINTE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPICEE, SPICEE_ENABLE_ALL)) {
            dbg_error("Switch SPICEE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI3EE, SPI3EE_ENABLE_ALL)) {
            dbg_error("Switch SPI3EE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI4EE, SPI45EE_ENABLE_ALL)) {
            dbg_error("Switch SPI4EE register write failed\n");
            return -EIO;
        }
        if (switch_internal_setattr(sw, SPI5EE, SPI45EE_ENABLE_ALL)) {
            dbg_error("Switch SPI5EE register write failed\n");
            return -EIO;
        }
    } else {
        /* Disable switch interrupt */
        stm32_gpiosetevent_priv(sw->pdata->gpio_irq, false, false, false,
                                NULL, NULL);
    }

    return OK;
}

/* Enable/disable the interrupts for the port */
static int es3_port_irq_enable(struct tsb_switch *sw, uint8_t port_id,
                               bool enable)
{
    if (switch_dme_set(sw, port_id, TSB_INTERRUPTENABLE, 0x0,
                       enable ? TSB_INTERRUPTENABLE_ALL : 0)) {
        dbg_error("Port %d INTERRUPTENABLE register write failed\n", port_id);
        return -EIO;
    }

    return OK;
}

/* NCP commands */
static int es3_set(struct tsb_switch *sw,
                   uint8_t port_id,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
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

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d, val=0x%04x\n",
                __func__, port_id, attrid, select_index, val);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): portId=%u, attrId=0x%04x failed: rc=%d\n",
                  __func__, port_id, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

static int es3_get(struct tsb_switch *sw,
                   uint8_t port_id,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t *val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        port_id,
        NCP_GETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
                __func__, port_id, attrid, select_index);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_GETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, *val);

    return cnf.rc;
}


static int es3_peer_set(struct tsb_switch *sw,
                        uint8_t port_id,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
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

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d, val=0x%04x\n",
                __func__, port_id, attrid, select_index, val);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *)  &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): portId=%u, attrId=0x%04x failed: rc=%d\n",
                  __func__, port_id, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_PEERSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

static int es3_peer_get(struct tsb_switch *sw,
                        uint8_t port_id,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t *val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        port_id,
        NCP_PEERGETREQ,
        (attrid >> 8),
        (attrid & 0xff),
        (select_index >> 8),
        (select_index & 0xff),
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t port_id;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
                 __func__, port_id, attrid, select_index);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_PEERGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, *val);

    return cnf.rc;
}

/* Only the 5 bit LUT is supported (RT10b = 0, RTBitSel = 1) */
static int es3_lut_set(struct tsb_switch *sw,
                       uint8_t unipro_portid,
                       uint8_t lut_address,
                       uint8_t dest_portid)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_LUTSETREQ,
        lut_address,
        NCP_RESERVED,
        NCP_RESERVED,
        dest_portid
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;

    dbg_verbose("%s(): unipro_portid=%d, lutAddress=%d, destPortId=%d\n",
                __func__, unipro_portid, lut_address, dest_portid);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): unipro_portid=%d, destPortId=%d failed: rc=%d\n",
                  __func__, unipro_portid, dest_portid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_LUTSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n",
                __func__, cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}

static int es3_lut_get(struct tsb_switch *sw,
                       uint8_t unipro_portid,
                       uint8_t lut_address,
                       uint8_t *dest_portid)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_LUTGETREQ,
        lut_address,
        NCP_RESERVED,
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t dest_portid;
    } cnf;

    dbg_verbose("%s(): unipro_portid=%d, lutAddress=%d, destPortId=%d\n",
                __func__, unipro_portid, lut_address, *dest_portid);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): unipro_portid=%d, destPortId=%d failed: rc=%d\n",
                  __func__, unipro_portid, *dest_portid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_LUTGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *dest_portid = cnf.dest_portid;

    dbg_verbose("%s(): fid=0x%02x, rc=%u, portID=%u\n", __func__,
                cnf.function_id, cnf.rc, cnf.dest_portid);

    /* Return resultCode */
    return cnf.rc;
}

/*
 * Routing table: dest_valid table helpers.
 * Only 5 bit routing is supported, with max 32 device IDs per switch
 * and 32 CPorts per device.
 *
 * The routing is handled globally per destination device,
 * i.e. all 32 CPorts for the destination device are handled at once.
 */
static inline int set_valid_entry(uint8_t *table, int entry, bool valid)
{
    int offset;

    if ((entry < 0) || (entry > ES3_DEVICEID_MAX)) {
        return -EINVAL;
    }

    offset = 127 - (entry * 4);

    table[offset--] = valid ? 0xFF : 0x00;
    table[offset--] = valid ? 0xFF : 0x00;
    table[offset--] = valid ? 0xFF : 0x00;
    table[offset] = valid ? 0xFF : 0x00;

    return 0;
}

static inline bool check_valid_entry(uint8_t *table, int entry)
{
    int offset;

    if ((entry < 0) || (entry > ES3_DEVICEID_MAX)) {
        return false;
    }

    offset = 127 - (entry * 4);

    return ((table[offset--] == 0xFF) &&
            (table[offset--] == 0xFF) &&
            (table[offset--] == 0xFF) &&
            (table[offset] == 0xFF));
}

/**
 * @brief Update the valid device bitmask for device_id on port_id
 */
static int es3_set_valid_device(struct tsb_switch *sw,
                                uint8_t port_id,
                                uint8_t device_id,
                                bool valid)
{
    uint8_t dest_valid[128];
    int rc;

    rc = switch_dev_id_mask_get(sw, port_id, dest_valid);
    if (rc) {
        dbg_error("Failed to get valid device bitmask for port %u\n", port_id);
        return rc;
    }

    rc = set_valid_entry(dest_valid, device_id, valid);
    if (rc) {
        dbg_error("Failed to update valid device bitmask for port %u\n",
                  port_id);
        return rc;
    }

    rc = switch_dev_id_mask_set(sw, port_id, dest_valid);
    if (rc) {
        dbg_error("Failed to set valid device bitmask for port %u\n", port_id);
        return rc;
    }

    return rc;
}

/**
 * @brief Dump routing table to low level console
 */
static int es3_dump_routing_table(struct tsb_switch *sw) {
    int i, j, devid, unipro_portid;
    uint8_t p = 0, dest_valid[128];

    dbg_info("======================================================\n");
    dbg_info("Routing table:\n");
    dbg_info(" [Port,DevId] -> [Port]\n");

    for (unipro_portid = 0; unipro_portid <= SWITCH_PORT_ID; unipro_portid++) {
        if (switch_dev_id_mask_get(sw, unipro_portid, dest_valid)) {
            dbg_error("%s() Failed to retrieve routing table.\n", __func__);
            return -1;
        }
        dbg_insane("%s(): DestValid %d\n", __func__, unipro_portid);
        dbg_print_buf(ARADBG_INSANE, dest_valid, sizeof(dest_valid));

        for (i = 0; i < 8; i++) {
            for (j = 0; j < 16; j++) {
                devid = i * 16 + j;
                if (check_valid_entry(dest_valid, devid)) {
                    switch_lut_get(sw, unipro_portid, devid, &p);
                    dbg_info(" [%2u,%2u] -> %2u\n", unipro_portid, devid, p);
               }
            }
        }
    }

    dbg_info("======================================================\n");

    return 0;
}

static int es3_sys_ctrl_set(struct tsb_switch *sw,
                            uint16_t sc_addr,
                            uint32_t val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_SYSCTRLSETREQ,
        sc_addr >> 8,
        sc_addr & 0xff,
        val >> 24,
        (val >> 16) & 0xff,
        (val >> 8) & 0xff,
        val & 0xff,
    };

    struct __attribute__((packed)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;

    uint8_t *cnf_buf = (uint8_t *) &cnf;

    switch (sc_addr) {
    /* Special case for PMU_StandbySqStart register: ignore reply */
    case SC_PMUSTANDBYSS:
        cnf_buf = NULL;
        break;
    default:
        break;
    }

    dbg_verbose("%s(): sc_addr=0x%x, val=0x%x (%d)\n",
                __func__, sc_addr, val, val);
    rc = es3_ncp_transfer(sw, req, sizeof(req), cnf_buf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): sc_addr=0x%x, val=0x%x (%d) failed: %d\n",
                  __func__, sc_addr, val, val, rc);
        return rc;
    }
    if (cnf.function_id != NCP_SYSCTRLSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__, cnf.function_id, cnf.rc);
    return cnf.rc;
}

static int es3_sys_ctrl_get(struct tsb_switch *sw,
                            uint16_t sc_addr,
                            uint32_t *val)
{
    int rc;
    uint8_t req[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_SYSCTRLGETREQ,
        sc_addr >> 8,
        sc_addr & 0xff,
    };
    struct __attribute__((packed)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint32_t val;
    } cnf;

    dbg_verbose("%s(): sc_addr=0x%x\n", __func__, sc_addr);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *)&cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): sc_addr=0x%x failed: %d\n", __func__, sc_addr, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SYSCTRLGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    if (cnf.rc == 0) {
        *val = be32_to_cpu(cnf.val);
    }
    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__, cnf.function_id, cnf.rc);

    return cnf.rc;
}

static int es3_dest_valid_set(struct tsb_switch *sw,
                              uint8_t unipro_portid,
                              uint8_t *mask)
{
    int rc;

    struct __attribute__ ((__packed__)) req {
        uint8_t reserved;
        uint8_t portid;
        uint8_t function_id;
        uint8_t mask[128];
     } req = {
        NCP_RESERVED,
        unipro_portid,
        NCP_DESTVALIDSETREQ,
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
    } cnf;

    dbg_verbose("%s()\n", __func__);

    memcpy(req.mask, mask, sizeof(req.mask));

    rc = es3_ncp_transfer(sw, (uint8_t *) &req, sizeof(req),
                         (uint8_t *) &cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s()failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_DESTVALIDSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__,
                cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}

static int es3_dest_valid_get(struct tsb_switch *sw,
                              uint8_t unipro_portid,
                              uint8_t *dst)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        unipro_portid,
        NCP_DESTVALIDGETREQ,
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
        uint8_t mask[128];
    } cnf;

    dbg_verbose("%s(%d)\n", __func__, unipro_portid);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s()failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_DESTVALIDGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    memcpy(dst, &cnf.mask, sizeof(cnf.mask));

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__,
                cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}

static int es3_switch_attr_set(struct tsb_switch *sw,
                               uint16_t attrid,
                               uint32_t val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_SWITCHATTRSETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
        ((val >> 24) & 0xff),
        ((val >> 16) & 0xff),
        ((val >> 8) & 0xff),
        (val & 0xff)
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;

    uint8_t *cnf_buf = (uint8_t *) &cnf;

    switch (attrid) {
    /* Special case for SWRES attribute: ignore reply */
    case SWRES:
        cnf_buf = NULL;
        break;
    default:
        break;
    }

    dbg_verbose("%s(): attrId=0x%04x\n", __func__, attrid);

    rc = es3_ncp_transfer(sw, req, sizeof(req), cnf_buf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SWITCHATTRSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

static int es3_switch_attr_get(struct tsb_switch *sw,
                               uint16_t attrid,
                               uint32_t *val)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_SWITCHATTRGETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): attrId=0x%04x\n", __func__, attrid);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t *) &cnf,
                          sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SWITCHATTRGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, *val);

    return cnf.rc;
}

static int es3_switch_id_set(struct tsb_switch *sw,
                             uint8_t cportid,
                             uint8_t peer_cportid,
                             uint8_t dis,
                             uint8_t irt)
{
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        (dis << 2) | (irt << 0),
        NCP_SWITCHIDSETREQ,
        SWITCH_DEVICE_ID,
        cportid,            // L4 CPortID
        SWITCH_DEVICE_ID,
        peer_cportid,
        NCP_RESERVED,
        SWITCH_PORT_ID      // Source portID
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;

    dbg_verbose("%s: cportid: %u peer_cportid: %u dis: %u irt: %u\n",
                __func__,
                cportid,
                peer_cportid,
                dis,
                irt);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t*)&cnf, sizeof(cnf));
    if (rc) {
        dbg_error("%s() failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SWITCHIDSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): ret=0x%02x, switchDeviceId=0x%01x, cPortId=0x%01x -> peerCPortId=0x%01x\n",
                __func__,
                cnf.rc,
                SWITCH_DEVICE_ID,
                cportid,
                peer_cportid);

    return cnf.rc;
}

/**
 * @brief Send raw data down CPort 4. We're only using this for the SVC
 * connection, so fix the CPort number.
 */
static int es3_data_send(struct tsb_switch *sw, void *data, size_t len) {
    struct sw_es3_priv *priv = sw->priv;
    int rc;

    pthread_mutex_lock(&priv->data_cport4.lock);

    rc = es3_session_write(sw, CPORT_DATA4, data, len);
    if (rc) {
        dbg_error("%s() write failed: rc=%d\n", __func__, rc);
        goto done;
    }

done:
    pthread_mutex_unlock(&priv->data_cport4.lock);

    return rc;
}

/**
 * @brief Start the transmission of FCTs on CPort 4.
 *        Other CPorts not tested/unsupported.
 */
int es3_fct_enable(struct tsb_switch *sw) {
    uint32_t spictlb = 0xC;
    int rc;

    rc = switch_internal_setattr(sw, SPICTLB, spictlb);

    if (rc) {
        dbg_error("Failed to set spictlb\n");
    }

    return rc;
}

static int es3_qos_attr_set(struct tsb_switch *sw,
                            uint8_t  portid,
                            uint8_t  attrid,
                            uint32_t attr_val) {
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        portid,
        NCP_QOSATTRSETREQ,
        NCP_RESERVED,
        attrid,
        (attr_val & 0xff000000) >> 24,
        (attr_val & 0xff0000) >> 16,
        (attr_val & 0xff00) >> 8,
        (attr_val & 0xff) >> 0
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
    } cnf;

    dbg_verbose("%s: portid: %u attrid: %u attr_val: %u\n",
                __func__,
                portid,
                attrid,
                attr_val);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t*)&cnf, sizeof(cnf));

    if (rc) {
        dbg_error("%s() failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.portid != portid) {
        dbg_error("%s(): unexpected portid 0x%x\n", __func__, cnf.portid);
        return -EPROTO;
    }

    if (cnf.function_id != NCP_QOSATTRSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): ret=0x%02x, portid=0x%02x, attr(0x%04x)=0x%04x\n",
                __func__,
                cnf.rc,
                portid,
                attrid,
                attr_val);

    return cnf.rc;
}

static int es3_qos_attr_get(struct tsb_switch *sw,
                            uint8_t portid,
                            uint8_t attrid,
                            uint32_t *val) {
    int rc;

    uint8_t req[] = {
        NCP_RESERVED,
        portid,
        NCP_QOSATTRGETREQ,
        NCP_RESERVED,
        attrid
    };

    struct __attribute__ ((__packed__)) cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s: portid: %u attrid: %u\n", __func__, portid, attrid);

    rc = es3_ncp_transfer(sw, req, sizeof(req), (uint8_t*)&cnf, sizeof(cnf));

    if (rc) {
        dbg_error("%s() failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.portid != portid) {
        dbg_error("%s(): unexpected portid 0x%x\n", __func__, cnf.portid);
        return -EPROTO;
    }

    if (cnf.function_id != NCP_QOSATTRGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *val = be32_to_cpu(cnf.attr_val);
    dbg_verbose("%s(): ret=0x%02x, portid=0x%02x, attr(0x%04x)=0x%04x\n",
                    __func__,
                    cnf.rc,
                    portid,
                    attrid,
                    *val);

    return cnf.rc;
}

static struct tsb_switch_ops es3_ops = {
    .init_comm             = es3_init_seq,
    .enable_port           = es3_enable_port,
    .link_startup          = es3_linkstartup_request,

    .set                   = es3_set,
    .get                   = es3_get,

    .peer_set              = es3_peer_set,
    .peer_get              = es3_peer_get,

    .lut_set               = es3_lut_set,
    .lut_get               = es3_lut_get,
    .set_valid_device      = es3_set_valid_device,
    .dump_routing_table    = es3_dump_routing_table,

    .sys_ctrl_set          = es3_sys_ctrl_set,
    .sys_ctrl_get          = es3_sys_ctrl_get,

    .dev_id_mask_get       = es3_dest_valid_get,
    .dev_id_mask_set       = es3_dest_valid_set,

    .port_irq_enable       = es3_port_irq_enable,

    .switch_attr_get       = es3_switch_attr_get,
    .switch_attr_set       = es3_switch_attr_set,
    .switch_id_set         = es3_switch_id_set,

    .qos_attr_set          = es3_qos_attr_set,
    .qos_attr_get          = es3_qos_attr_get,

    .fct_enable            = es3_fct_enable,

    .switch_irq_enable     = es3_switch_irq_enable,
    .switch_irq_handler    = es3_switch_irq_handler,

    .switch_data_send      = es3_data_send,
};

int tsb_switch_es3_init(struct tsb_switch *sw, unsigned int spi_bus)
{
    struct spi_dev_s *spi_dev;
    struct sw_es3_priv *priv;
    int rc = 0;

    dbg_info("Initializing ES3 switch...\n");

    stm32_configgpio(sw->pdata->spi_cs);
    stm32_gpiowrite(sw->pdata->spi_cs, true);

    spi_dev = up_spiinitialize(spi_bus);
    if (!spi_dev) {
        dbg_error("%s: Failed to initialize spi device\n", __func__);
        return -ENODEV;
    }

    priv = malloc(sizeof(struct sw_es3_priv));
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
    sw->ops = &es3_ops;

    /* Configure the SPI1 bus in Mode0, 8bits, 13MHz clock */
    SPI_SETMODE(spi_dev, SPIDEV_MODE0);
    SPI_SETBITS(spi_dev, 8);
    SPI_SETFREQUENCY(spi_dev, SWITCH_SPI_FREQUENCY);

    dbg_info("... Done!\n");

    return rc;

error:
    tsb_switch_es3_exit(sw);
    return rc;
}

void tsb_switch_es3_exit(struct tsb_switch *sw) {
    struct sw_es3_priv *priv;

    if (!sw)
        return;

    priv = sw->priv;
    free(priv);
    sw->priv = NULL;
    sw->ops = NULL;
}
