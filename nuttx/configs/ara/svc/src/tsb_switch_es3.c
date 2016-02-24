/*
 * Copyright (c) 2016 Google Inc.
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

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <ara_debug.h>
#include "tsb_switch.h"

/* Max NULL frames to wait for a reply from the switch */
#define ES3_SWITCH_WAIT_REPLY_LEN      (9)
/* NCP read data command: 4 bytes header + END */
#define ES3_SWITCH_NCP_READ_DATA_LEN   (5)
/* Write status reply length */
#define ES3_SWITCH_WRITE_STATUS_LEN    (9)
/* Returned session data header/trailer: 4 bytes header + END */
#define SWITCH_DATA_READ_LEN           (5)
/* Total number of NULLs to clock out to ensure a write status is read */
#define ES3_SWITCH_WRITE_STATUS_NNULL  (ES3_SWITCH_WAIT_REPLY_LEN + \
                                        ES3_SWITCH_WRITE_STATUS_LEN)
/* Total number of NULLs to clock out to ensure the returned NCP is read */
#define ES3_SWITCH_NCP_READ_LEN        (ES3_SWITCH_WAIT_REPLY_LEN + \
                                        ES3_SWITCH_NCP_READ_DATA_LEN)

/* Interrupt enable bits */
#define ES3_SPICEE_ENABLE_ALL        (0xF)
#define ES3_SPI3EE_ENABLE_ALL        (0xEF)
#define ES3_SPI45EE_ENABLE_ALL       (0xFE0000FF)
/* Dest valid set is the longest NCP request. */
#define ES3_NCP_MAX_REQ_SIZE        (131)
/* Each ingress port has a validation block with a one-hot bit per
 * destination address */
#define ES3_DEST_VALID_MASK_SIZE    (128)
/* 9-byte max delay + 4-byte header + 272-byte max payload + 1-byte footer */
#define ES3_CPORT_RX_MAX_SIZE       (9 + 4 + 272 + 1)

#define RTBITSEL                    (0x00BC)

/* System registers, accessible via switch_sys_ctrl_set, switch_sys_ctrl_get */
#define SC_SYSCLOCKGATE             (0x0200)
#define SC_SYSCLOCKENABLE           (0x0300)
#define SC_SYSCLOCKDIV0             (0x0410)
#define SC_SYSCLOCKDIV1             (0x0414)
#define SC_SYSCLOCKDIV2             (0x0418)
#define SC_LINKSTARTUPMODE          (0x0B00)
#define SC_PMUSTANDBYSS             (0x0C00)
#define SC_VDDVSAVE                 (0x0C10)
#define SC_VDDVRESTORE              (0x0C14)
#define SC_VDDVNPOWEROFF            (0x0C20)
#define SC_VDDVNPOWERON             (0x0C24)
#define SC_VDDNCPOWEREDSET          (0x0C30)
#define SC_VDDNCPOWEREDCLR          (0x0C34)
#define SC_VDDNSAVE                 (0x0C40)
#define SC_VDDNRESTORE              (0x0C44)
#define SC_VDDVNISOSET              (0x0C50)
#define SC_VDDVNISOCLR              (0x0C54)
#define SC_VDDVNHB8CLKEN            (0x0C60)
#define SC_VDDVNHB8CLKGATE          (0x0C64)
#define SC_VDDVNPOWERSTAT           (0x0D10)
#define SC_VDDVNCPOWEREDST          (0x0D20)
#define SC_VDDVNISOSTAT             (0x0D40)
#define SC_VDDVNPSWACK              (0x0E00)
#define SC_VDDVNSYSCLKOFFN          (0x0E10)

/* System registers values */
#define SC_RESET_CLK_SHARED         (1 << 15)
/*  Enable Shared domain, PMU; RT10b = 0 */
#define SC_SYSCLOCKENABLE_SH_PMU    (0x80008000)
/*  Enable Unipro and M-port clock */
#define SC_SYSCLOCKENABLE_PORT(i)   ((1 << (i + 16)) | (1 << i))
/*  Mask for all Unipro and M-ports reset and clock control */
#define SC_SYSCLOCK_ALL_PORTS       (0x3FFF3FFF)
/*  Enable VDDn for M-port */
#define SC_VDDVN_PORT(i)            (1 << i)
/*  Manul LinkStartupMode for all ports */
#define SC_LINK_MANUAL_ALL_PORTS    (0x3FFF)

#define ES3_DEVICEID_MAX            31

struct es3_cport {
    pthread_mutex_t lock;
    uint8_t rxbuf[ES3_CPORT_RX_MAX_SIZE];
};

struct sw_es3_priv {
    struct es3_cport ncp_cport;
    struct es3_cport data_cport4;
    struct es3_cport data_cport5;
};

static inline uint8_t *fifo_to_rxbuf(struct sw_es3_priv *priv, unsigned int fifo) {
    switch (fifo) {
    case SWITCH_FIFO_NCP:
        return priv->ncp_cport.rxbuf;
    case SWITCH_FIFO_DATA4:
        return priv->data_cport4.rxbuf;
    case SWITCH_FIFO_DATA5:
        return priv->data_cport5.rxbuf;
    };
    return NULL;
}

/*
 * Routing table: dest_valid table helpers.
 * Only 5 bit routing is supported, with max 32 device IDs per switch
 * and 32 CPorts per device.
 *
 * The routing is handled globally per destination device,
 * i.e. all 32 CPorts for the destination device are handled at once.
 */
static int es3_set_valid_entry(struct tsb_switch *sw,
                               uint8_t *table, int entry, bool valid) {
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

static bool es3_check_valid_entry(struct tsb_switch *sw,
                                  uint8_t *table, int entry) {
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


static int es3_ncp_write(struct tsb_switch *sw,
                         uint8_t cportid,
                         uint8_t *tx_buf,
                         size_t tx_size,
                         size_t *out_size) {
    struct spi_dev_s *spi_dev = sw->spi_dev;

    uint8_t write_header[] = {
        STRW,
        cportid,
        (tx_size & 0xFF00) >> 8,
        (tx_size & 0xFF),
    };

    uint8_t write_trailer[] = {
        ENDP,
    };

    if (tx_size >= SWITCH_CPORT_NCP_MAX_PAYLOAD) {
        return -ENOMEM;
    }

    _switch_spi_select(sw, true);

    SPI_SNDBLOCK(spi_dev, write_header, sizeof write_header);
    SPI_SNDBLOCK(spi_dev, tx_buf, tx_size);
    SPI_SNDBLOCK(spi_dev, write_trailer, sizeof write_trailer);
    *out_size = sizeof write_header + tx_size + sizeof write_trailer;

    dbg_insane("TX Data (%d):\n", *out_size);
    dbg_print_buf(ARADBG_INSANE, write_header, sizeof write_header);
    dbg_print_buf(ARADBG_INSANE, tx_buf, tx_size);
    dbg_print_buf(ARADBG_INSANE, write_trailer, sizeof write_trailer);

    _switch_spi_select(sw, false);

    return OK;
}

static int es3_ncp_read(struct tsb_switch *sw,
                        uint8_t cportid,
                        uint8_t *rx_buf,
                        size_t rx_size,
                        size_t out_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = sw->spi_dev;
    uint8_t *rxbuf = fifo_to_rxbuf(priv, cportid);
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

    _switch_spi_select(sw, true);

    // Read CNF and retry if NACK received
    do {
        // Read the CNF
        size = ES3_SWITCH_NCP_READ_LEN + rx_size;
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

    _switch_spi_select(sw, false);

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
    rc = es3_ncp_write(sw, SWITCH_FIFO_NCP, tx_buf, tx_size, &out_size);
    if (rc) {
        dbg_error("%s() write failed: rc=%d\n", __func__, rc);
        goto done;
    }

    /* Read the CNF, back-to-back after the NCP request */
    rc = es3_ncp_read(sw, SWITCH_FIFO_NCP, rx_buf, rx_size, out_size);
    if (rc) {
        dbg_error("%s() read failed: rc=%d\n", __func__, rc);
        goto done;
    }

done:
    pthread_mutex_unlock(&priv->ncp_cport.lock);

    return rc;
}

/* Status report data size */
#define SRPT_REPORT_SIZE             (12)
/* Status report total size: 7 bytes header + data + Switch reply delay */
#define SRPT_SIZE                   (7 + SRPT_REPORT_SIZE + \
                                     ES3_SWITCH_WAIT_REPLY_LEN)

struct __attribute__ ((__packed__)) srpt_read_status_report {
    unsigned char raw[SRPT_REPORT_SIZE];
    size_t rx_fifo_size; // number of bytes available in the rx buffer
    size_t tx_fifo_size; // bytes available for enqueue in the tx buffer
};

/**
 * @brief retrieve the state of the cport spi fifo.
 */
static int es3_read_status(struct tsb_switch *sw,
                           unsigned int cport,
                           struct srpt_read_status_report *status) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = sw->spi_dev;
    unsigned int offset;
    struct srpt_read_status_report *rpt = NULL;
    uint8_t *rxbuf = fifo_to_rxbuf(priv, cport);

    const char srpt_cmd[] = {SRPT, cport, 0, 0, ENDP};
    const char srpt_report_header[] = {SRPT, cport, 0x00, 0xC};

    _switch_spi_select(sw, true);

    SPI_SNDBLOCK(spi_dev, srpt_cmd, sizeof srpt_cmd);
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, SRPT_SIZE);

    /* Make sure we use 16-bit frames */
    if ((sizeof srpt_cmd + SRPT_SIZE) & 0x1) {
        SPI_SEND(spi_dev, LNUL);
    }

    _switch_spi_select(sw, false);

    dbg_insane("SRPT RX:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, SRPT_SIZE);

    /* Find the header */
    for (offset = 0; offset < (SRPT_SIZE - SRPT_REPORT_SIZE); offset++) {
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

        memcpy(status, rpt, SRPT_REPORT_SIZE);

        switch (cport) {
        case SWITCH_FIFO_NCP:
            fifo_max_size = SWITCH_CPORT_NCP_FIFO_SIZE;
            break;
        case SWITCH_FIFO_DATA4:
        case SWITCH_FIFO_DATA5:
        default:
            fifo_max_size = SWITCH_CPORT_DATA_FIFO_SIZE;
            break;
        }

        status->tx_fifo_size = rpt->raw[7] * 8;
        status->rx_fifo_size = fifo_max_size -
                               ((rpt->raw[10] << 8) | rpt->raw[11]);

        dbg_insane("SRPT: TX%u, RX%u\n", status->tx_fifo_size,
                   status->rx_fifo_size);
    }

    return 0;
}

static int es3_session_write(struct tsb_switch *sw,
                             uint8_t cportid,
                             uint8_t *tx_buf,
                             size_t tx_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = sw->spi_dev;
    struct srpt_read_status_report rpt;
    uint8_t *rxbuf = fifo_to_rxbuf(priv, cportid);
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
    if (tx_size >= SWITCH_CPORT_DATA_MAX_PAYLOAD) {
        return -EINVAL;
    }

    if (tx_size >= rpt.tx_fifo_size) {
        return -ENOMEM;
    }

    _switch_spi_select(sw, true);
    /* Write */
    SPI_SNDBLOCK(spi_dev, write_header, sizeof write_header);
    SPI_SNDBLOCK(spi_dev, tx_buf, tx_size);
    SPI_SNDBLOCK(spi_dev, write_trailer, sizeof write_trailer);

    /* Wait write status, send NULL frames while waiting */
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, ES3_SWITCH_WRITE_STATUS_NNULL);

    dbg_insane("Write payload:\n");
    dbg_print_buf(ARADBG_INSANE, tx_buf, tx_size);
    dbg_insane("Write status:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, ES3_SWITCH_WRITE_STATUS_NNULL);

    /* Make sure we use 16-bit frames */
    size = sizeof write_header + tx_size + sizeof write_trailer
           + ES3_SWITCH_WRITE_STATUS_NNULL;
    if (size & 1) {
        SPI_SEND(spi_dev, LNUL);
    }

    /* Parse the write status and bail on error. */
    ret = _switch_transfer_check_write_status(rxbuf,
                                              ES3_SWITCH_WRITE_STATUS_NNULL);
    if (ret) {
        goto out;
    }

out:
    _switch_spi_select(sw, false);
    return ret;
}

static int es3_session_read(struct tsb_switch *sw,
                            uint8_t cportid,
                            uint8_t *rx_buf,
                            size_t *rx_size) {
    struct sw_es3_priv *priv = sw->priv;
    struct spi_dev_s *spi_dev = sw->spi_dev;
    uint8_t *rxbuf = fifo_to_rxbuf(priv, cportid);
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

    _switch_spi_select(sw, true);

    // Read CNF and retry if NACK received
    do {
        // Read the CNF
        size = ES3_SWITCH_WAIT_REPLY_LEN + *rx_size + SWITCH_DATA_READ_LEN;
        SPI_SNDBLOCK(spi_dev, read_header, sizeof read_header);
        SPI_EXCHANGE(spi_dev, NULL, rxbuf, size);
        /* Make sure we use 16-bit frames */
        if (size & 0x1) {
            SPI_SEND(spi_dev, LNUL);
        }

        dbg_insane("RX raw Data (%u):\n", size);
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
                /* Return the actual message len */
                *rx_size = resp_len;
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

static int es3_irq_fifo_rx(struct tsb_switch *sw, unsigned int cportid) {
    struct sw_es3_priv *priv = sw->priv;
    struct es3_cport *cport;
    struct srpt_read_status_report rpt;
    size_t len;
    int rc;

    switch (cportid) {
    case SWITCH_FIFO_DATA4:
        cport = &priv->data_cport4;
        break;
    case SWITCH_FIFO_DATA5:
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
    /* Get the raw RX len */
    len = rpt.rx_fifo_size;

    /*
     * Drain the fifo data.
     * Pass the raw RX len and get the actual session data length back.
     */
    rc = es3_session_read(sw, cportid, cport->rxbuf, &len);
    if (rc) {
        dbg_error("%s: read failed: %d\n", __func__, rc);
        rc = -EIO;
        goto fill_done;
    }
    dbg_verbose("RX Data (%u):\n", len);
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

static uint8_t* es3_init_rxbuf(struct tsb_switch *sw) {
    /* The NCP CPorts are big enough for this. */
    struct sw_es3_priv *priv = sw->priv;
    return fifo_to_rxbuf(priv, SWITCH_FIFO_NCP);
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

static int es3_post_init_seq(struct tsb_switch *sw)
{
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

    return 0;
}

/*
 * NCP command request helpers
 */

static void es3_dme_set_req(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint16_t attrid,
                            uint16_t select_index,
                            uint32_t val,
                            uint8_t *req, size_t *req_size) {
    uint8_t set_req[] = {
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
    DEBUGASSERT(*req_size >= sizeof(set_req));
    memcpy(req, set_req, sizeof(set_req));
    *req_size = sizeof(set_req);
}

static void es3_dme_get_req(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint16_t attrid,
                            uint16_t select_index,
                            uint8_t *req, size_t *req_size) {
    uint8_t get_req[] = {
        NCP_RESERVED,
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

static void es3_dme_peer_set_req(struct tsb_switch *sw,
                                 uint8_t port_id,
                                 uint16_t attrid,
                                 uint16_t select_index,
                                 uint32_t val,
                                 uint8_t *req, size_t *req_size) {
    uint8_t peer_set[] = {
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
    DEBUGASSERT(*req_size >= sizeof(peer_set));
    memcpy(req, peer_set, sizeof(peer_set));
    *req_size = sizeof(peer_set);
}

static void es3_dme_peer_get_req(struct tsb_switch *sw,
                                 uint8_t port_id,
                                 uint16_t attrid,
                                 uint16_t select_index,
                                 uint8_t *req, size_t *req_size) {
    uint8_t peer_get[] = {
        NCP_RESERVED,
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

static void es3_lut_set_req(struct tsb_switch *sw,
                            uint8_t unipro_portid,
                            uint8_t lut_address,
                            uint8_t dest_portid,
                            uint8_t *req, size_t *req_size) {
    uint8_t lut_set[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_LUTSETREQ,
        lut_address,
        NCP_RESERVED,
        NCP_RESERVED,
        dest_portid
    };
    DEBUGASSERT(*req_size >= sizeof(lut_set));
    memcpy(req, lut_set, sizeof(lut_set));
    *req_size = sizeof(lut_set);
}

static void es3_lut_get_req(struct tsb_switch *sw,
                            uint8_t unipro_portid,
                            uint8_t lut_address,
                            uint8_t *req, size_t *req_size) {
    uint8_t lut_get[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_LUTGETREQ,
        lut_address,
        NCP_RESERVED,
    };
    DEBUGASSERT(*req_size >= sizeof(lut_get));
    memcpy(req, lut_get, sizeof(lut_get));
    *req_size = sizeof(lut_get);
}

static void es3_switch_attr_set_req(struct tsb_switch *sw,
                                    uint16_t attrid,
                                    uint32_t val,
                                    uint8_t *req, size_t *req_size) {
    uint8_t switch_attr_set[] = {
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
    DEBUGASSERT(*req_size >= sizeof(switch_attr_set));
    memcpy(req, switch_attr_set, sizeof(switch_attr_set));
    *req_size = sizeof(switch_attr_set);
}

static void es3_switch_attr_get_req(struct tsb_switch *sw,
                                    uint16_t attrid,
                                    uint8_t *req, size_t *req_size) {
    uint8_t switch_attr_get[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_SWITCHATTRGETREQ,
        (attrid & 0xFF00) >> 8,
        (attrid & 0xFF),
    };
    DEBUGASSERT(*req_size >= sizeof(switch_attr_get));
    memcpy(req, switch_attr_get, sizeof(switch_attr_get));
    *req_size = sizeof(switch_attr_get);
}

static void es3_sys_ctrl_set_req(struct tsb_switch *sw,
                                 uint16_t sc_addr,
                                 uint32_t val,
                                 uint8_t *req, size_t *req_size)
{
    uint8_t sys_ctrl_set[] = {
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
    DEBUGASSERT(*req_size >= sizeof(sys_ctrl_set));
    memcpy(req, sys_ctrl_set, sizeof(sys_ctrl_set));
    *req_size = sizeof(sys_ctrl_set);
}

static void es3_sys_ctrl_get_req(struct tsb_switch *sw,
                                 uint16_t sc_addr,
                                 uint8_t *req, size_t *req_size)
{
    uint8_t sys_ctrl_get[] = {
        NCP_RESERVED,
        NCP_RESERVED,
        NCP_SYSCTRLGETREQ,
        sc_addr >> 8,
        sc_addr & 0xff,
    };
    DEBUGASSERT(*req_size >= sizeof(sys_ctrl_get));
    memcpy(req, sys_ctrl_get, sizeof(sys_ctrl_get));
    *req_size = sizeof(sys_ctrl_get);
}

static void es3_qos_attr_set_req(struct tsb_switch *sw,
                                 uint8_t  portid,
                                 uint8_t  attrid,
                                 uint32_t attr_val,
                                 uint8_t *req, size_t *req_size) {
    uint8_t qos_attr_set[] = {
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
    DEBUGASSERT(*req_size >= sizeof(qos_attr_set));
    memcpy(req, qos_attr_set, sizeof(qos_attr_set));
    *req_size = sizeof(qos_attr_set);
}

static void es3_qos_attr_get_req(struct tsb_switch *sw,
                                 uint8_t portid,
                                 uint8_t attrid,
                                 uint8_t *req, size_t *req_size) {
    uint8_t qos_attr_get[] = {
        NCP_RESERVED,
        portid,
        NCP_QOSATTRGETREQ,
        NCP_RESERVED,
        attrid
    };
    DEBUGASSERT(*req_size >= sizeof(qos_attr_get));
    memcpy(req, qos_attr_get, sizeof(qos_attr_get));
    *req_size = sizeof(qos_attr_get);
}

static void es3_dev_id_mask_set_req(struct tsb_switch *sw,
                                    uint8_t unipro_portid,
                                    uint8_t *mask,
                                    uint8_t *req, size_t *req_size) {
    struct __attribute__ ((__packed__)) {
        uint8_t reserved;
        uint8_t portid;
        uint8_t function_id;
        uint8_t mask[128];
    } dev_id_mask_set = {
        NCP_RESERVED,
        unipro_portid,
        NCP_DESTVALIDSETREQ,
    };
    DEBUGASSERT(*req_size >= sizeof(dev_id_mask_set));
    memcpy(&dev_id_mask_set.mask, mask, sizeof(dev_id_mask_set.mask));
    memcpy(req, &dev_id_mask_set, sizeof(dev_id_mask_set));
    *req_size = sizeof(dev_id_mask_set);
}

static void es3_dev_id_mask_get_req(struct tsb_switch *sw,
                                    uint8_t unipro_portid,
                                    uint8_t *req, size_t *req_size) {
    uint8_t dev_id_mask_get[] = {
        NCP_RESERVED,
        unipro_portid,
        NCP_DESTVALIDGETREQ,
    };
    DEBUGASSERT(*req_size >= sizeof(dev_id_mask_get));
    memcpy(req, dev_id_mask_get, sizeof(dev_id_mask_get));
    *req_size = sizeof(dev_id_mask_get);
}

static void es3_switch_id_set_req(struct tsb_switch *sw,
                                  uint8_t cportid,
                                  uint8_t peer_cportid,
                                  uint8_t dis,
                                  uint8_t irt,
                                  uint8_t *req, size_t *req_size) {
    uint8_t switch_id_set[] = {
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
    DEBUGASSERT(*req_size >= sizeof(switch_id_set));
    memcpy(req, switch_id_set, sizeof(switch_id_set));
    *req_size = sizeof(switch_id_set);
}

/**
 * Send raw data down CPort 4. We're only using this for the SVC
 * connection, so fix the CPort number.
 */
static int es3_data_send(struct tsb_switch *sw, void *data, size_t len) {
    struct sw_es3_priv *priv = sw->priv;
    int rc;

    pthread_mutex_lock(&priv->data_cport4.lock);

    rc = es3_session_write(sw, SWITCH_FIFO_DATA4, data, len);
    if (rc) {
        dbg_error("%s() write failed: rc=%d\n", __func__, rc);
        goto done;
    }

done:
    pthread_mutex_unlock(&priv->data_cport4.lock);

    return rc;
}

static struct tsb_rev_data es3_rev_data = {
    .wait_reply_len         = ES3_SWITCH_WAIT_REPLY_LEN,
    .ncp_req_max_size       = ES3_NCP_MAX_REQ_SIZE,
    .dev_id_mask_size       = ES3_DEST_VALID_MASK_SIZE,
    .spicee_enable_all      = ES3_SPICEE_ENABLE_ALL,
    .spi3ee_enable_all      = ES3_SPI3EE_ENABLE_ALL,
    .spi45ee_enable_all     = ES3_SPI45EE_ENABLE_ALL,
};

static struct tsb_switch_ops es3_ops = {
    .enable_port           = es3_enable_port,
    .link_startup          = es3_linkstartup_request,

    .switch_id_set_req     = es3_switch_id_set_req,

    .set_req               = es3_dme_set_req,
    .get_req               = es3_dme_get_req,

    .peer_set_req          = es3_dme_peer_set_req,
    .peer_get_req          = es3_dme_peer_get_req,

    .lut_set_req           = es3_lut_set_req,
    .lut_get_req           = es3_lut_get_req,

    .switch_attr_get_req   = es3_switch_attr_get_req,
    .switch_attr_set_req   = es3_switch_attr_set_req,

    .sys_ctrl_set_req      = es3_sys_ctrl_set_req,
    .sys_ctrl_get_req      = es3_sys_ctrl_get_req,

    .qos_attr_set_req      = es3_qos_attr_set_req,
    .qos_attr_get_req      = es3_qos_attr_get_req,

    .dev_id_mask_get_req   = es3_dev_id_mask_get_req,
    .dev_id_mask_set_req   = es3_dev_id_mask_set_req,

    .switch_data_send      = es3_data_send,

    .__switch_init_rxbuf   = es3_init_rxbuf,
    .__post_init_seq       = es3_post_init_seq,
    .__irq_fifo_rx         = es3_irq_fifo_rx,
    .__ncp_transfer        = es3_ncp_transfer,
    .__set_valid_entry     = es3_set_valid_entry,
    .__check_valid_entry   = es3_check_valid_entry,
};

int tsb_switch_es3_init(struct tsb_switch *sw, struct spi_dev_s *spi_dev) {
    struct sw_es3_priv *priv;
    int rc = 0;

    priv = malloc(sizeof(struct sw_es3_priv));
    if (!priv) {
        dbg_error("%s: Failed to alloc the priv struct\n", __func__);
        rc = -ENOMEM;
        goto error;
    }

    pthread_mutex_init(&priv->ncp_cport.lock, NULL);
    pthread_mutex_init(&priv->data_cport4.lock, NULL);
    pthread_mutex_init(&priv->data_cport5.lock, NULL);

    sw->priv = priv;
    sw->ops = &es3_ops;
    sw->rdata = &es3_rev_data;

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
