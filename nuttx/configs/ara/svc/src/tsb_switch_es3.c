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
/* Write status reply length */
#define ES3_SWITCH_WRITE_STATUS_LEN    (9)
/* Total number of NULLs to clock out to ensure a write status is read */
#define ES3_SWITCH_WRITE_STATUS_NNULL  (ES3_SWITCH_WAIT_REPLY_LEN + \
                                        ES3_SWITCH_WRITE_STATUS_LEN)

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
/*  Enable all Unipro and M-ports clocks */
#define SC_SYSCLOCKENABLE_ALL_PORTS (0x3FFF3FFF)
/*  Enable VDDn for M-port */
#define SC_VDDVN_PORT(i)            (1 << i)

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

static void es3_set_valid_entry(struct tsb_switch *sw, /* FIXME */
                                uint8_t *table, int entry, bool valid) {
    DEBUGASSERT(0);
}

static bool es3_check_valid_entry(struct tsb_switch *sw, /* FIXME */
                                  uint8_t *table, int entry) {
    return false;
}

static int es3_ncp_transfer(struct tsb_switch *sw, /* FIXME */
                            uint8_t *tx_buf,
                            size_t tx_size,
                            uint8_t *rx_buf,
                            size_t rx_size) {
    return -1;
}

static int es3_irq_fifo_rx(struct tsb_switch *sw, unsigned int cportid) { /* FIXME */
    return -1;
}

static uint8_t* es3_init_rxbuf(struct tsb_switch *sw) {
    /* The NCP CPorts are big enough for this. */
    struct sw_es3_priv *priv = sw->priv;
    return fifo_to_rxbuf(priv, SWITCH_FIFO_NCP);
}

static int es3_enable_port(struct tsb_switch *sw, uint8_t port) {
    uint32_t value = 0;

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

    /* Turn off isolation between VDDVn domain and VDDV domain (VDDVnIsoClr) */
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

    return 0;
}

static int es3_post_init_seq(struct tsb_switch *sw) {
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

    /* Setup automatic LinkStartupMode for all ports */
    if (switch_sys_ctrl_set(sw, SC_LINKSTARTUPMODE, 0)) {
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
 * Send raw data down CPort 4
 */
static int es3_data_send(struct tsb_switch *sw, void *data, size_t len) { /* FIXME */
    return -1;
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
