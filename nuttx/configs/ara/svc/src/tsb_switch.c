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
 * @author: Perry Hung
 * @author: Jean Pihet
 */

#define DBG_COMP    ARADBG_SWITCH
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/unipro/unipro.h>
#include <arch/byteorder.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include <interface.h>

#include <nuttx/util.h>

#include "stm32.h"
#include <ara_debug.h>
#include "tsb_switch.h"
#include "vreg.h"

#define SWITCH_SPI_INIT_DELAY_US    (700)

#define SWITCH_SPI_FREQUENCY        13000000

#define IRQ_WORKER_DEFPRIO          50
#define IRQ_WORKER_STACKSIZE        2048

#define SWITCH_SETTLE_TIME_S        (1)

/*
 * CportID and peerCPortID for L4 access by the SVC
 *
 * L4 CPortID: 0 for SVC -> CC, 2 for CC -> SVC
 * Peer CPortID: 3 for SVC -> CC, 2 for CC -> SVC
 */
#define L4_CPORT_SVC_TO_CC          0
#define L4_CPORT_CC_TO_SVC          2
#define L4_PEERCPORT_SVC_TO_CC      3
#define L4_PEERCPORT_CC_TO_SVC      2

/* NCP_SwitchIDSetReq: DIS and IRT fields values */
#define CPORT_ENABLE                0
#define CPORT_DISABLE               1
#define IRT_DISABLE                 0
#define IRT_ENABLE                  1

/* Default CPort configuration values. */
#define CPORT_DEFAULT_TOKENVALUE           32
#define CPORT_DEFAULT_T_PROTOCOLID         0
#define CPORT_DEFAULT_TSB_MAXSEGMENTCONFIG 0x118

/* Switch write status */
#define WSTATUS0_TXENTFIFOREMAIN_MASK  (0x03)
#define WSTATUS1_TXENTFIFOREMAIN_MASK  (0xff)
#define WSTATUS2_TXDATAFIFOREMAIN_MASK (0x7f)
#define WSTATUS3_LENERR_MASK           (0x40)
#define WSTATUS3_TXENTFIFOFULL_MASK    (0x02)
#define WSTATUS3_TXDATAFIFOFULL_MASK   (0x01)

extern int _switch_internal_set_id(struct tsb_switch *,
                                   uint8_t, uint8_t, uint8_t, uint8_t);


/*
 * Common internal switch communication helpers
 */

void _switch_spi_select(struct tsb_switch *sw, int select) {
    if (select) {
        SPI_LOCK(spi_dev, true);
    }
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
    if (!select) {
        SPI_LOCK(spi_dev, false);
    }
}

int _switch_transfer_check_write_status(uint8_t *status_block, size_t size)
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
            dbg_print_buf(ARADBG_ERROR, status_block, size);
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


/*
 * Device ID helpers
 */

static inline void dev_ids_update(struct tsb_switch *sw,
                                  uint8_t port_id,
                                  uint8_t dev_id) {
    sw->dev_ids[port_id] = dev_id;
}

static inline uint8_t dev_ids_port_to_dev(struct tsb_switch *sw,
                                          uint8_t port_id) {
    if (port_id >= SWITCH_PORT_MAX)
         return INVALID_PORT;

    return sw->dev_ids[port_id];
}

static inline uint8_t dev_ids_dev_to_port(struct tsb_switch *sw,
                                          uint8_t dev_id) {
    int i;

    for (i = 0; i < SWITCH_PORT_MAX; i++) {
        if (sw->dev_ids[i] == dev_id) {
             return i;
        }
    }

    return INVALID_PORT;
}

static void dev_ids_destroy(struct tsb_switch *sw) {
    memset(sw->dev_ids, INVALID_PORT, sizeof(sw->dev_ids));
}

static void switch_power_on_reset(struct tsb_switch *sw)
{
    /* Enable the switch power supplies regulator */
    vreg_get(sw->pdata->vreg);

    /* Release reset */
    stm32_gpiowrite(sw->pdata->gpio_reset, true);
    sleep(SWITCH_SETTLE_TIME_S);
}

static void switch_power_off(struct tsb_switch *sw)
{
    /* Release the switch power supplies regulator */
    vreg_put(sw->pdata->vreg);

    /* Assert reset */
    stm32_gpiowrite(sw->pdata->gpio_reset, false);
}

static uint8_t* switch_init_rxbuf(struct tsb_switch *sw)
{
    DEBUGASSERT(sw->ops->__switch_init_rxbuf(sw));
    return sw->ops->__switch_init_rxbuf(sw);
}

/* Switch communication link init and de-init */
static int switch_init_comm(struct tsb_switch *sw)
{
    uint8_t *rxbuf = switch_init_rxbuf(sw);
    struct spi_dev_s *spi_dev = sw->spi_dev;
    const char init_reply[] = { INIT, LNUL };
    int i, rc = -1;

    dbg_info("Initializing switch SPI communications...\n");
    _switch_spi_select(sw, true);

    up_udelay(SWITCH_SPI_INIT_DELAY_US);

    SPI_SEND(spi_dev, INIT);
    SPI_SEND(spi_dev, INIT);
    SPI_EXCHANGE(spi_dev, NULL, rxbuf, sw->rdata->wait_reply_len);

    dbg_insane("Init RX Data:\n");
    dbg_print_buf(ARADBG_INSANE, rxbuf, sw->rdata->wait_reply_len);

    /* Check for the transition from INIT to LNUL after sending INITs */
    for (i = 0; i < sw->rdata->wait_reply_len - 1; i++) {
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
    return 0;
}

/* Linkstartup request */
int switch_link_startup(struct tsb_switch *sw,
                        uint8_t portid)
{
    if (!sw->ops->link_startup) {
        return -EOPNOTSUPP;
    }
    return sw->ops->link_startup(sw, portid);
}

/* Switch port enable (VDD and clocks) */
int switch_enable_port(struct tsb_switch *sw,
                       uint8_t portid,
                       bool enable)
{
    if (!sw->ops->enable_port) {
        return -EOPNOTSUPP;
    }
    return sw->ops->enable_port(sw, portid, enable);
}

static int set_valid_entry(struct tsb_switch *sw,
                           uint8_t *table, int entry, bool valid) {
    DEBUGASSERT(sw->ops->__set_valid_entry);
    return sw->ops->__set_valid_entry(sw, table, entry, valid);
}

int switch_set_valid_device(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint8_t device_id,
                            bool valid) {
    uint8_t id_mask[sw->rdata->dev_id_mask_size];
    int rc;

    rc = switch_dev_id_mask_get(sw, port_id, id_mask);
    if (rc) {
        dbg_error("Failed to get valid device bitmask for port %u\n", port_id);
        return rc;
    }

    rc = set_valid_entry(sw, id_mask, device_id, valid);
    if (rc) {
        dbg_error("Failed to set valid device bitmask for port %u\n", port_id);
        return rc;
    }

    rc = switch_dev_id_mask_set(sw, port_id, id_mask);
    if (rc) {
        dbg_error("Failed to set valid device bitmask for port %u\n", port_id);
        return rc;
    }

    return rc;
}

static bool check_valid_entry(struct tsb_switch *sw,
                              uint8_t *table, int entry) {
    DEBUGASSERT(sw->ops->__check_valid_entry);
    return sw->ops->__check_valid_entry(sw, table, entry);
}

/**
 * @brief Log the UniPro switch's routing table
 */
int switch_dump_routing_table(struct tsb_switch *sw) {
    int devid, unipro_portid, rc;
    uint8_t p = 0, id_mask[sw->rdata->dev_id_mask_size];

    dbg_info("======================================================\n");
    dbg_info("Routing table:\n");
    dbg_info(" [Port,DevId] -> [Port]\n");

    for (unipro_portid = 0; unipro_portid <= SWITCH_PORT_ID; unipro_portid++) {
        rc = switch_dev_id_mask_get(sw, unipro_portid, id_mask);
        /* Check for error. Ignore disabled Switch ports */
        if (rc) {
            if (rc == NCP_RC_DISABLED_TARGET) {
                continue;
            }
            dbg_error("%s() Failed to retrieve routing table.\n", __func__);
            return -1;
        }
        dbg_insane("%s(): Mask ID %d\n", __func__, unipro_portid);
        dbg_print_buf(ARADBG_INSANE, id_mask, sizeof(id_mask));

        for (devid = 0; devid < N_MAXDEVICEID; devid++) {
            if (check_valid_entry(sw, id_mask, devid)) {
                switch_lut_get(sw, unipro_portid, devid, &p);
                dbg_info(" [%2u,%2u] -> %2u\n", unipro_portid, devid, p);
            }
        }
    }

    dbg_info("======================================================\n");

    return 0;
}

int switch_enable_test_traffic(struct tsb_switch *sw,
                               uint8_t src_portid, uint8_t dst_portid,
                               const struct unipro_test_feature_cfg *cfg) {
    int i, rc;
    uint16_t src_cportid = cfg->tf_src_cportid;
    uint16_t dst_cportid = cfg->tf_dst_cportid;
    struct pasv {
        uint8_t  p; /* port */
        uint16_t a; /* attribute */
        uint16_t s; /* selector index */
        uint32_t v; /* value */
    } test_src_enable[] = {
        /*
         * First, disable the CPorts and configure them for test
         * feature use.
         */
        {.p = src_portid,
         .a = T_CONNECTIONSTATE,
         .s = src_cportid,
         .v = 0},
        {.p = dst_portid,
         .a = T_CONNECTIONSTATE,
         .s = dst_cportid,
         .v = 0},

        {.p = src_portid,
         .a = T_CPORTMODE,
         .s = src_cportid,
         .v = CPORT_MODE_UNDER_TEST},
        {.p = dst_portid,
         .a = T_CPORTMODE,
         .s = dst_cportid,
         .v = CPORT_MODE_UNDER_TEST},

        /*
         * Next, configure the test feature at the source.
         */
        {.p = src_portid,
         .a = T_TSTCPORTID,
         .s = cfg->tf_src,
         .v = src_cportid},
        {.p = src_portid,
         .a = T_TSTSRCINCREMENT,
         .s = cfg->tf_src,
         .v = cfg->tf_src_inc},
        {.p = src_portid,
         .a = T_TSTSRCMESSAGESIZE,
         .s = cfg->tf_src,
         .v = cfg->tf_src_size},
        {.p = src_portid,
         .a = T_TSTSRCMESSAGECOUNT,
         .s = cfg->tf_src,
         .v = cfg->tf_src_count},
        {.p = src_portid,
         .a = T_TSTSRCINTERMESSAGEGAP,
         .s = cfg->tf_src,
         .v = cfg->tf_src_gap_us},

        /*
         * Then configure the test feature at the destination.
         */
        {.p = dst_portid,
         .a = T_TSTCPORTID,
         .s = cfg->tf_dst,
         .v = dst_cportid},
        {.p = dst_portid,
         .a = T_TSTDSTERRORDETECTIONENABLE,
         .s = cfg->tf_dst,
         .v = cfg->tf_dst_error_detection_enable},
        {.p = dst_portid,
         .a = T_TSTDSTINCREMENT,
         .s = cfg->tf_dst,
         .v = cfg->tf_src_inc},
        {.p = dst_portid,
         .a = T_TSTDSTMESSAGESIZE,
         .s = cfg->tf_dst,
         .v = cfg->tf_src_size},
        {.p = dst_portid,
         .a = T_TSTDSTMESSAGECOUNT,
         .s = cfg->tf_dst,
         .v = 0 /* Reset message count when starting the test */ },

        /*
         * Finally, connect the CPorts again, and turn on the source
         * and the destination.
         */
        {.p = src_portid,
         .a = T_CONNECTIONSTATE,
         .s = src_cportid,
         .v = 1},
        {.p = dst_portid,
         .a = T_CONNECTIONSTATE,
         .s = dst_cportid,
         .v = 1},
        {.p = src_portid,
         .a = T_TSTSRCON,
         .s = cfg->tf_src,
         .v = 1},
        {.p = dst_portid,
         .a = T_TSTDSTON,
         .s = cfg->tf_dst,
         .v = 1},
    };

    if (!sw ||
        src_portid >= SWITCH_PORT_MAX || dst_portid >= SWITCH_PORT_MAX) {
        return -EINVAL;
    }

    dbg_info("Enabling UniPro test feature: port=%u,src=%u->port=%u,dst=%u\n",
             src_portid, cfg->tf_src, dst_portid, cfg->tf_dst);
    dbg_info("Test source: cport=%u, inc=%u, size=%u, count=%u, gap=%u\n",
             src_cportid, cfg->tf_src_inc, cfg->tf_src_size, cfg->tf_src_count,
             cfg->tf_src_gap_us);
    if (cfg->tf_dst_error_detection_enable) {
        dbg_info("Test destination: cport=%u, inc=%u, size=%u, count=%u\n. Error checking enabled.\n",
                 dst_cportid, cfg->tf_src_inc, cfg->tf_src_size,
                 cfg->tf_src_count);
    } else {
        dbg_info("Test destination: cport=%u. Error checking not enabled.\n",
                 dst_cportid);
    }

    for (i = 0; i < ARRAY_SIZE(test_src_enable); i++) {
        struct pasv *s = &test_src_enable[i];
        rc = switch_dme_peer_set(sw, s->p, s->a, s->s, s->v);
        if (rc) {
            return rc;
        }
    }

    return 0;
}

int switch_disable_test_traffic(struct tsb_switch *sw,
                                uint8_t src_portid, uint8_t dst_portid,
                                const struct unipro_test_feature_cfg *cfg) {
    int rc, i;
    uint16_t src_cportid = cfg->tf_src_cportid;
    uint16_t dst_cportid = cfg->tf_dst_cportid;
    struct pasv {
        uint8_t  p; /* port */
        uint16_t a; /* attribute */
        uint16_t s; /* selector index */
        uint32_t v; /* value */
    } test_src_disable[] = {
        /*
         * Stop test feature traffic generation.
         */
        {.p = src_portid,
         .a = T_TSTSRCON,
         .s = cfg->tf_src,
         .v = 0},
        {.p = dst_portid,
         .a = T_TSTDSTON,
         .s = cfg->tf_dst,
         .v = 0},

        /*
         * Clean up by disconnecting the CPorts and returning them to
         * the application.
         */
        {.p = src_portid,
         .a = T_CONNECTIONSTATE,
         .s = src_cportid,
         .v = 0},
        {.p = dst_portid,
         .a = T_CONNECTIONSTATE,
         .s = dst_cportid,
         .v = 0},

        {.p = src_portid,
         .a = T_CPORTMODE,
         .s = src_cportid,
         .v = CPORT_MODE_APPLICATION},
        {.p = dst_portid,
         .a = T_CPORTMODE,
         .s = dst_cportid,
         .v = CPORT_MODE_APPLICATION},
    };

    if (!sw ||
        src_portid >= SWITCH_PORT_MAX || dst_portid >= SWITCH_PORT_MAX) {
        return -EINVAL;
    }

    dbg_info("Disabling UniPro test feature: port=%u,src=%u->port=%u,dst=%u\n",
             src_portid, cfg->tf_src, dst_portid, cfg->tf_dst);
    for (i = 0; i < ARRAY_SIZE(test_src_disable); i++) {
        struct pasv *s = &test_src_disable[i];
        rc = switch_dme_peer_set(sw, s->p, s->a, s->s, s->v);
        if (rc) {
            return rc;
        }
    }

    return 0;
}

int switch_qos_band_reset(struct tsb_switch *sw, uint8_t portid) {
    if (!sw || portid >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    return switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_CTRL, 1 << (portid + 16));
}

int switch_qos_bwctrl_enabled(struct tsb_switch *sw, uint8_t portid, uint8_t tc,
                              uint8_t *val) {
    int rc;
    uint8_t drr2, priolut;
    uint32_t attr_val;

    if (!sw ||
        portid >= SWITCH_UNIPORT_MAX ||
        !val) {
        return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0BAND:
        *val = true;
        break;
    case SWITCH_TRAFFIC_CLASS_TC0HIGH:
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_CTRL, &attr_val);
        if (rc) {
            return rc;
        }

        drr2 = attr_val & AR_CTRL_DRR2ENABLE;

        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_PRIOLUT, &attr_val);
        if (rc) {
            return rc;
        }

        priolut = (attr_val >> (portid * 2)) & ~3;

        *val = (priolut >> 1) || (priolut && drr2);
        break;
    default:
        *val = false;
        break;
    }

    return 0;
}

int switch_qos_enable_bwctrl(struct tsb_switch *sw, uint8_t portid, uint8_t tc) {
    int rc;
    uint32_t attr_val;

    if (!sw ||
        tc != SWITCH_TRAFFIC_CLASS_TC0HIGH ||
        portid >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_CTRL, &attr_val);
    if (rc) {
        return rc;
    }

    attr_val |= AR_CTRL_DRR2ENABLE;
    return switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_CTRL, attr_val);
}

int switch_qos_disable_bwctrl(struct tsb_switch *sw, uint8_t portid, uint8_t tc) {
    int rc;
    uint32_t attr_val;

    if (!sw ||
        tc != SWITCH_TRAFFIC_CLASS_TC0HIGH ||
        portid >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_CTRL, &attr_val);
    if (rc) {
        return rc;
    }

    attr_val &= ~AR_CTRL_DRR2ENABLE;
    return switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_CTRL, attr_val);
}

int switch_qos_get_subtc(struct tsb_switch *sw, uint8_t portid, uint8_t *tc) {
    int rc;
    uint32_t attr_val, mask;

    if (!sw || portid >= SWITCH_UNIPORT_MAX || !tc) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_PRIOLUT, &attr_val);
    if (rc) {
        return rc;
    }

    mask = attr_val & AR_PRIOLUT_PCP(portid);
    if (mask & AR_PRIOLUT_PCP_MASK(portid, 0x2)) {
        *tc = SWITCH_TRAFFIC_CLASS_TC0BAND;
    } else if (mask & AR_PRIOLUT_PCP_MASK(portid, 0x1)) {
        *tc = SWITCH_TRAFFIC_CLASS_TC0HIGH;
    } else {
        *tc = SWITCH_TRAFFIC_CLASS_TC0;
    }

    return 0;
}

int switch_qos_set_subtc(struct tsb_switch *sw, uint8_t portid, uint8_t tc) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_PRIOLUT, &attr_val);
    if (rc) {
        return rc;
    }

    attr_val &= ~AR_PRIOLUT_PCP_MASK(portid, 0x3);
    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0BAND:
        attr_val |= AR_PRIOLUT_PCP_MASK(portid, 0x2);
        break;
    case SWITCH_TRAFFIC_CLASS_TC0HIGH:
        attr_val |= AR_PRIOLUT_PCP_MASK(portid, 0x1);
        break;
    case SWITCH_TRAFFIC_CLASS_TC0:
        attr_val |= AR_PRIOLUT_PCP_MASK(portid, 0x0);
        break;
    default:
        return -EINVAL;
        break;
    }

    return switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_PRIOLUT, attr_val);
}

int switch_qos_port_closed(struct tsb_switch *sw, uint8_t portid, bool *closed) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX || !closed) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, portid, AR_CTRL, &attr_val);
    if (rc) {
        return rc;
    }

    *closed = (bool)(attr_val & AR_CTRL_CLOSURE);

    return 0;
}

int switch_qos_open_port(struct tsb_switch *sw, uint8_t portid) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX) {
        if (portid == SWITCH_PORT_ID) {
            dbg_error("Cannot open or close internal switch port.\n");
        }
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, portid, AR_CTRL, &attr_val);
    if (rc) {
        return rc;
    }

    attr_val &= ~AR_CTRL_CLOSURE;

    return switch_qos_attr_set(sw, portid, AR_CTRL, attr_val);
}

int switch_qos_close_port(struct tsb_switch *sw, uint8_t portid) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX) {
        if (portid == SWITCH_PORT_ID) {
            dbg_error("Cannot open or close internal switch port.\n");
        }
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, portid, AR_CTRL, &attr_val);
    if (rc) {
        return rc;
    }

    attr_val |= AR_CTRL_CLOSURE;

    return switch_qos_attr_set(sw, portid, AR_CTRL, attr_val);
}

int switch_qos_peer_implements_tc(struct tsb_switch *sw, uint8_t portid,
                                  uint8_t tc, uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, portid, AR_STATUS_INTERRUPT, &attr_val);
    if (rc) {
        return rc;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        *val = attr_val & AR_STATUS_INTERRUPT_PRESENT_TC0;
        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        *val = attr_val & AR_STATUS_INTERRUPT_PRESENT_TC1;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_transmit_accept_signal(struct tsb_switch *sw, uint8_t portid,
                                      uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, portid, AR_STATUS_INTERRUPT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = attr_val & AR_STATUS_INTERRUPT_WDT_A;

    return 0;
}

int switch_qos_transmit_valid_signal(struct tsb_switch *sw, uint8_t portid,
                                     uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || portid >= SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, portid, AR_STATUS_INTERRUPT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = attr_val & AR_STATUS_INTERRUPT_WDT_V;

    return 0;
}

int switch_qos_gate_arb_busy(struct tsb_switch *sw, uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_CONNECT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = attr_val & AR_STATUS_CONNECT_ARB_BUSY;

    return 0;
}

int switch_qos_gate_arb_output(struct tsb_switch *sw, uint8_t port,
                               uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_CONNECT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = attr_val & AR_STATUS_CONNECT_ARB_OUTPUT;

    return 0;
}

int switch_qos_output_traffic_class(struct tsb_switch *sw, uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_CONNECT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = 0;
    if (attr_val & AR_STATUS_CONNECT_CLASS_TC0) {
        *val |= (AR_STATUS_CONNECT_CLASS_TC0 >> 16);
    }
    if (attr_val & AR_STATUS_CONNECT_CLASS_TC0HIGH) {
        *val |= (AR_STATUS_CONNECT_CLASS_TC0HIGH >> 16);
    }
    if (attr_val & AR_STATUS_CONNECT_CLASS_TC0BAND) {
        *val |= (AR_STATUS_CONNECT_CLASS_TC0BAND >> 16);
    }
    if (attr_val & AR_STATUS_CONNECT_CLASS_TC1) {
        *val |= (AR_STATUS_CONNECT_CLASS_TC1 >> 16);
    }

    return 0;
}

int switch_qos_port_connected(struct tsb_switch *sw, uint8_t port,
                              uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_CONNECT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = attr_val & AR_STATUS_CONNECT_PORT(port);

    return 0;
}

int switch_qos_request_status(struct tsb_switch *sw, uint8_t tc, uint8_t port,
                              uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_REQ0,
                                 &attr_val);
        if (rc) {
            return rc;
        }

        *val = attr_val & AR_STATUS_REQ0_REQ_TC0(port);
        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0HIGH: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_REQ0,
                                 &attr_val);
        if (rc) {
            return rc;
        }

        *val = attr_val & AR_STATUS_REQ0_REQ_TC0HIGH(port);
        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0BAND: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_REQ1,
                                 &attr_val);
        if (rc) {
            return rc;
        }

        *val = attr_val & AR_STATUS_REQ1_REQ_TC0BAND(port);
        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC1: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_STATUS_REQ1,
                                 &attr_val);
        if (rc) {
            return rc;
        }

        *val = attr_val & AR_STATUS_REQ1_REQ_TC1(port);
        break;
    }
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_get_out_wdt_count(struct tsb_switch *sw, uint32_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || !val) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_WDT, &attr_val);
    if (rc) {
        return rc;
    }

    *val = attr_val & AR_WDT_WDT_COUNT;
    return 0;
}

int switch_qos_set_out_wdt_count(struct tsb_switch *sw, uint32_t val) {
    if (!sw || val & ~AR_WDT_WDT_COUNT) {
        return -EINVAL;
    }

    return switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_WDT, val);
}

int switch_qos_get_bwperiod(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                            uint32_t *val) {
    int rc;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_BCFGPERIOD0, val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0HIGH: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID,
                                 AR_DRR2CFG_PERIOD(port), val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0BAND: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID,
                                 AR_DRR1CFG_PERIOD(port), val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC1: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_BCFGPERIOD1, val);
        if (rc) {
            return rc;
        }

        break;
    }
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_set_bwperiod(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                            uint32_t val) {
    int rc;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0: {
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_BCFGPERIOD0, val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0HIGH: {
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID,
                                 AR_DRR2CFG_PERIOD(port), val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0BAND: {
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID,
                                 AR_DRR1CFG_PERIOD(port), val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC1: {
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_BCFGPERIOD1, val);
        if (rc) {
            return rc;
        }

        break;
    }
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_get_decsize(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                           uint32_t *val) {
    int rc;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_BCFGDEC0, val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0HIGH: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_DRR2CFG_DEC(port),
                                 val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC0BAND: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_DRR1CFG_DEC(port),
                                 val);
        if (rc) {
            return rc;
        }

        break;
    }
    case SWITCH_TRAFFIC_CLASS_TC1: {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_BCFGDEC1, val);
        if (rc) {
            return rc;
        }

        break;
    }
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_set_decsize(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                           uint32_t val) {
    int rc;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
       return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_BCFGDEC0, val);
        if (rc) {
            return rc;
        }
        break;
    case SWITCH_TRAFFIC_CLASS_TC0HIGH:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_DRR2CFG_DEC(port),
                                 val);
        if (rc) {
            return rc;
        }
        break;
    case SWITCH_TRAFFIC_CLASS_TC0BAND:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_DRR1CFG_DEC(port),
                                 val);
        if (rc) {
            return rc;
        }
        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_BCFGDEC1, val);
        if (rc) {
            return rc;
        }
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_get_limit(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                         uint32_t *val) {
    int rc;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
       return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_BCFGLIMIT0, val);
        if (rc) {
            return rc;
        }

        break;
    case SWITCH_TRAFFIC_CLASS_TC0HIGH:
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_DRR1CFG_LIMIT(port),
                                 val);
        if (rc) {
            return rc;
        }

        break;
    case SWITCH_TRAFFIC_CLASS_TC0BAND:
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_DRR1CFG_LIMIT(port),
                                 val);
        if (rc) {
            return rc;
        }

        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_BCFGLIMIT1, val);
        if (rc) {
            return rc;
        }

        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_set_limit(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                         uint32_t val) {
    int rc;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
       return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_BCFGLIMIT0, val);
        if (rc) {
            return rc;
        }
        break;
    case SWITCH_TRAFFIC_CLASS_TC0HIGH:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_DRR2CFG_LIMIT(port),
                                 val);
        if (rc) {
            return rc;
        }
        break;
    case SWITCH_TRAFFIC_CLASS_TC0BAND:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, AR_DRR1CFG_LIMIT(port),
                                 val);
        if (rc) {
            return rc;
        }
        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        rc = switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_BCFGLIMIT1, val);
        if (rc) {
            return rc;
        }
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_source_quantity(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                               uint32_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || port > SWITCH_UNIPORT_MAX || !val) {
        return -EINVAL;
    }

    if (tc == SWITCH_TRAFFIC_CLASS_TC0BAND) {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_DRR1STAT_QUANTITY(port),
                                 val);
        if (rc) {
            return rc;
        }
    } else {
        rc = switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_BSTAT_QUANTITY(port),
                                 &attr_val);
        if (rc) {
            return rc;
        }

        switch (tc) {
        case SWITCH_TRAFFIC_CLASS_TC0:
            *val = attr_val & AR_BSTAT_QUANTITY_RATE00_TC0;
            break;
        case SWITCH_TRAFFIC_CLASS_TC1:
            *val = attr_val & AR_BSTAT_QUANTITY_RATE00_TC1 >> 16;
            break;
        default:
            return -EINVAL;
            break;
        }
    }

    return 0;
}

int switch_qos_transmit_quantity(struct tsb_switch *sw, uint32_t *val) {
    if (!sw || !val) {
        return -EINVAL;
    }

    return switch_qos_attr_get(sw, SWITCH_PORT_ID, AR_BSTAT_QUANTITYTX, val);
}

int switch_qos_reset_routing_table(struct tsb_switch *sw, uint8_t tc) {
    uint32_t attr_val = 0;

    if (!sw) {
        return -EINVAL;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        attr_val |= RT_CTRL_RST_TC0;
        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        attr_val |= RT_CTRL_RST_TC1;
        break;
    default:
        return -EINVAL;
        break;
    }

    return switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_CTRL, attr_val);
}

int switch_qos_source_accept_signal(struct tsb_switch *sw, uint8_t port,
                                    uint8_t tc, uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || !val || port >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, port, RT_STATUS, &attr_val);
    if (rc) {
        return rc;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        *val = attr_val & RT_STATUS_WDT_A_TC0;
        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        *val = attr_val & RT_STATUS_WDT_A_TC1;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_source_valid_signal(struct tsb_switch *sw, uint8_t port,
                                   uint8_t tc, uint8_t *val) {
    int rc;
    uint32_t attr_val;

    if (!sw || !val || port >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_qos_attr_get(sw, port, RT_STATUS, &attr_val);
    if (rc) {
        return rc;
    }

    switch (tc) {
    case SWITCH_TRAFFIC_CLASS_TC0:
        *val = attr_val & RT_STATUS_WDT_V_TC0;
        break;
    case SWITCH_TRAFFIC_CLASS_TC1:
        *val = attr_val & RT_STATUS_WDT_V_TC1;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

int switch_qos_get_in_wdt_count(struct tsb_switch *sw, uint32_t *val) {
    if (!sw || !val) {
        return -EINVAL;
    }

    return switch_qos_attr_get(sw, SWITCH_PORT_ID, RT_WDT, val);
}

int switch_qos_set_in_wdt_count(struct tsb_switch *sw, uint32_t val) {
    if (!sw) {
        return -EINVAL;
    }

    return switch_qos_attr_set(sw, SWITCH_PORT_ID, RT_WDT, val);
}

static int switch_set_port_l4attr(struct tsb_switch *sw,
                                  uint8_t portid,
                                  uint16_t attrid,
                                  uint16_t selector,
                                  uint32_t val) {
    int rc;

    if (portid == SWITCH_PORT_ID) {
        rc = switch_dme_set(sw, portid, attrid, selector, val);
    } else {
        rc = switch_dme_peer_set(sw, portid, attrid, selector, val);
    }

    return rc;
}

static int switch_get_port_l4attr(struct tsb_switch *sw,
                                  uint8_t portid,
                                  uint16_t attrid,
                                  uint16_t selector,
                                  uint32_t *val) {
    int rc;

    if (portid == SWITCH_PORT_ID) {
        rc = switch_dme_get(sw, portid, attrid, selector, val);
    } else {
        rc = switch_dme_peer_get(sw, portid, attrid, selector, val);
    }

    return rc;
}


static int switch_set_pair_attr(struct tsb_switch *sw,
                                struct unipro_connection *c,
                                uint16_t attrid,
                                uint32_t val0,
                                uint32_t val1) {
    int rc;
    rc = switch_set_port_l4attr(sw,
            c->port_id0,
            attrid,
            c->cport_id0,
            val0);
    if (rc) {
        return rc;
    }

    rc = switch_set_port_l4attr(sw,
            c->port_id1,
            attrid,
            c->cport_id1,
            val1);
    if (rc) {
        return rc;
    }

    return 0;
}

int switch_cport_connect(struct tsb_switch *sw, uint8_t port_id,
                         uint16_t cport_id)
{
    return switch_set_port_l4attr(sw, port_id, T_CONNECTIONSTATE, cport_id, 1);
}

static int switch_cport_connect_prepare(struct tsb_switch *sw,
                                        struct unipro_connection *c) {
    int e2efc_enabled = (!!(c->flags & CPORT_FLAGS_E2EFC) == 1);
    int csd_enabled = (!!(c->flags & CPORT_FLAGS_CSD_N) == 0);
    int rc = 0;

    /* Disable any existing connection(s). */
    rc = switch_set_pair_attr(sw, c, T_CONNECTIONSTATE, 0, 0);
    if (rc) {
        return rc;
    }

    /*
     * Point each device at the other.
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_PEERDEVICEID,
                              c->device_id1,
                              c->device_id0);
    if (rc) {
        return rc;
    }

    /*
     * Point each CPort at the other.
     */
    rc = switch_set_pair_attr(sw, c, T_PEERCPORTID, c->cport_id1, c->cport_id0);
    if (rc) {
        return rc;
    }

    /*
     * Match up traffic classes.
     */
    rc = switch_set_pair_attr(sw, c, T_TRAFFICCLASS, c->tc, c->tc);
    if (rc) {
        return rc;
    }

    /*
     * Make sure the protocol IDs are equal. (We don't use them otherwise.)
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_PROTOCOLID,
                              CPORT_DEFAULT_T_PROTOCOLID,
                              CPORT_DEFAULT_T_PROTOCOLID);
    if (rc) {
        return rc;
    }

    /*
     * Set default TxTokenValue and RxTokenValue values.
     *
     * IMPORTANT: TX and RX token values must be equal if E2EFC is
     * enabled, so don't change them to different values unless you
     * also patch up the E2EFC case, below.
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_TXTOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE);
    if (rc) {
        return rc;
    }

    rc = switch_set_pair_attr(sw,
                              c,
                              T_RXTOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE,
                              CPORT_DEFAULT_TOKENVALUE);
    if (rc) {
        return rc;
    }


    /*
     * Set CPort flags.
     *
     * (E2EFC needs to be the same on both sides, which is handled by
     * having a single flags value for now.)
     */
    rc = switch_set_pair_attr(sw, c, T_CPORTFLAGS, c->flags, c->flags);
    if (rc) {
        return rc;
    }

    /*
     * If E2EFC is enabled, or E2EFC is disabled and CSD is enabled,
     * then each CPort's T_PeerBufferSpace must equal the peer CPort's
     * T_LocalBufferSpace.
     */
    if (e2efc_enabled || (!e2efc_enabled && csd_enabled)) {
        uint32_t cport0_local = 0;
        uint32_t cport1_local = 0;
        rc = switch_get_port_l4attr(sw,
                c->port_id0,
                T_LOCALBUFFERSPACE,
                c->cport_id0,
                &cport0_local);
        if (rc) {
            return rc;
        }

        rc = switch_get_port_l4attr(sw,
                c->port_id1,
                T_LOCALBUFFERSPACE,
                c->cport_id1,
                &cport1_local);
        if (rc) {
            return rc;
        }

        rc = switch_set_pair_attr(sw,
                                  c,
                                  T_LOCALBUFFERSPACE,
                                  cport0_local,
                                  cport1_local);
        if (rc) {
            return rc;
        }
    }

    /*
     * Ensure the CPorts aren't in test mode.
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              T_CPORTMODE,
                              CPORT_MODE_APPLICATION,
                              CPORT_MODE_APPLICATION);
    if (rc) {
        return rc;
    }

    /*
     * Clear out the credits to send on each side.
     */
    rc = switch_set_pair_attr(sw, c, T_CREDITSTOSEND, 0, 0);
    if (rc) {
        return rc;
    }

    /*
     * XXX Toshiba-specific TSB_MaxSegmentConfig (move to bridge ASIC code.)
     */
    rc = switch_set_pair_attr(sw,
                              c,
                              TSB_MAXSEGMENTCONFIG,
                              CPORT_DEFAULT_TSB_MAXSEGMENTCONFIG,
                              CPORT_DEFAULT_TSB_MAXSEGMENTCONFIG);
    if (rc) {
        return rc;
    }

    return rc;
}

static int switch_cport_disconnect(struct tsb_switch *sw,
                                   uint8_t port_id0,
                                   uint8_t cport_id0,
                                   uint8_t port_id1,
                                   uint8_t cport_id1) {
    int rc0, rc1;
    rc0 = switch_dme_peer_set(sw, port_id0, T_CONNECTIONSTATE,
                             cport_id0, 0x0);
    rc1 = switch_dme_peer_set(sw, port_id1, T_CONNECTIONSTATE,
                              cport_id1, 0x0);
    return rc0 || rc1;
}

/**
 * @brief Retrieve a device id for a given port id
 */
int switch_if_dev_id_get(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t *dev_id) {
    int dev;

    if (!dev_id) {
        return -EINVAL;
    }

    dev = dev_ids_port_to_dev(sw, port_id);
    *dev_id = dev;

    if (dev == INVALID_PORT) {
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Assign a device id to a given port id
 */
int switch_if_dev_id_set(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t dev_id) {
    int rc;

    if (port_id >= SWITCH_UNIPORT_MAX) {
        return -EINVAL;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID,
                             UNIPRO_SELINDEX_NULL, dev_id);
    if (rc) {
        return rc;
    }

    rc = switch_dme_peer_set(sw, port_id, N_DEVICEID_VALID,
                             UNIPRO_SELINDEX_NULL, 1);
    if (rc) {
        /* do what on failure? */
        return rc;
    }

    /* update the table */
    dev_ids_update(sw, port_id, dev_id);

    return 0;
}

/**
 * @brief Setup network routing table
 *
 * Setup of the deviceID Mask tables (if supported) and the Switch LUTs,
 * bidirectionally between the source and destination Switch ports.
 */
int switch_setup_routing_table(struct tsb_switch *sw,
                               uint8_t device_id_0,
                               uint8_t port_id_0,
                               uint8_t device_id_1,
                               uint8_t port_id_1) {

    int rc;

    dbg_verbose("Setup routing table [p=%u:d=%u]<->[p=%u:d=%u]\n",
                device_id_0, port_id_0, device_id_1, port_id_1);

    // Set valid device bitmask 0->1
    rc = switch_set_valid_device(sw, port_id_0, device_id_1, true);
    if (rc) {
        return rc;
    }

    // Set valid device bitmask 1->0
    rc = switch_set_valid_device(sw, port_id_1, device_id_0, true);
    if (rc) {
        return rc;
    }

    // Setup routing table for devices 0->1
    rc = switch_lut_set(sw, port_id_0, device_id_1, port_id_1);
    if (rc) {
        dbg_error("Failed to set Lut for source port %u, disabling\n",
                  port_id_0);
        /* Undo deviceid_valid on failure */
        switch_dme_peer_set(sw, port_id_0, N_DEVICEID_VALID,
                            UNIPRO_SELINDEX_NULL, 0);
        return rc;
    }

    // Setup routing table for devices 1->0
    rc = switch_lut_set(sw, port_id_1, device_id_0, port_id_0);
    if (rc) {
        dbg_error("Failed to set Lut for source port %u, disabling\n",
                  port_id_1);
        /* Undo deviceid_valid on failure */
        switch_dme_peer_set(sw, port_id_1, N_DEVICEID_VALID,
                            UNIPRO_SELINDEX_NULL, 0);
        return rc;
    }

    return 0;
}

/**
 * @brief Invalidate network routing table
 *
 * Invalidate the deviceID Mask tables (if supported) and the N_DEVICEID_VALID
 * attribute, bidirectionally between the source and destination Switch ports.
 */
int switch_invalidate_routing_table(struct tsb_switch *sw,
                                    uint8_t device_id_0,
                                    uint8_t port_id_0,
                                    uint8_t device_id_1,
                                    uint8_t port_id_1) {

    int rc;

    dbg_verbose("Invalidate routing table [p=%u:d=%u]<->[p=%u:d=%u]\n",
                device_id_0, port_id_0, device_id_1, port_id_1);

    // Set invalid device bitmask 0->1
    rc = switch_set_valid_device(sw, port_id_0, device_id_1, false);
    if (rc) {
        return rc;
    }

    // Set invalid device bitmask 1->0
    rc = switch_set_valid_device(sw, port_id_1, device_id_0, false);
    if (rc) {
        return rc;
    }

    /* Undo deviceid_valid attribute for device 0's port */
    switch_dme_peer_set(sw, port_id_0, N_DEVICEID_VALID,
                        UNIPRO_SELINDEX_NULL, 0);

    /* Undo deviceid_valid attribute for device 1's port */
    switch_dme_peer_set(sw, port_id_1, N_DEVICEID_VALID,
                        UNIPRO_SELINDEX_NULL, 0);

    return 0;
}

/**
 * @brief Create a connection between two cports
 */
int switch_connection_create(struct tsb_switch *sw,
                             struct unipro_connection *c) {
    int rc;

    if (!c) {
        rc = -EINVAL;
        goto err0;
    }

    dbg_info("Creating connection: "
             "[p=%u,d=%u,c=%u]<->[p=%u,d=%u,c=%u] "
             "TC: %u Flags: 0x%x\n",
             c->port_id0,
             c->device_id0,
             c->cport_id0,
             c->port_id1,
             c->device_id1,
             c->cport_id1,
             c->tc,
             c->flags);

    rc = switch_cport_connect_prepare(sw, c);
    if (rc) {
        switch_cport_disconnect(sw,
                                c->port_id0,
                                c->cport_id0,
                                c->port_id1,
                                c->cport_id1);
        dbg_error("%s: couldn't create connection: %d\n", __func__, rc);
        goto err0;
    }

    return 0;

err0:
    dbg_error("%s: Connection setup failed. "
              "[p=%u,d=%u,c=%u]<->[p=%u,d=%u,c=%u] "
              "TC: %u Flags: %x rc: %d\n",
               __func__,
               c->port_id0,
               c->device_id0,
               c->cport_id0,
               c->port_id1,
               c->device_id1,
               c->cport_id1,
               c->tc,
               c->flags,
               rc);
    return rc;
}

/**
 * @brief Create a connection between two cports
 */
int switch_connection_destroy(struct tsb_switch *sw,
                              struct unipro_connection *c)
{
    int retval;

    if (!c) {
        return -EINVAL;
    }

    dbg_info("Destroying connection: [p=%hhu,d=%hhu,c=%hu]<->[p=%hhu,d=%hhu,c=%hu]\n",
             c->port_id0, c->device_id0, c->cport_id0,
             c->port_id1, c->device_id1, c->cport_id1);

    retval = switch_cport_disconnect(sw, c->port_id0, c->cport_id0,
                                     c->port_id1, c->cport_id1);
    if (retval) {
        dbg_error("%s: couldn't destroy connection: %d\n", __func__, retval);
        return retval;
    }

    return 0;
}

/*
 * Sanity check an HS or PWM configuration.
 */
static int switch_active_cfg_is_sane(const struct unipro_pwr_cfg *pcfg,
                                     unsigned max_nlanes) {
    if (pcfg->upro_mode == UNIPRO_MODE_UNCHANGED) {
        /* Unchanged is always sane. */
        return 1;
    }

    if (pcfg->upro_nlanes < 1 || pcfg->upro_nlanes > max_nlanes) {
        dbg_error("%s(): attempt to use %u lanes, support at most %u\n",
                  __func__, pcfg->upro_nlanes, max_nlanes);
        return 0;
    }
    switch (pcfg->upro_mode) {
    case UNIPRO_FAST_MODE:      /* fall through */
    case UNIPRO_FASTAUTO_MODE:
        if (pcfg->upro_gear == 0 || pcfg->upro_gear > 3) {
            dbg_error("%s(): invalid HS gear %u\n", __func__, pcfg->upro_gear);
            return 0;
        }
        break;
    case UNIPRO_SLOW_MODE:      /* fall through */
    case UNIPRO_SLOWAUTO_MODE:
        if (pcfg->upro_gear == 0 || pcfg->upro_gear > 7) {
            dbg_error("%s(): invalid PWM gear %u\n", __func__, pcfg->upro_gear);
            return 0;
        }
        break;
    default:
        dbg_error("%s(): unexpected mode %u\n", __func__, pcfg->upro_mode);
        return 0;
    }
    return 1;
}

static int switch_configure_link_tx(struct tsb_switch *sw,
                                    uint8_t port_id,
                                    const struct unipro_pwr_cfg *tx,
                                    uint32_t tx_term) {
    int rc = 0;
    /* If it needs changing, apply the TX side of the new link
     * configuration. */
    if (tx->upro_mode != UNIPRO_MODE_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_TXGEAR,
                            UNIPRO_SELINDEX_NULL, tx->upro_gear) ||
            switch_dme_set(sw, port_id, PA_TXTERMINATION,
                           UNIPRO_SELINDEX_NULL, tx_term) ||
            switch_dme_set(sw, port_id, PA_ACTIVETXDATALANES,
                           UNIPRO_SELINDEX_NULL, tx->upro_nlanes);
    }
    return rc;
}

static int switch_configure_link_rx(struct tsb_switch *sw,
                                    uint8_t port_id,
                                    const struct unipro_pwr_cfg *rx,
                                    uint32_t rx_term) {
    int rc = 0;
    /* If it needs changing, apply the RX side of the new link
     * configuration.
     */
    if (rx->upro_mode != UNIPRO_MODE_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_RXGEAR,
                            UNIPRO_SELINDEX_NULL, rx->upro_gear) ||
            switch_dme_set(sw, port_id, PA_RXTERMINATION,
                           UNIPRO_SELINDEX_NULL, rx_term) ||
            switch_dme_set(sw, port_id, PA_ACTIVERXDATALANES,
                           UNIPRO_SELINDEX_NULL, rx->upro_nlanes);
    }
    return rc;
}

/**
 * @brief UniPro link L2 User Data configuration routine.
 *
 * This function supports configuration of the UniPro link
 * L2 timeout values, FCx Protection, AFCx Replay and AFCx Request.
 *
 * Note: The Toshiba UniPro stack updates all L2 Timeout Values
 *       during a UniPro Power Mode change regardless of whether
 *       or not the caller updated any L2 User Data (flags = 0).
 *       Therefore, the PA_PWRMODEUSERDATAn Attributes must be
 *       updated with some values to avoid inadvertently
 *       disabling the L2 Timers (PA_PWRMODEUSERDATAn = 0).
 *
 * Note: The L2 timeout values are written to DME Attributes that
 *       are updated on the next PA_PWRMode update.
 *
 * It is not known if this function is needed, so it is included for completeness.
 *
 * @param sw Switch handle
 * @param port_id Port whose link to reconfigure
 * @param udata UniPro L2 timeout values to apply
 */
static int switch_configure_link_user_data
        (struct tsb_switch *sw,
         uint8_t port_id,
         const struct unipro_pwr_user_data *udata) {
    int rc = 0;
    const uint32_t flags = udata->flags;
    uint32_t val;

    if (flags & UPRO_PWRF_FC0) {    // Update FC0 Protection timeout value
        val = udata->upro_pwr_fc0_protection_timeout;
    } else {    // Use existing FC0 Protection timeout value
        rc = switch_dme_get(sw,
                            port_id,
                            DME_FC0PROTECTIONTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            &val);
        if (rc) {
            return rc;
        }
    }
    rc = switch_dme_set(sw,
                        port_id,
                        PA_PWRMODEUSERDATA0,
                        UNIPRO_SELINDEX_NULL,
                        val);
    if (rc) {
        return rc;
    }
    if (flags & UPRO_PWRF_TC0) {    // Update TC0 Replay timeout value
        val = udata->upro_pwr_tc0_replay_timeout;
    } else {    // Use existing TC0 Replay timeout value
        rc = switch_dme_get(sw,
                            port_id,
                            DME_TC0REPLAYTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            &val);
        if (rc) {
            return rc;
        }
    }
    rc = switch_dme_set(sw,
                        port_id,
                        PA_PWRMODEUSERDATA1,
                        UNIPRO_SELINDEX_NULL,
                        val);
    if (rc) {
        return rc;
    }
    if (flags & UPRO_PWRF_AFC0) {    // Update AFC0 Request timeout value
        val = udata->upro_pwr_afc0_req_timeout;
    } else {    // Use existing AFC0 Request timeout value
        rc = switch_dme_get(sw,
                            port_id,
                            DME_AFC0REQTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            &val);
        if (rc) {
            return rc;
        }
    }
    rc = switch_dme_set(sw,
                        port_id,
                        PA_PWRMODEUSERDATA2,
                        UNIPRO_SELINDEX_NULL,
                        val);
    if (rc) {
        return rc;
    }
    if (flags & UPRO_PWRF_FC1) {    // Update FC1 Protection timeout value
        val = udata->upro_pwr_fc1_protection_timeout;
    } else {    // Use existing FC1 Protection timeout value
        rc = switch_dme_get(sw,
                            port_id,
                            DME_FC1PROTECTIONTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            &val);
        if (rc) {
            return rc;
        }
    }
    rc = switch_dme_set(sw,
                        port_id,
                        PA_PWRMODEUSERDATA3,
                        UNIPRO_SELINDEX_NULL,
                        val);
    if (rc) {
        return rc;
    }
    if (flags & UPRO_PWRF_TC1) {    // Update TC1 Replay timeout value
        val = udata->upro_pwr_tc1_replay_timeout;
    } else {    // Use existing TC1 Replay timeout value
        rc = switch_dme_get(sw,
                            port_id,
                            DME_TC1REPLAYTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            &val);
        if (rc) {
            return rc;
        }
    }
    rc = switch_dme_set(sw,
                        port_id,
                        PA_PWRMODEUSERDATA4,
                        UNIPRO_SELINDEX_NULL,
                        val);
    if (rc) {
        return rc;
    }
    if (flags & UPRO_PWRF_AFC1) {    // Update AFC1 Request timeout value
        val = udata->upro_pwr_afc1_req_timeout;
    } else {    // Use existing AFC1 Request timeout value
        rc = switch_dme_get(sw,
                            port_id,
                            DME_AFC1REQTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            &val);
        if (rc) {
            return rc;
        }
    }
    rc = switch_dme_set(sw,
                        port_id,
                        PA_PWRMODEUSERDATA5,
                        UNIPRO_SELINDEX_NULL,
                        val);
    return rc;
}

static int switch_configure_link_tsbdata
        (struct tsb_switch *sw,
         uint8_t port_id,
         const struct tsb_local_l2_timer_cfg *tcfg) {
    int rc = 0;
    const unsigned int flags = tcfg->tsb_flags;
    if (flags & TSB_LOCALL2F_FC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_FC0PROTECTIONTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_fc0_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_TC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_TC0REPLAYTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_tc0_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_AFC0) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_AFC0REQTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_afc0_req_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_FC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_FC1PROTECTIONTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_fc1_protection_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_TC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_TC1REPLAYTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_tc1_replay_timeout);
        if (rc) {
            return rc;
        }
    }
    if (flags & TSB_LOCALL2F_AFC1) {
        rc = switch_dme_set(sw,
                            port_id,
                            DME_AFC1REQTIMEOUTVAL,
                            UNIPRO_SELINDEX_NULL,
                            tcfg->tsb_afc1_req_timeout);
    }
    return rc;
}

static int switch_apply_power_mode(struct tsb_switch *sw,
                                   uint8_t port_id,
                                   uint32_t pwr_mode) {
    const int max_tries = 100;
    int rc;
    uint32_t val;
    int i;
    struct interface *iface;

    dbg_insane("%s(): enter, port=%u, pwr_mode=0x%x\n", __func__, port_id,
               pwr_mode);

    iface = interface_get_by_portid(port_id);
    if (!iface) {
        dbg_error("%s: No interface for portId %u, aborting\n",
                  __func__, port_id);
        return -ENODEV;
    }
    val = atomic_get(&iface->dme_powermodeind);
    dbg_verbose("%s(): previous TSB_DME_POWERMODEIND=0x%x\n",
                __func__, val);

    /* Trigger power mode change */
    rc = switch_dme_set(sw, port_id, PA_PWRMODE, UNIPRO_SELINDEX_NULL,
                        pwr_mode);
    if (rc) {
        dbg_error("%s(): can't set PA_PWRMODE (0x%x) to 0x%x: %d\n",
                  __func__, PA_PWRMODE, pwr_mode, rc);
        if (rc == UNIPRO_CONFIGRESULT_BUSY) {
            /*
             * This is a healthy margin above what the spec says we
             * should give the link to recover from a previously
             * unsuccessful change.
             */
            dbg_warn("%s(): waiting 200 ms for link recovery\n", __func__);
            usleep(200 * 1000);
        }
        goto out;
    }

    /* Read power mode change result */
    for (i = 0; i < max_tries; i++) {
        /*
         * Wait until the power mode change completes.
         *
         * FIXME other error handling (UniPro specification 5.7.12.5).
         */
        val = atomic_get(&iface->dme_powermodeind);
        switch (val) {
        case TSB_DME_POWERMODEIND_NONE:
            /* This happens sometimes, and seems harmless. */
            break;
        case TSB_DME_POWERMODEIND_OK:
            dbg_warn("%s: TSB_DME_POWERMODEIND=0x%x (can't happen!)\n",
                     __func__, val);
            break;
        case TSB_DME_POWERMODEIND_LOCAL:
            /* ... and done */
            break;
        case TSB_DME_POWERMODEIND_REMOTE:
            dbg_info("%s: TSB_DME_POWERMODEIND=0x%x (remote)\n",
                     __func__, val);
            break;
        case TSB_DME_POWERMODEIND_BUSY:
            dbg_warn("%s: TSB_DME_POWERMODEIND=0x%x (busy)\n",
                     __func__, val);
            rc = val;
            goto out;
        case TSB_DME_POWERMODEIND_CAP_ERR:
            dbg_error("%s: TSB_DME_POWERMODEIND=0x%x (capability err)\n",
                      __func__, val);
            rc = val;
            goto out;
        case TSB_DME_POWERMODEIND_FATAL_ERR:
            dbg_error("%s: TSB_DME_POWERMODEIND=0x%x (fatal error)\n",
                      __func__, val);
            rc = val;
            goto out;
        default:
            dbg_error("%s: TSB_DME_POWERMODEIND=0x%x (invalid value)\n",
                      __func__, val);
            rc = val;
            goto out;

        }
        if (val == TSB_DME_POWERMODEIND_LOCAL) {
            break;
        }
        /* Wait a little bit for the power mode to complete */
        usleep(500);
    }
    if (val != TSB_DME_POWERMODEIND_LOCAL) {
        rc = -ETIMEDOUT;
        goto out;
    }

    /*
     * Read a random DME attribute (in this case, the connection state
     * of CPort 0) to see if the link is actually working after the
     * power mode change, or if it apparently succeeded but actually
     * went off and died.
     */
    dbg_insane("%s(): testing link state with peer DME access\n", __func__);
    rc = switch_dme_peer_get(sw,
                             port_id,
                             T_CONNECTIONSTATE,
                             0,
                             &val);

 out:
    dbg_insane("%s(): exit, rc=%d\n", __func__, rc);
    return rc;
}

/**
 * @brief Low-level UniPro link configuration routine.
 *
 * This supports separate reconfiguration of each direction of the
 * UniPro link to different power modes, and allows for hibernation
 * and off modes.
 *
 * Higher level convenience functions to set both directions of a link
 * to the same HS or PWM gear are available.
 *
 * @param sw Switch handle
 * @param port_id Port whose link to reconfigure
 * @param cfg UniPro power configuration to apply
 * @param tcg Toshiba extensions to UniPro power config to apply,
 *            This may be NULL if no extensions are needed.
 *
 * @see switch_configure_link_hs()
 * @see switch_configure_link_pwm()
 */
int switch_configure_link(struct tsb_switch *sw,
                          uint8_t port_id,
                          const struct unipro_link_cfg *cfg,
                          const struct tsb_link_cfg *tcfg) {
    int rc = 0;
    const struct unipro_pwr_cfg *tx = &cfg->upro_tx_cfg;
    uint32_t tx_term = !!(cfg->flags & UPRO_LINKF_TX_TERMINATION);
    const struct unipro_pwr_cfg *rx = &cfg->upro_rx_cfg;
    uint32_t rx_term = !!(cfg->flags & UPRO_LINKF_RX_TERMINATION);
    uint32_t scrambling = !!(cfg->flags & UPRO_LINKF_SCRAMBLING);
    const struct unipro_pwr_user_data *udata = &cfg->upro_user;
    uint32_t pwr_mode = tx->upro_mode | (rx->upro_mode << 4);

    dbg_verbose("%s(): port=%d\n", __func__, port_id);

    /* FIXME ADD JIRA support hibernation and link off modes. */
    if (tx->upro_mode == UNIPRO_HIBERNATE_MODE ||
        tx->upro_mode == UNIPRO_OFF_MODE ||
        rx->upro_mode == UNIPRO_HIBERNATE_MODE ||
        rx->upro_mode == UNIPRO_OFF_MODE) {
        rc = -EOPNOTSUPP;
        goto out;
    }

    /* Sanity-check the configuration. */
    if (!(switch_active_cfg_is_sane(tx, PA_CONN_TX_DATA_LANES_NR) &&
          switch_active_cfg_is_sane(rx, PA_CONN_RX_DATA_LANES_NR))) {
        rc = -EINVAL;
        goto out;
    }

    /* Changes to a link's HS series require special preparation, and
     * involve restrictions on the power mode to apply next.
     *
     * Handle that properly before setting PA_HSSERIES. */
    if (cfg->upro_hs_ser != UNIPRO_HS_SERIES_UNCHANGED) {
        rc = switch_dme_set(sw, port_id, PA_HSSERIES, UNIPRO_SELINDEX_NULL,
                            cfg->upro_hs_ser);
        if (rc) {
            dbg_error("%s(): can't change PA_HSSeries: %d\n",
                      __func__, rc);
            goto out;
        }
    }

    /* Apply TX and RX link reconfiguration as needed. */
    rc = switch_configure_link_tx(sw, port_id, tx, tx_term) ||
        switch_configure_link_rx(sw, port_id, rx, rx_term);
    if (rc) {
        goto out;
    }

    /* Handle scrambling. */
    rc = switch_dme_set(sw, port_id, PA_SCRAMBLING,
                        UNIPRO_SELINDEX_NULL, scrambling);
    if (rc) {
        goto out;
    }

    /* Set any DME user data we understand. */
    rc = switch_configure_link_user_data(sw, port_id, udata);
    if (rc) {
        goto out;
    }

    /* Handle Toshiba extensions to the link configuration procedure. */
    if (tcfg) {
        rc = switch_configure_link_tsbdata(sw, port_id, &tcfg->tsb_l2tim_cfg);
    }
    if (rc) {
        goto out;
    }

    /* Kick off the actual power mode change, and see what happens. */
    rc = switch_apply_power_mode(sw, port_id, pwr_mode);
 out:
    dbg_insane("%s(): exit, rc=%d\n", __func__, rc);
    return rc;
}

/*
 * send data down the SVC connection cport
 */
int switch_data_send(struct tsb_switch *sw, void *data, size_t len) {
    if (!sw->ops->switch_data_send) {
        return -EOPNOTSUPP;
    }
    return sw->ops->switch_data_send(sw, data, len);
}

/**
 * @brief Start the transmission of FCTs on CPort 4.
 *        Other CPorts not tested/unsupported.
 */
int switch_fct_enable(struct tsb_switch *sw) {
    uint32_t spictlb = 0xC;
    int rc;

    rc = switch_internal_setattr(sw, SPICTLB, spictlb);

    if (rc) {
        dbg_error("Failed to set spictlb\n");
    }

    return rc;
}

/* IRQ worker creation */
static int create_switch_irq_worker(struct tsb_switch *sw)
{
    const char* argv[2];
    char buf[16];
    int ret;

    sprintf(buf, "%p", sw);
    argv[0] = buf;
    argv[1] = NULL;

    ret = task_create("switch_irq_worker",
                      IRQ_WORKER_DEFPRIO, IRQ_WORKER_STACKSIZE,
                      _switch_irq_pending_worker,
                      (char * const*) argv);
    if (ret == ERROR) {
        dbg_error("%s: Failed to create IRQ worker\n", __func__);
        return ERROR;
    }

    sw->worker_id = ret;

    return 0;
}

/* IRQ worker destruction */
static int destroy_switch_irq_worker(struct tsb_switch *sw)
{
    int ret = 0, status;

    sw->sw_irq_worker_exit = true;

    if (!sw->worker_id) {
        return ret;
    }

    sem_post(&sw->sw_irq_lock);

    ret = waitpid(sw->worker_id, &status, 0);
    if (ret < 0) {
        dbg_warn("%s: waitpid failed with ret=%d\n", __func__, ret);
    } else {
        sw->worker_id = 0;
    }

    return ret;
}

/*
 * Required callbacks for NuttX SPI. unused.
 */
void stm32_spi1select(struct spi_dev_s *dev, enum spi_dev_e devid,
                      bool selected) {
}

uint8_t stm32_spi1status(struct spi_dev_s *dev,
                         enum spi_dev_e devid) {
    return SPI_STATUS_PRESENT;
}

void stm32_spi2select(struct spi_dev_s *dev,
                      enum spi_dev_e devid,
                      bool selected) {
}

uint8_t stm32_spi2status(struct spi_dev_s *dev,
                         enum spi_dev_e devid) {
    return SPI_STATUS_PRESENT;
}

/*
 * Platform-specific post-initialization, to be performed after SPI
 * link with the switch is up.
 */
static int switch_post_init_comm(struct tsb_switch *sw)
{
    if (!sw->ops->__post_init_seq) {
        return -EOPNOTSUPP;
    }
    return sw->ops->__post_init_seq(sw);
}

/**
 * @brief Initialize the switch and set default SVC<->Switch route
 * @param sw pdata platform-specific data for this switch
 * @return pointer to initialized switch instance on success, NULL on error
 */
struct tsb_switch *switch_init(struct tsb_switch_data *pdata) {
    struct tsb_switch *sw ;
    struct spi_dev_s *spi_dev;
    unsigned int attr_value;
    int rc;

    dbg_verbose("%s: Initializing switch\n", __func__);

    if (!pdata) {
        return NULL;
    }

    sw = zalloc(sizeof(struct tsb_switch));
    if (!sw) {
        return NULL;
    }

    sw->pdata = pdata;
    sem_init(&sw->sw_irq_lock, 0, 0);
    sw->worker_id = 0;
    sw->sw_irq_worker_exit = false;

    list_init(&sw->listeners);

    switch_power_on_reset(sw);

    stm32_configgpio(sw->pdata->spi_cs);
    stm32_gpiowrite(sw->pdata->spi_cs, true);

    spi_dev = up_spiinitialize(sw->pdata->bus);
    if (!spi_dev) {
        dbg_error("%s: Failed to initialize spi device\n", __func__);
        rc = -ENODEV;
        goto error;
    }
    sw->spi_dev = spi_dev;

    switch (sw->pdata->rev) {
    case SWITCH_REV_ES2:
        if (tsb_switch_es2_init(sw, spi_dev)) {
            goto error;
        }
        break;
    case SWITCH_REV_ES3:
        if (tsb_switch_es3_init(sw, spi_dev)) {
            goto error;
        }
        break;
    default:
        dbg_error("Unsupported switch revision: %u\n", sw->pdata->rev);
        goto error;
    }

    /* Configure the SPI1 bus in Mode0, 8bits, 13MHz clock */
    SPI_SETMODE(spi_dev, SPIDEV_MODE0);
    SPI_SETBITS(spi_dev, 8);
    SPI_SETFREQUENCY(spi_dev, SWITCH_SPI_FREQUENCY);

    rc = switch_init_comm(sw);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error;
    }
    rc = switch_post_init_comm(sw);
    if (rc && (rc != -EOPNOTSUPP)) {
        goto error;
    }

    /*
     * Sanity check
     */
    if (switch_internal_getattr(sw, SWVER, &attr_value)) {
        dbg_error("Switch probe failed\n");
        goto error;
    }

    /*
     * Now that the switch is on, let's give it some time to initialize
     * the Unipro busses before the bridges connect to it
     */
    switch (sw->pdata->rev) {
    case SWITCH_REV_ES2:
        /* Wait 360ms as per Toshiba recomandation */
        usleep(360 * 1000);
        break;
    default:
        break;
    }

    // Init port <-> deviceID mapping table
    dev_ids_destroy(sw);
    dev_ids_update(sw, SWITCH_PORT_ID, SWITCH_DEVICE_ID);

    /*
     * Set initial SVC deviceId to SWITCH_DEVICE_ID and setup
     * routes for internal L4 access from the SVC
     */
    rc = _switch_internal_set_id(sw,
                                 L4_CPORT_SVC_TO_CC,
                                 L4_PEERCPORT_SVC_TO_CC,
                                 CPORT_ENABLE,
                                 IRT_ENABLE);
    if (rc) {
        goto error;
    }
    rc = _switch_internal_set_id(sw,
                                 L4_CPORT_CC_TO_SVC,
                                 L4_PEERCPORT_CC_TO_SVC,
                                 CPORT_ENABLE,
                                 IRT_DISABLE);
    if (rc) {
        goto error;
    }

    // Create Switch interrupt handling worker
    rc = create_switch_irq_worker(sw);
    if (rc) {
        dbg_error("%s: Failed to create Switch IRQ worker\n", __func__);
        goto error;
    }

    return sw;

error:
    switch_exit(sw);
    dbg_error("%s: Failed to initialize switch.\n", __func__);

    return NULL;
}


/**
 * @brief Power down and disable the switch
 */
void switch_exit(struct tsb_switch *sw)
{
    dbg_verbose("%s: Disabling switch\n", __func__);

    switch_irq_enable(sw, false);
    destroy_switch_irq_worker(sw);
    dev_ids_destroy(sw);

    switch (sw->pdata->rev) {
        break;
    case SWITCH_REV_ES2:
        tsb_switch_es2_exit(sw);
        break;
    case SWITCH_REV_ES3:
        tsb_switch_es3_exit(sw);
        break;
    default:
        dbg_error("Unsupported switch revision: %u\n", sw->pdata->rev);
        break;
    }

    switch_power_off(sw);
    free(sw);
}

int switch_event_register_listener(struct tsb_switch *sw,
                                   struct tsb_switch_event_listener *l) {
    if (!sw || !sw) {
        return -EINVAL;
    }

    list_init(&l->entry);
    list_add(&sw->listeners, &l->entry);
    return 0;
}
