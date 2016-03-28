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

#ifndef  _TSB_SWITCH_H_
#define  _TSB_SWITCH_H_

#include <sched.h>

#include <nuttx/list.h>
#include <nuttx/spi/spi.h>
#include <nuttx/unipro/unipro.h>
#include <nuttx/greybus/tsb_unipro.h>

#include "interface.h"

/* SPI port to the Switch */
#define SW_SPI_PORT_1               (1)
#define SW_SPI_PORT_2               (2)

/* Switch internal attributes */
#define SWVER                       (0x0000)
#define SWPOR                       (0x0002)
#define SWSTA                       (0x0003)
#define SWINT                       (0x0010)
#define SWINE                       (0x0015)
#define SWINS                       (0x0016)
#define SWRES                       (0x0020)
#define SPICTLA                     (0x0040)
#define SPICTLB                     (0x0041)
#define SPIINTE                     (0x0042)
#define SPICEE                      (0x0043)
#define SPICES                      (0x0044)
#define SPI3EE                      (0x0045)
#define SPI3ES                      (0x0046)
#define SPI4EE                      (0x0047)
#define SPI4ES                      (0x0048)
#define SPI5EE                      (0x0049)
#define SPI5ES                      (0x004a)

/* ES3-specific switch internal attributes */
#define SWCCT                       (0x0022)
#define SWALS                       (0x0023)
#define SWLCE                       (0x0024)
#define SWLCS                       (0x0025)
#define SPITIMER                    (0x004c)

/* NCP commands */
#define NCP_SETREQ                  (0x00)
#define NCP_SETCNF                  (0x01)
#define NCP_PEERSETREQ              (0x02)
#define NCP_PEERSETCNF              (0x03)
#define NCP_GETREQ                  (0x04)
#define NCP_GETCNF                  (0x05)
#define NCP_PEERGETREQ              (0x06)
#define NCP_PEERGETCNF              (0x07)
#define NCP_LUTSETREQ               (0x10)
#define NCP_LUTSETCNF               (0x11)
#define NCP_LUTGETREQ               (0x12)
#define NCP_LUTGETCNF               (0x13)
#define NCP_SETDEVICEIDMASKREQ      (0x14)
#define NCP_DESTVALIDSETREQ         (0x14)
#define NCP_SETDEVICEIDMASKCNF      (0x15)
#define NCP_DESTVALIDSETCNF         (0x15)
#define NCP_GETDEVICEIDMASKREQ      (0x16)
#define NCP_DESTVALIDGETREQ         (0x16)
#define NCP_GETDEVICEIDMASKCNF      (0x17)
#define NCP_DESTVALIDGETCNF         (0x17)
#define NCP_SWITCHATTRSETREQ        (0x20)
#define NCP_SWITCHATTRSETCNF        (0x21)
#define NCP_SWITCHATTRGETREQ        (0x22)
#define NCP_SWITCHATTRGETCNF        (0x23)
#define NCP_SWITCHIDSETREQ          (0x24)
#define NCP_SWITCHIDSETCNF          (0x25)
#define NCP_SYSCTRLSETREQ           (0x30)
#define NCP_SYSCTRLSETCNF           (0x31)
#define NCP_SYSCTRLGETREQ           (0x32)
#define NCP_SYSCTRLGETCNF           (0x33)

#define NCP_QOSATTRSETREQ           (0x50)
#define NCP_QOSATTRSETCNF           (0x51)
#define NCP_QOSATTRGETREQ           (0x52)
#define NCP_QOSATTRGETCNF           (0x53)

/* NCP commands return codes */
#define NCP_RC_SUCCESS                      (0x00)
#define NCP_RC_INVALID_ATTRIBUTE            (0x01)
#define NCP_RC_INVALID_CPORTID              (0x01)
#define NCP_RC_TIMEOUT                      (0x01)
#define NCP_RC_INVALID_ATTRIBUTE_VALUE      (0x02)
#define NCP_RC_INVALID_SOURCE_PORTID        (0x02)
#define NCP_RC_READ_ONLY_ATTRIBUTE          (0x03)
#define NCP_RC_WRITE_ONLY_ATTRIBUTE         (0x04)
#define NCP_RC_BAD_INDEX                    (0x05)
#define NCP_RC_LOCKED_ATTRIBUTE             (0x06)
#define NCP_RC_BAD_TEST_FEATURE_INDEX       (0x07)
#define NCP_RC_PEER_COMMUNICATION_FAILURE   (0x08)
#define NCP_RC_BUSY                         (0x09)
#define NCP_RC_DME_FAILURE                  (0x0A)
#define NCP_RC_INVALID_PORTID               (0x20)
#define NCP_RC_CPORT_DISABLE_FAILURE        (0x21)
#define NCP_RC_INVALID_COMMAND_LENGTH       (0x22)
#define NCP_RC_DISABLED_TARGET              (0x23)
#define NCP_RC_TARGET_NO_RESPONSE           (0x24)
#define NCP_RC_INVALID_PEER_ACCESS          (0x25)

/* System registers, accessible via switch_sys_ctrl_set, switch_sys_ctrl_get */
#define SC_SOFTRESET                (0x0000)
#define SC_SOFTRESETRELEASE         (0x0100)
#define SC_CLOCKGATING              (0x0200)
#define SC_CLOCKENABLE              (0x0300)
#define SC_SYSTEMCONF               (0x04C0)
#define SC_ECCERROR                 (0x04C4)
#define SC_VID                      (0x0700)
#define SC_PID                      (0x0704)
#define SC_REVISION0                (0x0710)
#define SC_REVISION1                (0x0714)
#define SC_MPHY_TRIM0               (0x0720)
#define SC_MPHY_TRIM1               (0x0724)
#define SC_MPHY_TRIM2               (0x0728)
#define SC_MPHY_TRIM3               (0x072C)
#define SC_MPHY_EFUSE1              (0x0770)
#define SC_MPHY_EFUSE2              (0x0774)
#define SC_PINSHARE                 (0x0800)
#define SC_CHIPID0                  (0x0880)
#define SC_CHIPID1                  (0x0884)
#define SC_CHIPID2                  (0x0888)

/* ES3-specific system registers */
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
#define SC_RESET_CLK_UNIPROPORT(i)  (1 << i)
#define SC_RESET_CLK_NCP            (1 << 14)
#define SC_RESET_CLK_MPORT(i)       (1 << (i + 16))
#define SC_RESET_CLK_RT10B          (1 << 30)

/* NCP field values */
#define NCP_RESERVED                (0x00)

/* Unipro attributes values */
#define PA_CONN_TX_DATA_LANES_NR        (2)
#define PA_CONN_RX_DATA_LANES_NR        (2)
#define PA_ACTIVE_TX_DATA_LANES_NR      (2)
#define PA_ACTIVE_RX_DATA_LANES_NR      (2)
#define MAX_SEGMENT_CONFIG              (280)
#define PA_FASTMODE_RXTX                (0x11)
#define PA_FASTAUTOMODE_RXTX            (0x44)
#define PA_GEAR                         (0x1)
#define TSB_DME_POWERMODEIND_NONE       (0) // no new value since last read
#define TSB_DME_POWERMODEIND_OK         (1 << 0)
#define TSB_DME_POWERMODEIND_LOCAL      (1 << 1)
#define TSB_DME_POWERMODEIND_REMOTE     (1 << 2)
#define TSB_DME_POWERMODEIND_BUSY       (1 << 3)
#define TSB_DME_POWERMODEIND_CAP_ERR    (1 << 4)
#define TSB_DME_POWERMODEIND_FATAL_ERR  (1 << 5)

/* Number of external ports, excluding the internal switch port */
#define SWITCH_UNIPORT_MAX          (14)
#define SWITCH_PORT_ID              SWITCH_UNIPORT_MAX
/* Number of ports, including the internal switch port */
#define SWITCH_PORT_MAX             (SWITCH_UNIPORT_MAX + 1)
#define INVALID_PORT                (0xFF)

#define SWITCH_DEVICE_ID            (0)

/* Switch-internal traffic class and sub-class numberings */
#define SWITCH_TRAFFIC_CLASS_TC0            (0) /* TC0 */
#define SWITCH_TRAFFIC_CLASS_TC1            (1) /* TC1 */
#define SWITCH_TRAFFIC_CLASS_TC0BAND        (2) /* TC0BAND */
#define SWITCH_TRAFFIC_CLASS_TC0HIGH        (3) /* TC0HIGH */

/* QoS attributes, accessible via qos_attr_get/qos_attr_set */
#define AR_CTRL                     (0x00)
#define AR_STATUS_INTERRUPT         (0x0C)
#define AR_STATUS_CONNECT           (0x0D)
#define AR_STATUS_REQ0              (0x0E)
#define AR_STATUS_REQ1              (0x0F)
#define AR_WDT                      (0x10)
#define AR_PRIOLUT                  (0x20)
#define AR_DRR1CFG_PERIOD(i)        (0x30 + i)
#define AR_DRR1CFG_PERIOD_BROADCAST (0x3F)
#define AR_DRR1CFG_DEC(i)           (0x40 + i)
#define AR_DRR1CFG_DEC_BROADCAST    (0x4F)
#define AR_DRR1CFG_LIMIT(i)         (0x50 + i)
#define AR_DRR1CFG_LIMIT_BROADCAST  (0x5F)
#define AR_DRR1STAT_QUANTITY(i)     (0x60 + i)
#define AR_BSTAT_QUANTITY(i)        (0x70 + i)
#define AR_BSTAT_QUANTITYTX         (0x80)
#define AR_DRR2CFG_PERIOD(i)        (0x90 + i)
#define AR_DRR2CFG_PERIOD_BROADCAST (0x9F)
#define AR_DRR2CFG_DEC(i)           (0xA0 + i)
#define AR_DRR2CFG_DEC_BROADCAST    (0xAF)
#define AR_DRR2CFG_LIMIT(i)         (0xB0 + i)
#define AR_DRR2CFG_LIMIT_BROADCAST  (0xBF)
#define RT_CTRL                     (0xC0)
#define RT_STATUS                   (0xC3)
#define RT_WDT                      (0xC4)
#define RT_BCFGDEC0                 (0xD0)
#define RT_BCFGLIMIT0               (0xD1)
#define RT_BCFGPERIOD0              (0xD2)
#define RT_BCFGDEC1                 (0xE0)
#define RT_BCFGLIMIT1               (0xE1)
#define RT_BCFGPERIOD1              (0xE2)

/* QoS attribute bitmasks */
#define AR_CTRL_CLOSURE                     (1 << 0)
#define AR_CTRL_DRR2ENABLE                  (1 << 8)
#define AR_PRIOLUT_PCP_MASK(i, b)           (b << (i * 2))
#define AR_PRIOLUT_PCP(i)                   AR_PRIOLUT_PCP_MASK(i, 0x3)
#define AR_STATUS_INTERRUPT_WDT_V           (1 << 0)
#define AR_STATUS_INTERRUPT_WDT_A           (1 << 1)
#define AR_STATUS_INTERRUPT_PRESENT_TC0     (1 << 2)
#define AR_STATUS_INTERRUPT_PRESENT_TC1     (1 << 3)
#define AR_STATUS_CONNECT_PORT(i)           (1 << i)
#define AR_STATUS_CONNECT_CLASS_TC0         (1 << 16)
#define AR_STATUS_CONNECT_CLASS_TC0HIGH     (1 << 17)
#define AR_STATUS_CONNECT_CLASS_TC0BAND     (1 << 18)
#define AR_STATUS_CONNECT_CLASS_TC1         (1 << 19)
#define AR_STATUS_CONNECT_ARB_OUTPUT        (1 << 24)
#define AR_STATUS_CONNECT_ARB_BUSY          (1 << 25)
#define AR_STATUS_REQ0_REQ_TC0(i)           (1 << i)
#define AR_STATUS_REQ0_REQ_TC0HIGH(i)       (1 << (16 + i))
#define AR_STATUS_REQ1_REQ_TC0BAND(i)       (1 << i)
#define AR_STATUS_REQ1_REQ_TC1(i)           (1 << (16 + i))
#define AR_WDT_WDT_COUNT                    ((1 << 24) - 1)
#define AR_BSTAT_QUANTITY_RATE00_TC0        ((1 << 16) - 1)
#define AR_BSTAT_QUANTITY_RATE00_TC1        (AR_BSTAT_QUANTITY_RATE00_TC0 << 16)
#define RT_CTRL_RST_TC0                     (1 << 0)
#define RT_CTRL_RST_TC1                     (1 << 1)
#define RT_STATUS_WDT_V_TC0                 (1 << 0)
#define RT_STATUS_WDT_A_TC0                 (1 << 1)
#define RT_STATUS_WDT_V_TC1                 (1 << 4)
#define RT_STATUS_WDT_A_TC1                 (1 << 5)
#define RT_WDT_WDT_COUNT                    ((1 << 24) - 1)

/* SPI framing bytes */
#define LNUL        (0x00)
#define STRW        (0x01)
#define STRR        (0x02)
#define SRPT        (0x03)
#define NACK        (0x04)
#define ENDP        (0x06)
#define INIT        (0x08)
#define HNUL        (0xff)

/* Switch payload size parameters */
#define SWITCH_CPORT_NCP_MAX_PAYLOAD    (256)
#define SWITCH_CPORT_DATA_MAX_PAYLOAD   (272)
#define SWITCH_CPORT_NCP_FIFO_SIZE      (256)
#define SWITCH_CPORT_DATA_FIFO_SIZE     (576)
#define SWITCH_CPORT_NCP_ENTRY_SIZE     (32)
#define SWITCH_CPORT_DATA_ENTRY_SIZE    (576)

/* Switch FIFOs for SVC communication over SPI */
#define SWITCH_FIFO_NCP   (0x03)
#define SWITCH_FIFO_DATA4 (0x04)
#define SWITCH_FIFO_DATA5 (0x05)

/*
 * @brief Switch QoS cycle rate is 166 MHz.  Convert back and forth.
 */
#define QOS_CYCLES_TO_USECS(n) (n/166)

/*
 * @brief UniPro connections
 */
struct unipro_connection {
    uint8_t port_id0;
    uint8_t device_id0;
    uint16_t cport_id0;
    uint8_t port_id1;
    uint8_t device_id1;
    uint16_t cport_id1;
    uint8_t tc;
    uint8_t flags;
    uint8_t state;
};

/**
 * @brief Toshiba-specific L2 timer configuraion to apply to the local
 *        end of the link during a power mode change.
 *
 * A particular attribute is only reconfigured if the corresponding
 * flag is set. E.g., `tsb_fc0_protection_timeout` is used to set
 * TSB_DME_FC0PROTECTIONTIMEOUTVAL if and only if TSB_LOCAL_L2_FC0 is
 * set in `flags'.
 *
 * @see switch_configure_link()
 */
struct tsb_local_l2_timer_cfg {
    /* Set TSB_DME_FC0PROTECTIONTIMEOUTVAL */
#   define TSB_LOCALL2F_FC0  (1U << 0)
    /* Set TSB_DME_TC0REPLAYTIMEOUTVAL */
#   define TSB_LOCALL2F_TC0  (1U << 1)
    /* Set TSB_DME_AFC0REQTIMEOUTVAL */
#   define TSB_LOCALL2F_AFC0 (1U << 2)
    /* Set TSB_DME_FC1PROTECTIONTIMEOUTVAL */
#   define TSB_LOCALL2F_FC1  (1U << 3)
    /* Set TSB_DME_TC1REPLAYTIMEOUTVAL */
#   define TSB_LOCALL2F_TC1  (1U << 4)
    /* Set TSB_DME_AFC1REQTIMEOUTVAL */
#   define TSB_LOCALL2F_AFC1 (1U << 5)
    unsigned int tsb_flags;

    uint16_t tsb_fc0_protection_timeout;
    uint16_t tsb_tc0_replay_timeout;
    uint16_t tsb_afc0_req_timeout;
    uint16_t tsb_fc1_protection_timeout;
    uint16_t tsb_tc1_replay_timeout;
    uint16_t tsb_afc1_req_timeout;
};

/**
 * Extra Toshiba extensions for UniPro link configuration.
 */
struct tsb_link_cfg {
    struct tsb_local_l2_timer_cfg tsb_l2tim_cfg;
};

/**
 * Switch structs
 */
struct tsb_switch;

struct tsb_switch_ops {
    int (*init_comm)(struct tsb_switch *);

    int (*enable_port)(struct tsb_switch *,
                       uint8_t portid,
                       bool enable);

    int (*link_startup)(struct tsb_switch *,
                        uint8_t portid);

    /*
     * Fill in NCP DME requests and their sizes
     *
     * These routines take the parameters needed for the DME NCP
     * request payloads, and copy them into req, which has size
     * *req_size at invocation time. They leave the number of bytes
     * copied into req in *req_size.
     *
     * The caller must ensure a safe maximum buffer size. A minimum
     * value safe for all requests is in sw->rdata->ncp_req_max_size.
     */
    void (*switch_id_set_req)(struct tsb_switch *sw,
                              uint8_t cportid,
                              uint8_t peer_cportid,
                              uint8_t dis,
                              uint8_t irt,
                              uint8_t *req, size_t *req_size);
    void (*set_req)(struct tsb_switch *sw,
                    uint8_t portid,
                    uint16_t attrid,
                    uint16_t select_index,
                    uint32_t attr_value,
                    uint8_t *req, size_t *req_size);
    void (*get_req)(struct tsb_switch *sw,
                    uint8_t portid,
                    uint16_t attrid,
                    uint16_t select_index,
                    uint8_t *req, size_t *req_size);
    void (*peer_set_req)(struct tsb_switch *sw,
                         uint8_t portid,
                         uint16_t attrid,
                         uint16_t select_index,
                         uint32_t attr_value,
                         uint8_t *req, size_t *req_size);
    void (*peer_get_req)(struct tsb_switch *sw,
                         uint8_t portid,
                         uint16_t attrid,
                         uint16_t select_index,
                         uint8_t *req, size_t *req_size);
    void (*lut_set_req)(struct tsb_switch *sw,
                        uint8_t unipro_portid,
                        uint8_t lut_address,
                        uint8_t dest_portid,
                        uint8_t *req, size_t *req_size);
    void (*lut_get_req)(struct tsb_switch *sw,
                        uint8_t unipro_portid,
                        uint8_t lut_address,
                        uint8_t *req, size_t *req_size);
    void (*switch_attr_get_req)(struct tsb_switch *sw,
                               uint16_t attrid,
                               uint8_t *req, size_t *req_size);
    void (*switch_attr_set_req)(struct tsb_switch *sw,
                               uint16_t attrid,
                               uint32_t val,
                               uint8_t *req, size_t *req_size);
    void (*sys_ctrl_set_req)(struct tsb_switch *sw,
                             uint16_t sc_addr,
                             uint32_t val,
                             uint8_t *req, size_t *req_size);
    void (*sys_ctrl_get_req)(struct tsb_switch *sw,
                             uint16_t sc_addr,
                             uint8_t *req, size_t *req_size);
    void (*qos_attr_set_req)(struct tsb_switch *sw,
                             uint8_t portid,
                             uint8_t attrid,
                             uint32_t attr_val,
                             uint8_t *req, size_t *req_size);
    void (*qos_attr_get_req)(struct tsb_switch *sw,
                             uint8_t portid,
                             uint8_t attrid,
                             uint8_t *req, size_t *req_size);
    void (*dev_id_mask_get_req)(struct tsb_switch *sw,
                                uint8_t unipro_portid,
                                uint8_t *req, size_t *req_size);
    void (*dev_id_mask_set_req)(struct tsb_switch *sw,
                                uint8_t unipro_portid,
                                uint8_t *mask,
                                uint8_t *req, size_t *req_size);

    int (*switch_data_send)(struct tsb_switch *, void *, size_t);

    /*
     * The following functions exist to make it easier to support ES2
     * and ES3 at the same time.
     *
     * They don't have corresponding user-visible APIs.
     */
    uint8_t* (*__switch_init_rxbuf)(struct tsb_switch *sw);
    int (*__post_init_seq)(struct tsb_switch *sw);
    int (*__ncp_transfer)(struct tsb_switch *sw,
                          uint8_t *tx_buf, size_t tx_size,
                          uint8_t *rx_buf, size_t rx_size);
    int (*__irq_fifo_rx)(struct tsb_switch *sw, unsigned int spi_fifo);
    int (*__set_valid_entry)(struct tsb_switch *sw,
                             uint8_t *table, int entry, bool valid);
    bool (*__check_valid_entry)(struct tsb_switch *sw,
                                uint8_t *table, int entry);
};

struct tsb_switch {
    void                    *priv;
    struct tsb_switch_ops   *ops;
    struct vreg             *vreg;
    struct tsb_switch_data  *pdata;
    struct tsb_rev_data     *rdata;
    struct spi_dev_s        *spi_dev;
    sem_t                   sw_irq_lock;
    int                     worker_id;
    bool                    sw_irq_worker_exit;
    uint8_t                 dev_ids[SWITCH_PORT_MAX];

    struct list_head        listeners;
};

/*
 * Low-level NCP support routines
 */

int switch_dme_set(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t attr_value);

int switch_dme_get(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t *attr_value);

int switch_dme_peer_set(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t attr_value);

int switch_dme_peer_get(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t *attr_value);

int switch_lut_get(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t addr,
                   uint8_t *dst_portid);

int switch_lut_set(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t addr,
                   uint8_t dst_portid);

int switch_internal_getattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t *val);

int switch_internal_setattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t val);

int switch_sys_ctrl_set(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t val);

int switch_sys_ctrl_get(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t *val);

int switch_qos_attr_set(struct tsb_switch *sw, uint8_t portid, uint8_t attrid,
                        uint32_t attr_val);

int switch_qos_attr_get(struct tsb_switch *sw, uint8_t portid, uint8_t attrid,
                        uint32_t *val);

int switch_dev_id_mask_get(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *dst);

int switch_dev_id_mask_set(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *mask);

/*
 * Switch events
 */

enum tsb_switch_event_type {
    TSB_SWITCH_EVENT_MAILBOX,
    TSB_SWITCH_EVENT_LINKUP_IND,
    TSB_SWITCH_EVENT_LINKUP,
};

struct tsb_switch_event {
    enum tsb_switch_event_type type;

    union {
        struct tsb_switch_event_linkup {
            uint32_t port;
            uint32_t val;
        } linkup;
        struct tsb_switch_event_mbox {
            uint32_t port;
            uint32_t val;
        } mbox;
    };
};

struct tsb_switch_event_listener {
    struct list_head entry;
    int (*cb)(struct tsb_switch_event *event);
};

int switch_event_register_listener(struct tsb_switch *sw,
                                   struct tsb_switch_event_listener *l);

int switch_if_dev_id_get(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t *dev_id);
int switch_if_dev_id_set(struct tsb_switch *sw,
                         uint8_t port_id,
                         uint8_t dev_id);
int switch_connection_create(struct tsb_switch *sw,
                             struct unipro_connection *conn);
int switch_connection_destroy(struct tsb_switch *sw,
                              struct unipro_connection *c);
int switch_cport_connect(struct tsb_switch *sw, uint8_t port_id,
                         uint16_t cport_id);

int switch_configure_link(struct tsb_switch *sw,
                          uint8_t port_id,
                          const struct unipro_link_cfg *cfg,
                          const struct tsb_link_cfg *tcfg);

/**
 * @brief "Auto" mode variant selection during link configuration.
 *
 * If set when configuring a link for an HS or PWM gear, choose the
 * "auto" mode variant. This provides lower power with automatic
 * transitions between BURST and SLEEP M-PHY states, at the cost of
 * extra latency to come out of SLEEP. */
#define UNIPRO_LINK_CFGF_AUTO         (1U << 0)

#define TSB_DEFAULT_PWR_USER_DATA                               \
    {                                                           \
        .flags = UPRO_PWRF_FC0,                                 \
        .upro_pwr_fc0_protection_timeout = 0x1FFF,              \
    }

/**
 * @brief Configure both directions of a link to the given HS gear.
 *
 * @param sw Switch handle
 * @param port_id Port whose link to configure
 * @param gear HS gear to use (HS-G1=1, HS-G2=2, HS-G3=3)
 * @param nlanes Number of lanes to use (1, 2, ...)
 * @param flags UNIPRO_LINK_CFGF_xxx
 */
static inline int switch_configure_link_hs(struct tsb_switch *sw,
                                           uint8_t port_id,
                                           unsigned int gear,
                                           unsigned int nlanes,
                                           unsigned int flags) {
    int auto_variant = flags & UNIPRO_LINK_CFGF_AUTO;
    const struct unipro_link_cfg lcfg = {
        .upro_hs_ser = UNIPRO_HS_SERIES_UNCHANGED,
        .upro_tx_cfg = UNIPRO_FAST_PWR_CFG(auto_variant, gear, nlanes),
        .upro_rx_cfg = UNIPRO_FAST_PWR_CFG(auto_variant, gear, nlanes),
        .upro_user   = TSB_DEFAULT_PWR_USER_DATA,
        .flags       = UPRO_LINKF_TX_TERMINATION | UPRO_LINKF_RX_TERMINATION,
    };
    return switch_configure_link(sw, port_id, &lcfg, NULL);
}

/**
 * @brief Configure both directions of a link to the given PWM gear.
 *
 * @param sw Switch handle
 * @param port_id Port whose link to configure
 * @param gear PWM gear to use (PWM-G1=1, PWM-G2=2, ..., PWM-G7=7)
 * @param nlanes Number of lanes to use (1, 2, ...)
 * @param flags UNIPRO_LINK_CFGF_xxx
 */
static inline int switch_configure_link_pwm(struct tsb_switch *sw,
                                            uint8_t port_id,
                                            unsigned int gear,
                                            unsigned int nlanes,
                                            unsigned int flags) {
    int auto_variant = flags & UNIPRO_LINK_CFGF_AUTO;
    const struct unipro_link_cfg lcfg = {
        .upro_hs_ser = UNIPRO_HS_SERIES_UNCHANGED,
        .upro_tx_cfg = UNIPRO_SLOW_PWR_CFG(auto_variant, gear, nlanes),
        .upro_rx_cfg = UNIPRO_SLOW_PWR_CFG(auto_variant, gear, nlanes),
        .upro_user   = TSB_DEFAULT_PWR_USER_DATA,
        .flags       = UPRO_LINKF_TX_TERMINATION,
    };
    return switch_configure_link(sw, port_id, &lcfg, NULL);
}

int switch_setup_routing_table(struct tsb_switch *sw,
                               uint8_t device_id_0,
                               uint8_t port_id_0,
                               uint8_t device_id_1,
                               uint8_t port_id_1);

int switch_set_valid_device(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint8_t device_id,
                            bool valid);

int switch_invalidate_routing_table(struct tsb_switch *sw,
                                    uint8_t device_id_0,
                                    uint8_t port_id_0,
                                    uint8_t device_id_1,
                                    uint8_t port_id_1);

int switch_dump_routing_table(struct tsb_switch *sw);

/**
 * @brief Parameters used by the UniPro Test Feature
 */
struct unipro_test_feature_cfg {
    /*
     * Source test feature parameters
     */

    /** Which source Test Feature to use */
    uint16_t tf_src;
    /** Which CPort to associate to the source Test Feature. */
    uint16_t tf_src_cportid;
    /**
     * Increment between successive bytes in messages sent by the test feature.
     */
    uint8_t tf_src_inc;
    /**
     * Size of messages sent by the test feature.
     */
    uint16_t tf_src_size;
    /**
     * Number of messages to be sent by the test feature source.
     *
     * If zero, messages will be sent as long as the test feature is enabled.
     */
    uint16_t tf_src_count;
    /**
     * Time delay, in microseconds, between messages sent by the test feature.
     */
    uint16_t tf_src_gap_us;

    /*
     * Destination test feature parameters
     *
     * TODO: add support for the T_TstDstXXX attributes, to support
     *       test destination traffic analyzer (to e.g. search for
     *       dropped packets on the switch).
     */

    /** Which destination Test Feature to use */
    uint16_t tf_dst;
    /** Which CPort to associate to the destination Test Feature */
    uint16_t tf_dst_cportid;
    /** If error checking is enabled on the destination */
    bool tf_dst_error_detection_enable;
};

int switch_enable_test_traffic(struct tsb_switch *sw,
                               uint8_t src_portid,
                               uint8_t dst_portid,
                               const struct unipro_test_feature_cfg *cfg);

int switch_disable_test_traffic(struct tsb_switch *sw,
                                uint8_t src_portid, uint8_t dst_portid,
                                const struct unipro_test_feature_cfg *cfg);

/*
 * Revision (ES2, ES3) specific data for switch management
 */
struct tsb_rev_data {
    /* Number of bytes to wait for a reply in init */
    size_t wait_reply_len;

    /* Max size in bytes of an NCP command request. */
    size_t ncp_req_max_size;

    /*
     * Number of entries in a device ID mask (for destination
     * validation blocks)
     */
    size_t dev_id_mask_size;

    /*
     * Switch internal attribute values used during IRQ enable
     */
    uint32_t spicee_enable_all;
    uint32_t spi3ee_enable_all;
    uint32_t spi45ee_enable_all;

    /* Revision-specific flags */
    uint32_t rflags;
    /* This revision of the switch needs port IRQs re-enabled every
     * time they're disabled, as long as the global switch IRQ is
     * enabled. */
#define TSB_SWITCH_RFLAG_REENABLE_PORT_IRQ_HACK 0x1
};

/*
 * Platform specific data for switch initialization
 */
struct tsb_switch_data {
    struct vreg *vreg;
    unsigned int gpio_1p1;
    unsigned int gpio_1p8;
    unsigned int gpio_reset;
    unsigned int gpio_irq;
    bool         irq_rising_edge;
    unsigned int rev;
    unsigned int bus;
    unsigned int spi_cs;
};

enum {
    SWITCH_REV_ES1 = 1,
    SWITCH_REV_ES2 = 2,
    SWITCH_REV_ES3 = 3,
};

struct tsb_switch *switch_init(struct tsb_switch_data *pdata);
void switch_exit(struct tsb_switch*);

int switch_enable_port(struct tsb_switch *,
                       uint8_t portid,
                       bool enable);

int switch_link_startup(struct tsb_switch *,
                        uint8_t portid);

int _switch_irq_pending_worker(int argc, char *argv[]);
int switch_port_irq_enable(struct tsb_switch *sw,
                           uint8_t portid,
                           bool enable);

int switch_qos_band_reset(struct tsb_switch *sw, uint8_t portid);
int switch_qos_bwctrl_enabled(struct tsb_switch *sw, uint8_t portid, uint8_t tc,
                              uint8_t *val);
int switch_qos_enable_bwctrl(struct tsb_switch *sw, uint8_t portid, uint8_t tc);
int switch_qos_disable_bwctrl(struct tsb_switch *sw, uint8_t portid,
                              uint8_t tc);
int switch_qos_get_subtc(struct tsb_switch *sw, uint8_t portid, uint8_t *tc);
int switch_qos_set_subtc(struct tsb_switch *sw, uint8_t portid, uint8_t tc);
int switch_qos_port_closed(struct tsb_switch *sw, uint8_t portid, bool *closed);
int switch_qos_open_port(struct tsb_switch *sw, uint8_t portid);
int switch_qos_close_port(struct tsb_switch *sw, uint8_t portid);
int switch_qos_peer_implements_tc(struct tsb_switch *sw, uint8_t portid,
                                  uint8_t tc, uint8_t *val);
int switch_qos_transmit_accept_signal(struct tsb_switch *sw, uint8_t portid,
                                      uint8_t *val);
int switch_qos_transmit_valid_signal(struct tsb_switch *sw, uint8_t portid,
                                     uint8_t *val);
int switch_qos_gate_arb_busy(struct tsb_switch *sw, uint8_t *val);
int switch_qos_gate_arb_output(struct tsb_switch *sw, uint8_t port,
                               uint8_t *val);
int switch_qos_output_traffic_class(struct tsb_switch *sw, uint8_t *val);
int switch_qos_port_connected(struct tsb_switch *sw, uint8_t port,
                              uint8_t *val);
int switch_qos_request_status(struct tsb_switch *sw, uint8_t tc, uint8_t port,
                              uint8_t *val);
int switch_qos_get_bwperiod(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                            uint32_t *val);
int switch_qos_set_bwperiod(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                            uint32_t val);
int switch_qos_get_decsize(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                           uint32_t *val);
int switch_qos_set_decsize(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                           uint32_t val);
int switch_qos_get_limit(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                         uint32_t *val);
int switch_qos_set_limit(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                         uint32_t val);
int switch_qos_source_quantity(struct tsb_switch *sw, uint8_t port, uint8_t tc,
                               uint32_t *val);
int switch_qos_transmit_quantity(struct tsb_switch *sw, uint32_t *val);
int switch_qos_reset_routing_table(struct tsb_switch *sw, uint8_t tc);
int switch_qos_source_accept_signal(struct tsb_switch *sw, uint8_t port,
                                    uint8_t tc, uint8_t *val);
int switch_qos_source_valid_signal(struct tsb_switch *sw, uint8_t port,
                                   uint8_t tc, uint8_t *val);
int switch_qos_get_in_wdt_count(struct tsb_switch *sw, uint32_t *val);
int switch_qos_set_in_wdt_count(struct tsb_switch *sw, uint32_t val);
int switch_qos_get_out_wdt_count(struct tsb_switch *sw, uint32_t *val);
int switch_qos_set_out_wdt_count(struct tsb_switch *sw, uint32_t val);

int switch_irq_enable(struct tsb_switch *sw,
                      bool enable);
int switch_post_irq(struct tsb_switch *sw);

int switch_fct_enable(struct tsb_switch *);
int switch_data_send(struct tsb_switch *sw, void *data, size_t len);

/*
 * Switch specific init/exit routines, called to init the
 * communication and register the Switch ops
 */
int tsb_switch_es2_init(struct tsb_switch *, struct spi_dev_s *spi_dev);
void tsb_switch_es2_exit(struct tsb_switch *);
int tsb_switch_es3_init(struct tsb_switch *, struct spi_dev_s *spi_dev);
void tsb_switch_es3_exit(struct tsb_switch *);

/* Internal SPI communication helpers. */
void _switch_spi_select(struct tsb_switch *sw, int select);
int _switch_transfer_check_write_status(uint8_t *status_block, size_t size);

#endif
