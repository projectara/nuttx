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

#define DBG_COMP ARADBG_SWITCH
#include "tsb_switch.h"

#include <stdbool.h>
#include <string.h>

#include <nuttx/config.h>
#include <arch/byteorder.h>
#include <ara_debug.h>

/**
 * This file contains the shared implementations of the primitive
 * "NCP" commands used by the SVC to control the switch.
 *
 * They can be used either directly for debugging or as subroutines by
 * higher-level switch code.
 */

/*
 * Internal "ops" implementations and helpers.
 */

static int ncp_transfer(struct tsb_switch *sw,
                        uint8_t *tx_buf, size_t tx_size,
                        uint8_t *rx_buf, size_t rx_size) {
    DEBUGASSERT(sw->ops->__ncp_transfer);
    return sw->ops->__ncp_transfer(sw, tx_buf, tx_size, rx_buf, rx_size);
}

static void get_switch_id_set_req(struct tsb_switch *sw,
                                  uint8_t cportid,
                                  uint8_t peer_cportid,
                                  uint8_t dis,
                                  uint8_t irt,
                                  uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->switch_id_set_req);
    sw->ops->switch_id_set_req(sw, cportid, peer_cportid, dis, irt, req, req_size);
}

static void get_dme_set_req(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint16_t attrid,
                            uint16_t select_index,
                            uint32_t val,
                            uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->set_req);
    sw->ops->set_req(sw, port_id, attrid, select_index, val, req, req_size);
}

static void get_dme_get_req(struct tsb_switch *sw,
                            uint8_t port_id,
                            uint16_t attrid,
                            uint16_t select_index,
                            uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->get_req);
    sw->ops->get_req(sw, port_id, attrid, select_index, req, req_size);
}

static void get_dme_peer_set_req(struct tsb_switch *sw,
                                 uint8_t port_id,
                                 uint16_t attrid,
                                 uint16_t select_index,
                                 uint32_t val,
                                 uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->peer_set_req);
    sw->ops->peer_set_req(sw, port_id, attrid, select_index, val, req, req_size);
}

static void get_dme_peer_get_req(struct tsb_switch *sw,
                                 uint8_t port_id,
                                 uint16_t attrid,
                                 uint16_t select_index,
                                 uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->peer_get_req);
    sw->ops->peer_get_req(sw, port_id, attrid, select_index, req, req_size);
}

static void get_lut_set_req(struct tsb_switch *sw,
                            uint8_t unipro_portid,
                            uint8_t lut_address,
                            uint8_t dest_portid,
                            uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->lut_set_req);
    sw->ops->lut_set_req(sw, unipro_portid, lut_address, dest_portid,
                         req, req_size);
}

static void get_lut_get_req(struct tsb_switch *sw,
                            uint8_t unipro_portid,
                            uint8_t lut_address,
                            uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->lut_get_req);
    sw->ops->lut_get_req(sw, unipro_portid, lut_address, req, req_size);
}

static void get_switch_attr_get_req(struct tsb_switch *sw,
                                    uint16_t attrid,
                                    uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->switch_attr_get_req);
    sw->ops->switch_attr_get_req(sw, attrid, req, req_size);
}

static void get_switch_attr_set_req(struct tsb_switch *sw,
                                    uint16_t attrid,
                                    uint32_t val,
                                    uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->switch_attr_set_req);
    sw->ops->switch_attr_set_req(sw, attrid, val, req, req_size);
}

static void get_sys_ctrl_set_req(struct tsb_switch *sw,
                                 uint16_t sc_addr,
                                 uint32_t val,
                                 uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->sys_ctrl_set_req);
    sw->ops->sys_ctrl_set_req(sw, sc_addr, val, req, req_size);
}

static void get_sys_ctrl_get_req(struct tsb_switch *sw,
                                 uint16_t sc_addr,
                                 uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->sys_ctrl_get_req);
    sw->ops->sys_ctrl_get_req(sw, sc_addr, req, req_size);
}

static void get_qos_attr_set_req(struct tsb_switch *sw,
                                 uint8_t portid,
                                 uint8_t attrid,
                                 uint32_t attr_val,
                                 uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->qos_attr_set_req);
    sw->ops->qos_attr_set_req(sw, portid, attrid, attr_val, req, req_size);
}

static void get_qos_attr_get_req(struct tsb_switch *sw,
                                 uint8_t portid,
                                 uint8_t attrid,
                                 uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->qos_attr_get_req);
    sw->ops->qos_attr_get_req(sw, portid, attrid, req, req_size);
}

static void get_dev_id_mask_get_req(struct tsb_switch *sw,
                                    uint8_t unipro_portid,
                                    uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->dev_id_mask_get_req);
    sw->ops->dev_id_mask_get_req(sw, unipro_portid, req, req_size);
}

static void get_dev_id_mask_set_req(struct tsb_switch *sw,
                                    uint8_t unipro_portid,
                                    uint8_t *mask,
                                    uint8_t *req, size_t *req_size) {
    DEBUGASSERT(sw->ops->dev_id_mask_set_req);
    sw->ops->dev_id_mask_set_req(sw, unipro_portid, mask, req, req_size);
}


/*
 * Switch initialization NCP primitive
 */

int _switch_internal_set_id(struct tsb_switch *sw,
                            uint8_t cportid,
                            uint8_t peer_cportid,
                            uint8_t dis,
                            uint8_t irt) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
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

    get_switch_id_set_req(sw, cportid, peer_cportid, dis, irt, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
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

/*
 * DME accessors
 */

int switch_dme_set(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t attr_value) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d, val=0x%04x\n",
                __func__, portid, attrid, select_index, attr_value);

    get_dme_set_req(sw, portid, attrid, select_index, attr_value,
                    req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): portId=%u, attrId=0x%04x failed: rc=%d\n",
                  __func__, portid, attrid, rc);
        return rc;
    }
    if (cnf.function_id != NCP_SETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, attr_value);

    return cnf.rc;
}

int switch_dme_get(struct tsb_switch *sw,
                   uint8_t portid,
                   uint16_t attrid,
                   uint16_t select_index,
                   uint32_t *attr_value) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): portId=%d, attrId=0x%04x, selectIndex=%d\n",
                __func__, portid, attrid, select_index);

    get_dme_get_req(sw, portid, attrid, select_index,
                    req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }
    if (cnf.function_id != NCP_GETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *attr_value = be32_to_cpu(cnf.attr_val);
    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, *attr_value);

    return cnf.rc;
}

int switch_dme_peer_set(struct tsb_switch *sw,
                        uint8_t portid,
                        uint16_t attrid,
                        uint16_t select_index,
                        uint32_t attr_value) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    } cnf;

    dbg_verbose("%s(): portid=%d, attrId=0x%04x, selectIndex=%d, val=0x%04x\n",
                __func__, portid, attrid, select_index, attr_value);

    get_dme_peer_set_req(sw, portid, attrid, select_index, attr_value,
                         req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): portid=%u, attrId=0x%04x failed: rc=%d\n",
                  __func__, portid, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_PEERSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, attr_value);

    return cnf.rc;
}

int switch_dme_peer_get(struct tsb_switch *sw,
                               uint8_t portid,
                               uint16_t attrid,
                               uint16_t select_index,
                               uint32_t *attr_value) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): portid=%d, attrId=0x%04x, selectIndex=%d\n",
                 __func__, portid, attrid, select_index);

    get_dme_peer_get_req(sw, portid, attrid, select_index, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }

    if (cnf.function_id != NCP_PEERGETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    *attr_value = be32_to_cpu(cnf.attr_val);
    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, *attr_value);

    return cnf.rc;
}

/*
 * Routing table configuration commands
 */

int switch_lut_set(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t lut_address,
                   uint8_t dest_portid) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;

        /* Two pad bytes are needed to retrieve a couple of fields
         * returned on ES2 that we don't care about. (We get back null
         * bytes here on ES3.) */
        uint8_t padding1;
        uint8_t padding2;
    } cnf;

    dbg_verbose("%s(): unipro_portid=%d, lutAddress=%d, destPortId=%d\n",
                __func__, unipro_portid, lut_address, dest_portid);

    get_lut_set_req(sw, unipro_portid, lut_address, dest_portid,
                    req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
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

    return cnf.rc;

}

int switch_lut_get(struct tsb_switch *sw,
                   uint8_t unipro_portid,
                   uint8_t lut_address,
                   uint8_t *dest_portid) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t padding;  /* for a "don't care" field on ES2 */
        uint8_t dest_portid;
    } cnf;

    dbg_verbose("%s(): unipro_portid=%d, lutAddress=%d, destPortId=%d\n",
                __func__, unipro_portid, lut_address, *dest_portid);

    get_lut_get_req(sw, unipro_portid, lut_address, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
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

    return cnf.rc;
}

/*
 * Switch attribute accessors
 */

int switch_internal_getattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t *val) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s(): attrId=0x%04x\n", __func__, attrid);

    get_switch_attr_get_req(sw, attrid, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
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

int switch_internal_setattr(struct tsb_switch *sw,
                            uint16_t attrid,
                            uint32_t val) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;
    bool ignore_reply = false;

    dbg_verbose("%s(): attrId=0x%04x\n", __func__, attrid);

    switch (attrid) {
    /* Special case for SW_ATTR_SWRES attribute: ignore reply */
    case SW_ATTR_SWRES:
        ignore_reply = true;
        break;
    default:
        break;
    }

    get_switch_attr_set_req(sw, attrid, val, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): attrId=0x%04x failed: rc=%d\n", __func__, attrid, rc);
        return rc;
    }
    if (ignore_reply) {
        /* Hopefully that worked. We have no way of knowing. */
        dbg_verbose("%s(): ignoring reply\n", __func__);
        return 0;
    }

    if (cnf.function_id != NCP_SWITCHATTRSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u, attr(0x%04x)=0x%04x\n",
                __func__, cnf.function_id, cnf.rc, attrid, val);

    return cnf.rc;
}

/*
 * System controller accessors
 */

int switch_sys_ctrl_set(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t val) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__((packed)) cnf {
        uint8_t rc;
        uint8_t function_id;
    } cnf;
    bool ignore_reply = false;

    dbg_verbose("%s(): sc_addr=0x%x, val=0x%x (%d)\n",
                __func__, sc_addr, val, val);

    switch (sc_addr) {
    /* Special case for PMU_StandbySqStart register: ignore reply */
    case SC_PMUSTANDBYSS:
        ignore_reply = true;
        break;
    default:
        break;
    }

    get_sys_ctrl_set_req(sw, sc_addr, val, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s(): sc_addr=0x%x, val=0x%x (%d) failed: %d\n",
                  __func__, sc_addr, val, val, rc);
        return rc;
    }
    if (ignore_reply) {
        /* Hopefully that worked. We have no way of knowing. */
        dbg_verbose("%s(): ignoring reply\n", __func__);
        return 0;
    }

    if (cnf.function_id != NCP_SYSCTRLSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__, cnf.function_id, cnf.rc);
    return cnf.rc;
}

int switch_sys_ctrl_get(struct tsb_switch *sw,
                        uint16_t sc_addr,
                        uint32_t *val) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__((packed)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint32_t val;
    } cnf;

    dbg_verbose("%s(): sc_addr=0x%x\n", __func__, sc_addr);
    get_sys_ctrl_get_req(sw, sc_addr, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
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

/*
 * Switch QoS configuration commands
 */

int switch_qos_attr_set(struct tsb_switch *sw,
                        uint8_t portid,
                        uint8_t attrid,
                        uint32_t attr_val) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    /*
     * There is a gratuitous difference in the cnf framing for this
     * NCP command on ES3, which this wart / abstraction violation
     * reflects.
     */
    struct __attribute__((packed)) es2_cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
    };
    struct __attribute__((packed)) es3_cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
    };
    union {
        struct es2_cnf es2;
        struct es3_cnf es3;
    } cnf;
    size_t cnf_size = sizeof(cnf);
    uint8_t *cnf_portid;
    uint8_t *cnf_function_id;
    uint8_t *cnf_rc;

    dbg_verbose("%s: portid: %u attrid: %u attr_val: %u\n",
                __func__,
                portid,
                attrid,
                attr_val);

    get_qos_attr_set_req(sw, portid, attrid, attr_val, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, cnf_size);
    if (rc) {
        dbg_error("%s() failed: rc=%d\n", __func__, rc);
        return rc;
    }

    switch (sw->pdata->rev) {
    case SWITCH_REV_ES2:
        cnf_portid = &cnf.es2.portid;
        cnf_function_id = &cnf.es2.function_id;
        cnf_rc = &cnf.es2.rc;
        break;
    case SWITCH_REV_ES3:
        cnf_portid = &cnf.es3.portid;
        cnf_function_id = &cnf.es3.function_id;
        cnf_rc = &cnf.es3.rc;
        break;
    default:
        dbg_error("%s: unsupported switch revision: %u\n",
                  __func__, sw->pdata->rev);
        return -EINVAL;
    }

    if (*cnf_portid != portid) {
        dbg_error("%s(): unexpected portid 0x%x\n", __func__, *cnf_portid);
        return -EPROTO;
    }

    if (*cnf_function_id != NCP_QOSATTRSETCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, *cnf_function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): ret=0x%02x, portid=0x%02x, attr(0x%04x)=0x%04x\n",
                __func__,
                *cnf_rc,
                portid,
                attrid,
                attr_val);

    return *cnf_rc;
}

int switch_qos_attr_get(struct tsb_switch *sw,
                        uint8_t portid,
                        uint8_t attrid,
                        uint32_t *val) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t portid;
        uint8_t function_id;
        uint8_t reserved;
        uint8_t rc;
        uint32_t attr_val;
    } cnf;

    dbg_verbose("%s: portid: %u attrid: %u\n", __func__, portid, attrid);

    get_qos_attr_get_req(sw, portid, attrid, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));

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

/*
 * Device ID masking (for destination validation)
 */

int switch_dev_id_mask_get(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *dst) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
        uint8_t mask[sw->rdata->dev_id_mask_size];
    } cnf;

    dbg_verbose("%s(%d)\n", __func__, unipro_portid);

    get_dev_id_mask_get_req(sw, unipro_portid, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s()failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_GETDEVICEIDMASKCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    memcpy(dst, &cnf.mask, sizeof(cnf.mask));

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__,
                cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}

int switch_dev_id_mask_set(struct tsb_switch *sw,
                           uint8_t unipro_portid,
                           uint8_t *mask) {
    int rc;
    size_t req_size = sw->rdata->ncp_req_max_size;
    uint8_t req[req_size];
    struct __attribute__ ((__packed__)) cnf {
        uint8_t rc;
        uint8_t function_id;
        uint8_t portid;
        uint8_t reserved;
    } cnf;

    dbg_verbose("%s()\n", __func__);
    get_dev_id_mask_set_req(sw, unipro_portid, mask, req, &req_size);
    rc = ncp_transfer(sw, req, req_size, (uint8_t*)&cnf, sizeof(struct cnf));
    if (rc) {
        dbg_error("%s()failed: rc=%d\n", __func__, rc);
        return rc;
    }

    if (cnf.function_id != NCP_SETDEVICEIDMASKCNF) {
        dbg_error("%s(): unexpected CNF 0x%x\n", __func__, cnf.function_id);
        return -EPROTO;
    }

    dbg_verbose("%s(): fid=0x%02x, rc=%u\n", __func__,
                cnf.function_id, cnf.rc);

    /* Return resultCode */
    return cnf.rc;
}
