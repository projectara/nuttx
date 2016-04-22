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

#define DBG_COMP ARADBG_SVC

#include <errno.h>
#include <string.h>

#include <arch/byteorder.h>
#include <nuttx/config.h>
#include <nuttx/greybus/greybus.h>
#include <nuttx/greybus/debug.h>
#include <nuttx/unipro/unipro.h>

#include <arch/byteorder.h>

#include "svc.h"
#include <ara_debug.h>
#include "tsb_switch.h"
#include "gb_svc.h"
#include "timesync.h"

/*
 * FIXME: use the hardcoded endo id used in the kernel for now
 */
#define GB_ENDO_ID         0x4755

static unsigned int g_svc_cport;

/*
 * SVC Protocol Requests
 */
int gb_svc_protocol_version(void) {
    struct gb_operation *op_req;
    struct gb_operation *op_resp;
    struct gb_svc_protocol_version_request *version_request;
    struct gb_svc_protocol_version_response *version_response;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_PROTOCOL_VERSION,
                                 sizeof(*version_request));
    if (!op_req) {
        return -ENOMEM;
    }
    version_request = gb_operation_get_request_payload(op_req);
    version_request->major = GB_SVC_VERSION_MAJOR;
    version_request->minor = GB_SVC_VERSION_MINOR;

    gb_operation_send_request_sync(op_req);

    op_resp = gb_operation_get_response_op(op_req);
    if (!op_resp) {
        gb_operation_destroy(op_req);
        dbg_error("protocol error during SVC protocol version\n");
        return GB_OP_PROTOCOL_BAD;
    }
    version_response = gb_operation_get_request_payload(op_resp);

    if (version_response->major > GB_SVC_VERSION_MAJOR) {
        dbg_error("unsupported major version: %u\n", version_response->major);
        gb_operation_destroy(op_req);
        return -EPROTO;
    }
    dbg_info("SVC Protocol version_major = %u version_minor = %u\n",
             version_response->major, version_response->minor);

    gb_operation_destroy(op_req);

    return 0;
}

int gb_svc_hello(uint8_t ap_intf_id) {
    struct gb_operation *op_req;
    struct gb_svc_hello_request *req;
    int rc;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_HELLO,
                                 sizeof(*req));
    if (!op_req) {
        dbg_error("Can't create SVC Hello operation\n");
        return -ENOMEM;
    }

    req = gb_operation_get_request_payload(op_req);
    req->endo_id = cpu_to_le16(GB_ENDO_ID);
    req->interface_id = ap_intf_id;

    rc = gb_operation_send_request_sync(op_req);
    if (rc) {
        dbg_error("Error during SVC Hello operation: %d\n", rc);
        goto err_send;
    }

    rc = gb_operation_get_request_result(op_req);
    if (rc) {
        dbg_error("Error status retrieved from SVC Hello result: %d\n",
                  rc);
    }

 err_send:
    gb_operation_destroy(op_req);
    return rc;
}

int gb_svc_intf_hotplug(uint32_t intf_id, uint32_t ddbl1_mfr_id,
                        uint32_t ddbl1_prod_id, uint32_t ara_vend_id,
                        uint32_t ara_prod_id, uint64_t serial_number) {
    struct gb_operation *op_req;
    struct gb_svc_intf_hotplug_request *req;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_INTF_HOTPLUG,
                                 sizeof(*req));
    if (!op_req) {
        return -ENOMEM;
    }

    req = gb_operation_get_request_payload(op_req);
    req->intf_id = intf_id;
    req->data.ddbl1_mfr_id = cpu_to_le32(ddbl1_mfr_id);
    req->data.ddbl1_prod_id = cpu_to_le32(ddbl1_prod_id);
    req->data.ara_vend_id = cpu_to_le32(ara_vend_id);
    req->data.ara_prod_id = cpu_to_le32(ara_prod_id);
    req->data.serial_number = cpu_to_le64(serial_number);

    gb_operation_send_request_sync(op_req);
    gb_operation_destroy(op_req);

    return 0;
}

int gb_svc_intf_hot_unplug(uint32_t intf_id) {
    struct gb_operation *op_req;
    struct gb_svc_intf_hot_unplug_request *req;

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_INTF_HOT_UNPLUG,
                                 sizeof(*req));
    if (!op_req) {
        return -ENOMEM;
    }

    req = gb_operation_get_request_payload(op_req);
    req->intf_id = intf_id;

    gb_operation_send_request_sync(op_req);
    gb_operation_destroy(op_req);

    return 0;
}

int gb_svc_key_event(uint16_t code, uint8_t event) {
    struct gb_operation *op_req;
    struct gb_svc_key_event_request *req;

    if ((event != GB_SVC_KEY_PRESSED) && (event != GB_SVC_KEY_RELEASED)) {
        return -EINVAL;
    }

    op_req = gb_operation_create(g_svc_cport, GB_SVC_TYPE_KEY_EVENT,
                                 sizeof(*req));
    if (!op_req) {
        return -ENOMEM;
    }

    req = gb_operation_get_request_payload(op_req);
    req->key_code = cpu_to_le16(code);
    req->key_event = event;

    gb_operation_send_request(op_req, NULL, false);
    gb_operation_destroy(op_req);

    return 0;
}

/*
 * SVC Protocol Request handlers
 */
static uint8_t gb_svc_intf_device_id(struct gb_operation *op) {
    struct gb_svc_intf_device_id_request *req;
    u8 intf_id;
    u8 dev_id;
    int rc;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    intf_id = req->intf_id;
    dev_id  = req->device_id;

    rc = svc_intf_device_id(intf_id, dev_id);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_intf_eject(struct gb_operation *op) {
    struct gb_svc_intf_eject_request *req;
    int rc;
    u8 intf_id;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    intf_id = req->intf_id;

    rc = svc_intf_eject(intf_id);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_ping(struct gb_operation *op) {
    /*
     * Perhaps in the future we might want to do some sort of "health" check
     * before responding, but for now, a simple ACK is all we need.
     */
    return 0;
}

static uint8_t gb_svc_connection_create(struct gb_operation *op) {
    struct gb_svc_conn_create_request *req;
    int rc;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    rc = svc_connection_create(req->intf1_id, le16_to_cpu(req->cport1_id),
                               req->intf2_id, le16_to_cpu(req->cport2_id),
                               req->tc, req->flags);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_connection_destroy(struct gb_operation *op)
{
    int retval;
    struct gb_svc_conn_destroy_request *req;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    retval = svc_connection_destroy(req->intf1_id, le16_to_cpu(req->cport1_id),
                                    req->intf2_id, le16_to_cpu(req->cport2_id));

    return gb_errno_to_op_result(retval);
}

static uint8_t gb_svc_dme_peer_get(struct gb_operation *op) {
    struct gb_svc_dme_peer_get_request *req;
    struct gb_svc_dme_peer_get_response *resp;
    int rc;
    uint16_t attr, selector;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    resp = gb_operation_alloc_response(op, sizeof(*resp));
    if (!resp) {
        return GB_OP_NO_MEMORY;
    }

    attr = le16_to_cpu(req->attr);
    selector = le16_to_cpu(req->selector);

    rc = svc_dme_peer_get(req->intf_id, attr, selector, &resp->result_code,
                          &resp->attr_value);
    resp->result_code = cpu_to_le16(resp->result_code);
    resp->attr_value = cpu_to_le32(resp->attr_value);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_dme_peer_set(struct gb_operation *op) {
    struct gb_svc_dme_peer_set_request *req;
    struct gb_svc_dme_peer_set_response *resp;
    int rc;
    uint16_t attr, selector;
    uint32_t value;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    resp = gb_operation_alloc_response(op, sizeof(*resp));
    if (!resp) {
        return GB_OP_NO_MEMORY;
    }

    attr = le16_to_cpu(req->attr);
    selector = le16_to_cpu(req->selector);
    value = le32_to_cpu(req->value);

    rc = svc_dme_peer_set(req->intf_id, attr, selector, value,
                          &resp->result_code);
    resp->result_code = cpu_to_le16(resp->result_code);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_route_create(struct gb_operation *op) {
    struct gb_svc_route_create_request *req;
    int rc;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    rc = svc_route_create(req->intf1_id, req->dev1_id,
                          req->intf2_id, req->dev2_id);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_route_destroy(struct gb_operation *op) {
    struct gb_svc_route_destroy_request *req;
    int rc;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    rc = svc_route_destroy(req->intf1_id, req->intf2_id);

    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_intf_set_power_mode(struct gb_operation *op) {
    struct gb_svc_intf_set_pwrm_request *req;
    struct gb_svc_intf_set_pwrm_response *resp;
    struct unipro_link_cfg cfg;
    uint32_t quirks;
    int rc;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    resp = gb_operation_alloc_response(op, sizeof(*resp));
    if (!resp) {
        return GB_OP_NO_MEMORY;
    }

    quirks = le32_to_cpu(req->quirks);

    /*
     * Validate the input parameters. The RX and TX parameters mode, gear and
     * nlanes are (indirectly) checked by svc_intf_set_power_mode(), we don't
     * need to duplicate the checks.
     */
    if (req->hs_series != GB_SVC_UNIPRO_HS_SERIES_A &&
        req->hs_series != GB_SVC_UNIPRO_HS_SERIES_B)
        return GB_OP_INVALID;

    if (req->flags & ~(GB_SVC_PWRM_RXTERMINATION | GB_SVC_PWRM_TXTERMINATION |
                       GB_SVC_PWRM_LINE_RESET | GB_SVC_PWRM_SCRAMBLING))
        return GB_OP_INVALID;

    if (quirks & ~GB_SVC_PWRM_QUIRK_HSSER)
        return GB_OP_INVALID;

    /* FIXME: GB_SVC_PWRM_LINE_RESET is not supported. */
    if (req->flags & GB_SVC_PWRM_LINE_RESET)
        return GB_OP_INVALID;

    /* Set the power mode */
    memset(&cfg, 0, sizeof(cfg));

    if (req->tx_mode == GB_SVC_UNIPRO_FAST_MODE ||
        req->tx_mode == GB_SVC_UNIPRO_FAST_AUTO_MODE ||
        req->rx_mode == GB_SVC_UNIPRO_FAST_MODE ||
        req->rx_mode == GB_SVC_UNIPRO_FAST_AUTO_MODE ||
        (quirks & GB_SVC_PWRM_QUIRK_HSSER))
        cfg.upro_hs_ser = req->hs_series;
    else
        cfg.upro_hs_ser = UNIPRO_HS_SERIES_UNCHANGED;

    switch (req->tx_mode) {
    case GB_SVC_UNIPRO_HIBERNATE_MODE:
        cfg.upro_tx_cfg.upro_mode = UNIPRO_HIBERNATE_MODE;
        break;
    case GB_SVC_UNIPRO_OFF_MODE:
        cfg.upro_tx_cfg.upro_mode = UNIPRO_OFF_MODE;
        break;
    default:
        /* All other values are identical. */
        cfg.upro_tx_cfg.upro_mode = req->tx_mode;
        break;
    }

    cfg.upro_tx_cfg.upro_gear = req->tx_gear;
    cfg.upro_tx_cfg.upro_nlanes = req->tx_nlanes;

    switch (req->rx_mode) {
    case GB_SVC_UNIPRO_HIBERNATE_MODE:
        cfg.upro_rx_cfg.upro_mode = UNIPRO_HIBERNATE_MODE;
        break;
    case GB_SVC_UNIPRO_OFF_MODE:
        cfg.upro_rx_cfg.upro_mode = UNIPRO_OFF_MODE;
        break;
    default:
        /* All other values are identical. */
        cfg.upro_rx_cfg.upro_mode = req->rx_mode;
        break;
    }

    cfg.upro_rx_cfg.upro_gear = req->rx_gear;
    cfg.upro_rx_cfg.upro_nlanes = req->rx_nlanes;

    if (req->flags & GB_SVC_PWRM_RXTERMINATION)
        cfg.flags |= UPRO_LINKF_RX_TERMINATION;
    if (req->flags & GB_SVC_PWRM_TXTERMINATION)
        cfg.flags |= UPRO_LINKF_TX_TERMINATION;
    if (req->flags & GB_SVC_PWRM_SCRAMBLING)
        cfg.flags |= UPRO_LINKF_SCRAMBLING;

    /* Default user data */
    cfg.upro_user.flags = UPRO_PWRF_FC0;
    cfg.upro_user.upro_pwr_fc0_protection_timeout = 0x1fff;

    rc = svc_intf_set_power_mode(req->intf_id, &cfg);
    if (rc < 0)
        return gb_errno_to_op_result(rc);

    resp->result_code = cpu_to_le16(rc);
    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_pwrmon_rail_count_get(struct gb_operation *operation)
{
    struct gb_svc_pwrmon_rail_count_get_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    return GB_OP_PROTOCOL_BAD; /* not supported yet */
}

static uint8_t gb_svc_pwrmon_rail_names_get(struct gb_operation *operation)
{
    return GB_OP_PROTOCOL_BAD; /* not supported yet */
}

static uint8_t gb_svc_pwrmon_sample_get(struct gb_operation *operation)
{
    struct gb_svc_pwrmon_sample_get_request *request;
    struct gb_svc_pwrmon_sample_get_response *response;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    return GB_OP_PROTOCOL_BAD; /* not supported yet */
}

static uint8_t gb_svc_pwrmon_intf_sample_get(struct gb_operation *operation)
{
    struct gb_svc_pwrmon_intf_sample_get_request *request;
    struct gb_svc_pwrmon_intf_sample_get_response *response;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    return GB_OP_PROTOCOL_BAD; /* not supported yet */
}

static uint8_t gb_svc_intf_vsys_enable(struct gb_operation *operation)
{
    struct gb_svc_intf_vsys_enable_request *request;
    struct gb_svc_intf_vsys_enable_response *response;
    int rc;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    request = gb_operation_get_request_payload(operation);

    rc = svc_intf_vsys_enable(request->intf_id);
    if (!rc) {
        response->result_code = GB_SVC_INTF_VSYS_OK;
    } else {
        response->result_code = GB_SVC_INTF_VSYS_FAIL;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_intf_vsys_disable(struct gb_operation *operation)
{
    struct gb_svc_intf_vsys_enable_request *request;
    struct gb_svc_intf_vsys_enable_response *response;
    int rc;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    request = gb_operation_get_request_payload(operation);

    rc = svc_intf_vsys_disable(request->intf_id);
    if (!rc) {
        response->result_code = GB_SVC_INTF_VSYS_OK;
    } else {
        response->result_code = GB_SVC_INTF_VSYS_FAIL;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_intf_refclk_enable(struct gb_operation *operation)
{
    struct gb_svc_intf_refclk_enable_request *request;
    struct gb_svc_intf_refclk_enable_response *response;
    int rc;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    request = gb_operation_get_request_payload(operation);

    rc = svc_intf_refclk_enable(request->intf_id);
    if (!rc) {
        response->result_code = GB_SVC_INTF_REFCLK_OK;
    } else {
        response->result_code = GB_SVC_INTF_REFCLK_FAIL;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_intf_refclk_disable(struct gb_operation *operation)
{
    struct gb_svc_intf_refclk_enable_request *request;
    struct gb_svc_intf_refclk_enable_response *response;
    int rc;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    request = gb_operation_get_request_payload(operation);

    rc = svc_intf_refclk_disable(request->intf_id);
    if (!rc) {
        response->result_code = GB_SVC_INTF_REFCLK_OK;
    } else {
        response->result_code = GB_SVC_INTF_REFCLK_FAIL;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_intf_unipro_enable(struct gb_operation *operation)
{
    struct gb_svc_intf_unipro_enable_request *request;
    struct gb_svc_intf_unipro_enable_response *response;
    int rc;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    request = gb_operation_get_request_payload(operation);

    rc = svc_intf_unipro_enable(request->intf_id);
    if (!rc) {
        response->result_code = GB_SVC_INTF_UNIPRO_OK;
    } else {
        response->result_code = GB_SVC_INTF_UNIPRO_FAIL;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_intf_unipro_disable(struct gb_operation *operation)
{
    struct gb_svc_intf_unipro_enable_request *request;
    struct gb_svc_intf_unipro_enable_response *response;
    int rc;

    if (gb_operation_get_request_payload_size(operation) < sizeof(*request)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response) {
        return GB_OP_NO_MEMORY;
    }

    request = gb_operation_get_request_payload(operation);

    rc = svc_intf_unipro_disable(request->intf_id);
    if (!rc) {
        response->result_code = GB_SVC_INTF_UNIPRO_OK;
    } else {
        response->result_code = GB_SVC_INTF_UNIPRO_FAIL;
    }

    return GB_OP_SUCCESS;
}

static uint8_t gb_svc_pwr_down(struct gb_operation *op) {
    return gb_errno_to_op_result(svcd_power_down());
}

static uint8_t gb_svc_timesync_enable(struct gb_operation *op) {
    struct gb_svc_timesync_enable_request *req;
    int rc;
    uint8_t count;
    uint64_t frame_time;
    uint32_t strobe_delay;
    uint32_t refclk;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);

    count = req->count;
    frame_time = le64_to_cpu(req->frame_time);
    strobe_delay = le32_to_cpu(req->strobe_delay);
    refclk = le32_to_cpu(req->refclk);

    rc = timesync_enable(count, frame_time, strobe_delay, refclk);
    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_timesync_disable(struct gb_operation *op) {
    int rc;

    rc = timesync_disable();
    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_timesync_authoritative(struct gb_operation *op) {
    struct gb_svc_timesync_authoritative_response *resp;
    int rc, i;

    resp = gb_operation_alloc_response(op, sizeof(*resp));
    if (!resp) {
        return GB_OP_NO_MEMORY;
    }

    rc = timesync_authoritative((uint64_t*)&resp->frame_time);
    if (rc == 0) {
        for (i = 0; i < GB_TIMESYNC_MAX_STROBES; i++)
            resp->frame_time[i] = cpu_to_le64(resp->frame_time[i]);
    }
    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_timesync_ping(struct gb_operation *op) {
    struct gb_svc_timesync_ping_response *resp;
    uint64_t frame_time;
    int rc;

    resp = gb_operation_alloc_response(op, sizeof(*resp));
    if (!resp) {
        return GB_OP_NO_MEMORY;
    }

    rc = timesync_ping(&frame_time);
    if (rc == 0) {
        resp->frame_time = cpu_to_le64(frame_time);
    }
    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_timesync_wd_pins_init(struct gb_operation *op) {
    struct gb_svc_timesync_wd_pins_init_request *req;
    uint32_t strobe_mask;
    int rc;

    if (gb_operation_get_request_payload_size(op) < sizeof(*req)) {
        gb_error("dropping short message\n");
        return GB_OP_INVALID;
    }

    req = gb_operation_get_request_payload(op);
    strobe_mask = le32_to_cpu(req->strobe_mask);
    rc = timesync_wd_pins_init(strobe_mask);
    return gb_errno_to_op_result(rc);
}

static uint8_t gb_svc_timesync_wd_pins_fini(struct gb_operation *op) {
    int rc;

    rc = timesync_wd_pins_fini();
    return gb_errno_to_op_result(rc);
}

static struct gb_operation_handler gb_svc_handlers[] = {
    GB_HANDLER(GB_SVC_TYPE_INTF_DEVICE_ID, gb_svc_intf_device_id),
    GB_HANDLER(GB_SVC_TYPE_INTF_EJECT, gb_svc_intf_eject),
    GB_HANDLER(GB_SVC_TYPE_CONN_CREATE, gb_svc_connection_create),
    GB_HANDLER(GB_SVC_TYPE_CONN_DESTROY, gb_svc_connection_destroy),
    GB_HANDLER(GB_SVC_TYPE_ROUTE_CREATE, gb_svc_route_create),
    GB_HANDLER(GB_SVC_TYPE_ROUTE_DESTROY, gb_svc_route_destroy),
    GB_HANDLER(GB_SVC_TYPE_DME_PEER_GET, gb_svc_dme_peer_get),
    GB_HANDLER(GB_SVC_TYPE_DME_PEER_SET, gb_svc_dme_peer_set),
    GB_HANDLER(GB_SVC_TYPE_INTF_SET_PWRM, gb_svc_intf_set_power_mode),
    GB_HANDLER(GB_SVC_TYPE_PING, gb_svc_ping),
    GB_HANDLER(GB_SVC_TYPE_INTF_VSYS_ENABLE, gb_svc_intf_vsys_enable),
    GB_HANDLER(GB_SVC_TYPE_INTF_VSYS_DISABLE, gb_svc_intf_vsys_disable),
    GB_HANDLER(GB_SVC_TYPE_INTF_REFCLK_ENABLE, gb_svc_intf_refclk_enable),
    GB_HANDLER(GB_SVC_TYPE_INTF_REFCLK_DISABLE, gb_svc_intf_refclk_disable),
    GB_HANDLER(GB_SVC_TYPE_INTF_UNIPRO_ENABLE, gb_svc_intf_unipro_enable),
    GB_HANDLER(GB_SVC_TYPE_INTF_UNIPRO_DISABLE, gb_svc_intf_unipro_disable),
    GB_HANDLER(GB_SVC_TYPE_TIMESYNC_ENABLE, gb_svc_timesync_enable),
    GB_HANDLER(GB_SVC_TYPE_TIMESYNC_DISABLE, gb_svc_timesync_disable),
    GB_HANDLER(GB_SVC_TYPE_TIMESYNC_AUTHORITATIVE, gb_svc_timesync_authoritative),
    GB_HANDLER(GB_SVC_TYPE_TIMESYNC_PING, gb_svc_timesync_ping),
    GB_HANDLER(GB_SVC_TYPE_TIMESYNC_WD_PINS_INIT, gb_svc_timesync_wd_pins_init),
    GB_HANDLER(GB_SVC_TYPE_TIMESYNC_WD_PINS_FINI, gb_svc_timesync_wd_pins_fini),
    GB_HANDLER(GB_SVC_TYPE_PWRMON_RAIL_COUNT_GET, gb_svc_pwrmon_rail_count_get),
    GB_HANDLER(GB_SVC_TYPE_PWRMON_RAIL_NAMES_GET, gb_svc_pwrmon_rail_names_get),
    GB_HANDLER(GB_SVC_TYPE_PWRMON_SAMPLE_GET, gb_svc_pwrmon_sample_get),
    GB_HANDLER(GB_SVC_TYPE_PWRMON_INTF_SAMPLE_GET, gb_svc_pwrmon_intf_sample_get),
    GB_HANDLER(GB_SVC_TYPE_PWR_DOWN, gb_svc_pwr_down),
};

struct gb_driver svc_driver = {
    .op_handlers = (struct gb_operation_handler*) gb_svc_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_svc_handlers),
};

void gb_svc_register(int cport) {
    g_svc_cport = cport;
    gb_register_driver(cport, -1 /* dummy bundle id */, &svc_driver);
}
