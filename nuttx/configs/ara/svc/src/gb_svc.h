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

#ifndef _GB_SVC_H_
#define _GB_SVC_H_

#include <nuttx/greybus/types.h>

/* SVC IDs */
#define GB_SVC_CPORT_ID                         0x00
#define GB_SVC_DEVICE_ID                        0x01

/* Version of the Greybus SVC protocol we support */
#define GB_SVC_VERSION_MAJOR                    0x00
#define GB_SVC_VERSION_MINOR                    0x01

/* Greybus SVC request types */
#define GB_SVC_TYPE_INVALID                     0x00
#define GB_SVC_TYPE_PROTOCOL_VERSION            0x01
#define GB_SVC_TYPE_HELLO                       0x02
#define GB_SVC_TYPE_INTF_DEVICE_ID              0x03
#define GB_SVC_TYPE_INTF_HOTPLUG                0x04
#define GB_SVC_TYPE_INTF_HOT_UNPLUG             0x05
#define GB_SVC_TYPE_INTF_RESET                  0x06
#define GB_SVC_TYPE_CONN_CREATE                 0x07
#define GB_SVC_TYPE_CONN_DESTROY                0x08
#define GB_SVC_TYPE_DME_PEER_GET                0x09
#define GB_SVC_TYPE_DME_PEER_SET                0x0a
#define GB_SVC_TYPE_ROUTE_CREATE                0x0b
#define GB_SVC_TYPE_ROUTE_DESTROY               0x0c
#define GB_SVC_TYPE_INTF_SET_PWRM               0x10
#define GB_SVC_TYPE_INTF_EJECT                  0x11
#define GB_SVC_TYPE_KEY_EVENT                   0x12
#define GB_SVC_TYPE_PING                        0x13
#define GB_SVC_TYPE_PWRMON_RAIL_COUNT_GET       0x14
#define GB_SVC_TYPE_PWRMON_RAIL_NAMES_GET       0x15
#define GB_SVC_TYPE_PWRMON_SAMPLE_GET           0x16
#define GB_SVC_TYPE_PWRMON_INTF_SAMPLE_GET      0x17
#define GB_SVC_TYPE_INTF_PWR_ENABLE             0x18
#define GB_SVC_TYPE_INTF_REFCLK_ENABLE          0x19
#define GB_SVC_TYPE_PWR_DOWN                    0x1d


struct gb_svc_protocol_version_request {
    __u8        major;
    __u8        minor;
} __packed;

struct gb_svc_protocol_version_response {
    __u8        major;
    __u8        minor;
} __packed;

struct gb_svc_route_create_request {
    __u8        intf1_id;
    __u8        dev1_id;
    __u8        intf2_id;
    __u8        dev2_id;
} __packed;

struct gb_svc_route_destroy_request {
    __u8        intf1_id;
    __u8        intf2_id;
} __packed;

struct gb_svc_hello_request {
    __le16      endo_id;
    __u8        interface_id;
} __packed;

struct gb_svc_intf_device_id_request {
    __u8        intf_id;
    __u8        device_id;
} __packed;
/* device id response has no payload */

struct gb_svc_intf_hotplug_request {
    __u8 intf_id;
    struct {
        __le32  ddbl1_mfr_id;
        __le32  ddbl1_prod_id;
        __le32  ara_vend_id;
        __le32  ara_prod_id;
	__le64  serial_number;
    } data;
} __packed;
/* hotplug response has no payload */

struct gb_svc_intf_hot_unplug_request {
    __u8        intf_id;
} __packed;
/* hot unplug response has no payload */

struct gb_svc_intf_reset_request {
    __u8        intf_id;
} __packed;
/* interface reset response has no payload */

struct gb_svc_intf_eject_request {
    __u8        intf_id;
} __packed;
/* interface eject response has no payload */

struct gb_svc_conn_create_request {
    __u8        intf1_id;
    __le16      cport1_id;
    __u8        intf2_id;
    __le16      cport2_id;
    __u8        tc;
    __u8        flags;
} __packed;
/* connection create response has no payload */

struct gb_svc_conn_destroy_request {
    __u8        intf1_id;
    __le16      cport1_id;
    __u8        intf2_id;
    __le16      cport2_id;
} __packed;
/* connection destroy response has no payload */

struct gb_svc_dme_peer_get_request {
    __u8        intf_id;
    __le16      attr;
    __le16      selector;
} __packed;

struct gb_svc_dme_peer_get_response {
    __le16      result_code;
    __le32      attr_value;
} __packed;

struct gb_svc_dme_peer_set_request {
    __u8        intf_id;
    __le16      attr;
    __le16      selector;
    __le32      value;
} __packed;

struct gb_svc_dme_peer_set_response {
    __le16      result_code;
} __packed;

#define GB_SVC_UNIPRO_FAST_MODE         0x01
#define GB_SVC_UNIPRO_SLOW_MODE         0x02
#define GB_SVC_UNIPRO_FAST_AUTO_MODE    0x04
#define GB_SVC_UNIPRO_SLOW_AUTO_MODE    0x05
#define GB_SVC_UNIPRO_MODE_UNCHANGED    0x07
#define GB_SVC_UNIPRO_HIBERNATE_MODE    0x11
#define GB_SVC_UNIPRO_OFF_MODE          0x12

#define GB_SVC_PWRM_RXTERMINATION       0x01
#define GB_SVC_PWRM_TXTERMINATION       0x02
#define GB_SVC_PWRM_LINE_RESET          0x04
#define GB_SVC_PWRM_SCRAMBLING          0x20

#define GB_SVC_PWRM_QUIRK_HSSER         0x00000001

#define GB_SVC_UNIPRO_HS_SERIES_A       0x01
#define GB_SVC_UNIPRO_HS_SERIES_B       0x02

struct gb_svc_intf_set_pwrm_request {
    __u8        intf_id;
    __u8        hs_series;
    __u8        tx_mode;
    __u8        tx_gear;
    __u8        tx_nlanes;
    __u8        rx_mode;
    __u8        rx_gear;
    __u8        rx_nlanes;
    __u8        flags;
    __le32      quirks;
} __packed;

struct gb_svc_intf_set_pwrm_response {
    __le16      result_code;
} __packed;

struct gb_svc_key_event_request {
    __le16	key_code;
#define GB_KEYCODE_ARA          0x00

    __u8	key_event;
#define GB_SVC_KEY_RELEASED	0x00
#define GB_SVC_KEY_PRESSED	0x01
} __packed;

struct gb_svc_pwrmon_rail_count_get_response {
    __u8        rail_count;
} __packed;

#define GB_SVC_PWRMON_RAIL_NAME_BUFSIZE 32

#define GB_SVC_PWRMON_TYPE_CURR         0x01
#define GB_SVC_PWRMON_TYPE_VOL          0x02
#define GB_SVC_PWRMON_TYPE_PWR          0x03

#define GB_SVC_PWRMON_GET_SAMPLE_OK     0x00
#define GB_SVC_PWRMON_GET_SAMPLE_INVAL  0x01
#define GB_SVC_PWRMON_GET_SAMPLE_NOSUPP 0x02
#define GB_SVC_PWRMON_GET_SAMPLE_HWERR  0x03

struct gb_svc_pwrmon_sample_get_request {
    __u8        rail_id;
    __u8        measurement_type;
} __packed;

struct gb_svc_pwrmon_sample_get_response {
    __u8        result;
    __le32      measurement;
} __packed;

struct gb_svc_pwrmon_intf_sample_get_request {
    __u8        intf_id;
    __u8        measurement_type;
} __packed;

struct gb_svc_pwrmon_intf_sample_get_response {
    __u8        result;
    __le32      measurement;
} __packed;

struct gb_svc_intf_pwr_enable_request {
    __u8        intf_id;
    __u8        enable;
#define GB_SVC_INTF_PWR_DISABLE         0x00
#define GB_SVC_INTF_PWR_ENABLE          0x01
} __packed;

struct gb_svc_intf_pwr_enable_response {
    __u8        result_code;
#define GB_SVC_INTF_PWR_OK              0x00
#define GB_SVC_INTF_PWR_BUSY            0x01
#define GB_SVC_INTF_PWR_ERROR_CAP       0x02
#define GB_SVC_INTF_PWR_INSUFFICIENT    0x03
#define GB_SVC_INTF_PWR_FAIL            0x04
} __packed;

struct gb_svc_intf_refclk_enable_request {
    __u8        intf_id;
    __u8        enable;
#define GB_SVC_INTF_REFCLK_DISABLE      0x00
#define GB_SVC_INTF_REFCLK_ENABLE       0x01
} __packed;

struct gb_svc_intf_refclk_enable_response {
    __u8        result_code;
#define GB_SVC_INTF_REFCLK_OK           0x00
#define GB_SVC_INTF_REFCLK_BUSY         0x01
#define GB_SVC_INTF_REFCLK_ERROR_CAP    0x02
#define GB_SVC_INTF_REFCLK_FAIL         0x03
} __packed;

int gb_svc_protocol_version(void);
int gb_svc_hello(uint8_t ap_intf_id);
int gb_svc_intf_hotplug(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint64_t);
int gb_svc_intf_hot_unplug(uint32_t);
int gb_svc_key_event(uint16_t, uint8_t);
void gb_svc_register(int cport);

#endif
