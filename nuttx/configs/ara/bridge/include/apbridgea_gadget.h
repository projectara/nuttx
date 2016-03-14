/*
 * Copyright (c) 2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#ifndef _APBRIDGEA_GADGET_H_
#define _APBRIDGEA_GADGET_H_

#include <sys/types.h>
#include <arch/board/common_gadget.h>

/* Vender specific control requests *******************************************/

#define APBRIDGE_RWREQUEST_LOG                      (0x02)
#define APBRIDGE_RWREQUEST_EP_MAPPING               (0x03)
#define APBRIDGE_ROREQUEST_CPORT_COUNT              (0x04)
#define APBRIDGE_WOREQUEST_CPORT_RESET              (0x05)
#define APBRIDGE_ROREQUEST_LATENCY_TAG_EN           (0x06)
#define APBRIDGE_ROREQUEST_LATENCY_TAG_DIS          (0x07)
#define APBRIDGE_RWREQUEST_CSI_TX_CONTROL           (0x08)
#define APBRIDGE_RWREQUEST_AUDIO_APBRIDGEA          (0x09)
#define APBRIDGE_WOREQUEST_SET_REQUEST_COUNT        (0x0a)
#define APBRIDGE_WOREQUEST_CPORT_ENA_FCT_TX_FLOW    (0x0b)
#define APBRIDGE_WOREQUEST_CPORT_DIS_FCT_TX_FLOW    (0x0c)
#define APBRIDGE_WOREQUEST_TIMESYNC_ENABLE          (0x0d)
#define APBRIDGE_WOREQUEST_TIMESYNC_DISABLE         (0x0e)
#define APBRIDGE_WOREQUEST_TIMESYNC_AUTHORITATIVE   (0x0f)
#define APBRIDGE_ROREQUEST_TIMESYNC_GET_LAST_EVENT  (0x10)

struct apbridge_dev_s;

enum ep_mapping {
    MULTIPLEXED_EP,
    DIRECT_EP,
};

struct apbridge_dev_s *get_apbridge_dev(void);

int unipro_to_usb(struct apbridge_dev_s *dev, unsigned cportid,
                  const void *payload, size_t size);

void usb_wait(struct apbridge_dev_s *dev);
int usbdev_apbinitialize(struct device *dev);

int usb_release_buffer(struct apbridge_dev_s *priv, const void *buf);

/*
 * Offloaded cport
 * Give ability to APBridgeA to use a cport without AP involvment.
 * When a cport id offloaded, data comming from UniPro or USB will
 * be redirected to callbacks.
 */

/* Offloaded cport callback definition */
typedef int (*unipro_offloaded_cb)(unsigned int cportid,
                                   void *payload, size_t len);

/*
 * @brief Offload a cport
 * @param priv Pointer on usb gadget device.
 * @param cportid ID of cport to offload.
 * @param unipro_cb The callback to invoke when there is incoming data from
 * UniPro. It is the responsability of callback to release the UniPro buffer
 * using unipro_rxbuf_free().
 * @param usb_cb The callback to invoke when there is incoming data from USB.
 * It is the responsability of callback to release the USB buffer using
 * usb_release_buffer().
 * @return 0 on success or -EBUSY is the cport is already offloaded or mapped
 * to a dirrect mapped endpoint.
 */
int map_offloaded_cport(struct apbridge_dev_s *priv, unsigned int cportid,
                         unipro_offloaded_cb unipro_cb,
                         unipro_offloaded_cb usb_cb);
/*
 * @brief Un-offload a cport
 * @param priv Pointer on usb gadget device.
 * @param cportid ID of cport to un-offload.
 * @return 0 on success or -EINVAL is the cport is not offloaded.
 */
int unmap_offloaded_cport(struct apbridge_dev_s *priv, unsigned int cportid);

#endif /* _APBRIDGEA_GADGET_H_ */
