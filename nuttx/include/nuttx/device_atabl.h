/**
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
 *
 * @author Kim Mui
 */

#ifndef __INCLUDE_NUTTX_DEVICE_ATABL_H
#define __INCLUDE_NUTTX_DEVICE_ATABL_H

#include <errno.h>

#include <nuttx/util.h>
#include <nuttx/device.h>

#define DEVICE_TYPE_ATABL_HW                 "atabl"

enum device_atabl_error {
    DEVICE_ATABL_ERROR_NONE,
    DEVICE_ATABL_ERROR_INVALID,
    DEVICE_ATABL_ERROR_BAD_DEV,
    DEVICE_ATABL_ERROR_BAD_REQ,
    DEVICE_ATABL_ERROR_BAD_CPORTID,
    DEVICE_ATABL_ERROR_REQ_BUSY,
    DEVICE_ATABL_ERROR_REQ_NOT_CONNECTED,
    DEVICE_ATABL_ERROR_REQ_INVALID_STATE,
    DEVICE_ATABL_ERROR_NO_REQ_AVAIL,
    DEVICE_ATABL_ERROR_NO_MEMORY,
    DEVICE_ATABL_ERROR_UNKOWN_FAILURE = 0xff,
};

struct device_atabl_type_ops {
    int (*req_free_count)(struct device *dev);
    int (*req_alloc)(struct device *dev, void **req);
    int (*req_free)(struct device *dev, void *req);
    int (*connect_cport_to_req)(struct device *dev, unsigned int cportid,
            void *req);
    int (*disconnect_cport_from_req)(struct device *dev, void *req);
    int (*activate_req)(struct device *dev, void *req);
    int (*deactivate_req)(struct device *dev, void *req);
    int (*transfer_completed)(struct device *dev, void *req);
    int (*req_is_activated)(struct device *dev, void *req);
    int (*req_to_peripheral_id)(struct device *dev, void *req);
};

/**
 * @brief Get number of unallocated ATABL REQn
 * @param dev ATABL device whose number of free ATABL REQn is queried
 * @return 'n': Number of unallocated/free ATABL REQn
 *         -errno: Cause of failure
 */
static inline int device_atabl_req_free_count(struct device *dev) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->req_free_count)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->req_free_count(dev);

    return -ENOSYS;
}

/**
 * @brief Allocate a ATABL REQn
 * @param dev ATABL device whose REQn is being allocated
 * @param req Pointer to ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_req_alloc(struct device *dev, void **req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->req_alloc)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->req_alloc(dev, req);

    return -ENOSYS;
}

/****
 *
 * @brief Free a ATABL REQn
 * @param dev ATABL device whose REQn is being freed
 * @param req ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_req_free(struct device *dev, void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->req_free)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->req_free(dev, req);

    return -ENOSYS;
}

/**
 * @brief Connect a CPort to an ATABL REQn
 * @param dev ATABL device
 * @param cportid cport identifier
 * @param req A point to an ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_connect_cport_to_req(struct device *dev,
        unsigned int cportid, void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->connect_cport_to_req)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->connect_cport_to_req(dev,
                cportid, req);

    return -ENOSYS;
}

/**
 * @brief Disconnect a CPort from an ATABL REQn
 * @param dev ATABL device
 * @param req A point to an ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_disconnect_cport_from_req(struct device *dev,
        void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->disconnect_cport_from_req)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->disconnect_cport_from_req(dev,
                req);

    return -ENOSYS;
}

/**
 * @brief Activate an ATABL REQn
 * @param dev ATABL device
 * @param req A point to an ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_activate_req(struct device *dev, void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->activate_req)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->activate_req(dev, req);

    return -ENOSYS;
}

/**
 * @brief Notify ATABL driver a data transfer operation is completed
 * @param dev ATABL device
 * @param req A point to an ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_deactivate_req(struct device *dev, void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->deactivate_req)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->deactivate_req(dev, req);

    return -ENOSYS;
}

/**
 * @brief Notify ATABL driver a data transfer operation is completed
 * @param dev ATABL device
 * @param req A point to an ATABL REQn cookie
 * @return 0: Success
 *         -errno: Cause of failure
 */
static inline int device_atabl_transfer_completed(struct device *dev,
        void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->transfer_completed)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->transfer_completed(dev, req);

    return -ENOSYS;
}

/**
 * @brief Check if an ATABL REQn is activated
 * @param dev ATABL device
 * @param req A point to an ATABL REQn cookie
 * @return 0: REQn is not activated
 *         1: REQn is activated
 *         -errno: Cause of failure
 */
static inline int device_atabl_req_is_activated(struct device *dev, void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->req_is_activated)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->req_is_activated(dev, req);

    return -ENOSYS;
}

/**
 * @brief get REQn's peripheral ID
 * @param dev ATABL device
 * @param req A point to an ATABL REQn cookie
 * @return REQn peripheral ID
 *         -errno: Cause of failure
 */
static inline int device_atabl_req_to_peripheral_id(struct device *dev,
        void *req) {
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev))
        return -ENODEV;

    if (DEVICE_DRIVER_GET_OPS(dev, atabl)->req_to_peripheral_id)
        return DEVICE_DRIVER_GET_OPS(dev, atabl)->req_to_peripheral_id(dev, req);

    return -ENOSYS;
}

#endif /* __INCLUDE_NUTTX_DEVICE_ATABL_H */
