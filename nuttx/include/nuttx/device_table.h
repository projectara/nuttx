/*
 * Copyright (c) 2015-2016 Google Inc.
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

#ifndef __INCLUDE_NUTTX_DEVICE_TABLE_H
#define __INCLUDE_NUTTX_DEVICE_TABLE_H

/**
 * @file nuttx/device_table.h
 * @brief Device API
 * @author Mark Greer
 * @author Fabien Parent
 * @attention This file is officially included in the Firmware Documentation.
 * Please contact the Firmware Documentation team before modifying it.
 */

#include <nuttx/list.h>

/** Device table */
struct device_table {
    /** List of devices */
    struct device *device;
    /** Number of devices */
    unsigned int device_count;
    /** Internal list for attaching the device table to other device tables */
    struct list_head list;
};

/** Iterator initializator */
#define DEVICE_TABLE_ITER_INITIALIZER {0,}

/** Device table iterator */
struct device_table_iter {
    /** Current device table */
    struct device_table *table;
    /** Current item in the device table */
    unsigned int item;
};

/**
 * @brief Register a device table
 * @param table The device table to be registered
 * @return 0 on success, !=0 on failure
 */
int device_table_register(struct device_table *table);

/**
 * @brief Get the next device from a device table iterator
 * @param iter The device table iterator
 * @return The next device, or NULL when the iterator reaches the end of the
 * list of all the existing devices
 */
struct device *device_table_iter_next(struct device_table_iter *iter);

/** Helper for looping through all the existing devices */
#define device_table_for_each_dev(dev, iter)          \
    while ((dev = device_table_iter_next(iter)))

#endif /* __INCLUDE_NUTTX_DEVICE_TABLE_H */
