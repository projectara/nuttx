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

#ifndef __INCLUDE_NUTTX_DEVICE_H
#define __INCLUDE_NUTTX_DEVICE_H

/**
 * @file nuttx/device.h
 * @brief Device API
 * @author Mark Greer
 */

#include <assert.h>
#include <stdint.h>

/** Device resource types */
enum device_resource_type {
    /** Invalid resource (forces device resources to define a type) */
    DEVICE_RESOURCE_TYPE_INVALID,
    /** Address in a register map */
    DEVICE_RESOURCE_TYPE_REG,
    /** IRQ number */
    DEVICE_RESOURCE_TYPE_IRQ,
    /** GPIO number */
    DEVICE_RESOURCE_TYPE_GPIO,
    /** I2C address */
    DEVICE_RESOURCE_TYPE_I2C_ADDR,
};

/** Device resource */
struct device_resource {
    /** Resource's name */
    char *name;
    /** Resource's type */
    enum device_resource_type type;
    /** Resource's start value */
    uint32_t start;
    /** Count of such resource */
    unsigned int count;
};

/** Device states */
enum device_state {
    DEVICE_STATE_REMOVED,   /**< Device is removed */
    DEVICE_STATE_PROBING,   /**< Device is being probed */
    DEVICE_STATE_PROBED,    /**< Device is probed */
    DEVICE_STATE_OPENING,   /**< Device is being opened */
    DEVICE_STATE_OPEN,      /**< Device is open */
    DEVICE_STATE_CLOSING,   /**< Device is being closed */
    DEVICE_STATE_REMOVING,  /**< Device is being removed */
};

struct device;

/** Device driver operations */
struct device_driver_ops {
    /** Probe the device ``dev``; Return 0 on success, !=0 on error */
    int (*probe)(struct device *dev);
    /** Remove the device ``dev`` */
    void (*remove)(struct device *dev);
    /** Open the device ``dev``; Return 0 on success, !=0 on error */
    int (*open)(struct device *dev);
    /** Close the device ``dev`` */
    void (*close)(struct device *dev);
    /** Device type operations */
    void *type_ops;
};

/** Device driver structure */
struct device_driver {
    /** Device driver's type */
    char *type;
    /** Device driver's name */
    char *name;
    /** Device driver's full description */
    char *desc;
    /** Device driver's operations */
    struct device_driver_ops *ops;
    /** Device driver's private data */
    void *priv;
};

/** Device structure */
struct device {
    /** Device's type */
    char *type;
    /** Device's name */
    char *name;
    /** Device's full description */
    char *desc;
    /** Device's identifier */
    unsigned int id;
    /** Array of resources associated to the device */
    struct device_resource *resources;
    /** Total number of the device's resources */
    unsigned int resource_count;
    /** Device's initialization data */
    void *init_data;
    /** Device's internal state device_state */
    enum device_state state;
    /** Attached device driver */
    struct device_driver *driver;
    /** Device's private data */
    void *priv;
};

/**
 * @brief Open a device by its type and identifier
 * @param type The type of the device to open
 * @param id The identifier of the device to open
 * @return The specified device on success, NULL on failure
 */
struct device *device_open(char *type, unsigned int id);

/**
 * @brief Close a device
 * @param dev The device to close
 */
void device_close(struct device *dev);

/**
 * @brief Register a device driver
 * @param driver The device driver to register
 * @return 0 on success, !=0 on failure
 */
int device_register_driver(struct device_driver *driver);

/**
 * @brief Unregister a device driver
 * @param driver The device driver to unregister
 */
void device_unregister_driver(struct device_driver *driver);

/**
 * @brief Get a device's resource by its type and number
 * @param dev The device containing the requested resource
 * @param type The device resource's type
 * @param num The number of the device resource
 * @return The specified resource on success, NULL on failure
 */
struct device_resource *device_resource_get(struct device *dev,
                                            enum device_resource_type type,
                                            unsigned int num);

/**
 * @brief Get a device's resource by its type and name
 * @param dev The device containing the requested resource
 * @param type The device resource's type
 * @param name The name of the device resource
 * @return The specified resource on success, NULL on failure
 */
struct device_resource *device_resource_get_by_name(
                                                 struct device *dev,
                                                 enum device_resource_type type,
                                                 char *name);
/**
 * @brief Get the type of a device
 * @param dev The device whose type to return
 * @return The type of the specified device
 */
static inline char *device_get_type(struct device *dev)
{
    return dev->type;
}

/**
 * @brief Get the name of a device
 * @param dev The device whose name to return
 * @return The name of the specified device
 */
static inline char *device_get_name(struct device *dev)
{
    return dev->name;
}

/**
 * @brief Get the description of a device
 * @param dev The device whose description to return
 * @return The description of the specified device
 */
static inline char *device_get_desc(struct device *dev)
{
    return dev->desc;
}

/**
 * @brief Get the identifier of a device
 * @param dev The device whose identifier to return
 * @return The identifier of the specified device
 */
static inline unsigned int device_get_id(struct device *dev)
{
    return dev->id;
}

/**
 * @brief Get the initialization data of a device
 * @param dev The device whose initialization data to return
 * @return The initialization data of the specified device
 */
static inline void *device_get_init_data(struct device *dev)
{
    return dev->init_data;
}

/**
 * @brief Set the initialization data of a device
 * @param dev The device whose initialization data to set
 * @param init_data The initialization data to associate with the specified
 * device
 */
static inline void device_set_init_data(struct device *dev, void *init_data)
{
    dev->init_data = init_data;
}

/**
 * @brief Get the state of a device
 * @param dev The device whose internal state to return
 * @return The state of the specified device
 */
static inline enum device_state device_get_state(struct device *dev)
{
    return dev->state;
}

/**
 * @brief Return whether a device is open or not
 * @param dev The device whose state to test
 * @return 1 if the specified device is open, 0 otherwise
 */
static inline int device_is_open(struct device *dev)
{
    return device_get_state(dev) == DEVICE_STATE_OPEN;
}

/**
 * @brief Get the number of resources associate with a device
 * @param dev The device whose resource count to return
 * @return The number of resources of the specified device
 */
static inline unsigned int device_get_resource_count(struct device *dev)
{
    return dev->resource_count;
}

/**
 * @brief Get the private data of a device
 * @param dev The device whose private data to return
 * @return The private data of the specified device
 */
static inline void *device_get_private(struct device *dev)
{
    return dev->priv;
}

/**
 * @brief Set the private data of a device
 * @param dev The device whose private data to set
 * @param priv The private data to associate with the specified device
 */
static inline void device_set_private(struct device *dev, void *priv)
{
    dev->priv= priv;
}

/**
 * @brief Return whether a device driver is attached to the device
 * @param dev The device whose device driver to test
 * @return 1 if a device driver is attached to the specified device, 0 otherwise
 */
static inline int device_driver_is_attached(struct device *dev)
{
    return !!dev->driver;
}

/**
 * @brief Get the device driver's type of a device
 * @param dev The device whose device driver's type to return
 * @return The device driver's type of the specified device
 */
static inline char *device_driver_get_type(struct device *dev)
{
    return dev->driver->type;
}

/**
 * @brief Get the device driver's name of a device
 * @param dev The device whose device driver's name to return
 * @return The device driver's name of the specified device
 */
static inline char *device_driver_get_name(struct device *dev)
{
    return dev->driver->name;
}

/**
 * @brief Get the device driver's description of a device
 * @param dev The device whose device driver's description to return
 * @return The device driver's description of the specified device
 */
static inline char *device_driver_get_desc(struct device *dev)
{
    return dev->driver->desc;
}

/**
 * @brief Check the entire path from a device to the device type operations of
 * its device driver
 *
 * Check the chain of pointers from the dev pointer to the type_ops pointer to
 * ensure none of them are NULL. If one is NULL, then raise ASSERT if
 * CONFIG_DEBUG is enabled.
 *
 * @param _dev The device on which to perform the check
 */
#define DEVICE_DRIVER_ASSERT_OPS(_dev)                              \
    DEBUGASSERT((_dev) && (_dev)->driver && (_dev)->driver->ops &&  \
                (_dev)->driver->ops->type_ops)

/**
 * @brief Get the device type operations of a device's device driver
 * @param _dev The device whose device type operations to return
 * @param _type The device's type
 * @return The device type operations of the specified device
 */
#define DEVICE_DRIVER_GET_OPS(_dev, _type)  \
    ((struct device_##_type##_type_ops *)((_dev)->driver->ops->type_ops))

/**
 * @brief Get the device driver's private data of a device
 * @param dev The device whose device driver's private data to return
 * @return The device driver's private data of the specified device
 */
static inline void *device_driver_get_private(struct device *dev)
{
    return dev->driver->priv;
}

/**
 * @brief Set the device driver's private data of a device
 * @param dev The device whose device driver's private data to set
 * @param priv The device driver's private data of the specified device
 */
static inline void device_driver_set_private(struct device *dev, void *priv)
{
    dev->driver->priv= priv;
}

/**
 * @brief Get the device resource's name of a device by its index
 * @param dev The device whose device resource's name to return
 * @param idx The index of the device resource
 * @return The device resource's name
 */
static inline char *device_resource_get_name(struct device *dev,
                                             unsigned int idx)
{
    return dev->resources[idx].name;
}

/**
 * @brief Get the device resources's type of a device by its index
 * @param dev The device whose device resource's type to return
 * @param idx The index of the device resource
 * @return The device resource's type
 */
static inline enum device_resource_type device_resource_get_type(
                                        struct device *dev, unsigned int idx)
{
    return dev->resources[idx].type;
}

/**
 * @brief Get the device resource's start value of a device by its index
 * @param dev The device whose device resource's start value to return
 * @param idx The index of the device resource
 * @return The device resource's start value
 */
static inline uint32_t device_resource_get_start(struct device *dev,
                                                 unsigned int idx)
{
    return dev->resources[idx].start;
}

/**
 * @brief Get the device resource's count of a device by its index
 * @param dev The device whose device resource's count to return
 * @param idx The index of the device resource
 * @return The device resource's count
 */
static inline unsigned int device_resource_get_count(struct device *dev,
                                                     unsigned int idx)
{
    return dev->resources[idx].count;
}

#endif /* __INCLUDE_NUTTX_DEVICE_H */
