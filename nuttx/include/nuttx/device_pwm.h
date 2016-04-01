/**
 * Copyright (c) 2015-2016 Google, Inc.
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

#ifndef __INCLUDE_NUTTX_DEVICE_PWM_H
#define __INCLUDE_NUTTX_DEVICE_PWM_H

/**
 * @file nuttx/device_pwm.h
 * @brief PWM API
 * @attention This file is officially included in the Firmware Documentation.
 * Please contact the Firmware Documentation team before modifying it.
 */

#include <errno.h>
#include <stdbool.h>

#include <nuttx/device.h>
#include <nuttx/util.h>

/** PWM Device type */
#define DEVICE_TYPE_PWM_HW                 "pwm"

/** PWM output modes */
enum pwm_mode {
    /** mode 0: use a set pulse width duration */
    PWM_PULSECOUNT_MODE,
    /** mode 1: signal is static high or low when frequency is 0 */
    PWM_STOP_LEVEL_MODE,
    /** mode 2: start PWM generators with same frequency simultaneously */
    PWM_SYNC_MODE,
};

/**
 * @brief PWM event callback function
 *
 * This callback function is registered with the PWM controller and is called
 * whenever one of the selected interrupt events occur.
 *
 * @param mask_int Mask of selected interrupt events
 * @param mask_err Mask of selected error events
 */
typedef void (*pwm_event_callback)(uint32_t mask_int, uint32_t mask_err);

/** PWM device driver operations */
struct device_pwm_type_ops {
    /** Get the number of supported PWM generators
     * @param dev Pointer to the PWM device controller
     * @param count Pointer to a variable whose is to be filled out with the
     * number of supported PWM generators
     * @return 0 on success, negative errno on failure
     */
    int (*count)(struct device *dev, uint16_t *count);
    /** Initialize a PWM generator
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @return 0 on success, negative errno on failure
     */
    int (*activate)(struct device *dev, uint16_t which);
    /** Deinitialize a PWM generator
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @return 0 on success, negative errno on failure
     */
    int (*deactivate)(struct device *dev, uint16_t which);
    /** Set the period and duty cycle of a PWM generator
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @param duty Duty cycle of PWM generator in nanoseconds
     * @param period Period of PWM generator in nanoseconds
     * @return 0 on success, negative errno on failure
     */
    int (*config)(struct device *dev, uint16_t which, uint32_t duty,
                  uint32_t period);
    /** Enable the pulse width output of a PWM generator
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @return 0 on success, negative errno on failure
     */
    int (*enable)(struct device *dev, uint16_t  which);
    /** Disable the pulse width output of a PWM generator
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @return 0 on success, negative errno on failure
     */
    int (*disable)(struct device *dev, uint16_t  which);
    /** Set polarity of PWM generator active signal
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @param polarity Polarity of active signal. True for inverted, false for
     * normal
     * @return 0 on success, negative errno on failure
     */
    int (*set_polarity)(struct device *dev, uint16_t  which, bool polarity);
    /** Set output mode of PWM generator
     * @param dev Pointer to the PWM device controller
     * @param which PWM generator device number
     * @param mode PWM generator output mode
     * @param param For mode 0: number of pulses; For mode 1: true for high
     * level, false for low level; For mode 2: true to enable sync
     * @return 0 on success, negative errno on failure
     */
    int (*set_mode)(struct device *dev, uint16_t  which, enum pwm_mode mode,
                    void *param);
    /** Start PWM generators with same frequency simultaneously
     * @param dev Pointer to the PWM device controller
     * @param enable True for start, false for stop
     * @return 0 on success, negative errno on failure
     */
    int (*sync_output)(struct device *dev, bool enable);
    /** Register callback function to be notified when one of the selected
     * interrupt events occur
     * @param dev Pointer to the PWM device controller
     * @param mask_int Mask of selected interrupt events
     * @param mask_err Mask of selected error events
     * @param mode Interrupt mode (0: end of cycle interrupts, 1: update
     * interrupts)
     * @param callback The callback function to be called on an event, when
     * NULL deregister the existing callback for the specified masks of events
     * @return 0 on success, negative errno on failure
     */
    int (*register_callback)(struct device *dev, uint32_t mask_int,
                             uint32_t mask_err, uint32_t mode,
                             pwm_event_callback callback);
};

/** Get number of PWM generators supported by driver
 * @param dev Pointer to the PWM device controller
 * @param count Pointer to variable to which number of PWM generators
 * will be assigned.
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_get_count(struct device *dev, uint16_t *count)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->count) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->count(dev, count);
}

/** Initialize a PWM generator
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_activate(struct device *dev, uint16_t which)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->activate) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->activate(dev, which);
}

/** Deinitialize a PWM generator
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_deactivate(struct device *dev, uint16_t which)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->deactivate) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->deactivate(dev, which);
}

/** Set the period and duty cycle of a PWM generator
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @param duty Duty cycle of PWM generator in nanoseconds
 * @param period Period of PWM generator in nanoseconds
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_config(struct device *dev, uint16_t which,
                                    uint32_t duty, uint32_t period)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->config) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->config(dev, which, duty, period);
}

/** Set polarity of PWM generator active signal
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @param polarity Polarity of active signal. True for inverted, false for
 * normal
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_set_polarity(struct device *dev, uint16_t which,
                                          bool polarity)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->set_polarity) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->set_polarity(dev, which, polarity);
}

/** Enable the pulse width output of a PWM generator
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_enable(struct device *dev, uint16_t which)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->enable) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->enable(dev, which);
}

/** Disable the pulse width output of a PWM generator
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_disable(struct device *dev, uint16_t which)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->disable) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->disable(dev, which);
}

/** Set output mode of PWM generator
 * @param dev Pointer to the PWM device controller
 * @param which PWM generator device number
 * @param mode PWM generator output mode
 * @param param For mode 0: number of pulses; For mode 1: true for high level,
 * false for low level; For mode 2: true to start sync
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_set_mode(struct device *dev, uint16_t which,
                                      enum pwm_mode mode, void *param)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->set_mode) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->set_mode(dev, which, mode, param);
}

/** Start PWM generators with same frequency simultaneously
 * @param dev Pointer to the PWM device controller
 * @param enable True for start, false for stop
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_sync_output(struct device *dev, bool enable)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->sync_output) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->sync_output(dev, enable);
}

/** Register callback function to be notified when one of the selected
 * interrupt events occur
 * @param dev Pointer to the PWM device controller
 * @param mask_int Mask of selected interrupt events
 * @param mask_err Mask of selected error events
 * @param mode Interrupt mode (0: end of cycle interrupts, 1: update interrupts)
 * @param callback The callback function to be called on an event, when NULL
 * deregister the existing callback for the specified masks of events
 * @return 0 on success, negative errno on failure
 */
static inline int device_pwm_register_callback(struct device *dev,
                                               uint32_t mask_int,
                                               uint32_t mask_err,
                                               uint32_t mode,
                                               pwm_event_callback callback)
{
    DEVICE_DRIVER_ASSERT_OPS(dev);

    if (!device_is_open(dev)) {
        return -ENODEV;
    }

    if (!DEVICE_DRIVER_GET_OPS(dev, pwm)->register_callback) {
        return -ENOSYS;
    }

    return DEVICE_DRIVER_GET_OPS(dev, pwm)->register_callback(dev,
                                                              mask_int,
                                                              mask_err,
                                                              mode,
                                                              callback);
}

#endif
