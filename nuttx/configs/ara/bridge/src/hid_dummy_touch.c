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

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <nuttx/lib.h>
#include <nuttx/util.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_hid.h>
#include <nuttx/gpio.h>
#include <nuttx/clock.h>

#include <arch/tsb/chip.h>
#include "clock/clock.h"

#define TIP_SWITCH                      BIT(0)  /* finger touched */
#define IN_RANGE                        BIT(1)  /* touch is valid */

#define VENDORID                        0xABCD
#define PRODUCTID                       0x1234

#define REPORT_ID_ZERO                  0   /* Report ID zero is reserved and
                                             * should not be used. */
#define REPORT_ID_MULTITOUCH            1   /* Report ID: Multitouch */
#define REPORT_ID_MAX_CONTACTS          2   /* Report ID: Max fingers config */

#define MAX_VALID_CONTACTS              3   /* max fingers count */
#define TOUCH_DEV_WIDTH                 3840 /* default touch panel width */
#define TOUCH_DEV_HEIGHT                2400 /* default touch panel height */

#define TESTCASE_ONE_FINGER_CLICK       0   /* test case 0: 1-finger click */
#define TESTCASE_ONE_FINGER_SWIPE       1   /* test case 1: 1-finger swipe */
#define TESTCASE_TWO_FINGER_SWIPE       2   /* test case 2: 2-finger swipe */
#define TESTCASE_THREE_FINGER_SWIPE     3   /* test case 3: 3-finger swipe */
#define NUM_OF_TESTCASE                 4

#define MIN_MT_COUNT                    10
#define MAX_MT_COUNT                    100
#define MT_MOVE_STEP                    2
#define CLICK_PRESSURE                  20
#define SWIPE_PRESSURE                  30

#define GPIO_TRIGGER                    9   /* Trigger GPIO pin for testing */
#define DEFAULT_DEBOUNCE_TIME           25  /* 250ms (1 SysTick = 10ms) */
#define TOUCH_SAMPLE_RATE               20000 /* 20ms */

static struct device *touch_dev = NULL;

/**
 * Touch finger data
 */
struct finger {
    /** the finger status:  BIT(0): Tip Switch,  BIT(1): In Range */
    uint8_t state;
    /** contact id */
    uint8_t cid;
    /** x coordinate */
    uint16_t x;
    /** y coordinate */
    uint16_t y;
    /** pressure of the touch contact */
    uint16_t pressure;
} __packed;

/**
 * Touch report data
 */
struct hid_touch_data {
    /** report id */
    uint8_t report_id;
    /** touch finger data array */
    struct finger points[MAX_VALID_CONTACTS];
    /** actual finger count */
    uint8_t actual;
} __packed;

struct touch_info {
    /** Chain to button linking list. */
    struct list_head list;

    uint8_t gpio;

    /** number of fingers supported */
    uint8_t maximum_contacts;

    /** store latest valid keyboard interrupt time */
    uint32_t last_activetime;

    /** store latest valid keyboard state */
    uint8_t last_gpiostate;

    /**
     * GPIO debounce time count
     *
     * Since current SysTick resolution is 10 milliseconds, so the
     * debounce_time unit is 10 milliseconds.
     */
    int debounce_time;

    /** thread handle */
    pthread_t hid_thread;

    /** thread exit flag */
    int abort;

    /** Notifying event thread to start */
    sem_t signal_thread;

    /** multitouch data. for testing */
    struct hid_touch_data data;

    /** start to generate multitouch data. for testing */
    int mt_generate;

    /** multitouch data count. for testing */
    int mt_count;
};

#define LOGICAL_FINGER_COLLECTION \
    0xa1, 0x02,                    /*   COLLECTION (Logical) */\
    0x05, 0x0d,                    /*     USAGE_PAGE (Digitizers) */\
    0x15, 0x00,                    /*     LOGICAL_MINIMUM (0) */\
    0x25, 0x01,                    /*     LOGICAL_MAXIMUM (1) */\
    0x75, 0x01,                    /*     REPORT_SIZE (1) */\
    0x95, 0x02,                    /*     REPORT_COUNT (2) */\
    0x09, 0x42,                    /*     USAGE (Tip Switch) */\
    0x09, 0x32,                    /*     USAGE (In Range) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x95, 0x06,                    /*     REPORT_COUNT (6) */\
    0x81, 0x01,                    /*     INPUT (Cnst,Ary,Abs) */\
    0x75, 0x08,                    /*     REPORT_SIZE (8) */\
    0x09, 0x51,                    /*     USAGE (Contact identifier) */\
    0x95, 0x01,                    /*     REPORT_COUNT (1) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x05, 0x01,                    /*     USAGE_PAGE (Generic Desktop) */\
    0x15, 0x00,                    /*     LOGICAL_MINIMUM (0) */\
    0x75, 0x10,                    /*     REPORT_SIZE (16) */\
    0x09, 0x30,                    /*     USAGE (X) */\
    0x26, 0x00, 0x0f,              /*     LOGICAL_MAXIMUM (3840) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x26, 0x60, 0x09,              /*     LOGICAL_MAXIMUM (2400) */\
    0x09, 0x31,                    /*     USAGE (Y) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0x05, 0x0d,                    /*     USAGE_PAGE (Digitizers) */\
    0x09, 0x30,                    /*     USAGE (Tip Pressure) */\
    0x26, 0xFF, 0x00,              /*     LOGICAL_MAXIMUM (255) */\
    0x81, 0x02,                    /*     INPUT (Data,Var,Abs) */\
    0xc0                           /*   END_COLLECTION */

/**
 * Multitouch HID report descriptor
 */
uint8_t touch_report_desc[] = {
    0x05, 0x0d,                    /* USAGE_PAGE (Digitizers) */
    0x09, 0x04,                    /* USAGE (Touch Screen) */
    0xa1, 0x01,                    /* COLLECTION (Application) */
    0x85, REPORT_ID_MULTITOUCH,    /*   REPORT_ID (multitouch) */
    0x09, 0x22,                    /*   USAGE (Finger) */
    LOGICAL_FINGER_COLLECTION,     /*   1st finger */
    LOGICAL_FINGER_COLLECTION,     /*   2nd finger */
    LOGICAL_FINGER_COLLECTION,     /*   3rd finger */
    0x05, 0x0d,                    /*   USAGE_PAGE (Digitizers) */
    0x09, 0x54,                    /*   USAGE (Contact count) */
    0x95, 0x01,                    /*   REPORT_COUNT (1) */
    0x75, 0x08,                    /*   REPORT_SIZE (8) */
    0x25, MAX_VALID_CONTACTS,      /*   LOGICAL_MAXIMUM (MAX_VALID_CONTACTS) */
    0x81, 0x02,                    /*   INPUT (Data,Var,Abs) */
    0x85, REPORT_ID_MAX_CONTACTS,  /*   REPORT_ID (max_contacts) */
    0x09, 0x55,                    /*   USAGE (Contact count maximum) */
    0xb1, 0x02,                    /*   FEATURE (Data,Var,Abs) */
    0xc0                           /* END_COLLECTION */
};

/**
 * Keyboard HID Device Descriptor
 */
struct hid_descriptor touch_dev_desc = {
    sizeof(struct hid_descriptor), /* hid_descriptor size */
    sizeof(touch_report_desc), /* hid_report_desc size */
    0x0111, /* HID v1.11 compliant */
    PRODUCTID,
    VENDORID,
    0x00, /* no country code */
};

/**
 * report length of each HID Reports in HID Report Descriptor
 *
 * all value is parsed from HID Report Descriptor manually
 */
struct hid_size_info hid_sizeinfo[] =
{
    { .id = REPORT_ID_MULTITOUCH,
      .reports = {
          .size = { 25, 0, 0 }
      }
    },
    { .id = REPORT_ID_MAX_CONTACTS,
      .reports = {
          .size = { 0, 0, 1 }
      }
    },
};

static struct touch_info *touch_get_info(struct device *dev, uint8_t gpio)
{
    struct hid_info *info = device_get_private(dev);
    struct touch_info *dev_info = NULL;
    struct list_head *iter;

    list_foreach(&info->device_list, iter) {
        dev_info = list_entry(iter, struct touch_info, list);
        if (dev_info->gpio == gpio) {
            return dev_info;
        }
    }

    return NULL;
}

/**
 * @brief generate fake test data for multitouch
 *
 * @param info - pointer to structure of touch_info
 * @param testcase - type of multitouch test case
 */
int update_touch_data(struct touch_info *info, uint8_t testcase)
{
    int i = 0, ret = 0;

    if (!info) {
        return -EINVAL;
    }

    if (!info->mt_generate) {
        /* start to generate fake multitouch data */

        /* reset, set all points to null-values */
        memset(&info->data, 0, sizeof(struct hid_touch_data));

        /* first touch packet */
        info->mt_generate = 1;
        info->data.report_id = REPORT_ID_MULTITOUCH;
        switch (testcase) {
            case TESTCASE_ONE_FINGER_CLICK:
                info->data.actual = 1;
                info->data.points[0].state = TIP_SWITCH | IN_RANGE;
                info->data.points[0].cid = 1;
                info->data.points[0].x = rand() % TOUCH_DEV_WIDTH;
                info->data.points[0].y = rand() % TOUCH_DEV_HEIGHT;
                info->data.points[0].pressure = CLICK_PRESSURE;
                info->mt_count = 1;
            break;
            case TESTCASE_ONE_FINGER_SWIPE:
            case TESTCASE_TWO_FINGER_SWIPE:
            case TESTCASE_THREE_FINGER_SWIPE:
                info->data.actual = testcase;
                if (info->data.actual > info->maximum_contacts) {
                    info->data.actual = info->maximum_contacts;
                }
                for (i = 0; i < info->data.actual; i++) {
                    info->data.points[i].state = TIP_SWITCH | IN_RANGE;
                    info->data.points[i].cid = i + 1;
                    /* x = 0 ~ (TOUCH_DEV_WIDTH - 1) */
                    info->data.points[i].x = rand() % TOUCH_DEV_WIDTH;
                    /* y = 0 ~ (TOUCH_DEV_HEIGHT -
                     *         (MAX_MT_COUNT * MT_MOVE_STEP) - 1) */
                    info->data.points[i].y = rand() % (TOUCH_DEV_HEIGHT -
                                             (MAX_MT_COUNT * MT_MOVE_STEP));
                    info->data.points[i].pressure = SWIPE_PRESSURE;
                }
                /* mt_count = MIN_MT_COUNT ~ MAX_MT_COUNT */
                info->mt_count = (rand() % (MAX_MT_COUNT - MIN_MT_COUNT + 1))
                                   + MIN_MT_COUNT;
            break;
            default:
                info->mt_generate = 0;
                ret = -EINVAL;
            break;
        }
    } else {
        /* next touch packet */
        info->mt_count--;
        for (i = 0; i < info->data.actual; i++) {
            if (info->mt_count) {
                info->data.points[i].state = TIP_SWITCH | IN_RANGE;
                info->data.points[i].y += MT_MOVE_STEP;
            } else {
                /* stop to generate touch packet */
                info->data.points[i].state &= ~TIP_SWITCH;
                info->mt_generate = 0;
            }
        }
    }
    return ret;
}

/**
 * @brief Get multi-touch report length
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @return the report size on success, negative errno on error
 */
static int touch_get_report_length(struct device *dev, uint8_t report_type,
                                   uint8_t report_id)
{
    struct hid_info *info = device_get_private(dev);
    struct touch_info *tuh_info = NULL;
    int len = 0, i;

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return -EIO;
    }

    /* lookup the hid_size_info table to find the report size */
    for (i = 0; i < info->num_ids; i++) {
        if (info->sinfo[i].id == report_id) {
            len = info->sinfo[i].reports.size[report_type];
            if (report_id != REPORT_ID_ZERO) {
                len++;
            }
            break;
        }
    }
    return len;
}

/**
 * @brief HID demo thread function
 *
 * @param context - pointer to structure of device data
 */
void touch_thread_func(void *context)
{
    struct device *dev = context;
    struct hid_info *info = device_get_private(dev);
    struct touch_info *tuh_info = NULL;
    uint8_t testcase = 0;

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return;
    }

    /* Initialize random number generator for Multitouch HID demo case */
    srand(time(NULL));

    while (1) {
        /* wait for gpio trigger event */
        sem_wait(&tuh_info->signal_thread);

        if (tuh_info->abort) {
            /* exit hid_thread_func loop */
            break;
        }

        /* send demo touch packet to host */
        do {
            /* prepare touch packet */
            update_touch_data(tuh_info, testcase);

            if (info->event_callback) {
                /* send touch packet to the caller */
                info->event_callback(dev, info->event_data, HID_INPUT_REPORT,
                                     (uint8_t*)&tuh_info->data,
                                     sizeof(struct hid_touch_data));
            }
            usleep(TOUCH_SAMPLE_RATE);
        } while (tuh_info->mt_count > 0);

        /* switch to next testcase */
        testcase = (testcase + 1) % NUM_OF_TESTCASE;
    }
}

/**
 * @brief Get TOUCH Input report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of input buffer size
 * @param len - max input buffer size
 * @return 0 on success, negative errno on error
 */
static int touch_get_input_report(struct device *dev, uint8_t report_id,
                                  uint8_t *data, uint16_t len)
{
    struct touch_info *tuh_info = NULL;

    if (report_id != REPORT_ID_MULTITOUCH) {
        /* For current case, only supports REPORT_ID_MULTITOUCH input report,
         * if report id isn't REPORT_ID_MULTITOUCH, returns error. */
        return -EINVAL;
    }

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return -EIO;
    }

    if (len < sizeof(struct hid_touch_data)) {
        /* no enough buffer space to receive touch data */
        return -ENOBUFS;
    }

    memcpy(data, &tuh_info->data, sizeof(struct hid_touch_data));

    return 0;
}

/**
 * @brief Get TOUCH Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of input buffer size
 * @param len - max input buffer size
 * @return 0 on success, negative errno on error
 */
static int touch_get_feature_report(struct device *dev, uint8_t report_id,
                                    uint8_t *data, uint16_t len)
{
    struct touch_info *tuh_info = NULL;
    int rptlen = 0;

    if (report_id != REPORT_ID_MAX_CONTACTS) {
        /* For current case, only supports REPORT_ID_MAX_CONTACTS feature
         * report, if report id isn't REPORT_ID_MULTITOUCH, returns error. */
        return -EINVAL;
    }

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return -EIO;
    }

    rptlen = touch_get_report_length(dev, HID_FEATURE_REPORT, report_id);

    if (len < rptlen) {
        /* no enough buffer space to receive feature data */
        return -ENOBUFS;
    }

    /* Feature report format (2Bytes): [Report ID][Max Contacts] */
    data[0] = report_id;
    data[1] = tuh_info->maximum_contacts;
    return 0;
}

/**
 * @brief Set HID Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_id - HID report id
 * @param data - pointer of output buffer size
 * @param len - max output buffer size
 * @return 0 on success, negative errno on error
 */
static int touch_set_feature_report(struct device *dev, uint8_t report_id,
                                    uint8_t *data, uint16_t len)
{
    struct touch_info *tuh_info = NULL;
    int rptlen = 0;

    if (report_id != REPORT_ID_MAX_CONTACTS) {
        /* For current case, only supports REPORT_ID_MAX_CONTACTS feature
         * report, if report id isn't REPORT_ID_MULTITOUCH, returns error. */
        return -EINVAL;
    }

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return -EIO;
    }

    rptlen = touch_get_report_length(dev, HID_FEATURE_REPORT, report_id);

    if (len < rptlen) {
        /* no enough buffer space to receive feature data */
        return -ENOBUFS;
    }

    if (data[0] > MAX_VALID_CONTACTS) {
        /* the new value can't larger than maximum contacts which defined in
         * HID report descriptor */
        return -EIO;
    }

    /* Because Greybus HID protocol passed the feature data without report id,
     * so the first byte is maximum contacts field. */
    tuh_info->maximum_contacts = data[0];

    return 0;
}

/**
 * @brief multi-touch interrupt routine
 *
 * @param context - pointer to structure of device data
 */
int touch_irq_event(int irq, void *context, void *priv)
{
    struct device *dev = touch_dev;
    struct touch_info *tuh_info = NULL;
    uint8_t new_gpiostate = 0;
    int elapsed = 0;

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return -EIO;
    }

    elapsed = clock_systimer() - tuh_info->last_activetime;

    if (elapsed >= tuh_info->debounce_time) {
        gpio_irq_mask(irq);
        new_gpiostate = gpio_get_value(GPIO_TRIGGER);

        /* check whether the key state change or not */
        if (tuh_info->last_gpiostate != new_gpiostate) {
            tuh_info->last_gpiostate = new_gpiostate;
            tuh_info->last_activetime = clock_systimer();

            /* notify thread function to send demo report data*/
            if (new_gpiostate) {
                sem_post(&tuh_info->signal_thread);
            }
        }
        gpio_irq_unmask(irq);
    }
    return OK;
}

/**
 * @brief Power-on/off the HID device.
 *
 * @param dev - pointer to structure of device data
 * @param on - true for on, false for off
 * @return 0 on success, negative errno on error
 */
static int touch_power_set(struct device *dev, bool on)
{
    if (on) {
        /* enable interrupt */
        gpio_irq_unmask(GPIO_TRIGGER);
    } else {
        gpio_irq_mask(GPIO_TRIGGER);
    }

    return 0;
}

/**
 * @brief Set HID Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @param data - pointer of output buffer size
 * @param len - max output buffer size
 * @return 0 on success, negative errno on error
 */
static int touch_set_report(struct device *dev, uint8_t report_type,
                            uint8_t report_id, uint8_t *data, uint16_t len)
{
    int ret = 0;

    switch (report_type) {
        case HID_FEATURE_REPORT:
            ret = touch_set_feature_report(dev, report_id, data, len);
        break;
        default:
            /* only support output and feature report */
            ret = -EINVAL;
        break;
    }

    return ret;
}

/**
 * @brief Get HID Input / Feature report data
 *
 * @param dev - pointer to structure of device data
 * @param report_type - HID report type
 * @param report_id - HID report id
 * @param data - pointer of input buffer size
 * @param len - max input buffer size
 * @return 0 on success, negative errno on error
 */
static int touch_get_report(struct device *dev, uint8_t report_type,
                            uint8_t report_id, uint8_t *data, uint16_t len)
{
    int ret = 0;

    switch (report_type) {
        case HID_INPUT_REPORT:
            ret = touch_get_input_report(dev, report_id, data, len);
        break;
        case HID_FEATURE_REPORT:
            ret = touch_get_feature_report(dev, report_id, data, len);
        break;
        default:
            /* only support input and feature report */
            ret = -EINVAL;
        break;
    }

    return ret;
}

/**
 * @brief Configure multi-touch hardware setting
 *
 * @param dev Pointer to structure of device data
 * @param dev_info The pointer for hid_info struct
 *
 * @return 0 on success, negative errno on error
 */
static int touch_hw_initialize(struct device *dev, struct hid_info *dev_info)
{
    int ret = 0;
    struct touch_info *tuh_info = NULL;

    /* check GPIO pin validly*/
    if (GPIO_TRIGGER >= gpio_line_count()) {
        return -EIO;
    }

    tuh_info = zalloc(sizeof(*tuh_info));
    if (!tuh_info) {
        ret = -ENOMEM;
        goto err_hw_init;
    }

    /* set device default value */
    tuh_info->gpio = GPIO_TRIGGER;
    tuh_info->last_activetime = 0;
    tuh_info->last_gpiostate = 0xFF;
    tuh_info->debounce_time = DEFAULT_DEBOUNCE_TIME;
    tuh_info->maximum_contacts = MAX_VALID_CONTACTS;

    /* initialize testcase variable */
    memset(&tuh_info->data, 0, sizeof(struct hid_touch_data));
    tuh_info->mt_count = 0;
    tuh_info->mt_generate = 0;
    tuh_info->data.report_id = REPORT_ID_MULTITOUCH;
    sem_init(&tuh_info->signal_thread, 0, 0);

    list_add(&dev_info->device_list, &tuh_info->list);

    /* create thread to send demo report data */
    if (pthread_create(&tuh_info->hid_thread, NULL, (void*)touch_thread_func,
                       (void*)dev) != 0) {
        ret = -EIO;
        goto err_free_tuh_info;
    }

    gpio_activate(GPIO_TRIGGER);
    gpio_direction_in(GPIO_TRIGGER);
    gpio_irq_mask(GPIO_TRIGGER);
    gpio_irq_settriggering(GPIO_TRIGGER, IRQ_TYPE_EDGE_BOTH);
    gpio_irq_attach(GPIO_TRIGGER, touch_irq_event, NULL);

    /* initialize waitqueue */
    tuh_info->abort = 0;
    return ret;

err_free_tuh_info:
    list_del(&tuh_info->list);
    free(tuh_info);
err_hw_init:
    return ret;
}

/**
 * @brief Deinitialize multi-touch hardware setting
 *
 * @param dev Pointer to structure of device data
 * @param dev_info The pointer for hid_info struct
 *
 * @return 0 on success, negative errno on error
 */
static int touch_hw_deinitialize(struct device *dev)
{
    struct touch_info *tuh_info = NULL;

    tuh_info = touch_get_info(dev, GPIO_TRIGGER);
    if (!tuh_info) {
        return -EIO;
    }

    if (tuh_info->hid_thread != (pthread_t)0) {
        tuh_info->abort = 1;
        sem_post(&tuh_info->signal_thread);
        /* wait for thread completed */
        pthread_join(tuh_info->hid_thread, NULL);
    }

    /* uninitialize GPIO pin */
    gpio_irq_mask(GPIO_TRIGGER);
    gpio_deactivate(GPIO_TRIGGER);
    list_del(&tuh_info->list);
    free(tuh_info);

    return 0;
}

static struct hid_vendor_ops touch_ops = {
    .hw_initialize = touch_hw_initialize,
    .hw_deinitialize = touch_hw_deinitialize,
    .power_control = touch_power_set,
    .get_report = touch_get_report,
    .set_report = touch_set_report,
};

int hid_device_init(struct device *dev, struct hid_info *dev_info)
{
    int ret = 0;

    dev_info->hdesc = &touch_dev_desc;
    dev_info->rdesc = touch_report_desc;
    dev_info->sinfo = hid_sizeinfo;
    dev_info->num_ids = ARRAY_SIZE(hid_sizeinfo);
    dev_info->hid_dev_ops = &touch_ops;
    touch_dev = dev;

    return ret;
}
