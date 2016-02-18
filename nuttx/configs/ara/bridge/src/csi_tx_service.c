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

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <arch/tsb/cdsi.h>
#include <arch/tsb/csi.h>

#define CSI_TX_PRIORITY         60
#define CSI_TX_STACK_SIZE       2048

/* State of CSI */
#define CSI_STATE_STOP          0
#define CSI_STATE_START         1

/* Command of CSI TX */
#define CSI_CMD_STOP            0
#define CSI_CMD_START           1

/**
 * @brief csi tx information
 */
struct csi_tx_info {
    /** data from AP */
    struct csi_tx_config cfg;
    /** csi id */
    uint8_t csi_id;
    /** csi commmand */
    uint32_t csi_cmd;
    /** csi state */
    uint32_t csi_state;
    /** low level driver */
    struct cdsi_dev *csi_dev;
    /** semapher for csi tx operation */
    sem_t csi_sem;
    /** csi tx control thread */
    pthread_t csi_thread;
};

static struct csi_tx_info *csi_tx_info;

/**
 * @brief Start the CSI Tx for camera stream
 *
 * @param cfg Pointer to structure of CSI configuration parameters.
 * @return 0 on success, negative errno on error.
 */
int csi_tx_srv_start(uint8_t csi_id, struct csi_tx_config *cfg)
{
    struct csi_tx_info *info = csi_tx_info;

    if (info->csi_state != CSI_STATE_STOP)
        return -EINVAL;

    /* copy parameters to internal space */
    info->cfg = *cfg;

    /* signal thread to start */
    info->csi_id = csi_id;
    info->csi_cmd = CSI_CMD_START;
    sem_post(&info->csi_sem);

    return 0;
}

/**
 * @brief The CSI stopping task for data streaming
 *
 * @return 0 on success, negative errno on error.
 */
int csi_tx_srv_stop(uint8_t csi_id)
{
    struct csi_tx_info *info = csi_tx_info;

    if (info->csi_state != CSI_STATE_START)
        return -EINVAL;

    info->csi_id = csi_id;
    info->csi_cmd = CSI_CMD_STOP;
    sem_post(&info->csi_sem);

    return 0;
}

/**
 * @brief The CSI control thread
 *
 * @param p_data Pointer to data for thread process.
 * @return None.
 */
static void *csi_tx_srv_thread(int argc, char *argv[])
{
    struct csi_tx_info *info = csi_tx_info;

    while (1) {
        sem_wait(&info->csi_sem);
        if (info->csi_cmd == CSI_CMD_START) {
            info->csi_dev = cdsi_open(info->csi_id, TSB_CDSI_TX);
            if (info->csi_dev) {
                csi_tx_start(info->csi_dev, &info->cfg);
                info->csi_state = CSI_STATE_START;
            }
        } else {
            csi_tx_stop(info->csi_dev);
            cdsi_close(info->csi_dev);
            info->csi_state = CSI_STATE_STOP;
        }
    }

    return NULL;
}

/**
 * @brief Start the CSI control thread for coming request
 *
 * @return 0 on success, negative errno on error.
 */
int csi_tx_srv_init(void)
{
    struct csi_tx_info *info;
    int ret;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    info->csi_state = CSI_STATE_STOP;

    ret = sem_init(&info->csi_sem, 0, 0);
    if (ret) {
        lldbg("csi_tx create semaphore failed \n");
        ret = -ret;
        goto err_free_mem;
    }

    csi_tx_info = info;

    ret = task_create("csi_tx_worker", CSI_TX_PRIORITY, CSI_TX_STACK_SIZE,
                      (main_t)csi_tx_srv_thread, NULL);
    if (ret == ERROR) {
        lldbg("failed to create csi_tx task\n");
        ret = -ret;
        goto err_destroy_sem;
    }

    return 0;

err_destroy_sem:
    sem_destroy(&info->csi_sem);
err_free_mem:
    free(info);

    return ret;
}
