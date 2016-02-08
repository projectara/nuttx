/**
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
 *
 * @brief Toshiba CSI Receiver and Transmitter Configuration
 */

#ifndef __ARCH_ARM_INCLUDE_TSB_CSI_H
#define __ARCH_ARM_INCLUDE_TSB_CSI_H

#include <arch/tsb/cdsi.h>

/* MIPI CSI-2 Data Types */
#define MIPI_DT_YUV420_8BIT         0x18
#define MIPI_DT_YUV420_10BIT        0x19
#define MIPI_DT_YUV420_LEGACY_8BIT  0x1a
#define MIPI_DT_YUV422_8BIT         0x1e
#define MIPI_DT_YUV422_10BIT        0x1f
#define MIPI_DT_RGB444              0x20
#define MIPI_DT_RGB555              0x21
#define MIPI_DT_RGB565              0x22
#define MIPI_DT_RGB666              0x23
#define MIPI_DT_RGB888              0x24
#define MIPI_DT_RAW6                0x28
#define MIPI_DT_RAW7                0x29
#define MIPI_DT_RAW8                0x2a
#define MIPI_DT_RAW10               0x2b
#define MIPI_DT_RAW12               0x2c
#define MIPI_DT_RAW14               0x2d

/* Flags for the CSI Tx configuration */
#define CSI_TX_FLAG_CLOCK_CONTINUOUS  BIT(0)

/* CSI RX */
struct csi_rx_config {
    unsigned int vchan;
    unsigned int num_lanes;
};

struct cdsi_dev *csi_rx_open(unsigned int cdsi);
void csi_rx_close(struct cdsi_dev *dev);

int csi_rx_init(struct cdsi_dev *dev, const struct csi_rx_config *cfg);
int csi_rx_uninit(struct cdsi_dev *dev);

int csi_rx_start(struct cdsi_dev *dev);
int csi_rx_stop(struct cdsi_dev *dev);

uint32_t csi_rx_get_error(struct cdsi_dev *dev);

/* CSI TX */

/**
 * CSI configuration for camera stream start
 */
struct csi_tx_config {
    /** flags for csi transmitter configuration */
    uint8_t flags;
    /** lane number for camera stream */
    uint8_t num_lanes;
    /** CSI frequency in Hz */
    uint32_t bus_freq;
    /** line per second including blankings */
    uint32_t lines_per_second;
};

/**
 * @brief Validate settings provided to CSI interface
 *
 * @param cfg Pointer to structure of CSI settings to validate
 * @return 0 on success, negative on error
 */
int csi_tx_validate_config(struct csi_tx_config *cfg);

/**
 * @brief Start the CSI for data streaming
 *
 * @param dev Pointer to the CDSI device structure
 * @param cfg Pointer to structure of CSI configuration settings.
 * @return 0 on success, negative errno on error.
 */
int csi_tx_start(struct cdsi_dev *dev, struct csi_tx_config *cfg);

/**
 * @brief Stop the CSI for data streaming
 * @param csi_id The CDSI transmitter (0 or 1)
 *
 * @return 0 on success, negative errno on error.
 */
int csi_tx_stop(struct cdsi_dev *dev);

#endif  /* __ARCH_ARM_INCLUDE_TSB_CSI_H */
