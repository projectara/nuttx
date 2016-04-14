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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

#include <arch/tsb/cdsi.h>
#include <arch/tsb/cdsi0_offs_def.h>
#include <arch/tsb/cdsi0_reg_def.h>
#include <arch/tsb/csi.h>
#include <nuttx/util.h>

#define CDSI_TX_START_TIMEOUT           10
#define CDSI_TX_STOP_TIMEOUT            10

/* CSI TX interface parameters */
#define CDSITX_PLL_REF_CLK_HZ           38400000
#define CDSITX_PLL_VCO_MIN_FREQ_HZ      1000000000
#define CDSITX_PLL_HSCK_MIN_FREQ_HZ     62500000
#define CDSITX_PLL_HSCK_MAX_FREQ_HZ     1000000000

/* Timing parameters for csi transmitter */
/**
 * @brief Associates clocks configuration values to
 *        an upper limit of line speed in Mbps
 */
struct tsb_csi_tx_timing_params {
    /** The value to compare the current line speed in Mpbs to */
    uint16_t fence;
    /** Value used to configure the CSI timing registers */
    uint16_t val;
};

/* ---- Configurations value --- */
static const struct tsb_csi_tx_timing_params non_bta_lptxtimecnt[] = {
    { .fence = 95,   .val = 2 },
    { .fence = 127,  .val = 3 },
    { .fence = 159,  .val = 4 },
    { .fence = 190,  .val = 5 },
    { .fence = 220,  .val = 7 },
    { .fence = 254,  .val = 3 },
    { .fence = 318,  .val = 4 },
    { .fence = 381,  .val = 5 },
    { .fence = 400,  .val = 6 },
    { .fence = 509,  .val = 3 },
    { .fence = 636,  .val = 4 },
    { .fence = 763,  .val = 5 },
    { .fence = 891,  .val = 6 },
    { .fence = 1000, .val = 7 },
};

static const struct tsb_csi_tx_timing_params non_bta_twakeupcnt[] = {
    { .fence = 95,   .val = 7918 },
    { .fence = 127,  .val = 7939 },
    { .fence = 159,  .val = 7951 },
    { .fence = 190,  .val = 7951 },
    { .fence = 220,  .val = 7939 },
    { .fence = 254,  .val = 7939 },
    { .fence = 318,  .val = 7951 },
    { .fence = 381,  .val = 7951 },
    { .fence = 400,  .val = 7951 },
    { .fence = 509,  .val = 7955 },
    { .fence = 636,  .val = 7955 },
    { .fence = 763,  .val = 7955 },
    { .fence = 891,  .val = 7957 },
    { .fence = 1000, .val = 7958 },
};

static const struct tsb_csi_tx_timing_params rxtasurecnt[] = {
    { .fence = 95,   .val = 7  },
    { .fence = 127,  .val = 8  },
    { .fence = 159,  .val = 9  },
    { .fence = 190,  .val = 12 },
    { .fence = 220,  .val = 14 },
    { .fence = 254,  .val = 9  },
    { .fence = 318,  .val = 10 },
    { .fence = 381,  .val = 12 },
    { .fence = 400,  .val = 13 },
    { .fence = 509,  .val = 9  },
    { .fence = 636,  .val = 10 },
    { .fence = 763,  .val = 12 },
    { .fence = 891,  .val = 13 },
    { .fence = 1000, .val = 14 },
};

static const struct tsb_csi_tx_timing_params tclk_prezerocnt[] = {
    { .fence = 83,   .val = 13 },
    { .fence = 90,   .val = 14 },
    { .fence = 97,   .val = 15 },
    { .fence = 104,  .val = 16 },
    { .fence = 110,  .val = 17 },
    { .fence = 117,  .val = 18 },
    { .fence = 124,  .val = 19 },
    { .fence = 131,  .val = 20 },
    { .fence = 137,  .val = 21 },
    { .fence = 144,  .val = 22 },
    { .fence = 151,  .val = 23 },
    { .fence = 157,  .val = 24 },
    { .fence = 164,  .val = 25 },
    { .fence = 171,  .val = 26 },
    { .fence = 178,  .val = 27 },
    { .fence = 184,  .val = 28 },
    { .fence = 191,  .val = 29 },
    { .fence = 198,  .val = 30 },
    { .fence = 204,  .val = 31 },
    { .fence = 211,  .val = 32 },
    { .fence = 220,  .val = 33 },
    { .fence = 225,  .val = 16 },
    { .fence = 238,  .val = 17 },
    { .fence = 251,  .val = 18 },
    { .fence = 265,  .val = 19 },
    { .fence = 278,  .val = 20 },
    { .fence = 292,  .val = 21 },
    { .fence = 305,  .val = 22 },
    { .fence = 319,  .val = 23 },
    { .fence = 332,  .val = 24 },
    { .fence = 346,  .val = 25 },
    { .fence = 359,  .val = 26 },
    { .fence = 372,  .val = 27 },
    { .fence = 386,  .val = 28 },
    { .fence = 399,  .val = 29 },
    { .fence = 400,  .val = 30 },
    { .fence = 413,  .val = 14 },
    { .fence = 440,  .val = 15 },
    { .fence = 467,  .val = 16 },
    { .fence = 493,  .val = 17 },
    { .fence = 520,  .val = 18 },
    { .fence = 547,  .val = 19 },
    { .fence = 574,  .val = 20 },
    { .fence = 601,  .val = 21 },
    { .fence = 628,  .val = 22 },
    { .fence = 655,  .val = 23 },
    { .fence = 682,  .val = 24 },
    { .fence = 708,  .val = 25 },
    { .fence = 735,  .val = 26 },
    { .fence = 762,  .val = 27 },
    { .fence = 789,  .val = 28 },
    { .fence = 816,  .val = 29 },
    { .fence = 843,  .val = 30 },
    { .fence = 870,  .val = 31 },
    { .fence = 897,  .val = 32 },
    { .fence = 923,  .val = 33 },
    { .fence = 950,  .val = 34 },
    { .fence = 977,  .val = 35 },
    { .fence = 1000, .val = 36 },
};

static const struct tsb_csi_tx_timing_params tclk_precnt[] = {
    { .fence = 198,  .val = 7 },
    { .fence = 220,  .val = 8 },
    { .fence = 397,  .val = 2 },
    { .fence = 400,  .val = 3 },
    { .fence = 1000, .val = 0 },
};

static const struct tsb_csi_tx_timing_params tclk_preparecnt[] = {
    { .fence = 82,   .val = 2 },
    { .fence = 106,  .val = 3 },
    { .fence = 130,  .val = 4 },
    { .fence = 154,  .val = 5 },
    { .fence = 178,  .val = 6 },
    { .fence = 220,  .val = 7 },
    { .fence = 260,  .val = 4 },
    { .fence = 308,  .val = 5 },
    { .fence = 356,  .val = 6 },
    { .fence = 400,  .val = 7 },
    { .fence = 425,  .val = 3 },
    { .fence = 521,  .val = 4 },
    { .fence = 617,  .val = 5 },
    { .fence = 713,  .val = 6 },
    { .fence = 809,  .val = 7 },
    { .fence = 905,  .val = 8 },
    { .fence = 1000, .val = 9 },
};

static const struct tsb_csi_tx_timing_params tclk_exitcnt[] = {
    { .fence = 84,   .val = 5  },
    { .fence = 98,   .val = 6  },
    { .fence = 112,  .val = 7  },
    { .fence = 126,  .val = 8  },
    { .fence = 140,  .val = 9  },
    { .fence = 154,  .val = 10 },
    { .fence = 168,  .val = 11 },
    { .fence = 182,  .val = 12 },
    { .fence = 196,  .val = 13 },
    { .fence = 210,  .val = 14 },
    { .fence = 220,  .val = 15 },
    { .fence = 224,  .val = 7  },
    { .fence = 252,  .val = 8  },
    { .fence = 280,  .val = 9  },
    { .fence = 308,  .val = 10 },
    { .fence = 336,  .val = 11 },
    { .fence = 364,  .val = 12 },
    { .fence = 392,  .val = 13 },
    { .fence = 400,  .val = 14 },
    { .fence = 448,  .val = 7  },
    { .fence = 504,  .val = 8  },
    { .fence = 560,  .val = 9  },
    { .fence = 616,  .val = 10 },
    { .fence = 673,  .val = 11 },
    { .fence = 729,  .val = 12 },
    { .fence = 785,  .val = 13 },
    { .fence = 841,  .val = 14 },
    { .fence = 897,  .val = 15 },
    { .fence = 953,  .val = 16 },
    { .fence = 1000, .val = 17 },
};

static const struct tsb_csi_tx_timing_params tclk_trailcnt[] = {
    { .fence = 103,  .val = 4 },
    { .fence = 136,  .val = 5 },
    { .fence = 168,  .val = 6 },
    { .fence = 200,  .val = 7 },
    { .fence = 220,  .val = 8 },
    { .fence = 281,  .val = 3 },
    { .fence = 346,  .val = 4 },
    { .fence = 400,  .val = 5 },
    { .fence = 443,  .val = 1 },
    { .fence = 573,  .val = 2 },
    { .fence = 702,  .val = 3 },
    { .fence = 831,  .val = 4 },
    { .fence = 960,  .val = 5 },
    { .fence = 1000, .val = 6 },
};

static const struct tsb_csi_tx_timing_params ths_prezerocnt[] = {
    { .fence = 199,  .val = 0  },
    { .fence = 212,  .val = 1  },
    { .fence = 220,  .val = 2  },
    { .fence = 226,  .val = 0  },
    { .fence = 254,  .val = 1  },
    { .fence = 281,  .val = 2  },
    { .fence = 309,  .val = 3  },
    { .fence = 336,  .val = 4  },
    { .fence = 363,  .val = 5  },
    { .fence = 391,  .val = 6  },
    { .fence = 400,  .val = 7  },
    { .fence = 446,  .val = 3  },
    { .fence = 501,  .val = 4  },
    { .fence = 556,  .val = 5  },
    { .fence = 611,  .val = 6  },
    { .fence = 666,  .val = 7  },
    { .fence = 721,  .val = 8  },
    { .fence = 775,  .val = 9  },
    { .fence = 830,  .val = 10 },
    { .fence = 885,  .val = 11 },
    { .fence = 940,  .val = 12 },
    { .fence = 995,  .val = 13 },
    { .fence = 1000, .val = 14 },
};

static const struct tsb_csi_tx_timing_params ths_preparecnt[] = {
    { .fence = 99,   .val = 5  },
    { .fence = 124,  .val = 6  },
    { .fence = 148,  .val = 7  },
    { .fence = 173,  .val = 8  },
    { .fence = 198,  .val = 9  },
    { .fence = 220,  .val = 10 },
    { .fence = 259,  .val = 5  },
    { .fence = 309,  .val = 6  },
    { .fence = 358,  .val = 7  },
    { .fence = 400,  .val = 8  },
    { .fence = 480,  .val = 4  },
    { .fence = 579,  .val = 5  },
    { .fence = 679,  .val = 6  },
    { .fence = 778,  .val = 7  },
    { .fence = 877,  .val = 8  },
    { .fence = 976,  .val = 9  },
    { .fence = 1000, .val = 10 },
};

static const struct tsb_csi_tx_timing_params ths_exitcnt[] = {
    { .fence = 85,   .val = 9  },
    { .fence = 99,   .val = 10 },
    { .fence = 113,  .val = 11 },
    { .fence = 127,  .val = 12 },
    { .fence = 142,  .val = 13 },
    { .fence = 156,  .val = 14 },
    { .fence = 170,  .val = 15 },
    { .fence = 184,  .val = 16 },
    { .fence = 199,  .val = 17 },
    { .fence = 213,  .val = 18 },
    { .fence = 220,  .val = 19 },
    { .fence = 227,  .val = 9  },
    { .fence = 255,  .val = 10 },
    { .fence = 284,  .val = 11 },
    { .fence = 312,  .val = 12 },
    { .fence = 341,  .val = 13 },
    { .fence = 369,  .val = 14 },
    { .fence = 398,  .val = 15 },
    { .fence = 400,  .val = 16 },
    { .fence = 455,  .val = 8  },
    { .fence = 511,  .val = 9  },
    { .fence = 568,  .val = 10 },
    { .fence = 625,  .val = 11 },
    { .fence = 682,  .val = 12 },
    { .fence = 739,  .val = 13 },
    { .fence = 796,  .val = 14 },
    { .fence = 853,  .val = 15 },
    { .fence = 910,  .val = 16 },
    { .fence = 967,  .val = 17 },
    { .fence = 1000, .val = 18 },
};

static struct  tsb_csi_tx_timing_params ths_trailcnt[] = {
    { .fence = 96,   .val = 10 },
    { .fence = 128,  .val = 11 },
    { .fence = 160,  .val = 12 },
    { .fence = 193,  .val = 13 },
    { .fence = 220,  .val = 14 },
    { .fence = 241,  .val = 6  },
    { .fence = 306,  .val = 7  },
    { .fence = 371,  .val = 8  },
    { .fence = 400,  .val = 9  },
    { .fence = 461,  .val = 4  },
    { .fence = 589,  .val = 5  },
    { .fence = 716,  .val = 6  },
    { .fence = 844,  .val = 7  },
    { .fence = 971,  .val = 8  },
    { .fence = 1000, .val = 9  },
};

static struct tsb_csi_tx_timing_params cnt_tclk_postcnt[] = {
    { .fence = 100,  .val = 16 },
    { .fence = 120,  .val = 17 },
    { .fence = 141,  .val = 18 },
    { .fence = 161,  .val = 19 },
    { .fence = 181,  .val = 20 },
    { .fence = 201,  .val = 21 },
    { .fence = 220,  .val = 23 },
    { .fence = 221,  .val = 6  },
    { .fence = 261,  .val = 7  },
    { .fence = 302,  .val = 8  },
    { .fence = 342,  .val = 9  },
    { .fence = 382,  .val = 10 },
    { .fence = 400,  .val = 11 },
    { .fence = 463,  .val = 1  },
    { .fence = 544,  .val = 2  },
    { .fence = 624,  .val = 3  },
    { .fence = 705,  .val = 4  },
    { .fence = 785,  .val = 5  },
    { .fence = 866,  .val = 6  },
    { .fence = 946,  .val = 7  },
    { .fence = 1000, .val = 8  },
};

static struct tsb_csi_tx_timing_params non_cnt_tclk_postcnt[] = {
    { .fence = 100,  .val = 32 },
    { .fence = 120,  .val = 33 },
    { .fence = 141,  .val = 34 },
    { .fence = 161,  .val = 35 },
    { .fence = 181,  .val = 36 },
    { .fence = 201,  .val = 37 },
    { .fence = 220,  .val = 39 },
    { .fence = 221,  .val = 18 },
    { .fence = 261,  .val = 19 },
    { .fence = 302,  .val = 20 },
    { .fence = 342,  .val = 21 },
    { .fence = 382,  .val = 22 },
    { .fence = 400,  .val = 23 },
    { .fence = 463,  .val = 11 },
    { .fence = 544,  .val = 12 },
    { .fence = 624,  .val = 13 },
    { .fence = 705,  .val = 14 },
    { .fence = 785,  .val = 15 },
    { .fence = 866,  .val = 16 },
    { .fence = 946,  .val = 17 },
    { .fence = 1000, .val = 18 },
};

static struct tsb_csi_tx_timing_params syscldtclksel[] = {
    { .fence = 220,  .val = 2 },
    { .fence = 400,  .val = 1 },
    { .fence = 1000, .val = 0 },
};

/**
 * @brief Walk the array of possible fence-value pairs
 *
 * @param dev The CDSI device
 * @param params The table of values to inspect
 *
 * @return The value associated with current interface hsck
 */
static int csi_tx_get_param(struct cdsi_dev *dev,
        const struct tsb_csi_tx_timing_params *params)
{
    struct tsb_csi_tx_timing_params *rover =
        (struct tsb_csi_tx_timing_params *)params;

    /*
     * The hsck value is guaranteed by the PLL computation code to
     * be lower than 1000 MHz, ensuring that the loop will not
     * go past the last entry of the timing parameters table
     */
    while (dev->hsck_mhz >= rover->fence)
        rover++;

    return rover->val;
}

/**
 * @brief Compute delay settings for tx bridge pixel interface
 *
 * @param dev The CDSI device
 * @param cfg The CDSI device configuration as supplied by AP
 */
static void csi_reg_pic_com_delay(struct cdsi_dev *dev,
        struct csi_tx_config *cfg)
{
    uint32_t line_len_nsec;
    uint32_t line_len_samples;

    /* Assign to variables for rounding */
    line_len_nsec = DIV_ROUND_CLOSEST(1000000000, cfg->lines_per_second);
    line_len_samples = line_len_nsec * (cfg->bus_freq / 1000000);

    dev->pic_com_delay = line_len_samples / 16000 + 1;
}

/**
 * @brief Compute pll register settings based on AP requested
 *        bus frequency
 *
 * @param dev  CDSI interface
 * @param bus_freq_hz The bus frequency in Hz requested by the AP
 */
static void csi_reg_pll_config(struct cdsi_dev *dev, uint32_t bus_freq_hz)
{
    uint32_t pll_vco_hz;
    uint16_t pll_frs;
    uint16_t pll_fbd;

    /*
     * The PLL is organized as follows.
     *
     * External Input                        1GHz < VCO < 2GHz             HSCK
     * Clock (38.4 MHz)
     *   |                                           |                      |
     *   |                                           |                      |
     *   v       ,-----------.     ,--------------.  |  ,------------.      v
     *  ,--.     |   Pre PLL |     |   PLL        |  v  |   Post PLL |     ,--.
     *  |  | --> | ÷ Clock   | --> | x Multiplier | --> | ÷ Clock    | --> |  |
     *  `--'     |   Divider |     |              |     |   Divider  |     `--'
     *           `-----------'     `--------------'     `------------'
     *                 ^                  ^                   ^
     *                 |                  |                   |
     *                PRD                FBD                2^FRS
     *
     * PRD: PRe-Divider
     * FBD: FeedBack Divider
     * FRS: Frequency Range Selection
     *
     * Note that the values written in the hardware
     * registers are PRD - 1, FBD - 1 and FRS - 1.
     *
     * The code below doesn't take this into account to simplify PLL
     * calculations, the offset is only subtracted when computing the register
     * values.
     */

    /*
     * Compute the Post PLL Clock Divider value. As the divider can only takes
     * power of two values, and as the maximum allowed VCO frequency is exactly
     * twice its minimum allowed value, there's exactly one possible
     * post-divider value except for output frequencies equal to the VCO lower
     * bound divided by a power of two, in which case two post-divider values
     * are possible. We can thus simply look for the lowest power of two
     * multiplier of the output frequency that leads to a VCO frequency higher
     * than or equal to the VCO lower bound. As a result the VCO frequency will
     * always be lower than and never equal to its upper bound.
     */
    pll_frs = 0;
    do {
        pll_vco_hz = bus_freq_hz * (1 << (++pll_frs));
    } while (pll_vco_hz < CDSITX_PLL_VCO_MIN_FREQ_HZ);

    /*
     * We need to round the FBD multiplier up to achieve a VCO equal to or
     * higher than the requested output frequency value.
     *
     * Hardcode the PRD pre-divider to 1 for now, the output frequency
     * accuracy can be improved by making active use of it.
     */
    pll_fbd = (pll_vco_hz + CDSITX_PLL_REF_CLK_HZ - 1)
	        / CDSITX_PLL_REF_CLK_HZ;

    pll_vco_hz = pll_fbd * CDSITX_PLL_REF_CLK_HZ;
    dev->hsck_mhz = pll_vco_hz / (1 << pll_frs) / 1000000;
    dev->pll_config_fbd = pll_fbd;
    dev->pll_config_frs = pll_frs;
    dev->pll_config_prd = 1;
}

/**
 * @brief Configure CSI TX timing parameters using parameter tables
 *
 * @param dev The cdsi device
 * @param cfg The CSI transmitter configuration structure
 */
static void csi_tx_set_timings(struct cdsi_dev *dev, struct csi_tx_config *cfg)
{
    uint32_t val;

    val = csi_tx_get_param(dev, syscldtclksel);
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_00_OFFS, val);

    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_OFFS,
               CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_SBS_DPHY_CLM_LPTXCURR1EN_MASK |
               CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_SBS_DPHY_D3M_LPTXCURR1EN_MASK |
               CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_SBS_DPHY_D2M_LPTXCURR1EN_MASK |
               CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_SBS_DPHY_D1M_LPTXCURR1EN_MASK |
               CDSI0_CDSITX_GLOBAL_TIMING_PARAM_01_SBS_DPHY_D0M_LPTXCURR1EN_MASK);

    val = csi_tx_get_param(dev, non_bta_lptxtimecnt);
    if (val == 2)
        cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_OFFS,
                   (val + 1) <<
                   CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_SBS_DPHY_HSTXCLK_DIVCNT_SHIFT);
    else
        cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_OFFS,
                   val <<
                   CDSI0_CDSITX_GLOBAL_TIMING_PARAM_02_SBS_DPHY_HSTXCLK_DIVCNT_SHIFT);

    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_03_OFFS,
               val <<
               CDSI0_CDSITX_GLOBAL_TIMING_PARAM_03_SBS_DPHY_PPI_LPTXTIMECNT_SHIFT);

    val = csi_tx_get_param(dev, tclk_preparecnt) <<
	    CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_SBS_DPHY_PPI_TCLK_PREPARECNT_SHIFT |
          csi_tx_get_param(dev, tclk_precnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_SBS_DPHY_PPI_TCLK_PRECNT_SHIFT |
          csi_tx_get_param(dev, tclk_prezerocnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_SBS_DPHY_PPI_TCLK_PREZEROCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_04_OFFS, val);

    val = csi_tx_get_param(dev, tclk_trailcnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_05_SBS_DPHY_PPI_TCLK_TRAILCNT_SHIFT |
          csi_tx_get_param(dev, tclk_exitcnt)  <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_05_SBS_DPHY_PPI_TCLK_EXITCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_05_OFFS, val );

    val = csi_tx_get_param(dev, ths_preparecnt) <<
	    CDSI0_CDSITX_GLOBAL_TIMING_PARAM_06_SBS_DPHY_PPI_THS_PREPARECNT_SHIFT |
          csi_tx_get_param(dev, ths_prezerocnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_06_SBS_DPHY_PPI_THS_PREZEROCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_06_OFFS, val);

    val = csi_tx_get_param(dev, ths_trailcnt) <<
	    CDSI0_CDSITX_GLOBAL_TIMING_PARAM_07_SBS_DPHY_PPI_THS_TRAILCNT_SHIFT |
          csi_tx_get_param(dev, ths_exitcnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_07_SBS_DPHY_PPI_THS_EXITCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_07_OFFS, val);

    val = csi_tx_get_param(dev, non_bta_twakeupcnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_08_SBS_DPHY_PPI_TWAKEUPCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_08_OFFS, val);

    if (cfg->flags & CSI_TX_FLAG_CLOCK_CONTINUOUS)
        val = csi_tx_get_param(dev, cnt_tclk_postcnt) <<
              CDSI0_CDSITX_GLOBAL_TIMING_PARAM_09_SBS_DPHY_PPI_TCLK_POSTCNT_SHIFT;
    else
        val = csi_tx_get_param(dev, non_cnt_tclk_postcnt) <<
              CDSI0_CDSITX_GLOBAL_TIMING_PARAM_09_SBS_DPHY_PPI_TCLK_POSTCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_09_OFFS, val);

    val = csi_tx_get_param(dev, rxtasurecnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_10_SBS_DPHY_PPI_RXTASURECNT_SHIFT |
          csi_tx_get_param(dev, non_bta_lptxtimecnt) <<
            CDSI0_CDSITX_GLOBAL_TIMING_PARAM_10_SBS_DPHY_PPI_TXTAGOCNT_SHIFT;
    cdsi_write(dev, CDSI0_CDSITX_GLOBAL_TIMING_PARAM_10_OFFS, val);
}

/**
 * @brief Validate the supplied configuration values for the CSI interface
 *
 * @param cfg The CSI interface configuration values
 *
 * @return 0 on success, a negative value for errors
 */
int csi_tx_validate_config(struct csi_tx_config *cfg)
{
    /* Validate the bus requested frequency */
    if (cfg->bus_freq < CDSITX_PLL_HSCK_MIN_FREQ_HZ ||
        cfg->bus_freq > CDSITX_PLL_HSCK_MAX_FREQ_HZ) {
        lldbg("CDSI: Invalid bus frequency %u\n", cfg->bus_freq);
        return -EINVAL;
    }

    /* Validate the number of CSI data lanes */
    if (cfg->num_lanes < 1 || cfg->num_lanes > 4) {
        lldbg("CDSI: Invalid number of data lanes\n");
        return -EINVAL;
    }

    /* Validate flags: only clock mode is currently supported */
    if (cfg->flags & ~CSI_TX_FLAG_CLOCK_CONTINUOUS) {
        lldbg("CDSI: Unsupported flags 0x%2x\n", cfg->flags);
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Set the registers in CSI controller to start transfer.
 * @param dev Pointer to the CDSI device
 * @param cfg Pointer to CSI transmitter configuration structure
 *
 * @return 0 on success or a negative error code on failure.
 */
int csi_tx_start(struct cdsi_dev *dev, struct csi_tx_config *cfg)
{
    unsigned int timeout;
    uint32_t val;

    cdsi_enable(dev);

    csi_reg_pll_config(dev, cfg->bus_freq);
    csi_reg_pic_com_delay(dev, cfg);

    /* Set to Tx mode for CDSI */
    cdsi_write(dev, CDSI0_AL_TX_BRG_CDSITX_MODE_OFFS,
               CDSI0_AL_TX_BRG_CDSITX_MODE_AL_TX_CDSITX_MODE_MASK);
    cdsi_write(dev, CDSI0_AL_TX_BRG_SOFT_RESET_OFFS, 1);
    cdsi_write(dev, CDSI0_AL_TX_BRG_SYSCLK_ENABLE_OFFS, 1);
    cdsi_write(dev, CDSI0_AL_TX_BRG_ENABLE_OFFS, 1);

    /* Tx bridge setting */
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS,
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_WAIT_INTERVAL_MODE_MASK |
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_MASTER_SYNC_MODE_MASK |
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_CSI_MODE_MASK);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_SET_OFFS,
               (1 << CDSI0_AL_TX_BRG_PIC_COM_SET_AL_TX_BRG_REG_COM_3D_EN_A_SHIFT) |
               CDSI0_AL_TX_BRG_PIC_COM_SET_AL_TX_BRG_REG_COM_PLURALIMGTYPE_A_MASK |
               CDSI0_AL_TX_BRG_PIC_COM_SET_AL_TX_BRG_REG_COM_FRAMENUM_EN_A_MASK |
               CDSI0_AL_TX_BRG_PIC_COM_SET_AL_TX_BRG_REG_COM_UPDATE_EN_A_MASK);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYSTRCOUNT_OFFS,
               dev->pic_com_delay);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_VDELAYENDCOUNT_OFFS,
               dev->pic_com_delay);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_MAXFCNT_OFFS, 0);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_SET_OFFS,
               CDSI0_AL_TX_BRG_PIC_SYN_SET_AL_TX_BRG_REG_SYN_UPDATE_EN_A_MASK);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_SYN_LINE_OFFS, 0);

    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_PLDT_OFFS, 0);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_3DCM_LINE_OFFS, 0);

    cdsi_write(dev, CDSI0_AL_TX_BRG_WAIT_CYCLE_SET_OFFS,
               (16 << CDSI0_AL_TX_BRG_WAIT_CYCLE_SET_AL_TX_BRG_TIM_WAIT_CYCLE_SHIFT) |
               (16 << CDSI0_AL_TX_BRG_WAIT_CYCLE_SET_AL_TX_BRG_IMAGE_WAIT_CYCLE_SHIFT));
    cdsi_write(dev, CDSI0_AL_TX_BRG_VSYNC_LINE_SET_OFFS, 1);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL1_SET_OFFS, 0x08fbfffc);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_SEL2_SET_OFFS, 0xff00c0e0);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK1_SET_OFFS, 0x08fb00fc);
    cdsi_write(dev, CDSI0_AL_TX_BRG_TYPE_MASK2_SET_OFFS, 0xff00c0e0);
    cdsi_write(dev, CDSI0_AL_TX_BRG_STATUS_ENABLE_OFFS,
               0x80 | CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_PKTIF_BUSY_START_ENA_MASK |
               CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_VS_AUTOINS_EXEC_INFO_ENA_MASK |
               CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_HS_AUTOINS_EXEC_INFO_ENA_MASK |
               CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_BRG_WC_ERROR_ENA_MASK |
               CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_BRG_SP_NOEOM_2ND_ERROR_ENA_MASK |
               CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_BRG_EOM_1ST_ERROR_ENA_MASK |
               CDSI0_AL_TX_BRG_STATUS_ENABLE_AL_TX_BRG_FIFO_OVERFLOW_ENA_MASK);
    cdsi_write(dev, CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_OFFS,
               (7 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE7_SHIFT) |
               (6 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE6_SHIFT) |
               (5 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE5_SHIFT) |
               (4 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE4_SHIFT) |
               (3 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE3_SHIFT) |
               (2 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE2_SHIFT) |
               (1 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE1_SHIFT) |
               (0 << CDSI0_AL_TX_BRG_UNIPRO_BYTESWAP_AL_TX_BRG_UNIPRO_BYTESWAP_BYTE0_SHIFT));

    /* Tx bridge enable */
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS,
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_WAIT_INTERVAL_MODE_MASK |
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_MASTER_SYNC_MODE_MASK |
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_CSI_MODE_MASK |
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_EN_MASK);

    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_OFFS,
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_SBS_DPHY_LOCKUPDONE_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_SBS_DPHY_AUTOCALDONE_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_SBS_DPHY_HSTXVREGRDY_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_00_SBS_DPHY_LINEINITDONE_EN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_OFFS,
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_SBS_DPHY_ERRFALSECONTROL_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_SBS_DPHY_ERRSYNCESC_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_SBS_DPHY_ERRESC_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_SBS_DPHY_RXTRIGGERESC_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_SBS_DPHY_DIRECTION_TX_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_01_SBS_DPHY_DIRECTION_RX_EN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_OFFS,
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_CONT_ERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_ERRWC_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_ERRVCID_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_ERRDATATYPE_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_ERRECCDBL_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_ERRECCSINGLE_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_CHKSUMERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_PRESP_TO_ERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_PR_TO_ERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_TA_TO_ERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRX_TO_ERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_HS_TO_ERR_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_PRESP_TO_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_PR_TO_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_TA_TO_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_LPRXH_TO_EN_MASK |
               CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_02_SBS_LINK_HS_TO_EN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_FUNC_ENABLE_03_OFFS, 0);

    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_MASK_00_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI0_CDSITX_POWER_SW_OFFS,
               CDSI0_CDSITX_POWER_SW_SBD_DPHY_COM_EN_ZEMIPIV12_MASK |
               CDSI0_CDSITX_POWER_SW_SBD_DPHY_COM_V12PEN_MASK |
               CDSI0_CDSITX_POWER_SW_SBD_DPHY_COM_V12EN_MASK);

    /* D-PHY reset deassertion */
    cdsi_write(dev, CDSI0_CDSITX_DPHY_RESET_OFFS,
               CDSI0_CDSITX_DPHY_RESET_DPHY_RESET_N_MASK);

    /* D-PHY PLL setting */
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_00_OFFS,
               (1 << CDSI0_CDSITX_PLL_CONFIG_00_SBS_DPHY_CLM_LFBREN_SHIFT) |
               (2 << CDSI0_CDSITX_PLL_CONFIG_00_SBS_DPHY_MPM_LBWS_SHIFT)   |
               ((dev->pll_config_fbd - 1) <<
                    CDSI0_CDSITX_PLL_CONFIG_00_SBS_DPHY_MPM_FBD_SHIFT)     |
               ((dev->pll_config_frs - 1) <<
                    CDSI0_CDSITX_PLL_CONFIG_00_SBS_DPHY_MPM_FRS_SHIFT)     |
               ((dev->pll_config_prd - 1) <<
                    CDSI0_CDSITX_PLL_CONFIG_00_SBS_DPHY_MPM_PRD_SHIFT));
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONFIG_02_OFFS, 5760);

    /* D-PHY PLL enable */
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_00_OFFS,
               CDSI0_CDSITX_PLL_CONTROL_00_SBD_DPHY_MPM_PLLPSEN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_01_OFFS,
               CDSI0_CDSITX_PLL_CONTROL_01_SBD_DPHY_MP_ENABLE_MASK);

    /* CDSITX setting */
    val = CDSI0_CDSITX_LANE_ENABLE_00_SBS_DPHY_CLM_HM_EN_MASK
        | ((1 << cfg->num_lanes) - 1);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_00_OFFS, val);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_01_OFFS, val);
    cdsi_write(dev, CDSI0_CDSITX_LANE_ENABLE_02_OFFS, val);

    /* D-PHY Vreg setting */
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONFIG_OFFS, 19);

    /* D-PHY Vreg enable */
    val = CDSI0_CDSITX_LANE_ENABLE_00_SBS_DPHY_CLM_HM_EN_MASK
        | ((1 << cfg->num_lanes) - 1);
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONTROL_OFFS, val);

    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONFIG_OFFS, 10);
    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONTROL_OFFS,
               CDSI0_CDSITX_LPRX_CALIB_CONTROL_SBD_DPHY_CALIB_START_MASK);

    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_CSI2DSI_SELECT_OFFS,
               CDSI0_CDSITX_CSI2DSI_SELECT_SBS_APF_CSI2_MODE_MASK |
               CDSI0_CDSITX_CSI2DSI_SELECT_SBS_LINK_CSI2_MODE_MASK);

    csi_tx_set_timings(dev, cfg);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_00_OFFS, 1940);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_01_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_02_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_03_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_04_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_05_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_06_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_07_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_08_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_09_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_10_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_COUNT_CONFIG_11_OFFS, 0xffffffff);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_00_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_01_OFFS,
               CDSI0_CDSITX_SIDEBAND_CONFIG_01_SBS_DPHY_CLM_LPRXVTHLOW_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_01_SBS_DPHY_D3M_LPRXVTHLOW_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_01_SBS_DPHY_D2M_LPRXVTHLOW_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_01_SBS_DPHY_D1M_LPRXVTHLOW_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_01_SBS_DPHY_D0M_LPRXVTHLOW_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_02_OFFS,
               CDSI0_CDSITX_SIDEBAND_CONFIG_02_SBS_DPHY_CLM_CAP1_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_02_SBS_DPHY_D3M_CAP1_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_02_SBS_DPHY_D2M_CAP1_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_02_SBS_DPHY_D1M_CAP1_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_02_SBS_DPHY_D0M_CAP1_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_03_OFFS,
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_CLM_LPTXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D3M_LPTXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D2M_LPTXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D1M_LPTXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D0M_LPTXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_CLM_LPCDTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_CLM_LPRXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D3M_LPCDTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D3M_LPRXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D2M_LPCDTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D2M_LPRXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D1M_LPCDTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D1M_LPRXTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D0M_LPCDTRM_SHIFT) |
               (1 << CDSI0_CDSITX_SIDEBAND_CONFIG_03_SBS_DPHY_D0M_LPRXTRM_SHIFT));
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_04_OFFS,
               (2 << CDSI0_CDSITX_SIDEBAND_CONFIG_04_SBS_DPHY_CLM_HSTXTRM_SHIFT) |
               (2 << CDSI0_CDSITX_SIDEBAND_CONFIG_04_SBS_DPHY_D3M_HSTXTRM_SHIFT) |
               (2 << CDSI0_CDSITX_SIDEBAND_CONFIG_04_SBS_DPHY_D2M_HSTXTRM_SHIFT) |
               (2 << CDSI0_CDSITX_SIDEBAND_CONFIG_04_SBS_DPHY_D1M_HSTXTRM_SHIFT) |
               (2 << CDSI0_CDSITX_SIDEBAND_CONFIG_04_SBS_DPHY_D0M_HSTXTRM_SHIFT));

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_05_OFFS,
               CDSI0_CDSITX_SIDEBAND_CONFIG_05_SBS_LINK_RXVC3EN_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_05_SBS_LINK_RXVC2EN_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_05_SBS_LINK_RXVC1EN_MASK |
               CDSI0_CDSITX_SIDEBAND_CONFIG_05_SBS_LINK_RXVC0EN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_06_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_07_OFFS,
               cfg->flags & CSI_TX_FLAG_CLOCK_CONTINUOUS ?
               CDSI0_CDSITX_SIDEBAND_CONFIG_07_SBS_LINK_HS_CLK_MODE_MASK : 0);

    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_08_OFFS,
               CDSI0_CDSITX_SIDEBAND_CONFIG_08_SBS_APF_DSI_DTVALID_POL_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_09_OFFS,
               CDSI0_CDSITX_SIDEBAND_CONFIG_09_SBS_APF_DSI_SYNC_MODE_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_10_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_11_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_12_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_13_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_14_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONFIG_15_OFFS, 6144);

    /* Wait for PLL lock and D-PHY Vreg startup. */
    for (timeout = CDSI_TX_START_TIMEOUT; timeout != 0; --timeout) {
        const uint32_t mask = CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_LOCKUPDONE_MASK
                            | CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_HSTXVREGRDY_MASK;

        val = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
        if ((val & mask) == mask)
            break;

        usleep(10);
    }

    if (timeout == 0) {
        if (!(val & CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_LOCKUPDONE_MASK))
            printf("cdsi: PLL lock timeout\n");
        if (!(val & CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_HSTXVREGRDY_MASK))
            printf("cdsi: D-PHY Vreg startup timeout\n");

        return -ETIMEDOUT;
    }

    printf("cdsi: PLL lock and D-PHY Vreg startup complete (%u us)\n",
           (CDSI_TX_START_TIMEOUT - timeout) * 10);

    cdsi_write(dev, CDSI0_CDSITX_PISO_ENABLE_OFFS,
               CDSI0_CDSITX_PISO_ENABLE_SBS_DPHY_CLM_PISOEN_MASK |
               CDSI0_CDSITX_PISO_ENABLE_SBS_DPHY_D1M_PISOEN_MASK |
               CDSI0_CDSITX_PISO_ENABLE_SBS_DPHY_D0M_PISOEN_MASK);

    /* CDSITX setting */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_SBD_DPHY_MPM_CKEN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_01_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_01_SBD_DPHY_MPM_PLLINTCKEN_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_SBD_DPHY_HSTXCLKENABLE_MASK);
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_03_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_03_SBD_DPHY_STARTPPI_MASK);

    /* Wait for D-PHY line initialization. */
    for (timeout = CDSI_TX_START_TIMEOUT; timeout != 0; --timeout) {
        val = cdsi_read(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS);
        if (val & CDSI0_CDSITX_INTERRUPT_STATUS_00_INT_DPHY_LINEINITDONE_MASK)
            break;

        usleep(10);
    }

    if (timeout == 0) {
        printf("cdsi: D-PHY line initialization timeout\n");
        return -ETIMEDOUT;
    }

    printf("cdsi: D-PHY line initialization complete (%u us)\n",
           (CDSI_TX_START_TIMEOUT - timeout) * 10);

    /* Clear all interrupt statuses */
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_01_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_02_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_03_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_04_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_05_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS, 0xffffffff);

    /* Reset deassertion for LINK */
    cdsi_write(dev, CDSI0_CDSITX_LINK_RESET_OFFS,
               CDSI0_CDSITX_LINK_RESET_LINK_RESET_N_MASK);
    /* Reset deassertion for APF */
    cdsi_write(dev, CDSI0_CDSITX_APF_RESET_OFFS,
               CDSI0_CDSITX_APF_RESET_APF_RESET_N_MASK);

    /* APF start */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_04_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_04_SBD_APF_CDSITX_START_MASK);
    cdsi_write(dev, CDSI0_AL_TX_BRG_VPARAM_UPDATE_OFFS,
               CDSI0_AL_TX_BRG_VPARAM_UPDATE_AL_TX_BRG_SYS_UPDATE_EN_MASK |
               CDSI0_AL_TX_BRG_VPARAM_UPDATE_AL_TX_BRG_COM_UPDATE_EN_MASK);
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_START_OFFS,
               CDSI0_AL_TX_BRG_PIC_COM_START_AL_TX_BRG_REG_COM_START_A_MASK);

    return 0;
}

/**
 * @brief Set the registers in CSI controller to stop transfer.
 * @param dev Pointer to the CDSI device
 *
 * @return 0 on success or a negative error code on failure.
 */
int csi_tx_stop(struct cdsi_dev *dev)
{
    unsigned int timeout;

    /* Stop the Pixel I/F of CDSITX and wait for it to stop. */
    cdsi_write(dev, CDSI0_AL_TX_BRG_PIC_COM_START_OFFS, 0);

    for (timeout = CDSI_TX_STOP_TIMEOUT; timeout != 0; --timeout) {
        uint32_t val;

        val = cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_STATUS_05_OFFS);
        if (!(val & CDSI0_CDSITX_SIDEBAND_STATUS_05_SBO_APF_VHIF_BUSY_MASK))
            break;

        usleep(10);
    }

    if (timeout == 0) {
        printf("cdsi: Pixel I/F stop timeout\n");
    } else {
        printf("cdsi: Pixel I/F stop complete (%u us)\n",
               (CDSI_TX_STOP_TIMEOUT - timeout) * 10);
    }

    /* Wait until the transmission from UniPro to CDSITX is stopped */

    /* Disable the Tx bridge */
    cdsi_write(dev, CDSI0_AL_TX_BRG_MODE_OFFS,
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_WAIT_INTERVAL_MODE_MASK |
               CDSI0_AL_TX_BRG_MODE_AL_TX_BRG_MASTER_SYNC_MODE_MASK);

    /* Assert the Tx bridge reset */
    cdsi_write(dev, CDSI0_AL_TX_BRG_SOFT_RESET_OFFS, 1);

    /* Stop the APF function */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_04_OFFS, 0);

    /* Disable the PPI function */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_03_OFFS, 0);

    /* Disable the clock distribution for LINK and APF */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS, 0);

    /* Assert the APF reset */
    cdsi_write(dev, CDSI0_CDSITX_APF_RESET_OFFS, 0);

    /* Assert the LINK reset */
    cdsi_write(dev, CDSI0_CDSITX_LINK_RESET_OFFS, 0);

    /* Initialize the PPI */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONTROL_00_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_SBD_DPHY_MPM_CKEN_MASK);

    /* Enable clock distribution to stop the D-PHY clock lane */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_SBD_DPHY_HSTXCLKENABLE_MASK);

    /* Wait for the D-PHY clock lane to stop */
    for (timeout = CDSI_TX_STOP_TIMEOUT; timeout != 0; --timeout) {
        uint32_t val;

        val = cdsi_read(dev, CDSI0_CDSITX_SIDEBAND_STATUS_07_OFFS);
        if (val & CDSI0_CDSITX_SIDEBAND_STATUS_07_SBO_PPIIF_CLM_STOPSTATE_MASK)
            break;

        usleep(10);
    }

    if (timeout == 0) {
        printf("cdsi: D-PHY clock lane stop timeout\n");
        return -ETIMEDOUT;
    }

    printf("cdsi: D-PHY clock lane stop complete (%u us)\n",
           (CDSI_TX_STOP_TIMEOUT - timeout) * 10);

    /* Disable the clock distribution to stop the D-PHY clock lane */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_02_OFFS, 0);

    /* Disable the voltage requlator for D-PHY */
    cdsi_write(dev, CDSI0_CDSITX_VREG_CONTROL_OFFS, 0);

    /* Disable D-PHY functions */
    cdsi_write(dev, CDSI0_CDSITX_LPRX_CALIB_CONTROL_OFFS, 0);

    /* Disable PLL for CDSI */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_OFFS, 0);
    cdsi_write(dev, CDSI0_CDSITX_PLL_CONTROL_01_OFFS, 0);

    /* Clear the interrupt statuses */
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_00_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_01_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_02_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_03_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_04_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_05_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_CDSITX_INTERRUPT_STATUS_06_OFFS, 0xffffffff);
    cdsi_write(dev, CDSI0_AL_TX_BRG_STATUS_OFFS, 0xffffffff);

    /* Release the Initialization signal for the PPI */
    cdsi_write(dev, CDSI0_CDSITX_SIDEBAND_CONTROL_00_OFFS,
               CDSI0_CDSITX_SIDEBAND_INIT_CONTROL_00_SBD_DPHY_MPM_CKEN_MASK);

    cdsi_disable(dev);

    return 0;
}
