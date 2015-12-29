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
#ifndef _NXP9890_H_
#define _NXP9890_H_

/* ADS2 = 0 ADS1 = 0 */
#define NXP9890_I2C_ADDR                     0x68
/* ADS2 = 0 ADS1 = 1 */
/*#define NXP9890_I2C_ADDR                     0x6A*/
/* ADS2 = 1 ADS1 = 0 */
/*#define NXP9890_I2C_ADDR                     0x6C*/
/* ADS2 = 1 ADS1 = 1 */
/*#define NXP9890_I2C_ADDR                     0x6E*/


/* sample registers and masks */
#define NXP9890_DUMMY_VOL_MUTE_REG  0x10

#define NXP9890_DUMMY_L_MUTE                       (0x1 << 15)
#define NXP9890_DUMMY_L_MUTE_SFT                   15
#define NXP9890_DUMMY_VOL_L_MUTE                   (0x1 << 14)
#define NXP9890_DUMMY_VOL_L_SFT                    14
#define NXP9890_DUMMY_R_MUTE                       (0x1 << 7)
#define NXP9890_DUMMY_R_MUTE_SFT                   7
#define NXP9890_DUMMY_VOL_R_MUTE                   (0x1 << 6)
#define NXP9890_DUMMY_VOL_R_SFT                    6
#define NXP9890_DUMMY_L_VOL_MASK                   (0x3F << 8)
#define NXP9890_DUMMY_L_VOL_SFT                    8
#define NXP9890_DUMMY_R_VOL_MASK                   (0x3F)
#define NXP9890_DUMMY_R_VOL_SFT                    0


#define NXP9890_DUMMY_POWER_REG     0x20

#define NXP9890_DUMMY_PWR_EN                        0
#define NXP9890_DUMMY_SPKR_AMP_L_EN                 6
#define NXP9890_DUMMY_SPKR_AMP_R_EN                 7
#define NXP9890_DUMMY_DAC_L_EN                      10
#define NXP9890_DUMMY_DAC_R_EN                      11


#define NXP9890_DUMMY_VENDOR_ID_REG 0X30
#define NXP9890_DUMMY_VENDOR_ID_VALUE     0XAABB

/* sample supported formats and rates */
#define NXP9890_FORMATS (GB_AUDIO_PCM_FMT_S8 | GB_AUDIO_PCM_FMT_S16_LE | \
                         GB_AUDIO_PCM_FMT_S24_LE)

#define NXP9890_STEREO_RATES (GB_AUDIO_PCM_RATE_8000 | GB_AUDIO_PCM_RATE_11025 |\
                              GB_AUDIO_PCM_RATE_16000 | GB_AUDIO_PCM_RATE_22050 |\
                              GB_AUDIO_PCM_RATE_32000 | GB_AUDIO_PCM_RATE_44100 |\
                              GB_AUDIO_PCM_RATE_48000 | GB_AUDIO_PCM_RATE_64000 |\
                              GB_AUDIO_PCM_RATE_88200 | GB_AUDIO_PCM_RATE_96000 |\
                              GB_AUDIO_PCM_RATE_176400 | GB_AUDIO_PCM_RATE_192000)

#endif /* _NXP9890_H_ */
