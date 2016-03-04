/*
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
*/

/**
Generate i2s data based on recorded sample
*/

#include <nuttx/config.h>
#include <nuttx/util.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <nuttx/time.h>
#include <gen_pcm.h>
#include <arch/armv7-m/byteorder.h> /* __swap16() */
#include "voice8k.h"
#include "tick8k.h"


#define SAMPLE_STREAM_SIGNATURE 0xFEEC
#define I2S_PLAY_RATE   48000

#define NUM_SUB_SAMPLES  (I2S_PLAY_RATE/SAMP_BUFF_RATE)

struct {
    uint32_t signature;
    uint32_t sample_index;
    uint32_t sub_sample_count;
    uint8_t  volume;
} sample_stream_state;

sem_t i2s_sample_done_sem;

/**
Initialize Sample Audio Output Stream

*/

int gen_audio_sample_init( uint8_t volume )
{
    int ret = 0;

    if (sample_stream_state.signature == SAMPLE_STREAM_SIGNATURE) {
        fprintf(stderr, "gen_audio_sample_inital already initialized.\n");
        ret = -EINVAL;
    } else if (volume > log_table_size -1) {
        fprintf(stderr, "Volume too large %d, must be at or under: %d\n",
                volume,
                log_table_size);
        ret = -EINVAL;
    } else if (SAMP_BUFF_BITS != 16) {
        /* ToDo add 24 and 32 bit sample handling */
        fprintf(stderr, "Only support 16 bit samples.\n");
        ret = -EINVAL;
    } else if (SAMP_BUFF_CHAN != 1) {
        /* ToDo add 2 channel sample handling */
        fprintf(stderr, "Only supporting 1 channel samples.\n");
        ret = -EINVAL;
    } else {
        sample_stream_state.signature = SAMPLE_STREAM_SIGNATURE;
        sample_stream_state.sub_sample_count = 0;
        sample_stream_state.sample_index = 0;
        sample_stream_state.volume = log_scale[volume];

        sem_init(&i2s_sample_done_sem, 0, 0);
    }



    return ret;
}

/**
Mark structure as unused
@return 0 OK
*/
int gen_audio_sample_deinit(void)
{
    int ret = 0;

    if (sample_stream_state.signature != SAMPLE_STREAM_SIGNATURE) {
        fprintf(stderr, "delete uninitialized\n");
        ret = -EINVAL;
    } else {
        sem_destroy(&i2s_sample_done_sem);

        /* just in case someone tries to use a stale structure. */
        sample_stream_state.signature = 0;
    }

    return ret;
}

static void get_sample(int16_t *current_sample, int16_t *sub_sample_step)
{
    int16_t next_sample;

    *current_sample = sample_buffer[sample_stream_state.sample_index];
    if (SAMP_BIG_ENDIAN) {
        *current_sample = __swap16(*current_sample);
    }
    /* apply volume scaling */
    if (sample_stream_state.volume == 0) {
        *current_sample = 0;
    } else {
        *current_sample = (int16_t)((*current_sample * sample_stream_state.volume) / 100);
    }

    if ((sample_stream_state.sample_index+1) == samples_in_sample_buffer) {
        next_sample = 0;
    } else {
        next_sample = sample_buffer[sample_stream_state.sample_index+1];
        if (SAMP_BIG_ENDIAN) {
            next_sample = __swap16(next_sample);
        }
        /* apply volume scaling */
        if (sample_stream_state.volume == 0) {
            next_sample = 0;
        } else {
            next_sample = (int16_t)((next_sample * sample_stream_state.volume) / 100);
        }
    }

    *sub_sample_step = (next_sample-*current_sample) / NUM_SUB_SAMPLES;
}


/**
Fill in 16bit audio data
@param buffer pointer to buffer to fill
@param buff_size  number bytes available, OUT
@param number of channels to fill
@return -ENXIO device was uninitialized
@return -EINVAL invalid argument
@return positive_value  number of bytes filled
*/
int fill_output_buff_with_samp( int16_t *buffer,
                                uint32_t *buff_size,
                                uint8_t number_of_channels)
{
    int ret = 0;
    int j;
    int16_t *p_fill;
    int16_t current_sample;
    int16_t sub_sample_step;
    uint32_t current_buff_size;

    if (!buffer ||
        !buff_size ||
        !number_of_channels) {
        fprintf(stderr, "invalid argument\n");
        ret = -EINVAL;
    } else if (sample_stream_state.signature != SAMPLE_STREAM_SIGNATURE) {
        fprintf(stderr, "uninitialized PCM data\n");
        ret = -ENXIO;
    } else {
        current_buff_size = *buff_size;
        p_fill = buffer;

        get_sample(&current_sample, &sub_sample_step);

        while (current_buff_size >= number_of_channels * sizeof(current_sample)) {

            if (sample_stream_state.sub_sample_count >= NUM_SUB_SAMPLES) {
                sample_stream_state.sub_sample_count = 0;
                sample_stream_state.sample_index++;
                if ( sample_stream_state.sample_index == samples_in_sample_buffer ) {
                    sample_stream_state.sample_index = 0;
                    sem_post(&i2s_sample_done_sem);
                    break;
                }

                get_sample(&current_sample, &sub_sample_step);
            }

            sample_stream_state.sub_sample_count++;

            for (j = 0; j < number_of_channels; j++) {
                *p_fill = current_sample + (sample_stream_state.sub_sample_count * sub_sample_step);
                p_fill++;
            }

            current_buff_size -= number_of_channels*sizeof(current_sample);
        }

        /* update number of filled */
        ret = *buff_size - current_buff_size;
    }

    return ret;
}
