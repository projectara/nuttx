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
 * Author: Fabien Parent <fparent@baylibre.com>
 */

.global tsb_enter_standby
.global tsb_leave_standby

.syntax unified
.thumb
.text

.thumb_func
tsb_enter_standby:
    ldr r1, =saved_context
    stm r1!, {r4 - r12, r14}
    mrs r2, primask
    mrs r3, faultmask
    mrs r4, basepri
    mrs r5, control
    mrs r6, psp
    mrs r7, msp
    mrs r8, xpsr
    stm r1, {r2 - r8}

    bx r0

.thumb_func
tsb_leave_standby:
    ldr r1, =saved_context
    add r0, r1, #40 // skip the general purpose registers
    ldm r0, {r2 - r8}
    msr primask, r2
    msr faultmask, r3
    msr basepri, r4
    msr control, r5
    msr psp, r6
    msr msp, r7
    msr xpsr, r8
    ldm r1, {r4 - r12, r14}

    bx lr

.bss
saved_context:
.skip 0x44 // 17 registers
