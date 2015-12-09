/*
 * Copyright (c) 2014-2015 Google, Inc.
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
 * * may be used to endorse or promote products derived from this
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

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/power/pm.h>

static int tsb_pm_curr_state = PM_NORMAL;

/*
 * Called from up_idle(). Checks the power state suggested by the power
 * management algorithm, then tries to change the power state of all
 * power-managed drivers. If that succeeds - the bridge-specific power
 * management code is executed.
 */
void up_idlepm(void)
{
    int ret, newstate;
    irqstate_t flags;

    newstate = pm_checkstate();
    if (newstate != tsb_pm_curr_state) {
        flags = irqsave();

        ret = pm_changestate(newstate);
        if (ret < 0) {
            /* Restore previous state on failure. */
            (void)pm_changestate(tsb_pm_curr_state);
        } else {
            tsb_pm_curr_state = newstate;

            /* This is where bridge-specific pm should be done. */
            switch (newstate) {
            case PM_NORMAL:
                break;
            case PM_IDLE:
                break;
            case PM_STANDBY:
                break;
            case PM_SLEEP:
                break;
            default:
                break;
            }
        }

        irqrestore(flags);
    }
}

void up_pminitialize(void)
{
    pm_initialize();
}
