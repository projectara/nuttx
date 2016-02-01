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

#include "tsb_es2_mphy_fixups.h"

#include <stdint.h>
#include <ara_debug.h>
#include "tsb_switch.h"

/*
 * This specifies an M-PHY "fixup"; i.e., a value that must be set to
 * a DME attribute while the link is still in PWM-G1, before
 * transitioning to HS link power modes.
 *
 * Use tsb_mphy_r1_fixup_is_magic() to test if a register 1 map fixup
 * must have its value drawn from M-PHY trim values or magic debug
 * registers (switch ports and bridges handle this case differently).
 */
struct tsb_mphy_fixup {
    uint16_t attrid;
    uint16_t select_index;
    uint32_t value;

#define TSB_MPHY_FIXUP_LAST     0x1
#define TSB_MPHY_FIXUP_MAGIC_R1 0x2
    uint32_t flags;
};

static inline int tsb_mphy_fixup_is_last(const struct tsb_mphy_fixup *fu) {
    return !!(fu->flags & TSB_MPHY_FIXUP_LAST);
}

static inline int tsb_mphy_r1_fixup_is_magic(const struct tsb_mphy_fixup *fu) {
    return !!(fu->flags & TSB_MPHY_FIXUP_MAGIC_R1);
}

/* This is provided to apply port-dependent M-PHY fixups, in case a
 * later switch increases SWITCH_UNIPORT_MAX */
#define ES2_SWITCH_NUM_UNIPORTS 14

#define TSB_MPHY_MAP (0x7F)
    #define TSB_MPHY_MAP_TSB_REGISTER_1 (0x01)
    #define TSB_MPHY_MAP_NORMAL         (0x00)
    #define TSB_MPHY_MAP_TSB_REGISTER_2 (0x81)

#define __TSB_MPHY_FIXUP(a, s, v, f)                                    \
    { .attrid = (a), .select_index = (s), .value = (v), .flags = (f) }
#define TSB_MPHY_FIXUP(a, s, v)                                         \
    __TSB_MPHY_FIXUP((a), (s), (v), 0)
#define TSB_MPHY_LAST_FIXUP(a, s, v)                                    \
    __TSB_MPHY_FIXUP((a), (s), (v), TSB_MPHY_FIXUP_LAST)
#define TSB_MPHY_MAGIC_R1_FIXUP()                                       \
    __TSB_MPHY_FIXUP(0, 0, 0, TSB_MPHY_FIXUP_MAGIC_R1)

const struct tsb_mphy_fixup tsb_register_1_map_mphy_fixups[] = {
    TSB_MPHY_MAGIC_R1_FIXUP(),

    TSB_MPHY_FIXUP(0x8004, 0, 0xCA),
    TSB_MPHY_FIXUP(0x8015, 0, 0x01),
    TSB_MPHY_FIXUP(0x8022, 0, 0x44),
    TSB_MPHY_FIXUP(0x8023, 0, 0x42),
    TSB_MPHY_FIXUP(0x80A2, 0, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 0, 0xA8),
    TSB_MPHY_FIXUP(0x80BA, 0, 0x20),

    TSB_MPHY_FIXUP(0x80A2, 1, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 1, 0xA8),
    TSB_MPHY_FIXUP(0x80BA, 1, 0x20),

    TSB_MPHY_FIXUP(0x8094, 4, 0x09),
    TSB_MPHY_FIXUP(0x809A, 4, 0x06),
    TSB_MPHY_FIXUP(0x809B, 4, 0x03),
    TSB_MPHY_FIXUP(0x809C, 4, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 4, 0x0F),
    TSB_MPHY_FIXUP(0x80B4, 4, 0x50),
    TSB_MPHY_FIXUP(0x80B6, 4, 0x82),
    TSB_MPHY_FIXUP(0x80B7, 4, 0x01),

    TSB_MPHY_FIXUP(0x8094, 5, 0x09),
    TSB_MPHY_FIXUP(0x809A, 5, 0x06),
    TSB_MPHY_FIXUP(0x809B, 5, 0x03),
    TSB_MPHY_FIXUP(0x809C, 5, 0x00),
    TSB_MPHY_FIXUP(0x80AA, 5, 0x0F),
    TSB_MPHY_FIXUP(0x80B4, 5, 0x50),
    TSB_MPHY_FIXUP(0x80B6, 5, 0x82),
    TSB_MPHY_FIXUP(0x80B7, 5, 0x01),

    TSB_MPHY_LAST_FIXUP(0x8000, 0, 0x01),
};

const struct tsb_mphy_fixup tsb_register_2_map_mphy_fixups[] = {
    TSB_MPHY_FIXUP(0x8000, 0, 0x02),

    TSB_MPHY_FIXUP(0x8080, 0, 0x20),
    TSB_MPHY_FIXUP(0x8081, 0, 0x03),

    TSB_MPHY_FIXUP(0x8080, 1, 0x20),
    TSB_MPHY_FIXUP(0x8081, 1, 0x03),

    TSB_MPHY_FIXUP(0x8082, 4, 0x3F),
    TSB_MPHY_FIXUP(0x8084, 4, 0x10),
    TSB_MPHY_FIXUP(0x8086, 4, 0x10),
    TSB_MPHY_FIXUP(0x8087, 4, 0x01),
    TSB_MPHY_FIXUP(0x8088, 4, 0x10),
    TSB_MPHY_FIXUP(0x808D, 4, 0x0B),
    TSB_MPHY_FIXUP(0x808E, 4, 0x00),
    TSB_MPHY_FIXUP(0x8094, 4, 0x00),
    TSB_MPHY_FIXUP(0x8096, 4, 0x00),
    TSB_MPHY_FIXUP(0x8098, 4, 0x08),
    TSB_MPHY_FIXUP(0x8099, 4, 0x50),

    TSB_MPHY_FIXUP(0x8082, 5, 0x3F),
    TSB_MPHY_FIXUP(0x8084, 5, 0x10),
    TSB_MPHY_FIXUP(0x8086, 5, 0x10),
    TSB_MPHY_FIXUP(0x8087, 5, 0x01),
    TSB_MPHY_FIXUP(0x8088, 5, 0x10),
    TSB_MPHY_FIXUP(0x808D, 5, 0x0B),
    TSB_MPHY_FIXUP(0x808E, 5, 0x00),
    TSB_MPHY_FIXUP(0x8094, 5, 0x00),
    TSB_MPHY_FIXUP(0x8096, 5, 0x00),
    TSB_MPHY_FIXUP(0x8098, 5, 0x08),
    TSB_MPHY_LAST_FIXUP(0x8099, 5, 0x50),
};

static uint32_t es2_mphy_trim_fixup_value(uint32_t mphy_trim[4], uint8_t port)
{
    switch (port) {
    case 0:
        return (mphy_trim[0] >> 1) & 0x1f;
    case 1:
        return (mphy_trim[0] >> 9) & 0x1f;
    case 2:
        return (mphy_trim[0] >> 17) & 0x1f;
    case 3:
        return (mphy_trim[0] >> 25) & 0x1f;
    case 4:
        return (mphy_trim[1] >> 1) & 0x1f;
    case 5:
        return (mphy_trim[1] >> 9) & 0x1f;
    case 6:
        return (mphy_trim[1] >> 17) & 0x1f;
    case 7:
        return (mphy_trim[1] >> 25) & 0x1f;
    case 8:
        return (mphy_trim[2] >> 1) & 0x1f;
    case 9:
        return (mphy_trim[2] >> 9) & 0x1f;
    case 10:
        return (mphy_trim[2] >> 17) & 0x1f;
    case 11:
        return (mphy_trim[2] >> 25) & 0x1f;
    case 12:
        return (mphy_trim[3] >> 1) & 0x1f;
    case 13:
        return (mphy_trim[3] >> 9) & 0x1f;
    default:                    /* can't happen */
        DEBUGASSERT(0);
        return 0xdeadbeef;
    }
}

int tsb_switch_es2_fixup_mphy(struct tsb_switch *sw) {
    uint32_t mphy_trim[4];
    int rc;
    uint8_t port;

    dbg_info("Applying M-PHY fixup settings to switch ports\n");

    rc = (switch_sys_ctrl_get(sw, SC_MPHY_TRIM0, mphy_trim + 0) ||
          switch_sys_ctrl_get(sw, SC_MPHY_TRIM1, mphy_trim + 1) ||
          switch_sys_ctrl_get(sw, SC_MPHY_TRIM2, mphy_trim + 2) ||
          switch_sys_ctrl_get(sw, SC_MPHY_TRIM3, mphy_trim + 3));
    if (rc) {
        dbg_error("%s(): can't read MPHY trim values: %d\n",
                  __func__, rc);
        return rc;
    }

    for (port = 0; port < ES2_SWITCH_NUM_UNIPORTS; port++) {
        const struct tsb_mphy_fixup *fu;

        /*
         * Apply the "register 2" map fixups.
         */
        rc = switch_dme_set(sw, port, TSB_MPHY_MAP, 0,
                            TSB_MPHY_MAP_TSB_REGISTER_2);
        if (rc) {
            dbg_error("%s(): failed to set switch map to \"TSB register 2\": %d\n",
                      __func__, rc);
            return rc;
        }
        fu = tsb_register_2_map_mphy_fixups;
        do {
            dbg_verbose("%s: port=%u, attrid=0x%04x, select_index=%u, value=0x%02x\n",
                        __func__, port, fu->attrid, fu->select_index,
                        fu->value);
            rc = switch_dme_set(sw, port, fu->attrid, fu->select_index,
                                fu->value);
            if (rc) {
                dbg_error("%s(): failed to apply register 2 map fixup: %d\n",
                          __func__, rc);
                return rc;
            }
        } while (!tsb_mphy_fixup_is_last(fu++));


        /*
         * Switch to "normal" map.
         */
        rc = switch_dme_set(sw, port, TSB_MPHY_MAP, 0,
                            TSB_MPHY_MAP_NORMAL);
        if (rc) {
            dbg_error("%s(): failed to set switch map to normal: %d\n",
                      __func__, rc);
            return rc;
        }

        /*
         * Apply the "register 1" map fixups.
         */
        rc = switch_dme_set(sw, port, TSB_MPHY_MAP, 0,
                          TSB_MPHY_MAP_TSB_REGISTER_1);
        if (rc) {
            dbg_error("%s(): failed to switch to TSB register 1 map: %d\n",
                      __func__, rc);
            return rc;
        }
        fu = tsb_register_1_map_mphy_fixups;
        do {
            if (tsb_mphy_r1_fixup_is_magic(fu)) {
                /* The "magic" fixups for the switch come from the
                 * M-PHY trim values. */
                uint32_t mpt = es2_mphy_trim_fixup_value(mphy_trim, port);
                dbg_verbose("%s: port=%u, attrid=0x%04x, select_index=%u, value=0x%02x\n",
                            __func__, port, 0x8002, 0, mpt);
                rc = switch_dme_set(sw, port, 0x8002, 0, mpt);
            } else {
                dbg_verbose("%s: port=%u, attrid=0x%04x, select_index=%u, value=0x%02x\n",
                            __func__, port, fu->attrid, fu->select_index,
                            fu->value);
                rc = switch_dme_set(sw, port, fu->attrid,
                                    fu->select_index, fu->value);
            }
            if (rc) {
                dbg_error("%s(): failed to apply register 1 map fixup: %d\n",
                          __func__, rc);
                return rc;
            }
        } while (!tsb_mphy_fixup_is_last(fu++));

        /*
         * Switch to "normal" map.
         */
        rc = switch_dme_set(sw, port, TSB_MPHY_MAP, 0,
                            TSB_MPHY_MAP_NORMAL);
        if (rc) {
            dbg_error("%s(): failed to re-set switch map to normal: %d\n",
                      __func__, rc);
            return rc;
        }
    }
    return 0;
}
