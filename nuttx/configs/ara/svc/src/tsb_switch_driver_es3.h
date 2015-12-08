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

/**
 * @author: Jean Pihet
 */

#ifndef  _TSB_SWITCH_DRIVER_ES3_H_
#define  _TSB_SWITCH_DRIVER_ES3_H_

#define SW_SPI_PORT_1               (1)
#define SW_SPI_PORT_2               (2)
#define SWITCH_SPI_FREQUENCY        13000000

/* Switch internal attributes */
#define SWPOR                       (0x0002)
#define SWINE                       (0x0015)
#define SWINS                       (0x0016)
#define SWRES                       (0x0020)
#define SWCCT                       (0x0022)
#define SWALS                       (0x0023)
#define SWLCE                       (0x0024)
#define SWLCS                       (0x0025)
#define SPICTLA                     (0x0040)
#define SPICTLB                     (0x0041)
#define SPIINTE                     (0x0042)
#define SPICEE                      (0x0043)
#define SPICES                      (0x0044)
#define SPI3EE                      (0x0045)
#define SPI3ES                      (0x0046)
#define SPI4EE                      (0x0047)
#define SPI4ES                      (0x0048)
#define SPI5EE                      (0x0049)
#define SPI5ES                      (0x004a)
#define SPITIMER                    (0x004c)

/* Switch internal attributes values */
#define SWINE_ENABLE_ALL            (0x7FFFB805)
#define SWLCE_ENABLE_ALL            (0x3FFF)
#define SPIINTE_ENABLE_ALL          (0x3)
#define SPICEE_ENABLE_ALL           (0xF)
#define SPI3EE_ENABLE_ALL           (0xEF)
#define SPI45EE_ENABLE_ALL          (0xFE0000FF)

/* QoS interrupt attributes */
#define QOSC_IE(i)                  (0x0080 + (i * 4))
#define QOSC_IS(i)                  (0x0081 + (i * 4))
#define RTBITSEL                    (0x00BC)

/* TSB attributes fields values */
#define TSB_INTERRUPTENABLE_ALL     (0xFFFF)
#define TSB_L4_INTERRUPTENABLE_ALL  (0x2800)
#define TSB_INTERRUPT_SPI3ES        (1 << 11)
#define TSB_INTERRUPT_SPI4ES        (1 << 12)
#define TSB_INTERRUPT_SPI5ES        (1 << 13)
#define TSB_INTERRUPT_SPICES        (1 << 15)
#define TSB_INTERRUPT_SPIPORT4_RX   (1 << 20)
#define TSB_INTERRUPT_SPIPORT5_RX   (1 << 21)
#define TSB_INTERRUPT_LSCINTERNAL   (1 << 30)
#define TSB_INTERRUPT_SWINTERNAL    (1 << 31)

/* System registers, accessible via switch_sys_ctrl_set, switch_sys_ctrl_get */
#define SC_SYSCLOCKENABLE           (0x0300)
#define SC_SYSCLOCKDIV0             (0x0410)
#define SC_SYSCLOCKDIV1             (0x0414)
#define SC_SYSCLOCKDIV2             (0x0418)
#define SC_LINKSTARTUPMODE          (0x0B00)
#define SC_PMUSTANDBYSS             (0x0C00)
#define SC_VDDVSAVE                 (0x0C10)
#define SC_VDDVRESTORE              (0x0C14)
#define SC_VDDVNPOWEROFF            (0x0C20)
#define SC_VDDVNPOWERON             (0x0C24)
#define SC_VDDNCPOWEREDSET          (0x0C30)
#define SC_VDDNCPOWEREDCLR          (0x0C34)
#define SC_VDDNSAVE                 (0x0C40)
#define SC_VDDNRESTORE              (0x0C44)
#define SC_VDDVNISOSET              (0x0C50)
#define SC_VDDVNISOCLR              (0x0C54)
#define SC_VDDVNHB8CLKEN            (0x0C60)
#define SC_VDDVNHB8CLKGATE          (0x0C64)
#define SC_VDDVNPOWERSTAT           (0x0D10)
#define SC_VDDVNCPOWEREDST          (0x0D20)
#define SC_VDDVNISOSTAT             (0x0D40)
#define SC_VDDVNPSWACK              (0x0E00)
#define SC_VDDVNSYSCLKOFFN          (0x0E10)

/* System registers values */
#define SC_RESET_CLK_SHARED         (1 << 15)
/*  Enable Shared domain, PMU; RT10b = 0 */
#define SC_SYSCLOCKENABLE_SH_PMU    (0x80008000)
/*  Enable Unipro and M-port clock */
#define SC_SYSCLOCKENABLE_PORT(i)   ((1 << (i + 16)) | (1 << i))
/*  Enable all Unipro and M-ports clocks */
#define SC_SYSCLOCKENABLE_ALL_PORTS (0x3FFF3FFF)
/*  Enable VDDn for M-port */
#define SC_VDDVN_PORT(i)            (1 << i)

#endif
