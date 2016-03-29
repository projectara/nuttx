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

/*
 * DWC OTG USB device registers dump utilities.
 * The driver of the IP already provide some facilities
 * to dump registers but enabling increase a lot the the size of
 * the flash, and so, use too munch memory.
 * The goal of this file is to provide facilities that doesn't increase
 * too munch the size of binary.
 */

#include <stdio.h>
#include <stdint.h>
#include <nuttx/util.h>
#include <arch/board/apbridgea_gadget.h>

#include "dwc_otg_pcd_debug.h"

#define getreg32(a)          (*(volatile uint32_t *)(a))

#define HSIC_BASE               0x40040000
#define EP_OFFSET               0x0020
#define DCFG                    0x0800
#define DCTL                    0x0804
#define DSTS                    0x0808
#define DIEPMSK                 0x0810
#define DOEPMSK                 0x0814
#define DAINT                   0x0818
#define DAINTMSK                0x081C
#define DVBUSDIS                0x0828
#define DVBUSPULSE              0x082C
#define DTHRCTL                 0x0830
#define DIEPEMPMSK              0x0834
#define DIEPCTL                 0x0900
#define DIEPINT                 0x0908
#define DIEPSIZ                 0x0910
#define DIEPDMA                 0x0914
#define DIEPXFSTS               0x0918
#define DIEPDMAB                0x091C
#define DOEPCTL                 0x0b00
#define DOEPINT                 0x0b08
#define DOEPSIZ                 0x0b10
#define DOEPDMA                 0x0b14
#define DOEPDMAB                0x0b1C

/**
 * Dump the value of one device endpoint register
 * @param n the offset of the register
 * @param epnum the number of the endpoint to dump
 */
#define DUMP_EP_REGISTER(n, epnum)                  \
    do {                                            \
        dwc_otg_read_reg(n + 0x20 * epnum);         \
        lowsyslog(#n": %08x\n", reg);               \
    } while(0)

/**
 * Dump the value of a global register
 * @param n the offset of the register
 */
#define DUMP_REGISTER(n)                            \
    do {                                            \
        dwc_otg_read_reg(n);                        \
        lowsyslog(#n": %08x\n", reg);               \
    } while(0)

#ifdef CONFIG_USB_DUMP_VERBOSE
/**
 * Display the name of bit if it's set.
 * You must call DUMP_EP_REGISTER() or DUMP_REGISTER(),
 * since this macro will use the dumped value to get the bit value.
 * @param n the name of bit
 * @param b the bit to display
 */
#define DUMP_BIT(n, b)                              \
    if (reg & BIT(b))                               \
        lowsyslog("  "#n"\n")

/**
 * Display the name of bits and their value.
 * You must call DUMP_EP_REGISTER() or DUMP_REGISTER(),
 * since this macro will use the dumped value to get the value.
 * @param n the name of the bits
 * @param o the offset of the bits
 * @param s number of bits to display
 */
#define DUMP_VALUE(n, o, s)                         \
    lowsyslog("  "#n": %d\n", (reg >> o) & ((1 << s) - 1))

/** Same has DUMP_VALUE but display the value in hex */
#define DUMP_HEX(n, o, s)                           \
    lowsyslog("  "#n": %x\n", (reg >> o) & ((1 << s) - 1))

/**
 * Display the name and the value of register in hex.
 * You must call DUMP_EP_REGISTER() or DUMP_REGISTER(),
 * since this macro will use the dumped value to get the value.
 * @param n the name of the dumped value
 */
#define DUMP_HEX32(n)                               \
    lowsyslog("  "#n": %x\n", reg)
#else
#define DUMP_BIT(n, b)
#define DUMP_VALUE(n, o, s)
#define DUMP_HEX(n, o, s)
#define DUMP_HEX32(n)
#endif

/** Store the value of dumped register */
static uint32_t reg;
/** Read the value of register and store it in reg variable */
static void dwc_otg_read_reg(uint32_t offset) {
    uint32_t addr = HSIC_BASE + offset;
    reg = getreg32(addr);
}

static void usb_dump_diepctl(int epnum)
{
    DUMP_EP_REGISTER(DIEPCTL, epnum);
    DUMP_BIT(EPEna, 31);
    DUMP_BIT(EPDis, 30);
    DUMP_VALUE(TxFNum, 22, 4);
    DUMP_BIT(Stall, 21);
    DUMP_VALUE(EPType, 18, 2);
    DUMP_BIT(NAKSTS, 17);
    DUMP_BIT(USBACTEP, 15);
    DUMP_VALUE(MPS, 0, 2);
}

static void usb_dump_diepint(int epnum)
{
    DUMP_EP_REGISTER(DIEPINT, epnum);
    DUMP_BIT(NYETIntrpt, 14);
    DUMP_BIT(NAKIntrpt, 13);
    DUMP_BIT(BbleErr, 12);
    DUMP_BIT(PktDrpSts, 11);
    DUMP_BIT(BNAIntr, 9);
    DUMP_BIT(TxfifoUndrn, 8);
    DUMP_BIT(TxFEmp, 7);
    DUMP_BIT(INEPNakEff, 6);
    DUMP_BIT(INTknEPMis, 5);
    DUMP_BIT(INTknTXFEmp, 4);
    DUMP_BIT(TimeOUT, 3);
    DUMP_BIT(AHBErr, 2);
    DUMP_BIT(EPDisbld, 1);
    DUMP_BIT(XferCompl, 0);
}

static void usb_dump_diepsiz(int epnum)
{
    DUMP_EP_REGISTER(DIEPSIZ, epnum);
    DUMP_VALUE(PktCnt, 19, 2);
    DUMP_VALUE(XferSize, 0, 7);
}

static void usb_dump_diepdma(int epnum)
{
    DUMP_EP_REGISTER(DIEPDMA, epnum);
    DUMP_HEX32(DMAAddr);
}

static void usb_dump_dtxfsts(int epnum)
{
    DUMP_EP_REGISTER(DIEPXFSTS, epnum);
    DUMP_VALUE(INEPTxFSpcAvail, 0, 16);
}

static void usb_dump_diepdmab(int epnum)
{
    DUMP_EP_REGISTER(DIEPDMAB, epnum);
    DUMP_HEX32(DMABufferAddr);
}

static void usb_dump_doepctl(int epnum)
{
    DUMP_EP_REGISTER(DOEPCTL, epnum);
    DUMP_BIT(EPEna, 31);
    DUMP_BIT(EPDis, 30);
    DUMP_BIT(Stall, 21);
    DUMP_BIT(Snp, 20);
    DUMP_VALUE(EPType, 18, 2);
    DUMP_BIT(NAKSTS, 17);
    DUMP_BIT(USBACTEP, 15);
    DUMP_VALUE(MPS, 0, 2);
}

static void usb_dump_doepint(int epnum)
{
    DUMP_EP_REGISTER(DOEPINT, epnum);
    DUMP_BIT(StupPktRcvd, 15);
    DUMP_BIT(NYETIntrpt, 14);
    DUMP_BIT(NAKIntrpt, 13);
    DUMP_BIT(BbleErr, 12);
    DUMP_BIT(PktDrpSts, 11);
    DUMP_BIT(BNAIntr, 9);
    DUMP_BIT(OutPktErr, 8);
    DUMP_BIT(Back2BackSETup, 6);
    DUMP_BIT(StsPhseRcvd, 5);
    DUMP_BIT(OUTTknEPdis, 4);
    DUMP_BIT(SetUp, 3);
    DUMP_BIT(AHBErr, 2);
    DUMP_BIT(EPDisbld, 1);
    DUMP_BIT(XferCompl, 0);
}

static void usb_dump_doepsiz(int epnum)
{
    DUMP_EP_REGISTER(DOEPSIZ, epnum);
    DUMP_VALUE(SUPCnt, 29, 2);
    DUMP_BIT(PktCnt, 19);
    DUMP_VALUE(XferSize, 0, 7);
}

static void usb_dump_doepdma(int epnum)
{
    DUMP_EP_REGISTER(DOEPDMA, epnum);
    DUMP_HEX32(DMAAddr);
}

static void usb_dump_doepdmab(int epnum)
{
    DUMP_EP_REGISTER(DOEPDMAB, epnum);
    DUMP_HEX32(DMABufferAddr);
}

/** Dump the USB endpoint registers */
void usb_dump_ep(int epnum)
{
    struct apbridge_dev_s *priv = get_apbridge_dev();
    struct usbdev_ep_s *ep = get_apbridge_ep(priv, epnum);

    /* Only dump existing endpoint */
    if (!ep) {
        return;
    }

    lowsyslog("EP%d\n", epnum);

    if (!epnum || USB_ISEPIN(ep->eplog)) {
        usb_dump_diepctl(epnum);
        usb_dump_diepint(epnum);
        usb_dump_diepsiz(epnum);
        usb_dump_diepdma(epnum);
        usb_dump_dtxfsts(epnum);
        usb_dump_diepdmab(epnum);
    }
    if (!epnum || USB_ISEPOUT(ep->eplog)) {
        usb_dump_doepctl(epnum);
        usb_dump_doepint(epnum);
        usb_dump_doepsiz(epnum);
        usb_dump_doepdma(epnum);
        usb_dump_doepdmab(epnum);
    }
}

static void usb_dump_dcfg(void)
{
    DUMP_REGISTER(DCFG);
    DUMP_VALUE(ResValid, 26, 6);
    DUMP_VALUE(PerSchIntvl, 24, 2);
    DUMP_BIT(DescDMA, 23);
    DUMP_BIT(ErraticIntMsk, 15);
    DUMP_BIT(XCVRDLY, 14);
    DUMP_BIT(EnDevOutNak, 13);
    DUMP_VALUE(PerFrInt, 11, 2);
    DUMP_VALUE(DevAddr, 4, 7);
    DUMP_BIT(Ena32KHzSusp, 3);
    DUMP_BIT(NZStsOUTHShk, 2);
    DUMP_VALUE(DevSpd, 0, 2);
}

static void dump_usb_dctl(void)
{
    DUMP_REGISTER(DCTL);
    DUMP_BIT(DeepSleepBESLReject, 18);
    DUMP_BIT(EnContOnBNA, 17);
    DUMP_BIT(NakOnBble, 16);
    DUMP_BIT(IgnrFrmNum, 15);
    DUMP_VALUE(GMC, 13, 2);
    DUMP_BIT(PWROnPrgDone, 11);
    DUMP_BIT(CGOUTNak, 10);
    DUMP_BIT(SGOUTNak, 9);
    DUMP_BIT(CGNPInNak, 8);
    DUMP_BIT(SGNPInNak, 7);
    DUMP_VALUE(TstCtl, 4, 3);
    DUMP_BIT(GOUTNakSts, 3);
    DUMP_BIT(GNPINNakSts, 2);
    DUMP_BIT(SftDiscon, 1);
    DUMP_BIT(RmtWkUpSig, 0);
}

static void dump_usb_dsts(void)
{
    DUMP_REGISTER(DSTS);
    DUMP_VALUE(DevLnSts, 22, 2);
    DUMP_VALUE(SOFFN, 8, 22);
    DUMP_BIT(ErrticErr, 3);
    DUMP_VALUE(EnumSpd, 1, 2);
    DUMP_BIT(SuspSts, 0);
}

static void dump_usb_diepmsk(void)
{
    DUMP_REGISTER(DIEPMSK);
    DUMP_BIT(NAKMsk, 13);
    DUMP_BIT(BNAInIntrMsk, 9);
    DUMP_BIT(TxfifoUndrnMsk, 8);
    DUMP_BIT(INEPNakEffMsk, 6);
    DUMP_BIT(INTknEPMisMsk, 5);
    DUMP_BIT(INTknTXFEmpMsk, 4);
    DUMP_BIT(TimeOUTMsk, 3);
    DUMP_BIT(AHBErrMsk, 2);
    DUMP_BIT(EPDisbldMsk, 1);
    DUMP_BIT(XferComplMsk, 0);
}

static void dump_usb_doepmsk(void)
{
    DUMP_REGISTER(DOEPMSK);
    DUMP_BIT(NYETKMsk, 14);
    DUMP_BIT(NAKMsk, 13);
    DUMP_BIT(BbleErrMsk, 12);
    DUMP_BIT(BnaOutIntrMsk, 9);
    DUMP_BIT(OutPktErrMsk, 8);
    DUMP_BIT(Back2BackSETup, 6);
    DUMP_BIT(StsPhseRcvdMsk, 5);
    DUMP_BIT(OUTTknEPdisMsk, 4);
    DUMP_BIT(SetUPMsk, 3);
    DUMP_BIT(AHBErrMsk, 2);
    DUMP_BIT(EPDisbldMsk, 1);
    DUMP_BIT(XferComplMsk, 0);
}

static void dump_usb_daint(void)
{
    DUMP_REGISTER(DAINT);
    DUMP_BIT(OutEPInt4, 20);
    DUMP_BIT(OutEPInt2, 28);
    DUMP_BIT(OutEPInt0, 16);
    DUMP_BIT(InEPInt3, 3);
    DUMP_BIT(InEPInt2, 1);
    DUMP_BIT(InEPInt0, 0);
}

static void dump_usb_daintmsk(void)
{
    DUMP_REGISTER(DAINTMSK);
    DUMP_BIT(OutEPIntMsk4, 20);
    DUMP_BIT(OutEPIntMsk2, 28);
    DUMP_BIT(OutEPIntMsk0, 16);
    DUMP_BIT(InEPIntMsk3, 3);
    DUMP_BIT(InEPIntMsk2, 1);
    DUMP_BIT(InEPIntMsk0, 0);
}

static void dump_usb_dthrctl(void)
{
    DUMP_REGISTER(DTHRCTL);
    DUMP_BIT(ArbPrkEn, 27);
    DUMP_VALUE(RxThrLen, 17, 9);
    DUMP_BIT(RxThrEn, 1);
    DUMP_VALUE(AHBThrRatio, 11, 2);
    DUMP_VALUE(TxThrLen, 2, 9);
    DUMP_BIT(ISOThrEn, 1);
    DUMP_BIT(NonISOThrEn, 0);
}

static void dump_usb_diepmpmsk(void)
{
    DUMP_REGISTER(DIEPEMPMSK);
    DUMP_HEX(InEpTxfEmpMsk, 0, 16);
}

/** Dump the USB device global registers */
void usb_dump_global_device(void)
{
    usb_dump_dcfg();
    dump_usb_dctl();
    dump_usb_dsts();
    dump_usb_diepmsk();
    dump_usb_doepmsk();
    dump_usb_daint();
    dump_usb_daintmsk();
    dump_usb_dthrctl();
    dump_usb_diepmpmsk();
}

#ifdef CONFIG_DWC_ENHANCED_BULK_OUT
extern void usbdev_dump_ring_descriptor(struct usbdev_ep_s *usb_ep);
int usb_dump_ring_descriptor(int epnum)
{
    struct usbdev_ep_s *ep;
    struct apbridge_dev_s *priv;

    priv = get_apbridge_dev();
    ep = get_apbridge_ep(priv, epnum);

    if (!ep) {
        return -EINVAL;
    }

    usbdev_dump_ring_descriptor(ep);

    return 0;
}
#endif
