/* $OpenBSD$ */
/*-
 * SPDX-License-Identifier: BSD-4-Clause
 *
 * Copyright (c) Comtrol Corporation <support@comtrol.com>
 * All rights reserved.
 *
 * PCI-specific part separated from:
 * sys/i386/isa/rp.c,v 1.33 1999/09/28 11:45:27 phk Exp
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted prodived that the follwoing conditions
 * are met.
 * 1. Redistributions of source code must retain the above copyright 
 *    notive, this list of conditions and the following disclainer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials prodided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *       This product includes software developed by Comtrol Corporation.
 * 4. The name of Comtrol Corporation may not be used to endorse or 
 *    promote products derived from this software without specific 
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY COMTROL CORPORATION ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL COMTROL CORPORATION BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, LIFE OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/fcntl.h>
#include <sys/malloc.h>
#include <sys/tty.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <machine/bus.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#define ROCKET_C
#include <dev/ic/rpreg.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

int rp_pci_match(struct device *, void *, void *);
void rp_pci_attach(struct device *, struct device *, void *);

struct rp_pci_softc {
	struct rp_softc		sc_rp;	/* real softc */

	bus_space_tag_t		sc_iot;	/* PLX i/o tag */
	bus_space_handle_t	sc_ioh;	/* PLX i/o handle */
};

const struct cfattach rp_pci_ca = {
	sizeof(struct rp_pci_softc), rp_pci_match, rp_pci_attach
};

const struct pci_matchid rp_pci_devices[] = {
	{ PCI_VENDOR_COMCORP, PCI_PRODUCT_COMCORP_ROCKETPORT_16 },
};

/* PCI IDs  */
#define RP_VENDOR_ID		0x11FE
#define RP_DEVICE_ID_32I	0x0001
#define RP_DEVICE_ID_8I		0x0002
#define RP_DEVICE_ID_16I	0x0003
#define RP_DEVICE_ID_4Q		0x0004
#define RP_DEVICE_ID_8O		0x0005
#define RP_DEVICE_ID_8J		0x0006
#define RP_DEVICE_ID_4J		0x0007
#define RP_DEVICE_ID_6M		0x000C
#define RP_DEVICE_ID_4M		0x000D
#define RP_DEVICE_ID_UPCI_32	0x0801
#define RP_DEVICE_ID_UPCI_16	0x0803
#define RP_DEVICE_ID_UPCI_8O	0x0805

/*
 * MUDBAC remapped for PCI
 */

#define _CFG_INT_PCI	0x40
#define _PCI_INT_FUNC	0x3A

#define PCI_STROB	0x2000
#define INTR_EN_PCI	0x0010

int rp_pci_match(struct device *, void *, void *);
void rp_pci_attach(struct device *, struct device *, void *);
int rp_pci_init_controller(struct rp_softc *, int, int, uint8_t, int, int);

int rp_pci_aiop2rid(int, int);				/* XXX */
int rp_pci_aiop2off(int, int);				/* XXX */
unsigned char rp_pci_ctlmask(struct rp_softc *);	/* XXX */

/*
 * The following functions are the pci-specific part of rp driver.
 */

int
rp_pci_match(struct device *parent, void *match, void *aux)
{
	return (pci_matchbyid((struct pci_attach_args *)aux, rp_pci_devices,
	    nitems(rp_pci_devices)));
}

void
rp_pci_attach(struct device *parent, struct device *self, void *aux)
{
	struct rp_softc *sc = (struct rp_softc *)self;
	struct pci_attach_args *pa = aux;
	int num_ports, num_aiops;
	int aiop;
	pcireg_t maptype;

	sc->aiop2rid = rp_pci_aiop2rid;
	sc->aiop2off = rp_pci_aiop2off;
	sc->ctlmask = rp_pci_ctlmask;

	sc->bus_ctlp = NULL;

	maptype = pci_mapreg_type(pa->pa_pc, pa->pa_tag, RP_PCI_BAR_1);
	if (pci_mapreg_map(pa, RP_PCI_BAR_1, maptype, 0, &sc->sc_iot,
	    &sc->sc_ioh, NULL, &sc->sc_ios, 0) != 0) {
		printf(" unable to map registers\n");
		return;
	}

	num_aiops = rp_pci_init_controller(sc, RP_MAX_AIOPS_PER_BOARD, 0,
	    FREQ_DIS, 0, PCI_PRODUCT(pa->pa_id));

	num_ports = 0;
	for (aiop = 0; aiop < num_aiops; aiop++) {
		rp_reset_aiop_by_num(sc, aiop);
		num_ports += sc->AiopNumChan[aiop];
	}

	rp_attach(sc, num_aiops, num_ports);
}

int
rp_pci_init_controller(struct rp_softc *sc, int AiopNum, int IRQNum,
    uint8_t Frequency, int PeriodicOnly, int VendorDevice)
{
	int i;

	/* Strobe the MUDBAC's End Of Interrupt bit */
	rp_writeio2(sc, 0, _PCI_INT_FUNC, PCI_STROB);

	/* Init AIOPs */
	sc->NumAiop = 0;
	for (i = 0; i < AiopNum; i++) {
		/*device_printf(sc->dev, "aiop %d.\n", i);*/
		sc->AiopID[i] = rp_read_aiopid(sc, i);	/* read AIOP ID */
		/*device_printf(sc->dev, "ID = %d.\n", sc->AiopID[i]);*/
		if (sc->AiopID[i] == RP_AIOPID_NULL)	/* if AIOP does not exist */
			break;				/* done looking for AIOPs */

		switch (VendorDevice) {
		case RP_DEVICE_ID_4Q:
		case RP_DEVICE_ID_4J:
		case RP_DEVICE_ID_4M:
			sc->AiopNumChan[i] = 4;
			break;
		case RP_DEVICE_ID_6M:
			sc->AiopNumChan[i] = 6;
			break;
		case RP_DEVICE_ID_8O:
		case RP_DEVICE_ID_8J:
		case RP_DEVICE_ID_8I:
		case RP_DEVICE_ID_16I:
		case RP_DEVICE_ID_32I:
			sc->AiopNumChan[i] = 8;
			break;
		default:
			sc->AiopNumChan[i] = rp_read_aiop_numchan(sc, i);
			break;
		}
#ifdef RP_DEBUG
		printf("%s %d channels\n", sc->sc_dev.dv_xname,
		    sc->AiopNumChan[i]);
#endif
		rp_writeaiop2(sc, i, _INDX_ADDR, _CLK_PRE);	/* clock prescaler */
#ifdef RP_DEBUG
		printf("%s configuring clock prescaler\n", sc->sc_dev.dv_xname);
#endif
		rp_writeaiop1(sc, i, _INDX_DATA, CLOCK_PRESC);
#ifdef RP_DEBUG
		printf("%s configured clock prescaler\n", sc->sc_dev.dv_xname);
#endif
		sc->NumAiop++;	/* bump count of AIOPs */
	}

	if (sc->NumAiop == 0)
		return (-1);
	else
		return (sc->NumAiop);
}

/*
 * ARGSUSED
 * Maps (aiop, offset) to rid.
 */
int
rp_pci_aiop2rid(int aiop, int offset)
{
	/* Always return zero for a PCI controller. */
	return 0;
}

/*
 * ARGSUSED
 * Maps (aiop, offset) to the offset of resource.
 */
int
rp_pci_aiop2off(int aiop, int offset)
{
	/* Each AIOP reserves 0x40 bytes. */
	return (aiop * 0x40 + offset);
}

/* Read the int status for a PCI controller. */
unsigned char
rp_pci_ctlmask(struct rp_softc *sc)
{
	/*
	 * Get the controller interrupt status
	 *
	 * Returns the controller interrupt status in the lower 4 bits.  Bits 0
	 * through 3 represent AIOP's 0 through 3 respectively.  If a bit is
	 * set that AIOP is interrupting.  Bits 4 through 7 will always be
	 * cleared.
	 */
	return ((rp_readio2(sc, 0, _PCI_INT_FUNC) >> 8) & 0x1f);
}
