/* $OpenBSD$ */
/*-
 * SPDX-License-Identifier: BSD-4-Clause
 *
 * Copyright (c) Comtrol Corporation <support@comtrol.com>
 * All rights reserved.
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
 *
 */

/*
 * Begin OS-specific defines for RocketPort
 */

#define rp_readio(size, sc, rid, offset) \
	(bus_space_read_##size((sc)->sc_iot, (sc)->sc_ioh, (offset)))
#define rp_readmultiio(size, sc, rid, offset, addr, count) \
	(bus_space_read_multi_##size((sc)->sc_iot, (sc)->sc_ioh, (offset), (addr), (count)))
#define rp_writeio(size, sc, rid, offset, data) \
	(bus_space_write_##size((sc)->sc_iot, (sc)->sc_ioh, (offset), (data)))
#define rp_writemultiio(size, sc, rid, offset, addr, count) \
	(bus_space_write_multi_##size((sc)->sc_iot, (sc)->sc_ioh, (offset), (addr), (count)))

#define rp_readio1(sc, rid, offset) rp_readio(1, (sc), (rid), (offset))
#define rp_readio2(sc, rid, offset) rp_readio(2, (sc), (rid), (offset))
#define rp_readio4(sc, rid, offset) rp_readio(4, (sc), (rid), (offset))

#define rp_writeio1(sc, rid, offset, data) \
	rp_writeio(1, (sc), (rid), (offset), (data))
#define rp_writeio2(sc, rid, offset, data) \
	rp_writeio(2, (sc), (rid), (offset), (data))
#define rp_writeio4(sc, rid, offset, data) \
	rp_writeio(4, (sc), (rid), (offset), (data))

#define rp_readmultiio1(sc, rid, offset, addr, count) \
	rp_readmultiio(1, (sc), (rid), (offset), (addr), (count))
#define rp_readmultiio2(sc, rid, offset, addr, count) \
	rp_readmultiio(2, (sc), (rid), (offset), (addr), (count))
#define rp_readmultiio4(sc, rid, offset, addr, count) \
	rp_readmultiio(4, (sc), (rid), (offset), (addr), (count))

#define rp_writemultiio1(sc, rid, offset, addr, count) \
	rp_writemultiio(1, (sc), (rid), (offset), (addr), (count))
#define rp_writemultiio2(sc, rid, offset, addr, count) \
	rp_writemultiio(2, (sc), (rid), (offset), (addr), (count)) 
#define rp_writemultiio4(sc, rid, offset, addr, count) \
	rp_writemultiio(4, (sc), (rid), (offset), (addr), (count)) 

#define rp_readaiop1(sc, aiop, offset) \
	(rp_readio1((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset)))
#define rp_readaiop2(sc, aiop, offset) \
	(rp_readio2((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset)))
#define rp_readaiop4(sc, aiop, offset) \
	(rp_readio4((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset)))
#define rp_readmultiaiop1(sc, aiop, offset, addr, count) \
	(rp_readmultiio1((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), addr, count))
#define rp_readmultiaiop2(sc, aiop, offset, addr, count) \
	(rp_readmultiio2((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), addr, count))
#define rp_readmultiaiop4(sc, aiop, offset, addr, count) \
	(rp_readmultiio4((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), addr, count))
#define rp_writeaiop1(sc, aiop, offset, data) \
	(rp_writeio1((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), data))
#define rp_writeaiop2(sc, aiop, offset, data) \
	(rp_writeio2((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), data))
#define rp_writeaiop4(sc, aiop, offset, data) \
	(rp_writeio4((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), data))
#define rp_writemultiaiop1(sc, aiop, offset, addr, count) \
	(rp_writemultiio1((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), addr, count))
#define rp_writemultiaiop2(sc, aiop, offset, addr, count) \
	(rp_writemultiio2((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), addr, count))
#define rp_writemultiaiop4(sc, aiop, offset, addr, count) \
	(rp_writemultiio4((sc), (sc)->aiop2rid(aiop, offset), (sc)->aiop2off(aiop, offset), addr, count))

#define rp_readch1(ch, offset) \
	(rp_readaiop1((ch)->sc, (ch)->AiopNum, offset))
#define rp_readch2(ch, offset) \
	(rp_readaiop2((ch)->sc, (ch)->AiopNum, offset))
#define rp_readch4(ch, offset) \
	(rp_readaiop4((ch)->sc, (ch)->AiopNum, offset))
#define rp_readmultich1(ch, offset, addr, count) \
	(rp_readmultiaiop1((ch)->sc, (ch)->AiopNum, offset, addr, count))
#define rp_readmultich2(ch, offset, addr, count) \
	(rp_readmultiaiop2((ch)->sc, (ch)->AiopNum, offset, addr, count))
#define rp_readmultich4(ch, offset, addr, count) \
	(rp_readmultiaiop4((ch)->sc, (ch)->AiopNum, offset, addr, count))
#define rp_writech1(ch, offset, data) \
	(rp_writeaiop1((ch)->sc, (ch)->AiopNum, offset, data))
#define rp_writech2(ch, offset, data) \
	(rp_writeaiop2((ch)->sc, (ch)->AiopNum, offset, data))
#define rp_writech4(ch, offset, data) \
	(rp_writeaiop4((ch)->sc, (ch)->AiopNum, offset, data))
#define rp_writemultich1(ch, offset, addr, count) \
	(rp_writemultiaiop1((ch)->sc, (ch)->AiopNum, offset, addr, count))
#define rp_writemultich2(ch, offset, addr, count) \
	(rp_writemultiaiop2((ch)->sc, (ch)->AiopNum, offset, addr, count))
#define rp_writemultich4(ch, offset, addr, count) \
	(rp_writemultiaiop4((ch)->sc, (ch)->AiopNum, offset, addr, count))

/*
 * Port number on card encoded in low 5 bits
 * card number in next 2 bits (only space for 4 cards)
 * high bit reserved for dialout flag
 */
#define RP_PORT(x) (minor(x) & 0xf)
#define RP_CARD(x) ((minor(x) >> 5) & 3)

/*
 * End of OS-specific defines
 */

#define RP_CTL_SIZE		 4
#define RP_AIOP_CTL_SIZE	 4
#define RP_CHAN_AIOP_SIZE	 8
#define RP_MAX_PORTS_PER_AIOP	 8
#define RP_MAX_AIOPS_PER_BOARD	 4
#define RP_MAX_PORTS_PER_BOARD	32

/* AIOP ID numbers, identifies AIOP type implementing channel */
#define RP_AIOPID_NULL -1	/* no AIOP or channel exists */
#define RP_AIOPID_0001 0x0001	/* AIOP release 1 */

/*
 * Global Register Offsets - Direct Access - Fixed values
 */
#define _CMD_REG   0x38   /* Command Register		 8    Write */
#define _INT_CHAN  0x39   /* Interrupt Channel Register  8    Read */
#define _INT_MASK  0x3A   /* Interrupt Mask Register	 8    Read / Write */
#define _UNUSED    0x3B   /* Unused			 8 */
#define _INDX_ADDR 0x3C   /* Index Register Address	 16   Write */
#define _INDX_DATA 0x3E   /* Index Register Data	 8/16 Read / Write */

/*
 * Channel Register Offsets for 1st channel in AIOP - Direct Access
 */
#define _TD0	   0x00  /* Transmit Data		16   Write */
#define _RD0	   0x00  /* Receive Data		16   Read */
#define _CHN_STAT0 0x20  /* Channel Status		8/16 Read / Write */
#define _FIFO_CNT0 0x10  /* Transmit/Receive FIFO Count 16   Read */
#define _INT_ID0   0x30  /* Interrupt Identification	8    Read */

/*
 * Tx Control Register Offsets - Indexed - External - Fixed
 */
#define _TX_ENBLS  0x980    /* Tx Processor Enables Register 8 Read / Write */
#define _TXCMP1    0x988    /* Transmit Compare Value #1     8 Read / Write */
#define _TXCMP2    0x989    /* Transmit Compare Value #2     8 Read / Write */
#define _TXREP1B1  0x98A    /* Tx Replace Value #1 - Byte 1  8 Read / Write */
#define _TXREP1B2  0x98B    /* Tx Replace Value #1 - Byte 2  8 Read / Write */
#define _TXREP2    0x98C    /* Transmit Replace Value #2     8 Read / Write */

/*
 * Receive FIFO
 */
#define RXFIFO_DATA	0x5f
#define RXFIFO_OUT	0x5c
#define RXFIFO_EN	0x08
#define RXFIFO_DIS	0xa7

/*
 * Memory Controller Register Offsets - Indexed - External - Fixed
 */
#define _RX_FIFO    0x000    /* Rx FIFO */
#define _TX_FIFO    0x800    /* Tx FIFO */
#define _RXF_OUTP   0x990    /* Rx FIFO OUT pointer	16 Read / Write */
#define _RXF_INP    0x992    /* Rx FIFO IN pointer	16 Read / Write */
#define _TXF_OUTP   0x994    /* Tx FIFO OUT pointer	 8 Read / Write */
#define _TXF_INP    0x995    /* Tx FIFO IN pointer	 8 Read / Write */
#define _TXP_CNT    0x996    /* Tx Priority Count	 8 Read / Write */
#define _TXP_PNTR   0x997    /* Tx Priority Pointer	 8 Read / Write */

#define PRI_PEND    0x80     /* Priority data pending (bit7, Tx pri cnt) */
#define TXFIFO_SIZE 255      /* size of Tx FIFO */
#define RXFIFO_SIZE 1023     /* size of Rx FIFO */

/*
 * Tx Priority Buffer - Indexed - External - Fixed
 */
#define _TXP_BUF    0x9C0    /* Tx Priority Buffer  32	Bytes	Read / Write */
#define TXP_SIZE    0x20     /* 32 bytes */

/*
 * Channel Register Offsets - Indexed - Internal - Fixed
 */
#define _TX_CTRL	0xFF0	/* Transmit Control	16  Write */
#define _RX_CTRL	0xFF2	/* Receive Control 	 8  Write */
#define _BAUD		0xFF4	/* Baud Rate		16  Write */
#define _CLK_PRE	0xFF6	/* Clock Prescaler 	 8  Write */

#define CLOCK_PRESC 0x19	/* mod 9 (divide by 10) prescale */

#define RP_BRD50	4607
#define RP_BRD75	3071
#define RP_BRD110	2094
#define RP_BRD134	1712
#define RP_BRD150	1535
#define RP_BRD200	1151
#define RP_BRD300	 767
#define RP_BRD600	 383
#define RP_BRD1200 	 191
#define RP_BRD1800 	 127
#define RP_BRD2000 	 114
#define RP_BRD2400 	  95
#define RP_BRD3600 	  64
#define RP_BRD4800 	  47
#define RP_BRD7200 	  31
#define RP_BRD9600 	  23
#define RP_BRD14400	  15
#define RP_BRD19200	  11
#define RP_BRD38400	   5
#define RP_BRD57600	   3
#define RP_BRD76800	   2
#define RP_BRD115200	   1
#define RP_BRD230400	   0

#define STMPARITY	0x01	/* parity error */
#define STMRCVROVR	0x02	/* receiver over run error */
#define STMFRAME	0x04	/* framing error */
#define STMBREAK	0x08	/* BREAK */
#define STMERROR	(STMBREAK | STMFRAME | STMPARITY)
#define STMPARITYH	0x100	/* parity error */
#define STMRCVROVRH	0x200	/* receiver over run error */
#define STMFRAMEH	0x400	/* framing error */
#define STMBREAKH	0x800	/* BREAK */
#define STMERRORH	(STMBREAKH | STMFRAMEH | STMPARITYH)

#define RDA		0x01	/* Rx data available */
#define TXSHRMT		0x02	/* Tx shift register is empty */
#define TXFIFOMT	0x04	/* Tx FIFO is empty */
#define CD_ACT		0x08	/* CD input asserted */
#define DSR_ACT		0x10	/* DSR input asserted */
#define CTS_ACT		0x20	/* CTS input asserted */
#define DRAINED		(TXFIFOMT | TXSHRMT)	/* indicates Tx is drained */

#define RXPARITY	0x0100	/* received parity error */
#define RXFRAME		0x0200	/* received framing error */
#define RXBREAK		0x0400	/* received BREAK */
#define RX1MATCH	0x0800	/* receive compare byte 1 match */
#define RX2MATCH	0x1000	/* receive compare byte 2 match */
#define RXFOVERFL	0x2000	/* receive FIFO overflow */
#define STATMODE	0x8000	/* status mode enable bit */
#define STATERROR (RXBREAK | RXFRAME | RXPARITY)

#define DATA8BIT	0x01	/* 8 bit data (0 = 7 bit data) */
#define EVEN_PAR	0x02	/* even parity (0 = odd parity) */
#define PARITY_EN	0x04	/* enable parity (0 = no parity) */
#define STOP2		0x08	/* enable 2 stop bits (0 = 1 stop) */
#define TXINT_EN	0x10	/* transmit interrupt enable */
#define RTSTOG_EN	0x40	/* RTS toggle enable bit */
#define CTSFC_EN	0x80	/* CTS flow control enable bit */

#define TX_ENABLE	0x01	/* enable transmitter */
#define SET_RTS		0x02	/* assert RTS */
#define SET_DTR		0x04	/* assert DTR */
#define LOCALLOOP	0x08	/* local loopback set for test */
#define SETBREAK	0x10	/* send break condition (must clear) */

#define TRIG_NO		0x00	/* Rx FIFO trigger level 0 (no trigger) */
#define MCINT_EN	0x01	/* modem change interrupt enable */
#define RXINT_EN	0x02	/* Rx interrupt enable */
#define SRCINT_EN	0x04	/* special Rx condition interrupt enable */
#define TRIG_1		0x08	/* trigger level 1 char */
#define TRIG_1_2	0x10	/* trigger level 1/2 */
#define TRIG_7_8	0x18	/* trigger level 7/8 */
#define TRIG_MASK	0x18	/* trigger level mask */
#define RXPROC_EN	0x20	/* receive processor enable */
#define RTSFC_EN	0x40	/* RTS flow control enable */

#define DELTA_DSR	0x01	/* DSR change interrupt */
#define DELTA_CTS	0x02	/* CTS change interrupt */
#define DELTA_CD	0x04	/* CD change interrupt */
#define SRC_INT		0x08	/* special receive condition interrupt */
#define TXFIFO_MT	0x10	/* Tx FIFO empty interrupt */
#define RXF_TRIG	0x20	/* Rx FIFO trigger level interrupt */

#define COMP1_EN	0x01	/* compare byte 1 enable */
#define COMP2_EN	0x02	/* compare byte 2 enable */
#define IGN1_EN		0x04	/* ignore byte 1 enable */
#define IGN2_EN		0x08	/* ignore byte 2 enable */
#define REP1W2_EN	0x10	/* replace byte 1 with 2 bytes enable */

#define RESET_ALL	0x80	/* reset AIOP (all channels) */
#define TXOVERIDE	0x40	/* Transmit software off override */
#define RESETUART	0x20	/* reset channel's UART */
#define RESTXFCNT	0x10	/* reset channel's Tx FIFO count register */
#define RESRXFCNT	0x08	/* reset channel's Rx FIFO count register */

#define INTSTAT0	0x01	/* AIOP 0 interrupt status */
#define INTSTAT1	0x02	/* AIOP 1 interrupt status */
#define INTSTAT2	0x04	/* AIOP 2 interrupt status */
#define INTSTAT3	0x08	/* AIOP 3 interrupt status */

#define INTR_EN		0x08	/* allow interrupts to host */
#define INT_STROB	0x04	/* strobe and clear interrupt line (EOI) */

#define CHAN0_EN	0x01	/* enable AIOP 0 */
#define CHAN1_EN	0x02	/* enable AIOP 1 */
#define CHAN2_EN	0x04	/* enable AIOP 2 */
#define CHAN3_EN	0x08	/* enable AIOP 3 */

#define FREQ_DIS	0x00
#define FREQ_9HZ	0x10
#define FREQ_17HZ	0x20
#define FREQ_34HZ	0x30
#define FREQ_69HZ	0x40
#define FREQ_137HZ	0x50
#define FREQ_274HZ	0x60
#define PERIODIC_ONLY	0x80	/* only PERIODIC interrupt */

#define CHANINT_EN	0x0100	/* flags to enable/disable channel ints */

#define RDATASIZE	72
#define RREGDATASIZE	52

#define RP_PCI_BAR_1	PCI_MAPREG_START
#define RP_PCI_BAR_2	PCI_MAPREG_START + 8

struct rp_softc;
struct rp_chan;

/* The types of bus-specific methods */
typedef int rp_aiop2rid_t(int, int);
typedef int rp_aiop2off_t(int, int);
typedef unsigned char rp_ctlmask_t(struct rp_softc *);

/* Controller level information structure */
struct rp_softc {
	/* Device and resource management */
	struct device	sc_dev;		/* device */

	int		NumAiop;
	int		AiopID[RP_AIOP_CTL_SIZE];
	int		AiopNumChan[RP_AIOP_CTL_SIZE];

        struct mutex	hwmtx;		/* Spinlock protecting hardware. */
	int		hwmtx_init;
//	int		free;
	int		num_ports;

	void			*sc_ih;
	bus_space_tag_t		 sc_iot;
	bus_space_handle_t	 sc_ioh;
	bus_size_t		 sc_ios;

	struct rp_port	*sc_rp;		/* port */
	struct cdev    **dev_nodes;	/* Device nodes */
	void		*bus_ctlp;	/* Bus-specific properties */

	/* Bus-specific methods */
	rp_aiop2rid_t	*aiop2rid;	/* (aiop, offset) -> rid */
	rp_aiop2off_t	*aiop2off;	/* (aiop, offset) -> off */
	rp_ctlmask_t	*ctlmask;	/* Int status */
};

/* Channel level information structure */
struct rp_chan
{
	struct rp_softc	*sc;
	int		 AiopNum;
	int		 ChanID;
	int		 ChanNum;

	uint32_t	 TxFIFO;
	uint32_t	 TxFIFOPtrs;
	uint32_t	 RxFIFO;
	uint32_t	 RxFIFOPtrs;
	uint32_t	 TxPrioCnt;
	uint32_t	 TxPrioPtr;
	uint32_t	 TxPrioBuf;

	uint8_t		 R[RREGDATASIZE];

	uint8_t		 BaudDiv[4];
	uint8_t		 TxControl[4];
	uint8_t		 RxControl[4];
	uint8_t		 TxEnables[4];
	uint8_t		 TxCompare[4];
	uint8_t		 TxReplace1[4];
	uint8_t		 TxReplace2[4];
};

#define CHNOFF_TXRXDATA(ch)	((ch)->ChanNum * 2 + _TD0)
#define CHNOFF_CHANSTAT(ch)	((ch)->ChanNum * 2 + _CHN_STAT0)
#define CHNOFF_TXRXCOUNT(ch)	((ch)->ChanNum * 2 + _FIFO_CNT0)
#define CHNOFF_INTID(ch)	((ch)->ChanNum     + _INT_ID0)

/* Clear the DTR output */
#define rp_clr_DTR(ch) do {					\
	(ch)->TxControl[3] &= ~SET_DTR;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/* Clear the RTS output */
#define rp_clr_RTS(ch) do {					\
	(ch)->TxControl[3] &= ~SET_RTS;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/* Clear any existing transmit software flow control off condition */
#define rp_clr_tx_xoff(ch) do {						\
	rp_writech1(ch, _CMD_REG, TXOVERIDE | (uint8_t)(ch)->ChanNum);	\
	rp_writech1(ch, _CMD_REG, (uint8_t)(ch)->ChanNum);		\
} while (0)

/* Disable output flow control using CTS */
#define rp_disable_CTS_flowctl(ch) do {				\
	(ch)->TxControl[2] &= ~CTSFC_EN;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Function sSetParity() can be used in place of functions sEnParity(),
 * sDisParity(), rp_set_odd_parity(), and sSetEvenParity().
 */
#define rp_disable_parity(ch) do {				\
	(ch)->TxControl[2] &= ~PARITY_EN;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

#define rp_disable_rx_fifo(ch) do {				\
	(ch)->R[0x32] = 0x0a;					\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->R + 0x30));	\
} while (0)

/*
 * This takes the channel out of the receive status mode.  All subsequent reads
 * of receive data using sReadRxWord() will return two data bytes.
 */
#define rp_dis_rx_status_mode(ch) rp_writech2((ch), CHNOFF_CHANSTAT(ch), 0)

/*
 * This disables movement of Tx data from the Tx FIFO into the 1 byte Tx
 * buffer.  Therefore there could be up to a 2 byte latency between the time
 * rp_disable_transmit() is called and the transmit buffer and transmit shift
 * register going completely empty.
 */
#define rp_disable_transmit(cp) do {				\
	(cp)->TxControl[3] &= ~TX_ENABLE;			\
	rp_writech4(cp, _INDX_ADDR, lemtoh32((cp)->TxControl));	\
} while (0)

#define rp_disable_tx_soft_flowctl(ch) do {			\
	(ch)->R[0x06] = 0x8a;					\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->R + 0x04));	\
} while (0)

/* enable output flow control using CTS */
#define rp_enable_CTS_flowctl(ch) do {				\
	(ch)->TxControl[2] |= CTSFC_EN;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Function sSetParity() can be used in place of functions sEnParity(),
 * sDisParity(), rp_set_odd_parity(), and sSetEvenParity().
 *
 * Warnings:
 * Before enabling parity odd or even parity should be chosen using functions
 * rp_set_odd_parity() or sSetEvenParity().
 */
#define rp_enable_parity(ch) do {				\
	(ch)->TxControl[2] |= PARITY_EN;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

#define rp_enable_RTS_flowctl(ch) do {				\
	(ch)->TxControl[2] &= ~RTSTOG_EN;			\
	(ch)->TxControl[3] &= ~SET_RTS;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
	(ch)->RxControl[2] |= RTSFC_EN;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->RxControl));	\
} while (0)

#define rp_disable_RTS_flowctl(ch) do {				\
	(ch)->RxControl[2] &= ~RTSFC_EN;			\
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->RxControl));\
} while (0)

#define rp_enable_rx_fifo(ch) do {				\
	(ch)->R[0x32] = 0x08;					\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->R + 0x30));	\
} while (0)

/*
 * This function is used to start the receive processor.  When the channel is
 * in the reset state the receive processor is not running.  This is done to
 * prevent the receive processor from executing invalid microcode instructions
 * prior to the downloading of the microcode.
 *
 * Warnings:
 * This function must be called after valid microcode has been downloaded to
 * the AIOP, and it must not be called before the microcode has been
 * downloaded.
 */
#define rp_enable_rx_processor(ch) do {				\
	(ch)->RxControl[2] |= RXPROC_EN;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->RxControl));	\
} while (0)

/*
 * This places the channel in the receive status mode.  All subsequent reads of
 * receive data using sReadRxWord() will return a data byte in the low word and
 * a status byte in the high word.
 */
#define rp_enable_rx_status_mode(ch) \
	rp_writech2(ch, CHNOFF_CHANSTAT(ch), STATMODE)

#define rp_enable_transmit(ch) do {				\
	(ch)->TxControl[3] |= TX_ENABLE;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Purpose:  Get the AIOP interrupt status
 *
 * Returns the AIOP interrupt status.  Bits 0 through 7 represent channels 0
 * through 7 respectively.  If a bit is set that channel is interrupting.
 */
#define rp_aiop_intr_status(sc, AIOPNUM) \
	rp_readaiop1((sc), (AIOPNUM), _INT_CHAN)

/*
 * Get a channel's interrupt identification byte and returns the channel
 * interrupt ID.  Can be any combination of the following flags:
 *	RXF_TRIG:  Rx FIFO trigger level interrupt
 *	TXFIFO_MT: Tx FIFO empty interrupt
 *	SRC_INT:   Special receive condition interrupt
 *	DELTA_CD:  CD change interrupt
 *	DELTA_CTS: CTS change interrupt
 *	DELTA_DSR: DSR change interrupt
 */
#define rp_chan_intr_id(ch)					\
	(rp_readch1(ch, (ch)->ChanNum+_INT_ID0) &		\
	 (RXF_TRIG | TXFIFO_MT | SRC_INT | DELTA_CD | DELTA_CTS | DELTA_DSR))

/*
 * Returns the channel status.  Can be any combination of the following flags:
 *
 * LOW BYTE FLAGS			HIGH BYTE FLAGS
 *
 * CTS_ACT:  CTS input asserted		STATMODE:  status mode enable bit
 * DSR_ACT:  DSR input asserted		RXFOVERFL: receive FIFO overflow
 * D_ACT:    CD input asserted		RX2MATCH:  receive compare byte 2 match
 * TXFIFOMT: Tx FIFO is empty		RX1MATCH:  receive compare byte 1 match
 * TXSHRMT:  Tx shift reg. is empty	RXBREAK:   received BREAK
 * RDA:	     Rx data available		RXFRAME:   received framing error
 *					RXPARITY:  received parity error
 *
 * Warnings:
 * This function will clear the high byte flags in the Channel Status Register.
 */
#define rp_chan_status(ch) rp_readch2((ch), CHNOFF_CHANSTAT(ch))

/*
 * Get the low byte only of the channel status
 *
 * Returns the channel status low byte.  Can be any combination of the
 * following flags:
 *
 * 	CTS_ACT:  CTS input asserted
 * 	DSR_ACT:  DSR input asserted
 * 	CD_ACT:   CD input asserted
 * 	TXFIFOMT: Tx FIFO is empty
 * 	TXSHRMT:  Tx shift register is empty
 * 	RDA:      Rx data available
 */
#define rp_chan_status_lo(ch) rp_readch1((ch), CHNOFF_CHANSTAT(ch))

/*
 * Purpose:  Get the number of data bytes in the Rx FIFO
 * Returns the number of data bytes in the Rx FIFO.
 * Comments: Byte read of count register is required to obtain Rx count.
*/
#define rp_get_rx_cnt(ChP) rp_readch2((ChP), CHNOFF_TXRXCOUNT(ChP))

/*
 * Returns the number of data bytes in the Tx FIFO.
 * Comments: Byte read of count register is required to obtain Tx count.
 */
#define rp_get_tx_cnt(ch) rp_readch1((ch), CHNOFF_TXRXCOUNT(ch))

/* Return the offset of a channel's TxRx Data register */
#define rp_txrx_data_io(ch) CHNOFF_TXRXDATA(ch)

/*
 * Initialize a channel structure to its default state.
 *
 * This function must be called once for every channel structure that exists
 * before any other SSCI calls can be made.
 */
#define rp_init_chan_defaults(ch) do {	\
	(ch)->sc = NULL;		\
	(ch)->AiopNum = -1;		\
	(ch)->ChanID = -1;		\
	(ch)->ChanNum = -1;		\
} while (0)

#define rp_reset_aiop_by_num(sc, AIOPNUM) do {			\
	rp_writeaiop1((sc), (AIOPNUM), _CMD_REG, RESET_ALL);	\
	rp_writeaiop1((sc), (AIOPNUM), _CMD_REG, 0x0);		\
} while (0)

/* Set baud rate */
#define rp_set_baud(ch, DIVISOR) do {				\
	(ch)->BaudDiv[2] = (uint8_t)(DIVISOR);			\
	(ch)->BaudDiv[3] = (uint8_t)((DIVISOR) >> 8);		\
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->BaudDiv));	\
} while (0)

/* Set data bits to 7 */
#define rp_set_data7(ch) do {					\
	(ch)->TxControl[2] &= ~DATA8BIT;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/* Set data bits to 8 */
#define rp_set_data8(ch) do {					\
	(ch)->TxControl[2] |= DATA8BIT;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/* Set the DTR output */
#define rp_set_DTR(ch) do {					\
	(ch)->TxControl[3] |= SET_DTR;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Function sSetParity() can be used in place of functions sEnParity(),
 * sDisParity(), rp_set_odd_parity(), and sSetEvenParity().
 *
 * Warnings:
 * This function has no effect unless parity is enabled with function
 * sEnParity().
 */
#define rp_set_even_parity(ch) do {				\
	(ch)->TxControl[2] |= EVEN_PAR;				\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Function sSetParity() can be used in place of functions sEnParity(),
 * sDisParity(), rp_set_odd_parity(), and sSetEvenParity().
 *
 * Warnings:
 * This function has no effect unless parity is enabled with function
 * sEnParity().
 */
#define rp_set_odd_parity(ch) do {				\
	(ch)->TxControl[2] &= ~EVEN_PAR;			\
	rp_writech4(ch, _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/* Set the RTS output */
#define rp_set_RTS(ch) do {						\
	(ch)->TxControl[3] |= SET_RTS;					\
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Set the Rx FIFO trigger level
 *
 * Number of characters in Rx FIFO at which the interrupt will be generated.
 * Can be any of the following flags:
 *
 *	TRIG_NO:  no trigger
 *	TRIG_1:   1 character in FIFO
 *	TRIG_1_2: FIFO 1/2 full
 *	TRIG_7_8: FIFO 7/8 full
 *
 * An interrupt will be generated when the trigger level is reached only if
 * function sEnInterrupt() has been called with flag RXINT_EN set.  The
 * RXF_TRIG flag in the Interrupt Idenfification register will be set whenever
 * the trigger level is reached regardless of the setting of RXINT_EN.
 */
#define rp_rx_trigger(ch, level) do {					\
	(ch)->RxControl[2] &= ~TRIG_MASK;				\
	(ch)->RxControl[2] |= (level);					\
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->RxControl));	\
} while (0)

/* Set stop bits to 1 */
#define rp_set_stop1(ch) do {						\
	(ch)->TxControl[2] &= ~STOP2;					\
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/* Set stop bits to 2 */
#define rp_set_stop2(ch) do {						\
	(ch)->TxControl[2] |= STOP2;					\
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->TxControl));	\
} while (0)

/*
 * Start a channel's receive processor
 *
 * This function is used to start a Rx processor after it was stopped with
 * rp_stop_rx_processor().  It will restart both the Rx processor and software
 * input flow control.
 */
#define rp_start_rx_processor(ch) \
	rp_writech4((ch), _INDX_ADDR, lemtoh32((ch)->R))

/*
 * Write a transmit data byte to a channel.
 *
 * Warnings:
 * This function writes the data byte without checking to see if sMaxTxSize is
 * exceeded in the Tx FIFO.
 */
#define rp_write_tx_byte(ch, io, data) rp_writech1((ch), (io), (data))

int rp_read_aiopid(struct rp_softc *, int);
int rp_read_aiop_numchan(struct rp_softc *sc, int);
int rp_init_chan(struct rp_softc *, struct rp_chan *, int, int);
void rp_stop_rx_processor(struct rp_chan *);
void rp_flush_rx_fifo(struct rp_chan *);
void rp_flush_tx_fifo(struct rp_chan *);
int rp_write_tx_prio_byte(struct rp_chan *, uint8_t);
int rp_intr(void *);
void rp_enable_interrupts(struct rp_chan *, uint16_t);
void rp_disable_interrupts(struct rp_chan *, uint16_t);
int rp_attach(struct rp_softc* sc, int, int);
void rp_releaseresource(struct rp_softc *sc);
static __inline void
rp_lock(struct rp_softc *sc)
{
	if (sc->hwmtx_init != 0)
		mtx_enter(&sc->hwmtx);
}
static __inline void
rp_unlock(struct rp_softc *sc)
{
	if (sc->hwmtx_init != 0)
		mtx_leave(&sc->hwmtx);
}

/* XXX: useless block */
//extern uint8_t R[RDATASIZE];
//XXX: extern struct rp_softc sController[CTL_SIZE];
//extern uint8_t sIRQMap[16];

extern uint8_t rp_sBitMapClrTbl[8];
extern uint8_t rp_sBitMapSetTbl[8];

#define RP_UNIT(x)	dv_unit(x)
#define MAX_RP_PORTS	128

/*
 * Port number on card encoded in low 5 bits
 * card number in next 2 bits (only space for 4 cards)
 * high bit reserved for dialout flag
 */
#define RP_PORT(x) (minor(x) & 0xf)
#define RP_CARD(x) ((minor(x) >> 5) & 3)

#define RPF_STOP 0x01

struct rp_port {
	struct tty	*rp_tty;	/* cross reference */
	struct timeout	 rp_timer;

	unsigned char	 state;		/* state of dtr */

	int		 rp_swflags;
	int		 rp_cua;
	int		 rp_port;
	int		 rp_flags;
	int		 rp_unit:2;
	int		 rp_aiop:2;
	int		 rp_chan:3;
	int		 rp_intmask;
	int		 rp_imask;	/* input mask */
	int		 rp_fifo_lw;
	int		 rp_restart;
	int		 rp_overflows;
	int		 rp_rts_iflow:1;
	int		 rp_disable_writes:1;
	int		 rp_cts:1;
	int		 rp_waiting:1;
	int		 rp_xmit_stopped:1;
	struct rp_softc	*rp_sc;
	struct rp_chan	 rp_channel;
	unsigned char	 TxBuf[TXFIFO_SIZE];
	unsigned char	 RxBuf[RXFIFO_SIZE];
};
