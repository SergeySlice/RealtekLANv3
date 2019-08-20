/*
 * Copyright (c) 1997, 1998
 *	Bill Paul <wpaul@ctr.columbia.edu>.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Bill Paul.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/re/if_rereg.h,v 1.14.2.1 2001/07/19 18:33:07 wpaul Exp $
 */

/*#define VERSION(_MainVer,_MinorVer)	((_MainVer)*10+(_MinorVer))*/
/*#define OS_VER	VERSION(5,1)*/
#if __FreeBSD_version < 500000
#define VERSION(_MainVer,_MinorVer)	((_MainVer)*100000+(_MinorVer)*10000)
#else
#define VERSION(_MainVer,_MinorVer)	((_MainVer)*100000+(_MinorVer)*1000)
#endif
#define OS_VER	__FreeBSD_version


/*
 * RealTek RTL8110S/SB/SC register offsets
 */

#define	RE_TPPOLL	0x0038		/* transmit priority polling */

/*
 * RealTek RTL8110S/SB/SC register contents
 */

/* Transmit Priority Polling --- 0x40 */
#define	RE_HPQ		0x80		/* high priority queue polling */
#define	RE_NPQ		0x40		/* normal priority queue polling */
#define	RE_FSWInt	0x01		/* Forced Software Interrupt */


/*
 * RealTek 8129/8139 register offsets
 */

#define	RE_IDR0		0x0000		/* ID register 0 (station addr) */
#define RE_IDR1		0x0001		/* Must use 32-bit accesses (?) */
#define RE_IDR2		0x0002
#define RE_IDR3		0x0003
#define RE_IDR4		0x0004
#define RE_IDR5		0x0005
					/* 0006-0007 reserved */
#define RE_MAR0		0x0008		/* Multicast hash table */
#define RE_MAR1		0x0009
#define RE_MAR2		0x000A
#define RE_MAR3		0x000B
#define RE_MAR4		0x000C
#define RE_MAR5		0x000D
#define RE_MAR6		0x000E
#define RE_MAR7		0x000F

#define RE_TXSTAT0	0x0010		/* status of TX descriptor 0 */
#define RE_TXSTAT1	0x0014		/* status of TX descriptor 1 */
#define RE_TXSTAT2	0x0018		/* status of TX descriptor 2 */
#define RE_TXSTAT3	0x001C		/* status of TX descriptor 3 */

#define RE_TXADDR0	0x0020		/* address of TX descriptor 0 */
#define RE_TXADDR1	0x0024		/* address of TX descriptor 1 */
#define RE_TXADDR2	0x0028		/* address of TX descriptor 2 */
#define RE_TXADDR3	0x002C		/* address of TX descriptor 3 */

#define RE_RXADDR	0x0030		/* RX ring start address */
#define RE_COMMAND	0x0037		/* command register */
#define RE_CURRXADDR	0x0038		/* current address of packet read */
#define RE_CURRXBUF	0x003A		/* current RX buffer address */
#define RE_IMR		0x003C		/* interrupt mask register */
#define RE_ISR		0x003E		/* interrupt status register */
#define RE_TXCFG	0x0040		/* transmit config */
#define RE_RXCFG	0x0044		/* receive config */
#define RE_TIMERCNT	0x0048		/* timer count register */
#define RE_MISSEDPKT	0x004C		/* missed packet counter */
#define RE_EECMD	0x0050		/* EEPROM command register */
#define RE_CFG0		0x0051		/* config register #0 */
#define RE_CFG1		0x0052		/* config register #1 */
#define RE_CFG2		0x0053		/* config register #2 */
#define RE_CFG3		0x0054		/* config register #3 */
#define RE_CFG4		0x0055		/* config register #4 */
#define RE_CFG5		0x0056		/* config register #5 */
					/* 0053-0057 reserved */
#define RE_MEDIASTAT	0x0058		/* media status register (8139) */
					/* 0059-005A reserved */
#define RE_MII		0x005A		/* 8129 chip only */
#define RE_HALTCLK	0x005B
#define RE_MULTIINTR	0x005C		/* multiple interrupt */
#define RE_PCIREV	0x005E		/* PCI revision value */
					/* 005F reserved */
#define RE_PHYAR	0x0060		/* PHY register access */
#define RE_CSIDR	0x0064
#define RE_CSIAR	0x0068
#define RE_PHY_STATUS	0x006C		/* PHY status */
#define RE_ERIDR	0x70
#define RE_ERIAR	0x74
#define RE_EPHYAR	0x0080
#define RE_DBG_reg	0x00D1
#define RE_RxMaxSize	0x00DA
#define RE_CPlusCmd	0x00E0
#define	RE_MTPS		0x00EC

/* ERI access */
#define	ERIAR_Flag   0x80000000
#define	ERIAR_Write   0x80000000
#define	ERIAR_Read   0x00000000
#define	ERIAR_Addr_Align  4 /* ERI access register address must be 4 byte alignment */
#define	ERIAR_ExGMAC  0
#define	ERIAR_MSIX  1
#define	ERIAR_ASF  2
#define	ERIAR_Type_shift  16
#define	ERIAR_ByteEn  0x0f
#define	ERIAR_ByteEn_shift  12





/* Direct PHY access registers only available on 8139 */
#define RE_BMCR		0x0062		/* PHY basic mode control */
#define RE_BMSR		0x0064		/* PHY basic mode status */
#define RE_ANAR		0x0066		/* PHY autoneg advert */
#define RE_LPAR		0x0068		/* PHY link partner ability */
#define RE_ANER		0x006A		/* PHY autoneg expansion */

#define RE_DISCCNT	0x006C		/* disconnect counter */
#define RE_FALSECAR	0x006E		/* false carrier counter */
#define RE_NWAYTST	0x0070		/* NWAY test register */
#define RE_RX_ER	0x0072		/* RX_ER counter */
#define RE_CSCFG	0x0074		/* CS configuration register */
#define RE_LDPS		0x0082		/* Link Down Power Saving */
#define RE_CPCR		0x00E0
#define	RE_IM		0x00E2


/*
 * TX config register bits
 */
#define RE_TXCFG_CLRABRT	0x00000001	/* retransmit aborted pkt */
#define RE_TXCFG_MAXDMA		0x00000700	/* max DMA burst size */
#define RE_TXCFG_CRCAPPEND	0x00010000	/* CRC append (0 = yes) */
#define RE_TXCFG_LOOPBKTST	0x00060000	/* loopback test */
#define RE_TXCFG_IFG		0x03000000	/* interframe gap */

#define RE_TXDMA_16BYTES	0x00000000
#define RE_TXDMA_32BYTES	0x00000100
#define RE_TXDMA_64BYTES	0x00000200
#define RE_TXDMA_128BYTES	0x00000300
#define RE_TXDMA_256BYTES	0x00000400
#define RE_TXDMA_512BYTES	0x00000500
#define RE_TXDMA_1024BYTES	0x00000600
#define RE_TXDMA_2048BYTES	0x00000700

/*
 * Transmit descriptor status register bits.
 */
#define RE_TXSTAT_LENMASK	0x00001FFF
#define RE_TXSTAT_OWN		0x00002000
#define RE_TXSTAT_TX_UNDERRUN	0x00004000
#define RE_TXSTAT_TX_OK		0x00008000
#define RE_TXSTAT_COLLCNT	0x0F000000
#define RE_TXSTAT_CARR_HBEAT	0x10000000
#define RE_TXSTAT_OUTOFWIN	0x20000000
#define RE_TXSTAT_TXABRT	0x40000000
#define RE_TXSTAT_CARRLOSS	0x80000000

/*
 * Interrupt status register bits.
 */
#define RE_ISR_RX_OK		0x0001
#define RE_ISR_RX_ERR		0x0002
#define RE_ISR_TX_OK		0x0004
#define RE_ISR_TX_ERR		0x0008
#define RE_ISR_RX_OVERRUN	0x0010
#define RE_ISR_PKT_UNDERRUN	0x0020
#define RE_ISR_LINKCHG		0x0020
#define RE_ISR_FIFO_OFLOW	0x0040	/* 8139 only */
#define RE_ISR_TDU		0x0080
#define RE_ISR_PCS_TIMEOUT	0x4000	/* 8129 only */
#define RE_ISR_SYSTEM_ERR	0x8000

/*
#define RE_INTRS	\
	(RE_ISR_TX_OK|RE_ISR_RX_OK|RE_ISR_RX_ERR|RE_ISR_TX_ERR|		\
	RE_ISR_RX_OVERRUN|RE_ISR_PKT_UNDERRUN|RE_ISR_FIFO_OFLOW|	\
	RE_ISR_PCS_TIMEOUT|RE_ISR_SYSTEM_ERR)
*/

#define RE_INTRS	\
	(RE_ISR_TX_OK|RE_ISR_RX_OK|RE_ISR_RX_ERR|RE_ISR_TX_ERR|		\
	RE_ISR_RX_OVERRUN|RE_ISR_PKT_UNDERRUN|	\
	RE_ISR_PCS_TIMEOUT|RE_ISR_SYSTEM_ERR)

/*
 * Media status register. (8139 only)
 */
#define RE_MEDIASTAT_RXPAUSE	0x01
#define RE_MEDIASTAT_TXPAUSE	0x02
#define RE_MEDIASTAT_LINK	0x04
#define RE_MEDIASTAT_SPEED10	0x08
#define RE_MEDIASTAT_RXFLOWCTL	0x40	/* duplex mode */
#define RE_MEDIASTAT_TXFLOWCTL	0x80	/* duplex mode */

/*
 * Receive config register.
 */
#define RE_RXCFG_RX_ALLPHYS	0x00000001	/* accept all nodes */
#define RE_RXCFG_RX_INDIV	0x00000002	/* match filter */
#define RE_RXCFG_RX_MULTI	0x00000004	/* accept all multicast */
#define RE_RXCFG_RX_BROAD	0x00000008	/* accept all broadcast */
#define RE_RXCFG_RX_RUNT	0x00000010
#define RE_RXCFG_RX_ERRPKT	0x00000020
#define RE_RXCFG_RX_9356SEL	0x00000040
#define RE_RXCFG_WRAP		0x00000080
#define RE_RXCFG_MAXDMA		0x00000700
#define RE_RXCFG_BUFSZ		0x00001800

#define RE_RXDMA_16BYTES	0x00000000
#define RE_RXDMA_32BYTES	0x00000100
#define RE_RXDMA_64BYTES	0x00000200
#define RE_RXDMA_128BYTES	0x00000300
#define RE_RXDMA_256BYTES	0x00000400
#define RE_RXDMA_512BYTES	0x00000500
#define RE_RXDMA_1024BYTES	0x00000600
#define RE_RXDMA_UNLIMITED	0x00000700

#define RE_RXBUF_8		0x00000000
#define RE_RXBUF_16		0x00000800
#define RE_RXBUF_32		0x00001000
#define RE_RXBUF_64		0x00001800

#define RE_RXRESVERED		0x0000E000

/*
 * Bits in RX status header (included with RX'ed packet
 * in ring buffer).
 */
#define RE_RXSTAT_RXOK		0x00000001
#define RE_RXSTAT_ALIGNERR	0x00000002
#define RE_RXSTAT_CRCERR	0x00000004
#define RE_RXSTAT_GIANT		0x00000008
#define RE_RXSTAT_RUNT		0x00000010
#define RE_RXSTAT_BADSYM	0x00000020
#define RE_RXSTAT_BROAD		0x00002000
#define RE_RXSTAT_INDIV		0x00004000
#define RE_RXSTAT_MULTI		0x00008000
#define RE_RXSTAT_LENMASK	0xFFFF0000

#define RE_RXSTAT_UNFINISHED	0xFFF0		/* DMA still in progress */
/*
 * Command register.
 */
#define RE_CMD_EMPTY_RXBUF	0x0001
#define RE_CMD_TX_ENB		0x0004
#define RE_CMD_RX_ENB		0x0008
#define RE_CMD_RESET		0x0010

/*
 * EEPROM control register
 */
#define RE_EE_DATAOUT		0x01	/* Data out */
#define RE_EE_DATAIN		0x02	/* Data in */
#define RE_EE_CLK		0x04	/* clock */
#define RE_EE_SEL		0x08	/* chip select */
#define RE_EE_MODE		(0x40|0x80)

#define RE_EEMODE_OFF		0x00
#define RE_EEMODE_AUTOLOAD	0x40
#define RE_EEMODE_PROGRAM	0x80
#define RE_EEMODE_WRITECFG	(0x80|0x40)

/* 9346 EEPROM commands */
#define RE_EECMD_WRITE		0x140
#define RE_EECMD_READ		0x180
#define RE_EECMD_ERASE		0x1c0

#define RE_EE_ID			0x00
#define RE_EE_PCI_VID		0x01
#define RE_EE_PCI_DID		0x02
/* Location of station address inside EEPROM */
 #define RE_EE_EADDR		0x07

/*
 * MII register (8129 only)
 */
#define RE_MII_CLK		0x01
#define RE_MII_DATAIN		0x02
#define RE_MII_DATAOUT		0x04
#define RE_MII_DIR		0x80	/* 0 == input, 1 == output */

/*
 * Config 0 register
 */
#define RE_CFG0_ROM0		0x01
#define RE_CFG0_ROM1		0x02
#define RE_CFG0_ROM2		0x04
#define RE_CFG0_PL0		0x08
#define RE_CFG0_PL1		0x10
#define RE_CFG0_10MBPS		0x20	/* 10 Mbps internal mode */
#define RE_CFG0_PCS		0x40
#define RE_CFG0_SCR		0x80

/*
 * Config 1 register
 */
#define RE_CFG1_PWRDWN		0x01
#define RE_CFG1_SLEEP		0x02
#define RE_CFG1_IOMAP		0x04
#define RE_CFG1_MEMMAP		0x08
#define RE_CFG1_RSVD		0x10
#define RE_CFG1_DRVLOAD		0x20
#define RE_CFG1_LED0		0x40
#define RE_CFG1_FULLDUPLEX	0x40	/* 8129 only */
#define RE_CFG1_LED1		0x80

/*
 * The RealTek doesn't use a fragment-based descriptor mechanism.
 * Instead, there are only four register sets, each or which represents
 * one 'descriptor.' Basically, each TX descriptor is just a contiguous
 * packet buffer (32-bit aligned!) and we place the buffer addresses in
 * the registers so the chip knows where they are.
 *
 * We can sort of kludge together the same kind of buffer management
 * used in previous drivers, but we have to do buffer copies almost all
 * the time, so it doesn't really buy us much.
 *
 * For reception, there's just one large buffer where the chip stores
 * all received packets.
 */

#define RE_RX_BUF_SZ		RE_RXBUF_64
#define RE_RXBUFLEN		(1 << ((RE_RX_BUF_SZ >> 11) + 13))
#define RE_TX_LIST_CNT		4		/*  C mode Tx buffer number */
#define RE_TX_BUF_NUM		256		/* Tx buffer number */
#define RE_RX_BUF_NUM		256		/* Rx buffer number */
#define RE_BUF_SIZE		9216		/* Buffer size of descriptor buffer */
#define RE_MIN_FRAMELEN		60
#define RE_TXREV(x)		((x) << 11)
#define RE_RX_RESVERED		RE_RXRESVERED
#define RE_RX_MAXDMA		RE_RXDMA_UNLIMITED
#define RE_TX_MAXDMA		RE_TXDMA_2048BYTES
#define	RE_NTXSEGS		32

#define RE_RXCFG_CONFIG_1 (RE_RX_RESVERED | RE_RX_MAXDMA | RE_RX_BUF_SZ | 0x0E)
#define RE_RXCFG_CONFIG_2 (RE_RX_MAXDMA | RE_RX_BUF_SZ | 0x0E | 0x8000)

#define RE_TXCFG_CONFIG		0x03000780 //(RE_TXCFG_IFG|RE_TX_MAXDMA)

#define RE_DESC_ALIGN	256		/* descriptor alignment */

#define RE_ETHER_ALIGN	2

#define Jumbo_Frame_2k	(2 * 1024)
#define Jumbo_Frame_3k	(3 * 1024)
#define Jumbo_Frame_4k	(4 * 1024)
#define Jumbo_Frame_5k	(5 * 1024)
#define Jumbo_Frame_6k	(6 * 1024)
#define Jumbo_Frame_7k	(7 * 1024)
#define Jumbo_Frame_8k	(8 * 1024)
#define Jumbo_Frame_9k	(9 * 1024)
struct re_chain_data {
	u_int16_t		cur_rx;
	caddr_t			re_rx_buf;
	caddr_t			re_rx_buf_ptr;

	struct mbuf		*re_tx_chain[RE_TX_LIST_CNT];
	u_int8_t		last_tx;	/* Previous Tx OK */
	u_int8_t		cur_tx;		/* Next to TX */
};

union RxDesc
{
	u_int32_t	ul[4];
	struct
	{
		u_int32_t Frame_Length:14;
		u_int32_t TCPF:1;
		u_int32_t UDPF:1;
		u_int32_t IPF:1;
		u_int32_t TCPT:1;
		u_int32_t UDPT:1;
		u_int32_t CRC:1;
		u_int32_t RUNT:1;
		u_int32_t RES:1;
		u_int32_t RWT:1;
		u_int32_t RESV:2;	
		u_int32_t BAR:1;
		u_int32_t PAM:1;
		u_int32_t MAR:1;
		u_int32_t LS:1;
		u_int32_t FS:1;
		u_int32_t EOR:1;
		u_int32_t OWN:1;

		u_int32_t VLAN_TAG:16;
		u_int32_t TAVA:1;
		u_int32_t RESV1:15;

		u_int32_t RxBuffL;
		u_int32_t RxBuffH;
	}so0;	/* symbol owner=0 */
};

union TxDesc
{
	u_int32_t	ul[4];
	struct
	{
		u_int32_t Frame_Length:16;
		u_int32_t TCPCS:1;
		u_int32_t UDPCS:1;
		u_int32_t IPCS:1;
		u_int32_t SCRC:1;
		u_int32_t RESV:6;
		u_int32_t TDMA:1;
		u_int32_t LGSEN:1;
		u_int32_t LS:1;
		u_int32_t FS:1;
		u_int32_t EOR:1;
		u_int32_t OWN:1;

		u_int32_t VLAN_TAG:16;
		u_int32_t TAGC0:1;
		u_int32_t TAGC1:1;
		u_int32_t RESV1:14;

		u_int32_t TxBuffL;
		u_int32_t TxBuffH;
	}so1;	/* symbol owner=1 */
};

struct re_descriptor
{
	u_int8_t		rx_cur_index;
	u_int8_t		rx_last_index;
	union RxDesc 		*rx_desc;	/* 8 bits alignment */
	struct mbuf		*rx_buf[RE_RX_BUF_NUM];

	u_int8_t		tx_cur_index;
	u_int8_t		tx_last_index;
	union TxDesc		*tx_desc;	/* 8 bits alignment */
	struct mbuf		*tx_buf[RE_TX_BUF_NUM];
	bus_dma_tag_t		rx_desc_tag;
	bus_dmamap_t		rx_desc_dmamap;
	bus_dma_tag_t		re_rx_mtag;	/* mbuf RX mapping tag */
	bus_dmamap_t		re_rx_dmamap[RE_RX_BUF_NUM];

	bus_dma_tag_t		tx_desc_tag;
	bus_dmamap_t		tx_desc_dmamap;
	bus_dma_tag_t		re_tx_mtag;	/* mbuf TX mapping tag */
	bus_dmamap_t		re_tx_dmamap[RE_TX_BUF_NUM];
};

#define RE_INC(x)		(x = (x + 1) % RE_TX_LIST_CNT)
#define RE_CUR_TXADDR(x)	((x->re_cdata.cur_tx * 4) + RE_TXADDR0)
#define RE_CUR_TXSTAT(x)	((x->re_cdata.cur_tx * 4) + RE_TXSTAT0)
#define RE_CUR_TXMBUF(x)	(x->re_cdata.re_tx_chain[x->re_cdata.cur_tx])
#define RE_LAST_TXADDR(x)	((x->re_cdata.last_tx * 4) + RE_TXADDR0)
#define RE_LAST_TXSTAT(x)	((x->re_cdata.last_tx * 4) + RE_TXSTAT0)
#define RE_LAST_TXMBUF(x)	(x->re_cdata.re_tx_chain[x->re_cdata.last_tx])

struct re_type {
	u_int16_t		re_vid;
	u_int16_t		re_did;
	char			*re_name;
};

struct re_mii_frame {
	u_int8_t		mii_stdelim;
	u_int8_t		mii_opcode;
	u_int8_t		mii_phyaddr;
	u_int8_t		mii_regaddr;
	u_int8_t		mii_turnaround;
	u_int16_t		mii_data;
};

/*
 * MII constants
 */
#define RE_MII_STARTDELIM	0x01
#define RE_MII_READOP		0x02
#define RE_MII_WRITEOP		0x01
#define RE_MII_TURNAROUND	0x02
#define RL_TDESC_VLANCTL_TAG 0x00020000
#define RL_RDESC_VLANCTL_TAG 0x00010000
#define RL_RDESC_VLANCTL_DATA	0x0000FFFF	
#define RL_CPLUSCMD_VLANSTRIP 0x0040
#define	RL_FLAG_DESCV2		0x0040

#define RL_ProtoIP  	((1<<17)|(1<<18))
//#define RL_ProtoIP  	((1<<16)|(1<<17))
#define RL_TCPT 		(1<<17)
#define RL_UDPT 		(1<<18)
#define RL_IPF		(1<<16)
#define RL_UDPF		(1<<15)
#define RL_TCPF		(1<<14)
#define RL_V4F         	(1<<30)

#define RL_IPV4CS      (1<<29)
#define RL_TCPCS		(1<<30)
#define RL_UDPCS	(1<<31)
#define RL_IPV4CS1     (1<<18) 
#define RL_TCPCS1	(1<<16)
#define RL_UDPCS1	(1<<17)

#define RL_RxChkSum (1<<5)

enum
{
	RE_8129 = 0x01,
	RE_8139,
	MACFG_3,
	MACFG_4,
	MACFG_5,
	MACFG_6,

	MACFG_11,
	MACFG_12,
	MACFG_13,
	MACFG_14,
	MACFG_15,
	MACFG_16,
	MACFG_17,
	MACFG_18,
	MACFG_19,

	MACFG_21,
	MACFG_22,
	MACFG_23,
	MACFG_24,
	MACFG_25,
	MACFG_26,
	MACFG_27,
	MACFG_28,

	MACFG_31,
	MACFG_32,
	MACFG_33,
	MACFG_34,
	MACFG_35,
	MACFG_36,
	MACFG_37,
	MACFG_38,
	MACFG_39,

	MACFG_41,
	MACFG_42,
	MACFG_43,

	MACFG_50,
	MACFG_51,
	MACFG_52,
	MACFG_53,

	MACFG_FF = 0xFF
};

//#define MAC_STYLE_1	1	/* RTL8110S/SB/SC, RTL8111B and RTL8101E */
//#define MAC_STYLE_2	2	/* RTL8111C/CP/D and RTL8102E */

struct re_softc
{
#if OS_VER<VERSION(6,0)
	struct arpcom		arpcom;			/* interface info */
#else
	struct ifnet		*re_ifp;
#endif

	bus_space_handle_t	re_bhandle;		/* bus space handle */
	bus_space_tag_t		re_btag;			/* bus space tag */
	struct resource		*re_res;
	struct resource		*re_irq;
	void			*re_intrhand;
	struct ifmedia		media;			/* used to instead of MII */

	/* Variable for 8169 family */
	u_int8_t		re_8169_MacVersion;
	u_int8_t		re_8169_PhyVersion;

	u_int8_t		rx_fifo_overflow;
	u_int8_t		driver_detach;

	u_int8_t		re_unit;			/* interface number */
	u_int8_t		re_type;
	u_int8_t		re_stats_no_timeout;
	u_int8_t		re_revid;
	u_int16_t		re_device_id;

	struct re_chain_data	re_cdata;		/* Tx buffer chain, Used only in ~C+ mode */
	struct re_descriptor	re_desc;			/* Descriptor, Used only in C+ mode */

	struct callout_handle	re_stat_ch;
	struct mtx		mtx;
	bus_dma_tag_t		re_parent_tag;
	device_t		dev;
	int			 max_jumbo_frame_size;
	int 			re_rx_buf_sz;
	int			re_if_flags;	
	int 			re_tx_cstag;
	int			re_rx_cstag;
#if OS_VER>=VERSION(7,0)
	struct task		re_inttask;
#endif
};

enum bits {
	BIT_0 = (1 << 0),
	BIT_1 = (1 << 1),
	BIT_2 = (1 << 2),
	BIT_3 = (1 << 3),
	BIT_4 = (1 << 4),
	BIT_5 = (1 << 5),
	BIT_6 = (1 << 6),
	BIT_7 = (1 << 7),
	BIT_8 = (1 << 8),
	BIT_9 = (1 << 9),
	BIT_10 = (1 << 10),
	BIT_11 = (1 << 11),
	BIT_12 = (1 << 12),
	BIT_13 = (1 << 13),
	BIT_14 = (1 << 14),
	BIT_15 = (1 << 15),
	BIT_16 = (1 << 16),
	BIT_17 = (1 << 17),
	BIT_18 = (1 << 18),
	BIT_19 = (1 << 19),
	BIT_20 = (1 << 20),
	BIT_21 = (1 << 21),
	BIT_22 = (1 << 22),
	BIT_23 = (1 << 23),
	BIT_24 = (1 << 24),
	BIT_25 = (1 << 25),
	BIT_26 = (1 << 26),
	BIT_27 = (1 << 27),
	BIT_28 = (1 << 28),
	BIT_29 = (1 << 29),
	BIT_30 = (1 << 30),
	BIT_31 = (1 << 31)
};

#define RE_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define RE_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define RE_LOCK_INIT(_sc,_name)	mtx_init(&(_sc)->mtx,_name,MTX_NETWORK_LOCK,MTX_DEF)
#define RE_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->mtx)
#define RE_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->mtx,MA_OWNED)

/*
 * register space access macros
 */
#if OS_VER>VERSION(5,9)
#define CSR_WRITE_STREAM_4(sc, reg, val)	\
	bus_space_write_stream_4(sc->re_btag, sc->re_bhandle, reg, val)
#endif
#define CSR_WRITE_4(sc, reg, val)	\
	bus_space_write_4(sc->re_btag, sc->re_bhandle, reg, val)
#define CSR_WRITE_2(sc, reg, val)	\
	bus_space_write_2(sc->re_btag, sc->re_bhandle, reg, val)
#define CSR_WRITE_1(sc, reg, val)	\
	bus_space_write_1(sc->re_btag, sc->re_bhandle, reg, val)

#define CSR_READ_4(sc, reg)		\
	bus_space_read_4(sc->re_btag, sc->re_bhandle, reg)
#define CSR_READ_2(sc, reg)		\
	bus_space_read_2(sc->re_btag, sc->re_bhandle, reg)
#define CSR_READ_1(sc, reg)		\
	bus_space_read_1(sc->re_btag, sc->re_bhandle, reg)

#define RE_TIMEOUT		1000

/*
 * General constants that are fun to know.
 *
 * RealTek PCI vendor ID
 */
#define	RT_VENDORID				0x10EC

/*
 * RealTek chip device IDs.
 */
#define RT_DEVICEID_8129			0x8129
#define RT_DEVICEID_8139			0x8139
#define RT_DEVICEID_8169			0x8169		/* For RTL8169 */
#define RT_DEVICEID_8169SC			0x8167		/* For RTL8169SC */
#define RT_DEVICEID_8168			0x8168		/* For RTL8168B */
#define RT_DEVICEID_8136			0x8136		/* For RTL8101E */

/*
 * Accton PCI vendor ID
 */
#define ACCTON_VENDORID				0x1113

/*
 * Accton MPX 5030/5038 device ID.
 */
#define ACCTON_DEVICEID_5030			0x1211

/*
 * Delta Electronics Vendor ID.
 */
#define DELTA_VENDORID				0x1500

/*
 * Delta device IDs.
 */
#define DELTA_DEVICEID_8139			0x1360

/*
 * Addtron vendor ID.
 */
#define ADDTRON_VENDORID			0x4033

/*
 * Addtron device IDs.
 */
#define ADDTRON_DEVICEID_8139			0x1360

/*
 * D-Link vendor ID.
 */
#define DLINK_VENDORID				0x1186

/*
 * D-Link DFE-530TX+ device ID
 */
#define DLINK_DEVICEID_530TXPLUS		0x1300

/*
 * PCI low memory base and low I/O base register, and
 * other PCI registers.
 */

#define RE_PCI_VENDOR_ID	0x00
#define RE_PCI_DEVICE_ID	0x02
#define RE_PCI_COMMAND		0x04
#define RE_PCI_STATUS		0x06
#define RE_PCI_REVISION_ID	0x08	/* 8 bits */
#define RE_PCI_CLASSCODE	0x09
#define RE_PCI_LATENCY_TIMER	0x0D
#define RE_PCI_HEADER_TYPE	0x0E
#define RE_PCI_LOIO		0x10
#define RE_PCI_LOMEM		0x14
#define RE_PCI_BIOSROM		0x30
#define RE_PCI_INTLINE		0x3C
#define RE_PCI_INTPIN		0x3D
#define RE_PCI_MINGNT		0x3E
#define RE_PCI_MINLAT		0x0F
#define RE_PCI_RESETOPT		0x48
#define RE_PCI_EEPROM_DATA	0x4C

#define RE_PCI_CAPID		0x50 /* 8 bits */
#define RE_PCI_NEXTPTR		0x51 /* 8 bits */
#define RE_PCI_PWRMGMTCAP	0x52 /* 16 bits */
#define RE_PCI_PWRMGMTCTRL	0x54 /* 16 bits */

#define RE_PSTATE_MASK		0x0003
#define RE_PSTATE_D0		0x0000
#define RE_PSTATE_D1		0x0002
#define RE_PSTATE_D2		0x0002
#define RE_PSTATE_D3		0x0003
#define RE_PME_EN		0x0010
#define RE_PME_STATUS		0x8000

#ifdef __alpha__
#undef vtophys
#define vtophys(va)     alpha_XXX_dmamap((vm_offset_t)va)
#endif

#ifndef TRUE
#define TRUE		1
#endif
#ifndef FALSE
#define FALSE		0
#endif

#define PHYAR_Flag		0x80000000
#define RE_CPlusMode		0x20		/* In Revision ID */
#define RE_MINI_DESC_SIZE	4

/* interrupt service routine loop time*/
/* the minimum value is 1 */
#define	INTR_MAX_LOOP	1

/*#define RE_DBG*/

#ifdef RE_DBG
	#define DBGPRINT(_unit, _msg)			printf ("re%d: %s\n", _unit,_msg)
	#define DBGPRINT1(_unit, _msg, _para1)	\
		{									\
			char buf[100];					\
			sprintf(buf,_msg,_para1);		\
			printf ("re%d: %s\n", _unit,buf);	\
		}
#else
	#define DBGPRINT(_unit, _msg)
	#define DBGPRINT1(_unit, _msg, _para1)
#endif

#if OS_VER<VERSION(4,9)
	#define IFM_1000_T		IFM_1000_TX
#elif OS_VER<VERSION(6,0)
	#define RE_GET_IFNET(SC)	&SC->arpcom.ac_if
	#define if_drv_flags		if_flags
	#define IFF_DRV_RUNNING		IFF_RUNNING
	#define IFF_DRV_OACTIVE		IFF_OACTIVE
#else
	#define RE_GET_IFNET(SC)	SC->re_ifp
#endif

