/*
 *  R1000Regs.h - Register and constant definitions for RealTek Ethernet chips
 *  RealtekR1000SL
 *
 *  Copyright 2009 Chuck Fry. All rights reserved.
 *
 * This software incorporates code from Realtek's open source Linux drivers
 * and the open source Mac OS X project RealtekR1000 by Dmitri Arekhta,
 * as modified by PSYSTAR Corporation.
 * 
 * Copyright(c) 2009 Realtek Semiconductor Corp. All rights reserved.
 * copyright PSYSTAR Corporation, 2008
 * 2006 (c) Dmitri Arekhta (DaemonES@gmail.com)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _R1000REGS_H_
#define _R1000REGS_H_

#include <net/ethernet.h>

#define	R1000_HW_FLOW_CONTROL_SUPPORT
//#define R1000_JUMBO_FRAME_SUPPORT

/* media options */
#define MAX_UNITS 8

// IEEE 802.3 Ethernet magic constants.
// From linux/if_ether.h
// FIXME - use Mac OS equivalents in net/ethernet.h
//#define ETH_DATA_LEN		1500	// Mac OS X: ETHERMTU
//#define ETH_HDR_LEN		14		// Mac OS X: ETHER_HDR_LEN
//#define FCS_LEN			4		// Mac OS X: ETHER_CRC_LEN
//#define MAC_ADDR_LEN		6		// Mac OS X: ETHER_ADDR_LEN
//#define MAC_PROTOCOL_LEN	2		// Mac OS X: ETHER_TYPE_LEN

// RX FIFO thresholds
// # of bytes to receive before PCI data transfer begins
#define RX_FIFO_THRESH_64	2
#define RX_FIFO_THRESH_128	3
#define RX_FIFO_THRESH_256	4
#define RX_FIFO_THRESH_512	5
#define RX_FIFO_THRESH_1024	6
#define RX_FIFO_THRESH_NONE 7       /* Wait for whole packet to be received */

#define RX_DMA_BURST        7       /* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST        7       /* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST_unlimited	7
#define TX_DMA_BURST_1024	6
#define TX_DMA_BURST_512	5
#define TX_DMA_BURST_256	4
#define TX_DMA_BURST_128	3
#define TX_DMA_BURST_64		2
#define TX_DMA_BURST_32		1
#define TX_DMA_BURST_16		0
#define RxPacketMaxSize	0x3FE8	/* 16K - 1 - ETH_HLEN - VLAN - CRC... */
#define Jumbo_Frame_2k	(2 * 1024)
#define Jumbo_Frame_3k	(3 * 1024)
#define Jumbo_Frame_4k	(4 * 1024)
#define Jumbo_Frame_5k	(5 * 1024)
#define Jumbo_Frame_6k	(6 * 1024)
#define Jumbo_Frame_7k	(7 * 1024)
#define Jumbo_Frame_8k	(8 * 1024)
#define Jumbo_Frame_9k	(9 * 1024)
#define InterFrameGap   0x03    /* 3 means InterFrameGap = the shortest one */
#define RxEarly_off_V1 (0x07 << 11)
#define RxEarly_off_V2 (1 << 11)

#define R8168_REGS_SIZE     256
#define R8168_NAPI_WEIGHT   64

#define RX_BUF_SIZE	0x05F3	/* 0x05F3 = 1522bye + 1 */
#define R8168_TX_RING_BYTES	(NUM_TX_DESC * sizeof(struct TxDesc))
#define R8168_RX_RING_BYTES	(NUM_RX_DESC * sizeof(struct RxDesc))

#define RTL8168_TX_TIMEOUT	(6 * HZ)
#define RTL8168_LINK_TIMEOUT	(1 * HZ)
#define RTL8168_ESD_TIMEOUT	(2 * HZ)


#define Reserved1_data 	0x3F
#define Reserved2_data	7
#define ETTh                0x3F    /* 0x3F means NO threshold */

#define DEFAULT_MTU         ETHERMTU
#define DEFAULT_RX_BUF_LEN  1536

#define kTransmitQueueCapacity  384

#define MBit				1000000

#ifdef R1000_JUMBO_FRAME_SUPPORT
#define MAX_JUMBO_FRAME_MTU	( 10000 )
#define MAX_RX_SKBDATA_SIZE	( MAX_JUMBO_FRAME_MTU + ETH_HDR_LEN )
#define MAX_TX_SKBDATA_SIZE ( MAX_JUMBO_FRAME_MTU + ETH_HDR_LEN )
#else
//#define MAX_RX_SKBDATA_SIZE 1600
#define MAX_RX_SKBDATA_SIZE 1608
#define MAX_TX_SKBDATA_SIZE 1608
#endif //end #ifdef R1000_JUMBO_FRAME_SUPPORT

#define NUM_TX_DESC         1024	/* Number of Tx descriptor registers*/
#define NUM_RX_DESC         1024	/* Number of Rx descriptor registers*/
#define RTL8169_NUM_TX_DESC	256		/* Number of Tx descriptor registers */
#define RTL8169_NUM_RX_DESC	256		/* Number of Rx descriptor registers */

#define RTL_MIN_IO_SIZE     0x80
#define TX_TIMEOUT			10000	//Sleep time in milliseconds, old value(6*HZ)
#define R1000_TIMER_EXPIRE_TIME 100 //100

#define PCFG_METHOD_1		0x01	//PHY Reg 0x03 bit0-3 == 0x0000
#define PCFG_METHOD_2		0x02	//PHY Reg 0x03 bit0-3 == 0x0001
#define PCFG_METHOD_3		0x03	//PHY Reg 0x03 bit0-3 == 0x0002

// Map to C99 standard types for 64-bit correctness.
typedef unsigned char	uchar,  u8;
typedef uint16_t		ushort, u16;
typedef uint32_t		ulong,  u32;
typedef signed char		schar,  s8;
typedef int16_t			sshort, s16;
typedef int32_t			slong,  s32;

//
// *** N.B.: The order of these entries MUST match 
// ***       the struct rtl_chip_info in RealtekR1000SL.cpp!!
// Sorted in alphanumeric order by chip model.
//

enum mcfg_methods
{
	//
	// RTL8100 series
	//
	MCFG_8100E_1,	// NO CFG_METHOD
	MCFG_8100E_2,	// NO CFG_METHOD

	MCFG_8101E_1,	// 8100 CFG_METHOD_1
	MCFG_8101E_2,	// 8100 CFG_METHOD_2
	MCFG_8101E_3,	// 8100 CFG_METHOD_3

	MCFG_8102E_1,	// 8100 CFG_METHOD_4
	MCFG_8102E_2,	// 8100 CFG_METHOD_5

	MCFG_8103E_1,	// 8100 CFG_METHOD_6
	MCFG_8103E_2,	// 8100 CFG_METHOD_7
	MCFG_8103E_3,	// 8100 CFG_METHOD_8
	//
	// RTL8100 series
	//
	MCFG_8401_1,	// 8100 CFG_METHOD__9
  
	/* NOTE: The CFG_METHOD number jumps here */
	MCFG_8105E_1,	// 8100 CFG_METHOD__10
	MCFG_8105E_2,	// 8100 CFG_METHOD__11
	MCFG_8105E_3,	// 8100 CFG_METHOD__12
	MCFG_8105E_4,	// 8100 CFG_METHOD__13
  
 	MCFG_8402_1,	// 8100 CFG_METHOD__14
  
  MCFG_8106E_1,
  MCFG_8106E_2,
  MCFG_8106EUS,
 

	//
	// RTL8168 series
	//
	MCFG_8168B_1,	// 8168 CFG_METHOD__1
	MCFG_8168B_2,	// 8168 CFG_METHOD__2
	MCFG_8168B_3,	// 8168 CFG_METHOD__3

	MCFG_8168C_1,	// 8168 CFG_METHOD__4
	MCFG_8168C_2,	// 8168 CFG_METHOD__5
	MCFG_8168C_3,	// 8168 CFG_METHOD__6

	MCFG_8168CP_1,	// 8168 CFG_METHOD__7
	MCFG_8168CP_2,	// 8168 CFG_METHOD__8

	MCFG_8168D_1,	// 8168 CFG_METHOD__9
	MCFG_8168D_2,	// 8168 CFG_METHOD__10

	MCFG_8168DP_1,	// 8168 CFG_METHOD__11
	MCFG_8168DP_2,	// 8168 CFG_METHOD__12
	MCFG_8168DP_3,	// 8168 CFG_METHOD__13

	MCFG_8168E_1,	// 8168 CFG_METHOD__14
	MCFG_8168E_2,	// 8168 CFG_METHOD__15

	MCFG_8168E_VL_1,	// 8168 CFG_METHOD__16
	MCFG_8168E_VL_2,	// 8168 CFG_METHOD__17

	MCFG_8168F_1,	// 8168 CFG_METHOD__18
	MCFG_8168F_2,	// 8168 CFG_METHOD__19
  
	MCFG_8411_1,  //CFG_METHOD__20,
	CFG_METHOD_21,
	CFG_METHOD_22,
	CFG_METHOD_23,
	CFG_METHOD_24,
  CFG_METHOD_25,
  MCFG_8411B,
  CFG_METHOD_27,


	//
	// RTL8169 series
	//
	MCFG_8169_1,	// 8169 CFG_METHOD_1

	MCFG_8169S_1,	// 8169 CFG_METHOD_2
	MCFG_8169S_2,	// 8169 CFG_METHOD_3

	MCFG_8169SB_1,	// 8169 CFG_METHOD_4

	MCFG_8169SC_1,	// 8169 CFG_METHOD_5
	MCFG_8169SC_2,	// 8169 CFG_METHOD_6



	MCFG_LIMIT
};

/* This definitions should have been in IOPCIDevice.h. */
enum
{
  kIOPCIPMCapability = 2,
};

enum
{
  kIOPCIELinkCapability = 12,
  kIOPCIELinkControl = 16,
};


#define OOB_CMD_RESET		0x00
#define OOB_CMD_DRIVER_START	0x05
#define OOB_CMD_DRIVER_STOP	0x06
#define OOB_CMD_SET_IPMAC	0x41

//Ram Code Version
#define NIC_RAMCODE_VERSION_CFG_METHOD_14 (0x0057)
#define NIC_RAMCODE_VERSION_MCFG_8168E_VL_1 (0x0055)
#define NIC_RAMCODE_VERSION_MCFG_8168F_1 (0x0044)
#define NIC_RAMCODE_VERSION_MCFG_8411_1 (0x0044)
#define NIC_RAMCODE_VERSION_CFG_METHOD_21 (0x0019)
#define NIC_RAMCODE_VERSION_CFG_METHOD_24 (0x0001)
#define NIC_RAMCODE_VERSION_CFG_METHOD_23 (0x0015)
#define NIC_RAMCODE_VERSION_MCFG_8411B (0x0012)


// Convenience macros for distinguishing chip families

#define MCFG_IS_8100(cfg) ((cfg >= MCFG_8100E_1) && (cfg <= MCFG_8106EUS)) 
//#define MCFG_IS_8168(cfg) (((cfg >= MCFG_8168B_1) && (cfg <= MCFG_8168F_2)) || (cfg == MCFG_8411_1))
#define MCFG_IS_8168(cfg) ((cfg >= MCFG_8168B_1) && (cfg <= CFG_METHOD_27))
#define MCFG_IS_8169(cfg) ((cfg >= MCFG_8169_1) && (cfg <= MCFG_8169SC_2))

#define WAKE_PHY 1
#define WAKE_UCAST 2
#define WAKE_BCAST 4
#define WAKE_MCAST 8
#define WAKE_MAGIC 0x10
#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

#define ARRAY_SIZE(abc) (sizeof(abc) / sizeof(abc[0]))

enum R1000_DSM_STATE
{
	DSM_MAC_INIT = 1,
	DSM_NIC_GOTO_D3 = 2,
	DSM_IF_DOWN = 3,
	DSM_NIC_RESUME_D3 = 4,
	DSM_IF_UP = 5,
};


// added regs from Linux RTL8168 driver
// Chucko 04 Oct 2009
enum r1000_registers {
	MAC0 = 0x00,	/* Ethernet hardware address. */
	MAC4 = 0x04,
	MAR0 = 0x08,	/* Multicast filter. */
	CounterAddrLow  = 0x10,
	CounterAddrHigh	= 0x14,
	TxDescStartAddr	= 0x20,		/* Linux TxDescStartAddrLow */
	TxDescStartAddrHigh	= 0x24,
	TxHDescStartAddr= 0x28,		/* Linux TxHDescStartAddrLow */
	TxHDescStartAddrHigh = 0x2c,
	FLASH	= 0x30,	/* Not in RTL8101 */
	ERSR	= 0x36,
	ChipCmd	= 0x37,
	TxPoll	= 0x38,
	IntrMask = 0x3C,
	IntrStatus = 0x3E,
	TxConfig = 0x40,
	RxConfig = 0x44,
	TCTR	 = 0x48,	/* Not in RTL8101 */
	RxMissed = 0x4C,	/* Commented out in RTL8101 */
	Cfg9346 = 0x50,
	Config0	= 0x51,
	Config1	= 0x52,
	Config2	= 0x53,
	Config3	= 0x54,
	Config4	= 0x55,
	Config5	= 0x56,
  	TDFNR			= 0x57,
	TimeIntr = 0x58,	/* Not in RTL8101 */
	MultiIntr = 0x5C,	/* Not in RTL8101 */
	PHYAR	= 0x60,
	CSIDR	= 0x64,		/* was TBICSR */
	CSIAR	= 0x68,		/* was TBI_ANAR */
	TBI_LPAR = 0x6A,	/* Not in RTL8101 nor RTL8168 */
	PHYstatus = 0x6C,
	MACDBG	= 0x6D,
	GPIO	= 0x6E,
	PMCH	= 0x6F,
	ERIDR	= 0x70,
	ERIAR	= 0x74,
	EPHY_RXER_NUM = 0x7C,	/* Neither this nor the below are in RTL8101 */
	Offset_7Ch = 0x7C,		/* Linux EPHY_RXER_NUM */
	EPHYAR	= 0x80,
	OCPDR   = 0xB0,	/* Not in RTL8101 */
  MACOCP	= 0xB0,
	OCPAR   = 0xB4,	/* Not in RTL8101 */
  PHYOCP	= 0xB8,
	DBG_reg	= 0xD1,
  MCUCmd_reg		= 0xD3,
	RxMaxSize = 0xDA,
	EFUSEAR = 0xDC,	/* Not in RTL8101 */
	CPlusCmd = 0xE0,
	IntrMitigate = 0xE2,
	RxDescStartAddr	= 0xE4,
	RxDescAddrLow   = 0xE4,
	RxDescAddrHigh  = 0xE8,
	ETThReg	= 0xEC,		/* Linux Reserved1 */	/* aka MTPS */
	FuncEvent	= 0xF0,
	FuncEventMask	= 0xF4,
	FuncPresetState	= 0xF8,
	PHYIO           = 0xF8, /* Only in RTL8101 */
	FuncForceEvent	= 0xFC,
};

enum r1000_register_content {
	/*InterruptStatusBits*/
	SYSErr 		= 0x8000,
	PCSTimeout	= 0x4000,
	SWInt		= 0x0100,
	TxDescUnavail	= 0x80,
	RxFIFOOver 	= 0x40,
	LinkChg 	= 0x20,
	RxOverflow 	= 0x10,	/* Not in either RTL8101 nor RTL8168 */
	RxDescUnavail = 0x10,
	TxErr 	= 0x08,
	TxOK 	= 0x04,
	RxErr 	= 0x02,
	RxOK 	= 0x01,
	
	/*RxStatusDesc*/
	RxRWT = 0x00400000,
	RxRES = 0x00200000,
	RxRUNT= 0x00100000,
	RxCRC = 0x00080000,
	
	/*ChipCmdBits*/
	StopReq  = 0x80,
	CmdReset = 0x10,
	CmdRxEnb = 0x08,
	CmdTxEnb = 0x04,
	RxBufEmpty = 0x01,
	
	/*Cfg9346Bits*/
	Cfg9346_Lock = 0x00,
	Cfg9346_Unlock = 0xC0,
	Cfg9346_EEDO = (1 << 0),
	Cfg9346_EEDI = (1 << 1),
	Cfg9346_EESK = (1 << 2),
	Cfg9346_EECS = (1 << 3),
	Cfg9346_EEM0 = (1 << 6),
	Cfg9346_EEM1 = (1 << 7),
	
	/*rx_mode_bits*/
	AcceptErr = 0x20,
	AcceptRunt = 0x10,
	AcceptBroadcast = 0x08,
	AcceptMulticast = 0x04,
	AcceptMyPhys = 0x02,
	AcceptAllPhys = 0x01,
	
	/* Transmit Priority Polling*/
	HPQ = 0x80,
	NPQ = 0x40,
	FSWInt = 0x01,
	
	/*RxConfigBits*/
	RxCfgFIFOShift = 13,
	RxCfgDMAShift = 8,
	RxCfg_128_int_en = (1 << 15),	/* Not in RTL 8101 */
	RxCfg_fet_multi_en = (1 << 14),	/* Not in RTL 8101 */
	RxCfg_half_refetch = (1 << 13),	/* Not in RTL 8101 */
	RxCfg_9356SEL = (1 << 6),
	
	/*TxConfigBits*/
	TxInterFrameGapShift = 24,
	TxDMAShift = 8,	/* DMA burst value (0-7) is shift this many bits */
	TxMACLoopBack = (1 << 17),	/* MAC loopback */
	
	/* Config1 register p.24 */
	LEDS1		= (1 << 7),
	LEDS0		= (1 << 6),
	Speed_down	= (1 << 4),
	MEMMAP		= (1 << 3),
	IOMAP		= (1 << 2),
	VPD			= (1 << 1),
	PMEnable	= (1 << 0),	/* Power Management Enable */

  /* Config2 register */
  PMSTS_En    = (1 << 5),
	
	/* Config3 register */
  Isolate_en  = (1 << 12), /* Isolate enable */
	MagicPacket	= (1 << 5),	/* Wake up when receives a Magic Packet */
	LinkUp		= (1 << 4),	/* This bit is reserved in RTL8168B.*/
	/* Wake up when the cable connection is re-established */
	/* RTL8101 Does has the following four bits in Config5, not Config3 */
	ECRCEN		= (1 << 3),	/* This bit is reserved in RTL8168B*/
	Jumbo_En0	= (1 << 2),	/* This bit is reserved in RTL8168B*/
	RDY_TO_L23	= (1 << 1),	/* This bit is reserved in RTL8168B*/
	Beacon_en	= (1 << 0),	/* This bit is reserved in RTL8168B*/
	
	/* Config4 register */
	Jumbo_En1	= (1 << 1),	/* This bit is reserved in RTL8168B*/
	
	/* Config5 register */
	BWF		= (1 << 6),	/* Accept Broadcast wakeup frame */
	MWF		= (1 << 5),	/* Accept Multicast wakeup frame */
	UWF		= (1 << 4),	/* Accept Unicast wakeup frame */
	LanWake		= (1 << 1),	/* LanWake enable/disable */
	PMEStatus	= (1 << 0),	/* PME status can be reset by PCI RST# */

	Jumbo_En	= (1 << 2),	/* Only defined for RTL8101 */
	
	/* CPlusCmd */
	EnableBist	= (1 << 15),
	Macdbgo_oe	= (1 << 14),
	Normal_mode	= (1 << 13),
	Force_halfdup	= (1 << 12),
	Force_halpdup	= (1 << 12),	/* Pretty sure this is a typo in RTL8101 */
	Force_rxflow_en	= (1 << 11),
	Force_txflow_en	= (1 << 10),
	Cxpl_dbg_sel	= (1 << 9),//This bit is reserved in RTL8168B
	ASF		= (1 << 8),//This bit is reserved in RTL8168C
	PktCntrDisable	= (1 << 7),
	RxVlan		= (1 << 6),
	RxChkSum	= (1 << 5),
	PCIDAC		= (1 << 4),	/* This is only defined for RTL8101 */
	Macdbgo_sel	= 0x001C,
	INTT_0		= 0x0000,
	INTT_1		= 0x0001,
	INTT_2		= 0x0002,
	INTT_3		= 0x0003,
	
	/*rtl8169_PHYstatus (MAC offset 0x6C)*/
	TBI_Enable	= 0x80,		/* Not defined in RTL8168 not RTL8101 */
	TxFlowCtrl	= 0x40,
	RxFlowCtrl	= 0x20,
	_1000Mbps	= 0x10,		/* Linux _1000bps */ /* Not in RTL8101 */
	_100Mbps	= 0x08,		/* Linux _100bps */
	_10Mbps		= 0x04,		/* Linux _10bps */
	LinkStatus	= 0x02,
	FullDup		= 0x01,
	
	/* DBG_reg */
	/* These are not defined for RTL8101 */
	Fix_Nak_1 = (1 << 4),
	Fix_Nak_2 = (1 << 3),
	DBGPIN_E2 = (1 << 0),
	
	/* DumpCounterCommand */
	CounterDump = 0x8,
	
	/* PHY access */
	PHYAR_Flag = 0x80000000,
	PHYAR_Write = 0x80000000,
	PHYAR_Read = 0x00000000,
	PHYAR_Reg_Mask = 0x1f,
	PHYAR_Reg_shift = 16,
	PHYAR_Data_Mask = 0xffff,

	/* Only in RTL8101 */
	/* PHY IO access */
	PHYIO_Flag = 0x80000000,
	PHYIO_Write = 0x80000000,
	PHYIO_Read = 0x00000000,
	PHYIO_Reg_Mask = 0x1f,
	PHYIO_Reg_shift = 16,
	PHYIO_Data_Mask = 0xffff,

	/* EPHY access */
	EPHYAR_Flag = 0x80000000,
	EPHYAR_Write = 0x80000000,
	EPHYAR_Read = 0x00000000,
	EPHYAR_Reg_Mask = 0x1f,
	EPHYAR_Reg_shift = 16,
	EPHYAR_Data_Mask = 0xffff,
	
	/* CSI access */	
	CSIAR_Flag = 0x80000000,
	CSIAR_Write = 0x80000000,
	CSIAR_Read = 0x00000000,
	CSIAR_ByteEn = 0x0f,
	CSIAR_ByteEn_shift = 12,
	CSIAR_Addr_Mask = 0x0fff,
	
	/* ERI access */
	ERIAR_Flag = 0x80000000,
	ERIAR_Write = 0x80000000,
	ERIAR_Read = 0x00000000,
	ERIAR_Addr_Align = 4, /* ERI access register address must be 4 byte alignment */
	ERIAR_ExGMAC = 0,
	ERIAR_MSIX = 1,
	ERIAR_ASF = 2,
  ERIAR_OOB = 2,
	ERIAR_Type_shift = 16,
	ERIAR_ByteEn = 0x0f,
	ERIAR_ByteEn_shift = 12,
	
	/* Not in RTL8101 */
	/* OCP GPHY access */
	OCPDR_Write = 0x80000000,
	OCPDR_Read = 0x00000000,
	OCPDR_Reg_Mask = 0xFF,
	OCPDR_Data_Mask = 0xFFFF,
	OCPDR_GPHY_Reg_shift = 12,
	OCPAR_Flag = 0x80000000,
	OCPAR_GPHY_Write = 0x8000F060,
	OCPAR_GPHY_Read = 0x0000F060,
	OCPR_Write = 0x80000000,
	OCPR_Read = 0x00000000,
	OCPR_Addr_Reg_shift = 16,
	OCPR_Flag = 0x80000000,
	OCP_STD_PHY_BASE_PAGE = 0x0A40,
	
	/* MCU Command */
	Now_is_oob = (1 << 7),
	Txfifo_empty = (1 << 5),
	Rxfifo_empty = (1 << 4),
  
	/* Not in RTL8101 */
	/* E-FUSE access */
	EFUSE_WRITE	= 0x80000000,
	EFUSE_WRITE_OK	= 0x00000000,
	EFUSE_READ	= 0x00000000,
	EFUSE_READ_OK	= 0x80000000,
	EFUSE_Reg_Mask	= 0x03FF,
	EFUSE_Reg_Shift	= 8,
	EFUSE_Check_Cnt	= 300,
	EFUSE_READ_FAIL	= 0xFF,
	EFUSE_Data_Mask	= 0x000000FF,
	
	/* GPIO */
	GPIO_en = (1 << 0),	
	
	/* Obviously, RTL8101 does not support _any_ of the Gigabit registers */

	/*GIGABIT_PHY_registers*/
	PHY_BMCR          = 0,	/* Linux MII_BMCR */
	PHY_BMSR          = 1,	/* Linux MII_BMSR */
	PHY_AD1           = 2,  /* Linux MII_PHYSID1 */
	PHY_AD2           = 3,  /* Linux MII_PHYSID2 */
	PHY_AUTO_NEGO_REG	= 4,	/* Linux PHY_AUTO_NEGO_REG */
	PHY_ANAR          = 4,  /* Linux       */
	PHY_ANLPAR        = 5,  /* Linux MII_LPA  */
	PHY_ANER          = 6,  /* Linux MII_EXPANSION */
	PHY_ANNPTR        = 7,  /* Linux */
	PHY_ANNRPR        = 8,  /* Linux */
	PHY_1000_CTRL_REG	= 9,	/* Linux MII_CTRL1000 */
	PHY_GBCR          = 9,  /* Linux */
	PHY_GBSR          = 0x0A,  /* Linux MII_STAT1000 */
	PHY_MMD_CTRL      = 0x0D,  /* Linux MII_MMD_CTRL */
	PHY_MMD_DATA      = 0x0E,  /* Linux MII_MMD_DATA */
	PHY_GBESR         = 0x0F,  /* Linux MII_ESTATUS  */
	
	// PHY_BMCR = 0;
	PHY_Restart_Auto_Nego	= 0x0200,
	PHY_Enable_Auto_Nego	= 0x1000,
	PHY_Power_Down			= 0x0800,
	// alternate names from Linux drivers
	BMCR_RESET				= 0x8000,
	BMCR_ANENABLE			= 0x1000,
	BMCR_PDOWN				= 0x0800,
	BMCR_ANRESTART		= 0x0200,
	BMCR_FULLDPLX			= 0x0100,
	BMCR_SPEED100			= 0x0040,
	BMCR_SPEED10			= 0x0000,
	
	//PHY_BMSR = 1;
	PHY_Auto_Nego_Comp	= 0x0020,
	// alternate names
	BMSR_LSTATUS		= 0x0004,
	
	//PHY_AUTO_NEGO_REG = 4;
	PHY_Cap_10_Half		= 0x0020,	/* Linux ADVERTISE_10HALF */
	PHY_Cap_10_Full		= 0x0040,	/* Linux ADVERTISE_10FULL */
	PHY_Cap_100_Half	= 0x0080,	/* Linux ADVERTISE_100HALF */
	PHY_Cap_100_Full	= 0x0100,	/* Linux ADVERTISE_100FULL */
	
	//PHY_1000_CTRL_REG = 9;
	PHY_Cap_1000_Full	= 0x0200,	/* Linux ADVERTISE_1000FULL */
	PHY_Cap_1000_Half	= 0x0100,	/* Linux ADVERTISE_1000HALF */
	
	PHY_Cap_PAUSE		= 0x0400,   /* Linux ADVERTISE_PAUSE_CAP */
	PHY_Cap_ASYM_PAUSE	= 0x0800,	/* Linux ADVERTISE_PAUSE_ASYM */
	
	PHY_Cap_Null		= 0x0,
	
	/*_MediaType*/
	_10_Half	= 0x01,
	_10_Full	= 0x02,
	_100_Half	= 0x04,
	_100_Full	= 0x08,
	_1000_Full	= 0x10,
	
	/*_TBICSRBit*/
	TBILinkOK 	= 0x02000000,
};

/* Link partner ability register. */
#define LPA_SLCT                0x001f  /* Same as advertise selector  */
#define LPA_10HALF              0x0020  /* Can do 10mbps half-duplex   */
#define LPA_1000XFULL           0x0020  /* Can do 1000BASE-X full-duplex */
#define LPA_10FULL              0x0040  /* Can do 10mbps full-duplex   */
#define LPA_1000XHALF           0x0040  /* Can do 1000BASE-X half-duplex */
#define LPA_100HALF             0x0080  /* Can do 100mbps half-duplex  */
#define LPA_1000XPAUSE          0x0080  /* Can do 1000BASE-X pause     */
#define LPA_100FULL             0x0100  /* Can do 100mbps full-duplex  */
#define LPA_1000XPAUSE_ASYM     0x0100  /* Can do 1000BASE-X pause asym*/
#define LPA_100BASE4            0x0200  /* Can do 100mbps 4k packets   */
#define LPA_PAUSE_CAP           0x0400  /* Can pause                   */
#define LPA_PAUSE_ASYM          0x0800  /* Can pause asymetrically     */
#define LPA_RESV                0x1000  /* Unused...                   */
#define LPA_RFAULT              0x2000  /* Link partner faulted        */
#define LPA_LPACK               0x4000  /* Link partner acked us       */
#define LPA_NPAGE               0x8000  /* Next page bit               */
 
#define LPA_DUPLEX              (LPA_10FULL | LPA_100FULL)
#define LPA_100                 (LPA_100FULL | LPA_100HALF | LPA_100BASE4)



enum _DescStatusBit {
	DescOwn		= (1 << 31), /* Descriptor is owned by NIC */
	RingEnd		= (1 << 30), /* End of descriptor ring */
	FirstFrag	= (1 << 29), /* First segment of a packet */
	LastFrag	= (1 << 28), /* Final segment of a packet */
	
	/* Tx private */
	/*------ offset 0 of tx descriptor ------*/
	LargeSend	= (1 << 27), /* TCP Large Send Offload (TSO) */
  LargeSend_DP = (1 << 16), /* TCP Large Send Offload (TSO) */
	MSSShift	= 16,        /* MSS value position */
	MSSMask		= 0xfff,     /* MSS value + LargeSend bit: 12 bits */
							 /* NOTE: in RTL8168 MSSMask is set to 0x7ffU */
	TxIPCS		= (1 << 18), /* Calculate IP checksum */
	TxUDPCS		= (1 << 17), /* Calculate UDP/IP checksum */
	TxTCPCS		= (1 << 16), /* Calculate TCP/IP checksum */
	TxVlanTag	= (1 << 17), /* Add VLAN tag */
	
	/*@@@@@@ offset 4 of tx descriptor => bits for RTL8102E, RTL8168C/CP only		begin @@@@@@*/
	TxUDPCS_C	= (1 << 31), /* Calculate UDP/IP checksum */
	TxTCPCS_C	= (1 << 30), /* Calculate TCP/IP checksum */
	TxIPCS_C	= (1 << 29), /* Calculate IP checksum */
	/*@@@@@@ offset 4 of tx descriptor => bits for RTL8102E, RTL8168C/CP only		end @@@@@@*/
	
	
	/* Rx private */
	/*------ offset 0 of rx descriptor ------*/
	PID1		= (1 << 18), /* Protocol ID bit 1/2 */
	PID0		= (1 << 17), /* Protocol ID bit 2/2 */
	
#define RxProtoUDP	(PID1)
#define RxProtoTCP	(PID0)
#define RxProtoIP	(PID1 | PID0)
#define RxProtoMask	RxProtoIP
	
	RxIPF		= (1 << 16), /* IP checksum failed */
	RxUDPF		= (1 << 15), /* UDP/IP checksum failed */
	RxTCPF		= (1 << 14), /* TCP/IP checksum failed */
	RxVlanTag	= (1 << 16), /* VLAN tag available */
	
	/*@@@@@@ offset 0 of rx descriptor => bits for RTL8102E, RTL8168C/CP only		begin @@@@@@*/
	RxUDPT		= (1 << 18),
	RxTCPT		= (1 << 17),
	/*@@@@@@ offset 0 of rx descriptor => bits for RTL8102E, RTL8168C/CP only		end @@@@@@*/
	
	/*@@@@@@ offset 4 of rx descriptor => bits for RTL8102E, RTL8168C/CP only		begin @@@@@@*/
	RxV6F		= (1 << 31),
	RxV4F		= (1 << 30),
	/*@@@@@@ offset 4 of rx descriptor => bits for RTL8102E, RTL8168C/CP only		end @@@@@@*/
};

enum featuress {
  //	RTL_FEATURE_WOL	= (1 << 0),
	RTL_FEATURE_MSI	= (1 << 1),
};

/*
 Net device features */
#define NETIF_F_SG              1       /* Scatter/gather IO. */
#define NETIF_F_IP_CSUM         2       /* Can checksum only TCP/UDP over IPv4. */
#define NETIF_F_NO_CSUM         4       /* Does not require checksum. F.e. loopack. */
#define NETIF_F_HW_CSUM         8       /* Can checksum all the packets. */
#define NETIF_F_DYNALLOC        16      /* Self-dectructable device. */
#define NETIF_F_HIGHDMA         32      /* Can DMA to high memory. */
#define NETIF_F_FRAGLIST        1       /* Scatter/gather IO. */


enum wol_capability {
	WOL_DISABLED = 0,
	WOL_ENABLED = 1
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


enum effuse 
{
	EFUSE_SUPPORT = 1,
	EFUSE_NOT_SUPPORT = 0,
};
#define RsvdMask	0x3fffc000

struct TxDesc 
{
	u32		status;
	u32		vlan_tag;
	u32		buf_addr;
	u32		buf_Haddr;
};

struct RxDesc 
{
	u32		status;
	u32		vlan_tag;
	u32		buf_addr;
	u32		buf_Haddr;
};

struct ring_info {
	struct sk_buff	*skb;
	u32		len;
	u8		__pad[sizeof(void *) - sizeof(u32)];
};

// Used only in R1000InitBoard and rtl8168_esd_timer
// (and maybe somewhere in rtl8101)
struct pci_resource {
	u8	cmd;
	u8	cls;
	u16	io_base_h;
	u16	io_base_l;
	u16	mem_base_h;
	u16	mem_base_l;
	u8	ilr;
	u16	resv_0x20_h;
	u16	resv_0x20_l;
	u16	resv_0x24_h;
	u16	resv_0x24_l;
  u16	resv_0x2c_h;
	u16	resv_0x2c_l;
	u32	pci_nvidia_geforce_6200;
	u32 pci_nvidia_geforce__6250_1;
};

//EEPROM opcodes
#define	RTL_EEPROM_READ_OPCODE		06
#define	RTL_EEPROM_WRITE_OPCODE		05
#define	RTL_EEPROM_ERASE_OPCODE		07
#define	RTL_EEPROM_EWEN_OPCODE		19
#define	RTL_EEPROM_EWDS_OPCODE		16

#define	RTL_CLOCK_RATE	3

#endif
