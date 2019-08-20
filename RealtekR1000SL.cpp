/*
 *  RealtekR1000SL.cpp - OS specific and hardware generic methods
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

#include "RealtekR1000SL.h"
#include "impl_defs.h"

#define BaseClass IOEthernetController

OSDefineMetaClassAndStructors(RealtekR1000, IOEthernetController)

// **********************************
//
// Static Data Member Initialization
//
// **********************************

//
// Configuration data
//

//
// *** N.B.: The order of these entries MUST match 
// ***       the enum mcfg_methods in R1000Regs.h!!
//

const struct RtlChipInfo RealtekR1000::rtl_chip_info[] = 
{
	//
	// RTL810x Family
	//
	{"RTL8100E",			
		MCFG_8100E_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8100E",			
		MCFG_8100E_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8101E",
		MCFG_8101E_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8101E",
		MCFG_8101E_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8101E",
		MCFG_8101E_3,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8102E",
		MCFG_8102E_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8102E",
		MCFG_8102E_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8103E",
		MCFG_8103E_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8103E",
		MCFG_8103E_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8103E",
		MCFG_8103E_3,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8401E",
		MCFG_8401_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8105E",
		MCFG_8105E_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8105E",
		MCFG_8105E_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8105E",
		MCFG_8105E_3,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8105E",
		MCFG_8105E_4,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8402",
		MCFG_8402_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8106E",
		MCFG_8106E_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8106E",
		MCFG_8106E_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8106EUS",
		MCFG_8106EUS,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	
	//
	// RTL8168/8111 Family
	//
	{"RTL8168B/8111B",
		MCFG_8168B_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_4k,
		EFUSE_NOT_SUPPORT},
	{"RTL8168B/8111B",
		MCFG_8168B_2,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_4k,
		EFUSE_NOT_SUPPORT},
	{"RTL8168B/8111B",
		MCFG_8168B_3,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_4k,
		EFUSE_NOT_SUPPORT},
	{"RTL8168C/8111C",
		MCFG_8168C_1,
		1024,
		RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_6k,
		EFUSE_NOT_SUPPORT},	
	{"RTL8168C/8111C",
		MCFG_8168C_2,
		1024,
		RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_6k,
		EFUSE_NOT_SUPPORT},	
	{"RTL8168C/8111C",
		MCFG_8168C_3,
		1024,
		RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_6k,
		EFUSE_NOT_SUPPORT},
	{"RTL8168CP/8111CP",
		MCFG_8168CP_1,
		1024,
		RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_6k,
		EFUSE_NOT_SUPPORT},
	{"RTL8168CP/8111CP",
		MCFG_8168CP_2,
		1024,
		RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_6k,
		EFUSE_NOT_SUPPORT},
	{"RTL8168D/8111D",
		MCFG_8168D_1,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168D/8111D",
		MCFG_8168D_2,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168DP/8111DP",
		MCFG_8168DP_1,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168DP/8111DP",
		MCFG_8168DP_2,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168DP/8111DP",
		MCFG_8168DP_3,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168E/8111E",
		MCFG_8168E_1,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168E/8111E",
		MCFG_8168E_2,
		1024,
		RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168E-VL/8111E-VL",
		MCFG_8168E_VL_1,
		1024,
		RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168E-VL/8111E-VL",
		MCFG_8168E_VL_2,
		1024,
		RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168F/8111F",
		MCFG_8168F_1,
		1024,
		RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_SUPPORT},
	{"RTL8168F/8111F",
		MCFG_8168F_2,
		1024,
		RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k},
	{"RTL8411",
		MCFG_8411_1,  //MCFG_8411_1
		1024,
		(RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_9k,
		EFUSE_NOT_SUPPORT},
  {"RTL8168G/8111G",
     CFG_METHOD_21,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},

  {"RTL8168G/8111G",
     CFG_METHOD_22,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},

  {"RTL8168EP/8111EP",
     CFG_METHOD_23,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},

  {"RTL8168GU/8111GU",
     CFG_METHOD_24,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},
  {"RTL8168GU/8111GU",
     CFG_METHOD_25,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},

  {"RTL8411B",
     MCFG_8411B,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},

  {"RTL8168EP/8111EP",
     CFG_METHOD_27,
     RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
     0xff7e1880,
    Jumbo_Frame_9k},
  

	//
	// RTL8169/8110 Family
	//
	{"RTL8169",
		MCFG_8169_1,
		256,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_7k,
		EFUSE_NOT_SUPPORT},
	{"RTL8169S/8110S",
		MCFG_8169S_1,
		256,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_7k,
		EFUSE_NOT_SUPPORT},
	{"RTL8169S/8110S",
		MCFG_8169S_2,
		256,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_7k,
		EFUSE_NOT_SUPPORT},
	{"RTL8169SB/8110SB",
		MCFG_8169SB_1,
		256,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_7k,
		EFUSE_NOT_SUPPORT},
	{"RTL8169SC/8110SC",
		MCFG_8169SC_1,
		256,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_7k,
		EFUSE_NOT_SUPPORT},
	{"RTL8169SC/8110SC",
		MCFG_8169SC_2,
		256,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		Jumbo_Frame_7k,
		EFUSE_NOT_SUPPORT},
	
	//
	// RTL8401
	//
	{"RTL8401",
		MCFG_8401_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},
	{"RTL8401",
		MCFG_8402_1,
		1024,
		(RX_FIFO_THRESH_NONE << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift),
		0xff7e1880,
		0,
		EFUSE_NOT_SUPPORT},

	// *** end of table ***
	{ 0 }
};

//
// Power management table
//

struct IOPMPowerState RealtekR1000::powerStateArray[ kR1000PowerStateCount ] =
{
	// kR1000PowerStateOff
	{ kIOPMPowerStateVersion1,0,0,0,0,0,0,0,0,0,0,0 },
	// kR1000PowerStateOn
	{ kIOPMPowerStateVersion1,				// version
		IOPMDeviceUsable,					// capability
		IOPMPowerOn,						// pwr character
		IOPMPowerOn,						// pwr requirement
		// below this line unused, 0 is OK
		0,									// static pwr in mw
		0,									// unbudgeted pwr
		0,									// pwr to attain
		0,									// time to attain in us
		0,									// settle up time
		0,									// time to lower
		0,									// settle down time
		0									// power domain budget
	}
};


/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
int RealtekR1000::max_interrupt_work = 20;

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
 The RTL chips use a 64 element hash table based on the Ethernet CRC.  */
UInt32 RealtekR1000::multicast_filter_limit = 32;

const u32 RealtekR1000::ethernet_polynomial = 0x04c11db7U;

// Default interrupt mask.  See chip-specific code for more.
const u16 RealtekR1000::r1000_intr_mask =
	LinkChg | RxDescUnavail | RxFIFOOver | TxErr | TxOK | RxErr | RxOK ;

/*
 * Initialization of driver instance,
 * i.e. resources allocation and so on.
 */
// FIXME - be sure to initialize all member variables!
bool RealtekR1000::init(OSDictionary *properties)
{
	//
	// logging
	//
	strncpy(bsdName, "RealtekR1000", BSD_NAME_LEN);
	
	DLog("init\n");
	if (BaseClass::init(properties) == false) 
	{
		DLog("BaseClass::init() returned false!\n");
		return false;
	}
	
	//
	// pointers to OS objects
	//
	pciDev = NULL;
	workLoop = NULL;
	intSource = NULL;
    timerSource = NULL;
    netStats = NULL;
    etherStats = NULL;
	transmitQueue = NULL;
    etherif = NULL;
	mediumDict = NULL;
	
	//
	// capabilities to advertise to OS
	//
	canOffload = 0;
	
	//
	// internal driver status
	//
	board_inited = false;
	buffers_inited = false;
	enabled = false;
	linked = false;
	
	activationLevel = kActivationLevelNone;
	enabledForKDP = enabledForBSD = false;
	
	powerState = kR1000PowerStateOn;		// presume power is already on at startup
	
	//
	// Tx & Rx ring buffer management
	//
	R1000InitRingIndices();
	rxdesc_space = NULL;
	RxDescArray = NULL;
	rx_descMd = NULL;
	rxdesc_phy_dma_addr = NULL;
	txdesc_space = NULL;
	TxDescArray = NULL;
	tx_descMd = NULL;
	txdesc_phy_dma_addr = NULL;
	for (int i = 0; i < NUM_TX_DESC; i++)
	{
		Tx_skbuff[i] = NULL;
	}
	InitializeBufferMemoryPointers();
	
	//
	// register addressing
	//
	pioBase = 0;
	mmioBase = NULL;
	forcedPio = true;
  wolCapable = false;
  wolActive = false;
	
	return true;
}

/*
 * Calling before destroying driver instance.
 * Frees all allocated resources.
 */
void RealtekR1000::free()
{
	DLog("free\n");
	
	// give back our buffer pool
	FreeBufferMemory();
	FreeDescriptorsMemory();
	
	//free resource of base instance
	if (intSource && workLoop)
	{
		//Detaching interrupt source from work loop
		workLoop->removeEventSource(intSource);
	}
	
	// TODO - figure out if this order is significant!
	// TODO - release contents of various pointer tables
	RELEASE(etherif);
    RELEASE(intSource);
    RELEASE(timerSource);
    RELEASE(mmioBase);
    RELEASE(pciDev);
    RELEASE(workLoop);
	RELEASE(mediumDict);
	
	BaseClass::free();
} 

/*
 * Starting driver.
 */
bool RealtekR1000::start(IOService *provider)
{
	DLog("start\n");
	if (!BaseClass::start(provider))
	{
		DLog("start: Failed, super::start returned false\n");
		return false;
	}
	pciDev = OSDynamicCast(IOPCIDevice, provider);
	if (pciDev == NULL)
	{
		DLog("start: Failed, provider is not IOPCIDevice\n");
		return false;
	}
	// Increment OS reference count
	pciDev->retain();
	
	if (!pciDev->open(this))
	{
		DLog("start: Failed to open PCI Device/Nub\n");
		return false;
	}

	bool success = false;

	do
	{
		// FIXME - Defer adding Gigabit media until we've ID'd the chip
		// See Apple driver: RTL8139PHY.cpp
		
		// Assuming RTL8168B/8111B by default
		// Actual chipset will be ID'd in R1000InitBoard(),
		// called from R1000ProbeAndStartBoard() below
		mcfg = MCFG_8168B_1;
		
		//Adding Mac OS X PHY's
		mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);
		OSAddNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO);	
		OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 * MBit, MEDIUM_INDEX_10HD);
		OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 * MBit, MEDIUM_INDEX_10FD);
		OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 * MBit, MEDIUM_INDEX_100HD);
		OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 * MBit, MEDIUM_INDEX_100FD);	
		OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionHalfDuplex, 1000 * MBit, MEDIUM_INDEX_1000HD);
		OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionFullDuplex, 1000 * MBit, MEDIUM_INDEX_1000FD);
		
		if (!publishMediumDictionary(mediumDict))
		{
			DLog("start: Failed, publishMediumDictionary returned false\n");
			break;
		}
		
		if (!R1000ProbeAndStartBoard())
		{
			DLog("start: Failed, R1000ProbeAndStartBoard returned false\n");
			break;
		}
		
		if (!AllocateDescriptorsMemory())
		{
			DLog("start: Failed, AllocateDescriptorsMemory returned false\n");
			break;
		}
		
		if (!AllocateBufferMemory())
		{
			DLog("start: Failed, AllocateBufferMemory returned false\n");
			break;
		}
		
		InitializeRingBufferDescriptors();
		
		if (!R1000InitEventSources(provider))
		{
			DLog("start: Failed, R1000InitEventSources returned false\n");
			break;
		}
		
		// set up power management
		PMinit();
		pciDev->joinPMtree(this);
		
		success = true;
	}
	while ( false );
	
	// Close our provider, it will be re-opened on demand when
	// our enable() is called by a client.
	if (pciDev != NULL)
	{
		pciDev->close(this);
	}
	
	do
	{
		// break if we've had an error before this
    if ( false == success )
		{
			break;
		}
		// Attaching dynamic link layer
		// Callback to configureInterface() method happens here
		if (false == attachInterface((IONetworkInterface**)&etherif, false))
		{
			DLog("start: Failed 'attachInterface' in attaching to data link layer\n");
			break;
		}
		
		// Do this here instead of in attachInterface() to allow us to cleanly finish initializing
		etherif->registerService();
		success = true;
	}
	while ( false );
	
	DLog("start: returning '%d'\n",success);
	return success;
}

/*
 * Stopping driver.
 */

// FIXME - should we return buffer memory here?
void RealtekR1000::stop(IOService *provider)
{
	DLog("stop\n");
	detachInterface(etherif);
	R1000StopBoard();
	PMstop();
	BaseClass::stop(provider);
}

bool RealtekR1000::OSAddNetworkMedium(ulong type, UInt32 bps, ulong index)
{
	IONetworkMedium *medium = IONetworkMedium::medium(type, bps, 0, index);
	if (!medium) 
	{
		IOLog("Couldn't allocate medium\n");		
		return false;
	}
	if (!IONetworkMedium::addMedium(mediumDict, medium)) 
	{
		IOLog("Couldn't add medium\n");
		return false;
	}
	mediumTable[index] = medium;
	return true;
}

bool RealtekR1000::increaseActivationLevel(UInt32 level)
{
	bool ret = false;
	
	switch (level)
	{
		case kActivationLevelKDP:
		{
			if (!pciDev) break;
			pciDev->open(this);
			
			// PHY medium selection.
			const IONetworkMedium *medium = getSelectedMedium();
			if (!medium) {
				DLog("Selected medium is NULL, forcing to autonegotiation\n");
				medium = mediumTable[MEDIUM_INDEX_AUTO];
			} else {
				DLog("Selected medium index %u\n",(unsigned int)medium->getIndex());
			}
			
			selectMedium(medium);

			// FIXME - why is this here? Anything to do with link_timer in Linux?
			//timerSource->setTimeoutMS(TX_TIMEOUT);
			ret = true;
			break;
		}
			
		case kActivationLevelBSD:
		{
			if (!R1000OpenAdapter()) break;
			transmitQueue->setCapacity(kTransmitQueueCapacity);
			transmitQueue->start();
			
			ret = true;
			break;
		}
	}
	
	return ret;
}

bool RealtekR1000::decreaseActivationLevel(UInt32 level)
{
	switch (level)
	{
		case kActivationLevelKDP:
			// FIXME - why is this here? Anything to do with link_timer in Linux?
			//timerSource->cancelTimeout();
			
			if (pciDev) pciDev->close(this);
			break;
			
		case kActivationLevelBSD:
			transmitQueue->stop();
			
			transmitQueue->setCapacity(0);
			transmitQueue->flush();
			R1000CloseAdapter();
			break;
	}
	
	return true;
}

bool RealtekR1000::setActivationLevel(UInt32 level)
{
	DLog("setActivationLevel(%u)\n", (unsigned int)level);
	
    if (activationLevel == level)
		return true;

    bool success = false;
		
    for ( ; activationLevel > level; activationLevel--) {
        if (!(success = decreaseActivationLevel(activationLevel)))
            break;
    }
	
    for ( ; activationLevel < level; activationLevel++ ) {
        if (!(success = increaseActivationLevel(activationLevel+1)))
            break;
    }
	
    return success;
}

/*
 * A request from an interface client to enable the controller.
 */
IOReturn RealtekR1000::enable(IONetworkInterface *netif)
{
	DLog("enable\n");
	
	if (enabledForBSD)
		return kIOReturnSuccess;
	
	enabledForBSD = setActivationLevel(kActivationLevelBSD);
	if (enabledForBSD) {
		enabled = true;
		return kIOReturnSuccess;
	} else 
    return kIOReturnIOError;
}

/*
 * A request from an interface client to disable the controller.
 */

IOReturn RealtekR1000::disable(IONetworkInterface *netif)
{
	DLog("disable\n");
	
	enabledForBSD = false;
	
	setActivationLevel(enabledForKDP ? kActivationLevelKDP : kActivationLevelNone);
	
	enabled = false;
	return kIOReturnSuccess;
}



bool RealtekR1000::setLinkStatus(	UInt32					status,
                            const IONetworkMedium	*activeMedium,
                            UInt64					speedSt,
                            OSData					*data )
{
//  DLog("setLinkStatus speed=%lld\n", speedSt);
	return BaseClass::setLinkStatus( status, activeMedium, speedSt, data );
}/* end setLinkStatus */


/*
 * Transmits an output packet.
 * packet - an mbuf chain containing the output packet to be sent on the network.
 * param - a parameter provided by the caller.
 */

// TODO - implement checksum offload
UInt32 RealtekR1000::outputPacket(mbuf_t m, void *param)
{
//	DLog("outputPacket, length = %lu\n", mbuf_pkthdr_len(m));
#ifdef DEBUG
	if (!buffers_inited)
	{
		DLog("outputPacket: buffers not initialized!!\n");
		return kIOReturnOutputStall;
	}
#endif
	
	// Is packet larger than MTU?
	if (mbuf_pkthdr_len(m) > tx_pkt_len) 
	{
		DLog("Tx Packet size is too big, dropping\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	
	ulong buf_len = static_cast<ulong>(mbuf_pkthdr_len(m));
	
	// Allocate an entry in the Tx buffer descriptor ring
	ulong entry = OSIncrementAtomic(&cur_tx) % n_tx_desc;
	
	// Is this entry available?
	if ((OSSwapLittleToHostInt32(TxDescArray[entry].status) & DescOwn))
	{
		DLog("TX_RING_IS_FULL, stalling\n");
		return kIOReturnOutputStall;
	}
	
	// copy user packet into Tx buffer, coalescing if needed
	uchar *data_ptr = TxBufferVirtualAddress(entry);
	ulong pkt_snd_len = 0;
	mbuf_t cur_buf = m;
	
	while ((cur_buf != NULL) && (pkt_snd_len <= buf_len))
	{
		// Sanity check chunk size
		size_t cur_buf_len = mbuf_len(cur_buf);
		if (cur_buf_len > (buf_len - pkt_snd_len))
		{
			DLog("Tx Packet mbuf chain malformed, dropping\n");
			freePacket(m);
			return kIOReturnOutputDropped;
		}
		
		if (mbuf_data(cur_buf))
		{
			bcopy(mbuf_data(cur_buf), data_ptr, cur_buf_len);
			data_ptr += cur_buf_len;
			pkt_snd_len += static_cast<ulong>(cur_buf_len);
		}
		cur_buf = mbuf_next(cur_buf);
	}
	
	// final sanity check
	if (buf_len != pkt_snd_len)
	{
		DLog("Tx Packet mbuf chain missing data, dropping\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	
	// now can log the packet as in the queue
	Tx_skbuff[entry] = m;
	
	// mark the descriptor as ready to send
	if (entry == (n_tx_desc - 1))
	{
		TxDescArray[entry].status = OSSwapHostToLittleInt32((DescOwn | RingEnd | FirstFrag | LastFrag) | pkt_snd_len);
	}
	else
	{
		TxDescArray[entry].status = OSSwapHostToLittleInt32((DescOwn | FirstFrag | LastFrag) | pkt_snd_len);
	}
	
	// tell the chip there's work to do
	WriteMMIO8(TxPoll, 0x40); // Normal Priority Queue bit
	
	return kIOReturnOutputSuccess;
}

void RealtekR1000::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
	//DLog("getPacketBufferConstraints\n");
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}

IOOutputQueue *RealtekR1000::createOutputQueue()
{
	//DLog("createOutputQueue\n");
	// Slight optimization 
	// switch back if Tx/Rx conflicts cause problems
	//return IOGatedOutputQueue::withTarget(this, getWorkLoop());
	return IOBasicOutputQueue::withTarget(this);
}

/*
 * Returns a string describing the vendor of the network controller. The caller is responsible for releasing the string object returned.
 */
const OSString *RealtekR1000::newVendorString() const
{
	return OSString::withCString("Realtek");
}

/*
 * Returns a string describing the model of the network controller. The caller is responsible for releasing the string object returned.
 */
const OSString *RealtekR1000::newModelString() const
{
	return OSString::withCString(rtl_chip_info[mcfg].name);
}

/*
 * A client request to change the medium selection.
 * This method is called when a client issues a command for the controller to change its 
 * current medium selection. The implementation must call setSelectedMedium() after the change 
 * has occurred. This method call is synchronized by the workloop's gate.
 */
IOReturn RealtekR1000::selectMedium(const IONetworkMedium *medium)
{
  unsigned long physt = 0;
  if (!medium) {
    medium = mediumTable[MEDIUM_INDEX_AUTO];
  }
  
	if (!medium)
	{
//		DLog("Selected medium is NULL\n");
		return kIOReturnBadArgument;
	}
	
	DLog("selectMedium, index=%u\n", (unsigned int)medium->getIndex());
	switch (medium->getIndex())
	{
		case MEDIUM_INDEX_AUTO:
			R1000SetMedium(SPEED_1000, DUPLEX_FULL, AUTONEG_ENABLE);
			break;
		case MEDIUM_INDEX_10HD:
			R1000SetMedium(SPEED_10, DUPLEX_HALF, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_10FD:
			R1000SetMedium(SPEED_10, DUPLEX_FULL, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_100HD:
			R1000SetMedium(SPEED_100, DUPLEX_HALF, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_100FD:
			R1000SetMedium(SPEED_100, DUPLEX_FULL, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_1000HD:
			R1000SetMedium(SPEED_1000, DUPLEX_HALF, AUTONEG_DISABLE);
			break;
		case MEDIUM_INDEX_1000FD:
			R1000SetMedium(SPEED_1000, DUPLEX_FULL, AUTONEG_DISABLE);
			break;
	}
	this->setSelectedMedium(medium);
	physt = ReadMMIO8(PHYstatus);
  DLog("PHYstatus=%lx, LinkStatus=%x\n", physt, LinkStatus);
	if (physt & LinkStatus)
	{
		if (physt & _1000Mbps) {
			this->setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, 
								medium, //getSelectedMedium(),
								1000 * MBit,
								NULL);
      DLog("setLinkStatus speed=1000 :)\n");
		} else if(physt & _100Mbps) {
			this->setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, 
								medium, //getSelectedMedium(),
								100 * MBit, 
								NULL);
      DLog("setLinkStatus speed=100\n");
		} else if(physt & _10Mbps) {
			this->setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, 
								medium, //getSelectedMedium(),
								10 * MBit, 
								NULL);
      DLog("setLinkStatus speed=10 :(\n");
    }
	}
	else
	{
    DLog("setLinkStatus but no PHYStatus\n");
		this->setLinkStatus(kIONetworkLinkValid, medium, 0 * MBit, NULL); //medium <->NULL
	}
	
	return kIOReturnSuccess;
}

/*
 * Configures a newly created network interface object.
 * This method configures an interface object that was created by createInterface(). 
 * Subclasses can override this method to customize and examine the interface object that will be 
 * attached to the controller as a client.
 */

//TODO - Add configuration info about features: 
// MTU, checksum offload, magic packet, etc.

bool RealtekR1000::configureInterface(IONetworkInterface *netif)
{
	DLog("configureInterface\n");
	
    if (!BaseClass::configureInterface(netif))
		return false;
	
	// Set name here
	DLog("attaching as %s%d\n", netif->getNamePrefix(), netif->getUnitNumber());
	snprintf(bsdName, BSD_NAME_LEN, "%s%d",
			 netif->getNamePrefix(), netif->getUnitNumber());
	
    // Get the generic network statistics structure.
	IONetworkData * data = netif->getParameter( kIONetworkStatsKey );
    if ( !data || !(netStats = (IONetworkStats *) data->getBuffer()) ) {
      DLog("configureInterface: failed to get IONetworkStats buffer\n");
      return false;
    }
	
    // Get the Ethernet statistics structure.
    data = netif->getParameter( kIOEthernetStatsKey );
    if ( !data || !(etherStats = (IOEthernetStats *) data->getBuffer()) ) {
      DLog("configureInterface: failed to get IOEthernetStats buffer\n");
      return false;
    }

	// Jumbo packet capabilities
	//TODO - later!
	/*
	if (max_jumbo_frame_sz)
	{
		if (!netif->setMaxTransferUnit(max_jumbo_frame_sz))
		{
			DLog("configureInterface: setMaxTransferUnit failed\n");
			return false;
		}
		DLog("configureInterface: set jumbo frame MTU to %u\n", max_jumbo_frame_sz);
	}
	 */

    return true;
}

/*
 * Method called by IONetworkController prior to the initial getWorkLoop() call.
 */
bool RealtekR1000::createWorkLoop()
{
	//DLog("createWorkLoop\n");
	workLoop = IOWorkLoop::workLoop();
	if (workLoop == NULL)
		return false;
	workLoop->retain();
	return true;
}

IOWorkLoop *RealtekR1000::getWorkLoop() const
{
	//DLog("getWorkLoop\n");
	return workLoop;
}

/*
 * Gets the Ethernet controller's permanent station address.
 * Ethernet drivers must implement this method, by reading the address from hardware and writing 
 * it to the buffer provided. This method is called from the workloop context.
 */
IOReturn RealtekR1000::getHardwareAddress(IOEthernetAddress *addr)
{
	//DLog("getHardwareAddress\n");

	if (!board_inited)
		return kIOReturnNotReady;
	
	for (uchar i = 0; i < ETHER_ADDR_LEN ; i++)
	{
		addr->bytes[i] = ReadMMIO8(MAC0 + i);
	}
	return kIOReturnSuccess;
}

/*
 * Enables or disables promiscuous mode.
 * Called by enablePacketFilter() or disablePacketFilter() when there is a change 
 * in the activation state of the promiscuous filter identified by kIOPacketFilterPromiscuous. 
 * This method is called from the workloop context.
 */
IOReturn RealtekR1000::setPromiscuousMode(bool enableIt)
{
	DLog("setPromiscuousMode(%s)\n", (enableIt ? "true" : "false"));
	
	UInt32 rx_mode;
	UInt32 mc_filter[2];
	
	if (enableIt) {
		//Accept all multicasts
		mc_filter[0] = mc_filter[1] = 0xffffffff;
		
		rx_mode = rx_config_base | AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys |
		(ReadMMIO32(RxConfig) & rtl_chip_info[mcfg].RxConfigMask);
	} else {
		//Restoring old multicast filter
		mc_filter[0] = mc_filter0;
		mc_filter[1] = mc_filter1;
		
		rx_mode = rx_config_base | 
		AcceptBroadcast | AcceptMulticast | AcceptMyPhys | 
		(ReadMMIO32(RxConfig) & rtl_chip_info[mcfg].RxConfigMask);
	}
	
	WriteMMIO32(RxConfig, rx_mode);
	// FIXME - is this correct?
	if ((mcfg == MCFG_8168B_1) || (mcfg == MCFG_8168B_2) ||
	    (mcfg == MCFG_8101E_1) || (mcfg == MCFG_8100E_1) ||
	    (mcfg == MCFG_8100E_2)) 
	{
		WriteMMIO32(MAR0 + 0, 0xffffffff);
		WriteMMIO32(MAR0 + 4, 0xffffffff);
	}
	else
	{
		WriteMMIO32(MAR0 + 0, mc_filter[0]);
		WriteMMIO32(MAR0 + 4, mc_filter[1]);
	}
	
	return kIOReturnSuccess;
}

/*
 * Enables or disables multicast mode.
 * Called by enablePacketFilter() or disablePacketFilter() when there is a 
 * change in the activation state of the multicast filter identified by kIOPacketFilterMulticast. 
 * This method is called from the workloop context.
 */
IOReturn RealtekR1000::setMulticastMode(bool enableIt)
{
//	DLog("setMulticastMode(%s)\n", (enableIt ? "true" : "false"));
	
	UInt32 rx_mode;
	if (enableIt) {
		rx_mode = rx_config_base | 
		AcceptBroadcast | AcceptMulticast | AcceptMyPhys | 
		(ReadMMIO32(RxConfig) & rtl_chip_info[mcfg].RxConfigMask);
	} else {
		rx_mode = (rx_config_base |
				   AcceptBroadcast | AcceptMyPhys |
				   (ReadMMIO32(RxConfig) & rtl_chip_info[mcfg].RxConfigMask)) &
				  ~AcceptMulticast;	
	}
	
	WriteMMIO32(RxConfig, rx_mode);
	return kIOReturnSuccess;
}

/* Sets the list of multicast addresses a multicast filter should use to match 
 * against the destination address of an incoming frame.
 * This method sets the list of multicast addresses that the multicast filter should use 
 * to match against the destination address of an incoming frame. The frame should be 
 * accepted when a match occurs. Called when the multicast group membership of an interface object is changed. 
 * Drivers that support kIOPacketFilterMulticast should override this method and update the 
 * hardware multicast filter using the list of Ethernet addresses provided. Perfect multicast filtering 
 * is preferred if supported by the hardware, in order to reduce the number of unwanted packets received. 
 * If the number of multicast addresses in the list exceeds what the hardware is capable of supporting, 
 * or if perfect filtering is not supported, then ideally the hardware should be programmed to perform 
 * imperfect filtering, through some form of hash filtering mechanism. Only as a last resort should the driver 
 * enable reception of all multicast packets to satisfy this request. This method is called from the workloop 
 * context, and only if the driver reports kIOPacketFilterMulticast support in getPacketFilters().
 */
// Copied logic from rtl8168_set_rx_mode
// FIXME - verify this is correct for all chips
// rtl8168_set_rx_mode now has a slightly different last bit,
// but rtl8101_set_rx_mode is (as far as I can tell) functionally
// identical to this.
IOReturn RealtekR1000::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
//	DLog("setMulticastList(count=%u)\n", (unsigned int)count);
	//DLog("Forcing method!\n");
	//return kIOReturnSuccess;

	ulong mc_filter[2];
	
	bool enableMulticast = true;
	
	if (count > multicast_filter_limit) {
		DLog("Break multicast filter limit, accept all!\n");
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else {
		mc_filter[1] = mc_filter[0] = 0;
		for ( UInt32 i = 0; i < count; i++, addrs++ ) {
			int bit = ether_crc(6, reinterpret_cast<uchar *>(addrs)) >> 26;
			mc_filter[bit >> 5] |= 1 << (bit & 31);
		}
		enableMulticast = (count != 0);
	}
	
	for (int j = 0; j < 2; j++) {
		u32 mask = 0x000000ff;
		u32 tmp1 = 0;
		u32 tmp2 = 0;
		int x = 0;
		int y = 0;
		
		for (int k = 0; k < 4; k++) {
			tmp1 = mc_filter[j] & mask;
			x = 32 - (8 + 16 * k);
			y = x - 2 * x;
			
			if (x > 0)
				tmp2 = tmp2 | (tmp1 << x);
			else
				tmp2 = tmp2 | (tmp1 >> y);
			
			mask = mask << 8;
		}
		mc_filter[j] = tmp2;
	}
	
	setMulticastMode(enableMulticast);
	WriteMMIO32(MAR0 + 0, mc_filter[1]);
	WriteMMIO32(MAR0 + 4, mc_filter[0]);
	
	return kIOReturnSuccess;
}

/*
 * Debugger polled-mode transmit handler.
 * This method must be implemented by a driver that supports kernel debugging.
 * pkt - pointer to a transmit buffer containing the packet to be sent on the network.
 * pktSize - the size of the transmit buffer in bytes.
 */
// TODO - Implement!
void RealtekR1000::sendPacket(void * pkt, UInt32 pkt_len)
{
	DLog("sendPacket(void * pkt, UInt32 pkt_len)\n");
}

/*
 * Debugger polled-mode receive handler.
 * This method must be implemented by a driver that supports kernel debugging.
 * pkt - address of a receive buffer where the received packet should be stored. This buffer has room for 1518 bytes.
 * pktSize - address where the number of bytes received must be recorded. Set this to zero if no packets were received during the timeout interval.
 * timeout - the maximum amount of time in milliseconds to poll for a packet to arrive before this method must return.
 */
// TODO - Implement!
void RealtekR1000::receivePacket(void *pkt, UInt32 *pkt_len, UInt32 timeout)
{
	DLog("receivePacket(void *pkt, UInt32 *pkt_len, UInt32 timeout)\n");
}

/*
 * Implements the framework for a generic network controller.
 */
IOReturn RealtekR1000::registerWithPolicyMaker(IOService *policyMaker)
{
	DLog("registerWithPolicyMaker\n");
    return policyMaker->registerPowerDriver(this,
											powerStateArray,
											kR1000PowerStateCount);
}

//from Mieze
IOReturn RealtekR1000::setWakeOnMagicPacket(bool active)
{
  IOReturn result = kIOReturnUnsupported;
  
  DLog("setWakeOnMagicPacket() to %s active\n", active?"":"not");
  if (wolCapable) {
    wol_enabled = active ? WOL_ENABLED : WOL_DISABLED;
    wolActive = active;
    result = kIOReturnSuccess;
  }
  DLog("WOL is %s supported\n", wolCapable?"":"not");
  return result;
}



// N.B. The OS will only call this routine after disabling or before enabling the controller.

// FIXME - Add error returns
IOReturn RealtekR1000::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
	DLog("setPowerState(%lu)\n", powerStateOrdinal);
	
	// are we already in the correct state?
	if (powerState == powerStateOrdinal)
		return IOPMAckImplied; 
	
  if (kR1000PowerStateOff == powerStateOrdinal) {
		R1000Suspend();
	} else {
		R1000Resume();
	}
	
	powerState = powerStateOrdinal;
	return IOPMAckImplied;
}

// FIXME - Add error return
void RealtekR1000::R1000Suspend()
{
	DLog("R1000Suspend\n");
	
	// report that the link has gone down
	this->setLinkStatus(kIONetworkLinkValid);
	
	// not on RTL8169
	R1000DSM(DSM_NIC_GOTO_D3);
	
	if (MCFG_IS_8169(mcfg))
		RTL8169PowerDownPHY();
	else
		R1000PowerDownPLL();
	
	R1000ASICDown();
	
	// only on RTL8168
	if (MCFG_IS_8168(mcfg))
		RTL8168SleepRxEnable();
	
	// Update RxMissed stats
	// only on RTL8169 ?
	
	// FIXME - Enable wake-on-LAN here if user wants it
	
	return;
}

// FIXME - Add error return
void RealtekR1000::R1000Resume()
{
	DLog("R1000Resume\n");
	
	// FIXME - disable wake-on-LAN here

	// Tell chip it can use receive buffers
	R1000InitRxDescCmds(true);
	
	// not on RTL8169
	R1000DSM(DSM_NIC_RESUME_D3);
	
	// Schedule a deferred reset
	workLoop->runAction(OSMemberFunctionCast(IOWorkLoop::Action, 
											 this, 
											 &RealtekR1000::R1000ResetTask),
						this);
	
	if (MCFG_IS_8169(mcfg))
		RTL8169PowerUpPHY();
	else
		R1000PowerUpPLL();
	
	// report that the link has come up, if it has
	if (ReadMMIO8(PHYstatus) & LinkStatus) {
		this->setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive,
							getSelectedMedium());
    DLog("link active again\n");
	} else
		this->setLinkStatus(kIONetworkLinkValid);
	
	return;
}

// Meant to be called as an IOWorkLoop Action.
IOReturn RealtekR1000::R1000ResetTask()
{
	DLog("R1000ResetTask\n");
	
	// process any pending Rx packets
	R1000RxInterrupt(ReadMMIO16(IntrStatus));
	
	// Clear the Tx queue
	R1000TxClear();

	if (~(RxDescArray[cur_rx].status & DescOwn)) {
		// Rx queue is clear, restart hardware
		R1000InitRingIndices();
		R1000HwStart();
		return kIOReturnSuccess;
	}
	
	// tail recurse if we're not done
	// N.B. this may not be the right intent -
	// we may want to wait for more stuff to happen in the work loop
	return R1000ResetTask();
}

void RealtekR1000::R1000NicReset()
{
	if (MCFG_IS_8100(mcfg))
		RTL8100NicReset();
	else if (MCFG_IS_8168(mcfg))
		RTL8168NicReset();
	else if (MCFG_IS_8169(mcfg))
		RTL8169NicReset();
}

void RealtekR1000::R1000DSM(int dev_state)
{
	if (MCFG_IS_8100(mcfg))
		RTL8100DSM(dev_state);
	else if (MCFG_IS_8168(mcfg))
		RTL8168DSM(dev_state);
	return;
}

void RealtekR1000::R1000PowerDownPLL()
{
	if (MCFG_IS_8100(mcfg))
		RTL8100PowerDownPLL();
	else if (MCFG_IS_8168(mcfg))
		RTL8168PowerDownPLL();
	return;
}

void RealtekR1000::R1000PowerUpPLL()
{
	if (MCFG_IS_8100(mcfg))
		RTL8100PowerUpPLL();
	else if (MCFG_IS_8168(mcfg))
		RTL8168PowerUpPLL();
	return;	
}

void RealtekR1000::R1000PowerDownPHY()
{
	if (MCFG_IS_8100(mcfg))
		RTL8100PowerDownPHY();
	else if (MCFG_IS_8168(mcfg))
		RTL8168PowerDownPHY();
	else if (MCFG_IS_8169(mcfg))
		RTL8169PowerDownPHY();
	return;
}

void RealtekR1000::R1000PowerUpPHY()
{
	if (MCFG_IS_8100(mcfg))
		RTL8100PowerUpPHY();
	else if (MCFG_IS_8168(mcfg))
		RTL8168PowerUpPHY();
	else if (MCFG_IS_8169(mcfg))
		RTL8169PowerUpPHY();
	return;	
}

void RealtekR1000::R1000ASICDown()
{
	R1000NicReset();
	R1000IRQMaskAndAck();
}

void RealtekR1000::R1000IRQMaskAndAck()
{
	WriteMMIO16(IntrMask, 0x0000);
	WriteMMIO16(IntrStatus, 0xFFFF);
}

// TODO - Add support for chips that can do this
/*
enum {
  kChecksumFamilyTCPIP         = 0x00000001,
  kChecksumIP                  = 0x0001,
  kChecksumTCP                 = 0x0002,
  kChecksumUDP                 = 0x0004,
  kChecksumTCPNoPseudoHeader   = 0x0100,
  kChecksumUDPNoPseudoHeader   = 0x0200,
  kChecksumTCPSum16            = 0x1000
};
*/

IOReturn RealtekR1000::getChecksumSupport(UInt32 * checksumMask,
										  UInt32   checksumFamily,
										  bool     isOutput)
{
#ifdef R1000_CHECKSUM_OFFLOAD
	// FIXME -- implement!!
  //canOffload &= ~kChecksumIP;
  *checksumMask = canOffload;
	return kIOReturnSuccess;
#else
	return kIOReturnUnsupported;
#endif	
}

/* Temporarily commented out until I can get jumbo frames working
IOReturn RealtekR1000::getMaxPacketSize(UInt32 *maxSize) const
{
	if (!max_jumbo_frame_sz)
	return kIOReturnUnsupported;

	*maxSize = max_jumbo_frame_sz + ETHER_HDR_LEN + ETHER_CRC_LEN;
	return kIOReturnSuccess;
}
*/

/* Temporarily commented out until I can get jumbo frames working
IOReturn RealtekR1000::setMaxPacketSize(UInt32 maxSize)
{
 DLog("setMaxPacketSize(UInt32 maxSize) Packet size %d\n", maxSize);
 
 maxSize = maxSize - ETHER_HDR_LEN - ETHER_CRC_LEN;
 if (maxSize > max_jumbo_frame_sz)
 {
 DLog("New MTU is too high!\n");
 return kIOReturnUnsupported;
 }
 
 curr_mtu_size = maxSize;
 tx_pkt_len = maxSize + ETHER_HDR_LEN;
 rx_pkt_len = maxSize + ETHER_HDR_LEN;
 hw_rx_pkt_len = rx_pkt_len + 8;
 
 WriteMMIO8(Cfg9346, Cfg9346_Unlock);
 WriteMMIO16(RxMaxSize, static_cast<unsigned short>(hw_rx_pkt_len));
 WriteMMIO8(Cfg9346, Cfg9346_Lock);
 
 R1000CloseAdapter();
 R1000OpenAdapter();
 
 return kIOReturnSuccess;
}
 */


/*
 *  Private methods
 */

//=================================================================
//	PHYAR
//	bit		Symbol
//	31		Flag
//	30-21	reserved
//	20-16	5-bit GMII/MII register address
//	15-0	16-bit GMII/MII register data
//=================================================================
// Equivalent to Linux mdio_write()
void RealtekR1000::WriteGMII16(int RegAddr, u16 value)
{
  u16 cur_page = OCP_STD_PHY_BASE_PAGE;

  if (RegAddr == 0x1F) {
    cur_page = value;
  }

	if (mcfg == MCFG_8168DP_1) {
		WriteMMIO32(OCPDR, OCPDR_Write |
                (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift |
                (value & OCPDR_Data_Mask));
		WriteMMIO32(OCPAR, OCPAR_GPHY_Write);
		WriteMMIO32(EPHY_RXER_NUM, 0);
    
		for (int i = 0; i < 100; i++) {
			IODelay(1000);
			if (!(ReadMMIO32(OCPAR) & OCPAR_Flag))
				break;
		}
	} else if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 || mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_24 || mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B || mcfg == CFG_METHOD_27) {
		u32 data32;
		u16 ocp_addr;

//		if (RegAddr == 0x1F) {
//			cur_page = value ? value : OCP_STD_PHY_BASE_PAGE;
//			return;
//		}
		ocp_addr = map_phy_ocp_addr(cur_page, RegAddr);
    
		data32 = ocp_addr/2;
		data32 <<= OCPR_Addr_Reg_shift;
		data32 |= OCPR_Write | value;
    
		WriteMMIO32(PHYOCP, data32);
		for (int i = 0; i < 10; i++) {
			IODelay(100);
      
			if (!(ReadMMIO32(PHYOCP) & OCPR_Flag))
				break;
		}
	} else {
		if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
			WriteMMIO32(0xD0, ReadMMIO32(0xD0) & ~0x00020000);
    
    WriteMMIO32(PHYAR,
                PHYAR_Write |
                (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift |
                (value & PHYAR_Data_Mask));
    
    for (int i = 0; i < 20 ; i++) // 20 for RTL8169, 10 for others
    {
      IODelay(100);
      // Check if the write has finished
      if (!(ReadMMIO32(PHYAR) & PHYAR_Flag))
        break;
    }
    if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
			WriteMMIO32(0xD0, ReadMMIO32(0xD0) | 0x00020000);
	}

}

// Equivalent to Linux mdio_read()
u16 RealtekR1000::ReadGMII16(int RegAddr)
{
	u16 value = 0xFFFF;
	if (mcfg==MCFG_8168DP_1) {
		WriteMMIO32(OCPDR, OCPDR_Read |
                (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift);
		WriteMMIO32(OCPAR, OCPAR_GPHY_Write);
		WriteMMIO32(EPHY_RXER_NUM, 0);
    
		for (int i = 0; i < 100; i++) {
			IODelay(1000);
			if (!(ReadMMIO32(OCPAR) & OCPAR_Flag))
				break;
		}
    
		IODelay(1000);
		WriteMMIO32(OCPAR, OCPAR_GPHY_Read);
		WriteMMIO32(EPHY_RXER_NUM, 0);
    
		for (int i = 0; i < 100; i++) {
			IODelay(1000);
			if (ReadMMIO32(OCPAR) & OCPAR_Flag)
				break;
		}
    
		value = ReadMMIO32(OCPDR) & OCPDR_Data_Mask;
	} else if (mcfg==CFG_METHOD_21 || mcfg==CFG_METHOD_22 || mcfg==CFG_METHOD_23 || mcfg==CFG_METHOD_24 || mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B || mcfg == CFG_METHOD_27) {
		u32 data32;
		u16 ocp_addr;
    u16 cur_page = OCP_STD_PHY_BASE_PAGE;
    
		ocp_addr = map_phy_ocp_addr(cur_page, RegAddr);
    
		data32 = ocp_addr/2;
		data32 <<= OCPR_Addr_Reg_shift;
    
		WriteMMIO32(PHYOCP, data32);
		for (int i = 0; i < 10; i++) {
			IODelay(100);
      
			if (ReadMMIO32(PHYOCP) & OCPR_Flag)
				break;
		}
		value = ReadMMIO32(PHYOCP) & OCPDR_Data_Mask;
	} else {
		if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
			WriteMMIO32(0xD0, ReadMMIO32(0xD0) & ~0x00020000);
    
    
    WriteMMIO32(PHYAR, PHYAR_Read | (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift);
    
    for (int i = 0; i < 20 ; i++) // 20 for RTL8169, 10 for others
    {
      IODelay(100);
      
      // Check if the read has finished
      if (ReadMMIO32(PHYAR) & PHYAR_Flag)
      {
        value = static_cast<u16>(ReadMMIO32(PHYAR) & PHYAR_Data_Mask);
        break;
      }
    }
    
    if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
      WriteMMIO32(0xD0, ReadMMIO32(0xD0) | 0x00020000);
  }

	return value;
}

// Equivalent to RTL8xxx_ephy_write
void RealtekR1000::WriteEPHY16(int RegAddr, 
							  u16 value)
{
	WriteMMIO32(EPHYAR, 
				EPHYAR_Write | 
				(RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift | 
				(value & EPHYAR_Data_Mask));
	
	for (int i = 0; i < 10; i++) 
	{
		IODelay(100);
		
		/* Check if the chip has completed EPHY write */
		if (!(ReadMMIO32(EPHYAR) & EPHYAR_Flag)) 
			break;
	}
	
	IODelay(20);
	return;
}

// Equivalent to RTL8xxx_ephy_read
u16 RealtekR1000::ReadEPHY16(int RegAddr)
{
	u16 value = 0xffff;
	
	WriteMMIO32(EPHYAR, 
				EPHYAR_Read | (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift);
	
	for (int i = 0; i < 10; i++) 
	{
		IODelay(100);
		
		/* Check if the chip has completed EPHY read */
		if (ReadMMIO32(EPHYAR) & EPHYAR_Flag)
		{
			value = static_cast<u16>(ReadMMIO32(EPHYAR) & EPHYAR_Data_Mask);
			break;
		}
	}
	
	IODelay(20);
	return value;
}

u32 RealtekR1000::rtl8168_csi_other_fun_read(
                           u8 multi_fun_sel_bit,
                           u32 addr)
{
  u32 cmd;
  int i;
  u32 value = 0;

  cmd = CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);

  if (mcfg != MCFG_8411_1 && mcfg != CFG_METHOD_23
      && mcfg != MCFG_8411B && mcfg != CFG_METHOD_27) {
    multi_fun_sel_bit = 0;
  }

  if( multi_fun_sel_bit > 7 ) {
    return 0xffffffff;
  }

  cmd |= multi_fun_sel_bit << 16;

  WriteMMIO32(CSIAR, cmd);

  for (i = 0; i < 10; i++) {
    IODelay(100);

    /* Check if the RTL8168 has completed CSI read */
    if (ReadMMIO32(CSIAR) & CSIAR_Flag) {
      value = (u32)ReadMMIO32(CSIDR);
      break;
    }
  }

  IODelay(20);

  return value;
}

void RealtekR1000::rtl8168_csi_other_fun_write(
                            u8 multi_fun_sel_bit,
                            u32 addr,
                            u32 value)
{
  u32 cmd;
  int i;

  WriteMMIO32(CSIDR, value);
  cmd = CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);
  if (mcfg != MCFG_8411_1 && mcfg != CFG_METHOD_23
      && mcfg != MCFG_8411B && mcfg != CFG_METHOD_27) {
    multi_fun_sel_bit = 0;
  }

  if( multi_fun_sel_bit > 7 ) {
    return;
  }

  cmd |= multi_fun_sel_bit << 16;

  WriteMMIO32(CSIAR, cmd);

  for (i = 0; i < 10; i++) {
    IODelay(100);

    /* Check if the RTL8168 has completed CSI write */
    if (!(ReadMMIO32(CSIAR) & CSIAR_Flag))
      break;
  }

  IODelay(20);
}


// Equivalent to RTL8xxx_csi_write
void RealtekR1000::WriteCSI32(int addr,
							  int value)
{
/*	WriteMMIO32(CSIDR, value);
	
	u32 cmd = CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift |
		(addr & CSIAR_Addr_Mask);
	if (mcfg == MCFG_8402_1 || mcfg == MCFG_8411_1) {
		cmd |= 0x00020000;
	}
	WriteMMIO32(CSIAR, cmd);

	for (int i = 0; i < 10; i++)
	{
		IODelay(100);
		
		// Check if the chip has completed CSI write
		if (!(ReadMMIO32(CSIAR) & CSIAR_Flag)) 
			break;
	}
	IODelay(20); */

  u8 multi_fun_sel_bit;

  if (mcfg == MCFG_8411_1)
    multi_fun_sel_bit = 2;
  else if (mcfg == MCFG_8411B)
    multi_fun_sel_bit = 1;
  else
    multi_fun_sel_bit = 0;

  rtl8168_csi_other_fun_write(multi_fun_sel_bit, addr, value);

	return;
}

// Equivalent to RTL8xxx_csi_read
int RealtekR1000::ReadCSI32(int addr)
{
/*	int value = -1;
	
	u32 cmd = CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift |
		(addr & CSIAR_Addr_Mask);

	if (mcfg == MCFG_8402_1 || mcfg == MCFG_8411_1)
		cmd |= 0x00020000;

	WriteMMIO32(CSIAR, cmd);
	
	for (int i = 0; i < 10; i++)
	{
		IODelay(100);
		
		// Check if the chip has completed CSI read
		if (ReadMMIO32(CSIAR) & CSIAR_Flag)
		{
			value = static_cast<int>(ReadMMIO32(CSIDR));
			break;
		}
	}
	IODelay(20);
  return value; */
  u8 multi_fun_sel_bit;

  if (mcfg == MCFG_8411_1)
    multi_fun_sel_bit = 2;
  else if (mcfg == MCFG_8411B)
    multi_fun_sel_bit = 1;
  else
    multi_fun_sel_bit = 0;

  return rtl8168_csi_other_fun_read(multi_fun_sel_bit, addr);
}

// Equivalent to rtlbxxx_eri_write
int RealtekR1000::WriteERI(int addr,
								  int len,
								  int value,
								  int type)
{

	int i, val_shift, shift = 0;
	u32 value1 = 0, mask;

	if (len > 4 || len <= 0)
		return -1;
	
	while(len > 0)
	{
		val_shift = addr % ERIAR_Addr_Align;
		addr = addr & ~0x3;
		
		if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		
		value1 = ReadERI(addr, 4, type) & ~mask;
		value1 |= (((value << val_shift * 8) >> shift * 8));
		
		WriteMMIO32(ERIDR, value1);
		WriteMMIO32(ERIAR, 
					ERIAR_Write |
					type << ERIAR_Type_shift |
					ERIAR_ByteEn << ERIAR_ByteEn_shift |
					addr);
		
		for (i = 0; i < 10; i++)
		{
			IODelay(100);
			
			/* Check if the chip has completed ERI write */
			if (!(ReadMMIO32(ERIAR) & ERIAR_Flag)) 
				break;
		}
		
		if (len <= 4 - val_shift)
			len = 0;
		else
		{
			len -= (4 - val_shift);
			shift = 4 - val_shift;
			addr += 4;
		}
	}
	
	return 0;
}

// Equivalent to rtlbxxx_eri_read
int RealtekR1000::ReadERI(int addr,
								 int len,
								 int type)
{
	int i, val_shift, shift = 0;
	u32 value1 = 0, value2 = 0, mask;
	
	if (len > 4 || len <= 0)
		return -1;
	
	while (len > 0) {
		val_shift = addr % ERIAR_Addr_Align;
		addr = addr & ~0x3;
		
		WriteMMIO32(ERIAR, 
					ERIAR_Read |
					type << ERIAR_Type_shift |
					ERIAR_ByteEn << ERIAR_ByteEn_shift |
					addr);
		
		for (i = 0; i < 10; i++) {
			IODelay(100);
			
			/* Check if the chip has completed ERI read */
			if (ReadMMIO32(ERIAR) & ERIAR_Flag) 
				break;
		}
		
		if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		
		value1 = ReadMMIO32(ERIDR) & mask;
		value2 |= (value1 >> val_shift * 8) << shift * 8;
		
		if (len <= 4 - val_shift)
			len = 0;
		else
		{
			len -= (4 - val_shift);
			shift = 4 - val_shift;
			addr += 4;
		}
	}
	
	return value2;
}

// Cribbed from Linux rtl*_get_mac_version()
void RealtekR1000::R1000GetMacVersion()
{
	u32 val32 = ReadMMIO32(TxConfig);	
	u32 reg = val32 & 0xFC800000; // was 0x7C800000
	u32 ICVerID = val32 & 0x00700000;
  IOLog("Found chip reg=%08x VerID=%x\n", val32, ICVerID);
	switch (reg) 
	{
		case 0x00000000:
			mcfg = MCFG_8169_1;
			break;
			
		case 0x00800000:
			mcfg = MCFG_8169S_1;
			break;
			
		case 0x04000000:
			mcfg = MCFG_8169S_2;
			break;
			
		case 0x10000000:
			mcfg = MCFG_8169SB_1;
			break;
			
		case 0x18000000:
			mcfg = MCFG_8169SC_1;
			break;

		case 0x24000000:
			mcfg = MCFG_8401_1;
			break;
            
		case 0x24800000:
		case 0x34800000:
			if (ICVerID == 0x00100000)
				mcfg = MCFG_8102E_1;
			else if (ICVerID == 0x00200000)
				mcfg = MCFG_8102E_2;
			else if (ICVerID == 0x00400000)
				mcfg = MCFG_8103E_1;
			else if (ICVerID == 0x00500000)
				mcfg = MCFG_8103E_2;
			else if (ICVerID == 0x00600000)
				mcfg = MCFG_8103E_3;
			else
				mcfg = MCFG_8103E_3;
			break;

		case 0x28000000:
			if(ICVerID == 0x00100000) {
				mcfg = MCFG_8168D_1;
			} else if(ICVerID == 0x00300000) {
				mcfg = MCFG_8168D_2;
			} else {
				mcfg = MCFG_8168D_2;
			}
			break;
			
		case 0x28800000:
			if (ICVerID == 0x00000000) { //was 0x00100000 but linux 0
				mcfg = MCFG_8168DP_1;
			} else if (ICVerID == 0x00200000) {
				mcfg = MCFG_8168DP_2;
				WriteMMIO32(0xD0, ReadMMIO32(0xD0) | 0x00020000);
			} else {
				mcfg = MCFG_8168DP_3;
			}
			break;

		case 0x2C000000:
			// Note: Both R8101 and R8168 use this value.
			// Because the R8101 does not specify an ICVerID to check, it will
			// be the terminal case
			if (ICVerID == 0x00100000)
				mcfg = MCFG_8168E_1;
			else if (ICVerID == 0x00200000)
				mcfg = MCFG_8168E_2;
			else
				mcfg = MCFG_8105E_1;
			break;

		case 0x2C800000:
			if (ICVerID == 0x00000000)
				mcfg = MCFG_8168E_VL_1;
			else if (ICVerID == 0x00100000)
				mcfg = MCFG_8168E_VL_2;
			break;

		case 0x30000000:
			mcfg = MCFG_8168B_1;
			break;
			
		case 0x3080000:
			mcfg = MCFG_8100E_1;
			break;
			
		case 0x34000000:
			if (ICVerID == 0x00000000)
				mcfg = MCFG_8101E_1;
			else if (ICVerID == 0x00200000)
				mcfg = MCFG_8101E_2;
			else if (ICVerID == 0x00300000)
				mcfg = MCFG_8101E_3;
			else
				mcfg = MCFG_8101E_3;
			break;
			
    case 0xB8000000:
		case 0x38000000:
			if(ICVerID == 0x00000000) {
				mcfg = MCFG_8168B_2;
			} else if(ICVerID == 0x00500000) {
				mcfg = MCFG_8168B_3;
			} else {
				mcfg = MCFG_8168B_3;
			}
			break;
			
		case 0x38800000:
			mcfg = MCFG_8100E_2;
			break;
			
		case 0x3C000000:
			if(ICVerID == 0x00000000) {
				mcfg = MCFG_8168C_1;
			} else if(ICVerID == 0x00200000) {
				mcfg = MCFG_8168C_2;
			} else if(ICVerID == 0x00400000) {
				mcfg = MCFG_8168C_3;
			} else {
				mcfg = MCFG_8168C_3;
			}
			break;
			
		case 0x3C800000:
			if (ICVerID == 0x00100000){
				mcfg = MCFG_8168CP_1;
			} else if (ICVerID == 0x00300000){
				mcfg = MCFG_8168CP_2;
			} else {
				mcfg = MCFG_8168CP_2;
			}
			break;

		case 0x40800000:
			if (ICVerID == 0x00100000)
				mcfg = MCFG_8105E_2;
			else if (ICVerID == 0x00200000)
				mcfg = MCFG_8105E_3;
			else if (ICVerID == 0x00300000 || ICVerID == 0x00400000)
				mcfg = MCFG_8105E_4;
			break;

		case 0x44000000:
			mcfg = MCFG_8402_1;
			break;
      
    case 0x44800000:
      if (ICVerID == 0x00000000)
        mcfg = MCFG_8106E_1;
      else if (ICVerID == 0x00100000)
        mcfg = MCFG_8106E_2;
      break;
      

		case 0x48000000:
			if (ICVerID == 0x00000000)
				mcfg = MCFG_8168F_1;
			else if (ICVerID == 0x00100000)
				mcfg = MCFG_8168F_2;
			break;

		case 0x48800000:
			mcfg = MCFG_8411_1;
			break;

    case 0x4C000000:
      if (ICVerID == 0x00000000)
        mcfg = CFG_METHOD_21;
      else if (ICVerID == 0x00100000)
        mcfg = CFG_METHOD_22;
      break;

    case 0x50000000:
      if (ICVerID == 0x00000000) {
        mcfg = CFG_METHOD_23;
      } else if (ICVerID == 0x00100000) {
        mcfg = CFG_METHOD_27;
      } else {
        mcfg = CFG_METHOD_27;
      }
      efuse = EFUSE_SUPPORT;
      break;
      
    case 0x50800000:
      if (ICVerID == 0x00000000) {
        mcfg = CFG_METHOD_24;
      } else if (ICVerID == 0x00100000) {
        mcfg = MCFG_8106EUS;
      } else {
        mcfg = CFG_METHOD_25;
      }
      efuse = EFUSE_SUPPORT;
      break;

    case 0x5C800000:
      mcfg = MCFG_8411B;
      efuse = EFUSE_SUPPORT;
      break;


		case 0x98000000:
			mcfg = MCFG_8169SC_2;
			break;
			
		default:
			mcfg = -1;
			break;
	}
	
	if (mcfg < 0)
	{
		DLog("Unknown device type, reg = %#08x\n", reg);
	}
	else
	{
#ifdef DEBUG
		// Development sanity checks
		if (rtl_chip_info[mcfg].mcfg != mcfg)
		{
			panic("Mismatch in config data at mcfg %d", mcfg);
		}
#endif
		DLog("Realtek %s (mcfg %d)\n", rtl_chip_info[mcfg].name, mcfg);
		// Get configuration data from table
		n_rx_desc = n_tx_desc = rtl_chip_info[mcfg].max_desc;
		rx_config_base = rtl_chip_info[mcfg].RCR_Cfg;
		rx_config_mask = rtl_chip_info[mcfg].RxConfigMask;
		max_jumbo_frame_sz = rtl_chip_info[mcfg].jumbo_frame_sz;
		efuse = rtl_chip_info[mcfg].efuse;
	}
}

bool RealtekR1000::R1000InitBoard()
{
  UInt16 pmCap;
  UInt8 pmCapOffset;
//  UInt8 pcieCapOffset;
  
	DLog("R1000InitBoard @ PCI %#02x,%#02x\n", pciDev->getBusNumber(), pciDev->getDeviceNumber());
	
	// enable PCI (PCIe) bus
	pciDev->setBusMasterEnable(true);
	pciDev->setIOEnable(true);
	pciDev->setMemoryEnable(true);
  
  //from Mieze
  /* Setup power management. */
  if (pciDev->findPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
    pmCap = pciDev->configRead16(pmCapOffset + kIOPCIPMCapability);
    DLog("Ethernet [R1000]: PCI power management capabilities: 0x%x.\n", pmCap);
    
    if (pmCap & kPCIPMCPMESupportFromD3Cold) {
      wolCapable = true;
      DLog("Ethernet [R1000]: PME# from D3 (cold) supported.\n");
    }
  } else {
    IOLog("Ethernet [RealtekRTL8111]: PCI power management unsupported.\n");
  }
  

	// enable PCI bus power management
	pciDev->enablePCIPowerManagement();
	
	IOSleep(10);
	
	pioBase = pciDev->configRead16(kIOPCIConfigBaseAddress0) & 0xFFFC;
	
	// set more descriptive name
	snprintf(bsdName, BSD_NAME_LEN, "RTL8xxx@%#04x", pioBase);
	
	// Set up memory mapping for device registers
	mmioBase = pciDev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	if (mmioBase) 
	{
		forcedPio = false;
		//DLog("Mapped at virt addr %#8lx, phys addr %#8lx, length %#x\n", mmioBase->getVirtualAddress(), mmioBase->getPhysicalAddress(), mmioBase->getLength());
	}
	else
	{
		pciDev->setMemoryEnable(false);
		forcedPio = true;
		DLog("**** Couldn't set up memory mapped io, forcing port IO ****\n");
	}
	
	// Need common init method - complicated by fact we don't yet know which chip it is!
	// RTL8101, RTL8169 drivers write IRQ mask and resets the chip before IDing which chip
	// RTL8168 driver IDs chip before reset, BUT:
	// RTL8168 docs say must set up C+Command and Command registers before other registers are configured
	
	aspm = 0;
	
	// Disable interrupts, clear any pending IRQ status
	WriteMMIO16(IntrMask, 0x0000);
	WriteMMIO16(IntrStatus, 0xFFFF);
	
	// Soft reset the chip.
	WriteMMIO8(ChipCmd, CmdReset);
	
	// Check that the chip has finished the reset.
	for (int i = 1000; i > 0; i--)
	{
		if ((ReadMMIO8(ChipCmd) & CmdReset) == 0)
			break;
		IODelay(100);
	}
	
	// identify chip
	R1000GetMacVersion();
	if (mcfg < 0)
	{
		DLog("R1000InitBoard: **** Ethernet chip unrecognized ****\n");
		return false;
	}
	IOLog("identified as %s\n", rtl_chip_info[mcfg].name);
	
	// Enable power management
	WriteMMIO8(Cfg9346, Cfg9346_Unlock);
	WriteMMIO8(Config1, ReadMMIO8(Config1) | PMEnable);
	WriteMMIO8(Config5, ReadMMIO8(Config5) & PMEStatus);
	WriteMMIO8(Cfg9346, Cfg9346_Lock);
	
	board_inited = true;
	return true;
}

bool RealtekR1000::R1000ProbeAndStartBoard()
{
	if (!R1000InitBoard()) return false;
	
	// Default to std (non-jumbo) packet size
	curr_mtu_size = DEFAULT_MTU;
	tx_pkt_len = DEFAULT_MTU + ETHER_HDR_LEN;
	rx_pkt_len = DEFAULT_MTU + ETHER_HDR_LEN;
	hw_rx_pkt_len = rx_pkt_len + 8;
	
	//Config PHY
	R1000HwPhyConfig();
	
	if (MCFG_IS_8100(mcfg))
	{
		pciDev->configWrite8(kIOPCIConfigLatencyTimer, 0x40);
	}
	else if (MCFG_IS_8168(mcfg))
	{
		pciDev->configWrite8(kIOPCIConfigLatencyTimer, 0x40);
	}
	else if (MCFG_IS_8169(mcfg))
	{
		DLog("Set MAC Reg C+CR Offset 0x82h = 0x01h\n");
		WriteMMIO8(0x82, 0x01);
		
		if ((mcfg >= MCFG_8169_1) && (mcfg < MCFG_8169S_2))
		{
			DLog("Set PCI Latency=0x40\n");
			pciDev->configWrite8(kIOPCIConfigLatencyTimer, 0x40);
		}
		
		if (mcfg == MCFG_8169S_1)
		{
			DLog("Set MAC Reg C+CR Offset 0x82h = 0x01h\n");
			WriteMMIO8(0x82, 0x01);
			DLog("Set PHY Reg 0x0bh = 0x00h\n");
			WriteGMII16(0x0b, 0x0000);	//w 0x0b 15 0 0
		}
	}
	
	int speed_opt = SPEED_100;
	int duplex_opt = DUPLEX_FULL;
	int autoneg_opt = AUTONEG_ENABLE;
	int val = 0;
	
	// FIXME -- WTF does this do?!
	// if TBI is not endbled
	if (!(ReadMMIO8(PHYstatus) & TBI_Enable))
	{
		val = ReadGMII16(PHY_AUTO_NEGO_REG);
		val |= PHY_Cap_PAUSE | PHY_Cap_ASYM_PAUSE ;
		
		R1000SetMedium(speed_opt, duplex_opt, autoneg_opt);
	}// end of TBI is not enabled
	else
	{
		IODelay(100);
		DLog("1000Mbps Full-duplex operation, TBI Link %s\n", (ReadMMIO32(CSIDR) & TBILinkOK) ? "OK" : "Failed" );
	}// end of TBI is not enabled
	
#ifdef DEBUG	
	if (ReadMMIO8(PHYstatus) & LinkStatus)
	{
		DLog("Link Status: %s\n","Linked");
		
		if (ReadMMIO8(PHYstatus) & _1000Mbps) {
			DLog("Link Speed: 1000Mbps\n");
		} else if(ReadMMIO8(PHYstatus) & _100Mbps) {
			DLog("Link Speed: 100Mbps\n");
		} else if(ReadMMIO8(PHYstatus) & _10Mbps) {
			DLog("Link Speed: 10Mbps\n");
    }
		
		DLog("Duplex mode: %s\n", ReadMMIO8(PHYstatus) & FullDup ? "Full-Duplex" : "Half-Duplex");
	}
	else
	{
		DLog("Link Status: %s\n", "Not Linked");
	}
#endif
	
	return true;
}

// FIXME - Check against Linux versions, e.g. rtl8168_down()
bool RealtekR1000::R1000StopBoard()
{
	// Need to flesh this out, but for now...
	WriteMMIO8(ChipCmd, 0);
	R1000IRQMaskAndAck();
	R1000InitRxDescCmds(false);
	R1000TxClear();
	R1000NicReset();
	R1000HwPhyReset();
	return true;
}

void RealtekR1000::R1000SetMedium(ushort speedIn, uchar duplexIn, uchar autonegIn)
{
	DLog("R1000SetMedium(%#02x, %#02x, %#02x)\n", speedIn, duplexIn, autonegIn);
	
	if (MCFG_IS_8100(mcfg)) {
		RTL8100SetMedium(speedIn, duplexIn, autonegIn);
	} else if (MCFG_IS_8168(mcfg)) {
		RTL8168SetMedium(speedIn, duplexIn, autonegIn);
	} else if (MCFG_IS_8169(mcfg)) {
		RTL8169SetMedium(speedIn, duplexIn, autonegIn);
	}	
}


void RealtekR1000::R1000HwPhyReset()
{
//	DLog("Reset PHY\n");
	WriteGMII16(0x1f, 0x0000);
	WriteGMII16(PHY_BMCR, (ReadGMII16(PHY_BMCR) | BMCR_RESET));
	
	// FIXME - non-blocking timeout might be nice here...
	for (int i = 0; i < 2500; i++)
	{
		if (MCFG_IS_8169(mcfg))
			WriteGMII16(0x1f, 0x0000);
		if (!(ReadGMII16(PHY_BMCR) & BMCR_RESET))
			break;
		IOSleep(1);
	}
	if (MCFG_IS_8100(mcfg))
		RTL8100HwPhyConfig();
}

void RealtekR1000::R1000HwPhyConfig()
{
//	DLog("PHY config\n");
  rtl8168_init_software_variable();

	
	if (MCFG_IS_8100(mcfg))
	{
		RTL8100HwPhyConfig();
	}
    else if (MCFG_IS_8168(mcfg))
	{
		RTL8168HwPhyConfig();
	}
    else if (MCFG_IS_8169(mcfg))
	{
		RTL8169HwPhyConfig();
	}
}

void RealtekR1000::R1000HwStart()
{
//	DLog("R1000HwStart\n");
	
	// FIXME - This doesn't need to panic, but it does need to cause the calling function to fail.
#ifdef DEBUG
	if (!buffers_inited)
	{
		panic("R1000HwStart called before buffers initialized!!");
		return;
	}
#endif
	
	if (MCFG_IS_8100(mcfg))
	{
		RTL8100HwStart();
	}
	if (MCFG_IS_8168(mcfg))
	{
		RTL8168HwStart();
	}
	else if (MCFG_IS_8169(mcfg))
	{
		RTL8169HwStart();
	}
}

ulong RealtekR1000::ether_crc(int length, unsigned char *data)
{
	slong crc = -1;
	
	while (--length >= 0) 
	{
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^ ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
	}
	
	return crc;
}

//
// Ring buffer descriptor allocation/deallocation
//

bool RealtekR1000::AllocateDescriptorsMemory()
{
//	DLog("AllocateDescriptorsMemory, n_tx_desc = %d, n_rx_desc = %d\n", n_tx_desc, n_rx_desc);
	
	// Allocate Rx descriptor memory
	
	sizeof_rxdesc_space = n_rx_desc * sizeof(RxDesc) + 256;
	rx_descMd = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous | kIODirectionInOut,
													  sizeof_rxdesc_space,
													  page_size);
	
	if (!rx_descMd || rx_descMd->prepare() != kIOReturnSuccess)
	{
		DLog("Couldn't allocate physical memory for rx_desc\n");
		return false;
	}
	
	rxdesc_space = rx_descMd->getBytesNoCopy();
	IOByteCount len = 0;
	rxdesc_phy_dma_addr = rx_descMd->getPhysicalSegment(0, &len);
	if (len < n_rx_desc * sizeof(RxDesc))
	{
		DLog("Couldn't get physical memory segment for rx_desc\n");
		return false;
	}
	
	RxDescArray = reinterpret_cast<RxDesc *>(rxdesc_space);
	
	// Allocate Tx descriptor memory
	
	sizeof_txdesc_space = n_tx_desc * sizeof(TxDesc) + 256;
	tx_descMd = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous | kIODirectionInOut,
													  sizeof_txdesc_space,
													  page_size);
	
	if (!tx_descMd || tx_descMd->prepare() != kIOReturnSuccess)
	{
		DLog("Couldn't allocate physical memory for tx_desc\n");
		return false;
	}
	
	txdesc_space = tx_descMd->getBytesNoCopy();
	len = 0;
	txdesc_phy_dma_addr = tx_descMd->getPhysicalSegment(0, &len);
	if (len < n_tx_desc * sizeof(TxDesc))
	{
		DLog("Couldn't get physical memory segment for tx_desc\n");
		return false;
	}
	
	TxDescArray = reinterpret_cast<TxDesc *>(txdesc_space);
	
	return true;
}

void RealtekR1000::FreeDescriptorsMemory()
{
	if (tx_descMd)
	{
		TxDescArray = NULL;
		tx_descMd->complete();
		tx_descMd->release();
		tx_descMd = NULL;
	}
	
	if (rx_descMd)
	{
		RxDescArray = NULL;
		rx_descMd->complete();
		rx_descMd->release();
		rx_descMd = NULL;
	}
}

//
// Packet Buffer Memory Allocation
//
// This needs to be separate from descriptor allocation for a couple of reasons.
// The first is in case we change from standard to jumbo packets, or vice versa,
// after initialization.
//
// The other very good reason is to facilitate different allocation strategies.
// The original version of the RealtekR1000 driver allocated a memory descriptor
// for each packet buffer, or 2048 memory descriptors in all.  These are fairly
// heavyweight objects.  Just the pointers to the descriptors take up 16KB!
// Worse, since a Mac OS X page is considerably larger than an Ethernet packet,
// internal fragmentation makes more than half of the wired memory unusable.
// 
// It may make more sense to allocate one contiguous chunk of RAM for all Tx buffers,
// and another for all Rx buffers.  Other strategies are possible.
//
// These seven methods encapsulate everything the driver needs to know about packet
// buffer memory allocation: 
//
//  void InitializeBufferMemoryPointers();
//  bool AllocateBufferMemory();
//  void FreeBufferMemory();
//  IOPhysicalAddress RxBufferPhysicalAddress(int n);
//  IOPhysicalAddress TxBufferPhysicalAddress(int n);
//  IOVirtualAddress RxBufferVirtualAddress(int n);
//  IOVirtualAddress TxBufferVirtualAddress(int n);
//  
// As long as they are consistent amongst themselves, any strategy can be implemented.
//

#ifdef R1000_ORIGINAL_BUFFER_ALLOCATION
void RealtekR1000::InitializeBufferMemoryPointers()
{
	int i = 0;
	for (i = 0; i < NUM_RX_DESC; i++)
	{
		Rx_dbuff[i] = NULL;
	}
	for (i = 0; i < NUM_RX_DESC; i++)
	{
		Rx_skbuff_Md[i] = NULL;
	}
	for (i = 0; i < NUM_RX_DESC; i++)
	{
		Rx_skbuff_Dma[i] = NULL;
	}
	
	for (i = 0; i < NUM_TX_DESC; i++)
	{
		Tx_dbuff[i] = NULL;
	}
	for (i = 0; i < NUM_TX_DESC; i++)
	{
		Tx_skbuff_Md[i] = NULL;
	}
	for (i = 0; i < NUM_TX_DESC; i++)
	{
		Tx_skbuff_Dma[i] = NULL;
	}
}

bool RealtekR1000::AllocateBufferMemory()
{
//	DLog("AllocateBufferMemory\n");
	
	IOByteCount len = 0;
	
	// Rx Ring allocation
	for (int i = 0; i < n_rx_desc; i++)
	{
		Rx_dbuff[i] = 0;
		// FIXME - size should be a variable based on requested packet size, not a constant
		// FIXME - do these really need to be page aligned??
		Rx_skbuff_Md[i] = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous | kIODirectionIn,
																MAX_RX_SKBDATA_SIZE,
																page_size);
		if (!Rx_skbuff_Md[i] || Rx_skbuff_Md[i]->prepare() != kIOReturnSuccess)
		{
			DLog("Couldn't allocate physical memory for Rx_dbuff, step %d\n", i);
			return false;
		}
		Rx_dbuff[i] = static_cast<uchar *>(Rx_skbuff_Md[i]->getBytesNoCopy());
		if (!Rx_dbuff[i])
		{
			DLog("Rx_dbuff pointer is NULL, step %d\n", i);
			return false;
		}		
		Rx_skbuff_Dma[i] = Rx_skbuff_Md[i]->getPhysicalSegment(0, &len);
		if (len < MAX_RX_SKBDATA_SIZE)
		{
			DLog("Couldn't get physical segment for Rx_skbuff_Dma, step %d", i);
			return false;
		}
	}
	
	// Tx Ring allocation
	for (int i = 0; i < n_tx_desc; i++)
	{
		Tx_dbuff[i] = NULL;
		// FIXME - size should be a variable based on requested packet size, not a constant
		// FIXME - do these really need to be page aligned??
		Tx_skbuff_Md[i] = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous | kIODirectionOut,
																MAX_TX_SKBDATA_SIZE,
																page_size);
		if (!Tx_skbuff_Md[i] || Tx_skbuff_Md[i]->prepare() != kIOReturnSuccess)
		{
			DLog("Couldn't allocate physical memory for Tx_dbuff, step %d\n", i);
			return false;
		}
		Tx_dbuff[i] = static_cast<uchar *>(Tx_skbuff_Md[i]->getBytesNoCopy());
		if (!Tx_dbuff[i])
		{
			DLog("Tx_dbuff pointer is NULL, step %d\n", i);
			return false;
		}
		Tx_skbuff_Dma[i] = static_cast<IOPhysicalAddress>(Tx_skbuff_Md[i]->getPhysicalSegment(0, &len));
		if (len < MAX_TX_SKBDATA_SIZE)
		{
			DLog("Couldn't get physical segment for Tx_skbuff_Dma, step %d", i);
			return false;
		}
	}
	
	return true;
}

void RealtekR1000::FreeBufferMemory()
{
//	DLog("FreeBufferMemory\n");
	buffers_inited = false;
	
	for (int i = 0; i < n_rx_desc; i++)
	{
		if (Rx_skbuff_Md[i])
		{
			Rx_dbuff[i] = NULL;
			Rx_skbuff_Md[i]->complete();
			Rx_skbuff_Md[i]->release();
			Rx_skbuff_Md[i] = NULL;
		}
	}
	
	for (int i = 0; i < n_tx_desc; i++)
	{
		if (Tx_skbuff_Md[i])
		{
			Tx_dbuff[i] = NULL;
			Tx_skbuff_Md[i]->complete();
			Tx_skbuff_Md[i]->release();
			Tx_skbuff_Md[i] = NULL;
		}
	}	
}
#endif // R1000_ORIGINAL_BUFFER_ALLOCATION

#ifdef R1000_NEW_BUFFER_ALLOCATION_1
void RealtekR1000::InitializeBufferMemoryPointers()
{
	Rx_skbuff_pool_Md = NULL;
	Rx_skbuff_pool_Map = NULL;
	Rx_skbuff_pool_virt_addr = NULL;
	Rx_skbuff_pool_phys_addr = NULL;
	
	Tx_skbuff_pool_Md = NULL;
	Tx_skbuff_pool_Map = NULL;
	Tx_skbuff_pool_virt_addr = NULL;
	Tx_skbuff_pool_phys_addr = NULL;
}

bool RealtekR1000::AllocateBufferMemory()
{
//	DLog("AllocateBufferMemory - NEW\n");
	
	// Rx buffer pool allocation
	// FIXME - buffer size should be a variable, not a macro
	vm_size_t rxBufferPoolSize = MAX_RX_SKBDATA_SIZE * n_rx_desc + page_size;
	Rx_skbuff_pool_Md = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous | kIODirectionIn,
															  rxBufferPoolSize,
															  page_size);
	if (!Rx_skbuff_pool_Md)
	{
		DLog("Failed to allocate %u bytes for Rx buffer pool\n", (unsigned int)rxBufferPoolSize);
		return false;
	}
	
	Rx_skbuff_pool_Map = Rx_skbuff_pool_Md->createMappingInTask(kernel_task,
																NULL, 
																kIOMapAnywhere | kIOMapCopybackCache // ???
																);
	if (!Rx_skbuff_pool_Map)
	{
		DLog("Failed to get virtual memory mapping for Rx buffer pool\n");
		return false;
	}

	Rx_skbuff_pool_virt_addr = Rx_skbuff_pool_Map->getVirtualAddress();
#ifdef DEBUG
	if (!Rx_skbuff_pool_virt_addr)
	{
		DLog("Invalid virtual address for Rx buffer pool\n");
		return false;
	}
#endif
	
	if (Rx_skbuff_pool_Md->prepare() != kIOReturnSuccess)
	{
		DLog("prepare() failed for Rx buffer pool\n");
		return false;
	}
	Rx_skbuff_pool_phys_addr = Rx_skbuff_pool_Map->getPhysicalAddress();
#ifdef DEBUG
	if (!Rx_skbuff_pool_phys_addr)
	{
		DLog("Invalid physical address for Rx buffer pool\n");
		return false;
	}
#endif
	
	// Tx buffer pool allocation
	// FIXME - buffer size should be a variable, not a macro
	vm_size_t txBufferPoolSize = MAX_TX_SKBDATA_SIZE * n_tx_desc + page_size;
	Tx_skbuff_pool_Md = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous | kIODirectionOut,
															  txBufferPoolSize,
															  page_size);
	if (!Tx_skbuff_pool_Md)
	{
		DLog("Failed to allocate %u bytes for Tx packet buffers\n", (unsigned int)rxBufferPoolSize);
		return false;
	}
	
	Tx_skbuff_pool_Map = Tx_skbuff_pool_Md->createMappingInTask(kernel_task,
																NULL, 
																kIOMapAnywhere | kIOMapCopybackCache // ???
																);
	if (!Tx_skbuff_pool_Map)
	{
		DLog("Failed to get virtual memory mapping for Tx buffer pool\n");
		return false;
	}
	
	Tx_skbuff_pool_virt_addr = Tx_skbuff_pool_Map->getVirtualAddress();
#ifdef DEBUG
	if (!Tx_skbuff_pool_virt_addr)
	{
		DLog("Invalid virtual address for Tx buffer pool\n");
		return false;
	}
#endif
	
	if (Tx_skbuff_pool_Md->prepare() != kIOReturnSuccess)
	{
		DLog("prepare() failed for Tx packet buffers\n");
		return false;
	}
	Tx_skbuff_pool_phys_addr = Tx_skbuff_pool_Map->getPhysicalAddress();
#ifdef DEBUG
	if (!Tx_skbuff_pool_phys_addr)
	{
		DLog("Invalid physical address for Tx buffer pool\n");
		return false;
	}
#endif
	
	return true;
}

void RealtekR1000::FreeBufferMemory()
{
//	DLog("FreeBufferMemory\n");
	buffers_inited = false;
	
	if (Rx_skbuff_pool_Md)
	{
		if (Rx_skbuff_pool_phys_addr)
		{
			Rx_skbuff_pool_Md->complete();
		}
		Rx_skbuff_pool_phys_addr = Rx_skbuff_pool_virt_addr = NULL;
		if (Rx_skbuff_pool_Map)
		{
			Rx_skbuff_pool_virt_addr = NULL;
			Rx_skbuff_pool_Map->release();
			Rx_skbuff_pool_Map = NULL;
		}
		Rx_skbuff_pool_Md->release();
		Rx_skbuff_pool_Md = NULL;
	}
	
	if (Tx_skbuff_pool_Md)
	{
		if (Tx_skbuff_pool_phys_addr)
		{
			Tx_skbuff_pool_Md->complete();
		}
		Tx_skbuff_pool_phys_addr = Tx_skbuff_pool_virt_addr = NULL;
		if (Tx_skbuff_pool_Map)
		{
			Tx_skbuff_pool_virt_addr = NULL;
			Tx_skbuff_pool_Map->release();
			Tx_skbuff_pool_Map = NULL;
		}
		Tx_skbuff_pool_Md->release();
		Tx_skbuff_pool_Md = NULL;
	}
}
#endif // R1000_NEW_BUFFER_ALLOCATION_1


void RealtekR1000::InitializeRingBufferDescriptors()
{
	// Rx Ring
	for (int i = 0; i < n_rx_desc; i++)
	{
		RxDescArray[i].status = OSSwapHostToLittleInt32(DescOwn | hw_rx_pkt_len);
		RxDescArray[i].vlan_tag = 0; // not used
		IOPhysicalAddress rxBufAddr = RxBufferPhysicalAddress(i);
#ifdef DEBUG
		if (!rxBufAddr)
		{
			panic("RealtekR1000: NULL Rx buffer physical address, step %d", i);
		}
#endif
		RxDescArray[i].buf_addr = OSSwapHostToLittleInt32(static_cast<UInt32>(rxBufAddr));
#if defined(__LP64__)
		RxDescArray[i].buf_Haddr = OSSwapHostToLittleInt32(static_cast<UInt32>(rxBufAddr >> 32));
#endif
	}
	// mark end of ring
	RxDescArray[n_rx_desc - 1].status = OSSwapHostToLittleInt32(DescOwn | RingEnd | hw_rx_pkt_len);

	// Tx Ring
	for (int i = 0; i < n_tx_desc; i++)
	{
		TxDescArray[i].status = 0;
		TxDescArray[i].vlan_tag = 0; // not used
		IOPhysicalAddress txBufAddr = TxBufferPhysicalAddress(i);
#ifdef DEBUG
		if (!txBufAddr)
		{
			panic("RealtekR1000: NULL Tx buffer physical address, step %d", i);
		}
#endif
		TxDescArray[i].buf_addr = OSSwapHostToLittleInt32(static_cast<UInt32>(txBufAddr));
#if defined(__LP64__)
		TxDescArray[i].buf_Haddr = OSSwapHostToLittleInt32(static_cast<UInt32>(txBufAddr >> 32));
#endif    
	}
	// mark end of ring
	TxDescArray[n_tx_desc - 1].status = OSSwapHostToLittleInt32(RingEnd);	
	
	buffers_inited = true;
}


// Separate function so can disable Rx during suspend/resume

void RealtekR1000::R1000InitRxDescCmds(bool nicOwn)
{
	u32 own = (nicOwn ? DescOwn : 0);
	for (int i = 0; i < n_rx_desc; i++)
	{
		if (i == (n_rx_desc - 1))
		{
			RxDescArray[i].status = OSSwapHostToLittleInt32((own | RingEnd) | hw_rx_pkt_len);
		}
		else
		{
			RxDescArray[i].status = OSSwapHostToLittleInt32(own | hw_rx_pkt_len);
		}
	}	
}

bool RealtekR1000::R1000InitEventSources(IOService *provider)
{
//	DLog("R1000InitEventSources\n");
	
	//Sanity check
	if (workLoop == NULL)
	{
		DLog("R1000InitEventSources: no WorkLoop.\n");
		return false;
	}
	
	transmitQueue = getOutputQueue();
	if (transmitQueue == NULL)
	{ 
		DLog("R1000InitEventSources: no OutputQueue.\n");
		return false;
	}
	
	// we need to see what else is listening on this interrupt to allow multiple devices to function
	// or else we break other things, like other NIC's and what not
	/*
	 int msi_index = -1;
	 int intr_index = 0, intr_type = 0;
	 IOReturn intr_ret;
	 while (true)
	 {
	 intr_ret = provider->getInterruptType(intr_index, &intr_type);
	 if (intr_ret != kIOReturnSuccess) break;
	 if (intr_type & kIOInterruptTypePCIMessaged) msi_index = intr_index;
	 intr_index++;		
	 }
	 
	 DLog("RTL1000: Got interrupt index of '%d'.\n", msi_index);
	 // set to zero just in case we didnt find anything
	 if(!msi_index) { msi_index = 0; }
	 DLog("RTL1000: Got interrupt index of '%d'.\n", msi_index);
	 
	 intSource = IOInterruptEventSource::interruptEventSource(this, 
	 OSMemberFunctionCast(IOInterruptEventSource::Action, this, &RealtekR1000::R1000Interrupt),
	 pciDev, msi_index);
	 
	 // check for some type of fuckup with msi_index = 0
	 */
	intSource = 
	IOInterruptEventSource::interruptEventSource(this, 
												 OSMemberFunctionCast(IOInterruptEventSource::Action, 
																	  this, 
																	  &RealtekR1000::R1000Interrupt),
												 pciDev);
	
	
	
	//Adding interrupt to our workloop event sources
	if (!intSource || workLoop->addEventSource(intSource) != kIOReturnSuccess)
	{
		DLog("R1000InitEventSources: Could not get InterruptEventSource and/or addEventSource.\n");
		return false;
	}
	
	// comment copied from AppleRTL8139Ethernet sources:
	// This is important. If the interrupt line is shared with other devices,
	// then the interrupt vector will be enabled only if all corresponding
	// interrupt event sources are enabled. To avoid masking interrupts for
	// other devices that are sharing the interrupt line, the event source
	// is enabled immediately. Hardware interrupt sources remain disabled.
	
	intSource->enable();
	
	//Registering watchdog (i.e. if timeout exceeds)
	timerSource = 
	IOTimerEventSource::timerEventSource(this, 
										 OSMemberFunctionCast(IOTimerEventSource::Action, 
															  this, 
															  &RealtekR1000::R1000TxTimeout));
	
	if (!timerSource || workLoop->addEventSource(timerSource) != kIOReturnSuccess)
	{
		DLog("R1000InitEventSources: Could not get timerEventSource and/or addEventSource.\n");
		return false;
	}
	
	return true;
}

bool RealtekR1000::R1000OpenAdapter()
{
//	DLog("R1000OpenAdapter\n");
	R1000HwStart();
	return true;
}

void RealtekR1000::R1000CloseAdapter()
{
	/* Stop the chip's Tx and Rx processes. */
	WriteMMIO8(ChipCmd, 0x00);
	
	/* Disable interrupts and clear any interrupt status. */
	R1000IRQMaskAndAck();
	
	/* Update the error counts. */
	//TODO: FIX-ME, Realy need?
	//priv->stats.rx_missed_errors += ReadMMIO32(RxMissed);
	WriteMMIO32(RxMissed, 0);
}

void RealtekR1000::R1000TxClear()
{
//	DLog("R1000TxClear\n");
	cur_tx = dirty_tx = 0;
	
#ifdef DEBUG
	if (TxDescArray == NULL)
	{
		DLog("R1000TxClear called before buffers initialized!!\n");
		return;
	}
#endif
	
	for (int i = 0; i < n_tx_desc; i++)
	{
		TxDescArray[i].status = 0;
		if (Tx_skbuff[i] != NULL)
		{
			freePacket(Tx_skbuff[i]);
			Tx_skbuff[i] = NULL;
		}
	}
}

//analog rtl8168_check_link_status
bool RealtekR1000::R1000CheckLinkStatus()
{
	// read physical status
	if (!MCFG_IS_8100(mcfg))
	{
		WriteGMII16(0x1F, 0x0000);
	}

// rtl8168dp_10mbps_gphy_para
	u8 phy_status = ReadMMIO8(PHYstatus);
  
  if (mcfg == MCFG_8168DP_1) {
    if ((phy_status & LinkStatus) && (phy_status & _10Mbps)) {
      WriteGMII16(0x1f, 0x0000);
      WriteGMII16(0x10, 0x04EE);
    } else {
      WriteGMII16(0x1f, 0x0000);
      WriteGMII16(0x10, 0x01EE);
    }
  }
  
	if (phy_status &= LinkStatus)
	{
    if (mcfg == MCFG_8168F_1 || mcfg == MCFG_8168F_2 || mcfg == MCFG_8411_1) {
			if (ReadMMIO8(PHYstatus) & _1000Mbps) {
				WriteERI(0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
				WriteERI(0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
			} else {
				WriteERI(0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
				WriteERI(0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
			}
			if ((ReadMMIO8(ChipCmd) & (CmdRxEnb | CmdTxEnb))==0) {
				int timeout;
				for (timeout = 0; timeout < 10; timeout++) {
					if ((ReadERI(0x1AE, 4, ERIAR_ExGMAC) & BIT_13)==0)
						break;
					IODelay(1000);
				}
        R1000InitRingIndices();
				WriteMMIO8(ChipCmd, CmdRxEnb | CmdTxEnb);
			}
		} else if ((mcfg == MCFG_8168E_VL_1 || mcfg == MCFG_8168E_VL_2)) {
			u32 eri_data;
			if (mcfg == MCFG_8168E_VL_1 && (ReadMMIO8(PHYstatus) & _10Mbps)) {
				WriteMMIO32(RxConfig, ReadMMIO32(RxConfig) | AcceptAllPhys);
			} else if (mcfg == MCFG_8168E_VL_2) {
				if (ReadMMIO8(PHYstatus) & _1000Mbps) {
					WriteERI(0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
					WriteERI(0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
				} else if (ReadMMIO8(PHYstatus) & _100Mbps) {
					WriteERI(0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
					WriteERI(0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
				} else {
					WriteERI(0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
					WriteERI(0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
				}
			}
      
			eri_data = ReadERI(0xDC, 1, ERIAR_ExGMAC);
			eri_data &= ~BIT_0;
			WriteERI(0xDC, 1, eri_data, ERIAR_ExGMAC);
			eri_data |= BIT_0;
			WriteERI(0xDC, 1, eri_data, ERIAR_ExGMAC);
			if ((ReadMMIO8(ChipCmd) & (CmdRxEnb | CmdTxEnb))==0) {
				int timeout;
				for (timeout = 0; timeout < 10; timeout++) {
					if ((ReadERI(0x1AE, 4, ERIAR_ExGMAC) & BIT_13)==0)
						break;
					IODelay(1000);
				}
        R1000InitRingIndices();
				WriteMMIO8(ChipCmd, CmdRxEnb | CmdTxEnb);
			}
      
		} else if ((mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2) && eee_enable ==1){
			//Full -Duplex  mode
			if (ReadMMIO8(PHYstatus) & FullDup) {
				WriteGMII16(0x1F, 0x0006);
				WriteGMII16(0x00, 0x5a30);
				WriteGMII16(0x1F, 0x0000);
				if (ReadMMIO8(PHYstatus) & (_10Mbps | _100Mbps))
					WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) & ~BIT_19) | BIT_25);
        
			} else {
				WriteGMII16(0x1F, 0x0006);
				WriteGMII16(0x00, 0x5a00);
				WriteGMII16(0x1F, 0x0000);
				if (ReadMMIO8(PHYstatus) & (_10Mbps | _100Mbps))
					WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) & ~BIT_19) | (InterFrameGap << TxInterFrameGapShift));
			}
		} else if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 || mcfg == CFG_METHOD_22 ||
               mcfg == CFG_METHOD_24 ||
               mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B || mcfg == CFG_METHOD_27) {
/*			if ((ReadMMIO8(ChipCmd) & (CmdRxEnb | CmdTxEnb)) == 0) {
        R1000InitRingIndices();
//				rtl8168_desc_addr_fill(tp); //Slice - I don't think it is needed
				WriteMMIO8(ChipCmd, CmdRxEnb | CmdTxEnb);
			}
		} else if (mcfg == CFG_METHOD_23) {
			WriteMMIO32(ERIDR, 0x00000000);
			WriteMMIO32(ERIAR, 0x8042f108); */
      if (ReadMMIO8(PHYstatus) & FullDup)
        WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) | (BIT_24 | BIT_25)) & ~BIT_19);
      else
        WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) | BIT_25) & ~(BIT_19 | BIT_24));

		}
    if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 || mcfg == CFG_METHOD_27) {
      /*half mode*/
      if (!(ReadMMIO8(PHYstatus) & FullDup)) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( PHY_AUTO_NEGO_REG, ReadGMII16( PHY_AUTO_NEGO_REG)&~(PHY_Cap_PAUSE|PHY_Cap_ASYM_PAUSE));
      }
    }


		DLog("Link up\n");
		// report to OSX that link is up
		this->setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, // medium);
							getSelectedMedium());
	}
	else
	{
		DLog("Link down\n");
		// report to OSX that link is down
		this->setLinkStatus(kIONetworkLinkValid);
    switch (mcfg) {
      case MCFG_8401_1:
        WriteGMII16(0x1F, 0x0000);
				WriteGMII16(0x11, ReadGMII16(0x11) | BIT_12);
				WriteGMII16(0x1F, 0x0002);
				WriteGMII16(0x0F, ReadGMII16(0x0F) | BIT_0 | BIT_1);
				WriteGMII16(0x1F, 0x0000);

        break;
      case CFG_METHOD_23:
      case CFG_METHOD_24:
      case CFG_METHOD_25:
      case CFG_METHOD_27:
        WriteMMIO32(ERIDR, 0x00000001);
        WriteMMIO32(ERIAR, 0x8042f108);
      default:
        break;
    }

	}
	
	// FIXME - see rtl8168_check_link_status, rtl8168dp_10mbps_gphy_para
	
	return phy_status;
}

//Interrupt handler
void RealtekR1000::R1000Interrupt(OSObject * client, IOInterruptEventSource * src, int count)
{
  UInt16 speedInt, duplexInt;
  UInt32 currentMediumIndex = MEDIUM_INDEX_AUTO;

#ifdef DEBUG
	// We can't even process interrupts here 
	// because IO isn't initialized!!
	if (!board_inited)
	{
		DLog("Ignoring interrupt, NIC uninitialized\n");
		return;
	}
#endif
//Slice   
//for test about link speed ---------------------------------------
  if (ReadMMIO8(PHYstatus) & _1000Mbps) {
    speedInt = 1000;
  } else if(ReadMMIO8(PHYstatus) & _100Mbps) {
    speedInt = 100;
  } else if(ReadMMIO8(PHYstatus) & _10Mbps) {
    speedInt = 10;
  }
  duplexInt = (ReadMMIO8(PHYstatus) & FullDup)?DUPLEX_FULL:DUPLEX_HALF;
  
  if(speedInt == 10 && duplexInt == DUPLEX_FULL) currentMediumIndex = MEDIUM_INDEX_10FD;
  if(speedInt == 100 && duplexInt == DUPLEX_FULL) currentMediumIndex = MEDIUM_INDEX_100FD;
  if(speedInt == 1000 && duplexInt == DUPLEX_FULL) currentMediumIndex = MEDIUM_INDEX_1000FD;
  if(speedInt == 10 && duplexInt == DUPLEX_HALF) currentMediumIndex = MEDIUM_INDEX_10HD;
  if(speedInt == 100 && duplexInt == DUPLEX_HALF) currentMediumIndex = MEDIUM_INDEX_100HD;
  if(speedInt == 1000 && duplexInt == DUPLEX_HALF) currentMediumIndex = MEDIUM_INDEX_1000HD;		
  
  setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, mediumTable[currentMediumIndex], speedInt * MBit, NULL);
  
  
//-----------------------------------------------------------------  
  
	
	// disable further interrupts while we're working on this one
	WriteMMIO16(IntrMask, 0x0000);
	
	// get status from card
	u16 status = ReadMMIO16(IntrStatus);
	
	// clear interrupt status bits
	WriteMMIO16(IntrStatus, status);
	
//	DLog("R1000Interrupt, status = %#04x\n", status);
	
	int boguscnt = max_interrupt_work;
	do 
	{
		// from realtek linux driver
		// ** hotplug/major error/no more work/shared irq
		if (status == 0xFFFF) 
			break;
		
		// if no work to be done...exit
		if ((status & intr_mask) == 0) 
			break;
		
		//FIXME - 8168 has Rx FIFO o'flow check here
		
		//FIXME - implement PCI error handling
		if (status & SYSErr)
		{
			R1000PCIErrorInterrupt(); //rtl8168_pcierr_interrupt
			break;
		}
		
		// Seems redundant, but it's in the 8168 code...
		if ((status & TxOK) && (status & TxDescUnavail))
		{
			WriteMMIO8(TxPoll, NPQ);	/* set polling bit */
		}
		
		// if the link status has changed
		if (status & LinkChg)
		{
			R1000CheckLinkStatus();
		}
		
		// handle RX
		if (status & (RxOK | RxDescUnavail | RxFIFOOver)) 
		{
			R1000RxInterrupt(status);
		}
		
		// handle TX
		if (status & (TxOK | TxErr)) 
		{
			R1000TxInterrupt(status);
		}
		
		boguscnt--;
		
		// get new status from card
		status = ReadMMIO16(IntrStatus);
		
		// clear interrupt status bits
		WriteMMIO16(IntrStatus, status);
	} 
	while (boguscnt > 0);
	
	if (boguscnt <= 0) 
	{
		// FIXME - probably should reset chip at this point
		DLog("Too much work at interrupt!\n");
	}
	
	// write clean mask
	WriteMMIO16(IntrMask, intr_mask);
	// DLog("Interrupt service complete\n");
}

void RealtekR1000::R1000RxInterrupt(u16 intrStatus)
{
#ifdef DEBUG
	// quick, cheap sanity checks
	if (!buffers_inited)
	{
		DLog("R1000RxInterrupt called before buffers initialized!!\n");
		return;
	}
	if (etherif == NULL)
	{
		DLog("R1000RxInterrupt called before etherif initialized!!\n");
		return;
	}
#endif

	// DLog("R1000RxInterrupt(%#04x), cur_rx=%u\n", intrStatus, cur_rx);
	BUMP_ETHER_RX_COUNTER(interrupts);
	
	if (intrStatus & RxErr)
	{
		DLog("R1000RxInterrupt Rx error\n");
		BUMP_NET_COUNTER(inputErrors);
	}
	
	int rxPacketsToFlush = max_interrupt_work;
	bool rxOverflow = (intrStatus & RxDescUnavail);
	bool rxOK		= (intrStatus & RxOK);
	
	if (rxOverflow)
	{
		DLog("Rx overflow detected! Starting recovery\n");
		
		// Turn off receive
		WriteMMIO8(ChipCmd, (ReadMMIO8(ChipCmd) & ~CmdRxEnb));
		// Prepare to go around the whole ring
		rxPacketsToFlush = n_rx_desc;
	}
	
	ulong lcur_rx = cur_rx;
	int rxPacketsFlushed = 0;
	
	do
	{	
		struct RxDesc *rxdesc = &RxDescArray[lcur_rx];
		ulong rxdesc_status = OSSwapLittleToHostInt32(rxdesc->status);
		
		// *** README ***
		// Normally we enter this loop with lcur_rx pointing at the
		// next Rx buffer to flush, and we simply step forward through
		// the ring until we run out of populated buffers.
		//
		// This breaks down if cur_rx gets out of sync with the chip's
		// counter. This could be caused by an unexpected reset, recovery
		// from an Rx overflow, or the driver writer having a brain fart. 
		//
		// To deal with these cases, we need to search for the
		// first populated buffer so we can resync cur_rx with the chip.
		
		if (rxdesc_status & DescOwn)
		{
			// lcur_rx points to an empty buffer.
			// Are we done?
			if (rxOK)
			{
				if (rxPacketsFlushed > 0)
					break; // we're done - normal exit from loop
			}
		}
		else
		{
			// Buffer is ours to play with
			rxPacketsFlushed++;
			if ((rxdesc_status & LastFrag) && (rxdesc_status & RxRES))
			{
				if (rxdesc_status & RxRWT)
				{
					BUMP_ETHER_RX_COUNTER(watchdogTimeouts);
				}
				if (rxdesc_status & RxRUNT)
				{
					BUMP_ETHER_RX_COUNTER(frameTooShorts);
				}
				// TODO - Add more error counters?
				if (rxdesc_status & RxCRC)
				{
					//priv->stats.rx_crc_errors++;
				}
			}
			else
			{
				ulong pkt_size = (OSSwapLittleToHostInt32(rxdesc->status) & 0x00001FFF) - 4;
				if (pkt_size > rx_pkt_len)
				{
					DLog("R1000RxInterrupt: Rx packet size() > mtu()+14!\n");
					BUMP_ETHER_COUNTER(frameTooLongs);
					pkt_size = rx_pkt_len;
				}
				
				mbuf_t rx_skb = allocatePacket(pkt_size); // was MAX_RX_SKBDATA_SIZE
				if (rx_skb)
				{
					bcopy(RxBufferVirtualAddress(lcur_rx), mbuf_data(rx_skb), pkt_size);
					mbuf_setlen(rx_skb, pkt_size);
					// DLog("Rx packet len %u\n", pkt_size);
					etherif->inputPacket(rx_skb, pkt_size, IONetworkInterface::kInputOptionQueuePacket);
					BUMP_NET_COUNTER(inputPackets);
				}
				else
				{
					DLog("R1000RxInterrupt: failed to allocate mbuf, ignoring packet\n");
					BUMP_ETHER_RX_COUNTER(resourceErrors);
				}
			}
			
			// Give rx descriptor back to chip
			if (lcur_rx == (n_rx_desc - 1))
			{
				RxDescArray[lcur_rx].status = OSSwapHostToLittleInt32((DescOwn | RingEnd) | hw_rx_pkt_len);
			}
			else
			{
				RxDescArray[lcur_rx].status = OSSwapHostToLittleInt32(DescOwn | hw_rx_pkt_len);
			}
		}
		
	    lcur_rx = (lcur_rx + 1) % n_rx_desc;
	}
	while ((lcur_rx != cur_rx) && (rxPacketsFlushed <= rxPacketsToFlush));
	
	if (rxPacketsFlushed) {
		etherif->flushInputQueue();
//	DLog("R1000RxInterrupt received %d packets, cur_rx = %d\n", rxPacketsFlushed, lcur_rx);
  }
	
	if (rxOverflow) {
//		DLog("Rx overflow recovery complete\n");
		
		// Safe to re-enable receive now (?)
		WriteMMIO8(ChipCmd, (ReadMMIO8(ChipCmd) | CmdRxEnb));
	}
#ifdef DEBUG
	else if (lcur_rx != ((cur_rx + rxPacketsFlushed) % n_rx_desc))
	{
		DLog("Resynchronized cur_rx\n");
	}
#endif	
	cur_rx = lcur_rx;
}

// At entry:
//  (dirty_tx % n_tx_desc) is the index of the next packet to free
//  (cur_tx % n_tx_desc) is the index of the first free packet descriptor
void RealtekR1000::R1000TxInterrupt(u16 intrStatus)
{
#ifdef DEBUG
	// quick, cheap sanity check
	if (!buffers_inited)
	{
		DLog("R1000TxInterrupt called before buffers initialized!!\n");
		return;
	}
#endif
	//DLog("R1000TxInterrupt(%#04x)\n", intrStatus);
	BUMP_ETHER_TX_COUNTER(interrupts);
	
	// check Tx error status
	if (intrStatus & TxErr)
	{
		DLog("R1000TxInterrupt: Tx abort\n");
		BUMP_NET_COUNTER(outputErrors);
	}
	
	// clean up after sent buffers
	ulong ldirty_tx = dirty_tx;
	ulong tx_left = cur_tx - ldirty_tx;
	//DLog("R1000TxInterrupt cur_tx %d, dirty_tx %d, tx_left %d\n", cur_tx, dirty_tx, tx_left);
	
	tx_left = min(tx_left, max_interrupt_work);
	while (tx_left > 0)
	{
		ulong entry = ldirty_tx % n_tx_desc;
		if ((OSSwapLittleToHostInt32(TxDescArray[entry].status) & DescOwn) == 0)
		{
			if (Tx_skbuff[entry] != NULL)
			{
				/*	
				DLog("packet sent, packet size %u, phys addr %x%08x\n", 
					 mbuf_len(Tx_skbuff[entry]),
					 OSSwapLittleToHostInt32(TxDescArray[entry].buf_Haddr),
					 OSSwapLittleToHostInt32(TxDescArray[entry].buf_addr));
				 */
				// Clean up after this packet
				freePacket(Tx_skbuff[entry]);
				Tx_skbuff[entry] = NULL;
				BUMP_NET_COUNTER(outputPackets);
			}
			else
			{
				// Queue slot was reserved but not used
				// See outputPacket() for possible causes
				DLog("Tx_skbuff[%d] is NULL\n", entry);
			}
			
			ldirty_tx++;
			tx_left--;
		}
		else 
		{
//			DLog("Tx packet at index %d is pending\n", entry);
			break;
		}
	}
	
	if (dirty_tx != ldirty_tx) 
	{
//		DLog("R1000TxInterrupt cleaned up %d Tx packets\n", ldirty_tx - dirty_tx);
		dirty_tx = ldirty_tx;
		// restart transmit queue in case we had been stalled
		transmitQueue->start();
	}
}

//rtl8168_pcierr_interrupt
void RealtekR1000::R1000PCIErrorInterrupt()
{
	DLog("R1000PCIErrorInterrupt\n");
#ifdef PCI_ERROR	
  u16 pci_status, pci_cmd;
  
	pci_cmd = pciDev->configRead16(kIOPCIConfigCommand);
	pci_status = pciDev->configRead16(kIOPCIConfigStatus);

  pciDev->configWrite16(kIOPCIConfigCommand,
                        pci_cmd | kIOPCICommandSERR | kIOPCICommandParityError);
  
	pciDev->configWrite16(kIOPCIConfigStatus,
                        pci_status & (kIOPCIStatusParityErrActive |
                                      kIOPCIStatusSERRActive | kIOPCIStatusMasterAbortActive |
                                      kIOPCIStatusTargetAbortActive));
  
//	rtl8168_hw_reset(dev);
  WriteMMIO16(IntrMask, 0x0000);
	RTL8168NicReset();  
#endif  
}

void RealtekR1000::R1000TxTimeout(OSObject *owner, IOTimerEventSource * timer)
{
//	DLog("R1000TxTimeout\n");
	
	uchar tmp8;
	
	/* disable Tx, if not already */
	tmp8 = ReadMMIO8( ChipCmd );
	if (tmp8 & CmdTxEnb)
	{
		WriteMMIO8 (ChipCmd, tmp8 & ~CmdTxEnb);
	}
	
	/* Disable interrupts by clearing the interrupt mask. */
	WriteMMIO16(IntrMask, 0x0000);
	
	BUMP_ETHER_TX_COUNTER(timeouts);
	
	R1000TxClear();
	R1000HwStart();
	transmitQueue->start();
}


//
// EEPROM access
// Derived (OK, largely copied) from the Realtek RTL8168 driver for Linux
//


//-------------------------------------------------------------------
//rtl_eeprom_type():
//	tell the eeprom type
//return value:
//	0: the eeprom type is 93C46
//	1: the eeprom type is 93C56 or 93C66
//-------------------------------------------------------------------
int RealtekR1000::rtl_eeprom_type()
{
	return ReadMMIO32(RxConfig) & RxCfg_9356SEL;
}

void RealtekR1000::rtl_eeprom_cleanup()
{
	u8 x;
	
	x = ReadMMIO8(Cfg9346);
	x &= ~(Cfg9346_EEDI | Cfg9346_EECS);
	
	WriteMMIO8(Cfg9346, x);
	
	rtl_raise_clock(&x);
	rtl_lower_clock(&x);
}

int RealtekR1000::rtl_eeprom_cmd_done()
{
	u8 x;
	int i;
	
	rtl_stand_by();
	
	for (i = 0; i < 50000; i++) {
		x = ReadMMIO8(Cfg9346);
		
		if (x & Cfg9346_EEDO) {
			IODelay(RTL_CLOCK_RATE * 2 * 3);
			return 0;	
		}
		IODelay(1);
	}
	
	return -1;
}

//-------------------------------------------------------------------
//rtl_eeprom_read_sc():
//	read one word from eeprom
//-------------------------------------------------------------------
u16 RealtekR1000::rtl_eeprom_read_sc(u16 reg)
{
	
	int addr_sz = 6;
	u8 x;
	u16 data;
	
	if (rtl_eeprom_type())
		addr_sz = 8;
	else
		addr_sz = 6;
	
	x = ReadMMIO8(Cfg9346);
	x &= ~(Cfg9346_EEDI | Cfg9346_EEDO | Cfg9346_EESK);
	x |= Cfg9346_EEM1 | Cfg9346_EECS;
	WriteMMIO8(Cfg9346, x);
	
	rtl_shift_out_bits(RTL_EEPROM_READ_OPCODE, 3);
	rtl_shift_out_bits(reg, addr_sz);
	
	data = rtl_shift_in_bits();
	
	rtl_eeprom_cleanup();
	
	return data;
}

//-------------------------------------------------------------------
//rtl_eeprom_write_sc():
//	write one word to a specific address in the eeprom
//-------------------------------------------------------------------
void RealtekR1000::rtl_eeprom_write_sc(u16 reg, u16 data)
{
	u8 x;
	int addr_sz = 6;
	int w_dummy_addr = 4;
	
	if (rtl_eeprom_type()) {
		addr_sz = 8;
		w_dummy_addr = 6;
	} else {
		addr_sz = 6;
		w_dummy_addr = 4;
	}
	
	x = ReadMMIO8(Cfg9346);
	x &= ~(Cfg9346_EEDI | Cfg9346_EEDO | Cfg9346_EESK);
	x |= Cfg9346_EEM1 | Cfg9346_EECS;
	WriteMMIO8(Cfg9346, x);
	
	rtl_shift_out_bits(RTL_EEPROM_EWEN_OPCODE, 5);
	rtl_shift_out_bits(reg, w_dummy_addr);
	rtl_stand_by();
	
	rtl_shift_out_bits(RTL_EEPROM_ERASE_OPCODE, 3);
	rtl_shift_out_bits(reg, addr_sz);
	if (rtl_eeprom_cmd_done() < 0) {
		return;
	}
	rtl_stand_by();
	
	rtl_shift_out_bits(RTL_EEPROM_WRITE_OPCODE, 3);
	rtl_shift_out_bits(reg, addr_sz);
	rtl_shift_out_bits(data, 16);
	if (rtl_eeprom_cmd_done() < 0) {
		return;
	}
	rtl_stand_by();
	
	rtl_shift_out_bits(RTL_EEPROM_EWDS_OPCODE, 5);
	rtl_shift_out_bits(reg, w_dummy_addr);
	
	rtl_eeprom_cleanup();
}

void RealtekR1000::rtl_raise_clock(u8 *x)
{
	
	*x = *x | Cfg9346_EESK;
	WriteMMIO8(Cfg9346, *x);
	IODelay(RTL_CLOCK_RATE);
}

void RealtekR1000::rtl_lower_clock(u8 *x)
{
	
	*x = *x & ~Cfg9346_EESK;
	WriteMMIO8(Cfg9346, *x);
	IODelay(RTL_CLOCK_RATE);
}

void RealtekR1000::rtl_shift_out_bits(int data, int count)
{
	u8 x;
	int  mask;
	
	mask = 0x01 << (count - 1);
	x = ReadMMIO8(Cfg9346);
	x &= ~(Cfg9346_EEDI | Cfg9346_EEDO);
	
	do {
		x &= ~Cfg9346_EEDI;
		if (data & mask)
			x |= Cfg9346_EEDI;
		
		WriteMMIO8(Cfg9346, x);
		IODelay(RTL_CLOCK_RATE);
		rtl_raise_clock(&x);
		rtl_lower_clock(&x);
		mask = mask >> 1;
	} while(mask);
	
	x &= ~Cfg9346_EEDI;
	WriteMMIO8(Cfg9346, x);
}

u16 RealtekR1000::rtl_shift_in_bits()
{
	u8 x;
	u16 d, i;
	
	x = ReadMMIO8(Cfg9346);
	x &= ~(Cfg9346_EEDI | Cfg9346_EEDO);
	
	d = 0;
	
	for (i = 0; i < 16; i++) {
		d = d << 1;
		rtl_raise_clock(&x);
		
		x = ReadMMIO8(Cfg9346);
		x &= ~Cfg9346_EEDI;
		
		if (x & Cfg9346_EEDO)
			d |= 1;
		
		rtl_lower_clock(&x);
	}
	
	return d;
}

void RealtekR1000::rtl_stand_by()
{
	u8 x;
	
	x = ReadMMIO8(Cfg9346);
	x &= ~(Cfg9346_EECS | Cfg9346_EESK);
	WriteMMIO8(Cfg9346, x);
	IODelay(RTL_CLOCK_RATE);
	
	x |= Cfg9346_EECS;
	WriteMMIO8(Cfg9346, x);
}
