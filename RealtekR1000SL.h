/*
 *  RealtekR1000SL.h - Class definition for Realtek Ethernet driver
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

#ifndef _REALTEKR1000_H_
#define _REALTEKR1000_H_

#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOBasicOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOFilterInterruptEventSource.h>

extern "C"
{
#include <sys/kpi_mbuf.h>
#include <architecture/i386/pio.h>
}

#include "R1000Regs.h"
#include "mii.h"

#define RealtekR1000 com_chucko_RealtekR1000

//
// Memory allocation method
//
// Choose ONE of these
//

//#define R1000_ORIGINAL_BUFFER_ALLOCATION
#define R1000_NEW_BUFFER_ALLOCATION_1

//
// Debug logging
//

#ifdef DEBUG
#define DLog(format, ...) IOLog("%s: "format, bsdName, ##__VA_ARGS__)
#else 
#define DLog(...)
#endif

//
// Configuration macros
//

//#define R1000_CHECKSUM_OFFLOAD

#define BSD_NAME_LEN 16

enum
{
	MEDIUM_INDEX_10HD	= 0,
	MEDIUM_INDEX_10FD	= 1,
	MEDIUM_INDEX_100HD	= 2,
	MEDIUM_INDEX_100FD	= 3,
	MEDIUM_INDEX_1000HD = 4,
	MEDIUM_INDEX_1000FD = 5,
	MEDIUM_INDEX_AUTO	= 6,
	MEDIUM_INDEX_COUNT	= 7
};

// Power states

enum 
{
	kR1000PowerStateOff = 0,
	kR1000PowerStateOn,
	kR1000PowerStateCount
};


class RealtekR1000 : public IOEthernetController
{
	OSDeclareDefaultStructors(RealtekR1000)
	
public:
	
	//
	// IONetworkController API
	//
	
	virtual bool            init(OSDictionary *properties);
	virtual void            free();
	virtual bool            start(IOService *provider);
	virtual void            stop(IOService *provider);
	
	virtual IOReturn        enable(IONetworkInterface *netif);
  virtual IOReturn        disable(IONetworkInterface *netif);
  virtual bool            setLinkStatus(	UInt32					status,
                                const IONetworkMedium	*activeMedium = 0,
                                UInt64					speed = 0,
                                OSData					*data = 0 );
	
  virtual UInt32          outputPacket(mbuf_t m, void *param);
  virtual void            getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const;
  virtual IOOutputQueue   *createOutputQueue();
  virtual const OSString	*newVendorString() const;
  virtual const OSString	*newModelString() const;
  virtual IOReturn        selectMedium(const IONetworkMedium *medium);
  virtual bool            configureInterface(IONetworkInterface *netif);
  virtual bool            createWorkLoop();
  virtual IOWorkLoop      *getWorkLoop() const;
  virtual IOReturn        getHardwareAddress(IOEthernetAddress *addr);
	
	// Broadcast, multicast, promiscuous modes
  virtual IOReturn        setPromiscuousMode(bool enabled);
  virtual IOReturn        setMulticastMode(bool enabled);
  virtual IOReturn        setMulticastList(IOEthernetAddress *addrs, UInt32 count);
	
	// Kernel debug methods
  virtual void            sendPacket(void *pkt, UInt32 pkt_len);
  virtual void            receivePacket(void * pkt, UInt32 *pkt_len, UInt32 timeout);
	
	// Power management
  virtual IOReturn        registerWithPolicyMaker(IOService *policyMaker);
  virtual IOReturn        setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker);
	
	//
	// H/W checksum support
	virtual IOReturn        getChecksumSupport(UInt32 *checksumMask,
                                         UInt32 checksumFamily,
                                         bool isOutput);
	
	
private:
	//
	// Private methods used in IONetworkInterface API
	//
	
  void R1000Interrupt(OSObject * client, IOInterruptEventSource * src, int count);
	void R1000TxTimeout(OSObject *owner, IOTimerEventSource * timer);
  
	//
	// IOKit interface objects
	//
	
	IOPCIDevice						  *pciDev;
	IOWorkLoop						  *workLoop;
	IOInterruptEventSource	*intSource;
  IOTimerEventSource			*timerSource;
  IONetworkStats					*netStats;
  IOEthernetStats					*etherStats;
	IOOutputQueue					  *transmitQueue;
  IOEthernetInterface			*etherif;
	OSDictionary					  *mediumDict;
	const IONetworkMedium		*mediumTable[MEDIUM_INDEX_COUNT];	// *** only used in increaseActivationLevel() ??
  
	//
	// BSD name for this device
	//
	char                    bsdName[BSD_NAME_LEN];
	
	//
	// OS flag data for configureInterface() method
	//
	UInt32                  canOffload;		// chip h/w capabilities
	
	bool                    board_inited;
	bool                    buffers_inited;
	bool                    enabled;
	bool                    linked;
	
	UInt32                  activationLevel;
	bool                    enabledForBSD;
	bool                    enabledForKDP;
  
	unsigned long           powerState;
	bool                    rx_fifo_overflow;
	
	// HW Configuration info
  
	int mcfg;						// Which chip
	u32 rx_config_base;				// Base value of RxConfig register for this chip
	u32 rx_config_mask;				// Mask for RxConfig register
	u16 max_jumbo_frame_sz;			// Max jumbo frame size for this chip ; 0 == not supported
	u16 efuse;						// EFUSE capability (some 8168 variants only)
	
	ulong expire_time;
	
	ulong n_rx_desc;				/* Number of receive buffers for this chip. */
	ulong cur_rx;                   /* Index into the Rx descriptor buffer of next Rx pkt. Max value is n_rx_desc - 1. */
	
	// cur_tx and dirty_tx are indices into the Tx descriptor ring buffer.
	// (cur_tx % n_tx_desc) is the index of the next free Tx descriptor.
	// (dirty_tx % n_tx_desc) is the index of the Tx descriptor currently being transmitted (if any).
	// dirty_tx lags cur_tx when transmission is in progress.
	// Both increase monotonically - no consequences if they wrap around,
	// as long as n_tx_desc is a power of 2.
	ulong n_tx_desc;				/* Number of transmit buffers for this chip. */
	ulong cur_tx;                   /* Index into the Tx descriptor buffer of next Tx pkt. */
	ulong dirty_tx;
	
	//
	// Transmit buffers
	//
	
	// packets as passed in by caller
	struct	__mbuf				*Tx_skbuff[NUM_TX_DESC];
#ifdef R1000_ORIGINAL_BUFFER_ALLOCATION
  // virtual memory addresses of buffer pool
	uchar						*Tx_dbuff[NUM_TX_DESC];
	// OS descriptors
	IOBufferMemoryDescriptor	*Tx_skbuff_Md[NUM_TX_DESC];
	// Physical addresses for hardware to use
	IOPhysicalAddress			Tx_skbuff_Dma[NUM_TX_DESC];
#endif // R1000_ORIGINAL_BUFFER_ALLOCATION
	
#ifdef R1000_NEW_BUFFER_ALLOCATION_1
	// OS descriptor
	IOBufferMemoryDescriptor	*Tx_skbuff_pool_Md;
	// Virtual memory map
	IOMemoryMap					*Tx_skbuff_pool_Map;
	// Physical base address
	IOPhysicalAddress			Tx_skbuff_pool_phys_addr;
	// Virtual base address
	IOVirtualAddress			Tx_skbuff_pool_virt_addr;
#endif // R1000_NEW_BUFFER_ALLOCATION_1
	
	//
	// Receive buffers
	//
	
#ifdef R1000_ORIGINAL_BUFFER_ALLOCATION
  // virtual memory addresses of buffer pool
	uchar						*Rx_dbuff[NUM_RX_DESC];
	// OS descriptors
	IOBufferMemoryDescriptor	*Rx_skbuff_Md[NUM_RX_DESC];
	// Physical addresses for hardware to use
	IOPhysicalAddress			Rx_skbuff_Dma[NUM_RX_DESC];
#endif // R1000_ORIGINAL_BUFFER_ALLOCATION
  
#ifdef R1000_NEW_BUFFER_ALLOCATION_1
	// OS descriptor
	IOBufferMemoryDescriptor	*Rx_skbuff_pool_Md;
	// Virtual memory map
	IOMemoryMap					*Rx_skbuff_pool_Map;
	// Physical base address
	IOPhysicalAddress			Rx_skbuff_pool_phys_addr;
	// Virtual base address
	IOVirtualAddress			Rx_skbuff_pool_virt_addr;
#endif // R1000_NEW_BUFFER_ALLOCATION_1
	
	void *txdesc_space;
	struct	TxDesc	*TxDescArray;           /* Index of 256-alignment Tx Descriptor buffer */
	IOBufferMemoryDescriptor *tx_descMd;
	IOPhysicalAddress txdesc_phy_dma_addr;
	int sizeof_txdesc_space;
	
	void *rxdesc_space;
	struct	RxDesc	*RxDescArray;           /* Index of 256-alignment Rx Descriptor buffer */
	IOBufferMemoryDescriptor *rx_descMd;
	IOPhysicalAddress rxdesc_phy_dma_addr;
	int sizeof_rxdesc_space;
	
	// Maximum packet sizes
	// These will always be smaller than 2^16.
	ulong curr_mtu_size;
	ulong tx_pkt_len;
	ulong rx_pkt_len;
	ulong hw_rx_pkt_len;
	
	// Medium settings
	u16	speed;
	u8	duplex;
	u8	autoneg;
  u16 aspm;

	//
	// Cached copies of registers
	//
	u16 cp_cmd;
	u16 intr_mask;
	
	u32 msg_enable;
	u32 tx_tcp_csum_cmd;
	u32 tx_udp_csum_cmd;
	u32 tx_ip_csum_cmd;

  u8 org_pci_offset_99;
  u8 org_pci_offset_180;
  u8 issue_offset_99_event;

  u8 org_pci_offset_80;
  u8 org_pci_offset_81;
  u8 use_timer_interrrupt;

	u16 wol_enabled; /* Wake On Lan */
  bool wolCapable;
  bool wolActive;

  unsigned features;
  unsigned wol_opts;
  u16 mac_ocp_data;
//  u16 gphy_val;
  u32 bios_setting;

	ulong mc_filter0, mc_filter1;	// cached multicast filter bits
	
	static int max_interrupt_work;
	static UInt32 multicast_filter_limit;
	static const u32 ethernet_polynomial;
	
	static const u16 r1000_intr_mask;
	static const u16 rtl8101_intr_mask;
	static const u16 rtl8101_napi_event;
	static const uint32_t rtl8101_rx_config;
	static const u16 rtl8168_intr_mask;
	static const u16 rtl8168_napi_event;
	static const struct RtlChipInfo rtl_chip_info[];
	static struct IOPMPowerState powerStateArray[kR1000PowerStateCount];
	
	//
	// Primitive IO operations
	//
	
	UInt16							pioBase;		// Port IO base address
	IOMemoryMap					*mmioBase;		// Memory map for IO
	bool							  forcedPio;		// True = no memory mapped IO
	
	//
	// Writes - equivalent to RTL_W{8,16,32}
	//
	inline void WriteMMIO8(ushort offset, uchar value)
	{ (forcedPio) ? outb(pioBase + offset, value) : pciDev->ioWrite8(offset, value, mmioBase); }
	inline void WriteMMIO16(ushort offset, ushort value)
	{ (forcedPio) ? outw(pioBase + offset, value) : pciDev->ioWrite16(offset, value, mmioBase); }
	inline void WriteMMIO32(ushort offset, UInt32 value)
	{ (forcedPio) ? outl(pioBase + offset, value) : pciDev->ioWrite32(offset, value, mmioBase); }
	
	//
	// Reads - equivalent to RTL_R{8,16,32}
	//
	inline uchar ReadMMIO8(ushort offset)
	{ return ((forcedPio) ? inb(pioBase + offset) : pciDev->ioRead8(offset, mmioBase)); }
	inline ushort ReadMMIO16(ushort offset)
	{ return ((forcedPio) ? inw(pioBase + offset) : pciDev->ioRead16(offset, mmioBase)); }
	inline ulong ReadMMIO32(ushort offset)
	{ return ((forcedPio) ? inl(pioBase + offset) : pciDev->ioRead32(offset, mmioBase)); }
	
	//
	// Less primitive IO
	//
	
	//
	// GMII access
	// Equivalent to mdio_{write,read}
	//
	void WriteGMII16(int RegAddr, u16 value);
	u16 ReadGMII16(int RegAddr);
	
	//
	// EPHY access
	// Equivalent to RTL8xxx_ephy_{write,read}
	//
	void WriteEPHY16(int RegAddr, u16 value);
	u16 ReadEPHY16(int RegAddr);
	
	//
	// CSI access
	// Equivalent to RTL8xxx_csi_{write,read}
	//
	void WriteCSI32(int addr, int value);
	int ReadCSI32(int addr);

  u32 rtl8168_csi_other_fun_read(u8 multi_fun_sel_bit,
                                 u32 addr);
  void rtl8168_csi_other_fun_write(u8 multi_fun_sel_bit,
                                   u32 addr,
                                   u32 value);
  u8 rtl8168_csi_fun0_read_byte(u32 addr);
  void rtl8168_csi_fun0_write_byte(u32 addr, u8 value);

  void rtl8168_get_hw_wol();
  void rtl8168_set_hw_wol(u32 wolopts);
	
	//
	// ERI Register access
	// Equivalent to rtl8xxx_eri_write/rtl8xxx_eri_read
	int WriteERI(int addr, int len, int value, int type);
	int ReadERI(int addr, int len, int type);
  
	//
	// EEPROM access
	//
	int  rtl_eeprom_type();
	void rtl_eeprom_cleanup();
	u16  rtl_eeprom_read_sc(u16 reg);
	void rtl_eeprom_write_sc(u16 reg, u16 data);
	void rtl_shift_out_bits(int data, int count);
	u16  rtl_shift_in_bits();
	void rtl_raise_clock(u8 *x);
	void rtl_lower_clock(u8 *);
	void rtl_stand_by();
	int  rtl_eeprom_cmd_done();

  void rtl8168_enable_rxdvgate();
  void rtl8168_disable_rxdvgate();
  void rtl8168_wait_txrx_fifo_empty();
  void rtl8168_rar_set(uint8_t *addr);
	
	
	void R1000GetMacVersion();
	bool R1000InitBoard();
	bool R1000ProbeAndStartBoard();
	bool R1000StopBoard();
  
  IOReturn setWakeOnMagicPacket(bool active);
	
	// Meant to be called as an IOWorkLoop Action.
	IOReturn R1000ResetTask();
	
	void R1000SetMedium(ushort speed, uchar duplex, uchar autoneg);
	
	bool increaseActivationLevel(UInt32 level);
	bool decreaseActivationLevel(UInt32 level);
	bool setActivationLevel(UInt32 level);
	
	void R1000HwPhyReset();
	void R1000HwPhyConfig();
	void R1000NicReset();
	void R1000HwStart();
	void R1000TxClear();
	void R1000Suspend();
	void R1000Resume();
	bool R1000CheckLinkStatus();
	
	void R1000DSM(int dev_state);
	
	void R1000PowerDownPLL();
	void R1000PowerUpPLL();
	
	void R1000PowerDownPHY();
	void R1000PowerUpPHY();
	
	void R1000ASICDown();
	
	void R1000IRQMaskAndAck();
	
	
	inline void R1000InitRingIndices()
	{
		cur_rx = cur_tx = dirty_tx = 0;
	}
  
  inline u16 map_phy_ocp_addr(u16 page, u16 reg)
  {
    if (page != OCP_STD_PHY_BASE_PAGE) {
      reg -= 16;
    }
    
    page <<= 4;
    reg <<= 1;
    
    return (page + reg);
  }
  

	
	//
	// Specialized methods by chip family
	//
	
	// RTL8100
	//
	void RTL8100HwStart();
	// Chip specific subroutines
	void RTL8100HwStart1Gen();
	void RTL8105EHwStart1();
	void RTL8105EHwStart();
	void RTL8402HwStart();
  void RTL8106HwStart();
  
	void RTL8100HwPhyConfig();
	// Chip specific configuration
	void RTL8102EHwPhyConfig();
	void RTL8401PhyHwConfig();
	void RTL8105E1HwPhyConfig();
	void RTL8105EHwPhyConfig();
	void RTL8402HwPhyConfig();
  void RTL8106EHwPhyConfig();
  
	void RTL8100NicReset();
	
	void RTL8100SetMedium(ushort speedIn, uchar duplexIn, uchar autonegIn);
  
	void RTL8100DSM(int dev_state);
  
	static const int eee_enable = 0;
	void RTL8100EnableEEE();
	void RTL8100DisableEEE();
	
	void RTL8100PowerDownPLL();
	void RTL8100PowerUpPLL();
	
	void RTL8100PowerDownPHY();
	void RTL8100PowerUpPHY();
  
	void RTL8100WritePhyIO(int RegAddr, int value);
	int RTL8100ReadPhyIO(int RegAddr);
	
	//
	// RTL8168
	//
	void RTL8168HwStart();
	// subroutines of above
	void RTL8168BHwStart2();
	void RTL8168CHwStart2();
	void RTL8168CPHwStart2();
	void RTL8168DHwStart2();
	void RTL8168DPHwStart2();
	void RTL8168EHwStart2();
	void RTL8168EVLHwStart2();
	void RTL8168FHwStart2();

  void rtl8168_get_bios_setting();
  void rtl8168_set_bios_setting();
	void rtl8168_init_software_variable();
	void RTL8168HwPhyConfig();
	// subroutines of above
	void RTL8168BHwPhyConfig();
	void RTL8168CHwPhyConfig();
	void RTL8168CPHwPhyConfig();
	void RTL8168DHwPhyConfig();
	void RTL8168DPHwPhyConfig();
  void RTL8168EHwPhyConfig();
  void RTL8168FHwPhyConfig();
  void RTL8168GHwPhyConfig();
  void RTL8411HwPhyConfig();
  
	void RTL8168NicReset();
	
	void RTL8168SleepRxEnable();
	
	void RTL8168SetMedium(ushort speedIn, uchar duplexIn, uchar autonegIn);
	
	void RTL8168DSM(int dev_state);
	
	void RTL8168PowerDownPLL();
	void RTL8168PowerUpPLL();
	
	void RTL8168PowerDownPHY();
	void RTL8168PowerUpPHY();
  
  u16 mac_ocp_read(u16 reg_addr);
  void mac_ocp_write(u16 reg_addr, u16 value);
  u32 OCP_read(u8 mask, u16 Reg);
  void OCP_write(u8 mask, u16 Reg, u32 data);
  
	void RTL8168WriteOCP_GPHY(int RegAddr, u16 value);
	u16 RTL8168ReadOCP_GPHY(int RegAddr);
  
	u8 RTL8168ReadEfuse(u16 reg);
  
	//
	// RTL8169
	//
	void RTL8169HwStart();
	void RTL8169HwPhyConfig();
	void RTL8169NicReset();
	
	void RTL8169SetMedium(ushort speedIn, uchar duplexIn, uchar autonegIn);
	
	void RTL8169PowerDownPHY();
	void RTL8169PowerUpPHY();
	
	
	ulong ether_crc(int length, unsigned char *data);
	
	//
	// Buffer pool allocation/deallocation
	//
	
	// Descriptor buffers
	bool AllocateDescriptorsMemory();
	void FreeDescriptorsMemory();
  
	// Packet buffers
	void InitializeBufferMemoryPointers();
	bool AllocateBufferMemory();
	void FreeBufferMemory();
#ifdef R1000_ORIGINAL_BUFFER_ALLOCATION
	inline IOPhysicalAddress	RxBufferPhysicalAddress(int n)	{ return Rx_skbuff_Dma[n]; }
	inline IOPhysicalAddress	TxBufferPhysicalAddress(int n)	{ return Tx_skbuff_Dma[n]; }
	inline uchar*				RxBufferVirtualAddress(int n)	{ return Rx_dbuff[n]; }
	inline uchar*				TxBufferVirtualAddress(int n)	{ return Tx_dbuff[n]; }
#endif
  
#ifdef R1000_NEW_BUFFER_ALLOCATION_1
	// FIXME - buffer sizes should be a variable, not a macro!
	inline IOPhysicalAddress	RxBufferPhysicalAddress(int n)
	{ return Rx_skbuff_pool_phys_addr + (n * MAX_RX_SKBDATA_SIZE); }
	inline IOPhysicalAddress	TxBufferPhysicalAddress(int n)
	{ return Tx_skbuff_pool_phys_addr + (n * MAX_TX_SKBDATA_SIZE); }
	inline uchar*				RxBufferVirtualAddress(int n)
	{ return reinterpret_cast<uchar*>(Rx_skbuff_pool_virt_addr + (n * MAX_RX_SKBDATA_SIZE)); }
	inline uchar*				TxBufferVirtualAddress(int n)
	{ return reinterpret_cast<uchar*>(Tx_skbuff_pool_virt_addr + (n * MAX_TX_SKBDATA_SIZE)); }
#endif
  
  void rtl8168_disable_pci_offset_99();
  void rtl8168_enable_pci_offset_99();
  void rtl8168_init_pci_offset_99();
  void rtl8168_disable_pci_offset_180();
  void rtl8168_enable_pci_offset_180();
  void rtl8168_init_pci_offset_180();
  
  void rtl8168_set_pci_99_180_exit_driver_para();
  void rtl8168_issue_offset_99_event();
  
  void rtl8168_hw_d3_para();
	
	void InitializeRingBufferDescriptors();
	void R1000InitRxDescCmds(bool nicOwn);
  
	
	bool R1000InitEventSources(IOService *provide);
	bool R1000OpenAdapter();
	void R1000CloseAdapter();
  
	void R1000RxInterrupt(u16 intStatus);
	void R1000TxInterrupt(u16 intStatus);
	void R1000PCIErrorInterrupt();
	
	bool OSAddNetworkMedium(ulong type, UInt32 bps, ulong index);
};

#endif
