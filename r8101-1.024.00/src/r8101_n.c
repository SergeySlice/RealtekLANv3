/*
################################################################################
#
# r8101 is the Linux device driver released for Realtek Fast Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2013 Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

/*
This driver is modified from r8169.c in Linux kernel 2.6.18
*/

#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "r8101.h"
#include "rtl_eeprom.h"
#include "rtl_ethtool.h"
#include "rtltool.h"

static int eee_enable = 0 ;
module_param(eee_enable, int, S_IRUGO);
/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static const int max_interrupt_work = 20;

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
   The RTL chips use a 64 element hash table based on the Ethernet CRC. */
static const int multicast_filter_limit = 32;

#define _R(NAME,MAC,MASK) \
	{ .name = NAME, .mcfg = MAC, .RxConfigMask = MASK }

static const struct {
    const char *name;
    u8 mcfg;
    u32 RxConfigMask;	/* Clears the bits supported by this chip */
} rtl_chip_info[] = {
    _R("RTL8101E", MCFG_8101E_1, 0xff7e1880),
    _R("RTL8101E", MCFG_8101E_2, 0xff7e1880),
    _R("RTL8101E", MCFG_8101E_3, 0xff7e1880),
    _R("RTL8102E", MCFG_8102E_1, 0xff7e1880),
    _R("RTL8102E", MCFG_8102E_2, 0xff7e1880),
    _R("RTL8103E", MCFG_8103E_1, 0xff7e1880),
    _R("RTL8103E", MCFG_8103E_2, 0xff7e1880),
    _R("RTL8103E", MCFG_8103E_3, 0xff7e1880),
    _R("RTL8401E", MCFG_8401_1,  0xff7e1880),
    _R("RTL8105E", MCFG_8105E,   0xff7e1880),
    _R("RTL8105E", MCFG_8105E_2, 0xff7e1880),
    _R("RTL8105E", MCFG_8105E_3, 0xff7e1880),
    _R("RTL8105E", MCFG_8105E_4, 0xff7e1880),
    _R("RTL8402",  MCFG_8402_1,  0xff7e1880),
    _R("RTL8106E", MCFG_8106E_1, 0xff7e1880),
    _R("RTL8106E", MCFG_8106E_2, 0xff7e1880),
    _R("RTL8106EUS", MCFG_8106EUS, 0xff7e1880)
};
#undef _R

static struct pci_device_id rtl8101_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8136), },
    {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8101_pci_tbl);

static int rx_copybreak = 200;
static int timer_count = 0x2600;
static int use_dac;
static struct {
    u32 msg_enable;
} debug = { -1 };

static unsigned short speed = SPEED_100;
static int duplex = DUPLEX_FULL;
static int autoneg = AUTONEG_ENABLE;
#ifdef CONFIG_ASPM
static int aspm = 1;
#else
static int aspm = 0;
#endif
#ifdef ENABLE_S5WOL
static int s5wol = 1;
#else
static int s5wol = 0;
#endif


MODULE_AUTHOR("Realtek and the Linux r8101 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("RealTek RTL-8101 Fast Ethernet driver");

module_param(speed, ushort, 0);
MODULE_PARM_DESC(speed, "force phy operation. Deprecated by ethtool (8).");

module_param(duplex, int, 0);
MODULE_PARM_DESC(duplex, "force phy operation. Deprecated by ethtool (8).");

module_param(autoneg, int, 0);
MODULE_PARM_DESC(autoneg, "force phy operation. Deprecated by ethtool (8).");

module_param(aspm, int, 0);
MODULE_PARM_DESC(aspm, "Enable ASPM.");

module_param(s5wol, int, 0);
MODULE_PARM_DESC(s5wol, "Enable Shutdown Wake On Lan.");

module_param(rx_copybreak, int, 0);
MODULE_PARM_DESC(rx_copybreak, "Copy breakpoint for copy-only-tiny-frames");

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

MODULE_LICENSE("GPL");

MODULE_VERSION(RTL8101_VERSION);

static void rtl8101_dsm(struct net_device *dev, int dev_state);

static void rtl8101_esd_timer(unsigned long __opaque);

static void rtl8101_hw_phy_config(struct net_device *dev);

static void rtl8101_wait_for_quiescence(struct net_device *dev);

static void rtl8101_tx_clear(struct rtl8101_private *tp);
static void rtl8101_rx_clear(struct rtl8101_private *tp);

static void rtl8101_link_timer(unsigned long __opaque);
static void rtl8101_aspm_fix1(struct net_device *dev);

static int rtl8101_open(struct net_device *dev);
static int rtl8101_start_xmit(struct sk_buff *skb, struct net_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance, struct pt_regs *regs);
#else
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance);
#endif
static int rtl8101_init_ring(struct net_device *dev);
static void rtl8101_hw_start(struct net_device *dev);
static int rtl8101_close(struct net_device *dev);
static void rtl8101_set_rx_mode(struct net_device *dev);
static void rtl8101_tx_timeout(struct net_device *dev);
static struct net_device_stats *rtl8101_get_stats(struct net_device *dev);
static int rtl8101_rx_interrupt(struct net_device *, struct rtl8101_private *, void __iomem *, u32 budget);
static int rtl8101_change_mtu(struct net_device *dev, int new_mtu);
static void rtl8101_down(struct net_device *dev);

static int rtl8101_set_speed(struct net_device *dev, u8 autoneg, u16 speed, u8 duplex);
static int rtl8101_set_mac_address(struct net_device *dev, void *p);
void rtl8101_rar_set(struct rtl8101_private *tp, uint8_t *addr);
static void rtl8101_desc_addr_fill(struct rtl8101_private *);
static void rtl8101_tx_desc_init(struct rtl8101_private *tp);
static void rtl8101_rx_desc_init(struct rtl8101_private *tp);

static void rtl8101_hw_reset(struct net_device *dev);
static void rtl8101_phy_power_down (struct net_device *dev);
static void rtl8101_phy_power_up (struct net_device *dev);

#ifdef CONFIG_R8101_NAPI
static int rtl8101_poll(napi_ptr napi, napi_budget budget);
#endif

static const unsigned int rtl8101_rx_config =
    (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,3)
/* copied from linux kernel 2.6.20 include/linux/netdev.h */
#define	NETDEV_ALIGN		32
#define	NETDEV_ALIGN_CONST	(NETDEV_ALIGN - 1)

static inline void *netdev_priv(struct net_device *dev)
{
    return (char *)dev + ((sizeof(struct net_device)
                           + NETDEV_ALIGN_CONST)
                          & ~NETDEV_ALIGN_CONST);
}
#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,3)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)
/* copied from linux kernel 2.6.20 include/linux/netdevice.h */
static inline u32 netif_msg_init(int debug_value, int default_msg_enable_bits)
{
    /* use default */
    if (debug_value < 0 || debug_value >= (sizeof(u32) * 8))
        return default_msg_enable_bits;
    if (debug_value == 0)	/* no output */
        return 0;
    /* set low N bits */
    return (1 << debug_value) - 1;
}

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
static inline void eth_copy_and_sum (struct sk_buff *dest,
                                     const unsigned char *src,
                                     int len, int base)
{
    memcpy (dest->data, src, len);
}
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
/* copied from linux kernel 2.6.20 /include/linux/time.h */
/* Parameters used to convert the timespec values: */
#define MSEC_PER_SEC	1000L

/* copied from linux kernel 2.6.20 /include/linux/jiffies.h */
/*
 * Change timeval to jiffies, trying to avoid the
 * most obvious overflows..
 *
 * And some not so obvious.
 *
 * Note that we don't want to return MAX_LONG, because
 * for various timeout reasons we often end up having
 * to wait "jiffies+1" in order to guarantee that we wait
 * at _least_ "jiffies" - so "jiffies+1" had better still
 * be positive.
 */
#define MAX_JIFFY_OFFSET ((~0UL >> 1)-1)

/*
 * Convert jiffies to milliseconds and back.
 *
 * Avoid unnecessary multiplications/divisions in the
 * two most common HZ cases:
 */
static inline unsigned int jiffies_to_msecs(const unsigned long j)
{
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
    return (MSEC_PER_SEC / HZ) * j;
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
    return (j + (HZ / MSEC_PER_SEC) - 1)/(HZ / MSEC_PER_SEC);
#else
    return (j * MSEC_PER_SEC) / HZ;
#endif
}

static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
    if (m > jiffies_to_msecs(MAX_JIFFY_OFFSET))
        return MAX_JIFFY_OFFSET;
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
    return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
    return m * (HZ / MSEC_PER_SEC);
#else
    return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
#endif
}
#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
//for linux kernel 2.6.10 and earlier.

/* copied from linux kernel 2.6.12.6 /include/linux/pm.h */
typedef int __bitwise pci_power_t;

/* copied from linux kernel 2.6.12.6 /include/linux/pci.h */
typedef u32 __bitwise pm_message_t;

#define PCI_D0	((pci_power_t __force) 0)
#define PCI_D1	((pci_power_t __force) 1)
#define PCI_D2	((pci_power_t __force) 2)
#define PCI_D3hot	((pci_power_t __force) 3)
#define PCI_D3cold	((pci_power_t __force) 4)
#define PCI_POWER_ERROR	((pci_power_t __force) -1)

/* copied from linux kernel 2.6.12.6 /drivers/pci/pci.c */
/**
 * pci_choose_state - Choose the power state of a PCI device
 * @dev: PCI device to be suspended
 * @state: target sleep state for the whole system. This is the value
 *	that is passed to suspend() function.
 *
 * Returns PCI power state suitable for given device and given system
 * message.
 */

pci_power_t pci_choose_state(struct pci_dev *dev, pm_message_t state)
{
    if (!pci_find_capability(dev, PCI_CAP_ID_PM))
        return PCI_D0;

    switch (state) {
    case 0:
        return PCI_D0;
    case 3:
        return PCI_D3hot;
    default:
        printk("They asked me for state %d\n", state);
//		BUG();
    }
    return PCI_D0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
//+porting on 2.6.8.1 and earlier
/**
 * msleep_interruptible - sleep waiting for waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
unsigned long msleep_interruptible(unsigned int msecs)
{
    unsigned long timeout = msecs_to_jiffies(msecs);

    while (timeout && !signal_pending(current)) {
        set_current_state(TASK_INTERRUPTIBLE);
        timeout = schedule_timeout(timeout);
    }
    return jiffies_to_msecs(timeout);
}

/* copied from linux kernel 2.6.20 include/linux/mii.h */
#undef if_mii
#define if_mii _kc_if_mii
static inline struct mii_ioctl_data *if_mii(struct ifreq *rq) {
    return (struct mii_ioctl_data *) &rq->ifr_ifru;
}
#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

static inline u16 map_phy_ocp_addr(u16 PageNum, u8 RegNum)
{
    u16 OcpPageNum = 0;
    u8 OcpRegNum = 0;
    u16 OcpPhyAddress = 0;

    if( PageNum == 0 ) {
        OcpPageNum = OCP_STD_PHY_BASE_PAGE + ( RegNum / 8 );
        OcpRegNum = 0x10 + ( RegNum % 8 );
    } else {
        OcpPageNum = PageNum;
        OcpRegNum = RegNum;
    }

    OcpPageNum <<= 4;

    if( OcpRegNum < 16 ) {
        OcpPhyAddress = 0;
    } else {
        OcpRegNum -= 16;
        OcpRegNum <<= 1;

        OcpPhyAddress = OcpPageNum + OcpRegNum;
    }


    return OcpPhyAddress;
}

void mdio_write(struct rtl8101_private *tp,
                u32 RegAddr,
                u32 value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    if (RegAddr == 0x1F) {
        tp->cur_page = value;
    }

    if (mcfg == MCFG_8168E_VL_2) {
        u32 data32;
        u16 ocp_addr;

        if (RegAddr == 0x1F) {
            return;
        }
        ocp_addr = map_phy_ocp_addr(tp->cur_page, RegAddr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(ocp_addr % 2);
#endif
        data32 = ocp_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 |= OCPR_Write | value;

        WriteMMIO32(PHYOCP, data32);
        for (i = 0; i < 100; i++) {
            IODelay(1);

            if (!(ReadMMIO32(PHYOCP) & OCPR_Flag))
                break;
        }
    } else {
        WriteMMIO32(PHYAR, PHYAR_Write |
                (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift |
                (value & PHYAR_Data_Mask));

        for (i = 0; i < 10; i++) {
            IODelay(100);

            /* Check if the RTL8101 has completed writing to the specified MII register */
            if (!(ReadMMIO32(PHYAR) & PHYAR_Flag)) {
                IODelay(20);
                break;
            }
        }
    }
}

u32 mdio_read(struct rtl8101_private *tp,
              u32 RegAddr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i, value = 0;

    if (mcfg == MCFG_8168E_VL_2) {
        u32 data32;
        u16 ocp_addr;

        ocp_addr = map_phy_ocp_addr(tp->cur_page, RegAddr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(ocp_addr % 2);
#endif
        data32 = ocp_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;

        WriteMMIO32(PHYOCP, data32);
        for (i = 0; i < 100; i++) {
            IODelay(1);

            if (ReadMMIO32(PHYOCP) & OCPR_Flag)
                break;
        }
        value = ReadMMIO32(PHYOCP) & OCPDR_Data_Mask;
    } else {
        WriteMMIO32(PHYAR,
                PHYAR_Read | (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift);

        for (i = 0; i < 10; i++) {
            IODelay(100);

            /* Check if the RTL8101 has completed retrieving data from the specified MII register */
            if (ReadMMIO32(PHYAR) & PHYAR_Flag) {
                value = ReadMMIO32(PHYAR) & PHYAR_Data_Mask;
                IODelay(20);
                break;
            }
        }
    }

    return value;
}

void mac_ocp_write(struct rtl8101_private *tp, u16 reg_addr, u16 value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 data32;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
    WARN_ON_ONCE(reg_addr % 2);
#endif

    data32 = reg_addr/2;
    data32 <<= OCPR_Addr_Reg_shift;
    data32 += value;
    data32 |= OCPR_Write;

    WriteMMIO32(MACOCP, data32);
}

u16 mac_ocp_read(struct rtl8101_private *tp, u16 reg_addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 data32;
    u16 data16 = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
    WARN_ON_ONCE(reg_addr % 2);
#endif

    data32 = reg_addr/2;
    data32 <<= OCPR_Addr_Reg_shift;

    WriteMMIO32(MACOCP, data32);
    data16 = (u16)ReadMMIO32(MACOCP);

    return data16;
}


static void
rtl8101_phyio_write(void __iomem *ioaddr,
                    int RegAddr,
                    int value)
{
    int i;

    WriteMMIO32(PHYIO, PHYIO_Write |
            (RegAddr & PHYIO_Reg_Mask) << PHYIO_Reg_shift |
            (value & PHYIO_Data_Mask));

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8101 has completed writing to the specified MII register */
        if (!(ReadMMIO32(PHYIO) & PHYIO_Flag))
            break;
    }

    IODelay(100);
}

#if 0
static int
rtl8101_phyio_read(void __iomem *ioaddr,
                   int RegAddr)
{
    int i, value = -1;

    WriteMMIO32(PHYIO,
            PHYIO_Read | (RegAddr & PHYIO_Reg_Mask) << PHYIO_Reg_shift);

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8101 has completed retrieving data from the specified MII register */
        if (ReadMMIO32(PHYIO) & PHYIO_Flag) {
            value = (int) (ReadMMIO32(PHYIO) & PHYIO_Data_Mask);
            break;
        }
    }

    IODelay(100);

    return value;
}
#endif

void rtl8101_ephy_write(void __iomem *ioaddr, u32 RegAddr, u32 value)
{
    int i;

    WriteMMIO32(EPHYAR,
            EPHYAR_Write |
            (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift |
            (value & EPHYAR_Data_Mask));

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8101 has completed EPHY write */
        if (!(ReadMMIO32(EPHYAR) & EPHYAR_Flag))
            break;
    }

    IODelay(20);
}

u16 rtl8101_ephy_read(void __iomem *ioaddr, u32 RegAddr)
{
    int i;
    u16 value = 0xffff;

    WriteMMIO32(EPHYAR,
            EPHYAR_Read | (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift);

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8101 has completed EPHY read */
        if (ReadMMIO32(EPHYAR) & EPHYAR_Flag) {
            value = (u16) (ReadMMIO32(EPHYAR) & EPHYAR_Data_Mask);
            break;
        }
    }

    IODelay(20);

    return value;
}

static void
rtl8101_csi_write(struct rtl8101_private *tp,
                  int addr,
                  int value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 cmd;
    int i;

    WriteMMIO32(CSIDR, value);
    cmd = CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);
    if (mcfg == MCFG_8402_1)
        cmd |= 0x00020000;
    WriteMMIO32(CSIAR, cmd);

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8101 has completed CSI write */
        if (!(ReadMMIO32(CSIAR) & CSIAR_Flag))
            break;
    }
}

static int
rtl8101_csi_read(struct rtl8101_private *tp,
                 int addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 cmd;
    int i, value = -1;

    cmd = CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);

    if (mcfg == MCFG_8402_1)
        cmd |= 0x00020000;

    WriteMMIO32(CSIAR, cmd);

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8101 has completed CSI read */
        if (ReadMMIO32(CSIAR) & CSIAR_Flag) {
            value = (int)ReadMMIO32(CSIDR);
            break;
        }
    }
    return value;
}

static u32 rtl8101_eri_read(void __iomem *ioaddr, int addr, int len, int type)
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

            /* Check if the RTL8101 has completed ERI read */
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
        else {
            len -= (4 - val_shift);
            shift = 4 - val_shift;
            addr += 4;
        }
    }

    return value2;
}

static int rtl8101_eri_write(void __iomem *ioaddr, int addr, int len, u32 value, int type)
{

    int i, val_shift, shift = 0;
    u32 value1 = 0, mask;

    if (len > 4 || len <= 0)
        return -1;

    while (len > 0) {
        val_shift = addr % ERIAR_Addr_Align;
        addr = addr & ~0x3;

        if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

        value1 = ReadERI(addr, 4, type) & ~mask;
        value1 |= ((value << val_shift * 8) >> shift * 8);

        WriteMMIO32(ERIDR, value1);
        WriteMMIO32(ERIAR,
                ERIAR_Write |
                type << ERIAR_Type_shift |
                ERIAR_ByteEn << ERIAR_ByteEn_shift |
                addr);

        for (i = 0; i < 10; i++) {
            IODelay(100);

            /* Check if the RTL8101 has completed ERI write */
            if (!(ReadMMIO32(ERIAR) & ERIAR_Flag))
                break;
        }

        if (len <= 4 - val_shift)
            len = 0;
        else {
            len -= (4 - val_shift);
            shift = 4 - val_shift;
            addr += 4;
        }
    }

    return 0;
}

#if 1

static int rtl8101_enable_EEE(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int	ret;
    unsigned long flags;
    __u16	data;

    ret = 0;
    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
        WriteERI(0x1B0, 2, 0xED03, ERIAR_ExGMAC);
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0004);
        if (ReadMMIO8(0xEF) & 0x02) {
            WriteGMII16( 0x10, 0x731F);
            WriteGMII16( 0x19, 0x7630);
        } else {
            WriteGMII16( 0x10, 0x711F);
            WriteGMII16( 0x19, 0x7030);
        }
        WriteGMII16( 0x1A, 0x1506);
        WriteGMII16( 0x1B, 0x0551);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0002);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0003);
        WriteGMII16( 0x0E, 0x0015);
        WriteGMII16( 0x0D, 0x4003);
        WriteGMII16( 0x0E, 0x0002);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
        WriteERI(0x1B0, 2, 0xED03, ERIAR_ExGMAC);
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x10, 0x731F);
        WriteGMII16( 0x19, 0x7630);
        WriteGMII16( 0x1A, 0x1506);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0002);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8168E_VL_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        data = ReadERI(0x1B0, 4, ERIAR_ExGMAC);
        data |= BIT_1 | BIT_0;
        WriteERI(0x1B0, 4, data, ERIAR_ExGMAC);
        WriteGMII16( 0x1F, 0x0A43);
        data = ReadGMII16( 0x11);
        WriteGMII16( 0x11, data | BIT_4);
        WriteGMII16( 0x1F, 0x0A5D);
        WriteGMII16( 0x10, 0x0006);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    default:
        //dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support EEE\n");
        ret = -EOPNOTSUPP;
        break;
    }

    /*Advanced EEE*/
    switch (mcfg) {
    case MCFG_8168E_VL_2:
        data = ReadERI(0x1EA, 1, ERIAR_ExGMAC);
        data |= 0xFA;
        WriteERI(0x1EA, 1, data, ERIAR_ExGMAC);

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0A43);
        data = ReadGMII16( 0x10);
        if (data & BIT_10) {
            WriteGMII16( 0x1F, 0x0A42);
            data = ReadGMII16( 0x16);
            data &= ~(BIT_1);
            WriteGMII16( 0x16, data);
        } else {
            WriteGMII16( 0x1F, 0x0A42);
            data = ReadGMII16( 0x16);
            data |= BIT_1;
            WriteGMII16( 0x16, data);
        }
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;
    }


    return ret;
}

static int rtl8101_disable_EEE(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int	ret;
    unsigned long flags;
    __u16	data;

    ret = 0;
    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
        WriteERI(0x1B0, 2, 0, ERIAR_ExGMAC);
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x10, 0x401F);
        WriteGMII16( 0x19, 0x7030);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0003);
        WriteGMII16( 0x0E, 0x0015);
        WriteGMII16( 0x0D, 0x4003);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8402_1:
        WriteERI(0x1B0, 2, 0, ERIAR_ExGMAC);
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x10, 0x401F);
        WriteGMII16( 0x19, 0x7030);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
        WriteERI(0x1B0, 2, 0, ERIAR_ExGMAC);
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x10, 0xC07F);
        WriteGMII16( 0x19, 0x7030);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);

        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8168E_VL_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        data = ReadERI(0x1B0, 4, ERIAR_ExGMAC);
        data &= ~(BIT_1 | BIT_0);
        WriteERI(0x1B0, 4, data, ERIAR_ExGMAC);
        WriteGMII16( 0x1F, 0x0A43);
        data = ReadGMII16( 0x11);
        WriteGMII16( 0x11, data & ~BIT_4);
        WriteGMII16( 0x1F, 0x0A5D);
        WriteGMII16( 0x10, 0x0000);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    default:
        //dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support EEE\n");
        ret = -EOPNOTSUPP;
        break;
    }

    /*Advanced EEE*/
    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteERI(0x1EA, 1, 0x00, ERIAR_ExGMAC);

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0A42);
        data = ReadGMII16( 0x16);
        data &= ~(BIT_1);
        WriteGMII16( 0x16, data);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;
    }

    return ret;
}

#endif

static void
rtl8101_enable_rxdvgate(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_3);
        mdelay(2);
        break;
    }
}

static void
rtl8101_disable_rxdvgate(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) & ~BIT_3);
        mdelay(2);
        break;
    }
}

static void
rtl8101_wait_txrx_fifo_empty(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        for (i = 0; i < 10; i++) {
            IODelay(100);
            if (ReadMMIO32(TxConfig) & BIT_11)
                break;
        }

        if(ReadMMIO8(ChipCmd) & (CmdTxEnb | CmdRxEnb)) {
            mdelay(1);
            WriteMMIO8(ChipCmd, ReadMMIO8(ChipCmd) & ~(CmdTxEnb | CmdRxEnb));
        }

        for (i = 0; i < 10; i++) {
            IODelay(100);
            if ((ReadMMIO8(MCUCmd_reg) & (Txfifo_empty | Rxfifo_empty)) == (Txfifo_empty | Rxfifo_empty))
                break;

        }
        break;
    }
}

static void
rtl8101_irq_mask_and_ack(void __iomem *ioaddr)
{
    WriteMMIO16(IntrMask, 0x0000);
    WriteMMIO16(IntrStatus, ReadMMIO16(IntrStatus));
}

static void
rtl8101_nic_reset(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    WriteMMIO32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));

    rtl8101_enable_rxdvgate(dev);

    rtl8101_wait_txrx_fifo_empty(dev);

    switch (mcfg) {
    case CFG_METHOD_1:
    case CFG_METHOD_2:
    case CFG_METHOD_3:
        WriteMMIO8(ChipCmd, StopReq | CmdRxEnb | CmdTxEnb);
        IODelay(100);
        break;
    case MCFG_8402_1:
    case MCFG_8168E_VL_2:
        mdelay(2);
        break;
    default:
        mdelay(10);
        break;
    }

    /* Soft reset the chip. */
    WriteMMIO8(ChipCmd, CmdReset);

    /* Check that the chip has finished the reset. */
    for (i = 100; i > 0; i--) {
        if ((ReadMMIO8(ChipCmd) & CmdReset) == 0)
            break;
        IODelay(100);
    }
}

static void
rtl8101_hw_reset(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    /* Disable interrupts */
    rtl8101_irq_mask_and_ack(ioaddr);

    rtl8101_nic_reset(dev);
}

static unsigned int
rtl8101_xmii_reset_pending(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    unsigned int retval;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1f, 0x0000);
    retval = ReadGMII16( MII_BMCR) & BMCR_RESET;
    spin_unlock_irqrestore(&tp->phy_lock, flags);

    return retval;
}

static unsigned int
rtl8101_xmii_link_ok(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned int retval;

    retval = (ReadMMIO8(PHYstatus) & LinkStatus) ? 1 : 0;

    return retval;
}

static void
rtl8101_xmii_reset_enable(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    int i;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1f, 0x0000);
    WriteGMII16( MII_BMCR, ReadGMII16( MII_BMCR) | BMCR_RESET);
    for (i = 0; i < 2500; i++) {
        if (!(ReadGMII16( MII_BMSR) & BMCR_RESET))
            break;

        mdelay(1);
    }
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
set_offset70F(struct rtl8101_private *tp, u8 setting)
{
    u32 csi_tmp;
    u32 temp = (u32)setting;
    temp = temp << 24;
    /*set PCI configuration space offset 0x70F to setting*/
    /*When the register offset of PCI configuration space larger than 0xff, use CSI to access it.*/

    csi_tmp = rtl8101_csi_read(tp, 0x70c) & 0x00ffffff;
    rtl8101_csi_write(tp, 0x70c, csi_tmp | temp);
}

static void
set_offset79(struct rtl8101_private *tp, u8 setting)
{
    //Set PCI configuration space offset 0x79 to setting

    struct pci_dev *pdev = tp->pci_dev;
    u8 device_control;

    pci_read_config_byte(pdev, 0x79, &device_control);
    device_control &= ~0x70;
    device_control |= setting;
    pci_write_config_byte(pdev, 0x79, device_control);

}

static void
rtl8101_init_ring_indexes(struct rtl8101_private *tp)
{
    tp->dirty_tx = 0;
    tp->dirty_rx = 0;
    tp->cur_tx = 0;
    tp->cur_rx = 0;
}

static void
rtl8101_check_link_status(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int link_status_on;
    u32 data32;
    unsigned long flags;

    link_status_on = tp->link_ok(dev);

    if (netif_carrier_ok(dev) != link_status_on) {

        if (link_status_on) {
            if (mcfg == CFG_METHOD_5 || mcfg == CFG_METHOD_6 ||
                mcfg == CFG_METHOD_7 || mcfg == CFG_METHOD_8)
                set_offset70F(tp, 0x3F);

            if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
                mcfg == MCFG_8168DP_3) {
                if ((ReadMMIO8(PHYstatus) & FullDup) == 0 && eee_enable == 1)
                    rtl8101_disable_EEE(tp);

                if (ReadMMIO8(PHYstatus) & _10bps) {
                    WriteERI(0x1D0, 2, 0x4D02, ERIAR_ExGMAC);
                    WriteERI(0x1DC, 2, 0x0060, ERIAR_ExGMAC);

                    WriteERI(0x1B0, 2, 0, ERIAR_ExGMAC);
                    spin_lock_irqsave(&tp->phy_lock, flags);
                    mdio_write( tp, 0x1F, 0x0004);
                    data32 = mdio_read( tp, 0x10);
                    data32 |= 0x0400;
                    data32 &= ~0x0800;
                    WriteGMII16( 0x10, data32);
                    WriteGMII16( 0x1F, 0x0000);
                    spin_unlock_irqrestore(&tp->phy_lock, flags);
                } else {
                    WriteERI(0x1D0, 2, 0, ERIAR_ExGMAC);
                    if ( eee_enable == 1 && (ReadMMIO8(0xEF) & BIT_0) == 0)
                        WriteERI(0x1B0, 2, 0xED03, ERIAR_ExGMAC);
                }
            } else if (mcfg == MCFG_8402_1 || mcfg == MCFG_8106E_1 ||
                       mcfg == MCFG_8168E_VL_1) {
                if (ReadMMIO8(PHYstatus) & _10bps) {
                    WriteERI(0x1D0, 2, 0x4d02, ERIAR_ExGMAC);
                    WriteERI(0x1DC, 2, 0x0060, ERIAR_ExGMAC);
                } else {
                    WriteERI(0x1D0, 2, 0, ERIAR_ExGMAC);
                }
            }

            rtl8101_hw_start(dev);

            netif_carrier_on(dev);

            if (netif_msg_ifup(tp))
                printk(KERN_INFO PFX "%s: link up\n", dev->name);
        } else {
            if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
                mcfg == MCFG_8168DP_3) {
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write( tp, 0x1F, 0x0004);
                data32 = mdio_read( tp, 0x10);
                data32 &= ~0x0C00;
                WriteGMII16( 0x1F, 0x0000);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
            if (netif_msg_ifdown(tp))
                printk(KERN_INFO PFX "%s: link down\n", dev->name);
            netif_carrier_off(dev);

            netif_stop_queue(dev);

            rtl8101_hw_reset(dev);

            rtl8101_tx_clear(tp);

            rtl8101_init_ring_indexes(tp);

            rtl8101_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);

            if (mcfg == CFG_METHOD_5 || mcfg == CFG_METHOD_6 ||
                mcfg == CFG_METHOD_7 || mcfg == CFG_METHOD_8)
                set_offset70F(tp, 0x17);
        }
    }
    switch (mcfg) {
    case CFG_METHOD_4:
        rtl8101_aspm_fix1(dev);
        break;
    }
}

static void
rtl8101_wait_ll_share_fifo_ready(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    for (i = 0; i < 10; i++) {
        IODelay(100);
        if (ReadMMIO16(0xD2) & BIT_9)
            break;
    }
}

static void
rtl8101_hw_d3_para(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    switch (mcfg) {
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        break;
    }

    if ((mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
         mcfg == MCFG_8168DP_3) && (eee_enable == 1))
        rtl8101_disable_EEE(tp);

    if (mcfg == MCFG_8168E_VL_2) {
        u32 csi_tmp;

        csi_tmp = ReadERI(0x3F2, 2, ERIAR_ExGMAC);
        csi_tmp &= ~(BIT_0 | BIT_1);
        WriteERI(0x3F2, 2, csi_tmp, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0x1E2, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_2;
        WriteERI(0x1E2, 1, csi_tmp, ERIAR_ExGMAC);

        WriteERI(0x2F8, 2, 0x0064, ERIAR_ExGMAC);

        /*disable ocp phy power saving*/
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0C41);
        WriteGMII16( 0x13, 0x0000);
        WriteGMII16( 0x13, 0x0500);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }

    if (bios_setting & BIT_28) {
        if (mcfg == MCFG_8168DP_3) {
            if (!(ReadMMIO8(0xEF) & BIT_2)) {
                u32 gphy_val;

                spin_lock_irqsave(&tp->phy_lock, flags);
                WriteGMII16( 0x1F, 0x0000);
                WriteGMII16( 0x04, 0x0061);
                WriteGMII16( 0x00, 0x1200);
                WriteGMII16( 0x18, 0x0310);
                mdelay(20);
                WriteGMII16( 0x1F, 0x0005);
                gphy_val = ReadGMII16( 0x1a);
                gphy_val |= BIT_8 | BIT_0;
                WriteGMII16( 0x1a, gphy_val);
                mdelay(30);
                WriteGMII16( 0x1f, 0x0000);
                WriteGMII16( 0x18, 0x8310);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
        }
    }

    rtl8101_disable_rxdvgate(dev);
}

static void
rtl8101_powerdown_pll(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    if (tp->wol_enabled == WOL_ENABLED) {
        int auto_nego;
        u16 val;
        unsigned long flags;

        if (mcfg == MCFG_8168E_VL_2) {
            WriteMMIO8(Cfg9346, Cfg9346_Unlock);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | PMSTS_En);
            WriteMMIO8(Cfg9346, Cfg9346_Lock);
        }
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0000);
        auto_nego = ReadGMII16( PHY_AUTO_NEGO_REG);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                       | ADVERTISE_100HALF | ADVERTISE_100FULL);

        val = ReadGMII16( MII_LPA);

        if (val & (LPA_10HALF | LPA_10FULL))
            auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL);
        else
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

        WriteGMII16( PHY_AUTO_NEGO_REG, auto_nego);
        WriteGMII16( MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);

        switch (mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        case CFG_METHOD_4:
        case CFG_METHOD_5:
        case CFG_METHOD_6:
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_9:
            break;
        default:
            WriteMMIO32(RxConfig, ReadMMIO32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
            break;
        }

        return;
    }

    rtl8101_phy_power_down(dev);

    switch (mcfg) {
    case CFG_METHOD_6:
    case CFG_METHOD_9:
        WriteMMIO8(DBG_reg, ReadMMIO8(DBG_reg) | BIT_3);
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) & ~BIT_7);
        break;

    case CFG_METHOD_8:
        pci_write_config_byte(tp->pci_dev, 0x81, 0);
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) & ~BIT_7);
        break;
    case CFG_METHOD_7:
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) & ~BIT_7);
        break;
    default:
        break;
    }
}

static void
rtl8101_powerup_pll(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case CFG_METHOD_6:
    case CFG_METHOD_9:
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) | BIT_7);
        WriteMMIO8(DBG_reg, ReadMMIO8(DBG_reg) & ~BIT_3);
        break;
    case CFG_METHOD_7:
    case CFG_METHOD_8:
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) | BIT_7);
        break;
    }

    rtl8101_phy_power_up(dev);
}


static void
rtl8101_link_option(int idx,
                    u8 *aut,
                    u16 *spd,
                    u8 *dup)
{
    if ((*spd != SPEED_100) && (*spd != SPEED_10))
        *spd = SPEED_100;

    if ((*dup != DUPLEX_FULL) && (*dup != DUPLEX_HALF))
        *dup = DUPLEX_FULL;

    if ((*aut != AUTONEG_ENABLE) && (*aut != AUTONEG_DISABLE))
        *aut = AUTONEG_ENABLE;
}

static void
rtl8101_get_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 options;
    u32 csi_tmp;
    unsigned long flags;

    wol->wolopts = 0;

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)
    wol->supported = WAKE_ANY;

    spin_lock_irqsave(&tp->lock, flags);

    options = ReadMMIO8(Config1);
    if (!(options & PMEnable))
        goto out_unlock;

    options = ReadMMIO8(Config3);
    if (options & LinkUp)
        wol->wolopts |= WAKE_PHY;

    switch (mcfg) {
    case MCFG_8402_1:
    case MCFG_8168E_VL_2:
        csi_tmp = ReadERI(0xDE, 1, ERIAR_ExGMAC);
        if (csi_tmp & BIT_0)
            wol->wolopts |= WAKE_MAGIC;
        break;
    default:
        if (options & MagicPacket)
            wol->wolopts |= WAKE_MAGIC;
        break;
    }

    options = ReadMMIO8(Config5);
    if (options & UWF)
        wol->wolopts |= WAKE_UCAST;
    if (options & BWF)
        wol->wolopts |= WAKE_BCAST;
    if (options & MWF)
        wol->wolopts |= WAKE_MCAST;

out_unlock:
    spin_unlock_irqrestore(&tp->lock, flags);
}

static int
rtl8101_set_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i,tmp;
    u32 csi_tmp;
    static struct {
        u32 opt;
        u16 reg;
        u8  mask;
    } cfg[] = {
        { WAKE_ANY,   Config1, PMEnable },
        { WAKE_PHY,   Config3, LinkUp },
        { WAKE_UCAST, Config5, UWF },
        { WAKE_BCAST, Config5, BWF },
        { WAKE_MCAST, Config5, MWF },
        { WAKE_ANY,   Config5, LanWake },
        { WAKE_MAGIC, Config3, MagicPacket }
    };
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    WriteMMIO8(Cfg9346, Cfg9346_Unlock);

    switch (mcfg) {
    case MCFG_8402_1:
    case MCFG_8168E_VL_2:
        tmp = ARRAY_SIZE(cfg) - 1;

        csi_tmp = ReadERI(0xDE, 1, ERIAR_ExGMAC);
        if (wol->wolopts & WAKE_MAGIC)
            csi_tmp |= BIT_0;
        else
            csi_tmp &= ~BIT_0;
        WriteERI(0xDE, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    default:
        tmp = ARRAY_SIZE(cfg);
        break;
    }

    for (i = 0; i < tmp; i++) {
        u8 options = ReadMMIO8(cfg[i].reg) & ~cfg[i].mask;
        if (wol->wolopts & cfg[i].opt)
            options |= cfg[i].mask;
        WriteMMIO8(cfg[i].reg, options);
    }

    WriteMMIO8(Cfg9346, Cfg9346_Lock);

    tp->wol_enabled = (wol->wolopts) ? WOL_ENABLED : WOL_DISABLED;

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}

static void
rtl8101_get_drvinfo(struct net_device *dev,
                    struct ethtool_drvinfo *info)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    strcpy(info->driver, MODULENAME);
    strcpy(info->version, RTL8101_VERSION);
    strcpy(info->bus_info, pci_name(tp->pci_dev));
}

static int
rtl8101_get_regs_len(struct net_device *dev)
{
    return R8101_REGS_SIZE;
}

static int
rtl8101_set_speed_xmii(struct net_device *dev,
                       u8 autoneg,
                       u16 speed,
                       u8 duplex)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    int auto_nego = 0;
    int bmcr_true_force = 0;
    unsigned long flags;

    if ((speed != SPEED_100) &&
        (speed != SPEED_10)) {
        speed = SPEED_100;
        duplex = DUPLEX_FULL;
    }

    auto_nego = ReadGMII16( PHY_AUTO_NEGO_REG);

    auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL | PHY_Cap_PAUSE | PHY_Cap_ASYM_PAUSE);

    if (autoneg == AUTONEG_ENABLE) {
        /*n-way force*/
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            auto_nego |= ADVERTISE_10HALF;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            auto_nego |= ADVERTISE_10HALF |
                         ADVERTISE_10FULL;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            auto_nego |= ADVERTISE_100HALF |
                         ADVERTISE_10HALF |
                         ADVERTISE_10FULL;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            auto_nego |= ADVERTISE_100HALF |
                         ADVERTISE_100FULL |
                         ADVERTISE_10HALF |
                         ADVERTISE_10FULL;
        }

        //flow contorol
        auto_nego |= PHY_Cap_PAUSE|PHY_Cap_ASYM_PAUSE;

        if ((mcfg == CFG_METHOD_4) || (mcfg == CFG_METHOD_5) ||
            (mcfg == CFG_METHOD_6) || (mcfg == CFG_METHOD_7) ||
            (mcfg == CFG_METHOD_8) || (mcfg == CFG_METHOD_9)) {
            auto_nego &= ~(PHY_Cap_PAUSE|PHY_Cap_ASYM_PAUSE);
        }

        if ((mcfg == MCFG_8105E) || (mcfg == MCFG_8105E_2) ||
            (mcfg == MCFG_8105E_3) || (mcfg == MCFG_8105E_4) ||
            (mcfg == MCFG_8402_1) || (mcfg == MCFG_8106E_1) ||
            (mcfg == MCFG_8106E_2)) {
            if (eee_enable == 1)
                auto_nego &= ~(PHY_Cap_PAUSE|PHY_Cap_ASYM_PAUSE);
        }

        tp->phy_auto_nego_reg = auto_nego;

        if ((mcfg == CFG_METHOD_4) ||
            (mcfg == CFG_METHOD_5)) {
            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1f, 0x0000);
            WriteGMII16( MII_BMCR, BMCR_RESET);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            IODelay(100);
            rtl8101_hw_phy_config(dev);
        } else if (((mcfg == CFG_METHOD_1) ||
                    (mcfg == CFG_METHOD_2) ||
                    (mcfg == CFG_METHOD_3)) &&
                   (speed == SPEED_10)) {
            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1f, 0x0000);
            WriteGMII16( MII_BMCR, BMCR_RESET);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            rtl8101_hw_phy_config(dev);
        }

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( PHY_AUTO_NEGO_REG, auto_nego);
        if (mcfg == MCFG_8105E)
            WriteGMII16( MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        else
            WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        mdelay(20);
    } else {
        /*true force*/
#ifndef BMCR_SPEED100
#define BMCR_SPEED100	0x0040
#endif

#ifndef BMCR_SPEED10
#define BMCR_SPEED10	0x0000
#endif
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED10 |
                              BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED100 |
                              BMCR_FULLDPLX;
        }

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( MII_BMCR, bmcr_true_force);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }

    tp->autoneg = autoneg;
    tp->speed = speed;
    tp->duplex = duplex;

    return 0;
}

static int
rtl8101_set_speed(struct net_device *dev,
                  u8 autoneg,
                  u16 speed,
                  u8 duplex)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    int ret;

    ret = tp->set_speed(dev, autoneg, speed, duplex);

    return ret;
}

static int
rtl8101_set_settings(struct net_device *dev,
                     struct ethtool_cmd *cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&tp->lock, flags);
    ret = rtl8101_set_speed(dev, cmd->autoneg, cmd->speed, cmd->duplex);
    spin_unlock_irqrestore(&tp->lock, flags);

    return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32
rtl8101_get_tx_csum(struct net_device *dev)
{
    return (dev->features & NETIF_F_IP_CSUM) != 0;
}

static u32
rtl8101_get_rx_csum(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    return cp_cmd & RxChkSum;
}

static int
rtl8101_set_tx_csum(struct net_device *dev,
                    u32 data)
{
    if (data)
        dev->features |= NETIF_F_IP_CSUM;
    else
        dev->features &= ~NETIF_F_IP_CSUM;

    return 0;
}

static int
rtl8101_set_rx_csum(struct net_device *dev,
                    u32 data)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    if (data)
        cp_cmd |= RxChkSum;
    else
        cp_cmd &= ~RxChkSum;

    WriteMMIO16(CPlusCmd, cp_cmd);

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}
#endif

#ifdef CONFIG_R8101_VLAN

static inline u32
rtl8101_tx_vlan_tag(struct rtl8101_private *tp,
                    struct sk_buff *skb)
{
    u32 tag;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    tag = (tp->vlgrp && vlan_tx_tag_present(skb)) ?
          TxVlanTag | swab16(vlan_tx_tag_get(skb)) : 0x00;
#else
    tag = (vlan_tx_tag_present(skb)) ?
          TxVlanTag | swab16(vlan_tx_tag_get(skb)) : 0x00;
#endif

    return tag;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)

static void
rtl8101_vlan_rx_register(struct net_device *dev,
                         struct vlan_group *grp)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    tp->vlgrp = grp;
    if (tp->vlgrp)
        cp_cmd |= RxVlan;
    else
        cp_cmd &= ~RxVlan;
    WriteMMIO16(CPlusCmd, cp_cmd);
    spin_unlock_irqrestore(&tp->lock, flags);
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
static void
rtl8101_vlan_rx_kill_vid(struct net_device *dev,
                         unsigned short vid)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
    if (tp->vlgrp)
        tp->vlgrp->vlan_devices[vid] = NULL;
#else
    vlan_group_set_device(tp->vlgrp, vid, NULL);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
    spin_unlock_irqrestore(&tp->lock, flags);
}
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)

static int
rtl8101_rx_vlan_skb(struct rtl8101_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
    u32 opts2 = le32_to_cpu(desc->opts2);
    int ret = -1;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    if (tp->vlgrp && (opts2 & RxVlanTag)) {
        rtl8101_rx_hwaccel_skb(skb, tp->vlgrp, swab16(opts2 & 0xffff));
        ret = 0;
    }
#else
    if (opts2 & RxVlanTag)
        __vlan_hwaccel_put_tag(skb, swab16(opts2 & 0xffff));
#endif

    desc->opts2 = 0;
    return ret;
}

#else /* !CONFIG_R8101_VLAN */

static inline u32
rtl8101_tx_vlan_tag(struct rtl8101_private *tp,
                    struct sk_buff *skb)
{
    return 0;
}

static int
rtl8101_rx_vlan_skb(struct rtl8101_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
    return -1;
}

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32 rtl8101_fix_features(struct net_device *dev, u32 features)
#else
static netdev_features_t rtl8101_fix_features(struct net_device *dev,
        netdev_features_t features)
#endif
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    if (dev->mtu > ETH_DATA_LEN)
        features &= ~NETIF_F_ALL_TSO;
    spin_unlock_irqrestore(&tp->lock, flags);

    return features;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int rtl8101_hw_set_features(struct net_device *dev, u32 features)
#else
static int rtl8101_hw_set_features(struct net_device *dev,
                                   netdev_features_t features)
#endif
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    if (features & NETIF_F_RXCSUM)
        cp_cmd |= RxChkSum;
    else
        cp_cmd &= ~RxChkSum;

    if (dev->features & NETIF_F_HW_VLAN_RX)
        cp_cmd |= RxVlan;
    else
        cp_cmd &= ~RxVlan;

    WriteMMIO16(CPlusCmd, cp_cmd);
    ReadMMIO16(CPlusCmd);

    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int rtl8101_set_features(struct net_device *dev, u32 features)
#else
static int rtl8101_set_features(struct net_device *dev,
                                netdev_features_t features)
#endif
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    rtl8101_hw_set_features(dev, features);

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}

#endif

static void
rtl8101_gset_xmii(struct net_device *dev,
                  struct ethtool_cmd *cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 status;
    unsigned long flags;

    cmd->supported = SUPPORTED_10baseT_Half |
                     SUPPORTED_10baseT_Full |
                     SUPPORTED_100baseT_Half |
                     SUPPORTED_100baseT_Full |
                     SUPPORTED_Autoneg |
                     SUPPORTED_TP;

    spin_lock_irqsave(&tp->phy_lock, flags);
    cmd->autoneg = (ReadGMII16( MII_BMCR) & BMCR_ANENABLE) ? 1 : 0;
    spin_unlock_irqrestore(&tp->phy_lock, flags);
    cmd->advertising = ADVERTISED_TP | ADVERTISED_Autoneg;

    if (tp->phy_auto_nego_reg & ADVERTISE_10HALF)
        cmd->advertising |= ADVERTISED_10baseT_Half;

    if (tp->phy_auto_nego_reg & ADVERTISE_10FULL)
        cmd->advertising |= ADVERTISED_10baseT_Half |
                            ADVERTISED_10baseT_Full;

    if (tp->phy_auto_nego_reg & ADVERTISE_100HALF)
        cmd->advertising |= ADVERTISED_10baseT_Half |
                            ADVERTISED_10baseT_Full |
                            ADVERTISED_100baseT_Half;

    if (tp->phy_auto_nego_reg & ADVERTISE_100FULL)
        cmd->advertising |= ADVERTISED_10baseT_Half |
                            ADVERTISED_10baseT_Full |
                            ADVERTISED_100baseT_Half |
                            ADVERTISED_100baseT_Full;

    status = ReadMMIO8(PHYstatus);

    if (status & _100bps)
        cmd->speed = SPEED_100;
    else if (status & _10bps)
        cmd->speed = SPEED_10;

    if (status & TxFlowCtrl)
        cmd->advertising |= ADVERTISED_Asym_Pause;

    if (status & RxFlowCtrl)
        cmd->advertising |= ADVERTISED_Pause;

    cmd->duplex = (status & FullDup) ? DUPLEX_FULL : DUPLEX_HALF;
}

static int
rtl8101_get_settings(struct net_device *dev,
                     struct ethtool_cmd *cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    tp->get_settings(dev, cmd);

    spin_unlock_irqrestore(&tp->lock, flags);
    return 0;
}

static void
rtl8101_get_regs(struct net_device *dev,
                 struct ethtool_regs *regs,
                 void *p)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    if (regs->len > R8101_REGS_SIZE)
        regs->len = R8101_REGS_SIZE;

    spin_lock_irqsave(&tp->lock, flags);
    memcpy_fromio(p, tp->mmio_addr, regs->len);
    spin_unlock_irqrestore(&tp->lock, flags);
}

static u32
rtl8101_get_msglevel(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    return tp->msg_enable;
}

static void
rtl8101_set_msglevel(struct net_device *dev,
                     u32 value)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    tp->msg_enable = value;
}

static const char rtl8101_gstrings[][ETH_GSTRING_LEN] = {
    "tx_packets",
    "rx_packets",
    "tx_errors",
    "rx_errors",
    "rx_missed",
    "align_errors",
    "tx_single_collisions",
    "tx_multi_collisions",
    "unicast",
    "broadcast",
    "multicast",
    "tx_aborted",
    "tx_underrun",
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
static int rtl8101_get_stats_count(struct net_device *dev)
{
    return ARRAY_SIZE(rtl8101_gstrings);
}
#else
static int rtl8101_get_sset_count(struct net_device *dev, int sset)
{
    switch (sset) {
    case ETH_SS_STATS:
        return ARRAY_SIZE(rtl8101_gstrings);
    default:
        return -EOPNOTSUPP;
    }
}
#endif

static void
rtl8101_get_ethtool_stats(struct net_device *dev,
                          struct ethtool_stats *stats,
                          u64 *data)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    struct rtl8101_counters *counters;
    dma_addr_t paddr;
    u32 cmd;

    ASSERT_RTNL();

    counters = tp->tally_vaddr;
    paddr = tp->tally_paddr;
    if (!counters)
        return;

    WriteMMIO32(CounterAddrHigh, (u64)paddr >> 32);
    cmd = (u64)paddr & DMA_BIT_MASK(32);
    WriteMMIO32(CounterAddrLow, cmd);
    WriteMMIO32(CounterAddrLow, cmd | CounterDump);

    while (ReadMMIO32(CounterAddrLow) & CounterDump) {
        if (msleep_interruptible(1))
            break;
    }

    data[0]	= le64_to_cpu(counters->tx_packets);
    data[1] = le64_to_cpu(counters->rx_packets);
    data[2] = le64_to_cpu(counters->tx_errors);
    data[3] = le32_to_cpu(counters->rx_errors);
    data[4] = le16_to_cpu(counters->rx_missed);
    data[5] = le16_to_cpu(counters->align_errors);
    data[6] = le32_to_cpu(counters->tx_one_collision);
    data[7] = le32_to_cpu(counters->tx_multi_collision);
    data[8] = le64_to_cpu(counters->rx_unicast);
    data[9] = le64_to_cpu(counters->rx_broadcast);
    data[10] = le32_to_cpu(counters->rx_multicast);
    data[11] = le16_to_cpu(counters->tx_aborted);
    data[12] = le16_to_cpu(counters->tx_underun);
}

static void
rtl8101_get_strings(struct net_device *dev,
                    u32 stringset,
                    u8 *data)
{
    switch (stringset) {
    case ETH_SS_STATS:
        memcpy(data, *rtl8101_gstrings, sizeof(rtl8101_gstrings));
        break;
    }
}

#undef ethtool_op_get_link
#define ethtool_op_get_link _kc_ethtool_op_get_link
u32 _kc_ethtool_op_get_link(struct net_device *dev)
{
    return netif_carrier_ok(dev) ? 1 : 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
#undef ethtool_op_get_sg
#define ethtool_op_get_sg _kc_ethtool_op_get_sg
u32 _kc_ethtool_op_get_sg(struct net_device *dev)
{
#ifdef NETIF_F_SG
    return (dev->features & NETIF_F_SG) != 0;
#else
    return 0;
#endif
}

#undef ethtool_op_set_sg
#define ethtool_op_set_sg _kc_ethtool_op_set_sg
int _kc_ethtool_op_set_sg(struct net_device *dev, u32 data)
{
#ifdef NETIF_F_SG
    if (data)
        dev->features |= NETIF_F_SG;
    else
        dev->features &= ~NETIF_F_SG;
#endif

    return 0;
}
#endif

static struct ethtool_ops rtl8101_ethtool_ops = {
    .get_drvinfo		= rtl8101_get_drvinfo,
    .get_regs_len		= rtl8101_get_regs_len,
    .get_link		= ethtool_op_get_link,
    .get_settings		= rtl8101_get_settings,
    .set_settings		= rtl8101_set_settings,
    .get_msglevel		= rtl8101_get_msglevel,
    .set_msglevel		= rtl8101_set_msglevel,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
    .get_rx_csum		= rtl8101_get_rx_csum,
    .set_rx_csum		= rtl8101_set_rx_csum,
    .get_tx_csum		= rtl8101_get_tx_csum,
    .set_tx_csum		= rtl8101_set_tx_csum,
    .get_sg			= ethtool_op_get_sg,
    .set_sg			= ethtool_op_set_sg,
#ifdef NETIF_F_TSO
    .get_tso		= ethtool_op_get_tso,
    .set_tso		= ethtool_op_set_tso,
#endif
#endif
    .get_regs		= rtl8101_get_regs,
    .get_wol		= rtl8101_get_wol,
    .set_wol		= rtl8101_set_wol,
    .get_strings		= rtl8101_get_strings,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
    .get_stats_count    = rtl8101_get_stats_count,
#else
    .get_sset_count     = rtl8101_get_sset_count,
#endif
    .get_ethtool_stats	= rtl8101_get_ethtool_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#ifdef ETHTOOL_GPERMADDR
    .get_perm_addr		= ethtool_op_get_perm_addr,
#endif
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
};

static void rtl8101_get_mac_version(struct rtl8101_private *tp, void __iomem *ioaddr)
{
    u32 reg,val32;
    u32 ICVerID;

    val32 = ReadMMIO32(TxConfig);
    reg = val32 & 0x7c800000;
    ICVerID = val32 & 0x00700000;

    mcfg = CFG_METHOD_UNKNOWN;
    switch (reg) {
    case 0x34000000:
        if (ICVerID == 0x00000000)
            mcfg = CFG_METHOD_1;
        else if (ICVerID == 0x00200000)
            mcfg = CFG_METHOD_2;
        else if (ICVerID == 0x00300000)
            mcfg = CFG_METHOD_3;
        else
            mcfg = CFG_METHOD_3;
        break;
    case 0x34800000:
    case 0x24800000:
        if (ICVerID == 0x00100000)
            mcfg = CFG_METHOD_4;
        else if (ICVerID == 0x00200000)
            mcfg = CFG_METHOD_5;
        else if (ICVerID == 0x00400000)
            mcfg = CFG_METHOD_6;
        else if (ICVerID == 0x00500000)
            mcfg = CFG_METHOD_7;
        else if (ICVerID == 0x00600000)
            mcfg = CFG_METHOD_8;
        else
            mcfg = CFG_METHOD_8;
        break;
    case 0x24000000:
        mcfg = CFG_METHOD_9;
        break;
    case 0x2C000000:
        mcfg = MCFG_8105E;
        break;
    case 0x40800000:
        if (ICVerID == 0x00100000)
            mcfg = CFG_METHOD_11;
        else if (ICVerID == 0x00200000)
            mcfg = MCFG_8168DP_2;
        else if (ICVerID == 0x00300000)
            mcfg = MCFG_8168DP_3;
        else if (ICVerID == 0x00400000)
            mcfg = MCFG_8168DP_3;
        break;
    case 0x44000000:
        mcfg = MCFG_8402_1;
        break;
    case 0x44800000:
        if (ICVerID == 0x00000000)
            mcfg = MCFG_8106E_1;
        else if (ICVerID == 0x00100000)
            mcfg = MCFG_8168E_VL_1;
        break;
    case 0x50800000:
        if (ICVerID == 0x00100000) {
            mcfg = MCFG_8168E_VL_2;
        } else {
            mcfg = MCFG_8168E_VL_2;
        }
        break;
    default:
        printk("unknown chip version (%x)\n",reg);
        break;
    }
}

static void
rtl8101_print_mac_version(struct rtl8101_private *tp)
{
    int i;
    for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
        if (mcfg == rtl_chip_info[i].mcfg) {
            dprintk("Realtek PCIe FE Family Controller mcfg = %04d\n",
                    rtl_chip_info[i].mcfg);
            return;
        }
    }

    dprintk("mac_version == Unknown\n");
}

static void
rtl8101_tally_counter_addr_fill(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;

    if (!tp->tally_paddr)
        return;

    WriteMMIO32(CounterAddrHigh, (u64)tp->tally_paddr >> 32);
    WriteMMIO32(CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32)));
}

static void
rtl8101_tally_counter_clear(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;

    if (mcfg == CFG_METHOD_1 || mcfg == CFG_METHOD_2 ||
        mcfg == CFG_METHOD_3 )
        return;

    if (!tp->tally_paddr)
        return;

    WriteMMIO32(CounterAddrHigh, (u64)tp->tally_paddr >> 32);
    WriteMMIO32(CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32) | BIT_0));
}

static void
rtl8101_exit_oob(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 data16;

    rtl8101_enable_rxdvgate(dev);

    rtl8101_wait_txrx_fifo_empty(dev);

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteMMIO8(MCUCmd_reg, ReadMMIO8(MCUCmd_reg) & ~Now_is_oob);

        data16 = mac_ocp_read(0xE8DE) & ~BIT_14;
        mac_ocp_write(0xE8DE, data16);
        rtl8101_wait_ll_share_fifo_ready(dev);

        data16 = mac_ocp_read(0xE8DE) | BIT_15;
        mac_ocp_write(0xE8DE, data16);

        rtl8101_wait_ll_share_fifo_ready(dev);
        break;
    }
}

static void
rtl8101_hw_mac_mcu_config(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned int i;

    if (mcfg == MCFG_8168E_VL_2) {
        u16 rtl8101_phy_value[]= {
            0xE008, 0xE01B, 0xE02E, 0xE044, 0xE046,
            0xE048, 0xE04A, 0xE04C, 0x49D2, 0xF10D,
            0x766C, 0x49E2, 0xF00A, 0x1EC0, 0x8EE1,
            0xC60A, 0x77C0, 0x4870, 0x9FC0, 0x1EA0,
            0xC707, 0x8EE1, 0x9D6C, 0xC603, 0xBE00,
            0xB416, 0x0076, 0xE86C, 0xC513, 0x64A0,
            0x49C1, 0xF00A, 0x1CEA, 0x2242, 0x0402,
            0xC50B, 0x9CA2, 0x1C11, 0x9CA0, 0xC506,
            0xBD00, 0x7444, 0xC502, 0xBD00, 0x0A30,
            0x0A46, 0xE434, 0xE096, 0x49D9, 0xF00F,
            0xC512, 0x74A0, 0x48C8, 0x48CA, 0x9CA0,
            0xC50F, 0x1B00, 0x9BA0, 0x1B1C, 0x483F,
            0x9BA2, 0x1B04, 0xC5F0, 0x9BA0, 0xC602,
            0xBE00, 0x03DE, 0xE434, 0xE096, 0xE860,
            0xDE20, 0xC602, 0xBE00, 0x0000, 0xC602,
            0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000,
            0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00,
            0x0000
        };
        mac_ocp_write(0xFC28, 0x0000);
        mac_ocp_write(0xFC2A, 0x0000);
        mac_ocp_write(0xFC2C, 0x0000);
        mac_ocp_write(0xFC2E, 0x0000);
        mac_ocp_write(0xFC30, 0x0000);
        mac_ocp_write(0xFC32, 0x0000);
        mac_ocp_write(0xFC34, 0x0000);
        mac_ocp_write(0xFC36, 0x0000);
        mdelay(3);
        mac_ocp_write(0xFC26, 0x0000);
        for (i = 0; i < ARRAY_SIZE(rtl8101_phy_value); i++)
            mac_ocp_write(0xF800+i*2, rtl8101_phy_value[i]);
        mac_ocp_write(0xFC26, 0x8000);
        mac_ocp_write(0xFC2A, 0x0A2F);
        mac_ocp_write(0xFC2C, 0x0297);
    }
}

static void
rtl8101_hw_init(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    switch (mcfg) {
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        break;
    }

    if (mcfg == MCFG_8105E)
        WriteMMIO8(0xF3, ReadMMIO8(0xF3) | BIT_2);

    rtl8101_hw_mac_mcu_config(dev);

    /*disable ocp phy power saving*/
    if (mcfg == MCFG_8168E_VL_2 ) {
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0C41);
        WriteGMII16( 0x13, 0x0000);
        WriteGMII16( 0x13, 0x0500);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
}

static void
rtl8101_hw_ephy_config(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 ephy_data;

    if (mcfg == CFG_METHOD_4) {
        rtl8101_ephy_write(ioaddr, 0x03, 0xc2f9);
    } else if (mcfg == CFG_METHOD_5) {
        rtl8101_ephy_write(ioaddr, 0x01, 0x6FE5);
        rtl8101_ephy_write(ioaddr, 0x03, 0xD7D9);
    } else if (mcfg == CFG_METHOD_6) {
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF35);
    } else if (mcfg == CFG_METHOD_7) {
        rtl8101_ephy_write(ioaddr, 0x19, 0xEC90);
        rtl8101_ephy_write(ioaddr, 0x01, 0x6FE5);
        rtl8101_ephy_write(ioaddr, 0x03, 0x05D9);
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF35);
    } else if (mcfg == CFG_METHOD_8) {
        rtl8101_ephy_write(ioaddr, 0x01, 0x6FE5);
        rtl8101_ephy_write(ioaddr, 0x03, 0x05D9);
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF35);
        rtl8101_ephy_write(ioaddr, 0x19, 0xECFA);
    } else if (mcfg == CFG_METHOD_9) {
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF25);
        rtl8101_ephy_write(ioaddr, 0x07, 0x8E68);
    } else if (mcfg == MCFG_8105E) {
        ephy_data = rtl8101_ephy_read(ioaddr, 0x00) & ~0x0200;
        ephy_data |= 0x0100;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x00);
        ephy_data |= 0x0004;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x06) & ~0x0002;
        ephy_data |= 0x0001;
        rtl8101_ephy_write(ioaddr, 0x06, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x06);
        ephy_data |= 0x0030;
        rtl8101_ephy_write(ioaddr, 0x06, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x07, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x00);
        ephy_data |= 0x0020;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x03) & ~0x5800;
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x03, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x03);
        ephy_data |= 0x0001;
        rtl8101_ephy_write(ioaddr, 0x03, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x01) & ~0x0800;
        ephy_data |= 0x1000;
        rtl8101_ephy_write(ioaddr, 0x01, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x4000;
        rtl8101_ephy_write(ioaddr, 0x07, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x1E);
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x1E, ephy_data);

        rtl8101_ephy_write(ioaddr, 0x19, 0xFE6C);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x0A);
        ephy_data |= 0x0040;
        rtl8101_ephy_write(ioaddr, 0x0A, ephy_data);

    } else if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
               mcfg == MCFG_8168DP_3) {
        ephy_data = rtl8101_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x4000;
        rtl8101_ephy_write(ioaddr, 0x07, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0200;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0020;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x1E);
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x1E, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x03);
        ephy_data |= 0x0001;
        rtl8101_ephy_write(ioaddr, 0x03, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0100;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0004;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x0A);
        ephy_data |= 0x0020;
        rtl8101_ephy_write(ioaddr, 0x0A, ephy_data);

        if (mcfg == MCFG_8168DP_1) {
            WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        } else if (mcfg == MCFG_8168DP_2 ||
                   mcfg == MCFG_8168DP_3) {
            ephy_data = rtl8101_ephy_read(ioaddr, 0x1E);
            ephy_data |= 0x8000;
            rtl8101_ephy_write(ioaddr, 0x1E, ephy_data);
        }
    } else if (mcfg == MCFG_8402_1) {
        rtl8101_ephy_write(ioaddr, 0x19, 0xff64);
    } else if (mcfg == MCFG_8168E_VL_2) {
        ephy_data = rtl8101_ephy_read(ioaddr, 0x00);
        ephy_data |= BIT_3;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);
        ephy_data = rtl8101_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        ephy_data |= BIT_10;
        rtl8101_ephy_write(ioaddr, 0x0C, ephy_data);

        rtl8101_ephy_write(ioaddr, 0x19, 0xFC00);
        rtl8101_ephy_write(ioaddr, 0x1E, 0x20EB);

        ephy_data = rtl8101_ephy_read(ioaddr, 0x06);
        ephy_data |= BIT_4;
        rtl8101_ephy_write(ioaddr, 0x06, ephy_data);
    }
}

static int
rtl8101_check_hw_phy_mcu_code_ver(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    int ram_code_ver_match = 0;
    u16 sw_ram_code_ver = 0xFFFF;
    u16 hw_ram_code_ver = 0;


    switch (mcfg) {
    case MCFG_8168E_VL_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168E_VL_2;
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x801E);
        hw_ram_code_ver = ReadGMII16( 0x14);
        WriteGMII16( 0x1F, 0x0000);
        break;
    }

    if( hw_ram_code_ver == sw_ram_code_ver)
        ram_code_ver_match = 1;

    return ram_code_ver_match;
}

static void
rtl8101_write_hw_phy_mcu_code_ver(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    u16 sw_ram_code_ver = 0xFFFF;


    switch (mcfg) {
    case MCFG_8168E_VL_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168E_VL_2;
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_VL_2:
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x801E);
        WriteGMII16( 0x14, sw_ram_code_ver);
        WriteGMII16( 0x1F, 0x0000);
        break;
    }
}

static void
rtl8101_init_hw_phy_mcu(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned int gphy_val,i;
    u32 csi_tmp;

    if(rtl8101_check_hw_phy_mcu_code_ver(dev))
        return;

    if (mcfg == MCFG_8105E) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x1800);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x17, 0x0117);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1E, 0x002C);
        WriteGMII16( 0x1B, 0x5000);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x16, 0x4104);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x1E);
            gphy_val &= 0x03FF;
            if (gphy_val==0x000C)
                break;
        }
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x07);
            if ((gphy_val & BIT_5)==0)
                break;
        }
        gphy_val = ReadGMII16( 0x07);
        if (gphy_val & BIT_5) {
            WriteGMII16( 0x1f, 0x0007);
            WriteGMII16( 0x1e, 0x00a1);
            WriteGMII16( 0x17, 0x1000);
            WriteGMII16( 0x17, 0x0000);
            WriteGMII16( 0x17, 0x2000);
            WriteGMII16( 0x1e, 0x002f);
            WriteGMII16( 0x18, 0x9bfb);
            WriteGMII16( 0x1f, 0x0005);
            WriteGMII16( 0x07, 0x0000);
            WriteGMII16( 0x1f, 0x0000);
        }
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        gphy_val = ReadGMII16( 0x00);
        gphy_val &= ~(BIT_7);
        WriteGMII16( 0x00, gphy_val);
        WriteGMII16( 0x1f, 0x0002);
        gphy_val = ReadGMII16( 0x08);
        gphy_val &= ~(BIT_7);
        WriteGMII16( 0x08, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0307);
        WriteGMII16( 0x15, 0x000e);
        WriteGMII16( 0x19, 0x000a);
        WriteGMII16( 0x15, 0x0010);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x0018);
        WriteGMII16( 0x19, 0x4801);
        WriteGMII16( 0x15, 0x0019);
        WriteGMII16( 0x19, 0x6801);
        WriteGMII16( 0x15, 0x001a);
        WriteGMII16( 0x19, 0x66a1);
        WriteGMII16( 0x15, 0x001f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0020);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0021);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0022);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0023);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0024);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0025);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x0026);
        WriteGMII16( 0x19, 0x40ea);
        WriteGMII16( 0x15, 0x0027);
        WriteGMII16( 0x19, 0x4503);
        WriteGMII16( 0x15, 0x0028);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x0029);
        WriteGMII16( 0x19, 0xa631);
        WriteGMII16( 0x15, 0x002a);
        WriteGMII16( 0x19, 0x9717);
        WriteGMII16( 0x15, 0x002b);
        WriteGMII16( 0x19, 0x302c);
        WriteGMII16( 0x15, 0x002c);
        WriteGMII16( 0x19, 0x4802);
        WriteGMII16( 0x15, 0x002d);
        WriteGMII16( 0x19, 0x58da);
        WriteGMII16( 0x15, 0x002e);
        WriteGMII16( 0x19, 0x400d);
        WriteGMII16( 0x15, 0x002f);
        WriteGMII16( 0x19, 0x4488);
        WriteGMII16( 0x15, 0x0030);
        WriteGMII16( 0x19, 0x9e00);
        WriteGMII16( 0x15, 0x0031);
        WriteGMII16( 0x19, 0x63c8);
        WriteGMII16( 0x15, 0x0032);
        WriteGMII16( 0x19, 0x6481);
        WriteGMII16( 0x15, 0x0033);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0034);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0035);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0036);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0037);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0038);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0039);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x003a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x003b);
        WriteGMII16( 0x19, 0x63e8);
        WriteGMII16( 0x15, 0x003c);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x003d);
        WriteGMII16( 0x19, 0x59d4);
        WriteGMII16( 0x15, 0x003e);
        WriteGMII16( 0x19, 0x63f8);
        WriteGMII16( 0x15, 0x0040);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x0041);
        WriteGMII16( 0x19, 0x30de);
        WriteGMII16( 0x15, 0x0044);
        WriteGMII16( 0x19, 0x480f);
        WriteGMII16( 0x15, 0x0045);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x0046);
        WriteGMII16( 0x19, 0x6680);
        WriteGMII16( 0x15, 0x0047);
        WriteGMII16( 0x19, 0x7c10);
        WriteGMII16( 0x15, 0x0048);
        WriteGMII16( 0x19, 0x63c8);
        WriteGMII16( 0x15, 0x0049);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004b);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004d);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004f);
        WriteGMII16( 0x19, 0x40ea);
        WriteGMII16( 0x15, 0x0050);
        WriteGMII16( 0x19, 0x4503);
        WriteGMII16( 0x15, 0x0051);
        WriteGMII16( 0x19, 0x58ca);
        WriteGMII16( 0x15, 0x0052);
        WriteGMII16( 0x19, 0x63c8);
        WriteGMII16( 0x15, 0x0053);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x0054);
        WriteGMII16( 0x19, 0x66a0);
        WriteGMII16( 0x15, 0x0055);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x0056);
        WriteGMII16( 0x19, 0x3000);
        WriteGMII16( 0x15, 0x006E);
        WriteGMII16( 0x19, 0x9afa);
        WriteGMII16( 0x15, 0x00a1);
        WriteGMII16( 0x19, 0x3044);
        WriteGMII16( 0x15, 0x00ab);
        WriteGMII16( 0x19, 0x5820);
        WriteGMII16( 0x15, 0x00ac);
        WriteGMII16( 0x19, 0x5e04);
        WriteGMII16( 0x15, 0x00ad);
        WriteGMII16( 0x19, 0xb60c);
        WriteGMII16( 0x15, 0x00af);
        WriteGMII16( 0x19, 0x000a);
        WriteGMII16( 0x15, 0x00b2);
        WriteGMII16( 0x19, 0x30b9);
        WriteGMII16( 0x15, 0x00b9);
        WriteGMII16( 0x19, 0x4408);
        WriteGMII16( 0x15, 0x00ba);
        WriteGMII16( 0x19, 0x480b);
        WriteGMII16( 0x15, 0x00bb);
        WriteGMII16( 0x19, 0x5e00);
        WriteGMII16( 0x15, 0x00bc);
        WriteGMII16( 0x19, 0x405f);
        WriteGMII16( 0x15, 0x00bd);
        WriteGMII16( 0x19, 0x4448);
        WriteGMII16( 0x15, 0x00be);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x00bf);
        WriteGMII16( 0x19, 0x4468);
        WriteGMII16( 0x15, 0x00c0);
        WriteGMII16( 0x19, 0x9c02);
        WriteGMII16( 0x15, 0x00c1);
        WriteGMII16( 0x19, 0x58a0);
        WriteGMII16( 0x15, 0x00c2);
        WriteGMII16( 0x19, 0xb605);
        WriteGMII16( 0x15, 0x00c3);
        WriteGMII16( 0x19, 0xc0d3);
        WriteGMII16( 0x15, 0x00c4);
        WriteGMII16( 0x19, 0x00e6);
        WriteGMII16( 0x15, 0x00c5);
        WriteGMII16( 0x19, 0xdaec);
        WriteGMII16( 0x15, 0x00c6);
        WriteGMII16( 0x19, 0x00fa);
        WriteGMII16( 0x15, 0x00c7);
        WriteGMII16( 0x19, 0x9df9);
        WriteGMII16( 0x15, 0x00c8);
        WriteGMII16( 0x19, 0x307a);
        WriteGMII16( 0x15, 0x0112);
        WriteGMII16( 0x19, 0x6421);
        WriteGMII16( 0x15, 0x0113);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0114);
        WriteGMII16( 0x19, 0x63f0);
        WriteGMII16( 0x15, 0x0115);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x0116);
        WriteGMII16( 0x19, 0x4418);
        WriteGMII16( 0x15, 0x0117);
        WriteGMII16( 0x19, 0x9b00);
        WriteGMII16( 0x15, 0x0118);
        WriteGMII16( 0x19, 0x6461);
        WriteGMII16( 0x15, 0x0119);
        WriteGMII16( 0x19, 0x64e1);
        WriteGMII16( 0x15, 0x011a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0150);
        WriteGMII16( 0x19, 0x6461);
        WriteGMII16( 0x15, 0x0151);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x0152);
        WriteGMII16( 0x19, 0x4540);
        WriteGMII16( 0x15, 0x0153);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x0155);
        WriteGMII16( 0x19, 0x6421);
        WriteGMII16( 0x15, 0x0156);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x021e);
        WriteGMII16( 0x19, 0x5410);
        WriteGMII16( 0x15, 0x0225);
        WriteGMII16( 0x19, 0x5400);
        WriteGMII16( 0x15, 0x023D);
        WriteGMII16( 0x19, 0x4050);
        WriteGMII16( 0x15, 0x0295);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x02bd);
        WriteGMII16( 0x19, 0xa523);
        WriteGMII16( 0x15, 0x02be);
        WriteGMII16( 0x19, 0x32ca);
        WriteGMII16( 0x15, 0x02ca);
        WriteGMII16( 0x19, 0x48b3);
        WriteGMII16( 0x15, 0x02cb);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x02cc);
        WriteGMII16( 0x19, 0x4823);
        WriteGMII16( 0x15, 0x02cd);
        WriteGMII16( 0x19, 0x4510);
        WriteGMII16( 0x15, 0x02ce);
        WriteGMII16( 0x19, 0xb63a);
        WriteGMII16( 0x15, 0x02cf);
        WriteGMII16( 0x19, 0x7dc8);
        WriteGMII16( 0x15, 0x02d6);
        WriteGMII16( 0x19, 0x9bf8);
        WriteGMII16( 0x15, 0x02d8);
        WriteGMII16( 0x19, 0x85f6);
        WriteGMII16( 0x15, 0x02d9);
        WriteGMII16( 0x19, 0x32e0);
        WriteGMII16( 0x15, 0x02e0);
        WriteGMII16( 0x19, 0x4834);
        WriteGMII16( 0x15, 0x02e1);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x02e2);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x02e3);
        WriteGMII16( 0x19, 0x4824);
        WriteGMII16( 0x15, 0x02e4);
        WriteGMII16( 0x19, 0x4520);
        WriteGMII16( 0x15, 0x02e5);
        WriteGMII16( 0x19, 0x4008);
        WriteGMII16( 0x15, 0x02e6);
        WriteGMII16( 0x19, 0x4560);
        WriteGMII16( 0x15, 0x02e7);
        WriteGMII16( 0x19, 0x9d04);
        WriteGMII16( 0x15, 0x02e8);
        WriteGMII16( 0x19, 0x48c4);
        WriteGMII16( 0x15, 0x02e9);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02ea);
        WriteGMII16( 0x19, 0x4844);
        WriteGMII16( 0x15, 0x02eb);
        WriteGMII16( 0x19, 0x7dc8);
        WriteGMII16( 0x15, 0x02f0);
        WriteGMII16( 0x19, 0x9cf7);
        WriteGMII16( 0x15, 0x02f1);
        WriteGMII16( 0x19, 0xdf94);
        WriteGMII16( 0x15, 0x02f2);
        WriteGMII16( 0x19, 0x0002);
        WriteGMII16( 0x15, 0x02f3);
        WriteGMII16( 0x19, 0x6810);
        WriteGMII16( 0x15, 0x02f4);
        WriteGMII16( 0x19, 0xb614);
        WriteGMII16( 0x15, 0x02f5);
        WriteGMII16( 0x19, 0xc42b);
        WriteGMII16( 0x15, 0x02f6);
        WriteGMII16( 0x19, 0x00d4);
        WriteGMII16( 0x15, 0x02f7);
        WriteGMII16( 0x19, 0xc455);
        WriteGMII16( 0x15, 0x02f8);
        WriteGMII16( 0x19, 0x0093);
        WriteGMII16( 0x15, 0x02f9);
        WriteGMII16( 0x19, 0x92ee);
        WriteGMII16( 0x15, 0x02fa);
        WriteGMII16( 0x19, 0xefed);
        WriteGMII16( 0x15, 0x02fb);
        WriteGMII16( 0x19, 0x3312);
        WriteGMII16( 0x15, 0x0312);
        WriteGMII16( 0x19, 0x49b5);
        WriteGMII16( 0x15, 0x0313);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x0314);
        WriteGMII16( 0x19, 0x4d00);
        WriteGMII16( 0x15, 0x0315);
        WriteGMII16( 0x19, 0x6810);
        WriteGMII16( 0x15, 0x031e);
        WriteGMII16( 0x19, 0x404f);
        WriteGMII16( 0x15, 0x031f);
        WriteGMII16( 0x19, 0x44c8);
        WriteGMII16( 0x15, 0x0320);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x0321);
        WriteGMII16( 0x19, 0x00e7);
        WriteGMII16( 0x15, 0x0322);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0323);
        WriteGMII16( 0x19, 0x8203);
        WriteGMII16( 0x15, 0x0324);
        WriteGMII16( 0x19, 0x4d48);
        WriteGMII16( 0x15, 0x0325);
        WriteGMII16( 0x19, 0x3327);
        WriteGMII16( 0x15, 0x0326);
        WriteGMII16( 0x19, 0x4d40);
        WriteGMII16( 0x15, 0x0327);
        WriteGMII16( 0x19, 0xc8d7);
        WriteGMII16( 0x15, 0x0328);
        WriteGMII16( 0x19, 0x0003);
        WriteGMII16( 0x15, 0x0329);
        WriteGMII16( 0x19, 0x7c20);
        WriteGMII16( 0x15, 0x032a);
        WriteGMII16( 0x19, 0x4c20);
        WriteGMII16( 0x15, 0x032b);
        WriteGMII16( 0x19, 0xc8ed);
        WriteGMII16( 0x15, 0x032c);
        WriteGMII16( 0x19, 0x00f4);
        WriteGMII16( 0x15, 0x032d);
        WriteGMII16( 0x19, 0x82b3);
        WriteGMII16( 0x15, 0x032e);
        WriteGMII16( 0x19, 0xd11d);
        WriteGMII16( 0x15, 0x032f);
        WriteGMII16( 0x19, 0x00b1);
        WriteGMII16( 0x15, 0x0330);
        WriteGMII16( 0x19, 0xde18);
        WriteGMII16( 0x15, 0x0331);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x0332);
        WriteGMII16( 0x19, 0x91ee);
        WriteGMII16( 0x15, 0x0333);
        WriteGMII16( 0x19, 0x3339);
        WriteGMII16( 0x15, 0x033a);
        WriteGMII16( 0x19, 0x4064);
        WriteGMII16( 0x15, 0x0340);
        WriteGMII16( 0x19, 0x9e06);
        WriteGMII16( 0x15, 0x0341);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0342);
        WriteGMII16( 0x19, 0x8203);
        WriteGMII16( 0x15, 0x0343);
        WriteGMII16( 0x19, 0x4d48);
        WriteGMII16( 0x15, 0x0344);
        WriteGMII16( 0x19, 0x3346);
        WriteGMII16( 0x15, 0x0345);
        WriteGMII16( 0x19, 0x4d40);
        WriteGMII16( 0x15, 0x0346);
        WriteGMII16( 0x19, 0xd11d);
        WriteGMII16( 0x15, 0x0347);
        WriteGMII16( 0x19, 0x0099);
        WriteGMII16( 0x15, 0x0348);
        WriteGMII16( 0x19, 0xbb17);
        WriteGMII16( 0x15, 0x0349);
        WriteGMII16( 0x19, 0x8102);
        WriteGMII16( 0x15, 0x034a);
        WriteGMII16( 0x19, 0x334d);
        WriteGMII16( 0x15, 0x034b);
        WriteGMII16( 0x19, 0xa22c);
        WriteGMII16( 0x15, 0x034c);
        WriteGMII16( 0x19, 0x3397);
        WriteGMII16( 0x15, 0x034d);
        WriteGMII16( 0x19, 0x91f2);
        WriteGMII16( 0x15, 0x034e);
        WriteGMII16( 0x19, 0xc218);
        WriteGMII16( 0x15, 0x034f);
        WriteGMII16( 0x19, 0x00f0);
        WriteGMII16( 0x15, 0x0350);
        WriteGMII16( 0x19, 0x3397);
        WriteGMII16( 0x15, 0x0351);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0364);
        WriteGMII16( 0x19, 0xbc05);
        WriteGMII16( 0x15, 0x0367);
        WriteGMII16( 0x19, 0xa1fc);
        WriteGMII16( 0x15, 0x0368);
        WriteGMII16( 0x19, 0x3377);
        WriteGMII16( 0x15, 0x0369);
        WriteGMII16( 0x19, 0x328b);
        WriteGMII16( 0x15, 0x036a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0377);
        WriteGMII16( 0x19, 0x4b97);
        WriteGMII16( 0x15, 0x0378);
        WriteGMII16( 0x19, 0x6818);
        WriteGMII16( 0x15, 0x0379);
        WriteGMII16( 0x19, 0x4b07);
        WriteGMII16( 0x15, 0x037a);
        WriteGMII16( 0x19, 0x40ac);
        WriteGMII16( 0x15, 0x037b);
        WriteGMII16( 0x19, 0x4445);
        WriteGMII16( 0x15, 0x037c);
        WriteGMII16( 0x19, 0x404e);
        WriteGMII16( 0x15, 0x037d);
        WriteGMII16( 0x19, 0x4461);
        WriteGMII16( 0x15, 0x037e);
        WriteGMII16( 0x19, 0x9c09);
        WriteGMII16( 0x15, 0x037f);
        WriteGMII16( 0x19, 0x63da);
        WriteGMII16( 0x15, 0x0380);
        WriteGMII16( 0x19, 0x5440);
        WriteGMII16( 0x15, 0x0381);
        WriteGMII16( 0x19, 0x4b98);
        WriteGMII16( 0x15, 0x0382);
        WriteGMII16( 0x19, 0x7c60);
        WriteGMII16( 0x15, 0x0383);
        WriteGMII16( 0x19, 0x4c00);
        WriteGMII16( 0x15, 0x0384);
        WriteGMII16( 0x19, 0x4b08);
        WriteGMII16( 0x15, 0x0385);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x0386);
        WriteGMII16( 0x19, 0x338d);
        WriteGMII16( 0x15, 0x0387);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x0388);
        WriteGMII16( 0x19, 0x0080);
        WriteGMII16( 0x15, 0x0389);
        WriteGMII16( 0x19, 0x820c);
        WriteGMII16( 0x15, 0x038a);
        WriteGMII16( 0x19, 0xa10b);
        WriteGMII16( 0x15, 0x038b);
        WriteGMII16( 0x19, 0x9df3);
        WriteGMII16( 0x15, 0x038c);
        WriteGMII16( 0x19, 0x3395);
        WriteGMII16( 0x15, 0x038d);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x038e);
        WriteGMII16( 0x19, 0x00f9);
        WriteGMII16( 0x15, 0x038f);
        WriteGMII16( 0x19, 0xc017);
        WriteGMII16( 0x15, 0x0390);
        WriteGMII16( 0x19, 0x0005);
        WriteGMII16( 0x15, 0x0391);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x0392);
        WriteGMII16( 0x19, 0xa103);
        WriteGMII16( 0x15, 0x0393);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x0394);
        WriteGMII16( 0x19, 0x9df9);
        WriteGMII16( 0x15, 0x0395);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x0396);
        WriteGMII16( 0x19, 0x3397);
        WriteGMII16( 0x15, 0x0399);
        WriteGMII16( 0x19, 0x6810);
        WriteGMII16( 0x15, 0x03a4);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x03a5);
        WriteGMII16( 0x19, 0x8203);
        WriteGMII16( 0x15, 0x03a6);
        WriteGMII16( 0x19, 0x4d08);
        WriteGMII16( 0x15, 0x03a7);
        WriteGMII16( 0x19, 0x33a9);
        WriteGMII16( 0x15, 0x03a8);
        WriteGMII16( 0x19, 0x4d00);
        WriteGMII16( 0x15, 0x03a9);
        WriteGMII16( 0x19, 0x9bfa);
        WriteGMII16( 0x15, 0x03aa);
        WriteGMII16( 0x19, 0x33b6);
        WriteGMII16( 0x15, 0x03bb);
        WriteGMII16( 0x19, 0x4056);
        WriteGMII16( 0x15, 0x03bc);
        WriteGMII16( 0x19, 0x44e9);
        WriteGMII16( 0x15, 0x03bd);
        WriteGMII16( 0x19, 0x4054);
        WriteGMII16( 0x15, 0x03be);
        WriteGMII16( 0x19, 0x44f8);
        WriteGMII16( 0x15, 0x03bf);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x03c0);
        WriteGMII16( 0x19, 0x0037);
        WriteGMII16( 0x15, 0x03c1);
        WriteGMII16( 0x19, 0xbd37);
        WriteGMII16( 0x15, 0x03c2);
        WriteGMII16( 0x19, 0x9cfd);
        WriteGMII16( 0x15, 0x03c3);
        WriteGMII16( 0x19, 0xc639);
        WriteGMII16( 0x15, 0x03c4);
        WriteGMII16( 0x19, 0x0011);
        WriteGMII16( 0x15, 0x03c5);
        WriteGMII16( 0x19, 0x9b03);
        WriteGMII16( 0x15, 0x03c6);
        WriteGMII16( 0x19, 0x7c01);
        WriteGMII16( 0x15, 0x03c7);
        WriteGMII16( 0x19, 0x4c01);
        WriteGMII16( 0x15, 0x03c8);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x03c9);
        WriteGMII16( 0x19, 0x7c20);
        WriteGMII16( 0x15, 0x03ca);
        WriteGMII16( 0x19, 0x4c20);
        WriteGMII16( 0x15, 0x03cb);
        WriteGMII16( 0x19, 0x9af4);
        WriteGMII16( 0x15, 0x03cc);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03cd);
        WriteGMII16( 0x19, 0x4c52);
        WriteGMII16( 0x15, 0x03ce);
        WriteGMII16( 0x19, 0x4470);
        WriteGMII16( 0x15, 0x03cf);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03d0);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x03d1);
        WriteGMII16( 0x19, 0x33bf);
        WriteGMII16( 0x15, 0x03d6);
        WriteGMII16( 0x19, 0x4047);
        WriteGMII16( 0x15, 0x03d7);
        WriteGMII16( 0x19, 0x4469);
        WriteGMII16( 0x15, 0x03d8);
        WriteGMII16( 0x19, 0x492b);
        WriteGMII16( 0x15, 0x03d9);
        WriteGMII16( 0x19, 0x4479);
        WriteGMII16( 0x15, 0x03da);
        WriteGMII16( 0x19, 0x7c09);
        WriteGMII16( 0x15, 0x03db);
        WriteGMII16( 0x19, 0x8203);
        WriteGMII16( 0x15, 0x03dc);
        WriteGMII16( 0x19, 0x4d48);
        WriteGMII16( 0x15, 0x03dd);
        WriteGMII16( 0x19, 0x33df);
        WriteGMII16( 0x15, 0x03de);
        WriteGMII16( 0x19, 0x4d40);
        WriteGMII16( 0x15, 0x03df);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x03e0);
        WriteGMII16( 0x19, 0x0017);
        WriteGMII16( 0x15, 0x03e1);
        WriteGMII16( 0x19, 0xbd17);
        WriteGMII16( 0x15, 0x03e2);
        WriteGMII16( 0x19, 0x9b03);
        WriteGMII16( 0x15, 0x03e3);
        WriteGMII16( 0x19, 0x7c20);
        WriteGMII16( 0x15, 0x03e4);
        WriteGMII16( 0x19, 0x4c20);
        WriteGMII16( 0x15, 0x03e5);
        WriteGMII16( 0x19, 0x88f5);
        WriteGMII16( 0x15, 0x03e6);
        WriteGMII16( 0x19, 0xc428);
        WriteGMII16( 0x15, 0x03e7);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x03e8);
        WriteGMII16( 0x19, 0x9af2);
        WriteGMII16( 0x15, 0x03e9);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03ea);
        WriteGMII16( 0x19, 0x4c52);
        WriteGMII16( 0x15, 0x03eb);
        WriteGMII16( 0x19, 0x4470);
        WriteGMII16( 0x15, 0x03ec);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03ed);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x03ee);
        WriteGMII16( 0x19, 0x33da);
        WriteGMII16( 0x15, 0x03ef);
        WriteGMII16( 0x19, 0x3312);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x17, 0x2179);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0040);
        WriteGMII16( 0x18, 0x0645);
        WriteGMII16( 0x19, 0xe200);
        WriteGMII16( 0x18, 0x0655);
        WriteGMII16( 0x19, 0x9000);
        WriteGMII16( 0x18, 0x0d05);
        WriteGMII16( 0x19, 0xbe00);
        WriteGMII16( 0x18, 0x0d15);
        WriteGMII16( 0x19, 0xd300);
        WriteGMII16( 0x18, 0x0d25);
        WriteGMII16( 0x19, 0xfe00);
        WriteGMII16( 0x18, 0x0d35);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x0d45);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x0d55);
        WriteGMII16( 0x19, 0x1000);
        WriteGMII16( 0x18, 0x0d65);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x0d75);
        WriteGMII16( 0x19, 0x8200);
        WriteGMII16( 0x18, 0x0d85);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x0d95);
        WriteGMII16( 0x19, 0x7000);
        WriteGMII16( 0x18, 0x0da5);
        WriteGMII16( 0x19, 0x0f00);
        WriteGMII16( 0x18, 0x0db5);
        WriteGMII16( 0x19, 0x0100);
        WriteGMII16( 0x18, 0x0dc5);
        WriteGMII16( 0x19, 0x9b00);
        WriteGMII16( 0x18, 0x0dd5);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x0de5);
        WriteGMII16( 0x19, 0xe000);
        WriteGMII16( 0x18, 0x0df5);
        WriteGMII16( 0x19, 0xef00);
        WriteGMII16( 0x18, 0x16d5);
        WriteGMII16( 0x19, 0xe200);
        WriteGMII16( 0x18, 0x16e5);
        WriteGMII16( 0x19, 0xab00);
        WriteGMII16( 0x18, 0x2904);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x2914);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x2924);
        WriteGMII16( 0x19, 0x0100);
        WriteGMII16( 0x18, 0x2934);
        WriteGMII16( 0x19, 0x2000);
        WriteGMII16( 0x18, 0x2944);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2954);
        WriteGMII16( 0x19, 0x4600);
        WriteGMII16( 0x18, 0x2964);
        WriteGMII16( 0x19, 0xfc00);
        WriteGMII16( 0x18, 0x2974);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2984);
        WriteGMII16( 0x19, 0x5000);
        WriteGMII16( 0x18, 0x2994);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x18, 0x29a4);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x29b4);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x29c4);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x29d4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x29e4);
        WriteGMII16( 0x19, 0x2000);
        WriteGMII16( 0x18, 0x29f4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2a04);
        WriteGMII16( 0x19, 0xe600);
        WriteGMII16( 0x18, 0x2a14);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x2a24);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2a34);
        WriteGMII16( 0x19, 0x5000);
        WriteGMII16( 0x18, 0x2a44);
        WriteGMII16( 0x19, 0x8500);
        WriteGMII16( 0x18, 0x2a54);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x2a64);
        WriteGMII16( 0x19, 0xac00);
        WriteGMII16( 0x18, 0x2a74);
        WriteGMII16( 0x19, 0x0800);
        WriteGMII16( 0x18, 0x2a84);
        WriteGMII16( 0x19, 0xfc00);
        WriteGMII16( 0x18, 0x2a94);
        WriteGMII16( 0x19, 0xe000);
        WriteGMII16( 0x18, 0x2aa4);
        WriteGMII16( 0x19, 0x7400);
        WriteGMII16( 0x18, 0x2ab4);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x2ac4);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x2ad4);
        WriteGMII16( 0x19, 0x0100);
        WriteGMII16( 0x18, 0x2ae4);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x2af4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2b04);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x18, 0x2b14);
        WriteGMII16( 0x19, 0xfc00);
        WriteGMII16( 0x18, 0x2b24);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2b34);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x2b44);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x18, 0x2b54);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x2b64);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x2b74);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x2b84);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2b94);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x2ba4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2bb4);
        WriteGMII16( 0x19, 0xfc00);
        WriteGMII16( 0x18, 0x2bc4);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x2bd4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2be4);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x2bf4);
        WriteGMII16( 0x19, 0x8900);
        WriteGMII16( 0x18, 0x2c04);
        WriteGMII16( 0x19, 0x8300);
        WriteGMII16( 0x18, 0x2c14);
        WriteGMII16( 0x19, 0xe000);
        WriteGMII16( 0x18, 0x2c24);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x18, 0x2c34);
        WriteGMII16( 0x19, 0xac00);
        WriteGMII16( 0x18, 0x2c44);
        WriteGMII16( 0x19, 0x0800);
        WriteGMII16( 0x18, 0x2c54);
        WriteGMII16( 0x19, 0xfa00);
        WriteGMII16( 0x18, 0x2c64);
        WriteGMII16( 0x19, 0xe100);
        WriteGMII16( 0x18, 0x2c74);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x18, 0x0001);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x17, 0x2100);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0xd480);
        WriteGMII16( 0x06, 0xc1e4);
        WriteGMII16( 0x06, 0x8b9a);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x9bee);
        WriteGMII16( 0x06, 0x8b83);
        WriteGMII16( 0x06, 0x41bf);
        WriteGMII16( 0x06, 0x8b88);
        WriteGMII16( 0x06, 0xec00);
        WriteGMII16( 0x06, 0x19a9);
        WriteGMII16( 0x06, 0x8b90);
        WriteGMII16( 0x06, 0xf9ee);
        WriteGMII16( 0x06, 0xfff6);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xffe0);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0xe1e1);
        WriteGMII16( 0x06, 0x41f7);
        WriteGMII16( 0x06, 0x2ff6);
        WriteGMII16( 0x06, 0x28e4);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0xe5e1);
        WriteGMII16( 0x06, 0x41f7);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x020c);
        WriteGMII16( 0x06, 0x0202);
        WriteGMII16( 0x06, 0x1d02);
        WriteGMII16( 0x06, 0x0230);
        WriteGMII16( 0x06, 0x0202);
        WriteGMII16( 0x06, 0x4002);
        WriteGMII16( 0x06, 0x028b);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x6c02);
        WriteGMII16( 0x06, 0x8085);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x88e1);
        WriteGMII16( 0x06, 0x8b89);
        WriteGMII16( 0x06, 0x1e01);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x8a1e);
        WriteGMII16( 0x06, 0x01e1);
        WriteGMII16( 0x06, 0x8b8b);
        WriteGMII16( 0x06, 0x1e01);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x8c1e);
        WriteGMII16( 0x06, 0x01e1);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0x1e01);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x8e1e);
        WriteGMII16( 0x06, 0x01a0);
        WriteGMII16( 0x06, 0x00c7);
        WriteGMII16( 0x06, 0xaec3);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x10ee);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x1310);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0xc602);
        WriteGMII16( 0x06, 0x1f0c);
        WriteGMII16( 0x06, 0x0227);
        WriteGMII16( 0x06, 0x49fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x200b);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x852d);
        WriteGMII16( 0x06, 0x021b);
        WriteGMII16( 0x06, 0x67ad);
        WriteGMII16( 0x06, 0x2211);
        WriteGMII16( 0x06, 0xf622);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x2ba5);
        WriteGMII16( 0x06, 0x022a);
        WriteGMII16( 0x06, 0x2402);
        WriteGMII16( 0x06, 0x82e5);
        WriteGMII16( 0x06, 0x022a);
        WriteGMII16( 0x06, 0xf0ad);
        WriteGMII16( 0x06, 0x2511);
        WriteGMII16( 0x06, 0xf625);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x8445);
        WriteGMII16( 0x06, 0x0204);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x19cc);
        WriteGMII16( 0x06, 0x022b);
        WriteGMII16( 0x06, 0x5bfc);
        WriteGMII16( 0x06, 0x04ee);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0x0105);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfae0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac26);
        WriteGMII16( 0x06, 0x08e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x6bee);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0xe0eb);
        WriteGMII16( 0x06, 0x00e2);
        WriteGMII16( 0x06, 0xe07c);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x7da5);
        WriteGMII16( 0x06, 0x1111);
        WriteGMII16( 0x06, 0x15d2);
        WriteGMII16( 0x06, 0x60d6);
        WriteGMII16( 0x06, 0x6666);
        WriteGMII16( 0x06, 0x0207);
        WriteGMII16( 0x06, 0x6cd2);
        WriteGMII16( 0x06, 0xa0d6);
        WriteGMII16( 0x06, 0xaaaa);
        WriteGMII16( 0x06, 0x0207);
        WriteGMII16( 0x06, 0x6c02);
        WriteGMII16( 0x06, 0x201d);
        WriteGMII16( 0x06, 0xae44);
        WriteGMII16( 0x06, 0xa566);
        WriteGMII16( 0x06, 0x6602);
        WriteGMII16( 0x06, 0xae38);
        WriteGMII16( 0x06, 0xa5aa);
        WriteGMII16( 0x06, 0xaa02);
        WriteGMII16( 0x06, 0xae32);
        WriteGMII16( 0x06, 0xeee0);
        WriteGMII16( 0x06, 0xea04);
        WriteGMII16( 0x06, 0xeee0);
        WriteGMII16( 0x06, 0xeb06);
        WriteGMII16( 0x06, 0xe2e0);
        WriteGMII16( 0x06, 0x7ce3);
        WriteGMII16( 0x06, 0xe07d);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x38e1);
        WriteGMII16( 0x06, 0xe039);
        WriteGMII16( 0x06, 0xad2e);
        WriteGMII16( 0x06, 0x21ad);
        WriteGMII16( 0x06, 0x3f13);
        WriteGMII16( 0x06, 0xe0e4);
        WriteGMII16( 0x06, 0x14e1);
        WriteGMII16( 0x06, 0xe415);
        WriteGMII16( 0x06, 0x6880);
        WriteGMII16( 0x06, 0xe4e4);
        WriteGMII16( 0x06, 0x14e5);
        WriteGMII16( 0x06, 0xe415);
        WriteGMII16( 0x06, 0x0220);
        WriteGMII16( 0x06, 0x1dae);
        WriteGMII16( 0x06, 0x0bac);
        WriteGMII16( 0x06, 0x3e02);
        WriteGMII16( 0x06, 0xae06);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x4602);
        WriteGMII16( 0x06, 0x2057);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x20a7);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2109);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x2eac);
        WriteGMII16( 0x06, 0x2003);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x61fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ac);
        WriteGMII16( 0x06, 0x2505);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0xaeae);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x8172);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69fa);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x20a0);
        WriteGMII16( 0x06, 0x8016);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x21e1);
        WriteGMII16( 0x06, 0x8b33);
        WriteGMII16( 0x06, 0x1b10);
        WriteGMII16( 0x06, 0x9e06);
        WriteGMII16( 0x06, 0x0223);
        WriteGMII16( 0x06, 0x91af);
        WriteGMII16( 0x06, 0x8252);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x2081);
        WriteGMII16( 0x06, 0xaee4);
        WriteGMII16( 0x06, 0xa081);
        WriteGMII16( 0x06, 0x1402);
        WriteGMII16( 0x06, 0x2399);
        WriteGMII16( 0x06, 0xbf25);
        WriteGMII16( 0x06, 0xcc02);
        WriteGMII16( 0x06, 0x2d21);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x2100);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x2082);
        WriteGMII16( 0x06, 0xaf82);
        WriteGMII16( 0x06, 0x52a0);
        WriteGMII16( 0x06, 0x8232);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x21e1);
        WriteGMII16( 0x06, 0x8b32);
        WriteGMII16( 0x06, 0x1b10);
        WriteGMII16( 0x06, 0x9e06);
        WriteGMII16( 0x06, 0x0223);
        WriteGMII16( 0x06, 0x91af);
        WriteGMII16( 0x06, 0x8252);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x2100);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0x5910);
        WriteGMII16( 0x06, 0xa004);
        WriteGMII16( 0x06, 0xf9e0);
        WriteGMII16( 0x06, 0x861f);
        WriteGMII16( 0x06, 0xa000);
        WriteGMII16( 0x06, 0x07ee);
        WriteGMII16( 0x06, 0x8620);
        WriteGMII16( 0x06, 0x83af);
        WriteGMII16( 0x06, 0x8178);
        WriteGMII16( 0x06, 0x0224);
        WriteGMII16( 0x06, 0x0102);
        WriteGMII16( 0x06, 0x2399);
        WriteGMII16( 0x06, 0xae72);
        WriteGMII16( 0x06, 0xa083);
        WriteGMII16( 0x06, 0x4b1f);
        WriteGMII16( 0x06, 0x55d0);
        WriteGMII16( 0x06, 0x04bf);
        WriteGMII16( 0x06, 0x8615);
        WriteGMII16( 0x06, 0x1a90);
        WriteGMII16( 0x06, 0x0c54);
        WriteGMII16( 0x06, 0xd91e);
        WriteGMII16( 0x06, 0x31b0);
        WriteGMII16( 0x06, 0xf4e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x23ad);
        WriteGMII16( 0x06, 0x2e0c);
        WriteGMII16( 0x06, 0xef02);
        WriteGMII16( 0x06, 0xef12);
        WriteGMII16( 0x06, 0x0e44);
        WriteGMII16( 0x06, 0xef23);
        WriteGMII16( 0x06, 0x0e54);
        WriteGMII16( 0x06, 0xef21);
        WriteGMII16( 0x06, 0xe6e4);
        WriteGMII16( 0x06, 0x2ae7);
        WriteGMII16( 0x06, 0xe42b);
        WriteGMII16( 0x06, 0xe2e4);
        WriteGMII16( 0x06, 0x28e3);
        WriteGMII16( 0x06, 0xe429);
        WriteGMII16( 0x06, 0x6d20);
        WriteGMII16( 0x06, 0x00e6);
        WriteGMII16( 0x06, 0xe428);
        WriteGMII16( 0x06, 0xe7e4);
        WriteGMII16( 0x06, 0x29bf);
        WriteGMII16( 0x06, 0x25ca);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x21ee);
        WriteGMII16( 0x06, 0x8620);
        WriteGMII16( 0x06, 0x84ee);
        WriteGMII16( 0x06, 0x8621);
        WriteGMII16( 0x06, 0x00af);
        WriteGMII16( 0x06, 0x8178);
        WriteGMII16( 0x06, 0xa084);
        WriteGMII16( 0x06, 0x19e0);
        WriteGMII16( 0x06, 0x8621);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x341b);
        WriteGMII16( 0x06, 0x109e);
        WriteGMII16( 0x06, 0x0602);
        WriteGMII16( 0x06, 0x2391);
        WriteGMII16( 0x06, 0xaf82);
        WriteGMII16( 0x06, 0x5202);
        WriteGMII16( 0x06, 0x241f);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x2085);
        WriteGMII16( 0x06, 0xae08);
        WriteGMII16( 0x06, 0xa085);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x2442);
        WriteGMII16( 0x06, 0xfeef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xfad1);
        WriteGMII16( 0x06, 0x801f);
        WriteGMII16( 0x06, 0x66e2);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0xeb5a);
        WriteGMII16( 0x06, 0xf81e);
        WriteGMII16( 0x06, 0x20e6);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe5e0);
        WriteGMII16( 0x06, 0xebd3);
        WriteGMII16( 0x06, 0x05b3);
        WriteGMII16( 0x06, 0xfee2);
        WriteGMII16( 0x06, 0xe07c);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x7dad);
        WriteGMII16( 0x06, 0x3703);
        WriteGMII16( 0x06, 0x7dff);
        WriteGMII16( 0x06, 0xff0d);
        WriteGMII16( 0x06, 0x581c);
        WriteGMII16( 0x06, 0x55f8);
        WriteGMII16( 0x06, 0xef46);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xc7ef);
        WriteGMII16( 0x06, 0x65ef);
        WriteGMII16( 0x06, 0x54fc);
        WriteGMII16( 0x06, 0xac30);
        WriteGMII16( 0x06, 0x2b11);
        WriteGMII16( 0x06, 0xa188);
        WriteGMII16( 0x06, 0xcabf);
        WriteGMII16( 0x06, 0x860e);
        WriteGMII16( 0x06, 0xef10);
        WriteGMII16( 0x06, 0x0c11);
        WriteGMII16( 0x06, 0x1a91);
        WriteGMII16( 0x06, 0xda19);
        WriteGMII16( 0x06, 0xdbf8);
        WriteGMII16( 0x06, 0xef46);
        WriteGMII16( 0x06, 0x021e);
        WriteGMII16( 0x06, 0x17ef);
        WriteGMII16( 0x06, 0x54fc);
        WriteGMII16( 0x06, 0xad30);
        WriteGMII16( 0x06, 0x0fef);
        WriteGMII16( 0x06, 0x5689);
        WriteGMII16( 0x06, 0xde19);
        WriteGMII16( 0x06, 0xdfe2);
        WriteGMII16( 0x06, 0x861f);
        WriteGMII16( 0x06, 0xbf86);
        WriteGMII16( 0x06, 0x161a);
        WriteGMII16( 0x06, 0x90de);
        WriteGMII16( 0x06, 0xfeef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04ac);
        WriteGMII16( 0x06, 0x2707);
        WriteGMII16( 0x06, 0xac37);
        WriteGMII16( 0x06, 0x071a);
        WriteGMII16( 0x06, 0x54ae);
        WriteGMII16( 0x06, 0x11ac);
        WriteGMII16( 0x06, 0x3707);
        WriteGMII16( 0x06, 0xae00);
        WriteGMII16( 0x06, 0x1a54);
        WriteGMII16( 0x06, 0xac37);
        WriteGMII16( 0x06, 0x07d0);
        WriteGMII16( 0x06, 0x01d5);
        WriteGMII16( 0x06, 0xffff);
        WriteGMII16( 0x06, 0xae02);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x83ad);
        WriteGMII16( 0x06, 0x2444);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x22e1);
        WriteGMII16( 0x06, 0xe023);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x3be0);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0xa000);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x28de);
        WriteGMII16( 0x06, 0xae42);
        WriteGMII16( 0x06, 0xa001);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x28f1);
        WriteGMII16( 0x06, 0xae3a);
        WriteGMII16( 0x06, 0xa002);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x8344);
        WriteGMII16( 0x06, 0xae32);
        WriteGMII16( 0x06, 0xa003);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x299a);
        WriteGMII16( 0x06, 0xae2a);
        WriteGMII16( 0x06, 0xa004);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x29ae);
        WriteGMII16( 0x06, 0xae22);
        WriteGMII16( 0x06, 0xa005);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x29d7);
        WriteGMII16( 0x06, 0xae1a);
        WriteGMII16( 0x06, 0xa006);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x29fe);
        WriteGMII16( 0x06, 0xae12);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc000);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc100);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc600);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xbe00);
        WriteGMII16( 0x06, 0xae00);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf802);
        WriteGMII16( 0x06, 0x2a67);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x22e1);
        WriteGMII16( 0x06, 0xe023);
        WriteGMII16( 0x06, 0x0d06);
        WriteGMII16( 0x06, 0x5803);
        WriteGMII16( 0x06, 0xa002);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x2da0);
        WriteGMII16( 0x06, 0x0102);
        WriteGMII16( 0x06, 0xae2d);
        WriteGMII16( 0x06, 0xa000);
        WriteGMII16( 0x06, 0x4de0);
        WriteGMII16( 0x06, 0xe200);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0x01ad);
        WriteGMII16( 0x06, 0x2444);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xc2e4);
        WriteGMII16( 0x06, 0x8ac4);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xc3e4);
        WriteGMII16( 0x06, 0x8ac5);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xbe03);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x83ad);
        WriteGMII16( 0x06, 0x253a);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xbe05);
        WriteGMII16( 0x06, 0xae34);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xceae);
        WriteGMII16( 0x06, 0x03e0);
        WriteGMII16( 0x06, 0x8acf);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xc249);
        WriteGMII16( 0x06, 0x05e5);
        WriteGMII16( 0x06, 0x8ac4);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xc349);
        WriteGMII16( 0x06, 0x05e5);
        WriteGMII16( 0x06, 0x8ac5);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xbe05);
        WriteGMII16( 0x06, 0x022a);
        WriteGMII16( 0x06, 0xb6ac);
        WriteGMII16( 0x06, 0x2012);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0xbaac);
        WriteGMII16( 0x06, 0x200c);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc100);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc600);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xbe02);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0xcc59);
        WriteGMII16( 0x06, 0x0f39);
        WriteGMII16( 0x06, 0x02aa);
        WriteGMII16( 0x06, 0x04d0);
        WriteGMII16( 0x06, 0x01ae);
        WriteGMII16( 0x06, 0x02d0);
        WriteGMII16( 0x06, 0x0004);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xe2e2);
        WriteGMII16( 0x06, 0xd2e3);
        WriteGMII16( 0x06, 0xe2d3);
        WriteGMII16( 0x06, 0xf95a);
        WriteGMII16( 0x06, 0xf7e6);
        WriteGMII16( 0x06, 0xe2d2);
        WriteGMII16( 0x06, 0xe7e2);
        WriteGMII16( 0x06, 0xd3e2);
        WriteGMII16( 0x06, 0xe02c);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x2df9);
        WriteGMII16( 0x06, 0x5be0);
        WriteGMII16( 0x06, 0x1e30);
        WriteGMII16( 0x06, 0xe6e0);
        WriteGMII16( 0x06, 0x2ce7);
        WriteGMII16( 0x06, 0xe02d);
        WriteGMII16( 0x06, 0xe2e2);
        WriteGMII16( 0x06, 0xcce3);
        WriteGMII16( 0x06, 0xe2cd);
        WriteGMII16( 0x06, 0xf95a);
        WriteGMII16( 0x06, 0x0f6a);
        WriteGMII16( 0x06, 0x50e6);
        WriteGMII16( 0x06, 0xe2cc);
        WriteGMII16( 0x06, 0xe7e2);
        WriteGMII16( 0x06, 0xcde0);
        WriteGMII16( 0x06, 0xe03c);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x3def);
        WriteGMII16( 0x06, 0x64fd);
        WriteGMII16( 0x06, 0xe0e2);
        WriteGMII16( 0x06, 0xcce1);
        WriteGMII16( 0x06, 0xe2cd);
        WriteGMII16( 0x06, 0x580f);
        WriteGMII16( 0x06, 0x5af0);
        WriteGMII16( 0x06, 0x1e02);
        WriteGMII16( 0x06, 0xe4e2);
        WriteGMII16( 0x06, 0xcce5);
        WriteGMII16( 0x06, 0xe2cd);
        WriteGMII16( 0x06, 0xfde0);
        WriteGMII16( 0x06, 0xe02c);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x2d59);
        WriteGMII16( 0x06, 0xe05b);
        WriteGMII16( 0x06, 0x1f1e);
        WriteGMII16( 0x06, 0x13e4);
        WriteGMII16( 0x06, 0xe02c);
        WriteGMII16( 0x06, 0xe5e0);
        WriteGMII16( 0x06, 0x2dfd);
        WriteGMII16( 0x06, 0xe0e2);
        WriteGMII16( 0x06, 0xd2e1);
        WriteGMII16( 0x06, 0xe2d3);
        WriteGMII16( 0x06, 0x58f7);
        WriteGMII16( 0x06, 0x5a08);
        WriteGMII16( 0x06, 0x1e02);
        WriteGMII16( 0x06, 0xe4e2);
        WriteGMII16( 0x06, 0xd2e5);
        WriteGMII16( 0x06, 0xe2d3);
        WriteGMII16( 0x06, 0xef46);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x22e1);
        WriteGMII16( 0x06, 0xe023);
        WriteGMII16( 0x06, 0x58c4);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x6e1f);
        WriteGMII16( 0x06, 0x109e);
        WriteGMII16( 0x06, 0x58e4);
        WriteGMII16( 0x06, 0x8b6e);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x22ac);
        WriteGMII16( 0x06, 0x2755);
        WriteGMII16( 0x06, 0xac26);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x1ad1);
        WriteGMII16( 0x06, 0x06bf);
        WriteGMII16( 0x06, 0x3bba);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x07bf);
        WriteGMII16( 0x06, 0x3bbd);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x07bf);
        WriteGMII16( 0x06, 0x3bc0);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1ae);
        WriteGMII16( 0x06, 0x30d1);
        WriteGMII16( 0x06, 0x03bf);
        WriteGMII16( 0x06, 0x3bc3);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x3bc6);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x84e9);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x0fbf);
        WriteGMII16( 0x06, 0x3bba);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x3bbd);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x3bc0);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x3bc3);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d0);
        WriteGMII16( 0x06, 0x1102);
        WriteGMII16( 0x06, 0x2bfb);
        WriteGMII16( 0x06, 0x5903);
        WriteGMII16( 0x06, 0xef01);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xa000);
        WriteGMII16( 0x06, 0x02d1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x3bc6);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1d1);
        WriteGMII16( 0x06, 0x11ad);
        WriteGMII16( 0x06, 0x2002);
        WriteGMII16( 0x06, 0x0c11);
        WriteGMII16( 0x06, 0xad21);
        WriteGMII16( 0x06, 0x020c);
        WriteGMII16( 0x06, 0x12bf);
        WriteGMII16( 0x06, 0x84e9);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1ae);
        WriteGMII16( 0x06, 0xc870);
        WriteGMII16( 0x06, 0xe426);
        WriteGMII16( 0x06, 0x0284);
        WriteGMII16( 0x06, 0xf005);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe0e2);
        WriteGMII16( 0x06, 0xfee1);
        WriteGMII16( 0x06, 0xe2ff);
        WriteGMII16( 0x06, 0xad2d);
        WriteGMII16( 0x06, 0x1ae0);
        WriteGMII16( 0x06, 0xe14e);
        WriteGMII16( 0x06, 0xe1e1);
        WriteGMII16( 0x06, 0x4fac);
        WriteGMII16( 0x06, 0x2d22);
        WriteGMII16( 0x06, 0xf603);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0x3bf7);
        WriteGMII16( 0x06, 0x03f7);
        WriteGMII16( 0x06, 0x06bf);
        WriteGMII16( 0x06, 0x8561);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x21ae);
        WriteGMII16( 0x06, 0x11e0);
        WriteGMII16( 0x06, 0xe14e);
        WriteGMII16( 0x06, 0xe1e1);
        WriteGMII16( 0x06, 0x4fad);
        WriteGMII16( 0x06, 0x2d08);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0x6c02);
        WriteGMII16( 0x06, 0x2d21);
        WriteGMII16( 0x06, 0xf606);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0xe000);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x01ad);
        WriteGMII16( 0x06, 0x271f);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0x5e02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x20e1);
        WriteGMII16( 0x06, 0xe021);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x0ed1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x855e);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1bf);
        WriteGMII16( 0x06, 0x3b96);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x21ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0x00e2);
        WriteGMII16( 0x06, 0x34a7);
        WriteGMII16( 0x06, 0x25e5);
        WriteGMII16( 0x06, 0x0a1d);
        WriteGMII16( 0x06, 0xe50a);
        WriteGMII16( 0x06, 0x2ce5);
        WriteGMII16( 0x06, 0x0a6d);
        WriteGMII16( 0x06, 0xe50a);
        WriteGMII16( 0x06, 0x1de5);
        WriteGMII16( 0x06, 0x0a1c);
        WriteGMII16( 0x06, 0xe50a);
        WriteGMII16( 0x06, 0x2da7);
        WriteGMII16( 0x06, 0x5500);
        WriteGMII16( 0x05, 0x8b94);
        WriteGMII16( 0x06, 0x84ec);
        gphy_val = ReadGMII16( 0x01);
        gphy_val |= BIT_0;
        WriteGMII16( 0x01, gphy_val);
        WriteGMII16( 0x00, 0x0005);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x17, 0x0116);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0028);
        WriteGMII16( 0x15, 0x0010);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0020);
        WriteGMII16( 0x15, 0x0100);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0041);
        WriteGMII16( 0x15, 0x0802);
        WriteGMII16( 0x16, 0x2185);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
               mcfg == MCFG_8168DP_3) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x18, 0x0310);
        WriteGMII16( 0x1F, 0x0000);

        mdelay(20);

        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x19, 0x7070);
        WriteGMII16( 0x1c, 0x0600);
        WriteGMII16( 0x1d, 0x9700);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6900);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x4899);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8000);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x4007);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x4800);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x571f);
        WriteGMII16( 0x1d, 0x5ffb);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x301e);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0xa6fc);
        WriteGMII16( 0x1d, 0xdcdb);
        WriteGMII16( 0x1d, 0x0014);
        WriteGMII16( 0x1d, 0xd9a9);
        WriteGMII16( 0x1d, 0x0013);
        WriteGMII16( 0x1d, 0xd16b);
        WriteGMII16( 0x1d, 0x0011);
        WriteGMII16( 0x1d, 0xb40e);
        WriteGMII16( 0x1d, 0xd06b);
        WriteGMII16( 0x1d, 0x000c);
        WriteGMII16( 0x1d, 0xb206);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c00);
        WriteGMII16( 0x1d, 0x301a);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5801);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c04);
        WriteGMII16( 0x1d, 0x301e);
        WriteGMII16( 0x1d, 0x314d);
        WriteGMII16( 0x1d, 0x31f0);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c20);
        WriteGMII16( 0x1d, 0x6004);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x4833);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c08);
        WriteGMII16( 0x1d, 0x8300);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6600);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xb90c);
        WriteGMII16( 0x1d, 0x30d3);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4de0);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x300b);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c60);
        WriteGMII16( 0x1d, 0x6803);
        WriteGMII16( 0x1d, 0x6520);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xaf03);
        WriteGMII16( 0x1d, 0x6015);
        WriteGMII16( 0x1d, 0x3059);
        WriteGMII16( 0x1d, 0x6017);
        WriteGMII16( 0x1d, 0x57e0);
        WriteGMII16( 0x1d, 0x580c);
        WriteGMII16( 0x1d, 0x588c);
        WriteGMII16( 0x1d, 0x7ffc);
        WriteGMII16( 0x1d, 0x5fa3);
        WriteGMII16( 0x1d, 0x4827);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x7c30);
        WriteGMII16( 0x1d, 0x6020);
        WriteGMII16( 0x1d, 0x48bf);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0xad09);
        WriteGMII16( 0x1d, 0x7c03);
        WriteGMII16( 0x1d, 0x5c03);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0xad2c);
        WriteGMII16( 0x1d, 0xd6cf);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0x80f4);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c80);
        WriteGMII16( 0x1d, 0x7c20);
        WriteGMII16( 0x1d, 0x5c20);
        WriteGMII16( 0x1d, 0x481e);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c02);
        WriteGMII16( 0x1d, 0xad0a);
        WriteGMII16( 0x1d, 0x7c03);
        WriteGMII16( 0x1d, 0x5c03);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x8d02);
        WriteGMII16( 0x1d, 0x4401);
        WriteGMII16( 0x1d, 0x81f4);
        WriteGMII16( 0x1d, 0x3114);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d00);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0xa4b7);
        WriteGMII16( 0x1d, 0xd9b3);
        WriteGMII16( 0x1d, 0xfffe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d20);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0x3045);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d40);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x4401);
        WriteGMII16( 0x1d, 0x5210);
        WriteGMII16( 0x1d, 0x4833);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x4c08);
        WriteGMII16( 0x1d, 0x8300);
        WriteGMII16( 0x1d, 0x5f80);
        WriteGMII16( 0x1d, 0x55e0);
        WriteGMII16( 0x1d, 0xc06f);
        WriteGMII16( 0x1d, 0x0005);
        WriteGMII16( 0x1d, 0xd9b3);
        WriteGMII16( 0x1d, 0xfffd);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x6040);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d60);
        WriteGMII16( 0x1d, 0x57e0);
        WriteGMII16( 0x1d, 0x4814);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7c03);
        WriteGMII16( 0x1d, 0x5c03);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xad02);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0xc0e9);
        WriteGMII16( 0x1d, 0x0003);
        WriteGMII16( 0x1d, 0xadd8);
        WriteGMII16( 0x1d, 0x30c6);
        WriteGMII16( 0x1d, 0x3078);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4dc0);
        WriteGMII16( 0x1d, 0x6730);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xd09d);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0xb4fe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d80);
        WriteGMII16( 0x1d, 0x6802);
        WriteGMII16( 0x1d, 0x6600);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x486c);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x9503);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0x571f);
        WriteGMII16( 0x1d, 0x5fbb);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x30e9);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0xcdab);
        WriteGMII16( 0x1d, 0xff5b);
        WriteGMII16( 0x1d, 0xcd8d);
        WriteGMII16( 0x1d, 0xff59);
        WriteGMII16( 0x1d, 0xd96b);
        WriteGMII16( 0x1d, 0xff57);
        WriteGMII16( 0x1d, 0xd0a0);
        WriteGMII16( 0x1d, 0xffdb);
        WriteGMII16( 0x1d, 0xcba0);
        WriteGMII16( 0x1d, 0x0003);
        WriteGMII16( 0x1d, 0x80f0);
        WriteGMII16( 0x1d, 0x30f6);
        WriteGMII16( 0x1d, 0x3109);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ce0);
        WriteGMII16( 0x1d, 0x7d30);
        WriteGMII16( 0x1d, 0x6530);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7ce0);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c08);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6008);
        WriteGMII16( 0x1d, 0x8300);
        WriteGMII16( 0x1d, 0xb902);
        WriteGMII16( 0x1d, 0x30d3);
        WriteGMII16( 0x1d, 0x308f);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4da0);
        WriteGMII16( 0x1d, 0x57a0);
        WriteGMII16( 0x1d, 0x590c);
        WriteGMII16( 0x1d, 0x5fa2);
        WriteGMII16( 0x1d, 0xcba4);
        WriteGMII16( 0x1d, 0x0005);
        WriteGMII16( 0x1d, 0xcd8d);
        WriteGMII16( 0x1d, 0x0003);
        WriteGMII16( 0x1d, 0x80fc);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ca0);
        WriteGMII16( 0x1d, 0xb603);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6010);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x541f);
        WriteGMII16( 0x1d, 0x7ffc);
        WriteGMII16( 0x1d, 0x5fb3);
        WriteGMII16( 0x1d, 0x9403);
        WriteGMII16( 0x1d, 0x7c03);
        WriteGMII16( 0x1d, 0x5c03);
        WriteGMII16( 0x1d, 0xaa05);
        WriteGMII16( 0x1d, 0x7c80);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x3128);
        WriteGMII16( 0x1d, 0x7c80);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0x4827);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4cc0);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6400);
        WriteGMII16( 0x1d, 0x7ffc);
        WriteGMII16( 0x1d, 0x5fbb);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6a00);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x30f6);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e00);
        WriteGMII16( 0x1d, 0x4007);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x570f);
        WriteGMII16( 0x1d, 0x5fff);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x585b);
        WriteGMII16( 0x1d, 0x315c);
        WriteGMII16( 0x1d, 0x5867);
        WriteGMII16( 0x1d, 0x9402);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0xcda3);
        WriteGMII16( 0x1d, 0x009d);
        WriteGMII16( 0x1d, 0xcd85);
        WriteGMII16( 0x1d, 0x009b);
        WriteGMII16( 0x1d, 0xd96b);
        WriteGMII16( 0x1d, 0x0099);
        WriteGMII16( 0x1d, 0x96e9);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e20);
        WriteGMII16( 0x1d, 0x96e4);
        WriteGMII16( 0x1d, 0x8b04);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x5008);
        WriteGMII16( 0x1d, 0xab03);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x5000);
        WriteGMII16( 0x1d, 0x6801);
        WriteGMII16( 0x1d, 0x6776);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xdb7c);
        WriteGMII16( 0x1d, 0xfff0);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7fe1);
        WriteGMII16( 0x1d, 0x4e40);
        WriteGMII16( 0x1d, 0x4837);
        WriteGMII16( 0x1d, 0x4418);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e40);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8fc9);
        WriteGMII16( 0x1d, 0xd2a0);
        WriteGMII16( 0x1d, 0x004a);
        WriteGMII16( 0x1d, 0x9203);
        WriteGMII16( 0x1d, 0xa041);
        WriteGMII16( 0x1d, 0x3184);
        WriteGMII16( 0x1d, 0x7fe1);
        WriteGMII16( 0x1d, 0x4e60);
        WriteGMII16( 0x1d, 0x489c);
        WriteGMII16( 0x1d, 0x4628);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e60);
        WriteGMII16( 0x1d, 0x7e28);
        WriteGMII16( 0x1d, 0x4628);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c00);
        WriteGMII16( 0x1d, 0x41e8);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8fb0);
        WriteGMII16( 0x1d, 0xb241);
        WriteGMII16( 0x1d, 0xa02a);
        WriteGMII16( 0x1d, 0x319d);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ea0);
        WriteGMII16( 0x1d, 0x7c02);
        WriteGMII16( 0x1d, 0x4402);
        WriteGMII16( 0x1d, 0x4448);
        WriteGMII16( 0x1d, 0x4894);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c03);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c07);
        WriteGMII16( 0x1d, 0x41ef);
        WriteGMII16( 0x1d, 0x41ff);
        WriteGMII16( 0x1d, 0x4891);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c07);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c17);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x8ef8);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x8f95);
        WriteGMII16( 0x1d, 0x92d5);
        WriteGMII16( 0x1d, 0xa10f);
        WriteGMII16( 0x1d, 0xd480);
        WriteGMII16( 0x1d, 0x0008);
        WriteGMII16( 0x1d, 0xd580);
        WriteGMII16( 0x1d, 0xffb9);
        WriteGMII16( 0x1d, 0xa202);
        WriteGMII16( 0x1d, 0x31b8);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x4404);
        WriteGMII16( 0x1d, 0x31b8);
        WriteGMII16( 0x1d, 0xd484);
        WriteGMII16( 0x1d, 0xfff3);
        WriteGMII16( 0x1d, 0xd484);
        WriteGMII16( 0x1d, 0xfff1);
        WriteGMII16( 0x1d, 0x314d);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ee0);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x4488);
        WriteGMII16( 0x1d, 0x41cf);
        WriteGMII16( 0x1d, 0x314d);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ec0);
        WriteGMII16( 0x1d, 0x48f3);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c09);
        WriteGMII16( 0x1d, 0x4508);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x8f24);
        WriteGMII16( 0x1d, 0xd218);
        WriteGMII16( 0x1d, 0x0022);
        WriteGMII16( 0x1d, 0xd2a4);
        WriteGMII16( 0x1d, 0xff9f);
        WriteGMII16( 0x1d, 0x31d9);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e80);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c11);
        WriteGMII16( 0x1d, 0x4428);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5440);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5801);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c04);
        WriteGMII16( 0x1d, 0x41e8);
        WriteGMII16( 0x1d, 0xa4b3);
        WriteGMII16( 0x1d, 0x31ee);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x570f);
        WriteGMII16( 0x1d, 0x5fff);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x585b);
        WriteGMII16( 0x1d, 0x31fa);
        WriteGMII16( 0x1d, 0x5867);
        WriteGMII16( 0x1d, 0xbcf6);
        WriteGMII16( 0x1d, 0x300b);
        WriteGMII16( 0x1d, 0x300b);
        WriteGMII16( 0x1d, 0x314d);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1c, 0x0200);
        WriteGMII16( 0x19, 0x7030);
        WriteGMII16( 0x1f, 0x0000);


    } else if (mcfg == MCFG_8402_1) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x18, 0x0310);
        WriteGMII16( 0x1F, 0x0000);

        mdelay(20);

        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x19, 0x7070);
        WriteGMII16( 0x1c, 0x0600);
        WriteGMII16( 0x1d, 0x9700);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6900);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x4899);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8000);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x4007);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x4800);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x571f);
        WriteGMII16( 0x1d, 0x5ffb);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x301e);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0xa6fc);
        WriteGMII16( 0x1d, 0xdcdb);
        WriteGMII16( 0x1d, 0x0015);
        WriteGMII16( 0x1d, 0xb915);
        WriteGMII16( 0x1d, 0xb511);
        WriteGMII16( 0x1d, 0xd16b);
        WriteGMII16( 0x1d, 0x000f);
        WriteGMII16( 0x1d, 0xb40f);
        WriteGMII16( 0x1d, 0xd06b);
        WriteGMII16( 0x1d, 0x000d);
        WriteGMII16( 0x1d, 0xb206);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c00);
        WriteGMII16( 0x1d, 0x301a);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5801);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c04);
        WriteGMII16( 0x1d, 0x301e);
        WriteGMII16( 0x1d, 0x3079);
        WriteGMII16( 0x1d, 0x30f1);
        WriteGMII16( 0x1d, 0x3199);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c60);
        WriteGMII16( 0x1d, 0x6803);
        WriteGMII16( 0x1d, 0x6420);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xaf03);
        WriteGMII16( 0x1d, 0x6015);
        WriteGMII16( 0x1d, 0x3040);
        WriteGMII16( 0x1d, 0x6017);
        WriteGMII16( 0x1d, 0x57e0);
        WriteGMII16( 0x1d, 0x580c);
        WriteGMII16( 0x1d, 0x588c);
        WriteGMII16( 0x1d, 0x5fa3);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x4827);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x7c30);
        WriteGMII16( 0x1d, 0x6020);
        WriteGMII16( 0x1d, 0x48bf);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0xd6cf);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0x80fe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c80);
        WriteGMII16( 0x1d, 0x7c20);
        WriteGMII16( 0x1d, 0x5c20);
        WriteGMII16( 0x1d, 0x481e);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c02);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x81ff);
        WriteGMII16( 0x1d, 0x30ba);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d00);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0xa4cc);
        WriteGMII16( 0x1d, 0xd9b3);
        WriteGMII16( 0x1d, 0xfffe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d20);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0x300b);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4dc0);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xd09d);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0xb4fe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d80);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x6004);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6802);
        WriteGMII16( 0x1d, 0x6720);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x486c);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x9503);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0x571f);
        WriteGMII16( 0x1d, 0x5fbb);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x3092);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0xcdab);
        WriteGMII16( 0x1d, 0xff78);
        WriteGMII16( 0x1d, 0xcd8d);
        WriteGMII16( 0x1d, 0xff76);
        WriteGMII16( 0x1d, 0xd96b);
        WriteGMII16( 0x1d, 0xff74);
        WriteGMII16( 0x1d, 0xd0a0);
        WriteGMII16( 0x1d, 0xffd9);
        WriteGMII16( 0x1d, 0xcba0);
        WriteGMII16( 0x1d, 0x0003);
        WriteGMII16( 0x1d, 0x80f0);
        WriteGMII16( 0x1d, 0x309f);
        WriteGMII16( 0x1d, 0x30ac);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ce0);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c08);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6008);
        WriteGMII16( 0x1d, 0x8300);
        WriteGMII16( 0x1d, 0xb902);
        WriteGMII16( 0x1d, 0x3079);
        WriteGMII16( 0x1d, 0x3061);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4da0);
        WriteGMII16( 0x1d, 0x6400);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x57a0);
        WriteGMII16( 0x1d, 0x590c);
        WriteGMII16( 0x1d, 0x5fa3);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xcba4);
        WriteGMII16( 0x1d, 0x0004);
        WriteGMII16( 0x1d, 0xcd8d);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0x80fc);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ca0);
        WriteGMII16( 0x1d, 0xb603);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6010);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x541f);
        WriteGMII16( 0x1d, 0x5fb3);
        WriteGMII16( 0x1d, 0xaa05);
        WriteGMII16( 0x1d, 0x7c80);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x30ca);
        WriteGMII16( 0x1d, 0x7c80);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x4827);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4cc0);
        WriteGMII16( 0x1d, 0x5fbb);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7ce0);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x6720);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6a00);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x309f);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e00);
        WriteGMII16( 0x1d, 0x4007);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x570f);
        WriteGMII16( 0x1d, 0x5fff);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x585b);
        WriteGMII16( 0x1d, 0x3100);
        WriteGMII16( 0x1d, 0x5867);
        WriteGMII16( 0x1d, 0x9403);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0xcda3);
        WriteGMII16( 0x1d, 0x002d);
        WriteGMII16( 0x1d, 0xcd85);
        WriteGMII16( 0x1d, 0x002b);
        WriteGMII16( 0x1d, 0xd96b);
        WriteGMII16( 0x1d, 0x0029);
        WriteGMII16( 0x1d, 0x9629);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x9624);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e20);
        WriteGMII16( 0x1d, 0x8b04);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x5008);
        WriteGMII16( 0x1d, 0xab03);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x5000);
        WriteGMII16( 0x1d, 0x6801);
        WriteGMII16( 0x1d, 0x6776);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xdb7c);
        WriteGMII16( 0x1d, 0xffee);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7fe1);
        WriteGMII16( 0x1d, 0x4e40);
        WriteGMII16( 0x1d, 0x4837);
        WriteGMII16( 0x1d, 0x4418);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e40);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8f07);
        WriteGMII16( 0x1d, 0xd2a0);
        WriteGMII16( 0x1d, 0x004c);
        WriteGMII16( 0x1d, 0x9205);
        WriteGMII16( 0x1d, 0xa043);
        WriteGMII16( 0x1d, 0x312b);
        WriteGMII16( 0x1d, 0x300b);
        WriteGMII16( 0x1d, 0x30f1);
        WriteGMII16( 0x1d, 0x7fe1);
        WriteGMII16( 0x1d, 0x4e60);
        WriteGMII16( 0x1d, 0x489c);
        WriteGMII16( 0x1d, 0x4628);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e60);
        WriteGMII16( 0x1d, 0x7e28);
        WriteGMII16( 0x1d, 0x4628);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c00);
        WriteGMII16( 0x1d, 0x41e8);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8fec);
        WriteGMII16( 0x1d, 0xb241);
        WriteGMII16( 0x1d, 0xa02a);
        WriteGMII16( 0x1d, 0x3146);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ea0);
        WriteGMII16( 0x1d, 0x7c02);
        WriteGMII16( 0x1d, 0x4402);
        WriteGMII16( 0x1d, 0x4448);
        WriteGMII16( 0x1d, 0x4894);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c03);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c07);
        WriteGMII16( 0x1d, 0x41ef);
        WriteGMII16( 0x1d, 0x41ff);
        WriteGMII16( 0x1d, 0x4891);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c07);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c17);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x8ef8);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x8fd1);
        WriteGMII16( 0x1d, 0x92d5);
        WriteGMII16( 0x1d, 0xa10f);
        WriteGMII16( 0x1d, 0xd480);
        WriteGMII16( 0x1d, 0x0008);
        WriteGMII16( 0x1d, 0xd580);
        WriteGMII16( 0x1d, 0xffb7);
        WriteGMII16( 0x1d, 0xa202);
        WriteGMII16( 0x1d, 0x3161);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x4404);
        WriteGMII16( 0x1d, 0x3161);
        WriteGMII16( 0x1d, 0xd484);
        WriteGMII16( 0x1d, 0xfff3);
        WriteGMII16( 0x1d, 0xd484);
        WriteGMII16( 0x1d, 0xfff1);
        WriteGMII16( 0x1d, 0x30f1);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ee0);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x4488);
        WriteGMII16( 0x1d, 0x41cf);
        WriteGMII16( 0x1d, 0x30f1);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ec0);
        WriteGMII16( 0x1d, 0x48f3);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c09);
        WriteGMII16( 0x1d, 0x4508);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x8fb0);
        WriteGMII16( 0x1d, 0xd218);
        WriteGMII16( 0x1d, 0xffae);
        WriteGMII16( 0x1d, 0xd2a4);
        WriteGMII16( 0x1d, 0xff9d);
        WriteGMII16( 0x1d, 0x3182);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e80);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c11);
        WriteGMII16( 0x1d, 0x4428);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5440);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5801);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c04);
        WriteGMII16( 0x1d, 0x41e8);
        WriteGMII16( 0x1d, 0xa4b3);
        WriteGMII16( 0x1d, 0x3197);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4f20);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x6736);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x570f);
        WriteGMII16( 0x1d, 0x5fff);
        WriteGMII16( 0x1d, 0xaa03);
        WriteGMII16( 0x1d, 0x585b);
        WriteGMII16( 0x1d, 0x31a5);
        WriteGMII16( 0x1d, 0x5867);
        WriteGMII16( 0x1d, 0xbcf4);
        WriteGMII16( 0x1d, 0x300b);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1c, 0x0200);
        WriteGMII16( 0x19, 0x7030);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == MCFG_8168E_VL_1) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x18, 0x0310);

        mdelay(20);

        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x19, 0x7070);
        WriteGMII16( 0x1c, 0x0600);
        WriteGMII16( 0x1d, 0x9700);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x4007);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x4800);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x673e);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x571f);
        WriteGMII16( 0x1d, 0x5ffb);
        WriteGMII16( 0x1d, 0xaa04);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x6100);
        WriteGMII16( 0x1d, 0x3016);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0x6080);
        WriteGMII16( 0x1d, 0xa6fa);
        WriteGMII16( 0x1d, 0xdcdb);
        WriteGMII16( 0x1d, 0x0015);
        WriteGMII16( 0x1d, 0xb915);
        WriteGMII16( 0x1d, 0xb511);
        WriteGMII16( 0x1d, 0xd16b);
        WriteGMII16( 0x1d, 0x000f);
        WriteGMII16( 0x1d, 0xb40f);
        WriteGMII16( 0x1d, 0xd06b);
        WriteGMII16( 0x1d, 0x000d);
        WriteGMII16( 0x1d, 0xb206);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c00);
        WriteGMII16( 0x1d, 0x3010);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5801);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c04);
        WriteGMII16( 0x1d, 0x3016);
        WriteGMII16( 0x1d, 0x307e);
        WriteGMII16( 0x1d, 0x30f4);
        WriteGMII16( 0x1d, 0x319f);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c60);
        WriteGMII16( 0x1d, 0x6803);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6900);
        WriteGMII16( 0x1d, 0x6520);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xaf03);
        WriteGMII16( 0x1d, 0x6115);
        WriteGMII16( 0x1d, 0x303a);
        WriteGMII16( 0x1d, 0x6097);
        WriteGMII16( 0x1d, 0x57e0);
        WriteGMII16( 0x1d, 0x580c);
        WriteGMII16( 0x1d, 0x588c);
        WriteGMII16( 0x1d, 0x5f80);
        WriteGMII16( 0x1d, 0x4827);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x7c30);
        WriteGMII16( 0x1d, 0x6020);
        WriteGMII16( 0x1d, 0x48bf);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0xb802);
        WriteGMII16( 0x1d, 0x3053);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6808);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6810);
        WriteGMII16( 0x1d, 0xd6cf);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0x80fe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4c80);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7c23);
        WriteGMII16( 0x1d, 0x5c23);
        WriteGMII16( 0x1d, 0x481e);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c02);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x81ff);
        WriteGMII16( 0x1d, 0x30c1);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d00);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0xa4bd);
        WriteGMII16( 0x1d, 0xd9b3);
        WriteGMII16( 0x1d, 0x00fe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d20);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0x3001);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4dc0);
        WriteGMII16( 0x1d, 0xd09d);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0xb4fe);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4d80);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x6004);
        WriteGMII16( 0x1d, 0x6802);
        WriteGMII16( 0x1d, 0x6728);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x486c);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x9503);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0x571f);
        WriteGMII16( 0x1d, 0x5fbb);
        WriteGMII16( 0x1d, 0xaa05);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x7d80);
        WriteGMII16( 0x1d, 0x6100);
        WriteGMII16( 0x1d, 0x309a);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0x7d80);
        WriteGMII16( 0x1d, 0x6080);
        WriteGMII16( 0x1d, 0xcdab);
        WriteGMII16( 0x1d, 0x0058);
        WriteGMII16( 0x1d, 0xcd8d);
        WriteGMII16( 0x1d, 0x0056);
        WriteGMII16( 0x1d, 0xd96b);
        WriteGMII16( 0x1d, 0x0054);
        WriteGMII16( 0x1d, 0xd0a0);
        WriteGMII16( 0x1d, 0x00d8);
        WriteGMII16( 0x1d, 0xcba0);
        WriteGMII16( 0x1d, 0x0003);
        WriteGMII16( 0x1d, 0x80ec);
        WriteGMII16( 0x1d, 0x30a7);
        WriteGMII16( 0x1d, 0x30b4);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ce0);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c08);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6008);
        WriteGMII16( 0x1d, 0x8300);
        WriteGMII16( 0x1d, 0xb902);
        WriteGMII16( 0x1d, 0x307e);
        WriteGMII16( 0x1d, 0x3068);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4da0);
        WriteGMII16( 0x1d, 0x6628);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x56a0);
        WriteGMII16( 0x1d, 0x590c);
        WriteGMII16( 0x1d, 0x5fa0);
        WriteGMII16( 0x1d, 0xcba4);
        WriteGMII16( 0x1d, 0x0004);
        WriteGMII16( 0x1d, 0xcd8d);
        WriteGMII16( 0x1d, 0x0002);
        WriteGMII16( 0x1d, 0x80fc);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ca0);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x6408);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0xb603);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6010);
        WriteGMII16( 0x1d, 0x7d1f);
        WriteGMII16( 0x1d, 0x551f);
        WriteGMII16( 0x1d, 0x5fb3);
        WriteGMII16( 0x1d, 0xaa05);
        WriteGMII16( 0x1d, 0x7c80);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x5b58);
        WriteGMII16( 0x1d, 0x30d7);
        WriteGMII16( 0x1d, 0x7c80);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x5b64);
        WriteGMII16( 0x1d, 0x4827);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c10);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x7c10);
        WriteGMII16( 0x1d, 0x6000);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4cc0);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6400);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x5fbb);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c00);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c04);
        WriteGMII16( 0x1d, 0x8200);
        WriteGMII16( 0x1d, 0x7ce0);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7d00);
        WriteGMII16( 0x1d, 0x6500);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x30a7);
        WriteGMII16( 0x1d, 0x3001);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e00);
        WriteGMII16( 0x1d, 0x4007);
        WriteGMII16( 0x1d, 0x4400);
        WriteGMII16( 0x1d, 0x5310);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x673e);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x570f);
        WriteGMII16( 0x1d, 0x5fff);
        WriteGMII16( 0x1d, 0xaa05);
        WriteGMII16( 0x1d, 0x585b);
        WriteGMII16( 0x1d, 0x7d80);
        WriteGMII16( 0x1d, 0x6100);
        WriteGMII16( 0x1d, 0x3107);
        WriteGMII16( 0x1d, 0x5867);
        WriteGMII16( 0x1d, 0x7d80);
        WriteGMII16( 0x1d, 0x6080);
        WriteGMII16( 0x1d, 0x9403);
        WriteGMII16( 0x1d, 0x7e00);
        WriteGMII16( 0x1d, 0x6200);
        WriteGMII16( 0x1d, 0xcda3);
        WriteGMII16( 0x1d, 0x00e8);
        WriteGMII16( 0x1d, 0xcd85);
        WriteGMII16( 0x1d, 0x00e6);
        WriteGMII16( 0x1d, 0xd96b);
        WriteGMII16( 0x1d, 0x00e4);
        WriteGMII16( 0x1d, 0x96e4);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x673e);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e20);
        WriteGMII16( 0x1d, 0x96dd);
        WriteGMII16( 0x1d, 0x8b04);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x5008);
        WriteGMII16( 0x1d, 0xab03);
        WriteGMII16( 0x1d, 0x7c08);
        WriteGMII16( 0x1d, 0x5000);
        WriteGMII16( 0x1d, 0x6801);
        WriteGMII16( 0x1d, 0x677e);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0xdb7c);
        WriteGMII16( 0x1d, 0x00ee);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x7fe1);
        WriteGMII16( 0x1d, 0x4e40);
        WriteGMII16( 0x1d, 0x4837);
        WriteGMII16( 0x1d, 0x4418);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e40);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8fc2);
        WriteGMII16( 0x1d, 0xd2a0);
        WriteGMII16( 0x1d, 0x004b);
        WriteGMII16( 0x1d, 0x9204);
        WriteGMII16( 0x1d, 0xa042);
        WriteGMII16( 0x1d, 0x3132);
        WriteGMII16( 0x1d, 0x30f4);
        WriteGMII16( 0x1d, 0x7fe1);
        WriteGMII16( 0x1d, 0x4e60);
        WriteGMII16( 0x1d, 0x489c);
        WriteGMII16( 0x1d, 0x4628);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e60);
        WriteGMII16( 0x1d, 0x7e28);
        WriteGMII16( 0x1d, 0x4628);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5800);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c00);
        WriteGMII16( 0x1d, 0x41e8);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x8fa8);
        WriteGMII16( 0x1d, 0xb241);
        WriteGMII16( 0x1d, 0xa02a);
        WriteGMII16( 0x1d, 0x314c);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ea0);
        WriteGMII16( 0x1d, 0x7c02);
        WriteGMII16( 0x1d, 0x4402);
        WriteGMII16( 0x1d, 0x4448);
        WriteGMII16( 0x1d, 0x4894);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c03);
        WriteGMII16( 0x1d, 0x4824);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c07);
        WriteGMII16( 0x1d, 0x41ef);
        WriteGMII16( 0x1d, 0x41ff);
        WriteGMII16( 0x1d, 0x4891);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c07);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c17);
        WriteGMII16( 0x1d, 0x8400);
        WriteGMII16( 0x1d, 0x8ef8);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x8f8d);
        WriteGMII16( 0x1d, 0x92d5);
        WriteGMII16( 0x1d, 0xa10f);
        WriteGMII16( 0x1d, 0xd480);
        WriteGMII16( 0x1d, 0x0008);
        WriteGMII16( 0x1d, 0xd580);
        WriteGMII16( 0x1d, 0x00b8);
        WriteGMII16( 0x1d, 0xa202);
        WriteGMII16( 0x1d, 0x3167);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x4404);
        WriteGMII16( 0x1d, 0x3167);
        WriteGMII16( 0x1d, 0xd484);
        WriteGMII16( 0x1d, 0x00f3);
        WriteGMII16( 0x1d, 0xd484);
        WriteGMII16( 0x1d, 0x00f1);
        WriteGMII16( 0x1d, 0x30f4);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ee0);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5400);
        WriteGMII16( 0x1d, 0x4488);
        WriteGMII16( 0x1d, 0x41cf);
        WriteGMII16( 0x1d, 0x30f4);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4ec0);
        WriteGMII16( 0x1d, 0x48f3);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c09);
        WriteGMII16( 0x1d, 0x4508);
        WriteGMII16( 0x1d, 0x41c7);
        WriteGMII16( 0x1d, 0x8fb0);
        WriteGMII16( 0x1d, 0xd218);
        WriteGMII16( 0x1d, 0x00ae);
        WriteGMII16( 0x1d, 0xd2a4);
        WriteGMII16( 0x1d, 0x009e);
        WriteGMII16( 0x1d, 0x3188);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4e80);
        WriteGMII16( 0x1d, 0x4832);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c01);
        WriteGMII16( 0x1d, 0x7c1f);
        WriteGMII16( 0x1d, 0x4c11);
        WriteGMII16( 0x1d, 0x4428);
        WriteGMII16( 0x1d, 0x7c40);
        WriteGMII16( 0x1d, 0x5440);
        WriteGMII16( 0x1d, 0x7c01);
        WriteGMII16( 0x1d, 0x5801);
        WriteGMII16( 0x1d, 0x7c04);
        WriteGMII16( 0x1d, 0x5c04);
        WriteGMII16( 0x1d, 0x41e8);
        WriteGMII16( 0x1d, 0xa4b3);
        WriteGMII16( 0x1d, 0x319d);
        WriteGMII16( 0x1d, 0x7fe0);
        WriteGMII16( 0x1d, 0x4f20);
        WriteGMII16( 0x1d, 0x6800);
        WriteGMII16( 0x1d, 0x673e);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x0000);
        WriteGMII16( 0x1d, 0x570f);
        WriteGMII16( 0x1d, 0x5fff);
        WriteGMII16( 0x1d, 0xaa04);
        WriteGMII16( 0x1d, 0x585b);
        WriteGMII16( 0x1d, 0x6100);
        WriteGMII16( 0x1d, 0x31ad);
        WriteGMII16( 0x1d, 0x5867);
        WriteGMII16( 0x1d, 0x6080);
        WriteGMII16( 0x1d, 0xbcf2);
        WriteGMII16( 0x1d, 0x3001);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1c, 0x0200);
        WriteGMII16( 0x19, 0x7030);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == MCFG_8168E_VL_2) {
        WriteGMII16( 0x1f, 0x0B82);
        csi_tmp = ReadGMII16( 0x10);
        csi_tmp |= BIT_4;
        WriteGMII16( 0x10, csi_tmp);
        WriteGMII16( 0x1f, 0x0B80);
        i = 0;
        do {
            csi_tmp = ReadGMII16( 0x10);
            csi_tmp &= 0x0040;
            IODelay(50);
            IODelay(50);
            i++;
        } while(csi_tmp != 0x0040 && i <1000);
        WriteGMII16( 0x1f, 0x0A43);
        WriteGMII16( 0x13, 0x8146);
        WriteGMII16( 0x14, 0x0300);
        WriteGMII16( 0x13, 0xB82E);
        WriteGMII16( 0x14, 0x0001);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0xb820);
        WriteGMII16( 0x14, 0x0290);
        WriteGMII16( 0x13, 0xa012);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x13, 0xa014);
        WriteGMII16( 0x14, 0x2c04);
        WriteGMII16( 0x14, 0x2c07);
        WriteGMII16( 0x14, 0x2c07);
        WriteGMII16( 0x14, 0x2c07);
        WriteGMII16( 0x14, 0xa304);
        WriteGMII16( 0x14, 0xa301);
        WriteGMII16( 0x14, 0x207e);
        WriteGMII16( 0x13, 0xa01a);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x13, 0xa006);
        WriteGMII16( 0x14, 0x0fff);
        WriteGMII16( 0x13, 0xa004);
        WriteGMII16( 0x14, 0x0fff);
        WriteGMII16( 0x13, 0xa002);
        WriteGMII16( 0x14, 0x0fff);
        WriteGMII16( 0x13, 0xa000);
        WriteGMII16( 0x14, 0x107c);
        WriteGMII16( 0x13, 0xb820);
        WriteGMII16( 0x14, 0x0210);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x0000);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x1f, 0x0B82);
        csi_tmp = ReadGMII16( 0x17);
        csi_tmp &= ~(BIT_0);
        WriteGMII16( 0x17, csi_tmp);
        WriteGMII16( 0x1f, 0x0A43);
        WriteGMII16( 0x13, 0x8146);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x1f, 0x0B82);
        csi_tmp = ReadGMII16( 0x10);
        csi_tmp &= ~(BIT_4);
        WriteGMII16( 0x10, csi_tmp);
    }

    rtl8101_write_hw_phy_mcu_code_ver(dev);

    WriteGMII16( 0x1F, 0x0000);
}

static void
rtl8101_hw_phy_config(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;
    __u32	gphy_val;

    tp->phy_reset_enable(dev);

    spin_lock_irqsave(&tp->phy_lock, flags);

    rtl8101_init_hw_phy_mcu(dev);
    if (mcfg == CFG_METHOD_4) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | 0x1000);
        WriteGMII16( 0x19, ReadGMII16( 0x19) | 0x2000);
        WriteGMII16( 0x10, ReadGMII16( 0x10) | 0x8000);

        WriteGMII16( 0x1f, 0x0003);
        WriteGMII16( 0x08, 0x441D);
        WriteGMII16( 0x01, 0x9100);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == CFG_METHOD_5) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | 0x1000);
        WriteGMII16( 0x19, ReadGMII16( 0x19) | 0x2000);
        WriteGMII16( 0x10, ReadGMII16( 0x10) | 0x8000);

        WriteGMII16( 0x1f, 0x0003);
        WriteGMII16( 0x08, 0x441D);
        WriteGMII16( 0x01, 0x9100);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == CFG_METHOD_6) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | 0x1000);
        WriteGMII16( 0x19, ReadGMII16( 0x19) | 0x2000);
        WriteGMII16( 0x10, ReadGMII16( 0x10) | 0x8000);

        WriteGMII16( 0x1f, 0x0003);
        WriteGMII16( 0x08, 0x441D);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == CFG_METHOD_7 || mcfg == CFG_METHOD_8) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | 0x1000);
        WriteGMII16( 0x19, ReadGMII16( 0x19) | 0x2000);
        WriteGMII16( 0x10, ReadGMII16( 0x10) | 0x8000);
    } else if (mcfg == CFG_METHOD_9) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | BIT_12);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0F, ReadGMII16( 0x0F) | BIT_0 | BIT_1);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x0068);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x0069);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x006A);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x006B);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x006C);
    } else if (mcfg == MCFG_8105E) {
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0023);
        WriteGMII16( 0x17, 0x0116);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8b80);
        WriteGMII16( 0x06, 0xc896);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0x8C60);
        WriteGMII16( 0x07, 0x2872);
        WriteGMII16( 0x1C, 0xEFFF);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x14, 0x94B0);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x08) & 0x00FF;
        WriteGMII16( 0x08, gphy_val | 0x8000);

        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        WriteGMII16( 0x18, gphy_val | 0x0010);
        WriteGMII16( 0x1F, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        WriteGMII16( 0x14, gphy_val | 0x8000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x00, 0x080B);
        WriteGMII16( 0x0B, 0x09D7);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x15, 0x1006);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x19, 0x7F46);
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8AD2);
        WriteGMII16( 0x06, 0x6810);
        WriteGMII16( 0x05, 0x8AD4);
        WriteGMII16( 0x06, 0x8002);
        WriteGMII16( 0x05, 0x8ADE);
        WriteGMII16( 0x06, 0x8025);
        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
               mcfg == MCFG_8168DP_3) {
        if (ReadMMIO8(0xEF) & 0x08) {
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x1A, 0x0004);
            WriteGMII16( 0x1F, 0x0000);
        } else {
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x1A, 0x0000);
            WriteGMII16( 0x1F, 0x0000);
        }

        if (ReadMMIO8(0xEF) & 0x010) {
            WriteGMII16( 0x1F, 0x0004);
            WriteGMII16( 0x1C, 0x0000);
            WriteGMII16( 0x1F, 0x0000);
        } else {
            WriteGMII16( 0x1F, 0x0004);
            WriteGMII16( 0x1C, 0x0200);
            WriteGMII16( 0x1F, 0x0000);
        }

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x15, 0x7701);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        gphy_val = ReadGMII16( 0x1A);
        WriteGMII16( 0x08, gphy_val & ~BIT_14);

        if(aspm) {
            WriteGMII16( 0x1F, 0x0000);
            WriteGMII16( 0x18, 0x8310);
            WriteGMII16( 0x1F, 0x0000);
        }
    }
    else if (mcfg == MCFG_8402_1) {
        if(aspm) {
            WriteGMII16( 0x1F, 0x0000);
            WriteGMII16( 0x18, 0x8310);
            WriteGMII16( 0x1F, 0x0000);
        }
    } else if (mcfg == MCFG_8106E_1 || mcfg == MCFG_8106E_2) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x11, 0x83BA);
        WriteGMII16( 0x1F, 0x0000);

        if(aspm) {
            WriteGMII16( 0x1F, 0x0000);
            WriteGMII16( 0x18, 0x8310);
            WriteGMII16( 0x1F, 0x0000);
        }
    } else if (mcfg == MCFG_8106EUS) {
        WriteGMII16( 0x1F, 0x0BCC);
        WriteGMII16( 0x14, ReadGMII16( 0x14) & ~BIT_8);
        WriteGMII16( 0x1F, 0x0A44);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | BIT_7);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | BIT_6);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8084);
        WriteGMII16( 0x14, ReadGMII16( 0x14) & ~(BIT_14 | BIT_13));
        WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_12);
        WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_1);
        WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_0);

        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8012);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | BIT_15);

        WriteGMII16( 0x1F, 0x0BCE);
        WriteGMII16( 0x12, 0x8860);

        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x80F3);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x8B00);
        WriteGMII16( 0x13, 0x80F0);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x3A00);
        WriteGMII16( 0x13, 0x80EF);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x0500);
        WriteGMII16( 0x13, 0x80F6);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x6E00);
        WriteGMII16( 0x13, 0x80EC);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x6800);
        WriteGMII16( 0x13, 0x80ED);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x7C00);
        WriteGMII16( 0x13, 0x80F2);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0xF400);
        WriteGMII16( 0x13, 0x80F4);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x8500);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8110);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0xA800);
        WriteGMII16( 0x13, 0x810F);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x1D00);
        WriteGMII16( 0x13, 0x8111);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0xF500);
        WriteGMII16( 0x13, 0x8113);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x6100);
        WriteGMII16( 0x13, 0x8115);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x9200);
        WriteGMII16( 0x13, 0x810E);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x0400);
        WriteGMII16( 0x13, 0x810C);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x7C00);
        WriteGMII16( 0x13, 0x810B);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x5A00);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x80D1);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0xFF00);
        WriteGMII16( 0x13, 0x80CD);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x9E00);
        WriteGMII16( 0x13, 0x80D3);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x0E00);
        WriteGMII16( 0x13, 0x80D5);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0xCA00);
        WriteGMII16( 0x13, 0x80D7);
        WriteGMII16( 0x14, (ReadGMII16( 0x14) & ~0xFF00) | 0x8400);

        if (aspm) {
            WriteGMII16( 0x1F, 0x0A43);
            WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_2);
        }
    }

    /*ocp phy power saving*/
    if (mcfg == MCFG_8106EUS) {
        if (aspm) {
            WriteGMII16( 0x1F, 0x0C41);
            WriteGMII16( 0x13, 0x0000);
            WriteGMII16( 0x13, 0x0050);
            WriteGMII16( 0x1F, 0x0000);
        }
    }

    WriteGMII16( 0x1F, 0x0000);

    spin_unlock_irqrestore(&tp->phy_lock, flags);

    if (eee_enable == 1)
        rtl8101_enable_EEE(tp);
    else
        rtl8101_disable_EEE(tp);
}

static inline void rtl8101_delete_link_timer(struct net_device *dev, struct timer_list *timer)
{
    del_timer_sync(timer);
}

static inline void rtl8101_request_link_timer(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->link_timer;

    init_timer(timer);
    timer->expires = jiffies + RTL8101_LINK_TIMEOUT;
    timer->data = (unsigned long)(dev);
    timer->function = rtl8101_link_timer;
    add_timer(timer);
}

static inline void rtl8101_delete_esd_timer(struct net_device *dev, struct timer_list *timer)
{
    del_timer_sync(timer);
}

static inline void rtl8101_request_esd_timer(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->esd_timer;

    init_timer(timer);
    timer->expires = jiffies + RTL8101_ESD_TIMEOUT;
    timer->data = (unsigned long)(dev);
    timer->function = rtl8101_esd_timer;
    add_timer(timer);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void
rtl8101_netpoll(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;

    disable_irq(pdev->irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
    rtl8101_interrupt(pdev->irq, dev, NULL);
#else
    rtl8101_interrupt(pdev->irq, dev);
#endif
    enable_irq(pdev->irq);
}
#endif

static void
rtl8101_get_bios_setting(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    bios_setting = 0;
    switch (mcfg) {
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        bios_setting = ReadMMIO32(0x8c);
        break;
    }
}

static void
rtl8101_set_bios_setting(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        WriteMMIO32(0x8C, bios_setting);
        break;
    }
}

static void
rtl8101_release_board(struct pci_dev *pdev,
                      struct net_device *dev,
                      void __iomem *ioaddr)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    rtl8101_set_bios_setting(dev);
    rtl8101_rar_set(tp, tp->org_mac_addr);
    tp->wol_enabled = WOL_DISABLED;

    rtl8101_phy_power_down(dev);

    iounmap(ioaddr);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    free_netdev(dev);
}

static int
rtl8101_get_mac_address(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 mac_addr[3];
    int i;


    if (mcfg == MCFG_8402_1 ||
        mcfg == MCFG_8168E_VL_2) {
        *(u32*)&mac_addr[0] = ReadERI(0xE0, 4, ERIAR_ExGMAC);
        *(u16*)&mac_addr[2] = ReadERI(0xE4, 2, ERIAR_ExGMAC);
    } else {
        if (tp->eeprom_type != EEPROM_TYPE_NONE) {
            /* Get MAC address from EEPROM */
            mac_addr[0] = rtl_eeprom_read_sc(tp, 7);
            mac_addr[1] = rtl_eeprom_read_sc(tp, 8);
            mac_addr[2] = rtl_eeprom_read_sc(tp, 9);
            WriteMMIO8(Cfg9346, Cfg9346_Unlock);
            WriteMMIO32(MAC0, (mac_addr[1] << 16) | mac_addr[0]);
            WriteMMIO16(MAC4, mac_addr[2]);
            WriteMMIO8(Cfg9346, Cfg9346_Lock);
        }
    }

    for (i = 0; i < MAC_ADDR_LEN; i++) {
        dev->dev_addr[i] = ReadMMIO8(MAC0 + i);
        tp->org_mac_addr[i] = dev->dev_addr[i]; /* keep the original MAC address */
    }
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
    memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);
#endif
//  memcpy(dev->dev_addr, dev->dev_addr, dev->addr_len);

    return 0;
}

/**
 * rtl8101_set_mac_address - Change the Ethernet Address of the NIC
 * @dev: network interface device structure
 * @p:   pointer to an address structure
 *
 * Return 0 on success, negative on failure
 **/
static int
rtl8101_set_mac_address(struct net_device *dev,
                        void *p)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

    rtl8101_rar_set(tp, dev->dev_addr);

    return 0;
}

/******************************************************************************
 * rtl8101_rar_set - Puts an ethernet address into a receive address register.
 *
 * tp - The private data structure for driver
 * addr - Address to put into receive address register
 *****************************************************************************/
void
rtl8101_rar_set(struct rtl8101_private *tp,
                uint8_t *addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    uint32_t rar_low = 0;
    uint32_t rar_high = 0;

    rar_low = ((uint32_t) addr[0] |
               ((uint32_t) addr[1] << 8) |
               ((uint32_t) addr[2] << 16) |
               ((uint32_t) addr[3] << 24));

    rar_high = ((uint32_t) addr[4] |
                ((uint32_t) addr[5] << 8));

    WriteMMIO8(Cfg9346, Cfg9346_Unlock);
    WriteMMIO32(MAC0, rar_low);
    WriteMMIO32(MAC4, rar_high);
    WriteMMIO8(Cfg9346, Cfg9346_Lock);
}

#ifdef ETHTOOL_OPS_COMPAT
static int ethtool_get_settings(struct net_device *dev, void *useraddr)
{
    struct ethtool_cmd cmd = { ETHTOOL_GSET };
    int err;

    if (!ethtool_ops->get_settings)
        return -EOPNOTSUPP;

    err = ethtool_ops->get_settings(dev, &cmd);
    if (err < 0)
        return err;

    if (copy_to_user(useraddr, &cmd, sizeof(cmd)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_settings(struct net_device *dev, void *useraddr)
{
    struct ethtool_cmd cmd;

    if (!ethtool_ops->set_settings)
        return -EOPNOTSUPP;

    if (copy_from_user(&cmd, useraddr, sizeof(cmd)))
        return -EFAULT;

    return ethtool_ops->set_settings(dev, &cmd);
}

static int ethtool_get_drvinfo(struct net_device *dev, void *useraddr)
{
    struct ethtool_drvinfo info;
    struct ethtool_ops *ops = ethtool_ops;

    if (!ops->get_drvinfo)
        return -EOPNOTSUPP;

    memset(&info, 0, sizeof(info));
    info.cmd = ETHTOOL_GDRVINFO;
    ops->get_drvinfo(dev, &info);

    if (ops->self_test_count)
        info.testinfo_len = ops->self_test_count(dev);
    if (ops->get_stats_count)
        info.n_stats = ops->get_stats_count(dev);
    if (ops->get_regs_len)
        info.regdump_len = ops->get_regs_len(dev);
    if (ops->get_eeprom_len)
        info.eedump_len = ops->get_eeprom_len(dev);

    if (copy_to_user(useraddr, &info, sizeof(info)))
        return -EFAULT;
    return 0;
}

static int ethtool_get_regs(struct net_device *dev, char *useraddr)
{
    struct ethtool_regs regs;
    struct ethtool_ops *ops = ethtool_ops;
    void *regbuf;
    int reglen, ret;

    if (!ops->get_regs || !ops->get_regs_len)
        return -EOPNOTSUPP;

    if (copy_from_user(&regs, useraddr, sizeof(regs)))
        return -EFAULT;

    reglen = ops->get_regs_len(dev);
    if (regs.len > reglen)
        regs.len = reglen;

    regbuf = kmalloc(reglen, GFP_USER);
    if (!regbuf)
        return -ENOMEM;

    ops->get_regs(dev, &regs, regbuf);

    ret = -EFAULT;
    if (copy_to_user(useraddr, &regs, sizeof(regs)))
        goto out;
    useraddr += offsetof(struct ethtool_regs, data);
    if (copy_to_user(useraddr, regbuf, reglen))
        goto out;
    ret = 0;

out:
    kfree(regbuf);
    return ret;
}

static int ethtool_get_wol(struct net_device *dev, char *useraddr)
{
    struct ethtool_wolinfo wol = { ETHTOOL_GWOL };

    if (!ethtool_ops->get_wol)
        return -EOPNOTSUPP;

    ethtool_ops->get_wol(dev, &wol);

    if (copy_to_user(useraddr, &wol, sizeof(wol)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_wol(struct net_device *dev, char *useraddr)
{
    struct ethtool_wolinfo wol;

    if (!ethtool_ops->set_wol)
        return -EOPNOTSUPP;

    if (copy_from_user(&wol, useraddr, sizeof(wol)))
        return -EFAULT;

    return ethtool_ops->set_wol(dev, &wol);
}

static int ethtool_get_msglevel(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GMSGLVL };

    if (!ethtool_ops->get_msglevel)
        return -EOPNOTSUPP;

    edata.data = ethtool_ops->get_msglevel(dev);

    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_msglevel(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;

    if (!ethtool_ops->set_msglevel)
        return -EOPNOTSUPP;

    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;

    ethtool_ops->set_msglevel(dev, edata.data);
    return 0;
}

static int ethtool_nway_reset(struct net_device *dev)
{
    if (!ethtool_ops->nway_reset)
        return -EOPNOTSUPP;

    return ethtool_ops->nway_reset(dev);
}

static int ethtool_get_link(struct net_device *dev, void *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GLINK };

    if (!ethtool_ops->get_link)
        return -EOPNOTSUPP;

    edata.data = ethtool_ops->get_link(dev);

    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_get_eeprom(struct net_device *dev, void *useraddr)
{
    struct ethtool_eeprom eeprom;
    struct ethtool_ops *ops = ethtool_ops;
    u8 *data;
    int ret;

    if (!ops->get_eeprom || !ops->get_eeprom_len)
        return -EOPNOTSUPP;

    if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
        return -EFAULT;

    /* Check for wrap and zero */
    if (eeprom.offset + eeprom.len <= eeprom.offset)
        return -EINVAL;

    /* Check for exceeding total eeprom len */
    if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
        return -EINVAL;

    data = kmalloc(eeprom.len, GFP_USER);
    if (!data)
        return -ENOMEM;

    ret = -EFAULT;
    if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
        goto out;

    ret = ops->get_eeprom(dev, &eeprom, data);
    if (ret)
        goto out;

    ret = -EFAULT;
    if (copy_to_user(useraddr, &eeprom, sizeof(eeprom)))
        goto out;
    if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
        goto out;
    ret = 0;

out:
    kfree(data);
    return ret;
}

static int ethtool_set_eeprom(struct net_device *dev, void *useraddr)
{
    struct ethtool_eeprom eeprom;
    struct ethtool_ops *ops = ethtool_ops;
    u8 *data;
    int ret;

    if (!ops->set_eeprom || !ops->get_eeprom_len)
        return -EOPNOTSUPP;

    if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
        return -EFAULT;

    /* Check for wrap and zero */
    if (eeprom.offset + eeprom.len <= eeprom.offset)
        return -EINVAL;

    /* Check for exceeding total eeprom len */
    if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
        return -EINVAL;

    data = kmalloc(eeprom.len, GFP_USER);
    if (!data)
        return -ENOMEM;

    ret = -EFAULT;
    if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
        goto out;

    ret = ops->set_eeprom(dev, &eeprom, data);
    if (ret)
        goto out;

    if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
        ret = -EFAULT;

out:
    kfree(data);
    return ret;
}

static int ethtool_get_coalesce(struct net_device *dev, void *useraddr)
{
    struct ethtool_coalesce coalesce = { ETHTOOL_GCOALESCE };

    if (!ethtool_ops->get_coalesce)
        return -EOPNOTSUPP;

    ethtool_ops->get_coalesce(dev, &coalesce);

    if (copy_to_user(useraddr, &coalesce, sizeof(coalesce)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_coalesce(struct net_device *dev, void *useraddr)
{
    struct ethtool_coalesce coalesce;

    if (!ethtool_ops->get_coalesce)
        return -EOPNOTSUPP;

    if (copy_from_user(&coalesce, useraddr, sizeof(coalesce)))
        return -EFAULT;

    return ethtool_ops->set_coalesce(dev, &coalesce);
}

static int ethtool_get_ringparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_ringparam ringparam = { ETHTOOL_GRINGPARAM };

    if (!ethtool_ops->get_ringparam)
        return -EOPNOTSUPP;

    ethtool_ops->get_ringparam(dev, &ringparam);

    if (copy_to_user(useraddr, &ringparam, sizeof(ringparam)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_ringparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_ringparam ringparam;

    if (!ethtool_ops->get_ringparam)
        return -EOPNOTSUPP;

    if (copy_from_user(&ringparam, useraddr, sizeof(ringparam)))
        return -EFAULT;

    return ethtool_ops->set_ringparam(dev, &ringparam);
}

static int ethtool_get_pauseparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_pauseparam pauseparam = { ETHTOOL_GPAUSEPARAM };

    if (!ethtool_ops->get_pauseparam)
        return -EOPNOTSUPP;

    ethtool_ops->get_pauseparam(dev, &pauseparam);

    if (copy_to_user(useraddr, &pauseparam, sizeof(pauseparam)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_pauseparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_pauseparam pauseparam;

    if (!ethtool_ops->get_pauseparam)
        return -EOPNOTSUPP;

    if (copy_from_user(&pauseparam, useraddr, sizeof(pauseparam)))
        return -EFAULT;

    return ethtool_ops->set_pauseparam(dev, &pauseparam);
}

static int ethtool_get_rx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GRXCSUM };

    if (!ethtool_ops->get_rx_csum)
        return -EOPNOTSUPP;

    edata.data = ethtool_ops->get_rx_csum(dev);

    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_rx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;

    if (!ethtool_ops->set_rx_csum)
        return -EOPNOTSUPP;

    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;

    ethtool_ops->set_rx_csum(dev, edata.data);
    return 0;
}

static int ethtool_get_tx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GTXCSUM };

    if (!ethtool_ops->get_tx_csum)
        return -EOPNOTSUPP;

    edata.data = ethtool_ops->get_tx_csum(dev);

    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_tx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;

    if (!ethtool_ops->set_tx_csum)
        return -EOPNOTSUPP;

    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;

    return ethtool_ops->set_tx_csum(dev, edata.data);
}

static int ethtool_get_sg(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GSG };

    if (!ethtool_ops->get_sg)
        return -EOPNOTSUPP;

    edata.data = ethtool_ops->get_sg(dev);

    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_sg(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;

    if (!ethtool_ops->set_sg)
        return -EOPNOTSUPP;

    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;

    return ethtool_ops->set_sg(dev, edata.data);
}

static int ethtool_get_tso(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GTSO };

    if (!ethtool_ops->get_tso)
        return -EOPNOTSUPP;

    edata.data = ethtool_ops->get_tso(dev);

    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_tso(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;

    if (!ethtool_ops->set_tso)
        return -EOPNOTSUPP;

    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;

    return ethtool_ops->set_tso(dev, edata.data);
}

static int ethtool_self_test(struct net_device *dev, char *useraddr)
{
    struct ethtool_test test;
    struct ethtool_ops *ops = ethtool_ops;
    u64 *data;
    int ret;

    if (!ops->self_test || !ops->self_test_count)
        return -EOPNOTSUPP;

    if (copy_from_user(&test, useraddr, sizeof(test)))
        return -EFAULT;

    test.len = ops->self_test_count(dev);
    data = kmalloc(test.len * sizeof(u64), GFP_USER);
    if (!data)
        return -ENOMEM;

    ops->self_test(dev, &test, data);

    ret = -EFAULT;
    if (copy_to_user(useraddr, &test, sizeof(test)))
        goto out;
    useraddr += sizeof(test);
    if (copy_to_user(useraddr, data, test.len * sizeof(u64)))
        goto out;
    ret = 0;

out:
    kfree(data);
    return ret;
}

static int ethtool_get_strings(struct net_device *dev, void *useraddr)
{
    struct ethtool_gstrings gstrings;
    struct ethtool_ops *ops = ethtool_ops;
    u8 *data;
    int ret;

    if (!ops->get_strings)
        return -EOPNOTSUPP;

    if (copy_from_user(&gstrings, useraddr, sizeof(gstrings)))
        return -EFAULT;

    switch (gstrings.string_set) {
    case ETH_SS_TEST:
        if (!ops->self_test_count)
            return -EOPNOTSUPP;
        gstrings.len = ops->self_test_count(dev);
        break;
    case ETH_SS_STATS:
        if (!ops->get_stats_count)
            return -EOPNOTSUPP;
        gstrings.len = ops->get_stats_count(dev);
        break;
    default:
        return -EINVAL;
    }

    data = kmalloc(gstrings.len * ETH_GSTRING_LEN, GFP_USER);
    if (!data)
        return -ENOMEM;

    ops->get_strings(dev, gstrings.string_set, data);

    ret = -EFAULT;
    if (copy_to_user(useraddr, &gstrings, sizeof(gstrings)))
        goto out;
    useraddr += sizeof(gstrings);
    if (copy_to_user(useraddr, data, gstrings.len * ETH_GSTRING_LEN))
        goto out;
    ret = 0;

out:
    kfree(data);
    return ret;
}

static int ethtool_phys_id(struct net_device *dev, void *useraddr)
{
    struct ethtool_value id;

    if (!ethtool_ops->phys_id)
        return -EOPNOTSUPP;

    if (copy_from_user(&id, useraddr, sizeof(id)))
        return -EFAULT;

    return ethtool_ops->phys_id(dev, id.data);
}

static int ethtool_get_stats(struct net_device *dev, void *useraddr)
{
    struct ethtool_stats stats;
    struct ethtool_ops *ops = ethtool_ops;
    u64 *data;
    int ret;

    if (!ops->get_ethtool_stats || !ops->get_stats_count)
        return -EOPNOTSUPP;

    if (copy_from_user(&stats, useraddr, sizeof(stats)))
        return -EFAULT;

    stats.n_stats = ops->get_stats_count(dev);
    data = kmalloc(stats.n_stats * sizeof(u64), GFP_USER);
    if (!data)
        return -ENOMEM;

    ops->get_ethtool_stats(dev, &stats, data);

    ret = -EFAULT;
    if (copy_to_user(useraddr, &stats, sizeof(stats)))
        goto out;
    useraddr += sizeof(stats);
    if (copy_to_user(useraddr, data, stats.n_stats * sizeof(u64)))
        goto out;
    ret = 0;

out:
    kfree(data);
    return ret;
}

static int ethtool_ioctl(struct ifreq *ifr)
{
    struct net_device *dev = __dev_get_by_name(ifr->ifr_name);
    void *useraddr = (void *) ifr->ifr_data;
    u32 ethcmd;

    /*
     * XXX: This can be pushed down into the ethtool_* handlers that
     * need it.  Keep existing behaviour for the moment.
     */
    if (!capable(CAP_NET_ADMIN))
        return -EPERM;

    if (!dev || !netif_device_present(dev))
        return -ENODEV;

    if (copy_from_user(&ethcmd, useraddr, sizeof (ethcmd)))
        return -EFAULT;

    switch (ethcmd) {
    case ETHTOOL_GSET:
        return ethtool_get_settings(dev, useraddr);
    case ETHTOOL_SSET:
        return ethtool_set_settings(dev, useraddr);
    case ETHTOOL_GDRVINFO:
        return ethtool_get_drvinfo(dev, useraddr);
    case ETHTOOL_GREGS:
        return ethtool_get_regs(dev, useraddr);
    case ETHTOOL_GWOL:
        return ethtool_get_wol(dev, useraddr);
    case ETHTOOL_SWOL:
        return ethtool_set_wol(dev, useraddr);
    case ETHTOOL_GMSGLVL:
        return ethtool_get_msglevel(dev, useraddr);
    case ETHTOOL_SMSGLVL:
        return ethtool_set_msglevel(dev, useraddr);
    case ETHTOOL_NWAY_RST:
        return ethtool_nway_reset(dev);
    case ETHTOOL_GLINK:
        return ethtool_get_link(dev, useraddr);
    case ETHTOOL_GEEPROM:
        return ethtool_get_eeprom(dev, useraddr);
    case ETHTOOL_SEEPROM:
        return ethtool_set_eeprom(dev, useraddr);
    case ETHTOOL_GCOALESCE:
        return ethtool_get_coalesce(dev, useraddr);
    case ETHTOOL_SCOALESCE:
        return ethtool_set_coalesce(dev, useraddr);
    case ETHTOOL_GRINGPARAM:
        return ethtool_get_ringparam(dev, useraddr);
    case ETHTOOL_SRINGPARAM:
        return ethtool_set_ringparam(dev, useraddr);
    case ETHTOOL_GPAUSEPARAM:
        return ethtool_get_pauseparam(dev, useraddr);
    case ETHTOOL_SPAUSEPARAM:
        return ethtool_set_pauseparam(dev, useraddr);
    case ETHTOOL_GRXCSUM:
        return ethtool_get_rx_csum(dev, useraddr);
    case ETHTOOL_SRXCSUM:
        return ethtool_set_rx_csum(dev, useraddr);
    case ETHTOOL_GTXCSUM:
        return ethtool_get_tx_csum(dev, useraddr);
    case ETHTOOL_STXCSUM:
        return ethtool_set_tx_csum(dev, useraddr);
    case ETHTOOL_GSG:
        return ethtool_get_sg(dev, useraddr);
    case ETHTOOL_SSG:
        return ethtool_set_sg(dev, useraddr);
    case ETHTOOL_GTSO:
        return ethtool_get_tso(dev, useraddr);
    case ETHTOOL_STSO:
        return ethtool_set_tso(dev, useraddr);
    case ETHTOOL_TEST:
        return ethtool_self_test(dev, useraddr);
    case ETHTOOL_GSTRINGS:
        return ethtool_get_strings(dev, useraddr);
    case ETHTOOL_PHYS_ID:
        return ethtool_phys_id(dev, useraddr);
    case ETHTOOL_GSTATS:
        return ethtool_get_stats(dev, useraddr);
    default:
        return -EOPNOTSUPP;
    }

    return -EOPNOTSUPP;
}
#endif //ETHTOOL_OPS_COMPAT


static int
rtl8101_do_ioctl(struct net_device *dev,
                 struct ifreq *ifr,
                 int cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct mii_ioctl_data *data = if_mii(ifr);
    unsigned long flags;

    if (!netif_running(dev))
        return -ENODEV;

    switch (cmd) {
    case SIOCGMIIPHY:
        data->phy_id = 32; /* Internal PHY */
        return 0;

    case SIOCGMIIREG:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0000);
        data->val_out = ReadGMII16( data->reg_num & 0x1f);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        return 0;

    case SIOCSMIIREG:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( data->reg_num & 0x1f, data->val_in);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        return 0;
#ifdef ETHTOOL_OPS_COMPAT
    case SIOCETHTOOL:
        return ethtool_ioctl(ifr);
#endif

    case SIOCRTLTOOL:
        return rtltool_ioctl(tp, ifr);

    default:
        return -EOPNOTSUPP;
    }

    return -EOPNOTSUPP;
}

static void
rtl8101_phy_power_down (struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1f, 0x0000);
    if (mcfg == MCFG_8105E)
        WriteGMII16( MII_BMCR, BMCR_ANENABLE|BMCR_PDOWN);
    else
        WriteGMII16( MII_BMCR, BMCR_PDOWN);

    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
rtl8101_phy_power_up (struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1f, 0x0000);
    WriteGMII16( MII_BMCR, BMCR_ANENABLE);
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static int __devinit
rtl8101_init_board(struct pci_dev *pdev,
                   struct net_device **dev_out,
                   void __iomem **ioaddr_out)
{
    void __iomem *ioaddr;
    struct net_device *dev;
    struct rtl8101_private *tp;
    int rc = -ENOMEM, i, acpi_idle_state = 0, pm_cap;

    assert(ioaddr_out != NULL);

    /* dev zeroed in alloc_etherdev */
    dev = alloc_etherdev(sizeof (*tp));
    if (dev == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_drv(&debug))
            dev_err(&pdev->dev, "unable to alloc new ethernet\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        goto err_out;
    }

    SET_MODULE_OWNER(dev);
    SET_NETDEV_DEV(dev, &pdev->dev);
    tp = netdev_priv(dev);
    tp->dev = dev;
    tp->pci_dev = pdev;
    tp->msg_enable = netif_msg_init(debug.msg_enable, R8101_MSG_DEFAULT);

    /* enable device (incl. PCI PM wakeup and hotplug setup) */
    rc = pci_enable_device(pdev);
    if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "enable failure\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        goto err_out_free_dev;
    }

    rc = pci_set_mwi(pdev);
    if (rc < 0)
        goto err_out_disable;

    /* save power state before pci_enable_device overwrites it */
    pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
    if (pm_cap) {
        u16 pwr_command;

        pci_read_config_word(pdev, pm_cap + PCI_PM_CTRL, &pwr_command);
        acpi_idle_state = pwr_command & PCI_PM_CTRL_STATE_MASK;
    } else {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp)) {
            dev_err(&pdev->dev, "PowerManagement capability not found.\n");
        }
#else
        printk("PowerManagement capability not found.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    }

    /* make sure PCI base addr 1 is MMIO */
    if (!(pci_resource_flags(pdev, 2) & IORESOURCE_MEM)) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev,
                    "region #1 not an MMIO resource, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -ENODEV;
        goto err_out_mwi;
    }
    /* check for weird/broken PCI region reporting */
    if (pci_resource_len(pdev, 2) < R8101_REGS_SIZE) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev,
                    "Invalid PCI region size(s), aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -ENODEV;
        goto err_out_mwi;
    }

    rc = pci_request_regions(pdev, MODULENAME);
    if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "could not request regions.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        goto err_out_mwi;
    }

    if ((sizeof(dma_addr_t) > 4) &&
        !pci_set_dma_mask(pdev, DMA_BIT_MASK(64)) && use_dac) {
        cp_cmd |= PCIDAC;
        dev->features |= NETIF_F_HIGHDMA;
    } else {
        rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
        if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
            if (netif_msg_probe(tp))
                dev_err(&pdev->dev, "DMA configuration failed.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
            goto err_out_free_res;
        }
    }

    pci_set_master(pdev);

    /* ioremap MMIO region */
    ioaddr = ioremap(pci_resource_start(pdev, 2), R8101_REGS_SIZE);
    if (ioaddr == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "cannot remap MMIO, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -EIO;
        goto err_out_free_res;
    }

    pci_write_config_dword(pdev, 0x30, 0);

    /* Unneeded ? Don't mess with Mrs. Murphy. */
    rtl8101_irq_mask_and_ack(ioaddr);

    /* Soft reset the chip. */
    WriteMMIO8(ChipCmd, CmdReset);

    /* Check that the chip has finished the reset. */
    for (i = 1000; i > 0; i--) {
        if ((ReadMMIO8(ChipCmd) & CmdReset) == 0)
            break;
        IODelay(10);
    }

    /* Identify chip attached to board */
    rtl8101_get_mac_version(tp, ioaddr);

    rtl8101_print_mac_version(tp);

    for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
        if (mcfg == rtl_chip_info[i].mcfg)
            break;
    }

    if (i < 0) {
        /* Unknown chip: assume array element #0, original RTL-8101 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp)) {
            dev_printk(KERN_DEBUG, &pdev->dev, "unknown chip version, assuming %s\n", rtl_chip_info[0].name);
        }
#else
        printk("Realtek unknown chip version, assuming %s\n", rtl_chip_info[0].name);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        i++;
    }

    tp->chipset = i;

    *ioaddr_out = ioaddr;
    *dev_out = dev;
out:
    return rc;

err_out_free_res:
    pci_release_regions(pdev);

err_out_mwi:
    pci_clear_mwi(pdev);

err_out_disable:
    pci_disable_device(pdev);

err_out_free_dev:
    free_netdev(dev);
err_out:
    *ioaddr_out = NULL;
    *dev_out = NULL;
    goto out;
}

static void
rtl8101_esd_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    struct timer_list *timer = &tp->esd_timer;
    unsigned long timeout = RTL8101_ESD_TIMEOUT;
    unsigned long flags;
    u8 cmd;
    u8 cls;
    u16 io_base_l;
    u16 io_base_h;
    u16 mem_base_l;
    u16 mem_base_h;
    u8 ilr;
    u16 resv_0x20_l;
    u16 resv_0x20_h;
    u16 resv_0x24_l;
    u16 resv_0x24_h;

    spin_lock_irqsave(&tp->lock, flags);

    tp->esd_flag = 0;

    pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
    if (cmd != tp->pci_cfg_space.cmd) {
        pci_write_config_byte(pdev, PCI_COMMAND, tp->pci_cfg_space.cmd);
        tp->esd_flag |= 0x0001;
    }

    pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cls);
    if (cls != tp->pci_cfg_space.cls) {
        pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, tp->pci_cfg_space.cls);
        tp->esd_flag |= 0x0002;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &io_base_l);
    if (io_base_l != tp->pci_cfg_space.io_base_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_0, tp->pci_cfg_space.io_base_l);
        tp->esd_flag |= 0x0004;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &io_base_h);
    if (io_base_h != tp->pci_cfg_space.io_base_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, tp->pci_cfg_space.io_base_h);

        /* Put carrier down when fnf2 disable network */
        netif_carrier_off(dev);

        tp->esd_flag |= 0x0008;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &mem_base_l);
    if (mem_base_l != tp->pci_cfg_space.mem_base_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_2, tp->pci_cfg_space.mem_base_l);
        tp->esd_flag |= 0x0010;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &mem_base_h);
    if (mem_base_h != tp->pci_cfg_space.mem_base_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, tp->pci_cfg_space.mem_base_h);
        tp->esd_flag |= 0x0020;
    }

    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &ilr);
    if (ilr != tp->pci_cfg_space.ilr) {
        pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, tp->pci_cfg_space.ilr);
        tp->esd_flag |= 0x0040;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &resv_0x20_l);
    if (resv_0x20_l != tp->pci_cfg_space.resv_0x20_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_4, tp->pci_cfg_space.resv_0x20_l);
        tp->esd_flag |= 0x0080;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &resv_0x20_h);
    if (resv_0x20_h != tp->pci_cfg_space.resv_0x20_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, tp->pci_cfg_space.resv_0x20_h);
        tp->esd_flag |= 0x0100;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &resv_0x24_l);
    if (resv_0x24_l != tp->pci_cfg_space.resv_0x24_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_5, tp->pci_cfg_space.resv_0x24_l);
        tp->esd_flag |= 0x0200;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &resv_0x24_h);
    if (resv_0x24_h != tp->pci_cfg_space.resv_0x24_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, tp->pci_cfg_space.resv_0x24_h);
        tp->esd_flag |= 0x0400;
    }

    if (tp->esd_flag != 0) {
        netif_stop_queue(dev);
        netif_carrier_off(dev);
        rtl8101_hw_reset(dev);
        rtl8101_tx_clear(tp);
        rtl8101_init_ring_indexes(tp);
        rtl8101_hw_init(dev);
        rtl8101_powerup_pll(dev);
        rtl8101_hw_ephy_config(dev);
        rtl8101_hw_phy_config(dev);
        rtl8101_hw_start(dev);
        rtl8101_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
        tp->esd_flag = 0;
    }
    spin_unlock_irqrestore(&tp->lock, flags);

    mod_timer(timer, jiffies + timeout);
}

static void
rtl8101_link_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8101_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->link_timer;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    rtl8101_check_link_status(dev);
    spin_unlock_irqrestore(&tp->lock, flags);

    mod_timer(timer, jiffies + RTL8101_LINK_TIMEOUT);
}

static unsigned rtl8101_try_msi(struct pci_dev *pdev, void __iomem *ioaddr)
{
    unsigned msi = 0;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
    if (pci_enable_msi(pdev))
        dev_info(&pdev->dev, "no MSI. Back to INTx.\n");
    else
        msi |= RTL_FEATURE_MSI;
#endif

    return msi;
}

static void rtl8101_disable_msi(struct pci_dev *pdev, struct rtl8101_private *tp)
{
    if (tp->features & RTL_FEATURE_MSI) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
        pci_disable_msi(pdev);
#endif
        tp->features &= ~RTL_FEATURE_MSI;
    }
}

static void
rtl8101_aspm_fix1(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int data;

    data = rtl8101_csi_read(tp, 0x110);

    if ((data & (1 << 7)) && (data & (1 << 8))) {
        rtl8101_ephy_write(ioaddr, 0x01, 0x2e65);
        rtl8101_ephy_write(ioaddr, 0x01, 0x6e65);
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops rtl8101_netdev_ops = {
    .ndo_open		= rtl8101_open,
    .ndo_stop		= rtl8101_close,
    .ndo_get_stats		= rtl8101_get_stats,
    .ndo_start_xmit		= rtl8101_start_xmit,
    .ndo_tx_timeout		= rtl8101_tx_timeout,
    .ndo_change_mtu		= rtl8101_change_mtu,
    .ndo_set_mac_address	= rtl8101_set_mac_address,
    .ndo_do_ioctl		= rtl8101_do_ioctl,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
    .ndo_set_multicast_list	= rtl8101_set_rx_mode,
#else
    .ndo_set_rx_mode	= rtl8101_set_rx_mode,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#ifdef CONFIG_R8101_VLAN
    dev->vlan_rx_register	= rtl8101_vlan_rx_register,
#endif
#else
    .ndo_fix_features	= rtl8101_fix_features,
    .ndo_set_features	= rtl8101_set_features,
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
    .ndo_poll_controller	= rtl8101_netpoll,
#endif
};
#endif

static int __devinit
rtl8101_init_one(struct pci_dev *pdev,
                 const struct pci_device_id *ent)
{
    struct net_device *dev = NULL;
    struct rtl8101_private *tp;
    void __iomem *ioaddr = NULL;
    static int board_idx = -1;
    u8 autoneg, duplex;
    u16 speed;
    int rc;

    assert(pdev != NULL);
    assert(ent != NULL);

    board_idx++;

    if (netif_msg_drv(&debug)) {
        printk(KERN_INFO "%s Fast Ethernet driver %s loaded\n",
               MODULENAME, RTL8101_VERSION);
    }

    rc = rtl8101_init_board(pdev, &dev, &ioaddr);
    if (rc)
        return rc;

    tp = netdev_priv(dev);
    assert(ioaddr != NULL);

    tp->mmio_addr = ioaddr;
    tp->set_speed = rtl8101_set_speed_xmii;
    tp->get_settings = rtl8101_gset_xmii;
    tp->phy_reset_enable = rtl8101_xmii_reset_enable;
    tp->phy_reset_pending = rtl8101_xmii_reset_pending;
    tp->link_ok = rtl8101_xmii_link_ok;

    tp->features |= rtl8101_try_msi(pdev, ioaddr);

    RTL_NET_DEVICE_OPS(rtl8101_netdev_ops);

    SET_ETHTOOL_OPS(dev, &rtl8101_ethtool_ops);

    dev->watchdog_timeo = RTL8101_TX_TIMEOUT;
    dev->irq = pdev->irq;
    dev->base_addr = (unsigned long) ioaddr;

#ifdef CONFIG_R8101_NAPI
    RTL_NAPI_CONFIG(dev, tp, rtl8101_poll, R8101_NAPI_WEIGHT);
#endif

#ifdef CONFIG_R8101_VLAN
    dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    dev->vlan_rx_kill_vid = rtl8101_vlan_rx_kill_vid;
#endif//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#endif

    dev->features |= NETIF_F_IP_CSUM;
    cp_cmd |= ReadMMIO16(CPlusCmd);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    cp_cmd |= RxChkSum;
#else
    dev->features |= NETIF_F_RXCSUM | NETIF_F_SG;
    dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                       NETIF_F_RXCSUM | NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
    dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                         NETIF_F_HIGHDMA;
#endif

    switch (mcfg) {
    case CFG_METHOD_1:
    case CFG_METHOD_2:
    case CFG_METHOD_3:
        tp->intr_mask = RxDescUnavail | TxDescUnavail | TxOK | RxOK | SWInt;
        break;
    default:
        tp->intr_mask = RxDescUnavail | TxOK | RxOK | SWInt;
        break;
    }

    spin_lock_init(&tp->lock);

    spin_lock_init(&tp->phy_lock);

    rtl8101_get_bios_setting(dev);

    rtl8101_exit_oob(dev);

    rtl8101_hw_init(dev);

    rtl8101_hw_reset(dev);

    /* Get production from EEPROM */
    if (mcfg == MCFG_8168E_VL_2 && (mac_ocp_read(0xDC00) & BIT_3))
        tp->eeprom_type = EEPROM_TYPE_NONE;
    else
        rtl_eeprom_type(tp);

    if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
        rtl_set_eeprom_sel_low(ioaddr);

    rtl8101_get_mac_address(dev);

    pci_set_drvdata(pdev, dev);

    if (netif_msg_probe(tp)) {
        printk(KERN_INFO "%s: %s at 0x%lx, "
               "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
               "IRQ %d\n",
               dev->name,
               rtl_chip_info[ent->driver_data].name,
               dev->base_addr,
               dev->dev_addr[0], dev->dev_addr[1],
               dev->dev_addr[2], dev->dev_addr[3],
               dev->dev_addr[4], dev->dev_addr[5], dev->irq);
    }

    rtl8101_link_option(board_idx, &autoneg, &speed, &duplex);

    rc = register_netdev(dev);
    if (rc) {
        rtl8101_release_board(pdev, dev, ioaddr);
        return rc;
    }

    printk(KERN_INFO "%s: This product is covered by one or more of the following patents: US6,570,884, US6,115,776, and US6,327,625.\n", MODULENAME);

    if (netif_msg_probe(tp)) {
        printk(KERN_DEBUG "%s: Identified chip type is '%s'.\n",
               dev->name, rtl_chip_info[tp->chipset].name);
    }

    netif_carrier_off(dev);

    printk("%s", GPL_CLAIM);

    return 0;
}

static void __devexit
rtl8101_remove_one(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);

    assert(dev != NULL);
    assert(tp != NULL);

    flush_scheduled_work();

    unregister_netdev(dev);

    rtl8101_disable_msi(pdev, tp);
    rtl8101_release_board(pdev, dev, tp->mmio_addr);
    pci_set_drvdata(pdev, NULL);
}

static void rtl8101_set_rxbufsize(struct rtl8101_private *tp,
                                  struct net_device *dev)
{
    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        tp->rx_buf_sz = 0x05F3;
        break;
    default:
        tp->rx_buf_sz = 0x05EF;
        break;
    }
}

static int rtl8101_open(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    int retval;

    retval = -ENOMEM;

    rtl8101_set_rxbufsize(tp, dev);

    /*
     * Rx and Tx desscriptors needs 256 bytes alignment.
     * pci_alloc_consistent provides more.
     */
    tp->TxDescArray = pci_alloc_consistent(pdev, R8101_TX_RING_BYTES,
                                           &tp->TxPhyAddr);
    if (!tp->TxDescArray)
        goto out;

    tp->RxDescArray = pci_alloc_consistent(pdev, R8101_RX_RING_BYTES,
                                           &tp->RxPhyAddr);
    if (!tp->RxDescArray)
        goto err_free_tx;

    tp->tally_vaddr = pci_alloc_consistent(pdev, sizeof(*tp->tally_vaddr), &tp->tally_paddr);
    if (!tp->tally_vaddr)
        goto err_free_rx;

    retval = rtl8101_init_ring(dev);
    if (retval < 0)
        goto err_free_counters;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    INIT_WORK(&tp->task, NULL, dev);
#else
    INIT_DELAYED_WORK(&tp->task, NULL);
#endif

#ifdef	CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif

    rtl8101_exit_oob(dev);

    rtl8101_tally_counter_clear(tp);

    rtl8101_hw_init(dev);

    rtl8101_hw_reset(dev);

    rtl8101_powerup_pll(dev);

    rtl8101_hw_ephy_config(dev);

    rtl8101_hw_phy_config(dev);

    rtl8101_hw_start(dev);

    rtl8101_dsm(dev, DSM_IF_UP);

    rtl8101_set_speed(dev, autoneg, speed, duplex);

    retval = request_irq(dev->irq, rtl8101_interrupt, (tp->features & RTL_FEATURE_MSI) ? 0 : SA_SHIRQ, dev->name, dev);

    if (retval < 0)
        goto err_free_counters;

    if (tp->esd_flag == 0)
        rtl8101_request_esd_timer(dev);

    rtl8101_request_link_timer(dev);
out:
    return retval;

err_free_counters:
    pci_free_consistent(pdev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);

    tp->tally_vaddr = NULL;
err_free_rx:
    pci_free_consistent(pdev, R8101_RX_RING_BYTES, tp->RxDescArray,
                        tp->RxPhyAddr);
    tp->RxDescArray = NULL;
err_free_tx:
    pci_free_consistent(pdev, R8101_TX_RING_BYTES, tp->TxDescArray,
                        tp->TxPhyAddr);
    tp->TxDescArray = NULL;
    goto out;
}

static void
rtl8101_dsm(struct net_device *dev, int dev_state)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (dev_state) {
    case DSM_MAC_INIT:
        if ((mcfg == CFG_METHOD_4) ||
            (mcfg == CFG_METHOD_5) ||
            (mcfg == CFG_METHOD_6)) {
            if (ReadMMIO8(MACDBG) & 0x80) {
                WriteGMII16( 0x1f, 0x0000);
                WriteGMII16( 0x11, ReadGMII16( 0x11) & ~(1 << 12));
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) | GPIO_en);
            } else {
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) & ~GPIO_en);
            }
        }

        break;
    case DSM_NIC_GOTO_D3:
    case DSM_IF_DOWN:
        if (ReadMMIO8(MACDBG) & 0x80) {
            if ((mcfg == CFG_METHOD_4) || (mcfg == CFG_METHOD_5)) {
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) | GPIO_en);
                WriteGMII16( 0x11, ReadGMII16( 0x11) | (1 << 12));
            } else if (mcfg == CFG_METHOD_6) {
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) & ~GPIO_en);
            }
        }
        break;
    case DSM_NIC_RESUME_D3:
    case DSM_IF_UP:
        if (ReadMMIO8(MACDBG) & 0x80) {
            if ((mcfg == CFG_METHOD_4) || (mcfg == CFG_METHOD_5)) {
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) & ~GPIO_en);
            } else if (mcfg == CFG_METHOD_6) {
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) | GPIO_en);
            }
        }

        break;
    }

}

static void
rtl8101_hw_set_rx_packet_filter(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u32 mc_filter[2];	/* Multicast hash filter */
    int i, j, rx_mode;
    u32 tmp = 0;

    if (dev->flags & IFF_PROMISC) {
        /* Unconditionally log net taps. */
        if (netif_msg_link(tp)) {
            printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n",
                   dev->name);
        }
        rx_mode =
            AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
            AcceptAllPhys;
        mc_filter[1] = mc_filter[0] = 0xffffffff;
    } else if ((netdev_mc_count(dev) > multicast_filter_limit)
               || (dev->flags & IFF_ALLMULTI)) {
        /* Too many to filter perfectly -- accept all multicasts. */
        rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
        mc_filter[1] = mc_filter[0] = 0xffffffff;
    } else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
        struct dev_mc_list *mclist;
        rx_mode = AcceptBroadcast | AcceptMyPhys;
        mc_filter[1] = mc_filter[0] = 0;
        for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
             i++, mclist = mclist->next) {
            int bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;
            mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
            rx_mode |= AcceptMulticast;
        }
#else
        struct netdev_hw_addr *ha;

        rx_mode = AcceptBroadcast | AcceptMyPhys;
        mc_filter[1] = mc_filter[0] = 0;
        netdev_for_each_mc_addr(ha, dev) {
            int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;
            mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
            rx_mode |= AcceptMulticast;
        }
#endif
    }

    tmp = rtl8101_rx_config | rx_mode |
          (ReadMMIO32(RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);

    for (i = 0; i < 2; i++) {
        u32 mask = 0x000000ff;
        u32 tmp1 = 0;
        u32 tmp2 = 0;
        int x = 0;
        int y = 0;

        for (j = 0; j < 4; j++) {
            tmp1 = mc_filter[i] & mask;
            x = 32 - (8 + 16 * j);
            y = x - 2 * x;

            if (x > 0)
                tmp2 = tmp2 | (tmp1 << x);
            else
                tmp2 = tmp2 | (tmp1 >> y);

            mask = mask << 8;
        }
        mc_filter[i] = tmp2;
    }

    WriteMMIO32(RxConfig, tmp);
    WriteMMIO32(MAR0 + 0, mc_filter[1]);
    WriteMMIO32(MAR0 + 4, mc_filter[0]);
}

static void
rtl8101_set_rx_mode(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    rtl8101_hw_set_rx_packet_filter(dev);

    spin_unlock_irqrestore(&tp->lock, flags);
}

static void
rtl8101_hw_start(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    struct pci_dev *pdev = tp->pci_dev;
    u8 link_control, options1, options2;
    u32 csi_tmp;
    unsigned long flags;

    netif_stop_queue(dev);

    WriteMMIO32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));

    rtl8101_hw_reset(dev);

    WriteMMIO8(Cfg9346, Cfg9346_Unlock);
    switch (mcfg) {
    case MCFG_8105E:
    case MCFG_8105E_2:
    case MCFG_8105E_3:
    case MCFG_8105E_4:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8106E_2:
    case MCFG_8106EUS:
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        break;
    }
    WriteMMIO8(MTPS, Reserved1_data);

    /* Set DMA burst size and Interframe Gap Time */
    WriteMMIO32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
            (InterFrameGap << TxInterFrameGapShift));

    cp_cmd &= 0x2063;

    WriteMMIO16(IntrMitigate, 0x0000);

    rtl8101_tally_counter_addr_fill(tp);

    rtl8101_desc_addr_fill(tp);

    if (mcfg == CFG_METHOD_4) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);

            WriteMMIO8(DBG_reg, 0x98);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | BIT_2);

            pci_write_config_byte(pdev, 0x81, 1);
        }

        WriteMMIO8(Config1, 0x0f);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == CFG_METHOD_5) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);

            WriteMMIO8(DBG_reg, 0x98);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | BIT_2);

            pci_write_config_byte(pdev, 0x81, 1);
        }

        set_offset79(tp, 0x50);

        WriteMMIO8(Config1, 0x0f);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == CFG_METHOD_6) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);

            WriteMMIO8(DBG_reg, 0x98);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | BIT_2);

            pci_write_config_byte(pdev, 0x81, 1);
        }

        set_offset79(tp, 0x50);

//		WriteMMIO8(Config1, 0xDF);

        WriteMMIO8(0xF4, 0x01);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == CFG_METHOD_7) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);

            WriteMMIO8(DBG_reg, 0x98);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | BIT_2);

            pci_write_config_byte(pdev, 0x81, 1);
        }

        set_offset79(tp, 0x50);

//		WriteMMIO8(Config1, (ReadMMIO8(Config1)&0xC0)|0x1F);

        WriteMMIO8(0xF4, 0x01);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        WriteMMIO8(0xF5, ReadMMIO8(0xF5) | BIT_2);
    } else if (mcfg == CFG_METHOD_8) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);

            WriteMMIO8(DBG_reg, 0x98);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | BIT_2);
            WriteMMIO8(0xF4, ReadMMIO8(0xF4) | BIT_3);
            WriteMMIO8(0xF5, ReadMMIO8(0xF5) | BIT_2);

            pci_write_config_byte(pdev, 0x81, 1);

            if (rtl8101_ephy_read(ioaddr, 0x10)==0x0008) {
                rtl8101_ephy_write(ioaddr, 0x10, 0x000C);
            }
        }

        pci_read_config_byte(pdev, 0x80, &link_control);
        if (link_control & 3)
            rtl8101_ephy_write(ioaddr, 0x02, 0x011F);

        set_offset79(tp, 0x50);

//		WriteMMIO8(Config1, (ReadMMIO8(Config1)&0xC0)|0x1F);

        WriteMMIO8(0xF4, ReadMMIO8(0xF4) | BIT_0);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == CFG_METHOD_9) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);

            WriteMMIO8(DBG_reg, 0x98);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | BIT_2);

            pci_write_config_byte(pdev, 0x81, 1);
        }

        set_offset79(tp, 0x50);

//		WriteMMIO8(Config1, 0xDF);

        WriteMMIO8(0xF4, 0x01);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == MCFG_8105E) {
        set_offset70F(tp, 0x27);
        set_offset79(tp, 0x50);

        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;

        WriteMMIO8(0xF3, ReadMMIO8(0xF3) | BIT_5);
        WriteMMIO8(0xF3, ReadMMIO8(0xF3) & ~BIT_5);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_7 | BIT_6);

        WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_6 | BIT_5 | BIT_4 | BIT_2 | BIT_1);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

        WriteMMIO8(Config5, (ReadMMIO8(Config5)&~0x08) | BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
               mcfg == MCFG_8168DP_3) {
        u8	pci_config;

        cp_cmd &= 0x2063;

        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;

        pci_read_config_byte(pdev, 0x80, &pci_config);
        if (pci_config & 0x03) {
            WriteMMIO8(Config5, ReadMMIO8(Config5) | BIT_0);
            WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_7);
            if (aspm)
                WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
        }

        WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_5 | BIT_3);
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) & ~BIT_0);
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) | BIT_3 | BIT_2);
        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_6);
        WriteMMIO16(0xE0, ReadMMIO16(0xE0) & ~0xDF9C);

        if (mcfg == MCFG_8168DP_1)
            WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
    } else if (mcfg == MCFG_8402_1) {
        set_offset70F(tp, 0x27);
        set_offset79(tp, 0x50);

        WriteERI(0xC8, 4, 0x00000002, ERIAR_ExGMAC);
        WriteERI(0xE8, 4, 0x00000006, ERIAR_ExGMAC);
        WriteMMIO32(TxConfig, ReadMMIO32(TxConfig) | BIT_7);
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) & ~BIT_7);
        csi_tmp = ReadERI(0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        rtl8101_eri_write( ioaddr, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        rtl8101_eri_write( ioaddr, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);

        rtl8101_ephy_write(ioaddr, 0x19, 0xff64);

        WriteMMIO8(Config5, ReadMMIO8(Config5) | BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);

        WriteERI(0xC0, 2, 0x00000000, ERIAR_ExGMAC);
        WriteERI(0xB8, 2, 0x00000000, ERIAR_ExGMAC);
        WriteERI(0xD5, 1, 0x0000000E, ERIAR_ExGMAC);
    } else if (mcfg == MCFG_8106E_1 || mcfg == MCFG_8106E_2) {
        u8	pci_config;

        cp_cmd &= 0x2063;

        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;

        pci_read_config_byte(pdev, 0x80, &pci_config);
        if (pci_config & 0x03) {
            WriteMMIO8(Config5, ReadMMIO8(Config5) | BIT_0);
            WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_7);
            if (aspm)
                WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
        }

        WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_5 | BIT_3);
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) & ~BIT_0);
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) | BIT_3 | BIT_2);
        WriteMMIO8(0xD0, ReadMMIO8(0xD0) & ~BIT_6);
        WriteMMIO16(0xE0, ReadMMIO16(0xE0) & ~0xDF9C);
    } else if (mcfg == MCFG_8106EUS) {
        u8 data8;

        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        WriteERI(0xC8, 4, 0x00080002, ERIAR_ExGMAC);
        WriteERI(0xCC, 1, 0x38, ERIAR_ExGMAC);
        WriteERI(0xD0, 1, 0x48, ERIAR_ExGMAC);
        WriteERI(0xE8, 4, 0x00100006, ERIAR_ExGMAC);

        WriteMMIO32(TxConfig, ReadMMIO32(TxConfig) | BIT_7);

        csi_tmp = ReadERI(0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        cp_cmd = ReadMMIO16(CPlusCmd) &
                     ~(EnableBist | Macdbgo_oe | Force_halfdup |
                       Force_rxflow_en | Force_txflow_en |
                       Cxpl_dbg_sel | ASF | PktCntrDisable |
                       Macdbgo_sel);

        WriteMMIO8(0x1B, ReadMMIO8(0x1B) & ~0x07);

        WriteMMIO8(TDFNR, 0x4);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_6);
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_6);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_7);

        WriteERI(0xC0, 2, 0x0000, ERIAR_ExGMAC);
        WriteERI(0xB8, 4, 0x00000000, ERIAR_ExGMAC);

        WriteERI(0x5F0, 2, 0x4f87, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0xD4, 4, ERIAR_ExGMAC);
        csi_tmp  |= ( BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12 );
        WriteERI(0xD4, 4, csi_tmp, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0x1B0, 4, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_12;
        WriteERI(0x1B0, 4, csi_tmp, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0x2FC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~(BIT_0 | BIT_1 | BIT_2);
        csi_tmp |= BIT_0;
        WriteERI(0x2FC, 1, csi_tmp, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0x1D0, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_1;
        WriteERI(0x1D0, 1, csi_tmp, ERIAR_ExGMAC);

        if (aspm) {
            csi_tmp = ReadERI(0x3F2, 2, ERIAR_ExGMAC);
            csi_tmp &= ~( BIT_8 | BIT_9  | BIT_10 | BIT_11  | BIT_12  | BIT_13  | BIT_14 | BIT_15 );
            csi_tmp |= ( BIT_9 | BIT_10 | BIT_13  | BIT_14 | BIT_15 );
            WriteERI(0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
            csi_tmp = ReadERI(0x3F5, 1, ERIAR_ExGMAC);
            csi_tmp |= BIT_6 | BIT_7;
            WriteERI(0x3F5, 1, csi_tmp, ERIAR_ExGMAC);
            mac_ocp_write(0xE02C, 0x1880);
            mac_ocp_write(0xE02E, 0x4880);
            WriteERI(0x2E8, 2, 0x9003, ERIAR_ExGMAC);
            WriteERI(0x2EA, 2, 0x9003, ERIAR_ExGMAC);
            WriteERI(0x2EC, 2, 0x9003, ERIAR_ExGMAC);
            WriteERI(0x2E2, 2, 0x883C, ERIAR_ExGMAC);
            WriteERI(0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
            WriteERI(0x2E6, 2, 0x9003, ERIAR_ExGMAC);
            csi_tmp = ReadERI(0x3FA, 2, ERIAR_ExGMAC);
            csi_tmp |= BIT_14;
            WriteERI(0x3FA, 2, csi_tmp, ERIAR_ExGMAC);
            csi_tmp = ReadERI(0x3F2, 2, ERIAR_ExGMAC);
            csi_tmp &= ~(BIT_0 | BIT_1);
            csi_tmp |= BIT_0;
            pci_read_config_byte(pdev, 0x99, &data8);
            if (!(data8 & (BIT_5 | BIT_6)))
                csi_tmp &= ~(BIT_1);
            if (!(data8 & BIT_2))
                csi_tmp &= ~(BIT_0 );

            WriteERI(0x3F2, 2, csi_tmp, ERIAR_ExGMAC);

            pci_read_config_byte(pdev, 0x180, &data8);
            if (data8 & (BIT_0|BIT_1)) {
                csi_tmp = ReadERI(0x1E2, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_2;
                WriteERI(0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
            } else {
                csi_tmp = ReadERI(0x1E2, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_2;
                WriteERI(0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
            }
        }
    }

    if ((mcfg == CFG_METHOD_1) ||
        (mcfg == CFG_METHOD_2) ||
        (mcfg == CFG_METHOD_3)) {
        /* csum offload command for RTL8101E */
        tp->tx_tcp_csum_cmd = TxIPCS | TxTCPCS;
        tp->tx_udp_csum_cmd = TxIPCS | TxUDPCS;
        tp->tx_ip_csum_cmd = TxIPCS;
    } else {
        /* csum offload command for RTL8102E */
        tp->tx_tcp_csum_cmd = TxIPCS_C | TxTCPCS_C;
        tp->tx_udp_csum_cmd = TxIPCS_C | TxUDPCS_C;
        tp->tx_ip_csum_cmd = TxIPCS_C;
    }

    //other hw parameretrs
    if (mcfg == MCFG_8168E_VL_2)
        WriteERI(0x2F8, 2, 0x1D8F, ERIAR_ExGMAC);

    if (bios_setting & BIT_28) {
        if (mcfg == MCFG_8168DP_3) {
            if (ReadMMIO8(0xEF) & BIT_2) {
                u32 gphy_val;

                spin_lock_irqsave(&tp->phy_lock, flags);
                WriteGMII16( 0x1F, 0x0001);
                gphy_val = ReadGMII16( 0x1B);
                gphy_val |= BIT_2;
                WriteGMII16( 0x1B, gphy_val);
                WriteGMII16( 0x1F, 0x0000);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
        }

        if (mcfg == MCFG_8402_1) {
            u32 gphy_val;

            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1F, 0x0001);
            gphy_val = ReadGMII16( 0x13);
            gphy_val |= BIT_15;
            WriteGMII16( 0x13, gphy_val);
            WriteGMII16( 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    WriteMMIO16(CPlusCmd, cp_cmd);
#else
    rtl8101_hw_set_features(dev, dev->features);
#endif

    switch (mcfg) {
    case MCFG_8168E_VL_2: {
        int timeout;
        for (timeout = 0; timeout < 10; timeout++) {
            if ((ReadERI(0x1AE, 2, ERIAR_ExGMAC) & BIT_13)==0)
                break;
            mdelay(1);
        }
    }
    break;
    }

    WriteMMIO16(RxMaxSize, tp->rx_buf_sz);

    rtl8101_disable_rxdvgate(dev);

    if (!tp->pci_cfg_is_read) {
        pci_read_config_byte(pdev, PCI_COMMAND, &tp->pci_cfg_space.cmd);
        pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &tp->pci_cfg_space.cls);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &tp->pci_cfg_space.io_base_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &tp->pci_cfg_space.io_base_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &tp->pci_cfg_space.mem_base_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &tp->pci_cfg_space.mem_base_h);
        pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &tp->pci_cfg_space.ilr);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &tp->pci_cfg_space.resv_0x20_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &tp->pci_cfg_space.resv_0x20_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &tp->pci_cfg_space.resv_0x24_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &tp->pci_cfg_space.resv_0x24_h);

        tp->pci_cfg_is_read = 1;
    }

    rtl8101_dsm(dev, DSM_MAC_INIT);

    options1 = ReadMMIO8(Config3);
    options2 = ReadMMIO8(Config5);
    if ((options1 & (LinkUp | MagicPacket)) || (options2 & (UWF | BWF | MWF)))
        tp->wol_enabled = WOL_ENABLED;
    else
        tp->wol_enabled = WOL_DISABLED;

    /* Set Rx packet filter */
    rtl8101_hw_set_rx_packet_filter(dev);

    switch (mcfg) {
    case MCFG_8105E:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8402_1:
    case MCFG_8106E_1:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        if (aspm) {
            WriteMMIO8(Config5, ReadMMIO8(Config5) | BIT_0);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
        } else {
            WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
            WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        }
        break;
    }

    WriteMMIO8(Cfg9346, Cfg9346_Lock);

    WriteMMIO8(ChipCmd, CmdTxEnb | CmdRxEnb);

    /* Enable all known interrupts by setting the interrupt mask. */
    WriteMMIO16(IntrMask, tp->intr_mask);
    netif_start_queue(dev);


    IODelay(10);
}

static int
rtl8101_change_mtu(struct net_device *dev,
                   int new_mtu)
{
    int ret = 0;

    if (new_mtu < ETH_ZLEN || new_mtu > ETH_DATA_LEN)
        return -EINVAL;

    dev->mtu = new_mtu;

    return ret;
}

static inline void
rtl8101_make_unusable_by_asic(struct RxDesc *desc)
{
    desc->addr = 0x0badbadbadbadbadull;
    desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

static void
rtl8101_free_rx_skb(struct rtl8101_private *tp,
                    struct sk_buff **sk_buff,
                    struct RxDesc *desc)
{
    struct pci_dev *pdev = tp->pci_dev;

    pci_unmap_single(pdev, le64_to_cpu(desc->addr), tp->rx_buf_sz,
                     PCI_DMA_FROMDEVICE);
    dev_kfree_skb(*sk_buff);
    *sk_buff = NULL;
    rtl8101_make_unusable_by_asic(desc);
}

static inline void
rtl8101_mark_to_asic(struct RxDesc *desc,
                     u32 rx_buf_sz)
{
    u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

    desc->opts1 = cpu_to_le32(DescOwn | eor | rx_buf_sz);
}

static inline void
rtl8101_map_to_asic(struct RxDesc *desc,
                    dma_addr_t mapping,
                    u32 rx_buf_sz)
{
    desc->addr = cpu_to_le64(mapping);
    wmb();
    rtl8101_mark_to_asic(desc, rx_buf_sz);
}

static int
rtl8101_alloc_rx_skb(struct pci_dev *pdev,
                     struct sk_buff **sk_buff,
                     struct RxDesc *desc,
                     int rx_buf_sz)
{
    struct sk_buff *skb;
    dma_addr_t mapping;
    int ret = 0;

    skb = dev_alloc_skb(rx_buf_sz + RTK_RX_ALIGN);
    if (!skb)
        goto err_out;

    skb_reserve(skb, RTK_RX_ALIGN);
    *sk_buff = skb;

    mapping = pci_map_single(pdev, skb->data, rx_buf_sz,
                             PCI_DMA_FROMDEVICE);

    rtl8101_map_to_asic(desc, mapping, rx_buf_sz);

out:
    return ret;

err_out:
    ret = -ENOMEM;
    rtl8101_make_unusable_by_asic(desc);
    goto out;
}

static void
rtl8101_rx_clear(struct rtl8101_private *tp)
{
    int i;

    for (i = 0; i < NUM_RX_DESC; i++) {
        if (tp->Rx_skbuff[i]) {
            rtl8101_free_rx_skb(tp, tp->Rx_skbuff + i,
                                tp->RxDescArray + i);
        }
    }
}

static u32
rtl8101_rx_fill(struct rtl8101_private *tp,
                struct net_device *dev,
                u32 start, u32 end)
{
    u32 cur;

    for (cur = start; end - cur > 0; cur++) {
        int ret, i = cur % NUM_RX_DESC;

        if (tp->Rx_skbuff[i])
            continue;

        ret = rtl8101_alloc_rx_skb(tp->pci_dev, tp->Rx_skbuff + i,
                                   tp->RxDescArray + i, tp->rx_buf_sz);
        if (ret < 0)
            break;
    }
    return cur - start;
}

static inline void
rtl8101_mark_as_last_descriptor(struct RxDesc *desc)
{
    desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8101_desc_addr_fill(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;

    if (!tp->TxPhyAddr || !tp->RxPhyAddr)
        return;

    WriteMMIO32(TxDescStartAddrLow, ((u64) tp->TxPhyAddr & DMA_BIT_MASK(32)));
    WriteMMIO32(TxDescStartAddrHigh, ((u64) tp->TxPhyAddr >> 32));
    WriteMMIO32(RxDescAddrLow, ((u64) tp->RxPhyAddr & DMA_BIT_MASK(32)));
    WriteMMIO32(RxDescAddrHigh, ((u64) tp->RxPhyAddr >> 32));
}

static void
rtl8101_tx_desc_init(struct rtl8101_private *tp)
{
    int i = 0;

    memset(tp->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));

    for (i = 0; i < NUM_TX_DESC; i++) {
        if (i == (NUM_TX_DESC - 1))
            tp->TxDescArray[i].opts1 = cpu_to_le32(RingEnd);
    }
}

static void
rtl8101_rx_desc_init(struct rtl8101_private *tp)
{
    memset(tp->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));
}

static int
rtl8101_init_ring(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    rtl8101_init_ring_indexes(tp);

    memset(tp->tx_skb, 0x0, NUM_TX_DESC * sizeof(struct ring_info));
    memset(tp->Rx_skbuff, 0x0, NUM_RX_DESC * sizeof(struct sk_buff *));

    rtl8101_tx_desc_init(tp);
    rtl8101_rx_desc_init(tp);

    if (rtl8101_rx_fill(tp, dev, 0, NUM_RX_DESC) != NUM_RX_DESC)
        goto err_out;

    rtl8101_mark_as_last_descriptor(tp->RxDescArray + NUM_RX_DESC - 1);

    return 0;

err_out:
    rtl8101_rx_clear(tp);
    return -ENOMEM;
}

static void
rtl8101_unmap_tx_skb(struct pci_dev *pdev,
                     struct ring_info *tx_skb,
                     struct TxDesc *desc)
{
    unsigned int len = tx_skb->len;

    pci_unmap_single(pdev, le64_to_cpu(desc->addr), len, PCI_DMA_TODEVICE);
    desc->opts1 = 0x00;
    desc->opts2 = 0x00;
    desc->addr = 0x00;
    tx_skb->len = 0;
}

static void
rtl8101_tx_clear(struct rtl8101_private *tp)
{
    unsigned int i;
    struct net_device *dev = tp->dev;

    for (i = tp->dirty_tx; i < tp->dirty_tx + NUM_TX_DESC; i++) {
        unsigned int entry = i % NUM_TX_DESC;
        struct ring_info *tx_skb = tp->tx_skb + entry;
        unsigned int len = tx_skb->len;

        if (len) {
            struct sk_buff *skb = tx_skb->skb;

            rtl8101_unmap_tx_skb(tp->pci_dev, tx_skb,
                                 tp->TxDescArray + entry);
            if (skb) {
                dev_kfree_skb(skb);
                tx_skb->skb = NULL;
            }
            RTLDEV->stats.tx_dropped++;
        }
    }
    tp->cur_tx = tp->dirty_tx = 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8101_schedule_work(struct net_device *dev, void (*task)(void *))
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    struct rtl8101_private *tp = netdev_priv(dev);

    PREPARE_WORK(&tp->task, task, dev);
    schedule_delayed_work(&tp->task, 4);
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
}
#else
static void rtl8101_schedule_work(struct net_device *dev, work_func_t task)
{
    struct rtl8101_private *tp = netdev_priv(dev);

    PREPARE_DELAYED_WORK(&tp->task, task);
    schedule_delayed_work(&tp->task, 4);
}
#endif

static void
rtl8101_wait_for_quiescence(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    synchronize_irq(dev->irq);

    /* Wait for any pending NAPI task to complete */
#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_DISABLE(dev, &tp->napi);
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#endif

    rtl8101_irq_mask_and_ack(ioaddr);

#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#endif
}

#if 0
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8101_reinit_task(void *_data)
#else
static void rtl8101_reinit_task(struct work_struct *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    struct net_device *dev = _data;
#else
    struct rtl8101_private *tp =
        container_of(work, struct rtl8101_private, task.work);
    struct net_device *dev = tp->dev;
#endif
    int ret;

    if (netif_running(dev)) {
        rtl8101_wait_for_quiescence(dev);
        rtl8101_close(dev);
    }

    ret = rtl8101_open(dev);
    if (unlikely(ret < 0)) {
        if (net_ratelimit()) {
            struct rtl8101_private *tp = netdev_priv(dev);

            if (netif_msg_drv(tp)) {
                printk(PFX KERN_ERR
                       "%s: reinit failure (status = %d)."
                       " Rescheduling.\n", dev->name, ret);
            }
        }
        rtl8101_schedule_work(dev, rtl8101_reinit_task);
    }
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8101_reset_task(void *_data)
{
    struct net_device *dev = _data;
    struct rtl8101_private *tp = netdev_priv(dev);
#else
static void rtl8101_reset_task(struct work_struct *work)
{
    struct rtl8101_private *tp =
        container_of(work, struct rtl8101_private, task.work);
    struct net_device *dev = tp->dev;
#endif
    unsigned long flags;

    if (!netif_running(dev))
        return;

    rtl8101_wait_for_quiescence(dev);

    rtl8101_rx_interrupt(dev, tp, tp->mmio_addr, ~(u32)0);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8101_tx_clear(tp);

    if (tp->dirty_rx == tp->cur_rx) {
        rtl8101_init_ring_indexes(tp);
        rtl8101_hw_start(dev);
        rtl8101_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
        netif_wake_queue(dev);
        spin_unlock_irqrestore(&tp->lock, flags);
    } else {
        spin_unlock_irqrestore(&tp->lock, flags);
        if (net_ratelimit()) {
            struct rtl8101_private *tp = netdev_priv(dev);

            if (netif_msg_intr(tp)) {
                printk(PFX KERN_EMERG
                       "%s: Rx buffers shortage\n", dev->name);
            }
        }
        rtl8101_schedule_work(dev, rtl8101_reset_task);
    }
}

static void
rtl8101_tx_timeout(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    rtl8101_hw_reset(dev);
    spin_unlock_irqrestore(&tp->lock, flags);

    /* Let's wait a bit while any (async) irq lands on */
    rtl8101_schedule_work(dev, rtl8101_reset_task);
}

static int
rtl8101_xmit_frags(struct rtl8101_private *tp,
                   struct sk_buff *skb,
                   u32 opts1,
                   u32 opts2)
{
    struct skb_shared_info *info = skb_shinfo(skb);
    unsigned int cur_frag, entry;
    struct TxDesc *txd = NULL;

    entry = tp->cur_tx;
    for (cur_frag = 0; cur_frag < info->nr_frags; cur_frag++) {
        skb_frag_t *frag = info->frags + cur_frag;
        dma_addr_t mapping;
        u32 status, len;
        void *addr;

        entry = (entry + 1) % NUM_TX_DESC;

        txd = tp->TxDescArray + entry;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
        len = frag->size;
        addr = ((void *) page_address(frag->page)) + frag->page_offset;
#else
        len = skb_frag_size(frag);
        addr = skb_frag_address(frag);
#endif
        mapping = pci_map_single(tp->pci_dev, addr, len, PCI_DMA_TODEVICE);

        /* anti gcc 2.95.3 bugware (sic) */
        status = opts1 | len | (RingEnd * !((entry + 1) % NUM_TX_DESC));

        txd->addr = cpu_to_le64(mapping);

        tp->tx_skb[entry].len = len;

        txd->opts1 = cpu_to_le32(status);
        txd->opts2 = cpu_to_le32(opts2);
    }

    if (cur_frag) {
        tp->tx_skb[entry].skb = skb;
        wmb();
        txd->opts1 |= cpu_to_le32(LastFrag);
    }

    return cur_frag;
}

static inline u32
rtl8101_tx_csum(struct sk_buff *skb,
                struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    const struct iphdr *ip = skb->nh.iph;
#else
    const struct iphdr *ip = ip_hdr(skb);
#endif

    if (skb->ip_summed == CHECKSUM_PARTIAL) {
        if (ip->protocol == IPPROTO_TCP)
            return tp->tx_tcp_csum_cmd;
        else if (ip->protocol == IPPROTO_UDP)
            return tp->tx_udp_csum_cmd;
        else if (ip->protocol == IPPROTO_IP)
            return tp->tx_ip_csum_cmd;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        WARN_ON(1);	/* we need a WARN() */
#endif
    }

    return 0;
}

static int
rtl8101_start_xmit(struct sk_buff *skb,
                   struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned int frags, entry = tp->cur_tx % NUM_TX_DESC;
    struct TxDesc *txd = tp->TxDescArray + entry;
    void __iomem *ioaddr = tp->mmio_addr;
    dma_addr_t mapping;
    u32 len;
    u32 opts1;
    u32 opts2;
    int ret = NETDEV_TX_OK;
    unsigned long flags, large_send;

    spin_lock_irqsave(&tp->lock, flags);

    if (unlikely(TX_BUFFS_AVAIL(tp) < skb_shinfo(skb)->nr_frags)) {
        if (netif_msg_drv(tp)) {
            printk(KERN_ERR
                   "%s: BUG! Tx Ring full when queue awake!\n",
                   dev->name);
        }
        goto err_stop;
    }

    if (unlikely(le32_to_cpu(txd->opts1) & DescOwn))
        goto err_stop;

    opts1 = DescOwn;
    opts2 = rtl8101_tx_vlan_tag(tp, skb);

    large_send = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    if (dev->features & NETIF_F_TSO) {
        u32 mss = skb_is_gso(skb);

        /* TCP Segmentation Offload (or TCP Large Send) */
        if (mss) {
            if ((mcfg == CFG_METHOD_1) ||
                (mcfg == CFG_METHOD_2) ||
                (mcfg == CFG_METHOD_3)) {
                opts1 |= LargeSend | ((mss & MSSMask) << 16);
            } else {
                opts1 |= LargeSend;
                opts2 |= (mss & MSSMask) << 18;
            }
            large_send = 1;
        }
    }
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)	

    if (large_send == 0) {
        if (dev->features & NETIF_F_IP_CSUM) {
            if ((mcfg == CFG_METHOD_1) ||
                (mcfg == CFG_METHOD_2) ||
                (mcfg == CFG_METHOD_3))
                opts1 |= rtl8101_tx_csum(skb, dev);
            else
                opts2 |= rtl8101_tx_csum(skb, dev);
        }
    }

    frags = rtl8101_xmit_frags(tp, skb, opts1, opts2);
    if (frags) {
        len = skb_headlen(skb);
        opts1 |= FirstFrag;
    } else {
        len = skb->len;

        opts1 |= FirstFrag | LastFrag;

        tp->tx_skb[entry].skb = skb;
    }

    opts1 |= len | (RingEnd * !((entry + 1) % NUM_TX_DESC));
    mapping = pci_map_single(tp->pci_dev, skb->data, len, PCI_DMA_TODEVICE);
    tp->tx_skb[entry].len = len;
    txd->addr = cpu_to_le64(mapping);
    txd->opts2 = cpu_to_le32(opts2);
    txd->opts1 = cpu_to_le32(opts1&~DescOwn);
    wmb();
    txd->opts1 = cpu_to_le32(opts1);

    dev->trans_start = jiffies;

    tp->cur_tx += frags + 1;

    wmb();

    WriteMMIO8(TxPoll, NPQ);    /* set polling bit */

    if (TX_BUFFS_AVAIL(tp) < MAX_SKB_FRAGS) {
        netif_stop_queue(dev);
        rtl_rmb();
        if (TX_BUFFS_AVAIL(tp) >= MAX_SKB_FRAGS)
            netif_wake_queue(dev);
    }

    spin_unlock_irqrestore(&tp->lock, flags);

out:
    return ret;

err_stop:
    netif_stop_queue(dev);
    ret = NETDEV_TX_BUSY;
    RTLDEV->stats.tx_dropped++;

    spin_unlock_irqrestore(&tp->lock, flags);
    goto out;
}

static void
rtl8101_tx_interrupt(struct net_device *dev,
                     struct rtl8101_private *tp,
                     void __iomem *ioaddr)
{
    unsigned int dirty_tx, tx_left;

    assert(dev != NULL);
    assert(tp != NULL);
    assert(ioaddr != NULL);

    dirty_tx = tp->dirty_tx;
    rtl_rmb();
    tx_left = tp->cur_tx - dirty_tx;

    while (tx_left > 0) {
        unsigned int entry = dirty_tx % NUM_TX_DESC;
        struct ring_info *tx_skb = tp->tx_skb + entry;
        u32 len = tx_skb->len;
        u32 status;

        rmb();
        status = le32_to_cpu(tp->TxDescArray[entry].opts1);
        if (status & DescOwn)
            break;

        RTLDEV->stats.tx_bytes += len;
        RTLDEV->stats.tx_packets++;

        rtl8101_unmap_tx_skb(tp->pci_dev, tx_skb, tp->TxDescArray + entry);

        if (status & LastFrag) {
            dev_kfree_skb_irq(tx_skb->skb);
            tx_skb->skb = NULL;
        }
        dirty_tx++;
        tx_left--;
    }

    if (tp->dirty_tx != dirty_tx) {
        tp->dirty_tx = dirty_tx;
        rtl_wmb();
        if (netif_queue_stopped(dev) &&
            (TX_BUFFS_AVAIL(tp) >= MAX_SKB_FRAGS)) {
            netif_wake_queue(dev);
        }
        rtl_wmb();
        if (tp->cur_tx != dirty_tx)
            WriteMMIO8(TxPoll, NPQ);
    }
}

static inline int
rtl8101_fragmented_frame(u32 status)
{
    return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

static inline void
rtl8101_rx_csum(struct rtl8101_private *tp,
                struct sk_buff *skb,
                struct RxDesc *desc)
{
    u32 opts1 = le32_to_cpu(desc->opts1);
    u32 opts2 = le32_to_cpu(desc->opts2);
    u32 status = opts1 & RxProtoMask;

    if ((mcfg == CFG_METHOD_1) ||
        (mcfg == CFG_METHOD_2) ||
        (mcfg == CFG_METHOD_3)) {
        /* rx csum offload for RTL8101E */
        if (((status == RxProtoTCP) && !(opts1 & RxTCPF)) ||
            ((status == RxProtoUDP) && !(opts1 & RxUDPF)) ||
            ((status == RxProtoIP) && !(opts1 & RxIPF)))
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
            skb->ip_summed = CHECKSUM_NONE;
    } else {
        /* rx csum offload for RTL8102E */
        if (((status == RxTCPT) && !(opts1 & RxTCPF)) ||
            ((status == RxUDPT) && !(opts1 & RxUDPF)) ||
            ((status == 0) && (opts2 & RxV4F) && !(opts1 & RxIPF)))
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
            skb->ip_summed = CHECKSUM_NONE;
    }

}

static inline int
rtl8101_try_rx_copy(struct sk_buff **sk_buff,
                    int pkt_size,
                    struct RxDesc *desc,
                    int rx_buf_sz)
{
    int ret = -1;

    if (pkt_size < rx_copybreak) {
        struct sk_buff *skb;

        skb = dev_alloc_skb(pkt_size + NET_IP_ALIGN);
        if (skb) {
            skb_reserve(skb, NET_IP_ALIGN);
            eth_copy_and_sum(skb, sk_buff[0]->data, pkt_size, 0);
            *sk_buff = skb;
            ret = 0;
        }
    }
    return ret;
}

static inline void
rtl8101_rx_skb(struct rtl8101_private *tp,
               struct sk_buff *skb)
{
#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
    netif_receive_skb(skb);
#else
    napi_gro_receive(&tp->napi, skb);
#endif
#else
    netif_rx(skb);
#endif
}

static int
rtl8101_rx_interrupt(struct net_device *dev,
                     struct rtl8101_private *tp,
                     void __iomem *ioaddr, u32 budget)
{
    unsigned int cur_rx, rx_left;
    unsigned int delta, count = 0;
    u32 rx_quota = RTL_RX_QUOTA(dev, budget);

    assert(dev != NULL);
    assert(tp != NULL);
    assert(ioaddr != NULL);

    cur_rx = tp->cur_rx;
    rx_left = NUM_RX_DESC + tp->dirty_rx - cur_rx;
    rx_left = rtl8101_rx_quota(rx_left, (u32) rx_quota);

    if ((tp->RxDescArray == NULL) || (tp->Rx_skbuff == NULL))
        goto rx_out;

    for (; rx_left > 0; rx_left--, cur_rx++) {
        unsigned int entry = cur_rx % NUM_RX_DESC;
        struct RxDesc *desc = tp->RxDescArray + entry;
        u32 status;

        rmb();
        status = le32_to_cpu(desc->opts1);

        if (status & DescOwn)
            break;
        if (unlikely(status & RxRES)) {
            if (netif_msg_rx_err(tp)) {
                printk(KERN_INFO
                       "%s: Rx ERROR. status = %08x\n",
                       dev->name, status);
            }
            RTLDEV->stats.rx_errors++;

            if (status & (RxRWT | RxRUNT))
                RTLDEV->stats.rx_length_errors++;
            if (status & RxCRC)
                RTLDEV->stats.rx_crc_errors++;
            rtl8101_mark_to_asic(desc, tp->rx_buf_sz);
        } else {
            struct sk_buff *skb = tp->Rx_skbuff[entry];
            int pkt_size = (status & 0x00003FFF) - 4;

            /*
             * The driver does not support incoming fragmented
             * frames. They are seen as a symptom of over-mtu
             * sized frames.
             */
            if (unlikely(rtl8101_fragmented_frame(status))) {
                RTLDEV->stats.rx_dropped++;
                RTLDEV->stats.rx_length_errors++;
                rtl8101_mark_to_asic(desc, tp->rx_buf_sz);
                continue;
            }

            rtl8101_rx_csum(tp, skb, desc);

            pci_unmap_single(tp->pci_dev,
                             le64_to_cpu(desc->addr), tp->rx_buf_sz,
                             PCI_DMA_FROMDEVICE);

            if (rtl8101_try_rx_copy(&skb, pkt_size, desc,
                                    tp->rx_buf_sz)) {
                tp->Rx_skbuff[entry] = NULL;
            } else {
                dma_addr_t mapping;

                mapping = pci_map_single(tp->pci_dev, tp->Rx_skbuff[entry]->data, tp->rx_buf_sz,
                                         PCI_DMA_FROMDEVICE);
                rtl8101_map_to_asic(desc, mapping, tp->rx_buf_sz);
            }

            skb->dev = dev;
            skb_put(skb, pkt_size);
            skb->protocol = eth_type_trans(skb, dev);

            if (rtl8101_rx_vlan_skb(tp, desc, skb) < 0)
                rtl8101_rx_skb(tp, skb);

            dev->last_rx = jiffies;
            RTLDEV->stats.rx_bytes += pkt_size;
            RTLDEV->stats.rx_packets++;
        }
    }

    count = cur_rx - tp->cur_rx;
    tp->cur_rx = cur_rx;

    delta = rtl8101_rx_fill(tp, dev, tp->dirty_rx, tp->cur_rx);
    if (!delta && count && netif_msg_intr(tp))
        printk(KERN_INFO "%s: no Rx buffer allocated\n", dev->name);
    tp->dirty_rx += delta;

    /*
     * FIXME: until there is periodic timer to try and refill the ring,
     * a temporary shortage may definitely kill the Rx process.
     * - disable the asic to try and avoid an overflow and kick it again
     *   after refill ?
     * - how do others driver handle this condition (Uh oh...).
     */
    if ((tp->dirty_rx + NUM_RX_DESC == tp->cur_rx) && netif_msg_intr(tp))
        printk(KERN_EMERG "%s: Rx buffers exhausted\n", dev->name);

rx_out:

    return count;
}

static inline void
rtl8101_switch_to_timer_interrupt(struct rtl8101_private *tp, void __iomem *ioaddr)
{
    WriteMMIO32(TCTR, timer_count);
    WriteMMIO32(TimeIntr, timer_count);
    WriteMMIO16(IntrMask, PCSTimeout);
}

static inline void
rtl8101_switch_to_hw_interrupt(struct rtl8101_private *tp, void __iomem *ioaddr)
{
    WriteMMIO16(TimeIntr, 0x0000);
    WriteMMIO16(IntrMask, tp->intr_mask);
}

/* The interrupt handler does all of the Rx thread work and cleans up after the Tx thread. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
#else
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance)
#endif
{
    struct net_device *dev = (struct net_device *) dev_instance;
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int status;
    int handled = 0;

    do {
        status = ReadMMIO16(IntrStatus);

        /* hotplug/major error/no more work/shared irq */
        if ((status == 0xFFFF) || !status)
            break;

        if (!(status & (tp->intr_mask | PCSTimeout)))
            break;

        if (unlikely(!netif_running(dev))) {
            break;
        }

        handled = 1;

        WriteMMIO16(IntrMask, 0x0000);

        WriteMMIO16(IntrStatus, status);

#ifdef CONFIG_R8101_NAPI
        if (status & tp->intr_mask) {
            if (likely(RTL_NETIF_RX_SCHEDULE_PREP(dev, &tp->napi)))
                __RTL_NETIF_RX_SCHEDULE(dev, &tp->napi);
            else if (netif_msg_intr(tp))
                printk(KERN_INFO "%s: interrupt %04x in poll\n",
                       dev->name, status);
        } else
            rtl8101_switch_to_hw_interrupt(tp, ioaddr);
#else
        if (status & tp->intr_mask) {
            rtl8101_rx_interrupt(dev, tp, tp->mmio_addr, ~(u32)0);
            rtl8101_tx_interrupt(dev, tp, ioaddr);

            rtl8101_switch_to_timer_interrupt(tp, ioaddr);
        } else
            rtl8101_switch_to_hw_interrupt(tp, ioaddr);
#endif
    } while (false);

    return IRQ_RETVAL(handled);
}

#ifdef CONFIG_R8101_NAPI
static int rtl8101_poll(napi_ptr napi, napi_budget budget)
{
    struct rtl8101_private *tp = RTL_GET_PRIV(napi, struct rtl8101_private);
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_GET_NETDEV(tp)
    unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
    unsigned int work_done;
    unsigned long flags;

    work_done = rtl8101_rx_interrupt(dev, tp, ioaddr, (u32) budget);
    spin_lock_irqsave(&tp->lock, flags);
    rtl8101_tx_interrupt(dev, tp, ioaddr);
    spin_unlock_irqrestore(&tp->lock, flags);

    RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);

    if (work_done < work_to_do) {
        RTL_NETIF_RX_COMPLETE(dev, napi);
        /*
         * 20040426: the barrier is not strictly required but the
         * behavior of the irq handler could be less predictable
         * without it. Btw, the lack of flush for the posted pci
         * write is safe - FR
         */
        smp_wmb();
        rtl8101_switch_to_timer_interrupt(tp, ioaddr);
    }

    return RTL_NAPI_RETURN_VALUE;
}
#endif//CONFIG_R8101_NAPI

static void
rtl8101_down(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;

    rtl8101_delete_link_timer(dev, &tp->link_timer);

    rtl8101_delete_esd_timer(dev, &tp->esd_timer);

#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
    napi_disable(&tp->napi);
#endif
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0))
    netif_poll_disable(dev);
#endif
#endif

    netif_stop_queue(dev);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
    /* Give a racing hard_start_xmit a few cycles to complete. */
    synchronize_sched();  /* FIXME: should this be synchronize_irq()? */
#endif

    netif_carrier_off(dev);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8101_dsm(dev, DSM_IF_DOWN);

    rtl8101_hw_reset(dev);

    spin_unlock_irqrestore(&tp->lock, flags);

    synchronize_irq(dev->irq);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8101_tx_clear(tp);

    rtl8101_rx_clear(tp);

    spin_unlock_irqrestore(&tp->lock, flags);
}

static int rtl8101_close(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    void __iomem *ioaddr = tp->mmio_addr;
    u8 options;

    options = ReadMMIO8(Config1);
    if (((mcfg == CFG_METHOD_4) || (mcfg == CFG_METHOD_5)) &&
        !(options & PMEnable)) {
        WriteMMIO8(Config4, ReadMMIO8(Config4) | (1 << 0));
        WriteMMIO8(DBG_reg, ReadMMIO8(DBG_reg) | (1 << 3));
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) & !(1 << 7));
        WriteMMIO8(CPlusCmd, ReadMMIO8(CPlusCmd) | (1 << 1));
    }

    if (tp->TxDescArray!=NULL && tp->RxDescArray!=NULL) {
        rtl8101_down(dev);

        rtl8101_hw_d3_para(dev);

        rtl8101_powerdown_pll(dev);

        free_irq(dev->irq, dev);

        pci_free_consistent(pdev, R8101_RX_RING_BYTES, tp->RxDescArray, tp->RxPhyAddr);

        pci_free_consistent(pdev, R8101_TX_RING_BYTES, tp->TxDescArray, tp->TxPhyAddr);

        tp->TxDescArray = NULL;
        tp->RxDescArray = NULL;

        pci_free_consistent(pdev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);

        tp->tally_vaddr = NULL;
    }

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
static void rtl8101_shutdown(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);

    rtl8101_set_bios_setting(dev);
    rtl8101_rar_set(tp, tp->org_mac_addr);

    if (s5wol == 0)
        tp->wol_enabled = WOL_DISABLED;

    /*
    if (s5wol) {
        void __iomem *ioaddr = tp->mmio_addr;
        u32 csi_tmp;

        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        switch (mcfg) {
        case MCFG_8402_1:
        case MCFG_8168E_VL_2:
            csi_tmp = ReadERI(0xDE, 1, ERIAR_ExGMAC);
            csi_tmp |= BIT_0;
            WriteERI(0xDE, 1, csi_tmp, ERIAR_ExGMAC);
            break;
        default:
            WriteMMIO8(Config3, ReadMMIO8(Config3) | MagicPacket);
            break;
        }
        WriteMMIO8(Config5, ReadMMIO8(Config5) | LanWake);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);

        tp->wol_enabled = WOL_ENABLED;
    }
    */

    rtl8101_close(dev);
    rtl8101_disable_msi(pdev, tp);
}
#endif

/**
 *  rtl8101_get_stats - Get rtl8101 read/write statistics
 *  @dev: The Ethernet Device to get statistics for
 *
 *  Get TX/RX statistics for rtl8101
 */
static struct net_device_stats *
rtl8101_get_stats(struct net_device *dev) {
    struct rtl8101_private *tp = netdev_priv(dev);
//	void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    if (netif_running(dev)) {
        spin_lock_irqsave(&tp->lock, flags);
//		tp->stats.rx_missed_errors += ReadMMIO32(RxMissed);
//		WriteMMIO32(RxMissed, 0);
        spin_unlock_irqrestore(&tp->lock, flags);
    }

    return &RTLDEV->stats;
}

#ifdef CONFIG_PM

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int
rtl8101_suspend(struct pci_dev *pdev,
                u32 state)
#else
static int
rtl8101_suspend(struct pci_dev *pdev,
                pm_message_t state)
#endif
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    u32 pci_pm_state = pci_choose_state(pdev, state);
#endif
    unsigned long flags;


    if (!netif_running(dev))
        goto out;

    rtl8101_delete_esd_timer(dev, &tp->esd_timer);

    rtl8101_delete_link_timer(dev, &tp->link_timer);

    netif_stop_queue(dev);

    netif_carrier_off(dev);

    rtl8101_dsm(dev, DSM_NIC_GOTO_D3);

    netif_device_detach(dev);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8101_hw_reset(dev);

    rtl8101_hw_d3_para(dev);

    rtl8101_powerdown_pll(dev);

    spin_unlock_irqrestore(&tp->lock, flags);

out:

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    pci_save_state(pdev, &pci_pm_state);
#else
    pci_save_state(pdev);
#endif
    pci_enable_wake(pdev, pci_choose_state(pdev, state), tp->wol_enabled);
    pci_set_power_state(pdev, pci_choose_state(pdev, state));

    return 0;
}

static int
rtl8101_resume(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    u32 pci_pm_state = PCI_D0;
#endif

    pci_set_power_state(pdev, PCI_D0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    pci_restore_state(pdev, &pci_pm_state);
#else
    pci_restore_state(pdev);
#endif
    pci_enable_wake(pdev, PCI_D0, 0);

    /* restore last modified mac address */
    rtl8101_rar_set(tp, dev->dev_addr);

    if (!netif_running(dev))
        goto out;

    rtl8101_exit_oob(dev);

    rtl8101_dsm(dev, DSM_NIC_RESUME_D3);

    rtl8101_hw_init(dev);

    rtl8101_powerup_pll(dev);

    rtl8101_hw_ephy_config(dev);

    rtl8101_hw_phy_config(dev);

    rtl8101_schedule_work(dev, rtl8101_reset_task);

    netif_device_attach(dev);

out:
    return 0;
}

#endif /* CONFIG_PM */

static struct pci_driver rtl8101_pci_driver = {
    .name		= MODULENAME,
    .id_table	= rtl8101_pci_tbl,
    .probe		= rtl8101_init_one,
    .remove		= __devexit_p(rtl8101_remove_one),
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
    .shutdown	= rtl8101_shutdown,
#endif
#ifdef CONFIG_PM
    .suspend	= rtl8101_suspend,
    .resume		= rtl8101_resume,
#endif
};

static int __init
rtl8101_init_module(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    return pci_register_driver(&rtl8101_pci_driver);
#else
    return pci_module_init(&rtl8101_pci_driver);
#endif
}

static void __exit
rtl8101_cleanup_module(void)
{
    pci_unregister_driver(&rtl8101_pci_driver);
}

module_init(rtl8101_init_module);
module_exit(rtl8101_cleanup_module);
