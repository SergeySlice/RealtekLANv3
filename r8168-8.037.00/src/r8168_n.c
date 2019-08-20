/*
################################################################################
#
# r8168 is the Linux device driver released for Realtek Gigabit Ethernet
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
 * This driver is modified from r8169.c in Linux kernel 2.6.18
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
#include <linux/pci-aspm.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define dev_printk(A,B,fmt,args...) printk(A fmt,##args)
#else
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "r8168.h"
#include "r8168_asf.h"
#include "rtl_eeprom.h"
#include "rtltool.h"

static int eee_enable = 0 ;
module_param(eee_enable, int, S_IRUGO);

/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static const int max_interrupt_work = 20;

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
   The RTL chips use a 64 element hash table based on the Ethernet CRC. */
static const int multicast_filter_limit = 32;

#define _R(NAME,MAC,RCR,MASK, JumFrameSz) \
    { .name = NAME, .mcfg = MAC, .RCR_Cfg = RCR, .RxConfigMask = MASK, .jumbo_frame_sz = JumFrameSz }

static const struct {
    const char *name;
    u8 mcfg;
    u32 RCR_Cfg;
    u32 RxConfigMask;   /* Clears the bits supported by this chip */
    u32 jumbo_frame_sz;
} rtl_chip_info[] = {
    _R("RTL8168B/8111B",
    MCFG_8168B_1,
    (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_4k),

    _R("RTL8168B/8111B",
    MCFG_8168B_2,
    (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_4k),

    _R("RTL8168B/8111B",
    MCFG_8168B_3,
    (Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_4k),

    _R("RTL8168C/8111C",
    MCFG_8168C_1, RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_6k),

    _R("RTL8168C/8111C",
    MCFG_8168C_2,
    RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_6k),

    _R("RTL8168C/8111C",
    MCFG_8168C_3,
    RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_6k),

    _R("RTL8168CP/8111CP",
    MCFG_8168CP_1,
    RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_6k),

    _R("RTL8168CP/8111CP",
    MCFG_8168CP_2,
    RxCfg_128_int_en | RxCfg_fet_multi_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_6k),

    _R("RTL8168D/8111D",
    MCFG_8168D_1,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168D/8111D",
    MCFG_8168D_2,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168DP/8111DP",
    MCFG_8168DP_1,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168DP/8111DP",
    MCFG_8168DP_2,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168DP/8111DP",
    MCFG_8168DP_3,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168E/8111E",
    MCFG_8168E_1,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168E/8111E",
    MCFG_8168E_2,
    RxCfg_128_int_en | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168E-VL/8111E-VL",
    MCFG_8168E_VL_1,
    RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e0080,
    Jumbo_Frame_9k),

    _R("RTL8168E-VL/8111E-VL",
    MCFG_8168E_VL_2,
    RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168F/8111F",
    MCFG_8168F_1,
    RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168F/8111F",
    MCFG_8168F_2,
    RxEarly_off_V1 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8411",
    MCFG_8411_1,
    (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168G/8111G",
    CFG_METHOD_21,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168G/8111G",
    CFG_METHOD_22,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168EP/8111EP",
    CFG_METHOD_23,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168GU/8111GU",
    CFG_METHOD_24,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168GU/8111GU",
    CFG_METHOD_25,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("8411B",
    MCFG_8411B,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("RTL8168EP/8111EP",
    CFG_METHOD_27,
    RxEarly_off_V2 | (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    Jumbo_Frame_9k),

    _R("Unknown",
    CFG_METHOD_DEFAULT,
    (RX_DMA_BURST << RxCfgDMAShift),
    0xff7e1880,
    RX_BUF_SIZE)
};
#undef _R

#ifndef PCI_VENDOR_ID_DLINK
#define PCI_VENDOR_ID_DLINK 0x1186
#endif

static struct pci_device_id rtl8168_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_REALTEK, 0x8168), },
    { PCI_VENDOR_ID_DLINK, 0x4300, 0x1186, 0x4b10,},
    {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8168_pci_tbl);

static int rx_copybreak = 200;
static int timer_count = 0x2600;
static int use_dac;
static struct {
    u32 msg_enable;
} debug = { -1 };

static unsigned short speed = SPEED_1000;
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

MODULE_AUTHOR("Realtek and the Linux r8168 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("RealTek RTL-8168 Gigabit Ethernet driver");

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
module_param(use_dac, int, 0);
MODULE_PARM_DESC(use_dac, "Enable PCI DAC. Unsafe on 32 bit PCI slot.");

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

MODULE_LICENSE("GPL");

MODULE_VERSION(RTL8168_VERSION);

static void rtl8168_sleep_rx_enable(struct net_device *dev);
static void rtl8168_dsm(struct net_device *dev, int dev_state);

static void rtl8168_esd_timer(unsigned long __opaque);
static void rtl8168_link_timer(unsigned long __opaque);
static void rtl8168_tx_clear(struct rtl8168_private *tp);
static void rtl8168_rx_clear(struct rtl8168_private *tp);

static int rtl8168_open(struct net_device *dev);
static int rtl8168_start_xmit(struct sk_buff *skb, struct net_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance, struct pt_regs *regs);
#else
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance);
#endif
static void rtl8168_rx_desc_offset0_init(struct rtl8168_private *, int);
static int rtl8168_init_ring(struct net_device *dev);
static void rtl8168_hw_start(struct net_device *dev);
static int rtl8168_close(struct net_device *dev);
static void rtl8168_set_rx_mode(struct net_device *dev);
static void rtl8168_tx_timeout(struct net_device *dev);
static struct net_device_stats *rtl8168_get_stats(struct net_device *dev);
static int rtl8168_rx_interrupt(struct net_device *, struct rtl8168_private *, void __iomem *, u32 budget);
static int rtl8168_change_mtu(struct net_device *dev, int new_mtu);
static void rtl8168_down(struct net_device *dev);

static int rtl8168_set_mac_address(struct net_device *dev, void *p);
void rtl8168_rar_set(struct rtl8168_private *tp, uint8_t *addr);
static void rtl8168_desc_addr_fill(struct rtl8168_private *);
static void rtl8168_tx_desc_init(struct rtl8168_private *tp);
static void rtl8168_rx_desc_init(struct rtl8168_private *tp);

static void rtl8168_hw_reset(struct net_device *dev);

static void rtl8168_phy_power_up (struct net_device *dev);
static void rtl8168_phy_power_down (struct net_device *dev);
static int rtl8168_set_speed(struct net_device *dev, u8 autoneg,  u16 speed, u8 duplex);

#ifdef CONFIG_R8168_NAPI
static int rtl8168_poll(napi_ptr napi, napi_budget budget);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#undef ethtool_ops
#define ethtool_ops _kc_ethtool_ops

struct _kc_ethtool_ops {
    int  (*get_settings)(struct net_device *, struct ethtool_cmd *);
    int  (*set_settings)(struct net_device *, struct ethtool_cmd *);
    void (*get_drvinfo)(struct net_device *, struct ethtool_drvinfo *);
    int  (*get_regs_len)(struct net_device *);
    void (*get_regs)(struct net_device *, struct ethtool_regs *, void *);
    void (*get_wol)(struct net_device *, struct ethtool_wolinfo *);
    int  (*set_wol)(struct net_device *, struct ethtool_wolinfo *);
    u32  (*get_msglevel)(struct net_device *);
    void (*set_msglevel)(struct net_device *, u32);
    int  (*nway_reset)(struct net_device *);
    u32  (*get_link)(struct net_device *);
    int  (*get_eeprom_len)(struct net_device *);
    int  (*get_eeprom)(struct net_device *, struct ethtool_eeprom *, u8 *);
    int  (*set_eeprom)(struct net_device *, struct ethtool_eeprom *, u8 *);
    int  (*get_coalesce)(struct net_device *, struct ethtool_coalesce *);
    int  (*set_coalesce)(struct net_device *, struct ethtool_coalesce *);
    void (*get_ringparam)(struct net_device *, struct ethtool_ringparam *);
    int  (*set_ringparam)(struct net_device *, struct ethtool_ringparam *);
    void (*get_pauseparam)(struct net_device *,
                           struct ethtool_pauseparam*);
    int  (*set_pauseparam)(struct net_device *,
                           struct ethtool_pauseparam*);
    u32  (*get_rx_csum)(struct net_device *);
    int  (*set_rx_csum)(struct net_device *, u32);
    u32  (*get_tx_csum)(struct net_device *);
    int  (*set_tx_csum)(struct net_device *, u32);
    u32  (*get_sg)(struct net_device *);
    int  (*set_sg)(struct net_device *, u32);
    u32  (*get_tso)(struct net_device *);
    int  (*set_tso)(struct net_device *, u32);
    int  (*self_test_count)(struct net_device *);
    void (*self_test)(struct net_device *, struct ethtool_test *, u64 *);
    void (*get_strings)(struct net_device *, u32 stringset, u8 *);
    int  (*phys_id)(struct net_device *, u32);
    int  (*get_stats_count)(struct net_device *);
    void (*get_ethtool_stats)(struct net_device *, struct ethtool_stats *,
                              u64 *);
} *ethtool_ops = NULL;

#undef SET_ETHTOOL_OPS
#define SET_ETHTOOL_OPS(netdev, ops) (ethtool_ops = (ops))

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)


//#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)
#ifndef netif_msg_init
#define netif_msg_init _kc_netif_msg_init
/* copied from linux kernel 2.6.20 include/linux/netdevice.h */
static inline u32 netif_msg_init(int debug_value, int default_msg_enable_bits)
{
    /* use default */
    if (debug_value < 0 || debug_value >= (sizeof(u32) * 8))
        return default_msg_enable_bits;
    if (debug_value == 0)   /* no output */
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
#define MSEC_PER_SEC    1000L

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
static inline unsigned int _kc_jiffies_to_msecs(const unsigned long j)
{
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
    return (MSEC_PER_SEC / HZ) * j;
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
    return (j + (HZ / MSEC_PER_SEC) - 1)/(HZ / MSEC_PER_SEC);
#else
    return (j * MSEC_PER_SEC) / HZ;
#endif
}

static inline unsigned long _kc_msecs_to_jiffies(const unsigned int m)
{
    if (m > _kc_jiffies_to_msecs(MAX_JIFFY_OFFSET))
        return MAX_JIFFY_OFFSET;
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
    return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
    return m * (HZ / MSEC_PER_SEC);
#else
    return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
#endif
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)

/* copied from linux kernel 2.6.12.6 /include/linux/pm.h */
typedef int __bitwise pci_power_t;

/* copied from linux kernel 2.6.12.6 /include/linux/pci.h */
typedef u32 __bitwise pm_message_t;

#define PCI_D0  ((pci_power_t __force) 0)
#define PCI_D1  ((pci_power_t __force) 1)
#define PCI_D2  ((pci_power_t __force) 2)
#define PCI_D3hot   ((pci_power_t __force) 3)
#define PCI_D3cold  ((pci_power_t __force) 4)
#define PCI_POWER_ERROR ((pci_power_t __force) -1)

/* copied from linux kernel 2.6.12.6 /drivers/pci/pci.c */
/**
 * pci_choose_state - Choose the power state of a PCI device
 * @dev: PCI device to be suspended
 * @state: target sleep state for the whole system. This is the value
 *  that is passed to suspend() function.
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
//      BUG();
    }
    return PCI_D0;
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
/**
 * msleep_interruptible - sleep waiting for waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
#define msleep_interruptible _kc_msleep_interruptible
unsigned long _kc_msleep_interruptible(unsigned int msecs)
{
    unsigned long timeout = _kc_msecs_to_jiffies(msecs);

    while (timeout && !signal_pending(current)) {
        set_current_state(TASK_INTERRUPTIBLE);
        timeout = schedule_timeout(timeout);
    }
    return _kc_jiffies_to_msecs(timeout);
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
/* copied from linux kernel 2.6.20 include/linux/sched.h */
#ifndef __sched
#define __sched     __attribute__((__section__(".sched.text")))
#endif

/* copied from linux kernel 2.6.20 kernel/timer.c */
signed long __sched schedule_timeout_uninterruptible(signed long timeout)
{
    __set_current_state(TASK_UNINTERRUPTIBLE);
    return schedule_timeout(timeout);
}

/* copied from linux kernel 2.6.20 include/linux/mii.h */
#undef if_mii
#define if_mii _kc_if_mii
static inline struct mii_ioctl_data *if_mii(struct ifreq *rq) {
    return (struct mii_ioctl_data *) &rq->ifr_ifru;
}
#endif  //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)

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

void mdio_write(struct rtl8168_private *tp,
                u32 RegAddr,
                u32 value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    if (RegAddr == 0x1F) {
        tp->cur_page = value;
    }

    if (mcfg == MCFG_8168DP_1) {
        WriteMMIO32(OCPDR, OCPDR_Write |
                (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift |
                (value & OCPDR_Data_Mask));
        WriteMMIO32(OCPAR, OCPAR_GPHY_Write);
        WriteMMIO32(EPHY_RXER_NUM, 0);

        for (i = 0; i < 100; i++) {
            mdelay(1);
            if (!(ReadMMIO32(OCPAR) & OCPAR_Flag))
                break;
        }
    } else if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
               mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_24 ||
               mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B ||
               mcfg == CFG_METHOD_27) {
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
        if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
            WriteMMIO32(0xD0, ReadMMIO32(0xD0) & ~0x00020000);

        WriteMMIO32(PHYAR, PHYAR_Write |
                (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift |
                (value & PHYAR_Data_Mask));

        for (i = 0; i < 10; i++) {
            IODelay(100);

            /* Check if the RTL8168 has completed writing to the specified MII register */
            if (!(ReadMMIO32(PHYAR) & PHYAR_Flag)) {
                IODelay(20);
                break;
            }
        }

        if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
            WriteMMIO32(0xD0, ReadMMIO32(0xD0) | 0x00020000);
    }
}

u32 mdio_read(struct rtl8168_private *tp,
              u32 RegAddr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i, value = 0;

    if (mcfg==MCFG_8168DP_1) {
        WriteMMIO32(OCPDR, OCPDR_Read |
                (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift);
        WriteMMIO32(OCPAR, OCPAR_GPHY_Write);
        WriteMMIO32(EPHY_RXER_NUM, 0);

        for (i = 0; i < 100; i++) {
            mdelay(1);
            if (!(ReadMMIO32(OCPAR) & OCPAR_Flag))
                break;
        }

        mdelay(1);
        WriteMMIO32(OCPAR, OCPAR_GPHY_Read);
        WriteMMIO32(EPHY_RXER_NUM, 0);

        for (i = 0; i < 100; i++) {
            mdelay(1);
            if (ReadMMIO32(OCPAR) & OCPAR_Flag)
                break;
        }

        value = ReadMMIO32(OCPDR) & OCPDR_Data_Mask;
    } else if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
               mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_24 ||
               mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B ||
               mcfg == CFG_METHOD_27) {
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
        if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
            WriteMMIO32(0xD0, ReadMMIO32(0xD0) & ~0x00020000);

        WriteMMIO32(PHYAR,
                PHYAR_Read | (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift);

        for (i = 0; i < 10; i++) {
            IODelay(100);

            /* Check if the RTL8168 has completed retrieving data from the specified MII register */
            if (ReadMMIO32(PHYAR) & PHYAR_Flag) {
                value = ReadMMIO32(PHYAR) & PHYAR_Data_Mask;
                IODelay(20);
                break;
            }
        }

        if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3)
            WriteMMIO32(0xD0, ReadMMIO32(0xD0) | 0x00020000);
    }

    return value;
}

void mac_ocp_write(struct rtl8168_private *tp, u16 reg_addr, u16 value)
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

u16 mac_ocp_read(struct rtl8168_private *tp, u16 reg_addr)
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

u32 OCP_read(struct rtl8168_private *tp, u8 mask, u16 Reg)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    if (mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_27) {
        WriteMMIO32(ERIAR, ERIAR_Read |
                ERIAR_OOB << ERIAR_Type_shift |
                mask << ERIAR_ByteEn_shift |
                Reg);

        for (i = 0; i < 10; i++) {
            IODelay(100);
            /* Check if the RTL8168 has completed ERI read */
            if (ReadMMIO32(ERIAR) & ERIAR_Flag)
                break;
        }

        return ReadMMIO32(ERIDR);
    } else {
        WriteMMIO32(OCPAR, ((u32)mask&0xF)<<12 | (Reg&0xFFF));
        for (i = 0; i < 20; i++) {
            IODelay(100);
            if (ReadMMIO32(OCPAR) & OCPAR_Flag)
                break;
        }
        return ReadMMIO32(OCPDR);
    }
}

void OCP_write(struct rtl8168_private *tp, u8 mask, u16 Reg, u32 data)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    if (mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_27) {
        WriteMMIO32(ERIDR, data);
        WriteMMIO32(ERIAR, ERIAR_Write |
                ERIAR_OOB << ERIAR_Type_shift |
                mask << ERIAR_ByteEn_shift |
                Reg);

        for (i = 0; i < 10; i++) {
            IODelay(100);
            /* Check if the RTL8168 has completed ERI write */
            if (!(ReadMMIO32(ERIAR) & ERIAR_Flag))
                break;
        }
    } else {
        WriteMMIO32(OCPDR, data);
        WriteMMIO32(OCPAR, OCPAR_Flag | ((u32)mask&0xF)<<12 | (Reg&0xFFF));
        for (i = 0; i < 20; i++) {
            IODelay(100);
            if ((ReadMMIO32(OCPAR)&OCPAR_Flag) == 0)
                break;
        }
    }
}

void OOB_mutex_lock(struct rtl8168_private *tp)
{
    u8 reg_16, reg_a0;
    u32 wait_cnt_0, wait_Cnt_1;
    u16 ocp_reg_mutex_ib;
    u16 ocp_reg_mutex_oob;
    u16 ocp_reg_mutex_prio;

    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
        ocp_reg_mutex_oob = 0x16;
        ocp_reg_mutex_ib = 0x17;
        ocp_reg_mutex_prio = 0x9C;
        break;
    case MCFG_8168DP_3:
        ocp_reg_mutex_oob = 0x06;
        ocp_reg_mutex_ib = 0x07;
        ocp_reg_mutex_prio = 0x9C;
        break;
    case CFG_METHOD_23:
    case CFG_METHOD_27:
    default:
        ocp_reg_mutex_oob = 0x110;
        ocp_reg_mutex_ib = 0x114;
        ocp_reg_mutex_prio = 0x11C;
        break;
    }

    OCP_write(tp, 0x1, ocp_reg_mutex_ib, BIT_0);
    reg_16 = OCP_read(tp, 0xF, ocp_reg_mutex_oob);
    wait_cnt_0 = 0;
    while(reg_16) {
        reg_a0 = OCP_read(tp, 0xF, ocp_reg_mutex_prio);
        if(reg_a0) {
            OCP_write(tp, 0x1, ocp_reg_mutex_ib, 0x00);
            reg_a0 = OCP_read(tp, 0xF, ocp_reg_mutex_prio);
            wait_Cnt_1 = 0;
            while(reg_a0) {
                reg_a0 = OCP_read(tp, 0xF, ocp_reg_mutex_prio);

                wait_Cnt_1++;

                if(wait_Cnt_1 > 2000)
                    break;
            };
            OCP_write(tp, 0x1, ocp_reg_mutex_ib, BIT_0);

        }
        reg_16 = OCP_read(tp, 0xF, ocp_reg_mutex_oob);

        wait_cnt_0++;

        if(wait_cnt_0 > 2000)
            break;
    };
}

void OOB_mutex_unlock(struct rtl8168_private *tp)
{
    u16 ocp_reg_mutex_ib;
    u16 ocp_reg_mutex_oob;
    u16 ocp_reg_mutex_prio;

    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
        ocp_reg_mutex_oob = 0x16;
        ocp_reg_mutex_ib = 0x17;
        ocp_reg_mutex_prio = 0x9C;
        break;
    case MCFG_8168DP_3:
        ocp_reg_mutex_oob = 0x06;
        ocp_reg_mutex_ib = 0x07;
        ocp_reg_mutex_prio = 0x9C;
        break;
    case CFG_METHOD_23:
    case CFG_METHOD_27:
    default:
        ocp_reg_mutex_oob = 0x110;
        ocp_reg_mutex_ib = 0x114;
        ocp_reg_mutex_prio = 0x11C;
        break;
    }

    OCP_write(tp, 0x1, ocp_reg_mutex_prio, BIT_0);
    OCP_write(tp, 0x1, ocp_reg_mutex_ib, 0x00);
}

void OOB_notify(struct rtl8168_private *tp, u8 cmd)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    WriteMMIO8(ERIDR, cmd);
    WriteMMIO32(ERIAR, 0x800010E8);
    mdelay(2);
    for (i = 0; i < 5; i++) {
        IODelay(100);
        if (!(ReadMMIO32(ERIAR) & ERIAR_Flag))
            break;
    }

    OCP_write(tp, 0x1, 0x30, 0x00000001);
}

static int rtl8168_check_dash(struct rtl8168_private *tp)
{
    if (mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_27) {
        if (OCP_read(tp, 0xF, 0x128) & BIT_0)
            return 1;
        else
            return 0;
    } else {
        u32 reg;

        if (mcfg == MCFG_8168DP_3)
            reg = 0xb8;
        else
            reg = 0x10;

        if (OCP_read(tp, 0xF, reg) & 0x00008000)
            return 1;
        else
            return 0;
    }
}

static void rtl8168_driver_start(struct rtl8168_private *tp)
{
    if (mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_27) {
        u32 tmp_value;

        if (!rtl8168_check_dash(tp))
            return;

        OCP_write(tp, 0x1, 0x180, OOB_CMD_DRIVER_START);
        tmp_value = OCP_read(tp, 0xF, 0x30);
        tmp_value |= BIT_0;
        OCP_write(tp, 0x1, 0x30, tmp_value);
    } else {
        int timeout;
        u32 reg;

        OOB_notify(tp, OOB_CMD_DRIVER_START);

        if (mcfg == MCFG_8168DP_3)
            reg = 0xB8;
        else
            reg = 0x10;

        for (timeout = 0; timeout < 10; timeout++) {
            mdelay(10);
            if (OCP_read(tp, 0xF, reg) & BIT_11)
                break;
        }
    }
}

static void rtl8168_driver_stop(struct rtl8168_private *tp)
{
    if (mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_27) {
        u32 tmp_value;

        if (!rtl8168_check_dash(tp))
            return;

        OCP_write(tp, 0x1, 0x180, OOB_CMD_DRIVER_STOP);
        tmp_value = OCP_read(tp, 0xF, 0x30);
        tmp_value |= BIT_0;
        OCP_write(tp, 0x1, 0x30, tmp_value);
    } else {
        int timeout;
        u32 reg;

        OOB_notify(tp, OOB_CMD_DRIVER_STOP);

        if (mcfg == MCFG_8168DP_3)
            reg = 0xB8;
        else
            reg = 0x10;

        for (timeout = 0; timeout < 10; timeout++) {
            mdelay(10);
            if ((OCP_read(tp, 0xF, reg) & BIT_11) == 0)
                break;
        }
    }
}

void rtl8168_ephy_write(void __iomem *ioaddr, int RegAddr, int value)
{
    int i;

    WriteMMIO32(EPHYAR,
            EPHYAR_Write |
            (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift |
            (value & EPHYAR_Data_Mask));

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8168 has completed EPHY write */
        if (!(ReadMMIO32(EPHYAR) & EPHYAR_Flag))
            break;
    }

    IODelay(20);
}

u16 rtl8168_ephy_read(void __iomem *ioaddr, int RegAddr)
{
    int i;
    u16 value = 0xffff;

    WriteMMIO32(EPHYAR,
            EPHYAR_Read | (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift);

    for (i = 0; i < 10; i++) {
        IODelay(100);

        /* Check if the RTL8168 has completed EPHY read */
        if (ReadMMIO32(EPHYAR) & EPHYAR_Flag) {
            value = (u16) (ReadMMIO32(EPHYAR) & EPHYAR_Data_Mask);
            break;
        }
    }

    IODelay(20);

    return value;
}

static u32
rtl8168_csi_other_fun_read(struct rtl8168_private *tp,
                           u8 multi_fun_sel_bit,
                           u32 addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
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

static void
rtl8168_csi_other_fun_write(struct rtl8168_private *tp,
                            u8 multi_fun_sel_bit,
                            u32 addr,
                            u32 value)
{
    void __iomem *ioaddr = tp->mmio_addr;
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

static u32
rtl8168_csi_read(struct rtl8168_private *tp,
                 u32 addr)
{
    u8 multi_fun_sel_bit;

    if (mcfg == MCFG_8411_1)
        multi_fun_sel_bit = 2;
    else if (mcfg == MCFG_8411B)
        multi_fun_sel_bit = 1;
    else
        multi_fun_sel_bit = 0;


    return rtl8168_csi_other_fun_read(tp, multi_fun_sel_bit, addr);
}

static void
rtl8168_csi_write(struct rtl8168_private *tp,
                  u32 addr,
                  u32 value)
{
    u8 multi_fun_sel_bit;

    if (mcfg == MCFG_8411_1)
        multi_fun_sel_bit = 2;
    else if (mcfg == MCFG_8411B)
        multi_fun_sel_bit = 1;
    else
        multi_fun_sel_bit = 0;

    rtl8168_csi_other_fun_write(tp, multi_fun_sel_bit, addr, value);
}

static u8
rtl8168_csi_fun0_read_byte(struct rtl8168_private *tp,
                           u32 addr)
{
    u8 RetVal = 0;

    if (mcfg == MCFG_8411_1 || mcfg == MCFG_8411B) {
        u32 TmpUlong;
        u16 RegAlignAddr;
        u8 ShiftByte;

        RegAlignAddr = addr & ~(0x3);
        ShiftByte = addr & (0x3);
        TmpUlong = rtl8168_csi_other_fun_read(tp, 0, addr);
        TmpUlong >>= (8*ShiftByte);
        RetVal = (u8)TmpUlong;
    } else {
        struct pci_dev *pdev = tp->pci_dev;

        pci_read_config_byte(pdev, addr, &RetVal);
    }

    return RetVal;
}

static void
rtl8168_csi_fun0_write_byte(struct rtl8168_private *tp,
                            u32 addr,
                            u8 value)
{
    if (mcfg == MCFG_8411_1 || mcfg == MCFG_8411B) {
        u32 TmpUlong;
        u16 RegAlignAddr;
        u8 ShiftByte;

        RegAlignAddr = addr & ~(0x3);
        ShiftByte = addr & (0x3);
        TmpUlong = rtl8168_csi_other_fun_read(tp, 0, RegAlignAddr);
        TmpUlong &= ~(0xFF << (8*ShiftByte));
        TmpUlong |= (value << (8*ShiftByte));
        rtl8168_csi_other_fun_write( tp, 0, RegAlignAddr, TmpUlong );
    } else {
        struct pci_dev *pdev = tp->pci_dev;

        pci_write_config_byte(pdev, addr, value);
    }
}

u32 ReadERI(void __iomem *ioaddr, int addr, int len, int type)
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

            /* Check if the RTL8168 has completed ERI read */
            if (ReadMMIO32(ERIAR) & ERIAR_Flag)
                break;
        }

        if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

        value1 = ReadMMIO32(ERIDR) & mask;
        value2 |= (value1 >> val_shift * 8) << shift * 8;

        if (len <= 4 - val_shift) {
            len = 0;
        } else {
            len -= (4 - val_shift);
            shift = 4 - val_shift;
            addr += 4;
        }
    }

    return value2;
}

int WriteERI(void __iomem *ioaddr, int addr, int len, u32 value, int type)
{

    int i, val_shift, shift = 0;
    u32 value1 = 0, mask;

    if (len > 4 || len <= 0)
        return -1;

    while (len > 0) {
        val_shift = addr % ERIAR_Addr_Align;
        addr = addr & ~0x3;

        if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

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

            /* Check if the RTL8168 has completed ERI write */
            if (!(ReadMMIO32(ERIAR) & ERIAR_Flag))
                break;
        }

        if (len <= 4 - val_shift) {
            len = 0;
        } else {
            len -= (4 - val_shift);
            shift = 4 - val_shift;
            addr += 4;
        }
    }

    return 0;
}

static void
rtl8168_enable_rxdvgate(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_3);
        mdelay(2);
        break;
    }
}

static void
rtl8168_disable_rxdvgate(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) & ~BIT_3);
        mdelay(2);
        break;
    }
}

static void
rtl8168_wait_txrx_fifo_empty(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        for (i = 0; i < 10; i++) {
            IODelay(100);
            if (ReadMMIO32(TxConfig) & BIT_11)
                break;
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
rtl8168_irq_mask_and_ack(void __iomem *ioaddr)
{
    WriteMMIO16(IntrMask, 0x0000);
    WriteMMIO16(IntrStatus, ReadMMIO16(IntrStatus));
}

static void
rtl8168_nic_reset(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    WriteMMIO32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));

    rtl8168_enable_rxdvgate(dev);

    rtl8168_wait_txrx_fifo_empty(dev);

    switch (mcfg) {
    case MCFG_8168B_1:
    case MCFG_8168B_2:
    case MCFG_8168B_3:
        mdelay(10);
        break;
    case MCFG_8168C_1:
    case MCFG_8168C_2:
    case MCFG_8168C_3:
    case MCFG_8168CP_1:
    case MCFG_8168CP_2:
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        WriteMMIO8(ChipCmd, StopReq | CmdRxEnb | CmdTxEnb);
        IODelay(100);
        break;
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
        while (ReadMMIO8(TxPoll) & NPQ)
            IODelay(20);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
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
        IODelay(100);
        if ((ReadMMIO8(ChipCmd) & CmdReset) == 0)
            break;
    }

    switch (mcfg) {
    case MCFG_8168DP_1:
        OOB_mutex_lock(tp);
        OCP_write(tp, 0x3, 0x10, OCP_read(tp, 0xF, 0x010)&~0x00004000);
        OOB_mutex_unlock(tp);

        OOB_notify(tp, OOB_CMD_RESET);

        for (i = 0; i < 10; i++) {
            mdelay(10);
            if (OCP_read(tp, 0xF, 0x010)&0x00004000)
                break;
        }

        for (i = 0; i < 5; i++) {
            if ( (OCP_read(tp, 0xF, 0x034) & 0xFFFF) == 0)
                break;
        }
        break;
    }
}

static void
rtl8168_hw_reset(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    /* Disable interrupts */
    rtl8168_irq_mask_and_ack(ioaddr);

    rtl8168_nic_reset(dev);
}

static void rtl8168_mac_loopback_test(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    struct net_device *dev = tp->dev;
    struct sk_buff *skb, *rx_skb;
    dma_addr_t mapping;
    struct TxDesc *txd;
    struct RxDesc *rxd;
    void *tmpAddr;
    u32 len, rx_len, rx_cmd;
    u16 type;
    u8 pattern;
    int i;

    if (rtl8168_check_dash(tp))
        return;

    pattern = 0x5A;
    len = 60;
    type = htons(ETH_P_IP);
    txd = tp->TxDescArray;
    rxd = tp->RxDescArray;
    rx_skb = tp->Rx_skbuff[0];
    WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) & ~0x00060000) | 0x00020000);

    do {
        skb = dev_alloc_skb(len + RTK_RX_ALIGN);
        if (unlikely(!skb))
            dev_printk(KERN_NOTICE, &tp->pci_dev->dev, "-ENOMEM;\n");
    } while (unlikely(skb == NULL));
    skb_reserve(skb, RTK_RX_ALIGN);

    memcpy(skb_put(skb, dev->addr_len), dev->dev_addr, dev->addr_len);
    memcpy(skb_put(skb, dev->addr_len), dev->dev_addr, dev->addr_len);
    memcpy(skb_put(skb, sizeof(type)), &type, sizeof(type));
    tmpAddr = skb_put(skb, len - 14);

    mapping = pci_map_single(tp->pci_dev, skb->data, len, PCI_DMA_TODEVICE);
    pci_dma_sync_single_for_cpu(tp->pci_dev, le64_to_cpu(mapping),
                                len, PCI_DMA_TODEVICE);
    txd->addr = cpu_to_le64(mapping);
    txd->opts2 = 0;
    while (1) {
        memset(tmpAddr, pattern++, len - 14);
        pci_dma_sync_single_for_device(tp->pci_dev,
                                       le64_to_cpu(mapping),
                                       len, PCI_DMA_TODEVICE);
        txd->opts1 = cpu_to_le32(DescOwn | FirstFrag | LastFrag | len);

        WriteMMIO32(RxConfig, ReadMMIO32(RxConfig)  | AcceptMyPhys);

        smp_wmb();
        WriteMMIO8(TxPoll, NPQ);    /* set polling bit */

        for (i = 0; i < 50; i++) {
            IODelay(200);
            rx_cmd = le32_to_cpu(rxd->opts1);
            if ((rx_cmd & DescOwn) == 0)
                break;
        }

        WriteMMIO32(RxConfig, ReadMMIO32(RxConfig) & ~(AcceptErr | AcceptRunt | AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));

        rx_len = rx_cmd & 0x3FFF;
        rx_len -= 4;
        rxd->opts1 = cpu_to_le32(DescOwn | tp->rx_buf_sz);

        pci_dma_sync_single_for_cpu(tp->pci_dev, le64_to_cpu(mapping), len, PCI_DMA_TODEVICE);

        if (rx_len == len) {
            pci_dma_sync_single_for_cpu(tp->pci_dev, le64_to_cpu(rxd->addr), tp->rx_buf_sz, PCI_DMA_FROMDEVICE);
            i = memcmp(skb->data, rx_skb->data, rx_len);
            pci_dma_sync_single_for_device(tp->pci_dev, le64_to_cpu(rxd->addr), tp->rx_buf_sz, PCI_DMA_FROMDEVICE);
            if (i == 0) {
//              dev_printk(KERN_INFO, &tp->pci_dev->dev, "loopback test finished\n",rx_len,len);
                break;
            }
        }

        rtl8168_hw_reset(dev);
        rtl8168_disable_rxdvgate(dev);
        WriteMMIO8(ChipCmd, CmdTxEnb | CmdRxEnb);
    }
    tp->dirty_tx++;
    tp->dirty_rx++;
    tp->cur_tx++;
    tp->cur_rx++;
    pci_unmap_single(tp->pci_dev, le64_to_cpu(mapping),
                     len, PCI_DMA_TODEVICE);
    WriteMMIO32(TxConfig, ReadMMIO32(TxConfig) & ~0x00060000);
    dev_kfree_skb_any(skb);
    WriteMMIO16(IntrStatus, 0xFFBF);
}

static unsigned int
rtl8168_xmii_reset_pending(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned int retval;
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1f, 0x0000);
    retval = ReadGMII16( MII_BMCR) & BMCR_RESET;
    spin_unlock_irqrestore(&tp->phy_lock, flags);

    return retval;
}

static unsigned int
rtl8168_xmii_link_ok(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned int retval;

    retval = (ReadMMIO8(PHYstatus) & LinkStatus) ? 1 : 0;

    return retval;
}

static void
rtl8168_xmii_reset_enable(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int i, val = 0;
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1f, 0x0000);
    WriteGMII16( MII_BMCR, ReadGMII16( MII_BMCR) | BMCR_RESET);
    spin_unlock_irqrestore(&tp->phy_lock, flags);

    for (i = 0; i < 2500; i++) {
        spin_lock_irqsave(&tp->phy_lock, flags);
        val = ReadGMII16( MII_BMSR) & BMCR_RESET;
        spin_unlock_irqrestore(&tp->phy_lock, flags);

        if (!val)
            return;

        mdelay(1);
    }
}

static void
rtl8168dp_10mbps_gphy_para(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 status = ReadMMIO8(PHYstatus);
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    if ((status & LinkStatus) && (status & _10bps)) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x10, 0x04EE);
    } else {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x10, 0x01EE);
    }
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

void rtl8168_init_ring_indexes(struct rtl8168_private *tp)
{
    tp->dirty_tx = 0;
    tp->dirty_rx = 0;
    tp->cur_tx = 0;
    tp->cur_rx = 0;
}

static void
rtl8168_issue_offset_99_event(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case CFG_METHOD_27:
        if (mcfg == CFG_METHOD_24 || mcfg == CFG_METHOD_25 ||
            mcfg == CFG_METHOD_27) {
            WriteERI(0x3FC, 4, 0x00000000, ERIAR_ExGMAC);
        } else {
            WriteERI(0x3FC, 4, 0x083C083C, ERIAR_ExGMAC);
        }
        csi_tmp = ReadERI(0x3F8, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        WriteERI(0x3F8, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    }
}

static void
rtl8168_check_link_status(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int link_status_on;
    unsigned long flags;

    link_status_on = tp->link_ok(dev);

    if (mcfg == MCFG_8168DP_1)
        rtl8168dp_10mbps_gphy_para(dev);

    if (netif_carrier_ok(dev) != link_status_on) {
        if (link_status_on) {
            if (mcfg == MCFG_8168F_1 || mcfg == MCFG_8168F_2 || mcfg == MCFG_8411_1) {
                if (ReadMMIO8(PHYstatus) & _1000bpsF) {
                    WriteERI(0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
                    WriteERI(0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                } else {
                    WriteERI(0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                    WriteERI(0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
                }
            } else if ((mcfg == MCFG_8168E_VL_1 || mcfg == MCFG_8168E_VL_2) && netif_running(dev)) {
                if (mcfg == MCFG_8168E_VL_1 && (ReadMMIO8(PHYstatus) & _10bps)) {
                    WriteMMIO32(RxConfig, ReadMMIO32(RxConfig) | AcceptAllPhys);
                } else if (mcfg == MCFG_8168E_VL_2) {
                    if (ReadMMIO8(PHYstatus) & _1000bpsF) {
                        WriteERI(0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
                        WriteERI(0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                    } else if (ReadMMIO8(PHYstatus) & _100bps) {
                        WriteERI(0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                        WriteERI(0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                    } else {
                        WriteERI(0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                        WriteERI(0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
                    }
                }
            } else if ((mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2) && eee_enable ==1) {
                /*Full -Duplex  mode*/
                if (ReadMMIO8(PHYstatus)&FullDup) {
                    spin_lock_irqsave(&tp->phy_lock, flags);
                    WriteGMII16( 0x1F, 0x0006);
                    WriteGMII16( 0x00, 0x5a30);
                    WriteGMII16( 0x1F, 0x0000);
                    spin_unlock_irqrestore(&tp->phy_lock, flags);
                    if (ReadMMIO8(PHYstatus) & (_10bps | _100bps))
                        WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) & ~BIT_19) | BIT_25);

                } else {
                    spin_lock_irqsave(&tp->phy_lock, flags);
                    WriteGMII16( 0x1F, 0x0006);
                    WriteGMII16( 0x00, 0x5a00);
                    WriteGMII16( 0x1F, 0x0000);
                    spin_unlock_irqrestore(&tp->phy_lock, flags);
                    if (ReadMMIO8(PHYstatus) & (_10bps | _100bps))
                        WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) & ~BIT_19) | (InterFrameGap << TxInterFrameGapShift));
                }
            } else if ((mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
                        mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_24 ||
                        mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B ||
                        mcfg == CFG_METHOD_27) && netif_running(dev)) {
                if (ReadMMIO8(PHYstatus)&FullDup)
                    WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) | (BIT_24 | BIT_25)) & ~BIT_19);
                else
                    WriteMMIO32(TxConfig, (ReadMMIO32(TxConfig) | BIT_25) & ~(BIT_19 | BIT_24));
            }

            if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 || mcfg == CFG_METHOD_27) {
                /*half mode*/
                if (!(ReadMMIO8(PHYstatus)&FullDup)) {
                    spin_lock_irqsave(&tp->phy_lock, flags);
                    WriteGMII16( 0x1F, 0x0000);
                    WriteGMII16( PHY_AUTO_NEGO_REG, ReadGMII16( PHY_AUTO_NEGO_REG)&~(PHY_Cap_PAUSE|PHY_Cap_ASYM_PAUSE));
                    spin_unlock_irqrestore(&tp->phy_lock, flags);
                }
            }

            rtl8168_hw_start(dev);

            netif_carrier_on(dev);

            if (netif_msg_ifup(tp))
                printk(KERN_INFO PFX "%s: link up\n", dev->name);
        } else {
            if (netif_msg_ifdown(tp))
                printk(KERN_INFO PFX "%s: link down\n", dev->name);
            netif_carrier_off(dev);

            netif_stop_queue(dev);

            rtl8168_hw_reset(dev);

            rtl8168_tx_clear(tp);

            rtl8168_init_ring_indexes(tp);

            rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);

            switch (mcfg) {
            case CFG_METHOD_21:
            case CFG_METHOD_22:
            case CFG_METHOD_23:
            case CFG_METHOD_24:
            case CFG_METHOD_25:
            case CFG_METHOD_27:
                if (tp->org_pci_offset_99 & BIT_2)
                    tp->issue_offset_99_event = TRUE;
                break;
            }
        }
    }

    if (!link_status_on) {
        switch (mcfg) {
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case CFG_METHOD_27:
            if (tp->issue_offset_99_event) {
                if (!(ReadMMIO8(PHYstatus) & PowerSaveStatus)) {
                    tp->issue_offset_99_event = FALSE;
                    rtl8168_issue_offset_99_event(tp);
                }
            }
            break;
        }
    }
}

static void
rtl8168_link_option(int idx,
                    u8 *aut,
                    u16 *spd,
                    u8 *dup)
{
    if ((*spd != SPEED_1000) && (*spd != SPEED_100) && (*spd != SPEED_10))
        *spd = SPEED_1000;

    if ((*dup != DUPLEX_FULL) && (*dup != DUPLEX_HALF))
        *dup = DUPLEX_FULL;

    if ((*aut != AUTONEG_ENABLE) && (*aut != AUTONEG_DISABLE))
        *aut = AUTONEG_ENABLE;
}

static void
rtl8168_wait_ll_share_fifo_ready(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;

    for (i = 0; i < 10; i++) {
        IODelay(100);
        if (ReadMMIO16(0xD2) & BIT_9)
            break;
    }
}

static void
rtl8168_disable_pci_offset_99(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        csi_tmp = ReadERI(0x3F2, 2, ERIAR_ExGMAC);
        csi_tmp &= ~(BIT_0 | BIT_1);
        WriteERI(0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
        break;
    }

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case MCFG_8411B:
        rtl8168_csi_fun0_write_byte(tp, 0x99, 0x00);
        break;
    }
}

static void
rtl8168_enable_pci_offset_99(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        csi_tmp = ReadERI(0x3F2, 2, ERIAR_ExGMAC);
        csi_tmp &= ~(BIT_0 | BIT_1);
        if (!(tp->org_pci_offset_99 & (BIT_5 | BIT_6)))
            csi_tmp |= BIT_1;
        if (!(tp->org_pci_offset_99 & BIT_2))
            csi_tmp |= BIT_0;
        WriteERI(0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
        break;
    }
}

static void
rtl8168_init_pci_offset_99(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;

    switch (mcfg) {
    case MCFG_8411B:
        csi_tmp = ReadERI(0x5C2, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_1;
        WriteERI(0x5C2, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    }

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        csi_tmp = ReadERI(0x3F2, 2, ERIAR_ExGMAC);
        csi_tmp &= ~( BIT_8 | BIT_9  | BIT_10 | BIT_11  | BIT_12  | BIT_13  | BIT_14 | BIT_15 );
        csi_tmp |= ( BIT_9 | BIT_10 | BIT_13  | BIT_14 | BIT_15 );
        WriteERI(0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0x3F5, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_6 | BIT_7;
        WriteERI(0x3F5, 1, csi_tmp, ERIAR_ExGMAC);
        mac_ocp_write(0xE02C, 0x1880);
        mac_ocp_write(0xE02E, 0x4880);
        break;
    }

    switch (mcfg) {
    case MCFG_8411B:
        csi_tmp = ReadERI(0x5C8, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        WriteERI(0x5C8, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    }

    switch (mcfg) {
    case CFG_METHOD_23:
        WriteERI(0x2E8, 2, 0x883C, ERIAR_ExGMAC);
        WriteERI(0x2EA, 2, 0x8C12, ERIAR_ExGMAC);
        WriteERI(0x2EC, 2, 0x9003, ERIAR_ExGMAC);
        WriteERI(0x2E2, 2, 0x883C, ERIAR_ExGMAC);
        WriteERI(0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
        WriteERI(0x2E6, 2, 0x9003, ERIAR_ExGMAC);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteERI(0x2E8, 2, 0x9003, ERIAR_ExGMAC);
        WriteERI(0x2EA, 2, 0x9003, ERIAR_ExGMAC);
        WriteERI(0x2EC, 2, 0x9003, ERIAR_ExGMAC);
        WriteERI(0x2E2, 2, 0x883C, ERIAR_ExGMAC);
        WriteERI(0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
        WriteERI(0x2E6, 2, 0x9003, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0x3FA, 2, ERIAR_ExGMAC);
        csi_tmp |= BIT_14;
        WriteERI(0x3FA, 2, csi_tmp, ERIAR_ExGMAC);

        break;
    }

    rtl8168_enable_pci_offset_99(tp);

    switch (mcfg) {
    case MCFG_8411B:
        WriteMMIO8(0xB6, ReadMMIO8(0xB6) | BIT_0);
        break;
    }
}

static void
rtl8168_disable_pci_offset_180(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;

    switch (mcfg) {
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        csi_tmp = ReadERI(0x1E2, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_2;
        WriteERI(0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    }

    switch (mcfg) {
    case MCFG_8411B:
        WriteERI(0x1E9, 1, 0x0A, ERIAR_ExGMAC);
        break;
    }
}

static void
rtl8168_enable_pci_offset_180(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;

    switch (mcfg) {
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        csi_tmp = ReadERI(0x1E2, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_2;
        if (tp->org_pci_offset_180 & (BIT_0|BIT_1))
            csi_tmp |= BIT_2;
        WriteERI(0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    }

    switch (mcfg) {
    case MCFG_8411B:
        if (tp->org_pci_offset_180 & (BIT_0|BIT_1))
            WriteERI(0x1E9, 1, 0x64, ERIAR_ExGMAC);
        else
            WriteERI(0x1E9, 1, 0x0A, ERIAR_ExGMAC);
        break;
    }
}

static void
rtl8168_init_pci_offset_180(struct rtl8168_private *tp)
{
    rtl8168_enable_pci_offset_180(tp);
}

static void
rtl8168_set_pci_99_180_exit_driver_para(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case CFG_METHOD_27:
        rtl8168_issue_offset_99_event(tp);
        break;
    }

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        rtl8168_disable_pci_offset_99(tp);
        break;
    }
    switch (mcfg) {
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        rtl8168_disable_pci_offset_180(tp);
        break;
    }
}

static void
rtl8168_hw_d3_para(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    WriteMMIO16(RxMaxSize, RX_BUF_SIZE);

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);
        break;
    }

    if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
        mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_24 ||
        mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B ||
        mcfg == CFG_METHOD_27) {
        WriteERI(0x2F8, 2, 0x0064, ERIAR_ExGMAC);
    }

    if (bios_setting & BIT_28) {
        if (mcfg == MCFG_8168F_1 || mcfg == MCFG_8168F_2 ||
            mcfg == MCFG_8411_1) {
            u32 gphy_val;

            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1F, 0x0000);
            WriteGMII16( 0x04, 0x0061);
            WriteGMII16( 0x09, 0x0000);
            WriteGMII16( 0x00, 0x9200);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8B80);
            gphy_val = ReadGMII16( 0x06);
            gphy_val &= ~BIT_7;
            WriteGMII16( 0x06, gphy_val);
            mdelay(1);
            WriteGMII16( 0x1F, 0x0007);
            WriteGMII16( 0x1E, 0x002C);
            gphy_val = ReadGMII16( 0x16);
            gphy_val &= ~BIT_10;
            WriteGMII16( 0x16, gphy_val);
            WriteGMII16( 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
    }

    rtl8168_set_pci_99_180_exit_driver_para(dev);

    /*disable ocp phy power saving*/
    if (mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B || mcfg == CFG_METHOD_27) {
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0C41);
        WriteGMII16( 0x13, 0x0000);
        WriteGMII16( 0x13, 0x0500);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }

    rtl8168_disable_rxdvgate(dev);
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

static void
rtl8168_get_hw_wol(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 options;
    u32 csi_tmp;
    unsigned long flags;


    spin_lock_irqsave(&tp->lock, flags);

    tp->wol_opts = 0;
    options = ReadMMIO8(Config1);
    if (!(options & PMEnable))
        goto out_unlock;

    options = ReadMMIO8(Config3);
    if (options & LinkUp)
        tp->wol_opts |= WAKE_PHY;

    switch (mcfg) {
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
        csi_tmp = ReadERI(0xDE, 1, ERIAR_ExGMAC);
        if (csi_tmp & BIT_0)
            tp->wol_opts |= WAKE_MAGIC;
        break;
    default:
        if (options & MagicPacket)
            tp->wol_opts |= WAKE_MAGIC;
        break;
    }

    options = ReadMMIO8(Config5);
    if (options & UWF)
        tp->wol_opts |= WAKE_UCAST;
    if (options & BWF)
        tp->wol_opts |= WAKE_BCAST;
    if (options & MWF)
        tp->wol_opts |= WAKE_MCAST;

out_unlock:
    tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

    spin_unlock_irqrestore(&tp->lock, flags);
}

static void
rtl8168_set_hw_wol(struct net_device *dev, u32 wolopts)
{
    struct rtl8168_private *tp = netdev_priv(dev);
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
        { WAKE_MAGIC, Config3, MagicPacket },
    };

    WriteMMIO8(Cfg9346, Cfg9346_Unlock);

    switch (mcfg) {
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
        tmp = ARRAY_SIZE(cfg) - 1;

        csi_tmp = ReadERI(0xDE, 1, ERIAR_ExGMAC);
        if (wolopts & WAKE_MAGIC)
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
        if (wolopts & cfg[i].opt)
            options |= cfg[i].mask;
        WriteMMIO8(cfg[i].reg, options);
    }

    WriteMMIO8(Cfg9346, Cfg9346_Lock);
}

static void
rtl8168_powerdown_pll(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    if (tp->wol_enabled == WOL_ENABLED) {
        int auto_nego;
        int giga_ctrl;
        u16 val;
        unsigned long flags;

        rtl8168_set_hw_wol(dev, tp->wol_opts);

        if (mcfg == MCFG_8168E_VL_1 || mcfg == MCFG_8168E_VL_2 ||
            mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
            mcfg == CFG_METHOD_24 || mcfg == CFG_METHOD_25 ||
            mcfg == MCFG_8411B) {
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

#ifdef CONFIG_DOWN_SPEED_100
        auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
#else
        if (val & (LPA_10HALF | LPA_10FULL))
            auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL);
        else
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
#endif

        if ((mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
             mcfg == MCFG_8168DP_3 || mcfg == CFG_METHOD_23 ||
             mcfg == CFG_METHOD_27) && rtl8168_check_dash(tp))
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

        if (((mcfg == MCFG_8168CP_1) || (mcfg == MCFG_8168CP_2)) && (ReadMMIO16(CPlusCmd) & ASF))
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

        giga_ctrl = ReadGMII16( MII_CTRL1000) & ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
        WriteGMII16( PHY_AUTO_NEGO_REG, auto_nego);
        WriteGMII16( MII_CTRL1000, giga_ctrl);
        WriteGMII16( MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);

        WriteMMIO32(RxConfig, ReadMMIO32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);

        return;
    }

    if ((mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2 ||
         mcfg == MCFG_8168DP_3 || mcfg == CFG_METHOD_23 ||
         mcfg == CFG_METHOD_27) && rtl8168_check_dash(tp))
        return;

    if (((mcfg == MCFG_8168CP_1) || (mcfg == MCFG_8168CP_2)) && (ReadMMIO16(CPlusCmd) & ASF))
        return;

    rtl8168_phy_power_down(dev);

    switch (mcfg) {
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) & ~BIT_7);
        break;
    }
}

static void rtl8168_powerup_pll(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;


    switch (mcfg) {
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
        WriteMMIO8(PMCH, ReadMMIO8(PMCH) | BIT_7 | BIT_6);
        break;
    }

    rtl8168_phy_power_up (dev);
}

static void
rtl8168_get_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 options;
    unsigned long flags;

    wol->wolopts = 0;

    if (mcfg == CFG_METHOD_DEFAULT) {
        wol->supported = 0;
        return;
    } else {
        wol->supported = WAKE_ANY;
    }

    spin_lock_irqsave(&tp->lock, flags);

    options = ReadMMIO8(Config1);
    if (!(options & PMEnable))
        goto out_unlock;

    wol->wolopts = tp->wol_opts;

out_unlock:
    spin_unlock_irqrestore(&tp->lock, flags);
}

static int
rtl8168_set_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    if (mcfg == CFG_METHOD_DEFAULT)
        return -EOPNOTSUPP;

    spin_lock_irqsave(&tp->lock, flags);

    tp->wol_opts = wol->wolopts;

    tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}

static void
rtl8168_get_drvinfo(struct net_device *dev,
                    struct ethtool_drvinfo *info)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    strcpy(info->driver, MODULENAME);
    strcpy(info->version, RTL8168_VERSION);
    strcpy(info->bus_info, pci_name(tp->pci_dev));
    info->regdump_len = R8168_REGS_SIZE;
    info->eedump_len = tp->eeprom_len;
}

static int
rtl8168_get_regs_len(struct net_device *dev)
{
    return R8168_REGS_SIZE;
}

static int
rtl8168_set_speed_xmii(struct net_device *dev,
                       u8 autoneg,
                       u16 speed,
                       u8 duplex)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int auto_nego = 0;
    int giga_ctrl = 0;
    int bmcr_true_force = 0;
    unsigned long flags;

    if ((speed != SPEED_1000) &&
        (speed != SPEED_100) &&
        (speed != SPEED_10)) {
        speed = SPEED_1000;
        duplex = DUPLEX_FULL;
    }

    auto_nego = ReadGMII16( PHY_AUTO_NEGO_REG);
    auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL | PHY_Cap_PAUSE | PHY_Cap_ASYM_PAUSE);

    giga_ctrl = ReadGMII16( MII_CTRL1000);
    giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);

    if ((autoneg == AUTONEG_ENABLE) || (speed == SPEED_1000)) {
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
        } else if (speed == SPEED_1000) {
            giga_ctrl |= ADVERTISE_1000HALF |
                         ADVERTISE_1000FULL;

            auto_nego |= ADVERTISE_100HALF |
                         ADVERTISE_100FULL |
                         ADVERTISE_10HALF |
                         ADVERTISE_10FULL;
        }

        //flow contorol
        if (dev->mtu <= ETH_DATA_LEN)
            auto_nego |= PHY_Cap_PAUSE|PHY_Cap_ASYM_PAUSE;

        tp->phy_auto_nego_reg = auto_nego;
        tp->phy_1000_ctrl_reg = giga_ctrl;

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( PHY_AUTO_NEGO_REG, auto_nego);
        WriteGMII16( MII_CTRL1000, giga_ctrl);
        WriteGMII16( MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        mdelay(20);
    } else {
        /*true force*/
#ifndef BMCR_SPEED100
#define BMCR_SPEED100   0x0040
#endif

#ifndef BMCR_SPEED10
#define BMCR_SPEED10    0x0000
#endif
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED10 | BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED100 | BMCR_FULLDPLX;
        }

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( MII_BMCR, bmcr_true_force);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }

    tp->autoneg = autoneg;
    tp->speed = speed;
    tp->duplex = duplex;

    if (mcfg == MCFG_8168DP_1)
        rtl8168dp_10mbps_gphy_para(dev);

    return 0;
}

static int
rtl8168_set_speed(struct net_device *dev,
                  u8 autoneg,
                  u16 speed,
                  u8 duplex)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int ret;

    ret = tp->set_speed(dev, autoneg, speed, duplex);

    return ret;
}

static int
rtl8168_set_settings(struct net_device *dev,
                     struct ethtool_cmd *cmd)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int ret;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    ret = rtl8168_set_speed(dev, cmd->autoneg, cmd->speed, cmd->duplex);
    spin_unlock_irqrestore(&tp->lock, flags);

    return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32
rtl8168_get_tx_csum(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    u32 ret;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    ret = ((dev->features & NETIF_F_IP_CSUM) != 0);
    spin_unlock_irqrestore(&tp->lock, flags);

    return ret;
}

static u32
rtl8168_get_rx_csum(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    u32 ret;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    ret = cp_cmd & RxChkSum;
    spin_unlock_irqrestore(&tp->lock, flags);

    return ret;
}

static int
rtl8168_set_tx_csum(struct net_device *dev,
                    u32 data)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    if (mcfg == CFG_METHOD_DEFAULT)
        return -EOPNOTSUPP;

    spin_lock_irqsave(&tp->lock, flags);

    if (data)
        dev->features |= NETIF_F_IP_CSUM;
    else
        dev->features &= ~NETIF_F_IP_CSUM;

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}

static int
rtl8168_set_rx_csum(struct net_device *dev,
                    u32 data)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    if (mcfg == CFG_METHOD_DEFAULT)
        return -EOPNOTSUPP;

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

#ifdef CONFIG_R8168_VLAN

static inline u32
rtl8168_tx_vlan_tag(struct rtl8168_private *tp,
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
rtl8168_vlan_rx_register(struct net_device *dev,
                         struct vlan_group *grp)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    tp->vlgrp = grp;
    if (tp->vlgrp)
        cp_cmd |= RxVlan;
    else
        cp_cmd &= ~RxVlan;
    WriteMMIO16(CPlusCmd, cp_cmd);
    ReadMMIO16(CPlusCmd);
    spin_unlock_irqrestore(&tp->lock, flags);
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
static void
rtl8168_vlan_rx_kill_vid(struct net_device *dev,
                         unsigned short vid)
{
    struct rtl8168_private *tp = netdev_priv(dev);
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
rtl8168_rx_vlan_skb(struct rtl8168_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
    u32 opts2 = le32_to_cpu(desc->opts2);
    int ret = -1;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    if (tp->vlgrp && (opts2 & RxVlanTag)) {
        rtl8168_rx_hwaccel_skb(skb, tp->vlgrp,
                               swab16(opts2 & 0xffff));
        ret = 0;
    }
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    if (opts2 & RxVlanTag)
        __vlan_hwaccel_put_tag(skb, swab16(opts2 & 0xffff));
#else
    if (opts2 & RxVlanTag)
        __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), swab16(opts2 & 0xffff));
#endif

    desc->opts2 = 0;
    return ret;
}

#else /* !CONFIG_R8168_VLAN */

static inline u32
rtl8168_tx_vlan_tag(struct rtl8168_private *tp,
                    struct sk_buff *skb)
{
    return 0;
}

static int
rtl8168_rx_vlan_skb(struct rtl8168_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
    return -1;
}

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32 rtl8168_fix_features(struct net_device *dev, u32 features)
#else
static netdev_features_t rtl8168_fix_features(struct net_device *dev,
        netdev_features_t features)
#endif
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    if (dev->mtu > ETH_DATA_LEN) {
        features &= ~NETIF_F_ALL_TSO;
        features &= ~NETIF_F_ALL_CSUM;
    }
    spin_unlock_irqrestore(&tp->lock, flags);

    return features;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int rtl8168_hw_set_features(struct net_device *dev, u32 features)
#else
static int rtl8168_hw_set_features(struct net_device *dev,
                                   netdev_features_t features)
#endif
{
    struct rtl8168_private *tp = netdev_priv(dev);
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
static int rtl8168_set_features(struct net_device *dev, u32 features)
#else
static int rtl8168_set_features(struct net_device *dev,
                                netdev_features_t features)
#endif
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    rtl8168_hw_set_features(dev, features);

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}

#endif

static void rtl8168_gset_xmii(struct net_device *dev,
                              struct ethtool_cmd *cmd)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 status;
    unsigned long flags;

    cmd->supported = SUPPORTED_10baseT_Half |
                     SUPPORTED_10baseT_Full |
                     SUPPORTED_100baseT_Half |
                     SUPPORTED_100baseT_Full |
                     SUPPORTED_1000baseT_Full |
                     SUPPORTED_Autoneg |
                     SUPPORTED_TP;

    spin_lock_irqsave(&tp->phy_lock, flags);
    cmd->autoneg = (ReadGMII16( MII_BMCR) & BMCR_ANENABLE) ? 1 : 0;
    spin_unlock_irqrestore(&tp->phy_lock, flags);
    cmd->advertising = ADVERTISED_TP | ADVERTISED_Autoneg;

    if (tp->phy_auto_nego_reg & ADVERTISE_10HALF)
        cmd->advertising |= ADVERTISED_10baseT_Half;
    if (tp->phy_auto_nego_reg & ADVERTISE_10FULL)
        cmd->advertising |= ADVERTISED_10baseT_Full;
    if (tp->phy_auto_nego_reg & ADVERTISE_100HALF)
        cmd->advertising |= ADVERTISED_100baseT_Half;
    if (tp->phy_auto_nego_reg & ADVERTISE_100FULL)
        cmd->advertising |= ADVERTISED_100baseT_Full;
    if (tp->phy_1000_ctrl_reg & ADVERTISE_1000FULL)
        cmd->advertising |= ADVERTISED_1000baseT_Full;

    status = ReadMMIO8(PHYstatus);

    if (status & _1000bpsF)
        cmd->speed = SPEED_1000;
    else if (status & _100bps)
        cmd->speed = SPEED_100;
    else if (status & _10bps)
        cmd->speed = SPEED_10;

    if (status & TxFlowCtrl)
        cmd->advertising |= ADVERTISED_Asym_Pause;

    if (status & RxFlowCtrl)
        cmd->advertising |= ADVERTISED_Pause;

    cmd->duplex = ((status & _1000bpsF) || (status & FullDup)) ?
                  DUPLEX_FULL : DUPLEX_HALF;


}

static int
rtl8168_get_settings(struct net_device *dev,
                     struct ethtool_cmd *cmd)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    tp->get_settings(dev, cmd);

    return 0;
}

static void rtl8168_get_regs(struct net_device *dev, struct ethtool_regs *regs,
                             void *p)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned int i;
    u8 *data = p;
    unsigned long flags;

    if (regs->len > R8168_REGS_SIZE)
        regs->len = R8168_REGS_SIZE;

    spin_lock_irqsave(&tp->lock, flags);
    for (i = 0; i < regs->len; i++)
        data[i] = readb(tp->mmio_addr + i);
    spin_unlock_irqrestore(&tp->lock, flags);
}

static u32
rtl8168_get_msglevel(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    return tp->msg_enable;
}

static void
rtl8168_set_msglevel(struct net_device *dev,
                     u32 value)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    tp->msg_enable = value;
}

static const char rtl8168_gstrings[][ETH_GSTRING_LEN] = {
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

struct rtl8168_counters {
    u64 tx_packets;
    u64 rx_packets;
    u64 tx_errors;
    u32 rx_errors;
    u16 rx_missed;
    u16 align_errors;
    u32 tx_one_collision;
    u32 tx_multi_collision;
    u64 rx_unicast;
    u64 rx_broadcast;
    u32 rx_multicast;
    u16 tx_aborted;
    u16 tx_underun;
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
static int rtl8168_get_stats_count(struct net_device *dev)
{
    return ARRAY_SIZE(rtl8168_gstrings);
}
#else
static int rtl8168_get_sset_count(struct net_device *dev, int sset)
{
    switch (sset) {
    case ETH_SS_STATS:
        return ARRAY_SIZE(rtl8168_gstrings);
    default:
        return -EOPNOTSUPP;
    }
}
#endif
static void
rtl8168_get_ethtool_stats(struct net_device *dev,
                          struct ethtool_stats *stats,
                          u64 *data)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    struct rtl8168_counters *counters;
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

    data[0] = le64_to_cpu(counters->tx_packets);
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
rtl8168_get_strings(struct net_device *dev,
                    u32 stringset,
                    u8 *data)
{
    switch (stringset) {
    case ETH_SS_STATS:
        memcpy(data, *rtl8168_gstrings, sizeof(rtl8168_gstrings));
        break;
    }
}
static int rtl_get_eeprom_len(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    return tp->eeprom_len;
}

static int rtl_get_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom, u8 *buf)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int i,j,ret;
    int start_w, end_w;
    int VPD_addr, VPD_data;
    u32 *eeprom_buff;
    u16 tmp;
    void __iomem *ioaddr = tp->mmio_addr;

    if (tp->eeprom_type==EEPROM_TYPE_NONE) {
        dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Detect none EEPROM\n");
        return -EOPNOTSUPP;
    } else if (eeprom->len == 0 || (eeprom->offset+eeprom->len) > tp->eeprom_len) {
        dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Invalid parameter\n");
        return -EINVAL;
    }

    switch (mcfg) {
    case MCFG_8168D_1:
    case MCFG_8168D_2:
        VPD_addr = 0xCE;
        VPD_data = 0xD0;
        break;

    case MCFG_8168B_1:
    case MCFG_8168B_2:
    case MCFG_8168B_3:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
        return -EOPNOTSUPP;
    default:
        VPD_addr = 0xD2;
        VPD_data = 0xD4;
        break;
    }

    start_w = eeprom->offset >> 2;
    end_w = (eeprom->offset + eeprom->len - 1) >> 2;

    eeprom_buff = kmalloc(sizeof(u32)*(end_w - start_w + 1), GFP_KERNEL);
    if (!eeprom_buff)
        return -ENOMEM;

    WriteMMIO8(Cfg9346, Cfg9346_Unlock);
    ret = -EFAULT;
    for (i=start_w; i<=end_w; i++) {
        pci_write_config_word(tp->pci_dev, VPD_addr, (u16)i*4);
        ret = -EFAULT;
        for (j = 0; j < 10; j++) {
            IODelay(400);
            pci_read_config_word(tp->pci_dev, VPD_addr, &tmp);
            if (tmp&0x8000) {
                ret = 0;
                break;
            }
        }

        if (ret)
            break;

        pci_read_config_dword(tp->pci_dev, VPD_data, &eeprom_buff[i-start_w]);
    }
    WriteMMIO8(Cfg9346, Cfg9346_Lock);

    if (!ret)
        memcpy(buf, (u8 *)eeprom_buff + (eeprom->offset & 3), eeprom->len);

    kfree(eeprom_buff);

    return ret;
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
    struct rtl8168_private *tp = netdev_priv(dev);

    if (mcfg == CFG_METHOD_DEFAULT)
        return -EOPNOTSUPP;

#ifdef NETIF_F_SG
    if (data)
        dev->features |= NETIF_F_SG;
    else
        dev->features &= ~NETIF_F_SG;
#endif

    return 0;
}
#endif

static const struct ethtool_ops rtl8168_ethtool_ops = {
    .get_drvinfo        = rtl8168_get_drvinfo,
    .get_regs_len       = rtl8168_get_regs_len,
    .get_link       = ethtool_op_get_link,
    .get_settings       = rtl8168_get_settings,
    .set_settings       = rtl8168_set_settings,
    .get_msglevel       = rtl8168_get_msglevel,
    .set_msglevel       = rtl8168_set_msglevel,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
    .get_rx_csum        = rtl8168_get_rx_csum,
    .set_rx_csum        = rtl8168_set_rx_csum,
    .get_tx_csum        = rtl8168_get_tx_csum,
    .set_tx_csum        = rtl8168_set_tx_csum,
    .get_sg         = ethtool_op_get_sg,
    .set_sg         = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
    .get_tso        = ethtool_op_get_tso,
    .set_tso        = ethtool_op_set_tso,
#endif
#endif
    .get_regs       = rtl8168_get_regs,
    .get_wol        = rtl8168_get_wol,
    .set_wol        = rtl8168_set_wol,
    .get_strings        = rtl8168_get_strings,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
    .get_stats_count    = rtl8168_get_stats_count,
#else
    .get_sset_count     = rtl8168_get_sset_count,
#endif
    .get_ethtool_stats  = rtl8168_get_ethtool_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#ifdef ETHTOOL_GPERMADDR
    .get_perm_addr      = ethtool_op_get_perm_addr,
#endif
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
    .get_eeprom     = rtl_get_eeprom,
    .get_eeprom_len     = rtl_get_eeprom_len,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    .get_ts_info        = ethtool_op_get_ts_info,
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
};


static int rtl8168_enable_EEE(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int ret;
    u16 data;
    unsigned long flags;

    ret = 0;
    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0020);
        data = ReadGMII16( 0x15) | 0x0100;
        WriteGMII16( 0x15, data);
        WriteGMII16( 0x1F, 0x0006);
        WriteGMII16( 0x00, 0x5A30);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0006);
        WriteGMII16( 0x0D, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        if ((ReadMMIO8(Config4)&0x40) && (ReadMMIO8(0x6D) & BIT_7)) {
            data = ReadMMIO16(CustomLED);
            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8AC8);
            WriteGMII16( 0x06, data);
            WriteGMII16( 0x05, 0x8B82);
            data = ReadGMII16( 0x06) | 0x0010;
            WriteGMII16( 0x05, 0x8B82);
            WriteGMII16( 0x06, data);
            WriteGMII16( 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
        break;

    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        data = ReadERI(ioaddr,0x1B0 ,4,ERIAR_ExGMAC) | 0x0003;
        WriteERI(0x1B0, 4, data, ERIAR_ExGMAC);
        WriteGMII16(0x1F , 0x0004);
        WriteGMII16(0x1F , 0x0007);
        WriteGMII16(0x1E , 0x0020);
        data = ReadGMII16( 0x15)|0x0100;
        WriteGMII16(0x15 , data);
        WriteGMII16(0x1F , 0x0002);
        WriteGMII16(0x1F , 0x0005);
        WriteGMII16(0x05 , 0x8B85);
        data = ReadGMII16( 0x06)|0x2000;
        WriteGMII16(0x06 , data);
        WriteGMII16(0x1F , 0x0000);
        WriteGMII16(0x0D , 0x0007);
        WriteGMII16(0x0E , 0x003C);
        WriteGMII16(0x0D , 0x4007);
        WriteGMII16(0x0E , 0x0006);
        WriteGMII16(0x1D , 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        spin_lock_irqsave(&tp->phy_lock, flags);
        data = ReadERI(ioaddr,0x1B0 ,4,ERIAR_ExGMAC);
        data |= BIT_1 | BIT_0;
        WriteERI(0x1B0, 4, data, ERIAR_ExGMAC);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1e, 0x0020);
        data = ReadGMII16( 0x15);
        data |= BIT_8;
        WriteGMII16( 0x15, data);
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        data = ReadGMII16( 0x06);
        data |= BIT_13;
        WriteGMII16( 0x06, data);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0006);
        WriteGMII16( 0x0D, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
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
//      dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support EEE\n");
        ret = -EOPNOTSUPP;
    }

    /*Advanced EEE*/
    switch (mcfg) {
    case CFG_METHOD_25:
        WriteERI(0x1EA, 1, 0xFA, ERIAR_ExGMAC);

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
    case MCFG_8411B:
        data = mac_ocp_read(0xE052);
        data |= BIT_0;
        mac_ocp_write(0xE052, data);
        data = mac_ocp_read(0xE056);
        data &= 0xFF0F;
        data |= (BIT_4 | BIT_5 | BIT_6);
        mac_ocp_write(0xE056, data);

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
    case CFG_METHOD_27:
        data = mac_ocp_read(0xE052);
        data |= BIT_0;
        mac_ocp_write(0xE052, data);
        data = mac_ocp_read(0xE056);
        data &= 0xFF0F;
        data |= (BIT_4 | BIT_5 | BIT_6);
        mac_ocp_write(0xE056, data);
        break;
    }

    return ret;
}

static int rtl8168_disable_EEE(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int ret;
    u16 data;
    unsigned long flags;

    ret = 0;
    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0020);
        data = ReadGMII16( 0x15) & ~0x0100;
        WriteGMII16( 0x15, data);
        WriteGMII16( 0x1F, 0x0006);
        WriteGMII16( 0x00, 0x5A00);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        if (ReadMMIO8(Config4) & 0x40) {
            data = ReadMMIO16(CustomLED);
            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8B82);
            data = ReadGMII16( 0x06) & ~0x0010;
            WriteGMII16( 0x05, 0x8B82);
            WriteGMII16( 0x06, data);
            WriteGMII16( 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
        break;

    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        data = ReadERI(ioaddr,0x1B0 ,4,ERIAR_ExGMAC)& ~0x0003;
        WriteERI(0x1B0, 4, data, ERIAR_ExGMAC);
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        data = ReadGMII16( 0x06) & ~0x2000;
        WriteGMII16( 0x06, data);
        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0020);
        data = ReadGMII16( 0x15) & ~0x0100;
        WriteGMII16(0x15 , data);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        spin_lock_irqsave(&tp->phy_lock, flags);
        data = ReadERI(ioaddr,0x1B0 ,4,ERIAR_ExGMAC);
        data &= ~(BIT_1 | BIT_0);
        WriteERI(0x1B0, 4, data, ERIAR_ExGMAC);
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        data = ReadGMII16( 0x06);
        data &= ~BIT_13;
        WriteGMII16( 0x06, data);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1e, 0x0020);
        data = ReadGMII16( 0x15);
        data &= ~BIT_8;
        WriteGMII16( 0x15, data);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0x0007);
        WriteGMII16( 0x0E, 0x003C);
        WriteGMII16( 0x0D, 0x4007);
        WriteGMII16( 0x0E, 0x0000);
        WriteGMII16( 0x0D, 0x0000);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
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
//      dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support EEE\n");
        ret = -EOPNOTSUPP;
        break;
    }

    /*Advanced EEE*/
    switch (mcfg) {
    case CFG_METHOD_25:
        WriteERI(0x1EA, 1, 0x00, ERIAR_ExGMAC);

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0A42);
        data = ReadGMII16( 0x16);
        data &= ~(BIT_1);
        WriteGMII16( 0x16, data);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;
    case MCFG_8411B:
        data = mac_ocp_read(0xE052);
        data &= ~(BIT_0);
        mac_ocp_write(0xE052, data);

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0A42);
        data = ReadGMII16( 0x16);
        data &= ~(BIT_1);
        WriteGMII16( 0x16, data);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;
    case CFG_METHOD_27:
        data = mac_ocp_read(0xE052);
        data &= ~(BIT_0);
        mac_ocp_write(0xE052, data);
        break;
    }

    return ret;
}

#if 0

static int rtl8168_enable_green_feature(struct rtl8168_private *tp)
{
    u16 gphy_val;
    unsigned long flags;

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0003);
        gphy_val = ReadGMII16( 0x10) | 0x0400;
        WriteGMII16( 0x10, gphy_val);
        gphy_val = ReadGMII16( 0x19) | 0x0001;
        WriteGMII16( 0x19, gphy_val);
        WriteGMII16( 0x1F, 0x0005);
        gphy_val = ReadGMII16( 0x01) & ~0x0100;
        WriteGMII16( 0x01, gphy_val);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x00, 0x9200);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        mdelay(20);
        break;

    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0003);
        gphy_val = ReadGMII16( 0x10);
        gphy_val |= BIT_10;
        WriteGMII16( 0x10, gphy_val);
        gphy_val = ReadGMII16( 0x19);
        gphy_val |= BIT_0;
        WriteGMII16( 0x19, gphy_val);
        WriteGMII16( 0x1F, 0x0005);
        gphy_val = ReadGMII16( 0x01);
        gphy_val |= BIT_8;
        WriteGMII16( 0x01, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x9200);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8011);
        gphy_val = ReadGMII16( 0x14) | BIT_14;
        WriteGMII16( 0x14, gphy_val);
        WriteGMII16( 0x1F, 0x0A40);
        WriteGMII16( 0x00, 0x9200);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    default:
        dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support Green Feature\n");
        break;
    }

    return 0;
}

static int rtl8168_disable_green_feature(struct rtl8168_private *tp)
{
    u16 gphy_val;
    unsigned long flags;

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0005);
        gphy_val = ReadGMII16( 0x01) | 0x0100;
        WriteGMII16( 0x01, gphy_val);
        WriteGMII16( 0x1F, 0x0003);
        gphy_val = ReadGMII16( 0x10) & ~0x0400;
        WriteGMII16( 0x10, gphy_val);
        gphy_val = ReadGMII16( 0x19) & ~0x0001;
        WriteGMII16( 0x19, gphy_val);
        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x06) & ~0x7000;
        gphy_val |= 0x3000;
        WriteGMII16( 0x06, gphy_val);
        gphy_val = ReadGMII16( 0x0D) & 0x0700;
        gphy_val |= 0x0500;
        WriteGMII16( 0x0D, gphy_val);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1f, 0x0003);
        gphy_val = ReadGMII16( 0x19);
        gphy_val &= ~BIT_0;
        WriteGMII16( 0x19, gphy_val);
        gphy_val = ReadGMII16( 0x10);
        gphy_val &= ~BIT_10;
        WriteGMII16( 0x10, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8011);
        gphy_val = ReadGMII16( 0x14) & ~BIT_14;
        WriteGMII16( 0x14, gphy_val);
        WriteGMII16( 0x1F, 0x0A40);
        WriteGMII16( 0x00, 0x9200);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    default:
        dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support Green Feature\n");
        break;
    }

    return 0;
}

#endif

static void rtl8168_get_mac_version(struct rtl8168_private *tp, void __iomem *ioaddr)
{
    u32 reg,val32;
    u32 ICVerID;

    val32 = ReadMMIO32(TxConfig)  ;
    reg = val32 & 0x7c800000;
    ICVerID = val32 & 0x00700000;

    switch (reg) {
    case 0x30000000:
        mcfg = MCFG_8168B_1;
        tp->efuse = EFUSE_NOT_SUPPORT;
        break;
    case 0x38000000:
        if (ICVerID == 0x00000000) {
            mcfg = CFG_METHOD_2;
        } else if (ICVerID == 0x00500000) {
            mcfg = MCFG_8168B_3;
        } else {
            mcfg = MCFG_8168B_3;
        }
        tp->efuse = EFUSE_NOT_SUPPORT;
        break;
    case 0x3C000000:
        if (ICVerID == 0x00000000) {
            mcfg = MCFG_8168C_1;
        } else if (ICVerID == 0x00200000) {
            mcfg = MCFG_8168C_2;
        } else if (ICVerID == 0x00400000) {
            mcfg = MCFG_8168C_3;
        } else {
            mcfg = MCFG_8168C_3;
        }
        tp->efuse = EFUSE_NOT_SUPPORT;
        break;
    case 0x3C800000:
        if (ICVerID == 0x00100000) {
            mcfg = MCFG_8168CP_1;
        } else if (ICVerID == 0x00300000) {
            mcfg = MCFG_8168CP_2;
        } else {
            mcfg = MCFG_8168CP_2;
        }
        tp->efuse = EFUSE_NOT_SUPPORT;
        break;
    case 0x28000000:
        if (ICVerID == 0x00100000) {
            mcfg = MCFG_8168D_1;
        } else if (ICVerID == 0x00300000) {
            mcfg = MCFG_8168D_2;
        } else {
            mcfg = MCFG_8168D_2;
        }
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x28800000:
        if (ICVerID == 0x00000000) {
            mcfg = MCFG_8168DP_1;
        } else if (ICVerID == 0x00200000) {
            mcfg = MCFG_8168DP_2;
            WriteMMIO32(0xD0, ReadMMIO32(0xD0) | 0x00020000);
        } else {// if (ICVerID == 0x00300000)
            mcfg = MCFG_8168DP_3;
        }
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x2C000000:
        if (ICVerID == 0x00100000)
            mcfg = MCFG_8168E_1;
        else if (ICVerID == 0x00200000)
            mcfg = MCFG_8168E_2;
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x2C800000:
        if (ICVerID == 0x00000000)
            mcfg = MCFG_8168E_VL_1;
        else if (ICVerID == 0x00100000)
            mcfg = MCFG_8168E_VL_2;
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x48000000:
        if (ICVerID == 0x00000000)
            mcfg = MCFG_8168F_1;
        else if (ICVerID == 0x00100000)
            mcfg = MCFG_8168F_2;
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x48800000:
        mcfg = MCFG_8411_1;
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x4C000000:
        if (ICVerID == 0x00000000)
            mcfg = CFG_METHOD_21;
        else if (ICVerID == 0x00100000)
            mcfg = CFG_METHOD_22;
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x50000000:
        if (ICVerID == 0x00000000) {
            mcfg = CFG_METHOD_23;
        } else if (ICVerID == 0x00100000) {
            mcfg = CFG_METHOD_27;
        } else {
            mcfg = CFG_METHOD_27;
        }
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x50800000:
        if (ICVerID == 0x00000000) {
            mcfg = CFG_METHOD_24;
        } else if (ICVerID == 0x00100000) {
            mcfg = CFG_METHOD_25;
        } else {
            mcfg = CFG_METHOD_25;
        }
        tp->efuse = EFUSE_SUPPORT;
        break;
    case 0x5C800000:
        mcfg = MCFG_8411B;
        tp->efuse = EFUSE_SUPPORT;
        break;
    default:
        printk("unknown chip version (%x)\n",reg);
        mcfg = CFG_METHOD_DEFAULT;
        tp->efuse = EFUSE_NOT_SUPPORT;
        break;
    }
}

static void
rtl8168_print_mac_version(struct rtl8168_private *tp)
{
    int i;
    for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
        if (mcfg == rtl_chip_info[i].mcfg) {
            dprintk("Realtek PCIe GBE Family Controller mcfg = %04d\n",
                    rtl_chip_info[i].mcfg);
            return;
        }
    }

    dprintk("mac_version == Unknown\n");
}

static u8 rtl8168_efuse_read(struct rtl8168_private *tp, u16 reg)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u8 efuse_data;
    u32 temp;
    int cnt;

    if (tp->efuse == EFUSE_NOT_SUPPORT)
        return EFUSE_READ_FAIL;

    temp = EFUSE_READ | ((reg & EFUSE_Reg_Mask) << EFUSE_Reg_Shift);
    WriteMMIO32(EFUSEAR, temp);

    do {
        IODelay(100);
        temp = ReadMMIO32(EFUSEAR);
        cnt++;
    } while (!(temp & EFUSE_READ_OK) && (temp < EFUSE_Check_Cnt));

    if (temp == EFUSE_Check_Cnt)
        efuse_data = EFUSE_READ_FAIL;
    else
        efuse_data = (u8)(ReadMMIO32(EFUSEAR) & EFUSE_Data_Mask);

    return efuse_data;
}

static void
rtl8168_tally_counter_addr_fill(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;

    if (!tp->tally_paddr)
        return;

    WriteMMIO32(CounterAddrHigh, (u64)tp->tally_paddr >> 32);
    WriteMMIO32(CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32)));
}

static void
rtl8168_tally_counter_clear(struct rtl8168_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;

    if (mcfg == MCFG_8168B_1 || mcfg == MCFG_8168B_2 ||
        mcfg == MCFG_8168B_3 )
        return;

    if (!tp->tally_paddr)
        return;

    WriteMMIO32(CounterAddrHigh, (u64)tp->tally_paddr >> 32);
    WriteMMIO32(CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32) | BIT_0));
}

static void
rtl8168_exit_oob(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 data16;

    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case CFG_METHOD_23:
    case CFG_METHOD_27:
        rtl8168_driver_start(tp);
        break;
    }

    rtl8168_nic_reset(dev);

    switch (mcfg) {
    case MCFG_8411_1:
        rtl8168_wait_ll_share_fifo_ready(dev);

        data16 = mac_ocp_read(0xD4DE) | BIT_15;
        mac_ocp_write(0xD4DE, data16);

        rtl8168_wait_ll_share_fifo_ready(dev);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO8(MCUCmd_reg, ReadMMIO8(MCUCmd_reg) & ~Now_is_oob);

        data16 = mac_ocp_read(0xE8DE) & ~BIT_14;
        mac_ocp_write(0xE8DE, data16);
        rtl8168_wait_ll_share_fifo_ready(dev);

        data16 = mac_ocp_read(0xE8DE) | BIT_15;
        mac_ocp_write(0xE8DE, data16);

        rtl8168_wait_ll_share_fifo_ready(dev);
        break;
    }
}

static void
rtl8168_hw_mac_mcu_config(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned int i;


    if (mcfg == CFG_METHOD_21) {
        u16 rtl8111g_phy_value[]= {
            0xE008, 0xE01B, 0xE01D, 0xE01F, 0xE022,
            0xE025, 0xE031, 0xE04D, 0x49D2, 0xF10D,
            0x766C, 0x49E2, 0xF00A, 0x1EC0, 0x8EE1,
            0xC60A, 0x77C0, 0x4870, 0x9FC0, 0x1EA0,
            0xC707, 0x8EE1, 0x9D6C, 0xC603, 0xBE00,
            0xB416, 0x0076, 0xE86C, 0xC602, 0xBE00,
            0xA000, 0xC602, 0xBE00, 0x0000, 0x1B76,
            0xC202, 0xBA00, 0x059C, 0x1B76, 0xC602,
            0xBE00, 0x065A, 0x74E6, 0x1B78, 0x46DC,
            0x1300, 0xF005, 0x74F8, 0x48C3, 0x48C4,
            0x8CF8, 0x64E7, 0xC302, 0xBB00, 0x06A0,
            0x74E4, 0x49C5, 0xF106, 0x49C6, 0xF107,
            0x48C8, 0x48C9, 0xE011, 0x48C9, 0x4848,
            0xE00E, 0x4848, 0x49C7, 0xF00A, 0x48C9,
            0xC60D, 0x1D1F, 0x8DC2, 0x1D00, 0x8DC3,
            0x1D11, 0x8DC0, 0xE002, 0x4849, 0x94E5,
            0xC602, 0xBE00, 0x01F0, 0xE434, 0x49D9,
            0xF01B, 0xC31E, 0x7464, 0x49C4, 0xF114,
            0xC31B, 0x6460, 0x14FA, 0xFA02, 0xE00F,
            0xC317, 0x7460, 0x49C0, 0xF10B, 0xC311,
            0x7462, 0x48C1, 0x9C62, 0x4841, 0x9C62,
            0xC30A, 0x1C04, 0x8C60, 0xE004, 0x1C15,
            0xC305, 0x8C60, 0xC602, 0xBE00, 0x0384,
            0xE434, 0xE030, 0xE61C, 0xE906
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
        for (i = 0; i < ARRAY_SIZE(rtl8111g_phy_value); i++)
            mac_ocp_write(0xF800+i*2, rtl8111g_phy_value[i]);
        mac_ocp_write(0xFC26, 0x8000);
        mac_ocp_write(0xFC28, 0x0075);
        mac_ocp_write(0xFC2E, 0x059B);
        mac_ocp_write(0xFC30, 0x0659);
        mac_ocp_write(0xFC32, 0x0000);
        mac_ocp_write(0xFC34, 0x0000);
        mac_ocp_write(0xFC36, 0x0000);
    } else if (mcfg == CFG_METHOD_24) {
        u16 rtl8111g_phy_value[]= {
            0xE008, 0xE011, 0xE015, 0xE018, 0xE01B,
            0xE027, 0xE043, 0xE065, 0x49E2, 0xF005,
            0x49EA, 0xF003, 0xC404, 0xBC00, 0xC403,
            0xBC00, 0x0496, 0x051A, 0x1D01, 0x8DE8,
            0xC602, 0xBE00, 0x0206, 0x1B76, 0xC202,
            0xBA00, 0x058A, 0x1B76, 0xC602, 0xBE00,
            0x0648, 0x74E6, 0x1B78, 0x46DC, 0x1300,
            0xF005, 0x74F8, 0x48C3, 0x48C4, 0x8CF8,
            0x64E7, 0xC302, 0xBB00, 0x068E, 0x74E4,
            0x49C5, 0xF106, 0x49C6, 0xF107, 0x48C8,
            0x48C9, 0xE011, 0x48C9, 0x4848, 0xE00E,
            0x4848, 0x49C7, 0xF00A, 0x48C9, 0xC60D,
            0x1D1F, 0x8DC2, 0x1D00, 0x8DC3, 0x1D11,
            0x8DC0, 0xE002, 0x4849, 0x94E5, 0xC602,
            0xBE00, 0x0238, 0xE434, 0x49D9, 0xF01B,
            0xC31E, 0x7464, 0x49C4, 0xF114, 0xC31B,
            0x6460, 0x14FA, 0xFA02, 0xE00F, 0xC317,
            0x7460, 0x49C0, 0xF10B, 0xC311, 0x7462,
            0x48C1, 0x9C62, 0x4841, 0x9C62, 0xC30A,
            0x1C04, 0x8C60, 0xE004, 0x1C15, 0xC305,
            0x8C60, 0xC602, 0xBE00, 0x0374, 0xE434,
            0xE030, 0xE61C, 0xE906, 0xC602, 0xBE00,
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
        for (i = 0; i < ARRAY_SIZE(rtl8111g_phy_value); i++)
            mac_ocp_write(0xF800+i*2, rtl8111g_phy_value[i]);
        mac_ocp_write(0xFC26, 0x8000);
        mac_ocp_write(0xFC28, 0x0493);
        mac_ocp_write(0xFC2A, 0x0205);
        mac_ocp_write(0xFC2C, 0x0589);
        mac_ocp_write(0xFC2E, 0x0647);
        mac_ocp_write(0xFC30, 0x0000);
        mac_ocp_write(0xFC32, 0x0215);
        mac_ocp_write(0xFC34, 0x0285);
    } else if (mcfg == CFG_METHOD_25) {
        u16 rtl8111g_phy_value[]= {
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
        for (i = 0; i < ARRAY_SIZE(rtl8111g_phy_value); i++)
            mac_ocp_write(0xF800+i*2, rtl8111g_phy_value[i]);
        mac_ocp_write(0xFC26, 0x8000);
        mac_ocp_write(0xFC2A, 0x0A2F);
        mac_ocp_write(0xFC2C, 0x0297);
    } else if (mcfg == MCFG_8411B) {
        u16 rtl8111g_phy_value[]= {
            0xE008, 0xE00A, 0xE00C, 0xE00E, 0xE027,
            0xE04F, 0xE051, 0xE053, 0xC602, 0xBE00,
            0x0000, 0xC502, 0xBD00, 0x074C, 0xC302,
            0xBB00, 0x080A, 0x6420, 0x48C2, 0x8C20,
            0xC516, 0x64A4, 0x49C0, 0xF009, 0x74A2,
            0x8CA5, 0x74A0, 0xC50E, 0x9CA2, 0x1C11,
            0x9CA0, 0xE006, 0x74F8, 0x48C4, 0x8CF8,
            0xC404, 0xBC00, 0xC403, 0xBC00, 0x0BF2,
            0x0C0A, 0xE434, 0xD3C0, 0x49D9, 0xF01F,
            0xC526, 0x64A5, 0x1400, 0xF007, 0x0C01,
            0x8CA5, 0x1C15, 0xC51B, 0x9CA0, 0xE013,
            0xC519, 0x74A0, 0x48C4, 0x8CA0, 0xC516,
            0x74A4, 0x48C8, 0x48CA, 0x9CA4, 0xC512,
            0x1B00, 0x9BA0, 0x1B1C, 0x483F, 0x9BA2,
            0x1B04, 0xC508, 0x9BA0, 0xC505, 0xBD00,
            0xC502, 0xBD00, 0x0300, 0x051E, 0xE434,
            0xE018, 0xE092, 0xDE20, 0xD3C0, 0xC602,
            0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000,
            0xC602, 0xBE00, 0x0000
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
        for (i = 0; i < ARRAY_SIZE(rtl8111g_phy_value); i++)
            mac_ocp_write(0xF800+i*2, rtl8111g_phy_value[i]);
        mac_ocp_write(0xFC26, 0x8000);
        mac_ocp_write(0xFC2A, 0x0743);
        mac_ocp_write(0xFC2C, 0x0801);
        mac_ocp_write(0xFC2E, 0x0BE9);
        mac_ocp_write(0xFC30, 0x02FD);
    } else if (mcfg == CFG_METHOD_27) {
        u16 rtl8111g_phy_value[]= {
            0xE008, 0xE0D3, 0xE0D6, 0xE0D9, 0xE0DB,
            0xE0DD, 0xE0DF, 0xE0E1, 0xC251, 0x7340,
            0x49B1, 0xF010, 0x1D02, 0x8D40, 0xC202,
            0xBA00, 0x2C3A, 0xC0F0, 0xE8DE, 0x2000,
            0x8000, 0xC0B6, 0x268C, 0x752C, 0x49D4,
            0xF112, 0xE025, 0xC2F6, 0x7146, 0xC2F5,
            0x7340, 0x49BE, 0xF103, 0xC7F2, 0xE002,
            0xC7F1, 0x304F, 0x6226, 0x49A1, 0xF1F0,
            0x7222, 0x49A0, 0xF1ED, 0x2525, 0x1F28,
            0x3097, 0x3091, 0x9A36, 0x752C, 0x21DC,
            0x25BC, 0xC6E2, 0x77C0, 0x1304, 0xF014,
            0x1303, 0xF014, 0x1302, 0xF014, 0x1301,
            0xF014, 0x49D4, 0xF103, 0xC3D7, 0xBB00,
            0xC618, 0x67C6, 0x752E, 0x22D7, 0x26DD,
            0x1505, 0xF013, 0xC60A, 0xBE00, 0xC309,
            0xBB00, 0xC308, 0xBB00, 0xC307, 0xBB00,
            0xC306, 0xBB00, 0x25C8, 0x25A6, 0x25AC,
            0x25B2, 0x25B8, 0xCD08, 0x0000, 0xC0BC,
            0xC2FF, 0x7340, 0x49B0, 0xF04E, 0x1F46,
            0x308F, 0xC3F7, 0x1C04, 0xE84D, 0x1401,
            0xF147, 0x7226, 0x49A7, 0xF044, 0x7222,
            0x2525, 0x1F30, 0x3097, 0x3091, 0x7340,
            0xC4EA, 0x401C, 0xF006, 0xC6E8, 0x75C0,
            0x49D7, 0xF105, 0xE036, 0x1D08, 0x8DC1,
            0x0208, 0x6640, 0x2764, 0x1606, 0xF12F,
            0x6346, 0x133B, 0xF12C, 0x9B34, 0x1B18,
            0x3093, 0xC32A, 0x1C10, 0xE82A, 0x1401,
            0xF124, 0x1A36, 0x308A, 0x7322, 0x25B5,
            0x0B0E, 0x1C00, 0xE82C, 0xC71F, 0x4027,
            0xF11A, 0xE838, 0x1F42, 0x308F, 0x1B08,
            0xE824, 0x7236, 0x7746, 0x1700, 0xF00D,
            0xC313, 0x401F, 0xF103, 0x1F00, 0x9F46,
            0x7744, 0x449F, 0x445F, 0xE817, 0xC70A,
            0x4027, 0xF105, 0xC302, 0xBB00, 0x2E08,
            0x2DC2, 0xC7FF, 0xBF00, 0xCDB8, 0xFFFF,
            0x0C02, 0xA554, 0xA5DC, 0x402F, 0xF105,
            0x1400, 0xF1FA, 0x1C01, 0xE002, 0x1C00,
            0xFF80, 0x49B0, 0xF004, 0x0B01, 0xA1D3,
            0xE003, 0x0B02, 0xA5D3, 0x3127, 0x3720,
            0x0B02, 0xA5D3, 0x3127, 0x3720, 0x1300,
            0xF1FB, 0xFF80, 0x7322, 0x25B5, 0x1E28,
            0x30DE, 0x30D9, 0x7264, 0x1E11, 0x2368,
            0x3116, 0xFF80, 0x1B7E, 0xC602, 0xBE00,
            0x06A6, 0x1B7E, 0xC602, 0xBE00, 0x0764,
            0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00,
            0x0000, 0xC602, 0xBE00, 0x0000, 0xC602,
            0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000
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
        for (i = 0; i < ARRAY_SIZE(rtl8111g_phy_value); i++)
            mac_ocp_write(0xF800+i*2, rtl8111g_phy_value[i]);
        mac_ocp_write(0xFC26, 0x8000);
        mac_ocp_write(0xFC28, 0x2549);
        mac_ocp_write(0xFC2A, 0x06A5);
        mac_ocp_write(0xFC2C, 0x0763);
    }
}

static void
rtl8168_hw_init(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        break;
    }

    switch (mcfg) {
    case MCFG_8168D_1:
    case MCFG_8168D_2:
        WriteMMIO8(DBG_reg, ReadMMIO8(DBG_reg) | BIT_1 | BIT_7);
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
        WriteMMIO8(0xF2, (ReadMMIO8(0xF2) % ~(BIT_2 | BIT_1 | BIT_0)));
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        if (aspm) {
            WriteMMIO8(0x6E, ReadMMIO8(0x6E) | BIT_6);
            WriteERI(0x1AE, 2, 0x0403, ERIAR_ExGMAC);
        }
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
        if (aspm) {
            if (ReadMMIO8(Config5) & BIT_3) {
                WriteMMIO8(0x6E, ReadMMIO8(0x6E) | BIT_6);
                WriteERI(0x1AE, 2, 0x0403, ERIAR_ExGMAC);

            }
        }
        break;
    }

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
        WriteERI(0x174, 2, 0x0000, ERIAR_ExGMAC);
        mac_ocp_write(0xE428, 0x0010);
        break;
    }

    if (mcfg == MCFG_8168D_2 || mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2)
        WriteMMIO8(0xF3, ReadMMIO8(0xF3) | BIT_2);


    rtl8168_hw_mac_mcu_config(dev);

    /*disable ocp phy power saving*/
    if (mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B || mcfg == CFG_METHOD_27) {
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0C41);
        WriteGMII16( 0x13, 0x0000);
        WriteGMII16( 0x13, 0x0500);
        WriteGMII16( 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
}

static void
rtl8168_hw_ephy_config(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 ephy_data;


    if (mcfg == MCFG_8168C_1) {
        /*Set EPHY registers    begin*/
        /*Set EPHY register offset 0x02 bit 11 to 0 and bit 12 to 1*/
        ephy_data = rtl8168_ephy_read(ioaddr, 0x02);
        ephy_data &= ~BIT_11;
        ephy_data |= BIT_12;
        rtl8168_ephy_write(ioaddr, 0x02, ephy_data);

        /*Set EPHY register offset 0x03 bit 1 to 1*/
        ephy_data = rtl8168_ephy_read(ioaddr, 0x03);
        ephy_data |= (1 << 1);
        rtl8168_ephy_write(ioaddr, 0x03, ephy_data);

        /*Set EPHY register offset 0x06 bit 7 to 0*/
        ephy_data = rtl8168_ephy_read(ioaddr, 0x06);
        ephy_data &= ~(1 << 7);
        rtl8168_ephy_write(ioaddr, 0x06, ephy_data);
        /*Set EPHY registers    end*/
    } else if (mcfg == MCFG_8168C_2) {
        /******set EPHY registers for RTL8168CP begin******/
        /*Set EPHY register offset 0x01 bit 0 to 1.*/
        ephy_data = rtl8168_ephy_read(ioaddr, 0x01);
        ephy_data |= (1 << 0);
        rtl8168_ephy_write(ioaddr, 0x01, ephy_data);

        /*Set EPHY register offset 0x03 bit 10 to 0, bit 9 to 1 and bit 5 to 1.*/
        ephy_data = rtl8168_ephy_read(ioaddr, 0x03);
        ephy_data &= ~(1 << 10);
        ephy_data |= (1 << 9);
        ephy_data |= (1 << 5);
        rtl8168_ephy_write(ioaddr, 0x03, ephy_data);
        /******set EPHY registers for RTL8168CP end******/
    } else if (mcfg == MCFG_8168D_1) {
        /* set EPHY registers */
        rtl8168_ephy_write(ioaddr, 0x01, 0x7C7D);
        rtl8168_ephy_write(ioaddr, 0x02, 0x091F);
        rtl8168_ephy_write(ioaddr, 0x06, 0xB271);
        rtl8168_ephy_write(ioaddr, 0x07, 0xCE00);
    } else if (mcfg == MCFG_8168D_2) {
        /* set EPHY registers */
        rtl8168_ephy_write(ioaddr, 0x01, 0x6C7F);
        rtl8168_ephy_write(ioaddr, 0x02, 0x011F);
        rtl8168_ephy_write(ioaddr, 0x03, 0xC1B2);
        rtl8168_ephy_write(ioaddr, 0x1A, 0x0546);
        rtl8168_ephy_write(ioaddr, 0x1C, 0x80C4);
        rtl8168_ephy_write(ioaddr, 0x1D, 0x78E4);
        rtl8168_ephy_write(ioaddr, 0x0A, 0x8100);
    } else if (mcfg == MCFG_8168DP_2) {
        ephy_data = rtl8168_ephy_read(ioaddr, 0x0B);
        rtl8168_ephy_write(ioaddr, 0x0B, ephy_data|0x48);
        ephy_data = rtl8168_ephy_read(ioaddr, 0x19);
        ephy_data &= ~0x20;
        rtl8168_ephy_write(ioaddr, 0x19, ephy_data|0x50);
        ephy_data = rtl8168_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~0x100;
        rtl8168_ephy_write(ioaddr, 0x0C, ephy_data|0x20);
    } else if (mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2) {
        /* set EPHY registers */
        ephy_data = rtl8168_ephy_read(ioaddr, 0x00) & ~0x0200;
        ephy_data |= 0x0100;
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x00);
        ephy_data |= 0x0004;
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x06) & ~0x0002;
        ephy_data |= 0x0001;
        rtl8168_ephy_write(ioaddr, 0x06, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x06);
        ephy_data |= 0x0030;
        rtl8168_ephy_write(ioaddr, 0x06, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x2000;
        rtl8168_ephy_write(ioaddr, 0x07, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x00);
        ephy_data |= 0x0020;
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x03) & ~0x5800;
        ephy_data |= 0x2000;
        rtl8168_ephy_write(ioaddr, 0x03, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x03);
        ephy_data |= 0x0001;
        rtl8168_ephy_write(ioaddr, 0x03, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x01) & ~0x0800;
        ephy_data |= 0x1000;
        rtl8168_ephy_write(ioaddr, 0x01, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x4000;
        rtl8168_ephy_write(ioaddr, 0x07, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x1E);
        ephy_data |= 0x2000;
        rtl8168_ephy_write(ioaddr, 0x1E, ephy_data);

        rtl8168_ephy_write(ioaddr, 0x19, 0xFE6C);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x0A);
        ephy_data |= 0x0040;
        rtl8168_ephy_write(ioaddr, 0x0A, ephy_data);
    } else if (mcfg == MCFG_8168E_VL_1 || mcfg == MCFG_8168E_VL_2) {
        if (mcfg == MCFG_8168E_VL_1) {
            rtl8168_ephy_write(ioaddr, 0x06, 0xF020);
            rtl8168_ephy_write(ioaddr, 0x07, 0x01FF);
            rtl8168_ephy_write(ioaddr, 0x00, 0x5027);
            rtl8168_ephy_write(ioaddr, 0x01, 0x0003);
            rtl8168_ephy_write(ioaddr, 0x02, 0x2D16);
            rtl8168_ephy_write(ioaddr, 0x03, 0x6D49);
            rtl8168_ephy_write(ioaddr, 0x08, 0x0006);
            rtl8168_ephy_write(ioaddr, 0x0A, 0x00C8);
        }

        ephy_data = rtl8168_ephy_read(ioaddr, 0x09);
        ephy_data |= BIT_7;
        rtl8168_ephy_write(ioaddr, 0x09, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x19);
        ephy_data |= (BIT_2 | BIT_5 | BIT_9);
        rtl8168_ephy_write(ioaddr, 0x19, ephy_data);
    } else if (mcfg == MCFG_8168F_1 || mcfg == MCFG_8168F_2) {
        if (mcfg == MCFG_8168F_1) {
            ephy_data = rtl8168_ephy_read(ioaddr, 0x06);
            ephy_data |= BIT_5;
            ephy_data &= ~(BIT_7 | BIT_6);
            rtl8168_ephy_write(ioaddr, 0x06, ephy_data);

            ephy_data = rtl8168_ephy_read(ioaddr, 0x08);
            ephy_data |= BIT_1;
            ephy_data &= ~BIT_0;
            rtl8168_ephy_write(ioaddr, 0x08, ephy_data);
        }

        ephy_data = rtl8168_ephy_read(ioaddr, 0x09);
        ephy_data |= BIT_7;
        rtl8168_ephy_write(ioaddr, 0x09, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x19);
        ephy_data |= (BIT_2 | BIT_5 | BIT_9);
        rtl8168_ephy_write(ioaddr, 0x19, ephy_data);
    } else if (mcfg == MCFG_8411_1) {
        ephy_data = rtl8168_ephy_read(ioaddr, 0x06);
        ephy_data |= BIT_5;
        ephy_data &= ~(BIT_7 | BIT_6);
        rtl8168_ephy_write(ioaddr, 0x06, ephy_data);

        rtl8168_ephy_write(ioaddr, 0x0f, 0x5200);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x19);
        ephy_data |= (BIT_2 | BIT_5 | BIT_9);
        rtl8168_ephy_write(ioaddr, 0x19, ephy_data);
    } else if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22) {

        ephy_data = rtl8168_ephy_read(ioaddr, 0x00);
        ephy_data &= ~(BIT_3);
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);
        ephy_data = rtl8168_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        ephy_data |= (BIT_5 | BIT_11);
        rtl8168_ephy_write(ioaddr, 0x0C, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x1E);
        ephy_data &= ~(BIT_0);
        rtl8168_ephy_write(ioaddr, 0x1E, ephy_data);
    } else if (mcfg == CFG_METHOD_25) {
        ephy_data = rtl8168_ephy_read(ioaddr, 0x00);
        ephy_data |= BIT_3;
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);
        ephy_data = rtl8168_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        ephy_data |= BIT_10;
        rtl8168_ephy_write(ioaddr, 0x0C, ephy_data);

        rtl8168_ephy_write(ioaddr, 0x19, 0xFC00);
        rtl8168_ephy_write(ioaddr, 0x1E, 0x20EB);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x06);
        ephy_data |= BIT_4;
        rtl8168_ephy_write(ioaddr, 0x06, ephy_data);
    } else if (mcfg == MCFG_8411B) {
        ephy_data = rtl8168_ephy_read(ioaddr, 0x00);
        ephy_data |= BIT_3;
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);
        ephy_data = rtl8168_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        ephy_data |= BIT_10;
        rtl8168_ephy_write(ioaddr, 0x0C, ephy_data);

        rtl8168_ephy_write(ioaddr, 0x0F, 0x5200);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x19);
        ephy_data &= ~BIT_5;
        rtl8168_ephy_write(ioaddr, 0x19, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x1E);
        ephy_data |= BIT_13;
        rtl8168_ephy_write(ioaddr, 0x1E, ephy_data);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x06);
        ephy_data |= BIT_4;
        rtl8168_ephy_write(ioaddr, 0x06, ephy_data);
    } else if (mcfg == CFG_METHOD_23) {
        rtl8168_ephy_write(ioaddr, 0x00, 0x10AB);
        rtl8168_ephy_write(ioaddr, 0x06, 0xf030);
        rtl8168_ephy_write(ioaddr, 0x08, 0x2006);
        rtl8168_ephy_write(ioaddr, 0x0D, 0x1666);

        ephy_data = rtl8168_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        rtl8168_ephy_write(ioaddr, 0x0C, ephy_data);
    } else if (mcfg == CFG_METHOD_27) {
        ephy_data = rtl8168_ephy_read(ioaddr, 0x00);
        ephy_data |= BIT_3;
        rtl8168_ephy_write(ioaddr, 0x00, ephy_data);
        ephy_data = rtl8168_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        ephy_data |= BIT_9;
        rtl8168_ephy_write(ioaddr, 0x0C, ephy_data);

        rtl8168_ephy_write(ioaddr, 0x19, 0xFC00);
        rtl8168_ephy_write(ioaddr, 0x1E, 0x20EB);
    }
}

static int
rtl8168_check_hw_phy_mcu_code_ver(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int ram_code_ver_match = 0;
    u16 sw_ram_code_ver = 0xFFFF;
    u16 hw_ram_code_ver = 0;


    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168E_1;
        break;
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168E_VL_1;
        break;
    case MCFG_8168F_1:
    case MCFG_8168F_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168F_1;
        break;
    case MCFG_8411_1:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8411_1;
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_21;
        break;
    case CFG_METHOD_23:
    case CFG_METHOD_27:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_23;
        break;
    case CFG_METHOD_24:
    case CFG_METHOD_25:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_24;
        break;
    case MCFG_8411B:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8411B;
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B60);
        hw_ram_code_ver = ReadGMII16( 0x06);
        WriteGMII16( 0x1F, 0x0000);
        break;
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B30);
        hw_ram_code_ver = ReadGMII16( 0x06);
        WriteGMII16( 0x1F, 0x0000);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_27:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
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
rtl8168_write_hw_phy_mcu_code_ver(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    u16 sw_ram_code_ver = 0xFFFF;


    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168E_1;
        break;
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168E_VL_1;
        break;
    case MCFG_8168F_1:
    case MCFG_8168F_2:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8168F_1;
        break;
    case MCFG_8411_1:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8411_1;
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_21;
        break;
    case CFG_METHOD_23:
    case CFG_METHOD_27:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_23;
        break;
    case CFG_METHOD_24:
    case CFG_METHOD_25:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_24;
        break;
    case MCFG_8411B:
        sw_ram_code_ver = NIC_RAMCODE_VERSION_MCFG_8411B;
        break;
    }

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B60);
        WriteGMII16( 0x06, sw_ram_code_ver);
        WriteGMII16( 0x1F, 0x0000);
        break;
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B30);
        WriteGMII16( 0x06, sw_ram_code_ver);
        WriteGMII16( 0x1F, 0x0000);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_27:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x801E);
        WriteGMII16( 0x14, sw_ram_code_ver);
        WriteGMII16( 0x1F, 0x0000);
        break;
    }
}

static void
rtl8168_init_hw_phy_mcu(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    unsigned int gphy_val,i;
    u32 csi_tmp;

    if(rtl8168_check_hw_phy_mcu_code_ver(dev))
        return;

    if (mcfg == MCFG_8168E_1) {
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
            if (gphy_val == 0x000C)
                break;
        }
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x07);
            if ((gphy_val & BIT_5) == 0)
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
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x0151);
        WriteGMII16( 0x19, 0x6461);
        WriteGMII16( 0x15, 0x0152);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x0153);
        WriteGMII16( 0x19, 0x4540);
        WriteGMII16( 0x15, 0x0154);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x0155);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x15, 0x0156);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0157);
        WriteGMII16( 0x19, 0x6421);
        WriteGMII16( 0x15, 0x0158);
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x0159);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x015a);
        WriteGMII16( 0x19, 0x30fe);
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
        WriteGMII16( 0x19, 0x405e);
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
        WriteGMII16( 0x05, 0x8b88);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
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
        WriteGMII16( 0x06, 0x021f);
        WriteGMII16( 0x06, 0x9d02);
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
        WriteGMII16( 0x06, 0x830e);
        WriteGMII16( 0x06, 0x021b);
        WriteGMII16( 0x06, 0x67ad);
        WriteGMII16( 0x06, 0x2211);
        WriteGMII16( 0x06, 0xf622);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x2ba5);
        WriteGMII16( 0x06, 0x022a);
        WriteGMII16( 0x06, 0x2402);
        WriteGMII16( 0x06, 0x80c6);
        WriteGMII16( 0x06, 0x022a);
        WriteGMII16( 0x06, 0xf0ad);
        WriteGMII16( 0x06, 0x2511);
        WriteGMII16( 0x06, 0xf625);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x8226);
        WriteGMII16( 0x06, 0x0204);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x19cc);
        WriteGMII16( 0x06, 0x022b);
        WriteGMII16( 0x06, 0x5bfc);
        WriteGMII16( 0x06, 0x04ee);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0x0105);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b83);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x44e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x23ad);
        WriteGMII16( 0x06, 0x223b);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xbea0);
        WriteGMII16( 0x06, 0x0005);
        WriteGMII16( 0x06, 0x0228);
        WriteGMII16( 0x06, 0xdeae);
        WriteGMII16( 0x06, 0x42a0);
        WriteGMII16( 0x06, 0x0105);
        WriteGMII16( 0x06, 0x0228);
        WriteGMII16( 0x06, 0xf1ae);
        WriteGMII16( 0x06, 0x3aa0);
        WriteGMII16( 0x06, 0x0205);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x25ae);
        WriteGMII16( 0x06, 0x32a0);
        WriteGMII16( 0x06, 0x0305);
        WriteGMII16( 0x06, 0x0229);
        WriteGMII16( 0x06, 0x9aae);
        WriteGMII16( 0x06, 0x2aa0);
        WriteGMII16( 0x06, 0x0405);
        WriteGMII16( 0x06, 0x0229);
        WriteGMII16( 0x06, 0xaeae);
        WriteGMII16( 0x06, 0x22a0);
        WriteGMII16( 0x06, 0x0505);
        WriteGMII16( 0x06, 0x0229);
        WriteGMII16( 0x06, 0xd7ae);
        WriteGMII16( 0x06, 0x1aa0);
        WriteGMII16( 0x06, 0x0605);
        WriteGMII16( 0x06, 0x0229);
        WriteGMII16( 0x06, 0xfeae);
        WriteGMII16( 0x06, 0x12ee);
        WriteGMII16( 0x06, 0x8ac0);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8ac1);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8ac6);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x00ae);
        WriteGMII16( 0x06, 0x00fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0x022a);
        WriteGMII16( 0x06, 0x67e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x230d);
        WriteGMII16( 0x06, 0x0658);
        WriteGMII16( 0x06, 0x03a0);
        WriteGMII16( 0x06, 0x0202);
        WriteGMII16( 0x06, 0xae2d);
        WriteGMII16( 0x06, 0xa001);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x2da0);
        WriteGMII16( 0x06, 0x004d);
        WriteGMII16( 0x06, 0xe0e2);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0xe201);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x44e0);
        WriteGMII16( 0x06, 0x8ac2);
        WriteGMII16( 0x06, 0xe48a);
        WriteGMII16( 0x06, 0xc4e0);
        WriteGMII16( 0x06, 0x8ac3);
        WriteGMII16( 0x06, 0xe48a);
        WriteGMII16( 0x06, 0xc5ee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x03e0);
        WriteGMII16( 0x06, 0x8b83);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x3aee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x05ae);
        WriteGMII16( 0x06, 0x34e0);
        WriteGMII16( 0x06, 0x8ace);
        WriteGMII16( 0x06, 0xae03);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xcfe1);
        WriteGMII16( 0x06, 0x8ac2);
        WriteGMII16( 0x06, 0x4905);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xc4e1);
        WriteGMII16( 0x06, 0x8ac3);
        WriteGMII16( 0x06, 0x4905);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xc5ee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x2ab6);
        WriteGMII16( 0x06, 0xac20);
        WriteGMII16( 0x06, 0x1202);
        WriteGMII16( 0x06, 0x819b);
        WriteGMII16( 0x06, 0xac20);
        WriteGMII16( 0x06, 0x0cee);
        WriteGMII16( 0x06, 0x8ac1);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8ac6);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x02fc);
        WriteGMII16( 0x06, 0x04d0);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x590f);
        WriteGMII16( 0x06, 0x3902);
        WriteGMII16( 0x06, 0xaa04);
        WriteGMII16( 0x06, 0xd001);
        WriteGMII16( 0x06, 0xae02);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x04f9);
        WriteGMII16( 0x06, 0xfae2);
        WriteGMII16( 0x06, 0xe2d2);
        WriteGMII16( 0x06, 0xe3e2);
        WriteGMII16( 0x06, 0xd3f9);
        WriteGMII16( 0x06, 0x5af7);
        WriteGMII16( 0x06, 0xe6e2);
        WriteGMII16( 0x06, 0xd2e7);
        WriteGMII16( 0x06, 0xe2d3);
        WriteGMII16( 0x06, 0xe2e0);
        WriteGMII16( 0x06, 0x2ce3);
        WriteGMII16( 0x06, 0xe02d);
        WriteGMII16( 0x06, 0xf95b);
        WriteGMII16( 0x06, 0xe01e);
        WriteGMII16( 0x06, 0x30e6);
        WriteGMII16( 0x06, 0xe02c);
        WriteGMII16( 0x06, 0xe7e0);
        WriteGMII16( 0x06, 0x2de2);
        WriteGMII16( 0x06, 0xe2cc);
        WriteGMII16( 0x06, 0xe3e2);
        WriteGMII16( 0x06, 0xcdf9);
        WriteGMII16( 0x06, 0x5a0f);
        WriteGMII16( 0x06, 0x6a50);
        WriteGMII16( 0x06, 0xe6e2);
        WriteGMII16( 0x06, 0xcce7);
        WriteGMII16( 0x06, 0xe2cd);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x3ce1);
        WriteGMII16( 0x06, 0xe03d);
        WriteGMII16( 0x06, 0xef64);
        WriteGMII16( 0x06, 0xfde0);
        WriteGMII16( 0x06, 0xe2cc);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0xcd58);
        WriteGMII16( 0x06, 0x0f5a);
        WriteGMII16( 0x06, 0xf01e);
        WriteGMII16( 0x06, 0x02e4);
        WriteGMII16( 0x06, 0xe2cc);
        WriteGMII16( 0x06, 0xe5e2);
        WriteGMII16( 0x06, 0xcdfd);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x2ce1);
        WriteGMII16( 0x06, 0xe02d);
        WriteGMII16( 0x06, 0x59e0);
        WriteGMII16( 0x06, 0x5b1f);
        WriteGMII16( 0x06, 0x1e13);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0x2ce5);
        WriteGMII16( 0x06, 0xe02d);
        WriteGMII16( 0x06, 0xfde0);
        WriteGMII16( 0x06, 0xe2d2);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0xd358);
        WriteGMII16( 0x06, 0xf75a);
        WriteGMII16( 0x06, 0x081e);
        WriteGMII16( 0x06, 0x02e4);
        WriteGMII16( 0x06, 0xe2d2);
        WriteGMII16( 0x06, 0xe5e2);
        WriteGMII16( 0x06, 0xd3ef);
        WriteGMII16( 0x06, 0x46fe);
        WriteGMII16( 0x06, 0xfd04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x2358);
        WriteGMII16( 0x06, 0xc4e1);
        WriteGMII16( 0x06, 0x8b6e);
        WriteGMII16( 0x06, 0x1f10);
        WriteGMII16( 0x06, 0x9e58);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x6ead);
        WriteGMII16( 0x06, 0x2222);
        WriteGMII16( 0x06, 0xac27);
        WriteGMII16( 0x06, 0x55ac);
        WriteGMII16( 0x06, 0x2602);
        WriteGMII16( 0x06, 0xae1a);
        WriteGMII16( 0x06, 0xd106);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xba02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd107);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xbd02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd107);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xc002);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xae30);
        WriteGMII16( 0x06, 0xd103);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xc302);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xc602);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xca02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd10f);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xba02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xbd02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xc002);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xc302);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd011);
        WriteGMII16( 0x06, 0x022b);
        WriteGMII16( 0x06, 0xfb59);
        WriteGMII16( 0x06, 0x03ef);
        WriteGMII16( 0x06, 0x01d1);
        WriteGMII16( 0x06, 0x00a0);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0xc602);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xd111);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x020c);
        WriteGMII16( 0x06, 0x11ad);
        WriteGMII16( 0x06, 0x2102);
        WriteGMII16( 0x06, 0x0c12);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xca02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xaec8);
        WriteGMII16( 0x06, 0x70e4);
        WriteGMII16( 0x06, 0x2602);
        WriteGMII16( 0x06, 0x82d1);
        WriteGMII16( 0x06, 0x05f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0xe2fe);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0xffad);
        WriteGMII16( 0x06, 0x2d1a);
        WriteGMII16( 0x06, 0xe0e1);
        WriteGMII16( 0x06, 0x4ee1);
        WriteGMII16( 0x06, 0xe14f);
        WriteGMII16( 0x06, 0xac2d);
        WriteGMII16( 0x06, 0x22f6);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x033b);
        WriteGMII16( 0x06, 0xf703);
        WriteGMII16( 0x06, 0xf706);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0x4402);
        WriteGMII16( 0x06, 0x2d21);
        WriteGMII16( 0x06, 0xae11);
        WriteGMII16( 0x06, 0xe0e1);
        WriteGMII16( 0x06, 0x4ee1);
        WriteGMII16( 0x06, 0xe14f);
        WriteGMII16( 0x06, 0xad2d);
        WriteGMII16( 0x06, 0x08bf);
        WriteGMII16( 0x06, 0x844f);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x21f6);
        WriteGMII16( 0x06, 0x06ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0x4502);
        WriteGMII16( 0x06, 0x83a2);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0xe001);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x1fd1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x843b);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1e0);
        WriteGMII16( 0x06, 0xe020);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x21ad);
        WriteGMII16( 0x06, 0x200e);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0x3b02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xbf3b);
        WriteGMII16( 0x06, 0x9602);
        WriteGMII16( 0x06, 0x2d21);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x204c);
        WriteGMII16( 0x06, 0xd200);
        WriteGMII16( 0x06, 0xe0e2);
        WriteGMII16( 0x06, 0x0058);
        WriteGMII16( 0x06, 0x010c);
        WriteGMII16( 0x06, 0x021e);
        WriteGMII16( 0x06, 0x20e0);
        WriteGMII16( 0x06, 0xe000);
        WriteGMII16( 0x06, 0x5810);
        WriteGMII16( 0x06, 0x1e20);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x3658);
        WriteGMII16( 0x06, 0x031e);
        WriteGMII16( 0x06, 0x20e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x2358);
        WriteGMII16( 0x06, 0xe01e);
        WriteGMII16( 0x06, 0x20e0);
        WriteGMII16( 0x06, 0x8b64);
        WriteGMII16( 0x06, 0x1f02);
        WriteGMII16( 0x06, 0x9e22);
        WriteGMII16( 0x06, 0xe68b);
        WriteGMII16( 0x06, 0x64ad);
        WriteGMII16( 0x06, 0x3214);
        WriteGMII16( 0x06, 0xad34);
        WriteGMII16( 0x06, 0x11ef);
        WriteGMII16( 0x06, 0x0258);
        WriteGMII16( 0x06, 0x039e);
        WriteGMII16( 0x06, 0x07ad);
        WriteGMII16( 0x06, 0x3508);
        WriteGMII16( 0x06, 0x5ac0);
        WriteGMII16( 0x06, 0x9f04);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xae02);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0x3e02);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfbe0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x22e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x23e2);
        WriteGMII16( 0x06, 0xe036);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x375a);
        WriteGMII16( 0x06, 0xc40d);
        WriteGMII16( 0x06, 0x0158);
        WriteGMII16( 0x06, 0x021e);
        WriteGMII16( 0x06, 0x20e3);
        WriteGMII16( 0x06, 0x8ae7);
        WriteGMII16( 0x06, 0xac31);
        WriteGMII16( 0x06, 0x60ac);
        WriteGMII16( 0x06, 0x3a08);
        WriteGMII16( 0x06, 0xac3e);
        WriteGMII16( 0x06, 0x26ae);
        WriteGMII16( 0x06, 0x67af);
        WriteGMII16( 0x06, 0x8437);
        WriteGMII16( 0x06, 0xad37);
        WriteGMII16( 0x06, 0x61e0);
        WriteGMII16( 0x06, 0x8ae8);
        WriteGMII16( 0x06, 0x10e4);
        WriteGMII16( 0x06, 0x8ae8);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xe91b);
        WriteGMII16( 0x06, 0x109e);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x51d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x8441);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0xc1ee);
        WriteGMII16( 0x06, 0x8ae8);
        WriteGMII16( 0x06, 0x00ae);
        WriteGMII16( 0x06, 0x43ad);
        WriteGMII16( 0x06, 0x3627);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xeee1);
        WriteGMII16( 0x06, 0x8aef);
        WriteGMII16( 0x06, 0xef74);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xeae1);
        WriteGMII16( 0x06, 0x8aeb);
        WriteGMII16( 0x06, 0x1b74);
        WriteGMII16( 0x06, 0x9e2e);
        WriteGMII16( 0x06, 0x14e4);
        WriteGMII16( 0x06, 0x8aea);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xebef);
        WriteGMII16( 0x06, 0x74e0);
        WriteGMII16( 0x06, 0x8aee);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xef1b);
        WriteGMII16( 0x06, 0x479e);
        WriteGMII16( 0x06, 0x0fae);
        WriteGMII16( 0x06, 0x19ee);
        WriteGMII16( 0x06, 0x8aea);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8aeb);
        WriteGMII16( 0x06, 0x00ae);
        WriteGMII16( 0x06, 0x0fac);
        WriteGMII16( 0x06, 0x390c);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0x4102);
        WriteGMII16( 0x06, 0x2dc1);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xe800);
        WriteGMII16( 0x06, 0xe68a);
        WriteGMII16( 0x06, 0xe7ff);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x0400);
        WriteGMII16( 0x06, 0xe234);
        WriteGMII16( 0x06, 0xcce2);
        WriteGMII16( 0x06, 0x0088);
        WriteGMII16( 0x06, 0xe200);
        WriteGMII16( 0x06, 0xa725);
        WriteGMII16( 0x06, 0xe50a);
        WriteGMII16( 0x06, 0x1de5);
        WriteGMII16( 0x06, 0x0a2c);
        WriteGMII16( 0x06, 0xe50a);
        WriteGMII16( 0x06, 0x6de5);
        WriteGMII16( 0x06, 0x0a1d);
        WriteGMII16( 0x06, 0xe50a);
        WriteGMII16( 0x06, 0x1ce5);
        WriteGMII16( 0x06, 0x0a2d);
        WriteGMII16( 0x06, 0xa755);
        WriteGMII16( 0x05, 0x8b64);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8b94);
        WriteGMII16( 0x06, 0x82cd);
        WriteGMII16( 0x05, 0x8b85);
        WriteGMII16( 0x06, 0x2000);
        WriteGMII16( 0x05, 0x8aee);
        WriteGMII16( 0x06, 0x03b8);
        WriteGMII16( 0x05, 0x8ae8);
        WriteGMII16( 0x06, 0x0002);
        gphy_val = ReadGMII16( 0x01);
        gphy_val |= BIT_0;
        WriteGMII16( 0x01, gphy_val);
        gphy_val = ReadGMII16( 0x00);
        gphy_val |= BIT_0;
        WriteGMII16( 0x00, gphy_val);
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
        gphy_val = ReadGMII16( 0x17);
        gphy_val &= ~(BIT_0);
        if ((pdev->subsystem_vendor == 0x144d &&
             pdev->subsystem_device == 0xc098) ||
            (pdev->subsystem_vendor == 0x144d &&
             pdev->subsystem_device == 0xc0b1)) {
            gphy_val &= ~(BIT_2);
        }
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0028);
        WriteGMII16( 0x15, 0x0010);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0041);
        WriteGMII16( 0x15, 0x0802);
        WriteGMII16( 0x16, 0x2185);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == MCFG_8168E_2) {
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
            if ((gphy_val & BIT_5) == 0)
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
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x0151);
        WriteGMII16( 0x19, 0x6461);
        WriteGMII16( 0x15, 0x0152);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x0153);
        WriteGMII16( 0x19, 0x4540);
        WriteGMII16( 0x15, 0x0154);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x0155);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x15, 0x0156);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0157);
        WriteGMII16( 0x19, 0x6421);
        WriteGMII16( 0x15, 0x0158);
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x0159);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x015a);
        WriteGMII16( 0x19, 0x30fe);
        WriteGMII16( 0x15, 0x02e7);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0329);
        WriteGMII16( 0x19, 0x7c00);
        WriteGMII16( 0x15, 0x0382);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x03bd);
        WriteGMII16( 0x19, 0x405e);
        WriteGMII16( 0x15, 0x03c9);
        WriteGMII16( 0x19, 0x7c00);
        WriteGMII16( 0x15, 0x03e3);
        WriteGMII16( 0x19, 0x7c00);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x48f7);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xa080);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xf602);
        WriteGMII16( 0x06, 0x0200);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x8c02);
        WriteGMII16( 0x06, 0x0224);
        WriteGMII16( 0x06, 0x0202);
        WriteGMII16( 0x06, 0x3402);
        WriteGMII16( 0x06, 0x027f);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0xa202);
        WriteGMII16( 0x06, 0x80bb);
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
        WriteGMII16( 0x06, 0xaebb);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xe600);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xee03);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xefb8);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xe902);
        WriteGMII16( 0x06, 0xee8b);
        WriteGMII16( 0x06, 0x8520);
        WriteGMII16( 0x06, 0xee8b);
        WriteGMII16( 0x06, 0x8701);
        WriteGMII16( 0x06, 0xd481);
        WriteGMII16( 0x06, 0x31e4);
        WriteGMII16( 0x06, 0x8b94);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x95bf);
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
        WriteGMII16( 0x06, 0x4104);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b89);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x0dee);
        WriteGMII16( 0x06, 0x8b89);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x82ed);
        WriteGMII16( 0x06, 0x021f);
        WriteGMII16( 0x06, 0x4102);
        WriteGMII16( 0x06, 0x2812);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x10ee);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x139d);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0xcf02);
        WriteGMII16( 0x06, 0x1f99);
        WriteGMII16( 0x06, 0x0227);
        WriteGMII16( 0x06, 0xeafc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2014);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x8100);
        WriteGMII16( 0x06, 0x021b);
        WriteGMII16( 0x06, 0xf402);
        WriteGMII16( 0x06, 0x2c9c);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x7202);
        WriteGMII16( 0x06, 0x843c);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x11f6);
        WriteGMII16( 0x06, 0x22e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x022c);
        WriteGMII16( 0x06, 0x4602);
        WriteGMII16( 0x06, 0x2ac5);
        WriteGMII16( 0x06, 0x0229);
        WriteGMII16( 0x06, 0x2002);
        WriteGMII16( 0x06, 0x2b91);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x11f6);
        WriteGMII16( 0x06, 0x25e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0x5a02);
        WriteGMII16( 0x06, 0x043a);
        WriteGMII16( 0x06, 0x021a);
        WriteGMII16( 0x06, 0x5902);
        WriteGMII16( 0x06, 0x2bfc);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0xe001);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x1fd1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x84eb);
        WriteGMII16( 0x06, 0x022f);
        WriteGMII16( 0x06, 0x50e0);
        WriteGMII16( 0x06, 0xe020);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x21ad);
        WriteGMII16( 0x06, 0x200e);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0xeb02);
        WriteGMII16( 0x06, 0x2f50);
        WriteGMII16( 0x06, 0xbf3d);
        WriteGMII16( 0x06, 0x3902);
        WriteGMII16( 0x06, 0x2eb0);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefc);
        WriteGMII16( 0x06, 0x0402);
        WriteGMII16( 0x06, 0x8135);
        WriteGMII16( 0x06, 0x05f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0xe2fe);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0xffad);
        WriteGMII16( 0x06, 0x2d1a);
        WriteGMII16( 0x06, 0xe0e1);
        WriteGMII16( 0x06, 0x4ee1);
        WriteGMII16( 0x06, 0xe14f);
        WriteGMII16( 0x06, 0xac2d);
        WriteGMII16( 0x06, 0x22f6);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x0336);
        WriteGMII16( 0x06, 0xf703);
        WriteGMII16( 0x06, 0xf706);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0xd502);
        WriteGMII16( 0x06, 0x2eb0);
        WriteGMII16( 0x06, 0xae11);
        WriteGMII16( 0x06, 0xe0e1);
        WriteGMII16( 0x06, 0x4ee1);
        WriteGMII16( 0x06, 0xe14f);
        WriteGMII16( 0x06, 0xad2d);
        WriteGMII16( 0x06, 0x08bf);
        WriteGMII16( 0x06, 0x84e0);
        WriteGMII16( 0x06, 0x022e);
        WriteGMII16( 0x06, 0xb0f6);
        WriteGMII16( 0x06, 0x06ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x4cd2);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xe200);
        WriteGMII16( 0x06, 0x5801);
        WriteGMII16( 0x06, 0x0c02);
        WriteGMII16( 0x06, 0x1e20);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x0058);
        WriteGMII16( 0x06, 0x101e);
        WriteGMII16( 0x06, 0x20e0);
        WriteGMII16( 0x06, 0xe036);
        WriteGMII16( 0x06, 0x5803);
        WriteGMII16( 0x06, 0x1e20);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x22e1);
        WriteGMII16( 0x06, 0xe023);
        WriteGMII16( 0x06, 0x58e0);
        WriteGMII16( 0x06, 0x1e20);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xe61f);
        WriteGMII16( 0x06, 0x029e);
        WriteGMII16( 0x06, 0x22e6);
        WriteGMII16( 0x06, 0x8ae6);
        WriteGMII16( 0x06, 0xad32);
        WriteGMII16( 0x06, 0x14ad);
        WriteGMII16( 0x06, 0x3411);
        WriteGMII16( 0x06, 0xef02);
        WriteGMII16( 0x06, 0x5803);
        WriteGMII16( 0x06, 0x9e07);
        WriteGMII16( 0x06, 0xad35);
        WriteGMII16( 0x06, 0x085a);
        WriteGMII16( 0x06, 0xc09f);
        WriteGMII16( 0x06, 0x04d1);
        WriteGMII16( 0x06, 0x01ae);
        WriteGMII16( 0x06, 0x02d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x84f1);
        WriteGMII16( 0x06, 0x022f);
        WriteGMII16( 0x06, 0x50ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ac);
        WriteGMII16( 0x06, 0x260e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ac);
        WriteGMII16( 0x06, 0x2108);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ac);
        WriteGMII16( 0x06, 0x2402);
        WriteGMII16( 0x06, 0xae6b);
        WriteGMII16( 0x06, 0xeee0);
        WriteGMII16( 0x06, 0xea00);
        WriteGMII16( 0x06, 0xeee0);
        WriteGMII16( 0x06, 0xeb00);
        WriteGMII16( 0x06, 0xe2e0);
        WriteGMII16( 0x06, 0x7ce3);
        WriteGMII16( 0x06, 0xe07d);
        WriteGMII16( 0x06, 0xa511);
        WriteGMII16( 0x06, 0x1115);
        WriteGMII16( 0x06, 0xd260);
        WriteGMII16( 0x06, 0xd666);
        WriteGMII16( 0x06, 0x6602);
        WriteGMII16( 0x06, 0x07f9);
        WriteGMII16( 0x06, 0xd2a0);
        WriteGMII16( 0x06, 0xd6aa);
        WriteGMII16( 0x06, 0xaa02);
        WriteGMII16( 0x06, 0x07f9);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0x55ae);
        WriteGMII16( 0x06, 0x44a5);
        WriteGMII16( 0x06, 0x6666);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x38a5);
        WriteGMII16( 0x06, 0xaaaa);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x32ee);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0x04ee);
        WriteGMII16( 0x06, 0xe0eb);
        WriteGMII16( 0x06, 0x06e2);
        WriteGMII16( 0x06, 0xe07c);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x7de0);
        WriteGMII16( 0x06, 0xe038);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x39ad);
        WriteGMII16( 0x06, 0x2e21);
        WriteGMII16( 0x06, 0xad3f);
        WriteGMII16( 0x06, 0x13e0);
        WriteGMII16( 0x06, 0xe414);
        WriteGMII16( 0x06, 0xe1e4);
        WriteGMII16( 0x06, 0x1568);
        WriteGMII16( 0x06, 0x80e4);
        WriteGMII16( 0x06, 0xe414);
        WriteGMII16( 0x06, 0xe5e4);
        WriteGMII16( 0x06, 0x1502);
        WriteGMII16( 0x06, 0x8255);
        WriteGMII16( 0x06, 0xae0b);
        WriteGMII16( 0x06, 0xac3e);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x0602);
        WriteGMII16( 0x06, 0x827f);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xa9fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x2ee0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x21f3);
        WriteGMII16( 0x06, 0xf728);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2105);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0xf8f7);
        WriteGMII16( 0x06, 0x29e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x82e4);
        WriteGMII16( 0x06, 0xf72a);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x2efc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2603);
        WriteGMII16( 0x06, 0x0221);
        WriteGMII16( 0x06, 0x34e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad21);
        WriteGMII16( 0x06, 0x09e0);
        WriteGMII16( 0x06, 0x8b2e);
        WriteGMII16( 0x06, 0xac20);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x834b);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x2409);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x2eac);
        WriteGMII16( 0x06, 0x2103);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0x30fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x2ee0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x08e0);
        WriteGMII16( 0x06, 0x85d2);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x02f6);
        WriteGMII16( 0x06, 0x28e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad21);
        WriteGMII16( 0x06, 0x0ae0);
        WriteGMII16( 0x06, 0x860a);
        WriteGMII16( 0x06, 0xf627);
        WriteGMII16( 0x06, 0xa005);
        WriteGMII16( 0x06, 0x02f6);
        WriteGMII16( 0x06, 0x29e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x08e0);
        WriteGMII16( 0x06, 0x8aed);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x02f6);
        WriteGMII16( 0x06, 0x2ae5);
        WriteGMII16( 0x06, 0x8b2e);
        WriteGMII16( 0x06, 0xa100);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x2111);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xed00);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xec00);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x243a);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0xeae1);
        WriteGMII16( 0x06, 0xe0eb);
        WriteGMII16( 0x06, 0x58f8);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0xeae5);
        WriteGMII16( 0x06, 0xe0eb);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x7ce1);
        WriteGMII16( 0x06, 0xe07d);
        WriteGMII16( 0x06, 0x5c00);
        WriteGMII16( 0x06, 0xff3c);
        WriteGMII16( 0x06, 0x001e);
        WriteGMII16( 0x06, 0xab1c);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x4ce1);
        WriteGMII16( 0x06, 0xe04d);
        WriteGMII16( 0x06, 0x58c1);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0x4ce5);
        WriteGMII16( 0x06, 0xe04d);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0xeee1);
        WriteGMII16( 0x06, 0xe0ef);
        WriteGMII16( 0x06, 0x693c);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0xeee5);
        WriteGMII16( 0x06, 0xe0ef);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x12e0);
        WriteGMII16( 0x06, 0xe0ee);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0xef59);
        WriteGMII16( 0x06, 0xc3e4);
        WriteGMII16( 0x06, 0xe0ee);
        WriteGMII16( 0x06, 0xe5e0);
        WriteGMII16( 0x06, 0xefee);
        WriteGMII16( 0x06, 0x8aed);
        WriteGMII16( 0x06, 0x01fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ac);
        WriteGMII16( 0x06, 0x2505);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0x5cae);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x2516);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69fa);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x0aa0);
        WriteGMII16( 0x06, 0x0019);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x0be1);
        WriteGMII16( 0x06, 0x8b33);
        WriteGMII16( 0x06, 0x1b10);
        WriteGMII16( 0x06, 0x9e04);
        WriteGMII16( 0x06, 0xaa02);
        WriteGMII16( 0x06, 0xae06);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x0a01);
        WriteGMII16( 0x06, 0xaee6);
        WriteGMII16( 0x06, 0x0224);
        WriteGMII16( 0x06, 0x1eae);
        WriteGMII16( 0x06, 0x14a0);
        WriteGMII16( 0x06, 0x0114);
        WriteGMII16( 0x06, 0x0224);
        WriteGMII16( 0x06, 0x26bf);
        WriteGMII16( 0x06, 0x266d);
        WriteGMII16( 0x06, 0x022e);
        WriteGMII16( 0x06, 0xb0ee);
        WriteGMII16( 0x06, 0x860b);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x860a);
        WriteGMII16( 0x06, 0x02af);
        WriteGMII16( 0x06, 0x8435);
        WriteGMII16( 0x06, 0xa002);
        WriteGMII16( 0x06, 0x52ee);
        WriteGMII16( 0x06, 0x8604);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8605);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0x860b);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x321b);
        WriteGMII16( 0x06, 0x109e);
        WriteGMII16( 0x06, 0x04aa);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xcbee);
        WriteGMII16( 0x06, 0x860b);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x243a);
        WriteGMII16( 0x06, 0xe286);
        WriteGMII16( 0x06, 0x04e3);
        WriteGMII16( 0x06, 0x8605);
        WriteGMII16( 0x06, 0xef65);
        WriteGMII16( 0x06, 0xe286);
        WriteGMII16( 0x06, 0x06e3);
        WriteGMII16( 0x06, 0x8607);
        WriteGMII16( 0x06, 0x1b56);
        WriteGMII16( 0x06, 0xaa0e);
        WriteGMII16( 0x06, 0xef56);
        WriteGMII16( 0x06, 0xe686);
        WriteGMII16( 0x06, 0x06e7);
        WriteGMII16( 0x06, 0x8607);
        WriteGMII16( 0x06, 0xe286);
        WriteGMII16( 0x06, 0x09e6);
        WriteGMII16( 0x06, 0x8608);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x09a0);
        WriteGMII16( 0x06, 0x0007);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x0a03);
        WriteGMII16( 0x06, 0xaf83);
        WriteGMII16( 0x06, 0x6202);
        WriteGMII16( 0x06, 0x248e);
        WriteGMII16( 0x06, 0x0224);
        WriteGMII16( 0x06, 0x26ae);
        WriteGMII16( 0x06, 0x48a0);
        WriteGMII16( 0x06, 0x0321);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x08e1);
        WriteGMII16( 0x06, 0x8609);
        WriteGMII16( 0x06, 0x1b01);
        WriteGMII16( 0x06, 0x9e0c);
        WriteGMII16( 0x06, 0xaa05);
        WriteGMII16( 0x06, 0x0224);
        WriteGMII16( 0x06, 0x9dae);
        WriteGMII16( 0x06, 0xe702);
        WriteGMII16( 0x06, 0x248e);
        WriteGMII16( 0x06, 0xaee2);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x0a04);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x0b00);
        WriteGMII16( 0x06, 0xaf83);
        WriteGMII16( 0x06, 0x62a0);
        WriteGMII16( 0x06, 0x0415);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x0be1);
        WriteGMII16( 0x06, 0x8b34);
        WriteGMII16( 0x06, 0x1b10);
        WriteGMII16( 0x06, 0x9e05);
        WriteGMII16( 0x06, 0xaa03);
        WriteGMII16( 0x06, 0xaf83);
        WriteGMII16( 0x06, 0x7cee);
        WriteGMII16( 0x06, 0x860a);
        WriteGMII16( 0x06, 0x05ae);
        WriteGMII16( 0x06, 0x0ca0);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0xae07);
        WriteGMII16( 0x06, 0x0223);
        WriteGMII16( 0x06, 0x09ee);
        WriteGMII16( 0x06, 0x860a);
        WriteGMII16( 0x06, 0x00fe);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfbe0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x22e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x23e2);
        WriteGMII16( 0x06, 0xe036);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x375a);
        WriteGMII16( 0x06, 0xc40d);
        WriteGMII16( 0x06, 0x0158);
        WriteGMII16( 0x06, 0x021e);
        WriteGMII16( 0x06, 0x20e3);
        WriteGMII16( 0x06, 0x8ae7);
        WriteGMII16( 0x06, 0xac31);
        WriteGMII16( 0x06, 0x60ac);
        WriteGMII16( 0x06, 0x3a08);
        WriteGMII16( 0x06, 0xac3e);
        WriteGMII16( 0x06, 0x26ae);
        WriteGMII16( 0x06, 0x67af);
        WriteGMII16( 0x06, 0x84d1);
        WriteGMII16( 0x06, 0xad37);
        WriteGMII16( 0x06, 0x61e0);
        WriteGMII16( 0x06, 0x8ae8);
        WriteGMII16( 0x06, 0x10e4);
        WriteGMII16( 0x06, 0x8ae8);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xe91b);
        WriteGMII16( 0x06, 0x109e);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x51d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x84ee);
        WriteGMII16( 0x06, 0x022f);
        WriteGMII16( 0x06, 0x50ee);
        WriteGMII16( 0x06, 0x8ae8);
        WriteGMII16( 0x06, 0x00ae);
        WriteGMII16( 0x06, 0x43ad);
        WriteGMII16( 0x06, 0x3627);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xeee1);
        WriteGMII16( 0x06, 0x8aef);
        WriteGMII16( 0x06, 0xef74);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xeae1);
        WriteGMII16( 0x06, 0x8aeb);
        WriteGMII16( 0x06, 0x1b74);
        WriteGMII16( 0x06, 0x9e2e);
        WriteGMII16( 0x06, 0x14e4);
        WriteGMII16( 0x06, 0x8aea);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xebef);
        WriteGMII16( 0x06, 0x74e0);
        WriteGMII16( 0x06, 0x8aee);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xef1b);
        WriteGMII16( 0x06, 0x479e);
        WriteGMII16( 0x06, 0x0fae);
        WriteGMII16( 0x06, 0x19ee);
        WriteGMII16( 0x06, 0x8aea);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8aeb);
        WriteGMII16( 0x06, 0x00ae);
        WriteGMII16( 0x06, 0x0fac);
        WriteGMII16( 0x06, 0x390c);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf84);
        WriteGMII16( 0x06, 0xee02);
        WriteGMII16( 0x06, 0x2f50);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xe800);
        WriteGMII16( 0x06, 0xe68a);
        WriteGMII16( 0x06, 0xe7ff);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04a7);
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
        WriteGMII16( 0x06, 0xe234);
        WriteGMII16( 0x06, 0x88e2);
        WriteGMII16( 0x06, 0x00cc);
        WriteGMII16( 0x06, 0xe200);
        gphy_val = ReadGMII16( 0x01);
        gphy_val |= BIT_0;
        WriteGMII16( 0x01, gphy_val);
        gphy_val = ReadGMII16( 0x00);
        gphy_val |= BIT_0;
        WriteGMII16( 0x00, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x17, 0x2179);
        WriteGMII16( 0x1f, 0x0001);
        WriteGMII16( 0x10, 0xf274);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0042);
        WriteGMII16( 0x15, 0x0f00);
        WriteGMII16( 0x15, 0x0f00);
        WriteGMII16( 0x16, 0x7408);
        WriteGMII16( 0x15, 0x0e00);
        WriteGMII16( 0x15, 0x0f00);
        WriteGMII16( 0x15, 0x0f01);
        WriteGMII16( 0x16, 0x4000);
        WriteGMII16( 0x15, 0x0e01);
        WriteGMII16( 0x15, 0x0f01);
        WriteGMII16( 0x15, 0x0f02);
        WriteGMII16( 0x16, 0x9400);
        WriteGMII16( 0x15, 0x0e02);
        WriteGMII16( 0x15, 0x0f02);
        WriteGMII16( 0x15, 0x0f03);
        WriteGMII16( 0x16, 0x7408);
        WriteGMII16( 0x15, 0x0e03);
        WriteGMII16( 0x15, 0x0f03);
        WriteGMII16( 0x15, 0x0f04);
        WriteGMII16( 0x16, 0x4008);
        WriteGMII16( 0x15, 0x0e04);
        WriteGMII16( 0x15, 0x0f04);
        WriteGMII16( 0x15, 0x0f05);
        WriteGMII16( 0x16, 0x9400);
        WriteGMII16( 0x15, 0x0e05);
        WriteGMII16( 0x15, 0x0f05);
        WriteGMII16( 0x15, 0x0f06);
        WriteGMII16( 0x16, 0x0803);
        WriteGMII16( 0x15, 0x0e06);
        WriteGMII16( 0x15, 0x0f06);
        WriteGMII16( 0x15, 0x0d00);
        WriteGMII16( 0x15, 0x0100);
        WriteGMII16( 0x1f, 0x0001);
        WriteGMII16( 0x10, 0xf074);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x17, 0x2149);
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        gphy_val &= ~(BIT_0 | BIT_2);
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        gphy_val |= BIT_14;
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1e, 0x0020);
        gphy_val = ReadGMII16( 0x1b);
        gphy_val |= BIT_7;
        WriteGMII16( 0x1b, gphy_val);
        WriteGMII16( 0x1e, 0x0041);
        WriteGMII16( 0x15, 0x0e02);
        WriteGMII16( 0x1e, 0x0028);
        gphy_val = ReadGMII16( 0x19);
        gphy_val |= BIT_15;
        WriteGMII16( 0x19, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == MCFG_8168E_VL_1) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x1800);
        gphy_val = ReadGMII16( 0x15);
        gphy_val &= ~(BIT_12);
        WriteGMII16( 0x15, gphy_val);
        mdelay(20);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        if ((gphy_val & BIT_11) == 0x0000) {
            gphy_val |= BIT_0;
            WriteGMII16( 0x17, gphy_val);
            for (i = 0; i < 200; i++) {
                IODelay(100);
                gphy_val = ReadGMII16( 0x17);
                if (gphy_val & BIT_11)
                    break;
            }
        }
        gphy_val = ReadGMII16( 0x17);
        gphy_val |= BIT_0;
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1E, 0x002C);
        WriteGMII16( 0x1B, 0x5000);
        WriteGMII16( 0x1E, 0x002d);
        WriteGMII16( 0x19, 0x0004);
        WriteGMII16( 0x1f, 0x0002);
        WriteGMII16( 0x1f, 0x0000);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x1E);
            if ((gphy_val & 0x03FF) == 0x0014)
                break;
        }
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x07);
            if ((gphy_val & BIT_5) == 0)
                break;
        }
        gphy_val = ReadGMII16( 0x07);
        if (gphy_val & BIT_5) {
            WriteGMII16( 0x1f, 0x0004);
            WriteGMII16( 0x1f, 0x0007);
            WriteGMII16( 0x1e, 0x00a1);
            WriteGMII16( 0x17, 0x1000);
            WriteGMII16( 0x17, 0x0000);
            WriteGMII16( 0x17, 0x2000);
            WriteGMII16( 0x1e, 0x002f);
            WriteGMII16( 0x18, 0x9bfb);
            WriteGMII16( 0x1f, 0x0005);
            WriteGMII16( 0x07, 0x0000);
            WriteGMII16( 0x1f, 0x0002);
            WriteGMII16( 0x1f, 0x0000);
        }
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        gphy_val = ReadGMII16( 0x00);
        gphy_val &= ~(BIT_7);
        WriteGMII16( 0x00, gphy_val);
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0307);
        WriteGMII16( 0x15, 0x0000);
        WriteGMII16( 0x19, 0x407d);
        WriteGMII16( 0x15, 0x0001);
        WriteGMII16( 0x19, 0x440f);
        WriteGMII16( 0x15, 0x0002);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0003);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x0004);
        WriteGMII16( 0x19, 0xc4d5);
        WriteGMII16( 0x15, 0x0005);
        WriteGMII16( 0x19, 0x00ff);
        WriteGMII16( 0x15, 0x0006);
        WriteGMII16( 0x19, 0x74f0);
        WriteGMII16( 0x15, 0x0007);
        WriteGMII16( 0x19, 0x4880);
        WriteGMII16( 0x15, 0x0008);
        WriteGMII16( 0x19, 0x4c00);
        WriteGMII16( 0x15, 0x0009);
        WriteGMII16( 0x19, 0x4800);
        WriteGMII16( 0x15, 0x000a);
        WriteGMII16( 0x19, 0x5000);
        WriteGMII16( 0x15, 0x000b);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x000c);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x000d);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x15, 0x000e);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x000f);
        WriteGMII16( 0x19, 0x7010);
        WriteGMII16( 0x15, 0x0010);
        WriteGMII16( 0x19, 0x6804);
        WriteGMII16( 0x15, 0x0011);
        WriteGMII16( 0x19, 0x64a0);
        WriteGMII16( 0x15, 0x0012);
        WriteGMII16( 0x19, 0x63da);
        WriteGMII16( 0x15, 0x0013);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x0014);
        WriteGMII16( 0x19, 0x6f05);
        WriteGMII16( 0x15, 0x0015);
        WriteGMII16( 0x19, 0x5420);
        WriteGMII16( 0x15, 0x0016);
        WriteGMII16( 0x19, 0x58ce);
        WriteGMII16( 0x15, 0x0017);
        WriteGMII16( 0x19, 0x5cf3);
        WriteGMII16( 0x15, 0x0018);
        WriteGMII16( 0x19, 0xb600);
        WriteGMII16( 0x15, 0x0019);
        WriteGMII16( 0x19, 0xc659);
        WriteGMII16( 0x15, 0x001a);
        WriteGMII16( 0x19, 0x0018);
        WriteGMII16( 0x15, 0x001b);
        WriteGMII16( 0x19, 0xc403);
        WriteGMII16( 0x15, 0x001c);
        WriteGMII16( 0x19, 0x0016);
        WriteGMII16( 0x15, 0x001d);
        WriteGMII16( 0x19, 0xaa05);
        WriteGMII16( 0x15, 0x001e);
        WriteGMII16( 0x19, 0xc503);
        WriteGMII16( 0x15, 0x001f);
        WriteGMII16( 0x19, 0x0003);
        WriteGMII16( 0x15, 0x0020);
        WriteGMII16( 0x19, 0x89f8);
        WriteGMII16( 0x15, 0x0021);
        WriteGMII16( 0x19, 0x32ae);
        WriteGMII16( 0x15, 0x0022);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0023);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x0024);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0025);
        WriteGMII16( 0x19, 0x6801);
        WriteGMII16( 0x15, 0x0026);
        WriteGMII16( 0x19, 0x66a0);
        WriteGMII16( 0x15, 0x0027);
        WriteGMII16( 0x19, 0xa300);
        WriteGMII16( 0x15, 0x0028);
        WriteGMII16( 0x19, 0x64a0);
        WriteGMII16( 0x15, 0x0029);
        WriteGMII16( 0x19, 0x76f0);
        WriteGMII16( 0x15, 0x002a);
        WriteGMII16( 0x19, 0x7670);
        WriteGMII16( 0x15, 0x002b);
        WriteGMII16( 0x19, 0x7630);
        WriteGMII16( 0x15, 0x002c);
        WriteGMII16( 0x19, 0x31a6);
        WriteGMII16( 0x15, 0x002d);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x002e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x002f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0030);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0031);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0032);
        WriteGMII16( 0x19, 0x4801);
        WriteGMII16( 0x15, 0x0033);
        WriteGMII16( 0x19, 0x6803);
        WriteGMII16( 0x15, 0x0034);
        WriteGMII16( 0x19, 0x66a1);
        WriteGMII16( 0x15, 0x0035);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0036);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x0037);
        WriteGMII16( 0x19, 0xa300);
        WriteGMII16( 0x15, 0x0038);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x0039);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x003a);
        WriteGMII16( 0x19, 0x74f8);
        WriteGMII16( 0x15, 0x003b);
        WriteGMII16( 0x19, 0x63d0);
        WriteGMII16( 0x15, 0x003c);
        WriteGMII16( 0x19, 0x7ff0);
        WriteGMII16( 0x15, 0x003d);
        WriteGMII16( 0x19, 0x77f0);
        WriteGMII16( 0x15, 0x003e);
        WriteGMII16( 0x19, 0x7ff0);
        WriteGMII16( 0x15, 0x003f);
        WriteGMII16( 0x19, 0x7750);
        WriteGMII16( 0x15, 0x0040);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x0041);
        WriteGMII16( 0x19, 0x7cf0);
        WriteGMII16( 0x15, 0x0042);
        WriteGMII16( 0x19, 0x7708);
        WriteGMII16( 0x15, 0x0043);
        WriteGMII16( 0x19, 0xa654);
        WriteGMII16( 0x15, 0x0044);
        WriteGMII16( 0x19, 0x304a);
        WriteGMII16( 0x15, 0x0045);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0046);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0047);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0048);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0049);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x004a);
        WriteGMII16( 0x19, 0x4802);
        WriteGMII16( 0x15, 0x004b);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x004c);
        WriteGMII16( 0x19, 0x4440);
        WriteGMII16( 0x15, 0x004d);
        WriteGMII16( 0x19, 0x63c8);
        WriteGMII16( 0x15, 0x004e);
        WriteGMII16( 0x19, 0x6481);
        WriteGMII16( 0x15, 0x004f);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x15, 0x0050);
        WriteGMII16( 0x19, 0x63e8);
        WriteGMII16( 0x15, 0x0051);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x0052);
        WriteGMII16( 0x19, 0x5900);
        WriteGMII16( 0x15, 0x0053);
        WriteGMII16( 0x19, 0x63f8);
        WriteGMII16( 0x15, 0x0054);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x0055);
        WriteGMII16( 0x19, 0x3116);
        WriteGMII16( 0x15, 0x0056);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0057);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0058);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0059);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x005a);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x005b);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x005c);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x005d);
        WriteGMII16( 0x19, 0x6000);
        WriteGMII16( 0x15, 0x005e);
        WriteGMII16( 0x19, 0x59ce);
        WriteGMII16( 0x15, 0x005f);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x0060);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x0061);
        WriteGMII16( 0x19, 0x72b0);
        WriteGMII16( 0x15, 0x0062);
        WriteGMII16( 0x19, 0x400e);
        WriteGMII16( 0x15, 0x0063);
        WriteGMII16( 0x19, 0x4440);
        WriteGMII16( 0x15, 0x0064);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x15, 0x0065);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x15, 0x0066);
        WriteGMII16( 0x19, 0x70b0);
        WriteGMII16( 0x15, 0x0067);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0068);
        WriteGMII16( 0x19, 0x6008);
        WriteGMII16( 0x15, 0x0069);
        WriteGMII16( 0x19, 0x7cf0);
        WriteGMII16( 0x15, 0x006a);
        WriteGMII16( 0x19, 0x7750);
        WriteGMII16( 0x15, 0x006b);
        WriteGMII16( 0x19, 0x4007);
        WriteGMII16( 0x15, 0x006c);
        WriteGMII16( 0x19, 0x4500);
        WriteGMII16( 0x15, 0x006d);
        WriteGMII16( 0x19, 0x4023);
        WriteGMII16( 0x15, 0x006e);
        WriteGMII16( 0x19, 0x4580);
        WriteGMII16( 0x15, 0x006f);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x0070);
        WriteGMII16( 0x19, 0xcd78);
        WriteGMII16( 0x15, 0x0071);
        WriteGMII16( 0x19, 0x0003);
        WriteGMII16( 0x15, 0x0072);
        WriteGMII16( 0x19, 0xbe02);
        WriteGMII16( 0x15, 0x0073);
        WriteGMII16( 0x19, 0x3070);
        WriteGMII16( 0x15, 0x0074);
        WriteGMII16( 0x19, 0x7cf0);
        WriteGMII16( 0x15, 0x0075);
        WriteGMII16( 0x19, 0x77f0);
        WriteGMII16( 0x15, 0x0076);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x0077);
        WriteGMII16( 0x19, 0x4007);
        WriteGMII16( 0x15, 0x0078);
        WriteGMII16( 0x19, 0x4500);
        WriteGMII16( 0x15, 0x0079);
        WriteGMII16( 0x19, 0x4023);
        WriteGMII16( 0x15, 0x007a);
        WriteGMII16( 0x19, 0x4580);
        WriteGMII16( 0x15, 0x007b);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x007c);
        WriteGMII16( 0x19, 0xce80);
        WriteGMII16( 0x15, 0x007d);
        WriteGMII16( 0x19, 0x0004);
        WriteGMII16( 0x15, 0x007e);
        WriteGMII16( 0x19, 0xce80);
        WriteGMII16( 0x15, 0x007f);
        WriteGMII16( 0x19, 0x0002);
        WriteGMII16( 0x15, 0x0080);
        WriteGMII16( 0x19, 0x307c);
        WriteGMII16( 0x15, 0x0081);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x0082);
        WriteGMII16( 0x19, 0x480f);
        WriteGMII16( 0x15, 0x0083);
        WriteGMII16( 0x19, 0x6802);
        WriteGMII16( 0x15, 0x0084);
        WriteGMII16( 0x19, 0x6680);
        WriteGMII16( 0x15, 0x0085);
        WriteGMII16( 0x19, 0x7c10);
        WriteGMII16( 0x15, 0x0086);
        WriteGMII16( 0x19, 0x6010);
        WriteGMII16( 0x15, 0x0087);
        WriteGMII16( 0x19, 0x400a);
        WriteGMII16( 0x15, 0x0088);
        WriteGMII16( 0x19, 0x4580);
        WriteGMII16( 0x15, 0x0089);
        WriteGMII16( 0x19, 0x9e00);
        WriteGMII16( 0x15, 0x008a);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x008b);
        WriteGMII16( 0x19, 0x5800);
        WriteGMII16( 0x15, 0x008c);
        WriteGMII16( 0x19, 0x63c8);
        WriteGMII16( 0x15, 0x008d);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x008e);
        WriteGMII16( 0x19, 0x66a0);
        WriteGMII16( 0x15, 0x008f);
        WriteGMII16( 0x19, 0x8300);
        WriteGMII16( 0x15, 0x0090);
        WriteGMII16( 0x19, 0x7ff0);
        WriteGMII16( 0x15, 0x0091);
        WriteGMII16( 0x19, 0x74f0);
        WriteGMII16( 0x15, 0x0092);
        WriteGMII16( 0x19, 0x3006);
        WriteGMII16( 0x15, 0x0093);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0094);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0095);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0096);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0097);
        WriteGMII16( 0x19, 0x4803);
        WriteGMII16( 0x15, 0x0098);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0099);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x009a);
        WriteGMII16( 0x19, 0xa203);
        WriteGMII16( 0x15, 0x009b);
        WriteGMII16( 0x19, 0x64b1);
        WriteGMII16( 0x15, 0x009c);
        WriteGMII16( 0x19, 0x309e);
        WriteGMII16( 0x15, 0x009d);
        WriteGMII16( 0x19, 0x64b3);
        WriteGMII16( 0x15, 0x009e);
        WriteGMII16( 0x19, 0x4030);
        WriteGMII16( 0x15, 0x009f);
        WriteGMII16( 0x19, 0x440e);
        WriteGMII16( 0x15, 0x00a0);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x00a1);
        WriteGMII16( 0x19, 0x4419);
        WriteGMII16( 0x15, 0x00a2);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x00a3);
        WriteGMII16( 0x19, 0xc520);
        WriteGMII16( 0x15, 0x00a4);
        WriteGMII16( 0x19, 0x000b);
        WriteGMII16( 0x15, 0x00a5);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x00a6);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x00a7);
        WriteGMII16( 0x19, 0x58a4);
        WriteGMII16( 0x15, 0x00a8);
        WriteGMII16( 0x19, 0x63da);
        WriteGMII16( 0x15, 0x00a9);
        WriteGMII16( 0x19, 0x5cb0);
        WriteGMII16( 0x15, 0x00aa);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x00ab);
        WriteGMII16( 0x19, 0x72b0);
        WriteGMII16( 0x15, 0x00ac);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x15, 0x00ad);
        WriteGMII16( 0x19, 0x70b0);
        WriteGMII16( 0x15, 0x00ae);
        WriteGMII16( 0x19, 0x30b8);
        WriteGMII16( 0x15, 0x00AF);
        WriteGMII16( 0x19, 0x4060);
        WriteGMII16( 0x15, 0x00B0);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x00B1);
        WriteGMII16( 0x19, 0x7e00);
        WriteGMII16( 0x15, 0x00B2);
        WriteGMII16( 0x19, 0x72B0);
        WriteGMII16( 0x15, 0x00B3);
        WriteGMII16( 0x19, 0x7F00);
        WriteGMII16( 0x15, 0x00B4);
        WriteGMII16( 0x19, 0x73B0);
        WriteGMII16( 0x15, 0x00b5);
        WriteGMII16( 0x19, 0x58a0);
        WriteGMII16( 0x15, 0x00b6);
        WriteGMII16( 0x19, 0x63d2);
        WriteGMII16( 0x15, 0x00b7);
        WriteGMII16( 0x19, 0x5c00);
        WriteGMII16( 0x15, 0x00b8);
        WriteGMII16( 0x19, 0x5780);
        WriteGMII16( 0x15, 0x00b9);
        WriteGMII16( 0x19, 0xb60d);
        WriteGMII16( 0x15, 0x00ba);
        WriteGMII16( 0x19, 0x9bff);
        WriteGMII16( 0x15, 0x00bb);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x00bc);
        WriteGMII16( 0x19, 0x6001);
        WriteGMII16( 0x15, 0x00bd);
        WriteGMII16( 0x19, 0xc020);
        WriteGMII16( 0x15, 0x00be);
        WriteGMII16( 0x19, 0x002b);
        WriteGMII16( 0x15, 0x00bf);
        WriteGMII16( 0x19, 0xc137);
        WriteGMII16( 0x15, 0x00c0);
        WriteGMII16( 0x19, 0x0006);
        WriteGMII16( 0x15, 0x00c1);
        WriteGMII16( 0x19, 0x9af8);
        WriteGMII16( 0x15, 0x00c2);
        WriteGMII16( 0x19, 0x30c6);
        WriteGMII16( 0x15, 0x00c3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00c4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00c5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00c6);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x00c7);
        WriteGMII16( 0x19, 0x70b0);
        WriteGMII16( 0x15, 0x00c8);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x00c9);
        WriteGMII16( 0x19, 0x4804);
        WriteGMII16( 0x15, 0x00ca);
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x00cb);
        WriteGMII16( 0x19, 0x5c80);
        WriteGMII16( 0x15, 0x00cc);
        WriteGMII16( 0x19, 0x4010);
        WriteGMII16( 0x15, 0x00cd);
        WriteGMII16( 0x19, 0x4415);
        WriteGMII16( 0x15, 0x00ce);
        WriteGMII16( 0x19, 0x9b00);
        WriteGMII16( 0x15, 0x00cf);
        WriteGMII16( 0x19, 0x7f00);
        WriteGMII16( 0x15, 0x00d0);
        WriteGMII16( 0x19, 0x70b0);
        WriteGMII16( 0x15, 0x00d1);
        WriteGMII16( 0x19, 0x3177);
        WriteGMII16( 0x15, 0x00d2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00d3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00d4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00d5);
        WriteGMII16( 0x19, 0x4808);
        WriteGMII16( 0x15, 0x00d6);
        WriteGMII16( 0x19, 0x4007);
        WriteGMII16( 0x15, 0x00d7);
        WriteGMII16( 0x19, 0x4420);
        WriteGMII16( 0x15, 0x00d8);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x00d9);
        WriteGMII16( 0x19, 0xb608);
        WriteGMII16( 0x15, 0x00da);
        WriteGMII16( 0x19, 0xbcbd);
        WriteGMII16( 0x15, 0x00db);
        WriteGMII16( 0x19, 0xc60b);
        WriteGMII16( 0x15, 0x00dc);
        WriteGMII16( 0x19, 0x00fd);
        WriteGMII16( 0x15, 0x00dd);
        WriteGMII16( 0x19, 0x30e1);
        WriteGMII16( 0x15, 0x00de);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00df);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00e0);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00e1);
        WriteGMII16( 0x19, 0x4809);
        WriteGMII16( 0x15, 0x00e2);
        WriteGMII16( 0x19, 0x7e40);
        WriteGMII16( 0x15, 0x00e3);
        WriteGMII16( 0x19, 0x5a40);
        WriteGMII16( 0x15, 0x00e4);
        WriteGMII16( 0x19, 0x305a);
        WriteGMII16( 0x15, 0x00e5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00e6);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00e7);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00e8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00e9);
        WriteGMII16( 0x19, 0x480a);
        WriteGMII16( 0x15, 0x00ea);
        WriteGMII16( 0x19, 0x5820);
        WriteGMII16( 0x15, 0x00eb);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x00ec);
        WriteGMII16( 0x19, 0xb60a);
        WriteGMII16( 0x15, 0x00ed);
        WriteGMII16( 0x19, 0xda07);
        WriteGMII16( 0x15, 0x00ee);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x00ef);
        WriteGMII16( 0x19, 0xc60b);
        WriteGMII16( 0x15, 0x00f0);
        WriteGMII16( 0x19, 0x00fc);
        WriteGMII16( 0x15, 0x00f1);
        WriteGMII16( 0x19, 0x30f6);
        WriteGMII16( 0x15, 0x00f2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00f3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00f4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00f5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x00f6);
        WriteGMII16( 0x19, 0x4408);
        WriteGMII16( 0x15, 0x00f7);
        WriteGMII16( 0x19, 0x480b);
        WriteGMII16( 0x15, 0x00f8);
        WriteGMII16( 0x19, 0x6f03);
        WriteGMII16( 0x15, 0x00f9);
        WriteGMII16( 0x19, 0x405f);
        WriteGMII16( 0x15, 0x00fa);
        WriteGMII16( 0x19, 0x4448);
        WriteGMII16( 0x15, 0x00fb);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x00fc);
        WriteGMII16( 0x19, 0x4468);
        WriteGMII16( 0x15, 0x00fd);
        WriteGMII16( 0x19, 0x9c03);
        WriteGMII16( 0x15, 0x00fe);
        WriteGMII16( 0x19, 0x6f07);
        WriteGMII16( 0x15, 0x00ff);
        WriteGMII16( 0x19, 0x58a0);
        WriteGMII16( 0x15, 0x0100);
        WriteGMII16( 0x19, 0xd6d1);
        WriteGMII16( 0x15, 0x0101);
        WriteGMII16( 0x19, 0x0004);
        WriteGMII16( 0x15, 0x0102);
        WriteGMII16( 0x19, 0xc137);
        WriteGMII16( 0x15, 0x0103);
        WriteGMII16( 0x19, 0x0002);
        WriteGMII16( 0x15, 0x0104);
        WriteGMII16( 0x19, 0xa0e5);
        WriteGMII16( 0x15, 0x0105);
        WriteGMII16( 0x19, 0x9df8);
        WriteGMII16( 0x15, 0x0106);
        WriteGMII16( 0x19, 0x30c6);
        WriteGMII16( 0x15, 0x0107);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0108);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0109);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x010a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x010b);
        WriteGMII16( 0x19, 0x4808);
        WriteGMII16( 0x15, 0x010c);
        WriteGMII16( 0x19, 0xc32d);
        WriteGMII16( 0x15, 0x010d);
        WriteGMII16( 0x19, 0x0003);
        WriteGMII16( 0x15, 0x010e);
        WriteGMII16( 0x19, 0xc8b3);
        WriteGMII16( 0x15, 0x010f);
        WriteGMII16( 0x19, 0x00fc);
        WriteGMII16( 0x15, 0x0110);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x0111);
        WriteGMII16( 0x19, 0x3116);
        WriteGMII16( 0x15, 0x0112);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0113);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0114);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0115);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0116);
        WriteGMII16( 0x19, 0x4803);
        WriteGMII16( 0x15, 0x0117);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0118);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x0119);
        WriteGMII16( 0x19, 0x7c04);
        WriteGMII16( 0x15, 0x011a);
        WriteGMII16( 0x19, 0x6000);
        WriteGMII16( 0x15, 0x011b);
        WriteGMII16( 0x19, 0x5cf7);
        WriteGMII16( 0x15, 0x011c);
        WriteGMII16( 0x19, 0x7c2a);
        WriteGMII16( 0x15, 0x011d);
        WriteGMII16( 0x19, 0x5800);
        WriteGMII16( 0x15, 0x011e);
        WriteGMII16( 0x19, 0x5400);
        WriteGMII16( 0x15, 0x011f);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0120);
        WriteGMII16( 0x19, 0x74f0);
        WriteGMII16( 0x15, 0x0121);
        WriteGMII16( 0x19, 0x4019);
        WriteGMII16( 0x15, 0x0122);
        WriteGMII16( 0x19, 0x440d);
        WriteGMII16( 0x15, 0x0123);
        WriteGMII16( 0x19, 0xb6c1);
        WriteGMII16( 0x15, 0x0124);
        WriteGMII16( 0x19, 0xc05b);
        WriteGMII16( 0x15, 0x0125);
        WriteGMII16( 0x19, 0x00bf);
        WriteGMII16( 0x15, 0x0126);
        WriteGMII16( 0x19, 0xc025);
        WriteGMII16( 0x15, 0x0127);
        WriteGMII16( 0x19, 0x00bd);
        WriteGMII16( 0x15, 0x0128);
        WriteGMII16( 0x19, 0xc603);
        WriteGMII16( 0x15, 0x0129);
        WriteGMII16( 0x19, 0x00bb);
        WriteGMII16( 0x15, 0x012a);
        WriteGMII16( 0x19, 0x8805);
        WriteGMII16( 0x15, 0x012b);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x012c);
        WriteGMII16( 0x19, 0x4001);
        WriteGMII16( 0x15, 0x012d);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x012e);
        WriteGMII16( 0x19, 0xa3dd);
        WriteGMII16( 0x15, 0x012f);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0130);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x0131);
        WriteGMII16( 0x19, 0x8407);
        WriteGMII16( 0x15, 0x0132);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0133);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x0134);
        WriteGMII16( 0x19, 0xd9b8);
        WriteGMII16( 0x15, 0x0135);
        WriteGMII16( 0x19, 0x0003);
        WriteGMII16( 0x15, 0x0136);
        WriteGMII16( 0x19, 0xc240);
        WriteGMII16( 0x15, 0x0137);
        WriteGMII16( 0x19, 0x0015);
        WriteGMII16( 0x15, 0x0138);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0139);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x013a);
        WriteGMII16( 0x19, 0x9ae9);
        WriteGMII16( 0x15, 0x013b);
        WriteGMII16( 0x19, 0x3140);
        WriteGMII16( 0x15, 0x013c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x013d);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x013e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x013f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0140);
        WriteGMII16( 0x19, 0x4807);
        WriteGMII16( 0x15, 0x0141);
        WriteGMII16( 0x19, 0x4004);
        WriteGMII16( 0x15, 0x0142);
        WriteGMII16( 0x19, 0x4410);
        WriteGMII16( 0x15, 0x0143);
        WriteGMII16( 0x19, 0x7c0c);
        WriteGMII16( 0x15, 0x0144);
        WriteGMII16( 0x19, 0x600c);
        WriteGMII16( 0x15, 0x0145);
        WriteGMII16( 0x19, 0x9b00);
        WriteGMII16( 0x15, 0x0146);
        WriteGMII16( 0x19, 0xa68f);
        WriteGMII16( 0x15, 0x0147);
        WriteGMII16( 0x19, 0x3116);
        WriteGMII16( 0x15, 0x0148);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0149);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x014a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x014b);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x014c);
        WriteGMII16( 0x19, 0x4804);
        WriteGMII16( 0x15, 0x014d);
        WriteGMII16( 0x19, 0x54c0);
        WriteGMII16( 0x15, 0x014e);
        WriteGMII16( 0x19, 0xb703);
        WriteGMII16( 0x15, 0x014f);
        WriteGMII16( 0x19, 0x5cff);
        WriteGMII16( 0x15, 0x0150);
        WriteGMII16( 0x19, 0x315f);
        WriteGMII16( 0x15, 0x0151);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0152);
        WriteGMII16( 0x19, 0x74f8);
        WriteGMII16( 0x15, 0x0153);
        WriteGMII16( 0x19, 0x6421);
        WriteGMII16( 0x15, 0x0154);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0155);
        WriteGMII16( 0x19, 0x6000);
        WriteGMII16( 0x15, 0x0156);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x0157);
        WriteGMII16( 0x19, 0x4418);
        WriteGMII16( 0x15, 0x0158);
        WriteGMII16( 0x19, 0x9b00);
        WriteGMII16( 0x15, 0x0159);
        WriteGMII16( 0x19, 0x6461);
        WriteGMII16( 0x15, 0x015a);
        WriteGMII16( 0x19, 0x64e1);
        WriteGMII16( 0x15, 0x015b);
        WriteGMII16( 0x19, 0x7c20);
        WriteGMII16( 0x15, 0x015c);
        WriteGMII16( 0x19, 0x5820);
        WriteGMII16( 0x15, 0x015d);
        WriteGMII16( 0x19, 0x5ccf);
        WriteGMII16( 0x15, 0x015e);
        WriteGMII16( 0x19, 0x7050);
        WriteGMII16( 0x15, 0x015f);
        WriteGMII16( 0x19, 0xd9b8);
        WriteGMII16( 0x15, 0x0160);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x0161);
        WriteGMII16( 0x19, 0xdab1);
        WriteGMII16( 0x15, 0x0162);
        WriteGMII16( 0x19, 0x0015);
        WriteGMII16( 0x15, 0x0163);
        WriteGMII16( 0x19, 0xc244);
        WriteGMII16( 0x15, 0x0164);
        WriteGMII16( 0x19, 0x0013);
        WriteGMII16( 0x15, 0x0165);
        WriteGMII16( 0x19, 0xc021);
        WriteGMII16( 0x15, 0x0166);
        WriteGMII16( 0x19, 0x00f9);
        WriteGMII16( 0x15, 0x0167);
        WriteGMII16( 0x19, 0x3177);
        WriteGMII16( 0x15, 0x0168);
        WriteGMII16( 0x19, 0x5cf7);
        WriteGMII16( 0x15, 0x0169);
        WriteGMII16( 0x19, 0x4010);
        WriteGMII16( 0x15, 0x016a);
        WriteGMII16( 0x19, 0x4428);
        WriteGMII16( 0x15, 0x016b);
        WriteGMII16( 0x19, 0x9c00);
        WriteGMII16( 0x15, 0x016c);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x016d);
        WriteGMII16( 0x19, 0x6008);
        WriteGMII16( 0x15, 0x016e);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x016f);
        WriteGMII16( 0x19, 0x74f0);
        WriteGMII16( 0x15, 0x0170);
        WriteGMII16( 0x19, 0x6461);
        WriteGMII16( 0x15, 0x0171);
        WriteGMII16( 0x19, 0x6421);
        WriteGMII16( 0x15, 0x0172);
        WriteGMII16( 0x19, 0x64a1);
        WriteGMII16( 0x15, 0x0173);
        WriteGMII16( 0x19, 0x3116);
        WriteGMII16( 0x15, 0x0174);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0175);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0176);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0177);
        WriteGMII16( 0x19, 0x4805);
        WriteGMII16( 0x15, 0x0178);
        WriteGMII16( 0x19, 0xa103);
        WriteGMII16( 0x15, 0x0179);
        WriteGMII16( 0x19, 0x7c02);
        WriteGMII16( 0x15, 0x017a);
        WriteGMII16( 0x19, 0x6002);
        WriteGMII16( 0x15, 0x017b);
        WriteGMII16( 0x19, 0x7e00);
        WriteGMII16( 0x15, 0x017c);
        WriteGMII16( 0x19, 0x5400);
        WriteGMII16( 0x15, 0x017d);
        WriteGMII16( 0x19, 0x7c6b);
        WriteGMII16( 0x15, 0x017e);
        WriteGMII16( 0x19, 0x5c63);
        WriteGMII16( 0x15, 0x017f);
        WriteGMII16( 0x19, 0x407d);
        WriteGMII16( 0x15, 0x0180);
        WriteGMII16( 0x19, 0xa602);
        WriteGMII16( 0x15, 0x0181);
        WriteGMII16( 0x19, 0x4001);
        WriteGMII16( 0x15, 0x0182);
        WriteGMII16( 0x19, 0x4420);
        WriteGMII16( 0x15, 0x0183);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x0184);
        WriteGMII16( 0x19, 0x44a1);
        WriteGMII16( 0x15, 0x0185);
        WriteGMII16( 0x19, 0xd6e0);
        WriteGMII16( 0x15, 0x0186);
        WriteGMII16( 0x19, 0x0009);
        WriteGMII16( 0x15, 0x0187);
        WriteGMII16( 0x19, 0x9efe);
        WriteGMII16( 0x15, 0x0188);
        WriteGMII16( 0x19, 0x7c02);
        WriteGMII16( 0x15, 0x0189);
        WriteGMII16( 0x19, 0x6000);
        WriteGMII16( 0x15, 0x018a);
        WriteGMII16( 0x19, 0x9c00);
        WriteGMII16( 0x15, 0x018b);
        WriteGMII16( 0x19, 0x318f);
        WriteGMII16( 0x15, 0x018c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x018d);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x018e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x018f);
        WriteGMII16( 0x19, 0x4806);
        WriteGMII16( 0x15, 0x0190);
        WriteGMII16( 0x19, 0x7c10);
        WriteGMII16( 0x15, 0x0191);
        WriteGMII16( 0x19, 0x5c10);
        WriteGMII16( 0x15, 0x0192);
        WriteGMII16( 0x19, 0x40fa);
        WriteGMII16( 0x15, 0x0193);
        WriteGMII16( 0x19, 0xa602);
        WriteGMII16( 0x15, 0x0194);
        WriteGMII16( 0x19, 0x4010);
        WriteGMII16( 0x15, 0x0195);
        WriteGMII16( 0x19, 0x4440);
        WriteGMII16( 0x15, 0x0196);
        WriteGMII16( 0x19, 0x9d00);
        WriteGMII16( 0x15, 0x0197);
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x0198);
        WriteGMII16( 0x19, 0x6400);
        WriteGMII16( 0x15, 0x0199);
        WriteGMII16( 0x19, 0x4003);
        WriteGMII16( 0x15, 0x019a);
        WriteGMII16( 0x19, 0x4540);
        WriteGMII16( 0x15, 0x019b);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x019c);
        WriteGMII16( 0x19, 0x6008);
        WriteGMII16( 0x15, 0x019d);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x019e);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x019f);
        WriteGMII16( 0x19, 0x6400);
        WriteGMII16( 0x15, 0x01a0);
        WriteGMII16( 0x19, 0x7c80);
        WriteGMII16( 0x15, 0x01a1);
        WriteGMII16( 0x19, 0x6480);
        WriteGMII16( 0x15, 0x01a2);
        WriteGMII16( 0x19, 0x3140);
        WriteGMII16( 0x15, 0x01a3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01a4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01a5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01a6);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x01a7);
        WriteGMII16( 0x19, 0x7c0b);
        WriteGMII16( 0x15, 0x01a8);
        WriteGMII16( 0x19, 0x6c01);
        WriteGMII16( 0x15, 0x01a9);
        WriteGMII16( 0x19, 0x64a8);
        WriteGMII16( 0x15, 0x01aa);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x01ab);
        WriteGMII16( 0x19, 0x5cf0);
        WriteGMII16( 0x15, 0x01ac);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x01ad);
        WriteGMII16( 0x19, 0xb628);
        WriteGMII16( 0x15, 0x01ae);
        WriteGMII16( 0x19, 0xc053);
        WriteGMII16( 0x15, 0x01af);
        WriteGMII16( 0x19, 0x0026);
        WriteGMII16( 0x15, 0x01b0);
        WriteGMII16( 0x19, 0xc02d);
        WriteGMII16( 0x15, 0x01b1);
        WriteGMII16( 0x19, 0x0024);
        WriteGMII16( 0x15, 0x01b2);
        WriteGMII16( 0x19, 0xc603);
        WriteGMII16( 0x15, 0x01b3);
        WriteGMII16( 0x19, 0x0022);
        WriteGMII16( 0x15, 0x01b4);
        WriteGMII16( 0x19, 0x8cf9);
        WriteGMII16( 0x15, 0x01b5);
        WriteGMII16( 0x19, 0x31ba);
        WriteGMII16( 0x15, 0x01b6);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01b7);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01b8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01b9);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01ba);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x01bb);
        WriteGMII16( 0x19, 0x5420);
        WriteGMII16( 0x15, 0x01bc);
        WriteGMII16( 0x19, 0x4811);
        WriteGMII16( 0x15, 0x01bd);
        WriteGMII16( 0x19, 0x5000);
        WriteGMII16( 0x15, 0x01be);
        WriteGMII16( 0x19, 0x4801);
        WriteGMII16( 0x15, 0x01bf);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x01c0);
        WriteGMII16( 0x19, 0x31f5);
        WriteGMII16( 0x15, 0x01c1);
        WriteGMII16( 0x19, 0xb614);
        WriteGMII16( 0x15, 0x01c2);
        WriteGMII16( 0x19, 0x8ce4);
        WriteGMII16( 0x15, 0x01c3);
        WriteGMII16( 0x19, 0xb30c);
        WriteGMII16( 0x15, 0x01c4);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x01c5);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x01c6);
        WriteGMII16( 0x19, 0x8206);
        WriteGMII16( 0x15, 0x01c7);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x01c8);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x01c9);
        WriteGMII16( 0x19, 0x7c04);
        WriteGMII16( 0x15, 0x01ca);
        WriteGMII16( 0x19, 0x7404);
        WriteGMII16( 0x15, 0x01cb);
        WriteGMII16( 0x19, 0x31c0);
        WriteGMII16( 0x15, 0x01cc);
        WriteGMII16( 0x19, 0x7c04);
        WriteGMII16( 0x15, 0x01cd);
        WriteGMII16( 0x19, 0x7400);
        WriteGMII16( 0x15, 0x01ce);
        WriteGMII16( 0x19, 0x31c0);
        WriteGMII16( 0x15, 0x01cf);
        WriteGMII16( 0x19, 0x8df1);
        WriteGMII16( 0x15, 0x01d0);
        WriteGMII16( 0x19, 0x3248);
        WriteGMII16( 0x15, 0x01d1);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01d2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01d3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01d4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01d5);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x01d6);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x01d7);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x01d8);
        WriteGMII16( 0x19, 0x7670);
        WriteGMII16( 0x15, 0x01d9);
        WriteGMII16( 0x19, 0x4023);
        WriteGMII16( 0x15, 0x01da);
        WriteGMII16( 0x19, 0x4500);
        WriteGMII16( 0x15, 0x01db);
        WriteGMII16( 0x19, 0x4069);
        WriteGMII16( 0x15, 0x01dc);
        WriteGMII16( 0x19, 0x4580);
        WriteGMII16( 0x15, 0x01dd);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x01de);
        WriteGMII16( 0x19, 0xcff5);
        WriteGMII16( 0x15, 0x01df);
        WriteGMII16( 0x19, 0x00ff);
        WriteGMII16( 0x15, 0x01e0);
        WriteGMII16( 0x19, 0x76f0);
        WriteGMII16( 0x15, 0x01e1);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x01e2);
        WriteGMII16( 0x19, 0x4023);
        WriteGMII16( 0x15, 0x01e3);
        WriteGMII16( 0x19, 0x4500);
        WriteGMII16( 0x15, 0x01e4);
        WriteGMII16( 0x19, 0x4069);
        WriteGMII16( 0x15, 0x01e5);
        WriteGMII16( 0x19, 0x4580);
        WriteGMII16( 0x15, 0x01e6);
        WriteGMII16( 0x19, 0x9f00);
        WriteGMII16( 0x15, 0x01e7);
        WriteGMII16( 0x19, 0xd0f5);
        WriteGMII16( 0x15, 0x01e8);
        WriteGMII16( 0x19, 0x00ff);
        WriteGMII16( 0x15, 0x01e9);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x01ea);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x01eb);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x01ec);
        WriteGMII16( 0x19, 0x66a0);
        WriteGMII16( 0x15, 0x01ed);
        WriteGMII16( 0x19, 0x8300);
        WriteGMII16( 0x15, 0x01ee);
        WriteGMII16( 0x19, 0x74f0);
        WriteGMII16( 0x15, 0x01ef);
        WriteGMII16( 0x19, 0x3006);
        WriteGMII16( 0x15, 0x01f0);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01f1);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01f2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01f3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01f4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x01f5);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x01f6);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x01f7);
        WriteGMII16( 0x19, 0x409d);
        WriteGMII16( 0x15, 0x01f8);
        WriteGMII16( 0x19, 0x7c87);
        WriteGMII16( 0x15, 0x01f9);
        WriteGMII16( 0x19, 0xae14);
        WriteGMII16( 0x15, 0x01fa);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x01fb);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x01fc);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x01fd);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x01fe);
        WriteGMII16( 0x19, 0x980e);
        WriteGMII16( 0x15, 0x01ff);
        WriteGMII16( 0x19, 0x930c);
        WriteGMII16( 0x15, 0x0200);
        WriteGMII16( 0x19, 0x9206);
        WriteGMII16( 0x15, 0x0201);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0202);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0203);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0204);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0205);
        WriteGMII16( 0x19, 0x320c);
        WriteGMII16( 0x15, 0x0206);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x15, 0x0207);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0208);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x0209);
        WriteGMII16( 0x19, 0x5500);
        WriteGMII16( 0x15, 0x020a);
        WriteGMII16( 0x19, 0x320c);
        WriteGMII16( 0x15, 0x020b);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x020c);
        WriteGMII16( 0x19, 0x3220);
        WriteGMII16( 0x15, 0x020d);
        WriteGMII16( 0x19, 0x4480);
        WriteGMII16( 0x15, 0x020e);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x020f);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0210);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x0211);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0212);
        WriteGMII16( 0x19, 0x980e);
        WriteGMII16( 0x15, 0x0213);
        WriteGMII16( 0x19, 0x930c);
        WriteGMII16( 0x15, 0x0214);
        WriteGMII16( 0x19, 0x9206);
        WriteGMII16( 0x15, 0x0215);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x15, 0x0216);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0217);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0218);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0219);
        WriteGMII16( 0x19, 0x3220);
        WriteGMII16( 0x15, 0x021a);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x021b);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x021c);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x021d);
        WriteGMII16( 0x19, 0x5540);
        WriteGMII16( 0x15, 0x021e);
        WriteGMII16( 0x19, 0x3220);
        WriteGMII16( 0x15, 0x021f);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x15, 0x0220);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0221);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0222);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0223);
        WriteGMII16( 0x19, 0x3231);
        WriteGMII16( 0x15, 0x0224);
        WriteGMII16( 0x19, 0xab06);
        WriteGMII16( 0x15, 0x0225);
        WriteGMII16( 0x19, 0xbf08);
        WriteGMII16( 0x15, 0x0226);
        WriteGMII16( 0x19, 0x4076);
        WriteGMII16( 0x15, 0x0227);
        WriteGMII16( 0x19, 0x7d07);
        WriteGMII16( 0x15, 0x0228);
        WriteGMII16( 0x19, 0x4502);
        WriteGMII16( 0x15, 0x0229);
        WriteGMII16( 0x19, 0x3231);
        WriteGMII16( 0x15, 0x022a);
        WriteGMII16( 0x19, 0x7d80);
        WriteGMII16( 0x15, 0x022b);
        WriteGMII16( 0x19, 0x5180);
        WriteGMII16( 0x15, 0x022c);
        WriteGMII16( 0x19, 0x322f);
        WriteGMII16( 0x15, 0x022d);
        WriteGMII16( 0x19, 0x7d80);
        WriteGMII16( 0x15, 0x022e);
        WriteGMII16( 0x19, 0x5000);
        WriteGMII16( 0x15, 0x022f);
        WriteGMII16( 0x19, 0x7d07);
        WriteGMII16( 0x15, 0x0230);
        WriteGMII16( 0x19, 0x4402);
        WriteGMII16( 0x15, 0x0231);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0232);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x0233);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0234);
        WriteGMII16( 0x19, 0xb309);
        WriteGMII16( 0x15, 0x0235);
        WriteGMII16( 0x19, 0xb204);
        WriteGMII16( 0x15, 0x0236);
        WriteGMII16( 0x19, 0xb105);
        WriteGMII16( 0x15, 0x0237);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0238);
        WriteGMII16( 0x19, 0x31c1);
        WriteGMII16( 0x15, 0x0239);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x023a);
        WriteGMII16( 0x19, 0x3261);
        WriteGMII16( 0x15, 0x023b);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x023c);
        WriteGMII16( 0x19, 0x3250);
        WriteGMII16( 0x15, 0x023d);
        WriteGMII16( 0x19, 0xb203);
        WriteGMII16( 0x15, 0x023e);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x023f);
        WriteGMII16( 0x19, 0x327a);
        WriteGMII16( 0x15, 0x0240);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0241);
        WriteGMII16( 0x19, 0x3293);
        WriteGMII16( 0x15, 0x0242);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0243);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0244);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0245);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0246);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0247);
        WriteGMII16( 0x19, 0x32a3);
        WriteGMII16( 0x15, 0x0248);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0249);
        WriteGMII16( 0x19, 0x403d);
        WriteGMII16( 0x15, 0x024a);
        WriteGMII16( 0x19, 0x440c);
        WriteGMII16( 0x15, 0x024b);
        WriteGMII16( 0x19, 0x4812);
        WriteGMII16( 0x15, 0x024c);
        WriteGMII16( 0x19, 0x5001);
        WriteGMII16( 0x15, 0x024d);
        WriteGMII16( 0x19, 0x4802);
        WriteGMII16( 0x15, 0x024e);
        WriteGMII16( 0x19, 0x6880);
        WriteGMII16( 0x15, 0x024f);
        WriteGMII16( 0x19, 0x31f5);
        WriteGMII16( 0x15, 0x0250);
        WriteGMII16( 0x19, 0xb685);
        WriteGMII16( 0x15, 0x0251);
        WriteGMII16( 0x19, 0x801c);
        WriteGMII16( 0x15, 0x0252);
        WriteGMII16( 0x19, 0xbaf5);
        WriteGMII16( 0x15, 0x0253);
        WriteGMII16( 0x19, 0xc07c);
        WriteGMII16( 0x15, 0x0254);
        WriteGMII16( 0x19, 0x00fb);
        WriteGMII16( 0x15, 0x0255);
        WriteGMII16( 0x19, 0x325a);
        WriteGMII16( 0x15, 0x0256);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0257);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0258);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0259);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x025a);
        WriteGMII16( 0x19, 0x481a);
        WriteGMII16( 0x15, 0x025b);
        WriteGMII16( 0x19, 0x5001);
        WriteGMII16( 0x15, 0x025c);
        WriteGMII16( 0x19, 0x401b);
        WriteGMII16( 0x15, 0x025d);
        WriteGMII16( 0x19, 0x480a);
        WriteGMII16( 0x15, 0x025e);
        WriteGMII16( 0x19, 0x4418);
        WriteGMII16( 0x15, 0x025f);
        WriteGMII16( 0x19, 0x6900);
        WriteGMII16( 0x15, 0x0260);
        WriteGMII16( 0x19, 0x31f5);
        WriteGMII16( 0x15, 0x0261);
        WriteGMII16( 0x19, 0xb64b);
        WriteGMII16( 0x15, 0x0262);
        WriteGMII16( 0x19, 0xdb00);
        WriteGMII16( 0x15, 0x0263);
        WriteGMII16( 0x19, 0x0048);
        WriteGMII16( 0x15, 0x0264);
        WriteGMII16( 0x19, 0xdb7d);
        WriteGMII16( 0x15, 0x0265);
        WriteGMII16( 0x19, 0x0002);
        WriteGMII16( 0x15, 0x0266);
        WriteGMII16( 0x19, 0xa0fa);
        WriteGMII16( 0x15, 0x0267);
        WriteGMII16( 0x19, 0x4408);
        WriteGMII16( 0x15, 0x0268);
        WriteGMII16( 0x19, 0x3248);
        WriteGMII16( 0x15, 0x0269);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x026a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x026b);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x026c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x026d);
        WriteGMII16( 0x19, 0xb806);
        WriteGMII16( 0x15, 0x026e);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x026f);
        WriteGMII16( 0x19, 0x5500);
        WriteGMII16( 0x15, 0x0270);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0271);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0272);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0273);
        WriteGMII16( 0x19, 0x4814);
        WriteGMII16( 0x15, 0x0274);
        WriteGMII16( 0x19, 0x500b);
        WriteGMII16( 0x15, 0x0275);
        WriteGMII16( 0x19, 0x4804);
        WriteGMII16( 0x15, 0x0276);
        WriteGMII16( 0x19, 0x40c4);
        WriteGMII16( 0x15, 0x0277);
        WriteGMII16( 0x19, 0x4425);
        WriteGMII16( 0x15, 0x0278);
        WriteGMII16( 0x19, 0x6a00);
        WriteGMII16( 0x15, 0x0279);
        WriteGMII16( 0x19, 0x31f5);
        WriteGMII16( 0x15, 0x027a);
        WriteGMII16( 0x19, 0xb632);
        WriteGMII16( 0x15, 0x027b);
        WriteGMII16( 0x19, 0xdc03);
        WriteGMII16( 0x15, 0x027c);
        WriteGMII16( 0x19, 0x0027);
        WriteGMII16( 0x15, 0x027d);
        WriteGMII16( 0x19, 0x80fc);
        WriteGMII16( 0x15, 0x027e);
        WriteGMII16( 0x19, 0x3283);
        WriteGMII16( 0x15, 0x027f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0280);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0281);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0282);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0283);
        WriteGMII16( 0x19, 0xb806);
        WriteGMII16( 0x15, 0x0284);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0285);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0286);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0287);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x15, 0x0288);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0289);
        WriteGMII16( 0x19, 0x4818);
        WriteGMII16( 0x15, 0x028a);
        WriteGMII16( 0x19, 0x5051);
        WriteGMII16( 0x15, 0x028b);
        WriteGMII16( 0x19, 0x4808);
        WriteGMII16( 0x15, 0x028c);
        WriteGMII16( 0x19, 0x4050);
        WriteGMII16( 0x15, 0x028d);
        WriteGMII16( 0x19, 0x4462);
        WriteGMII16( 0x15, 0x028e);
        WriteGMII16( 0x19, 0x40c4);
        WriteGMII16( 0x15, 0x028f);
        WriteGMII16( 0x19, 0x4473);
        WriteGMII16( 0x15, 0x0290);
        WriteGMII16( 0x19, 0x5041);
        WriteGMII16( 0x15, 0x0291);
        WriteGMII16( 0x19, 0x6b00);
        WriteGMII16( 0x15, 0x0292);
        WriteGMII16( 0x19, 0x31f5);
        WriteGMII16( 0x15, 0x0293);
        WriteGMII16( 0x19, 0xb619);
        WriteGMII16( 0x15, 0x0294);
        WriteGMII16( 0x19, 0x80d9);
        WriteGMII16( 0x15, 0x0295);
        WriteGMII16( 0x19, 0xbd06);
        WriteGMII16( 0x15, 0x0296);
        WriteGMII16( 0x19, 0xbb0d);
        WriteGMII16( 0x15, 0x0297);
        WriteGMII16( 0x19, 0xaf14);
        WriteGMII16( 0x15, 0x0298);
        WriteGMII16( 0x19, 0x8efa);
        WriteGMII16( 0x15, 0x0299);
        WriteGMII16( 0x19, 0x5049);
        WriteGMII16( 0x15, 0x029a);
        WriteGMII16( 0x19, 0x3248);
        WriteGMII16( 0x15, 0x029b);
        WriteGMII16( 0x19, 0x4c10);
        WriteGMII16( 0x15, 0x029c);
        WriteGMII16( 0x19, 0x44b0);
        WriteGMII16( 0x15, 0x029d);
        WriteGMII16( 0x19, 0x4c00);
        WriteGMII16( 0x15, 0x029e);
        WriteGMII16( 0x19, 0x3292);
        WriteGMII16( 0x15, 0x029f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02a0);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02a1);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02a2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02a3);
        WriteGMII16( 0x19, 0x481f);
        WriteGMII16( 0x15, 0x02a4);
        WriteGMII16( 0x19, 0x5005);
        WriteGMII16( 0x15, 0x02a5);
        WriteGMII16( 0x19, 0x480f);
        WriteGMII16( 0x15, 0x02a6);
        WriteGMII16( 0x19, 0xac00);
        WriteGMII16( 0x15, 0x02a7);
        WriteGMII16( 0x19, 0x31a6);
        WriteGMII16( 0x15, 0x02a8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02a9);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02aa);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02ab);
        WriteGMII16( 0x19, 0x31ba);
        WriteGMII16( 0x15, 0x02ac);
        WriteGMII16( 0x19, 0x31d5);
        WriteGMII16( 0x15, 0x02ad);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02ae);
        WriteGMII16( 0x19, 0x5cf0);
        WriteGMII16( 0x15, 0x02af);
        WriteGMII16( 0x19, 0x588c);
        WriteGMII16( 0x15, 0x02b0);
        WriteGMII16( 0x19, 0x542f);
        WriteGMII16( 0x15, 0x02b1);
        WriteGMII16( 0x19, 0x7ffb);
        WriteGMII16( 0x15, 0x02b2);
        WriteGMII16( 0x19, 0x6ff8);
        WriteGMII16( 0x15, 0x02b3);
        WriteGMII16( 0x19, 0x64a4);
        WriteGMII16( 0x15, 0x02b4);
        WriteGMII16( 0x19, 0x64a0);
        WriteGMII16( 0x15, 0x02b5);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x02b6);
        WriteGMII16( 0x19, 0x4400);
        WriteGMII16( 0x15, 0x02b7);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x02b8);
        WriteGMII16( 0x19, 0x4480);
        WriteGMII16( 0x15, 0x02b9);
        WriteGMII16( 0x19, 0x9e00);
        WriteGMII16( 0x15, 0x02ba);
        WriteGMII16( 0x19, 0x4891);
        WriteGMII16( 0x15, 0x02bb);
        WriteGMII16( 0x19, 0x4cc0);
        WriteGMII16( 0x15, 0x02bc);
        WriteGMII16( 0x19, 0x4801);
        WriteGMII16( 0x15, 0x02bd);
        WriteGMII16( 0x19, 0xa609);
        WriteGMII16( 0x15, 0x02be);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x02bf);
        WriteGMII16( 0x19, 0x004e);
        WriteGMII16( 0x15, 0x02c0);
        WriteGMII16( 0x19, 0x87fe);
        WriteGMII16( 0x15, 0x02c1);
        WriteGMII16( 0x19, 0x32c6);
        WriteGMII16( 0x15, 0x02c2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02c3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02c4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02c5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02c6);
        WriteGMII16( 0x19, 0x48b2);
        WriteGMII16( 0x15, 0x02c7);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x02c8);
        WriteGMII16( 0x19, 0x4822);
        WriteGMII16( 0x15, 0x02c9);
        WriteGMII16( 0x19, 0x4488);
        WriteGMII16( 0x15, 0x02ca);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x02cb);
        WriteGMII16( 0x19, 0x0042);
        WriteGMII16( 0x15, 0x02cc);
        WriteGMII16( 0x19, 0x8203);
        WriteGMII16( 0x15, 0x02cd);
        WriteGMII16( 0x19, 0x4cc8);
        WriteGMII16( 0x15, 0x02ce);
        WriteGMII16( 0x19, 0x32d0);
        WriteGMII16( 0x15, 0x02cf);
        WriteGMII16( 0x19, 0x4cc0);
        WriteGMII16( 0x15, 0x02d0);
        WriteGMII16( 0x19, 0xc4d4);
        WriteGMII16( 0x15, 0x02d1);
        WriteGMII16( 0x19, 0x00f9);
        WriteGMII16( 0x15, 0x02d2);
        WriteGMII16( 0x19, 0xa51a);
        WriteGMII16( 0x15, 0x02d3);
        WriteGMII16( 0x19, 0x32d9);
        WriteGMII16( 0x15, 0x02d4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02d5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02d6);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02d7);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02d8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02d9);
        WriteGMII16( 0x19, 0x48b3);
        WriteGMII16( 0x15, 0x02da);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x02db);
        WriteGMII16( 0x19, 0x4823);
        WriteGMII16( 0x15, 0x02dc);
        WriteGMII16( 0x19, 0x4410);
        WriteGMII16( 0x15, 0x02dd);
        WriteGMII16( 0x19, 0xb630);
        WriteGMII16( 0x15, 0x02de);
        WriteGMII16( 0x19, 0x7dc8);
        WriteGMII16( 0x15, 0x02df);
        WriteGMII16( 0x19, 0x8203);
        WriteGMII16( 0x15, 0x02e0);
        WriteGMII16( 0x19, 0x4c48);
        WriteGMII16( 0x15, 0x02e1);
        WriteGMII16( 0x19, 0x32e3);
        WriteGMII16( 0x15, 0x02e2);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x02e3);
        WriteGMII16( 0x19, 0x9bfa);
        WriteGMII16( 0x15, 0x02e4);
        WriteGMII16( 0x19, 0x84ca);
        WriteGMII16( 0x15, 0x02e5);
        WriteGMII16( 0x19, 0x85f8);
        WriteGMII16( 0x15, 0x02e6);
        WriteGMII16( 0x19, 0x32ec);
        WriteGMII16( 0x15, 0x02e7);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02e8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02e9);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02ea);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02eb);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x02ec);
        WriteGMII16( 0x19, 0x48d4);
        WriteGMII16( 0x15, 0x02ed);
        WriteGMII16( 0x19, 0x4020);
        WriteGMII16( 0x15, 0x02ee);
        WriteGMII16( 0x19, 0x4844);
        WriteGMII16( 0x15, 0x02ef);
        WriteGMII16( 0x19, 0x4420);
        WriteGMII16( 0x15, 0x02f0);
        WriteGMII16( 0x19, 0x6800);
        WriteGMII16( 0x15, 0x02f1);
        WriteGMII16( 0x19, 0x7dc0);
        WriteGMII16( 0x15, 0x02f2);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x02f3);
        WriteGMII16( 0x19, 0x7c0b);
        WriteGMII16( 0x15, 0x02f4);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x02f5);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x02f6);
        WriteGMII16( 0x19, 0x9cfd);
        WriteGMII16( 0x15, 0x02f7);
        WriteGMII16( 0x19, 0xb616);
        WriteGMII16( 0x15, 0x02f8);
        WriteGMII16( 0x19, 0xc42b);
        WriteGMII16( 0x15, 0x02f9);
        WriteGMII16( 0x19, 0x00e0);
        WriteGMII16( 0x15, 0x02fa);
        WriteGMII16( 0x19, 0xc455);
        WriteGMII16( 0x15, 0x02fb);
        WriteGMII16( 0x19, 0x00b3);
        WriteGMII16( 0x15, 0x02fc);
        WriteGMII16( 0x19, 0xb20a);
        WriteGMII16( 0x15, 0x02fd);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x02fe);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x02ff);
        WriteGMII16( 0x19, 0x8204);
        WriteGMII16( 0x15, 0x0300);
        WriteGMII16( 0x19, 0x7c04);
        WriteGMII16( 0x15, 0x0301);
        WriteGMII16( 0x19, 0x7404);
        WriteGMII16( 0x15, 0x0302);
        WriteGMII16( 0x19, 0x32f3);
        WriteGMII16( 0x15, 0x0303);
        WriteGMII16( 0x19, 0x7c04);
        WriteGMII16( 0x15, 0x0304);
        WriteGMII16( 0x19, 0x7400);
        WriteGMII16( 0x15, 0x0305);
        WriteGMII16( 0x19, 0x32f3);
        WriteGMII16( 0x15, 0x0306);
        WriteGMII16( 0x19, 0xefed);
        WriteGMII16( 0x15, 0x0307);
        WriteGMII16( 0x19, 0x3342);
        WriteGMII16( 0x15, 0x0308);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0309);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x030a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x030b);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x030c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x030d);
        WriteGMII16( 0x19, 0x3006);
        WriteGMII16( 0x15, 0x030e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x030f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0310);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0311);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0312);
        WriteGMII16( 0x19, 0xa207);
        WriteGMII16( 0x15, 0x0313);
        WriteGMII16( 0x19, 0x4c00);
        WriteGMII16( 0x15, 0x0314);
        WriteGMII16( 0x19, 0x3322);
        WriteGMII16( 0x15, 0x0315);
        WriteGMII16( 0x19, 0x4041);
        WriteGMII16( 0x15, 0x0316);
        WriteGMII16( 0x19, 0x7d07);
        WriteGMII16( 0x15, 0x0317);
        WriteGMII16( 0x19, 0x4502);
        WriteGMII16( 0x15, 0x0318);
        WriteGMII16( 0x19, 0x3322);
        WriteGMII16( 0x15, 0x0319);
        WriteGMII16( 0x19, 0x4c08);
        WriteGMII16( 0x15, 0x031a);
        WriteGMII16( 0x19, 0x3322);
        WriteGMII16( 0x15, 0x031b);
        WriteGMII16( 0x19, 0x7d80);
        WriteGMII16( 0x15, 0x031c);
        WriteGMII16( 0x19, 0x5180);
        WriteGMII16( 0x15, 0x031d);
        WriteGMII16( 0x19, 0x3320);
        WriteGMII16( 0x15, 0x031e);
        WriteGMII16( 0x19, 0x7d80);
        WriteGMII16( 0x15, 0x031f);
        WriteGMII16( 0x19, 0x5000);
        WriteGMII16( 0x15, 0x0320);
        WriteGMII16( 0x19, 0x7d07);
        WriteGMII16( 0x15, 0x0321);
        WriteGMII16( 0x19, 0x4402);
        WriteGMII16( 0x15, 0x0322);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0323);
        WriteGMII16( 0x19, 0x6c02);
        WriteGMII16( 0x15, 0x0324);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x0325);
        WriteGMII16( 0x19, 0xb30c);
        WriteGMII16( 0x15, 0x0326);
        WriteGMII16( 0x19, 0xb206);
        WriteGMII16( 0x15, 0x0327);
        WriteGMII16( 0x19, 0xb103);
        WriteGMII16( 0x15, 0x0328);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0329);
        WriteGMII16( 0x19, 0x32f6);
        WriteGMII16( 0x15, 0x032a);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x032b);
        WriteGMII16( 0x19, 0x3352);
        WriteGMII16( 0x15, 0x032c);
        WriteGMII16( 0x19, 0xb103);
        WriteGMII16( 0x15, 0x032d);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x032e);
        WriteGMII16( 0x19, 0x336a);
        WriteGMII16( 0x15, 0x032f);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0330);
        WriteGMII16( 0x19, 0x3382);
        WriteGMII16( 0x15, 0x0331);
        WriteGMII16( 0x19, 0xb206);
        WriteGMII16( 0x15, 0x0332);
        WriteGMII16( 0x19, 0xb103);
        WriteGMII16( 0x15, 0x0333);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0334);
        WriteGMII16( 0x19, 0x3395);
        WriteGMII16( 0x15, 0x0335);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0336);
        WriteGMII16( 0x19, 0x33c6);
        WriteGMII16( 0x15, 0x0337);
        WriteGMII16( 0x19, 0xb103);
        WriteGMII16( 0x15, 0x0338);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x0339);
        WriteGMII16( 0x19, 0x33d7);
        WriteGMII16( 0x15, 0x033a);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x033b);
        WriteGMII16( 0x19, 0x33f2);
        WriteGMII16( 0x15, 0x033c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x033d);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x033e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x033f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0340);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0341);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0342);
        WriteGMII16( 0x19, 0x49b5);
        WriteGMII16( 0x15, 0x0343);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x15, 0x0344);
        WriteGMII16( 0x19, 0x4d00);
        WriteGMII16( 0x15, 0x0345);
        WriteGMII16( 0x19, 0x6880);
        WriteGMII16( 0x15, 0x0346);
        WriteGMII16( 0x19, 0x7c08);
        WriteGMII16( 0x15, 0x0347);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x0348);
        WriteGMII16( 0x19, 0x4925);
        WriteGMII16( 0x15, 0x0349);
        WriteGMII16( 0x19, 0x403b);
        WriteGMII16( 0x15, 0x034a);
        WriteGMII16( 0x19, 0xa602);
        WriteGMII16( 0x15, 0x034b);
        WriteGMII16( 0x19, 0x402f);
        WriteGMII16( 0x15, 0x034c);
        WriteGMII16( 0x19, 0x4484);
        WriteGMII16( 0x15, 0x034d);
        WriteGMII16( 0x19, 0x40c8);
        WriteGMII16( 0x15, 0x034e);
        WriteGMII16( 0x19, 0x44c4);
        WriteGMII16( 0x15, 0x034f);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x0350);
        WriteGMII16( 0x19, 0x00bd);
        WriteGMII16( 0x15, 0x0351);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x0352);
        WriteGMII16( 0x19, 0xc8ed);
        WriteGMII16( 0x15, 0x0353);
        WriteGMII16( 0x19, 0x00fc);
        WriteGMII16( 0x15, 0x0354);
        WriteGMII16( 0x19, 0x8221);
        WriteGMII16( 0x15, 0x0355);
        WriteGMII16( 0x19, 0xd11d);
        WriteGMII16( 0x15, 0x0356);
        WriteGMII16( 0x19, 0x001f);
        WriteGMII16( 0x15, 0x0357);
        WriteGMII16( 0x19, 0xde18);
        WriteGMII16( 0x15, 0x0358);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x0359);
        WriteGMII16( 0x19, 0x91f6);
        WriteGMII16( 0x15, 0x035a);
        WriteGMII16( 0x19, 0x3360);
        WriteGMII16( 0x15, 0x035b);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x035c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x035d);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x035e);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x035f);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0360);
        WriteGMII16( 0x19, 0x4bb6);
        WriteGMII16( 0x15, 0x0361);
        WriteGMII16( 0x19, 0x4064);
        WriteGMII16( 0x15, 0x0362);
        WriteGMII16( 0x19, 0x4b26);
        WriteGMII16( 0x15, 0x0363);
        WriteGMII16( 0x19, 0x4410);
        WriteGMII16( 0x15, 0x0364);
        WriteGMII16( 0x19, 0x4006);
        WriteGMII16( 0x15, 0x0365);
        WriteGMII16( 0x19, 0x4490);
        WriteGMII16( 0x15, 0x0366);
        WriteGMII16( 0x19, 0x6900);
        WriteGMII16( 0x15, 0x0367);
        WriteGMII16( 0x19, 0xb6a6);
        WriteGMII16( 0x15, 0x0368);
        WriteGMII16( 0x19, 0x9e02);
        WriteGMII16( 0x15, 0x0369);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x036a);
        WriteGMII16( 0x19, 0xd11d);
        WriteGMII16( 0x15, 0x036b);
        WriteGMII16( 0x19, 0x000a);
        WriteGMII16( 0x15, 0x036c);
        WriteGMII16( 0x19, 0xbb0f);
        WriteGMII16( 0x15, 0x036d);
        WriteGMII16( 0x19, 0x8102);
        WriteGMII16( 0x15, 0x036e);
        WriteGMII16( 0x19, 0x3371);
        WriteGMII16( 0x15, 0x036f);
        WriteGMII16( 0x19, 0xa21e);
        WriteGMII16( 0x15, 0x0370);
        WriteGMII16( 0x19, 0x33b6);
        WriteGMII16( 0x15, 0x0371);
        WriteGMII16( 0x19, 0x91f6);
        WriteGMII16( 0x15, 0x0372);
        WriteGMII16( 0x19, 0xc218);
        WriteGMII16( 0x15, 0x0373);
        WriteGMII16( 0x19, 0x00f4);
        WriteGMII16( 0x15, 0x0374);
        WriteGMII16( 0x19, 0x33b6);
        WriteGMII16( 0x15, 0x0375);
        WriteGMII16( 0x19, 0x32ec);
        WriteGMII16( 0x15, 0x0376);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0377);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0378);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x0379);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x037a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x037b);
        WriteGMII16( 0x19, 0x4b97);
        WriteGMII16( 0x15, 0x037c);
        WriteGMII16( 0x19, 0x402b);
        WriteGMII16( 0x15, 0x037d);
        WriteGMII16( 0x19, 0x4b07);
        WriteGMII16( 0x15, 0x037e);
        WriteGMII16( 0x19, 0x4422);
        WriteGMII16( 0x15, 0x037f);
        WriteGMII16( 0x19, 0x6980);
        WriteGMII16( 0x15, 0x0380);
        WriteGMII16( 0x19, 0xb608);
        WriteGMII16( 0x15, 0x0381);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x0382);
        WriteGMII16( 0x19, 0xbc05);
        WriteGMII16( 0x15, 0x0383);
        WriteGMII16( 0x19, 0xc21c);
        WriteGMII16( 0x15, 0x0384);
        WriteGMII16( 0x19, 0x0032);
        WriteGMII16( 0x15, 0x0385);
        WriteGMII16( 0x19, 0xa1fb);
        WriteGMII16( 0x15, 0x0386);
        WriteGMII16( 0x19, 0x338d);
        WriteGMII16( 0x15, 0x0387);
        WriteGMII16( 0x19, 0x32ae);
        WriteGMII16( 0x15, 0x0388);
        WriteGMII16( 0x19, 0x330d);
        WriteGMII16( 0x15, 0x0389);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x038a);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x038b);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x038c);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x038d);
        WriteGMII16( 0x19, 0x4b97);
        WriteGMII16( 0x15, 0x038e);
        WriteGMII16( 0x19, 0x6a08);
        WriteGMII16( 0x15, 0x038f);
        WriteGMII16( 0x19, 0x4b07);
        WriteGMII16( 0x15, 0x0390);
        WriteGMII16( 0x19, 0x40ac);
        WriteGMII16( 0x15, 0x0391);
        WriteGMII16( 0x19, 0x4445);
        WriteGMII16( 0x15, 0x0392);
        WriteGMII16( 0x19, 0x404e);
        WriteGMII16( 0x15, 0x0393);
        WriteGMII16( 0x19, 0x4461);
        WriteGMII16( 0x15, 0x0394);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x0395);
        WriteGMII16( 0x19, 0x9c0a);
        WriteGMII16( 0x15, 0x0396);
        WriteGMII16( 0x19, 0x63da);
        WriteGMII16( 0x15, 0x0397);
        WriteGMII16( 0x19, 0x6f0c);
        WriteGMII16( 0x15, 0x0398);
        WriteGMII16( 0x19, 0x5440);
        WriteGMII16( 0x15, 0x0399);
        WriteGMII16( 0x19, 0x4b98);
        WriteGMII16( 0x15, 0x039a);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x039b);
        WriteGMII16( 0x19, 0x4c00);
        WriteGMII16( 0x15, 0x039c);
        WriteGMII16( 0x19, 0x4b08);
        WriteGMII16( 0x15, 0x039d);
        WriteGMII16( 0x19, 0x63d8);
        WriteGMII16( 0x15, 0x039e);
        WriteGMII16( 0x19, 0x33a5);
        WriteGMII16( 0x15, 0x039f);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x03a0);
        WriteGMII16( 0x19, 0x00e8);
        WriteGMII16( 0x15, 0x03a1);
        WriteGMII16( 0x19, 0x820e);
        WriteGMII16( 0x15, 0x03a2);
        WriteGMII16( 0x19, 0xa10d);
        WriteGMII16( 0x15, 0x03a3);
        WriteGMII16( 0x19, 0x9df1);
        WriteGMII16( 0x15, 0x03a4);
        WriteGMII16( 0x19, 0x33af);
        WriteGMII16( 0x15, 0x03a5);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x03a6);
        WriteGMII16( 0x19, 0x00f9);
        WriteGMII16( 0x15, 0x03a7);
        WriteGMII16( 0x19, 0xc017);
        WriteGMII16( 0x15, 0x03a8);
        WriteGMII16( 0x19, 0x0007);
        WriteGMII16( 0x15, 0x03a9);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x03aa);
        WriteGMII16( 0x19, 0x6c03);
        WriteGMII16( 0x15, 0x03ab);
        WriteGMII16( 0x19, 0xa104);
        WriteGMII16( 0x15, 0x03ac);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x03ad);
        WriteGMII16( 0x19, 0x6c00);
        WriteGMII16( 0x15, 0x03ae);
        WriteGMII16( 0x19, 0x9df7);
        WriteGMII16( 0x15, 0x03af);
        WriteGMII16( 0x19, 0x7c03);
        WriteGMII16( 0x15, 0x03b0);
        WriteGMII16( 0x19, 0x6c08);
        WriteGMII16( 0x15, 0x03b1);
        WriteGMII16( 0x19, 0x33b6);
        WriteGMII16( 0x15, 0x03b2);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03b3);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03b4);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03b5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03b6);
        WriteGMII16( 0x19, 0x55af);
        WriteGMII16( 0x15, 0x03b7);
        WriteGMII16( 0x19, 0x7ff0);
        WriteGMII16( 0x15, 0x03b8);
        WriteGMII16( 0x19, 0x6ff0);
        WriteGMII16( 0x15, 0x03b9);
        WriteGMII16( 0x19, 0x4bb9);
        WriteGMII16( 0x15, 0x03ba);
        WriteGMII16( 0x19, 0x6a80);
        WriteGMII16( 0x15, 0x03bb);
        WriteGMII16( 0x19, 0x4b29);
        WriteGMII16( 0x15, 0x03bc);
        WriteGMII16( 0x19, 0x4041);
        WriteGMII16( 0x15, 0x03bd);
        WriteGMII16( 0x19, 0x440a);
        WriteGMII16( 0x15, 0x03be);
        WriteGMII16( 0x19, 0x4029);
        WriteGMII16( 0x15, 0x03bf);
        WriteGMII16( 0x19, 0x4418);
        WriteGMII16( 0x15, 0x03c0);
        WriteGMII16( 0x19, 0x4090);
        WriteGMII16( 0x15, 0x03c1);
        WriteGMII16( 0x19, 0x4438);
        WriteGMII16( 0x15, 0x03c2);
        WriteGMII16( 0x19, 0x40c4);
        WriteGMII16( 0x15, 0x03c3);
        WriteGMII16( 0x19, 0x447b);
        WriteGMII16( 0x15, 0x03c4);
        WriteGMII16( 0x19, 0xb6c4);
        WriteGMII16( 0x15, 0x03c5);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x03c6);
        WriteGMII16( 0x19, 0x9bfe);
        WriteGMII16( 0x15, 0x03c7);
        WriteGMII16( 0x19, 0x33cc);
        WriteGMII16( 0x15, 0x03c8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03c9);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03ca);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03cb);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03cc);
        WriteGMII16( 0x19, 0x542f);
        WriteGMII16( 0x15, 0x03cd);
        WriteGMII16( 0x19, 0x499a);
        WriteGMII16( 0x15, 0x03ce);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x03cf);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x03d0);
        WriteGMII16( 0x19, 0x490a);
        WriteGMII16( 0x15, 0x03d1);
        WriteGMII16( 0x19, 0x405e);
        WriteGMII16( 0x15, 0x03d2);
        WriteGMII16( 0x19, 0x44f8);
        WriteGMII16( 0x15, 0x03d3);
        WriteGMII16( 0x19, 0x6b00);
        WriteGMII16( 0x15, 0x03d4);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x03d5);
        WriteGMII16( 0x19, 0x0028);
        WriteGMII16( 0x15, 0x03d6);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x03d7);
        WriteGMII16( 0x19, 0xbd27);
        WriteGMII16( 0x15, 0x03d8);
        WriteGMII16( 0x19, 0x9cfc);
        WriteGMII16( 0x15, 0x03d9);
        WriteGMII16( 0x19, 0xc639);
        WriteGMII16( 0x15, 0x03da);
        WriteGMII16( 0x19, 0x000f);
        WriteGMII16( 0x15, 0x03db);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x03dc);
        WriteGMII16( 0x19, 0x7c01);
        WriteGMII16( 0x15, 0x03dd);
        WriteGMII16( 0x19, 0x4c01);
        WriteGMII16( 0x15, 0x03de);
        WriteGMII16( 0x19, 0x9af6);
        WriteGMII16( 0x15, 0x03df);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03e0);
        WriteGMII16( 0x19, 0x4c52);
        WriteGMII16( 0x15, 0x03e1);
        WriteGMII16( 0x19, 0x4470);
        WriteGMII16( 0x15, 0x03e2);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03e3);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x03e4);
        WriteGMII16( 0x19, 0x33d4);
        WriteGMII16( 0x15, 0x03e5);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03e6);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03e7);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03e8);
        WriteGMII16( 0x19, 0x0000);
        WriteGMII16( 0x15, 0x03e9);
        WriteGMII16( 0x19, 0x49bb);
        WriteGMII16( 0x15, 0x03ea);
        WriteGMII16( 0x19, 0x4478);
        WriteGMII16( 0x15, 0x03eb);
        WriteGMII16( 0x19, 0x492b);
        WriteGMII16( 0x15, 0x03ec);
        WriteGMII16( 0x19, 0x6b80);
        WriteGMII16( 0x15, 0x03ed);
        WriteGMII16( 0x19, 0x7c01);
        WriteGMII16( 0x15, 0x03ee);
        WriteGMII16( 0x19, 0x4c00);
        WriteGMII16( 0x15, 0x03ef);
        WriteGMII16( 0x19, 0xd64f);
        WriteGMII16( 0x15, 0x03f0);
        WriteGMII16( 0x19, 0x000d);
        WriteGMII16( 0x15, 0x03f1);
        WriteGMII16( 0x19, 0x3311);
        WriteGMII16( 0x15, 0x03f2);
        WriteGMII16( 0x19, 0xbd0c);
        WriteGMII16( 0x15, 0x03f3);
        WriteGMII16( 0x19, 0xc428);
        WriteGMII16( 0x15, 0x03f4);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x15, 0x03f5);
        WriteGMII16( 0x19, 0x9afa);
        WriteGMII16( 0x15, 0x03f6);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03f7);
        WriteGMII16( 0x19, 0x4c52);
        WriteGMII16( 0x15, 0x03f8);
        WriteGMII16( 0x19, 0x4470);
        WriteGMII16( 0x15, 0x03f9);
        WriteGMII16( 0x19, 0x7c12);
        WriteGMII16( 0x15, 0x03fa);
        WriteGMII16( 0x19, 0x4c40);
        WriteGMII16( 0x15, 0x03fb);
        WriteGMII16( 0x19, 0x33ef);
        WriteGMII16( 0x15, 0x03fc);
        WriteGMII16( 0x19, 0x3342);
        WriteGMII16( 0x15, 0x03fd);
        WriteGMII16( 0x19, 0x330d);
        WriteGMII16( 0x15, 0x03fe);
        WriteGMII16( 0x19, 0x32ae);
        WriteGMII16( 0x15, 0x0000);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0002);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x48f7);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xa080);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xf602);
        WriteGMII16( 0x06, 0x0112);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x1f02);
        WriteGMII16( 0x06, 0x012c);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x3c02);
        WriteGMII16( 0x06, 0x0156);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x6d02);
        WriteGMII16( 0x06, 0x809d);
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
        WriteGMII16( 0x06, 0xaebb);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xc702);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd105);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xcd02);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xca02);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd105);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xd002);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd481);
        WriteGMII16( 0x06, 0xc9e4);
        WriteGMII16( 0x06, 0x8b90);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x91d4);
        WriteGMII16( 0x06, 0x81b8);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x92e5);
        WriteGMII16( 0x06, 0x8b93);
        WriteGMII16( 0x06, 0xbf8b);
        WriteGMII16( 0x06, 0x88ec);
        WriteGMII16( 0x06, 0x0019);
        WriteGMII16( 0x06, 0xa98b);
        WriteGMII16( 0x06, 0x90f9);
        WriteGMII16( 0x06, 0xeeff);
        WriteGMII16( 0x06, 0xf600);
        WriteGMII16( 0x06, 0xeeff);
        WriteGMII16( 0x06, 0xf7fc);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xc102);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xc402);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x201a);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x824b);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x1902);
        WriteGMII16( 0x06, 0x2c9d);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0x9602);
        WriteGMII16( 0x06, 0x0473);
        WriteGMII16( 0x06, 0x022e);
        WriteGMII16( 0x06, 0x3902);
        WriteGMII16( 0x06, 0x044d);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x210b);
        WriteGMII16( 0x06, 0xf621);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x0416);
        WriteGMII16( 0x06, 0x021b);
        WriteGMII16( 0x06, 0xa4e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x22e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2305);
        WriteGMII16( 0x06, 0xf623);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8ee0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x24e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2505);
        WriteGMII16( 0x06, 0xf625);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8ee0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x26e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0xdae0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x27e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0x5cfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad21);
        WriteGMII16( 0x06, 0x57e0);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x2358);
        WriteGMII16( 0x06, 0xc059);
        WriteGMII16( 0x06, 0x021e);
        WriteGMII16( 0x06, 0x01e1);
        WriteGMII16( 0x06, 0x8b3c);
        WriteGMII16( 0x06, 0x1f10);
        WriteGMII16( 0x06, 0x9e44);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x3cad);
        WriteGMII16( 0x06, 0x211d);
        WriteGMII16( 0x06, 0xe18b);
        WriteGMII16( 0x06, 0x84f7);
        WriteGMII16( 0x06, 0x29e5);
        WriteGMII16( 0x06, 0x8b84);
        WriteGMII16( 0x06, 0xac27);
        WriteGMII16( 0x06, 0x0dac);
        WriteGMII16( 0x06, 0x2605);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x7fae);
        WriteGMII16( 0x06, 0x2b02);
        WriteGMII16( 0x06, 0x2c23);
        WriteGMII16( 0x06, 0xae26);
        WriteGMII16( 0x06, 0x022c);
        WriteGMII16( 0x06, 0x41ae);
        WriteGMII16( 0x06, 0x21e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x18e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0x58fc);
        WriteGMII16( 0x06, 0xe4ff);
        WriteGMII16( 0x06, 0xf7d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x2eee);
        WriteGMII16( 0x06, 0x0232);
        WriteGMII16( 0x06, 0x0ad1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x82e8);
        WriteGMII16( 0x06, 0x0232);
        WriteGMII16( 0x06, 0x0a02);
        WriteGMII16( 0x06, 0x2bdf);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefc);
        WriteGMII16( 0x06, 0x04d0);
        WriteGMII16( 0x06, 0x0202);
        WriteGMII16( 0x06, 0x1e97);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x2228);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xd302);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd10c);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xd602);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd104);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xd902);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xe802);
        WriteGMII16( 0x06, 0x320a);
        WriteGMII16( 0x06, 0xe0ff);
        WriteGMII16( 0x06, 0xf768);
        WriteGMII16( 0x06, 0x03e4);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xd004);
        WriteGMII16( 0x06, 0x0228);
        WriteGMII16( 0x06, 0x7a04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0xe234);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0x35f6);
        WriteGMII16( 0x06, 0x2be4);
        WriteGMII16( 0x06, 0xe234);
        WriteGMII16( 0x06, 0xe5e2);
        WriteGMII16( 0x06, 0x35fc);
        WriteGMII16( 0x06, 0x05f8);
        WriteGMII16( 0x06, 0xe0e2);
        WriteGMII16( 0x06, 0x34e1);
        WriteGMII16( 0x06, 0xe235);
        WriteGMII16( 0x06, 0xf72b);
        WriteGMII16( 0x06, 0xe4e2);
        WriteGMII16( 0x06, 0x34e5);
        WriteGMII16( 0x06, 0xe235);
        WriteGMII16( 0x06, 0xfc05);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69ac);
        WriteGMII16( 0x06, 0x1b4c);
        WriteGMII16( 0x06, 0xbf2e);
        WriteGMII16( 0x06, 0x3002);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0xef01);
        WriteGMII16( 0x06, 0xe28a);
        WriteGMII16( 0x06, 0x76e4);
        WriteGMII16( 0x06, 0x8a76);
        WriteGMII16( 0x06, 0x1f12);
        WriteGMII16( 0x06, 0x9e3a);
        WriteGMII16( 0x06, 0xef12);
        WriteGMII16( 0x06, 0x5907);
        WriteGMII16( 0x06, 0x9f12);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf721);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40d0);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x287a);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0x34fc);
        WriteGMII16( 0x06, 0xa000);
        WriteGMII16( 0x06, 0x1002);
        WriteGMII16( 0x06, 0x2dc3);
        WriteGMII16( 0x06, 0x022e);
        WriteGMII16( 0x06, 0x21e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf621);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40ae);
        WriteGMII16( 0x06, 0x0fbf);
        WriteGMII16( 0x06, 0x3fa5);
        WriteGMII16( 0x06, 0x0231);
        WriteGMII16( 0x06, 0x6cbf);
        WriteGMII16( 0x06, 0x3fa2);
        WriteGMII16( 0x06, 0x0231);
        WriteGMII16( 0x06, 0x6c02);
        WriteGMII16( 0x06, 0x2dc3);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0xe2f4);
        WriteGMII16( 0x06, 0xe1e2);
        WriteGMII16( 0x06, 0xf5e4);
        WriteGMII16( 0x06, 0x8a78);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0x79ee);
        WriteGMII16( 0x06, 0xe2f4);
        WriteGMII16( 0x06, 0xd8ee);
        WriteGMII16( 0x06, 0xe2f5);
        WriteGMII16( 0x06, 0x20fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x2065);
        WriteGMII16( 0x06, 0xd200);
        WriteGMII16( 0x06, 0xbf2e);
        WriteGMII16( 0x06, 0xe802);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0x1e21);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xdf02);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0x0c11);
        WriteGMII16( 0x06, 0x1e21);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xe202);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0x0c12);
        WriteGMII16( 0x06, 0x1e21);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xe502);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0x0c13);
        WriteGMII16( 0x06, 0x1e21);
        WriteGMII16( 0x06, 0xbf1f);
        WriteGMII16( 0x06, 0x5302);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0x0c14);
        WriteGMII16( 0x06, 0x1e21);
        WriteGMII16( 0x06, 0xbf82);
        WriteGMII16( 0x06, 0xeb02);
        WriteGMII16( 0x06, 0x31dd);
        WriteGMII16( 0x06, 0x0c16);
        WriteGMII16( 0x06, 0x1e21);
        WriteGMII16( 0x06, 0xe083);
        WriteGMII16( 0x06, 0xe01f);
        WriteGMII16( 0x06, 0x029e);
        WriteGMII16( 0x06, 0x22e6);
        WriteGMII16( 0x06, 0x83e0);
        WriteGMII16( 0x06, 0xad31);
        WriteGMII16( 0x06, 0x14ad);
        WriteGMII16( 0x06, 0x3011);
        WriteGMII16( 0x06, 0xef02);
        WriteGMII16( 0x06, 0x580c);
        WriteGMII16( 0x06, 0x9e07);
        WriteGMII16( 0x06, 0xad36);
        WriteGMII16( 0x06, 0x085a);
        WriteGMII16( 0x06, 0x309f);
        WriteGMII16( 0x06, 0x04d1);
        WriteGMII16( 0x06, 0x01ae);
        WriteGMII16( 0x06, 0x02d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x82dc);
        WriteGMII16( 0x06, 0x0232);
        WriteGMII16( 0x06, 0x0aef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x0400);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0x77e1);
        WriteGMII16( 0x06, 0x4010);
        WriteGMII16( 0x06, 0xe150);
        WriteGMII16( 0x06, 0x32e1);
        WriteGMII16( 0x06, 0x5030);
        WriteGMII16( 0x06, 0xe144);
        WriteGMII16( 0x06, 0x74e1);
        WriteGMII16( 0x06, 0x44bb);
        WriteGMII16( 0x06, 0xe2d2);
        WriteGMII16( 0x06, 0x40e0);
        WriteGMII16( 0x06, 0x2cfc);
        WriteGMII16( 0x06, 0xe2cc);
        WriteGMII16( 0x06, 0xcce2);
        WriteGMII16( 0x06, 0x00cc);
        WriteGMII16( 0x06, 0xe000);
        WriteGMII16( 0x06, 0x99e0);
        WriteGMII16( 0x06, 0x3688);
        WriteGMII16( 0x06, 0xe036);
        WriteGMII16( 0x06, 0x99e1);
        WriteGMII16( 0x06, 0x40dd);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x05, 0xe142);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x05, 0xe140);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        gphy_val &= ~(BIT_0);
        gphy_val |= BIT_2;
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1f, 0x0002);
        WriteGMII16( 0x1f, 0x0000);
    } else if (mcfg == MCFG_8168E_VL_2) {
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x1800);
        gphy_val = ReadGMII16( 0x15);
        gphy_val &= ~(BIT_12);
        WriteGMII16( 0x15, gphy_val);
        WriteGMII16( 0x00, 0x4800);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x002f);
        for (i = 0; i < 1000; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x1c);
            if ((gphy_val & 0x0080) == 0x0080)
                break;
        }
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x1800);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x17);
            if (!(gphy_val & 0x0001))
                break;
        }
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0307);
        WriteGMII16( 0x15, 0x00AF);
        WriteGMII16( 0x19, 0x4060);
        WriteGMII16( 0x15, 0x00B0);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x00B1);
        WriteGMII16( 0x19, 0x7e00);
        WriteGMII16( 0x15, 0x00B2);
        WriteGMII16( 0x19, 0x72B0);
        WriteGMII16( 0x15, 0x00B3);
        WriteGMII16( 0x19, 0x7F00);
        WriteGMII16( 0x15, 0x00B4);
        WriteGMII16( 0x19, 0x73B0);
        WriteGMII16( 0x15, 0x0101);
        WriteGMII16( 0x19, 0x0005);
        WriteGMII16( 0x15, 0x0103);
        WriteGMII16( 0x19, 0x0003);
        WriteGMII16( 0x15, 0x0105);
        WriteGMII16( 0x19, 0x30FD);
        WriteGMII16( 0x15, 0x0106);
        WriteGMII16( 0x19, 0x9DF7);
        WriteGMII16( 0x15, 0x0107);
        WriteGMII16( 0x19, 0x30C6);
        WriteGMII16( 0x15, 0x0098);
        WriteGMII16( 0x19, 0x7c0b);
        WriteGMII16( 0x15, 0x0099);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00eb);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00f8);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00fe);
        WriteGMII16( 0x19, 0x6f0f);
        WriteGMII16( 0x15, 0x00db);
        WriteGMII16( 0x19, 0x6f09);
        WriteGMII16( 0x15, 0x00dc);
        WriteGMII16( 0x19, 0xaefd);
        WriteGMII16( 0x15, 0x00dd);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00de);
        WriteGMII16( 0x19, 0xc60b);
        WriteGMII16( 0x15, 0x00df);
        WriteGMII16( 0x19, 0x00fa);
        WriteGMII16( 0x15, 0x00e0);
        WriteGMII16( 0x19, 0x30e1);
        WriteGMII16( 0x15, 0x020c);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x020e);
        WriteGMII16( 0x19, 0x9813);
        WriteGMII16( 0x15, 0x020f);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0210);
        WriteGMII16( 0x19, 0x930f);
        WriteGMII16( 0x15, 0x0211);
        WriteGMII16( 0x19, 0x9206);
        WriteGMII16( 0x15, 0x0212);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0213);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0214);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0215);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0216);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0217);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0218);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0219);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x021a);
        WriteGMII16( 0x19, 0x5540);
        WriteGMII16( 0x15, 0x021b);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x021c);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x021d);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x021e);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x021f);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0220);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0221);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x0222);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0223);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x0224);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0225);
        WriteGMII16( 0x19, 0x3231);
        WriteGMII16( 0x15, 0x0000);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0002);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x17, 0x2160);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0040);
        WriteGMII16( 0x18, 0x0004);
        if (pdev->subsystem_vendor == 0x144d &&
            pdev->subsystem_device == 0xc0a6) {
            WriteGMII16( 0x18, 0x0724);
            WriteGMII16( 0x19, 0xfe00);
            WriteGMII16( 0x18, 0x0734);
            WriteGMII16( 0x19, 0xfd00);
            WriteGMII16( 0x18, 0x1824);
            WriteGMII16( 0x19, 0xfc00);
            WriteGMII16( 0x18, 0x1834);
            WriteGMII16( 0x19, 0xfd00);
        }
        WriteGMII16( 0x18, 0x09d4);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x09e4);
        WriteGMII16( 0x19, 0x0800);
        WriteGMII16( 0x18, 0x09f4);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x0a04);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x0a14);
        WriteGMII16( 0x19, 0x0c00);
        WriteGMII16( 0x18, 0x0a24);
        WriteGMII16( 0x19, 0xff00);
        WriteGMII16( 0x18, 0x0a74);
        WriteGMII16( 0x19, 0xf600);
        WriteGMII16( 0x18, 0x1a24);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x18, 0x1a64);
        WriteGMII16( 0x19, 0x0500);
        WriteGMII16( 0x18, 0x1a74);
        WriteGMII16( 0x19, 0x9500);
        WriteGMII16( 0x18, 0x1a84);
        WriteGMII16( 0x19, 0x8000);
        WriteGMII16( 0x18, 0x1a94);
        WriteGMII16( 0x19, 0x7d00);
        WriteGMII16( 0x18, 0x1aa4);
        WriteGMII16( 0x19, 0x9600);
        WriteGMII16( 0x18, 0x1ac4);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x1ad4);
        WriteGMII16( 0x19, 0x0800);
        WriteGMII16( 0x18, 0x1af4);
        WriteGMII16( 0x19, 0xc400);
        WriteGMII16( 0x18, 0x1b04);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x1b14);
        WriteGMII16( 0x19, 0x0800);
        WriteGMII16( 0x18, 0x1b24);
        WriteGMII16( 0x19, 0xfd00);
        WriteGMII16( 0x18, 0x1b34);
        WriteGMII16( 0x19, 0x4000);
        WriteGMII16( 0x18, 0x1b44);
        WriteGMII16( 0x19, 0x0400);
        WriteGMII16( 0x18, 0x1b94);
        WriteGMII16( 0x19, 0xf100);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x17, 0x2100);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0040);
        WriteGMII16( 0x18, 0x0000);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x48f7);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xa080);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xf602);
        WriteGMII16( 0x06, 0x0115);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x2202);
        WriteGMII16( 0x06, 0x80a0);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x3f02);
        WriteGMII16( 0x06, 0x0159);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0xbd02);
        WriteGMII16( 0x06, 0x80da);
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
        WriteGMII16( 0x06, 0xaebb);
        WriteGMII16( 0x06, 0xd481);
        WriteGMII16( 0x06, 0xd2e4);
        WriteGMII16( 0x06, 0x8b92);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x93d1);
        WriteGMII16( 0x06, 0x03bf);
        WriteGMII16( 0x06, 0x859e);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23d1);
        WriteGMII16( 0x06, 0x02bf);
        WriteGMII16( 0x06, 0x85a1);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23ee);
        WriteGMII16( 0x06, 0x8608);
        WriteGMII16( 0x06, 0x03ee);
        WriteGMII16( 0x06, 0x860a);
        WriteGMII16( 0x06, 0x60ee);
        WriteGMII16( 0x06, 0x8610);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8611);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x07ee);
        WriteGMII16( 0x06, 0x8abf);
        WriteGMII16( 0x06, 0x73ee);
        WriteGMII16( 0x06, 0x8a95);
        WriteGMII16( 0x06, 0x02bf);
        WriteGMII16( 0x06, 0x8b88);
        WriteGMII16( 0x06, 0xec00);
        WriteGMII16( 0x06, 0x19a9);
        WriteGMII16( 0x06, 0x8b90);
        WriteGMII16( 0x06, 0xf9ee);
        WriteGMII16( 0x06, 0xfff6);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xfed1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x8595);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23d1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x8598);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x2304);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8a);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x14ee);
        WriteGMII16( 0x06, 0x8b8a);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x1f9a);
        WriteGMII16( 0x06, 0xe0e4);
        WriteGMII16( 0x06, 0x26e1);
        WriteGMII16( 0x06, 0xe427);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x2623);
        WriteGMII16( 0x06, 0xe5e4);
        WriteGMII16( 0x06, 0x27fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8dad);
        WriteGMII16( 0x06, 0x2014);
        WriteGMII16( 0x06, 0xee8b);
        WriteGMII16( 0x06, 0x8d00);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0x5a78);
        WriteGMII16( 0x06, 0x039e);
        WriteGMII16( 0x06, 0x0902);
        WriteGMII16( 0x06, 0x05db);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0x7b02);
        WriteGMII16( 0x06, 0x3231);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x1df6);
        WriteGMII16( 0x06, 0x20e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x5c02);
        WriteGMII16( 0x06, 0x2bcb);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x2902);
        WriteGMII16( 0x06, 0x03b4);
        WriteGMII16( 0x06, 0x0285);
        WriteGMII16( 0x06, 0x6402);
        WriteGMII16( 0x06, 0x2eca);
        WriteGMII16( 0x06, 0x0284);
        WriteGMII16( 0x06, 0xcd02);
        WriteGMII16( 0x06, 0x046f);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x210b);
        WriteGMII16( 0x06, 0xf621);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x8520);
        WriteGMII16( 0x06, 0x021b);
        WriteGMII16( 0x06, 0xe8e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x22e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2308);
        WriteGMII16( 0x06, 0xf623);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x311c);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2405);
        WriteGMII16( 0x06, 0xf624);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8ee0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x25e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2608);
        WriteGMII16( 0x06, 0xf626);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x2df5);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2705);
        WriteGMII16( 0x06, 0xf627);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x037a);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x65d2);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x2fe9);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf61e);
        WriteGMII16( 0x06, 0x21bf);
        WriteGMII16( 0x06, 0x2ff5);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf60c);
        WriteGMII16( 0x06, 0x111e);
        WriteGMII16( 0x06, 0x21bf);
        WriteGMII16( 0x06, 0x2ff8);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf60c);
        WriteGMII16( 0x06, 0x121e);
        WriteGMII16( 0x06, 0x21bf);
        WriteGMII16( 0x06, 0x2ffb);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf60c);
        WriteGMII16( 0x06, 0x131e);
        WriteGMII16( 0x06, 0x21bf);
        WriteGMII16( 0x06, 0x1f97);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf60c);
        WriteGMII16( 0x06, 0x141e);
        WriteGMII16( 0x06, 0x21bf);
        WriteGMII16( 0x06, 0x859b);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf60c);
        WriteGMII16( 0x06, 0x161e);
        WriteGMII16( 0x06, 0x21e0);
        WriteGMII16( 0x06, 0x8a8c);
        WriteGMII16( 0x06, 0x1f02);
        WriteGMII16( 0x06, 0x9e22);
        WriteGMII16( 0x06, 0xe68a);
        WriteGMII16( 0x06, 0x8cad);
        WriteGMII16( 0x06, 0x3114);
        WriteGMII16( 0x06, 0xad30);
        WriteGMII16( 0x06, 0x11ef);
        WriteGMII16( 0x06, 0x0258);
        WriteGMII16( 0x06, 0x0c9e);
        WriteGMII16( 0x06, 0x07ad);
        WriteGMII16( 0x06, 0x3608);
        WriteGMII16( 0x06, 0x5a30);
        WriteGMII16( 0x06, 0x9f04);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xae02);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf2f);
        WriteGMII16( 0x06, 0xf202);
        WriteGMII16( 0x06, 0x3723);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xface);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69fa);
        WriteGMII16( 0x06, 0xd401);
        WriteGMII16( 0x06, 0x55b4);
        WriteGMII16( 0x06, 0xfebf);
        WriteGMII16( 0x06, 0x85a7);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf6ac);
        WriteGMII16( 0x06, 0x280b);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0xa402);
        WriteGMII16( 0x06, 0x36f6);
        WriteGMII16( 0x06, 0xac28);
        WriteGMII16( 0x06, 0x49ae);
        WriteGMII16( 0x06, 0x64bf);
        WriteGMII16( 0x06, 0x85a4);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf6ac);
        WriteGMII16( 0x06, 0x285b);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0x60ac);
        WriteGMII16( 0x06, 0x2105);
        WriteGMII16( 0x06, 0xac22);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x4ebf);
        WriteGMII16( 0x06, 0xe0c4);
        WriteGMII16( 0x06, 0xbe86);
        WriteGMII16( 0x06, 0x14d2);
        WriteGMII16( 0x06, 0x04d8);
        WriteGMII16( 0x06, 0x19d9);
        WriteGMII16( 0x06, 0x1907);
        WriteGMII16( 0x06, 0xdc19);
        WriteGMII16( 0x06, 0xdd19);
        WriteGMII16( 0x06, 0x0789);
        WriteGMII16( 0x06, 0x89ef);
        WriteGMII16( 0x06, 0x645e);
        WriteGMII16( 0x06, 0x07ff);
        WriteGMII16( 0x06, 0x0d65);
        WriteGMII16( 0x06, 0x5cf8);
        WriteGMII16( 0x06, 0x001e);
        WriteGMII16( 0x06, 0x46dc);
        WriteGMII16( 0x06, 0x19dd);
        WriteGMII16( 0x06, 0x19b2);
        WriteGMII16( 0x06, 0xe2d4);
        WriteGMII16( 0x06, 0x0001);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0xa402);
        WriteGMII16( 0x06, 0x3723);
        WriteGMII16( 0x06, 0xae1d);
        WriteGMII16( 0x06, 0xbee0);
        WriteGMII16( 0x06, 0xc4bf);
        WriteGMII16( 0x06, 0x8614);
        WriteGMII16( 0x06, 0xd204);
        WriteGMII16( 0x06, 0xd819);
        WriteGMII16( 0x06, 0xd919);
        WriteGMII16( 0x06, 0x07dc);
        WriteGMII16( 0x06, 0x19dd);
        WriteGMII16( 0x06, 0x1907);
        WriteGMII16( 0x06, 0xb2f4);
        WriteGMII16( 0x06, 0xd400);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x85a4);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23fe);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfec6);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc05);
        WriteGMII16( 0x06, 0xf9e2);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0xeb5a);
        WriteGMII16( 0x06, 0x070c);
        WriteGMII16( 0x06, 0x031e);
        WriteGMII16( 0x06, 0x20e6);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe7e0);
        WriteGMII16( 0x06, 0xebe0);
        WriteGMII16( 0x06, 0xe0fc);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0xfdfd);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac26);
        WriteGMII16( 0x06, 0x1ae0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x14e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xac20);
        WriteGMII16( 0x06, 0x0ee0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xac23);
        WriteGMII16( 0x06, 0x08e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xac24);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x3802);
        WriteGMII16( 0x06, 0x1ab5);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x1c04);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x1d04);
        WriteGMII16( 0x06, 0xe2e0);
        WriteGMII16( 0x06, 0x7ce3);
        WriteGMII16( 0x06, 0xe07d);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x38e1);
        WriteGMII16( 0x06, 0xe039);
        WriteGMII16( 0x06, 0xad2e);
        WriteGMII16( 0x06, 0x1bad);
        WriteGMII16( 0x06, 0x390d);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf21);
        WriteGMII16( 0x06, 0xd502);
        WriteGMII16( 0x06, 0x3723);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xd8ae);
        WriteGMII16( 0x06, 0x0bac);
        WriteGMII16( 0x06, 0x3802);
        WriteGMII16( 0x06, 0xae06);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0x1802);
        WriteGMII16( 0x06, 0x8360);
        WriteGMII16( 0x06, 0x021a);
        WriteGMII16( 0x06, 0xc6fd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e1);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2605);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0xa4f7);
        WriteGMII16( 0x06, 0x28e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad21);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x23a9);
        WriteGMII16( 0x06, 0xf729);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2005);
        WriteGMII16( 0x06, 0x0214);
        WriteGMII16( 0x06, 0xabf7);
        WriteGMII16( 0x06, 0x2ae0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad23);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x12e7);
        WriteGMII16( 0x06, 0xf72b);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x2405);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0xbcf7);
        WriteGMII16( 0x06, 0x2ce5);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x21e5);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2109);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xf4ac);
        WriteGMII16( 0x06, 0x2003);
        WriteGMII16( 0x06, 0x0223);
        WriteGMII16( 0x06, 0x98e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x09e0);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x13fb);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2309);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xf4ac);
        WriteGMII16( 0x06, 0x2203);
        WriteGMII16( 0x06, 0x0212);
        WriteGMII16( 0x06, 0xfae0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x09e0);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xac23);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x83c1);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e1);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2608);
        WriteGMII16( 0x06, 0xe083);
        WriteGMII16( 0x06, 0xd2ad);
        WriteGMII16( 0x06, 0x2502);
        WriteGMII16( 0x06, 0xf628);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x210a);
        WriteGMII16( 0x06, 0xe084);
        WriteGMII16( 0x06, 0x0af6);
        WriteGMII16( 0x06, 0x27a0);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0xf629);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2008);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xe8ad);
        WriteGMII16( 0x06, 0x2102);
        WriteGMII16( 0x06, 0xf62a);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2308);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x20a0);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0xf62b);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x2408);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x02a0);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0xf62c);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xf4a1);
        WriteGMII16( 0x06, 0x0008);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf21);
        WriteGMII16( 0x06, 0xd502);
        WriteGMII16( 0x06, 0x3723);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x0200);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x241e);
        WriteGMII16( 0x06, 0xe086);
        WriteGMII16( 0x06, 0x02a0);
        WriteGMII16( 0x06, 0x0005);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0xe8ae);
        WriteGMII16( 0x06, 0xf5a0);
        WriteGMII16( 0x06, 0x0105);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0xf8ae);
        WriteGMII16( 0x06, 0x0ba0);
        WriteGMII16( 0x06, 0x0205);
        WriteGMII16( 0x06, 0x0284);
        WriteGMII16( 0x06, 0x14ae);
        WriteGMII16( 0x06, 0x03a0);
        WriteGMII16( 0x06, 0x0300);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0x0284);
        WriteGMII16( 0x06, 0x2bee);
        WriteGMII16( 0x06, 0x8602);
        WriteGMII16( 0x06, 0x01ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8ee);
        WriteGMII16( 0x06, 0x8609);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x8461);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xae10);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8608);
        WriteGMII16( 0x06, 0xe186);
        WriteGMII16( 0x06, 0x091f);
        WriteGMII16( 0x06, 0x019e);
        WriteGMII16( 0x06, 0x0611);
        WriteGMII16( 0x06, 0xe586);
        WriteGMII16( 0x06, 0x09ae);
        WriteGMII16( 0x06, 0x04ee);
        WriteGMII16( 0x06, 0x8602);
        WriteGMII16( 0x06, 0x01fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xfbbf);
        WriteGMII16( 0x06, 0x8604);
        WriteGMII16( 0x06, 0xef79);
        WriteGMII16( 0x06, 0xd200);
        WriteGMII16( 0x06, 0xd400);
        WriteGMII16( 0x06, 0x221e);
        WriteGMII16( 0x06, 0x02bf);
        WriteGMII16( 0x06, 0x2fec);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23bf);
        WriteGMII16( 0x06, 0x13f2);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf60d);
        WriteGMII16( 0x06, 0x4559);
        WriteGMII16( 0x06, 0x1fef);
        WriteGMII16( 0x06, 0x97dd);
        WriteGMII16( 0x06, 0xd308);
        WriteGMII16( 0x06, 0x1a93);
        WriteGMII16( 0x06, 0xdd12);
        WriteGMII16( 0x06, 0x17a2);
        WriteGMII16( 0x06, 0x04de);
        WriteGMII16( 0x06, 0xffef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xfbee);
        WriteGMII16( 0x06, 0x8602);
        WriteGMII16( 0x06, 0x03d5);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x06, 0xbf86);
        WriteGMII16( 0x06, 0x04ef);
        WriteGMII16( 0x06, 0x79ef);
        WriteGMII16( 0x06, 0x45bf);
        WriteGMII16( 0x06, 0x2fec);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23bf);
        WriteGMII16( 0x06, 0x13f2);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf6ad);
        WriteGMII16( 0x06, 0x2702);
        WriteGMII16( 0x06, 0x78ff);
        WriteGMII16( 0x06, 0xe186);
        WriteGMII16( 0x06, 0x0a1b);
        WriteGMII16( 0x06, 0x01aa);
        WriteGMII16( 0x06, 0x2eef);
        WriteGMII16( 0x06, 0x97d9);
        WriteGMII16( 0x06, 0x7900);
        WriteGMII16( 0x06, 0x9e2b);
        WriteGMII16( 0x06, 0x81dd);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0xad02);
        WriteGMII16( 0x06, 0x3723);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xef02);
        WriteGMII16( 0x06, 0x100c);
        WriteGMII16( 0x06, 0x11b0);
        WriteGMII16( 0x06, 0xfc0d);
        WriteGMII16( 0x06, 0x11bf);
        WriteGMII16( 0x06, 0x85aa);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x85aa);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x23ee);
        WriteGMII16( 0x06, 0x8602);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x0413);
        WriteGMII16( 0x06, 0xa38b);
        WriteGMII16( 0x06, 0xb4d3);
        WriteGMII16( 0x06, 0x8012);
        WriteGMII16( 0x06, 0x17a2);
        WriteGMII16( 0x06, 0x04ad);
        WriteGMII16( 0x06, 0xffef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x48e0);
        WriteGMII16( 0x06, 0x8a96);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0x977c);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x9e35);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x9600);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x9700);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xbee1);
        WriteGMII16( 0x06, 0x8abf);
        WriteGMII16( 0x06, 0xe286);
        WriteGMII16( 0x06, 0x10e3);
        WriteGMII16( 0x06, 0x8611);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0x1aad);
        WriteGMII16( 0x06, 0x2012);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x9603);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x97b7);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x1000);
        WriteGMII16( 0x06, 0xee86);
        WriteGMII16( 0x06, 0x1100);
        WriteGMII16( 0x06, 0xae11);
        WriteGMII16( 0x06, 0x15e6);
        WriteGMII16( 0x06, 0x8610);
        WriteGMII16( 0x06, 0xe786);
        WriteGMII16( 0x06, 0x11ae);
        WriteGMII16( 0x06, 0x08ee);
        WriteGMII16( 0x06, 0x8610);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8611);
        WriteGMII16( 0x06, 0x00fd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0xe001);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x32e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf720);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40bf);
        WriteGMII16( 0x06, 0x31f5);
        WriteGMII16( 0x06, 0x0236);
        WriteGMII16( 0x06, 0xf6ad);
        WriteGMII16( 0x06, 0x2821);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x20e1);
        WriteGMII16( 0x06, 0xe021);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x18e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40ee);
        WriteGMII16( 0x06, 0x8b3b);
        WriteGMII16( 0x06, 0xffe0);
        WriteGMII16( 0x06, 0x8a8a);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0x8be4);
        WriteGMII16( 0x06, 0xe000);
        WriteGMII16( 0x06, 0xe5e0);
        WriteGMII16( 0x06, 0x01ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x80ad);
        WriteGMII16( 0x06, 0x2722);
        WriteGMII16( 0x06, 0xbf44);
        WriteGMII16( 0x06, 0xfc02);
        WriteGMII16( 0x06, 0x36f6);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x441f);
        WriteGMII16( 0x06, 0x019e);
        WriteGMII16( 0x06, 0x15e5);
        WriteGMII16( 0x06, 0x8b44);
        WriteGMII16( 0x06, 0xad29);
        WriteGMII16( 0x06, 0x07ac);
        WriteGMII16( 0x06, 0x2804);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xae02);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0xb002);
        WriteGMII16( 0x06, 0x3723);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfefc);
        WriteGMII16( 0x06, 0x0400);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0x77e1);
        WriteGMII16( 0x06, 0x40dd);
        WriteGMII16( 0x06, 0xe022);
        WriteGMII16( 0x06, 0x32e1);
        WriteGMII16( 0x06, 0x5074);
        WriteGMII16( 0x06, 0xe144);
        WriteGMII16( 0x06, 0xffe0);
        WriteGMII16( 0x06, 0xdaff);
        WriteGMII16( 0x06, 0xe0c0);
        WriteGMII16( 0x06, 0x52e0);
        WriteGMII16( 0x06, 0xeed9);
        WriteGMII16( 0x06, 0xe04c);
        WriteGMII16( 0x06, 0xbbe0);
        WriteGMII16( 0x06, 0x2a00);
        WriteGMII16( 0x05, 0xe142);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x05, 0xe140);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0042);
        WriteGMII16( 0x18, 0x2300);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        if ((pdev->subsystem_vendor == 0x144d &&
             pdev->subsystem_device == 0xc098) ||
            (pdev->subsystem_vendor == 0x144d &&
             pdev->subsystem_device == 0xc0b1)) {
            gphy_val = ReadGMII16( 0x17);
            gphy_val &= ~BIT_2;
            WriteGMII16( 0x17, gphy_val);
        }
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x9200);
    } else if (mcfg == MCFG_8168F_1) {
        WriteGMII16(0x1f, 0x0000);
        WriteGMII16(0x00, 0x1800);
        gphy_val = ReadGMII16( 0x15);
        gphy_val &= ~(BIT_12);
        WriteGMII16(0x15, gphy_val);
        WriteGMII16(0x00, 0x4800);
        WriteGMII16(0x1f, 0x0007);
        WriteGMII16(0x1e, 0x002f);
        for (i = 0; i < 1000; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x1c);
            if (gphy_val & 0x0080)
                break;
        }
        WriteGMII16(0x1f, 0x0000);
        WriteGMII16(0x00, 0x1800);
        WriteGMII16(0x1f, 0x0007);
        WriteGMII16(0x1e, 0x0023);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x18);
            if (!(gphy_val & 0x0001))
                break;
        }
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0307);
        WriteGMII16( 0x15, 0x0194);
        WriteGMII16( 0x19, 0x407D);
        WriteGMII16( 0x15, 0x0098);
        WriteGMII16( 0x19, 0x7c0b);
        WriteGMII16( 0x15, 0x0099);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00eb);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00f8);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00fe);
        WriteGMII16( 0x19, 0x6f0f);
        WriteGMII16( 0x15, 0x00db);
        WriteGMII16( 0x19, 0x6f09);
        WriteGMII16( 0x15, 0x00dc);
        WriteGMII16( 0x19, 0xaefd);
        WriteGMII16( 0x15, 0x00dd);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00de);
        WriteGMII16( 0x19, 0xc60b);
        WriteGMII16( 0x15, 0x00df);
        WriteGMII16( 0x19, 0x00fa);
        WriteGMII16( 0x15, 0x00e0);
        WriteGMII16( 0x19, 0x30e1);
        WriteGMII16( 0x15, 0x020c);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x020e);
        WriteGMII16( 0x19, 0x9813);
        WriteGMII16( 0x15, 0x020f);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0210);
        WriteGMII16( 0x19, 0x930f);
        WriteGMII16( 0x15, 0x0211);
        WriteGMII16( 0x19, 0x9206);
        WriteGMII16( 0x15, 0x0212);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0213);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0214);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0215);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0216);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0217);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0218);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0219);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x021a);
        WriteGMII16( 0x19, 0x5540);
        WriteGMII16( 0x15, 0x021b);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x021c);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x021d);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x021e);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x021f);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0220);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0221);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x0222);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0223);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x0224);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0225);
        WriteGMII16( 0x19, 0x3231);
        WriteGMII16( 0x15, 0x0000);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x48f7);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xa080);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xf602);
        WriteGMII16( 0x06, 0x0118);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x2502);
        WriteGMII16( 0x06, 0x8090);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x4202);
        WriteGMII16( 0x06, 0x015c);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0xad02);
        WriteGMII16( 0x06, 0x80ca);
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
        WriteGMII16( 0x06, 0xaebb);
        WriteGMII16( 0x06, 0xd484);
        WriteGMII16( 0x06, 0x3ce4);
        WriteGMII16( 0x06, 0x8b92);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x93ee);
        WriteGMII16( 0x06, 0x8ac8);
        WriteGMII16( 0x06, 0x03ee);
        WriteGMII16( 0x06, 0x8aca);
        WriteGMII16( 0x06, 0x60ee);
        WriteGMII16( 0x06, 0x8ac0);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8ac1);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8abe);
        WriteGMII16( 0x06, 0x07ee);
        WriteGMII16( 0x06, 0x8abf);
        WriteGMII16( 0x06, 0x73ee);
        WriteGMII16( 0x06, 0x8a95);
        WriteGMII16( 0x06, 0x02bf);
        WriteGMII16( 0x06, 0x8b88);
        WriteGMII16( 0x06, 0xec00);
        WriteGMII16( 0x06, 0x19a9);
        WriteGMII16( 0x06, 0x8b90);
        WriteGMII16( 0x06, 0xf9ee);
        WriteGMII16( 0x06, 0xfff6);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xfed1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x85a4);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7dd1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x85a7);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7d04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8a);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x14ee);
        WriteGMII16( 0x06, 0x8b8a);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x204b);
        WriteGMII16( 0x06, 0xe0e4);
        WriteGMII16( 0x06, 0x26e1);
        WriteGMII16( 0x06, 0xe427);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x2623);
        WriteGMII16( 0x06, 0xe5e4);
        WriteGMII16( 0x06, 0x27fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8dad);
        WriteGMII16( 0x06, 0x2014);
        WriteGMII16( 0x06, 0xee8b);
        WriteGMII16( 0x06, 0x8d00);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0x5a78);
        WriteGMII16( 0x06, 0x039e);
        WriteGMII16( 0x06, 0x0902);
        WriteGMII16( 0x06, 0x05e8);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x4f02);
        WriteGMII16( 0x06, 0x326c);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x1df6);
        WriteGMII16( 0x06, 0x20e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x022f);
        WriteGMII16( 0x06, 0x0902);
        WriteGMII16( 0x06, 0x2ab0);
        WriteGMII16( 0x06, 0x0285);
        WriteGMII16( 0x06, 0x1602);
        WriteGMII16( 0x06, 0x03ba);
        WriteGMII16( 0x06, 0x0284);
        WriteGMII16( 0x06, 0xe502);
        WriteGMII16( 0x06, 0x2df1);
        WriteGMII16( 0x06, 0x0283);
        WriteGMII16( 0x06, 0x8302);
        WriteGMII16( 0x06, 0x0475);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x210b);
        WriteGMII16( 0x06, 0xf621);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x83f8);
        WriteGMII16( 0x06, 0x021c);
        WriteGMII16( 0x06, 0x99e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x22e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0235);
        WriteGMII16( 0x06, 0x63e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad23);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x23e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0231);
        WriteGMII16( 0x06, 0x57e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x24e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2505);
        WriteGMII16( 0x06, 0xf625);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8ee0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x26e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x1ce0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x27e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0x80fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac26);
        WriteGMII16( 0x06, 0x1ae0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x14e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xac20);
        WriteGMII16( 0x06, 0x0ee0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xac23);
        WriteGMII16( 0x06, 0x08e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xac24);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x3802);
        WriteGMII16( 0x06, 0x1ac2);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x1c04);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x1d04);
        WriteGMII16( 0x06, 0xe2e0);
        WriteGMII16( 0x06, 0x7ce3);
        WriteGMII16( 0x06, 0xe07d);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x38e1);
        WriteGMII16( 0x06, 0xe039);
        WriteGMII16( 0x06, 0xad2e);
        WriteGMII16( 0x06, 0x1bad);
        WriteGMII16( 0x06, 0x390d);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf22);
        WriteGMII16( 0x06, 0x7a02);
        WriteGMII16( 0x06, 0x387d);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0xacae);
        WriteGMII16( 0x06, 0x0bac);
        WriteGMII16( 0x06, 0x3802);
        WriteGMII16( 0x06, 0xae06);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0xe902);
        WriteGMII16( 0x06, 0x822e);
        WriteGMII16( 0x06, 0x021a);
        WriteGMII16( 0x06, 0xd3fd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e1);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2602);
        WriteGMII16( 0x06, 0xf728);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2105);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0x8ef7);
        WriteGMII16( 0x06, 0x29e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x14b8);
        WriteGMII16( 0x06, 0xf72a);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2305);
        WriteGMII16( 0x06, 0x0212);
        WriteGMII16( 0x06, 0xf4f7);
        WriteGMII16( 0x06, 0x2be0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0x8284);
        WriteGMII16( 0x06, 0xf72c);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xf4fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2600);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2109);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xf4ac);
        WriteGMII16( 0x06, 0x2003);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0x7de0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x09e0);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x1408);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2309);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xf4ac);
        WriteGMII16( 0x06, 0x2203);
        WriteGMII16( 0x06, 0x0213);
        WriteGMII16( 0x06, 0x07e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x09e0);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xac23);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0x8289);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e1);
        WriteGMII16( 0x06, 0x8af4);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x2602);
        WriteGMII16( 0x06, 0xf628);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ad);
        WriteGMII16( 0x06, 0x210a);
        WriteGMII16( 0x06, 0xe083);
        WriteGMII16( 0x06, 0xecf6);
        WriteGMII16( 0x06, 0x27a0);
        WriteGMII16( 0x06, 0x0502);
        WriteGMII16( 0x06, 0xf629);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2008);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xe8ad);
        WriteGMII16( 0x06, 0x2102);
        WriteGMII16( 0x06, 0xf62a);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ad);
        WriteGMII16( 0x06, 0x2308);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x20a0);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0xf62b);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x2408);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xc2a0);
        WriteGMII16( 0x06, 0x0302);
        WriteGMII16( 0x06, 0xf62c);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xf4a1);
        WriteGMII16( 0x06, 0x0008);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf22);
        WriteGMII16( 0x06, 0x7a02);
        WriteGMII16( 0x06, 0x387d);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc200);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ad);
        WriteGMII16( 0x06, 0x241e);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xc2a0);
        WriteGMII16( 0x06, 0x0005);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xb0ae);
        WriteGMII16( 0x06, 0xf5a0);
        WriteGMII16( 0x06, 0x0105);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xc0ae);
        WriteGMII16( 0x06, 0x0ba0);
        WriteGMII16( 0x06, 0x0205);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xcaae);
        WriteGMII16( 0x06, 0x03a0);
        WriteGMII16( 0x06, 0x0300);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xe1ee);
        WriteGMII16( 0x06, 0x8ac2);
        WriteGMII16( 0x06, 0x01ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8ee);
        WriteGMII16( 0x06, 0x8ac9);
        WriteGMII16( 0x06, 0x0002);
        WriteGMII16( 0x06, 0x8317);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8ac8);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xc91f);
        WriteGMII16( 0x06, 0x019e);
        WriteGMII16( 0x06, 0x0611);
        WriteGMII16( 0x06, 0xe58a);
        WriteGMII16( 0x06, 0xc9ae);
        WriteGMII16( 0x06, 0x04ee);
        WriteGMII16( 0x06, 0x8ac2);
        WriteGMII16( 0x06, 0x01fc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xfbbf);
        WriteGMII16( 0x06, 0x8ac4);
        WriteGMII16( 0x06, 0xef79);
        WriteGMII16( 0x06, 0xd200);
        WriteGMII16( 0x06, 0xd400);
        WriteGMII16( 0x06, 0x221e);
        WriteGMII16( 0x06, 0x02bf);
        WriteGMII16( 0x06, 0x3024);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7dbf);
        WriteGMII16( 0x06, 0x13ff);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x500d);
        WriteGMII16( 0x06, 0x4559);
        WriteGMII16( 0x06, 0x1fef);
        WriteGMII16( 0x06, 0x97dd);
        WriteGMII16( 0x06, 0xd308);
        WriteGMII16( 0x06, 0x1a93);
        WriteGMII16( 0x06, 0xdd12);
        WriteGMII16( 0x06, 0x17a2);
        WriteGMII16( 0x06, 0x04de);
        WriteGMII16( 0x06, 0xffef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xfbee);
        WriteGMII16( 0x06, 0x8ac2);
        WriteGMII16( 0x06, 0x03d5);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x06, 0xbf8a);
        WriteGMII16( 0x06, 0xc4ef);
        WriteGMII16( 0x06, 0x79ef);
        WriteGMII16( 0x06, 0x45bf);
        WriteGMII16( 0x06, 0x3024);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7dbf);
        WriteGMII16( 0x06, 0x13ff);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x50ad);
        WriteGMII16( 0x06, 0x2702);
        WriteGMII16( 0x06, 0x78ff);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0xca1b);
        WriteGMII16( 0x06, 0x01aa);
        WriteGMII16( 0x06, 0x2eef);
        WriteGMII16( 0x06, 0x97d9);
        WriteGMII16( 0x06, 0x7900);
        WriteGMII16( 0x06, 0x9e2b);
        WriteGMII16( 0x06, 0x81dd);
        WriteGMII16( 0x06, 0xbf85);
        WriteGMII16( 0x06, 0xad02);
        WriteGMII16( 0x06, 0x387d);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xef02);
        WriteGMII16( 0x06, 0x100c);
        WriteGMII16( 0x06, 0x11b0);
        WriteGMII16( 0x06, 0xfc0d);
        WriteGMII16( 0x06, 0x11bf);
        WriteGMII16( 0x06, 0x85aa);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7dd1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x85aa);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7dee);
        WriteGMII16( 0x06, 0x8ac2);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x0413);
        WriteGMII16( 0x06, 0xa38b);
        WriteGMII16( 0x06, 0xb4d3);
        WriteGMII16( 0x06, 0x8012);
        WriteGMII16( 0x06, 0x17a2);
        WriteGMII16( 0x06, 0x04ad);
        WriteGMII16( 0x06, 0xffef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x48e0);
        WriteGMII16( 0x06, 0x8a96);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0x977c);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x9e35);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x9600);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x9700);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0xbee1);
        WriteGMII16( 0x06, 0x8abf);
        WriteGMII16( 0x06, 0xe28a);
        WriteGMII16( 0x06, 0xc0e3);
        WriteGMII16( 0x06, 0x8ac1);
        WriteGMII16( 0x06, 0x0237);
        WriteGMII16( 0x06, 0x74ad);
        WriteGMII16( 0x06, 0x2012);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x9603);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0x97b7);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc000);
        WriteGMII16( 0x06, 0xee8a);
        WriteGMII16( 0x06, 0xc100);
        WriteGMII16( 0x06, 0xae11);
        WriteGMII16( 0x06, 0x15e6);
        WriteGMII16( 0x06, 0x8ac0);
        WriteGMII16( 0x06, 0xe78a);
        WriteGMII16( 0x06, 0xc1ae);
        WriteGMII16( 0x06, 0x08ee);
        WriteGMII16( 0x06, 0x8ac0);
        WriteGMII16( 0x06, 0x00ee);
        WriteGMII16( 0x06, 0x8ac1);
        WriteGMII16( 0x06, 0x00fd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xae20);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0xe001);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x32e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf720);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40bf);
        WriteGMII16( 0x06, 0x3230);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x50ad);
        WriteGMII16( 0x06, 0x2821);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x20e1);
        WriteGMII16( 0x06, 0xe021);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x18e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40ee);
        WriteGMII16( 0x06, 0x8b3b);
        WriteGMII16( 0x06, 0xffe0);
        WriteGMII16( 0x06, 0x8a8a);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0x8be4);
        WriteGMII16( 0x06, 0xe000);
        WriteGMII16( 0x06, 0xe5e0);
        WriteGMII16( 0x06, 0x01ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xface);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69fa);
        WriteGMII16( 0x06, 0xd401);
        WriteGMII16( 0x06, 0x55b4);
        WriteGMII16( 0x06, 0xfebf);
        WriteGMII16( 0x06, 0x1c1e);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x50ac);
        WriteGMII16( 0x06, 0x280b);
        WriteGMII16( 0x06, 0xbf1c);
        WriteGMII16( 0x06, 0x1b02);
        WriteGMII16( 0x06, 0x3850);
        WriteGMII16( 0x06, 0xac28);
        WriteGMII16( 0x06, 0x49ae);
        WriteGMII16( 0x06, 0x64bf);
        WriteGMII16( 0x06, 0x1c1b);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x50ac);
        WriteGMII16( 0x06, 0x285b);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x0284);
        WriteGMII16( 0x06, 0xcaac);
        WriteGMII16( 0x06, 0x2105);
        WriteGMII16( 0x06, 0xac22);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x4ebf);
        WriteGMII16( 0x06, 0xe0c4);
        WriteGMII16( 0x06, 0xbe85);
        WriteGMII16( 0x06, 0xf6d2);
        WriteGMII16( 0x06, 0x04d8);
        WriteGMII16( 0x06, 0x19d9);
        WriteGMII16( 0x06, 0x1907);
        WriteGMII16( 0x06, 0xdc19);
        WriteGMII16( 0x06, 0xdd19);
        WriteGMII16( 0x06, 0x0789);
        WriteGMII16( 0x06, 0x89ef);
        WriteGMII16( 0x06, 0x645e);
        WriteGMII16( 0x06, 0x07ff);
        WriteGMII16( 0x06, 0x0d65);
        WriteGMII16( 0x06, 0x5cf8);
        WriteGMII16( 0x06, 0x001e);
        WriteGMII16( 0x06, 0x46dc);
        WriteGMII16( 0x06, 0x19dd);
        WriteGMII16( 0x06, 0x19b2);
        WriteGMII16( 0x06, 0xe2d4);
        WriteGMII16( 0x06, 0x0001);
        WriteGMII16( 0x06, 0xbf1c);
        WriteGMII16( 0x06, 0x1b02);
        WriteGMII16( 0x06, 0x387d);
        WriteGMII16( 0x06, 0xae1d);
        WriteGMII16( 0x06, 0xbee0);
        WriteGMII16( 0x06, 0xc4bf);
        WriteGMII16( 0x06, 0x85f6);
        WriteGMII16( 0x06, 0xd204);
        WriteGMII16( 0x06, 0xd819);
        WriteGMII16( 0x06, 0xd919);
        WriteGMII16( 0x06, 0x07dc);
        WriteGMII16( 0x06, 0x19dd);
        WriteGMII16( 0x06, 0x1907);
        WriteGMII16( 0x06, 0xb2f4);
        WriteGMII16( 0x06, 0xd400);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x1c1b);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7dfe);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfec6);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc05);
        WriteGMII16( 0x06, 0xf9e2);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0xeb5a);
        WriteGMII16( 0x06, 0x070c);
        WriteGMII16( 0x06, 0x031e);
        WriteGMII16( 0x06, 0x20e6);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe7e0);
        WriteGMII16( 0x06, 0xebe0);
        WriteGMII16( 0x06, 0xe0fc);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0xfdfd);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0x8b80);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x22bf);
        WriteGMII16( 0x06, 0x4616);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x50e0);
        WriteGMII16( 0x06, 0x8b44);
        WriteGMII16( 0x06, 0x1f01);
        WriteGMII16( 0x06, 0x9e15);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x44ad);
        WriteGMII16( 0x06, 0x2907);
        WriteGMII16( 0x06, 0xac28);
        WriteGMII16( 0x06, 0x04d1);
        WriteGMII16( 0x06, 0x01ae);
        WriteGMII16( 0x06, 0x02d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x85b0);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7def);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x30e0);
        WriteGMII16( 0x06, 0xe036);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x37e1);
        WriteGMII16( 0x06, 0x8b3f);
        WriteGMII16( 0x06, 0x1f10);
        WriteGMII16( 0x06, 0x9e23);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x3fac);
        WriteGMII16( 0x06, 0x200b);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x0dac);
        WriteGMII16( 0x06, 0x250f);
        WriteGMII16( 0x06, 0xac27);
        WriteGMII16( 0x06, 0x11ae);
        WriteGMII16( 0x06, 0x1202);
        WriteGMII16( 0x06, 0x2c47);
        WriteGMII16( 0x06, 0xae0d);
        WriteGMII16( 0x06, 0x0285);
        WriteGMII16( 0x06, 0x4fae);
        WriteGMII16( 0x06, 0x0802);
        WriteGMII16( 0x06, 0x2c69);
        WriteGMII16( 0x06, 0xae03);
        WriteGMII16( 0x06, 0x022c);
        WriteGMII16( 0x06, 0x7cfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x6902);
        WriteGMII16( 0x06, 0x856c);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x14e1);
        WriteGMII16( 0x06, 0xe015);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x08d1);
        WriteGMII16( 0x06, 0x1ebf);
        WriteGMII16( 0x06, 0x2cd9);
        WriteGMII16( 0x06, 0x0238);
        WriteGMII16( 0x06, 0x7def);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x2fd0);
        WriteGMII16( 0x06, 0x0b02);
        WriteGMII16( 0x06, 0x3682);
        WriteGMII16( 0x06, 0x5882);
        WriteGMII16( 0x06, 0x7882);
        WriteGMII16( 0x06, 0x9f24);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x32e1);
        WriteGMII16( 0x06, 0x8b33);
        WriteGMII16( 0x06, 0x1f10);
        WriteGMII16( 0x06, 0x9e1a);
        WriteGMII16( 0x06, 0x10e4);
        WriteGMII16( 0x06, 0x8b32);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x28e1);
        WriteGMII16( 0x06, 0xe029);
        WriteGMII16( 0x06, 0xf72c);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0x28e5);
        WriteGMII16( 0x06, 0xe029);
        WriteGMII16( 0x06, 0xf62c);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0x28e5);
        WriteGMII16( 0x06, 0xe029);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0x4077);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0x52e0);
        WriteGMII16( 0x06, 0xeed9);
        WriteGMII16( 0x06, 0xe04c);
        WriteGMII16( 0x06, 0xbbe0);
        WriteGMII16( 0x06, 0x2a00);
        WriteGMII16( 0x05, 0xe142);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16(0x06, gphy_val);
        WriteGMII16( 0x05, 0xe140);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16(0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16(0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        gphy_val |= BIT_1;
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x09, 0xA20F);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x1f, 0x0003);
        WriteGMII16( 0x01, 0x328A);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x9200);
    } else if (mcfg == MCFG_8168F_2) {
        WriteGMII16(0x1f, 0x0000);
        WriteGMII16(0x00, 0x1800);
        gphy_val = ReadGMII16( 0x15);
        gphy_val &= ~(BIT_12);
        WriteGMII16(0x15, gphy_val);
        WriteGMII16(0x00, 0x9800);
        WriteGMII16(0x1f, 0x0007);
        WriteGMII16(0x1e, 0x002f);
        for (i = 0; i < 1000; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x1c);
            if (gphy_val & 0x0080)
                break;
        }
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0307);
        WriteGMII16( 0x15, 0x0098);
        WriteGMII16( 0x19, 0x7c0b);
        WriteGMII16( 0x15, 0x0099);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00eb);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00f8);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00fe);
        WriteGMII16( 0x19, 0x6f0f);
        WriteGMII16( 0x15, 0x00db);
        WriteGMII16( 0x19, 0x6f09);
        WriteGMII16( 0x15, 0x00dc);
        WriteGMII16( 0x19, 0xaefd);
        WriteGMII16( 0x15, 0x00dd);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00de);
        WriteGMII16( 0x19, 0xc60b);
        WriteGMII16( 0x15, 0x00df);
        WriteGMII16( 0x19, 0x00fa);
        WriteGMII16( 0x15, 0x00e0);
        WriteGMII16( 0x19, 0x30e1);
        WriteGMII16( 0x15, 0x020c);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x020e);
        WriteGMII16( 0x19, 0x9813);
        WriteGMII16( 0x15, 0x020f);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0210);
        WriteGMII16( 0x19, 0x930f);
        WriteGMII16( 0x15, 0x0211);
        WriteGMII16( 0x19, 0x9206);
        WriteGMII16( 0x15, 0x0212);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0213);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0214);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0215);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0216);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0217);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0218);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0219);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x021a);
        WriteGMII16( 0x19, 0x5540);
        WriteGMII16( 0x15, 0x021b);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x021c);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x021d);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x021e);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x021f);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0220);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0221);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x0222);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0223);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x0224);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0225);
        WriteGMII16( 0x19, 0x3231);
        WriteGMII16( 0x15, 0x0000);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x48f7);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xa080);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xf602);
        WriteGMII16( 0x06, 0x011b);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x2802);
        WriteGMII16( 0x06, 0x0135);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x4502);
        WriteGMII16( 0x06, 0x015f);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x6b02);
        WriteGMII16( 0x06, 0x80e5);
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
        WriteGMII16( 0x06, 0xaebb);
        WriteGMII16( 0x06, 0xbf8b);
        WriteGMII16( 0x06, 0x88ec);
        WriteGMII16( 0x06, 0x0019);
        WriteGMII16( 0x06, 0xa98b);
        WriteGMII16( 0x06, 0x90f9);
        WriteGMII16( 0x06, 0xeeff);
        WriteGMII16( 0x06, 0xf600);
        WriteGMII16( 0x06, 0xeeff);
        WriteGMII16( 0x06, 0xf7fe);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf81);
        WriteGMII16( 0x06, 0x9802);
        WriteGMII16( 0x06, 0x39f3);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf81);
        WriteGMII16( 0x06, 0x9b02);
        WriteGMII16( 0x06, 0x39f3);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8dad);
        WriteGMII16( 0x06, 0x2014);
        WriteGMII16( 0x06, 0xee8b);
        WriteGMII16( 0x06, 0x8d00);
        WriteGMII16( 0x06, 0xe08a);
        WriteGMII16( 0x06, 0x5a78);
        WriteGMII16( 0x06, 0x039e);
        WriteGMII16( 0x06, 0x0902);
        WriteGMII16( 0x06, 0x05fc);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x8802);
        WriteGMII16( 0x06, 0x32dd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ac);
        WriteGMII16( 0x06, 0x261a);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x81ac);
        WriteGMII16( 0x06, 0x2114);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ac);
        WriteGMII16( 0x06, 0x200e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x85ac);
        WriteGMII16( 0x06, 0x2308);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x87ac);
        WriteGMII16( 0x06, 0x2402);
        WriteGMII16( 0x06, 0xae38);
        WriteGMII16( 0x06, 0x021a);
        WriteGMII16( 0x06, 0xd6ee);
        WriteGMII16( 0x06, 0xe41c);
        WriteGMII16( 0x06, 0x04ee);
        WriteGMII16( 0x06, 0xe41d);
        WriteGMII16( 0x06, 0x04e2);
        WriteGMII16( 0x06, 0xe07c);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0x7de0);
        WriteGMII16( 0x06, 0xe038);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x39ad);
        WriteGMII16( 0x06, 0x2e1b);
        WriteGMII16( 0x06, 0xad39);
        WriteGMII16( 0x06, 0x0dd1);
        WriteGMII16( 0x06, 0x01bf);
        WriteGMII16( 0x06, 0x22c8);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xf302);
        WriteGMII16( 0x06, 0x21f0);
        WriteGMII16( 0x06, 0xae0b);
        WriteGMII16( 0x06, 0xac38);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x0602);
        WriteGMII16( 0x06, 0x222d);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0x7202);
        WriteGMII16( 0x06, 0x1ae7);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x201a);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x2afe);
        WriteGMII16( 0x06, 0x022c);
        WriteGMII16( 0x06, 0x5c02);
        WriteGMII16( 0x06, 0x03c5);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x6702);
        WriteGMII16( 0x06, 0x2e4f);
        WriteGMII16( 0x06, 0x0204);
        WriteGMII16( 0x06, 0x8902);
        WriteGMII16( 0x06, 0x2f7a);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x210b);
        WriteGMII16( 0x06, 0xf621);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x0445);
        WriteGMII16( 0x06, 0x021c);
        WriteGMII16( 0x06, 0xb8e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad22);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x22e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0235);
        WriteGMII16( 0x06, 0xd4e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad23);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x23e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0231);
        WriteGMII16( 0x06, 0xc8e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad24);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x24e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2505);
        WriteGMII16( 0x06, 0xf625);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8ee0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x08f6);
        WriteGMII16( 0x06, 0x26e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x022d);
        WriteGMII16( 0x06, 0x6ae0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x27e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0x8bfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0x8b80);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x22bf);
        WriteGMII16( 0x06, 0x479a);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xc6e0);
        WriteGMII16( 0x06, 0x8b44);
        WriteGMII16( 0x06, 0x1f01);
        WriteGMII16( 0x06, 0x9e15);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x44ad);
        WriteGMII16( 0x06, 0x2907);
        WriteGMII16( 0x06, 0xac28);
        WriteGMII16( 0x06, 0x04d1);
        WriteGMII16( 0x06, 0x01ae);
        WriteGMII16( 0x06, 0x02d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x819e);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xf3ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0x4077);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0xbbe0);
        WriteGMII16( 0x06, 0x2a00);
        WriteGMII16( 0x05, 0xe142);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x05, 0xe140);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        gphy_val |= BIT_1;
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x00, 0x9200);
    } else if (mcfg == MCFG_8411_1) {
        WriteGMII16(0x1f, 0x0000);
        WriteGMII16(0x00, 0x1800);
        gphy_val = ReadGMII16( 0x15);
        gphy_val &= ~(BIT_12);
        WriteGMII16(0x15, gphy_val);
        WriteGMII16(0x00, 0x4800);
        WriteGMII16(0x1f, 0x0007);
        WriteGMII16(0x1e, 0x002f);
        for (i = 0; i < 1000; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x1c);
            if (gphy_val & 0x0080)
                break;
        }
        WriteGMII16(0x1f, 0x0000);
        WriteGMII16(0x00, 0x1800);
        WriteGMII16(0x1f, 0x0007);
        WriteGMII16(0x1e, 0x0023);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x18);
            if (!(gphy_val & 0x0001))
                break;
        }
        WriteGMII16(0x1f, 0x0005);
        WriteGMII16(0x05, 0xfff6);
        WriteGMII16(0x06, 0x0080);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x0023);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0307);
        WriteGMII16( 0x15, 0x0098);
        WriteGMII16( 0x19, 0x7c0b);
        WriteGMII16( 0x15, 0x0099);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00eb);
        WriteGMII16( 0x19, 0x6c0b);
        WriteGMII16( 0x15, 0x00f8);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00fe);
        WriteGMII16( 0x19, 0x6f0f);
        WriteGMII16( 0x15, 0x00db);
        WriteGMII16( 0x19, 0x6f09);
        WriteGMII16( 0x15, 0x00dc);
        WriteGMII16( 0x19, 0xaefd);
        WriteGMII16( 0x15, 0x00dd);
        WriteGMII16( 0x19, 0x6f0b);
        WriteGMII16( 0x15, 0x00de);
        WriteGMII16( 0x19, 0xc60b);
        WriteGMII16( 0x15, 0x00df);
        WriteGMII16( 0x19, 0x00fa);
        WriteGMII16( 0x15, 0x00e0);
        WriteGMII16( 0x19, 0x30e1);
        WriteGMII16( 0x15, 0x020c);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x020e);
        WriteGMII16( 0x19, 0x9813);
        WriteGMII16( 0x15, 0x020f);
        WriteGMII16( 0x19, 0x7801);
        WriteGMII16( 0x15, 0x0210);
        WriteGMII16( 0x19, 0x930f);
        WriteGMII16( 0x15, 0x0211);
        WriteGMII16( 0x19, 0x9206);
        WriteGMII16( 0x15, 0x0212);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0213);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0214);
        WriteGMII16( 0x19, 0x588f);
        WriteGMII16( 0x15, 0x0215);
        WriteGMII16( 0x19, 0x5520);
        WriteGMII16( 0x15, 0x0216);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0217);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0218);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0219);
        WriteGMII16( 0x19, 0x588d);
        WriteGMII16( 0x15, 0x021a);
        WriteGMII16( 0x19, 0x5540);
        WriteGMII16( 0x15, 0x021b);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x021c);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x021d);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x021e);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x021f);
        WriteGMII16( 0x19, 0x4002);
        WriteGMII16( 0x15, 0x0220);
        WriteGMII16( 0x19, 0x3224);
        WriteGMII16( 0x15, 0x0221);
        WriteGMII16( 0x19, 0x9e03);
        WriteGMII16( 0x15, 0x0222);
        WriteGMII16( 0x19, 0x7c40);
        WriteGMII16( 0x15, 0x0223);
        WriteGMII16( 0x19, 0x6840);
        WriteGMII16( 0x15, 0x0224);
        WriteGMII16( 0x19, 0x7800);
        WriteGMII16( 0x15, 0x0225);
        WriteGMII16( 0x19, 0x3231);
        WriteGMII16( 0x15, 0x0000);
        WriteGMII16( 0x16, 0x0306);
        WriteGMII16( 0x16, 0x0300);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0xfff6);
        WriteGMII16( 0x06, 0x0080);
        WriteGMII16( 0x05, 0x8000);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x48f7);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0xfff7);
        WriteGMII16( 0x06, 0xa080);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0xf602);
        WriteGMII16( 0x06, 0x011e);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x2b02);
        WriteGMII16( 0x06, 0x8077);
        WriteGMII16( 0x06, 0x0201);
        WriteGMII16( 0x06, 0x4802);
        WriteGMII16( 0x06, 0x0162);
        WriteGMII16( 0x06, 0x0280);
        WriteGMII16( 0x06, 0x9402);
        WriteGMII16( 0x06, 0x810e);
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
        WriteGMII16( 0x06, 0xaebb);
        WriteGMII16( 0x06, 0xd481);
        WriteGMII16( 0x06, 0xd4e4);
        WriteGMII16( 0x06, 0x8b92);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x9302);
        WriteGMII16( 0x06, 0x2e5a);
        WriteGMII16( 0x06, 0xbf8b);
        WriteGMII16( 0x06, 0x88ec);
        WriteGMII16( 0x06, 0x0019);
        WriteGMII16( 0x06, 0xa98b);
        WriteGMII16( 0x06, 0x90f9);
        WriteGMII16( 0x06, 0xeeff);
        WriteGMII16( 0x06, 0xf600);
        WriteGMII16( 0x06, 0xeeff);
        WriteGMII16( 0x06, 0xf7fc);
        WriteGMII16( 0x06, 0xd100);
        WriteGMII16( 0x06, 0xbf83);
        WriteGMII16( 0x06, 0x3c02);
        WriteGMII16( 0x06, 0x3a21);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf83);
        WriteGMII16( 0x06, 0x3f02);
        WriteGMII16( 0x06, 0x3a21);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8aad);
        WriteGMII16( 0x06, 0x2014);
        WriteGMII16( 0x06, 0xee8b);
        WriteGMII16( 0x06, 0x8a00);
        WriteGMII16( 0x06, 0x0220);
        WriteGMII16( 0x06, 0x8be0);
        WriteGMII16( 0x06, 0xe426);
        WriteGMII16( 0x06, 0xe1e4);
        WriteGMII16( 0x06, 0x27ee);
        WriteGMII16( 0x06, 0xe426);
        WriteGMII16( 0x06, 0x23e5);
        WriteGMII16( 0x06, 0xe427);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x14ee);
        WriteGMII16( 0x06, 0x8b8d);
        WriteGMII16( 0x06, 0x00e0);
        WriteGMII16( 0x06, 0x8a5a);
        WriteGMII16( 0x06, 0x7803);
        WriteGMII16( 0x06, 0x9e09);
        WriteGMII16( 0x06, 0x0206);
        WriteGMII16( 0x06, 0x2802);
        WriteGMII16( 0x06, 0x80b1);
        WriteGMII16( 0x06, 0x0232);
        WriteGMII16( 0x06, 0xfdfc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xf9e0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac26);
        WriteGMII16( 0x06, 0x1ae0);
        WriteGMII16( 0x06, 0x8b81);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x14e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xac20);
        WriteGMII16( 0x06, 0x0ee0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xac23);
        WriteGMII16( 0x06, 0x08e0);
        WriteGMII16( 0x06, 0x8b87);
        WriteGMII16( 0x06, 0xac24);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x3802);
        WriteGMII16( 0x06, 0x1b02);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x1c04);
        WriteGMII16( 0x06, 0xeee4);
        WriteGMII16( 0x06, 0x1d04);
        WriteGMII16( 0x06, 0xe2e0);
        WriteGMII16( 0x06, 0x7ce3);
        WriteGMII16( 0x06, 0xe07d);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x38e1);
        WriteGMII16( 0x06, 0xe039);
        WriteGMII16( 0x06, 0xad2e);
        WriteGMII16( 0x06, 0x1bad);
        WriteGMII16( 0x06, 0x390d);
        WriteGMII16( 0x06, 0xd101);
        WriteGMII16( 0x06, 0xbf22);
        WriteGMII16( 0x06, 0xe802);
        WriteGMII16( 0x06, 0x3a21);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0x10ae);
        WriteGMII16( 0x06, 0x0bac);
        WriteGMII16( 0x06, 0x3802);
        WriteGMII16( 0x06, 0xae06);
        WriteGMII16( 0x06, 0x0222);
        WriteGMII16( 0x06, 0x4d02);
        WriteGMII16( 0x06, 0x2292);
        WriteGMII16( 0x06, 0x021b);
        WriteGMII16( 0x06, 0x13fd);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x1af6);
        WriteGMII16( 0x06, 0x20e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x022b);
        WriteGMII16( 0x06, 0x1e02);
        WriteGMII16( 0x06, 0x82ae);
        WriteGMII16( 0x06, 0x0203);
        WriteGMII16( 0x06, 0xc002);
        WriteGMII16( 0x06, 0x827d);
        WriteGMII16( 0x06, 0x022e);
        WriteGMII16( 0x06, 0x6f02);
        WriteGMII16( 0x06, 0x047b);
        WriteGMII16( 0x06, 0x022f);
        WriteGMII16( 0x06, 0x9ae0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad21);
        WriteGMII16( 0x06, 0x0bf6);
        WriteGMII16( 0x06, 0x21e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0x0281);
        WriteGMII16( 0x06, 0x9002);
        WriteGMII16( 0x06, 0x1cd9);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2208);
        WriteGMII16( 0x06, 0xf622);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x35f4);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2308);
        WriteGMII16( 0x06, 0xf623);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x31e8);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2405);
        WriteGMII16( 0x06, 0xf624);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8ee0);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xad25);
        WriteGMII16( 0x06, 0x05f6);
        WriteGMII16( 0x06, 0x25e4);
        WriteGMII16( 0x06, 0x8b8e);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2608);
        WriteGMII16( 0x06, 0xf626);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x2d8a);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x8ead);
        WriteGMII16( 0x06, 0x2705);
        WriteGMII16( 0x06, 0xf627);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x8e02);
        WriteGMII16( 0x06, 0x0386);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8fa);
        WriteGMII16( 0x06, 0xef69);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0xe001);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x32e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf720);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40bf);
        WriteGMII16( 0x06, 0x32c1);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xf4ad);
        WriteGMII16( 0x06, 0x2821);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x20e1);
        WriteGMII16( 0x06, 0xe021);
        WriteGMII16( 0x06, 0xad20);
        WriteGMII16( 0x06, 0x18e0);
        WriteGMII16( 0x06, 0x8b40);
        WriteGMII16( 0x06, 0xf620);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x40ee);
        WriteGMII16( 0x06, 0x8b3b);
        WriteGMII16( 0x06, 0xffe0);
        WriteGMII16( 0x06, 0x8a8a);
        WriteGMII16( 0x06, 0xe18a);
        WriteGMII16( 0x06, 0x8be4);
        WriteGMII16( 0x06, 0xe000);
        WriteGMII16( 0x06, 0xe5e0);
        WriteGMII16( 0x06, 0x01ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8f9);
        WriteGMII16( 0x06, 0xface);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69fa);
        WriteGMII16( 0x06, 0xd401);
        WriteGMII16( 0x06, 0x55b4);
        WriteGMII16( 0x06, 0xfebf);
        WriteGMII16( 0x06, 0x1c5e);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xf4ac);
        WriteGMII16( 0x06, 0x280b);
        WriteGMII16( 0x06, 0xbf1c);
        WriteGMII16( 0x06, 0x5b02);
        WriteGMII16( 0x06, 0x39f4);
        WriteGMII16( 0x06, 0xac28);
        WriteGMII16( 0x06, 0x49ae);
        WriteGMII16( 0x06, 0x64bf);
        WriteGMII16( 0x06, 0x1c5b);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xf4ac);
        WriteGMII16( 0x06, 0x285b);
        WriteGMII16( 0x06, 0xd000);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0x62ac);
        WriteGMII16( 0x06, 0x2105);
        WriteGMII16( 0x06, 0xac22);
        WriteGMII16( 0x06, 0x02ae);
        WriteGMII16( 0x06, 0x4ebf);
        WriteGMII16( 0x06, 0xe0c4);
        WriteGMII16( 0x06, 0xbe85);
        WriteGMII16( 0x06, 0xecd2);
        WriteGMII16( 0x06, 0x04d8);
        WriteGMII16( 0x06, 0x19d9);
        WriteGMII16( 0x06, 0x1907);
        WriteGMII16( 0x06, 0xdc19);
        WriteGMII16( 0x06, 0xdd19);
        WriteGMII16( 0x06, 0x0789);
        WriteGMII16( 0x06, 0x89ef);
        WriteGMII16( 0x06, 0x645e);
        WriteGMII16( 0x06, 0x07ff);
        WriteGMII16( 0x06, 0x0d65);
        WriteGMII16( 0x06, 0x5cf8);
        WriteGMII16( 0x06, 0x001e);
        WriteGMII16( 0x06, 0x46dc);
        WriteGMII16( 0x06, 0x19dd);
        WriteGMII16( 0x06, 0x19b2);
        WriteGMII16( 0x06, 0xe2d4);
        WriteGMII16( 0x06, 0x0001);
        WriteGMII16( 0x06, 0xbf1c);
        WriteGMII16( 0x06, 0x5b02);
        WriteGMII16( 0x06, 0x3a21);
        WriteGMII16( 0x06, 0xae1d);
        WriteGMII16( 0x06, 0xbee0);
        WriteGMII16( 0x06, 0xc4bf);
        WriteGMII16( 0x06, 0x85ec);
        WriteGMII16( 0x06, 0xd204);
        WriteGMII16( 0x06, 0xd819);
        WriteGMII16( 0x06, 0xd919);
        WriteGMII16( 0x06, 0x07dc);
        WriteGMII16( 0x06, 0x19dd);
        WriteGMII16( 0x06, 0x1907);
        WriteGMII16( 0x06, 0xb2f4);
        WriteGMII16( 0x06, 0xd400);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x1c5b);
        WriteGMII16( 0x06, 0x023a);
        WriteGMII16( 0x06, 0x21fe);
        WriteGMII16( 0x06, 0xef96);
        WriteGMII16( 0x06, 0xfec6);
        WriteGMII16( 0x06, 0xfefd);
        WriteGMII16( 0x06, 0xfc05);
        WriteGMII16( 0x06, 0xf9e2);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe3e0);
        WriteGMII16( 0x06, 0xeb5a);
        WriteGMII16( 0x06, 0x070c);
        WriteGMII16( 0x06, 0x031e);
        WriteGMII16( 0x06, 0x20e6);
        WriteGMII16( 0x06, 0xe0ea);
        WriteGMII16( 0x06, 0xe7e0);
        WriteGMII16( 0x06, 0xebe0);
        WriteGMII16( 0x06, 0xe0fc);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0xfdfd);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x69e0);
        WriteGMII16( 0x06, 0x8b80);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x22bf);
        WriteGMII16( 0x06, 0x47ba);
        WriteGMII16( 0x06, 0x0239);
        WriteGMII16( 0x06, 0xf4e0);
        WriteGMII16( 0x06, 0x8b44);
        WriteGMII16( 0x06, 0x1f01);
        WriteGMII16( 0x06, 0x9e15);
        WriteGMII16( 0x06, 0xe58b);
        WriteGMII16( 0x06, 0x44ad);
        WriteGMII16( 0x06, 0x2907);
        WriteGMII16( 0x06, 0xac28);
        WriteGMII16( 0x06, 0x04d1);
        WriteGMII16( 0x06, 0x01ae);
        WriteGMII16( 0x06, 0x02d1);
        WriteGMII16( 0x06, 0x00bf);
        WriteGMII16( 0x06, 0x8342);
        WriteGMII16( 0x06, 0x023a);
        WriteGMII16( 0x06, 0x21ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x30e0);
        WriteGMII16( 0x06, 0xe036);
        WriteGMII16( 0x06, 0xe1e0);
        WriteGMII16( 0x06, 0x37e1);
        WriteGMII16( 0x06, 0x8b3f);
        WriteGMII16( 0x06, 0x1f10);
        WriteGMII16( 0x06, 0x9e23);
        WriteGMII16( 0x06, 0xe48b);
        WriteGMII16( 0x06, 0x3fac);
        WriteGMII16( 0x06, 0x200b);
        WriteGMII16( 0x06, 0xac21);
        WriteGMII16( 0x06, 0x0dac);
        WriteGMII16( 0x06, 0x250f);
        WriteGMII16( 0x06, 0xac27);
        WriteGMII16( 0x06, 0x11ae);
        WriteGMII16( 0x06, 0x1202);
        WriteGMII16( 0x06, 0x2cb5);
        WriteGMII16( 0x06, 0xae0d);
        WriteGMII16( 0x06, 0x0282);
        WriteGMII16( 0x06, 0xe7ae);
        WriteGMII16( 0x06, 0x0802);
        WriteGMII16( 0x06, 0x2cd7);
        WriteGMII16( 0x06, 0xae03);
        WriteGMII16( 0x06, 0x022c);
        WriteGMII16( 0x06, 0xeafc);
        WriteGMII16( 0x06, 0x04f8);
        WriteGMII16( 0x06, 0xfaef);
        WriteGMII16( 0x06, 0x6902);
        WriteGMII16( 0x06, 0x8304);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x14e1);
        WriteGMII16( 0x06, 0xe015);
        WriteGMII16( 0x06, 0xad26);
        WriteGMII16( 0x06, 0x08d1);
        WriteGMII16( 0x06, 0x1ebf);
        WriteGMII16( 0x06, 0x2d47);
        WriteGMII16( 0x06, 0x023a);
        WriteGMII16( 0x06, 0x21ef);
        WriteGMII16( 0x06, 0x96fe);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0xf8e0);
        WriteGMII16( 0x06, 0x8b85);
        WriteGMII16( 0x06, 0xad27);
        WriteGMII16( 0x06, 0x2fd0);
        WriteGMII16( 0x06, 0x0b02);
        WriteGMII16( 0x06, 0x3826);
        WriteGMII16( 0x06, 0x5882);
        WriteGMII16( 0x06, 0x7882);
        WriteGMII16( 0x06, 0x9f24);
        WriteGMII16( 0x06, 0xe08b);
        WriteGMII16( 0x06, 0x32e1);
        WriteGMII16( 0x06, 0x8b33);
        WriteGMII16( 0x06, 0x1f10);
        WriteGMII16( 0x06, 0x9e1a);
        WriteGMII16( 0x06, 0x10e4);
        WriteGMII16( 0x06, 0x8b32);
        WriteGMII16( 0x06, 0xe0e0);
        WriteGMII16( 0x06, 0x28e1);
        WriteGMII16( 0x06, 0xe029);
        WriteGMII16( 0x06, 0xf72c);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0x28e5);
        WriteGMII16( 0x06, 0xe029);
        WriteGMII16( 0x06, 0xf62c);
        WriteGMII16( 0x06, 0xe4e0);
        WriteGMII16( 0x06, 0x28e5);
        WriteGMII16( 0x06, 0xe029);
        WriteGMII16( 0x06, 0xfc04);
        WriteGMII16( 0x06, 0x00e1);
        WriteGMII16( 0x06, 0x4077);
        WriteGMII16( 0x06, 0xe140);
        WriteGMII16( 0x06, 0xbbe0);
        WriteGMII16( 0x06, 0x2a00);
        WriteGMII16( 0x05, 0xe142);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16(0x06, gphy_val);
        WriteGMII16( 0x05, 0xe140);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16(0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16(0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            IODelay(100);
            gphy_val = ReadGMII16( 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        WriteGMII16(0x1f, 0x0007);
        WriteGMII16(0x1e, 0x0023);
        gphy_val = ReadGMII16( 0x17);
        gphy_val |= BIT_1;
        WriteGMII16(0x17, gphy_val);
        WriteGMII16(0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x09, 0xA20F);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x1f, 0x0003);
        WriteGMII16( 0x01, 0x328A);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16(0x1f, 0x0000);
        WriteGMII16(0x00, 0x9200);
    } else if (mcfg == CFG_METHOD_21) {
        WriteGMII16( 0x1f, 0x0B82);
        gphy_val = ReadGMII16( 0x10);
        gphy_val |= BIT_4;
        WriteGMII16( 0x10, gphy_val);
        WriteGMII16( 0x1f, 0x0B80);
        for (i = 0; i < 10; i++) {
            if (ReadGMII16( 0x10) & 0x0040)
                break;
            mdelay(10);
        }
        WriteGMII16( 0x1f, 0x0A43);
        WriteGMII16( 0x13, 0x8146);
        WriteGMII16( 0x14, 0x2300);
        WriteGMII16( 0x13, 0xB820);
        WriteGMII16( 0x14, 0x0210);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0xB820);
        WriteGMII16( 0x14, 0x0290);
        WriteGMII16( 0x13, 0xA012);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x13, 0xA014);
        WriteGMII16( 0x14, 0x2c04);
        WriteGMII16( 0x14, 0x2c1b);
        WriteGMII16( 0x14, 0x2c65);
        WriteGMII16( 0x14, 0x2d06);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x4092);
        WriteGMII16( 0x14, 0xba04);
        WriteGMII16( 0x14, 0x3084);
        WriteGMII16( 0x14, 0x1cf6);
        WriteGMII16( 0x14, 0x1ccf);
        WriteGMII16( 0x14, 0x1cda);
        WriteGMII16( 0x14, 0xaeff);
        WriteGMII16( 0x14, 0xaf02);
        WriteGMII16( 0x14, 0x8f02);
        WriteGMII16( 0x14, 0x8eff);
        WriteGMII16( 0x14, 0xce01);
        WriteGMII16( 0x14, 0xe070);
        WriteGMII16( 0x14, 0x0f00);
        WriteGMII16( 0x14, 0xaf01);
        WriteGMII16( 0x14, 0x8f01);
        WriteGMII16( 0x14, 0xd712);
        WriteGMII16( 0x14, 0x5fe8);
        WriteGMII16( 0x14, 0xaf02);
        WriteGMII16( 0x14, 0x8f02);
        WriteGMII16( 0x14, 0x8e01);
        WriteGMII16( 0x14, 0x1ce4);
        WriteGMII16( 0x14, 0x27f2);
        WriteGMII16( 0x14, 0xd05a);
        WriteGMII16( 0x14, 0xd19a);
        WriteGMII16( 0x14, 0xd709);
        WriteGMII16( 0x14, 0x608f);
        WriteGMII16( 0x14, 0xd06b);
        WriteGMII16( 0x14, 0xd18a);
        WriteGMII16( 0x14, 0x2c25);
        WriteGMII16( 0x14, 0xd0be);
        WriteGMII16( 0x14, 0xd188);
        WriteGMII16( 0x14, 0x2c25);
        WriteGMII16( 0x14, 0xd708);
        WriteGMII16( 0x14, 0x4072);
        WriteGMII16( 0x14, 0xc104);
        WriteGMII16( 0x14, 0x2c37);
        WriteGMII16( 0x14, 0x4076);
        WriteGMII16( 0x14, 0xc110);
        WriteGMII16( 0x14, 0x2c37);
        WriteGMII16( 0x14, 0x4071);
        WriteGMII16( 0x14, 0xc102);
        WriteGMII16( 0x14, 0x2c37);
        WriteGMII16( 0x14, 0x4070);
        WriteGMII16( 0x14, 0xc101);
        WriteGMII16( 0x14, 0x2c37);
        WriteGMII16( 0x14, 0x175b);
        WriteGMII16( 0x14, 0xd709);
        WriteGMII16( 0x14, 0x3390);
        WriteGMII16( 0x14, 0x5c32);
        WriteGMII16( 0x14, 0x2c47);
        WriteGMII16( 0x14, 0x175b);
        WriteGMII16( 0x14, 0xd708);
        WriteGMII16( 0x14, 0x6193);
        WriteGMII16( 0x14, 0xd709);
        WriteGMII16( 0x14, 0x5f9d);
        WriteGMII16( 0x14, 0x408b);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x6042);
        WriteGMII16( 0x14, 0xb401);
        WriteGMII16( 0x14, 0x175b);
        WriteGMII16( 0x14, 0xd708);
        WriteGMII16( 0x14, 0x6073);
        WriteGMII16( 0x14, 0x5fbc);
        WriteGMII16( 0x14, 0x2c46);
        WriteGMII16( 0x14, 0x26ed);
        WriteGMII16( 0x14, 0xb280);
        WriteGMII16( 0x14, 0xa841);
        WriteGMII16( 0x14, 0x9420);
        WriteGMII16( 0x14, 0x8710);
        WriteGMII16( 0x14, 0xd709);
        WriteGMII16( 0x14, 0x42ec);
        WriteGMII16( 0x14, 0x606d);
        WriteGMII16( 0x14, 0xd207);
        WriteGMII16( 0x14, 0x2c50);
        WriteGMII16( 0x14, 0xd203);
        WriteGMII16( 0x14, 0x33ff);
        WriteGMII16( 0x14, 0x563b);
        WriteGMII16( 0x14, 0x3275);
        WriteGMII16( 0x14, 0x7c57);
        WriteGMII16( 0x14, 0xb240);
        WriteGMII16( 0x14, 0xb402);
        WriteGMII16( 0x14, 0x263b);
        WriteGMII16( 0x14, 0x6096);
        WriteGMII16( 0x14, 0xb240);
        WriteGMII16( 0x14, 0xb406);
        WriteGMII16( 0x14, 0x263b);
        WriteGMII16( 0x14, 0x31d7);
        WriteGMII16( 0x14, 0x7c60);
        WriteGMII16( 0x14, 0xb240);
        WriteGMII16( 0x14, 0xb40e);
        WriteGMII16( 0x14, 0x263b);
        WriteGMII16( 0x14, 0xb410);
        WriteGMII16( 0x14, 0x8802);
        WriteGMII16( 0x14, 0xb240);
        WriteGMII16( 0x14, 0x940e);
        WriteGMII16( 0x14, 0x263b);
        WriteGMII16( 0x14, 0xba04);
        WriteGMII16( 0x14, 0x1ccf);
        WriteGMII16( 0x14, 0xa902);
        WriteGMII16( 0x14, 0xd711);
        WriteGMII16( 0x14, 0x4045);
        WriteGMII16( 0x14, 0xa980);
        WriteGMII16( 0x14, 0x3003);
        WriteGMII16( 0x14, 0x59b1);
        WriteGMII16( 0x14, 0xa540);
        WriteGMII16( 0x14, 0xa601);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4043);
        WriteGMII16( 0x14, 0xa910);
        WriteGMII16( 0x14, 0xd711);
        WriteGMII16( 0x14, 0x60a0);
        WriteGMII16( 0x14, 0xca33);
        WriteGMII16( 0x14, 0xcb33);
        WriteGMII16( 0x14, 0xa941);
        WriteGMII16( 0x14, 0x2c7b);
        WriteGMII16( 0x14, 0xcaff);
        WriteGMII16( 0x14, 0xcbff);
        WriteGMII16( 0x14, 0xa921);
        WriteGMII16( 0x14, 0xce02);
        WriteGMII16( 0x14, 0xe070);
        WriteGMII16( 0x14, 0x0f10);
        WriteGMII16( 0x14, 0xaf01);
        WriteGMII16( 0x14, 0x8f01);
        WriteGMII16( 0x14, 0x1766);
        WriteGMII16( 0x14, 0x8e02);
        WriteGMII16( 0x14, 0x1787);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x609c);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fa4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0x1ce2);
        WriteGMII16( 0x14, 0xce04);
        WriteGMII16( 0x14, 0xe070);
        WriteGMII16( 0x14, 0x0f20);
        WriteGMII16( 0x14, 0xaf01);
        WriteGMII16( 0x14, 0x8f01);
        WriteGMII16( 0x14, 0x1766);
        WriteGMII16( 0x14, 0x8e04);
        WriteGMII16( 0x14, 0x6044);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0xa520);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4043);
        WriteGMII16( 0x14, 0x2cba);
        WriteGMII16( 0x14, 0xe00f);
        WriteGMII16( 0x14, 0x0501);
        WriteGMII16( 0x14, 0x1ce8);
        WriteGMII16( 0x14, 0xb801);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x4060);
        WriteGMII16( 0x14, 0x7fc4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0x1cee);
        WriteGMII16( 0x14, 0xe00f);
        WriteGMII16( 0x14, 0x0502);
        WriteGMII16( 0x14, 0x1ce8);
        WriteGMII16( 0x14, 0xb802);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x4061);
        WriteGMII16( 0x14, 0x7fc4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0x1cee);
        WriteGMII16( 0x14, 0xe00f);
        WriteGMII16( 0x14, 0x0504);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6099);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fa4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0xc17f);
        WriteGMII16( 0x14, 0xc200);
        WriteGMII16( 0x14, 0xc43f);
        WriteGMII16( 0x14, 0xcc03);
        WriteGMII16( 0x14, 0xa701);
        WriteGMII16( 0x14, 0xa510);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4018);
        WriteGMII16( 0x14, 0x9910);
        WriteGMII16( 0x14, 0x8510);
        WriteGMII16( 0x14, 0x2860);
        WriteGMII16( 0x14, 0xe00f);
        WriteGMII16( 0x14, 0x0504);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6099);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fa4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0xa608);
        WriteGMII16( 0x14, 0xc17d);
        WriteGMII16( 0x14, 0xc200);
        WriteGMII16( 0x14, 0xc43f);
        WriteGMII16( 0x14, 0xcc03);
        WriteGMII16( 0x14, 0xa701);
        WriteGMII16( 0x14, 0xa510);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4018);
        WriteGMII16( 0x14, 0x9910);
        WriteGMII16( 0x14, 0x8510);
        WriteGMII16( 0x14, 0x2926);
        WriteGMII16( 0x14, 0x1792);
        WriteGMII16( 0x14, 0x27db);
        WriteGMII16( 0x14, 0xc000);
        WriteGMII16( 0x14, 0xc100);
        WriteGMII16( 0x14, 0xc200);
        WriteGMII16( 0x14, 0xc300);
        WriteGMII16( 0x14, 0xc400);
        WriteGMII16( 0x14, 0xc500);
        WriteGMII16( 0x14, 0xc600);
        WriteGMII16( 0x14, 0xc7c1);
        WriteGMII16( 0x14, 0xc800);
        WriteGMII16( 0x14, 0xcc00);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xca0f);
        WriteGMII16( 0x14, 0xcbff);
        WriteGMII16( 0x14, 0xa901);
        WriteGMII16( 0x14, 0x8902);
        WriteGMII16( 0x14, 0xc900);
        WriteGMII16( 0x14, 0xca00);
        WriteGMII16( 0x14, 0xcb00);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xb804);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x6044);
        WriteGMII16( 0x14, 0x9804);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6099);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fa4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xa510);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6098);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fa4);
        WriteGMII16( 0x14, 0x2ccd);
        WriteGMII16( 0x14, 0x8510);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xd711);
        WriteGMII16( 0x14, 0x3003);
        WriteGMII16( 0x14, 0x1cfa);
        WriteGMII16( 0x14, 0x2d04);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x60be);
        WriteGMII16( 0x14, 0xe060);
        WriteGMII16( 0x14, 0x0920);
        WriteGMII16( 0x14, 0x1ccf);
        WriteGMII16( 0x14, 0x2c82);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x3063);
        WriteGMII16( 0x14, 0x1948);
        WriteGMII16( 0x14, 0x288a);
        WriteGMII16( 0x14, 0x1ccf);
        WriteGMII16( 0x14, 0x29bd);
        WriteGMII16( 0x14, 0xa802);
        WriteGMII16( 0x14, 0xa303);
        WriteGMII16( 0x14, 0x843f);
        WriteGMII16( 0x14, 0x81ff);
        WriteGMII16( 0x14, 0x8208);
        WriteGMII16( 0x14, 0xa201);
        WriteGMII16( 0x14, 0xc001);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x30a0);
        WriteGMII16( 0x14, 0x0d15);
        WriteGMII16( 0x14, 0x30a0);
        WriteGMII16( 0x14, 0x3d0c);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7f4c);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0xe003);
        WriteGMII16( 0x14, 0x0202);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6090);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fac);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0xa20c);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6091);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fac);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0x820e);
        WriteGMII16( 0x14, 0xa3e0);
        WriteGMII16( 0x14, 0xa520);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x609d);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fac);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0x8520);
        WriteGMII16( 0x14, 0x6703);
        WriteGMII16( 0x14, 0x2d2d);
        WriteGMII16( 0x14, 0xa13e);
        WriteGMII16( 0x14, 0xc001);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0x6046);
        WriteGMII16( 0x14, 0x2d06);
        WriteGMII16( 0x14, 0xa43f);
        WriteGMII16( 0x14, 0xa101);
        WriteGMII16( 0x14, 0xc020);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x3121);
        WriteGMII16( 0x14, 0x0d3e);
        WriteGMII16( 0x14, 0x30c0);
        WriteGMII16( 0x14, 0x3d06);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7f4c);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0xa540);
        WriteGMII16( 0x14, 0xc001);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4001);
        WriteGMII16( 0x14, 0xe00f);
        WriteGMII16( 0x14, 0x0501);
        WriteGMII16( 0x14, 0x1da5);
        WriteGMII16( 0x14, 0xc1c4);
        WriteGMII16( 0x14, 0xa268);
        WriteGMII16( 0x14, 0xa303);
        WriteGMII16( 0x14, 0x8420);
        WriteGMII16( 0x14, 0xe00f);
        WriteGMII16( 0x14, 0x0502);
        WriteGMII16( 0x14, 0x1da5);
        WriteGMII16( 0x14, 0xc002);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0x8208);
        WriteGMII16( 0x14, 0x8410);
        WriteGMII16( 0x14, 0xa121);
        WriteGMII16( 0x14, 0xc002);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0x8120);
        WriteGMII16( 0x14, 0x8180);
        WriteGMII16( 0x14, 0x1d90);
        WriteGMII16( 0x14, 0xa180);
        WriteGMII16( 0x14, 0xa13a);
        WriteGMII16( 0x14, 0x8240);
        WriteGMII16( 0x14, 0xa430);
        WriteGMII16( 0x14, 0xc010);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x30e1);
        WriteGMII16( 0x14, 0x0abc);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7f8c);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0xa480);
        WriteGMII16( 0x14, 0xa230);
        WriteGMII16( 0x14, 0xa303);
        WriteGMII16( 0x14, 0xc001);
        WriteGMII16( 0x14, 0xd70c);
        WriteGMII16( 0x14, 0x4124);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x6120);
        WriteGMII16( 0x14, 0xd711);
        WriteGMII16( 0x14, 0x3128);
        WriteGMII16( 0x14, 0x3d6f);
        WriteGMII16( 0x14, 0x2d69);
        WriteGMII16( 0x14, 0xa801);
        WriteGMII16( 0x14, 0x2d65);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0xe018);
        WriteGMII16( 0x14, 0x0208);
        WriteGMII16( 0x14, 0xa1f8);
        WriteGMII16( 0x14, 0x8480);
        WriteGMII16( 0x14, 0xc004);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0x6046);
        WriteGMII16( 0x14, 0x2d06);
        WriteGMII16( 0x14, 0xa43f);
        WriteGMII16( 0x14, 0xa105);
        WriteGMII16( 0x14, 0x8228);
        WriteGMII16( 0x14, 0xc004);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0x81bc);
        WriteGMII16( 0x14, 0xa220);
        WriteGMII16( 0x14, 0x1d90);
        WriteGMII16( 0x14, 0x8220);
        WriteGMII16( 0x14, 0xa1bc);
        WriteGMII16( 0x14, 0xc040);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x30e1);
        WriteGMII16( 0x14, 0x0abc);
        WriteGMII16( 0x14, 0x30e1);
        WriteGMII16( 0x14, 0x3d06);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7f4c);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0xa802);
        WriteGMII16( 0x14, 0xd70c);
        WriteGMII16( 0x14, 0x4244);
        WriteGMII16( 0x14, 0xa301);
        WriteGMII16( 0x14, 0xc004);
        WriteGMII16( 0x14, 0xd711);
        WriteGMII16( 0x14, 0x3128);
        WriteGMII16( 0x14, 0x3d9e);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x5f80);
        WriteGMII16( 0x14, 0xd711);
        WriteGMII16( 0x14, 0x3109);
        WriteGMII16( 0x14, 0x3da0);
        WriteGMII16( 0x14, 0x2da4);
        WriteGMII16( 0x14, 0xa801);
        WriteGMII16( 0x14, 0x2d93);
        WriteGMII16( 0x14, 0xa802);
        WriteGMII16( 0x14, 0xc004);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x4000);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x14, 0xa510);
        WriteGMII16( 0x14, 0xd710);
        WriteGMII16( 0x14, 0x609a);
        WriteGMII16( 0x14, 0xd71e);
        WriteGMII16( 0x14, 0x7fac);
        WriteGMII16( 0x14, 0x2ab6);
        WriteGMII16( 0x14, 0x8510);
        WriteGMII16( 0x14, 0x0800);
        WriteGMII16( 0x13, 0xA01A);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x13, 0xA006);
        WriteGMII16( 0x14, 0x0ad6);
        WriteGMII16( 0x13, 0xA004);
        WriteGMII16( 0x14, 0x07f5);
        WriteGMII16( 0x13, 0xA002);
        WriteGMII16( 0x14, 0x06cc);
        WriteGMII16( 0x13, 0xA000);
        WriteGMII16( 0x14, 0xf7db);
        WriteGMII16( 0x13, 0xB820);
        WriteGMII16( 0x14, 0x0210);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x83a0);
        WriteGMII16( 0x14, 0xaf83);
        WriteGMII16( 0x14, 0xacaf);
        WriteGMII16( 0x14, 0x83b8);
        WriteGMII16( 0x14, 0xaf83);
        WriteGMII16( 0x14, 0xcdaf);
        WriteGMII16( 0x14, 0x83d3);
        WriteGMII16( 0x14, 0x0204);
        WriteGMII16( 0x14, 0x9a02);
        WriteGMII16( 0x14, 0x09a9);
        WriteGMII16( 0x14, 0x0284);
        WriteGMII16( 0x14, 0x61af);
        WriteGMII16( 0x14, 0x02fc);
        WriteGMII16( 0x14, 0xad20);
        WriteGMII16( 0x14, 0x0302);
        WriteGMII16( 0x14, 0x867c);
        WriteGMII16( 0x14, 0xad21);
        WriteGMII16( 0x14, 0x0302);
        WriteGMII16( 0x14, 0x85c9);
        WriteGMII16( 0x14, 0xad22);
        WriteGMII16( 0x14, 0x0302);
        WriteGMII16( 0x14, 0x1bc0);
        WriteGMII16( 0x14, 0xaf17);
        WriteGMII16( 0x14, 0xe302);
        WriteGMII16( 0x14, 0x8703);
        WriteGMII16( 0x14, 0xaf18);
        WriteGMII16( 0x14, 0x6201);
        WriteGMII16( 0x14, 0x06e0);
        WriteGMII16( 0x14, 0x8148);
        WriteGMII16( 0x14, 0xaf3c);
        WriteGMII16( 0x14, 0x69f8);
        WriteGMII16( 0x14, 0xf9fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0x10f7);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0x131f);
        WriteGMII16( 0x14, 0xd104);
        WriteGMII16( 0x14, 0xbf87);
        WriteGMII16( 0x14, 0xf302);
        WriteGMII16( 0x14, 0x4259);
        WriteGMII16( 0x14, 0x0287);
        WriteGMII16( 0x14, 0x88bf);
        WriteGMII16( 0x14, 0x87cf);
        WriteGMII16( 0x14, 0xd7b8);
        WriteGMII16( 0x14, 0x22d0);
        WriteGMII16( 0x14, 0x0c02);
        WriteGMII16( 0x14, 0x4252);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xcda0);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xce8b);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xd1f5);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xd2a9);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xd30a);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xf010);
        WriteGMII16( 0x14, 0xee80);
        WriteGMII16( 0x14, 0xf38f);
        WriteGMII16( 0x14, 0xee81);
        WriteGMII16( 0x14, 0x011e);
        WriteGMII16( 0x14, 0xee81);
        WriteGMII16( 0x14, 0x0b4a);
        WriteGMII16( 0x14, 0xee81);
        WriteGMII16( 0x14, 0x0c7c);
        WriteGMII16( 0x14, 0xee81);
        WriteGMII16( 0x14, 0x127f);
        WriteGMII16( 0x14, 0xd100);
        WriteGMII16( 0x14, 0x0210);
        WriteGMII16( 0x14, 0xb5ee);
        WriteGMII16( 0x14, 0x8088);
        WriteGMII16( 0x14, 0xa4ee);
        WriteGMII16( 0x14, 0x8089);
        WriteGMII16( 0x14, 0x44ee);
        WriteGMII16( 0x14, 0x809a);
        WriteGMII16( 0x14, 0xa4ee);
        WriteGMII16( 0x14, 0x809b);
        WriteGMII16( 0x14, 0x44ee);
        WriteGMII16( 0x14, 0x809c);
        WriteGMII16( 0x14, 0xa7ee);
        WriteGMII16( 0x14, 0x80a5);
        WriteGMII16( 0x14, 0xa7d2);
        WriteGMII16( 0x14, 0x0002);
        WriteGMII16( 0x14, 0x0e66);
        WriteGMII16( 0x14, 0x0285);
        WriteGMII16( 0x14, 0xc0ee);
        WriteGMII16( 0x14, 0x87fc);
        WriteGMII16( 0x14, 0x00e0);
        WriteGMII16( 0x14, 0x8245);
        WriteGMII16( 0x14, 0xf622);
        WriteGMII16( 0x14, 0xe482);
        WriteGMII16( 0x14, 0x45ef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfdfc);
        WriteGMII16( 0x14, 0x0402);
        WriteGMII16( 0x14, 0x847a);
        WriteGMII16( 0x14, 0x0284);
        WriteGMII16( 0x14, 0xb302);
        WriteGMII16( 0x14, 0x0cab);
        WriteGMII16( 0x14, 0x020c);
        WriteGMII16( 0x14, 0xc402);
        WriteGMII16( 0x14, 0x0cef);
        WriteGMII16( 0x14, 0x020d);
        WriteGMII16( 0x14, 0x0802);
        WriteGMII16( 0x14, 0x0d33);
        WriteGMII16( 0x14, 0x020c);
        WriteGMII16( 0x14, 0x3d04);
        WriteGMII16( 0x14, 0xf8fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xe182);
        WriteGMII16( 0x14, 0x2fac);
        WriteGMII16( 0x14, 0x291a);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x24ac);
        WriteGMII16( 0x14, 0x2102);
        WriteGMII16( 0x14, 0xae22);
        WriteGMII16( 0x14, 0x0210);
        WriteGMII16( 0x14, 0x57f6);
        WriteGMII16( 0x14, 0x21e4);
        WriteGMII16( 0x14, 0x8224);
        WriteGMII16( 0x14, 0xd101);
        WriteGMII16( 0x14, 0xbf44);
        WriteGMII16( 0x14, 0xd202);
        WriteGMII16( 0x14, 0x4259);
        WriteGMII16( 0x14, 0xae10);
        WriteGMII16( 0x14, 0x0212);
        WriteGMII16( 0x14, 0x4cf6);
        WriteGMII16( 0x14, 0x29e5);
        WriteGMII16( 0x14, 0x822f);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x24f6);
        WriteGMII16( 0x14, 0x21e4);
        WriteGMII16( 0x14, 0x8224);
        WriteGMII16( 0x14, 0xef96);
        WriteGMII16( 0x14, 0xfefc);
        WriteGMII16( 0x14, 0x04f8);
        WriteGMII16( 0x14, 0xe182);
        WriteGMII16( 0x14, 0x2fac);
        WriteGMII16( 0x14, 0x2a18);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x24ac);
        WriteGMII16( 0x14, 0x2202);
        WriteGMII16( 0x14, 0xae26);
        WriteGMII16( 0x14, 0x0284);
        WriteGMII16( 0x14, 0xf802);
        WriteGMII16( 0x14, 0x8565);
        WriteGMII16( 0x14, 0xd101);
        WriteGMII16( 0x14, 0xbf44);
        WriteGMII16( 0x14, 0xd502);
        WriteGMII16( 0x14, 0x4259);
        WriteGMII16( 0x14, 0xae0e);
        WriteGMII16( 0x14, 0x0284);
        WriteGMII16( 0x14, 0xea02);
        WriteGMII16( 0x14, 0x85a9);
        WriteGMII16( 0x14, 0xe182);
        WriteGMII16( 0x14, 0x2ff6);
        WriteGMII16( 0x14, 0x2ae5);
        WriteGMII16( 0x14, 0x822f);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x24f6);
        WriteGMII16( 0x14, 0x22e4);
        WriteGMII16( 0x14, 0x8224);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xf9e2);
        WriteGMII16( 0x14, 0x8011);
        WriteGMII16( 0x14, 0xad31);
        WriteGMII16( 0x14, 0x05d2);
        WriteGMII16( 0x14, 0x0002);
        WriteGMII16( 0x14, 0x0e66);
        WriteGMII16( 0x14, 0xfd04);
        WriteGMII16( 0x14, 0xf8f9);
        WriteGMII16( 0x14, 0xfaef);
        WriteGMII16( 0x14, 0x69e0);
        WriteGMII16( 0x14, 0x8011);
        WriteGMII16( 0x14, 0xad21);
        WriteGMII16( 0x14, 0x5cbf);
        WriteGMII16( 0x14, 0x43be);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97ac);
        WriteGMII16( 0x14, 0x281b);
        WriteGMII16( 0x14, 0xbf43);
        WriteGMII16( 0x14, 0xc102);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0xac28);
        WriteGMII16( 0x14, 0x12bf);
        WriteGMII16( 0x14, 0x43c7);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97ac);
        WriteGMII16( 0x14, 0x2804);
        WriteGMII16( 0x14, 0xd300);
        WriteGMII16( 0x14, 0xae07);
        WriteGMII16( 0x14, 0xd306);
        WriteGMII16( 0x14, 0xaf85);
        WriteGMII16( 0x14, 0x56d3);
        WriteGMII16( 0x14, 0x03e0);
        WriteGMII16( 0x14, 0x8011);
        WriteGMII16( 0x14, 0xad26);
        WriteGMII16( 0x14, 0x25bf);
        WriteGMII16( 0x14, 0x4559);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97e2);
        WriteGMII16( 0x14, 0x8073);
        WriteGMII16( 0x14, 0x0d21);
        WriteGMII16( 0x14, 0xf637);
        WriteGMII16( 0x14, 0x0d11);
        WriteGMII16( 0x14, 0xf62f);
        WriteGMII16( 0x14, 0x1b21);
        WriteGMII16( 0x14, 0xaa02);
        WriteGMII16( 0x14, 0xae10);
        WriteGMII16( 0x14, 0xe280);
        WriteGMII16( 0x14, 0x740d);
        WriteGMII16( 0x14, 0x21f6);
        WriteGMII16( 0x14, 0x371b);
        WriteGMII16( 0x14, 0x21aa);
        WriteGMII16( 0x14, 0x0313);
        WriteGMII16( 0x14, 0xae02);
        WriteGMII16( 0x14, 0x2b02);
        WriteGMII16( 0x14, 0x020e);
        WriteGMII16( 0x14, 0x5102);
        WriteGMII16( 0x14, 0x0e66);
        WriteGMII16( 0x14, 0x020f);
        WriteGMII16( 0x14, 0xa3ef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfdfc);
        WriteGMII16( 0x14, 0x04f8);
        WriteGMII16( 0x14, 0xf9fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xe080);
        WriteGMII16( 0x14, 0x12ad);
        WriteGMII16( 0x14, 0x2733);
        WriteGMII16( 0x14, 0xbf43);
        WriteGMII16( 0x14, 0xbe02);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0xac28);
        WriteGMII16( 0x14, 0x09bf);
        WriteGMII16( 0x14, 0x43c1);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97ad);
        WriteGMII16( 0x14, 0x2821);
        WriteGMII16( 0x14, 0xbf45);
        WriteGMII16( 0x14, 0x5902);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0xe387);
        WriteGMII16( 0x14, 0xffd2);
        WriteGMII16( 0x14, 0x001b);
        WriteGMII16( 0x14, 0x45ac);
        WriteGMII16( 0x14, 0x2711);
        WriteGMII16( 0x14, 0xe187);
        WriteGMII16( 0x14, 0xfebf);
        WriteGMII16( 0x14, 0x87e4);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x590d);
        WriteGMII16( 0x14, 0x11bf);
        WriteGMII16( 0x14, 0x87e7);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59ef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfdfc);
        WriteGMII16( 0x14, 0x04f8);
        WriteGMII16( 0x14, 0xfaef);
        WriteGMII16( 0x14, 0x69d1);
        WriteGMII16( 0x14, 0x00bf);
        WriteGMII16( 0x14, 0x87e4);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59bf);
        WriteGMII16( 0x14, 0x87e7);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59ef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xee87);
        WriteGMII16( 0x14, 0xff46);
        WriteGMII16( 0x14, 0xee87);
        WriteGMII16( 0x14, 0xfe01);
        WriteGMII16( 0x14, 0x04f8);
        WriteGMII16( 0x14, 0xfaef);
        WriteGMII16( 0x14, 0x69e0);
        WriteGMII16( 0x14, 0x8241);
        WriteGMII16( 0x14, 0xa000);
        WriteGMII16( 0x14, 0x0502);
        WriteGMII16( 0x14, 0x85eb);
        WriteGMII16( 0x14, 0xae0e);
        WriteGMII16( 0x14, 0xa001);
        WriteGMII16( 0x14, 0x0502);
        WriteGMII16( 0x14, 0x1a5a);
        WriteGMII16( 0x14, 0xae06);
        WriteGMII16( 0x14, 0xa002);
        WriteGMII16( 0x14, 0x0302);
        WriteGMII16( 0x14, 0x1ae6);
        WriteGMII16( 0x14, 0xef96);
        WriteGMII16( 0x14, 0xfefc);
        WriteGMII16( 0x14, 0x04f8);
        WriteGMII16( 0x14, 0xf9fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x29f6);
        WriteGMII16( 0x14, 0x21e4);
        WriteGMII16( 0x14, 0x8229);
        WriteGMII16( 0x14, 0xe080);
        WriteGMII16( 0x14, 0x10ac);
        WriteGMII16( 0x14, 0x2202);
        WriteGMII16( 0x14, 0xae76);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x27f7);
        WriteGMII16( 0x14, 0x21e4);
        WriteGMII16( 0x14, 0x8227);
        WriteGMII16( 0x14, 0xbf43);
        WriteGMII16( 0x14, 0x1302);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0xef21);
        WriteGMII16( 0x14, 0xbf43);
        WriteGMII16( 0x14, 0x1602);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0x0c11);
        WriteGMII16( 0x14, 0x1e21);
        WriteGMII16( 0x14, 0xbf43);
        WriteGMII16( 0x14, 0x1902);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0x0c12);
        WriteGMII16( 0x14, 0x1e21);
        WriteGMII16( 0x14, 0xe682);
        WriteGMII16( 0x14, 0x43a2);
        WriteGMII16( 0x14, 0x000a);
        WriteGMII16( 0x14, 0xe182);
        WriteGMII16( 0x14, 0x27f6);
        WriteGMII16( 0x14, 0x29e5);
        WriteGMII16( 0x14, 0x8227);
        WriteGMII16( 0x14, 0xae42);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x44f7);
        WriteGMII16( 0x14, 0x21e4);
        WriteGMII16( 0x14, 0x8244);
        WriteGMII16( 0x14, 0x0246);
        WriteGMII16( 0x14, 0xaebf);
        WriteGMII16( 0x14, 0x4325);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97ef);
        WriteGMII16( 0x14, 0x21bf);
        WriteGMII16( 0x14, 0x431c);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x970c);
        WriteGMII16( 0x14, 0x121e);
        WriteGMII16( 0x14, 0x21bf);
        WriteGMII16( 0x14, 0x431f);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x970c);
        WriteGMII16( 0x14, 0x131e);
        WriteGMII16( 0x14, 0x21bf);
        WriteGMII16( 0x14, 0x4328);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x970c);
        WriteGMII16( 0x14, 0x141e);
        WriteGMII16( 0x14, 0x21bf);
        WriteGMII16( 0x14, 0x44b1);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x970c);
        WriteGMII16( 0x14, 0x161e);
        WriteGMII16( 0x14, 0x21e6);
        WriteGMII16( 0x14, 0x8242);
        WriteGMII16( 0x14, 0xee82);
        WriteGMII16( 0x14, 0x4101);
        WriteGMII16( 0x14, 0xef96);
        WriteGMII16( 0x14, 0xfefd);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xf8fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x46a0);
        WriteGMII16( 0x14, 0x0005);
        WriteGMII16( 0x14, 0x0286);
        WriteGMII16( 0x14, 0x96ae);
        WriteGMII16( 0x14, 0x06a0);
        WriteGMII16( 0x14, 0x0103);
        WriteGMII16( 0x14, 0x0219);
        WriteGMII16( 0x14, 0x19ef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xf8fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x29f6);
        WriteGMII16( 0x14, 0x20e4);
        WriteGMII16( 0x14, 0x8229);
        WriteGMII16( 0x14, 0xe080);
        WriteGMII16( 0x14, 0x10ac);
        WriteGMII16( 0x14, 0x2102);
        WriteGMII16( 0x14, 0xae54);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x27f7);
        WriteGMII16( 0x14, 0x20e4);
        WriteGMII16( 0x14, 0x8227);
        WriteGMII16( 0x14, 0xbf42);
        WriteGMII16( 0x14, 0xe602);
        WriteGMII16( 0x14, 0x4297);
        WriteGMII16( 0x14, 0xac28);
        WriteGMII16( 0x14, 0x22bf);
        WriteGMII16( 0x14, 0x430d);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97e5);
        WriteGMII16( 0x14, 0x8247);
        WriteGMII16( 0x14, 0xac28);
        WriteGMII16( 0x14, 0x20d1);
        WriteGMII16( 0x14, 0x03bf);
        WriteGMII16( 0x14, 0x4307);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59ee);
        WriteGMII16( 0x14, 0x8246);
        WriteGMII16( 0x14, 0x00e1);
        WriteGMII16( 0x14, 0x8227);
        WriteGMII16( 0x14, 0xf628);
        WriteGMII16( 0x14, 0xe582);
        WriteGMII16( 0x14, 0x27ae);
        WriteGMII16( 0x14, 0x21d1);
        WriteGMII16( 0x14, 0x04bf);
        WriteGMII16( 0x14, 0x4307);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59ae);
        WriteGMII16( 0x14, 0x08d1);
        WriteGMII16( 0x14, 0x05bf);
        WriteGMII16( 0x14, 0x4307);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59e0);
        WriteGMII16( 0x14, 0x8244);
        WriteGMII16( 0x14, 0xf720);
        WriteGMII16( 0x14, 0xe482);
        WriteGMII16( 0x14, 0x4402);
        WriteGMII16( 0x14, 0x46ae);
        WriteGMII16( 0x14, 0xee82);
        WriteGMII16( 0x14, 0x4601);
        WriteGMII16( 0x14, 0xef96);
        WriteGMII16( 0x14, 0xfefc);
        WriteGMII16( 0x14, 0x04f8);
        WriteGMII16( 0x14, 0xfaef);
        WriteGMII16( 0x14, 0x69e0);
        WriteGMII16( 0x14, 0x8013);
        WriteGMII16( 0x14, 0xad24);
        WriteGMII16( 0x14, 0x1cbf);
        WriteGMII16( 0x14, 0x87f0);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x97ad);
        WriteGMII16( 0x14, 0x2813);
        WriteGMII16( 0x14, 0xe087);
        WriteGMII16( 0x14, 0xfca0);
        WriteGMII16( 0x14, 0x0005);
        WriteGMII16( 0x14, 0x0287);
        WriteGMII16( 0x14, 0x36ae);
        WriteGMII16( 0x14, 0x10a0);
        WriteGMII16( 0x14, 0x0105);
        WriteGMII16( 0x14, 0x0287);
        WriteGMII16( 0x14, 0x48ae);
        WriteGMII16( 0x14, 0x08e0);
        WriteGMII16( 0x14, 0x8230);
        WriteGMII16( 0x14, 0xf626);
        WriteGMII16( 0x14, 0xe482);
        WriteGMII16( 0x14, 0x30ef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xf8e0);
        WriteGMII16( 0x14, 0x8245);
        WriteGMII16( 0x14, 0xf722);
        WriteGMII16( 0x14, 0xe482);
        WriteGMII16( 0x14, 0x4502);
        WriteGMII16( 0x14, 0x46ae);
        WriteGMII16( 0x14, 0xee87);
        WriteGMII16( 0x14, 0xfc01);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xf8fa);
        WriteGMII16( 0x14, 0xef69);
        WriteGMII16( 0x14, 0xfb02);
        WriteGMII16( 0x14, 0x46d3);
        WriteGMII16( 0x14, 0xad50);
        WriteGMII16( 0x14, 0x2fbf);
        WriteGMII16( 0x14, 0x87ed);
        WriteGMII16( 0x14, 0xd101);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59bf);
        WriteGMII16( 0x14, 0x87ed);
        WriteGMII16( 0x14, 0xd100);
        WriteGMII16( 0x14, 0x0242);
        WriteGMII16( 0x14, 0x59e0);
        WriteGMII16( 0x14, 0x8245);
        WriteGMII16( 0x14, 0xf622);
        WriteGMII16( 0x14, 0xe482);
        WriteGMII16( 0x14, 0x4502);
        WriteGMII16( 0x14, 0x46ae);
        WriteGMII16( 0x14, 0xd100);
        WriteGMII16( 0x14, 0xbf87);
        WriteGMII16( 0x14, 0xf002);
        WriteGMII16( 0x14, 0x4259);
        WriteGMII16( 0x14, 0xee87);
        WriteGMII16( 0x14, 0xfc00);
        WriteGMII16( 0x14, 0xe082);
        WriteGMII16( 0x14, 0x30f6);
        WriteGMII16( 0x14, 0x26e4);
        WriteGMII16( 0x14, 0x8230);
        WriteGMII16( 0x14, 0xffef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xfc04);
        WriteGMII16( 0x14, 0xf8f9);
        WriteGMII16( 0x14, 0xface);
        WriteGMII16( 0x14, 0xfaef);
        WriteGMII16( 0x14, 0x69fb);
        WriteGMII16( 0x14, 0xbf87);
        WriteGMII16( 0x14, 0xb3d7);
        WriteGMII16( 0x14, 0x001c);
        WriteGMII16( 0x14, 0xd819);
        WriteGMII16( 0x14, 0xd919);
        WriteGMII16( 0x14, 0xda19);
        WriteGMII16( 0x14, 0xdb19);
        WriteGMII16( 0x14, 0x07ef);
        WriteGMII16( 0x14, 0x9502);
        WriteGMII16( 0x14, 0x4259);
        WriteGMII16( 0x14, 0x073f);
        WriteGMII16( 0x14, 0x0004);
        WriteGMII16( 0x14, 0x9fec);
        WriteGMII16( 0x14, 0xffef);
        WriteGMII16( 0x14, 0x96fe);
        WriteGMII16( 0x14, 0xc6fe);
        WriteGMII16( 0x14, 0xfdfc);
        WriteGMII16( 0x14, 0x0400);
        WriteGMII16( 0x14, 0x0145);
        WriteGMII16( 0x14, 0x7d00);
        WriteGMII16( 0x14, 0x0345);
        WriteGMII16( 0x14, 0x5c00);
        WriteGMII16( 0x14, 0x0143);
        WriteGMII16( 0x14, 0x4f00);
        WriteGMII16( 0x14, 0x0387);
        WriteGMII16( 0x14, 0xdb00);
        WriteGMII16( 0x14, 0x0987);
        WriteGMII16( 0x14, 0xde00);
        WriteGMII16( 0x14, 0x0987);
        WriteGMII16( 0x14, 0xe100);
        WriteGMII16( 0x14, 0x0087);
        WriteGMII16( 0x14, 0xeaa4);
        WriteGMII16( 0x14, 0x00b8);
        WriteGMII16( 0x14, 0x20c4);
        WriteGMII16( 0x14, 0x1600);
        WriteGMII16( 0x14, 0x000f);
        WriteGMII16( 0x14, 0xf800);
        WriteGMII16( 0x14, 0x7098);
        WriteGMII16( 0x14, 0xa58a);
        WriteGMII16( 0x14, 0xb6a8);
        WriteGMII16( 0x14, 0x3e50);
        WriteGMII16( 0x14, 0xa83e);
        WriteGMII16( 0x14, 0x33bc);
        WriteGMII16( 0x14, 0xc622);
        WriteGMII16( 0x14, 0xbcc6);
        WriteGMII16( 0x14, 0xaaa4);
        WriteGMII16( 0x14, 0x42ff);
        WriteGMII16( 0x14, 0xc408);
        WriteGMII16( 0x14, 0x00c4);
        WriteGMII16( 0x14, 0x16a8);
        WriteGMII16( 0x14, 0xbcc0);
        WriteGMII16( 0x13, 0xb818);
        WriteGMII16( 0x14, 0x02f3);
        WriteGMII16( 0x13, 0xb81a);
        WriteGMII16( 0x14, 0x17d1);
        WriteGMII16( 0x13, 0xb81c);
        WriteGMII16( 0x14, 0x185a);
        WriteGMII16( 0x13, 0xb81e);
        WriteGMII16( 0x14, 0x3c66);
        WriteGMII16( 0x13, 0xb820);
        WriteGMII16( 0x14, 0x021f);
        WriteGMII16( 0x13, 0xc416);
        WriteGMII16( 0x14, 0x0500);
        WriteGMII16( 0x13, 0xb82e);
        WriteGMII16( 0x14, 0xfffc);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x0000);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x1f, 0x0B82);
        gphy_val = ReadGMII16( 0x10);
        gphy_val &= ~(BIT_9);
        WriteGMII16( 0x10, gphy_val);
        WriteGMII16( 0x1f, 0x0A43);
        WriteGMII16( 0x13, 0x8146);
        WriteGMII16( 0x14, 0x0000);
        WriteGMII16( 0x1f, 0x0B82);
        gphy_val = ReadGMII16( 0x10);
        gphy_val &= ~(BIT_4);
        WriteGMII16( 0x10, gphy_val);
    } else if (mcfg == CFG_METHOD_25) {
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
    } else if (mcfg == MCFG_8411B) {
        WriteGMII16(0x1f, 0x0B82);
        csi_tmp = ReadGMII16( 0x10);
        csi_tmp |= BIT_4;
        WriteGMII16(0x10, csi_tmp);

        WriteGMII16(0x1f, 0x0B80);
        i = 0;
        do {
            csi_tmp = ReadGMII16( 0x10);
            csi_tmp &= 0x0040;
            IODelay(50);
            IODelay(50);
            i++;
        } while(csi_tmp != 0x0040 && i <1000);

        WriteGMII16(0x1f, 0x0A43);
        WriteGMII16(0x13, 0x8146);
        WriteGMII16(0x14, 0x0100);
        WriteGMII16(0x13, 0xB82E);
        WriteGMII16(0x14, 0x0001);


        WriteGMII16(0x1F, 0x0A43);
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


        WriteGMII16(0x1F, 0x0A43);
        WriteGMII16(0x13, 0x0000);
        WriteGMII16(0x14, 0x0000);
        WriteGMII16(0x1f, 0x0B82);
        csi_tmp = ReadGMII16( 0x17);
        csi_tmp &= ~(BIT_0);
        WriteGMII16(0x17, csi_tmp);
        WriteGMII16(0x1f, 0x0A43);
        WriteGMII16(0x13, 0x8146);
        WriteGMII16(0x14, 0x0000);


        WriteGMII16(0x1f, 0x0B82);
        csi_tmp = ReadGMII16( 0x10);
        csi_tmp &= ~(BIT_4);
        WriteGMII16(0x10, csi_tmp);
    }

    rtl8168_write_hw_phy_mcu_code_ver(dev);

    WriteGMII16(0x1F, 0x0000);
}

static void
rtl8168_hw_phy_config(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    unsigned int gphy_val,i;
    unsigned long flags;

    tp->phy_reset_enable(dev);

    spin_lock_irqsave(&tp->phy_lock, flags);

    rtl8168_init_hw_phy_mcu(dev);

    if (mcfg == MCFG_8168B_1) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0x94B0);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x12, 0x6096);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x0D, 0xF8A0);
    }
    else if (mcfg == MCFG_8168B_2) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0x94B0);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x12, 0x6096);

        WriteGMII16( 0x1F, 0x0000);
    } 
    else if (mcfg == MCFG_8168B_3) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0x94B0);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x12, 0x6096);

        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168C_1) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x12, 0x2300);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x16, 0x000A);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x12, 0xC096);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x00, 0x88DE);
        WriteGMII16( 0x01, 0x82B1);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x08, 0x9E30);
        WriteGMII16( 0x09, 0x01F0);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0A, 0x5500);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x03, 0x7002);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0C, 0x00C8);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | (1 << 5));
        WriteGMII16( 0x0D, ReadGMII16( 0x0D) & ~(1 << 5));
    }
    else if (mcfg == MCFG_8168C_2) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x12, 0x2300);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x16, 0x0F0A);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x00, 0x88DE);
        WriteGMII16( 0x01, 0x82B1);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0C, 0x7EB8);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x06, 0x0761);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x03, 0x802F);
        WriteGMII16( 0x02, 0x4F02);
        WriteGMII16( 0x01, 0x0409);
        WriteGMII16( 0x00, 0xF099);
        WriteGMII16( 0x04, 0x9800);
        WriteGMII16( 0x04, 0x9000);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x16, ReadGMII16( 0x16) | (1 << 0));

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | (1 << 5));
        WriteGMII16( 0x0D, ReadGMII16( 0x0D) & ~(1 << 5));

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x1D, 0x3D98);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);
        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168C_3) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x12, 0x2300);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x16, 0x0F0A);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x00, 0x88DE);
        WriteGMII16( 0x01, 0x82B1);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0C, 0x7EB8);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x06, 0x5461);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x06, 0x5461);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x16, ReadGMII16( 0x16) | (1 << 0));

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | (1 << 5));
        WriteGMII16( 0x0D, ReadGMII16( 0x0D) & ~(1 << 5));

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x1D, 0x3D98);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);
        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168CP_1) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | (1 << 5));
        WriteGMII16( 0x0D, ReadGMII16( 0x0D) & ~(1 << 5));

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x1D, 0x3D98);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x14, 0xCAA3);
        WriteGMII16( 0x1C, 0x000A);
        WriteGMII16( 0x18, 0x65D0);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x17, 0xB580);
        WriteGMII16( 0x18, 0xFF54);
        WriteGMII16( 0x19, 0x3954);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0D, 0x310C);
        WriteGMII16( 0x0E, 0x310C);
        WriteGMII16( 0x0F, 0x311C);
        WriteGMII16( 0x06, 0x0761);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x18, 0xFF55);
        WriteGMII16( 0x19, 0x3955);
        WriteGMII16( 0x18, 0xFF54);
        WriteGMII16( 0x19, 0x3954);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);

        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168CP_2) {
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | (1 << 5));
        WriteGMII16( 0x0D, ReadGMII16( 0x0D) & ~(1 << 5));

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x14, 0xCAA3);
        WriteGMII16( 0x1C, 0x000A);
        WriteGMII16( 0x18, 0x65D0);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x17, 0xB580);
        WriteGMII16( 0x18, 0xFF54);
        WriteGMII16( 0x19, 0x3954);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x0D, 0x310C);
        WriteGMII16( 0x0E, 0x310C);
        WriteGMII16( 0x0F, 0x311C);
        WriteGMII16( 0x06, 0x0761);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x18, 0xFF55);
        WriteGMII16( 0x19, 0x3955);
        WriteGMII16( 0x18, 0xFF54);
        WriteGMII16( 0x19, 0x3954);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x16, ReadGMII16( 0x16) | (1 << 0));

        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168D_1) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x06, 0x4064);
        WriteGMII16( 0x07, 0x2863);
        WriteGMII16( 0x08, 0x059C);
        WriteGMII16( 0x09, 0x26B4);
        WriteGMII16( 0x0A, 0x6A19);
        WriteGMII16( 0x0B, 0xDCC8);
        WriteGMII16( 0x10, 0xF06D);
        WriteGMII16( 0x14, 0x7F68);
        WriteGMII16( 0x18, 0x7FD9);
        WriteGMII16( 0x1C, 0xF0FF);
        WriteGMII16( 0x1D, 0x3D9C);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x12, 0xF49F);
        WriteGMII16( 0x13, 0x070B);
        WriteGMII16( 0x1A, 0x05AD);
        WriteGMII16( 0x14, 0x94C0);

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x0B) & 0xFF00;
        gphy_val |= 0x10;
        WriteGMII16( 0x0B, gphy_val);
        gphy_val = ReadGMII16( 0x0C) & 0x00FF;
        gphy_val |= 0xA200;
        WriteGMII16( 0x0C, gphy_val);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x06, 0x5561);
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8332);
        WriteGMII16( 0x06, 0x5561);

        if (rtl8168_efuse_read(tp, 0x01) == 0xb1) {
            WriteGMII16( 0x1F, 0x0002);
            WriteGMII16( 0x05, 0x669A);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8330);
            WriteGMII16( 0x06, 0x669A);

            WriteGMII16( 0x1F, 0x0002);
            gphy_val = ReadGMII16( 0x0D);
            if ((gphy_val & 0x00FF) != 0x006C) {
                gphy_val &= 0xFF00;
                WriteGMII16( 0x1F, 0x0002);
                WriteGMII16( 0x0D, gphy_val | 0x0065);
                WriteGMII16( 0x0D, gphy_val | 0x0066);
                WriteGMII16( 0x0D, gphy_val | 0x0067);
                WriteGMII16( 0x0D, gphy_val | 0x0068);
                WriteGMII16( 0x0D, gphy_val | 0x0069);
                WriteGMII16( 0x0D, gphy_val | 0x006A);
                WriteGMII16( 0x0D, gphy_val | 0x006B);
                WriteGMII16( 0x0D, gphy_val | 0x006C);
            }
        } else {
            WriteGMII16( 0x1F, 0x0002);
            WriteGMII16( 0x05, 0x6662);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8330);
            WriteGMII16( 0x06, 0x6662);
        }

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x0D);
        gphy_val |= BIT_9;
        gphy_val |= BIT_8;
        WriteGMII16( 0x0D, gphy_val);
        gphy_val = ReadGMII16( 0x0F);
        gphy_val |= BIT_4;
        WriteGMII16( 0x0F, gphy_val);

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x02);
        gphy_val &= ~BIT_10;
        gphy_val &= ~BIT_9;
        gphy_val |= BIT_8;
        WriteGMII16( 0x02, gphy_val);
        gphy_val = ReadGMII16( 0x03);
        gphy_val &= ~BIT_15;
        gphy_val &= ~BIT_14;
        gphy_val &= ~BIT_13;
        WriteGMII16( 0x03, gphy_val);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x001B);
        if (ReadGMII16( 0x06) == 0xBF00) {
            WriteGMII16( 0x1f, 0x0005);
            WriteGMII16( 0x05, 0xfff6);
            WriteGMII16( 0x06, 0x0080);
            WriteGMII16( 0x05, 0x8000);
            WriteGMII16( 0x06, 0xf8f9);
            WriteGMII16( 0x06, 0xfaef);
            WriteGMII16( 0x06, 0x59ee);
            WriteGMII16( 0x06, 0xf8ea);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0xf8eb);
            WriteGMII16( 0x06, 0x00e0);
            WriteGMII16( 0x06, 0xf87c);
            WriteGMII16( 0x06, 0xe1f8);
            WriteGMII16( 0x06, 0x7d59);
            WriteGMII16( 0x06, 0x0fef);
            WriteGMII16( 0x06, 0x0139);
            WriteGMII16( 0x06, 0x029e);
            WriteGMII16( 0x06, 0x06ef);
            WriteGMII16( 0x06, 0x1039);
            WriteGMII16( 0x06, 0x089f);
            WriteGMII16( 0x06, 0x2aee);
            WriteGMII16( 0x06, 0xf8ea);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0xf8eb);
            WriteGMII16( 0x06, 0x01e0);
            WriteGMII16( 0x06, 0xf87c);
            WriteGMII16( 0x06, 0xe1f8);
            WriteGMII16( 0x06, 0x7d58);
            WriteGMII16( 0x06, 0x409e);
            WriteGMII16( 0x06, 0x0f39);
            WriteGMII16( 0x06, 0x46aa);
            WriteGMII16( 0x06, 0x0bbf);
            WriteGMII16( 0x06, 0x8290);
            WriteGMII16( 0x06, 0xd682);
            WriteGMII16( 0x06, 0x9802);
            WriteGMII16( 0x06, 0x014f);
            WriteGMII16( 0x06, 0xae09);
            WriteGMII16( 0x06, 0xbf82);
            WriteGMII16( 0x06, 0x98d6);
            WriteGMII16( 0x06, 0x82a0);
            WriteGMII16( 0x06, 0x0201);
            WriteGMII16( 0x06, 0x4fef);
            WriteGMII16( 0x06, 0x95fe);
            WriteGMII16( 0x06, 0xfdfc);
            WriteGMII16( 0x06, 0x05f8);
            WriteGMII16( 0x06, 0xf9fa);
            WriteGMII16( 0x06, 0xeef8);
            WriteGMII16( 0x06, 0xea00);
            WriteGMII16( 0x06, 0xeef8);
            WriteGMII16( 0x06, 0xeb00);
            WriteGMII16( 0x06, 0xe2f8);
            WriteGMII16( 0x06, 0x7ce3);
            WriteGMII16( 0x06, 0xf87d);
            WriteGMII16( 0x06, 0xa511);
            WriteGMII16( 0x06, 0x1112);
            WriteGMII16( 0x06, 0xd240);
            WriteGMII16( 0x06, 0xd644);
            WriteGMII16( 0x06, 0x4402);
            WriteGMII16( 0x06, 0x8217);
            WriteGMII16( 0x06, 0xd2a0);
            WriteGMII16( 0x06, 0xd6aa);
            WriteGMII16( 0x06, 0xaa02);
            WriteGMII16( 0x06, 0x8217);
            WriteGMII16( 0x06, 0xae0f);
            WriteGMII16( 0x06, 0xa544);
            WriteGMII16( 0x06, 0x4402);
            WriteGMII16( 0x06, 0xae4d);
            WriteGMII16( 0x06, 0xa5aa);
            WriteGMII16( 0x06, 0xaa02);
            WriteGMII16( 0x06, 0xae47);
            WriteGMII16( 0x06, 0xaf82);
            WriteGMII16( 0x06, 0x13ee);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0x0fee);
            WriteGMII16( 0x06, 0x834c);
            WriteGMII16( 0x06, 0x0fee);
            WriteGMII16( 0x06, 0x834f);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0x8351);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0x834a);
            WriteGMII16( 0x06, 0xffee);
            WriteGMII16( 0x06, 0x834b);
            WriteGMII16( 0x06, 0xffe0);
            WriteGMII16( 0x06, 0x8330);
            WriteGMII16( 0x06, 0xe183);
            WriteGMII16( 0x06, 0x3158);
            WriteGMII16( 0x06, 0xfee4);
            WriteGMII16( 0x06, 0xf88a);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x8be0);
            WriteGMII16( 0x06, 0x8332);
            WriteGMII16( 0x06, 0xe183);
            WriteGMII16( 0x06, 0x3359);
            WriteGMII16( 0x06, 0x0fe2);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0x0c24);
            WriteGMII16( 0x06, 0x5af0);
            WriteGMII16( 0x06, 0x1e12);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x8ce5);
            WriteGMII16( 0x06, 0xf88d);
            WriteGMII16( 0x06, 0xaf82);
            WriteGMII16( 0x06, 0x13e0);
            WriteGMII16( 0x06, 0x834f);
            WriteGMII16( 0x06, 0x10e4);
            WriteGMII16( 0x06, 0x834f);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x009f);
            WriteGMII16( 0x06, 0x0ae0);
            WriteGMII16( 0x06, 0x834f);
            WriteGMII16( 0x06, 0xa010);
            WriteGMII16( 0x06, 0xa5ee);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x01e0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7805);
            WriteGMII16( 0x06, 0x9e9a);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x049e);
            WriteGMII16( 0x06, 0x10e0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7803);
            WriteGMII16( 0x06, 0x9e0f);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x019e);
            WriteGMII16( 0x06, 0x05ae);
            WriteGMII16( 0x06, 0x0caf);
            WriteGMII16( 0x06, 0x81f8);
            WriteGMII16( 0x06, 0xaf81);
            WriteGMII16( 0x06, 0xa3af);
            WriteGMII16( 0x06, 0x81dc);
            WriteGMII16( 0x06, 0xaf82);
            WriteGMII16( 0x06, 0x13ee);
            WriteGMII16( 0x06, 0x8348);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0x8349);
            WriteGMII16( 0x06, 0x00e0);
            WriteGMII16( 0x06, 0x8351);
            WriteGMII16( 0x06, 0x10e4);
            WriteGMII16( 0x06, 0x8351);
            WriteGMII16( 0x06, 0x5801);
            WriteGMII16( 0x06, 0x9fea);
            WriteGMII16( 0x06, 0xd000);
            WriteGMII16( 0x06, 0xd180);
            WriteGMII16( 0x06, 0x1f66);
            WriteGMII16( 0x06, 0xe2f8);
            WriteGMII16( 0x06, 0xeae3);
            WriteGMII16( 0x06, 0xf8eb);
            WriteGMII16( 0x06, 0x5af8);
            WriteGMII16( 0x06, 0x1e20);
            WriteGMII16( 0x06, 0xe6f8);
            WriteGMII16( 0x06, 0xeae5);
            WriteGMII16( 0x06, 0xf8eb);
            WriteGMII16( 0x06, 0xd302);
            WriteGMII16( 0x06, 0xb3fe);
            WriteGMII16( 0x06, 0xe2f8);
            WriteGMII16( 0x06, 0x7cef);
            WriteGMII16( 0x06, 0x325b);
            WriteGMII16( 0x06, 0x80e3);
            WriteGMII16( 0x06, 0xf87d);
            WriteGMII16( 0x06, 0x9e03);
            WriteGMII16( 0x06, 0x7dff);
            WriteGMII16( 0x06, 0xff0d);
            WriteGMII16( 0x06, 0x581c);
            WriteGMII16( 0x06, 0x551a);
            WriteGMII16( 0x06, 0x6511);
            WriteGMII16( 0x06, 0xa190);
            WriteGMII16( 0x06, 0xd3e2);
            WriteGMII16( 0x06, 0x8348);
            WriteGMII16( 0x06, 0xe383);
            WriteGMII16( 0x06, 0x491b);
            WriteGMII16( 0x06, 0x56ab);
            WriteGMII16( 0x06, 0x08ef);
            WriteGMII16( 0x06, 0x56e6);
            WriteGMII16( 0x06, 0x8348);
            WriteGMII16( 0x06, 0xe783);
            WriteGMII16( 0x06, 0x4910);
            WriteGMII16( 0x06, 0xd180);
            WriteGMII16( 0x06, 0x1f66);
            WriteGMII16( 0x06, 0xa004);
            WriteGMII16( 0x06, 0xb9e2);
            WriteGMII16( 0x06, 0x8348);
            WriteGMII16( 0x06, 0xe383);
            WriteGMII16( 0x06, 0x49ef);
            WriteGMII16( 0x06, 0x65e2);
            WriteGMII16( 0x06, 0x834a);
            WriteGMII16( 0x06, 0xe383);
            WriteGMII16( 0x06, 0x4b1b);
            WriteGMII16( 0x06, 0x56aa);
            WriteGMII16( 0x06, 0x0eef);
            WriteGMII16( 0x06, 0x56e6);
            WriteGMII16( 0x06, 0x834a);
            WriteGMII16( 0x06, 0xe783);
            WriteGMII16( 0x06, 0x4be2);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0xe683);
            WriteGMII16( 0x06, 0x4ce0);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0xa000);
            WriteGMII16( 0x06, 0x0caf);
            WriteGMII16( 0x06, 0x81dc);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4d10);
            WriteGMII16( 0x06, 0xe483);
            WriteGMII16( 0x06, 0x4dae);
            WriteGMII16( 0x06, 0x0480);
            WriteGMII16( 0x06, 0xe483);
            WriteGMII16( 0x06, 0x4de0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7803);
            WriteGMII16( 0x06, 0x9e0b);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x049e);
            WriteGMII16( 0x06, 0x04ee);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x02e0);
            WriteGMII16( 0x06, 0x8332);
            WriteGMII16( 0x06, 0xe183);
            WriteGMII16( 0x06, 0x3359);
            WriteGMII16( 0x06, 0x0fe2);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0x0c24);
            WriteGMII16( 0x06, 0x5af0);
            WriteGMII16( 0x06, 0x1e12);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x8ce5);
            WriteGMII16( 0x06, 0xf88d);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x30e1);
            WriteGMII16( 0x06, 0x8331);
            WriteGMII16( 0x06, 0x6801);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x8ae5);
            WriteGMII16( 0x06, 0xf88b);
            WriteGMII16( 0x06, 0xae37);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4e03);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4ce1);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0x1b01);
            WriteGMII16( 0x06, 0x9e04);
            WriteGMII16( 0x06, 0xaaa1);
            WriteGMII16( 0x06, 0xaea8);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4e04);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4f00);
            WriteGMII16( 0x06, 0xaeab);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4f78);
            WriteGMII16( 0x06, 0x039f);
            WriteGMII16( 0x06, 0x14ee);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x05d2);
            WriteGMII16( 0x06, 0x40d6);
            WriteGMII16( 0x06, 0x5554);
            WriteGMII16( 0x06, 0x0282);
            WriteGMII16( 0x06, 0x17d2);
            WriteGMII16( 0x06, 0xa0d6);
            WriteGMII16( 0x06, 0xba00);
            WriteGMII16( 0x06, 0x0282);
            WriteGMII16( 0x06, 0x17fe);
            WriteGMII16( 0x06, 0xfdfc);
            WriteGMII16( 0x06, 0x05f8);
            WriteGMII16( 0x06, 0xe0f8);
            WriteGMII16( 0x06, 0x60e1);
            WriteGMII16( 0x06, 0xf861);
            WriteGMII16( 0x06, 0x6802);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x60e5);
            WriteGMII16( 0x06, 0xf861);
            WriteGMII16( 0x06, 0xe0f8);
            WriteGMII16( 0x06, 0x48e1);
            WriteGMII16( 0x06, 0xf849);
            WriteGMII16( 0x06, 0x580f);
            WriteGMII16( 0x06, 0x1e02);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x48e5);
            WriteGMII16( 0x06, 0xf849);
            WriteGMII16( 0x06, 0xd000);
            WriteGMII16( 0x06, 0x0282);
            WriteGMII16( 0x06, 0x5bbf);
            WriteGMII16( 0x06, 0x8350);
            WriteGMII16( 0x06, 0xef46);
            WriteGMII16( 0x06, 0xdc19);
            WriteGMII16( 0x06, 0xddd0);
            WriteGMII16( 0x06, 0x0102);
            WriteGMII16( 0x06, 0x825b);
            WriteGMII16( 0x06, 0x0282);
            WriteGMII16( 0x06, 0x77e0);
            WriteGMII16( 0x06, 0xf860);
            WriteGMII16( 0x06, 0xe1f8);
            WriteGMII16( 0x06, 0x6158);
            WriteGMII16( 0x06, 0xfde4);
            WriteGMII16( 0x06, 0xf860);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x61fc);
            WriteGMII16( 0x06, 0x04f9);
            WriteGMII16( 0x06, 0xfafb);
            WriteGMII16( 0x06, 0xc6bf);
            WriteGMII16( 0x06, 0xf840);
            WriteGMII16( 0x06, 0xbe83);
            WriteGMII16( 0x06, 0x50a0);
            WriteGMII16( 0x06, 0x0101);
            WriteGMII16( 0x06, 0x071b);
            WriteGMII16( 0x06, 0x89cf);
            WriteGMII16( 0x06, 0xd208);
            WriteGMII16( 0x06, 0xebdb);
            WriteGMII16( 0x06, 0x19b2);
            WriteGMII16( 0x06, 0xfbff);
            WriteGMII16( 0x06, 0xfefd);
            WriteGMII16( 0x06, 0x04f8);
            WriteGMII16( 0x06, 0xe0f8);
            WriteGMII16( 0x06, 0x48e1);
            WriteGMII16( 0x06, 0xf849);
            WriteGMII16( 0x06, 0x6808);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x48e5);
            WriteGMII16( 0x06, 0xf849);
            WriteGMII16( 0x06, 0x58f7);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x48e5);
            WriteGMII16( 0x06, 0xf849);
            WriteGMII16( 0x06, 0xfc04);
            WriteGMII16( 0x06, 0x4d20);
            WriteGMII16( 0x06, 0x0002);
            WriteGMII16( 0x06, 0x4e22);
            WriteGMII16( 0x06, 0x0002);
            WriteGMII16( 0x06, 0x4ddf);
            WriteGMII16( 0x06, 0xff01);
            WriteGMII16( 0x06, 0x4edd);
            WriteGMII16( 0x06, 0xff01);
            WriteGMII16( 0x06, 0xf8fa);
            WriteGMII16( 0x06, 0xfbef);
            WriteGMII16( 0x06, 0x79bf);
            WriteGMII16( 0x06, 0xf822);
            WriteGMII16( 0x06, 0xd819);
            WriteGMII16( 0x06, 0xd958);
            WriteGMII16( 0x06, 0x849f);
            WriteGMII16( 0x06, 0x09bf);
            WriteGMII16( 0x06, 0x82be);
            WriteGMII16( 0x06, 0xd682);
            WriteGMII16( 0x06, 0xc602);
            WriteGMII16( 0x06, 0x014f);
            WriteGMII16( 0x06, 0xef97);
            WriteGMII16( 0x06, 0xfffe);
            WriteGMII16( 0x06, 0xfc05);
            WriteGMII16( 0x06, 0x17ff);
            WriteGMII16( 0x06, 0xfe01);
            WriteGMII16( 0x06, 0x1700);
            WriteGMII16( 0x06, 0x0102);
            WriteGMII16( 0x05, 0x83d8);
            WriteGMII16( 0x06, 0x8051);
            WriteGMII16( 0x05, 0x83d6);
            WriteGMII16( 0x06, 0x82a0);
            WriteGMII16( 0x05, 0x83d4);
            WriteGMII16( 0x06, 0x8000);
            WriteGMII16( 0x02, 0x2010);
            WriteGMII16( 0x03, 0xdc00);
            WriteGMII16( 0x1f, 0x0000);
            WriteGMII16( 0x0b, 0x0600);
            WriteGMII16( 0x1f, 0x0005);
            WriteGMII16( 0x05, 0xfff6);
            WriteGMII16( 0x06, 0x00fc);
            WriteGMII16( 0x1f, 0x0000);
        }

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0xF880);
        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168D_2) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x06, 0x4064);
        WriteGMII16( 0x07, 0x2863);
        WriteGMII16( 0x08, 0x059C);
        WriteGMII16( 0x09, 0x26B4);
        WriteGMII16( 0x0A, 0x6A19);
        WriteGMII16( 0x0B, 0xDCC8);
        WriteGMII16( 0x10, 0xF06D);
        WriteGMII16( 0x14, 0x7F68);
        WriteGMII16( 0x18, 0x7FD9);
        WriteGMII16( 0x1C, 0xF0FF);
        WriteGMII16( 0x1D, 0x3D9C);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x12, 0xF49F);
        WriteGMII16( 0x13, 0x070B);
        WriteGMII16( 0x1A, 0x05AD);
        WriteGMII16( 0x14, 0x94C0);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x06, 0x5561);
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8332);
        WriteGMII16( 0x06, 0x5561);

        if (rtl8168_efuse_read(tp, 0x01) == 0xb1) {
            WriteGMII16( 0x1F, 0x0002);
            WriteGMII16( 0x05, 0x669A);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8330);
            WriteGMII16( 0x06, 0x669A);

            WriteGMII16( 0x1F, 0x0002);
            gphy_val = ReadGMII16( 0x0D);
            if ((gphy_val & 0x00FF) != 0x006C) {
                gphy_val &= 0xFF00;
                WriteGMII16( 0x1F, 0x0002);
                WriteGMII16( 0x0D, gphy_val | 0x0065);
                WriteGMII16( 0x0D, gphy_val | 0x0066);
                WriteGMII16( 0x0D, gphy_val | 0x0067);
                WriteGMII16( 0x0D, gphy_val | 0x0068);
                WriteGMII16( 0x0D, gphy_val | 0x0069);
                WriteGMII16( 0x0D, gphy_val | 0x006A);
                WriteGMII16( 0x0D, gphy_val | 0x006B);
                WriteGMII16( 0x0D, gphy_val | 0x006C);
            }
        } else {
            WriteGMII16( 0x1F, 0x0002);
            WriteGMII16( 0x05, 0x2642);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8330);
            WriteGMII16( 0x06, 0x2642);
        }

        if (rtl8168_efuse_read(tp, 0x30) == 0x98) {
            WriteGMII16( 0x1F, 0x0000);
            WriteGMII16( 0x11, ReadGMII16( 0x11) & ~BIT_1);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x01, ReadGMII16( 0x01) | BIT_9);
        } else if (rtl8168_efuse_read(tp, 0x30) == 0x90) {
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x01, ReadGMII16( 0x01) & ~BIT_9);
            WriteGMII16( 0x1F, 0x0000);
            WriteGMII16( 0x16, 0x5101);
        }

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x02);
        gphy_val &= ~BIT_10;
        gphy_val &= ~BIT_9;
        gphy_val |= BIT_8;
        WriteGMII16( 0x02, gphy_val);
        gphy_val = ReadGMII16( 0x03);
        gphy_val &= ~BIT_15;
        gphy_val &= ~BIT_14;
        gphy_val &= ~BIT_13;
        WriteGMII16( 0x03, gphy_val);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x0F);
        gphy_val |= BIT_4;
        gphy_val |= BIT_2;
        gphy_val |= BIT_1;
        gphy_val |= BIT_0;
        WriteGMII16( 0x0F, gphy_val);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x001B);
        if (ReadGMII16( 0x06) == 0xB300) {
            WriteGMII16( 0x1f, 0x0005);
            WriteGMII16( 0x05, 0xfff6);
            WriteGMII16( 0x06, 0x0080);
            WriteGMII16( 0x05, 0x8000);
            WriteGMII16( 0x06, 0xf8f9);
            WriteGMII16( 0x06, 0xfaee);
            WriteGMII16( 0x06, 0xf8ea);
            WriteGMII16( 0x06, 0x00ee);
            WriteGMII16( 0x06, 0xf8eb);
            WriteGMII16( 0x06, 0x00e2);
            WriteGMII16( 0x06, 0xf87c);
            WriteGMII16( 0x06, 0xe3f8);
            WriteGMII16( 0x06, 0x7da5);
            WriteGMII16( 0x06, 0x1111);
            WriteGMII16( 0x06, 0x12d2);
            WriteGMII16( 0x06, 0x40d6);
            WriteGMII16( 0x06, 0x4444);
            WriteGMII16( 0x06, 0x0281);
            WriteGMII16( 0x06, 0xc6d2);
            WriteGMII16( 0x06, 0xa0d6);
            WriteGMII16( 0x06, 0xaaaa);
            WriteGMII16( 0x06, 0x0281);
            WriteGMII16( 0x06, 0xc6ae);
            WriteGMII16( 0x06, 0x0fa5);
            WriteGMII16( 0x06, 0x4444);
            WriteGMII16( 0x06, 0x02ae);
            WriteGMII16( 0x06, 0x4da5);
            WriteGMII16( 0x06, 0xaaaa);
            WriteGMII16( 0x06, 0x02ae);
            WriteGMII16( 0x06, 0x47af);
            WriteGMII16( 0x06, 0x81c2);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4e00);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4d0f);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4c0f);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4f00);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x5100);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4aff);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4bff);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x30e1);
            WriteGMII16( 0x06, 0x8331);
            WriteGMII16( 0x06, 0x58fe);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x8ae5);
            WriteGMII16( 0x06, 0xf88b);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x32e1);
            WriteGMII16( 0x06, 0x8333);
            WriteGMII16( 0x06, 0x590f);
            WriteGMII16( 0x06, 0xe283);
            WriteGMII16( 0x06, 0x4d0c);
            WriteGMII16( 0x06, 0x245a);
            WriteGMII16( 0x06, 0xf01e);
            WriteGMII16( 0x06, 0x12e4);
            WriteGMII16( 0x06, 0xf88c);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x8daf);
            WriteGMII16( 0x06, 0x81c2);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4f10);
            WriteGMII16( 0x06, 0xe483);
            WriteGMII16( 0x06, 0x4fe0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7800);
            WriteGMII16( 0x06, 0x9f0a);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4fa0);
            WriteGMII16( 0x06, 0x10a5);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4e01);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x059e);
            WriteGMII16( 0x06, 0x9ae0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7804);
            WriteGMII16( 0x06, 0x9e10);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x039e);
            WriteGMII16( 0x06, 0x0fe0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7801);
            WriteGMII16( 0x06, 0x9e05);
            WriteGMII16( 0x06, 0xae0c);
            WriteGMII16( 0x06, 0xaf81);
            WriteGMII16( 0x06, 0xa7af);
            WriteGMII16( 0x06, 0x8152);
            WriteGMII16( 0x06, 0xaf81);
            WriteGMII16( 0x06, 0x8baf);
            WriteGMII16( 0x06, 0x81c2);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4800);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4900);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x5110);
            WriteGMII16( 0x06, 0xe483);
            WriteGMII16( 0x06, 0x5158);
            WriteGMII16( 0x06, 0x019f);
            WriteGMII16( 0x06, 0xead0);
            WriteGMII16( 0x06, 0x00d1);
            WriteGMII16( 0x06, 0x801f);
            WriteGMII16( 0x06, 0x66e2);
            WriteGMII16( 0x06, 0xf8ea);
            WriteGMII16( 0x06, 0xe3f8);
            WriteGMII16( 0x06, 0xeb5a);
            WriteGMII16( 0x06, 0xf81e);
            WriteGMII16( 0x06, 0x20e6);
            WriteGMII16( 0x06, 0xf8ea);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0xebd3);
            WriteGMII16( 0x06, 0x02b3);
            WriteGMII16( 0x06, 0xfee2);
            WriteGMII16( 0x06, 0xf87c);
            WriteGMII16( 0x06, 0xef32);
            WriteGMII16( 0x06, 0x5b80);
            WriteGMII16( 0x06, 0xe3f8);
            WriteGMII16( 0x06, 0x7d9e);
            WriteGMII16( 0x06, 0x037d);
            WriteGMII16( 0x06, 0xffff);
            WriteGMII16( 0x06, 0x0d58);
            WriteGMII16( 0x06, 0x1c55);
            WriteGMII16( 0x06, 0x1a65);
            WriteGMII16( 0x06, 0x11a1);
            WriteGMII16( 0x06, 0x90d3);
            WriteGMII16( 0x06, 0xe283);
            WriteGMII16( 0x06, 0x48e3);
            WriteGMII16( 0x06, 0x8349);
            WriteGMII16( 0x06, 0x1b56);
            WriteGMII16( 0x06, 0xab08);
            WriteGMII16( 0x06, 0xef56);
            WriteGMII16( 0x06, 0xe683);
            WriteGMII16( 0x06, 0x48e7);
            WriteGMII16( 0x06, 0x8349);
            WriteGMII16( 0x06, 0x10d1);
            WriteGMII16( 0x06, 0x801f);
            WriteGMII16( 0x06, 0x66a0);
            WriteGMII16( 0x06, 0x04b9);
            WriteGMII16( 0x06, 0xe283);
            WriteGMII16( 0x06, 0x48e3);
            WriteGMII16( 0x06, 0x8349);
            WriteGMII16( 0x06, 0xef65);
            WriteGMII16( 0x06, 0xe283);
            WriteGMII16( 0x06, 0x4ae3);
            WriteGMII16( 0x06, 0x834b);
            WriteGMII16( 0x06, 0x1b56);
            WriteGMII16( 0x06, 0xaa0e);
            WriteGMII16( 0x06, 0xef56);
            WriteGMII16( 0x06, 0xe683);
            WriteGMII16( 0x06, 0x4ae7);
            WriteGMII16( 0x06, 0x834b);
            WriteGMII16( 0x06, 0xe283);
            WriteGMII16( 0x06, 0x4de6);
            WriteGMII16( 0x06, 0x834c);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4da0);
            WriteGMII16( 0x06, 0x000c);
            WriteGMII16( 0x06, 0xaf81);
            WriteGMII16( 0x06, 0x8be0);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0x10e4);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0xae04);
            WriteGMII16( 0x06, 0x80e4);
            WriteGMII16( 0x06, 0x834d);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x4e78);
            WriteGMII16( 0x06, 0x039e);
            WriteGMII16( 0x06, 0x0be0);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x7804);
            WriteGMII16( 0x06, 0x9e04);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4e02);
            WriteGMII16( 0x06, 0xe083);
            WriteGMII16( 0x06, 0x32e1);
            WriteGMII16( 0x06, 0x8333);
            WriteGMII16( 0x06, 0x590f);
            WriteGMII16( 0x06, 0xe283);
            WriteGMII16( 0x06, 0x4d0c);
            WriteGMII16( 0x06, 0x245a);
            WriteGMII16( 0x06, 0xf01e);
            WriteGMII16( 0x06, 0x12e4);
            WriteGMII16( 0x06, 0xf88c);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x8de0);
            WriteGMII16( 0x06, 0x8330);
            WriteGMII16( 0x06, 0xe183);
            WriteGMII16( 0x06, 0x3168);
            WriteGMII16( 0x06, 0x01e4);
            WriteGMII16( 0x06, 0xf88a);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x8bae);
            WriteGMII16( 0x06, 0x37ee);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x03e0);
            WriteGMII16( 0x06, 0x834c);
            WriteGMII16( 0x06, 0xe183);
            WriteGMII16( 0x06, 0x4d1b);
            WriteGMII16( 0x06, 0x019e);
            WriteGMII16( 0x06, 0x04aa);
            WriteGMII16( 0x06, 0xa1ae);
            WriteGMII16( 0x06, 0xa8ee);
            WriteGMII16( 0x06, 0x834e);
            WriteGMII16( 0x06, 0x04ee);
            WriteGMII16( 0x06, 0x834f);
            WriteGMII16( 0x06, 0x00ae);
            WriteGMII16( 0x06, 0xabe0);
            WriteGMII16( 0x06, 0x834f);
            WriteGMII16( 0x06, 0x7803);
            WriteGMII16( 0x06, 0x9f14);
            WriteGMII16( 0x06, 0xee83);
            WriteGMII16( 0x06, 0x4e05);
            WriteGMII16( 0x06, 0xd240);
            WriteGMII16( 0x06, 0xd655);
            WriteGMII16( 0x06, 0x5402);
            WriteGMII16( 0x06, 0x81c6);
            WriteGMII16( 0x06, 0xd2a0);
            WriteGMII16( 0x06, 0xd6ba);
            WriteGMII16( 0x06, 0x0002);
            WriteGMII16( 0x06, 0x81c6);
            WriteGMII16( 0x06, 0xfefd);
            WriteGMII16( 0x06, 0xfc05);
            WriteGMII16( 0x06, 0xf8e0);
            WriteGMII16( 0x06, 0xf860);
            WriteGMII16( 0x06, 0xe1f8);
            WriteGMII16( 0x06, 0x6168);
            WriteGMII16( 0x06, 0x02e4);
            WriteGMII16( 0x06, 0xf860);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x61e0);
            WriteGMII16( 0x06, 0xf848);
            WriteGMII16( 0x06, 0xe1f8);
            WriteGMII16( 0x06, 0x4958);
            WriteGMII16( 0x06, 0x0f1e);
            WriteGMII16( 0x06, 0x02e4);
            WriteGMII16( 0x06, 0xf848);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x49d0);
            WriteGMII16( 0x06, 0x0002);
            WriteGMII16( 0x06, 0x820a);
            WriteGMII16( 0x06, 0xbf83);
            WriteGMII16( 0x06, 0x50ef);
            WriteGMII16( 0x06, 0x46dc);
            WriteGMII16( 0x06, 0x19dd);
            WriteGMII16( 0x06, 0xd001);
            WriteGMII16( 0x06, 0x0282);
            WriteGMII16( 0x06, 0x0a02);
            WriteGMII16( 0x06, 0x8226);
            WriteGMII16( 0x06, 0xe0f8);
            WriteGMII16( 0x06, 0x60e1);
            WriteGMII16( 0x06, 0xf861);
            WriteGMII16( 0x06, 0x58fd);
            WriteGMII16( 0x06, 0xe4f8);
            WriteGMII16( 0x06, 0x60e5);
            WriteGMII16( 0x06, 0xf861);
            WriteGMII16( 0x06, 0xfc04);
            WriteGMII16( 0x06, 0xf9fa);
            WriteGMII16( 0x06, 0xfbc6);
            WriteGMII16( 0x06, 0xbff8);
            WriteGMII16( 0x06, 0x40be);
            WriteGMII16( 0x06, 0x8350);
            WriteGMII16( 0x06, 0xa001);
            WriteGMII16( 0x06, 0x0107);
            WriteGMII16( 0x06, 0x1b89);
            WriteGMII16( 0x06, 0xcfd2);
            WriteGMII16( 0x06, 0x08eb);
            WriteGMII16( 0x06, 0xdb19);
            WriteGMII16( 0x06, 0xb2fb);
            WriteGMII16( 0x06, 0xfffe);
            WriteGMII16( 0x06, 0xfd04);
            WriteGMII16( 0x06, 0xf8e0);
            WriteGMII16( 0x06, 0xf848);
            WriteGMII16( 0x06, 0xe1f8);
            WriteGMII16( 0x06, 0x4968);
            WriteGMII16( 0x06, 0x08e4);
            WriteGMII16( 0x06, 0xf848);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x4958);
            WriteGMII16( 0x06, 0xf7e4);
            WriteGMII16( 0x06, 0xf848);
            WriteGMII16( 0x06, 0xe5f8);
            WriteGMII16( 0x06, 0x49fc);
            WriteGMII16( 0x06, 0x044d);
            WriteGMII16( 0x06, 0x2000);
            WriteGMII16( 0x06, 0x024e);
            WriteGMII16( 0x06, 0x2200);
            WriteGMII16( 0x06, 0x024d);
            WriteGMII16( 0x06, 0xdfff);
            WriteGMII16( 0x06, 0x014e);
            WriteGMII16( 0x06, 0xddff);
            WriteGMII16( 0x06, 0x01f8);
            WriteGMII16( 0x06, 0xfafb);
            WriteGMII16( 0x06, 0xef79);
            WriteGMII16( 0x06, 0xbff8);
            WriteGMII16( 0x06, 0x22d8);
            WriteGMII16( 0x06, 0x19d9);
            WriteGMII16( 0x06, 0x5884);
            WriteGMII16( 0x06, 0x9f09);
            WriteGMII16( 0x06, 0xbf82);
            WriteGMII16( 0x06, 0x6dd6);
            WriteGMII16( 0x06, 0x8275);
            WriteGMII16( 0x06, 0x0201);
            WriteGMII16( 0x06, 0x4fef);
            WriteGMII16( 0x06, 0x97ff);
            WriteGMII16( 0x06, 0xfefc);
            WriteGMII16( 0x06, 0x0517);
            WriteGMII16( 0x06, 0xfffe);
            WriteGMII16( 0x06, 0x0117);
            WriteGMII16( 0x06, 0x0001);
            WriteGMII16( 0x06, 0x0200);
            WriteGMII16( 0x05, 0x83d8);
            WriteGMII16( 0x06, 0x8000);
            WriteGMII16( 0x05, 0x83d6);
            WriteGMII16( 0x06, 0x824f);
            WriteGMII16( 0x02, 0x2010);
            WriteGMII16( 0x03, 0xdc00);
            WriteGMII16( 0x1f, 0x0000);
            WriteGMII16( 0x0b, 0x0600);
            WriteGMII16( 0x1f, 0x0005);
            WriteGMII16( 0x05, 0xfff6);
            WriteGMII16( 0x06, 0x00fc);
            WriteGMII16( 0x1f, 0x0000);
        }

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x0D, 0xF880);
        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == MCFG_8168DP_1) {
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x10, 0x0008);
        WriteGMII16( 0x0D, 0x006C);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0xA4D8);
        WriteGMII16( 0x09, 0x281C);
        WriteGMII16( 0x07, 0x2883);
        WriteGMII16( 0x0A, 0x6B35);
        WriteGMII16( 0x1D, 0x3DA4);
        WriteGMII16( 0x1C, 0xEFFD);
        WriteGMII16( 0x14, 0x7F52);
        WriteGMII16( 0x18, 0x7FC6);
        WriteGMII16( 0x08, 0x0601);
        WriteGMII16( 0x06, 0x4063);
        WriteGMII16( 0x10, 0xF074);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x13, 0x0789);
        WriteGMII16( 0x12, 0xF4BD);
        WriteGMII16( 0x1A, 0x04FD);
        WriteGMII16( 0x14, 0x84B0);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x00, 0x9200);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x01, 0x0340);
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x04, 0x4000);
        WriteGMII16( 0x03, 0x1D21);
        WriteGMII16( 0x02, 0x0C32);
        WriteGMII16( 0x01, 0x0200);
        WriteGMII16( 0x00, 0x5554);
        WriteGMII16( 0x04, 0x4800);
        WriteGMII16( 0x04, 0x4000);
        WriteGMII16( 0x04, 0xF000);
        WriteGMII16( 0x03, 0xDF01);
        WriteGMII16( 0x02, 0xDF20);
        WriteGMII16( 0x01, 0x101A);
        WriteGMII16( 0x00, 0xA0FF);
        WriteGMII16( 0x04, 0xF800);
        WriteGMII16( 0x04, 0xF000);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0023);
        WriteGMII16( 0x16, 0x0000);
        WriteGMII16( 0x1F, 0x0000);

        gphy_val = ReadGMII16( 0x0D);
        gphy_val |= BIT_5;
        WriteGMII16( 0x0D, gphy_val);
    } 
    else if (mcfg == MCFG_8168DP_2 || mcfg == MCFG_8168DP_3) {
        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x17, 0x0CC0);

        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x002D);
        WriteGMII16( 0x18, 0x0040);

        WriteGMII16( 0x1F, 0x0000);
        gphy_val = ReadGMII16( 0x0D);
        gphy_val |= BIT_5;
        WriteGMII16( 0x0D, gphy_val);

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x0C);
        gphy_val |= BIT_10;
        WriteGMII16( 0x0C, gphy_val);
    }
    else if (mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2) {
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0023);
        gphy_val = ReadGMII16( 0x17)|0x02;
        WriteGMII16( 0x17, gphy_val);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8b80);
        WriteGMII16( 0x06, 0xc896);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0x6C20);
        WriteGMII16( 0x07, 0x2872);
        WriteGMII16( 0x1C, 0xEFFF);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x14, 0x6420);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0002);
        gphy_val = ReadGMII16( 0x08) & 0x00FF;
        WriteGMII16( 0x08, gphy_val | 0x8000);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        WriteGMII16( 0x18, gphy_val | 0x0050);
        WriteGMII16( 0x1F, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        WriteGMII16( 0x14, gphy_val | 0x8000);

        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x00, 0x080B);
        WriteGMII16( 0x0B, 0x09D7);
        WriteGMII16( 0x1f, 0x0000);
        WriteGMII16( 0x15, 0x1006);

        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x002F);
        WriteGMII16( 0x15, 0x1919);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B86);
        gphy_val = ReadGMII16( 0x06);
        WriteGMII16( 0x06, gphy_val | 0x0001);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x00AC);
        WriteGMII16( 0x18, 0x0006);
        WriteGMII16( 0x1F, 0x0000);

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
    else if (mcfg == MCFG_8168E_VL_1) {
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B80);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_2 | BIT_1;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        gphy_val |= BIT_4;
        WriteGMII16( 0x18, gphy_val);
        WriteGMII16( 0x1f, 0x0002);
        WriteGMII16( 0x1f, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        gphy_val |= BIT_15;
        WriteGMII16( 0x14, gphy_val);

        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x15, 0x1006);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B86);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0001);
        WriteGMII16( 0x0B, 0x6C14);
        WriteGMII16( 0x14, 0x7F3D);
        WriteGMII16( 0x1C, 0xFAFE);
        WriteGMII16( 0x08, 0x07C5);
        WriteGMII16( 0x10, 0xF090);
        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x14, 0x641A);
        WriteGMII16( 0x1A, 0x0606);
        WriteGMII16( 0x12, 0xF480);
        WriteGMII16( 0x13, 0x0747);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0078);
        WriteGMII16( 0x15, 0xA408);
        WriteGMII16( 0x17, 0x5100);
        WriteGMII16( 0x19, 0x0008);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x0D, 0x0207);
        WriteGMII16( 0x02, 0x5FD0);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x00A1);
        gphy_val = ReadGMII16( 0x1A);
        gphy_val &= ~BIT_2;
        WriteGMII16( 0x1A, gphy_val);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x002D);
        gphy_val = ReadGMII16( 0x16);
        gphy_val |= BIT_5;
        WriteGMII16( 0x16, gphy_val);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x00AC);
        WriteGMII16( 0x18, 0x0006);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x09, 0xA20F);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B5B);
        WriteGMII16( 0x06, 0x9222);
        WriteGMII16( 0x05, 0x8B6D);
        WriteGMII16( 0x06, 0x8000);
        WriteGMII16( 0x05, 0x8B76);
        WriteGMII16( 0x06, 0x8000);
        WriteGMII16( 0x1F, 0x0000);

        if (pdev->subsystem_vendor == 0x1043 &&
            pdev->subsystem_device == 0x13F7) {

            static const u16 evl_phy_value[] = {
                0x8B56, 0x8B5F, 0x8B68, 0x8B71,
                0x8B7A, 0x8A7B, 0x8A7E, 0x8A81,
                0x8A84, 0x8A87
            };

            WriteGMII16( 0x1F, 0x0005);
            for (i = 0; i < ARRAY_SIZE(evl_phy_value); i++) {
                WriteGMII16( 0x05, evl_phy_value[i]);
                gphy_val = (0xAA << 8) | (ReadGMII16( 0x06) & 0xFF);
                WriteGMII16( 0x06, gphy_val);
            }
            WriteGMII16( 0x1F, 0x0007);
            WriteGMII16( 0x1E, 0x0078);
            WriteGMII16( 0x17, 0x51AA);
            WriteGMII16( 0x1F, 0x0000);
        }

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8B54);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8B5D);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8A7C);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A7F);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_8);
        WriteGMII16( 0x05, 0x8A82);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A88);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        gphy_val = ReadGMII16( 0x06) | BIT_15;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
    }
    else if (mcfg == MCFG_8168E_VL_2) {
        if (pdev->subsystem_vendor == 0x144d &&
            pdev->subsystem_device == 0xc0a6) {
            WriteGMII16( 0x1F, 0x0001);
            WriteGMII16( 0x0e, 0x6b7f);
            WriteGMII16( 0x1f, 0x0000);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8B86);
            gphy_val = ReadGMII16( 0x06);
            gphy_val |= BIT_4;
            WriteGMII16( 0x06, gphy_val);
            WriteGMII16( 0x1f, 0x0000);
        } else {
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8B80);
            gphy_val = ReadGMII16( 0x06);
            gphy_val |= BIT_2 | BIT_1;
            WriteGMII16( 0x06, gphy_val);
            WriteGMII16( 0x1f, 0x0000);

            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8B86);
            gphy_val = ReadGMII16( 0x06);
            gphy_val &= ~BIT_4;
            WriteGMII16( 0x06, gphy_val);
            WriteGMII16( 0x1f, 0x0000);
        }

        WriteGMII16( 0x1f, 0x0004);
        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        gphy_val |= BIT_4;
        WriteGMII16( 0x18, gphy_val);
        WriteGMII16( 0x1f, 0x0002);
        WriteGMII16( 0x1f, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        gphy_val |= BIT_15;
        WriteGMII16( 0x14, gphy_val);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B86);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0004);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x00AC);
        WriteGMII16( 0x18, 0x0006);
        WriteGMII16( 0x1F, 0x0002);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x09, 0xA20F);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        gphy_val = ReadGMII16( 0x06) | BIT_14 | BIT_15;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B5B);
        WriteGMII16( 0x06, 0x9222);
        WriteGMII16( 0x05, 0x8B6D);
        WriteGMII16( 0x06, 0x8000);
        WriteGMII16( 0x05, 0x8B76);
        WriteGMII16( 0x06, 0x8000);
        WriteGMII16( 0x1F, 0x0000);

        if (pdev->subsystem_vendor == 0x1043 &&
            pdev->subsystem_device == 0x13F7) {

            static const u16 evl_phy_value[] = {
                0x8B56, 0x8B5F, 0x8B68, 0x8B71,
                0x8B7A, 0x8A7B, 0x8A7E, 0x8A81,
                0x8A84, 0x8A87
            };

            WriteGMII16( 0x1F, 0x0005);
            for (i = 0; i < ARRAY_SIZE(evl_phy_value); i++) {
                WriteGMII16( 0x05, evl_phy_value[i]);
                gphy_val = (0xAA << 8) | (ReadGMII16( 0x06) & 0xFF);
                WriteGMII16( 0x06, gphy_val);
            }
            WriteGMII16( 0x1F, 0x0007);
            WriteGMII16( 0x1E, 0x0078);
            WriteGMII16( 0x17, 0x51AA);
            WriteGMII16( 0x1F, 0x0000);
        }

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8B54);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8B5D);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8A7C);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A7F);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_8);
        WriteGMII16( 0x05, 0x8A82);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A88);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x1f, 0x0000);

        if (aspm) {
            WriteGMII16( 0x1f, 0x0000);
            gphy_val = ReadGMII16( 0x15);
            gphy_val |= BIT_12;
            WriteGMII16( 0x15, gphy_val);
        }
    }
    else if (mcfg == MCFG_8168F_1) {
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8b80);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_2 | BIT_1;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        gphy_val |= BIT_4;
        WriteGMII16( 0x18, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        gphy_val |= BIT_15;
        WriteGMII16( 0x14, gphy_val);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B86);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_14;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x09, 0xA20F);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B55);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8B5E);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8B67);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8B70);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0078);
        WriteGMII16( 0x17, 0x0000);
        WriteGMII16( 0x19, 0x00FB);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B79);
        WriteGMII16( 0x06, 0xAA00);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0003);
        WriteGMII16( 0x01, 0x328A);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8B54);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8B5D);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8A7C);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A7F);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_8);
        WriteGMII16( 0x05, 0x8A82);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A88);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8b85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_15);
        WriteGMII16( 0x1f, 0x0000);

        if (aspm) {
            WriteGMII16( 0x1f, 0x0000);
            gphy_val = ReadGMII16( 0x15);
            gphy_val |= BIT_12;
            WriteGMII16( 0x15, gphy_val);
        }
    }
    else if (mcfg == MCFG_8168F_2) {
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8b80);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_2 | BIT_1;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        gphy_val |= BIT_4;
        WriteGMII16( 0x18, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        gphy_val |= BIT_15;
        WriteGMII16( 0x14, gphy_val);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B86);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8B54);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8B5D);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8A7C);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A7F);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_8);
        WriteGMII16( 0x05, 0x8A82);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A88);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8b85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_15);
        WriteGMII16( 0x1f, 0x0000);

        if (aspm) {
            WriteGMII16( 0x1f, 0x0000);
            gphy_val = ReadGMII16( 0x15);
            gphy_val |= BIT_12;
            WriteGMII16( 0x15, gphy_val);
        }
    }
    else if (mcfg == MCFG_8411_1) {
        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8b80);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_2 | BIT_1;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0007);
        WriteGMII16( 0x1e, 0x002D);
        gphy_val = ReadGMII16( 0x18);
        gphy_val |= BIT_4;
        WriteGMII16( 0x18, gphy_val);
        WriteGMII16( 0x1f, 0x0000);
        gphy_val = ReadGMII16( 0x14);
        gphy_val |= BIT_15;
        WriteGMII16( 0x14, gphy_val);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B86);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_0;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B85);
        gphy_val = ReadGMII16( 0x06);
        gphy_val |= BIT_14;
        WriteGMII16( 0x06, gphy_val);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1F, 0x0003);
        WriteGMII16( 0x09, 0xA20F);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B55);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8B5E);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8B67);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x05, 0x8B70);
        WriteGMII16( 0x06, 0x0000);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( 0x1F, 0x0007);
        WriteGMII16( 0x1E, 0x0078);
        WriteGMII16( 0x17, 0x0000);
        WriteGMII16( 0x19, 0x00FB);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1F, 0x0005);
        WriteGMII16( 0x05, 0x8B79);
        WriteGMII16( 0x06, 0xAA00);
        WriteGMII16( 0x1F, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8B54);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8B5D);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_11);
        WriteGMII16( 0x05, 0x8A7C);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A7F);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_8);
        WriteGMII16( 0x05, 0x8A82);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x05, 0x8A88);
        WriteGMII16( 0x06, ReadGMII16( 0x06) & ~BIT_8);
        WriteGMII16( 0x1f, 0x0000);

        WriteGMII16( 0x1f, 0x0005);
        WriteGMII16( 0x05, 0x8b85);
        WriteGMII16( 0x06, ReadGMII16( 0x06) | BIT_15);
        WriteGMII16( 0x1f, 0x0000);

        if (aspm) {
            WriteGMII16( 0x1f, 0x0000);
            gphy_val = ReadGMII16( 0x15);
            gphy_val |= BIT_12;
            WriteGMII16( 0x15, gphy_val);
        }
    }
    else if (mcfg == CFG_METHOD_21) {
        WriteGMII16( 0x1F, 0x0A46);
        gphy_val = ReadGMII16( 0x10);
        WriteGMII16( 0x1F, 0x0BCC);
        if (gphy_val & BIT_8)
            gphy_val = ReadGMII16( 0x12) & ~BIT_15;
        else
            gphy_val = ReadGMII16( 0x12) | BIT_15;
        WriteGMII16( 0x1F, 0x0A46);
        gphy_val = ReadGMII16( 0x13);
        WriteGMII16( 0x1F, 0x0C41);
        if (gphy_val & BIT_8)
            gphy_val = ReadGMII16( 0x15) | BIT_1;
        else
            gphy_val = ReadGMII16( 0x15) & ~BIT_1;

        WriteGMII16( 0x1F, 0x0A44);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | BIT_2 | BIT_3);

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

        WriteGMII16( 0x1F, 0x0A4B);
        WriteGMII16( 0x11, ReadGMII16( 0x11) | BIT_2);

        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8012);
        WriteGMII16( 0x14, ReadGMII16( 0x14) | BIT_15);

        WriteGMII16( 0x1F, 0x0C42);
        gphy_val = ReadGMII16( 0x11);
        gphy_val |= BIT_14;
        gphy_val &= ~BIT_13;
        WriteGMII16( 0x11, gphy_val);

        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x809A);
        WriteGMII16( 0x14, 0x8022);
        WriteGMII16( 0x13, 0x80A0);
        gphy_val = ReadGMII16( 0x14) & 0x00FF;
        gphy_val |= 0x1000;
        WriteGMII16( 0x14, gphy_val);
        WriteGMII16( 0x13, 0x8088);
        WriteGMII16( 0x14, 0x9222);

        WriteGMII16( 0x1F, 0x0BCD);
        WriteGMII16( 0x14, 0x5065);
        WriteGMII16( 0x14, 0xD065);
        WriteGMII16( 0x1F, 0x0BC8);
        WriteGMII16( 0x11, 0x5655);
        WriteGMII16( 0x1F, 0x0BCD);
        WriteGMII16( 0x14, 0x1065);
        WriteGMII16( 0x14, 0x9065);
        WriteGMII16( 0x14, 0x1065);

        if (aspm) {
            WriteGMII16( 0x1F, 0x0A43);
            WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_2);
        }

        WriteGMII16( 0x1F, 0x0000);
    }
    else if (mcfg == CFG_METHOD_22) {
        WriteGMII16( 0x1F, 0x0BCD);
        WriteGMII16( 0x14, 0x5065);
        WriteGMII16( 0x14, 0xD065);
        WriteGMII16( 0x1F, 0x0BC8);
        WriteGMII16( 0x11, 0x5655);
        WriteGMII16( 0x1F, 0x0BCD);
        WriteGMII16( 0x14, 0x1065);
        WriteGMII16( 0x14, 0x9065);
        WriteGMII16( 0x14, 0x1065);
    }
    else if (mcfg == CFG_METHOD_23) {
        WriteGMII16( 0x1F, 0x0BCC);
        gphy_val = ReadGMII16( 0x14);
        gphy_val |= BIT_8;
        WriteGMII16( 0x14, gphy_val);
        WriteGMII16( 0x1F, 0x0A44);
        gphy_val = ReadGMII16( 0x11);
        gphy_val |= BIT_7 | BIT_6;
        WriteGMII16( 0x11, gphy_val);
        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x8084);
        gphy_val = ReadGMII16( 0x14);
        gphy_val &= ~(BIT_14 | BIT_13);
        WriteGMII16( 0x14, gphy_val);
        gphy_val = ReadGMII16( 0x10);
        gphy_val |= BIT_12;
        WriteGMII16( 0x10, gphy_val);
        gphy_val = ReadGMII16( 0x10);
        gphy_val |= BIT_2 | BIT_1 | BIT_0;
        WriteGMII16( 0x10, gphy_val);

        WriteGMII16( 0x1F, 0x0A43);
        WriteGMII16( 0x13, 0x809C);
        WriteGMII16( 0x14, 0xA700);
        WriteGMII16( 0x13, 0x80A5);
        WriteGMII16( 0x14, 0xA700);
    }
    else if (mcfg == CFG_METHOD_24) {
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

        WriteGMII16( 0x1F, 0x0C42);
        gphy_val = ReadGMII16( 0x11);
        gphy_val |= BIT_14;
        gphy_val &= ~BIT_13;
        WriteGMII16( 0x11, gphy_val);

        if (aspm) {
            WriteGMII16( 0x1F, 0x0A43);
            WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_2);
        }
    }
    else if (mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B) {
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
    else if (mcfg == CFG_METHOD_27) {
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

        WriteGMII16( 0x1F, 0x0C42);
        WriteGMII16( 0x11, (ReadGMII16( 0x11) & ~BIT_13) | BIT_14);

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

        WriteGMII16( 0x1F, 0x0BCD);
        WriteGMII16( 0x14, 0x5065);
        WriteGMII16( 0x14, 0xD065);
        WriteGMII16( 0x1F, 0x0BC8);
        WriteGMII16( 0x12, 0x00ED);
        WriteGMII16( 0x1F, 0x0BCD);
        WriteGMII16( 0x14, 0x1065);
        WriteGMII16( 0x14, 0x9065);
        WriteGMII16( 0x14, 0x1065);
        WriteGMII16( 0x1F, 0x0000);


        if (aspm) {
            WriteGMII16( 0x1F, 0x0A43);
            WriteGMII16( 0x10, ReadGMII16( 0x10) | BIT_2);
        }
    }


    /*ocp phy power saving*/
    if (mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B || mcfg == CFG_METHOD_27) {
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
        rtl8168_enable_EEE(tp);
    else
        rtl8168_disable_EEE(tp);
}


static inline void rtl8168_delete_esd_timer(struct net_device *dev, struct timer_list *timer)
{
    del_timer_sync(timer);
}

static inline void rtl8168_request_esd_timer(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->esd_timer;

    init_timer(timer);
    timer->expires = jiffies + RTL8168_ESD_TIMEOUT;
    timer->data = (unsigned long)(dev);
    timer->function = rtl8168_esd_timer;
    add_timer(timer);
}

static inline void rtl8168_delete_link_timer(struct net_device *dev, struct timer_list *timer)
{
    del_timer_sync(timer);
}

static inline void rtl8168_request_link_timer(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->link_timer;

    init_timer(timer);
    timer->expires = jiffies + RTL8168_LINK_TIMEOUT;
    timer->data = (unsigned long)(dev);
    timer->function = rtl8168_link_timer;
    add_timer(timer);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void
rtl8168_netpoll(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;

    disable_irq(pdev->irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
    rtl8168_interrupt(pdev->irq, dev, NULL);
#else
    rtl8168_interrupt(pdev->irq, dev);
#endif
    enable_irq(pdev->irq);
}
#endif

static void
rtl8168_get_bios_setting(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        bios_setting = ReadMMIO32(0x8c);
        break;
    }
}

static void
rtl8168_set_bios_setting(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (mcfg) {
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO32(0x8C, bios_setting);
        break;
    }
}

static void
rtl8168_init_software_variable(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;

    rtl8168_get_bios_setting(dev);

    switch (mcfg) {
    case MCFG_8168B_1:
        tp->intr_mask = RxDescUnavail | RxFIFOOver | TxDescUnavail | TxOK | RxOK | SWInt;
        break;
    case MCFG_8168B_2:
    case MCFG_8168B_3:
    case MCFG_8168C_1:
        tp->intr_mask = RxDescUnavail | TxDescUnavail | TxOK | RxOK | SWInt;
        break;
    default:
        tp->intr_mask = RxDescUnavail | TxOK | RxOK | SWInt;
        break;
    }

    tp->max_jumbo_frame_size = rtl_chip_info[tp->chipset].jumbo_frame_sz;

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        tp->org_pci_offset_99 = rtl8168_csi_fun0_read_byte(tp, 0x99);
        break;
    }
    switch (mcfg) {
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        tp->org_pci_offset_180 = rtl8168_csi_fun0_read_byte(tp, 0x180);
        break;
    }

    pci_read_config_byte(pdev, 0x80, &tp->org_pci_offset_80);
    pci_read_config_byte(pdev, 0x81, &tp->org_pci_offset_81);

    switch (mcfg) {
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        if ((tp->features & RTL_FEATURE_MSI) && (tp->org_pci_offset_80 & BIT_1))
            tp->use_timer_interrrupt = FALSE;
        else
            tp->use_timer_interrrupt = TRUE;
        break;
    default:
        tp->use_timer_interrrupt = TRUE;
        break;
    }

    rtl8168_get_hw_wol(dev);
}

static void
rtl8168_release_board(struct pci_dev *pdev,
                      struct net_device *dev,
                      void __iomem *ioaddr)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    rtl8168_set_bios_setting(dev);
    rtl8168_rar_set(tp, tp->org_mac_addr);
    tp->wol_enabled = WOL_DISABLED;

    rtl8168_phy_power_down(dev);

    iounmap(ioaddr);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    free_netdev(dev);
}

static int
rtl8168_get_mac_address(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;


    if (mcfg == MCFG_8168F_1 ||
        mcfg == MCFG_8168F_2 ||
        mcfg == MCFG_8411_1 ||
        mcfg == CFG_METHOD_21 ||
        mcfg == CFG_METHOD_22 ||
        mcfg == CFG_METHOD_23 ||
        mcfg == CFG_METHOD_24 ||
        mcfg == CFG_METHOD_25 ||
        mcfg == MCFG_8411B ||
        mcfg == CFG_METHOD_27) {
        u16 mac_addr[3];

        *(u32*)&mac_addr[0] = ReadERI(0xE0, 4, ERIAR_ExGMAC);
        *(u16*)&mac_addr[2] = ReadERI(0xE4, 2, ERIAR_ExGMAC);

        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        WriteMMIO32(MAC0, (mac_addr[1] << 16) | mac_addr[0]);
        WriteMMIO16(MAC4, mac_addr[2]);
        WriteMMIO8(Cfg9346, Cfg9346_Lock);
    } else {
        if (tp->eeprom_type != EEPROM_TYPE_NONE) {
            u16 mac_addr[3];

            /* Get MAC address from EEPROM */
            if (mcfg == MCFG_8168E_VL_1 ||
                mcfg == MCFG_8168E_VL_2 ||
                mcfg == MCFG_8168F_1 ||
                mcfg == MCFG_8168F_2 ||
                mcfg == MCFG_8411_1 ||
                mcfg == CFG_METHOD_21 ||
                mcfg == CFG_METHOD_22 ||
                mcfg == CFG_METHOD_23 ||
                mcfg == CFG_METHOD_24 ||
                mcfg == CFG_METHOD_25 ||
                mcfg == MCFG_8411B ||
                mcfg == CFG_METHOD_27) {
                mac_addr[0] = rtl_eeprom_read_sc(tp, 1);
                mac_addr[1] = rtl_eeprom_read_sc(tp, 2);
                mac_addr[2] = rtl_eeprom_read_sc(tp, 3);
            } else {
                mac_addr[0] = rtl_eeprom_read_sc(tp, 7);
                mac_addr[1] = rtl_eeprom_read_sc(tp, 8);
                mac_addr[2] = rtl_eeprom_read_sc(tp, 9);
            }
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
 * rtl8168_set_mac_address - Change the Ethernet Address of the NIC
 * @dev: network interface device structure
 * @p:   pointer to an address structure
 *
 * Return 0 on success, negative on failure
 **/
static int
rtl8168_set_mac_address(struct net_device *dev,
                        void *p)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct sockaddr *addr = p;
    unsigned long flags;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8168_rar_set(tp, dev->dev_addr);

    spin_unlock_irqrestore(&tp->lock, flags);

    return 0;
}

/******************************************************************************
 * rtl8168_rar_set - Puts an ethernet address into a receive address register.
 *
 * tp - The private data structure for driver
 * addr - Address to put into receive address register
 *****************************************************************************/
void
rtl8168_rar_set(struct rtl8168_private *tp,
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

    if (mcfg == MCFG_8168E_VL_2) {
        WriteERI(0xe0, 4, rar_low, ERIAR_ExGMAC);
        WriteERI(0xe4, 4, rar_high, ERIAR_ExGMAC);
        WriteERI(0xf0, 4, rar_low << 16, ERIAR_ExGMAC);
        WriteERI(0xf4, 4, rar_low >> 16 | rar_high << 16, ERIAR_ExGMAC);
    }

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
rtl8168_do_ioctl(struct net_device *dev,
                 struct ifreq *ifr,
                 int cmd)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct mii_ioctl_data *data = if_mii(ifr);
    int ret;
    unsigned long flags;

    ret = 0;
    switch (cmd) {
    case SIOCGMIIPHY:
        data->phy_id = 32; /* Internal PHY */
        break;

    case SIOCGMIIREG:
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0000);
        data->val_out = ReadGMII16( data->reg_num);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case SIOCSMIIREG:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;
        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( 0x1F, 0x0000);
        WriteGMII16( data->reg_num, data->val_in);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

#ifdef ETHTOOL_OPS_COMPAT
    case SIOCETHTOOL:
        ret = ethtool_ioctl(ifr);
        break;
#endif
    case SIOCDEVPRIVATE_RTLASF:
        if (!netif_running(dev)) {
            ret = -ENODEV;
            break;
        }

        ret = rtl8168_asf_ioctl(dev, ifr);
        break;

    case SIOCRTLTOOL:
        ret = rtltool_ioctl(tp, ifr);
        break;

    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}

static void
rtl8168_phy_power_up (struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1F, 0x0000);
    switch (mcfg) {
    case MCFG_8168B_1:
    case MCFG_8168B_2:
    case MCFG_8168B_3:
    case MCFG_8168C_1:
    case MCFG_8168C_2:
    case MCFG_8168C_3:
    case MCFG_8168CP_1:
    case MCFG_8168CP_2:
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
        WriteGMII16( 0x0E, 0x0000);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case CFG_METHOD_27:
        csi_tmp = ReadERI(0x1AB, 1, ERIAR_ExGMAC);
        csi_tmp |=  ( BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 );
        WriteERI(0x1AB, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    default:
        break;
    }
    WriteGMII16( MII_BMCR, BMCR_ANENABLE);
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
rtl8168_phy_power_down (struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u32 csi_tmp;
    unsigned long flags;

    spin_lock_irqsave(&tp->phy_lock, flags);
    WriteGMII16( 0x1F, 0x0000);
    switch (mcfg) {
    case MCFG_8168B_1:
    case MCFG_8168B_2:
    case MCFG_8168B_3:
    case MCFG_8168C_1:
    case MCFG_8168C_2:
    case MCFG_8168C_3:
    case MCFG_8168CP_1:
    case MCFG_8168CP_2:
    case MCFG_8168D_1:
    case MCFG_8168D_2:
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
        WriteGMII16( 0x0E, 0x0200);
        WriteGMII16( MII_BMCR, BMCR_PDOWN);
        break;
    case MCFG_8168E_1:
    case MCFG_8168E_2:
        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);
        break;
    case CFG_METHOD_21:
    case CFG_METHOD_22:
        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);

        csi_tmp = ReadERI(0x1AB, 1, ERIAR_ExGMAC);
        csi_tmp &= ~( BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 );
        WriteERI(0x1AB, 1, csi_tmp, ERIAR_ExGMAC);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) & ~BIT_6);
        break;
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case CFG_METHOD_27:
        WriteGMII16( MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN);

        csi_tmp = ReadERI(0x1AB, 1, ERIAR_ExGMAC);
        csi_tmp &= ~( BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 );
        WriteERI(0x1AB, 1, csi_tmp, ERIAR_ExGMAC);
        break;
    default:
        WriteGMII16( MII_BMCR, BMCR_PDOWN);
        break;
    }
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static int __devinit
rtl8168_init_board(struct pci_dev *pdev,
                   struct net_device **dev_out,
                   void __iomem **ioaddr_out)
{
    void __iomem *ioaddr;
    struct net_device *dev;
    struct rtl8168_private *tp;
    int rc = -ENOMEM, i, pm_cap;

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
    tp->msg_enable = netif_msg_init(debug.msg_enable, R8168_MSG_DEFAULT);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    if (!aspm)
        pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
                               PCIE_LINK_STATE_CLKPM);
#endif

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
            dev_err(&pdev->dev, "region #1 not an MMIO resource, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -ENODEV;
        goto err_out_mwi;
    }
    /* check for weird/broken PCI region reporting */
    if (pci_resource_len(pdev, 2) < R8168_REGS_SIZE) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "Invalid PCI region size(s), aborting\n");
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
    ioaddr = ioremap(pci_resource_start(pdev, 2), R8168_REGS_SIZE);
    if (ioaddr == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "cannot remap MMIO, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -EIO;
        goto err_out_free_res;
    }

    /* Identify chip attached to board */
    rtl8168_get_mac_version(tp, ioaddr);

    rtl8168_print_mac_version(tp);

    for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
        if (mcfg == rtl_chip_info[i].mcfg)
            break;
    }

    if (i < 0) {
        /* Unknown chip: assume array element #0, original RTL-8168 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_printk(KERN_DEBUG, &pdev->dev, "unknown chip version, assuming %s\n", rtl_chip_info[0].name);
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
rtl8168_esd_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    struct timer_list *timer = &tp->esd_timer;
    unsigned long timeout = RTL8168_ESD_TIMEOUT;
    unsigned long flags;
    u8 cmd;
    u16 io_base_l;
    u16 mem_base_l;
    u16 mem_base_h;
    u8 ilr;
    u16 resv_0x1c_h;
    u16 resv_0x1c_l;
    u16 resv_0x20_l;
    u16 resv_0x20_h;
    u16 resv_0x24_l;
    u16 resv_0x24_h;
    u16 resv_0x2c_h;
    u16 resv_0x2c_l;
    u32 pci_nvidia_geforce_6200;
    u32 pci_nvidia_geforce__6250_1;

    spin_lock_irqsave(&tp->lock, flags);

    tp->esd_flag = 0;

    pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
    if (cmd != tp->pci_cfg_space.cmd) {
        pci_write_config_byte(pdev, PCI_COMMAND, tp->pci_cfg_space.cmd);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &io_base_l);
    if (io_base_l != tp->pci_cfg_space.io_base_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_0, tp->pci_cfg_space.io_base_l);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &mem_base_l);
    if (mem_base_l != tp->pci_cfg_space.mem_base_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_2, tp->pci_cfg_space.mem_base_l);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &mem_base_h);
    if (mem_base_h!= tp->pci_cfg_space.mem_base_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, tp->pci_cfg_space.mem_base_h);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &resv_0x1c_l);
    if (resv_0x1c_l != tp->pci_cfg_space.resv_0x1c_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_3, tp->pci_cfg_space.resv_0x1c_l);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &resv_0x1c_h);
    if (resv_0x1c_h != tp->pci_cfg_space.resv_0x1c_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, tp->pci_cfg_space.resv_0x1c_h);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &resv_0x20_l);
    if (resv_0x20_l != tp->pci_cfg_space.resv_0x20_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_4, tp->pci_cfg_space.resv_0x20_l);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &resv_0x20_h);
    if (resv_0x20_h != tp->pci_cfg_space.resv_0x20_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, tp->pci_cfg_space.resv_0x20_h);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &resv_0x24_l);
    if (resv_0x24_l != tp->pci_cfg_space.resv_0x24_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_5, tp->pci_cfg_space.resv_0x24_l);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &resv_0x24_h);
    if (resv_0x24_h != tp->pci_cfg_space.resv_0x24_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, tp->pci_cfg_space.resv_0x24_h);
        tp->esd_flag = 1;
    }

    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &ilr);
    if (ilr != tp->pci_cfg_space.ilr) {
        pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, tp->pci_cfg_space.ilr);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &resv_0x2c_l);
    if (resv_0x2c_l != tp->pci_cfg_space.resv_0x2c_l) {
        pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, tp->pci_cfg_space.resv_0x2c_l);
        tp->esd_flag = 1;
    }

    pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &resv_0x2c_h);
    if (resv_0x2c_h != tp->pci_cfg_space.resv_0x2c_h) {
        pci_write_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, tp->pci_cfg_space.resv_0x2c_h);
        tp->esd_flag = 1;
    }

    pci_nvidia_geforce_6200 = rtl8168_csi_read(tp, PCI_DEVICE_ID_NVIDIA_GEFORCE_GO_6200);
    if (pci_nvidia_geforce_6200 != tp->pci_cfg_space.pci_nvidia_geforce_6200) {
        rtl8168_csi_write(tp, PCI_DEVICE_ID_NVIDIA_GEFORCE_GO_6200, tp->pci_cfg_space.pci_nvidia_geforce_6200);
        tp->esd_flag = 1;
    }

    pci_nvidia_geforce__6250_1 = rtl8168_csi_read(tp, PCI_DEVICE_ID_NVIDIA_GEFORCE_GO_6250_1);
    if (pci_nvidia_geforce__6250_1 != tp->pci_cfg_space.pci_nvidia_geforce__6250_1) {
        rtl8168_csi_write(tp, PCI_DEVICE_ID_NVIDIA_GEFORCE_GO_6250_1, tp->pci_cfg_space.pci_nvidia_geforce__6250_1);
        tp->esd_flag = 1;
    }

    if (tp->esd_flag != 0) {
        netif_stop_queue(dev);
        netif_carrier_off(dev);
        rtl8168_hw_reset(dev);
        rtl8168_tx_clear(tp);
        rtl8168_init_ring_indexes(tp);
        rtl8168_hw_init(dev);
        rtl8168_powerup_pll(dev);
        rtl8168_hw_ephy_config(dev);
        rtl8168_hw_phy_config(dev);
        rtl8168_hw_start(dev);
        rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
        tp->esd_flag = 0;
    }
    spin_unlock_irqrestore(&tp->lock, flags);

    mod_timer(timer, jiffies + timeout);
}

static void
rtl8168_link_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8168_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->link_timer;
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    rtl8168_check_link_status(dev);
    spin_unlock_irqrestore(&tp->lock, flags);

    mod_timer(timer, jiffies + RTL8168_LINK_TIMEOUT);
}

/* Cfg9346_Unlock assumed. */
static unsigned rtl8168_try_msi(struct pci_dev *pdev, void __iomem *ioaddr)
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

static void rtl8168_disable_msi(struct pci_dev *pdev, struct rtl8168_private *tp)
{
    if (tp->features & RTL_FEATURE_MSI) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
        pci_disable_msi(pdev);
#endif
        tp->features &= ~RTL_FEATURE_MSI;
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops rtl8168_netdev_ops = {
    .ndo_open       = rtl8168_open,
    .ndo_stop       = rtl8168_close,
    .ndo_get_stats      = rtl8168_get_stats,
    .ndo_start_xmit     = rtl8168_start_xmit,
    .ndo_tx_timeout     = rtl8168_tx_timeout,
    .ndo_change_mtu     = rtl8168_change_mtu,
    .ndo_set_mac_address    = rtl8168_set_mac_address,
    .ndo_do_ioctl       = rtl8168_do_ioctl,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
    .ndo_set_multicast_list = rtl8168_set_rx_mode,
#else
    .ndo_set_rx_mode    = rtl8168_set_rx_mode,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#ifdef CONFIG_R8168_VLAN
    .ndo_vlan_rx_register   = rtl8168_vlan_rx_register,
#endif
#else
    .ndo_fix_features   = rtl8168_fix_features,
    .ndo_set_features   = rtl8168_set_features,
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
    .ndo_poll_controller    = rtl8168_netpoll,
#endif
};
#endif

static int __devinit
rtl8168_init_one(struct pci_dev *pdev,
                 const struct pci_device_id *ent)
{
    struct net_device *dev = NULL;
    struct rtl8168_private *tp;
    void __iomem *ioaddr = NULL;
    static int board_idx = -1;

    int rc;

    assert(pdev != NULL);
    assert(ent != NULL);

    board_idx++;

    if (netif_msg_drv(&debug))
        printk(KERN_INFO "%s Gigabit Ethernet driver %s loaded\n",
               MODULENAME, RTL8168_VERSION);

    rc = rtl8168_init_board(pdev, &dev, &ioaddr);
    if (rc)
        return rc;

    tp = netdev_priv(dev);
    assert(ioaddr != NULL);

    tp->mmio_addr = ioaddr;
    tp->set_speed = rtl8168_set_speed_xmii;
    tp->get_settings = rtl8168_gset_xmii;
    tp->phy_reset_enable = rtl8168_xmii_reset_enable;
    tp->phy_reset_pending = rtl8168_xmii_reset_pending;
    tp->link_ok = rtl8168_xmii_link_ok;

    tp->features |= rtl8168_try_msi(pdev, ioaddr);

    RTL_NET_DEVICE_OPS(rtl8168_netdev_ops);

    SET_ETHTOOL_OPS(dev, &rtl8168_ethtool_ops);

    dev->watchdog_timeo = RTL8168_TX_TIMEOUT;
    dev->irq = pdev->irq;
    dev->base_addr = (unsigned long) ioaddr;

#ifdef CONFIG_R8168_NAPI
    RTL_NAPI_CONFIG(dev, tp, rtl8168_poll, R8168_NAPI_WEIGHT);
#endif

#ifdef CONFIG_R8168_VLAN
    if (mcfg != CFG_METHOD_DEFAULT) {
        dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
        dev->vlan_rx_kill_vid = rtl8168_vlan_rx_kill_vid;
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    }
#endif

    cp_cmd |= ReadMMIO16(CPlusCmd);
    if (mcfg != CFG_METHOD_DEFAULT) {
        dev->features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        cp_cmd |= RxChkSum;
#else
        dev->features |= NETIF_F_RXCSUM | NETIF_F_SG;
        dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                           NETIF_F_RXCSUM | NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
        dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
                             NETIF_F_HIGHDMA;
#endif
    }

    tp->pci_dev = pdev;

    spin_lock_init(&tp->lock);

    spin_lock_init(&tp->phy_lock);

    rtl8168_init_software_variable(dev);

    rtl8168_exit_oob(dev);

    rtl8168_hw_init(dev);

    rtl8168_hw_reset(dev);

    /* Get production from EEPROM */
    if (((mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 || mcfg == CFG_METHOD_25) && (mac_ocp_read(0xDC00) & BIT_3)) ||
        ((mcfg == MCFG_8411B) && (mac_ocp_read(0xDC00) & BIT_4)))
        tp->eeprom_type = EEPROM_TYPE_NONE;
    else
        rtl_eeprom_type(tp);

    if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
        rtl_set_eeprom_sel_low(ioaddr);

    rtl8168_get_mac_address(dev);

    pci_set_drvdata(pdev, dev);

    if (netif_msg_probe(tp)) {
        printk(KERN_INFO "%s: 0x%lx, "
               "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
               "IRQ %d\n",
               dev->name,
               dev->base_addr,
               dev->dev_addr[0], dev->dev_addr[1],
               dev->dev_addr[2], dev->dev_addr[3],
               dev->dev_addr[4], dev->dev_addr[5], dev->irq);
    }

    rtl8168_link_option(board_idx, (u8*)&autoneg, (u16*)&speed, (u8*)&duplex);

    rc = register_netdev(dev);
    if (rc) {
        rtl8168_release_board(pdev, dev, ioaddr);
        return rc;
    }

    printk(KERN_INFO "%s: This product is covered by one or more of the following patents: US6,570,884, US6,115,776, and US6,327,625.\n", MODULENAME);

    netif_carrier_off(dev);

    printk("%s", GPL_CLAIM);

    return 0;
}

static void __devexit
rtl8168_remove_one(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8168_private *tp = netdev_priv(dev);

    assert(dev != NULL);
    assert(tp != NULL);

    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case CFG_METHOD_23:
    case CFG_METHOD_27:
        rtl8168_driver_stop(tp);
        break;
    }

    flush_scheduled_work();

    unregister_netdev(dev);
    rtl8168_disable_msi(pdev, tp);
    rtl8168_release_board(pdev, dev, tp->mmio_addr);
    pci_set_drvdata(pdev, NULL);
}

static void
rtl8168_set_rxbufsize(struct rtl8168_private *tp,
                      struct net_device *dev)
{
    unsigned int mtu = dev->mtu;

    tp->rx_buf_sz = (mtu > ETH_DATA_LEN) ? mtu + ETH_HLEN + 8 + 1 : RX_BUF_SIZE;
}

static int rtl8168_open(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    int retval;

    tp->in_open_fun = TRUE;

    retval = -ENOMEM;

    rtl8168_set_rxbufsize(tp, dev);
    /*
     * Rx and Tx descriptors needs 256 bytes alignment.
     * pci_alloc_consistent provides more.
     */
    tp->TxDescArray = pci_alloc_consistent(pdev, R8168_TX_RING_BYTES,
                                           &tp->TxPhyAddr);
    if (!tp->TxDescArray)
        goto out;

    tp->RxDescArray = pci_alloc_consistent(pdev, R8168_RX_RING_BYTES,
                                           &tp->RxPhyAddr);
    if (!tp->RxDescArray)
        goto err_free_tx;

    tp->tally_vaddr = pci_alloc_consistent(pdev, sizeof(*tp->tally_vaddr), &tp->tally_paddr);
    if (!tp->tally_vaddr)
        goto err_free_rx;

    retval = rtl8168_init_ring(dev);
    if (retval < 0)
        goto err_free_counters;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    INIT_WORK(&tp->task, NULL, dev);
#else
    INIT_DELAYED_WORK(&tp->task, NULL);
#endif

#ifdef  CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif

    rtl8168_exit_oob(dev);

    rtl8168_tally_counter_clear(tp);

    rtl8168_hw_init(dev);

    rtl8168_hw_reset(dev);

    rtl8168_powerup_pll(dev);

    rtl8168_hw_ephy_config(dev);

    rtl8168_hw_phy_config(dev);

    rtl8168_hw_start(dev);

    rtl8168_dsm(dev, DSM_IF_UP);

    rtl8168_set_speed(dev, autoneg, speed, duplex);

    retval = request_irq(dev->irq, rtl8168_interrupt, (tp->features & RTL_FEATURE_MSI) ? 0 : SA_SHIRQ, dev->name, dev);
    if (retval<0)
        goto err_free_counters;

    if (tp->esd_flag == 0)
        rtl8168_request_esd_timer(dev);

    rtl8168_request_link_timer(dev);

out:
    tp->in_open_fun = FALSE;

    return retval;

err_free_counters:
    pci_free_consistent(pdev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);

    tp->tally_vaddr = NULL;
err_free_rx:
    pci_free_consistent(pdev, R8168_RX_RING_BYTES, tp->RxDescArray,
                        tp->RxPhyAddr);
    tp->RxDescArray = NULL;
err_free_tx:
    pci_free_consistent(pdev, R8168_TX_RING_BYTES, tp->TxDescArray,
                        tp->TxPhyAddr);
    tp->TxDescArray = NULL;
    goto out;
}

static void
rtl8168_dsm(struct net_device *dev, int dev_state)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    switch (dev_state) {
    case DSM_MAC_INIT:
        if ((mcfg == MCFG_8168C_2) || (mcfg == MCFG_8168C_3)) {
            if (ReadMMIO8(MACDBG) & 0x80)
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) | GPIO_en);
            else
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) & ~GPIO_en);
        }

        break;
    case DSM_NIC_GOTO_D3:
    case DSM_IF_DOWN:
        if ((mcfg == MCFG_8168C_2) || (mcfg == MCFG_8168C_3)) {
            if (ReadMMIO8(MACDBG) & 0x80)
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) & ~GPIO_en);
        }
        break;

    case DSM_NIC_RESUME_D3:
    case DSM_IF_UP:
        if ((mcfg == MCFG_8168C_2) || (mcfg == MCFG_8168C_3)) {
            if (ReadMMIO8(MACDBG) & 0x80)
                WriteMMIO8(GPIO, ReadMMIO8(GPIO) | GPIO_en);
        }

        break;
    }

}
static void
set_offset70F(struct rtl8168_private *tp, u8 setting)
{

    u32 csi_tmp;
    u32 temp = (u32)setting;
    temp = temp << 24;
    /*set PCI configuration space offset 0x70F to setting*/
    /*When the register offset of PCI configuration space larger than 0xff, use CSI to access it.*/

    csi_tmp = rtl8168_csi_read(tp, 0x70c) & 0x00ffffff;
    rtl8168_csi_write(tp, 0x70c, csi_tmp | temp);
}

static void
set_offset79(struct rtl8168_private *tp, u8 setting)
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
rtl8168_hw_set_rx_packet_filter(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u32 mc_filter[2];   /* Multicast hash filter */
    int rx_mode;
    u32 tmp = 0;

    if (dev->flags & IFF_PROMISC) {
        /* Unconditionally log net taps. */
        if (netif_msg_link(tp))
            printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n",
                   dev->name);

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
        unsigned int i;

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

    tmp = mc_filter[0];
    mc_filter[0] = swab32(mc_filter[1]);
    mc_filter[1] = swab32(tmp);

    tp->rtl8168_rx_config = rtl_chip_info[tp->chipset].RCR_Cfg;
    tmp = tp->rtl8168_rx_config | rx_mode | (ReadMMIO32(RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);

    WriteMMIO32(RxConfig, tmp);
    WriteMMIO32(MAR0 + 0, mc_filter[0]);
    WriteMMIO32(MAR0 + 4, mc_filter[1]);
}

static void
rtl8168_set_rx_mode(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);

    rtl8168_hw_set_rx_packet_filter(dev);

    spin_unlock_irqrestore(&tp->lock, flags);
}

static void
rtl8168_hw_start(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    struct pci_dev *pdev = tp->pci_dev;
    u8 device_control;
    u16 mac_ocp_data;
    u32 csi_tmp;
    unsigned long flags;

    netif_stop_queue(dev);

    WriteMMIO32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));

    rtl8168_hw_reset(dev);

    rtl8168_rx_desc_offset0_init(tp, 1);

    WriteMMIO8(Cfg9346, Cfg9346_Unlock);
    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        WriteMMIO8(0xF1, ReadMMIO8(0xF1) & ~BIT_7);
        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        break;
    }
    WriteMMIO8(MTPS, Reserved1_data);

    cp_cmd |= PktCntrDisable | INTT_1;
    WriteMMIO16(CPlusCmd, cp_cmd);

    WriteMMIO16(IntrMitigate, 0x5f51);

    rtl8168_tally_counter_addr_fill(tp);

    rtl8168_desc_addr_fill(tp);

    /* Set DMA burst size and Interframe Gap Time */
    if (mcfg == MCFG_8168B_1)
        WriteMMIO32(TxConfig, (TX_DMA_BURST_512 << TxDMAShift) |
                (InterFrameGap << TxInterFrameGapShift));
    else
        WriteMMIO32(TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
                (InterFrameGap << TxInterFrameGapShift));

    if (mcfg == MCFG_8168C_1) {
        set_offset70F(tp, 0x27);

        WriteMMIO8(DBG_reg, (0x0E << 4) | Fix_Nak_1 | Fix_Nak_2);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        //disable clock request.
        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            //tx checksum offload disable
            dev->features &= ~NETIF_F_IP_CSUM;

            //rx checksum offload disable
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);

            set_offset79(tp, 0x50);

            //tx checksum offload enable
            dev->features |= NETIF_F_IP_CSUM;
        }

        //rx checksum offload enable
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        cp_cmd |= RxChkSum;
        WriteMMIO16(CPlusCmd, cp_cmd);
#else
        dev->features |= NETIF_F_RXCSUM;
#endif
    } else if (mcfg == MCFG_8168C_2) {

        set_offset70F(tp, 0x27);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        //disable clock request.
        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            //tx checksum offload disable
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);

            set_offset79(tp, 0x50);

            //tx checksum offload enable
            dev->features |= NETIF_F_IP_CSUM;
        }

        //rx checksum offload enable
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        cp_cmd |= RxChkSum;
        WriteMMIO16(CPlusCmd, cp_cmd);
#else
        dev->features |= NETIF_F_RXCSUM;
#endif
    } else if (mcfg == MCFG_8168C_3) {
        set_offset70F(tp, 0x27);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        //disable clock request.
        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            //tx checksum offload disable
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);

            set_offset79(tp, 0x50);

            //tx checksum offload enable
            dev->features |= NETIF_F_IP_CSUM;
        }

        //rx checksum offload enable
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
        cp_cmd |= RxChkSum;
        WriteMMIO16(CPlusCmd, cp_cmd);
#else
        dev->features |= NETIF_F_RXCSUM;
#endif
    } else if (mcfg == MCFG_8168CP_1) {
        set_offset70F(tp, 0x27);

        WriteERI(0x1EC, 1, 0x07, ERIAR_ASF);

        //disable clock request.
        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            //tx checksum offload disable
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);


            set_offset79(tp, 0x50);

            //tx checksum offload enable
            dev->features |= NETIF_F_IP_CSUM;
        }
    } else if (mcfg == MCFG_8168CP_2) {

        set_offset70F(tp, 0x27);

        WriteERI(0x1EC, 1, 0x07, ERIAR_ASF);

        //disable clock request.
        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        WriteMMIO8(0xD1, 0x20);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            //tx checksum offload disable
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);


            set_offset79(tp, 0x50);

            //tx checksum offload enable
            dev->features |= NETIF_F_IP_CSUM;
        }

    } else if (mcfg == MCFG_8168D_1) {
        set_offset70F(tp, 0x27);

        /* disable clock request. */
        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~BIT_4);
        WriteMMIO8(DBG_reg, ReadMMIO8(DBG_reg) | BIT_7 | BIT_1);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);


            set_offset79(tp, 0x50);

            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        WriteMMIO8(TDFNR, 0x8);

    } else if (mcfg == MCFG_8168D_2) {
        set_offset70F(tp, 0x27);

        WriteMMIO8(DBG_reg, ReadMMIO8(DBG_reg) | BIT_7 | BIT_1);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | Jumbo_En1);

            set_offset79(tp, 0x20);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~Jumbo_En1);

            set_offset79(tp, 0x50);

            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        WriteMMIO8(TDFNR, 0x8);

        WriteMMIO8(Config1, ReadMMIO8(Config1) | 0x10);

        /* disable clock request. */
        pci_write_config_byte(pdev, 0x81, 0x00);
    } else if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_3) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);

            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        pci_write_config_byte(pdev, 0x81, 0x00);

        WriteMMIO8(Config1, ReadMMIO8(Config1) | 0x10);

    } else if (mcfg == MCFG_8168DP_2) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);

            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        pci_write_config_byte(pdev, 0x81, 0x01);

        WriteMMIO8(Config1, ReadMMIO8(Config1) | 0x10);

    } else if (mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2) {

        set_offset70F(tp, 0x27);
        set_offset79(tp, 0x50);

        cp_cmd &= 0x2063;
        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(MTPS, 0x24);
            WriteMMIO8(Config3, ReadMMIO8(Config3) | Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) | 0x01);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Jumbo_En0);
            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~0x01);

            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

//      WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_0);
//      WriteMMIO32(CounterAddrLow, ReadMMIO32(CounterAddrLow) | BIT_0);

        WriteMMIO8(0xF3, ReadMMIO8(0xF3) | BIT_5);
        WriteMMIO8(0xF3, ReadMMIO8(0xF3) & ~BIT_5);

//      WriteMMIO8(0xD3, ReadMMIO8(0xD3) | BIT_3 | BIT_2);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_7 | BIT_6);

        WriteMMIO8(0xD1, ReadMMIO8(0xD1) | BIT_2 | BIT_3);

        WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_6 | BIT_5 | BIT_4 | BIT_2 | BIT_1);

        WriteMMIO8(TDFNR, 0x8);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

        WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_3);

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);
    } else if (mcfg == MCFG_8168E_VL_1 || mcfg == MCFG_8168E_VL_2) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        csi_tmp = ReadERI(0xD5, 1, ERIAR_ExGMAC) | BIT_3 | BIT_2;
        WriteERI(0xD5, 1, csi_tmp, ERIAR_ExGMAC);
        WriteERI(0xC0, 2, 0x0000, ERIAR_ExGMAC);
        WriteERI(0xB8, 4, 0x00000000, ERIAR_ExGMAC);
        WriteERI(0xC8, 4, 0x00100002, ERIAR_ExGMAC);
        WriteERI(0xE8, 4, 0x00100006, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0x1D0, 4, ERIAR_ExGMAC);
        csi_tmp |= BIT_1;
        WriteERI(0x1D0, 1, csi_tmp, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);

        WriteMMIO32(TxConfig, ReadMMIO32(TxConfig) | BIT_7);
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) & ~BIT_7);
        WriteMMIO8(0x1B, ReadMMIO8(0x1B) & ~0x07);

        if (mcfg == MCFG_8168E_VL_1) {
            WriteMMIO32(0xB0, 0xEE480010);
            WriteMMIO8(0x1A, ReadMMIO8(0x1A) & ~(BIT_2|BIT_3));
            WriteERI(0x1DC, 1, 0x64, ERIAR_ExGMAC);
        } else {
            csi_tmp = ReadERI(0x1B0, 4, ERIAR_ExGMAC);
            csi_tmp |= BIT_4;
            WriteERI(0x1B0, 1, csi_tmp, ERIAR_ExGMAC);
            WriteERI(0xCC, 4, 0x00000050, ERIAR_ExGMAC);
            WriteERI(0xD0, 4, 0x07ff0060, ERIAR_ExGMAC);
        }

        WriteMMIO8(TDFNR, 0x8);

        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~PMSTS_En);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_6);
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_6);

        cp_cmd &= 0x2063;
        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(MTPS, 0x27);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        /* disable clock request. */
        pci_write_config_byte(pdev, 0x81, 0x00);

    } else if (mcfg == MCFG_8168F_1 || mcfg == MCFG_8168F_2) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        WriteERI(0xC8, 4, 0x00100002, ERIAR_ExGMAC);
        WriteERI(0xE8, 4, 0x00100006, ERIAR_ExGMAC);
        WriteMMIO32(TxConfig, ReadMMIO32(TxConfig) | BIT_7);
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) & ~BIT_7);
        csi_tmp = ReadERI(0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

        cp_cmd &= 0x2063;
        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(MTPS, 0x27);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        WriteMMIO8(TDFNR, 0x8);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_6);
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_6);

        WriteERI(0xC0, 2, 0x0000, ERIAR_ExGMAC);
        WriteERI(0xB8, 4, 0x00000000, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0xD5, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_3 | BIT_2;
        WriteERI(0xD5, 1, csi_tmp, ERIAR_ExGMAC);
        WriteMMIO8(0x1B,ReadMMIO8(0x1B) & ~0x07);

        csi_tmp = ReadERI(0x1B0, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_4;
        WriteERI(0x1B0, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0x1d0, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_4 | BIT_1;
        WriteERI(0x1d0, 1, csi_tmp, ERIAR_ExGMAC);
        WriteERI(0xCC, 4, 0x00000050, ERIAR_ExGMAC);
        WriteERI(0xd0, 4, 0x00000060, ERIAR_ExGMAC);
    } else if (mcfg == MCFG_8411_1) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        WriteERI(0xC8, 4, 0x00100002, ERIAR_ExGMAC);
        WriteERI(0xE8, 4, 0x00100006, ERIAR_ExGMAC);
        WriteMMIO32(TxConfig, ReadMMIO32(TxConfig) | BIT_7);
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) & ~BIT_7);
        csi_tmp = ReadERI(0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        WriteERI(0xDC, 1, csi_tmp, ERIAR_ExGMAC);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

        cp_cmd &= 0x2063;
        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(MTPS, 0x27);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        WriteMMIO8(TDFNR, 0x8);

        WriteMMIO8(0xD0, ReadMMIO8(0xD0) | BIT_6);
        WriteMMIO8(0xF2, ReadMMIO8(0xF2) | BIT_6);
        WriteERI(0xC0, 2, 0x0000, ERIAR_ExGMAC);
        WriteERI(0xB8, 4, 0x00000000, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0xD5, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_3 | BIT_2;
        WriteERI(0xD5, 1, csi_tmp, ERIAR_ExGMAC);

        csi_tmp = ReadERI(0x1B0, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_4;
        WriteERI(0x1B0, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp = ReadERI(0x1d0, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_4 | BIT_1;
        WriteERI(0x1d0, 1, csi_tmp, ERIAR_ExGMAC);
        WriteERI(0xCC, 4, 0x00000050, ERIAR_ExGMAC);
        WriteERI(0xd0, 4, 0x00000060, ERIAR_ExGMAC);
    } else if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
               mcfg == CFG_METHOD_24 || mcfg == CFG_METHOD_25 ||
               mcfg == MCFG_8411B) {
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

        if (mcfg == MCFG_8411B) {
            mac_ocp_data = mac_ocp_read(0xD3C0);
            mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
            mac_ocp_data |= 0x03A9;
            mac_ocp_write(0xD3C0, mac_ocp_data);
            mac_ocp_data = mac_ocp_read(0xD3C2);
            mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
            mac_ocp_write(0xD3C2, mac_ocp_data);
            mac_ocp_data = mac_ocp_read(0xD3C4);
            mac_ocp_data |= BIT_0;
            mac_ocp_write(0xD3C4, mac_ocp_data);
        }

        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        cp_cmd = ReadMMIO16(CPlusCmd) &
                     ~(EnableBist | Macdbgo_oe | Force_halfdup |
                       Force_rxflow_en | Force_txflow_en |
                       Cxpl_dbg_sel | ASF | PktCntrDisable |
                       Macdbgo_sel);

        WriteMMIO8(0x1B, ReadMMIO8(0x1B) & ~0x07);

        WriteMMIO8(TDFNR, 0x4);

        WriteMMIO8(Config2, ReadMMIO8(Config2) & ~PMSTS_En);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(MTPS, 0x27);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

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
    } else if (mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_27) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);

        WriteERI(0xC8, 4, 0x00080002, ERIAR_ExGMAC);
        WriteERI(0xCC, 1, 0x2f, ERIAR_ExGMAC);
        WriteERI(0xD0, 1, 0x5f, ERIAR_ExGMAC);
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

        WriteERI(0xC0, 2, 0x00000000, ERIAR_ExGMAC);
        WriteERI(0xB8, 2, 0x00000000, ERIAR_ExGMAC);
        WriteMMIO8(0x1B, ReadMMIO8(0x1B) & ~0x07);

        WriteMMIO8(TDFNR, 0x4);

        if (aspm)
            WriteMMIO8(0xF1, ReadMMIO8(0xF1) | BIT_7);

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

        if (dev->mtu > ETH_DATA_LEN) {
            WriteMMIO8(MTPS, 0x27);

            /* tx checksum offload disable */
            dev->features &= ~NETIF_F_IP_CSUM;
        } else {
            /* tx checksum offload enable */
            dev->features |= NETIF_F_IP_CSUM;
        }

        csi_tmp = ReadERI(0xD4, 4, ERIAR_ExGMAC);
        csi_tmp  |= ( BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12 );
        WriteERI(0xD4, 4, csi_tmp, ERIAR_ExGMAC);
    } else if (mcfg == MCFG_8168B_1) {
        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        if (dev->mtu > ETH_DATA_LEN) {
            pci_read_config_byte(pdev, 0x69, &device_control);
            device_control &= ~0x70;
            device_control |= 0x28;
            pci_write_config_byte(pdev, 0x69, device_control);
        } else {
            pci_read_config_byte(pdev, 0x69, &device_control);
            device_control &= ~0x70;
            device_control |= 0x58;
            pci_write_config_byte(pdev, 0x69, device_control);
        }
    } else if (mcfg == MCFG_8168B_2) {
        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        if (dev->mtu > ETH_DATA_LEN) {
            pci_read_config_byte(pdev, 0x69, &device_control);
            device_control &= ~0x70;
            device_control |= 0x28;
            pci_write_config_byte(pdev, 0x69, device_control);

            WriteMMIO8(Config4, ReadMMIO8(Config4) | (1 << 0));
        } else {
            pci_read_config_byte(pdev, 0x69, &device_control);
            device_control &= ~0x70;
            device_control |= 0x58;
            pci_write_config_byte(pdev, 0x69, device_control);

            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~(1 << 0));
        }
    } else if (mcfg == MCFG_8168B_3) {
        WriteMMIO8(Config3, ReadMMIO8(Config3) & ~Beacon_en);

        WriteMMIO16(CPlusCmd, ReadMMIO16(CPlusCmd) &
                ~(EnableBist | Macdbgo_oe | Force_halfdup | Force_rxflow_en | Force_txflow_en |
                  Cxpl_dbg_sel | ASF | PktCntrDisable | Macdbgo_sel));

        if (dev->mtu > ETH_DATA_LEN) {
            pci_read_config_byte(pdev, 0x69, &device_control);
            device_control &= ~0x70;
            device_control |= 0x28;
            pci_write_config_byte(pdev, 0x69, device_control);

            WriteMMIO8(Config4, ReadMMIO8(Config4) | (1 << 0));
        } else {
            pci_read_config_byte(pdev, 0x69, &device_control);
            device_control &= ~0x70;
            device_control |= 0x58;
            pci_write_config_byte(pdev, 0x69, device_control);

            WriteMMIO8(Config4, ReadMMIO8(Config4) & ~(1 << 0));
        }
    } else if (mcfg == CFG_METHOD_DEFAULT) {
        cp_cmd &= 0x2043;

        dev->features &= ~NETIF_F_IP_CSUM;
    }

    if ((mcfg == MCFG_8168B_1) || (mcfg == MCFG_8168B_2) || (mcfg == MCFG_8168B_3)) {
        /* csum offload command for RTL8168B/8111B */
        tp->tx_tcp_csum_cmd = TxIPCS | TxTCPCS;
        tp->tx_udp_csum_cmd = TxIPCS | TxUDPCS;
        tp->tx_ip_csum_cmd = TxIPCS;
    } else {
        /* csum offload command for RTL8168C/8111C and RTL8168CP/8111CP */
        tp->tx_tcp_csum_cmd = TxIPCS_C | TxTCPCS_C;
        tp->tx_udp_csum_cmd = TxIPCS_C | TxUDPCS_C;
        tp->tx_ip_csum_cmd = TxIPCS_C;
    }


    //other hw parameters
    if (mcfg == CFG_METHOD_21 || mcfg == CFG_METHOD_22 ||
        mcfg == CFG_METHOD_23 || mcfg == CFG_METHOD_24 ||
        mcfg == CFG_METHOD_25 || mcfg == MCFG_8411B ||
        mcfg == CFG_METHOD_27)
        WriteERI(0x2F8, 2, 0x1D8F, ERIAR_ExGMAC);

    if (bios_setting & BIT_28) {
        if (mcfg == MCFG_8168F_1 || mcfg == MCFG_8168F_2 ||
            mcfg == MCFG_8411_1) {
            u32 gphy_val;

            spin_lock_irqsave(&tp->phy_lock, flags);
            WriteGMII16( 0x1F, 0x0007);
            WriteGMII16( 0x1E, 0x002C);
            gphy_val = ReadGMII16( 0x16);
            gphy_val |= BIT_10;
            WriteGMII16( 0x16, gphy_val);
            WriteGMII16( 0x1F, 0x0005);
            WriteGMII16( 0x05, 0x8B80);
            gphy_val = ReadGMII16( 0x06);
            gphy_val |= BIT_7;
            WriteGMII16( 0x06, gphy_val);
            WriteGMII16( 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
    }

    switch (mcfg) {
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        if (aspm) {
            rtl8168_init_pci_offset_99(tp);
        }
        break;
    }
    switch (mcfg) {
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        if (aspm) {
            rtl8168_init_pci_offset_180(tp);
        }
        break;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    WriteMMIO16(CPlusCmd, cp_cmd);
#else
    rtl8168_hw_set_features(dev, dev->features);
#endif

    switch (mcfg) {
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27: {
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

    rtl8168_disable_rxdvgate(dev);

    if (mcfg == MCFG_8168DP_1 || mcfg == MCFG_8168DP_2)
        rtl8168_mac_loopback_test(tp);

    if (!tp->pci_cfg_is_read) {
        pci_read_config_byte(pdev, PCI_COMMAND, &tp->pci_cfg_space.cmd);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &tp->pci_cfg_space.io_base_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &tp->pci_cfg_space.io_base_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &tp->pci_cfg_space.mem_base_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &tp->pci_cfg_space.mem_base_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3, &tp->pci_cfg_space.resv_0x1c_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_3 + 2, &tp->pci_cfg_space.resv_0x1c_h);
        pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &tp->pci_cfg_space.ilr);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &tp->pci_cfg_space.resv_0x20_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &tp->pci_cfg_space.resv_0x20_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &tp->pci_cfg_space.resv_0x24_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &tp->pci_cfg_space.resv_0x24_h);
        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &tp->pci_cfg_space.resv_0x2c_l);
        pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID + 2, &tp->pci_cfg_space.resv_0x2c_h);
        tp->pci_cfg_space.pci_nvidia_geforce_6200 = rtl8168_csi_read(tp, PCI_DEVICE_ID_NVIDIA_GEFORCE_GO_6200);

        tp->pci_cfg_is_read = 1;
    }

    rtl8168_dsm(dev, DSM_MAC_INIT);

    /* Set Rx packet filter */
    rtl8168_hw_set_rx_packet_filter(dev);

    switch (mcfg) {
    case MCFG_8168E_1:
    case MCFG_8168E_2:
    case MCFG_8168E_VL_1:
    case MCFG_8168E_VL_2:
    case MCFG_8168F_1:
    case MCFG_8168F_2:
    case MCFG_8411_1:
    case CFG_METHOD_21:
    case CFG_METHOD_22:
    case CFG_METHOD_23:
    case CFG_METHOD_24:
    case CFG_METHOD_25:
    case MCFG_8411B:
    case CFG_METHOD_27:
        if (aspm) {
            WriteMMIO8(Config5, ReadMMIO8(Config5) | BIT_0);
            WriteMMIO8(Config2, ReadMMIO8(Config2) | BIT_7);
        } else {
            WriteMMIO8(Config2, ReadMMIO8(Config2) & ~BIT_7);
            WriteMMIO8(Config5, ReadMMIO8(Config5) & ~BIT_0);
        }
        break;
    }

    WriteMMIO8(Cfg9346, Cfg9346_Lock);

    if (!tp->in_open_fun) {
        WriteMMIO8(ChipCmd, CmdTxEnb | CmdRxEnb);

        if (tp->rx_fifo_overflow == 0) {
            /* Enable all known interrupts by setting the interrupt mask. */
            WriteMMIO16(IntrMask, tp->intr_mask);
            netif_start_queue(dev);
        }
    }

    IODelay(10);
}


static int
rtl8168_change_mtu(struct net_device *dev,
                   int new_mtu)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    int max_mtu;
    int ret = 0;
    unsigned long flags;

    if (mcfg == CFG_METHOD_DEFAULT)
        max_mtu = ETH_DATA_LEN;
    else
        max_mtu = tp->max_jumbo_frame_size - ETH_HLEN - 8;

    if (new_mtu < ETH_ZLEN)
        return -EINVAL;
    else if (new_mtu > max_mtu)
        new_mtu = max_mtu;

    if (!netif_running(dev))
        goto out;

    rtl8168_down(dev);

    spin_lock_irqsave(&tp->lock, flags);

    dev->mtu = new_mtu;

    rtl8168_set_rxbufsize(tp, dev);

    ret = rtl8168_init_ring(dev);

    if (ret < 0) {
        spin_unlock_irqrestore(&tp->lock, flags);
        goto out;
    }

#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI		

    rtl8168_hw_start(dev);
    spin_unlock_irqrestore(&tp->lock, flags);
    rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);

    mod_timer(&tp->esd_timer, jiffies + RTL8168_ESD_TIMEOUT);
    mod_timer(&tp->link_timer, jiffies + RTL8168_LINK_TIMEOUT);

out:
    return ret;
}

static inline void
rtl8168_make_unusable_by_asic(struct RxDesc *desc)
{
    desc->addr = 0x0badbadbadbadbadull;
    desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

static void
rtl8168_free_rx_skb(struct rtl8168_private *tp,
                    struct sk_buff **sk_buff,
                    struct RxDesc *desc)
{
    struct pci_dev *pdev = tp->pci_dev;

    pci_unmap_single(pdev, le64_to_cpu(desc->addr), tp->rx_buf_sz,
                     PCI_DMA_FROMDEVICE);
    dev_kfree_skb(*sk_buff);
    *sk_buff = NULL;
    rtl8168_make_unusable_by_asic(desc);
}

static inline void
rtl8168_mark_to_asic(struct RxDesc *desc,
                     u32 rx_buf_sz)
{
    u32 eor = le32_to_cpu(desc->opts1) & RingEnd;

    desc->opts1 = cpu_to_le32(DescOwn | eor | rx_buf_sz);
}

static inline void
rtl8168_map_to_asic(struct RxDesc *desc,
                    dma_addr_t mapping,
                    u32 rx_buf_sz)
{
    desc->addr = cpu_to_le64(mapping);
    wmb();
    rtl8168_mark_to_asic(desc, rx_buf_sz);
}

static int
rtl8168_alloc_rx_skb(struct pci_dev *pdev,
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

    rtl8168_map_to_asic(desc, mapping, rx_buf_sz);

out:
    return ret;

err_out:
    ret = -ENOMEM;
    rtl8168_make_unusable_by_asic(desc);
    goto out;
}

static void
rtl8168_rx_clear(struct rtl8168_private *tp)
{
    int i;

    for (i = 0; i < NUM_RX_DESC; i++) {
        if (tp->Rx_skbuff[i])
            rtl8168_free_rx_skb(tp, tp->Rx_skbuff + i,
                                tp->RxDescArray + i);
    }
}

static u32
rtl8168_rx_fill(struct rtl8168_private *tp,
                struct net_device *dev,
                u32 start,
                u32 end)
{
    u32 cur;

    for (cur = start; end - cur > 0; cur++) {
        int ret, i = cur % NUM_RX_DESC;

        if (tp->Rx_skbuff[i])
            continue;

        ret = rtl8168_alloc_rx_skb(tp->pci_dev, tp->Rx_skbuff + i,
                                   tp->RxDescArray + i, tp->rx_buf_sz);
        if (ret < 0)
            break;
    }
    return cur - start;
}

static inline void
rtl8168_mark_as_last_descriptor(struct RxDesc *desc)
{
    desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8168_desc_addr_fill(struct rtl8168_private *tp)
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
rtl8168_tx_desc_init(struct rtl8168_private *tp)
{
    int i = 0;

    memset(tp->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));

    for (i = 0; i < NUM_TX_DESC; i++) {
        if (i == (NUM_TX_DESC - 1))
            tp->TxDescArray[i].opts1 = cpu_to_le32(RingEnd);
    }
}

static void
rtl8168_rx_desc_offset0_init(struct rtl8168_private *tp, int own)
{
    int i = 0;
    int ownbit = 0;

    if (own)
        ownbit = DescOwn;

    for (i = 0; i < NUM_RX_DESC; i++) {
        if (i == (NUM_RX_DESC - 1))
            tp->RxDescArray[i].opts1 = cpu_to_le32((ownbit | RingEnd) | (unsigned long)tp->rx_buf_sz);
        else
            tp->RxDescArray[i].opts1 = cpu_to_le32(ownbit | (unsigned long)tp->rx_buf_sz);
    }
}

static void
rtl8168_rx_desc_init(struct rtl8168_private *tp)
{
    memset(tp->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));
}

static int
rtl8168_init_ring(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    rtl8168_init_ring_indexes(tp);

    memset(tp->tx_skb, 0x0, NUM_TX_DESC * sizeof(struct ring_info));
    memset(tp->Rx_skbuff, 0x0, NUM_RX_DESC * sizeof(struct sk_buff *));

    rtl8168_tx_desc_init(tp);
    rtl8168_rx_desc_init(tp);

    if (rtl8168_rx_fill(tp, dev, 0, NUM_RX_DESC) != NUM_RX_DESC)
        goto err_out;

    rtl8168_mark_as_last_descriptor(tp->RxDescArray + NUM_RX_DESC - 1);

    return 0;

err_out:
    rtl8168_rx_clear(tp);
    return -ENOMEM;
}

static void
rtl8168_unmap_tx_skb(struct pci_dev *pdev,
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
rtl8168_tx_clear(struct rtl8168_private *tp)
{
    unsigned int i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
    struct net_device *dev = tp->dev;
#endif

    for (i = tp->dirty_tx; i < tp->dirty_tx + NUM_TX_DESC; i++) {
        unsigned int entry = i % NUM_TX_DESC;
        struct ring_info *tx_skb = tp->tx_skb + entry;
        unsigned int len = tx_skb->len;

        if (len) {
            struct sk_buff *skb = tx_skb->skb;

            rtl8168_unmap_tx_skb(tp->pci_dev, tx_skb,
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
static void rtl8168_schedule_work(struct net_device *dev, void (*task)(void *))
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    struct rtl8168_private *tp = netdev_priv(dev);

    PREPARE_WORK(&tp->task, task, dev);
    schedule_delayed_work(&tp->task, 4);
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
}
#else
static void rtl8168_schedule_work(struct net_device *dev, work_func_t task)
{
    struct rtl8168_private *tp = netdev_priv(dev);

    PREPARE_DELAYED_WORK(&tp->task, task);
    schedule_delayed_work(&tp->task, 4);
}
#endif

static void
rtl8168_wait_for_quiescence(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    synchronize_irq(dev->irq);

    /* Wait for any pending NAPI task to complete */
#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_DISABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI

    rtl8168_irq_mask_and_ack(ioaddr);

#ifdef CONFIG_R8168_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif//CONFIG_R8168_NAPI
}

#if 0
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8168_reinit_task(void *_data)
#else
static void rtl8168_reinit_task(struct work_struct *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    struct net_device *dev = _data;
#else
    struct rtl8168_private *tp =
        container_of(work, struct rtl8168_private, task.work);
    struct net_device *dev = tp->dev;
#endif
    int ret;

    if (netif_running(dev)) {
        rtl8168_wait_for_quiescence(dev);
        rtl8168_close(dev);
    }

    ret = rtl8168_open(dev);
    if (unlikely(ret < 0)) {
        if (net_ratelimit()) {
            struct rtl8168_private *tp = netdev_priv(dev);

            if (netif_msg_drv(tp)) {
                printk(PFX KERN_ERR
                       "%s: reinit failure (status = %d)."
                       " Rescheduling.\n", dev->name, ret);
            }
        }
        rtl8168_schedule_work(dev, rtl8168_reinit_task);
    }
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8168_reset_task(void *_data)
{
    struct net_device *dev = _data;
    struct rtl8168_private *tp = netdev_priv(dev);
#else
static void rtl8168_reset_task(struct work_struct *work)
{
    struct rtl8168_private *tp =
        container_of(work, struct rtl8168_private, task.work);
    struct net_device *dev = tp->dev;
#endif
    unsigned long flags;

    if (!netif_running(dev))
        return;

    rtl8168_wait_for_quiescence(dev);

    rtl8168_rx_interrupt(dev, tp, tp->mmio_addr, ~(u32)0);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8168_tx_clear(tp);

    if (tp->dirty_rx == tp->cur_rx) {
        rtl8168_init_ring_indexes(tp);
        rtl8168_hw_start(dev);
        rtl8168_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
        netif_wake_queue(dev);
        spin_unlock_irqrestore(&tp->lock, flags);
    } else {
        spin_unlock_irqrestore(&tp->lock, flags);
        if (net_ratelimit()) {
            struct rtl8168_private *tp = netdev_priv(dev);

            if (netif_msg_intr(tp)) {
                printk(PFX KERN_EMERG
                       "%s: Rx buffers shortage\n", dev->name);
            }
        }
        rtl8168_schedule_work(dev, rtl8168_reset_task);
    }
}

static void
rtl8168_tx_timeout(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    spin_lock_irqsave(&tp->lock, flags);
    rtl8168_hw_reset(dev);
    spin_unlock_irqrestore(&tp->lock, flags);

    /* Let's wait a bit while any (async) irq lands on */
    rtl8168_schedule_work(dev, rtl8168_reset_task);
}

static int
rtl8168_xmit_frags(struct rtl8168_private *tp,
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
rtl8168_tx_csum(struct sk_buff *skb,
                struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
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
        WARN_ON(1); /* we need a WARN() */
#endif
    }

    return 0;
}

static int
rtl8168_start_xmit(struct sk_buff *skb,
                   struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
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

    //Work around for rx fifo overflow
    if (tp->rx_fifo_overflow == 1)
        goto err_stop;

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
    opts2 = rtl8168_tx_vlan_tag(tp, skb);

    large_send = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    if (dev->features & NETIF_F_TSO) {
        u32 mss = skb_is_gso(skb);

        /* TCP Segmentation Offload (or TCP Large Send) */
        if (mss) {
            if ((mcfg == MCFG_8168B_1) ||
                (mcfg == MCFG_8168B_2) ||
                (mcfg == MCFG_8168B_3)) {
                opts1 |= LargeSend | ((mss & MSSMask) << 16);
            } else if ((mcfg == MCFG_8168DP_1) ||
                       (mcfg == MCFG_8168DP_2) ||
                       (mcfg == MCFG_8168DP_3)) {
                opts2 |= LargeSend_DP | ((mss & MSSMask) << 18);
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
            if ((mcfg == MCFG_8168B_1) || (mcfg == MCFG_8168B_2) || (mcfg == MCFG_8168B_3))
                opts1 |= rtl8168_tx_csum(skb, dev);
            else
                opts2 |= rtl8168_tx_csum(skb, dev);
        }
    }

    frags = rtl8168_xmit_frags(tp, skb, opts1, opts2);
    if (frags) {
        len = skb_headlen(skb);
        opts1 |= FirstFrag;
    } else {
        len = skb->len;

        if ((mcfg == MCFG_8168E_VL_1|| mcfg == MCFG_8168E_VL_2)&& len < 60) {
            if (opts2 & 0xE0000000) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
                skb_checksum_help(skb, 0);
#else
                skb_checksum_help(skb);
#endif
                opts2 &= ~0xE0000000;
            }
            len = 60;
        }
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
        smp_rmb();
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
rtl8168_tx_interrupt(struct net_device *dev,
                     struct rtl8168_private *tp,
                     void __iomem *ioaddr)
{
    unsigned int dirty_tx, tx_left;

    assert(dev != NULL);
    assert(tp != NULL);
    assert(ioaddr != NULL);

    dirty_tx = tp->dirty_tx;
    smp_rmb();
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

        rtl8168_unmap_tx_skb(tp->pci_dev,
                             tx_skb,
                             tp->TxDescArray + entry);

        if (tx_skb->skb!=NULL) {
            dev_kfree_skb_irq(tx_skb->skb);
            tx_skb->skb = NULL;
        }
        dirty_tx++;
        tx_left--;
    }

    if (tp->dirty_tx != dirty_tx) {
        tp->dirty_tx = dirty_tx;
        smp_wmb();
        if (netif_queue_stopped(dev) &&
            (TX_BUFFS_AVAIL(tp) >= MAX_SKB_FRAGS)) {
            netif_wake_queue(dev);
        }
        smp_rmb();
        if (tp->cur_tx != dirty_tx)
            WriteMMIO8(TxPoll, NPQ);
    }
}

static inline int
rtl8168_fragmented_frame(u32 status)
{
    return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

static inline void
rtl8168_rx_csum(struct rtl8168_private *tp,
                struct sk_buff *skb,
                struct RxDesc *desc)
{
    u32 opts1 = le32_to_cpu(desc->opts1);
    u32 opts2 = le32_to_cpu(desc->opts2);
    u32 status = opts1 & RxProtoMask;

    if ((mcfg == MCFG_8168B_1) ||
        (mcfg == MCFG_8168B_2) ||
        (mcfg == MCFG_8168B_3)) {
        /* rx csum offload for RTL8168B/8111B */
        if (((status == RxProtoTCP) && !(opts1 & RxTCPF)) ||
            ((status == RxProtoUDP) && !(opts1 & RxUDPF)) ||
            ((status == RxProtoIP) && !(opts1 & RxIPF)))
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
            skb->ip_summed = CHECKSUM_NONE;
    } else {
        /* rx csum offload for RTL8168C/8111C and RTL8168CP/8111CP */
        if (((status == RxTCPT) && !(opts1 & RxTCPF)) ||
            ((status == RxUDPT) && !(opts1 & RxUDPF)) ||
            ((status == 0) && (opts2 & RxV4F) && !(opts1 & RxIPF)))
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
            skb->ip_summed = CHECKSUM_NONE;
    }
}

static inline int
rtl8168_try_rx_copy(struct sk_buff **sk_buff,
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
            rtl8168_mark_to_asic(desc, rx_buf_sz);
            ret = 0;
        }
    }
    return ret;
}

static inline void
rtl8168_rx_skb(struct rtl8168_private *tp,
               struct sk_buff *skb)
{
#ifdef CONFIG_R8168_NAPI
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
rtl8168_rx_interrupt(struct net_device *dev,
                     struct rtl8168_private *tp,
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
    rx_left = rtl8168_rx_quota(rx_left, (u32) rx_quota);

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
            rtl8168_mark_to_asic(desc, tp->rx_buf_sz);
        } else {
            struct sk_buff *skb = tp->Rx_skbuff[entry];
            int pkt_size = (status & 0x00003FFF) - 4;
            void (*pci_action)(struct pci_dev *, dma_addr_t,
                               size_t, int) = pci_dma_sync_single_for_device;

            /*
             * The driver does not support incoming fragmented
             * frames. They are seen as a symptom of over-mtu
             * sized frames.
             */
            if (unlikely(rtl8168_fragmented_frame(status))) {
                RTLDEV->stats.rx_dropped++;
                RTLDEV->stats.rx_length_errors++;
                rtl8168_mark_to_asic(desc, tp->rx_buf_sz);
                continue;
            }

            if (cp_cmd & RxChkSum)
                rtl8168_rx_csum(tp, skb, desc);

            pci_dma_sync_single_for_cpu(tp->pci_dev,
                                        le64_to_cpu(desc->addr), tp->rx_buf_sz,
                                        PCI_DMA_FROMDEVICE);

            if (rtl8168_try_rx_copy(&skb, pkt_size, desc,
                                    tp->rx_buf_sz)) {
                pci_action = pci_unmap_single;
                tp->Rx_skbuff[entry] = NULL;
            }

            pci_action(tp->pci_dev, le64_to_cpu(desc->addr),
                       tp->rx_buf_sz, PCI_DMA_FROMDEVICE);

            skb->dev = dev;
            skb_put(skb, pkt_size);
            skb->protocol = eth_type_trans(skb, dev);

            if (rtl8168_rx_vlan_skb(tp, desc, skb) < 0)
                rtl8168_rx_skb(tp, skb);

            dev->last_rx = jiffies;
            RTLDEV->stats.rx_bytes += pkt_size;
            RTLDEV->stats.rx_packets++;
        }
    }

    count = cur_rx - tp->cur_rx;
    tp->cur_rx = cur_rx;

    delta = rtl8168_rx_fill(tp, dev, tp->dirty_rx, tp->cur_rx);
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
rtl8168_switch_to_hw_interrupt(struct rtl8168_private *tp, void __iomem *ioaddr)
{
    WriteMMIO32(TimeIntr, 0x0000);
    WriteMMIO16(IntrMask, tp->intr_mask);
}

static inline void
rtl8168_switch_to_timer_interrupt(struct rtl8168_private *tp, void __iomem *ioaddr)
{
    if (tp->use_timer_interrrupt) {
        WriteMMIO32(TCTR, timer_count);
        WriteMMIO32(TimeIntr, timer_count);
        WriteMMIO16(IntrMask, PCSTimeout);
    } else {
        rtl8168_switch_to_hw_interrupt(tp, ioaddr);
    }
}

/*
 *The interrupt handler does all of the Rx thread work and cleans up after
 *the Tx thread.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
#else
static irqreturn_t rtl8168_interrupt(int irq, void *dev_instance)
#endif
{
    struct net_device *dev = (struct net_device *) dev_instance;
    struct rtl8168_private *tp = netdev_priv(dev);
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

        switch (mcfg) {
        case MCFG_8168D_1:
        case MCFG_8168D_2:
        case MCFG_8168DP_1:
        case MCFG_8168DP_2:
        case MCFG_8168DP_3:
        case MCFG_8168E_1:
        case MCFG_8168E_2:
        case MCFG_8168E_VL_1:
        case MCFG_8168E_VL_2:
        case MCFG_8168F_1:
        case MCFG_8168F_2:
        case MCFG_8411_1:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_23:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case MCFG_8411B:
        case CFG_METHOD_27:
            /* RX_OVERFLOW RE-START mechanism now HW handles it automatically*/
            WriteMMIO16(IntrStatus, status&~RxFIFOOver);
            break;
        default:
            WriteMMIO16(IntrStatus, status);
            break;
        }

        //Work around for rx fifo overflow
        if (unlikely(status & RxFIFOOver))
            if (mcfg == MCFG_8168B_1) {
                tp->rx_fifo_overflow = 1;
                netif_stop_queue(dev);
                IODelay(300);
                rtl8168_rx_clear(tp);
                rtl8168_init_ring(dev);
                rtl8168_hw_start(dev);
                WriteMMIO16(IntrStatus, RxFIFOOver);
                netif_wake_queue(dev);
                tp->rx_fifo_overflow = 0;
            }

#ifdef CONFIG_R8168_NAPI
        if (status & tp->intr_mask) {
            if (likely(RTL_NETIF_RX_SCHEDULE_PREP(dev, &tp->napi)))
                __RTL_NETIF_RX_SCHEDULE(dev, &tp->napi);
            else if (netif_msg_intr(tp))
                printk(KERN_INFO "%s: interrupt %04x in poll\n",
                       dev->name, status);
        } else
            rtl8168_switch_to_hw_interrupt(tp, ioaddr);
#else
        if (status & tp->intr_mask) {
            rtl8168_rx_interrupt(dev, tp, tp->mmio_addr, ~(u32)0);
            rtl8168_tx_interrupt(dev, tp, ioaddr);

            rtl8168_switch_to_timer_interrupt(tp, ioaddr);
        } else
            rtl8168_switch_to_hw_interrupt(tp, ioaddr);
#endif

    } while (false);

    return IRQ_RETVAL(handled);
}

#ifdef CONFIG_R8168_NAPI
static int rtl8168_poll(napi_ptr napi, napi_budget budget)
{
    struct rtl8168_private *tp = RTL_GET_PRIV(napi, struct rtl8168_private);
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_GET_NETDEV(tp)
    unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
    unsigned int work_done;
    unsigned long flags;



    work_done = rtl8168_rx_interrupt(dev, tp, ioaddr, (u32) budget);

    spin_lock_irqsave(&tp->lock, flags);
    rtl8168_tx_interrupt(dev, tp, ioaddr);
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

        rtl8168_switch_to_timer_interrupt(tp, ioaddr);
    }

    return RTL_NAPI_RETURN_VALUE;
}
#endif//CONFIG_R8168_NAPI

static void rtl8168_sleep_rx_enable(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;

    if ((mcfg == MCFG_8168B_1) || (mcfg == MCFG_8168B_2)) {
        WriteMMIO8(ChipCmd, CmdReset);
        rtl8168_rx_desc_offset0_init(tp, 0);
        WriteMMIO8(ChipCmd, CmdRxEnb);
    } else if (mcfg == MCFG_8168E_1 || mcfg == MCFG_8168E_2) {
        rtl8168_ephy_write(ioaddr, 0x19, 0xFF64);
        WriteMMIO32(RxConfig, ReadMMIO32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
    }
}

static void rtl8168_down(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    unsigned long flags;

    rtl8168_delete_esd_timer(dev, &tp->esd_timer);

    rtl8168_delete_link_timer(dev, &tp->link_timer);

#ifdef CONFIG_R8168_NAPI
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

    rtl8168_dsm(dev, DSM_IF_DOWN);

    rtl8168_hw_reset(dev);

    spin_unlock_irqrestore(&tp->lock, flags);

    synchronize_irq(dev->irq);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8168_tx_clear(tp);

    rtl8168_rx_clear(tp);

    rtl8168_sleep_rx_enable(dev);

    spin_unlock_irqrestore(&tp->lock, flags);
}

static int rtl8168_close(struct net_device *dev)
{
    struct rtl8168_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;

    if (tp->TxDescArray!=NULL && tp->RxDescArray!=NULL) {
        rtl8168_down(dev);

        rtl8168_hw_d3_para(dev);

        rtl8168_powerdown_pll(dev);

        free_irq(dev->irq, dev);

        pci_free_consistent(pdev, R8168_RX_RING_BYTES, tp->RxDescArray,
                            tp->RxPhyAddr);
        pci_free_consistent(pdev, R8168_TX_RING_BYTES, tp->TxDescArray,
                            tp->TxPhyAddr);
        tp->TxDescArray = NULL;
        tp->RxDescArray = NULL;

        if (tp->tally_vaddr!=NULL) {
            pci_free_consistent(pdev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
            tp->tally_vaddr = NULL;
        }
    }

    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
static void rtl8168_shutdown(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8168_private *tp = netdev_priv(dev);

    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case CFG_METHOD_23:
    case CFG_METHOD_27:
        rtl8168_driver_stop(tp);
        break;
    }

    rtl8168_set_bios_setting(dev);
    rtl8168_rar_set(tp, tp->org_mac_addr);

    if (s5wol == 0)
        tp->wol_enabled = WOL_DISABLED;

    /*
    if (s5wol) {
        void __iomem *ioaddr = tp->mmio_addr;
        u32 csi_tmp;

        WriteMMIO8(Cfg9346, Cfg9346_Unlock);
        switch (mcfg) {
        case MCFG_8168E_VL_1:
        case MCFG_8168E_VL_2:
        case MCFG_8168F_1:
        case MCFG_8168F_2:
        case MCFG_8411_1:
        case CFG_METHOD_21:
        case CFG_METHOD_22:
        case CFG_METHOD_24:
        case CFG_METHOD_25:
        case MCFG_8411B:
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

    rtl8168_close(dev);
    rtl8168_disable_msi(pdev, tp);
}
#endif

/**
 *  rtl8168_get_stats - Get rtl8168 read/write statistics
 *  @dev: The Ethernet Device to get statistics for
 *
 *  Get TX/RX statistics for rtl8168
 */
static struct
net_device_stats *rtl8168_get_stats(struct net_device *dev) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    struct rtl8168_private *tp = netdev_priv(dev);
#endif
    if (netif_running(dev)) {
//      spin_lock_irqsave(&tp->lock, flags);
//      spin_unlock_irqrestore(&tp->lock, flags);
    }

    return &RTLDEV->stats;
}

#ifdef CONFIG_PM

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int
rtl8168_suspend(struct pci_dev *pdev, u32 state)
#else
static int
rtl8168_suspend(struct pci_dev *pdev, pm_message_t state)
#endif
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8168_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    u32 pci_pm_state = pci_choose_state(pdev, state);
#endif
    unsigned long flags;

    if (!netif_running(dev))
        goto out;

    rtl8168_delete_esd_timer(dev, &tp->esd_timer);

    rtl8168_delete_link_timer(dev, &tp->link_timer);

    netif_stop_queue(dev);

    netif_carrier_off(dev);

    rtl8168_dsm(dev, DSM_NIC_GOTO_D3);

    netif_device_detach(dev);

    spin_lock_irqsave(&tp->lock, flags);

    rtl8168_hw_reset(dev);

    rtl8168_sleep_rx_enable(dev);

    rtl8168_hw_d3_para(dev);

    rtl8168_powerdown_pll(dev);

    spin_unlock_irqrestore(&tp->lock, flags);

    switch (mcfg) {
    case MCFG_8168DP_1:
    case MCFG_8168DP_2:
    case MCFG_8168DP_3:
    case CFG_METHOD_23:
    case CFG_METHOD_27:
        rtl8168_driver_stop(tp);
        break;
    }

out:

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    pci_save_state(pdev, &pci_pm_state);
#else
    pci_save_state(pdev);
#endif
    pci_enable_wake(pdev, pci_choose_state(pdev, state), tp->wol_enabled);
//  pci_set_power_state(pdev, pci_choose_state(pdev, state));

    return 0;
}

static int
rtl8168_resume(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8168_private *tp = netdev_priv(dev);
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
    rtl8168_rar_set(tp, dev->dev_addr);

    if (!netif_running(dev))
        goto out;

    rtl8168_exit_oob(dev);

    rtl8168_rx_desc_offset0_init(tp, 1);

    rtl8168_dsm(dev, DSM_NIC_RESUME_D3);

    rtl8168_hw_init(dev);

    rtl8168_powerup_pll(dev);

    rtl8168_hw_ephy_config(dev);

    rtl8168_hw_phy_config(dev);

    rtl8168_schedule_work(dev, rtl8168_reset_task);

    netif_device_attach(dev);

    mod_timer(&tp->esd_timer, jiffies + RTL8168_ESD_TIMEOUT);
    mod_timer(&tp->link_timer, jiffies + RTL8168_LINK_TIMEOUT);
out:
    return 0;
}

#endif /* CONFIG_PM */

static struct pci_driver rtl8168_pci_driver = {
    .name       = MODULENAME,
    .id_table   = rtl8168_pci_tbl,
    .probe      = rtl8168_init_one,
    .remove     = __devexit_p(rtl8168_remove_one),
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
    .shutdown   = rtl8168_shutdown,
#endif
#ifdef CONFIG_PM
    .suspend    = rtl8168_suspend,
    .resume     = rtl8168_resume,
#endif
};

static int __init
rtl8168_init_module(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    return pci_register_driver(&rtl8168_pci_driver);
#else
    return pci_module_init(&rtl8168_pci_driver);
#endif
}

static void __exit
rtl8168_cleanup_module(void)
{
    pci_unregister_driver(&rtl8168_pci_driver);
}

module_init(rtl8168_init_module);
module_exit(rtl8168_cleanup_module);
