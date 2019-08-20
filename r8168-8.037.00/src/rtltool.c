#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/in.h>
#include <linux/ethtool.h>
#include "r8168.h"
#include "rtl_eeprom.h"
#include "rtltool.h"

static int
OOB_set_ip_mac(struct rtl8168_private *tp, struct sockaddr_in *sa, u8 *mac)
{
    u32 data;

    if (mcfg == MCFG_8168DP_3) {
        OCP_write(tp, 0xF, 0xd0, be32_to_cpu(sa->sin_addr.s_addr));

        memcpy(&data, mac, 4);
        OCP_write(tp, 0xF, 0x00, le32_to_cpu(data));
        data = 0;
        memcpy(&data, mac + 4, 2);
        OCP_write(tp, 0x3, 0x04, le32_to_cpu(data));

        OOB_notify(tp, OOB_CMD_SET_IPMAC);
    } else if (mcfg == MCFG_8168E_VL_2) {
        void __iomem *ioaddr = tp->mmio_addr;
        struct net_device *dev = tp->dev;
        u32 rx_mode;

        rx_mode = ReadMMIO32(RxConfig);
        if (netif_running(dev)) {
            netif_stop_queue(dev);
            WriteMMIO32(RxConfig, rx_mode & ~0x3f);
            while ((ReadMMIO8(0xd3) & (BIT_5 | BIT_4)) != ((BIT_5 | BIT_4)))
                IODelay(20);
            WriteMMIO8(ChipCmd, ReadMMIO8(ChipCmd) & ~(CmdTxEnb | CmdRxEnb));
//		} else {
//			unsigned long flags;
//
//			spin_lock_irqsave(&tp->phy_lock, flags);
//			WriteGMII16( 0x1f, 0x0000);
//			data = ReadGMII16( MII_CTRL1000);
//			data &=	~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
//			WriteGMII16( MII_CTRL1000, data);
//			WriteGMII16( 0x00, 0x9200);
//			spin_unlock_irqrestore(&tp->phy_lock, flags);
//
//			ssleep(3);
//			WriteMMIO16(IntrStatus, ReadMMIO16(IntrStatus));
//
//			WriteMMIO32(MAR0, 0);
//			WriteMMIO32(MAR0 + 4, 0);
//			WriteMMIO16(RxMaxSize, 0x05f3);
        }
        WriteMMIO8(0xD3, ReadMMIO8(0xD3) & ~BIT_7);
        WriteERI(0x180, 4, 0x06080888, ERIAR_ExGMAC);
        WriteERI(0x184, 4, 0xdd860008, ERIAR_ExGMAC);

        memcpy(&data, mac, 2);
        WriteERI(0xf0, 4, (le32_to_cpu(data) << 16), ERIAR_ExGMAC);
        memcpy(&data, mac + 2, 4);
        WriteERI(0xf4, 4, le32_to_cpu(data), ERIAR_ExGMAC);

        WriteERI(0x190, 4, 0x3c110600, ERIAR_ExGMAC);
        WriteERI(0x194, 4, 0x2c32332b, ERIAR_ExGMAC);
        WriteERI(0x198, 4, 0x003a0201, ERIAR_ExGMAC);
        WriteERI(0x19c, 4, 0x00000000, ERIAR_ExGMAC);

        WriteERI(0x1f0, 4, cpu_to_le32(sa->sin_addr.s_addr), ERIAR_ExGMAC);

        memcpy(&data, mac, 4);
        WriteERI(0x258, 4, le32_to_cpu(data), ERIAR_ExGMAC);
        memcpy(&data, mac + 4, 2);
        WriteERI(0x25c, 2, le32_to_cpu(data), ERIAR_ExGMAC);

        WriteMMIO8(0xe0, ReadMMIO8(0xe0) | BIT_6);
        while (!(ReadMMIO8(0xd3) & BIT_1))
            IODelay(20);

        WriteMMIO32(0xb0, 0x9800e035);
        WriteMMIO32(0xb0, 0x9801e034);
        WriteMMIO32(0xb0, 0x9802e019);
        WriteMMIO32(0xb0, 0x98039918);
        WriteMMIO32(0xb0, 0x9804c011);
        WriteMMIO32(0xb0, 0x98057100);
        WriteMMIO32(0xb0, 0x9806499f);
        WriteMMIO32(0xb0, 0x9807f011);
        WriteMMIO32(0xb0, 0x9808c00e);
        WriteMMIO32(0xb0, 0x98097100);
        WriteMMIO32(0xb0, 0x980A4995);
        WriteMMIO32(0xb0, 0x980Bf00d);
        WriteMMIO32(0xb0, 0x980C4895);
        WriteMMIO32(0xb0, 0x980D9900);
        WriteMMIO32(0xb0, 0x980Ec009);
        WriteMMIO32(0xb0, 0x980F7100);
        WriteMMIO32(0xb0, 0x98104890);
        WriteMMIO32(0xb0, 0x98119900);
        WriteMMIO32(0xb0, 0x98124810);
        WriteMMIO32(0xb0, 0x98139900);
        WriteMMIO32(0xb0, 0x9814e004);
        WriteMMIO32(0xb0, 0x9815d44e);
        WriteMMIO32(0xb0, 0x9816d506);
        WriteMMIO32(0xb0, 0x9817c0b4);
        WriteMMIO32(0xb0, 0x9818c002);
        WriteMMIO32(0xb0, 0x9819b800);
        WriteMMIO32(0xb0, 0x981A0500);
        WriteMMIO32(0xb0, 0x981B1a26);
        WriteMMIO32(0xb0, 0x981Ca4ca);
        WriteMMIO32(0xb0, 0x981D21bc);
        WriteMMIO32(0xb0, 0x981E25bc);
        WriteMMIO32(0xb0, 0x981F1305);
        WriteMMIO32(0xb0, 0x9820f00d);
        WriteMMIO32(0xb0, 0x9821c213);
        WriteMMIO32(0xb0, 0x98227340);
        WriteMMIO32(0xb0, 0x982349b0);
        WriteMMIO32(0xb0, 0x9824f009);
        WriteMMIO32(0xb0, 0x98251a3a);
        WriteMMIO32(0xb0, 0x9826a4ca);
        WriteMMIO32(0xb0, 0x982721b9);
        WriteMMIO32(0xb0, 0x982825b9);
        WriteMMIO32(0xb0, 0x98291303);
        WriteMMIO32(0xb0, 0x982Af006);
        WriteMMIO32(0xb0, 0x982B1309);
        WriteMMIO32(0xb0, 0x982Cf004);
        WriteMMIO32(0xb0, 0x982Dc306);
        WriteMMIO32(0xb0, 0x982E1a26);
        WriteMMIO32(0xb0, 0x982Fbb00);
        WriteMMIO32(0xb0, 0x9830c302);
        WriteMMIO32(0xb0, 0x9831bb00);
        WriteMMIO32(0xb0, 0x98320f3e);
        WriteMMIO32(0xb0, 0x98330f4e);
        WriteMMIO32(0xb0, 0x9834c0ae);
        WriteMMIO32(0xb0, 0x98351800);
        WriteMMIO32(0xb0, 0x9836b800);
        WriteMMIO32(0xb0, 0xfe173000);
        WriteMMIO32(0xb0, 0xfe1604ff);
        WriteMMIO32(0xb0, 0xfe150f4d);
        data = ReadERI(0xd6, 1, ERIAR_ExGMAC);
        WriteERI(0xd6, 1, data | BIT_0, ERIAR_ExGMAC);

        if (netif_running(dev)) {
            rtl8168_init_ring_indexes(tp);
            WriteMMIO8(ChipCmd, CmdRxEnb | CmdTxEnb);
            WriteMMIO32(RxConfig, rx_mode);
            netif_wake_queue(dev);
        } else {
            WriteMMIO8(0xD3, ReadMMIO8(0xD3) | BIT_7);

//			data = ReadERI(0xDC, 1, ERIAR_ExGMAC);
//			data &= ~BIT_0;
//			WriteERI( ioaddr, 0xDC, 1, data, ERIAR_ExGMAC);
//			data |= BIT_0;
//			WriteERI( ioaddr, 0xDC, 1, data, ERIAR_ExGMAC);

            WriteMMIO32(RxConfig, rx_mode | 0x0e);
            WriteERI(0x2F8, 1, 0x0064, ERIAR_ExGMAC);
        }
    } else {
        return -EFAULT;
    }
    return 0;
}

int rtltool_ioctl(struct rtl8168_private *tp, struct ifreq *ifr)
{
    struct rtltool_cmd my_cmd;
    unsigned long flags;
    int ret;

    if (copy_from_user(&my_cmd, ifr->ifr_data, sizeof(my_cmd)))
        return -EFAULT;

    ret = 0;
    switch (my_cmd.cmd) {
    case RTLTOOL_READ_MAC:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        if (my_cmd.len==1)
            my_cmd.data = readb(tp->mmio_addr+my_cmd.offset);
        else if (my_cmd.len==2)
            my_cmd.data = readw(tp->mmio_addr+(my_cmd.offset&~1));
        else if (my_cmd.len==4)
            my_cmd.data = readl(tp->mmio_addr+(my_cmd.offset&~3));
        else {
            ret = -EOPNOTSUPP;
            break;
        }

        if (copy_to_user(ifr->ifr_data, &my_cmd, sizeof(my_cmd))) {
            ret = -EFAULT;
            break;
        }
        break;

    case RTLTOOL_WRITE_MAC:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        if (my_cmd.len==1)
            writeb(my_cmd.data, tp->mmio_addr+my_cmd.offset);
        else if (my_cmd.len==2)
            writew(my_cmd.data, tp->mmio_addr+(my_cmd.offset&~1));
        else if (my_cmd.len==4)
            writel(my_cmd.data, tp->mmio_addr+(my_cmd.offset&~3));
        else {
            ret = -EOPNOTSUPP;
            break;
        }

        break;

    case RTLTOOL_READ_PHY:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        spin_lock_irqsave(&tp->phy_lock, flags);
        my_cmd.data = ReadGMII16( my_cmd.offset);
        spin_unlock_irqrestore(&tp->phy_lock, flags);

        if (copy_to_user(ifr->ifr_data, &my_cmd, sizeof(my_cmd))) {
            ret = -EFAULT;
            break;
        }

        break;

    case RTLTOOL_WRITE_PHY:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        spin_lock_irqsave(&tp->phy_lock, flags);
        WriteGMII16( my_cmd.offset, my_cmd.data);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        break;

    case RTLTOOL_READ_EPHY:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        my_cmd.data = rtl8168_ephy_read(tp->mmio_addr, my_cmd.offset);

        if (copy_to_user(ifr->ifr_data, &my_cmd, sizeof(my_cmd))) {
            ret = -EFAULT;
            break;
        }

        break;

    case RTLTOOL_WRITE_EPHY:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        rtl8168_ephy_write(tp->mmio_addr, my_cmd.offset, my_cmd.data);
        break;

    case RTLTOOL_READ_PCI:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        my_cmd.data = 0;
        if (my_cmd.len==1)
            pci_read_config_byte(tp->pci_dev, my_cmd.offset,
                                 (u8 *)&my_cmd.data);
        else if (my_cmd.len==2)
            pci_read_config_word(tp->pci_dev, my_cmd.offset,
                                 (u16 *)&my_cmd.data);
        else if (my_cmd.len==4)
            pci_read_config_dword(tp->pci_dev, my_cmd.offset,
                                  &my_cmd.data);
        else {
            ret = -EOPNOTSUPP;
            break;
        }

        if (copy_to_user(ifr->ifr_data, &my_cmd, sizeof(my_cmd))) {
            ret = -EFAULT;
            break;
        }
        break;

    case RTLTOOL_WRITE_PCI:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        if (my_cmd.len==1)
            pci_write_config_byte(tp->pci_dev, my_cmd.offset,
                                  my_cmd.data);
        else if (my_cmd.len==2)
            pci_write_config_word(tp->pci_dev, my_cmd.offset,
                                  my_cmd.data);
        else if (my_cmd.len==4)
            pci_write_config_dword(tp->pci_dev, my_cmd.offset,
                                   my_cmd.data);
        else {
            ret = -EOPNOTSUPP;
            break;
        }

        break;

    case RTLTOOL_READ_EEPROM:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        my_cmd.data = rtl_eeprom_read_sc(tp, my_cmd.offset);

        if (copy_to_user(ifr->ifr_data, &my_cmd, sizeof(my_cmd))) {
            ret = -EFAULT;
            break;
        }

        break;

    case RTLTOOL_WRITE_EEPROM:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        rtl_eeprom_write_sc(tp->mmio_addr, my_cmd.offset, my_cmd.data);
        break;

    case RTL_ARP_NS_OFFLOAD:
        break;

    case RTL_SET_OOB_IPMAC:
        ret = OOB_set_ip_mac(tp,
                             (struct sockaddr_in *)&my_cmd.ifru_addr,
                             my_cmd.ifru_hwaddr.sa_data);
        break;

    case RTL_READ_OOB_MAC:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        my_cmd.data = OCP_read(tp, 0xf, my_cmd.offset);

        if (copy_to_user(ifr->ifr_data, &my_cmd, sizeof(my_cmd))) {
            ret = -EFAULT;
            break;
        }
        break;

    case RTL_WRITE_OOB_MAC:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;

        OOB_mutex_lock(tp);
        if (my_cmd.len == 1)
            OCP_write(tp, 0x1, my_cmd.offset, my_cmd.data);
        else if (my_cmd.len == 2)
            OCP_write(tp, 0x3, my_cmd.offset, my_cmd.data);
        else if (my_cmd.len == 4)
            OCP_write(tp, 0xf, my_cmd.offset, my_cmd.data);
        else {
            ret = -EOPNOTSUPP;
        }
        OOB_mutex_unlock(tp);
        break;

    default:
        ret = -EOPNOTSUPP;
        break;
    }

    return ret;
}
