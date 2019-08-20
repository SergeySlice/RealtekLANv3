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
 * $FreeBSD: src/sys/dev/if_rl.c,v 1.38.2.7 2001/07/19 18:33:07 wpaul Exp $
 */

/*
 * RealTek 8129/8139 PCI NIC driver
 *
 * Written by Bill Paul <wpaul@ctr.columbia.edu>
 * Electrical Engineering Department
 * Columbia University, New York City
 */

 /*
 * This driver also support Realtek 8139C+, 8110S/SB/SC, RTL8111B/C/CP/D and RTL8101E/8102E/8103E.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/taskqueue.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <vm/vm.h>              /* for vtophys */
#include <vm/pmap.h>            /* for vtophys */
#include <machine/clock.h>      /* for DELAY */

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>

#include <dev/mii/mii.h>
#include <dev/re/if_rereg.h>
#if OS_VER < VERSION(5,3)
#include <pci/pcireg.h>
#include <pci/pcivar.h>
#include <machine/bus_pio.h>
#include <machine/bus_memio.h>
#else
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <sys/module.h>
#endif

#if OS_VER > VERSION(5,9)
#include <sys/cdefs.h>
#include <sys/endian.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>
#endif


/*
 * Default to using PIO access for this driver. On SMP systems,
 * there appear to be problems with memory mapped mode: it looks like
 * doing too many memory mapped access back to back in rapid succession
 * can hang the bus. I'm inclined to blame this on crummy design/construction
 * on the part of RealTek. Memory mapped mode does appear to work on
 * uniprocessor systems though.
 */
#define RE_USEIOSPACE


#ifndef lint
static const char rcsid[] =
  "$FreeBSD: src/sys/dev/re/if_re.c,v 1.38.2.7 2001/07/19 18:33:07 wpaul Exp $";
#endif

#define EE_SET(x)					\
	CSR_WRITE_1(sc, RE_EECMD,			\
		CSR_READ_1(sc, RE_EECMD) | x)

#define EE_CLR(x)					\
	CSR_WRITE_1(sc, RE_EECMD,			\
		CSR_READ_1(sc, RE_EECMD) & ~x)

#ifdef RE_USEIOSPACE
#define RE_RES			SYS_RES_IOPORT
#define RE_RID			RE_PCI_LOIO
#else
#define RE_RES			SYS_RES_MEMORY
#define RE_RID			RE_PCI_LOMEM
#endif

/*
 * Various supported device vendors/types and their names.
 */
static struct re_type re_devs[] = {
	{ RT_VENDORID, RT_DEVICEID_8169,
		"Realtek PCI GBE Family Controller" },
	{ RT_VENDORID, RT_DEVICEID_8169SC,
		"Realtek PCI GBE Family Controller" },
	{ RT_VENDORID, RT_DEVICEID_8168,
		"Realtek PCIe GBE Family Controller" },
	{ RT_VENDORID, RT_DEVICEID_8136,
		"Realtek PCIe FE Family Controller" },
	{ DLINK_VENDORID, 0x4300,
		"Realtek PCI GBE Family Controller" },
	{ 0, 0, NULL }
};

static int	re_probe			__P((device_t));
static int	re_attach			__P((device_t));
static int	re_detach			__P((device_t));
static int	re_shutdown			__P((device_t));

static void MP_WritePhyUshort			__P((struct re_softc*, u_int8_t, u_int16_t));
static u_int16_t MP_ReadPhyUshort		__P((struct re_softc *,u_int8_t));
static void MP_WriteEPhyUshort			__P((struct re_softc*, u_int8_t, u_int16_t));
static u_int16_t MP_ReadEPhyUshort		__P((struct re_softc *,u_int8_t));
static u_int8_t MP_ReadEfuse			__P((struct re_softc *,u_int16_t));

static void re_8169s_init			__P((struct re_softc *));
static void re_init				__P((void *));
static int 	re_var_init			__P((struct re_softc *));
static void re_reset				__P((struct re_softc *));
static void re_stop				__P((struct re_softc *));

static void re_start				__P((struct ifnet *));
static int re_encap				__P((struct re_softc *, struct mbuf *));
static void WritePacket				__P((struct re_softc *, caddr_t, int, int, int, uint32_t ,uint32_t));
static int CountFreeTxDescNum			__P((struct re_descriptor));
static int CountMbufNum				__P((struct mbuf *, int *));
static void re_txeof				__P((struct re_softc *));

static void	re_rxeof			__P((struct re_softc *));

static void re_intr				__P((void *));
static void re_setmulti				__P((struct re_softc *));
static int	re_ioctl			__P((struct ifnet *, u_long, caddr_t));
static void re_tick				__P((void *));
#if OS_VER < VERSION(7,0)
static void re_watchdog				__P((struct ifnet *));
#endif

static int	re_ifmedia_upd			__P((struct ifnet *));
static void re_ifmedia_sts			__P((struct ifnet *, struct ifmediareq *));

static void re_eeprom_ShiftOutBits		__P((struct re_softc *, int, int));
static u_int16_t re_eeprom_ShiftInBits		__P((struct re_softc *));
static void re_eeprom_EEpromCleanup		__P((struct re_softc *));
static void re_eeprom_getword			__P((struct re_softc *, int, u_int16_t *));
static void re_read_eeprom			__P((struct re_softc *, caddr_t, int, int, int));
static void re_int_task				(void *, int);

static void re_phy_power_up(device_t dev);
static int re_alloc_buf(struct re_softc *);
static void re_release_buf(struct re_softc *);
static void set_rxbufsize(struct re_softc*);
static void re_release_rx_buf(struct re_softc *);
static void re_release_tx_buf(struct re_softc *);
u_int32_t re_eri_read(struct re_softc *, int , int , int);
int re_eri_write(struct re_softc *, int , int , u_int32_t, int);

#if 0
static void re_phy_power_down(device_t dev);
#endif

#define RE_CSUM_FEATURES    (CSUM_IP | CSUM_TCP | CSUM_UDP)

static device_method_t re_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		re_probe),
	DEVMETHOD(device_attach,	re_attach),
	DEVMETHOD(device_detach,	re_detach),
	DEVMETHOD(device_shutdown,	re_shutdown),
	{ 0, 0 }
};

static driver_t re_driver = {
	"re",
	re_methods,
	sizeof(struct re_softc)
};

static devclass_t re_devclass;

DRIVER_MODULE(if_re, pci, re_driver, re_devclass, 0, 0);

static void re_phy_power_up(dev)
	device_t		dev;
{
	struct re_softc		*sc;

	sc = device_get_softc(dev);

	MP_WritePhyUshort(sc, 0x1f, 0x0000);
	MP_WritePhyUshort(sc, 0x00, 0x1000);
	switch (sc->re_type) {
	case MACFG_4:
	case MACFG_5:
	case MACFG_6:
	case MACFG_21:
	case MACFG_22:
	case MACFG_23:
	case MACFG_24:
	case MACFG_25:
	case MACFG_26:
	case MACFG_27:
	case MACFG_28:
	case MACFG_31:
	case MACFG_32:
	case MACFG_33:
		MP_WritePhyUshort(sc, 0x0e, 0x0000);
		break;
//	case MACFG_11:
//	case MACFG_12:
//	case MACFG_13:
//	case MACFG_14:
//	case MACFG_15:
//	case MACFG_17:
	default:
		break;
	};
}

static void re_dma_map_buf(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	union RxDesc *rxptr = arg;

	if (error) {
		rxptr->so0.Frame_Length = 0;
		*((uint64_t *)&rxptr->so0.RxBuffL) = 0;
		return;
	}
//	rxptr->so0.RxBuffL = segs->ds_addr & 0xFFFFFFFF;
	*((uint64_t *)&rxptr->so0.RxBuffL) = htole64(segs->ds_addr);
}

static void re_dma_map_rxdesc(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct re_softc *sc = arg;
	uint32_t ds_addr[2];

	if (error)
		return;

	*((uint64_t *)ds_addr) = htole64(segs->ds_addr);
	CSR_WRITE_4(sc, 0xe4, ds_addr[0]);
	CSR_WRITE_4(sc, 0xe8, ds_addr[1]);
}

static void re_dma_map_txdesc(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct re_softc *sc = arg;
	uint32_t ds_addr[2];

	if (error)
		return;

	*((uint64_t *)ds_addr) = htole64(segs->ds_addr);
	CSR_WRITE_4(sc, 0x20, ds_addr[0]);
	CSR_WRITE_4(sc, 0x24, ds_addr[1]);
}

#if 0
static void re_phy_power_down(dev)
	device_t		dev;
{
	struct re_softc		*sc;

	sc = device_get_softc(dev);

	MP_WritePhyUshort(sc, 0x1f, 0x0000);
	switch (sc->re_type) {
	case MACFG_21:
	case MACFG_22:
	case MACFG_23:
	case MACFG_24:
	case MACFG_25:
	case MACFG_26:
	case MACFG_27:
	case MACFG_28:
	case MACFG_31:
	case MACFG_32:
	case MACFG_33:
		MP_WritePhyUshort(sc, 0x0e, 0x0200);
		MP_WritePhyUshort(sc, 0x00, 0x0800);
		break;
//	case MACFG_4:
//	case MACFG_5:
//	case MACFG_6:
//	case MACFG_11:
//	case MACFG_12:
//	case MACFG_13:
//	case MACFG_14:
//	case MACFG_17:
	default:
		MP_WritePhyUshort(sc, 0x00, 0x0800);
		break;
	}
}
#endif

/*
 * Probe for a RealTek 8129/8139 chip. Check the PCI vendor and device
 * IDs against our list and return a device name if we find a match.
 */
static int re_probe(dev)	/* Search for Realtek NIC chip */
	device_t		dev;
{
	struct re_type		*t; t = re_devs;
	while (t->re_name != NULL) {
		if ((pci_get_vendor(dev) == t->re_vid) &&
		    (pci_get_device(dev) == t->re_did))
		{
			device_set_desc(dev, t->re_name);
			return(0);
		}
		t++;
	}

	return(ENXIO);
}


u_int32_t re_eri_read(struct re_softc *sc, int addr, int len, int type)
{
	int i, val_shift, shift = 0;
	u_int32_t value1 = 0, value2 = 0, mask;

	if (len > 4 || len <= 0)
		return -1;

	while (len > 0) {
		val_shift = addr % ERIAR_Addr_Align;
		addr = addr & ~0x3;

		CSR_WRITE_4(sc,RE_ERIAR,
			ERIAR_Read |
			type << ERIAR_Type_shift |
			ERIAR_ByteEn << ERIAR_ByteEn_shift |
			addr);

		for (i = 0; i < 10; i++) {
			DELAY(100);

			/* Check if the RTL8168 has completed ERI read */
			if (CSR_READ_4(sc,RE_ERIAR) & ERIAR_Flag)
				break;
		}

		if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

		value1 = CSR_READ_4(sc,RE_ERIDR) & mask;
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

int re_eri_write(struct re_softc *sc, int addr, int len, u_int32_t value, int type)
{

	int i, val_shift, shift = 0;
	u_int32_t value1 = 0, mask;

	if (len > 4 || len <= 0)
		return -1;

	while (len > 0) {
		val_shift = addr % ERIAR_Addr_Align;
		addr = addr & ~0x3;

		if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
		else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

		value1 = re_eri_read(sc, addr, 4, type) & ~mask;
		value1 |= ((value << val_shift * 8) >> shift * 8);

		CSR_WRITE_4(sc,RE_ERIDR, value1);
		CSR_WRITE_4(sc,RE_ERIAR,
			ERIAR_Write |
			type << ERIAR_Type_shift |
			ERIAR_ByteEn << ERIAR_ByteEn_shift |
			addr);

		for (i = 0; i < 10; i++) {
			DELAY(100);

			/* Check if the RTL8168 has completed ERI write */
			if (!(CSR_READ_4(sc,RE_ERIAR) & ERIAR_Flag))
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



static void re_release_rx_buf(struct re_softc *sc)
{
	struct ifnet		*ifp;
	int i;
	ifp = RE_GET_IFNET(sc);

	if (sc->re_desc.re_rx_mtag) {
		for (i = 0; i < RE_RX_BUF_NUM; i++)
		{
			if (sc->re_desc.rx_buf[i]!=NULL)
			{
				bus_dmamap_sync(sc->re_desc.re_rx_mtag,
					sc->re_desc.re_rx_dmamap[i],
					BUS_DMASYNC_POSTREAD);
				bus_dmamap_unload(sc->re_desc.re_rx_mtag,
					sc->re_desc.re_rx_dmamap[i]);
				bus_dmamap_destroy(sc->re_desc.re_rx_mtag,
					sc->re_desc.re_rx_dmamap[i]);
				m_freem(sc->re_desc.rx_buf[i]);
				sc->re_desc.rx_buf[i] =NULL;
			}
		}
		bus_dma_tag_destroy(sc->re_desc.re_rx_mtag);
		sc->re_desc.re_rx_mtag =0;
	}

}
static void re_release_tx_buf(struct re_softc *sc)
{
	struct ifnet		*ifp;
	int i;
	ifp = RE_GET_IFNET(sc);

	if (sc->re_desc.re_tx_mtag) {
		for (i = 0; i < RE_TX_BUF_NUM; i++)
		{

				bus_dmamap_destroy(sc->re_desc.re_tx_mtag,
					sc->re_desc.re_tx_dmamap[i]);
				m_freem(sc->re_desc.tx_buf[i]);

		}
		bus_dma_tag_destroy(sc->re_desc.re_tx_mtag);
		sc->re_desc.re_tx_mtag = 0;
	}


}
static void re_release_buf(struct re_softc *sc)
{
	 re_release_rx_buf(sc);
	 re_release_tx_buf(sc);
}



static int re_alloc_buf(struct re_softc *sc)
{
	int error =0;
	int i,size;

	error = bus_dma_tag_create(sc->re_parent_tag, 1, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL,
	    NULL, MCLBYTES* RE_NTXSEGS, RE_NTXSEGS, 4096, 0,
	    NULL, NULL, &sc->re_desc.re_tx_mtag);

	if (error) {
		//device_printf(dev,"re_tx_mtag fail\n");
		//goto fail;
		return error;
	}

	error = bus_dma_tag_create(
			sc->re_parent_tag,
			RE_DESC_ALIGN, 0,		/* alignment, boundary */
			BUS_SPACE_MAXADDR,		/* lowaddr */
			BUS_SPACE_MAXADDR,		/* highaddr */
			NULL, NULL,			/* filter, filterarg */
			sc->re_rx_buf_sz, 1,			/* maxsize,nsegments */
			sc->re_rx_buf_sz,			/* maxsegsize */
			0,				/* flags */
			NULL, NULL,			/* lockfunc, lockarg */
			&sc->re_desc.re_rx_mtag);
	if (error) {
		//device_printf(dev,"re_rx_mtag fail\n");
		//goto fail;
		return error;
	}



	if (sc->max_jumbo_frame_size <= MCLBYTES)
		size = MCLBYTES;
	else if ((sc->max_jumbo_frame_size > MCLBYTES) && (sc->max_jumbo_frame_size <=  MJUMPAGESIZE))
		size = MJUMPAGESIZE;
	else
		size =MJUM9BYTES;
	for (i = 0; i < RE_RX_BUF_NUM; i++) {
		sc->re_desc.rx_buf[i] = m_getjcl	(M_DONTWAIT, MT_DATA, M_PKTHDR, size);
		if (!sc->re_desc.rx_buf[i])
		{
			//device_printf(dev, "m_getcl fail!!!\n");
			error = ENXIO;
			//goto fail;
			return error;
		}

		error = bus_dmamap_create(sc->re_desc.re_rx_mtag, BUS_DMA_NOWAIT, &sc->re_desc.re_rx_dmamap[i]);
		if (error)
		{
			//device_printf(dev, "bus_dmamap_create fail!!!\n");
			//goto fail;
			return error;
		}
	}

	for (i = 0; i < RE_TX_BUF_NUM; i++) {
		error = bus_dmamap_create(sc->re_desc.re_tx_mtag, BUS_DMA_NOWAIT, &sc->re_desc.re_tx_dmamap[i]);
		if (error)
		{
			//device_printf(dev, "bus_dmamap_create fail!!!\n");
			//goto fail;
			return error;
		}
	}

	return 0;
}

static void set_rxbufsize(struct re_softc *sc)
{

	//printf("set size\n");

	struct ifnet		*ifp;
	ifp = RE_GET_IFNET(sc);
	sc->re_rx_buf_sz = (ifp->if_mtu > ETHERMTU) ? (ifp->if_mtu + 36) : MCLBYTES;
	CSR_WRITE_2(sc, RE_RxMaxSize, sc->re_rx_buf_sz);

}
/*
 * Attach the interface. Allocate softc structures, do ifmedia
 * setup and ethernet/BPF attach.
 */
static int re_attach(device_t dev)
{
	/*int			s;*/
	u_char			eaddr[ETHER_ADDR_LEN];
	u_int32_t		command;
	struct re_softc		*sc;
	struct ifnet		*ifp;
	u_int16_t		re_did = 0;
	int			unit, error = 0, rid, i;
//	int			mac_version;
//	int			mode;
//	u_int8_t		data8;

	/*s = splimp();*/

	sc = device_get_softc(dev);
	unit = device_get_unit(dev);
	bzero(sc, sizeof(struct re_softc));
	RE_LOCK_INIT(sc,device_get_nameunit(dev));
	sc->dev = dev;

	sc->driver_detach = 0;

	sc->re_device_id = pci_get_device(dev);
	sc->re_revid = pci_get_revid(dev);
	pci_enable_busmaster(dev);
	/*
	 * Handle power management nonsense.
	 */
	command = pci_read_config(dev, RE_PCI_CAPID, 4) & 0x000000FF;
	if (command == 0x01) {
		command = pci_read_config(dev, RE_PCI_PWRMGMTCTRL, 4);
		if (command & RE_PSTATE_MASK) {
			u_int32_t	iobase, membase, irq;

			/* Save important PCI config data. */
			iobase = pci_read_config(dev, RE_PCI_LOIO, 4);
			membase = pci_read_config(dev, RE_PCI_LOMEM, 4);
			irq = pci_read_config(dev, RE_PCI_INTLINE, 4);

			/* Reset the power state. */
			printf("re%d: chip is is in D%d power mode "
			"-- setting to D0\n", unit, command & RE_PSTATE_MASK);
			command &= 0xFFFFFFFC;
			pci_write_config(dev, RE_PCI_PWRMGMTCTRL, command, 4);

			/* Restore PCI config data. */
			pci_write_config(dev, RE_PCI_LOIO, iobase, 4);
			pci_write_config(dev, RE_PCI_LOMEM, membase, 4);
			pci_write_config(dev, RE_PCI_INTLINE, irq, 4);
		}
	}

	/*
	 * Map control/status registers.
	 */
	command = pci_read_config(dev, PCIR_COMMAND, 4);
	command |= (PCIM_CMD_PORTEN|PCIM_CMD_MEMEN|PCIM_CMD_BUSMASTEREN);
	pci_write_config(dev, PCIR_COMMAND, command, 4);
	command = pci_read_config(dev, PCIR_COMMAND, 4);

#ifdef RE_USEIOSPACE
	if (!(command & PCIM_CMD_PORTEN)) {
		printf("re%d: failed to enable I/O ports!\n", unit);
		error = ENXIO;
		goto fail;
	}
#else
	if (!(command & PCIM_CMD_MEMEN)) {
		printf("re%d: failed to enable memory mapping!\n", unit);
		error = ENXIO;
		goto fail;
	}
#endif

	rid = RE_RID;
	sc->re_res = bus_alloc_resource(dev, RE_RES, &rid,
	    0, ~0, 1, RF_ACTIVE);

	if (sc->re_res == NULL) {
		device_printf(dev,"couldn't map ports/memory\n");
		error = ENXIO;
		goto fail;
	}

	sc->re_btag = rman_get_bustag(sc->re_res);
	sc->re_bhandle = rman_get_bushandle(sc->re_res);

	rid = 0;
	sc->re_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, 0, ~0, 1,
	    RF_SHAREABLE | RF_ACTIVE);

	if (sc->re_irq == NULL) {
		device_printf(dev,"couldn't map interrupt\n");
		error = ENXIO;
		goto fail;
	}

	callout_handle_init(&sc->re_stat_ch);

	switch(CSR_READ_4(sc, RE_TXCFG) & 0xFCF00000) {
		case 0x00800000:
		case 0x04000000:
			sc->re_type = MACFG_3;
			sc->max_jumbo_frame_size =Jumbo_Frame_7k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
			break;
		case 0x10000000:
			sc->re_type = MACFG_4;
			sc->max_jumbo_frame_size = Jumbo_Frame_7k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
			break;
		case 0x18000000:
			sc->re_type = MACFG_5;
			sc->max_jumbo_frame_size = Jumbo_Frame_7k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
			break;
		case 0x98000000:
			sc->re_type = MACFG_6;
			sc->max_jumbo_frame_size = Jumbo_Frame_7k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
			break;
		case 0x34000000:
		case 0xB4000000:
			sc->re_type = MACFG_11;
			sc->max_jumbo_frame_size = ETHERMTU;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34200000:
		case 0xB4200000:
			sc->re_type = MACFG_12;
			sc->max_jumbo_frame_size = ETHERMTU;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34300000:
		case 0xB4300000:
			sc->re_type = MACFG_13;
			sc->max_jumbo_frame_size = ETHERMTU;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34900000:
		case 0x24900000:
			sc->re_type = MACFG_14;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34A00000:
		case 0x24A00000:
			sc->re_type = MACFG_15;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34B00000:
		case 0x24B00000:
			sc->re_type = MACFG_16;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34C00000:
		case 0x24C00000:
			sc->re_type = MACFG_17;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34D00000:
		case 0x24D00000:
			sc->re_type = MACFG_18;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x34E00000:
		case 0x24E00000:
			sc->re_type = MACFG_19;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x30000000:
			sc->re_type = MACFG_21;
			sc->max_jumbo_frame_size = Jumbo_Frame_4k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x38000000:
			sc->re_type = MACFG_22;
			sc->max_jumbo_frame_size = Jumbo_Frame_4k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x38500000:
		case 0xB8500000:
		case 0x38700000:
		case 0xB8700000:
			sc->re_type = MACFG_23;
			sc->max_jumbo_frame_size = Jumbo_Frame_4k;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x3C000000:
			sc->re_type = MACFG_24;
			sc->max_jumbo_frame_size = Jumbo_Frame_6k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
			break;
		case 0x3C200000:
			sc->re_type = MACFG_25;
			sc->max_jumbo_frame_size = Jumbo_Frame_6k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
			break;
		case 0x3C400000:
			sc->re_type = MACFG_26;
			sc->max_jumbo_frame_size = Jumbo_Frame_6k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
			break;
		case 0x3C900000:
			sc->re_type = MACFG_27;
			sc->max_jumbo_frame_size = Jumbo_Frame_6k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
			break;
		case 0x3CB00000:
			sc->re_type = MACFG_28;
			sc->max_jumbo_frame_size = Jumbo_Frame_6k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
			break;
		case 0x28100000:
			sc->re_type = MACFG_31;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x28200000:
			sc->re_type = MACFG_32;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x28300000:
			sc->re_type = MACFG_33;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x2C100000:
			sc->re_type = MACFG_36;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x2C200000:
			sc->re_type = MACFG_37;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x2C800000:
			sc->re_type = MACFG_38;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x2C900000:
			sc->re_type = MACFG_39;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x24000000:
			sc->re_type = MACFG_41;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x40900000:
			sc->re_type = MACFG_42;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x40A00000:
		case 0x40B00000:
		case 0x40C00000:
			sc->re_type = MACFG_43;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;
		case 0x48000000:
			sc->re_type = MACFG_50;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x48100000:
			sc->re_type = MACFG_51;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x48800000:
			sc->re_type = MACFG_52;
			sc->max_jumbo_frame_size = Jumbo_Frame_9k;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
			break;
		case 0x44000000:
			sc->re_type = MACFG_53;
			sc->max_jumbo_frame_size = ETHERMTU;
			sc->re_if_flags = RL_FLAG_DESCV2;
			CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
			break;

		default:
			device_printf(dev,"unknown device\n");
			sc->re_type = MACFG_FF;
			error = ENXIO;
			goto fail;
	}

	/*
	 * Reset the adapter. Only take the lock here as it's needed in
	 * order to call re_reset().
	 */
	RE_LOCK(sc);
	re_reset(sc);
	RE_UNLOCK(sc);

	/* Get station address from the EEPROM. */
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		eaddr[i] = CSR_READ_1(sc, RE_IDR0 + i);

	/*
	 * A RealTek chip was detected. Inform the world.
	 */
	device_printf(dev,"version:1.81\n");
	device_printf(dev,"Ethernet address: %6D\n", eaddr, ":");
	printf("\nThis product is covered by one or more of the following patents: US5,307,459, US5,434,872, US5,732,094, US6,570,884, US6,115,776, and US6,327,625.\n");

	sc->re_unit = unit;

#if OS_VER < VERSION(6,0)
	bcopy(eaddr, (char *)&sc->arpcom.ac_enaddr, ETHER_ADDR_LEN);
#endif

	/*
	 * Now read the exact device type from the EEPROM to find
	 * out if it's an 8129 or 8139.
	 */
	re_read_eeprom(sc, (caddr_t)&re_did, RE_EE_PCI_DID, 1, 0);

	if (sc->re_type == MACFG_3) {	/* Change PCI Latency time*/
		pci_write_config(dev, RE_PCI_LATENCY_TIMER, 0x40, 1);
	}

	error = bus_dma_tag_create(
#if OS_VER < VERSION(7,0)
		NULL,
#else
		bus_get_dma_tag(dev),		/* parent */
#endif
		1, 0,		/* alignment, boundary */
		BUS_SPACE_MAXADDR,		/* lowaddr */
		BUS_SPACE_MAXADDR,		/* highaddr */
		NULL, NULL,			/* filter, filterarg */
		BUS_SPACE_MAXSIZE_32BIT,	/* maxsize */
		0,				/* nsegments */
		BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
		0,				/* flags */
		NULL, NULL,			/* lockfunc, lockarg */
		&sc->re_parent_tag);

	i = roundup2(sizeof(union RxDesc)*RE_RX_BUF_NUM, RE_DESC_ALIGN);
	error = bus_dma_tag_create(
		sc->re_parent_tag,
		RE_DESC_ALIGN, 0,		/* alignment, boundary */
		BUS_SPACE_MAXADDR,		/* lowaddr */
		BUS_SPACE_MAXADDR,		/* highaddr */
		NULL, NULL,			/* filter, filterarg */
		i,				/* maxsize */
		1,				/* nsegments */
		i,				/* maxsegsize */
		0,				/* flags */
		NULL, NULL,			/* lockfunc, lockarg */
		&sc->re_desc.rx_desc_tag);
	if (error) {
		device_printf(dev,"bus_dma_tag_create fail\n");
		goto fail;
	}

	error = bus_dmamem_alloc(sc->re_desc.rx_desc_tag,
			(void**) &sc->re_desc.rx_desc,
			BUS_DMA_WAITOK|BUS_DMA_COHERENT|BUS_DMA_ZERO,
			&sc->re_desc.rx_desc_dmamap);
	if (error) {
		device_printf(dev,"bus_dmamem_alloc fail\n");
		goto fail;
	}

	i = roundup2(sizeof(union TxDesc)*RE_TX_BUF_NUM, RE_DESC_ALIGN);
	error = bus_dma_tag_create(
			sc->re_parent_tag,
			RE_DESC_ALIGN, 0,		/* alignment, boundary */
			BUS_SPACE_MAXADDR,		/* lowaddr */
			BUS_SPACE_MAXADDR,		/* highaddr */
			NULL, NULL,			/* filter, filterarg */
			i,				/* maxsize */
			1,				/* nsegments */
			i,				/* maxsegsize */
			0,				/* flags */
			NULL, NULL,			/* lockfunc, lockarg */
			&sc->re_desc.tx_desc_tag);
	if (error) {
		device_printf(dev,"bus_dma_tag_create fail\n");
		goto fail;
	}

	error = bus_dmamem_alloc(sc->re_desc.tx_desc_tag,
			(void**) &sc->re_desc.tx_desc,
			BUS_DMA_WAITOK|BUS_DMA_COHERENT|BUS_DMA_ZERO,
			&sc->re_desc.tx_desc_dmamap);

	if (error) {
		device_printf(dev,"bus_dmamem_alloc fail\n");
		goto fail;
	}


	sc->re_8169_MacVersion=(CSR_READ_4(sc, RE_TXCFG)&0x7c800000)>>25;		/* Get bit 26~30 	*/
	sc->re_8169_MacVersion|=((CSR_READ_4(sc, RE_TXCFG)&0x00800000)!=0 ? 1:0);	/* Get bit 23 		*/
	DBGPRINT1(sc->re_unit,"8169 Mac Version %d",sc->re_8169_MacVersion);

	/* Rtl8169s single chip detected */
	if (sc->re_8169_MacVersion > 0) {
		sc->re_8169_PhyVersion=(MP_ReadPhyUshort(sc, 0x03)&0x000f);
		DBGPRINT1(sc->re_unit,"8169 Phy Version %d",sc->re_8169_PhyVersion);

		if (sc->re_8169_MacVersion == 1) {
			CSR_WRITE_1(sc, 0x82, 0x01);
			MP_WritePhyUshort(sc, 0x0b, 0x00);
		}

		re_phy_power_up(dev);
		re_8169s_init(sc);
	}
	sc->re_tx_cstag =1;
	sc->re_rx_cstag =1;

#if OS_VER < VERSION(6,0)
	ifp = &sc->arpcom.ac_if;
#else
	ifp = sc->re_ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "can not if_alloc()\n");
		error = ENOSPC;
		goto fail;
	}
#endif
	ifp->if_softc = sc;
#if OS_VER < VERSION(5,3)
	ifp->if_unit = unit;
	ifp->if_name = "re";
#else
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
#endif
	ifp->if_mtu = ETHERMTU;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = re_ioctl;
	ifp->if_output = ether_output;
	ifp->if_start = re_start;
#if OS_VER < VERSION(7,0)
	ifp->if_watchdog = re_watchdog;
#endif
	if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
		ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
	else
		ifp->if_hwassist |= RE_CSUM_FEATURES;

	ifp->if_capabilities = IFCAP_HWCSUM;
	ifp->if_capenable = ifp->if_capabilities;
		CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
	ifp->if_init = re_init;
	/* VLAN capability setup */
	ifp->if_capabilities |= IFCAP_VLAN_MTU | IFCAP_VLAN_HWTAGGING;
	ifp->if_capenable = ifp->if_capabilities;

	set_rxbufsize(sc);
	error =re_alloc_buf(sc);

	if (error) {
		goto fail;
	}
	switch(sc->re_device_id) {
		case RT_DEVICEID_8169:
		case RT_DEVICEID_8169SC:
		case RT_DEVICEID_8168:
			ifp->if_baudrate = 1000000000;
			break;

		default:
			ifp->if_baudrate = 100000000;
			break;
	}
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);

#if OS_VER>=VERSION(7,0)
	TASK_INIT(&sc->re_inttask, 0, re_int_task, sc);
#endif

	/*
	 * Call MI attach routine.
	 */
/*#if OS_VER < VERSION(5, 1)*/
#if OS_VER < VERSION(4,9)
	ether_ifattach(ifp, ETHER_BPF_SUPPORTED);
#else
	ether_ifattach(ifp, eaddr);
#endif


#if OS_VER < VERSION(7,0)
	error = bus_setup_intr(dev, sc->re_irq, INTR_TYPE_NET,
	    re_intr, sc, &sc->re_intrhand);
#else
	error = bus_setup_intr(dev, sc->re_irq, INTR_TYPE_NET|INTR_MPSAFE,
			NULL, re_intr, sc, &sc->re_intrhand);
#endif

	if (error) {
#if OS_VER < VERSION(4,9)
		ether_ifdetach(ifp, ETHER_BPF_SUPPORTED);
#else
		ether_ifdetach(ifp);
#endif
		device_printf(dev,"couldn't set up irq\n");
		goto fail;
	}

	/*
	 * Specify the media types supported by this adapter and register
	 * callbacks to update media and link information
	 */
	ifmedia_init(&sc->media, IFM_IMASK, re_ifmedia_upd, re_ifmedia_sts);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_10_T, 0, NULL);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_10_T | IFM_FDX, 0, NULL);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_100_TX, 0, NULL);
	ifmedia_add(&sc->media, IFM_ETHER | IFM_100_TX | IFM_FDX, 0, NULL);
	switch(sc->re_device_id) {
		case RT_DEVICEID_8169:
		case RT_DEVICEID_8169SC:
		case RT_DEVICEID_8168:
			ifmedia_add(&sc->media, IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
			ifmedia_add(&sc->media, IFM_ETHER | IFM_1000_T, 0, NULL);
			break;

		default:
			break;
	}
	ifmedia_add(&sc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_set(&sc->media, IFM_ETHER | IFM_AUTO);
	sc->media.ifm_media = IFM_ETHER | IFM_AUTO;
	re_ifmedia_upd(ifp);

fail:
	if (error)
		re_detach(dev);

	return(error);
}

static int re_detach(device_t dev)
{
	struct re_softc		*sc;
	struct ifnet		*ifp;
	/*int			s;*/
	int			i;

	/*s = splimp();*/

	sc = device_get_softc(dev);

	if (sc->re_intrhand)
		bus_teardown_intr(dev, sc->re_irq, sc->re_intrhand);
	if (sc->re_irq)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->re_irq);
	if (sc->re_res)
		bus_release_resource(dev, RE_RES, RE_RID, sc->re_res);

	ifp = RE_GET_IFNET(sc);

	/* These should only be active if attach succeeded */
	if (device_is_attached(dev)) {
		RE_LOCK(sc);
		re_stop(sc);
		RE_UNLOCK(sc);
#if OS_VER>=VERSION(7,0)
		taskqueue_drain(taskqueue_fast, &sc->re_inttask);
#endif
#if OS_VER < VERSION(4,9)
		ether_ifdetach(ifp, ETHER_BPF_SUPPORTED);
#else
		ether_ifdetach(ifp);
#endif
	}
	sc->driver_detach = 1;

#if OS_VER>=VERSION(6,0)
	if (ifp)
		if_free(ifp);
#endif
	bus_generic_detach(dev);

	if (sc->re_desc.re_rx_mtag) {
		for (i = 0; i < RE_RX_BUF_NUM; i++)
		{
			if (sc->re_desc.rx_buf[i]!=NULL)
			{
				bus_dmamap_sync(sc->re_desc.re_rx_mtag,
					sc->re_desc.re_rx_dmamap[i],
					BUS_DMASYNC_POSTREAD);
				bus_dmamap_unload(sc->re_desc.re_rx_mtag,
					sc->re_desc.re_rx_dmamap[i]);
				bus_dmamap_destroy(sc->re_desc.re_rx_mtag,
					sc->re_desc.re_rx_dmamap[i]);
				m_freem(sc->re_desc.rx_buf[i]);
				sc->re_desc.rx_buf[i] =NULL;
			}
		}
		bus_dma_tag_destroy(sc->re_desc.re_rx_mtag);
		sc->re_desc.re_rx_mtag =0;
	}

	if (sc->re_desc.re_tx_mtag) {
		for (i = 0; i < RE_TX_BUF_NUM; i++)
		{
			bus_dmamap_destroy(sc->re_desc.re_tx_mtag,
				sc->re_desc.re_tx_dmamap[i]);
		}
		bus_dma_tag_destroy(sc->re_desc.re_tx_mtag);
		sc->re_desc.re_tx_mtag =0;
	}

	if (sc->re_desc.rx_desc_tag) {
		bus_dmamap_sync(sc->re_desc.rx_desc_tag,
			sc->re_desc.rx_desc_dmamap,
			BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->re_desc.rx_desc_tag,
			sc->re_desc.rx_desc_dmamap);
		bus_dmamem_free(sc->re_desc.rx_desc_tag,
			sc->re_desc.rx_desc,
			sc->re_desc.rx_desc_dmamap);
		bus_dma_tag_destroy(sc->re_desc.rx_desc_tag);
	}

	if (sc->re_desc.tx_desc_tag) {
		bus_dmamap_sync(sc->re_desc.tx_desc_tag,
			sc->re_desc.tx_desc_dmamap,
			BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->re_desc.tx_desc_tag,
			sc->re_desc.tx_desc_dmamap);
		bus_dmamem_free(sc->re_desc.tx_desc_tag,
			sc->re_desc.tx_desc,
			sc->re_desc.tx_desc_dmamap);
		bus_dma_tag_destroy(sc->re_desc.tx_desc_tag);
	}

	if (sc->re_parent_tag) {
		bus_dma_tag_destroy(sc->re_parent_tag);
	}

	/*splx(s);*/
	RE_LOCK_DESTROY(sc);

	return(0);
}

/*
 * Stop all chip I/O so that the kernel's probe routines don't
 * get confused by errant DMAs when rebooting.
 */
static int re_shutdown(dev)	/* The same with re_stop(sc) */
	device_t		dev;
{
	struct re_softc		*sc;

	sc = device_get_softc(dev);

	RE_LOCK(sc);
	re_stop(sc);
	RE_UNLOCK(sc);

	return 0;
}

static void re_hw_start(struct re_softc *sc)
{
	struct ifnet		*ifp;
	u_int32_t		macver;
	u_int8_t		data8;
	u_int16_t		data16 = 0;
	u_int32_t		rxcfg = 0, Data32;

	ifp = RE_GET_IFNET(sc);
	RE_LOCK(sc);

	/* Init descriptors. */
	re_var_init(sc);

	/*disable Link Down Power Saving(non-LDPS)*/
	/*CSR_WRITE_1(sc, RE_LDPS, 0x05);*/
	/*ldps= CSR_READ_1(sc, RE_LDPS);*/

	CSR_WRITE_2(sc, RE_CPCR, 0x2060);

	CSR_WRITE_2(sc, RE_IM, 0x5151);

	/*
	 * Enable interrupts.
	 */
	CSR_WRITE_2(sc, RE_IMR, RE_INTRS);


	/* Start RX/TX process. */
	CSR_WRITE_4(sc, RE_MISSEDPKT, 0);

	/* Enable receiver and transmitter. */
/*	CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_TX_ENB|RE_CMD_RX_ENB);*/

	macver = CSR_READ_4(sc, RE_TXCFG) & 0xFC800000;
	CSR_WRITE_1(sc, RE_EECMD, RE_EEMODE_WRITECFG);

	CSR_WRITE_1(sc, 0xec, 0x3f);

	if (macver == 0x00800000) {
		CSR_WRITE_2(sc, RE_CPlusCmd, 0x0063| ((sc->re_type == MACFG_3 && sc->re_8169_MacVersion==1) ? 0x4008:0));
	} else if (macver == 0x04000000) {
		CSR_WRITE_2(sc, RE_CPlusCmd, 0x0063| ((sc->re_type == MACFG_3 && sc->re_8169_MacVersion==1) ? 0x4008:0));
	} else if (macver == 0x10000000) {
		CSR_WRITE_2(sc, RE_CPlusCmd, 0x0063| ((sc->re_type == MACFG_3 && sc->re_8169_MacVersion==1) ? 0x4008:0));
	} else if (macver == 0x18000000) {
		if (CSR_READ_1(sc, RE_CFG2) & 1)
		{
			CSR_WRITE_4(sc, 0x7C, 0x000FFFFF);
		}
		else
		{
			CSR_WRITE_4(sc, 0x7C, 0x000FFF00);
		}
		CSR_WRITE_2(sc, RE_CPlusCmd, 0x0068);
		CSR_WRITE_2(sc, 0xe2, 0x0000);
	} else if (macver == 0x98000000) {
		if (CSR_READ_1(sc, RE_CFG2) & 1)
		{
			CSR_WRITE_4(sc, 0x7C, 0x003FFFFF);
		}
		else
		{
			CSR_WRITE_4(sc, 0x7C, 0x003FFF00);
		}
		CSR_WRITE_2(sc, RE_CPlusCmd, 0x0068);
		CSR_WRITE_2(sc, 0xe2, 0x0000);
	} else if (macver == 0x30000000) {
		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

		if (ifp->if_mtu > ETHERMTU) {
			data8 = pci_read_config(sc->dev, 0x69, 1);
			data8 &= ~0x70;
			data8 |= 0x28;
			pci_write_config(sc->dev, 0x69, data8, 1);
		 } else {
			data8 = pci_read_config(sc->dev, 0x69, 1);
			data8 &= ~0x70;
			data8 |= 0x58;
			pci_write_config(sc->dev, 0x69, data8, 1);
		 }
	} else if (macver == 0x38000000) {
		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

		if (ifp->if_mtu > ETHERMTU) {
			data8 = pci_read_config(sc->dev, 0x69, 1);
			data8 &= ~0x70;
			data8 |= 0x28;
			pci_write_config(sc->dev, 0x69, data8, 1);
			CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | BIT_0);
		 } else {
			data8 = pci_read_config(sc->dev, 0x69, 1);
			data8 &= ~0x70;
			data8 |= 0x58;
			pci_write_config(sc->dev, 0x69, data8, 1);
			CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~ BIT_0);
		 }
	} else if (macver == 0x34000000 || macver == 0xB4000000) {
		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
	} else if (macver == 0x34800000 || macver == 0x24800000) {
		if (pci_read_config(sc->dev, 0x81, 1) == 1)
		{
			CSR_WRITE_1(sc, RE_DBG_reg, 0x98);
			CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | 0x80);
			CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | 0x04);
			pci_write_config(sc->dev, 0x81, 1, 1);
		}

		data8 = pci_read_config(sc->dev, 0x79, 1);
		data8 &= ~0x70;
		data8 |= 0x50;
		pci_write_config(sc->dev, 0x79, data8, 1);

		/*set configuration space offset 0x70f to 0x3f*/
		CSR_WRITE_4(sc, RE_CSIDR, 0x3F000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		CSR_WRITE_2(sc, RE_RxMaxSize, 0x05EF);
		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
		if (sc->re_type == MACFG_14)
		{
			CSR_WRITE_1(sc,RE_CFG1, 0x0f);

			MP_WriteEPhyUshort(sc, 0x03, 0xC2F9);
		}
		else if (sc->re_type == MACFG_15)
		{
			CSR_WRITE_1(sc,RE_CFG1, 0x0f);

			MP_WriteEPhyUshort(sc, 0x01, 0x6FE5);
			MP_WriteEPhyUshort(sc, 0x03, 0x07D9);
		}
		else if (sc->re_type == MACFG_17)
		{
			CSR_WRITE_1(sc, 0xF4, 0x01);
			MP_WriteEPhyUshort(sc, 0x06, 0xAF35);
		}
		else if (sc->re_type == MACFG_18)
		{
			CSR_WRITE_1(sc, 0xF4, 0x01);
			CSR_WRITE_1(sc, 0xF5, CSR_READ_1(sc, 0xF5)|0x04);
			MP_WriteEPhyUshort(sc, 0x19, 0xEC90);
			MP_WriteEPhyUshort(sc, 0x01, 0x6FE5);
			MP_WriteEPhyUshort(sc, 0x03, 0x05D9);
			MP_WriteEPhyUshort(sc, 0x06, 0xAF35);
		}
		else if (sc->re_type == MACFG_19)
		{
			if (pci_read_config(sc->dev, 0x80, 1)&3)
			{
				MP_WriteEPhyUshort(sc, 0x02, 0x011F);
			}
			CSR_WRITE_1(sc, 0xF4, CSR_READ_1(sc, 0xF4)|0x08);
			CSR_WRITE_1(sc, 0xF5, CSR_READ_1(sc, 0xF5)|0x04);
			CSR_WRITE_1(sc, 0xF4, CSR_READ_1(sc, 0xF4));
			MP_WriteEPhyUshort(sc, 0x19, 0xEC90);
			MP_WriteEPhyUshort(sc, 0x01, 0x6FE5);
			MP_WriteEPhyUshort(sc, 0x03, 0x05D9);
			MP_WriteEPhyUshort(sc, 0x06, 0xAF35);
		}
	} else if (macver == 0x3C000000) {
		/*set configuration space offset 0x70f to 0x3f*/
		CSR_WRITE_4(sc, RE_CSIDR, 0x27000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		CSR_WRITE_1(sc, RE_CFG1, CSR_READ_1(sc, RE_CFG1)|0x10);
		//CSR_WRITE_2(sc, RE_RxMaxSize, 0x05F3);
		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
		if (sc->re_type == MACFG_24)
		{
			/*set mac register offset 0xd1 to 0xf8*/
			CSR_WRITE_1(sc, RE_DBG_reg, 0xF8);

			data16 = MP_ReadEPhyUshort(sc, 0x02) & ~0x1800;
			data16 |= 0x1000;
			MP_WriteEPhyUshort(sc, 0x02, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x03) | 0x0002;
			MP_WriteEPhyUshort(sc, 0x03, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x06) & ~0x0080;
			MP_WriteEPhyUshort(sc, 0x06, data16);

			//disable clock request.
			pci_write_config(sc->dev, 0x81, 0, 1);

			if (ifp->if_mtu > ETHERMTU) {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1 << 1)); //Jumbo_en1

				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x20;
				pci_write_config(sc->dev, 0x79, data8, 1);
				ifp->if_capenable &= ~IFCAP_HWCSUM;
				CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;

			 } else {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1 << 1)); //Jumbo_en1
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x50;
				pci_write_config(sc->dev, 0x79, data8, 1);
				if (sc->re_tx_cstag) {
					ifp->if_capenable |= IFCAP_TXCSUM;
					if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
						ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
					else
						ifp->if_hwassist |= RE_CSUM_FEATURES;
				}
				if (sc->re_rx_cstag) {
					ifp->if_capenable |= IFCAP_RXCSUM;
					CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
				}
			 }


		}
		else if (sc->re_type == MACFG_25)
		{
			data16 = MP_ReadEPhyUshort(sc, 0x01) | 0x0001;
			MP_WriteEPhyUshort(sc, 0x01, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x03) & ~0x0620;
			data16 |= 0x0220;
			MP_WriteEPhyUshort(sc, 0x03, data16);

			//disable clock request.
			pci_write_config(sc->dev, 0x81, 0, 1);

			if (ifp->if_mtu > ETHERMTU) {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x20;
				pci_write_config(sc->dev, 0x79, data8, 1);
				ifp->if_capenable &= ~IFCAP_HWCSUM;
				CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;

			 } else {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x50;
				pci_write_config(sc->dev, 0x79, data8, 1);
				if (sc->re_tx_cstag) {
					ifp->if_capenable |= IFCAP_TXCSUM;
					if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
						ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
					else
						ifp->if_hwassist |= RE_CSUM_FEATURES;
				}
				if (sc->re_rx_cstag) {
					ifp->if_capenable |= IFCAP_RXCSUM;
					CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
				}


			 }

		} else if (sc->re_type == MACFG_26) {
				//disable clock request.
			pci_write_config(sc->dev, 0x81, 0, 1);

			if (ifp->if_mtu > ETHERMTU) {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x20;
				pci_write_config(sc->dev, 0x79, data8, 1);
				ifp->if_capenable &= ~IFCAP_HWCSUM;
				CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;
			 } else {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x50;
				pci_write_config(sc->dev, 0x79, data8, 1);
				if (sc->re_tx_cstag) {
					ifp->if_capenable |= IFCAP_TXCSUM;
					if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
						ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
					else
						ifp->if_hwassist |= RE_CSUM_FEATURES;
				}
				if (sc->re_rx_cstag) {
					ifp->if_capenable |= IFCAP_RXCSUM;
					CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
				}
			 }
		}
	} else if (macver == 0x3C800000) {
		/*set configuration space offset 0x70f to 0x3f*/
		CSR_WRITE_4(sc, RE_CSIDR, 0x27000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		re_eri_write(sc, 0x1EC, 1, 0x07, ERIAR_ASF);

		//disable clock request.
		pci_write_config(sc->dev, 0x81, 0x00, 1);

		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);
		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
		if (sc->re_type == MACFG_28)
			CSR_WRITE_1(sc, 0xD1, 0x20);

		if (ifp->if_mtu > ETHERMTU) {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x20;
				pci_write_config(sc->dev, 0x79, data8, 1);
				ifp->if_capenable &= ~IFCAP_HWCSUM;
				CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;
			 } else {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x50;
				pci_write_config(sc->dev, 0x79, data8, 1);
				if (sc->re_tx_cstag) {
					ifp->if_capenable |= IFCAP_TXCSUM;
					ifp->if_hwassist |= RE_CSUM_FEATURES;
				}
				if (sc->re_rx_cstag) {
					ifp->if_capenable |= IFCAP_RXCSUM;
					CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
				}
		 }


	} else if (macver == 0x28000000) {
		/*set configuration space offset 0x70f to 0x3f*/
		CSR_WRITE_4(sc, RE_CSIDR, 0x27000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		/* disable clock request. */
		pci_write_config(sc->dev, 0x81, 0x00, 1);

		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);
		CSR_WRITE_1(sc, RE_DBG_reg, CSR_READ_1(sc, RE_DBG_reg)|0x82);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

		if (ifp->if_mtu > ETHERMTU) {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x20;
				pci_write_config(sc->dev, 0x79, data8, 1);
				ifp->if_capenable &= ~IFCAP_HWCSUM;
				CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;

			 } else {
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x50;
				pci_write_config(sc->dev, 0x79, data8, 1);
				if (sc->re_tx_cstag) {
					ifp->if_capenable |= IFCAP_TXCSUM;
					ifp->if_hwassist |= RE_CSUM_FEATURES;
				}
				if (sc->re_rx_cstag) {
					ifp->if_capenable |= IFCAP_RXCSUM;
					CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
				}
		 }

		if (sc->re_type == MACFG_31)
		{
			CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~(1<<4));

			MP_WriteEPhyUshort(sc, 0x01, 0x7C7D);
			MP_WriteEPhyUshort(sc, 0x02, 0x091F);
			MP_WriteEPhyUshort(sc, 0x06, 0xB271);
			MP_WriteEPhyUshort(sc, 0x07, 0xCE00);
		}
		else if (sc->re_type == MACFG_32)
		{
			MP_WriteEPhyUshort(sc, 0x01, 0x7C7D);
			MP_WriteEPhyUshort(sc, 0x02, 0x091F);
			MP_WriteEPhyUshort(sc, 0x03, 0xC5BA);
			MP_WriteEPhyUshort(sc, 0x06, 0xB279);
			MP_WriteEPhyUshort(sc, 0x07, 0xAF00);
			MP_WriteEPhyUshort(sc, 0x1E, 0xB8EB);
		}
		else if (sc->re_type == MACFG_33)
		{
			CSR_WRITE_1(sc, RE_CFG1, CSR_READ_1(sc, RE_CFG1)|0x10);

			MP_WriteEPhyUshort(sc, 0x01, 0x6C7F);
			MP_WriteEPhyUshort(sc, 0x02, 0x011F);
			MP_WriteEPhyUshort(sc, 0x03, 0xC1B2);
			MP_WriteEPhyUshort(sc, 0x1A, 0x0546);
			MP_WriteEPhyUshort(sc, 0x1C, 0x80C4);
			MP_WriteEPhyUshort(sc, 0x1D, 0x78E4);
			MP_WriteEPhyUshort(sc, 0x0A, 0x8100);

			CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3) | BIT_2);
		}
	} else if (macver == 0x2C000000) {
		CSR_WRITE_4(sc, RE_CSIDR, 0x27000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3)|0x20);
		CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3)& ~0x20);

		CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0)|0xC0);
		CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1)|0xF3);
		CSR_WRITE_1(sc, RE_CFG5, (CSR_READ_1(sc, RE_CFG5)& ~0x08)|0x01);
		CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2)|0x80);
		CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

		if (sc->re_type == MACFG_36 || sc->re_type == MACFG_37)
		{
			/* set EPHY registers */
			data16 = MP_ReadEPhyUshort(sc, 0x00) & ~0x0200;
			data16 |= 0x0100;
			MP_WriteEPhyUshort(sc, 0x00, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x00);
			data16 |= 0x0004;
			MP_WriteEPhyUshort(sc, 0x00, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x06) & ~0x0002;
			data16 |= 0x0001;
			MP_WriteEPhyUshort(sc, 0x06, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x06);
			data16 |= 0x0030;
			MP_WriteEPhyUshort(sc, 0x06, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x07);
			data16 |= 0x2000;
			MP_WriteEPhyUshort(sc, 0x07, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x00);
			data16 |= 0x0020;
			MP_WriteEPhyUshort(sc, 0x00, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x03) & ~0x5800;
			data16 |= 0x2000;
			MP_WriteEPhyUshort(sc, 0x03, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x03);
			data16 |= 0x0001;
			MP_WriteEPhyUshort(sc, 0x03, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x01) & ~0x0800;
			data16 |= 0x1000;
			MP_WriteEPhyUshort(sc, 0x01, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x07);
			data16 |= 0x4000;
			MP_WriteEPhyUshort(sc, 0x07, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x1E);
			data16 |= 0x2000;
			MP_WriteEPhyUshort(sc, 0x1E, data16);

			MP_WriteEPhyUshort(sc, 0x19, 0xFE6C);

			data16 = MP_ReadEPhyUshort(sc, 0x0A);
			data16 |= 0x0040;
			MP_WriteEPhyUshort(sc, 0x0A, data16);

			if (ifp->if_mtu > ETHERMTU)
			{
				CSR_WRITE_1 (sc, RE_MTPS, 0x24);
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2);
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) |0x01);
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x20;
				pci_write_config(sc->dev, 0x79, data8, 1);
				ifp->if_capenable &= ~IFCAP_HWCSUM;
				CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;
			} else {
				CSR_WRITE_1 (sc, RE_MTPS, 0x0c);
				CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2);
				CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~0x01);
				data8 = pci_read_config(sc->dev, 0x79, 1);
				data8 &= ~0x70;
				data8 |= 0x50;
				pci_write_config(sc->dev, 0x79, data8, 1);

				if (sc->re_tx_cstag) {
					ifp->if_capenable |= IFCAP_TXCSUM;
					ifp->if_hwassist |= RE_CSUM_FEATURES;
				}
				if (sc->re_rx_cstag) {
					ifp->if_capenable |= IFCAP_RXCSUM;
					CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
				}
			}

			//disable clock request.
			pci_write_config(sc->dev, 0x81, 1, 1);
		}
	} else if (macver == 0x2C800000) {
		CSR_WRITE_4(sc, RE_CSIDR, 0x17000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		data8 = pci_read_config(sc->dev, 0x79, 1);
		data8 &= ~0x70;
		data8 |= 0x50;
		pci_write_config(sc->dev, 0x79, data8, 1);

		re_eri_write(sc, 0xD5, 1, 0x0000000C, ERIAR_ExGMAC);
		re_eri_write(sc, 0xC0, 2, 0x00000000, ERIAR_ExGMAC);
		re_eri_write(sc, 0xB8, 2, 0x00000000, ERIAR_ExGMAC);
		re_eri_write(sc, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
		re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

		CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) |BIT_7);
		CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
		CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);
		CSR_WRITE_1(sc, 0xF1,CSR_READ_1(sc, 0xF1) & ~ BIT_7);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

		if (sc ->re_type == MACFG_38) {
			CSR_WRITE_4(sc, 0xB0, 0xEE480010);
			CSR_WRITE_1(sc, 0x1A, CSR_READ_1(sc, 0x1A) & ~(BIT_2 |BIT_3));
			re_eri_write(sc, 0x1DC, 1, 0x64, ERIAR_ExGMAC);

			MP_WriteEPhyUshort(sc, 0x06, 0xF020);
			MP_WriteEPhyUshort(sc, 0x07, 0x01FF);
			MP_WriteEPhyUshort(sc, 0x00, 0x5027);
			MP_WriteEPhyUshort(sc, 0x01, 0x0003);
			MP_WriteEPhyUshort(sc, 0x02, 0x2D16);
			MP_WriteEPhyUshort(sc, 0x03, 0x6D49);
			MP_WriteEPhyUshort(sc, 0x08, 0x0006);
			MP_WriteEPhyUshort(sc, 0x0A, 0x00C8);
		}

		data16 = MP_ReadEPhyUshort(sc, 0x09);
		data16 |= BIT_7;
		MP_WriteEPhyUshort(sc, 0x09, data16);

		data16 = MP_ReadEPhyUshort(sc, 0x19);
		data16 |= (BIT_2 | BIT_5 | BIT_9);
		MP_WriteEPhyUshort(sc, 0x19, data16);

		CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) | BIT_0);
		CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_5 |BIT_7);

		CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
		CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

		CSR_WRITE_1 (sc, RE_MTPS, 0x27);
		ifp->if_capenable &= ~IFCAP_HWCSUM ;
		ifp->if_hwassist &= ~RE_CSUM_FEATURES;

		/* disable clock request. */
		pci_write_config(sc->dev, 0x81, 0x00, 1);
	} else if (macver == 0x24000000) {
		if (pci_read_config(sc->dev, 0x81, 1)==1)
		{
			CSR_WRITE_1(sc, RE_DBG_reg, 0x98);
			CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | 0x80);
			CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | 0x04);
			pci_write_config(sc->dev, 0x81, 1, 1);
		}
		data8 = pci_read_config(sc->dev, 0x79, 1);
		data8 &= ~0x70;
		data8 |= 0x50;
		pci_write_config(sc->dev, 0x79, data8, 1);

		/*set configuration space offset 0x70f to 0x3f*/
		CSR_WRITE_4(sc, RE_CSIDR, 0x3F000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		//CSR_WRITE_2(sc, RE_RxMaxSize, 0x05EF);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

		MP_WriteEPhyUshort(sc, 0x06, 0xAF25);
		MP_WriteEPhyUshort(sc, 0x07, 0x8E68);
	} else if (macver == 0x40800000) {
		if (pci_read_config(sc->dev, 0x80, 1) & 0x03)
		{
			CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) | 1);
			CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1) | 0x80);
			CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | 0x80);
			CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | 0x80);
		}
		CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1) | 0x28);
		CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~0x01);
		CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) | 0x0C);
		CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | 0x40);
		CSR_WRITE_2(sc, 0xE0, CSR_READ_2(sc, 0xE0) & ~0xDF9C);
		if (sc->re_type == MACFG_42)
		{
			/* set EPHY registers */
			data16 = MP_ReadEPhyUshort(sc, 0x07);
			data16 |= 0x4000;
			MP_WriteEPhyUshort(sc, 0x07, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x19);
			data16 |= 0x0200;
			MP_WriteEPhyUshort(sc, 0x19, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x19);
			data16 |= 0x0020;
			MP_WriteEPhyUshort(sc, 0x19, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x1E);
			data16 |= 0x2000;
			MP_WriteEPhyUshort(sc, 0x1E, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x03);
			data16 |= 0x0001;
			MP_WriteEPhyUshort(sc, 0x03, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x19);
			data16 |= 0x0100;
			MP_WriteEPhyUshort(sc, 0x19, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x19);
			data16 |= 0x0004;
			MP_WriteEPhyUshort(sc, 0x19, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x0A);
			data16 |= 0x0020;
			MP_WriteEPhyUshort(sc, 0x0A, data16);

			if (sc->re_type == MACFG_42) {
				data16 = MP_ReadEPhyUshort(sc, 0x1E);
				data16 |= 0x8000;
				MP_WriteEPhyUshort(sc, 0x1E, data16);
			}
		}
	} else if (macver == 0x48000000) {
		CSR_WRITE_4(sc, RE_CSIDR, 0x27000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8000870C);

		data8 = pci_read_config(sc->dev, 0x79, 1);
		data8 &= ~0x70;
		data8 |= 0x50;
		pci_write_config(sc->dev, 0x79, data8, 1);

		Data32 = re_eri_read(sc, 0xd4, 4, ERIAR_ExGMAC);
		Data32 |= BIT_11 | BIT_10;
		re_eri_write(sc, 0xD4, 2, Data32, ERIAR_ExGMAC);
		re_eri_write(sc, 0xC0, 2, 0x00000000, ERIAR_ExGMAC);
		re_eri_write(sc, 0xB8, 2, 0x00000000, ERIAR_ExGMAC);
		re_eri_write(sc, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
		re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
		Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
		Data32 &= ~BIT_0;
		re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
		Data32 |= BIT_0;
		re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);

		CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
		CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
		CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

		if (sc->re_type == MACFG_50) {
			data16 = MP_ReadEPhyUshort(sc, 0x06);
			data16 &= ~(BIT_7 | BIT_6);
			data16 |= BIT_5;
			MP_WriteEPhyUshort(sc, 0x06, data16);

			data16 = MP_ReadEPhyUshort(sc, 0x08);
			data16 &= ~BIT_0;
			data16 |= BIT_1;
			MP_WriteEPhyUshort(sc, 0x08, data16);
		}

		data16 = MP_ReadEPhyUshort(sc, 0x09);
		data16 |= BIT_7;
		MP_WriteEPhyUshort(sc, 0x09, data16);

		data16 = MP_ReadEPhyUshort(sc, 0x19);
		data16 |= (BIT_2 | BIT_5 | BIT_9);
		MP_WriteEPhyUshort(sc, 0x19, data16);

		CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) | BIT_0);
		CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_7);

		CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
		CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

		CSR_WRITE_1 (sc, RE_MTPS, 0x27);

		if (ifp->if_mtu > ETHERMTU) {
			ifp->if_capenable &= ~IFCAP_HWCSUM ;
			ifp->if_hwassist &= ~RE_CSUM_FEATURES;
		} else {
			if (sc->re_tx_cstag) {
				ifp->if_capenable |= IFCAP_TXCSUM;
				ifp->if_hwassist |= RE_CSUM_FEATURES;
			}
			if (sc->re_rx_cstag) {
				ifp->if_capenable |= IFCAP_RXCSUM;
			}
		}

		/* disable clock request. */
		pci_write_config(sc->dev, 0x81, 0x00, 1);
	} else if (macver == 0x48800000) {
		CSR_WRITE_4(sc, RE_CSIDR, 0x27000000);
		CSR_WRITE_4(sc, RE_CSIAR, 0x8002870C);

		data8 = pci_read_config(sc->dev, 0x79, 1);
		data8 &= ~0x70;
		data8 |= 0x50;
		pci_write_config(sc->dev, 0x79, data8, 1);

		Data32 = re_eri_read(sc, 0xd4, 4, ERIAR_ExGMAC);
		Data32 |= BIT_11 | BIT_10;
		re_eri_write(sc, 0xD4, 2, Data32, ERIAR_ExGMAC);
		re_eri_write(sc, 0xC0, 2, 0x00000000, ERIAR_ExGMAC);
		re_eri_write(sc, 0xB8, 2, 0x00000000, ERIAR_ExGMAC);
		re_eri_write(sc, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
		re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
		Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
		Data32 &= ~BIT_0;
		re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
		Data32 |= BIT_0;
		re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
		Data32 = re_eri_read(sc, 0x1d0, 4, ERIAR_ExGMAC);
		Data32 &= ~BIT_0;
		re_eri_write(sc, 0x1d0, 1, Data32, ERIAR_ExGMAC);

		CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
		CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
//		CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

		CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

		data16 = MP_ReadEPhyUshort(sc, 0x06);
		data16 &= ~(BIT_7 | BIT_6);
		data16 |= BIT_5;
		MP_WriteEPhyUshort(sc, 0x06, data16);

		MP_WriteEPhyUshort(sc, 0x0f, 0x5200);

		data16 = MP_ReadEPhyUshort(sc, 0x1e);
		data16 |= BIT_14;
		MP_WriteEPhyUshort(sc, 0x1e, data16);

		data16 = MP_ReadEPhyUshort(sc, 0x19);
		data16 |= (BIT_2 | BIT_5 | BIT_9);
		MP_WriteEPhyUshort(sc, 0x19, data16);

		CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) | BIT_0);
		CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_7);

		CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
		CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

		CSR_WRITE_1 (sc, RE_MTPS, 0x27);

		if (ifp->if_mtu > ETHERMTU) {
			ifp->if_capenable &= ~IFCAP_HWCSUM ;
			ifp->if_hwassist &= ~RE_CSUM_FEATURES;
		} else {
			if (sc->re_tx_cstag) {
				ifp->if_capenable |= IFCAP_TXCSUM;
				ifp->if_hwassist |= RE_CSUM_FEATURES;
			}
			if (sc->re_rx_cstag) {
				ifp->if_capenable |= IFCAP_RXCSUM;
			}
		}

		/* disable clock request. */
		pci_write_config(sc->dev, 0x81, 0x00, 1);
	}

	data16 = CSR_READ_2(sc, RE_CPlusCmd);
	if ((ifp->if_capenable & IFCAP_VLAN_HWTAGGING) != 0)
		data16 |= RL_CPLUSCMD_VLANSTRIP;
	else
		data16 &= ~RL_CPLUSCMD_VLANSTRIP;

	if ((ifp->if_capenable & IFCAP_RXCSUM) != 0)
		data16 |= RL_RxChkSum;
	else
		data16 &= ~RL_RxChkSum;
	CSR_WRITE_2 (sc, RE_CPlusCmd, data16);

	CSR_WRITE_1(sc, RE_EECMD, RE_EEMODE_OFF);
	//CSR_WRITE_1(sc, 0xec, 0x3f);

	/* Enable transmit and receive.*/
	CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_TX_ENB | RE_CMD_RX_ENB);

	/* Set the initial TX configuration.*/
	CSR_WRITE_4(sc, RE_TXCFG, RE_TXCFG_CONFIG);

	/* Set the initial RX configuration.*/
	/* Set the individual bit to receive frames for this host only. */
	rxcfg = CSR_READ_4(sc, RE_RXCFG);
	rxcfg |= RE_RXCFG_RX_INDIV;

	/* If we want promiscuous mode, set the allframes bit. */
	if (ifp->if_flags & IFF_PROMISC) {
		rxcfg |= RE_RXCFG_RX_ALLPHYS;
	} else {
		rxcfg &= ~RE_RXCFG_RX_ALLPHYS;
	}

	/*
	 * Set capture broadcast bit to capture broadcast frames.
	 */
	if (ifp->if_flags & IFF_BROADCAST) {
		rxcfg |= RE_RXCFG_RX_BROAD;
	} else {
		rxcfg &= ~RE_RXCFG_RX_BROAD;
	}

	CSR_WRITE_4(sc, RE_RXCFG, rxcfg);

	/*
	 * Program the multicast filter, if necessary.
	 */
	re_setmulti(sc);

	if ((sc->re_type == MACFG_3) || (sc->re_type == MACFG_4)) {
		CSR_WRITE_2(sc, RE_RxMaxSize, 0x800);	/* RMS */
	}

	CSR_WRITE_1(sc, RE_CFG1, RE_CFG1_DRVLOAD|RE_CFG1_FULLDUPLEX);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	/*(void)splx(s);*/
	RE_UNLOCK(sc);

	sc->re_stat_ch = timeout(re_tick, sc, hz);
}

static void re_init(void *xsc)		/* Software & Hardware Initialize */
{
	struct re_softc		*sc = xsc;
	struct ifnet		*ifp;
#if OS_VER < VERSION(6,0)
	int			i;
#endif
	//u_int8_t		data8;
	//u_int16_t		data16 = 0;
	//u_int32_t		rxcfg = 0;
	//u_int32_t	macver;


	ifp = RE_GET_IFNET(sc);
	/*s = splimp();*/
	RE_LOCK(sc);

/*	RE_LOCK_ASSERT(sc);*/

	/*mii = device_get_softc(sc->re_miibus);*/

	/*
	 * Cancel pending I/O and free all RX/TX buffers.
	 */
	re_stop(sc);

	/* Init our MAC address */
	CSR_WRITE_1(sc, RE_EECMD, RE_EEMODE_WRITECFG);
#if OS_VER < VERSION(6,0)
	for (i = 0; i < ETHER_ADDR_LEN; i++) {
		CSR_WRITE_1(sc, RE_IDR0 + i, sc->arpcom.ac_enaddr[i]);
	}
#elif OS_VER < VERSION(7,0)
	CSR_WRITE_STREAM_4(sc, RE_IDR0,
	    *(u_int32_t *)(&IFP2ENADDR(sc->re_ifp)[0]));
	CSR_WRITE_STREAM_4(sc, RE_IDR4,
	    *(u_int32_t *)(&IFP2ENADDR(sc->re_ifp)[4]));
#else
	CSR_WRITE_STREAM_4(sc, RE_IDR0,
	    *(u_int32_t *)(&IF_LLADDR(sc->re_ifp)[0]));
	CSR_WRITE_STREAM_4(sc, RE_IDR4,
	    *(u_int32_t *)(&IF_LLADDR(sc->re_ifp)[4]));
#endif
	CSR_WRITE_1(sc, RE_EECMD, RE_EEMODE_OFF);

	RE_UNLOCK(sc);

	re_hw_start(sc);

	return;
}

/*
 * Initialize the transmit descriptors.
 */
static int re_var_init(struct re_softc *sc)
{
	int			i;
	union RxDesc *rxptr;
	union TxDesc *txptr;

	sc->re_desc.rx_cur_index = 0;
	sc->re_desc.rx_last_index = 0;
	rxptr = sc->re_desc.rx_desc;
	for (i = 0; i < RE_RX_BUF_NUM; i++) {
		memset(&rxptr[i], 0, sizeof(union RxDesc));
		rxptr[i].so0.OWN = 1;
		if (i == (RE_RX_BUF_NUM - 1))
			rxptr[i].so0.EOR = 1;
		rxptr[i].so0.Frame_Length = sc->re_rx_buf_sz;

		/* Init the RX buffer pointer register. */
		bus_dmamap_load(sc->re_desc.re_rx_mtag,
			sc->re_desc.re_rx_dmamap[i],
			sc->re_desc.rx_buf[i]->m_data, sc->re_rx_buf_sz,
			re_dma_map_buf,
			&rxptr[i],
			0);
		bus_dmamap_sync(sc->re_desc.re_rx_mtag,
			sc->re_desc.re_rx_dmamap[i],
			BUS_DMASYNC_PREWRITE);
	}

	bus_dmamap_load(sc->re_desc.rx_desc_tag,
		sc->re_desc.rx_desc_dmamap,
		sc->re_desc.rx_desc,
		sizeof(union RxDesc)*RE_RX_BUF_NUM,
		re_dma_map_rxdesc,
		sc,
		0);
	bus_dmamap_sync(sc->re_desc.rx_desc_tag,
		sc->re_desc.rx_desc_dmamap,
		BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

	sc->re_desc.tx_cur_index = 0;
	sc->re_desc.tx_last_index = 0;
	txptr = sc->re_desc.tx_desc;
	for (i = 0; i < RE_TX_BUF_NUM; i++) {
		memset(&txptr[i], 0, sizeof(union TxDesc));
		if (i == (RE_TX_BUF_NUM - 1))
			txptr[i].so1.EOR = 1;
	}

	bus_dmamap_load(sc->re_desc.tx_desc_tag,
		sc->re_desc.tx_desc_dmamap,
		sc->re_desc.tx_desc,
		sizeof(union RxDesc) * RE_TX_BUF_NUM,
		re_dma_map_txdesc,
		sc,
		0);
//	bus_dmamap_sync(sc->re_desc.tx_desc_tag,
//		sc->re_desc.tx_desc_dmamap,
//		BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

	return 0;
}

static void re_reset(struct re_softc *sc)
{
	register int		i;

	CSR_WRITE_4(sc, RE_RXCFG, CSR_READ_4(sc, RE_RXCFG)& ~0x3F);

	switch (sc->re_type) {
		case RE_8129:
		case RE_8139:
		case MACFG_3:
		case MACFG_4:
		case MACFG_5:
		case MACFG_6:
			break;
		case MACFG_11:
		case MACFG_12:
		case MACFG_13:
		case MACFG_14:
		case MACFG_15:
		case MACFG_16:
		case MACFG_17:
		case MACFG_18:
		case MACFG_19:
		case MACFG_21:
		case MACFG_22:
		case MACFG_23:
		case MACFG_24:
		case MACFG_25:
		case MACFG_26:
		case MACFG_27:
		case MACFG_28:
		case MACFG_31:
		case MACFG_32:
		case MACFG_33:
		case MACFG_34:
		case MACFG_35:
		case MACFG_36:
		case MACFG_37:
		case MACFG_41:
		case MACFG_42:
		case MACFG_43:
			CSR_WRITE_1(sc, RE_COMMAND, 0x8C);
			break;
		case MACFG_38:
		case MACFG_39:
		case MACFG_50:
		case MACFG_51:
		case MACFG_52:
		default:
			CSR_WRITE_1(sc, RE_COMMAND, 0x8C);
			while (!(CSR_READ_4(sc,RE_TXCFG) & BIT_11)) DELAY(100);
			break;
	}
	DELAY(200);
	CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_RESET);

	for (i = 0; i < RE_TIMEOUT; i++) {
		DELAY(10);
		if (!(CSR_READ_1(sc, RE_COMMAND) & RE_CMD_RESET))
			break;
	}

	if (i == RE_TIMEOUT)
		device_printf(sc->dev,"reset never completed!\n");

	return;
}

/*
 * Stop the adapter and free any mbufs allocated to the
 * RX and TX lists.
 */
static void re_stop(struct re_softc *sc)		/* Stop Driver */
{
	struct ifnet		*ifp;

/*	RE_LOCK_ASSERT(sc);*/

	ifp = RE_GET_IFNET(sc);
#if OS_VER < VERSION(9,0)
	ifp->if_timer = 0;
#endif

	untimeout(re_tick, sc, sc->re_stat_ch);

	CSR_WRITE_2(sc, RE_IMR, 0x0000);
	re_reset(sc);

	/*
	 * Free the TX list buffers.
	 */
	while (sc->re_desc.tx_last_index!=sc->re_desc.tx_cur_index) {
		if (sc->re_desc.re_tx_mtag)
		{
			bus_dmamap_sync(sc->re_desc.re_tx_mtag,
				sc->re_desc.re_tx_dmamap[sc->re_desc.tx_last_index],
				BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->re_desc.re_tx_mtag,
				sc->re_desc.re_tx_dmamap[sc->re_desc.tx_last_index]);
		}

		if (sc->re_desc.tx_buf[sc->re_desc.tx_last_index]!=NULL)
		{
			m_freem(sc->re_desc.tx_buf[sc->re_desc.tx_last_index]);
			sc->re_desc.tx_buf[sc->re_desc.tx_last_index] = NULL;
		}
		sc->re_desc.tx_last_index = (sc->re_desc.tx_last_index+1)%RE_TX_BUF_NUM;
	}

	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	return;
}

/*
 * Main transmit routine.
 */
static void re_start(struct ifnet *ifp)	/* Transmit Packet*/
{
	struct re_softc		*sc;
	struct mbuf		*m_head = NULL;
	static int count =1;
	uint32_t  opts1 =0;
	uint32_t  opts2 =0;

#if 0	/*print the destination and source MAC addresses of tx packets*/
	int i;
#endif

	sc = ifp->if_softc;	/* Paste to ifp in function re_attach(dev) */

	RE_LOCK(sc);

/*	RE_LOCK_ASSERT(sc);*/

	if ((sc->driver_detach == 1) || (sc->rx_fifo_overflow != 0)) {
		RE_UNLOCK(sc);
		return;
	}

	while (1) {
		int fs = 1, ls = 0, TxLen = 0, PktLen, limit;
		struct mbuf *ptr;
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);	/* Remove(get) data from system transmit queue */
		if (m_head == NULL) {
			break;
		}

		if (CountMbufNum(m_head,&limit)>CountFreeTxDescNum(sc->re_desc))	/* No enough descriptor */
		{
			count++;

			IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		if (ifp->if_bpf)			/* If there's a BPF listener, bounce a copy of this frame to him. */
		{
			//printf("If there's a BPF listener, bounce a copy of this frame to him. \n");

/*#if OS_VER < VERSION(5, 1)*/
#if OS_VER < VERSION(4,9)
			bpf_mtap(ifp, m_head);
#else
			bpf_mtap(ifp->if_bpf, m_head);
#endif
		}

		if (limit)	/* At least one mbuf data size small than RE_MINI_DESC_SIZE */
		{
#ifdef _DEBUG_
			ptr = m_head;
			//printf("Limit=%d",limit);
			while (ptr != NULL)
			{
				printf("(before), len=%d T=%d F=%d",ptr->m_len,ptr->m_type,ptr->m_flags);
				ptr=ptr->m_next;
			}
			printf("\n===== Reach limit ======\n");
#endif
			if (re_encap(sc, m_head))
			{
				IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
				ifp->if_drv_flags |= IFF_DRV_OACTIVE;

				break;
			}

			m_head = sc->re_desc.tx_buf[sc->re_desc.tx_cur_index];
		}

		//hwchecksum
		if (ifp->if_capenable & IFCAP_TXCSUM) {
			if ((m_head->m_pkthdr.csum_flags & RE_CSUM_FEATURES) !=0) 	{
				if (!(sc->re_if_flags & RL_FLAG_DESCV2)) {
					opts1 |= RL_IPV4CS1;
					if ((m_head->m_pkthdr.csum_flags & CSUM_TCP)!=0)
							opts1 |=RL_TCPCS1;
					if ((m_head->m_pkthdr.csum_flags & CSUM_UDP)!=0)
							opts1 |=RL_UDPCS1;
				} else {
						opts2 |=  RL_IPV4CS;
						if ((m_head->m_pkthdr.csum_flags & CSUM_TCP)!=0)
							opts2 |= RL_TCPCS;
						else if ((m_head->m_pkthdr.csum_flags & CSUM_UDP)!=0)
							opts2 |= RL_UDPCS;
				}
			}
		}

		//vlan
		if (m_head->m_flags & M_VLANTAG)
				opts2 |= bswap16(m_head->m_pkthdr.ether_vtag) | RL_TDESC_VLANCTL_TAG ;
		ptr = m_head;
		PktLen = ptr->m_pkthdr.len;
#ifdef _DEBUG_
		printf("PktLen=%d",PktLen);
#endif
		while (ptr!=NULL)
		{
			if (ptr->m_len >0)
			{
#ifdef _DEBUG_
				printf(", len=%d T=%d F=%d",ptr->m_len,ptr->m_type,ptr->m_flags);
#endif
				TxLen += ptr->m_len;
				if (TxLen >= PktLen)
				{
					ls=1;
					sc->re_desc.tx_buf[sc->re_desc.tx_cur_index] = m_head;
				}
				else
					sc->re_desc.tx_buf[sc->re_desc.tx_cur_index] = NULL;

				//vlan
				WritePacket(sc,ptr->m_data,ptr->m_len,fs,ls,opts2,opts1);

				fs=0;
			}
			ptr = ptr->m_next;
		}
#ifdef _DEBUG_
		printf("\n");
#endif
	}
#if OS_VER < VERSION(9,0)
	ifp->if_timer = 5;
#endif

	RE_UNLOCK(sc);

	return;
}

/*
 * Encapsulate an mbuf chain in a descriptor by coupling the mbuf data
 * pointers to the fragment pointers.
 */
static int re_encap(struct re_softc *sc,struct mbuf *m_head)		/* Only used in ~C+ mode */
{
	struct mbuf		*m_new = NULL;

	m_new = m_defrag(m_head, M_DONTWAIT);

	if (m_new == NULL) {
		printf("re%d: no memory for tx list", sc->re_unit);
		return (1);
	}
	m_head = m_new;

	/* Pad frames to at least 60 bytes. */
	if (m_head->m_pkthdr.len < RE_MIN_FRAMELEN) {	/* Case length < 60 bytes */
		/*
		 * Make security concious people happy: zero out the
		 * bytes in the pad area, since we don't know what
		 * this mbuf cluster buffer's previous user might
		 * have left in it.
		 */
		bzero(mtod(m_head, char *) + m_head->m_pkthdr.len,
		     RE_MIN_FRAMELEN - m_head->m_pkthdr.len);
		m_head->m_pkthdr.len = RE_MIN_FRAMELEN;
		m_head->m_len = m_head->m_pkthdr.len;
	}

	sc->re_desc.tx_buf[sc->re_desc.tx_cur_index] = m_head;

	return(0);
}

static void WritePacket(struct re_softc	*sc, caddr_t addr, int len,int fs_flag,int ls_flag, uint32_t opts2,uint32_t opts1)
{
	union TxDesc *txptr;

	bus_dmamap_sync(sc->re_desc.tx_desc_tag,
		sc->re_desc.tx_desc_dmamap,
		BUS_DMASYNC_POSTWRITE);

	txptr=&(sc->re_desc.tx_desc[sc->re_desc.tx_cur_index]);

	txptr->ul[0]&=0x40000000;
	txptr->ul[0] |= htole32(opts1);
	txptr->ul[1]= htole32(opts2);

	if (fs_flag)
		txptr->so1.FS=1;
	if (ls_flag)
		txptr->so1.LS=1;
	txptr->so1.Frame_Length = len;
	bus_dmamap_load(sc->re_desc.re_tx_mtag,
		sc->re_desc.re_tx_dmamap[sc->re_desc.tx_cur_index],
		addr,
		len,
		re_dma_map_buf, txptr,
		0);
	bus_dmamap_sync(sc->re_desc.re_tx_mtag,
		sc->re_desc.re_tx_dmamap[sc->re_desc.tx_cur_index],
		BUS_DMASYNC_PREREAD);
	txptr->so1.OWN = 1;

	bus_dmamap_sync(sc->re_desc.tx_desc_tag,
		sc->re_desc.tx_desc_dmamap,
		BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

	if (ls_flag) {
		CSR_WRITE_1(sc, RE_TPPOLL, RE_NPQ);
	}

	sc->re_desc.tx_cur_index = (sc->re_desc.tx_cur_index+1)%RE_TX_BUF_NUM;
}

static int CountFreeTxDescNum(struct re_descriptor desc)
{
	int ret=desc.tx_last_index-desc.tx_cur_index;
	if (ret<=0)
		ret+=RE_TX_BUF_NUM;
	ret--;
	return ret;
}

static int CountMbufNum(struct mbuf *m_head, int *limit)
{
	int ret=0;
	struct mbuf *ptr = m_head;

	*limit=0;	/* 0:no limit find, 1:intermediate mbuf data size < RE_MINI_DESC_SIZE byte */
	while (ptr!=NULL) {
		if (ptr->m_len >0)
		{
			ret++;
			//printf("(before), len=%d T=%d F=%d",ptr->m_len,ptr->m_type,ptr->m_flags);
			if (ptr->m_len<RE_MINI_DESC_SIZE && ptr->m_next!=NULL)	/* except last descriptor */
				*limit=1;
		}
		ptr=ptr->m_next;
	}
	return ret;
}

/*
 * A frame was downloaded to the chip. It's safe for us to clean up
 * the list buffers.
 */
static void re_txeof(struct re_softc *sc)	/* Transmit OK/ERR handler */
{
	union TxDesc *txptr;
	struct ifnet		*ifp;

	/*	printf("X");*/

	ifp = RE_GET_IFNET(sc);

#if OS_VER < VERSION(9,0)
	/* Clear the timeout timer. */
	ifp->if_timer = 0;
#endif

	bus_dmamap_sync(sc->re_desc.tx_desc_tag,
		sc->re_desc.tx_desc_dmamap,
		BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);

	txptr=&(sc->re_desc.tx_desc[sc->re_desc.tx_last_index]);
	while (txptr->so1.OWN==0 && sc->re_desc.tx_last_index!=sc->re_desc.tx_cur_index) {
#ifdef _DEBUG_
			printf("**** Tx OK  ****\n");
#endif
		bus_dmamap_sync(sc->re_desc.re_tx_mtag,
			sc->re_desc.re_tx_dmamap[sc->re_desc.tx_last_index],
			BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->re_desc.re_tx_mtag,
			sc->re_desc.re_tx_dmamap[sc->re_desc.tx_last_index]);

		if (sc->re_desc.tx_buf[sc->re_desc.tx_last_index]!=NULL)
		{
			m_freem(sc->re_desc.tx_buf[sc->re_desc.tx_last_index]);	/* Free Current MBuf in a Mbuf list*/
			sc->re_desc.tx_buf[sc->re_desc.tx_last_index] = NULL;
		}

		sc->re_desc.tx_last_index = (sc->re_desc.tx_last_index+1)%RE_TX_BUF_NUM;
		txptr=&sc->re_desc.tx_desc[sc->re_desc.tx_last_index];
		ifp->if_opackets++;
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	}

	bus_dmamap_sync(sc->re_desc.tx_desc_tag,
		sc->re_desc.tx_desc_dmamap,
		BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

	return;
}

/*
 * A frame has been uploaded: pass the resulting mbuf chain up to
 * the higher level protocols.
 *
 * You know there's something wrong with a PCI bus-master chip design
 * when you have to use m_devget().
 *
 * The receive operation is badly documented in the datasheet, so I'll
 * attempt to document it here. The driver provides a buffer area and
 * places its base address in the RX buffer start address register.
 * The chip then begins copying frames into the RX buffer. Each frame
 * is preceeded by a 32-bit RX status word which specifies the length
 * of the frame and certain other status bits. Each frame (starting with
 * the status word) is also 32-bit aligned. The frame length is in the
 * first 16 bits of the status word; the lower 15 bits correspond with
 * the 'rx status register' mentioned in the datasheet.
 *
 * Note: to make the Alpha happy, the frame payload needs to be aligned
 * on a 32-bit boundary. To achieve this, we cheat a bit by copying from
 * the ring buffer starting at an address two bytes before the actual
 * data location. We can then shave off the first two bytes using m_adj().
 * The reason we do this is because m_devget() doesn't let us specify an
 * offset into the mbuf storage space, so we have to artificially create
 * one. The ring is allocated in such a way that there are a few unused
 * bytes of space preceecing it so that it will be safe for us to do the
 * 2-byte backstep even if reading from the ring at offset 0.
 */
static void re_rxeof(sc)	/* Receive Data OK/ERR handler */
	struct re_softc		*sc;
{
	struct ether_header	*eh;
	struct mbuf		*m;
	struct ifnet		*ifp;
	union RxDesc *rxptr;
	int bError;
	struct mbuf *buf;
	int size;

	u_int32_t opts2,opts1;

/*		RE_LOCK_ASSERT(sc);*/

	ifp = RE_GET_IFNET(sc);

	bus_dmamap_sync(sc->re_desc.rx_desc_tag,
		sc->re_desc.rx_desc_dmamap,
		BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);

	rxptr=&(sc->re_desc.rx_desc[sc->re_desc.rx_cur_index]);
	while (rxptr->so0.OWN==0)	/* Receive OK */
	{
		bError = 0;

		/* Check if this packet is received correctly*/
		if (rxptr->ul[0]&0x200000)	/*Check RES bit*/
		{
			bError=1;
			goto update_desc;
		}
		opts2= le32toh(rxptr->ul[1]);
		opts1 =  le32toh(rxptr->ul[0]);

		//buf = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR); /* Alloc a new mbuf */

		if (sc->max_jumbo_frame_size <= MCLBYTES)
			size = MCLBYTES;
		else if ((sc->max_jumbo_frame_size > MCLBYTES) && (sc->max_jumbo_frame_size <=  MJUMPAGESIZE))
			size = MJUMPAGESIZE;
		else
			size =MJUM9BYTES;

		buf = m_getjcl(M_DONTWAIT, MT_DATA, M_PKTHDR,size);
		if (buf==NULL)
		{
			bError=1;
			goto update_desc;
		}

		bus_dmamap_sync(sc->re_desc.re_rx_mtag,
			sc->re_desc.re_rx_dmamap[sc->re_desc.rx_cur_index],
			BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->re_desc.re_rx_mtag,
			sc->re_desc.re_rx_dmamap[sc->re_desc.rx_cur_index]);

		m = sc->re_desc.rx_buf[sc->re_desc.rx_cur_index];
		sc->re_desc.rx_buf[sc->re_desc.rx_cur_index] = buf;
		m->m_pkthdr.len = m->m_len = rxptr->so0.Frame_Length-ETHER_CRC_LEN;
		m->m_pkthdr.rcvif = ifp;
		//vlan
		if (opts2 & RL_RDESC_VLANCTL_TAG) {
			m->m_pkthdr.ether_vtag =
			    bswap16((opts2 & RL_RDESC_VLANCTL_DATA));
			m->m_flags |= M_VLANTAG;
		}
		if (ifp->if_capenable & IFCAP_RXCSUM) {
			if (!(sc->re_if_flags & RL_FLAG_DESCV2)) {
				if (opts1 & RL_ProtoIP)
					m->m_pkthdr.csum_flags |=  CSUM_IP_CHECKED;
				if (!(opts1 & RL_IPF))
					m->m_pkthdr.csum_flags |= CSUM_IP_VALID;
				if ((((opts1 & RL_ProtoIP)==(1<<17)) && !(opts1 & RL_TCPF))   || (((opts1 & RL_ProtoIP)==(1<<18)) && !(opts1 & RL_UDPF))) {
					m->m_pkthdr.csum_flags |= CSUM_DATA_VALID|CSUM_PSEUDO_HDR;
	   				m->m_pkthdr.csum_data = 0xffff;
				}
			} else {
	   			if ((opts1 & RL_ProtoIP) && (opts2 & RL_V4F))
	   				m->m_pkthdr.csum_flags |=  CSUM_IP_CHECKED;
	   			if (!(opts1 & RL_IPF) && (opts2 & RL_V4F))
	   				m->m_pkthdr.csum_flags |= CSUM_IP_VALID;
	   			if (((opts1 & RL_TCPT) && !(opts2 & RL_TCPF)) || ((opts1 & RL_UDPT) && !(opts2 & RL_UDPF))) {
	   				m->m_pkthdr.csum_flags |= CSUM_DATA_VALID|CSUM_PSEUDO_HDR;
	   				m->m_pkthdr.csum_data = 0xffff;
	   			}
			}
		}

		eh = mtod(m, struct ether_header *);
		ifp->if_ipackets++;
#ifdef _DEBUG_
		printf("Rcv Packet, Len=%d \n", m->m_len);
#endif

		RE_UNLOCK(sc);

/*#if OS_VER < VERSION(5, 1)*/
#if OS_VER < VERSION(4,9)
			/* Remove header from mbuf and pass it on. */
		m_adj(m, sizeof(struct ether_header));
		ether_input(ifp, eh, m);
#else
		(*ifp->if_input)(ifp, m);
#endif
		RE_LOCK(sc);

update_desc:
		rxptr->ul[0]&=0x40000000;	/* keep EOR bit */
		rxptr->ul[1]=0;

		rxptr->so0.Frame_Length = sc->re_rx_buf_sz;
		if (!bError)
		{
			bus_dmamap_load(sc->re_desc.re_rx_mtag,
				sc->re_desc.re_rx_dmamap[sc->re_desc.rx_cur_index],
				sc->re_desc.rx_buf[sc->re_desc.rx_cur_index]->m_data,
				sc->re_rx_buf_sz,
				re_dma_map_buf, rxptr,
				0);
			bus_dmamap_sync(sc->re_desc.re_rx_mtag,
				sc->re_desc.re_rx_dmamap[sc->re_desc.rx_cur_index],
				BUS_DMASYNC_PREWRITE);
		}
		rxptr->so0.OWN=1;
		sc->re_desc.rx_cur_index = (sc->re_desc.rx_cur_index+1)%RE_RX_BUF_NUM;
		rxptr=&sc->re_desc.rx_desc[sc->re_desc.rx_cur_index];
	}

	bus_dmamap_sync(sc->re_desc.rx_desc_tag,
		sc->re_desc.rx_desc_dmamap,
		BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

	return;
}

static void re_intr(void *arg)	/* Interrupt Handler */
{
	struct re_softc		*sc;

	sc = arg;

	if ((CSR_READ_2(sc, RE_ISR) & RE_INTRS) == 0) {
		return;// (FILTER_STRAY);
	}

	/* Disable interrupts. */
	CSR_WRITE_2(sc, RE_IMR, 0x0000);

#if OS_VER < VERSION(7,0)
	re_int_task(arg, 0);
#else
	taskqueue_enqueue_fast(taskqueue_fast, &sc->re_inttask);
#endif
//	return (FILTER_HANDLED);
}

static void re_int_task(void *arg, int npending)
{
	struct re_softc		*sc;
	struct ifnet		*ifp;
	u_int16_t		status;
	u_int8_t		link_status;

	sc = arg;

	RE_LOCK(sc);

	ifp = RE_GET_IFNET(sc);

	status = CSR_READ_2(sc, RE_ISR);

	if (status) {
		CSR_WRITE_2(sc, RE_ISR, status & 0xffbf);
	}

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
		RE_UNLOCK(sc);
		return;
	}

	if ((status & RE_ISR_RX_OK) || (status & RE_ISR_RX_ERR) || (status & RE_ISR_FIFO_OFLOW) || (status & RE_ISR_RX_OVERRUN)) {
		re_rxeof(sc);
	}

	if (sc->re_type == MACFG_21) {
		if (status & RE_ISR_FIFO_OFLOW) {
			sc->rx_fifo_overflow = 1;
			CSR_WRITE_2(sc, 0x00e2, 0x0000);
			CSR_WRITE_4(sc, 0x0048, 0x4000);
			CSR_WRITE_4(sc, 0x0058, 0x4000);
		} else{
			sc->rx_fifo_overflow = 0;
			CSR_WRITE_4(sc,RE_CPCR, 0x51512082);
		}

		if (status & RE_ISR_PCS_TIMEOUT) {
			if ((status & RE_ISR_FIFO_OFLOW) &&
			   (!(status & (RE_ISR_RX_OK | RE_ISR_TX_OK | RE_ISR_RX_OVERRUN)))) {
				re_reset(sc);
				re_init(sc);
				sc->rx_fifo_overflow = 0;
				CSR_WRITE_2(sc, RE_ISR, RE_ISR_FIFO_OFLOW);
			}
		}
	}

	if (status & RE_ISR_LINKCHG) {
		link_status = CSR_READ_1(sc, RE_PHY_STATUS);
		if (link_status & 0x02)
		{
			ifp->if_link_state = LINK_STATE_UP;
		}
		else
		{
			ifp->if_link_state = LINK_STATE_DOWN;
		}
	}

	if ((status & RE_ISR_TX_OK) || (status & RE_ISR_TX_ERR)) {
		re_txeof(sc);
	}

	if (status & RE_ISR_SYSTEM_ERR) {
		re_reset(sc);
		re_init(sc);
	}

	switch(sc->re_type) {
		case MACFG_21:
		case MACFG_22:
		case MACFG_23:
		case MACFG_24:
			CSR_WRITE_1(sc, 0x38, 0x40);
			break;

		default:
			break;
	}

	RE_UNLOCK(sc);

	if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
		re_start(ifp);

#if OS_VER>=VERSION(7,0)
	if (CSR_READ_2(sc, RE_ISR) & RE_INTRS) {
		taskqueue_enqueue_fast(taskqueue_fast, &sc->re_inttask);
		return;
	}
#endif

	/* Re-enable interrupts. */
	CSR_WRITE_2(sc, RE_IMR, RE_INTRS);
}

/*
 * Program the 64-bit multicast hash filter.
 */
static void re_setmulti(sc)
	struct re_softc		*sc;
{
	struct ifnet		*ifp;
	int			h = 0;
	u_int32_t		hashes[2] = { 0, 0 };
	struct ifmultiaddr	*ifma;
	u_int32_t		rxfilt;
	int			mcnt = 0;

	ifp = RE_GET_IFNET(sc);

	rxfilt = CSR_READ_4(sc, RE_RXCFG);

	if (ifp->if_flags & IFF_ALLMULTI || ifp->if_flags & IFF_PROMISC) {
		rxfilt |= RE_RXCFG_RX_MULTI;
		CSR_WRITE_4(sc, RE_RXCFG, rxfilt);
		CSR_WRITE_4(sc, RE_MAR0, 0xFFFFFFFF);
		CSR_WRITE_4(sc, RE_MAR4, 0xFFFFFFFF);
		return;
	}

	/* first, zot all the existing hash bits */
	CSR_WRITE_4(sc, RE_MAR0, 0);
	CSR_WRITE_4(sc, RE_MAR4, 0);

	/* now program new ones */
#if OS_VER > VERSION(6,0)
	IF_ADDR_LOCK(ifp);
#endif
#if OS_VER < VERSION(4,9)
	for (ifma = ifp->if_multiaddrs.lh_first; ifma != NULL;
				ifma = ifma->ifma_link.le_next)
#else
	TAILQ_FOREACH(ifma,&ifp->if_multiaddrs,ifma_link)
#endif
	{
		if (ifma->ifma_addr->sa_family != AF_LINK)
			continue;
		h = ether_crc32_be(LLADDR((struct sockaddr_dl *)
		    ifma->ifma_addr), ETHER_ADDR_LEN) >> 26;
		if (h < 32)
			hashes[0] |= (1 << h);
		else
			hashes[1] |= (1 << (h - 32));
		mcnt++;
	}
#if OS_VER > VERSION(6,0)
	IF_ADDR_UNLOCK(ifp);
#endif

	if (mcnt)
		rxfilt |= RE_RXCFG_RX_MULTI;
	else
		rxfilt &= ~RE_RXCFG_RX_MULTI;

	CSR_WRITE_4(sc, RE_RXCFG, rxfilt);
	CSR_WRITE_4(sc, RE_MAR0, hashes[0]);
	CSR_WRITE_4(sc, RE_MAR4, hashes[1]);

	return;
}

static int re_ioctl(ifp, command, data)
	struct ifnet		*ifp;
	u_long			command;
	caddr_t			data;
{
	struct re_softc		*sc = ifp->if_softc;
	struct ifreq		*ifr = (struct ifreq *) data;
	/*int			s;*/
	int			error = 0;
	int mask, reinit;
	/*s = splimp();*/

	switch(command) {
	case SIOCSIFADDR:
	case SIOCGIFADDR:
		error = ether_ioctl(ifp, command, data);
		break;
	case SIOCSIFMTU:

		//printf("before mtu =%d\n",(int)ifp->if_mtu);
		if (ifr->ifr_mtu > sc->max_jumbo_frame_size)
			error = EINVAL;
		else
		{	ifp->if_mtu = ifr->ifr_mtu;

			//if running
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
			{
				//printf("set mtu when running\n");

				RE_LOCK(sc);
				re_stop(sc);
				RE_UNLOCK(sc);

				re_release_buf(sc);
				set_rxbufsize(sc);
				error =re_alloc_buf(sc);

				RE_LOCK(sc);
				re_init(sc);
				RE_UNLOCK(sc);

			} else{
			//if ¨S¦brunning

				re_release_buf(sc);
				set_rxbufsize(sc);
				error =re_alloc_buf(sc);
			}

		}
	//	printf("after mtu =%d\n",(int)ifp->if_mtu);
		break;
	case SIOCSIFFLAGS:
		RE_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			re_init(sc);
		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			re_stop(sc);
		}
		error = 0;
		RE_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		RE_LOCK(sc);
		re_setmulti(sc);
		RE_UNLOCK(sc);
		error = 0;
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->media, command);
		break;
	case SIOCSIFCAP:


		mask = ifr->ifr_reqcap ^ ifp->if_capenable;
		reinit = 0;

		if ((mask & IFCAP_TXCSUM) != 0 && (ifp->if_capabilities & IFCAP_TXCSUM) != 0) {
			ifp->if_capenable ^= IFCAP_TXCSUM;
			if ((ifp->if_capenable & IFCAP_TXCSUM) != 0)  {
				if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_24) || (sc->re_type == MACFG_26))
					ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
				else
					ifp->if_hwassist |= RE_CSUM_FEATURES;
			}
			else
				ifp->if_hwassist &= ~RE_CSUM_FEATURES;
			reinit = 1;
		}

		if ((mask & IFCAP_RXCSUM) != 0 &&
		    (ifp->if_capabilities & IFCAP_RXCSUM) != 0) {
			ifp->if_capenable ^= IFCAP_RXCSUM;
			reinit = 1;
		}

		if ((ifp->if_mtu <= ETHERMTU ) || ((sc->re_type>= MACFG_3) &&(sc->re_type <=MACFG_6)) || ((sc->re_type>= MACFG_21) && (sc->re_type <=MACFG_23)))
		{
			if (ifp->if_capenable & IFCAP_TXCSUM)
				sc->re_tx_cstag = 1;
			else
				sc->re_tx_cstag = 0;

			if (ifp->if_capenable & IFCAP_RXCSUM)
				sc->re_rx_cstag = 1;
			else
				sc->re_rx_cstag = 0;
		}
		if ((mask & IFCAP_VLAN_HWTAGGING) != 0 &&
		    (ifp->if_capabilities & IFCAP_VLAN_HWTAGGING) != 0) {
			ifp->if_capenable ^= IFCAP_VLAN_HWTAGGING;
			/* TSO over VLAN requires VLAN hardware tagging. */
			//if ((ifp->if_capenable & IFCAP_VLAN_HWTAGGING) == 0)
			//	ifp->if_capenable &= ~IFCAP_VLAN_HWTSO;
			reinit = 1;
		}
		if (reinit && ifp->if_drv_flags & IFF_DRV_RUNNING) {
			ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
			re_init(sc);
		}
		VLAN_CAPABILITIES(ifp);

		break;
	default:
		error = EINVAL;
		break;
	}

	/*(void)splx(s);*/

	return(error);
}

static void re_tick(xsc)
	void			*xsc;
{	/*called per second*/
	struct re_softc		*sc;
	int			s;

	s = splimp();

	sc = xsc;
	/*mii = device_get_softc(sc->re_miibus);

	mii_tick(mii);*/

	splx(s);

	if (sc->re_type == MACFG_3 &&
	    sc->re_8169_MacVersion != 0 &&
	    sc->re_8169_PhyVersion == 0) {
		static int count = 0;

		if (CSR_READ_1 (sc, 0x6C) & 0x02)
			count = 0;
		else
			count++;
		if (count> 14) {
			MP_WritePhyUshort (sc, 0, 0x9000);
			count = 0;
		}
	}

	sc->re_stat_ch = timeout (re_tick, sc, hz);

	return;
}

#if OS_VER < VERSION(7,0)
static void re_watchdog(ifp)
	struct ifnet		*ifp;
{
	struct re_softc		*sc;

	sc = ifp->if_softc;

	printf("re%d: watchdog timeout\n", sc->re_unit);
	ifp->if_oerrors++;

	re_txeof(sc);
	re_rxeof(sc);
	re_init(sc);

	return;
}
#endif

/*
 * Set media options.
 */
static int re_ifmedia_upd(struct ifnet *ifp)
{
	struct re_softc	*sc = ifp->if_softc;
	struct ifmedia	*ifm = &sc->media;
	int anar;
	int gbcr;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return(EINVAL);

	switch (IFM_SUBTYPE(ifm->ifm_media)) {
	case IFM_AUTO:
		anar = ANAR_TX_FD |
		       ANAR_TX |
		       ANAR_10_FD |
		       ANAR_10;
		gbcr = GTCR_ADV_1000TFDX |
		       GTCR_ADV_1000THDX;
		break;
	case IFM_1000_SX:
#if OS_VER < 500000
	case IFM_1000_TX:
#else
	case IFM_1000_T:
#endif
		anar = ANAR_TX_FD |
		       ANAR_TX |
		       ANAR_10_FD |
		       ANAR_10;
		gbcr = GTCR_ADV_1000TFDX |
		       GTCR_ADV_1000THDX;
		break;
	case IFM_100_TX:
		gbcr = MP_ReadPhyUshort(sc, MII_100T2CR) &
		       ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
		if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX) {
			anar = ANAR_TX_FD |
			       ANAR_TX |
			       ANAR_10_FD |
			       ANAR_10;
		} else {
			anar = ANAR_TX |
			       ANAR_10_FD |
			       ANAR_10;
		}
		break;
	case IFM_10_T:
		gbcr = MP_ReadPhyUshort(sc, MII_100T2CR) &
		       ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
		if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX) {
			anar = ANAR_10_FD |
			       ANAR_10;
		} else {
			anar = ANAR_10;
		}

		if (sc->re_type == MACFG_13) {
			MP_WritePhyUshort(sc, MII_BMCR, 0x8000);
		}

		break;
	default:
		printf("re%d: Unsupported media type\n", sc->re_unit);
		return(0);
	}

	MP_WritePhyUshort(sc, 0x1F, 0x0000);
	if (sc->re_device_id==RT_DEVICEID_8169 || sc->re_device_id==RT_DEVICEID_8169SC || sc->re_device_id==RT_DEVICEID_8168) {
		MP_WritePhyUshort(sc, MII_ANAR, anar | 0x0800 | ANAR_FC);
		MP_WritePhyUshort(sc, MII_100T2CR, gbcr);
		MP_WritePhyUshort(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);
	} else if (sc->re_type == MACFG_36) {
		MP_WritePhyUshort(sc, MII_ANAR, anar | 0x0800 | ANAR_FC);
		MP_WritePhyUshort(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);
	} else
	{
		MP_WritePhyUshort(sc, MII_ANAR, anar | 1);
		MP_WritePhyUshort(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
	}
	return(0);
}

/*
 * Report current media status.
 */
static void re_ifmedia_sts(ifp, ifmr)
	struct ifnet		*ifp;
	struct ifmediareq	*ifmr;
{
	struct re_softc		*sc;
	unsigned char msr;

	sc = ifp->if_softc;

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	msr = CSR_READ_1(sc, 0x6c);

	if (msr & 0x02)
		ifmr->ifm_status |= IFM_ACTIVE;
	else
		return;

	if (msr & 0x01)
		ifmr->ifm_active |= IFM_FDX;
	else
		ifmr->ifm_active |= IFM_HDX;


	if (msr & 0x04)
		ifmr->ifm_active |= IFM_10_T;
	else if (msr & 0x08)
		ifmr->ifm_active |= IFM_100_TX;
	else if (msr & 0x10)
		ifmr->ifm_active |= IFM_1000_T;

	return;
}

static void re_8169s_init(struct re_softc *sc)
{
	u_int16_t Data;
	u_int32_t Data_u32;
	int	i;

	if (sc->re_type == MACFG_3) {
		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0x006e);
		MP_WritePhyUshort(sc, 0x08, 0x0708);
		MP_WritePhyUshort(sc, 0x15, 0x4000);
		MP_WritePhyUshort(sc, 0x18, 0x65c7);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x03, 0x00a1);
		MP_WritePhyUshort(sc, 0x02, 0x0008);
		MP_WritePhyUshort(sc, 0x01, 0x0120);
		MP_WritePhyUshort(sc, 0x00, 0x1000);
		MP_WritePhyUshort(sc, 0x04, 0x0800);
		MP_WritePhyUshort(sc, 0x04, 0x0000);

		MP_WritePhyUshort(sc, 0x03, 0xff41);
		MP_WritePhyUshort(sc, 0x02, 0xdf60);
		MP_WritePhyUshort(sc, 0x01, 0x0140);
		MP_WritePhyUshort(sc, 0x00, 0x0077);
		MP_WritePhyUshort(sc, 0x04, 0x7800);
		MP_WritePhyUshort(sc, 0x04, 0x7000);

		MP_WritePhyUshort(sc, 0x03, 0x802f);
		MP_WritePhyUshort(sc, 0x02, 0x4f02);
		MP_WritePhyUshort(sc, 0x01, 0x0409);
		MP_WritePhyUshort(sc, 0x00, 0xf0f9);
		MP_WritePhyUshort(sc, 0x04, 0x9800);
		MP_WritePhyUshort(sc, 0x04, 0x9000);

		MP_WritePhyUshort(sc, 0x03, 0xdf01);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0xff95);
		MP_WritePhyUshort(sc, 0x00, 0xba00);
		MP_WritePhyUshort(sc, 0x04, 0xa800);
		MP_WritePhyUshort(sc, 0x04, 0xa000);

		MP_WritePhyUshort(sc, 0x03, 0xff41);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0x0140);
		MP_WritePhyUshort(sc, 0x00, 0x00bb);
		MP_WritePhyUshort(sc, 0x04, 0xb800);
		MP_WritePhyUshort(sc, 0x04, 0xb000);

		MP_WritePhyUshort(sc, 0x03, 0xdf41);
		MP_WritePhyUshort(sc, 0x02, 0xdc60);
		MP_WritePhyUshort(sc, 0x01, 0x6340);
		MP_WritePhyUshort(sc, 0x00, 0x007d);
		MP_WritePhyUshort(sc, 0x04, 0xd800);
		MP_WritePhyUshort(sc, 0x04, 0xd000);

		MP_WritePhyUshort(sc, 0x03, 0xdf01);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0x100a);
		MP_WritePhyUshort(sc, 0x00, 0xa0ff);
		MP_WritePhyUshort(sc, 0x04, 0xf800);
		MP_WritePhyUshort(sc, 0x04, 0xf000);

		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x0b, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x9200);

		CSR_WRITE_1(sc, 0x82, 0x0d);
	} else if (sc->re_type == MACFG_4) {
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x01, 0x90D0);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1e, 0x8c00);
	} else if (sc->re_type == MACFG_5) {
		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x04, 0x0000);
		MP_WritePhyUshort(sc, 0x03, 0x00a1);
		MP_WritePhyUshort(sc, 0x02, 0x0008);
		MP_WritePhyUshort(sc, 0x01, 0x0120);
		MP_WritePhyUshort(sc, 0x00, 0x1000);
		MP_WritePhyUshort(sc, 0x04, 0x0800);

		MP_WritePhyUshort(sc, 0x04, 0x9000);
		MP_WritePhyUshort(sc, 0x03, 0x802f);
		MP_WritePhyUshort(sc, 0x02, 0x4f02);
		MP_WritePhyUshort(sc, 0x01, 0x0409);
		MP_WritePhyUshort(sc, 0x00, 0xf099);
		MP_WritePhyUshort(sc, 0x04, 0x9800);

		MP_WritePhyUshort(sc, 0x04, 0xa000);
		MP_WritePhyUshort(sc, 0x03, 0xdf01);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0xff95);
		MP_WritePhyUshort(sc, 0x00, 0xba00);
		MP_WritePhyUshort(sc, 0x04, 0xa800);

		MP_WritePhyUshort(sc, 0x04, 0xf000);
		MP_WritePhyUshort(sc, 0x03, 0xdf01);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0x101a);
		MP_WritePhyUshort(sc, 0x00, 0xa0ff);
		MP_WritePhyUshort(sc, 0x04, 0xf800);
		MP_WritePhyUshort(sc, 0x04, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x10, 0xf41b);
		MP_WritePhyUshort(sc, 0x14, 0xfb54);
		MP_WritePhyUshort(sc, 0x18, 0xf5c7);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x10, 0xf01b);

	} else if (sc->re_type == MACFG_6) {
		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x04, 0x0000);
		MP_WritePhyUshort(sc, 0x03, 0x00a1);
		MP_WritePhyUshort(sc, 0x02, 0x0008);
		MP_WritePhyUshort(sc, 0x01, 0x0120);
		MP_WritePhyUshort(sc, 0x00, 0x1000);
		MP_WritePhyUshort(sc, 0x04, 0x0800);

		MP_WritePhyUshort(sc, 0x04, 0x9000);
		MP_WritePhyUshort(sc, 0x03, 0x802f);
		MP_WritePhyUshort(sc, 0x02, 0x4f02);
		MP_WritePhyUshort(sc, 0x01, 0x0409);
		MP_WritePhyUshort(sc, 0x00, 0xf099);
		MP_WritePhyUshort(sc, 0x04, 0x9800);

		MP_WritePhyUshort(sc, 0x04, 0xa000);
		MP_WritePhyUshort(sc, 0x03, 0xdf01);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0xff95);
		MP_WritePhyUshort(sc, 0x00, 0xba00);
		MP_WritePhyUshort(sc, 0x04, 0xa800);

		MP_WritePhyUshort(sc, 0x04, 0xf000);
		MP_WritePhyUshort(sc, 0x03, 0xdf01);
		MP_WritePhyUshort(sc, 0x02, 0xdf20);
		MP_WritePhyUshort(sc, 0x01, 0x101a);
		MP_WritePhyUshort(sc, 0x00, 0xa0ff);
		MP_WritePhyUshort(sc, 0x04, 0xf800);
		MP_WritePhyUshort(sc, 0x04, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x0b, 0x8480);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x18, 0x67c7);
		MP_WritePhyUshort(sc, 0x04, 0x2000);
		MP_WritePhyUshort(sc, 0x03, 0x002f);
		MP_WritePhyUshort(sc, 0x02, 0x4360);
		MP_WritePhyUshort(sc, 0x01, 0x0109);
		MP_WritePhyUshort(sc, 0x00, 0x3022);
		MP_WritePhyUshort(sc, 0x04, 0x2800);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);
	} else if (sc->re_type == MACFG_14) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x11, MP_ReadPhyUshort(sc, 0x11) | 0x1000);
		MP_WritePhyUshort(sc, 0x19, MP_ReadPhyUshort(sc, 0x19) | 0x2000);
		MP_WritePhyUshort(sc, 0x10, MP_ReadPhyUshort(sc, 0x10) | 0x8000);

		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		MP_WritePhyUshort(sc, 0x08, 0x441D);
		MP_WritePhyUshort(sc, 0x01, 0x9100);
	} else if (sc->re_type == MACFG_15) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x11, MP_ReadPhyUshort(sc, 0x11) | 0x1000);
		MP_WritePhyUshort(sc, 0x19, MP_ReadPhyUshort(sc, 0x19) | 0x2000);
		MP_WritePhyUshort(sc, 0x10, MP_ReadPhyUshort(sc, 0x10) | 0x8000);

		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		MP_WritePhyUshort(sc, 0x08, 0x441D);
		MP_WritePhyUshort(sc, 0x01, 0x9100);
	} else if (sc->re_type == MACFG_17) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x11, MP_ReadPhyUshort(sc, 0x11) | 0x1000);
		MP_WritePhyUshort(sc, 0x19, MP_ReadPhyUshort(sc, 0x19) | 0x2000);
		MP_WritePhyUshort(sc, 0x10, MP_ReadPhyUshort(sc, 0x10) | 0x8000);

		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		MP_WritePhyUshort(sc, 0x08, 0x441D);

		MP_WritePhyUshort(sc, 0x1f, 0x0000);
	} else if (sc->re_type == MACFG_21) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x0B, 0x94B0);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0x6096);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_22) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x0B, 0x94B0);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0x6096);
	} else if (sc->re_type == MACFG_23) {
	 	MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x0B, 0x94B0);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0x6096);
	} else if (sc->re_type == MACFG_24) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x12, 0x2300);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x16, 0x000A);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0xC096);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x00, 0x88DE);
		MP_WritePhyUshort(sc, 0x01, 0x82B1);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x08, 0x9E30);
		MP_WritePhyUshort(sc, 0x09, 0x01F0);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0A, 0x5500);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x03, 0x7002);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x14, MP_ReadPhyUshort(sc, 0x14) | BIT_5);
		MP_WritePhyUshort(sc, 0x0d, MP_ReadPhyUshort(sc, 0x0d) | BIT_5);
	} else if (sc->re_type == MACFG_25) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x12, 0x2300);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x16, 0x0F0A);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x00, 0x88DE);
		MP_WritePhyUshort(sc, 0x01, 0x82B1);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0C, 0x7EB8);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x0761);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x03, 0x802F);
		MP_WritePhyUshort(sc, 0x02, 0x4F02);
		MP_WritePhyUshort(sc, 0x01, 0x0409);
		MP_WritePhyUshort(sc, 0x00, 0xF099);
		MP_WritePhyUshort(sc, 0x04, 0x9800);
		MP_WritePhyUshort(sc, 0x04, 0x9000);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x16, MP_ReadPhyUshort(sc , 0x16) | BIT_0);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x14, MP_ReadPhyUshort(sc , 0x14) | BIT_5);
		MP_WritePhyUshort(sc, 0x0D, MP_ReadPhyUshort(sc , 0x0D) & ~BIT_5);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x1D, 0x3D98);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_26) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x12, 0x2300);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x16, 0x0F0A);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x00, 0x88DE);
		MP_WritePhyUshort(sc, 0x01, 0x82B1);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0C, 0x7EB8);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x5461);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x5461);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x16, MP_ReadPhyUshort(sc, 0x16) | BIT_0);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x14, MP_ReadPhyUshort(sc, 0x14) | BIT_5);
		MP_WritePhyUshort(sc, 0x0D, MP_ReadPhyUshort(sc, 0x0D) & ~BIT_5);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x1D, 0x3D98);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_27) {
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, MP_ReadPhyUshort(sc, 0x0d) | BIT_5);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x1D, 0x3D98);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x14, 0xCAA3);
		MP_WritePhyUshort(sc, 0x1C, 0x000A);
		MP_WritePhyUshort(sc, 0x18, 0x65D0);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x17, 0xB580);
		MP_WritePhyUshort(sc, 0x18, 0xFF54);
		MP_WritePhyUshort(sc, 0x19, 0x3954);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0D, 0x310C);
		MP_WritePhyUshort(sc, 0x0E, 0x310C);
		MP_WritePhyUshort(sc, 0x0F, 0x311C);
		MP_WritePhyUshort(sc, 0x06, 0x0761);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x18, 0xFF55);
		MP_WritePhyUshort(sc, 0x19, 0x3955);
		MP_WritePhyUshort(sc, 0x18, 0xFF54);
		MP_WritePhyUshort(sc, 0x19, 0x3954);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);
	} else if (sc->re_type == MACFG_28) {
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x14, MP_ReadPhyUshort(sc, 0x14) | BIT_5);
		MP_WritePhyUshort(sc, 0x0d, MP_ReadPhyUshort(sc, 0x0d) | BIT_5);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x14, 0xCAA3);
		MP_WritePhyUshort(sc, 0x1C, 0x000A);
		MP_WritePhyUshort(sc, 0x18, 0x65D0);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x17, 0xB580);
		MP_WritePhyUshort(sc, 0x18, 0xFF54);
		MP_WritePhyUshort(sc, 0x19, 0x3954);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0D, 0x310C);
		MP_WritePhyUshort(sc, 0x0E, 0x310C);
		MP_WritePhyUshort(sc, 0x0F, 0x311C);
		MP_WritePhyUshort(sc, 0x06, 0x0761);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x18, 0xFF55);
		MP_WritePhyUshort(sc, 0x19, 0x3955);
		MP_WritePhyUshort(sc, 0x18, 0xFF54);
		MP_WritePhyUshort(sc, 0x19, 0x3954);

		MP_WritePhyUshort(sc, 0x1f, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x16, MP_ReadPhyUshort(sc, 0x16) | BIT_0);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_31) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0x4064);
		MP_WritePhyUshort(sc, 0x07, 0x2863);
		MP_WritePhyUshort(sc, 0x08, 0x059C);
		MP_WritePhyUshort(sc, 0x09, 0x26B4);
		MP_WritePhyUshort(sc, 0x0A, 0x6A19);
		MP_WritePhyUshort(sc, 0x0B, 0xDCC8);
		MP_WritePhyUshort(sc, 0x10, 0xF06D);
		MP_WritePhyUshort(sc, 0x14, 0x7F68);
		MP_WritePhyUshort(sc, 0x18, 0x7FD9);
		MP_WritePhyUshort(sc, 0x1C, 0xF0FF);
		MP_WritePhyUshort(sc, 0x1D, 0x3D9C);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0xF49F);
		MP_WritePhyUshort(sc, 0x13, 0x070B);
		MP_WritePhyUshort(sc, 0x1A, 0x05AD);
		MP_WritePhyUshort(sc, 0x14, 0x94C0);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		Data = MP_ReadPhyUshort(sc, 0x0B) & 0xFF00;
		Data |= 0x10;
		MP_WritePhyUshort(sc, 0x0B, Data);
		Data = MP_ReadPhyUshort(sc, 0x0C) & 0x00FF;
		Data |= 0xA200;
		MP_WritePhyUshort(sc, 0x0C, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x5561);
		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8332);
		MP_WritePhyUshort(sc, 0x06, 0x5561);

		if (MP_ReadEfuse(sc, 0x01) == 0xb1) {
			MP_WritePhyUshort(sc, 0x1F, 0x0002);
			MP_WritePhyUshort(sc, 0x05, 0x669A);
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0x8330);
			MP_WritePhyUshort(sc, 0x06, 0x669A);

			MP_WritePhyUshort(sc, 0x1F, 0x0002);
			Data = MP_ReadPhyUshort(sc, 0x0D);
			if ((Data & 0x00FF) != 0x006C) {
				Data &= 0xFF00;
				MP_WritePhyUshort(sc, 0x1F, 0x0002);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0065);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0066);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0067);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0068);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0069);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x006A);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x006B);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x006C);
			}
		} else {
			MP_WritePhyUshort(sc, 0x1F, 0x0002);
			MP_WritePhyUshort(sc, 0x05, 0x6662);
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0x8330);
			MP_WritePhyUshort(sc, 0x06, 0x6662);
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		Data = MP_ReadPhyUshort(sc, 0x0D);
		Data |= 0x300;
		MP_WritePhyUshort(sc, 0x0D, Data);
		Data = MP_ReadPhyUshort(sc, 0x0F);
		Data |= 0x10;
		MP_WritePhyUshort(sc, 0x0F, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		Data = MP_ReadPhyUshort(sc, 0x02);
		Data &= ~0x600;
		Data |= 0x100;
		MP_WritePhyUshort(sc, 0x02, Data);
		Data = MP_ReadPhyUshort(sc, 0x03);
		Data &= ~0xE000;
		MP_WritePhyUshort(sc, 0x03, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x001B);
		if (MP_ReadPhyUshort(sc, 0x06) == 0xBF00) {
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x0080);
			MP_WritePhyUshort(sc, 0x05, 0x8000);
			MP_WritePhyUshort(sc, 0x06, 0xf8f9);
			MP_WritePhyUshort(sc, 0x06, 0xfaef);
			MP_WritePhyUshort(sc, 0x06, 0x59ee);
			MP_WritePhyUshort(sc, 0x06, 0xf8ea);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0xf8eb);
			MP_WritePhyUshort(sc, 0x06, 0x00e0);
			MP_WritePhyUshort(sc, 0x06, 0xf87c);
			MP_WritePhyUshort(sc, 0x06, 0xe1f8);
			MP_WritePhyUshort(sc, 0x06, 0x7d59);
			MP_WritePhyUshort(sc, 0x06, 0x0fef);
			MP_WritePhyUshort(sc, 0x06, 0x0139);
			MP_WritePhyUshort(sc, 0x06, 0x029e);
			MP_WritePhyUshort(sc, 0x06, 0x06ef);
			MP_WritePhyUshort(sc, 0x06, 0x1039);
			MP_WritePhyUshort(sc, 0x06, 0x089f);
			MP_WritePhyUshort(sc, 0x06, 0x2aee);
			MP_WritePhyUshort(sc, 0x06, 0xf8ea);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0xf8eb);
			MP_WritePhyUshort(sc, 0x06, 0x01e0);
			MP_WritePhyUshort(sc, 0x06, 0xf87c);
			MP_WritePhyUshort(sc, 0x06, 0xe1f8);
			MP_WritePhyUshort(sc, 0x06, 0x7d58);
			MP_WritePhyUshort(sc, 0x06, 0x409e);
			MP_WritePhyUshort(sc, 0x06, 0x0f39);
			MP_WritePhyUshort(sc, 0x06, 0x46aa);
			MP_WritePhyUshort(sc, 0x06, 0x0bbf);
			MP_WritePhyUshort(sc, 0x06, 0x8290);
			MP_WritePhyUshort(sc, 0x06, 0xd682);
			MP_WritePhyUshort(sc, 0x06, 0x9802);
			MP_WritePhyUshort(sc, 0x06, 0x014f);
			MP_WritePhyUshort(sc, 0x06, 0xae09);
			MP_WritePhyUshort(sc, 0x06, 0xbf82);
			MP_WritePhyUshort(sc, 0x06, 0x98d6);
			MP_WritePhyUshort(sc, 0x06, 0x82a0);
			MP_WritePhyUshort(sc, 0x06, 0x0201);
			MP_WritePhyUshort(sc, 0x06, 0x4fef);
			MP_WritePhyUshort(sc, 0x06, 0x95fe);
			MP_WritePhyUshort(sc, 0x06, 0xfdfc);
			MP_WritePhyUshort(sc, 0x06, 0x05f8);
			MP_WritePhyUshort(sc, 0x06, 0xf9fa);
			MP_WritePhyUshort(sc, 0x06, 0xeef8);
			MP_WritePhyUshort(sc, 0x06, 0xea00);
			MP_WritePhyUshort(sc, 0x06, 0xeef8);
			MP_WritePhyUshort(sc, 0x06, 0xeb00);
			MP_WritePhyUshort(sc, 0x06, 0xe2f8);
			MP_WritePhyUshort(sc, 0x06, 0x7ce3);
			MP_WritePhyUshort(sc, 0x06, 0xf87d);
			MP_WritePhyUshort(sc, 0x06, 0xa511);
			MP_WritePhyUshort(sc, 0x06, 0x1112);
			MP_WritePhyUshort(sc, 0x06, 0xd240);
			MP_WritePhyUshort(sc, 0x06, 0xd644);
			MP_WritePhyUshort(sc, 0x06, 0x4402);
			MP_WritePhyUshort(sc, 0x06, 0x8217);
			MP_WritePhyUshort(sc, 0x06, 0xd2a0);
			MP_WritePhyUshort(sc, 0x06, 0xd6aa);
			MP_WritePhyUshort(sc, 0x06, 0xaa02);
			MP_WritePhyUshort(sc, 0x06, 0x8217);
			MP_WritePhyUshort(sc, 0x06, 0xae0f);
			MP_WritePhyUshort(sc, 0x06, 0xa544);
			MP_WritePhyUshort(sc, 0x06, 0x4402);
			MP_WritePhyUshort(sc, 0x06, 0xae4d);
			MP_WritePhyUshort(sc, 0x06, 0xa5aa);
			MP_WritePhyUshort(sc, 0x06, 0xaa02);
			MP_WritePhyUshort(sc, 0x06, 0xae47);
			MP_WritePhyUshort(sc, 0x06, 0xaf82);
			MP_WritePhyUshort(sc, 0x06, 0x13ee);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0x0fee);
			MP_WritePhyUshort(sc, 0x06, 0x834c);
			MP_WritePhyUshort(sc, 0x06, 0x0fee);
			MP_WritePhyUshort(sc, 0x06, 0x834f);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0x8351);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0x834a);
			MP_WritePhyUshort(sc, 0x06, 0xffee);
			MP_WritePhyUshort(sc, 0x06, 0x834b);
			MP_WritePhyUshort(sc, 0x06, 0xffe0);
			MP_WritePhyUshort(sc, 0x06, 0x8330);
			MP_WritePhyUshort(sc, 0x06, 0xe183);
			MP_WritePhyUshort(sc, 0x06, 0x3158);
			MP_WritePhyUshort(sc, 0x06, 0xfee4);
			MP_WritePhyUshort(sc, 0x06, 0xf88a);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x8be0);
			MP_WritePhyUshort(sc, 0x06, 0x8332);
			MP_WritePhyUshort(sc, 0x06, 0xe183);
			MP_WritePhyUshort(sc, 0x06, 0x3359);
			MP_WritePhyUshort(sc, 0x06, 0x0fe2);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0x0c24);
			MP_WritePhyUshort(sc, 0x06, 0x5af0);
			MP_WritePhyUshort(sc, 0x06, 0x1e12);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x8ce5);
			MP_WritePhyUshort(sc, 0x06, 0xf88d);
			MP_WritePhyUshort(sc, 0x06, 0xaf82);
			MP_WritePhyUshort(sc, 0x06, 0x13e0);
			MP_WritePhyUshort(sc, 0x06, 0x834f);
			MP_WritePhyUshort(sc, 0x06, 0x10e4);
			MP_WritePhyUshort(sc, 0x06, 0x834f);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x009f);
			MP_WritePhyUshort(sc, 0x06, 0x0ae0);
			MP_WritePhyUshort(sc, 0x06, 0x834f);
			MP_WritePhyUshort(sc, 0x06, 0xa010);
			MP_WritePhyUshort(sc, 0x06, 0xa5ee);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x01e0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7805);
			MP_WritePhyUshort(sc, 0x06, 0x9e9a);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x049e);
			MP_WritePhyUshort(sc, 0x06, 0x10e0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7803);
			MP_WritePhyUshort(sc, 0x06, 0x9e0f);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x019e);
			MP_WritePhyUshort(sc, 0x06, 0x05ae);
			MP_WritePhyUshort(sc, 0x06, 0x0caf);
			MP_WritePhyUshort(sc, 0x06, 0x81f8);
			MP_WritePhyUshort(sc, 0x06, 0xaf81);
			MP_WritePhyUshort(sc, 0x06, 0xa3af);
			MP_WritePhyUshort(sc, 0x06, 0x81dc);
			MP_WritePhyUshort(sc, 0x06, 0xaf82);
			MP_WritePhyUshort(sc, 0x06, 0x13ee);
			MP_WritePhyUshort(sc, 0x06, 0x8348);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0x8349);
			MP_WritePhyUshort(sc, 0x06, 0x00e0);
			MP_WritePhyUshort(sc, 0x06, 0x8351);
			MP_WritePhyUshort(sc, 0x06, 0x10e4);
			MP_WritePhyUshort(sc, 0x06, 0x8351);
			MP_WritePhyUshort(sc, 0x06, 0x5801);
			MP_WritePhyUshort(sc, 0x06, 0x9fea);
			MP_WritePhyUshort(sc, 0x06, 0xd000);
			MP_WritePhyUshort(sc, 0x06, 0xd180);
			MP_WritePhyUshort(sc, 0x06, 0x1f66);
			MP_WritePhyUshort(sc, 0x06, 0xe2f8);
			MP_WritePhyUshort(sc, 0x06, 0xeae3);
			MP_WritePhyUshort(sc, 0x06, 0xf8eb);
			MP_WritePhyUshort(sc, 0x06, 0x5af8);
			MP_WritePhyUshort(sc, 0x06, 0x1e20);
			MP_WritePhyUshort(sc, 0x06, 0xe6f8);
			MP_WritePhyUshort(sc, 0x06, 0xeae5);
			MP_WritePhyUshort(sc, 0x06, 0xf8eb);
			MP_WritePhyUshort(sc, 0x06, 0xd302);
			MP_WritePhyUshort(sc, 0x06, 0xb3fe);
			MP_WritePhyUshort(sc, 0x06, 0xe2f8);
			MP_WritePhyUshort(sc, 0x06, 0x7cef);
			MP_WritePhyUshort(sc, 0x06, 0x325b);
			MP_WritePhyUshort(sc, 0x06, 0x80e3);
			MP_WritePhyUshort(sc, 0x06, 0xf87d);
			MP_WritePhyUshort(sc, 0x06, 0x9e03);
			MP_WritePhyUshort(sc, 0x06, 0x7dff);
			MP_WritePhyUshort(sc, 0x06, 0xff0d);
			MP_WritePhyUshort(sc, 0x06, 0x581c);
			MP_WritePhyUshort(sc, 0x06, 0x551a);
			MP_WritePhyUshort(sc, 0x06, 0x6511);
			MP_WritePhyUshort(sc, 0x06, 0xa190);
			MP_WritePhyUshort(sc, 0x06, 0xd3e2);
			MP_WritePhyUshort(sc, 0x06, 0x8348);
			MP_WritePhyUshort(sc, 0x06, 0xe383);
			MP_WritePhyUshort(sc, 0x06, 0x491b);
			MP_WritePhyUshort(sc, 0x06, 0x56ab);
			MP_WritePhyUshort(sc, 0x06, 0x08ef);
			MP_WritePhyUshort(sc, 0x06, 0x56e6);
			MP_WritePhyUshort(sc, 0x06, 0x8348);
			MP_WritePhyUshort(sc, 0x06, 0xe783);
			MP_WritePhyUshort(sc, 0x06, 0x4910);
			MP_WritePhyUshort(sc, 0x06, 0xd180);
			MP_WritePhyUshort(sc, 0x06, 0x1f66);
			MP_WritePhyUshort(sc, 0x06, 0xa004);
			MP_WritePhyUshort(sc, 0x06, 0xb9e2);
			MP_WritePhyUshort(sc, 0x06, 0x8348);
			MP_WritePhyUshort(sc, 0x06, 0xe383);
			MP_WritePhyUshort(sc, 0x06, 0x49ef);
			MP_WritePhyUshort(sc, 0x06, 0x65e2);
			MP_WritePhyUshort(sc, 0x06, 0x834a);
			MP_WritePhyUshort(sc, 0x06, 0xe383);
			MP_WritePhyUshort(sc, 0x06, 0x4b1b);
			MP_WritePhyUshort(sc, 0x06, 0x56aa);
			MP_WritePhyUshort(sc, 0x06, 0x0eef);
			MP_WritePhyUshort(sc, 0x06, 0x56e6);
			MP_WritePhyUshort(sc, 0x06, 0x834a);
			MP_WritePhyUshort(sc, 0x06, 0xe783);
			MP_WritePhyUshort(sc, 0x06, 0x4be2);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0xe683);
			MP_WritePhyUshort(sc, 0x06, 0x4ce0);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0xa000);
			MP_WritePhyUshort(sc, 0x06, 0x0caf);
			MP_WritePhyUshort(sc, 0x06, 0x81dc);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4d10);
			MP_WritePhyUshort(sc, 0x06, 0xe483);
			MP_WritePhyUshort(sc, 0x06, 0x4dae);
			MP_WritePhyUshort(sc, 0x06, 0x0480);
			MP_WritePhyUshort(sc, 0x06, 0xe483);
			MP_WritePhyUshort(sc, 0x06, 0x4de0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7803);
			MP_WritePhyUshort(sc, 0x06, 0x9e0b);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x049e);
			MP_WritePhyUshort(sc, 0x06, 0x04ee);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x02e0);
			MP_WritePhyUshort(sc, 0x06, 0x8332);
			MP_WritePhyUshort(sc, 0x06, 0xe183);
			MP_WritePhyUshort(sc, 0x06, 0x3359);
			MP_WritePhyUshort(sc, 0x06, 0x0fe2);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0x0c24);
			MP_WritePhyUshort(sc, 0x06, 0x5af0);
			MP_WritePhyUshort(sc, 0x06, 0x1e12);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x8ce5);
			MP_WritePhyUshort(sc, 0x06, 0xf88d);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x30e1);
			MP_WritePhyUshort(sc, 0x06, 0x8331);
			MP_WritePhyUshort(sc, 0x06, 0x6801);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x8ae5);
			MP_WritePhyUshort(sc, 0x06, 0xf88b);
			MP_WritePhyUshort(sc, 0x06, 0xae37);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4e03);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4ce1);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0x1b01);
			MP_WritePhyUshort(sc, 0x06, 0x9e04);
			MP_WritePhyUshort(sc, 0x06, 0xaaa1);
			MP_WritePhyUshort(sc, 0x06, 0xaea8);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4e04);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4f00);
			MP_WritePhyUshort(sc, 0x06, 0xaeab);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4f78);
			MP_WritePhyUshort(sc, 0x06, 0x039f);
			MP_WritePhyUshort(sc, 0x06, 0x14ee);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x05d2);
			MP_WritePhyUshort(sc, 0x06, 0x40d6);
			MP_WritePhyUshort(sc, 0x06, 0x5554);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0x17d2);
			MP_WritePhyUshort(sc, 0x06, 0xa0d6);
			MP_WritePhyUshort(sc, 0x06, 0xba00);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0x17fe);
			MP_WritePhyUshort(sc, 0x06, 0xfdfc);
			MP_WritePhyUshort(sc, 0x06, 0x05f8);
			MP_WritePhyUshort(sc, 0x06, 0xe0f8);
			MP_WritePhyUshort(sc, 0x06, 0x60e1);
			MP_WritePhyUshort(sc, 0x06, 0xf861);
			MP_WritePhyUshort(sc, 0x06, 0x6802);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x60e5);
			MP_WritePhyUshort(sc, 0x06, 0xf861);
			MP_WritePhyUshort(sc, 0x06, 0xe0f8);
			MP_WritePhyUshort(sc, 0x06, 0x48e1);
			MP_WritePhyUshort(sc, 0x06, 0xf849);
			MP_WritePhyUshort(sc, 0x06, 0x580f);
			MP_WritePhyUshort(sc, 0x06, 0x1e02);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x48e5);
			MP_WritePhyUshort(sc, 0x06, 0xf849);
			MP_WritePhyUshort(sc, 0x06, 0xd000);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0x5bbf);
			MP_WritePhyUshort(sc, 0x06, 0x8350);
			MP_WritePhyUshort(sc, 0x06, 0xef46);
			MP_WritePhyUshort(sc, 0x06, 0xdc19);
			MP_WritePhyUshort(sc, 0x06, 0xddd0);
			MP_WritePhyUshort(sc, 0x06, 0x0102);
			MP_WritePhyUshort(sc, 0x06, 0x825b);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0x77e0);
			MP_WritePhyUshort(sc, 0x06, 0xf860);
			MP_WritePhyUshort(sc, 0x06, 0xe1f8);
			MP_WritePhyUshort(sc, 0x06, 0x6158);
			MP_WritePhyUshort(sc, 0x06, 0xfde4);
			MP_WritePhyUshort(sc, 0x06, 0xf860);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x61fc);
			MP_WritePhyUshort(sc, 0x06, 0x04f9);
			MP_WritePhyUshort(sc, 0x06, 0xfafb);
			MP_WritePhyUshort(sc, 0x06, 0xc6bf);
			MP_WritePhyUshort(sc, 0x06, 0xf840);
			MP_WritePhyUshort(sc, 0x06, 0xbe83);
			MP_WritePhyUshort(sc, 0x06, 0x50a0);
			MP_WritePhyUshort(sc, 0x06, 0x0101);
			MP_WritePhyUshort(sc, 0x06, 0x071b);
			MP_WritePhyUshort(sc, 0x06, 0x89cf);
			MP_WritePhyUshort(sc, 0x06, 0xd208);
			MP_WritePhyUshort(sc, 0x06, 0xebdb);
			MP_WritePhyUshort(sc, 0x06, 0x19b2);
			MP_WritePhyUshort(sc, 0x06, 0xfbff);
			MP_WritePhyUshort(sc, 0x06, 0xfefd);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xe0f8);
			MP_WritePhyUshort(sc, 0x06, 0x48e1);
			MP_WritePhyUshort(sc, 0x06, 0xf849);
			MP_WritePhyUshort(sc, 0x06, 0x6808);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x48e5);
			MP_WritePhyUshort(sc, 0x06, 0xf849);
			MP_WritePhyUshort(sc, 0x06, 0x58f7);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x48e5);
			MP_WritePhyUshort(sc, 0x06, 0xf849);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0x4d20);
			MP_WritePhyUshort(sc, 0x06, 0x0002);
			MP_WritePhyUshort(sc, 0x06, 0x4e22);
			MP_WritePhyUshort(sc, 0x06, 0x0002);
			MP_WritePhyUshort(sc, 0x06, 0x4ddf);
			MP_WritePhyUshort(sc, 0x06, 0xff01);
			MP_WritePhyUshort(sc, 0x06, 0x4edd);
			MP_WritePhyUshort(sc, 0x06, 0xff01);
			MP_WritePhyUshort(sc, 0x06, 0xf8fa);
			MP_WritePhyUshort(sc, 0x06, 0xfbef);
			MP_WritePhyUshort(sc, 0x06, 0x79bf);
			MP_WritePhyUshort(sc, 0x06, 0xf822);
			MP_WritePhyUshort(sc, 0x06, 0xd819);
			MP_WritePhyUshort(sc, 0x06, 0xd958);
			MP_WritePhyUshort(sc, 0x06, 0x849f);
			MP_WritePhyUshort(sc, 0x06, 0x09bf);
			MP_WritePhyUshort(sc, 0x06, 0x82be);
			MP_WritePhyUshort(sc, 0x06, 0xd682);
			MP_WritePhyUshort(sc, 0x06, 0xc602);
			MP_WritePhyUshort(sc, 0x06, 0x014f);
			MP_WritePhyUshort(sc, 0x06, 0xef97);
			MP_WritePhyUshort(sc, 0x06, 0xfffe);
			MP_WritePhyUshort(sc, 0x06, 0xfc05);
			MP_WritePhyUshort(sc, 0x06, 0x17ff);
			MP_WritePhyUshort(sc, 0x06, 0xfe01);
			MP_WritePhyUshort(sc, 0x06, 0x1700);
			MP_WritePhyUshort(sc, 0x06, 0x0102);
			MP_WritePhyUshort(sc, 0x05, 0x83d8);
			MP_WritePhyUshort(sc, 0x06, 0x8051);
			MP_WritePhyUshort(sc, 0x05, 0x83d6);
			MP_WritePhyUshort(sc, 0x06, 0x82a0);
			MP_WritePhyUshort(sc, 0x05, 0x83d4);
			MP_WritePhyUshort(sc, 0x06, 0x8000);
			MP_WritePhyUshort(sc, 0x02, 0x2010);
			MP_WritePhyUshort(sc, 0x03, 0xdc00);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x0b, 0x0600);
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x00fc);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x0D, 0xF880);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_32) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0x4064);
		MP_WritePhyUshort(sc, 0x07, 0x2863);
		MP_WritePhyUshort(sc, 0x08, 0x059C);
		MP_WritePhyUshort(sc, 0x09, 0x26B4);
		MP_WritePhyUshort(sc, 0x0A, 0x6A19);
		MP_WritePhyUshort(sc, 0x0B, 0xBCC0);
		MP_WritePhyUshort(sc, 0x10, 0xF06D);
		MP_WritePhyUshort(sc, 0x14, 0x7F68);
		MP_WritePhyUshort(sc, 0x18, 0x7FD9);
		MP_WritePhyUshort(sc, 0x1C, 0xF0FF);
		MP_WritePhyUshort(sc, 0x1D, 0x3D9C);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0xF49F);
		MP_WritePhyUshort(sc, 0x13, 0x070B);
		MP_WritePhyUshort(sc, 0x1A, 0x05AD);
		MP_WritePhyUshort(sc, 0x14, 0x94C0);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x5571);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x05, 0x2642);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x02, 0xC107);
		MP_WritePhyUshort(sc, 0x03, 0x1002);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x16, 0x0CC0);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0F, 0x0017);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8200);
		MP_WritePhyUshort(sc, 0x06, 0xF8F9);
		MP_WritePhyUshort(sc, 0x06, 0xFAEF);
		MP_WritePhyUshort(sc, 0x06, 0x59EE);
		MP_WritePhyUshort(sc, 0x06, 0xF8EA);
		MP_WritePhyUshort(sc, 0x06, 0x00EE);
		MP_WritePhyUshort(sc, 0x06, 0xF8EB);
		MP_WritePhyUshort(sc, 0x06, 0x00E0);
		MP_WritePhyUshort(sc, 0x06, 0xF87C);
		MP_WritePhyUshort(sc, 0x06, 0xE1F8);
		MP_WritePhyUshort(sc, 0x06, 0x7D59);
		MP_WritePhyUshort(sc, 0x06, 0x0FEF);
		MP_WritePhyUshort(sc, 0x06, 0x0139);
		MP_WritePhyUshort(sc, 0x06, 0x029E);
		MP_WritePhyUshort(sc, 0x06, 0x06EF);
		MP_WritePhyUshort(sc, 0x06, 0x1039);
		MP_WritePhyUshort(sc, 0x06, 0x089F);
		MP_WritePhyUshort(sc, 0x06, 0x2AEE);
		MP_WritePhyUshort(sc, 0x06, 0xF8EA);
		MP_WritePhyUshort(sc, 0x06, 0x00EE);
		MP_WritePhyUshort(sc, 0x06, 0xF8EB);
		MP_WritePhyUshort(sc, 0x06, 0x01E0);
		MP_WritePhyUshort(sc, 0x06, 0xF87C);
		MP_WritePhyUshort(sc, 0x06, 0xE1F8);
		MP_WritePhyUshort(sc, 0x06, 0x7D58);
		MP_WritePhyUshort(sc, 0x06, 0x409E);
		MP_WritePhyUshort(sc, 0x06, 0x0F39);
		MP_WritePhyUshort(sc, 0x06, 0x46AA);
		MP_WritePhyUshort(sc, 0x06, 0x0BBF);
		MP_WritePhyUshort(sc, 0x06, 0x8251);
		MP_WritePhyUshort(sc, 0x06, 0xD682);
		MP_WritePhyUshort(sc, 0x06, 0x5902);
		MP_WritePhyUshort(sc, 0x06, 0x014F);
		MP_WritePhyUshort(sc, 0x06, 0xAE09);
		MP_WritePhyUshort(sc, 0x06, 0xBF82);
		MP_WritePhyUshort(sc, 0x06, 0x59D6);
		MP_WritePhyUshort(sc, 0x06, 0x8261);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x4FEF);
		MP_WritePhyUshort(sc, 0x06, 0x95FE);
		MP_WritePhyUshort(sc, 0x06, 0xFDFC);
		MP_WritePhyUshort(sc, 0x06, 0x054D);
		MP_WritePhyUshort(sc, 0x06, 0x2000);
		MP_WritePhyUshort(sc, 0x06, 0x024E);
		MP_WritePhyUshort(sc, 0x06, 0x2200);
		MP_WritePhyUshort(sc, 0x06, 0x024D);
		MP_WritePhyUshort(sc, 0x06, 0xDFFF);
		MP_WritePhyUshort(sc, 0x06, 0x014E);
		MP_WritePhyUshort(sc, 0x06, 0xDDFF);
		MP_WritePhyUshort(sc, 0x06, 0x0100);
		MP_WritePhyUshort(sc, 0x02, 0x6010);
		MP_WritePhyUshort(sc, 0x05, 0xFFF6);
		MP_WritePhyUshort(sc, 0x06, 0x00EC);
		MP_WritePhyUshort(sc, 0x05, 0x83D4);
		MP_WritePhyUshort(sc, 0x06, 0x8200);

	} else if (sc->re_type == MACFG_33) {
		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0x4064);
		MP_WritePhyUshort(sc, 0x07, 0x2863);
		MP_WritePhyUshort(sc, 0x08, 0x059C);
		MP_WritePhyUshort(sc, 0x09, 0x26B4);
		MP_WritePhyUshort(sc, 0x0A, 0x6A19);
		MP_WritePhyUshort(sc, 0x0B, 0xDCC8);
		MP_WritePhyUshort(sc, 0x10, 0xF06D);
		MP_WritePhyUshort(sc, 0x14, 0x7F68);
		MP_WritePhyUshort(sc, 0x18, 0x7FD9);
		MP_WritePhyUshort(sc, 0x1C, 0xF0FF);
		MP_WritePhyUshort(sc, 0x1D, 0x3D9C);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x12, 0xF49F);
		MP_WritePhyUshort(sc, 0x13, 0x070B);
		MP_WritePhyUshort(sc, 0x1A, 0x05AD);
		MP_WritePhyUshort(sc, 0x14, 0x94C0);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x5561);
		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8332);
		MP_WritePhyUshort(sc, 0x06, 0x5561);

		if (MP_ReadEfuse(sc, 0x01) == 0xb1) {
			MP_WritePhyUshort(sc, 0x1F, 0x0002);
			MP_WritePhyUshort(sc, 0x05, 0x669A);
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0x8330);
			MP_WritePhyUshort(sc, 0x06, 0x669A);

			MP_WritePhyUshort(sc, 0x1F, 0x0002);
			Data = MP_ReadPhyUshort(sc, 0x0D);
			if ((Data & 0x00FF) != 0x006C) {
				Data &= 0xFF00;
				MP_WritePhyUshort(sc, 0x1F, 0x0002);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0065);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0066);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0067);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0068);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x0069);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x006A);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x006B);
				MP_WritePhyUshort(sc, 0x0D, Data | 0x006C);
			}
		} else {
			MP_WritePhyUshort(sc, 0x1F, 0x0002);
			MP_WritePhyUshort(sc, 0x05, 0x2642);
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0x8330);
			MP_WritePhyUshort(sc, 0x06, 0x2642);
		}

		if (MP_ReadEfuse(sc, 0x30) == 0x98) {
			MP_WritePhyUshort(sc, 0x1F, 0x0000);
			MP_WritePhyUshort(sc, 0x11, MP_ReadPhyUshort(sc, 0x11) & ~0x02);
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x01, MP_ReadPhyUshort(sc, 0x01) | 0x200);
		} else if (MP_ReadEfuse(sc, 0x30) == 0x90) {
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x01, MP_ReadPhyUshort(sc, 0x01) & ~0x200);
			MP_WritePhyUshort(sc, 0x1F, 0x0000);
			MP_WritePhyUshort(sc, 0x16, 0x5101);
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		Data = MP_ReadPhyUshort(sc, 0x02);
		Data &= ~0x600;
		Data |= 0x100;
		MP_WritePhyUshort(sc, 0x02, Data);
		Data = MP_ReadPhyUshort(sc, 0x03);
		Data &= ~0xE000;
		MP_WritePhyUshort(sc, 0x03, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x17, 0x0CC0);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		Data = MP_ReadPhyUshort(sc, 0x0F);
		Data |= 0x17;
		MP_WritePhyUshort(sc, 0x0F, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x001B);
		if (MP_ReadPhyUshort(sc, 0x06) == 0xB300) {
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x0080);
			MP_WritePhyUshort(sc, 0x05, 0x8000);
			MP_WritePhyUshort(sc, 0x06, 0xf8f9);
			MP_WritePhyUshort(sc, 0x06, 0xfaee);
			MP_WritePhyUshort(sc, 0x06, 0xf8ea);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0xf8eb);
			MP_WritePhyUshort(sc, 0x06, 0x00e2);
			MP_WritePhyUshort(sc, 0x06, 0xf87c);
			MP_WritePhyUshort(sc, 0x06, 0xe3f8);
			MP_WritePhyUshort(sc, 0x06, 0x7da5);
			MP_WritePhyUshort(sc, 0x06, 0x1111);
			MP_WritePhyUshort(sc, 0x06, 0x12d2);
			MP_WritePhyUshort(sc, 0x06, 0x40d6);
			MP_WritePhyUshort(sc, 0x06, 0x4444);
			MP_WritePhyUshort(sc, 0x06, 0x0281);
			MP_WritePhyUshort(sc, 0x06, 0xc6d2);
			MP_WritePhyUshort(sc, 0x06, 0xa0d6);
			MP_WritePhyUshort(sc, 0x06, 0xaaaa);
			MP_WritePhyUshort(sc, 0x06, 0x0281);
			MP_WritePhyUshort(sc, 0x06, 0xc6ae);
			MP_WritePhyUshort(sc, 0x06, 0x0fa5);
			MP_WritePhyUshort(sc, 0x06, 0x4444);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0x4da5);
			MP_WritePhyUshort(sc, 0x06, 0xaaaa);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0x47af);
			MP_WritePhyUshort(sc, 0x06, 0x81c2);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4e00);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4d0f);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4c0f);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4f00);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x5100);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4aff);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4bff);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x30e1);
			MP_WritePhyUshort(sc, 0x06, 0x8331);
			MP_WritePhyUshort(sc, 0x06, 0x58fe);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x8ae5);
			MP_WritePhyUshort(sc, 0x06, 0xf88b);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x32e1);
			MP_WritePhyUshort(sc, 0x06, 0x8333);
			MP_WritePhyUshort(sc, 0x06, 0x590f);
			MP_WritePhyUshort(sc, 0x06, 0xe283);
			MP_WritePhyUshort(sc, 0x06, 0x4d0c);
			MP_WritePhyUshort(sc, 0x06, 0x245a);
			MP_WritePhyUshort(sc, 0x06, 0xf01e);
			MP_WritePhyUshort(sc, 0x06, 0x12e4);
			MP_WritePhyUshort(sc, 0x06, 0xf88c);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x8daf);
			MP_WritePhyUshort(sc, 0x06, 0x81c2);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4f10);
			MP_WritePhyUshort(sc, 0x06, 0xe483);
			MP_WritePhyUshort(sc, 0x06, 0x4fe0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7800);
			MP_WritePhyUshort(sc, 0x06, 0x9f0a);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4fa0);
			MP_WritePhyUshort(sc, 0x06, 0x10a5);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4e01);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x059e);
			MP_WritePhyUshort(sc, 0x06, 0x9ae0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7804);
			MP_WritePhyUshort(sc, 0x06, 0x9e10);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x039e);
			MP_WritePhyUshort(sc, 0x06, 0x0fe0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7801);
			MP_WritePhyUshort(sc, 0x06, 0x9e05);
			MP_WritePhyUshort(sc, 0x06, 0xae0c);
			MP_WritePhyUshort(sc, 0x06, 0xaf81);
			MP_WritePhyUshort(sc, 0x06, 0xa7af);
			MP_WritePhyUshort(sc, 0x06, 0x8152);
			MP_WritePhyUshort(sc, 0x06, 0xaf81);
			MP_WritePhyUshort(sc, 0x06, 0x8baf);
			MP_WritePhyUshort(sc, 0x06, 0x81c2);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4800);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4900);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x5110);
			MP_WritePhyUshort(sc, 0x06, 0xe483);
			MP_WritePhyUshort(sc, 0x06, 0x5158);
			MP_WritePhyUshort(sc, 0x06, 0x019f);
			MP_WritePhyUshort(sc, 0x06, 0xead0);
			MP_WritePhyUshort(sc, 0x06, 0x00d1);
			MP_WritePhyUshort(sc, 0x06, 0x801f);
			MP_WritePhyUshort(sc, 0x06, 0x66e2);
			MP_WritePhyUshort(sc, 0x06, 0xf8ea);
			MP_WritePhyUshort(sc, 0x06, 0xe3f8);
			MP_WritePhyUshort(sc, 0x06, 0xeb5a);
			MP_WritePhyUshort(sc, 0x06, 0xf81e);
			MP_WritePhyUshort(sc, 0x06, 0x20e6);
			MP_WritePhyUshort(sc, 0x06, 0xf8ea);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0xebd3);
			MP_WritePhyUshort(sc, 0x06, 0x02b3);
			MP_WritePhyUshort(sc, 0x06, 0xfee2);
			MP_WritePhyUshort(sc, 0x06, 0xf87c);
			MP_WritePhyUshort(sc, 0x06, 0xef32);
			MP_WritePhyUshort(sc, 0x06, 0x5b80);
			MP_WritePhyUshort(sc, 0x06, 0xe3f8);
			MP_WritePhyUshort(sc, 0x06, 0x7d9e);
			MP_WritePhyUshort(sc, 0x06, 0x037d);
			MP_WritePhyUshort(sc, 0x06, 0xffff);
			MP_WritePhyUshort(sc, 0x06, 0x0d58);
			MP_WritePhyUshort(sc, 0x06, 0x1c55);
			MP_WritePhyUshort(sc, 0x06, 0x1a65);
			MP_WritePhyUshort(sc, 0x06, 0x11a1);
			MP_WritePhyUshort(sc, 0x06, 0x90d3);
			MP_WritePhyUshort(sc, 0x06, 0xe283);
			MP_WritePhyUshort(sc, 0x06, 0x48e3);
			MP_WritePhyUshort(sc, 0x06, 0x8349);
			MP_WritePhyUshort(sc, 0x06, 0x1b56);
			MP_WritePhyUshort(sc, 0x06, 0xab08);
			MP_WritePhyUshort(sc, 0x06, 0xef56);
			MP_WritePhyUshort(sc, 0x06, 0xe683);
			MP_WritePhyUshort(sc, 0x06, 0x48e7);
			MP_WritePhyUshort(sc, 0x06, 0x8349);
			MP_WritePhyUshort(sc, 0x06, 0x10d1);
			MP_WritePhyUshort(sc, 0x06, 0x801f);
			MP_WritePhyUshort(sc, 0x06, 0x66a0);
			MP_WritePhyUshort(sc, 0x06, 0x04b9);
			MP_WritePhyUshort(sc, 0x06, 0xe283);
			MP_WritePhyUshort(sc, 0x06, 0x48e3);
			MP_WritePhyUshort(sc, 0x06, 0x8349);
			MP_WritePhyUshort(sc, 0x06, 0xef65);
			MP_WritePhyUshort(sc, 0x06, 0xe283);
			MP_WritePhyUshort(sc, 0x06, 0x4ae3);
			MP_WritePhyUshort(sc, 0x06, 0x834b);
			MP_WritePhyUshort(sc, 0x06, 0x1b56);
			MP_WritePhyUshort(sc, 0x06, 0xaa0e);
			MP_WritePhyUshort(sc, 0x06, 0xef56);
			MP_WritePhyUshort(sc, 0x06, 0xe683);
			MP_WritePhyUshort(sc, 0x06, 0x4ae7);
			MP_WritePhyUshort(sc, 0x06, 0x834b);
			MP_WritePhyUshort(sc, 0x06, 0xe283);
			MP_WritePhyUshort(sc, 0x06, 0x4de6);
			MP_WritePhyUshort(sc, 0x06, 0x834c);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4da0);
			MP_WritePhyUshort(sc, 0x06, 0x000c);
			MP_WritePhyUshort(sc, 0x06, 0xaf81);
			MP_WritePhyUshort(sc, 0x06, 0x8be0);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0x10e4);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0xae04);
			MP_WritePhyUshort(sc, 0x06, 0x80e4);
			MP_WritePhyUshort(sc, 0x06, 0x834d);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x4e78);
			MP_WritePhyUshort(sc, 0x06, 0x039e);
			MP_WritePhyUshort(sc, 0x06, 0x0be0);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x7804);
			MP_WritePhyUshort(sc, 0x06, 0x9e04);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4e02);
			MP_WritePhyUshort(sc, 0x06, 0xe083);
			MP_WritePhyUshort(sc, 0x06, 0x32e1);
			MP_WritePhyUshort(sc, 0x06, 0x8333);
			MP_WritePhyUshort(sc, 0x06, 0x590f);
			MP_WritePhyUshort(sc, 0x06, 0xe283);
			MP_WritePhyUshort(sc, 0x06, 0x4d0c);
			MP_WritePhyUshort(sc, 0x06, 0x245a);
			MP_WritePhyUshort(sc, 0x06, 0xf01e);
			MP_WritePhyUshort(sc, 0x06, 0x12e4);
			MP_WritePhyUshort(sc, 0x06, 0xf88c);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x8de0);
			MP_WritePhyUshort(sc, 0x06, 0x8330);
			MP_WritePhyUshort(sc, 0x06, 0xe183);
			MP_WritePhyUshort(sc, 0x06, 0x3168);
			MP_WritePhyUshort(sc, 0x06, 0x01e4);
			MP_WritePhyUshort(sc, 0x06, 0xf88a);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x8bae);
			MP_WritePhyUshort(sc, 0x06, 0x37ee);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x03e0);
			MP_WritePhyUshort(sc, 0x06, 0x834c);
			MP_WritePhyUshort(sc, 0x06, 0xe183);
			MP_WritePhyUshort(sc, 0x06, 0x4d1b);
			MP_WritePhyUshort(sc, 0x06, 0x019e);
			MP_WritePhyUshort(sc, 0x06, 0x04aa);
			MP_WritePhyUshort(sc, 0x06, 0xa1ae);
			MP_WritePhyUshort(sc, 0x06, 0xa8ee);
			MP_WritePhyUshort(sc, 0x06, 0x834e);
			MP_WritePhyUshort(sc, 0x06, 0x04ee);
			MP_WritePhyUshort(sc, 0x06, 0x834f);
			MP_WritePhyUshort(sc, 0x06, 0x00ae);
			MP_WritePhyUshort(sc, 0x06, 0xabe0);
			MP_WritePhyUshort(sc, 0x06, 0x834f);
			MP_WritePhyUshort(sc, 0x06, 0x7803);
			MP_WritePhyUshort(sc, 0x06, 0x9f14);
			MP_WritePhyUshort(sc, 0x06, 0xee83);
			MP_WritePhyUshort(sc, 0x06, 0x4e05);
			MP_WritePhyUshort(sc, 0x06, 0xd240);
			MP_WritePhyUshort(sc, 0x06, 0xd655);
			MP_WritePhyUshort(sc, 0x06, 0x5402);
			MP_WritePhyUshort(sc, 0x06, 0x81c6);
			MP_WritePhyUshort(sc, 0x06, 0xd2a0);
			MP_WritePhyUshort(sc, 0x06, 0xd6ba);
			MP_WritePhyUshort(sc, 0x06, 0x0002);
			MP_WritePhyUshort(sc, 0x06, 0x81c6);
			MP_WritePhyUshort(sc, 0x06, 0xfefd);
			MP_WritePhyUshort(sc, 0x06, 0xfc05);
			MP_WritePhyUshort(sc, 0x06, 0xf8e0);
			MP_WritePhyUshort(sc, 0x06, 0xf860);
			MP_WritePhyUshort(sc, 0x06, 0xe1f8);
			MP_WritePhyUshort(sc, 0x06, 0x6168);
			MP_WritePhyUshort(sc, 0x06, 0x02e4);
			MP_WritePhyUshort(sc, 0x06, 0xf860);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x61e0);
			MP_WritePhyUshort(sc, 0x06, 0xf848);
			MP_WritePhyUshort(sc, 0x06, 0xe1f8);
			MP_WritePhyUshort(sc, 0x06, 0x4958);
			MP_WritePhyUshort(sc, 0x06, 0x0f1e);
			MP_WritePhyUshort(sc, 0x06, 0x02e4);
			MP_WritePhyUshort(sc, 0x06, 0xf848);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x49d0);
			MP_WritePhyUshort(sc, 0x06, 0x0002);
			MP_WritePhyUshort(sc, 0x06, 0x820a);
			MP_WritePhyUshort(sc, 0x06, 0xbf83);
			MP_WritePhyUshort(sc, 0x06, 0x50ef);
			MP_WritePhyUshort(sc, 0x06, 0x46dc);
			MP_WritePhyUshort(sc, 0x06, 0x19dd);
			MP_WritePhyUshort(sc, 0x06, 0xd001);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0x0a02);
			MP_WritePhyUshort(sc, 0x06, 0x8226);
			MP_WritePhyUshort(sc, 0x06, 0xe0f8);
			MP_WritePhyUshort(sc, 0x06, 0x60e1);
			MP_WritePhyUshort(sc, 0x06, 0xf861);
			MP_WritePhyUshort(sc, 0x06, 0x58fd);
			MP_WritePhyUshort(sc, 0x06, 0xe4f8);
			MP_WritePhyUshort(sc, 0x06, 0x60e5);
			MP_WritePhyUshort(sc, 0x06, 0xf861);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xf9fa);
			MP_WritePhyUshort(sc, 0x06, 0xfbc6);
			MP_WritePhyUshort(sc, 0x06, 0xbff8);
			MP_WritePhyUshort(sc, 0x06, 0x40be);
			MP_WritePhyUshort(sc, 0x06, 0x8350);
			MP_WritePhyUshort(sc, 0x06, 0xa001);
			MP_WritePhyUshort(sc, 0x06, 0x0107);
			MP_WritePhyUshort(sc, 0x06, 0x1b89);
			MP_WritePhyUshort(sc, 0x06, 0xcfd2);
			MP_WritePhyUshort(sc, 0x06, 0x08eb);
			MP_WritePhyUshort(sc, 0x06, 0xdb19);
			MP_WritePhyUshort(sc, 0x06, 0xb2fb);
			MP_WritePhyUshort(sc, 0x06, 0xfffe);
			MP_WritePhyUshort(sc, 0x06, 0xfd04);
			MP_WritePhyUshort(sc, 0x06, 0xf8e0);
			MP_WritePhyUshort(sc, 0x06, 0xf848);
			MP_WritePhyUshort(sc, 0x06, 0xe1f8);
			MP_WritePhyUshort(sc, 0x06, 0x4968);
			MP_WritePhyUshort(sc, 0x06, 0x08e4);
			MP_WritePhyUshort(sc, 0x06, 0xf848);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x4958);
			MP_WritePhyUshort(sc, 0x06, 0xf7e4);
			MP_WritePhyUshort(sc, 0x06, 0xf848);
			MP_WritePhyUshort(sc, 0x06, 0xe5f8);
			MP_WritePhyUshort(sc, 0x06, 0x49fc);
			MP_WritePhyUshort(sc, 0x06, 0x044d);
			MP_WritePhyUshort(sc, 0x06, 0x2000);
			MP_WritePhyUshort(sc, 0x06, 0x024e);
			MP_WritePhyUshort(sc, 0x06, 0x2200);
			MP_WritePhyUshort(sc, 0x06, 0x024d);
			MP_WritePhyUshort(sc, 0x06, 0xdfff);
			MP_WritePhyUshort(sc, 0x06, 0x014e);
			MP_WritePhyUshort(sc, 0x06, 0xddff);
			MP_WritePhyUshort(sc, 0x06, 0x01f8);
			MP_WritePhyUshort(sc, 0x06, 0xfafb);
			MP_WritePhyUshort(sc, 0x06, 0xef79);
			MP_WritePhyUshort(sc, 0x06, 0xbff8);
			MP_WritePhyUshort(sc, 0x06, 0x22d8);
			MP_WritePhyUshort(sc, 0x06, 0x19d9);
			MP_WritePhyUshort(sc, 0x06, 0x5884);
			MP_WritePhyUshort(sc, 0x06, 0x9f09);
			MP_WritePhyUshort(sc, 0x06, 0xbf82);
			MP_WritePhyUshort(sc, 0x06, 0x6dd6);
			MP_WritePhyUshort(sc, 0x06, 0x8275);
			MP_WritePhyUshort(sc, 0x06, 0x0201);
			MP_WritePhyUshort(sc, 0x06, 0x4fef);
			MP_WritePhyUshort(sc, 0x06, 0x97ff);
			MP_WritePhyUshort(sc, 0x06, 0xfefc);
			MP_WritePhyUshort(sc, 0x06, 0x0517);
			MP_WritePhyUshort(sc, 0x06, 0xfffe);
			MP_WritePhyUshort(sc, 0x06, 0x0117);
			MP_WritePhyUshort(sc, 0x06, 0x0001);
			MP_WritePhyUshort(sc, 0x06, 0x0200);
			MP_WritePhyUshort(sc, 0x05, 0x83d8);
			MP_WritePhyUshort(sc, 0x06, 0x8000);
			MP_WritePhyUshort(sc, 0x05, 0x83d6);
			MP_WritePhyUshort(sc, 0x06, 0x824f);
			MP_WritePhyUshort(sc, 0x02, 0x2010);
			MP_WritePhyUshort(sc, 0x03, 0xdc00);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x0b, 0x0600);
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x00fc);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x0D, 0xF880);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_36 || sc->re_type == MACFG_37) {
		CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3)|0x04);

		if (sc->re_type == MACFG_36)
		{
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x00, 0x1800);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			MP_WritePhyUshort(sc, 0x17, 0x0117);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1E, 0x002C);
			MP_WritePhyUshort(sc, 0x1B, 0x5000);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x16, 0x4104);
			for (i = 0; i < 200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x1E);
				Data &= 0x03FF;
				if (Data== 0x000C)
					break;
			}
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			for (i = 0; i < 200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x07);
				if ((Data & 0x0020)==0)
					break;
			}
			Data = MP_ReadPhyUshort(sc, 0x07);
			if (Data & 0x0020)
			{
				MP_WritePhyUshort(sc, 0x1f, 0x0007);
				MP_WritePhyUshort(sc, 0x1e, 0x00a1);
				MP_WritePhyUshort(sc, 0x17, 0x1000);
				MP_WritePhyUshort(sc, 0x17, 0x0000);
				MP_WritePhyUshort(sc, 0x17, 0x2000);
				MP_WritePhyUshort(sc, 0x1e, 0x002f);
				MP_WritePhyUshort(sc, 0x18, 0x9bfb);
				MP_WritePhyUshort(sc, 0x1f, 0x0005);
				MP_WritePhyUshort(sc, 0x07, 0x0000);
				MP_WritePhyUshort(sc, 0x1f, 0x0000);
			}
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x0080);
			Data = MP_ReadPhyUshort(sc, 0x00);
			Data &= ~(0x0080);
			MP_WritePhyUshort(sc, 0x00, Data);
			MP_WritePhyUshort(sc, 0x1f, 0x0002);
			Data = MP_ReadPhyUshort(sc, 0x08);
			Data &= ~(0x0080);
			MP_WritePhyUshort(sc, 0x08, Data);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			MP_WritePhyUshort(sc, 0x16, 0x0306);
			MP_WritePhyUshort(sc, 0x16, 0x0307);
			MP_WritePhyUshort(sc, 0x15, 0x000e);
			MP_WritePhyUshort(sc, 0x19, 0x000a);
			MP_WritePhyUshort(sc, 0x15, 0x0010);
			MP_WritePhyUshort(sc, 0x19, 0x0008);
			MP_WritePhyUshort(sc, 0x15, 0x0018);
			MP_WritePhyUshort(sc, 0x19, 0x4801);
			MP_WritePhyUshort(sc, 0x15, 0x0019);
			MP_WritePhyUshort(sc, 0x19, 0x6801);
			MP_WritePhyUshort(sc, 0x15, 0x001a);
			MP_WritePhyUshort(sc, 0x19, 0x66a1);
			MP_WritePhyUshort(sc, 0x15, 0x001f);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0020);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0021);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0022);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0023);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0024);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0025);
			MP_WritePhyUshort(sc, 0x19, 0x64a1);
			MP_WritePhyUshort(sc, 0x15, 0x0026);
			MP_WritePhyUshort(sc, 0x19, 0x40ea);
			MP_WritePhyUshort(sc, 0x15, 0x0027);
			MP_WritePhyUshort(sc, 0x19, 0x4503);
			MP_WritePhyUshort(sc, 0x15, 0x0028);
			MP_WritePhyUshort(sc, 0x19, 0x9f00);
			MP_WritePhyUshort(sc, 0x15, 0x0029);
			MP_WritePhyUshort(sc, 0x19, 0xa631);
			MP_WritePhyUshort(sc, 0x15, 0x002a);
			MP_WritePhyUshort(sc, 0x19, 0x9717);
			MP_WritePhyUshort(sc, 0x15, 0x002b);
			MP_WritePhyUshort(sc, 0x19, 0x302c);
			MP_WritePhyUshort(sc, 0x15, 0x002c);
			MP_WritePhyUshort(sc, 0x19, 0x4802);
			MP_WritePhyUshort(sc, 0x15, 0x002d);
			MP_WritePhyUshort(sc, 0x19, 0x58da);
			MP_WritePhyUshort(sc, 0x15, 0x002e);
			MP_WritePhyUshort(sc, 0x19, 0x400d);
			MP_WritePhyUshort(sc, 0x15, 0x002f);
			MP_WritePhyUshort(sc, 0x19, 0x4488);
			MP_WritePhyUshort(sc, 0x15, 0x0030);
			MP_WritePhyUshort(sc, 0x19, 0x9e00);
			MP_WritePhyUshort(sc, 0x15, 0x0031);
			MP_WritePhyUshort(sc, 0x19, 0x63c8);
			MP_WritePhyUshort(sc, 0x15, 0x0032);
			MP_WritePhyUshort(sc, 0x19, 0x6481);
			MP_WritePhyUshort(sc, 0x15, 0x0033);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0034);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0035);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0036);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0037);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0038);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0039);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x003a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x003b);
			MP_WritePhyUshort(sc, 0x19, 0x63e8);
			MP_WritePhyUshort(sc, 0x15, 0x003c);
			MP_WritePhyUshort(sc, 0x19, 0x7d00);
			MP_WritePhyUshort(sc, 0x15, 0x003d);
			MP_WritePhyUshort(sc, 0x19, 0x59d4);
			MP_WritePhyUshort(sc, 0x15, 0x003e);
			MP_WritePhyUshort(sc, 0x19, 0x63f8);
			MP_WritePhyUshort(sc, 0x15, 0x0040);
			MP_WritePhyUshort(sc, 0x19, 0x64a1);
			MP_WritePhyUshort(sc, 0x15, 0x0041);
			MP_WritePhyUshort(sc, 0x19, 0x30de);
			MP_WritePhyUshort(sc, 0x15, 0x0044);
			MP_WritePhyUshort(sc, 0x19, 0x480f);
			MP_WritePhyUshort(sc, 0x15, 0x0045);
			MP_WritePhyUshort(sc, 0x19, 0x6800);
			MP_WritePhyUshort(sc, 0x15, 0x0046);
			MP_WritePhyUshort(sc, 0x19, 0x6680);
			MP_WritePhyUshort(sc, 0x15, 0x0047);
			MP_WritePhyUshort(sc, 0x19, 0x7c10);
			MP_WritePhyUshort(sc, 0x15, 0x0048);
			MP_WritePhyUshort(sc, 0x19, 0x63c8);
			MP_WritePhyUshort(sc, 0x15, 0x0049);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004b);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004c);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004d);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004e);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004f);
			MP_WritePhyUshort(sc, 0x19, 0x40ea);
			MP_WritePhyUshort(sc, 0x15, 0x0050);
			MP_WritePhyUshort(sc, 0x19, 0x4503);
			MP_WritePhyUshort(sc, 0x15, 0x0051);
			MP_WritePhyUshort(sc, 0x19, 0x58ca);
			MP_WritePhyUshort(sc, 0x15, 0x0052);
			MP_WritePhyUshort(sc, 0x19, 0x63c8);
			MP_WritePhyUshort(sc, 0x15, 0x0053);
			MP_WritePhyUshort(sc, 0x19, 0x63d8);
			MP_WritePhyUshort(sc, 0x15, 0x0054);
			MP_WritePhyUshort(sc, 0x19, 0x66a0);
			MP_WritePhyUshort(sc, 0x15, 0x0055);
			MP_WritePhyUshort(sc, 0x19, 0x9f00);
			MP_WritePhyUshort(sc, 0x15, 0x0056);
			MP_WritePhyUshort(sc, 0x19, 0x3000);
			MP_WritePhyUshort(sc, 0x15, 0x006E);
			MP_WritePhyUshort(sc, 0x19, 0x9afa);
			MP_WritePhyUshort(sc, 0x15, 0x00a1);
			MP_WritePhyUshort(sc, 0x19, 0x3044);
			MP_WritePhyUshort(sc, 0x15, 0x00ab);
			MP_WritePhyUshort(sc, 0x19, 0x5820);
			MP_WritePhyUshort(sc, 0x15, 0x00ac);
			MP_WritePhyUshort(sc, 0x19, 0x5e04);
			MP_WritePhyUshort(sc, 0x15, 0x00ad);
			MP_WritePhyUshort(sc, 0x19, 0xb60c);
			MP_WritePhyUshort(sc, 0x15, 0x00af);
			MP_WritePhyUshort(sc, 0x19, 0x000a);
			MP_WritePhyUshort(sc, 0x15, 0x00b2);
			MP_WritePhyUshort(sc, 0x19, 0x30b9);
			MP_WritePhyUshort(sc, 0x15, 0x00b9);
			MP_WritePhyUshort(sc, 0x19, 0x4408);
			MP_WritePhyUshort(sc, 0x15, 0x00ba);
			MP_WritePhyUshort(sc, 0x19, 0x480b);
			MP_WritePhyUshort(sc, 0x15, 0x00bb);
			MP_WritePhyUshort(sc, 0x19, 0x5e00);
			MP_WritePhyUshort(sc, 0x15, 0x00bc);
			MP_WritePhyUshort(sc, 0x19, 0x405f);
			MP_WritePhyUshort(sc, 0x15, 0x00bd);
			MP_WritePhyUshort(sc, 0x19, 0x4448);
			MP_WritePhyUshort(sc, 0x15, 0x00be);
			MP_WritePhyUshort(sc, 0x19, 0x4020);
			MP_WritePhyUshort(sc, 0x15, 0x00bf);
			MP_WritePhyUshort(sc, 0x19, 0x4468);
			MP_WritePhyUshort(sc, 0x15, 0x00c0);
			MP_WritePhyUshort(sc, 0x19, 0x9c02);
			MP_WritePhyUshort(sc, 0x15, 0x00c1);
			MP_WritePhyUshort(sc, 0x19, 0x58a0);
			MP_WritePhyUshort(sc, 0x15, 0x00c2);
			MP_WritePhyUshort(sc, 0x19, 0xb605);
			MP_WritePhyUshort(sc, 0x15, 0x00c3);
			MP_WritePhyUshort(sc, 0x19, 0xc0d3);
			MP_WritePhyUshort(sc, 0x15, 0x00c4);
			MP_WritePhyUshort(sc, 0x19, 0x00e6);
			MP_WritePhyUshort(sc, 0x15, 0x00c5);
			MP_WritePhyUshort(sc, 0x19, 0xdaec);
			MP_WritePhyUshort(sc, 0x15, 0x00c6);
			MP_WritePhyUshort(sc, 0x19, 0x00fa);
			MP_WritePhyUshort(sc, 0x15, 0x00c7);
			MP_WritePhyUshort(sc, 0x19, 0x9df9);
			MP_WritePhyUshort(sc, 0x15, 0x00c8);
			MP_WritePhyUshort(sc, 0x19, 0x307a);
			MP_WritePhyUshort(sc, 0x15, 0x0112);
			MP_WritePhyUshort(sc, 0x19, 0x6421);
			MP_WritePhyUshort(sc, 0x15, 0x0113);
			MP_WritePhyUshort(sc, 0x19, 0x7c08);
			MP_WritePhyUshort(sc, 0x15, 0x0114);
			MP_WritePhyUshort(sc, 0x19, 0x63f0);
			MP_WritePhyUshort(sc, 0x15, 0x0115);
			MP_WritePhyUshort(sc, 0x19, 0x4003);
			MP_WritePhyUshort(sc, 0x15, 0x0116);
			MP_WritePhyUshort(sc, 0x19, 0x4418);
			MP_WritePhyUshort(sc, 0x15, 0x0117);
			MP_WritePhyUshort(sc, 0x19, 0x9b00);
			MP_WritePhyUshort(sc, 0x15, 0x0118);
			MP_WritePhyUshort(sc, 0x19, 0x6461);
			MP_WritePhyUshort(sc, 0x15, 0x0119);
			MP_WritePhyUshort(sc, 0x19, 0x64e1);
			MP_WritePhyUshort(sc, 0x15, 0x011a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0150);
			MP_WritePhyUshort(sc, 0x19, 0x7c80);
			MP_WritePhyUshort(sc, 0x15, 0x0151);
			MP_WritePhyUshort(sc, 0x19, 0x6461);
			MP_WritePhyUshort(sc, 0x15, 0x0152);
			MP_WritePhyUshort(sc, 0x19, 0x4003);
			MP_WritePhyUshort(sc, 0x15, 0x0153);
			MP_WritePhyUshort(sc, 0x19, 0x4540);
			MP_WritePhyUshort(sc, 0x15, 0x0154);
			MP_WritePhyUshort(sc, 0x19, 0x9f00);
			MP_WritePhyUshort(sc, 0x15, 0x0155);
			MP_WritePhyUshort(sc, 0x19, 0x9d00);
			MP_WritePhyUshort(sc, 0x15, 0x0156);
			MP_WritePhyUshort(sc, 0x19, 0x7c40);
			MP_WritePhyUshort(sc, 0x15, 0x0157);
			MP_WritePhyUshort(sc, 0x19, 0x6421);
			MP_WritePhyUshort(sc, 0x15, 0x0158);
			MP_WritePhyUshort(sc, 0x19, 0x7c80);
			MP_WritePhyUshort(sc, 0x15, 0x0159);
			MP_WritePhyUshort(sc, 0x19, 0x64a1);
			MP_WritePhyUshort(sc, 0x15, 0x015a);
			MP_WritePhyUshort(sc, 0x19, 0x30fe);
			MP_WritePhyUshort(sc, 0x15, 0x021e);
			MP_WritePhyUshort(sc, 0x19, 0x5410);
			MP_WritePhyUshort(sc, 0x15, 0x0225);
			MP_WritePhyUshort(sc, 0x19, 0x5400);
			MP_WritePhyUshort(sc, 0x15, 0x023D);
			MP_WritePhyUshort(sc, 0x19, 0x4050);
			MP_WritePhyUshort(sc, 0x15, 0x0295);
			MP_WritePhyUshort(sc, 0x19, 0x6c08);
			MP_WritePhyUshort(sc, 0x15, 0x02bd);
			MP_WritePhyUshort(sc, 0x19, 0xa523);
			MP_WritePhyUshort(sc, 0x15, 0x02be);
			MP_WritePhyUshort(sc, 0x19, 0x32ca);
			MP_WritePhyUshort(sc, 0x15, 0x02ca);
			MP_WritePhyUshort(sc, 0x19, 0x48b3);
			MP_WritePhyUshort(sc, 0x15, 0x02cb);
			MP_WritePhyUshort(sc, 0x19, 0x4020);
			MP_WritePhyUshort(sc, 0x15, 0x02cc);
			MP_WritePhyUshort(sc, 0x19, 0x4823);
			MP_WritePhyUshort(sc, 0x15, 0x02cd);
			MP_WritePhyUshort(sc, 0x19, 0x4510);
			MP_WritePhyUshort(sc, 0x15, 0x02ce);
			MP_WritePhyUshort(sc, 0x19, 0xb63a);
			MP_WritePhyUshort(sc, 0x15, 0x02cf);
			MP_WritePhyUshort(sc, 0x19, 0x7dc8);
			MP_WritePhyUshort(sc, 0x15, 0x02d6);
			MP_WritePhyUshort(sc, 0x19, 0x9bf8);
			MP_WritePhyUshort(sc, 0x15, 0x02d8);
			MP_WritePhyUshort(sc, 0x19, 0x85f6);
			MP_WritePhyUshort(sc, 0x15, 0x02d9);
			MP_WritePhyUshort(sc, 0x19, 0x32e0);
			MP_WritePhyUshort(sc, 0x15, 0x02e0);
			MP_WritePhyUshort(sc, 0x19, 0x4834);
			MP_WritePhyUshort(sc, 0x15, 0x02e1);
			MP_WritePhyUshort(sc, 0x19, 0x6c08);
			MP_WritePhyUshort(sc, 0x15, 0x02e2);
			MP_WritePhyUshort(sc, 0x19, 0x4020);
			MP_WritePhyUshort(sc, 0x15, 0x02e3);
			MP_WritePhyUshort(sc, 0x19, 0x4824);
			MP_WritePhyUshort(sc, 0x15, 0x02e4);
			MP_WritePhyUshort(sc, 0x19, 0x4520);
			MP_WritePhyUshort(sc, 0x15, 0x02e5);
			MP_WritePhyUshort(sc, 0x19, 0x4008);
			MP_WritePhyUshort(sc, 0x15, 0x02e6);
			MP_WritePhyUshort(sc, 0x19, 0x4560);
			MP_WritePhyUshort(sc, 0x15, 0x02e7);
			MP_WritePhyUshort(sc, 0x19, 0x9d04);
			MP_WritePhyUshort(sc, 0x15, 0x02e8);
			MP_WritePhyUshort(sc, 0x19, 0x48c4);
			MP_WritePhyUshort(sc, 0x15, 0x02e9);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x02ea);
			MP_WritePhyUshort(sc, 0x19, 0x4844);
			MP_WritePhyUshort(sc, 0x15, 0x02eb);
			MP_WritePhyUshort(sc, 0x19, 0x7dc8);
			MP_WritePhyUshort(sc, 0x15, 0x02f0);
			MP_WritePhyUshort(sc, 0x19, 0x9cf7);
			MP_WritePhyUshort(sc, 0x15, 0x02f1);
			MP_WritePhyUshort(sc, 0x19, 0xdf94);
			MP_WritePhyUshort(sc, 0x15, 0x02f2);
			MP_WritePhyUshort(sc, 0x19, 0x0002);
			MP_WritePhyUshort(sc, 0x15, 0x02f3);
			MP_WritePhyUshort(sc, 0x19, 0x6810);
			MP_WritePhyUshort(sc, 0x15, 0x02f4);
			MP_WritePhyUshort(sc, 0x19, 0xb614);
			MP_WritePhyUshort(sc, 0x15, 0x02f5);
			MP_WritePhyUshort(sc, 0x19, 0xc42b);
			MP_WritePhyUshort(sc, 0x15, 0x02f6);
			MP_WritePhyUshort(sc, 0x19, 0x00d4);
			MP_WritePhyUshort(sc, 0x15, 0x02f7);
			MP_WritePhyUshort(sc, 0x19, 0xc455);
			MP_WritePhyUshort(sc, 0x15, 0x02f8);
			MP_WritePhyUshort(sc, 0x19, 0x0093);
			MP_WritePhyUshort(sc, 0x15, 0x02f9);
			MP_WritePhyUshort(sc, 0x19, 0x92ee);
			MP_WritePhyUshort(sc, 0x15, 0x02fa);
			MP_WritePhyUshort(sc, 0x19, 0xefed);
			MP_WritePhyUshort(sc, 0x15, 0x02fb);
			MP_WritePhyUshort(sc, 0x19, 0x3312);
			MP_WritePhyUshort(sc, 0x15, 0x0312);
			MP_WritePhyUshort(sc, 0x19, 0x49b5);
			MP_WritePhyUshort(sc, 0x15, 0x0313);
			MP_WritePhyUshort(sc, 0x19, 0x7d00);
			MP_WritePhyUshort(sc, 0x15, 0x0314);
			MP_WritePhyUshort(sc, 0x19, 0x4d00);
			MP_WritePhyUshort(sc, 0x15, 0x0315);
			MP_WritePhyUshort(sc, 0x19, 0x6810);
			MP_WritePhyUshort(sc, 0x15, 0x031e);
			MP_WritePhyUshort(sc, 0x19, 0x404f);
			MP_WritePhyUshort(sc, 0x15, 0x031f);
			MP_WritePhyUshort(sc, 0x19, 0x44c8);
			MP_WritePhyUshort(sc, 0x15, 0x0320);
			MP_WritePhyUshort(sc, 0x19, 0xd64f);
			MP_WritePhyUshort(sc, 0x15, 0x0321);
			MP_WritePhyUshort(sc, 0x19, 0x00e7);
			MP_WritePhyUshort(sc, 0x15, 0x0322);
			MP_WritePhyUshort(sc, 0x19, 0x7c08);
			MP_WritePhyUshort(sc, 0x15, 0x0323);
			MP_WritePhyUshort(sc, 0x19, 0x8203);
			MP_WritePhyUshort(sc, 0x15, 0x0324);
			MP_WritePhyUshort(sc, 0x19, 0x4d48);
			MP_WritePhyUshort(sc, 0x15, 0x0325);
			MP_WritePhyUshort(sc, 0x19, 0x3327);
			MP_WritePhyUshort(sc, 0x15, 0x0326);
			MP_WritePhyUshort(sc, 0x19, 0x4d40);
			MP_WritePhyUshort(sc, 0x15, 0x0327);
			MP_WritePhyUshort(sc, 0x19, 0xc8d7);
			MP_WritePhyUshort(sc, 0x15, 0x0328);
			MP_WritePhyUshort(sc, 0x19, 0x0003);
			MP_WritePhyUshort(sc, 0x15, 0x0329);
			MP_WritePhyUshort(sc, 0x19, 0x7c20);
			MP_WritePhyUshort(sc, 0x15, 0x032a);
			MP_WritePhyUshort(sc, 0x19, 0x4c20);
			MP_WritePhyUshort(sc, 0x15, 0x032b);
			MP_WritePhyUshort(sc, 0x19, 0xc8ed);
			MP_WritePhyUshort(sc, 0x15, 0x032c);
			MP_WritePhyUshort(sc, 0x19, 0x00f4);
			MP_WritePhyUshort(sc, 0x15, 0x032d);
			MP_WritePhyUshort(sc, 0x19, 0x82b3);
			MP_WritePhyUshort(sc, 0x15, 0x032e);
			MP_WritePhyUshort(sc, 0x19, 0xd11d);
			MP_WritePhyUshort(sc, 0x15, 0x032f);
			MP_WritePhyUshort(sc, 0x19, 0x00b1);
			MP_WritePhyUshort(sc, 0x15, 0x0330);
			MP_WritePhyUshort(sc, 0x19, 0xde18);
			MP_WritePhyUshort(sc, 0x15, 0x0331);
			MP_WritePhyUshort(sc, 0x19, 0x0008);
			MP_WritePhyUshort(sc, 0x15, 0x0332);
			MP_WritePhyUshort(sc, 0x19, 0x91ee);
			MP_WritePhyUshort(sc, 0x15, 0x0333);
			MP_WritePhyUshort(sc, 0x19, 0x3339);
			MP_WritePhyUshort(sc, 0x15, 0x033a);
			MP_WritePhyUshort(sc, 0x19, 0x4064);
			MP_WritePhyUshort(sc, 0x15, 0x0340);
			MP_WritePhyUshort(sc, 0x19, 0x9e06);
			MP_WritePhyUshort(sc, 0x15, 0x0341);
			MP_WritePhyUshort(sc, 0x19, 0x7c08);
			MP_WritePhyUshort(sc, 0x15, 0x0342);
			MP_WritePhyUshort(sc, 0x19, 0x8203);
			MP_WritePhyUshort(sc, 0x15, 0x0343);
			MP_WritePhyUshort(sc, 0x19, 0x4d48);
			MP_WritePhyUshort(sc, 0x15, 0x0344);
			MP_WritePhyUshort(sc, 0x19, 0x3346);
			MP_WritePhyUshort(sc, 0x15, 0x0345);
			MP_WritePhyUshort(sc, 0x19, 0x4d40);
			MP_WritePhyUshort(sc, 0x15, 0x0346);
			MP_WritePhyUshort(sc, 0x19, 0xd11d);
			MP_WritePhyUshort(sc, 0x15, 0x0347);
			MP_WritePhyUshort(sc, 0x19, 0x0099);
			MP_WritePhyUshort(sc, 0x15, 0x0348);
			MP_WritePhyUshort(sc, 0x19, 0xbb17);
			MP_WritePhyUshort(sc, 0x15, 0x0349);
			MP_WritePhyUshort(sc, 0x19, 0x8102);
			MP_WritePhyUshort(sc, 0x15, 0x034a);
			MP_WritePhyUshort(sc, 0x19, 0x334d);
			MP_WritePhyUshort(sc, 0x15, 0x034b);
			MP_WritePhyUshort(sc, 0x19, 0xa22c);
			MP_WritePhyUshort(sc, 0x15, 0x034c);
			MP_WritePhyUshort(sc, 0x19, 0x3397);
			MP_WritePhyUshort(sc, 0x15, 0x034d);
			MP_WritePhyUshort(sc, 0x19, 0x91f2);
			MP_WritePhyUshort(sc, 0x15, 0x034e);
			MP_WritePhyUshort(sc, 0x19, 0xc218);
			MP_WritePhyUshort(sc, 0x15, 0x034f);
			MP_WritePhyUshort(sc, 0x19, 0x00f0);
			MP_WritePhyUshort(sc, 0x15, 0x0350);
			MP_WritePhyUshort(sc, 0x19, 0x3397);
			MP_WritePhyUshort(sc, 0x15, 0x0351);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0364);
			MP_WritePhyUshort(sc, 0x19, 0xbc05);
			MP_WritePhyUshort(sc, 0x15, 0x0367);
			MP_WritePhyUshort(sc, 0x19, 0xa1fc);
			MP_WritePhyUshort(sc, 0x15, 0x0368);
			MP_WritePhyUshort(sc, 0x19, 0x3377);
			MP_WritePhyUshort(sc, 0x15, 0x0369);
			MP_WritePhyUshort(sc, 0x19, 0x328b);
			MP_WritePhyUshort(sc, 0x15, 0x036a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0377);
			MP_WritePhyUshort(sc, 0x19, 0x4b97);
			MP_WritePhyUshort(sc, 0x15, 0x0378);
			MP_WritePhyUshort(sc, 0x19, 0x6818);
			MP_WritePhyUshort(sc, 0x15, 0x0379);
			MP_WritePhyUshort(sc, 0x19, 0x4b07);
			MP_WritePhyUshort(sc, 0x15, 0x037a);
			MP_WritePhyUshort(sc, 0x19, 0x40ac);
			MP_WritePhyUshort(sc, 0x15, 0x037b);
			MP_WritePhyUshort(sc, 0x19, 0x4445);
			MP_WritePhyUshort(sc, 0x15, 0x037c);
			MP_WritePhyUshort(sc, 0x19, 0x404e);
			MP_WritePhyUshort(sc, 0x15, 0x037d);
			MP_WritePhyUshort(sc, 0x19, 0x4461);
			MP_WritePhyUshort(sc, 0x15, 0x037e);
			MP_WritePhyUshort(sc, 0x19, 0x9c09);
			MP_WritePhyUshort(sc, 0x15, 0x037f);
			MP_WritePhyUshort(sc, 0x19, 0x63da);
			MP_WritePhyUshort(sc, 0x15, 0x0380);
			MP_WritePhyUshort(sc, 0x19, 0x5440);
			MP_WritePhyUshort(sc, 0x15, 0x0381);
			MP_WritePhyUshort(sc, 0x19, 0x4b98);
			MP_WritePhyUshort(sc, 0x15, 0x0382);
			MP_WritePhyUshort(sc, 0x19, 0x7c60);
			MP_WritePhyUshort(sc, 0x15, 0x0383);
			MP_WritePhyUshort(sc, 0x19, 0x4c00);
			MP_WritePhyUshort(sc, 0x15, 0x0384);
			MP_WritePhyUshort(sc, 0x19, 0x4b08);
			MP_WritePhyUshort(sc, 0x15, 0x0385);
			MP_WritePhyUshort(sc, 0x19, 0x63d8);
			MP_WritePhyUshort(sc, 0x15, 0x0386);
			MP_WritePhyUshort(sc, 0x19, 0x338d);
			MP_WritePhyUshort(sc, 0x15, 0x0387);
			MP_WritePhyUshort(sc, 0x19, 0xd64f);
			MP_WritePhyUshort(sc, 0x15, 0x0388);
			MP_WritePhyUshort(sc, 0x19, 0x0080);
			MP_WritePhyUshort(sc, 0x15, 0x0389);
			MP_WritePhyUshort(sc, 0x19, 0x820c);
			MP_WritePhyUshort(sc, 0x15, 0x038a);
			MP_WritePhyUshort(sc, 0x19, 0xa10b);
			MP_WritePhyUshort(sc, 0x15, 0x038b);
			MP_WritePhyUshort(sc, 0x19, 0x9df3);
			MP_WritePhyUshort(sc, 0x15, 0x038c);
			MP_WritePhyUshort(sc, 0x19, 0x3395);
			MP_WritePhyUshort(sc, 0x15, 0x038d);
			MP_WritePhyUshort(sc, 0x19, 0xd64f);
			MP_WritePhyUshort(sc, 0x15, 0x038e);
			MP_WritePhyUshort(sc, 0x19, 0x00f9);
			MP_WritePhyUshort(sc, 0x15, 0x038f);
			MP_WritePhyUshort(sc, 0x19, 0xc017);
			MP_WritePhyUshort(sc, 0x15, 0x0390);
			MP_WritePhyUshort(sc, 0x19, 0x0005);
			MP_WritePhyUshort(sc, 0x15, 0x0391);
			MP_WritePhyUshort(sc, 0x19, 0x6c0b);
			MP_WritePhyUshort(sc, 0x15, 0x0392);
			MP_WritePhyUshort(sc, 0x19, 0xa103);
			MP_WritePhyUshort(sc, 0x15, 0x0393);
			MP_WritePhyUshort(sc, 0x19, 0x6c08);
			MP_WritePhyUshort(sc, 0x15, 0x0394);
			MP_WritePhyUshort(sc, 0x19, 0x9df9);
			MP_WritePhyUshort(sc, 0x15, 0x0395);
			MP_WritePhyUshort(sc, 0x19, 0x6c08);
			MP_WritePhyUshort(sc, 0x15, 0x0396);
			MP_WritePhyUshort(sc, 0x19, 0x3397);
			MP_WritePhyUshort(sc, 0x15, 0x0399);
			MP_WritePhyUshort(sc, 0x19, 0x6810);
			MP_WritePhyUshort(sc, 0x15, 0x03a4);
			MP_WritePhyUshort(sc, 0x19, 0x7c08);
			MP_WritePhyUshort(sc, 0x15, 0x03a5);
			MP_WritePhyUshort(sc, 0x19, 0x8203);
			MP_WritePhyUshort(sc, 0x15, 0x03a6);
			MP_WritePhyUshort(sc, 0x19, 0x4d08);
			MP_WritePhyUshort(sc, 0x15, 0x03a7);
			MP_WritePhyUshort(sc, 0x19, 0x33a9);
			MP_WritePhyUshort(sc, 0x15, 0x03a8);
			MP_WritePhyUshort(sc, 0x19, 0x4d00);
			MP_WritePhyUshort(sc, 0x15, 0x03a9);
			MP_WritePhyUshort(sc, 0x19, 0x9bfa);
			MP_WritePhyUshort(sc, 0x15, 0x03aa);
			MP_WritePhyUshort(sc, 0x19, 0x33b6);
			MP_WritePhyUshort(sc, 0x15, 0x03bb);
			MP_WritePhyUshort(sc, 0x19, 0x4056);
			MP_WritePhyUshort(sc, 0x15, 0x03bc);
			MP_WritePhyUshort(sc, 0x19, 0x44e9);
			MP_WritePhyUshort(sc, 0x15, 0x03bd);
			MP_WritePhyUshort(sc, 0x19, 0x405e);
			MP_WritePhyUshort(sc, 0x15, 0x03be);
			MP_WritePhyUshort(sc, 0x19, 0x44f8);
			MP_WritePhyUshort(sc, 0x15, 0x03bf);
			MP_WritePhyUshort(sc, 0x19, 0xd64f);
			MP_WritePhyUshort(sc, 0x15, 0x03c0);
			MP_WritePhyUshort(sc, 0x19, 0x0037);
			MP_WritePhyUshort(sc, 0x15, 0x03c1);
			MP_WritePhyUshort(sc, 0x19, 0xbd37);
			MP_WritePhyUshort(sc, 0x15, 0x03c2);
			MP_WritePhyUshort(sc, 0x19, 0x9cfd);
			MP_WritePhyUshort(sc, 0x15, 0x03c3);
			MP_WritePhyUshort(sc, 0x19, 0xc639);
			MP_WritePhyUshort(sc, 0x15, 0x03c4);
			MP_WritePhyUshort(sc, 0x19, 0x0011);
			MP_WritePhyUshort(sc, 0x15, 0x03c5);
			MP_WritePhyUshort(sc, 0x19, 0x9b03);
			MP_WritePhyUshort(sc, 0x15, 0x03c6);
			MP_WritePhyUshort(sc, 0x19, 0x7c01);
			MP_WritePhyUshort(sc, 0x15, 0x03c7);
			MP_WritePhyUshort(sc, 0x19, 0x4c01);
			MP_WritePhyUshort(sc, 0x15, 0x03c8);
			MP_WritePhyUshort(sc, 0x19, 0x9e03);
			MP_WritePhyUshort(sc, 0x15, 0x03c9);
			MP_WritePhyUshort(sc, 0x19, 0x7c20);
			MP_WritePhyUshort(sc, 0x15, 0x03ca);
			MP_WritePhyUshort(sc, 0x19, 0x4c20);
			MP_WritePhyUshort(sc, 0x15, 0x03cb);
			MP_WritePhyUshort(sc, 0x19, 0x9af4);
			MP_WritePhyUshort(sc, 0x15, 0x03cc);
			MP_WritePhyUshort(sc, 0x19, 0x7c12);
			MP_WritePhyUshort(sc, 0x15, 0x03cd);
			MP_WritePhyUshort(sc, 0x19, 0x4c52);
			MP_WritePhyUshort(sc, 0x15, 0x03ce);
			MP_WritePhyUshort(sc, 0x19, 0x4470);
			MP_WritePhyUshort(sc, 0x15, 0x03cf);
			MP_WritePhyUshort(sc, 0x19, 0x7c12);
			MP_WritePhyUshort(sc, 0x15, 0x03d0);
			MP_WritePhyUshort(sc, 0x19, 0x4c40);
			MP_WritePhyUshort(sc, 0x15, 0x03d1);
			MP_WritePhyUshort(sc, 0x19, 0x33bf);
			MP_WritePhyUshort(sc, 0x15, 0x03d6);
			MP_WritePhyUshort(sc, 0x19, 0x4047);
			MP_WritePhyUshort(sc, 0x15, 0x03d7);
			MP_WritePhyUshort(sc, 0x19, 0x4469);
			MP_WritePhyUshort(sc, 0x15, 0x03d8);
			MP_WritePhyUshort(sc, 0x19, 0x492b);
			MP_WritePhyUshort(sc, 0x15, 0x03d9);
			MP_WritePhyUshort(sc, 0x19, 0x4479);
			MP_WritePhyUshort(sc, 0x15, 0x03da);
			MP_WritePhyUshort(sc, 0x19, 0x7c09);
			MP_WritePhyUshort(sc, 0x15, 0x03db);
			MP_WritePhyUshort(sc, 0x19, 0x8203);
			MP_WritePhyUshort(sc, 0x15, 0x03dc);
			MP_WritePhyUshort(sc, 0x19, 0x4d48);
			MP_WritePhyUshort(sc, 0x15, 0x03dd);
			MP_WritePhyUshort(sc, 0x19, 0x33df);
			MP_WritePhyUshort(sc, 0x15, 0x03de);
			MP_WritePhyUshort(sc, 0x19, 0x4d40);
			MP_WritePhyUshort(sc, 0x15, 0x03df);
			MP_WritePhyUshort(sc, 0x19, 0xd64f);
			MP_WritePhyUshort(sc, 0x15, 0x03e0);
			MP_WritePhyUshort(sc, 0x19, 0x0017);
			MP_WritePhyUshort(sc, 0x15, 0x03e1);
			MP_WritePhyUshort(sc, 0x19, 0xbd17);
			MP_WritePhyUshort(sc, 0x15, 0x03e2);
			MP_WritePhyUshort(sc, 0x19, 0x9b03);
			MP_WritePhyUshort(sc, 0x15, 0x03e3);
			MP_WritePhyUshort(sc, 0x19, 0x7c20);
			MP_WritePhyUshort(sc, 0x15, 0x03e4);
			MP_WritePhyUshort(sc, 0x19, 0x4c20);
			MP_WritePhyUshort(sc, 0x15, 0x03e5);
			MP_WritePhyUshort(sc, 0x19, 0x88f5);
			MP_WritePhyUshort(sc, 0x15, 0x03e6);
			MP_WritePhyUshort(sc, 0x19, 0xc428);
			MP_WritePhyUshort(sc, 0x15, 0x03e7);
			MP_WritePhyUshort(sc, 0x19, 0x0008);
			MP_WritePhyUshort(sc, 0x15, 0x03e8);
			MP_WritePhyUshort(sc, 0x19, 0x9af2);
			MP_WritePhyUshort(sc, 0x15, 0x03e9);
			MP_WritePhyUshort(sc, 0x19, 0x7c12);
			MP_WritePhyUshort(sc, 0x15, 0x03ea);
			MP_WritePhyUshort(sc, 0x19, 0x4c52);
			MP_WritePhyUshort(sc, 0x15, 0x03eb);
			MP_WritePhyUshort(sc, 0x19, 0x4470);
			MP_WritePhyUshort(sc, 0x15, 0x03ec);
			MP_WritePhyUshort(sc, 0x19, 0x7c12);
			MP_WritePhyUshort(sc, 0x15, 0x03ed);
			MP_WritePhyUshort(sc, 0x19, 0x4c40);
			MP_WritePhyUshort(sc, 0x15, 0x03ee);
			MP_WritePhyUshort(sc, 0x19, 0x33da);
			MP_WritePhyUshort(sc, 0x15, 0x03ef);
			MP_WritePhyUshort(sc, 0x19, 0x3312);
			MP_WritePhyUshort(sc, 0x16, 0x0306);
			MP_WritePhyUshort(sc, 0x16, 0x0300);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x17, 0x2179);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0040);
			MP_WritePhyUshort(sc, 0x18, 0x0645);
			MP_WritePhyUshort(sc, 0x19, 0xe200);
			MP_WritePhyUshort(sc, 0x18, 0x0655);
			MP_WritePhyUshort(sc, 0x19, 0x9000);
			MP_WritePhyUshort(sc, 0x18, 0x0d05);
			MP_WritePhyUshort(sc, 0x19, 0xbe00);
			MP_WritePhyUshort(sc, 0x18, 0x0d15);
			MP_WritePhyUshort(sc, 0x19, 0xd300);
			MP_WritePhyUshort(sc, 0x18, 0x0d25);
			MP_WritePhyUshort(sc, 0x19, 0xfe00);
			MP_WritePhyUshort(sc, 0x18, 0x0d35);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x0d45);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x0d55);
			MP_WritePhyUshort(sc, 0x19, 0x1000);
			MP_WritePhyUshort(sc, 0x18, 0x0d65);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x0d75);
			MP_WritePhyUshort(sc, 0x19, 0x8200);
			MP_WritePhyUshort(sc, 0x18, 0x0d85);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x0d95);
			MP_WritePhyUshort(sc, 0x19, 0x7000);
			MP_WritePhyUshort(sc, 0x18, 0x0da5);
			MP_WritePhyUshort(sc, 0x19, 0x0f00);
			MP_WritePhyUshort(sc, 0x18, 0x0db5);
			MP_WritePhyUshort(sc, 0x19, 0x0100);
			MP_WritePhyUshort(sc, 0x18, 0x0dc5);
			MP_WritePhyUshort(sc, 0x19, 0x9b00);
			MP_WritePhyUshort(sc, 0x18, 0x0dd5);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x0de5);
			MP_WritePhyUshort(sc, 0x19, 0xe000);
			MP_WritePhyUshort(sc, 0x18, 0x0df5);
			MP_WritePhyUshort(sc, 0x19, 0xef00);
			MP_WritePhyUshort(sc, 0x18, 0x16d5);
			MP_WritePhyUshort(sc, 0x19, 0xe200);
			MP_WritePhyUshort(sc, 0x18, 0x16e5);
			MP_WritePhyUshort(sc, 0x19, 0xab00);
			MP_WritePhyUshort(sc, 0x18, 0x2904);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x2914);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x2924);
			MP_WritePhyUshort(sc, 0x19, 0x0100);
			MP_WritePhyUshort(sc, 0x18, 0x2934);
			MP_WritePhyUshort(sc, 0x19, 0x2000);
			MP_WritePhyUshort(sc, 0x18, 0x2944);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2954);
			MP_WritePhyUshort(sc, 0x19, 0x4600);
			MP_WritePhyUshort(sc, 0x18, 0x2964);
			MP_WritePhyUshort(sc, 0x19, 0xfc00);
			MP_WritePhyUshort(sc, 0x18, 0x2974);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2984);
			MP_WritePhyUshort(sc, 0x19, 0x5000);
			MP_WritePhyUshort(sc, 0x18, 0x2994);
			MP_WritePhyUshort(sc, 0x19, 0x9d00);
			MP_WritePhyUshort(sc, 0x18, 0x29a4);
			MP_WritePhyUshort(sc, 0x19, 0xff00);
			MP_WritePhyUshort(sc, 0x18, 0x29b4);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x29c4);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x29d4);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x29e4);
			MP_WritePhyUshort(sc, 0x19, 0x2000);
			MP_WritePhyUshort(sc, 0x18, 0x29f4);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2a04);
			MP_WritePhyUshort(sc, 0x19, 0xe600);
			MP_WritePhyUshort(sc, 0x18, 0x2a14);
			MP_WritePhyUshort(sc, 0x19, 0xff00);
			MP_WritePhyUshort(sc, 0x18, 0x2a24);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2a34);
			MP_WritePhyUshort(sc, 0x19, 0x5000);
			MP_WritePhyUshort(sc, 0x18, 0x2a44);
			MP_WritePhyUshort(sc, 0x19, 0x8500);
			MP_WritePhyUshort(sc, 0x18, 0x2a54);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x2a64);
			MP_WritePhyUshort(sc, 0x19, 0xac00);
			MP_WritePhyUshort(sc, 0x18, 0x2a74);
			MP_WritePhyUshort(sc, 0x19, 0x0800);
			MP_WritePhyUshort(sc, 0x18, 0x2a84);
			MP_WritePhyUshort(sc, 0x19, 0xfc00);
			MP_WritePhyUshort(sc, 0x18, 0x2a94);
			MP_WritePhyUshort(sc, 0x19, 0xe000);
			MP_WritePhyUshort(sc, 0x18, 0x2aa4);
			MP_WritePhyUshort(sc, 0x19, 0x7400);
			MP_WritePhyUshort(sc, 0x18, 0x2ab4);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x2ac4);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x2ad4);
			MP_WritePhyUshort(sc, 0x19, 0x0100);
			MP_WritePhyUshort(sc, 0x18, 0x2ae4);
			MP_WritePhyUshort(sc, 0x19, 0xff00);
			MP_WritePhyUshort(sc, 0x18, 0x2af4);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2b04);
			MP_WritePhyUshort(sc, 0x19, 0x4400);
			MP_WritePhyUshort(sc, 0x18, 0x2b14);
			MP_WritePhyUshort(sc, 0x19, 0xfc00);
			MP_WritePhyUshort(sc, 0x18, 0x2b24);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2b34);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x2b44);
			MP_WritePhyUshort(sc, 0x19, 0x9d00);
			MP_WritePhyUshort(sc, 0x18, 0x2b54);
			MP_WritePhyUshort(sc, 0x19, 0xff00);
			MP_WritePhyUshort(sc, 0x18, 0x2b64);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x2b74);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x2b84);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2b94);
			MP_WritePhyUshort(sc, 0x19, 0xff00);
			MP_WritePhyUshort(sc, 0x18, 0x2ba4);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2bb4);
			MP_WritePhyUshort(sc, 0x19, 0xfc00);
			MP_WritePhyUshort(sc, 0x18, 0x2bc4);
			MP_WritePhyUshort(sc, 0x19, 0xff00);
			MP_WritePhyUshort(sc, 0x18, 0x2bd4);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2be4);
			MP_WritePhyUshort(sc, 0x19, 0x4000);
			MP_WritePhyUshort(sc, 0x18, 0x2bf4);
			MP_WritePhyUshort(sc, 0x19, 0x8900);
			MP_WritePhyUshort(sc, 0x18, 0x2c04);
			MP_WritePhyUshort(sc, 0x19, 0x8300);
			MP_WritePhyUshort(sc, 0x18, 0x2c14);
			MP_WritePhyUshort(sc, 0x19, 0xe000);
			MP_WritePhyUshort(sc, 0x18, 0x2c24);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x18, 0x2c34);
			MP_WritePhyUshort(sc, 0x19, 0xac00);
			MP_WritePhyUshort(sc, 0x18, 0x2c44);
			MP_WritePhyUshort(sc, 0x19, 0x0800);
			MP_WritePhyUshort(sc, 0x18, 0x2c54);
			MP_WritePhyUshort(sc, 0x19, 0xfa00);
			MP_WritePhyUshort(sc, 0x18, 0x2c64);
			MP_WritePhyUshort(sc, 0x19, 0xe100);
			MP_WritePhyUshort(sc, 0x18, 0x2c74);
			MP_WritePhyUshort(sc, 0x19, 0x7f00);
			MP_WritePhyUshort(sc, 0x18, 0x0001);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x17, 0x2100);
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x0080);
			MP_WritePhyUshort(sc, 0x05, 0x8b88);
			MP_WritePhyUshort(sc, 0x06, 0x0000);
			MP_WritePhyUshort(sc, 0x06, 0x0000);
			MP_WritePhyUshort(sc, 0x06, 0x0000);
			MP_WritePhyUshort(sc, 0x06, 0x0000);
			MP_WritePhyUshort(sc, 0x05, 0x8000);
			MP_WritePhyUshort(sc, 0x06, 0xd480);
			MP_WritePhyUshort(sc, 0x06, 0xc1e4);
			MP_WritePhyUshort(sc, 0x06, 0x8b9a);
			MP_WritePhyUshort(sc, 0x06, 0xe58b);
			MP_WritePhyUshort(sc, 0x06, 0x9bee);
			MP_WritePhyUshort(sc, 0x06, 0x8b83);
			MP_WritePhyUshort(sc, 0x06, 0x41bf);
			MP_WritePhyUshort(sc, 0x06, 0x8b88);
			MP_WritePhyUshort(sc, 0x06, 0xec00);
			MP_WritePhyUshort(sc, 0x06, 0x19a9);
			MP_WritePhyUshort(sc, 0x06, 0x8b90);
			MP_WritePhyUshort(sc, 0x06, 0xf9ee);
			MP_WritePhyUshort(sc, 0x06, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0xfff7);
			MP_WritePhyUshort(sc, 0x06, 0xffe0);
			MP_WritePhyUshort(sc, 0x06, 0xe140);
			MP_WritePhyUshort(sc, 0x06, 0xe1e1);
			MP_WritePhyUshort(sc, 0x06, 0x41f7);
			MP_WritePhyUshort(sc, 0x06, 0x2ff6);
			MP_WritePhyUshort(sc, 0x06, 0x28e4);
			MP_WritePhyUshort(sc, 0x06, 0xe140);
			MP_WritePhyUshort(sc, 0x06, 0xe5e1);
			MP_WritePhyUshort(sc, 0x06, 0x41f7);
			MP_WritePhyUshort(sc, 0x06, 0x0002);
			MP_WritePhyUshort(sc, 0x06, 0x020c);
			MP_WritePhyUshort(sc, 0x06, 0x0202);
			MP_WritePhyUshort(sc, 0x06, 0x1d02);
			MP_WritePhyUshort(sc, 0x06, 0x0230);
			MP_WritePhyUshort(sc, 0x06, 0x0202);
			MP_WritePhyUshort(sc, 0x06, 0x4002);
			MP_WritePhyUshort(sc, 0x06, 0x028b);
			MP_WritePhyUshort(sc, 0x06, 0x0280);
			MP_WritePhyUshort(sc, 0x06, 0x6c02);
			MP_WritePhyUshort(sc, 0x06, 0x8085);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x88e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b89);
			MP_WritePhyUshort(sc, 0x06, 0x1e01);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x8a1e);
			MP_WritePhyUshort(sc, 0x06, 0x01e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b8b);
			MP_WritePhyUshort(sc, 0x06, 0x1e01);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x8c1e);
			MP_WritePhyUshort(sc, 0x06, 0x01e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b8d);
			MP_WritePhyUshort(sc, 0x06, 0x1e01);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x8e1e);
			MP_WritePhyUshort(sc, 0x06, 0x01a0);
			MP_WritePhyUshort(sc, 0x06, 0x00c7);
			MP_WritePhyUshort(sc, 0x06, 0xaec3);
			MP_WritePhyUshort(sc, 0x06, 0xf8e0);
			MP_WritePhyUshort(sc, 0x06, 0x8b8d);
			MP_WritePhyUshort(sc, 0x06, 0xad20);
			MP_WritePhyUshort(sc, 0x06, 0x10ee);
			MP_WritePhyUshort(sc, 0x06, 0x8b8d);
			MP_WritePhyUshort(sc, 0x06, 0x0002);
			MP_WritePhyUshort(sc, 0x06, 0x1310);
			MP_WritePhyUshort(sc, 0x06, 0x0280);
			MP_WritePhyUshort(sc, 0x06, 0xc602);
			MP_WritePhyUshort(sc, 0x06, 0x1f0c);
			MP_WritePhyUshort(sc, 0x06, 0x0227);
			MP_WritePhyUshort(sc, 0x06, 0x49fc);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x8ead);
			MP_WritePhyUshort(sc, 0x06, 0x200b);
			MP_WritePhyUshort(sc, 0x06, 0xf620);
			MP_WritePhyUshort(sc, 0x06, 0xe48b);
			MP_WritePhyUshort(sc, 0x06, 0x8e02);
			MP_WritePhyUshort(sc, 0x06, 0x852d);
			MP_WritePhyUshort(sc, 0x06, 0x021b);
			MP_WritePhyUshort(sc, 0x06, 0x67ad);
			MP_WritePhyUshort(sc, 0x06, 0x2211);
			MP_WritePhyUshort(sc, 0x06, 0xf622);
			MP_WritePhyUshort(sc, 0x06, 0xe48b);
			MP_WritePhyUshort(sc, 0x06, 0x8e02);
			MP_WritePhyUshort(sc, 0x06, 0x2ba5);
			MP_WritePhyUshort(sc, 0x06, 0x022a);
			MP_WritePhyUshort(sc, 0x06, 0x2402);
			MP_WritePhyUshort(sc, 0x06, 0x82e5);
			MP_WritePhyUshort(sc, 0x06, 0x022a);
			MP_WritePhyUshort(sc, 0x06, 0xf0ad);
			MP_WritePhyUshort(sc, 0x06, 0x2511);
			MP_WritePhyUshort(sc, 0x06, 0xf625);
			MP_WritePhyUshort(sc, 0x06, 0xe48b);
			MP_WritePhyUshort(sc, 0x06, 0x8e02);
			MP_WritePhyUshort(sc, 0x06, 0x8445);
			MP_WritePhyUshort(sc, 0x06, 0x0204);
			MP_WritePhyUshort(sc, 0x06, 0x0302);
			MP_WritePhyUshort(sc, 0x06, 0x19cc);
			MP_WritePhyUshort(sc, 0x06, 0x022b);
			MP_WritePhyUshort(sc, 0x06, 0x5bfc);
			MP_WritePhyUshort(sc, 0x06, 0x04ee);
			MP_WritePhyUshort(sc, 0x06, 0x8b8d);
			MP_WritePhyUshort(sc, 0x06, 0x0105);
			MP_WritePhyUshort(sc, 0x06, 0xf8f9);
			MP_WritePhyUshort(sc, 0x06, 0xfae0);
			MP_WritePhyUshort(sc, 0x06, 0x8b81);
			MP_WritePhyUshort(sc, 0x06, 0xac26);
			MP_WritePhyUshort(sc, 0x06, 0x08e0);
			MP_WritePhyUshort(sc, 0x06, 0x8b81);
			MP_WritePhyUshort(sc, 0x06, 0xac21);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0x6bee);
			MP_WritePhyUshort(sc, 0x06, 0xe0ea);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0xe0eb);
			MP_WritePhyUshort(sc, 0x06, 0x00e2);
			MP_WritePhyUshort(sc, 0x06, 0xe07c);
			MP_WritePhyUshort(sc, 0x06, 0xe3e0);
			MP_WritePhyUshort(sc, 0x06, 0x7da5);
			MP_WritePhyUshort(sc, 0x06, 0x1111);
			MP_WritePhyUshort(sc, 0x06, 0x15d2);
			MP_WritePhyUshort(sc, 0x06, 0x60d6);
			MP_WritePhyUshort(sc, 0x06, 0x6666);
			MP_WritePhyUshort(sc, 0x06, 0x0207);
			MP_WritePhyUshort(sc, 0x06, 0x6cd2);
			MP_WritePhyUshort(sc, 0x06, 0xa0d6);
			MP_WritePhyUshort(sc, 0x06, 0xaaaa);
			MP_WritePhyUshort(sc, 0x06, 0x0207);
			MP_WritePhyUshort(sc, 0x06, 0x6c02);
			MP_WritePhyUshort(sc, 0x06, 0x201d);
			MP_WritePhyUshort(sc, 0x06, 0xae44);
			MP_WritePhyUshort(sc, 0x06, 0xa566);
			MP_WritePhyUshort(sc, 0x06, 0x6602);
			MP_WritePhyUshort(sc, 0x06, 0xae38);
			MP_WritePhyUshort(sc, 0x06, 0xa5aa);
			MP_WritePhyUshort(sc, 0x06, 0xaa02);
			MP_WritePhyUshort(sc, 0x06, 0xae32);
			MP_WritePhyUshort(sc, 0x06, 0xeee0);
			MP_WritePhyUshort(sc, 0x06, 0xea04);
			MP_WritePhyUshort(sc, 0x06, 0xeee0);
			MP_WritePhyUshort(sc, 0x06, 0xeb06);
			MP_WritePhyUshort(sc, 0x06, 0xe2e0);
			MP_WritePhyUshort(sc, 0x06, 0x7ce3);
			MP_WritePhyUshort(sc, 0x06, 0xe07d);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x38e1);
			MP_WritePhyUshort(sc, 0x06, 0xe039);
			MP_WritePhyUshort(sc, 0x06, 0xad2e);
			MP_WritePhyUshort(sc, 0x06, 0x21ad);
			MP_WritePhyUshort(sc, 0x06, 0x3f13);
			MP_WritePhyUshort(sc, 0x06, 0xe0e4);
			MP_WritePhyUshort(sc, 0x06, 0x14e1);
			MP_WritePhyUshort(sc, 0x06, 0xe415);
			MP_WritePhyUshort(sc, 0x06, 0x6880);
			MP_WritePhyUshort(sc, 0x06, 0xe4e4);
			MP_WritePhyUshort(sc, 0x06, 0x14e5);
			MP_WritePhyUshort(sc, 0x06, 0xe415);
			MP_WritePhyUshort(sc, 0x06, 0x0220);
			MP_WritePhyUshort(sc, 0x06, 0x1dae);
			MP_WritePhyUshort(sc, 0x06, 0x0bac);
			MP_WritePhyUshort(sc, 0x06, 0x3e02);
			MP_WritePhyUshort(sc, 0x06, 0xae06);
			MP_WritePhyUshort(sc, 0x06, 0x0281);
			MP_WritePhyUshort(sc, 0x06, 0x4602);
			MP_WritePhyUshort(sc, 0x06, 0x2057);
			MP_WritePhyUshort(sc, 0x06, 0xfefd);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xf8e0);
			MP_WritePhyUshort(sc, 0x06, 0x8b81);
			MP_WritePhyUshort(sc, 0x06, 0xad26);
			MP_WritePhyUshort(sc, 0x06, 0x0302);
			MP_WritePhyUshort(sc, 0x06, 0x20a7);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x81ad);
			MP_WritePhyUshort(sc, 0x06, 0x2109);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x2eac);
			MP_WritePhyUshort(sc, 0x06, 0x2003);
			MP_WritePhyUshort(sc, 0x06, 0x0281);
			MP_WritePhyUshort(sc, 0x06, 0x61fc);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x81ac);
			MP_WritePhyUshort(sc, 0x06, 0x2505);
			MP_WritePhyUshort(sc, 0x06, 0x0222);
			MP_WritePhyUshort(sc, 0x06, 0xaeae);
			MP_WritePhyUshort(sc, 0x06, 0x0302);
			MP_WritePhyUshort(sc, 0x06, 0x8172);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xf8f9);
			MP_WritePhyUshort(sc, 0x06, 0xfaef);
			MP_WritePhyUshort(sc, 0x06, 0x69fa);
			MP_WritePhyUshort(sc, 0x06, 0xe086);
			MP_WritePhyUshort(sc, 0x06, 0x20a0);
			MP_WritePhyUshort(sc, 0x06, 0x8016);
			MP_WritePhyUshort(sc, 0x06, 0xe086);
			MP_WritePhyUshort(sc, 0x06, 0x21e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b33);
			MP_WritePhyUshort(sc, 0x06, 0x1b10);
			MP_WritePhyUshort(sc, 0x06, 0x9e06);
			MP_WritePhyUshort(sc, 0x06, 0x0223);
			MP_WritePhyUshort(sc, 0x06, 0x91af);
			MP_WritePhyUshort(sc, 0x06, 0x8252);
			MP_WritePhyUshort(sc, 0x06, 0xee86);
			MP_WritePhyUshort(sc, 0x06, 0x2081);
			MP_WritePhyUshort(sc, 0x06, 0xaee4);
			MP_WritePhyUshort(sc, 0x06, 0xa081);
			MP_WritePhyUshort(sc, 0x06, 0x1402);
			MP_WritePhyUshort(sc, 0x06, 0x2399);
			MP_WritePhyUshort(sc, 0x06, 0xbf25);
			MP_WritePhyUshort(sc, 0x06, 0xcc02);
			MP_WritePhyUshort(sc, 0x06, 0x2d21);
			MP_WritePhyUshort(sc, 0x06, 0xee86);
			MP_WritePhyUshort(sc, 0x06, 0x2100);
			MP_WritePhyUshort(sc, 0x06, 0xee86);
			MP_WritePhyUshort(sc, 0x06, 0x2082);
			MP_WritePhyUshort(sc, 0x06, 0xaf82);
			MP_WritePhyUshort(sc, 0x06, 0x52a0);
			MP_WritePhyUshort(sc, 0x06, 0x8232);
			MP_WritePhyUshort(sc, 0x06, 0xe086);
			MP_WritePhyUshort(sc, 0x06, 0x21e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b32);
			MP_WritePhyUshort(sc, 0x06, 0x1b10);
			MP_WritePhyUshort(sc, 0x06, 0x9e06);
			MP_WritePhyUshort(sc, 0x06, 0x0223);
			MP_WritePhyUshort(sc, 0x06, 0x91af);
			MP_WritePhyUshort(sc, 0x06, 0x8252);
			MP_WritePhyUshort(sc, 0x06, 0xee86);
			MP_WritePhyUshort(sc, 0x06, 0x2100);
			MP_WritePhyUshort(sc, 0x06, 0xd000);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0x5910);
			MP_WritePhyUshort(sc, 0x06, 0xa004);
			MP_WritePhyUshort(sc, 0x06, 0xf9e0);
			MP_WritePhyUshort(sc, 0x06, 0x861f);
			MP_WritePhyUshort(sc, 0x06, 0xa000);
			MP_WritePhyUshort(sc, 0x06, 0x07ee);
			MP_WritePhyUshort(sc, 0x06, 0x8620);
			MP_WritePhyUshort(sc, 0x06, 0x83af);
			MP_WritePhyUshort(sc, 0x06, 0x8178);
			MP_WritePhyUshort(sc, 0x06, 0x0224);
			MP_WritePhyUshort(sc, 0x06, 0x0102);
			MP_WritePhyUshort(sc, 0x06, 0x2399);
			MP_WritePhyUshort(sc, 0x06, 0xae72);
			MP_WritePhyUshort(sc, 0x06, 0xa083);
			MP_WritePhyUshort(sc, 0x06, 0x4b1f);
			MP_WritePhyUshort(sc, 0x06, 0x55d0);
			MP_WritePhyUshort(sc, 0x06, 0x04bf);
			MP_WritePhyUshort(sc, 0x06, 0x8615);
			MP_WritePhyUshort(sc, 0x06, 0x1a90);
			MP_WritePhyUshort(sc, 0x06, 0x0c54);
			MP_WritePhyUshort(sc, 0x06, 0xd91e);
			MP_WritePhyUshort(sc, 0x06, 0x31b0);
			MP_WritePhyUshort(sc, 0x06, 0xf4e0);
			MP_WritePhyUshort(sc, 0x06, 0xe022);
			MP_WritePhyUshort(sc, 0x06, 0xe1e0);
			MP_WritePhyUshort(sc, 0x06, 0x23ad);
			MP_WritePhyUshort(sc, 0x06, 0x2e0c);
			MP_WritePhyUshort(sc, 0x06, 0xef02);
			MP_WritePhyUshort(sc, 0x06, 0xef12);
			MP_WritePhyUshort(sc, 0x06, 0x0e44);
			MP_WritePhyUshort(sc, 0x06, 0xef23);
			MP_WritePhyUshort(sc, 0x06, 0x0e54);
			MP_WritePhyUshort(sc, 0x06, 0xef21);
			MP_WritePhyUshort(sc, 0x06, 0xe6e4);
			MP_WritePhyUshort(sc, 0x06, 0x2ae7);
			MP_WritePhyUshort(sc, 0x06, 0xe42b);
			MP_WritePhyUshort(sc, 0x06, 0xe2e4);
			MP_WritePhyUshort(sc, 0x06, 0x28e3);
			MP_WritePhyUshort(sc, 0x06, 0xe429);
			MP_WritePhyUshort(sc, 0x06, 0x6d20);
			MP_WritePhyUshort(sc, 0x06, 0x00e6);
			MP_WritePhyUshort(sc, 0x06, 0xe428);
			MP_WritePhyUshort(sc, 0x06, 0xe7e4);
			MP_WritePhyUshort(sc, 0x06, 0x29bf);
			MP_WritePhyUshort(sc, 0x06, 0x25ca);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0x21ee);
			MP_WritePhyUshort(sc, 0x06, 0x8620);
			MP_WritePhyUshort(sc, 0x06, 0x84ee);
			MP_WritePhyUshort(sc, 0x06, 0x8621);
			MP_WritePhyUshort(sc, 0x06, 0x00af);
			MP_WritePhyUshort(sc, 0x06, 0x8178);
			MP_WritePhyUshort(sc, 0x06, 0xa084);
			MP_WritePhyUshort(sc, 0x06, 0x19e0);
			MP_WritePhyUshort(sc, 0x06, 0x8621);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x341b);
			MP_WritePhyUshort(sc, 0x06, 0x109e);
			MP_WritePhyUshort(sc, 0x06, 0x0602);
			MP_WritePhyUshort(sc, 0x06, 0x2391);
			MP_WritePhyUshort(sc, 0x06, 0xaf82);
			MP_WritePhyUshort(sc, 0x06, 0x5202);
			MP_WritePhyUshort(sc, 0x06, 0x241f);
			MP_WritePhyUshort(sc, 0x06, 0xee86);
			MP_WritePhyUshort(sc, 0x06, 0x2085);
			MP_WritePhyUshort(sc, 0x06, 0xae08);
			MP_WritePhyUshort(sc, 0x06, 0xa085);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0x0302);
			MP_WritePhyUshort(sc, 0x06, 0x2442);
			MP_WritePhyUshort(sc, 0x06, 0xfeef);
			MP_WritePhyUshort(sc, 0x06, 0x96fe);
			MP_WritePhyUshort(sc, 0x06, 0xfdfc);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xf9fa);
			MP_WritePhyUshort(sc, 0x06, 0xef69);
			MP_WritePhyUshort(sc, 0x06, 0xfad1);
			MP_WritePhyUshort(sc, 0x06, 0x801f);
			MP_WritePhyUshort(sc, 0x06, 0x66e2);
			MP_WritePhyUshort(sc, 0x06, 0xe0ea);
			MP_WritePhyUshort(sc, 0x06, 0xe3e0);
			MP_WritePhyUshort(sc, 0x06, 0xeb5a);
			MP_WritePhyUshort(sc, 0x06, 0xf81e);
			MP_WritePhyUshort(sc, 0x06, 0x20e6);
			MP_WritePhyUshort(sc, 0x06, 0xe0ea);
			MP_WritePhyUshort(sc, 0x06, 0xe5e0);
			MP_WritePhyUshort(sc, 0x06, 0xebd3);
			MP_WritePhyUshort(sc, 0x06, 0x05b3);
			MP_WritePhyUshort(sc, 0x06, 0xfee2);
			MP_WritePhyUshort(sc, 0x06, 0xe07c);
			MP_WritePhyUshort(sc, 0x06, 0xe3e0);
			MP_WritePhyUshort(sc, 0x06, 0x7dad);
			MP_WritePhyUshort(sc, 0x06, 0x3703);
			MP_WritePhyUshort(sc, 0x06, 0x7dff);
			MP_WritePhyUshort(sc, 0x06, 0xff0d);
			MP_WritePhyUshort(sc, 0x06, 0x581c);
			MP_WritePhyUshort(sc, 0x06, 0x55f8);
			MP_WritePhyUshort(sc, 0x06, 0xef46);
			MP_WritePhyUshort(sc, 0x06, 0x0282);
			MP_WritePhyUshort(sc, 0x06, 0xc7ef);
			MP_WritePhyUshort(sc, 0x06, 0x65ef);
			MP_WritePhyUshort(sc, 0x06, 0x54fc);
			MP_WritePhyUshort(sc, 0x06, 0xac30);
			MP_WritePhyUshort(sc, 0x06, 0x2b11);
			MP_WritePhyUshort(sc, 0x06, 0xa188);
			MP_WritePhyUshort(sc, 0x06, 0xcabf);
			MP_WritePhyUshort(sc, 0x06, 0x860e);
			MP_WritePhyUshort(sc, 0x06, 0xef10);
			MP_WritePhyUshort(sc, 0x06, 0x0c11);
			MP_WritePhyUshort(sc, 0x06, 0x1a91);
			MP_WritePhyUshort(sc, 0x06, 0xda19);
			MP_WritePhyUshort(sc, 0x06, 0xdbf8);
			MP_WritePhyUshort(sc, 0x06, 0xef46);
			MP_WritePhyUshort(sc, 0x06, 0x021e);
			MP_WritePhyUshort(sc, 0x06, 0x17ef);
			MP_WritePhyUshort(sc, 0x06, 0x54fc);
			MP_WritePhyUshort(sc, 0x06, 0xad30);
			MP_WritePhyUshort(sc, 0x06, 0x0fef);
			MP_WritePhyUshort(sc, 0x06, 0x5689);
			MP_WritePhyUshort(sc, 0x06, 0xde19);
			MP_WritePhyUshort(sc, 0x06, 0xdfe2);
			MP_WritePhyUshort(sc, 0x06, 0x861f);
			MP_WritePhyUshort(sc, 0x06, 0xbf86);
			MP_WritePhyUshort(sc, 0x06, 0x161a);
			MP_WritePhyUshort(sc, 0x06, 0x90de);
			MP_WritePhyUshort(sc, 0x06, 0xfeef);
			MP_WritePhyUshort(sc, 0x06, 0x96fe);
			MP_WritePhyUshort(sc, 0x06, 0xfdfc);
			MP_WritePhyUshort(sc, 0x06, 0x04ac);
			MP_WritePhyUshort(sc, 0x06, 0x2707);
			MP_WritePhyUshort(sc, 0x06, 0xac37);
			MP_WritePhyUshort(sc, 0x06, 0x071a);
			MP_WritePhyUshort(sc, 0x06, 0x54ae);
			MP_WritePhyUshort(sc, 0x06, 0x11ac);
			MP_WritePhyUshort(sc, 0x06, 0x3707);
			MP_WritePhyUshort(sc, 0x06, 0xae00);
			MP_WritePhyUshort(sc, 0x06, 0x1a54);
			MP_WritePhyUshort(sc, 0x06, 0xac37);
			MP_WritePhyUshort(sc, 0x06, 0x07d0);
			MP_WritePhyUshort(sc, 0x06, 0x01d5);
			MP_WritePhyUshort(sc, 0x06, 0xffff);
			MP_WritePhyUshort(sc, 0x06, 0xae02);
			MP_WritePhyUshort(sc, 0x06, 0xd000);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x83ad);
			MP_WritePhyUshort(sc, 0x06, 0x2444);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x22e1);
			MP_WritePhyUshort(sc, 0x06, 0xe023);
			MP_WritePhyUshort(sc, 0x06, 0xad22);
			MP_WritePhyUshort(sc, 0x06, 0x3be0);
			MP_WritePhyUshort(sc, 0x06, 0x8abe);
			MP_WritePhyUshort(sc, 0x06, 0xa000);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x28de);
			MP_WritePhyUshort(sc, 0x06, 0xae42);
			MP_WritePhyUshort(sc, 0x06, 0xa001);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x28f1);
			MP_WritePhyUshort(sc, 0x06, 0xae3a);
			MP_WritePhyUshort(sc, 0x06, 0xa002);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x8344);
			MP_WritePhyUshort(sc, 0x06, 0xae32);
			MP_WritePhyUshort(sc, 0x06, 0xa003);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x299a);
			MP_WritePhyUshort(sc, 0x06, 0xae2a);
			MP_WritePhyUshort(sc, 0x06, 0xa004);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x29ae);
			MP_WritePhyUshort(sc, 0x06, 0xae22);
			MP_WritePhyUshort(sc, 0x06, 0xa005);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x29d7);
			MP_WritePhyUshort(sc, 0x06, 0xae1a);
			MP_WritePhyUshort(sc, 0x06, 0xa006);
			MP_WritePhyUshort(sc, 0x06, 0x0502);
			MP_WritePhyUshort(sc, 0x06, 0x29fe);
			MP_WritePhyUshort(sc, 0x06, 0xae12);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xc000);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xc100);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xc600);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xbe00);
			MP_WritePhyUshort(sc, 0x06, 0xae00);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xf802);
			MP_WritePhyUshort(sc, 0x06, 0x2a67);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x22e1);
			MP_WritePhyUshort(sc, 0x06, 0xe023);
			MP_WritePhyUshort(sc, 0x06, 0x0d06);
			MP_WritePhyUshort(sc, 0x06, 0x5803);
			MP_WritePhyUshort(sc, 0x06, 0xa002);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0x2da0);
			MP_WritePhyUshort(sc, 0x06, 0x0102);
			MP_WritePhyUshort(sc, 0x06, 0xae2d);
			MP_WritePhyUshort(sc, 0x06, 0xa000);
			MP_WritePhyUshort(sc, 0x06, 0x4de0);
			MP_WritePhyUshort(sc, 0x06, 0xe200);
			MP_WritePhyUshort(sc, 0x06, 0xe1e2);
			MP_WritePhyUshort(sc, 0x06, 0x01ad);
			MP_WritePhyUshort(sc, 0x06, 0x2444);
			MP_WritePhyUshort(sc, 0x06, 0xe08a);
			MP_WritePhyUshort(sc, 0x06, 0xc2e4);
			MP_WritePhyUshort(sc, 0x06, 0x8ac4);
			MP_WritePhyUshort(sc, 0x06, 0xe08a);
			MP_WritePhyUshort(sc, 0x06, 0xc3e4);
			MP_WritePhyUshort(sc, 0x06, 0x8ac5);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xbe03);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x83ad);
			MP_WritePhyUshort(sc, 0x06, 0x253a);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xbe05);
			MP_WritePhyUshort(sc, 0x06, 0xae34);
			MP_WritePhyUshort(sc, 0x06, 0xe08a);
			MP_WritePhyUshort(sc, 0x06, 0xceae);
			MP_WritePhyUshort(sc, 0x06, 0x03e0);
			MP_WritePhyUshort(sc, 0x06, 0x8acf);
			MP_WritePhyUshort(sc, 0x06, 0xe18a);
			MP_WritePhyUshort(sc, 0x06, 0xc249);
			MP_WritePhyUshort(sc, 0x06, 0x05e5);
			MP_WritePhyUshort(sc, 0x06, 0x8ac4);
			MP_WritePhyUshort(sc, 0x06, 0xe18a);
			MP_WritePhyUshort(sc, 0x06, 0xc349);
			MP_WritePhyUshort(sc, 0x06, 0x05e5);
			MP_WritePhyUshort(sc, 0x06, 0x8ac5);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xbe05);
			MP_WritePhyUshort(sc, 0x06, 0x022a);
			MP_WritePhyUshort(sc, 0x06, 0xb6ac);
			MP_WritePhyUshort(sc, 0x06, 0x2012);
			MP_WritePhyUshort(sc, 0x06, 0x0283);
			MP_WritePhyUshort(sc, 0x06, 0xbaac);
			MP_WritePhyUshort(sc, 0x06, 0x200c);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xc100);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xc600);
			MP_WritePhyUshort(sc, 0x06, 0xee8a);
			MP_WritePhyUshort(sc, 0x06, 0xbe02);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xd000);
			MP_WritePhyUshort(sc, 0x06, 0x0283);
			MP_WritePhyUshort(sc, 0x06, 0xcc59);
			MP_WritePhyUshort(sc, 0x06, 0x0f39);
			MP_WritePhyUshort(sc, 0x06, 0x02aa);
			MP_WritePhyUshort(sc, 0x06, 0x04d0);
			MP_WritePhyUshort(sc, 0x06, 0x01ae);
			MP_WritePhyUshort(sc, 0x06, 0x02d0);
			MP_WritePhyUshort(sc, 0x06, 0x0004);
			MP_WritePhyUshort(sc, 0x06, 0xf9fa);
			MP_WritePhyUshort(sc, 0x06, 0xe2e2);
			MP_WritePhyUshort(sc, 0x06, 0xd2e3);
			MP_WritePhyUshort(sc, 0x06, 0xe2d3);
			MP_WritePhyUshort(sc, 0x06, 0xf95a);
			MP_WritePhyUshort(sc, 0x06, 0xf7e6);
			MP_WritePhyUshort(sc, 0x06, 0xe2d2);
			MP_WritePhyUshort(sc, 0x06, 0xe7e2);
			MP_WritePhyUshort(sc, 0x06, 0xd3e2);
			MP_WritePhyUshort(sc, 0x06, 0xe02c);
			MP_WritePhyUshort(sc, 0x06, 0xe3e0);
			MP_WritePhyUshort(sc, 0x06, 0x2df9);
			MP_WritePhyUshort(sc, 0x06, 0x5be0);
			MP_WritePhyUshort(sc, 0x06, 0x1e30);
			MP_WritePhyUshort(sc, 0x06, 0xe6e0);
			MP_WritePhyUshort(sc, 0x06, 0x2ce7);
			MP_WritePhyUshort(sc, 0x06, 0xe02d);
			MP_WritePhyUshort(sc, 0x06, 0xe2e2);
			MP_WritePhyUshort(sc, 0x06, 0xcce3);
			MP_WritePhyUshort(sc, 0x06, 0xe2cd);
			MP_WritePhyUshort(sc, 0x06, 0xf95a);
			MP_WritePhyUshort(sc, 0x06, 0x0f6a);
			MP_WritePhyUshort(sc, 0x06, 0x50e6);
			MP_WritePhyUshort(sc, 0x06, 0xe2cc);
			MP_WritePhyUshort(sc, 0x06, 0xe7e2);
			MP_WritePhyUshort(sc, 0x06, 0xcde0);
			MP_WritePhyUshort(sc, 0x06, 0xe03c);
			MP_WritePhyUshort(sc, 0x06, 0xe1e0);
			MP_WritePhyUshort(sc, 0x06, 0x3def);
			MP_WritePhyUshort(sc, 0x06, 0x64fd);
			MP_WritePhyUshort(sc, 0x06, 0xe0e2);
			MP_WritePhyUshort(sc, 0x06, 0xcce1);
			MP_WritePhyUshort(sc, 0x06, 0xe2cd);
			MP_WritePhyUshort(sc, 0x06, 0x580f);
			MP_WritePhyUshort(sc, 0x06, 0x5af0);
			MP_WritePhyUshort(sc, 0x06, 0x1e02);
			MP_WritePhyUshort(sc, 0x06, 0xe4e2);
			MP_WritePhyUshort(sc, 0x06, 0xcce5);
			MP_WritePhyUshort(sc, 0x06, 0xe2cd);
			MP_WritePhyUshort(sc, 0x06, 0xfde0);
			MP_WritePhyUshort(sc, 0x06, 0xe02c);
			MP_WritePhyUshort(sc, 0x06, 0xe1e0);
			MP_WritePhyUshort(sc, 0x06, 0x2d59);
			MP_WritePhyUshort(sc, 0x06, 0xe05b);
			MP_WritePhyUshort(sc, 0x06, 0x1f1e);
			MP_WritePhyUshort(sc, 0x06, 0x13e4);
			MP_WritePhyUshort(sc, 0x06, 0xe02c);
			MP_WritePhyUshort(sc, 0x06, 0xe5e0);
			MP_WritePhyUshort(sc, 0x06, 0x2dfd);
			MP_WritePhyUshort(sc, 0x06, 0xe0e2);
			MP_WritePhyUshort(sc, 0x06, 0xd2e1);
			MP_WritePhyUshort(sc, 0x06, 0xe2d3);
			MP_WritePhyUshort(sc, 0x06, 0x58f7);
			MP_WritePhyUshort(sc, 0x06, 0x5a08);
			MP_WritePhyUshort(sc, 0x06, 0x1e02);
			MP_WritePhyUshort(sc, 0x06, 0xe4e2);
			MP_WritePhyUshort(sc, 0x06, 0xd2e5);
			MP_WritePhyUshort(sc, 0x06, 0xe2d3);
			MP_WritePhyUshort(sc, 0x06, 0xef46);
			MP_WritePhyUshort(sc, 0x06, 0xfefd);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xf9fa);
			MP_WritePhyUshort(sc, 0x06, 0xef69);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x22e1);
			MP_WritePhyUshort(sc, 0x06, 0xe023);
			MP_WritePhyUshort(sc, 0x06, 0x58c4);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x6e1f);
			MP_WritePhyUshort(sc, 0x06, 0x109e);
			MP_WritePhyUshort(sc, 0x06, 0x58e4);
			MP_WritePhyUshort(sc, 0x06, 0x8b6e);
			MP_WritePhyUshort(sc, 0x06, 0xad22);
			MP_WritePhyUshort(sc, 0x06, 0x22ac);
			MP_WritePhyUshort(sc, 0x06, 0x2755);
			MP_WritePhyUshort(sc, 0x06, 0xac26);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0x1ad1);
			MP_WritePhyUshort(sc, 0x06, 0x06bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bba);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x07bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bbd);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x07bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bc0);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1ae);
			MP_WritePhyUshort(sc, 0x06, 0x30d1);
			MP_WritePhyUshort(sc, 0x06, 0x03bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bc3);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x00bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bc6);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x00bf);
			MP_WritePhyUshort(sc, 0x06, 0x84e9);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x0fbf);
			MP_WritePhyUshort(sc, 0x06, 0x3bba);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x01bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bbd);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x01bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bc0);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1ef);
			MP_WritePhyUshort(sc, 0x06, 0x96fe);
			MP_WritePhyUshort(sc, 0x06, 0xfdfc);
			MP_WritePhyUshort(sc, 0x06, 0x04d1);
			MP_WritePhyUshort(sc, 0x06, 0x00bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bc3);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d0);
			MP_WritePhyUshort(sc, 0x06, 0x1102);
			MP_WritePhyUshort(sc, 0x06, 0x2bfb);
			MP_WritePhyUshort(sc, 0x06, 0x5903);
			MP_WritePhyUshort(sc, 0x06, 0xef01);
			MP_WritePhyUshort(sc, 0x06, 0xd100);
			MP_WritePhyUshort(sc, 0x06, 0xa000);
			MP_WritePhyUshort(sc, 0x06, 0x02d1);
			MP_WritePhyUshort(sc, 0x06, 0x01bf);
			MP_WritePhyUshort(sc, 0x06, 0x3bc6);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1d1);
			MP_WritePhyUshort(sc, 0x06, 0x11ad);
			MP_WritePhyUshort(sc, 0x06, 0x2002);
			MP_WritePhyUshort(sc, 0x06, 0x0c11);
			MP_WritePhyUshort(sc, 0x06, 0xad21);
			MP_WritePhyUshort(sc, 0x06, 0x020c);
			MP_WritePhyUshort(sc, 0x06, 0x12bf);
			MP_WritePhyUshort(sc, 0x06, 0x84e9);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1ae);
			MP_WritePhyUshort(sc, 0x06, 0xc870);
			MP_WritePhyUshort(sc, 0x06, 0xe426);
			MP_WritePhyUshort(sc, 0x06, 0x0284);
			MP_WritePhyUshort(sc, 0x06, 0xf005);
			MP_WritePhyUshort(sc, 0x06, 0xf8fa);
			MP_WritePhyUshort(sc, 0x06, 0xef69);
			MP_WritePhyUshort(sc, 0x06, 0xe0e2);
			MP_WritePhyUshort(sc, 0x06, 0xfee1);
			MP_WritePhyUshort(sc, 0x06, 0xe2ff);
			MP_WritePhyUshort(sc, 0x06, 0xad2d);
			MP_WritePhyUshort(sc, 0x06, 0x1ae0);
			MP_WritePhyUshort(sc, 0x06, 0xe14e);
			MP_WritePhyUshort(sc, 0x06, 0xe1e1);
			MP_WritePhyUshort(sc, 0x06, 0x4fac);
			MP_WritePhyUshort(sc, 0x06, 0x2d22);
			MP_WritePhyUshort(sc, 0x06, 0xf603);
			MP_WritePhyUshort(sc, 0x06, 0x0203);
			MP_WritePhyUshort(sc, 0x06, 0x3bf7);
			MP_WritePhyUshort(sc, 0x06, 0x03f7);
			MP_WritePhyUshort(sc, 0x06, 0x06bf);
			MP_WritePhyUshort(sc, 0x06, 0x85c4);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0x21ae);
			MP_WritePhyUshort(sc, 0x06, 0x11e0);
			MP_WritePhyUshort(sc, 0x06, 0xe14e);
			MP_WritePhyUshort(sc, 0x06, 0xe1e1);
			MP_WritePhyUshort(sc, 0x06, 0x4fad);
			MP_WritePhyUshort(sc, 0x06, 0x2d08);
			MP_WritePhyUshort(sc, 0x06, 0xbf85);
			MP_WritePhyUshort(sc, 0x06, 0xcf02);
			MP_WritePhyUshort(sc, 0x06, 0x2d21);
			MP_WritePhyUshort(sc, 0x06, 0xf606);
			MP_WritePhyUshort(sc, 0x06, 0xef96);
			MP_WritePhyUshort(sc, 0x06, 0xfefc);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xfaef);
			MP_WritePhyUshort(sc, 0x06, 0x6902);
			MP_WritePhyUshort(sc, 0x06, 0x8561);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x00e1);
			MP_WritePhyUshort(sc, 0x06, 0xe001);
			MP_WritePhyUshort(sc, 0x06, 0xad27);
			MP_WritePhyUshort(sc, 0x06, 0x1fd1);
			MP_WritePhyUshort(sc, 0x06, 0x01bf);
			MP_WritePhyUshort(sc, 0x06, 0x85be);
			MP_WritePhyUshort(sc, 0x06, 0x022d);
			MP_WritePhyUshort(sc, 0x06, 0xc1e0);
			MP_WritePhyUshort(sc, 0x06, 0xe020);
			MP_WritePhyUshort(sc, 0x06, 0xe1e0);
			MP_WritePhyUshort(sc, 0x06, 0x21ad);
			MP_WritePhyUshort(sc, 0x06, 0x200e);
			MP_WritePhyUshort(sc, 0x06, 0xd100);
			MP_WritePhyUshort(sc, 0x06, 0xbf85);
			MP_WritePhyUshort(sc, 0x06, 0xbe02);
			MP_WritePhyUshort(sc, 0x06, 0x2dc1);
			MP_WritePhyUshort(sc, 0x06, 0xbf3b);
			MP_WritePhyUshort(sc, 0x06, 0x9602);
			MP_WritePhyUshort(sc, 0x06, 0x2d21);
			MP_WritePhyUshort(sc, 0x06, 0xef96);
			MP_WritePhyUshort(sc, 0x06, 0xfefc);
			MP_WritePhyUshort(sc, 0x06, 0x04f8);
			MP_WritePhyUshort(sc, 0x06, 0xf9fa);
			MP_WritePhyUshort(sc, 0x06, 0xef69);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x87ad);
			MP_WritePhyUshort(sc, 0x06, 0x204c);
			MP_WritePhyUshort(sc, 0x06, 0xd200);
			MP_WritePhyUshort(sc, 0x06, 0xe0e2);
			MP_WritePhyUshort(sc, 0x06, 0x0058);
			MP_WritePhyUshort(sc, 0x06, 0x010c);
			MP_WritePhyUshort(sc, 0x06, 0x021e);
			MP_WritePhyUshort(sc, 0x06, 0x20e0);
			MP_WritePhyUshort(sc, 0x06, 0xe000);
			MP_WritePhyUshort(sc, 0x06, 0x5810);
			MP_WritePhyUshort(sc, 0x06, 0x1e20);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x3658);
			MP_WritePhyUshort(sc, 0x06, 0x031e);
			MP_WritePhyUshort(sc, 0x06, 0x20e0);
			MP_WritePhyUshort(sc, 0x06, 0xe022);
			MP_WritePhyUshort(sc, 0x06, 0xe1e0);
			MP_WritePhyUshort(sc, 0x06, 0x2358);
			MP_WritePhyUshort(sc, 0x06, 0xe01e);
			MP_WritePhyUshort(sc, 0x06, 0x20e0);
			MP_WritePhyUshort(sc, 0x06, 0x8b64);
			MP_WritePhyUshort(sc, 0x06, 0x1f02);
			MP_WritePhyUshort(sc, 0x06, 0x9e22);
			MP_WritePhyUshort(sc, 0x06, 0xe68b);
			MP_WritePhyUshort(sc, 0x06, 0x64ad);
			MP_WritePhyUshort(sc, 0x06, 0x3214);
			MP_WritePhyUshort(sc, 0x06, 0xad34);
			MP_WritePhyUshort(sc, 0x06, 0x11ef);
			MP_WritePhyUshort(sc, 0x06, 0x0258);
			MP_WritePhyUshort(sc, 0x06, 0x039e);
			MP_WritePhyUshort(sc, 0x06, 0x07ad);
			MP_WritePhyUshort(sc, 0x06, 0x3508);
			MP_WritePhyUshort(sc, 0x06, 0x5ac0);
			MP_WritePhyUshort(sc, 0x06, 0x9f04);
			MP_WritePhyUshort(sc, 0x06, 0xd101);
			MP_WritePhyUshort(sc, 0x06, 0xae02);
			MP_WritePhyUshort(sc, 0x06, 0xd100);
			MP_WritePhyUshort(sc, 0x06, 0xbf85);
			MP_WritePhyUshort(sc, 0x06, 0xc102);
			MP_WritePhyUshort(sc, 0x06, 0x2dc1);
			MP_WritePhyUshort(sc, 0x06, 0xef96);
			MP_WritePhyUshort(sc, 0x06, 0xfefd);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0x00e2);
			MP_WritePhyUshort(sc, 0x06, 0x34cc);
			MP_WritePhyUshort(sc, 0x06, 0xe200);
			MP_WritePhyUshort(sc, 0x06, 0xa725);
			MP_WritePhyUshort(sc, 0x06, 0xe50a);
			MP_WritePhyUshort(sc, 0x06, 0x1de5);
			MP_WritePhyUshort(sc, 0x06, 0x0a2c);
			MP_WritePhyUshort(sc, 0x06, 0xe50a);
			MP_WritePhyUshort(sc, 0x06, 0x6de5);
			MP_WritePhyUshort(sc, 0x06, 0x0a1d);
			MP_WritePhyUshort(sc, 0x06, 0xe50a);
			MP_WritePhyUshort(sc, 0x06, 0x1ce5);
			MP_WritePhyUshort(sc, 0x06, 0x0a2d);
			MP_WritePhyUshort(sc, 0x06, 0xa755);
			MP_WritePhyUshort(sc, 0x05, 0x8b64);
			MP_WritePhyUshort(sc, 0x06, 0x0000);
			MP_WritePhyUshort(sc, 0x05, 0x8b94);
			MP_WritePhyUshort(sc, 0x06, 0x84ec);
			Data = MP_ReadPhyUshort(sc, 0x01);
			Data |= 0x0001;
			MP_WritePhyUshort(sc, 0x01, Data);
			MP_WritePhyUshort(sc, 0x00, 0x0005);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			for (i = 0; i < 200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x00);
				if (Data & 0x0080)
					break;
			}
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			MP_WritePhyUshort(sc, 0x17, 0x0116);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0028);
			MP_WritePhyUshort(sc, 0x15, 0x0010);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			//MP_WritePhyUshort(sc, 0x1e, 0x0020);
			//MP_WritePhyUshort(sc, 0x15, 0x0100);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0041);
			MP_WritePhyUshort(sc, 0x15, 0x0802);
			MP_WritePhyUshort(sc, 0x16, 0x2185);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
		}
		else if (sc->re_type == MACFG_37)
		{
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x00, 0x1800);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			MP_WritePhyUshort(sc, 0x17, 0x0117);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1E, 0x002C);
			MP_WritePhyUshort(sc, 0x1B, 0x5000);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x16, 0x4104);
			for (i=0; i<200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x1E);
				Data &= 0x03FF;
				if (Data== 0x000C)
					break;
			}
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			for (i=0; i<200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x07);
				if ((Data&0x0020)==0)
					break;
			}
			Data = MP_ReadPhyUshort(sc, 0x07);
			if (Data & 0x0020)
			{
				MP_WritePhyUshort(sc, 0x1f, 0x0007);
				MP_WritePhyUshort(sc, 0x1e, 0x00a1);
				MP_WritePhyUshort(sc, 0x17, 0x1000);
				MP_WritePhyUshort(sc, 0x17, 0x0000);
				MP_WritePhyUshort(sc, 0x17, 0x2000);
				MP_WritePhyUshort(sc, 0x1e, 0x002f);
				MP_WritePhyUshort(sc, 0x18, 0x9bfb);
				MP_WritePhyUshort(sc, 0x1f, 0x0005);
				MP_WritePhyUshort(sc, 0x07, 0x0000);
				MP_WritePhyUshort(sc, 0x1f, 0x0000);
			}
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x0080);
			Data = MP_ReadPhyUshort(sc, 0x00);
			Data &= ~0x0080;
			MP_WritePhyUshort(sc, 0x00, Data);
			MP_WritePhyUshort(sc, 0x1f, 0x0002);
			Data = MP_ReadPhyUshort(sc, 0x08);
			Data &= ~0x0080;
			MP_WritePhyUshort(sc, 0x08, Data);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			MP_WritePhyUshort(sc, 0x16, 0x0306);
			MP_WritePhyUshort(sc, 0x16, 0x0307);
			MP_WritePhyUshort(sc, 0x15, 0x000e);
			MP_WritePhyUshort(sc, 0x19, 0x000a);
			MP_WritePhyUshort(sc, 0x15, 0x0010);
			MP_WritePhyUshort(sc, 0x19, 0x0008);
			MP_WritePhyUshort(sc, 0x15, 0x0018);
			MP_WritePhyUshort(sc, 0x19, 0x4801);
			MP_WritePhyUshort(sc, 0x15, 0x0019);
			MP_WritePhyUshort(sc, 0x19, 0x6801);
			MP_WritePhyUshort(sc, 0x15, 0x001a);
			MP_WritePhyUshort(sc, 0x19, 0x66a1);
			MP_WritePhyUshort(sc, 0x15, 0x001f);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0020);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0021);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0022);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0023);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0024);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0025);
			MP_WritePhyUshort(sc, 0x19, 0x64a1);
			MP_WritePhyUshort(sc, 0x15, 0x0026);
			MP_WritePhyUshort(sc, 0x19, 0x40ea);
			MP_WritePhyUshort(sc, 0x15, 0x0027);
			MP_WritePhyUshort(sc, 0x19, 0x4503);
			MP_WritePhyUshort(sc, 0x15, 0x0028);
			MP_WritePhyUshort(sc, 0x19, 0x9f00);
			MP_WritePhyUshort(sc, 0x15, 0x0029);
			MP_WritePhyUshort(sc, 0x19, 0xa631);
			MP_WritePhyUshort(sc, 0x15, 0x002a);
			MP_WritePhyUshort(sc, 0x19, 0x9717);
			MP_WritePhyUshort(sc, 0x15, 0x002b);
			MP_WritePhyUshort(sc, 0x19, 0x302c);
			MP_WritePhyUshort(sc, 0x15, 0x002c);
			MP_WritePhyUshort(sc, 0x19, 0x4802);
			MP_WritePhyUshort(sc, 0x15, 0x002d);
			MP_WritePhyUshort(sc, 0x19, 0x58da);
			MP_WritePhyUshort(sc, 0x15, 0x002e);
			MP_WritePhyUshort(sc, 0x19, 0x400d);
			MP_WritePhyUshort(sc, 0x15, 0x002f);
			MP_WritePhyUshort(sc, 0x19, 0x4488);
			MP_WritePhyUshort(sc, 0x15, 0x0030);
			MP_WritePhyUshort(sc, 0x19, 0x9e00);
			MP_WritePhyUshort(sc, 0x15, 0x0031);
			MP_WritePhyUshort(sc, 0x19, 0x63c8);
			MP_WritePhyUshort(sc, 0x15, 0x0032);
			MP_WritePhyUshort(sc, 0x19, 0x6481);
			MP_WritePhyUshort(sc, 0x15, 0x0033);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0034);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0035);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0036);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0037);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0038);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0039);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x003a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x003b);
			MP_WritePhyUshort(sc, 0x19, 0x63e8);
			MP_WritePhyUshort(sc, 0x15, 0x003c);
			MP_WritePhyUshort(sc, 0x19, 0x7d00);
			MP_WritePhyUshort(sc, 0x15, 0x003d);
			MP_WritePhyUshort(sc, 0x19, 0x59d4);
			MP_WritePhyUshort(sc, 0x15, 0x003e);
			MP_WritePhyUshort(sc, 0x19, 0x63f8);
			MP_WritePhyUshort(sc, 0x15, 0x0040);
			MP_WritePhyUshort(sc, 0x19, 0x64a1);
			MP_WritePhyUshort(sc, 0x15, 0x0041);
			MP_WritePhyUshort(sc, 0x19, 0x30de);
			MP_WritePhyUshort(sc, 0x15, 0x0044);
			MP_WritePhyUshort(sc, 0x19, 0x480f);
			MP_WritePhyUshort(sc, 0x15, 0x0045);
			MP_WritePhyUshort(sc, 0x19, 0x6800);
			MP_WritePhyUshort(sc, 0x15, 0x0046);
			MP_WritePhyUshort(sc, 0x19, 0x6680);
			MP_WritePhyUshort(sc, 0x15, 0x0047);
			MP_WritePhyUshort(sc, 0x19, 0x7c10);
			MP_WritePhyUshort(sc, 0x15, 0x0048);
			MP_WritePhyUshort(sc, 0x19, 0x63c8);
			MP_WritePhyUshort(sc, 0x15, 0x0049);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004b);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004c);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004d);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004e);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x004f);
			MP_WritePhyUshort(sc, 0x19, 0x40ea);
			MP_WritePhyUshort(sc, 0x15, 0x0050);
			MP_WritePhyUshort(sc, 0x19, 0x4503);
			MP_WritePhyUshort(sc, 0x15, 0x0051);
			MP_WritePhyUshort(sc, 0x19, 0x58ca);
			MP_WritePhyUshort(sc, 0x15, 0x0052);
			MP_WritePhyUshort(sc, 0x19, 0x63c8);
			MP_WritePhyUshort(sc, 0x15, 0x0053);
			MP_WritePhyUshort(sc, 0x19, 0x63d8);
			MP_WritePhyUshort(sc, 0x15, 0x0054);
			MP_WritePhyUshort(sc, 0x19, 0x66a0);
			MP_WritePhyUshort(sc, 0x15, 0x0055);
			MP_WritePhyUshort(sc, 0x19, 0x9f00);
			MP_WritePhyUshort(sc, 0x15, 0x0056);
			MP_WritePhyUshort(sc, 0x19, 0x3000);
			MP_WritePhyUshort(sc, 0x15, 0x00a1);
			MP_WritePhyUshort(sc, 0x19, 0x3044);
			MP_WritePhyUshort(sc, 0x15, 0x00ab);
			MP_WritePhyUshort(sc, 0x19, 0x5820);
			MP_WritePhyUshort(sc, 0x15, 0x00ac);
			MP_WritePhyUshort(sc, 0x19, 0x5e04);
			MP_WritePhyUshort(sc, 0x15, 0x00ad);
			MP_WritePhyUshort(sc, 0x19, 0xb60c);
			MP_WritePhyUshort(sc, 0x15, 0x00af);
			MP_WritePhyUshort(sc, 0x19, 0x000a);
			MP_WritePhyUshort(sc, 0x15, 0x00b2);
			MP_WritePhyUshort(sc, 0x19, 0x30b9);
			MP_WritePhyUshort(sc, 0x15, 0x00b9);
			MP_WritePhyUshort(sc, 0x19, 0x4408);
			MP_WritePhyUshort(sc, 0x15, 0x00ba);
			MP_WritePhyUshort(sc, 0x19, 0x480b);
			MP_WritePhyUshort(sc, 0x15, 0x00bb);
			MP_WritePhyUshort(sc, 0x19, 0x5e00);
			MP_WritePhyUshort(sc, 0x15, 0x00bc);
			MP_WritePhyUshort(sc, 0x19, 0x405f);
			MP_WritePhyUshort(sc, 0x15, 0x00bd);
			MP_WritePhyUshort(sc, 0x19, 0x4448);
			MP_WritePhyUshort(sc, 0x15, 0x00be);
			MP_WritePhyUshort(sc, 0x19, 0x4020);
			MP_WritePhyUshort(sc, 0x15, 0x00bf);
			MP_WritePhyUshort(sc, 0x19, 0x4468);
			MP_WritePhyUshort(sc, 0x15, 0x00c0);
			MP_WritePhyUshort(sc, 0x19, 0x9c02);
			MP_WritePhyUshort(sc, 0x15, 0x00c1);
			MP_WritePhyUshort(sc, 0x19, 0x58a0);
			MP_WritePhyUshort(sc, 0x15, 0x00c2);
			MP_WritePhyUshort(sc, 0x19, 0xb605);
			MP_WritePhyUshort(sc, 0x15, 0x00c3);
			MP_WritePhyUshort(sc, 0x19, 0xc0d3);
			MP_WritePhyUshort(sc, 0x15, 0x00c4);
			MP_WritePhyUshort(sc, 0x19, 0x00e6);
			MP_WritePhyUshort(sc, 0x15, 0x00c5);
			MP_WritePhyUshort(sc, 0x19, 0xdaec);
			MP_WritePhyUshort(sc, 0x15, 0x00c6);
			MP_WritePhyUshort(sc, 0x19, 0x00fa);
			MP_WritePhyUshort(sc, 0x15, 0x00c7);
			MP_WritePhyUshort(sc, 0x19, 0x9df9);
			MP_WritePhyUshort(sc, 0x15, 0x0112);
			MP_WritePhyUshort(sc, 0x19, 0x6421);
			MP_WritePhyUshort(sc, 0x15, 0x0113);
			MP_WritePhyUshort(sc, 0x19, 0x7c08);
			MP_WritePhyUshort(sc, 0x15, 0x0114);
			MP_WritePhyUshort(sc, 0x19, 0x63f0);
			MP_WritePhyUshort(sc, 0x15, 0x0115);
			MP_WritePhyUshort(sc, 0x19, 0x4003);
			MP_WritePhyUshort(sc, 0x15, 0x0116);
			MP_WritePhyUshort(sc, 0x19, 0x4418);
			MP_WritePhyUshort(sc, 0x15, 0x0117);
			MP_WritePhyUshort(sc, 0x19, 0x9b00);
			MP_WritePhyUshort(sc, 0x15, 0x0118);
			MP_WritePhyUshort(sc, 0x19, 0x6461);
			MP_WritePhyUshort(sc, 0x15, 0x0119);
			MP_WritePhyUshort(sc, 0x19, 0x64e1);
			MP_WritePhyUshort(sc, 0x15, 0x011a);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0150);
			MP_WritePhyUshort(sc, 0x19, 0x7c80);
			MP_WritePhyUshort(sc, 0x15, 0x0151);
			MP_WritePhyUshort(sc, 0x19, 0x6461);
			MP_WritePhyUshort(sc, 0x15, 0x0152);
			MP_WritePhyUshort(sc, 0x19, 0x4003);
			MP_WritePhyUshort(sc, 0x15, 0x0153);
			MP_WritePhyUshort(sc, 0x19, 0x4540);
			MP_WritePhyUshort(sc, 0x15, 0x0154);
			MP_WritePhyUshort(sc, 0x19, 0x9f00);
			MP_WritePhyUshort(sc, 0x15, 0x0155);
			MP_WritePhyUshort(sc, 0x19, 0x9d00);
			MP_WritePhyUshort(sc, 0x15, 0x0156);
			MP_WritePhyUshort(sc, 0x19, 0x7c40);
			MP_WritePhyUshort(sc, 0x15, 0x0157);
			MP_WritePhyUshort(sc, 0x19, 0x6421);
			MP_WritePhyUshort(sc, 0x15, 0x0158);
			MP_WritePhyUshort(sc, 0x19, 0x7c80);
			MP_WritePhyUshort(sc, 0x15, 0x0159);
			MP_WritePhyUshort(sc, 0x19, 0x64a1);
			MP_WritePhyUshort(sc, 0x15, 0x015a);
			MP_WritePhyUshort(sc, 0x19, 0x30fe);
			MP_WritePhyUshort(sc, 0x15, 0x02e7);
			MP_WritePhyUshort(sc, 0x19, 0x0000);
			MP_WritePhyUshort(sc, 0x15, 0x0329);
			MP_WritePhyUshort(sc, 0x19, 0x7c00);
			MP_WritePhyUshort(sc, 0x15, 0x0382);
			MP_WritePhyUshort(sc, 0x19, 0x7c40);
			MP_WritePhyUshort(sc, 0x15, 0x03bd);
			MP_WritePhyUshort(sc, 0x19, 0x405e);
			MP_WritePhyUshort(sc, 0x15, 0x03c9);
			MP_WritePhyUshort(sc, 0x19, 0x7c00);
			MP_WritePhyUshort(sc, 0x15, 0x03e3);
			MP_WritePhyUshort(sc, 0x19, 0x7c00);
			MP_WritePhyUshort(sc, 0x16, 0x0306);
			MP_WritePhyUshort(sc, 0x16, 0x0300);
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			MP_WritePhyUshort(sc, 0x05, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x0080);
			MP_WritePhyUshort(sc, 0x05, 0x8000);
			MP_WritePhyUshort(sc, 0x06, 0x0280);
			MP_WritePhyUshort(sc, 0x06, 0x48f7);
			MP_WritePhyUshort(sc, 0x06, 0x00e0);
			MP_WritePhyUshort(sc, 0x06, 0xfff7);
			MP_WritePhyUshort(sc, 0x06, 0xa080);
			MP_WritePhyUshort(sc, 0x06, 0x02ae);
			MP_WritePhyUshort(sc, 0x06, 0xf602);
			MP_WritePhyUshort(sc, 0x06, 0x0200);
			MP_WritePhyUshort(sc, 0x06, 0x0202);
			MP_WritePhyUshort(sc, 0x06, 0x1102);
			MP_WritePhyUshort(sc, 0x06, 0x0224);
			MP_WritePhyUshort(sc, 0x06, 0x0202);
			MP_WritePhyUshort(sc, 0x06, 0x3402);
			MP_WritePhyUshort(sc, 0x06, 0x027f);
			MP_WritePhyUshort(sc, 0x06, 0x0202);
			MP_WritePhyUshort(sc, 0x06, 0x9202);
			MP_WritePhyUshort(sc, 0x06, 0x8078);
			MP_WritePhyUshort(sc, 0x06, 0xe08b);
			MP_WritePhyUshort(sc, 0x06, 0x88e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b89);
			MP_WritePhyUshort(sc, 0x06, 0x1e01);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x8a1e);
			MP_WritePhyUshort(sc, 0x06, 0x01e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b8b);
			MP_WritePhyUshort(sc, 0x06, 0x1e01);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x8c1e);
			MP_WritePhyUshort(sc, 0x06, 0x01e1);
			MP_WritePhyUshort(sc, 0x06, 0x8b8d);
			MP_WritePhyUshort(sc, 0x06, 0x1e01);
			MP_WritePhyUshort(sc, 0x06, 0xe18b);
			MP_WritePhyUshort(sc, 0x06, 0x8e1e);
			MP_WritePhyUshort(sc, 0x06, 0x01a0);
			MP_WritePhyUshort(sc, 0x06, 0x00c7);
			MP_WritePhyUshort(sc, 0x06, 0xaebb);
			MP_WritePhyUshort(sc, 0x06, 0xee85);
			MP_WritePhyUshort(sc, 0x06, 0x0000);
			MP_WritePhyUshort(sc, 0x06, 0xd480);
			MP_WritePhyUshort(sc, 0x06, 0xebe4);
			MP_WritePhyUshort(sc, 0x06, 0x8b94);
			MP_WritePhyUshort(sc, 0x06, 0xe58b);
			MP_WritePhyUshort(sc, 0x06, 0x95bf);
			MP_WritePhyUshort(sc, 0x06, 0x8b88);
			MP_WritePhyUshort(sc, 0x06, 0xec00);
			MP_WritePhyUshort(sc, 0x06, 0x19a9);
			MP_WritePhyUshort(sc, 0x06, 0x8b90);
			MP_WritePhyUshort(sc, 0x06, 0xf9ee);
			MP_WritePhyUshort(sc, 0x06, 0xfff6);
			MP_WritePhyUshort(sc, 0x06, 0x00ee);
			MP_WritePhyUshort(sc, 0x06, 0xfff7);
			MP_WritePhyUshort(sc, 0x06, 0xffe0);
			MP_WritePhyUshort(sc, 0x06, 0xe140);
			MP_WritePhyUshort(sc, 0x06, 0xe1e1);
			MP_WritePhyUshort(sc, 0x06, 0x41f7);
			MP_WritePhyUshort(sc, 0x06, 0x2ff6);
			MP_WritePhyUshort(sc, 0x06, 0x28e4);
			MP_WritePhyUshort(sc, 0x06, 0xe140);
			MP_WritePhyUshort(sc, 0x06, 0xe5e1);
			MP_WritePhyUshort(sc, 0x06, 0x4104);
			MP_WritePhyUshort(sc, 0x06, 0xf8e0);
			MP_WritePhyUshort(sc, 0x06, 0x8b8e);
			MP_WritePhyUshort(sc, 0x06, 0xad20);
			MP_WritePhyUshort(sc, 0x06, 0x11f6);
			MP_WritePhyUshort(sc, 0x06, 0x20e4);
			MP_WritePhyUshort(sc, 0x06, 0x8b8e);
			MP_WritePhyUshort(sc, 0x06, 0x0280);
			MP_WritePhyUshort(sc, 0x06, 0xba02);
			MP_WritePhyUshort(sc, 0x06, 0x1bf4);
			MP_WritePhyUshort(sc, 0x06, 0x022c);
			MP_WritePhyUshort(sc, 0x06, 0x9c02);
			MP_WritePhyUshort(sc, 0x06, 0x812c);
			MP_WritePhyUshort(sc, 0x06, 0xad22);
			MP_WritePhyUshort(sc, 0x06, 0x11f6);
			MP_WritePhyUshort(sc, 0x06, 0x22e4);
			MP_WritePhyUshort(sc, 0x06, 0x8b8e);
			MP_WritePhyUshort(sc, 0x06, 0x022c);
			MP_WritePhyUshort(sc, 0x06, 0x4602);
			MP_WritePhyUshort(sc, 0x06, 0x2ac5);
			MP_WritePhyUshort(sc, 0x06, 0x0229);
			MP_WritePhyUshort(sc, 0x06, 0x2002);
			MP_WritePhyUshort(sc, 0x06, 0x2b91);
			MP_WritePhyUshort(sc, 0x06, 0xad25);
			MP_WritePhyUshort(sc, 0x06, 0x11f6);
			MP_WritePhyUshort(sc, 0x06, 0x25e4);
			MP_WritePhyUshort(sc, 0x06, 0x8b8e);
			MP_WritePhyUshort(sc, 0x06, 0x0203);
			MP_WritePhyUshort(sc, 0x06, 0x5a02);
			MP_WritePhyUshort(sc, 0x06, 0x043a);
			MP_WritePhyUshort(sc, 0x06, 0x021a);
			MP_WritePhyUshort(sc, 0x06, 0x5902);
			MP_WritePhyUshort(sc, 0x06, 0x2bfc);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xf8fa);
			MP_WritePhyUshort(sc, 0x06, 0xef69);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x00e1);
			MP_WritePhyUshort(sc, 0x06, 0xe001);
			MP_WritePhyUshort(sc, 0x06, 0xad27);
			MP_WritePhyUshort(sc, 0x06, 0x1fd1);
			MP_WritePhyUshort(sc, 0x06, 0x01bf);
			MP_WritePhyUshort(sc, 0x06, 0x819f);
			MP_WritePhyUshort(sc, 0x06, 0x022f);
			MP_WritePhyUshort(sc, 0x06, 0x50e0);
			MP_WritePhyUshort(sc, 0x06, 0xe020);
			MP_WritePhyUshort(sc, 0x06, 0xe1e0);
			MP_WritePhyUshort(sc, 0x06, 0x21ad);
			MP_WritePhyUshort(sc, 0x06, 0x200e);
			MP_WritePhyUshort(sc, 0x06, 0xd100);
			MP_WritePhyUshort(sc, 0x06, 0xbf81);
			MP_WritePhyUshort(sc, 0x06, 0x9f02);
			MP_WritePhyUshort(sc, 0x06, 0x2f50);
			MP_WritePhyUshort(sc, 0x06, 0xbf3d);
			MP_WritePhyUshort(sc, 0x06, 0x3902);
			MP_WritePhyUshort(sc, 0x06, 0x2eb0);
			MP_WritePhyUshort(sc, 0x06, 0xef96);
			MP_WritePhyUshort(sc, 0x06, 0xfefc);
			MP_WritePhyUshort(sc, 0x06, 0x0402);
			MP_WritePhyUshort(sc, 0x06, 0x80ef);
			MP_WritePhyUshort(sc, 0x06, 0x05f8);
			MP_WritePhyUshort(sc, 0x06, 0xfaef);
			MP_WritePhyUshort(sc, 0x06, 0x69e0);
			MP_WritePhyUshort(sc, 0x06, 0xe2fe);
			MP_WritePhyUshort(sc, 0x06, 0xe1e2);
			MP_WritePhyUshort(sc, 0x06, 0xffad);
			MP_WritePhyUshort(sc, 0x06, 0x2d1a);
			MP_WritePhyUshort(sc, 0x06, 0xe0e1);
			MP_WritePhyUshort(sc, 0x06, 0x4ee1);
			MP_WritePhyUshort(sc, 0x06, 0xe14f);
			MP_WritePhyUshort(sc, 0x06, 0xac2d);
			MP_WritePhyUshort(sc, 0x06, 0x22f6);
			MP_WritePhyUshort(sc, 0x06, 0x0302);
			MP_WritePhyUshort(sc, 0x06, 0x0336);
			MP_WritePhyUshort(sc, 0x06, 0xf703);
			MP_WritePhyUshort(sc, 0x06, 0xf706);
			MP_WritePhyUshort(sc, 0x06, 0xbf81);
			MP_WritePhyUshort(sc, 0x06, 0x8902);
			MP_WritePhyUshort(sc, 0x06, 0x2eb0);
			MP_WritePhyUshort(sc, 0x06, 0xae11);
			MP_WritePhyUshort(sc, 0x06, 0xe0e1);
			MP_WritePhyUshort(sc, 0x06, 0x4ee1);
			MP_WritePhyUshort(sc, 0x06, 0xe14f);
			MP_WritePhyUshort(sc, 0x06, 0xad2d);
			MP_WritePhyUshort(sc, 0x06, 0x08bf);
			MP_WritePhyUshort(sc, 0x06, 0x8194);
			MP_WritePhyUshort(sc, 0x06, 0x022e);
			MP_WritePhyUshort(sc, 0x06, 0xb0f6);
			MP_WritePhyUshort(sc, 0x06, 0x06ef);
			MP_WritePhyUshort(sc, 0x06, 0x96fe);
			MP_WritePhyUshort(sc, 0x06, 0xfc04);
			MP_WritePhyUshort(sc, 0x06, 0xf8f9);
			MP_WritePhyUshort(sc, 0x06, 0xfaef);
			MP_WritePhyUshort(sc, 0x06, 0x69e0);
			MP_WritePhyUshort(sc, 0x06, 0x8b87);
			MP_WritePhyUshort(sc, 0x06, 0xad20);
			MP_WritePhyUshort(sc, 0x06, 0x4cd2);
			MP_WritePhyUshort(sc, 0x06, 0x00e0);
			MP_WritePhyUshort(sc, 0x06, 0xe200);
			MP_WritePhyUshort(sc, 0x06, 0x5801);
			MP_WritePhyUshort(sc, 0x06, 0x0c02);
			MP_WritePhyUshort(sc, 0x06, 0x1e20);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x0058);
			MP_WritePhyUshort(sc, 0x06, 0x101e);
			MP_WritePhyUshort(sc, 0x06, 0x20e0);
			MP_WritePhyUshort(sc, 0x06, 0xe036);
			MP_WritePhyUshort(sc, 0x06, 0x5803);
			MP_WritePhyUshort(sc, 0x06, 0x1e20);
			MP_WritePhyUshort(sc, 0x06, 0xe0e0);
			MP_WritePhyUshort(sc, 0x06, 0x22e1);
			MP_WritePhyUshort(sc, 0x06, 0xe023);
			MP_WritePhyUshort(sc, 0x06, 0x58e0);
			MP_WritePhyUshort(sc, 0x06, 0x1e20);
			MP_WritePhyUshort(sc, 0x06, 0xe085);
			MP_WritePhyUshort(sc, 0x06, 0x001f);
			MP_WritePhyUshort(sc, 0x06, 0x029e);
			MP_WritePhyUshort(sc, 0x06, 0x22e6);
			MP_WritePhyUshort(sc, 0x06, 0x8500);
			MP_WritePhyUshort(sc, 0x06, 0xad32);
			MP_WritePhyUshort(sc, 0x06, 0x14ad);
			MP_WritePhyUshort(sc, 0x06, 0x3411);
			MP_WritePhyUshort(sc, 0x06, 0xef02);
			MP_WritePhyUshort(sc, 0x06, 0x5803);
			MP_WritePhyUshort(sc, 0x06, 0x9e07);
			MP_WritePhyUshort(sc, 0x06, 0xad35);
			MP_WritePhyUshort(sc, 0x06, 0x085a);
			MP_WritePhyUshort(sc, 0x06, 0xc09f);
			MP_WritePhyUshort(sc, 0x06, 0x04d1);
			MP_WritePhyUshort(sc, 0x06, 0x01ae);
			MP_WritePhyUshort(sc, 0x06, 0x02d1);
			MP_WritePhyUshort(sc, 0x06, 0x00bf);
			MP_WritePhyUshort(sc, 0x06, 0x81a5);
			MP_WritePhyUshort(sc, 0x06, 0x022f);
			MP_WritePhyUshort(sc, 0x06, 0x50ef);
			MP_WritePhyUshort(sc, 0x06, 0x96fe);
			MP_WritePhyUshort(sc, 0x06, 0xfdfc);
			MP_WritePhyUshort(sc, 0x06, 0x04a7);
			MP_WritePhyUshort(sc, 0x06, 0x25e5);
			MP_WritePhyUshort(sc, 0x06, 0x0a1d);
			MP_WritePhyUshort(sc, 0x06, 0xe50a);
			MP_WritePhyUshort(sc, 0x06, 0x2ce5);
			MP_WritePhyUshort(sc, 0x06, 0x0a6d);
			MP_WritePhyUshort(sc, 0x06, 0xe50a);
			MP_WritePhyUshort(sc, 0x06, 0x1de5);
			MP_WritePhyUshort(sc, 0x06, 0x0a1c);
			MP_WritePhyUshort(sc, 0x06, 0xe50a);
			MP_WritePhyUshort(sc, 0x06, 0x2da7);
			MP_WritePhyUshort(sc, 0x06, 0x5500);
			MP_WritePhyUshort(sc, 0x06, 0xe234);
			MP_WritePhyUshort(sc, 0x06, 0x88e2);
			MP_WritePhyUshort(sc, 0x06, 0x00cc);
			MP_WritePhyUshort(sc, 0x06, 0xe200);
			MP_WritePhyUshort(sc, 0x05, 0x8b86);
			MP_WritePhyUshort(sc, 0x06, 0x0001);
			Data = MP_ReadPhyUshort(sc, 0x01);
			Data |= 0x0001;
			MP_WritePhyUshort(sc, 0x01, Data);
			MP_WritePhyUshort(sc, 0x00, 0x0005);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x17, 0x2179);
			MP_WritePhyUshort(sc, 0x1f, 0x0001);
			MP_WritePhyUshort(sc, 0x10, 0xf274);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0042);
			MP_WritePhyUshort(sc, 0x15, 0x0f00);
			MP_WritePhyUshort(sc, 0x15, 0x0f00);
			MP_WritePhyUshort(sc, 0x16, 0x7408);
			MP_WritePhyUshort(sc, 0x15, 0x0e00);
			MP_WritePhyUshort(sc, 0x15, 0x0f00);
			MP_WritePhyUshort(sc, 0x15, 0x0f01);
			MP_WritePhyUshort(sc, 0x16, 0x4000);
			MP_WritePhyUshort(sc, 0x15, 0x0e01);
			MP_WritePhyUshort(sc, 0x15, 0x0f01);
			MP_WritePhyUshort(sc, 0x15, 0x0f02);
			MP_WritePhyUshort(sc, 0x16, 0x9400);
			MP_WritePhyUshort(sc, 0x15, 0x0e02);
			MP_WritePhyUshort(sc, 0x15, 0x0f02);
			MP_WritePhyUshort(sc, 0x15, 0x0f03);
			MP_WritePhyUshort(sc, 0x16, 0x7408);
			MP_WritePhyUshort(sc, 0x15, 0x0e03);
			MP_WritePhyUshort(sc, 0x15, 0x0f03);
			MP_WritePhyUshort(sc, 0x15, 0x0f04);
			MP_WritePhyUshort(sc, 0x16, 0x4008);
			MP_WritePhyUshort(sc, 0x15, 0x0e04);
			MP_WritePhyUshort(sc, 0x15, 0x0f04);
			MP_WritePhyUshort(sc, 0x15, 0x0f05);
			MP_WritePhyUshort(sc, 0x16, 0x9400);
			MP_WritePhyUshort(sc, 0x15, 0x0e05);
			MP_WritePhyUshort(sc, 0x15, 0x0f05);
			MP_WritePhyUshort(sc, 0x15, 0x0f06);
			MP_WritePhyUshort(sc, 0x16, 0x0803);
			MP_WritePhyUshort(sc, 0x15, 0x0e06);
			MP_WritePhyUshort(sc, 0x15, 0x0f06);
			MP_WritePhyUshort(sc, 0x15, 0x0d00);
			MP_WritePhyUshort(sc, 0x15, 0x0100);
			MP_WritePhyUshort(sc, 0x1f, 0x0001);
			MP_WritePhyUshort(sc, 0x10, 0xf074);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x17, 0x2149);
			MP_WritePhyUshort(sc, 0x1f, 0x0005);
			for (i=0; i<200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x00);
				if (Data&0x0080)
					break;
			}
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			MP_WritePhyUshort(sc, 0x17, 0x0116);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
			MP_WritePhyUshort(sc, 0x1f, 0x0007);
			MP_WritePhyUshort(sc, 0x1e, 0x0023);
			Data = MP_ReadPhyUshort(sc, 0x17);
			Data |= 0x4000;
			MP_WritePhyUshort(sc, 0x17, Data);
			MP_WritePhyUshort(sc, 0x1e, 0x0020);
			Data = MP_ReadPhyUshort(sc, 0x1b);
			Data |= 0x0080;
			MP_WritePhyUshort(sc, 0x1b, Data);
			MP_WritePhyUshort(sc, 0x1e, 0x0041);
			MP_WritePhyUshort(sc, 0x15, 0x0e02);
			MP_WritePhyUshort(sc, 0x1e, 0x0028);
			Data = MP_ReadPhyUshort(sc, 0x19);
			Data |= 0x8000;
			MP_WritePhyUshort(sc, 0x19, Data);
			MP_WritePhyUshort(sc, 0x1f, 0x0000);
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17) | 0x0006;
		MP_WritePhyUshort(sc, 0x17, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8b80);
		MP_WritePhyUshort(sc, 0x06, 0xc896);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x0B, 0x6C20);
		MP_WritePhyUshort(sc, 0x07, 0x2872);
		MP_WritePhyUshort(sc, 0x1C, 0xEFFF);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x14, 0x6420);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		Data = MP_ReadPhyUshort(sc, 0x08) & 0x00FF;
		MP_WritePhyUshort(sc, 0x08, Data | 0x8000);

		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x18);
		MP_WritePhyUshort(sc, 0x18, Data | 0x0050);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		Data = MP_ReadPhyUshort(sc, 0x14);
		MP_WritePhyUshort(sc, 0x14, Data | 0x8000);

		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x00, 0x080B);
		MP_WritePhyUshort(sc, 0x0B, 0x09D7);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x1006);

		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002F);
		MP_WritePhyUshort(sc, 0x15, 0x1919);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x19, 0x7F46);
		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8AD2);
		MP_WritePhyUshort(sc, 0x06, 0x6810);
		MP_WritePhyUshort(sc, 0x05, 0x8AD4);
		MP_WritePhyUshort(sc, 0x06, 0x8002);
		MP_WritePhyUshort(sc, 0x05, 0x8ADE);
		MP_WritePhyUshort(sc, 0x06, 0x8025);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_38) {
		CSR_WRITE_1(sc, 0x6E, CSR_READ_1(sc, 0x6E)| (1<<6));
		re_eri_write(sc, 0x1AE, 2, 0x0403, ERIAR_ExGMAC);

		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		Data= MP_ReadPhyUshort(sc, 0x15);
		Data &= ~BIT_12;
		MP_WritePhyUshort(sc, 0x15, Data);
		DELAY(200);
		DELAY(200);
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17);
		if ((Data & BIT_11) == 0x0000)
		{
			Data |= BIT_0;
 			MP_WritePhyUshort(sc, 0x17, Data);
			for (i = 0; i < 200; i++)
			{
				DELAY(100);
				Data = MP_ReadPhyUshort(sc, 0x17);
				if (Data & BIT_11)
					break;
			}
		}
		Data = MP_ReadPhyUshort(sc, 0x17);
		Data |= BIT_11;
		MP_WritePhyUshort(sc, 0x17,Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002C);
		MP_WritePhyUshort(sc, 0x1B, 0x5000);
		MP_WritePhyUshort(sc, 0x1E, 0x002d);
		MP_WritePhyUshort(sc, 0x19, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data= MP_ReadPhyUshort(sc, 0x1E);
			if ((Data& 0x03FF) == 0x0014)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data= MP_ReadPhyUshort(sc, 0x07);
			if ((Data& BIT_5) == 0)
				break;
		}
		Data = MP_ReadPhyUshort(sc, 0x07);
		if (Data & BIT_5)
		{
	 		MP_WritePhyUshort(sc, 0x1f, 0x0004);
	 		MP_WritePhyUshort(sc, 0x1f, 0x0007);
	 		MP_WritePhyUshort(sc, 0x1e, 0x00a1);
	 		MP_WritePhyUshort(sc, 0x17, 0x1000);
	 		MP_WritePhyUshort(sc, 0x17, 0x0000);
	 		MP_WritePhyUshort(sc, 0x17, 0x2000);
	 		MP_WritePhyUshort(sc, 0x1e, 0x002f);
	 		MP_WritePhyUshort(sc, 0x18, 0x9bfb);
	 		MP_WritePhyUshort(sc, 0x1f, 0x0005);
	 		MP_WritePhyUshort(sc, 0x07, 0x0000);
	 		MP_WritePhyUshort(sc, 0x1f, 0x0002);
	 		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		Data = MP_ReadPhyUshort(sc, 0x00);
		Data &= ~BIT_7;
		MP_WritePhyUshort(sc, 0x00, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0307);
		MP_WritePhyUshort(sc, 0x15, 0x0000);
		MP_WritePhyUshort(sc, 0x19, 0x407d);
		MP_WritePhyUshort(sc, 0x15, 0x0001);
		MP_WritePhyUshort(sc, 0x19, 0x440f);
		MP_WritePhyUshort(sc, 0x15, 0x0002);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0003);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x0004);
		MP_WritePhyUshort(sc, 0x19, 0xc4d5);
		MP_WritePhyUshort(sc, 0x15, 0x0005);
		MP_WritePhyUshort(sc, 0x19, 0x00ff);
		MP_WritePhyUshort(sc, 0x15, 0x0006);
		MP_WritePhyUshort(sc, 0x19, 0x74f0);
		MP_WritePhyUshort(sc, 0x15, 0x0007);
		MP_WritePhyUshort(sc, 0x19, 0x4880);
		MP_WritePhyUshort(sc, 0x15, 0x0008);
		MP_WritePhyUshort(sc, 0x19, 0x4c00);
		MP_WritePhyUshort(sc, 0x15, 0x0009);
		MP_WritePhyUshort(sc, 0x19, 0x4800);
		MP_WritePhyUshort(sc, 0x15, 0x000a);
		MP_WritePhyUshort(sc, 0x19, 0x5000);
		MP_WritePhyUshort(sc, 0x15, 0x000b);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x000c);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x000d);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x15, 0x000e);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x000f);
		MP_WritePhyUshort(sc, 0x19, 0x7010);
		MP_WritePhyUshort(sc, 0x15, 0x0010);
		MP_WritePhyUshort(sc, 0x19, 0x6804);
		MP_WritePhyUshort(sc, 0x15, 0x0011);
		MP_WritePhyUshort(sc, 0x19, 0x64a0);
		MP_WritePhyUshort(sc, 0x15, 0x0012);
		MP_WritePhyUshort(sc, 0x19, 0x63da);
		MP_WritePhyUshort(sc, 0x15, 0x0013);
		MP_WritePhyUshort(sc, 0x19, 0x63d8);
		MP_WritePhyUshort(sc, 0x15, 0x0014);
		MP_WritePhyUshort(sc, 0x19, 0x6f05);
		MP_WritePhyUshort(sc, 0x15, 0x0015);
		MP_WritePhyUshort(sc, 0x19, 0x5420);
		MP_WritePhyUshort(sc, 0x15, 0x0016);
		MP_WritePhyUshort(sc, 0x19, 0x58ce);
		MP_WritePhyUshort(sc, 0x15, 0x0017);
		MP_WritePhyUshort(sc, 0x19, 0x5cf3);
		MP_WritePhyUshort(sc, 0x15, 0x0018);
		MP_WritePhyUshort(sc, 0x19, 0xb600);
		MP_WritePhyUshort(sc, 0x15, 0x0019);
		MP_WritePhyUshort(sc, 0x19, 0xc659);
		MP_WritePhyUshort(sc, 0x15, 0x001a);
		MP_WritePhyUshort(sc, 0x19, 0x0018);
		MP_WritePhyUshort(sc, 0x15, 0x001b);
		MP_WritePhyUshort(sc, 0x19, 0xc403);
		MP_WritePhyUshort(sc, 0x15, 0x001c);
		MP_WritePhyUshort(sc, 0x19, 0x0016);
		MP_WritePhyUshort(sc, 0x15, 0x001d);
		MP_WritePhyUshort(sc, 0x19, 0xaa05);
		MP_WritePhyUshort(sc, 0x15, 0x001e);
		MP_WritePhyUshort(sc, 0x19, 0xc503);
		MP_WritePhyUshort(sc, 0x15, 0x001f);
		MP_WritePhyUshort(sc, 0x19, 0x0003);
		MP_WritePhyUshort(sc, 0x15, 0x0020);
		MP_WritePhyUshort(sc, 0x19, 0x89f8);
		MP_WritePhyUshort(sc, 0x15, 0x0021);
		MP_WritePhyUshort(sc, 0x19, 0x32ae);
		MP_WritePhyUshort(sc, 0x15, 0x0022);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0023);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x0024);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0025);
		MP_WritePhyUshort(sc, 0x19, 0x6801);
		MP_WritePhyUshort(sc, 0x15, 0x0026);
		MP_WritePhyUshort(sc, 0x19, 0x66a0);
		MP_WritePhyUshort(sc, 0x15, 0x0027);
		MP_WritePhyUshort(sc, 0x19, 0xa300);
		MP_WritePhyUshort(sc, 0x15, 0x0028);
		MP_WritePhyUshort(sc, 0x19, 0x64a0);
		MP_WritePhyUshort(sc, 0x15, 0x0029);
		MP_WritePhyUshort(sc, 0x19, 0x76f0);
		MP_WritePhyUshort(sc, 0x15, 0x002a);
		MP_WritePhyUshort(sc, 0x19, 0x7670);
		MP_WritePhyUshort(sc, 0x15, 0x002b);
		MP_WritePhyUshort(sc, 0x19, 0x7630);
		MP_WritePhyUshort(sc, 0x15, 0x002c);
		MP_WritePhyUshort(sc, 0x19, 0x31a6);
		MP_WritePhyUshort(sc, 0x15, 0x002d);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x002e);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x002f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0030);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0031);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0032);
		MP_WritePhyUshort(sc, 0x19, 0x4801);
		MP_WritePhyUshort(sc, 0x15, 0x0033);
		MP_WritePhyUshort(sc, 0x19, 0x6803);
		MP_WritePhyUshort(sc, 0x15, 0x0034);
		MP_WritePhyUshort(sc, 0x19, 0x66a1);
		MP_WritePhyUshort(sc, 0x15, 0x0035);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0036);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x0037);
		MP_WritePhyUshort(sc, 0x19, 0xa300);
		MP_WritePhyUshort(sc, 0x15, 0x0038);
		MP_WritePhyUshort(sc, 0x19, 0x64a1);
		MP_WritePhyUshort(sc, 0x15, 0x0039);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x003a);
		MP_WritePhyUshort(sc, 0x19, 0x74f8);
		MP_WritePhyUshort(sc, 0x15, 0x003b);
		MP_WritePhyUshort(sc, 0x19, 0x63d0);
		MP_WritePhyUshort(sc, 0x15, 0x003c);
		MP_WritePhyUshort(sc, 0x19, 0x7ff0);
		MP_WritePhyUshort(sc, 0x15, 0x003d);
		MP_WritePhyUshort(sc, 0x19, 0x77f0);
		MP_WritePhyUshort(sc, 0x15, 0x003e);
		MP_WritePhyUshort(sc, 0x19, 0x7ff0);
		MP_WritePhyUshort(sc, 0x15, 0x003f);
		MP_WritePhyUshort(sc, 0x19, 0x7750);
		MP_WritePhyUshort(sc, 0x15, 0x0040);
		MP_WritePhyUshort(sc, 0x19, 0x63d8);
		MP_WritePhyUshort(sc, 0x15, 0x0041);
		MP_WritePhyUshort(sc, 0x19, 0x7cf0);
		MP_WritePhyUshort(sc, 0x15, 0x0042);
		MP_WritePhyUshort(sc, 0x19, 0x7708);
		MP_WritePhyUshort(sc, 0x15, 0x0043);
		MP_WritePhyUshort(sc, 0x19, 0xa654);
		MP_WritePhyUshort(sc, 0x15, 0x0044);
		MP_WritePhyUshort(sc, 0x19, 0x304a);
		MP_WritePhyUshort(sc, 0x15, 0x0045);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0046);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0047);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0048);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0049);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x004a);
		MP_WritePhyUshort(sc, 0x19, 0x4802);
		MP_WritePhyUshort(sc, 0x15, 0x004b);
		MP_WritePhyUshort(sc, 0x19, 0x4003);
		MP_WritePhyUshort(sc, 0x15, 0x004c);
		MP_WritePhyUshort(sc, 0x19, 0x4440);
		MP_WritePhyUshort(sc, 0x15, 0x004d);
		MP_WritePhyUshort(sc, 0x19, 0x63c8);
		MP_WritePhyUshort(sc, 0x15, 0x004e);
		MP_WritePhyUshort(sc, 0x19, 0x6481);
		MP_WritePhyUshort(sc, 0x15, 0x004f);
		MP_WritePhyUshort(sc, 0x19, 0x9d00);
		MP_WritePhyUshort(sc, 0x15, 0x0050);
		MP_WritePhyUshort(sc, 0x19, 0x63e8);
		MP_WritePhyUshort(sc, 0x15, 0x0051);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x15, 0x0052);
		MP_WritePhyUshort(sc, 0x19, 0x5900);
		MP_WritePhyUshort(sc, 0x15, 0x0053);
		MP_WritePhyUshort(sc, 0x19, 0x63f8);
		MP_WritePhyUshort(sc, 0x15, 0x0054);
		MP_WritePhyUshort(sc, 0x19, 0x64a1);
		MP_WritePhyUshort(sc, 0x15, 0x0055);
		MP_WritePhyUshort(sc, 0x19, 0x3116);
		MP_WritePhyUshort(sc, 0x15, 0x0056);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0057);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0058);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0059);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x005a);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x005b);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x005c);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x005d);
		MP_WritePhyUshort(sc, 0x19, 0x6000);
		MP_WritePhyUshort(sc, 0x15, 0x005e);
		MP_WritePhyUshort(sc, 0x19, 0x59ce);
		MP_WritePhyUshort(sc, 0x15, 0x005f);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x0060);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x15, 0x0061);
		MP_WritePhyUshort(sc, 0x19, 0x72b0);
		MP_WritePhyUshort(sc, 0x15, 0x0062);
		MP_WritePhyUshort(sc, 0x19, 0x400e);
		MP_WritePhyUshort(sc, 0x15, 0x0063);
		MP_WritePhyUshort(sc, 0x19, 0x4440);
		MP_WritePhyUshort(sc, 0x15, 0x0064);
		MP_WritePhyUshort(sc, 0x19, 0x9d00);
		MP_WritePhyUshort(sc, 0x15, 0x0065);
		MP_WritePhyUshort(sc, 0x19, 0x7f00);
		MP_WritePhyUshort(sc, 0x15, 0x0066);
		MP_WritePhyUshort(sc, 0x19, 0x70b0);
		MP_WritePhyUshort(sc, 0x15, 0x0067);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x0068);
		MP_WritePhyUshort(sc, 0x19, 0x6008);
		MP_WritePhyUshort(sc, 0x15, 0x0069);
		MP_WritePhyUshort(sc, 0x19, 0x7cf0);
		MP_WritePhyUshort(sc, 0x15, 0x006a);
		MP_WritePhyUshort(sc, 0x19, 0x7750);
		MP_WritePhyUshort(sc, 0x15, 0x006b);
		MP_WritePhyUshort(sc, 0x19, 0x4007);
		MP_WritePhyUshort(sc, 0x15, 0x006c);
		MP_WritePhyUshort(sc, 0x19, 0x4500);
		MP_WritePhyUshort(sc, 0x15, 0x006d);
		MP_WritePhyUshort(sc, 0x19, 0x4023);
		MP_WritePhyUshort(sc, 0x15, 0x006e);
		MP_WritePhyUshort(sc, 0x19, 0x4580);
		MP_WritePhyUshort(sc, 0x15, 0x006f);
		MP_WritePhyUshort(sc, 0x19, 0x9f00);
		MP_WritePhyUshort(sc, 0x15, 0x0070);
		MP_WritePhyUshort(sc, 0x19, 0xcd78);
		MP_WritePhyUshort(sc, 0x15, 0x0071);
		MP_WritePhyUshort(sc, 0x19, 0x0003);
		MP_WritePhyUshort(sc, 0x15, 0x0072);
		MP_WritePhyUshort(sc, 0x19, 0xbe02);
		MP_WritePhyUshort(sc, 0x15, 0x0073);
		MP_WritePhyUshort(sc, 0x19, 0x3070);
		MP_WritePhyUshort(sc, 0x15, 0x0074);
		MP_WritePhyUshort(sc, 0x19, 0x7cf0);
		MP_WritePhyUshort(sc, 0x15, 0x0075);
		MP_WritePhyUshort(sc, 0x19, 0x77f0);
		MP_WritePhyUshort(sc, 0x15, 0x0076);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x0077);
		MP_WritePhyUshort(sc, 0x19, 0x4007);
		MP_WritePhyUshort(sc, 0x15, 0x0078);
		MP_WritePhyUshort(sc, 0x19, 0x4500);
		MP_WritePhyUshort(sc, 0x15, 0x0079);
		MP_WritePhyUshort(sc, 0x19, 0x4023);
		MP_WritePhyUshort(sc, 0x15, 0x007a);
		MP_WritePhyUshort(sc, 0x19, 0x4580);
		MP_WritePhyUshort(sc, 0x15, 0x007b);
		MP_WritePhyUshort(sc, 0x19, 0x9f00);
		MP_WritePhyUshort(sc, 0x15, 0x007c);
		MP_WritePhyUshort(sc, 0x19, 0xce80);
		MP_WritePhyUshort(sc, 0x15, 0x007d);
		MP_WritePhyUshort(sc, 0x19, 0x0004);
		MP_WritePhyUshort(sc, 0x15, 0x007e);
		MP_WritePhyUshort(sc, 0x19, 0xce80);
		MP_WritePhyUshort(sc, 0x15, 0x007f);
		MP_WritePhyUshort(sc, 0x19, 0x0002);
		MP_WritePhyUshort(sc, 0x15, 0x0080);
		MP_WritePhyUshort(sc, 0x19, 0x307c);
		MP_WritePhyUshort(sc, 0x15, 0x0081);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x0082);
		MP_WritePhyUshort(sc, 0x19, 0x480f);
		MP_WritePhyUshort(sc, 0x15, 0x0083);
		MP_WritePhyUshort(sc, 0x19, 0x6802);
		MP_WritePhyUshort(sc, 0x15, 0x0084);
		MP_WritePhyUshort(sc, 0x19, 0x6680);
		MP_WritePhyUshort(sc, 0x15, 0x0085);
		MP_WritePhyUshort(sc, 0x19, 0x7c10);
		MP_WritePhyUshort(sc, 0x15, 0x0086);
		MP_WritePhyUshort(sc, 0x19, 0x6010);
		MP_WritePhyUshort(sc, 0x15, 0x0087);
		MP_WritePhyUshort(sc, 0x19, 0x400a);
		MP_WritePhyUshort(sc, 0x15, 0x0088);
		MP_WritePhyUshort(sc, 0x19, 0x4580);
		MP_WritePhyUshort(sc, 0x15, 0x0089);
		MP_WritePhyUshort(sc, 0x19, 0x9e00);
		MP_WritePhyUshort(sc, 0x15, 0x008a);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x15, 0x008b);
		MP_WritePhyUshort(sc, 0x19, 0x5800);
		MP_WritePhyUshort(sc, 0x15, 0x008c);
		MP_WritePhyUshort(sc, 0x19, 0x63c8);
		MP_WritePhyUshort(sc, 0x15, 0x008d);
		MP_WritePhyUshort(sc, 0x19, 0x63d8);
		MP_WritePhyUshort(sc, 0x15, 0x008e);
		MP_WritePhyUshort(sc, 0x19, 0x66a0);
		MP_WritePhyUshort(sc, 0x15, 0x008f);
		MP_WritePhyUshort(sc, 0x19, 0x8300);
		MP_WritePhyUshort(sc, 0x15, 0x0090);
		MP_WritePhyUshort(sc, 0x19, 0x7ff0);
		MP_WritePhyUshort(sc, 0x15, 0x0091);
		MP_WritePhyUshort(sc, 0x19, 0x74f0);
		MP_WritePhyUshort(sc, 0x15, 0x0092);
		MP_WritePhyUshort(sc, 0x19, 0x3006);
		MP_WritePhyUshort(sc, 0x15, 0x0093);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0094);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0095);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0096);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0097);
		MP_WritePhyUshort(sc, 0x19, 0x4803);
		MP_WritePhyUshort(sc, 0x15, 0x0098);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0099);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x009a);
		MP_WritePhyUshort(sc, 0x19, 0xa203);
		MP_WritePhyUshort(sc, 0x15, 0x009b);
		MP_WritePhyUshort(sc, 0x19, 0x64b1);
		MP_WritePhyUshort(sc, 0x15, 0x009c);
		MP_WritePhyUshort(sc, 0x19, 0x309e);
		MP_WritePhyUshort(sc, 0x15, 0x009d);
		MP_WritePhyUshort(sc, 0x19, 0x64b3);
		MP_WritePhyUshort(sc, 0x15, 0x009e);
		MP_WritePhyUshort(sc, 0x19, 0x4030);
		MP_WritePhyUshort(sc, 0x15, 0x009f);
		MP_WritePhyUshort(sc, 0x19, 0x440e);
		MP_WritePhyUshort(sc, 0x15, 0x00a0);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x00a1);
		MP_WritePhyUshort(sc, 0x19, 0x4419);
		MP_WritePhyUshort(sc, 0x15, 0x00a2);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x00a3);
		MP_WritePhyUshort(sc, 0x19, 0xc520);
		MP_WritePhyUshort(sc, 0x15, 0x00a4);
		MP_WritePhyUshort(sc, 0x19, 0x000b);
		MP_WritePhyUshort(sc, 0x15, 0x00a5);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x00a6);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x00a7);
		MP_WritePhyUshort(sc, 0x19, 0x58a4);
		MP_WritePhyUshort(sc, 0x15, 0x00a8);
		MP_WritePhyUshort(sc, 0x19, 0x63da);
		MP_WritePhyUshort(sc, 0x15, 0x00a9);
		MP_WritePhyUshort(sc, 0x19, 0x5cb0);
		MP_WritePhyUshort(sc, 0x15, 0x00aa);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x15, 0x00ab);
		MP_WritePhyUshort(sc, 0x19, 0x72b0);
		MP_WritePhyUshort(sc, 0x15, 0x00ac);
		MP_WritePhyUshort(sc, 0x19, 0x7f00);
		MP_WritePhyUshort(sc, 0x15, 0x00ad);
		MP_WritePhyUshort(sc, 0x19, 0x70b0);
		MP_WritePhyUshort(sc, 0x15, 0x00ae);
		MP_WritePhyUshort(sc, 0x19, 0x30b8);
		MP_WritePhyUshort(sc, 0x15, 0x00AF);
		MP_WritePhyUshort(sc, 0x19, 0x4060);
		MP_WritePhyUshort(sc, 0x15, 0x00B0);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x00B1);
		MP_WritePhyUshort(sc, 0x19, 0x7e00);
		MP_WritePhyUshort(sc, 0x15, 0x00B2);
		MP_WritePhyUshort(sc, 0x19, 0x72B0);
		MP_WritePhyUshort(sc, 0x15, 0x00B3);
		MP_WritePhyUshort(sc, 0x19, 0x7F00);
		MP_WritePhyUshort(sc, 0x15, 0x00B4);
		MP_WritePhyUshort(sc, 0x19, 0x73B0);
		MP_WritePhyUshort(sc, 0x15, 0x00b5);
		MP_WritePhyUshort(sc, 0x19, 0x58a0);
		MP_WritePhyUshort(sc, 0x15, 0x00b6);
		MP_WritePhyUshort(sc, 0x19, 0x63d2);
		MP_WritePhyUshort(sc, 0x15, 0x00b7);
		MP_WritePhyUshort(sc, 0x19, 0x5c00);
		MP_WritePhyUshort(sc, 0x15, 0x00b8);
		MP_WritePhyUshort(sc, 0x19, 0x5780);
		MP_WritePhyUshort(sc, 0x15, 0x00b9);
		MP_WritePhyUshort(sc, 0x19, 0xb60d);
		MP_WritePhyUshort(sc, 0x15, 0x00ba);
		MP_WritePhyUshort(sc, 0x19, 0x9bff);
		MP_WritePhyUshort(sc, 0x15, 0x00bb);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x00bc);
		MP_WritePhyUshort(sc, 0x19, 0x6001);
		MP_WritePhyUshort(sc, 0x15, 0x00bd);
		MP_WritePhyUshort(sc, 0x19, 0xc020);
		MP_WritePhyUshort(sc, 0x15, 0x00be);
		MP_WritePhyUshort(sc, 0x19, 0x002b);
		MP_WritePhyUshort(sc, 0x15, 0x00bf);
		MP_WritePhyUshort(sc, 0x19, 0xc137);
		MP_WritePhyUshort(sc, 0x15, 0x00c0);
		MP_WritePhyUshort(sc, 0x19, 0x0006);
		MP_WritePhyUshort(sc, 0x15, 0x00c1);
		MP_WritePhyUshort(sc, 0x19, 0x9af8);
		MP_WritePhyUshort(sc, 0x15, 0x00c2);
		MP_WritePhyUshort(sc, 0x19, 0x30c6);
		MP_WritePhyUshort(sc, 0x15, 0x00c3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00c4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00c5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00c6);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x15, 0x00c7);
		MP_WritePhyUshort(sc, 0x19, 0x70b0);
		MP_WritePhyUshort(sc, 0x15, 0x00c8);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x00c9);
		MP_WritePhyUshort(sc, 0x19, 0x4804);
		MP_WritePhyUshort(sc, 0x15, 0x00ca);
		MP_WritePhyUshort(sc, 0x19, 0x7c80);
		MP_WritePhyUshort(sc, 0x15, 0x00cb);
		MP_WritePhyUshort(sc, 0x19, 0x5c80);
		MP_WritePhyUshort(sc, 0x15, 0x00cc);
		MP_WritePhyUshort(sc, 0x19, 0x4010);
		MP_WritePhyUshort(sc, 0x15, 0x00cd);
		MP_WritePhyUshort(sc, 0x19, 0x4415);
		MP_WritePhyUshort(sc, 0x15, 0x00ce);
		MP_WritePhyUshort(sc, 0x19, 0x9b00);
		MP_WritePhyUshort(sc, 0x15, 0x00cf);
		MP_WritePhyUshort(sc, 0x19, 0x7f00);
		MP_WritePhyUshort(sc, 0x15, 0x00d0);
		MP_WritePhyUshort(sc, 0x19, 0x70b0);
		MP_WritePhyUshort(sc, 0x15, 0x00d1);
		MP_WritePhyUshort(sc, 0x19, 0x3177);
		MP_WritePhyUshort(sc, 0x15, 0x00d2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00d3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00d4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00d5);
		MP_WritePhyUshort(sc, 0x19, 0x4808);
		MP_WritePhyUshort(sc, 0x15, 0x00d6);
		MP_WritePhyUshort(sc, 0x19, 0x4007);
		MP_WritePhyUshort(sc, 0x15, 0x00d7);
		MP_WritePhyUshort(sc, 0x19, 0x4420);
		MP_WritePhyUshort(sc, 0x15, 0x00d8);
		MP_WritePhyUshort(sc, 0x19, 0x63d8);
		MP_WritePhyUshort(sc, 0x15, 0x00d9);
		MP_WritePhyUshort(sc, 0x19, 0xb608);
		MP_WritePhyUshort(sc, 0x15, 0x00da);
		MP_WritePhyUshort(sc, 0x19, 0xbcbd);
		MP_WritePhyUshort(sc, 0x15, 0x00db);
		MP_WritePhyUshort(sc, 0x19, 0xc60b);
		MP_WritePhyUshort(sc, 0x15, 0x00dc);
		MP_WritePhyUshort(sc, 0x19, 0x00fd);
		MP_WritePhyUshort(sc, 0x15, 0x00dd);
		MP_WritePhyUshort(sc, 0x19, 0x30e1);
		MP_WritePhyUshort(sc, 0x15, 0x00de);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00df);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00e0);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00e1);
		MP_WritePhyUshort(sc, 0x19, 0x4809);
		MP_WritePhyUshort(sc, 0x15, 0x00e2);
		MP_WritePhyUshort(sc, 0x19, 0x7e40);
		MP_WritePhyUshort(sc, 0x15, 0x00e3);
		MP_WritePhyUshort(sc, 0x19, 0x5a40);
		MP_WritePhyUshort(sc, 0x15, 0x00e4);
		MP_WritePhyUshort(sc, 0x19, 0x305a);
		MP_WritePhyUshort(sc, 0x15, 0x00e5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00e6);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00e7);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00e8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00e9);
		MP_WritePhyUshort(sc, 0x19, 0x480a);
		MP_WritePhyUshort(sc, 0x15, 0x00ea);
		MP_WritePhyUshort(sc, 0x19, 0x5820);
		MP_WritePhyUshort(sc, 0x15, 0x00eb);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x00ec);
		MP_WritePhyUshort(sc, 0x19, 0xb60a);
		MP_WritePhyUshort(sc, 0x15, 0x00ed);
		MP_WritePhyUshort(sc, 0x19, 0xda07);
		MP_WritePhyUshort(sc, 0x15, 0x00ee);
		MP_WritePhyUshort(sc, 0x19, 0x0008);
		MP_WritePhyUshort(sc, 0x15, 0x00ef);
		MP_WritePhyUshort(sc, 0x19, 0xc60b);
		MP_WritePhyUshort(sc, 0x15, 0x00f0);
		MP_WritePhyUshort(sc, 0x19, 0x00fc);
		MP_WritePhyUshort(sc, 0x15, 0x00f1);
		MP_WritePhyUshort(sc, 0x19, 0x30f6);
		MP_WritePhyUshort(sc, 0x15, 0x00f2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00f3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00f4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00f5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x00f6);
		MP_WritePhyUshort(sc, 0x19, 0x4408);
		MP_WritePhyUshort(sc, 0x15, 0x00f7);
		MP_WritePhyUshort(sc, 0x19, 0x480b);
		MP_WritePhyUshort(sc, 0x15, 0x00f8);
		MP_WritePhyUshort(sc, 0x19, 0x6f03);
		MP_WritePhyUshort(sc, 0x15, 0x00f9);
		MP_WritePhyUshort(sc, 0x19, 0x405f);
		MP_WritePhyUshort(sc, 0x15, 0x00fa);
		MP_WritePhyUshort(sc, 0x19, 0x4448);
		MP_WritePhyUshort(sc, 0x15, 0x00fb);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x00fc);
		MP_WritePhyUshort(sc, 0x19, 0x4468);
		MP_WritePhyUshort(sc, 0x15, 0x00fd);
		MP_WritePhyUshort(sc, 0x19, 0x9c03);
		MP_WritePhyUshort(sc, 0x15, 0x00fe);
		MP_WritePhyUshort(sc, 0x19, 0x6f07);
		MP_WritePhyUshort(sc, 0x15, 0x00ff);
		MP_WritePhyUshort(sc, 0x19, 0x58a0);
		MP_WritePhyUshort(sc, 0x15, 0x0100);
		MP_WritePhyUshort(sc, 0x19, 0xd6d1);
		MP_WritePhyUshort(sc, 0x15, 0x0101);
		MP_WritePhyUshort(sc, 0x19, 0x0004);
		MP_WritePhyUshort(sc, 0x15, 0x0102);
		MP_WritePhyUshort(sc, 0x19, 0xc137);
		MP_WritePhyUshort(sc, 0x15, 0x0103);
		MP_WritePhyUshort(sc, 0x19, 0x0002);
		MP_WritePhyUshort(sc, 0x15, 0x0104);
		MP_WritePhyUshort(sc, 0x19, 0xa0e5);
		MP_WritePhyUshort(sc, 0x15, 0x0105);
		MP_WritePhyUshort(sc, 0x19, 0x9df8);
		MP_WritePhyUshort(sc, 0x15, 0x0106);
		MP_WritePhyUshort(sc, 0x19, 0x30c6);
		MP_WritePhyUshort(sc, 0x15, 0x0107);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0108);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0109);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x010a);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x010b);
		MP_WritePhyUshort(sc, 0x19, 0x4808);
		MP_WritePhyUshort(sc, 0x15, 0x010c);
		MP_WritePhyUshort(sc, 0x19, 0xc32d);
		MP_WritePhyUshort(sc, 0x15, 0x010d);
		MP_WritePhyUshort(sc, 0x19, 0x0003);
		MP_WritePhyUshort(sc, 0x15, 0x010e);
		MP_WritePhyUshort(sc, 0x19, 0xc8b3);
		MP_WritePhyUshort(sc, 0x15, 0x010f);
		MP_WritePhyUshort(sc, 0x19, 0x00fc);
		MP_WritePhyUshort(sc, 0x15, 0x0110);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x0111);
		MP_WritePhyUshort(sc, 0x19, 0x3116);
		MP_WritePhyUshort(sc, 0x15, 0x0112);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0113);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0114);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0115);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0116);
		MP_WritePhyUshort(sc, 0x19, 0x4803);
		MP_WritePhyUshort(sc, 0x15, 0x0117);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0118);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x0119);
		MP_WritePhyUshort(sc, 0x19, 0x7c04);
		MP_WritePhyUshort(sc, 0x15, 0x011a);
		MP_WritePhyUshort(sc, 0x19, 0x6000);
		MP_WritePhyUshort(sc, 0x15, 0x011b);
		MP_WritePhyUshort(sc, 0x19, 0x5cf7);
		MP_WritePhyUshort(sc, 0x15, 0x011c);
		MP_WritePhyUshort(sc, 0x19, 0x7c2a);
		MP_WritePhyUshort(sc, 0x15, 0x011d);
		MP_WritePhyUshort(sc, 0x19, 0x5800);
		MP_WritePhyUshort(sc, 0x15, 0x011e);
		MP_WritePhyUshort(sc, 0x19, 0x5400);
		MP_WritePhyUshort(sc, 0x15, 0x011f);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x0120);
		MP_WritePhyUshort(sc, 0x19, 0x74f0);
		MP_WritePhyUshort(sc, 0x15, 0x0121);
		MP_WritePhyUshort(sc, 0x19, 0x4019);
		MP_WritePhyUshort(sc, 0x15, 0x0122);
		MP_WritePhyUshort(sc, 0x19, 0x440d);
		MP_WritePhyUshort(sc, 0x15, 0x0123);
		MP_WritePhyUshort(sc, 0x19, 0xb6c1);
		MP_WritePhyUshort(sc, 0x15, 0x0124);
		MP_WritePhyUshort(sc, 0x19, 0xc05b);
		MP_WritePhyUshort(sc, 0x15, 0x0125);
		MP_WritePhyUshort(sc, 0x19, 0x00bf);
		MP_WritePhyUshort(sc, 0x15, 0x0126);
		MP_WritePhyUshort(sc, 0x19, 0xc025);
		MP_WritePhyUshort(sc, 0x15, 0x0127);
		MP_WritePhyUshort(sc, 0x19, 0x00bd);
		MP_WritePhyUshort(sc, 0x15, 0x0128);
		MP_WritePhyUshort(sc, 0x19, 0xc603);
		MP_WritePhyUshort(sc, 0x15, 0x0129);
		MP_WritePhyUshort(sc, 0x19, 0x00bb);
		MP_WritePhyUshort(sc, 0x15, 0x012a);
		MP_WritePhyUshort(sc, 0x19, 0x8805);
		MP_WritePhyUshort(sc, 0x15, 0x012b);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x012c);
		MP_WritePhyUshort(sc, 0x19, 0x4001);
		MP_WritePhyUshort(sc, 0x15, 0x012d);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x012e);
		MP_WritePhyUshort(sc, 0x19, 0xa3dd);
		MP_WritePhyUshort(sc, 0x15, 0x012f);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0130);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x0131);
		MP_WritePhyUshort(sc, 0x19, 0x8407);
		MP_WritePhyUshort(sc, 0x15, 0x0132);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0133);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x0134);
		MP_WritePhyUshort(sc, 0x19, 0xd9b8);
		MP_WritePhyUshort(sc, 0x15, 0x0135);
		MP_WritePhyUshort(sc, 0x19, 0x0003);
		MP_WritePhyUshort(sc, 0x15, 0x0136);
		MP_WritePhyUshort(sc, 0x19, 0xc240);
		MP_WritePhyUshort(sc, 0x15, 0x0137);
		MP_WritePhyUshort(sc, 0x19, 0x0015);
		MP_WritePhyUshort(sc, 0x15, 0x0138);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0139);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x013a);
		MP_WritePhyUshort(sc, 0x19, 0x9ae9);
		MP_WritePhyUshort(sc, 0x15, 0x013b);
		MP_WritePhyUshort(sc, 0x19, 0x3140);
		MP_WritePhyUshort(sc, 0x15, 0x013c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x013d);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x013e);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x013f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0140);
		MP_WritePhyUshort(sc, 0x19, 0x4807);
		MP_WritePhyUshort(sc, 0x15, 0x0141);
		MP_WritePhyUshort(sc, 0x19, 0x4004);
		MP_WritePhyUshort(sc, 0x15, 0x0142);
		MP_WritePhyUshort(sc, 0x19, 0x4410);
		MP_WritePhyUshort(sc, 0x15, 0x0143);
		MP_WritePhyUshort(sc, 0x19, 0x7c0c);
		MP_WritePhyUshort(sc, 0x15, 0x0144);
		MP_WritePhyUshort(sc, 0x19, 0x600c);
		MP_WritePhyUshort(sc, 0x15, 0x0145);
		MP_WritePhyUshort(sc, 0x19, 0x9b00);
		MP_WritePhyUshort(sc, 0x15, 0x0146);
		MP_WritePhyUshort(sc, 0x19, 0xa68f);
		MP_WritePhyUshort(sc, 0x15, 0x0147);
		MP_WritePhyUshort(sc, 0x19, 0x3116);
		MP_WritePhyUshort(sc, 0x15, 0x0148);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0149);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x014a);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x014b);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x014c);
		MP_WritePhyUshort(sc, 0x19, 0x4804);
		MP_WritePhyUshort(sc, 0x15, 0x014d);
		MP_WritePhyUshort(sc, 0x19, 0x54c0);
		MP_WritePhyUshort(sc, 0x15, 0x014e);
		MP_WritePhyUshort(sc, 0x19, 0xb703);
		MP_WritePhyUshort(sc, 0x15, 0x014f);
		MP_WritePhyUshort(sc, 0x19, 0x5cff);
		MP_WritePhyUshort(sc, 0x15, 0x0150);
		MP_WritePhyUshort(sc, 0x19, 0x315f);
		MP_WritePhyUshort(sc, 0x15, 0x0151);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x0152);
		MP_WritePhyUshort(sc, 0x19, 0x74f8);
		MP_WritePhyUshort(sc, 0x15, 0x0153);
		MP_WritePhyUshort(sc, 0x19, 0x6421);
		MP_WritePhyUshort(sc, 0x15, 0x0154);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x0155);
		MP_WritePhyUshort(sc, 0x19, 0x6000);
		MP_WritePhyUshort(sc, 0x15, 0x0156);
		MP_WritePhyUshort(sc, 0x19, 0x4003);
		MP_WritePhyUshort(sc, 0x15, 0x0157);
		MP_WritePhyUshort(sc, 0x19, 0x4418);
		MP_WritePhyUshort(sc, 0x15, 0x0158);
		MP_WritePhyUshort(sc, 0x19, 0x9b00);
		MP_WritePhyUshort(sc, 0x15, 0x0159);
		MP_WritePhyUshort(sc, 0x19, 0x6461);
		MP_WritePhyUshort(sc, 0x15, 0x015a);
		MP_WritePhyUshort(sc, 0x19, 0x64e1);
		MP_WritePhyUshort(sc, 0x15, 0x015b);
		MP_WritePhyUshort(sc, 0x19, 0x7c20);
		MP_WritePhyUshort(sc, 0x15, 0x015c);
		MP_WritePhyUshort(sc, 0x19, 0x5820);
		MP_WritePhyUshort(sc, 0x15, 0x015d);
		MP_WritePhyUshort(sc, 0x19, 0x5ccf);
		MP_WritePhyUshort(sc, 0x15, 0x015e);
		MP_WritePhyUshort(sc, 0x19, 0x7050);
		MP_WritePhyUshort(sc, 0x15, 0x015f);
		MP_WritePhyUshort(sc, 0x19, 0xd9b8);
		MP_WritePhyUshort(sc, 0x15, 0x0160);
		MP_WritePhyUshort(sc, 0x19, 0x0008);
		MP_WritePhyUshort(sc, 0x15, 0x0161);
		MP_WritePhyUshort(sc, 0x19, 0xdab1);
		MP_WritePhyUshort(sc, 0x15, 0x0162);
		MP_WritePhyUshort(sc, 0x19, 0x0015);
		MP_WritePhyUshort(sc, 0x15, 0x0163);
		MP_WritePhyUshort(sc, 0x19, 0xc244);
		MP_WritePhyUshort(sc, 0x15, 0x0164);
		MP_WritePhyUshort(sc, 0x19, 0x0013);
		MP_WritePhyUshort(sc, 0x15, 0x0165);
		MP_WritePhyUshort(sc, 0x19, 0xc021);
		MP_WritePhyUshort(sc, 0x15, 0x0166);
		MP_WritePhyUshort(sc, 0x19, 0x00f9);
		MP_WritePhyUshort(sc, 0x15, 0x0167);
		MP_WritePhyUshort(sc, 0x19, 0x3177);
		MP_WritePhyUshort(sc, 0x15, 0x0168);
		MP_WritePhyUshort(sc, 0x19, 0x5cf7);
		MP_WritePhyUshort(sc, 0x15, 0x0169);
		MP_WritePhyUshort(sc, 0x19, 0x4010);
		MP_WritePhyUshort(sc, 0x15, 0x016a);
		MP_WritePhyUshort(sc, 0x19, 0x4428);
		MP_WritePhyUshort(sc, 0x15, 0x016b);
		MP_WritePhyUshort(sc, 0x19, 0x9c00);
		MP_WritePhyUshort(sc, 0x15, 0x016c);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x016d);
		MP_WritePhyUshort(sc, 0x19, 0x6008);
		MP_WritePhyUshort(sc, 0x15, 0x016e);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x016f);
		MP_WritePhyUshort(sc, 0x19, 0x74f0);
		MP_WritePhyUshort(sc, 0x15, 0x0170);
		MP_WritePhyUshort(sc, 0x19, 0x6461);
		MP_WritePhyUshort(sc, 0x15, 0x0171);
		MP_WritePhyUshort(sc, 0x19, 0x6421);
		MP_WritePhyUshort(sc, 0x15, 0x0172);
		MP_WritePhyUshort(sc, 0x19, 0x64a1);
		MP_WritePhyUshort(sc, 0x15, 0x0173);
		MP_WritePhyUshort(sc, 0x19, 0x3116);
		MP_WritePhyUshort(sc, 0x15, 0x0174);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0175);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0176);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0177);
		MP_WritePhyUshort(sc, 0x19, 0x4805);
		MP_WritePhyUshort(sc, 0x15, 0x0178);
		MP_WritePhyUshort(sc, 0x19, 0xa103);
		MP_WritePhyUshort(sc, 0x15, 0x0179);
		MP_WritePhyUshort(sc, 0x19, 0x7c02);
		MP_WritePhyUshort(sc, 0x15, 0x017a);
		MP_WritePhyUshort(sc, 0x19, 0x6002);
		MP_WritePhyUshort(sc, 0x15, 0x017b);
		MP_WritePhyUshort(sc, 0x19, 0x7e00);
		MP_WritePhyUshort(sc, 0x15, 0x017c);
		MP_WritePhyUshort(sc, 0x19, 0x5400);
		MP_WritePhyUshort(sc, 0x15, 0x017d);
		MP_WritePhyUshort(sc, 0x19, 0x7c6b);
		MP_WritePhyUshort(sc, 0x15, 0x017e);
		MP_WritePhyUshort(sc, 0x19, 0x5c63);
		MP_WritePhyUshort(sc, 0x15, 0x017f);
		MP_WritePhyUshort(sc, 0x19, 0x407d);
		MP_WritePhyUshort(sc, 0x15, 0x0180);
		MP_WritePhyUshort(sc, 0x19, 0xa602);
		MP_WritePhyUshort(sc, 0x15, 0x0181);
		MP_WritePhyUshort(sc, 0x19, 0x4001);
		MP_WritePhyUshort(sc, 0x15, 0x0182);
		MP_WritePhyUshort(sc, 0x19, 0x4420);
		MP_WritePhyUshort(sc, 0x15, 0x0183);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x0184);
		MP_WritePhyUshort(sc, 0x19, 0x44a1);
		MP_WritePhyUshort(sc, 0x15, 0x0185);
		MP_WritePhyUshort(sc, 0x19, 0xd6e0);
		MP_WritePhyUshort(sc, 0x15, 0x0186);
		MP_WritePhyUshort(sc, 0x19, 0x0009);
		MP_WritePhyUshort(sc, 0x15, 0x0187);
		MP_WritePhyUshort(sc, 0x19, 0x9efe);
		MP_WritePhyUshort(sc, 0x15, 0x0188);
		MP_WritePhyUshort(sc, 0x19, 0x7c02);
		MP_WritePhyUshort(sc, 0x15, 0x0189);
		MP_WritePhyUshort(sc, 0x19, 0x6000);
		MP_WritePhyUshort(sc, 0x15, 0x018a);
		MP_WritePhyUshort(sc, 0x19, 0x9c00);
		MP_WritePhyUshort(sc, 0x15, 0x018b);
		MP_WritePhyUshort(sc, 0x19, 0x318f);
		MP_WritePhyUshort(sc, 0x15, 0x018c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x018d);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x018e);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x018f);
		MP_WritePhyUshort(sc, 0x19, 0x4806);
		MP_WritePhyUshort(sc, 0x15, 0x0190);
		MP_WritePhyUshort(sc, 0x19, 0x7c10);
		MP_WritePhyUshort(sc, 0x15, 0x0191);
		MP_WritePhyUshort(sc, 0x19, 0x5c10);
		MP_WritePhyUshort(sc, 0x15, 0x0192);
		MP_WritePhyUshort(sc, 0x19, 0x40fa);
		MP_WritePhyUshort(sc, 0x15, 0x0193);
		MP_WritePhyUshort(sc, 0x19, 0xa602);
		MP_WritePhyUshort(sc, 0x15, 0x0194);
		MP_WritePhyUshort(sc, 0x19, 0x4010);
		MP_WritePhyUshort(sc, 0x15, 0x0195);
		MP_WritePhyUshort(sc, 0x19, 0x4440);
		MP_WritePhyUshort(sc, 0x15, 0x0196);
		MP_WritePhyUshort(sc, 0x19, 0x9d00);
		MP_WritePhyUshort(sc, 0x15, 0x0197);
		MP_WritePhyUshort(sc, 0x19, 0x7c80);
		MP_WritePhyUshort(sc, 0x15, 0x0198);
		MP_WritePhyUshort(sc, 0x19, 0x6400);
		MP_WritePhyUshort(sc, 0x15, 0x0199);
		MP_WritePhyUshort(sc, 0x19, 0x4003);
		MP_WritePhyUshort(sc, 0x15, 0x019a);
		MP_WritePhyUshort(sc, 0x19, 0x4540);
		MP_WritePhyUshort(sc, 0x15, 0x019b);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x019c);
		MP_WritePhyUshort(sc, 0x19, 0x6008);
		MP_WritePhyUshort(sc, 0x15, 0x019d);
		MP_WritePhyUshort(sc, 0x19, 0x9f00);
		MP_WritePhyUshort(sc, 0x15, 0x019e);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x019f);
		MP_WritePhyUshort(sc, 0x19, 0x6400);
		MP_WritePhyUshort(sc, 0x15, 0x01a0);
		MP_WritePhyUshort(sc, 0x19, 0x7c80);
		MP_WritePhyUshort(sc, 0x15, 0x01a1);
		MP_WritePhyUshort(sc, 0x19, 0x6480);
		MP_WritePhyUshort(sc, 0x15, 0x01a2);
		MP_WritePhyUshort(sc, 0x19, 0x3140);
		MP_WritePhyUshort(sc, 0x15, 0x01a3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01a4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01a5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01a6);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x01a7);
		MP_WritePhyUshort(sc, 0x19, 0x7c0b);
		MP_WritePhyUshort(sc, 0x15, 0x01a8);
		MP_WritePhyUshort(sc, 0x19, 0x6c01);
		MP_WritePhyUshort(sc, 0x15, 0x01a9);
		MP_WritePhyUshort(sc, 0x19, 0x64a8);
		MP_WritePhyUshort(sc, 0x15, 0x01aa);
		MP_WritePhyUshort(sc, 0x19, 0x6800);
		MP_WritePhyUshort(sc, 0x15, 0x01ab);
		MP_WritePhyUshort(sc, 0x19, 0x5cf0);
		MP_WritePhyUshort(sc, 0x15, 0x01ac);
		MP_WritePhyUshort(sc, 0x19, 0x588f);
		MP_WritePhyUshort(sc, 0x15, 0x01ad);
		MP_WritePhyUshort(sc, 0x19, 0xb628);
		MP_WritePhyUshort(sc, 0x15, 0x01ae);
		MP_WritePhyUshort(sc, 0x19, 0xc053);
		MP_WritePhyUshort(sc, 0x15, 0x01af);
		MP_WritePhyUshort(sc, 0x19, 0x0026);
		MP_WritePhyUshort(sc, 0x15, 0x01b0);
		MP_WritePhyUshort(sc, 0x19, 0xc02d);
		MP_WritePhyUshort(sc, 0x15, 0x01b1);
		MP_WritePhyUshort(sc, 0x19, 0x0024);
		MP_WritePhyUshort(sc, 0x15, 0x01b2);
		MP_WritePhyUshort(sc, 0x19, 0xc603);
		MP_WritePhyUshort(sc, 0x15, 0x01b3);
		MP_WritePhyUshort(sc, 0x19, 0x0022);
		MP_WritePhyUshort(sc, 0x15, 0x01b4);
		MP_WritePhyUshort(sc, 0x19, 0x8cf9);
		MP_WritePhyUshort(sc, 0x15, 0x01b5);
		MP_WritePhyUshort(sc, 0x19, 0x31ba);
		MP_WritePhyUshort(sc, 0x15, 0x01b6);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01b7);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01b8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01b9);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01ba);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x01bb);
		MP_WritePhyUshort(sc, 0x19, 0x5420);
		MP_WritePhyUshort(sc, 0x15, 0x01bc);
		MP_WritePhyUshort(sc, 0x19, 0x4811);
		MP_WritePhyUshort(sc, 0x15, 0x01bd);
		MP_WritePhyUshort(sc, 0x19, 0x5000);
		MP_WritePhyUshort(sc, 0x15, 0x01be);
		MP_WritePhyUshort(sc, 0x19, 0x4801);
		MP_WritePhyUshort(sc, 0x15, 0x01bf);
		MP_WritePhyUshort(sc, 0x19, 0x6800);
		MP_WritePhyUshort(sc, 0x15, 0x01c0);
		MP_WritePhyUshort(sc, 0x19, 0x31f5);
		MP_WritePhyUshort(sc, 0x15, 0x01c1);
		MP_WritePhyUshort(sc, 0x19, 0xb614);
		MP_WritePhyUshort(sc, 0x15, 0x01c2);
		MP_WritePhyUshort(sc, 0x19, 0x8ce4);
		MP_WritePhyUshort(sc, 0x15, 0x01c3);
		MP_WritePhyUshort(sc, 0x19, 0xb30c);
		MP_WritePhyUshort(sc, 0x15, 0x01c4);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x01c5);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x01c6);
		MP_WritePhyUshort(sc, 0x19, 0x8206);
		MP_WritePhyUshort(sc, 0x15, 0x01c7);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x01c8);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x01c9);
		MP_WritePhyUshort(sc, 0x19, 0x7c04);
		MP_WritePhyUshort(sc, 0x15, 0x01ca);
		MP_WritePhyUshort(sc, 0x19, 0x7404);
		MP_WritePhyUshort(sc, 0x15, 0x01cb);
		MP_WritePhyUshort(sc, 0x19, 0x31c0);
		MP_WritePhyUshort(sc, 0x15, 0x01cc);
		MP_WritePhyUshort(sc, 0x19, 0x7c04);
		MP_WritePhyUshort(sc, 0x15, 0x01cd);
		MP_WritePhyUshort(sc, 0x19, 0x7400);
		MP_WritePhyUshort(sc, 0x15, 0x01ce);
		MP_WritePhyUshort(sc, 0x19, 0x31c0);
		MP_WritePhyUshort(sc, 0x15, 0x01cf);
		MP_WritePhyUshort(sc, 0x19, 0x8df1);
		MP_WritePhyUshort(sc, 0x15, 0x01d0);
		MP_WritePhyUshort(sc, 0x19, 0x3248);
		MP_WritePhyUshort(sc, 0x15, 0x01d1);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01d2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01d3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01d4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01d5);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x01d6);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x01d7);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x01d8);
		MP_WritePhyUshort(sc, 0x19, 0x7670);
		MP_WritePhyUshort(sc, 0x15, 0x01d9);
		MP_WritePhyUshort(sc, 0x19, 0x4023);
		MP_WritePhyUshort(sc, 0x15, 0x01da);
		MP_WritePhyUshort(sc, 0x19, 0x4500);
		MP_WritePhyUshort(sc, 0x15, 0x01db);
		MP_WritePhyUshort(sc, 0x19, 0x4069);
		MP_WritePhyUshort(sc, 0x15, 0x01dc);
		MP_WritePhyUshort(sc, 0x19, 0x4580);
		MP_WritePhyUshort(sc, 0x15, 0x01dd);
		MP_WritePhyUshort(sc, 0x19, 0x9f00);
		MP_WritePhyUshort(sc, 0x15, 0x01de);
		MP_WritePhyUshort(sc, 0x19, 0xcff5);
		MP_WritePhyUshort(sc, 0x15, 0x01df);
		MP_WritePhyUshort(sc, 0x19, 0x00ff);
		MP_WritePhyUshort(sc, 0x15, 0x01e0);
		MP_WritePhyUshort(sc, 0x19, 0x76f0);
		MP_WritePhyUshort(sc, 0x15, 0x01e1);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x01e2);
		MP_WritePhyUshort(sc, 0x19, 0x4023);
		MP_WritePhyUshort(sc, 0x15, 0x01e3);
		MP_WritePhyUshort(sc, 0x19, 0x4500);
		MP_WritePhyUshort(sc, 0x15, 0x01e4);
		MP_WritePhyUshort(sc, 0x19, 0x4069);
		MP_WritePhyUshort(sc, 0x15, 0x01e5);
		MP_WritePhyUshort(sc, 0x19, 0x4580);
		MP_WritePhyUshort(sc, 0x15, 0x01e6);
		MP_WritePhyUshort(sc, 0x19, 0x9f00);
		MP_WritePhyUshort(sc, 0x15, 0x01e7);
		MP_WritePhyUshort(sc, 0x19, 0xd0f5);
		MP_WritePhyUshort(sc, 0x15, 0x01e8);
		MP_WritePhyUshort(sc, 0x19, 0x00ff);
		MP_WritePhyUshort(sc, 0x15, 0x01e9);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x01ea);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x01eb);
		MP_WritePhyUshort(sc, 0x19, 0x6800);
		MP_WritePhyUshort(sc, 0x15, 0x01ec);
		MP_WritePhyUshort(sc, 0x19, 0x66a0);
		MP_WritePhyUshort(sc, 0x15, 0x01ed);
		MP_WritePhyUshort(sc, 0x19, 0x8300);
		MP_WritePhyUshort(sc, 0x15, 0x01ee);
		MP_WritePhyUshort(sc, 0x19, 0x74f0);
		MP_WritePhyUshort(sc, 0x15, 0x01ef);
		MP_WritePhyUshort(sc, 0x19, 0x3006);
		MP_WritePhyUshort(sc, 0x15, 0x01f0);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01f1);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01f2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01f3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01f4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x01f5);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x01f6);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x01f7);
		MP_WritePhyUshort(sc, 0x19, 0x409d);
		MP_WritePhyUshort(sc, 0x15, 0x01f8);
		MP_WritePhyUshort(sc, 0x19, 0x7c87);
		MP_WritePhyUshort(sc, 0x15, 0x01f9);
		MP_WritePhyUshort(sc, 0x19, 0xae14);
		MP_WritePhyUshort(sc, 0x15, 0x01fa);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x01fb);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x01fc);
		MP_WritePhyUshort(sc, 0x19, 0x6800);
		MP_WritePhyUshort(sc, 0x15, 0x01fd);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x01fe);
		MP_WritePhyUshort(sc, 0x19, 0x980e);
		MP_WritePhyUshort(sc, 0x15, 0x01ff);
		MP_WritePhyUshort(sc, 0x19, 0x930c);
		MP_WritePhyUshort(sc, 0x15, 0x0200);
		MP_WritePhyUshort(sc, 0x19, 0x9206);
		MP_WritePhyUshort(sc, 0x15, 0x0201);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x0202);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0203);
		MP_WritePhyUshort(sc, 0x19, 0x588f);
		MP_WritePhyUshort(sc, 0x15, 0x0204);
		MP_WritePhyUshort(sc, 0x19, 0x5520);
		MP_WritePhyUshort(sc, 0x15, 0x0205);
		MP_WritePhyUshort(sc, 0x19, 0x320c);
		MP_WritePhyUshort(sc, 0x15, 0x0206);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x15, 0x0207);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0208);
		MP_WritePhyUshort(sc, 0x19, 0x588d);
		MP_WritePhyUshort(sc, 0x15, 0x0209);
		MP_WritePhyUshort(sc, 0x19, 0x5500);
		MP_WritePhyUshort(sc, 0x15, 0x020a);
		MP_WritePhyUshort(sc, 0x19, 0x320c);
		MP_WritePhyUshort(sc, 0x15, 0x020b);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x020c);
		MP_WritePhyUshort(sc, 0x19, 0x3220);
		MP_WritePhyUshort(sc, 0x15, 0x020d);
		MP_WritePhyUshort(sc, 0x19, 0x4480);
		MP_WritePhyUshort(sc, 0x15, 0x020e);
		MP_WritePhyUshort(sc, 0x19, 0x9e03);
		MP_WritePhyUshort(sc, 0x15, 0x020f);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x0210);
		MP_WritePhyUshort(sc, 0x19, 0x6840);
		MP_WritePhyUshort(sc, 0x15, 0x0211);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x0212);
		MP_WritePhyUshort(sc, 0x19, 0x980e);
		MP_WritePhyUshort(sc, 0x15, 0x0213);
		MP_WritePhyUshort(sc, 0x19, 0x930c);
		MP_WritePhyUshort(sc, 0x15, 0x0214);
		MP_WritePhyUshort(sc, 0x19, 0x9206);
		MP_WritePhyUshort(sc, 0x15, 0x0215);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x15, 0x0216);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0217);
		MP_WritePhyUshort(sc, 0x19, 0x588f);
		MP_WritePhyUshort(sc, 0x15, 0x0218);
		MP_WritePhyUshort(sc, 0x19, 0x5520);
		MP_WritePhyUshort(sc, 0x15, 0x0219);
		MP_WritePhyUshort(sc, 0x19, 0x3220);
		MP_WritePhyUshort(sc, 0x15, 0x021a);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x021b);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x021c);
		MP_WritePhyUshort(sc, 0x19, 0x588d);
		MP_WritePhyUshort(sc, 0x15, 0x021d);
		MP_WritePhyUshort(sc, 0x19, 0x5540);
		MP_WritePhyUshort(sc, 0x15, 0x021e);
		MP_WritePhyUshort(sc, 0x19, 0x3220);
		MP_WritePhyUshort(sc, 0x15, 0x021f);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x15, 0x0220);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0221);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0222);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0223);
		MP_WritePhyUshort(sc, 0x19, 0x3231);
		MP_WritePhyUshort(sc, 0x15, 0x0224);
		MP_WritePhyUshort(sc, 0x19, 0xab06);
		MP_WritePhyUshort(sc, 0x15, 0x0225);
		MP_WritePhyUshort(sc, 0x19, 0xbf08);
		MP_WritePhyUshort(sc, 0x15, 0x0226);
		MP_WritePhyUshort(sc, 0x19, 0x4076);
		MP_WritePhyUshort(sc, 0x15, 0x0227);
		MP_WritePhyUshort(sc, 0x19, 0x7d07);
		MP_WritePhyUshort(sc, 0x15, 0x0228);
		MP_WritePhyUshort(sc, 0x19, 0x4502);
		MP_WritePhyUshort(sc, 0x15, 0x0229);
		MP_WritePhyUshort(sc, 0x19, 0x3231);
		MP_WritePhyUshort(sc, 0x15, 0x022a);
		MP_WritePhyUshort(sc, 0x19, 0x7d80);
		MP_WritePhyUshort(sc, 0x15, 0x022b);
		MP_WritePhyUshort(sc, 0x19, 0x5180);
		MP_WritePhyUshort(sc, 0x15, 0x022c);
		MP_WritePhyUshort(sc, 0x19, 0x322f);
		MP_WritePhyUshort(sc, 0x15, 0x022d);
		MP_WritePhyUshort(sc, 0x19, 0x7d80);
		MP_WritePhyUshort(sc, 0x15, 0x022e);
		MP_WritePhyUshort(sc, 0x19, 0x5000);
		MP_WritePhyUshort(sc, 0x15, 0x022f);
		MP_WritePhyUshort(sc, 0x19, 0x7d07);
		MP_WritePhyUshort(sc, 0x15, 0x0230);
		MP_WritePhyUshort(sc, 0x19, 0x4402);
		MP_WritePhyUshort(sc, 0x15, 0x0231);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0232);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x0233);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0234);
		MP_WritePhyUshort(sc, 0x19, 0xb309);
		MP_WritePhyUshort(sc, 0x15, 0x0235);
		MP_WritePhyUshort(sc, 0x19, 0xb204);
		MP_WritePhyUshort(sc, 0x15, 0x0236);
		MP_WritePhyUshort(sc, 0x19, 0xb105);
		MP_WritePhyUshort(sc, 0x15, 0x0237);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0238);
		MP_WritePhyUshort(sc, 0x19, 0x31c1);
		MP_WritePhyUshort(sc, 0x15, 0x0239);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x023a);
		MP_WritePhyUshort(sc, 0x19, 0x3261);
		MP_WritePhyUshort(sc, 0x15, 0x023b);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x023c);
		MP_WritePhyUshort(sc, 0x19, 0x3250);
		MP_WritePhyUshort(sc, 0x15, 0x023d);
		MP_WritePhyUshort(sc, 0x19, 0xb203);
		MP_WritePhyUshort(sc, 0x15, 0x023e);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x023f);
		MP_WritePhyUshort(sc, 0x19, 0x327a);
		MP_WritePhyUshort(sc, 0x15, 0x0240);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0241);
		MP_WritePhyUshort(sc, 0x19, 0x3293);
		MP_WritePhyUshort(sc, 0x15, 0x0242);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0243);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0244);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0245);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0246);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0247);
		MP_WritePhyUshort(sc, 0x19, 0x32a3);
		MP_WritePhyUshort(sc, 0x15, 0x0248);
		MP_WritePhyUshort(sc, 0x19, 0x5520);
		MP_WritePhyUshort(sc, 0x15, 0x0249);
		MP_WritePhyUshort(sc, 0x19, 0x403d);
		MP_WritePhyUshort(sc, 0x15, 0x024a);
		MP_WritePhyUshort(sc, 0x19, 0x440c);
		MP_WritePhyUshort(sc, 0x15, 0x024b);
		MP_WritePhyUshort(sc, 0x19, 0x4812);
		MP_WritePhyUshort(sc, 0x15, 0x024c);
		MP_WritePhyUshort(sc, 0x19, 0x5001);
		MP_WritePhyUshort(sc, 0x15, 0x024d);
		MP_WritePhyUshort(sc, 0x19, 0x4802);
		MP_WritePhyUshort(sc, 0x15, 0x024e);
		MP_WritePhyUshort(sc, 0x19, 0x6880);
		MP_WritePhyUshort(sc, 0x15, 0x024f);
		MP_WritePhyUshort(sc, 0x19, 0x31f5);
		MP_WritePhyUshort(sc, 0x15, 0x0250);
		MP_WritePhyUshort(sc, 0x19, 0xb685);
		MP_WritePhyUshort(sc, 0x15, 0x0251);
		MP_WritePhyUshort(sc, 0x19, 0x801c);
		MP_WritePhyUshort(sc, 0x15, 0x0252);
		MP_WritePhyUshort(sc, 0x19, 0xbaf5);
		MP_WritePhyUshort(sc, 0x15, 0x0253);
		MP_WritePhyUshort(sc, 0x19, 0xc07c);
		MP_WritePhyUshort(sc, 0x15, 0x0254);
		MP_WritePhyUshort(sc, 0x19, 0x00fb);
		MP_WritePhyUshort(sc, 0x15, 0x0255);
		MP_WritePhyUshort(sc, 0x19, 0x325a);
		MP_WritePhyUshort(sc, 0x15, 0x0256);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0257);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0258);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0259);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x025a);
		MP_WritePhyUshort(sc, 0x19, 0x481a);
		MP_WritePhyUshort(sc, 0x15, 0x025b);
		MP_WritePhyUshort(sc, 0x19, 0x5001);
		MP_WritePhyUshort(sc, 0x15, 0x025c);
		MP_WritePhyUshort(sc, 0x19, 0x401b);
		MP_WritePhyUshort(sc, 0x15, 0x025d);
		MP_WritePhyUshort(sc, 0x19, 0x480a);
		MP_WritePhyUshort(sc, 0x15, 0x025e);
		MP_WritePhyUshort(sc, 0x19, 0x4418);
		MP_WritePhyUshort(sc, 0x15, 0x025f);
		MP_WritePhyUshort(sc, 0x19, 0x6900);
		MP_WritePhyUshort(sc, 0x15, 0x0260);
		MP_WritePhyUshort(sc, 0x19, 0x31f5);
		MP_WritePhyUshort(sc, 0x15, 0x0261);
		MP_WritePhyUshort(sc, 0x19, 0xb64b);
		MP_WritePhyUshort(sc, 0x15, 0x0262);
		MP_WritePhyUshort(sc, 0x19, 0xdb00);
		MP_WritePhyUshort(sc, 0x15, 0x0263);
		MP_WritePhyUshort(sc, 0x19, 0x0048);
		MP_WritePhyUshort(sc, 0x15, 0x0264);
		MP_WritePhyUshort(sc, 0x19, 0xdb7d);
		MP_WritePhyUshort(sc, 0x15, 0x0265);
		MP_WritePhyUshort(sc, 0x19, 0x0002);
		MP_WritePhyUshort(sc, 0x15, 0x0266);
		MP_WritePhyUshort(sc, 0x19, 0xa0fa);
		MP_WritePhyUshort(sc, 0x15, 0x0267);
		MP_WritePhyUshort(sc, 0x19, 0x4408);
		MP_WritePhyUshort(sc, 0x15, 0x0268);
		MP_WritePhyUshort(sc, 0x19, 0x3248);
		MP_WritePhyUshort(sc, 0x15, 0x0269);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x026a);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x026b);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x026c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x026d);
		MP_WritePhyUshort(sc, 0x19, 0xb806);
		MP_WritePhyUshort(sc, 0x15, 0x026e);
		MP_WritePhyUshort(sc, 0x19, 0x588d);
		MP_WritePhyUshort(sc, 0x15, 0x026f);
		MP_WritePhyUshort(sc, 0x19, 0x5500);
		MP_WritePhyUshort(sc, 0x15, 0x0270);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x0271);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x0272);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0273);
		MP_WritePhyUshort(sc, 0x19, 0x4814);
		MP_WritePhyUshort(sc, 0x15, 0x0274);
		MP_WritePhyUshort(sc, 0x19, 0x500b);
		MP_WritePhyUshort(sc, 0x15, 0x0275);
		MP_WritePhyUshort(sc, 0x19, 0x4804);
		MP_WritePhyUshort(sc, 0x15, 0x0276);
		MP_WritePhyUshort(sc, 0x19, 0x40c4);
		MP_WritePhyUshort(sc, 0x15, 0x0277);
		MP_WritePhyUshort(sc, 0x19, 0x4425);
		MP_WritePhyUshort(sc, 0x15, 0x0278);
		MP_WritePhyUshort(sc, 0x19, 0x6a00);
		MP_WritePhyUshort(sc, 0x15, 0x0279);
		MP_WritePhyUshort(sc, 0x19, 0x31f5);
		MP_WritePhyUshort(sc, 0x15, 0x027a);
		MP_WritePhyUshort(sc, 0x19, 0xb632);
		MP_WritePhyUshort(sc, 0x15, 0x027b);
		MP_WritePhyUshort(sc, 0x19, 0xdc03);
		MP_WritePhyUshort(sc, 0x15, 0x027c);
		MP_WritePhyUshort(sc, 0x19, 0x0027);
		MP_WritePhyUshort(sc, 0x15, 0x027d);
		MP_WritePhyUshort(sc, 0x19, 0x80fc);
		MP_WritePhyUshort(sc, 0x15, 0x027e);
		MP_WritePhyUshort(sc, 0x19, 0x3283);
		MP_WritePhyUshort(sc, 0x15, 0x027f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0280);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0281);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0282);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0283);
		MP_WritePhyUshort(sc, 0x19, 0xb806);
		MP_WritePhyUshort(sc, 0x15, 0x0284);
		MP_WritePhyUshort(sc, 0x19, 0x588f);
		MP_WritePhyUshort(sc, 0x15, 0x0285);
		MP_WritePhyUshort(sc, 0x19, 0x5520);
		MP_WritePhyUshort(sc, 0x15, 0x0286);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x0287);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x15, 0x0288);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0289);
		MP_WritePhyUshort(sc, 0x19, 0x4818);
		MP_WritePhyUshort(sc, 0x15, 0x028a);
		MP_WritePhyUshort(sc, 0x19, 0x5051);
		MP_WritePhyUshort(sc, 0x15, 0x028b);
		MP_WritePhyUshort(sc, 0x19, 0x4808);
		MP_WritePhyUshort(sc, 0x15, 0x028c);
		MP_WritePhyUshort(sc, 0x19, 0x4050);
		MP_WritePhyUshort(sc, 0x15, 0x028d);
		MP_WritePhyUshort(sc, 0x19, 0x4462);
		MP_WritePhyUshort(sc, 0x15, 0x028e);
		MP_WritePhyUshort(sc, 0x19, 0x40c4);
		MP_WritePhyUshort(sc, 0x15, 0x028f);
		MP_WritePhyUshort(sc, 0x19, 0x4473);
		MP_WritePhyUshort(sc, 0x15, 0x0290);
		MP_WritePhyUshort(sc, 0x19, 0x5041);
		MP_WritePhyUshort(sc, 0x15, 0x0291);
		MP_WritePhyUshort(sc, 0x19, 0x6b00);
		MP_WritePhyUshort(sc, 0x15, 0x0292);
		MP_WritePhyUshort(sc, 0x19, 0x31f5);
		MP_WritePhyUshort(sc, 0x15, 0x0293);
		MP_WritePhyUshort(sc, 0x19, 0xb619);
		MP_WritePhyUshort(sc, 0x15, 0x0294);
		MP_WritePhyUshort(sc, 0x19, 0x80d9);
		MP_WritePhyUshort(sc, 0x15, 0x0295);
		MP_WritePhyUshort(sc, 0x19, 0xbd06);
		MP_WritePhyUshort(sc, 0x15, 0x0296);
		MP_WritePhyUshort(sc, 0x19, 0xbb0d);
		MP_WritePhyUshort(sc, 0x15, 0x0297);
		MP_WritePhyUshort(sc, 0x19, 0xaf14);
		MP_WritePhyUshort(sc, 0x15, 0x0298);
		MP_WritePhyUshort(sc, 0x19, 0x8efa);
		MP_WritePhyUshort(sc, 0x15, 0x0299);
		MP_WritePhyUshort(sc, 0x19, 0x5049);
		MP_WritePhyUshort(sc, 0x15, 0x029a);
		MP_WritePhyUshort(sc, 0x19, 0x3248);
		MP_WritePhyUshort(sc, 0x15, 0x029b);
		MP_WritePhyUshort(sc, 0x19, 0x4c10);
		MP_WritePhyUshort(sc, 0x15, 0x029c);
		MP_WritePhyUshort(sc, 0x19, 0x44b0);
		MP_WritePhyUshort(sc, 0x15, 0x029d);
		MP_WritePhyUshort(sc, 0x19, 0x4c00);
		MP_WritePhyUshort(sc, 0x15, 0x029e);
		MP_WritePhyUshort(sc, 0x19, 0x3292);
		MP_WritePhyUshort(sc, 0x15, 0x029f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02a0);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02a1);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02a2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02a3);
		MP_WritePhyUshort(sc, 0x19, 0x481f);
		MP_WritePhyUshort(sc, 0x15, 0x02a4);
		MP_WritePhyUshort(sc, 0x19, 0x5005);
		MP_WritePhyUshort(sc, 0x15, 0x02a5);
		MP_WritePhyUshort(sc, 0x19, 0x480f);
		MP_WritePhyUshort(sc, 0x15, 0x02a6);
		MP_WritePhyUshort(sc, 0x19, 0xac00);
		MP_WritePhyUshort(sc, 0x15, 0x02a7);
		MP_WritePhyUshort(sc, 0x19, 0x31a6);
		MP_WritePhyUshort(sc, 0x15, 0x02a8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02a9);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02aa);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02ab);
		MP_WritePhyUshort(sc, 0x19, 0x31ba);
		MP_WritePhyUshort(sc, 0x15, 0x02ac);
		MP_WritePhyUshort(sc, 0x19, 0x31d5);
		MP_WritePhyUshort(sc, 0x15, 0x02ad);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02ae);
		MP_WritePhyUshort(sc, 0x19, 0x5cf0);
		MP_WritePhyUshort(sc, 0x15, 0x02af);
		MP_WritePhyUshort(sc, 0x19, 0x588c);
		MP_WritePhyUshort(sc, 0x15, 0x02b0);
		MP_WritePhyUshort(sc, 0x19, 0x542f);
		MP_WritePhyUshort(sc, 0x15, 0x02b1);
		MP_WritePhyUshort(sc, 0x19, 0x7ffb);
		MP_WritePhyUshort(sc, 0x15, 0x02b2);
		MP_WritePhyUshort(sc, 0x19, 0x6ff8);
		MP_WritePhyUshort(sc, 0x15, 0x02b3);
		MP_WritePhyUshort(sc, 0x19, 0x64a4);
		MP_WritePhyUshort(sc, 0x15, 0x02b4);
		MP_WritePhyUshort(sc, 0x19, 0x64a0);
		MP_WritePhyUshort(sc, 0x15, 0x02b5);
		MP_WritePhyUshort(sc, 0x19, 0x6800);
		MP_WritePhyUshort(sc, 0x15, 0x02b6);
		MP_WritePhyUshort(sc, 0x19, 0x4400);
		MP_WritePhyUshort(sc, 0x15, 0x02b7);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x02b8);
		MP_WritePhyUshort(sc, 0x19, 0x4480);
		MP_WritePhyUshort(sc, 0x15, 0x02b9);
		MP_WritePhyUshort(sc, 0x19, 0x9e00);
		MP_WritePhyUshort(sc, 0x15, 0x02ba);
		MP_WritePhyUshort(sc, 0x19, 0x4891);
		MP_WritePhyUshort(sc, 0x15, 0x02bb);
		MP_WritePhyUshort(sc, 0x19, 0x4cc0);
		MP_WritePhyUshort(sc, 0x15, 0x02bc);
		MP_WritePhyUshort(sc, 0x19, 0x4801);
		MP_WritePhyUshort(sc, 0x15, 0x02bd);
		MP_WritePhyUshort(sc, 0x19, 0xa609);
		MP_WritePhyUshort(sc, 0x15, 0x02be);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x02bf);
		MP_WritePhyUshort(sc, 0x19, 0x004e);
		MP_WritePhyUshort(sc, 0x15, 0x02c0);
		MP_WritePhyUshort(sc, 0x19, 0x87fe);
		MP_WritePhyUshort(sc, 0x15, 0x02c1);
		MP_WritePhyUshort(sc, 0x19, 0x32c6);
		MP_WritePhyUshort(sc, 0x15, 0x02c2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02c3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02c4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02c5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02c6);
		MP_WritePhyUshort(sc, 0x19, 0x48b2);
		MP_WritePhyUshort(sc, 0x15, 0x02c7);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x02c8);
		MP_WritePhyUshort(sc, 0x19, 0x4822);
		MP_WritePhyUshort(sc, 0x15, 0x02c9);
		MP_WritePhyUshort(sc, 0x19, 0x4488);
		MP_WritePhyUshort(sc, 0x15, 0x02ca);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x02cb);
		MP_WritePhyUshort(sc, 0x19, 0x0042);
		MP_WritePhyUshort(sc, 0x15, 0x02cc);
		MP_WritePhyUshort(sc, 0x19, 0x8203);
		MP_WritePhyUshort(sc, 0x15, 0x02cd);
		MP_WritePhyUshort(sc, 0x19, 0x4cc8);
		MP_WritePhyUshort(sc, 0x15, 0x02ce);
		MP_WritePhyUshort(sc, 0x19, 0x32d0);
		MP_WritePhyUshort(sc, 0x15, 0x02cf);
		MP_WritePhyUshort(sc, 0x19, 0x4cc0);
		MP_WritePhyUshort(sc, 0x15, 0x02d0);
		MP_WritePhyUshort(sc, 0x19, 0xc4d4);
		MP_WritePhyUshort(sc, 0x15, 0x02d1);
		MP_WritePhyUshort(sc, 0x19, 0x00f9);
		MP_WritePhyUshort(sc, 0x15, 0x02d2);
		MP_WritePhyUshort(sc, 0x19, 0xa51a);
		MP_WritePhyUshort(sc, 0x15, 0x02d3);
		MP_WritePhyUshort(sc, 0x19, 0x32d9);
		MP_WritePhyUshort(sc, 0x15, 0x02d4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02d5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02d6);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02d7);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02d8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02d9);
		MP_WritePhyUshort(sc, 0x19, 0x48b3);
		MP_WritePhyUshort(sc, 0x15, 0x02da);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x02db);
		MP_WritePhyUshort(sc, 0x19, 0x4823);
		MP_WritePhyUshort(sc, 0x15, 0x02dc);
		MP_WritePhyUshort(sc, 0x19, 0x4410);
		MP_WritePhyUshort(sc, 0x15, 0x02dd);
		MP_WritePhyUshort(sc, 0x19, 0xb630);
		MP_WritePhyUshort(sc, 0x15, 0x02de);
		MP_WritePhyUshort(sc, 0x19, 0x7dc8);
		MP_WritePhyUshort(sc, 0x15, 0x02df);
		MP_WritePhyUshort(sc, 0x19, 0x8203);
		MP_WritePhyUshort(sc, 0x15, 0x02e0);
		MP_WritePhyUshort(sc, 0x19, 0x4c48);
		MP_WritePhyUshort(sc, 0x15, 0x02e1);
		MP_WritePhyUshort(sc, 0x19, 0x32e3);
		MP_WritePhyUshort(sc, 0x15, 0x02e2);
		MP_WritePhyUshort(sc, 0x19, 0x4c40);
		MP_WritePhyUshort(sc, 0x15, 0x02e3);
		MP_WritePhyUshort(sc, 0x19, 0x9bfa);
		MP_WritePhyUshort(sc, 0x15, 0x02e4);
		MP_WritePhyUshort(sc, 0x19, 0x84ca);
		MP_WritePhyUshort(sc, 0x15, 0x02e5);
		MP_WritePhyUshort(sc, 0x19, 0x85f8);
		MP_WritePhyUshort(sc, 0x15, 0x02e6);
		MP_WritePhyUshort(sc, 0x19, 0x32ec);
		MP_WritePhyUshort(sc, 0x15, 0x02e7);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02e8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02e9);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02ea);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02eb);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x02ec);
		MP_WritePhyUshort(sc, 0x19, 0x48d4);
		MP_WritePhyUshort(sc, 0x15, 0x02ed);
		MP_WritePhyUshort(sc, 0x19, 0x4020);
		MP_WritePhyUshort(sc, 0x15, 0x02ee);
		MP_WritePhyUshort(sc, 0x19, 0x4844);
		MP_WritePhyUshort(sc, 0x15, 0x02ef);
		MP_WritePhyUshort(sc, 0x19, 0x4420);
		MP_WritePhyUshort(sc, 0x15, 0x02f0);
		MP_WritePhyUshort(sc, 0x19, 0x6800);
		MP_WritePhyUshort(sc, 0x15, 0x02f1);
		MP_WritePhyUshort(sc, 0x19, 0x7dc0);
		MP_WritePhyUshort(sc, 0x15, 0x02f2);
		MP_WritePhyUshort(sc, 0x19, 0x4c40);
		MP_WritePhyUshort(sc, 0x15, 0x02f3);
		MP_WritePhyUshort(sc, 0x19, 0x7c0b);
		MP_WritePhyUshort(sc, 0x15, 0x02f4);
		MP_WritePhyUshort(sc, 0x19, 0x6c08);
		MP_WritePhyUshort(sc, 0x15, 0x02f5);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x02f6);
		MP_WritePhyUshort(sc, 0x19, 0x9cfd);
		MP_WritePhyUshort(sc, 0x15, 0x02f7);
		MP_WritePhyUshort(sc, 0x19, 0xb616);
		MP_WritePhyUshort(sc, 0x15, 0x02f8);
		MP_WritePhyUshort(sc, 0x19, 0xc42b);
		MP_WritePhyUshort(sc, 0x15, 0x02f9);
		MP_WritePhyUshort(sc, 0x19, 0x00e0);
		MP_WritePhyUshort(sc, 0x15, 0x02fa);
		MP_WritePhyUshort(sc, 0x19, 0xc455);
		MP_WritePhyUshort(sc, 0x15, 0x02fb);
		MP_WritePhyUshort(sc, 0x19, 0x00b3);
		MP_WritePhyUshort(sc, 0x15, 0x02fc);
		MP_WritePhyUshort(sc, 0x19, 0xb20a);
		MP_WritePhyUshort(sc, 0x15, 0x02fd);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x02fe);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x02ff);
		MP_WritePhyUshort(sc, 0x19, 0x8204);
		MP_WritePhyUshort(sc, 0x15, 0x0300);
		MP_WritePhyUshort(sc, 0x19, 0x7c04);
		MP_WritePhyUshort(sc, 0x15, 0x0301);
		MP_WritePhyUshort(sc, 0x19, 0x7404);
		MP_WritePhyUshort(sc, 0x15, 0x0302);
		MP_WritePhyUshort(sc, 0x19, 0x32f3);
		MP_WritePhyUshort(sc, 0x15, 0x0303);
		MP_WritePhyUshort(sc, 0x19, 0x7c04);
		MP_WritePhyUshort(sc, 0x15, 0x0304);
		MP_WritePhyUshort(sc, 0x19, 0x7400);
		MP_WritePhyUshort(sc, 0x15, 0x0305);
		MP_WritePhyUshort(sc, 0x19, 0x32f3);
		MP_WritePhyUshort(sc, 0x15, 0x0306);
		MP_WritePhyUshort(sc, 0x19, 0xefed);
		MP_WritePhyUshort(sc, 0x15, 0x0307);
		MP_WritePhyUshort(sc, 0x19, 0x3342);
		MP_WritePhyUshort(sc, 0x15, 0x0308);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0309);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x030a);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x030b);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x030c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x030d);
		MP_WritePhyUshort(sc, 0x19, 0x3006);
		MP_WritePhyUshort(sc, 0x15, 0x030e);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x030f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0310);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0311);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x0312);
		MP_WritePhyUshort(sc, 0x19, 0xa207);
		MP_WritePhyUshort(sc, 0x15, 0x0313);
		MP_WritePhyUshort(sc, 0x19, 0x4c00);
		MP_WritePhyUshort(sc, 0x15, 0x0314);
		MP_WritePhyUshort(sc, 0x19, 0x3322);
		MP_WritePhyUshort(sc, 0x15, 0x0315);
		MP_WritePhyUshort(sc, 0x19, 0x4041);
		MP_WritePhyUshort(sc, 0x15, 0x0316);
		MP_WritePhyUshort(sc, 0x19, 0x7d07);
		MP_WritePhyUshort(sc, 0x15, 0x0317);
		MP_WritePhyUshort(sc, 0x19, 0x4502);
		MP_WritePhyUshort(sc, 0x15, 0x0318);
		MP_WritePhyUshort(sc, 0x19, 0x3322);
		MP_WritePhyUshort(sc, 0x15, 0x0319);
		MP_WritePhyUshort(sc, 0x19, 0x4c08);
		MP_WritePhyUshort(sc, 0x15, 0x031a);
		MP_WritePhyUshort(sc, 0x19, 0x3322);
		MP_WritePhyUshort(sc, 0x15, 0x031b);
		MP_WritePhyUshort(sc, 0x19, 0x7d80);
		MP_WritePhyUshort(sc, 0x15, 0x031c);
		MP_WritePhyUshort(sc, 0x19, 0x5180);
		MP_WritePhyUshort(sc, 0x15, 0x031d);
		MP_WritePhyUshort(sc, 0x19, 0x3320);
		MP_WritePhyUshort(sc, 0x15, 0x031e);
		MP_WritePhyUshort(sc, 0x19, 0x7d80);
		MP_WritePhyUshort(sc, 0x15, 0x031f);
		MP_WritePhyUshort(sc, 0x19, 0x5000);
		MP_WritePhyUshort(sc, 0x15, 0x0320);
		MP_WritePhyUshort(sc, 0x19, 0x7d07);
		MP_WritePhyUshort(sc, 0x15, 0x0321);
		MP_WritePhyUshort(sc, 0x19, 0x4402);
		MP_WritePhyUshort(sc, 0x15, 0x0322);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0323);
		MP_WritePhyUshort(sc, 0x19, 0x6c02);
		MP_WritePhyUshort(sc, 0x15, 0x0324);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x0325);
		MP_WritePhyUshort(sc, 0x19, 0xb30c);
		MP_WritePhyUshort(sc, 0x15, 0x0326);
		MP_WritePhyUshort(sc, 0x19, 0xb206);
		MP_WritePhyUshort(sc, 0x15, 0x0327);
		MP_WritePhyUshort(sc, 0x19, 0xb103);
		MP_WritePhyUshort(sc, 0x15, 0x0328);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0329);
		MP_WritePhyUshort(sc, 0x19, 0x32f6);
		MP_WritePhyUshort(sc, 0x15, 0x032a);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x032b);
		MP_WritePhyUshort(sc, 0x19, 0x3352);
		MP_WritePhyUshort(sc, 0x15, 0x032c);
		MP_WritePhyUshort(sc, 0x19, 0xb103);
		MP_WritePhyUshort(sc, 0x15, 0x032d);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x032e);
		MP_WritePhyUshort(sc, 0x19, 0x336a);
		MP_WritePhyUshort(sc, 0x15, 0x032f);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0330);
		MP_WritePhyUshort(sc, 0x19, 0x3382);
		MP_WritePhyUshort(sc, 0x15, 0x0331);
		MP_WritePhyUshort(sc, 0x19, 0xb206);
		MP_WritePhyUshort(sc, 0x15, 0x0332);
		MP_WritePhyUshort(sc, 0x19, 0xb103);
		MP_WritePhyUshort(sc, 0x15, 0x0333);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0334);
		MP_WritePhyUshort(sc, 0x19, 0x3395);
		MP_WritePhyUshort(sc, 0x15, 0x0335);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0336);
		MP_WritePhyUshort(sc, 0x19, 0x33c6);
		MP_WritePhyUshort(sc, 0x15, 0x0337);
		MP_WritePhyUshort(sc, 0x19, 0xb103);
		MP_WritePhyUshort(sc, 0x15, 0x0338);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x0339);
		MP_WritePhyUshort(sc, 0x19, 0x33d7);
		MP_WritePhyUshort(sc, 0x15, 0x033a);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x033b);
		MP_WritePhyUshort(sc, 0x19, 0x33f2);
		MP_WritePhyUshort(sc, 0x15, 0x033c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x033d);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x033e);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x033f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0340);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0341);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0342);
		MP_WritePhyUshort(sc, 0x19, 0x49b5);
		MP_WritePhyUshort(sc, 0x15, 0x0343);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x15, 0x0344);
		MP_WritePhyUshort(sc, 0x19, 0x4d00);
		MP_WritePhyUshort(sc, 0x15, 0x0345);
		MP_WritePhyUshort(sc, 0x19, 0x6880);
		MP_WritePhyUshort(sc, 0x15, 0x0346);
		MP_WritePhyUshort(sc, 0x19, 0x7c08);
		MP_WritePhyUshort(sc, 0x15, 0x0347);
		MP_WritePhyUshort(sc, 0x19, 0x6c08);
		MP_WritePhyUshort(sc, 0x15, 0x0348);
		MP_WritePhyUshort(sc, 0x19, 0x4925);
		MP_WritePhyUshort(sc, 0x15, 0x0349);
		MP_WritePhyUshort(sc, 0x19, 0x403b);
		MP_WritePhyUshort(sc, 0x15, 0x034a);
		MP_WritePhyUshort(sc, 0x19, 0xa602);
		MP_WritePhyUshort(sc, 0x15, 0x034b);
		MP_WritePhyUshort(sc, 0x19, 0x402f);
		MP_WritePhyUshort(sc, 0x15, 0x034c);
		MP_WritePhyUshort(sc, 0x19, 0x4484);
		MP_WritePhyUshort(sc, 0x15, 0x034d);
		MP_WritePhyUshort(sc, 0x19, 0x40c8);
		MP_WritePhyUshort(sc, 0x15, 0x034e);
		MP_WritePhyUshort(sc, 0x19, 0x44c4);
		MP_WritePhyUshort(sc, 0x15, 0x034f);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x0350);
		MP_WritePhyUshort(sc, 0x19, 0x00bd);
		MP_WritePhyUshort(sc, 0x15, 0x0351);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x0352);
		MP_WritePhyUshort(sc, 0x19, 0xc8ed);
		MP_WritePhyUshort(sc, 0x15, 0x0353);
		MP_WritePhyUshort(sc, 0x19, 0x00fc);
		MP_WritePhyUshort(sc, 0x15, 0x0354);
		MP_WritePhyUshort(sc, 0x19, 0x8221);
		MP_WritePhyUshort(sc, 0x15, 0x0355);
		MP_WritePhyUshort(sc, 0x19, 0xd11d);
		MP_WritePhyUshort(sc, 0x15, 0x0356);
		MP_WritePhyUshort(sc, 0x19, 0x001f);
		MP_WritePhyUshort(sc, 0x15, 0x0357);
		MP_WritePhyUshort(sc, 0x19, 0xde18);
		MP_WritePhyUshort(sc, 0x15, 0x0358);
		MP_WritePhyUshort(sc, 0x19, 0x0008);
		MP_WritePhyUshort(sc, 0x15, 0x0359);
		MP_WritePhyUshort(sc, 0x19, 0x91f6);
		MP_WritePhyUshort(sc, 0x15, 0x035a);
		MP_WritePhyUshort(sc, 0x19, 0x3360);
		MP_WritePhyUshort(sc, 0x15, 0x035b);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x035c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x035d);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x035e);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x035f);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0360);
		MP_WritePhyUshort(sc, 0x19, 0x4bb6);
		MP_WritePhyUshort(sc, 0x15, 0x0361);
		MP_WritePhyUshort(sc, 0x19, 0x4064);
		MP_WritePhyUshort(sc, 0x15, 0x0362);
		MP_WritePhyUshort(sc, 0x19, 0x4b26);
		MP_WritePhyUshort(sc, 0x15, 0x0363);
		MP_WritePhyUshort(sc, 0x19, 0x4410);
		MP_WritePhyUshort(sc, 0x15, 0x0364);
		MP_WritePhyUshort(sc, 0x19, 0x4006);
		MP_WritePhyUshort(sc, 0x15, 0x0365);
		MP_WritePhyUshort(sc, 0x19, 0x4490);
		MP_WritePhyUshort(sc, 0x15, 0x0366);
		MP_WritePhyUshort(sc, 0x19, 0x6900);
		MP_WritePhyUshort(sc, 0x15, 0x0367);
		MP_WritePhyUshort(sc, 0x19, 0xb6a6);
		MP_WritePhyUshort(sc, 0x15, 0x0368);
		MP_WritePhyUshort(sc, 0x19, 0x9e02);
		MP_WritePhyUshort(sc, 0x15, 0x0369);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x036a);
		MP_WritePhyUshort(sc, 0x19, 0xd11d);
		MP_WritePhyUshort(sc, 0x15, 0x036b);
		MP_WritePhyUshort(sc, 0x19, 0x000a);
		MP_WritePhyUshort(sc, 0x15, 0x036c);
		MP_WritePhyUshort(sc, 0x19, 0xbb0f);
		MP_WritePhyUshort(sc, 0x15, 0x036d);
		MP_WritePhyUshort(sc, 0x19, 0x8102);
		MP_WritePhyUshort(sc, 0x15, 0x036e);
		MP_WritePhyUshort(sc, 0x19, 0x3371);
		MP_WritePhyUshort(sc, 0x15, 0x036f);
		MP_WritePhyUshort(sc, 0x19, 0xa21e);
		MP_WritePhyUshort(sc, 0x15, 0x0370);
		MP_WritePhyUshort(sc, 0x19, 0x33b6);
		MP_WritePhyUshort(sc, 0x15, 0x0371);
		MP_WritePhyUshort(sc, 0x19, 0x91f6);
		MP_WritePhyUshort(sc, 0x15, 0x0372);
		MP_WritePhyUshort(sc, 0x19, 0xc218);
		MP_WritePhyUshort(sc, 0x15, 0x0373);
		MP_WritePhyUshort(sc, 0x19, 0x00f4);
		MP_WritePhyUshort(sc, 0x15, 0x0374);
		MP_WritePhyUshort(sc, 0x19, 0x33b6);
		MP_WritePhyUshort(sc, 0x15, 0x0375);
		MP_WritePhyUshort(sc, 0x19, 0x32ec);
		MP_WritePhyUshort(sc, 0x15, 0x0376);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0377);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0378);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x0379);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x037a);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x037b);
		MP_WritePhyUshort(sc, 0x19, 0x4b97);
		MP_WritePhyUshort(sc, 0x15, 0x037c);
		MP_WritePhyUshort(sc, 0x19, 0x402b);
		MP_WritePhyUshort(sc, 0x15, 0x037d);
		MP_WritePhyUshort(sc, 0x19, 0x4b07);
		MP_WritePhyUshort(sc, 0x15, 0x037e);
		MP_WritePhyUshort(sc, 0x19, 0x4422);
		MP_WritePhyUshort(sc, 0x15, 0x037f);
		MP_WritePhyUshort(sc, 0x19, 0x6980);
		MP_WritePhyUshort(sc, 0x15, 0x0380);
		MP_WritePhyUshort(sc, 0x19, 0xb608);
		MP_WritePhyUshort(sc, 0x15, 0x0381);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x0382);
		MP_WritePhyUshort(sc, 0x19, 0xbc05);
		MP_WritePhyUshort(sc, 0x15, 0x0383);
		MP_WritePhyUshort(sc, 0x19, 0xc21c);
		MP_WritePhyUshort(sc, 0x15, 0x0384);
		MP_WritePhyUshort(sc, 0x19, 0x0032);
		MP_WritePhyUshort(sc, 0x15, 0x0385);
		MP_WritePhyUshort(sc, 0x19, 0xa1fb);
		MP_WritePhyUshort(sc, 0x15, 0x0386);
		MP_WritePhyUshort(sc, 0x19, 0x338d);
		MP_WritePhyUshort(sc, 0x15, 0x0387);
		MP_WritePhyUshort(sc, 0x19, 0x32ae);
		MP_WritePhyUshort(sc, 0x15, 0x0388);
		MP_WritePhyUshort(sc, 0x19, 0x330d);
		MP_WritePhyUshort(sc, 0x15, 0x0389);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x038a);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x038b);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x038c);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x038d);
		MP_WritePhyUshort(sc, 0x19, 0x4b97);
		MP_WritePhyUshort(sc, 0x15, 0x038e);
		MP_WritePhyUshort(sc, 0x19, 0x6a08);
		MP_WritePhyUshort(sc, 0x15, 0x038f);
		MP_WritePhyUshort(sc, 0x19, 0x4b07);
		MP_WritePhyUshort(sc, 0x15, 0x0390);
		MP_WritePhyUshort(sc, 0x19, 0x40ac);
		MP_WritePhyUshort(sc, 0x15, 0x0391);
		MP_WritePhyUshort(sc, 0x19, 0x4445);
		MP_WritePhyUshort(sc, 0x15, 0x0392);
		MP_WritePhyUshort(sc, 0x19, 0x404e);
		MP_WritePhyUshort(sc, 0x15, 0x0393);
		MP_WritePhyUshort(sc, 0x19, 0x4461);
		MP_WritePhyUshort(sc, 0x15, 0x0394);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x0395);
		MP_WritePhyUshort(sc, 0x19, 0x9c0a);
		MP_WritePhyUshort(sc, 0x15, 0x0396);
		MP_WritePhyUshort(sc, 0x19, 0x63da);
		MP_WritePhyUshort(sc, 0x15, 0x0397);
		MP_WritePhyUshort(sc, 0x19, 0x6f0c);
		MP_WritePhyUshort(sc, 0x15, 0x0398);
		MP_WritePhyUshort(sc, 0x19, 0x5440);
		MP_WritePhyUshort(sc, 0x15, 0x0399);
		MP_WritePhyUshort(sc, 0x19, 0x4b98);
		MP_WritePhyUshort(sc, 0x15, 0x039a);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x039b);
		MP_WritePhyUshort(sc, 0x19, 0x4c00);
		MP_WritePhyUshort(sc, 0x15, 0x039c);
		MP_WritePhyUshort(sc, 0x19, 0x4b08);
		MP_WritePhyUshort(sc, 0x15, 0x039d);
		MP_WritePhyUshort(sc, 0x19, 0x63d8);
		MP_WritePhyUshort(sc, 0x15, 0x039e);
		MP_WritePhyUshort(sc, 0x19, 0x33a5);
		MP_WritePhyUshort(sc, 0x15, 0x039f);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x03a0);
		MP_WritePhyUshort(sc, 0x19, 0x00e8);
		MP_WritePhyUshort(sc, 0x15, 0x03a1);
		MP_WritePhyUshort(sc, 0x19, 0x820e);
		MP_WritePhyUshort(sc, 0x15, 0x03a2);
		MP_WritePhyUshort(sc, 0x19, 0xa10d);
		MP_WritePhyUshort(sc, 0x15, 0x03a3);
		MP_WritePhyUshort(sc, 0x19, 0x9df1);
		MP_WritePhyUshort(sc, 0x15, 0x03a4);
		MP_WritePhyUshort(sc, 0x19, 0x33af);
		MP_WritePhyUshort(sc, 0x15, 0x03a5);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x03a6);
		MP_WritePhyUshort(sc, 0x19, 0x00f9);
		MP_WritePhyUshort(sc, 0x15, 0x03a7);
		MP_WritePhyUshort(sc, 0x19, 0xc017);
		MP_WritePhyUshort(sc, 0x15, 0x03a8);
		MP_WritePhyUshort(sc, 0x19, 0x0007);
		MP_WritePhyUshort(sc, 0x15, 0x03a9);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x03aa);
		MP_WritePhyUshort(sc, 0x19, 0x6c03);
		MP_WritePhyUshort(sc, 0x15, 0x03ab);
		MP_WritePhyUshort(sc, 0x19, 0xa104);
		MP_WritePhyUshort(sc, 0x15, 0x03ac);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x03ad);
		MP_WritePhyUshort(sc, 0x19, 0x6c00);
		MP_WritePhyUshort(sc, 0x15, 0x03ae);
		MP_WritePhyUshort(sc, 0x19, 0x9df7);
		MP_WritePhyUshort(sc, 0x15, 0x03af);
		MP_WritePhyUshort(sc, 0x19, 0x7c03);
		MP_WritePhyUshort(sc, 0x15, 0x03b0);
		MP_WritePhyUshort(sc, 0x19, 0x6c08);
		MP_WritePhyUshort(sc, 0x15, 0x03b1);
		MP_WritePhyUshort(sc, 0x19, 0x33b6);
		MP_WritePhyUshort(sc, 0x15, 0x03b2);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03b3);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03b4);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03b5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03b6);
		MP_WritePhyUshort(sc, 0x19, 0x55af);
		MP_WritePhyUshort(sc, 0x15, 0x03b7);
		MP_WritePhyUshort(sc, 0x19, 0x7ff0);
		MP_WritePhyUshort(sc, 0x15, 0x03b8);
		MP_WritePhyUshort(sc, 0x19, 0x6ff0);
		MP_WritePhyUshort(sc, 0x15, 0x03b9);
		MP_WritePhyUshort(sc, 0x19, 0x4bb9);
		MP_WritePhyUshort(sc, 0x15, 0x03ba);
		MP_WritePhyUshort(sc, 0x19, 0x6a80);
		MP_WritePhyUshort(sc, 0x15, 0x03bb);
		MP_WritePhyUshort(sc, 0x19, 0x4b29);
		MP_WritePhyUshort(sc, 0x15, 0x03bc);
		MP_WritePhyUshort(sc, 0x19, 0x4041);
		MP_WritePhyUshort(sc, 0x15, 0x03bd);
		MP_WritePhyUshort(sc, 0x19, 0x440a);
		MP_WritePhyUshort(sc, 0x15, 0x03be);
		MP_WritePhyUshort(sc, 0x19, 0x4029);
		MP_WritePhyUshort(sc, 0x15, 0x03bf);
		MP_WritePhyUshort(sc, 0x19, 0x4418);
		MP_WritePhyUshort(sc, 0x15, 0x03c0);
		MP_WritePhyUshort(sc, 0x19, 0x4090);
		MP_WritePhyUshort(sc, 0x15, 0x03c1);
		MP_WritePhyUshort(sc, 0x19, 0x4438);
		MP_WritePhyUshort(sc, 0x15, 0x03c2);
		MP_WritePhyUshort(sc, 0x19, 0x40c4);
		MP_WritePhyUshort(sc, 0x15, 0x03c3);
		MP_WritePhyUshort(sc, 0x19, 0x447b);
		MP_WritePhyUshort(sc, 0x15, 0x03c4);
		MP_WritePhyUshort(sc, 0x19, 0xb6c4);
		MP_WritePhyUshort(sc, 0x15, 0x03c5);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x03c6);
		MP_WritePhyUshort(sc, 0x19, 0x9bfe);
		MP_WritePhyUshort(sc, 0x15, 0x03c7);
		MP_WritePhyUshort(sc, 0x19, 0x33cc);
		MP_WritePhyUshort(sc, 0x15, 0x03c8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03c9);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03ca);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03cb);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03cc);
		MP_WritePhyUshort(sc, 0x19, 0x542f);
		MP_WritePhyUshort(sc, 0x15, 0x03cd);
		MP_WritePhyUshort(sc, 0x19, 0x499a);
		MP_WritePhyUshort(sc, 0x15, 0x03ce);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x03cf);
		MP_WritePhyUshort(sc, 0x19, 0x4c40);
		MP_WritePhyUshort(sc, 0x15, 0x03d0);
		MP_WritePhyUshort(sc, 0x19, 0x490a);
		MP_WritePhyUshort(sc, 0x15, 0x03d1);
		MP_WritePhyUshort(sc, 0x19, 0x405e);
		MP_WritePhyUshort(sc, 0x15, 0x03d2);
		MP_WritePhyUshort(sc, 0x19, 0x44f8);
		MP_WritePhyUshort(sc, 0x15, 0x03d3);
		MP_WritePhyUshort(sc, 0x19, 0x6b00);
		MP_WritePhyUshort(sc, 0x15, 0x03d4);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x03d5);
		MP_WritePhyUshort(sc, 0x19, 0x0028);
		MP_WritePhyUshort(sc, 0x15, 0x03d6);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x03d7);
		MP_WritePhyUshort(sc, 0x19, 0xbd27);
		MP_WritePhyUshort(sc, 0x15, 0x03d8);
		MP_WritePhyUshort(sc, 0x19, 0x9cfc);
		MP_WritePhyUshort(sc, 0x15, 0x03d9);
		MP_WritePhyUshort(sc, 0x19, 0xc639);
		MP_WritePhyUshort(sc, 0x15, 0x03da);
		MP_WritePhyUshort(sc, 0x19, 0x000f);
		MP_WritePhyUshort(sc, 0x15, 0x03db);
		MP_WritePhyUshort(sc, 0x19, 0x9e03);
		MP_WritePhyUshort(sc, 0x15, 0x03dc);
		MP_WritePhyUshort(sc, 0x19, 0x7c01);
		MP_WritePhyUshort(sc, 0x15, 0x03dd);
		MP_WritePhyUshort(sc, 0x19, 0x4c01);
		MP_WritePhyUshort(sc, 0x15, 0x03de);
		MP_WritePhyUshort(sc, 0x19, 0x9af6);
		MP_WritePhyUshort(sc, 0x15, 0x03df);
		MP_WritePhyUshort(sc, 0x19, 0x7c12);
		MP_WritePhyUshort(sc, 0x15, 0x03e0);
		MP_WritePhyUshort(sc, 0x19, 0x4c52);
		MP_WritePhyUshort(sc, 0x15, 0x03e1);
		MP_WritePhyUshort(sc, 0x19, 0x4470);
		MP_WritePhyUshort(sc, 0x15, 0x03e2);
		MP_WritePhyUshort(sc, 0x19, 0x7c12);
		MP_WritePhyUshort(sc, 0x15, 0x03e3);
		MP_WritePhyUshort(sc, 0x19, 0x4c40);
		MP_WritePhyUshort(sc, 0x15, 0x03e4);
		MP_WritePhyUshort(sc, 0x19, 0x33d4);
		MP_WritePhyUshort(sc, 0x15, 0x03e5);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03e6);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03e7);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03e8);
		MP_WritePhyUshort(sc, 0x19, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x03e9);
		MP_WritePhyUshort(sc, 0x19, 0x49bb);
		MP_WritePhyUshort(sc, 0x15, 0x03ea);
		MP_WritePhyUshort(sc, 0x19, 0x4478);
		MP_WritePhyUshort(sc, 0x15, 0x03eb);
		MP_WritePhyUshort(sc, 0x19, 0x492b);
		MP_WritePhyUshort(sc, 0x15, 0x03ec);
		MP_WritePhyUshort(sc, 0x19, 0x6b80);
		MP_WritePhyUshort(sc, 0x15, 0x03ed);
		MP_WritePhyUshort(sc, 0x19, 0x7c01);
		MP_WritePhyUshort(sc, 0x15, 0x03ee);
		MP_WritePhyUshort(sc, 0x19, 0x4c00);
		MP_WritePhyUshort(sc, 0x15, 0x03ef);
		MP_WritePhyUshort(sc, 0x19, 0xd64f);
		MP_WritePhyUshort(sc, 0x15, 0x03f0);
		MP_WritePhyUshort(sc, 0x19, 0x000d);
		MP_WritePhyUshort(sc, 0x15, 0x03f1);
		MP_WritePhyUshort(sc, 0x19, 0x3311);
		MP_WritePhyUshort(sc, 0x15, 0x03f2);
		MP_WritePhyUshort(sc, 0x19, 0xbd0c);
		MP_WritePhyUshort(sc, 0x15, 0x03f3);
		MP_WritePhyUshort(sc, 0x19, 0xc428);
		MP_WritePhyUshort(sc, 0x15, 0x03f4);
		MP_WritePhyUshort(sc, 0x19, 0x0008);
		MP_WritePhyUshort(sc, 0x15, 0x03f5);
		MP_WritePhyUshort(sc, 0x19, 0x9afa);
		MP_WritePhyUshort(sc, 0x15, 0x03f6);
		MP_WritePhyUshort(sc, 0x19, 0x7c12);
		MP_WritePhyUshort(sc, 0x15, 0x03f7);
		MP_WritePhyUshort(sc, 0x19, 0x4c52);
		MP_WritePhyUshort(sc, 0x15, 0x03f8);
		MP_WritePhyUshort(sc, 0x19, 0x4470);
		MP_WritePhyUshort(sc, 0x15, 0x03f9);
		MP_WritePhyUshort(sc, 0x19, 0x7c12);
		MP_WritePhyUshort(sc, 0x15, 0x03fa);
		MP_WritePhyUshort(sc, 0x19, 0x4c40);
		MP_WritePhyUshort(sc, 0x15, 0x03fb);
		MP_WritePhyUshort(sc, 0x19, 0x33ef);
		MP_WritePhyUshort(sc, 0x15, 0x03fc);
		MP_WritePhyUshort(sc, 0x19, 0x3342);
		MP_WritePhyUshort(sc, 0x15, 0x03fd);
		MP_WritePhyUshort(sc, 0x19, 0x330d);
		MP_WritePhyUshort(sc, 0x15, 0x03fe);
		MP_WritePhyUshort(sc, 0x19, 0x32ae);
		MP_WritePhyUshort(sc, 0x15, 0x0000);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0300);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x05, 0x8000);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x48f7);
		MP_WritePhyUshort(sc, 0x06, 0x00e0);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xa080);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0xf602);
		MP_WritePhyUshort(sc, 0x06, 0x0112);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x1f02);
		MP_WritePhyUshort(sc, 0x06, 0x012c);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x3c02);
		MP_WritePhyUshort(sc, 0x06, 0x0156);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x6d02);
		MP_WritePhyUshort(sc, 0x06, 0x809d);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x88e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b89);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8a1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8b);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8c1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8e1e);
		MP_WritePhyUshort(sc, 0x06, 0x01a0);
		MP_WritePhyUshort(sc, 0x06, 0x00c7);
		MP_WritePhyUshort(sc, 0x06, 0xaebb);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xc702);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd105);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xcd02);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xca02);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd105);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xd002);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd481);
		MP_WritePhyUshort(sc, 0x06, 0xc9e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b90);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x91d4);
		MP_WritePhyUshort(sc, 0x06, 0x81b8);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x92e5);
		MP_WritePhyUshort(sc, 0x06, 0x8b93);
		MP_WritePhyUshort(sc, 0x06, 0xbf8b);
		MP_WritePhyUshort(sc, 0x06, 0x88ec);
		MP_WritePhyUshort(sc, 0x06, 0x0019);
		MP_WritePhyUshort(sc, 0x06, 0xa98b);
		MP_WritePhyUshort(sc, 0x06, 0x90f9);
		MP_WritePhyUshort(sc, 0x06, 0xeeff);
		MP_WritePhyUshort(sc, 0x06, 0xf600);
		MP_WritePhyUshort(sc, 0x06, 0xeeff);
		MP_WritePhyUshort(sc, 0x06, 0xf7fc);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xc102);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xc402);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x201a);
		MP_WritePhyUshort(sc, 0x06, 0xf620);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x824b);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0x1902);
		MP_WritePhyUshort(sc, 0x06, 0x2c9d);
		MP_WritePhyUshort(sc, 0x06, 0x0203);
		MP_WritePhyUshort(sc, 0x06, 0x9602);
		MP_WritePhyUshort(sc, 0x06, 0x0473);
		MP_WritePhyUshort(sc, 0x06, 0x022e);
		MP_WritePhyUshort(sc, 0x06, 0x3902);
		MP_WritePhyUshort(sc, 0x06, 0x044d);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x210b);
		MP_WritePhyUshort(sc, 0x06, 0xf621);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x0416);
		MP_WritePhyUshort(sc, 0x06, 0x021b);
		MP_WritePhyUshort(sc, 0x06, 0xa4e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad22);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x22e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2305);
		MP_WritePhyUshort(sc, 0x06, 0xf623);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad24);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x24e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2505);
		MP_WritePhyUshort(sc, 0x06, 0xf625);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad26);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x26e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0xdae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x27e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0203);
		MP_WritePhyUshort(sc, 0x06, 0x5cfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad21);
		MP_WritePhyUshort(sc, 0x06, 0x57e0);
		MP_WritePhyUshort(sc, 0x06, 0xe022);
		MP_WritePhyUshort(sc, 0x06, 0xe1e0);
		MP_WritePhyUshort(sc, 0x06, 0x2358);
		MP_WritePhyUshort(sc, 0x06, 0xc059);
		MP_WritePhyUshort(sc, 0x06, 0x021e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b3c);
		MP_WritePhyUshort(sc, 0x06, 0x1f10);
		MP_WritePhyUshort(sc, 0x06, 0x9e44);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x3cad);
		MP_WritePhyUshort(sc, 0x06, 0x211d);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x84f7);
		MP_WritePhyUshort(sc, 0x06, 0x29e5);
		MP_WritePhyUshort(sc, 0x06, 0x8b84);
		MP_WritePhyUshort(sc, 0x06, 0xac27);
		MP_WritePhyUshort(sc, 0x06, 0x0dac);
		MP_WritePhyUshort(sc, 0x06, 0x2605);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0x7fae);
		MP_WritePhyUshort(sc, 0x06, 0x2b02);
		MP_WritePhyUshort(sc, 0x06, 0x2c23);
		MP_WritePhyUshort(sc, 0x06, 0xae26);
		MP_WritePhyUshort(sc, 0x06, 0x022c);
		MP_WritePhyUshort(sc, 0x06, 0x41ae);
		MP_WritePhyUshort(sc, 0x06, 0x21e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xad22);
		MP_WritePhyUshort(sc, 0x06, 0x18e0);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0x58fc);
		MP_WritePhyUshort(sc, 0x06, 0xe4ff);
		MP_WritePhyUshort(sc, 0x06, 0xf7d1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x2eee);
		MP_WritePhyUshort(sc, 0x06, 0x0232);
		MP_WritePhyUshort(sc, 0x06, 0x0ad1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x82e8);
		MP_WritePhyUshort(sc, 0x06, 0x0232);
		MP_WritePhyUshort(sc, 0x06, 0x0a02);
		MP_WritePhyUshort(sc, 0x06, 0x2bdf);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfefc);
		MP_WritePhyUshort(sc, 0x06, 0x04d0);
		MP_WritePhyUshort(sc, 0x06, 0x0202);
		MP_WritePhyUshort(sc, 0x06, 0x1e97);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x2228);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xd302);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd10c);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xd602);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd104);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xd902);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xe802);
		MP_WritePhyUshort(sc, 0x06, 0x320a);
		MP_WritePhyUshort(sc, 0x06, 0xe0ff);
		MP_WritePhyUshort(sc, 0x06, 0xf768);
		MP_WritePhyUshort(sc, 0x06, 0x03e4);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xd004);
		MP_WritePhyUshort(sc, 0x06, 0x0228);
		MP_WritePhyUshort(sc, 0x06, 0x7a04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0xe234);
		MP_WritePhyUshort(sc, 0x06, 0xe1e2);
		MP_WritePhyUshort(sc, 0x06, 0x35f6);
		MP_WritePhyUshort(sc, 0x06, 0x2be4);
		MP_WritePhyUshort(sc, 0x06, 0xe234);
		MP_WritePhyUshort(sc, 0x06, 0xe5e2);
		MP_WritePhyUshort(sc, 0x06, 0x35fc);
		MP_WritePhyUshort(sc, 0x06, 0x05f8);
		MP_WritePhyUshort(sc, 0x06, 0xe0e2);
		MP_WritePhyUshort(sc, 0x06, 0x34e1);
		MP_WritePhyUshort(sc, 0x06, 0xe235);
		MP_WritePhyUshort(sc, 0x06, 0xf72b);
		MP_WritePhyUshort(sc, 0x06, 0xe4e2);
		MP_WritePhyUshort(sc, 0x06, 0x34e5);
		MP_WritePhyUshort(sc, 0x06, 0xe235);
		MP_WritePhyUshort(sc, 0x06, 0xfc05);
		MP_WritePhyUshort(sc, 0x06, 0xf8f9);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69ac);
		MP_WritePhyUshort(sc, 0x06, 0x1b4c);
		MP_WritePhyUshort(sc, 0x06, 0xbf2e);
		MP_WritePhyUshort(sc, 0x06, 0x3002);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0xef01);
		MP_WritePhyUshort(sc, 0x06, 0xe28a);
		MP_WritePhyUshort(sc, 0x06, 0x76e4);
		MP_WritePhyUshort(sc, 0x06, 0x8a76);
		MP_WritePhyUshort(sc, 0x06, 0x1f12);
		MP_WritePhyUshort(sc, 0x06, 0x9e3a);
		MP_WritePhyUshort(sc, 0x06, 0xef12);
		MP_WritePhyUshort(sc, 0x06, 0x5907);
		MP_WritePhyUshort(sc, 0x06, 0x9f12);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf721);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40d0);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0x287a);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0x34fc);
		MP_WritePhyUshort(sc, 0x06, 0xa000);
		MP_WritePhyUshort(sc, 0x06, 0x1002);
		MP_WritePhyUshort(sc, 0x06, 0x2dc3);
		MP_WritePhyUshort(sc, 0x06, 0x022e);
		MP_WritePhyUshort(sc, 0x06, 0x21e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf621);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40ae);
		MP_WritePhyUshort(sc, 0x06, 0x0fbf);
		MP_WritePhyUshort(sc, 0x06, 0x3fa5);
		MP_WritePhyUshort(sc, 0x06, 0x0231);
		MP_WritePhyUshort(sc, 0x06, 0x6cbf);
		MP_WritePhyUshort(sc, 0x06, 0x3fa2);
		MP_WritePhyUshort(sc, 0x06, 0x0231);
		MP_WritePhyUshort(sc, 0x06, 0x6c02);
		MP_WritePhyUshort(sc, 0x06, 0x2dc3);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfefd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0xe2f4);
		MP_WritePhyUshort(sc, 0x06, 0xe1e2);
		MP_WritePhyUshort(sc, 0x06, 0xf5e4);
		MP_WritePhyUshort(sc, 0x06, 0x8a78);
		MP_WritePhyUshort(sc, 0x06, 0xe58a);
		MP_WritePhyUshort(sc, 0x06, 0x79ee);
		MP_WritePhyUshort(sc, 0x06, 0xe2f4);
		MP_WritePhyUshort(sc, 0x06, 0xd8ee);
		MP_WritePhyUshort(sc, 0x06, 0xe2f5);
		MP_WritePhyUshort(sc, 0x06, 0x20fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x2065);
		MP_WritePhyUshort(sc, 0x06, 0xd200);
		MP_WritePhyUshort(sc, 0x06, 0xbf2e);
		MP_WritePhyUshort(sc, 0x06, 0xe802);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0x1e21);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xdf02);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0x0c11);
		MP_WritePhyUshort(sc, 0x06, 0x1e21);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xe202);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0x0c12);
		MP_WritePhyUshort(sc, 0x06, 0x1e21);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xe502);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0x0c13);
		MP_WritePhyUshort(sc, 0x06, 0x1e21);
		MP_WritePhyUshort(sc, 0x06, 0xbf1f);
		MP_WritePhyUshort(sc, 0x06, 0x5302);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0x0c14);
		MP_WritePhyUshort(sc, 0x06, 0x1e21);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xeb02);
		MP_WritePhyUshort(sc, 0x06, 0x31dd);
		MP_WritePhyUshort(sc, 0x06, 0x0c16);
		MP_WritePhyUshort(sc, 0x06, 0x1e21);
		MP_WritePhyUshort(sc, 0x06, 0xe083);
		MP_WritePhyUshort(sc, 0x06, 0xe01f);
		MP_WritePhyUshort(sc, 0x06, 0x029e);
		MP_WritePhyUshort(sc, 0x06, 0x22e6);
		MP_WritePhyUshort(sc, 0x06, 0x83e0);
		MP_WritePhyUshort(sc, 0x06, 0xad31);
		MP_WritePhyUshort(sc, 0x06, 0x14ad);
		MP_WritePhyUshort(sc, 0x06, 0x3011);
		MP_WritePhyUshort(sc, 0x06, 0xef02);
		MP_WritePhyUshort(sc, 0x06, 0x580c);
		MP_WritePhyUshort(sc, 0x06, 0x9e07);
		MP_WritePhyUshort(sc, 0x06, 0xad36);
		MP_WritePhyUshort(sc, 0x06, 0x085a);
		MP_WritePhyUshort(sc, 0x06, 0x309f);
		MP_WritePhyUshort(sc, 0x06, 0x04d1);
		MP_WritePhyUshort(sc, 0x06, 0x01ae);
		MP_WritePhyUshort(sc, 0x06, 0x02d1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x82dc);
		MP_WritePhyUshort(sc, 0x06, 0x0232);
		MP_WritePhyUshort(sc, 0x06, 0x0aef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x0400);
		MP_WritePhyUshort(sc, 0x06, 0xe140);
		MP_WritePhyUshort(sc, 0x06, 0x77e1);
		MP_WritePhyUshort(sc, 0x06, 0x4010);
		MP_WritePhyUshort(sc, 0x06, 0xe150);
		MP_WritePhyUshort(sc, 0x06, 0x32e1);
		MP_WritePhyUshort(sc, 0x06, 0x5030);
		MP_WritePhyUshort(sc, 0x06, 0xe144);
		MP_WritePhyUshort(sc, 0x06, 0x74e1);
		MP_WritePhyUshort(sc, 0x06, 0x44bb);
		MP_WritePhyUshort(sc, 0x06, 0xe2d2);
		MP_WritePhyUshort(sc, 0x06, 0x40e0);
		MP_WritePhyUshort(sc, 0x06, 0x2cfc);
		MP_WritePhyUshort(sc, 0x06, 0xe2cc);
		MP_WritePhyUshort(sc, 0x06, 0xcce2);
		MP_WritePhyUshort(sc, 0x06, 0x00cc);
		MP_WritePhyUshort(sc, 0x06, 0xe000);
		MP_WritePhyUshort(sc, 0x06, 0x99e0);
		MP_WritePhyUshort(sc, 0x06, 0x3688);
		MP_WritePhyUshort(sc, 0x06, 0xe036);
		MP_WritePhyUshort(sc, 0x06, 0x99e1);
		MP_WritePhyUshort(sc, 0x06, 0x40dd);
		MP_WritePhyUshort(sc, 0x06, 0xe022);
		MP_WritePhyUshort(sc, 0x05, 0xe142);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0xe140);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x00);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17);
		Data &= ~BIT_0;
		Data |= BIT_2;
		MP_WritePhyUshort(sc, 0x17, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B80);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_2 | BIT_1;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x18);
		Data |= BIT_4;
		MP_WritePhyUshort(sc, 0x18, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		Data = MP_ReadPhyUshort(sc, 0x14);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x14, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x15, 0x1006);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B86);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x0B, 0x6C14);
		MP_WritePhyUshort(sc, 0x14, 0x7F3D);
		MP_WritePhyUshort(sc, 0x1C, 0xFAFE);
		MP_WritePhyUshort(sc, 0x08, 0x07C5);
		MP_WritePhyUshort(sc, 0x10, 0xF090);
		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x14, 0x641A);
		MP_WritePhyUshort(sc, 0x1A, 0x0606);
		MP_WritePhyUshort(sc, 0x12, 0xF480);
		MP_WritePhyUshort(sc, 0x13, 0x0747);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0004);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x0078);
		MP_WritePhyUshort(sc, 0x15, 0xA408);
		MP_WritePhyUshort(sc, 0x17, 0x5100);
		MP_WritePhyUshort(sc, 0x19, 0x0008);
		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x0D, 0x0207);
		MP_WritePhyUshort(sc, 0x02, 0x5FD0);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0004);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x00A1);
		Data = MP_ReadPhyUshort(sc, 0x1A);
		Data &= ~BIT_2;
		MP_WritePhyUshort(sc, 0x1A, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0004);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x16);
		Data |= BIT_5;
		MP_WritePhyUshort(sc, 0x16, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0004);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x00AC);
		MP_WritePhyUshort(sc, 0x18, 0x0006);
		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B5B);
		MP_WritePhyUshort(sc, 0x06, 0x9222);
		MP_WritePhyUshort(sc, 0x05, 0x8B6D);
		MP_WritePhyUshort(sc, 0x06, 0x8000);
		MP_WritePhyUshort(sc, 0x05, 0x8B76);
		MP_WritePhyUshort(sc, 0x06, 0x8000);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_39) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~(BIT_12);
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x00, 0x4800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x002f);
		for (i = 0; i < 1000; i++) {
			if (MP_ReadPhyUshort(sc, 0x1c) & BIT_7)
				break;
			DELAY(100);
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		for (i = 0; i < 200; i++) {
			if ((MP_ReadPhyUshort(sc, 0x17) & BIT_0) == 0)
				break;
			DELAY(100);
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0307);
		MP_WritePhyUshort(sc, 0x15, 0x00AF);
		MP_WritePhyUshort(sc, 0x19, 0x4060);
		MP_WritePhyUshort(sc, 0x15, 0x00B0);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x00B1);
		MP_WritePhyUshort(sc, 0x19, 0x7e00);
		MP_WritePhyUshort(sc, 0x15, 0x00B2);
		MP_WritePhyUshort(sc, 0x19, 0x72B0);
		MP_WritePhyUshort(sc, 0x15, 0x00B3);
		MP_WritePhyUshort(sc, 0x19, 0x7F00);
		MP_WritePhyUshort(sc, 0x15, 0x00B4);
		MP_WritePhyUshort(sc, 0x19, 0x73B0);
		MP_WritePhyUshort(sc, 0x15, 0x0101);
		MP_WritePhyUshort(sc, 0x19, 0x0005);
		MP_WritePhyUshort(sc, 0x15, 0x0103);
		MP_WritePhyUshort(sc, 0x19, 0x0003);
		MP_WritePhyUshort(sc, 0x15, 0x0105);
		MP_WritePhyUshort(sc, 0x19, 0x30FD);
		MP_WritePhyUshort(sc, 0x15, 0x0106);
		MP_WritePhyUshort(sc, 0x19, 0x9DF7);
		MP_WritePhyUshort(sc, 0x15, 0x0107);
		MP_WritePhyUshort(sc, 0x19, 0x30C6);
		MP_WritePhyUshort(sc, 0x15, 0x0098);
		MP_WritePhyUshort(sc, 0x19, 0x7c0b);
		MP_WritePhyUshort(sc, 0x15, 0x0099);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00eb);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00f8);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00fe);
		MP_WritePhyUshort(sc, 0x19, 0x6f0f);
		MP_WritePhyUshort(sc, 0x15, 0x00db);
		MP_WritePhyUshort(sc, 0x19, 0x6f09);
		MP_WritePhyUshort(sc, 0x15, 0x00dc);
		MP_WritePhyUshort(sc, 0x19, 0xaefd);
		MP_WritePhyUshort(sc, 0x15, 0x00dd);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00de);
		MP_WritePhyUshort(sc, 0x19, 0xc60b);
		MP_WritePhyUshort(sc, 0x15, 0x00df);
		MP_WritePhyUshort(sc, 0x19, 0x00fa);
		MP_WritePhyUshort(sc, 0x15, 0x00e0);
		MP_WritePhyUshort(sc, 0x19, 0x30e1);
		MP_WritePhyUshort(sc, 0x15, 0x020c);
		MP_WritePhyUshort(sc, 0x19, 0x3224);
		MP_WritePhyUshort(sc, 0x15, 0x020e);
		MP_WritePhyUshort(sc, 0x19, 0x9813);
		MP_WritePhyUshort(sc, 0x15, 0x020f);
		MP_WritePhyUshort(sc, 0x19, 0x7801);
		MP_WritePhyUshort(sc, 0x15, 0x0210);
		MP_WritePhyUshort(sc, 0x19, 0x930f);
		MP_WritePhyUshort(sc, 0x15, 0x0211);
		MP_WritePhyUshort(sc, 0x19, 0x9206);
		MP_WritePhyUshort(sc, 0x15, 0x0212);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x0213);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0214);
		MP_WritePhyUshort(sc, 0x19, 0x588f);
		MP_WritePhyUshort(sc, 0x15, 0x0215);
		MP_WritePhyUshort(sc, 0x19, 0x5520);
		MP_WritePhyUshort(sc, 0x15, 0x0216);
		MP_WritePhyUshort(sc, 0x19, 0x3224);
		MP_WritePhyUshort(sc, 0x15, 0x0217);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x0218);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0219);
		MP_WritePhyUshort(sc, 0x19, 0x588d);
		MP_WritePhyUshort(sc, 0x15, 0x021a);
		MP_WritePhyUshort(sc, 0x19, 0x5540);
		MP_WritePhyUshort(sc, 0x15, 0x021b);
		MP_WritePhyUshort(sc, 0x19, 0x9e03);
		MP_WritePhyUshort(sc, 0x15, 0x021c);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x021d);
		MP_WritePhyUshort(sc, 0x19, 0x6840);
		MP_WritePhyUshort(sc, 0x15, 0x021e);
		MP_WritePhyUshort(sc, 0x19, 0x3224);
		MP_WritePhyUshort(sc, 0x15, 0x021f);
		MP_WritePhyUshort(sc, 0x19, 0x4002);
		MP_WritePhyUshort(sc, 0x15, 0x0220);
		MP_WritePhyUshort(sc, 0x19, 0x3224);
		MP_WritePhyUshort(sc, 0x15, 0x0221);
		MP_WritePhyUshort(sc, 0x19, 0x9e03);
		MP_WritePhyUshort(sc, 0x15, 0x0222);
		MP_WritePhyUshort(sc, 0x19, 0x7c40);
		MP_WritePhyUshort(sc, 0x15, 0x0223);
		MP_WritePhyUshort(sc, 0x19, 0x6840);
		MP_WritePhyUshort(sc, 0x15, 0x0224);
		MP_WritePhyUshort(sc, 0x19, 0x7800);
		MP_WritePhyUshort(sc, 0x15, 0x0225);
		MP_WritePhyUshort(sc, 0x19, 0x3231);
		MP_WritePhyUshort(sc, 0x15, 0x0000);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0300);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x17, 0x2160);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0040);
		MP_WritePhyUshort(sc, 0x18, 0x0004);
		MP_WritePhyUshort(sc, 0x18, 0x09d4);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x18, 0x09e4);
		MP_WritePhyUshort(sc, 0x19, 0x0800);
		MP_WritePhyUshort(sc, 0x18, 0x09f4);
		MP_WritePhyUshort(sc, 0x19, 0xff00);
		MP_WritePhyUshort(sc, 0x18, 0x0a04);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x18, 0x0a14);
		MP_WritePhyUshort(sc, 0x19, 0x0c00);
		MP_WritePhyUshort(sc, 0x18, 0x0a24);
		MP_WritePhyUshort(sc, 0x19, 0xff00);
		MP_WritePhyUshort(sc, 0x18, 0x0a74);
		MP_WritePhyUshort(sc, 0x19, 0xf600);
		MP_WritePhyUshort(sc, 0x18, 0x1a24);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x18, 0x1a64);
		MP_WritePhyUshort(sc, 0x19, 0x0500);
		MP_WritePhyUshort(sc, 0x18, 0x1a74);
		MP_WritePhyUshort(sc, 0x19, 0x9500);
		MP_WritePhyUshort(sc, 0x18, 0x1a84);
		MP_WritePhyUshort(sc, 0x19, 0x8000);
		MP_WritePhyUshort(sc, 0x18, 0x1a94);
		MP_WritePhyUshort(sc, 0x19, 0x7d00);
		MP_WritePhyUshort(sc, 0x18, 0x1aa4);
		MP_WritePhyUshort(sc, 0x19, 0x9600);
		MP_WritePhyUshort(sc, 0x18, 0x1ac4);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x18, 0x1ad4);
		MP_WritePhyUshort(sc, 0x19, 0x0800);
		MP_WritePhyUshort(sc, 0x18, 0x1af4);
		MP_WritePhyUshort(sc, 0x19, 0xc400);
		MP_WritePhyUshort(sc, 0x18, 0x1b04);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x18, 0x1b14);
		MP_WritePhyUshort(sc, 0x19, 0x0800);
		MP_WritePhyUshort(sc, 0x18, 0x1b24);
		MP_WritePhyUshort(sc, 0x19, 0xfd00);
		MP_WritePhyUshort(sc, 0x18, 0x1b34);
		MP_WritePhyUshort(sc, 0x19, 0x4000);
		MP_WritePhyUshort(sc, 0x18, 0x1b44);
		MP_WritePhyUshort(sc, 0x19, 0x0400);
		MP_WritePhyUshort(sc, 0x18, 0x1b94);
		MP_WritePhyUshort(sc, 0x19, 0xf100);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x17, 0x2100);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0040);
		MP_WritePhyUshort(sc, 0x18, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x05, 0x8000);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x48f7);
		MP_WritePhyUshort(sc, 0x06, 0x00e0);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xa080);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0xf602);
		MP_WritePhyUshort(sc, 0x06, 0x0115);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x2202);
		MP_WritePhyUshort(sc, 0x06, 0x80a0);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x3f02);
		MP_WritePhyUshort(sc, 0x06, 0x0159);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0xbd02);
		MP_WritePhyUshort(sc, 0x06, 0x80da);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x88e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b89);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8a1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8b);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8c1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8e1e);
		MP_WritePhyUshort(sc, 0x06, 0x01a0);
		MP_WritePhyUshort(sc, 0x06, 0x00c7);
		MP_WritePhyUshort(sc, 0x06, 0xaebb);
		MP_WritePhyUshort(sc, 0x06, 0xd481);
		MP_WritePhyUshort(sc, 0x06, 0xd2e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b92);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x93d1);
		MP_WritePhyUshort(sc, 0x06, 0x03bf);
		MP_WritePhyUshort(sc, 0x06, 0x859e);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23d1);
		MP_WritePhyUshort(sc, 0x06, 0x02bf);
		MP_WritePhyUshort(sc, 0x06, 0x85a1);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23ee);
		MP_WritePhyUshort(sc, 0x06, 0x8608);
		MP_WritePhyUshort(sc, 0x06, 0x03ee);
		MP_WritePhyUshort(sc, 0x06, 0x860a);
		MP_WritePhyUshort(sc, 0x06, 0x60ee);
		MP_WritePhyUshort(sc, 0x06, 0x8610);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0x8611);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0x8abe);
		MP_WritePhyUshort(sc, 0x06, 0x07ee);
		MP_WritePhyUshort(sc, 0x06, 0x8abf);
		MP_WritePhyUshort(sc, 0x06, 0x73ee);
		MP_WritePhyUshort(sc, 0x06, 0x8a95);
		MP_WritePhyUshort(sc, 0x06, 0x02bf);
		MP_WritePhyUshort(sc, 0x06, 0x8b88);
		MP_WritePhyUshort(sc, 0x06, 0xec00);
		MP_WritePhyUshort(sc, 0x06, 0x19a9);
		MP_WritePhyUshort(sc, 0x06, 0x8b90);
		MP_WritePhyUshort(sc, 0x06, 0xf9ee);
		MP_WritePhyUshort(sc, 0x06, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xfed1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x8595);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23d1);
		MP_WritePhyUshort(sc, 0x06, 0x01bf);
		MP_WritePhyUshort(sc, 0x06, 0x8598);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x2304);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8a);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x14ee);
		MP_WritePhyUshort(sc, 0x06, 0x8b8a);
		MP_WritePhyUshort(sc, 0x06, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x1f9a);
		MP_WritePhyUshort(sc, 0x06, 0xe0e4);
		MP_WritePhyUshort(sc, 0x06, 0x26e1);
		MP_WritePhyUshort(sc, 0x06, 0xe427);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x2623);
		MP_WritePhyUshort(sc, 0x06, 0xe5e4);
		MP_WritePhyUshort(sc, 0x06, 0x27fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8dad);
		MP_WritePhyUshort(sc, 0x06, 0x2014);
		MP_WritePhyUshort(sc, 0x06, 0xee8b);
		MP_WritePhyUshort(sc, 0x06, 0x8d00);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0x5a78);
		MP_WritePhyUshort(sc, 0x06, 0x039e);
		MP_WritePhyUshort(sc, 0x06, 0x0902);
		MP_WritePhyUshort(sc, 0x06, 0x05db);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0x7b02);
		MP_WritePhyUshort(sc, 0x06, 0x3231);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x1df6);
		MP_WritePhyUshort(sc, 0x06, 0x20e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0x5c02);
		MP_WritePhyUshort(sc, 0x06, 0x2bcb);
		MP_WritePhyUshort(sc, 0x06, 0x022d);
		MP_WritePhyUshort(sc, 0x06, 0x2902);
		MP_WritePhyUshort(sc, 0x06, 0x03b4);
		MP_WritePhyUshort(sc, 0x06, 0x0285);
		MP_WritePhyUshort(sc, 0x06, 0x6402);
		MP_WritePhyUshort(sc, 0x06, 0x2eca);
		MP_WritePhyUshort(sc, 0x06, 0x0284);
		MP_WritePhyUshort(sc, 0x06, 0xcd02);
		MP_WritePhyUshort(sc, 0x06, 0x046f);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x210b);
		MP_WritePhyUshort(sc, 0x06, 0xf621);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x8520);
		MP_WritePhyUshort(sc, 0x06, 0x021b);
		MP_WritePhyUshort(sc, 0x06, 0xe8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad22);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x22e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2308);
		MP_WritePhyUshort(sc, 0x06, 0xf623);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x311c);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2405);
		MP_WritePhyUshort(sc, 0x06, 0xf624);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad25);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x25e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2608);
		MP_WritePhyUshort(sc, 0x06, 0xf626);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x2df5);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2705);
		MP_WritePhyUshort(sc, 0x06, 0xf627);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x037a);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8f9);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x65d2);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x2fe9);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf61e);
		MP_WritePhyUshort(sc, 0x06, 0x21bf);
		MP_WritePhyUshort(sc, 0x06, 0x2ff5);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf60c);
		MP_WritePhyUshort(sc, 0x06, 0x111e);
		MP_WritePhyUshort(sc, 0x06, 0x21bf);
		MP_WritePhyUshort(sc, 0x06, 0x2ff8);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf60c);
		MP_WritePhyUshort(sc, 0x06, 0x121e);
		MP_WritePhyUshort(sc, 0x06, 0x21bf);
		MP_WritePhyUshort(sc, 0x06, 0x2ffb);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf60c);
		MP_WritePhyUshort(sc, 0x06, 0x131e);
		MP_WritePhyUshort(sc, 0x06, 0x21bf);
		MP_WritePhyUshort(sc, 0x06, 0x1f97);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf60c);
		MP_WritePhyUshort(sc, 0x06, 0x141e);
		MP_WritePhyUshort(sc, 0x06, 0x21bf);
		MP_WritePhyUshort(sc, 0x06, 0x859b);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf60c);
		MP_WritePhyUshort(sc, 0x06, 0x161e);
		MP_WritePhyUshort(sc, 0x06, 0x21e0);
		MP_WritePhyUshort(sc, 0x06, 0x8a8c);
		MP_WritePhyUshort(sc, 0x06, 0x1f02);
		MP_WritePhyUshort(sc, 0x06, 0x9e22);
		MP_WritePhyUshort(sc, 0x06, 0xe68a);
		MP_WritePhyUshort(sc, 0x06, 0x8cad);
		MP_WritePhyUshort(sc, 0x06, 0x3114);
		MP_WritePhyUshort(sc, 0x06, 0xad30);
		MP_WritePhyUshort(sc, 0x06, 0x11ef);
		MP_WritePhyUshort(sc, 0x06, 0x0258);
		MP_WritePhyUshort(sc, 0x06, 0x0c9e);
		MP_WritePhyUshort(sc, 0x06, 0x07ad);
		MP_WritePhyUshort(sc, 0x06, 0x3608);
		MP_WritePhyUshort(sc, 0x06, 0x5a30);
		MP_WritePhyUshort(sc, 0x06, 0x9f04);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xae02);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf2f);
		MP_WritePhyUshort(sc, 0x06, 0xf202);
		MP_WritePhyUshort(sc, 0x06, 0x3723);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfefd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8f9);
		MP_WritePhyUshort(sc, 0x06, 0xface);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69fa);
		MP_WritePhyUshort(sc, 0x06, 0xd401);
		MP_WritePhyUshort(sc, 0x06, 0x55b4);
		MP_WritePhyUshort(sc, 0x06, 0xfebf);
		MP_WritePhyUshort(sc, 0x06, 0x85a7);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf6ac);
		MP_WritePhyUshort(sc, 0x06, 0x280b);
		MP_WritePhyUshort(sc, 0x06, 0xbf85);
		MP_WritePhyUshort(sc, 0x06, 0xa402);
		MP_WritePhyUshort(sc, 0x06, 0x36f6);
		MP_WritePhyUshort(sc, 0x06, 0xac28);
		MP_WritePhyUshort(sc, 0x06, 0x49ae);
		MP_WritePhyUshort(sc, 0x06, 0x64bf);
		MP_WritePhyUshort(sc, 0x06, 0x85a4);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf6ac);
		MP_WritePhyUshort(sc, 0x06, 0x285b);
		MP_WritePhyUshort(sc, 0x06, 0xd000);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0x60ac);
		MP_WritePhyUshort(sc, 0x06, 0x2105);
		MP_WritePhyUshort(sc, 0x06, 0xac22);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x4ebf);
		MP_WritePhyUshort(sc, 0x06, 0xe0c4);
		MP_WritePhyUshort(sc, 0x06, 0xbe86);
		MP_WritePhyUshort(sc, 0x06, 0x14d2);
		MP_WritePhyUshort(sc, 0x06, 0x04d8);
		MP_WritePhyUshort(sc, 0x06, 0x19d9);
		MP_WritePhyUshort(sc, 0x06, 0x1907);
		MP_WritePhyUshort(sc, 0x06, 0xdc19);
		MP_WritePhyUshort(sc, 0x06, 0xdd19);
		MP_WritePhyUshort(sc, 0x06, 0x0789);
		MP_WritePhyUshort(sc, 0x06, 0x89ef);
		MP_WritePhyUshort(sc, 0x06, 0x645e);
		MP_WritePhyUshort(sc, 0x06, 0x07ff);
		MP_WritePhyUshort(sc, 0x06, 0x0d65);
		MP_WritePhyUshort(sc, 0x06, 0x5cf8);
		MP_WritePhyUshort(sc, 0x06, 0x001e);
		MP_WritePhyUshort(sc, 0x06, 0x46dc);
		MP_WritePhyUshort(sc, 0x06, 0x19dd);
		MP_WritePhyUshort(sc, 0x06, 0x19b2);
		MP_WritePhyUshort(sc, 0x06, 0xe2d4);
		MP_WritePhyUshort(sc, 0x06, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0xbf85);
		MP_WritePhyUshort(sc, 0x06, 0xa402);
		MP_WritePhyUshort(sc, 0x06, 0x3723);
		MP_WritePhyUshort(sc, 0x06, 0xae1d);
		MP_WritePhyUshort(sc, 0x06, 0xbee0);
		MP_WritePhyUshort(sc, 0x06, 0xc4bf);
		MP_WritePhyUshort(sc, 0x06, 0x8614);
		MP_WritePhyUshort(sc, 0x06, 0xd204);
		MP_WritePhyUshort(sc, 0x06, 0xd819);
		MP_WritePhyUshort(sc, 0x06, 0xd919);
		MP_WritePhyUshort(sc, 0x06, 0x07dc);
		MP_WritePhyUshort(sc, 0x06, 0x19dd);
		MP_WritePhyUshort(sc, 0x06, 0x1907);
		MP_WritePhyUshort(sc, 0x06, 0xb2f4);
		MP_WritePhyUshort(sc, 0x06, 0xd400);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x85a4);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23fe);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfec6);
		MP_WritePhyUshort(sc, 0x06, 0xfefd);
		MP_WritePhyUshort(sc, 0x06, 0xfc05);
		MP_WritePhyUshort(sc, 0x06, 0xf9e2);
		MP_WritePhyUshort(sc, 0x06, 0xe0ea);
		MP_WritePhyUshort(sc, 0x06, 0xe3e0);
		MP_WritePhyUshort(sc, 0x06, 0xeb5a);
		MP_WritePhyUshort(sc, 0x06, 0x070c);
		MP_WritePhyUshort(sc, 0x06, 0x031e);
		MP_WritePhyUshort(sc, 0x06, 0x20e6);
		MP_WritePhyUshort(sc, 0x06, 0xe0ea);
		MP_WritePhyUshort(sc, 0x06, 0xe7e0);
		MP_WritePhyUshort(sc, 0x06, 0xebe0);
		MP_WritePhyUshort(sc, 0x06, 0xe0fc);
		MP_WritePhyUshort(sc, 0x06, 0xe1e0);
		MP_WritePhyUshort(sc, 0x06, 0xfdfd);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xac26);
		MP_WritePhyUshort(sc, 0x06, 0x1ae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xac21);
		MP_WritePhyUshort(sc, 0x06, 0x14e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xac20);
		MP_WritePhyUshort(sc, 0x06, 0x0ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xac23);
		MP_WritePhyUshort(sc, 0x06, 0x08e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xac24);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x3802);
		MP_WritePhyUshort(sc, 0x06, 0x1ab5);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x1c04);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x1d04);
		MP_WritePhyUshort(sc, 0x06, 0xe2e0);
		MP_WritePhyUshort(sc, 0x06, 0x7ce3);
		MP_WritePhyUshort(sc, 0x06, 0xe07d);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x38e1);
		MP_WritePhyUshort(sc, 0x06, 0xe039);
		MP_WritePhyUshort(sc, 0x06, 0xad2e);
		MP_WritePhyUshort(sc, 0x06, 0x1bad);
		MP_WritePhyUshort(sc, 0x06, 0x390d);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf21);
		MP_WritePhyUshort(sc, 0x06, 0xd502);
		MP_WritePhyUshort(sc, 0x06, 0x3723);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0xd8ae);
		MP_WritePhyUshort(sc, 0x06, 0x0bac);
		MP_WritePhyUshort(sc, 0x06, 0x3802);
		MP_WritePhyUshort(sc, 0x06, 0xae06);
		MP_WritePhyUshort(sc, 0x06, 0x0283);
		MP_WritePhyUshort(sc, 0x06, 0x1802);
		MP_WritePhyUshort(sc, 0x06, 0x8360);
		MP_WritePhyUshort(sc, 0x06, 0x021a);
		MP_WritePhyUshort(sc, 0x06, 0xc6fd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e1);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2605);
		MP_WritePhyUshort(sc, 0x06, 0x0222);
		MP_WritePhyUshort(sc, 0x06, 0xa4f7);
		MP_WritePhyUshort(sc, 0x06, 0x28e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xad21);
		MP_WritePhyUshort(sc, 0x06, 0x0502);
		MP_WritePhyUshort(sc, 0x06, 0x23a9);
		MP_WritePhyUshort(sc, 0x06, 0xf729);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2005);
		MP_WritePhyUshort(sc, 0x06, 0x0214);
		MP_WritePhyUshort(sc, 0x06, 0xabf7);
		MP_WritePhyUshort(sc, 0x06, 0x2ae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad23);
		MP_WritePhyUshort(sc, 0x06, 0x0502);
		MP_WritePhyUshort(sc, 0x06, 0x12e7);
		MP_WritePhyUshort(sc, 0x06, 0xf72b);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x2405);
		MP_WritePhyUshort(sc, 0x06, 0x0283);
		MP_WritePhyUshort(sc, 0x06, 0xbcf7);
		MP_WritePhyUshort(sc, 0x06, 0x2ce5);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xad26);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0x21e5);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2109);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xf4ac);
		MP_WritePhyUshort(sc, 0x06, 0x2003);
		MP_WritePhyUshort(sc, 0x06, 0x0223);
		MP_WritePhyUshort(sc, 0x06, 0x98e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x09e0);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xac21);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0x13fb);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2309);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xf4ac);
		MP_WritePhyUshort(sc, 0x06, 0x2203);
		MP_WritePhyUshort(sc, 0x06, 0x0212);
		MP_WritePhyUshort(sc, 0x06, 0xfae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xad24);
		MP_WritePhyUshort(sc, 0x06, 0x09e0);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xac23);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0x83c1);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e1);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2608);
		MP_WritePhyUshort(sc, 0x06, 0xe083);
		MP_WritePhyUshort(sc, 0x06, 0xd2ad);
		MP_WritePhyUshort(sc, 0x06, 0x2502);
		MP_WritePhyUshort(sc, 0x06, 0xf628);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x210a);
		MP_WritePhyUshort(sc, 0x06, 0xe084);
		MP_WritePhyUshort(sc, 0x06, 0x0af6);
		MP_WritePhyUshort(sc, 0x06, 0x27a0);
		MP_WritePhyUshort(sc, 0x06, 0x0502);
		MP_WritePhyUshort(sc, 0x06, 0xf629);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2008);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xe8ad);
		MP_WritePhyUshort(sc, 0x06, 0x2102);
		MP_WritePhyUshort(sc, 0x06, 0xf62a);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2308);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x20a0);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0xf62b);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x2408);
		MP_WritePhyUshort(sc, 0x06, 0xe086);
		MP_WritePhyUshort(sc, 0x06, 0x02a0);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0xf62c);
		MP_WritePhyUshort(sc, 0x06, 0xe58a);
		MP_WritePhyUshort(sc, 0x06, 0xf4a1);
		MP_WritePhyUshort(sc, 0x06, 0x0008);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf21);
		MP_WritePhyUshort(sc, 0x06, 0xd502);
		MP_WritePhyUshort(sc, 0x06, 0x3723);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xee86);
		MP_WritePhyUshort(sc, 0x06, 0x0200);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x241e);
		MP_WritePhyUshort(sc, 0x06, 0xe086);
		MP_WritePhyUshort(sc, 0x06, 0x02a0);
		MP_WritePhyUshort(sc, 0x06, 0x0005);
		MP_WritePhyUshort(sc, 0x06, 0x0283);
		MP_WritePhyUshort(sc, 0x06, 0xe8ae);
		MP_WritePhyUshort(sc, 0x06, 0xf5a0);
		MP_WritePhyUshort(sc, 0x06, 0x0105);
		MP_WritePhyUshort(sc, 0x06, 0x0283);
		MP_WritePhyUshort(sc, 0x06, 0xf8ae);
		MP_WritePhyUshort(sc, 0x06, 0x0ba0);
		MP_WritePhyUshort(sc, 0x06, 0x0205);
		MP_WritePhyUshort(sc, 0x06, 0x0284);
		MP_WritePhyUshort(sc, 0x06, 0x14ae);
		MP_WritePhyUshort(sc, 0x06, 0x03a0);
		MP_WritePhyUshort(sc, 0x06, 0x0300);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0x0284);
		MP_WritePhyUshort(sc, 0x06, 0x2bee);
		MP_WritePhyUshort(sc, 0x06, 0x8602);
		MP_WritePhyUshort(sc, 0x06, 0x01ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8ee);
		MP_WritePhyUshort(sc, 0x06, 0x8609);
		MP_WritePhyUshort(sc, 0x06, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x8461);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xae10);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8608);
		MP_WritePhyUshort(sc, 0x06, 0xe186);
		MP_WritePhyUshort(sc, 0x06, 0x091f);
		MP_WritePhyUshort(sc, 0x06, 0x019e);
		MP_WritePhyUshort(sc, 0x06, 0x0611);
		MP_WritePhyUshort(sc, 0x06, 0xe586);
		MP_WritePhyUshort(sc, 0x06, 0x09ae);
		MP_WritePhyUshort(sc, 0x06, 0x04ee);
		MP_WritePhyUshort(sc, 0x06, 0x8602);
		MP_WritePhyUshort(sc, 0x06, 0x01fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xfbbf);
		MP_WritePhyUshort(sc, 0x06, 0x8604);
		MP_WritePhyUshort(sc, 0x06, 0xef79);
		MP_WritePhyUshort(sc, 0x06, 0xd200);
		MP_WritePhyUshort(sc, 0x06, 0xd400);
		MP_WritePhyUshort(sc, 0x06, 0x221e);
		MP_WritePhyUshort(sc, 0x06, 0x02bf);
		MP_WritePhyUshort(sc, 0x06, 0x2fec);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23bf);
		MP_WritePhyUshort(sc, 0x06, 0x13f2);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf60d);
		MP_WritePhyUshort(sc, 0x06, 0x4559);
		MP_WritePhyUshort(sc, 0x06, 0x1fef);
		MP_WritePhyUshort(sc, 0x06, 0x97dd);
		MP_WritePhyUshort(sc, 0x06, 0xd308);
		MP_WritePhyUshort(sc, 0x06, 0x1a93);
		MP_WritePhyUshort(sc, 0x06, 0xdd12);
		MP_WritePhyUshort(sc, 0x06, 0x17a2);
		MP_WritePhyUshort(sc, 0x06, 0x04de);
		MP_WritePhyUshort(sc, 0x06, 0xffef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xfbee);
		MP_WritePhyUshort(sc, 0x06, 0x8602);
		MP_WritePhyUshort(sc, 0x06, 0x03d5);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x06, 0xbf86);
		MP_WritePhyUshort(sc, 0x06, 0x04ef);
		MP_WritePhyUshort(sc, 0x06, 0x79ef);
		MP_WritePhyUshort(sc, 0x06, 0x45bf);
		MP_WritePhyUshort(sc, 0x06, 0x2fec);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23bf);
		MP_WritePhyUshort(sc, 0x06, 0x13f2);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf6ad);
		MP_WritePhyUshort(sc, 0x06, 0x2702);
		MP_WritePhyUshort(sc, 0x06, 0x78ff);
		MP_WritePhyUshort(sc, 0x06, 0xe186);
		MP_WritePhyUshort(sc, 0x06, 0x0a1b);
		MP_WritePhyUshort(sc, 0x06, 0x01aa);
		MP_WritePhyUshort(sc, 0x06, 0x2eef);
		MP_WritePhyUshort(sc, 0x06, 0x97d9);
		MP_WritePhyUshort(sc, 0x06, 0x7900);
		MP_WritePhyUshort(sc, 0x06, 0x9e2b);
		MP_WritePhyUshort(sc, 0x06, 0x81dd);
		MP_WritePhyUshort(sc, 0x06, 0xbf85);
		MP_WritePhyUshort(sc, 0x06, 0xad02);
		MP_WritePhyUshort(sc, 0x06, 0x3723);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xef02);
		MP_WritePhyUshort(sc, 0x06, 0x100c);
		MP_WritePhyUshort(sc, 0x06, 0x11b0);
		MP_WritePhyUshort(sc, 0x06, 0xfc0d);
		MP_WritePhyUshort(sc, 0x06, 0x11bf);
		MP_WritePhyUshort(sc, 0x06, 0x85aa);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23d1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x85aa);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x23ee);
		MP_WritePhyUshort(sc, 0x06, 0x8602);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x0413);
		MP_WritePhyUshort(sc, 0x06, 0xa38b);
		MP_WritePhyUshort(sc, 0x06, 0xb4d3);
		MP_WritePhyUshort(sc, 0x06, 0x8012);
		MP_WritePhyUshort(sc, 0x06, 0x17a2);
		MP_WritePhyUshort(sc, 0x06, 0x04ad);
		MP_WritePhyUshort(sc, 0x06, 0xffef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad25);
		MP_WritePhyUshort(sc, 0x06, 0x48e0);
		MP_WritePhyUshort(sc, 0x06, 0x8a96);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0x977c);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x9e35);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x9600);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x9700);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xbee1);
		MP_WritePhyUshort(sc, 0x06, 0x8abf);
		MP_WritePhyUshort(sc, 0x06, 0xe286);
		MP_WritePhyUshort(sc, 0x06, 0x10e3);
		MP_WritePhyUshort(sc, 0x06, 0x8611);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0x1aad);
		MP_WritePhyUshort(sc, 0x06, 0x2012);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x9603);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x97b7);
		MP_WritePhyUshort(sc, 0x06, 0xee86);
		MP_WritePhyUshort(sc, 0x06, 0x1000);
		MP_WritePhyUshort(sc, 0x06, 0xee86);
		MP_WritePhyUshort(sc, 0x06, 0x1100);
		MP_WritePhyUshort(sc, 0x06, 0xae11);
		MP_WritePhyUshort(sc, 0x06, 0x15e6);
		MP_WritePhyUshort(sc, 0x06, 0x8610);
		MP_WritePhyUshort(sc, 0x06, 0xe786);
		MP_WritePhyUshort(sc, 0x06, 0x11ae);
		MP_WritePhyUshort(sc, 0x06, 0x08ee);
		MP_WritePhyUshort(sc, 0x06, 0x8610);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0x8611);
		MP_WritePhyUshort(sc, 0x06, 0x00fd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x00e1);
		MP_WritePhyUshort(sc, 0x06, 0xe001);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x32e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf720);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40bf);
		MP_WritePhyUshort(sc, 0x06, 0x31f5);
		MP_WritePhyUshort(sc, 0x06, 0x0236);
		MP_WritePhyUshort(sc, 0x06, 0xf6ad);
		MP_WritePhyUshort(sc, 0x06, 0x2821);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x20e1);
		MP_WritePhyUshort(sc, 0x06, 0xe021);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x18e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf620);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40ee);
		MP_WritePhyUshort(sc, 0x06, 0x8b3b);
		MP_WritePhyUshort(sc, 0x06, 0xffe0);
		MP_WritePhyUshort(sc, 0x06, 0x8a8a);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0x8be4);
		MP_WritePhyUshort(sc, 0x06, 0xe000);
		MP_WritePhyUshort(sc, 0x06, 0xe5e0);
		MP_WritePhyUshort(sc, 0x06, 0x01ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x80ad);
		MP_WritePhyUshort(sc, 0x06, 0x2722);
		MP_WritePhyUshort(sc, 0x06, 0xbf44);
		MP_WritePhyUshort(sc, 0x06, 0xfc02);
		MP_WritePhyUshort(sc, 0x06, 0x36f6);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x441f);
		MP_WritePhyUshort(sc, 0x06, 0x019e);
		MP_WritePhyUshort(sc, 0x06, 0x15e5);
		MP_WritePhyUshort(sc, 0x06, 0x8b44);
		MP_WritePhyUshort(sc, 0x06, 0xad29);
		MP_WritePhyUshort(sc, 0x06, 0x07ac);
		MP_WritePhyUshort(sc, 0x06, 0x2804);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xae02);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf85);
		MP_WritePhyUshort(sc, 0x06, 0xb002);
		MP_WritePhyUshort(sc, 0x06, 0x3723);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfefc);
		MP_WritePhyUshort(sc, 0x06, 0x0400);
		MP_WritePhyUshort(sc, 0x06, 0xe140);
		MP_WritePhyUshort(sc, 0x06, 0x77e1);
		MP_WritePhyUshort(sc, 0x06, 0x40dd);
		MP_WritePhyUshort(sc, 0x06, 0xe022);
		MP_WritePhyUshort(sc, 0x06, 0x32e1);
		MP_WritePhyUshort(sc, 0x06, 0x5074);
		MP_WritePhyUshort(sc, 0x06, 0xe144);
		MP_WritePhyUshort(sc, 0x06, 0xffe0);
		MP_WritePhyUshort(sc, 0x06, 0xdaff);
		MP_WritePhyUshort(sc, 0x06, 0xe0c0);
		MP_WritePhyUshort(sc, 0x06, 0x52e0);
		MP_WritePhyUshort(sc, 0x06, 0xeed9);
		MP_WritePhyUshort(sc, 0x06, 0xe04c);
		MP_WritePhyUshort(sc, 0x06, 0xbbe0);
		MP_WritePhyUshort(sc, 0x06, 0x2a00);
		MP_WritePhyUshort(sc, 0x05, 0xe142);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0xe140);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x00);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0042);
		MP_WritePhyUshort(sc, 0x18, 0x2300);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17);
		MP_WritePhyUshort(sc, 0x17, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x9200);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B80);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_2 | BIT_1;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x18);
		Data |= BIT_4;
		MP_WritePhyUshort(sc, 0x18, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		Data = MP_ReadPhyUshort(sc, 0x14);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x14, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B86);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0004);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x00AC);
		MP_WritePhyUshort(sc, 0x18, 0x0006);
		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0003);
		MP_WritePhyUshort(sc, 0x09, 0xA20F);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_14;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B54);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8B5D);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7C);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7F);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A82);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A88);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		Data_u32 = re_eri_read(sc, 0x1b0, 4, ERIAR_ExGMAC);
		Data_u32 &= ~(BIT_0 | BIT_1);
		re_eri_write(sc, 0x1b0, 2, Data_u32, ERIAR_ExGMAC);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_13;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0020);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0002);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0007);
		MP_WritePhyUshort(sc, 0x0e, 0x003c);
		MP_WritePhyUshort(sc, 0x0d, 0x4007);
		MP_WritePhyUshort(sc, 0x0e, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		Data = MP_ReadPhyUshort(sc, 0x19);
		Data &= ~BIT_0;
		MP_WritePhyUshort(sc, 0x19, Data);
		Data = MP_ReadPhyUshort(sc, 0x10);
		Data &= ~BIT_10;
		MP_WritePhyUshort(sc, 0x10, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
	} else if (sc->re_type == MACFG_41) {
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x11, MP_ReadPhyUshort(sc, 0x11) | 0x1000);
		MP_WritePhyUshort(sc, 0x1F, 0x0002);
		MP_WritePhyUshort(sc, 0x0F, MP_ReadPhyUshort(sc, 0x0F) | 0x0003);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		for (Data_u32=0x800E0068; Data_u32<0x800E006D; Data_u32++)
		{
			CSR_WRITE_4(sc, 0xF8, Data_u32);
			for (i=0; i<10; i++)
			{
				DELAY(400);
				if ((CSR_READ_4(sc, 0xF8)&0x80000000)==0)
					break;
			}
		}
	} else if (sc->re_type == MACFG_42 || sc->re_type == MACFG_43) {
		CSR_WRITE_4(sc, RE_ERIAR, 0x000041D0);
		for (i=0; i<10; i++)
		{
			DELAY(400);
			if (CSR_READ_4(sc, RE_ERIAR)&0x80000000)
				break;
		}
		Data_u32 = CSR_READ_4(sc, RE_ERIDR) & 0xFFFF0000;
		Data_u32 |= 0x4D02;
		CSR_WRITE_4(sc, RE_ERIDR, Data_u32);
		CSR_WRITE_4(sc, RE_ERIAR, 0x000021D0);
		for (i=0; i<10; i++)
		{
			DELAY(400);
			if ((CSR_READ_4(sc, RE_ERIAR)&0x80000000)==0)
				break;
		}

		CSR_WRITE_4(sc, RE_ERIAR, 0x000041DC);
		for (i=0; i<10; i++)
		{
			DELAY(400);
			if (CSR_READ_4(sc, RE_ERIAR)&0x80000000)
				break;
		}
		Data_u32 = CSR_READ_4(sc, RE_ERIDR) & 0xFFFF0000;
		Data_u32 |= 0x0050;
		CSR_WRITE_4(sc, RE_ERIDR, Data_u32);
		CSR_WRITE_4(sc, RE_ERIAR, 0x000021DC);
		for (i=0; i<10; i++)
		{
			DELAY(400);
			if ((CSR_READ_4(sc, RE_ERIAR)&0x80000000)==0)
				break;
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x18, 0x8310);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x19, 0x7070);
		MP_WritePhyUshort(sc, 0x1c, 0x0600);
		MP_WritePhyUshort(sc, 0x1d, 0x9700);
		MP_WritePhyUshort(sc, 0x1d, 0x7d00);
		MP_WritePhyUshort(sc, 0x1d, 0x6900);
		MP_WritePhyUshort(sc, 0x1d, 0x7d00);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x4899);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x8000);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x4007);
		MP_WritePhyUshort(sc, 0x1d, 0x4400);
		MP_WritePhyUshort(sc, 0x1d, 0x4800);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x5310);
		MP_WritePhyUshort(sc, 0x1d, 0x6000);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x6736);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x571f);
		MP_WritePhyUshort(sc, 0x1d, 0x5ffb);
		MP_WritePhyUshort(sc, 0x1d, 0xaa03);
		MP_WritePhyUshort(sc, 0x1d, 0x5b58);
		MP_WritePhyUshort(sc, 0x1d, 0x301e);
		MP_WritePhyUshort(sc, 0x1d, 0x5b64);
		MP_WritePhyUshort(sc, 0x1d, 0xa6fc);
		MP_WritePhyUshort(sc, 0x1d, 0xdcdb);
		MP_WritePhyUshort(sc, 0x1d, 0x0014);
		MP_WritePhyUshort(sc, 0x1d, 0xd9a9);
		MP_WritePhyUshort(sc, 0x1d, 0x0013);
		MP_WritePhyUshort(sc, 0x1d, 0xd16b);
		MP_WritePhyUshort(sc, 0x1d, 0x0011);
		MP_WritePhyUshort(sc, 0x1d, 0xb40e);
		MP_WritePhyUshort(sc, 0x1d, 0xd06b);
		MP_WritePhyUshort(sc, 0x1d, 0x000c);
		MP_WritePhyUshort(sc, 0x1d, 0xb206);
		MP_WritePhyUshort(sc, 0x1d, 0x7c01);
		MP_WritePhyUshort(sc, 0x1d, 0x5800);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x5c00);
		MP_WritePhyUshort(sc, 0x1d, 0x301a);
		MP_WritePhyUshort(sc, 0x1d, 0x7c01);
		MP_WritePhyUshort(sc, 0x1d, 0x5801);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x5c04);
		MP_WritePhyUshort(sc, 0x1d, 0x301e);
		MP_WritePhyUshort(sc, 0x1d, 0x314d);
		MP_WritePhyUshort(sc, 0x1d, 0x31f0);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4c20);
		MP_WritePhyUshort(sc, 0x1d, 0x6004);
		MP_WritePhyUshort(sc, 0x1d, 0x5310);
		MP_WritePhyUshort(sc, 0x1d, 0x4833);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c08);
		MP_WritePhyUshort(sc, 0x1d, 0x8300);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x6600);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0xb90c);
		MP_WritePhyUshort(sc, 0x1d, 0x30d3);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4de0);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x6000);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x6736);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x5310);
		MP_WritePhyUshort(sc, 0x1d, 0x300b);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4c60);
		MP_WritePhyUshort(sc, 0x1d, 0x6803);
		MP_WritePhyUshort(sc, 0x1d, 0x6520);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0xaf03);
		MP_WritePhyUshort(sc, 0x1d, 0x6015);
		MP_WritePhyUshort(sc, 0x1d, 0x3059);
		MP_WritePhyUshort(sc, 0x1d, 0x6017);
		MP_WritePhyUshort(sc, 0x1d, 0x57e0);
		MP_WritePhyUshort(sc, 0x1d, 0x580c);
		MP_WritePhyUshort(sc, 0x1d, 0x588c);
		MP_WritePhyUshort(sc, 0x1d, 0x7ffc);
		MP_WritePhyUshort(sc, 0x1d, 0x5fa3);
		MP_WritePhyUshort(sc, 0x1d, 0x4827);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c10);
		MP_WritePhyUshort(sc, 0x1d, 0x8400);
		MP_WritePhyUshort(sc, 0x1d, 0x7c30);
		MP_WritePhyUshort(sc, 0x1d, 0x6020);
		MP_WritePhyUshort(sc, 0x1d, 0x48bf);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0xad09);
		MP_WritePhyUshort(sc, 0x1d, 0x7c03);
		MP_WritePhyUshort(sc, 0x1d, 0x5c03);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x4400);
		MP_WritePhyUshort(sc, 0x1d, 0xad2c);
		MP_WritePhyUshort(sc, 0x1d, 0xd6cf);
		MP_WritePhyUshort(sc, 0x1d, 0x0002);
		MP_WritePhyUshort(sc, 0x1d, 0x80f4);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4c80);
		MP_WritePhyUshort(sc, 0x1d, 0x7c20);
		MP_WritePhyUshort(sc, 0x1d, 0x5c20);
		MP_WritePhyUshort(sc, 0x1d, 0x481e);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c02);
		MP_WritePhyUshort(sc, 0x1d, 0xad0a);
		MP_WritePhyUshort(sc, 0x1d, 0x7c03);
		MP_WritePhyUshort(sc, 0x1d, 0x5c03);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x4400);
		MP_WritePhyUshort(sc, 0x1d, 0x5310);
		MP_WritePhyUshort(sc, 0x1d, 0x8d02);
		MP_WritePhyUshort(sc, 0x1d, 0x4401);
		MP_WritePhyUshort(sc, 0x1d, 0x81f4);
		MP_WritePhyUshort(sc, 0x1d, 0x3114);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4d00);
		MP_WritePhyUshort(sc, 0x1d, 0x4832);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c10);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x6000);
		MP_WritePhyUshort(sc, 0x1d, 0xa4b7);
		MP_WritePhyUshort(sc, 0x1d, 0xd9b3);
		MP_WritePhyUshort(sc, 0x1d, 0xfffe);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4d20);
		MP_WritePhyUshort(sc, 0x1d, 0x7e00);
		MP_WritePhyUshort(sc, 0x1d, 0x6200);
		MP_WritePhyUshort(sc, 0x1d, 0x3045);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4d40);
		MP_WritePhyUshort(sc, 0x1d, 0x7c40);
		MP_WritePhyUshort(sc, 0x1d, 0x6000);
		MP_WritePhyUshort(sc, 0x1d, 0x4401);
		MP_WritePhyUshort(sc, 0x1d, 0x5210);
		MP_WritePhyUshort(sc, 0x1d, 0x4833);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x4c08);
		MP_WritePhyUshort(sc, 0x1d, 0x8300);
		MP_WritePhyUshort(sc, 0x1d, 0x5f80);
		MP_WritePhyUshort(sc, 0x1d, 0x55e0);
		MP_WritePhyUshort(sc, 0x1d, 0xc06f);
		MP_WritePhyUshort(sc, 0x1d, 0x0005);
		MP_WritePhyUshort(sc, 0x1d, 0xd9b3);
		MP_WritePhyUshort(sc, 0x1d, 0xfffd);
		MP_WritePhyUshort(sc, 0x1d, 0x7c40);
		MP_WritePhyUshort(sc, 0x1d, 0x6040);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4d60);
		MP_WritePhyUshort(sc, 0x1d, 0x57e0);
		MP_WritePhyUshort(sc, 0x1d, 0x4814);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x4c04);
		MP_WritePhyUshort(sc, 0x1d, 0x8200);
		MP_WritePhyUshort(sc, 0x1d, 0x7c03);
		MP_WritePhyUshort(sc, 0x1d, 0x5c03);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0xad02);
		MP_WritePhyUshort(sc, 0x1d, 0x4400);
		MP_WritePhyUshort(sc, 0x1d, 0xc0e9);
		MP_WritePhyUshort(sc, 0x1d, 0x0003);
		MP_WritePhyUshort(sc, 0x1d, 0xadd8);
		MP_WritePhyUshort(sc, 0x1d, 0x30c6);
		MP_WritePhyUshort(sc, 0x1d, 0x3078);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4dc0);
		MP_WritePhyUshort(sc, 0x1d, 0x6730);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0xd09d);
		MP_WritePhyUshort(sc, 0x1d, 0x0002);
		MP_WritePhyUshort(sc, 0x1d, 0xb4fe);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4d80);
		MP_WritePhyUshort(sc, 0x1d, 0x6802);
		MP_WritePhyUshort(sc, 0x1d, 0x6600);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x6000);
		MP_WritePhyUshort(sc, 0x1d, 0x486c);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x9503);
		MP_WritePhyUshort(sc, 0x1d, 0x7e00);
		MP_WritePhyUshort(sc, 0x1d, 0x6200);
		MP_WritePhyUshort(sc, 0x1d, 0x571f);
		MP_WritePhyUshort(sc, 0x1d, 0x5fbb);
		MP_WritePhyUshort(sc, 0x1d, 0xaa03);
		MP_WritePhyUshort(sc, 0x1d, 0x5b58);
		MP_WritePhyUshort(sc, 0x1d, 0x30e9);
		MP_WritePhyUshort(sc, 0x1d, 0x5b64);
		MP_WritePhyUshort(sc, 0x1d, 0xcdab);
		MP_WritePhyUshort(sc, 0x1d, 0xff5b);
		MP_WritePhyUshort(sc, 0x1d, 0xcd8d);
		MP_WritePhyUshort(sc, 0x1d, 0xff59);
		MP_WritePhyUshort(sc, 0x1d, 0xd96b);
		MP_WritePhyUshort(sc, 0x1d, 0xff57);
		MP_WritePhyUshort(sc, 0x1d, 0xd0a0);
		MP_WritePhyUshort(sc, 0x1d, 0xffdb);
		MP_WritePhyUshort(sc, 0x1d, 0xcba0);
		MP_WritePhyUshort(sc, 0x1d, 0x0003);
		MP_WritePhyUshort(sc, 0x1d, 0x80f0);
		MP_WritePhyUshort(sc, 0x1d, 0x30f6);
		MP_WritePhyUshort(sc, 0x1d, 0x3109);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4ce0);
		MP_WritePhyUshort(sc, 0x1d, 0x7d30);
		MP_WritePhyUshort(sc, 0x1d, 0x6530);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x7ce0);
		MP_WritePhyUshort(sc, 0x1d, 0x5400);
		MP_WritePhyUshort(sc, 0x1d, 0x4832);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c08);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x6008);
		MP_WritePhyUshort(sc, 0x1d, 0x8300);
		MP_WritePhyUshort(sc, 0x1d, 0xb902);
		MP_WritePhyUshort(sc, 0x1d, 0x30d3);
		MP_WritePhyUshort(sc, 0x1d, 0x308f);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4da0);
		MP_WritePhyUshort(sc, 0x1d, 0x57a0);
		MP_WritePhyUshort(sc, 0x1d, 0x590c);
		MP_WritePhyUshort(sc, 0x1d, 0x5fa2);
		MP_WritePhyUshort(sc, 0x1d, 0xcba4);
		MP_WritePhyUshort(sc, 0x1d, 0x0005);
		MP_WritePhyUshort(sc, 0x1d, 0xcd8d);
		MP_WritePhyUshort(sc, 0x1d, 0x0003);
		MP_WritePhyUshort(sc, 0x1d, 0x80fc);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4ca0);
		MP_WritePhyUshort(sc, 0x1d, 0xb603);
		MP_WritePhyUshort(sc, 0x1d, 0x7c10);
		MP_WritePhyUshort(sc, 0x1d, 0x6010);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x541f);
		MP_WritePhyUshort(sc, 0x1d, 0x7ffc);
		MP_WritePhyUshort(sc, 0x1d, 0x5fb3);
		MP_WritePhyUshort(sc, 0x1d, 0x9403);
		MP_WritePhyUshort(sc, 0x1d, 0x7c03);
		MP_WritePhyUshort(sc, 0x1d, 0x5c03);
		MP_WritePhyUshort(sc, 0x1d, 0xaa05);
		MP_WritePhyUshort(sc, 0x1d, 0x7c80);
		MP_WritePhyUshort(sc, 0x1d, 0x5800);
		MP_WritePhyUshort(sc, 0x1d, 0x5b58);
		MP_WritePhyUshort(sc, 0x1d, 0x3128);
		MP_WritePhyUshort(sc, 0x1d, 0x7c80);
		MP_WritePhyUshort(sc, 0x1d, 0x5800);
		MP_WritePhyUshort(sc, 0x1d, 0x5b64);
		MP_WritePhyUshort(sc, 0x1d, 0x4827);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c10);
		MP_WritePhyUshort(sc, 0x1d, 0x8400);
		MP_WritePhyUshort(sc, 0x1d, 0x7c10);
		MP_WritePhyUshort(sc, 0x1d, 0x6000);
		MP_WritePhyUshort(sc, 0x1d, 0x4824);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c04);
		MP_WritePhyUshort(sc, 0x1d, 0x8200);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4cc0);
		MP_WritePhyUshort(sc, 0x1d, 0x7d00);
		MP_WritePhyUshort(sc, 0x1d, 0x6400);
		MP_WritePhyUshort(sc, 0x1d, 0x7ffc);
		MP_WritePhyUshort(sc, 0x1d, 0x5fbb);
		MP_WritePhyUshort(sc, 0x1d, 0x4824);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c04);
		MP_WritePhyUshort(sc, 0x1d, 0x8200);
		MP_WritePhyUshort(sc, 0x1d, 0x7e00);
		MP_WritePhyUshort(sc, 0x1d, 0x6a00);
		MP_WritePhyUshort(sc, 0x1d, 0x4824);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c00);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c04);
		MP_WritePhyUshort(sc, 0x1d, 0x8200);
		MP_WritePhyUshort(sc, 0x1d, 0x7e00);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x30f6);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4e00);
		MP_WritePhyUshort(sc, 0x1d, 0x4007);
		MP_WritePhyUshort(sc, 0x1d, 0x4400);
		MP_WritePhyUshort(sc, 0x1d, 0x5310);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x6736);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x570f);
		MP_WritePhyUshort(sc, 0x1d, 0x5fff);
		MP_WritePhyUshort(sc, 0x1d, 0xaa03);
		MP_WritePhyUshort(sc, 0x1d, 0x585b);
		MP_WritePhyUshort(sc, 0x1d, 0x315c);
		MP_WritePhyUshort(sc, 0x1d, 0x5867);
		MP_WritePhyUshort(sc, 0x1d, 0x9402);
		MP_WritePhyUshort(sc, 0x1d, 0x6200);
		MP_WritePhyUshort(sc, 0x1d, 0xcda3);
		MP_WritePhyUshort(sc, 0x1d, 0x009d);
		MP_WritePhyUshort(sc, 0x1d, 0xcd85);
		MP_WritePhyUshort(sc, 0x1d, 0x009b);
		MP_WritePhyUshort(sc, 0x1d, 0xd96b);
		MP_WritePhyUshort(sc, 0x1d, 0x0099);
		MP_WritePhyUshort(sc, 0x1d, 0x96e9);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x6736);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4e20);
		MP_WritePhyUshort(sc, 0x1d, 0x96e4);
		MP_WritePhyUshort(sc, 0x1d, 0x8b04);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x5008);
		MP_WritePhyUshort(sc, 0x1d, 0xab03);
		MP_WritePhyUshort(sc, 0x1d, 0x7c08);
		MP_WritePhyUshort(sc, 0x1d, 0x5000);
		MP_WritePhyUshort(sc, 0x1d, 0x6801);
		MP_WritePhyUshort(sc, 0x1d, 0x6776);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0xdb7c);
		MP_WritePhyUshort(sc, 0x1d, 0xfff0);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe1);
		MP_WritePhyUshort(sc, 0x1d, 0x4e40);
		MP_WritePhyUshort(sc, 0x1d, 0x4837);
		MP_WritePhyUshort(sc, 0x1d, 0x4418);
		MP_WritePhyUshort(sc, 0x1d, 0x41c7);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4e40);
		MP_WritePhyUshort(sc, 0x1d, 0x7c40);
		MP_WritePhyUshort(sc, 0x1d, 0x5400);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x8fc9);
		MP_WritePhyUshort(sc, 0x1d, 0xd2a0);
		MP_WritePhyUshort(sc, 0x1d, 0x004a);
		MP_WritePhyUshort(sc, 0x1d, 0x9203);
		MP_WritePhyUshort(sc, 0x1d, 0xa041);
		MP_WritePhyUshort(sc, 0x1d, 0x3184);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe1);
		MP_WritePhyUshort(sc, 0x1d, 0x4e60);
		MP_WritePhyUshort(sc, 0x1d, 0x489c);
		MP_WritePhyUshort(sc, 0x1d, 0x4628);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4e60);
		MP_WritePhyUshort(sc, 0x1d, 0x7e28);
		MP_WritePhyUshort(sc, 0x1d, 0x4628);
		MP_WritePhyUshort(sc, 0x1d, 0x7c40);
		MP_WritePhyUshort(sc, 0x1d, 0x5400);
		MP_WritePhyUshort(sc, 0x1d, 0x7c01);
		MP_WritePhyUshort(sc, 0x1d, 0x5800);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x5c00);
		MP_WritePhyUshort(sc, 0x1d, 0x41e8);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x8fb0);
		MP_WritePhyUshort(sc, 0x1d, 0xb241);
		MP_WritePhyUshort(sc, 0x1d, 0xa02a);
		MP_WritePhyUshort(sc, 0x1d, 0x319d);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4ea0);
		MP_WritePhyUshort(sc, 0x1d, 0x7c02);
		MP_WritePhyUshort(sc, 0x1d, 0x4402);
		MP_WritePhyUshort(sc, 0x1d, 0x4448);
		MP_WritePhyUshort(sc, 0x1d, 0x4894);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c03);
		MP_WritePhyUshort(sc, 0x1d, 0x4824);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c07);
		MP_WritePhyUshort(sc, 0x1d, 0x41ef);
		MP_WritePhyUshort(sc, 0x1d, 0x41ff);
		MP_WritePhyUshort(sc, 0x1d, 0x4891);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c07);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c17);
		MP_WritePhyUshort(sc, 0x1d, 0x8400);
		MP_WritePhyUshort(sc, 0x1d, 0x8ef8);
		MP_WritePhyUshort(sc, 0x1d, 0x41c7);
		MP_WritePhyUshort(sc, 0x1d, 0x8f95);
		MP_WritePhyUshort(sc, 0x1d, 0x92d5);
		MP_WritePhyUshort(sc, 0x1d, 0xa10f);
		MP_WritePhyUshort(sc, 0x1d, 0xd480);
		MP_WritePhyUshort(sc, 0x1d, 0x0008);
		MP_WritePhyUshort(sc, 0x1d, 0xd580);
		MP_WritePhyUshort(sc, 0x1d, 0xffb9);
		MP_WritePhyUshort(sc, 0x1d, 0xa202);
		MP_WritePhyUshort(sc, 0x1d, 0x31b8);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x4404);
		MP_WritePhyUshort(sc, 0x1d, 0x31b8);
		MP_WritePhyUshort(sc, 0x1d, 0xd484);
		MP_WritePhyUshort(sc, 0x1d, 0xfff3);
		MP_WritePhyUshort(sc, 0x1d, 0xd484);
		MP_WritePhyUshort(sc, 0x1d, 0xfff1);
		MP_WritePhyUshort(sc, 0x1d, 0x314d);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4ee0);
		MP_WritePhyUshort(sc, 0x1d, 0x7c40);
		MP_WritePhyUshort(sc, 0x1d, 0x5400);
		MP_WritePhyUshort(sc, 0x1d, 0x4488);
		MP_WritePhyUshort(sc, 0x1d, 0x41cf);
		MP_WritePhyUshort(sc, 0x1d, 0x314d);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4ec0);
		MP_WritePhyUshort(sc, 0x1d, 0x48f3);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c09);
		MP_WritePhyUshort(sc, 0x1d, 0x4508);
		MP_WritePhyUshort(sc, 0x1d, 0x41c7);
		MP_WritePhyUshort(sc, 0x1d, 0x8f24);
		MP_WritePhyUshort(sc, 0x1d, 0xd218);
		MP_WritePhyUshort(sc, 0x1d, 0x0022);
		MP_WritePhyUshort(sc, 0x1d, 0xd2a4);
		MP_WritePhyUshort(sc, 0x1d, 0xff9f);
		MP_WritePhyUshort(sc, 0x1d, 0x31d9);
		MP_WritePhyUshort(sc, 0x1d, 0x7fe0);
		MP_WritePhyUshort(sc, 0x1d, 0x4e80);
		MP_WritePhyUshort(sc, 0x1d, 0x4832);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c01);
		MP_WritePhyUshort(sc, 0x1d, 0x7c1f);
		MP_WritePhyUshort(sc, 0x1d, 0x4c11);
		MP_WritePhyUshort(sc, 0x1d, 0x4428);
		MP_WritePhyUshort(sc, 0x1d, 0x7c40);
		MP_WritePhyUshort(sc, 0x1d, 0x5440);
		MP_WritePhyUshort(sc, 0x1d, 0x7c01);
		MP_WritePhyUshort(sc, 0x1d, 0x5801);
		MP_WritePhyUshort(sc, 0x1d, 0x7c04);
		MP_WritePhyUshort(sc, 0x1d, 0x5c04);
		MP_WritePhyUshort(sc, 0x1d, 0x41e8);
		MP_WritePhyUshort(sc, 0x1d, 0xa4b3);
		MP_WritePhyUshort(sc, 0x1d, 0x31ee);
		MP_WritePhyUshort(sc, 0x1d, 0x6800);
		MP_WritePhyUshort(sc, 0x1d, 0x6736);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x0000);
		MP_WritePhyUshort(sc, 0x1d, 0x570f);
		MP_WritePhyUshort(sc, 0x1d, 0x5fff);
		MP_WritePhyUshort(sc, 0x1d, 0xaa03);
		MP_WritePhyUshort(sc, 0x1d, 0x585b);
		MP_WritePhyUshort(sc, 0x1d, 0x31fa);
		MP_WritePhyUshort(sc, 0x1d, 0x5867);
		MP_WritePhyUshort(sc, 0x1d, 0xbcf6);
		MP_WritePhyUshort(sc, 0x1d, 0x300b);
		MP_WritePhyUshort(sc, 0x1d, 0x300b);
		MP_WritePhyUshort(sc, 0x1d, 0x314d);
		MP_WritePhyUshort(sc, 0x1f, 0x0004);
		MP_WritePhyUshort(sc, 0x1c, 0x0200);
		MP_WritePhyUshort(sc, 0x19, 0x7030);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		if (CSR_READ_1(sc, 0xEF)&0x08)
		{
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x1A, 0x0004);
			MP_WritePhyUshort(sc, 0x1F, 0x0000);
		}
		else
		{
			MP_WritePhyUshort(sc, 0x1F, 0x0005);
			MP_WritePhyUshort(sc, 0x1A, 0x0000);
			MP_WritePhyUshort(sc, 0x1F, 0x0000);
		}

		if (CSR_READ_1(sc, 0xEF)&0x10)
		{
			MP_WritePhyUshort(sc, 0x1F, 0x0004);
			MP_WritePhyUshort(sc, 0x1C, 0x0000);
			MP_WritePhyUshort(sc, 0x1F, 0x0000);
		}
		else
		{
			MP_WritePhyUshort(sc, 0x1F, 0x0004);
			MP_WritePhyUshort(sc, 0x1C, 0x0200);
			MP_WritePhyUshort(sc, 0x1F, 0x0000);
		}

		MP_WritePhyUshort(sc, 0x1F, 0x0001);
		MP_WritePhyUshort(sc, 0x15, 0x7701);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0000);
		MP_WritePhyUshort(sc, 0x18, 0x8310);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);
	} else if (sc->re_type == MACFG_50) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~(BIT_12);
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x00, 0x4800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x002f);
		for (i = 0; i < 1000; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x1c);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x18);
			if (!(Data & BIT_0))
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0307);
		MP_WritePhyUshort(sc, 0x15, 0x0194);
		MP_WritePhyUshort(sc, 0x19, 0x407D);
		MP_WritePhyUshort(sc, 0x15, 0x0098);
		MP_WritePhyUshort(sc, 0x19, 0x7c0b);
		MP_WritePhyUshort(sc, 0x15, 0x0099);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00eb);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00f8);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00fe);
		MP_WritePhyUshort(sc, 0x19, 0x6f0f);
		MP_WritePhyUshort(sc, 0x15, 0x00db);
		MP_WritePhyUshort(sc, 0x19, 0x6f09);
		MP_WritePhyUshort(sc, 0x15, 0x00dc);
		MP_WritePhyUshort(sc, 0x19, 0xaefd);
		MP_WritePhyUshort(sc, 0x15, 0x00dd);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00de);
		MP_WritePhyUshort(sc, 0x19, 0xc60b);
		MP_WritePhyUshort(sc, 0x15, 0x00df);
		MP_WritePhyUshort(sc, 0x19, 0x00fa);
		MP_WritePhyUshort(sc, 0x15, 0x00e0);
		MP_WritePhyUshort(sc, 0x19, 0x30e1);
		MP_WritePhyUshort(sc, 0x15, 0x0000);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0300);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x05, 0x8000);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x48f7);
		MP_WritePhyUshort(sc, 0x06, 0x00e0);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xa080);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0xf602);
		MP_WritePhyUshort(sc, 0x06, 0x0118);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x2502);
		MP_WritePhyUshort(sc, 0x06, 0x8090);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x4202);
		MP_WritePhyUshort(sc, 0x06, 0x015c);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0xad02);
		MP_WritePhyUshort(sc, 0x06, 0x80ca);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x88e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b89);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8a1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8b);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8c1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8e1e);
		MP_WritePhyUshort(sc, 0x06, 0x01a0);
		MP_WritePhyUshort(sc, 0x06, 0x00c7);
		MP_WritePhyUshort(sc, 0x06, 0xaebb);
		MP_WritePhyUshort(sc, 0x06, 0xd484);
		MP_WritePhyUshort(sc, 0x06, 0x3ce4);
		MP_WritePhyUshort(sc, 0x06, 0x8b92);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x93ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac8);
		MP_WritePhyUshort(sc, 0x06, 0x03ee);
		MP_WritePhyUshort(sc, 0x06, 0x8aca);
		MP_WritePhyUshort(sc, 0x06, 0x60ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac0);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac1);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0x8abe);
		MP_WritePhyUshort(sc, 0x06, 0x07ee);
		MP_WritePhyUshort(sc, 0x06, 0x8abf);
		MP_WritePhyUshort(sc, 0x06, 0x73ee);
		MP_WritePhyUshort(sc, 0x06, 0x8a95);
		MP_WritePhyUshort(sc, 0x06, 0x02bf);
		MP_WritePhyUshort(sc, 0x06, 0x8b88);
		MP_WritePhyUshort(sc, 0x06, 0xec00);
		MP_WritePhyUshort(sc, 0x06, 0x19a9);
		MP_WritePhyUshort(sc, 0x06, 0x8b90);
		MP_WritePhyUshort(sc, 0x06, 0xf9ee);
		MP_WritePhyUshort(sc, 0x06, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xfed1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x8516);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7dd1);
		MP_WritePhyUshort(sc, 0x06, 0x01bf);
		MP_WritePhyUshort(sc, 0x06, 0x8519);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7d04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8a);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x14ee);
		MP_WritePhyUshort(sc, 0x06, 0x8b8a);
		MP_WritePhyUshort(sc, 0x06, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x204b);
		MP_WritePhyUshort(sc, 0x06, 0xe0e4);
		MP_WritePhyUshort(sc, 0x06, 0x26e1);
		MP_WritePhyUshort(sc, 0x06, 0xe427);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x2623);
		MP_WritePhyUshort(sc, 0x06, 0xe5e4);
		MP_WritePhyUshort(sc, 0x06, 0x27fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8dad);
		MP_WritePhyUshort(sc, 0x06, 0x2014);
		MP_WritePhyUshort(sc, 0x06, 0xee8b);
		MP_WritePhyUshort(sc, 0x06, 0x8d00);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0x5a78);
		MP_WritePhyUshort(sc, 0x06, 0x039e);
		MP_WritePhyUshort(sc, 0x06, 0x0902);
		MP_WritePhyUshort(sc, 0x06, 0x05e8);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0x4f02);
		MP_WritePhyUshort(sc, 0x06, 0x326c);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x1df6);
		MP_WritePhyUshort(sc, 0x06, 0x20e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x022f);
		MP_WritePhyUshort(sc, 0x06, 0x0902);
		MP_WritePhyUshort(sc, 0x06, 0x2ab0);
		MP_WritePhyUshort(sc, 0x06, 0x022c);
		MP_WritePhyUshort(sc, 0x06, 0x0e02);
		MP_WritePhyUshort(sc, 0x06, 0x03ba);
		MP_WritePhyUshort(sc, 0x06, 0x0284);
		MP_WritePhyUshort(sc, 0x06, 0xe502);
		MP_WritePhyUshort(sc, 0x06, 0x2df1);
		MP_WritePhyUshort(sc, 0x06, 0x0283);
		MP_WritePhyUshort(sc, 0x06, 0x8302);
		MP_WritePhyUshort(sc, 0x06, 0x0475);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x210b);
		MP_WritePhyUshort(sc, 0x06, 0xf621);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x83f8);
		MP_WritePhyUshort(sc, 0x06, 0x021c);
		MP_WritePhyUshort(sc, 0x06, 0x99e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad22);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x22e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0235);
		MP_WritePhyUshort(sc, 0x06, 0x63e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad23);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x23e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0231);
		MP_WritePhyUshort(sc, 0x06, 0x57e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad24);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x24e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2505);
		MP_WritePhyUshort(sc, 0x06, 0xf625);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad26);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x26e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x022d);
		MP_WritePhyUshort(sc, 0x06, 0x1ce0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x27e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0203);
		MP_WritePhyUshort(sc, 0x06, 0x80fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xac26);
		MP_WritePhyUshort(sc, 0x06, 0x1ae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xac21);
		MP_WritePhyUshort(sc, 0x06, 0x14e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xac20);
		MP_WritePhyUshort(sc, 0x06, 0x0ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xac23);
		MP_WritePhyUshort(sc, 0x06, 0x08e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xac24);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x3802);
		MP_WritePhyUshort(sc, 0x06, 0x1ac2);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x1c04);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x1d04);
		MP_WritePhyUshort(sc, 0x06, 0xe2e0);
		MP_WritePhyUshort(sc, 0x06, 0x7ce3);
		MP_WritePhyUshort(sc, 0x06, 0xe07d);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x38e1);
		MP_WritePhyUshort(sc, 0x06, 0xe039);
		MP_WritePhyUshort(sc, 0x06, 0xad2e);
		MP_WritePhyUshort(sc, 0x06, 0x1bad);
		MP_WritePhyUshort(sc, 0x06, 0x390d);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf22);
		MP_WritePhyUshort(sc, 0x06, 0x7a02);
		MP_WritePhyUshort(sc, 0x06, 0x387d);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0xacae);
		MP_WritePhyUshort(sc, 0x06, 0x0bac);
		MP_WritePhyUshort(sc, 0x06, 0x3802);
		MP_WritePhyUshort(sc, 0x06, 0xae06);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0xe902);
		MP_WritePhyUshort(sc, 0x06, 0x822e);
		MP_WritePhyUshort(sc, 0x06, 0x021a);
		MP_WritePhyUshort(sc, 0x06, 0xd3fd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e1);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2602);
		MP_WritePhyUshort(sc, 0x06, 0xf728);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2105);
		MP_WritePhyUshort(sc, 0x06, 0x0222);
		MP_WritePhyUshort(sc, 0x06, 0x8ef7);
		MP_WritePhyUshort(sc, 0x06, 0x29e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x0502);
		MP_WritePhyUshort(sc, 0x06, 0x14b8);
		MP_WritePhyUshort(sc, 0x06, 0xf72a);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2305);
		MP_WritePhyUshort(sc, 0x06, 0x0212);
		MP_WritePhyUshort(sc, 0x06, 0xf4f7);
		MP_WritePhyUshort(sc, 0x06, 0x2be0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xad24);
		MP_WritePhyUshort(sc, 0x06, 0x0502);
		MP_WritePhyUshort(sc, 0x06, 0x8284);
		MP_WritePhyUshort(sc, 0x06, 0xf72c);
		MP_WritePhyUshort(sc, 0x06, 0xe58a);
		MP_WritePhyUshort(sc, 0x06, 0xf4fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2600);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2109);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xf4ac);
		MP_WritePhyUshort(sc, 0x06, 0x2003);
		MP_WritePhyUshort(sc, 0x06, 0x0222);
		MP_WritePhyUshort(sc, 0x06, 0x7de0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x09e0);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xac21);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0x1408);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2309);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xf4ac);
		MP_WritePhyUshort(sc, 0x06, 0x2203);
		MP_WritePhyUshort(sc, 0x06, 0x0213);
		MP_WritePhyUshort(sc, 0x06, 0x07e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xad24);
		MP_WritePhyUshort(sc, 0x06, 0x09e0);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xac23);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0x8289);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e1);
		MP_WritePhyUshort(sc, 0x06, 0x8af4);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x2602);
		MP_WritePhyUshort(sc, 0x06, 0xf628);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ad);
		MP_WritePhyUshort(sc, 0x06, 0x210a);
		MP_WritePhyUshort(sc, 0x06, 0xe083);
		MP_WritePhyUshort(sc, 0x06, 0xecf6);
		MP_WritePhyUshort(sc, 0x06, 0x27a0);
		MP_WritePhyUshort(sc, 0x06, 0x0502);
		MP_WritePhyUshort(sc, 0x06, 0xf629);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2008);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xe8ad);
		MP_WritePhyUshort(sc, 0x06, 0x2102);
		MP_WritePhyUshort(sc, 0x06, 0xf62a);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ad);
		MP_WritePhyUshort(sc, 0x06, 0x2308);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x20a0);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0xf62b);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x2408);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xc2a0);
		MP_WritePhyUshort(sc, 0x06, 0x0302);
		MP_WritePhyUshort(sc, 0x06, 0xf62c);
		MP_WritePhyUshort(sc, 0x06, 0xe58a);
		MP_WritePhyUshort(sc, 0x06, 0xf4a1);
		MP_WritePhyUshort(sc, 0x06, 0x0008);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf22);
		MP_WritePhyUshort(sc, 0x06, 0x7a02);
		MP_WritePhyUshort(sc, 0x06, 0x387d);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0xc200);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ad);
		MP_WritePhyUshort(sc, 0x06, 0x241e);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xc2a0);
		MP_WritePhyUshort(sc, 0x06, 0x0005);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0xb0ae);
		MP_WritePhyUshort(sc, 0x06, 0xf5a0);
		MP_WritePhyUshort(sc, 0x06, 0x0105);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0xc0ae);
		MP_WritePhyUshort(sc, 0x06, 0x0ba0);
		MP_WritePhyUshort(sc, 0x06, 0x0205);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0xcaae);
		MP_WritePhyUshort(sc, 0x06, 0x03a0);
		MP_WritePhyUshort(sc, 0x06, 0x0300);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0xe1ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac2);
		MP_WritePhyUshort(sc, 0x06, 0x01ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac9);
		MP_WritePhyUshort(sc, 0x06, 0x0002);
		MP_WritePhyUshort(sc, 0x06, 0x8317);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8ac8);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0xc91f);
		MP_WritePhyUshort(sc, 0x06, 0x019e);
		MP_WritePhyUshort(sc, 0x06, 0x0611);
		MP_WritePhyUshort(sc, 0x06, 0xe58a);
		MP_WritePhyUshort(sc, 0x06, 0xc9ae);
		MP_WritePhyUshort(sc, 0x06, 0x04ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac2);
		MP_WritePhyUshort(sc, 0x06, 0x01fc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xfbbf);
		MP_WritePhyUshort(sc, 0x06, 0x8ac4);
		MP_WritePhyUshort(sc, 0x06, 0xef79);
		MP_WritePhyUshort(sc, 0x06, 0xd200);
		MP_WritePhyUshort(sc, 0x06, 0xd400);
		MP_WritePhyUshort(sc, 0x06, 0x221e);
		MP_WritePhyUshort(sc, 0x06, 0x02bf);
		MP_WritePhyUshort(sc, 0x06, 0x3024);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7dbf);
		MP_WritePhyUshort(sc, 0x06, 0x13ff);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x500d);
		MP_WritePhyUshort(sc, 0x06, 0x4559);
		MP_WritePhyUshort(sc, 0x06, 0x1fef);
		MP_WritePhyUshort(sc, 0x06, 0x97dd);
		MP_WritePhyUshort(sc, 0x06, 0xd308);
		MP_WritePhyUshort(sc, 0x06, 0x1a93);
		MP_WritePhyUshort(sc, 0x06, 0xdd12);
		MP_WritePhyUshort(sc, 0x06, 0x17a2);
		MP_WritePhyUshort(sc, 0x06, 0x04de);
		MP_WritePhyUshort(sc, 0x06, 0xffef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xfbee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac2);
		MP_WritePhyUshort(sc, 0x06, 0x03d5);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x06, 0xbf8a);
		MP_WritePhyUshort(sc, 0x06, 0xc4ef);
		MP_WritePhyUshort(sc, 0x06, 0x79ef);
		MP_WritePhyUshort(sc, 0x06, 0x45bf);
		MP_WritePhyUshort(sc, 0x06, 0x3024);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7dbf);
		MP_WritePhyUshort(sc, 0x06, 0x13ff);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x50ad);
		MP_WritePhyUshort(sc, 0x06, 0x2702);
		MP_WritePhyUshort(sc, 0x06, 0x78ff);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0xca1b);
		MP_WritePhyUshort(sc, 0x06, 0x01aa);
		MP_WritePhyUshort(sc, 0x06, 0x2eef);
		MP_WritePhyUshort(sc, 0x06, 0x97d9);
		MP_WritePhyUshort(sc, 0x06, 0x7900);
		MP_WritePhyUshort(sc, 0x06, 0x9e2b);
		MP_WritePhyUshort(sc, 0x06, 0x81dd);
		MP_WritePhyUshort(sc, 0x06, 0xbf85);
		MP_WritePhyUshort(sc, 0x06, 0x1f02);
		MP_WritePhyUshort(sc, 0x06, 0x387d);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xef02);
		MP_WritePhyUshort(sc, 0x06, 0x100c);
		MP_WritePhyUshort(sc, 0x06, 0x11b0);
		MP_WritePhyUshort(sc, 0x06, 0xfc0d);
		MP_WritePhyUshort(sc, 0x06, 0x11bf);
		MP_WritePhyUshort(sc, 0x06, 0x851c);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7dd1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x851c);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7dee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac2);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x0413);
		MP_WritePhyUshort(sc, 0x06, 0xa38b);
		MP_WritePhyUshort(sc, 0x06, 0xb4d3);
		MP_WritePhyUshort(sc, 0x06, 0x8012);
		MP_WritePhyUshort(sc, 0x06, 0x17a2);
		MP_WritePhyUshort(sc, 0x06, 0x04ad);
		MP_WritePhyUshort(sc, 0x06, 0xffef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xad25);
		MP_WritePhyUshort(sc, 0x06, 0x48e0);
		MP_WritePhyUshort(sc, 0x06, 0x8a96);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0x977c);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x9e35);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x9600);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x9700);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0xbee1);
		MP_WritePhyUshort(sc, 0x06, 0x8abf);
		MP_WritePhyUshort(sc, 0x06, 0xe28a);
		MP_WritePhyUshort(sc, 0x06, 0xc0e3);
		MP_WritePhyUshort(sc, 0x06, 0x8ac1);
		MP_WritePhyUshort(sc, 0x06, 0x0237);
		MP_WritePhyUshort(sc, 0x06, 0x74ad);
		MP_WritePhyUshort(sc, 0x06, 0x2012);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x9603);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0x97b7);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0xc000);
		MP_WritePhyUshort(sc, 0x06, 0xee8a);
		MP_WritePhyUshort(sc, 0x06, 0xc100);
		MP_WritePhyUshort(sc, 0x06, 0xae11);
		MP_WritePhyUshort(sc, 0x06, 0x15e6);
		MP_WritePhyUshort(sc, 0x06, 0x8ac0);
		MP_WritePhyUshort(sc, 0x06, 0xe78a);
		MP_WritePhyUshort(sc, 0x06, 0xc1ae);
		MP_WritePhyUshort(sc, 0x06, 0x08ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac0);
		MP_WritePhyUshort(sc, 0x06, 0x00ee);
		MP_WritePhyUshort(sc, 0x06, 0x8ac1);
		MP_WritePhyUshort(sc, 0x06, 0x00fd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xae20);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x06, 0xf8fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x00e1);
		MP_WritePhyUshort(sc, 0x06, 0xe001);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x32e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf720);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40bf);
		MP_WritePhyUshort(sc, 0x06, 0x3230);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x50ad);
		MP_WritePhyUshort(sc, 0x06, 0x2821);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x20e1);
		MP_WritePhyUshort(sc, 0x06, 0xe021);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x18e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf620);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40ee);
		MP_WritePhyUshort(sc, 0x06, 0x8b3b);
		MP_WritePhyUshort(sc, 0x06, 0xffe0);
		MP_WritePhyUshort(sc, 0x06, 0x8a8a);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0x8be4);
		MP_WritePhyUshort(sc, 0x06, 0xe000);
		MP_WritePhyUshort(sc, 0x06, 0xe5e0);
		MP_WritePhyUshort(sc, 0x06, 0x01ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8f9);
		MP_WritePhyUshort(sc, 0x06, 0xface);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69fa);
		MP_WritePhyUshort(sc, 0x06, 0xd401);
		MP_WritePhyUshort(sc, 0x06, 0x55b4);
		MP_WritePhyUshort(sc, 0x06, 0xfebf);
		MP_WritePhyUshort(sc, 0x06, 0x1c1e);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x50ac);
		MP_WritePhyUshort(sc, 0x06, 0x280b);
		MP_WritePhyUshort(sc, 0x06, 0xbf1c);
		MP_WritePhyUshort(sc, 0x06, 0x1b02);
		MP_WritePhyUshort(sc, 0x06, 0x3850);
		MP_WritePhyUshort(sc, 0x06, 0xac28);
		MP_WritePhyUshort(sc, 0x06, 0x49ae);
		MP_WritePhyUshort(sc, 0x06, 0x64bf);
		MP_WritePhyUshort(sc, 0x06, 0x1c1b);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x50ac);
		MP_WritePhyUshort(sc, 0x06, 0x285b);
		MP_WritePhyUshort(sc, 0x06, 0xd000);
		MP_WritePhyUshort(sc, 0x06, 0x0284);
		MP_WritePhyUshort(sc, 0x06, 0xcaac);
		MP_WritePhyUshort(sc, 0x06, 0x2105);
		MP_WritePhyUshort(sc, 0x06, 0xac22);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x4ebf);
		MP_WritePhyUshort(sc, 0x06, 0xe0c4);
		MP_WritePhyUshort(sc, 0x06, 0xbe85);
		MP_WritePhyUshort(sc, 0x06, 0xf6d2);
		MP_WritePhyUshort(sc, 0x06, 0x04d8);
		MP_WritePhyUshort(sc, 0x06, 0x19d9);
		MP_WritePhyUshort(sc, 0x06, 0x1907);
		MP_WritePhyUshort(sc, 0x06, 0xdc19);
		MP_WritePhyUshort(sc, 0x06, 0xdd19);
		MP_WritePhyUshort(sc, 0x06, 0x0789);
		MP_WritePhyUshort(sc, 0x06, 0x89ef);
		MP_WritePhyUshort(sc, 0x06, 0x645e);
		MP_WritePhyUshort(sc, 0x06, 0x07ff);
		MP_WritePhyUshort(sc, 0x06, 0x0d65);
		MP_WritePhyUshort(sc, 0x06, 0x5cf8);
		MP_WritePhyUshort(sc, 0x06, 0x001e);
		MP_WritePhyUshort(sc, 0x06, 0x46dc);
		MP_WritePhyUshort(sc, 0x06, 0x19dd);
		MP_WritePhyUshort(sc, 0x06, 0x19b2);
		MP_WritePhyUshort(sc, 0x06, 0xe2d4);
		MP_WritePhyUshort(sc, 0x06, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0xbf1c);
		MP_WritePhyUshort(sc, 0x06, 0x1b02);
		MP_WritePhyUshort(sc, 0x06, 0x387d);
		MP_WritePhyUshort(sc, 0x06, 0xae1d);
		MP_WritePhyUshort(sc, 0x06, 0xbee0);
		MP_WritePhyUshort(sc, 0x06, 0xc4bf);
		MP_WritePhyUshort(sc, 0x06, 0x85f6);
		MP_WritePhyUshort(sc, 0x06, 0xd204);
		MP_WritePhyUshort(sc, 0x06, 0xd819);
		MP_WritePhyUshort(sc, 0x06, 0xd919);
		MP_WritePhyUshort(sc, 0x06, 0x07dc);
		MP_WritePhyUshort(sc, 0x06, 0x19dd);
		MP_WritePhyUshort(sc, 0x06, 0x1907);
		MP_WritePhyUshort(sc, 0x06, 0xb2f4);
		MP_WritePhyUshort(sc, 0x06, 0xd400);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x1c1b);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7dfe);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfec6);
		MP_WritePhyUshort(sc, 0x06, 0xfefd);
		MP_WritePhyUshort(sc, 0x06, 0xfc05);
		MP_WritePhyUshort(sc, 0x06, 0xf9e2);
		MP_WritePhyUshort(sc, 0x06, 0xe0ea);
		MP_WritePhyUshort(sc, 0x06, 0xe3e0);
		MP_WritePhyUshort(sc, 0x06, 0xeb5a);
		MP_WritePhyUshort(sc, 0x06, 0x070c);
		MP_WritePhyUshort(sc, 0x06, 0x031e);
		MP_WritePhyUshort(sc, 0x06, 0x20e6);
		MP_WritePhyUshort(sc, 0x06, 0xe0ea);
		MP_WritePhyUshort(sc, 0x06, 0xe7e0);
		MP_WritePhyUshort(sc, 0x06, 0xebe0);
		MP_WritePhyUshort(sc, 0x06, 0xe0fc);
		MP_WritePhyUshort(sc, 0x06, 0xe1e0);
		MP_WritePhyUshort(sc, 0x06, 0xfdfd);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b80);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x22bf);
		MP_WritePhyUshort(sc, 0x06, 0x4616);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x50e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b44);
		MP_WritePhyUshort(sc, 0x06, 0x1f01);
		MP_WritePhyUshort(sc, 0x06, 0x9e15);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x44ad);
		MP_WritePhyUshort(sc, 0x06, 0x2907);
		MP_WritePhyUshort(sc, 0x06, 0xac28);
		MP_WritePhyUshort(sc, 0x06, 0x04d1);
		MP_WritePhyUshort(sc, 0x06, 0x01ae);
		MP_WritePhyUshort(sc, 0x06, 0x02d1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x8522);
		MP_WritePhyUshort(sc, 0x06, 0x0238);
		MP_WritePhyUshort(sc, 0x06, 0x7def);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0x00e1);
		MP_WritePhyUshort(sc, 0x06, 0x4077);
		MP_WritePhyUshort(sc, 0x06, 0xe140);
		MP_WritePhyUshort(sc, 0x06, 0x52e0);
		MP_WritePhyUshort(sc, 0x06, 0xeed9);
		MP_WritePhyUshort(sc, 0x06, 0xe04c);
		MP_WritePhyUshort(sc, 0x06, 0xbbe0);
		MP_WritePhyUshort(sc, 0x06, 0x2a00);
		MP_WritePhyUshort(sc, 0x05, 0xe142);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0xe140);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x00);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17);
		Data |= BIT_1;
		MP_WritePhyUshort(sc, 0x17, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		MP_WritePhyUshort(sc, 0x09, 0xA20F);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		MP_WritePhyUshort(sc, 0x01, 0x328A);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		Data = MP_ReadPhyUshort(sc, 0x19);
		Data &= ~BIT_0;
		MP_WritePhyUshort(sc, 0x19, Data);
		Data = MP_ReadPhyUshort(sc, 0x10);
		Data &= ~BIT_10;
		MP_WritePhyUshort(sc, 0x10, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x9200);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B80);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_2 | BIT_1;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x18);
		Data |= BIT_4;
		MP_WritePhyUshort(sc, 0x18, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		Data = MP_ReadPhyUshort(sc, 0x14);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x14, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B86);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_14;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B55);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x05, 0x8B5E);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x05, 0x8B67);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x05, 0x8B70);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x0078);
		MP_WritePhyUshort(sc, 0x17, 0x0000);
		MP_WritePhyUshort(sc, 0x19, 0x00FB);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B79);
		MP_WritePhyUshort(sc, 0x06, 0xAA00);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B54);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8B5D);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7C);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7F);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A82);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A88);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		Data_u32 = re_eri_read(sc, 0x1b0, 4, ERIAR_ExGMAC);
		Data_u32 &= ~(BIT_0 | BIT_1);
		re_eri_write(sc, 0x1b0, 2, Data_u32, ERIAR_ExGMAC);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_13;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0020);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0007);
		MP_WritePhyUshort(sc, 0x0e, 0x003c);
		MP_WritePhyUshort(sc, 0x0d, 0x4007);
		MP_WritePhyUshort(sc, 0x0e, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0000);
	} else if (sc->re_type == MACFG_51) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~(BIT_12);
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x00, 0x9800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x002f);
		for (i = 0; i < 1000; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x1c);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0307);
		MP_WritePhyUshort(sc, 0x15, 0x0098);
		MP_WritePhyUshort(sc, 0x19, 0x7c0b);
		MP_WritePhyUshort(sc, 0x15, 0x0099);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00eb);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00f8);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00fe);
		MP_WritePhyUshort(sc, 0x19, 0x6f0f);
		MP_WritePhyUshort(sc, 0x15, 0x00db);
		MP_WritePhyUshort(sc, 0x19, 0x6f09);
		MP_WritePhyUshort(sc, 0x15, 0x00dc);
		MP_WritePhyUshort(sc, 0x19, 0xaefd);
		MP_WritePhyUshort(sc, 0x15, 0x00dd);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00de);
		MP_WritePhyUshort(sc, 0x19, 0xc60b);
		MP_WritePhyUshort(sc, 0x15, 0x00df);
		MP_WritePhyUshort(sc, 0x19, 0x00fa);
		MP_WritePhyUshort(sc, 0x15, 0x00e0);
		MP_WritePhyUshort(sc, 0x19, 0x30e1);
		MP_WritePhyUshort(sc, 0x15, 0x0000);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0300);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x05, 0x8000);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x48f7);
		MP_WritePhyUshort(sc, 0x06, 0x00e0);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xa080);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0xf602);
		MP_WritePhyUshort(sc, 0x06, 0x011b);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x2802);
		MP_WritePhyUshort(sc, 0x06, 0x0135);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x4502);
		MP_WritePhyUshort(sc, 0x06, 0x015f);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x6b02);
		MP_WritePhyUshort(sc, 0x06, 0x80e5);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x88e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b89);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8a1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8b);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8c1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8e1e);
		MP_WritePhyUshort(sc, 0x06, 0x01a0);
		MP_WritePhyUshort(sc, 0x06, 0x00c7);
		MP_WritePhyUshort(sc, 0x06, 0xaebb);
		MP_WritePhyUshort(sc, 0x06, 0xbf8b);
		MP_WritePhyUshort(sc, 0x06, 0x88ec);
		MP_WritePhyUshort(sc, 0x06, 0x0019);
		MP_WritePhyUshort(sc, 0x06, 0xa98b);
		MP_WritePhyUshort(sc, 0x06, 0x90f9);
		MP_WritePhyUshort(sc, 0x06, 0xeeff);
		MP_WritePhyUshort(sc, 0x06, 0xf600);
		MP_WritePhyUshort(sc, 0x06, 0xeeff);
		MP_WritePhyUshort(sc, 0x06, 0xf7fe);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf81);
		MP_WritePhyUshort(sc, 0x06, 0x9802);
		MP_WritePhyUshort(sc, 0x06, 0x39f3);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf81);
		MP_WritePhyUshort(sc, 0x06, 0x9b02);
		MP_WritePhyUshort(sc, 0x06, 0x39f3);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8dad);
		MP_WritePhyUshort(sc, 0x06, 0x2014);
		MP_WritePhyUshort(sc, 0x06, 0xee8b);
		MP_WritePhyUshort(sc, 0x06, 0x8d00);
		MP_WritePhyUshort(sc, 0x06, 0xe08a);
		MP_WritePhyUshort(sc, 0x06, 0x5a78);
		MP_WritePhyUshort(sc, 0x06, 0x039e);
		MP_WritePhyUshort(sc, 0x06, 0x0902);
		MP_WritePhyUshort(sc, 0x06, 0x05fc);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x8802);
		MP_WritePhyUshort(sc, 0x06, 0x32dd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8f9);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ac);
		MP_WritePhyUshort(sc, 0x06, 0x261a);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x81ac);
		MP_WritePhyUshort(sc, 0x06, 0x2114);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ac);
		MP_WritePhyUshort(sc, 0x06, 0x200e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x85ac);
		MP_WritePhyUshort(sc, 0x06, 0x2308);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x87ac);
		MP_WritePhyUshort(sc, 0x06, 0x2402);
		MP_WritePhyUshort(sc, 0x06, 0xae38);
		MP_WritePhyUshort(sc, 0x06, 0x021a);
		MP_WritePhyUshort(sc, 0x06, 0xd6ee);
		MP_WritePhyUshort(sc, 0x06, 0xe41c);
		MP_WritePhyUshort(sc, 0x06, 0x04ee);
		MP_WritePhyUshort(sc, 0x06, 0xe41d);
		MP_WritePhyUshort(sc, 0x06, 0x04e2);
		MP_WritePhyUshort(sc, 0x06, 0xe07c);
		MP_WritePhyUshort(sc, 0x06, 0xe3e0);
		MP_WritePhyUshort(sc, 0x06, 0x7de0);
		MP_WritePhyUshort(sc, 0x06, 0xe038);
		MP_WritePhyUshort(sc, 0x06, 0xe1e0);
		MP_WritePhyUshort(sc, 0x06, 0x39ad);
		MP_WritePhyUshort(sc, 0x06, 0x2e1b);
		MP_WritePhyUshort(sc, 0x06, 0xad39);
		MP_WritePhyUshort(sc, 0x06, 0x0dd1);
		MP_WritePhyUshort(sc, 0x06, 0x01bf);
		MP_WritePhyUshort(sc, 0x06, 0x22c8);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xf302);
		MP_WritePhyUshort(sc, 0x06, 0x21f0);
		MP_WritePhyUshort(sc, 0x06, 0xae0b);
		MP_WritePhyUshort(sc, 0x06, 0xac38);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x0602);
		MP_WritePhyUshort(sc, 0x06, 0x222d);
		MP_WritePhyUshort(sc, 0x06, 0x0222);
		MP_WritePhyUshort(sc, 0x06, 0x7202);
		MP_WritePhyUshort(sc, 0x06, 0x1ae7);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x201a);
		MP_WritePhyUshort(sc, 0x06, 0xf620);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x2afe);
		MP_WritePhyUshort(sc, 0x06, 0x022c);
		MP_WritePhyUshort(sc, 0x06, 0x5c02);
		MP_WritePhyUshort(sc, 0x06, 0x03c5);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0x6702);
		MP_WritePhyUshort(sc, 0x06, 0x2e4f);
		MP_WritePhyUshort(sc, 0x06, 0x0204);
		MP_WritePhyUshort(sc, 0x06, 0x8902);
		MP_WritePhyUshort(sc, 0x06, 0x2f7a);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x210b);
		MP_WritePhyUshort(sc, 0x06, 0xf621);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x0445);
		MP_WritePhyUshort(sc, 0x06, 0x021c);
		MP_WritePhyUshort(sc, 0x06, 0xb8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad22);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x22e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0235);
		MP_WritePhyUshort(sc, 0x06, 0xd4e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad23);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x23e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0231);
		MP_WritePhyUshort(sc, 0x06, 0xc8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad24);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x24e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2505);
		MP_WritePhyUshort(sc, 0x06, 0xf625);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad26);
		MP_WritePhyUshort(sc, 0x06, 0x08f6);
		MP_WritePhyUshort(sc, 0x06, 0x26e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x022d);
		MP_WritePhyUshort(sc, 0x06, 0x6ae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x27e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0203);
		MP_WritePhyUshort(sc, 0x06, 0x8bfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b80);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x22bf);
		MP_WritePhyUshort(sc, 0x06, 0x479a);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xc6e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b44);
		MP_WritePhyUshort(sc, 0x06, 0x1f01);
		MP_WritePhyUshort(sc, 0x06, 0x9e15);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x44ad);
		MP_WritePhyUshort(sc, 0x06, 0x2907);
		MP_WritePhyUshort(sc, 0x06, 0xac28);
		MP_WritePhyUshort(sc, 0x06, 0x04d1);
		MP_WritePhyUshort(sc, 0x06, 0x01ae);
		MP_WritePhyUshort(sc, 0x06, 0x02d1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x819e);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xf3ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0x00e1);
		MP_WritePhyUshort(sc, 0x06, 0x4077);
		MP_WritePhyUshort(sc, 0x06, 0xe140);
		MP_WritePhyUshort(sc, 0x06, 0xbbe0);
		MP_WritePhyUshort(sc, 0x06, 0x2a00);
		MP_WritePhyUshort(sc, 0x05, 0xe142);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0xe140);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17);
		Data |= BIT_1;
		MP_WritePhyUshort(sc, 0x17, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		Data = MP_ReadPhyUshort(sc, 0x19);
		Data &= ~BIT_0;
		MP_WritePhyUshort(sc, 0x19, Data);
		Data = MP_ReadPhyUshort(sc, 0x10);
		Data &= ~BIT_10;
		MP_WritePhyUshort(sc, 0x10, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x9200);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B80);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_2 | BIT_1;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x18);
		Data |= BIT_4;
		MP_WritePhyUshort(sc, 0x18, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		Data = MP_ReadPhyUshort(sc, 0x14);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x14, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B86);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B54);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8B5D);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7C);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7F);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A82);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A88);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		Data_u32 = re_eri_read(sc, 0x1b0, 4, ERIAR_ExGMAC);
		Data_u32 &= ~(BIT_0 | BIT_1);
		re_eri_write(sc, 0x1b0, 2, Data_u32, ERIAR_ExGMAC);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_13;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0020);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0007);
		MP_WritePhyUshort(sc, 0x0e, 0x003c);
		MP_WritePhyUshort(sc, 0x0d, 0x4007);
		MP_WritePhyUshort(sc, 0x0e, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0000);
	} else if (sc->re_type == MACFG_52) {
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~(BIT_12);
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x00, 0x4800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x002f);
		for (i = 0; i < 1000; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x1c);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x1800);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x18);
			if (!(Data & BIT_0))
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0307);
		MP_WritePhyUshort(sc, 0x15, 0x0098);
		MP_WritePhyUshort(sc, 0x19, 0x7c0b);
		MP_WritePhyUshort(sc, 0x15, 0x0099);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00eb);
		MP_WritePhyUshort(sc, 0x19, 0x6c0b);
		MP_WritePhyUshort(sc, 0x15, 0x00f8);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00fe);
		MP_WritePhyUshort(sc, 0x19, 0x6f0f);
		MP_WritePhyUshort(sc, 0x15, 0x00db);
		MP_WritePhyUshort(sc, 0x19, 0x6f09);
		MP_WritePhyUshort(sc, 0x15, 0x00dc);
		MP_WritePhyUshort(sc, 0x19, 0xaefd);
		MP_WritePhyUshort(sc, 0x15, 0x00dd);
		MP_WritePhyUshort(sc, 0x19, 0x6f0b);
		MP_WritePhyUshort(sc, 0x15, 0x00de);
		MP_WritePhyUshort(sc, 0x19, 0xc60b);
		MP_WritePhyUshort(sc, 0x15, 0x00df);
		MP_WritePhyUshort(sc, 0x19, 0x00fa);
		MP_WritePhyUshort(sc, 0x15, 0x00e0);
		MP_WritePhyUshort(sc, 0x19, 0x30e1);
		MP_WritePhyUshort(sc, 0x15, 0x0000);
		MP_WritePhyUshort(sc, 0x16, 0x0306);
		MP_WritePhyUshort(sc, 0x16, 0x0300);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0xfff6);
		MP_WritePhyUshort(sc, 0x06, 0x0080);
		MP_WritePhyUshort(sc, 0x05, 0x8000);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x48f7);
		MP_WritePhyUshort(sc, 0x06, 0x00e0);
		MP_WritePhyUshort(sc, 0x06, 0xfff7);
		MP_WritePhyUshort(sc, 0x06, 0xa080);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0xf602);
		MP_WritePhyUshort(sc, 0x06, 0x011e);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x2b02);
		MP_WritePhyUshort(sc, 0x06, 0x8077);
		MP_WritePhyUshort(sc, 0x06, 0x0201);
		MP_WritePhyUshort(sc, 0x06, 0x4802);
		MP_WritePhyUshort(sc, 0x06, 0x0162);
		MP_WritePhyUshort(sc, 0x06, 0x0280);
		MP_WritePhyUshort(sc, 0x06, 0x9402);
		MP_WritePhyUshort(sc, 0x06, 0x810e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x88e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b89);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8a1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8b);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8c1e);
		MP_WritePhyUshort(sc, 0x06, 0x01e1);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0x1e01);
		MP_WritePhyUshort(sc, 0x06, 0xe18b);
		MP_WritePhyUshort(sc, 0x06, 0x8e1e);
		MP_WritePhyUshort(sc, 0x06, 0x01a0);
		MP_WritePhyUshort(sc, 0x06, 0x00c7);
		MP_WritePhyUshort(sc, 0x06, 0xaebb);
		MP_WritePhyUshort(sc, 0x06, 0xd481);
		MP_WritePhyUshort(sc, 0x06, 0xd4e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b92);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x9302);
		MP_WritePhyUshort(sc, 0x06, 0x2e5a);
		MP_WritePhyUshort(sc, 0x06, 0xbf8b);
		MP_WritePhyUshort(sc, 0x06, 0x88ec);
		MP_WritePhyUshort(sc, 0x06, 0x0019);
		MP_WritePhyUshort(sc, 0x06, 0xa98b);
		MP_WritePhyUshort(sc, 0x06, 0x90f9);
		MP_WritePhyUshort(sc, 0x06, 0xeeff);
		MP_WritePhyUshort(sc, 0x06, 0xf600);
		MP_WritePhyUshort(sc, 0x06, 0xeeff);
		MP_WritePhyUshort(sc, 0x06, 0xf7fc);
		MP_WritePhyUshort(sc, 0x06, 0xd100);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xae02);
		MP_WritePhyUshort(sc, 0x06, 0x3a21);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf82);
		MP_WritePhyUshort(sc, 0x06, 0xb102);
		MP_WritePhyUshort(sc, 0x06, 0x3a21);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8aad);
		MP_WritePhyUshort(sc, 0x06, 0x2014);
		MP_WritePhyUshort(sc, 0x06, 0xee8b);
		MP_WritePhyUshort(sc, 0x06, 0x8a00);
		MP_WritePhyUshort(sc, 0x06, 0x0220);
		MP_WritePhyUshort(sc, 0x06, 0x8be0);
		MP_WritePhyUshort(sc, 0x06, 0xe426);
		MP_WritePhyUshort(sc, 0x06, 0xe1e4);
		MP_WritePhyUshort(sc, 0x06, 0x27ee);
		MP_WritePhyUshort(sc, 0x06, 0xe426);
		MP_WritePhyUshort(sc, 0x06, 0x23e5);
		MP_WritePhyUshort(sc, 0x06, 0xe427);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x14ee);
		MP_WritePhyUshort(sc, 0x06, 0x8b8d);
		MP_WritePhyUshort(sc, 0x06, 0x00e0);
		MP_WritePhyUshort(sc, 0x06, 0x8a5a);
		MP_WritePhyUshort(sc, 0x06, 0x7803);
		MP_WritePhyUshort(sc, 0x06, 0x9e09);
		MP_WritePhyUshort(sc, 0x06, 0x0206);
		MP_WritePhyUshort(sc, 0x06, 0x2802);
		MP_WritePhyUshort(sc, 0x06, 0x80b1);
		MP_WritePhyUshort(sc, 0x06, 0x0232);
		MP_WritePhyUshort(sc, 0x06, 0xfdfc);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xf9e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xac26);
		MP_WritePhyUshort(sc, 0x06, 0x1ae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b81);
		MP_WritePhyUshort(sc, 0x06, 0xac21);
		MP_WritePhyUshort(sc, 0x06, 0x14e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xac20);
		MP_WritePhyUshort(sc, 0x06, 0x0ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b85);
		MP_WritePhyUshort(sc, 0x06, 0xac23);
		MP_WritePhyUshort(sc, 0x06, 0x08e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b87);
		MP_WritePhyUshort(sc, 0x06, 0xac24);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x3802);
		MP_WritePhyUshort(sc, 0x06, 0x1b02);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x1c04);
		MP_WritePhyUshort(sc, 0x06, 0xeee4);
		MP_WritePhyUshort(sc, 0x06, 0x1d04);
		MP_WritePhyUshort(sc, 0x06, 0xe2e0);
		MP_WritePhyUshort(sc, 0x06, 0x7ce3);
		MP_WritePhyUshort(sc, 0x06, 0xe07d);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x38e1);
		MP_WritePhyUshort(sc, 0x06, 0xe039);
		MP_WritePhyUshort(sc, 0x06, 0xad2e);
		MP_WritePhyUshort(sc, 0x06, 0x1bad);
		MP_WritePhyUshort(sc, 0x06, 0x390d);
		MP_WritePhyUshort(sc, 0x06, 0xd101);
		MP_WritePhyUshort(sc, 0x06, 0xbf22);
		MP_WritePhyUshort(sc, 0x06, 0xe802);
		MP_WritePhyUshort(sc, 0x06, 0x3a21);
		MP_WritePhyUshort(sc, 0x06, 0x0222);
		MP_WritePhyUshort(sc, 0x06, 0x10ae);
		MP_WritePhyUshort(sc, 0x06, 0x0bac);
		MP_WritePhyUshort(sc, 0x06, 0x3802);
		MP_WritePhyUshort(sc, 0x06, 0xae06);
		MP_WritePhyUshort(sc, 0x06, 0x0222);
		MP_WritePhyUshort(sc, 0x06, 0x4d02);
		MP_WritePhyUshort(sc, 0x06, 0x2292);
		MP_WritePhyUshort(sc, 0x06, 0x021b);
		MP_WritePhyUshort(sc, 0x06, 0x13fd);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x1af6);
		MP_WritePhyUshort(sc, 0x06, 0x20e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x022b);
		MP_WritePhyUshort(sc, 0x06, 0x1e02);
		MP_WritePhyUshort(sc, 0x06, 0x2c7c);
		MP_WritePhyUshort(sc, 0x06, 0x0203);
		MP_WritePhyUshort(sc, 0x06, 0xc002);
		MP_WritePhyUshort(sc, 0x06, 0x827d);
		MP_WritePhyUshort(sc, 0x06, 0x022e);
		MP_WritePhyUshort(sc, 0x06, 0x6f02);
		MP_WritePhyUshort(sc, 0x06, 0x047b);
		MP_WritePhyUshort(sc, 0x06, 0x022f);
		MP_WritePhyUshort(sc, 0x06, 0x9ae0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad21);
		MP_WritePhyUshort(sc, 0x06, 0x0bf6);
		MP_WritePhyUshort(sc, 0x06, 0x21e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0x0281);
		MP_WritePhyUshort(sc, 0x06, 0x9002);
		MP_WritePhyUshort(sc, 0x06, 0x1cd9);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2208);
		MP_WritePhyUshort(sc, 0x06, 0xf622);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x35f4);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2308);
		MP_WritePhyUshort(sc, 0x06, 0xf623);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x31e8);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2405);
		MP_WritePhyUshort(sc, 0x06, 0xf624);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8ee0);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xad25);
		MP_WritePhyUshort(sc, 0x06, 0x05f6);
		MP_WritePhyUshort(sc, 0x06, 0x25e4);
		MP_WritePhyUshort(sc, 0x06, 0x8b8e);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2608);
		MP_WritePhyUshort(sc, 0x06, 0xf626);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x2d8a);
		MP_WritePhyUshort(sc, 0x06, 0xe08b);
		MP_WritePhyUshort(sc, 0x06, 0x8ead);
		MP_WritePhyUshort(sc, 0x06, 0x2705);
		MP_WritePhyUshort(sc, 0x06, 0xf627);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x8e02);
		MP_WritePhyUshort(sc, 0x06, 0x0386);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8fa);
		MP_WritePhyUshort(sc, 0x06, 0xef69);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x00e1);
		MP_WritePhyUshort(sc, 0x06, 0xe001);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x32e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf720);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40bf);
		MP_WritePhyUshort(sc, 0x06, 0x32c1);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xf4ad);
		MP_WritePhyUshort(sc, 0x06, 0x2821);
		MP_WritePhyUshort(sc, 0x06, 0xe0e0);
		MP_WritePhyUshort(sc, 0x06, 0x20e1);
		MP_WritePhyUshort(sc, 0x06, 0xe021);
		MP_WritePhyUshort(sc, 0x06, 0xad20);
		MP_WritePhyUshort(sc, 0x06, 0x18e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b40);
		MP_WritePhyUshort(sc, 0x06, 0xf620);
		MP_WritePhyUshort(sc, 0x06, 0xe48b);
		MP_WritePhyUshort(sc, 0x06, 0x40ee);
		MP_WritePhyUshort(sc, 0x06, 0x8b3b);
		MP_WritePhyUshort(sc, 0x06, 0xffe0);
		MP_WritePhyUshort(sc, 0x06, 0x8a8a);
		MP_WritePhyUshort(sc, 0x06, 0xe18a);
		MP_WritePhyUshort(sc, 0x06, 0x8be4);
		MP_WritePhyUshort(sc, 0x06, 0xe000);
		MP_WritePhyUshort(sc, 0x06, 0xe5e0);
		MP_WritePhyUshort(sc, 0x06, 0x01ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0xf8f9);
		MP_WritePhyUshort(sc, 0x06, 0xface);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69fa);
		MP_WritePhyUshort(sc, 0x06, 0xd401);
		MP_WritePhyUshort(sc, 0x06, 0x55b4);
		MP_WritePhyUshort(sc, 0x06, 0xfebf);
		MP_WritePhyUshort(sc, 0x06, 0x1c5e);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xf4ac);
		MP_WritePhyUshort(sc, 0x06, 0x280b);
		MP_WritePhyUshort(sc, 0x06, 0xbf1c);
		MP_WritePhyUshort(sc, 0x06, 0x5b02);
		MP_WritePhyUshort(sc, 0x06, 0x39f4);
		MP_WritePhyUshort(sc, 0x06, 0xac28);
		MP_WritePhyUshort(sc, 0x06, 0x49ae);
		MP_WritePhyUshort(sc, 0x06, 0x64bf);
		MP_WritePhyUshort(sc, 0x06, 0x1c5b);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xf4ac);
		MP_WritePhyUshort(sc, 0x06, 0x285b);
		MP_WritePhyUshort(sc, 0x06, 0xd000);
		MP_WritePhyUshort(sc, 0x06, 0x0282);
		MP_WritePhyUshort(sc, 0x06, 0x62ac);
		MP_WritePhyUshort(sc, 0x06, 0x2105);
		MP_WritePhyUshort(sc, 0x06, 0xac22);
		MP_WritePhyUshort(sc, 0x06, 0x02ae);
		MP_WritePhyUshort(sc, 0x06, 0x4ebf);
		MP_WritePhyUshort(sc, 0x06, 0xe0c4);
		MP_WritePhyUshort(sc, 0x06, 0xbe85);
		MP_WritePhyUshort(sc, 0x06, 0xecd2);
		MP_WritePhyUshort(sc, 0x06, 0x04d8);
		MP_WritePhyUshort(sc, 0x06, 0x19d9);
		MP_WritePhyUshort(sc, 0x06, 0x1907);
		MP_WritePhyUshort(sc, 0x06, 0xdc19);
		MP_WritePhyUshort(sc, 0x06, 0xdd19);
		MP_WritePhyUshort(sc, 0x06, 0x0789);
		MP_WritePhyUshort(sc, 0x06, 0x89ef);
		MP_WritePhyUshort(sc, 0x06, 0x645e);
		MP_WritePhyUshort(sc, 0x06, 0x07ff);
		MP_WritePhyUshort(sc, 0x06, 0x0d65);
		MP_WritePhyUshort(sc, 0x06, 0x5cf8);
		MP_WritePhyUshort(sc, 0x06, 0x001e);
		MP_WritePhyUshort(sc, 0x06, 0x46dc);
		MP_WritePhyUshort(sc, 0x06, 0x19dd);
		MP_WritePhyUshort(sc, 0x06, 0x19b2);
		MP_WritePhyUshort(sc, 0x06, 0xe2d4);
		MP_WritePhyUshort(sc, 0x06, 0x0001);
		MP_WritePhyUshort(sc, 0x06, 0xbf1c);
		MP_WritePhyUshort(sc, 0x06, 0x5b02);
		MP_WritePhyUshort(sc, 0x06, 0x3a21);
		MP_WritePhyUshort(sc, 0x06, 0xae1d);
		MP_WritePhyUshort(sc, 0x06, 0xbee0);
		MP_WritePhyUshort(sc, 0x06, 0xc4bf);
		MP_WritePhyUshort(sc, 0x06, 0x85ec);
		MP_WritePhyUshort(sc, 0x06, 0xd204);
		MP_WritePhyUshort(sc, 0x06, 0xd819);
		MP_WritePhyUshort(sc, 0x06, 0xd919);
		MP_WritePhyUshort(sc, 0x06, 0x07dc);
		MP_WritePhyUshort(sc, 0x06, 0x19dd);
		MP_WritePhyUshort(sc, 0x06, 0x1907);
		MP_WritePhyUshort(sc, 0x06, 0xb2f4);
		MP_WritePhyUshort(sc, 0x06, 0xd400);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x1c5b);
		MP_WritePhyUshort(sc, 0x06, 0x023a);
		MP_WritePhyUshort(sc, 0x06, 0x21fe);
		MP_WritePhyUshort(sc, 0x06, 0xef96);
		MP_WritePhyUshort(sc, 0x06, 0xfec6);
		MP_WritePhyUshort(sc, 0x06, 0xfefd);
		MP_WritePhyUshort(sc, 0x06, 0xfc05);
		MP_WritePhyUshort(sc, 0x06, 0xf9e2);
		MP_WritePhyUshort(sc, 0x06, 0xe0ea);
		MP_WritePhyUshort(sc, 0x06, 0xe3e0);
		MP_WritePhyUshort(sc, 0x06, 0xeb5a);
		MP_WritePhyUshort(sc, 0x06, 0x070c);
		MP_WritePhyUshort(sc, 0x06, 0x031e);
		MP_WritePhyUshort(sc, 0x06, 0x20e6);
		MP_WritePhyUshort(sc, 0x06, 0xe0ea);
		MP_WritePhyUshort(sc, 0x06, 0xe7e0);
		MP_WritePhyUshort(sc, 0x06, 0xebe0);
		MP_WritePhyUshort(sc, 0x06, 0xe0fc);
		MP_WritePhyUshort(sc, 0x06, 0xe1e0);
		MP_WritePhyUshort(sc, 0x06, 0xfdfd);
		MP_WritePhyUshort(sc, 0x06, 0x04f8);
		MP_WritePhyUshort(sc, 0x06, 0xfaef);
		MP_WritePhyUshort(sc, 0x06, 0x69e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b80);
		MP_WritePhyUshort(sc, 0x06, 0xad27);
		MP_WritePhyUshort(sc, 0x06, 0x22bf);
		MP_WritePhyUshort(sc, 0x06, 0x47ba);
		MP_WritePhyUshort(sc, 0x06, 0x0239);
		MP_WritePhyUshort(sc, 0x06, 0xf4e0);
		MP_WritePhyUshort(sc, 0x06, 0x8b44);
		MP_WritePhyUshort(sc, 0x06, 0x1f01);
		MP_WritePhyUshort(sc, 0x06, 0x9e15);
		MP_WritePhyUshort(sc, 0x06, 0xe58b);
		MP_WritePhyUshort(sc, 0x06, 0x44ad);
		MP_WritePhyUshort(sc, 0x06, 0x2907);
		MP_WritePhyUshort(sc, 0x06, 0xac28);
		MP_WritePhyUshort(sc, 0x06, 0x04d1);
		MP_WritePhyUshort(sc, 0x06, 0x01ae);
		MP_WritePhyUshort(sc, 0x06, 0x02d1);
		MP_WritePhyUshort(sc, 0x06, 0x00bf);
		MP_WritePhyUshort(sc, 0x06, 0x82b4);
		MP_WritePhyUshort(sc, 0x06, 0x023a);
		MP_WritePhyUshort(sc, 0x06, 0x21ef);
		MP_WritePhyUshort(sc, 0x06, 0x96fe);
		MP_WritePhyUshort(sc, 0x06, 0xfc04);
		MP_WritePhyUshort(sc, 0x06, 0x00e1);
		MP_WritePhyUshort(sc, 0x06, 0x4077);
		MP_WritePhyUshort(sc, 0x06, 0xe140);
		MP_WritePhyUshort(sc, 0x06, 0xbbe0);
		MP_WritePhyUshort(sc, 0x06, 0x2a00);
		MP_WritePhyUshort(sc, 0x05, 0xe142);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0xe140);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		for (i = 0; i < 200; i++)
		{
			DELAY(100);
			Data = MP_ReadPhyUshort(sc, 0x00);
			if (Data & BIT_7)
				break;
		}
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0023);
		Data = MP_ReadPhyUshort(sc, 0x17);
		Data |= BIT_1;
		MP_WritePhyUshort(sc, 0x17, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0003);
		MP_WritePhyUshort(sc, 0x09, 0xA20F);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x00, 0x9200);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B80);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_2 | BIT_1;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x002D);
		Data = MP_ReadPhyUshort(sc, 0x18);
		Data |= BIT_4;
		MP_WritePhyUshort(sc, 0x18, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		Data = MP_ReadPhyUshort(sc, 0x14);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x14, Data);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B86);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_0;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_14;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B55);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x05, 0x8B5E);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x05, 0x8B67);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x05, 0x8B70);
		MP_WritePhyUshort(sc, 0x06, 0x0000);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x1F, 0x0007);
		MP_WritePhyUshort(sc, 0x1E, 0x0078);
		MP_WritePhyUshort(sc, 0x17, 0x0000);
		MP_WritePhyUshort(sc, 0x19, 0x00AA);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B79);
		MP_WritePhyUshort(sc, 0x06, 0xAA00);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B54);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8B5D);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_11;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7C);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A7F);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A82);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x05, 0x8A88);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1F, 0x0000);

		MP_WritePhyUshort(sc, 0x1F, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data |= BIT_15;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);

		Data_u32 = re_eri_read(sc, 0x1b0, 4, ERIAR_ExGMAC);
		Data_u32 &= ~(BIT_0 | BIT_1);
		re_eri_write(sc, 0x1b0, 2, Data_u32, ERIAR_ExGMAC);
		MP_WritePhyUshort(sc, 0x1f, 0x0005);
		MP_WritePhyUshort(sc, 0x05, 0x8B85);
		Data = MP_ReadPhyUshort(sc, 0x06);
		Data &= ~BIT_13;
		MP_WritePhyUshort(sc, 0x06, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0007);
		MP_WritePhyUshort(sc, 0x1e, 0x0020);
		Data = MP_ReadPhyUshort(sc, 0x15);
		Data &= ~BIT_8;
		MP_WritePhyUshort(sc, 0x15, Data);
		MP_WritePhyUshort(sc, 0x1f, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0007);
		MP_WritePhyUshort(sc, 0x0e, 0x003c);
		MP_WritePhyUshort(sc, 0x0d, 0x4007);
		MP_WritePhyUshort(sc, 0x0e, 0x0000);
		MP_WritePhyUshort(sc, 0x0d, 0x0000);
	}
	MP_WritePhyUshort(sc, 0x1F, 0x0000);
}

void MP_WritePhyUshort(struct re_softc *sc,u_int8_t RegAddr,u_int16_t RegData)
{
	u_int32_t		TmpUlong=0x80000000;
	u_int32_t		Timeout=0;

	TmpUlong |= (((u_int32_t)RegAddr)<<16 | (u_int32_t)RegData);

	CSR_WRITE_4(sc, RE_PHYAR, TmpUlong);

	/* Wait for writing to Phy ok */
	for (Timeout=0; Timeout<5; Timeout++) {
		DELAY(1000);
		if ((CSR_READ_4(sc, RE_PHYAR)&PHYAR_Flag)==0)
			break;
	}
}

u_int16_t MP_ReadPhyUshort(struct re_softc *sc,u_int8_t RegAddr)
{
	u_int16_t		RegData;
	u_int32_t		TmpUlong;
	u_int32_t		Timeout=0;

	TmpUlong = ((u_int32_t)RegAddr << 16);
	CSR_WRITE_4(sc, RE_PHYAR, TmpUlong);

	/* Wait for writing to Phy ok */
	for (Timeout=0; Timeout<5; Timeout++) {
		DELAY(1000);
		TmpUlong = CSR_READ_4(sc, RE_PHYAR);
		if ((TmpUlong&PHYAR_Flag)!=0)
			break;
	}

	RegData = (u_int16_t)(TmpUlong & 0x0000ffff);

	return RegData;
}

void MP_WriteEPhyUshort(struct re_softc *sc, u_int8_t RegAddr, u_int16_t RegData)
{
	u_int32_t		TmpUlong=0x80000000;
	u_int32_t		Timeout=0;

	TmpUlong |= (((u_int32_t)RegAddr<<16) | (u_int32_t)RegData);

	CSR_WRITE_4(sc, RE_EPHYAR, TmpUlong);

	/* Wait for writing to Phy ok */
	for (Timeout=0; Timeout<5; Timeout++) {
		DELAY(1000);
		if ((CSR_READ_4(sc, RE_EPHYAR)&PHYAR_Flag)==0)
			break;
	}
}

u_int16_t MP_ReadEPhyUshort(struct re_softc *sc, u_int8_t RegAddr)
{
	u_int16_t		RegData;
	u_int32_t		TmpUlong;
	u_int32_t		Timeout=0;

	TmpUlong = ((u_int32_t)RegAddr << 16);
	CSR_WRITE_4(sc, RE_EPHYAR, TmpUlong);

	/* Wait for writing to Phy ok */
	for (Timeout=0; Timeout<5; Timeout++) {
		DELAY(1000);
		TmpUlong = CSR_READ_4(sc, RE_EPHYAR);
		if ((TmpUlong&PHYAR_Flag)!=0)
			break;
	}

	RegData = (u_int16_t)(TmpUlong & 0x0000ffff);

	return RegData;
}

u_int8_t MP_ReadEfuse(struct re_softc *sc, u_int16_t RegAddr)
{
	u_int8_t		RegData;
	u_int32_t		TmpUlong;
	u_int32_t		Timeout=0;

	RegAddr &= 0x3FF;
	TmpUlong = ((u_int32_t)RegAddr << 8);
	CSR_WRITE_4(sc, 0xDC, TmpUlong);

	/* Wait for writing to Phy ok */
	for (Timeout=0; Timeout<5; Timeout++) {
		DELAY(1000);
		TmpUlong = CSR_READ_4(sc, 0xDC);
		if ((TmpUlong&PHYAR_Flag)!=0)
			break;
	}

	RegData = (u_int8_t)(TmpUlong & 0x000000ff);

	return RegData;
}

/*----------------------------------------------------------------------------*/
/*	8139 (CR9346) 9346 command register bits (offset 0x50, 1 byte)*/
/*----------------------------------------------------------------------------*/
#define CR9346_EEDO				0x01			/* 9346 data out*/
#define CR9346_EEDI				0x02			/* 9346 data in*/
#define CR9346_EESK				0x04			/* 9346 serial clock*/
#define CR9346_EECS				0x08			/* 9346 chip select*/
#define CR9346_EEM0				0x40			/* select 8139 operating mode*/
#define CR9346_EEM1				0x80			/* 00: normal*/
#define CR9346_CFGRW			0xC0			/* Config register write*/
#define CR9346_NORM			0x00

/*----------------------------------------------------------------------------*/
/*	EEPROM bit definitions(EEPROM control register bits)*/
/*----------------------------------------------------------------------------*/
#define EN_TRNF					0x10			/* Enable turnoff*/
#define EEDO						CR9346_EEDO	/* EEPROM data out*/
#define EEDI						CR9346_EEDI		/* EEPROM data in (set for writing data)*/
#define EECS						CR9346_EECS		/* EEPROM chip select (1=high, 0=low)*/
#define EESK						CR9346_EESK		/* EEPROM shift clock (1=high, 0=low)*/

/*----------------------------------------------------------------------------*/
/*	EEPROM opcodes*/
/*----------------------------------------------------------------------------*/
#define EEPROM_READ_OPCODE	06
#define EEPROM_WRITE_OPCODE	05
#define EEPROM_ERASE_OPCODE	07
#define EEPROM_EWEN_OPCODE	19				/* Erase/write enable*/
#define EEPROM_EWDS_OPCODE	16				/* Erase/write disable*/

#define	CLOCK_RATE				50				/* us*/

#define RaiseClock(_sc,_x)				\
	(_x) = (_x) | EESK;					\
	CSR_WRITE_1((_sc), RE_EECMD, (_x));	\
	DELAY(CLOCK_RATE);

#define LowerClock(_sc,_x)				\
	(_x) = (_x) & ~EESK;					\
	CSR_WRITE_1((_sc), RE_EECMD, (_x));	\
	DELAY(CLOCK_RATE);

/*
 * Shift out bit(s) to the EEPROM.
 */
static void re_eeprom_ShiftOutBits(sc, data, count)
	struct re_softc		*sc;
	int			data;
	int 			count;
{
	u_int16_t x, mask;

	mask = 0x01 << (count - 1);
	x = CSR_READ_1(sc, RE_EECMD);

	x &= ~(EEDO | EEDI);

	do
	{
		x &= ~EEDI;
		if (data & mask)
			x |= EEDI;

		CSR_WRITE_1(sc, RE_EECMD, x);
		DELAY(CLOCK_RATE);
		RaiseClock(sc,x);
		LowerClock(sc,x);
		mask = mask >> 1;
	} while (mask);

	x &= ~EEDI;
	CSR_WRITE_1(sc, RE_EECMD, x);
}

/*
 * Shift in bit(s) from the EEPROM.
 */
static u_int16_t re_eeprom_ShiftInBits(sc)
	struct re_softc		*sc;
{
	u_int16_t x,d,i;
	x = CSR_READ_1(sc, RE_EECMD);

	x &= ~(EEDO | EEDI);
	d = 0;

	for (i=0; i<16; i++) {
		d = d << 1;
		RaiseClock(sc, x);

		x = CSR_READ_1(sc, RE_EECMD);

		x &= ~(EEDI);
		if (x & EEDO)
			d |= 1;

		LowerClock(sc, x);
	}

	return d;
}

/*
 * Clean up EEprom read/write setting
 */
static void re_eeprom_EEpromCleanup(sc)
	struct re_softc		*sc;
{
	u_int16_t x;
	x = CSR_READ_1(sc, RE_EECMD);

	x &= ~(EECS | EEDI);
	CSR_WRITE_1(sc, RE_EECMD, x);

	RaiseClock(sc, x);
	LowerClock(sc, x);
}

/*
 * Read a word of data stored in the EEPROM at address 'addr.'
 */
static void re_eeprom_getword(sc, addr, dest)
	struct re_softc		*sc;
	int			addr;
	u_int16_t		*dest;
{
	u_int16_t x;

	/* select EEPROM, reset bits, set EECS*/
	x = CSR_READ_1(sc, RE_EECMD);

	x &= ~(EEDI | EEDO | EESK | CR9346_EEM0);
	x |= CR9346_EEM1 | EECS;
	CSR_WRITE_1(sc, RE_EECMD, x);

	/* write the read opcode and register number in that order*/
	/* The opcode is 3bits in length, reg is 6 bits long*/
	re_eeprom_ShiftOutBits(sc, EEPROM_READ_OPCODE, 3);

	if (CSR_READ_4(sc, RE_RXCFG) & RE_RXCFG_RX_9356SEL)
		re_eeprom_ShiftOutBits(sc, addr,8);	/*93c56=8*/
	else
		re_eeprom_ShiftOutBits(sc, addr,6);	/*93c46=6*/

	/* Now read the data (16 bits) in from the selected EEPROM word*/
	*dest=re_eeprom_ShiftInBits(sc);

	re_eeprom_EEpromCleanup(sc);
	return;
}

/*
 * Read a sequence of words from the EEPROM.
 */
static void re_read_eeprom(sc, dest, off, cnt, swap)
	struct re_softc		*sc;
	caddr_t			dest;
	int			off;
	int			cnt;
	int			swap;
{
	int			i;
	u_int16_t		word = 0, *ptr;

	for (i = 0; i < cnt; i++) {
		re_eeprom_getword(sc, off + i, &word);
		ptr = (u_int16_t *)(dest + (i * 2));
		if (swap)
			*ptr = ntohs(word);
		else
			*ptr = word;
	}

	return;
}
