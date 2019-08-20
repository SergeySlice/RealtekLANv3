/*
 *  impl_defs.h - Miscellaneous definitions for implementation use
 *  RealtekR1000SL
 *
 *  Created by Chuck Fry on 10/8/09.
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



// Implementation macros
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)
// copied from AppleRTL8139Ethernet source
#define BUMP_NET_COUNTER(x)			do { netStats->x += 1; } while(0)
#define BUMP_ETHER_COUNTER(x)		do { etherStats->dot3StatsEntry.x += 1; } while(0)
#define BUMP_ETHER_RX_COUNTER(x)	do { etherStats->dot3RxExtraEntry.x += 1; } while(0)
#define BUMP_ETHER_TX_COUNTER(x)	do { etherStats->dot3TxExtraEntry.x += 1; } while(0)

enum 
{
    kActivationLevelNone = 0,  /* adapter shut off */
    kActivationLevelKDP,       /* adapter partially up to support KDP */
    kActivationLevelBSD        /* adapter fully up to support KDP and BSD */
};

// Used in configuration description
struct RtlChipInfo
{
	const char *name;
	u16 mcfg;
	u16 max_desc;		/* Maximum # of Rx/Tx buffer descriptors    */
	u32 RCR_Cfg;		/* Base value for RxConfig register         */
	u32 RxConfigMask;	/* Clears the bits supported by this chip   */
	u16 jumbo_frame_sz; /* Max size of jumbo frame; 0 = not capable */
	u16 efuse;			/* Whether or not chip supports efuse       */
};
