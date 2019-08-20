/* This driver based on R1000 Linux Driver for Realtek controllers.
 * It's not supported by Realtek company, so use it for your own risk.
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
 *
 *****************************************************************
 *
 * MODIFIED by PSYSTAR, 2008 -- (Rudy Pedraza)
 * -- all changes released under GPL as required
 * -- changes made to code are copyright PSYSTAR Corporation, 2008
 **** Enhancement Log
 * - added sleep/wake DHCP fix for
 * - changed tx/rx interrupt handling, 2x speedup
 * - added support for multiple NIC's, driver didnt play nice before
 * - fixed com.apple.kernel & com.apple.kpi dependencies, you cant use both (warning now, future error)
 * - cleaned up Info.plist, fixed matching
 *
 *****************************************************************
 *
 * MODIFIED by Chuck Fry (chucko@chucko.com) 2009
 * -- released under GPL
 **** Enhancement log
 * - 64 bit clean
 * - Handle 64 bit pointers as appropriate
 * - you can safely ignore compiler warning "Right shift count >= width of type" on 32 bit platforms
 * - fix compiler errors in increaseActivationLevel()
 * - better thread safety for outputPacket()
 * - added liberal comments where I needed to figure out what was happening
 *
 *****************************************************************
 *
 * modified by Slice 2013
 * -- updated according latest linux's sources r8168-8.035.00, 2012
 * -- added speed check
 * -- warning eliminatings
 *
 ******************************************************************
 */

#ifndef _MII_H_
#define _MII_H_

#define AUTONEG_ENABLE			0x01
#define AUTONEG_DISABLE			0x02

#define DUPLEX_HALF				0x01
#define DUPLEX_FULL				0x02

#define SPEED_10				0x01
#define SPEED_100				0x02
#define SPEED_1000				0x03


#endif
