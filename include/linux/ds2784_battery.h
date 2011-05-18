/* include/linux/ds2784_battery.h
 *
 * Copyright (C) 2009 HTC Corporation
 * Copyright (C) 2009 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_DS2784_BATTERY_H
#define __LINUX_DS2784_BATTERY_H

#ifdef __KERNEL__

struct ds2784_platform_data {
	int (*charge)(int on, int fast);
	void *w1_slave;
};

#endif /* __KERNEL__ */

#endif /* __LINUX_DS2784_BATTERY_H */
