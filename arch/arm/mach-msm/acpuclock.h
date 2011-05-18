/* arch/arm/mach-msm/acpuclock.h
 *
 * MSM architecture clock driver header
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2010, Code Aurora Forum. All rights reserved.
 * Author: San Mehat <san@android.com>
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

#ifndef __ARCH_ARM_MACH_MSM_ACPUCLOCK_H
#define __ARCH_ARM_MACH_MSM_ACPUCLOCK_H

#include <linux/list.h>

enum setrate_reason {
	SETRATE_CPUFREQ = 0,
	SETRATE_SWFI,
	SETRATE_PC,
	SETRATE_PC_IDLE,
	SETRATE_INIT,
};

#ifdef CONFIG_ARCH_MSM8X60
unsigned long acpuclk_power_collapse(void);
int acpuclk_set_rate(int cpu, unsigned long rate, enum setrate_reason reason);
unsigned long acpuclk_get_rate(int);
#else
unsigned long acpuclk_power_collapse(int from_idle);
int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason);
unsigned long acpuclk_get_rate(void);
#endif

uint32_t acpuclk_get_switch_time(void);
unsigned long acpuclk_wait_for_irq(void);
unsigned long acpuclk_get_wfi_rate(void);


#endif

