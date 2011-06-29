/**
 * htc_battery_options.c
 * Battery Options by Savaged-Zen
 *
 * Copyright (C) 2011 Brandon Berhent <bbedward@gmail.com>
 *
 * Released under the GPL version 2 only.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>

#include <mach/htc_battery_sysfs_option.h>
#include <mach/htc_battery_sysfs.h>

int sysctl_batt_sbc;
extern int batt_sbc;

#ifdef CONFIG_HTC_BATTCHG_SBC_DEFAULT
int system_option = SBC;
#else
int system_option = NOSBC;
#endif
EXPORT_SYMBOL(system_option);

struct option available_options[LAST] =
{
	{
		/* SBC ON */
		.name = "sbc",
		.batt_sbc = 1,
	},
	{
		/* SBC OFF */
		.name = "nosbc",
		.batt_sbc = 0,
	},
};
EXPORT_SYMBOL(available_options);

int batt_set_option(int option)
{
	sysctl_batt_sbc = available_options[option].batt_sbc;

	return 0;
}
EXPORT_SYMBOL(batt_set_option);

static int batt_sbc_option_init(void)
{
	return batt_set_option(system_option);
}

static void batt_sbc_option_exit(void)
{
}

module_init(batt_sbc_option_init);
module_exit(batt_sbc_option_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Brandon Berhent <bbedward@gmail.com>");


