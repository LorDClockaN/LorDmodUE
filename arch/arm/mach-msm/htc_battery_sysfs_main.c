/**
 * htc_battery_sysfs.c
 * Battery Options by Savaged-Zen
 *
 * Copyright (C) 2011 Brandon Berhent <bbedward@gmail.com>
 * Based on the zen-tune sysfs interface (C) 2008 Ryan Hope <rmh3093@gmail.com>
 *
 * Released under the GPL version 2 only.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <mach/htc_battery_sysfs.h>

static int batt_sysfs_init(void)
{
        int ret;

        ret = init_sysfs_interface();
        if (ret)
                goto out_error;

        return 0;

 out_error:
        return ret;
}

static void batt_sysfs_exit(void)
{
        cleanup_sysfs_interface();
}

module_init(batt_sysfs_init);
module_exit(batt_sysfs_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Brandon Berhent <bbedward@gmail.com>");

