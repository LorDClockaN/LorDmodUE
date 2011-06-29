/**
 * htc_battery_sysfs.c
 * Battery Options by Savaged-Zen
 *
 * Copyright (C) 2011 Brandon Berhent <bbedward@gmail.com>
 *
 * Released under the GPL version 2 only.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include <mach/htc_battery_sysfs.h>

struct kobject *batt_kobj;
EXPORT_SYMBOL(batt_kobj);

int init_sysfs_interface(void)
{
	int ret = 0;

	/* Create /sys/batt_options/ */
	batt_kobj = kobject_create_and_add("batt_options", kernel_kobj);
        if (batt_kobj == NULL) {
                printk(KERN_ERR "batt_options: subsystem_register failed.\n");
                ret = -ENOMEM;
                return ret;
        } else {
		printk(KERN_INFO "batt_options: sysfs interface initiated.\n");
	}
	return 0;
}

void cleanup_sysfs_interface(void)
{
	kobject_put(batt_kobj);
}

