/**
 * htc_battery_options_sysfs.c
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

#include <mach/htc_battery_sysfs_option.h>
#include <mach/htc_battery_sysfs.h>

extern int system_option;
extern struct option available_options[LAST];
extern struct kobject *batt_kobj;

static struct kobject *sbc_kobj;

static ssize_t cl_settings_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int var;

	if (strcmp(attr->attr.name, "sysctl_batt_sbc") == 0) {
		var = sysctl_batt_sbc;
	} else {
		return 0;
	}
        return sprintf(buf, "%d\n", var);
}

static ssize_t cl_settings_store(struct kobject *kobj,
        struct kobj_attribute *attr, const char *buf, size_t count)
{
        int var;

        sscanf(buf, "%du", &var);
        if (strcmp(attr->attr.name, "sysctl_batt_sbc") == 0) {
                sysctl_batt_sbc = var;
        }

        return count;
}

static struct kobj_attribute cl_batt_sbc =
	__ATTR(sysctl_batt_sbc, 0666, cl_settings_show,
	cl_settings_store);

static struct attribute *custom_option_attrs[] = {
        &cl_batt_sbc.attr,
	NULL,
};

static struct attribute_group custom_option_group = {
	.attrs = custom_option_attrs,
};

static ssize_t options_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int i;

	int len = 0;
	for (i=0; (i<LAST); i++) {
		if (i==system_option)
			len += sprintf(buf+len, "[%s] ",
				available_options[i].name);
		else
			len += sprintf(buf+len, "%s ",
				available_options[i].name);
	}
	len += sprintf(len+buf, "\n");

	return len;
}

static ssize_t options_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
        int i;

        for (i=0; i<(LAST); i++) {
		if (strcmp(buf, available_options[i].name) == 0) {
		        system_option = i;
			batt_set_option(i);
			break;
		}
	}
	return count;
}

static struct kobj_attribute options_attribute =
        __ATTR(system_option, 0666, options_show, options_store);

static struct attribute *option_attrs[] = {
	&options_attribute.attr,
        NULL,
};

static struct attribute_group option_group = {
        .attrs = option_attrs,
};

int init_sysfs_interface_system_option(void)
{
        int ret = 0;

	ret = sysfs_create_group(batt_kobj, &option_group);
	if (!ret) {
		sbc_kobj =
			kobject_create_and_add("sbc", batt_kobj);
		if (!sbc_kobj) {
			ret = -ENOMEM;
		} else {
			ret = sysfs_create_group(sbc_kobj,
				&custom_option_group);
			if (ret) {
				kobject_put(batt_kobj);
			}
		}
	}

	if (!ret)
	        printk(KERN_INFO "Battery Options [SBC Toggle Support].\n");

        return ret;
}

void cleanup_sysfs_interface_system_option(void)
{
	kobject_put(sbc_kobj);
}

module_init(init_sysfs_interface_system_option);
module_exit(cleanup_sysfs_interface_system_option);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Brandon Berhent <bbedward@gmail.com>");


