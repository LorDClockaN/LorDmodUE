/**
 * htc_battery_sysfs_option.h
 * Battery Options by Savaged-Zen
 *
 * Copyright (C) 2011 Brandon Berhent <bbedward@gmail.com>
 *
 * Released under the GPL version 2 only.
 *
 */

#define SBC 0
#define NOSBC 1
#define LAST 2

struct option
{
	char *name;
	int batt_sbc;
};

int batt_set_option(int option);

