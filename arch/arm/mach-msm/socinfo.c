/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*
 * SOC Info Routines
 *
 */

#include <linux/types.h>
#include <linux/sysdev.h>
#include <mach/socinfo.h>
#include "smd_private.h"

#define BUILD_ID_LENGTH 32

enum {
	HW_PLATFORM_UNKNOWN = 0,
	HW_PLATFORM_SURF    = 1,
	HW_PLATFORM_FFA     = 2,
	HW_PLATFORM_FLUID   = 3,
	HW_PLATFORM_SVLTE_FFA	= 4,
	HW_PLATFORM_SVLTE_SURF	= 5,
	HW_PLATFORM_INVALID
};

char *hw_platform[] = {
	"Unknown",
	"Surf",
	"FFA",
	"Fluid",
	"SVLTE_FFA",
	"SLVTE_SURF"
};

enum {
	ACCESSORY_CHIP_UNKNOWN = 0,
	ACCESSORY_CHIP_CHARM = 58,
};

/* Used to parse shared memory.  Must match the modem. */
struct socinfo_v1 {
	uint32_t format;
	uint32_t id;
	uint32_t version;
	char build_id[BUILD_ID_LENGTH];
};

struct socinfo_v2 {
	struct socinfo_v1 v1;

	/* only valid when format==2 */
	uint32_t raw_id;
	uint32_t raw_version;
};

struct socinfo_v3 {
	struct socinfo_v2 v2;

	/* only valid when format==3 */
	uint32_t hw_platform;
};

struct socinfo_v4 {
	struct socinfo_v3 v3;

	/* only valid when format==4 */
	uint32_t platform_version;
};

struct socinfo_v5 {
	struct socinfo_v4 v4;

	/* only valid when format==5 */
	uint32_t accessory_chip;
};

static union {
	struct socinfo_v1 v1;
	struct socinfo_v2 v2;
	struct socinfo_v3 v3;
	struct socinfo_v4 v4;
	struct socinfo_v5 v5;
} *socinfo;

static enum msm_cpu cpu_of_id[] = {

	/* 7x01 IDs */
	[1]  = MSM_CPU_7X01,
	[16] = MSM_CPU_7X01,
	[17] = MSM_CPU_7X01,
	[18] = MSM_CPU_7X01,
	[19] = MSM_CPU_7X01,
	[23] = MSM_CPU_7X01,
	[25] = MSM_CPU_7X01,
	[26] = MSM_CPU_7X01,
	[32] = MSM_CPU_7X01,
	[33] = MSM_CPU_7X01,
	[34] = MSM_CPU_7X01,
	[35] = MSM_CPU_7X01,

	/* 7x25 IDs */
	[20] = MSM_CPU_7X25,
	[21] = MSM_CPU_7X25, /* 7225 */
	[24] = MSM_CPU_7X25, /* 7525 */
	[27] = MSM_CPU_7X25, /* 7625 */
	[39] = MSM_CPU_7X25,
	[40] = MSM_CPU_7X25,
	[41] = MSM_CPU_7X25,
	[42] = MSM_CPU_7X25,
	[62] = MSM_CPU_7X25, /* 7625-1 */
	[63] = MSM_CPU_7X25, /* 7225-1 */
	[66] = MSM_CPU_7X25, /* 7225-2 */


	/* 7x27 IDs */
	[43] = MSM_CPU_7X27,
	[44] = MSM_CPU_7X27,
	[61] = MSM_CPU_7X27,
	[67] = MSM_CPU_7X27, /* 7227-1 */
	[68] = MSM_CPU_7X27, /* 7627-1 */
	[69] = MSM_CPU_7X27, /* 7627-2 */


	/* 8x50 IDs */
	[30] = MSM_CPU_8X50,
	[36] = MSM_CPU_8X50,
	[37] = MSM_CPU_8X50,
	[38] = MSM_CPU_8X50,

	/* 8x50A IDs */
	[64] = MSM_CPU_8X50A,
	[65] = MSM_CPU_8X50A,

	/* 7x30 IDs */
	[59] = MSM_CPU_7X30,
	[60] = MSM_CPU_7X30,

	/* 8x55 IDs */
	[74] = MSM_CPU_8X55,
	[75] = MSM_CPU_8X55,
	[85] = MSM_CPU_8X55,

	/* 8x60 IDs */
	[70] = MSM_CPU_8X60,
	[71] = MSM_CPU_8X60,
	[86] = MSM_CPU_8X60,

	/* Uninitialized IDs are not known to run Linux.
	   MSM_CPU_UNKNOWN is set to 0 to ensure these IDs are
	   considered as unknown CPU. */
};

static enum msm_cpu cur_cpu;

uint32_t socinfo_get_id(void)
{
	return (socinfo) ? socinfo->v1.id : 0;
}

uint32_t socinfo_get_version(void)
{
	return (socinfo) ? socinfo->v1.version : 0;
}

char *socinfo_get_build_id(void)
{
	return (socinfo) ? socinfo->v1.build_id : NULL;
}

uint32_t socinfo_get_raw_id(void)
{
	return socinfo ?
		(socinfo->v1.format >= 2 ? socinfo->v2.raw_id : 0)
		: 0;
}

uint32_t socinfo_get_raw_version(void)
{
	return socinfo ?
		(socinfo->v1.format >= 2 ? socinfo->v2.raw_version : 0)
		: 0;
}

uint32_t socinfo_get_platform_type(void)
{
	return socinfo ?
		(socinfo->v1.format >= 3 ? socinfo->v3.hw_platform : 0)
		: 0;
}


uint32_t socinfo_get_platform_version(void)
{
	return socinfo ?
		(socinfo->v1.format >= 4 ? socinfo->v4.platform_version : 0)
		: 0;
}

uint32_t socinfo_get_accessory_chip(void)
{
	return socinfo ?
		(socinfo->v1.format >= 5 ? socinfo->v5.accessory_chip : 0)
		: 0;
}


enum msm_cpu socinfo_get_msm_cpu(void)
{
	return cur_cpu;
}
EXPORT_SYMBOL_GPL(socinfo_get_msm_cpu);

static ssize_t
socinfo_show_id(struct sys_device *dev,
		struct sysdev_attribute *attr,
		char *buf)
{
	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", socinfo_get_id());
}

static ssize_t
socinfo_show_version(struct sys_device *dev,
		     struct sysdev_attribute *attr,
		     char *buf)
{
	uint32_t version;

	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}

	version = socinfo_get_version();
	return snprintf(buf, PAGE_SIZE, "%u.%u\n",
			SOCINFO_VERSION_MAJOR(version),
			SOCINFO_VERSION_MINOR(version));
}

static ssize_t
socinfo_show_build_id(struct sys_device *dev,
		      struct sysdev_attribute *attr,
		      char *buf)
{
	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%-.32s\n", socinfo_get_build_id());
}

static ssize_t
socinfo_show_raw_id(struct sys_device *dev,
		    struct sysdev_attribute *attr,
		    char *buf)
{
	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}
	if (socinfo->v1.format < 2) {
		pr_err("%s: Raw ID not available!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", socinfo_get_raw_id());
}

static ssize_t
socinfo_show_raw_version(struct sys_device *dev,
			 struct sysdev_attribute *attr,
			 char *buf)
{
	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}
	if (socinfo->v1.format < 2) {
		pr_err("%s: Raw version not available!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", socinfo_get_raw_version());
}

static ssize_t
socinfo_show_platform_type(struct sys_device *dev,
			 struct sysdev_attribute *attr,
			 char *buf)
{
	uint32_t hw_type;

	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}
	if (socinfo->v1.format < 3) {
		pr_err("%s: platform type not available!\n", __func__);
		return 0;
	}

	hw_type = socinfo_get_platform_type();
	if (hw_type >= HW_PLATFORM_INVALID) {
		pr_err("%s: Invalid hardware platform type found\n",
								   __func__);
		hw_type = HW_PLATFORM_UNKNOWN;
	}

	return snprintf(buf, PAGE_SIZE, "%-.32s\n", hw_platform[hw_type]);
}

static ssize_t
socinfo_show_platform_version(struct sys_device *dev,
			 struct sysdev_attribute *attr,
			 char *buf)
{

	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}
	if (socinfo->v1.format < 4) {
		pr_err("%s: platform version not available!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
		socinfo_get_platform_version());
}

static ssize_t
socinfo_show_accessory_chip(struct sys_device *dev,
			struct sysdev_attribute *attr,
			char *buf)
{
	if (!socinfo) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}
	if (socinfo->v1.format < 5) {
		pr_err("%s: accessory chip not available!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
		socinfo_get_accessory_chip());
}

static struct sysdev_attribute socinfo_v1_files[] = {
	_SYSDEV_ATTR(id, 0444, socinfo_show_id, NULL),
	_SYSDEV_ATTR(version, 0444, socinfo_show_version, NULL),
	_SYSDEV_ATTR(build_id, 0444, socinfo_show_build_id, NULL),
};

static struct sysdev_attribute socinfo_v2_files[] = {
	_SYSDEV_ATTR(raw_id, 0444, socinfo_show_raw_id, NULL),
	_SYSDEV_ATTR(raw_version, 0444, socinfo_show_raw_version, NULL),
};

static struct sysdev_attribute socinfo_v3_files[] = {
	_SYSDEV_ATTR(hw_platform, 0444, socinfo_show_platform_type, NULL),
};

static struct sysdev_attribute socinfo_v4_files[] = {
	_SYSDEV_ATTR(platform_version, 0444,
			socinfo_show_platform_version, NULL),
};

static struct sysdev_attribute socinfo_v5_files[] = {
	_SYSDEV_ATTR(accessory_chip, 0444,
			socinfo_show_accessory_chip, NULL),
};

static struct sysdev_class soc_sysdev_class = {
	.name = "soc",
};

static struct sys_device soc_sys_device = {
	.id = 0,
	.cls = &soc_sysdev_class,
};

static int __init socinfo_create_files(struct sys_device *dev,
					struct sysdev_attribute files[],
					int size)
{
	int i;
	for (i = 0; i < size; i++) {
		int err = sysdev_create_file(dev, &files[i]);
		if (err) {
			pr_err("%s: sysdev_create_file(%s)=%d\n",
			       __func__, files[i].attr.name, err);
			return err;
		}
	}
	return 0;
}

static int __init socinfo_init_sysdev(void)
{
	int err;

	err = sysdev_class_register(&soc_sysdev_class);
	if (err) {
		pr_err("%s: sysdev_class_register fail (%d)\n",
		       __func__, err);
		return err;
	}
	err = sysdev_register(&soc_sys_device);
	if (err) {
		pr_err("%s: sysdev_register fail (%d)\n",
		       __func__, err);
		return err;
	}
	socinfo_create_files(&soc_sys_device, socinfo_v1_files,
				ARRAY_SIZE(socinfo_v1_files));
	if (socinfo->v1.format < 2)
		return err;
	socinfo_create_files(&soc_sys_device, socinfo_v2_files,
				ARRAY_SIZE(socinfo_v2_files));

	if (socinfo->v1.format < 3)
		return err;

	socinfo_create_files(&soc_sys_device, socinfo_v3_files,
				ARRAY_SIZE(socinfo_v3_files));

	if (socinfo->v1.format < 4)
		return err;

	socinfo_create_files(&soc_sys_device, socinfo_v4_files,
				ARRAY_SIZE(socinfo_v4_files));

	if (socinfo->v1.format < 5)
		return err;

	return socinfo_create_files(&soc_sys_device, socinfo_v5_files,
				ARRAY_SIZE(socinfo_v5_files));

}

arch_initcall(socinfo_init_sysdev);

int __init socinfo_init(void)
{
	socinfo = smem_alloc(SMEM_HW_SW_BUILD_ID, sizeof(struct socinfo_v5));
	if (!socinfo)
		socinfo = smem_alloc(SMEM_HW_SW_BUILD_ID,
				sizeof(struct socinfo_v3));

	if (!socinfo)
		socinfo = smem_alloc(SMEM_HW_SW_BUILD_ID,
				sizeof(struct socinfo_v2));

	if (!socinfo)
		socinfo = smem_alloc(SMEM_HW_SW_BUILD_ID,
				sizeof(struct socinfo_v1));

	if (!socinfo) {
		pr_err("%s: Can't find SMEM_HW_SW_BUILD_ID\n",
		       __func__);
		return -EIO;
	}

	WARN(!socinfo_get_id(), "Unknown SOC ID!\n");
	WARN(socinfo_get_id() >= ARRAY_SIZE(cpu_of_id),
		"New IDs added! ID => CPU mapping might need an update.\n");

	if (socinfo->v1.id < ARRAY_SIZE(cpu_of_id))
		cur_cpu = cpu_of_id[socinfo->v1.id];

	pr_info("cur_cpu 0x%x, id 0x%x\n", cur_cpu, socinfo->v1.id);

	switch (socinfo->v1.format) {
	case 1:
		pr_info("%s: v%u, id=%u, ver=%u.%u\n",
			__func__, socinfo->v1.format, socinfo->v1.id,
			SOCINFO_VERSION_MAJOR(socinfo->v1.version),
			SOCINFO_VERSION_MINOR(socinfo->v1.version));
		break;
	case 2:
		pr_info("%s: v%u, id=%u, ver=%u.%u, "
			 "raw_id=%u, raw_ver=%u\n",
			__func__, socinfo->v1.format, socinfo->v1.id,
			SOCINFO_VERSION_MAJOR(socinfo->v1.version),
			SOCINFO_VERSION_MINOR(socinfo->v1.version),
			socinfo->v2.raw_id, socinfo->v2.raw_version);
		break;
	case 3:
		pr_info("%s: v%u, id=%u, ver=%u.%u, "
			 "raw_id=%u, raw_ver=%u, hw_plat=%u\n",
			__func__, socinfo->v1.format, socinfo->v1.id,
			SOCINFO_VERSION_MAJOR(socinfo->v1.version),
			SOCINFO_VERSION_MINOR(socinfo->v1.version),
			socinfo->v2.raw_id, socinfo->v2.raw_version,
			socinfo->v3.hw_platform);
		break;
	case 4:
		pr_info("%s: v%u, id=%u, ver=%u.%u, "
			 "raw_id=%u, raw_ver=%u, hw_plat=%u, hw_plat_ver=%u\n",
			__func__, socinfo->v1.format, socinfo->v1.id,
			SOCINFO_VERSION_MAJOR(socinfo->v1.version),
			SOCINFO_VERSION_MINOR(socinfo->v1.version),
			socinfo->v2.raw_id, socinfo->v2.raw_version,
			socinfo->v3.hw_platform, socinfo->v4.platform_version);
		break;
	case 5:
		pr_info("%s: v%u, id=%u, ver=%u.%u, "
			 "raw_id=%u, raw_ver=%u, hw_plat=%u,  hw_plat_ver=%u\n"
			" accessory_chip=%u\n", __func__, socinfo->v1.format,
			socinfo->v1.id,
			SOCINFO_VERSION_MAJOR(socinfo->v1.version),
			SOCINFO_VERSION_MINOR(socinfo->v1.version),
			socinfo->v2.raw_id, socinfo->v2.raw_version,
			socinfo->v3.hw_platform, socinfo->v4.platform_version,
			socinfo->v5.accessory_chip);
		break;
	default:
		pr_err("%s: Unknown format found\n", __func__);
		break;
	}

	return 0;
}
