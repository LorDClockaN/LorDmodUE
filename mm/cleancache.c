/*
 * Cleancache frontend
 *
 * This code provides the generic "frontend" layer to call a matching
 * "backend" driver implementation of cleancache.  See
 * Documentation/vm/cleancache.txt for more information.
 *
 * Copyright (C) 2009-2010 Oracle Corp. All rights reserved.
 * Author: Dan Magenheimer
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cleancache.h>

/*
 * cleancache_ops contains the pointers to the cleancache "backend"
 * implementation functions
 */
struct cleancache_ops cleancache_ops;
EXPORT_SYMBOL(cleancache_ops);

/* useful stats available in /sys/kernel/mm/cleancache */
static unsigned long succ_gets;
static unsigned long failed_gets;
static unsigned long puts;
static unsigned long flushes;

/*
 * "Get" data from cleancache associated with the poolid/inode/index
 * that were specified when the data was put to cleanache and, if
 * successful, use it to fill the specified page with data and return 0.
 * The pageframe is unchanged and returns -1 if the get fails.
 * Page must be locked by caller.
 */
int __cleancache_get_page(struct page *page)
{
	int ret = -1;
	int pool_id;

	VM_BUG_ON(!PageLocked(page));
	pool_id = page->mapping->host->i_sb->cleancache_poolid;
	if (pool_id >= 0) {
		ret = (*cleancache_ops.get_page)(pool_id,
						 page->mapping->host->i_ino,
						 page->index,
						 page);
		if (ret == 0)
			succ_gets++;
		else
			failed_gets++;
	}
	return ret;
}
EXPORT_SYMBOL(__cleancache_get_page);

/*
 * "Put" data from a page to cleancache and associate it with the
 * (previously-obtained per-filesystem) poolid and the page's,
 * inode and page index.  Page must be locked.  Note that a put_page
 * always "succeeds", though a subsequent get_page may succeed or fail.
 */
void __cleancache_put_page(struct page *page)
{
	int pool_id;

	VM_BUG_ON(!PageLocked(page));
	pool_id = page->mapping->host->i_sb->cleancache_poolid;
	if (pool_id >= 0) {
		(*cleancache_ops.put_page)(pool_id, page->mapping->host->i_ino,
					   page->index, page);
		puts++;
	}
}

/*
 * Flush any data from cleancache associated with the poolid and the
 * page's inode and page index so that a subsequent "get" will fail.
 */
void __cleancache_flush_page(struct address_space *mapping, struct page *page)
{
	int pool_id = mapping->host->i_sb->cleancache_poolid;

	if (pool_id >= 0) {
		VM_BUG_ON(!PageLocked(page));
		(*cleancache_ops.flush_page)(pool_id, mapping->host->i_ino,
					     page->index);
		flushes++;
	}
}
EXPORT_SYMBOL(__cleancache_flush_page);

/*
 * Flush all data from cleancache associated with the poolid and the
 * mappings's inode so that all subsequent gets to this poolid/inode
 * will fail.
 */
void __cleancache_flush_inode(struct address_space *mapping)
{
	int pool_id = mapping->host->i_sb->cleancache_poolid;

	if (pool_id >= 0)
		(*cleancache_ops.flush_inode)(pool_id, mapping->host->i_ino);
}
EXPORT_SYMBOL(__cleancache_flush_inode);

#ifdef CONFIG_SYSFS

/* see Documentation/ABI/xxx/sysfs-kernel-mm-cleancache */

#define CLEANCACHE_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

static ssize_t succ_gets_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", succ_gets);
}
CLEANCACHE_ATTR_RO(succ_gets);

static ssize_t failed_gets_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", failed_gets);
}
CLEANCACHE_ATTR_RO(failed_gets);

static ssize_t puts_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", puts);
}
CLEANCACHE_ATTR_RO(puts);

static ssize_t flushes_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", flushes);
}
CLEANCACHE_ATTR_RO(flushes);

static struct attribute *cleancache_attrs[] = {
	&succ_gets_attr.attr,
	&failed_gets_attr.attr,
	&puts_attr.attr,
	&flushes_attr.attr,
	NULL,
};

static struct attribute_group cleancache_attr_group = {
	.attrs = cleancache_attrs,
	.name = "cleancache",
};

#endif /* CONFIG_SYSFS */

static int __init init_cleancache(void)
{
#ifdef CONFIG_SYSFS
	int err;

	err = sysfs_create_group(mm_kobj, &cleancache_attr_group);
#endif /* CONFIG_SYSFS */
	return 0;
}
module_init(init_cleancache)
