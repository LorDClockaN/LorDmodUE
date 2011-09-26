/*
 * Page cache compression support
 *
 * Copyright (C) 2010  Nitin Gupta
 * Released under the terms of GNU General Public License Version 2.0
 *
 * Project home: http://compcache.googlecode.com/
 */

#ifndef _ZCACHE_DRV_H_
#define _ZCACHE_DRV_H_

#include <linux/kref.h>
#include <linux/radix-tree.h>
#include <linux/rbtree.h>
#include <linux/rwlock.h>
#include <linux/spinlock.h>
#include <linux/seqlock.h>
#include <linux/types.h>

#define MAX_ZCACHE_POOLS	32	/* arbitrary */
#define MAX_ZPOOL_NAME_LEN	8	/* "pool"+id (shown in sysfs) */

enum zcache_pool_stats_index {
	ZPOOL_STAT_PAGES_ZERO,
	ZPOOL_STAT_PAGES_STORED,
	ZPOOL_STAT_COMPR_SIZE,
	ZPOOL_STAT_NSTATS,
};

/* Radix-tree tags */
enum zcache_tag {
	ZCACHE_TAG_NONZERO_PAGE,
	ZCACHE_TAG_UNUSED,
	ZCACHE_TAG_INVALID
};

/* Default zcache per-pool memlimit: 10% of total RAM */
static const unsigned zcache_pool_default_memlimit_perc_ram = 10;

 /* We only keep pages that compress to less than this size */
static const int zcache_max_page_size = PAGE_SIZE / 2;

/* Stored in the beginning of each compressed object */
struct zcache_objheader {
	unsigned long index;
};

/* Red-Black tree node. Maps inode to its page-tree */
struct zcache_inode_rb {
	struct radix_tree_root page_tree; /* maps inode index to page */
	spinlock_t tree_lock;		/* protects page_tree */
	struct kref refcount;
	struct rb_node rb_node;
	ino_t inode_no;
	struct zcache_pool *pool;	/* back-reference to parent pool */
};

struct zcache_pool_stats_cpu {
	s64 count[ZPOOL_STAT_NSTATS];
	struct u64_stats_sync syncp;
};

/* One zcache pool per (cleancache aware) filesystem mount instance */
struct zcache_pool {
	struct rb_root inode_tree;	/* maps inode number to page tree */
	rwlock_t tree_lock;		/* protects inode_tree */

	seqlock_t memlimit_lock;	/* protects memlimit */
	u64 memlimit;			/* bytes */

	struct xv_pool *xv_pool;	/* xvmalloc pool */
	struct zcache_pool_stats_cpu *stats;	/* percpu stats */
#ifdef CONFIG_SYSFS
	unsigned char name[MAX_ZPOOL_NAME_LEN];
	struct kobject *kobj;		/* sysfs */
#endif
};

/* Manage all zcache pools */
struct zcache {
	struct zcache_pool *pools[MAX_ZCACHE_POOLS];
	u32 num_pools;		/* current no. of zcache pools */
	spinlock_t pool_lock;	/* protects pools[] and num_pools */
#ifdef CONFIG_SYSFS
	struct kobject *kobj;	/* sysfs */
#endif
};

#endif
