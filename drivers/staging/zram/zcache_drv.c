/*
 * Page cache compression support
 *
 * Copyright (C) 2010  Nitin Gupta
 * Released under the terms of GNU General Public License Version 2.0
 *
 * Project home: http://compcache.googlecode.com/
 *
 * Design:
 * zcache is implemented using 'cleancache' which provides callbacks
 * (cleancache_ops) for events such as when a page is evicted from the
 * (uncompressed) page cache (cleancache_ops.put_page), when it is
 * needed again (cleancache_ops.get_page), when it needs to be freed
 * (cleancache_ops.flush_page) and so on.
 *
 * A page in zcache is identified with tuple <pool_id, inode_no, index>
 *  - pool_id: For every cleancache aware filesystem mounted, zcache
 * creates a separate pool (struct zcache_pool). This is container for
 * all metadata/data structures needed to locate pages cached for this
 * instance of mounted filesystem.
 *  - inode_no: This represents a file/inode within this filesystem. An
 * inode is represented using struct zcache_inode_rb within zcache which
 * are arranged in a red-black tree indexed using inode_no.
 *  - index: This represents page/index within an inode. Pages within an
 * inode are arranged in a radix tree. So, each node of red-black tree
 * above refers to a separate radix tree.
 *
 * Locking order:
 * 1. zcache_inode_rb->tree_lock	(spin_lock)
 * 2. zcache_pool->tree_lock		(rwlock_t: write)
 *
 * Nodes in an inode tree are reference counted and are freed when they
 * do not any pages i.e. corresponding radix tree, maintaining list of
 * pages associated with this inode, is empty.
 */

#define KMSG_COMPONENT "zcache"
#define pr_fmt(fmt) KMSG_COMPONENT ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cleancache.h>
#include <linux/cpu.h>
#include <linux/highmem.h>
#include <linux/lzo.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/u64_stats_sync.h>

#include "xvmalloc.h"
#include "zcache_drv.h"

static DEFINE_PER_CPU(unsigned char *, compress_buffer);
static DEFINE_PER_CPU(unsigned char *, compress_workmem);

/*
 * For zero-filled pages, we directly insert 'index' value
 * in corresponding radix node. These defines make sure we
 * do not try to store NULL value in radix node (index can
 * be 0) and that we do not use the lowest bit (which radix
 * tree uses for its own purpose).
 */
#define ZCACHE_ZERO_PAGE_INDEX_SHIFT	2
#define ZCACHE_ZERO_PAGE_MARK_BIT	(1 << 1)

struct zcache *zcache;

/*
 * Individual percpu values can go negative but the sum across all CPUs
 * must always be positive (we store various counts). So, return sum as
 * unsigned value.
 */
static u64 zcache_get_stat(struct zcache_pool *zpool,
		enum zcache_pool_stats_index idx)
{
	int cpu;
	s64 val = 0;

	for_each_possible_cpu(cpu) {
		unsigned int start;
		struct zcache_pool_stats_cpu *stats;

		stats = per_cpu_ptr(zpool->stats, cpu);
		do {
			start = u64_stats_fetch_begin(&stats->syncp);
			val += stats->count[idx];
		} while (u64_stats_fetch_retry(&stats->syncp, start));
	}

	BUG_ON(val < 0);
	return val;
}

static void zcache_add_stat(struct zcache_pool *zpool,
		enum zcache_pool_stats_index idx, s64 val)
{
	struct zcache_pool_stats_cpu *stats;

	preempt_disable();
	stats = __this_cpu_ptr(zpool->stats);
	u64_stats_update_begin(&stats->syncp);
	stats->count[idx] += val;
	u64_stats_update_end(&stats->syncp);
	preempt_enable();
}

static void zcache_inc_stat(struct zcache_pool *zpool,
		enum zcache_pool_stats_index idx)
{
	zcache_add_stat(zpool, idx, 1);
}

static void zcache_dec_stat(struct zcache_pool *zpool,
		enum zcache_pool_stats_index idx)
{
	zcache_add_stat(zpool, idx, -1);
}

static void zcache_dec_pages(struct zcache_pool *zpool, int is_zero)
{
	enum zcache_pool_stats_index idx;

	if (is_zero)
		idx = ZPOOL_STAT_PAGES_ZERO;
	else
		idx = ZPOOL_STAT_PAGES_STORED;

	zcache_dec_stat(zpool, idx);
}

static u64 zcache_get_memlimit(struct zcache_pool *zpool)
{
	u64 memlimit;
	unsigned int start;

	do {
		start = read_seqbegin(&zpool->memlimit_lock);
		memlimit = zpool->memlimit;
	} while (read_seqretry(&zpool->memlimit_lock, start));

	return memlimit;
}

static void zcache_set_memlimit(struct zcache_pool *zpool, u64 memlimit)
{
	write_seqlock(&zpool->memlimit_lock);
	zpool->memlimit = memlimit;
	write_sequnlock(&zpool->memlimit_lock);
}

static void zcache_dump_stats(struct zcache_pool *zpool)
{
	enum zcache_pool_stats_index idx;

	pr_debug("Dumping statistics for pool: %p\n", zpool);
	pr_debug("%llu ", zpool->memlimit);
	for (idx = 0; idx < ZPOOL_STAT_NSTATS; idx++)
		pr_debug("%llu ", zcache_get_stat(zpool, idx));
	pr_debug("\n");
}

static void zcache_destroy_pool(struct zcache_pool *zpool)
{
	int i;

	if (!zpool)
		return;

	spin_lock(&zcache->pool_lock);
	zcache->num_pools--;
	for (i = 0; i < MAX_ZCACHE_POOLS; i++)
		if (zcache->pools[i] == zpool)
			break;
	zcache->pools[i] = NULL;
	spin_unlock(&zcache->pool_lock);

	if (!RB_EMPTY_ROOT(&zpool->inode_tree)) {
		pr_warn("Memory leak detected. Freeing non-empty pool!\n");
		zcache_dump_stats(zpool);
	}

	free_percpu(zpool->stats);
	xv_destroy_pool(zpool->xv_pool);
	kfree(zpool);
}

/*
 * Allocate a new zcache pool and set default memlimit.
 *
 * Returns pool_id on success, negative error code otherwise.
 */
int zcache_create_pool(void)
{
	int ret;
	u64 memlimit;
	struct zcache_pool *zpool = NULL;

	spin_lock(&zcache->pool_lock);
	if (zcache->num_pools == MAX_ZCACHE_POOLS) {
		spin_unlock(&zcache->pool_lock);
		pr_info("Cannot create new pool (limit: %u)\n",
					MAX_ZCACHE_POOLS);
		ret = -EPERM;
		goto out;
	}
	zcache->num_pools++;
	spin_unlock(&zcache->pool_lock);

	zpool = kzalloc(sizeof(*zpool), GFP_KERNEL);
	if (!zpool) {
		spin_lock(&zcache->pool_lock);
		zcache->num_pools--;
		spin_unlock(&zcache->pool_lock);
		ret = -ENOMEM;
		goto out;
	}

	zpool->stats = alloc_percpu(struct zcache_pool_stats_cpu);
	if (!zpool->stats) {
		ret = -ENOMEM;
		goto out;
	}

	zpool->xv_pool = xv_create_pool();
	if (!zpool->xv_pool) {
		ret = -ENOMEM;
		goto out;
	}

	rwlock_init(&zpool->tree_lock);
	seqlock_init(&zpool->memlimit_lock);
	zpool->inode_tree = RB_ROOT;

	memlimit = zcache_pool_default_memlimit_perc_ram *
				((totalram_pages << PAGE_SHIFT) / 100);
	memlimit &= PAGE_MASK;
	zcache_set_memlimit(zpool, memlimit);

	/* Add to pool list */
	spin_lock(&zcache->pool_lock);
	for (ret = 0; ret < MAX_ZCACHE_POOLS; ret++)
		if (!zcache->pools[ret])
			break;
	zcache->pools[ret] = zpool;
	spin_unlock(&zcache->pool_lock);

out:
	if (ret < 0)
		zcache_destroy_pool(zpool);

	return ret;
}

/*
 * Allocate a new zcache node and insert it in given pool's
 * inode_tree at location 'inode_no'.
 *
 * On success, returns newly allocated node and increments
 * its refcount for caller. Returns NULL on failure.
 */
static struct zcache_inode_rb *zcache_inode_create(int pool_id,
						ino_t inode_no)
{
	unsigned long flags;
	struct rb_node *parent, **link;
	struct zcache_inode_rb *new_znode;
	struct zcache_pool *zpool = zcache->pools[pool_id];

	/*
	 * We can end up allocating multiple nodes due to racing
	 * zcache_put_page(). But only one will be added to zpool
	 * inode tree and rest will be freed.
	 *
	 * To avoid this possibility of redundant allocation, we
	 * could do it inside zpool tree lock. However, that seems
	 * more wasteful.
	 */
	new_znode = kzalloc(sizeof(*new_znode), GFP_NOWAIT);
	if (unlikely(!new_znode))
		return NULL;

	INIT_RADIX_TREE(&new_znode->page_tree, GFP_NOWAIT);
	spin_lock_init(&new_znode->tree_lock);
	kref_init(&new_znode->refcount);
	RB_CLEAR_NODE(&new_znode->rb_node);
	new_znode->inode_no = inode_no;
	new_znode->pool = zpool;

	parent = NULL;
	write_lock_irqsave(&zpool->tree_lock, flags);
	link = &zpool->inode_tree.rb_node;
	while (*link) {
		struct zcache_inode_rb *znode;

		znode = rb_entry(*link, struct zcache_inode_rb, rb_node);
		parent = *link;

		if (znode->inode_no > inode_no)
			link = &(*link)->rb_left;
		else if (znode->inode_no < inode_no)
			link = &(*link)->rb_right;
		else {
			/*
			 * New node added by racing zcache_put_page(). Free
			 * this newly allocated node and use the existing one.
			 */
			kfree(new_znode);
			new_znode = znode;
			goto out;
		}
	}

	rb_link_node(&new_znode->rb_node, parent, link);
	rb_insert_color(&new_znode->rb_node, &zpool->inode_tree);

out:
	kref_get(&new_znode->refcount);
	write_unlock_irqrestore(&zpool->tree_lock, flags);

	return new_znode;
}

/*
 * Called under zcache_inode_rb->tree_lock
 */
static int zcache_inode_is_empty(struct zcache_inode_rb *znode)
{
	return znode->page_tree.rnode == NULL;
}

/*
 * kref_put callback for zcache node.
 *
 * The node must have been isolated already.
 */
static void zcache_inode_release(struct kref *kref)
{
	struct zcache_inode_rb *znode;

	znode = container_of(kref, struct zcache_inode_rb, refcount);
	BUG_ON(!zcache_inode_is_empty(znode));
	kfree(znode);
}

/*
 * Removes the given node from its inode_tree and drops corresponding
 * refcount. Its called when someone removes the last page from a
 * zcache node.
 *
 * Called under zcache_inode_rb->spin_lock
 */
static void zcache_inode_isolate(struct zcache_inode_rb *znode)
{
	unsigned long flags;
	struct zcache_pool *zpool = znode->pool;

	write_lock_irqsave(&zpool->tree_lock, flags);
	/*
	 * Someone can get reference on this node before we could
	 * acquire write lock above. We want to remove it from its
	 * inode_tree when only the caller and corresponding inode_tree
	 * holds a reference to it. This ensures that a racing zcache
	 * put will not end up adding a page to an isolated node and
	 * thereby losing that memory.
	 *
	 */
	if (atomic_read(&znode->refcount.refcount) == 2) {
		rb_erase(&znode->rb_node, &znode->pool->inode_tree);
		RB_CLEAR_NODE(&znode->rb_node);
		kref_put(&znode->refcount, zcache_inode_release);
	}
	write_unlock_irqrestore(&zpool->tree_lock, flags);
}

/*
 * Find inode in the given pool at location 'inode_no'.
 *
 * If found, return the node pointer and increment its reference
 * count; NULL otherwise.
 */
static struct zcache_inode_rb *zcache_find_inode(struct zcache_pool *zpool,
						ino_t inode_no)
{
	unsigned long flags;
	struct rb_node *node;

	read_lock_irqsave(&zpool->tree_lock, flags);
	node = zpool->inode_tree.rb_node;
	while (node) {
		struct zcache_inode_rb *znode;

		znode = rb_entry(node, struct zcache_inode_rb, rb_node);
		if (znode->inode_no > inode_no)
			node = node->rb_left;
		else if (znode->inode_no < inode_no)
			node = node->rb_right;
		else {
			/* found */
			kref_get(&znode->refcount);
			read_unlock_irqrestore(&zpool->tree_lock, flags);
			return znode;
		}
	}
	read_unlock_irqrestore(&zpool->tree_lock, flags);

	/* not found */
	return NULL;
}

static void zcache_handle_zero_page(struct page *page)
{
	void *user_mem;

	user_mem = kmap_atomic(page, KM_USER0);
	memset(user_mem, 0, PAGE_SIZE);
	kunmap_atomic(user_mem, KM_USER0);

	flush_dcache_page(page);
}

static int zcache_page_zero_filled(void *ptr)
{
	unsigned int pos;
	unsigned long *page;

	page = (unsigned long *)ptr;

	for (pos = 0; pos != PAGE_SIZE / sizeof(*page); pos++) {
		if (page[pos])
			return 0;
	}

	return 1;
}


/*
 * Identifies if the given radix node pointer actually refers
 * to a zero-filled page.
 */
static int zcache_is_zero_page(void *ptr)
{
	return (unsigned long)(ptr) & ZCACHE_ZERO_PAGE_MARK_BIT;
}

/*
 * Returns "pointer" value to be stored in radix node for
 * zero-filled page at the given index.
 */
static void *zcache_index_to_ptr(unsigned long index)
{
	return (void *)((index << ZCACHE_ZERO_PAGE_INDEX_SHIFT) |
				ZCACHE_ZERO_PAGE_MARK_BIT);
}

/*
 * Encode <page, offset> as a single "pointer" value which is stored
 * in corresponding radix node.
 */
static void *zcache_xv_location_to_ptr(struct page *page, u32 offset)
{
	unsigned long ptrval;

	ptrval = page_to_pfn(page) << PAGE_SHIFT;
	ptrval |= (offset & ~PAGE_MASK);

	return (void *)ptrval;
}

/*
 * Decode <page, offset> pair from "pointer" value returned from
 * radix tree lookup.
 */
static void zcache_ptr_to_xv_location(void *ptr, struct page **page,
				u32 *offset)
{
	unsigned long ptrval = (unsigned long)ptr;

	*page = pfn_to_page(ptrval >> PAGE_SHIFT);
	*offset = ptrval & ~PAGE_MASK;
}

/*
 * Radix node contains "pointer" value which encode <page, offset>
 * pair, locating the compressed object. Header of the object then
 * contains corresponding 'index' value.
 */
static unsigned long zcache_ptr_to_index(void *ptr)
{
	u32 offset;
	struct page *page;
	unsigned long index;
	struct zcache_objheader *zheader;

	if (zcache_is_zero_page(ptr))
		return (unsigned long)(ptr) >> ZCACHE_ZERO_PAGE_INDEX_SHIFT;

	zcache_ptr_to_xv_location(ptr, &page, &offset);

	zheader = kmap_atomic(page, KM_USER0) + offset;
	index = zheader->index;
	kunmap_atomic(zheader, KM_USER0);

	return index;
}

void zcache_free_page(struct zcache_pool *zpool, void *ptr)
{
	int is_zero;
	unsigned long flags;

	if (unlikely(!ptr))
		return;

	is_zero = zcache_is_zero_page(ptr);
	if (!is_zero) {
		int clen;
		void *obj;
		u32 offset;
		struct page *page;

		zcache_ptr_to_xv_location(ptr, &page, &offset);
		obj = kmap_atomic(page, KM_USER0) + offset;
		clen = xv_get_object_size(obj) -
				sizeof(struct zcache_objheader);
		kunmap_atomic(obj, KM_USER0);

		zcache_add_stat(zpool, ZPOOL_STAT_COMPR_SIZE, -clen);
		local_irq_save(flags);
		xv_free(zpool->xv_pool, page, offset);
		local_irq_restore(flags);
	}

	zcache_dec_pages(zpool, is_zero);
}

/*
 * Allocate memory for storing the given page and insert
 * it in the given node's page tree at location 'index'.
 * Parameter 'is_zero' specifies if the page is zero-filled.
 *
 * Returns 0 on success, negative error code on failure.
 */
static int zcache_store_page(struct zcache_inode_rb *znode,
			pgoff_t index, struct page *page, int is_zero)
{
	int ret;
	void *nodeptr;
	size_t clen;
	unsigned long flags;

	u32 zoffset;
	struct page *zpage;
	unsigned char *zbuffer, *zworkmem;
	unsigned char *src_data, *dest_data;

	struct zcache_objheader *zheader;
	struct zcache_pool *zpool = znode->pool;

	if (is_zero) {
		nodeptr = zcache_index_to_ptr(index);
		goto out_store;
	}

	preempt_disable();
	zbuffer = __get_cpu_var(compress_buffer);
	zworkmem = __get_cpu_var(compress_workmem);
	if (unlikely(!zbuffer || !zworkmem)) {
		ret = -EFAULT;
		preempt_enable();
		goto out;
	}

	src_data = kmap_atomic(page, KM_USER0);
	ret = lzo1x_1_compress(src_data, PAGE_SIZE, zbuffer, &clen, zworkmem);
	kunmap_atomic(src_data, KM_USER0);

	if (unlikely(ret != LZO_E_OK) || clen > zcache_max_page_size) {
		ret = -EINVAL;
		preempt_enable();
		goto out;
	}

	local_irq_save(flags);
	ret = xv_malloc(zpool->xv_pool, clen + sizeof(*zheader),
			&zpage, &zoffset, GFP_NOWAIT);
	local_irq_restore(flags);
	if (unlikely(ret)) {
		ret = -ENOMEM;
		preempt_enable();
		goto out;
	}

	dest_data = kmap_atomic(zpage, KM_USER0) + zoffset;

	/* Store index value in header */
	zheader = (struct zcache_objheader *)dest_data;
	zheader->index = index;
	dest_data += sizeof(*zheader);

	memcpy(dest_data, zbuffer, clen);
	kunmap_atomic(dest_data, KM_USER0);
	preempt_enable();

	nodeptr = zcache_xv_location_to_ptr(zpage, zoffset);

out_store:
	spin_lock_irqsave(&znode->tree_lock, flags);
	ret = radix_tree_insert(&znode->page_tree, index, nodeptr);
	if (unlikely(ret)) {
		spin_unlock_irqrestore(&znode->tree_lock, flags);
		if (!is_zero)
			__free_page(zpage);
		goto out;
	}
	if (is_zero) {
		zcache_inc_stat(zpool, ZPOOL_STAT_PAGES_ZERO);
	} else {
		int delta = zcache_max_page_size - clen;
		zcache_add_stat(zpool, ZPOOL_STAT_COMPR_SIZE, -delta);
		zcache_inc_stat(zpool, ZPOOL_STAT_PAGES_STORED);
		radix_tree_tag_set(&znode->page_tree, index,
				ZCACHE_TAG_NONZERO_PAGE);
	}
	spin_unlock_irqrestore(&znode->tree_lock, flags);

	ret = 0; /* success */

out:
	return ret;
}

/*
 * Free 'pages_to_free' number of pages associated with the
 * given zcache node. Actual number of pages freed might be
 * less than this since the node might not contain enough
 * pages.
 *
 * Called under zcache_inode_rb->tree_lock
 * (Code adapted from brd.c)
 */
#define FREE_BATCH 16
static void zcache_free_inode_pages(struct zcache_inode_rb *znode,
				u32 pages_to_free, enum zcache_tag tag)
{
	int count;
	unsigned long index = 0;
	struct zcache_pool *zpool = znode->pool;

	do {
		int i;
		void *objs[FREE_BATCH];

		if (tag == ZCACHE_TAG_INVALID)
			count = radix_tree_gang_lookup(&znode->page_tree,
					objs, index, FREE_BATCH);
		else
			count = radix_tree_gang_lookup_tag(&znode->page_tree,
					objs, index, FREE_BATCH, tag);

		if (count > pages_to_free)
			count = pages_to_free;

		for (i = 0; i < count; i++) {
			void *obj;
			unsigned long index;

			index = zcache_ptr_to_index(objs[i]);
			obj = radix_tree_delete(&znode->page_tree, index);
			BUG_ON(obj != objs[i]);
			zcache_free_page(zpool, obj);
		}

		index++;
		pages_to_free -= count;
	} while (pages_to_free && (count == FREE_BATCH));
}

/*
 * Returns number of pages stored in excess of currently
 * set memlimit for the given pool.
 */
static u32 zcache_count_excess_pages(struct zcache_pool *zpool)
{
	u32 excess_pages, memlimit_pages, pages_stored;

	memlimit_pages = zcache_get_memlimit(zpool) >> PAGE_SHIFT;
	pages_stored = zcache_get_stat(zpool, ZPOOL_STAT_PAGES_STORED);
	excess_pages = pages_stored > memlimit_pages ?
			pages_stored - memlimit_pages : 0;

	return excess_pages;
}

/*
 * Free pages from this pool till we come within its memlimit.
 *
 * Currently, its called only when user sets memlimit lower than the
 * number of pages currently stored in that pool. We select nodes in
 * order of increasing inode number. This, in general, has no correlation
 * with the order in which these are added. So, it is essentially random
 * selection of nodes. Pages within a victim node node are freed in order
 * of increasing index number.
 *
 * Automatic cache resizing and better page replacement policies will
 * be implemented later.
 */
static void zcache_shrink_pool(struct zcache_pool *zpool)
{
	struct rb_node *node;
	struct zcache_inode_rb *znode;

	read_lock(&zpool->tree_lock);
	node = rb_first(&zpool->inode_tree);
	if (unlikely(!node)) {
		read_unlock(&zpool->tree_lock);
		return;
	}
	znode = rb_entry(node, struct zcache_inode_rb, rb_node);
	kref_get(&znode->refcount);
	read_unlock(&zpool->tree_lock);

	do {
		u32 pages_to_free;
		struct rb_node *next_node;
		struct zcache_inode_rb *next_znode;

		pages_to_free = zcache_count_excess_pages(zpool);
		if (!pages_to_free) {
			spin_lock(&znode->tree_lock);
			if (zcache_inode_is_empty(znode))
				zcache_inode_isolate(znode);
			spin_unlock(&znode->tree_lock);

			kref_put(&znode->refcount, zcache_inode_release);
			break;
		}

		/*
		 * Get the next victim node before we (possibly) isolate
		 * the current node.
		 */
		read_lock(&zpool->tree_lock);
		next_node = rb_next(node);
		next_znode = NULL;
		if (next_node) {
			next_znode = rb_entry(next_node,
				struct zcache_inode_rb, rb_node);
			kref_get(&next_znode->refcount);
		}
		read_unlock(&zpool->tree_lock);

		spin_lock(&znode->tree_lock);
		/* Free 'pages_to_free' non-zero pages in the current node */
		zcache_free_inode_pages(znode, pages_to_free,
					ZCACHE_TAG_NONZERO_PAGE);
		if (zcache_inode_is_empty(znode))
			zcache_inode_isolate(znode);
		spin_unlock(&znode->tree_lock);

		kref_put(&znode->refcount, zcache_inode_release);

		/* Avoid busy-looping */
		cond_resched();

		node = next_node;
		znode = next_znode;
	} while (znode);
}

#ifdef CONFIG_SYSFS

#define ZCACHE_POOL_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define ZCACHE_POOL_ATTR(_name) \
	static struct kobj_attribute _name##_attr = \
		__ATTR(_name, 0644, _name##_show, _name##_store)

static struct zcache_pool *zcache_kobj_to_pool(struct kobject *kobj)
{
	int i;

	spin_lock(&zcache->pool_lock);
	for (i = 0; i < MAX_ZCACHE_POOLS; i++)
		if (zcache->pools[i]->kobj == kobj)
			break;
	spin_unlock(&zcache->pool_lock);

	return zcache->pools[i];
}

static ssize_t zero_pages_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct zcache_pool *zpool = zcache_kobj_to_pool(kobj);

	return sprintf(buf, "%llu\n", zcache_get_stat(
			zpool, ZPOOL_STAT_PAGES_ZERO));
}
ZCACHE_POOL_ATTR_RO(zero_pages);

static ssize_t orig_data_size_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct zcache_pool *zpool = zcache_kobj_to_pool(kobj);

	return sprintf(buf, "%llu\n", zcache_get_stat(
			zpool, ZPOOL_STAT_PAGES_STORED) << PAGE_SHIFT);
}
ZCACHE_POOL_ATTR_RO(orig_data_size);

static ssize_t compr_data_size_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct zcache_pool *zpool = zcache_kobj_to_pool(kobj);

	return sprintf(buf, "%llu\n", zcache_get_stat(
			zpool, ZPOOL_STAT_COMPR_SIZE));
}
ZCACHE_POOL_ATTR_RO(compr_data_size);

/*
 * Total memory used by this pool, including allocator fragmentation
 * and metadata overhead.
 */
static ssize_t mem_used_total_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct zcache_pool *zpool = zcache_kobj_to_pool(kobj);

	return sprintf(buf, "%llu\n", xv_get_total_size_bytes(zpool->xv_pool));
}
ZCACHE_POOL_ATTR_RO(mem_used_total);

static void memlimit_sysfs_common(struct kobject *kobj, u64 *value, int store)
{
	struct zcache_pool *zpool = zcache_kobj_to_pool(kobj);

	if (store) {
		zcache_set_memlimit(zpool, *value);
		if (zcache_count_excess_pages(zpool))
			zcache_shrink_pool(zpool);
	} else {
		*value = zcache_get_memlimit(zpool);
	}
}

static ssize_t memlimit_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t len)
{
	int ret;
	u64 memlimit;

	ret = strict_strtoull(buf, 10, &memlimit);
	if (ret)
		return ret;

	memlimit &= PAGE_MASK;
	memlimit_sysfs_common(kobj, &memlimit, 1);

	return len;
}

static ssize_t memlimit_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	u64 memlimit;

	memlimit_sysfs_common(kobj, &memlimit, 0);
	return sprintf(buf, "%llu\n", memlimit);
}
ZCACHE_POOL_ATTR(memlimit);

static struct attribute *zcache_pool_attrs[] = {
	&zero_pages_attr.attr,
	&orig_data_size_attr.attr,
	&compr_data_size_attr.attr,
	&mem_used_total_attr.attr,
	&memlimit_attr.attr,
	NULL,
};

static struct attribute_group zcache_pool_attr_group = {
	.attrs = zcache_pool_attrs,
};
#endif	/* CONFIG_SYSFS */

/*
 * cleancache_ops.init_fs
 *
 * Called whenever a cleancache aware filesystem is mounted.
 * Creates and initializes a new zcache pool and inserts it
 * in zcache pool list.
 *
 * Returns pool id on success, negative error code on failure.
 */
static int zcache_init_fs(size_t pagesize)
{
	int ret, pool_id;
	struct zcache_pool *zpool = NULL;

	/*
	 * pagesize parameter probably makes sense only for Xen's
	 * cleancache_ops provider which runs inside guests, passing
	 * pages to the host. Since a guest might have a different
	 * page size than that of host (really?), they need to pass
	 * around this value.
	 *
	 * However, zcache runs on the host (or natively), so there
	 * is no point of different page sizes.
	 */
	if (pagesize != PAGE_SIZE) {
		pr_info("Unsupported page size: %zu", pagesize);
		ret = -EINVAL;
		goto out;
	}

	pool_id = zcache_create_pool();
	if (pool_id < 0) {
		pr_info("Failed to create new pool\n");
		ret = -ENOMEM;
		goto out;
	}
	zpool = zcache->pools[pool_id];

#ifdef CONFIG_SYSFS
	snprintf(zpool->name, MAX_ZPOOL_NAME_LEN, "pool%d", pool_id);

	/* Create /sys/kernel/mm/zcache/pool<id> (<id> = 0, 1, ...) */
	zpool->kobj = kobject_create_and_add(zpool->name, zcache->kobj);
	if (!zpool->kobj) {
		ret = -ENOMEM;
		goto out;
	}

	/* Create various nodes under /sys/.../pool<id>/ */
	ret = sysfs_create_group(zpool->kobj, &zcache_pool_attr_group);
	if (ret) {
		kobject_put(zpool->kobj);
		goto out;
	}
#endif

	ret = pool_id;	/* success */

out:
	if (ret < 0)	/* failure */
		zcache_destroy_pool(zpool);

	return ret;
}

/*
 * cleancache_ops.init_shared_fs
 *
 * Called whenever a cleancache aware clustered filesystem is mounted.
 */
static int zcache_init_shared_fs(char *uuid, size_t pagesize)
{
	/*
	 * In Xen's implementation, cleancache_ops provider runs in each
	 * guest, sending/receiving pages to/from the host. For each guest
	 * that participates in ocfs2 like cluster, the client passes the
	 * same uuid to the host. This allows the host to create a single
	 * "shared pool" for all such guests to allow for feature like
	 * data de-duplication among these guests.
	 *
	 * Whereas, zcache run directly on the host (or natively). So, for
	 * any shared resource like ocfs2 mount point on host, it implicitly
	 * creates a single pool. Thus, we can simply ignore this 'uuid' and
	 * treat it like a usual filesystem.
	 */
	return zcache_init_fs(pagesize);
}

/*
 * cleancache_ops.get_page
 *
 * Locates stored zcache page using <pool_id, inode_no, index>.
 * If found, copies it to the given output page 'page' and frees
 * zcache copy of the same.
 *
 * Returns 0 on success, negative error code on failure.
 */
static int zcache_get_page(int pool_id, ino_t inode_no,
			pgoff_t index, struct page *page)
{
	int ret;
	void *nodeptr;
	size_t clen;
	unsigned long flags;

	u32 offset;
	struct page *src_page;
	unsigned char *src_data, *dest_data;

	struct zcache_inode_rb *znode;
	struct zcache_objheader *zheader;
	struct zcache_pool *zpool = zcache->pools[pool_id];

	znode = zcache_find_inode(zpool, inode_no);
	if (!znode) {
		ret = -EFAULT;
		goto out;
	}

	BUG_ON(znode->inode_no != inode_no);

	spin_lock_irqsave(&znode->tree_lock, flags);
	nodeptr = radix_tree_delete(&znode->page_tree, index);
	if (zcache_inode_is_empty(znode))
		zcache_inode_isolate(znode);
	spin_unlock_irqrestore(&znode->tree_lock, flags);

	kref_put(&znode->refcount, zcache_inode_release);

	if (!nodeptr) {
		ret = -EFAULT;
		goto out;
	}

	if (zcache_is_zero_page(nodeptr)) {
		zcache_handle_zero_page(page);
		goto out_free;
	}

	clen = PAGE_SIZE;
	zcache_ptr_to_xv_location(nodeptr, &src_page, &offset);

	src_data = kmap_atomic(src_page, KM_USER0) + offset;
	zheader = (struct zcache_objheader *)src_data;
	BUG_ON(zheader->index != index);

	dest_data = kmap_atomic(page, KM_USER1);

	ret = lzo1x_decompress_safe(src_data + sizeof(*zheader),
			xv_get_object_size(src_data) - sizeof(*zheader),
			dest_data, &clen);

	kunmap_atomic(src_data, KM_USER0);
	kunmap_atomic(dest_data, KM_USER1);

	/* Failure here means bug in LZO! */
	if (unlikely(ret != LZO_E_OK))
		goto out_free;

	flush_dcache_page(page);

out_free:
	zcache_free_page(zpool, nodeptr);
	ret = 0; /* success */

out:
	return ret;
}

/*
 * cleancache_ops.put_page
 *
 * Copies given input page 'page' to a newly allocated page.
 * If allocation is successful, inserts it at zcache location
 * <pool_id, inode_no, index>.
 */
static void zcache_put_page(int pool_id, ino_t inode_no,
			pgoff_t index, struct page *page)
{
	int ret, is_zero;
	unsigned long flags;
	struct page *zpage;
	struct zcache_inode_rb *znode;
	struct zcache_pool *zpool = zcache->pools[pool_id];

	/*
	 * Check if the page is zero-filled. We do not allocate any
	 * memory for such pages and hence they do not contribute
	 * towards pool's memory usage. So, we can keep accepting
	 * such pages even after we have reached memlimit.
	 */
	void *src_data = kmap_atomic(page, KM_USER0);
	is_zero = zcache_page_zero_filled(src_data);
	kunmap_atomic(src_data, KM_USER0);
	if (is_zero)
		goto out_find_store;

	/*
	 * Incrementing local compr_size before summing it from
	 * all CPUs makes sure we do not end up storing pages in
	 * excess of memlimit. In case of failure, we revert back
	 * this local increment.
	 */
	zcache_add_stat(zpool, ZPOOL_STAT_COMPR_SIZE, zcache_max_page_size);

	/*
	 * memlimit can be changed any time by user using sysfs. If
	 * it is set to a value smaller than current number of pages
	 * stored, then excess pages are freed synchronously when this
	 * sysfs event occurs.
	 */
	if (zcache_get_stat(zpool, ZPOOL_STAT_COMPR_SIZE) >
			zcache_get_memlimit(zpool)) {
		zcache_add_stat(zpool, ZPOOL_STAT_COMPR_SIZE,
				-zcache_max_page_size);
		return;
	}

out_find_store:
	znode = zcache_find_inode(zpool, inode_no);
	if (!znode) {
		znode = zcache_inode_create(pool_id, inode_no);
		if (unlikely(!znode)) {
			zcache_add_stat(zpool, ZPOOL_STAT_COMPR_SIZE,
					-zcache_max_page_size);
			return;
		}
	}

	/* Free page that might already be present at this index */
	spin_lock_irqsave(&znode->tree_lock, flags);
	zpage = radix_tree_delete(&znode->page_tree, index);
	spin_unlock_irqrestore(&znode->tree_lock, flags);
	if (zpage)
		zcache_free_page(zpool, zpage);

	ret = zcache_store_page(znode, index, page, is_zero);
	if (unlikely(ret)) {
		zcache_add_stat(zpool, ZPOOL_STAT_COMPR_SIZE,
				-zcache_max_page_size);
		/*
		 * Its possible that racing zcache get/flush could not
		 * isolate this node since we held a reference to it.
		 */
		spin_lock_irqsave(&znode->tree_lock, flags);
		if (zcache_inode_is_empty(znode))
			zcache_inode_isolate(znode);
		spin_unlock_irqrestore(&znode->tree_lock, flags);
	}

	kref_put(&znode->refcount, zcache_inode_release);
}

/*
 * cleancache_ops.flush_page
 *
 * Locates and fees page at zcache location <pool_id, inode_no, index>
 */
static void zcache_flush_page(int pool_id, ino_t inode_no, pgoff_t index)
{
	unsigned long flags;
	struct page *page;
	struct zcache_inode_rb *znode;
	struct zcache_pool *zpool = zcache->pools[pool_id];

	znode = zcache_find_inode(zpool, inode_no);
	if (!znode)
		return;

	spin_lock_irqsave(&znode->tree_lock, flags);
	page = radix_tree_delete(&znode->page_tree, index);
	if (zcache_inode_is_empty(znode))
		zcache_inode_isolate(znode);
	spin_unlock_irqrestore(&znode->tree_lock, flags);

	kref_put(&znode->refcount, zcache_inode_release);

	zcache_free_page(zpool, page);
}

/*
 * cleancache_ops.flush_inode
 *
 * Free all pages associated with the given inode.
 */
static void zcache_flush_inode(int pool_id, ino_t inode_no)
{
	unsigned long flags;
	struct zcache_pool *zpool;
	struct zcache_inode_rb *znode;

	zpool = zcache->pools[pool_id];

	znode = zcache_find_inode(zpool, inode_no);
	if (!znode)
		return;

	spin_lock_irqsave(&znode->tree_lock, flags);
	zcache_free_inode_pages(znode, UINT_MAX, ZCACHE_TAG_INVALID);
	if (zcache_inode_is_empty(znode))
		zcache_inode_isolate(znode);
	spin_unlock_irqrestore(&znode->tree_lock, flags);

	kref_put(&znode->refcount, zcache_inode_release);
}

/*
 * cleancache_ops.flush_fs
 *
 * Called whenever a cleancache aware filesystem is unmounted.
 * Frees all metadata and data pages in corresponding zcache pool.
 */
static void zcache_flush_fs(int pool_id)
{
	struct rb_node *node;
	struct zcache_inode_rb *znode;
	struct zcache_pool *zpool = zcache->pools[pool_id];

#ifdef CONFIG_SYSFS
	/* Remove per-pool sysfs entries */
	sysfs_remove_group(zpool->kobj, &zcache_pool_attr_group);
	kobject_put(zpool->kobj);
#endif

	/*
	 * At this point, there is no active I/O on this filesystem.
	 * So we can free all its pages without holding any locks.
	 */
	node = rb_first(&zpool->inode_tree);
	while (node) {
		znode = rb_entry(node, struct zcache_inode_rb, rb_node);
		node = rb_next(node);
		zcache_free_inode_pages(znode, UINT_MAX, ZCACHE_TAG_INVALID);
		rb_erase(&znode->rb_node, &zpool->inode_tree);
		kfree(znode);
	}

	zcache_destroy_pool(zpool);
}

/*
 * Callback for CPU hotplug events. Allocates percpu compression buffers.
 */
static int zcache_cpu_notify(struct notifier_block *nb, unsigned long action,
			void *pcpu)
{
	int cpu = (long)pcpu;

	switch (action) {
	case CPU_UP_PREPARE:
		per_cpu(compress_buffer, cpu) = (void *)__get_free_pages(
					GFP_KERNEL | __GFP_ZERO, 1);
		per_cpu(compress_workmem, cpu) = kzalloc(
					LZO1X_MEM_COMPRESS, GFP_KERNEL);

		break;
	case CPU_DEAD:
	case CPU_UP_CANCELED:
		free_pages((unsigned long)(per_cpu(compress_buffer, cpu)), 1);
		per_cpu(compress_buffer, cpu) = NULL;

		kfree(per_cpu(compress_buffer, cpu));
		per_cpu(compress_buffer, cpu) = NULL;

		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block zcache_cpu_nb = {
	.notifier_call = zcache_cpu_notify
};

static int __init zcache_init(void)
{
	int ret = -ENOMEM;
	unsigned int cpu;

	struct cleancache_ops ops = {
		.init_fs = zcache_init_fs,
		.init_shared_fs = zcache_init_shared_fs,
		.get_page = zcache_get_page,
		.put_page = zcache_put_page,
		.flush_page = zcache_flush_page,
		.flush_inode = zcache_flush_inode,
		.flush_fs = zcache_flush_fs,
	};

	zcache = kzalloc(sizeof(*zcache), GFP_KERNEL);
	if (!zcache)
		goto out;

	ret = register_cpu_notifier(&zcache_cpu_nb);
	if (ret)
		goto out;

	for_each_online_cpu(cpu) {
		void *pcpu = (void *)(long)cpu;
		zcache_cpu_notify(&zcache_cpu_nb, CPU_UP_PREPARE, pcpu);
	}

#ifdef CONFIG_SYSFS
	/* Create /sys/kernel/mm/zcache/ */
	zcache->kobj = kobject_create_and_add("zcache", mm_kobj);
	if (!zcache->kobj)
		goto out;
#endif

	spin_lock_init(&zcache->pool_lock);
	cleancache_ops = ops;

	ret = 0; /* success */

out:
	if (ret)
		kfree(zcache);

	return 0;
}

module_init(zcache_init);
