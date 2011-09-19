#ifndef _LINUX_CLEANCACHE_H
#define _LINUX_CLEANCACHE_H

#include <linux/fs.h>
#include <linux/mm.h>

struct cleancache_ops {
	int (*init_fs)(size_t);
	int (*init_shared_fs)(char *uuid, size_t);
	int (*get_page)(int, ino_t, pgoff_t, struct page *);
	void (*put_page)(int, ino_t, pgoff_t, struct page *);
	void (*flush_page)(int, ino_t, pgoff_t);
	void (*flush_inode)(int, ino_t);
	void (*flush_fs)(int);
};

extern struct cleancache_ops cleancache_ops;
extern int __cleancache_get_page(struct page *);
extern void __cleancache_put_page(struct page *);
extern void __cleancache_flush_page(struct address_space *, struct page *);
extern void __cleancache_flush_inode(struct address_space *);

#ifdef CONFIG_CLEANCACHE
#define cleancache_enabled (cleancache_ops.init_fs)
#else
#define cleancache_enabled (0)
#endif

/* called by a cleancache-enabled filesystem at time of mount */
static inline int cleancache_init_fs(size_t pagesize)
{
	int ret = -1;

	if (cleancache_enabled)
		ret = (*cleancache_ops.init_fs)(pagesize);
	return ret;
}

/* called by a cleancache-enabled clustered filesystem at time of mount */
static inline int cleancache_init_shared_fs(char *uuid, size_t pagesize)
{
	int ret = -1;

	if (cleancache_enabled)
		ret = (*cleancache_ops.init_shared_fs)(uuid, pagesize);
	return ret;
}

static inline int cleancache_get_page(struct page *page)
{
	int ret = -1;

	if (cleancache_enabled)
		ret = __cleancache_get_page(page);
	return ret;
}

static inline void cleancache_put_page(struct page *page)
{
	if (cleancache_enabled)
		__cleancache_put_page(page);
}

static inline void cleancache_flush_page(struct address_space *mapping,
					struct page *page)
{
	if (cleancache_enabled)
		__cleancache_flush_page(mapping, page);
}

static inline void cleancache_flush_inode(struct address_space *mapping)
{
	if (cleancache_enabled)
		__cleancache_flush_inode(mapping);
}

/*
 * called by any cleancache-enabled filesystem at time of unmount;
 * note that pool_id is surrendered and may be returned by a subsequent
 * cleancache_init_fs or cleancache_init_shared_fs
 */
static inline void cleancache_flush_fs(int pool_id)
{
	if (cleancache_enabled && pool_id >= 0)
		(*cleancache_ops.flush_fs)(pool_id);
}

#endif /* _LINUX_CLEANCACHE_H */
