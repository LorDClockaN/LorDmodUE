/*
 * Copyright (C) 2005-2011 Junjiro R. Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * workqueue for asynchronous/super-io operations
 * todo: try new dredential scheme
 */

#include <linux/module.h>
#include "aufs.h"

/* internal workqueue named AUFS_WKQ_NAME and AUFS_WKQ_PRE_NAME */
enum {
	AuWkq_INORMAL,
	AuWkq_IPRE
};

static struct {
	char *name;
	struct workqueue_struct *wkq;
} au_wkq[] = {
	[AuWkq_INORMAL] = {
		.name = AUFS_WKQ_NAME
	},
	[AuWkq_IPRE] = {
		.name = AUFS_WKQ_PRE_NAME
	}
};

struct au_wkinfo {
	struct work_struct wk;
	struct kobject *kobj;

	unsigned int flags; /* see wkq.h */

	au_wkq_func_t func;
	void *args;

	struct completion *comp;
};

/* ---------------------------------------------------------------------- */

static void wkq_func(struct work_struct *wk)
{
	struct au_wkinfo *wkinfo = container_of(wk, struct au_wkinfo, wk);

	wkinfo->func(wkinfo->args);
	if (au_ftest_wkq(wkinfo->flags, WAIT))
		complete(wkinfo->comp);
	else {
		kobject_put(wkinfo->kobj);
		module_put(THIS_MODULE);
		kfree(wkinfo);
	}
}

/*
 * Since struct completion is large, try allocating it dynamically.
 */
#if defined(CONFIG_4KSTACKS) || defined(AuTest4KSTACKS)
#define AuWkqCompDeclare(name)	struct completion *comp = NULL

static int au_wkq_comp_alloc(struct au_wkinfo *wkinfo, struct completion **comp)
{
	*comp = kmalloc(sizeof(**comp), GFP_NOFS);
	if (*comp) {
		init_completion(*comp);
		wkinfo->comp = *comp;
		return 0;
	}
	return -ENOMEM;
}

static void au_wkq_comp_free(struct completion *comp)
{
	kfree(comp);
}

#else

/* no braces */
#define AuWkqCompDeclare(name) \
	DECLARE_COMPLETION_ONSTACK(_ ## name); \
	struct completion *comp = &_ ## name

static int au_wkq_comp_alloc(struct au_wkinfo *wkinfo, struct completion **comp)
{
	wkinfo->comp = *comp;
	return 0;
}

static void au_wkq_comp_free(struct completion *comp __maybe_unused)
{
	/* empty */
}
#endif /* 4KSTACKS */

static void au_wkq_run(struct au_wkinfo *wkinfo, unsigned int flags)
{
	struct workqueue_struct *wkq;

	au_dbg_verify_kthread();
	if (flags & AuWkq_WAIT) {
		INIT_WORK_ON_STACK(&wkinfo->wk, wkq_func);
		wkq = au_wkq[AuWkq_INORMAL].wkq;
		if (flags & AuWkq_PRE)
			wkq = au_wkq[AuWkq_IPRE].wkq;
		queue_work(wkq, &wkinfo->wk);
	} else {
		INIT_WORK(&wkinfo->wk, wkq_func);
		schedule_work(&wkinfo->wk);
	}
}

/*
 * Be careful. It is easy to make deadlock happen.
 * processA: lock, wkq and wait
 * processB: wkq and wait, lock in wkq
 * --> deadlock
 */
int au_wkq_do_wait(unsigned int flags, au_wkq_func_t func, void *args)
{
	int err;
	AuWkqCompDeclare(comp);
	struct au_wkinfo wkinfo = {
		.flags	= flags,
		.func	= func,
		.args	= args
	};

	err = au_wkq_comp_alloc(&wkinfo, &comp);
	if (!err) {
		au_wkq_run(&wkinfo, flags);
		/* no timeout, no interrupt */
		wait_for_completion(wkinfo.comp);
		au_wkq_comp_free(comp);
		destroy_work_on_stack(&wkinfo.wk);
	}

	return err;

}

/*
 * Note: dget/dput() in func for aufs dentries are not supported. It will be a
 * problem in a concurrent umounting.
 */
int au_wkq_nowait(au_wkq_func_t func, void *args, struct super_block *sb)
{
	int err;
	struct au_wkinfo *wkinfo;

	atomic_inc(&au_sbi(sb)->si_nowait.nw_len);

	/*
	 * wkq_func() must free this wkinfo.
	 * it highly depends upon the implementation of workqueue.
	 */
	err = 0;
	wkinfo = kmalloc(sizeof(*wkinfo), GFP_NOFS);
	if (wkinfo) {
		wkinfo->kobj = &au_sbi(sb)->si_kobj;
		wkinfo->flags = !AuWkq_WAIT;
		wkinfo->func = func;
		wkinfo->args = args;
		wkinfo->comp = NULL;
		kobject_get(wkinfo->kobj);
		__module_get(THIS_MODULE);

		au_wkq_run(wkinfo, !AuWkq_WAIT);
	} else {
		err = -ENOMEM;
		au_nwt_done(&au_sbi(sb)->si_nowait);
	}

	return err;
}

/* ---------------------------------------------------------------------- */

void au_nwt_init(struct au_nowait_tasks *nwt)
{
	atomic_set(&nwt->nw_len, 0);
	/* smp_mb(); */ /* atomic_set */
	init_waitqueue_head(&nwt->nw_wq);
}

void au_wkq_fin(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(au_wkq); i++)
		if (au_wkq[i].wkq)
			destroy_workqueue(au_wkq[i].wkq);
}

int __init au_wkq_init(void)
{
	int err, i;

	err = 0;
	for (i = 0; !err && i < ARRAY_SIZE(au_wkq); i++) {
		au_wkq[i].wkq = create_workqueue(au_wkq[i].name);
		if (IS_ERR(au_wkq[i].wkq))
			err = PTR_ERR(au_wkq[i].wkq);
		else if (!au_wkq[i].wkq)
			err = -ENOMEM;
		if (unlikely(err))
			au_wkq[i].wkq = NULL;
	}
	if (!err)
		au_dbg_verify_wkq();
	else
		au_wkq_fin();

	return err;
}
