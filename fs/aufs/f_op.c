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
 * file and vm operations
 */

#include <linux/file.h>
#include <linux/fs_stack.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/security.h>
#include "aufs.h"

int au_do_open_nondir(struct file *file, int flags)
{
	int err;
	aufs_bindex_t bindex;
	struct file *h_file;
	struct dentry *dentry;
	struct au_finfo *finfo;

	FiMustWriteLock(file);

	dentry = file->f_dentry;
	err = au_d_alive(dentry);
	if (unlikely(err))
		goto out;

	finfo = au_fi(file);
	memset(&finfo->fi_htop, 0, sizeof(finfo->fi_htop));
	finfo->fi_hvmop = NULL;
	bindex = au_dbstart(dentry);
	h_file = au_h_open(dentry, bindex, flags, file);
	if (IS_ERR(h_file))
		err = PTR_ERR(h_file);
	else {
		au_set_fbstart(file, bindex);
		au_set_h_fptr(file, bindex, h_file);
		au_update_figen(file);
		/* todo: necessary? */
		/* file->f_ra = h_file->f_ra; */
	}

out:
	return err;
}

static int aufs_open_nondir(struct inode *inode __maybe_unused,
			    struct file *file)
{
	int err;
	struct super_block *sb;

	AuDbg("%.*s, f_flags 0x%x, f_mode 0x%x\n",
	      AuDLNPair(file->f_dentry), vfsub_file_flags(file),
	      file->f_mode);

	sb = file->f_dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_do_open(file, au_do_open_nondir, /*fidir*/NULL);
	si_read_unlock(sb);
	return err;
}

int aufs_release_nondir(struct inode *inode __maybe_unused, struct file *file)
{
	struct au_finfo *finfo;
	aufs_bindex_t bindex;

	finfo = au_fi(file);
	bindex = finfo->fi_btop;
	if (bindex >= 0) {
		/* remove me from sb->s_files */
		file_kill(file);
		au_set_h_fptr(file, bindex, NULL);
	}

	au_finfo_fin(file);
	return 0;
}

/* ---------------------------------------------------------------------- */

static int au_do_flush_nondir(struct file *file, fl_owner_t id)
{
	int err;
	struct file *h_file;

	err = 0;
	h_file = au_hf_top(file);
	if (h_file)
		err = vfsub_flush(h_file, id);
	return err;
}

static int aufs_flush_nondir(struct file *file, fl_owner_t id)
{
	return au_do_flush(file, id, au_do_flush_nondir);
}

/* ---------------------------------------------------------------------- */

static ssize_t aufs_read(struct file *file, char __user *buf, size_t count,
			 loff_t *ppos)
{
	ssize_t err;
	struct dentry *dentry;
	struct file *h_file;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH | AuLock_NOPLMW);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	h_file = au_hf_top(file);
	err = vfsub_read_u(h_file, buf, count, ppos);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);
out:
	si_read_unlock(sb);
	return err;
}

/*
 * todo: very ugly
 * it locks both of i_mutex and si_rwsem for read in safe.
 * if the plink maintenance mode continues forever (that is the problem),
 * may loop forever.
 */
static void au_mtx_and_read_lock(struct inode *inode)
{
	int err;
	struct super_block *sb = inode->i_sb;

	while (1) {
		mutex_lock(&inode->i_mutex);
		err = si_read_lock(sb, AuLock_FLUSH | AuLock_NOPLM);
		if (!err)
			break;
		mutex_unlock(&inode->i_mutex);
		si_read_lock(sb, AuLock_NOPLMW);
		si_read_unlock(sb);
	}
}

static ssize_t aufs_write(struct file *file, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	ssize_t err;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct file *h_file;
	char __user *buf = (char __user *)ubuf;

	dentry = file->f_dentry;
	inode = dentry->d_inode;
	au_mtx_and_read_lock(inode);

	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	h_file = au_hf_top(file);
	au_unpin(&pin);
	err = vfsub_write_u(h_file, buf, count, ppos);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;

out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
out:
	si_read_unlock(inode->i_sb);
	mutex_unlock(&inode->i_mutex);
	return err;
}

static ssize_t au_do_aio(struct file *h_file, int rw, struct kiocb *kio,
			 const struct iovec *iov, unsigned long nv, loff_t pos)
{
	ssize_t err;
	struct file *file;
	ssize_t (*func)(struct kiocb *, const struct iovec *, unsigned long,
			loff_t);

	err = security_file_permission(h_file, rw);
	if (unlikely(err))
		goto out;

	err = -ENOSYS;
	func = NULL;
	if (rw == MAY_READ)
		func = h_file->f_op->aio_read;
	else if (rw == MAY_WRITE)
		func = h_file->f_op->aio_write;
	if (func) {
		file = kio->ki_filp;
		kio->ki_filp = h_file;
		err = func(kio, iov, nv, pos);
		kio->ki_filp = file;
	} else
		/* currently there is no such fs */
		WARN_ON_ONCE(1);

out:
	return err;
}

static ssize_t aufs_aio_read(struct kiocb *kio, const struct iovec *iov,
			     unsigned long nv, loff_t pos)
{
	ssize_t err;
	struct file *file, *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	file = kio->ki_filp;
	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH | AuLock_NOPLMW);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	h_file = au_hf_top(file);
	err = au_do_aio(h_file, MAY_READ, kio, iov, nv, pos);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);
	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

out:
	si_read_unlock(sb);
	return err;
}

static ssize_t aufs_aio_write(struct kiocb *kio, const struct iovec *iov,
			      unsigned long nv, loff_t pos)
{
	ssize_t err;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct file *file, *h_file;

	file = kio->ki_filp;
	dentry = file->f_dentry;
	inode = dentry->d_inode;
	au_mtx_and_read_lock(inode);

	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	au_unpin(&pin);
	h_file = au_hf_top(file);
	err = au_do_aio(h_file, MAY_WRITE, kio, iov, nv, pos);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;

out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
out:
	si_read_unlock(inode->i_sb);
	mutex_unlock(&inode->i_mutex);
	return err;
}

static ssize_t aufs_splice_read(struct file *file, loff_t *ppos,
				struct pipe_inode_info *pipe, size_t len,
				unsigned int flags)
{
	ssize_t err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH | AuLock_NOPLMW);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	err = -EINVAL;
	h_file = au_hf_top(file);
	if (au_test_loopback_kthread()) {
		file->f_mapping = h_file->f_mapping;
		smp_mb(); /* unnecessary? */
	}
	err = vfsub_splice_to(h_file, ppos, pipe, len, flags);
	/* todo: necessasry? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

out:
	si_read_unlock(sb);
	return err;
}

static ssize_t
aufs_splice_write(struct pipe_inode_info *pipe, struct file *file, loff_t *ppos,
		  size_t len, unsigned int flags)
{
	ssize_t err;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct file *h_file;

	dentry = file->f_dentry;
	inode = dentry->d_inode;
	au_mtx_and_read_lock(inode);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	h_file = au_hf_top(file);
	au_unpin(&pin);
	err = vfsub_splice_from(pipe, h_file, ppos, len, flags);
	au_cpup_attr_timesizes(inode);
	inode->i_mode = h_file->f_dentry->d_inode->i_mode;

out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
out:
	si_read_unlock(inode->i_sb);
	mutex_unlock(&inode->i_mutex);
	return err;
}

/* ---------------------------------------------------------------------- */

static struct file *au_safe_file(struct vm_area_struct *vma)
{
	struct file *file;

	file = vma->vm_file;
	if (au_fi(file) && au_test_aufs(file->f_dentry->d_sb))
		return file;
	return NULL;
}

static void au_reset_file(struct vm_area_struct *vma, struct file *file)
{
	vma->vm_file = file;
	/* smp_mb(); */ /* flush vm_file */
}

static int aufs_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	struct file *file, *h_file;
	struct au_finfo *finfo;

	/* todo: non-robr mode, user vm_file as it is? */
	wait_event(wq, (file = au_safe_file(vma)));

	/* do not revalidate, no si lock */
	finfo = au_fi(file);
	AuDebugOn(finfo->fi_hdir);
	h_file = finfo->fi_htop.hf_file;
	AuDebugOn(!h_file || !finfo->fi_hvmop);

	mutex_lock(&finfo->fi_vm_mtx);
	vma->vm_file = h_file;
	err = finfo->fi_hvmop->fault(vma, vmf);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	au_reset_file(vma, file);
	mutex_unlock(&finfo->fi_vm_mtx);
#if 0 /* def CONFIG_SMP */
	/* wake_up_nr(&wq, online_cpu - 1); */
	wake_up_all(&wq);
#else
	wake_up(&wq);
#endif

	return err;
}

static int aufs_page_mkwrite(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	struct file *file, *h_file;
	struct au_finfo *finfo;

	wait_event(wq, (file = au_safe_file(vma)));

	finfo = au_fi(file);
	AuDebugOn(finfo->fi_hdir);
	h_file = finfo->fi_htop.hf_file;
	AuDebugOn(!h_file || !finfo->fi_hvmop);

	mutex_lock(&finfo->fi_vm_mtx);
	vma->vm_file = h_file;
	err = finfo->fi_hvmop->page_mkwrite(vma, vmf);
	au_reset_file(vma, file);
	mutex_unlock(&finfo->fi_vm_mtx);
	wake_up(&wq);

	return err;
}

static void aufs_vm_close(struct vm_area_struct *vma)
{
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	struct file *file, *h_file;
	struct au_finfo *finfo;

	wait_event(wq, (file = au_safe_file(vma)));

	finfo = au_fi(file);
	AuDebugOn(finfo->fi_hdir);
	h_file = finfo->fi_htop.hf_file;
	AuDebugOn(!h_file || !finfo->fi_hvmop);

	mutex_lock(&finfo->fi_vm_mtx);
	vma->vm_file = h_file;
	finfo->fi_hvmop->close(vma);
	au_reset_file(vma, file);
	mutex_unlock(&finfo->fi_vm_mtx);
	wake_up(&wq);
}

const struct vm_operations_struct aufs_vm_ops = {
	.close		= aufs_vm_close,
	.fault		= aufs_fault,
	.page_mkwrite	= aufs_page_mkwrite
};

/* ---------------------------------------------------------------------- */

/* cf. linux/include/linux/mman.h: calc_vm_prot_bits() */
#define AuConv_VM_PROT(f, b)	_calc_vm_trans(f, VM_##b, PROT_##b)

static unsigned long au_arch_prot_conv(unsigned long flags)
{
	/* currently ppc64 only */
#ifdef CONFIG_PPC64
	/* cf. linux/arch/powerpc/include/asm/mman.h */
	AuDebugOn(arch_calc_vm_prot_bits(-1) != VM_SAO);
	return AuConv_VM_PROT(flags, SAO);
#else
	AuDebugOn(arch_calc_vm_prot_bits(-1));
	return 0;
#endif
}

static unsigned long au_prot_conv(unsigned long flags)
{
	return AuConv_VM_PROT(flags, READ)
		| AuConv_VM_PROT(flags, WRITE)
		| AuConv_VM_PROT(flags, EXEC)
		| au_arch_prot_conv(flags);
}

/* cf. linux/include/linux/mman.h: calc_vm_flag_bits() */
#define AuConv_VM_MAP(f, b)	_calc_vm_trans(f, VM_##b, MAP_##b)

static unsigned long au_flag_conv(unsigned long flags)
{
	return AuConv_VM_MAP(flags, GROWSDOWN)
		| AuConv_VM_MAP(flags, DENYWRITE)
		| AuConv_VM_MAP(flags, EXECUTABLE)
		| AuConv_VM_MAP(flags, LOCKED);
}

static struct vm_operations_struct *
au_hvmop(struct file *h_file, struct vm_area_struct *vma, unsigned long *flags)
{
	struct vm_operations_struct *h_vmop;
	unsigned long prot;
	int err;

	h_vmop = ERR_PTR(-ENODEV);
	if (!h_file->f_op || !h_file->f_op->mmap)
		goto out;

	prot = au_prot_conv(vma->vm_flags);
	err = security_file_mmap(h_file, /*reqprot*/prot, prot,
				 au_flag_conv(vma->vm_flags), vma->vm_start, 0);
	h_vmop = ERR_PTR(err);
	if (unlikely(err))
		goto out;

	err = h_file->f_op->mmap(h_file, vma);
	h_vmop = ERR_PTR(err);
	if (unlikely(err))
		goto out;

	/* oops, it became 'const' */
	h_vmop = (struct vm_operations_struct *)vma->vm_ops;
	*flags = vma->vm_flags;
	err = do_munmap(current->mm, vma->vm_start,
			vma->vm_end - vma->vm_start);
	if (unlikely(err)) {
		AuIOErr("failed internal unmapping %.*s, %d\n",
			AuDLNPair(h_file->f_dentry), err);
		h_vmop = ERR_PTR(-EIO);
	}

out:
	return h_vmop;
}

/*
 * This is another ugly approach to keep the lock order, particularly
 * mm->mmap_sem and aufs rwsem. The previous approach was reverted and you can
 * find it in git-log, if you want.
 *
 * native readdir: i_mutex, copy_to_user, mmap_sem
 * aufs readdir: i_mutex, rwsem, nested-i_mutex, copy_to_user, mmap_sem
 *
 * Before aufs_mmap() mmap_sem is acquired already, but aufs_mmap() has to
 * acquire aufs rwsem. It introduces a circular locking dependency.
 * To address this problem, aufs_mmap() delegates the part which requires aufs
 * rwsem to its internal workqueue.
 */

/* very ugly approach */
#include "mtx.h"

static void au_fi_mmap_lock_and_sell(struct file *file)
{
	struct mutex *mtx;

	FiMustWriteLock(file);

	mtx = &au_fi(file)->fi_mmap;
	mutex_lock(mtx);
	mutex_release(&mtx->dep_map, /*nested*/0, _RET_IP_);
}

static void au_fi_mmap_buy(struct file *file)
{
	struct mutex *mtx;

	mtx = &au_fi(file)->fi_mmap;
	MtxMustLock(mtx);

	mutex_set_owner(mtx);
	mutex_acquire(&mtx->dep_map, /*subclass*/0, /*trylock*/0, _RET_IP_);
}

static void au_fi_mmap_unlock(struct file *file)
{
	mutex_unlock(&au_fi(file)->fi_mmap);
}

struct au_mmap_pre_args {
	/* input */
	struct file *file;
	struct vm_area_struct *vma;

	/* output */
	int *errp;
	struct file *h_file;
	struct au_branch *br;
	int mmapped;
};

static int au_mmap_pre(struct file *file, struct vm_area_struct *vma,
		       struct file **h_file, struct au_branch **br,
		       int *mmapped)
{
	int err;
	aufs_bindex_t bstart;
	const unsigned char wlock
		= !!(file->f_mode & FMODE_WRITE) && (vma->vm_flags & VM_SHARED);
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_NOPLMW);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	*mmapped = !!au_test_mmapped(file);
	if (wlock) {
		struct au_pin pin;

		err = au_ready_to_write(file, -1, &pin);
		di_write_unlock(dentry);
		if (unlikely(err))
			goto out_unlock;
		au_unpin(&pin);
	} else
		di_write_unlock(dentry);
	bstart = au_fbstart(file);
	*br = au_sbr(sb, bstart);
	*h_file = au_hf_top(file);
	get_file(*h_file);
	if (!*mmapped)
		au_fi_mmap_lock_and_sell(file);

out_unlock:
	fi_write_unlock(file);
out:
	si_read_unlock(sb);
	return err;
}

static void au_call_mmap_pre(void *args)
{
	struct au_mmap_pre_args *a = args;
	*a->errp = au_mmap_pre(a->file, a->vma, &a->h_file, &a->br,
			       &a->mmapped);
}

static int aufs_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err, wkq_err;
	unsigned long h_vmflags;
	struct au_finfo *finfo;
	struct dentry *h_dentry;
	struct vm_operations_struct *h_vmop, *vmop;
	struct au_mmap_pre_args args = {
		.file		= file,
		.vma		= vma,
		.errp		= &err
	};

	wkq_err = au_wkq_wait_pre(au_call_mmap_pre, &args);
	if (unlikely(wkq_err))
		err = wkq_err;
	if (unlikely(err))
		goto out;
	if (!args.mmapped)
		au_fi_mmap_buy(file);

	h_dentry = args.h_file->f_dentry;
	if (!args.mmapped && au_test_fs_bad_mapping(h_dentry->d_sb)) {
		/*
		 * by this assignment, f_mapping will differs from aufs inode
		 * i_mapping.
		 * if someone else mixes the use of f_dentry->d_inode and
		 * f_mapping->host, then a problem may arise.
		 */
		file->f_mapping = args.h_file->f_mapping;
	}

	/* always try this internal mmap to get vma flags */
	h_vmflags = 0; /* gcc warning */
	h_vmop = au_hvmop(args.h_file, vma, &h_vmflags);
	err = PTR_ERR(h_vmop);
	if (IS_ERR(h_vmop))
		goto out_unlock;
	finfo = au_fi(file);
	AuDebugOn(args.mmapped && h_vmop != finfo->fi_hvmop);

	vmop = (void *)au_dy_vmop(file, args.br, h_vmop);
	err = PTR_ERR(vmop);
	if (IS_ERR(vmop))
		goto out_unlock;

	/*
	 * unnecessary to handle MAP_DENYWRITE and deny_write_access()?
	 * currently MAP_DENYWRITE from userspace is ignored, but elf loader
	 * sets it. when FMODE_EXEC is set (by open_exec() or sys_uselib()),
	 * both of the aufs file and the lower file is deny_write_access()-ed.
	 * finally I hope we can skip handlling MAP_DENYWRITE here.
	 */
	err = generic_file_mmap(file, vma);
	if (unlikely(err))
		goto out_unlock;

	vma->vm_ops = vmop;
	vma->vm_flags = h_vmflags;
	if (!args.mmapped)
		finfo->fi_hvmop = h_vmop;

	vfsub_file_accessed(args.h_file);
	/* update without lock, I don't think it a problem */
	fsstack_copy_attr_atime(file->f_dentry->d_inode, h_dentry->d_inode);

out_unlock:
	if (!args.mmapped)
		au_fi_mmap_unlock(file);
	fput(args.h_file);
out:
	return err;
}

/* ---------------------------------------------------------------------- */

static int aufs_fsync_nondir(struct file *file, int datasync)
{
	int err;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct file *h_file;
	struct super_block *sb;

	dentry = file->f_dentry;
	inode = dentry->d_inode;
	IMustLock(file->f_mapping->host);
	if (inode != file->f_mapping->host) {
		mutex_unlock(&file->f_mapping->host->i_mutex);
		mutex_lock(&inode->i_mutex);
	}
	IMustLock(inode);

	sb = dentry->d_sb;
	err = si_read_lock(sb, AuLock_FLUSH | AuLock_NOPLM);
	if (unlikely(err))
		goto out;

	err = 0; /* -EBADF; */ /* posix? */
	if (unlikely(!(file->f_mode & FMODE_WRITE)))
		goto out_si;
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out_si;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;
	au_unpin(&pin);

	err = -EINVAL;
	h_file = au_hf_top(file);
	if (h_file->f_op && h_file->f_op->fsync) {
		struct mutex *h_mtx;

		/*
		 * no filemap_fdatawrite() since aufs file has no its own
		 * mapping, but dir.
		 */
		h_mtx = &h_file->f_dentry->d_inode->i_mutex;
		mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
		err = h_file->f_op->fsync(h_file, datasync);
		if (!err)
			vfsub_update_h_iattr(&h_file->f_path, /*did*/NULL);
		/*ignore*/
		au_cpup_attr_timesizes(inode);
		mutex_unlock(h_mtx);
	}

out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
out_si:
	si_read_unlock(sb);
out:
	if (inode != file->f_mapping->host) {
		mutex_unlock(&inode->i_mutex);
		mutex_lock(&file->f_mapping->host->i_mutex);
	}
	return err;
}

/* no one supports this operation, currently */
#if 0
static int aufs_aio_fsync_nondir(struct kiocb *kio, int datasync)
{
	int err;
	struct au_pin pin;
	struct dentry *dentry;
	struct inode *inode;
	struct file *file, *h_file;

	file = kio->ki_filp;
	dentry = file->f_dentry;
	inode = dentry->d_inode;
	au_mtx_and_read_lock(inode);

	err = 0; /* -EBADF; */ /* posix? */
	if (unlikely(!(file->f_mode & FMODE_WRITE)))
		goto out;
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1);
	if (unlikely(err))
		goto out;

	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;
	au_unpin(&pin);

	err = -ENOSYS;
	h_file = au_hf_top(file);
	if (h_file->f_op && h_file->f_op->aio_fsync) {
		struct dentry *h_d;
		struct mutex *h_mtx;

		h_d = h_file->f_dentry;
		h_mtx = &h_d->d_inode->i_mutex;
		if (!is_sync_kiocb(kio)) {
			get_file(h_file);
			fput(file);
		}
		kio->ki_filp = h_file;
		err = h_file->f_op->aio_fsync(kio, datasync);
		mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
		if (!err)
			vfsub_update_h_iattr(&h_file->f_path, /*did*/NULL);
		/*ignore*/
		au_cpup_attr_timesizes(inode);
		mutex_unlock(h_mtx);
	}

out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
out:
	si_read_unlock(inode->sb);
	mutex_unlock(&inode->i_mutex);
	return err;
}
#endif

static int aufs_fasync(int fd, struct file *file, int flag)
{
	int err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH | AuLock_NOPLMW);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0);
	if (unlikely(err))
		goto out;

	h_file = au_hf_top(file);
	if (h_file->f_op && h_file->f_op->fasync)
		err = h_file->f_op->fasync(fd, h_file, flag);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

out:
	si_read_unlock(sb);
	return err;
}

/* ---------------------------------------------------------------------- */

/* no one supports this operation, currently */
#if 0
static ssize_t aufs_sendpage(struct file *file, struct page *page, int offset,
			     size_t len, loff_t *pos , int more)
{
}
#endif

/* ---------------------------------------------------------------------- */

const struct file_operations aufs_file_fop = {
	.owner		= THIS_MODULE,
	/*
	 * while generic_file_llseek/_unlocked() don't use BKL,
	 * don't use it since it operates file->f_mapping->host.
	 * in aufs, it may be a real file and may confuse users by UDBA.
	 */
	/* .llseek		= generic_file_llseek, */

	.read		= aufs_read,
	.write		= aufs_write,
	.aio_read	= aufs_aio_read,
	.aio_write	= aufs_aio_write,
#ifdef CONFIG_AUFS_POLL
	.poll		= aufs_poll,
#endif
	.unlocked_ioctl	= aufs_ioctl_nondir,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= aufs_ioctl_nondir, /* same */
#endif
	.mmap		= aufs_mmap,
	.open		= aufs_open_nondir,
	.flush		= aufs_flush_nondir,
	.release	= aufs_release_nondir,
	.fsync		= aufs_fsync_nondir,
	/* .aio_fsync	= aufs_aio_fsync_nondir, */
	.fasync		= aufs_fasync,
	/* .sendpage	= aufs_sendpage, */
	.splice_write	= aufs_splice_write,
	.splice_read	= aufs_splice_read,
#if 0
	.aio_splice_write = aufs_aio_splice_write,
	.aio_splice_read  = aufs_aio_splice_read
#endif
};
