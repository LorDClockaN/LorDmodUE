/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include "vidc_type.h"
#include "vcd_ddl_utils.h"

#if DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

#define DBG_TIME(x...) printk(KERN_DEBUG x)
#define ERR(x...) printk(KERN_ERR x)

struct time_data {
	unsigned int ddl_t1;
	unsigned int ddl_ttotal;
	unsigned int ddl_count;
};

static struct time_data proc_time[MAX_TIME_DATA];

#ifdef NO_IN_KERNEL_PMEM

void ddl_pmem_alloc(struct ddl_buf_addr *buff_addr, size_t sz, u32 align)
{
	u32 guard_bytes, align_mask;
	u32 physical_addr, align_offset;
	dma_addr_t phy_addr;

	if (align == DDL_LINEAR_BUFFER_ALIGN_BYTES) {

		guard_bytes = 31;
		align_mask = 0xFFFFFFE0U;

	} else {

		guard_bytes = DDL_TILE_BUF_ALIGN_GUARD_BYTES;
		align_mask = DDL_TILE_BUF_ALIGN_MASK;
	}

	buff_addr->virtual_base_addr =
		kmalloc((sz + guard_bytes), GFP_KERNEL);

	if (!buff_addr->virtual_base_addr) {
		ERR("\n ERROR %s:%u kamlloc fails to allocate"
			" sz + guard_bytes = %u\n", __func__, __LINE__,
			(sz + guard_bytes));
		return;
	}

	phy_addr = dma_map_single(NULL, buff_addr->virtual_base_addr,
				  sz + guard_bytes, DMA_TO_DEVICE);

	buff_addr->buffer_size = sz;
	physical_addr = (u32) phy_addr;
	buff_addr->align_physical_addr =
	    (u32 *) ((physical_addr + guard_bytes) & align_mask);
	align_offset =
	    (u32) (buff_addr->align_physical_addr) - physical_addr;
	buff_addr->align_virtual_addr =
	    (u32 *) ((u32) (buff_addr->virtual_base_addr)
		     + align_offset);
}

void ddl_pmem_free(struct ddl_buf_addr *buff_addr)
{
	kfree(buff_addr->virtual_base_addr);
	buff_addr->buffer_size = 0;
	buff_addr->virtual_base_addr = NULL;
}

#else

void ddl_pmem_alloc(struct ddl_buf_addr *buff_addr, size_t sz, u32 align)
{
	u32 guard_bytes, align_mask;
	s32 physical_addr;
	u32 align_offset;

	DBG_PMEM("\n%s() IN: Requested alloc size(%u)", __func__, (u32)sz);

	if (align == DDL_LINEAR_BUFFER_ALIGN_BYTES) {

		guard_bytes = 31;
		align_mask = 0xFFFFFFE0U;

	} else {

		guard_bytes = DDL_TILE_BUF_ALIGN_GUARD_BYTES;
		align_mask = DDL_TILE_BUF_ALIGN_MASK;
	}

	physical_addr = pmem_kalloc((sz + guard_bytes),
				      PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);
	buff_addr->physical_base_addr = (u32 *)physical_addr;

	if (IS_ERR((void *)physical_addr)) {
		pr_err("%s(): could not allocte in kernel pmem buffers\n",
		       __func__);
		goto bailout;
	}

	buff_addr->virtual_base_addr =
	    (u32 *) ioremap((unsigned long)physical_addr,
			    sz + guard_bytes);
	if (!buff_addr->virtual_base_addr) {

		pr_err("%s: could not ioremap in kernel pmem buffers\n",
		       __func__);
		pmem_kfree(physical_addr);
		goto bailout;
	}
	memset(buff_addr->virtual_base_addr, 0 , sz + guard_bytes);
	buff_addr->buffer_size = sz;

	buff_addr->align_physical_addr =
	    (u32 *) ((physical_addr + guard_bytes) & align_mask);

	align_offset =
	    (u32) (buff_addr->align_physical_addr) - physical_addr;

	buff_addr->align_virtual_addr =
	    (u32 *) ((u32) (buff_addr->virtual_base_addr)
		     + align_offset);

	DBG_PMEM("\n%s() OUT: phy_addr(%p) ker_addr(%p) size(%u)", __func__,
		buff_addr->physical_base_addr, buff_addr->virtual_base_addr,
		buff_addr->buffer_size);

	return;
bailout:
	buff_addr->physical_base_addr = NULL;
	buff_addr->virtual_base_addr = NULL;
	buff_addr->buffer_size = 0;
}

void ddl_pmem_free(struct ddl_buf_addr *buff_addr)
{
	if (!buff_addr) {
		ERR("\n %s() invalid arguments %p", __func__, buff_addr);
		return;
	}
	DBG_PMEM("\n%s() IN: phy_addr(%p) ker_addr(%p) size(%u)", __func__,
		buff_addr->physical_base_addr, buff_addr->virtual_base_addr,
		buff_addr->buffer_size);

	if (buff_addr->virtual_base_addr)
		iounmap((void *)buff_addr->virtual_base_addr);

	if ((buff_addr->physical_base_addr) &&
		pmem_kfree((s32) buff_addr->physical_base_addr)) {
		ERR("\n %s(): Error in Freeing ddl_pmem_free "
		"Physical Address %p", __func__,
		buff_addr->physical_base_addr);
	}
	DBG_PMEM("\n%s() OUT: phy_addr(%p) ker_addr(%p) size(%u)", __func__,
		buff_addr->physical_base_addr, buff_addr->virtual_base_addr,
		buff_addr->buffer_size);
	buff_addr->buffer_size = 0;
	buff_addr->physical_base_addr = NULL;
	buff_addr->virtual_base_addr = NULL;
}
#endif

void ddl_set_core_start_time(const char *func_name, u32 index)
{
	u32 act_time;
	struct timeval ddl_tv;
	struct time_data *time_data = &proc_time[index];
	do_gettimeofday(&ddl_tv);
	act_time = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	if (!time_data->ddl_t1) {
		time_data->ddl_t1 = act_time;
		DBG("\n%s(): Start Time (%u)", func_name, act_time);
	} else {
		DBG_TIME("\n%s(): Timer already started! St(%u) Act(%u)",
			func_name, time_data->ddl_t1, act_time);
	}
}

void ddl_calc_core_proc_time(const char *func_name, u32 index)
{
	struct time_data *time_data = &proc_time[index];
	if (time_data->ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		time_data->ddl_ttotal += (ddl_t2 - time_data->ddl_t1);
		time_data->ddl_count++;
		DBG_TIME("\n%s(): cnt(%u) Diff(%u) Avg(%u)",
			func_name, time_data->ddl_count,
			ddl_t2 - time_data->ddl_t1,
			time_data->ddl_ttotal/time_data->ddl_count);
		time_data->ddl_t1 = 0;
	}
}

void ddl_reset_core_time_variables(u32 index)
{
	proc_time[index].ddl_t1 = 0;
	proc_time[index].ddl_ttotal = 0;
	proc_time[index].ddl_count = 0;
}
