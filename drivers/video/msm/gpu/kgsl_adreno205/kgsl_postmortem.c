/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include <linux/delay.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_cmdstream.h"
#include "kgsl_log.h"
#include "kgsl_postmortem.h"
#include "kgsl_pm4types.h"
#include "yamato_reg.h"
#include "kgsl_yamato.h"

#define INVALID_RB_CMD 0xaaaaaaaa

struct pm_id_name {
	uint32_t id;
	char name[9];
};

static const struct pm_id_name pm0_types[] = {
	{REG_PA_SC_AA_CONFIG,		"RPASCAAC"},
	{REG_RBBM_PM_OVERRIDE2,		"RRBBPMO2"},
	{REG_SCRATCH_REG2,		"RSCRTRG2"},
	{REG_SQ_GPR_MANAGEMENT,		"RSQGPRMN"},
	{REG_SQ_INST_STORE_MANAGMENT,	"RSQINSTS"},
	{REG_TC_CNTL_STATUS,		"RTCCNTLS"},
	{REG_TP0_CHICKEN,		"RTP0CHCK"},
	{REG_CP_TIMESTAMP,		"CP_TM_ST"},
};

static const struct pm_id_name pm3_types[] = {
	{PM4_COND_EXEC,			"CND_EXEC"},
	{PM4_CONTEXT_UPDATE,		"CX__UPDT"},
	{PM4_DRAW_INDX,			"DRW_NDX_"},
	{PM4_DRAW_INDX_BIN,		"DRW_NDXB"},
	{PM4_EVENT_WRITE,		"EVENT_WT"},
	{PM4_IM_LOAD,			"IN__LOAD"},
	{PM4_IM_LOAD_IMMEDIATE,		"IM_LOADI"},
	{PM4_IM_STORE,			"IM_STORE"},
	{PM4_INDIRECT_BUFFER,		"IND_BUF_"},
	{PM4_INDIRECT_BUFFER_PFD,	"IND_BUFP"},
	{PM4_INTERRUPT,			"PM4_INTR"},
	{PM4_INVALIDATE_STATE,		"INV_STAT"},
	{PM4_LOAD_CONSTANT_CONTEXT,	"LD_CN_CX"},
	{PM4_ME_INIT,			"ME__INIT"},
	{PM4_NOP,			"PM4__NOP"},
	{PM4_REG_RMW,			"REG__RMW"},
	{PM4_REG_TO_MEM,		"REG2_MEM"},
	{PM4_SET_BIN_BASE_OFFSET,	"ST_BIN_O"},
	{PM4_SET_CONSTANT,		"ST_CONST"},
	{PM4_SET_PROTECTED_MODE,	"ST_PRT_M"},
	{PM4_SET_SHADER_BASES,		"ST_SHD_B"},
	{PM4_WAIT_FOR_IDLE,		"WAIT4IDL"},
};

static uint32_t kgsl_is_pm4_len(uint32_t word)
{
	if (word == INVALID_RB_CMD)
		return 0;

	return (word >> 16) & 0x3FFF;
}

static bool kgsl_is_pm4_type(uint32_t word)
{
	int i;

	if (word == INVALID_RB_CMD)
		return 1;

	if (kgsl_is_pm4_len(word) > 16)
		return 0;

	if ((word & (3<<30)) == PM4_TYPE0_PKT) {
		for (i = 0; i < ARRAY_SIZE(pm0_types); ++i) {
			if ((word & 0x7FFF) == pm0_types[i].id)
				return 1;
		}
		return 0;
	}
	if ((word & (3<<30)) == PM4_TYPE3_PKT) {
		for (i = 0; i < ARRAY_SIZE(pm3_types); ++i) {
			if ((word & 0xFFFF) == (pm3_types[i].id << 8))
				return 1;
		}
		return 0;
	}
	return 0;
}

static const char *kgsl_pm4_name(uint32_t word)
{
	int i;

	if (word == INVALID_RB_CMD)
		return "--------";

	if ((word & (3<<30)) == PM4_TYPE0_PKT) {
		for (i = 0; i < ARRAY_SIZE(pm0_types); ++i) {
			if ((word & 0x7FFF) == pm0_types[i].id)
				return pm0_types[i].name;
		}
		return "????????";
	}
	if ((word & (3<<30)) == PM4_TYPE3_PKT) {
		for (i = 0; i < ARRAY_SIZE(pm3_types); ++i) {
			if ((word & 0xFFFF) == (pm3_types[i].id << 8))
				return pm3_types[i].name;
		}
		return "????????";
	}
	return "????????";
}

static void kgsl_yamato_dump_regs(struct kgsl_device *device)
{
#ifndef CONFIG_MSM_KGSL_PSTMRTMDMP_NO_REG_DUMP
	/* Byte offsets for registers. Each pair is in format of (start, end).
	 */
	static const int reg_ranges[] = {
		0x0000, 0x0008, 0x0010, 0x002c, 0x00ec, 0x00f4,
		0x0100, 0x0110, 0x0118, 0x011c, 0x0300, 0x0304,
		0x0700, 0x0704, 0x070c, 0x0720, 0x0754, 0x0764,
		0x0770, 0x0774, 0x07a8, 0x07a8, 0x07b8, 0x07cc,
		0x07d8, 0x07dc, 0x07f0, 0x07fc, 0x0e44, 0x0e48,
		0x0e6c, 0x0e78, 0x0ec8, 0x0ed4, 0x0edc, 0x0edc,
		0x0fe0, 0x0fec, 0x1100, 0x1100,

		0x110c, 0x1110, 0x112c, 0x112c, 0x1134, 0x113c,
		0x1148, 0x1148, 0x1150, 0x116c, 0x11fc, 0x11fc,
		0x15e0, 0x161c, 0x1724, 0x1724, 0x1740, 0x1740,
		0x1804, 0x1810, 0x1818, 0x1824, 0x182c, 0x1838,
		0x184c, 0x1850, 0x28a4, 0x28ac, 0x28bc, 0x28c4,
		0x2900, 0x290c, 0x2914, 0x2914, 0x2938, 0x293c,
		0x30b0, 0x30b0, 0x30c0, 0x30c0, 0x30e0, 0x30f0,
		0x3100, 0x3100, 0x3110, 0x3110, 0x3200, 0x3218,
		0x3220, 0x3250, 0x3264, 0x3268, 0x3290, 0x3294,
		0x3400, 0x340c, 0x3418, 0x3418, 0x3420, 0x342c,
		0x34d0, 0x34d4, 0x36b8, 0x3704, 0x3720, 0x3750,
		0x3760, 0x3764, 0x3800, 0x3800, 0x3808, 0x3810,
		0x385c, 0x3878, 0x3b00, 0x3b24, 0x3b2c, 0x3b30,
		0x3b40, 0x3b40, 0x3b50, 0x3b5c, 0x3b80, 0x3b88,
		0x3c04, 0x3c08, 0x3c30, 0x3c30, 0x3c38, 0x3c48,
		0x3c98, 0x3ca8, 0x3cb0, 0x3cb0,

		0x8000, 0x8008, 0x8018, 0x803c, 0x8200, 0x8208,
		0x8400, 0x8424, 0x8430, 0x8450, 0x8600, 0x8610,
		0x87d4, 0x87dc, 0x8800, 0x8820, 0x8a00, 0x8a0c,
		0x8a4c, 0x8a50, 0x8c00, 0x8c20, 0x8c48, 0x8c48,
		0x8c58, 0x8c74, 0x8c90, 0x8c98, 0x8e00, 0x8e0c,

		0x9000, 0x9008, 0x9018, 0x903c, 0x9200, 0x9208,
		0x9400, 0x9424, 0x9430, 0x9450, 0x9600, 0x9610,
		0x97d4, 0x97dc, 0x9800, 0x9820, 0x9a00, 0x9a0c,
		0x9a4c, 0x9a50, 0x9c00, 0x9c20, 0x9c48, 0x9c48,
		0x9c58, 0x9c74, 0x9c90, 0x9c98, 0x9e00, 0x9e0c,

		0x10000, 0x1000c, 0x12000, 0x12014,
		0x12400, 0x12400, 0x12420, 0x12420
	};

	int range = 0, offset = 0;

	for (range = 0; range < (ARRAY_SIZE(reg_ranges) / 2); range++) {
		/* start and end are in dword offsets */
		int start = reg_ranges[range * 2] / 4;
		int end = reg_ranges[range * 2 + 1] / 4;

		unsigned char linebuf[32 * 3 + 2 + 32 + 1];
		int linelen, i;

		for (offset = start; offset <= end; offset += linelen) {
			unsigned int regvals[32/4];
			linelen = min(end+1-offset, 32/4);

			for (i = 0; i < linelen; ++i)
				kgsl_regread(device, offset+i, regvals+i);

			hex_dump_to_buffer(regvals, linelen*4, 32, 4,
				linebuf, sizeof(linebuf), 0);
			KGSL_LOG_DUMP("REG: %5.5X: %s\n", offset<<2, linebuf);
		}
	}
#endif
}

static void dump_ib(struct kgsl_device *device, char* buffId, uint32_t pt_base,
	uint32_t base_offset, uint32_t ib_base, uint32_t ib_size, bool dump)
{
	unsigned int memsize;
	uint8_t *base_addr = kgsl_sharedmem_convertaddr(device, pt_base,
		ib_base, &memsize);

	if (base_addr && dump)
		print_hex_dump(KERN_ERR, buffId, DUMP_PREFIX_OFFSET,
				 32, 4, base_addr, ib_size*4, 0);
	else
		KGSL_LOG_DUMP("%s base:%8.8X  ib_size:%d  "
			"offset:%5.5X%s\n",
			buffId, ib_base, ib_size*4, base_offset,
			base_addr ? "" : " [Invalid]");
}

#define IB_LIST_SIZE	64
struct ib_list {
	int count;
	uint32_t bases[IB_LIST_SIZE];
	uint32_t sizes[IB_LIST_SIZE];
	uint32_t offsets[IB_LIST_SIZE];
};

static void dump_ib1(struct kgsl_device *device, uint32_t pt_base,
			uint32_t base_offset,
			uint32_t ib1_base, uint32_t ib1_size,
			struct ib_list *ib_list, bool dump)
{
	int i, j;
	uint32_t value;
	uint32_t *ib1_addr;
	unsigned int memsize;

	dump_ib(device, "IB1:", pt_base, base_offset, ib1_base,
		ib1_size, dump);

	/* fetch virtual address for given IB base */
	ib1_addr = (uint32_t *)kgsl_sharedmem_convertaddr(device, pt_base,
		ib1_base, &memsize);
	if (!ib1_addr)
		return;

	for (i = 0; i+3 < ib1_size; ) {
		value = ib1_addr[i++];
		if (value == pm4_type3_packet(PM4_INDIRECT_BUFFER_PFD, 2)) {
			uint32_t ib2_base = ib1_addr[i++];
			uint32_t ib2_size = ib1_addr[i++];

			/* find previous match */
			for (j = 0; j < ib_list->count; ++j)
				if (ib_list->sizes[j] == ib2_size
					&& ib_list->bases[j] == ib2_base)
					break;

			if (j < ib_list->count || ib_list->count
				>= IB_LIST_SIZE)
				continue;

			/* store match */
			ib_list->sizes[ib_list->count] = ib2_size;
			ib_list->bases[ib_list->count] = ib2_base;
			ib_list->offsets[ib_list->count] = i<<2;
			++ib_list->count;
		}
	}
}

static void kgsl_dump_rb_buffer(const void *buf, size_t len,
		char *linebuf, size_t linebuflen, int *argp)
{
	const u32 *ptr4 = buf;
	const int ngroups = len;
	int lx = 0, j;
	bool nxsp = 1;

	for (j = 0; j < ngroups; j++) {
		if (*argp < 0) {
			lx += scnprintf(linebuf + lx, linebuflen - lx, " <");
			*argp = -*argp;
		} else if (nxsp)
			lx += scnprintf(linebuf + lx, linebuflen - lx, "  ");
		else
			nxsp = 1;
		if (!*argp && kgsl_is_pm4_type(ptr4[j])) {
			lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%s", kgsl_pm4_name(ptr4[j]));
			*argp = -(kgsl_is_pm4_len(ptr4[j])+1);
		} else {
			lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%8.8X", ptr4[j]);
			if (*argp > 1)
				--*argp;
			else if (*argp == 1) {
				*argp = 0;
				nxsp = 0;
				lx += scnprintf(linebuf + lx, linebuflen - lx,
					"> ");
			}
		}
	}
	linebuf[lx] = '\0';
}

static bool kgsl_rb_use_hex(void)
{
#ifdef CONFIG_MSM_KGSL_PSTMRTMDMP_RB_HEX
	return 1;
#else
	return 0;
#endif
}

static void kgsl_dump_rb(const void *buf, size_t len, int start, int size)
{
	const uint32_t *ptr = buf;
	int i, remaining, args = 0;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];
	const int rowsize = 8;

	len >>= 2;
	remaining = len;
	for (i = 0; i < len; i += rowsize) {
		int linelen = min(remaining, rowsize);
		remaining -= rowsize;

		if (kgsl_rb_use_hex())
			hex_dump_to_buffer(ptr+i, linelen*4, rowsize*4, 4,
				linebuf, sizeof(linebuf), 0);
		else
			kgsl_dump_rb_buffer(ptr+i, linelen, linebuf,
				sizeof(linebuf), &args);
		KGSL_LOG_DUMP("RB: %4.4X:%s\n", (start+i)%size, linebuf);
	}
}

static bool kgsl_ib_dump_enabled(void)
{
#ifdef CONFIG_MSM_KGSL_PSTMRTMDMP_NO_IB_DUMP
	return 0;
#else
	return 1;
#endif
}

struct log_field {
	bool show;
	const char *display;
};

static void kgsl_dump_fields(const char *start, const struct log_field *lines,
	int num)
{
	int count, i;
	char lb[90];

	strncpy(lb, start, sizeof(lb));
	count = 0;
	for (i = 0; i < num; ++i, ++lines) {
		if (lines->show) {
			if (count++)
				strncat(lb, " | ", sizeof(lb));
			strncat(lb, lines->display, sizeof(lb));
			if (count == 6) {
				KGSL_LOG_DUMP("%s\n", lb);
				strncpy(lb, "        ", sizeof(lb));
				count = 0;
			}
		}
	}
	if (count)
		KGSL_LOG_DUMP("%s\n", lb);
}

static int kgsl_dump_yamato(struct kgsl_device *device)
{
	unsigned int r1, r2, r3, rbbm_status;
	unsigned int cp_ib1_base, cp_ib1_bufsz, cp_stat;
	unsigned int cp_ib2_base, cp_ib2_bufsz;
	unsigned int pt_base;
	unsigned int cp_rb_base, rb_count;
	unsigned int cp_rb_wptr, cp_rb_rptr;
	unsigned int i;
	int result = 0;
	uint32_t *rb_copy;
	const uint32_t *rb_vaddr;
	int num_item = 0;
	int read_idx, write_idx;
	unsigned int ts_processed, rb_memsize;

	static struct ib_list ib_list;

	mb();

	kgsl_regread(device, REG_RBBM_STATUS, &rbbm_status);
	kgsl_regread(device, REG_RBBM_PM_OVERRIDE1, &r2);
	kgsl_regread(device, REG_RBBM_PM_OVERRIDE2, &r3);
	KGSL_LOG_DUMP("RBBM:   STATUS   = %08X | PM_OVERRIDE1 = %08X | "
		"PM_OVERRIDE2 = %08X\n", rbbm_status, r2, r3);

	kgsl_regread(device, REG_RBBM_INT_CNTL, &r1);
	kgsl_regread(device, REG_RBBM_INT_STATUS, &r2);
	kgsl_regread(device, REG_RBBM_READ_ERROR, &r3);
	KGSL_LOG_DUMP("        INT_CNTL = %08X | INT_STATUS   = %08X | "
		"READ_ERROR   = %08X\n", r1, r2, r3);

	{
		char cmdFifo[16];
		struct log_field lines[] = {
			{rbbm_status &  0x000F, cmdFifo},
			{rbbm_status &  BIT(5), "TC busy     "},
			{rbbm_status &  BIT(8), "HIRQ pending"},
			{rbbm_status &  BIT(9), "CPRQ pending"},
			{rbbm_status & BIT(10), "CFRQ pending"},
			{rbbm_status & BIT(11), "PFRQ pending"},
			{rbbm_status & BIT(12), "VGT 0DMA bsy"},
			{rbbm_status & BIT(14), "RBBM WU busy"},
			{rbbm_status & BIT(16), "CP NRT busy "},
			{rbbm_status & BIT(18), "MH busy     "},
			{rbbm_status & BIT(19), "MH chncy bsy"},
			{rbbm_status & BIT(21), "SX busy     "},
			{rbbm_status & BIT(22), "TPC busy    "},
			{rbbm_status & BIT(24), "SC CNTX busy"},
			{rbbm_status & BIT(25), "PA busy     "},
			{rbbm_status & BIT(26), "VGT busy    "},
			{rbbm_status & BIT(27), "SQ cntx1 bsy"},
			{rbbm_status & BIT(28), "SQ cntx0 bsy"},
			{rbbm_status & BIT(30), "RB busy     "},
			{rbbm_status & BIT(31), "Grphs pp bsy"},
		};
		snprintf(cmdFifo, sizeof(cmdFifo), "CMD FIFO=%01X  ",
			rbbm_status & 0xf);
		kgsl_dump_fields(" STATUS=", lines, ARRAY_SIZE(lines));
	}

	kgsl_regread(device, REG_CP_RB_BASE, &cp_rb_base);
	kgsl_regread(device, REG_CP_RB_CNTL, &r2);
	rb_count = 2 << (r2 & (BIT(6)-1));
	kgsl_regread(device, REG_CP_RB_RPTR_ADDR, &r3);
	KGSL_LOG_DUMP("CP_RB:  BASE = %08X | CNTL   = %08X | RPTR_ADDR = %08X"
		"\n", cp_rb_base, r2, r3);

	kgsl_regread(device, REG_CP_RB_RPTR, &cp_rb_rptr);
	kgsl_regread(device, REG_CP_RB_WPTR, &cp_rb_wptr);
	kgsl_regread(device, REG_CP_RB_RPTR_WR, &r3);
	KGSL_LOG_DUMP("        RPTR = %08X | WPTR   = %08X | RPTR_WR   = %08X"
		"\n", cp_rb_rptr, cp_rb_wptr, r3);

	kgsl_regread(device, REG_CP_IB1_BASE, &cp_ib1_base);
	kgsl_regread(device, REG_CP_IB1_BUFSZ, &cp_ib1_bufsz);
	KGSL_LOG_DUMP("CP_IB1: BASE = %08X | BUFSZ  = %d\n", cp_ib1_base,
		cp_ib1_bufsz);

	kgsl_regread(device, REG_CP_IB2_BASE, &cp_ib2_base);
	kgsl_regread(device, REG_CP_IB2_BUFSZ, &cp_ib2_bufsz);
	KGSL_LOG_DUMP("CP_IB2: BASE = %08X | BUFSZ  = %d\n", cp_ib2_base,
		cp_ib2_bufsz);

	kgsl_regread(device, REG_CP_INT_CNTL, &r1);
	kgsl_regread(device, REG_CP_INT_STATUS, &r2);
	KGSL_LOG_DUMP("CP_INT: CNTL = %08X | STATUS = %08X\n", r1, r2);

	kgsl_regread(device, REG_CP_ME_CNTL, &r1);
	kgsl_regread(device, REG_CP_ME_STATUS, &r2);
	kgsl_regread(device, REG_MASTER_INT_SIGNAL, &r3);
	KGSL_LOG_DUMP("CP_ME:  CNTL = %08X | STATUS = %08X | MSTR_INT_SGNL = "
		"%08X\n", r1, r2, r3);

	kgsl_regread(device, REG_CP_STAT, &cp_stat);
	KGSL_LOG_DUMP("CP_STAT      = %08X\n", cp_stat);
#ifndef CONFIG_MSM_KGSL_PSTMRTMDMP_CP_STAT_NO_DETAIL
	{
		struct log_field lns[] = {
			{cp_stat &  BIT(0), "WR_BSY     0"},
			{cp_stat &  BIT(1), "RD_RQ_BSY  1"},
			{cp_stat &  BIT(2), "RD_RTN_BSY 2"},
		};
		kgsl_dump_fields("    MIU=", lns, ARRAY_SIZE(lns));
	}
	{
		struct log_field lns[] = {
			{cp_stat &  BIT(5), "RING_BUSY  5"},
			{cp_stat &  BIT(6), "NDRCTS_BSY 6"},
			{cp_stat &  BIT(7), "NDRCT2_BSY 7"},
			{cp_stat &  BIT(9), "ST_BUSY    9"},
			{cp_stat & BIT(10), "BUSY      10"},
		};
		kgsl_dump_fields("    CSF=", lns, ARRAY_SIZE(lns));
	}
	{
		struct log_field lns[] = {
			{cp_stat & BIT(11), "RNG_Q_BSY 11"},
			{cp_stat & BIT(12), "NDRCTS_Q_B12"},
			{cp_stat & BIT(13), "NDRCT2_Q_B13"},
			{cp_stat & BIT(16), "ST_QUEUE_B16"},
			{cp_stat & BIT(17), "PFP_BUSY  17"},
		};
		kgsl_dump_fields("   RING=", lns, ARRAY_SIZE(lns));
	}
	{
		struct log_field lns[] = {
			{cp_stat &  BIT(3), "RBIU_BUSY  3"},
			{cp_stat &  BIT(4), "RCIU_BUSY  4"},
			{cp_stat & BIT(18), "MQ_RG_BSY 18"},
			{cp_stat & BIT(19), "MQ_NDRS_BS19"},
			{cp_stat & BIT(20), "MQ_NDR2_BS20"},
			{cp_stat & BIT(21), "MIU_WC_STL21"},
			{cp_stat & BIT(22), "CP_NRT_BSY22"},
			{cp_stat & BIT(23), "3D_BUSY   23"},
			{cp_stat & BIT(26), "ME_BUSY   26"},
			{cp_stat & BIT(29), "ME_WC_BSY 29"},
			{cp_stat & BIT(30), "MIU_FF EM 30"},
			{cp_stat & BIT(31), "CP_BUSY   31"},
		};
		kgsl_dump_fields(" CP_STT=", lns, ARRAY_SIZE(lns));
	}
#endif

	kgsl_regread(device, REG_SCRATCH_REG0, &r1);
	KGSL_LOG_DUMP("SCRATCH_REG0       = %08X\n", r1);

	kgsl_regread(device, REG_COHER_SIZE_PM4, &r1);
	kgsl_regread(device, REG_COHER_BASE_PM4, &r2);
	kgsl_regread(device, REG_COHER_STATUS_PM4, &r3);
	KGSL_LOG_DUMP("COHER:  SIZE_PM4   = %08X | BASE_PM4 = %08X | STATUS_PM4"
		" = %08X\n", r1, r2, r3);

	kgsl_regread(device, REG_MH_AXI_ERROR, &r1);
	KGSL_LOG_DUMP("MH:     AXI_ERROR  = %08X\n", r1);

	kgsl_regread(device, REG_MH_MMU_PAGE_FAULT, &r1);
	kgsl_regread(device, REG_MH_MMU_CONFIG, &r2);
	kgsl_regread(device, REG_MH_MMU_MPU_BASE, &r3);
	KGSL_LOG_DUMP("MH_MMU: PAGE_FAULT = %08X | CONFIG   = %08X | MPU_BASE ="
		" %08X\n", r1, r2, r3);

	kgsl_regread(device, REG_MH_MMU_MPU_END, &r1);
	kgsl_regread(device, REG_MH_MMU_VA_RANGE, &r2);
	kgsl_regread(device, REG_MH_MMU_PT_BASE, &pt_base);
	KGSL_LOG_DUMP("        MPU_END    = %08X | VA_RANGE = %08X | PT_BASE  ="
		" %08X\n", r1, r2, pt_base);

	kgsl_regread(device, REG_MH_MMU_TRAN_ERROR, &r1);
	KGSL_LOG_DUMP("        TRAN_ERROR = %08X\n", r1);

	kgsl_regread(device, REG_MH_INTERRUPT_MASK, &r1);
	kgsl_regread(device, REG_MH_INTERRUPT_STATUS, &r2);
	KGSL_LOG_DUMP("MH_INTERRUPT: MASK = %08X | STATUS   = %08X\n", r1, r2);

	if (device->ftbl.device_cmdstream_readtimestamp != NULL) {
		ts_processed = device->ftbl.device_cmdstream_readtimestamp(
				device, KGSL_TIMESTAMP_RETIRED);
		KGSL_LOG_DUMP("TIMESTM RTRD: %08X\n", ts_processed);
	}

	num_item = kgsl_ringbuffer_count(&device->ringbuffer, cp_rb_rptr);
	if (num_item <= 0) {
		KGSL_LOG_POSTMORTEM_WRITE("Ringbuffer is Empty.\n");
		goto end;
	}

	rb_copy = kmalloc(rb_count<<2, GFP_KERNEL);
	if (!rb_copy) {
		KGSL_LOG_POSTMORTEM_WRITE("Failed to allocate buffer.\n");
		result = -ENOMEM;
		goto end;
	}

	KGSL_LOG_DUMP("RB: rd_addr:%8.8x  rb_size:%d  num_item:%d\n",
		cp_rb_base, rb_count<<2, num_item);
	rb_vaddr = (const uint32_t *)kgsl_sharedmem_convertaddr(device, pt_base,
					cp_rb_base, &rb_memsize);
	if (!rb_vaddr) {
		KGSL_LOG_POSTMORTEM_WRITE("Can't fetch vaddr for CP_RB_BASE\n");
		goto error_kfree;
	}

	read_idx = (int)cp_rb_rptr - 64;
	if (read_idx < 0)
		read_idx += rb_count;
	write_idx = (int)cp_rb_wptr + 16;
	if (write_idx > rb_count)
		write_idx -= rb_count;
	num_item += 64+16;
	if (num_item > rb_count)
		num_item = rb_count;
	if (write_idx >= read_idx)
		memcpy(rb_copy, rb_vaddr+read_idx, num_item<<2);
	else {
		int part1_c = rb_count-read_idx;
		memcpy(rb_copy, rb_vaddr+read_idx, part1_c<<2);
		memcpy(rb_copy+part1_c, rb_vaddr, (num_item-part1_c)<<2);
	}

	/* extract the latest ib commands from the buffer */
	ib_list.count = 0;
	i = 0;
	for (read_idx = 0; read_idx < num_item; ) {
		uint32_t this_cmd = rb_copy[read_idx++];
		if (this_cmd == pm4_type3_packet(PM4_INDIRECT_BUFFER_PFD, 2)) {
			uint32_t ib_addr = rb_copy[read_idx++];
			uint32_t ib_size = rb_copy[read_idx++];
			dump_ib1(device, pt_base, (read_idx-3)<<2, ib_addr,
				ib_size, &ib_list, 0);
			for (; i < ib_list.count; ++i)
				dump_ib(device, "IB2:", pt_base,
					ib_list.offsets[i],
					ib_list.bases[i],
					ib_list.sizes[i], 0);
		}
	}

	read_idx = (int)cp_rb_rptr - 64;
	if (read_idx < 0)
		read_idx += rb_count;
	KGSL_LOG_DUMP("RB: addr=%8.8x  window:%4.4x-%4.4x, start:%4.4x\n",
		cp_rb_base, cp_rb_rptr, cp_rb_wptr, read_idx);
	kgsl_dump_rb(rb_copy, num_item<<2, read_idx, rb_count);

	if (kgsl_ib_dump_enabled()) {
		for (read_idx = 64; read_idx >= 0; --read_idx) {
			uint32_t this_cmd = rb_copy[read_idx];
			if (this_cmd == pm4_type3_packet(
				PM4_INDIRECT_BUFFER_PFD, 2)) {
				uint32_t ib_addr = rb_copy[read_idx+1];
				uint32_t ib_size = rb_copy[read_idx+2];
				if (cp_ib1_bufsz && cp_ib1_base == ib_addr) {
					KGSL_LOG_DUMP("IB1: base:%8.8X  "
						"count:%d\n", ib_addr, ib_size);
					dump_ib(device, "IB1: ", pt_base,
						read_idx<<2, ib_addr, ib_size,
						1);
				}
			}
		}
		for (i = 0; i < ib_list.count; ++i) {
			if (cp_ib2_bufsz && cp_ib2_base == ib_list.bases[i]) {
				uint32_t ib_size = ib_list.sizes[i];
				uint32_t ib_offset = ib_list.offsets[i];
				KGSL_LOG_DUMP("IB2: base:%8.8X  count:%d\n",
					cp_ib2_base, ib_size);
				dump_ib(device, "IB2: ", pt_base, ib_offset,
					ib_list.bases[i], ib_size, 1);
			}
		}
	}

	kgsl_yamato_dump_regs(device);
error_kfree:
	kfree(rb_copy);
end:
	return result;
}

int kgsl_postmortem_dump(struct kgsl_device *device)
{
	static int dump_done;

	if (dump_done)
		return 0;
	dump_done = 1;

	BUG_ON(device == NULL);

	if (device->id == KGSL_DEVICE_YAMATO)
		kgsl_dump_yamato(device);
	else
		KGSL_DRV_FATAL("Unknown device id - 0x%x\n", device->id);

	KGSL_DRV_FATAL("Dump Finished\n");

	return 0;
}