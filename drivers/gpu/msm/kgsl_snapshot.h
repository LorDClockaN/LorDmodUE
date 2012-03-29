/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
 */

#ifndef _KGSL_SNAPSHOT_H_
#define _KGSL_SNAPSHOT_H_

#include <linux/types.h>

/* Snapshot header */

#define SNAPSHOT_MAGIC 0x504D0001

/* GPU ID scheme:
 * [16:31] - core identifer (0x0002 for 2D or 0x0003 for 3D)
 * [00:16] - GPU specific identifier
 */

struct kgsl_snapshot_header {
	__u32 magic; /* Magic identifier */
	__u32 gpuid; /* GPU ID - see above */
} __packed;

/* Section header */
#define SNAPSHOT_SECTION_MAGIC 0xABCD

struct kgsl_snapshot_section_header {
	__u16 magic; /* Magic identifier */
	__u16 id;    /* Type of section */
	__u32 size;  /* Size of the section including this header */
} __packed;

/* Section identifiers */
#define KGSL_SNAPSHOT_SECTION_OS           0x0101
#define KGSL_SNAPSHOT_SECTION_REGS         0x0201
#define KGSL_SNAPSHOT_SECTION_RB           0x0301
#define KGSL_SNAPSHOT_SECTION_IB           0x0401
#define KGSL_SNAPSHOT_SECTION_INDEXED_REGS 0x0501
#define KGSL_SNAPSHOT_SECTION_ISTORE       0x0801
#define KGSL_SNAPSHOT_SECTION_DEBUG        0x0901
#define KGSL_SNAPSHOT_SECTION_END          0xFFFF

/* OS sub-section header */
#define KGSL_SNAPSHOT_OS_LINUX             0x0001

/* Linux OS specific information */

#define SNAPSHOT_STATE_HUNG 0
#define SNAPSHOT_STATE_RUNNING 1

struct kgsl_snapshot_linux {
	int osid;                   /* subsection OS identifier */
	int state;		    /* 1 if the thread is running, 0 for hung */
	__u32 seconds;		    /* Unix timestamp for the snapshot */
	__u32 power_flags;            /* Current power flags */
	__u32 power_level;            /* Current power level */
	__u32 power_interval_timeout; /* Power interval timeout */
	__u32 grpclk;                 /* Current GP clock value */
	__u32 busclk;		    /* Current busclk value */
	__u32 ptbase;		    /* Current ptbase */
	__u32 pid;		    /* PID of the process that owns the PT */
	__u32 current_context;	    /* ID of the current context */
	__u32 ctxtcount;	    /* Number of contexts appended to section */
	unsigned char release[32];  /* kernel release */
	unsigned char version[32];  /* kernel version */
	unsigned char comm[16];	    /* Name of the process that owns the PT */
} __packed;

/*
 * This structure contains a record of an active context.
 * These are appended one after another in the OS section below
 * the header above
 */

struct kgsl_snapshot_linux_context {
	__u32 id;			/* The context ID */
	__u32 timestamp_queued;		/* The last queued timestamp */
	__u32 timestamp_retired;	/* The last timestamp retired by HW */
};

/* Ringbuffer sub-section header */
struct kgsl_snapshot_rb {
	int start;  /* dword at the start of the dump */
	int end;    /* dword at the end of the dump */
	int rbsize; /* Size (in dwords) of the ringbuffer */
	int wptr;   /* Current index of the CPU write pointer */
	int rptr;   /* Current index of the GPU read pointer */
	int count;  /* Number of dwords in the dump */
} __packed;

/* Indirect buffer sub-section header */
struct kgsl_snapshot_ib {
	__u32 gpuaddr; /* GPU address of the the IB */
	__u32 ptbase;  /* Base for the pagetable the GPU address is valid in */
	int size;    /* Size of the IB */
} __packed;

/* Register sub-section header */
struct kgsl_snapshot_regs {
	__u32 count; /* Number of register pairs in the section */
} __packed;

/* Indexed register sub-section header */
struct kgsl_snapshot_indexed_regs {
	__u32 index_reg; /* Offset of the index register for this section */
	__u32 data_reg;  /* Offset of the data register for this section */
	int start;     /* Starting index */
	int count;     /* Number of dwords in the data */
} __packed;

/* Istore sub-section header */
struct kgsl_snapshot_istore {
	int count;   /* Number of instructions in the istore */
} __packed;

/* Debug data sub-section header */

#define SNAPSHOT_DEBUG_SX         1
#define SNAPSHOT_DEBUG_CP         2
#define SNAPSHOT_DEBUG_SQ         3
#define SNAPSHOT_DEBUG_SQTHREAD   4
#define SNAPSHOT_DEBUG_MIU        5

struct kgsl_snapshot_debug {
	int type;    /* Type identifier for the attached tata */
	int size;   /* Size of the section in bytes */
} __packed;

#ifdef __KERNEL__

/* Allocate 512K for each device snapshot */
#define KGSL_SNAPSHOT_MEMSIZE (512 * 1024)

struct kgsl_device;
/*
 * A helper macro to print out "not enough memory functions" - this
 * makes it easy to standardize the messages as well as cut down on
 * the number of strings in the binary
 */

#define SNAPSHOT_ERR_NOMEM(_d, _s) \
	KGSL_DRV_ERR((_d), \
	"snapshot: not enough snapshot memory for section %s\n", (_s))

/*
 * kgsl_snapshot_add_section - Add a new section to the GPU snapshot
 * @device - the KGSL device being snapshotted
 * @id - the section id
 * @snapshot - pointer to the memory for the snapshot
 * @remain - pointer to the number of bytes left in the snapshot region
 * @func - Function pointer to fill the section
 * @priv - Priv pointer to pass to the function
 *
 * Set up a KGSL snapshot header by filling the memory with the callback
 * function and adding the standard section header
 */

static inline void *kgsl_snapshot_add_section(struct kgsl_device *device,
	u16 id, void *snapshot, int *remain,
	int (*func)(struct kgsl_device *, void *, int, void *), void *priv)
{
	struct kgsl_snapshot_section_header *header = snapshot;
	void *data = snapshot + sizeof(*header);
	int ret = 0;

	/*
	 * Sanity check to make sure there is enough for the header.  The
	 * callback will check to make sure there is enough for the rest
	 * of the data.  If there isn't enough room then don't advance the
	 * pointer.
	 */

	if (*remain < sizeof(*header))
		return snapshot;

	/* It is legal to have no function (i.e. - make an empty section) */

	if (func) {
		ret = func(device, data, *remain, priv);

		/*
		 * If there wasn't enough room for the data then don't bother
		 * setting up the header.
		 */

		if (ret == 0)
			return snapshot;
	}

	header->magic = SNAPSHOT_SECTION_MAGIC;
	header->id = id;
	header->size = ret + sizeof(*header);

	/* Decrement the room left in the snapshot region */
	*remain -= header->size;
	/* Advance the pointer to the end of the next function */
	return snapshot + header->size;
}

/* A common helper function to dump a range of registers.  This will be used in
 * the GPU specific devices like this:
 *
 * struct kgsl_snapshot_registers priv;
 * priv.regs = registers_array;;
 * priv.count = num_registers;
 *
 * kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS, snapshot,
 *	remain, kgsl_snapshot_dump_regs, &priv).
 *
 * Pass in an array of register range pairs in the form of:
 * start reg, stop reg
 * All the registers between start and stop inclusive will be dumped
 */

struct kgsl_snapshot_registers {
	unsigned int *regs;  /* Pointer to the array of register ranges */
	int count;	     /* Number of entries in the array */
};

int kgsl_snapshot_dump_regs(struct kgsl_device *device, void *snapshot,
	int remain, void *priv);

/*
 * A common helper function to dump a set of indexed registers. Use it
 * like this:
 *
 * struct kgsl_snapshot_indexed_registers priv;
 * priv.index = REG_INDEX;
 * priv.data = REG_DATA;
 * priv.count = num_registers
 *
 * kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_INDEXED_REGS,
 *	snapshot, remain, kgsl_snapshot_dump_indexed_regs, &priv).
 *
 * The callback function will write an index from 0 to priv.count to
 * the index register and read the data from the data register.
 */

struct kgsl_snapshot_indexed_registers {
	unsigned int index; /* Offset of the index register */
	unsigned int data;  /* Offset of the data register */
	unsigned int start;	/* Index to start with */
	unsigned int count; /* Number of values to read from the pair */
};

int kgsl_snapshot_dump_indexed_regs(struct kgsl_device *device,
	void *snapshot, int remain, void *priv);

#endif
#endif
