/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * QUP driver for Qualcomm MSM platforms
 *
 */

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <mach/board.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("0.2");
MODULE_ALIAS("platform:i2c_qup");

/* QUP Registers */
enum {
	QUP_CONFIG              = 0x0,
	QUP_STATE               = 0x4,
	QUP_IO_MODE             = 0x8,
	QUP_SW_RESET            = 0xC,
	QUP_OPERATIONAL         = 0x18,
	QUP_ERROR_FLAGS         = 0x1C,
	QUP_ERROR_FLAGS_EN      = 0x20,
	QUP_MX_READ_CNT         = 0x208,
	QUP_MX_INPUT_CNT        = 0x200,
	QUP_MX_WR_CNT           = 0x100,
	QUP_OUT_DEBUG           = 0x108,
	QUP_OUT_FIFO_CNT        = 0x10C,
	QUP_OUT_FIFO_BASE       = 0x110,
	QUP_IN_READ_CUR         = 0x20C,
	QUP_IN_DEBUG            = 0x210,
	QUP_IN_FIFO_CNT         = 0x214,
	QUP_IN_FIFO_BASE        = 0x218,
	QUP_I2C_CLK_CTL         = 0x400,
	QUP_I2C_STATUS          = 0x404,
};

/* QUP States and reset values */
enum {
	QUP_RESET_STATE         = 0,
	QUP_RUN_STATE           = 1U,
	QUP_STATE_MASK          = 3U,
	QUP_PAUSE_STATE         = 3U,
	QUP_STATE_VALID         = 1U << 2,
	QUP_I2C_MAST_GEN        = 1U << 4,
	QUP_OPERATIONAL_RESET   = 0xFF0,
	QUP_I2C_STATUS_RESET    = 0xFFFFFC,
};

/* QUP OPERATIONAL FLAGS */
enum {
	QUP_OUT_SVC_FLAG        = 1U << 8,
	QUP_IN_SVC_FLAG         = 1U << 9,
	QUP_MX_INPUT_DONE       = 1U << 11,
};

/* I2C mini core related values */
enum {
	I2C_MINI_CORE           = 2U << 8,
	I2C_N_VAL               = 0xF,

};

/* Packing Unpacking words in FIFOs , and IO modes*/
enum {
	QUP_WR_BLK_MODE  = 1U << 10,
	QUP_RD_BLK_MODE  = 1U << 12,
	QUP_UNPACK_EN = 1U << 14,
	QUP_PACK_EN = 1U << 15,
};

/* QUP tags */
enum {
	QUP_OUT_NOP   = 0,
	QUP_OUT_START = 1U << 8,
	QUP_OUT_DATA  = 2U << 8,
	QUP_OUT_STOP  = 3U << 8,
	QUP_OUT_REC   = 4U << 8,
	QUP_IN_DATA   = 5U << 8,
	QUP_IN_STOP   = 6U << 8,
	QUP_IN_NACK   = 7U << 8,
};

/* Status, Error flags */
enum {
	I2C_STATUS_WR_BUFFER_FULL  = 1U << 0,
	I2C_STATUS_BUS_ACTIVE      = 1U << 8,
	I2C_STATUS_ERROR_MASK      = 0x38000FC,
	QUP_IN_NOT_EMPTY           = 1U << 5,
	QUP_STATUS_ERROR_FLAGS     = 0x7C,
};

struct qup_i2c_dev {
	struct device                *dev;
	void __iomem                 *base;		/* virtual */
	void __iomem                 *gsbi;		/* virtual */
	int                          in_irq;
	int                          out_irq;
	int                          err_irq;
	int                          num_irqs;
	struct clk                   *clk;
	struct clk                   *pclk;
	struct i2c_adapter           adapter;

	struct i2c_msg               *msg;
	int                          pos;
	int                          cnt;
	int                          err;
	int                          mode;
	int                          clk_ctl;
	int                          one_bit_t;
	int                          out_fifo_sz;
	int                          in_fifo_sz;
	int                          out_blk_sz;
	int                          in_blk_sz;
	int                          wr_sz;
	struct msm_i2c_platform_data *pdata;
	int                          suspended;
	int                          clk_state;
	struct timer_list            pwr_timer;
	struct mutex                 mlock;
	void                         *complete;
};

#ifdef DEBUG
static void
qup_print_status(struct qup_i2c_dev *dev)
{
	uint32_t val;
	val = readl(dev->base+QUP_CONFIG);
	dev_dbg(dev->dev, "Qup config is :0x%x\n", val);
	val = readl(dev->base+QUP_STATE);
	dev_dbg(dev->dev, "Qup state is :0x%x\n", val);
	val = readl(dev->base+QUP_IO_MODE);
	dev_dbg(dev->dev, "Qup mode is :0x%x\n", val);
}
#else
static inline void qup_print_status(struct qup_i2c_dev *dev)
{
}
#endif

static irqreturn_t
qup_i2c_interrupt(int irq, void *devid)
{
	struct qup_i2c_dev *dev = devid;
	uint32_t status = readl(dev->base + QUP_I2C_STATUS);
	uint32_t status1 = readl(dev->base + QUP_ERROR_FLAGS);
	uint32_t op_flgs = readl(dev->base + QUP_OPERATIONAL);
	int err = 0;

	if (!dev->msg)
		return IRQ_HANDLED;

	if (status & I2C_STATUS_ERROR_MASK) {
		dev_err(dev->dev, "QUP: Got i2c error :0x%x, irq:%d\n",
			status, irq);
		err = -status;
		/* Clear Error interrupt if it's a level triggered interrupt*/
		if (dev->num_irqs == 1)
			writel((status & I2C_STATUS_ERROR_MASK),
				dev->base + QUP_I2C_STATUS);
		goto intr_done;
	}

	if (status1 & 0x7F) {
		dev_err(dev->dev, "QUP: Got QUP error :0x%x\n", status1);
		err = -status1;
		/* Clear Error interrupt if it's a level triggered interrupt*/
		if (dev->num_irqs == 1)
			writel((status1 & QUP_STATUS_ERROR_FLAGS),
				dev->base + QUP_ERROR_FLAGS);
		goto intr_done;
	}

	if ((dev->num_irqs == 3) && (dev->msg->flags == I2C_M_RD)
		&& (irq == dev->out_irq))
		return IRQ_HANDLED;
	if (op_flgs & QUP_OUT_SVC_FLAG)
		writel(QUP_OUT_SVC_FLAG, dev->base + QUP_OPERATIONAL);
	if (dev->msg->flags == I2C_M_RD) {
		if ((op_flgs & QUP_MX_INPUT_DONE) ||
			(op_flgs & QUP_IN_SVC_FLAG))
			writel(QUP_IN_SVC_FLAG, dev->base + QUP_OPERATIONAL);
		else
			return IRQ_HANDLED;
	}

intr_done:
	dev_dbg(dev->dev, "QUP intr= %d, i2c status=0x%x, qup status = 0x%x\n",
			irq, status, status1);
	qup_print_status(dev);
	dev->err = err;
	complete(dev->complete);
	return IRQ_HANDLED;
}

static void
qup_i2c_pwr_mgmt(struct qup_i2c_dev *dev, unsigned int state)
{
	dev->clk_state = state;
	if (state != 0) {
		clk_enable(dev->clk);
		if (dev->pclk)
			clk_enable(dev->pclk);
	} else {
		clk_disable(dev->clk);
		if (dev->pclk)
			clk_disable(dev->pclk);
	}
}

static void
qup_i2c_pwr_timer(unsigned long data)
{
	struct qup_i2c_dev *dev = (struct qup_i2c_dev *) data;
	dev_dbg(dev->dev, "QUP_Power: Inactivity based power management\n");
	if (dev->clk_state == 1)
		qup_i2c_pwr_mgmt(dev, 0);
}

static int
qup_i2c_poll_writeready(struct qup_i2c_dev *dev)
{
	uint32_t retries = 0;

	while (retries != 2000) {
		uint32_t status = readl(dev->base + QUP_I2C_STATUS);

		if (!(status & I2C_STATUS_WR_BUFFER_FULL)) {
			if (!(status & I2C_STATUS_BUS_ACTIVE))
				return 0;
			else /* 1-bit delay before we check for bus busy */
				udelay(dev->one_bit_t);
		}
		if (retries++ == 1000)
			udelay(100);
	}
	qup_print_status(dev);
	return -ETIMEDOUT;
}

static int
qup_i2c_poll_state(struct qup_i2c_dev *dev, uint32_t state)
{
	uint32_t retries = 0;

	dev_dbg(dev->dev, "Polling Status for state:0x%x\n", state);

	while (retries != 2000) {
		uint32_t status = readl(dev->base + QUP_STATE);

		if ((status & (QUP_STATE_VALID | state)) ==
				(QUP_STATE_VALID | state))
			return 0;
		else if (retries++ == 1000)
			udelay(100);
	}
	return -ETIMEDOUT;
}

#ifdef DEBUG
static void qup_verify_fifo(struct qup_i2c_dev *dev, uint32_t val,
				uint32_t addr, int rdwr)
{
	if (rdwr)
		dev_dbg(dev->dev, "RD:Wrote 0x%x to out_ff:0x%x\n", val, addr);
	else
		dev_dbg(dev->dev, "WR:Wrote 0x%x to out_ff:0x%x\n", val, addr);
}
#else
static inline void qup_verify_fifo(struct qup_i2c_dev *dev, uint32_t val,
				uint32_t addr, int rdwr)
{
}
#endif

static void
qup_issue_read(struct qup_i2c_dev *dev, struct i2c_msg *msg, int *idx,
		uint32_t carry_over)
{
	uint16_t addr = (msg->addr << 1) | 1;
	/* QUP limit 256 bytes per read. By HW design, 0 in the 8-bit field
	 * is treated as 256 byte read.
	 */
	uint16_t rd_len = ((dev->cnt == 256) ? 0 : dev->cnt);

	if (*idx % 4) {
		writel(carry_over | ((QUP_OUT_START | addr) << 16),
		dev->base + QUP_OUT_FIFO_BASE);/* + (*idx-2)); */

		qup_verify_fifo(dev, carry_over |
			((QUP_OUT_START | addr) << 16), (uint32_t)dev->base
			+ QUP_OUT_FIFO_BASE + (*idx - 2), 1);
		writel((QUP_OUT_REC | rd_len),
			dev->base + QUP_OUT_FIFO_BASE);/* + (*idx+2)); */

		qup_verify_fifo(dev, (QUP_OUT_REC | rd_len),
		(uint32_t)dev->base + QUP_OUT_FIFO_BASE + (*idx + 2), 1);
	} else {
		writel(((QUP_OUT_REC | rd_len) << 16) | QUP_OUT_START | addr,
			dev->base + QUP_OUT_FIFO_BASE);/* + (*idx)); */

		qup_verify_fifo(dev, QUP_OUT_REC << 16 | rd_len << 16 |
		QUP_OUT_START | addr,
		(uint32_t)dev->base + QUP_OUT_FIFO_BASE + (*idx), 1);
	}
	*idx += 4;
}

static void
qup_issue_write(struct qup_i2c_dev *dev, struct i2c_msg *msg, int rem,
			int *idx, uint32_t *carry_over)
{
	int entries = dev->cnt;
	int empty_sl = dev->wr_sz - ((*idx) >> 1);
	int i = 0;
	uint32_t val = 0;
	uint32_t last_entry = 0;
	uint16_t addr = msg->addr << 1;

	if (dev->pos == 0) {
		if (*idx % 4) {
			writel(*carry_over | ((QUP_OUT_START | addr) << 16),
					dev->base + QUP_OUT_FIFO_BASE);

			qup_verify_fifo(dev, *carry_over | QUP_OUT_DATA << 16 |
				addr << 16, (uint32_t)dev->base +
				QUP_OUT_FIFO_BASE + (*idx) - 2, 0);
		} else
			val = QUP_OUT_START | addr;
		*idx += 2;
		i++;
		entries++;
	} else {
		/* Avoid setp time issue by adding 1 NOP when number of bytes
		 * are more than FIFO/BLOCK size. setup time issue can't appear
		 * otherwise since next byte to be written will always be ready
		 */
		val = (QUP_OUT_NOP | 1);
		*idx += 2;
		i++;
		entries++;
	}
	if (entries > empty_sl)
		entries = empty_sl;

	for (; i < (entries - 1); i++) {
		if (*idx % 4) {
			writel(val | ((QUP_OUT_DATA |
				msg->buf[dev->pos]) << 16),
				dev->base + QUP_OUT_FIFO_BASE);

			qup_verify_fifo(dev, val | QUP_OUT_DATA << 16 |
				msg->buf[dev->pos] << 16, (uint32_t)dev->base +
				QUP_OUT_FIFO_BASE + (*idx) - 2, 0);
		} else
			val = QUP_OUT_DATA | msg->buf[dev->pos];
		(*idx) += 2;
		dev->pos++;
	}
	if (dev->pos < (msg->len - 1))
		last_entry = QUP_OUT_DATA;
	else if (rem > 1) /* not last array entry */
		last_entry = QUP_OUT_DATA;
	else
		last_entry = QUP_OUT_STOP;
	if ((*idx % 4) == 0) {
		/*
		 * If read-start and read-command end up in different fifos, it
		 * may result in extra-byte being read due to extra-read cycle.
		 * Avoid that by inserting NOP as the last entry of fifo only
		 * if write command(s) leave 1 space in fifo.
		 */
		if (rem > 1) {
			struct i2c_msg *next = msg + 1;
			if (next->addr == msg->addr && (next->flags | I2C_M_RD)
				&& *idx == ((dev->wr_sz*2) - 4)) {
				writel(((last_entry | msg->buf[dev->pos]) |
					((1 | QUP_OUT_NOP) << 16)), dev->base +
					QUP_OUT_FIFO_BASE);/* + (*idx) - 2); */
				*idx += 2;
			} else
				*carry_over = (last_entry | msg->buf[dev->pos]);
		} else {
			writel((last_entry | msg->buf[dev->pos]),
			dev->base + QUP_OUT_FIFO_BASE);/* + (*idx) - 2); */

			qup_verify_fifo(dev, last_entry | msg->buf[dev->pos],
			(uint32_t)dev->base + QUP_OUT_FIFO_BASE +
			(*idx), 0);
		}
	} else {
		writel(val | ((last_entry | msg->buf[dev->pos]) << 16),
		dev->base + QUP_OUT_FIFO_BASE);/* + (*idx) - 2); */

		qup_verify_fifo(dev, val | (last_entry << 16) |
		(msg->buf[dev->pos] << 16), (uint32_t)dev->base +
		QUP_OUT_FIFO_BASE + (*idx) - 2, 0);
	}

	*idx += 2;
	dev->pos++;
	dev->cnt = msg->len - dev->pos;
}

static int
qup_update_state(struct qup_i2c_dev *dev, uint32_t state)
{
	if (qup_i2c_poll_state(dev, 0) != 0)
		return -EIO;
	writel(state, dev->base + QUP_STATE);
	if (qup_i2c_poll_state(dev, state) != 0)
		return -EIO;
	return 0;
}

static int
qup_set_read_mode(struct qup_i2c_dev *dev, int rd_len)
{
	uint32_t wr_mode = (dev->wr_sz < dev->out_fifo_sz) ?
				QUP_WR_BLK_MODE : 0;
	if (rd_len > 256) {
		dev_err(dev->dev, "HW doesn't support READs > 256 bytes\n");
		return -EPROTONOSUPPORT;
	}
	if (rd_len <= dev->in_fifo_sz) {
		writel(wr_mode | QUP_PACK_EN | QUP_UNPACK_EN,
			dev->base + QUP_IO_MODE);
		writel(rd_len, dev->base + QUP_MX_READ_CNT);
	} else {
		writel(wr_mode | QUP_RD_BLK_MODE |
			QUP_PACK_EN | QUP_UNPACK_EN, dev->base + QUP_IO_MODE);
		writel(rd_len, dev->base + QUP_MX_INPUT_CNT);
	}
	return 0;
}

static int
qup_set_wr_mode(struct qup_i2c_dev *dev, int rem)
{
	int total_len = 0;
	int ret = 0;
	if (dev->msg->len >= (dev->out_fifo_sz - 1)) {
		total_len = dev->msg->len + 1 +
				(dev->msg->len/(dev->out_blk_sz-1));
		writel(QUP_WR_BLK_MODE | QUP_PACK_EN | QUP_UNPACK_EN,
			dev->base + QUP_IO_MODE);
		dev->wr_sz = dev->out_blk_sz;
	} else
		writel(QUP_PACK_EN | QUP_UNPACK_EN,
			dev->base + QUP_IO_MODE);

	if (rem > 1) {
		struct i2c_msg *next = dev->msg + 1;
		if (next->addr == dev->msg->addr &&
			next->flags == I2C_M_RD) {
			ret = qup_set_read_mode(dev, next->len);
			/* make sure read start & read command are in 1 blk */
			if ((total_len % dev->out_blk_sz) ==
				(dev->out_blk_sz - 1))
				total_len += 3;
			else
				total_len += 2;
		}
	}
	/* WRITE COUNT register valid/used only in block mode */
	if (dev->wr_sz == dev->out_blk_sz)
		writel(total_len, dev->base + QUP_MX_WR_CNT);
	return ret;
}

static int
qup_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	DECLARE_COMPLETION_ONSTACK(complete);
	struct qup_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	int rem = num;
	long timeout;
	int err;

	del_timer_sync(&dev->pwr_timer);
	mutex_lock(&dev->mlock);

	if (dev->suspended) {
		mutex_unlock(&dev->mlock);
		return -EIO;
	}

	if (dev->clk_state == 0)
		qup_i2c_pwr_mgmt(dev, 1);
	/* Initialize QUP registers during first transfer */
	if (dev->clk_ctl == 0) {
		int fs_div;
		int hs_div;
		int i2c_clk;
		uint32_t fifo_reg;
		writel(0x2 << 4, dev->gsbi);

		i2c_clk = 19200000; /* input clock */
		fs_div = ((i2c_clk / dev->pdata->clk_freq) / 2) - 3;
		hs_div = 3;
		dev->clk_ctl = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
		fifo_reg = readl(dev->base + QUP_IO_MODE);
		if (fifo_reg & 0x3)
			dev->out_blk_sz = (fifo_reg & 0x3) * 16;
		else
			dev->out_blk_sz = 16;
		if (fifo_reg & 0x60)
			dev->in_blk_sz = ((fifo_reg & 0x60) >> 5) * 16;
		else
			dev->in_blk_sz = 16;
		/*
		 * The block/fifo size w.r.t. 'actual data' is 1/2 due to 'tag'
		 * associated with each byte written/received
		 */
		dev->out_blk_sz /= 2;
		dev->in_blk_sz /= 2;
		dev->out_fifo_sz = dev->out_blk_sz *
					(2 << ((fifo_reg & 0x1C) >> 2));
		dev->in_fifo_sz = dev->in_blk_sz *
					(2 << ((fifo_reg & 0x380) >> 7));
		dev_dbg(dev->dev, "QUP IN:bl:%d, ff:%d, OUT:bl:%d, ff:%d\n",
				dev->in_blk_sz, dev->in_fifo_sz,
				dev->out_blk_sz, dev->out_fifo_sz);
	}

	if (dev->num_irqs == 3) {
		enable_irq(dev->in_irq);
		enable_irq(dev->out_irq);
	}
	enable_irq(dev->err_irq);
	writel(QUP_RESET_STATE, dev->base + QUP_STATE);
	ret = qup_i2c_poll_state(dev, QUP_RESET_STATE);
	if (ret) {
		dev_err(dev->dev, "QUP Busy:Trying to recover\n");
		goto out_err;
	}

	/* Initialize QUP registers */
	writel(1, dev->base + QUP_SW_RESET);
	writel(0, dev->base + QUP_CONFIG);
	writel(QUP_OPERATIONAL_RESET, dev->base + QUP_OPERATIONAL);
	writel(QUP_STATUS_ERROR_FLAGS, dev->base + QUP_ERROR_FLAGS_EN);

	writel(I2C_MINI_CORE | I2C_N_VAL, dev->base + QUP_CONFIG);

	/* Initialize I2C mini core registers */
	writel(0, dev->base + QUP_I2C_CLK_CTL);
	writel(QUP_I2C_STATUS_RESET, dev->base + QUP_I2C_STATUS);

	dev->cnt = msgs->len;
	dev->pos = 0;
	dev->msg = msgs;
	while (rem) {
		bool filled = false;

		dev->wr_sz = dev->out_fifo_sz;
		dev->err = 0;
		dev->complete = &complete;

		if (qup_i2c_poll_state(dev, QUP_I2C_MAST_GEN) != 0) {
			ret = -EIO;
			goto out_err;
		}

		qup_print_status(dev);
		/* HW limits Read upto 256 bytes in 1 read without stop */
		if (dev->msg->flags & I2C_M_RD) {
			ret = qup_set_read_mode(dev, dev->cnt);
			if (ret != 0)
				goto out_err;
		} else {
			ret = qup_set_wr_mode(dev, rem);
			if (ret != 0)
				goto out_err;
			/* Don't fill block till we get interrupt */
			if (dev->wr_sz == dev->out_blk_sz)
				filled = true;
		}

		err = qup_update_state(dev, QUP_RUN_STATE);
		if (err < 0) {
			ret = err;
			goto out_err;
		}

		qup_print_status(dev);
		writel(dev->clk_ctl, dev->base + QUP_I2C_CLK_CTL);

		do {
			int idx = 0;
			uint32_t carry_over = 0;

			/* Transition to PAUSE state only possible from RUN */
			err = qup_update_state(dev, QUP_PAUSE_STATE);
			if (err < 0) {
				ret = err;
				goto out_err;
			}

			qup_print_status(dev);
			/* This operation is Write, check the next operation
			 * and decide mode
			 */
			while (filled == false) {
				if ((msgs->flags & I2C_M_RD) &&
					(dev->cnt == msgs->len))
					qup_issue_read(dev, msgs, &idx,
							carry_over);
				else if (!(msgs->flags & I2C_M_RD))
					qup_issue_write(dev, msgs, rem, &idx,
							&carry_over);
				if (idx >= (dev->wr_sz << 1))
					filled = true;
				/* Start new message */
				if (filled == false) {
					if (msgs->flags & I2C_M_RD)
							filled = true;
					else if (rem > 1) {
						/* Only combine operations with
						 * same address
						 */
						struct i2c_msg *next = msgs + 1;
						if (next->addr != msgs->addr ||
							next->flags == 0)
							filled = true;
						else {
							rem--;
							msgs++;
							dev->msg = msgs;
							dev->pos = 0;
							dev->cnt = msgs->len;
						}
					} else
						filled = true;
				}
			}
			err = qup_update_state(dev, QUP_RUN_STATE);
			if (err < 0) {
				ret = err;
				goto out_err;
			}
			dev_dbg(dev->dev, "idx:%d, rem:%d, num:%d, mode:%d\n",
				idx, rem, num, dev->mode);

			qup_print_status(dev);
			timeout = wait_for_completion_timeout(&complete,
					msecs_to_jiffies(dev->out_fifo_sz));
			if (!timeout) {
				dev_err(dev->dev, "Transaction timed out\n");
				writel(1, dev->base + QUP_SW_RESET);
				msleep(10);
				ret = -ETIMEDOUT;
				goto out_err;
			}
			if (dev->err) {
				dev_err(dev->dev,
					"Error during data xfer (%d)\n",
					dev->err);
				ret = dev->err;
				goto out_err;
			}
			if (dev->msg->flags & I2C_M_RD) {
				int i;
				uint32_t dval = 0;
				for (i = 0; dev->pos < dev->msg->len; i++,
						dev->pos++) {
					uint32_t rd_status = readl(dev->base +
							QUP_OPERATIONAL);
					if (i % 2 == 0) {
						if ((rd_status &
							QUP_IN_NOT_EMPTY) == 0)
							break;
						dval = readl(dev->base +
							QUP_IN_FIFO_BASE);
						dev->msg->buf[dev->pos] =
							dval & 0xFF;
					} else
						dev->msg->buf[dev->pos] =
							((dval & 0xFF0000) >>
							 16);
				}
				dev->cnt -= i;
			} else
				filled = false; /* refill output FIFO */
		} while (dev->cnt > 0);
		if (dev->cnt == 0) {
			rem--;
			msgs++;
			if (rem) {
				dev->pos = 0;
				dev->cnt = msgs->len;
				dev->msg = msgs;
			}
		}
		/* Wait for I2C bus to be idle */
		ret = qup_i2c_poll_writeready(dev);
		if (ret) {
			dev_err(dev->dev,
				"Error waiting for write ready\n");
			goto out_err;
		}
	}

	ret = num;
 out_err:
	dev->complete = NULL;
	dev->msg = NULL;
	dev->pos = 0;
	dev->err = 0;
	dev->cnt = 0;
	disable_irq(dev->err_irq);
	if (dev->num_irqs == 3) {
		disable_irq(dev->in_irq);
		disable_irq(dev->out_irq);
	}
	dev->pwr_timer.expires = jiffies + 3*HZ;
	add_timer(&dev->pwr_timer);
	mutex_unlock(&dev->mlock);
	return ret;
}

static u32
qup_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm qup_i2c_algo = {
	.master_xfer	= qup_i2c_xfer,
	.functionality	= qup_i2c_func,
};

static int __devinit
qup_i2c_probe(struct platform_device *pdev)
{
	struct qup_i2c_dev	*dev;
	struct resource         *qup_mem, *gsbi_mem, *qup_io, *gsbi_io;
	struct resource		*in_irq, *out_irq, *err_irq;
	struct clk         *clk, *pclk;
	int ret = 0;
	struct msm_i2c_platform_data *pdata;
	const char *qup_apps_clk_name = "qup_clk";

	dev_dbg(&pdev->dev, "qup_i2c_probe\n");

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform data not initialized\n");
		return -ENOSYS;
	}
	qup_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"qup_phys_addr");
	if (!qup_mem) {
		dev_err(&pdev->dev, "no qup mem resource?\n");
		return -ENODEV;
	}
	gsbi_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"gsbi_qup_i2c_addr");
	if (!gsbi_mem) {
		dev_err(&pdev->dev, "no gsbi mem resource?\n");
		return -ENODEV;
	}

	/*
	 * We only have 1 interrupt for new hardware targets and in_irq,
	 * out_irq will be NULL for those platforms
	 */
	in_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						"qup_in_intr");

	out_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						"qup_out_intr");

	err_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						"qup_err_intr");
	if (!err_irq) {
		dev_err(&pdev->dev, "no error irq resource?\n");
		return -ENODEV;
	}

	qup_io = request_mem_region(qup_mem->start, resource_size(qup_mem),
					pdev->name);
	if (!qup_io) {
		dev_err(&pdev->dev, "QUP region already claimed\n");
		return -EBUSY;
	}
	gsbi_io = request_mem_region(gsbi_mem->start, resource_size(gsbi_mem),
					pdev->name);
	if (!gsbi_io) {
		dev_err(&pdev->dev, "GSBI region already claimed\n");
		return -EBUSY;
	}

	if (pdata->clk != NULL)
		qup_apps_clk_name = pdata->clk;

	clk = clk_get(&pdev->dev, qup_apps_clk_name);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "Could not get clock\n");
		ret = PTR_ERR(clk);
		goto err_clk_get_failed;
	}

	if (pdata->pclk != NULL) {
		pclk = clk_get(&pdev->dev, pdata->pclk);
		if (IS_ERR(pclk)) {
			dev_err(&pdev->dev, "Could not get pclock\n");
			ret = PTR_ERR(pclk);
			clk_put(clk);
			goto err_clk_get_failed;
		}
	} else
		pclk = NULL;

	if (!(pdata->msm_i2c_config_gpio)) {
		dev_err(&pdev->dev, "config_gpio function not initialized\n");
		ret = -ENOSYS;
		goto err_config_failed;
	}

	/* We support frequencies upto FAST Mode(400KHz) */
	if (pdata->clk_freq <= 0 ||
			pdata->clk_freq > 400000) {
		dev_err(&pdev->dev, "clock frequency not supported\n");
		ret = -EIO;
		goto err_config_failed;
	}

	dev = kzalloc(sizeof(struct qup_i2c_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_alloc_dev_failed;
	}

	dev->dev = &pdev->dev;
	if (in_irq)
		dev->in_irq = in_irq->start;
	if (out_irq)
		dev->out_irq = out_irq->start;
	dev->err_irq = err_irq->start;
	if (in_irq && out_irq)
		dev->num_irqs = 3;
	else
		dev->num_irqs = 1;
	dev->clk = clk;
	dev->pclk = pclk;
	dev->base = ioremap(qup_mem->start, resource_size(qup_mem));
	if (!dev->base) {
		ret = -ENOMEM;
		goto err_ioremap_failed;
	}

	/* Configure GSBI block to use I2C functionality */
	dev->gsbi = ioremap(gsbi_mem->start, resource_size(gsbi_mem));
	if (!dev->gsbi) {
		ret = -ENOMEM;
		goto err_gsbi_failed;
	}

	platform_set_drvdata(pdev, dev);

	dev->one_bit_t = USEC_PER_SEC/pdata->clk_freq;
	dev->pdata = pdata;
	dev->clk_ctl = 0;

	i2c_set_adapdata(&dev->adapter, dev);
	dev->adapter.algo = &qup_i2c_algo;
	strlcpy(dev->adapter.name,
		"QUP I2C adapter",
		sizeof(dev->adapter.name));

	dev->adapter.nr = pdev->id;
	ret = i2c_add_numbered_adapter(&dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "i2c_add_adapter failed\n");
		goto err_i2c_add_adapter_failed;
	}

	/*
	 * We use num_irqs to also indicate if we got 3 interrupts or just 1.
	 * If we have just 1, we use err_irq as the general purpose irq
	 * and handle the changes in ISR accordingly
	 * Per Hardware guidelines, if we have 3 interrupts, they are always
	 * edge triggering, and if we have 1, it's always level-triggering
	 */
	if (dev->num_irqs == 3) {
		ret = request_irq(dev->in_irq, qup_i2c_interrupt,
				IRQF_TRIGGER_RISING, "qup_in_intr", dev);
		if (ret) {
			dev_err(&pdev->dev, "request_in_irq failed\n");
			goto err_request_irq_failed;
		}
		/*
		 * We assume out_irq exists if in_irq does since platform
		 * configuration either has 3 interrupts assigned to QUP or 1
		 */
		ret = request_irq(dev->out_irq, qup_i2c_interrupt,
				IRQF_TRIGGER_RISING, "qup_out_intr", dev);
		if (ret) {
			dev_err(&pdev->dev, "request_out_irq failed\n");
			free_irq(dev->in_irq, dev);
			goto err_request_irq_failed;
		}
		ret = request_irq(dev->err_irq, qup_i2c_interrupt,
				IRQF_TRIGGER_RISING, "qup_err_intr", dev);
		if (ret) {
			dev_err(&pdev->dev, "request_err_irq failed\n");
			free_irq(dev->out_irq, dev);
			free_irq(dev->in_irq, dev);
			goto err_request_irq_failed;
		}
	} else {
		ret = request_irq(dev->err_irq, qup_i2c_interrupt,
				IRQF_TRIGGER_HIGH, "qup_err_intr", dev);
		if (ret) {
			dev_err(&pdev->dev, "request_err_irq failed\n");
			goto err_request_irq_failed;
		}
	}
	disable_irq(dev->err_irq);
	if (dev->num_irqs == 3) {
		disable_irq(dev->in_irq);
		disable_irq(dev->out_irq);
	}
	pdata->msm_i2c_config_gpio(dev->adapter.nr, 1);

	dev->suspended = 0;
	mutex_init(&dev->mlock);
	dev->clk_state = 0;
	setup_timer(&dev->pwr_timer, qup_i2c_pwr_timer, (unsigned long) dev);
	return 0;

err_request_irq_failed:
	i2c_del_adapter(&dev->adapter);
err_i2c_add_adapter_failed:
	iounmap(dev->gsbi);
err_gsbi_failed:
	iounmap(dev->base);
err_ioremap_failed:
	kfree(dev);
err_alloc_dev_failed:
err_config_failed:
	clk_put(clk);
	if (pclk)
		clk_put(pclk);
err_clk_get_failed:
	release_mem_region(gsbi_mem->start, resource_size(gsbi_mem));
	release_mem_region(qup_mem->start, resource_size(qup_mem));
	return ret;
}

static int __devexit
qup_i2c_remove(struct platform_device *pdev)
{
	struct qup_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*qup_mem, *gsbi_mem;

	/* Grab mutex to ensure ongoing transaction is over */
	mutex_lock(&dev->mlock);
	dev->suspended = 1;
	mutex_unlock(&dev->mlock);
	mutex_destroy(&dev->mlock);
	del_timer_sync(&dev->pwr_timer);
	if (dev->clk_state != 0)
		qup_i2c_pwr_mgmt(dev, 0);
	platform_set_drvdata(pdev, NULL);
	if (dev->num_irqs == 3) {
		free_irq(dev->out_irq, dev);
		free_irq(dev->in_irq, dev);
	}
	free_irq(dev->err_irq, dev);
	i2c_del_adapter(&dev->adapter);
	clk_put(dev->clk);
	if (dev->pclk)
		clk_put(dev->pclk);
	iounmap(dev->gsbi);
	iounmap(dev->base);
	kfree(dev);
	gsbi_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"gsbi_qup_i2c_addr");
	release_mem_region(gsbi_mem->start, resource_size(gsbi_mem));
	qup_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"qup_phys_addr");
	release_mem_region(qup_mem->start, resource_size(qup_mem));
	return 0;
}

#ifdef CONFIG_PM
static int qup_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct qup_i2c_dev *dev = platform_get_drvdata(pdev);

	/* Grab mutex to ensure ongoing transaction is over */
	mutex_lock(&dev->mlock);
	dev->suspended = 1;
	mutex_unlock(&dev->mlock);
	del_timer_sync(&dev->pwr_timer);
	if (dev->clk_state != 0)
		qup_i2c_pwr_mgmt(dev, 0);
	return 0;
}

static int qup_i2c_resume(struct platform_device *pdev)
{
	struct qup_i2c_dev *dev = platform_get_drvdata(pdev);
	dev->suspended = 0;
	return 0;
}
#else
#define qup_i2c_suspend NULL
#define qup_i2c_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver qup_i2c_driver = {
	.probe		= qup_i2c_probe,
	.remove		= __devexit_p(qup_i2c_remove),
	.suspend	= qup_i2c_suspend,
	.resume		= qup_i2c_resume,
	.driver		= {
		.name	= "qup_i2c",
		.owner	= THIS_MODULE,
	},
};

/* QUP may be needed to bring up other drivers */
static int __init
qup_i2c_init_driver(void)
{
	return platform_driver_register(&qup_i2c_driver);
}
subsys_initcall(qup_i2c_init_driver);

static void __exit qup_i2c_exit_driver(void)
{
	platform_driver_unregister(&qup_i2c_driver);
}
module_exit(qup_i2c_exit_driver);

