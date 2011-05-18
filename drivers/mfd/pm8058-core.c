/*
 * Copyright (c) 2010 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Author: Dima Zavin <dima@android.com>
 *   - Based on a driver from Code Aurora Forum.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
/*include <linux/mfd/pm8058.h>*/
#include <linux/mfd/pmic8058.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <asm-generic/gpio.h>

#include <mach/msm_ssbi.h>

enum {
	DEBUG_IRQS = 1U << 0,
};
static int debug_mask;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define REG_HWREV		0x0002  /* PMIC4 revision */

#define REG_IRQ_ROOT		0x01bb
#define REG_IRQ_M_STATUS1	0x01bc
#define REG_IRQ_M_STATUS2	0x01bd
#define REG_IRQ_M_STATUS3	0x01be
#define REG_IRQ_M_STATUS4	0x01bf
#define REG_IRQ_BLK_SEL		0x01c0
#define REG_IRQ_IT_STATUS	0x01c1
#define REG_IRQ_CONFIG		0x01c2
#define REG_IRQ_RT_STATUS	0x01c3
#define REG_GPIO_CTRL(x)	(0x0150 + (x))

#define IRQ_CFG_CLR		(1 << 3)
#define IRQ_CFG_MASK_RE		(1 << 2)
#define IRQ_CFG_MASK_FE		(1 << 1)
#define IRQ_CFG_LVL_SEL		(1 << 0)

#define NUM_BLOCKS		32
#define IRQS_PER_BLOCK		8
#define NUM_PMIRQS		(NUM_BLOCKS * IRQS_PER_BLOCK)

/* XXX: why are mpp's different than gpios? should we just put them into
 * the gpio space? */
#define MPP_IRQ_OFFSET		(16 * 8)
#define GPIO_IRQ_OFFSET		(24 * 8)
#define KEYPAD_IRQ_OFFSET	(9 * 8 + 2)

/* this defines banks of irq space. We want to provide a compact irq space
 * to the kernel, but there several ranges of irqs in an otherwise sparse
 * map of available/accessible irqs on the pm8058. So,
 *
 * bank 0 - GPIO IRQs start=(24 * 8) cnt=40 (gpios 0-39)
 * bank 1 - MPP IRQs start=(16 * 8) cnt=12 (mpps 0-11)
 * bank 2 - keypad irqs start=(9*8 + 1) cnt=2
 *
 */
struct pm8058_irq_bank {
	unsigned int	start; /* will be added to the chip irq_base */
	unsigned int	cnt;
	unsigned int	offset; /* offset into device's real irq map */
};

static struct pm8058_irq_bank pm8058_irq_banks[] = {
	{
		.start	= PM8058_FIRST_GPIO_IRQ,
		.cnt	= PM8058_NUM_GPIO_IRQS,
		.offset	= GPIO_IRQ_OFFSET,
	},
	{
		.start	= PM8058_FIRST_MPP_IRQ,
		.cnt	= PM8058_NUM_MPP_IRQS,
		.offset	= MPP_IRQ_OFFSET,
	},
	{
		.start	= PM8058_FIRST_KEYPAD_IRQ,
		.cnt	= PM8058_NUM_KEYPAD_IRQS,
		.offset	= KEYPAD_IRQ_OFFSET,
	},
};
#define NUM_IRQ_BANKS		ARRAY_SIZE(pm8058_irq_banks)

struct pm8058_irq_group {
	u16	stat_reg;
	u8	valid_mask;
	u8	root_mask;
	u8	block_offset;
};

static const struct pm8058_irq_group pm8058_irq_groups[] = {
	{
		.stat_reg	= REG_IRQ_M_STATUS2,
		.valid_mask	= 0x2,
		.root_mask	= 0x4,
		.block_offset	= 8,
	},
	{
		.stat_reg	= REG_IRQ_M_STATUS4,
		.valid_mask	= 0x1f,
		.root_mask	= 0x10,
		.block_offset	= 24,
	},
};
#define NUM_ROOT_GROUPS		ARRAY_SIZE(pm8058_irq_groups)

struct pm8058_irq_info {
	u8	cfg;
	u8	cfg_val;
	u8	mask;
	u8	blk;
	u8	blk_bit;
	u8	wake;
};

struct pm8058 {
	struct device			*dev;
	unsigned int			devirq;

	spinlock_t			lock;

	unsigned int			irq_base;
	struct pm8058_irq_info		irqs[PM8058_NUM_IRQS];
	unsigned int			pmirqs[NUM_PMIRQS];
	int				wake_cnt;

	struct gpio_chip		gpio_chip;
	u8				gpio_flags[PM8058_NUM_GPIOS];
};

static struct pm8058 *the_pm8058;

static int read_irq_block_reg(struct pm8058 *pmic, u8 blk, u16 reg, u8 *val);

int pm8058_readb(struct device *dev, u16 addr, u8 *val)
{
	struct pm8058 *pmic = dev_get_drvdata(dev);

	return msm_ssbi_read(pmic->dev->parent, addr, val, 1);
}
EXPORT_SYMBOL(pm8058_readb);

int pm8058_writeb(struct device *dev, u16 addr, u8 val)
{
	struct pm8058 *pmic = dev_get_drvdata(dev);

	return msm_ssbi_write(pmic->dev->parent, addr, &val, 1);
}
EXPORT_SYMBOL(pm8058_writeb);

int pm8058_write_buf(struct device *dev, u16 addr, u8 *buf, int cnt)
{
	struct pm8058 *pmic = dev_get_drvdata(dev);

	return msm_ssbi_write(pmic->dev->parent, addr, buf, cnt);
}
EXPORT_SYMBOL(pm8058_write_buf);

int pm8058_read_buf(struct device *dev, u16 addr, u8 *buf, int cnt)
{
	struct pm8058 *pmic = dev_get_drvdata(dev);

	return msm_ssbi_read(pmic->dev->parent, addr, buf, cnt);
}
EXPORT_SYMBOL(pm8058_read_buf);

static int _dir_map[] = {
	[0]				= 0x3,
	[PM8058_GPIO_INPUT]		= 0x0,
	[PM8058_GPIO_OUTPUT]		= 0x2,
	[PM8058_GPIO_OUTPUT_HIGH]	= 0x2,
};

int pm8058_gpio_mux_cfg(struct device *dev, unsigned int gpio,
			struct pm8058_pin_config *cfg)
{
	struct pm8058 *pmic = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;
	u8 bank[6];

	gpio -= pmic->gpio_chip.base;

	/* bit 7   - write
	 * bit 6:4 - bank select */
	bank[0] = ((1 << 7) | (0 << 4) | ((cfg->vin_src & 0x7) << 1) | 0x1);
	bank[1] = ((1 << 7) | (1 << 4) | (_dir_map[cfg->dir] << 2) |
		   ((cfg->flags & PM8058_GPIO_OPEN_DRAIN ? 0x1 : 0) << 1) |
		   ((cfg->dir & PM8058_GPIO_OUTPUT_HIGH ? 0x1 : 0) << 0));
	bank[2] = ((1 << 7) | (2 << 4) | ((cfg->pull_up & 0x7) << 1));
	bank[3] = ((1 << 7) | (3 << 4) |
		   ((cfg->strength & 0x3) << 2) |
		   ((cfg->flags & PM8058_GPIO_HIGH_Z ? 0x1 : 0x0) << 0));
	bank[4] = ((1 << 7) | (4 << 4) | ((cfg->func & 0x7) << 1));
	bank[5] = ((1 << 7) | (5 << 4) |
		   ((cfg->flags & PM8058_GPIO_INV_IRQ_POL ? 0 : 1) << 3));

	spin_lock_irqsave(&pmic->lock, flags);

	pmic->gpio_flags[gpio] = cfg->flags | PM8058_GPIO_CONFIGURED;
	ret = pm8058_write_buf(pmic->dev, REG_GPIO_CTRL(gpio),
			       bank, sizeof(bank));

	spin_unlock_irqrestore(&pmic->lock, flags);

	if (ret)
		pr_err("%s: failed writing config for gpio %d (%d)\n", __func__,
		       gpio, ret);

	pr_info("%s: ok writing config for gpio %d (%d)\n", __func__,
		       gpio, ret);
	return ret;
}
EXPORT_SYMBOL(pm8058_gpio_mux_cfg);

int pm8058_gpio_mux(unsigned int gpio, struct pm8058_pin_config *cfg)
{
	if (!the_pm8058)
		return -ENODEV;
	return pm8058_gpio_mux_cfg(the_pm8058->dev, gpio, cfg);
}
EXPORT_SYMBOL(pm8058_gpio_mux);

int pm8058_gpio_config(int gpio, struct pm8058_gpio *param)
{
	struct pm8058_pin_config cfg;
	struct pm8058 *pmic = dev_get_drvdata(the_pm8058->dev);

	if (param->output_value == 1) {
		cfg.dir = PM8058_GPIO_OUTPUT_HIGH;
	} else{
		cfg.dir = param->direction;
	}

	cfg.output_buffer = param->output_buffer;
	cfg.output_value = param->output_value;
	cfg.pull_up = param->pull;
	cfg.vin_src = param->vin_sel;
	cfg.strength = param->out_strength;
	cfg.func = param->function;
	cfg.flags = param->inv_int_pol;

	gpio += pmic->gpio_chip.base;
	return pm8058_gpio_mux(gpio, &cfg);
}
EXPORT_SYMBOL(pm8058_gpio_config);

int pm8058_gpio_cfg(int num, int direction, int output_buffer,
			int output_value, int pull, int vin_sel, int out_strength, int function, int inv_int_pol)
{
	struct pm8058_gpio pmic_gpio_setting = {
		.direction      = (u8)direction,
		.output_buffer  = (u8)output_buffer,
		.output_value   = (u8)output_value,
		.pull           = (u8)pull,
		.vin_sel        = (u8)vin_sel,
		.out_strength   = (u8)out_strength,
		.function       = (u8)function,
		.inv_int_pol    = (u8)inv_int_pol,
		};
	return pm8058_gpio_config(num, &pmic_gpio_setting);
}
EXPORT_SYMBOL(pm8058_gpio_cfg);
/* gpio funcs */
static int read_gpio_bank(struct pm8058 *pmic, unsigned gpio, u8 bank, u8 *val)
{
	int ret;

	ret = pm8058_writeb(pmic->dev, REG_GPIO_CTRL(gpio), (bank & 0x7) << 4);
	if (ret)
		goto out;
	ret = pm8058_readb(pmic->dev, REG_GPIO_CTRL(gpio), val);
	if (ret)
		goto out;
out:
	return ret;
}

static int pm8058_gpio_request(struct gpio_chip *chip, unsigned gpio)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);
	unsigned long flags;
	int ret;
	u8 bank1;
	u8 bank3;
	u8 bank5;

	spin_lock_irqsave(&pmic->lock, flags);
	if (pmic->gpio_flags[gpio] & PM8058_GPIO_CONFIGURED) {
		ret = 0;
		goto out;
	}

	ret = read_gpio_bank(pmic, gpio, 1, &bank1);
	if (ret) {
		pr_err("%s: can't read bank 1\n", __func__);
		goto out;
	}

	ret = read_gpio_bank(pmic, gpio, 3, &bank3);
	if (ret) {
		pr_err("%s: can't read bank 3\n", __func__);
		goto out;
	}

	ret = read_gpio_bank(pmic, gpio, 5, &bank5);
	if (ret) {
		pr_err("%s: can't read bank 5\n", __func__);
		goto out;
	}

	pmic->gpio_flags[gpio] = bank1 & 0x2 ? PM8058_GPIO_OPEN_DRAIN : 0;
	pmic->gpio_flags[gpio] |= bank3 & 0x1 ? PM8058_GPIO_HIGH_Z : 0;
	pmic->gpio_flags[gpio] |= bank5 & 0x8 ? 0 : PM8058_GPIO_INV_IRQ_POL;
	pmic->gpio_flags[gpio] |= PM8058_GPIO_CONFIGURED;

out:
	spin_unlock_irqrestore(&pmic->lock, flags);
	return 0;
}

static void pm8058_gpio_free(struct gpio_chip *chip, unsigned gpio)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);
	unsigned long flags;

	/* XXX: set high Z maybe?? */
	spin_lock_irqsave(&pmic->lock, flags);
	pmic->gpio_flags[gpio] = 0;
	spin_unlock_irqrestore(&pmic->lock, flags);
}

static int gpio_set_dir(struct pm8058 *pmic, unsigned gpio, int dir)
{
	unsigned long flags;
	int ret;
	u8 val;

	spin_lock_irqsave(&pmic->lock, flags);
	/* only need to write bank1 */
	val = (pmic->gpio_flags[gpio] & PM8058_GPIO_OPEN_DRAIN ? 0x1 : 0) << 1;
	val |= ((1 << 7) | (1 << 4) | (_dir_map[dir] << 2) |
		(dir & PM8058_GPIO_OUTPUT_HIGH ? 0x1 : 0x0));
	ret = pm8058_writeb(pmic->dev, REG_GPIO_CTRL(gpio), val);
	if (ret)
		pr_err("%s: erorr setting dir %x (%d)\n", __func__, dir, ret);

	spin_unlock_irqrestore(&pmic->lock, flags);
	return ret;
}

int gpio_set_dir_htc(struct device *dev, unsigned gpio, int dir)
{
	struct pm8058 *pmic = dev_get_drvdata(dev);
	return gpio_set_dir(pmic, gpio, dir);
}

static int pm8058_gpio_direction_in(struct gpio_chip *chip, unsigned gpio)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);

	return gpio_set_dir(pmic, gpio, PM8058_GPIO_INPUT);
}

static int pm8058_gpio_direction_out(struct gpio_chip *chip, unsigned gpio,
				     int val)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);

	val = val ? PM8058_GPIO_OUTPUT_HIGH : PM8058_GPIO_OUTPUT;
	return gpio_set_dir(pmic, gpio, val);
}

static void pm8058_gpio_set(struct gpio_chip *chip, unsigned gpio, int val)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);

	/* XXX: for now, let's always force the gpio to be an output when
	 * the user calls this func. I'm not even sure that it's wrong to
	 * assume that. */
	val = val ? PM8058_GPIO_OUTPUT_HIGH : PM8058_GPIO_OUTPUT;
	gpio_set_dir(pmic, gpio, val);
}

static int pm8058_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);
	int ret;
	u8 val;

	/* XXX: assumes gpio maps 1:1 to irq @ 0 */
	ret = read_irq_block_reg(pmic, pmic->irqs[gpio].blk, REG_IRQ_RT_STATUS,
				 &val);
	if (ret) {
		pr_err("%s: can't read block status\n", __func__);
		goto done;
	}

	ret = !!(val & (1 << pmic->irqs[gpio].blk_bit));
done:
	return ret;
}

static int pm8058_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	struct pm8058 *pmic = container_of(chip, struct pm8058, gpio_chip);
	return pmic->irq_base + gpio;
}

static struct gpio_chip pm8058_base_gpio_chip = {
	.label			= "pm8058",
	.owner			= THIS_MODULE,
	.request		= pm8058_gpio_request,
	.free			= pm8058_gpio_free,
	.direction_input	= pm8058_gpio_direction_in,
	.get			= pm8058_gpio_get,
	.direction_output	= pm8058_gpio_direction_out,
	.set			= pm8058_gpio_set,
	.to_irq			= pm8058_gpio_to_irq,
};

/* irq funcs */
static int read_irq_block_reg(struct pm8058 *pmic, u8 blk, u16 reg, u8 *val)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&pmic->lock, flags);
	ret = pm8058_writeb(pmic->dev, REG_IRQ_BLK_SEL, blk);
	if (ret) {
		pr_err("%s: error setting block select (%d)\n", __func__, ret);
		goto done;
	}

	ret = pm8058_readb(pmic->dev, reg, val);
	if (ret)
		pr_err("%s: error setting bit select (%d)\n", __func__, ret);

done:
	spin_unlock_irqrestore(&pmic->lock, flags);
	return ret;
}

static int _write_irq_blk_bit_cfg(struct pm8058 *pmic, u8 blk, u8 bit, u8 cfg)
{
	int ret;

	ret = pm8058_writeb(pmic->dev, REG_IRQ_BLK_SEL, blk);
	if (ret) {
		pr_err("%s: error setting block select (%d)\n", __func__, ret);
		goto done;
	}

	cfg = (1 << 7) | (cfg & 0xf) | (bit << 4);
	ret = pm8058_writeb(pmic->dev, REG_IRQ_CONFIG, cfg);
	if (ret)
		pr_err("%s: error writing irq cfg (%d)\n", __func__, ret);

done:
	return ret;
}

static int write_irq_config_locked(struct pm8058 *pmic, unsigned int irq,
				   u8 cfg)
{
	return _write_irq_blk_bit_cfg(pmic, pmic->irqs[irq].blk,
				      pmic->irqs[irq].blk_bit, cfg);
}

static int do_irq_master(struct pm8058 *pmic, int group)
{
	int i;
	int j;
	int ret;
	u8 val;
	unsigned long flags;
	unsigned long stat;

	ret = pm8058_readb(pmic->dev, pm8058_irq_groups[group].stat_reg, &val);
	if (ret) {
		pr_err("%s: Can't read master status\n", __func__);
		goto done;
	}

	if (debug_mask & DEBUG_IRQS)
		pr_info("%s: master %d %02x\n", __func__, group, val);
	stat = val & pm8058_irq_groups[group].valid_mask;
	for_each_set_bit(i, &stat, BITS_PER_BYTE) {
		u8 blk = pm8058_irq_groups[group].block_offset + i;
		unsigned long blk_stat;

		ret = read_irq_block_reg(pmic, blk, REG_IRQ_IT_STATUS, &val);
		if (ret) {
			pr_err("%s: can't read block status\n", __func__);
			goto done;
		}

		blk_stat = val;
		for_each_set_bit(j, &blk_stat, BITS_PER_BYTE) {
			u8 irq = blk * 8 + j;

			/* XXX: we should mask these out and count em' */
			if (pmic->pmirqs[irq] == 0xffffffffU) {
				pr_warning("Unexpected pmirq %d\n", irq);
				continue;
			}

			local_irq_save(flags);
			if (debug_mask & DEBUG_IRQS)
				pr_info("%s: irq %d is fired\n", __func__, pmic->pmirqs[irq] + pmic->irq_base);
			generic_handle_irq(pmic->pmirqs[irq] + pmic->irq_base);
			local_irq_restore(flags);
		}
	}

done:
	return ret;
}

static irqreturn_t pm8058_irq_handler(int irq, void *dev)
{
	struct pm8058 *pmic = dev;
	int ret;
	int i;
	u8 root;

	ret = pm8058_readb(pmic->dev, REG_IRQ_ROOT, &root);
	if (ret) {
		pr_err("%s: Can't read root status\n", __func__);
		goto done;
	}

	if (debug_mask & DEBUG_IRQS)
		pr_info("%s: root %02x\n", __func__, root);
	for (i = 0; i < NUM_ROOT_GROUPS; ++i) {
		if (root & pm8058_irq_groups[i].root_mask)
			do_irq_master(pmic, i);
	}

done:
	return IRQ_HANDLED;
}

static void pm8058_irq_ack(unsigned int _irq)
{
	struct pm8058 *pmic = get_irq_chip_data(_irq);
	unsigned int irq = _irq - pmic->irq_base;
	unsigned long flags;

	spin_lock_irqsave(&pmic->lock, flags);
	write_irq_config_locked(pmic, irq,
				pmic->irqs[irq].cfg_val | IRQ_CFG_CLR);
	spin_unlock_irqrestore(&pmic->lock, flags);
}

static void pm8058_irq_mask(unsigned int _irq)
{
	struct pm8058 *pmic = get_irq_chip_data(_irq);
	unsigned int irq = _irq - pmic->irq_base;
	struct pm8058_irq_info *irq_info = &pmic->irqs[irq];
	unsigned long flags;

	spin_lock_irqsave(&pmic->lock, flags);
	irq_info->mask = IRQ_CFG_MASK_FE | IRQ_CFG_MASK_RE;
	irq_info->cfg_val = irq_info->cfg | irq_info->mask;
	write_irq_config_locked(pmic, irq, irq_info->cfg_val);
	spin_unlock_irqrestore(&pmic->lock, flags);
}

static void pm8058_irq_unmask(unsigned int _irq)
{
	struct pm8058 *pmic = get_irq_chip_data(_irq);
	unsigned int irq = _irq - pmic->irq_base;
	struct pm8058_irq_info *irq_info = &pmic->irqs[irq];
	unsigned long flags;

	spin_lock_irqsave(&pmic->lock, flags);
	irq_info->mask = 0;
	irq_info->cfg_val = irq_info->cfg;
	write_irq_config_locked(pmic, irq, irq_info->cfg_val);
	spin_unlock_irqrestore(&pmic->lock, flags);
}

static void pm8058_irq_disable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	pm8058_irq_mask(irq);
	desc->status |= IRQ_MASKED;
}

static void pm8058_irq_enable(unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);

	pm8058_irq_unmask(irq);
	desc->status &= ~IRQ_MASKED;
}

static int pm8058_irq_set_type(unsigned int _irq, unsigned int flow_type)
{
	struct pm8058 *pmic = get_irq_chip_data(_irq);
	unsigned int irq = _irq - pmic->irq_base;
	struct pm8058_irq_info *irq_info = &pmic->irqs[irq];
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pmic->lock, flags);
	irq_info->cfg = IRQ_CFG_MASK_RE | IRQ_CFG_MASK_FE;

	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		if (flow_type & IRQF_TRIGGER_RISING)
			irq_info->cfg &= ~IRQ_CFG_MASK_RE;
		if (flow_type & IRQF_TRIGGER_FALLING)
			irq_info->cfg &= ~IRQ_CFG_MASK_FE;
	} else {
		irq_info->cfg |= IRQ_CFG_LVL_SEL;
		if (flow_type & IRQF_TRIGGER_HIGH)
			irq_info->cfg &= ~IRQ_CFG_MASK_RE;
		else
			irq_info->cfg &= ~IRQ_CFG_MASK_FE;
	}

	/* in case the irq was masked when the type was set, we don't want
	 * to unmask it */
	irq_info->cfg_val = irq_info->cfg | irq_info->mask;
	ret = write_irq_config_locked(pmic, irq,
				      irq_info->cfg_val | IRQ_CFG_CLR);
	spin_unlock_irqrestore(&pmic->lock, flags);

	return ret;
}

static int pm8058_irq_set_wake(unsigned int _irq, unsigned int on)
{
	struct pm8058 *pmic = get_irq_chip_data(_irq);
	unsigned int irq = _irq - pmic->irq_base;
	struct pm8058_irq_info *irq_info = &pmic->irqs[irq];
	unsigned long flags;

	spin_lock_irqsave(&pmic->lock, flags);
	if (on) {
		if (!irq_info->wake) {
			irq_info->wake = 1;
			pmic->wake_cnt++;
		}
	} else {
		if (irq_info->wake) {
			irq_info->wake = 0;
			pmic->wake_cnt--;
		}
	}
	spin_unlock_irqrestore(&pmic->lock, flags);

	return 0;
}

static struct irq_chip pm8058_irq_chip = {
	.name		= "pm8058",
	.ack		= pm8058_irq_ack,
	.mask		= pm8058_irq_mask,
	.unmask		= pm8058_irq_unmask,
	.disable	= pm8058_irq_disable,
	.enable		= pm8058_irq_enable,
	.set_type	= pm8058_irq_set_type,
	.set_wake	= pm8058_irq_set_wake,
};

static int pm8058_irq_init(struct pm8058 *pmic, unsigned int irq_base)
{
	int i;
	int j;

	/* mask/clear all the irqs */
	for (i = 0; i < NUM_BLOCKS; ++i)
		for (j = 0; j < IRQS_PER_BLOCK; ++j)
			_write_irq_blk_bit_cfg(pmic, i, j, (IRQ_CFG_MASK_RE |
							    IRQ_CFG_MASK_FE |
							    IRQ_CFG_CLR));

	memset(pmic->pmirqs, 0xff, NUM_PMIRQS * sizeof(pmic->pmirqs[0]));
	for (i = 0; i < NUM_IRQ_BANKS; ++i) {
		struct pm8058_irq_bank *bank = &pm8058_irq_banks[i];

		for (j = 0; j < bank->cnt; ++j) {
			unsigned int irq = bank->start + j;
			unsigned int pmirq = bank->offset + j;
			pr_info("%s: j=%d,start=%d,irq =%d,cnt = %d,PM8058_NUM_IRQS = %d'\n",
				__func__, j, bank->start, irq, bank->cnt, PM8058_NUM_IRQS);
			BUG_ON(irq >= PM8058_NUM_IRQS);

			/* by default mask the irq */
			pmic->irqs[irq].cfg = 0;
			pmic->irqs[irq].mask =
				IRQ_CFG_MASK_RE | IRQ_CFG_MASK_FE;
			pmic->irqs[irq].cfg_val = pmic->irqs[irq].mask;
			pmic->irqs[irq].blk = pmirq / 8;
			pmic->irqs[irq].blk_bit = pmirq % 8;
			pmic->pmirqs[pmirq] = irq;

			BUG_ON(pmic->irqs[irq].blk >= NUM_BLOCKS);

			set_irq_chip(irq_base + irq, &pm8058_irq_chip);
			set_irq_chip_data(irq_base + irq, pmic);
			set_irq_handler(irq_base + irq, handle_edge_irq);
			set_irq_flags(irq_base + irq, IRQF_VALID);
		}

	}

	return 0;
}

static int add_keypad_device(struct pm8058 *pmic, void *pdata)
{
	struct platform_device *pdev;
	struct resource irq_res[2];
	int ret;

	pdev = platform_device_alloc("pm8058-keypad", -1);
	if (!pdev) {
		pr_err("%s: cannot allocate pdev for keypad\n", __func__);
		return -ENOMEM;
	}

	pdev->dev.parent = pmic->dev;
	pdev->dev.platform_data = pdata;

	memset(&irq_res, 0, sizeof(irq_res));
	irq_res[0].start = pmic->irq_base + PM8058_KEYPAD_IRQ;
	irq_res[0].end = irq_res[0].start;
	irq_res[0].flags = IORESOURCE_IRQ;
	irq_res[0].name = "kp_sense";
	irq_res[1].start = pmic->irq_base + PM8058_KEYPAD_STUCK_IRQ;
	irq_res[1].end = irq_res[1].start;
	irq_res[1].flags = IORESOURCE_IRQ;
	irq_res[1].name = "kp_stuck";
	ret = platform_device_add_resources(pdev, irq_res, ARRAY_SIZE(irq_res));
	if (ret) {
		pr_err("%s: can't add irq resources for keypad'\n", __func__);
		goto err;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: cannot add child keypad platform device for\n",
		       __func__);
		goto err;
	}

	return 0;

err:
	if (pdev)
		platform_device_put(pdev);
	return ret;
}

static int add_sub_device(struct pm8058 *pmic, struct pm8058_sub_devices_data *pdata, const char *name)
{
	struct platform_device *pdev;
	int ret;

	pr_info("%s: add_sub_device name %s\n", __func__, name);
	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		pr_err("%s: cannot allocate pdev for %s\n", __func__, name);
		return -ENOMEM;
}

	pdev->dev.parent = pmic->dev;
	platform_set_drvdata(pdev, pdata->driver_data);
	pdev->dev.platform_data = pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: cannot add child  platform device for %s\n",
		       __func__, name);
		goto err;
	}

	return 0;

err:
	if (pdev)
		platform_device_put(pdev);
	return ret;
}

static int pm8058_probe(struct platform_device *pdev)
{
	struct pm8058_platform_data *pdata = pdev->dev.platform_data;
	struct pm8058 *pmic;
	int devirq;
	int ret, i;
	u8 val;

	if (!pdata) {
		pr_err("%s: no platform data\n", __func__);
		return -EINVAL;
	}

	devirq = platform_get_irq(pdev, 0);
	if (devirq < 0) {
		pr_err("%s: missing devirq\n", __func__);
		return devirq;
	}

	pmic = kzalloc(sizeof(struct pm8058), GFP_KERNEL);
	if (!pmic) {
		pr_err("%s: Cannot alloc pm8058 struct\n", __func__);
		return -ENOMEM;
	}

	/* Read PMIC chip revision */
	ret = msm_ssbi_read(pdev->dev.parent, REG_HWREV, &val, sizeof(val));
	if (ret)
		goto err_read_rev;
	pr_info("%s: PMIC revision: %x\n", __func__, val);

	pmic->dev = &pdev->dev;
	pmic->irq_base = pdata->irq_base;
	pmic->devirq = devirq;
	spin_lock_init(&pmic->lock);
	platform_set_drvdata(pdev, pmic);

	ret = pm8058_irq_init(pmic, pmic->irq_base);
	if (ret)
		goto err_irq_init;

	memcpy(&pmic->gpio_chip, &pm8058_base_gpio_chip,
	       sizeof(struct gpio_chip));
	pmic->gpio_chip.dev = pmic->dev;
	pmic->gpio_chip.base = pdata->gpio_base;
	pmic->gpio_chip.ngpio = PM8058_NUM_GPIOS;

	ret = gpiochip_add(&pmic->gpio_chip);
	if (ret) {
		pr_err("%s: can't register gpio chip\n", __func__);
		goto err_gpiochip_add;
	}

	ret = request_irq(devirq, pm8058_irq_handler, IRQF_TRIGGER_LOW,
			  "pm8058-irq", pmic);
	if (ret) {
		pr_err("%s: can't request device irq\n", __func__);
		goto err_request_irq;
	} else {
		set_irq_data(devirq, (void *)pmic);
		set_irq_wake(devirq, 1);
	}

	the_pm8058 = pmic;

	if (pdata->init) {
		ret = pdata->init(pmic->dev);
		if (ret) {
			pr_err("%s: error in board init\n", __func__);
			goto err_pdata_init;
		}
	}

	if (pdata->keypad_pdata) {
		ret = add_keypad_device(pmic, pdata->keypad_pdata);
		if (ret) {
			pr_err("%s: can't add child keypad device\n", __func__);
			goto err_add_kp_dev;
		}
	}
	for (i = 0; i < pdata->num_subdevs; i++) {
		pr_info("%s: %s\n", __func__, pdata->sub_devices_htc[i].name);
		pdata->sub_devices_htc[i].driver_data = pmic;
		add_sub_device(pmic, &(pdata->sub_devices_htc[i]), pdata->sub_devices_htc[i].name);
	}

	pr_info("%s: done\n", __func__);
	return 0;

err_add_kp_dev:
err_pdata_init:
	the_pm8058 = NULL;
	free_irq(devirq, pmic);
err_request_irq:
	WARN_ON(gpiochip_remove(&pmic->gpio_chip));
err_gpiochip_add:
err_irq_init:
	platform_set_drvdata(pdev, NULL);
err_read_rev:
	kfree(pmic);
	return ret;
}

static struct platform_driver pm8058_driver = {
	.probe		= pm8058_probe,
	.driver		= {
		.name	= "pm8058-core",
		.owner	= THIS_MODULE,
	},
};

static int __init pm8058_init(void)
{
	pr_info("%s: PMIC 8058\n", __func__);
	return platform_driver_register(&pm8058_driver);
}
postcore_initcall(pm8058_init);
