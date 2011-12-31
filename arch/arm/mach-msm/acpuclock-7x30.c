/*
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/sort.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include <mach/acpuclock_debug.h>

#include "smd_private.h"
#include "clock.h"
#include "acpuclock.h"
#include "socinfo.h"
#include "spm.h"

#define SCSS_CLK_CTL_ADDR	(MSM_ACC_BASE + 0x04)
#define SCSS_CLK_SEL_ADDR	(MSM_ACC_BASE + 0x08)

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, "cpufreq-msm", msg)

#define VREF_SEL     1	/* 0: 0.625V (50mV step), 1: 0.3125V (25mV step). */
#define V_STEP       (25 * (2 - VREF_SEL)) /* Minimum voltage step size. */
#define VREG_DATA    (VREG_CONFIG | (VREF_SEL << 5))
#define VREG_CONFIG  (BIT(7) | BIT(6)) /* Enable VREG, pull-down if disabled. */
/* Cause a compile error if the voltage is not a multiple of the step size. */
#define MV(mv)      ((mv) / (!((mv) % V_STEP)))
/* mv = (750mV + (raw * 25mV)) * (2 - VREF_SEL) */
#define VDD_RAW(mv) (((MV(mv) / V_STEP) - 30) | VREG_DATA)

#define MAX_AXI_KHZ 201600

#define PLL2_L_VAL_ADDR  (MSM_CLK_CTL_BASE + 0x33c)

#define ACE_ACPU_MIN_UV_MV 700U
#define ACE_ACPU_MAX_UV_MV 1550U

struct clock_state {
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			vdd_switch_time_us;
	unsigned long                   power_collapse_khz;
	unsigned long			wait_for_irq_khz;
	int				wfi_ramp_down;
	int				pwrc_ramp_down;
};

struct clkctl_acpu_speed {
	unsigned int	acpu_clk_khz;
	int		src;
	unsigned int	acpu_src_sel;
	unsigned int	acpu_src_div;
	unsigned int	axi_clk_khz;
	unsigned int	vdd_mv;
	unsigned int	vdd_raw;
	unsigned long	lpj; /* loops_per_jiffy */
};

static struct clock_state drv_state = { 0 };

static struct cpufreq_frequency_table freq_table[] = {
	{ 0, 122000 },
	{ 1, 230400 },
	{ 2, 307200 },
	{ 3, 384000 },
	{ 4, 460800 },
	{ 5, 537600 },
	{ 6, 614400 },
	{ 7, 691200 },
	{ 8, 768000 },
	{ 9, 844800 },
	{ 10, 921600 },
	{ 11, 998400 },
	{ 12, 1075200 },
	{ 13, 1152000 },
	{ 14, 1228800 },
	{ 15, 1305600 },
	{ 16, 1382400 },
	{ 17, 1459200 },
	{ 18, 1536000 },
	{ 19, 1612800 },
	{ 20, 1689600 },
	{ 21, 1766400 },
	{ 22, 1843200 },
	{ 23, 1920000 },
	{ 24, 1996800 },
	{ 25, 2016000 },
	{ 26, CPUFREQ_TABLE_END },
};

/* Use negative numbers for sources that can't be enabled/disabled */
#define SRC_LPXO (-2)
#define SRC_AXI  (-1)
static struct clkctl_acpu_speed acpu_freq_tbl[] = {
	{ 24576,  SRC_LPXO, 0, 0,  30720,  850, VDD_RAW(850) },
	{ 61440,  PLL_3,    5, 11, 61440,  900, VDD_RAW(900) },
	{ 122880, PLL_3,    5, 5,  61440,  900, VDD_RAW(900) },
	{ 184320, PLL_3,    5, 4,  61440,  900, VDD_RAW(900) },
      { MAX_AXI_KHZ, SRC_AXI, 1, 0, 61440, 900, VDD_RAW(900) },
//	{ 245000, PLL_3,    5, 2,  122500, 900, VDD_RAW(900) },
	{ 122000, PLL_3,    5, 2, 61440, 875, VDD_RAW(875) },
	{ 230400, PLL_3,    5, 1, 192000, 900, VDD_RAW(900) },
	{ 307200, PLL_3,    5, 1, 192000, 900, VDD_RAW(900) },
	{ 384000, PLL_1,    2, 0, 192000, 925, VDD_RAW(925) },
	{ 460800, PLL_3,    5, 1, 192000, 950, VDD_RAW(950) },
	{ 537600, PLL_2,    3, 0, 192000, 975, VDD_RAW(975) },
	{ 614400, PLL_2,    3, 0, 192000, 975, VDD_RAW(975) },
	{ 691200, PLL_2,    3, 0, 192000, 1000, VDD_RAW(1000) },
	{ 768000, PLL_2,    3, 0, 192000, 1025, VDD_RAW(1025) },
	{ 844800, PLL_2,    3, 0, 192000, 1025, VDD_RAW(1025) },
	{ 921600, PLL_2,    3, 0, 192000, 1050, VDD_RAW(1050) },
	{ 998400, PLL_2,    3, 0, 192000, 1050, VDD_RAW(1050) },
	{ 1075200, PLL_2,   3, 0, 192000, 1075, VDD_RAW(1075) },
	{ 1152000, PLL_2,   3, 0, 192000, 1100, VDD_RAW(1100) },
	{ 1228800, PLL_2,   3, 0, 192000, 1100, VDD_RAW(1100) },
	{ 1305600, PLL_2,   3, 0, 192000, 1150, VDD_RAW(1150) },
	{ 1382400, PLL_2,   3, 0, 192000, 1200, VDD_RAW(1200) },
	{ 1459200, PLL_2,   3, 0, 192000, 1225, VDD_RAW(1225) },
	{ 1536000, PLL_2,   3, 0, 192000, 1300, VDD_RAW(1300) },
	{ 1612800, PLL_2,   3, 0, 192000, 1325, VDD_RAW(1325) },
	{ 1689600, PLL_2,   3, 0, 192000, 1375, VDD_RAW(1375) },
	{ 1766400, PLL_2,   3, 0, 192000, 1425, VDD_RAW(1425) },
	{ 1843200, PLL_2,   3, 0, 192000, 1450, VDD_RAW(1450) },
	{ 1920000, PLL_2,   3, 0, 199680, 1475, VDD_RAW(1475) },
	{ 1996800, PLL_2,   3, 0, 199680, 1500, VDD_RAW(1500) },
	{ 2016000, PLL_2,   3, 0, 201600, 1525, VDD_RAW(1525) },
	{ 0 }
};
static unsigned long max_axi_rate;

#define POWER_COLLAPSE_HZ (MAX_AXI_KHZ * 1000)
unsigned long acpuclk_power_collapse(int from_idle)
{
	int ret = acpuclk_get_rate();
	ret *= 1000;
	if (ret > drv_state.power_collapse_khz)
		acpuclk_set_rate(drv_state.power_collapse_khz,
	(from_idle ? SETRATE_PC_IDLE : SETRATE_PC));
	return ret;
}

unsigned long acpuclk_get_wfi_rate(void)
{
	return drv_state.wait_for_irq_khz;
}

#define WAIT_FOR_IRQ_HZ (MAX_AXI_KHZ * 1000)
unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate();
	ret *= 1000;
	if (ret > drv_state.wait_for_irq_khz)
		acpuclk_set_rate(drv_state.wait_for_irq_khz, SETRATE_SWFI);
	return ret;
}

static int acpuclk_set_acpu_vdd(struct clkctl_acpu_speed *s)
{
	int ret = msm_spm_set_vdd(s->vdd_raw);
	if (ret)
		return ret;

	/* Wait for voltage to stabilize. */
	udelay(drv_state.vdd_switch_time_us);
	return 0;
}

/* Set clock source and divider given a clock speed */
static void acpuclk_set_src(const struct clkctl_acpu_speed *s)
{
	uint32_t reg_clksel, reg_clkctl, src_sel;

	reg_clksel = readl(SCSS_CLK_SEL_ADDR);

	/* CLK_SEL_SRC1NO */
	src_sel = reg_clksel & 1;

	/* Program clock source and divider. */
	reg_clkctl = readl(SCSS_CLK_CTL_ADDR);
	reg_clkctl &= ~(0xFF << (8 * src_sel));
	reg_clkctl |= s->acpu_src_sel << (4 + 8 * src_sel);
	reg_clkctl |= s->acpu_src_div << (0 + 8 * src_sel);
	writel(reg_clkctl, SCSS_CLK_CTL_ADDR);

	/* Program PLL2 L val for overclocked speeds. */
	if(s->src == PLL_2) {
		writel(s->acpu_clk_khz/19200, PLL2_L_VAL_ADDR);
	}

	/* Toggle clock source. */
	reg_clksel ^= 1;

	/* Program clock source selection. */
	writel(reg_clksel, SCSS_CLK_SEL_ADDR);
}

static struct clk *ebi1_clk;

int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason)
{
	struct clkctl_acpu_speed *tgt_s, *strt_s;
	int res, rc = 0;

	if (reason == SETRATE_CPUFREQ)
		mutex_lock(&drv_state.lock);

	strt_s = drv_state.current_speed;

	if (rate == (strt_s->acpu_clk_khz * 1000))
		goto out;

	for (tgt_s = acpu_freq_tbl; tgt_s->acpu_clk_khz != 0; tgt_s++) {
		if (tgt_s->acpu_clk_khz == (rate / 1000))
			break;
	}
	if (tgt_s->acpu_clk_khz == 0) {
		rc = -EINVAL;
		goto out;
	}

	if (reason == SETRATE_CPUFREQ) {
		/* Increase VDD if needed. */
		if (tgt_s->vdd_mv > strt_s->vdd_mv) {
			rc = acpuclk_set_acpu_vdd(tgt_s);
			if (rc < 0) {
				pr_err("ACPU VDD increase to %d mV failed "
					"(%d)\n", tgt_s->vdd_mv, rc);
				goto out;
			}
		}
	}

	dprintk("Switching from ACPU rate %u KHz -> %u KHz\n",
	       strt_s->acpu_clk_khz, tgt_s->acpu_clk_khz);

	/* Increase the AXI bus frequency if needed. This must be done before
	 * increasing the ACPU frequency, since voting for high AXI rates
	 * implicitly takes care of increasing the MSMC1 voltage, as needed. */
	if (tgt_s->axi_clk_khz > strt_s->axi_clk_khz) {
		res = clk_set_rate(ebi1_clk, tgt_s->axi_clk_khz * 1000);
		if (rc < 0) {
			pr_err("Setting AXI min rate failed (%d)\n", rc);
			goto out;
		}
	}

	/* Make sure target PLL is on. */
	if (strt_s->src != tgt_s->src && tgt_s->src >= 0) {
		dprintk("Enabling PLL %d\n", tgt_s->src);
		pll_enable(tgt_s->src);
	}

	/* Perform the frequency switch */
	acpuclk_set_src(tgt_s);
	drv_state.current_speed = tgt_s;
	loops_per_jiffy = tgt_s->lpj;

	/* Nothing else to do for SWFI. */
	if (reason == SETRATE_SWFI)
		goto out;

	/* Turn off previous PLL if not used. */
	if (strt_s->src != tgt_s->src && strt_s->src >= 0) {
		dprintk("Disabling PLL %d\n", strt_s->src);
		pll_disable(strt_s->src);
	}

	/* Decrease the AXI bus frequency if we can. */
	if (tgt_s->axi_clk_khz < strt_s->axi_clk_khz) {
		res = clk_set_rate(ebi1_clk, tgt_s->axi_clk_khz * 1000);
		if (res < 0)
			pr_warning("Setting AXI min rate failed (%d)\n", res);
	}

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_PC)
		goto out;

	/* Drop VDD level if we can. */
	if (tgt_s->vdd_mv < strt_s->vdd_mv) {
		res = acpuclk_set_acpu_vdd(tgt_s);
		if (res < 0) {
			pr_warning("ACPU VDD decrease to %d mV failed (%d)\n",
					tgt_s->vdd_mv, res);
		}
	}

	dprintk("ACPU speed change complete\n");
out:
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state.lock);

	return rc;
}

unsigned long acpuclk_get_max_axi_rate(void)
{
	return max_axi_rate;
}
EXPORT_SYMBOL(acpuclk_get_max_axi_rate);

unsigned long acpuclk_get_rate(void)
{
	WARN_ONCE(drv_state.current_speed == NULL,
		  "acpuclk_get_rate: not initialized\n");
	if (drv_state.current_speed)
		return drv_state.current_speed->acpu_clk_khz;
	else
		return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

unsigned long clk_get_max_axi_khz(void)
{
	return MAX_AXI_KHZ;
}
EXPORT_SYMBOL(clk_get_max_axi_khz);

static void acpuclk_set_wfi_ramp_down(int enable)
{
	drv_state.wfi_ramp_down = enable;
}

static void acpuclk_set_pwrc_ramp_down(int enable)
{
	drv_state.pwrc_ramp_down = enable;
}

static int acpuclk_get_wfi_ramp_down(void)
{
	return drv_state.wfi_ramp_down;
}

static int acpuclk_get_pwrc_ramp_down(void)
{
	return drv_state.pwrc_ramp_down;
}

static unsigned int acpuclk_get_current_vdd(void)
{
	unsigned int vdd_raw;
	unsigned int vdd_mv;

	vdd_raw = msm_spm_get_vdd();
	for (vdd_mv = ACE_ACPU_MIN_UV_MV; vdd_mv <= ACE_ACPU_MAX_UV_MV; vdd_mv += V_STEP)
		if (VDD_RAW(vdd_mv) == vdd_raw)
			break;

	if (vdd_mv > ACE_ACPU_MAX_UV_MV)
		return 0;

	return vdd_mv;
}

static int acpuclk_update_freq_tbl(unsigned int acpu_khz, unsigned int acpu_vdd)
{
	struct clkctl_acpu_speed *s;

	/* Check frequency table for matching sel/div pair. */
	for (s = acpu_freq_tbl; s->acpu_clk_khz != 0; s++) {
		if (s->acpu_clk_khz == acpu_khz)
			break;
	}
	if (s->acpu_clk_khz == 0) {
		pr_err("%s: acpuclk invalid speed %d\n", __func__, acpu_khz);
		return -1;
	}
	if (acpu_vdd > ACE_ACPU_MAX_UV_MV || acpu_vdd < ACE_ACPU_MIN_UV_MV) {
		pr_err("%s: acpuclk vdd out of ranage, %d\n",
			__func__, acpu_vdd);
		return -2;
	}

	s->vdd_mv = acpu_vdd;
	s->vdd_raw = VDD_RAW(acpu_vdd);
	if (drv_state.current_speed->acpu_clk_khz == acpu_khz)
		return acpuclk_set_acpu_vdd(s);

	return 0;
}

static struct acpuclock_debug_dev acpu_debug_7x30 = {
	.name = "acpu-7x30",
	.set_wfi_ramp_down = acpuclk_set_wfi_ramp_down,
	.set_pwrc_ramp_down = acpuclk_set_pwrc_ramp_down,
	.get_wfi_ramp_down = acpuclk_get_wfi_ramp_down,
	.get_pwrc_ramp_down = acpuclk_get_pwrc_ramp_down,
	.get_current_vdd = acpuclk_get_current_vdd,
	.update_freq_tbl = acpuclk_update_freq_tbl,
};

/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *s;
	uint32_t div, sel, src_num;
	uint32_t reg_clksel, reg_clkctl;
	int res;

	reg_clksel = readl(SCSS_CLK_SEL_ADDR);

	/* Determine the ACPU clock rate. */
	switch ((reg_clksel >> 1) & 0x3) {
	case 0:	/* Running off the output of the raw clock source mux. */
		reg_clkctl = readl(SCSS_CLK_CTL_ADDR);
		src_num = reg_clksel & 0x1;
		sel = (reg_clkctl >> (12 - (8 * src_num))) & 0x7;
		div = (reg_clkctl >> (8 -  (8 * src_num))) & 0xF;

		/* Check frequency table for matching sel/div pair. */
		for (s = acpu_freq_tbl; s->acpu_clk_khz != 0; s++) {
			if (s->acpu_src_sel == sel && s->acpu_src_div == div)
				break;
		}
		if (s->acpu_clk_khz == 0) {
			pr_err("Error - ACPU clock reports invalid speed\n");
			return;
		}
		break;
	case 2:	/* Running off of the SCPLL selected through the core mux. */
		/* Switch to run off of the SCPLL selected through the raw
		 * clock source mux. */
		for (s = acpu_freq_tbl; s->acpu_clk_khz != 0
			&& s->src != PLL_2 && s->acpu_src_div == 0; s++)
			;
		if (s->acpu_clk_khz != 0) {
			/* Program raw clock source mux. */
			acpuclk_set_src(s);

			/* Switch to raw clock source input of the core mux. */
			reg_clksel = readl(SCSS_CLK_SEL_ADDR);
			reg_clksel &= ~(0x3 << 1);
			writel(reg_clksel, SCSS_CLK_SEL_ADDR);
			break;
		}
		/* else fall through */
	default:
		pr_err("Error - ACPU clock reports invalid source\n");
		return;
	}

	/* Set initial ACPU VDD. */
	acpuclk_set_acpu_vdd(s);

	drv_state.current_speed = s;

	/* Initialize current PLL's reference count. */
	if (s->src >= 0)
		pll_enable(s->src);

	ebi1_clk = clk_get(NULL, "ebi1_clk");
	BUG_ON(ebi1_clk == NULL);

	res = clk_set_rate(ebi1_clk, s->axi_clk_khz * 1000);
	if (res < 0)
		pr_warning("Setting AXI min rate failed!\n");

	pr_info("ACPU running at %d KHz\n", s->acpu_clk_khz);

	s = acpu_freq_tbl + ARRAY_SIZE(acpu_freq_tbl) - 2;
	max_axi_rate = s->axi_clk_khz * 1000;
	return;
}

/* Initalize the lpj field in the acpu_freq_tbl. */
static void __init lpj_init(void)
{
	int i;
	const struct clkctl_acpu_speed *base_clk = drv_state.current_speed;

	for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->acpu_clk_khz,
						acpu_freq_tbl[i].acpu_clk_khz);
	}
}

static struct cpufreq_frequency_table cpufreq_tbl[ARRAY_SIZE(acpu_freq_tbl)];

static void setup_cpufreq_table(void)
{
	unsigned i;
	const struct clkctl_acpu_speed *speed;

	for (i = 0, speed = acpu_freq_tbl; speed->acpu_clk_khz; i++, speed++) {
		cpufreq_tbl[i].index = i;

		cpufreq_tbl[i].frequency = speed->acpu_clk_khz;
	}
	cpufreq_tbl[i].frequency = CPUFREQ_TABLE_END;

	cpufreq_frequency_table_get_attr(cpufreq_tbl, smp_processor_id());
}

#define RPM_BYPASS_MASK	(1 << 3)
#define PMIC_MODE_MASK	(1 << 4)
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	drv_state.power_collapse_khz = clkdata->power_collapse_khz;
	drv_state.wfi_ramp_down = 1;
	drv_state.pwrc_ramp_down = 1;
	acpuclk_init();
	lpj_init();

	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
	register_acpuclock_debug_dev(&acpu_debug_7x30);
}

#ifdef CONFIG_CPU_FREQ_VDD_LEVELS

ssize_t acpuclk_get_vdd_levels_str(char *buf)
{
int i, len = 0;
if (buf)
{
mutex_lock(&drv_state.lock);
for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++)
{
len += sprintf(buf + len, "%8u: %4d\n", acpu_freq_tbl[i].acpu_clk_khz, acpu_freq_tbl[i].vdd_mv);
}
mutex_unlock(&drv_state.lock);
}
return len;
}

void acpuclk_set_vdd(unsigned int khz, int vdd)
{
int i;
unsigned int new_vdd;
vdd = vdd / V_STEP * V_STEP;
mutex_lock(&drv_state.lock);
for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++)
{
if (khz == 0)
new_vdd = min(max((acpu_freq_tbl[i].vdd_mv + vdd), ACE_ACPU_MIN_UV_MV), ACE_ACPU_MAX_UV_MV);
else if (acpu_freq_tbl[i].acpu_clk_khz == khz)
new_vdd = min(max((unsigned int)vdd, ACE_ACPU_MIN_UV_MV), ACE_ACPU_MAX_UV_MV);
else continue;

acpu_freq_tbl[i].vdd_mv = new_vdd;
acpu_freq_tbl[i].vdd_raw = VDD_RAW(new_vdd);
}
mutex_unlock(&drv_state.lock);
}

#endif

#ifdef CONFIG_CPU_FREQ_USER_FREQS
ssize_t acpuclk_get_user_freqs(char *buf, struct cpufreq_policy *policy)
{
	int i, len = 0;
	if (buf)
	{
		mutex_lock(&drv_state.lock);
		for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++) 
		{
			if (!(acpu_freq_tbl[i].acpu_clk_khz < policy->cpuinfo.min_freq))
				len += sprintf(buf + len, "%8u: %s\n", acpu_freq_tbl[i].acpu_clk_khz, cpufreq_tbl[i].frequency == CPUFREQ_ENTRY_INVALID ? "Disabled" : "Enabled");
		}
		mutex_unlock(&drv_state.lock);
	}
	return len;
}

void acpuclk_set_user_freqs(unsigned acpu_khz, struct cpufreq_policy *policy)
{
	int i;
	int pol_min = policy->min;
	int pol_max = policy->max;
	
	mutex_lock(&drv_state.lock);
	cpufreq_frequency_table_put_attr(smp_processor_id());

	if (acpu_khz == 0)  // Reset the frequency table to the kernel defaults
		setup_cpufreq_table();
	else
	{
		for (i = 0; acpu_freq_tbl[i].acpu_clk_khz; i++)
		{
			if (acpu_freq_tbl[i].acpu_clk_khz == acpu_khz) {
				/* If the frequency is marked as CPUFREQ_ENTRY_INVALID, reset the
				 * frequency to the value in acpu_freq_tbl.
				 * 
				 * Else, if the frequency is marked as valid, mark it as
				 * CPUFREQ_ENTRY_INVALID. */
				if (cpufreq_tbl[i].frequency == CPUFREQ_ENTRY_INVALID)
					cpufreq_tbl[i].frequency = acpu_freq_tbl[i].acpu_clk_khz;
				else
					cpufreq_tbl[i].frequency = CPUFREQ_ENTRY_INVALID;
			}
			/* Mark all frequencies outside of the policy min and policy max
			 * as CPUFREQ_ENTRY_INVALID. */
			if (acpu_khz == 1 && (acpu_freq_tbl[i].acpu_clk_khz < pol_min || acpu_freq_tbl[i].acpu_clk_khz > pol_max))
				cpufreq_tbl[i].frequency = CPUFREQ_ENTRY_INVALID;
		}
		cpufreq_frequency_table_get_attr(cpufreq_tbl, smp_processor_id());
	}
	mutex_unlock(&drv_state.lock);
}
#endif //CONFIG_CPU_FREQ_USER_FREQS
