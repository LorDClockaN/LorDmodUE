/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __PMIC8058_PWM_H__
#define __PMIC8058_PWM_H__

/* The MAX value is computation limit. Hardware limit is 393 seconds. */
#define	PM_PWM_PERIOD_MAX		(274 * USEC_PER_SEC)
/* The MIN value is hardware limit. */
#define	PM_PWM_PERIOD_MIN		7 /* micro seconds */

#define	PM_PWM_LUT_SIZE			64
#define	PM_PWM_LUT_DUTY_TIME_MAX	512	/* ms */
#define	PM_PWM_LUT_PAUSE_MAX		(7000 * PM_PWM_LUT_DUTY_TIME_MAX)

/* Flags for Look Up Table */
#define	PM_PWM_LUT_LOOP		0x01
#define	PM_PWM_LUT_RAMP_UP	0x02
#define	PM_PWM_LUT_REVERSE	0x04
#define	PM_PWM_LUT_PAUSE_HI_EN	0x10
#define	PM_PWM_LUT_PAUSE_LO_EN	0x20

#define	PM_PWM_LUT_NO_TABLE	0x100
#define	PM8058_LPG_CTL_REGS		7

/* PWM LED ID */
#define	PM_PWM_LED_0		0
#define	PM_PWM_LED_1		1
#define	PM_PWM_LED_2		2
#define	PM_PWM_LED_KPD		3
#define	PM_PWM_LED_FLASH	4
#define	PM_PWM_LED_FLASH1	5

/* PWM LED configuration mode */
#define	PM_PWM_CONF_NONE	0x0
#define	PM_PWM_CONF_PWM1	0x1
#define	PM_PWM_CONF_PWM2	0x2
#define	PM_PWM_CONF_PWM3	0x3
#define	PM_PWM_CONF_DTEST1	0x4
#define	PM_PWM_CONF_DTEST2	0x5
#define	PM_PWM_CONF_DTEST3	0x6
#define	PM_PWM_CONF_DTEST4	0x7

struct pwm_device {
	int			pwm_id;		/* = bank/channel id */
	int			in_use;
	const char		*label;
	int			pwm_period;
	int			pwm_duty;
	u8			pwm_ctl[PM8058_LPG_CTL_REGS];
	int			irq;
	struct pm8058_pwm_chip	*chip;
};

struct pm8058_pwm_pdata {
	int 	(*config)(struct pwm_device *pwm, int ch, int on);
	int 	(*enable)(struct pwm_device *pwm, int ch, int on);
};

struct pw8058_pwm_config {
	int	pwm_size;	/* round up to 6 or 9 for 6/9-bit PWM SIZE */
	int	clk;
	int	pre_div;
	int	pre_div_exp;
	int	pwm_value;
	int	bypass_lut;

	/* LUT parameters when bypass_lut is 0 */
	int	lut_duty_ms;
	int	lut_lo_index;
	int	lut_hi_index;
	int	lut_pause_hi;
	int	lut_pause_lo;
	int	flags;
};

/*
 * pm8058_pwm_lut_config - change a PWM device configuration to use LUT
 *
 * @pwm: the PWM device
 * @period_us: period in micro second
 * @duty_pct: arrary of duty cycles in percent, like 20, 50.
 * @duty_time_ms: time for each duty cycle in millisecond
 * @start_idx: start index in lookup table from 0 to MAX-1
 * @idx_len: number of index
 * @pause_lo: pause time in millisecond at low index
 * @pause_hi: pause time in millisecond at high index
 * @flags: control flags
 *
 */
int pm8058_pwm_lut_config(struct pwm_device *pwm, int period_us,
			  int duty_pct[], int duty_time_ms, int start_idx,
			  int len, int pause_lo, int pause_hi, int flags);

/*
 * pm8058_pwm_lut_enable - control a PWM device to start/stop LUT ramp
 *
 * @pwm: the PWM device
 * @start: to start (1), or stop (0)
 */
int pm8058_pwm_lut_enable(struct pwm_device *pwm, int start);
/*
 * pwm_configure - change a PWM device configuration
 */
int pwm_configure(struct pwm_device *pwm, struct pw8058_pwm_config *pwm_conf);

int pm8058_pwm_set_dtest(struct pwm_device *pwm, int enable);

int pm8058_pwm_config_led(struct pwm_device *pwm, int id,
			  int mode, int max_current);

#if !defined(CONFIG_PMIC8058_PWM)
inline struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	return NULL;
}

inline void pwm_free(struct pwm_device *pwm)
{
}

inline int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	return 0;
}

inline int pwm_enable(struct pwm_device *pwm)
{
	return 0;
}

inline void pwm_disable(struct pwm_device *pwm)
{
}

inline int pwm_set_dtest(struct pwm_device *pwm, int enable)
{
	return 0;
}
#endif

#endif /* __PMIC8058_PWM_H__ */
