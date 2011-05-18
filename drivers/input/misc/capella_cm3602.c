/* drivers/input/misc/capella_cm3602.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Iliyan Malchev <malchev@google.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/capella_cm3602.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>

#define DPS(x...) printk(KERN_DEBUG "[PS][CM3602] " x)
#define IPS(x...) printk(KERN_INFO "[PS][CM3602] " x)
#define EPS(x...) printk(KERN_ERR "[PS][CM3602 ERROR] " x)

struct wake_lock proximity_wake_lock;

static struct capella_cm3602_data {
	struct input_dev *input_dev;
	struct capella_cm3602_platform_data *pdata;
	int enabled;
} the_data;

static int misc_opened;

static int capella_cm3602_report(struct capella_cm3602_data *data)
{
	int val = gpio_get_value(data->pdata->p_out);
	int value1, value2;
	int retry_limit = 10;
	int irq = data->pdata->irq;

	do {
		value1 = gpio_get_value(data->pdata->p_out);
		set_irq_type(irq, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(data->pdata->p_out);
	} while (value1 != value2 && retry_limit-- > 0);

	if (val < 0) {
		EPS("%s: gpio_get_value error %d\n", __func__, val);
		return val;
	}

	IPS("proximity %s\n", val ? "FAR" : "NEAR");

	/* 0 is close, 1 is far */
	input_report_abs(data->input_dev, ABS_DISTANCE, val);
	input_sync(data->input_dev);

	wake_lock_timeout(&proximity_wake_lock, 2*HZ);

	return val;
}

static irqreturn_t capella_cm3602_irq_handler(int irq, void *data)
{
	struct capella_cm3602_data *ip = data;
	capella_cm3602_report(ip);
	return IRQ_HANDLED;
}

static int capella_cm3602_enable(struct capella_cm3602_data *data)
{
	int rc;
	int irq = data->pdata->irq;

	IPS("%s\n", __func__);
	if (data->enabled) {
		DPS("%s: already enabled\n", __func__);
	return 0;
}

	/* dummy report */
	input_report_abs(data->input_dev, ABS_DISTANCE, -1);
	input_sync(data->input_dev);

	data->pdata->power(PS_PWR_ON, 1);

	rc = gpio_direction_output(data->pdata->p_en, 0);

	msleep(220);

	data->enabled = !rc;
	if (!rc)
		capella_cm3602_report(data);

	enable_irq(irq);
	rc = set_irq_wake(irq, 1);
	if (rc < 0)
		EPS("%s: failed to set irq %d as a wake interrupt\n",
			__func__, irq);

	return rc;
}

static int capella_cm3602_disable(struct capella_cm3602_data *data)
{
	int rc = -EIO;
	int irq = data->pdata->irq;

	IPS("%s\n", __func__);
	if (!data->enabled) {
		DPS("%s: already disabled\n", __func__);
	return 0;
}
	disable_irq(irq);
	rc = set_irq_wake(irq, 0);
	if (rc < 0)
		EPS("%s: failed to set irq %d as a non-wake interrupt\n",
			__func__, irq);

	rc = gpio_direction_output(data->pdata->p_en, 1);

	if (rc < 0)
		return rc;
	data->pdata->power(PS_PWR_ON, 0);
	data->enabled = 0;

	input_event(data->input_dev, EV_SYN, SYN_CONFIG, 0);
	return rc;
}

static int capella_cm3602_setup(struct capella_cm3602_data *ip)
{
	int rc = -EIO;
	struct capella_cm3602_platform_data *pdata = ip->pdata;
	int irq = pdata->irq;

	IPS("%s\n", __func__);

	if (pdata->p_out == 0 || pdata->p_en == 0) {
		EPS("%s: gpio == 0!!\n", __func__);
		rc = -1;
		goto done;
	}

	rc = gpio_request(pdata->p_out, "gpio_proximity_out");
	if (rc < 0) {
		EPS("%s: gpio %d request failed (%d)\n",
			__func__, pdata->p_out, rc);
		goto done;
	}

	rc = gpio_request(pdata->p_en, "gpio_proximity_en");
	if (rc < 0) {
		EPS("%s: gpio %d request failed (%d)\n",
			__func__, pdata->p_en, rc);
		goto fail_free_p_out;
	}

	rc = gpio_direction_input(pdata->p_out);
	if (rc < 0) {
		EPS("%s: failed to set gpio %d as input (%d)\n",
			__func__, pdata->p_out, rc);
		goto fail_free_p_en;
	}
	set_irq_flags(irq, IRQF_VALID | IRQF_NOAUTOEN);
	rc = request_irq(irq,
			capella_cm3602_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_TRIGGER_HIGH,
			"capella_cm3602",
			ip);
	if (rc < 0) {
		EPS("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			pdata->p_out, rc);
		goto fail_free_p_en;
	}

	goto done;

fail_free_p_en:
	gpio_free(pdata->p_en);
fail_free_p_out:
	gpio_free(pdata->p_out);
done:
	return rc;
}

static int capella_cm3602_open(struct inode *inode, struct file *file)
{
	/*DPS("%s\n", __func__);*/
	if (misc_opened)
		return -EBUSY;
	misc_opened = 1;
	return 0;
}

static int capella_cm3602_release(struct inode *inode, struct file *file)
{
	/*DPS("%s\n", __func__);*/
	misc_opened = 0;
	return capella_cm3602_disable(&the_data);
}

static long capella_cm3602_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	/*DPS("%s cmd %d\n", __func__, _IOC_NR(cmd));*/
	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val)
			return capella_cm3602_enable(&the_data);
		else
			return capella_cm3602_disable(&the_data);
		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(the_data.enabled, (unsigned long __user *)arg);
		break;
	default:
		DPS("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static struct file_operations capella_cm3602_fops = {
	.owner = THIS_MODULE,
	.open = capella_cm3602_open,
	.release = capella_cm3602_release,
	.unlocked_ioctl = capella_cm3602_ioctl
};

struct miscdevice capella_cm3602_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &capella_cm3602_fops
};

static int capella_cm3602_probe(struct platform_device *pdev)
{
	int rc = -EIO;
	struct input_dev *input_dev;
	struct capella_cm3602_data *ip;
	struct capella_cm3602_platform_data *pdata;

	IPS("%s: probe\n", __func__);

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		EPS("%s: missing pdata!\n", __func__);
		goto done;
	}
	if (!pdata->power) {
		EPS("%s: incomplete pdata!\n", __func__);
		goto done;
	}

	ip = &the_data;
	platform_set_drvdata(pdev, ip);

	/*DPS("%s: allocating input device\n", __func__);*/
	input_dev = input_allocate_device();
	if (!input_dev) {
		EPS("%s: could not allocate input device\n", __func__);
		rc = -ENOMEM;
		goto done;
	}
	ip->input_dev = input_dev;
	ip->pdata = pdata;
	input_set_drvdata(input_dev, ip);

	input_dev->name = "proximity";

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	/*DPS("%s: registering input device\n", __func__);*/
	rc = input_register_device(input_dev);
	if (rc < 0) {
		EPS("%s: could not register input device\n", __func__);
		goto err_free_input_device;
	}

	/*DPS("%s: registering misc device\n", __func__);*/
	rc = misc_register(&capella_cm3602_misc);
	if (rc < 0) {
		EPS("%s: could not register misc device\n", __func__);
		goto err_unregister_input_device;
	}

	wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");

	rc = capella_cm3602_setup(ip);
	if (!rc)
		goto done;

	misc_deregister(&capella_cm3602_misc);
err_unregister_input_device:
	input_unregister_device(input_dev);
err_free_input_device:
	input_free_device(input_dev);
done:
	return rc;
}

static struct platform_driver capella_cm3602_driver = {
	.probe = capella_cm3602_probe,
	.driver = {
		.name = CAPELLA_CM3602,
		.owner = THIS_MODULE
	},
};

static int __init capella_cm3602_init(void)
{
	return platform_driver_register(&capella_cm3602_driver);
}

device_initcall(capella_cm3602_init);
