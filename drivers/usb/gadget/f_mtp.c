/*
 * Gadget Function Driver for MTP
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <linux/types.h>
#include <linux/file.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb.h>
#include <linux/usb_usual.h>
#include <linux/usb/ch9.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/f_mtp.h>

#define BULK_BUFFER_SIZE           16384
#define INTR_BUFFER_SIZE           28

/* String IDs */
#define INTERFACE_STRING_INDEX	0

/* values for mtp_dev.state */
#define STATE_OFFLINE               0   /* initial state, disconnected */
#define STATE_READY                 1   /* ready for userspace calls */
#define STATE_BUSY                  2   /* processing userspace calls */
#define STATE_CANCELED              3   /* transaction canceled by host */
#define STATE_ERROR                 4   /* error from completion routine */

/* number of tx and rx requests to allocate */
#define TX_REQ_MAX 4
#define RX_REQ_MAX 2

/* IO Thread commands */
#define ANDROID_THREAD_QUIT				1
#define ANDROID_THREAD_SEND_FILE		2
#define ANDROID_THREAD_RECEIVE_FILE		3

/* ID for Microsoft MTP OS String */
#define MTP_OS_STRING_ID   0xEE

/* MTP class reqeusts */
#define MTP_REQ_CANCEL              0x64
#define MTP_REQ_GET_EXT_EVENT_DATA  0x65
#define MTP_REQ_RESET               0x66
#define MTP_REQ_GET_DEVICE_STATUS   0x67

/* constants for device status */
#define MTP_RESPONSE_OK             0x2001
#define MTP_RESPONSE_DEVICE_BUSY    0x2019

static const char shortname[] = "mtp_usb";

struct mtp_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	/* appear as MTP or PTP when enumerating */
	int interface_mode;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	struct usb_ep *ep_intr;

	int state;

	/* synchronize access to our device file */
	atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	wait_queue_head_t intr_wq;
	struct usb_request *rx_req[RX_REQ_MAX];
	struct usb_request *intr_req;
	int rx_done;

	/* synchronize access to interrupt endpoint */
	struct mutex intr_mutex;
	/* true if interrupt endpoint is busy */
	int intr_busy;

	/* for our file IO thread */
	struct task_struct			*thread;
	/* current command for IO thread (or zero for none) */
	int							thread_command;
	struct file 				*thread_file;
	loff_t						thread_file_offset;
	size_t						thread_file_length;
	/* used to wait for thread to complete current command */
	struct completion			thread_wait;
	/* result from current command */
	int							thread_result;
};

static struct usb_interface_descriptor mtp_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass     = USB_SUBCLASS_VENDOR_SPEC,
	.bInterfaceProtocol     = 0,
};

static struct usb_interface_descriptor ptp_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_STILL_IMAGE,
	.bInterfaceSubClass     = 1,
	.bInterfaceProtocol     = 1,
};

static struct usb_endpoint_descriptor mtp_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor mtp_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor mtp_intr_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize         = __constant_cpu_to_le16(INTR_BUFFER_SIZE),
	.bInterval              = 6,
};

static struct usb_descriptor_header *fs_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *hs_mtp_descs[] = {
	(struct usb_descriptor_header *) &mtp_interface_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *fs_ptp_descs[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_fullspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_descriptor_header *hs_ptp_descs[] = {
	(struct usb_descriptor_header *) &ptp_interface_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_in_desc,
	(struct usb_descriptor_header *) &mtp_highspeed_out_desc,
	(struct usb_descriptor_header *) &mtp_intr_desc,
	NULL,
};

static struct usb_string mtp_string_defs[] = {
	/* Naming interface "MTP" so libmtp will recognize us */
	[INTERFACE_STRING_INDEX].s	= "MTP",
	{  },	/* end of list */
};

static struct usb_gadget_strings mtp_string_table = {
	.language		= 0x0409,	/* en-US */
	.strings		= mtp_string_defs,
};

static struct usb_gadget_strings *mtp_strings[] = {
	&mtp_string_table,
	NULL,
};

/* Microsoft MTP OS String */
static u8 mtp_os_string[] = {
	18, /* sizeof(mtp_os_string) */
	USB_DT_STRING,
	/* Signature field: "MSFT100" */
	'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
	/* vendor code */
	1,
	/* padding */
	0
};

/* Microsoft Extended Configuration Descriptor Header Section */
struct mtp_ext_config_desc_header {
	__le32	dwLength;
	__u16	bcdVersion;
	__le16	wIndex;
	__u8	bCount;
	__u8	reserved[7];
};

/* Microsoft Extended Configuration Descriptor Function Section */
struct mtp_ext_config_desc_function {
	__u8	bFirstInterfaceNumber;
	__u8	bInterfaceCount;
	__u8	compatibleID[8];
	__u8	subCompatibleID[8];
	__u8	reserved[6];
};

/* MTP Extended Configuration Descriptor */
struct {
	struct mtp_ext_config_desc_header	header;
	struct mtp_ext_config_desc_function    function;
} mtp_ext_config_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(sizeof(mtp_ext_config_desc)),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(4),
		.bCount = __constant_cpu_to_le16(1),
	},
	.function = {
		.bFirstInterfaceNumber = 0,
		.bInterfaceCount = 1,
		.compatibleID = { 'M', 'T', 'P' },
	},
};

struct mtp_device_status {
	__le16	wLength;
	__le16	wCode;
};

/* temporary variable used between mtp_open() and mtp_gadget_bind() */
static struct mtp_dev *_mtp_dev;

static inline struct mtp_dev *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct mtp_dev, function);
}

static struct usb_request *mtp_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void mtp_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
static void req_put(struct mtp_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct mtp_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void mtp_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	if (req->status != 0)
		dev->state = STATE_ERROR;

	req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static void mtp_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	dev->rx_done = 1;
	if (req->status != 0)
		dev->state = STATE_ERROR;

	wake_up(&dev->read_wq);
}

static void mtp_complete_intr(struct usb_ep *ep, struct usb_request *req)
{
	struct mtp_dev *dev = _mtp_dev;

	DBG(dev->cdev, "mtp_complete_intr status: %d actual: %d\n", req->status, req->actual);
	dev->intr_busy = 0;
	if (req->status != 0)
		dev->state = STATE_ERROR;

	wake_up(&dev->intr_wq);
}

static int __init create_bulk_endpoints(struct mtp_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc,
				struct usb_endpoint_descriptor *intr_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, intr_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_intr failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for mtp ep_intr got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_intr = ep;

	/* now allocate requests for our endpoints */
	for (i = 0; i < TX_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_in, BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = mtp_complete_in;
		req_put(dev, &dev->tx_idle, req);
	}
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = mtp_request_new(dev->ep_out, BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = mtp_complete_out;
		dev->rx_req[i] = req;
	}
	req = mtp_request_new(dev->ep_intr, INTR_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = mtp_complete_intr;
	dev->intr_req = req;

	return 0;

fail:
	printk(KERN_ERR "mtp_bind() could not allocate requests\n");
	return -1;
}

static ssize_t mtp_read(struct file *fp, char __user *buf,
	size_t count, loff_t *pos)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int r = count, xfer;
	int ret = 0;

	DBG(cdev, "mtp_read(%d)\n", count);

	if (count > BULK_BUFFER_SIZE)
		return -EINVAL;

	/* we will block until we're online */
	DBG(cdev, "mtp_read: waiting for online state\n");
	ret = wait_event_interruptible(dev->read_wq,
		dev->state != STATE_OFFLINE);
	if (ret < 0) {
		r = ret;
		goto done;
	}
	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED) {
		/* report cancelation to userspace */
		dev->state = STATE_READY;
		spin_unlock_irq(&dev->lock);
		return -ECANCELED;
	}
	dev->state = STATE_BUSY;
	spin_unlock_irq(&dev->lock);

requeue_req:
	/* queue a request */
	req = dev->rx_req[0];
	req->length = count;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_KERNEL);
	if (ret < 0) {
		r = -EIO;
		goto done;
	} else {
		DBG(cdev, "rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	if (ret < 0) {
		r = ret;
		goto done;
	}
	if (dev->state == STATE_BUSY) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		DBG(cdev, "rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		r = xfer;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
	} else
		r = -EIO;

done:
	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED)
		r = -ECANCELED;
	else if (dev->state != STATE_OFFLINE)
		dev->state = STATE_READY;
	spin_unlock_irq(&dev->lock);

	DBG(cdev, "mtp_read returning %d\n", r);
	return r;
}

static ssize_t mtp_write(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos)
{
	struct mtp_dev *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG(cdev, "mtp_write(%d)\n", count);

	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED) {
		/* report cancelation to userspace */
		dev->state = STATE_READY;
		spin_unlock_irq(&dev->lock);
		return -ECANCELED;
	}
	if (dev->state == STATE_OFFLINE) {
		spin_unlock_irq(&dev->lock);
		return -ENODEV;
	}
	dev->state = STATE_BUSY;
	spin_unlock_irq(&dev->lock);

	while (count > 0) {
		if (dev->state != STATE_BUSY) {
			DBG(cdev, "mtp_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			((req = req_get(dev, &dev->tx_idle))
				|| dev->state != STATE_BUSY));
		if (!req) {
			r = ret;
			break;
		}

		if (count > BULK_BUFFER_SIZE)
			xfer = BULK_BUFFER_SIZE;
		else
			xfer = count;
		if (copy_from_user(req->buf, buf, xfer)) {
			r = -EFAULT;
			break;
		}

		req->length = xfer;
		ret = usb_ep_queue(dev->ep_in, req, GFP_KERNEL);
		if (ret < 0) {
			DBG(cdev, "mtp_write: xfer error %d\n", ret);
			r = -EIO;
			break;
		}

		buf += xfer;
		count -= xfer;

		/* zero this so we don't try to free it on error exit */
		req = 0;
	}

	if (req)
		req_put(dev, &dev->tx_idle, req);

	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED)
		r = -ECANCELED;
	else if (dev->state != STATE_OFFLINE)
		dev->state = STATE_READY;
	spin_unlock_irq(&dev->lock);

	DBG(cdev, "mtp_write returning %d\n", r);
	return r;
}

static int mtp_send_file(struct mtp_dev *dev, struct file *filp,
	loff_t offset, size_t count)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer, ret;

	DBG(cdev, "mtp_send_file(%lld %d)\n", offset, count);

	while (count > 0) {
		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
			(req = req_get(dev, &dev->tx_idle))
			|| dev->state != STATE_BUSY);
		if (!req) {
			r = ret;
			break;
		}

		if (count > BULK_BUFFER_SIZE)
			xfer = BULK_BUFFER_SIZE;
		else
			xfer = count;
		ret = vfs_read(filp, req->buf, xfer, &offset);
		if (ret < 0) {
			r = ret;
			break;
		}
		xfer = ret;

		req->length = xfer;
		ret = usb_ep_queue(dev->ep_in, req, GFP_KERNEL);
		if (ret < 0) {
			DBG(cdev, "mtp_write: xfer error %d\n", ret);
			dev->state = STATE_ERROR;
			r = -EIO;
			break;
		}

		count -= xfer;

		/* zero this so we don't try to free it on error exit */
		req = 0;
	}

	if (req)
		req_put(dev, &dev->tx_idle, req);

	DBG(cdev, "mtp_write returning %d\n", r);
	return r;
}

static int mtp_receive_file(struct mtp_dev *dev, struct file *filp,
	loff_t offset, size_t count)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *read_req = NULL, *write_req = NULL;
	int r = count;
	int ret;
	int cur_buf = 0;

	DBG(cdev, "mtp_receive_file(%d)\n", count);

	while (count > 0 || write_req) {
		if (count > 0) {
			/* queue a request */
			read_req = dev->rx_req[cur_buf];
			cur_buf = (cur_buf + 1) % RX_REQ_MAX;

			read_req->length = (count > BULK_BUFFER_SIZE
					? BULK_BUFFER_SIZE : count);
			dev->rx_done = 0;
			ret = usb_ep_queue(dev->ep_out, read_req, GFP_KERNEL);
			if (ret < 0) {
				r = -EIO;
				dev->state = STATE_ERROR;
				break;
			}
			count -= ret;
		}

		if (write_req) {
			DBG(cdev, "rx %p %d\n", write_req, write_req->actual);
			ret = vfs_write(filp, write_req->buf, write_req->actual,
				&offset);
			DBG(cdev, "vfs_write %d\n", ret);
			if (ret != write_req->actual) {
				r = -EIO;
				dev->state = STATE_ERROR;
				break;
			}
			write_req = NULL;
		}

		if (read_req) {
			/* wait for our last read to complete */
			ret = wait_event_interruptible(dev->read_wq,
				dev->rx_done || dev->state != STATE_BUSY);
			if (ret < 0 || dev->state != STATE_BUSY) {
				r = ret;
				break;
			}
			count -= read_req->actual;
			write_req = read_req;
			read_req = NULL;
		}
	}

	DBG(cdev, "mtp_read returning %d\n", r);
	return r;
}

/* Kernel thread for handling file IO operations */
static int mtp_thread(void *data)
{
	struct mtp_dev *dev = (struct mtp_dev *)data;
	struct usb_composite_dev *cdev = dev->cdev;
	int flags;

	DBG(cdev, "mtp_thread started\n");

	while (1) {
		/* wait for a command */
		while (1) {
			try_to_freeze();
			set_current_state(TASK_INTERRUPTIBLE);
			if (dev->thread_command != 0)
				break;
			schedule();
		}
		__set_current_state(TASK_RUNNING);

		if (dev->thread_command == ANDROID_THREAD_QUIT) {
			DBG(cdev, "ANDROID_THREAD_QUIT\n");
			dev->thread_result = 0;
			goto done;
		}

		if (dev->thread_command == ANDROID_THREAD_SEND_FILE)
			flags = O_RDONLY | O_LARGEFILE;
		else
			flags = O_WRONLY | O_LARGEFILE | O_CREAT;

		if (dev->thread_command == ANDROID_THREAD_SEND_FILE) {
			dev->thread_result = mtp_send_file(dev,
				dev->thread_file,
				dev->thread_file_offset,
				dev->thread_file_length);
		} else {
			dev->thread_result = mtp_receive_file(dev,
				dev->thread_file,
				dev->thread_file_offset,
				dev->thread_file_length);
		}

		if (dev->thread_file) {
			fput(dev->thread_file);
			dev->thread_file = NULL;
		}
		dev->thread_command = 0;
		complete(&dev->thread_wait);
	}

done:
	DBG(cdev, "android_thread done\n");
	complete_and_exit(&dev->thread_wait, 0);
}

static int mtp_send_event(struct mtp_dev *dev, struct mtp_event *event)
{
	struct usb_request *req;
	int ret;
	int length = event->length;

	DBG(dev->cdev, "mtp_send_event(%d)\n", event->length);

	if (length < 0 || length > INTR_BUFFER_SIZE)
		return -EINVAL;

	mutex_lock(&dev->intr_mutex);

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->intr_wq, !dev->intr_busy || dev->state == STATE_OFFLINE);
	if (ret < 0)
		goto done;
	if (dev->state == STATE_OFFLINE) {
		ret = -ENODEV;
		goto done;
	}
	req = dev->intr_req;
	if (copy_from_user(req->buf, (void __user *)event->data, length)) {
		ret = -EFAULT;
		goto done;
	}
	req->length = length;
	dev->intr_busy = 1;
	ret = usb_ep_queue(dev->ep_intr, req, GFP_KERNEL);
	if (ret)
		dev->intr_busy = 0;

done:
	mutex_unlock(&dev->intr_mutex);
	return ret;
}

static long mtp_ioctl(struct file *fp, unsigned code, unsigned long value)
{
	struct mtp_dev *dev = fp->private_data;
	struct file *filp = NULL;
	int ret = -EINVAL;

	switch (code) {
	case MTP_SEND_FILE:
	case MTP_RECEIVE_FILE:
	{
		struct mtp_file_range	mfr;

		spin_lock_irq(&dev->lock);
		if (dev->state == STATE_CANCELED) {
			/* report cancelation to userspace */
			dev->state = STATE_READY;
			spin_unlock_irq(&dev->lock);
			return -ECANCELED;
		}
		if (dev->state == STATE_OFFLINE) {
			spin_unlock_irq(&dev->lock);
			return -ENODEV;
		}
		dev->state = STATE_BUSY;
		spin_unlock_irq(&dev->lock);

		if (copy_from_user(&mfr, (void __user *)value, sizeof(mfr))) {
			ret = -EFAULT;
			goto fail;
		}
		filp = fget(mfr.fd);
		if (!filp) {
			ret = -EBADF;
			goto fail;
		}

		dev->thread_file = filp;
		dev->thread_file_offset = mfr.offset;
		dev->thread_file_length = mfr.length;

		if (code == MTP_SEND_FILE)
			dev->thread_command = ANDROID_THREAD_SEND_FILE;
		else
			dev->thread_command = ANDROID_THREAD_RECEIVE_FILE;

		/* wake up the thread */
		init_completion(&dev->thread_wait);
		wake_up_process(dev->thread);

		/* wait for the thread to complete the command */
		wait_for_completion(&dev->thread_wait);
		ret = dev->thread_result;
		DBG(dev->cdev, "thread returned %d\n", ret);
		break;
	}
	case MTP_SET_INTERFACE_MODE:
		if (value == MTP_INTERFACE_MODE_MTP ||
			value == MTP_INTERFACE_MODE_PTP) {
			dev->interface_mode = value;
			if (value == MTP_INTERFACE_MODE_PTP) {
				dev->function.descriptors = fs_ptp_descs;
				dev->function.hs_descriptors = hs_ptp_descs;
			} else {
				dev->function.descriptors = fs_mtp_descs;
				dev->function.hs_descriptors = hs_mtp_descs;
			}
			ret = 0;
		}
		break;
	case MTP_SEND_EVENT:
	{
		struct mtp_event	event;
		/* return here so we don't change dev->state below,
		 * which would interfere with bulk transfer state.
		 */
		if (copy_from_user(&event, (void __user *)value, sizeof(event)))
			return -EFAULT;
		else
			return mtp_send_event(dev, &event);
	}
	}

fail:
	if (filp)
		fput(filp);
	spin_lock_irq(&dev->lock);
	if (dev->state == STATE_CANCELED)
		ret = -ECANCELED;
	else if (dev->state != STATE_OFFLINE)
		dev->state = STATE_READY;
	spin_unlock_irq(&dev->lock);
	DBG(dev->cdev, "ioctl returning %d\n", ret);
	return ret;
}

static int mtp_open(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "mtp_open\n");
	if (_lock(&_mtp_dev->open_excl))
		return -EBUSY;

	_mtp_dev->thread = kthread_create(mtp_thread, _mtp_dev, "f_mtp");
	if (IS_ERR(_mtp_dev->thread))
		return -ENOMEM;

	/* clear any error condition */
	if (_mtp_dev->state != STATE_OFFLINE)
		_mtp_dev->state = STATE_READY;

	fp->private_data = _mtp_dev;
	return 0;
}

static int mtp_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "mtp_release\n");

	/* tell the thread to quit */
	if (_mtp_dev->thread) {
		_mtp_dev->thread_command = ANDROID_THREAD_QUIT;
		init_completion(&_mtp_dev->thread_wait);
		wake_up_process(_mtp_dev->thread);
		wait_for_completion(&_mtp_dev->thread_wait);
	}

	_unlock(&_mtp_dev->open_excl);
	return 0;
}

/* file operations for /dev/mtp_usb */
static const struct file_operations mtp_fops = {
	.owner = THIS_MODULE,
	.read = mtp_read,
	.write = mtp_write,
	.unlocked_ioctl = mtp_ioctl,
	.open = mtp_open,
	.release = mtp_release,
};

static struct miscdevice mtp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = shortname,
	.fops = &mtp_fops,
};

static int
mtp_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct mtp_dev	*dev = func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG(cdev, "mtp_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	mtp_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(dev, &mtp_fullspeed_in_desc,
			&mtp_fullspeed_out_desc, &mtp_intr_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		mtp_highspeed_in_desc.bEndpointAddress =
			mtp_fullspeed_in_desc.bEndpointAddress;
		mtp_highspeed_out_desc.bEndpointAddress =
			mtp_fullspeed_out_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
mtp_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_request *req;
	int i;

	spin_lock_irq(&dev->lock);
	while ((req = req_get(dev, &dev->tx_idle)))
		mtp_request_free(req, dev->ep_in);
	for (i = 0; i < RX_REQ_MAX; i++)
		mtp_request_free(dev->rx_req[i], dev->ep_out);
	mtp_request_free(dev->intr_req, dev->ep_intr);
	dev->state = STATE_OFFLINE;
	spin_unlock_irq(&dev->lock);
	wake_up(&dev->intr_wq);

	misc_deregister(&mtp_device);
	kfree(_mtp_dev);
	_mtp_dev = NULL;
}

static int mtp_function_setup(struct usb_function *f,
					const struct usb_ctrlrequest *ctrl)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_composite_dev *cdev = dev->cdev;
	int	value = -EOPNOTSUPP;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);
	unsigned long	flags;

	/* do nothing if we are disabled */
	if (dev->function.disabled)
		return value;

	VDBG(cdev, "mtp_function_setup "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);

	/* Handle MTP OS string */
	if (dev->interface_mode == MTP_INTERFACE_MODE_MTP
			&& ctrl->bRequestType ==
			(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE)
			&& ctrl->bRequest == USB_REQ_GET_DESCRIPTOR
			&& (w_value >> 8) == USB_DT_STRING
			&& (w_value & 0xFF) == MTP_OS_STRING_ID) {
		value = (w_length < sizeof(mtp_os_string)
				? w_length : sizeof(mtp_os_string));
		memcpy(cdev->req->buf, mtp_os_string, value);
		/* return here since composite.c will send for us */
		return value;
	}
	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		/* Handle MTP OS descriptor */
		DBG(cdev, "vendor request: %d index: %d value: %d length: %d\n",
			ctrl->bRequest, w_index, w_value, w_length);

		if (dev->interface_mode == MTP_INTERFACE_MODE_MTP
				&& ctrl->bRequest == 1
				&& (ctrl->bRequestType & USB_DIR_IN)
				&& (w_index == 4 || w_index == 5)) {
			value = (w_length < sizeof(mtp_ext_config_desc) ?
					w_length : sizeof(mtp_ext_config_desc));
			memcpy(cdev->req->buf, &mtp_ext_config_desc, value);
		}
	}
	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		DBG(cdev, "class request: %d index: %d value: %d length: %d\n",
			ctrl->bRequest, w_index, w_value, w_length);

		if (ctrl->bRequest == MTP_REQ_CANCEL && w_index == 0
				&& w_value == 0) {
			DBG(cdev, "MTP_REQ_CANCEL\n");

			spin_lock_irqsave(&dev->lock, flags);
			if (dev->state == STATE_BUSY) {
				dev->state = STATE_CANCELED;
				wake_up(&dev->read_wq);
				wake_up(&dev->write_wq);
			}
			spin_unlock_irqrestore(&dev->lock, flags);

			/* We need to queue a request to read the remaining
			 *  bytes, but we don't actually need to look at
			 * the contents.
			 */
			value = w_length;
		} else if (ctrl->bRequest == MTP_REQ_GET_DEVICE_STATUS
				&& w_index == 0 && w_value == 0) {
			struct mtp_device_status *status = cdev->req->buf;
			status->wLength =
				__constant_cpu_to_le16(sizeof(*status));

			DBG(cdev, "MTP_REQ_GET_DEVICE_STATUS\n");
			spin_lock_irqsave(&dev->lock, flags);
			/* device status is "busy" until we report
			 * the cancelation to userspace
			 */
			if (dev->state == STATE_BUSY
					|| dev->state == STATE_CANCELED)
				status->wCode =
					__cpu_to_le16(MTP_RESPONSE_DEVICE_BUSY);
			else
				status->wCode =
					__cpu_to_le16(MTP_RESPONSE_OK);
				spin_unlock_irqrestore(&dev->lock, flags);
			value = sizeof(*status);
		}
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		int rc;
		cdev->req->zero = value < w_length;
		cdev->req->length = value;
		rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (rc < 0)
			ERROR(cdev, "%s setup response queue error\n", __func__);
	}

	if (value == -EOPNOTSUPP)
		VDBG(cdev,
			"unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	return value;
}

static int mtp_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "mtp_function_set_alt intf: %d alt: %d\n", intf, alt);
	ret = usb_ep_enable(dev->ep_in,
			ep_choose(cdev->gadget,
				&mtp_highspeed_in_desc,
				&mtp_fullspeed_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_out,
			ep_choose(cdev->gadget,
				&mtp_highspeed_out_desc,
				&mtp_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_intr, &mtp_intr_desc);
	if (ret) {
		usb_ep_disable(dev->ep_out);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->state = STATE_READY;

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void mtp_function_disable(struct usb_function *f)
{
	struct mtp_dev	*dev = func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "mtp_function_disable\n");
	dev->state = STATE_OFFLINE;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);
	usb_ep_disable(dev->ep_intr);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	wake_up(&dev->intr_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int mtp_bind_config(struct usb_configuration *c)
{
	struct mtp_dev *dev;
	int ret;

	printk(KERN_INFO "mtp_bind_config\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* allocate a string ID for our interface */
	if (mtp_string_defs[INTERFACE_STRING_INDEX].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		mtp_string_defs[INTERFACE_STRING_INDEX].id = ret;
		mtp_interface_desc.iInterface = ret;
	}

	spin_lock_init(&dev->lock);
	init_completion(&dev->thread_wait);
	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);
	init_waitqueue_head(&dev->intr_wq);
	atomic_set(&dev->open_excl, 0);
	INIT_LIST_HEAD(&dev->tx_idle);
	mutex_init(&dev->intr_mutex);

	dev->cdev = c->cdev;
	dev->function.name = "mtp";
	dev->function.strings = mtp_strings,
	dev->function.descriptors = fs_mtp_descs;
	dev->function.hs_descriptors = hs_mtp_descs;
	dev->function.bind = mtp_function_bind;
	dev->function.unbind = mtp_function_unbind;
	dev->function.setup = mtp_function_setup;
	dev->function.set_alt = mtp_function_set_alt;
	dev->function.disable = mtp_function_disable;

	/* MTP mode by default */
	dev->interface_mode = MTP_INTERFACE_MODE_MTP;

	/* _mtp_dev must be set before calling usb_gadget_register_driver */
	_mtp_dev = dev;

	ret = misc_register(&mtp_device);
	if (ret)
		goto err1;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err2;

	return 0;

err2:
	misc_deregister(&mtp_device);
err1:
	kfree(dev);
	printk(KERN_ERR "mtp gadget driver failed to initialize\n");
	return ret;
}

static struct android_usb_function mtp_function = {
	.name = "mtp",
	.bind_config = mtp_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "f_mtp init\n");
	android_register_function(&mtp_function);
	return 0;
}
module_init(init);
