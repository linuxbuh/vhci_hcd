/*
 * usb-vhci-iocifc.c -- User-mode IOCTL-interface for
 *                         VHCI USB host controller driver.
 *
 * Copyright (C) 2007-2008 Conemis AG Karlsruhe Germany
 * Copyright (C) 2007-2010 Michael Singer <michael@a-singer.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/fs.h>

#include "usb-vhci-hcd.h"

#include <asm/atomic.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>

#define DRIVER_NAME "usb_vhci_iocifc"
#define DRIVER_DESC "User-mode IOCTL-interface for USB VHCI"
#define DRIVER_VERSION USB_VHCI_IOCIFC_VERSION " (" USB_VHCI_IOCIFC_DATE ")"

#ifdef vhci_printk
#	undef vhci_printk
#endif
#define vhci_printk(level, fmt, args...) \
	printk(level DRIVER_NAME ": " fmt, ## args)
#ifdef vhci_dbg
#	undef vhci_dbg
#endif
#ifdef DEBUG
#	warning DEBUG is defined
#	define vhci_dbg(fmt, args...) \
		if(debug_output) vhci_printk(KERN_DEBUG, fmt, ## args)
#else
#	define vhci_dbg(fmt, args...) do {} while(0)
#endif
#ifdef trace_function
#	undef trace_function
#endif
#ifdef DEBUG
#	define trace_function(dev) \
		if(debug_output) dev_dbg((dev), "%s%s\n", \
			in_interrupt() ? "IN_INTERRUPT: " : "", __FUNCTION__)
#else
#	define trace_function(dev) do {} while(0)
#endif

static const char driver_name[] = DRIVER_NAME;
static const char driver_desc[] = DRIVER_DESC;
#ifdef DEBUG
static unsigned int debug_output = 0;
#endif

MODULE_DESCRIPTION(DRIVER_DESC " driver");
MODULE_AUTHOR("Michael Singer <michael@a-singer.de>");
MODULE_LICENSE("GPL");

struct vhci_ifc_priv
{
	struct file *file;
	wait_queue_head_t work_event;
	u8 port_sched_offset;

#ifdef DEBUG
	u16 debug_magic;
#endif
};

static inline struct vhci_ifc_priv *vhcidev_to_ifcp(struct usb_vhci_device *vdev)
{
	return (struct vhci_ifc_priv *)vhcidev_to_ifc(vdev);
}

static inline struct vhci_ifc_priv *vhcihcd_to_ifcp(struct usb_vhci_hcd *vhc)
{
	return vhcidev_to_ifcp(vhcihcd_to_vhcidev(vhc));
}

static inline struct usb_vhci_device *file_to_vhcidev(struct file *file)
{
	return (struct usb_vhci_device *)file->private_data;
}

static inline struct usb_vhci_hcd *file_to_vhcihcd(struct file *file)
{
	return vhcidev_to_vhcihcd(file_to_vhcidev(file));
}

static inline struct vhci_ifc_priv *file_to_ifcp(struct file *file)
{
	return vhcidev_to_ifcp(file_to_vhcidev(file));
}

static int init_ifc_priv(void *context, void *ifc_priv)
{
	struct vhci_ifc_priv *ifcp;

	ifcp = ifc_priv;

#ifdef DEBUG
	if(ifcp->debug_magic == 0x55aa)
		vhci_printk(KERN_WARNING, "init_ifc_priv _maybe_ called twice\n");
#endif

	ifcp->file = context;
	init_waitqueue_head(&ifcp->work_event);
	ifcp->port_sched_offset = 0;

#ifdef DEBUG
	ifcp->debug_magic = 0x55aa;
#endif

	return 0;
}

#ifdef DEBUG
static void destroy_ifc_priv(void *ifc_priv)
{
	struct vhci_ifc_priv *ifcp = ifc_priv;

	if(ifcp->debug_magic == 0xaa55)
		vhci_printk(KERN_WARNING, "destroy_ifc_priv called twice\n");
	else if(ifcp->debug_magic != 0x55aa)
		vhci_printk(KERN_WARNING, "destroy_ifc_priv called, but ifc_priv was not initialized\n");

	ifcp->debug_magic = 0xaa55;
}
#endif

static void trigger_work_event(struct usb_vhci_device *vdev)
{
	wake_up_interruptible(&vhcidev_to_ifcp(vdev)->work_event);
}

static struct usb_vhci_ifc vhci_ioc_ifc = {
	.ifc_desc      = "USB VHCI user-mode IOCTL-interface",
	.owner         = THIS_MODULE,
	.ifc_priv_size = sizeof(struct vhci_ifc_priv),

#ifdef DEBUG
	.destroy = destroy_ifc_priv,
#endif

	.init   = init_ifc_priv,
	.wakeup = trigger_work_event
};

static int device_open(struct inode *inode, struct file *file)
{
	vhci_dbg("%s(inode=%p, file=%p)\n", __FUNCTION__, inode, file);

	if(unlikely(file->private_data != NULL))
	{
		vhci_printk(KERN_ERR, "file->private_data != NULL\n");
		return -EINVAL;
	}

	try_module_get(THIS_MODULE);
	return 0;
}

// called in device_ioctl only
static int ioc_register(struct file *file, struct usb_vhci_ioc_register __user *arg)
{
	const char *dname;
	int retval, i, usbbusnum;
	struct usb_vhci_device *vdev;
	u8 pc;

	vhci_dbg("cmd=USB_VHCI_HCD_IOCREGISTER\n");

	if(unlikely(file->private_data))
	{
		vhci_printk(KERN_ERR, "file->private_data != NULL (USB_VHCI_HCD_IOCREGISTER already done?)\n");
		return -EPROTO;
	}

	__get_user(pc, &arg->port_count);
	retval = usb_vhci_hcd_register(&vhci_ioc_ifc, file, pc, &vdev);
	if(unlikely(retval < 0)) return retval;
	file->private_data = vdev;

	// copy id to user space
	__put_user(usb_vhci_dev_id(vdev), &arg->id);

	// copy bus-id to user space
	dname = usb_vhci_dev_name(vdev);
	i = strlen(dname);
	i = (i < sizeof(arg->bus_id)) ? i : sizeof(arg->bus_id) - 1;
	if(copy_to_user(arg->bus_id, dname, i))
	{
		vhci_printk(KERN_WARNING, "Failed to copy bus_id to userspace.\n");
		__put_user('\0', arg->bus_id);
	}
	// make sure the last character is null
	__put_user('\0', arg->bus_id + i);

	usbbusnum = usb_vhci_dev_busnum(vdev);
	vhci_printk(KERN_INFO, "Usb bus #%d\n", usbbusnum);
	__put_user(usbbusnum, &arg->usb_busnum);

	return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
	struct usb_vhci_device *vdev;

	vhci_dbg("%s(inode=%p, file=%p)\n", __FUNCTION__, inode, file);

	vdev = file->private_data;
	file->private_data = NULL;

	if(likely(vdev))
		usb_vhci_hcd_unregister(vdev);
	else
		vhci_dbg("was not configured\n");

	module_put(THIS_MODULE);
	return 0;
}

static ssize_t device_read(struct file *file,
                           char __user *buffer,
                           size_t length,
                           loff_t *offset)
{
	vhci_dbg("%s(file=%p)\n", __FUNCTION__, file);
	return -ENODEV;
}

static ssize_t device_write(struct file *file,
                            const char __user *buffer,
                            size_t length,
                            loff_t *offset)
{
	vhci_dbg("%s(file=%p)\n", __FUNCTION__, file);
	return -ENODEV;
}

// called in device_ioctl only
static int ioc_port_stat(struct usb_vhci_device *vdev, struct usb_vhci_ioc_port_stat __user *arg)
{
	u16 status, change;
	u8 index;

#ifdef DEBUG
	if(debug_output) dev_dbg(vhcidev_to_dev(vdev), "cmd=USB_VHCI_HCD_IOCPORTSTAT\n");
#endif

	__get_user(status, &arg->status);
	__get_user(change, &arg->change);
	__get_user(index, &arg->index);
	return usb_vhci_apply_port_stat(vhcidev_to_vhcihcd(vdev), status, change, index);
}

static inline u8 conv_urb_type(u8 type)
{
	switch(type & 0x3)
	{
	case PIPE_ISOCHRONOUS: return USB_VHCI_URB_TYPE_ISO;
	case PIPE_INTERRUPT:   return USB_VHCI_URB_TYPE_INT;
	case PIPE_BULK:        return USB_VHCI_URB_TYPE_BULK;
	default:               return USB_VHCI_URB_TYPE_CONTROL;
	}
}

static inline u16 conv_urb_flags(unsigned int flags)
{
	return ((flags & URB_SHORT_NOT_OK) ? USB_VHCI_URB_FLAGS_SHORT_NOT_OK : 0) |
	       ((flags & URB_ISO_ASAP)     ? USB_VHCI_URB_FLAGS_ISO_ASAP     : 0) |
	       ((flags & URB_ZERO_PACKET)  ? USB_VHCI_URB_FLAGS_ZERO_PACKET  : 0);
}

#ifdef DEBUG
#include "usb-vhci-dump-urb.c"
#else
static inline void dump_urb(struct urb *urb) {/* do nothing */}
#endif

// called in device_ioctl only
static int ioc_fetch_work(struct usb_vhci_hcd *vhc, struct usb_vhci_ioc_work __user *arg, s16 timeout)
{
#ifdef DEBUG
	struct device *dev = vhcihcd_to_dev(vhc);
#endif
	struct usb_vhci_urb_priv *urbp;
	struct vhci_ifc_priv *ifcp;
	struct usb_vhci_port port_stat;
	struct usb_vhci_ioc_urb urb;
	u64 handle;
	unsigned long flags;
	long wret;
	u8 _port, port;

#ifdef DEBUG
	// Floods the logs
	//if(debug_output) dev_dbg(dev, "cmd=USB_VHCI_HCD_IOCFETCHWORK\n");
#endif

	ifcp = vhcihcd_to_ifcp(vhc);

	if(timeout)
	{
		if(timeout > 1000)
			timeout = 1000;
		if(timeout > 0)
			wret = wait_event_interruptible_timeout(ifcp->work_event, usb_vhci_hcd_has_work(vhc), msecs_to_jiffies(timeout));
		else
			wret = wait_event_interruptible(ifcp->work_event, usb_vhci_hcd_has_work(vhc));
		if(unlikely(wret < 0))
		{
			if(likely(wret == -ERESTARTSYS))
				return -EINTR;
			return wret;
		}
		else if(!wret)
			return -ETIMEDOUT;
	}
	else
	{
		if(!usb_vhci_hcd_has_work(vhc))
			return -ETIMEDOUT;
	}

	spin_lock_irqsave(&vhc->lock, flags);
	if(!list_empty(&vhc->urbp_list_cancel))
	{
		urbp = list_entry(vhc->urbp_list_cancel.next, struct usb_vhci_urb_priv, urbp_list);
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "cmd=USB_VHCI_HCD_IOCFETCHWORK [work=CANCEL_URB handle=0x%016llx]\n", (u64)(unsigned long)urbp->urb);
#endif
		handle = (u64)(unsigned long)urbp->urb;
		list_move_tail(&urbp->urbp_list, &vhc->urbp_list_canceling);
		spin_unlock_irqrestore(&vhc->lock, flags);
		__put_user(USB_VHCI_WORK_TYPE_CANCEL_URB, &arg->type);
		__put_user(handle, &arg->handle);
		return 0;
	}

	if(vhc->port_update)
	{
		if(ifcp->port_sched_offset >= vhc->port_count)
			ifcp->port_sched_offset = 0;
		for(_port = 0; _port < vhc->port_count; _port++)
		{
			// The port which will be checked first, is rotated by port_sched_offset, so that every port
			// has its chance to be reported to user space, even if the hcd is under heavy load.
			port = (_port + ifcp->port_sched_offset) % vhc->port_count;
			if(vhc->port_update & (1 << (port + 1)))
			{
				vhc->port_update &= ~(1 << (port + 1));
				ifcp->port_sched_offset = port + 1;
				port_stat = vhc->ports[port];
				spin_unlock_irqrestore(&vhc->lock, flags);
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "cmd=USB_VHCI_HCD_IOCFETCHWORK [work=PORT_STAT port=%d status=0x%04x change=0x%04x]\n", (int)(port + 1), (int)port_stat.port_status, (int)port_stat.port_change);
#endif
				__put_user(USB_VHCI_WORK_TYPE_PORT_STAT, &arg->type);
				__put_user(port + 1, &arg->work.port.index);
				__put_user(port_stat.port_status, &arg->work.port.status);
				__put_user(port_stat.port_change, &arg->work.port.change);
				__put_user(port_stat.port_flags, &arg->work.port.flags);
				return 0;
			}
		}
	}

repeat:
	if(!list_empty(&vhc->urbp_list_inbox))
	{
		urbp = list_entry(vhc->urbp_list_inbox.next, struct usb_vhci_urb_priv, urbp_list);
		handle = (u64)(unsigned long)urbp->urb;
		memset(&urb, 0, sizeof urb);
		urb.address = usb_pipedevice(urbp->urb->pipe);
		urb.endpoint = usb_pipeendpoint(urbp->urb->pipe) | (usb_pipein(urbp->urb->pipe) ? 0x80 : 0x00);
		urb.type = conv_urb_type(usb_pipetype(urbp->urb->pipe));
		urb.flags = conv_urb_flags(urbp->urb->transfer_flags);
		if(usb_pipecontrol(urbp->urb->pipe))
		{
			const struct usb_ctrlrequest *cmd;
			u16 wValue, wIndex, wLength;
			if(unlikely(!urbp->urb->setup_packet))
				goto invalid_urb;
			cmd = (struct usb_ctrlrequest *)urbp->urb->setup_packet;
			wValue = le16_to_cpu(cmd->wValue);
			wIndex = le16_to_cpu(cmd->wIndex);
			wLength = le16_to_cpu(cmd->wLength);
			if(unlikely(wLength > urbp->urb->transfer_buffer_length))
				goto invalid_urb;
			if(cmd->bRequestType & 0x80)
			{
				if(unlikely(!wLength || !urbp->urb->transfer_buffer))
					goto invalid_urb;
			}
			else
			{
				if(unlikely(wLength && !urbp->urb->transfer_buffer))
					goto invalid_urb;
			}
			urb.buffer_length = wLength;
			urb.setup_packet.bmRequestType = cmd->bRequestType;
			urb.setup_packet.bRequest = cmd->bRequest;
			urb.setup_packet.wValue = wValue;
			urb.setup_packet.wIndex = wIndex;
			urb.setup_packet.wLength = wLength;
		}
		else
		{
			if(usb_pipein(urbp->urb->pipe))
			{
				if(unlikely(!urbp->urb->transfer_buffer_length || !urbp->urb->transfer_buffer))
					goto invalid_urb;
			}
			else
			{
				if(unlikely(urbp->urb->transfer_buffer_length && !urbp->urb->transfer_buffer))
					goto invalid_urb;
			}
			urb.buffer_length = urbp->urb->transfer_buffer_length;
		}
		urb.interval = urbp->urb->interval;
		urb.packet_count = urbp->urb->number_of_packets;

#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "cmd=USB_VHCI_HCD_IOCFETCHWORK [work=PROCESS_URB handle=0x%016llx]\n", handle);
#endif
		dump_urb(urbp->urb);
		list_move_tail(&urbp->urbp_list, &vhc->urbp_list_fetched);
		spin_unlock_irqrestore(&vhc->lock, flags);

		__put_user(USB_VHCI_WORK_TYPE_PROCESS_URB, &arg->type);
		__put_user(handle, &arg->handle);
		if(unlikely(__copy_to_user(&arg->work.urb, &urb, sizeof urb)))
			return -EFAULT;
		return 0;

	invalid_urb:
		// reject invalid urbs immediately
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "cmd=USB_VHCI_HCD_IOCFETCHWORK  <<< THROWING AWAY INVALID URB >>>  [handle=0x%016llx]\n", handle);
#endif
		usb_vhci_maybe_set_status(urbp, -EPIPE);
		usb_vhci_urb_giveback(vhc, urbp);
		goto repeat;
	}

	spin_unlock_irqrestore(&vhc->lock, flags);
	return -ENODATA;
}

// caller has lock
static inline struct usb_vhci_urb_priv *urbp_from_handle(struct usb_vhci_hcd *vhc, const void *handle)
{
	struct usb_vhci_urb_priv *entry;
	list_for_each_entry(entry, &vhc->urbp_list_fetched, urbp_list)
		if(entry->urb == handle)
			return entry;
	return NULL;
}

// caller has lock
static inline struct usb_vhci_urb_priv *urbp_from_handle_in_cancel(struct usb_vhci_hcd *vhc, const void *handle)
{
	struct usb_vhci_urb_priv *entry;
	list_for_each_entry(entry, &vhc->urbp_list_cancel, urbp_list)
		if(entry->urb == handle)
			return entry;
	return NULL;
}

// caller has lock
static inline struct usb_vhci_urb_priv *urbp_from_handle_in_canceling(struct usb_vhci_hcd *vhc, const void *handle)
{
	struct usb_vhci_urb_priv *entry;
	list_for_each_entry(entry, &vhc->urbp_list_canceling, urbp_list)
		if(entry->urb == handle)
			return entry;
	return NULL;
}

// caller has lock
static inline int is_urb_dir_in(const struct urb *urb)
{
	if(unlikely(usb_pipecontrol(urb->pipe)))
	{
		const struct usb_ctrlrequest *cmd = (struct usb_ctrlrequest *)urb->setup_packet;
		return cmd->bRequestType & 0x80;
	}
	else
		return usb_pipein(urb->pipe);
}

// -ECANCELED doesn't report an error, but it indicates that the urb was in the "cancel"
// list or in the "canceling" list.
// If this function reports an error (other than -ENOENT), then the urb will be given back to its creator anyway,
// if its handle was found. (If its handle wasn't found, then -ENOENT is returned.)
// called in ioc_giveback{,32} only
static int ioc_giveback_common(struct usb_vhci_hcd *vhc, const void *handle, int status, int act, int iso_count, int err_count, const void __user *buf, const struct usb_vhci_ioc_iso_packet_giveback __user *iso)
{
	struct usb_vhci_urb_priv *urbp;
	unsigned long flags;
	int retval = 0, is_in, is_iso, i;
#ifdef DEBUG
	struct device *dev = vhcihcd_to_dev(vhc);
#endif

	// TODO: do we really need to disable interrupts for accessing the urb lists?
	spin_lock_irqsave(&vhc->lock, flags);

	if(unlikely(!(urbp = urbp_from_handle(vhc, handle))))
	{
		// if not found, check the cancel{,ing} list
		if(likely((urbp = urbp_from_handle_in_canceling(vhc, handle)) ||
			(urbp = urbp_from_handle_in_cancel(vhc, handle))))
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK: urb was canceled\n");
#endif
			retval = -ECANCELED;
		}
		else
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK: handle not found\n");
#endif
			spin_unlock_irqrestore(&vhc->lock, flags);
			return -ENOENT;
		}
	}

	// remove urb from list before we release the spinlock
	list_del(&urbp->urbp_list);

	spin_unlock_irqrestore(&vhc->lock, flags);

	// usb_vhci_urb_giveback() (called below) will fail if we don't re-initialize
	// the list entry, because it calls list_del(), too!
	INIT_LIST_HEAD(&urbp->urbp_list);

	is_in = is_urb_dir_in(urbp->urb);
	is_iso = usb_pipeisoc(urbp->urb->pipe);

	if(likely(is_iso))
	{
		if(unlikely(is_in && act != urbp->urb->transfer_buffer_length))
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK(ISO): invalid: buffer_actual != buffer_length\n");
#endif
			retval = -ENOBUFS;
			goto done_with_errors;
		}
		if(unlikely(iso_count != urbp->urb->number_of_packets))
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK(ISO): invalid: number_of_packets missmatch\n");
#endif
			retval = -EINVAL;
			goto done_with_errors;
		}
		if(unlikely(iso_count && !iso))
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK(ISO): invalid: iso_packets must not be zero\n");
#endif
			retval = -EINVAL;
			goto done_with_errors;
		}
		if(likely(iso_count))
		{
			if(!access_ok(VERIFY_READ, (void *)iso, iso_count * sizeof(struct usb_vhci_ioc_iso_packet_giveback)))
			{
				retval = -EFAULT;
				goto done_with_errors;
			}
		}
	}
	else if(unlikely(act > urbp->urb->transfer_buffer_length))
	{
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "GIVEBACK: invalid: buffer_actual > buffer_length\n");
#endif
		retval = is_in ? -ENOBUFS : -EINVAL;
		goto done_with_errors;
	}
	if(is_in)
	{
		if(unlikely(act && !buf))
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK: buf must not be zero\n");
#endif
			retval = -EINVAL;
			goto done_with_errors;
		}
		if(unlikely(copy_from_user(urbp->urb->transfer_buffer, buf, act)))
		{
#ifdef DEBUG
			if(debug_output) dev_dbg(dev, "GIVEBACK: copy_from_user(buf) failed\n");
#endif
			retval = -EFAULT;
			goto done_with_errors;
		}
	}
	else if(unlikely(buf))
	{
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "GIVEBACK: invalid: buf should be NULL\n");
#endif
		// no data expected, so buf should be NULL
		retval = -EINVAL;
		goto done_with_errors;
	}
	if(likely(is_iso && iso_count))
	{
		for(i = 0; i < iso_count; i++)
		{
			__get_user(urbp->urb->iso_frame_desc[i].status, &iso[i].status);
			__get_user(urbp->urb->iso_frame_desc[i].actual_length, &iso[i].packet_actual);
		}
	}
	urbp->urb->actual_length = act;
	urbp->urb->error_count = err_count;

	// now we are done with this urb and it can return to its creator
	usb_vhci_maybe_set_status(urbp, status);
	spin_lock_irqsave(&vhc->lock, flags);
	usb_vhci_urb_giveback(vhc, urbp);
	spin_unlock_irqrestore(&vhc->lock, flags);
#ifdef DEBUG
	if(debug_output) dev_dbg(dev, "GIVEBACK: done\n");
#endif
	return retval;

done_with_errors:
	spin_lock_irqsave(&vhc->lock, flags);
	usb_vhci_urb_giveback(vhc, urbp);
	spin_unlock_irqrestore(&vhc->lock, flags);
#ifdef DEBUG
	if(debug_output) dev_dbg(dev, "GIVEBACK: done (with errors)\n");
#endif
	return retval;
}

// called in device_ioctl only
static int ioc_giveback(struct usb_vhci_hcd *vhc, const struct usb_vhci_ioc_giveback __user *arg)
{
	const struct usb_vhci_ioc_iso_packet_giveback __user *iso;
	const void *handle;
	const void __user *buf;
	u64 handle64;
	int status, act, iso_count, err_count;

#ifdef DEBUG
	if(debug_output) dev_dbg(vhcihcd_to_dev(vhc), "cmd=USB_VHCI_HCD_IOCGIVEBACK\n");
#endif

	if(sizeof(void *) > 4)
		__get_user(handle64, &arg->handle);
	else
	{
		u32 handle1, handle2;
		__get_user(handle1, (u32 __user *)&arg->handle);
		__get_user(handle2, (u32 __user *)&arg->handle + 1);
		*((u32 *)&handle64) = handle1;
		*((u32 *)&handle64 + 1) = handle2;
		if(handle64 >> 32)
			return -EINVAL;
	}
	__get_user(status, &arg->status);
	__get_user(act, &arg->buffer_actual);
	__get_user(iso_count, &arg->packet_count);
	__get_user(err_count, &arg->error_count);
	__get_user(buf, &arg->buffer);
	__get_user(iso, &arg->iso_packets);
	handle = (const void *)(unsigned long)handle64;
	if(unlikely(!handle))
		return -EINVAL;
	return ioc_giveback_common(vhc, handle, status, act, iso_count, err_count, buf, iso);
}

// called in ioc_fetch_data{,32} only
static int ioc_fetch_data_common(struct usb_vhci_hcd *vhc, const void *handle, void __user *user_buf, int user_len, struct usb_vhci_ioc_iso_packet_data __user *iso, int iso_count)
{
	struct usb_vhci_urb_priv *urbp;
	unsigned long flags;
	int tb_len, is_in, is_iso, i, ret = -ENOMEM;
	void *user_buf_tmp = NULL;
	struct usb_vhci_ioc_iso_packet_data *iso_tmp = NULL;

	if(likely(user_len))
	{
		user_buf_tmp = kmalloc(user_len, GFP_KERNEL);
		if(unlikely(!user_buf_tmp))
			goto end;
	}
	if(likely(iso_count))
	{
		iso_tmp = kmalloc(iso_count * sizeof *iso_tmp, GFP_KERNEL);
		if(unlikely(!iso_tmp))
			goto end;
	}
	ret = 0;

	spin_lock_irqsave(&vhc->lock, flags);
	if(unlikely(!(urbp = urbp_from_handle(vhc, handle))))
	{
		// if not found, check the cancel{,ing} list
		if(likely((urbp = urbp_from_handle_in_cancel(vhc, handle)) ||
			(urbp = urbp_from_handle_in_canceling(vhc, handle))))
		{
			// we can give the urb back to its creator now, because the user space is informed about
			// its cancelation
			usb_vhci_urb_giveback(vhc, urbp);
			ret = -ECANCELED;
			goto end_unlock;
		}
		ret = -ENOENT;
		goto end_unlock;
	}

	tb_len = urbp->urb->transfer_buffer_length;
	if(unlikely(usb_pipecontrol(urbp->urb->pipe)))
	{
		const struct usb_ctrlrequest *cmd = (struct usb_ctrlrequest *)urbp->urb->setup_packet;
		tb_len = le16_to_cpu(cmd->wLength);
	}

	is_in = is_urb_dir_in(urbp->urb);
	is_iso = usb_pipeisoc(urbp->urb->pipe);

	if(likely(is_iso))
	{
		if(unlikely(iso_count != urbp->urb->number_of_packets))
		{
			ret = -EINVAL;
			goto end_unlock;
		}
		if(likely(iso_count))
		{
			if(unlikely(!iso))
			{
				ret = -EINVAL;
				goto end_unlock;
			}
			for(i = 0; i < iso_count; i++)
			{
				iso_tmp[i].offset = urbp->urb->iso_frame_desc[i].offset;
				iso_tmp[i].packet_length = urbp->urb->iso_frame_desc[i].length;
			}
		}
	}
	else if(unlikely(is_in || !tb_len || !urbp->urb->transfer_buffer))
	{
		ret = -ENODATA;
		goto end_unlock;
	}

	if(likely(!is_in && tb_len))
	{
		if(unlikely(!user_buf || user_len < tb_len))
		{
			ret = -EINVAL;
			goto end_unlock;
		}
		memcpy(user_buf_tmp, urbp->urb->transfer_buffer, tb_len);
	}

	// we have copied all data into our private buffers, so we can release the spinlock
	spin_unlock_irqrestore(&vhc->lock, flags);

	// since we do not hold the spinlock any longer, we can now safely write the user-mode buffers

	if(likely(is_iso && iso_count))
	{
		if(unlikely(copy_to_user(iso, iso_tmp, iso_count * sizeof *iso_tmp)))
		{
			ret = -EFAULT;
			goto end;
		}
	}

	if(likely(!is_in && tb_len))
	{
		if(unlikely(copy_to_user(user_buf, user_buf_tmp, tb_len)))
		{
			ret = -EFAULT;
			goto end;
		}
	}

	goto end;
end_unlock:
	spin_unlock_irqrestore(&vhc->lock, flags);
end:
	kfree(user_buf_tmp);
	kfree(iso_tmp);
	return ret;
}

// called in device_ioctl only
static int ioc_fetch_data(struct usb_vhci_hcd *vhc, struct usb_vhci_ioc_urb_data __user *arg)
{
	struct usb_vhci_ioc_iso_packet_data __user *iso;
	const void *handle;
	void __user *user_buf;
	u64 handle64;
	int user_len, iso_count;

#ifdef DEBUG
	if(debug_output) dev_dbg(vhcihcd_to_dev(vhc), "cmd=USB_VHCI_HCD_IOCFETCHDATA\n");
#endif

	if(sizeof(void *) > 4)
		__get_user(handle64, &arg->handle);
	else
	{
		u32 handle1, handle2;
		__get_user(handle1, (u32 __user *)&arg->handle);
		__get_user(handle2, (u32 __user *)&arg->handle + 1);
		*((u32 *)&handle64) = handle1;
		*((u32 *)&handle64 + 1) = handle2;
		if(handle64 >> 32)
			return -EINVAL;
	}
	__get_user(user_len, &arg->buffer_length);
	__get_user(iso_count, &arg->packet_count);
	__get_user(user_buf, &arg->buffer);
	__get_user(iso, &arg->iso_packets);
	handle = (const void *)(unsigned long)handle64;
	if(unlikely(!handle))
		return -EINVAL;
	return ioc_fetch_data_common(vhc, handle, user_buf, user_len, iso, iso_count);
}

#ifdef CONFIG_COMPAT
// called in device_ioctl only
static int ioc_giveback32(struct usb_vhci_hcd *vhc, const struct usb_vhci_ioc_giveback32 __user *arg)
{
	const struct usb_vhci_ioc_iso_packet_giveback __user *iso;
	const void __user *buf;
	const void *handle;
	u64 handle64;
	int status, act, iso_count, err_count;
	u32 buf32, iso32;

#ifdef DEBUG
	if(debug_output) dev_dbg(vhcihcd_to_dev(vhc), "cmd=USB_VHCI_HCD_IOCGIVEBACK32\n");
#endif

	__get_user(handle64, &arg->handle);
	__get_user(status, &arg->status);
	__get_user(act, &arg->buffer_actual);
	__get_user(iso_count, &arg->packet_count);
	__get_user(err_count, &arg->error_count);
	__get_user(buf32, &arg->buffer);
	__get_user(iso32, &arg->iso_packets);
	handle = (const void *)(unsigned long)handle64;
	if(unlikely(!handle))
		return -EINVAL;
	buf = compat_ptr(buf32);
	iso = compat_ptr(iso32);
	return ioc_giveback_common(vhc, handle, status, act, iso_count, err_count, buf, iso);
}

// called in device_ioctl only
static int ioc_fetch_data32(struct usb_vhci_hcd *vhc, struct usb_vhci_ioc_urb_data32 __user *arg)
{
	struct usb_vhci_ioc_iso_packet_data __user *iso;
	void __user *user_buf;
	const void *handle;
	u64 handle64;
	int user_len, iso_count;
	u32 user_buf32, iso32;

#ifdef DEBUG
	if(debug_output) dev_dbg(vhcihcd_to_dev(vhc), "cmd=USB_VHCI_HCD_IOCFETCHDATA32\n");
#endif

	__get_user(handle64, &arg->handle);
	__get_user(user_len, &arg->buffer_length);
	__get_user(iso_count, &arg->packet_count);
	__get_user(user_buf32, &arg->buffer);
	__get_user(iso32, &arg->iso_packets);
	handle = (const void *)(unsigned long)handle64;
	if(unlikely(!handle))
		return -EINVAL;
	user_buf = compat_ptr(user_buf32);
	iso = compat_ptr(iso32);
	return ioc_fetch_data_common(vhc, handle, user_buf, user_len, iso, iso_count);
}
#endif

static long device_do_ioctl(struct file *file,
                           unsigned int cmd,
                           void __user *arg)
{
	struct usb_vhci_device *vdev;
	struct usb_vhci_hcd *vhc;
	long ret = 0;
	s16 timeout;

	// Floods the logs
	//vhci_dbg("%s(file=%p)\n", __FUNCTION__, file);

	if(unlikely(_IOC_TYPE(cmd) != USB_VHCI_HCD_IOC_MAGIC)) return -ENOTTY;
	if(unlikely(_IOC_NR(cmd) > USB_VHCI_HCD_IOC_MAXNR)) return -ENOTTY;

	if(unlikely((_IOC_DIR(cmd) & _IOC_READ) && !access_ok(VERIFY_WRITE, arg, _IOC_SIZE(cmd))))
		return -EFAULT;
	if(unlikely((_IOC_DIR(cmd) & _IOC_WRITE) && !access_ok(VERIFY_READ, arg, _IOC_SIZE(cmd))))
		return -EFAULT;

	if(unlikely(cmd == USB_VHCI_HCD_IOCREGISTER))
		return ioc_register(file, (struct usb_vhci_ioc_register __user *)arg);

	vdev = file->private_data;

	if(unlikely(!vdev))
		return -EPROTO;

	vhc = vhcidev_to_vhcihcd(vdev);

	switch(__builtin_expect(cmd, USB_VHCI_HCD_IOCFETCHWORK))
	{
	case USB_VHCI_HCD_IOCPORTSTAT:
		ret = ioc_port_stat(vdev, (struct usb_vhci_ioc_port_stat __user *)arg);
		break;

	case USB_VHCI_HCD_IOCFETCHWORK_RO:
		ret = ioc_fetch_work(vhc, (struct usb_vhci_ioc_work __user *)arg, 100);
		break;

	case USB_VHCI_HCD_IOCFETCHWORK:
		__get_user(timeout, &((struct usb_vhci_ioc_work __user *)arg)->timeout);
		ret = ioc_fetch_work(vhc, (struct usb_vhci_ioc_work __user *)arg, timeout);
		break;

	case USB_VHCI_HCD_IOCGIVEBACK:
		ret = ioc_giveback(vhc, (struct usb_vhci_ioc_giveback __user *)arg);
		break;

	case USB_VHCI_HCD_IOCFETCHDATA:
		ret = ioc_fetch_data(vhc, (struct usb_vhci_ioc_urb_data __user *)arg);
		break;

#ifdef CONFIG_COMPAT
	case USB_VHCI_HCD_IOCGIVEBACK32:
		ret = ioc_giveback32(vhc, (struct usb_vhci_ioc_giveback32 __user *)arg);
		break;

	case USB_VHCI_HCD_IOCFETCHDATA32:
		ret = ioc_fetch_data32(vhc, (struct usb_vhci_ioc_urb_data32 __user *)arg);
		break;
#endif

	default:
		ret = -ENOTTY;
	}

	return ret;
}

static long device_ioctl(struct file *file,
                         unsigned int cmd,
                         unsigned long arg)
{
	return device_do_ioctl(file, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long device_ioctl32(struct file *file,
                           unsigned int cmd,
                           unsigned long arg)
{
	return device_do_ioctl(file, cmd, compat_ptr(arg));
}
#endif

static loff_t device_llseek(struct file *file, loff_t offset, int origin)
{
	vhci_dbg("%s(file=%p)\n", __FUNCTION__, file);
	return -ESPIPE;
}

static struct file_operations fops = {
	.owner          = THIS_MODULE,
	.llseek         = device_llseek,
	.read           = device_read,
	.write          = device_write,
	.unlocked_ioctl = device_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = device_ioctl32,
#endif
	.open           = device_open,
	.release        = device_release // a.k.a. close
};

#ifdef DEBUG
static ssize_t show_debug_output(struct device_driver *drv, char *buf)
{
	if(buf != NULL)
	{
		switch(debug_output)
		{
		case 0:  *buf = '0'; break; // No debug output
		case 1:  *buf = '1'; break; // Debug output without data buffer dumps
		case 2:  *buf = '2'; break; // Debug output with short data buffer dumps
		default: *buf = '3'; break; // Debug output with full data buffer dumps
		}
	}
	return 1;
}

static ssize_t store_debug_output(struct device_driver *drv, const char *buf, size_t count)
{
	if(count != 1 || buf == NULL) return -EINVAL;
	switch(*buf)
	{
	case '0': debug_output = 0; return 1;
	case '1': debug_output = 1; return 1;
	case '2': debug_output = 2; return 1;
	case '3': debug_output = 3; return 1;
	}
	return -EINVAL;
}

static DRIVER_ATTR(debug_output, S_IRUSR | S_IWUSR, show_debug_output, store_debug_output);
#endif

static struct platform_driver vhci_iocifc_driver = {
	.driver = {
		.name   = driver_name,
		.owner  = THIS_MODULE
	}
};

static void vhci_iocifc_device_release(struct device *dev)
{
}

static int vhci_iocifc_major;

static struct class vhci_iocifc_class = {
	.owner = THIS_MODULE,
	.name = driver_name
};

#define CHARDEV_NAME "usb-vhci"

static struct device vhci_iocifc_device = {
	.class = &vhci_iocifc_class,
	.release = vhci_iocifc_device_release,
#ifndef NO_DEV_INIT_NAME
	.init_name = CHARDEV_NAME,
#endif
	.driver = &vhci_iocifc_driver.driver
};

static int __init init(void)
{
	int retval;

	if(usb_disabled()) return -ENODEV;

	vhci_printk(KERN_INFO, DRIVER_DESC " -- Version " DRIVER_VERSION "\n");

#ifdef DEBUG
	vhci_printk(KERN_DEBUG, "register platform_driver %s\n", driver_name);
#endif
	retval = platform_driver_register(&vhci_iocifc_driver);
	if(unlikely(retval < 0))
	{
		vhci_printk(KERN_ERR, "register platform_driver failed\n");
		return retval;
	}

	vhci_iocifc_major = register_chrdev(0, driver_name, &fops);
	if(unlikely(vhci_iocifc_major < 0))
	{
		vhci_printk(KERN_ERR, "Sorry, registering the character device failed with %d.\n", retval);
#ifdef DEBUG
		vhci_printk(KERN_DEBUG, "unregister platform_driver %s\n", driver_name);
#endif
		platform_driver_unregister(&vhci_iocifc_driver);
		return retval;
	}

	vhci_printk(KERN_INFO, "Successfully registered the character device.\n");
	vhci_printk(KERN_INFO, "The major device number is %d.\n", vhci_iocifc_major);

	if(unlikely(class_register(&vhci_iocifc_class)))
	{
		vhci_printk(KERN_WARNING, "failed to register class %s\n", driver_name);
	}
	else
	{
#ifdef NO_DEV_INIT_NAME
#ifdef OLD_DEV_BUS_ID
		strlcpy((void *)&vhci_iocifc_device.bus_id, CHARDEV_NAME, strlen(CHARDEV_NAME) + 1);
#else
		dev_set_name(&vhci_iocifc_device, CHARDEV_NAME);
#endif
#endif
		vhci_iocifc_device.devt = MKDEV(vhci_iocifc_major, 0);
		if(unlikely(device_register(&vhci_iocifc_device)))
		{
			vhci_printk(KERN_WARNING, "failed to register device " CHARDEV_NAME "\n");
		}
	}

#ifdef DEBUG
	retval = driver_create_file(&vhci_iocifc_driver.driver, &driver_attr_debug_output);
	if(unlikely(retval != 0))
	{
		vhci_printk(KERN_DEBUG, "driver_create_file(&vhci_iocifc_driver, &driver_attr_debug_output) failed\n");
		vhci_printk(KERN_DEBUG, "==> ignoring\n");
	}
#endif

#ifdef DEBUG
	vhci_printk(KERN_DEBUG, "USB_VHCI_HCD_IOCREGISTER     = %08x\n", (unsigned int)USB_VHCI_HCD_IOCREGISTER);
	vhci_printk(KERN_DEBUG, "USB_VHCI_HCD_IOCPORTSTAT     = %08x\n", (unsigned int)USB_VHCI_HCD_IOCPORTSTAT);
	vhci_printk(KERN_DEBUG, "USB_VHCI_HCD_IOCFETCHWORK_RO = %08x\n", (unsigned int)USB_VHCI_HCD_IOCFETCHWORK_RO);
	vhci_printk(KERN_DEBUG, "USB_VHCI_HCD_IOCFETCHWORK    = %08x\n", (unsigned int)USB_VHCI_HCD_IOCFETCHWORK);
	vhci_printk(KERN_DEBUG, "USB_VHCI_HCD_IOCGIVEBACK     = %08x\n", (unsigned int)USB_VHCI_HCD_IOCGIVEBACK);
	vhci_printk(KERN_DEBUG, "USB_VHCI_HCD_IOCFETCHDATA    = %08x\n", (unsigned int)USB_VHCI_HCD_IOCFETCHDATA);
#endif

	return 0;
}
module_init(init);

static void __exit cleanup(void)
{
#ifdef DEBUG
	driver_remove_file(&vhci_iocifc_driver.driver, &driver_attr_debug_output);
#endif
	device_unregister(&vhci_iocifc_device);
	class_unregister(&vhci_iocifc_class);
	unregister_chrdev(vhci_iocifc_major, driver_name);
	vhci_dbg("unregister platform_driver %s\n", driver_name);
	platform_driver_unregister(&vhci_iocifc_driver);
	vhci_dbg("gone\n");
}
module_exit(cleanup);
