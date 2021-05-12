/*
 * usb-vhci-hcd.c -- VHCI USB host controller driver.
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
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/device.h>

#include <asm/atomic.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>

#include "usb-vhci-hcd.h"

#define DRIVER_NAME "usb_vhci_hcd"
#define DRIVER_DESC "USB Virtual Host Controller Interface"
#define DRIVER_VERSION USB_VHCI_HCD_VERSION " (" USB_VHCI_HCD_DATE ")"

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

static inline const char *vhci_dev_name(struct device *dev)
{
#ifdef OLD_DEV_BUS_ID
	return dev->bus_id;
#else
	return dev_name(dev);
#endif
}

const char *usb_vhci_dev_name(struct usb_vhci_device *vdev)
{
	struct platform_device *pdev;
	struct usb_vhci_hcd *vhc;

	vhc = vhcidev_to_vhcihcd(vdev);

	if(likely(vhc))
	{
		pdev = vhcidev_to_pdev(vdev);
		return vhci_dev_name(&pdev->dev);
	}

	return "<unknown>";
}
EXPORT_SYMBOL_GPL(usb_vhci_dev_name);

int usb_vhci_dev_id(struct usb_vhci_device *vdev)
{
	return vhcidev_to_pdev(vdev)->id;
}
EXPORT_SYMBOL_GPL(usb_vhci_dev_id);

int usb_vhci_dev_busnum(struct usb_vhci_device *vdev)
{
	return vhcidev_to_usbhcd(vdev)->self.busnum;
}
EXPORT_SYMBOL_GPL(usb_vhci_dev_busnum);

void usb_vhci_maybe_set_status(struct usb_vhci_urb_priv *urbp, int status)
{
#ifdef OLD_GIVEBACK_MECH
	struct urb *const urb = urbp->urb;
	unsigned long flags;
	spin_lock_irqsave(&urb->lock, flags);
	if(urb->status == -EINPROGRESS)
		urb->status = status;
	spin_unlock_irqrestore(&urb->lock, flags);
#else
	(void)atomic_cmpxchg(&urbp->status, -EINPROGRESS, status);
#endif
}
EXPORT_SYMBOL_GPL(usb_vhci_maybe_set_status);

#ifdef DEBUG
#include "usb-vhci-dump-urb.c"
#else
static inline void dump_urb(struct urb *urb) {/* do nothing */}
#endif

// caller has vhc->lock
// first port is port# 1 (not 0)
static void vhci_port_update(struct usb_vhci_hcd *vhc, u8 port)
{
	struct usb_vhci_device *vdev = vhcihcd_to_vhcidev(vhc);
	vhc->port_update |= 1 << port;
	vdev->ifc->wakeup(vdev);
}

// gives the urb back to its original owner/creator.
// caller owns vhc->lock and has irq disabled.
void usb_vhci_urb_giveback(struct usb_vhci_hcd *vhc, struct usb_vhci_urb_priv *urbp)
{
	struct device *dev;
	struct usb_hcd *hcd;
	struct urb *const urb = urbp->urb;
	struct usb_device *const udev = urb->dev;
#ifndef OLD_GIVEBACK_MECH
	int status;
#endif
	hcd = vhcihcd_to_usbhcd(vhc);
	dev = vhcihcd_to_dev(vhc);
	trace_function(dev);
#ifndef OLD_GIVEBACK_MECH
	status = atomic_read(&urbp->status);
#endif
	urb->hcpriv = NULL;
	list_del(&urbp->urbp_list);
#ifndef OLD_GIVEBACK_MECH
	usb_hcd_unlink_urb_from_ep(hcd, urb);
#endif
	spin_unlock(&vhc->lock);
	kfree(urbp);
	dump_urb(urb);
#ifdef OLD_GIVEBACK_MECH
	usb_hcd_giveback_urb(hcd, urb);
#else
#	ifdef DEBUG
	if(debug_output) vhci_printk(KERN_DEBUG, "usb_vhci_urb_giveback: status=%d(%s)\n", status, get_status_str(status));
#	endif
	usb_hcd_giveback_urb(hcd, urb, status);
#endif
	usb_put_dev(udev);
	spin_lock(&vhc->lock);
}
EXPORT_SYMBOL_GPL(usb_vhci_urb_giveback);

#ifdef OLD_GIVEBACK_MECH
static int vhci_urb_enqueue(struct usb_hcd *hcd, struct usb_host_endpoint *ep, struct urb *urb, gfp_t mem_flags)
#else
static int vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
#endif
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;
	struct usb_vhci_urb_priv *urbp;
	struct usb_vhci_device *vdev;
	unsigned long flags;
#ifndef OLD_GIVEBACK_MECH
	int retval;
#endif

	vhc = usbhcd_to_vhcihcd(hcd);
	dev = vhcihcd_to_dev(vhc);
	vdev = vhcihcd_to_vhcidev(vhc);

	trace_function(dev);

	if(unlikely(!urb->transfer_buffer && urb->transfer_buffer_length))
		return -EINVAL;

	urbp = kzalloc(sizeof *urbp, mem_flags);
	if(unlikely(!urbp))
		return -ENOMEM;
	urbp->urb = urb;
	atomic_set(&urbp->status, urb->status);

	vhci_dbg("vhci_urb_enqueue: urb->status = %d(%s)",urb->status,get_status_str(urb->status));

	spin_lock_irqsave(&vhc->lock, flags);
#ifndef OLD_GIVEBACK_MECH
	retval = usb_hcd_link_urb_to_ep(hcd, urb);
	if(unlikely(retval))
	{
		kfree(urbp);
		spin_unlock_irqrestore(&vhc->lock, flags);
		return retval;
	}
#endif
	usb_get_dev(urb->dev);
	list_add_tail(&urbp->urbp_list, &vhc->urbp_list_inbox);
	urb->hcpriv = urbp;
	spin_unlock_irqrestore(&vhc->lock, flags);
	vdev->ifc->wakeup(vdev);
	return 0;
}

#ifdef OLD_GIVEBACK_MECH
static int vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
#else
static int vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
#endif
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;
	struct usb_vhci_device *vdev;
	unsigned long flags;
	struct usb_vhci_urb_priv *entry, *urbp = NULL;
#ifndef OLD_GIVEBACK_MECH
	int retval;
#endif

	vhc = usbhcd_to_vhcihcd(hcd);
	dev = vhcihcd_to_dev(vhc);
	vdev = vhcihcd_to_vhcidev(vhc);

	trace_function(dev);

	spin_lock_irqsave(&vhc->lock, flags);
#ifndef OLD_GIVEBACK_MECH
	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	if(retval)
	{
		spin_unlock_irqrestore(&vhc->lock, flags);
		return retval;
	}
#endif

	// search the queue of unprocessed urbs (inbox)
	list_for_each_entry(entry, &vhc->urbp_list_inbox, urbp_list)
	{
		if(entry->urb == urb)
		{
			urbp = entry;
			break;
		}
	}

	// if found in inbox
	if(urbp)
		usb_vhci_urb_giveback(vhc, urbp);
	else // if not found...
	{
		// ...then check if the urb is on a vacation through user space
		list_for_each_entry(entry, &vhc->urbp_list_fetched, urbp_list)
		{
			if(entry->urb == urb)
			{
				// move it into the cancel list
				list_move_tail(&entry->urbp_list, &vhc->urbp_list_cancel);
				vdev->ifc->wakeup(vdev);
				break;
			}
		}
	}

	spin_unlock_irqrestore(&vhc->lock, flags);
	return 0;
}

/*
static void vhci_timer(unsigned long _vhc)
{
	struct usb_vhci_hcd *vhc = (struct usb_vhci_hcd *)_vhc;
}
*/

static int vhci_hub_status(struct usb_hcd *hcd, char *buf)
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;
	unsigned long flags;
	u8 port;
	int changed = 0;
	int idx, rel_bit, abs_bit;

	vhc = usbhcd_to_vhcihcd(hcd);
	dev = vhcihcd_to_dev(vhc);

	trace_function(dev);

	memset(buf, 0, 1 + vhc->port_count / 8);

	spin_lock_irqsave(&vhc->lock, flags);
	if(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))
	{
		spin_unlock_irqrestore(&vhc->lock, flags);
		return 0;
	}

	for(port = 0; port < vhc->port_count; port++)
	{
		if(vhc->ports[port].port_change)
		{
			abs_bit = port + 1;
			idx     = abs_bit / (sizeof *buf * 8);
			rel_bit = abs_bit % (sizeof *buf * 8);
			buf[idx] |= (1 << rel_bit);
			changed = 1;
		}
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "port %d status 0x%04x has changes at 0x%04x\n", (int)(port + 1), (int)vhc->ports[port].port_status, (int)vhc->ports[port].port_change);
#endif
	}

	if(vhc->rh_state == USB_VHCI_RH_SUSPENDED && changed)
		usb_hcd_resume_root_hub(hcd);

	spin_unlock_irqrestore(&vhc->lock, flags);
	return changed;
}

// caller has vhc->lock
// called in vhci_hub_control only
static inline void hub_descriptor(const struct usb_vhci_hcd *vhc, char *buf, u16 len)
{
	struct usb_hub_descriptor desc;
	int portArrLen = vhc->port_count / 8 + 1; // length of one port bit-array in bytes
	u16 l = USB_DT_HUB_NONVAR_SIZE + 2 * portArrLen; // length of our hub descriptor
	memset(&desc, 0, USB_DT_HUB_NONVAR_SIZE);

	if(likely(len > USB_DT_HUB_NONVAR_SIZE))
	{
		if(unlikely(len < l)) l = len;
		if(likely(l > USB_DT_HUB_NONVAR_SIZE))
		{
			memset(buf + USB_DT_HUB_NONVAR_SIZE, 0, l - USB_DT_HUB_NONVAR_SIZE);
			if(likely(l > USB_DT_HUB_NONVAR_SIZE + portArrLen))
				memset(buf + USB_DT_HUB_NONVAR_SIZE + portArrLen, 0xff, l - (USB_DT_HUB_NONVAR_SIZE + portArrLen));
		}
	}
	else l = len;

	desc.bDescLength = l;
	desc.bDescriptorType = 0x29;
	desc.bNbrPorts = vhc->port_count;
	desc.wHubCharacteristics = __constant_cpu_to_le16(0x0009); // Per port power and overcurrent
	memcpy(buf, &desc, l);
}

static int vhci_hub_control(struct usb_hcd *hcd,
                            u16 typeReq,
                            u16 wValue,
                            u16 wIndex,
                            char *buf,
                            u16 wLength)
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;
	int retval = 0;
	unsigned long flags;
	u16 *ps, *pc;
	u8 *pf;
	u8 port, has_changes = 0;

	vhc = usbhcd_to_vhcihcd(hcd);
	dev = vhcihcd_to_dev(vhc);

	trace_function(dev);

	if(unlikely(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)))
		return -ETIMEDOUT;

	spin_lock_irqsave(&vhc->lock, flags);

	switch(typeReq)
	{
	case ClearHubFeature:
	case SetHubFeature:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: %sHubFeature [wValue=0x%04x]\n", __FUNCTION__, (typeReq == ClearHubFeature) ? "Clear" : "Set", (int)wValue);
#endif
		if(unlikely(wIndex || wLength || (wValue != C_HUB_LOCAL_POWER && wValue != C_HUB_OVER_CURRENT)))
			goto err;
		break;
	case ClearPortFeature:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: ClearPortFeature [wValue=0x%04x, wIndex=%d]\n", __FUNCTION__, (int)wValue, (int)wIndex);
#endif
		if(unlikely(!wIndex || wIndex > vhc->port_count || wLength))
			goto err;
		ps = &vhc->ports[wIndex - 1].port_status;
		pc = &vhc->ports[wIndex - 1].port_change;
		pf = &vhc->ports[wIndex - 1].port_flags;
		switch(wValue)
		{
		case USB_PORT_FEAT_SUSPEND:
			// (see USB 2.0 spec section 11.5 and 11.24.2.7.1.3)
			if(*ps & USB_PORT_STAT_SUSPEND)
			{
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "Port %d resuming\n", (int)wIndex);
#endif
				*pf |= USB_VHCI_PORT_STAT_FLAG_RESUMING;
				vhci_port_update(vhc, wIndex);
			}
			break;
		case USB_PORT_FEAT_POWER:
			// (see USB 2.0 spec section 11.11 and 11.24.2.7.1.6)
			if(*ps & USB_PORT_STAT_POWER)
			{
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "Port %d power-off\n", (int)wIndex);
#endif
				// clear all status bits except overcurrent (see USB 2.0 spec section 11.24.2.7.1)
				*ps &= USB_PORT_STAT_OVERCURRENT;
				// clear all change bits except overcurrent (see USB 2.0 spec section 11.24.2.7.2)
				*pc &= USB_PORT_STAT_C_OVERCURRENT;
				// clear resuming flag
				*pf &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;
				vhci_port_update(vhc, wIndex);
			}
			break;
		case USB_PORT_FEAT_ENABLE:
			// (see USB 2.0 spec section 11.5.1.4 and 11.24.2.7.{1,2}.2)
			if(*ps & USB_PORT_STAT_ENABLE)
			{
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "Port %d disabled\n", (int)wIndex);
#endif
				// clear enable and suspend bits (see section 11.24.2.7.1.{2,3})
				*ps &= ~(USB_PORT_STAT_ENABLE | USB_PORT_STAT_SUSPEND);
				// i'm not quite sure if the suspend change bit should be cleared too (see section 11.24.2.7.2.{2,3})
				*pc &= ~(USB_PORT_STAT_C_ENABLE | USB_PORT_STAT_C_SUSPEND);
				// clear resuming flag
				*pf &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;
				// TODO: maybe we should clear the low/high speed bits here (section 11.24.2.7.1.{7,8})
				vhci_port_update(vhc, wIndex);
			}
			break;
		case USB_PORT_FEAT_CONNECTION:
		case USB_PORT_FEAT_OVER_CURRENT:
		case USB_PORT_FEAT_RESET:
		case USB_PORT_FEAT_LOWSPEED:
		case USB_PORT_FEAT_HIGHSPEED:
		case USB_PORT_FEAT_INDICATOR:
			break; // no-op
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			if(*pc & (1 << (wValue - 16)))
			{
				*pc &= ~(1 << (wValue - 16));
				vhci_port_update(vhc, wIndex);
			}
			break;
		//case USB_PORT_FEAT_TEST:
		default:
			goto err;
		}
		break;
	case GetHubDescriptor:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: GetHubDescriptor [wValue=0x%04x, wLength=%d]\n", __FUNCTION__, (int)wValue, (int)wLength);
#endif
		if(unlikely(wIndex))
			goto err;
		hub_descriptor(vhc, buf, wLength);
		break;
	case GetHubStatus:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: GetHubStatus\n", __FUNCTION__);
#endif
		if(unlikely(wValue || wIndex || wLength != 4))
			goto err;
		buf[0] = buf[1] = buf[2] = buf[3] = 0;
		break;
	case GetPortStatus:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: GetPortStatus [wIndex=%d]\n", __FUNCTION__, (int)wIndex);
#endif
		if(unlikely(wValue || !wIndex || wIndex > vhc->port_count || wLength != 4))
			goto err;
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: ==> [port_status=0x%04x] [port_change=0x%04x]\n", __FUNCTION__, (int)vhc->ports[wIndex - 1].port_status, (int)vhc->ports[wIndex - 1].port_change);
#endif
		buf[0] = (u8)vhc->ports[wIndex - 1].port_status;
		buf[1] = (u8)(vhc->ports[wIndex - 1].port_status >> 8);
		buf[2] = (u8)vhc->ports[wIndex - 1].port_change;
		buf[3] = (u8)(vhc->ports[wIndex - 1].port_change >> 8);
		break;
	case SetPortFeature:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: SetPortFeature [wValue=0x%04x, wIndex=%d]\n", __FUNCTION__, (int)wValue, (int)wIndex);
#endif
		if(unlikely(!wIndex || wIndex > vhc->port_count || wLength))
			goto err;
		ps = &vhc->ports[wIndex - 1].port_status;
		pc = &vhc->ports[wIndex - 1].port_change;
		pf = &vhc->ports[wIndex - 1].port_flags;
		switch(wValue)
		{
		case USB_PORT_FEAT_SUSPEND:
			// USB 2.0 spec section 11.24.2.7.1.3:
			//  "This bit can be set only if the port’s PORT_ENABLE bit is set and the hub receives
			//  a SetPortFeature(PORT_SUSPEND) request."
			// The spec also says that the suspend bit has to be cleared whenever the enable bit is cleared.
			// (see also section 11.5)
			if((*ps & USB_PORT_STAT_ENABLE) && !(*ps & USB_PORT_STAT_SUSPEND))
			{
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "Port %d suspended\n", (int)wIndex);
#endif
				*ps |= USB_PORT_STAT_SUSPEND;
				vhci_port_update(vhc, wIndex);
			}
			break;
		case USB_PORT_FEAT_POWER:
			// (see USB 2.0 spec section 11.11 and 11.24.2.7.1.6)
			if(!(*ps & USB_PORT_STAT_POWER))
			{
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "Port %d power-on\n", (int)wIndex);
#endif
				*ps |= USB_PORT_STAT_POWER;
				vhci_port_update(vhc, wIndex);
			}
			break;
		case USB_PORT_FEAT_RESET:
			// (see USB 2.0 spec section 11.24.2.7.1.5)
			// initiate reset only if there is a device plugged into the port and if there isn't already a reset pending
			if((*ps & USB_PORT_STAT_CONNECTION) && !(*ps & USB_PORT_STAT_RESET))
			{
#ifdef DEBUG
				if(debug_output) dev_dbg(dev, "Port %d resetting\n", (int)wIndex);
#endif

				// keep the state of these bits and clear all others
				*ps &= USB_PORT_STAT_POWER
				     | USB_PORT_STAT_CONNECTION
				     | USB_PORT_STAT_LOW_SPEED
				     | USB_PORT_STAT_HIGH_SPEED
				     | USB_PORT_STAT_OVERCURRENT;

				*ps |= USB_PORT_STAT_RESET; // reset initiated

				// clear resuming flag
				*pf &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;

				vhci_port_update(vhc, wIndex);
			}
#ifdef DEBUG
			else if(debug_output) dev_dbg(dev, "Port %d reset not possible because of port_state=%04x\n", (int)wIndex, (int)*ps);
#endif
			break;
		case USB_PORT_FEAT_CONNECTION:
		case USB_PORT_FEAT_OVER_CURRENT:
		case USB_PORT_FEAT_LOWSPEED:
		case USB_PORT_FEAT_HIGHSPEED:
		case USB_PORT_FEAT_INDICATOR:
			break; // no-op
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			if(!(*pc & (1 << (wValue - 16))))
			{
				*pc |= 1 << (wValue - 16);
				vhci_port_update(vhc, wIndex);
			}
			break;
		//case USB_PORT_FEAT_ENABLE: // port can't be enabled without reseting (USB 2.0 spec section 11.24.2.7.1.2)
		//case USB_PORT_FEAT_TEST:
		default:
			goto err;
		}
		break;
	default:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: +++UNHANDLED_REQUEST+++ [req=0x%04x, v=0x%04x, i=0x%04x, l=%d]\n", __FUNCTION__, (int)typeReq, (int)wValue, (int)wIndex, (int)wLength);
#endif
err:
#ifdef DEBUG
		if(debug_output) dev_dbg(dev, "%s: STALL\n", __FUNCTION__);
#endif
		// "protocol stall" on error
		retval = -EPIPE;
	}

	for(port = 0; port < vhc->port_count; port++)
		if(vhc->ports[port].port_change)
			has_changes = 1;

	spin_unlock_irqrestore(&vhc->lock, flags);

	if(has_changes)
		usb_hcd_poll_rh_status(hcd);
	return retval;
}

static int vhci_bus_suspend(struct usb_hcd *hcd)
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;
	unsigned long flags;
	u8 port;

	vhc = usbhcd_to_vhcihcd(hcd);
	dev = vhcihcd_to_dev(vhc);

	trace_function(dev);

	spin_lock_irqsave(&vhc->lock, flags);

	// suspend ports
	for(port = 0; port < vhc->port_count; port++)
	{
		if((vhc->ports[port].port_status & USB_PORT_STAT_ENABLE) &&
			!(vhc->ports[port].port_status & USB_PORT_STAT_SUSPEND))
		{
			dev_dbg(dev, "Port %d suspended\n", (int)port + 1);
			vhc->ports[port].port_status |= USB_PORT_STAT_SUSPEND;
			vhc->ports[port].port_flags &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;
			vhci_port_update(vhc, port + 1);
		}
	}

	// TODO: somehow we have to suppress the resuming of ports while the bus is suspended

	vhc->rh_state = USB_VHCI_RH_SUSPENDED;
	hcd->state = HC_STATE_SUSPENDED;

	spin_unlock_irqrestore(&vhc->lock, flags);

	return 0;
}

static int vhci_bus_resume(struct usb_hcd *hcd)
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;
	int rc = 0;
	unsigned long flags;

	vhc = usbhcd_to_vhcihcd(hcd);
	dev = vhcihcd_to_dev(vhc);

	trace_function(dev);

	spin_lock_irqsave(&vhc->lock, flags);
	if(unlikely(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)))
	{
		dev_warn(&hcd->self.root_hub->dev, "HC isn't running! You have to resume the host controller device before you resume the root hub.\n");
		rc = -ENODEV;
	}
	else
	{
		vhc->rh_state = USB_VHCI_RH_RUNNING;
		//set_link_state(vhc);
		hcd->state = HC_STATE_RUNNING;
	}
	spin_unlock_irqrestore(&vhc->lock, flags);

	return rc;
}

static inline ssize_t show_urb(char *buf, size_t size, struct urb *urb)
{
	int ep = usb_pipeendpoint(urb->pipe);

	return snprintf(buf, size,
		"urb/%p %s ep%d%s%s len %d/%d\n",
		urb,
		({
			char *s;
			switch(urb->dev->speed)
			{
			case USB_SPEED_LOW:  s = "ls"; break;
			case USB_SPEED_FULL: s = "fs"; break;
			case USB_SPEED_HIGH: s = "hs"; break;
			default:             s = "?";  break;
			};
			s;
		}),
		ep, ep ? (usb_pipein(urb->pipe) ? "in" : "out") : "",
		({
			char *s;
			switch(usb_pipetype(urb->pipe))
			{
			case PIPE_CONTROL:   s = "";      break;
			case PIPE_BULK:      s = "-bulk"; break;
			case PIPE_INTERRUPT: s = "-int";  break;
			default:             s = "-iso";  break;
			};
			s;
		}),
		urb->actual_length, urb->transfer_buffer_length);
}

static ssize_t show_urbs(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(urbs_inbox,     S_IRUSR, show_urbs, NULL);
static DEVICE_ATTR(urbs_fetched,   S_IRUSR, show_urbs, NULL);
static DEVICE_ATTR(urbs_cancel,    S_IRUSR, show_urbs, NULL);
static DEVICE_ATTR(urbs_canceling, S_IRUSR, show_urbs, NULL);

static ssize_t show_urbs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_vhci_hcd *vhc;
	struct platform_device *pdev;
	struct usb_vhci_urb_priv *urbp;
	size_t size = 0;
	unsigned long flags;
	struct list_head *list;

	pdev = to_platform_device(dev);
	vhc = pdev_to_vhcihcd(pdev);

	trace_function(dev);

	if(attr == &dev_attr_urbs_inbox)
		list = &vhc->urbp_list_inbox;
	else if(attr == &dev_attr_urbs_fetched)
		list = &vhc->urbp_list_fetched;
	else if(attr == &dev_attr_urbs_cancel)
		list = &vhc->urbp_list_cancel;
	else if(attr == &dev_attr_urbs_canceling)
		list = &vhc->urbp_list_canceling;
	else
	{
		dev_err(dev, "unreachable code reached... wtf?\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&vhc->lock, flags);
	list_for_each_entry(urbp, list, urbp_list)
	{
		size_t temp;

		temp = PAGE_SIZE - size;
		if(unlikely(temp <= 0)) break;

		temp = show_urb(buf, temp, urbp->urb);
		buf += temp;
		size += temp;
	}
	spin_unlock_irqrestore(&vhc->lock, flags);

	return size;
}

static int vhci_start(struct usb_hcd *hcd)
{
	struct usb_vhci_hcd *vhc;
	int retval;
	struct usb_vhci_port *ports;
	struct usb_vhci_device *vdev;
	struct device *dev;

	dev = usbhcd_to_dev(hcd);

	trace_function(dev);

	vhc = usbhcd_to_vhcihcd(hcd);
	vdev = vhcihcd_to_vhcidev(vhc);

	ports = kzalloc(vdev->port_count * sizeof(struct usb_vhci_port), GFP_KERNEL);
	if(unlikely(ports == NULL)) return -ENOMEM;

	spin_lock_init(&vhc->lock);
	//init_timer(&vhc->timer);
	//vhc->timer.function = vhci_timer;
	//vhc->timer.data = (unsigned long)vhc;
	vhc->ports = ports;
	vhc->port_count = vdev->port_count;
	vhc->port_update = 0;
	atomic_set(&vhc->frame_num, 0);
	INIT_LIST_HEAD(&vhc->urbp_list_inbox);
	INIT_LIST_HEAD(&vhc->urbp_list_fetched);
	INIT_LIST_HEAD(&vhc->urbp_list_cancel);
	INIT_LIST_HEAD(&vhc->urbp_list_canceling);
	vhc->rh_state = USB_VHCI_RH_RUNNING;

	hcd->power_budget = 500; // NOTE: practically we have unlimited power because this is a virtual device with... err... virtual power!
	hcd->state = HC_STATE_RUNNING;
	hcd->uses_new_polling = 1;
#ifndef NO_HAS_TT_FLAG
	hcd->has_tt = 1;
#endif

	retval = device_create_file(dev, &dev_attr_urbs_inbox);
	if(unlikely(retval != 0)) goto kfree_port_arr;
	retval = device_create_file(dev, &dev_attr_urbs_fetched);
	if(unlikely(retval != 0)) goto rem_file_inbox;
	retval = device_create_file(dev, &dev_attr_urbs_cancel);
	if(unlikely(retval != 0)) goto rem_file_fetched;
	retval = device_create_file(dev, &dev_attr_urbs_canceling);
	if(unlikely(retval != 0)) goto rem_file_cancel;

	return 0;

rem_file_cancel:
	device_remove_file(dev, &dev_attr_urbs_cancel);

rem_file_fetched:
	device_remove_file(dev, &dev_attr_urbs_fetched);

rem_file_inbox:
	device_remove_file(dev, &dev_attr_urbs_inbox);

kfree_port_arr:
	kfree(ports);
	vhc->ports = NULL;
	vhc->port_count = 0;
	return retval;
}

static void vhci_stop(struct usb_hcd *hcd)
{
	struct usb_vhci_hcd *vhc;
	struct device *dev;

	dev = usbhcd_to_dev(hcd);

	trace_function(dev);

	vhc = usbhcd_to_vhcihcd(hcd);

	device_remove_file(dev, &dev_attr_urbs_canceling);
	device_remove_file(dev, &dev_attr_urbs_cancel);
	device_remove_file(dev, &dev_attr_urbs_fetched);
	device_remove_file(dev, &dev_attr_urbs_inbox);

	if(likely(vhc->ports))
	{
		kfree(vhc->ports);
		vhc->ports = NULL;
		vhc->port_count = 0;
	}

	vhc->rh_state = USB_VHCI_RH_RESET;
	dev_info(dev, "stopped\n");
}

static int vhci_get_frame(struct usb_hcd *hcd)
{
	struct usb_vhci_hcd *vhc;
	vhc = usbhcd_to_vhcihcd(hcd);
	trace_function(usbhcd_to_dev(hcd));
	return atomic_read(&vhc->frame_num);
}

static const struct hc_driver vhci_hcd = {
	.description      = driver_name,
	.product_desc     = "VHCI Host Controller",
	.hcd_priv_size    = sizeof(struct usb_vhci_hcd),

	.flags            = HCD_USB2,

	.start            = vhci_start,
	.stop             = vhci_stop,

	.urb_enqueue      = vhci_urb_enqueue,
	.urb_dequeue      = vhci_urb_dequeue,

	.get_frame_number = vhci_get_frame,

	.hub_status_data  = vhci_hub_status,
	.hub_control      = vhci_hub_control,
	.bus_suspend      = vhci_bus_suspend,
	.bus_resume       = vhci_bus_resume
};

static int vhci_hcd_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct usb_vhci_device *vdev;
	int retval;

	vdev = pdev_to_vhcidev(pdev);

#ifdef DEBUG
	if(debug_output) dev_dbg(&pdev->dev, "%s\n", __FUNCTION__);
#endif
	dev_info(&pdev->dev, DRIVER_DESC " -- Version " DRIVER_VERSION "\n");
	dev_info(&pdev->dev, "--> Backend: %s\n", vdev->ifc->ifc_desc);

	hcd = usb_create_hcd(&vhci_hcd, &pdev->dev, vhci_dev_name(&pdev->dev));
	if(unlikely(!hcd)) return -ENOMEM;
	vdev->vhc = usbhcd_to_vhcihcd(hcd);

	retval = usb_add_hcd(hcd, 0, 0); // calls vhci_start
	if(unlikely(retval)) usb_put_hcd(hcd);

	return retval;
}

static int vhci_hcd_remove(struct platform_device *pdev)
{
	unsigned long flags;
	struct usb_hcd *hcd;
	struct usb_vhci_hcd *vhc;
	struct usb_vhci_urb_priv *urbp;
	struct usb_vhci_device *vdev;

	vdev = pdev_to_vhcidev(pdev);
	vhc = vhcidev_to_vhcihcd(vdev);
	hcd = vhcidev_to_usbhcd(vdev);

	trace_function(vhcihcd_to_dev(vhc));

	spin_lock_irqsave(&vhc->lock, flags);
	while(!list_empty(&vhc->urbp_list_inbox))
	{
		urbp = list_entry(vhc->urbp_list_inbox.next, struct usb_vhci_urb_priv, urbp_list);
		usb_vhci_maybe_set_status(urbp, -ESHUTDOWN);
		usb_vhci_urb_giveback(vhc, urbp);
	}
	while(!list_empty(&vhc->urbp_list_fetched))
	{
		urbp = list_entry(vhc->urbp_list_fetched.next, struct usb_vhci_urb_priv, urbp_list);
		usb_vhci_maybe_set_status(urbp, -ESHUTDOWN);
		usb_vhci_urb_giveback(vhc, urbp);
	}
	while(!list_empty(&vhc->urbp_list_cancel))
	{
		urbp = list_entry(vhc->urbp_list_cancel.next, struct usb_vhci_urb_priv, urbp_list);
		usb_vhci_maybe_set_status(urbp, -ESHUTDOWN);
		usb_vhci_urb_giveback(vhc, urbp);
	}
	while(!list_empty(&vhc->urbp_list_canceling))
	{
		urbp = list_entry(vhc->urbp_list_canceling.next, struct usb_vhci_urb_priv, urbp_list);
		usb_vhci_maybe_set_status(urbp, -ESHUTDOWN);
		usb_vhci_urb_giveback(vhc, urbp);
	}
	spin_unlock_irqrestore(&vhc->lock, flags);

	usb_remove_hcd(hcd); // calls vhci_stop
	usb_put_hcd(hcd);
	vdev->vhc = NULL;

	if(vdev->ifc->destroy)
	{
		vhci_dbg("call ifc->destroy\n");
		vdev->ifc->destroy(vhcidev_to_ifc(vdev));
	}

	return 0;
}

static int vhci_hcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd;
	struct usb_vhci_hcd *vhc;
	int rc = 0;

	vhc = pdev_to_vhcihcd(pdev);
	hcd = vhcihcd_to_usbhcd(vhc);

	trace_function(vhcihcd_to_dev(vhc));

	if(unlikely(vhc->rh_state == USB_VHCI_RH_RUNNING))
	{
		dev_warn(&pdev->dev, "Root hub isn't suspended! You have to suspend the root hub before you suspend the host controller device.\n");
		rc = -EBUSY;
	}
	else
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	return rc;
}

static int vhci_hcd_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct usb_vhci_hcd *vhc;

	vhc = pdev_to_vhcihcd(pdev);
	hcd = vhcihcd_to_usbhcd(vhc);

	trace_function(vhcihcd_to_dev(vhc));

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);
	return 0;
}

static struct platform_driver vhci_hcd_driver = {
	.probe      = vhci_hcd_probe,
	.remove     = vhci_hcd_remove,
	.suspend    = vhci_hcd_suspend,
	.resume     = vhci_hcd_resume,
	.driver     = {
		.name   = driver_name,
		.owner  = THIS_MODULE
	}
};

// Callback function for driver_for_each_device(..) in usb_vhci_hcd_register(...).
// Data points to the device-id we're looking for.
// This funktion returns an error (-EINVAL), if the device has the given id assigned to it.
// (Enumeration stops/finishes on errors.)
static int device_enum(struct device *dev, void *data)
{
	struct platform_device *pdev;
	pdev = to_platform_device(dev);
	return unlikely(*((const int *)data) == pdev->id) ? -EINVAL : 0;
}

static DEFINE_MUTEX(dev_enum_lock);

int usb_vhci_hcd_register(const struct usb_vhci_ifc *ifc, void *context, u8 port_count, struct usb_vhci_device **vdev_ret)
{
	int retval, i;
	struct platform_device *pdev;
	struct usb_vhci_device vdev, *vdev_ptr;

	if(unlikely(port_count > 31))
		return -EINVAL;

	// search for free device-id
	mutex_lock(&dev_enum_lock);
	for(i = 0; i < 10000; i++)
	{
		retval = driver_for_each_device(&vhci_hcd_driver.driver, NULL, &i, device_enum);
		if(unlikely(!retval)) break;
	}
	if(unlikely(i >= 10000))
	{
		mutex_unlock(&dev_enum_lock);
		vhci_printk(KERN_ERR, "there are too many devices!\n");
		return -EBUSY;
	}

	vhci_dbg("allocate platform_device %s.%d\n", driver_name, i);
	pdev = platform_device_alloc(driver_name, i);
	if(unlikely(!pdev))
	{
		mutex_unlock(&dev_enum_lock);
		return -ENOMEM;
	}

	if(!try_module_get(ifc->owner))
	{
		mutex_unlock(&dev_enum_lock);
		vhci_printk(KERN_ERR, "ifc module died\n");
		retval = -ENODEV;
		goto pdev_put;
	}

	vdev.ifc = ifc;
	vdev.pdev = pdev;
	vdev.vhc = NULL;
	vdev.port_count = port_count;

	vhci_dbg("install usb_vhci_device structure within pdev->dev.platform_data\n");
	retval = platform_device_add_data(pdev, &vdev, sizeof vdev + ifc->ifc_priv_size);
	if(unlikely(retval < 0))
	{
		mutex_unlock(&dev_enum_lock);
		goto mod_put;
	}
	vdev_ptr = pdev_to_vhcidev(pdev);

	if(ifc->init)
	{
		vhci_dbg("call ifc->init\n");
		retval = ifc->init(context, vhcidev_to_ifc(vdev_ptr));
		if(unlikely(retval < 0))
		{
			mutex_unlock(&dev_enum_lock);
			goto mod_put;
		}
	}

	vhci_dbg("add platform_device %s.%d\n", pdev->name, pdev->id);
	retval = platform_device_add(pdev); // calls vhci_hcd_probe
	mutex_unlock(&dev_enum_lock);
	if(unlikely(retval < 0))
	{
		vhci_printk(KERN_ERR, "add platform_device %s.%d failed\n", pdev->name, pdev->id);
		if(ifc->destroy)
		{
			vhci_dbg("call ifc->destroy\n");
			ifc->destroy(vhcidev_to_ifc(vdev_ptr));
		}
		goto mod_put;
	}

	try_module_get(THIS_MODULE);
	*vdev_ret = vdev_ptr;
	return 0;

mod_put:
	module_put(ifc->owner);

pdev_put:
	platform_device_put(pdev);
	return retval;
}
EXPORT_SYMBOL_GPL(usb_vhci_hcd_register);

int usb_vhci_hcd_unregister(struct usb_vhci_device *vdev)
{
	struct platform_device *pdev;
	struct device *dev;
	struct module *ifc_owner = vdev->ifc->owner; // we need a copy, because vdev gets destroyed on platform_device_unregister

	pdev = vhcidev_to_pdev(vdev);
	dev = &pdev->dev;

	vhci_dbg("unregister platform_device %s\n", vhci_dev_name(dev));
	platform_device_unregister(pdev); // calls vhci_hcd_remove which calls ifc->destroy

	module_put(ifc_owner);
	module_put(THIS_MODULE);
	return 0;
}
EXPORT_SYMBOL_GPL(usb_vhci_hcd_unregister);

int usb_vhci_hcd_has_work(struct usb_vhci_hcd *vhc)
{
	unsigned long flags;
	int y = 0;
	spin_lock_irqsave(&vhc->lock, flags);
	if(vhc->port_update ||
	   !list_empty(&vhc->urbp_list_cancel) ||
	   !list_empty(&vhc->urbp_list_inbox))
		y = 1;
	spin_unlock_irqrestore(&vhc->lock, flags);
	return y;
}
EXPORT_SYMBOL_GPL(usb_vhci_hcd_has_work);

int usb_vhci_apply_port_stat(struct usb_vhci_hcd *vhc, u16 status, u16 change, u8 index)
{
	struct device *dev;
	unsigned long flags;
	u16 overcurrent;

	dev = vhcihcd_to_dev(vhc);

	if(unlikely(!index || index > vhc->port_count))
		return -EINVAL;

	if(unlikely(change != USB_PORT_STAT_C_CONNECTION &&
	            change != USB_PORT_STAT_C_ENABLE &&
	            change != USB_PORT_STAT_C_SUSPEND &&
	            change != USB_PORT_STAT_C_OVERCURRENT &&
	            change != USB_PORT_STAT_C_RESET &&
	            change != (USB_PORT_STAT_C_RESET | USB_PORT_STAT_C_ENABLE)))
		return -EINVAL;

	spin_lock_irqsave(&vhc->lock, flags);
	if(unlikely(!(vhc->ports[index - 1].port_status & USB_PORT_STAT_POWER)))
	{
		spin_unlock_irqrestore(&vhc->lock, flags);
		return -EPROTO;
	}

#ifdef DEBUG
	if(debug_output) dev_dbg(dev, "performing PORT_STAT [port=%d ~status=0x%04x ~change=0x%04x]\n", (int)index, (int)status, (int)change);
#endif

	switch(change)
	{
	case USB_PORT_STAT_C_CONNECTION:
		overcurrent = vhc->ports[index - 1].port_status & USB_PORT_STAT_OVERCURRENT;
		vhc->ports[index - 1].port_change |= USB_PORT_STAT_C_CONNECTION;
		if(status & USB_PORT_STAT_CONNECTION)
			vhc->ports[index - 1].port_status = USB_PORT_STAT_POWER | USB_PORT_STAT_CONNECTION |
				((status & USB_PORT_STAT_LOW_SPEED) ? USB_PORT_STAT_LOW_SPEED :
				((status & USB_PORT_STAT_HIGH_SPEED) ? USB_PORT_STAT_HIGH_SPEED : 0)) |
				overcurrent;
		else
			vhc->ports[index - 1].port_status = USB_PORT_STAT_POWER | overcurrent;
		vhc->ports[index - 1].port_flags &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;
		break;

	case USB_PORT_STAT_C_ENABLE:
		if(unlikely(!(vhc->ports[index - 1].port_status & USB_PORT_STAT_CONNECTION) ||
			(vhc->ports[index - 1].port_status & USB_PORT_STAT_RESET) ||
			(status & USB_PORT_STAT_ENABLE)))
		{
			spin_unlock_irqrestore(&vhc->lock, flags);
			return -EPROTO;
		}
		vhc->ports[index - 1].port_change |= USB_PORT_STAT_C_ENABLE;
		vhc->ports[index - 1].port_status &= ~USB_PORT_STAT_ENABLE;
		vhc->ports[index - 1].port_flags &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;
		vhc->ports[index - 1].port_status &= ~USB_PORT_STAT_SUSPEND;
		break;

	case USB_PORT_STAT_C_SUSPEND:
		if(unlikely(!(vhc->ports[index - 1].port_status & USB_PORT_STAT_CONNECTION) ||
			!(vhc->ports[index - 1].port_status & USB_PORT_STAT_ENABLE) ||
			(vhc->ports[index - 1].port_status & USB_PORT_STAT_RESET) ||
			(status & USB_PORT_STAT_SUSPEND)))
		{
			spin_unlock_irqrestore(&vhc->lock, flags);
			return -EPROTO;
		}
		vhc->ports[index - 1].port_flags &= ~USB_VHCI_PORT_STAT_FLAG_RESUMING;
		vhc->ports[index - 1].port_change |= USB_PORT_STAT_C_SUSPEND;
		vhc->ports[index - 1].port_status &= ~USB_PORT_STAT_SUSPEND;
		break;

	case USB_PORT_STAT_C_OVERCURRENT:
		vhc->ports[index - 1].port_change |= USB_PORT_STAT_C_OVERCURRENT;
		vhc->ports[index - 1].port_status &= ~USB_PORT_STAT_OVERCURRENT;
		vhc->ports[index - 1].port_status |= status & USB_PORT_STAT_OVERCURRENT;
		break;

	default: // USB_PORT_STAT_C_RESET [| USB_PORT_STAT_C_ENABLE]
		if(unlikely(!(vhc->ports[index - 1].port_status & USB_PORT_STAT_CONNECTION) ||
			!(vhc->ports[index - 1].port_status & USB_PORT_STAT_RESET) ||
			(status & USB_PORT_STAT_RESET)))
		{
			spin_unlock_irqrestore(&vhc->lock, flags);
			return -EPROTO;
		}
		if(change & USB_PORT_STAT_C_ENABLE)
		{
			if(status & USB_PORT_STAT_ENABLE)
			{
				spin_unlock_irqrestore(&vhc->lock, flags);
				return -EPROTO;
			}
			vhc->ports[index - 1].port_change |= USB_PORT_STAT_C_ENABLE;
		}
		else
			vhc->ports[index - 1].port_status |= status & USB_PORT_STAT_ENABLE;
		vhc->ports[index - 1].port_change |= USB_PORT_STAT_C_RESET;
		vhc->ports[index - 1].port_status &= ~USB_PORT_STAT_RESET;
		break;
	}

	vhci_port_update(vhc, index);
	spin_unlock_irqrestore(&vhc->lock, flags);

	usb_hcd_poll_rh_status(vhcihcd_to_usbhcd(vhc));
	return 0;
}
EXPORT_SYMBOL_GPL(usb_vhci_apply_port_stat);

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

static int __init init(void)
{
	int retval;

	if(usb_disabled()) return -ENODEV;

	vhci_printk(KERN_INFO, DRIVER_DESC " -- Version " DRIVER_VERSION "\n");

#ifdef DEBUG
	vhci_printk(KERN_DEBUG, "register platform_driver %s\n", driver_name);
#endif
	retval = platform_driver_register(&vhci_hcd_driver);
	if(unlikely(retval < 0))
	{
		vhci_printk(KERN_ERR, "register platform_driver failed\n");
		return retval;
	}

#ifdef DEBUG
	retval = driver_create_file(&vhci_hcd_driver.driver, &driver_attr_debug_output);
	if(unlikely(retval != 0))
	{
		vhci_printk(KERN_DEBUG, "driver_create_file(&vhci_hcd_driver, &driver_attr_debug_output) failed\n");
		vhci_printk(KERN_DEBUG, "==> ignoring\n");
	}
#endif

	return 0;
}
module_init(init);

static void __exit cleanup(void)
{
#ifdef DEBUG
	driver_remove_file(&vhci_hcd_driver.driver, &driver_attr_debug_output);
#endif
	vhci_dbg("unregister platform_driver %s\n", driver_name);
	platform_driver_unregister(&vhci_hcd_driver);
	vhci_dbg("gone\n");
}
module_exit(cleanup);
