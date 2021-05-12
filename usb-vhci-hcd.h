/*
 * usb-vhci-hcd.h -- VHCI USB host controller driver header.
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

#ifndef _USB_VHCI_HCD_H
#define _USB_VHCI_HCD_H

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/device.h>

#include <asm/atomic.h>

#ifdef KBUILD_EXTMOD
#	include INCLUDE_CORE_HCD
#else
#	include <linux/usb/hcd.h>
#endif

// this is undefined in linux >= 2.6.35
#ifndef USB_PORT_FEAT_HIGHSPEED
#	define USB_PORT_FEAT_HIGHSPEED (10)
#endif

#ifdef KBUILD_EXTMOD
#	include "usb-vhci.h"
#	include "conf/usb-vhci.config.h"
#else
#	include <linux/usb-vhci.h>
#	include "usb-vhci.config.h"
#endif

struct usb_vhci_port
{
	u16 port_status;
	u16 port_change;
	u8 port_flags;
};

enum usb_vhci_rh_state
{
	USB_VHCI_RH_RESET     = 0,
	USB_VHCI_RH_SUSPENDED = 1,
	USB_VHCI_RH_RUNNING   = 2
} __attribute__((packed));

struct usb_vhci_device;

struct usb_vhci_ifc
{
	const char *ifc_desc;
	struct module *owner;
	size_t ifc_priv_size;

	// callbacks for backend drivers
	int (*init)(void *context, void *ifc_priv);
	void (*destroy)(void *ifc_priv);
	void (*wakeup)(struct usb_vhci_device *vdev);
};

struct usb_vhci_device
{
	const struct usb_vhci_ifc *ifc;
	struct platform_device *pdev;
	struct usb_vhci_hcd *vhc;

	u8 port_count;

	// private data for backend drivers
	unsigned long ifc_priv[0] __attribute__((aligned(sizeof(unsigned long))));
};

struct usb_vhci_urb_priv
{
	struct urb *urb;
	struct list_head urbp_list;
	atomic_t status;
};

struct usb_vhci_hcd
{
	struct usb_vhci_port *ports;
	u32 port_update;

	spinlock_t lock;

	atomic_t frame_num;
	enum usb_vhci_rh_state rh_state;

	// TODO: implement timer for incrementing frame_num every millisecond
	//struct timer_list timer;

	// urbs which are waiting to get fetched by user space are in this list
	struct list_head urbp_list_inbox;

	// urbs which were fetched by user space but not already given back are in this list
	struct list_head urbp_list_fetched;

	// urbs which were fetched by user space and not already given back, and which should be
	// canceled are in this list
	struct list_head urbp_list_cancel;

	// urbs which were fetched by user space and not already given back, and for which the
	// user space already knows about the cancelation state are in this list
	struct list_head urbp_list_canceling;

	u8 port_count;
};

static inline struct usb_vhci_device *pdev_to_vhcidev(struct platform_device *pdev)
{
	return pdev->dev.platform_data;
}

static inline void *vhcidev_to_ifc(struct usb_vhci_device *vdev)
{
	return &vdev->ifc_priv;
}

static inline struct usb_vhci_device *ifc_to_vhcidev(void *ifc)
{
	return container_of(ifc, struct usb_vhci_device, ifc_priv);
}

static inline struct usb_vhci_hcd *vhcidev_to_vhcihcd(struct usb_vhci_device *vdev)
{
	return vdev->vhc;
}

static inline struct platform_device *vhcidev_to_pdev(struct usb_vhci_device *vdev)
{
	return vdev->pdev;
}

static inline struct usb_hcd *vhcihcd_to_usbhcd(struct usb_vhci_hcd *vhc)
{
	return container_of((void *)vhc, struct usb_hcd, hcd_priv);
}

static inline struct usb_hcd *vhcidev_to_usbhcd(struct usb_vhci_device *vdev)
{
	return vhcihcd_to_usbhcd(vhcidev_to_vhcihcd(vdev));
}

static inline struct usb_vhci_hcd *usbhcd_to_vhcihcd(struct usb_hcd *hcd)
{
	return (struct usb_vhci_hcd *)&hcd->hcd_priv;
}

static inline struct device *usbhcd_to_dev(struct usb_hcd *hcd)
{
	return hcd->self.controller;
}

static inline struct device *vhcihcd_to_dev(struct usb_vhci_hcd *vhc)
{
	return usbhcd_to_dev(vhcihcd_to_usbhcd(vhc));
}

static inline struct device *vhcidev_to_dev(struct usb_vhci_device *vdev)
{
	return usbhcd_to_dev(vhcidev_to_usbhcd(vdev));
}

static inline struct platform_device *vhcihcd_to_pdev(struct usb_vhci_hcd *vhc)
{
	return to_platform_device(vhcihcd_to_dev(vhc));
}

static inline struct usb_vhci_device *vhcihcd_to_vhcidev(struct usb_vhci_hcd *vhc)
{
	return pdev_to_vhcidev(vhcihcd_to_pdev(vhc));
}

static inline struct usb_vhci_hcd *pdev_to_vhcihcd(struct platform_device *pdev)
{
	return vhcidev_to_vhcihcd(pdev_to_vhcidev(pdev));
}

static inline struct usb_hcd *pdev_to_usbhcd(struct platform_device *pdev)
{
	return vhcidev_to_usbhcd(pdev_to_vhcidev(pdev));
}

const char *usb_vhci_dev_name(struct usb_vhci_device *vdev);
int usb_vhci_dev_id(struct usb_vhci_device *vdev);
int usb_vhci_dev_busnum(struct usb_vhci_device *vdev);
void usb_vhci_maybe_set_status(struct usb_vhci_urb_priv *urbp, int status);
void usb_vhci_urb_giveback(struct usb_vhci_hcd *vhc, struct usb_vhci_urb_priv *urbp);
int usb_vhci_hcd_register(const struct usb_vhci_ifc *ifc, void *context, u8 port_count, struct usb_vhci_device **vdev_ret);
int usb_vhci_hcd_unregister(struct usb_vhci_device *vdev);
int usb_vhci_hcd_has_work(struct usb_vhci_hcd *vhc);
int usb_vhci_apply_port_stat(struct usb_vhci_hcd *vhc, u16 status, u16 change, u8 index);

#endif
