/*
 * test.c -- testing the ability to build kernel modules
 *
 * Copyright (C) 2009 Michael Singer <michael@a-singer.de>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/device.h>
#ifdef KBUILD_EXTMOD
#	include "../usb-vhci.h"
#else
#	include <linux/usb-vhci.h>
#endif

#include <asm/atomic.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>

#ifdef KBUILD_EXTMOD
#	include INCLUDE_CORE_HCD
#else
#	include "../core/hcd.h"
#endif

#define DRIVER_DESC "test"
MODULE_DESCRIPTION(DRIVER_DESC " driver");
MODULE_AUTHOR("Michael Singer <michael@a-singer.de>");
MODULE_LICENSE("GPL");

static struct device testdev = {
	.class = NULL,
	.release = NULL,
#ifdef TEST_DEV_INIT_NAME
	.init_name = "test",
#endif
	.driver = NULL
};

#ifdef TEST_HAS_TT_FLAG
static struct usb_hcd testhcd = {
	.has_tt = 1
};
#endif

static int __init init(void)
{
	if(usb_disabled()) return -ENODEV;

#ifdef TEST_GIVEBACK_MECH
	usb_hcd_giveback_urb((struct usb_hcd *)NULL, (struct urb *)NULL, (int)0);
#endif

#ifdef TEST_DEV_BUS_ID
	const char * foo = dev_name((struct device *)NULL);
	dev_set_name((struct device *)NULL, foo);
#endif

	return 0;
}
module_init(init);

static void __exit cleanup(void)
{
}
module_exit(cleanup);

