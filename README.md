# vhci_hcd

клон https://sourceforge.net/projects/usb-vhci/

vhci_hcd
This project consists of a Linux kernel driver and some user-mode libraries. They allow a process to create a virtual usb host controller. Real or virtual usb devices can be "plugged" into this controller.
Screenshot thumbnail
usb storage device forwarded to a virtual host controller

Install as modules
==================

Just run

  mkdir -p linux/"$(uname -r | cut -d'-' -f1)"/drivers/usb/core

  cp /usr/src/linux-source-"$(uname -r | cut -d'-' -f1)"/include/linux/usb/hcd.h linux/"$(uname -r | cut -d'-' -f1)"/drivers/usb/core/

  make KVERSION="$(uname -r)" KSRC=/usr/src/linux-source-"$(uname -r | cut -d'-' -f1)"

or

  make

and

  make install   # (as root)

to build and install the modules for the currently running kernel.
The modules are called usb-vhci-hcd and usb-vhci-iocifc. Run

  modprobe usb-vhci-hcd &&
  modprobe usb-vhci-iocifc

to load the modules.



Patch into kernel
=================

Run

  make KVERSION=<VERSION> KSRC=<PATH_TO_KERNEL_SOURCE> patchkernel

(replace <PATH_TO_KERNEL_SOURCE> with the actual path) to patch the vhci-hcd
sources into the kernel source. If you want to do this for a kernel with a
different version than the currently running one, then you need to create
the config header first by running

  make config

and answering the few questions about the target kernel.




Project Members:
Bruse Geng
Emil Lee
m_singer (admin)
About this project
I started working on this project in the year 2007 as an employee of the company Conemis AG in Karlsruhe.

The primary goal was to share USB devices over ethernet. At this point in time it works fine with Linux. The Windows kernel driver isn't finished yet.

The user space part is written in C# and runs on Mono and MS.NET. It consists of some libraries and a few executables (server and client).

How to use it
First you need to download and build the Linux kernel driver. You can download it from the Files tab. It's in the linux kernel module folder. You can also clone the vhci_hcd GIT repository:

git clone git://git.code.sf.net/p/usb-vhci/vhci_hcd
"make" should be enough to build it. When it is build successfully, you can load the modules with:

insmod usb-vhci-hcd.ko
insmod usb-vhci-iocifc.ko
The first module is the virtual host controller driver itself. The second module is the IOCTL interface to user mode. After loading the second module the device file /dev/usb-vhci should appear. All the communication between kernel driver and user mode happens through this file. If you don't want to be root all the time, you might want to chmod it:

chmod 666 /dev/usb-vhci
There are two user mode packages you can use now:

The native C/C++ library and
the .NET libraries
Currently, the .NET libraries offer much more stuff. For example they include a client and a server application for "tunneling" USB devices over TCP.

Native C/C++ library
The native library can be downloaded from the native libraries folder. Or you can clone the libusb_vhci repository:

git clone git://git.code.sf.net/p/usb-vhci/libusb_vhci
Before you can build it, you need the usb-vhci.h header file from the kernel driver installed in /usr/include/linux (or maybe /usr/local/include/linux might work too). You can run "make install" in the top level directory of the driver sources (vhci_hcd) that you downloaded and built earlier. It will install this header and the modules.

With the header file in place, you should be able to build the library and a few example applications:

./configure
make
To see if it works, run one of the examples:

examples/virtual_device
And use another terminal to look for the virtual USB device with "dmesg" or "cat /proc/bus/usb/devices" if you have usbfs mounted there.

.NET libraries
The .NET libraries can be downloaded from the dotnet libraries folder. Or you can clone the usb-vhci repository:

git clone git://git.code.sf.net/p/usb-vhci/usb-vhci
They can also be downloaded pre-built. Of course you need mono for building and using them. You can also build them in a mingw or cygwin console on Windows using the Microsoft C# compiler if you like.

If you want to build them on Linux (Again: You don't have to. There are pre-built ones available.), it should work just like the native libraries:

./configure
make
Building them on Windows (using mingw or cygwin), you might need to find out the full path to the Microsoft #C compiler and run configure like this:

./configure CSC=/path/to/compiler.exe
When they got built successfully, all DLLs and executables are in the bin folder. You can start the server now:

mono bin/VhciServer.exe 4 1138
This creates a virtual USB host controller with four USB ports. The server listens on TCP port 1138 for connections. Use "dmesg" to see the new virtual host controller.

Now you can start the client on another computer (or the same one), to connect a USB device to this virtual host controller. But first you have to search for the device you want to connect:

mono bin/VhciClient.exe -l
The parameter -l shows you a tree of all USB ports and which devices are connected to them. Choose a device and start the client like this:

mono bin/VhciClient.exe 4-3 127.0.0.1:1138
If you are not root, then you might run into an exception that looks like this:

Unhandled Exception:
System.IO.IOException: Couldn't open /dev/bus/usb/004/002. (13)
  at Usb.UsbFS.DeviceFile..ctor (System.String path, Boolean ro) [0x00000] in <filename unknown>:0
  at Usb.UsbFS.DeviceFile..ctor (System.String path) [0x00000] in <filename unknown>:0
  at Usb.UsbFS.UsbDevice..ctor (System.String path) [0x00000] in <filename unknown>:0
  at Usb.UsbFS.Enumeration.Device.Acquire () [0x00000] in <filename unknown>:0
  at VhciClient.MainClass.Main (System.String[] args) [0x00000] in <filename unknown>:0
If you don't want to start the client as root, use chmod as root to give yourself access of the device file. In this example:

chmod 666 /dev/bus/usb/004/002
