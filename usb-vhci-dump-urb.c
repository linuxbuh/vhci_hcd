// this is just for debugging

static const char *get_status_str(int status)
{
	switch(status)
	{
	case 0:            return "SUCCESS";
	case -EINPROGRESS: return "-EINPROGRESS";
	case -EREMOTEIO:   return "-EREMOTEIO [short packet]";
	case -ENOENT:      return "-ENOENT [sync cancel]";
	case -ECONNRESET:  return "-ECONNRESET [async cancel]";
	case -ETIMEDOUT:   return "-ETIMEDOUT";
	case -ESHUTDOWN:   return "-ESHUTDOWN [dev disabled]";
	case -ENODEV:      return "-ENODEV [dev disconn]";
	case -EPROTO:      return "-EPROTO [bit stuff]";
	case -EILSEQ:      return "-EILSEQ [crc]";
	case -ETIME:       return "-ETIME [no response]";
	case -EOVERFLOW:   return "-EOVERFLOW [babble]";
	case -EPIPE:       return "-EPIPE [stall]";
	case -ECOMM:       return "-ECOMM [buf overrun]";
	case -ENOSR:       return "-ENOSR [buf underrun]";
	case -EXDEV:       return "-EXDEV [see iso-pkt status]";
	case -EINVAL:      return "-EINVAL [all iso-pkts failed]";
	default:           return "???";
	}
}

static void dump_urb(struct urb *urb)
{
	int i, j;
	int max = urb->transfer_buffer_length;
	int in = usb_pipein(urb->pipe);
	if(!debug_output) return;
	vhci_printk(KERN_DEBUG, "dump_urb 0x%016llx:\n", (u64)(unsigned long)urb);
	vhci_printk(KERN_DEBUG, "dvadr=0x%02x epnum=%d epdir=%s eptpe=%s\n", (int)usb_pipedevice(urb->pipe), (int)usb_pipeendpoint(urb->pipe), (in ? "IN" : "OUT"), (usb_pipecontrol(urb->pipe) ? "CTRL" : (usb_pipebulk(urb->pipe) ? "BULK" : (usb_pipeint(urb->pipe) ? "INT" : (usb_pipeisoc(urb->pipe) ? "ISO" : "INV!")))));
#ifdef OLD_GIVEBACK_MECH
	vhci_printk(KERN_DEBUG, "status=%d(%s) flags=0x%08x buflen=%d/%d\n", urb->status, get_status_str(urb->status), urb->transfer_flags, urb->actual_length, max);
#else
	vhci_printk(KERN_DEBUG, "flags=0x%08x buflen=%d/%d\n", urb->transfer_flags, urb->actual_length, max);
#endif
	vhci_printk(KERN_DEBUG, "tbuf=0x%p tdma=0x%016llx sbuf=0x%p sdma=0x%016llx\n", urb->transfer_buffer, (u64)urb->transfer_dma, urb->setup_packet, (u64)urb->setup_dma);
	if(usb_pipeint(urb->pipe))
		vhci_printk(KERN_DEBUG, "interval=%d\n", urb->interval);
	else if(usb_pipeisoc(urb->pipe))
		vhci_printk(KERN_DEBUG, "interval=%d err=%d packets=%d startfrm=%d\n", urb->interval, urb->error_count, urb->number_of_packets, urb->start_frame);
	else if(usb_pipecontrol(urb->pipe))
	{
		const char *const sr[13] =
		{
			"GET_STATUS",
			"CLEAR_FEATURE",
			"reserved",
			"SET_FEATURE",
			"reserved",
			"SET_ADDRESS",
			"GET_DESCRIPTOR",
			"SET_DESCRIPTOR",
			"GET_CONFIGURATION",
			"SET_CONFIGURATION",
			"GET_INTERFACE",
			"SET_INTERFACE",
			"SYNCH_FRAME"
		};
		const char *const sd[9] =
		{
			"invalid",
			"DEVICE",
			"CONFIGURATION",
			"STRING",
			"INTERFACE",
			"ENDPOINT",
			"DEVICE_QUALIFIER",
			"OTHER_SPEED_CONFIGURATION",
			"INTERFACE_POWER"
		};
		const char *const sf[3] =
		{
			"ENDPOINT_HALT",
			"DEVICE_REMOTE_WAKEUP",
			"TEST_MODE"
		};
		max = urb->setup_packet[6] | (urb->setup_packet[7] << 8);
		in = urb->setup_packet[0] & 0x80;
		if(urb->setup_packet == NULL)
			vhci_printk(KERN_DEBUG, "(!!!) setup_packet is NULL\n");
		else
		{
			unsigned int val = urb->setup_packet[2] | (urb->setup_packet[3] << 8);
			vhci_printk(KERN_DEBUG, "bRequestType=0x%02x(%s,%s,%s) bRequest=0x%02x(%s)\n",
				(int)urb->setup_packet[0],
				in ? "IN" : "OUT",
				(((urb->setup_packet[0] >> 5) & 0x03) == 0) ? "STD" : ((((urb->setup_packet[0] >> 5) & 0x03) == 1) ? "CLS" : ((((urb->setup_packet[0] >> 5) & 0x03) == 2) ? "VDR" : "???")),
				((urb->setup_packet[0] & 0x1f) == 0) ? "DV" : (((urb->setup_packet[0] & 0x1f) == 1) ? "IF" : (((urb->setup_packet[0] & 0x1f) == 2) ? "EP" : (((urb->setup_packet[0] & 0x1f) == 3) ? "OT" : "??"))),
				(int)urb->setup_packet[1],
				(((urb->setup_packet[0] >> 5) & 0x03) == 0 && urb->setup_packet[1] < 13) ? sr[urb->setup_packet[1]] : "???");
			vhci_printk(KERN_DEBUG, "wValue=0x%04x", val);
			if(((urb->setup_packet[0] >> 5) & 0x03) == 0)
			{
				if(urb->setup_packet[1] == 1 || urb->setup_packet[1] == 3)
					printk("(%s)", (val < 3) ? sf[val] : "???");
				else if(urb->setup_packet[1] == 6 || urb->setup_packet[1] == 7)
					printk("(%s)", (urb->setup_packet[3] < 9) ? sd[urb->setup_packet[3]] : "???");
			}
			printk(" wIndex=0x%04x wLength=0x%04x\n", urb->setup_packet[4] | (urb->setup_packet[5] << 8), max);
		}
	}
	if(usb_pipeisoc(urb->pipe))
	{
		for(j = 0; j < urb->number_of_packets; j++)
		{
			vhci_printk(KERN_DEBUG, "PACKET%d: offset=%d pktlen=%d/%d status=%d(%s)\n", j, urb->iso_frame_desc[j].offset, urb->iso_frame_desc[j].actual_length, urb->iso_frame_desc[j].length, urb->iso_frame_desc[j].status, get_status_str(urb->iso_frame_desc[j].status));
			if(debug_output >= 2)
			{
				vhci_printk(KERN_DEBUG, "PACKET%d: data stage (%d/%d bytes %s):\n", j, urb->iso_frame_desc[j].actual_length, urb->iso_frame_desc[j].length, in ? "received" : "transmitted");
				vhci_printk(KERN_DEBUG, "PACKET%d: ", j);
				max = in ? urb->iso_frame_desc[j].actual_length : urb->iso_frame_desc[j].length;
				if(debug_output > 2 || max <= 16)
					for(i = urb->iso_frame_desc[j].offset; i < max + urb->iso_frame_desc[j].offset; i++)
						printk("%02x ", (unsigned int)((unsigned char*)urb->transfer_buffer)[i]);
				else
				{
					for(i = urb->iso_frame_desc[j].offset; i < 8 + urb->iso_frame_desc[j].offset; i++)
						printk("%02x ", (unsigned int)((unsigned char*)urb->transfer_buffer)[i]);
					printk("... ");
					for(i = max + urb->iso_frame_desc[j].offset - 8; i < max + urb->iso_frame_desc[j].offset; i++)
						printk("%02x ", (unsigned int)((unsigned char*)urb->transfer_buffer)[i]);
				}
				printk("\n");
			}
		}
	}
	else if(debug_output >= 2)
	{
		vhci_printk(KERN_DEBUG, "data stage (%d/%d bytes %s):\n", urb->actual_length, max, in ? "received" : "transmitted");
		vhci_printk(KERN_DEBUG, "");
		if(in) max = urb->actual_length;
		if(debug_output > 2 || max <= 16)
			for(i = 0; i < max; i++)
				printk("%02x ", (unsigned int)((unsigned char*)urb->transfer_buffer)[i]);
		else
		{
			for(i = 0; i < 8; i++)
				printk("%02x ", (unsigned int)((unsigned char*)urb->transfer_buffer)[i]);
			printk("... ");
			for(i = max - 8; i < max; i++)
				printk("%02x ", (unsigned int)((unsigned char*)urb->transfer_buffer)[i]);
		}
		printk("\n");
	}
}
