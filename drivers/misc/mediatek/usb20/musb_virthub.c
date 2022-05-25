// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <asm/unaligned.h>

#include "musb_core.h"

static int h_pre_disable = 1;
module_param(h_pre_disable, int, 0644);

static void musb_host_check_disconnect(struct musb *musb)
{
	u8 opstate = musb_readb(musb->mregs, MUSB_OPSTATE);
	bool is_con = musb->port1_status & USB_PORT_STAT_CONNECTION;

	if (opstate == MUSB_OPSTATE_HOST_WAIT_DEV && is_con) {
		DBG(0, "disconnect when suspend");
		musb->int_usb |= MUSB_INTR_DISCONNECT;
		musb->xceiv->otg->state = OTG_STATE_A_HOST;
		musb_interrupt(musb);
	}
}

int musb_port_suspend(struct musb *musb, bool do_suspend)
{
	struct usb_otg *otg = musb->xceiv->otg;
	u8 power;
	void __iomem *mbase = musb->mregs;

	if (!is_host_active(musb))
		return 0;

	DBG(0, "%s\n", do_suspend ? "suspend" : "resume");

	/* NOTE:  this doesn't necessarily put PHY into low power mode,
	 * turning off its clock; that's a function of PHY integration and
	 * MUSB_POWER_ENSUSPEND.  PHY may need a clock (sigh) to detect
	 * SE0 changing to connect (J) or wakeup (K) states.
	 */
	power = musb_readb(mbase, MUSB_POWER);
	if (do_suspend) {
		int retries = 10000;

		if (power & MUSB_POWER_RESUME)
			return -EBUSY;

		if (!(power & MUSB_POWER_SUSPENDM)) {
			power |= MUSB_POWER_SUSPENDM;
			musb_writeb(mbase, MUSB_POWER, power);

			/* Needed for OPT A tests */
			power = musb_readb(mbase, MUSB_POWER);
			while (power & MUSB_POWER_SUSPENDM) {
				power = musb_readb(mbase, MUSB_POWER);
				if (retries-- < 1)
					break;
			}
		}

		DBG(3, "Root port suspended, power %02x\n", power);

		musb->port1_status |= USB_PORT_STAT_SUSPEND;
		switch (musb->xceiv->otg->state) {
		case OTG_STATE_A_HOST:
			musb->xceiv->otg->state = OTG_STATE_A_SUSPEND;
			musb->is_active = otg->host->b_hnp_enable;
			if (musb->is_active)
				mod_timer(&musb->otg_timer, jiffies
				+ msecs_to_jiffies(OTG_TIME_A_AIDL_BDIS));
			musb_platform_try_idle(musb, 0);
			break;
		case OTG_STATE_B_HOST:
			musb->xceiv->otg->state = OTG_STATE_B_WAIT_ACON;
			musb->is_active = otg->host->b_hnp_enable;
			musb_platform_try_idle(musb, 0);
			break;
		default:
			DBG(0, "bogus rh suspend? %s\n",
				otg_state_string(musb->xceiv->otg->state));
		}
	} else if (power & MUSB_POWER_SUSPENDM) {
		power &= ~MUSB_POWER_SUSPENDM;
		power |= MUSB_POWER_RESUME;
		musb_writeb(mbase, MUSB_POWER, power);

		DBG(3, "Root port resuming, power %02x\n", power);

		/* later, GetPortStatus will stop RESUME signaling */
		musb->port1_status |= MUSB_PORT_STAT_RESUME;
		musb->rh_timer = jiffies + msecs_to_jiffies(20);
	}
	return 0;
}

static void musb_port_reset(struct musb *musb, bool do_reset)
{
	u8 power;
	void __iomem *mbase = musb->mregs;

	if (musb->xceiv->otg->state == OTG_STATE_B_IDLE) {
		DBG(2, "HNP: Returning from HNP; no hub reset from b_idle\n");
		musb->port1_status &= ~USB_PORT_STAT_RESET;
		return;
	}

	if (!is_host_active(musb))
		return;

	/* NOTE:  caller guarantees it will turn off the reset when
	 * the appropriate amount of time has passed
	 */
	power = musb_readb(mbase, MUSB_POWER);
	if (do_reset) {
		DBG(0, "force musb_platform_reset\n");
		musb_platform_reset(musb);
		mdelay(3);

		/*
		 * If RESUME is set, we must make sure it stays minimum 20 ms.
		 * Then we must clear RESUME and wait a bit to let musb start
		 * generating SOFs. If we don't do this, OPT HS A 6.8 tests
		 * fail with "Error! Did not receive an SOF before suspend
		 * detected".
		 */
		if (power & MUSB_POWER_RESUME) {
			while (time_before(jiffies, musb->rh_timer))
				mdelay(1);
			musb_writeb(mbase,
				MUSB_POWER, power & ~MUSB_POWER_RESUME);
			mdelay(1);
		}

		musb->ignore_disconnect = true;
		power &= 0xf0;
		musb_writeb(mbase, MUSB_POWER, power | MUSB_POWER_RESET);

		musb->port1_status |= USB_PORT_STAT_RESET;
		musb->port1_status &= ~USB_PORT_STAT_ENABLE;
		musb->rh_timer = jiffies + msecs_to_jiffies(50);
	} else {
		DBG(4, "root port reset stopped\n");
		musb_writeb(mbase, MUSB_POWER, power & ~MUSB_POWER_RESET);

		musb->ignore_disconnect = false;

		power = musb_readb(mbase, MUSB_POWER);
		if (power & MUSB_POWER_HSMODE) {
			DBG(4, "high-speed device connected\n");
			musb->port1_status |= USB_PORT_STAT_HIGH_SPEED;
		}

		musb->port1_status &= ~USB_PORT_STAT_RESET;
		musb->port1_status |= USB_PORT_STAT_ENABLE
			| (USB_PORT_STAT_C_RESET << 16)
		    | (USB_PORT_STAT_C_ENABLE << 16);
		usb_hcd_poll_rh_status(musb_to_hcd(musb));

		musb->vbuserr_retry = VBUSERR_RETRY_COUNT;
	}
}

void musb_root_disconnect(struct musb *musb)
{
	struct usb_otg *otg = musb->xceiv->otg;

	musb->port1_status = USB_PORT_STAT_POWER
					| (USB_PORT_STAT_C_CONNECTION << 16);

	usb_hcd_poll_rh_status(musb_to_hcd(musb));
	musb->is_active = 0;

	if (h_pre_disable) {
	/* when UMS device is detached,
	 * khubd need to wait for usb-storage
	 * thread to stop, then it will disable all endpoints,
	 * and clean up pending
	 * URBs. But if usb-storage is waiting for some URBs,
	 * it will never stop.
	 * So there is a dead lock:
	 * khubd need to end usb-storage then flush URB,
	 * but usb-storage need that URB to end itself.
	 * So we flush URB here first,
	 * this will cause usb-storage quit waiting and end
	 * itself when khubd asks.
	 */
		spin_unlock(&musb->lock);
		musb_h_pre_disable(musb);
		spin_lock(&musb->lock);
	} else
		DBG(0, "SKIP musb_h_pre_disable\n");

	DBG(0, "host disconnect (%s)\n",
		otg_state_string(musb->xceiv->otg->state));

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_SUSPEND:
		if (otg->host->b_hnp_enable) {
			musb->xceiv->otg->state = OTG_STATE_A_PERIPHERAL;
			musb->g.is_a_peripheral = 1;
			break;
		}
		fallthrough;
	case OTG_STATE_A_HOST:
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_BCON;
		musb->is_active = 0;
		break;
	case OTG_STATE_A_WAIT_VFALL:
		musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		break;
	default:
		DBG(0, "host disconnect (%s)\n",
			otg_state_string(musb->xceiv->otg->state));
	}
}
EXPORT_SYMBOL(musb_root_disconnect);

/*---------------------------------------------------------------------*/

/* Caller may or may not hold musb->lock */
int musb_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct musb *musb = hcd_to_musb(hcd);
	int retval = 0;

	/* called in_irq() via usb_hcd_poll_rh_status() */
	if (musb->port1_status & 0xffff0000) {
		*buf = 0x02;
		retval = 1;
	}
	return retval;
}

int musb_hub_control(struct usb_hcd *hcd,
	u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct musb *musb = hcd_to_musb(hcd);
	u32 temp;
	int retval = 0;
	unsigned long flags;
	#if IS_ENABLED(CONFIG_MTK_MUSB_PORT0_LOWPOWER_MODE)
	bool usb_active = true;

	if (!mtk_usb_power) {
		musb_platform_enable(musb);
		usb_active = false;
		DBG(1, "musb was in-active!!!\n");
	}
	#endif

	musb_platform_prepare_clk(musb);

	spin_lock_irqsave(&musb->lock, flags);

	if (unlikely(!HCD_HW_ACCESSIBLE(hcd))) {
		spin_unlock_irqrestore(&musb->lock, flags);
		return -ESHUTDOWN;
	}

	/* hub features:  always zero, setting is a NOP
	 * port features: reported, sometimes updated when host is active
	 * no indicators
	 */
	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		if ((wIndex & 0xff) != 1)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			break;
		case USB_PORT_FEAT_SUSPEND:
			musb_port_suspend(musb, false);
			break;
		case USB_PORT_FEAT_POWER:
			if (!hcd->self.is_b_host)
				musb_platform_set_vbus(musb, 0);
			break;
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_SUSPEND:
			break;
		default:
			goto error;
		}
		DBG(5, "clear feature %d\n", wValue);
		musb->port1_status &= ~(1 << wValue);
		break;
	case GetHubDescriptor:
		{
			struct usb_hub_descriptor *desc = (void *)buf;

			desc->bDescLength = 9;
			desc->bDescriptorType = 0x29;
			desc->bNbrPorts = 1;
			desc->wHubCharacteristics = cpu_to_le16(0x0001
				/* per-port power switching */
						| 0x0010
				/* no overcurrent reporting */
			    );
			desc->bPwrOn2PwrGood = 5;	/* msec/2 */
			desc->bHubContrCurrent = 0;

			/* workaround bogus struct definition */
			desc->u.hs.DeviceRemovable[0] = 0x02;	/* port 1 */
			desc->u.hs.DeviceRemovable[1] = 0xff;
		}
		break;
	case GetHubStatus:
		temp = 0;
		*(__le32 *) buf = cpu_to_le32(temp);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto error;

		/* finish RESET signaling? */
		if ((musb->port1_status & USB_PORT_STAT_RESET)
		    && time_after_eq(jiffies, musb->rh_timer))
			musb_port_reset(musb, false);

		/* finish RESUME signaling? */
		if ((musb->port1_status & MUSB_PORT_STAT_RESUME)
		    && time_after_eq(jiffies, musb->rh_timer)) {
			u8 power;

			power = musb_readb(musb->mregs, MUSB_POWER);
			power &= ~MUSB_POWER_RESUME;
			DBG(4, "root port resume stopped, power %02x\n"
						, power);
			musb_writeb(musb->mregs, MUSB_POWER, power);

			/* ISSUE:  DaVinci (RTL 1.300) disconnects after
			 * resume of high speed peripherals (but not full
			 * speed ones).
			 */

			musb->is_active = 1;
			musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
				| MUSB_PORT_STAT_RESUME);
			musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
			usb_hcd_poll_rh_status(musb_to_hcd(musb));
			/* NOTE: it might really be A_WAIT_BCON ... */
			musb->xceiv->otg->state = OTG_STATE_A_HOST;
		}
		musb_host_check_disconnect(musb);
		put_unaligned(cpu_to_le32(musb->port1_status
					  & ~MUSB_PORT_STAT_RESUME),
					  (__le32 *) buf);

		/* port change status is more interesting */
		DBG(0, "port status %08x,devctl=0x%x\n", musb->port1_status,
		    musb_readb(musb->mregs, MUSB_DEVCTL));
		break;
	case SetPortFeature:
		if ((wIndex & 0xff) != 1)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_POWER:
			/* NOTE: this controller has a strange state machine
			 * that involves "requesting sessions" according to
			 * magic side effects from incompletely-described
			 * rules about startup...
			 *
			 * This call is what really starts the host mode; be
			 * very careful about side effects if you reorder any
			 * initialization logic, e.g. for OTG, or change any
			 * logic relating to VBUS power-up.
			 */
			DBG(0, "try to call musb_start in virthub\n");
			/* if (!hcd->self.is_b_host) */
			/* musb_start(musb); */
			break;
		case USB_PORT_FEAT_RESET:
			musb_port_reset(musb, true);
			break;
		case USB_PORT_FEAT_SUSPEND:
			musb_port_suspend(musb, true);
			break;
		case USB_PORT_FEAT_TEST:
			break;	/* wz */
			/* if (unlikely(is_host_active(musb))) */
			/* goto error; */

			wIndex >>= 8;
			switch (wIndex) {
			case 1:
				pr_debug("TEST_J\n");
				temp = MUSB_TEST_J;
				break;
			case 2:
				pr_debug("TEST_K\n");
				temp = MUSB_TEST_K;
				break;
			case 3:
				pr_debug("TEST_SE0_NAK\n");
				temp = MUSB_TEST_SE0_NAK;
				break;
			case 4:
				pr_debug("TEST_PACKET\n");
				temp = MUSB_TEST_PACKET;
				musb_load_testpacket(musb);
				break;
			case 5:
				pr_debug("TEST_FORCE_ENABLE\n");
				temp = MUSB_TEST_FORCE_HOST
						| MUSB_TEST_FORCE_HS;

				musb_writeb(musb->mregs,
					MUSB_DEVCTL, MUSB_DEVCTL_SESSION);
				break;
			case 6:
				pr_debug("TEST_FIFO_ACCESS\n");
				temp = MUSB_TEST_FIFO_ACCESS;
				break;
			default:
				goto error;
			}
			musb_writeb(musb->mregs, MUSB_TESTMODE, temp);
			break;
		default:
			goto error;
		}
		DBG(5, "set feature %d\n", wValue);
		musb->port1_status |= 1 << wValue;
		break;

	default:
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	musb_platform_unprepare_clk(musb);

	#if IS_ENABLED(CONFIG_MTK_MUSB_PORT0_LOWPOWER_MODE)
	if (!usb_active)
		musb_platform_disable(musb);
	#endif
	return retval;
}
