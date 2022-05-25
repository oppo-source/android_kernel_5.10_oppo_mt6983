// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#include <usb20.h>
#include <musb_io.h>
#include <mtk_musb_reg.h>
#include <musb_core.h>

#if IS_ENABLED(CONFIG_USB_MTK_OTG)
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mtk_musb.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC)
#if IS_ENABLED(CONFIG_TCPC_CLASS)
#include <tcpm.h>
#endif
#endif
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>

MODULE_LICENSE("GPL v2");

struct device_node	*usb_node;
static int		iddig_eint_num;
static ktime_t		ktime_start, ktime_end;
static struct		regulator *reg_vbus;

static struct musb_fifo_cfg fifo_cfg_host[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 1, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 2, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 2, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 3, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 3, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 4, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 4, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 5, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 5, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 6, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 6, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 7, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 7, .style = FIFO_RX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 8, .style = FIFO_TX,
		.maxpacket = 512, .mode = BUF_SINGLE},
{ .hw_ep_num = 8, .style = FIFO_RX,
		.maxpacket = 64,  .mode = BUF_SINGLE},
};

u32 delay_time = 15;
module_param(delay_time, int, 0644);
u32 delay_time1 = 55;
module_param(delay_time1, int, 0644);
u32 iddig_cnt;
module_param(iddig_cnt, int, 0644);

static bool vbus_on;
module_param(vbus_on, bool, 0644);
static int vbus_control;
module_param(vbus_control, int, 0644);

static void _set_vbus(int is_on)
{
	if (!reg_vbus) {
		DBG(0, "vbus_init\n");
		reg_vbus = regulator_get(mtk_musb->controller, "usb-otg-vbus");
		if (IS_ERR_OR_NULL(reg_vbus)) {
			DBG(0, "failed to get vbus\n");
			return;
		}
	}

	DBG(0, "op<%d>, status<%d>\n", is_on, vbus_on);
	if (is_on && !vbus_on) {
		/* update flag 1st then enable VBUS to make
		 * host mode correct used by PMIC
		 */
		vbus_on = true;

		if (regulator_set_voltage(reg_vbus, 5000000, 5000000))
			DBG(0, "vbus regulator set voltage failed\n");

		if (regulator_set_current_limit(reg_vbus, 1500000, 1800000))
			DBG(0, "vbus regulator set current limit failed\n");

		if (regulator_enable(reg_vbus))
			DBG(0, "vbus regulator enable failed\n");

	} else if (!is_on && vbus_on) {
		/* disable VBUS 1st then update flag
		 * to make host mode correct used by PMIC
		 */
		vbus_on = false;
		regulator_disable(reg_vbus);
	}
}

int mt_usb_get_vbus_status(struct musb *musb)
{
	return true;
#ifdef NEVER
	int	ret = 0;

	if ((musb_readb(musb->mregs, MUSB_DEVCTL) &
		MUSB_DEVCTL_VBUS) != MUSB_DEVCTL_VBUS)
		ret = 1;
	else
		DBG(0, "VBUS error, devctl=%x, power=%d\n",
			musb_readb(musb->mregs, MUSB_DEVCTL),
			musb->power);
	pr_debug("vbus ready = %d\n", ret);
	return ret;
#endif
}

u32 sw_deboun_time = 400;
module_param(sw_deboun_time, int, 0644);

u32 typec_control;
module_param(typec_control, int, 0644);

static bool typec_req_host;
static bool iddig_req_host;

static void do_host_work(struct work_struct *data);
static void issue_host_work(int ops, int delay, bool on_st)
{
	struct mt_usb_work *work;

	if (!mtk_musb) {
		DBG(0, "mtk_musb = NULL\n");
		return;
	}

	/* create and prepare worker */
	work = kzalloc(sizeof(struct mt_usb_work), GFP_ATOMIC);
	if (!work) {
		DBG(0, "work is NULL, directly return\n");
		return;
	}
	work->ops = ops;
	INIT_DELAYED_WORK(&work->dwork, do_host_work);

	/* issue connection work */
	DBG(0, "issue work, ops<%d>, delay<%d>, on_st<%d>\n",
		ops, delay, on_st);

	if (on_st)
		queue_delayed_work(mtk_musb->st_wq,
					&work->dwork, msecs_to_jiffies(delay));
	else
		schedule_delayed_work(&work->dwork,
					msecs_to_jiffies(delay));
}
void mt_usb_host_connect(int delay)
{
	typec_req_host = true;
	DBG(0, "%s\n", typec_req_host ? "connect" : "disconnect");
	issue_host_work(CONNECTION_OPS_CONN, delay, true);
}
EXPORT_SYMBOL(mt_usb_host_connect);

void mt_usb_host_disconnect(int delay)
{
	typec_req_host = false;
	DBG(0, "%s\n", typec_req_host ? "connect" : "disconnect");
	issue_host_work(CONNECTION_OPS_DISC, delay, true);
}
EXPORT_SYMBOL(mt_usb_host_disconnect);

static bool musb_is_host(void)
{
	bool host_mode = 0;

	if (typec_control)
		host_mode = typec_req_host;
	else
		host_mode = iddig_req_host;

	return host_mode;
}

void musb_session_restart(struct musb *musb)
{
	void __iomem	*mbase = musb->mregs;

	musb_writeb(mbase, MUSB_DEVCTL,
				(musb_readb(mbase,
				MUSB_DEVCTL) & (~MUSB_DEVCTL_SESSION)));
	musb_writeb(mbase, MUSB_DEVCTL,
				(musb_readb(mbase,
				MUSB_DEVCTL) | MUSB_DEVCTL_SESSION));
	DBG(0, "[MUSB] restart session\n");
}
EXPORT_SYMBOL(musb_session_restart);

static struct delayed_work host_plug_test_work;
int host_plug_test_enable; /* default disable */
module_param(host_plug_test_enable, int, 0644);
int host_plug_in_test_period_ms = 5000;
module_param(host_plug_in_test_period_ms, int, 0644);
int host_plug_out_test_period_ms = 5000;
module_param(host_plug_out_test_period_ms, int, 0644);
int host_test_vbus_off_time_us = 3000;
module_param(host_test_vbus_off_time_us, int, 0644);
int host_test_vbus_only = 1;
module_param(host_test_vbus_only, int, 0644);
static int host_plug_test_triggered;
void switch_int_to_device(struct musb *musb)
{
	irq_set_irq_type(iddig_eint_num, IRQF_TRIGGER_HIGH);
	enable_irq(iddig_eint_num);
	DBG(0, "%s is done\n", __func__);
}

void switch_int_to_host(struct musb *musb)
{
	irq_set_irq_type(iddig_eint_num, IRQF_TRIGGER_LOW);
	enable_irq(iddig_eint_num);
	DBG(0, "%s is done\n", __func__);
}

static void do_host_plug_test_work(struct work_struct *data)
{
	static ktime_t ktime_begin, ktime_end;
	static s64 diff_time;
	static int host_on;
	static struct wakeup_source *host_test_wakelock;
	static int wake_lock_inited;

	if (!wake_lock_inited) {
		DBG(0, "wake_lock_init\n");
		host_test_wakelock = wakeup_source_register(NULL,
					"host.test.lock");
		wake_lock_inited = 1;
	}

	host_plug_test_triggered = 1;
	/* sync global status */
	mb();
	__pm_stay_awake(host_test_wakelock);
	DBG(0, "BEGIN");
	ktime_begin = ktime_get();

	host_on  = 1;
	while (1) {
		if (!musb_is_host() && host_on) {
			DBG(0, "about to exit");
			break;
		}
		msleep(50);

		ktime_end = ktime_get();
		diff_time = ktime_to_ms(ktime_sub(ktime_end, ktime_begin));
		if (host_on && diff_time >= host_plug_in_test_period_ms) {
			host_on = 0;
			DBG(0, "OFF\n");

			ktime_begin = ktime_get();

			/* simulate plug out */
			_set_vbus(0);
			udelay(host_test_vbus_off_time_us);

			if (!host_test_vbus_only)
				issue_host_work(CONNECTION_OPS_DISC, 0, false);
		} else if (!host_on && diff_time >=
					host_plug_out_test_period_ms) {
			host_on = 1;
			DBG(0, "ON\n");

			ktime_begin = ktime_get();
			if (!host_test_vbus_only)
				issue_host_work(CONNECTION_OPS_CONN, 0, false);

			_set_vbus(1);
			msleep(100);

		}
	}

	/* wait host_work done */
	msleep(1000);
	host_plug_test_triggered = 0;
	__pm_relax(host_test_wakelock);
	DBG(0, "END\n");
}

#define ID_PIN_WORK_RECHECK_TIME 30	/* 30 ms */
#define ID_PIN_WORK_BLOCK_TIMEOUT 30000 /* 30000 ms */
static void do_host_work(struct work_struct *data)
{
	u8 devctl = 0;
	unsigned long flags;
	static int inited, timeout; /* default to 0 */
	static s64 diff_time;
	bool host_on;
	int usb_clk_state = NO_CHANGE;
	struct mt_usb_work *work =
		container_of(data, struct mt_usb_work, dwork.work);
	struct mt_usb_glue *glue = mtk_musb->glue;

	/*
	 * kernel_init_done should be set in
	 * early-init stage through init.$platform.usb.rc
	 */
	while (!inited && !kernel_init_done &&
		   !mtk_musb->is_ready && !timeout) {
		ktime_end = ktime_get();
		diff_time = ktime_to_ms(ktime_sub(ktime_end, ktime_start));

		DBG_LIMIT(3,
			"init_done:%d, is_ready:%d, inited:%d, TO:%d, diff:%lld",
			kernel_init_done,
			mtk_musb->is_ready,
			inited,
			timeout,
			diff_time);

		if (diff_time > ID_PIN_WORK_BLOCK_TIMEOUT) {
			DBG(0, "diff_time:%lld\n", diff_time);
			timeout = 1;
		}
		msleep(ID_PIN_WORK_RECHECK_TIME);
	}

	if (!inited) {
		DBG(0, "PASS,init_done:%d,is_ready:%d,inited:%d, TO:%d\n",
				kernel_init_done,  mtk_musb->is_ready,
				inited, timeout);
		inited = 1;
	}

	/* always prepare clock and check if need to unprepater later */
	/* clk_prepare_cnt +1 here */
	usb_prepare_clock(true);

	down(&mtk_musb->musb_lock);

	host_on = (work->ops ==
			CONNECTION_OPS_CONN ? true : false);

	DBG(0, "work start, is_host=%d, host_on=%d\n",
		mtk_musb->is_host, host_on);

	if (host_on && !mtk_musb->is_host) {
		/* switch to HOST state before turn on VBUS */
		MUSB_HST_MODE(mtk_musb);

		/* to make sure all event clear */
		msleep(32);
#if IS_ENABLED(CONFIG_MTK_UAC_POWER_SAVING)
		if (!usb_on_sram) {
			int ret;

			ret = gpd_switch_to_sram(mtk_musb->controller);
			DBG(0, "gpd_switch_to_sram, ret<%d>\n", ret);
			if (ret == 0)
				usb_on_sram = 1;
		}
#endif
		/* setup fifo for host mode */
		ep_config_from_table_for_host(mtk_musb);

		if (!mtk_musb->host_suspend)
			__pm_stay_awake(mtk_musb->usb_lock);

		/* this make PHY operation workable */
		musb_platform_enable(mtk_musb);

		/* for no VBUS sensing IP*/

		/* wait VBUS ready */
		msleep(100);
		/* clear session*/
		devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
		musb_writeb(mtk_musb->mregs,
				MUSB_DEVCTL, (devctl&(~MUSB_DEVCTL_SESSION)));

		phy_set_mode(glue->phy, PHY_MODE_INVALID);

		/* wait */
		mdelay(5);
		/* restart session */
		devctl = musb_readb(mtk_musb->mregs, MUSB_DEVCTL);
		musb_writeb(mtk_musb->mregs,
				MUSB_DEVCTL, (devctl | MUSB_DEVCTL_SESSION));

		phy_set_mode(glue->phy, PHY_MODE_USB_HOST);

		musb_start(mtk_musb);

		if (!typec_control && !host_plug_test_triggered)
			switch_int_to_device(mtk_musb);

		if (host_plug_test_enable && !host_plug_test_triggered)
			queue_delayed_work(mtk_musb->st_wq,
						&host_plug_test_work, 0);
		usb_clk_state = OFF_TO_ON;
	}  else if (!host_on && mtk_musb->is_host) {
		/* switch from host -> device */
		/* for device no disconnect interrupt */
		spin_lock_irqsave(&mtk_musb->lock, flags);
		if (mtk_musb->is_active) {
			DBG(0, "for not receiving disconnect interrupt\n");
			usb_hcd_resume_root_hub(musb_to_hcd(mtk_musb));
			musb_root_disconnect(mtk_musb);
		}
		spin_unlock_irqrestore(&mtk_musb->lock, flags);

		DBG(1, "devctl is %x\n",
				musb_readb(mtk_musb->mregs, MUSB_DEVCTL));
		musb_writeb(mtk_musb->mregs, MUSB_DEVCTL, 0);
		if (mtk_musb->usb_lock->active)
			__pm_relax(mtk_musb->usb_lock);

		/* for no VBUS sensing IP */
		phy_set_mode(glue->phy, PHY_MODE_INVALID);

		musb_stop(mtk_musb);

		if (!typec_control && !host_plug_test_triggered)
			switch_int_to_host(mtk_musb);

#if IS_ENABLED(CONFIG_MTK_UAC_POWER_SAVING)
		if (usb_on_sram) {
			gpd_switch_to_dram(mtk_musb->controller);
			usb_on_sram = 0;
		}
#endif
		/* to make sure all event clear */
		msleep(32);

		mtk_musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		/* switch to DEV state after turn off VBUS */
		MUSB_DEV_MODE(mtk_musb);

		usb_clk_state = ON_TO_OFF;
	}
	DBG(0, "work end, is_host=%d\n", mtk_musb->is_host);
	up(&mtk_musb->musb_lock);

	if (usb_clk_state == ON_TO_OFF) {
		/* clock on -> of: clk_prepare_cnt -2 */
		usb_prepare_clock(false);
		usb_prepare_clock(false);
	} else if (usb_clk_state == NO_CHANGE) {
		/* clock no change : clk_prepare_cnt -1 */
		usb_prepare_clock(false);
	}
	/* free mt_usb_work */
	kfree(work);
}

static irqreturn_t mt_usb_ext_iddig_int(int irq, void *dev_id)
{
	iddig_cnt++;

	iddig_req_host = !iddig_req_host;
	DBG(0, "id pin assert, %s\n", iddig_req_host ?
			"connect" : "disconnect");

	if (iddig_req_host)
		mt_usb_host_connect(0);
	else
		mt_usb_host_disconnect(0);
	disable_irq_nosync(iddig_eint_num);
	return IRQ_HANDLED;
}

static const struct of_device_id otg_iddig_of_match[] = {
	{.compatible = "mediatek,usb_iddig_bi_eint"},
	{},
};

static int otg_iddig_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	iddig_eint_num = irq_of_parse_and_map(node, 0);
	DBG(0, "iddig_eint_num<%d>\n", iddig_eint_num);
	if (iddig_eint_num < 0)
		return -ENODEV;

	ret = request_irq(iddig_eint_num, mt_usb_ext_iddig_int,
					IRQF_TRIGGER_LOW, "USB_IDDIG", NULL);
	if (ret) {
		DBG(0,
			"request EINT <%d> fail, ret<%d>\n",
			iddig_eint_num, ret);
		return ret;
	}

	return 0;
}

static struct platform_driver otg_iddig_driver = {
	.probe = otg_iddig_probe,
	/* .remove = otg_iddig_remove, */
	/* .shutdown = otg_iddig_shutdown, */
	.driver = {
		.name = "otg_iddig",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(otg_iddig_of_match),
	},
};


static int iddig_int_init(void)
{
	int	ret = 0;

	ret = platform_driver_register(&otg_iddig_driver);
	if (ret)
		DBG(0, "ret:%d\n", ret);

	return 0;
}

void mt_usb_otg_init(struct musb *musb)
{
	/* test */
	INIT_DELAYED_WORK(&host_plug_test_work, do_host_plug_test_work);
	ktime_start = ktime_get();

	/* CONNECTION MANAGEMENT*/
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC)
	DBG(0, "host controlled by TYPEC\n");
	typec_control = 1;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	DBG(0, "host controlled by IDDIG\n");
	iddig_int_init();
	vbus_control = 1;
#endif /* CONFIG_TCPC_CLASS */
#endif /* CONFIG_MTK_USB_TYPEC */

	/* EP table */
	musb->fifo_cfg_host = fifo_cfg_host;
	musb->fifo_cfg_host_size = ARRAY_SIZE(fifo_cfg_host);

}
EXPORT_SYMBOL(mt_usb_otg_init);

void mt_usb_otg_exit(struct musb *musb)
{
	DBG(0, "OTG disable vbus\n");
}
EXPORT_SYMBOL(mt_usb_otg_exit);

enum {
	DO_IT = 0,
	REVERT,
};

static int option;
static int set_option(const char *val, const struct kernel_param *kp)
{
	int local_option;
	int rv;

	/* update module parameter */
	rv = param_set_int(val, kp);
	if (rv)
		return rv;

	/* update local_option */
	rv = kstrtoint(val, 10, &local_option);
	if (rv != 0)
		return rv;

	DBG(0, "option:%d, local_option:%d\n", option, local_option);

	switch (local_option) {
	case 0:
		DBG(0, "case %d\n", local_option);
		iddig_int_init();
		break;
	case 1:
		DBG(0, "case %d\n", local_option);
		mt_usb_host_connect(0);
		break;
	case 2:
		DBG(0, "case %d\n", local_option);
		mt_usb_host_disconnect(0);
		break;
	case 3:
		DBG(0, "case %d\n", local_option);
		mt_usb_host_connect(3000);
		break;
	case 4:
		DBG(0, "case %d\n", local_option);
		mt_usb_host_disconnect(3000);
		break;
	case 9:
		DBG(0, "case %d\n", local_option);
		_set_vbus(1);
		break;
	case 10:
		DBG(0, "case %d\n", local_option);
		_set_vbus(0);
		break;
	default:
		break;
	}
	return 0;
}
static struct kernel_param_ops option_param_ops = {
	.set = set_option,
	.get = param_get_int,
};
module_param_cb(option, &option_param_ops, &option, 0644);
#else
#include "musb_core.h"
/* for not define CONFIG_USB_MTK_OTG */
void mt_usb_otg_init(struct musb *musb) {}
EXPORT_SYMBOL(mt_usb_otg_init);

void mt_usb_otg_exit(struct musb *musb) {}
EXPORT_SYMBOL(mt_usb_otg_exit);

void mt_usb_set_vbus(struct musb *musb, int is_on) {}
int mt_usb_get_vbus_status(struct musb *musb) {return 1; }
void switch_int_to_device(struct musb *musb) {}
void switch_int_to_host(struct musb *musb) {}

void musb_session_restart(struct musb *musb) {}
EXPORT_SYMBOL(musb_session_restart);
#endif
