// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/kdev_t.h>
#include <linux/timekeeping.h>
#include <linux/usb/composite.h>
#include <trace/hooks/sound.h>
#include <linux/pm_wakeup.h>

#include "usb_boost.h"

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
#include "musb_core.h"
#include "musb_trace.h"
#endif

#define USB_BOOST_CLASS_NAME "usb_boost"
enum{
	ATTR_ENABLE,
	ATTR_TIMEOUT,
	ATTR_POLLING_INTVAL,
	ATTR_RAW,
	_ATTR_PARA_RW_MAXID,

	ATTR_CMD,

	ATTR_RO_REF_TIME,
	ATTR_RO_IS_RUNNING,
	ATTR_RO_WORK_CNT,
	_ATTR_PARA_MAXID
};
enum{
	_ATTR_ARG_BEGIN = _ATTR_PARA_MAXID - 1,
	ATTR_ARG1,
	ATTR_ARG2,
	ATTR_ARG3,
	_ATTR_ARG_MAXID
};

#define _ATTR_MAXID _ATTR_ARG_MAXID
static char *attr_name[_ATTR_MAXID] = {
	/* para part */
	"enable",
	"timeout",
	"poll_intval",
	"raw",
	"para_rx_maxid",

	"cmd",

	"ro_ref_time",
	"ro_is_running",
	"ro_work_cnt",

	/* arg part */
	"arg1",
	"arg2",
	"arg3",
};
enum{
	CMD_BOOST_TEST,
	CMD_OVERHEAD_TEST,
	CMD_OVERHEAD_TEST_BY_ID,
	CMD_DUMP_INFO,
	CMD_HOOK_NORMAL,
	CMD_HOOK_EMPTY,
	CMD_HOOK_CNT,
	_CMD_MAXID
};
static char *type_name[_TYPE_MAXID] = {
	"cpu_freq",
	"cpu_core",
	"dram_vcore",
};

enum {
	USB_TYPE_UNKNOWN,
	USB_TYPE_ADB,
	USB_TYPE_MTP,
	/* USB_TYPE_PTP, */
	USB_TYPE_RNDIS,
};

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
#define MAX_EP_NUM 32
/* ep0 + in ep + out ep */
static int boost_ep[MAX_EP_NUM * 2 + 1];
#endif

#define MAX_LEN_WQ_NAME 32
static int trigger_cnt_disabled;
static int enabled;
static int inited;
static struct class *usb_boost_class;
static struct wakeup_source *usb_boost_ws;
static int cpu_freq_dft_para[_ATTR_PARA_RW_MAXID] = {1, 3, 300, 0};
static int cpu_core_dft_para[_ATTR_PARA_RW_MAXID] = {1, 3, 300, 0};
static int dram_vcore_dft_para[_ATTR_PARA_RW_MAXID] = {1, 3, 300, 0};
static void __usb_boost_empty(void) { return; }
static void __usb_boost_cnt(void) { trigger_cnt_disabled++; return; }
static void __usb_boost_id_empty(int id) { return; }
static void __request_empty(int id) { return; }

struct boost_ops {
	void (*boost)(void);

	void (*boost_by_id[_TYPE_MAXID])(int);
};

struct boost_ops __the_boost_ops = {
	__usb_boost_empty,
	{__usb_boost_id_empty,
	 __usb_boost_id_empty,
	 __usb_boost_id_empty} };

/* -1 denote not used*/
static struct act_arg_obj cpu_freq_dft_arg = {1000000000, -1, -1};
static struct act_arg_obj cpu_core_dft_arg = {2, -1, -1};
static struct act_arg_obj dram_vcore_dft_arg = {-1, -1, -1};

static int test_diff_sec, test_diff_usec;

struct control_ops {
	int (*act[_ACT_MAXID]) (struct act_arg_obj *arg);
};
static struct mtk_usb_boost {
	struct control_ops ops;
	struct device_attribute attr[_ATTR_MAXID];
	int para[_ATTR_PARA_RW_MAXID];
	struct work_struct	work;
	struct workqueue_struct	*wq;
	int id;
	struct device *dev;
	int cmd;
	int is_running;
	struct timespec64 tv_ref_time;
	int work_cnt;
	struct act_arg_obj act_arg;
	void (*request_func)(int id);
} boost_inst[_TYPE_MAXID];

static struct mtk_usb_audio_boost {
	struct work_struct	work;
	struct workqueue_struct	*wq;
	struct timespec64 tv_ref_time;
	void (*request_func)(int id);
} audio_boost_inst;

static int update_time(int id);
static bool check_timeout(int id);
static void __usb_boost_by_id(int id);

/* public to IP platform level */
void usb_boost_set_para_and_arg(int id, int *para, int para_range,
	struct act_arg_obj *act_arg)
{
	int i;
	struct mtk_usb_boost *ptr_inst = &boost_inst[id];
	int *ptr_para = ptr_inst->para;

	USB_BOOST_NOTICE("para_range:<%d>\n", para_range);
	if (para_range > _ATTR_PARA_RW_MAXID) {
		USB_BOOST_NOTICE("ERROR, over range !!!!!\n");
		USB_BOOST_NOTICE("para_range<%d>, _ATTR_PARA_RW_MAXID<%d>\n",
			para_range, _ATTR_PARA_RW_MAXID);
		return;
	}

	for (i = 0; i < para_range; i++, ptr_para++)
		*ptr_para = para[i];

	boost_inst[id].act_arg.arg1 = act_arg->arg1;
	boost_inst[id].act_arg.arg2 = act_arg->arg2;
	boost_inst[id].act_arg.arg3 = act_arg->arg3;

	/* hook callback by enable flag */
	if (para[0])
		__the_boost_ops.boost_by_id[id] = __usb_boost_by_id;
	else
		__the_boost_ops.boost_by_id[id] = __usb_boost_id_empty;
}

void usb_boost(void)
{
	__the_boost_ops.boost();
}

void usb_boost_by_id(int id)
{
	__the_boost_ops.boost_by_id[id](id);
}

void register_usb_boost_act(int type_id, int action_id,
	int (*func)(struct act_arg_obj *arg))
{
	boost_inst[type_id].ops.act[action_id] = func;
}

static void __request_it(int id)
{
	USB_BOOST_DBG("ID<%d>, WQ<%p>, WORK<%p>\n",
		id, boost_inst[id].wq, &(boost_inst[id].work));

	queue_work(boost_inst[id].wq, &(boost_inst[id].work));
	USB_BOOST_DBG("\n");
}

static void __usb_boost_by_id(int id)
{
	update_time(id);
	boost_inst[id].request_func(id);
}

static void __usb_boost(void)
{
	int id;

	USB_BOOST_DBG("\n");
	for (id = 0; id < _TYPE_MAXID; id++)
		usb_boost_by_id(id);
}

static void __boost_act(int type_id, int action_id)
{
	int (*func)(struct act_arg_obj *arg);
	struct act_arg_obj *arg = &boost_inst[type_id].act_arg;

	func = boost_inst[type_id].ops.act[action_id];

	if (func)
		func(arg);
}

static void dump_info(int id)
{
	int n = 0;
	struct mtk_usb_boost *ptr_inst = &boost_inst[id];
	int *ptr_para = ptr_inst->para;

	/* PARA */
	for (n = 0; n < _ATTR_PARA_RW_MAXID; n++, ptr_para++) {
		USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d>\n",
			id, attr_name[n], *ptr_para);
	}
	/* RO */
	USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d,%d>\n",
		id,	attr_name[ATTR_RO_REF_TIME],
		(unsigned int)boost_inst[id].tv_ref_time.tv_sec,
		(unsigned int)boost_inst[id].tv_ref_time.tv_nsec);

	USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d>\n",
		id, attr_name[ATTR_RO_IS_RUNNING], boost_inst[id].is_running);

	USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d>\n",
		id, attr_name[ATTR_RO_WORK_CNT], boost_inst[id].work_cnt);

	/* ARG */
	USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d>\n",
		id, attr_name[ATTR_ARG1], boost_inst[id].act_arg.arg1);
	USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d>\n",
		id, attr_name[ATTR_ARG2], boost_inst[id].act_arg.arg2);
	USB_BOOST_NOTICE("id<%d>, attr<%s>, val<%d>\n",
		id, attr_name[ATTR_ARG3], boost_inst[id].act_arg.arg3);

}
static int update_time(int id)
{
	ktime_get_ts64(&boost_inst[id].tv_ref_time);
	USB_BOOST_DBG("id:%d, ref<%d,%d>\n", id,
		(int)boost_inst[id].tv_ref_time.tv_sec,
		(int)boost_inst[id].tv_ref_time.tv_nsec);
	return 1;
}

static bool check_timeout(int id)
{
	struct timespec64 tv, *ref;
	int diff_sec;

	ref = &boost_inst[id].tv_ref_time;
	ktime_get_ts64(&tv);
	diff_sec = tv.tv_sec - ref->tv_sec;
	if (diff_sec >= boost_inst[id].para[ATTR_TIMEOUT]) {
		USB_BOOST_DBG("id<%d>, cur<%d,%d>, ref<%d,%d>\n",
				id, (int)tv.tv_sec, (int)tv.tv_nsec,
				(int)ref->tv_sec, (int)ref->tv_nsec);
		return true;
	}
	USB_BOOST_DBG("id<%d>, cur<%d,%d>, ref<%d,%d>\n",
			id, (int)tv.tv_sec, (int)tv.tv_nsec,
			(int)ref->tv_sec, (int)ref->tv_nsec);

	return false;
}

static void boost_work(struct work_struct *work_struct)
{
	struct mtk_usb_boost *ptr_inst =
		container_of(work_struct, struct mtk_usb_boost, work);
	int id = ptr_inst->id;
	int raw = ptr_inst->para[ATTR_RAW];
	int poll_intval = ptr_inst->para[ATTR_POLLING_INTVAL];

	ptr_inst->is_running = true;
	boost_inst[id].request_func = __request_empty;
	ptr_inst->work_cnt++;
	USB_BOOST_NOTICE("id:%d, begin of work\n", id);

	/* dump_info(id); */
	__boost_act(id, ACT_HOLD);
	/* dump_info(id); */
	while (1) {
		int timeout;

		USB_BOOST_DBG("id:%d, running of work\n", id);
		if (!ptr_inst->para[ATTR_ENABLE]) {
			/* dump_info(id); */
			break;
		}
		timeout = check_timeout(id);
		if (timeout) {
			/* dump_info(id); */
			break;
		}

		if (raw)
			__boost_act(id, ACT_HOLD);

		msleep(poll_intval);

	}

	/* dump_info(id); */
	__boost_act(id, ACT_RELEASE);
	boost_inst[id].request_func = __request_it;
	ptr_inst->is_running = false;
	USB_BOOST_NOTICE("id:%d, end of work\n", id);
	/* dump_info(id); */
}

/* usb audio fine tune */
static void __request_audio(int id)
{
	queue_work(audio_boost_inst.wq, &(audio_boost_inst.work));
}

static bool check_timeout_audio(void)
{
	struct timespec64 tv, *ref;
	int diff_sec;

	ref = &audio_boost_inst.tv_ref_time;
	ktime_get_ts64(&tv);
	diff_sec = tv.tv_sec - ref->tv_sec;
	/* 5 secs */
	if (diff_sec >= 5) {
		USB_BOOST_DBG("audio_boost, cur<%d,%d>, ref<%d,%d>\n",
			(int)tv.tv_sec, (int)tv.tv_nsec,
			(int)ref->tv_sec, (int)ref->tv_nsec);
		return true;
	}

	return false;
}

static void audio_boost_work(struct work_struct *work_struct)
{
	audio_boost_inst.request_func = __request_empty;
	USB_BOOST_NOTICE("audio_boost, begin of work\n");
	audio_core_hold();
	audio_freq_hold();

	while (1) {
		int timeout;

		USB_BOOST_DBG("audio_boost, running of work\n");
		timeout = check_timeout_audio();
		if (timeout) {
			/* dump_info(id); */
			break;
		}

		msleep(500);
	}

	audio_core_release();
	audio_freq_release();
	audio_boost_inst.request_func = __request_audio;
	USB_BOOST_NOTICE("audio_boost, end of work\n");
}

static void default_setting(void)
{
	usb_boost_set_para_and_arg(TYPE_CPU_FREQ, cpu_freq_dft_para,
			ARRAY_SIZE(cpu_freq_dft_para), &cpu_freq_dft_arg);

	usb_boost_set_para_and_arg(TYPE_CPU_CORE, cpu_core_dft_para,
			ARRAY_SIZE(cpu_core_dft_para), &cpu_core_dft_arg);

	usb_boost_set_para_and_arg(TYPE_DRAM_VCORE, dram_vcore_dft_para,
			ARRAY_SIZE(dram_vcore_dft_para), &dram_vcore_dft_arg);
}

static int which_attr(struct mtk_usb_boost *inst, struct device_attribute
		      *attr)
{
	int i;

	for (i = 0; i < _ATTR_MAXID; i++) {
		if (attr == &inst->attr[i])
			return i;
	}
	return -1;
}

static void test_loops(int id)
{
	int n;
	struct timespec64 tv_before, tv_after;
#define TEST_LOOP 100000
	ktime_get_ts64(&tv_before);
	if (id < 0) {
		for (n = 0; n < TEST_LOOP; n++)
			usb_boost();
	} else {
		for (n = 0; n < TEST_LOOP; n++)
			usb_boost_by_id(id);
	}
	ktime_get_ts64(&tv_after);
	test_diff_sec = tv_after.tv_sec - tv_before.tv_sec;
	test_diff_usec = tv_after.tv_nsec - tv_before.tv_nsec;
	USB_BOOST_NOTICE("id<%d>, loops:%d, spent %d sec, %d usec\n",
		id, TEST_LOOP, test_diff_sec, test_diff_usec);
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int i, ret, idx = -1;
	long tmp;

	for (i = 0; i < _TYPE_MAXID; i++) {
		if (boost_inst[i].dev == dev) {
			idx = which_attr(&boost_inst[i], attr);
			break;
		}
	}
	if (i == _TYPE_MAXID)
		return 0;

	if (idx < 0) {
		USB_BOOST_NOTICE("sorry, I cannot find rawbulk fn '%s'\n",
			attr->attr.name);
		goto exit;
	}

	ret = kstrtol(buf, 0, &tmp);

	switch (idx) {
	/* normal usage */
	case ATTR_ENABLE:
		/* hook callback by enable flag */
		if (tmp)
			__the_boost_ops.boost_by_id[i] = __usb_boost_by_id;
		else
			__the_boost_ops.boost_by_id[i] = __usb_boost_id_empty;
		boost_inst[i].para[idx] = (int)tmp;
		break;
	case ATTR_TIMEOUT:
	case ATTR_POLLING_INTVAL:
	case ATTR_RAW:
		boost_inst[i].para[idx] = (int)tmp;
		break;
	/* command series */
	case ATTR_CMD:
		boost_inst[i].cmd = (int)tmp;
		switch (tmp) {
		case CMD_BOOST_TEST:
			USB_BOOST_NOTICE("usb_boost_by_id <%d>\n", i);
			usb_boost_by_id(i);
			break;
		case CMD_OVERHEAD_TEST:
			test_loops(-1);
			break;
		case CMD_OVERHEAD_TEST_BY_ID:
			test_loops(i);
			break;
		case CMD_DUMP_INFO:
			dump_info(i);
			break;
		case CMD_HOOK_NORMAL:
			__the_boost_ops.boost = __usb_boost;
			enabled = 1;
			break;
		case CMD_HOOK_EMPTY:
			__the_boost_ops.boost = __usb_boost_empty;
			enabled = 0;
			test_loops(-1);
			break;
		case CMD_HOOK_CNT:
			__the_boost_ops.boost = __usb_boost_cnt;
			enabled = 0;
			test_loops(-1);
			break;
		default:
			break;
		}
		break;
	/* ARG usage */
	case ATTR_ARG1:
		boost_inst[i].act_arg.arg1 = (int)tmp;
		break;
	case ATTR_ARG2:
		boost_inst[i].act_arg.arg2 = (int)tmp;
		break;
	case ATTR_ARG3:
		boost_inst[i].act_arg.arg3 = (int)tmp;
		break;
	default:
		break;
	}

exit:
	return count;
}

static ssize_t attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	int idx;
	ssize_t count = 0;

	for (i = 0; i < _TYPE_MAXID; i++) {
		if (boost_inst[i].dev == dev) {
			idx = which_attr(&boost_inst[i], attr);
			break;
		}
	}
	if (i == _TYPE_MAXID)
		return 0;

	switch (idx) {
	/* normal usage */
	case ATTR_ENABLE:
	case ATTR_TIMEOUT:
	case ATTR_POLLING_INTVAL:
	case ATTR_RAW:
		count = sprintf(buf, "%d\n", boost_inst[i].para[idx]);
		break;
	case _ATTR_PARA_RW_MAXID:
		count = sprintf(buf, "%d\n", _ATTR_PARA_RW_MAXID);
		break;
	/* command series */
	case ATTR_CMD:
		switch (boost_inst[i].cmd) {
		case CMD_BOOST_TEST:
		case CMD_DUMP_INFO:
			count = sprintf(buf, "cmd<%d>\n", boost_inst[i].cmd);
			break;
		case CMD_OVERHEAD_TEST:
		case CMD_OVERHEAD_TEST_BY_ID:
		case CMD_HOOK_NORMAL:
		case CMD_HOOK_EMPTY:
		case CMD_HOOK_CNT:
			count = sprintf(buf, "cmd<%d>, <%d, %d>\n",
				boost_inst[i].cmd, test_diff_sec,
				test_diff_usec);
			break;
		default:
			break;
		}
		break;
	/* RO usage */
	case ATTR_RO_REF_TIME:
		count = sprintf(buf, "<%d,%d>\n",
			(int)boost_inst[i].tv_ref_time.tv_sec,
			(int)boost_inst[i].tv_ref_time.tv_nsec);
		break;
	case ATTR_RO_IS_RUNNING:
		count = sprintf(buf, "%s\n",
			boost_inst[i].is_running ? "true" : "false");
		break;
	case ATTR_RO_WORK_CNT:
		count = sprintf(buf, "%d\n", boost_inst[i].work_cnt);
		break;
	/* ARG usage */
	case ATTR_ARG1:
		count = sprintf(buf, "%d\n", boost_inst[i].act_arg.arg1);
		break;
	case ATTR_ARG2:
		count = sprintf(buf, "%d\n", boost_inst[i].act_arg.arg2);
		break;
	case ATTR_ARG3:
		count = sprintf(buf, "%d\n", boost_inst[i].act_arg.arg3);
		break;
	default:
		break;
	}
	return count;
}

static int create_sys_fs(void)
{
	int i;
	int n, ret;

	USB_BOOST_NOTICE("\n");
	usb_boost_class = class_create(THIS_MODULE, USB_BOOST_CLASS_NAME);
	if (IS_ERR(usb_boost_class))
		return PTR_ERR(usb_boost_class);

	for (i = 0 ; i < _TYPE_MAXID ; i++) {
		boost_inst[i].dev = device_create(usb_boost_class,
			NULL, MKDEV(0, i), NULL, type_name[i]);

		for (n = 0; n < _ATTR_MAXID; n++) {
			boost_inst[i].attr[n].attr.name = attr_name[n];
			boost_inst[i].attr[n].attr.mode = 0400;
			boost_inst[i].attr[n].show = attr_show;
			boost_inst[i].attr[n].store = attr_store;

			ret = device_create_file(boost_inst[i].dev,
				&boost_inst[i].attr[n]);
			if (ret < 0) {
				while (--n >= 0) {
					device_remove_file(boost_inst[i].dev,
						&boost_inst[i].attr[n]);
				}
				return ret;
			}
		}

	}

	return 0;

}

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
static struct usb_descriptor_header **
get_function_descriptors(struct usb_function *f,
		     enum usb_device_speed speed)
{
	struct usb_descriptor_header **descriptors;

	switch (speed) {
	case USB_SPEED_SUPER_PLUS:
		descriptors = f->ssp_descriptors;
		if (descriptors)
			break;
		fallthrough;
	case USB_SPEED_SUPER:
		descriptors = f->ss_descriptors;
		if (descriptors)
			break;
		fallthrough;
	case USB_SPEED_HIGH:
		descriptors = f->hs_descriptors;
		if (descriptors)
			break;
		fallthrough;
	default:
		descriptors = f->fs_descriptors;
	}

	return descriptors;
}

static int musb_get_ep_type(struct usb_descriptor_header **f_desc)
{
	struct usb_interface_descriptor *int_desc;
	u8 int_class, int_subclass, int_protocol;

	for (; *f_desc; ++f_desc) {
		if ((*f_desc)->bDescriptorType != USB_DT_INTERFACE)
			continue;

		int_desc = (struct usb_interface_descriptor *)*f_desc;
		int_class = int_desc->bInterfaceClass;
		int_subclass = int_desc->bInterfaceSubClass;
		int_protocol = int_desc->bInterfaceProtocol;

		if (int_class == 0x6 && int_subclass == 0x1
			&& int_protocol == 0x1) {
			return USB_TYPE_MTP;
		} else if (int_class == 0xff && int_subclass == 0x42
			&& int_protocol == 0x1) {
			return USB_TYPE_ADB;
		} else if (int_class == 0x2 && int_subclass == 0x2
			&& int_protocol == 0xff) {
			return USB_TYPE_RNDIS;
		} else if (int_class == 0xe0 && int_subclass == 0x1
			&& int_protocol == 0x3) {
			return USB_TYPE_RNDIS;
		}
	}

	return USB_TYPE_UNKNOWN;
}

static void boost_set_ep_type(int type, int num, int is_in)
{
	if (num > MAX_EP_NUM)
		return;

	if (is_in)
		boost_ep[num] = type;
	else
		boost_ep[num + MAX_EP_NUM] = type;
}

static int boost_get_ep_type(int num, int is_in)
{
	int type;

	if (num > MAX_EP_NUM)
		return USB_TYPE_UNKNOWN;

	if (is_in)
		type = boost_ep[num];
	else
		type = boost_ep[num + MAX_EP_NUM];

	return type;
}

static void boost_ep_enable(void *unused, struct musb_ep *musb_ep)
{
	struct usb_ep *ep = &musb_ep->end_point;
	struct usb_gadget *g = &musb_ep->musb->g;
	struct usb_composite_dev *cdev;
	struct usb_function *f = NULL;
	struct usb_descriptor_header **f_desc;
	int addr;
	int type;

	cdev = get_gadget_data(&musb_ep->musb->g);
	if (!cdev || !cdev->config)
		return;

	if (!musb_ep->current_epnum)
		return;

	addr = ((ep->address & 0x80) >> 3)
			| (ep->address & 0x0f);

	list_for_each_entry(f, &cdev->config->functions, list) {
		if (test_bit(addr, f->endpoints))
			goto find_f;
	}
	return;
find_f:
	f_desc = get_function_descriptors(f, g->speed);
	if (f_desc) {
		type = musb_get_ep_type(f_desc);
		boost_set_ep_type(type, musb_ep->current_epnum, musb_ep->is_in);
	}
}

static void boost_ep_disable(void *unused, struct musb_ep *musb_ep)
{
	if (!musb_ep->current_epnum)
		return;

	boost_set_ep_type(USB_TYPE_UNKNOWN, musb_ep->current_epnum, musb_ep->is_in);
}

static void musb_g_giveback_boost(void *unused, struct musb_request *musb_req)
{
	struct usb_request *req = &musb_req->request;
	struct musb_ep *musb_ep = musb_req->ep;
	int type = boost_get_ep_type(musb_ep->current_epnum, musb_ep->is_in);

	switch (type) {
	case USB_TYPE_MTP:
		if (req->actual >= 8192)
			usb_boost();
		break;
	case USB_TYPE_RNDIS:
		if (musb_ep->is_in && musb_ep->type == USB_ENDPOINT_XFER_BULK)
			usb_boost();
		break;
	default:
		break;
	}
}

static int update_time_audio(void)
{
	ktime_get_ts64(&audio_boost_inst.tv_ref_time);
	return 1;
}

void musb_host_urb_giveback_dbg(void *unused, struct urb *urb)
{
	switch (usb_endpoint_type(&urb->ep->desc)) {
	case USB_ENDPOINT_XFER_BULK:
		if (urb->actual_length >= 8192) {
			__pm_wakeup_event(usb_boost_ws, 10000);
			usb_boost();
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		update_time_audio();
		break;
	}
}

static void vh_sound_usb_support_cpu_suspend(void *unused,
	struct usb_device *udev, int direction, bool *is_support)
{
	USB_BOOST_DBG("%s enter\n", __func__);

	update_time_audio();
	audio_boost_inst.request_func(0);
}

static int musb_trace_init(void)
{
	WARN_ON(register_trace_musb_gadget_enable(
		boost_ep_enable, NULL));
	WARN_ON(register_trace_musb_gadget_disable(
		boost_ep_disable, NULL));
	WARN_ON(register_trace_musb_g_giveback(
		musb_g_giveback_boost, NULL));
	WARN_ON(register_trace_android_vh_sound_usb_support_cpu_suspend(
		vh_sound_usb_support_cpu_suspend, NULL));
	WARN_ON(register_trace_musb_host_urb_giveback(
		musb_host_urb_giveback_dbg, NULL));
	return 0;
}
#endif

int usb_boost_init(void)
{
	int id;

	for (id = 0; id < _TYPE_MAXID; id++) {
		int count;
		char wq_name[MAX_LEN_WQ_NAME];

		count = sprintf(wq_name, "%s_wq", type_name[id]);
		wq_name[count] = '\0';
		boost_inst[id].id  = id;
		update_time(id);
		boost_inst[id].wq  = create_singlethread_workqueue(wq_name);
		INIT_WORK(&boost_inst[id].work, boost_work);
		USB_BOOST_DBG("ID<%d>, WQ<%p>, WORK<%p>\n",
			id, boost_inst[id].wq, &(boost_inst[id].work));
		boost_inst[id].request_func = __request_it;
	}
	/* hook workable interface */
	__the_boost_ops.boost = __usb_boost;
	enabled = 1;

	/* usb audio fine tune */
	audio_boost_inst.wq = create_singlethread_workqueue("usb_audio_wq");
	INIT_WORK(&audio_boost_inst.work, audio_boost_work);
	/* default off */
	audio_boost_inst.request_func = __request_empty;
	/* wakelock */
	usb_boost_ws = wakeup_source_register(NULL, "usb_boost");

	create_sys_fs();
	default_setting();
	inited = 1;

#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	musb_trace_init();
#endif

	return 0;
}


void usb_audio_boost(bool enable)
{
	USB_BOOST_NOTICE("%s: enable:%d\n", __func__, enable);

	if (enable) {
		/* hook workable interface */
		audio_boost_inst.request_func = __request_audio;
	}
}

module_param(trigger_cnt_disabled, int, 0400);
module_param(enabled, int, 0400);
module_param(inited, int, 0400);
MODULE_LICENSE("GPL v2");
