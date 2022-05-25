/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_private_api.h
** Description : oplus display private api implement
** Version : 1.0
** Date : 2018/03/20
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/20        1.0           Build this moudle
**   Guo.Ling        2018/10/11        1.1           Modify for SDM660
**   Guo.Ling        2018/11/27        1.2           Modify for mt6779
**   Li.Ping         2019/11/18        1.3           Modify for mt6885
******************************************************************/
#include "oplus_display_private_api.h"
#include "mtk_disp_aal.h"
#include "oplus_display_panel_power.h"
#include <linux/notifier.h>
#include <linux/kobject.h>
#include <linux/signal.h>

#include <soc/oplus/system/oplus_project.h>
#include "oplus_adfr.h"

#include "oplus_display_onscreenfingerprint.h"


/*
 * we will create a sysfs which called /sys/kernel/oplus_display,
 * In that directory, oplus display private api can be called
 */

#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_SERIAL_NUM_REG_SUMSUNG 0xD8
#define PANEL_REG_READ_LEN   10
#define BOE_PANEL_SERIAL_NUM_REG 0xA3
#define PANEL_SERIAL_NUM_REG_TIANMA 0xD6

#define OPLUS_ATTR(_name, _mode, _show, _store) \
struct kobj_attribute oplus_attr_##_name = __ATTR(_name, _mode, _show, _store)

uint64_t serial_number = 0x0;
EXPORT_SYMBOL(serial_number);
extern void mtk_read_ddic_v2(u8 ddic_reg, int ret_num, char ret_val[10]);

struct aod_area oplus_aod_area[RAMLESS_AOD_AREA_NUM];
char send_cmd[RAMLESS_AOD_PAYLOAD_SIZE];

typedef struct panel_serial_info
{
	int reg_index;
	uint64_t year;
	uint64_t month;
	uint64_t day;
	uint64_t hour;
	uint64_t minute;
	uint64_t second;
	uint64_t reserved[2];
} PANEL_SERIAL_INFO;

extern int dsi_cmd_log_enable;
extern int trig_db_enable;
extern struct drm_device* get_drm_device(void);
extern int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level);
extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);
extern PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX];
//extern struct drm_panel *p_node;
//extern  struct mtk_panel_funcs *funcs;
unsigned int esd_mode = 0;
EXPORT_SYMBOL(esd_mode);
unsigned int seed_mode = 0;
unsigned int oplus_display_brightness = 0;

unsigned int oplus_max_normal_brightness = 0;
unsigned int m_da;
unsigned int m_db;
unsigned int m_dc;
bool g_dp_support;

//extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int oplus_display_get_softiris_color_status(void *buf);


static BLOCKING_NOTIFIER_HEAD(lcdinfo_notifiers);
int register_lcdinfo_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lcdinfo_notifiers, nb);
}
int unregister_lcdinfo_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&lcdinfo_notifiers, nb);
}

EXPORT_SYMBOL(register_lcdinfo_notifier);
EXPORT_SYMBOL(unregister_lcdinfo_notifier);

void lcdinfo_notify(unsigned long val, void *v)
{
	blocking_notifier_call_chain(&lcdinfo_notifiers, val, v);
}
EXPORT_SYMBOL(lcdinfo_notify);



static ssize_t oplus_display_get_brightness(struct kobject *obj,
                                struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oplus_display_brightness);
}

static ssize_t oplus_display_set_brightness(struct kobject *obj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int oplus_set_brightness = 0;

	ret = kstrtouint(buf, 10, &oplus_set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > OPLUS_MAX_BRIGHTNESS || oplus_set_brightness < OPLUS_MIN_BRIGHTNESS) {
		printk(KERN_ERR "%s, brightness:%d out of scope\n", __func__, oplus_set_brightness);
		return num;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}
	mtk_drm_setbacklight(crtc, oplus_set_brightness);

	return num;
}

static ssize_t oplus_display_get_max_brightness(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", OPLUS_MAX_BRIGHTNESS);
}

static ssize_t oplus_display_get_maxbrightness(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", oplus_max_normal_brightness);
}

static ssize_t oplus_display_set_panel_pwr(struct kobject *obj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t num)
{
	u32 panel_vol_value = 0, panel_vol_id = 0;
	int rc = 0;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
		typeof(*crtc), head);
	if (!crtc) {
		DDPPR_ERR("get hbm find crtc fail\n");
		return 0;
	}
	mtk_crtc = to_mtk_crtc(crtc);

	sscanf(buf, "%d %d", &panel_vol_id, &panel_vol_value);
	panel_vol_id = panel_vol_id & 0x0F;

	pr_err("debug for %s, buf = [%s], id = %d value = %d, num = %d\n",
		__func__, buf, panel_vol_id, panel_vol_value, num);

	if (panel_vol_id < 0 || panel_vol_id > PANEL_VOLTAGE_ID_MAX) {
		return -EINVAL;
	}

	if (panel_vol_value < panel_vol_bak[panel_vol_id].voltage_min ||
		panel_vol_id > panel_vol_bak[panel_vol_id].voltage_max) {
		return -EINVAL;
	}

	if (panel_vol_id == PANEL_VOLTAGE_ID_VG_BASE) {
		pr_err("%s: set the VGH_L pwr = %d \n", __func__, panel_vol_value);
		rc = oplus_panel_set_vg_base(panel_vol_value);
		if (rc < 0) {
			return rc;
		}

		return num;
	}

	if (mtk_crtc->panel_ext->funcs->oplus_set_power) {
		rc = mtk_crtc->panel_ext->funcs->oplus_set_power(panel_vol_id, panel_vol_value);
		if (rc) {
			pr_err("Set voltage(%s) fail, rc=%d\n",
				 __func__, rc);
			return -EINVAL;
		}
	}

	return num;
}

static ssize_t oplus_display_get_panel_pwr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf) {
	int ret = 0;
	u32 i = 0;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
		typeof(*crtc), head);
	if (!crtc) {
		DDPPR_ERR("get hbm find crtc fail\n");
		return 0;
	}
	mtk_crtc = to_mtk_crtc(crtc);

	for (i = 0; i < (PANEL_VOLTAGE_ID_MAX-1); i++) {
		if (mtk_crtc->panel_ext->funcs->oplus_update_power_value) {
			ret = mtk_crtc->panel_ext->funcs->oplus_update_power_value(panel_vol_bak[i].voltage_id);
		}

		if (ret < 0) {
			pr_err("%s : update_current_voltage error = %d\n", __func__, ret);
		}
		else {
			panel_vol_bak[i].voltage_current = ret;
		}
	}

	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
		panel_vol_bak[0].voltage_id, panel_vol_bak[0].voltage_min,
		panel_vol_bak[0].voltage_current, panel_vol_bak[0].voltage_max,
		panel_vol_bak[1].voltage_id, panel_vol_bak[1].voltage_min,
		panel_vol_bak[1].voltage_current, panel_vol_bak[1].voltage_max,
		panel_vol_bak[2].voltage_id, panel_vol_bak[2].voltage_min,
		panel_vol_bak[2].voltage_current, panel_vol_bak[2].voltage_max);
}

static ssize_t oplus_display_get_dsi_cmd_log_switch(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", dsi_cmd_log_enable);
}

static ssize_t oplus_display_set_dsi_cmd_log_switch(struct kobject *obj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &dsi_cmd_log_enable);
	pr_err("debug for %s, dsi_cmd_log_enable = %d\n",
						 __func__, dsi_cmd_log_enable);

	return count;
}

static ssize_t oplus_display_get_disp_trig_db(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", trig_db_enable);
}

static ssize_t oplus_display_set_disp_trig_db(struct kobject *obj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &trig_db_enable);
	pr_err("debug for %s, trig_db_enable = %d\n", __func__, trig_db_enable);

	if (get_eng_version() == AGING) {
		pr_err("disp_trig_db, AGING version\n");
	} else {
		pr_err("disp_trig_db, normal version\n");
	}

	return count;
}

void oplus_kill_surfaceflinger(void) {
	 struct task_struct *p;
	 read_lock(&tasklist_lock);
	 for_each_process(p) {
			 get_task_struct(p);
			 if (strcmp(p->comm, "surfaceflinger") == 0) {
					send_sig_info(SIGABRT, SEND_SIG_PRIV, p);
			 }
			 put_task_struct(p);
	 }
	 read_unlock(&tasklist_lock);
}
EXPORT_SYMBOL(oplus_kill_surfaceflinger);

int panel_serial_number_read(struct drm_crtc *crtc, char cmd, int num)
{
	char para[20] = {0};
	char *panel_name;
	int count = 10;
	PANEL_SERIAL_INFO panel_serial_info;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_crtc *mtk_crtc;

	mtk_crtc = to_mtk_crtc(crtc);
	comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!(mtk_crtc->enabled)) {
		pr_err("[lh]Sleep State set backlight stop --crtc not ebable\n");
		return 0;
	}

	if (!comp || !comp->funcs || !comp->funcs->io_cmd) {
		pr_err("[lh]cannot find output component\n");
		return 0;
	}

	mtk_ddp_comp_io_cmd(comp, NULL, GET_PANEL_NAME, &panel_name);
	pr_info("[oplus]panelname=%s\n", panel_name);

	if (!strcmp(panel_name, "oplus21007_samsung_amb670yf01_qhd_dsi_cmd")) {
		g_dp_support = true;
	} else {
		g_dp_support = false;
	}

	while (count > 0) {
		mtk_read_ddic_v2(cmd, num, para);
		count--;
		panel_serial_info.reg_index = 4;
		panel_serial_info.month     = para[panel_serial_info.reg_index] & 0x0F;
		panel_serial_info.year      = (para[panel_serial_info.reg_index] & 0xF0) >> 4;
		panel_serial_info.day       = para[panel_serial_info.reg_index + 1] & 0x3F;
		panel_serial_info.hour      = para[panel_serial_info.reg_index + 2] & 0x3F;
		panel_serial_info.minute    = para[panel_serial_info.reg_index + 3] & 0x3F;
		panel_serial_info.second    = para[panel_serial_info.reg_index + 4] & 0x3F;
		panel_serial_info.reserved[0] = para[panel_serial_info.reg_index + 5];
		panel_serial_info.reserved[1] = para[panel_serial_info.reg_index + 6];

		serial_number = (panel_serial_info.year     << 56)\
							+ (panel_serial_info.month      << 48)\
							+ (panel_serial_info.day        << 40)\
							+ (panel_serial_info.hour       << 32)\
							+ (panel_serial_info.minute << 24)\
							+ (panel_serial_info.second << 16)\
							+ (panel_serial_info.reserved[0] << 8)\
							+ (panel_serial_info.reserved[1]);
		pr_info("[oplus]%s year:0x%llx, month:0x%llx, day:0x%llx, hour:0x%llx, minute:0x%llx, second:0x%llx, msecond:0x%llx, 0x%llx!\n",
						__func__,
						panel_serial_info.year,
						panel_serial_info.month,
						panel_serial_info.day,
						panel_serial_info.hour,
						panel_serial_info.minute,
						panel_serial_info.second,
						panel_serial_info.reserved[0],
						panel_serial_info.reserved[1]);
		if (panel_serial_info.year <= 7) {
			continue;
		} else {
			printk("%s year:0x%llx, month:0x%llx, day:0x%llx, hour:0x%llx, minute:0x%llx, second:0x%llx,  msecond:0x%llx, 0x%llx!\n",
			            __func__,
			            panel_serial_info.year,
			            panel_serial_info.month,
			            panel_serial_info.day,
			            panel_serial_info.hour,
			            panel_serial_info.minute,
			            panel_serial_info.second,
			            panel_serial_info.reserved[0],
			            panel_serial_info.reserved[1]);
			break;
		}
	}
	pr_err("[oplus]%s Get panel serial number=[0x%llx]\n", __func__, serial_number);
	mtk_read_ddic_v2(0xDA, 5, para);
	m_da = para[0] & 0xFF;

	mtk_read_ddic_v2(0xDB, 5, para);
	m_db = para[0] & 0xFF;

	mtk_read_ddic_v2(0xDC, 5, para);
	m_dc = para[0] & 0xFF;

	pr_err("[oplus]%s: 0xDA=0x%x, 0xDB=0x%x, 0xDC=0x%x\n", __func__, m_da, m_db, m_dc);
	return 1;
}

static ssize_t oplus_get_panel_serial_number(struct kobject *obj,
                struct kobj_attribute *attr, char *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
		typeof(*crtc), head);
	if (!crtc) {
		DDPPR_ERR("get_panel_serial_number find crtc fail\n");
		return 0;
	}
	if (serial_number == 0) {
		printk(KERN_ERR "dsi_cmd %s Failed. serial_number == 0,run panel_serial_number_read()\n", __func__);
		panel_serial_number_read(crtc, PANEL_SERIAL_NUM_REG, PANEL_REG_READ_LEN);
		printk(KERN_ERR "%s .after read, serial_number: %llx, da=0x%llx, db=0x%llx, dc=0x%llx\n", __func__, serial_number, m_da, m_db, m_dc);
	}

	return scnprintf(buf, PAGE_SIZE, "Get panel serial number: %llx\n", serial_number);
}

static ssize_t panel_serial_store(struct kobject *obj,
                struct kobj_attribute *attr,
                const char *buf, size_t count)
{
        printk("[soso] Lcm read 0xA1 reg = 0x%llx\n", serial_number);
        return count;
}


int oplus_dc_threshold = 260;
int oplus_panel_alpha = 0;
int oplus_underbrightness_alpha = 0;
int alpha_save = 0;

EXPORT_SYMBOL(oplus_underbrightness_alpha);
EXPORT_SYMBOL(oplus_panel_alpha);


struct ba {
	u32 brightness;
	u32 alpha;
};
struct ba brightness_seed_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xfc},
	{2, 0xfb},
	{3, 0xfa},
	{4, 0xf9},
	{5, 0xf8},
	{6, 0xf7},
	{8, 0xf6},
	{10, 0xf4},
	{15, 0xf0},
	{20, 0xea},
	{30, 0xe0},
	{45, 0xd0},
	{70, 0xbc},
	{100, 0x98},
	{120, 0x80},
	{140, 0x70},
	{160, 0x58},
	{180, 0x48},
	{200, 0x30},
	{220, 0x20},
	{240, 0x10},
	{260, 0x00},
};
struct ba brightness_alpha_lut[] = {
	{0, 0xFF},
	{5, 0xEE},
	{7, 0xED},
	{10, 0xE7},
	{20, 0xE3},
	{35, 0xDC},
	{60, 0xD1},
	{90, 0xCE},
	{150, 0xC1},
	{280, 0xAA},
	{460, 0x95},
	{650, 0x7F},
	{850, 0x79},
	{1000, 0x6E},
	{1150, 0x62},
	{1300, 0x52},
	{1500, 0x4C},
	{1700, 0x42},
	{1900, 0x35},
	{2047, 0x24},
};

struct ba brightness_alpha_lut_BOE[] = {
	{0, 0xFF},
	{12, 0xEE},
	{20, 0xE8},
	{35, 0xE5},
	{65, 0xDA},
	{100, 0xD8},
	{150, 0xCD},
	{210, 0xC5},
	{320, 0xB9},
	{450, 0xA9},
	{630, 0xA0},
	{870, 0x94},
	{1150, 0x86},
	{1500, 0x7B},
	{1850, 0x6B},
	{2250, 0x66},
	{2650, 0x55},
	{3050, 0x47},
	{3400, 0x39},
	{3515, 0x24},
};

struct ba brightness_alpha_lut_index[] = {
	{0, 0xFF},
	{150, 0xEE},
	{190, 0xEB},
	{230, 0xE6},
	{270, 0xE1},
	{310, 0xDA},
	{350, 0xD7},
	{400, 0xD3},
	{450, 0xD0},
	{500, 0xCD},
	{600, 0xC3},
	{700, 0xB8},
	{900, 0xA3},
	{1100, 0x90},
	{1300, 0x7B},
	{1400, 0x70},
	{1500, 0x65},
	{1700, 0x4E},
	{1900, 0x38},
	{2047, 0x23},
};

static int interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;
	if ((xa - xb) && (yb - ya))
		sub = 2 * (x - xa) * (x - xb) / (yb - ya) / (xa - xb);

	return ya + factor + plus + sub;
}

int bl_to_alpha(int brightness)
{
	int alpha;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;

	int i = 0;
	int level = 0;
	struct drm_device *ddev = get_drm_device();

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return 0;
	}

/*	if(!strcmp(mtk_crtc->panel_ext->params->manufacture, "boe_nt37800_2048")) {
		level = ARRAY_SIZE(brightness_alpha_lut_BOE);
		for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut_BOE); i++) {
			if (brightness_alpha_lut_BOE[i].brightness >= brightness)
				break;
		}

		if (i == 0)
			alpha = brightness_alpha_lut_BOE[0].alpha;
		else if (i == level)
			alpha = brightness_alpha_lut_BOE[level - 1].alpha;
		else
			alpha = interpolate(brightness,
				brightness_alpha_lut_BOE[i-1].brightness,
				brightness_alpha_lut_BOE[i].brightness,
				brightness_alpha_lut_BOE[i-1].alpha,
				brightness_alpha_lut_BOE[i].alpha);
	}
	else {*/
		level = ARRAY_SIZE(brightness_alpha_lut);
		for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut); i++) {
			if (brightness_alpha_lut[i].brightness >= brightness)
				break;
		}

		if (i == 0)
			alpha = brightness_alpha_lut[0].alpha;
		else if (i == level)
			alpha = brightness_alpha_lut[level - 1].alpha;
		else
			alpha = interpolate(brightness,
				brightness_alpha_lut[i-1].brightness,
				brightness_alpha_lut[i].brightness,
				brightness_alpha_lut[i-1].alpha,
				brightness_alpha_lut[i].alpha);
//	}

	return alpha;
}

int brightness_to_alpha(int brightness)
{
	int alpha;

	if (brightness <= 3)
		return alpha_save;

	alpha = bl_to_alpha(brightness);

	alpha_save = alpha;

	return alpha;
}

int oplus_seed_bright_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_seed_alpha_lut_dc); i++) {
		if (brightness_seed_alpha_lut_dc[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_seed_alpha_lut_dc[0].alpha;
	else if (i == level)
		alpha = brightness_seed_alpha_lut_dc[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_seed_alpha_lut_dc[i-1].brightness,
			brightness_seed_alpha_lut_dc[i].brightness,
			brightness_seed_alpha_lut_dc[i-1].alpha,
			brightness_seed_alpha_lut_dc[i].alpha);

	return alpha;
}

int oplus_get_panel_brightness_to_alpha(void)
{
	if (oplus_panel_alpha)
		return oplus_panel_alpha;

	return brightness_to_alpha(oplus_display_brightness);
}
EXPORT_SYMBOL(oplus_get_panel_brightness_to_alpha);

static ssize_t oplus_display_get_dim_alpha(struct kobject *obj,
                                struct kobj_attribute *attr, char *buf)
{
	if (!oplus_ofp_get_hbm_state()) {
		return sprintf(buf, "%d\n", 0);
	}
	oplus_underbrightness_alpha = oplus_get_panel_brightness_to_alpha();

	return sprintf(buf, "%d\n", oplus_underbrightness_alpha);
}


static ssize_t oplus_display_set_dim_alpha(struct kobject *obj,
                               struct kobj_attribute *attr,
                               const char *buf, size_t count)
{
	sscanf(buf, "%x", &oplus_panel_alpha);
	return count;
}

unsigned int silence_mode = 0;
EXPORT_SYMBOL(silence_mode);

static ssize_t silence_show(struct kobject *obj,
			struct kobj_attribute *attr, char *buf)
{
	printk("%s silence_mode=%d\n", __func__, silence_mode);
	return sprintf(buf, "%d\n", silence_mode);
}

static ssize_t silence_store(struct kobject *obj,
		struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret;
	msleep(1000);
	ret = kstrtouint(buf, 10, &silence_mode);
	printk("%s silence_mode=%d\n", __func__, silence_mode);
	return num;
}

/*unsigned long CABC_mode = 2;
*
* add dre only use for camera
*
* extern void disp_aal_set_dre_en(int enable);*/

/*static ssize_t LCM_CABC_show(struct kobject *obj,
                struct kobj_attribute *attr, char *buf)
{
    printk("%s CABC_mode=%d\n", __func__, CABC_mode);
    return sprintf(buf, "%d\n", CABC_mode);
}

static ssize_t LCM_CABC_store(struct kobject *obj,
        struct kobj_attribute *attr, const char *buf, size_t num)
{
    int ret = 0;

    ret = kstrtouint(buf, 10, &CABC_mode);
    if( CABC_mode > 3 ){
        CABC_mode = 3;
    }
    printk("%s CABC_mode=%d\n", __func__, CABC_mode);

    if (CABC_mode == 0) {
        disp_aal_set_dre_en(1);
        printk("%s enable dre\n", __func__);

    } else {
        disp_aal_set_dre_en(0);
        printk("%s disable dre\n", __func__);
    }

    if (oplus_display_cabc_support) {
        ret = primary_display_set_cabc_mode((unsigned int)CABC_mode);
    }

    return num;
}*/

static ssize_t oplus_display_get_ESD(struct kobject *obj,
                struct kobj_attribute *attr, char *buf)
{
	printk("%s esd=%d\n", __func__, esd_mode);
	return sprintf(buf, "%d\n", esd_mode);
}

static ssize_t oplus_display_set_ESD(struct kobject *obj,
        struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret = 0;

	ret = kstrtouint(buf, 10, &esd_mode);
	printk("%s,esd mode is %d\n", __func__, esd_mode);
	return num;
}

unsigned int cabc_mode = 1;
unsigned int cabc_true_mode = 1;
unsigned int cabc_sun_flag = 0;
unsigned int cabc_back_flag = 1;
extern void disp_aal_set_dre_en(int enable);

enum{
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_2 = 3,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
};

static ssize_t oplus_display_get_CABC(struct kobject *obj,
                struct kobj_attribute *attr, char *buf)
{
	printk("%s CABC_mode=%d\n", __func__, cabc_true_mode);
	return sprintf(buf, "%d\n", cabc_true_mode);
}

static ssize_t oplus_display_set_CABC(struct kobject *obj,
        struct kobj_attribute *attr, const char *buf, size_t num)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();

	ret = kstrtouint(buf, 10, &cabc_mode);
	cabc_true_mode = cabc_mode;
	printk("%s,cabc mode is %d, cabc_back_flag is %d\n", __func__, cabc_mode, cabc_back_flag);
	if(cabc_mode < 4)
		cabc_back_flag = cabc_mode;

	if (cabc_mode == CABC_ENTER_SPECIAL) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (cabc_mode == CABC_EXIT_SPECIAL) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (cabc_sun_flag == 1) {
		if (cabc_back_flag == CABC_LEVEL_0) {
			disp_aal_set_dre_en(1);
			printk("%s sun enable dre\n", __func__);
		} else {
			disp_aal_set_dre_en(0);
			printk("%s sun disable dre\n", __func__);
		}
		return num;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}
	if (cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0) {
		disp_aal_set_dre_en(1);
		printk("%s enable dre\n", __func__);
	} else {
		disp_aal_set_dre_en(0);
		printk("%s disable dre\n", __func__);
	}
//	oplus_mtk_drm_setcabc(crtc, cabc_true_mode);
	if (cabc_true_mode != cabc_back_flag) cabc_true_mode = cabc_back_flag;

	return num;
}

#if 0
unsigned char aod_area_set_flag = 0;
static ssize_t oplus_display_set_aod_area(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	char *bufp = (char *)buf;
	char *token;
	int i, cnt = 0;
	char payload[RAMLESS_AOD_PAYLOAD_SIZE];

	memset(oplus_aod_area, 0, sizeof(struct aod_area) * RAMLESS_AOD_AREA_NUM);

	pr_err("yzq: %s %d\n", __func__, __LINE__);
	while ((token = strsep(&bufp, ":")) != NULL) {
		struct aod_area *area = &oplus_aod_area[cnt];
		if (!*token)
			continue;

		sscanf(token, "%d %d %d %d %d %d %d %d",
			&area->x, &area->y, &area->w, &area->h,
			&area->color, &area->bitdepth, &area->mono, &area->gray);
		pr_err("yzq: %s %d rect[%dx%d-%dx%d]-%d-%d-%d-%x\n", __func__, __LINE__,
			area->x, area->y, area->w, area->h,
			area->color, area->bitdepth, area->mono, area->gray);
		area->enable = true;
		cnt++;
	}

	memset(payload, 0, RAMLESS_AOD_PAYLOAD_SIZE);
	memset(send_cmd, 0, RAMLESS_AOD_PAYLOAD_SIZE);

	for (i = 0; i < RAMLESS_AOD_AREA_NUM; i++) {
		struct aod_area *area = &oplus_aod_area[i];

		payload[0] |= (!!area->enable) << (RAMLESS_AOD_AREA_NUM - i - 1);
		if (area->enable) {
			int h_start = area->x;
			int h_block = area->w / 100;
			int v_start = area->y;
			int v_end = area->y + area->h;
			int off = i * 5;

			/* Rect Setting */
			payload[1 + off] = h_start >> 4;
			payload[2 + off] = ((h_start & 0xf) << 4) | (h_block & 0xf);
			payload[3 + off] = v_start >> 4;
			payload[4 + off] = ((v_start & 0xf) << 4) | ((v_end >> 8) & 0xf);
			payload[5 + off] = v_end & 0xff;

			/* Mono Setting */
			#define SET_MONO_SEL(index, shift) \
				if (i == index) \
					payload[31] |= area->mono << shift;

			SET_MONO_SEL(0, 6);
			SET_MONO_SEL(1, 5);
			SET_MONO_SEL(2, 4);
			SET_MONO_SEL(3, 2);
			SET_MONO_SEL(4, 1);
			SET_MONO_SEL(5, 0);
			#undef SET_MONO_SEL

			/* Depth Setting */
			if (i < 4)
				payload[32] |= (area->bitdepth & 0x3) << ((3 - i) * 2);
			else if (i == 4)
				payload[33] |= (area->bitdepth & 0x3) << 6;
			else if (i == 5)
				payload[33] |= (area->bitdepth & 0x3) << 4;
			/* Color Setting */
			#define SET_COLOR_SEL(index, reg, shift) \
				if (i == index) \
					payload[reg] |= (area->color & 0x7) << shift;
			SET_COLOR_SEL(0, 34, 4);
			SET_COLOR_SEL(1, 34, 0);
			SET_COLOR_SEL(2, 35, 4);
			SET_COLOR_SEL(3, 35, 0);
			SET_COLOR_SEL(4, 36, 4);
			SET_COLOR_SEL(5, 36, 0);
			#undef SET_COLOR_SEL
			/* Area Gray Setting */
			payload[37 + i] = area->gray & 0xff;
		}
	}
	payload[43] = 0x00;
	send_cmd[0] = 0x81;
	for(i = 0; i< 44; i++){
		pr_err("payload[%d] = 0x%x- send_cmd[%d] = 0x%x-", i,payload[i],i,send_cmd[i]);
		send_cmd[i+1] = payload[i];
	}
	aod_area_set_flag = 1;
	return count;
}
#endif

static ssize_t oplus_display_get_seed(struct kobject *obj,
		struct kobj_attribute *attr, char *buf)
{
        printk(KERN_INFO "seed_mode = %d\n", seed_mode);

        return sprintf(buf, "%d\n", seed_mode);
}

static ssize_t oplus_display_set_seed(struct kobject *obj,
                struct kobj_attribute *attr,
                const char *buf, size_t count) {
	struct drm_crtc *crtc;
	unsigned int temp_save = 0;
	int ret = 0;
	struct drm_device *ddev = get_drm_device();

	ret = kstrtouint(buf, 10, &temp_save);
	printk(KERN_INFO "seed = %d\n", temp_save);
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	oplus_mtk_drm_setseed(crtc, temp_save);
	seed_mode = temp_save;

	return count;
}

struct softiris_color
{
	uint32_t color_vivid_status;
	uint32_t color_srgb_status;
	uint32_t color_softiris_status;
	uint32_t color_dual_panel_status;
	uint32_t color_dual_brightness_status;
};

static struct softiris_color *data;
static ssize_t oplus_display_get_color_status(struct kobject *obj,
				struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	data = kzalloc(sizeof(struct softiris_color), GFP_KERNEL);

	ret = oplus_display_get_softiris_color_status((void*)data);
	printk(KERN_INFO "ret = %d\n", ret);

	return sprintf(buf, "vivid:%s, srgb:%s,softiris:%s,dual_panel:%s,dual_brightness:%s\n", data->color_vivid_status ? "true" : "false", \
	 data->color_srgb_status ? "true" : "false", data->color_softiris_status ? "true" : "false", data->color_dual_panel_status ? "true" : "false", \
	 data->color_dual_brightness_status ? "true" : "false");
}


static struct kobject *oplus_display_kobj;
static OPLUS_ATTR(oplus_brightness, S_IRUGO|S_IWUSR, oplus_display_get_brightness, oplus_display_set_brightness);
static OPLUS_ATTR(oplus_max_brightness, S_IRUGO|S_IWUSR, oplus_display_get_max_brightness, NULL);
static OPLUS_ATTR(oplus_get_color_status, S_IRUGO|S_IWUSR, oplus_display_get_color_status, NULL);
static OPLUS_ATTR(max_brightness, S_IRUGO|S_IWUSR, oplus_display_get_maxbrightness, NULL);
static OPLUS_ATTR(seed, S_IRUGO|S_IWUSR, oplus_display_get_seed, oplus_display_set_seed);
static OPLUS_ATTR(panel_pwr, S_IRUGO|S_IWUSR, oplus_display_get_panel_pwr, oplus_display_set_panel_pwr);
static OPLUS_ATTR(dim_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_alpha, oplus_display_set_dim_alpha);
//static OPLUS_ATTR(dimlayer_bl_en, S_IRUGO|S_IWUSR, oplus_display_get_dc_enable, oplus_display_set_dc_enable);
//static OPLUS_ATTR(dim_dc_alpha, S_IRUGO|S_IWUSR, oplus_display_get_dim_dc_alpha, oplus_display_set_dim_dc_alpha);
static OPLUS_ATTR(panel_serial_number, S_IRUGO|S_IWUSR, oplus_get_panel_serial_number, panel_serial_store);
static OPLUS_ATTR(LCM_CABC, S_IRUGO|S_IWUSR, oplus_display_get_CABC, oplus_display_set_CABC);
static OPLUS_ATTR(esd, S_IRUGO|S_IWUSR, oplus_display_get_ESD, oplus_display_set_ESD);
static OPLUS_ATTR(sau_closebl_node, S_IRUGO|S_IWUSR, silence_show, silence_store);

static OPLUS_ATTR(oplus_notify_fppress, S_IRUGO|S_IWUSR, NULL, oplus_ofp_notify_fp_press_attr);
static OPLUS_ATTR(hbm, S_IRUGO|S_IWUSR, oplus_ofp_get_hbm_attr, oplus_ofp_set_hbm_attr);
static OPLUS_ATTR(aod_light_mode_set, S_IRUGO|S_IWUSR, oplus_ofp_get_aod_light_mode_attr, oplus_ofp_set_aod_light_mode_attr);
static OPLUS_ATTR(dsi_cmd_log_switch, S_IRUGO|S_IWUSR, oplus_display_get_dsi_cmd_log_switch,
	oplus_display_set_dsi_cmd_log_switch);

static OPLUS_ATTR(adfr_debug, S_IRUGO|S_IWUSR, oplus_adfr_get_debug, oplus_adfr_set_debug);
static OPLUS_ATTR(adfr_params, S_IRUGO|S_IWUSR, oplus_adfr_get_params, oplus_adfr_set_params);
/* add for mux switch control */
static OPLUS_ATTR(vsync_switch, S_IRUGO|S_IWUSR, oplus_get_vsync_switch, oplus_set_vsync_switch);

static OPLUS_ATTR(disp_trig_db, S_IRUGO|S_IWUSR, oplus_display_get_disp_trig_db,
                        oplus_display_set_disp_trig_db);

EXPORT_SYMBOL(seed_mode);
EXPORT_SYMBOL(oplus_max_normal_brightness);
EXPORT_SYMBOL(oplus_display_brightness);
EXPORT_SYMBOL(cabc_mode);
EXPORT_SYMBOL(cabc_true_mode);
EXPORT_SYMBOL(cabc_sun_flag);
EXPORT_SYMBOL(cabc_back_flag);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *oplus_display_attrs[] = {
	&oplus_attr_oplus_brightness.attr,
	&oplus_attr_oplus_max_brightness.attr,
	&oplus_attr_oplus_get_color_status.attr,
	&oplus_attr_max_brightness.attr,
	&oplus_attr_seed.attr,
	&oplus_attr_panel_pwr.attr,
	&oplus_attr_dim_alpha.attr,
	//&oplus_attr_dimlayer_bl_en.attr,
	//&oplus_attr_dim_dc_alpha.attr,
	&oplus_attr_panel_serial_number.attr,
	&oplus_attr_LCM_CABC.attr,
	&oplus_attr_esd.attr,
	&oplus_attr_sau_closebl_node.attr,

	&oplus_attr_oplus_notify_fppress.attr,
	&oplus_attr_hbm.attr,
	&oplus_attr_aod_light_mode_set.attr,
	&oplus_attr_dsi_cmd_log_switch.attr,

	&oplus_attr_adfr_debug.attr,
	&oplus_attr_adfr_params.attr,
	/* add for mux switch control */
	&oplus_attr_vsync_switch.attr,

	&oplus_attr_disp_trig_db.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oplus_display_attr_group = {
	.attrs = oplus_display_attrs,
};

int oplus_display_private_api_init(void)
{
	int retval;

	oplus_display_kobj = kobject_create_and_add("oplus_display", kernel_kobj);
	if (!oplus_display_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(oplus_display_kobj, &oplus_display_attr_group);
	if (retval)
		kobject_put(oplus_display_kobj);

	return retval;
}

void oplus_display_private_api_exit(void)
{
	kobject_put(oplus_display_kobj);
}

//module_init(oplus_display_private_api_init);
//module_exit(oplus_display_private_api_exit);
//MODULE_LICENSE("GPL v2");
//MODULE_AUTHOR("Hujie <hujie@oplus.com>");
