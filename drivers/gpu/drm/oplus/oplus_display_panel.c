/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_panel.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#include "oplus_display_panel.h"
#include "oplus_display_onscreenfingerprint.h"
#include <linux/slab.h>
#include <linux/uaccess.h>

/*
*set_seed equals to dim_dc_alpha
*dc_enable equals to dimlay_bl_en
*/
extern int oplus_display_panel_set_pwr(void *buf);
extern int oplus_display_panel_get_pwr(void *buf);
extern int oplus_display_panel_get_max_brightness(void *buf);
extern int oplus_display_panel_get_serial_number(void *buf);
extern int oplus_display_panel_set_dim_alpha(void *buf);
extern int oplus_display_panel_get_dim_alpha(void *buf);
extern int oplus_display_panel_set_dim_dc_alpha(void *buf);
extern int oplus_display_panel_get_dim_dc_alpha(void *buf);
extern int oplus_display_panel_set_closebl_flag(void *buf);
extern int oplus_display_panel_get_closebl_flag(void *buf);
extern int oplus_display_panel_set_dimlayer_enable(void *buf);
extern int oplus_display_panel_get_dimlayer_enable(void *buf);
extern int oplus_display_set_brightness(void *buf);
extern int oplus_display_get_brightness(void *buf);
extern int oplus_display_panel_set_cabc(void *buf);
extern int oplus_display_panel_get_cabc(void *buf);
extern int oplus_display_panel_get_panel_bpp(void *buf);
extern int oplus_display_panel_set_esd(void *buf);
extern int oplus_display_panel_get_esd(void *buf);
extern int oplus_display_set_mtk_loglevel(void *buf);
extern int oplus_display_panel_get_vendor(void *buf);
extern int oplus_display_panel_set_seed(void *buf);
extern int oplus_display_panel_get_seed(void *buf);
extern int oplus_display_get_softiris_color_status(void *buf);
extern int oplus_display_panel_get_id(void *buf);
extern int oplus_display_get_dp_support(void *buf);
extern int oplus_display_set_limit_fps(void *buf);
extern int oplus_display_panel_get_pq_trigger(void *buf);
extern int oplus_display_panel_set_pq_trigger(void *buf);
extern int oplus_display_panel_set_pwm_status(void *buf);
extern int oplus_display_panel_get_pwm_status(void *buf);
extern int oplus_display_panel_get_pwm_status_for_90hz(void *buf);
extern int oplus_display_panel_get_panel_type(void *buf);


static const struct panel_ioctl_desc panel_ioctls[] = {
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_POWER, oplus_display_panel_set_pwr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_POWER, oplus_display_panel_get_pwr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_SEED, oplus_display_panel_set_seed),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SEED, oplus_display_panel_get_seed),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_AOD, oplus_ofp_set_aod_light_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_AOD, oplus_ofp_get_aod_light_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANELID, oplus_display_panel_get_id),
	/*PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FFL, oplus_display_panel_set_ffl),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_FFL, oplus_display_panel_get_ffl),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_MAX_BRIGHTNESS, oplus_display_panel_set_max_brightness),*/
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_MAX_BRIGHTNESS, oplus_display_panel_get_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANELINFO, oplus_display_panel_get_vendor),
	/*PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CCD, oplus_display_panel_get_ccd_check),*/
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SERIAL_NUMBER, oplus_display_panel_get_serial_number),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_HBM, oplus_ofp_set_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_HBM, oplus_ofp_get_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIM_ALPHA, oplus_display_panel_set_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIM_ALPHA, oplus_display_panel_get_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIM_DC_ALPHA, oplus_display_panel_set_dim_dc_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIM_DC_ALPHA, oplus_display_panel_get_dim_dc_alpha),
    /*PANEL_IOCTL_DEF(PANEL_IOCTL_SET_AUDIO_READY, oplus_display_panel_set_audio_ready),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DISPLAY_TIMING_INFO, oplus_display_panel_dump_info),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANEL_DSC, oplus_display_panel_get_dsc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_POWER_STATUS, oplus_display_panel_set_power_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_POWER_STATUS, oplus_display_panel_get_power_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_REGULATOR_CONTROL, oplus_display_panel_regulator_control),*/
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_CLOSEBL_FLAG, oplus_display_panel_set_closebl_flag),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CLOSEBL_FLAG, oplus_display_panel_get_closebl_flag),
	/*PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PANEL_REG, oplus_display_panel_set_reg),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANEL_REG, oplus_display_panel_get_reg),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIMLAYER_HBM, oplus_display_panel_set_dimlayer_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIMLAYER_HBM, oplus_display_panel_get_dimlayer_hbm),*/
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIMLAYER_BL_EN, oplus_display_panel_set_dimlayer_enable),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIMLAYER_BL_EN, oplus_display_panel_get_dimlayer_enable),
	/*PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PANEL_BLANK, oplus_display_panel_notify_blank),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_SPR, oplus_display_panel_set_spr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SPR, oplus_display_panel_get_spr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_ROUNDCORNER, oplus_display_panel_get_roundcorner),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DYNAMIC_OSC_CLOCK, oplus_display_panel_set_dynamic_osc_clock),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DYNAMIC_OSC_CLOCK, oplus_display_panel_get_dynamic_osc_clock),*/
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_OPLUS_BRIGHTNESS, oplus_display_set_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_OPLUS_BRIGHTNESS, oplus_display_get_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_LCM_CABC, oplus_display_panel_set_cabc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_LCM_CABC, oplus_display_panel_get_cabc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_CABC_STATUS, oplus_display_panel_set_cabc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CABC_STATUS, oplus_display_panel_get_cabc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FP_PRESS, oplus_ofp_notify_fp_press),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_MTK_LOG_LEVEL, oplus_display_set_mtk_loglevel),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FPS_LIMIT, oplus_display_set_limit_fps),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FAKE_AOD, oplus_ofp_set_fake_aod),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_FAKE_AOD, oplus_ofp_get_fake_aod),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SOFTIRIS_COLOR, oplus_display_get_softiris_color_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DP_SUPPORT, oplus_display_get_dp_support),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DRE_STATUS, oplus_display_panel_set_cabc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DRE_STATUS, oplus_display_panel_get_cabc),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANEL_BPP, oplus_display_panel_get_panel_bpp),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PQ_TRIGGER, oplus_display_panel_set_pq_trigger),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PQ_TRIGGER, oplus_display_panel_get_pq_trigger),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FP_TYPE, oplus_ofp_set_fp_type),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_FP_TYPE, oplus_ofp_get_fp_type),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_PWM_STATUS, oplus_display_panel_set_pwm_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PWM_STATUS, oplus_display_panel_get_pwm_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PWM_STATUS_FOR_90HZ, oplus_display_panel_get_pwm_status_for_90hz),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANEL_TYPE, oplus_display_panel_get_panel_type),
};

static int panel_open(struct inode *inode, struct file *filp)
{
	if (panel_ref) {
		pr_err("%s panel has already open\n", __func__);
		return -1;
	}

	++panel_ref;
	try_module_get(THIS_MODULE);

	return 0;
}

static ssize_t panel_read(struct file *filp, char __user *buffer,
		size_t count, loff_t *offset)
{
	pr_err("%s\n", __func__);
	return count;
}

static ssize_t panel_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *f_pos)
{
	pr_err("%s\n", __func__);
	return count;
}

long panel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int in_size, out_size, drv_size, ksize;
	unsigned int nr = PANEL_IOCTL_NR(cmd);
	char static_data[128];
	char *kdata = NULL;
	const struct panel_ioctl_desc *ioctl = NULL;
	oplus_panel_feature *func = NULL;
	int retcode = -EINVAL;

	if ((nr >= PANEL_COMMOND_MAX) || (nr <= PANEL_COMMOND_BASE)) {
		pr_err("%s invalid cmd\n", __func__);
		return retcode;
	}

	ioctl = &panel_ioctls[nr];
	if (!ioctl) {
		pr_err("%s invalid ioctl\n", __func__);
		return retcode;
	}

	func = ioctl->func;
	if (unlikely(!func)) {
		pr_err("%s no function\n", __func__);
		retcode = -EINVAL;
		return retcode;
	}

	drv_size = PANEL_IOCTL_SIZE(cmd);
	out_size = drv_size;
	in_size = drv_size;
	if ((cmd & ioctl->cmd & IOC_IN) == 0) {
		in_size = 0;
	}
	if ((cmd & ioctl->cmd & IOC_OUT) == 0) {
		out_size = 0;
	}
	ksize = max(max(in_size, out_size), drv_size);

	pr_debug("%s pid = %d, cmd = %s\n", __func__, task_pid_nr(current), ioctl->name);

	if (ksize <= sizeof(static_data)) {
		kdata = static_data;
	} else {
		kdata = kmalloc(ksize, GFP_KERNEL);
		if (!kdata) {
			retcode = -ENOMEM;
			goto err_panel;
		}
	}

	if (copy_from_user(kdata, (void __user *)arg, in_size) != 0) {
		retcode = -EFAULT;
		goto err_panel;
	}

	if (ksize > in_size) {
		memset(kdata+in_size, 0, ksize-in_size);
	}
	retcode = func(kdata);  /*any lock here?*/

	if (copy_to_user((void __user *)arg, kdata, out_size) != 0) {
		retcode = -EFAULT;
		goto err_panel;
	}

err_panel:
	if (kdata != static_data) {
		kfree(kdata);
	}
	if (retcode) {
		pr_err("%s pid = %d, retcode = %d\n", __func__, task_pid_nr(current), retcode);
	}
	return retcode;
}

int panel_release(struct inode *inode, struct file *filp)
{
	--panel_ref;
	module_put(THIS_MODULE);
	pr_err("%s\n", __func__);

	return 0;
}

static const struct file_operations panel_ops =
{
	.owner              = THIS_MODULE,
	.open               = panel_open,
	.release            = panel_release,
	.unlocked_ioctl     = panel_ioctl,
	.compat_ioctl       = panel_ioctl,
	.read               = panel_read,
	.write              = panel_write,
};

int oplus_display_panel_init(void)
{
	int rc = 0;

	printk("%s\n", __func__);

	rc = alloc_chrdev_region(&dev_num, 0, 1, OPLUS_PANEL_NAME);
	if (rc < 0) {
		pr_err("%s: failed to alloc chrdev region\n", __func__);
		return rc;
	}

	panel_class = class_create(THIS_MODULE, OPLUS_PANEL_CLASS_NAME);
	if (IS_ERR(panel_class)) {
		pr_err("%s class create error\n", __func__);
		goto err_class_create;
	}

	cdev_init(&panel_cdev, &panel_ops);
	rc = cdev_add(&panel_cdev, dev_num, 1);
	if (rc < 0) {
		pr_err("%s: failed to add cdev\n", __func__);
		goto err_cdev_add;
	}

	panel_dev = device_create(panel_class, NULL, dev_num, NULL, OPLUS_PANEL_NAME);
	if (IS_ERR(panel_dev)) {
		pr_err("%s device create error\n", __func__);
		goto err_device_create;
	}
	return 0;

err_device_create:
	cdev_del(&panel_cdev);
err_cdev_add:
	class_destroy(panel_class);
err_class_create:
	unregister_chrdev_region(dev_num, 1);

	return rc;
}

void oplus_display_panel_exit(void)
{
	pr_err("%s\n", __func__);

	cdev_del(&panel_cdev);
	device_destroy(panel_class, dev_num);
	class_destroy(panel_class);
	unregister_chrdev_region(dev_num, 1);
}

//module_init(oplus_display_panel_init);
//module_exit(oplus_display_panel_exit);
//MODULE_LICENSE("GPL v2");
//MODULE_AUTHOR("Lisheng <lisheng1@oplus.com>");
