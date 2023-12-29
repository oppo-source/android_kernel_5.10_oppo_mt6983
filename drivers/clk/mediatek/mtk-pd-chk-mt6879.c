// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/power/mt6879-power.h>

#include "mtk-pd-chk.h"
#include "clkchk-mt6879.h"

#define TAG				"[pdchk] "
#define BUG_ON_CHK_ENABLE		0

/*
 * The clk names in Mediatek CCF.
 */

/* afe */
struct pd_check_swcg afe_swcgs[] = {
	SWCG("afe_afe"),
	SWCG("afe_22m"),
	SWCG("afe_24m"),
	SWCG("afe_apll2_tuner"),
	SWCG("afe_apll_tuner"),
	SWCG("afe_tdm_ck"),
	SWCG("afe_adc"),
	SWCG("afe_dac"),
	SWCG("afe_dac_predis"),
	SWCG("afe_tml"),
	SWCG("afe_nle"),
	SWCG("afe_general3_asrc"),
	SWCG("afe_connsys_i2s_asrc"),
	SWCG("afe_general1_asrc"),
	SWCG("afe_general2_asrc"),
	SWCG("afe_dac_hires"),
	SWCG("afe_adc_hires"),
	SWCG("afe_adc_hires_tml"),
	SWCG("afe_adda6_adc"),
	SWCG("afe_adda6_adc_hires"),
	SWCG("afe_3rd_dac"),
	SWCG("afe_3rd_dac_predis"),
	SWCG("afe_3rd_dac_tml"),
	SWCG("afe_3rd_dac_hires"),
	SWCG("afe_i2s5_bclk"),
	SWCG("afe_i2s6_bclk"),
	SWCG("afe_i2s7_bclk"),
	SWCG("afe_i2s8_bclk"),
	SWCG("afe_i2s9_bclk"),
	SWCG("afe_etdm_in0_bclk"),
	SWCG("afe_etdm_out0_bclk"),
	SWCG("afe_i2s1_bclk"),
	SWCG("afe_i2s2_bclk"),
	SWCG("afe_i2s3_bclk"),
	SWCG("afe_i2s4_bclk"),
	SWCG("afe_etdm_in1_bclk"),
	SWCG("afe_etdm_out1_bclk"),
	SWCG(NULL),
};
/* camsys_mraw */
struct pd_check_swcg camsys_mraw_swcgs[] = {
	SWCG("cam_mr_larbx"),
	SWCG("cam_mr_camtg"),
	SWCG("cam_mr_mraw0"),
	SWCG("cam_mr_mraw1"),
	SWCG("cam_mr_mraw2"),
	SWCG("cam_mr_mraw3"),
	SWCG("cam_mr_pda0"),
	SWCG("cam_mr_pda1"),
	SWCG(NULL),
};
/* camsys_rawa */
struct pd_check_swcg camsys_rawa_swcgs[] = {
	SWCG("cam_ra_larbx"),
	SWCG("cam_ra_cam"),
	SWCG("cam_ra_camtg"),
	SWCG(NULL),
};
/* camsys_rawb */
struct pd_check_swcg camsys_rawb_swcgs[] = {
	SWCG("cam_rb_larbx"),
	SWCG("cam_rb_cam"),
	SWCG("cam_rb_camtg"),
	SWCG(NULL),
};
/* camsys_yuva */
struct pd_check_swcg camsys_yuva_swcgs[] = {
	SWCG("cam_ya_larbx"),
	SWCG("cam_ya_cam"),
	SWCG("cam_ya_camtg"),
	SWCG(NULL),
};
/* camsys_yuvb */
struct pd_check_swcg camsys_yuvb_swcgs[] = {
	SWCG("cam_yb_larbx"),
	SWCG("cam_yb_cam"),
	SWCG("cam_yb_camtg"),
	SWCG(NULL),
};
/* cam_main_r1a */
struct pd_check_swcg cam_main_r1a_swcgs[] = {
	SWCG("cam_m_larb13_con"),
	SWCG("cam_m_larb14_con"),
	SWCG("cam_m_cam_con"),
	SWCG("cam_m_cam_suba_con"),
	SWCG("cam_m_cam_subb_con"),
	SWCG("cam_m_cam_subc_con"),
	SWCG("cam_m_cam_mraw_con"),
	SWCG("cam_m_camtg_con"),
	SWCG("cam_m_seninf_con"),
	SWCG("cam_m_gcamsva_con"),
	SWCG("cam_m_gcamsvb_con"),
	SWCG("cam_m_gcamsvc_con"),
	SWCG("cam_m_gcamsvd_con"),
	SWCG("cam_m_gcamsve_con"),
	SWCG("cam_m_gcamsvf_con"),
	SWCG("cam_m_gcamsvg_con"),
	SWCG("cam_m_gcamsvh_con"),
	SWCG("cam_m_gcamsvi_con"),
	SWCG("cam_m_gcamsvj_con"),
	SWCG("cam_m_camsv_con"),
	SWCG("cam_m_camsv_cq_a_con"),
	SWCG("cam_m_camsv_cq_b_con"),
	SWCG("cam_m_camsv_cq_c_con"),
	SWCG("cam_m_adl_con"),
	SWCG("cam_m_asg_con"),
	SWCG("cam_m_pda0_con"),
	SWCG("cam_m_pda1_con"),
	SWCG("cam_m_pda2_con"),
	SWCG("cam_m_fake_eng_con"),
	SWCG("cam_m_cam2mm0_gcon"),
	SWCG("cam_m_cam2mm1_gcon"),
	SWCG("cam_m_cam2sys_gcon"),
	SWCG(NULL),
};
/* ccu_main */
struct pd_check_swcg ccu_main_swcgs[] = {
	SWCG("ccu_larb19"),
	SWCG("ccu_ahb"),
	SWCG("ccusys_ccu0"),
	SWCG("ccusys_ccu1"),
	SWCG(NULL),
};
/* dip_nr_dip1 */
struct pd_check_swcg dip_nr_dip1_swcgs[] = {
	SWCG("dip_nr_dip1_larb15"),
	SWCG("dip_nr_dip1_dip_nr"),
	SWCG(NULL),
};
/* dip_top_dip1 */
struct pd_check_swcg dip_top_dip1_swcgs[] = {
	SWCG("dip_dip1_larb10"),
	SWCG("dip_dip1_dip_top"),
	SWCG(NULL),
};
/* dispsys_config */
struct pd_check_swcg dispsys_config_swcgs[] = {
	SWCG("mm_disp_mutex0"),
	SWCG("mm_disp_ovl0"),
	SWCG("mm_disp_merge0"),
	SWCG("mm_disp_fake_eng0"),
	SWCG("mm_disp_inlinerot0"),
	SWCG("mm_disp_wdma0"),
	SWCG("mm_disp_fake_eng1"),
	SWCG("mm_disp_dpi0"),
	SWCG("mm_disp_ovl0_2l_nwcg"),
	SWCG("mm_disp_rdma0"),
	SWCG("mm_disp_rdma1"),
	SWCG("mm_disp_rsz0"),
	SWCG("mm_disp_color0"),
	SWCG("mm_disp_ccorr0"),
	SWCG("mm_disp_ccorr1"),
	SWCG("mm_disp_aal0"),
	SWCG("mm_disp_gamma0"),
	SWCG("mm_disp_postmask0"),
	SWCG("mm_disp_dither0"),
	SWCG("mm_disp_cm0"),
	SWCG("mm_disp_spr0"),
	SWCG("mm_disp_dsc_wrap0"),
	SWCG("mm_disp_dsi0"),
	SWCG("mm_disp_ufbc_wdma0"),
	SWCG("mm_disp_wdma1"),
	SWCG("mm_dispsys_config"),
	SWCG("mm_disp_tdshp0"),
	SWCG("mm_disp_c3d0"),
	SWCG("mm_disp_y2r0"),
	SWCG("mm_disp_chist0"),
	SWCG("mm_disp_ovl0_2l"),
	SWCG("mm_disp_dli_async3"),
	SWCG("mm_disp_dl0_async3"),
	SWCG("mm_smi_larb"),
	SWCG("mm_dsi_clk"),
	SWCG("mm_dpi_clk"),
	SWCG("mm_sig_emi"),
	SWCG(NULL),
};
/* imgsys_main */
struct pd_check_swcg imgsys_main_swcgs[] = {
	SWCG("img_larb9"),
	SWCG("img_traw0"),
	SWCG("img_traw1"),
	SWCG("img_vcore_gals"),
	SWCG("img_dip0"),
	SWCG("img_wpe0"),
	SWCG("img_ipe"),
	SWCG("img_wpe1"),
	SWCG("img_wpe2"),
	SWCG("img_gals"),
	SWCG(NULL),
};
/* ipesys */
struct pd_check_swcg ipesys_swcgs[] = {
	SWCG("ipe_dpe"),
	SWCG("ipe_fdvt"),
	SWCG("ipe_me"),
	SWCG("ipesys_top"),
	SWCG("ipe_smi_larb12"),
	SWCG(NULL),
};
/* mdpsys_config */
struct pd_check_swcg mdpsys_config_swcgs[] = {
	SWCG("mdp_mutex0"),
	SWCG("mdp_apb_bus"),
	SWCG("mdp_smi0"),
	SWCG("mdp_rdma0"),
	SWCG("mdp_hdr0"),
	SWCG("mdp_aal0"),
	SWCG("mdp_rsz0"),
	SWCG("mdp_tdshp0"),
	SWCG("mdp_color0"),
	SWCG("mdp_wrot0"),
	SWCG("mdp_fake_eng0"),
	SWCG("mdp_dli_async0"),
	SWCG("mdp_rdma1"),
	SWCG("mdp_hdr1"),
	SWCG("mdp_aal1"),
	SWCG("mdp_rsz1"),
	SWCG("mdp_tdshp1"),
	SWCG("mdp_color1"),
	SWCG("mdp_wrot1"),
	SWCG("mdp_dlo_async0"),
	SWCG("mdp_hre_mdpsys"),
	SWCG(NULL),
};
/* mfg_top_config */
struct pd_check_swcg mfg_top_config_swcgs[] = {
	SWCG("mfgcfg_bg3d"),
	SWCG(NULL),
};
/* mminfra_config */
struct pd_check_swcg mminfra_config_swcgs[] = {
	SWCG("mminfra_gce_d"),
	SWCG("mminfra_gce_m"),
	SWCG("mminfra_smi"),
	SWCG("mminfra_gce_26m"),
	SWCG(NULL),
};
/* vdec_gcon_base */
struct pd_check_swcg vdec_gcon_base_swcgs[] = {
	SWCG("vde2_larb1_cken"),
	SWCG("vde2_mini_mdp_cken"),
	SWCG("vde2_vdec_cken"),
	SWCG("vde2_vdec_active"),
	SWCG(NULL),
};
/* venc_gcon */
struct pd_check_swcg venc_gcon_swcgs[] = {
	SWCG("ven1_cke0_larb"),
	SWCG("ven1_cke1_venc"),
	SWCG("ven1_cke2_jpgenc"),
	SWCG("ven1_cke5_gals"),
	SWCG(NULL),
};
/* wpe1_dip1 */
struct pd_check_swcg wpe1_dip1_swcgs[] = {
	SWCG("wpe1_dip1_larb11"),
	SWCG("wpe1_dip1_wpe"),
	SWCG(NULL),
};
/* wpe2_dip1 */
struct pd_check_swcg wpe2_dip1_swcgs[] = {
	SWCG("wpe2_dip1_larb11"),
	SWCG("wpe2_dip1_wpe"),
	SWCG(NULL),
};
/* wpe3_dip1 */
struct pd_check_swcg wpe3_dip1_swcgs[] = {
	SWCG("wpe3_dip1_larb11"),
	SWCG("wpe3_dip1_wpe"),
	SWCG(NULL),
};

struct subsys_cgs_check {
	unsigned int pd_id;		/* power domain id */
	struct pd_check_swcg *swcgs;	/* those CGs that would be checked */
	enum chk_sys_id chk_id;		/*
					 * chk_id is used in
					 * print_subsys_reg() and can be NULL
					 * if not porting ready yet.
					 */
};

struct subsys_cgs_check mtk_subsys_check[] = {
	{MT6879_POWER_DOMAIN_AUDIO, afe_swcgs, afe},
	{MT6879_POWER_DOMAIN_CAM_MRAW, camsys_mraw_swcgs, cam_mr},
	{MT6879_POWER_DOMAIN_CAM_SUBA, camsys_rawa_swcgs, cam_ra},
	{MT6879_POWER_DOMAIN_CAM_SUBB, camsys_rawb_swcgs, cam_rb},
	{MT6879_POWER_DOMAIN_CAM_SUBA, camsys_yuva_swcgs, cam_ya},
	{MT6879_POWER_DOMAIN_CAM_SUBB, camsys_yuvb_swcgs, cam_yb},
	{MT6879_POWER_DOMAIN_CAM_MAIN, cam_main_r1a_swcgs, cam_m},
	{MT6879_POWER_DOMAIN_CAM_MAIN, ccu_main_swcgs, ccu},
	{MT6879_POWER_DOMAIN_ISP_DIP1, dip_nr_dip1_swcgs, dip_nr_dip1},
	{MT6879_POWER_DOMAIN_ISP_DIP1, dip_top_dip1_swcgs, dip_top_dip1},
	{MT6879_POWER_DOMAIN_DISP, dispsys_config_swcgs, mm},
	{MT6879_POWER_DOMAIN_ISP_MAIN, imgsys_main_swcgs, img},
	{MT6879_POWER_DOMAIN_ISP_IPE, ipesys_swcgs, ipe},
	{MT6879_POWER_DOMAIN_DISP, mdpsys_config_swcgs, mdp},
	{MT6879_POWER_DOMAIN_MFG1, mfg_top_config_swcgs, mfgcfg},
	{MT6879_POWER_DOMAIN_MM_INFRA, mminfra_config_swcgs, mminfra_config},
	{MT6879_POWER_DOMAIN_VDE0, vdec_gcon_base_swcgs, vde2},
	{MT6879_POWER_DOMAIN_VEN0, venc_gcon_swcgs, ven1},
	{MT6879_POWER_DOMAIN_ISP_DIP1, wpe1_dip1_swcgs, wpe1_dip1},
	{MT6879_POWER_DOMAIN_ISP_DIP1, wpe2_dip1_swcgs, wpe2_dip1},
	{MT6879_POWER_DOMAIN_ISP_DIP1, wpe3_dip1_swcgs, wpe3_dip1},
};

static struct pd_check_swcg *get_subsys_cg(unsigned int id)
{
	int i;

	if (id >= MT6879_POWER_DOMAIN_NR)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
		if (mtk_subsys_check[i].pd_id == id)
			return mtk_subsys_check[i].swcgs;
	}

	return NULL;
}

static void dump_subsys_reg(unsigned int id)
{
	int i;

	if (id >= MT6879_POWER_DOMAIN_NR)
		return;

	for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
		if (mtk_subsys_check[i].pd_id == id)
			print_subsys_reg_mt6879(mtk_subsys_check[i].chk_id);
	}
}

unsigned int pd_list[] = {
	MT6879_POWER_DOMAIN_MD,
	MT6879_POWER_DOMAIN_CONN,
	MT6879_POWER_DOMAIN_UFS0_SHUTDOWN,
	MT6879_POWER_DOMAIN_AUDIO,
	MT6879_POWER_DOMAIN_ADSP_TOP_DORMANT,
	MT6879_POWER_DOMAIN_ADSP_INFRA,
	MT6879_POWER_DOMAIN_ISP_MAIN,
	MT6879_POWER_DOMAIN_ISP_DIP1,
	MT6879_POWER_DOMAIN_ISP_IPE,
	MT6879_POWER_DOMAIN_ISP_VCORE,
	MT6879_POWER_DOMAIN_VDE0,
	MT6879_POWER_DOMAIN_VEN0,
	MT6879_POWER_DOMAIN_CAM_MAIN,
	MT6879_POWER_DOMAIN_CAM_MRAW,
	MT6879_POWER_DOMAIN_CAM_SUBA,
	MT6879_POWER_DOMAIN_CAM_SUBB,
	MT6879_POWER_DOMAIN_CAM_VCORE,
	MT6879_POWER_DOMAIN_DISP,
	MT6879_POWER_DOMAIN_MM_INFRA,
	MT6879_POWER_DOMAIN_MM_PROC_DORMANT,
	MT6879_POWER_DOMAIN_CSI_RX,
	MT6879_POWER_DOMAIN_MFG1,
	MT6879_POWER_DOMAIN_MFG2,
	MT6879_POWER_DOMAIN_MFG3,
	MT6879_POWER_DOMAIN_MFG4,
	MT6879_POWER_DOMAIN_MFG5,
	MT6879_POWER_DOMAIN_APU,
};

static bool is_in_pd_list(unsigned int id)
{
	int i;

	if (id >= MT6879_POWER_DOMAIN_NR)
		return false;

	for (i = 0; i < ARRAY_SIZE(pd_list); i++) {
		if (id == pd_list[i])
			return true;
	}

	return false;
}

static enum chk_sys_id debug_dump_id[] = {
	spm,
	top,
	infracfg,
	apmixed,
	mfg_ao,
	apu_ao,
	vlpcfg,
	vlp_ck,
	chk_sys_num,
};

static void debug_dump(unsigned int id, unsigned int pwr_sta)
{
	int i;

	if (id >= MT6879_POWER_DOMAIN_NR)
		return;

	set_subsys_reg_dump_mt6879(debug_dump_id);

	if (id == MT6879_POWER_DOMAIN_MM_PROC_DORMANT)
		print_subsys_reg_mt6879(hfrp);
	if (id == MT6879_POWER_DOMAIN_MM_INFRA)
		print_subsys_reg_mt6879(mminfra_smi);

	if (pwr_sta == PD_PWR_ON) {
		for (i = 0; i < ARRAY_SIZE(mtk_subsys_check); i++) {
			if (mtk_subsys_check[i].pd_id == id)
				print_subsys_reg_mt6879(mtk_subsys_check[i].chk_id);
		}
	}

	get_subsys_reg_dump_mt6879();

	BUG_ON(1);
}

static enum chk_sys_id log_dump_id[] = {
	infracfg,
	spm,
	vlpcfg,
	chk_sys_num,
};

static void log_dump(unsigned int id, unsigned int pwr_sta)
{
	if (id >= MT6879_POWER_DOMAIN_NR)
		return;

	if (id == MT6879_POWER_DOMAIN_MD) {
		set_subsys_reg_dump_mt6879(log_dump_id);
		get_subsys_reg_dump_mt6879();
	}
}

static struct pd_sta pd_pwr_msk[] = {
	{MT6879_POWER_DOMAIN_MD, PWR_STA, 0x00000001},
	{MT6879_POWER_DOMAIN_CONN, PWR_STA, 0x00000002},
	{MT6879_POWER_DOMAIN_UFS0_SHUTDOWN, PWR_STA, 0x00000010},
	{MT6879_POWER_DOMAIN_AUDIO, PWR_STA, 0x00000020},
	{MT6879_POWER_DOMAIN_ADSP_TOP_DORMANT, PWR_STA, 0x00000040},
	{MT6879_POWER_DOMAIN_ADSP_INFRA, PWR_STA, 0x00000080},
	{MT6879_POWER_DOMAIN_ISP_MAIN, PWR_STA, 0x00000200},
	{MT6879_POWER_DOMAIN_ISP_DIP1, PWR_STA, 0x00000400},
	{MT6879_POWER_DOMAIN_ISP_IPE, PWR_STA, 0x00000800},
	{MT6879_POWER_DOMAIN_ISP_VCORE, PWR_STA, 0x00001000},
	{MT6879_POWER_DOMAIN_VDE0, PWR_STA, 0x00002000},
	{MT6879_POWER_DOMAIN_VEN0, PWR_STA, 0x00008000},
	{MT6879_POWER_DOMAIN_CAM_MAIN, PWR_STA, 0x00020000},
	{MT6879_POWER_DOMAIN_CAM_MRAW, PWR_STA, 0x00040000},
	{MT6879_POWER_DOMAIN_CAM_SUBA, PWR_STA, 0x00080000},
	{MT6879_POWER_DOMAIN_CAM_SUBB, PWR_STA, 0x00100000},
	{MT6879_POWER_DOMAIN_CAM_VCORE, PWR_STA, 0x00400000},
	{MT6879_POWER_DOMAIN_DISP, PWR_STA, 0x02000000},
	{MT6879_POWER_DOMAIN_MM_INFRA, PWR_STA, 0x08000000},
	{MT6879_POWER_DOMAIN_MM_PROC_DORMANT, PWR_STA, 0x10000000},
	{MT6879_POWER_DOMAIN_CSI_RX, PWR_CON_STA, 0xC0000000},
	{MT6879_POWER_DOMAIN_MFG1, XPU_PWR_STA, 0x00000004},
	{MT6879_POWER_DOMAIN_MFG2, XPU_PWR_STA, 0x00000008},
	{MT6879_POWER_DOMAIN_MFG3, XPU_PWR_STA, 0x00000010},
	{MT6879_POWER_DOMAIN_MFG4, XPU_PWR_STA, 0x00000020},
	{MT6879_POWER_DOMAIN_MFG5, XPU_PWR_STA, 0x00000040},
	{MT6879_POWER_DOMAIN_APU, OTHER_STA, 0x00000200},
};

static struct pd_sta *get_pd_pwr_msk(int pd_id)
{
	int i;

	if (pd_id == PD_NULL || pd_id > ARRAY_SIZE(pd_pwr_msk))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(pd_pwr_msk); i++) {
		if (pd_id == pd_pwr_msk[i].pd_id)
			return &pd_pwr_msk[pd_id];
	}

	return NULL;
}

static int off_mtcmos_id[] = {
	MT6879_POWER_DOMAIN_UFS0_SHUTDOWN,
	MT6879_POWER_DOMAIN_ISP_MAIN,
	MT6879_POWER_DOMAIN_ISP_DIP1,
	MT6879_POWER_DOMAIN_ISP_IPE,
	MT6879_POWER_DOMAIN_ISP_VCORE,
	MT6879_POWER_DOMAIN_VDE0,
	MT6879_POWER_DOMAIN_VEN0,
	MT6879_POWER_DOMAIN_CAM_MAIN,
	MT6879_POWER_DOMAIN_CAM_MRAW,
	MT6879_POWER_DOMAIN_CAM_SUBA,
	MT6879_POWER_DOMAIN_CAM_SUBB,
	MT6879_POWER_DOMAIN_CAM_VCORE,
	MT6879_POWER_DOMAIN_DISP,
	MT6879_POWER_DOMAIN_MM_INFRA,
	MT6879_POWER_DOMAIN_MM_PROC_DORMANT,
	MT6879_POWER_DOMAIN_CSI_RX,
	MT6879_POWER_DOMAIN_MFG1,
	MT6879_POWER_DOMAIN_MFG2,
	MT6879_POWER_DOMAIN_MFG3,
	MT6879_POWER_DOMAIN_MFG4,
	MT6879_POWER_DOMAIN_MFG5,
	MT6879_POWER_DOMAIN_APU,
	PD_NULL,
};

static int notice_mtcmos_id[] = {
	MT6879_POWER_DOMAIN_MD,
	MT6879_POWER_DOMAIN_CONN,
	MT6879_POWER_DOMAIN_AUDIO,
	MT6879_POWER_DOMAIN_ADSP_TOP_DORMANT,
	MT6879_POWER_DOMAIN_ADSP_INFRA,
	PD_NULL,
};

static int *get_off_mtcmos_id(void)
{
	return off_mtcmos_id;
}

static int *get_notice_mtcmos_id(void)
{
	return notice_mtcmos_id;
}

static bool is_mtcmos_chk_bug_on(void)
{
#if BUG_ON_CHK_ENABLE
	return true;
#endif
	return false;
}

static int suspend_allow_id[] = {
	MT6879_POWER_DOMAIN_UFS0_SHUTDOWN,
	PD_NULL,
};

static int *get_suspend_allow_id(void)
{
	return suspend_allow_id;
}

/*
 * init functions
 */

static struct pdchk_ops pdchk_mt6879_ops = {
	.get_subsys_cg = get_subsys_cg,
	.dump_subsys_reg = dump_subsys_reg,
	.is_in_pd_list = is_in_pd_list,
	.debug_dump = debug_dump,
	.log_dump = log_dump,
	.get_pd_pwr_msk = get_pd_pwr_msk,
	.get_off_mtcmos_id = get_off_mtcmos_id,
	.get_notice_mtcmos_id = get_notice_mtcmos_id,
	.is_mtcmos_chk_bug_on = is_mtcmos_chk_bug_on,
	.get_suspend_allow_id = get_suspend_allow_id,
};

static int pd_chk_mt6879_probe(struct platform_device *pdev)
{
	pdchk_common_init(&pdchk_mt6879_ops);

	return 0;
}

static const struct of_device_id of_match_pdchk_mt6879[] = {
	{
		.compatible = "mediatek,mt6879-pdchk",
	}, {
		/* sentinel */
	}
};

static struct platform_driver pd_chk_mt6879_drv = {
	.probe = pd_chk_mt6879_probe,
	.driver = {
		.name = "pd-chk-mt6879",
		.owner = THIS_MODULE,
		.pm = &pdchk_dev_pm_ops,
		.of_match_table = of_match_pdchk_mt6879,
	},
};

/*
 * init functions
 */

static int __init pd_chk_init(void)
{
	return platform_driver_register(&pd_chk_mt6879_drv);
}

static void __exit pd_chk_exit(void)
{
	platform_driver_unregister(&pd_chk_mt6879_drv);
}

subsys_initcall(pd_chk_init);
module_exit(pd_chk_exit);
MODULE_LICENSE("GPL");
