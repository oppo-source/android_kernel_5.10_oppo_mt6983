// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __SENINF_CLK_H__
#define __SENINF_CLK_H__

#include <linux/device.h>
#if IS_ENABLED(CONFIG_PM_SLEEP)
#include <linux/pm_wakeup.h>
#endif

#include <linux/atomic.h>
#include <linux/platform_device.h>

#include "kd_imgsensor_define.h"

#include "seninf_common.h"

#if IS_ENABLED(CONFIG_FPGA_EARLY_PORTING)
#define SENINF_CLK_CONTROL 0
#else
#define SENINF_CLK_CONTROL 1
#endif

enum DFS_OPTION {
	DFS_CTRL_ENABLE,
	DFS_CTRL_DISABLE,
	DFS_UPDATE,
	DFS_RELEASE,
	DFS_SUPPORTED_ISP_CLOCKS,
	DFS_CUR_ISP_CLOCK,
};

#ifdef DFS_CTRL_BY_OPP
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
struct seninf_dfs_ctx {
	struct device *dev;
	struct regulator *reg;
	unsigned long *freqs;
	unsigned long *volts;
	int cnt;
};
int seninf_dfs_init(struct seninf_dfs_ctx *ctx, struct device *dev);
void seninf_dfs_exit(struct seninf_dfs_ctx *ctx);
int seninf_dfs_ctrl(
	struct seninf_dfs_ctx *ctx, enum DFS_OPTION option, void *pbuff);
#endif
// #define IMGSENSOR_DFS_CTRL_ENABLE
#ifdef IMGSENSOR_DFS_CTRL_ENABLE
#include <linux/pm_qos.h>
#include <mmdvfs_pmqos.h>
extern int imgsensor_dfs_ctrl(enum DFS_OPTION option, void *pbuff);
#endif

#ifndef SENINF_USE_RPM
enum SENINF_CLK_IDX_SYS {
	SENINF_CLK_IDX_SYS_MIN_NUM = 0,
	SENINF_CLK_IDX_SYS_SCP_SYS_MDP = SENINF_CLK_IDX_SYS_MIN_NUM,
	SENINF_CLK_IDX_SYS_SCP_SYS_CAM,
	SENINF_CLK_IDX_SYS_CAMSYS_SENINF_CGPDN,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF1,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF2,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF3,
	SENINF_CLK_IDX_SYS_MAX_NUM
};
#else
enum SENINF_CLK_IDX_SYS {
	SENINF_CLK_IDX_SYS_MIN_NUM = 0,
	SENINF_CLK_IDX_SYS_CAMSYS_SENINF_CGPDN = SENINF_CLK_IDX_SYS_MIN_NUM,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF1,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF2,
	SENINF_CLK_IDX_SYS_TOP_MUX_SENINF3,
	SENINF_CLK_IDX_SYS_MAX_NUM
};
#endif

enum SENINF_CLK_IDX_TG {
	SENINF_CLK_IDX_TG_MIN_NUM = SENINF_CLK_IDX_SYS_MAX_NUM,
	SENINF_CLK_IDX_TG_TOP_MUX_CAMTG = SENINF_CLK_IDX_TG_MIN_NUM,
	SENINF_CLK_IDX_TG_TOP_MUX_CAMTG2,
	SENINF_CLK_IDX_TG_TOP_MUX_CAMTG3,
	SENINF_CLK_IDX_TG_TOP_MUX_CAMTG4,
	SENINF_CLK_IDX_TG_TOP_MUX_CAMTG5,
	SENINF_CLK_IDX_TG_TOP_MUX_CAMTG6,
	SENINF_CLK_IDX_TG_MAX_NUM
};

enum SENINF_CLK_IDX_FREQ {
	SENINF_CLK_IDX_FREQ_MIN_NUM = SENINF_CLK_IDX_TG_MAX_NUM,
	SENINF_CLK_IDX_FREQ_TOP_UNIVP_192M_D32 = SENINF_CLK_IDX_FREQ_MIN_NUM,
	SENINF_CLK_IDX_FREQ_TOP_UNIVP_192M_D16,
	SENINF_CLK_IDX_FREQ_TOP_F26M_CK_D2,
	SENINF_CLK_IDX_FREQ_TOP_UNIVP_192M_D8,
	SENINF_CLK_IDX_FREQ_TOP_CLK26M,
	SENINF_CLK_IDX_FREQ_TOP_UNIVP_192M_D4,
	SENINF_CLK_IDX_FREQ_TOP_UNIVPLL_D3_D8,
	SENINF_CLK_IDX_FREQ_MAX_NUM
};

enum SENINF_CLK_TG {
	SENINF_CLK_TG_0,
	SENINF_CLK_TG_1,
	SENINF_CLK_TG_2,
	SENINF_CLK_TG_3,
	SENINF_CLK_TG_4,
	SENINF_CLK_TG_5,
	SENINF_CLK_TG_MAX_NUM
};

enum SENINF_CLK_MCLK_FREQ {
	SENINF_CLK_MCLK_FREQ_6MHZ = 6,
	SENINF_CLK_MCLK_FREQ_12MHZ = 12,
	SENINF_CLK_MCLK_FREQ_13MHZ = 13,
	SENINF_CLK_MCLK_FREQ_24MHZ = 24,
	SENINF_CLK_MCLK_FREQ_26MHZ = 26,
	SENINF_CLK_MCLK_FREQ_48MHZ = 48,
	SENINF_CLK_MCLK_FREQ_52MHZ = 52
};

#define SENINF_CLK_MCLK_FREQ_MAX    SENINF_CLK_MCLK_FREQ_52MHZ
#define SENINF_CLK_MCLK_FREQ_MIN    SENINF_CLK_MCLK_FREQ_6MHZ
#define SENINF_CLK_IDX_MIN_NUM      SENINF_CLK_IDX_SYS_MIN_NUM
#define SENINF_CLK_IDX_MAX_NUM      SENINF_CLK_IDX_FREQ_MAX_NUM
#define SENINF_CLK_IDX_FREQ_IDX_NUM \
(SENINF_CLK_IDX_FREQ_MAX_NUM - SENINF_CLK_IDX_FREQ_MIN_NUM)

struct SENINF_CLK_CTRL {
	char *pctrl;
};

static struct SENINF_CLK_CTRL gseninf_mclk_name[SENINF_CLK_IDX_MAX_NUM] = {
#ifndef SENINF_USE_RPM
	{"SCP_SYS_MDP"},
	{"SCP_SYS_CAM"},
#endif
	{"CAMSYS_SENINF_CGPDN"},
	{"TOP_MUX_SENINF"},
	{"TOP_MUX_SENINF1"},
	{"TOP_MUX_SENINF2"},
	{"TOP_MUX_SENINF3"},
	{"TOP_MUX_CAMTG"},
	{"TOP_MUX_CAMTG2"},
	{"TOP_MUX_CAMTG3"},
	{"TOP_MUX_CAMTG4"},
	{"TOP_MUX_CAMTG5"},
	{"TOP_MUX_CAMTG6"},
	{"TOP_UNIVP_192M_D32"}, /*   6*/
	{"TOP_UNIVP_192M_D16"}, /*  12*/
	{"TOP_F26M_CK_D2"},     /*  13*/
	{"TOP_UNIVP_192M_D8"},  /*  24*/
	{"TOP_CLK26M"},         /*  26*/
	{"TOP_UNIVP_192M_D4"},  /*  48*/
	{"TOP_UNIVPLL_D6_D8"},  /*  52*/
};

static enum SENINF_CLK_MCLK_FREQ
gseninf_clk_freq[SENINF_CLK_IDX_FREQ_IDX_NUM] = {
	SENINF_CLK_MCLK_FREQ_6MHZ,
	SENINF_CLK_MCLK_FREQ_12MHZ,
	SENINF_CLK_MCLK_FREQ_13MHZ,
	SENINF_CLK_MCLK_FREQ_24MHZ,
	SENINF_CLK_MCLK_FREQ_26MHZ,
	SENINF_CLK_MCLK_FREQ_48MHZ,
	SENINF_CLK_MCLK_FREQ_52MHZ,
};



struct SENINF_CLK {
	struct platform_device *pplatform_device;
	struct clk *mclk_sel[SENINF_CLK_IDX_MAX_NUM];
	atomic_t enable_cnt[SENINF_CLK_IDX_MAX_NUM];
	atomic_t wakelock_cnt;
	unsigned int g_platform_id;

#if IS_ENABLED(CONFIG_PM_SLEEP)
	struct wakeup_source *seninf_wake_lock;
#endif
};

enum SENINF_RETURN seninf_clk_init(struct SENINF_CLK *pclk);

#define HAVE_SENINF_CLK_EXIT
void seninf_clk_exit(struct SENINF_CLK *pclk);

int seninf_clk_set(
	struct SENINF_CLK *pclk, struct ACDK_SENSOR_MCLK_STRUCT *pmclk);
void seninf_clk_open(struct SENINF_CLK *pclk);
void seninf_clk_release(struct SENINF_CLK *pclk);
unsigned int seninf_clk_get_meter(struct SENINF_CLK *pclk, unsigned int clk);

extern unsigned int mt_get_ckgen_freq(int ID);
extern unsigned long clk_get_rate(struct clk *clk);

#endif

