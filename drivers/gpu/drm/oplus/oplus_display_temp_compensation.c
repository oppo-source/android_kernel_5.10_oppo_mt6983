/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_temp_compensation.c
** Description : oplus_display_temp_compensation implement
** Version : 1.0
** Date : 2022/11/20
** Author : Display
***************************************************************/

#include <linux/thermal.h>
#include <drm/drm_device.h>
#include "mtk_panel_ext.h"
#include "mtk_drm_ddp_comp.h"
#include "oplus_display_temp_compensation.h"
#include "oplus_display_onscreenfingerprint.h"
#include "oplus_adfr_ext.h"

/* -------------------- macro -------------------- */
#define REGFLAG_CMD		0xFFFA

/* -------------------- parameters -------------------- */
struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* log level config */
unsigned int oplus_temp_compensation_log_level = OPLUS_TEMP_COMPENSATION_LOG_LEVEL_DEBUG;
EXPORT_SYMBOL(oplus_temp_compensation_log_level);
/* temp compensation global structure */
static struct oplus_temp_compensation_params *g_oplus_temp_compensation_params = NULL;

/* ntc_resistance:100k internal_pull_up:100k voltage:1.84v */
static int con_temp_ntc_100k_1840mv[] = {
	-40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22,
	-21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
	26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
	74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
	98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
	117, 118, 119, 120, 121, 122, 123, 124, 125
};

static int con_volt_ntc_100k_1840mv[] = {
	1799, 1796, 1793, 1790, 1786, 1782, 1778, 1774, 1770, 1765, 1760, 1755, 1749, 1743, 1737, 1731,
	1724, 1717, 1709, 1701, 1693, 1684, 1675, 1666, 1656, 1646, 1635, 1624, 1612, 1600, 1588, 1575,
	1561, 1547, 1533, 1518, 1503, 1478, 1471, 1454, 1437, 1420, 1402, 1384, 1365, 1346, 1327, 1307,
	1287, 1267, 1246, 1225, 1204, 1183, 1161, 1139, 1118, 1096, 1074, 1052, 1030, 1008, 986, 964,
	942, 920, 898, 877, 855, 834, 813, 793, 772, 752, 732, 712, 693, 674, 655, 637, 619, 601, 584,
	567, 550, 534, 518, 503, 488, 473, 459, 445, 431, 418, 405, 392, 380, 368, 357, 345, 335, 324,
	314, 304, 294, 285, 276, 267, 259, 251, 243, 235, 227, 220, 213, 206, 200, 194, 187, 182, 176,
	170, 165, 160, 155, 150, 145, 140, 136, 132, 128, 124, 120, 117, 113, 110, 106, 103, 100, 97,
	94, 91, 88, 86, 83, 81, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 57, 55, 54, 52, 51, 49, 48,
	47, 45
};

static unsigned char temp_compensation_paras[11][11][25] = {
	/* dbv > 3515 */
	{
		{16, 20, 24, 16, 20, 16, 20, 24, 58, 58, 58, 58, 58, 58, 58, 58, 58, 0, 0, 0, 0, 0, 0, 0, 0}, /* -20 ~ -10 */
		{16, 20, 24, 16, 20, 16, 20, 24, 58, 58, 58, 58, 58, 58, 58, 58, 58, 0, 0, 0, 0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{20, 24, 28, 20, 24, 20, 24, 28, 57, 57, 57, 57, 57, 57, 57, 57, 57, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{24, 28, 32, 24, 28, 24, 28, 32, 56, 56, 56, 56, 56, 56, 56, 56, 56, 0, 0, 0, 0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{28, 32, 36, 28, 32, 28, 32, 36, 55, 55, 55, 55, 55, 55, 55, 55, 55, 0, 0, 0, 0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{36, 40, 48, 36, 40, 36, 40, 48, 55, 55, 55, 55, 55, 55, 55, 55, 55, 0, 0, 0, 0, 0, 0, 0, 0}, /* 25 ~ 30 */
		{40, 44, 52, 40, 44, 40, 44, 52, 54, 54, 54, 54, 54, 54, 54, 54, 54, 0, 0, 0, 0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{44, 48, 56, 44, 48, 44, 48, 56, 54, 54, 54, 54, 54, 54, 54, 54, 54, 0, 0, 0, 0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{48, 52, 60, 48, 52, 48, 52, 60, 53, 53, 53, 53, 53, 53, 53, 53, 53, 0, 0, 0, 0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{52, 56, 64, 52, 56, 52, 56, 64, 53, 53, 53, 53, 53, 53, 53, 53, 53, 0, 0, 0, 0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{56, 60, 68, 56, 60, 56, 60, 68, 52, 52, 52, 52, 52, 52, 52, 52, 52, 0, 0, 0, 0, 0, 0, 0, 0}  /* > 50 */
	},

	/* 1604 <= dbv <= 3515 */
	{
		{16, 20, 24, 16, 20, 16, 20, 24, 29, 29, 38, 31, 31, 40, 30, 30, 39, 0, 0, 0, 0, 0, 0, 0, 0}, /* -20 ~ -10 */
		{16, 20, 24, 16, 20, 16, 20, 24, 29, 29, 38, 31, 31, 40, 29, 29, 38, 0, 0, 0, 0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{20, 24, 28, 20, 24, 20, 24, 28, 28, 28, 37, 30, 30, 39, 28, 28, 37, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{24, 28, 32, 24, 28, 24, 28, 32, 28, 28, 37, 30, 30, 39, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{28, 32, 36, 24, 28, 28, 32, 36, 27, 27, 36, 29, 29, 38, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{36, 40, 48, 36, 40, 36, 40, 48, 27, 27, 36, 28, 28, 37, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 25 ~ 30 */
		{40, 44, 52, 40, 44, 40, 44, 52, 26, 26, 35, 27, 27, 36, 26, 26, 35, 0, 0, 0, 0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{44, 48, 56, 44, 48, 44, 48, 56, 26, 26, 35, 27, 27, 36, 26, 26, 35, 0, 0, 0, 0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{48, 52, 60, 48, 52, 48, 52, 60, 25, 25, 34, 26, 26, 35, 25, 25, 34, 0, 0, 0, 0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{52, 56, 64, 52, 56, 52, 56, 64, 25, 25, 34, 26, 26, 35, 25, 25, 34, 0, 0, 0, 0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{56, 60, 68, 56, 60, 56, 60, 68, 24, 24, 33, 25, 25, 34, 24, 24, 33, 0, 0, 0, 0, 0, 0, 0, 0}  /* > 50 */
	},

	/* 1511 <= dbv < 1604 */
	{
		{12, 16, 20, 12, 16,  4,  4,  8, 29, 29, 38, 31, 31, 40, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -20 ~ -10 */
		{12, 16, 20, 12, 16,  4,  4,  8, 29, 29, 38, 31, 31, 40, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{16, 20, 24, 16, 20,  4,  8, 12, 28, 28, 37, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{20, 24, 28, 20, 24,  4,  8, 12, 28, 28, 37, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{24, 28, 32, 20, 24,  8,  8, 12, 27, 27, 36, 29, 29, 38, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{32, 36, 44, 32, 36,  8, 12, 16, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 25 ~ 30 */
		{36, 40, 48, 36, 40,  8, 16, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{40, 44, 52, 40, 44,  8, 16, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{44, 48, 56, 44, 48,  8, 20, 24, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{48, 52, 60, 48, 52, 12, 20, 24, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{52, 56, 64, 52, 56, 12, 20, 24, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}  /* > 50 */
	},

	/* 1419 <= dbv < 1511 */
	{
		{12, 12, 16, 12, 16, 4,  4,  8, 29, 29, 38, 31, 31, 40, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -20 ~ -10 */
		{12, 12, 16, 12, 16, 4,  4,  8, 29, 29, 38, 31, 31, 40, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{16, 20, 24, 16, 20, 4,  8, 12, 28, 28, 37, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{20, 24, 28, 20, 24, 4,  8, 12, 28, 28, 37, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{24, 28, 32, 20, 24, 8,  8, 12, 27, 27, 36, 29, 29, 38, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{28, 32, 40, 28, 32, 8, 12, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 25 ~ 30 */
		{32, 36, 44, 32, 36, 8, 12, 16, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{36, 40, 48, 36, 40, 8, 12, 16, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{40, 44, 52, 40, 44, 8, 16, 20, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{44, 48, 56, 44, 48, 8, 16, 20, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{48, 52, 60, 48, 52, 8, 16, 20, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}  /* > 50 */
	},

	/* 1328 <= dbv < 1419 */
	{
		{12, 12, 16,  8, 12, 4,  4,  8, 29, 29, 38, 31, 31, 40, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -20 ~ -10 */
		{12, 12, 16,  8, 12, 4,  4,  8, 29, 29, 38, 31, 31, 40, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{16, 20, 24, 12, 16, 4,  4, 12, 28, 28, 37, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{16, 20, 24, 16, 20, 4,  8, 12, 28, 28, 37, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{20, 24, 28, 16, 20, 4,  8, 12, 27, 27, 36, 29, 29, 38, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{24, 28, 36, 24, 28, 4,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 25 ~ 30 */
		{28, 32, 40, 28, 32, 8,  8, 12, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{32, 36, 44, 32, 36, 8, 12, 16, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{36, 40, 48, 36, 40, 8, 12, 16, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{40, 44, 52, 40, 44, 8, 16, 20, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{44, 48, 56, 44, 48, 8, 16, 20, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}  /* > 50 */
	},

	/* 1212 <= dbv < 1328 */
	{
		{ 8, 12, 16,  8, 12, 4,  4,  8, 30, 30, 39, 30, 30, 39, 28, 28, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -20 ~ -10 */
		{12, 12, 16,  8, 12, 4,  4,  8, 29, 29, 38, 30, 30, 39, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{12, 12, 24,  8, 12, 4,  4, 12, 29, 29, 38, 30, 30, 39, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{12, 16, 24, 12, 16, 4,  4, 12, 28, 28, 37, 29, 29, 38, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{16, 20, 24, 12, 16, 4,  8, 12, 27, 27, 36, 28, 28, 37, 26, 26, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{20, 24, 32, 20, 24, 4,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 25 ~ 30 */
		{24, 28, 36, 24, 28, 4, 12, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{28, 32, 40, 28, 32, 4, 12, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{32, 36, 44, 32, 36, 8,  8, 16, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{36, 40, 48, 36, 40, 8,  8, 16, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{40, 44, 52, 40, 44, 8,  8, 16, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}  /* > 50 */
	},

	/* 1096 <= dbv < 1212 */
	{
		{ 8, 16, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0, 138, 138, 138, 0, 0, 0, 0}, /* -20 ~ -10 */
		{12, 20, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{12, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{16, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{16, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{16, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,  17,  17,  17, 0, 0, 0, 0}, /* 25 ~ 30 */
		{20, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 30 ~ 35 */
		{20, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{24, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{28, 24, 24, 16, 24, 4, 4, 8, 27, 27, 38, 28, 28, 39, 24, 24, 38, 0,   0,   0,   0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{28, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0,  86,  86,  86, 0, 0, 0, 0}  /* > 50 */
	},

	/* 950 <= dbv < 1096 */
	{
		{ 8, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0, 225, 225, 225, 0, 0, 0, 0}, /* -20 ~ -10 */
		{ 8, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{ 8, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{12, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{12, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{12, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,  17,  17,  17, 0, 0, 0, 0}, /* 25 ~ 30 */
		{16, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,  17,  17,  17, 0, 0, 0, 0}, /* 30 ~ 35 */
		{16, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{20, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{20, 16, 16, 12, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 23, 23, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{20, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0}  /* > 50 */
	},

	/* 761 <= dbv < 950 */
	{
		{ 4, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0, 225, 225, 225, 0, 0, 0, 0}, /* -20 ~ -10 */
		{ 8, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* -10 ~ 0 */
		{ 8, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 0 ~ 10 */
		{ 8, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 10 ~ 20 */
		{ 8, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 20 ~ 25 */
		{ 8, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,  86,  86,  86, 0, 0, 0, 0}, /* 25 ~ 30 */
		{12, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,  34,  34,  34, 0, 0, 0, 0}, /* 30 ~ 35 */
		{12, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 35 ~ 40 */
		{16, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 40 ~ 45 */
		{16, 16, 16, 8, 16, 4, 4, 4, 27, 27, 36, 27, 27, 37, 22, 22, 36, 0,   0,   0,   0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{16, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0}  /* > 50 */
	},

	/* 544 <= dbv < 761 */
	{
		{4, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0, 242, 242, 242, 0, 0, 0, 0}, /* -20 ~ -10 */
		{4, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0, 190, 190, 190, 0, 0, 0, 0}, /* -10 ~ 0 */
		{4, 8, 12, 8, 12, 0, 0, 0, 27, 27, 35, 26, 26, 37, 21, 21, 36,  0, 104, 104, 104, 0, 0, 0, 0}, /* 0 ~ 10 */
		{4, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0,  52,  52,  52, 0, 0, 0, 0}, /* 10 ~ 20 */
		{8, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0,  52,  52,  52, 0, 0, 0, 0}, /* 20 ~ 25 */
		{8, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0, 104, 104, 104, 0, 0, 0, 0}, /* 25 ~ 30 */
		{8, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0,  17,  17,  17, 0, 0, 0, 0}, /* 30 ~ 35 */
		{8, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0,  17,  17,  17, 0, 0, 0, 0}, /* 35 ~ 40 */
		{8, 8, 12, 8, 12, 0, 0, 0, 26, 26, 36, 26, 26, 37, 21, 21, 36,  0,  52,  52,  52, 0, 0, 0, 0}, /* 40 ~ 45 */
		{8, 8, 12, 8, 12, 0, 0, 0, 27, 27, 36, 26, 26, 37, 21, 21, 36,  0,   0,   0,   0, 0, 0, 0, 0}, /* 45 ~ 50 */
		{8, 8, 12, 8, 12, 0, 0, 0, 24, 24, 36, 26, 26, 37, 21, 21, 36,  0, 190, 190, 190, 0, 0, 0, 0}  /* > 50 */
	},

	/* dbv < 544 */
	{
		{0, 8, 12, 8, 12, 0, 0, 0, 26, 26, 35, 27, 27, 37, 20, 20, 36, 21,  56,  56,  56, 0, 0, 0, 0}, /* -20 ~ -10 */
		{4, 8, 12, 8, 12, 0, 0, 0, 26, 26, 35, 27, 27, 37, 20, 20, 36, 21,  21,  21,  21, 0, 0, 0, 0}, /* -10 ~ 0 */
		{4, 8, 12, 8, 12, 0, 0, 0, 26, 26, 35, 27, 27, 37, 20, 20, 36,  0, 208, 208, 208, 0, 0, 0, 0}, /* 0 ~ 10 */
		{4, 8, 12, 8, 12, 0, 0, 0, 26, 26, 35, 27, 27, 37, 20, 20, 36,  0, 242, 242, 242, 0, 0, 0, 0}, /* 10 ~ 20 */
		{4, 8, 12, 8, 12, 0, 0, 0, 26, 26, 35, 27, 27, 37, 20, 20, 36,  0, 173, 173, 173, 0, 0, 0, 0}, /* 20 ~ 25 */
		{4, 8, 12, 8, 12, 0, 0, 0, 26, 26, 35, 27, 27, 37, 20, 20, 36,  0, 173, 173, 173, 0, 0, 0, 0}, /* 25 ~ 30 */
		{4, 8, 12, 8, 12, 0, 0, 0, 25, 25, 34, 27, 27, 37, 20, 20, 36,  0, 190, 190, 190, 0, 0, 0, 0}, /* 30 ~ 35 */
		{4, 8, 12, 8, 12, 0, 0, 0, 24, 24, 33, 27, 27, 37, 20, 20, 36, 21,   4,   4,   4, 0, 0, 0, 0}, /* 35 ~ 40 */
		{4, 8, 12, 8, 12, 0, 0, 0, 24, 24, 33, 27, 27, 37, 20, 20, 36,  0, 225, 225, 225, 0, 0, 0, 0}, /* 40 ~ 45 */
		{4, 8, 12, 8, 12, 0, 0, 0, 24, 24, 33, 27, 27, 37, 20, 20, 36,  0, 225, 225, 225, 0, 0, 0, 0}, /* 45 ~ 50 */
		{4, 8, 12, 8, 12, 0, 0, 0, 24, 24, 33, 27, 27, 37, 20, 20, 36,  0, 225, 225, 225, 0, 0, 0, 0}  /* > 50 */
	}
};

/* -------------------- extern -------------------- */
/* extern params */
extern unsigned int hpwm_mode;
extern unsigned int oplus_display_brightness;


/* -------------------- function implementation -------------------- */

int oplus_temp_compensation_register_ntc_channel(void *device)
{
	static bool registered = false;
	int rc = 0;
	unsigned int i = 0;
	struct device *dev = device;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (registered)
		return rc;

	if (IS_ERR_OR_NULL(dev)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -ENODEV;
	}

	g_oplus_temp_compensation_params = devm_kzalloc(dev, sizeof(struct oplus_temp_compensation_params), GFP_KERNEL);
	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("failed to kzalloc g_oplus_temp_compensation_params\n");
		return -EINVAL;
	}

	for (i = 0; i < 5; i++) {
		g_oplus_temp_compensation_params->ntc_temp_chan = devm_iio_channel_get(dev, "panel-channel");
		rc = IS_ERR_OR_NULL(g_oplus_temp_compensation_params->ntc_temp_chan);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to get panel channel\n");
		} else {
			registered = true;
			break;
		}
	}

	if (rc) {
		g_oplus_temp_compensation_params->ntc_temp_chan = NULL;
		devm_kfree(dev, g_oplus_temp_compensation_params);
		g_oplus_temp_compensation_params = NULL;
		rc = -EINVAL;
	} else {
		g_oplus_temp_compensation_params->ntc_temp = 29;
		g_oplus_temp_compensation_params->shell_temp = 29;
		g_oplus_temp_compensation_params->fake_ntc_temp = false;
		g_oplus_temp_compensation_params->fake_shell_temp = false;
		TEMP_COMPENSATION_INFO("register ntc channel successfully\n");
	}

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_temp_compensation_register_ntc_channel);

static int oplus_temp_compensation_volt_to_temp(int volt)
{
	int volt_avg = 0;
	unsigned int i = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	for (i = 0; i < ARRAY_SIZE(con_temp_ntc_100k_1840mv) - 1; i++) {
		if ((volt >= con_volt_ntc_100k_1840mv[i + 1])
				&& (volt <= con_volt_ntc_100k_1840mv[i])) {
			volt_avg = (con_volt_ntc_100k_1840mv[i + 1] + con_volt_ntc_100k_1840mv[i]) / 2;
			if(volt <= volt_avg)
				i++;
			break;
		}
	}

	TEMP_COMPENSATION_DEBUG("end\n");

	return con_temp_ntc_100k_1840mv[i];
}

static int oplus_temp_compensation_get_ntc_temp(void)
{
	int rc = 0;
	int val_avg = 0;
	int val[3] = {0, 0, 0};
	unsigned int i = 0;
	unsigned int count = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid g_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params->ntc_temp_chan)) {
		TEMP_COMPENSATION_ERR("Invalid ntc_temp_chan params\n");
		g_oplus_temp_compensation_params->ntc_temp = 29;
		return -EINVAL;
	}

	if (g_oplus_temp_compensation_params->fake_ntc_temp) {
		TEMP_COMPENSATION_DEBUG("fake panel ntc temp is %d\n", g_oplus_temp_compensation_params->ntc_temp);
		return g_oplus_temp_compensation_params->ntc_temp;
	}

	do {
		for (i = 0; i < 3; i++) {
			rc = iio_read_channel_processed(g_oplus_temp_compensation_params->ntc_temp_chan, &val[i]);
			TEMP_COMPENSATION_DEBUG("ntc_volt:%d\n", val[i]);
			if (rc < 0) {
				TEMP_COMPENSATION_ERR("read ntc_temp_chan volt failed, rc=%d\n", rc);
			} else {
				val[i] = oplus_temp_compensation_volt_to_temp(val[i]);
				TEMP_COMPENSATION_DEBUG("ntc_temp:%d\n", val[i]);
			}
		}

		if (count) {
			TEMP_COMPENSATION_ERR("retry %u\n", count);
		}
		count++;
	} while (((abs(val[0] - val[1]) >= 2) || (abs(val[0] - val[2]) >= 2)
				|| (abs(val[1] - val[2]) >= 2)) && (count < 5));

	if (count == 5) {
		TEMP_COMPENSATION_ERR("use last panel ntc temp %d\n", g_oplus_temp_compensation_params->ntc_temp);
		return g_oplus_temp_compensation_params->ntc_temp;
	} else {
		val_avg = (val[0] + val[1] + val[2]) / 3;
	}

	g_oplus_temp_compensation_params->ntc_temp = val_avg;
	TEMP_COMPENSATION_DEBUG("panel ntc temp is %d\n", g_oplus_temp_compensation_params->ntc_temp);

	TEMP_COMPENSATION_DEBUG("end\n");

	return g_oplus_temp_compensation_params->ntc_temp;
}

static int oplus_temp_compensation_get_shell_temp(void)
{
	int temp = -127000;
	int max_temp = -127000;
	unsigned int i = 0;
	const char *shell_tz[] = {"shell_front", "shell_frame", "shell_back"};
	struct thermal_zone_device *tz = NULL;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (g_oplus_temp_compensation_params->fake_shell_temp) {
		TEMP_COMPENSATION_DEBUG("fake shell temp is %d\n", g_oplus_temp_compensation_params->shell_temp);
		return g_oplus_temp_compensation_params->shell_temp;
	}

	for (i = 0; i < ARRAY_SIZE(shell_tz); i++) {
		tz = thermal_zone_get_zone_by_name(shell_tz[i]);
		thermal_zone_get_temp(tz, &temp);
		if (max_temp < temp) {
			max_temp = temp;
		}
	}

	g_oplus_temp_compensation_params->shell_temp = max_temp / 1000;
	TEMP_COMPENSATION_DEBUG("shell temp is %d\n", g_oplus_temp_compensation_params->shell_temp);

	TEMP_COMPENSATION_DEBUG("end\n");

	return g_oplus_temp_compensation_params->shell_temp;
}

void oplus_temp_compensation_vpark_set(unsigned char voltage, struct LCM_setting_table *temp_compensation_cmd)
{
	unsigned char voltage1, voltage2, voltage3, voltage4;
	unsigned short vpark = (69 - voltage) * 1024 / (69 - 10);

	TEMP_COMPENSATION_DEBUG("start\n");

	voltage1 = ((vpark & 0xFF00) >> 8) + ((vpark & 0xFF00) >> 6) + ((vpark & 0xFF00) >> 4);
	voltage2 = vpark & 0xFF;
	voltage3 = vpark & 0xFF;
	voltage4 = vpark & 0xFF;
	temp_compensation_cmd[17].para_list[0+1] = voltage1;
	temp_compensation_cmd[17].para_list[1+1] = voltage2;
	temp_compensation_cmd[17].para_list[2+1] = voltage3;
	temp_compensation_cmd[17].para_list[3+1] = voltage4;

	TEMP_COMPENSATION_DEBUG("end\n");
}

static unsigned int oplus_temp_compensation_get_temp_index(int temp)
{
	unsigned int temp_index = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (temp < -10) {
		temp_index = OPLUS_TEMP_COMPENSATION_LESS_THAN_MINUS10_TEMP_INDEX;
	} else if (temp < 0) {
		temp_index = OPLUS_TEMP_COMPENSATION_MINUS10_0_TEMP_INDEX;
	} else if (temp < 10) {
		temp_index = OPLUS_TEMP_COMPENSATION_0_10_TEMP_INDEX;
	} else if (temp < 20) {
		temp_index = OPLUS_TEMP_COMPENSATION_10_20_TEMP_INDEX;
	} else if (temp < 25) {
		temp_index = OPLUS_TEMP_COMPENSAITON_20_25_TEMP_INDEX;
	} else if (temp < 30) {
		temp_index = OPLUS_TEMP_COMPENSATION_25_30_TEMP_INDEX;
	} else if (temp < 35) {
		temp_index = OPLUS_TEMP_COMPENSAITON_30_35_TEMP_INDEX;
	} else if (temp < 40) {
		temp_index = OPLUS_TEMP_COMPENSATION_35_40_TEMP_INDEX;
	} else if (temp < 45) {
		temp_index = OPLUS_TEMP_COMPENSATION_40_45_TEMP_INDEX;
	} else if (temp <= 50) {
		temp_index = OPLUS_TEMP_COMPENSATION_45_50_TEMP_INDEX;
	} else {
		temp_index = OPLUS_TEMP_COMPENSATION_GREATER_THAN_50_TEMP_INDEX;
	}

	TEMP_COMPENSATION_DEBUG("temp_index:%u\n", temp_index);

	TEMP_COMPENSATION_DEBUG("end\n");

	return temp_index;
}

static int oplus_temp_compensation_send_pack_hs_cmd(void *dsi, void *LCM_setting_table, unsigned int lcm_cmd_count, void *p_dcs_write_gce_pack, void *handle)
{
	unsigned int i = 0;
	struct LCM_setting_table *table = LCM_setting_table;
	dcs_write_gce_pack cb = p_dcs_write_gce_pack;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!dsi || !table || !lcm_cmd_count || !cb) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		TEMP_COMPENSATION_ERR("out of mtk_ddic_dsi_cmd\n");
		return -EINVAL;
	}

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}
	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;

	cb(dsi, handle, &send_cmd_to_ddic);

	TEMP_COMPENSATION_DEBUG("end\n");

	return 0;
}

int oplus_temp_compensation_cmd_set(void *dsi, void *p_dcs_write_gce_pack, void *handle, unsigned int setting_mode)
{
	int rc = 0;
	int ntc_temp = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int lcm_cmd_count = 0;
	unsigned int dbv_index = 0;
	unsigned int temp_index = 0;
	unsigned int bl_lvl = 0;
	static unsigned int last_dbv_index = 0;
	static unsigned int last_temp_index = 0;
	static unsigned int last_bl_lvl = 0;		/* Force sending temp compensation cmd when booting up */
	dcs_write_gce_pack cb = p_dcs_write_gce_pack;
	struct LCM_setting_table temp_compensation_cmd[] = {
		{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}},
		{REGFLAG_CMD, 2, {0x6F, 0x06}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x0A}},
		{REGFLAG_CMD, 5, {0xE5, 0x04, 0x04, 0x04, 0x04}},
		{REGFLAG_CMD, 2, {0x6F, 0x0E}},
		{REGFLAG_CMD, 5, {0xE5, 0x2C, 0x2C, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x28}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x2C}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x4A}},
		{REGFLAG_CMD, 5, {0xE5, 0x00, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x4E}},
		{REGFLAG_CMD, 5, {0xE5, 0x04, 0x04, 0x04, 0x04}},
		{REGFLAG_CMD, 2, {0x6F, 0x52}},
		{REGFLAG_CMD, 5, {0xE5, 0x2C, 0x2C, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x03}},
		{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x0C}},
		{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 2, {0x6F, 0x15}},
		{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
		{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
		{REGFLAG_CMD, 2, {0x6F, 0x12}},
		{REGFLAG_CMD, 5, {0xEC, 0x00, 0x00, 0x00, 0x00}},
	};

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!dsi || !cb) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	bl_lvl = oplus_display_brightness;

//#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_support()) {
		if ((oplus_ofp_get_hbm_state() && (setting_mode == OPLUS_TEMP_COMPENSATION_NORMAL_SETTING))
				|| (setting_mode == OPLUS_TEMP_COMPENSATION_FOD_ON_SETTING)) {
			bl_lvl = 3840;
		}
	}
//#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	if (bl_lvl > 3515) {
		dbv_index = OPLUS_TEMP_COMPENSATION_GREATER_THAN_3515_DBV_INDEX;
	} else if (bl_lvl >= 1604) {
		dbv_index = OPLUS_TEMP_COMPENSATION_1604_3515_DBV_INDEX;
	} else if (bl_lvl >= 1511) {
		dbv_index = OPLUS_TEMP_COMPENSATION_1511_1604_DBV_INDEX;
	} else if (bl_lvl >= 1419) {
		dbv_index = OPLUS_TEMP_COMPENSATION_1419_1511_DBV_INDEX;
	} else if (bl_lvl >= 1328) {
		dbv_index = OPLUS_TEMP_COMPENSATION_1328_1419_DBV_INDEX;
	} else if (bl_lvl >= 1212) {
		dbv_index = OPLUS_TEMP_COMPENSATION_1212_1328_DBV_INDEX;
	} else if (bl_lvl >= 1096) {
		dbv_index = OPLUS_TEMP_COMPENSATION_1096_1212_DBV_INDEX;
	} else if (bl_lvl >= 950) {
		dbv_index = OPLUS_TEMP_COMPENSATION_950_1096_DBV_INDEX;
	} else if (bl_lvl >= 761) {
		dbv_index = OPLUS_TEMP_COMPENSATION_761_950_DBV_INDEX;
	} else if (bl_lvl >= 544) {
		dbv_index = OPLUS_TEMP_COMPENSATION_544_761_DBV_INDEX;
	} else {
		dbv_index = OPLUS_TEMP_COMPENSATION_LESS_THAN_544_DBV_INDEX;
	}

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_DEBUG("panel ntc is not exist, use default ntc temp value\n");
		ntc_temp = 29;
	} else {
		ntc_temp = g_oplus_temp_compensation_params->ntc_temp;
	}

	temp_index = oplus_temp_compensation_get_temp_index(ntc_temp);

	TEMP_COMPENSATION_DEBUG("bl_lvl=%u,ntc_temp=%d,shell_temp=%d,dbv_index=%u,temp_index=%u,last_dbv_index=%u,last_temp_index=%u\n",
			bl_lvl, ntc_temp, oplus_temp_compensation_get_shell_temp(), dbv_index, temp_index, last_dbv_index, last_temp_index);

	if ((last_dbv_index != dbv_index) || (last_temp_index != temp_index) || (!last_bl_lvl && bl_lvl)) {
		for (i = 0; i < 4; i++) {
			temp_compensation_cmd[2].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][0];
			temp_compensation_cmd[4].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][1];
			temp_compensation_cmd[6].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][2];
			temp_compensation_cmd[12].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][5];
			temp_compensation_cmd[14].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][6];
			temp_compensation_cmd[16].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][7];

			if (hpwm_mode) {
				temp_compensation_cmd[25].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][21+i];
			} else {
				temp_compensation_cmd[25].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][17+i];
			}
		}

		for (i = 0; i < 3; i++) {
			temp_compensation_cmd[8].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][3];
			temp_compensation_cmd[10].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][4];
			temp_compensation_cmd[18].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][8+i];
			temp_compensation_cmd[20].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][11+i];
			temp_compensation_cmd[22].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][14+i];
		}

		if (last_dbv_index != dbv_index) {
			TEMP_COMPENSATION_INFO("dbv index change, set temp compensation cmd\n");
		} else if (last_temp_index != temp_index) {
			TEMP_COMPENSATION_INFO("temp index change, set temp compensation cmd\n");
		} else if (!last_bl_lvl && bl_lvl) {
			TEMP_COMPENSATION_INFO("panel resume, set temp compensation cmd\n");
		}

		lcm_cmd_count = sizeof(temp_compensation_cmd) / sizeof(struct LCM_setting_table);
		rc = oplus_temp_compensation_send_pack_hs_cmd(dsi, temp_compensation_cmd, lcm_cmd_count, cb, handle);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to send pack hs cmd\n");
		}

		if ((oplus_temp_compensation_log_level >= OPLUS_TEMP_COMPENSATION_LOG_LEVEL_DEBUG)
				&& (oplus_dsi_log_type & OPLUS_TEMP_COMPENSATION_DEBUG_LOG_TEMP_COMPENSATION)) {
			for(i = 0; i < lcm_cmd_count; i++) {
				pr_info(KERN_CONT "[TEMP_COMPENSATION][DEBUG][%s:%d]temp_compensation_cmd[%u]", __func__, __LINE__, i);
				for (j = 0; j < temp_compensation_cmd[i].count; j++) {
					pr_info(KERN_CONT "0x%02X ", temp_compensation_cmd[i].para_list[j]);
				}
				pr_info(KERN_CONT "\n");
			}
		}
	}

	last_dbv_index = dbv_index;
	last_temp_index = temp_index;
	last_bl_lvl = bl_lvl;

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_temp_compensation_cmd_set);

int oplus_temp_compensation_temp_check(void *mtk_ddp_comp, void *cmdq_pkt)
{
	int rc = 0;
	int ntc_temp = 0;
	int shell_temp = 0;
	int last_ntc_temp = 0;
	unsigned int setting_mode = OPLUS_TEMP_COMPENSATION_NORMAL_SETTING;
	struct mtk_ddp_comp *output_comp = mtk_ddp_comp;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!output_comp || !cmdq_handle) {
		TEMP_COMPENSATION_ERR("Invalid intput params\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_DEBUG("Invalid g_oplus_temp_compensation_params params\n");
		return -EINVAL;
	}

	last_ntc_temp = g_oplus_temp_compensation_params->ntc_temp;
	ntc_temp = oplus_temp_compensation_get_ntc_temp();
	shell_temp = oplus_temp_compensation_get_shell_temp();

	if (oplus_temp_compensation_get_temp_index(last_ntc_temp) != oplus_temp_compensation_get_temp_index(ntc_temp)) {
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_SET, &setting_mode);
		TEMP_COMPENSATION_INFO("last_ntc_temp:%d,current_ntc_temp:%d,bl_lvl:%u,update temp compensation cmd\n", last_ntc_temp, ntc_temp, oplus_display_brightness);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_is_support()) {
		rc = oplus_adfr_temperature_detection_handle(output_comp, cmdq_handle, ntc_temp, shell_temp);
		if (rc) {
			TEMP_COMPENSATION_ERR("failed to handle temperature detection\n");
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	TEMP_COMPENSATION_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_temp_compensation_temp_check);


/* -------------------- node -------------------- */
/* ntc temp */
ssize_t oplus_temp_compensation_set_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ntc_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!buf) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("ntc temp setting is not supported\n");
		return count;
	}

	sscanf(buf, "%u", &ntc_temp);
	TEMP_COMPENSATION_INFO("ntc_temp:%d\n", ntc_temp);

	if (ntc_temp == -1) {
		g_oplus_temp_compensation_params->fake_ntc_temp = false;
	} else {
		g_oplus_temp_compensation_params->ntc_temp = ntc_temp;
		g_oplus_temp_compensation_params->fake_ntc_temp = true;
	}

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_ntc_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int ntc_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!buf) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	ntc_temp = oplus_temp_compensation_get_ntc_temp();
	TEMP_COMPENSATION_INFO("ntc_temp:%d\n", ntc_temp);

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%d\n", ntc_temp);
}

/* shell temp */
ssize_t oplus_temp_compensation_set_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int shell_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!buf) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return count;
	}

	if (IS_ERR_OR_NULL(g_oplus_temp_compensation_params)) {
		TEMP_COMPENSATION_ERR("shell temp setting is not supported\n");
		return count;
	}

	sscanf(buf, "%u", &shell_temp);
	TEMP_COMPENSATION_INFO("shell_temp:%d\n", shell_temp);

	if (shell_temp == -1) {
		g_oplus_temp_compensation_params->fake_shell_temp = false;
	} else {
		g_oplus_temp_compensation_params->shell_temp = shell_temp;
		g_oplus_temp_compensation_params->fake_shell_temp = true;
	}

	TEMP_COMPENSATION_DEBUG("end\n");

	return count;
}

ssize_t oplus_temp_compensation_get_shell_temp_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	int shell_temp = 0;

	TEMP_COMPENSATION_DEBUG("start\n");

	if (!buf) {
		TEMP_COMPENSATION_ERR("Invalid params\n");
		return -EINVAL;
	}

	shell_temp = oplus_temp_compensation_get_shell_temp();
	TEMP_COMPENSATION_INFO("shell_temp:%d\n", shell_temp);

	TEMP_COMPENSATION_DEBUG("end\n");

	return sprintf(buf, "%d\n", shell_temp);
}
