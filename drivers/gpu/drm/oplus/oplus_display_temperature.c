/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_temperature.c
** Description : oplus_display_temperature implement
** Version : 1.0
** Date : 2022/11/20
** Author : Display
***************************************************************/

#include "oplus_display_temperature.h"
#include "oplus_adfr_ext.h"
#include <linux/thermal.h>
#include <drm/drm_device.h>
#include "mtk_panel_ext.h"
#include "mtk_drm_ddp_comp.h"

#define CHARGER_25C_VOLT	900
#define REGFLAG_CMD			0xFFFA

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

extern unsigned int lcm_id1;
extern unsigned int lcm_id2;
extern unsigned int oplus_display_brightness;
struct oplus_display_temp *g_oplus_display_temp = NULL;

/* ntc_resistance:100k internal_pull_up:100k voltage:1.84v */
int con_temp_ntc_100k_1840mv[] = {
	-40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22,
	-21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
	26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
	74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
	98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
	117, 118, 119, 120, 121, 122, 123, 124, 125
};

int con_volt_ntc_100k_1840mv[] = {
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
/* ntc_resistance:100k internal_pull_up:100k voltage:1.84v */
unsigned char temp_compensation_paras[11][11][25] = {
	/* dbv > 3515 */
	{
		{16, 20, 24, 16, 20, 16, 20, 24, 58, 58, 58, 58, 58, 58, 58, 58, 58, 0, 0, 0, 0, 0, 0, 0, 0},		/* -20 ~ -10 */
		{16, 20, 24, 16, 20, 16, 20, 24, 58, 58, 58, 58, 58, 58, 58, 58, 58, 0, 0, 0, 0, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 20, 24, 16, 20, 16, 20, 24, 57, 57, 57, 57, 57, 57, 57, 57, 57, 0, 0, 0, 0, 0, 0, 0, 0},		/* 0 ~ 10 */
		{20, 24, 28, 20, 24, 20, 24, 28, 56, 56, 56, 56, 56, 56, 56, 56, 56, 0, 0, 0, 0, 0, 0, 0, 0},		/* 10 ~ 20 */
		{28, 32, 36, 28, 32, 28, 32, 36, 55, 55, 55, 55, 55, 55, 55, 55, 55, 0, 0, 0, 0, 0, 0, 0, 0},		/* 20 ~ 25 */
		{36, 40, 48, 36, 40, 36, 40, 48, 55, 55, 55, 55, 55, 55, 55, 55, 55, 0, 0, 0, 0, 0, 0, 0, 0},		/* 25 ~ 30 */
		{40, 44, 52, 40, 44, 40, 44, 52, 54, 54, 54, 54, 54, 54, 54, 54, 54, 0, 0, 0, 0, 0, 0, 0, 0},		/* 30 ~ 35 */
		{44, 48, 56, 44, 48, 44, 48, 56, 54, 54, 54, 54, 54, 54, 54, 54, 54, 0, 0, 0, 0, 0, 0, 0, 0},		/* 35 ~ 40 */
		{48, 52, 60, 48, 52, 48, 52, 60, 53, 53, 53, 53, 53, 53, 53, 53, 53, 0, 0, 0, 0, 0, 0, 0, 0},		/* 40 ~ 45 */
		{52, 56, 64, 52, 56, 52, 56, 64, 53, 53, 53, 53, 53, 53, 53, 53, 53, 0, 0, 0, 0, 0, 0, 0, 0},		/* 45 ~ 50 */
		{56, 60, 68, 56, 60, 56, 60, 68, 52, 52, 52, 52, 52, 52, 52, 52, 52, 0, 0, 0, 0, 0, 0, 0, 0}		/* > 50 */
	},

	/* 1604 <= dbv <= 3515 */
	{
		{16, 20, 24, 16, 20, 16, 20, 24, 30, 30, 39, 31, 31, 40, 30, 30, 39, 0, 0, 0, 0, 0, 0, 0, 0},		/* -20 ~ -10 */
		{16, 20, 24, 16, 20, 16, 20, 24, 29, 29, 38, 30, 30, 39, 29, 29, 38, 0, 0, 0, 0, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 20, 24, 16, 20, 16, 20, 24, 28, 28, 37, 29, 29, 38, 28, 28, 37, 0, 0, 0, 0, 0, 0, 0, 0},		/* 0 ~ 10 */
		{20, 24, 28, 20, 24, 20, 24, 28, 28, 28, 37, 29, 29, 38, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 10 ~ 20 */
		{28, 32, 36, 28, 32, 28, 32, 36, 27, 27, 36, 28, 28, 37, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 20 ~ 25 */
		{36, 40, 48, 36, 40, 36, 40, 48, 27, 27, 36, 28, 28, 37, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 25 ~ 30 */
		{40, 44, 52, 40, 44, 40, 44, 52, 26, 26, 35, 27, 27, 36, 26, 26, 35, 0, 0, 0, 0, 0, 0, 0, 0},		/* 30 ~ 35 */
		{44, 48, 56, 44, 48, 44, 48, 56, 26, 26, 35, 27, 27, 36, 26, 26, 35, 0, 0, 0, 0, 0, 0, 0, 0},		/* 35 ~ 40 */
		{48, 52, 60, 48, 52, 48, 52, 60, 25, 25, 34, 26, 26, 35, 25, 25, 34, 0, 0, 0, 0, 0, 0, 0, 0},		/* 40 ~ 45 */
		{52, 56, 64, 52, 56, 52, 56, 64, 25, 25, 34, 26, 26, 35, 25, 25, 34, 0, 0, 0, 0, 0, 0, 0, 0},		/* 45 ~ 50 */
		{56, 60, 68, 56, 60, 56, 60, 68, 24, 24, 33, 25, 25, 34, 24, 24, 33, 0, 0, 0, 0, 0, 0, 0, 0}		/* > 50 */
	},

	/* 1511 <= dbv < 1604 */
	{
		{12, 16, 20, 12, 16,  4,  4,  8, 30, 30, 39, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -20 ~ -10 */
		{16, 20, 24, 16, 20,  4,  4,  8, 29, 29, 38, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 20, 24, 16, 20,  4,  8, 12, 29, 29, 38, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 0 ~ 10 */
		{20, 20, 24, 20, 20,  4,  8, 12, 28, 28, 37, 29, 29, 38, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 10 ~ 20 */
		{24, 28, 32, 24, 28,  8,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 20 ~ 25 */
		{32, 36, 44, 32, 36,  8, 12, 16, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 25 ~ 30 */
		{36, 40, 48, 36, 40,  8, 16, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 30 ~ 35 */
		{40, 44, 52, 40, 44,  8, 16, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 35 ~ 40 */
		{44, 48, 56, 44, 48,  8, 20, 24, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 40 ~ 45 */
		{48, 52, 60, 48, 52, 12, 20, 24, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 45 ~ 50 */
		{52, 56, 64, 52, 56, 12, 20, 24, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0}		/* > 50 */
	},

	/* 1419 <= dbv < 1511 */
	{
		{12, 16, 20, 12, 16, 4,  4,  8, 30, 30, 39, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -20 ~ -10 */
		{16, 16, 20, 16, 16, 4,  4,  8, 29, 29, 38, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 20, 24, 16, 20, 4,  8, 12, 29, 29, 38, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 0 ~ 10 */
		{20, 20, 24, 20, 20, 4,  8, 12, 28, 28, 37, 29, 29, 38, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 10 ~ 20 */
		{20, 24, 28, 20, 24, 8,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 20 ~ 25 */
		{28, 32, 40, 28, 32, 8, 12, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 25 ~ 30 */
		{32, 36, 44, 32, 36, 8, 12, 16, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 30 ~ 35 */
		{36, 40, 48, 36, 40, 8, 12, 16, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 35 ~ 40 */
		{40, 44, 52, 40, 44, 8, 16, 20, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 40 ~ 45 */
		{44, 48, 56, 44, 48, 8, 16, 20, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 45 ~ 50 */
		{48, 52, 60, 48, 52, 8, 16, 20, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0} 		/* > 50 */
	},

	/* 1328 <= dbv < 1419 */
	{
		{12, 16, 20, 12, 16, 4,  4,  8, 30, 30, 39, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -20 ~ -10 */
		{12, 16, 20, 12, 16, 4,  4,  8, 29, 29, 38, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 16, 24, 16, 16, 4,  4, 12, 29, 29, 38, 30, 30, 39, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 0 ~ 10 */
		{16, 20, 24, 16, 20, 4,  8, 12, 28, 28, 37, 29, 29, 38, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 10 ~ 20 */
		{20, 24, 24, 20, 24, 4,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 20 ~ 25 */
		{24, 28, 36, 24, 28, 4,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 25 ~ 30 */
		{28, 32, 40, 28, 32, 8,  8, 12, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 30 ~ 35 */
		{32, 36, 44, 32, 36, 8, 12, 16, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 35 ~ 40 */
		{36, 40, 48, 36, 40, 8, 12, 16, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 40 ~ 45 */
		{40, 44, 52, 40, 44, 8, 16, 20, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 45 ~ 50 */
		{44, 48, 56, 44, 48, 8, 16, 20, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0} 		/* > 50 */
	},

	/* 1212 <= dbv < 1328 */
	{
		{ 8, 12, 16,  8, 12, 4,  4,  8, 30, 30, 39, 30, 30, 39, 28, 28, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -20 ~ -10 */
		{12, 12, 16, 12, 12, 4,  4,  8, 29, 29, 38, 30, 30, 39, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 16, 24, 16, 16, 4,  4, 12, 29, 29, 38, 30, 30, 39, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 0 ~ 10 */
		{16, 16, 24, 16, 16, 4,  4, 12, 28, 28, 37, 29, 29, 38, 27, 27, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 10 ~ 20 */
		{16, 20, 24, 16, 20, 4,  8, 12, 27, 27, 36, 28, 28, 37, 26, 26, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 20 ~ 25 */
		{20, 24, 32, 20, 24, 4,  8, 12, 27, 27, 36, 28, 28, 37, 25, 25, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 25 ~ 30 */
		{24, 28, 36, 24, 28, 4, 12, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 30 ~ 35 */
		{28, 32, 40, 28, 32, 4, 12, 20, 26, 26, 35, 27, 27, 36, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 35 ~ 40 */
		{32, 36, 44, 32, 36, 8,  8, 16, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 40 ~ 45 */
		{36, 40, 48, 36, 40, 8,  8, 16, 25, 25, 34, 26, 26, 35, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0},		/* 45 ~ 50 */
		{40, 44, 52, 40, 44, 8,  8, 16, 24, 24, 33, 25, 25, 34, 24, 24, 36, 0, 0, 0, 0, 0, 0, 0, 0} 		/* > 50 */
	},

	/* 1096 <= dbv < 1212 */
	{
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* -20 ~ -10 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* -10 ~ 0 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 0 ~ 10 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 10 ~ 20 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 20 ~ 25 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 25 ~ 30 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 30 ~ 35 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 35 ~ 40 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 40 ~ 45 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0},		/* 45 ~ 50 */
		{16, 24, 24, 16, 24, 4, 4, 8, 26, 26, 38, 28, 28, 39, 24, 24, 38, 0, 86, 86, 86, 0, 0, 0, 0}		/* > 50 */
	},

	/* 950 <= dbv < 1096 */
	{
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* -20 ~ -10 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* -10 ~ 0 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 0 ~ 10 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 10 ~ 20 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 20 ~ 25 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 25 ~ 30 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 30 ~ 35 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 35 ~ 40 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 40 ~ 45 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0},	/* 45 ~ 50 */
		{12, 16, 16, 12, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 23, 23, 36, 0, 190, 190, 190, 0, 0, 0, 0}		/* > 50 */
	},

	/* 761 <= dbv < 950 */
	{
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* -20 ~ -10 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* -10 ~ 0 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 0 ~ 10 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 10 ~ 20 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 20 ~ 25 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 25 ~ 30 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 30 ~ 35 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 35 ~ 40 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 40 ~ 45 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0},		/* 45 ~ 50 */
		{8, 16, 16, 8, 16, 4, 4, 4, 25, 25, 36, 27, 27, 37, 22, 22, 36, 0, 190, 190, 190, 0, 0, 0, 0}		/* > 50 */
	},

	/* 544 <= dbv < 761 */
	{
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* -20 ~ -10 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* -10 ~ 0 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 0 ~ 10 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 10 ~ 20 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 20 ~ 25 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 25 ~ 30 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 30 ~ 35 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 35 ~ 40 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 40 ~ 45 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0},			/* 45 ~ 50 */
		{8, 8, 12, 8, 12, 4, 4, 4, 24, 24, 36, 26, 26, 37, 21, 21, 36, 21, 21, 21, 21, 0, 0, 0, 0}			/* > 50 */
	},

	/* dbv < 544 */
	{
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* -20 ~ -10 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* -10 ~ 0 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 0 ~ 10 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 10 ~ 20 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 20 ~ 25 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 25 ~ 30 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 30 ~ 35 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 35 ~ 40 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 40 ~ 45 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0},			/* 45 ~ 50 */
		{8, 8, 12, 8, 12, 4, 4, 4, 25, 25, 36, 27, 27, 37, 20, 20, 36, 0, 190, 190, 190, 0, 0, 0, 0}			/* > 50 */
	}
};

int oplus_display_register_ntc_channel(void *device)
{
	struct device *dev = device;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!dev) {
		pr_err("[DISP][ERR][%s:%d]Invalid params\n", __func__, __LINE__);
		return -ENODEV;
	}

	g_oplus_display_temp = devm_kzalloc(dev, sizeof(struct oplus_display_temp), GFP_KERNEL);
	if (!g_oplus_display_temp) {
		pr_err("[DISP][ERR][%s:%d]failed to kzalloc g_oplus_display_temp\n", __func__, __LINE__);
		return -EINVAL;
	}

	g_oplus_display_temp->ntc_temp_chan = devm_iio_channel_get(dev, "panel-channel");
	if (IS_ERR(g_oplus_display_temp->ntc_temp_chan)) {
		pr_err("[DISP][ERR][%s:%d]failed to get panel channel\n", __func__, __LINE__);
		g_oplus_display_temp->ntc_temp_chan = NULL;
		devm_kfree(dev, g_oplus_display_temp);
		g_oplus_display_temp = NULL;
		return -EINVAL;
	}

	pr_info("[DISP][INFO][%s:%d]register ntc channel successfully\n", __func__, __LINE__);
	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return 0;
}
EXPORT_SYMBOL(oplus_display_register_ntc_channel);

static int oplus_display_volt_to_temp(int volt)
{
	int i, volt_avg;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	for (i = 0; i < ARRAY_SIZE(con_temp_ntc_100k_1840mv) - 1; i++) {
		if ((volt >= con_volt_ntc_100k_1840mv[i + 1]) && (volt <= con_volt_ntc_100k_1840mv[i])) {
			volt_avg = (con_volt_ntc_100k_1840mv[i + 1] + con_volt_ntc_100k_1840mv[i]) / 2;
			if(volt <= volt_avg)
				i++;
			break;
		}
	}

	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return con_temp_ntc_100k_1840mv[i];
}

int oplus_display_get_ntc_temp(void)
{
	int val = 0;
	int rc = 0;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!g_oplus_display_temp || !g_oplus_display_temp->ntc_temp_chan) {
		pr_debug("[DISP][DEBUG][%s:%d]Invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	rc = iio_read_channel_processed(g_oplus_display_temp->ntc_temp_chan, &val);
	if (rc < 0) {
		pr_err("[DISP][ERR][%s:%d]read ntc_temp_chan volt failed, rc=%d\n", __func__, __LINE__, rc);
		return rc;
	}

	pr_debug("[DISP][DEBUG][%s:%d]panel ntc voltage is %d\n", __func__, __LINE__, val);
	if (val <= 0) {
		val = CHARGER_25C_VOLT;
	}

	g_oplus_display_temp->ntc_temp = oplus_display_volt_to_temp(val);

	pr_info("[DISP][INFO][%s:%d]panel ntc temp is %d\n", __func__, __LINE__, g_oplus_display_temp->ntc_temp);
	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return g_oplus_display_temp->ntc_temp;
}
EXPORT_SYMBOL(oplus_display_get_ntc_temp);

int oplus_display_get_shell_temp(void)
{
	int i = 0;
	int temp = -127000;
	int max_temp = -127000;
	const char *shell_tz[] = {"shell_front", "shell_frame", "shell_back"};
	struct thermal_zone_device *tz = NULL;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!g_oplus_display_temp) {
		pr_debug("[DISP][DEBUG][%s:%d]Invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(shell_tz); i++) {
		tz = thermal_zone_get_zone_by_name(shell_tz[i]);
		thermal_zone_get_temp(tz, &temp);
		if (max_temp < temp) {
			max_temp = temp;
		}
	}

	g_oplus_display_temp->shell_temp = max_temp / 1000;

	pr_info("[DISP][INFO][%s:%d]shell temp is %d\n", __func__, __LINE__, g_oplus_display_temp->shell_temp);
	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return g_oplus_display_temp->shell_temp;
}
EXPORT_SYMBOL(oplus_display_get_shell_temp);

static void vpark_set(unsigned char voltage, struct LCM_setting_table *temp_compensation_cmd)
{
	uint8_t voltage1, voltage2, voltage3, voltage4;
	unsigned short vpark = (69 - voltage) * 1024 / (69 - 10);

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	voltage1 = ((vpark & 0xFF00) >> 8) + ((vpark & 0xFF00) >> 6) + ((vpark & 0xFF00) >> 4);
	voltage2 = vpark & 0xFF;
	voltage3 = vpark & 0xFF;
	voltage4 = vpark & 0xFF;
	temp_compensation_cmd[17].para_list[0+1] = voltage1;
	temp_compensation_cmd[17].para_list[1+1] = voltage2;
	temp_compensation_cmd[17].para_list[2+1] = voltage3;
	temp_compensation_cmd[17].para_list[3+1] = voltage4;

	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);
}

static unsigned int oplus_display_get_temp_index(int temp)
{
	unsigned int temp_index = 0;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (temp < -10) {
		temp_index = OPLUS_DISPLAY_LESS_THAN_MINUS10_TEMP_INDEX;
	} else if (temp < 0) {
		temp_index = OPLUS_DISPLAY_MINUS10_0_TEMP_INDEX;
	} else if (temp < 10) {
		temp_index = OPLUS_DISPLAY_0_10_TEMP_INDEX;
	} else if (temp < 20) {
		temp_index = OPLUS_DISPLAY_10_20_TEMP_INDEX;
	} else if (temp < 25) {
		temp_index = OPLUS_DISPLAY_20_25_TEMP_INDEX;
	} else if (temp < 30) {
		temp_index = OPLUS_DISPLAY_25_30_TEMP_INDEX;
	} else if (temp < 35) {
		temp_index = OPLUS_DISPLAY_30_35_TEMP_INDEX;
	} else if (temp < 40) {
		temp_index = OPLUS_DISPLAY_35_40_TEMP_INDEX;
	} else if (temp < 45) {
		temp_index = OPLUS_DISPLAY_40_45_TEMP_INDEX;
	} else if (temp <= 50) {
		temp_index = OPLUS_DISPLAY_45_50_TEMP_INDEX;
	} else {
		temp_index = OPLUS_DISPLAY_GREATER_THAN_50_TEMP_INDEX;
	}

	pr_debug("[DISP][DEBUG][%s:%d]temp_index:%d\n", __func__, __LINE__, temp_index);
	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return temp_index;
}

int oplus_display_temp_compensation_set(void *dsi, void *p_dcs_write_gce, void *handle, unsigned int bl_lvl)
{
	int i = 0;
	int ntc_temp = 0;
	unsigned int dbv_index = 0;
	unsigned int temp_index = 0;
	unsigned char ED_120[3] = {40, 44, 56};
	unsigned char ED_90[2] = {40, 56};
	unsigned char Frameskip_Vset_120[3] = {0};
	unsigned char Frameskip_Vset_90[3] = {0};
	dcs_write_gce cb = p_dcs_write_gce;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!dsi || !cb) {
		pr_err("[DISP][ERR][%s:%d]Invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	if ((lcm_id1 == 0x76) && ((lcm_id2 != 3) && (lcm_id2 < 5))) {
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
			{REGFLAG_CMD, 2, {0x6F, 0x03}},
			{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
			{REGFLAG_CMD, 2, {0x6F, 0x0C}},
			{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
			{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
			{REGFLAG_CMD, 2, {0x6F, 0x12}},
			{REGFLAG_CMD, 5, {0xEC, 0x00, 0x00, 0x00, 0x00}},
			{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
			{REGFLAG_CMD, 2, {0x6F, 0x12}},
			{REGFLAG_CMD, 5, {0xEC, 0x00, 0x00, 0x00, 0x00}},
		};

		if (bl_lvl > 3515) {
			ED_120[0] = 36;
			ED_120[1] = 44;
			ED_120[2] = 52;
			ED_90[0] = 36;
			ED_90[1] = 44;
		} else if (bl_lvl >= 1604) {
			ED_120[0] = 36;
			ED_120[1] = 44;
			ED_120[2] = 52;
			ED_90[0] = 36;
			ED_90[1] = 44;
		} else if (bl_lvl >= 1511) {
			ED_120[0] = 32;
			ED_120[1] = 40;
			ED_120[2] = 48;
			ED_90[0] = 32;
			ED_90[1] = 40;
		} else if (bl_lvl >= 1419) {
			ED_120[0] = 28;
			ED_120[1] = 36;
			ED_120[2] = 44;
			ED_90[0] = 28;
			ED_90[1] = 36;
		} else if (bl_lvl >= 1328) {
			ED_120[0] = 24;
			ED_120[1] = 32;
			ED_120[2] = 36;
			ED_90[0] = 24;
			ED_90[1] = 32;
		} else if (bl_lvl >= 1212) {
			ED_120[0] = 20;
			ED_120[1] = 28;
			ED_120[2] = 32;
			ED_90[0] = 20;
			ED_90[1] = 28;
		} else if (bl_lvl >= 1096) {
			ED_120[0] = 16;
			ED_120[1] = 24;
			ED_120[2] = 24;
			ED_90[0] = 16;
			ED_90[1] = 24;
		} else if (bl_lvl >= 950) {
			ED_120[0] = 12;
			ED_120[1] = 16;
			ED_120[2] = 16;
			ED_90[0] = 12;
			ED_90[1] = 16;
		} else if (bl_lvl >= 761) {
			ED_120[0] = 8;
			ED_120[1] = 16;
			ED_120[2] = 16;
			ED_90[0] = 8;
			ED_90[1] = 16;
		} else if (bl_lvl >= 544) {
			ED_120[0] = 8;
			ED_120[1] = 8;
			ED_120[2] = 12;
			ED_90[0] = 8;
			ED_90[1] = 8;
		} else {
			ED_120[0] = 0;
			ED_120[1] = 0;
			ED_120[2] = 0;
			ED_90[0] = 0;
			ED_90[1] = 0;
		}

		for (i = 0; i < 3; i++) {
			temp_compensation_cmd[2].para_list[i+1] = ED_120[0];
			temp_compensation_cmd[4].para_list[i+1] = ED_120[1];
			temp_compensation_cmd[6].para_list[i+1] = ED_120[2];
			temp_compensation_cmd[8].para_list[i+1] = ED_90[0];
			temp_compensation_cmd[10].para_list[i+1] = ED_90[1];
		}

		if (bl_lvl > 3515) {
			Frameskip_Vset_120[0] = 49;
			Frameskip_Vset_120[1] = 49;
			Frameskip_Vset_120[2] = 49;
			Frameskip_Vset_90[0] = 51;
			Frameskip_Vset_90[1] = 51;
			Frameskip_Vset_90[2] = 51;
		} else if (bl_lvl >= 1604) {
			Frameskip_Vset_120[0] = 26;
			Frameskip_Vset_120[1] = 26;
			Frameskip_Vset_120[2] = 36;
			Frameskip_Vset_90[0] = 28;
			Frameskip_Vset_90[1] = 28;
			Frameskip_Vset_90[2] = 37;
		} else if (bl_lvl >= 1511) {
			Frameskip_Vset_120[0] = 26;
			Frameskip_Vset_120[1] = 26;
			Frameskip_Vset_120[2] = 36;
			Frameskip_Vset_90[0] = 29;
			Frameskip_Vset_90[1] = 29;
			Frameskip_Vset_90[2] = 37;
		} else if (bl_lvl > 544) {
			Frameskip_Vset_120[0] = 27;
			Frameskip_Vset_120[1] = 27;
			Frameskip_Vset_120[2] = 36;
			Frameskip_Vset_90[0] = 29;
			Frameskip_Vset_90[1] = 29;
			Frameskip_Vset_90[2] = 37;
		} else {
			Frameskip_Vset_120[0] = 27;
			Frameskip_Vset_120[1] = 27;
			Frameskip_Vset_120[2] = 36;
			Frameskip_Vset_90[0] = 27;
			Frameskip_Vset_90[1] = 27;
			Frameskip_Vset_90[2] = 36;
		}

		for (i = 0; i < 3; i++) {
			temp_compensation_cmd[12].para_list[i+1] = Frameskip_Vset_120[i];
			temp_compensation_cmd[14].para_list[i+1] = Frameskip_Vset_90[i];
		}

		if (bl_lvl > 3515) {
			temp_compensation_cmd[20].para_list[1] = 0x00;
			temp_compensation_cmd[20].para_list[2] = 0x00;
			temp_compensation_cmd[20].para_list[3] = 0x00;
			temp_compensation_cmd[20].para_list[4] = 0x00;
		} else if (bl_lvl > 1511) {
			temp_compensation_cmd[20].para_list[1] = 0x00;
			temp_compensation_cmd[20].para_list[2] = 0x14;
			temp_compensation_cmd[20].para_list[3] = 0x14;
			temp_compensation_cmd[20].para_list[4] = 0x00;
		} else if (bl_lvl > 950) {
			temp_compensation_cmd[20].para_list[1] = 0x00;
			temp_compensation_cmd[20].para_list[2] = 0x00;
			temp_compensation_cmd[20].para_list[3] = 0x00;
			temp_compensation_cmd[20].para_list[4] = 0x00;
		} else if (bl_lvl > 761) {
			temp_compensation_cmd[20].para_list[1] = 0x00;
			temp_compensation_cmd[20].para_list[2] = 0xA7;
			temp_compensation_cmd[20].para_list[3] = 0xA7;
			temp_compensation_cmd[20].para_list[4] = 0xA7;
		} else if (bl_lvl > 544) {
			temp_compensation_cmd[20].para_list[1] = 0x00;
			temp_compensation_cmd[20].para_list[2] = 0xBC;
			temp_compensation_cmd[20].para_list[3] = 0xBC;
			temp_compensation_cmd[20].para_list[4] = 0xBC;
		} else {
			temp_compensation_cmd[20].para_list[1] = 0x15;
			temp_compensation_cmd[20].para_list[2] = 0x8D;
			temp_compensation_cmd[20].para_list[3] = 0x8D;
			temp_compensation_cmd[20].para_list[4] = 0x8D;
		}

		for (i = 0; i < sizeof(temp_compensation_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, temp_compensation_cmd[i].para_list, temp_compensation_cmd[i].count);
		}
	} else if ((lcm_id1 = 0x70) && (lcm_id2 == 3)) {
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
			{REGFLAG_CMD, 2, {0x6F, 0x03}},
			{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
			{REGFLAG_CMD, 2, {0x6F, 0x0C}},
			{REGFLAG_CMD, 4, {0xC6, 0x00, 0x00, 0x00}},
			{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
			{REGFLAG_CMD, 2, {0x6F, 0x12}},
			{REGFLAG_CMD, 5, {0xEC, 0x00, 0x00, 0x00, 0x00}},
		};

		if (bl_lvl > 3515) {
			ED_120[0] = 36;
			ED_120[1] = 40;
			ED_120[2] = 48;
			ED_90[0] = 36;
			ED_90[1] = 40;
		} else if (bl_lvl >= 1604) {
			ED_120[0] = 36;
			ED_120[1] = 40;
			ED_120[2] = 48;
			ED_90[0] = 36;
			ED_90[1] = 40;
		} else if (bl_lvl >= 1511) {
			ED_120[0] = 32;
			ED_120[1] = 36;
			ED_120[2] = 44;
			ED_90[0] = 32;
			ED_90[1] = 36;
		} else if (bl_lvl >= 1419) {
			ED_120[0] = 28;
			ED_120[1] = 32;
			ED_120[2] = 40;
			ED_90[0] = 28;
			ED_90[1] = 32;
		} else if (bl_lvl >= 1328) {
			ED_120[0] = 24;
			ED_120[1] = 28;
			ED_120[2] = 36;
			ED_90[0] = 24;
			ED_90[1] = 28;
		} else if (bl_lvl >= 1212) {
			ED_120[0] = 20;
			ED_120[1] = 24;
			ED_120[2] = 32;
			ED_90[0] = 20;
			ED_90[1] = 24;
		} else if (bl_lvl >= 1096) {
			ED_120[0] = 16;
			ED_120[1] = 20;
			ED_120[2] = 24;
			ED_90[0] = 16;
			ED_90[1] = 20;
		} else if (bl_lvl >= 950) {
			ED_120[0] = 12;
			ED_120[1] = 16;
			ED_120[2] = 16;
			ED_90[0] = 12;
			ED_90[1] = 16;
		} else if (bl_lvl >= 761) {
			ED_120[0] = 12;
			ED_120[1] = 16;
			ED_120[2] = 16;
			ED_90[0] = 12;
			ED_90[1] = 16;
		} else if (bl_lvl >= 544) {
			ED_120[0] = 8;
			ED_120[1] = 8;
			ED_120[2] = 8;
			ED_90[0] = 8;
			ED_90[1] = 12;
		} else {
			ED_120[0] = 4;
			ED_120[1] = 4;
			ED_120[2] = 4;
			ED_90[0] = 4;
			ED_90[1] = 4;
		}

		for (i = 0; i < 3; i++) {
			temp_compensation_cmd[2].para_list[i+1] = ED_120[0];
			temp_compensation_cmd[4].para_list[i+1] = ED_120[1];
			temp_compensation_cmd[6].para_list[i+1] = ED_120[2];
			temp_compensation_cmd[8].para_list[i+1] = ED_90[0];
			temp_compensation_cmd[10].para_list[i+1] = ED_90[1];
		}

		if (bl_lvl > 3515) {
			Frameskip_Vset_120[0] = 44;
			Frameskip_Vset_120[1] = 44;
			Frameskip_Vset_120[2] = 44;
			Frameskip_Vset_90[0] = 46;
			Frameskip_Vset_90[1] = 46;
			Frameskip_Vset_90[2] = 46;
		} else if (bl_lvl >= 1604) {
			Frameskip_Vset_120[0] = 26;
			Frameskip_Vset_120[1] = 26;
			Frameskip_Vset_120[2] = 35;
			Frameskip_Vset_90[0] = 28;
			Frameskip_Vset_90[1] = 28;
			Frameskip_Vset_90[2] = 37;
		} else if (bl_lvl >= 1212) {
			Frameskip_Vset_120[0] = 26;
			Frameskip_Vset_120[1] = 26;
			Frameskip_Vset_120[2] = 26;
			Frameskip_Vset_90[0] = 28;
			Frameskip_Vset_90[1] = 28;
			Frameskip_Vset_90[2] = 28;
		} else if (bl_lvl > 7) {
			Frameskip_Vset_120[0] = 26;
			Frameskip_Vset_120[1] = 26;
			Frameskip_Vset_120[2] = 26;
			Frameskip_Vset_90[0] = 28;
			Frameskip_Vset_90[1] = 28;
			Frameskip_Vset_90[2] = 28;
		}

		for (i = 0; i < 3; i++) {
			temp_compensation_cmd[12].para_list[i+1] = Frameskip_Vset_120[i];
			temp_compensation_cmd[14].para_list[i+1] = Frameskip_Vset_90[i];
		}

		if(bl_lvl > 0x220) {
			vpark_set(69, temp_compensation_cmd);
		} else {
			vpark_set(55, temp_compensation_cmd);
		}

		for (i = 0; i < sizeof(temp_compensation_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, temp_compensation_cmd[i].para_list, temp_compensation_cmd[i].count);
		}
	} else {
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
			{REGFLAG_CMD, 2, {0x6F, 0x44}},
			{REGFLAG_CMD, 2, {0xE5, 0xB2}},
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

		if (bl_lvl > 3515) {
			dbv_index = OPLUS_DISPLAY_GREATER_THAN_3515_DBV_INDEX;
		} else if (bl_lvl >= 1604) {
			dbv_index = OPLUS_DISPLAY_1604_3515_DBV_INDEX;
		} else if (bl_lvl >= 1511) {
			dbv_index = OPLUS_DISPLAY_1511_1604_DBV_INDEX;
		} else if (bl_lvl >= 1419) {
			dbv_index = OPLUS_DISPLAY_1419_1511_DBV_INDEX;
		} else if (bl_lvl >= 1328) {
			dbv_index = OPLUS_DISPLAY_1328_1419_DBV_INDEX;
		} else if (bl_lvl >= 1212) {
			dbv_index = OPLUS_DISPLAY_1212_1328_DBV_INDEX;
		} else if (bl_lvl >= 1096) {
			dbv_index = OPLUS_DISPLAY_1096_1212_DBV_INDEX;
		} else if (bl_lvl >= 950) {
			dbv_index = OPLUS_DISPLAY_950_1096_DBV_INDEX;
		} else if (bl_lvl >= 761) {
			dbv_index = OPLUS_DISPLAY_761_950_DBV_INDEX;
		} else if (bl_lvl >= 544) {
			dbv_index = OPLUS_DISPLAY_544_761_DBV_INDEX;
		} else {
			dbv_index = OPLUS_DISPLAY_LESS_THAN_544_DBV_INDEX;
		}

		if (!g_oplus_display_temp) {
			pr_debug("[DISP][DEBUG][%s:%d]panel ntc is not exist, use default ntc temp value\n", __func__, __LINE__);
			ntc_temp = 29;
		} else {
			ntc_temp = oplus_display_get_ntc_temp();
		}

		temp_index = oplus_display_get_temp_index(ntc_temp);

		for (i = 0; i < 25; i++) {
			pr_info("[DISP][INFO][%s:%d]temp_compensation_paras[%u][%u][%u]=%u\n", __func__, __LINE__,
				dbv_index, temp_index, i, temp_compensation_paras[dbv_index][temp_index][i]);
		}

		for (i = 0; i < 4; i++) {
			temp_compensation_cmd[2].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][0];
			temp_compensation_cmd[4].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][1];
			temp_compensation_cmd[6].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][2];
			temp_compensation_cmd[14].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][5];
			temp_compensation_cmd[16].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][6];
			temp_compensation_cmd[18].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][7];
			temp_compensation_cmd[27].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][17+i];
		}

		for (i = 0; i < 3; i++) {
			temp_compensation_cmd[8].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][3];
			temp_compensation_cmd[10].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][4];
			temp_compensation_cmd[20].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][8+i];
			temp_compensation_cmd[22].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][11+i];
			temp_compensation_cmd[24].para_list[i+1] = temp_compensation_paras[dbv_index][temp_index][14+i];
		}

		for (i = 0; i < sizeof(temp_compensation_cmd)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, temp_compensation_cmd[i].para_list, temp_compensation_cmd[i].count);
		}
	}

	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return 0;
}
EXPORT_SYMBOL(oplus_display_temp_compensation_set);

int oplus_display_temp_check(void *mtk_ddp_comp, void *cmdq_pkt)
{
	int ntc_temp = 0;
	int shell_temp = 0;
	int last_ntc_temp = 0;
	struct mtk_ddp_comp *output_comp = mtk_ddp_comp;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!g_oplus_display_temp || !output_comp || !cmdq_handle) {
		pr_debug("[DISP][DEBUG][%s:%d]Invalid params\n", __func__, __LINE__);
		return -EINVAL;
	}

	last_ntc_temp = g_oplus_display_temp->ntc_temp;
	ntc_temp = oplus_display_get_ntc_temp();
	shell_temp = oplus_display_get_shell_temp();

	if (oplus_display_get_temp_index(last_ntc_temp) != oplus_display_get_temp_index(ntc_temp)) {
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_SET_BL, &oplus_display_brightness);
		pr_info("[DISP][INFO][%s:%d]last_ntc_temp:%d,current_ntc_temp:%d, update temp compensation cmd\n",
			__func__, __LINE__, last_ntc_temp, ntc_temp);
	}

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
	if (oplus_adfr_is_support()) {
		oplus_adfr_temperature_detection_handle(output_comp, cmdq_handle, ntc_temp, shell_temp);
	}
#endif /* OPLUS_FEATURE_DISPLAY_ADFR */

	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return 0;
}
