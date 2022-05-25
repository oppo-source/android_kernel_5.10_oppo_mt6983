// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX350mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "imx350_eeprom.h"
#include "imx350mipiraw_Sensor.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define read_cmos_sensor(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define imx350_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)

/************************Modify Following Strings for Debug********************/
#define PFX "IMX350_camera_sensor"
#define LOG_1 LOG_INF("IMX350,MIPI 4LANE\n")
/************************   Modify end    *************************************/

#undef IMX350_24_FPS

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#define H_FOV 63
#define V_FOV 49

/*******************************************************************************
 * Proifling
 *****************************************************************************/
#define PROFILE 0
#if PROFILE
static struct timeval tv1, tv2;
static DEFINE_SPINLOCK(kdsensor_drv_lock);
/****************************************************************************
 *
 *****************************************************************************/
static void KD_SENSOR_PROFILE_INIT(struct subdrv_ctx *ctx)
{
	do_gettimeofday(&tv1);
}

/****************************************************************************
 *
 ****************************************************************************/
static void KD_SENSOR_PROFILE(struct subdrv_ctx *ctx, char *tag)
{
	unsigned long TimeIntervalUS;

	spin_lock(&kdsensor_drv_lock);

	do_gettimeofday(&tv2);
	TimeIntervalUS =
	  (tv2.tv_sec - tv1.tv_sec) * 1000000 + (tv2.tv_usec - tv1.tv_usec);
	tv1 = tv2;

	spin_unlock(&kdsensor_drv_lock);
	LOG_INF("[%s]Profile = %lu us\n", tag, TimeIntervalUS);
}
#else
static void KD_SENSOR_PROFILE_INIT(struct subdrv_ctx *ctx)
{
}

static void KD_SENSOR_PROFILE(struct subdrv_ctx *ctx, char *tag)
{
}
#endif


#define BYTE               unsigned char

/* static BOOL read_spc_flag = FALSE; */

/*support ZHDR*/
/* #define imx350_ZHDR */


static BYTE imx350_SPC_data[352] = { 0 };

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX350_SENSOR_ID,
	.checksum_value = 0xD1EFF68B,
	.pre = {		/*data rate 1099.20 Mbps/lane */
		.pclk = 420000000,	/* record different mode's pclk */
		.linelength = 6648,	/* record different mode's linelength */
		.framelength = 2104, /* record different mode's framelength */
		.startx = 0, /* record different mode's startx of grabwindow */
		.starty = 0, /* record different mode's starty of grabwindow */

		/* record different mode's width of grabwindow */
		.grabwindow_width = 2592,
		/* record different mode's height of grabwindow */
		.grabwindow_height = 1936,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * by different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 319200000,
		/*     following for GetDefaultFramerateByScenario()    */
		.max_framerate = 300,
	},
#ifdef IMX350_24_FPS
	.cap = {		/*data rate 1499.20 Mbps/lane */
		.pclk = 840000000,
		.linelength = 8704,
		.framelength = 3948,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5184,
		.grabwindow_height = 3880,
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 600000000,
		.max_framerate = 240,
	},
#else
	.cap = {		/*data rate 1499.20 Mbps/lane */
		.pclk = 823200000,
		.linelength = 6720,
		.framelength = 4080,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 5184,
		.grabwindow_height = 3880,
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 673920000,
		.max_framerate = 300,
	},
#endif
	.normal_video = {	/*data rate 1499.20 Mbps/lane */
		.pclk = 801600000,
		.linelength = 8704,
		.framelength = 3064,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 5184,
		 .grabwindow_height = 2916,
		 .mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		 .mipi_pixel_rate = 673920000,
		 .max_framerate = 300,
	},
	.hs_video = {		/*data rate 600 Mbps/lane */
		.pclk = 595000000,
		.linelength = 6024,
		.framelength = 828,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 201600000,
		.max_framerate = 1200,
	},
	.margin = 18,		/* sensor framelength & shutter margin */
	.min_shutter = 8,	/* min shutter */
	.min_gain = BASEGAIN, /*1x gain*/
	.max_gain = BASEGAIN * 16, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 0,

	/* max framelength by sensor register's limitation */
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	/* shutter delay frame for AE cycle,
	 * 2 frame with ispGain_delay-shut_delay=2-0=2
	 */
	.ae_sensor_gain_delay_frame = 0,

	/* sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 1,	/* 1, support; 0,not support */
	.sensor_mode_num = 4,	/* support sensor mode num */
	.frame_time_delay_frame = 3,
	.cap_delay_frame = 1,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 1,	/* enter video delay frame num */
	.hs_video_delay_frame = 3, /* enter high speed video  delay frame num */

	.isp_driving_current = ISP_DRIVING_8MA,	/* mclk driving current */

	/* sensor_interface_type */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */

	/* sensor output first pixel color */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.mclk = 24,	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x34, 0x20, 0xff},
/* record sensor support all write id addr, only supprt 4must end with 0xff */
	.i2c_speed = 400,	/* i2c read/write speed */
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{5184, 3880, 0, 0, 5184, 3872, 2592, 1936,
	0000, 0000, 2592, 1936, 0, 0, 2592, 1936},	/* Preview */
	{5184, 3880, 0, 0, 5184, 3880, 5184, 3880,
	0000, 0000, 5184, 3880, 0, 0, 5184, 3880},	/* capture */
	{5184, 3880, 0, 480, 5184, 2916, 5184, 2916,
	0000, 0000, 5184, 2916, 0, 0, 5184, 2916},	/* video */
	{5184, 3880, 672, 856, 3840, 2160, 1280, 720,
	00, 0000, 1280, 720, 0, 0, 1280, 720},
};

/*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X36), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0A20, 0x0790, 0x01, 0x00, 0x0780, 0x0001,
	 0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
	/* Capture mode setting */
	{0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1440, 0x0F28, 0x01, 0x00, 0x1440, 0x0001,
	 0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1440, 0x0B64, 0x01, 0x00, 0x1440, 0x0001,
	 0x02, 0x00, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000}
};

#define IMX350MIPI_MaxGainIndex (389)
kal_uint16 imx350MIPI_sensorGainMapping[IMX350MIPI_MaxGainIndex][2] = {
	{64, 0},
	{65, 16},
	{66, 32},
	{67, 46},
	{68, 61},
	{69, 75},
	{70, 88},
	{71, 101},
	{72, 114},
	{73, 127},
	{74, 139},
	{75, 151},
	{76, 162},
	{77, 173},
	{78, 184},
	{79, 195},
	{80, 205},
	{81, 215},
	{82, 225},
	{83, 235},
	{84, 244},
	{85, 253},
	{86, 262},
	{87, 271},
	{88, 280},
	{89, 288},
	{90, 296},
	{91, 304},
	{92, 312},
	{93, 320},
	{94, 327},
	{95, 335},
	{96, 342},
	{97, 349},
	{98, 356},
	{99, 363},
	{100, 369},
	{101, 376},
	{102, 382},
	{103, 388},
	{104, 394},
	{105, 400},
	{106, 406},
	{107, 412},
	{108, 418},
	{109, 423},
	{110, 429},
	{111, 434},
	{112, 439},
	{113, 445},
	{114, 450},
	{115, 455},
	{116, 460},
	{117, 464},
	{118, 469},
	{119, 474},
	{120, 478},
	{121, 483},
	{122, 487},
	{123, 492},
	{124, 496},
	{125, 500},
	{126, 504},
	{127, 508},
	{128, 512},
	{129, 516},
	{130, 520},
	{131, 524},
	{132, 528},
	{133, 532},
	{134, 535},
	{135, 539},
	{136, 543},
	{137, 546},
	{138, 550},
	{139, 553},
	{140, 556},
	{141, 560},
	{142, 563},
	{143, 566},
	{144, 569},
	{145, 573},
	{146, 576},
	{147, 579},
	{148, 582},
	{149, 585},
	{150, 588},
	{151, 590},
	{152, 593},
	{153, 596},
	{154, 599},
	{155, 602},
	{156, 604},
	{157, 607},
	{158, 610},
	{159, 612},
	{160, 615},
	{161, 617},
	{162, 620},
	{163, 622},
	{164, 625},
	{165, 627},
	{165, 628},
	{166, 630},
	{167, 632},
	{168, 634},
	{169, 637},
	{170, 639},
	{171, 641},
	{172, 643},
	{173, 646},
	{174, 648},
	{175, 650},
	{176, 652},
	{177, 654},
	{178, 656},
	{179, 658},
	{180, 660},
	{181, 662},
	{182, 664},
	{183, 666},
	{184, 668},
	{185, 670},
	{186, 672},
	{187, 674},
	{188, 676},
	{189, 678},
	{190, 680},
	{191, 681},
	{192, 683},
	{193, 685},
	{194, 687},
	{195, 688},
	{196, 690},
	{197, 692},
	{198, 694},
	{199, 695},
	{200, 697},
	{201, 698},
	{201, 699},
	{202, 700},
	{203, 702},
	{204, 703},
	{205, 705},
	{206, 706},
	{207, 708},
	{208, 709},
	{209, 711},
	{210, 712},
	{211, 714},
	{212, 715},
	{213, 717},
	{214, 718},
	{215, 720},
	{216, 721},
	{217, 722},
	{218, 724},
	{219, 725},
	{220, 727},
	{221, 728},
	{222, 729},
	{223, 731},
	{224, 732},
	{225, 733},
	{226, 735},
	{227, 736},
	{228, 737},
	{229, 738},
	{229, 739},
	{230, 740},
	{231, 741},
	{232, 742},
	{233, 743},
	{234, 744},
	{234, 745},
	{235, 746},
	{236, 747},
	{237, 748},
	{238, 749},
	{239, 750},
	{240, 751},
	{241, 753},
	{242, 754},
	{243, 755},
	{244, 756},
	{245, 757},
	{246, 758},
	{247, 759},
	{248, 760},
	{249, 761},
	{250, 762},
	{251, 763},
	{252, 764},
	{253, 765},
	{254, 766},
	{255, 767},
	{256, 768},
	{257, 769},
	{258, 770},
	{259, 771},
	{260, 772},
	{261, 773},
	{262, 774},
	{263, 775},
	{264, 776},
	{265, 777},
	{266, 778},
	{267, 779},
	{268, 780},
	{269, 781},
	{270, 782},
	{271, 783},
	{273, 784},
	{274, 785},
	{275, 786},
	{276, 787},
	{277, 788},
	{278, 789},
	{280, 790},
	{281, 791},
	{282, 792},
	{283, 793},
	{284, 794},
	{286, 795},
	{287, 796},
	{288, 797},
	{289, 798},
	{291, 799},
	{292, 800},
	{293, 801},
	{295, 802},
	{296, 803},
	{297, 804},
	{299, 805},
	{300, 806},
	{302, 807},
	{303, 808},
	{304, 809},
	{306, 810},
	{307, 811},
	{309, 812},
	{310, 813},
	{312, 814},
	{313, 815},
	{315, 816},
	{316, 817},
	{318, 818},
	{319, 819},
	{321, 820},
	{322, 821},
	{324, 822},
	{326, 823},
	{327, 824},
	{329, 825},
	{330, 826},
	{332, 827},
	{334, 828},
	{336, 829},
	{337, 830},
	{339, 831},
	{341, 832},
	{343, 833},
	{344, 834},
	{346, 835},
	{348, 836},
	{350, 837},
	{352, 838},
	{354, 839},
	{356, 840},
	{358, 841},
	{360, 842},
	{362, 843},
	{364, 844},
	{366, 845},
	{368, 846},
	{370, 847},
	{372, 848},
	{374, 849},
	{376, 850},
	{378, 851},
	{381, 852},
	{383, 853},
	{385, 854},
	{387, 855},
	{390, 856},
	{392, 857},
	{394, 858},
	{397, 859},
	{399, 860},
	{402, 861},
	{404, 862},
	{407, 863},
	{409, 864},
	{412, 865},
	{414, 866},
	{417, 867},
	{420, 868},
	{422, 869},
	{425, 870},
	{428, 871},
	{431, 872},
	{434, 873},
	{436, 874},
	{439, 875},
	{442, 876},
	{445, 877},
	{448, 878},
	{451, 879},
	{455, 880},
	{458, 881},
	{461, 882},
	{464, 883},
	{468, 884},
	{471, 885},
	{474, 886},
	{478, 887},
	{481, 888},
	{485, 889},
	{489, 890},
	{492, 891},
	{496, 892},
	{500, 893},
	{504, 894},
	{508, 895},
	{512, 896},
	{516, 897},
	{520, 898},
	{524, 899},
	{528, 900},
	{532, 901},
	{537, 902},
	{541, 903},
	{546, 904},
	{550, 905},
	{555, 906},
	{560, 907},
	{564, 908},
	{569, 909},
	{574, 910},
	{579, 911},
	{585, 912},
	{590, 913},
	{595, 914},
	{601, 915},
	{606, 916},
	{612, 917},
	{618, 918},
	{624, 919},
	{630, 920},
	{636, 921},
	{642, 922},
	{648, 923},
	{655, 924},
	{661, 925},
	{668, 926},
	{675, 927},
	{682, 928},
	{689, 929},
	{697, 930},
	{704, 931},
	{712, 932},
	{720, 933},
	{728, 934},
	{736, 935},
	{744, 936},
	{753, 937},
	{762, 938},
	{771, 939},
	{780, 940},
	{789, 941},
	{799, 942},
	{809, 943},
	{819, 944},
	{829, 945},
	{840, 946},
	{851, 947},
	{862, 948},
	{873, 949},
	{885, 950},
	{897, 951},
	{910, 952},
	{923, 953},
	{936, 954},
	{949, 955},
	{963, 956},
	{978, 957},
	{992, 958},
	{1008, 959},
	{1024, 960},
};


static MUINT32 cur_startpos;
static MUINT32 cur_size;
static void imx350_set_pd_focus_area(struct subdrv_ctx *ctx, MUINT32 startpos, MUINT32 size)
{
	UINT16 start_x_pos, start_y_pos, end_x_pos, end_y_pos;
	UINT16 focus_width, focus_height;

	if ((cur_startpos == startpos) && (cur_size == size)) {
		LOG_INF("Not to need update focus area!\n");
		return;
	}
	cur_startpos = startpos;
	cur_size = size;

	start_x_pos = (startpos >> 16) & 0xFFFF;
	start_y_pos = startpos & 0xFFFF;
	focus_width = (size >> 16) & 0xFFFF;
	focus_height = size & 0xFFFF;

	end_x_pos = start_x_pos + focus_width;
	end_y_pos = start_y_pos + focus_height;

	if (ctx->pdaf_mode == 1) {
		LOG_INF("GC pre PDAF\n");
		/*PDAF*/
		/*PD_OUT_EN=1 */
		write_cmos_sensor(ctx, 0x3E37, 0x01);

		/*AREA MODE */
		write_cmos_sensor(ctx, 0x38A3, 0x01);	 /* 8x6 output */

		/*Fixed area mode */
		write_cmos_sensor(ctx, 0x38A4, (start_x_pos >> 8) & 0xFF);
		write_cmos_sensor(ctx, 0x38A5, start_x_pos & 0xFF);	/* X start */
		write_cmos_sensor(ctx, 0x38A6, (start_y_pos >> 8) & 0xFF);
		write_cmos_sensor(ctx, 0x38A7, start_y_pos & 0xFF);	/* Y start */
		write_cmos_sensor(ctx, 0x38A8, (end_x_pos >> 8) & 0xFF);
		write_cmos_sensor(ctx, 0x38A9, end_x_pos & 0xFF);	/* X end */
		write_cmos_sensor(ctx, 0x38AA, (end_y_pos >> 8) & 0xFF);
		write_cmos_sensor(ctx, 0x38AB, end_y_pos & 0xFF);	/* Y end */
	}

	LOG_INF(
		"start_x:%d, start_y:%d, width:%d, height:%d, end_x:%d, end_y:%d\n",
	     start_x_pos, start_y_pos, focus_width,
	     focus_height, end_x_pos, end_y_pos);
}


static void imx350_get_pdaf_reg_setting(struct subdrv_ctx *ctx,
	MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor(ctx, regDa[idx]);
		/* LOG_INF("%x %x", regDa[idx], regDa[idx+1]); */
	}
}

static void imx350_set_pdaf_reg_setting(struct subdrv_ctx *ctx,
	MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		write_cmos_sensor(ctx, regDa[idx], regDa[idx + 1]);
		/* LOG_INF("%x %x", regDa[idx], regDa[idx+1]); */
	}
}

static void imx350_apply_SPC(struct subdrv_ctx *ctx)
{

	LOG_INF("E");

	imx350_read_SPC(ctx, imx350_SPC_data);

	subdrv_i2c_wr_p8(ctx, 0x7500, imx350_SPC_data, 352);
}

static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_INF("frame_length = %d, line_length = %d\n",
				ctx->frame_length,
				ctx->line_length);

	write_cmos_sensor(ctx, 0x0104, 0x01);

	write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor(ctx, 0x0342, ctx->line_length >> 8);
	write_cmos_sensor(ctx, 0x0343, ctx->line_length & 0xFF);

	write_cmos_sensor(ctx, 0x0104, 0x00);
}				/*    set_dummy  */

static kal_uint32 return_lot_id_from_otp(struct subdrv_ctx *ctx)
{
	kal_uint16 val = 0;
	int i = 0;

	if (write_cmos_sensor(ctx, 0x0a02, 0x27) < 0) {
		LOG_INF("read otp fail Err!\n");
		return 0;
	}
	write_cmos_sensor(ctx, 0x0a00, 0x01);

	for (i = 0; i < 3; i++) {
		val = read_cmos_sensor(ctx, 0x0A01);
		if ((val & 0x01) == 0x01)
			break;
		mDELAY(3);
	}
	if (i == 3) {
		LOG_INF("read otp fail Err!\n");	/* print log */
		return 0;
	}
	/* LOG_INF("0x0A38 0x%x 0x0A39 0x%x\n",
	 * read_cmos_sensor(ctx, 0x0A38)<<4,read_cmos_sensor(ctx, 0x0A39)>>4);
	 */
	return ((read_cmos_sensor(ctx, 0x0A22) << 4) |
					read_cmos_sensor(ctx, 0x0A23) >> 4);
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = ctx->frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable %d\n",
			framerate,
			min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	ctx->frame_length =
	    (frame_length > ctx->min_frame_length)
	    ? frame_length : ctx->min_frame_length;

	ctx->dummy_line =
		ctx->frame_length - ctx->min_frame_length;
	/* dummy_line = frame_length - ctx->min_frame_length; */
	/* if (dummy_line < 0) */
	/* ctx->dummy_line = 0; */
	/* else */
	/* ctx->dummy_line = dummy_line; */
	/* ctx->frame_length = frame_length + ctx->dummy_line; */
	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	set_dummy(ctx);
}				/*    set_max_framerate  */

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
#define MAX_CIT_LSHIFT 7
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{	kal_uint16 realtime_fps = 0;
	kal_uint16 l_shift = 1;
	/* LOG_INF("Enter! shutter =%d, framelength =%d\n",
	 * shutter,ctx->frame_length);
	 */
	ctx->shutter = shutter;
	/* write_shutter(shutter); */
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
/* if shutter bigger than frame_length, should extend frame length first */
	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter : shutter;

if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
	/* long expsoure */
	for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
		if ((shutter >> l_shift) <
		  (imgsensor_info.max_frame_length - imgsensor_info.margin))
			break;
	}
	if (l_shift > MAX_CIT_LSHIFT) {
		LOG_INF("Unable to set such a long exposure %d, set to max\n",
			shutter);
		l_shift = MAX_CIT_LSHIFT;
	}
	shutter = shutter >> l_shift;
	/* imgsensor_info.max_frame_length; */
	ctx->frame_length = shutter + imgsensor_info.margin;

	/* LOG_INF("0x3028 0x%x l_shift %d l_shift&0x3 %d\n",
	 * read_cmos_sensor(ctx, 0x3028),l_shift,l_shift&0x7);
	 */
	write_cmos_sensor(ctx, 0x3028, read_cmos_sensor(ctx, 0x3028) | (l_shift & 0x7));
	/* LOG_INF("0x3028 0x%x\n", read_cmos_sensor(ctx, 0x3028)); */

} else {
	write_cmos_sensor(ctx, 0x3028, read_cmos_sensor(ctx, 0x3028) & 0xf8);
}

	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk /
			ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 237 && realtime_fps <= 243)
			set_max_framerate(ctx, 236, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(ctx, 0x0104, 0x01);
			write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);

			write_cmos_sensor(ctx, 0x0341,
				ctx->frame_length & 0xFF);

			write_cmos_sensor(ctx, 0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(ctx, 0x0104, 0x01);
		write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor(ctx, 0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor(ctx, 0x0104, 0x01);
	write_cmos_sensor(ctx, 0x0350, 0x01); /* enable auto extend */
	write_cmos_sensor(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0203, shutter & 0xFF);
	write_cmos_sensor(ctx, 0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n",
		shutter, ctx->frame_length);
} /*    set_shutter */

static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
	write_cmos_sensor(ctx, 0x0104, 0x01);
	write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor(ctx, 0x0104, 0x00);

	LOG_INF("Framelength: set=%d/input=%d/min=%d, auto_extend=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length,
		read_cmos_sensor(ctx, 0x0350));
}

static void set_shutter_frame_length(struct subdrv_ctx *ctx,
			kal_uint16 shutter, kal_uint16 frame_length,
			kal_bool auto_extend_en)
{	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	ctx->shutter = shutter;
	/* LOG_INF("shutter =%d, frame_time =%d\n", shutter, frame_time); */

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
/* if shutter bigger than frame_length, should extend frame length first */
	/*Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;
	ctx->frame_length = ctx->frame_length + dummy_line;

	/*  */
	if (shutter > ctx->frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk /
			ctx->line_length * 10 / ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(ctx, 0x0104, 0x01);
			write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
			write_cmos_sensor(ctx, 0x0341,
				ctx->frame_length & 0xFF);
			write_cmos_sensor(ctx, 0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(ctx, 0x0104, 0x01);
		write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor(ctx, 0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor(ctx, 0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor(ctx, 0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor(ctx, 0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0203, shutter & 0xFF);
	write_cmos_sensor(ctx, 0x0104, 0x00);

	LOG_INF(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter,
		ctx->frame_length, frame_length,
		dummy_line, read_cmos_sensor(ctx, 0x0350));
}			/* set_shutter_frame_length */

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint16 gain)
{
	kal_uint16 iI;

	for (iI = 0; iI < IMX350MIPI_MaxGainIndex; iI++) {
		if (gain <= imx350MIPI_sensorGainMapping[iI][0] * 16) //basegain 1x = 1024
			return imx350MIPI_sensorGainMapping[iI][1];
	}

	return imx350MIPI_sensorGainMapping[iI - 1][1];
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */

	/*  */
	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		LOG_INF("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}


	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(ctx, 0x0104, 0x01);
	/* Global analog Gain for Long expo */
	write_cmos_sensor(ctx, 0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0205, reg_gain & 0xFF);
	/* Global analog Gain for Short expo */
	write_cmos_sensor(ctx, 0x0216, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0217, reg_gain & 0xFF);
	write_cmos_sensor(ctx, 0x0104, 0x00);

	return gain;
}				/*    set_gain  */

/*************************************************************************
 * FUNCTION
 *    set_dual_gain
 *
 * DESCRIPTION
 *    This function is to set dual gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor dual gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_dual_gain(struct subdrv_ctx *ctx, kal_uint16 gain1, kal_uint16 gain2)
{
	kal_uint16 reg_gain1, reg_gain2;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */


	if (gain1 < imgsensor_info.min_gain ||
		gain1 > imgsensor_info.max_gain) {
		LOG_INF("Error gain setting");

		if (gain1 < imgsensor_info.min_gain)
			gain1 = imgsensor_info.min_gain;
		else
			gain1 = imgsensor_info.max_gain;
	}


	if (gain2 < BASEGAIN || gain2 > 8 * BASEGAIN) {
		LOG_INF("Error gain2 setting");

		if (gain2 < BASEGAIN)
			gain2 = BASEGAIN;
		else if (gain2 > 8 * BASEGAIN)
			gain2 = 8 * BASEGAIN;
	}

	reg_gain1 = gain2reg(ctx, gain1);
	reg_gain2 = gain2reg(ctx, gain2);

	ctx->gain = reg_gain1;
	LOG_INF(
		"gain1 = %d, reg_gain1 = 0x%x, gain2 = %d, reg_gain2 = 0x%x\n",
				gain1, reg_gain1, gain2, reg_gain2);

	write_cmos_sensor(ctx, 0x0104, 0x01);
	/* Global analog Gain for Long expo */
	write_cmos_sensor(ctx, 0x0204, (reg_gain1 >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0205, reg_gain1 & 0xFF);
	/* Global analog Gain for Short expo */
	write_cmos_sensor(ctx, 0x0216, (reg_gain2 >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0217, reg_gain2 & 0xFF);
	write_cmos_sensor(ctx, 0x0104, 0x00);

	return gain1;
}				/*    set_dual_gain  */

static void hdr_write_shutter(struct subdrv_ctx *ctx, kal_uint16 le, kal_uint16 se, kal_uint16 lv)
{
	kal_uint16 realtime_fps = 0;
	kal_uint16 ratio;

	LOG_INF("le:0x%x, se:0x%x\n", le, se);
	if (le > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = le + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk /
			ctx->line_length * 10 / ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			write_cmos_sensor(ctx, 0x0104, 0x01);
			write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
			write_cmos_sensor(ctx, 0x0341,
				ctx->frame_length & 0xFF);
			write_cmos_sensor(ctx, 0x0104, 0x00);
		}
	} else {
		write_cmos_sensor(ctx, 0x0104, 0x01);
		write_cmos_sensor(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor(ctx, 0x0104, 0x00);
	}
	write_cmos_sensor(ctx, 0x0104, 0x01);

	/* Long exposure */
	write_cmos_sensor(ctx, 0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0203, le & 0xFF);
	/* Short exposure */
	write_cmos_sensor(ctx, 0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0225, se & 0xFF);
	write_cmos_sensor(ctx, 0x0104, 0x00);

	/* Ratio */
	if (se == 0)
		ratio = 2;
	else {
		ratio = (le + (se >> 1)) / se;
		if (ratio > 16)
			ratio = 2;
	}

	LOG_INF("le:%d, se:%d, ratio:%d\n", le, se, ratio);
	write_cmos_sensor(ctx, 0x0222, ratio);
}

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif

kal_uint16 addr_data_pair_imx350_zvhdr_on[] = {
	0x30b1, 0x00,
	0x30c6, 0x00,
	0x30b2, 0x00,
	0x30b3, 0x00,
	0x30c7, 0x00,
	0x30b4, 0x01,
	0x30b5, 0x01,
	0x30b6, 0x01,
	0x30b7, 0x01,
	0x30b8, 0x01,
	0x30b9, 0x01,
	0x30ba, 0x01,
	0x30bb, 0x01,
	0x30bc, 0x01,
};

kal_uint16 addr_data_pair_imx350_zvhdr_off[] = {
	0x30b4, 0x00,
	0x30b5, 0x00,
	0x30b6, 0x00,
	0x30b7, 0x00,
	0x30b8, 0x00,
	0x30b9, 0x00,
	0x30ba, 0x00,
	0x30bb, 0x00,
	0x30bc, 0x00,
};

static kal_uint16 zvhdr_setting(struct subdrv_ctx *ctx)
{

	LOG_INF("zhdr(mode:%d)\n", ctx->hdr_mode);

	if (ctx->hdr_mode == 9) {
		imx350_table_write_cmos_sensor(ctx, addr_data_pair_imx350_zvhdr_on,
		sizeof(addr_data_pair_imx350_zvhdr_on) / sizeof(kal_uint16));
	} else {
		imx350_table_write_cmos_sensor(ctx, addr_data_pair_imx350_zvhdr_off,
		sizeof(addr_data_pair_imx350_zvhdr_off) / sizeof(kal_uint16));
	}
	return 0;

}

kal_uint16 addr_data_pair_init_imx350[] = {
	0x0136, 0x18,
	0x0137, 0x00,

	0x3C7D, 0x28,
	0x3C7E, 0x01,
	0x3C7F, 0x0C,

	0x3C07, 0x00,
	0x3C08, 0x00,
	0x3C09, 0x01,
	0x3F7F, 0x01,
	0x4430, 0x05,
	0x4431, 0xDC,
	0x5222, 0x02,
	0x56B7, 0x74,
	0x6204, 0xC6,
	0x620E, 0x27,
	0x6210, 0x69,
	0x6211, 0xD6,
	0x6213, 0x01,
	0x6215, 0x5A,
	0x6216, 0x75,
	0x6218, 0x5A,
	0x6219, 0x75,
	0x6220, 0x06,
	0x6222, 0x0C,
	0x6225, 0x19,
	0x6228, 0x32,
	0x6229, 0x70,
	0x622B, 0x64,
	0x622E, 0xB0,
	0x6231, 0x71,
	0x6234, 0x06,
	0x6236, 0x46,
	0x6237, 0x46,
	0x6239, 0x0C,
	0x623C, 0x19,
	0x623F, 0x32,
	0x6240, 0x71,
	0x6242, 0x64,
	0x6243, 0x44,
	0x6245, 0xB0,
	0x6246, 0xA8,
	0x6248, 0x71,
	0x624B, 0x06,
	0x624D, 0x46,
	0x625C, 0xC9,
	0x625F, 0x92,
	0x6262, 0x26,
	0x6264, 0x46,
	0x6265, 0x46,
	0x6267, 0x0C,
	0x626A, 0x19,
	0x626D, 0x32,
	0x626E, 0x72,
	0x6270, 0x64,
	0x6271, 0x68,
	0x6273, 0xC8,
	0x6276, 0x91,
	0x6279, 0x27,
	0x627B, 0x46,
	0x627C, 0x55,
	0x627F, 0x95,
	0x6282, 0x84,
	0x6283, 0x40,
	0x6284, 0x00,
	0x6285, 0x00,
	0x6286, 0x08,
	0x6287, 0xC0,
	0x6288, 0x00,
	0x6289, 0x00,
	0x628A, 0x1B,
	0x628B, 0x80,
	0x628C, 0x20,
	0x628E, 0x35,
	0x628F, 0x00,
	0x6290, 0x50,
	0x6291, 0x00,
	0x6292, 0x14,
	0x6293, 0x00,
	0x6294, 0x00,
	0x6296, 0x54,
	0x6297, 0x00,
	0x6298, 0x00,
	0x6299, 0x01,
	0x629A, 0x10,
	0x629B, 0x01,
	0x629C, 0x00,
	0x629D, 0x03,
	0x629E, 0x50,
	0x629F, 0x05,
	0x62A0, 0x00,
	0x62B1, 0x00,
	0x62B2, 0x00,
	0x62B3, 0x00,
	0x62B5, 0x00,
	0x62B6, 0x00,
	0x62B7, 0x00,
	0x62B8, 0x00,
	0x62B9, 0x00,
	0x62BA, 0x00,
	0x62BB, 0x00,
	0x62BC, 0x00,
	0x62BD, 0x00,
	0x62BE, 0x00,
	0x62BF, 0x00,
	0x62D0, 0x0C,
	0x62D1, 0x00,
	0x62D2, 0x00,
	0x62D4, 0x40,
	0x62D5, 0x00,
	0x62D6, 0x00,
	0x62D7, 0x00,
	0x62D8, 0xD8,
	0x62D9, 0x00,
	0x62DA, 0x00,
	0x62DB, 0x02,
	0x62DC, 0xB0,
	0x62DD, 0x03,
	0x62DE, 0x00,
	0x62EF, 0x14,
	0x62F0, 0x00,
	0x62F1, 0x00,
	0x62F3, 0x58,
	0x62F4, 0x00,
	0x62F5, 0x00,
	0x62F6, 0x01,
	0x62F7, 0x20,
	0x62F8, 0x00,
	0x62F9, 0x00,
	0x62FA, 0x03,
	0x62FB, 0x80,
	0x62FC, 0x00,
	0x62FD, 0x00,
	0x62FE, 0x04,
	0x62FF, 0x60,
	0x6300, 0x04,
	0x6301, 0x00,
	0x6302, 0x09,
	0x6303, 0x00,
	0x6304, 0x0C,
	0x6305, 0x00,
	0x6306, 0x1B,
	0x6307, 0x80,
	0x6308, 0x30,
	0x630A, 0x38,
	0x630B, 0x00,
	0x630C, 0x60,
	0x630E, 0x14,
	0x630F, 0x00,
	0x6310, 0x00,
	0x6312, 0x58,
	0x6313, 0x00,
	0x6314, 0x00,
	0x6315, 0x01,
	0x6316, 0x18,
	0x6317, 0x01,
	0x6318, 0x80,
	0x6319, 0x03,
	0x631A, 0x60,
	0x631B, 0x06,
	0x631C, 0x00,
	0x632D, 0x0E,
	0x632E, 0x00,
	0x632F, 0x00,
	0x6331, 0x44,
	0x6332, 0x00,
	0x6333, 0x00,
	0x6334, 0x00,
	0x6335, 0xE8,
	0x6336, 0x00,
	0x6337, 0x00,
	0x6338, 0x02,
	0x6339, 0xF0,
	0x633A, 0x00,
	0x633B, 0x00,
	0x634C, 0x0C,
	0x634D, 0x00,
	0x634E, 0x00,
	0x6350, 0x40,
	0x6351, 0x00,
	0x6352, 0x00,
	0x6353, 0x00,
	0x6354, 0xD8,
	0x6355, 0x00,
	0x6356, 0x00,
	0x6357, 0x02,
	0x6358, 0xB0,
	0x6359, 0x04,
	0x635A, 0x00,
	0x636B, 0x00,
	0x636C, 0x00,
	0x636D, 0x00,
	0x636F, 0x00,
	0x6370, 0x00,
	0x6371, 0x00,
	0x6372, 0x00,
	0x6373, 0x00,
	0x6374, 0x00,
	0x6375, 0x00,
	0x6376, 0x00,
	0x6377, 0x00,
	0x6378, 0x00,
	0x6379, 0x00,
	0x637A, 0x13,
	0x637B, 0xD4,
	0x6388, 0x22,
	0x6389, 0x82,
	0x638A, 0xC8,
	0x7BA0, 0x01,
	0x7BA9, 0x00,
	0x7BAA, 0x01,
	0x7BAD, 0x00,
	0x9002, 0x00,
	0x9003, 0x00,
	0x9004, 0x0C,
	0x9006, 0x01,
	0x9200, 0x93,
	0x9201, 0x85,
	0x9202, 0x93,
	0x9203, 0x87,
	0x9204, 0x93,
	0x9205, 0x8D,
	0x9206, 0x93,
	0x9207, 0x8F,
	0x9208, 0x62,
	0x9209, 0x2C,
	0x920A, 0x62,
	0x920B, 0x2F,
	0x920C, 0x6A,
	0x920D, 0x23,
	0x920E, 0x71,
	0x920F, 0x08,
	0x9210, 0x71,
	0x9211, 0x09,
	0x9212, 0x71,
	0x9213, 0x0B,
	0x9214, 0x6A,
	0x9215, 0x0F,
	0x9216, 0x71,
	0x9217, 0x07,
	0x935D, 0x01,
	0x9389, 0x05,
	0x938B, 0x05,
	0x9391, 0x05,
	0x9393, 0x05,
	0x9395, 0x65,
	0x9397, 0x5A,
	0x9399, 0x05,
	0x939B, 0x05,
	0x939D, 0x05,
	0x939F, 0x05,
	0x93A1, 0x05,
	0x93A3, 0x05,
	0xB3F2, 0x0E,
	0xBC40, 0x03,
	0xBC82, 0x07,
	0xBC83, 0xB0,
	0xBC84, 0x0D,
	0xBC85, 0x08,
	0xE0A6, 0x0A,

	0x7B80, 0x00,
	0x7B81, 0x00,
	0x8D1F, 0x00,
	0x8D27, 0x00,
	0x97C5, 0x14,
	0x9963, 0x64,
	0x9964, 0x50,
	0x9A00, 0x0C,
	0x9A01, 0x0C,
	0x9A06, 0x0C,
	0x9A18, 0x0C,
	0x9A19, 0x0C,
	0xA900, 0x20,
	0xA901, 0x20,
	0xA902, 0x20,
	0xA903, 0x15,
	0xA904, 0x15,
	0xA905, 0x15,
	0xA906, 0x20,
	0xA907, 0x20,
	0xA908, 0x20,
	0xA909, 0x15,
	0xA90A, 0x15,
	0xA90B, 0x15,
	0xA915, 0x3F,
	0xA916, 0x3F,
	0xA917, 0x3F,
	0xA91F, 0x04,
	0xA921, 0x03,
	0xA923, 0x02,
	0xA93D, 0x05,
	0xA93F, 0x03,
	0xA941, 0x02,
	0xA949, 0x03,
	0xA94B, 0x03,
	0xA94D, 0x03,
	0xA94F, 0x06,
	0xA951, 0x06,
	0xA953, 0x06,
	0xA955, 0x03,
	0xA957, 0x03,
	0xA959, 0x03,
	0xA95B, 0x06,
	0xA95D, 0x06,
	0xA95F, 0x06,
	0xA98B, 0x1F,
	0xA98D, 0x1F,
	0xA98F, 0x1F,
	0xA9AF, 0x04,
	0xA9B1, 0x03,
	0xA9B3, 0x02,
	0xA9CD, 0x05,
	0xA9CF, 0x03,
	0xA9D1, 0x02,
	0xAA20, 0x3F,
	0xAA21, 0x20,
	0xAA22, 0x20,
	0xAA23, 0x3F,
	0xAA24, 0x15,
	0xAA25, 0x15,
	0xAA26, 0x20,
	0xAA27, 0x20,
	0xAA28, 0x20,
	0xAA29, 0x15,
	0xAA2A, 0x15,
	0xAA2B, 0x15,
	0xAA32, 0x3F,
	0xAA35, 0x3F,
	0xAA36, 0x3F,
	0xAA37, 0x3F,
	0xAA3F, 0x04,
	0xAA41, 0x03,
	0xAA43, 0x02,
	0xAA5D, 0x05,
	0xAA5F, 0x03,
	0xAA61, 0x02,
	0xAA69, 0x3F,
	0xAA6B, 0x03,
	0xAA6D, 0x03,
	0xAA6F, 0x3F,
	0xAA71, 0x06,
	0xAA73, 0x06,
	0xAA75, 0x03,
	0xAA77, 0x03,
	0xAA79, 0x03,
	0xAA7B, 0x06,
	0xAA7D, 0x06,
	0xAA7F, 0x06,
	0xAAAB, 0x1F,
	0xAAAD, 0x1F,
	0xAAAF, 0x1F,
	0xAAB0, 0x20,
	0xAAB1, 0x20,
	0xAAB2, 0x20,
	0xAAC2, 0x3F,
	0xAACF, 0x04,
	0xAAD1, 0x03,
	0xAAD3, 0x02,
	0xAAED, 0x05,
	0xAAEF, 0x03,
	0xAAF1, 0x02,
	0xAB53, 0x20,
	0xAB54, 0x20,
	0xAB55, 0x20,
	0xAB57, 0x40,
	0xAB59, 0x40,
	0xAB5B, 0x40,
	0xAB63, 0x03,
	0xAB65, 0x03,
	0xAB67, 0x03,
	0xAB87, 0x04,
	0xAB89, 0x03,
	0xAB8B, 0x02,
	0xABA5, 0x05,
	0xABA7, 0x03,
	0xABA9, 0x02,
	0xABB7, 0x04,
	0xABB9, 0x03,
	0xABBB, 0x02,
	0xABD5, 0x05,
	0xABD7, 0x03,
	0xABD9, 0x02,
	0xAC01, 0x0A,
	0xAC03, 0x0A,
	0xAC05, 0x0A,
	0xAC06, 0x01,
	0xAC07, 0xC0,
	0xAC09, 0xC0,
	0xAC17, 0x0A,
	0xAC19, 0x0A,
	0xAC1B, 0x0A,
	0xAC1C, 0x01,
	0xAC1D, 0xC0,
	0xAC1F, 0xC0,
	0xB6D9, 0x00,
};

static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx350_table_write_cmos_sensor(ctx, addr_data_pair_init_imx350,
		sizeof(addr_data_pair_init_imx350)/sizeof(kal_uint16));

	/*enable temperature sensor, TEMP_SEN_CTL:*/
	write_cmos_sensor(ctx, 0x0138, 0x01);
}    /*    sensor_init  */

kal_uint16 addr_data_pair_preview_imx350[] = {
	0x0100, 0x00,

	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,

	0x0342, 0x19,
	0x0343, 0xF8,

	0x0340, 0x08,
	0x0341, 0x38,

	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x14,
	0x0349, 0x3F,
	0x034A, 0x0F,
	0x034B, 0x1F,

	0x0220, 0x00,
	0x0221, 0x11,
	0x0222, 0x01,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x0A,
	0x3140, 0x02,
	0x3243, 0x00,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x4254, 0x7F,

	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0A,
	0x040D, 0x20,
	0x040E, 0x07,
	0x040F, 0x90,

	0x034C, 0x0A,
	0x034D, 0x20,
	0x034E, 0x07,
	0x034F, 0x90,

	0x0301, 0x05,
	0x0303, 0x04,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x5E,
	0x030B, 0x02,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x0A,
	0x0310, 0x01,
	0x0820, 0x0C,
	0x0821, 0x78,
	0x0822, 0x00,
	0x0823, 0x00,
	0xBC41, 0x01,

	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E3B, 0x01,

	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x5B,
	0x3C01, 0x4A,
	0x3C02, 0x73,
	0x3C03, 0x64,
	0x3C04, 0x34,
	0x3C05, 0x88,
	0x3C06, 0x5C,
	0x3C0A, 0x14,
	0x3C0B, 0x00,
	0x3F14, 0x01,
	0x3F17, 0x00,
	0x3F3C, 0x01,
	0x3F78, 0x03,
	0x3F79, 0xD4,
	0x3F7A, 0x00,
	0x3F7B, 0x00,
	0x562B, 0x0A,
	0x562D, 0x0C,
	0x5617, 0x0A,
	0x9104, 0x04,

	0x0202, 0x08,
	0x0203, 0x24,
	0x0224, 0x01,
	0x0225, 0xF4,

	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,

	0xE000, 0x00
};

static void preview_setting(struct subdrv_ctx *ctx)
{
	imx350_table_write_cmos_sensor(ctx, addr_data_pair_preview_imx350,
	sizeof(addr_data_pair_preview_imx350) / sizeof(kal_uint16));
	/* zvhdr_setting(ctx); */
}				/*    preview_setting  */

kal_uint16 addr_data_pair_capture_imx350[] = {
	0x0100, 0x00,
/*24fps*/
#ifdef IMX350_24_FPS
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,

	0x0342, 0x22,
	0x0343, 0x00,

	0x0340, 0x0F,
	0x0341, 0x6C,

	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x14,
	0x0349, 0x3F,
	0x034A, 0x0F,
	0x034B, 0x27,

	0x0220, 0x00,
	0x0221, 0x11,
	0x0222, 0x01,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x02,
	0x3243, 0x00,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x4254, 0x7F,

	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x14,
	0x040D, 0x40,
	0x040E, 0x0F,
	0x040F, 0x28,

	0x034C, 0x14,
	0x034D, 0x40,
	0x034E, 0x0F,
	0x034F, 0x28,

	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x5E,
	0x030B, 0x01,
	0x030D, 0x04,
	0x030E, 0x00,
	0x030F, 0xFA,
	0x0310, 0x01,
	0x0820, 0x17,
	0x0821, 0x70,
	0x0822, 0x00,
	0x0823, 0x00,
	0xBC41, 0x01,

	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E3B, 0x01,

	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x5B,
	0x3C01, 0x4A,
	0x3C02, 0x73,
	0x3C03, 0x64,
	0x3C04, 0x34,
	0x3C05, 0x88,
	0x3C06, 0x9E,
	0x3C0A, 0x14,
	0x3C0B, 0x00,
	0x3F14, 0x01,
	0x3F17, 0x00,
	0x3F3C, 0x01,
	0x3F78, 0x04,
	0x3F79, 0x8C,
	0x3F7A, 0x00,
	0x3F7B, 0x00,
	0x562B, 0x32,
	0x562D, 0x34,
	0x5617, 0x32,
	0x9104, 0x04,
	0x97C1, 0x00,

	0x0202, 0x0F,
	0x0203, 0x58,
	0x0224, 0x01,
	0x0225, 0xF4,

	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,

/*30fps*/
#else
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,

	0x0342, 0x1A,
	0x0343, 0x40,

	0x0340, 0x0F,
	0x0341, 0xF0,

	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x14,
	0x0349, 0x3F,
	0x034A, 0x0F,
	0x034B, 0x27,

	0x0220, 0x00,
	0x0221, 0x11,
	0x0222, 0x01,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x02,
	0x3243, 0x00,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x4254, 0x7F,

	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x14,
	0x040D, 0x40,
	0x040E, 0x0F,
	0x040F, 0x28,

	0x034C, 0x14,
	0x034D, 0x40,
	0x034E, 0x0F,
	0x034F, 0x28,

	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x57,
	0x030B, 0x01,
	0x030D, 0x0A,
	0x030E, 0x02,
	0x030F, 0xBE,
	0x0310, 0x01,
	0x0820, 0x1A,
	0x0821, 0x53,
	0x0822, 0x33,
	0x0823, 0x33,
	0xBC41, 0x01,

	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E3B, 0x01,

	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x5B,
	0x3C01, 0x4A,
	0x3C02, 0x73,
	0x3C03, 0x64,
	0x3C04, 0x34,
	0x3C05, 0x88,
	0x3C06, 0x2E,
	0x3C0A, 0x14,
	0x3C0B, 0x00,
	0x3F14, 0x01,
	0x3F17, 0x00,
	0x3F3C, 0x01,
	0x3F78, 0x01,
	0x3F79, 0x44,
	0x3F7A, 0x01,
	0x3F7B, 0xF4,
	0x562B, 0x32,
	0x562D, 0x34,
	0x5617, 0x32,
	0x9104, 0x04,
	0x97C1,	0x00,/*NEW*/

	0x0202, 0x0F,
	0x0203, 0xDC,
	0x0224, 0x01,
	0x0225, 0xF4,

	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
#endif

	0x0830, 0x00,
	0x0831, 0x00,
	0x0832, 0x00,

	0xE000, 0x00
};

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable)
		write_cmos_sensor(ctx, 0x0100, 0X01);
	else
		write_cmos_sensor(ctx, 0x0100, 0x00);

	return ERROR_NONE;
}


kal_uint16 addr_data_pair_capture_imx350_pdaf_on[] = {
	/*PDAF*/
	/*PD_OUT_EN=1 */
	0x3E37, 0x01,
	/*AREA MODE */
	0x38A3, 0x01,
	/*Fixed area mode */
	0x38A4, 0x00,
	0x38A5, 0x70,
	0x38A6, 0x00,
	0x38A7, 0x58,
	0x38A8, 0x02,
	0x38A9, 0x80,
	0x38AA, 0x02,
	0x38AB, 0x80,
};

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d hdr:%d pdaf:%d\n",
		currefps, ctx->hdr_mode, ctx->pdaf_mode);

	imx350_table_write_cmos_sensor(ctx, addr_data_pair_capture_imx350,
	sizeof(addr_data_pair_capture_imx350) / sizeof(kal_uint16));

	if (ctx->pdaf_mode == 1) {
		imx350_table_write_cmos_sensor(ctx,
			addr_data_pair_capture_imx350_pdaf_on,

		sizeof(
		addr_data_pair_capture_imx350_pdaf_on) / sizeof(kal_uint16));

		imx350_apply_SPC(ctx);
	}

}

kal_uint16 addr_data_pair_video_imx350[] = {
	0x0100, 0x00,

	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,

	0x0342, 0x22,
	0x0343, 0x00,

	0x0340, 0x0B,
	0x0341, 0xFC,

	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x01,
	0x0347, 0xE0,
	0x0348, 0x14,
	0x0349, 0x3F,
	0x034A, 0x0D,
	0x034B, 0x43,

	0x0220, 0x00,
	0x0221, 0x11,
	0x0222, 0x01,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x02,
	0x3243, 0x00,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x4254, 0x7F,

	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x14,
	0x040D, 0x40,
	0x040E, 0x0B,
	0x040F, 0x64,

	0x034C, 0x14,
	0x034D, 0x40,
	0x034E, 0x0B,
	0x034F, 0x64,

	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x4E,
	0x030B, 0x01,
	0x030D, 0x0A,
	0x030E, 0x02,
	0x030F, 0xBE,
	0x0310, 0x01,
	0x0820, 0x1A,
	0x0821, 0x53,
	0x0822, 0x33,
	0x0823, 0x33,
	0xBC41, 0x01,

	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E3B, 0x01,

	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x5B,
	0x3C01, 0x4A,
	0x3C02, 0x73,
	0x3C03, 0x64,
	0x3C04, 0x34,
	0x3C05, 0x88,
	0x3C06, 0x9E,
	0x3C0A, 0x14,
	0x3C0B, 0x00,
	0x3F14, 0x01,
	0x3F17, 0x00,
	0x3F3C, 0x01,
	0x3F78, 0x04,
	0x3F79, 0x8C,
	0x3F7A, 0x02,
	0x3F7B, 0x74,
	0x562B, 0x32,
	0x562D, 0x34,
	0x5617, 0x32,
	0x9104, 0x04,
	0x97C1, 0x00,

	0x0202, 0x0B,
	0x0203, 0xE8,
	0x0224, 0x01,
	0x0225, 0xF4,


	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,

	0x0830, 0x00,
	0x0831, 0x00,
	0x0832, 0x00,

	0xE000, 0x00,
};

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF("E! %s:%d\n", __func__, currefps);

	imx350_table_write_cmos_sensor(ctx, addr_data_pair_video_imx350,
	sizeof(addr_data_pair_video_imx350) / sizeof(kal_uint16));
}

kal_uint16 addr_data_pair_hs_video_imx350[] = {
	0x0100, 0x00,

	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,

	0x0342, 0x15,
	0x0343, 0xF8,

	0x0340, 0x04,
	0x0341, 0xA0,

	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x03,
	0x0347, 0x50,
	0x0348, 0x14,
	0x0349, 0x3F,
	0x034A, 0x0B,
	0x034B, 0xCF,

	0x0220, 0x00,
	0x0221, 0x11,
	0x0222, 0x01,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x0A,
	0x3140, 0x02,
	0x3243, 0x00,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x4254, 0x7F,

	0x0401, 0x02,
	0x0404, 0x00,
	0x0405, 0x18,
	0x0408, 0x01,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x04,
	0x040C, 0x07,
	0x040D, 0x80,
	0x040E, 0x04,
	0x040F, 0x38,

	0x034C, 0x05,
	0x034D, 0x00,
	0x034E, 0x02,
	0x034F, 0xD0,

	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x4E,
	0x030B, 0x04,
	0x030D, 0x0C,
	0x030E, 0x03,
	0x030F, 0xF0,
	0x0310, 0x01,
	0x0820, 0x07,
	0x0821, 0xE0,
	0x0822, 0x00,
	0x0823, 0x00,
	0xBC41, 0x01,

	0x3E20, 0x01,
	0x3E37, 0x00,
	0x3E3B, 0x01,

	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x5B,
	0x3C01, 0x4A,
	0x3C02, 0x73,
	0x3C03, 0x64,
	0x3C04, 0x34,
	0x3C05, 0x88,
	0x3C06, 0x78,
	0x3C0A, 0x14,
	0x3C0B, 0x00,
	0x3F14, 0x01,
	0x3F17, 0x00,
	0x3F3C, 0x01,
	0x3F78, 0x02,
	0x3F79, 0x8C,
	0x3F7A, 0x04,
	0x3F7B, 0x5C,
	0x562B, 0x32,
	0x562D, 0x34,
	0x5617, 0x32,
	0x9104, 0x04,
	0x97C1, 0x04,

	0x0202, 0x04,
	0x0203, 0x8C,
	0x0224, 0x01,
	0x0225, 0xF4,

	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
};

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	imx350_table_write_cmos_sensor(ctx, addr_data_pair_hs_video_imx350,
	sizeof(addr_data_pair_hs_video_imx350) / sizeof(kal_uint16));
	zvhdr_setting(ctx);
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor(ctx, 0x0601, 0x02);
	else
		write_cmos_sensor(ctx, 0x0601, 0x00);

	ctx->test_pattern = enable;
	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = return_lot_id_from_otp(ctx);
			/* return_sensor_id(); */
			if (*sensor_id == imgsensor_info.sensor_id) {
				imx350_read_SPC(ctx, imx350_SPC_data);
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF(
				"Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				ctx->i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;

	KD_SENSOR_PROFILE_INIT(ctx);
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = return_lot_id_from_otp(ctx);
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF(
					"i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_INF(
				"Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				ctx->i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	KD_SENSOR_PROFILE(ctx, "imx350_open_1");
	/* initail sequence write in  */
	sensor_init(ctx);

	KD_SENSOR_PROFILE(ctx, "sensor_init");


	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->hdr_mode = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	KD_SENSOR_PROFILE(ctx, "imx350_open_2");
	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
	write_cmos_sensor(ctx, 0x0100, 0x00);	/*stream off */
	return ERROR_NONE;
}				/*    close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	KD_SENSOR_PROFILE_INIT(ctx);

	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	/* ctx->video_mode = KAL_FALSE; */
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	KD_SENSOR_PROFILE(ctx, "pre_lock");

	preview_setting(ctx);

	KD_SENSOR_PROFILE(ctx, "pre_setting");
	return ERROR_NONE;
}				/*    preview   */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	KD_SENSOR_PROFILE_INIT(ctx);

	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;


	KD_SENSOR_PROFILE(ctx, "cap_lock");

	capture_setting(ctx, ctx->current_fps);	/*Full mode */

	KD_SENSOR_PROFILE(ctx, "cap_setting");

	return ERROR_NONE;
}				/* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	KD_SENSOR_PROFILE_INIT(ctx);

	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;

	KD_SENSOR_PROFILE(ctx, "nv_lock");

	normal_video_setting(ctx, ctx->current_fps);

	KD_SENSOR_PROFILE(ctx, "nv_setting");
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */

	return ERROR_NONE;
}				/*    normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	KD_SENSOR_PROFILE_INIT(ctx);

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	KD_SENSOR_PROFILE(ctx, "hv_lock");

	hs_video_setting(ctx);

	KD_SENSOR_PROFILE(ctx, "hv_setting");
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    hs_video   */

static int get_resolution(
		struct subdrv_ctx *ctx,
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < imgsensor_info.sensor_mode_num) {
			sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
			sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
		} else {
			sensor_resolution->SensorWidth[i] = 0;
			sensor_resolution->SensorHeight[i] = 0;
		}
	}


	return ERROR_NONE;
}				/*    get_resolution    */

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*LOG_INF("scenario_id = %d\n", scenario_id); */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =
		imgsensor_info.pre_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =
		imgsensor_info.cap_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =
		imgsensor_info.video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM1] =
		imgsensor_info.custom1_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM2] =
		imgsensor_info.custom2_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

/* PDAF_SUPPORT_CAMSV; */
/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */
	sensor_info->PDAF_Support = 0;
#if defined(imx350_ZHDR)
	/* 3; */ /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR */
	sensor_info->HDR_Support = 0;
	/*0: no support, 1: G0,R0.B0, 2: G0,R0.B1, 3: G0,R1.B0, 4: G0,R1.B1 */
	/*5: G1,R0.B0, 6: G1,R0.B1, 7: G1,R1.B0, 8: G1,R1.B1 */
	sensor_info->ZHDR_Mode = 8;
#else
	sensor_info->HDR_Support = 2;	/*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR */
#endif

	sensor_info->SensorHorFOV = H_FOV;
	sensor_info->SensorVerFOV = V_FOV;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;


	return ERROR_NONE;
}				/*    get_info  */

static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	ctx->current_scenario_id = scenario_id;
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		preview(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		capture(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		normal_video(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		hs_video(ctx, image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{				/* This Function not used after ROME */
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;

	ctx->current_fps = framerate;
	set_max_framerate(ctx, ctx->current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx,
	kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	if (enable)		/* enable auto flicker */
		ctx->autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length =
		    imgsensor_info.pre.pclk /
		    framerate * 10 / imgsensor_info.pre.linelength;
		ctx->dummy_line =
		    (frame_length > imgsensor_info.pre.framelength)
		    ? (frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
		    imgsensor_info.normal_video.pclk /
		    framerate * 10 / imgsensor_info.normal_video.linelength;

		ctx->dummy_line =
		(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength) : 0;

		ctx->frame_length =
		 imgsensor_info.normal_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		if (ctx->current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			     framerate, imgsensor_info.cap.max_framerate / 10);

		frame_length = imgsensor_info.cap.pclk /
			framerate * 10 / imgsensor_info.cap.linelength;
		ctx->dummy_line =
		    (frame_length > imgsensor_info.cap.framelength)
		    ? (frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
		    imgsensor_info.cap.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk /
		    framerate * 10 / imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
		    (frame_length > imgsensor_info.hs_video.framelength)
		    ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		ctx->frame_length =
		    imgsensor_info.hs_video.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		break;
	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk /
		    framerate * 10 / imgsensor_info.pre.linelength;
		ctx->dummy_line =
		    (frame_length > imgsensor_info.pre.framelength)
		    ? (frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		set_dummy(ctx);
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*LOG_INF("scenario_id = %d\n", scenario_id); */

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 imx350_awb_gain(struct subdrv_ctx *ctx, struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	LOG_INF("%s\n", __func__);

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	LOG_INF("[%s] ABS_GAIN_GR:%d, grgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR,	grgain_32);
	LOG_INF("[%s] ABS_GAIN_R:%d, rgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_R, rgain_32);
	LOG_INF("[%s] ABS_GAIN_B:%d, bgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_B, bgain_32);
	LOG_INF("[%s] ABS_GAIN_GB:%d, gbgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GB,	gbgain_32);

	write_cmos_sensor(ctx, 0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor(ctx, 0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor(ctx, 0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor(ctx, 0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor(ctx, 0x0b95, gbgain_32 & 0xFF);
	return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
	UINT8 temperature;
	INT32 temperature_convert;

	temperature = read_cmos_sensor(ctx, 0x013a);

	if (temperature >= 0x0 && temperature <= 0x4F)
		temperature_convert = temperature;
	else if (temperature >= 0x50 && temperature <= 0x7F)
		temperature_convert = 80;
	else if (temperature >= 0x80 && temperature <= 0xEC)
		temperature_convert = -20;
	else
		temperature_convert = (INT8) temperature;

/* LOG_INF("temp_c(%d), read_reg(%d)\n", temperature_convert, temperature); */

	return temperature_convert;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
/* unsigned long long *feature_return_data =
 * (unsigned long long*)feature_para;
 */

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB =
		(struct SET_SENSOR_AWB_GAIN *) feature_para;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(imx350_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)imx350_ana_gain_table,
			sizeof(imx350_ana_gain_table));
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = ctx->line_length;
		*feature_return_para_16 = ctx->frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = ctx->pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN:
		set_dual_gain(ctx,
			(UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(ctx,
			sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(ctx, sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(ctx, feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(ctx,
			(BOOL)(*feature_data_16), *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)(*feature_data),
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM)(*feature_data),
			(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		imx350_read_DCC(ctx, (kal_uint16) (*feature_data),
				(char *)(uintptr_t) (*(feature_data + 1)),
				(kal_uint32) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (BOOL)(*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		ctx->current_fps = (UINT16)*feature_data_32;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32) *feature_data);

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[3],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[0],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		default:
			break;
		}
		break;
		/*HDR CMD */
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("hdr enable :%d\n", *feature_data_32);
		ctx->hdr_mode = (UINT8)*feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
		hdr_write_shutter(ctx,
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16) *feature_data);
		pvcinfo =
	     (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx350_awb_gain(ctx, pSetSensorAWB);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		LOG_INF(
			"SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu\n",
			*feature_data);
		/*
		 * SENSOR_VHDR_MODE_NONE  = 0x0,
		 * SENSOR_VHDR_MODE_IVHDR = 0x01,
		 * SENSOR_VHDR_MODE_MVHDR = 0x02,
		 * SENSOR_VHDR_MODE_ZVHDR = 0x09
		 */
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		break;

		/*END OF HDR CMD */
		/*PDAF CMD */
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF(
			"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu\n",
			*feature_data);
	/* PDAF capacity enable or not, 2p8 only full size support PDAF */
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		ctx->pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx,
		    (UINT16)(*feature_data), (UINT16)(*(feature_data + 1)),
		    (BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length(ctx) support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature(ctx);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx350_get_pdaf_reg_setting(ctx,
			(*feature_para_len) / sizeof(UINT32), feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx350_set_pdaf_reg_setting(ctx,
			(*feature_para_len) / sizeof(UINT32), feature_data_16);
		break;

	case SENSOR_FEATURE_SET_PDFOCUS_AREA:
		LOG_INF(
			"SENSOR_FEATURE_SET_imx350_PDFOCUS_AREA Start Pos=%d, Size=%d\n",
			(UINT32) *feature_data, (UINT32) *(feature_data + 1));
		imx350_set_pd_focus_area(ctx, *feature_data, *(feature_data + 1));
		break;
		/*End of PDAF */
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
		streaming_control(ctx, KAL_TRUE);
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;

	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control(ctx)  */


static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	/* support long exposure at most 128 times) */
	.exposure_max = (0xffff * 128) - 18,
	.exposure_min = 8,
	.exposure_step = 1,
	.frame_time_delay_frame = 3,
	.margin = 18,
	.max_frame_length = 0xffff,

	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/* current shutter */
	.gain = BASEGAIN * 4,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	/* auto flicker enable: KAL_FALSE for disable auto flicker,
	 * KAL_TRUE for enable auto flicker
	 */
	.test_pattern = KAL_FALSE,

	/* current scenario id */
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.hdr_mode = 0,	/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x6c,	/* record current sensor's i2c write id */
};

static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
	*temp = get_sensor_temperature(ctx) * 1000;
	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = get_info,
	.get_resolution = get_resolution,
	.control = control,
	.feature_control = feature_control,
	.close = close,
	.get_temp = get_temp,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_PDN, 0, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_AVDD, 2800000, 0},
	{HW_ID_DVDD, 1200000, 0},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 5},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 1},
};

const struct subdrv_entry imx350_mipi_raw_entry = {
	.name = "imx350_mipi_raw",
	.id = IMX350_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

