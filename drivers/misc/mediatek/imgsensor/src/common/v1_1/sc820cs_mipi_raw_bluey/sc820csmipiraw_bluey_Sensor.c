/*****************************************************************************D/HEAD
 *
 * Filename:
 * ---------
 *	 sc820cs_mipi_raw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
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

#include "sc820csmipiraw_bluey_Sensor.h"
#include "imgsensor_common.h"

#define PFX "sc820cs_camera_sensor"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define SC820CS_SENSOR_GAIN_MAX_VALID_INDEX  6
#define SC820CS_SENSOR_GAIN_MAP_SIZE         6

#define MULTI_WRITE 1

#define SC820CS_SENSOR_BASE_GAIN           0x400
#define SC820CS_SENSOR_MAX_GAIN            (32 * SC820CS_SENSOR_BASE_GAIN )
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_uint8 otp_data[0x2000] = {0};
static kal_uint8 deviceInfo_register_value = 0x00;
// extern enum IMGSENSOR_RETURN Eeprom_DataInit(
//             enum IMGSENSOR_SENSOR_IDX sensor_idx,
//             kal_uint32 sensorID);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = SC820CS_SENSOR_ID_BLUEY,
	.checksum_value = 0x55e2a82f,        //checksum value for Camera Auto Test

	.pre = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},

	.cap = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},

	.normal_video = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1840,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},

	.custom1 = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 3125,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 240,
	},

	.custom2 = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},
	.custom3 = {
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 292800000,//732Mbps x 4/10
		.max_framerate = 300,
	},
	.margin = 4,
	.min_shutter = 2,
	.max_frame_length = 0x3fff,
	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.gain_step = 4,
	.gain_type = 3,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,

	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 8,	  //support sensor mode num

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	.custom1_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x6c, 0x6d, 0x20,0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x4de,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0,
	.i2c_write_id = 0x6c,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] ={
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448},	/* Preview */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448},	/* capture */
	{ 3264, 2448, 0, 304, 3264, 1840, 3264, 1840, 0, 0, 3264, 1840, 0, 0, 3264, 1840},	/* normal video */
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224},	/* hs video */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448},	/* slim_video  */
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448},	//custom1
	{ 3264, 2448, 0, 0, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224},	//custom2
	{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}	   	//custom3
};

#if MULTI_WRITE
#define I2C_BUFFER_LEN 445

static kal_uint16 w1sc820cswidely_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;
		}

		if ((I2C_BUFFER_LEN - tosend) < 3 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				3, imgsensor_info.i2c_speed);

			tosend = 0;
		}
	}
	return 0;
}
#endif

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

	//kdSetI2CSpeed(400);

	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor8(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("frame length = %d\n", imgsensor.frame_length);
	write_cmos_sensor8(0x320e, (imgsensor.frame_length >> 8) & 0xff);
	write_cmos_sensor8(0x320f, imgsensor.frame_length & 0xff);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x3107) << 8) | read_cmos_sensor(0x3108)); //0xeb15
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

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
static void set_shutter(kal_uint16 shutter)
{
    kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	//printk("pangfei shutter %d line %d\n",shutter,__LINE__);
	spin_lock(&imgsensor_drv_lock);

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
              // Extend frame length
              // write_cmos_sensor8_8(0x0104, 0x01);
              //write_cmos_sensor8(0x326d, (imgsensor.frame_length >> 16) & 0x7f);
              write_cmos_sensor8(0x320e, (imgsensor.frame_length >> 8) & 0xff);
              write_cmos_sensor8(0x320f, imgsensor.frame_length & 0xFF);
              //write_cmos_sensor8_8(0x0104, 0x00);
            }
    } else {
              // Extend frame length
              //write_cmos_sensor8(0x0104, 0x01);
              //write_cmos_sensor8(0x326d, (imgsensor.frame_length >> 16) & 0x7f);
              write_cmos_sensor8(0x320e, imgsensor.frame_length >> 8);
              write_cmos_sensor8(0x320f, imgsensor.frame_length & 0xFF);
              //write_cmos_sensor8(0x0104, 0x00);
    }
    // Update Shutter
    shutter = shutter *2;
    //shutter = 3;
    //write_cmos_sensor8(0x3802,0x01);//group hold on
    //write_cmos_sensor8(0x3e20, (shutter >> 20) & 0x0F);
    write_cmos_sensor8(0x3e00, (shutter >> 12) & 0xFF);
    write_cmos_sensor8(0x3e01, (shutter >> 4)&0xFF);
    write_cmos_sensor8(0x3e02, (shutter<<4) & 0xF0);
    LOG_INF("Exit! shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = gain << 4;

	if (reg_gain < SC820CS_SENSOR_BASE_GAIN)
		reg_gain = SC820CS_SENSOR_BASE_GAIN;
	else if (reg_gain > SC820CS_SENSOR_MAX_GAIN)
		reg_gain = SC820CS_SENSOR_MAX_GAIN;

	return (kal_uint16)reg_gain;
}
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	kal_uint32 temp_gain;
	kal_int16 gain_index;
	kal_uint16 SC820CS_AGC_Param[SC820CS_SENSOR_GAIN_MAP_SIZE][2] = {
		{  1024,  0x00 },
		{  2048,  0x08 },
		{  4096,  0x09 },
		{  8192,  0x0B },
		{ 16384,  0x0f },
		{ 32768,  0x1f },
	};

	reg_gain = gain2reg(gain);

	for (gain_index = SC820CS_SENSOR_GAIN_MAX_VALID_INDEX - 1; gain_index >= 0; gain_index--)
		if (reg_gain >= SC820CS_AGC_Param[gain_index][0])
			break;

	write_cmos_sensor8(0x3e08, SC820CS_AGC_Param[gain_index][1]);
	temp_gain = reg_gain * SC820CS_SENSOR_BASE_GAIN / SC820CS_AGC_Param[gain_index][0];
	write_cmos_sensor8(0x3e07, (temp_gain >> 3) & 0xff);
	LOG_INF("SC820CS_AGC_Param[gain_index][1] = 0x%x, temp_gain = 0x%x, reg_gain = %d, gain = %d\n",
		SC820CS_AGC_Param[gain_index][1], temp_gain, reg_gain, gain);

	return reg_gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le: 0x%x, se: 0x%x, gain: 0x%x\n", le, se, gain);
}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
}
#endif
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
	/* No Need to implement this function */
}
#if MULTI_WRITE
kal_uint16 addr_data_pair_init_w1sc820cswidely[] = {
/* update20230322wangruo */
0x36e9,0x80,
0x37f9,0x80,
0x36ea,0x17,
0x36eb,0x0c,
0x36ec,0x4a,
0x36ed,0x18,
0x37fa,0x0b,
0x37fb,0x71,
0x37fc,0x10,
0x37fd,0x58,
0x36e9,0x44,
0x37f9,0x24,
0x301f,0x06,
0x3205,0xc7,
0x3211,0x04,
0x3270,0x00,
0x3271,0x00,
0x3272,0x00,
0x3273,0x03,
0x3301,0x08,
0x3303,0x10,
0x3306,0x84,
0x3309,0x88,
0x330a,0x01,
0x330b,0x0c,
0x330c,0x10,
0x330d,0x18,
0x330e,0x30,
0x3314,0x15,
0x331f,0x79,
0x3326,0x0e,
0x3327,0x0a,
0x3329,0x0b,
0x3333,0x10,
0x3334,0x40,
0x335d,0x60,
0x3364,0x56,
0x336c,0xce,
0x3390,0x08,
0x3391,0x09,
0x3392,0x0f,
0x3393,0x10,
0x3394,0x20,
0x3395,0x28,
0x33ad,0x3c,
0x33af,0x70,
0x33b2,0x70,
0x33b3,0x40,
0x349f,0x1e,
0x34a6,0x09,
0x34a7,0x0f,
0x34a8,0x30,
0x34a9,0x20,
0x34f8,0x1f,
0x34f9,0x08,
0x3637,0x43,
0x363c,0x8d,
0x3670,0x4a,
0x3674,0xf6,
0x3675,0xdc,
0x3676,0xcc,
0x367c,0x09,
0x367d,0x0f,
0x3690,0x34,
0x3691,0x44,
0x3692,0x55,
0x3698,0x86,
0x3699,0x8d,
0x369a,0x99,
0x369b,0xb7,
0x369c,0x0f,
0x369d,0x1f,
0x36a2,0x09,
0x36a3,0x0b,
0x36a4,0x0f,
0x36b0,0x48,
0x36b1,0x38,
0x36b2,0x41,
0x370f,0x01,
0x3724,0xc1,
0x3771,0x07,
0x3772,0x03,
0x3773,0x63,
0x377a,0x08,
0x377b,0x0f,
0x3901,0x04,
0x3903,0xa0,
0x3905,0x8d,
0x391d,0x01,
0x3926,0x23,
0x393f,0x80,
0x3940,0x00,
0x3941,0x00,
0x3942,0x00,
0x3943,0x63,
0x3944,0x5f,
0x3e00,0x01,
0x3e01,0x38,
0x3e02,0x00,
0x4401,0x13,
0x4402,0x03,
0x4403,0x0e,
0x4404,0x28,
0x4405,0x34,
0x4407,0x0e,
0x440c,0x42,
0x440d,0x42,
0x440e,0x32,
0x440f,0x53,
0x4412,0x01,
0x4424,0x01,
0x442d,0x00,
0x442e,0x00,
0x4509,0x28,
0x450d,0x18,
0x451d,0xc8,
0x4526,0x09,
0x4819,0x0b,
0x481b,0x07,
0x481d,0x19,
0x481f,0x05,
0x4821,0x0b,
0x4823,0x06,
0x4825,0x05,
0x4827,0x06,
0x4829,0x0a,
0x5000,0x0e,
0x550e,0x00,
0x550f,0x34,
0x5780,0x66,
0x578d,0x40,
0x57aa,0xeb,
0x3652,0x22,
0x3653,0x22,
0x3654,0x01,
};
#endif
static void sensor_init(void)
{/* update20230213wangruo */
	LOG_INF(">> %s()\n", __func__);
	/*V02P08_20210628*/
#if MULTI_WRITE
	write_cmos_sensor8(0x0103,0x01);
	mDELAY(10);
    write_cmos_sensor8(0x0100,0x00);

     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_init_w1sc820cswidely,
         sizeof(addr_data_pair_init_w1sc820cswidely) /
         sizeof(kal_uint16));
		 if ((read_cmos_sensor(0x800D)&0x0F)==0)
		{
		write_cmos_sensor8(0x550F,0x34);
		}else {
		write_cmos_sensor8(0x550F,0xBC);
		}

	#else
	write_cmos_sensor8(0x0103,0x01);
		mDELAY(10);
	write_cmos_sensor8(0x36ea,0x0c);
	write_cmos_sensor8(0x36eb,0x0c);
	write_cmos_sensor8(0x36ec,0x0a);
	write_cmos_sensor8(0x36ed,0x18);
	write_cmos_sensor8(0x37fa,0x0b);
	write_cmos_sensor8(0x37fb,0x71);
	write_cmos_sensor8(0x37fc,0x10);
	write_cmos_sensor8(0x37fd,0x58);
	write_cmos_sensor8(0x36e9,0x00);
	write_cmos_sensor8(0x37f9,0x24);
	write_cmos_sensor8(0x301f,0x11);
	write_cmos_sensor8(0x302d,0x20);
	write_cmos_sensor8(0x3205,0xc7);
	write_cmos_sensor8(0x3211,0x04);
	write_cmos_sensor8(0x3270,0x00);
	write_cmos_sensor8(0x3271,0x00);
	write_cmos_sensor8(0x3272,0x00);
	write_cmos_sensor8(0x3273,0x03);
	write_cmos_sensor8(0x3301,0x08);
	write_cmos_sensor8(0x3303,0x10);
	write_cmos_sensor8(0x3306,0x84);
	write_cmos_sensor8(0x3309,0x88);
	write_cmos_sensor8(0x330a,0x01);
	write_cmos_sensor8(0x330b,0x0c);
	write_cmos_sensor8(0x330c,0x10);
	write_cmos_sensor8(0x330d,0x18);
	write_cmos_sensor8(0x330e,0x30);
	write_cmos_sensor8(0x3314,0x15);
	write_cmos_sensor8(0x331f,0x79);
	write_cmos_sensor8(0x3326,0x0e);
	write_cmos_sensor8(0x3327,0x0a);
	write_cmos_sensor8(0x3329,0x0b);
	write_cmos_sensor8(0x3333,0x10);
	write_cmos_sensor8(0x3334,0x40);
	write_cmos_sensor8(0x335d,0x60);
	write_cmos_sensor8(0x3364,0x56);
	write_cmos_sensor8(0x336c,0xce);
	write_cmos_sensor8(0x3390,0x08);
	write_cmos_sensor8(0x3391,0x09);
	write_cmos_sensor8(0x3392,0x0f);
	write_cmos_sensor8(0x3393,0x10);
	write_cmos_sensor8(0x3394,0x20);
	write_cmos_sensor8(0x3395,0x28);
	write_cmos_sensor8(0x33ad,0x3c);
	write_cmos_sensor8(0x33af,0x70);
	write_cmos_sensor8(0x33b2,0x70);
	write_cmos_sensor8(0x33b3,0x40);
	write_cmos_sensor8(0x349f,0x1e);
	write_cmos_sensor8(0x34a6,0x09);
	write_cmos_sensor8(0x34a7,0x0f);
	write_cmos_sensor8(0x34a8,0x30);
	write_cmos_sensor8(0x34a9,0x20);
	write_cmos_sensor8(0x34f8,0x1f);
	write_cmos_sensor8(0x34f9,0x08);
	write_cmos_sensor8(0x3637,0x43);
	write_cmos_sensor8(0x363c,0x8d);
	write_cmos_sensor8(0x3670,0x4a);
	write_cmos_sensor8(0x3674,0xf6);
	write_cmos_sensor8(0x3675,0xdc);
	write_cmos_sensor8(0x3676,0xcc);
	write_cmos_sensor8(0x367c,0x09);
	write_cmos_sensor8(0x367d,0x0f);
	write_cmos_sensor8(0x3690,0x34);
	write_cmos_sensor8(0x3691,0x44);
	write_cmos_sensor8(0x3692,0x55);
	write_cmos_sensor8(0x3698,0x86);
	write_cmos_sensor8(0x3699,0x8d);
	write_cmos_sensor8(0x369a,0x99);
	write_cmos_sensor8(0x369b,0xb7);
	write_cmos_sensor8(0x369c,0x0f);
	write_cmos_sensor8(0x369d,0x1f);
	write_cmos_sensor8(0x36a2,0x09);
	write_cmos_sensor8(0x36a3,0x0b);
	write_cmos_sensor8(0x36a4,0x0f);
	write_cmos_sensor8(0x36b0,0x48);
	write_cmos_sensor8(0x36b1,0x38);
	write_cmos_sensor8(0x36b2,0x41);
	write_cmos_sensor8(0x370f,0x01);
	write_cmos_sensor8(0x3724,0xc1);
	write_cmos_sensor8(0x3771,0x07);
	write_cmos_sensor8(0x3772,0x03);
	write_cmos_sensor8(0x3773,0x63);
	write_cmos_sensor8(0x377a,0x08);
	write_cmos_sensor8(0x377b,0x0f);
	write_cmos_sensor8(0x3901,0x04);
	write_cmos_sensor8(0x3903,0xa0);
	write_cmos_sensor8(0x3905,0x8d);
	write_cmos_sensor8(0x391d,0x01);
	write_cmos_sensor8(0x3926,0x23);
	write_cmos_sensor8(0x393f,0x80);
	write_cmos_sensor8(0x3940,0x00);
	write_cmos_sensor8(0x3941,0x00);
	write_cmos_sensor8(0x3942,0x00);
	write_cmos_sensor8(0x3943,0x63);
	write_cmos_sensor8(0x3944,0x5f);
	write_cmos_sensor8(0x3e00,0x01);
	write_cmos_sensor8(0x3e01,0x38);
	write_cmos_sensor8(0x3e02,0x00);
	write_cmos_sensor8(0x4000,0x00);
	write_cmos_sensor8(0x4001,0x04);
	write_cmos_sensor8(0x4002,0x53);
	write_cmos_sensor8(0x4003,0xef);
	write_cmos_sensor8(0x4004,0x36);
	write_cmos_sensor8(0x4005,0x00);
	write_cmos_sensor8(0x4006,0x05);
	write_cmos_sensor8(0x4007,0x95);
	write_cmos_sensor8(0x4008,0x00);
	write_cmos_sensor8(0x4009,0xfe);
	write_cmos_sensor8(0x4401,0x13);
	write_cmos_sensor8(0x4402,0x03);
	write_cmos_sensor8(0x4403,0x0e);
	write_cmos_sensor8(0x4404,0x28);
	write_cmos_sensor8(0x4405,0x34);
	write_cmos_sensor8(0x4407,0x0e);
	write_cmos_sensor8(0x440c,0x42);
	write_cmos_sensor8(0x440d,0x42);
	write_cmos_sensor8(0x440e,0x32);
	write_cmos_sensor8(0x440f,0x53);
	write_cmos_sensor8(0x4412,0x01);
	write_cmos_sensor8(0x4424,0x01);
	write_cmos_sensor8(0x442d,0x00);
	write_cmos_sensor8(0x442e,0x00);
	write_cmos_sensor8(0x4509,0x28);
	write_cmos_sensor8(0x450d,0x18);
	write_cmos_sensor8(0x451d,0xc8);
	write_cmos_sensor8(0x4526,0x09);
	write_cmos_sensor8(0x4819,0x0b);
	write_cmos_sensor8(0x481b,0x07);
	write_cmos_sensor8(0x481d,0x18);
	write_cmos_sensor8(0x481f,0x05);
	write_cmos_sensor8(0x4821,0x0c);
	write_cmos_sensor8(0x4823,0x06);
	write_cmos_sensor8(0x4825,0x05);
	write_cmos_sensor8(0x4827,0x06);
	write_cmos_sensor8(0x4829,0x0a);
	write_cmos_sensor8(0x5000,0x0e);
	write_cmos_sensor8(0x550e,0x00);
	write_cmos_sensor8(0x3652,0x22);
	write_cmos_sensor8(0x3653,0x22);
	write_cmos_sensor8(0x3654,0x01);
	if ((read_cmos_sensor(0x800D)&0x0F)==0)
		{
		write_cmos_sensor8(0x550F,0x34);
	}else {
		write_cmos_sensor8(0x550F,0xBC);
		}
write_cmos_sensor8(0x5780,0x66);
write_cmos_sensor8(0x578d,0x40);
write_cmos_sensor8(0x57aa,0xeb);

#endif
	LOG_INF("<< %s()\n", __func__);
	LOG_INF("0X800D = 0x%x, 0X550f = 0x%x \n", read_cmos_sensor(0x800D),read_cmos_sensor(0x550F));
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_w1sc820cswidely[] = {

	0x0100,0x00,
	0x301f,0x01,
	0x3200,0x00,
	0x3201,0x00,
	0x3202,0x00,
	0x3203,0x00,
	0x3204,0x0c,
	0x3206,0x09,
	0x3207,0x9f,
	0x3208,0x0c,
	0x3209,0xc0,
	0x320a,0x09,
	0x320b,0x90,
	0x3210,0x00,
	0x3212,0x00,
	0x3213,0x08,
	0x3215,0x11,
	0x3220,0x00,
	0x5000,0x0e,
	0x5900,0x01,
	0x5901,0x00,

};
#endif

static void preview_setting(void)
{

#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_preview_w1sc820cswidely,
         sizeof(addr_data_pair_preview_w1sc820cswidely) /
         sizeof(kal_uint16));

#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);
	write_cmos_sensor8(0x301f,0x04);
	write_cmos_sensor8(0x3200,0x00);
	write_cmos_sensor8(0x3201,0x00);
	write_cmos_sensor8(0x3202,0x00);
	write_cmos_sensor8(0x3203,0x00);
	write_cmos_sensor8(0x3204,0x0c);
	write_cmos_sensor8(0x3206,0x09);
	write_cmos_sensor8(0x3207,0x9f);
	write_cmos_sensor8(0x3208,0x06);
	write_cmos_sensor8(0x3209,0x60);
	write_cmos_sensor8(0x320a,0x04);
	write_cmos_sensor8(0x320b,0xc8);
	write_cmos_sensor8(0x3210,0x00);
	write_cmos_sensor8(0x3212,0x00);
	write_cmos_sensor8(0x3213,0x04);
	write_cmos_sensor8(0x3215,0x31);
	write_cmos_sensor8(0x3220,0x01);
	write_cmos_sensor8(0x5000,0x4e);
	write_cmos_sensor8(0x5900,0xf1);
	write_cmos_sensor8(0x5901,0x04);


#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_fps_w1sc820cswidely[] = {

	0x0100,0x00,
	0x301f,0x01,
	0x3200,0x00,
	0x3201,0x00,
	0x3202,0x00,
	0x3203,0x00,
	0x3204,0x0c,
	0x3206,0x09,
	0x3207,0x9f,
	0x3208,0x0c,
	0x3209,0xc0,
	0x320a,0x09,
	0x320b,0x90,
	0x3210,0x00,
	0x3212,0x00,
	0x3213,0x08,
	0x3215,0x11,
	0x3220,0x00,
	0x5000,0x0e,
	0x5900,0x01,
	0x5901,0x00,
};
#endif

static void capture_setting(kal_uint16 currefps)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_capture_fps_w1sc820cswidely,
         sizeof(addr_data_pair_capture_fps_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/
	write_cmos_sensor8(0x0100,0x00);
	write_cmos_sensor8(0x301f,0x01);
	write_cmos_sensor8(0x3200,0x00);
	write_cmos_sensor8(0x3201,0x00);
	write_cmos_sensor8(0x3202,0x00);
	write_cmos_sensor8(0x3203,0x00);
	write_cmos_sensor8(0x3204,0x0c);
	write_cmos_sensor8(0x3206,0x09);
	write_cmos_sensor8(0x3207,0x9f);
	write_cmos_sensor8(0x3208,0x0c);
	write_cmos_sensor8(0x3209,0xc0);
	write_cmos_sensor8(0x320a,0x09);
	write_cmos_sensor8(0x320b,0x90);
	write_cmos_sensor8(0x3210,0x00);
	write_cmos_sensor8(0x3212,0x00);
	write_cmos_sensor8(0x3213,0x08);
	write_cmos_sensor8(0x3215,0x11);
	write_cmos_sensor8(0x3220,0x00);
	write_cmos_sensor8(0x5000,0x0e);
	write_cmos_sensor8(0x5900,0x01);
	write_cmos_sensor8(0x5901,0x00);


#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_normal_video_w1sc820cswidely[] = {
	0x301f,0x08,
	0x3200,0x00,
	0x3201,0x00,
	0x3202,0x01,
	0x3203,0x30,
	0x3204,0x0c,
	0x3205,0xc7,
	0x3206,0x08,
	0x3207,0x6f,
	0x3208,0x0c,
	0x3209,0xc0,
	0x320a,0x07,
	0x320b,0x30,
	0x3210,0x00,
	0x3211,0x04,
	0x3212,0x00,
	0x3213,0x08,
	0x3215,0x11,
	0x3220,0x00,
	0x5000,0x0e,
	0x5900,0x01,
	0x5901,0x00,
	0x320e,0x09,
	0x320f,0xc4,
};
#endif

static void normal_video_setting(void)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_normal_video_w1sc820cswidely,
         sizeof(addr_data_pair_normal_video_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);
	write_cmos_sensor8(0x301f,0x01);
	write_cmos_sensor8(0x3200,0x00);
	write_cmos_sensor8(0x3201,0x00);
	write_cmos_sensor8(0x3202,0x02);
	write_cmos_sensor8(0x3203,0xac);
	write_cmos_sensor8(0x3204,0x0c);
	write_cmos_sensor8(0x3206,0x06);
	write_cmos_sensor8(0x3207,0xf3);
	write_cmos_sensor8(0x3208,0x07);
	write_cmos_sensor8(0x3209,0x80);
	write_cmos_sensor8(0x320a,0x04);
	write_cmos_sensor8(0x320b,0x38);
	write_cmos_sensor8(0x3210,0x02);
	write_cmos_sensor8(0x3212,0x00);
	write_cmos_sensor8(0x3213,0x08);
	write_cmos_sensor8(0x3215,0x11);
	write_cmos_sensor8(0x3220,0x00);
	write_cmos_sensor8(0x5000,0x0e);
	write_cmos_sensor8(0x5900,0x01);
	write_cmos_sensor8(0x5901,0x00);

#endif
}

#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_w1sc820cswidely[] = {
	0x0100,0x00,
	0x301f,0x04,
	0x3200,0x00,
	0x3201,0x00,
	0x3202,0x00,
	0x3203,0x00,
	0x3204,0x0c,
	0x3206,0x09,
	0x3207,0x9f,
	0x3208,0x06,
	0x3209,0x60,
	0x320a,0x04,
	0x320b,0xc8,
	0x3210,0x00,
	0x3212,0x00,
	0x3213,0x04,
	0x3215,0x31,
	0x3220,0x01,
	0x5000,0x4e,
	0x5900,0xf1,
	0x5901,0x04,
};
#endif

static void hs_video_setting(void)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_hs_video_w1sc820cswidely,
         sizeof(addr_data_pair_hs_video_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);

#endif
}

#if 0
#if MULTI_WRITE
kal_uint16 addr_data_pair_slim_video_w1sc820cswidely[] = {
	0x0100,0x00,
};
#endif

static void slim_video_setting(void)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_slim_video_w1sc820cswidely,
         sizeof(addr_data_pair_slim_video_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);

#endif
}
#endif

#if MULTI_WRITE
kal_uint16 addr_data_pair_custom1_w1sc820cswidely[] = {
	0x301f,0x08,
	0x3200,0x00,
	0x3201,0x00,
	0x3202,0x01,
	0x3203,0x32,
	0x3204,0x0c,
	0x3206,0x08,
	0x3207,0x6d,
	0x3208,0x0c,
	0x3209,0xc0,
	0x320a,0x07,
	0x320b,0x2c,
	0x3210,0x00,
	0x3212,0x00,
	0x3213,0x08,
	0x3215,0x11,
	0x3220,0x00,
	0x5000,0x0e,
	0x5900,0x01,
	0x5901,0x00,
	0x320e,0x0c,
	0x320f,0x35,
};
#endif

static void custom1_setting(void)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_custom1_w1sc820cswidely,
         sizeof(addr_data_pair_custom1_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);

#endif
}
#if 0
#if MULTI_WRITE
kal_uint16 addr_data_pair_custom2_w1sc820cswidely[] = {
	0x0100,0x00,

};
#endif

static void custom2_setting(void)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_custom2_w1sc820cswidely,
         sizeof(addr_data_pair_custom2_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);

#endif
}
#endif

#if MULTI_WRITE
kal_uint16 addr_data_pair_custom3_w1sc820cswidely[] = {
	0x0100,0x00,

};
#endif

static void custom3_setting(void)
{
#if MULTI_WRITE
     w1sc820cswidely_table_write_cmos_sensor(
         addr_data_pair_custom3_w1sc820cswidely,
         sizeof(addr_data_pair_custom3_w1sc820cswidely) /
         sizeof(kal_uint16));
#else
	/*V02P08_20210628*/

	write_cmos_sensor8(0x0100,0x00);

#endif
}


static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable) {
		write_cmos_sensor8(0x4501, 0xac);
		write_cmos_sensor8(0x3902, 0x85);
		write_cmos_sensor8(0x3908, 0x00);
		write_cmos_sensor8(0x3909, 0xff);
		write_cmos_sensor8(0x390a, 0xff);
		write_cmos_sensor8(0x391d, 0x18);
	}
	else {
		write_cmos_sensor8(0x0100, 0x00);
		write_cmos_sensor8(0x4501, 0xb4);
		write_cmos_sensor8(0x3902, 0xc5);
		write_cmos_sensor8(0x3908, 0x41);
		write_cmos_sensor8(0x3909, 0x00);
		write_cmos_sensor8(0x390a, 0x00);
		write_cmos_sensor8(0x391d, 0x19);
		write_cmos_sensor8(0x0100, 0x01);
			mDELAY(10);
		//write_cmos_sensor8(0x302d, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static void read_cmos_eeprom_table(kal_uint16 addr, kal_uint8 *table, kal_uint32 size)
{
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd, 2, (u8 *)table, size, 0xA0);
}

static kal_uint16 read_otp_info(kal_uint8 *data)
{
	kal_uint16 addr = 0x00;
	kal_uint16 left_size = 0;

	left_size = 0x2000;
	while(left_size > 0){
		if (left_size >= 1024) {
			read_cmos_eeprom_table(addr,data, 1024);
			left_size = left_size - 1024;
		}else {
			read_cmos_eeprom_table(addr,data, left_size);
			break;
		}
		addr = addr + 1024;
		data = data + 1024;
	}
	return 0;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	LOG_INF("[get_imgsensor_id] ");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
				*sensor_id =return_sensor_id();
				LOG_INF("sensor id: 0x%x\n",*sensor_id);
				if (*sensor_id == imgsensor_info.sensor_id) {
					LOG_INF("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
					if(otp_data[0] == 0) {
						read_otp_info(otp_data);
					} else {
						pr_err("otp data has already read");
					}
					if(deviceInfo_register_value == 0x00){
					    //Eeprom_DataInit(1, SC820CS_TRULY_SENSOR_ID);
					    deviceInfo_register_value = 0x01;
					}
					return ERROR_NONE;
				}
				LOG_INF("get_imgsensor_id Read sensor id fail, i2c write id: 0x%x,sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				retry--;
			} while(retry > 0);
			i++;
			retry = 2;
}
	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	LOG_INF("[open]: PLATFORM:MT6750,MIPI 24LANE\n");
	LOG_INF("preview 1296*972@30fps,360Mbps/lane; capture 2593*1944@30fps,880Mbps/lane\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("open:Read sensor id fail open i2c write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}	/*	open  */
static kal_uint32 close(void)
{
	LOG_INF("close E");
	/*No Need to implement this function*/
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
//	capture_setting(300);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_TRUE;
    spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
//	slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_TRUE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
//	hs_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom1_setting();

    return ERROR_NONE;
}   /* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();

    return ERROR_NONE;
}   /* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    custom3_setting();

    return ERROR_NONE;
}	/* custom3 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

    sensor_resolution->SensorCustom1Width =  imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height =  imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width =  imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height =  imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width =  imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height =  imgsensor_info.custom3.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            break;

        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
            break;

        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
            break;

        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

			LOG_INF("preview\n");
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//case MSDK_SCENARIO_ID_CAMERA_ZSD:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			custom1(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			custom2(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			custom3(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */


static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);

	if ((framerate == 30) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 15) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = 10 * framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);
	set_dummy();
	return ERROR_NONE;
}


static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
			if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10	/ imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength): 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10	/ imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength): 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength	+ imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength): 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
			if (imgsensor.frame_length > imgsensor.shutter)
            set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

	if (enable){
		write_cmos_sensor8(0x0100, 0x01); // stream on
		mdelay(10);

	}else
		write_cmos_sensor8(0x0100, 0x00); // stream off

	   mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data =
		(unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
	    *feature_return_para_16++ = imgsensor.line_length;
	    *feature_return_para_16 = imgsensor.frame_length;
	    *feature_para_len = 4;
	break;
	// case SENSOR_FEATURE_GET_SENSOR_OTP_ALL:
	// {
	// 	memcpy(feature_return_para_32, (UINT32 *)otp_data, sizeof(otp_data));
	// 	break;
	// }
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
	    *feature_return_para_32 = imgsensor.pclk;
	    *feature_para_len = 4;
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
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (*(feature_data + 2))/* HDR on */
				*feature_return_para_32 = 1;/*BINNING_NONE*/
			else
				*feature_return_para_32 = 1;/*BINNING_AVERAGED*/
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d\n",
			*feature_return_para_32);

		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		* 1, if driver support new sw frame sync
		* set_shutter_frame_length() support third para auto_extend_en
		*/
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
	break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr,
			sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data,
			*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("current fps: %d\n", imgsensor.current_fps);
		break;
	case SENSOR_FEATURE_SET_HDR:
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("ihdr enable: %d\n", imgsensor.ihdr_en);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId: %d\n", (UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE = %d, SE = %d, Gain = %d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
			(UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));

	break;
	//+bug 558061, zhanglinfeng.wt, modify, 2020/06/19, modify codes for mipi rate is 0
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
    		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.cap.pclk;
    			break;
    		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.normal_video.pclk;
    			break;
    		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.hs_video.pclk;
    			break;
    		case MSDK_SCENARIO_ID_SLIM_VIDEO:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.slim_video.pclk;
    			break;
    		//+bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
    		case MSDK_SCENARIO_ID_CUSTOM1:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.custom1.pclk;
    			break;
    		case MSDK_SCENARIO_ID_CUSTOM2:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.custom2.pclk;
    			break;
    		case MSDK_SCENARIO_ID_CUSTOM3:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.custom3.pclk;
    			break;

    		//-bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
    		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    		default:
    			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
    				= imgsensor_info.pre.pclk;
    			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		//+bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		//-bug 558061, shaozhuchao.wt, modify, 2020/07/23, modify codes for main camera hw remosaic
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	//-bug 558061, zhanglinfeng.wt, modify, 2020/06/19, modify codes for mipi rate is 0
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        kal_uint32 rate;

        switch (*feature_data) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            rate = imgsensor_info.cap.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            rate = imgsensor_info.normal_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            rate = imgsensor_info.hs_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            rate = imgsensor_info.slim_video.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            rate = imgsensor_info.custom1.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            rate = imgsensor_info.custom2.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            rate = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            rate = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
        LOG_INF("hi556 SENSOR_FEATURE_GET_MIPI_PIXEL_RATE");
        *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
    }
    break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = 0;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		streaming_control(KAL_FALSE);
	break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
	break;
	default:
	break;
	}
    return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 SC820CS_MIPI_RAW_BLUEY_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
};	/*	SC820CS_MIPI_RAW_SensorInit	*/

