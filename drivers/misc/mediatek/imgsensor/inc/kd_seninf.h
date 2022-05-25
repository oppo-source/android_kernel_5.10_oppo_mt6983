/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _KD_SENINF_H_
#define _KD_SENINF_H_

#include "kd_imgsensor_define.h"
#include "kd_seninf_define.h"

#define SENINFMAGIC 's'
 /* IOCTRL(inode * ,file * ,cmd ,arg ) */
 /* S means "set through a ptr" */
 /* T means "tell by a arg value" */
 /* G means "get by a ptr" */
 /* Q means "get by return a value" */
 /* X means "switch G and S atomically" */
 /* H means "switch T and Q atomically" */

#define KDSENINFIOC_X_GET_REG_ADDR             \
		_IOWR(SENINFMAGIC, 40, struct KD_SENINF_REG)
#define KDSENINFIOC_X_SET_MCLK_PLL             \
		_IOWR(SENINFMAGIC, 60, struct ACDK_SENSOR_MCLK_STRUCT)
#define KDSENINFIOC_X_GET_ISP_CLK              \
		_IOWR(SENINFMAGIC, 80, u32)
#define KDSENINFIOC_X_GET_CSI_CLK              \
		_IOWR(SENINFMAGIC, 85, u32)
#define KDSENINFIOC_X_SECURE_DUMP              \
		_IO(SENINFMAGIC, 90)
/* Get ISP CLK via MMDVFS*/
#define KDSENINFIOC_DFS_UPDATE \
	_IOWR(SENINFMAGIC, 95, unsigned int)
#define KDSENINFIOC_GET_SUPPORTED_ISP_CLOCKS \
	_IOWR(SENINFMAGIC, 100, struct IMAGESENSOR_GET_SUPPORTED_ISP_CLK)
#define KDSENINFIOC_GET_CUR_ISP_CLOCK \
	_IOWR(SENINFMAGIC, 105, unsigned int)

/* store numbers of cam_mux for swich later */
#define KDSENINFIOC_SET_CAM_MUX_FOR_SWITCH \
	_IOWR(SENINFMAGIC, 110, unsigned int)

#endif
