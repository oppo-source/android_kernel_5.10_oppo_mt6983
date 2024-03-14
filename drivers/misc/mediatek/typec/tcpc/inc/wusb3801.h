// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2018-2020 Oplus. All rights reserved.
*/

#ifndef __LINUX_WUSB3801_H
#define __LINUX_WUSB3801_H

#undef __CONST_FFS
#define __CONST_FFS(_x) \
	((_x)&0x0F ? ((_x)&0x03 ? ((_x)&0x01 ? 0 : 1) : ((_x)&0x04 ? 2 : 3)) : ((_x)&0x30 ? ((_x)&0x10 ? 4 : 5) : ((_x)&0x40 ? 6 : 7)))

#undef FFS
#define FFS(_x) \
	((_x) ? __CONST_FFS(_x) : 0)

#undef BITS
#define BITS(_end, _start) \
	((BIT(_end) - BIT(_start)) + BIT(_end))

#undef __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
	(((_byte) & (_mask)) >> (_shift))

#undef BITS_GET
#define BITS_GET(_byte, _bit) \
	__BITS_GET(_byte, _bit, FFS(_bit))

#undef __BITS_SET
#define __BITS_SET(_byte, _mask, _shift, _val) \
	(((_byte) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef BITS_SET
#define BITS_SET(_byte, _bit, _val) \
	__BITS_SET(_byte, _bit, FFS(_bit), _val)

#undef BITS_MATCH
#define BITS_MATCH(_byte, _bit) \
	(((_byte) & (_bit)) == (_bit))

/* Register Map */

#define WUSB3801_REG_VERSION_ID 0x01
#define WUSB3801_REG_CONTROL0 0x02
#define WUSB3801_REG_INTERRUPT 0x03
#define WUSB3801_REG_STATUS 0x04
#define WUSB3801_REG_CONTROL1 0x05
#define WUSB3801_REG_TEST0 0x06
#define WUSB3801_REG_TEST_01 0x07
#define WUSB3801_REG_TEST_02 0x08
#define WUSB3801_REG_TEST_03 0x09
#define WUSB3801_REG_TEST_04 0x0A
#define WUSB3801_REG_TEST_05 0x0B
#define WUSB3801_REG_TEST_06 0x0C
#define WUSB3801_REG_TEST_07 0x0D
#define WUSB3801_REG_TEST_08 0x0E
#define WUSB3801_REG_TEST_09 0x0F
#define WUSB3801_REG_TEST_0A 0x10
#define WUSB3801_REG_TEST_0B 0x11
#define WUSB3801_REG_TEST_0C 0x12
#define WUSB3801_REG_TEST_0D 0x13
#define WUSB3801_REG_TEST_0E 0x14
#define WUSB3801_REG_TEST_0F 0x15
#define WUSB3801_REG_TEST_10 0x16
#define WUSB3801_REG_TEST_11 0x17
#define WUSB3801_REG_TEST_12 0x18

#define WUSB3801_SLAVE_ADDR0 0xc0
#define WUSB3801_SLAVE_ADDR1 0xd0

/*Available modes*/
#define WUSB3801_DRP_ACC (BIT_REG_CTRL0_RLE_DRP)
#define WUSB3801_DRP (BIT_REG_CTRL0_RLE_DRP | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SNK_ACC (BIT_REG_CTRL0_RLE_SNK)
#define WUSB3801_SNK (BIT_REG_CTRL0_RLE_SNK | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SRC_ACC (BIT_REG_CTRL0_RLE_SRC)
#define WUSB3801_SRC (BIT_REG_CTRL0_RLE_SRC | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_DRP_PREFER_SRC_ACC (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SRC (WUSB3801_DRP | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SNK_ACC (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SNK)
#define WUSB3801_DRP_PREFER_SNK (WUSB3801_DRP | BIT_REG_CTRL0_TRY_SNK)

/*TODO: redefine your prefer role here*/
#define WUSB3801_INIT_MODE (WUSB3801_DRP_PREFER_SNK_ACC)

/*Registers relevant values*/
#define WUSB3801_VENDOR_ID 0x06

/*Switch to enable/disable feature of specified Registers*/
#define BIT_REG_CTRL0_DIS_ACC (0x01 << 7)
#define BIT_REG_CTRL0_TRY_SRC (0x02 << 5)
#define BIT_REG_CTRL0_TRY_SNK (0x01 << 5)
#define BIT_REG_CTRL0_CUR_DEF (0x00 << 3)
#define BIT_REG_CTRL0_CUR_1P5 (0x01 << 3)
#define BIT_REG_CTRL0_CUR_3P0 (0x02 << 3)
#define BIT_REG_CTRL0_RLE_SNK (0x00 << 1)
#define BIT_REG_CTRL0_RLE_SRC (0x01 << 1)
#define BIT_REG_CTRL0_RLE_DRP (0x02 << 1)
#define BIT_REG_CTRL0_INT_MSK (0x01 << 0)

#define BIT_REG_STATUS_VBUS (0x01 << 7)
#define BIT_REG_STATUS_STANDBY (0x00 << 5)
#define BIT_REG_STATUS_CUR_DEF (0x01 << 5)
#define BIT_REG_STATUS_CUR_MID (0x02 << 5)
#define BIT_REG_STATUS_CUR_HIGH (0x03 << 5)

#define BIT_REG_STATUS_ATC_STB (0x00 << 1)
#define BIT_REG_STATUS_ATC_SNK (0x01 << 1)
#define BIT_REG_STATUS_ATC_SRC (0x02 << 1)
#define BIT_REG_STATUS_ATC_ACC (0x03 << 1)
#define BIT_REG_STATUS_ATC_DACC (0x04 << 1)

#define BIT_REG_STATUS_PLR_STB (0x00 << 0)
#define BIT_REG_STATUS_PLR_CC1 (0x01 << 0)
#define BIT_REG_STATUS_PLR_CC2 (0x02 << 0)
#define BIT_REG_STATUS_PLR_BOTH (0x03 << 0)

#define BIT_REG_CTRL1_SW02_DIN (0x01 << 4)
#define BIT_REG_CTRL1_SW02_EN (0x01 << 3)
#define BIT_REG_CTRL1_SW01_DIN (0x01 << 2)
#define BIT_REG_CTRL1_SW01_EN (0x01 << 1)
#define BIT_REG_CTRL1_SM_RST (0x01 << 0)

#define BIT_REG_TEST02_FORCE_ERR_RCY (0x01)

#define WUSB3801_WAIT_VBUS 0x40
/*Fixed duty cycle period. 40ms:40ms*/
#define WUSB3801_TGL_40MS 0
#define WUSB3801_HOST_DEFAULT 0
#define WUSB3801_HOST_1500MA 1
#define WUSB3801_HOST_3000MA 2
#define WUSB3801_INT_ENABLE 0x00
#define WUSB3801_INT_DISABLE 0x01
#define WUSB3801_DISABLED 0x0A
#define WUSB3801_ERR_REC 0x01
#define WUSB3801_VBUS_OK 0x80

#define WUSB3801_SNK_0MA (0x00 << 5)
#define WUSB3801_SNK_DEFAULT (0x01 << 5)
#define WUSB3801_SNK_1500MA (0x02 << 5)
#define WUSB3801_SNK_3000MA (0x03 << 5)
#define WUSB3801_ATTACH 0x1C

#define WUSB3801_TYPE_INVALID (0x00)
#define WUSB3801_TYPE_SNK (0x01 << 2)
#define WUSB3801_TYPE_SRC (0x02 << 2)
#define WUSB3801_TYPE_AUD_ACC (0x03 << 2)
#define WUSB3801_TYPE_DBG_ACC (0x04 << 2)

#define WUSB3801_INT_DETACH (0x01 << 1)
#define WUSB3801_INT_ATTACH (0x01 << 0)

#define WUSB3801_REV20 0x02

/* Masks for Read-Modified-Write operations*/
#define WUSB3801_HOST_CUR_MASK 0x18 /*Host current for IIC*/
#define WUSB3801_INT_MASK 0x01
#define WUSB3801_BCLVL_MASK 0x60
#define WUSB3801_TYPE_MASK 0x1C
#define WUSB3801_MODE_MASK 0xE6 /*Roles relevant bits*/
#define WUSB3801_INT_STS_MASK 0x03
#define WUSB3801_FORCE_ERR_RCY_MASK 0x80 /*Force Error recovery*/
#define WUSB3801_ROLE_MASK 0x06
#define WUSB3801_VENDOR_ID_MASK 0x07
#define WUSB3801_VERSION_ID_MASK 0xF8
#define WUSB3801_VENDOR_SUB_ID_MASK 0xA0
#define WUSB3801_POLARITY_CC_MASK 0x03
#define WUSB3801_CC_STS_MASK 0x03

/* WUSB3801 STATES MACHINES */
#define WUSB3801_STATE_DISABLED 0x00
#define WUSB3801_STATE_ERROR_RECOVERY 0x01
#define WUSB3801_STATE_UNATTACHED_SNK 0x02
#define WUSB3801_STATE_UNATTACHED_SRC 0x03
#define WUSB3801_STATE_ATTACHWAIT_SNK 0x04
#define WUSB3801_STATE_ATTACHWAIT_SRC 0x05
#define WUSB3801_STATE_ATTACHED_SNK 0x06
#define WUSB3801_STATE_ATTACHED_SRC 0x07
#define WUSB3801_STATE_AUDIO_ACCESSORY 0x08
#define WUSB3801_STATE_DEBUG_ACCESSORY 0x09
#define WUSB3801_STATE_TRY_SNK 0x0A
#define WUSB3801_STATE_TRYWAIT_SRC 0x0B
#define WUSB3801_STATE_TRY_SRC 0x0C
#define WUSB3801_STATE_TRYWAIT_SNK 0x0D

#define WUSB3801_CC2    0x02
#define WUSB3801_CC1	0x01
#define WUSB3801_CC2_CONNECTED 1
#define WUSB3801_CC1_CONNECTED 0

/*TEST02_DISABLE_RD_RP */
#define TEST02_DIS_RD_RP 0x01
#define TEST02_RD_RP_DIS_SHIFT 7

/* wake lock timeout in ms */
#define WUSB3801_WAKE_LOCK_TIMEOUT 1000
/*1.5 Seconds timeout for force detection*/
#define ROLE_SWITCH_TIMEOUT 1500
#define DEBOUNCE_TIME_OUT 50

#if defined(__MEDIATEK_PLATFORM__)

enum dual_role_supported_modes {
	DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP = 0,
	DUAL_ROLE_SUPPORTED_MODES_DFP,
	DUAL_ROLE_SUPPORTED_MODES_UFP,
	/*The following should be the last element*/
	DUAL_ROLE_PROP_SUPPORTED_MODES_TOTAL,
};

enum {
	DUAL_ROLE_PROP_MODE_UFP = 0,
	DUAL_ROLE_PROP_MODE_DFP,
	DUAL_ROLE_PROP_MODE_NONE,
	/*The following should be the last element*/
	DUAL_ROLE_PROP_MODE_TOTAL,
};

enum {
	DUAL_ROLE_PROP_PR_SRC = 0,
	DUAL_ROLE_PROP_PR_SNK,
	DUAL_ROLE_PROP_PR_NONE,
	/*The following should be the last element*/
	DUAL_ROLE_PROP_PR_TOTAL,
};

enum {
	DUAL_ROLE_PROP_DR_HOST = 0,
	DUAL_ROLE_PROP_DR_DEVICE,
	DUAL_ROLE_PROP_DR_NONE,
	/*The following should be the last element*/
	DUAL_ROLE_PROP_DR_TOTAL,
};

enum {
	DUAL_ROLE_PROP_VCONN_SUPPLY_NO = 0,
	DUAL_ROLE_PROP_VCONN_SUPPLY_YES,
	/*The following should be the last element*/
	DUAL_ROLE_PROP_VCONN_SUPPLY_TOTAL,
};

enum dual_role_property {
	DUAL_ROLE_PROP_SUPPORTED_MODES = 0,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
};
#endif

#endif   /* __LINUX_WUSB3801_H */
