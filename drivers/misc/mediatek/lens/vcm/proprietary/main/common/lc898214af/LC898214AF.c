// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */



/*
 * LC898214AF voice coil motor driver
 *
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"

#define AF_DRVNAME "LC898214AF_DRV"
#define AF_I2C_SLAVE_ADDR 0xE4
#define EEPROM_I2C_SLAVE_ADDR 0xA0

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4CurrPosition;

#define Min_Pos 0
#define Max_Pos 1023

static signed short Hall_Max = 0x4500;
static signed short Hall_Min = 0xBB00;

static int s4EEPROM_ReadReg(u16 addr, u16 *data)
{
	u8 u8data[2];
	u8 pu_send_cmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};

	g_pstAF_I2Cclient->addr = (EEPROM_I2C_SLAVE_ADDR) >> 1;
	if (i2c_master_send(g_pstAF_I2Cclient, pu_send_cmd, 2) < 0) {
		LOG_INF("read I2C send failed!!\n");
		return -1;
	}
	if (i2c_master_recv(g_pstAF_I2Cclient, u8data, 2) < 0) {
		LOG_INF("EEPROM_ReadReg failed!!\n");
		return -1;
	}
	LOG_INF("u8data[0] = 0x%x\n", u8data[0]);
	LOG_INF("u8data[1] = 0x%x\n", u8data[1]);

	*data = u8data[1] << 8 | u8data[0];
	LOG_INF("EEPROM 0x%x, 0x%x\n", addr, *data);

	return 0;
}

static int s4AF_ReadReg(u8 a_uAddr, u8 *a_uData)
{
	g_pstAF_I2Cclient->addr = (AF_I2C_SLAVE_ADDR) >> 1;

	if (i2c_master_send(g_pstAF_I2Cclient, &a_uAddr, 1) < 0) {
		LOG_INF("ReadI2C send failed!!\n");
		return -1;
	}

	if (i2c_master_recv(g_pstAF_I2Cclient, a_uData, 1) < 0) {
		LOG_INF("ReadI2C recv failed!!\n");
		return -1;
	}

	/* LOG_INF("ReadI2C 0x%x, 0x%x\n", a_uAddr, *a_uData); */

	return 0;
}

static int s4AF_WriteReg(u8 a_uLength, u8 a_uAddr, u16 a_u2Data)
{
	u8 puSendCmd[2] = {a_uAddr, (u8)(a_u2Data & 0xFF)};
	u8 puSendCmd2[3] = {a_uAddr, (u8)((a_u2Data >> 8) & 0xFF),
			    (u8)(a_u2Data & 0xFF)};

	g_pstAF_I2Cclient->addr = (AF_I2C_SLAVE_ADDR) >> 1;

	if (a_uLength == 0) {
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2) < 0) {
			LOG_INF("WriteI2C failed!!\n");
			return -1;
		}
	} else if (a_uLength == 1) {
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 3) < 0) {
			LOG_INF("WriteI2C 2 failed!!\n");
			return -1;
		}
	}

	return 0;
}

static unsigned short AF_convert(int position)
{
	return (((Max_Pos - position) * (unsigned short)(Hall_Max - Hall_Min) /
		 (Max_Pos - Min_Pos)) + Hall_Min) & 0xFFFF;
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
			 sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

/* initAF include driver initialization and standby mode */
static int initAF(void)
{
	LOG_INF("+\n");

	if (*g_pAF_Opened == 1) {

		unsigned char Temp;
		unsigned short Data;
		unsigned short Cnt;

		s4EEPROM_ReadReg(0x0F63, &Data);

		if (Data > 0 && Data < 0xFFFF)
			Hall_Max = Data;

		s4EEPROM_ReadReg(0x0F65, &Data);

		if (Data > 0 && Data < 0xFFFF)
			Hall_Min = Data;

		s4AF_ReadReg(0xF0, &Temp);

		if (Temp != 0x42) {
			LOG_INF("Check Read F0h fail\n");
			return -1;
		}

		s4AF_WriteReg(0, 0xE0, 0x1);

		Cnt = 0;

		while (1) {
			msleep(20);

			s4AF_ReadReg(0xE0, &Temp);

			Cnt++;

			if (Temp == 0 || Cnt > 3)
				break;
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("-\n");

	return 0;
}

/* moveAF only use to control moving the motor */
static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if (s4AF_WriteReg(1, 0xA0, (unsigned short)AF_convert(
					   a_u4Position)) == 0) {
		g_u4CurrPosition = a_u4Position;
		ret = 0;
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long LC898214AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		      unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue =
			getAFInfo((__user struct stAF_MotorInfo *)(a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int LC898214AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		LOG_INF("Close\n");
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int LC898214AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
			    spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	initAF();

	return 1;
}

int LC898214AF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
