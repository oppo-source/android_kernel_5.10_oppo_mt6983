/************************************************************************************
** File: - android\kernel\arch\arm\mach-msm\include\mach\oppo_boot.h
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**     change define of boot_mode here for other place to use it
** Version: 1.0 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
**
************************************************************************************/
#ifndef _OPPO_BOOT_H
#define _OPPO_BOOT_H
#include <soc/oppo/boot_mode_types.h>
//extern static int get_boot_mode(void);
extern int get_boot_mode(void);
extern bool qpnp_is_power_off_charging(void);
extern bool qpnp_is_charger_reboot(void);
#endif
