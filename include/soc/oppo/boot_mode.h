/************************************************************************************
** File: - android\kernel\arch\arm\mach-msm\include\mach\oppo_boot.h
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**     change define of boot_mode here for other place to use it
** Version: 1.0 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
************************************************************************************/
#ifndef _OPPO_BOOT_H
#define _OPPO_BOOT_H
#include <soc/oppo/boot_mode_types.h>
//#ifdef VENDOR_EDIT
//extern static int get_boot_mode(void);
extern int get_boot_mode(void);
//#endif /* VENDOR_EDIT */
#ifdef VENDOR_EDIT
extern bool qpnp_is_power_off_charging(void);
#endif
#ifdef VENDOR_EDIT
extern bool qpnp_is_charger_reboot(void);
#endif /*VENDOR_EDIT*/
#endif
