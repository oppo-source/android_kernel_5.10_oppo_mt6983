/************************************************************************************     
 ** File: - android\kernel\arch\arm\mach-msm\include\mach\oppo_boot.h                      
 ** VENDOR_EDIT
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd                                  
 **      
 ** Description:  
 **     change define of boot_mode here for other place to use it                          
 ** Version: 1.0                                                                           
 ** --------------------------- Revision History: --------------------------------         
 **  <author>    <data>          <desc>
 ************************************************************************************/     
#ifndef _OPPO_BOOT_TYPE_H    
#define _OPPO_BOOT_TYPE_H
enum{
        MSM_BOOT_MODE__NORMAL,
        MSM_BOOT_MODE__FASTBOOT,
        MSM_BOOT_MODE__RECOVERY,
        MSM_BOOT_MODE__FACTORY,
        MSM_BOOT_MODE__RF,
        MSM_BOOT_MODE__WLAN,
        MSM_BOOT_MODE__MOS,
        MSM_BOOT_MODE__CHARGE,
        MSM_BOOT_MODE__SILENCE,
        MSM_BOOT_MODE__SAU,
        MSM_BOOT_MODE__AGING,
        MSM_BOOT_MODE__SAFE = 999,
        //TODO
        MSM_BOOT_MODE__MAX,
};
#endif
