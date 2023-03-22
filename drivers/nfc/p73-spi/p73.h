/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012-2014 NXP Semiconductors
 *
 */

#define P61_MAGIC 0xEA
#define P61_SET_PWR _IOW(P61_MAGIC, 0x01, long)
#define P61_SET_DBG _IOW(P61_MAGIC, 0x02, long)
#define P61_SET_POLL _IOW(P61_MAGIC, 0x03, long)
/*
 * SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define P61_SET_SPM_PWR    _IOW(P61_MAGIC, 0x04, long)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define P61_GET_SPM_STATUS    _IOR(P61_MAGIC, 0x05, long)

#define P61_SET_THROUGHPUT    _IOW(P61_MAGIC, 0x06, long)
#define P61_GET_ESE_ACCESS    _IOW(P61_MAGIC, 0x07, long)

#define P61_SET_POWER_SCHEME  _IOW(P61_MAGIC, 0x08, long)

#define P61_SET_DWNLD_STATUS    _IOW(P61_MAGIC, 0x09, long)

#define P61_INHIBIT_PWR_CNTRL  _IOW(P61_MAGIC, 0x0A, long)

//#ifdef OPLUS_FEATURE_NFC_ARCH
#define P61_SPI_IOC_SPI_CLOCK_SET  _IOW(P61_MAGIC, 0xFF, long)
//#endif /* OPLUS_FEATURE_NFC_ARCH */

/* SPI can call this IOCTL to perform the eSE COLD_RESET
 * via NFC driver.
 */
#define ESE_PERFORM_COLD_RESET  _IOW(P61_MAGIC, 0x0C, long)

struct p61_spi_platform_data {
    unsigned int irq_gpio;
    unsigned int rst_gpio;
};
