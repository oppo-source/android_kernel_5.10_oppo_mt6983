/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifndef __SSPM_IPI_MBOX_LAYOUT_H__
#define __SSPM_IPI_MBOX_LAYOUT_H__

#define IPI_MBOX_TOTAL  4
#define IPI_MBOX0_64D   0
#define IPI_MBOX1_64D   0
#define IPI_MBOX2_64D   0
#define IPI_MBOX3_64D   0
#define IPI_MBOX4_64D   0
#define IPI_MBOX_MODE   ((IPI_MBOX4_64D<<4)|(IPI_MBOX3_64D<<3)| \
			(IPI_MBOX2_64D<<2)|(IPI_MBOX1_64D<<1)| \
			IPI_MBOX0_64D)

#define IPI_MBOX0_SLOTS ((IPI_MBOX0_64D+1)*32)
#define IPI_MBOX1_SLOTS ((IPI_MBOX1_64D+1)*32)
#define IPI_MBOX2_SLOTS ((IPI_MBOX2_64D+1)*32)
#define IPI_MBOX3_SLOTS ((IPI_MBOX3_64D+1)*32)
#define IPI_MBOX4_SLOTS ((IPI_MBOX4_64D+1)*32)


/* definition of slot size for send PINs */
#define PINS_SIZE_PLATFORM       3  /* the following will use mbox 0 */
#define PINS_SIZE_CPU_DVFS       4
#define PINS_SIZE_QOS            4
#define PINS_SIZE_TST1           4
#define PINS_SIZE_FHCTL          9
#define PINS_SIZE_SMI            3
/* ============================================================ */
#define PINS_SIZE_MCDI           2  /* the following will use mbox 1 */
#define PINS_SIZE_SPM_SUSPEND    8
#define PINS_SIZE_PMIC           5
#define PINS_SIZE_PPM            7
#define PINS_SIZE_THERMAL        4
#define PINS_SIZE_UPOWER         4
#define PINS_SIZE_CM             2
/* ============================================================ */

/* definition of slot offset for PINs */
#define PINS_OFFSET_PLATFORM     0  /* the following will use mbox 0 */
#define PINS_OFFSET_CPU_DVFS     (PINS_OFFSET_PLATFORM + PINS_SIZE_PLATFORM)
#define PINS_OFFSET_QOS          (PINS_OFFSET_CPU_DVFS + PINS_SIZE_CPU_DVFS)
#define PINS_OFFSET_TST1         (PINS_OFFSET_QOS + PINS_SIZE_QOS)
#define PINS_OFFSET_FHCTL        (PINS_OFFSET_TST1 + PINS_SIZE_TST1)
#define PINS_OFFSET_SMI          (PINS_OFFSET_FHCTL + PINS_SIZE_FHCTL)
#define PINS_MBOX0_USED          (PINS_OFFSET_SMI + PINS_SIZE_SMI)
#if (PINS_MBOX0_USED > IPI_MBOX0_SLOTS)
#error "MBOX0 cannot hold all pin definitions"
#endif
/* ============================================================ */
#define PINS_OFFSET_MCDI         0  /* the following will use mbox 1 */
#define PINS_OFFSET_SPM_SUSPEND  (PINS_OFFSET_MCDI + PINS_SIZE_MCDI)
#define PINS_OFFSET_PMIC         (PINS_OFFSET_SPM_SUSPEND + \
				PINS_SIZE_SPM_SUSPEND)
#define PINS_OFFSET_PPM          (PINS_OFFSET_PMIC + PINS_SIZE_PMIC)
#define PINS_OFFSET_THERMAL      (PINS_OFFSET_PPM + PINS_SIZE_PPM)
#define PINS_OFFSET_UPOWER       (PINS_OFFSET_THERMAL + PINS_SIZE_THERMAL)
#define PINS_OFFSET_CM           (PINS_OFFSET_UPOWER + PINS_SIZE_UPOWER)
#define PINS_MBOX1_USED          (PINS_OFFSET_CM + PINS_SIZE_CM)
#if (PINS_MBOX1_USED > IPI_MBOX1_SLOTS)
#error "MBOX1 cannot hold all pin definitions"
#endif
/* ============================================================ */

/* definition of slot size for received PINs */
#define PINR_SIZE_PLATFORM       3  /* the following will use mbox 2 */
#define PINR_SIZE_CPU_DVFS       4
#define PINR_SIZE_QOS            4
#define PINR_SIZE_TST1           4
/* definition of slot offset for PINs */
#define PINR_OFFSET_PLATFORM     0  /* the following will use mbox 2 */
#define PINR_OFFSET_CPU_DVFS     (PINR_OFFSET_PLATFORM + PINR_SIZE_PLATFORM)
#define PINR_OFFSET_QOS          (PINR_OFFSET_CPU_DVFS + PINR_SIZE_CPU_DVFS)
#define PINR_OFFSET_TST1         (PINR_OFFSET_QOS + PINR_SIZE_QOS)
#define PINR_MBOX2_USED          (PINR_OFFSET_TST1 + PINR_SIZE_TST1)
#if (PINR_MBOX2_USED > IPI_MBOX2_SLOTS)
#error "MBOX2 cannot hold all pin definitions"
#endif

#define SHAREMBOX_NO_MCDI            3
#define SHAREMBOX_OFFSET_MCDI        0
#define SHAREMBOX_SIZE_MCDI          20
#define SHAREMBOX_OFFSET_TIMESTAMP   (SHAREMBOX_OFFSET_MCDI + \
				SHAREMBOX_SIZE_MCDI)
#define SHAREMBOX_SIZE_TIMESTAMP     6

#endif /* __SSPM_IPI_MBOX_LAYOUT_H__ */
