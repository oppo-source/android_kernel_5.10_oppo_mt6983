/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 */

#ifndef AUDIO_LOG_H
#define AUDIO_LOG_H

#include <linux/printk.h>


#ifdef DEBUG_IPI
#define ipi_dbg(x...) pr_info(x)
#else
#define ipi_dbg(x...)
#endif


#ifndef AUD_LOG_V
#ifdef DEBUG_IPI
#define AUD_LOG_V(x...) pr_debug(x)
#else
#define AUD_LOG_V(x...)
#endif
#endif

#ifndef AUD_LOG_D
#define AUD_LOG_D pr_debug
#endif

#ifndef AUD_LOG_W
#define AUD_LOG_W pr_info
#endif

#ifndef AUD_LOG_E
#define AUD_LOG_E pr_notice
#endif

#endif /* end of AUDIO_LOG_H */

