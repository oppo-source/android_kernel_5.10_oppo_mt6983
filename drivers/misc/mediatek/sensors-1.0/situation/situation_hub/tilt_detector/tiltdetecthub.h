/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef TILTDETECTHUB_H
#define TILTDETECTHUB_H

#include <linux/ioctl.h>
#include <linux/module.h>

int __init tiltdetecthub_init(void);
void __exit tiltdetecthub_exit(void);

#endif
