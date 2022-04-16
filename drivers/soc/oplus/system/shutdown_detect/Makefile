# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT) += shutdown_detect.o
else
ifeq ($(CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT), y)
obj-$(CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT) += shutdown_detect.o
endif
ifeq ($(CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT), m)
obj-$(CONFIG_OPLUS_FEATURE_SHUTDOWN_DETECT) += gki/shutdown_detect.o
endif
endif

