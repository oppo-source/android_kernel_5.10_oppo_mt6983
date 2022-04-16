# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += black_screen_check.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += bright_screen_check.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += theia_kevent_kernel.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += powerkey_monitor.o
else
ifeq ($(CONFIG_OPLUS_FEATURE_THEIA), y)
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += black_screen_check.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += bright_screen_check.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += theia_kevent_kernel.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += powerkey_monitor.o
endif
ifeq ($(CONFIG_OPLUS_FEATURE_THEIA), m)
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += oplus_BScheck.o
obj-$(CONFIG_OPLUS_FEATURE_THEIA) += oplus_brightscreen_check.o
endif
endif
