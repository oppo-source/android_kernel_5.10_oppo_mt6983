# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifneq ($(CONFIG_OPLUS_FEATURE_SAUPWK),n)
ccflags-y += -I$(srctree)/drivers/ -I$(srctree)/drivers/input/
endif
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_SAUPWK) += oplus_saupwk.o
else
ifeq ($(CONFIG_OPLUS_FEATURE_SAUPWK), y)
obj-$(CONFIG_OPLUS_FEATURE_SAUPWK) += oplus_saupwk.o
endif
ifeq ($(CONFIG_OPLUS_FEATURE_SAUPWK), m)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
obj-$(CONFIG_OPLUS_FEATURE_SAUPWK) += gki/oplus_saupwk.o
endif
endif
