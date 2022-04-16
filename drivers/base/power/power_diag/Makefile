# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/devfreq
ccflags-y += -I$(srctree)/kernel/sched
ifeq ($(VERSION).$(PATCHLEVEL), 5.10)
obj-$(CONFIG_OPLUS_FEATURE_POWER_DIAG) += power_diag.o
endif