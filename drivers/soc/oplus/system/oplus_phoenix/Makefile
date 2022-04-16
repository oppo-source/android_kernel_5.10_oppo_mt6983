# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += op_bootprof.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += phoenix_base.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += phoenix_dump.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += phoenix_watchdog.o
else
ifeq ($(CONFIG_OPLUS_FEATURE_PHOENIX), y)
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += op_bootprof.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += phoenix_base.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += phoenix_dump.o
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += phoenix_watchdog.o
endif
ifeq ($(CONFIG_OPLUS_FEATURE_PHOENIX), m)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
obj-$(CONFIG_OPLUS_FEATURE_PHOENIX) += oplus_phoenix.o
oplus_phoenix-y := gki/op_bootprof.o gki/phoenix_base.o gki/phoenix_dump.o gki/phoenix_watchdog.o
endif
endif
