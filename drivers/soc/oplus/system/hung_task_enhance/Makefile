# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.
ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
obj-$(CONFIG_OPLUS_FEATURE_HUNG_TASK_ENHANCE) += hung_task_enhance.o
else
ifeq ($(CONFIG_OPLUS_FEATURE_HUNG_TASK_ENHANCE), y)
obj-$(CONFIG_OPLUS_FEATURE_HUNG_TASK_ENHANCE) += hung_task_enhance.o
endif
ifeq ($(CONFIG_OPLUS_FEATURE_HUNG_TASK_ENHANCE), m)
obj-$(CONFIG_OPLUS_FEATURE_HUNG_TASK_ENHANCE) += gki/hung_task_enhance.o
endif
endif
