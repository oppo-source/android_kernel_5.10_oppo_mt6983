# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2018-2020 Oplus. All rights reserved.

#ifdef OPLUS_FEATURE_ZRAM_OPT
obj-$(CONFIG_OPLUS_FEATURE_ZRAM_OPT) 	+= zram_opt/
#endif /* CONFIG_OPLUS_FEATURE_ZRAM_OPT */

#ifdef OPLUS_FEATURE_VIRTUAL_RESERVE_MEMORY
obj-$(CONFIG_VIRTUAL_RESERVE_MEMORY)	+= gloom/
#endif /* OPLUS_FEATURE_VIRTUAL_RESERVE_MEMORY */

#ifdef OPLUS_FEATURE_MEMLEAK_DETECT
obj-$(CONFIG_DUMP_TASKS_MEM)	+= dump_tasks_mem/
obj-$(CONFIG_MEMLEAK_DETECT)	+= memleak_detect/
#endif /* OPLUS_FEATURE_MEMLEAK_DETECT */

#ifdef OPLUS_FEATURE_HYBRIDSWAP_ZRAM
obj-$(CONFIG_HYBRIDSWAP_ZRAM) += hybridswap_zram/
#endif
