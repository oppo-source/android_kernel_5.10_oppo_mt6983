oplus_bsp_mm_osvelte-y += proc-memstat.o
oplus_bsp_mm_osvelte-y += logger.o
oplus_bsp_mm_osvelte-y += lowmem-dbg.o
oplus_bsp_mm_osvelte-y += sys-memstat.o
oplus_bsp_mm_osvelte-y += vsprintf-dup.o

ifdef CONFIG_QCOM_KGSL
ccflags-y  += -I$(srctree)/drivers/gpu/msm/
endif

obj-$(CONFIG_OPLUS_FEATURE_MM_OSVELTE) += oplus_bsp_mm_osvelte.o
