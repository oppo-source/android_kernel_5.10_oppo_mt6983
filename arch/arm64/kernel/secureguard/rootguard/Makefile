ifeq ($(TARGET_BUILD_VARIANT),userdebug)
ccflags-y += -DWHITE_LIST_SUPPORT
endif
obj-$(CONFIG_OPLUS_KERNEL_SECURE_GUARD)    +=  oplus_secure_harden.o
obj-$(CONFIG_OPLUS_KERNEL_SECURE_GUARD)	   +=  oplus_security_keventupload.o
oplus_security_keventupload-objs := oplus_kevent_upload.o oplus_secure_debug.o
obj-$(CONFIG_OPLUS_KERNEL_SECURE_GUARD)	 +=  oplus_security_guard.o
oplus_security_guard-objs := oplus_security_init.o oplus_hook.o oplus_root_hook.o oplus_guard_general.o oplus_exec_hook.o oplus_harden_hook.o
