/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __AP_THERMAL_LIMIT_H__
#define __AP_THERMAL_LIMIT_H__

struct apthermolmt_user {
	char *log;
	unsigned int cpu_limit;
	unsigned int vpu_limit;
	unsigned int mdla_limit;
	unsigned int gpu_limit;
	void *ptr;
};

/*
 *	apthermolmt_register_user
 *	@return 0 success, < 0 with errors
 *	@handle ptr to memory
 *	@log ptr to log string
 */
extern
int apthermolmt_register_user
(struct apthermolmt_user *handle, char *log);

extern
int apthermolmt_unregister_user
(struct apthermolmt_user *handle);

/*
 *	@limit 0x7FFFFFFF for unlimit
 */
extern
void apthermolmt_set_cpu_power_limit
(struct apthermolmt_user *handle, unsigned int limit);

/*
 *	@limit 0x7FFFFFFF for unlimit
 */
extern
void apthermolmt_set_vpu_power_limit
(struct apthermolmt_user *handle, unsigned int limit);

/*
 *	@limit 0x7FFFFFFF for unlimit
 */
extern
void apthermolmt_set_mdla_power_limit
(struct apthermolmt_user *handle, unsigned int limit);

/*
 *	@limit 0x7FFFFFFF for unlimit
 */
extern
void apthermolmt_set_gpu_power_limit
(struct apthermolmt_user *handle, unsigned int limit);

/*
 *	@limit 0 for unlimit
 */
extern
void apthermolmt_set_general_cpu_power_limit
(unsigned int limit);

/*
 *	@limit 0 for unlimit
 */
extern
void apthermolmt_set_general_gpu_power_limit
(unsigned int limit);

extern
unsigned int apthermolmt_get_cpu_power_limit(void);

extern
unsigned int apthermolmt_get_cpu_min_power(void);

extern
unsigned int apthermolmt_get_gpu_power_limit(void);

extern
unsigned int apthermolmt_get_gpu_min_power(void);

extern
unsigned int apthermolmt_get_vpu_power_limit(void);

extern
unsigned int apthermolmt_get_vpu_min_power(void);

extern
unsigned int apthermolmt_get_mdla_power_limit(void);

extern
unsigned int apthermolmt_get_mdla_min_power(void);

#endif	/* __AP_THERMAL_LIMIT_H__ */
