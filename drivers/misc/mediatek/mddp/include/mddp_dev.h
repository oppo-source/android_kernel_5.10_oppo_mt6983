/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mddp_dev.h - Structure/API of MDDP device node control.
 *
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MDDP_DEV_H
#define __MDDP_DEV_H

#include "mddp_export.h"

//------------------------------------------------------------------------------
// Struct definition.
// -----------------------------------------------------------------------------
#define MDDP_DETAILED_STATE_ENABLE 19283746
#define MDDP_DETAILED_STATE_DISABLE 0
#define MDDP_EM_SUPPORT	1                   /**< Engineer mode support */

//------------------------------------------------------------------------------
// Public functions.
// -----------------------------------------------------------------------------
int32_t mddp_dev_init(void);
void mddp_dev_uninit(void);
void mddp_dev_response(enum mddp_app_type_e type,
		enum mddp_ctrl_msg_e msg, bool is_success,
		uint8_t *data, uint32_t data_len);
void mddp_enqueue_dstate(enum mddp_dstate_id_e id, ...);

#endif /* __MDDP_DEV_H */
