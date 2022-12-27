// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_hif.h"
#include "port_cfg.h"

extern int port_md_gen;
#define EXP_CTRL_Q		6
#define DATA_TX_Q		0
#define DATA_RX_Q		0
#define DATA_TX_ACK_Q		1
#define DATA1_TX_Q		0
#define DATA1_RX_Q		0
#define DATA2_Q			2
#define DATA2_RX_Q		0
#define DATA_MDT_Q		0
#define DATA_C2K_PPP_Q	3
#define DATA_FSD_Q		4
#define DATA_AT_CMD_Q	5


#define DATA_TCHE	8


#define SMEM_Q			AP_MD_CCB_WAKEUP

static struct port_t md1_ccci_ports[] = {
/* network port first for performace */
	{CCCI_CCMNI1_TX, CCCI_CCMNI1_RX, DATA_TX_Q, DATA_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 0, "ccmni0",},
	{CCCI_CCMNI2_TX, CCCI_CCMNI2_RX, DATA1_TX_Q, DATA1_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 1, "ccmni1",},
	{CCCI_CCMNI3_TX, CCCI_CCMNI3_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 2, "ccmni2",},
	{CCCI_CCMNI4_TX, CCCI_CCMNI4_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 3, "ccmni3",},
	{CCCI_CCMNI5_TX, CCCI_CCMNI5_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 4, "ccmni4",},
	{CCCI_CCMNI6_TX, CCCI_CCMNI6_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 5, "ccmni5",},
	{CCCI_CCMNI7_TX, CCCI_CCMNI7_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 6, "ccmni6",},
	{CCCI_CCMNI8_TX, CCCI_CCMNI8_RX, DATA1_TX_Q, DATA_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 7, "ccmni7",},

	{CCCI_CCMNI10_TX, CCCI_CCMNI10_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 9, "ccmni9",},
	{CCCI_CCMNI11_TX, CCCI_CCMNI11_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 10, "ccmni10",},
	{CCCI_CCMNI12_TX, CCCI_CCMNI12_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 11, "ccmni11",},
	{CCCI_CCMNI13_TX, CCCI_CCMNI13_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 12, "ccmni12",},
	{CCCI_CCMNI14_TX, CCCI_CCMNI14_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 13, "ccmni13",},
	{CCCI_CCMNI15_TX, CCCI_CCMNI15_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 14, "ccmni14",},
	{CCCI_CCMNI16_TX, CCCI_CCMNI16_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 15, "ccmni15",},
	{CCCI_CCMNI17_TX, CCCI_CCMNI17_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 16, "ccmni16",},
	{CCCI_CCMNI18_TX, CCCI_CCMNI18_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 17, "ccmni17",},
	{CCCI_CCMNI19_TX, CCCI_CCMNI19_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 18, "ccmni18",},
	{CCCI_CCMNI20_TX, CCCI_CCMNI20_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 19, "ccmni19",},
	{CCCI_CCMNI21_TX, CCCI_CCMNI21_RX, DATA1_TX_Q, DATA2_RX_Q,
		0xF0 | DATA_TX_ACK_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 20, "ccmni20",},
	/*ccmni-lan port minor id should be same as ccmni_idx
	 * in ccci_get_ccmni_channel() function
	 */
	{CCCI_CCMNILAN_TX, CCCI_CCMNILAN_RX, DATA_MDT_Q, DATA_MDT_Q,
		0xF0 | DATA_MDT_Q, 0xFF, MD1_NET_HIF, 0,
		&net_port_ops, 21, "ccmni-lan",},
	/* char port, notes ccci_monitor must be first
	 * for get_port_by_minor() implement
	 */
	{CCCI_PCM_TX, CCCI_PCM_RX, 0, 0, 0xFF, 0xFF, MD1_NORMAL_HIF,
		PORT_F_USER_HEADER | PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 1, "ccci_aud",},
	{CCCI_UART1_TX, CCCI_UART1_RX, 1, 1, EXP_CTRL_Q, EXP_CTRL_Q,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 2, "ccci_md_log_ctrl",},
	{CCCI_UART2_TX, CCCI_UART2_RX, DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 3, "ttyC0",},
	{CCCI_FS_TX, CCCI_FS_RX, DATA_FSD_Q, DATA_FSD_Q, 1, 1, MD1_NORMAL_HIF,
		PORT_F_USER_HEADER | PORT_F_WITH_CHAR_NODE | PORT_F_CLEAN,
		&char_port_ops, 4, "ccci_fs",},
	{CCCI_IPC_UART_TX, CCCI_IPC_UART_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 5, "ttyC2",},
	{CCCI_ICUSB_TX, CCCI_ICUSB_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 6, "ttyC3",},
	{CCCI_MD_LOG_TX, CCCI_MD_LOG_RX, 2, 2, 2, 2,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 7, "ttyC1",},
	{CCCI_IMSV_UL, CCCI_IMSV_DL, 6, 6, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 8, "ccci_imsv",},
	{CCCI_IMSC_UL, CCCI_IMSC_DL, 6, 6, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 9, "ccci_imsc",},
	{CCCI_IMSA_UL, CCCI_IMSA_DL, 6, 6, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 10, "ccci_imsa",},
	{CCCI_IMSDC_UL, CCCI_IMSDC_DL, 6, 6, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 11, "ccci_imsdc",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 12, "ccci_ioctl0",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 13, "ccci_ioctl1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 14, "ccci_ioctl2",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 15, "ccci_ioctl3",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 16, "ccci_ioctl4",},
	{CCCI_IT_TX, CCCI_IT_RX, 0, 0, 0xFF, 0xFF, MD1_NORMAL_HIF,
		PORT_F_USER_HEADER | PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 17, "ccci_it",},
	{CCCI_LB_IT_TX, CCCI_LB_IT_RX, 0, 0, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 18, "ccci_lb_it",},
	{CCCI_MDL_MONITOR_UL, CCCI_MDL_MONITOR_DL, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 19, "ccci_mdl_monitor",},
	{CCCI_RPC_TX, CCCI_RPC_RX, 1, 1, 1, 1, MD1_NORMAL_HIF,
		PORT_F_USER_HEADER | PORT_F_WITH_CHAR_NODE,
		&rpc_port_ops, 20, "ccci_rpc",},
	{CCCI_RPC_TX, CCCI_RPC_RX, 1, 1, 1, 1, MD1_NORMAL_HIF, 0,
		&rpc_port_ops, 0xFF, "ccci_rpc_k",},
	{CCCI_IMSEM_UL, CCCI_IMSEM_DL, 6, 6, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 21, "ccci_imsem",},
	{CCCI_IMSM_TX, CCCI_IMSM_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 22, "ccci_imsm",},
	{CCCI_WOA_TX, CCCI_WOA_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 23, "ccci_woa",},
	{CCCI_C2K_PPP_TX, CCCI_C2K_PPP_RX, DATA_C2K_PPP_Q,
		DATA_C2K_PPP_Q, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 24, "ccci_c2k_ppp",},
	{CCCI_C2K_AGPS_TX, CCCI_C2K_AGPS_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 25, "ccci_c2k_agps",},
	{CCCI_XCAP_TX, CCCI_XCAP_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 26, "ccci_ss_xcap",},
	{CCCI_BIP_TX, CCCI_BIP_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 27, "ccci_bip",},
	{CCCI_TCHE_TX, CCCI_TCHE_RX, DATA_TCHE, DATA_TCHE, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 28, "ttyC5",},
	{CCCI_DISP_TX, CCCI_DISP_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 29, "ttyC6",},
	{CCCI_UDC_TX, CCCI_UDC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, 0,
		&ccci_udc_port_ops, 30, "ccci_udc",},
	{CCCI_WIFI_TX, CCCI_WIFI_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE|PORT_F_GEN95_NOT_SUPPORT,
		&char_port_ops, 31, "ccci_wifi_proxy",},
	{CCCI_VTS_TX, CCCI_VTS_RX, 3, 3, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE|PORT_F_GEN95_NOT_SUPPORT,
		&char_port_ops, 32, "ccci_vts",},
	{CCCI_MD_DIRC_TX, CCCI_MD_DIRC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE|PORT_F_GEN95_NOT_SUPPORT,
		&char_port_ops, 33, "ccci_0_200",},
	{CCCI_TIME_TX, CCCI_TIME_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE|PORT_F_GEN95_NOT_SUPPORT,
		&char_port_ops, 34, "ccci_0_202",},
	{CCCI_GARB_TX, CCCI_GARB_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE|PORT_F_GEN95_NOT_SUPPORT,
		&char_port_ops, 35, "ccci_0_204",},
	{CCCI_IKERAW_TX, CCCI_IKERAW_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 36, "ccci_ikeraw",},
/* for EPDG use */
	{CCCI_EPDG1_TX, CCCI_EPDG1_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 37, "ccci_epdg1",},
	{CCCI_EPDG2_TX, CCCI_EPDG2_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 38, "ccci_epdg2",},
	{CCCI_EPDG3_TX, CCCI_EPDG3_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 39, "ccci_epdg3",},
	{CCCI_EPDG4_TX, CCCI_EPDG4_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 40, "ccci_epdg4",},
	/* for muxd/rild mipc, ttyCMIPC0 ~ 9 */
	{CCCI_MIPC0_CHANNEL_TX, CCCI_MIPC0_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 41, "ttyCMIPC0",},
	{CCCI_MIPC1_CHANNEL_TX, CCCI_MIPC1_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 42, "ttyCMIPC1",},
	{CCCI_MIPC2_CHANNEL_TX, CCCI_MIPC2_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 43, "ttyCMIPC2",},
	{CCCI_MIPC3_CHANNEL_TX, CCCI_MIPC3_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 44, "ttyCMIPC3",},
	{CCCI_MIPC4_CHANNEL_TX, CCCI_MIPC4_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 45, "ttyCMIPC4",},
	{CCCI_MIPC5_CHANNEL_TX, CCCI_MIPC5_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 46, "ttyCMIPC5",},
	{CCCI_MIPC6_CHANNEL_TX, CCCI_MIPC6_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 47, "ttyCMIPC6",},
	{CCCI_MIPC7_CHANNEL_TX, CCCI_MIPC7_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 48, "ttyCMIPC7",},
	{CCCI_MIPC8_CHANNEL_TX, CCCI_MIPC8_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 49, "ttyCMIPC8",},
	{CCCI_MIPC9_CHANNEL_TX, CCCI_MIPC9_CHANNEL_RX,
		DATA_AT_CMD_Q, DATA_AT_CMD_Q, 0xFF, 0xFF, MD1_NORMAL_HIF,
		(PORT_F_WITH_CHAR_NODE|PORT_F_CH_TRAFFIC|PORT_F_DUMP_RAW_DATA),
		&char_port_ops, 50, "ttyCMIPC9",},
/* IPC char port minor= minor idx + CCCI_IPC_MINOR_BASE(100) */
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&ipc_port_ops, 0, "ccci_ipc_1220_0",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&ipc_port_ops, 2, "ccci_ipc_2",},
/* IPC kernel port */
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, 0, &ipc_port_ops, 3, "ccci_ipc_3",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&ipc_port_ops, 4, "ccci_ipc_4",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&ipc_port_ops, 5, "ccci_ipc_5",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, 0, &ipc_port_ops, 6, "ccci_ipc_6",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, 0, &ipc_port_ops, 7, "ccci_ipc_7",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, 0, &ipc_port_ops, 8, "ccci_ipc_8",},
	{CCCI_IPC_TX, CCCI_IPC_RX, 1, 1, 0xFF, 0xFF,
		MD1_NORMAL_HIF, PORT_F_WITH_CHAR_NODE,
		&ipc_port_ops, 9, "ccci_ipc_9",},
/* sys port */
	{CCCI_SYSTEM_TX, CCCI_SYSTEM_RX, 0, 0, 0xFF, 0xFF,
		MD1_NORMAL_HIF, 0, &sys_port_ops, 0xFF, "ccci_sys",},
	{CCCI_CONTROL_TX, CCCI_CONTROL_RX, 0, 0, 0, 0,
		MD1_NORMAL_HIF, 0, &ctl_port_ops, 0xFF, "ccci_ctrl",},
	{CCCI_STATUS_TX, CCCI_STATUS_RX, 0, 0, 0, 0,
		MD1_NORMAL_HIF, 0, &poller_port_ops, 0xFF, "ccci_poll",},
/* smem port */
	/*ccb array item must be put together and
	 * should same with memory layout
	 */
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, 0, &smem_port_ops, SMEM_USER_RAW_DBM,
		"ccci_raw_dbm",},
	{CCCI_CCB_CTRL, CCCI_CCB_CTRL, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE, &smem_port_ops,
		SMEM_USER_RAW_CCB_CTRL, "ccci_ccb_ctrl",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, SMEM_Q, SMEM_Q, SMEM_Q, SMEM_Q,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_CCB_DHL, "ccci_ccb_dhl",},

	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE, &smem_port_ops,
		SMEM_USER_RAW_DHL, "ccci_raw_dhl",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_RAW_NETD, "ccci_raw_netd",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_RAW_USB, "ccci_raw_usb",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_RAW_AUDIO, "ccci_raw_audio",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, 0, &smem_port_ops, SMEM_USER_RAW_LWA,
		"ccci_raw_lwa",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_RAW_MDM, "ccci_raw_mdm",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, SMEM_Q, SMEM_Q, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_CCB_MD_MONITOR,
		"ccci_ccb_md_monitor",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, SMEM_Q, SMEM_Q, SMEM_Q, SMEM_Q,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_CCB_META, "ccci_ccb_meta",},

};

#ifdef CONFIG_MTK_ECCCI_C2K
static struct port_t md3_ccci_ports[] = {
/* network port first for performace */
	{CCCI_CCMNI1_TX, CCCI_CCMNI1_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 0, "cc3mni0",},
	{CCCI_CCMNI2_TX, CCCI_CCMNI2_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 1, "cc3mni1",},
	{CCCI_CCMNI3_TX, CCCI_CCMNI3_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 2, "cc3mni2",},
	{CCCI_CCMNI4_TX, CCCI_CCMNI4_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 3, "cc3mni3",},
	{CCCI_CCMNI5_TX, CCCI_CCMNI5_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 4, "cc3mni4",},
	{CCCI_CCMNI6_TX, CCCI_CCMNI6_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 5, "cc3mni5",},
	{CCCI_CCMNI7_TX, CCCI_CCMNI7_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 6, "cc3mni6",},
	{CCCI_CCMNI8_TX, CCCI_CCMNI8_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 7, "cc3mni7",},
	{CCCI_CCMNI10_TX, CCCI_CCMNI10_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 9, "cc3mni9",},
	{CCCI_CCMNI11_TX, CCCI_CCMNI11_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 10, "cc3mni10",},
	{CCCI_CCMNI12_TX, CCCI_CCMNI12_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 11, "cc3mni11",},
	{CCCI_CCMNI13_TX, CCCI_CCMNI13_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 12, "cc3mni12",},
	{CCCI_CCMNI14_TX, CCCI_CCMNI14_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 13, "cc3mni13",},
	{CCCI_CCMNI15_TX, CCCI_CCMNI15_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 14, "cc3mni14",},
	{CCCI_CCMNI16_TX, CCCI_CCMNI16_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 15, "cc3mni15",},
	{CCCI_CCMNI17_TX, CCCI_CCMNI17_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 16, "cc3mni16",},
	{CCCI_CCMNI18_TX, CCCI_CCMNI18_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 17, "cc3mni17",},
	{CCCI_CCMNI19_TX, CCCI_CCMNI19_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 18, "cc3mni18",},
	{CCCI_CCMNI20_TX, CCCI_CCMNI20_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 19, "cc3mni19",},
	{CCCI_CCMNI21_TX, CCCI_CCMNI21_RX, 2, 2, 0xF2, 0xFF,
		CCIF_HIF_ID, 0,	&net_port_ops, 20, "cc3mni20",},

/* char port, notes ccci_monitor must be first */
	{CCCI_PCM_TX, CCCI_PCM_RX, C2K_PCM_TX_Q, C2K_PCM_RX_Q, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 1, "ccci3_aud",},
	{CCCI_UART1_TX, CCCI_UART1_RX, 0, 0, 0, 0,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 2, "ccci3_md_log_ctrl",},
	{CCCI_FS_TX, CCCI_FS_RX, 4, 4, 4, 4, CCIF_HIF_ID,
		PORT_F_USER_HEADER | PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 4, "ccci3_fs",},
	{CCCI_IPC_UART_TX, CCCI_IPC_UART_RX, 6, 6, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 5, "ccci3_tty2",},	/*for agps */
	{CCCI_MD_LOG_TX, CCCI_MD_LOG_RX, C2K_MD_LOG_TX_Q,
		C2K_MD_LOG_RX_Q, C2K_MD_LOG_TX_Q, C2K_MD_LOG_RX_Q,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE, &char_port_ops,
		7, "ccci3_tty1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 12, "ccci3_ioctl0",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 13, "ccci3_ioctl1",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 14, "ccci3_ioctl2",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 15, "ccci3_ioctl3",},
	{CCCI_DUMMY_CH, CCCI_DUMMY_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 16, "ccci3_ioctl4",},
	{CCCI_IT_TX, CCCI_IT_RX, 0, 0, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_USER_HEADER | PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 17, "ccci3_it",},
	{CCCI_LB_IT_TX, CCCI_LB_IT_RX, 0, 0, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 18, "ccci3_lb_it",},
/* c2k only port */
	{CCCI_C2K_PPP_DATA,	CCCI_C2K_PPP_DATA, 2, 2, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 19, "ccci3_data",},
	{CCCI_C2K_AT, CCCI_C2K_AT, 5, 5, 0xF4, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE,
		&char_port_ops, 20, "ccci3_at",},
	{CCCI_C2K_AT2, CCCI_C2K_AT2, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 21, "ccci3_at2",},
	{CCCI_C2K_AT3, CCCI_C2K_AT3, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 22, "ccci3_at3",},
	{CCCI_C2K_AT4, CCCI_C2K_AT4, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 23, "ccci3_at4",},
	{CCCI_C2K_AT5, CCCI_C2K_AT5, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 24, "ccci3_at5",},
	{CCCI_C2K_AT6, CCCI_C2K_AT6, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 25, "ccci3_at6",},
	{CCCI_C2K_AT7, CCCI_C2K_AT7, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 26, "ccci3_at7",},
	{CCCI_C2K_AT8, CCCI_C2K_AT8, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 27, "ccci3_at8",},
	{CCCI_C2K_LB_DL, CCCI_C2K_LB_DL, 5, 5, 0xFF, 0xFF, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &char_port_ops, 28, "ccci3_lb_dl",},
/* sys port */
	{CCCI_CONTROL_TX, CCCI_CONTROL_RX, 0, 0, 0, 0, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &ctl_port_ops, 0, "ccci3_ctrl",},
	{CCCI_STATUS_TX, CCCI_STATUS_RX, 0, 0, 0, 0, CCIF_HIF_ID,
		PORT_F_WITH_CHAR_NODE, &poller_port_ops, 0, "ccci3_poll",},
/* smem port */
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, 0, &smem_port_ops, SMEM_USER_RAW_DBM,
		"ccci3_raw_dbm",},
	{CCCI_SMEM_CH, CCCI_SMEM_CH, 0xFF, 0xFF, 0xFF, 0xFF,
		CCIF_HIF_ID, PORT_F_WITH_CHAR_NODE,
		&smem_port_ops, SMEM_USER_RAW_AUDIO, "ccci3_raw_audio",},
};

#endif

int port_get_cfg(int md_id, struct port_t **ports)
{
	int port_number = 0;

	switch (md_id) {
	case MD_SYS1:
		*ports = md1_ccci_ports;
		port_number = ARRAY_SIZE(md1_ccci_ports);
		break;

#ifdef CONFIG_MTK_ENABLE_MD2
	case MD_SYS2:
		*ports = md2_ccci_ports;
		port_number = ARRAY_SIZE(md2_ccci_ports);
		break;
#endif
#ifdef CONFIG_MTK_ECCCI_C2K
	case MD_SYS3:
		*ports = md3_ccci_ports;
		port_number = ARRAY_SIZE(md3_ccci_ports);
		break;
#endif
	default:
		*ports = NULL;
		port_number = 0;
		CCCI_ERROR_LOG(md_id, PORT, "md_port_cfg:no md enable\n");
		break;
	}
	return port_number;
}

int mtk_ccci_request_port(char *name)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(md1_ccci_ports); i++) {
		if (!strcmp(md1_ccci_ports[i].name, name))
			break;
	}
	if (i == ARRAY_SIZE(md1_ccci_ports)) {
		CCCI_ERROR_LOG(-1, PORT, "can not find port %s", name);
		return -1;
	}

	if ((port_md_gen < 6297) && (md1_ccci_ports[i].flags & PORT_F_GEN95_NOT_SUPPORT)) {
		CCCI_ERROR_LOG(-1, PORT, "%s:not support %s  md_gen = %d\n",
				__func__, md1_ccci_ports[i].name, port_md_gen);
		return -1;
	}

	return i;

}
EXPORT_SYMBOL(mtk_ccci_request_port);

int find_port_by_channel(int index, struct port_t **port)
{
	if (index < ARRAY_SIZE(md1_ccci_ports)) {
		*port = &md1_ccci_ports[index];
		return 0;
	}
	CCCI_ERROR_LOG(-1, PORT, "cannot find port by index %d\n", index);
	return -1;
}

int mtk_ccci_open_port(int index)
{
	if (index < 0 || index >= ARRAY_SIZE(md1_ccci_ports)) {
		CCCI_ERROR_LOG(-1, PORT, "invalid index = %d\n", index);
		return -EBUSY;
	}
	if (md1_ccci_ports[index].rx_ch != CCCI_CCB_CTRL &&
		atomic_read(&md1_ccci_ports[index].usage_cnt))
		return -EBUSY;
	atomic_inc(&md1_ccci_ports[index].usage_cnt);
	return 0;
}
EXPORT_SYMBOL(mtk_ccci_open_port);

int mtk_ccci_release_port(int index)
{
	if (index < 0 || index >= ARRAY_SIZE(md1_ccci_ports)) {
		CCCI_ERROR_LOG(-1, PORT, "invalid index = %d\n", index);
		return -EBUSY;
	}
	atomic_dec(&md1_ccci_ports[index].usage_cnt);
	return 0;
}
EXPORT_SYMBOL(mtk_ccci_release_port);
