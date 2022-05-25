// SPDX-License-Identifier: GPL-2.0
/*
 *  vow.c  --  VoW platform driver
 *
 *  Copyright (c) 2020 MediaTek Inc.
 *  Author: Michael HSiao <michael.hsiao@mediatek.com>
 */

/*****************************************************************************
 * Header Files
 *****************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <asm/page.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/miscdevice.h>
/* #include <linux/wakelock.h> */
#include <linux/pm_wakeup.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>  /* FOR SCP REVOCER */
#ifdef SIGTEST
#include <asm/siginfo.h>
#endif
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/pinctrl/consumer.h>
#include <uapi/linux/sched/types.h>

#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
#include "scp_helper.h"
#include "scp_ipi.h"
#include "scp_excep.h"
#include "audio_task_manager.h"
#include "audio_ipi_queue.h"
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */
#include "vow.h"
#include "vow_scp.h"
#include "vow_assert.h"


/*****************************************************************************
 * Variable Definition
 ****************************************************************************/
static unsigned int VowDrv_Wait_Queue_flag;
static unsigned int VoiceData_Wait_Queue_flag;
static DECLARE_WAIT_QUEUE_HEAD(VowDrv_Wait_Queue);
static DECLARE_WAIT_QUEUE_HEAD(VoiceData_Wait_Queue);
static DEFINE_SPINLOCK(vowdrv_lock);
static struct wakeup_source *vow_suspend_lock;
static int init_flag = -1;

static struct file *file_recog_data;
static uint32_t recog_dump_data_routine_cnt_pass;
static struct wakeup_source pcm_dump_wake_lock;
static struct dump_queue_t *dump_queue;
static struct task_struct *pcm_dump_task;
static bool b_enable_dump;
static struct dump_work_t dump_work[NUM_DUMP_DATA];
static struct workqueue_struct *dump_workqueue[NUM_DUMP_DATA];
static wait_queue_head_t wq_dump_pcm;
struct pcm_dump_t {
	char decode_pcm[FRAME_BUF_SIZE];
};
#define DUMP_PCM_DATA_PATH "/data/vendor/audiohal/audio_dump"
static struct file *file_bargein_pcm_input;
static struct file *file_bargein_echo_ref;
static struct file *file_bargein_delay_info;
static uint32_t bargein_dump_data_routine_cnt_pass;
static bool bargein_dump_info_flag;
static bool file_bargein_pcm_input_open;
static bool file_bargein_echo_ref_open;
static bool file_bargein_delay_info_open;
static bool file_recog_data_open;

/*****************************************************************************
 * Function  Declaration
 ****************************************************************************/
static void vow_service_getVoiceData(void);
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
static void vow_ipi_reg_ok(short uuid, int confidence_lv);
static bool vow_IPICmd_Send(uint8_t data_type,
			    uint8_t ack_type,
			    uint16_t msg_id,
			    uint32_t param1,
			    uint32_t param2,
			    char *payload);
static void vow_IPICmd_Received(struct ipi_msg_t *ipi_msg);
static bool vow_IPICmd_ReceiveAck(struct ipi_msg_t *ipi_msg);
static void vow_Task_Unloaded_Handling(void);
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */
static bool VowDrv_SetFlag(int type, unsigned int set);
static int VowDrv_GetHWStatus(void);
static void vow_service_OpenDumpFile(void);
static void vow_service_CloseDumpFile(void);
static void vow_service_OpenDumpFile_internal(void);
static void vow_service_CloseDumpFile_internal(void);
static void vow_pcm_dump_init(void);
static void vow_pcm_dump_deinit(void);
static int vow_pcm_dump_kthread(void *data);
static bool VowDrv_SetBargeIn(unsigned int set, unsigned int irq_id);
static void bargein_dump_routine(struct work_struct *ws);
static void recog_dump_routine(struct work_struct *ws);
static int vow_service_SearchSpeakerModelWithUuid(int uuid);
static int vow_service_SearchSpeakerModelWithId(int id);

/*****************************************************************************
 * VOW SERVICES
 *****************************************************************************/

static struct
{
	struct vow_speaker_model_t  vow_speaker_model[MAX_VOW_SPEAKER_MODEL];
	unsigned long        vow_info_apuser[MAX_VOW_INFO_LEN];
	unsigned long        voicedata_user_addr;
	unsigned long        voicedata_user_size;
	short                *voicedata_kernel_ptr;
	char                 *voicddata_scp_ptr;
	dma_addr_t           voicedata_scp_addr;
	unsigned int         voicedata_idx;
	bool                 scp_command_flag;
	bool                 recording_flag;
	int                  scp_command_id;
	int                  confidence_level;
	int                  eint_status;
	int                  pwr_status;
	bool                 suspend_lock;
	bool                 firstRead;
	unsigned long        voicedata_user_return_size_addr;
	unsigned int         voice_buf_offset;
	unsigned int         voice_length;
	unsigned int         transfer_length;
	struct device_node   *node;
	struct pinctrl       *pinctrl;
	struct pinctrl_state *pins_eint_on;
	struct pinctrl_state *pins_eint_off;
	bool                 bypass_enter_phase3;
	unsigned int         enter_phase3_cnt;
	unsigned int         force_phase_stage;
	bool                 swip_log_enable;
	struct vow_eint_data_struct_t  vow_eint_data_struct;
	unsigned long long   scp_recognize_ok_cycle;
	unsigned long long   ap_received_ipi_cycle;
	bool                 tx_keyword_start;
	unsigned int         dump_frm_cnt;
	unsigned int         voice_sample_delay;
	unsigned int         bargein_dump_cnt1;
	unsigned int         bargein_dump_cnt2;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	short                *interleave_pcmdata_ptr;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
	bool                 dump_pcm_flag;
	bool                 scp_vow_lch;
	bool                 scp_recovering;
	bool                 vow_recovering;
	unsigned int         recog_dump_cnt1;
	unsigned int         recog_dump_cnt2;
	bool                 split_dumpfile_flag;
	unsigned int         mtkif_type;
	unsigned int         google_engine_version;
	char                 alexa_engine_version[VOW_ENGINE_INFO_LENGTH_BYTE];
	char                 google_engine_arch[VOW_ENGINE_INFO_LENGTH_BYTE];
} vowserv;

static struct
{
	dma_addr_t    phy_addr;
	char          *vir_addr;
	uint32_t      size;
} bargein_resv_dram;

static struct
{
	dma_addr_t    phy_addr;
	char          *vir_addr;
	uint32_t      size;
} recog_resv_dram;

/*****************************************************************************
 * DSP IPI HANDELER
 *****************************************************************************/
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
static void vow_Task_Unloaded_Handling(void)
{
	VOWDRV_DEBUG("%s()\n", __func__);
}

static bool vow_IPICmd_ReceiveAck(struct ipi_msg_t *ipi_msg)
{
	bool result = false;


	switch (ipi_msg->msg_id) {
	case IPIMSG_VOW_ENABLE:
	case IPIMSG_VOW_DISABLE:
	case IPIMSG_VOW_SET_MODEL:
	case IPIMSG_VOW_SET_SMART_DEVICE:
	case IPIMSG_VOW_APREGDATA_ADDR:
	case IPIMSG_VOW_PCM_DUMP_ON:
	case IPIMSG_VOW_PCM_DUMP_OFF:
		if (ipi_msg->param1 == VOW_IPI_SUCCESS)
			result = true;
		break;
	case IPIMSG_VOW_SET_BARGEIN_ON:
	case IPIMSG_VOW_SET_BARGEIN_OFF:
		if (ipi_msg->param1 == VOW_IPI_SUCCESS)
			result = true;
		break;
	case IPIMSG_VOW_SET_FLAG:
		if (ipi_msg->param1 == VOW_IPI_SUCCESS) {
			unsigned int return_id;
			unsigned int return_value;

			result = true;
			return_id    = (ipi_msg->param2 >> WORD_H);
			return_value = (ipi_msg->param2 & WORD_L_MASK);
			switch (return_id) {
			case VOW_FLAG_FORCE_PHASE1_DEBUG:
			case VOW_FLAG_FORCE_PHASE2_DEBUG:
				vowserv.force_phase_stage = return_value;
				break;
			case VOW_FLAG_SWIP_LOG_PRINT:
				vowserv.swip_log_enable = return_value;
				break;
			default:
				break;
			}
		}
		break;
	default:
		break;
	}
	return result;
}

static void vow_IPICmd_Received(struct ipi_msg_t *ipi_msg)
{
	unsigned int *ptr32;

	switch (ipi_msg->msg_id) {
	case IPIMSG_VOW_COMBINED_INFO: {
		struct vow_ipi_combined_info_t *ipi_ptr;
		bool bypass_flag;

		ipi_ptr = (struct vow_ipi_combined_info_t *)ipi_msg->payload;
		/* IPIMSG_VOW_RECOGNIZE_OK */
		/*VOWDRV_DEBUG("[vow] IPIMSG_VOW_COMBINED_INFO\n");*/
		bypass_flag = false;
		if (ipi_ptr->ipi_type_flag & RECOG_OK_IDX_MASK) {
			if ((vowserv.recording_flag == true) &&
			    (vowserv.tx_keyword_start == true)) {
				VOWDRV_DEBUG("%s(), bypass this recog ok\n",
					__func__);
				bypass_flag = true;
			}
			if (bypass_flag == false) {
				vowserv.ap_received_ipi_cycle =
					get_cycles();
				vowserv.scp_recognize_ok_cycle =
					ipi_ptr->recog_ok_os_timer;
				vowserv.enter_phase3_cnt++;
				if (vowserv.bypass_enter_phase3 == false) {
					vow_ipi_reg_ok(
					    (short)ipi_ptr->recog_ok_uuid,
					    ipi_ptr->confidence_lv);
				}
			}
		}
		/* IPIMSG_VOW_DATAREADY */
		if ((ipi_ptr->ipi_type_flag & DEBUG_DUMP_IDX_MASK) &&
		    (vowserv.recording_flag)) {
			ptr32 = (unsigned int *)ipi_msg->payload;
			vowserv.voice_buf_offset = ipi_ptr->voice_buf_offset;
			vowserv.voice_length = ipi_ptr->voice_length;
			if (vowserv.voice_length > 320)
				VOWDRV_DEBUG("vow,v_len=%x\n",
					     vowserv.voice_length);
			vow_service_getVoiceData();
		}
		/* IPIMSG_VOW_BARGEIN_DUMP_INFO */
		if (ipi_ptr->ipi_type_flag & BARGEIN_DUMP_INFO_IDX_MASK) {
			vowserv.dump_frm_cnt = ipi_ptr->dump_frm_cnt;
			vowserv.voice_sample_delay =
				ipi_ptr->voice_sample_delay;
			VOWDRV_DEBUG("[BargeIn] dump write0 %d %d\n",
				     vowserv.dump_frm_cnt,
				     vowserv.voice_sample_delay);
			bargein_dump_info_flag = true;
		}
		/* IPIMSG_VOW_BARGEIN_PCMDUMP_OK */
		if ((ipi_ptr->ipi_type_flag & BARGEIN_DUMP_IDX_MASK) &&
		    (vowserv.dump_pcm_flag)) {
			int ret = 0;
			uint8_t idx = 0; /* dump_data_t */

			idx = DUMP_BARGEIN;
			dump_work[idx].mic_data_size = ipi_ptr->mic_dump_size;
			dump_work[idx].mic_offset = ipi_ptr->mic_offset;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
			dump_work[idx].mic_data_size_R =
					ipi_ptr->mic_dump_size_R;
			dump_work[idx].mic_offset_R = ipi_ptr->mic_offset_R;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
			dump_work[idx].echo_data_size = ipi_ptr->echo_dump_size;
			dump_work[idx].echo_offset = ipi_ptr->echo_offset;

			ret = queue_work(dump_workqueue[idx],
					 &dump_work[idx].work);
			if (ret == 0)
				bargein_dump_data_routine_cnt_pass++;
		}
		if ((ipi_ptr->ipi_type_flag & RECOG_DUMP_IDX_MASK) &&
		    (vowserv.dump_pcm_flag)) {
			int ret = 0;
			uint8_t idx = 0; /* dump_data_t */

			idx = DUMP_RECOG;
			dump_work[idx].recog_data_size =
				ipi_ptr->recog_dump_size;
			dump_work[idx].recog_data_offset =
				ipi_ptr->recog_dump_offset;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
			dump_work[idx].recog_data_size_R =
				ipi_ptr->recog_dump_size_R;
			dump_work[idx].recog_data_offset_R =
				ipi_ptr->recog_dump_offset_R;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
			ret = queue_work(dump_workqueue[idx],
					 &dump_work[idx].work);
			if (ret == 0)
				recog_dump_data_routine_cnt_pass++;
		}
		break;
	}
	case IPIMSG_VOW_ALEXA_ENGINE_VER: {
		VOWDRV_DEBUG("%s(), IPIMSG_VOW_ALEXA_ENGINE_VER %s\r",
			__func__, ipi_msg->payload);
		memcpy(vowserv.alexa_engine_version, ipi_msg->payload,
				 sizeof(vowserv.alexa_engine_version));
		}
		break;
	case IPIMSG_VOW_GOOGLE_ENGINE_VER: {
		unsigned int *temp = (unsigned int *)ipi_msg->payload;

		VOWDRV_DEBUG("%s(), IPIMSG_VOW_GOOGLE_ENGINE_VER 0x%x\r",
			__func__, temp[0]);
		vowserv.google_engine_version = temp[0];
		}
		break;
	case IPIMSG_VOW_GOOGLE_ARCH: {
		VOWDRV_DEBUG("%s(), IPIMSG_VOW_GOOGLE_ARCH %s\r",
			__func__, ipi_msg->payload);
		memcpy(vowserv.google_engine_arch, ipi_msg->payload,
				 sizeof(vowserv.google_engine_arch));
		}
		break;
	default:
		break;
	}
}

static bool vow_IPICmd_Send(uint8_t data_type,
			    uint8_t ack_type, uint16_t msg_id, uint32_t param1,
			    uint32_t param2, char *payload)
{
	bool ret = false;
	struct ipi_msg_t ipi_msg;
	int ipi_result = -1;
	unsigned int retry_time = VOW_IPI_SEND_CNT_TIMEOUT;
	unsigned int retry_cnt;

	if (!vow_check_scp_status()) {
		VOWDRV_DEBUG("SCP is off, bypass send ipi id(%d)\n", msg_id);
		return false;
	}
	if (vowserv.scp_recovering == true) {
		VOWDRV_DEBUG("scp is recovering, then break\n");
		return false;
	}
	for (retry_cnt = 0; retry_cnt <= retry_time; retry_cnt++) {
		ipi_result = audio_send_ipi_msg(&ipi_msg,
						TASK_SCENE_VOW,
						AUDIO_IPI_LAYER_TO_DSP,
						data_type,
						ack_type,
						msg_id,
						param1,
						param2,
						payload);
		if (ipi_result == 0)
			break;
		if (vowserv.scp_recovering == true) {
			VOWDRV_DEBUG("scp is recovering, then break\n");
			break;
		}
		VOW_ASSERT(retry_cnt != retry_time);
		msleep(VOW_WAITCHECK_INTERVAL_MS);
	}
	if (ipi_result == 0) {
		/* ipi send pass */
		if (ipi_msg.ack_type == AUDIO_IPI_MSG_ACK_BACK)
			ret = vow_IPICmd_ReceiveAck(&ipi_msg);
		else
			ret = true;
	}
	return ret;
}

static void vow_ipi_reg_ok(short uuid, int confidence_lv)
{
	int slot;

	vowserv.scp_command_flag = true;
	/* transfer uuid to model handle id */
	slot = vow_service_SearchSpeakerModelWithUuid(uuid);
	if (slot < 0)
		return;
	vowserv.scp_command_id = vowserv.vow_speaker_model[slot].id;
	vowserv.confidence_level = confidence_lv;
	VowDrv_Wait_Queue_flag = 1;
	wake_up_interruptible(&VowDrv_Wait_Queue);
}

static void vow_register_feature(enum feature_id id)
{
	if (!vow_check_scp_status()) {
		VOWDRV_DEBUG("SCP is off, bypass register id(%d)\n", id);
		return;
	}
	scp_register_feature(id);
}

static void vow_deregister_feature(enum feature_id id)
{
	if (!vow_check_scp_status()) {
		VOWDRV_DEBUG("SCP is off, bypass deregister id(%d)\n", id);
		return;
	}
	scp_deregister_feature(id);
}
#endif
static void vow_service_getVoiceData(void)
{
	if (VoiceData_Wait_Queue_flag == 0) {
		VoiceData_Wait_Queue_flag = 1;
		wake_up_interruptible(&VoiceData_Wait_Queue);
	} else {
		/* VOWDRV_DEBUG("getVoiceData but no one wait for it, */
		/* may lost it!!\n"); */
	}
}

/*****************************************************************************
 * DSP SERVICE FUNCTIONS
 *****************************************************************************/
static void vow_service_Init(void)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	int I;
	bool ret;
	unsigned int vow_ipi_buf[1];

	VOWDRV_DEBUG("%s():%x\n", __func__, init_flag);
	audio_load_task(TASK_SCENE_VOW);
	if (init_flag != 1) {

		/*register IPI handler*/
		audio_task_register_callback(TASK_SCENE_VOW,
					     vow_IPICmd_Received,
					     vow_Task_Unloaded_Handling);
		/*Initialization*/
		VowDrv_Wait_Queue_flag = 0;
		VoiceData_Wait_Queue_flag = 0;
		vowserv.scp_command_flag = false;
		vowserv.recording_flag = false;
		vowserv.suspend_lock = 0;
		vowserv.voice_length = 0;
		vowserv.firstRead = false;
		vowserv.voice_buf_offset = 0;
		vowserv.bypass_enter_phase3 = false;
		vowserv.enter_phase3_cnt = 0;
		vowserv.scp_recovering = false;
		vowserv.vow_recovering = false;
		spin_lock(&vowdrv_lock);
		vowserv.pwr_status = VOW_PWR_OFF;
		vowserv.eint_status = VOW_EINT_DISABLE;
		spin_unlock(&vowdrv_lock);
		vowserv.force_phase_stage = NO_FORCE;
		vowserv.swip_log_enable = true;
		vowserv.voicedata_user_addr = 0;
		vowserv.voicedata_user_size = 0;
		vowserv.voicedata_user_return_size_addr = 0;
		vowserv.tx_keyword_start = false;
		for (I = 0; I < MAX_VOW_SPEAKER_MODEL; I++) {
			vowserv.vow_speaker_model[I].model_ptr = NULL;
			vowserv.vow_speaker_model[I].id = -1;
			vowserv.vow_speaker_model[I].keyword = -1;
			vowserv.vow_speaker_model[I].uuid = 0;
			vowserv.vow_speaker_model[I].flag = 0;
			vowserv.vow_speaker_model[I].enabled = 0;
		}
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
		vowserv.voicddata_scp_ptr =
		    (char *)(scp_get_reserve_mem_virt(VOW_MEM_ID))
		    + VOW_VOICEDATA_OFFSET;
		vowserv.voicedata_scp_addr =
		    scp_get_reserve_mem_phys(VOW_MEM_ID)
		    + VOW_VOICEDATA_OFFSET;
#else
		VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
		vowserv.voicedata_kernel_ptr = NULL;
		vowserv.voicedata_idx = 0;
		init_flag = 1;
		vowserv.dump_pcm_flag = false;
		vowserv.split_dumpfile_flag = false;
		dump_queue = NULL;
		vow_pcm_dump_init();
		bargein_dump_info_flag = false;
		vowserv.scp_vow_lch = true;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
		vowserv.interleave_pcmdata_ptr = NULL;
#endif
		vowserv.google_engine_version = 254868980;
		memset(vowserv.alexa_engine_version, 0, VOW_ENGINE_INFO_LENGTH_BYTE);
	} else {
		/*Initialization*/
		vowserv.voicddata_scp_ptr =
		    (char *)(scp_get_reserve_mem_virt(VOW_MEM_ID))
		    + VOW_VOICEDATA_OFFSET;
		vowserv.voicedata_scp_addr =
		    scp_get_reserve_mem_phys(VOW_MEM_ID) + VOW_VOICEDATA_OFFSET;
		vow_ipi_buf[0] = vowserv.voicedata_scp_addr;
		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_BYPASS_ACK,
				      IPIMSG_VOW_APREGDATA_ADDR,
				      sizeof(unsigned int) * 1, 0,
				      (char *)&vow_ipi_buf[0]);
		if (ret == 0) {
			VOWDRV_DEBUG(
			"IPIMSG_VOW_APREGDATA_ADDR ipi send error\n");
		}
#if VOW_PRE_LEARN_MODE
		VowDrv_SetFlag(VOW_FLAG_PRE_LEARN, true);
#endif
	}
	vow_IPICmd_Send(AUDIO_IPI_MSG_ONLY,
		      AUDIO_IPI_MSG_BYPASS_ACK,
		      IPIMSG_VOW_GET_ALEXA_ENGINE_VER,
		      0, 0,
		      NULL);
	vow_IPICmd_Send(AUDIO_IPI_MSG_ONLY,
		      AUDIO_IPI_MSG_BYPASS_ACK,
		      IPIMSG_VOW_GET_GOOGLE_ENGINE_VER,
		      0, 0,
		      NULL);
	vow_IPICmd_Send(AUDIO_IPI_MSG_ONLY,
		      AUDIO_IPI_MSG_BYPASS_ACK,
		      IPIMSG_VOW_GET_GOOGLE_ARCH,
		      0, 0,
		      NULL);
#else  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */
}

int vow_service_GetParameter(unsigned long arg)
{
	if (copy_from_user((void *)(&vowserv.vow_info_apuser[0]),
			   (const void __user *)(arg),
			   sizeof(vowserv.vow_info_apuser))) {
		VOWDRV_DEBUG("vow get parameter fail\n");
		return -EFAULT;
	}
	VOWDRV_DEBUG("vow get parameter: %lu %lu %lu %lu %lu %lu %lu\n",
		     vowserv.vow_info_apuser[0],
		     vowserv.vow_info_apuser[1],
		     vowserv.vow_info_apuser[2],
		     vowserv.vow_info_apuser[3],
		     vowserv.vow_info_apuser[4],
		     vowserv.vow_info_apuser[5],
		     vowserv.vow_info_apuser[6]);

	return 0;
}

#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
static int vow_service_CopyModel(int slot)
{
	if (slot >= MAX_VOW_SPEAKER_MODEL || slot < 0) {
		VOWDRV_DEBUG("%s(), slot id=%d, over range\n", __func__, slot);
		return -EDOM;
	}
	if (vowserv.vow_info_apuser[3] > VOW_MODEL_SIZE) {
		VOWDRV_DEBUG("vow DMA Size Too Large\n");
		return -EFAULT;
	}
	if (copy_from_user((void *)(vowserv.vow_speaker_model[slot].model_ptr),
			   (const void __user *)(vowserv.vow_info_apuser[2]),
			   vowserv.vow_info_apuser[3])) {
		VOWDRV_DEBUG("vow Copy Speaker Model Fail\n");
		return -EFAULT;
	}
	vowserv.vow_speaker_model[slot].flag = 1;
	vowserv.vow_speaker_model[slot].enabled = 0;
	vowserv.vow_speaker_model[slot].id = vowserv.vow_info_apuser[0];
	vowserv.vow_speaker_model[slot].keyword = vowserv.vow_info_apuser[1];
	vowserv.vow_speaker_model[slot].uuid = vowserv.vow_info_apuser[5];
	vowserv.vow_speaker_model[slot].model_size = vowserv.vow_info_apuser[3];

	return 0;
}
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */

static int vow_service_FindFreeSpeakerModel(void)
{
	int I;

	I = 0;
	do {
		if (vowserv.vow_speaker_model[I].flag == 0)
			break;
		I++;
	} while (I < MAX_VOW_SPEAKER_MODEL);

	VOWDRV_DEBUG("vow FindFreeSpeakerModel:%d\n", I);

	if (I == MAX_VOW_SPEAKER_MODEL) {
		VOWDRV_DEBUG("vow Find Free Speaker Model Fail\n");
		return -1;
	}
	return I;
}

static int vow_service_SearchSpeakerModelWithId(int id)
{
	int I, J;

	I = 0;
	do {
		if (vowserv.vow_speaker_model[I].id == id)
			break;
		I++;
	} while (I < MAX_VOW_SPEAKER_MODEL);

	if (I == MAX_VOW_SPEAKER_MODEL) {
		VOWDRV_DEBUG("vow Search Speaker Model By ID Fail:%x\n", id);
		for (J = 0; J < MAX_VOW_SPEAKER_MODEL; J++) {
			/* print all id */
			VOWDRV_DEBUG("id[%d]=%d\n",
				     J, vowserv.vow_speaker_model[J].id);
		}
		return -1;
	}
	return I;
}

static int vow_service_SearchSpeakerModelWithUuid(int uuid)
{
	int I, J;

	I = 0;
	do {
		if (vowserv.vow_speaker_model[I].uuid == uuid)
			break;
		I++;
	} while (I < MAX_VOW_SPEAKER_MODEL);

	if (I == MAX_VOW_SPEAKER_MODEL) {
		VOWDRV_DEBUG("vow Search Speaker Model By UUID Fail:%x\n",
			     uuid);
		for (J = 0; J < MAX_VOW_SPEAKER_MODEL; J++) {
			/* print all uuid */
			VOWDRV_DEBUG("uuid[%d]=%d\n",
				     J, vowserv.vow_speaker_model[J].uuid);
		}
		return -1;
	}
	return I;
}

static bool vow_service_SendSpeakerModel(int slot, bool release_flag)
{
	bool ret = false;
	unsigned int vow_ipi_buf[5];
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	if (slot >= MAX_VOW_SPEAKER_MODEL || slot < 0) {
		VOWDRV_DEBUG("%s(), slot id=%d, over range\n", __func__, slot);
		return ret;
	}

	if (release_flag == VOW_CLEAN_MODEL) {
		if (vowserv.vow_speaker_model[slot].flag == 0) {
			VOWDRV_DEBUG("%s(), slot:%d, no model need to clean\n",
				     __func__, slot);
			return ret;
		}
		vow_ipi_buf[0] = VOW_MODEL_CLEAR;
	} else {  /* VOW_SET_MODEL */
		if ((vowserv.vow_speaker_model[slot].flag == 0) ||
		    (vowserv.vow_speaker_model[slot].id == -1) ||
		    (vowserv.vow_speaker_model[slot].keyword == -1) ||
		    (vowserv.vow_speaker_model[slot].uuid == 0) ||
		    (vowserv.vow_speaker_model[slot].model_size == 0)) {
			VOWDRV_DEBUG("%s(), slot:%d, no model need to set\n",
				     __func__, slot);
			return ret;
		}
		vow_ipi_buf[0] = VOW_MODEL_SPEAKER;
	}
	vow_ipi_buf[1] = vowserv.vow_speaker_model[slot].uuid;
	vow_ipi_buf[2] = scp_get_reserve_mem_phys(VOW_MEM_ID) +
			 (VOW_MODEL_SIZE * slot);
	vow_ipi_buf[3] = vowserv.vow_speaker_model[slot].model_size;
	vow_ipi_buf[4] = vowserv.vow_speaker_model[slot].keyword;

	VOWDRV_DEBUG(
	    "Model:slot_%d, model_%x, addr_%x, id_%x, size_%x, keyword_%x\n",
		      slot,
		      vow_ipi_buf[0],
		      vow_ipi_buf[2],
		      vow_ipi_buf[1],
		      vow_ipi_buf[3],
		      vow_ipi_buf[4]);

	ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
			      AUDIO_IPI_MSG_BYPASS_ACK,
			      IPIMSG_VOW_SET_MODEL,
			      sizeof(unsigned int) * 4, 0,
			      (char *)&vow_ipi_buf[0]);
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	return ret;
}

static bool vow_service_ReleaseSpeakerModel(int id)
{
	int I;
	bool ret = false;

	I = vow_service_SearchSpeakerModelWithId(id);

	if (I == -1) {
		VOWDRV_DEBUG("vow release Speaker Model Fail, id:%x\n", id);
		return false;
	}
	VOWDRV_DEBUG("vow ReleaseSpeakerModel:id_%x, slot_%d\n", id, I);

	ret = vow_service_SendSpeakerModel(I, VOW_CLEAN_MODEL);

	vowserv.vow_speaker_model[I].model_ptr = NULL;
	vowserv.vow_speaker_model[I].uuid = 0;
	vowserv.vow_speaker_model[I].id = -1;
	vowserv.vow_speaker_model[I].keyword = -1;
	vowserv.vow_speaker_model[I].flag = 0;
	vowserv.vow_speaker_model[I].enabled = 0;

	return ret;
}

static bool vow_service_SetSpeakerModel(unsigned long arg)
{
	bool ret = false;
	int I;
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	char *ptr8;
#endif

	I = vow_service_FindFreeSpeakerModel();
	if (I == -1)
		return false;

	vow_service_GetParameter(arg);
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	vowserv.vow_speaker_model[I].model_ptr =
	   (void *)(scp_get_reserve_mem_virt(VOW_MEM_ID))
	   + (VOW_MODEL_SIZE * I);

	if (vow_service_CopyModel(I) != 0)
		return false;

	ptr8 = (char *)vowserv.vow_speaker_model[I].model_ptr;
	VOWDRV_DEBUG("SetSPKModel:slot: %d, ID: %x, UUID: %x, flag: %x, keyword: %x\n",
		      I,
		      vowserv.vow_speaker_model[I].id,
		      vowserv.vow_speaker_model[I].uuid,
		      vowserv.vow_speaker_model[I].flag,
		      vowserv.vow_speaker_model[I].keyword);
	VOWDRV_DEBUG("vow SetSPKModel:CheckValue:%x %x %x %x %x %x\n",
		      *(char *)&ptr8[0], *(char *)&ptr8[1],
		      *(char *)&ptr8[2], *(char *)&ptr8[3],
		      *(short *)&ptr8[160], *(int *)&ptr8[7960]);

	ret = vow_service_SendSpeakerModel(I, VOW_SET_MODEL);
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	return ret;
}

static bool vow_service_SendModelStatus(int slot, bool enable)
{
	bool ret = false;
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	unsigned int vow_ipi_buf[2];

	if (slot >= MAX_VOW_SPEAKER_MODEL || slot < 0) {
		VOWDRV_DEBUG("%s(), slot id=%d, over range\n", __func__, slot);
		return ret;
	}
	if (!vowserv.vow_speaker_model[slot].flag) {
		VOWDRV_DEBUG("%s(), this speaker model isn't load\n", __func__);
		return ret;
	}

	vow_ipi_buf[0] = vowserv.vow_speaker_model[slot].uuid;
	vow_ipi_buf[1] = vowserv.vow_speaker_model[slot].confidence_lv;

	if (enable == VOW_MODEL_STATUS_START) {
		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_BYPASS_ACK,
				      IPIMSG_VOW_MODEL_START,
				      sizeof(unsigned int) * 2, 0,
				      (char *)&vow_ipi_buf[0]);
	} else {  /* VOW_MODEL_STATUS_STOP */
		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_BYPASS_ACK,
				      IPIMSG_VOW_MODEL_STOP,
				      sizeof(unsigned int) * 2, 0,
				      (char *)&vow_ipi_buf[0]);
	}
#else
	(void)enable;
	(void)slot;
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	return ret;
}

static void vow_register_vendor_feature(int uuid)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	switch (uuid) {
	case VENDOR_ID_MTK:
		vow_register_feature(VOW_VENDOR_M_FEATURE_ID);
		break;
	case VENDOR_ID_AMAZON:
		vow_register_feature(VOW_VENDOR_A_FEATURE_ID);
		break;
	case VENDOR_ID_OTHERS:
		vow_register_feature(VOW_VENDOR_G_FEATURE_ID);
		break;
	default:
		VOWDRV_DEBUG("VENDOR ID not support\n");
		break;
	}
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
}

static void vow_deregister_vendor_feature(int uuid)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	switch (uuid) {
	case VENDOR_ID_MTK:
		vow_deregister_feature(VOW_VENDOR_M_FEATURE_ID);
		break;
	case VENDOR_ID_AMAZON:
		vow_deregister_feature(VOW_VENDOR_A_FEATURE_ID);
		break;
	case VENDOR_ID_OTHERS:
		vow_deregister_feature(VOW_VENDOR_G_FEATURE_ID);
		break;
	default:
		VOWDRV_DEBUG("VENDOR ID not support\n");
		break;
	}
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
}

static bool vow_service_SetModelStatus(bool enable, unsigned long arg)
{
	bool ret = false;
	int slot;
	struct vow_model_start_t model_start;

	if (copy_from_user((void *)&model_start,
			   (const void __user *)(arg),
			   sizeof(struct vow_model_start_t))) {
		VOWDRV_DEBUG("vow get vow_model_start data fail\n");
		return false;
	}

	if (model_start.handle > INT_MAX || model_start.handle < INT_MIN) {
		VOWDRV_DEBUG("%s(), model_start.handle will cause truncated value\n",
					__func__);
		return false;
	}

	slot = vow_service_SearchSpeakerModelWithId(model_start.handle);
	if (slot < 0) {
		VOWDRV_DEBUG("%s(), no match id\n", __func__);
		return false;
	}
	if (enable == VOW_MODEL_STATUS_START) {
		if (vowserv.vow_speaker_model[slot].enabled == 0) {
			int uuid;

			uuid = vowserv.vow_speaker_model[slot].uuid;
			vow_register_vendor_feature(uuid);
		}
		vowserv.vow_speaker_model[slot].enabled = 1;
		vowserv.vow_speaker_model[slot].confidence_lv =
			model_start.confidence_level;
		VOWDRV_DEBUG("VOW_MODEL_START, id:%d\n",
			     (int)model_start.handle);
		/* send model status to scp */
		ret = vow_service_SendModelStatus(slot, enable);
		if (ret == false)
			VOWDRV_DEBUG("vow_service_SendModelStatus, err\n");
	} else {  /* VOW_MODEL_STATUS_STOP */
		vowserv.vow_speaker_model[slot].confidence_lv =
			model_start.confidence_level;
		/* send model status to scp */
		ret = vow_service_SendModelStatus(slot, enable);
		if (ret == false)
			VOWDRV_DEBUG("vow_service_SendModelStatus, err\n");
		if (vowserv.vow_speaker_model[slot].enabled == 1) {
			int uuid;

			uuid = vowserv.vow_speaker_model[slot].uuid;
			vow_deregister_vendor_feature(uuid);
		}
		vowserv.vow_speaker_model[slot].enabled = 0;
		VOWDRV_DEBUG("VOW_MODEL_STOP, id:%d\n",
			     (int)model_start.handle);
	}
	return ret;
}

static bool vow_service_SetApAddr(unsigned long arg)
{
	unsigned long vow_info[MAX_VOW_INFO_LEN];

	if (copy_from_user((void *)(&vow_info[0]), (const void __user *)(arg),
			   sizeof(vowserv.vow_info_apuser))) {
		VOWDRV_DEBUG("vow check parameter fail\n");
		return false;
	}

	/* add return condition */
	if ((vow_info[2] == 0) || (vow_info[3] != VOW_VBUF_LENGTH) ||
	    (vow_info[4] == 0)) {
		VOWDRV_DEBUG("vow SetVBufAddr:addr_%x, size_%x, addr_%x\n",
		 (unsigned int)vow_info[2],
		 (unsigned int)vow_info[3],
		 (unsigned int)vow_info[4]);
		return false;
	}

	vowserv.voicedata_user_addr = vow_info[2];
	vowserv.voicedata_user_size = vow_info[3];
	vowserv.voicedata_user_return_size_addr = vow_info[4];

	return true;
}
static bool vow_service_SetVBufAddr(unsigned long arg)
{
	if (!(vow_service_SetApAddr(arg)))
		return false;

	VOWDRV_DEBUG("vow SetVBufAddr:addr_%x, size_%x\n",
		 (unsigned int)vowserv.vow_info_apuser[2],
		 (unsigned int)vowserv.vow_info_apuser[3]);

	/* add return condition */
	if ((vowserv.vow_info_apuser[2] == 0) ||
	    (vowserv.vow_info_apuser[3] != VOW_VBUF_LENGTH) ||
	    (vowserv.vow_info_apuser[4] == 0))
		return false;

	if (vowserv.voicedata_kernel_ptr != NULL) {
		vfree(vowserv.voicedata_kernel_ptr);
		vowserv.voicedata_kernel_ptr = NULL;
	}

	vowserv.voicedata_user_addr = vowserv.vow_info_apuser[2];
	vowserv.voicedata_user_size = vowserv.vow_info_apuser[3];
	vowserv.voicedata_user_return_size_addr = vowserv.vow_info_apuser[4];

	if (vowserv.voicedata_user_size > 0) {
		vowserv.voicedata_kernel_ptr =
		    vmalloc(vowserv.voicedata_user_size);
		return true;
	} else {
		return false;
	}
}

static bool vow_service_Enable(void)
{
	bool ret = false;

	VOWDRV_DEBUG("+%s()\n", __func__);
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	ret = vow_IPICmd_Send(AUDIO_IPI_MSG_ONLY,
			      AUDIO_IPI_MSG_BYPASS_ACK,
			      IPIMSG_VOW_ENABLE,
			      0, 0,
			      NULL);
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	VOWDRV_DEBUG("-%s():%d\n", __func__, ret);
	return ret;
}

static bool vow_service_Disable(void)
{
	bool ret = false;

	VOWDRV_DEBUG("+%s()\n", __func__);
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	ret = vow_IPICmd_Send(AUDIO_IPI_MSG_ONLY,
			      AUDIO_IPI_MSG_BYPASS_ACK,
			      IPIMSG_VOW_DISABLE,
			      0, 0,
			      NULL);
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	/* release lock */
	if (vowserv.suspend_lock == 1) {
		vowserv.suspend_lock = 0;
		/* Let AP will suspend */
		__pm_relax(vow_suspend_lock);
	}
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	vow_deregister_feature(VOW_FEATURE_ID);
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	return ret;
}

static void vow_check_boundary(unsigned int copy_len, unsigned int bound_len)
{
	if (copy_len > bound_len) {
		VOWDRV_DEBUG("[vow check]copy_len=0x%x, bound_len=0x%x\n",
			     copy_len, bound_len);
		VOW_ASSERT(0);
	}
}

#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
static void vow_interleaving(short *out_buf,
			     short *l_sample,
			     short *r_sample,
			     unsigned int buf_length)
{
	int i;
	int smpl_max;

	smpl_max = buf_length / 2;
	for (i = 0; i < smpl_max; i++) {
		*out_buf++ = *l_sample++;
		*out_buf++ = *r_sample++;
	}
}
#endif

static int vow_service_ReadVoiceData_Internal(unsigned int buf_offset,
					      unsigned int buf_length)
{
	int stop_condition = 0;

	VOW_ASSERT(vowserv.voicedata_kernel_ptr != NULL);

	if (buf_length != 0) {
		if ((vowserv.voicedata_idx + (buf_length >> 1))
		  > (vowserv.voicedata_user_size >> 1)) {
			VOWDRV_DEBUG(
			    "[vow check]data_idx=0x%x(W),buf_length=0x%x(B)\n",
			    vowserv.voicedata_idx, buf_length);
			VOWDRV_DEBUG(
		    "[vow check] user_size=0x%x(B), voice_buf_offset=0x%x(B)\n",
			    (unsigned int)vowserv.voicedata_user_size,
			    buf_offset);
			/* VOW_ASSERT(0); */
			vowserv.voicedata_idx = 0;
		}
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
		/* start interleaving L+R */
		vow_interleaving(
			&vowserv.voicedata_kernel_ptr[vowserv.voicedata_idx],
			(short *)(vowserv.voicddata_scp_ptr + buf_offset),
			(short *)(vowserv.voicddata_scp_ptr + buf_offset +
			    VOW_VOICEDATA_SIZE),
			buf_length);
		/* end interleaving*/
#else  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
		memcpy(&vowserv.voicedata_kernel_ptr[vowserv.voicedata_idx],
		       vowserv.voicddata_scp_ptr + buf_offset, buf_length);
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */

		if (buf_length > VOW_VOICE_RECORD_BIG_THRESHOLD) {
			/* means now is start to transfer */
			/* keyword buffer(64kB) to AP */
			VOWDRV_DEBUG("%s(), start tx keyword, 0x%x/0x%x\n",
				     __func__,
				     vowserv.voicedata_idx,
				     buf_length);
			vowserv.tx_keyword_start = true;
		}

#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
		/* 2 Channels */
		vowserv.voicedata_idx += buf_length;
#else
		/* 1 Channel */
		vowserv.voicedata_idx += (buf_length >> 1);
#endif
	}
	if (vowserv.voicedata_idx >= (VOW_VOICE_RECORD_BIG_THRESHOLD >> 1))
		vowserv.transfer_length = VOW_VOICE_RECORD_BIG_THRESHOLD;
	else
		vowserv.transfer_length = VOW_VOICE_RECORD_THRESHOLD;

	if (vowserv.voicedata_idx >= (vowserv.transfer_length >> 1)) {
		unsigned int ret;

		/*VOWDRV_DEBUG("TX Leng:%d, %d, [%d]\n",*/
		/*	     vowserv.voicedata_idx,*/
		/*	     buf_length,*/
		/*	     vowserv.transfer_length);*/

		ret = copy_to_user(
		      (void __user *)(vowserv.voicedata_user_return_size_addr),
		      &vowserv.transfer_length,
		      4);

		ret = copy_to_user(
		      (void __user *)vowserv.voicedata_user_addr,
		      vowserv.voicedata_kernel_ptr,
		      vowserv.transfer_length);

		/* move left data to buffer's head */
		if (vowserv.voicedata_idx > (vowserv.transfer_length >> 1)) {
			unsigned int tmp;
			unsigned int idx;

			tmp = (vowserv.voicedata_idx << 1)
			      - vowserv.transfer_length;
			vow_check_boundary(tmp, vowserv.voicedata_user_size);
			idx = (vowserv.transfer_length >> 1);
			memcpy(&vowserv.voicedata_kernel_ptr[0],
			       &vowserv.voicedata_kernel_ptr[idx],
			       tmp);
			vowserv.voicedata_idx -= idx;
		} else
			vowserv.voicedata_idx = 0;

		/* speed up voicedata transfer to hal */
		if (vowserv.voicedata_idx >= VOW_VOICE_RECORD_THRESHOLD)
			vow_service_getVoiceData();

		stop_condition = 1;
	}
	if ((vowserv.tx_keyword_start == true)
	 && (vowserv.voicedata_idx < VOW_VOICE_RECORD_THRESHOLD)) {
		/* means now is end of transfer keyword buffer(64kB) to AP */
		vowserv.tx_keyword_start = false;
		VOWDRV_DEBUG("%s(), end tx keyword, 0x%x\n",
			     __func__,
			     vowserv.voicedata_idx);
	}

	return stop_condition;
}

static void vow_service_ReadVoiceData(void)
{
	int stop_condition = 0;

	/*int rdata;*/
	while (1) {
		if (VoiceData_Wait_Queue_flag == 0)
			wait_event_interruptible(VoiceData_Wait_Queue,
						 VoiceData_Wait_Queue_flag);

		if (VoiceData_Wait_Queue_flag == 1) {
			VoiceData_Wait_Queue_flag = 0;
			if ((VowDrv_GetHWStatus() == VOW_PWR_OFF)
			 || (vowserv.recording_flag == false)) {
				vowserv.voicedata_idx = 0;
				stop_condition = 1;
				VOWDRV_DEBUG(
				    "stop read vow voice data: %d, %d\n",
				    VowDrv_GetHWStatus(),
				    vowserv.recording_flag);
			} else {
				/* To Read Voice Data from Kernel to User */
				stop_condition =
				    vow_service_ReadVoiceData_Internal(
					vowserv.voice_buf_offset,
					vowserv.voice_length);
				vowserv.voice_buf_offset = 0;
				vowserv.voice_length = 0;
			}
			if (stop_condition == 1)
				break;
		}
	}
}

static void vow_service_reset(void)
{
	int I;
	bool need_disable_vow = false;
	bool ret = false;

	VOWDRV_DEBUG("+%s()\n", __func__);
	for (I = 0; I < MAX_VOW_SPEAKER_MODEL; I++) {
		if (vowserv.vow_speaker_model[I].enabled  == 1) {
			int uuid;

			/* need to close it */
			ret = vow_service_SendModelStatus(
				I, VOW_MODEL_STATUS_STOP);
			if (ret == false)
				VOWDRV_DEBUG("%s(), err\n", __func__);

			uuid = vowserv.vow_speaker_model[I].uuid;
			vow_deregister_vendor_feature(uuid);
			vowserv.vow_speaker_model[I].enabled = 0;
			need_disable_vow = true;
		}
		if (vowserv.vow_speaker_model[I].flag  == 1) {
			int id;

			/* need to clear model */
			id = vowserv.vow_speaker_model[I].id;
			vow_service_ReleaseSpeakerModel(id);
		}
	}
	if (need_disable_vow)
		vow_service_Disable();

	VOWDRV_DEBUG("-%s()\n", __func__);
}

static bool vow_stop_dump_wait(void)
{
	int timeout = 0;

	while (1) {
		msleep(VOW_WAITCHECK_INTERVAL_MS);
		if (timeout++ >= VOW_STOP_DUMP_WAIT)
			return false;
	}
	return true;
}

static int vow_pcm_dump_notify(bool enable)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	unsigned int vow_ipi_buf[5] = {0};
	bool ret;

	/* if scp reset happened, need re-send PCM dump IPI to SCP again */
	if (enable == true) {
		/* dump flag */
		vow_ipi_buf[1] = vowserv.dump_pcm_flag;
		/* TOTAL dram resrved size for barge in dump */
		vow_ipi_buf[0] = bargein_resv_dram.size;
		/* address for SCP using */
		vow_ipi_buf[2] = bargein_resv_dram.phy_addr;

		VOWDRV_DEBUG(
		"[BargeIn]dump on, dump flag:%d, resv sz:0x%x, addr:0x%x\n",
			     vow_ipi_buf[1],
			     vow_ipi_buf[0],
			     vow_ipi_buf[2]);
		/* TOTAL dram resrved size for recog data dump */
		vow_ipi_buf[3] = recog_resv_dram.size;
		/* address for SCP using */
		vow_ipi_buf[4] = recog_resv_dram.phy_addr;

		VOWDRV_DEBUG(
		"[Recog]dump on, dump flag:%d, resv sz:0x%x, addr:0x%x\n",
			    vow_ipi_buf[1],
			    vow_ipi_buf[3],
			    vow_ipi_buf[4]);

		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_BYPASS_ACK,
				      IPIMSG_VOW_PCM_DUMP_ON,
				      sizeof(unsigned int) * 5, 0,
				      (char *)&vow_ipi_buf[0]);

		if (ret == 0)
			VOWDRV_DEBUG("PCM_DUMP_ON ipi send error\n");
	} else {
		/* dump flag */
		vow_ipi_buf[1] = vowserv.dump_pcm_flag;
		/* TOTAL dram resrved size for barge in dump */
		vow_ipi_buf[0] = bargein_resv_dram.size;
		/* address for SCP using */
		vow_ipi_buf[2] = bargein_resv_dram.phy_addr;

		VOWDRV_DEBUG(
		"[BargeIn]dump off, dump flag:%d, resv sz:0x%x, addr:0x%x\n",
			     vow_ipi_buf[1],
			     vow_ipi_buf[0],
			     vow_ipi_buf[2]);
		/* TOTAL dram resrved size for recog data dump */
		vow_ipi_buf[3] = recog_resv_dram.size;
		/* address for SCP using */
		vow_ipi_buf[4] = recog_resv_dram.phy_addr;

		VOWDRV_DEBUG(
		"[Recog]dump off, dump flag:%d, resv sz:0x%x, addr:0x%x\n",
			    vow_ipi_buf[1],
			    vow_ipi_buf[3],
			    vow_ipi_buf[4]);

		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_BYPASS_ACK,
				      IPIMSG_VOW_PCM_DUMP_OFF,
				      sizeof(unsigned int) * 5, 0,
				      (char *)&vow_ipi_buf[0]);

		if (ret == 0)
			VOWDRV_DEBUG("PCM_DUMP_OFF ipi send error\n");
	}
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */
	return 0;
}

static int vow_pcm_dump_set(bool enable)
{
	VOWDRV_DEBUG("%s = %d, %d\n", __func__,
		     vowserv.dump_pcm_flag,
		     (unsigned int)enable);
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	bargein_resv_dram.vir_addr =
	    (char *)(scp_get_reserve_mem_virt(VOW_BARGEIN_MEM_ID))
	    + VOW_BARGEIN_DUMP_OFFSET;
	bargein_resv_dram.phy_addr =
	    scp_get_reserve_mem_phys(VOW_BARGEIN_MEM_ID)
	    + VOW_BARGEIN_DUMP_OFFSET;
	bargein_resv_dram.size = VOW_BARGEIN_DUMP_SIZE;

	VOWDRV_DEBUG("[Barge]vir: %p, phys: 0x%x\n",
		     bargein_resv_dram.vir_addr,
		     (unsigned int)bargein_resv_dram.phy_addr);
	recog_resv_dram.vir_addr =
	    (char *)(scp_get_reserve_mem_virt(VOW_MEM_ID))
	    + VOW_RECOGDATA_OFFSET;
	recog_resv_dram.phy_addr =
	    scp_get_reserve_mem_phys(VOW_MEM_ID)
	    + VOW_RECOGDATA_OFFSET;
	recog_resv_dram.size = VOW_RECOGDATA_SIZE;

	VOWDRV_DEBUG("[Recog]vir: %p, phys: 0x%x\n",
		     recog_resv_dram.vir_addr,
		     (unsigned int)recog_resv_dram.phy_addr);

	if ((vowserv.dump_pcm_flag == false) && (enable == true)) {
		vowserv.dump_pcm_flag = true;
		vow_service_OpenDumpFile();

		vow_pcm_dump_notify(true);
	} else if ((vowserv.dump_pcm_flag == true) && (enable == false)) {
		vowserv.dump_pcm_flag = false;

		vow_pcm_dump_notify(false);

		vow_stop_dump_wait();
		vow_service_CloseDumpFile();
	}
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */
	return 0;
}

static void vow_service_OpenDumpFile(void)
{
	VOWDRV_DEBUG("+%s() %d\n", __func__, b_enable_dump);
	/* only enable when debug pcm dump on */
	__pm_stay_awake(&pcm_dump_wake_lock);
	vow_service_OpenDumpFile_internal();

	if (dump_queue == NULL) {
		dump_queue = kmalloc(sizeof(struct dump_queue_t), GFP_KERNEL);
		if (dump_queue != NULL)
			memset_io(dump_queue, 0, sizeof(struct dump_queue_t));
	}
	if (!pcm_dump_task) {
		pcm_dump_task = kthread_create(vow_pcm_dump_kthread,
					       NULL,
					       "vow_pcm_dump_kthread");
		if (IS_ERR(pcm_dump_task))
			VOWDRV_DEBUG("can not create pcm dump kthread\n");

		b_enable_dump = true;
		wake_up_process(pcm_dump_task);
	}
	bargein_dump_data_routine_cnt_pass = 0;
	vowserv.bargein_dump_cnt1 = 0;
	vowserv.bargein_dump_cnt2 = 0;
	recog_dump_data_routine_cnt_pass = 0;
	vowserv.recog_dump_cnt1 = 0;
	vowserv.recog_dump_cnt2 = 0;
	VOWDRV_DEBUG("-%s() %d\n", __func__, b_enable_dump);
}

static void vow_service_CloseDumpFile(void)
{
	VOWDRV_DEBUG("+%s() %d\n", __func__, b_enable_dump);
	if (b_enable_dump == false)
		return;

	b_enable_dump = false;
	if (pcm_dump_task) {
		kthread_stop(pcm_dump_task);
		pcm_dump_task = NULL;
	}
	VOWDRV_DEBUG("[Recog] dump_queue = %p\n", dump_queue);
	VOWDRV_DEBUG("[BargeIn] bargein_pass: %d\n",
		bargein_dump_data_routine_cnt_pass);
	VOWDRV_DEBUG("[BargeIn] bargein dump cnt %d %d\n",
		     vowserv.bargein_dump_cnt1,
		     vowserv.bargein_dump_cnt2);
	VOWDRV_DEBUG("[Recog] recog_pass: %d\n",
		recog_dump_data_routine_cnt_pass);
	VOWDRV_DEBUG("[Recog] recog dump cnt %d %d\n",
		     vowserv.recog_dump_cnt1,
		     vowserv.recog_dump_cnt2);
	if (dump_queue != NULL) {
		kfree(dump_queue);
		dump_queue = NULL;
	}

	vow_service_CloseDumpFile_internal();

	__pm_relax(&pcm_dump_wake_lock);
	VOWDRV_DEBUG("-%s() %d\n", __func__, b_enable_dump);
}

static void vow_service_OpenDumpFile_internal(void)
{
	struct timespec curr_tm;
	char string_time[16];
	char string_input_pcm[16] = "input_pcm.pcm";
	char string_echo_pcm[16] = "echo_ref.pcm";
	char string_delay_info[16] = "delay_info";
	char path_input_pcm[64];
	char path_echo_ref[64];
	char path_delay_info[64];
	char string_recog[16] = "recog.pcm";
	char path_recog[64];

	VOWDRV_DEBUG("+%s()\n", __func__);
	memset(&curr_tm, 0, sizeof(struct timespec));
	getnstimeofday(&curr_tm);

	memset(string_time, '\0', 16);
	if (sprintf(string_time, "%.2lu_%.2lu_%.2lu_%.3lu",
		(8 + (curr_tm.tv_sec / 3600)) % (24),
		(curr_tm.tv_sec / 60) % (60),
		(curr_tm.tv_sec % 60),
		(curr_tm.tv_nsec % 1000)) < 0) {
		VOWDRV_DEBUG("%s(), sprintf failed\n", __func__);
	}
	if (sprintf(path_input_pcm, "%s/%s_%s",
		DUMP_PCM_DATA_PATH, string_time, string_input_pcm) < 0) {
		VOWDRV_DEBUG("%s(), sprintf failed\n", __func__);
	}
	VOWDRV_DEBUG("[BargeIn] %s path_input_pcm= %s\n", __func__,
		     path_input_pcm);

	if (sprintf(path_echo_ref, "%s/%s_%s",
		DUMP_PCM_DATA_PATH, string_time, string_echo_pcm) < 0) {
		VOWDRV_DEBUG("%s(), sprintf failed\n", __func__);
	}
	VOWDRV_DEBUG("[BargeIn] %s path_input_pcm= %s\n", __func__,
		     path_echo_ref);
	if (sprintf(path_delay_info, "%s/%s_%s",
		DUMP_PCM_DATA_PATH, string_time, string_delay_info) < 0) {
		VOWDRV_DEBUG("%s(), sprintf failed\n", __func__);
	}
	VOWDRV_DEBUG("[BargeIn] %s path_input_pcm= %s\n", __func__,
		     path_delay_info);

	if (sprintf(path_recog, "%s/%s_%s",
		DUMP_PCM_DATA_PATH, string_time, string_recog) < 0) {
		VOWDRV_DEBUG("%s(), sprintf failed\n", __func__);
	}
	VOWDRV_DEBUG("[Recog] %s path_recog= %s\n", __func__,
		     path_recog);

	file_recog_data = NULL;
	file_recog_data_open = false;
	file_bargein_pcm_input = NULL;
	file_bargein_pcm_input_open = false;
	file_bargein_echo_ref = NULL;
	file_bargein_echo_ref_open = false;
	file_bargein_delay_info = NULL;
	file_bargein_delay_info_open = false;

	file_bargein_pcm_input = filp_open(path_input_pcm,
					   O_CREAT | O_WRONLY | O_LARGEFILE,
					   0);
	if (IS_ERR(file_bargein_pcm_input)) {
		VOWDRV_DEBUG("[BargeIn] pcm_input:%d, path_input_pcm=%s\n",
			     (int)PTR_ERR(file_bargein_pcm_input),
			     path_input_pcm);
		return;
	}
	file_bargein_pcm_input_open = true;

	file_bargein_echo_ref = filp_open(path_echo_ref,
					  O_CREAT | O_WRONLY | O_LARGEFILE,
					  0);
	if (IS_ERR(file_bargein_echo_ref)) {
		VOWDRV_DEBUG("[BargeIn] echo_ref:%d, path_echo_ref=%s\n",
			     (int)PTR_ERR(file_bargein_echo_ref),
			     path_echo_ref);
		return;
	}
	file_bargein_echo_ref_open = true;

	file_bargein_delay_info = filp_open(path_delay_info,
					    O_CREAT | O_WRONLY | O_LARGEFILE,
					    0);
	if (IS_ERR(file_bargein_delay_info)) {
		VOWDRV_DEBUG(
		"[BargeIn] file_bargein_delay_info:%d, path_delay_info = %s\n",
		(int)PTR_ERR(file_bargein_delay_info),
		path_delay_info);
		return;
	}
	file_bargein_delay_info_open = true;

	file_recog_data = filp_open(path_recog,
				    O_CREAT | O_WRONLY | O_LARGEFILE,
				    0);
	if (IS_ERR(file_recog_data)) {
		VOWDRV_DEBUG(
		"[BargeIn] file_recog_data:%d, path_recog = %s\n",
		(int)PTR_ERR(file_recog_data),
		path_recog);
		return;
	}
	file_recog_data_open = true;
	VOWDRV_DEBUG("-%s()\n", __func__);
}

static void vow_service_CloseDumpFile_internal(void)
{
	VOWDRV_DEBUG("+%s()\n", __func__);
	if (file_bargein_pcm_input_open) {
		file_bargein_pcm_input_open = false;
		if (!IS_ERR(file_bargein_pcm_input)) {
			filp_close(file_bargein_pcm_input, NULL);
			file_bargein_pcm_input = NULL;
		}
	}
	if (file_bargein_echo_ref_open) {
		file_bargein_echo_ref_open = false;
		if (!IS_ERR(file_bargein_echo_ref)) {
			filp_close(file_bargein_echo_ref, NULL);
			file_bargein_echo_ref = NULL;
		}
	}
	if (file_bargein_delay_info_open) {
		file_bargein_delay_info_open = false;
		if (!IS_ERR(file_bargein_delay_info)) {
			filp_close(file_bargein_delay_info, NULL);
			file_bargein_delay_info = NULL;
		}
	}
	if (file_recog_data_open) {
		file_recog_data_open = false;
		if (!IS_ERR(file_recog_data)) {
			filp_close(file_recog_data, NULL);
			file_recog_data = NULL;
		}
	}
	VOWDRV_DEBUG("-%s()\n", __func__);
}

static int vow_pcm_dump_kthread(void *data)
{
	int ret = 0;
	int size = 0, writedata = 0;
	uint8_t current_idx = 0;
	struct pcm_dump_t *pcm_dump = NULL;
	struct dump_package_t *dump_package = NULL;
	mm_segment_t old_fs;

	struct sched_param param = {.sched_priority = 85 };

	sched_setscheduler(current, SCHED_RR, &param);

	/* VOWDRV_DEBUG("[BargeIn] dump_queue = %p\n", dump_queue); */

	while (b_enable_dump && !kthread_should_stop()) {
		spin_lock(&vowdrv_lock);
		if (dump_queue->idx_r != dump_queue->idx_w) {
			current_idx = dump_queue->idx_r;
			dump_queue->idx_r++;
			spin_unlock(&vowdrv_lock);
		} else {
			spin_unlock(&vowdrv_lock);
			ret = wait_event_interruptible(
			      wq_dump_pcm,
			      (dump_queue->idx_r != dump_queue->idx_w) ||
			      (b_enable_dump == false));
			if (ret == -ERESTARTSYS) {
				ret = -EINTR;
				break;
			}
			if (b_enable_dump == false)
				break;

			current_idx = dump_queue->idx_r;
			dump_queue->idx_r++;
		}

		if (vowserv.split_dumpfile_flag) {
			vow_service_CloseDumpFile_internal();
			vow_service_OpenDumpFile_internal();
			VOWDRV_DEBUG("Split audio dump done\n");
			vowserv.split_dumpfile_flag = false;
		}

		dump_package = &dump_queue->dump_package[current_idx];

		/* VOWDRV_DEBUG("[BargeIn] current_idx = %d\n", current_idx); */
		switch (dump_package->dump_data_type) {
		case DUMP_BARGEIN: {
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
			short *out_buf;
			/* DRAM to kernel buffer and sample interleaving */
			vow_interleaving(
				vowserv.interleave_pcmdata_ptr,
				(short *)(bargein_resv_dram.vir_addr +
				    dump_package->mic_offset),
				(short *)(bargein_resv_dram.vir_addr +
				    dump_package->mic_offset_R),
				dump_package->mic_data_size);

			size = (dump_package->mic_data_size) * 2;
			writedata = size;

			out_buf = vowserv.interleave_pcmdata_ptr;
			while (size > 0) {
				if (file_bargein_pcm_input_open &&
				    !IS_ERR(file_bargein_pcm_input)) {
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					ret = vfs_write(file_bargein_pcm_input,
					    (char __user *)out_buf,
					    writedata,
					    &file_bargein_pcm_input->f_pos);
					set_fs(old_fs);
					if (!ret) {
						VOWDRV_DEBUG(
						"[Bargein]vfs write failed\n");
					}
				}
				size -= writedata;
				pcm_dump++;
			}
#else  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
			/* Bargein dump Mic input data */
			size = dump_package->mic_data_size;
			writedata = size;
			pcm_dump = (struct pcm_dump_t *)
				   (bargein_resv_dram.vir_addr
				   + dump_package->mic_offset);
			while (size > 0) {
				if (file_bargein_pcm_input_open &&
				    !IS_ERR(file_bargein_pcm_input)) {
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					ret = vfs_write(file_bargein_pcm_input,
					    (char __user *)pcm_dump->decode_pcm,
					    writedata,
					    &file_bargein_pcm_input->f_pos);
					set_fs(old_fs);
					if (!ret) {
						VOWDRV_DEBUG(
						"[Bargein]vfs write failed\n");
					}
				}
				size -= writedata;
				pcm_dump++;
			}
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
			/* Bargein dump echo data */
			size = dump_package->echo_data_size;
			writedata = size;
			pcm_dump = (struct pcm_dump_t *)
				   (bargein_resv_dram.vir_addr
				   + dump_package->echo_offset);
			vowserv.bargein_dump_cnt2++;
			while (size > 0) {
				if (file_bargein_echo_ref_open &&
				    !IS_ERR(file_bargein_echo_ref)) {
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					ret = vfs_write(file_bargein_echo_ref,
					    (char __user *)pcm_dump->decode_pcm,
					    writedata,
					    &file_bargein_echo_ref->f_pos);
					set_fs(old_fs);
					if (!ret) {
						VOWDRV_DEBUG(
						"[Bargein]vfs write failed\n");
					}
				}
				size -= writedata;
				pcm_dump++;
			}
		}
		if (bargein_dump_info_flag) {
			if (file_bargein_delay_info_open &&
			    !IS_ERR(file_bargein_delay_info)) {
				uint32_t *ptr32;

				old_fs = get_fs();
				set_fs(KERNEL_DS);
				ptr32 = &vowserv.dump_frm_cnt;
				ret = vfs_write(file_bargein_delay_info,
					    (char __user *)ptr32,
					    sizeof(uint32_t),
					    &file_bargein_delay_info->f_pos);
				if (!ret)
					VOWDRV_DEBUG("vfs write failed\n");
				ptr32 = &vowserv.voice_sample_delay;
				ret = vfs_write(file_bargein_delay_info,
					    (char __user *)ptr32,
					    sizeof(uint32_t),
					    &file_bargein_delay_info->f_pos);
				if (!ret)
					VOWDRV_DEBUG("vfs write failed\n");
				set_fs(old_fs);
				bargein_dump_info_flag = false;
			}
		}
			break;
		case DUMP_RECOG: {
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
			short *out_buf;
			/* DRAM to kernel buffer and sample interleaving */
			vow_interleaving(
				vowserv.interleave_pcmdata_ptr,
				(short *)(recog_resv_dram.vir_addr +
				    dump_package->recog_data_offset),
				(short *)(recog_resv_dram.vir_addr +
				    dump_package->recog_data_offset_R),
				dump_package->recog_data_size);

			size = (dump_package->recog_data_size) * 2;
			writedata = size;

			out_buf = vowserv.interleave_pcmdata_ptr;
			while (size > 0) {
				if (file_recog_data_open &&
				    !IS_ERR(file_recog_data)) {
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					ret = vfs_write(file_recog_data,
					    (char __user *)out_buf,
					    writedata,
					    &file_recog_data->f_pos);
					set_fs(old_fs);
					if (!ret) {
						VOWDRV_DEBUG(
						"[Recog]vfs write failed\n");
					}
				}
				size -= writedata;
				pcm_dump++;
			}
#else  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
			/* Recog dump data */
			size = dump_package->recog_data_size;
			writedata = size;
			pcm_dump = (struct pcm_dump_t *)
				   (recog_resv_dram.vir_addr
				   + dump_package->recog_data_offset);
			while (size > 0) {
				if (file_recog_data_open &&
				    !IS_ERR(file_recog_data)) {
					old_fs = get_fs();
					set_fs(KERNEL_DS);
					ret = vfs_write(file_recog_data,
					    (char __user *)pcm_dump->decode_pcm,
					    writedata,
					    &file_recog_data->f_pos);
					set_fs(old_fs);
					if (!ret) {
						VOWDRV_DEBUG(
						"[Recog]vfs write failed\n");
					}
				}
				size -= writedata;
				pcm_dump++;
			}
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
		}
			break;
		default:
			break;
		}
	}
	VOWDRV_DEBUG("%s, exit\n", __func__);
	return 0;
}

static void recog_dump_routine(struct work_struct *ws)
{
	struct dump_work_t *dump_work = NULL;
	uint32_t offset = 0;
	uint32_t data_size = 0;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	uint32_t offset_R = 0;
	uint32_t data_size_R = 0;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */

	dump_work = container_of(ws, struct dump_work_t, work);

	offset = dump_work->recog_data_offset;
	data_size = dump_work->recog_data_size;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	offset_R = dump_work->recog_data_offset_R;
	data_size_R = dump_work->recog_data_size_R;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */

	spin_lock(&vowdrv_lock);
	dump_queue->dump_package[dump_queue->idx_w].dump_data_type =
	    DUMP_RECOG;
	dump_queue->dump_package[dump_queue->idx_w].recog_data_offset =
	    offset;
	dump_queue->dump_package[dump_queue->idx_w].recog_data_size =
	    data_size;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	dump_queue->dump_package[dump_queue->idx_w].recog_data_offset_R =
	    offset_R;
	dump_queue->dump_package[dump_queue->idx_w].recog_data_size_R =
	    data_size_R;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */

	dump_queue->idx_w++;
	spin_unlock(&vowdrv_lock);
	vowserv.recog_dump_cnt1++;

	wake_up_interruptible(&wq_dump_pcm);
}

static void bargein_dump_routine(struct work_struct *ws)
{
	struct dump_work_t *dump_work = NULL;
	uint32_t mic_offset = 0;
	uint32_t mic_data_size = 0;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	uint32_t mic_offset_R = 0;
	uint32_t mic_data_size_R = 0;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
	uint32_t echo_offset = 0;
	uint32_t echo_data_size = 0;

	dump_work = container_of(ws, struct dump_work_t, work);

	mic_offset = dump_work->mic_offset;
	mic_data_size = dump_work->mic_data_size;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	mic_offset_R = dump_work->mic_offset_R;
	mic_data_size_R = dump_work->mic_data_size_R;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
	echo_offset = dump_work->echo_offset;
	echo_data_size = dump_work->echo_data_size;

	spin_lock(&vowdrv_lock);
	dump_queue->dump_package[dump_queue->idx_w].dump_data_type =
	    DUMP_BARGEIN;
	dump_queue->dump_package[dump_queue->idx_w].mic_offset =
	    mic_offset;
	dump_queue->dump_package[dump_queue->idx_w].mic_data_size =
	    mic_data_size;
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	dump_queue->dump_package[dump_queue->idx_w].mic_offset_R =
	    mic_offset_R;
	dump_queue->dump_package[dump_queue->idx_w].mic_data_size_R =
	    mic_data_size_R;
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */
	dump_queue->dump_package[dump_queue->idx_w].echo_offset =
	    echo_offset;
	dump_queue->dump_package[dump_queue->idx_w].echo_data_size =
	    echo_data_size;

	dump_queue->idx_w++;
	spin_unlock(&vowdrv_lock);
	vowserv.bargein_dump_cnt1++;

	wake_up_interruptible(&wq_dump_pcm);
}

static void vow_pcm_dump_init(void)
{
	VOWDRV_DEBUG("[Recog] %s()\n", __func__);

	dump_workqueue[DUMP_RECOG] =
	    create_workqueue("dump_recog_data");
	if (dump_workqueue[DUMP_RECOG] == NULL) {
		VOWDRV_DEBUG("[Recog] dump_workqueue[DUMP_RECOG] = %p\n",
			     dump_workqueue[DUMP_RECOG]);
	}
	VOW_ASSERT(dump_workqueue[DUMP_RECOG] != NULL);
	dump_workqueue[DUMP_BARGEIN] =
	    create_workqueue("dump_bargein_recho_ref");
	if (dump_workqueue[DUMP_BARGEIN] == NULL) {
		VOWDRV_DEBUG("[BargeIn] dump_workqueue[recho_ref] = %p\n",
			     dump_workqueue[DUMP_BARGEIN]);
	}
	VOW_ASSERT(dump_workqueue[DUMP_BARGEIN] != NULL);

	INIT_WORK(&dump_work[DUMP_BARGEIN].work,
		  bargein_dump_routine);
	INIT_WORK(&dump_work[DUMP_RECOG].work,
		  recog_dump_routine);

	init_waitqueue_head(&wq_dump_pcm);

	pcm_dump_task = NULL;

#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	if (vowserv.interleave_pcmdata_ptr != NULL) {
		vfree(vowserv.interleave_pcmdata_ptr);
		vowserv.interleave_pcmdata_ptr = NULL;
	}
	/* Temp buffer for doing DUALMIC L/R channels interleave */
	vowserv.interleave_pcmdata_ptr =
		vmalloc(VOW_PCM_DUMP_BYTE_SIZE << 1);
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */

}

static void vow_pcm_dump_deinit(void)
{
	int i = 0;

	VOWDRV_DEBUG("[BargeIn] %s()\n", __func__);

	for (i = 0; i < NUM_DUMP_DATA; i++) {
		if (dump_workqueue[i]) {
			flush_workqueue(dump_workqueue[i]);
			destroy_workqueue(dump_workqueue[i]);
			dump_workqueue[i] = NULL;
		}
	}
#if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT)
	if (vowserv.interleave_pcmdata_ptr != NULL) {
		vfree(vowserv.interleave_pcmdata_ptr);
		vowserv.interleave_pcmdata_ptr = NULL;
	}
#endif  /* #if IS_ENABLED(CONFIG_MTK_VOW_DUAL_MIC_SUPPORT) */

}

/*****************************************************************************
 * VOW CONTROL FUNCTIONS
 *****************************************************************************/

static int VowDrv_SetHWStatus(int status)
{
	int ret = 0;

	VOWDRV_DEBUG("%s():set:%x, cur:%x\n",
		     __func__, status, vowserv.pwr_status);
	if ((status < NUM_OF_VOW_PWR_STATUS) && (status >= VOW_PWR_OFF)) {
		spin_lock(&vowdrv_lock);
		vowserv.pwr_status = status;
		spin_unlock(&vowdrv_lock);
	} else {
		VOWDRV_DEBUG("error input:%d\n", status);
		ret = -1;
	}
	return ret;
}

static int VowDrv_GetHWStatus(void)
{
	int ret = 0;

	spin_lock(&vowdrv_lock);
	ret = vowserv.pwr_status;
	spin_unlock(&vowdrv_lock);
	return ret;
}

int VowDrv_EnableHW(int status)
{
	int ret = 0;
	int pwr_status = 0;

	VOWDRV_DEBUG("%s():%x\n", __func__, status);

	if (status < 0) {
		VOWDRV_DEBUG("%s(), error input: %x\n", __func__, status);
		ret = -1;
	} else {
		pwr_status = (status == 0)?VOW_PWR_OFF : VOW_PWR_ON;

		if (pwr_status == VOW_PWR_OFF) {
			/* reset the transfer limitation to */
			/* avoid obstructing phase2.5 transferring */
			if (vowserv.tx_keyword_start == true)
				vowserv.tx_keyword_start = false;
		}
		if (pwr_status == VOW_PWR_ON) {
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
			vow_register_feature(VOW_FEATURE_ID);
#else
			VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
			/* clear enter_phase3_cnt */
			vowserv.enter_phase3_cnt = 0;
		}
		VowDrv_SetHWStatus(pwr_status);
	}
	return ret;
}

int VowDrv_ChangeStatus(void)
{
	VowDrv_Wait_Queue_flag = 1;
	wake_up_interruptible(&VowDrv_Wait_Queue);
	return 0;
}

void VowDrv_SetSmartDevice(bool enable)
{
	unsigned int eint_num;
	unsigned int ints[2] = {0, 0};
	unsigned int vow_ipi_buf[2];
	bool ret;

	if (!vow_check_scp_status()) {
		VOWDRV_DEBUG("SCP is off, do not support VOW\n");
		return;
	}

	VOWDRV_DEBUG("%s():%x\n", __func__, enable);
	if (vowserv.node) {
		/* query eint number from device tree */
		ret = of_property_read_u32_array(vowserv.node,
						 "debounce",
						 ints,
						 ARRAY_SIZE(ints));
		if (ret != 0) {
			VOWDRV_DEBUG("%s(), no debounce node, ret=%d\n",
				     __func__, ret);
			return;
		}

		eint_num = ints[0];

		if (enable == false)
			eint_num = 0xFF;
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
		vow_ipi_buf[0] = enable;
		vow_ipi_buf[1] = eint_num;
		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_BYPASS_ACK,
				      IPIMSG_VOW_SET_SMART_DEVICE,
				      sizeof(unsigned int) * 2, 0,
				      (char *)&vow_ipi_buf[0]);
		if (ret == 0) {
			VOWDRV_DEBUG(
			    "IPIMSG_VOW_SET_SMART_DEVICE ipi send error\n\r");
		}
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	} else {
		/* no node here */
		VOWDRV_DEBUG("there is no node\n");
	}
}

void VowDrv_SetSmartDevice_GPIO(bool enable)
{
	int ret = 0;

	if (vowserv.node) {
		if (enable == false) {
			VOWDRV_DEBUG("VowDrv_SetSmartDev_gpio:OFF\n");
			ret = pinctrl_select_state(vowserv.pinctrl,
						   vowserv.pins_eint_off);
			if (ret) {
				/* pinctrl setting error */
				VOWDRV_DEBUG(
				"error, can not set gpio vow pins_eint_off\n");
			}
		} else {
			VOWDRV_DEBUG("VowDrv_SetSmartDev_gpio:ON\n");
			ret = pinctrl_select_state(vowserv.pinctrl,
						   vowserv.pins_eint_on);
			if (ret) {
				/* pinctrl setting error */
				VOWDRV_DEBUG(
				"error, can not set gpio vow pins_eint_on\n");
			}
		}
	} else {
		/* no node here */
		VOWDRV_DEBUG("there is no node\n");
	}
}

static bool VowDrv_SetFlag(int type, unsigned int set)
{
	bool ret = false;
	unsigned int vow_ipi_buf[2];

	VOWDRV_DEBUG("%s(), type:%x, set:%x\n", __func__, type, set);
	vow_ipi_buf[0] = type;
	vow_ipi_buf[1] = set;

#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
			      AUDIO_IPI_MSG_NEED_ACK,
			      IPIMSG_VOW_SET_FLAG,
			      sizeof(unsigned int) * 2, 0,
			      (char *)&vow_ipi_buf[0]);
	if (ret == 0)
		VOWDRV_DEBUG("IPIMSG_VOW_SET_FLAG ipi send error\n\r");
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	return ret;
}

void VowDrv_SetDmicLowPower(bool enable)
{
	VowDrv_SetFlag(VOW_FLAG_DMIC_LOWPOWER, enable);
}

static bool VowDrv_SetMtkifType(unsigned int type)
{
	bool ret = false;

	if (type == 0)
		VOWDRV_DEBUG("error, wrong MTKIF Type\n\r");
	vowserv.mtkif_type = type;
	ret = VowDrv_SetFlag(VOW_FLAG_MTKIF_TYPE, type);

	return ret;
}

void VowDrv_SetPeriodicEnable(bool enable)
{
	VowDrv_SetFlag(VOW_FLAG_PERIODIC_ENABLE, enable);
}

static ssize_t VowDrv_GetPhase1Debug(struct device *kobj,
				     struct device_attribute *attr,
				     char *buf)
{
	unsigned int stat;
	char cstr[35];
	int size = sizeof(cstr);

	stat = (vowserv.force_phase_stage == FORCE_PHASE1) ? 1 : 0;

	return snprintf(buf, size, "Force Phase1 Setting = %s\n",
			(stat == 0x1) ? "YES" : "NO");
}

static ssize_t VowDrv_SetPhase1Debug(struct device *kobj,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	VowDrv_SetFlag(VOW_FLAG_FORCE_PHASE1_DEBUG, enable);
	return n;
}
DEVICE_ATTR(vow_SetPhase1,
	    0644, /*S_IWUSR | S_IRUGO*/
	    VowDrv_GetPhase1Debug,
	    VowDrv_SetPhase1Debug);

static ssize_t VowDrv_GetPhase2Debug(struct device *kobj,
				     struct device_attribute *attr,
				     char *buf)
{
	unsigned int stat;
	char cstr[35];
	int size = sizeof(cstr);

	stat = (vowserv.force_phase_stage == FORCE_PHASE2) ? 1 : 0;

	return snprintf(buf, size, "Force Phase2 Setting = %s\n",
			(stat == 0x1) ? "YES" : "NO");
}

static ssize_t VowDrv_SetPhase2Debug(struct device *kobj,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	VowDrv_SetFlag(VOW_FLAG_FORCE_PHASE2_DEBUG, enable);
	return n;
}
DEVICE_ATTR(vow_SetPhase2,
	    0644, /*S_IWUSR | S_IRUGO*/
	    VowDrv_GetPhase2Debug,
	    VowDrv_SetPhase2Debug);

static ssize_t VowDrv_GetDualMicDebug(struct device *kobj,
				     struct device_attribute *attr,
				     char *buf)
{
	unsigned int stat;
	char cstr[35];
	int size = sizeof(cstr);

	stat = (vowserv.scp_vow_lch == true) ? 1 : 0;

	return snprintf(buf, size, "Dual mic L channel = %s\n",
			(stat == 0x1) ? "YES" : "NO");
}

static ssize_t VowDrv_SetDualMicDebug(struct device *kobj,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	vowserv.scp_vow_lch = (enable == 1) ? true : false;

	VowDrv_SetFlag(VOW_FLAG_DUAL_MIC_LCH, enable);
	return n;
}
DEVICE_ATTR(vow_DualMicLch,
	    0644, /*S_IWUSR | S_IRUGO*/
	    VowDrv_GetDualMicDebug,
	    VowDrv_SetDualMicDebug);

static ssize_t VowDrv_GetSplitDumpFile(struct device *kobj,
				     struct device_attribute *attr,
				     char *buf)
{
	unsigned int stat;
	char cstr[35];
	int size = sizeof(cstr);

	stat = (vowserv.split_dumpfile_flag == true) ? 1 : 0;

	return snprintf(buf, size, "Split dump file = %s\n",
			(stat == 0x1) ? "YES" : "NO");
}

static ssize_t VowDrv_SetSplitDumpFile(struct device *kobj,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	VOWDRV_DEBUG("%s(),enable=%d\n", __func__, enable);
	vowserv.split_dumpfile_flag = (enable == 1) ? true : false;

	return n;
}
DEVICE_ATTR(vow_SplitDumpFile,
	    0644, /*S_IWUSR | S_IRUGO*/
	    VowDrv_GetSplitDumpFile,
	    VowDrv_SetSplitDumpFile);


static ssize_t VowDrv_SetBargeInDebug(struct device *kobj,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	VowDrv_SetBargeIn(enable, 1); /* temp fix irq */
	return n;
}
DEVICE_ATTR(vow_SetBargeIn,
	    0200, /*S_IWUSR*/
	    NULL,
	    VowDrv_SetBargeInDebug);

static bool VowDrv_SetBargeIn(unsigned int set, unsigned int irq_id)
{
	bool ret = false;
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	unsigned int vow_ipi_buf[1];

	vow_ipi_buf[0] = irq_id;

	VOWDRV_DEBUG("%s(), set = %d, irq = %d\n", __func__, set, irq_id);
	if (set == 1) {
		vow_register_feature(VOW_BARGEIN_FEATURE_ID);
		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_NEED_ACK,
				      IPIMSG_VOW_SET_BARGEIN_ON,
				      sizeof(unsigned int) * 1, 0,
				      (char *)&vow_ipi_buf[0]);
	} else if (set == 0) {
		ret = vow_IPICmd_Send(AUDIO_IPI_PAYLOAD,
				      AUDIO_IPI_MSG_NEED_ACK,
				      IPIMSG_VOW_SET_BARGEIN_OFF,
				      sizeof(unsigned int) * 1, 0,
				      (char *)&vow_ipi_buf[0]);
		vow_deregister_feature(VOW_BARGEIN_FEATURE_ID);
	} else {
		VOWDRV_DEBUG("Adb comment error\n\r");
	}
	if (ret == 0)
		VOWDRV_DEBUG("IPIMSG_BARGE_IN(%d) ipi send error\n", set);
#else
	VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
	return ret;
}

static ssize_t VowDrv_GetBypassPhase3Flag(struct device *kobj,
					  struct device_attribute *attr,
					  char *buf)
{
	unsigned int stat;
	char cstr[35];
	int size = sizeof(cstr);

	stat = (vowserv.bypass_enter_phase3 == true) ? 1 : 0;

	return snprintf(buf, size, "Enter Phase3 Setting is %s\n",
			(stat == 0x1) ? "Bypass" : "Allow");
}

static ssize_t VowDrv_SetBypassPhase3Flag(struct device *kobj,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	if (enable == 0) {
		VOWDRV_DEBUG("Allow enter phase3\n");
		vowserv.bypass_enter_phase3 = false;
	} else {
		VOWDRV_DEBUG("Bypass enter phase3\n");
		vowserv.bypass_enter_phase3 = true;
	}
	return n;
}
DEVICE_ATTR(vow_SetBypassPhase3,
	    0644, /*S_IWUSR | S_IRUGO*/
	    VowDrv_GetBypassPhase3Flag,
	    VowDrv_SetBypassPhase3Flag);

static ssize_t VowDrv_GetEnterPhase3Counter(struct device *kobj,
					    struct device_attribute *attr,
					    char *buf)
{
	char cstr[35];
	int size = sizeof(cstr);

	return snprintf(buf, size, "Enter Phase3 Counter is %u\n",
			vowserv.enter_phase3_cnt);
}
DEVICE_ATTR(vow_GetEnterPhase3Counter,
	    0444, /*S_IRUGO*/
	    VowDrv_GetEnterPhase3Counter,
	    NULL);

static ssize_t VowDrv_GetSWIPLog(struct device *kobj,
				 struct device_attribute *attr,
				 char *buf)
{
	unsigned int stat;
	char cstr[20];
	int size = sizeof(cstr);

	stat = (vowserv.swip_log_enable == true) ? 1 : 0;
	return snprintf(buf, size, "SWIP LOG is %s\n",
			(stat == true) ? "YES" : "NO");
}

static ssize_t VowDrv_SetSWIPLog(struct device *kobj,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	VowDrv_SetFlag(VOW_FLAG_SWIP_LOG_PRINT, enable);
	return n;
}
DEVICE_ATTR(vow_SetLibLog,
	    0644, /*S_IWUSR | S_IRUGO*/
	    VowDrv_GetSWIPLog,
	    VowDrv_SetSWIPLog);

static ssize_t VowDrv_SetEnableHW(struct device *kobj,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t n)
{
	unsigned int enable = 0;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	VowDrv_EnableHW(enable);
	VowDrv_ChangeStatus();
	return n;
}
DEVICE_ATTR(vow_SetEnableHW,
	    0200, /*S_IWUSR*/
	    NULL,
	    VowDrv_SetEnableHW);

static int VowDrv_SetVowEINTStatus(int status)
{
	int ret = 0;
	int wakeup_event = 0;

	if ((status < NUM_OF_VOW_EINT_STATUS)
	 && (status >= VOW_EINT_DISABLE)) {
		spin_lock(&vowdrv_lock);
		if ((vowserv.eint_status != VOW_EINT_PASS)
		 && (status == VOW_EINT_PASS))
			wakeup_event = 1;
		vowserv.eint_status = status;
		spin_unlock(&vowdrv_lock);
	} else {
		VOWDRV_DEBUG("%s() error input:%x\n",
			     __func__, status);
		ret = -1;
	}
	return ret;
}

static int VowDrv_QueryVowEINTStatus(void)
{
	int ret = 0;

	spin_lock(&vowdrv_lock);
	ret = vowserv.eint_status;
	spin_unlock(&vowdrv_lock);
	VOWDRV_DEBUG("%s():%d\n", __func__, ret);
	return ret;
}

static int VowDrv_open(struct inode *inode, struct file *fp)
{
	VOWDRV_DEBUG("%s() do nothing inode:%p, file:%p\n",
		    __func__, inode, fp);
	return 0;
}

static int VowDrv_release(struct inode *inode, struct file *fp)
{
	VOWDRV_DEBUG("%s() inode:%p, file:%p\n", __func__, inode, fp);

	if (!(fp->f_mode & FMODE_WRITE || fp->f_mode & FMODE_READ))
		return -ENODEV;
	return 0;
}

static long VowDrv_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int timeout = 0;

	timeout = 0;
	while (vowserv.vow_recovering) {
		msleep(VOW_WAITCHECK_INTERVAL_MS);
		timeout++;
		VOW_ASSERT(timeout != VOW_RECOVERY_WAIT);
	}

	VOWDRV_DEBUG("%s(), cmd = %u, arg = %lu\n", __func__, cmd, arg);
	switch ((unsigned int)cmd) {
	case VOW_SET_CONTROL:
		switch (arg) {
		case VOWControlCmd_Init:
			VOWDRV_DEBUG("VOW_SET_CONTROL Init");
			vow_service_Init();
			break;
		case VOWControlCmd_Reset:
			VOWDRV_DEBUG("VOW_SET_CONTROL Reset");
			vow_service_reset();
			break;
		case VOWControlCmd_EnableDebug:
			VOWDRV_DEBUG("VOW_SET_CONTROL EnableDebug");
			vowserv.voicedata_idx = 0;
			vowserv.recording_flag = true;
			vowserv.firstRead = true;
			/*VowDrv_SetFlag(VOW_FLAG_DEBUG, true);*/
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
			vow_register_feature(VOW_DUMP_FEATURE_ID);
#else
			VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
			if (vowserv.suspend_lock == 0) {
				vowserv.suspend_lock = 1;
				/* Let AP will not suspend */
				__pm_stay_awake(vow_suspend_lock);
				VOWDRV_DEBUG("==VOW DEBUG MODE START==\n");
			}
			break;
		case VOWControlCmd_DisableDebug:
			VOWDRV_DEBUG("VOW_SET_CONTROL DisableDebug");
			VowDrv_SetFlag(VOW_FLAG_DEBUG, false);
			vowserv.recording_flag = false;
			/* force stop vow_service_ReadVoiceData() 20180906 */
			vow_service_getVoiceData();
			if (vowserv.suspend_lock == 1) {
				vowserv.suspend_lock = 0;
				/* Let AP will suspend */
				__pm_relax(vow_suspend_lock);
				/* lock 1sec for screen on */
				VOWDRV_DEBUG("==VOW DEBUG MODE STOP==\n");
				__pm_wakeup_event(vow_suspend_lock,
						  jiffies_to_msecs(HZ));
			}
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
			vow_deregister_feature(VOW_DUMP_FEATURE_ID);
#else
			VOWDRV_DEBUG("%s(), vow: SCP no support\n\r", __func__);
#endif
			break;
		case VOWControlCmd_EnableSeamlessRecord:
			VOWDRV_DEBUG("VOW_SET_CONTROL EnableSeamlessRecord");
			VowDrv_SetFlag(VOW_FLAG_SEAMLESS, true);
			break;
		case VOWControlCmd_EnableDump:
			vow_pcm_dump_set(true);
			break;
		case VOWControlCmd_DisableDump:
			vow_pcm_dump_set(false);
			break;
		default:
			VOWDRV_DEBUG("VOW_SET_CONTROL no such command = %lu",
				     arg);
			break;
		}
		break;
	case VOW_READ_VOICE_DATA:
		if (!vow_service_SetApAddr(arg))
			ret = -EFAULT;
		if ((vowserv.recording_flag == true)
			&& (vowserv.firstRead == true)) {
			vowserv.firstRead = false;
			VowDrv_SetFlag(VOW_FLAG_DEBUG, true);
		}
		vow_service_ReadVoiceData();
		break;
	case VOW_SET_SPEAKER_MODEL:
		VOWDRV_DEBUG("VOW_SET_SPEAKER_MODEL(%lu)", arg);
		if (!vow_service_SetSpeakerModel(arg))
			ret = -EFAULT;
		break;
	case VOW_CLR_SPEAKER_MODEL:
		VOWDRV_DEBUG("VOW_CLR_SPEAKER_MODEL(%lu)", arg);
		if (!vow_service_ReleaseSpeakerModel((int)arg))
			ret = -EFAULT;
		break;
	case VOW_SET_APREG_INFO:
		VOWDRV_DEBUG("VOW_SET_APREG_INFO(%lu)", arg);
		if (!vow_service_SetVBufAddr(arg))
			ret = -EFAULT;
		break;
	case VOW_BARGEIN_ON:
		VOWDRV_DEBUG("VOW_BARGEIN_ON, irq: %d", (unsigned int)arg);
		if (!VowDrv_SetBargeIn(1, (unsigned int)arg))
			ret = -EFAULT;
		break;
	case VOW_BARGEIN_OFF:
		VOWDRV_DEBUG("VOW_BARGEIN_OFF, irq: %d", (unsigned int)arg);
		if (!VowDrv_SetBargeIn(0, (unsigned int)arg))
			ret = -EFAULT;
		break;
	case VOW_CHECK_STATUS:
		/* VOW disable already, then bypass second one */
		VowDrv_ChangeStatus();
		VOWDRV_DEBUG("VOW_CHECK_STATUS(%lu)", arg);
		break;
	case VOW_RECOG_ENABLE:
		pr_debug("+VOW_RECOG_ENABLE(%lu)+", arg);
		pr_debug("KERNEL_VOW_DRV_VER %s", KERNEL_VOW_DRV_VER);
		VowDrv_SetMtkifType((unsigned int)arg);
		VowDrv_EnableHW(1);
		VowDrv_ChangeStatus();
		vow_service_Enable();
		pr_debug("-VOW_RECOG_ENABLE(%lu)-", arg);
		break;
	case VOW_RECOG_DISABLE:
		pr_debug("+VOW_RECOG_DISABLE(%lu)+", arg);
		VowDrv_SetMtkifType((unsigned int)arg);
		VowDrv_EnableHW(0);
		VowDrv_ChangeStatus();
		vow_service_Disable();
		pr_debug("-VOW_RECOG_DISABLE(%lu)-", arg);
		break;
	case VOW_MODEL_START:
		vow_service_SetModelStatus(VOW_MODEL_STATUS_START, arg);
		break;
	case VOW_MODEL_STOP:
		vow_service_SetModelStatus(VOW_MODEL_STATUS_STOP, arg);
		break;
	case VOW_GET_GOOGLE_ENGINE_VER: {
		if (copy_to_user((void __user *)arg,
						 &vowserv.google_engine_version,
						 sizeof(vowserv.google_engine_version))) {
			VOWDRV_DEBUG("%s(), copy_to_user fail in VOW_GET_GOOGLE_ENGINE_VER",
						__func__);
		}
	}
		break;
	case VOW_GET_GOOGLE_ARCH:
	case VOW_GET_ALEXA_ENGINE_VER: {
		struct vow_engine_version_t engine_ver_temp;
		unsigned int length = VOW_ENGINE_INFO_LENGTH_BYTE;

		if (copy_from_user((void *)&engine_ver_temp,
				 (const void __user *)arg,
				 sizeof(struct vow_engine_version_t))) {
			VOWDRV_DEBUG("%s(), copy_from_user fail", __func__);
		}
		if ((unsigned int)cmd == VOW_GET_ALEXA_ENGINE_VER) {
			pr_debug("VOW_GET_ALEXA_ENGINE_VER = %s, %lu, %lu, %d",
				 vowserv.alexa_engine_version,
				 engine_ver_temp.data_addr,
				 engine_ver_temp.return_size_addr,
				 length);
			if (copy_to_user((void __user *)engine_ver_temp.data_addr,
					 vowserv.alexa_engine_version,
					 length)) {
				VOWDRV_DEBUG("%s(), copy_to_user fail", __func__);
			}
		} else if ((unsigned int)cmd == VOW_GET_GOOGLE_ARCH) {
			pr_debug("VOW_GET_GOOGLE_ARCH = %s, %lu, %lu, %d",
					 vowserv.google_engine_arch,
					 engine_ver_temp.data_addr,
					 engine_ver_temp.return_size_addr,
					 length);
			if (copy_to_user((void __user *)engine_ver_temp.data_addr,
					 vowserv.google_engine_arch,
					 length)) {
				VOWDRV_DEBUG("%s(), copy_to_user fail", __func__);
			}
		}
		if (copy_to_user((void __user *)engine_ver_temp.return_size_addr,
				 &length,
				 sizeof(unsigned int))) {
			VOWDRV_DEBUG("%s(), copy_to_user fail", __func__);
		}
	}
		break;
	default:
		VOWDRV_DEBUG("vow WrongParameter(%lu)", arg);
		break;
	}
	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long VowDrv_compat_ioctl(struct file *fp,
				unsigned int cmd,
				unsigned long arg)
{
	long ret = 0;

	/* VOWDRV_DEBUG("++VowDrv_compat_ioctl cmd = %u, arg = %lu\n" */
	/*		, cmd, arg); */
	if (!fp->f_op || !fp->f_op->unlocked_ioctl) {
		(void)ret;
		return -ENOTTY;
	}
	switch (cmd) {
	case VOW_CLR_SPEAKER_MODEL:
	case VOW_SET_CONTROL:
	case VOW_CHECK_STATUS:
	case VOW_RECOG_ENABLE:
	case VOW_RECOG_DISABLE:
	case VOW_BARGEIN_ON:
	case VOW_BARGEIN_OFF:
		ret = fp->f_op->unlocked_ioctl(fp, cmd, arg);
		break;
	case VOW_GET_GOOGLE_ENGINE_VER:
		ret = fp->f_op->unlocked_ioctl(fp, cmd, arg);
		break;
	case VOW_MODEL_START:
	case VOW_MODEL_STOP: {
		struct vow_model_start_kernel_t __user *data32;
		struct vow_model_start_t __user *data;
		int err;
		compat_size_t l;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));

		err  = get_user(l, &data32->handle);
		err |= put_user(l, &data->handle);
		err |= get_user(l, &data32->confidence_level);
		err |= put_user(l, &data->confidence_level);

		ret = fp->f_op->unlocked_ioctl(fp, cmd, (unsigned long)data);
	}
		break;
	case VOW_READ_VOICE_DATA:
	case VOW_SET_SPEAKER_MODEL:
	case VOW_SET_APREG_INFO: {
		struct vow_model_info_kernel_t __user *data32;
		struct vow_model_info_t __user *data;
		int err;
		compat_size_t l;
		compat_uptr_t p;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));

		err  = get_user(l, &data32->id);
		err |= put_user(l, &data->id);
		err |= get_user(l, &data32->keyword);
		err |= put_user(l, &data->keyword);
		err |= get_user(l, &data32->addr);
		err |= put_user(l, &data->addr);
		err |= get_user(l, &data32->size);
		err |= put_user(l, &data->size);
		err |= get_user(l, &data32->return_size_addr);
		err |= put_user(l, &data->return_size_addr);
		err |= get_user(l, &data32->uuid);
		err |= put_user(l, &data->uuid);
		err |= get_user(p, (compat_uptr_t *)&data32->data);
		err |= put_user(p, (compat_uptr_t *)&data->data);

		ret = fp->f_op->unlocked_ioctl(fp, cmd, (unsigned long)data);
	}
		break;
	case VOW_GET_GOOGLE_ARCH:
	case VOW_GET_ALEXA_ENGINE_VER: {
		struct vow_engine_version_kernel_t __user *data32;
		struct vow_engine_version_t __user *data;
		int err;
		compat_size_t l;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));

		err  = get_user(l, &data32->return_size_addr);
		err |= put_user(l, &data->return_size_addr);
		err |= get_user(l, &data32->data_addr);
		err |= put_user(l, &data->data_addr);

		ret = fp->f_op->unlocked_ioctl(fp, cmd, (unsigned long)data);
	}
		break;
	default:
		break;
	}
	/* VOWDRV_DEBUG("--VowDrv_compat_ioctl\n"); */
	return ret;
}
#endif

static ssize_t VowDrv_write(struct file *fp,
			    const char __user *data,
			    size_t count,
			    loff_t *offset)
{
	/* VOWDRV_DEBUG("+VowDrv_write = %p count = %d\n",fp ,count); */
	return 0;
}

static ssize_t VowDrv_read(struct file *fp,
			   char __user *data,
			   size_t count,
			   loff_t *offset)
{
	unsigned int read_count = 0;
	unsigned int time_diff_scp_ipi = 0;
	unsigned int time_diff_ipi_read = 0;
	unsigned long long vow_read_cycle = 0;
	int ret = 0;

	VOWDRV_DEBUG("+%s()+\n", __func__);
	VowDrv_SetVowEINTStatus(VOW_EINT_RETRY);

	if (VowDrv_Wait_Queue_flag == 0)
		ret = wait_event_interruptible(VowDrv_Wait_Queue,
					       VowDrv_Wait_Queue_flag);
	if (VowDrv_Wait_Queue_flag == 1) {
		VowDrv_Wait_Queue_flag = 0;
		if (VowDrv_GetHWStatus() == VOW_PWR_OFF) {
			VOWDRV_DEBUG("vow Enter_phase3_cnt = %d\n",
				      vowserv.enter_phase3_cnt);
			vowserv.scp_command_flag = false;
			VowDrv_SetVowEINTStatus(VOW_EINT_DISABLE);
		} else {
			if (vowserv.scp_command_flag) {
				VowDrv_SetVowEINTStatus(VOW_EINT_PASS);
				vow_read_cycle = get_cycles();
				time_diff_scp_ipi =
				    (unsigned int)CYCLE_TO_NS *
				    (unsigned int)(vowserv.ap_received_ipi_cycle
				    - vowserv.scp_recognize_ok_cycle);
				time_diff_ipi_read =
				    (unsigned int)CYCLE_TO_NS *
				    (unsigned int)(vow_read_cycle
				    - vowserv.ap_received_ipi_cycle);
				VOWDRV_DEBUG("vow Wakeup by SCP\n");
				VOWDRV_DEBUG("SCP->IPI:%d(ns),IPI->AP:%d(ns)\n",
					     time_diff_scp_ipi,
					     time_diff_ipi_read);
				if (vowserv.suspend_lock == 0) {
					/* lock 1sec for screen on */
					__pm_wakeup_event(vow_suspend_lock,
							  jiffies_to_msecs(HZ));
				}
				vowserv.scp_command_flag = false;
			} else {
				VOWDRV_DEBUG("vow Wakeup by other(%d,%d)\n",
					     VowDrv_Wait_Queue_flag,
					     VowDrv_GetHWStatus());
			}
		}
	}
	vowserv.vow_eint_data_struct.id = vowserv.scp_command_id;
	vowserv.vow_eint_data_struct.eint_status = VowDrv_QueryVowEINTStatus();
	vowserv.vow_eint_data_struct.data[0] = (char)vowserv.confidence_level;
	//reset data
	vowserv.confidence_level = 0;
	read_count = copy_to_user((void __user *)data,
				  &vowserv.vow_eint_data_struct,
				  sizeof(struct vow_eint_data_struct_t));
	VOWDRV_DEBUG("+%s()-, recog id: %d, confidence_lv=%d\n",
		     __func__,
		     vowserv.scp_command_id,
		     vowserv.confidence_level);
	return read_count;
}

static int VowDrv_flush(struct file *flip, fl_owner_t id)
{
	VOWDRV_DEBUG("%s()\n", __func__);
	return 0;
}

static int VowDrv_fasync(int fd, struct file *flip, int mode)
{
	VOWDRV_DEBUG("%s()\n", __func__);
	/*return fasync_helper(fd, flip, mode, &VowDrv_fasync);*/
	return 0;
}

static int VowDrv_remap_mmap(struct file *flip, struct vm_area_struct *vma)
{
	VOWDRV_DEBUG("%s()\n", __func__);
	return -1;
}

int VowDrv_setup_smartdev_eint(struct platform_device *pdev)
{
	int ret;
	unsigned int ints[2] = {0, 0};

	/* gpio setting */
	vowserv.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(vowserv.pinctrl)) {
		ret = PTR_ERR(vowserv.pinctrl);
		VOWDRV_DEBUG("Cannot find Vow pinctrl!\n");
		return ret;
	}
	vowserv.pins_eint_on = pinctrl_lookup_state(vowserv.pinctrl,
						    "vow_smartdev_eint_on");
	if (IS_ERR(vowserv.pins_eint_on)) {
		ret = PTR_ERR(vowserv.pins_eint_on);
		VOWDRV_DEBUG("Cannot find vow pinctrl eint_on!\n");
		return ret;
	}

	vowserv.pins_eint_off = pinctrl_lookup_state(vowserv.pinctrl,
						     "vow_smartdev_eint_off");
	if (IS_ERR(vowserv.pins_eint_off)) {
		ret = PTR_ERR(vowserv.pins_eint_off);
		VOWDRV_DEBUG("Cannot find vow pinctrl eint_off!\n");
		return ret;
	}
	/* eint setting */
	vowserv.node = pdev->dev.of_node;
	if (vowserv.node) {
		ret = of_property_read_u32_array(vowserv.node,
					   "debounce",
					   ints,
					   ARRAY_SIZE(ints));
		if (ret != 0) {
			VOWDRV_DEBUG("%s(), no debounce node, ret=%d\n",
				      __func__, ret);
			return ret;
		}

		VOWDRV_DEBUG("VOW EINT ID: %x, %x\n", ints[0], ints[1]);
	} else {
		/* no node here */
		VOWDRV_DEBUG("%s(), there is no this node\n", __func__);
	}
	return 0;
}

/*****************************************************************************
 * SCP Recovery Register
 *****************************************************************************/
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
static int vow_scp_recover_event(struct notifier_block *this,
				 unsigned long event,
				 void *ptr)
{
	switch (event) {
	case SCP_EVENT_READY: {
		int I;

		msleep(500);
		vowserv.vow_recovering = true;
		vowserv.scp_recovering = false;
		VOWDRV_DEBUG("%s(), SCP_EVENT_READY\n", __func__);
		if (!vow_check_scp_status()) {
			VOWDRV_DEBUG("SCP is Off, don't recover VOW\n");
			return NOTIFY_DONE;
		}
		if (vowserv.scp_recovering) {
			vowserv.vow_recovering = false;
			VOWDRV_DEBUG("fail: vow recover1\n");
			break;
		}
		vow_service_Init();
		if (vowserv.scp_recovering) {
			vowserv.vow_recovering = false;
			VOWDRV_DEBUG("fail: vow recover2\n");
			break;
		}
		for (I = 0; I < MAX_VOW_SPEAKER_MODEL; I++) {
			if (!vow_service_SendSpeakerModel(I, VOW_SET_MODEL))
				VOWDRV_DEBUG("fail: SendSpeakerModel\n");
		}

		/* if vow is not enable, then return */
		if (VowDrv_GetHWStatus() != VOW_PWR_ON) {
			vowserv.vow_recovering = false;
			break;
		}
		if (vowserv.scp_recovering) {
			vowserv.vow_recovering = false;
			VOWDRV_DEBUG("fail: vow recover3\n");
			break;
		}
		/* pcm dump recover */
		VOWDRV_DEBUG("recording_flag = %d, dump_pcm_flag = %d\n",
				vowserv.recording_flag, vowserv.dump_pcm_flag);
		if (vowserv.recording_flag == true)
			VowDrv_SetFlag(VOW_FLAG_DEBUG, true);

		if (vowserv.dump_pcm_flag == true)
			vow_pcm_dump_notify(true);

		if (vowserv.scp_recovering) {
			vowserv.vow_recovering = false;
			VOWDRV_DEBUG("fail: vow recover4\n");
			break;
		}

		if (!VowDrv_SetMtkifType(vowserv.mtkif_type))
			VOWDRV_DEBUG("fail: vow_SetMtkifType\n");

		if (!vow_service_Enable())
			VOWDRV_DEBUG("fail: vow_service_Enable\n");

		for (I = 0; I < MAX_VOW_SPEAKER_MODEL; I++) {
			if (vowserv.vow_speaker_model[I].enabled) {
				vow_service_SendModelStatus(
					I, VOW_MODEL_STATUS_START);
				VOWDRV_DEBUG("send Model start, slot%d\n", I);
			}
		}
		vowserv.vow_recovering = false;
		break;
	}
	case SCP_EVENT_STOP:
		vowserv.scp_recovering = true;
		VOWDRV_DEBUG("%s(), SCP_EVENT_STOP\n", __func__);
		/* Check if VOW is running phase2.5, then stop this */
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block vow_scp_recover_notifier = {
	.notifier_call = vow_scp_recover_event,
};
#endif  /* #if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT) */

/*****************************************************************************
 * VOW platform driver Registration
 *****************************************************************************/
static int VowDrv_probe(struct platform_device *dev)
{
	VOWDRV_DEBUG("%s()\n", __func__);
	vow_suspend_lock = wakeup_source_register(NULL, "vow wakelock");
	if (!vow_suspend_lock)
		pr_warn("wakeup source init failed.\n");
	VowDrv_setup_smartdev_eint(dev);
	return 0;
}

static int VowDrv_remove(struct platform_device *dev)
{
	VOWDRV_DEBUG("%s()\n", __func__);
	wakeup_source_unregister(vow_suspend_lock);
	return 0;
}

static void VowDrv_shutdown(struct platform_device *dev)
{
	VOWDRV_DEBUG("%s()\n", __func__);
}

static int VowDrv_suspend(struct platform_device *dev, pm_message_t state)
{
	/* only one suspend mode */
	VOWDRV_DEBUG("%s()\n", __func__);
	return 0;
}

static int VowDrv_resume(struct platform_device *dev) /* wake up */
{
	VOWDRV_DEBUG("%s()\n", __func__);
	return 0;
}

static const struct file_operations VOW_fops = {
	.owner   = THIS_MODULE,
	.open    = VowDrv_open,
	.release = VowDrv_release,
	.unlocked_ioctl   = VowDrv_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = VowDrv_compat_ioctl,
#endif
	.write   = VowDrv_write,
	.read    = VowDrv_read,
	.flush   = VowDrv_flush,
	.fasync  = VowDrv_fasync,
	.mmap    = VowDrv_remap_mmap
};

static struct miscdevice VowDrv_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = VOW_DEVNAME,
	.fops = &VOW_fops,
};

const struct dev_pm_ops VowDrv_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
	.freeze = NULL,
	.thaw = NULL,
	.poweroff = NULL,
	.restore = NULL,
	.restore_noirq = NULL,
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id vow_of_match[] = {
	{.compatible = "mediatek,vow"},
	{},
};
#endif

static struct platform_driver VowDrv_driver = {
	.probe    = VowDrv_probe,
	.remove   = VowDrv_remove,
	.shutdown = VowDrv_shutdown,
	.suspend  = VowDrv_suspend,
	.resume   = VowDrv_resume,
	.driver   = {
#if IS_ENABLED(CONFIG_PM)
	.pm       = &VowDrv_pm_ops,
#endif
	.name     = "VOW_driver_device",
#if IS_ENABLED(CONFIG_OF)
	.of_match_table = vow_of_match,
#endif
	},
};

static int __init VowDrv_mod_init(void)
{
	int ret = 0;

	VOWDRV_DEBUG("+%s()\n", __func__);

	/* Register platform DRIVER */
	ret = platform_driver_register(&VowDrv_driver);
	if (ret != 0) {
		VOWDRV_DEBUG("VowDrv Fail:%d - Register DRIVER\n", ret);
		return ret;
	}

	/* register MISC device */
	ret = misc_register(&VowDrv_misc_device);
	if (ret != 0) {
		VOWDRV_DEBUG("VowDrv_probe misc_register Fail:%d\n", ret);
		return ret;
	}

	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SetPhase1);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SetPhase2);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SetBypassPhase3);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_GetEnterPhase3Counter);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SetLibLog);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SetEnableHW);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SetBargeIn);
	if (unlikely(ret != 0))
		return ret;
	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_DualMicLch);
	if (unlikely(ret != 0))
		return ret;

	ret = device_create_file(VowDrv_misc_device.this_device,
				 &dev_attr_vow_SplitDumpFile);
	if (unlikely(ret != 0))
		return ret;

	VOWDRV_DEBUG("vow_service_Init");
	vow_service_Init();
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCP_SUPPORT)
	scp_A_register_notify(&vow_scp_recover_notifier);
#endif
	VOWDRV_DEBUG("-%s(): Init Audio WakeLock\n", __func__);

	return 0;
}

static void __exit VowDrv_mod_exit(void)
{
	VOWDRV_DEBUG("+%s()\n", __func__);
	vow_pcm_dump_deinit();
	VOWDRV_DEBUG("-%s()\n", __func__);
}
module_init(VowDrv_mod_init);
module_exit(VowDrv_mod_exit);


/*****************************************************************************
 * License
 *****************************************************************************/
MODULE_AUTHOR("Michael Hsiao <michael.hsiao@mediatek.com>");
MODULE_DESCRIPTION("MediaTek VoW Driver");
MODULE_LICENSE("GPL v2");

