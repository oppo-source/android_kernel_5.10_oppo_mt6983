// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/list.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>
#include <linux/dma-mapping.h>
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/aee.h>
#endif
#include <linux/clk.h>
#include <linux/atomic.h>
#include <linux/tcp.h>
#include <linux/udp.h>

#ifndef CCCI_KMODULE_ENABLE
#include "ccci_core.h"
#include "modem_sys.h"
#include "ccci_bm.h"
#include "ccci_platform.h"
#include "ccci_hif_dpmaif_v3.h"
#include "md_sys1_platform.h"
#include "dpmaif_drv_v3.h"
#include "modem_reg_base.h"
#include "ccci_fsm.h"
#include "ccci_port.h"
#else
#include "ccci_hif_dpmaif_v3.h"
#include "dpmaif_drv_v3.h"
#endif

#include "dpmaif_arch_v3.h"

#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/skbuff.h>

#include "ccci_hif_dpmaif_comm.h"
#include "modem_secure_base.h"
#include "dpmaif_bat_v3.h"
#include "net_speed.h"
#include "net_pool.h"
#include "md_spd_dvfs_method.h"
#include "md_spd_dvfs_fn.h"

#ifndef CCCI_KMODULE_ENABLE
#if defined(CCCI_SKB_TRACE)
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ccci

#include <linux/tracepoint.h>
#endif
#endif

#define UIDMASK 0x80000000
#define TAG "dpmaif"

struct hif_dpmaif_ctrl *dpmaif_ctrl_v3;
#define dpmaif_ctrl dpmaif_ctrl_v3


static struct dpmaif_clk_node g_clk_tbs[] = {
	{ NULL, "infra-dpmaif-clk"},
	{ NULL, "infra-dpmaif-blk-clk"},
	{ NULL, "infra-dpmaif-rg-mmw-clk"},
	{ NULL, NULL},
};


#ifdef CCCI_KMODULE_ENABLE
/*
 * for debug log:
 * 0 to disable; 1 for print to ram; 2 for print to uart
 * other value to desiable all log
 */
#ifndef CCCI_LOG_LEVEL /* for platform override */
#define CCCI_LOG_LEVEL CCCI_LOG_CRITICAL_UART
#endif

static inline struct device *ccci_md_get_dev_by_id(int md_id)
{
	return &dpmaif_ctrl->plat_dev->dev;
}
#endif

/* =======================================================
 *
 * Descriptions: Debug part
 *
 * ========================================================
 */
#ifndef CCCI_KMODULE_ENABLE
#if defined(CCCI_SKB_TRACE)
TRACE_EVENT(ccci_skb_rx,
	TP_PROTO(unsigned long long *dl_delay),
	TP_ARGS(dl_delay),
	TP_STRUCT__entry(
		__array(unsigned long long, dl_delay, 8)
	),

	TP_fast_assign(
		memcpy(__entry->dl_delay, dl_delay,
			8*sizeof(unsigned long long));
	),

	TP_printk(
	"	%llu	%llu	%llu	%llu	%llu	%llu	%llu	%llu",
		__entry->dl_delay[0], __entry->dl_delay[1],
		__entry->dl_delay[2], __entry->dl_delay[3],
		__entry->dl_delay[4], __entry->dl_delay[5],
		__entry->dl_delay[6], __entry->dl_delay[7])
);
#endif
#endif

static inline void dpmaif_set_cpu_mask(struct cpumask *cpu_mask,
		u32 cpus, int cpu_nr)
{
	int i;

	cpumask_clear(cpu_mask);

	for (i = 0; i < cpu_nr; i++) {
		if (cpus & (1 << i))
			cpumask_set_cpu(i, cpu_mask);
	}
}

static int dpmaif_send_skb_to_net(struct dpmaif_rx_queue *rxq,
	unsigned int skb_idx);

static void dpmaif_rxq_lro_info_init(struct dpmaif_rx_queue *rxq)
{
	 memset(&rxq->lro_info, 0, (sizeof(struct dpmaif_rx_lro_info)));
}

static inline int my_skb_gro_receive(struct sk_buff *p, struct sk_buff *skb)
{
	struct skb_shared_info *pinfo, *skbinfo = skb_shinfo(skb);
	unsigned int offset = skb_gro_offset(skb);
	unsigned int headlen = skb_headlen(skb);
	unsigned int len = skb_gro_len(skb);
	unsigned int delta_truesize;
	struct sk_buff *lp;

	if (unlikely(p->len + len >= 65536 || NAPI_GRO_CB(skb)->flush))
		return -E2BIG;

	lp = NAPI_GRO_CB(p)->last;
	pinfo = skb_shinfo(lp);

	if (headlen <= offset) {
		skb_frag_t *frag;
		skb_frag_t *frag2;
		int i = skbinfo->nr_frags;
		int nr_frags = pinfo->nr_frags + i;

		if (nr_frags > MAX_SKB_FRAGS)
			goto merge;

		offset -= headlen;
		pinfo->nr_frags = nr_frags;
		skbinfo->nr_frags = 0;

		frag = pinfo->frags + nr_frags;
		frag2 = skbinfo->frags + i;
		do {
			*--frag = *--frag2;
		} while (--i);

		skb_frag_off_add(frag, offset);
		skb_frag_size_sub(frag, offset);

		/* all fragments truesize : remove (head size + sk_buff) */
		delta_truesize = skb->truesize -
				 SKB_TRUESIZE(skb_end_offset(skb));

		skb->truesize -= skb->data_len;
		skb->len -= skb->data_len;
		skb->data_len = 0;

		NAPI_GRO_CB(skb)->free = NAPI_GRO_FREE;
		goto done;

	} else if (skb->head_frag) {
		int nr_frags = pinfo->nr_frags;
		skb_frag_t *frag = pinfo->frags + nr_frags;
		struct page *page = virt_to_head_page(skb->head);
		unsigned int first_size = headlen - offset;
		unsigned int first_offset;

		if (nr_frags + 1 + skbinfo->nr_frags > MAX_SKB_FRAGS)
			goto merge;

		first_offset = skb->data -
			       (unsigned char *)page_address(page) +
			       offset;

		pinfo->nr_frags = nr_frags + 1 + skbinfo->nr_frags;

		__skb_frag_set_page(frag, page);
		skb_frag_off_set(frag, first_offset);
		skb_frag_size_set(frag, first_size);

		memcpy(frag + 1, skbinfo->frags, sizeof(*frag) * skbinfo->nr_frags);
		/* We dont need to clear skbinfo->nr_frags here */

		delta_truesize = skb->truesize - SKB_DATA_ALIGN(sizeof(struct sk_buff));
		NAPI_GRO_CB(skb)->free = NAPI_GRO_FREE_STOLEN_HEAD;
		goto done;
	}

merge:
	delta_truesize = skb->truesize;
	if (offset > headlen) {
		unsigned int eat = offset - headlen;

		skb_frag_off_add(&skbinfo->frags[0], eat);
		skb_frag_size_sub(&skbinfo->frags[0], eat);
		skb->data_len -= eat;
		skb->len -= eat;
		offset = headlen;
	}

	__skb_pull(skb, offset);

	if (NAPI_GRO_CB(p)->last == p)
		skb_shinfo(p)->frag_list = skb;
	else
		NAPI_GRO_CB(p)->last->next = skb;
	NAPI_GRO_CB(p)->last = skb;
	__skb_header_release(skb);
	lp = p;

done:
	NAPI_GRO_CB(p)->count++;
	p->data_len += len;
	p->truesize += delta_truesize;
	p->len += len;
	if (lp != p) {
		lp->data_len += len;
		lp->truesize += delta_truesize;
		lp->len += len;
	}
	NAPI_GRO_CB(skb)->same_flow = 1;
	return 0;
}

static inline void dpmaif_rxq_lro_handle_bid(
		struct dpmaif_rx_queue *rxq, int count, int free_skb)
{
	int i;
	struct dpmaif_bat_skb_t *bat_skb;
	struct dpmaif_rx_lro_info *lro_info = &rxq->lro_info;

	for (i = 0; i < count; i++) {
		if (free_skb)
			dev_kfree_skb_any(lro_info->skb_tbl[i]);

		bat_skb = ((struct dpmaif_bat_skb_t *)
				dpmaif_ctrl->bat_req->bat_skb_ptr +
				lro_info->bid_tbl[i]);
		bat_skb->skb = NULL;
	}

	lro_info->count = 0;
}

static inline void dpmaif_rxq_push_all_skb(struct dpmaif_rx_queue *rxq)
{
	struct lhif_header *lhif_h = NULL;
	struct dpmaif_rx_lro_info *lro_info = &rxq->lro_info;
	struct sk_buff *skb;
	int i;

	for (i = 0; i < lro_info->count; i++) {
		skb = lro_info->skb_tbl[i];

		lhif_h = (struct lhif_header *)(skb_push(skb,
				sizeof(struct lhif_header)));
		lhif_h->netif = rxq->cur_chn_idx;

		ccci_dl_enqueue(rxq->index, skb);
	}

	dpmaif_rxq_lro_handle_bid(rxq, lro_info->count, 0);
}

static inline void dpmaif_rxq_lro_start(struct dpmaif_rx_lro_info *lro_info)
{
	lro_info->count = 0;
}

static inline void dpmaif_rxq_lro_join_skb(
		struct dpmaif_rx_queue *rxq,
		struct sk_buff *skb,
#ifndef GET_HEADER_OFFSET_FROM_PIT
		unsigned int bid)
#else
		unsigned int bid, unsigned int header_offset)
#endif
{
	struct dpmaif_rx_lro_info *lro_info = &rxq->lro_info;

	if (lro_info->count >= DPMAIF_MAX_LRO) {
		CCCI_ERROR_LOG(0, TAG,
			"[%s] lro skb count is too much.\n",
			__func__);

		dpmaif_rxq_push_all_skb(rxq);
	}

	lro_info->bid_tbl[lro_info->count] = bid;
	lro_info->skb_tbl[lro_info->count] = skb;
#ifdef GET_HEADER_OFFSET_FROM_PIT
	lro_info->hof_tbl[lro_info->count] = header_offset;
#endif
	lro_info->count++;
}

static inline int dpmaif_get_skb_header_len(
		struct sk_buff *skb, unsigned int bid)
{
	struct iphdr *iph = (struct iphdr *)skb->data;
	struct ipv6hdr *ip6h = (struct ipv6hdr *)skb->data;
	struct tcphdr *tcph;
	int ip_len = 0;
	u16 ipid = 0;

	if (iph->version == 4) {
		if(iph->protocol != IPPROTO_TCP)
			return 0;

		ipid = ntohs(iph->id);

		ip_len = (iph->ihl << 2);
		tcph = (struct tcphdr *)((void *)iph + ip_len);

	} else if (iph->version == 6) {
		if (ip6h->nexthdr != IPPROTO_TCP)
			return 0;

		ip_len = 40;
		tcph = (struct tcphdr *)((void *)ip6h + 40);

	} else {
		CCCI_ERROR_LOG(0, TAG,
			"[%s] error: ip verion is invalid: %d; bid: %d\n",
			__func__, iph->version, bid);
		return 0;
	}

	return (ip_len + (tcph->doff << 2));
}

static inline void dpmaif_lro_update_gro_info(
		struct sk_buff *skb,
		unsigned int total_len,
		int gro_skb_num)
{
	struct iphdr *iph = (struct iphdr *)skb->data;
	struct ipv6hdr *ip6h = (struct ipv6hdr *)skb->data;
	unsigned int gso_type;

	if (iph->version == 4) {
		gso_type = SKB_GSO_TCPV4;
		iph->tot_len = htons(total_len);
		iph->check = 0;
		iph->check = ip_fast_csum((const void*)iph, iph->ihl);

	} else if (iph->version == 6) {
		gso_type = SKB_GSO_TCPV6;
		ip6h->payload_len = htons(total_len - 40);

	} else
		return;

	skb->ip_summed = CHECKSUM_UNNECESSARY;

	skb_shinfo(skb)->gso_type |= gso_type;
	skb_shinfo(skb)->gso_size = 0;
	skb_shinfo(skb)->gso_segs = gro_skb_num;
}

static inline void dpmaif_handle_wakeup(struct dpmaif_rx_queue *rxq,
		struct sk_buff *skb)
{
	struct iphdr *iph = (struct iphdr *)skb->data;
	struct ipv6hdr *ip6h = (struct ipv6hdr *)skb->data;
	struct tcphdr *tcph = NULL;
	struct udphdr *udph = NULL;
	int ip_offset = 0;
	u32 version  = 0;
	u32 protocol = 0;
	u32 src_port = 0;
	u32 dst_port = 0;
	u32 skb_len  = 0;

	if (!skb)
		goto err;

	skb_len = skb->len;
	version = iph->version;
	if (version == 4) {
		protocol = iph->protocol;
		ip_offset = (iph->ihl << 2);
	} else if (version == 6) {
		protocol = ip6h->nexthdr;
		ip_offset = 40;
	} else
		goto err;
	if (protocol == IPPROTO_TCP) {
		tcph = (struct tcphdr *)((void *)iph + ip_offset);
		src_port = ntohs(tcph->source);
		dst_port = ntohs(tcph->dest);
	} else if (protocol == IPPROTO_UDP) {
		udph = (struct udphdr *)((void *)iph + ip_offset);
		src_port = ntohs(udph->source);
		dst_port = ntohs(udph->dest);
	} else
		goto err;

err:
	CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
		"[%s] ver: %u; pro: %u; spt: %u; dpt: %u; len: %u\n",
		__func__, version, protocol,
		src_port, dst_port, skb_len);

}

static inline int dpmaif_rxq_lro_end(struct dpmaif_rx_queue *rxq)
{
	struct lhif_header *lhif_h = NULL;
	struct sk_buff *skb0, *skb1;
	int i, header_len, ret, start_idx, data_len;
	unsigned int total_len, lro_num;
	struct dpmaif_rx_lro_info *lro_info = &rxq->lro_info;

	if (rxq->pit_dp) {
		dpmaif_rxq_lro_handle_bid(rxq, lro_info->count, 1);
		return 0;
	}

	start_idx = 0;

lro_continue:
	lro_num = 0;
	total_len = 0;

	for (i = start_idx; i < lro_info->count; i++) {
		if (i == start_idx) {
			skb0 = lro_info->skb_tbl[i];
			total_len = skb0->len;
			lro_num++;

			if ((lro_info->count - i) == 1) {
				start_idx = lro_info->count;
				goto lro_end;
			}

#ifdef GET_HEADER_OFFSET_FROM_PIT
			if (lro_info->hof_tbl[i])
				header_len = (lro_info->hof_tbl[i] << 2);
			else {
#endif
			header_len = dpmaif_get_skb_header_len(skb0,
					lro_info->bid_tbl[i]);
			if (header_len == 0) {
				start_idx = i + 1;
				goto gro_too_much_skb;
			}
#ifdef GET_HEADER_OFFSET_FROM_PIT
			}
#endif
			NAPI_GRO_CB(skb0)->data_offset = header_len;
			NAPI_GRO_CB(skb0)->last = skb0;

			continue;
		}

		skb1 = lro_info->skb_tbl[i];
#ifdef GET_HEADER_OFFSET_FROM_PIT
		if (lro_info->hof_tbl[i])
			header_len = (lro_info->hof_tbl[i] << 2);
		else {
#endif
		header_len = dpmaif_get_skb_header_len(skb1,
				lro_info->bid_tbl[i]);
		if (header_len == 0) {
			start_idx = i;
			goto gro_too_much_skb;
		}
#ifdef GET_HEADER_OFFSET_FROM_PIT
		}
#endif
		data_len = skb1->len - header_len;

		NAPI_GRO_CB(skb1)->data_offset = header_len;

		ret = my_skb_gro_receive(skb0, skb1);
		if (ret) {
			NAPI_GRO_CB(skb1)->data_offset = 0;
			start_idx = i;
			goto gro_too_much_skb;
		}

		total_len += data_len;

		lro_num++;
	}
	start_idx = lro_info->count;

gro_too_much_skb:
	if (lro_num > 1)
		dpmaif_lro_update_gro_info(skb0, total_len, lro_num);

lro_end:
	if (atomic_cmpxchg(&dpmaif_ctrl->wakeup_src, 1, 0) == 1) {
		CCCI_NOTICE_LOG(dpmaif_ctrl->md_id, TAG,
			"DPMA_MD wakeup source:(%d/%d)\n",
			rxq->index, rxq->cur_chn_idx);
		dpmaif_handle_wakeup(rxq, skb0);
	}

	lhif_h = (struct lhif_header *)(skb_push(skb0,
			sizeof(struct lhif_header)));
	lhif_h->netif = rxq->cur_chn_idx;

	ccci_dl_enqueue(rxq->index, skb0);

	if (start_idx < lro_info->count)
		goto lro_continue;

	dpmaif_rxq_lro_handle_bid(rxq, lro_info->count, 0);
	return 0;
}

static void dpmaif_dump_rx_pit(struct hif_dpmaif_ctrl *hif_ctrl)
{
	int i;
	struct dpmaif_rx_queue *rxq = NULL;

	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		rxq = &hif_ctrl->rxq[i];

		CCCI_BUF_LOG_TAG(hif_ctrl->md_id, CCCI_DUMP_DPMAIF, TAG,
			"dpmaif: rxq%d, pit request base: 0x%p(%d*%d)\n",
			rxq->index, rxq->pit_base,
			(int)sizeof(struct dpmaifq_normal_pit),
			rxq->pit_size_cnt);

		CCCI_BUF_LOG_TAG(hif_ctrl->md_id, CCCI_DUMP_DPMAIF, TAG,
			"Current rxq%d pit pos: w/r/rel=%x, %x, %x\n", i,
			rxq->pit_wr_idx, rxq->pit_rd_idx,
			rxq->pit_rel_rd_idx);

		ccci_util_mem_dump(hif_ctrl->md_id,
			CCCI_DUMP_DPMAIF, rxq->pit_base,
			(rxq->pit_size_cnt *
			sizeof(struct dpmaifq_normal_pit)));
	}
}

static void dpmaif_dump_register(struct hif_dpmaif_ctrl *hif_ctrl, int buf_type)
{
	if (hif_ctrl->dpmaif_state == HIFDPMAIF_STATE_PWROFF
		|| hif_ctrl->dpmaif_state == HIFDPMAIF_STATE_MIN) {
		CCCI_MEM_LOG_TAG(hif_ctrl->md_id, TAG,
			"DPMAIF not power on, skip dump\n");
		return;
	}

	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF Tx pdn register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_ul_base + DPMAIF_PD_UL_ADD_DESC,
		DPMAIF_PD_UL_ADD_DESC_CH - DPMAIF_PD_UL_ADD_DESC + 4);
	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF Tx ao register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_ao_ul_base + DPMAIF_AO_UL_CHNL0_STA,
		DPMAIF_AO_UL_CHNL3_STA - DPMAIF_AO_UL_CHNL0_STA + 4);

	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF Rx pdn register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_dl_base + DPMAIF_PD_DL_BAT_INIT,
		DPMAIF_PD_DL_MISC_CON0 - DPMAIF_PD_DL_BAT_INIT + 4);
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_dl_base + DPMAIF_PD_DL_STA0,
		DPMAIF_PD_DL_DBG_STA14 - DPMAIF_PD_DL_STA0 + 4);
	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF dma_rd config register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_dl_base + 0x100, 0xC8);
	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF dma_wr config register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_dl_base + 0x200, 0x58 + 4);
	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF Rx ao register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_ao_dl_base + DPMAIF_AO_DL_PKTINFO_CONO,
		DPMAIF_AO_DL_FRGBAT_STA2 - DPMAIF_AO_DL_PKTINFO_CONO + 4);

	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF MISC pdn register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_misc_base + DPMAIF_PD_AP_UL_L2TISAR0,
		DPMAIF_PD_AP_CODA_VER - DPMAIF_PD_AP_UL_L2TISAR0 + 4);

	/* open sram clock for debug sram needs sram clock. */
	DPMA_WRITE_PD_MISC(DPMAIF_PD_AP_CG_EN, 0x36);
	CCCI_BUF_LOG_TAG(hif_ctrl->md_id, buf_type, TAG,
		"dump AP DPMAIF SRAM pdn register\n");
	ccci_util_mem_dump(hif_ctrl->md_id, buf_type,
		hif_ctrl->dpmaif_pd_sram_base + 0x00,
		0x184);

	dpmaif_dump_rx_pit(hif_ctrl);
}

#if 0
void dpmaif_dump_reg(void)
{
	if (!dpmaif_ctrl)
		return;
	dpmaif_dump_register(dpmaif_ctrl, CCCI_DUMP_REGISTER);
}
EXPORT_SYMBOL(dpmaif_dump_reg);
#endif

static void dpmaif_dump_rxq_history(struct hif_dpmaif_ctrl *hif_ctrl,
	unsigned int qno, int dump_multi)
{
	unsigned char md_id = hif_ctrl->md_id;

	if (qno > DPMAIF_RXQ_NUM) {
		CCCI_MEM_LOG_TAG(md_id, TAG, "invalid rxq%d\n", qno);
		return;
	}
	ccci_md_dump_log_history(md_id, &hif_ctrl->traffic_info,
			dump_multi,	-1, qno);
}

static void dpmaif_dump_txq_history(struct hif_dpmaif_ctrl *hif_ctrl,
	unsigned int qno, int dump_multi)
{
	unsigned char md_id = hif_ctrl->md_id;

	if (qno > DPMAIF_TXQ_NUM) {
		CCCI_MEM_LOG_TAG(md_id, TAG, "invalid txq%d\n", qno);
		return;
	}
	ccci_md_dump_log_history(md_id, &hif_ctrl->traffic_info,
			dump_multi, qno, -1);
}

static void dpmaif_dump_rxq_remain(struct hif_dpmaif_ctrl *hif_ctrl,
	unsigned int qno, int dump_multi)
{
	int i, rx_qno;
	unsigned char md_id = hif_ctrl->md_id;
	struct dpmaif_rx_queue *rxq = NULL;

	if (!dump_multi && (qno >= DPMAIF_RXQ_NUM)) {
		CCCI_MEM_LOG_TAG(md_id, TAG, "invalid rxq%d\n", qno);
		return;
	}

	if (dump_multi) {
		rx_qno = ((qno <= MAX_RXQ_NUM) ? qno : MAX_RXQ_NUM);
		i = 0;
	} else {
		i = qno;
		rx_qno = qno + 1;
	}
	for (; i < rx_qno; i++) {
		rxq = &hif_ctrl->rxq[i];
		/* rxq struct dump */
		CCCI_MEM_LOG_TAG(md_id, TAG, "dpmaif:dump rxq(%d): 0x%p\n",
			i, rxq);
#ifdef DPMAIF_DEBUG_LOG
		ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP, (void *)rxq,
			(int)sizeof(struct dpmaif_rx_queue));
#endif
		/* PIT mem dump */
		CCCI_MEM_LOG(md_id, TAG,
			"dpmaif:pit request base: 0x%p(%d*%d)\n",
			rxq->pit_base,
			(int)sizeof(struct dpmaifq_normal_pit),
			rxq->pit_size_cnt);
		CCCI_MEM_LOG(md_id, TAG,
			"Current rxq%d pit pos: w/r/rel=%x, %x, %x\n", i,
		       rxq->pit_wr_idx, rxq->pit_rd_idx, rxq->pit_rel_rd_idx);
#ifdef DPMAIF_DEBUG_LOG
		ccci_util_mem_dump(-1, CCCI_DUMP_MEM_DUMP, rxq->pit_base,
			(rxq->pit_size_cnt *
			sizeof(struct dpmaifq_normal_pit)));
#endif
		/* BAT mem dump */
		if (dpmaif_ctrl->bat_req) {
			CCCI_MEM_LOG(md_id, TAG,
				"dpmaif:bat request base: 0x%p(%d*%d)\n",
				dpmaif_ctrl->bat_req->bat_base,
				(int)sizeof(struct dpmaif_bat_t),
			dpmaif_ctrl->bat_req->bat_size_cnt);
			CCCI_MEM_LOG(md_id, TAG,
				"Current rxq%d bat pos: w/r/rel=%x, %x\n", i,
				dpmaif_ctrl->bat_req->bat_wr_idx,
				dpmaif_ctrl->bat_req->bat_rd_idx);
		}
#ifdef DPMAIF_DEBUG_LOG
		/* BAT SKB mem dump */
		if (dpmaif_ctrl->bat_req) {
			CCCI_MEM_LOG(md_id, TAG, "dpmaif:bat skb base: 0x%p(%d*%d)\n",
				dpmaif_ctrl->bat_req->bat_skb_ptr,
				(int)sizeof(struct dpmaif_bat_skb_t),
				dpmaif_ctrl->bat_req->bat_size_cnt);
			ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
				dpmaif_ctrl->bat_req->bat_skb_ptr,
				(dpmaif_ctrl->bat_req->skb_pkt_cnt *
				sizeof(struct dpmaif_bat_skb_t)));
		}
#ifdef HW_FRG_FEATURE_ENABLE
		/* BAT frg mem dump */
		if (dpmaif_ctrl->bat_frag) {
			CCCI_MEM_LOG(md_id, TAG,
				"dpmaif:bat_frag base: 0x%p(%d*%d)\n",
				dpmaif_ctrl->bat_frag->bat_base,
				(int)sizeof(struct dpmaif_bat_t),
				dpmaif_ctrl->bat_frag->bat_size_cnt);
			CCCI_MEM_LOG(md_id, TAG,
				"Current rxq%d bat_frag pos: w/r/rel=%x, %x\n", i,
				dpmaif_ctrl->bat_frag->bat_wr_idx,
				dpmaif_ctrl->bat_frag->bat_rd_idx);
			/* BAT fragment mem dump */
			CCCI_MEM_LOG(md_id, TAG, "dpmaif:bat_frag base: 0x%p(%d*%d)\n",
				dpmaif_ctrl->bat_frag->bat_skb_ptr,
				(int)sizeof(struct dpmaif_bat_page_t),
				dpmaif_ctrl->bat_frag->bat_size_cnt);
			ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
					dpmaif_ctrl->bat_frag->bat_skb_ptr,
				(dpmaif_ctrl->bat_frag->skb_pkt_cnt *
				sizeof(struct dpmaif_bat_page_t)));
		}
#endif
#endif
	}
}

static void dpmaif_dump_txq_remain(struct hif_dpmaif_ctrl *hif_ctrl,
	unsigned int qno, int dump_multi)
{
	struct dpmaif_tx_queue *txq = NULL;
	int i, tx_qno;
	unsigned char md_id = hif_ctrl->md_id;

	if (!dump_multi && (qno >= DPMAIF_TXQ_NUM)) {
		CCCI_MEM_LOG_TAG(md_id, TAG, "invalid txq%d\n", qno);
		return;
	}

	if (dump_multi) {
		tx_qno = ((qno <= MAX_TXQ_NUM) ? qno : MAX_TXQ_NUM);
		i = 0;
	} else {
		tx_qno = qno + 1;
		i = qno;
	}

	for (; i < tx_qno; i++) {
		txq = &hif_ctrl->txq[i];
		CCCI_MEM_LOG_TAG(md_id, TAG, "dpmaif:dump txq(%d): 0x%p\n",
			i, txq);
		ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP, (void *)txq,
			sizeof(struct dpmaif_tx_queue));
		CCCI_MEM_LOG(md_id, TAG, "dpmaif: drb(%d) base: 0x%p(%d*%d)\n",
			txq->index, txq->drb_base,
			(int)sizeof(struct dpmaif_drb_pd), txq->drb_size_cnt);
		CCCI_MEM_LOG(md_id, TAG,
			"Current txq%d pos: w/r/rel=%x, %x, %x\n", i,
		       txq->drb_wr_idx, txq->drb_rd_idx, txq->drb_rel_rd_idx);
		ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP, txq->drb_base,
			(txq->drb_size_cnt * sizeof(struct dpmaif_drb_pd)));

	}
}

static void dpmaif_dump_dl_queue_drop_cnt(void)
{
	int i, cnt;

	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		cnt = ccci_get_dl_queue_dp_cnt(i);
		if (cnt)
			CCCI_NORMAL_LOG(0, TAG,
				"rxq%d drop skb count: %u", i, cnt);
	}
}

/*actrually, length is dump flag's private argument*/
static int dpmaif_dump_status(unsigned char hif_id,
		enum MODEM_DUMP_FLAG flag, void *buff, int length)
{
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;

	CCCI_MEM_LOG_TAG(hif_ctrl->md_id, TAG,
		"%s: q_bitmap = %d\n", __func__, length);

	if (length == -1) {
		/* dpmaif_dump_txq_history(hif_ctrl, DPMAIF_TXQ_NUM, 1); */
		/* dpmaif_dump_txq_remain(hif_ctrl, DPMAIF_TXQ_NUM, 1); */
		dpmaif_dump_rxq_history(hif_ctrl, DPMAIF_RXQ_NUM, 1);
		dpmaif_dump_rxq_remain(hif_ctrl, DPMAIF_RXQ_NUM, 1);
	}

	if (flag & DUMP_FLAG_REG)
		dpmaif_dump_register(hif_ctrl, CCCI_DUMP_REGISTER);

	if (flag & DUMP_FLAG_IRQ_STATUS) {
		CCCI_NORMAL_LOG(hif_ctrl->md_id, TAG,
			"Dump AP DPMAIF IRQ status not support\n");
	}
	if (flag & DUMP_FLAG_TOGGLE_NET_SPD)
		return mtk_ccci_net_spd_cfg(1);

	dpmaif_dump_dl_queue_drop_cnt();

	return 0;
}

#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
static void dpmaif_clear_traffic_data(unsigned char hif_id)
{
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;

	memset(hif_ctrl->tx_traffic_monitor, 0,
		sizeof(hif_ctrl->tx_traffic_monitor));
	memset(hif_ctrl->rx_traffic_monitor, 0,
		sizeof(hif_ctrl->rx_traffic_monitor));
	memset(hif_ctrl->tx_pre_traffic_monitor, 0,
		sizeof(hif_ctrl->tx_pre_traffic_monitor));
	memset(hif_ctrl->tx_done_last_count, 0,
		sizeof(hif_ctrl->tx_done_last_count));
	memset(hif_ctrl->tx_done_last_start_time, 0,
		sizeof(hif_ctrl->tx_done_last_start_time));
#ifdef DPMAIF_DEBUG_LOG
	hif_ctrl->traffic_info.isr_time_bak = 0;
	hif_ctrl->traffic_info.isr_cnt = 0;
	hif_ctrl->traffic_info.rx_full_cnt = 0;

	hif_ctrl->traffic_info.rx_done_isr_cnt[0] = 0;
	hif_ctrl->traffic_info.rx_other_isr_cnt[0] = 0;
	hif_ctrl->traffic_info.rx_tasket_cnt = 0;

	hif_ctrl->traffic_info.tx_done_isr_cnt[0] = 0;
	hif_ctrl->traffic_info.tx_done_isr_cnt[1] = 0;
	hif_ctrl->traffic_info.tx_done_isr_cnt[2] = 0;
	hif_ctrl->traffic_info.tx_done_isr_cnt[3] = 0;
	hif_ctrl->traffic_info.tx_other_isr_cnt[0] = 0;
	hif_ctrl->traffic_info.tx_other_isr_cnt[1] = 0;
	hif_ctrl->traffic_info.tx_other_isr_cnt[2] = 0;
	hif_ctrl->traffic_info.tx_other_isr_cnt[3] = 0;
#endif

}
#endif

#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
static void dpmaif_traffic_monitor_func(struct timer_list *t)
{
	struct hif_dpmaif_ctrl *hif_ctrl =
			from_timer(hif_ctrl, t, traffic_monitor);

	struct ccci_hif_traffic *tinfo = &hif_ctrl->traffic_info;
	unsigned long q_rx_rem_nsec[DPMAIF_RXQ_NUM] = {0};
	unsigned long isr_rem_nsec;
	int i, q_state = 0;

	if (3 < DPMAIF_TXQ_NUM) {
		q_state = (hif_ctrl->txq[0].que_started << 0) |
			(hif_ctrl->txq[1].que_started << 1) |
			(hif_ctrl->txq[2].que_started << 2) |
			(hif_ctrl->txq[3].que_started << 3);
		CCCI_NORMAL_LOG(hif_ctrl->md_id, TAG,
			"net txq0-3(status=0x%x)[%d]:%d-%d-%d(0x%x), %d-%d-%d(0x%x), %d-%d-%d(0x%x), %d-%d-%d(0x%x)\n",
			q_state, hif_ctrl->txq[0].drb_size_cnt,
			atomic_read(&hif_ctrl->txq[0].tx_budget),
			hif_ctrl->tx_pre_traffic_monitor[0],
			hif_ctrl->tx_traffic_monitor[0],
			DPMA_READ_AO_UL(DPMAIF_ULQ_STA0_n(0)),
			atomic_read(&hif_ctrl->txq[1].tx_budget),
			hif_ctrl->tx_pre_traffic_monitor[1],
			hif_ctrl->tx_traffic_monitor[1],
			DPMA_READ_AO_UL(DPMAIF_ULQ_STA0_n(1)),
			atomic_read(&hif_ctrl->txq[2].tx_budget),
			hif_ctrl->tx_pre_traffic_monitor[2],
			hif_ctrl->tx_traffic_monitor[2],
			DPMA_READ_AO_UL(DPMAIF_ULQ_STA0_n(2)),
			atomic_read(&hif_ctrl->txq[3].tx_budget),
			hif_ctrl->tx_pre_traffic_monitor[3],
			hif_ctrl->tx_traffic_monitor[3],
			DPMA_READ_AO_UL(DPMAIF_ULQ_STA0_n(3)));
		CCCI_NORMAL_LOG(hif_ctrl->md_id, TAG,
			"Current txq pos: w/r/rel=(%d,%d,%d)(%d,%d,%d)(%d,%d,%d)(%d,%d,%d), tx_busy=%d,%d,%d,%d\n",
			hif_ctrl->txq[0].drb_wr_idx,
			hif_ctrl->txq[0].drb_rd_idx,
			hif_ctrl->txq[0].drb_rel_rd_idx,
			hif_ctrl->txq[1].drb_wr_idx,
			hif_ctrl->txq[1].drb_rd_idx,
			hif_ctrl->txq[1].drb_rel_rd_idx,
			hif_ctrl->txq[2].drb_wr_idx,
			hif_ctrl->txq[2].drb_rd_idx,
			hif_ctrl->txq[2].drb_rel_rd_idx,
			hif_ctrl->txq[3].drb_wr_idx,
			hif_ctrl->txq[3].drb_rd_idx,
			hif_ctrl->txq[3].drb_rel_rd_idx,
			hif_ctrl->txq[0].busy_count,
			hif_ctrl->txq[1].busy_count,
			hif_ctrl->txq[2].busy_count,
			hif_ctrl->txq[3].busy_count);
	} else
		for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
			CCCI_NORMAL_LOG(hif_ctrl->md_id, TAG,
				"Current txq%d (%d) pos: w/r/rel=%d, %d, %d, tx_busy=%d\n",
				i, hif_ctrl->txq[i].que_started,
				hif_ctrl->txq[i].drb_wr_idx,
				hif_ctrl->txq[i].drb_rd_idx,
				hif_ctrl->txq[i].drb_rel_rd_idx,
				hif_ctrl->txq[i].busy_count);
			if (hif_ctrl->txq[i].busy_count != 0)
				hif_ctrl->txq[i].busy_count = 0;
		}

	isr_rem_nsec = (tinfo->latest_isr_time == 0 ?
		0 : do_div(tinfo->latest_isr_time, NSEC_PER_SEC));
	CCCI_REPEAT_LOG(hif_ctrl->md_id, TAG,
		"net Rx ISR %lu.%06lu, active %d\n",
		(unsigned long)tinfo->latest_isr_time, isr_rem_nsec / 1000,
		hif_ctrl->rxq[0].que_started);
	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		q_rx_rem_nsec[i] = (tinfo->latest_q_rx_isr_time[i] == 0 ?
			0 : do_div(tinfo->latest_q_rx_isr_time[i],
			NSEC_PER_SEC));
		CCCI_REPEAT_LOG(hif_ctrl->md_id, TAG, "net RX:%lu.%06lu, %d\n",
			(unsigned long)tinfo->latest_q_rx_isr_time[i],
			q_rx_rem_nsec[i] / 1000,
			hif_ctrl->rx_traffic_monitor[i]);

		q_rx_rem_nsec[i] = (tinfo->latest_q_rx_time[i] == 0 ?
			0 : do_div(tinfo->latest_q_rx_time[i], NSEC_PER_SEC));
		CCCI_REPEAT_LOG(hif_ctrl->md_id, TAG, "net RXq%d:%lu.%06lu\n",
			i, (unsigned long)tinfo->latest_q_rx_time[i],
			q_rx_rem_nsec[i] / 1000);
	}

	mod_timer(&hif_ctrl->traffic_monitor,
			jiffies + DPMAIF_TRAFFIC_MONITOR_INTERVAL * HZ);
}
#endif

/* =======================================================
 *
 * Descriptions: common part
 *
 * ========================================================
 */

static int dpmaif_queue_broadcast_state(struct hif_dpmaif_ctrl *hif_ctrl,
	enum HIF_STATE state, enum DIRECTION dir, unsigned char index)
{
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
		"%s sta(q%d) %d\n", ((dir == IN) ? "RX":"TX"), (int)index,
		(int)state);
#endif
	ccci_port_queue_status_notify(hif_ctrl->md_id, hif_ctrl->hif_id,
		(int)index, dir, state);
	return 0;
}

static int dpmaif_give_more(unsigned char hif_id, unsigned char qno)
{
	tasklet_hi_schedule(&dpmaif_ctrl->rxq[0].dpmaif_rxq0_task);
	return 0;
}

static int dpmaif_write_room(unsigned char hif_id, unsigned char qno)
{
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;

	if (qno >= DPMAIF_TXQ_NUM)
		return -CCCI_ERR_INVALID_QUEUE_INDEX;
	return atomic_read(&hif_ctrl->txq[qno].tx_budget);
}

static unsigned int  ringbuf_get_next_idx(unsigned int  buf_len,
	unsigned int buf_idx, unsigned int cnt)
{
	buf_idx += cnt;
	if (buf_idx >= buf_len)
		buf_idx -= buf_len;
	return buf_idx;
}

static unsigned int ringbuf_readable(unsigned int  total_cnt,
	unsigned int rd_idx, unsigned int  wrt_idx)
{
	unsigned int pkt_cnt = 0;

	if (wrt_idx >= rd_idx)
		pkt_cnt = wrt_idx - rd_idx;
	else
		pkt_cnt = total_cnt + wrt_idx - rd_idx;

	return pkt_cnt;
}

static unsigned int ringbuf_writeable(unsigned int  total_cnt,
	unsigned int rel_idx, unsigned int  wrt_idx)
{
	unsigned int pkt_cnt = 0;

	if (wrt_idx < rel_idx)
		pkt_cnt = rel_idx - wrt_idx - 1;
	else
		pkt_cnt = total_cnt + rel_idx - wrt_idx - 1;

	return pkt_cnt;
}

static unsigned int ringbuf_releasable(unsigned int  total_cnt,
	unsigned int rel_idx, unsigned int  rd_idx)
{
	unsigned int pkt_cnt = 0;

	if (rel_idx <= rd_idx)
		pkt_cnt = rd_idx - rel_idx;
	else
		pkt_cnt = total_cnt + rd_idx - rel_idx;

	return pkt_cnt;
}

/* =======================================================
 *
 * Descriptions: RX part start
 *
 * ========================================================
 */
static int dpmaif_net_rx_push_thread(void *arg)
{
	struct sk_buff *skb = NULL;
	struct dpmaif_rx_queue *queue = (struct dpmaif_rx_queue *)arg;
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;
#ifndef CCCI_KMODULE_ENABLE
#ifdef CCCI_SKB_TRACE
	struct ccci_per_md *per_md_data = ccci_get_per_md_data(hif_ctrl->md_id);
#endif
#endif
	int count = 0;
	int ret;

	while (1) {
		if (ccci_dl_queue_len(queue->index) == 0) {
			dpmaif_queue_broadcast_state(hif_ctrl, RX_FLUSH,
				IN, queue->index);
			count = 0;
			ret = wait_event_interruptible(queue->rx_wq,
				(ccci_dl_queue_len(queue->index) ||
				kthread_should_stop()));
			ccmni_clr_flush_timer();
			if (ret == -ERESTARTSYS)
				continue;	/* FIXME */
		}
		if (kthread_should_stop())
			break;
		skb = (struct sk_buff *)ccci_dl_dequeue(queue->index);
		if (!skb)
			continue;

		mtk_ccci_add_dl_pkt_bytes(queue->index, skb->len);
#ifndef CCCI_KMODULE_ENABLE
#ifdef CCCI_SKB_TRACE
		per_md_data->netif_rx_profile[6] = sched_clock();
		if (count > 0)
			skb->tstamp = sched_clock();
#endif
#endif
		ccci_port_recv_skb(hif_ctrl->md_id, hif_ctrl->hif_id, skb,
			CLDMA_NET_DATA);
		count++;
#ifndef CCCI_KMODULE_ENABLE
#ifdef CCCI_SKB_TRACE
		per_md_data->netif_rx_profile[6] = sched_clock() -
			per_md_data->netif_rx_profile[6];
		per_md_data->netif_rx_profile[5] = count;
		trace_ccci_skb_rx(per_md_data->netif_rx_profile);
#endif
#endif
	}
	return 0;
}

static int dpmaifq_rel_rx_pit_entry(struct dpmaif_rx_queue *rxq,
			unsigned short rel_entry_num)
{
	unsigned short old_sw_rel_idx = 0, new_sw_rel_idx = 0,
		old_hw_wr_idx = 0;
	int ret = 0;

	if (rxq->que_started != true)
		return 0;

	if (rel_entry_num >= rxq->pit_size_cnt) {
		CCCI_ERROR_LOG(-1, TAG,
			"%s: (num >= rxq->pit_size_cnt)\n", __func__);
		return -1;
	}

	old_sw_rel_idx = rxq->pit_rel_rd_idx;
	new_sw_rel_idx = old_sw_rel_idx + rel_entry_num;

	old_hw_wr_idx = rxq->pit_wr_idx;

	/*queue had empty and no need to release*/
	if (old_hw_wr_idx == old_sw_rel_idx) {
		CCCI_HISTORY_LOG(-1, TAG,
			"%s: (old_hw_wr_idx == old_sw_rel_idx)\n", __func__);
	}

	if (old_hw_wr_idx > old_sw_rel_idx) {
		if (new_sw_rel_idx > old_hw_wr_idx) {
			CCCI_HISTORY_LOG(-1, TAG,
				"%s: (new_rel_idx > old_hw_wr_idx)\n",
				__func__);
		}
	} else if (old_hw_wr_idx < old_sw_rel_idx) {
		if (new_sw_rel_idx >= rxq->pit_size_cnt) {
			new_sw_rel_idx = new_sw_rel_idx - rxq->pit_size_cnt;
			if (new_sw_rel_idx > old_hw_wr_idx) {
				CCCI_HISTORY_LOG(-1, TAG,
					"%s: (new_rel_idx > old_wr_idx)\n",
					__func__);
			}
		}
	}

	rxq->pit_rel_rd_idx = new_sw_rel_idx;
	ret = drv3_dpmaif_dl_add_pit_remain_cnt(rxq->index, rel_entry_num);

	return ret;
}

static inline void dpmaif_rx_msg_pit(struct dpmaif_rx_queue *rxq,
	struct dpmaifq_msg_pit *msg_pit)
{
	rxq->cur_chn_idx = msg_pit->channel_id;
	rxq->check_sum = msg_pit->check_sum;
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG,
		"rxq%d received a message pkt: channel=%d, checksum=%d\n",
		rxq->index, rxq->cur_chn_idx, rxq->check_sum);
#endif
}

#ifdef HW_FRG_FEATURE_ENABLE
static int dpmaif_set_rx_frag_to_skb(struct dpmaif_rx_queue *rxq,
	unsigned int skb_idx, struct dpmaifq_normal_pit *pkt_inf_t)
{
	struct dpmaif_bat_skb_t *cur_skb_info = ((struct dpmaif_bat_skb_t *)
		dpmaif_ctrl->bat_req->bat_skb_ptr + skb_idx);
	struct sk_buff *base_skb = cur_skb_info->skb;
	struct dpmaif_bat_page_t *cur_page_info = ((struct dpmaif_bat_page_t *)
		dpmaif_ctrl->bat_frag->bat_skb_ptr +
			(pkt_inf_t->buffer_id | ((pkt_inf_t->h_bid) << 13)));
	struct page *page = cur_page_info->page;
#ifndef REFINE_BAT_OFFSET_REMOVE
	unsigned long long data_phy_addr, data_base_addr;
#endif
	int data_offset = 0;
	unsigned int data_len;
	int ret = 0;

	/* rx current frag data unmapping */
	dma_unmap_page(
		ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
		cur_page_info->data_phy_addr, cur_page_info->data_len,
		DMA_FROM_DEVICE);
	if (!page) {
		CCCI_ERROR_LOG(-1, TAG, "frag check fail: 0x%x, 0x%x",
			pkt_inf_t->buffer_id | ((pkt_inf_t->h_bid) << 13),
			skb_idx);

		return DATA_CHECK_FAIL;
	}

#ifndef REFINE_BAT_OFFSET_REMOVE
	/* 2. calculate data address && data len. */
	data_phy_addr = pkt_inf_t->data_addr_ext;
	data_phy_addr = (data_phy_addr<<32) + pkt_inf_t->p_data_addr;
	data_base_addr = (unsigned long long)cur_page_info->data_phy_addr;

	data_offset = (int)(data_phy_addr - data_base_addr);
#endif

	data_len = pkt_inf_t->data_len;
	/* 3. record to skb for user: fragment data to nr_frags */
	skb_add_rx_frag(base_skb, skb_shinfo(base_skb)->nr_frags,
		page, cur_page_info->offset + data_offset,
		data_len, cur_page_info->data_len);

	cur_page_info->page = NULL;

	return ret;
}
#endif

static int dpmaif_rx_set_data_to_skb(struct dpmaif_rx_queue *rxq,
	struct dpmaifq_normal_pit *pkt_inf_t)
{
	struct sk_buff *new_skb = NULL;
	struct dpmaif_bat_skb_t *cur_skb_info = ((struct dpmaif_bat_skb_t *)
		dpmaif_ctrl->bat_req->bat_skb_ptr +
		(pkt_inf_t->buffer_id | ((pkt_inf_t->h_bid) << 13)));

	#ifndef REFINE_BAT_OFFSET_REMOVE
	unsigned long long data_phy_addr, data_base_addr;
	int data_offset = 0;
	#endif
	unsigned int data_len;
	unsigned int *temp_u32 = NULL;
	unsigned int buffer_id;

	/* rx current skb data unmapping */
	dma_unmap_single(ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
		cur_skb_info->data_phy_addr, cur_skb_info->data_len,
		DMA_FROM_DEVICE);

	#ifndef REFINE_BAT_OFFSET_REMOVE
	/* 2. calculate data address && data len. */
	/* cur pkt physical address(in a buffer bid pointed) */
	data_phy_addr = pkt_inf_t->data_addr_ext;
	data_phy_addr = (data_phy_addr<<32) + pkt_inf_t->p_data_addr;
	data_base_addr = (unsigned long long)cur_skb_info->data_phy_addr;
	data_offset = (int)(data_phy_addr - data_base_addr);
	#endif
	data_len = pkt_inf_t->data_len; /* cur pkt data len */

	/* 3. record to skb for user: wapper, enqueue */
	 /* get skb which data contained pkt data */
	new_skb = cur_skb_info->skb;

	buffer_id = pkt_inf_t->buffer_id | ((pkt_inf_t->h_bid) << 13);

	if (new_skb == NULL) {
		CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
			"get null skb(0x%x) from skb table",
			buffer_id);
		return DATA_CHECK_FAIL;
	}
	new_skb->len = 0;
	skb_reset_tail_pointer(new_skb);

	#ifndef REFINE_BAT_OFFSET_REMOVE
	/*set data len, data pointer*/
	skb_reserve(new_skb, data_offset);
	#endif
	/* for debug: */
	if (unlikely((new_skb->tail + data_len) > new_skb->end)) {
		/*dump*/
		#ifndef REFINE_BAT_OFFSET_REMOVE
		CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
			"pkt(%d/%d): len = 0x%x, offset=0x%llx-0x%llx, skb(%p, %p, 0x%x, 0x%x)\n",
			rxq->pit_rd_idx, buffer_id, data_len,
			data_phy_addr, data_base_addr, new_skb->head,
			new_skb->data, (unsigned int)new_skb->tail,
			(unsigned int)new_skb->end);
		#else
		CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
			"pkt(%d/%d): len = 0x%x, skb(%p, %p, 0x%x, 0x%x)\n",
			rxq->pit_rd_idx, buffer_id, data_len,
			new_skb->head, new_skb->data,
			(unsigned int)new_skb->tail,
			(unsigned int)new_skb->end);
		#endif

		if (rxq->pit_rd_idx > 2) {
			temp_u32 = (unsigned int *)
				((struct dpmaifq_normal_pit *)
				rxq->pit_base + rxq->pit_rd_idx - 2);
			CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
				"pit(%d):data(%x, %x, %x, %x, %x, %x, %x, %x, %x)\n",
				rxq->pit_rd_idx - 2, temp_u32[0], temp_u32[1],
				temp_u32[2], temp_u32[3], temp_u32[4],
				temp_u32[5], temp_u32[6],
				temp_u32[7], temp_u32[8]);
		}
		dpmaif_dump_rxq_remain(dpmaif_ctrl, DPMAIF_RXQ_NUM, 1);
		/* force modem assert: ERROR_STOP */
		/* CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
		 *	"Force MD assert called by %s(%d/%d)\n",
		 *	__func__, cur_pit, cur_bid);
		 * ret = ccci_md_force_assert(dpmaif_ctrl->md_id,
		 *	MD_FORCE_ASSERT_BY_AP_Q0_BLOCKED, NULL, 0);
		 */
		return DATA_CHECK_FAIL;
	}

	skb_put(new_skb, data_len);
	new_skb->ip_summed = (rxq->check_sum == 0) ?
		CHECKSUM_UNNECESSARY : CHECKSUM_NONE;

	if (dpmaif_ctrl->support_lro)
#ifndef GET_HEADER_OFFSET_FROM_PIT
		dpmaif_rxq_lro_join_skb(rxq, new_skb, buffer_id);
#else
		dpmaif_rxq_lro_join_skb(rxq, new_skb, buffer_id,
				pkt_inf_t->header_offset);
#endif

	return 0;
}

static int dpmaif_send_skb_to_net(struct dpmaif_rx_queue *rxq,
	unsigned int skb_idx)
{
	struct dpmaif_bat_skb_t *cur_skb = ((struct dpmaif_bat_skb_t *)
			dpmaif_ctrl->bat_req->bat_skb_ptr + skb_idx);
	struct sk_buff *new_skb = cur_skb->skb;
	int ret = 0;

	struct lhif_header *lhif_h = NULL; /* for uplayer: port/ccmni */
	struct ccci_header ccci_h; /* for collect debug info. */

	if (rxq->pit_dp) {
		dev_kfree_skb_any(new_skb);
		goto END;
	}

	if (atomic_cmpxchg(&dpmaif_ctrl->wakeup_src, 1, 0) == 1) {
		CCCI_NOTICE_LOG(dpmaif_ctrl->md_id, TAG,
			"DPMA_MD wakeup source:(%d/%d)\n",
			rxq->index, rxq->cur_chn_idx);
		dpmaif_handle_wakeup(rxq, new_skb);
	}

	/* md put the ccmni_index to the msg pkt,
	 * so we need push it by self. maybe no need
	 */
	lhif_h = (struct lhif_header *)(skb_push(new_skb,
			sizeof(struct lhif_header)));
	lhif_h->netif = rxq->cur_chn_idx;

	/* record before add to skb list. */
	ccci_h = *(struct ccci_header *)new_skb->data;
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "new skb len: 0x%x+0x%x = 0x%x=> 0x%x\n",
		skb_headlen(new_skb), new_skb->data_len,
		new_skb->len, new_skb->truesize);
#endif
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	dpmaif_ctrl->rx_traffic_monitor[rxq->index]++;
#endif
	ccci_md_add_log_history(&dpmaif_ctrl->traffic_info, IN,
		(int)rxq->index, &ccci_h, 0);

	ccci_dl_enqueue(rxq->index, new_skb);
END:
	cur_skb->skb = NULL;
	return ret;
}

struct dpmaif_rx_hw_notify {
	unsigned short pit_cnt;
};

static int dpmaifq_rx_notify_hw(struct dpmaif_rx_queue *rxq,
	struct dpmaif_rx_hw_notify *notify_cnt)
{
	int ret = 0;

	if (notify_cnt->pit_cnt) {
		ccci_dpmaif_bat_wakeup_thread_v3();

		ret = dpmaifq_rel_rx_pit_entry(rxq, notify_cnt->pit_cnt);
		if (ret < 0) {
			CCCI_MEM_LOG_TAG(0, TAG,
				"dpmaif: update pit fail(128)\n");
			return ret;
		}
		notify_cnt->pit_cnt = 0;
	}

	return ret;
}

/*
 * #define GET_PKT_INFO_PTR(rxq, pit_idx)  \
 * ((struct dpmaifq_normal_pit *)rxq->pit_base + pit_idx)

 * #define GET_BUF_ADDR_PTR(bat_table, bat_idx)  \
 * ((struct dpmaif_bat_t *)bat_table->bat_base + bat_idx)
 * #define GET_BUF_SKB_PTR(bat_table, bat_idx) \
 *	((struct dpmaif_bat_skb_t *)bat_table->bat_skb_ptr + bat_idx)
 */
#define NOTIFY_RX_PUSH(rxq)  wake_up_all(&rxq->rx_wq)

static int dpmaif_rx_start(struct dpmaif_rx_queue *rxq, unsigned short pit_cnt,
		int blocking, unsigned long time_limit)
{
	struct dpmaif_rx_hw_notify notify_hw = {0};
#if (!defined(PIT_USING_CACHE_MEM))
	struct dpmaifq_normal_pit pkt_inf_s;
#endif
	struct dpmaifq_normal_pit *pkt_inf_t = NULL;
	unsigned short rx_cnt;
	unsigned int pit_len = rxq->pit_size_cnt;
	unsigned int cur_pit;
	unsigned short recv_skb_cnt = 0;
	int ret = 0, ret_hw = 0;
	unsigned int buffer_id;

#ifdef PIT_USING_CACHE_MEM
	dma_addr_t cache_start_addr;
#endif
	struct dpmaif_rx_lro_info *lro_info = &rxq->lro_info;

	cur_pit = rxq->pit_rd_idx;

#ifdef PIT_USING_CACHE_MEM
	cache_start_addr = rxq->pit_phy_addr +
		sizeof(struct dpmaifq_normal_pit) * cur_pit;

	if ((cur_pit + pit_cnt) <= pit_len) {
		dma_sync_single_for_cpu(
				ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
				cache_start_addr,
				sizeof(struct dpmaifq_normal_pit) * pit_cnt,
				DMA_FROM_DEVICE);
	} else {

		dma_sync_single_for_cpu(
			ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
			cache_start_addr,
			sizeof(struct dpmaifq_normal_pit)
					* (pit_len - cur_pit),
			DMA_FROM_DEVICE);
		cache_start_addr = rxq->pit_phy_addr;
		dma_sync_single_for_cpu(
			ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
			cache_start_addr,
			sizeof(struct dpmaifq_normal_pit)
				* (cur_pit + pit_cnt - pit_len),
			DMA_FROM_DEVICE);
	}
#endif

	for (rx_cnt = 0; rx_cnt < pit_cnt; rx_cnt++) {
		if (!blocking && time_after_eq(jiffies, time_limit)) {
			CCCI_DEBUG_LOG(dpmaif_ctrl->md_id, TAG,
			"%s: timeout, cnt = %d/%d\n", __func__,
			rx_cnt, pit_cnt);
			break;
		}
		/*GET_PKT_INFO_PTR(rxq, cur_pit); pit item */
#ifndef PIT_USING_CACHE_MEM
		pkt_inf_s = *((struct dpmaifq_normal_pit *)rxq->pit_base +
			cur_pit);
		pkt_inf_t = &pkt_inf_s;
#else
		pkt_inf_t = (struct dpmaifq_normal_pit *)rxq->pit_base +
			cur_pit;
#endif
		if ((dpmaif_ctrl->enable_pit_debug > 0) &&
			dpmaif_debug_add_data(&rxq->dbg_data, pkt_inf_t,
				sizeof(struct dpmaifq_normal_pit)) < 0) {
			dpmaif_debug_push_data(&rxq->dbg_data,
				rxq->index, rxq->cur_chn_idx);
			dpmaif_debug_add_data(&rxq->dbg_data, pkt_inf_t,
				sizeof(struct dpmaifq_normal_pit));
		}

		if (pkt_inf_t->packet_type == DES_PT_MSG) {
			dpmaif_rx_msg_pit(rxq,
					(struct dpmaifq_msg_pit *)pkt_inf_t);

			rxq->skb_idx = -1;
			rxq->pit_dp =
					((struct dpmaifq_msg_pit *)pkt_inf_t)->dp;

			if (dpmaif_ctrl->support_lro)
				dpmaif_rxq_lro_start(lro_info);

		} else if (pkt_inf_t->packet_type == DES_PT_PD) {
			buffer_id = (pkt_inf_t->buffer_id |
					((pkt_inf_t->h_bid) << 13));
#ifdef HW_FRG_FEATURE_ENABLE
			if ((pkt_inf_t->buffer_type == PKT_BUF_FRAG)
					&& rxq->skb_idx < 0) {
				/* msg+frag pit, no data pkt received. */
				CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
					"skb_idx < 0 pit/bat/frag = %d, %d; buf: %d; %d, %d\n",
					cur_pit, buffer_id,
					pkt_inf_t->buffer_type, rx_cnt,
					pit_cnt);
#ifdef DPMAIF_DEBUG_LOG
				CCCI_HISTORY_TAG_LOG(dpmaif_ctrl->md_id, TAG,
					"skb_idx < 0 pit/bat/frag = %d, %d; buf: %d; %d, %d\n",
					cur_pit, buffer_id,
					pkt_inf_t->buffer_type, rx_cnt,
					pit_cnt);
#endif
				CCCI_MEM_LOG_TAG(dpmaif_ctrl->md_id, TAG,
					"skb_idx < 0 pit/bat/frag = %d, %d; buf: %d; %d, %d\n",
					cur_pit, buffer_id,
					pkt_inf_t->buffer_type, rx_cnt,
					pit_cnt);

				dpmaif_dump_rxq_remain(dpmaif_ctrl,
					DPMAIF_RXQ_NUM, 1);

					ret = DATA_CHECK_FAIL;
					break;
			}

			if (pkt_inf_t->buffer_type != PKT_BUF_FRAG) {
#endif
				/* skb->data: add to skb ptr */
				/* && record ptr. */
				rxq->skb_idx = (int)buffer_id;
				ret = dpmaif_rx_set_data_to_skb(rxq, pkt_inf_t);
				if (ret < 0)
					break;

#ifdef HW_FRG_FEATURE_ENABLE
			} else {
				/* skb->frag_list: add to frag_list */
				ret = dpmaif_set_rx_frag_to_skb(rxq,
						(unsigned int)rxq->skb_idx,
						pkt_inf_t);
				if (ret < 0)
					break;
			}
#endif
			if (pkt_inf_t->c_bit == 0) {
				/* last one, not msg pit, && data had rx. */
				if (dpmaif_ctrl->support_lro &&
						lro_info->count > 1)
					ret = dpmaif_rxq_lro_end(rxq);
				else
					ret = dpmaif_send_skb_to_net(rxq,
							rxq->skb_idx);
				if (ret < 0)
					break;
				recv_skb_cnt++;
				if ((recv_skb_cnt&0x7) == 0) {
					NOTIFY_RX_PUSH(rxq);
					recv_skb_cnt = 0;
				}
			}
		}

		/* get next pointer to get pkt data */
		cur_pit = ringbuf_get_next_idx(pit_len, cur_pit, 1);
		rxq->pit_rd_idx = cur_pit;
		notify_hw.pit_cnt++;
		if ((notify_hw.pit_cnt & 0x7F) == 0) {
			ret_hw = dpmaifq_rx_notify_hw(rxq, &notify_hw);
			if (ret_hw < 0)
				break;
		}
	}

	/* still need sync to SW/HW cnt. */
	if (recv_skb_cnt)
		NOTIFY_RX_PUSH(rxq);

	/* update to HW */
	if (ret_hw == 0 && notify_hw.pit_cnt)
		ret_hw = dpmaifq_rx_notify_hw(rxq, &notify_hw);
	if (ret_hw < 0 && ret == 0)
		ret = ret_hw;

#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "%s: pit:0x%x, 0x%x, 0x%x\n",
		__func__, rxq->pit_wr_idx, rxq->pit_rd_idx,
			rxq->pit_rel_rd_idx);
	CCCI_HISTORY_LOG(-1, TAG, "%s:bat: 0x%x, 0x%x\n",
		__func__, dpmaif_ctrl->bat_req->bat_wr_idx,
		dpmaif_ctrl->bat_req->bat_rd_idx);
#ifdef HW_FRG_FEATURE_ENABLE
	CCCI_HISTORY_LOG(-1, TAG, "%s:bat_frag: 0x%x, 0x%x\n",
		__func__, dpmaif_ctrl->bat_frag->bat_wr_idx,
		dpmaif_ctrl->bat_frag->bat_rd_idx);
#endif
#endif
	return ret < 0?ret:rx_cnt;
}

static unsigned int dpmaifq_poll_rx_pit(struct dpmaif_rx_queue *rxq)
{
	unsigned int pit_cnt = 0;
	unsigned short sw_rd_idx = 0, hw_wr_idx = 0;

	if (rxq->que_started == false)
		return pit_cnt;
	sw_rd_idx = rxq->pit_rd_idx;
#ifdef DPMAIF_NOT_ACCESS_HW
	hw_wr_idx = rxq->pit_size_cnt - 1;
#else
	hw_wr_idx = drv3_dpmaif_dl_get_wridx(rxq->index);
#endif
	pit_cnt = ringbuf_readable(rxq->pit_size_cnt, sw_rd_idx, hw_wr_idx);

	rxq->pit_wr_idx = hw_wr_idx;
	return pit_cnt;
}

/*
 * may be called from workqueue or NAPI or tasklet context,
 * NAPI and tasklet with blocking=false
 */
static int dpmaif_rx_data_collect(struct hif_dpmaif_ctrl *hif_ctrl,
		unsigned char q_num, int budget, int blocking)
{
	struct dpmaif_rx_queue *rxq = &hif_ctrl->rxq[q_num];
	unsigned int cnt, rd_cnt = 0, max_cnt = budget;
	int ret = ALL_CLEAR, real_cnt = 0;
	unsigned int L2RISAR0;
	unsigned long time_limit = jiffies + 2;

	cnt = dpmaifq_poll_rx_pit(rxq);
	if (cnt) {
		max_cnt = cnt;
		rd_cnt = (cnt > budget && !blocking) ? budget:cnt;
		real_cnt = dpmaif_rx_start(rxq, rd_cnt, blocking, time_limit);

		if (real_cnt < LOW_MEMORY_TYPE_MAX) {
			ret = LOW_MEMORY;
			CCCI_ERROR_LOG(-1, TAG, "rx low mem: %d\n", real_cnt);
		} else if (real_cnt <= ERROR_STOP_MAX) {
			ret = ERROR_STOP;
			CCCI_ERROR_LOG(-1, TAG, "rx ERR_STOP: %d\n", real_cnt);
		} else if (real_cnt < 0) {
			ret = LOW_MEMORY;
			CCCI_ERROR_LOG(-1, TAG, "rx ERROR: %d\n", real_cnt);
		} else if (real_cnt < max_cnt)
			ret = ONCE_MORE;
		else
			ret = ALL_CLEAR;
	} else
		ret = ALL_CLEAR;

	/* hw ack. */
	if (ret == ONCE_MORE) {
		/* hw interrupt ack. */
		L2RISAR0 = drv3_dpmaif_get_dl_isr_event();
		L2RISAR0 &= DPMAIF_DL_INT_QDONE_MSK;
		if (L2RISAR0)
			DPMA_WRITE_PD_MISC(DPMAIF_PD_AP_DL_L2TISAR0,
				L2RISAR0&DPMAIF_DL_INT_QDONE_MSK);

	}
	return ret;
}

static void dpmaif_rxq0_work(struct work_struct *work)
{
	struct dpmaif_rx_queue *rxq =
		container_of(work, struct dpmaif_rx_queue, dpmaif_rxq0_work);
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;
	int ret;

	atomic_set(&rxq->rx_processing, 1);
	smp_mb(); /* for cpu exec. */
	if (rxq->que_started != true) {
		ret = ALL_CLEAR;
		atomic_set(&rxq->rx_processing, 0);
		return;
	}

	hif_ctrl->traffic_info.latest_q_rx_time[rxq->index] = local_clock();
	ret = dpmaif_rx_data_collect(hif_ctrl, rxq->index, rxq->budget, 1);

	if (ret == ONCE_MORE) {
		tasklet_hi_schedule(&hif_ctrl->rxq[0].dpmaif_rxq0_task);
	} else if (unlikely(ret == LOW_MEMORY)) {
		/*Rx done and empty interrupt will be enabled in workqueue*/
		queue_work(rxq->worker,
			&rxq->dpmaif_rxq0_work);
	} else if (unlikely(ret == ERROR_STOP)) {
		ret = ccci_md_force_assert(dpmaif_ctrl->md_id,
			MD_FORCE_ASSERT_BY_AP_Q0_BLOCKED,
			NULL, 0);
	} else {
		/* clear IP busy register wake up cpu case */
		drv3_dpmaif_clear_ip_busy();
		drv3_dpmaif_unmask_dl_interrupt(rxq->index);
	}
	atomic_set(&rxq->rx_processing, 0);
}

static void dpmaif_rxq0_tasklet(unsigned long data)
{
	struct hif_dpmaif_ctrl *hif_ctrl = (struct hif_dpmaif_ctrl *)data;
	struct dpmaif_rx_queue *rxq = &hif_ctrl->rxq[0];
	int ret;

	atomic_set(&rxq->rx_processing, 1);
#ifdef DPMAIF_DEBUG_LOG
	hif_ctrl->traffic_info.rx_tasket_cnt++;
#endif
	smp_mb(); /* for cpu exec. */
	if (rxq->que_started != true) {
		ret = ALL_CLEAR;
		atomic_set(&rxq->rx_processing, 0);
		return;
	}
	hif_ctrl->traffic_info.latest_q_rx_time[rxq->index] = local_clock();
	ret = dpmaif_rx_data_collect(hif_ctrl, rxq->index, rxq->budget, 0);

	if (ret == ONCE_MORE) {
		tasklet_hi_schedule(&rxq->dpmaif_rxq0_task);
	} else if (unlikely(ret == LOW_MEMORY)) {
		/*Rx done and empty interrupt will be enabled in workqueue*/
		queue_work(rxq->worker,
			&rxq->dpmaif_rxq0_work);
	} else if (unlikely(ret == ERROR_STOP)) {
		ret = ccci_md_force_assert(dpmaif_ctrl->md_id,
			MD_FORCE_ASSERT_BY_AP_Q0_BLOCKED,
			NULL, 0);
	} else {
		/* clear IP busy register wake up cpu case */
		drv3_dpmaif_clear_ip_busy();
		drv3_dpmaif_unmask_dl_interrupt(rxq->index);
	}
	atomic_set(&rxq->rx_processing, 0);

	CCCI_DEBUG_LOG(hif_ctrl->md_id, TAG, "rxq0 tasklet result %d\n", ret);
}

/* =======================================================
 *
 * Descriptions:  TX part start
 *
 * ========================================================
 */

static unsigned int dpmaifq_poll_tx_drb(unsigned char q_num)
{
	struct dpmaif_tx_queue *txq;
	unsigned int drb_cnt = 0;
	unsigned short old_sw_rd_idx = 0, new_hw_rd_idx = 0;

	txq = &dpmaif_ctrl->txq[q_num];

	if (txq->que_started == false)
		return drb_cnt;

	old_sw_rd_idx = txq->drb_rd_idx;

	new_hw_rd_idx = (drv3_dpmaif_ul_get_ridx(q_num) /
		DPMAIF_UL_DRB_ENTRY_WORD);
	if (old_sw_rd_idx <= new_hw_rd_idx)
		drb_cnt = new_hw_rd_idx - old_sw_rd_idx;
	else
		drb_cnt = txq->drb_size_cnt - old_sw_rd_idx + new_hw_rd_idx;

	txq->drb_rd_idx = new_hw_rd_idx;
	return drb_cnt;
}

static unsigned short dpmaif_relase_tx_buffer(unsigned char q_num,
	unsigned int release_cnt)
{
	unsigned int drb_entry_num, idx;
	unsigned int *temp = NULL;
	unsigned short cur_idx;
	struct dpmaif_drb_pd *cur_drb = NULL;
	struct dpmaif_drb_pd *drb_base =
		(struct dpmaif_drb_pd *)(dpmaif_ctrl->txq[q_num].drb_base);
	struct sk_buff *skb_free = NULL;
	struct dpmaif_tx_queue *txq = &dpmaif_ctrl->txq[q_num];
	struct dpmaif_drb_skb *cur_drb_skb = NULL;

	if (release_cnt <= 0)
		return 0;

	drb_entry_num = txq->drb_size_cnt;
	cur_idx = txq->drb_rel_rd_idx;

	for (idx = 0 ; idx < release_cnt ; idx++) {
		cur_drb = drb_base + cur_idx;
		if (cur_drb->dtyp == DES_DTYP_PD && cur_drb->c_bit == 0) {
			CCCI_DEBUG_LOG(dpmaif_ctrl->md_id, TAG,
				"rxq%d release tx drb %d\n", q_num, cur_idx);
			cur_drb_skb =
				((struct dpmaif_drb_skb *)txq->drb_skb_base +
				cur_idx);
			dma_unmap_single(
				ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
				cur_drb_skb->phy_addr, cur_drb_skb->data_len,
				DMA_TO_DEVICE);
			skb_free = cur_drb_skb->skb;
			if (skb_free == NULL) {
				temp = (unsigned int *)cur_drb;
				CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
					"txq (%d)pkt(%d): drb check fail, (w/r/rel=%x, %x, %x)\n",
					q_num, cur_idx,
					txq->drb_wr_idx,
					txq->drb_rd_idx,
					txq->drb_rel_rd_idx);
				CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
					"drb pd: 0x%x, 0x%x (0x%x, 0x%x, 0x%x)\n",
					temp[0], temp[1],
					drv3_dpmaif_ul_get_rwidx(0),
					drv3_dpmaif_ul_get_rwidx(1),
					drv3_dpmaif_ul_get_rwidx(3));
				dpmaif_dump_register(dpmaif_ctrl,
					CCCI_DUMP_MEM_DUMP);
				dpmaif_dump_txq_history(dpmaif_ctrl,
					txq->index, 0);
				dpmaif_dump_txq_remain(dpmaif_ctrl,
					txq->index, 0);
				/* force dump */
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
				aee_kernel_warning("ccci",
					"dpmaif: tx warning");
#endif
				return DATA_CHECK_FAIL;
			}
			ccci_free_skb(skb_free);
			cur_drb_skb->skb = NULL;
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
			dpmaif_ctrl->tx_traffic_monitor[txq->index]++;
#endif
		} else if (cur_drb->dtyp == DES_DTYP_MSG) {
			txq->last_ch_id =
				((struct dpmaif_drb_msg *)cur_drb)->channel_id;
		} else {
			/* tx unmapping */
			cur_drb_skb = ((struct dpmaif_drb_skb *)
				txq->drb_skb_base + cur_idx);
			dma_unmap_single(
				ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
				cur_drb_skb->phy_addr, cur_drb_skb->data_len,
				DMA_TO_DEVICE);
		}

		cur_idx = ringbuf_get_next_idx(drb_entry_num, cur_idx, 1);
		txq->drb_rel_rd_idx = cur_idx;
		atomic_inc(&txq->tx_budget);
		if (likely(ccci_md_get_cap_by_id(dpmaif_ctrl->md_id)
			& MODEM_CAP_TXBUSY_STOP)) {
			if (atomic_read(&txq->tx_budget) >
				txq->drb_size_cnt / 8)
				dpmaif_queue_broadcast_state(dpmaif_ctrl,
					TX_IRQ, OUT, txq->index);
		}
		/* check wakeup source */
		if (atomic_cmpxchg(&dpmaif_ctrl->wakeup_src, 1, 0) == 1)
			CCCI_NOTICE_LOG(dpmaif_ctrl->md_id, TAG,
				"DPMA_MD wakeup source:(%d/%d%s)\n",
				txq->index, txq->last_ch_id,
				(cur_drb->dtyp == DES_DTYP_MSG) ?
					"" : "/data 1st received");
	}
	if (cur_drb->c_bit != 0)
		CCCI_DEBUG_LOG(dpmaif_ctrl->md_id, TAG,
			"txq(%d)_done: last one: c_bit != 0 ???\n", q_num);
	return idx;
}

int dpmaif_empty_query_v3(int qno)
{
	struct dpmaif_tx_queue *txq = &dpmaif_ctrl->txq[qno];

	if (txq == NULL) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"%s:fail NULL txq\n", __func__);
		return 0;
	}
	return atomic_read(&txq->tx_budget) >
				txq->drb_size_cnt / 8;
}

static atomic_t s_tx_busy_assert_on;
static int dpmaif_tx_release(unsigned char q_num, unsigned short budget)
{
	unsigned int rel_cnt, hw_rd_cnt, real_rel_cnt;
	struct dpmaif_tx_queue *txq = &dpmaif_ctrl->txq[q_num];

	/* update rd idx: from HW */
	hw_rd_cnt = dpmaifq_poll_tx_drb(q_num);
	rel_cnt = ringbuf_releasable(txq->drb_size_cnt,
				txq->drb_rel_rd_idx, txq->drb_rd_idx);

	if (budget != 0 && rel_cnt > budget)
		real_rel_cnt = budget;
	else
		real_rel_cnt = rel_cnt;
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
		"%s hw cnt = 0x%x, 0x%x, 0x%x\n", __func__,
		hw_rd_cnt, rel_cnt, real_rel_cnt);
#endif
	if (real_rel_cnt) {
		/* release data buff */
		real_rel_cnt = dpmaif_relase_tx_buffer(q_num, real_rel_cnt);
	}
	/* not need to know release idx hw, hw just update rd, and read wrt */
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	dpmaif_ctrl->tx_done_last_count[q_num] = real_rel_cnt;
#endif

	if (real_rel_cnt < 0 || txq->que_started == false)
		return ERROR_STOP;
	else
		return ((real_rel_cnt < rel_cnt)?ONCE_MORE : ALL_CLEAR);
}

static int dpmaif_wait_resume_done(void)
{
	int cnt = 0;

	while (atomic_read(&dpmaif_ctrl->suspend_flag) == 1) {
		cnt++;
		if (cnt >= 1600000) {
			CCCI_NORMAL_LOG(-1, TAG,
				"[%s] warning: suspend_flag = 1; (cnt: %d)",
				__func__, cnt);
			return -1;
		}
	}

	if (cnt)
		CCCI_NORMAL_LOG(-1, TAG,
			"[%s] suspend_flag = 0; (cnt: %d)",
			__func__, cnt);

	return 0;
}

static enum hrtimer_restart tx_done_action(struct hrtimer *timer)
{
	struct dpmaif_tx_queue *txq =
		container_of(timer, struct dpmaif_tx_queue, tx_done_timer);

	atomic_set(&txq->txq_done, 1);
	wake_up_all(&txq->tx_done_wait);

	return HRTIMER_NORESTART;
}

static int dpmaif_tx_done_kernel_thread(void *arg)
{
	struct dpmaif_tx_queue *txq = (struct dpmaif_tx_queue *)arg;
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;
	int ret, affinity_set = -1, tx_aff;
	unsigned int L2TISAR0;
	struct cpumask tmask;

	while (1) {
		ret = wait_event_interruptible(txq->tx_done_wait,
				(atomic_read(&txq->txq_done)
				|| kthread_should_stop()));
		if (kthread_should_stop())
			break;
		if (ret == -ERESTARTSYS)
			continue;

		if (affinity_set != mtk_ccci_get_tx_done_aff(txq->index)) {
			affinity_set = mtk_ccci_get_tx_done_aff(txq->index);
			if (affinity_set <= 0)
				tx_aff = 0xFF;
			else
				tx_aff = affinity_set;

			dpmaif_set_cpu_mask(&tmask, (u32)tx_aff, 8);

			ret = set_cpus_allowed_ptr(txq->tx_done_thread, &tmask);
			CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
				"[%s] txq%d; aff: 0x%X; ret: %d\n",
				__func__, txq->index, (u8)tx_aff, ret);
		}

		atomic_set(&txq->txq_done, 0);
		/* This is used to avoid race condition which may cause KE */
		if (dpmaif_ctrl->dpmaif_state != HIFDPMAIF_STATE_PWRON) {
			CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
				"%s meet hw power down(%d)\n",
				__func__, txq->index);
			continue;
		}
		if (atomic_read(&txq->tx_resume_done)) {
			CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
				"txq%d done/resume: 0x%x, 0x%x, 0x%x\n",
				txq->index,
				drv3_dpmaif_ul_get_rwidx(0),
				drv3_dpmaif_ul_get_rwidx(1),
				drv3_dpmaif_ul_get_rwidx(3));
			continue;
		}
		if (!txq->que_started) {
			CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
				"%s meet queue stop(%d)\n",
				__func__, txq->index);
			continue;
		}

#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
		hif_ctrl->tx_done_last_start_time[txq->index] = local_clock();
#endif

		ret = dpmaif_tx_release(txq->index, txq->drb_size_cnt);

		L2TISAR0 = drv3_dpmaif_get_ul_isr_event();
		L2TISAR0 &= (DPMAIF_UL_INT_QDONE_MSK & (1 << (txq->index +
			UL_INT_DONE_OFFSET)));
		if (ret == ERROR_STOP) {

		} else if (ret == ONCE_MORE || L2TISAR0) {
			hrtimer_start(&txq->tx_done_timer,
					ktime_set(0, 500000), HRTIMER_MODE_REL);

			if (L2TISAR0)
				DPMA_WRITE_PD_MISC(DPMAIF_PD_AP_UL_L2TISAR0,
							L2TISAR0);
		} else {
			/* clear IP busy register wake up cpu case */
			drv3_dpmaif_clear_ip_busy();
			/* enable tx done interrupt */
			drv3_dpmaif_unmask_ul_interrupt(txq->index);
		}
	}

	return 0;
}

static void wait_tx_done_thread_finish(struct dpmaif_tx_queue *txq)
{
	int ret;

	if (txq == NULL) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"%s meet NULL pointer\n", __func__);
		return;
	}

	hrtimer_cancel(&txq->tx_done_timer);
	msleep(20); /* Make sure hrtimer finish */

	while (!IS_ERR(txq->tx_done_thread)) {
		ret = kthread_stop(txq->tx_done_thread);
		if (ret == -EINTR) {
			CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
				"%s stop kthread meet EINTR\n", __func__);
			continue;
		} else
			break;
	}

}

static void set_drb_msg(unsigned char q_num, unsigned short cur_idx,
	unsigned int pkt_len, unsigned short count_l, unsigned char channel_id,
	unsigned short network_type, unsigned short ipv4, unsigned short l4)
{
	struct dpmaif_drb_msg *drb =
		((struct dpmaif_drb_msg *)dpmaif_ctrl->txq[q_num].drb_base +
		cur_idx);
#ifdef DPMAIF_DEBUG_LOG
	unsigned int *temp = NULL;
#endif

	drb->dtyp = DES_DTYP_MSG;
	drb->c_bit = 1;
	drb->packet_len = pkt_len;
	drb->count_l = count_l;
	drb->channel_id = channel_id;
	switch (network_type) {
	/*case htons(ETH_P_IP):
	 * drb->network_type = 4;
	 * break;
	 *
	 * case htons(ETH_P_IPV6):
	 * drb->network_type = 6;
	 * break;
	 */
	default:
		drb->network_type = 0;
		break;
	}
	drb->ipv4 = ipv4;
	drb->l4 = l4;
#ifdef DPMAIF_DEBUG_LOG
	temp = (unsigned int *)drb;
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
		"txq(%d)0x%p: drb message(%d): 0x%x, 0x%x\n",
		q_num, drb, cur_idx, temp[0], temp[1]);
#endif

}

static void set_drb_payload(unsigned char q_num, unsigned short cur_idx,
	unsigned long long data_addr, unsigned int pkt_size, char last_one)
{
	struct dpmaif_drb_pd *drb =
		((struct dpmaif_drb_pd *)dpmaif_ctrl->txq[q_num].drb_base +
		cur_idx);
#ifdef DPMAIF_DEBUG_LOG
	unsigned int *temp = NULL;
#endif

	drb->dtyp = DES_DTYP_PD;
	if (last_one)
		drb->c_bit = 0;
	else
		drb->c_bit = 1;
	drb->data_len = pkt_size;
	drb->p_data_addr = data_addr&0xFFFFFFFF;
	drb->data_addr_ext = (data_addr>>32)&0xFF;
#ifdef DPMAIF_DEBUG_LOG
	temp = (unsigned int *)drb;
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
		"txq(%d)0x%p: drb payload(%d): 0x%x, 0x%x\n",
		q_num, drb, cur_idx, temp[0], temp[1]);
#endif

}

static void record_drb_skb(unsigned char q_num, unsigned short cur_idx,
	struct sk_buff *skb, unsigned short is_msg, unsigned short is_frag,
	unsigned short is_last_one, dma_addr_t phy_addr, unsigned int data_len)
{
	struct dpmaif_drb_skb *drb_skb =
		((struct dpmaif_drb_skb *)dpmaif_ctrl->txq[q_num].drb_skb_base +
		cur_idx);
#ifdef DPMAIF_DEBUG_LOG
	unsigned int *temp;
#endif

	drb_skb->skb = skb;
	drb_skb->phy_addr = phy_addr;
	drb_skb->data_len = data_len;
	drb_skb->drb_idx = cur_idx;
	drb_skb->is_msg = is_msg;
	drb_skb->is_frag = is_frag;
	drb_skb->is_last_one = is_last_one;
#ifdef DPMAIF_DEBUG_LOG
	temp = (unsigned int *)drb_skb;
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
		"txq(%d)0x%p: drb skb(%d): 0x%x, 0x%x, 0x%x, 0x%x\n",
		q_num, drb_skb, cur_idx, temp[0], temp[1], temp[2], temp[3]);
#endif
}

static void tx_force_md_assert(char buf[])
{
	if (atomic_inc_return(&s_tx_busy_assert_on) <= 1) {
		CCCI_NORMAL_LOG(dpmaif_ctrl->md_id, TAG,
				"%s force assert\n", buf);
		ccci_md_force_assert(dpmaif_ctrl->md_id,
				MD_FORCE_ASSERT_BY_AP_Q0_BLOCKED, "TX", 3);
	}
}

static inline int cs_type(struct sk_buff *skb)
{
	u32 packet_type;
	struct iphdr *iph = (struct iphdr *)skb->data;

	packet_type = skb->data[0] & 0xF0;
	if (packet_type == IPV6_VERSION) {
		if (skb->ip_summed == CHECKSUM_NONE ||
			skb->ip_summed == CHECKSUM_UNNECESSARY ||
			skb->ip_summed == CHECKSUM_COMPLETE)
			return 0;
		else if (skb->ip_summed == CHECKSUM_PARTIAL)
			return 2;

		CCCI_ERROR_LOG(-1, TAG,
			"invalid ip_summed :%u; payload_len: %u\n",
			skb->ip_summed,
			((struct ipv6hdr *)skb->data)->payload_len);

		return 0;
	} else if (packet_type == IPV4_VERSION) {
		if (skb->ip_summed == CHECKSUM_NONE ||
			skb->ip_summed == CHECKSUM_UNNECESSARY ||
			skb->ip_summed == CHECKSUM_COMPLETE)
			return 0;
		else if (skb->ip_summed == CHECKSUM_PARTIAL)
			return 1;

		CCCI_ERROR_LOG(-1, TAG,
			"invalid checksum flags ipid: 0x%x\n",
			ntohs(iph->id));

		return 0;
	}

	CCCI_ERROR_LOG(-1, TAG,
		"invalid packet type:%u\n", packet_type);
	return 0;
}

static int dpmaif_tx_send_skb(unsigned char hif_id, int qno,
	struct sk_buff *skb, int skb_from_pool, int blocking)
{
	struct hif_dpmaif_ctrl *hif_ctrl = dpmaif_ctrl;
	struct dpmaif_tx_queue *txq = NULL;
	struct skb_shared_info *info = NULL;
	void *data_addr = NULL;
	unsigned int data_len;
	int ret = 0;
	unsigned short cur_idx, is_frag, is_last_one;
	struct ccci_header ccci_h;
	/* struct iphdr *iph = NULL;  add at port net layer? */
	unsigned int remain_cnt, wt_cnt = 0, send_cnt = 0, payload_cnt = 0;
	dma_addr_t phy_addr;
	unsigned long flags;
	unsigned short prio_count = 0;
	int total_size = 0;
	short cs_ipv4 = 0, cs_l4 = 0;

	/* 1. parameters check*/
	if (!skb)
		return 0;

	if (dpmaif_wait_resume_done() < 0)
		return -EBUSY;

	if (skb->mark & UIDMASK)
		prio_count = 0x1000;

	if (qno >= DPMAIF_TXQ_NUM) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"txq(%d) > %d\n", qno, DPMAIF_TXQ_NUM);
		ret = -CCCI_ERR_INVALID_QUEUE_INDEX;
		return ret;
	}
	txq = &hif_ctrl->txq[qno];

	if (atomic_read(&s_tx_busy_assert_on)) {
		if (likely(ccci_md_get_cap_by_id(hif_ctrl->md_id)
				&MODEM_CAP_TXBUSY_STOP))
			dpmaif_queue_broadcast_state(hif_ctrl, TX_FULL, OUT,
					txq->index);
		return HW_REG_CHK_FAIL;
	}
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
	"send_skb(%d): drb: %d, w(%d), r(%d), rel(%d)\n", qno,
		txq->drb_size_cnt, txq->drb_wr_idx,
		txq->drb_rd_idx, txq->drb_rel_rd_idx);
#endif

	atomic_set(&txq->tx_processing, 1);
	smp_mb(); /* for cpu exec. */
	if (txq->que_started != true) {
		ret = -CCCI_ERR_HIF_NOT_POWER_ON;
		atomic_set(&txq->tx_processing, 0);
		return ret;
	}

	if (dpmaif_ctrl->dpmaif_state != HIFDPMAIF_STATE_PWRON) {
		ret = -CCCI_ERR_HIF_NOT_POWER_ON;
		goto __EXIT_FUN;
	}

	info = skb_shinfo(skb);

	if (info->frag_list)
		CCCI_NOTICE_LOG(dpmaif_ctrl->md_id, TAG,
			"attention:q%d skb frag_list not supported!\n",
			qno);

	payload_cnt = info->nr_frags + 1;
	/* nr_frags: frag cnt, 1: skb->data, 1: msg drb */
	send_cnt = payload_cnt + 1;
retry:
	if (txq->que_started != true) {
		ret = -CCCI_ERR_HIF_NOT_POWER_ON;
		goto __EXIT_FUN;
	}
	/* 2. buffer check */
	remain_cnt = ringbuf_writeable(txq->drb_size_cnt,
			txq->drb_rel_rd_idx, txq->drb_wr_idx);

	if (remain_cnt < send_cnt) {
		/* buffer check: full */
		if (likely(ccci_md_get_cap_by_id(hif_ctrl->md_id)
				&MODEM_CAP_TXBUSY_STOP))
			dpmaif_queue_broadcast_state(hif_ctrl, TX_FULL, OUT,
					txq->index);
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
		txq->busy_count++;
#endif
		if (blocking) {
			/* infact, dpmaif for net, so no used here. */
			ret = wait_event_interruptible_exclusive(txq->req_wq,
				(atomic_read(&txq->tx_budget) >= send_cnt ||
				dpmaif_ctrl->dpmaif_state !=
				HIFDPMAIF_STATE_PWRON));
			if (ret == -ERESTARTSYS) {
				ret = -EINTR;
				goto __EXIT_FUN;
			} else if (dpmaif_ctrl->dpmaif_state !=
				HIFDPMAIF_STATE_PWRON) {
				ret = -CCCI_ERR_HIF_NOT_POWER_ON;
				goto __EXIT_FUN;
			}
			goto retry;
		} else {
			ret = -EBUSY;
			goto __EXIT_FUN;
		}

	}
	spin_lock_irqsave(&txq->tx_lock, flags);
	remain_cnt = ringbuf_writeable(txq->drb_size_cnt,
			txq->drb_rel_rd_idx, txq->drb_wr_idx);
	cur_idx = txq->drb_wr_idx;
	if (remain_cnt >= send_cnt) {
		txq->drb_wr_idx += send_cnt;
		if (txq->drb_wr_idx >= txq->drb_size_cnt)
			txq->drb_wr_idx -= txq->drb_size_cnt;
	} else {
		spin_unlock_irqrestore(&txq->tx_lock, flags);
		goto retry;
	}
	ccci_h = *(struct ccci_header *)skb->data;
	skb_pull(skb, sizeof(struct ccci_header));
	/* 3 send data. */
	/* 3.1 a msg drb first, then payload drb. */
	if (cs_type(skb) == 1) {
		cs_ipv4 = 1;
		cs_l4 = 1;
	} else if (cs_type(skb) == 2)
		cs_l4 = 1;
	set_drb_msg(txq->index, cur_idx, skb->len, prio_count,
			ccci_h.data[0], skb->protocol, cs_ipv4, cs_l4);
	record_drb_skb(txq->index, cur_idx, skb, 1, 0, 0, 0, 0);
	/* for debug */
	/*
	 * ccci_h = *(struct ccci_header *)skb->data;
	 * iph = (struct iphdr *)skb->data;
	 * ccci_h.reserved = iph->id;
	 */
	ccci_md_add_log_history(&dpmaif_ctrl->traffic_info, OUT,
			(int)txq->index, &ccci_h, 0);
	/* get next index. */
	cur_idx = ringbuf_get_next_idx(txq->drb_size_cnt, cur_idx, 1);

	/* 3.2 payload drb: skb->data + frag_list */
	for (wt_cnt = 0; wt_cnt < payload_cnt; wt_cnt++) {
		/* get data_addr && data_len */
		if (wt_cnt == 0) {
			data_len = skb_headlen(skb);
			data_addr = skb->data;
			is_frag = 0;
		} else {
			skb_frag_t *frag = info->frags + wt_cnt - 1;

			data_len = skb_frag_size(frag);
			data_addr = skb_frag_address(frag);
			is_frag = 1;
		}
		if (wt_cnt == payload_cnt - 1) {
			is_last_one = 1;
		} else {
			/* set 0~(n-1) drb, mabye none */
			is_last_one = 0;
		}
		/* tx mapping */
		phy_addr = dma_map_single(
			ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
				data_addr, data_len, DMA_TO_DEVICE);
		if (dma_mapping_error(
			ccci_md_get_dev_by_id(dpmaif_ctrl->md_id), phy_addr)) {
			CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
				"error dma mapping\n");
			ret = -1;
			skb_push(skb, sizeof(struct ccci_header));
			spin_unlock_irqrestore(&txq->tx_lock, flags);
			goto __EXIT_FUN;
		}

		set_drb_payload(txq->index, cur_idx, phy_addr, data_len,
			is_last_one);
		record_drb_skb(txq->index, cur_idx, skb, 0, is_frag,
			is_last_one, phy_addr, data_len);
		cur_idx = ringbuf_get_next_idx(txq->drb_size_cnt, cur_idx, 1);
		total_size += data_len;
	}

	/* debug: tx on ccci_channel && HW Q */
	ccci_channel_update_packet_counter(
		dpmaif_ctrl->traffic_info.logic_ch_pkt_pre_cnt, &ccci_h);
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	dpmaif_ctrl->tx_pre_traffic_monitor[txq->index]++;
#endif
	atomic_sub(send_cnt, &txq->tx_budget);
	/* 3.3 submit drb descriptor*/
	wmb();
	if (atomic_read(&txq->tx_resume_done) &&
		atomic_read(&txq->tx_resume_tx)) {
		CCCI_NOTICE_LOG(0, TAG,
			"tx%d_resume_tx: 0x%x, 0x%x, 0x%x\n",
			txq->index,
			drv3_dpmaif_ul_get_rwidx(0),
			drv3_dpmaif_ul_get_rwidx(1),
			drv3_dpmaif_ul_get_rwidx(3));
		atomic_set(&txq->tx_resume_tx, 0);
	}
	ret = drv3_dpmaif_ul_add_wcnt(txq->index,
		send_cnt * DPMAIF_UL_DRB_ENTRY_WORD);
	if (ret == HW_REG_CHK_FAIL) {
		tx_force_md_assert("HW_REG_CHK_FAIL");
		ret = 0;
	}
	if (ret == 0)
		mtk_ccci_add_ul_pkt_bytes(txq->index, total_size);
	spin_unlock_irqrestore(&txq->tx_lock, flags);
__EXIT_FUN:
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG,
		"send_skb(%d) end: drb: %xd w(%d), r(%d), rel(%d)\n", qno,
		txq->drb_size_cnt, txq->drb_wr_idx,
		txq->drb_rd_idx, txq->drb_rel_rd_idx);
#endif
	atomic_set(&txq->tx_processing, 0);
	return ret;
}

/* =======================================================
 *
 * Descriptions: ISR part start
 *
 * ========================================================
 */

static void dpmaif_enable_irq(struct hif_dpmaif_ctrl *hif_ctrl)
{
	if (atomic_cmpxchg(&hif_ctrl->dpmaif_irq_enabled, 0, 1) == 0)
		enable_irq(hif_ctrl->dpmaif_irq_id);
}

static void dpmaif_disable_irq(struct hif_dpmaif_ctrl *hif_ctrl)
{
	if (atomic_cmpxchg(&hif_ctrl->dpmaif_irq_enabled, 1, 0) == 1)
		disable_irq(hif_ctrl->dpmaif_irq_id);
}

static void dpmaif_irq_rx_lenerr_handler(unsigned int rx_int_isr)
{
	/*SKB buffer size error*/
	if (rx_int_isr & DPMAIF_DL_INT_SKB_LEN_ERR(0)) {
		CCCI_NOTICE_LOG(dpmaif_ctrl->md_id, TAG,
			"dpmaif: dl skb error L2\n");
	}

	/*Rx data length more than error*/
	if (rx_int_isr & DPMAIF_DL_INT_MTU_ERR_MSK) {
		CCCI_NOTICE_LOG(dpmaif_ctrl->md_id, TAG,
			"dpmaif: dl mtu error L2\n");
	}

#ifdef DPMAIF_DEBUG_LOG
	if (rx_int_isr & (DPMAIF_DL_INT_SKB_LEN_ERR(0) |
		DPMAIF_DL_INT_MTU_ERR_MSK))
		dpmaif_ctrl->traffic_info.rx_other_isr_cnt[0]++;
	if (rx_int_isr & (DPMAIF_DL_INT_PITCNT_LEN_ERR(0) |
		DPMAIF_DL_INT_BATCNT_LEN_ERR(0)))
		dpmaif_ctrl->traffic_info.rx_full_cnt++;
#endif

}

static void dpmaif_irq_rx_done(unsigned int rx_done_isr)
{
	/* debug information colloect */
	dpmaif_ctrl->traffic_info.latest_q_rx_isr_time[0] = local_clock();

	/* disable RX_DONE  interrupt */
	drv3_dpmaif_mask_dl_interrupt(0);
	/*always start work due to no napi*/
	/*for (i = 0; i < DPMAIF_HW_MAX_DLQ_NUM; i++)*/
	tasklet_hi_schedule(&dpmaif_ctrl->rxq[0].dpmaif_rxq0_task);
}

static void dpmaif_irq_tx_done(unsigned int tx_done_isr)
{
	int i;
	unsigned int intr_ul_que_done;

	for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
		intr_ul_que_done =
			tx_done_isr & (1 << (i + UL_INT_DONE_OFFSET));
		if (intr_ul_que_done) {
			if (atomic_read(&dpmaif_ctrl->txq[i].tx_resume_done)) {
				atomic_set(&dpmaif_ctrl->txq[i].tx_resume_done,
					0);
				CCCI_NOTICE_LOG(0, TAG,
					"clear txq%d_resume_done: 0x%x, 0x%x, 0x%x\n",
					i, drv3_dpmaif_ul_get_rwidx(0),
					drv3_dpmaif_ul_get_rwidx(1),
					drv3_dpmaif_ul_get_rwidx(3));
			}
			drv3_dpmaif_mask_ul_que_interrupt(i);

			hrtimer_start(&dpmaif_ctrl->txq[i].tx_done_timer,
					ktime_set(0, 500000), HRTIMER_MODE_REL);

#ifdef DPMAIF_DEBUG_LOG
			dpmaif_ctrl->traffic_info.tx_done_isr_cnt[i]++;
#endif
		}
	}
}

static void dpmaif_irq_cb(struct hif_dpmaif_ctrl *hif_ctrl)
{
	unsigned int L2RISAR0, L2TISAR0;
	unsigned int L2RIMR0, L2TIMR0;
#ifdef DPMAIF_DEBUG_LOG
	unsigned long long ts = 0, isr_rem_nsec;
#endif

	/* RX interrupt */
	L2RISAR0 = drv3_dpmaif_get_dl_isr_event();
	L2RIMR0 = drv3_dpmaif_get_dl_interrupt_mask();
	/* TX interrupt */
	L2TISAR0 = drv3_dpmaif_get_ul_isr_event();
	L2TIMR0 = drv3_dpmaif_ul_get_ul_interrupt_mask();

	/* clear IP busy register wake up cpu case */
	drv3_dpmaif_clear_ip_busy();

	if (atomic_read(&hif_ctrl->wakeup_src) == 1)
		CCCI_NOTICE_LOG(hif_ctrl->md_id, TAG,
			"wake up by MD%d HIF L2(%x/%x)(%x/%x)!\n",
			hif_ctrl->md_id, L2TISAR0, L2RISAR0, L2TIMR0, L2RIMR0);
	else
		CCCI_DEBUG_LOG(hif_ctrl->md_id, TAG,
			"DPMAIF IRQ L2(%x/%x)(%x/%x)!\n",
			L2TISAR0, L2RISAR0, L2TIMR0, L2RIMR0);

	/* TX interrupt */
	if (L2TISAR0) {
		L2TISAR0 &= ~(L2TIMR0);
		DPMA_WRITE_PD_MISC(DPMAIF_PD_AP_UL_L2TISAR0, L2TISAR0);
/* 0x1100: not error, just information
 * #define DPMAIF_UL_INT_MD_NOTREADY_MSK (0x0F << UL_INT_MD_NOTRDY_OFFSET)
 * bit 8: resource not ready: md buffer(meta table)
 * not ready[virtual ring buffer][URB], or buffer full.
 * #define DPMAIF_UL_INT_MD_PWR_NOTREADY_MSK (0x0F << UL_INT_PWR_NOTRDY_OFFSET)
 * bit 12: MD_PWR: read md, L2 subsys auto power on/off, (LHIF) auto power off,
 * receive data auto power on, and send inerrupt.
 */
		if (L2TISAR0 &
			(DPMAIF_UL_INT_MD_NOTREADY_MSK |
			DPMAIF_UL_INT_MD_PWR_NOTREADY_MSK))
			CCCI_REPEAT_LOG(hif_ctrl->md_id, TAG,
					"dpmaif: ul info: L2(%x)\n", L2TISAR0);
		else if (L2TISAR0 & AP_UL_L2INTR_ERR_En_Msk)
			CCCI_NOTICE_LOG(hif_ctrl->md_id, TAG,
					"dpmaif: ul error L2(%x)\n", L2TISAR0);

		/* tx done */
		if (L2TISAR0 & DPMAIF_UL_INT_QDONE_MSK)
			dpmaif_irq_tx_done(L2TISAR0 & DPMAIF_UL_INT_QDONE_MSK);
	}
	/* RX interrupt */
	if (L2RISAR0) {
		L2RISAR0 &= ~(L2RIMR0);

		if (L2RISAR0 & AP_DL_L2INTR_ERR_En_Msk)
			dpmaif_irq_rx_lenerr_handler(
				L2RISAR0 & AP_DL_L2INTR_ERR_En_Msk);
		/* ACK interrupt after lenerr_handler*/
		/* ACK RX  interrupt */
		DPMA_WRITE_PD_MISC(DPMAIF_PD_AP_DL_L2TISAR0, L2RISAR0);

		if (L2RISAR0 & DPMAIF_DL_INT_QDONE_MSK) {
			dpmaif_irq_rx_done(
				L2RISAR0 & DPMAIF_DL_INT_QDONE_MSK);
#ifdef DPMAIF_DEBUG_LOG
			hif_ctrl->traffic_info.rx_done_isr_cnt[0]++;
#endif
		}
	}
#ifdef DPMAIF_DEBUG_LOG
	hif_ctrl->traffic_info.isr_cnt++;

	ts = hif_ctrl->traffic_info.latest_isr_time;
	isr_rem_nsec = do_div(ts, NSEC_PER_SEC);

	if (hif_ctrl->traffic_info.isr_time_bak != ts) {
		CCCI_NORMAL_LOG(hif_ctrl->md_id, TAG,
			"DPMAIF IRQ cnt(%llu/%llu.%llu: %llu)(%llu/%llu/%llu/%llu)(%llu/%llu/%llu/%llu ~ %llu/%llu/%llu/%llu)!\n",
			hif_ctrl->traffic_info.isr_time_bak,
			ts, isr_rem_nsec/1000,
			hif_ctrl->traffic_info.isr_cnt,

			hif_ctrl->traffic_info.rx_done_isr_cnt[0],
			hif_ctrl->traffic_info.rx_other_isr_cnt[0],
			hif_ctrl->traffic_info.rx_full_cnt,
			hif_ctrl->traffic_info.rx_tasket_cnt,

			hif_ctrl->traffic_info.tx_done_isr_cnt[0],
			hif_ctrl->traffic_info.tx_done_isr_cnt[1],
			hif_ctrl->traffic_info.tx_done_isr_cnt[2],
			hif_ctrl->traffic_info.tx_done_isr_cnt[3],
			hif_ctrl->traffic_info.tx_other_isr_cnt[0],
			hif_ctrl->traffic_info.tx_other_isr_cnt[1],
			hif_ctrl->traffic_info.tx_other_isr_cnt[2],
			hif_ctrl->traffic_info.tx_other_isr_cnt[3]);
		hif_ctrl->traffic_info.isr_time_bak = ts;

		hif_ctrl->traffic_info.isr_cnt = 0;

		hif_ctrl->traffic_info.rx_done_isr_cnt[0] = 0;
		hif_ctrl->traffic_info.rx_other_isr_cnt[0] = 0;
		hif_ctrl->traffic_info.rx_tasket_cnt = 0;

		hif_ctrl->traffic_info.tx_done_isr_cnt[0] = 0;
		hif_ctrl->traffic_info.tx_done_isr_cnt[1] = 0;
		hif_ctrl->traffic_info.tx_done_isr_cnt[2] = 0;
		hif_ctrl->traffic_info.tx_done_isr_cnt[3] = 0;
		hif_ctrl->traffic_info.tx_other_isr_cnt[0] = 0;
		hif_ctrl->traffic_info.tx_other_isr_cnt[1] = 0;
		hif_ctrl->traffic_info.tx_other_isr_cnt[2] = 0;
		hif_ctrl->traffic_info.tx_other_isr_cnt[3] = 0;
	}

#endif

}

static irqreturn_t dpmaif_isr(int irq, void *data)
{
	struct hif_dpmaif_ctrl *hif_ctrl = (struct hif_dpmaif_ctrl *)data;

#ifdef DPMAIF_NOT_ACCESS_HW
	CCCI_HISTORY_LOG(hif_ctrl->md_id, TAG, "DPMAIF IRQ!\n");
	if (hif_ctrl->hif_id)
		return IRQ_HANDLED;
#elif defined(DPMAIF_DEBUG_LOG)
	CCCI_DEBUG_LOG(hif_ctrl->md_id, TAG, "DPMAIF IRQ!\n");
#endif
	hif_ctrl->traffic_info.latest_isr_time = local_clock();
	dpmaif_irq_cb(hif_ctrl);
	return IRQ_HANDLED;
}

static int dpmaif_rx_buf_init(struct dpmaif_rx_queue *rxq)
{
	int ret = 0;

	/* PIT buffer init */
	rxq->pit_size_cnt = DPMAIF_DL_PIT_ENTRY_SIZE;
	/* alloc buffer for HW && AP SW */
#ifndef PIT_USING_CACHE_MEM
#if (DPMAIF_DL_PIT_SIZE > PAGE_SIZE)
	rxq->pit_base = dma_alloc_coherent(
		ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
		(rxq->pit_size_cnt * sizeof(struct dpmaifq_normal_pit)),
		&rxq->pit_phy_addr, GFP_KERNEL);
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "pit dma_alloc_coherent\n");
#endif
#else
	rxq->pit_base = dma_pool_alloc(dpmaif_ctrl->rx_pit_dmapool,
		GFP_KERNEL, &rxq->pit_phy_addr);
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "pit dma_pool_alloc\n");
#endif
#endif
	if (rxq->pit_base == NULL) {
		CCCI_ERROR_LOG(-1, TAG, "pit request fail\n");
		return LOW_MEMORY_PIT;
	}
#else
	CCCI_BOOTUP_LOG(-1, TAG, "Using cacheable PIT memory\r\n");
	rxq->pit_base = kmalloc((rxq->pit_size_cnt
			* sizeof(struct dpmaifq_normal_pit)), GFP_KERNEL);
	if (!rxq->pit_base) {
		CCCI_ERROR_LOG(-1, TAG, "alloc PIT memory fail\r\n");
		return LOW_MEMORY_PIT;
	}
	rxq->pit_phy_addr = virt_to_phys(rxq->pit_base);
#endif
	memset(rxq->pit_base, 0, DPMAIF_DL_PIT_SIZE);
	/* dpmaif_pit_init(rxq->pit_base, rxq->pit_size_cnt); */

#ifdef DPMAIF_DEBUG_LOG
	dpmaif_dump_rxq_remain(dpmaif_ctrl, rxq->index, 0);
#endif
	return ret;
}

/* =======================================================
 *
 * Descriptions: State part start(1/3): init(RX) -- 1.1.1 rx hw init
 *
 * ========================================================
 */

static void dpmaif_rx_hw_init(struct dpmaif_rx_queue *rxq)
{
	unsigned int pkt_alignment;

	if (rxq->que_started == true) {
		/* 1. BAT buffer parameters setting */
		drv3_dpmaif_dl_set_remain_minsz(rxq->index,
			DPMAIF_HW_BAT_REMAIN);

		drv3_dpmaif_dl_set_bid_maxcnt(rxq->index, DPMAIF_HW_PKT_BIDCNT);
		pkt_alignment = DPMAIF_HW_PKT_ALIGN;
		if (pkt_alignment == 64) {
			drv3_dpmaif_dl_set_pkt_align(rxq->index, true,
				DPMAIF_PKT_ALIGN64_MODE);
		} else if (pkt_alignment == 128) {
			drv3_dpmaif_dl_set_pkt_align(rxq->index, true,
				DPMAIF_PKT_ALIGN128_MODE);
		} else {
			drv3_dpmaif_dl_set_pkt_align(rxq->index, false, 0);
		}
		drv3_dpmaif_dl_set_mtu(DPMAIF_HW_MTU_SIZE);
		/* 2. bat && pit threadhold, addr, len, enable */
		drv3_dpmaif_dl_set_pit_chknum(rxq->index,
			DPMAIF_HW_CHK_PIT_NUM);

		drv3_dpmaif_dl_set_chk_rbnum(rxq->index,
			DPMAIF_HW_CHK_RB_PIT_NUM);
		drv3_dpmaif_dl_set_performance();

		/*
		 * drv3_dpmaif_dl_set_ao_frag_check_thres(que_cnt,
		 *	p_dl_hw->chk_frg_num);
		 */

		drv3_dpmaif_dl_set_pit_base_addr(rxq->index,
					rxq->pit_phy_addr);

		drv3_dpmaif_dl_set_pit_size(rxq->index, rxq->pit_size_cnt);

		/*Enable PIT*/
		drv3_dpmaif_dl_pit_en(rxq->index, true);

		drv3_dpmaif_dl_pit_init_done(rxq->index);

#ifdef HW_CHECK_SUM_ENABLE
		drv3_dpmaif_dl_set_ao_chksum_en(rxq->index, true);
#endif

	}
}
/* =======================================================
 *
 * Descriptions: State part start(1/3): init(RX) -- 1.1.0 rx init
 *
 * =======================================================
 */
static int dpmaif_rxq_init(struct dpmaif_rx_queue *queue)
{
	int ret = -1;

	if (dpmaif_ctrl->support_lro)
		dpmaif_rxq_lro_info_init(queue);

	ret = dpmaif_rx_buf_init(queue);
	if (ret) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"rx buffer init fail %d\n", ret);
		return ret;
	}
	/* rx tasklet */
	tasklet_init(&queue->dpmaif_rxq0_task, dpmaif_rxq0_tasklet,
			(unsigned long)dpmaif_ctrl);
	/* rx work */
	INIT_WORK(&queue->dpmaif_rxq0_work, dpmaif_rxq0_work);
	queue->worker = alloc_workqueue("ccci_dpmaif_rx0_worker",
				WQ_UNBOUND | WQ_MEM_RECLAIM, 1);

	/* rx push */
	init_waitqueue_head(&queue->rx_wq);
	ccci_skb_queue_init(&queue->skb_list, DPMAIF_BUF_PKT_SIZE,
				SKB_RX_LIST_MAX_LEN, 0);

	queue->rx_thread = kthread_run(dpmaif_net_rx_push_thread,
				queue, "dpmaif_rx_push");
	spin_lock_init(&queue->rx_lock);

	if (dpmaif_ctrl->enable_pit_debug >= 0)
		dpmaif_debug_init_data(&queue->dbg_data, DEBUG_TYPE_RX_DONE,
			DEBUG_VERION_v3, queue->index);

	return 0;
}

/* =======================================================
 *
 * Descriptions: State part start(1/3): init(TX) -- 1.2.2 tx sw init
 *
 * ========================================================
 */
static int dpmaif_tx_buf_init(struct dpmaif_tx_queue *txq)
{
	int ret = 0;

	/* DRB buffer init */
	txq->drb_size_cnt = DPMAIF_UL_DRB_ENTRY_SIZE;
	/* alloc buffer for HW && AP SW */
#if (DPMAIF_UL_DRB_SIZE > PAGE_SIZE)
	txq->drb_base = dma_alloc_coherent(
		ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
		(txq->drb_size_cnt * sizeof(struct dpmaif_drb_pd)),
		&txq->drb_phy_addr, GFP_KERNEL);
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "drb dma_alloc_coherent\n");
#endif
#else
	txq->drb_base = dma_pool_alloc(dpmaif_ctrl->tx_drb_dmapool,
		GFP_KERNEL, &txq->drb_phy_addr);
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "drb dma_pool_alloc\n");
#endif
#endif

	if (!txq->drb_base) {
		CCCI_ERROR_LOG(-1, TAG, "drb request fail\n");
		return LOW_MEMORY_DRB;
	}
	memset(txq->drb_base, 0, DPMAIF_UL_DRB_SIZE);
	/* alloc buffer for AP SW */
	txq->drb_skb_base =
		kzalloc((txq->drb_size_cnt * sizeof(struct dpmaif_drb_skb)),
				GFP_KERNEL);
	if (!txq->drb_skb_base) {
		CCCI_ERROR_LOG(-1, TAG, "drb skb buffer request fail\n");
		return LOW_MEMORY_DRB;
	}
	return ret;
}

/* =======================================================
 *
 * Descriptions: State part start(1/3): init(TX) -- 1.2.1 tx hw init
 *
 * ========================================================
 */
static void dpmaif_tx_hw_init(struct dpmaif_tx_queue *txq)
{
	struct dpmaif_tx_queue *p_ul_que = txq;
	unsigned long long base_addr;

	if (p_ul_que->que_started == true) {
		/* 1. BAT buffer parameters setting */

		drv3_dpmaif_ul_update_drb_size(p_ul_que->index,
		(p_ul_que->drb_size_cnt * sizeof(struct dpmaif_drb_pd)));

		base_addr = (unsigned long long)p_ul_que->drb_phy_addr;

#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG,
		"drb(%d) base_addr: virt = 0x%p, phy = 0x%x\n",
		txq->index, p_ul_que->drb_base, (unsigned int)base_addr);
#endif

		drv3_dpmaif_ul_update_drb_base_addr(p_ul_que->index,
			(base_addr&0xFFFFFFFF), ((base_addr>>32)&0xFFFFFFFF));
		drv3_dpmaif_ul_rdy_en(p_ul_que->index, true);

		drv3_dpmaif_ul_arb_en(p_ul_que->index, true);
	} else {
		drv3_dpmaif_ul_arb_en(p_ul_que->index, false);
	}
}

/* =======================================================
 *
 * Descriptions: State part start(1/3): init(TX) -- 1.2.0 tx init
 *
 * ========================================================
 */

static int dpmaif_txq_init(struct dpmaif_tx_queue *txq)
{
	int ret = -1;

	init_waitqueue_head(&txq->req_wq);
	atomic_set(&txq->tx_budget, DPMAIF_UL_DRB_ENTRY_SIZE);
	ret = dpmaif_tx_buf_init(txq);
	if (ret) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"tx buffer init fail %d\n", ret);
		return ret;
	}

	init_waitqueue_head(&txq->tx_done_wait);
	atomic_set(&txq->txq_done, 0);
	hrtimer_init(&txq->tx_done_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	txq->tx_done_timer.function = tx_done_action;
	txq->tx_done_thread = NULL;

	spin_lock_init(&txq->tx_lock);
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	txq->busy_count = 0;
#endif
#ifdef DPMAIF_DEBUG_LOG
	dpmaif_dump_txq_remain(dpmaif_ctrl, txq->index, 0);
#endif
	return 0;
}

/* =======================================================
 *
 * Descriptions: State part start(1/3): init(ISR) -- 1.3
 *
 * ========================================================
 */

/* we put initializations which takes too much time here: SW init only */
static int dpmaif_late_init(unsigned char hif_id)
{
	struct dpmaif_rx_queue *rx_q = NULL;
	struct dpmaif_tx_queue *tx_q = NULL;
	int ret, i;
	unsigned int reg_val;

#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_TAG_LOG(-1, TAG, "dpmaif:%s\n", __func__);
#else
	CCCI_DEBUG_LOG(-1, TAG, "dpmaif:%s\n", __func__);
#endif
	/* set sw control data flow cb: isr/tx/rx/etc. */
	/* request IRQ */
	ret = request_irq(dpmaif_ctrl->dpmaif_irq_id, dpmaif_isr,
		dpmaif_ctrl->dpmaif_irq_flags, "DPMAIF_AP",
		dpmaif_ctrl);
	if (ret) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"request DPMAIF IRQ(%d) error %d\n",
			dpmaif_ctrl->dpmaif_irq_id, ret);
		return ret;
	}
	ret = irq_set_irq_wake(dpmaif_ctrl->dpmaif_irq_id, 1);
	if (ret)
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"irq_set_irq_wake dpmaif irq(%d) error %d\n",
			dpmaif_ctrl->dpmaif_irq_id, ret);
	atomic_set(&dpmaif_ctrl->dpmaif_irq_enabled, 1); /* init */
	dpmaif_disable_irq(dpmaif_ctrl);

	/* rx rx */
#if !(DPMAIF_DL_PIT_SIZE > PAGE_SIZE)
	dpmaif_ctrl->rx_pit_dmapool = dma_pool_create("dpmaif_pit_req_DMA",
		ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
		(DPMAIF_DL_PIT_ENTRY_SIZE*sizeof(struct dpmaifq_normal_pit)),
		64, 0);
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG, "pit dma pool\n");
#endif
#endif
	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		rx_q = &dpmaif_ctrl->rxq[i];
		rx_q->index = i;
		ret = dpmaif_rxq_init(rx_q);
		if (ret < 0)
			return ret;
		rx_q->skb_idx = -1;
	}
	ret = ccci_dpmaif_bat_sw_init_v3();
	if (ret)
		return ret;
	/* tx tx */
#if !(DPMAIF_UL_DRB_SIZE > PAGE_SIZE)
	dpmaif_ctrl->tx_drb_dmapool = dma_pool_create("dpmaif_drb_req_DMA",
		ccci_md_get_dev_by_id(dpmaif_ctrl->md_id),
		(DPMAIF_UL_DRB_ENTRY_SIZE*sizeof(struct dpmaif_drb_pd)), 64, 0);
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(dpmaif_ctrl->md_id, TAG, "drb dma pool\n");
#endif
#endif
	for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
		tx_q = &dpmaif_ctrl->txq[i];
		tx_q->index = i;
		ret = dpmaif_txq_init(tx_q);
		if (ret < 0)
			return ret;

	}

	/* wakeup source init */
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
		INFRA_DPMAIF_CTRL_REG, &reg_val);
	if (ret) {
		CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
			"%s:read INFRA_DPMAIF_CTRL_REG fail,ret=%d\n",
			__func__, ret);
		return ret;
	}
	reg_val |= DPMAIF_IP_BUSY_MASK;
	reg_val &= ~(1 << 13); /* MD to AP wakeup event */
	regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
		INFRA_DPMAIF_CTRL_REG, reg_val);

	if (DPMAIF_RXQ_NUM > 0)
		mtk_ccci_spd_qos_set_task(
			dpmaif_ctrl->rxq[0].rx_thread,
			dpmaif_ctrl->bat_alloc_thread,
			dpmaif_ctrl->dpmaif_irq_id);

#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_TAG_LOG(-1, TAG, "dpmaif:%s end\n", __func__);
#else
	CCCI_DEBUG_LOG(-1, TAG, "dpmaif:%s end\n", __func__);
#endif
	return 0;
}

/* =======================================================
 *
 * Descriptions: State part start(2/3): Start -- 2.
 *
 * ========================================================
 */

static int dpmaif_start(unsigned char hif_id)
{
	struct dpmaif_rx_queue *rxq = NULL;
	struct dpmaif_tx_queue *txq = NULL;
	int i, ret = 0;
	struct cpumask imask;
	unsigned int value = 0;

	if (dpmaif_ctrl->dpmaif_state == HIFDPMAIF_STATE_PWRON)
		return 0;
	else if (dpmaif_ctrl->dpmaif_state == HIFDPMAIF_STATE_MIN) {
		ret = dpmaif_late_init(hif_id);
		if (ret < 0)
			return ret;
	}
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_TAG_LOG(-1, TAG, "dpmaif:start\n");
#endif
	/* cg set */
	ccci_dpmaif_set_clk(1, g_clk_tbs);

	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	value |= (1<<15);

	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	//drv3_dpmaif_clr_axi_out_gated();

	drv3_dpmaif_common_hw_init();

	ret = drv3_dpmaif_intr_hw_init();
	if (ret < 0) {
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
		aee_kernel_warning("ccci",
			"dpmaif start fail to init hw intr\n");
#endif
		return ret;
	}
	/* rx rx */
	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		rxq = &dpmaif_ctrl->rxq[i];
		rxq->que_started = true;
		rxq->index = i;
		dpmaif_rx_hw_init(rxq);

		rxq->budget = DPMAIF_DL_BAT_ENTRY_SIZE - 1;
	}

	ccci_dpmaif_bat_hw_init_v3();
	ret = ccci_dpmaif_bat_start_v3();
	if (ret)
		return ret;

	/* tx tx */
	for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
		txq = &dpmaif_ctrl->txq[i];
		txq->que_started = true;
		dpmaif_tx_hw_init(txq);

		/* start kernel thread */
		txq->tx_done_thread =
				kthread_run(dpmaif_tx_done_kernel_thread,
				(void *)txq, "dpmaif_txq%d_done_kt",
				txq->index);
		if (IS_ERR(txq->tx_done_thread)) {
			CCCI_ERROR_LOG(dpmaif_ctrl->md_id, TAG,
				"%s tx done kthread_run fail %ld\n", __func__,
				(long)txq->tx_done_thread);
			return -1;
		}
	}

	drv3_dpmaif_hw_init_done();
	drv3_dpmaif_clear_ip_busy();

	/* debug */
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	dpmaif_clear_traffic_data(DPMAIF_HIF_ID);
	mod_timer(&dpmaif_ctrl->traffic_monitor,
			jiffies + DPMAIF_TRAFFIC_MONITOR_INTERVAL * HZ);
#endif
	dpmaif_enable_irq(dpmaif_ctrl);
	dpmaif_ctrl->dpmaif_state = HIFDPMAIF_STATE_PWRON;
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_TAG_LOG(-1, TAG, "dpmaif:start end: %d\n", ret);
#endif

	atomic_set(&s_tx_busy_assert_on, 0);

	cpumask_clear(&imask);
	cpumask_set_cpu(1, &imask);
	irq_set_affinity_hint(dpmaif_ctrl->dpmaif_irq_id, &imask);

	return 0;
}

/* =======================================================
 *
 * Descriptions: State part start(3/3): Stop -- 3.
 *
 * ========================================================
 */
static void dpmaif_stop_hw(void)
{
	struct dpmaif_rx_queue *rxq = NULL;
	struct dpmaif_tx_queue *txq = NULL;
	unsigned int que_cnt, ret;
	int count;

#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_TAG_LOG(-1, TAG, "dpmaif:stop hw\n");
#endif

	/*dpmaif_dump_register();*/

	/* ===Disable UL SW active=== */
	for (que_cnt = 0; que_cnt < DPMAIF_TXQ_NUM; que_cnt++) {
		txq = &dpmaif_ctrl->txq[que_cnt];
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "txq%d: 0x%x, 0x%x, 0x%x, (0x%x)\n", que_cnt,
	txq->drb_wr_idx, txq->drb_rd_idx, txq->drb_rel_rd_idx,
	atomic_read(&txq->tx_processing));
#endif
		txq->que_started = false;
		smp_mb(); /* for cpu exec. */
		/* just confirm sw will not update drb_wcnt reg. */
		count = 0;
		do {
			if (++count >= 1600000) {
				CCCI_ERROR_LOG(0, TAG, "pool stop Tx failed\n");
				break;
			}
		} while (atomic_read(&txq->tx_processing) != 0);
	}
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "dpmaif:stop tx proc cnt: 0x%x\n", count);
#endif

	count = 0;
	do {
		/*Disable HW arb and check idle*/
		drv3_dpmaif_ul_all_queue_en(false);
		ret = drv3_dpmaif_ul_idle_check();

		/*retry handler*/
		if ((++count) % 100000 == 0) {
			if (count >= 1600000) {
				CCCI_ERROR_LOG(0, TAG, "stop Tx failed, 0x%x\n",
					DPMA_READ_PD_UL(
						DPMAIF_PD_UL_DBG_STA2));
				break;
			}
		}
	} while (ret != 0);

	/* ===Disable DL/RX SW active=== */
	for (que_cnt = 0; que_cnt < DPMAIF_RXQ_NUM; que_cnt++) {
		rxq = &dpmaif_ctrl->rxq[que_cnt];
		rxq->que_started = false;
		smp_mb(); /* for cpu exec. */
		/* flush work */
		flush_work(&rxq->dpmaif_rxq0_work);/* tasklet no need */
		/* for BAT add_cnt register */
		count = 0;
		do {
			/*retry handler*/
			if (++count >= 1600000) {
				CCCI_ERROR_LOG(0, TAG,
					"stop Rx sw failed, 0x%x\n", count);
				CCCI_NORMAL_LOG(0, TAG,
					"dpmaif_stop_rxq: 0x%x, 0x%x, 0x%x\n",
					rxq->pit_rd_idx, rxq->pit_wr_idx,
					rxq->pit_rel_rd_idx);
				break;
			}
		} while (atomic_read(&rxq->rx_processing) != 0);
	}
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "dpmaif:stop rx proc cnt 0x%x\n", count);
#endif

	count = 0;
	do {
		/*Disable HW arb and check idle*/
		ret = drv3_dpmaif_dl_all_queue_en(false);
		if (ret < 0) {
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
			aee_kernel_warning("ccci",
				"dpmaif stop failed to enable dl queue\n");
#endif
		}
		ret = drv3_dpmaif_dl_idle_check();

		/*retry handler*/
		if (++count >= 1600000) {
			CCCI_ERROR_LOG(0, TAG, "stop Rx failed, 0x%x\n",
				DPMA_READ_PD_DL(
				DPMAIF_PD_DL_DBG_STA1));
			break;
		}
	} while (ret != 0);

	/* clear all interrupts */
}


static int dpmaif_stop_txq(struct dpmaif_tx_queue *txq)
{
	int j;

	if (!txq->drb_base)
		return -1;
	txq->que_started = false;

	wait_tx_done_thread_finish(txq);
	/* reset sw */
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "stop_txq%d: 0x%x, 0x%x, 0x%x\n",
	txq->index, txq->drb_wr_idx,
	txq->drb_rd_idx, txq->drb_rel_rd_idx);
#endif
	if (txq->drb_rd_idx != txq->drb_rel_rd_idx) {
		CCCI_NOTICE_LOG(0, TAG,
			"%s: tx_release maybe not end: rd(0x%x), rel(0x%x)\n",
			__func__, txq->drb_rd_idx, txq->drb_rel_rd_idx);
	}
	if (txq->drb_wr_idx != txq->drb_rel_rd_idx) {
		j = ringbuf_releasable(txq->drb_size_cnt,
			txq->drb_rel_rd_idx, txq->drb_wr_idx);
		dpmaif_relase_tx_buffer(txq->index, j);
	}

	memset(txq->drb_base, 0,
		(txq->drb_size_cnt * sizeof(struct dpmaif_drb_pd)));
	memset(txq->drb_skb_base, 0,
		(txq->drb_size_cnt * sizeof(struct dpmaif_drb_skb)));

	txq->drb_rd_idx = 0;
	txq->drb_wr_idx = 0;
	txq->drb_rel_rd_idx = 0;

	return 0;
}

static int dpmaif_stop_rxq(struct dpmaif_rx_queue *rxq)
{
	int j, cnt;

	if (rxq->pit_base == NULL)
		return -1;

	/* flush work */
	flush_work(&rxq->dpmaif_rxq0_work);/* tasklet no need */
	/* reset sw */
	rxq->que_started = false;
	j = 0;
	do {
		/*Disable HW arb and check idle*/
		cnt = ringbuf_readable(rxq->pit_size_cnt,
			rxq->pit_rd_idx, rxq->pit_wr_idx);
		/*retry handler*/
		if ((++j) % 100000 == 0) {
			if (j >= 1600000) {
				CCCI_ERROR_LOG(0, TAG,
					"stop Rx sw failed, 0x%x\n",
					cnt);
				CCCI_NORMAL_LOG(0, TAG,
					"%s: 0x%x, 0x%x, 0x%x\n", __func__,
					rxq->pit_rd_idx, rxq->pit_wr_idx,
					rxq->pit_rel_rd_idx);
				break;
			}
		}
	} while (cnt != 0);

	memset(rxq->pit_base, 0,
		(rxq->pit_size_cnt * sizeof(struct dpmaifq_normal_pit)));

	rxq->pit_rd_idx = 0;
	rxq->pit_wr_idx = 0;
	rxq->pit_rel_rd_idx = 0;

	return 0;
}

static int dpmaif_stop_rx_sw(unsigned char hif_id)
{
	struct dpmaif_rx_queue *rxq = NULL;
	int i;

	/* rx rx clear */
	for (i = 0; i < DPMAIF_RXQ_NUM; i++) {
		rxq = &dpmaif_ctrl->rxq[i];
		dpmaif_stop_rxq(rxq);
	}

	/* rx bat buf clear */
	ccci_dpmaif_bat_stop_v3();

	return 0;
}

static int dpmaif_stop_tx_sw(unsigned char hif_id)
{
	struct dpmaif_tx_queue *txq = NULL;
	int i;

	/*flush and release UL descriptor*/
	for (i = 0; i < DPMAIF_TXQ_NUM; i++) {
		txq = &dpmaif_ctrl->txq[i];
		dpmaif_stop_txq(txq);
		atomic_set(&txq->tx_resume_done, 0);
	}
	return 0;
}

static void dpmaif_hw_reset(void)
{
	unsigned char md_id = 0;
	unsigned int value;
	int ret;

	//drv3_dpmaif_set_axi_out_gated();
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	value &= ~(1<<15);

	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	udelay(500);

	/* DPMAIF HW reset */
	CCCI_DEBUG_LOG(md_id, TAG, "%s:rst dpmaif\n", __func__);
	/* reset dpmaif hw: PD Domain */
	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST0_REG_PD, DPMAIF_PD_RST_MASK);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
				INFRA_RST0_REG_PD, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	CCCI_BOOTUP_LOG(md_id, TAG, "%s:clear reset\n", __func__);

	udelay(500);

	/* reset dpmaif hw: AO Domain */
	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST0_REG_AO, DPMAIF_AO_RST_MASK);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST0_REG_AO, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	CCCI_BOOTUP_LOG(md_id, TAG, "%s:clear reset\n", __func__);

	udelay(500);

	/* reset dpmaif clr */
	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST1_REG_AO, DPMAIF_AO_RST_MASK);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST1_REG_AO, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	CCCI_BOOTUP_LOG(md_id, TAG, "%s:done\n", __func__);

	udelay(500);

	//drv3_dpmaif_set_axi_out_gated();

	/* reset dpmaif clr */
	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST1_REG_PD, DPMAIF_PD_RST_MASK);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			INFRA_RST1_REG_PD, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	CCCI_DEBUG_LOG(md_id, TAG, "%s:done\n", __func__);
}

static void dpmaif_hw_reset_v1(void)
{
	unsigned char md_id = 0;
	unsigned int value;
	int ret;

	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	value &= ~(1<<15);

	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x0208, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read infra_ao_base ret=%d\n",
		__func__, __LINE__, ret);
	udelay(500);

	/* DPMAIF HW reset */
	CCCI_DEBUG_LOG(md_id, TAG, "%s:rst dpmaif\n", __func__);
	/* reset dpmaif hw: PD Domain */
	dpmaif_write32(dpmaif_ctrl->dpmaif_reset_pd_base, 0xF50, 1<<22);

	value = dpmaif_read32(dpmaif_ctrl->dpmaif_reset_pd_base, 0xF50);
	CCCI_NORMAL_LOG(0, TAG, "[%s]-%d read 0xF50 value=%d\n",
			__func__, __LINE__, value);

	udelay(500);

	/* reset dpmaif hw: AO Domain */
	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			0x130, 1<<0);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
			"[%s]-%d write 0x130 ret=%d\n",
			__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x130, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read 0x130 ret=%d\n",
		__func__, __LINE__, ret);

	udelay(500);

	/* reset dpmaif clr */
	ret = regmap_write(dpmaif_ctrl->plat_val.infra_ao_base,
			0x134, 1<<0);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d write 0x134 ret=%d\n",
		__func__, __LINE__, ret);
	ret = regmap_read(dpmaif_ctrl->plat_val.infra_ao_base,
			0x134, &value);
	if (ret)
		CCCI_ERROR_LOG(0, TAG,
		"[%s]-%d read 0x134 ret=%d\n",
		__func__, __LINE__, ret);
	CCCI_BOOTUP_LOG(md_id, TAG, "%s:done\n", __func__);

	udelay(500);

	/* reset dpmaif clr */
	dpmaif_write32(dpmaif_ctrl->dpmaif_reset_pd_base, 0xF54, 1<<22);

	value = dpmaif_read32(dpmaif_ctrl->dpmaif_reset_pd_base, 0xF54);
	CCCI_NORMAL_LOG(0, TAG, "[%s]-%d read 0xF54 value=%d\n",
			__func__, __LINE__, value);
}

static int dpmaif_stop(unsigned char hif_id)
{
	if (dpmaif_ctrl->dpmaif_state == HIFDPMAIF_STATE_PWROFF
		|| dpmaif_ctrl->dpmaif_state == HIFDPMAIF_STATE_MIN)
		return 0;
#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "dpmaif:stop\n");
#else
	CCCI_DEBUG_LOG(-1, TAG, "dpmaif:stop\n");
#endif
	dpmaif_disable_irq(dpmaif_ctrl);

	dpmaif_ctrl->dpmaif_state = HIFDPMAIF_STATE_PWROFF;
	/* 1. stop HW */
	dpmaif_stop_hw();
	/* 2. stop sw */
	/* tx tx clear */
	dpmaif_stop_tx_sw(hif_id);
	/* rx rx clear */
	dpmaif_stop_rx_sw(hif_id);
	/* stop debug mechnism */
	del_timer(&dpmaif_ctrl->traffic_monitor);

	/* 3. todo: reset IP */
	/* CG set */
	ccci_dpmaif_set_clk(0, g_clk_tbs);
	if (g_plt_ops.hw_reset)
		g_plt_ops.hw_reset();

#ifdef DPMAIF_DEBUG_LOG
	CCCI_HISTORY_LOG(-1, TAG, "dpmaif:stop end\n");
#endif
	return 0;
}

/* =======================================================
 *
 * Descriptions: State part start(4/3): Misc -- 4.
 *
 * ========================================================
 */
static int dpmaif_stop_queue(unsigned char hif_id, unsigned char qno,
		enum DIRECTION dir)
{
	return 0;
}

static int dpmaif_start_queue(unsigned char hif_id, unsigned char qno,
		enum DIRECTION dir)
{
	return 0;
}

/* =======================================================
 *
 * Descriptions: State part start(5/3): Resume -- 5.
 *
 * ========================================================
 */
static int dpmaif_resume(unsigned char hif_id)
{
	/*IP don't power down before*/
	if (drv3_dpmaif_check_power_down() == false) {
		CCCI_DEBUG_LOG(0, TAG, "sys_resume no need restore\n");
	}

	return 0;
}

static void dpmaif_sysresume(void)
{
	if (dpmaif_ctrl->dpmaif_state != HIFDPMAIF_STATE_PWRON &&
		dpmaif_ctrl->dpmaif_state != HIFDPMAIF_STATE_EXCEPTION)
		return;

	dpmaif_resume(dpmaif_ctrl->hif_id);
}
/* =======================================================
 *
 * Descriptions: State part start(6/6): Suspend -- 6.
 *
 * ========================================================
 */
static int dpmaif_suspend(unsigned char hif_id __maybe_unused)
{
	if (dpmaif_ctrl->dpmaif_state != HIFDPMAIF_STATE_PWRON &&
		dpmaif_ctrl->dpmaif_state != HIFDPMAIF_STATE_EXCEPTION)
		return 0;
	/* dpmaif clock on: backup int mask. */
	dpmaif_ctrl->rxq[0].reg_int_mask_bak =
		drv3_dpmaif_get_dl_interrupt_mask();
	return 0;
}

static int dpmaif_syssuspend(void)
{
	return dpmaif_suspend(dpmaif_ctrl->hif_id);
}

static int dpmaif_debug(unsigned char hif_id,
		enum ccci_hif_debug_flg flag, int *para)
{
	int ret = -1;

	switch (flag) {
	case CCCI_HIF_DEBUG_SET_WAKEUP:
		atomic_set(&dpmaif_ctrl->wakeup_src, para[0]);
		ret = para[0];
		break;
	default:
		break;
	}
	return ret;
}

static int dpmaif_pre_stop(unsigned char hif_id)
{
	if (hif_id != DPMAIF_HIF_ID)
		return -1;

	dpmaif_stop_hw();

	return 0;
}

static struct ccci_hif_ops ccci_hif_dpmaif_ops = {
	.send_skb = &dpmaif_tx_send_skb,
	.give_more = &dpmaif_give_more,
	.write_room = &dpmaif_write_room,
	.stop_queue = &dpmaif_stop_queue,
	.start_queue = &dpmaif_start_queue,
	.dump_status = &dpmaif_dump_status,
	/* .suspend = &dpmaif_suspend, */
	/* .resume = &dpmaif_resume, */

	/* .init = &ccci_dpmaif_hif_init, */
	.start = &dpmaif_start,
	.pre_stop = &dpmaif_pre_stop,
	.stop = &dpmaif_stop,
	.debug = &dpmaif_debug,
	.empty_query = dpmaif_empty_query_v3,
};

static void dpmaif_total_spd_cb(u64 total_speed)
{
	ccmni_set_cur_speed(total_speed);

	if (total_speed < MAX_SPEED_THRESHOLD)
		dpmaif_ctrl->enable_pit_debug = 1;
	else
		dpmaif_ctrl->enable_pit_debug = 0;
}

static int dpmaif_init_cap(struct device *dev)
{
	unsigned int dpmaif_cap = 0;
	struct device_node *node;
	int ret;

	if (of_property_read_u32(dev->of_node,
			"mediatek,dpmaif_cap", &dpmaif_cap)) {
		dpmaif_cap = 0;
		CCCI_ERROR_LOG(-1, TAG,
			"[%s] read mediatek,dpmaif_cap fail!\n",
			__func__);
	}

	dpmaif_ctrl->support_lro = (dpmaif_cap & DPMAIF_CAP_LRO);
	if (dpmaif_cap & DPMAIF_CAP_PIT_DEG)
		dpmaif_ctrl->enable_pit_debug = 1;
	else
		dpmaif_ctrl->enable_pit_debug = -1;

	CCCI_NORMAL_LOG(-1, TAG,
		"[%s] dpmaif_cap: %x; support_lro: %u; pit_debug: %d\n",
		__func__, dpmaif_cap, dpmaif_ctrl->support_lro,
		dpmaif_ctrl->enable_pit_debug);

	if (dpmaif_ctrl->enable_pit_debug > -1)
		mtk_ccci_register_dl_speed_1s_callback(dpmaif_total_spd_cb);

	ret = of_property_read_u32(dev->of_node, "hw_reset_ver",
			&dpmaif_ctrl->hw_reset_ver);
	if (ret < 0)
		dpmaif_ctrl->hw_reset_ver = 0;

	if (dpmaif_ctrl->hw_reset_ver == 1) {
		node = of_find_compatible_node(NULL, NULL, "mediatek,infracfg");
		if (!node) {
			CCCI_NORMAL_LOG(-1, TAG,
				"[%s] error: infracfg node is not exist.\n",
				__func__);
			return -1;
		}
		dpmaif_ctrl->dpmaif_reset_pd_base = of_iomap(node, 0);
	}

	CCCI_NORMAL_LOG(-1, TAG,
		"[%s] hw_reset_ver: %d; dpmaif_reset_pd_base: %p\n",
		__func__, dpmaif_ctrl->hw_reset_ver,
		dpmaif_ctrl->dpmaif_reset_pd_base);

	return 0;
}

/* =======================================================
 *
 * Descriptions: State Module part End
 *
 * ========================================================
 */
static u64 dpmaif_dmamask = DMA_BIT_MASK(36);
static int ccci_dpmaif_hif_init(struct device *dev)
{
	struct device_node *node = NULL;
	struct device_node *node_md = NULL;
	struct hif_dpmaif_ctrl *hif_ctrl = NULL;
	int ret = 0;
	unsigned char md_id = 0;

	CCCI_HISTORY_TAG_LOG(-1, TAG,
			"%s: probe initl\n", __func__);
	/* get Hif hw information: register etc. */
	if (!dev) {
		CCCI_ERROR_LOG(-1, TAG, "No dpmaif driver in dtsi\n");
		ret = -3;
		goto DPMAIF_INIT_FAIL;
	}
	node = dev->of_node;
	if (!node) {
		CCCI_ERROR_LOG(-1, TAG, "No dpmaif driver in dtsi\n");
		ret = -2;
		goto DPMAIF_INIT_FAIL;
	}
	/* init local struct pointer */
	hif_ctrl = kzalloc(sizeof(struct hif_dpmaif_ctrl), GFP_KERNEL);
	if (!hif_ctrl) {
		CCCI_ERROR_LOG(-1, TAG,
			"%s:alloc hif_ctrl fail\n", __func__);
		ret = -1;
		goto DPMAIF_INIT_FAIL;
	}
	memset(hif_ctrl, 0, sizeof(struct hif_dpmaif_ctrl));

	hif_ctrl->md_id = md_id; /* maybe can get from dtsi or phase-out. */
	hif_ctrl->hif_id = DPMAIF_HIF_ID;
	hif_ctrl->bat_req = NULL;
	hif_ctrl->bat_frag = NULL;

	dpmaif_ctrl = hif_ctrl;
	atomic_set(&dpmaif_ctrl->suspend_flag, -1);

	node_md = of_find_compatible_node(NULL, NULL,
		"mediatek,mddriver");
	of_property_read_u32(node_md,
			"mediatek,md_generation",
			&dpmaif_ctrl->plat_val.md_gen);
	dpmaif_ctrl->plat_val.infra_ao_base =
		syscon_regmap_lookup_by_phandle(node_md,
		"ccci-infracfg");

	ret = ccci_dpmaif_init_clk(dev, g_clk_tbs);
	if (ret < 0)
		goto DPMAIF_INIT_FAIL;

	if (IS_ERR(dpmaif_ctrl->plat_val.infra_ao_base)) {
		CCCI_ERROR_LOG(-1, TAG, "No infra_ao register in dtsi\n");
		ret = -4;
		goto DPMAIF_INIT_FAIL;
	}

	hif_ctrl->dpmaif_ao_ul_base = of_iomap(node, 0);
	if (hif_ctrl->dpmaif_ao_ul_base == 0) {
		CCCI_ERROR_LOG(md_id, TAG,
			"ap_dpmaif error: ao_base=0x%p, iomap fail\n",
			(void *)hif_ctrl->dpmaif_ao_ul_base);
		ret = -1;
		goto DPMAIF_INIT_FAIL;
	}
	hif_ctrl->dpmaif_ao_dl_base = hif_ctrl->dpmaif_ao_ul_base + 0x400;
	hif_ctrl->dpmaif_ao_md_dl_base = hif_ctrl->dpmaif_ao_ul_base + 0x800;

	hif_ctrl->dpmaif_pd_ul_base = of_iomap(node, 1);
	if (hif_ctrl->dpmaif_pd_ul_base == 0) {
		CCCI_ERROR_LOG(md_id, TAG,
			"ap_dpmaif error: pd_base=0x%p, iomap fail\n",
			(void *)hif_ctrl->dpmaif_pd_ul_base);
		ret = -1;
		goto DPMAIF_INIT_FAIL;
	}
	hif_ctrl->dpmaif_pd_dl_base = hif_ctrl->dpmaif_pd_ul_base + 0x100;
	hif_ctrl->dpmaif_pd_rdma_base = hif_ctrl->dpmaif_pd_ul_base + 0x200;
	hif_ctrl->dpmaif_pd_wdma_base = hif_ctrl->dpmaif_pd_ul_base + 0x300;
	hif_ctrl->dpmaif_pd_misc_base = hif_ctrl->dpmaif_pd_ul_base + 0x400;

	hif_ctrl->dpmaif_ao_dl_sram_base =
		hif_ctrl->dpmaif_pd_ul_base + 0xC00;
	hif_ctrl->dpmaif_ao_ul_sram_base =
		hif_ctrl->dpmaif_pd_ul_base + 0xD00;
	hif_ctrl->dpmaif_ao_msic_sram_base =
		hif_ctrl->dpmaif_pd_ul_base + 0xE00;

	hif_ctrl->dpmaif_pd_md_misc_base = of_iomap(node, 2);
	if (hif_ctrl->dpmaif_pd_md_misc_base == 0) {
		CCCI_ERROR_LOG(md_id, TAG,
			"ap_dpmaif error: md_misc_base=0x%p, iomap fail\n",
			(void *)hif_ctrl->dpmaif_pd_md_misc_base);
		ret = -1;
		goto DPMAIF_INIT_FAIL;
	}
	hif_ctrl->dpmaif_pd_sram_base = of_iomap(node, 3);
	if (hif_ctrl->dpmaif_pd_sram_base == 0) {
		CCCI_ERROR_LOG(md_id, TAG,
			"ap_dpmaif error: md_sram_base=0x%p, iomap fail\n",
			(void *)hif_ctrl->dpmaif_pd_sram_base);
		ret = -1;
		goto DPMAIF_INIT_FAIL;
	}
	CCCI_DEBUG_LOG(md_id, TAG,
		     "ap_dpmaif register: ao_base=0x%p, pdn_base=0x%p\n",
		(void *)hif_ctrl->dpmaif_ao_ul_base,
		(void *)hif_ctrl->dpmaif_pd_ul_base);
	hif_ctrl->dpmaif_irq_id = irq_of_parse_and_map(node, 0);
	if (hif_ctrl->dpmaif_irq_id == 0) {
		CCCI_ERROR_LOG(md_id, TAG, "dpmaif_irq_id error:%d\n",
			hif_ctrl->dpmaif_irq_id);
		ret = -1;
		goto DPMAIF_INIT_FAIL;
	}
	hif_ctrl->dpmaif_irq_flags = IRQF_TRIGGER_NONE;
	CCCI_DEBUG_LOG(md_id, TAG, "dpmaif_irq_id:%d\n",
			hif_ctrl->dpmaif_irq_id);

	mtk_ccci_net_speed_init();
	ret = dpmaif_init_cap(dev);
	if (ret)
		goto DPMAIF_INIT_FAIL;

	ret = ccci_dl_pool_init(DPMAIF_RXQ_NUM);
	if (ret)
		goto DPMAIF_INIT_FAIL;

	dev->dma_mask = &dpmaif_dmamask;
	dev->coherent_dma_mask = dpmaif_dmamask;
	/* hook up to device */
	dev->platform_data = hif_ctrl; /* maybe no need */

	/* other ops: tx, dump */
	hif_ctrl->ops = &ccci_hif_dpmaif_ops;

	/* set debug related */
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	timer_setup(&hif_ctrl->traffic_monitor, dpmaif_traffic_monitor_func, 0);
#endif
	ccci_hif_register(DPMAIF_HIF_ID, (void *)dpmaif_ctrl,
		&ccci_hif_dpmaif_ops);

	atomic_set(&dpmaif_ctrl->suspend_flag, 0);

	mtk_ccci_md_spd_qos_init(dev);
	mtk_ccci_spd_qos_method_init();

	return 0;

DPMAIF_INIT_FAIL:
	kfree(hif_ctrl);
	dpmaif_ctrl = NULL;

	return ret;
}

static void dpmaif_init_plat_ops(void)
{
	if (dpmaif_ctrl->hw_reset_ver == 1)
		g_plt_ops.hw_reset = dpmaif_hw_reset_v1;
	else
		g_plt_ops.hw_reset = dpmaif_hw_reset;
}

int ccci_dpmaif_suspend_noirq_v3(struct device *dev)
{
	if ((!dpmaif_ctrl) || (atomic_read(&dpmaif_ctrl->suspend_flag) < 0))
		return 0;

	CCCI_NORMAL_LOG(-1, TAG, "[%s]\n", __func__);

	atomic_set(&dpmaif_ctrl->suspend_flag, 1);

	dpmaif_syssuspend();

	return 0;
}

int ccci_dpmaif_resume_noirq_v3(struct device *dev)
{
	struct arm_smccc_res res;

	if ((!dpmaif_ctrl) || (atomic_read(&dpmaif_ctrl->suspend_flag) < 0))
		return 0;

	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL,
			MD_CLOCK_REQUEST, MD_WAKEUP_AP_SRC,
			WAKE_SRC_HIF_DPMAIF, 0, 0, 0, 0, &res);

	CCCI_NORMAL_LOG(-1, TAG,
		"[%s] flag_1=0x%llx, flag_2=0x%llx, flag_3=0x%llx, flag_4=0x%llx\n",
		__func__, res.a0, res.a1, res.a2, res.a3);

	if ((!res.a0) && (res.a1 == WAKE_SRC_HIF_DPMAIF))
		arch_atomic_set(&dpmaif_ctrl->wakeup_src, 1);

	dpmaif_sysresume();

	atomic_set(&dpmaif_ctrl->suspend_flag, 0);

	return 0;
}

int ccci_dpmaif_hif_init_v3(struct platform_device *pdev)
{
	int ret;

	ret = ccci_dpmaif_hif_init(&pdev->dev);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG, "ccci dpmaif init fail");
		return ret;
	}

	dpmaif_init_plat_ops();

	dpmaif_ctrl->plat_dev = pdev;

	return 0;
}
